/*
 * Adreno 610 v2 (A6xx, chip id 0x06010001) bring-up on sapphire.
 *
 * The phone has no GMU: Linux drives this GPU through a "GMU wrapper", so the host does the
 * power-up itself. The sequence, the register values and the traps below are topaz-woa's
 * TopazGpu (docs/P8_gpu.md, G1 done on the same SoC under Windows), redone here on XNU with
 * the pieces TopazModem already brings to this kernel: SCM calls into TrustZone, an identity
 * SMMU context bank, and RAM below 4 GB that the kernel does not manage.
 *
 * Staged by boot-arg gpu=N, because reading a GPU register while its power domain or its
 * clocks are off hangs the bus (the same trap as the WLAN copy engines before mission mode):
 *
 * gpu_slow=N waits N ms after each step so the persistent log catches up with it,
 * gpu_delay=N waits N seconds before starting so the live console over USB is already up,
 * gpu_div=N runs the core at GPLL0/N (2 = 300 MHz, 8 = 75 MHz), and gpu_smmu_early=1 puts the
 * identity context bank in before the GX domain. Those two default to what has run before:
 * a boot that dies with every clock up and nothing else to show for it has to be bisected one
 * variable at a time. gpu_irq=1 is the exception, because it only turns off something this
 * driver has no business leaving on: the GPU's interrupt, masked by default in the GIC before
 * anything is powered and at the GPU itself as soon as its registers answer.
 *
 *   gpu=1   clock tree only; never touches 0x5900000
 *   gpu=2   + power up, then the first GPU reads (RBBM_STATUS, CP_ALWAYS_ON_COUNTER)
 *   gpu=3   + zap shader through TrustZone, identity SMMU, SQE microcode, ring buffer,
 *             CP_ME_INIT, CP_SET_SECURE_MODE and a CP_MEM_WRITE the CPU reads back
 *   gpu=4   the same without the zap shader, leaving secure mode by writing
 *             RBBM_SECVID_TRUST_CNTL as Linux does when a device has no zap firmware
 *
 * Both need the microcode (a630_sqe.fw, which is in linux-firmware); gpu=3 also needs this
 * phone's signed zap shader. Neither is in this repository. The driver takes them from the
 * device tree when the boot image carries them (tools/mkadt.py --gpu-sqe / --gpu-zap-mdt /
 * --gpu-zap-seg), and otherwise from a stock partition, read only through
 * sweet_ufs_read_part, with gpu_fw[] below filled in by tools/fwtable-file.py.
 *
 * gpu=4 is the shortcut for a first test with only the open microcode. It is a gamble Linux
 * warns about: if the bootloader left the GPU in secure mode, writing that register is
 * refused and a permissions violation follows, so prefer gpu=3 once the zap is in the table.
 */
#include <pexpert/arm64/board_config.h>
#if defined(ARM64_BOARD_CONFIG_SWEET)

#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IOUserClient.h>
#include <IOKit/IOKitKeys.h>
#include <IOKit/IOPlatformExpert.h>               /* IOService::getPlatform() (the msm service) */
#include <kern/thread.h>
#include <kern/clock.h>
#include <pexpert/pexpert.h>
#include <libkern/crc.h>
#include <sys/param.h>
#include <sys/errno.h>
#include <sys/systm.h>                         /* copyin, copyout (the msm client) */
#include <pexpert/device_tree.h>

#define LOG(fmt, ...)       IOLog("SapphireGpu: " fmt "\n", ##__VA_ARGS__)

extern "C" void sapphire_gpu_init(void);
extern "C" int sweet_ufs_read_part(const char *want, uint64_t off, void *dst, uint64_t len);

/* ---------------------------------------------------------------- the hardware */

#define GCC_PA              0x01400000ull
#define GCC_SIZE            0x00080000u
#define GPUCC_PA            0x05990000ull
#define GPUCC_SIZE          0x00009000u
#define GPU_PA              0x05900000ull
#define GPU_SIZE            0x00040000u
#define SMMU_PA             0x059A0000ull       /* kgsl-smmu, SMMU-500 */
#define SMMU_SIZE           0x00010000u

/*
 * The GIC-500 distributor, and the GPU's interrupt in it. The device tree gives kgsl
 * "interrupts = <0 0xb1 4>": SPI 177, level high, so INTID 177 + 32 = 209, which lives in
 * word 6 (209 / 32) of each GICD bitmap, bit 17. Nothing in this kernel handles it.
 */
#define GICD_PA             0x0F200000ull
#define GICD_SIZE           0x00010000u
#define GICD_ISENABLER      0x0100u
#define GICD_ICENABLER      0x0180u
#define GICD_ISPENDR        0x0200u
#define GICD_ICPENDR        0x0280u
#define GICD_ISACTIVER      0x0300u
#define GPU_INTID           (177u + 32u)
#define GPU_INT_WORD        (4u * (GPU_INTID / 32u))
#define GPU_INT_BIT         (1u << (GPU_INTID % 32u))

/* DT gpu_region@55A15000 (topaz has the same region at 0x55B15000) */
#define ZAP_PA              0x55A15000ull
#define ZAP_SIZE            0x00002000u
#define GPU_PAS_ID          13

/*
 * Buffers the GPU reads by physical address: RAM the kernel does not manage, below
 * 0xF0000000 for TrustZone and the hardware, right after the modem's pool
 * (MODEM_POOL_BASE 0xE0000000 + 16 MB). A610 has ADRENO_QUIRK_4GB_VA, so everything the
 * GPU touches must stay below 4 GB anyway.
 */
#define POOL_PA             0xE1000000ull
#define POOL_SIZE           0x00100000u
#define OFF_SQE             0x00000u            /* microcode, up to 64 KB */
#define OFF_RING            0x10000u            /* 32 KB, 64 KB aligned */
#define OFF_TEST            0x18000u            /* what CP_MEM_WRITE writes */
#define OFF_META            0x19000u            /* zap metadata, read by TrustZone */
#define OFF_FENCE           0x18100u            /* where the CCU/cache flush events write their seqno */
#define OFF_MSM_FENCE       0x18200u            /* 0153: the msm client's own fence, written by nothing else */
#define OFF_DRAW            0x40000u            /* what the 2D blitter fills: 256 x 256 RGBA8, 256 KB */
#define OFF_PT              0x80000u            /* GPU page tables: 128 pages of 4 KB */
#define OFF_CS              0x30000u            /* compute shader binary */
#define OFF_CSOUT           0x31000u            /* what it writes */
#define OFF_DIAG            0x20000u            /* 0336: the GPU's time marks, 2 x 64 pairs of 64-bit counters (2 KB) */
#define PT_PAGES            128u
#define DRAW_W              256u
#define DRAW_H              256u
#define DRAW_PITCH          (DRAW_W * 4u)       /* bytes; the blitter wants a multiple of 64 */
#define RING_BYTES          0x8000u
#define RING_DWORDS         (RING_BYTES / 4)

#define CBCR_EN             (1u << 0)
#define CBCR_OFF            (1u << 31)
#define GDSC_COLLAPSE       (1u << 0)
#define GDSC_PWR_ON         (1u << 31)
#define RCG_UPDATE          (1u << 0)
#define RCG_ROOT_OFF        (1u << 31)
#define SRC_GPLL0           5                   /* GPLL0_OUT_MAIN in the GPU CC parent maps */

/* A6xx registers, dword offsets (registers/adreno/a6xx.xml) */
#define REG_RBBM_GBIF_HALT      0x0016
#define REG_RBBM_GBIF_HALT_ACK  0x0017
#define REG_RBBM_INT_CLEAR_CMD  0x0037
#define REG_GBIF_HALT           0x3c45
#define REG_GBIF_HALT_ACK       0x3c46
#define GBIF_CLIENT_HALT        (1u << 0)
#define GBIF_ARB_HALT           (1u << 1)
#define REG_RBBM_INT_0_MASK     0x0038
#define REG_RBBM_INT_0_STATUS   0x0201
#define REG_RBBM_STATUS         0x0210
#define REG_RBBM_SW_RESET_CMD   0x0043
#define REG_CP_RB_BASE          0x0800
#define REG_CP_RB_CNTL          0x0802
#define REG_CP_RB_RPTR          0x0806
#define REG_CP_RB_WPTR          0x0807
#define REG_CP_SQE_CNTL         0x0808
#define REG_CP_HW_FAULT         0x0821
#define REG_CP_SQE_INSTR_BASE   0x0830
#define REG_CP_ALWAYS_ON_COUNTER 0x0980
#define REG_RBBM_SECVID_TRUST_CNTL 0xF400

#define CP_ME_INIT              0x48
#define CP_MEM_WRITE            0x3d
#define CP_SET_SECURE_MODE      0x66
#define CP_WAIT_FOR_IDLE        0x26
#define CP_EVENT_WRITE          0x46
#define CP_SET_MARKER           0x65
#define CP_BLIT                 0x2c
#define CP_REG_TO_MEM           0x3e                /* 0336: REG | CNT << 18 | 64B << 30, then the address (adreno_pm4.xml) */

/*
 * The 2D blitter, as Mesa's freedreno does a buffer clear on a6xx gen1 (fd6_blitter.cc
 * fd6_clear_buffer, emit_setup / emit_blit_setup / emit_blit_fini; offsets and fields from the
 * same tree's registers/adreno/a6xx.xml, a610's values from common/freedreno_devices.py).
 */
#define REG_RB_A2D_BLT_CNTL         0x8c00
#define REG_RB_A2D_PIXEL_CNTL       0x8c01
#define REG_RB_A2D_DEST_BUFFER_INFO 0x8c17      /* + BASE (64-bit) at 0x8c18, PITCH at 0x8c1a */
#define REG_RB_A2D_CLEAR_COLOR_DW0  0x8c2c      /* DW0..DW3 */
#define REG_TPL1_A2D_SRC_TEXTURE_INFO 0xb4c0    /* + SIZE 0xb4c1, BASE (64-bit) 0xb4c2, PITCH 0xb4c4 */
#define REG_GRAS_A2D_SRC_XMIN       0x8401      /* + XMAX, YMIN, YMAX: coordinate << 8 */
#define REG_GRAS_A2D_BLT_CNTL       0x8400
#define REG_GRAS_A2D_DEST_TL        0x8405      /* + BR at 0x8406 */
#define REG_SP_A2D_OUTPUT_INFO      0xacc0
#define REG_RB_CCU_CNTL             0x8e07
#define REG_RB_DBG_ECO_CNTL         0x8e04
#define REG_RB_RBP_CNTL             0x8e01
#define FMT6_8_8_8_8_UNORM          0x30
#define RM6_BLIT2DSCALE             12
#define BLIT_OP_SCALE               3
#define EV_PC_CCU_FLUSH_COLOR_TS    0x1d
#define EV_PC_CCU_FLUSH_DEPTH_TS    0x1c
#define EV_PC_CCU_INVALIDATE_COLOR  0x19
#define EV_PC_CCU_INVALIDATE_DEPTH  0x18
#define EV_CACHE_FLUSH_TS           0x04
#define EV_DEBUG_LABEL              0x3f

/* TrustZone (same calls as TopazModem's PAS boot, with the GPU's PAS id) */
#define SCM_FN(svc, cmd)    (0x42000000u | ((uint32_t)(svc) << 8) | (uint32_t)(cmd))
#define SCM_SVC_BOOT        0x01
#define SCM_BOOT_SET_REMOTE_STATE 0x0a
#define SCM_SVC_PIL         0x02
#define PIL_INIT_IMAGE      0x01
#define PIL_MEM_SETUP       0x02
#define PIL_AUTH_RESET      0x05
#define PIL_SHUTDOWN        0x06
#define SCM_ARG_RW          2

/* ---------------------------------------------------------------- firmware table */

/*
 * Where the GPU firmware lies in a stock partition of this phone, found on the host by
 * tools/fwtable-vendor.py and checked here by CRC32. Qualcomm's firmware is never copied
 * into this repository or into the iOS root; the driver reads it from the partition it
 * already sits in, read only.
 */
struct gpu_fw_file {
	const char *name;
	const char *part;
	uint32_t    off, size, crc;
};
static const struct gpu_fw_file gpu_fw[] = {
	/*
	 * Entries generated by tools/fwtable-vendor.py, for example
	 *   { "a630_sqe.fw", "vendor_a", 0x1234000, 32504, 0xdeadbeef },
	 *   { "a610_zap.mdt", "vendor_a", 0x1240000, 6860, 0xdeadbeef },
	 *   { "a610_zap.b02", "vendor_a", 0x1242000, 2096, 0xdeadbeef },
	 * The sentinel keeps the array valid while the table is empty.
	 */
	{ NULL, NULL, 0, 0, 0 },
};

/* Does the boot image carry the microcode in its device tree? */
static bool
have_adt_firmware(void)
{
	DTEntry chosen;
	void const *blob;
	unsigned int size;

	return SecureDTLookupEntry(NULL, "/chosen", &chosen) == kSuccess &&
	       SecureDTGetProperty(chosen, "gpu-sqe-firmware", &blob, &size) == kSuccess && size > 4;
}

static unsigned
fw_count(void)
{
	unsigned n = 0;
	while (n < sizeof(gpu_fw) / sizeof(gpu_fw[0]) && gpu_fw[n].name != NULL) {
		n++;
	}
	return n;
}

/* ---------------------------------------------------------------- register access */

static IOMemoryMap *gcc_map, *gpucc_map, *gpu_map, *smmu_map, *pool_map, *zap_map;
static volatile uint8_t *gcc, *gpucc, *gpu, *smmu;
static uint8_t *pool;
static uint32_t *ring, *test_buf;
static uint32_t ring_wptr;
static unsigned scm_resumes;
static uint32_t slow_ms;                        /* boot-arg gpu_slow: see step_done() */

/*
 * sapphire 0336: diagnostics, read with sysctl kern.sweet_gpu_diag (at the end of this file): counters and times of
 * what the msm client and the panel's copies cost here. Nothing in them changes what the driver does. The one
 * exception is opt-in and off by default: kern.sweet_gpu_marks=1 puts every msm submission and every panel copy
 * between two CP_REG_TO_MEM of CP_ALWAYS_ON_COUNTER (the GPU's 19.2 MHz counter), the way Linux's a6xx_submit takes
 * its per-submission stats (a6xx_gpu.c, get_stats_counter), and the differences are the GPU's own time for each one:
 * 8 more dwords in the ring and two 64-bit words the CP writes into the pool at OFF_DIAG.
 */
struct gdiag_time {
	uint64_t n, ns, max;
};
#define GDIAG_SLOTS         64u
#define GDIAG_HIST          10u
static struct {
	/* GEM_SUBMIT */
	uint64_t sub, sub_cmds, sub_bos, sub_behind, sub_ring_max, ring_waits, sub_flush_bytes;
	struct gdiag_time sub_all, sub_lock, sub_hold, sub_flush;
	/* buffers: GEM_NEW by size (under 256 KB, under 4 MB, larger), GEM_CLOSE, IOSurfaces, the SMMU's TLB */
	uint64_t new_n[3], new_bytes[3], close_n, close_left, surf_new, surf_again, tlb_slept, poll_sleeps;
	struct gdiag_time new_zero, new_place, bo_free, tlb;
	/* waits for a fence (WAIT_FENCE, CPU_PREP, a close) */
	uint64_t wait_n, wait_done, wait_loops, prep_n, prep_flush_bytes;
	struct gdiag_time wait;
	/* the panel's copies */
	uint64_t copy_n, copy_behind, copy_gpu_busy;
	struct gdiag_time copy_lock;
	/* the GPU's own time, in ticks of 19.2 MHz (kern.sweet_gpu_marks) */
	uint64_t gt_sub_n, gt_sub_ticks, gt_sub_max, gt_sub_hist[GDIAG_HIST];
	uint64_t gt_copy_n, gt_copy_ticks, gt_copy_max, gt_copy_hist[GDIAG_HIST];
	uint64_t gt_lost;
} gdiag;
static int gpu_marks;                           /* kern.sweet_gpu_marks */
static uint32_t gdiag_sub_fence[GDIAG_SLOTS], gdiag_copy_fence[GDIAG_SLOTS];    /* the fence each pair of marks goes with */
static uint32_t gdiag_sub_head, gdiag_sub_tail, gdiag_copy_head, gdiag_copy_tail;
static uint64_t gdiag_end_pa;                   /* where blit_end() puts a copy's end mark (0: none) */

static void
gdiag_since(struct gdiag_time *t, uint64_t t0)
{
	uint64_t ns;

	absolutetime_to_nanoseconds(mach_absolute_time() - t0, &ns);
	OSAddAtomic64(1, (volatile SInt64 *)&t->n);
	OSAddAtomic64((SInt64)ns, (volatile SInt64 *)&t->ns);
	if (ns > t->max) {
		t->max = ns;
	}
}

/*
 * boot-arg gpu_div: the GPLL0 divider for the core clock, 2 = 300 MHz. Nothing here raises
 * VDD_GFX (Linux does it through the OPP table and RPMh, and there is no GMU to do it for
 * us), so the rail stays wherever the bootloader left it and 300 MHz may be above what it
 * holds: a boot that dies with the clocks up and no bus error is the shape that has. gpu_div=8
 * (75 MHz) is the slowest useful setting; the divider field holds 2 * div - 1.
 */
static uint32_t core_div = 2;
/*
 * boot-arg gpu_mhz: a stock OPP above 600 MHz (785 820 980 1025 1100 1260) from gpu_cc_pll0
 * instead of GPLL0 (ios-topaz 0922, sequence from topaz-woa TopazGpuW v0.43 and the stock
 * gpucc-khaje.c). The same caveat as gpu_div: VDD_GFX is not raised, so a rate above what the
 * bootloader's corner holds may hang the GPU. 0 = off (gpu_div decides).
 */
static uint32_t core_mhz;
extern "C" int topaz_rpm_vote(uint32_t set, uint32_t type, uint32_t id, uint32_t key, uint32_t value);
/*
 * 0337: boot-arg gpu_vco2=1 makes gpu_mhz the frequency of the core clock, made the way the stock
 * driver makes it. gx_gfx3d takes gpu_cc_pll0's OUT_MAIN with a divider of 1 (stock gpucc-khaje.c,
 * every row of ftbl_gpu_cc_gx_gfx3d_clk_src), and OUT_MAIN is the VCO divided by two: USER_CTL[9:8]
 * = 1 in the 0x01000101 that the stock driver and gpu_pll_start() both write
 * (post_div_table_gpu_cc_pll0_out_main { 0x1, 2 }; the stock "640MHz configuration" of the PLL is
 * its 320 MHz OPP). So the stock clock framework runs the VCO at twice the OPP, 2520 MHz for 1260,
 * while gpu_mhz=N alone (0161) programs the VCO to N and the core runs at N / 2: 630.00 MHz measured
 * with gpu_mhz=1260 (the CP's and the RBBM's always-count counters against the 19.2 MHz always-on
 * counter). With gpu_vco2=1:
 *   - gpu_mhz must be a row of the stock table (320 465 600 785 820 980 1025 1100 1114 1260; 1114
 *     is 1114.8) and not above what the stock device tree gives this phone's speed bin (the fuse
 *     at QFPROM 0x6006, bits 5 to 12: 0 goes to 1260, 235 to 1114.8, 216 to 1025). gpu_bin=N gives
 *     the bin instead of reading the fuse, gpu_anybin=1 lifts the limit;
 *   - before the clock is raised the rails go to what the stock driver votes for the rate: CX (RPM
 *     rwcx 0) by rate_max of gpu_cc_gx_gfx3d_clk_src (464 for 1260, 416 for 1100 and 1114.8, 384
 *     down to 820, 256 for 785), but never under the 0x1a0 that 0204 votes today, because this
 *     kernel has one voter where Android adds up every consumer of the rail; and MX (RPM rwmx 0),
 *     which gpu_cc_pll0 hangs from (256 for a VCO above 2400 MHz), to 0x1a0. gpu_cx=N and gpu_mx=N
 *     give the levels instead (0: no vote);
 *   - from 900 MHz on, the CX peak-current votes kgsl makes (cx_ipeak@3ed000, clients 1 and 10);
 *     refused, the rate is 820 MHz. gpu_ipeak=0: no votes;
 *   - the PLL is configured and enabled as clk_zonda_pll_configure() and clk_zonda_pll_enable() do
 *     it, with L and alpha as clk_zonda_pll_set_rate() computes them for twice the OPP.
 * A rate that is not in the table, a refused vote or a PLL that does not lock leave it all to the
 * old path (gpu_mhz as the VCO, then gpu_div). gpu_vco2=0, the default, changes nothing.
 * kern.sweet_gpu_pll reads the PLL and the RCG back at any time.
 */
static uint32_t vco2;
static uint32_t cx_arg = ~0u, mx_arg = ~0u;
static uint32_t cx_voted, mx_voted;             /* the levels this driver asked the RPM for (0: none) */
static uint32_t bin_fuse = ~0u;                 /* the speed bin (~0: not read) */
/*
 * sapphire 0343: what the clocks and the temperatures really are, read only.
 *   kern.sweet_gpu_clocks  measures, when it is read (about 0.2 s), the GPU's core clock, the GMU clock, the two bus
 *                          branches the GPU's memory traffic crosses, the DDR's clock and both CPU clusters' clocks;
 *   kern.sweet_ddr_khz     the DDR's clock in kHz, from the memory controller's period register (no GPU needed);
 *   kern.sweet_thermal     every TSENS sensor of the SoC, by the name of its thermal zone in the stock device tree, in
 *                          milli-degrees C, with the state of the controller;
 *   kern.sweet_gpu_temp    the GPU's sensor (TSENS 15, thermal zone "gpu"), in milli-degrees C;
 *   gpu_clkdiag=1          puts the clocks and the temperatures in the boot log after the power-up.
 * Nothing here changes what a block does: the clock measurement picks a clock on the debug muxes and runs their
 * counter, as the stock clk_measure does from debugfs, and leaves the debug branches as it found them; the rest are
 * register reads.
 */
static uint32_t clkdiag;
static bool clocks_up;                          /* power_up() succeeded: the GPU CC's clocks may be measured */
static IOLock *clk_lock;
/* sapphire 0345: the core clock the stock path of 0337 set (kHz; 0: none, so no change of rate is possible) */
static uint32_t opp_khz;
static bool opp_quiet;                          /* rate changes after boot: cx_ipeak and the PLL log only failures */
static IOLock *opp_lock;                        /* serialises changes of rate (taken before service_lock) */
static uint32_t opp_drain_ms = 100;             /* boot-arg gpu_opp_drain: how long a change waits for an idle GPU */
static int opp_park;                            /* kern.sweet_gpu_opp_park (boot-arg gpu_opp_park) */
/* sapphire 0346: the governor (below the msm client); its switch and the rate it gives back when it stops */
static int gov_on;                              /* kern.sweet_gpu_gov (boot-arg gpu_gov) */
static uint32_t gov_boot_khz;                   /* the rate the stock path of 0337 set at boot */
static void gov_start(void);
static int ddr_mode_set(int on);                /* 0347 */
/*
 * boot-arg gpu_smmu_early: program the identity context bank as soon as its clock is voted,
 * before the GX domain comes up, instead of in cp_start(). The GPU's SMMU comes out of the
 * bootloader with every stream faulting, so anything the GPU does on the bus between the GX
 * domain coming up and cp_start() is a fault with no context bank behind it, and a NoC error
 * on this SoC resets it without the CPU seeing anything. Off by default: it changes the order
 * of the one sequence that has worked.
 */
static uint32_t smmu_early;
/*
 * boot-arg gpu_irq=1 leaves the GPU's interrupt alone; by default it is masked in the GIC
 * before anything is powered. Three boots have now died in the same place: the step after
 * the power-up, which only re-reads registers that were just read, with the phone stopping
 * dead mid-sentence and iOS with it, no panic and no further line. That is not what a bad
 * register read looks like (that hangs the reader), it is what an interrupt nobody handles
 * looks like: the GPU's is level triggered, so if it is asserted and enabled in the
 * distributor the CPU re-takes it forever and nothing ever logs again. Whether it is enabled
 * at all is the bootloader's business, so the state goes in the log either way.
 */
static uint32_t gpu_irq;
/*
 * boot-arg gpu_stop=N ends stage 3 early: 1 after the zap shader, 2 once the SQE runs, 3 after
 * CP_ME_INIT, 4 after CP_SET_SECURE_MODE; 0 (default) runs it all. boot-arg gpu_keep=1 leaves
 * the GPU running afterwards; by default it is parked (bus drained, GX and CX collapsed), as
 * Linux does once a GPU goes idle. Both exist for the one stage-3 boot whose panel stopped
 * taking DSI commands right after the GPU ran ("panel level: NO DONE", the screen filled with
 * colours), when stage 2 and stage 4 did not: they say which step does it, and whether it
 * lasts only while the GPU is up.
 */
static uint32_t stop_at, keep_running;
static uint32_t draw_first = 1;                 /* boot-arg gpu_draw=0 skips the 2D fill */
static bool stopped_early;
static bool service_ok;                         /* stage 3 and the draw tests passed */
static bool recover_zap;                        /* 0317: the zap shader is up (gpu=3): a reset resumes it */
static volatile bool service_ready;              /* gpu_service: the GPU takes work from the rest of the kernel */

static bool
stop_here(uint32_t step, const char *what)
{
	if (stop_at != step) {
		return false;
	}
	LOG("gpu_stop=%u: stopping after %s", step, what);
	stopped_early = true;
	return true;
}
static IOMemoryMap *gicd_map;
static volatile uint8_t *gicd;
static bool smmu_identity(void);
static void msm_publish(void);

/*
 * The persistent log is written to userdata about once a second, so a step that hangs or
 * panics the phone takes the lines around it with it: the last boot left nothing after
 * "starting in N s". gpu_slow=N waits N ms after each step, which gives that writer time to
 * put the step's line on disk before the next one runs. 1200 is a good value.
 *
 * Every step of the power-up, the zap shader and the CP ends in one of these, so a boot that
 * stops dead says in the log which register or which TrustZone call it stopped on.
 */
static void
step_done(void)
{
	if (slow_ms != 0) {
		IOSleep(slow_ms);
	}
}

static inline void
gmb(void)
{
	__asm__ volatile ("dsb sy" ::: "memory");
}

static uint32_t
rd(volatile uint8_t *b, uint32_t off)
{
	uint32_t v = *(volatile uint32_t *)(b + off);
	gmb();
	return v;
}

static void
wr(volatile uint8_t *b, uint32_t off, uint32_t v)
{
	gmb();
	*(volatile uint32_t *)(b + off) = v;
	gmb();
}

static void
rmw(volatile uint8_t *b, uint32_t off, uint32_t clear, uint32_t set)
{
	wr(b, off, (rd(b, off) & ~clear) | set);
}

/* Wait up to us microseconds for (reg & mask) == want. */
static bool
poll(volatile uint8_t *b, uint32_t off, uint32_t mask, uint32_t want, uint32_t us)
{
	for (uint32_t i = 0; i <= us; i++) {
		if ((rd(b, off) & mask) == want) {
			return true;
		}
		gdiag.poll_sleeps++;                    /* 0336: IODelay blocks the thread on this port */
		IODelay(1);
	}
	return false;
}

/* The GPU's own registers are addressed in dwords in Qualcomm's documentation. */
static uint32_t
grd(uint32_t reg)
{
	return rd(gpu, 4 * reg);
}

static void
gwr(uint32_t reg, uint32_t v)
{
	wr(gpu, 4 * reg, v);
}

static void
gwr64(uint32_t reg, uint64_t v)
{
	gwr(reg, (uint32_t)v);
	gwr(reg + 1, (uint32_t)(v >> 32));
}

static IOMemoryMap *
map_phys(uint64_t pa, uint64_t size, bool ram)
{
	IOMemoryDescriptor *md = IOMemoryDescriptor::withPhysicalAddress(pa, size, kIODirectionInOut);
	IOMemoryMap *m;

	if (md == NULL) {
		return NULL;
	}
	/*
	 * Registers are device memory (kIOMapInhibitCache). RAM the GPU shares is not: xnu's
	 * bzero and memcpy use dc zva and vector stores, which are alignment faults on device
	 * memory ("Unaligned kernel data abort" in bzero, from the first bzero of the pool,
	 * what stage 3 and 4 died of before). Write-combined is normal non-cacheable memory,
	 * which is what TopazModem and SweetSurface use for theirs, and the GPU sees every
	 * store after a dsb with no cache maintenance.
	 */
	m = md->map(ram ? kIOMapWriteCombineCache : kIOMapInhibitCache);
	md->release();
	return m;
}

/* ---------------------------------------------------------------- TrustZone */

static uint64_t
smc(uint64_t a[8])
{
	register uint64_t x0 __asm__("x0") = a[0], x1 __asm__("x1") = a[1], x2 __asm__("x2") = a[2],
	    x3 __asm__("x3") = a[3], x4 __asm__("x4") = a[4], x5 __asm__("x5") = a[5], x6 __asm__("x6") = a[6],
	    x7 __asm__("x7") = a[7];
	__asm__ volatile ("smc #0"
            : "+r"(x0), "+r"(x1), "+r"(x2), "+r"(x3), "+r"(x4), "+r"(x5), "+r"(x6), "+r"(x7)
            :
            : "x8", "x9", "x10", "x11", "x12", "x13", "x14", "x15", "x16", "x17", "memory");
	a[0] = x0; a[1] = x1; a[2] = x2; a[3] = x3; a[4] = x4; a[5] = x5; a[6] = x6; a[7] = x7;
	return x0;
}

static uint64_t
scm(uint32_t fn, uint64_t arginfo, uint64_t a, uint64_t b, uint64_t c, uint64_t *res1)
{
	uint64_t args[8] = { fn, arginfo, a, b, c, 0, 0, 0 };
	unsigned n;

	smc(args);
	/* long calls answer INTERRUPTED (1): resume with x0 = 1 and the session from x6 */
	for (n = 0; args[0] == 1 && n < 100000; n++) {
		uint64_t again[8] = { 1, arginfo, a, b, c, 0, args[6], 0 };
		memcpy(args, again, sizeof(args));
		smc(args);
	}
	scm_resumes = n;
	if (res1 != NULL) {
		*res1 = args[1];
	}
	return args[0];
}

/* ---------------------------------------------------------------- clocks and power */

struct reg_desc { uint8_t block, kind; uint32_t off; const char *name; };

static const struct reg_desc clock_regs[] = {
	{ 0, 1, 0x36004, "gcc_gpu_cfg_ahb" },
	{ 0, 1, 0x3600c, "gcc_gpu_memnoc_gfx" },
	{ 0, 1, 0x36018, "gcc_gpu_snoc_dvm_gfx" },
	{ 0, 1, 0x36100, "gcc_gpu_iref" },
	{ 0, 1, 0x36048, "gcc_gpu_throttle_core" },
	{ 0, 0, 0x79004, "gcc_vote (b15 gpll0)" },
	{ 1, 4, 0x0000,  "gpucc_pll0_mode" },
	{ 1, 0, 0x0004,  "gpucc_pll0_l" },
	{ 1, 0, 0x0008,  "gpucc_pll0_alpha" },
	{ 1, 0, 0x000c,  "gpucc_pll0_user_ctl" },
	{ 1, 4, 0x0100,  "gpucc_pll1_mode" },
	{ 1, 3, 0x101c,  "gpucc_gx_gfx3d_rcg_cmd" },
	{ 1, 0, 0x1020,  "gpucc_gx_gfx3d_rcg_cfg" },
	{ 1, 3, 0x1120,  "gpucc_gmu_rcg_cmd" },
	{ 1, 0, 0x1124,  "gpucc_gmu_rcg_cfg" },
	{ 1, 1, 0x1004,  "gpucc_cxo_aon" },
	{ 1, 1, 0x1054,  "gpucc_gx_gfx3d" },
	{ 1, 1, 0x1060,  "gpucc_gx_cxo" },
	{ 1, 1, 0x1078,  "gpucc_ahb" },
	{ 1, 1, 0x1098,  "gpucc_cx_gmu" },
	{ 1, 1, 0x109c,  "gpucc_cxo" },
	{ 1, 1, 0x10a4,  "gpucc_cx_gfx3d" },
	{ 1, 1, 0x5000,  "gpucc_hlos1_vote_gpu_smmu" },
	{ 1, 2, 0x100c,  "gpucc_gx_gdsc" },
	{ 1, 2, 0x106c,  "gpucc_cx_gdsc" },
	{ 1, 0, 0x1508,  "gpucc_gx_clamp_io" },
	{ 1, 0, 0x1540,  "gpucc_cx_gds_hw_ctrl" },
};

/* The GPU's line in the distributor: enabled, pending, active. */
static void
gic_report(const char *when)
{
	if (gicd == NULL) {
		return;
	}
	LOG("  GIC SPI 177 (INTID %u) %s: enabled %u pending %u active %u (words %08x %08x %08x)", GPU_INTID, when,
	    (rd(gicd, GICD_ISENABLER + GPU_INT_WORD) & GPU_INT_BIT) != 0,
	    (rd(gicd, GICD_ISPENDR + GPU_INT_WORD) & GPU_INT_BIT) != 0,
	    (rd(gicd, GICD_ISACTIVER + GPU_INT_WORD) & GPU_INT_BIT) != 0,
	    rd(gicd, GICD_ISENABLER + GPU_INT_WORD), rd(gicd, GICD_ISPENDR + GPU_INT_WORD),
	    rd(gicd, GICD_ISACTIVER + GPU_INT_WORD));
}

/*
 * Writing a 1 to ICENABLER/ICPENDR clears that interrupt's enable and pending bits and
 * leaves every other line alone, which is why these registers are a bitmap of ones to act on
 * rather than a value to store.
 */
static void
gic_mask_gpu(void)
{
	if (gicd == NULL) {
		return;
	}
	wr(gicd, GICD_ICENABLER + GPU_INT_WORD, GPU_INT_BIT);
	wr(gicd, GICD_ICPENDR + GPU_INT_WORD, GPU_INT_BIT);
	gmb();
	gic_report("masked");
}

static void
dump_clocks(const char *when)
{
	LOG("--- clock tree (%s)", when);
	for (unsigned i = 0; i < sizeof(clock_regs) / sizeof(clock_regs[0]); i++) {
		const struct reg_desc *r = &clock_regs[i];
		volatile uint8_t *b = r->block == 0 ? gcc : gpucc;
		uint32_t v = rd(b, r->off);
		const char *state = "";
		switch (r->kind) {
		case 1: state = (v & CBCR_OFF) ? " off" : " running"; break;
		case 2: state = (v & GDSC_PWR_ON) ? " PWR_ON" : " collapsed"; break;
		case 3: state = (v & RCG_ROOT_OFF) ? " root off" : (v & RCG_UPDATE) ? " update pending" : " root on"; break;
		case 4: state = (v & 1) ? " outctrl" : " unconfigured"; break;
		default: break;
		}
		LOG("  %s %08x%s", r->name, v, state);
	}
}

static bool
branch_on(volatile uint8_t *b, uint32_t off, const char *name)
{
	bool ok;

	rmw(b, off, 0, CBCR_EN);
	ok = poll(b, off, CBCR_OFF, 0, 200);
	LOG("  %s on: %08x %s", name, rd(b, off), ok ? "running" : "STILL OFF");
	step_done();
	return ok;
}

/*
 * A GDSC comes up by clearing SW_COLLAPSE and waiting for PWR_ON. The GX domain also needs
 * its block reset pulsed and its I/O clamp released first; its status is in its own GDSCR,
 * the CX domain's is in GPU_CC_CX_GDS_HW_CTRL (0x1540).
 */
static bool
gdsc_on(uint32_t gdscr, uint32_t bcr, uint32_t clamp, uint32_t status, const char *name)
{
	bool ok;

	if (bcr != 0) {
		rmw(gpucc, bcr, 0, 1);
		IODelay(1);
		rmw(gpucc, bcr, 1, 0);
	}
	if (clamp != 0) {
		rmw(gpucc, clamp, 1, 0);
	}
	rmw(gpucc, gdscr, GDSC_COLLAPSE, 0);
	IODelay(1);
	ok = poll(gpucc, status, GDSC_PWR_ON, GDSC_PWR_ON, 500);
	LOG("  %s on: gdscr %08x status %08x %s", name, rd(gpucc, gdscr), rd(gpucc, status),
	    ok ? "PWR_ON" : "NO PWR_ON");
	step_done();
	return ok;
}

/* Root clock generator: pick a parent and a divider (the register holds 2 * div - 1). */
/* ---- gpu_cc_pll0: Zonda PLL at GPU CC + 0, f = 19.2 MHz * (L + alpha / 2^16) ---- */
#define ZP_MODE         0x00
#define ZP_L            0x04
#define ZP_ALPHA        0x08
#define ZP_USER         0x0c
#define ZP_CFG          0x10
#define ZP_CFG_U        0x14
#define ZP_CFG_U1       0x18
#define ZP_OPMODE       0x28
#define ZP_OUTCTRL      (1u << 0)
#define ZP_BYPASSNL     (1u << 1)
#define ZP_RESET_N      (1u << 2)
#define ZP_LOCK         (1u << 31)
#define SRC_GPU_PLL0    1                   /* GPU_CC_PLL0_OUT_MAIN in gx_gfx3d's parent map */

static bool
zonda_la(uint32_t mhz, uint32_t *l, uint32_t *a)
{
	static const uint32_t t[][3] = {
		{ 785, 41, 0xE2AA }, { 820, 43, 0xB555 }, { 980, 51, 0x0AAB },
		{ 1025, 53, 0x62AA }, { 1100, 57, 0x4AAB }, { 1260, 66, 0xA000 },
	};
	for (unsigned i = 0; i < sizeof(t) / sizeof(t[0]); i++) {
		if (t[i][0] == mhz) {
			*l = t[i][1];
			*a = t[i][2];
			return true;
		}
	}
	return false;
}

static bool
gpu_pll_start(uint32_t mhz)
{
	uint32_t l, a;

	if (!zonda_la(mhz, &l, &a)) {
		LOG("gpu_mhz=%u is not a stock OPP above 600 (785 820 980 1025 1100 1260): GPLL0", mhz);
		return false;
	}
	rmw(gpucc, ZP_MODE, ZP_OUTCTRL, 0);
	wr(gpucc, ZP_OPMODE, 0);
	wr(gpucc, ZP_L, l);
	wr(gpucc, ZP_ALPHA, a);
	wr(gpucc, ZP_CFG, 0x08200800);
	wr(gpucc, ZP_CFG_U, 0x05022001);
	wr(gpucc, ZP_CFG_U1, 0x00000010);
	wr(gpucc, ZP_USER, 0x01000101);
	rmw(gpucc, ZP_MODE, ZP_BYPASSNL, 0);
	rmw(gpucc, ZP_MODE, 0, ZP_RESET_N);
	rmw(gpucc, ZP_MODE, 0, ZP_BYPASSNL);
	IODelay(1);
	rmw(gpucc, ZP_MODE, 0, ZP_RESET_N);
	wr(gpucc, ZP_OPMODE, 1);
	if (!poll(gpucc, ZP_MODE, ZP_LOCK, ZP_LOCK, 1000)) {
		LOG("gpu_cc_pll0: no lock at %u MHz (mode %08x): GPLL0", mhz, rd(gpucc, ZP_MODE));
		return false;
	}
	rmw(gpucc, ZP_USER, 0, 0xF);
	rmw(gpucc, ZP_MODE, 0, ZP_OUTCTRL);
	IODelay(100);
	LOG("gpu_cc_pll0 locked at %u MHz (l %u alpha %04x)", mhz, l, a);
	return true;
}

/* ---- 0337: the core clock as the stock driver makes it ---- */
#define ZP_TEST         0x1c
#define ZP_STAY_IN_CFA  (1u << 16)
#define ZP_FREQ_LOCK    (1u << 29)
#define QFPROM_PA       0x01b40000ull
#define CXIP_PA         0x003ed000ull           /* cx_ipeak@3ed000, qcom,cx-ipeak-v2 */
#define CXIP_SIZE       0x0000f000u

/* The rows of the stock ftbl_gpu_cc_gx_gfx3d_clk_src, in kHz. */
static const uint32_t stock_khz[] = { 320000, 465000, 600000, 785000, 820000, 980000, 1025000, 1100000, 1114800, 1260000 };

static uint32_t
stock_opp_khz(uint32_t mhz)
{
	for (unsigned i = 0; i < sizeof(stock_khz) / sizeof(stock_khz[0]); i++) {
		if (stock_khz[i] / 1000 == mhz) {
			return stock_khz[i];
		}
	}
	return 0;
}

/*
 * L and alpha for a VCO rate, as the stock clk_zonda_pll_set_rate() computes them: alpha is the
 * fraction in 16 bits, rounded down (alpha_pll_round_rate), and when its top bit is set L is the
 * nearest integer and not the floor (zonda_pll_adjust_l_val): alpha is then a negative fraction.
 * 2520000 kHz is L 131 alpha 0x4000; 640000 kHz is the stock L 0x21 alpha 0x5555.
 */
static void
zonda_la_khz(uint32_t vco_khz, uint32_t *l, uint32_t *a)
{
	uint32_t rem = vco_khz % 19200u;

	*l = vco_khz / 19200u;
	*a = (uint32_t)(((uint64_t)rem << 16) / 19200u);
	if ((*a & 0x8000u) != 0 && 2 * rem >= 19200u) {
		*l += 1;
	}
}

/* And back: the VCO in kHz that L and alpha stand for (clk_zonda_pll_recalc_rate). */
static uint32_t
zonda_khz(uint32_t l, uint32_t a)
{
	a &= 0xffffu;
	l &= 0xffffu;
	if ((a & 0x8000u) != 0 && l != 0) {
		l -= 1;
	}
	return (uint32_t)(((((uint64_t)l << 16) + a) * 19200000ull >> 16) + 999) / 1000;
}

/* The speed bin: the nvmem cell gpu_speed_bin of the stock device tree, reg <0x6006 2>, bits <5 8>. */
static uint32_t
speed_bin_fuse(void)
{
	uint32_t bin = ~0u;

	if (PE_parse_boot_argn("gpu_bin", &bin, sizeof(bin))) {
		LOG("  speed bin %u (boot-arg gpu_bin)", bin);
		return bin;
	}
	IOMemoryMap *m = map_phys(QFPROM_PA + 0x6000, 0x1000, false);
	if (m == NULL) {
		LOG("  the QFPROM could not be mapped: speed bin unknown");
		return ~0u;
	}
	volatile uint8_t *q = (volatile uint8_t *)m->getVirtualAddress();
	uint32_t w = rd(q, 0x04), g = rd(q, 0x2c);
	m->release();
	bin = (w >> 21) & 0xff;
	LOG("  speed bin fuse %u (QFPROM 6004: %08x), gaming bin %u (602c: %08x)", bin, w, (g >> 13) & 1, g);
	return bin;
}

/* The fastest rate the stock device tree gives a speed bin (qcom,gpu-pwrlevels-N of this phone's tree). */
static uint32_t
bin_max_khz(uint32_t bin)
{
	return bin == 0 ? 1260000 : bin == 235 ? 1114800 : bin == 216 ? 1025000 : (bin == 157 || bin == 127) ? 600000 : 1025000;
}

/* The CX level the stock driver votes for a core rate (rate_max of gpu_cc_gx_gfx3d_clk_src, vdd_l2_corner). */
static uint32_t
cx_level_for(uint32_t khz)
{
	return khz <= 320000 ? 64 : khz <= 465000 ? 128 : khz <= 600000 ? 192 : khz <= 785000 ? 256 :
	       khz <= 1025000 ? 384 : khz <= 1114800 ? 416 : 464;
}

/* The MX level the stock driver votes for the PLL's VCO (rate_max of gpu_cc_pll0, vdd_corner). */
static uint32_t
mx_level_for(uint32_t vco_khz)
{
	return vco_khz <= 1800000 ? 64 : vco_khz <= 2400000 ? 128 : vco_khz <= 3000000 ? 256 : 384;
}

/*
 * The CX peak-current votes kgsl makes before it crosses 900 and 950 MHz (stock device tree
 * qcom,gpu-cx-ipeak: client 1 of cx_ipeak@3ed000 from 900 MHz, client 10 from 950 MHz; "hardware
 * damage is possible at peak current if mitigation not done", kgsl_pwrctrl.c). As the stock
 * cx_ipeak.c does for the v2 block: client N's register is at (N + 1) * 0x1000, plus 0x3000 from
 * client 10 on; a client votes by writing 1 there unless the feature is off (+ 0x10 of the block)
 * or the client is bypassed (+ 4 of the client), and the vote stands if the danger register (+ 0x24)
 * reads 0 within 50 ms. Votes for the rates khz reaches, none for the others; false when the
 * limits manager answers with danger, and then no vote is left. gpu_ipeak=0: no votes at all.
 */
static bool
cx_ipeak_set(uint32_t khz)
{
	static const struct { uint32_t khz, off; } client[] = { { 900000, 2 * 0x1000 }, { 950000, 11 * 0x1000 + 0x3000 } };
	static volatile uint8_t *cxip;
	static bool voted[2];
	uint32_t on = 1, enabled;
	bool ok = true;

	(void)PE_parse_boot_argn("gpu_ipeak", &on, sizeof(on));
	if (on == 0 || (khz < client[0].khz && !voted[0] && !voted[1])) {
		return true;
	}
	if (cxip == NULL) {
		IOMemoryMap *m = map_phys(CXIP_PA, CXIP_SIZE, false);
		if (m == NULL) {
			LOG("  cx_ipeak could not be mapped: no peak-current vote");
			return true;
		}
		cxip = (volatile uint8_t *)m->getVirtualAddress();
	}
	enabled = rd(cxip, 0x10);
	for (unsigned i = 0; i < 2; i++) {
		if (khz < client[i].khz || !ok) {
			if (voted[i]) {
				wr(cxip, client[i].off, 0);
				voted[i] = false;
			}
			continue;
		}
		if (voted[i]) {
			continue;
		}
		if (enabled == 0 || (rd(cxip, client[i].off + 4) & 1) != 0) {
			if (!opp_quiet) {                   /* 0345: not at every change of rate */
				LOG("  cx_ipeak: no vote for %u kHz and above (feature enable %08x, the client's bypass %08x)",
				    client[i].khz, enabled, rd(cxip, client[i].off + 4));
			}
			continue;
		}
		wr(cxip, client[i].off, 1);
		ok = poll(cxip, 0x24, 0xFFFFFFFFu, 0, 50000);
		if (!opp_quiet || !ok) {
			LOG("  cx_ipeak: vote for %u kHz and above (client register %x): danger %08x: %s", client[i].khz,
			    client[i].off, rd(cxip, 0x24), ok ? "taken" : "REFUSED");
		}
		if (ok) {
			voted[i] = true;
		} else {
			wr(cxip, client[i].off, 0);
			if (voted[0]) {
				wr(cxip, client[0].off, 0);
				voted[0] = false;
			}
		}
	}
	return ok;
}

/* clk_zonda_pll_configure() and then clk_zonda_pll_enable(), for a VCO rate. */
static bool
gpu_pll_start_vco(uint32_t vco_khz)
{
	uint32_t l, a, lock;

	zonda_la_khz(vco_khz, &l, &a);
	wr(gpucc, ZP_L, l);
	wr(gpucc, ZP_ALPHA, a);
	wr(gpucc, ZP_CFG, 0x08200800);
	wr(gpucc, ZP_CFG_U, 0x05022001);
	wr(gpucc, ZP_CFG_U1, 0x00000010);
	wr(gpucc, ZP_USER, 0x01000101);             /* bits 9:8 = 1: OUT_MAIN is the VCO / 2 */
	rmw(gpucc, ZP_MODE, ZP_BYPASSNL, 0);
	rmw(gpucc, ZP_MODE, ZP_OUTCTRL, 0);
	wr(gpucc, ZP_OPMODE, 0);                    /* standby */
	rmw(gpucc, ZP_MODE, 0, ZP_RESET_N);
	rmw(gpucc, ZP_MODE, 0, ZP_BYPASSNL);
	IODelay(1);
	rmw(gpucc, ZP_MODE, 0, ZP_RESET_N);
	wr(gpucc, ZP_OPMODE, 1);                    /* run */
	lock = (rd(gpucc, ZP_TEST) & ZP_STAY_IN_CFA) != 0 ? ZP_FREQ_LOCK : ZP_LOCK;
	if (!poll(gpucc, ZP_MODE, lock, lock, 1000)) {
		LOG("gpu_cc_pll0: no lock with the VCO at %u kHz (l %u alpha %04x, mode %08x, test %08x)", vco_khz, l, a,
		    rd(gpucc, ZP_MODE), rd(gpucc, ZP_TEST));
		rmw(gpucc, ZP_MODE, ZP_OUTCTRL, 0);     /* clk_zonda_pll_disable() */
		rmw(gpucc, ZP_USER, 0xF, 0);
		rmw(gpucc, ZP_MODE, ZP_RESET_N | ZP_BYPASSNL, 0);
		wr(gpucc, ZP_OPMODE, 0);
		return false;
	}
	rmw(gpucc, ZP_USER, 0, 0xF);
	rmw(gpucc, ZP_MODE, 0, ZP_OUTCTRL);
	IODelay(100);
	if (!opp_quiet) {                           /* 0345 */
		LOG("gpu_cc_pll0 locked: VCO %u kHz (l %u alpha %04x), USER_CTL %08x: OUT_MAIN = VCO / 2 = %u kHz", vco_khz, l,
		    a, rd(gpucc, ZP_USER), vco_khz / 2);
	}
	return true;
}

/*
 * gpu_vco2=1: the speed bin, the rails, then the PLL at twice the OPP. The core clock in kHz, or 0
 * when the old path has to do it (nothing here is left half done: the old path redoes the PLL and
 * the votes it wants).
 */
static uint32_t
core_clock_stock(void)
{
	uint32_t khz = stock_opp_khz(core_mhz), vco_khz, cx, mx, top, anybin = 0;
	int r;

	if (khz == 0) {
		LOG("gpu_vco2: gpu_mhz=%u is not a rate of the stock table (320 465 600 785 820 980 1025 1100 1114 1260)",
		    core_mhz);
		return 0;
	}
	bin_fuse = speed_bin_fuse();
	top = bin_max_khz(bin_fuse);
	(void)PE_parse_boot_argn("gpu_anybin", &anybin, sizeof(anybin));
	if (khz > top && anybin == 0) {
		LOG("gpu_vco2: %u kHz is above what the stock device tree gives speed bin %u: %u kHz (gpu_anybin=1 lifts this)",
		    khz, bin_fuse, top);
		khz = top;
	}
	if (!cx_ipeak_set(khz)) {
		LOG("gpu_vco2: the CX peak-current limit refuses %u kHz: 820000 kHz, the fastest stock rate that needs no vote", khz);
		khz = 820000;
	}
	vco_khz = 2 * khz;
	cx = cx_arg != ~0u ? cx_arg : MAX(cx_level_for(khz), 0x1a0u);
	mx = mx_arg != ~0u ? mx_arg : MAX(mx_level_for(vco_khz), 0x1a0u);
	if (cx != 0) {
		r = topaz_rpm_vote(0, 0x78637772, 0, 0x6c766c76, cx);          /* "rwcx" 0, "vlvl" */
		LOG("  CX rail (RPM rwcx 0) to level %u for a %u kHz core clock (the stock driver votes %u): %d", cx, khz,
		    cx_level_for(khz), r);
		if (r != 0) {
			(void)cx_ipeak_set(0);
			return 0;
		}
		cx_voted = cx;
	}
	if (mx != 0) {
		r = topaz_rpm_vote(0, 0x786d7772, 0, 0x6c766c76, mx);          /* "rwmx" 0, "vlvl" */
		LOG("  MX rail (RPM rwmx 0) to level %u for a %u kHz VCO (the stock driver votes %u): %d", mx, vco_khz,
		    mx_level_for(vco_khz), r);
		if (r != 0) {
			(void)cx_ipeak_set(0);
			return 0;
		}
		mx_voted = mx;
	}
	if (!gpu_pll_start_vco(vco_khz)) {
		(void)cx_ipeak_set(0);
		return 0;
	}
	return khz;
}

/*
 * What the core clock is, read back from the PLL and the RCG (kern.sweet_gpu_pll, and one line of
 * the boot log). The GPU CC answers whether or not the GPU's domains are up.
 */
static void
gpu_pll_line(char *line, size_t size)
{
	uint32_t mode = rd(gpucc, ZP_MODE), l = rd(gpucc, ZP_L), a = rd(gpucc, ZP_ALPHA), user = rd(gpucc, ZP_USER);
	uint32_t cmd = rd(gpucc, 0x101c), cfg = rd(gpucc, 0x1020), cbcr = rd(gpucc, 0x1054);
	uint32_t src = (cfg >> 8) & 7, hid = cfg & 0x1f, post = (user >> 8) & 3;
	uint32_t postdiv = post == 0 ? 1 : post == 1 ? 2 : 4;                  /* 1 << fls(), as the stock postdiv reads it */
	uint32_t vco = (mode & ZP_OUTCTRL) != 0 ? zonda_khz(l, a) : 0;
	uint32_t from = src == SRC_GPLL0 ? 600000 : src == SRC_GPU_PLL0 ? vco / postdiv : src == 2 ? vco : src == 0 ? 19200 : 0;
	uint32_t core = (cbcr & CBCR_OFF) != 0 || (cmd & RCG_ROOT_OFF) != 0 ? 0 : 2 * from / (hid + 1);

	snprintf(line, size, "gx_gfx3d %u kHz from %s; gpu_cc_pll0 mode %08x l %u alpha %04x user_ctl %08x opmode %u "
	    "test_ctl %08x config_ctl %08x %08x %08x: VCO %u kHz, OUT_MAIN = VCO / %u; rcg cmd %08x cfg %08x (source %u, "
	    "divider %u.%u); branch %08x; RPM votes of this driver: CX %u MX %u; speed bin %d; gpu_mhz %u gpu_vco2 %u "
	    "gpu_div %u", core, src == SRC_GPLL0 ? "GPLL0 (600000 kHz)" : src == SRC_GPU_PLL0 ? "gpu_cc_pll0 OUT_MAIN" :
	    src == 2 ? "gpu_cc_pll0's VCO" : src == 0 ? "the crystal" : "another source", mode, l, a & 0xffff, user,
	    rd(gpucc, ZP_OPMODE), rd(gpucc, ZP_TEST), rd(gpucc, ZP_CFG), rd(gpucc, ZP_CFG_U), rd(gpucc, ZP_CFG_U1), vco,
	    postdiv, cmd, cfg, src, (hid + 1) / 2, ((hid + 1) & 1) * 5, cbcr, cx_voted, mx_voted, (int)bin_fuse, core_mhz,
	    vco2, core_div);
}

#include <sys/sysctl.h>
#include <libkern/libkern.h>

static int
sweet_gpu_pll_sysctl SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	char line[640];

	if (gpucc == NULL) {
		strlcpy(line, "the GPU clock controller is not mapped (no gpu= boot-arg, or too early)", sizeof(line));
	} else {
		gpu_pll_line(line, sizeof(line));
	}
	return SYSCTL_OUT(req, line, strlen(line) + 1);
}
SYSCTL_PROC(_kern, OID_AUTO, sweet_gpu_pll, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED,
    NULL, 0, sweet_gpu_pll_sysctl, "A", "0337: the GPU's PLL and core clock, read back from the GPU clock controller");

static bool
rcg_set(uint32_t cmd, uint32_t src, uint32_t div, const char *name)
{
	bool ok;

	wr(gpucc, cmd + 4, (src << 8) | (2 * div - 1));
	rmw(gpucc, cmd, 0, RCG_UPDATE);
	ok = poll(gpucc, cmd, RCG_UPDATE, 0, 500);
	if (src == SRC_GPLL0) {
		LOG("  %s src %u div %u (GPLL0/%u = %u MHz): cmd %08x cfg %08x %s", name, src, div, div, 600 / div,
		    rd(gpucc, cmd), rd(gpucc, cmd + 4), ok ? "updated" : "UPDATE STUCK");
	} else {
		/* 0337: not GPLL0; OUT_MAIN is half the VCO */
		LOG("  %s src %u div %u (gpu_cc_pll0 OUT_MAIN / %u; OUT_MAIN = VCO / 2): cmd %08x cfg %08x %s", name, src, div,
		    div, rd(gpucc, cmd), rd(gpucc, cmd + 4), ok ? "updated" : "UPDATE STUCK");
	}
	step_done();
	return ok;
}

static bool
power_up(void)
{
	bool ok = true;

	LOG("--- power up");
	/* GCC side: GPLL0 to the GPU CC, and the GPU's bus clocks */
	LOG("  about to vote GPLL0 to the GPU (GCC 79004 = %08x)", rd(gcc, 0x79004));
	step_done();
	rmw(gcc, 0x79004, 0, 1u << 15);
	LOG("  gcc vote 79004 = %08x (gpu_gpll0_clk_src)", rd(gcc, 0x79004));
	step_done();
	branch_on(gcc, 0x71154, "gcc_bimc_gpu_axi");
	branch_on(gcc, 0x3600c, "gcc_gpu_memnoc_gfx");      /* voted branch: may read off until used */
	branch_on(gcc, 0x36018, "gcc_gpu_snoc_dvm_gfx");
	/* CX first: the GPU CC cxo/ahb branches live in that domain and stay off before it */
	if (!gdsc_on(0x106c, 0, 0, 0x1540, "gpu_cx_gdsc")) {
		return false;
	}
	ok &= branch_on(gpucc, 0x109c, "gpucc_cxo");
	ok &= branch_on(gpucc, 0x1078, "gpucc_ahb");
	if (!ok) {
		return false;
	}
	rcg_set(0x1120, SRC_GPLL0, 3, "gmu_clk_src");
	rmw(gpucc, 0x1098, 0xFF0, 0xFF0);                   /* cx_gmu wakeup/sleep delays, as Linux probes */
	step_done();
	branch_on(gpucc, 0x1098, "gpucc_cx_gmu");
	branch_on(gpucc, 0x5000, "gpucc_hlos1_vote_gpu_smmu");
	if (smmu_early != 0 && smmu != NULL) {
		LOG("  gpu_smmu_early: the identity bank goes in before the GX domain");
		smmu_identity();
		step_done();
	}
	/* GX domain, then the core clock from GPLL0 (the GPU CC PLLs are left unconfigured) */
	if (!gdsc_on(0x100c, 0x1008, 0x1508, 0x100c, "gpu_gx_gdsc")) {
		return false;
	}
	/*
	 * 0204: the GX domain is fed by the CX rail (pm6125 S3, RPM "rwcx" 0, its level by key "vlvl";
	 * gpu_gx_gdsc's parent in the device tree). The stock levels: 1025 MHz TURBO (0x180),
	 * 1114.8 and 1260 MHz TURBO_L1 (0x1a0). Raised before the clock, never lowered.
	 */
	uint32_t stock = vco2 != 0 && core_mhz != 0 ? core_clock_stock() : 0;      /* 0337: the core clock in kHz */
	if (stock == 0 && core_mhz >= 980) {
		uint32_t level = core_mhz > 1025 ? 0x1a0 : 0x180;
		int r = topaz_rpm_vote(0, 0x78637772, 0, 0x6c766c76, level);
		LOG("  CX rail to RPM level %#x for %u MHz: %d", level, core_mhz, r);
		if (r == 0) {
			cx_voted = level;                       /* 0337: for kern.sweet_gpu_pll */
		}
		if (r != 0 && core_mhz > 1025) {
			LOG("  the CX level was not taken: 1025 MHz instead of %u", core_mhz);
			core_mhz = 1025;
		}
	}
	if (stock != 0) {
		ok &= rcg_set(0x101c, SRC_GPU_PLL0, 1, "gx_gfx3d_clk_src");
		opp_khz = ok ? stock : 0;               /* 0345: the rate kern.sweet_gpu_khz starts from */
	} else if (core_mhz != 0 && gpu_pll_start(core_mhz)) {
		ok &= rcg_set(0x101c, SRC_GPU_PLL0, 1, "gx_gfx3d_clk_src");
		LOG("  gpu_mhz=%u without gpu_vco2=1 is the PLL's VCO: the core clock is half of it, %u.%u MHz", core_mhz,
		    core_mhz / 2, (core_mhz & 1) * 5);
	} else {
		ok &= rcg_set(0x101c, SRC_GPLL0, core_div, "gx_gfx3d_clk_src");
	}
	rmw(gpucc, 0x1054, 0, (1u << 14) | (1u << 13));     /* force mem core / periph on */
	step_done();
	ok &= branch_on(gpucc, 0x1054, "gpucc_gx_gfx3d");
	branch_on(gpucc, 0x1060, "gpucc_gx_cxo");
	{
		char line[640];                         /* 0337: what the core clock is, read back */
		gpu_pll_line(line, sizeof(line));
		LOG("*** core clock: %s ***", line);
	}
	/* the line three boots have died just after this point */
	gic_report("with the GPU powered");
	step_done();
	return ok;
}

/*
 * 0343: Qualcomm's clock measurement, as the stock debugcc-khaje.c and clk-debug.c do it (MiCode topaz-t-oss
 * 345999da: clk-debug.c run_measurement() and clk_debug_mux_measure_rate(); debugcc-khaje.c for the muxes and their
 * inputs). The GCC debug mux (GCC + 0x62000, its divider + 0x30000, its branch + 0x30004) picks a clock; a counter runs
 * on it while XO/4 (4.8 MHz) counts a number of ticks (control GCC + 0x62038, status + 0x6203c); a short run first
 * tells a clock that is off, whose counter does not move. The GPU CC has its own mux (GPU CC + 0x1568, a divider by 2
 * at + 0x10fc, its branch + 0x1100) on input 0xed of the GCC's; the CPU clusters' clocks come through the APSS mux
 * (0xf11101c, a divider by 8) on input 0xb6; the DDR's clock is read as a period in ps from the memory controller's
 * clock block (mccc_debug, 0x447d200 + 0x20).
 */
#define GCC_DBG_MUX         0x62000u
#define GCC_DBG_DIV         0x30000u
#define GCC_DBG_CBCR        0x30004u
#define GCC_XO_DIV4_CBCR    0x28008u
#define GCC_MEAS_CTL        0x62038u
#define GCC_MEAS_STATUS     0x6203cu
#define GPUCC_DBG_MUX       0x1568u
#define GPUCC_DBG_DIV       0x10fcu
#define GPUCC_DBG_CBCR      0x1100u
#define MEAS_TERM_MASK      0x000fffffu         /* XO_DIV4_TERM_CNT_MASK */
#define MEAS_CNT_EN         (1u << 20)
#define MEAS_CLR_CNT        (1u << 21)
#define MEAS_DONE           (1u << 25)          /* XO_DIV4_CNT_DONE */
#define MEAS_CNT_MASK       0x01ffffffu         /* MEASURE_CNT */
#define MEAS_TICKS_SHORT    0x1000u             /* SAMPLE_TICKS_1_MS */
#define MEAS_TICKS_FULL     0x20000u            /* SAMPLE_TICKS_27_MS */
#define APSS_DBG_PA         0x0f111000ull       /* the mux register is at + 0x1c */
#define MCCC_PA             0x0447d000ull       /* the period register is at + 0x220 */

static volatile uint8_t *apss_dbg, *mccc;

/* A device page mapped once and kept; two callers racing both map it and one mapping is given back. */
static volatile uint8_t *
map_reg_page(volatile uint8_t *volatile *where, uint64_t pa)
{
	if (*where == NULL) {
		IOMemoryMap *m = map_phys(pa, 0x1000, false);
		if (m == NULL) {
			return NULL;
		}
		if (!OSCompareAndSwapPtr(NULL, (void *)m->getVirtualAddress(), (void * volatile *)where)) {
			m->release();
		}
	}
	return *where;
}

static uint32_t
meas_run(uint32_t ticks)
{
	rmw(gcc, GCC_MEAS_CTL, MEAS_CNT_EN, 0);
	rmw(gcc, GCC_MEAS_CTL, 0, MEAS_CLR_CNT);
	IODelay(1);
	rmw(gcc, GCC_MEAS_CTL, MEAS_CLR_CNT, 0);
	rmw(gcc, GCC_MEAS_CTL, MEAS_TERM_MASK, ticks & MEAS_TERM_MASK);
	rmw(gcc, GCC_MEAS_CTL, 0, MEAS_CNT_EN);
	for (unsigned i = 0; i < 100 && (rd(gcc, GCC_MEAS_STATUS) & MEAS_DONE) == 0; i++) {
		IOSleep(1);
	}
	rmw(gcc, GCC_MEAS_CTL, MEAS_CNT_EN, 0);
	return rd(gcc, GCC_MEAS_STATUS) & MEAS_CNT_MASK;
}

/* kHz of input sel of the GCC debug mux, times the dividers on the way to it; 0: the clock is off. */
static uint32_t
meas_khz(uint32_t sel, uint32_t dividers)
{
	uint32_t xo4 = rd(gcc, GCC_XO_DIV4_CBCR), brief, full;

	rmw(gcc, GCC_DBG_MUX, 0x3ff, sel);
	rmw(gcc, GCC_DBG_DIV, 0xf, 0);              /* post_div_val 1 */
	rmw(gcc, GCC_DBG_CBCR, 0, CBCR_EN);
	wr(gcc, GCC_XO_DIV4_CBCR, xo4 | CBCR_EN);
	brief = meas_run(MEAS_TICKS_SHORT);
	full = meas_run(MEAS_TICKS_FULL);
	wr(gcc, GCC_XO_DIV4_CBCR, xo4);
	rmw(gcc, GCC_DBG_CBCR, CBCR_EN, 0);
	if (full == brief) {
		return 0;
	}
	/* clk_debug_mux_measure_rate(): ((count * 10 + 15) * 4.8 MHz) / (ticks * 10 + 35) */
	uint64_t hz = ((uint64_t)full * 10 + 15) * 4800000ull / ((uint64_t)MEAS_TICKS_FULL * 10 + 35);
	return (uint32_t)(hz * dividers / 1000);
}

static uint32_t
meas_gpucc_khz(uint32_t sel)
{
	uint32_t khz;

	rmw(gpucc, GPUCC_DBG_MUX, 0xff, sel);
	rmw(gpucc, GPUCC_DBG_DIV, 0xf, 1);          /* post_div_val 2 */
	rmw(gpucc, GPUCC_DBG_CBCR, 0, CBCR_EN);
	khz = meas_khz(0xed, 2);
	rmw(gpucc, GPUCC_DBG_CBCR, CBCR_EN, 0);
	return khz;
}

static uint32_t
meas_cpu_khz(uint32_t perf)
{
	if (map_reg_page(&apss_dbg, APSS_DBG_PA) == NULL) {
		return 0;
	}
	rmw(apss_dbg, 0x1c, 0x3ff00, perf << 8);    /* 1 measure_only_perf_clk (gold), 0 measure_only_pwclk_clk (silver) */
	rmw(apss_dbg, 0x1c, 0xf0000000u, 0);        /* post_div_val 1; the fixed pre-divider is 8 */
	return meas_khz(0xb6, 8);
}

/* The DDR's clock from the memory controller's period register (stock clk-debug.c: 10^12 / period); 0: unknown. */
static uint32_t
ddr_period_ps(void)
{
	return map_reg_page(&mccc, MCCC_PA) != NULL ? rd(mccc, 0x220) : 0;
}

static uint32_t
ddr_khz(void)
{
	uint32_t ps = ddr_period_ps();
	return ps != 0 ? (uint32_t)((1000000000ull + ps / 2) / ps) : 0;
}

/* The nearest row of the DDR plan of this phone (stock device tree, and xbl_a's table) to a measured kHz. */
static uint32_t
ddr_plan_khz(uint32_t khz)
{
	static const uint32_t plan[] = { 200000, 547200, 768000, 1017600, 1555200, 1804800, 2092800 };
	uint32_t best = 0;

	for (unsigned i = 0; i < sizeof(plan) / sizeof(plan[0]); i++) {
		uint32_t d = plan[i] > khz ? plan[i] - khz : khz - plan[i];
		uint32_t bd = best > khz ? best - khz : khz - best;
		if (best == 0 || d < bd) {
			best = plan[i];
		}
	}
	return khz != 0 ? best : 0;
}

/* One line with the clocks that matter to the GPU, measured now. It sleeps: not for interrupt context. */
static void
clock_report(char *line, size_t size)
{
	uint32_t ddr = ddr_khz(), ps = ddr_period_ps();

	if (!clocks_up || clk_lock == NULL || gcc == NULL || gpucc == NULL) {
		snprintf(line, size, "the GPU's clocks are not up; ddr %u kHz (period %u ps, plan row %u kHz)", ddr, ps,
		    ddr_plan_khz(ddr));
		return;
	}
	IOLockLock(clk_lock);
	uint32_t gx = meas_gpucc_khz(0x0b), gmu = meas_gpucc_khz(0x18), memnoc = meas_khz(0xee, 1);
	uint32_t bimc = meas_khz(0x97, 1), gold = meas_cpu_khz(1), silver = meas_cpu_khz(0);
	IOLockUnlock(clk_lock);
	snprintf(line, size, "measured, kHz: gx_gfx3d %u, cx_gmu %u, gcc_gpu_memnoc_gfx %u, gcc_bimc_gpu_axi %u, "
	    "ddr %u (period %u ps, plan row %u), cpu gold %u, cpu silver %u", gx, gmu, memnoc, bimc, ddr, ps,
	    ddr_plan_khz(ddr), gold, silver);
}

static int
sweet_gpu_clocks_sysctl SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	char line[360];

	clock_report(line, sizeof(line));
	return SYSCTL_OUT(req, line, strlen(line) + 1);
}
SYSCTL_PROC(_kern, OID_AUTO, sweet_gpu_clocks, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED,
    NULL, 0, sweet_gpu_clocks_sysctl, "A", "sapphire 0343: the GPU, bus, DDR and CPU clocks, measured at the read");

static int
sweet_ddr_khz_sysctl SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	int khz = (int)ddr_khz();

	return SYSCTL_OUT(req, &khz, sizeof(khz));
}
SYSCTL_PROC(_kern, OID_AUTO, sweet_ddr_khz, CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_LOCKED,
    NULL, 0, sweet_ddr_khz_sysctl, "I", "sapphire 0343: the DDR's clock in kHz, from the memory controller's period register");

/*
 * 0343: the temperature sensors (TSENS v2, tsens0 of the stock device tree: compatible qcom,tsens24xx, SROT at 0x4410000,
 * TM at 0x4411000; the stock driver for it is msm-tsens-driver.ko, CodeLinaro msm-5.15 drivers/thermal/qcom/tsens2xxx.c,
 * and Linux's tsens-v2.c drives the same block on sm6115). The controller converts in hardware from its fused
 * calibration and the boot firmware starts it; the OS only reads:
 *   SROT_CTRL (SROT + 4)        bit 0 the controller is on, bit 3 + n sensor n is converted;
 *   TM_TRDY (TM + 0xe4)         bit 3 the first round of conversions is complete; 0 after a reset of the controller,
 *                               when the values are stale (stock checks it before every read, tsens2xxx.c);
 *   TM_Sn_STATUS (TM + 0xa0 + 4 n)  bits 11:0 the last temperature in tenths of a degree, signed; bit 21 VALID, 0 for a
 *                               few AHB cycles while a new value lands (polled here up to 20 times, 1 us apart).
 * A value outside -40..150 C is taken as not valid. Reads have no side effect (the interrupts are cleared by writes to
 * other registers). The names are those of the thermal zones that use each sensor.
 */
#define TSENS_SROT_PA       0x04410000ull
#define TSENS_TM_PA         0x04411000ull
#define TSENS_SENSORS       16u
#define TSENS_GPU           15u
#define TSENS_SROT_CTRL     0x04u
#define TSENS_TM_TRDY       0xe4u
#define TSENS_FIRST_ROUND   (1u << 3)
#define TSENS_TM_SN_STATUS  0xa0u
#define TSENS_VALID         (1u << 21)

static const char *const tsens_name[TSENS_SENSORS] = {
	"mapss", "cdsp-hvx", "wlan", "camera", "video", "mdm-1", "cpu-1-0", "cpu-1-1", "cpu-1-2", "cpu-1-3",
	"cpuss-0", "cpuss-1", "cpuss-2", "mdm-0", "display", "gpu",
};
static volatile uint8_t *tsens_srot, *tsens_tm;

/* tenths of a degree, 12 bits two's complement, to milli-degrees */
static int32_t
tsens_dc_to_mc(uint32_t status)
{
	int32_t dc = (int32_t)((status & 0xfffu) ^ 0x800u) - 0x800;
	return dc * 100;
}

/*
 * Sensor n in milli-degrees C. 0, or: ENXIO not mapped, ENODEV the controller or the sensor is off, EAGAIN the first
 * round is not complete (a controller reset: stale values), EIO no VALID within 20 us or a value out of range.
 */
static int
tsens_read(unsigned n, int32_t *mc)
{
	if (n >= TSENS_SENSORS || map_reg_page(&tsens_tm, TSENS_TM_PA) == NULL ||
	    map_reg_page(&tsens_srot, TSENS_SROT_PA) == NULL) {
		return ENXIO;
	}
	uint32_t ctrl = rd(tsens_srot, TSENS_SROT_CTRL);
	if ((ctrl & 1) == 0 || (ctrl & (1u << (3 + n))) == 0) {
		return ENODEV;
	}
	if ((rd(tsens_tm, TSENS_TM_TRDY) & TSENS_FIRST_ROUND) == 0) {
		return EAGAIN;
	}
	for (unsigned i = 0; i < 20; i++) {
		uint32_t st = rd(tsens_tm, TSENS_TM_SN_STATUS + 4 * n);
		if ((st & TSENS_VALID) != 0) {
			int32_t v = tsens_dc_to_mc(st);
			if (v < -40000 || v > 150000) {
				return EIO;
			}
			*mc = v;
			return 0;
		}
		IODelay(1);
	}
	return EIO;
}

static void
thermal_report(char *line, size_t size)
{
	uint32_t ctrl = map_reg_page(&tsens_srot, TSENS_SROT_PA) != NULL ? rd(tsens_srot, TSENS_SROT_CTRL) : 0;
	uint32_t trdy = map_reg_page(&tsens_tm, TSENS_TM_PA) != NULL ? rd(tsens_tm, TSENS_TM_TRDY) : 0;
	size_t off = 0;
	int n;

	n = snprintf(line, size, "tsens ctrl %08x (%s, sensors on %04x), trdy %08x (%s), milli-degrees C:", ctrl,
	    (ctrl & 1) != 0 ? "on" : "OFF", (ctrl >> 3) & 0xffff, trdy, (trdy & TSENS_FIRST_ROUND) != 0 ? "ready" : "NOT READY");
	off = n > 0 && (size_t)n < size ? (size_t)n : size - 1;
	for (unsigned i = 0; i < TSENS_SENSORS && off + 1 < size; i++) {
		int32_t mc = 0;
		int e = tsens_read(i, &mc);
		if (e == 0) {
			n = snprintf(line + off, size - off, " %s %d", tsens_name[i], mc);
		} else {
			n = snprintf(line + off, size - off, " %s %s", tsens_name[i], e == ENODEV ? "off" : e == EAGAIN ?
			    "not-ready" : e == ENXIO ? "unmapped" : "invalid");
		}
		off = n > 0 && off + (size_t)n < size ? off + (size_t)n : size - 1;
	}
}

static int
sweet_thermal_sysctl SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	char line[480];

	thermal_report(line, sizeof(line));
	return SYSCTL_OUT(req, line, strlen(line) + 1);
}
SYSCTL_PROC(_kern, OID_AUTO, sweet_thermal, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED,
    NULL, 0, sweet_thermal_sysctl, "A", "sapphire 0343: every TSENS sensor by its thermal zone's name, milli-degrees C");

static int
sweet_gpu_temp_sysctl SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	int32_t mc = 0;
	int e = tsens_read(TSENS_GPU, &mc);

	if (e != 0) {
		return e;
	}
	int v = (int)mc;
	return SYSCTL_OUT(req, &v, sizeof(v));
}
SYSCTL_PROC(_kern, OID_AUTO, sweet_gpu_temp, CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_LOCKED,
    NULL, 0, sweet_gpu_temp_sysctl, "I", "sapphire 0343: the GPU's temperature sensor (TSENS 15), milli-degrees C");

/*
 * 0361: for the RPM's clock scaling and the DDR manager (TopazModem.cpp, report 24): an input of the GCC debug mux
 * measured as kern.sweet_gpu_clocks does and under its lock (0: off, or the clock pages not mapped yet), the DDR's
 * clock (mccc's period), and the hottest of the sixteen sensors.
 */
extern "C" uint32_t
sapphire_clk_measure_khz(uint32_t sel)
{
	uint32_t khz = 0;

	if (!clocks_up || clk_lock == NULL || gcc == NULL) {
		return 0;
	}
	IOLockLock(clk_lock);
	khz = meas_khz(sel, 1);
	IOLockUnlock(clk_lock);
	return khz;
}

extern "C" uint32_t
sapphire_ddr_khz(void)
{
	return ddr_khz();
}

extern "C" int
sapphire_tsens_max_mc(int32_t *mc)
{
	bool any = false;
	int32_t best = 0;

	for (unsigned n = 0; n < TSENS_SENSORS; n++) {
		int32_t v = 0;
		if (tsens_read(n, &v) == 0 && (!any || v > best)) {
			best = v;
			any = true;
		}
	}
	if (!any) {
		return EIO;
	}
	*mc = best;
	return 0;
}

static void
power_down(void)
{
	LOG("--- power down");
	clocks_up = false;                          /* 0343 */
	rmw(gpucc, 0x1054, CBCR_EN, 0);
	rmw(gpucc, 0x100c, 0, GDSC_COLLAPSE);
	rmw(gpucc, 0x1508, 0, 1);
	rmw(gpucc, 0x1098, CBCR_EN, 0);
	rmw(gpucc, 0x5000, CBCR_EN, 0);
	rmw(gpucc, 0x106c, 0, GDSC_COLLAPSE);
	LOG("  gx gdscr %08x, cx gdscr %08x", rd(gpucc, 0x100c), rd(gpucc, 0x106c));
}

/*
 * Linux a6xx_bus_clear_pending_transactions(gx_off = true), which a GPU without a GMU runs
 * before every power collapse: halt the GX side of GBIF, then new client requests, then all
 * AXI requests, waiting for each ack, and release the GBIF halt. Collapsing the domains with
 * the CP's transactions still in flight leaves GBIF wedged mid-transfer, and the memory NoC
 * the display fetches from with it: the panel underflows to a flat colour and its DSI
 * commands stop completing. That is what every boot that parked the GPU after its CP had
 * run did (gpu_stop=3, 4 and the full stage 3), and what neither the zap-only boot nor the
 * one that left the GPU running did. RBBM_GBIF_HALT stays set; hw_init_regs clears it.
 */
static bool
halt_ack(uint32_t reg, uint32_t mask)
{
	for (int i = 0; i < 1000; i++) {
		if ((grd(reg) & mask) == mask) {
			return true;
		}
		IODelay(1);
	}
	return false;
}

static void
bus_drain(void)
{
	bool gx, client, arb;

	gwr(REG_RBBM_GBIF_HALT, 1);
	gx = halt_ack(REG_RBBM_GBIF_HALT_ACK, 1);
	gwr(REG_GBIF_HALT, GBIF_CLIENT_HALT);
	client = halt_ack(REG_GBIF_HALT_ACK, GBIF_CLIENT_HALT);
	gwr(REG_GBIF_HALT, GBIF_ARB_HALT);
	arb = halt_ack(REG_GBIF_HALT_ACK, GBIF_ARB_HALT);
	gwr(REG_GBIF_HALT, 0);
	gmb();
	LOG("  bus drained: GX %s, clients %s, AXI %s (GBIF_HALT_ACK %08x)", gx ? "halted" : "NO ACK",
	    client ? "halted" : "NO ACK", arb ? "halted" : "NO ACK", grd(REG_GBIF_HALT_ACK));
}

/* The counter runs off the 19.2 MHz crystal: ~192000 ticks in 10 ms means the GPU is alive. */
static void
first_reads(void)
{
	uint32_t lo1, hi1, lo2, hi2;

	LOG("--- first GPU reads: about to touch %llx for the first time", GPU_PA);
	step_done();
	LOG("  RBBM_STATUS   %08x", grd(REG_RBBM_STATUS));
	step_done();
	LOG("  RBBM_INT_0_STATUS %08x (what the GPU is asking for)", grd(REG_RBBM_INT_0_STATUS));
	if (gpu_irq == 0) {
		/* and now at the source, not only in the distributor */
		gwr(REG_RBBM_INT_0_MASK, 0);
		gwr(REG_RBBM_INT_CLEAR_CMD, 0xFFFFFFFF);
		gmb();
		LOG("  interrupts masked at the GPU too: INT_0_STATUS %08x", grd(REG_RBBM_INT_0_STATUS));
		gic_report("after masking at the GPU");
	}
	step_done();
	LOG("  CP_HW_FAULT   %08x", grd(REG_CP_HW_FAULT));
	lo1 = grd(REG_CP_ALWAYS_ON_COUNTER);
	hi1 = grd(REG_CP_ALWAYS_ON_COUNTER + 1);
	IOSleep(10);
	lo2 = grd(REG_CP_ALWAYS_ON_COUNTER);
	hi2 = grd(REG_CP_ALWAYS_ON_COUNTER + 1);
	LOG("  CP_ALWAYS_ON_COUNTER %08x%08x -> %08x%08x in 10 ms (delta %u, 19.2 MHz = ~192000)",
	    hi1, lo1, hi2, lo2, lo2 - lo1);
}

/* ---------------------------------------------------------------- firmware */

/* The same firmware, carried in the device tree by the boot image. */
static const struct { const char *file, *prop; } adt_fw_map[] = {
	{ "a630_sqe.fw",  "gpu-sqe-firmware" },
	{ "a610_zap.mdt", "gpu-zap-metadata" },
	{ "a610_zap.b02", "gpu-zap-segment" },
};

static int64_t
adt_fw_read(const char *name, void *dst, uint32_t max)
{
	DTEntry chosen;
	void const *blob;
	unsigned int size;

	for (unsigned i = 0; i < sizeof(adt_fw_map) / sizeof(adt_fw_map[0]); i++) {
		if (strcmp(adt_fw_map[i].file, name) != 0) {
			continue;
		}
		if (SecureDTLookupEntry(NULL, "/chosen", &chosen) != kSuccess ||
		    SecureDTGetProperty(chosen, adt_fw_map[i].prop, &blob, &size) != kSuccess) {
			return -1;
		}
		if (size > max) {
			LOG("  %s: %u bytes in the device tree, no room", name, size);
			return -1;
		}
		memcpy(dst, blob, size);
		LOG("  %s: %u bytes from the device tree", name, size);
		return size;
	}
	return -1;
}

/* Read a firmware file into dst. Returns its length, or -1. */
static int64_t
fw_read(const char *name, void *dst, uint32_t max)
{
	int64_t n = adt_fw_read(name, dst, max);

	if (n >= 0) {
		return n;
	}
	for (unsigned i = 0; i < fw_count(); i++) {
		if (strcmp(gpu_fw[i].name, name) != 0) {
			continue;
		}
		if (gpu_fw[i].size > max) {
			LOG("  %s: %u bytes, no room", name, gpu_fw[i].size);
			return -1;
		}
		int err = sweet_ufs_read_part(gpu_fw[i].part, gpu_fw[i].off, dst, gpu_fw[i].size);
		if (err != 0) {
			LOG("  %s: read from %s failed: %d", name, gpu_fw[i].part, err);
			return -1;
		}
		uint32_t crc = crc32(0, dst, gpu_fw[i].size);
		if (crc != gpu_fw[i].crc) {
			/* TrustZone checks the signature anyway: say so and go on */
			LOG("  %s: crc32 %08x, expected %08x: not the firmware this table was made for", name, crc,
			    gpu_fw[i].crc);
		}
		return gpu_fw[i].size;
	}
	LOG("  %s: not in the firmware table", name);
	return -1;
}

/*
 * The zap shader leaves the GPU's secure mode: TrustZone authenticates it (PAS id 13) and
 * runs it. Metadata = the whole .mdt; the loadable segment goes to the reserved region the
 * device tree keeps for it. PAS mem_setup wants a 4 KB-aligned size (Linux mdt_loader does
 * ALIGN(paddr + memsz, SZ_4K)); an unaligned one is refused with 0xffcfffba.
 */
static bool
zap_load(void)
{
	uint8_t *meta = pool + OFF_META;
	uint64_t meta_pa = POOL_PA + OFF_META, st, res = 0;
	uint32_t phoff, phnum, min_addr = 0xFFFFFFFFu, max_addr = 0, load_paddr = 0;
	uint32_t seg_size;
	int64_t mdt_size;
	bool found = false;

	LOG("--- zap shader (PAS %u)", GPU_PAS_ID);
	step_done();
	mdt_size = fw_read("a610_zap.mdt", meta, POOL_SIZE - OFF_META);
	if (mdt_size < 52) {
		return false;
	}
	memcpy(&phoff, meta + 28, 4);
	if (phoff >= (uint32_t)mdt_size) {
		LOG("  program headers at %x, past the %lld bytes read", phoff, mdt_size);
		return false;
	}
	uint16_t n16;
	memcpy(&n16, meta + 44, 2);
	phnum = n16;
	for (uint32_t i = 0; i < phnum && phoff + 32 * (i + 1) <= (uint32_t)mdt_size; i++) {
		uint32_t ptype, paddr, memsz, filesz, flags;
		memcpy(&ptype, meta + phoff + 32 * i, 4);
		memcpy(&paddr, meta + phoff + 32 * i + 12, 4);
		memcpy(&filesz, meta + phoff + 32 * i + 16, 4);
		memcpy(&memsz, meta + phoff + 32 * i + 20, 4);
		memcpy(&flags, meta + phoff + 32 * i + 24, 4);
		LOG("  phdr %u type %u paddr %x filesz %u memsz %u flags %08x", i, ptype, paddr, filesz, memsz, flags);
		if (ptype == 1 && memsz != 0 && ((flags >> 24) & 7) != 2) {   /* PT_LOAD, not the hash segment */
			if (paddr < min_addr) {
				min_addr = paddr;
			}
			if (((paddr + memsz + 0xFFFu) & ~0xFFFu) > max_addr) {
				max_addr = (paddr + memsz + 0xFFFu) & ~0xFFFu;
			}
			load_paddr = paddr;
			found = true;
		}
	}
	if (!found || max_addr - min_addr > ZAP_SIZE) {
		LOG("  unexpected layout (found %d, %x..%x)", found, min_addr, max_addr);
		return false;
	}

	/* A half-finished PAS from an earlier try refuses everything: start clean */
	step_done();
	st = scm(SCM_FN(SCM_SVC_PIL, PIL_SHUTDOWN), 1, GPU_PAS_ID, 0, 0, &res);
	LOG("  shutdown: ret %llx res %llx", st, res);
	step_done();
	st = scm(SCM_FN(SCM_SVC_PIL, PIL_INIT_IMAGE), 2 | (SCM_ARG_RW << 6), GPU_PAS_ID, meta_pa, 0, &res);
	LOG("  init_image: ret %llx res %llx (%lld bytes of metadata at %llx)", st, res, mdt_size, meta_pa);
	step_done();
	if (st != 0 || res != 0) {
		/* already authenticated in this boot: ask TrustZone to resume it */
		st = scm(SCM_FN(SCM_SVC_BOOT, SCM_BOOT_SET_REMOTE_STATE), 2, 0, GPU_PAS_ID, 0, &res);
		LOG("  set_remote_state(resume): ret %llx res %llx (0/0 = the zap shader is already up)", st, res);
		return st == 0 && res == 0;
	}
	st = scm(SCM_FN(SCM_SVC_PIL, PIL_MEM_SETUP), 3, GPU_PAS_ID, ZAP_PA, max_addr - min_addr, &res);
	LOG("  mem_setup %llx+%x: ret %llx res %llx", ZAP_PA, max_addr - min_addr, st, res);
	step_done();
	if (st != 0 || res != 0) {
		return false;
	}

	zap_map = map_phys(ZAP_PA, ZAP_SIZE, true);
	if (zap_map == NULL) {
		LOG("  the zap region could not be mapped");
		return false;
	}
	uint8_t *dst = (uint8_t *)zap_map->getVirtualAddress();
	bzero(dst, ZAP_SIZE);
	seg_size = ZAP_SIZE - (load_paddr - min_addr);
	if (fw_read("a610_zap.b02", dst + (load_paddr - min_addr), seg_size) < 0) {
		return false;
	}
	gmb();
	LOG("  segment loaded at %llx, authenticating", ZAP_PA + (load_paddr - min_addr));
	step_done();
	st = scm(SCM_FN(SCM_SVC_PIL, PIL_AUTH_RESET), 1, GPU_PAS_ID, 0, 0, &res);
	LOG("  auth_and_reset: ret %llx res %llx resumes %u", st, res, scm_resumes);
	step_done();
	return st == 0 && res == 0;
}

/* ---------------------------------------------------------------- GPU SMMU */

/*
 * The GPU's SMMU-500 comes out of the bootloader with every stream faulting. One
 * pass-through context bank (SCTLR.M = 0) for the GPU's stream makes GPU addresses equal
 * physical ones, which is all the ring and the buffers below 4 GB need. Real stage-1 page
 * tables come later, when a user space driver allocates its own buffers.
 */
/*
 * GPU virtual memory: stage-1 page tables in the GPU's SMMU context bank, so the GPU can reach
 * memory that is not physically contiguous (everything iOS allocates). Boot-arg gpu_mmu=1.
 *
 * Format and register values are Linux's: drivers/iommu/io-pgtable-arm.c (ARM_64_LPAE_S1,
 * 4 KB granule, a walker that is not cache coherent) and drivers/iommu/arm/arm-smmu
 * (arm_smmu_write_context_bank, with Qualcomm's Adreno write_sctlr). The a610 has
 * ADRENO_QUIRK_4GB_VA, so the GPU's addresses are 32 bits: T0SZ = 32, the walk starts at level
 * 1 (VA[31:30], 4 entries), then level 2 (2 MB) and level 3 (4 KB). Output addresses are 36
 * bits, which covers all of this phone's RAM.
 *
 * The tables live in the pool, which is write-combined: the CPU's stores reach RAM at the next
 * dsb, and the walker reads RAM (TCR IRGN0/ORGN0 = non-cacheable), so no cache maintenance.
 * Everything that worked with the identity bank stays at VA = PA (the pool, the boot
 * framebuffer) as 2 MB blocks; other memory gets 4 KB pages anywhere below 4 GB.
 */
#define PTE_TYPE_TABLE      3ull                /* also TYPE_PAGE at level 3 */
#define PTE_TYPE_BLOCK      1ull
#define PTE_ADDR_MASK       0x0000fffffffff000ull
/* AttrIndx 0 (MAIR NC), AP_UNPRIV, SH_OS, AF, nG: Linux's prot for non-cached read/write */
#define PTE_RW_NC           ((1ull << 6) | (2ull << 8) | (1ull << 10) | (1ull << 11))
#define SMMU_MAIR0          0xf404ff44u         /* NC, WBRWA, device, INC_OWBRWA */
#define SMMU_TCR            ((2u << 12) | (1u << 23) | 32u)  /* SH0 OS, EPD1, T0SZ 32, TG0 4 KB, NC walks */
#define SMMU_TCR2           ((7u << 15) | 1u)   /* SEP upstream, PASIZE 36 bits */
#define SMMU_CBAR_S1        ((1u << 16) | (0xFu << 12) | (3u << 8))  /* S1 trans S2 bypass, WB, NSH */
#define SMMU_SCTLR_XLATE    (0x1000u | 0x100u | 0x40u | 0x20u | 0x4u | 0x2u | 0x1u)  /* ASIDPNE HUPCF CFIE CFRE AFE TRE M */

static uint32_t mmu;
static uint32_t pt_used;
static uint64_t *pt_l1;
static uint64_t pt_l1_pa;

/*
 * More page tables, for the msm client's buffers (up to 2 GB of 4 KB pages need 1024 tables):
 * the 4 MB of the same unmanaged low-bank hole right after the pool (modem pool 0xE0000000 + 16
 * MB, our pool + 1 MB, these + 4 MB, all below 0xF0000000), mapped write-combined like the
 * pool, so the SMMU's non-cached walks see every entry after a dsb. Mapped on first use.
 */
#define PT2_PA              0xE1100000ull
#define PT2_SIZE            0x00400000u
#define PT2_PAGES           (PT2_SIZE / 4096u)
static IOMemoryMap *pt2_map;
static uint8_t *pt2;
static uint32_t pt2_used;

static uint64_t *
pt_va(uint64_t pa)
{
	if (pa >= POOL_PA && pa < POOL_PA + POOL_SIZE) {
		return (uint64_t *)(pool + (pa - POOL_PA));
	}
	return (uint64_t *)(pt2 + (pa - PT2_PA));
}

static uint64_t
pt_alloc(uint64_t **va)
{
	if (pt_used < PT_PAGES) {
		uint32_t off = OFF_PT + pt_used++ * 4096u;
		*va = (uint64_t *)(pool + off);
		bzero(*va, 4096);
		return POOL_PA + off;
	}
	if (pt2 == NULL) {
		pt2_map = map_phys(PT2_PA, PT2_SIZE, true);
		if (pt2_map == NULL) {
			return 0;
		}
		pt2 = (uint8_t *)pt2_map->getVirtualAddress();
		LOG("  page tables: the pool's %u are used up, %u more at %llx", PT_PAGES, PT2_PAGES, PT2_PA);
	}
	if (pt2_used >= PT2_PAGES) {
		return 0;
	}
	uint32_t off = pt2_used++ * 4096u;
	*va = (uint64_t *)(pt2 + off);
	bzero(*va, 4096);
	return PT2_PA + off;
}

static uint64_t *
pt_next(uint64_t *e)
{
	if (*e == 0) {
		uint64_t *t;
		uint64_t pa = pt_alloc(&t);
		if (pa == 0) {
			return NULL;
		}
		*e = pa | PTE_TYPE_TABLE;
		return t;
	}
	if ((*e & 3) != PTE_TYPE_TABLE) {
		return NULL;                            /* a block is already there */
	}
	return pt_va(*e & PTE_ADDR_MASK);
}

static bool
map_block_2m(uint64_t va, uint64_t pa)
{
	uint64_t *l2 = pt_next(&pt_l1[(va >> 30) & 3]);
	if (l2 == NULL || va >= (1ull << 32)) {
		return false;
	}
	l2[(va >> 21) & 511] = (pa & ~0x1FFFFFull) | PTE_TYPE_BLOCK | PTE_RW_NC;
	return true;
}

static bool
map_page_4k(uint64_t va, uint64_t pa)
{
	uint64_t *l2 = pt_next(&pt_l1[(va >> 30) & 3]);
	uint64_t *l3 = l2 != NULL ? pt_next(&l2[(va >> 21) & 511]) : NULL;
	if (l3 == NULL || va >= (1ull << 32)) {
		return false;
	}
	l3[(va >> 12) & 511] = (pa & PTE_ADDR_MASK) | PTE_TYPE_TABLE | PTE_RW_NC;
	return true;
}

/* Clear one 4 KB entry, if its tables exist; the caller flushes the TLB. */
static void
unmap_page_4k(uint64_t va)
{
	uint64_t e1 = pt_l1[(va >> 30) & 3];
	if ((e1 & 3) != PTE_TYPE_TABLE) {
		return;
	}
	uint64_t *l2 = pt_va(e1 & PTE_ADDR_MASK);
	uint64_t e2 = l2[(va >> 21) & 511];
	if ((e2 & 3) != PTE_TYPE_TABLE) {
		return;
	}
	uint64_t *l3 = pt_va(e2 & PTE_ADDR_MASK);
	l3[(va >> 12) & 511] = 0;
}

/*
 * sapphire 0341: kern.sweet_gpu_tlb (boot-arg gpu_tlb), two bits, both set by default; 0 is as before.
 *   2  the wait for the invalidation to finish looks at TLBSTATUS without sleeping, for up to 200 us, before it
 *      falls back on poll(). poll() waits with IODelay(1), which on this port blocks the thread (ml_delay_should_spin()
 *      is false: SweetCPU gives no idle latency), and the sync is almost never over at the first look: 13251 of
 *      13358 invalidations slept, 73 us each on average, with the ring's lock held (measured with 0336). Linux spins
 *      too before it delays (arm-smmu.c, __arm_smmu_tlb_sync: TLB_SPIN_COUNT reads, then udelay).
 *   1  a buffer that is only being mapped is not followed by an invalidation (place()). The entries written were
 *      invalid before, the SMMU caches no translation it could not make, and a range of GPU addresses is given out
 *      again only after freeBo() has unmapped it and invalidated: there is nothing stale to remove. Linux does the
 *      same: msm_iommu_pagetable_map() invalidates nothing, msm_iommu_pagetable_unmap() invalidates everything.
 *      The windows of the panel's copies replace live entries and go on invalidating (map_window()).
 * An invalidation that was not seen to finish leaves tlb_stale set, and then nothing is mapped without another.
 */
static int gpu_tlb = 3;
static bool tlb_stale;
static struct {
	uint64_t spun, looks, fell_back, maps_plain, maps_flushed;
} gtlb;

/* TLBIASID 0, then TLBSYNC and wait for TLBSTATUS.SACTIVE to clear. */
static bool
tlb_flush(void)
{
	uint64_t t0 = mach_absolute_time(), sleeps = gdiag.poll_sleeps;         /* 0336 */
	bool ok = false;

	gmb();
	wr(smmu, 0x8610, 0);
	wr(smmu, 0x87f0, 0);
	if (gpu_tlb & 2) {
		uint64_t span;
		nanoseconds_to_absolutetime(200 * NSEC_PER_USEC, &span);
		gtlb.spun++;
		do {
			gtlb.looks++;
			if ((rd(smmu, 0x87f4) & 1) == 0) {
				ok = true;
				break;
			}
		} while (mach_absolute_time() - t0 < span);
		if (!ok) {
			gtlb.fell_back++;
		}
	}
	if (!ok) {
		ok = poll(smmu, 0x87f4, 1, 0, 1000);
	}
	tlb_stale = !ok;
	gdiag_since(&gdiag.tlb, t0);
	if (gdiag.poll_sleeps != sleeps) {
		gdiag.tlb_slept++;                      /* the sync was not over at the first look: the thread slept */
	}
	return ok;
}

static int
sweet_gpu_tlb_stats_sysctl SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	char line[256];

	snprintf(line, sizeof(line), "tlb (kern.sweet_gpu_tlb %d): invalidations waited for without sleeping %llu, in %llu looks, %llu fell back on sleeping; "
	    "buffers mapped without an invalidation %llu, with one %llu\n", gpu_tlb, gtlb.spun, gtlb.looks, gtlb.fell_back, gtlb.maps_plain,
	    gtlb.maps_flushed);
	return SYSCTL_OUT(req, line, strlen(line) + 1);
}
SYSCTL_PROC(_kern, OID_AUTO, sweet_gpu_tlb_stats, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED,
    NULL, 0, sweet_gpu_tlb_stats_sysctl, "A", "sapphire 0341: the GPU SMMU's TLB invalidations");
SYSCTL_INT(_kern, OID_AUTO, sweet_gpu_tlb, CTLFLAG_RW | CTLFLAG_LOCKED, &gpu_tlb, 0,
    "sapphire 0341: bit 0 = no TLB invalidation after only mapping a buffer, bit 1 = wait for the sync without sleeping (0 = as before)");

static bool
smmu_pagetables(void)
{
	PE_Video video;

	if (smmu == NULL) {
		return false;
	}
	pt_used = 0;
	pt_l1_pa = pt_alloc(&pt_l1);
	bool ok = map_block_2m(POOL_PA, POOL_PA);
	PE_current_console(&video);
	uint64_t fb = video.v_baseAddr, fb_end = fb + (uint64_t)video.v_rowBytes * video.v_height;
	for (uint64_t a = fb & ~0x1FFFFFull; ok && fb != 0 && a < fb_end; a += 0x200000) {
		ok = map_block_2m(a, a);
	}
	if (!ok) {
		LOG("  SMMU: the identity blocks did not fit the page tables");
		return false;
	}
	gmb();
	wr(smmu, 0x8000, 0);                        /* CB0 off while it changes */
	wr(smmu, 0x8058, 0xFFFFFFFF);               /* CB0 FSR: clear */
	wr(smmu, 0x1800, 1);                        /* CBA2R0: VA64 */
	wr(smmu, 0x1000, SMMU_CBAR_S1);             /* CBAR0 */
	wr(smmu, 0x8010, SMMU_TCR2);
	wr(smmu, 0x8030, SMMU_TCR);
	wr(smmu, 0x8020, (uint32_t)pt_l1_pa);
	wr(smmu, 0x8024, (uint32_t)(pt_l1_pa >> 32));  /* TTBR0, ASID 0 */
	wr(smmu, 0x8028, 0);
	wr(smmu, 0x802C, 0);                        /* TTBR1 */
	wr(smmu, 0x8038, SMMU_MAIR0);
	wr(smmu, 0x803C, 0);
	gmb();
	wr(smmu, 0x8000, SMMU_SCTLR_XLATE);
	gmb();
	wr(smmu, 0x0C00, 0);                        /* S2CR0: translate through CB0 */
	wr(smmu, 0x0800, (1u << 31) | (1u << 16));  /* SMR0: SID 0, mask 1 */
	ok = tlb_flush();
	LOG("  SMMU page tables: L1 at %llx, %u tables used, identity for the pool and %llx-%llx; CB0 SCTLR %08x TCR %08x TCR2 %08x TTBR0 %08x%08x, TLB sync %s",
	    pt_l1_pa, pt_used, fb, fb_end, rd(smmu, 0x8000), rd(smmu, 0x8030), rd(smmu, 0x8010), rd(smmu, 0x8024),
	    rd(smmu, 0x8020), ok ? "done" : "STUCK");
	return true;
}

static bool
smmu_identity(void)
{
	if (smmu == NULL) {
		return false;
	}
	wr(smmu, 0x8058, 0xFFFFFFFF);               /* CB0 FSR: clear */
	wr(smmu, 0x8020, 0);
	wr(smmu, 0x8024, 0);                        /* TTBR0 */
	wr(smmu, 0x8030, 0);                        /* TCR */
	wr(smmu, 0x8010, 0);                        /* TCR2 */
	wr(smmu, 0x8038, 0);
	wr(smmu, 0x803C, 0);                        /* MAIR */
	wr(smmu, 0x1800, 1);                        /* CBA2R0: VA64 */
	wr(smmu, 0x1000, 0x0001f000);               /* CBAR0 */
	gmb();
	wr(smmu, 0x8000, 0xE0);                     /* CB0 SCTLR: M = 0, report faults */
	gmb();
	wr(smmu, 0x0C00, 0);                        /* S2CR0: translate through CB0 */
	wr(smmu, 0x0800, (1u << 31) | (1u << 16));  /* SMR0: SID 0, mask 1 */
	gmb();
	LOG("  SMMU: SMR0 %08x S2CR0 %08x CBAR0 %08x CBA2R0 %08x CB0 SCTLR %08x", rd(smmu, 0x800), rd(smmu, 0xC00),
	    rd(smmu, 0x1000), rd(smmu, 0x1800), rd(smmu, 0x8000));
	return true;
}

static void
smmu_fault(const char *tag)
{
	if (smmu == NULL) {
		return;
	}
	LOG("  [%s] SMMU gfsr %08x CB0 SCTLR %08x FSR %08x FAR %08x%08x FSYNR0 %08x", tag, rd(smmu, 0x48),
	    rd(smmu, 0x8000), rd(smmu, 0x8058), rd(smmu, 0x8064), rd(smmu, 0x8060), rd(smmu, 0x8068));
}

/* ---------------------------------------------------------------- CP bring-up */

/* The subset of Linux a6xx_gpu.c hw_init the command processor needs to run at all. */
/*
 * sapphire 0348: hardware clock gating of the GPU's blocks, as kgsl turns it on for this GPU at every start
 * (a6xx_hwcg_set(): the a612_hwcg table, which kgsl's adreno-gpulist.h gives the a610, and which is Linux's
 * a6xx_catalog.c a612_hwcg, 46 registers equal one by one; then UCHE_GBIF_GX_CONFIG[18:16] = 2, the GBIF's L2 gating;
 * then RBBM_CLOCK_CNTL = 0xaaa8aa82). Off: every register of the table and RBBM_CLOCK_CNTL to 0, the GBIF field to 0.
 * Nothing is written if RBBM_CLOCK_CNTL already says so. Without a GMU kgsl skips the GMU's always-on CGC registers
 * (Linux writes them through its wrapper); so does this. boot-arg gpu_hwcg=1: at every start of the CP (hw_init_regs,
 * also after a reset of the GPU); kern.sweet_gpu_hwcg: 0/1 now, with the service lock held and the GPU idle, and
 * from then on at every start. Off by default: with it on, the blocks' clocks stop while they have nothing to do.
 */
static int gpu_hwcg;                            /* boot-arg gpu_hwcg, kern.sweet_gpu_hwcg */

static void
hwcg_set(bool on)
{
	static const struct { uint16_t reg; uint32_t val; } hwcg[] = {
		{ 0x0b0, 0x22222222 }, { 0x0b4, 0x02222220 }, { 0x0b8, 0x00000081 }, { 0x0bc, 0x0000f3cf },
		{ 0x0c0, 0x22222222 }, { 0x0c4, 0x22222222 }, { 0x0c8, 0x22222222 }, { 0x0cc, 0x00022222 },
		{ 0x0d0, 0x11111111 }, { 0x0d4, 0x11111111 }, { 0x0d8, 0x11111111 }, { 0x0dc, 0x00011111 },
		{ 0x0e0, 0x77777777 }, { 0x0e4, 0x77777777 }, { 0x0e8, 0x77777777 }, { 0x0ec, 0x00077777 },
		{ 0x0f0, 0x22222222 }, { 0x0f4, 0x01202222 }, { 0x0f8, 0x00002220 }, { 0x100, 0x00040f00 },
		{ 0x104, 0x05522022 }, { 0x105, 0x00005555 }, { 0x106, 0x00000011 }, { 0x107, 0x00445044 },
		{ 0x108, 0x04222222 }, { 0x111, 0x00002222 }, { 0x114, 0x02222222 }, { 0x117, 0x00000002 },
		{ 0x11b, 0x00002222 }, { 0x109, 0x00004000 }, { 0x112, 0x00002222 }, { 0x115, 0x00000200 },
		{ 0x11c, 0x00000000 }, { 0x10a, 0x00000000 }, { 0x113, 0x00000000 }, { 0x116, 0x04104004 },
		{ 0x11d, 0x00000000 }, { 0x10b, 0x22222222 }, { 0x110, 0x00000004 }, { 0x10f, 0x00000002 },
		{ 0x533, 0x00000182 }, { 0x044, 0x00000000 }, { 0x042, 0x00000000 }, { 0x118, 0x00000222 },
		{ 0x119, 0x00000111 }, { 0x11a, 0x00000555 },
	};
	uint32_t cntl = grd(0xae);                  /* RBBM_CLOCK_CNTL */

	if ((on && cntl == 0xaaa8aa82u) || (!on && cntl == 0)) {
		return;
	}
	for (unsigned i = 0; i < sizeof(hwcg) / sizeof(hwcg[0]); i++) {
		gwr(hwcg[i].reg, on ? hwcg[i].val : 0);
	}
	gwr(0xe3a, (grd(0xe3a) & ~0x70000u) | (on ? 2u << 16 : 0));    /* UCHE_GBIF_GX_CONFIG */
	gwr(0xae, on ? 0xaaa8aa82u : 0);
}

/*
 * sapphire 0370: the GPU's UBWC and tiling configuration -- how the RB, the TP, the SP and the UCHE lay out and read a
 * TILE6_3 image (with or without UBWC): the highest bank bit, the UBWC 1.0 bank swizzling, the level-2 swizzling and
 * the minimum access length (RB/TPL1/SP_NC_MODE_CNTL, UCHE_MODE_CNTL), and the macrotile mode (RBBM_NC_MODE_CNTL).
 * Until now nothing here wrote them: they hold their reset values, the same for every unit as far as the images the
 * driver already compresses show (read-only textures written by the 2D engine, read by the TP).
 *   1: what kgsl writes on this phone (kgsl 5.x, graphics-kernel f900d903 adreno_a6xx.c a6xx_start :676-737): the DT's
 *      qcom,ubwc-mode = <1> (KGSL_UBWC_1_0: mode bit 1) and qcom,min-access-length = <0x40> (mal 1), and the a610's
 *      highest_bank_bit 14 (adreno-gpulist.h:1520): hbb_lo 1, hbb_hi 0, level-2 swizzling on, amsbc and the 565
 *      predicator off; RBBM_NC_MODE_CNTL left alone (kgsl writes it for the a680 and a663 alone)
 *   2: what Linux v6.12 writes for the a610 (a6xx_gpu.c a6xx_calc_ubwc_config :513-517, a6xx_set_ubwc_config :568-609):
 *      highest bank bit 13, min_acc_len 1, ubwc_swizzle 7 (mode bit 1, level-2 on), macrotile mode 0
 * The two differ in the highest bank bit alone (14 against 13; Mesa 25.0.7's a610 row says 13 too).
 * boot-arg gpu_ubwc=1|2: written at every start of the CP (hw_init_regs, also after a reset of the GPU, 0317), the
 * values found there first kept; kern.sweet_gpu_ubwc: read, the mode; write 0/1/2 now, with service_lock held and the
 * GPU idle (EBUSY if it does not go idle within gpu_opp_drain ms), and at every start after -- 0 puts back the values
 * found at the first start. Every tiled or compressed image already in memory was laid out under the old values:
 * restart its owner (killall backboardd) after a write. kern.sweet_gpu_ubwc_regs: the five registers as they read now,
 * and as they were found. GET_PARAM HIGHEST_BANK_BIT answers the bank bit in force (14 with gpu_ubwc 0, as before).
 */
static int gpu_ubwc;                            /* boot-arg gpu_ubwc, kern.sweet_gpu_ubwc */

extern "C" int
sapphire_gpu_ubwc_mode(void)
{
	return gpu_ubwc;
}
static bool ubwc_found_ok;                      /* the values hw_init_regs found before it first wrote any */
static uint32_t ubwc_found[5];

#define REG_RB_NC_MODE_CNTL     0x8e08          /* a6xx.xml (Mesa 25.0.7 a6xx.xml.h:6779) */
#define REG_TPL1_NC_MODE_CNTL   0xb604          /* a6xx.xml.h:10138 */
#define REG_SP_NC_MODE_CNTL     0xae02          /* a6xx.xml.h:9565 */
#define REG_UCHE_MODE_CNTL      0x0e01          /* a6xx.xml.h:3916 */
#define REG_RBBM_NC_MODE_CNTL   0x0534          /* a6xx.xml.h:3202 */

static const uint16_t ubwc_regs[5] = { REG_RB_NC_MODE_CNTL, REG_TPL1_NC_MODE_CNTL, REG_SP_NC_MODE_CNTL, REG_UCHE_MODE_CNTL,
	                               REG_RBBM_NC_MODE_CNTL };

/* the bank bit in force: what GET_PARAM HIGHEST_BANK_BIT answers */
static uint32_t
ubwc_hbb(void)
{
	return gpu_ubwc == 2 ? 13 : 14;
}

/* the five values of MODE (1 kgsl's, 2 Linux's), as a6xx_set_ubwc_config packs them; RBBM_NC_MODE_CNTL ~0: not written */
static void
ubwc_values(int mode, uint32_t v[5])
{
	const uint32_t hbb = (mode == 2 ? 13 : 14) - 13, hbb_lo = hbb & 3, hbb_hi = (hbb >> 2) & 1;
	const uint32_t ubwc_mode = 1, mal = 1, level2_off = 0, amsbc = 0, rgb565 = 0, uavflag = 0;

	v[0] = (level2_off << 12) | (rgb565 << 11) | (hbb_hi << 10) | (amsbc << 4) | (mal << 3) | (hbb_lo << 1) | ubwc_mode;
	v[1] = (level2_off << 6) | (hbb_hi << 4) | (mal << 3) | (hbb_lo << 1) | ubwc_mode;
	v[2] = (level2_off << 12) | (hbb_hi << 10) | (uavflag << 4) | (mal << 3) | (hbb_lo << 1) | ubwc_mode;
	v[3] = (mal << 23) | (hbb_lo << 21);
	v[4] = mode == 2 ? 0 : ~0u;                 /* Linux writes macrotile_mode 0; kgsl leaves it */
}

/* with the GPU idle (hw_init_regs, or the sysctl under service_lock after opp_drain) */
static void
ubwc_set(int mode)
{
	uint32_t v[5];

	if (!ubwc_found_ok) {
		for (unsigned i = 0; i < 5; i++) {
			ubwc_found[i] = grd(ubwc_regs[i]);
		}
		ubwc_found_ok = true;
	}
	if (mode == 0) {
		for (unsigned i = 0; i < 5; i++) {
			v[i] = ubwc_found[i];
		}
	} else {
		ubwc_values(mode, v);
	}
	for (unsigned i = 0; i < 5; i++) {
		if (v[i] != ~0u) {
			gwr(ubwc_regs[i], v[i]);
		}
	}
}

static void
hw_init_regs(void)
{
	static const uint32_t addr_mode[] = { 0x842, 0xC01, 0x8601, 0x8e05, 0x9e01, 0xbe05, 0xa601, 0x9601,
		                              0xE00, 0xae01, 0xb601, 0xF810 };

	gwr(0x3c45, 0);                             /* GBIF_HALT */
	(void)grd(0x3c45);
	gwr(0x16, 0);                               /* RBBM_GBIF_HALT */
	(void)grd(0x16);
	gwr(0xF803, 0);                             /* RBBM_SECVID_TSB_CNTL */
	gwr64(0xF800, 0);                           /* SECVID_TSB_TRUSTED_BASE */
	gwr(0xF802, 0);                             /* SECVID_TSB_TRUSTED_SIZE */
	for (unsigned i = 0; i < sizeof(addr_mode) / sizeof(addr_mode[0]); i++) {
		gwr(addr_mode[i], 1);               /* 64-bit addressing in every block */
	}
	for (unsigned i = 0; i < 4; i++) {
		gwr(0x3c03 + i, 0x00071620);        /* GBIF_QSB_SIDE0..3 (a610) */
	}
	gwr(0x11, 3);                               /* RBBM_GBIF_CLIENT_QOS_CNTL */
	gwr64(0xE05, 0x1fffffffff000ull + 0xfc0);   /* UCHE_WRITE_RANGE_MAX */
	gwr64(0xE09, 0x1fffffffff000ull);           /* UCHE_TRAP_BASE */
	gwr64(0xE07, 0x1fffffffff000ull);           /* UCHE_WRITE_THRU_BASE */
	gwr64(0xE0B, 0x100000);                     /* UCHE_GMEM_RANGE_MIN */
	gwr64(0xE0D, 0x100000 + 0x21000 - 1);       /* UCHE_GMEM_RANGE_MAX: gmem 128K + 4K */
	gwr(0xE18, 0x804);                          /* UCHE_FILTER_CNTL */
	gwr(0xE17, 4);                              /* UCHE_CACHE_WAYS */
	gwr(0x8c2, 0x00800060);                     /* CP_ROQ_THRESHOLDS_2 */
	gwr(0x8c1, 0x40201b16);                     /* CP_ROQ_THRESHOLDS_1 */
	gwr(0x8C3, 48);                             /* CP_MEM_POOL_SIZE */
	gwr(0x9e00, 0x00080000);                    /* PC_DBG_ECO_CNTL: prim fifo threshold */
	gwr(0x98d, 1);                              /* CP_AHB_CNTL */
	gwr(0x1f, (1u << 30) | 0x3ffff);            /* RBBM_INTERFACE_HANG_INT_CNTL */
	gwr(0xe19, 0x81);                           /* UCHE_CLIENT_PF */
	if (gpu_hwcg != 0) {
		hwcg_set(true);                         /* 0348 */
	}
	if (gpu_ubwc != 0) {
		ubwc_set(gpu_ubwc);                     /* 0370 */
	}
}

/* Type-7 packet header: opcode and count each carry an odd-parity bit. */
static uint32_t
par(uint32_t v)
{
	return (0x9669u >> (0xFu & (v ^ (v >> 4) ^ (v >> 8) ^ (v >> 12) ^ (v >> 16) ^ (v >> 20) ^ (v >> 24) ^
	    (v >> 28)))) & 1;
}

static uint32_t
pkt7(uint32_t op, uint32_t cnt)
{
	return 0x70000000u | cnt | (par(cnt) << 15) | ((op & 0x7F) << 16) | (par(op) << 23);
}

/* Type-4 packet header: write cnt registers from reg on, with the same odd-parity bits. */
static uint32_t
pkt4(uint32_t reg, uint32_t cnt)
{
	return 0x40000000u | cnt | (par(cnt) << 7) | ((reg & 0x3FFFF) << 8) | (par(reg) << 27);
}

static void
emit(uint32_t v)
{
	ring[ring_wptr % RING_DWORDS] = v;
	ring_wptr++;
}

/* Publish the write pointer and wait for the CP to catch up. */
static bool
submit(const char *what, uint32_t ms)
{
	uint32_t wptr = ring_wptr % RING_DWORDS, rptr = 0;

	gmb();
	gwr(REG_CP_RB_WPTR, wptr);
	for (uint32_t i = 0; i < ms * 100; i++) {
		rptr = grd(REG_CP_RB_RPTR);
		if (rptr == wptr) {
			if (!service_ready) {           /* (the tests' lines; frame after frame they cost time) */
				LOG("  %s: done (rptr %u) after ~%u us, RBBM_STATUS %08x", what, rptr, i * 10,
				    grd(REG_RBBM_STATUS));
			}
			step_done();
			return true;
		}
		IODelay(10);
	}
	LOG("  %s: TIMEOUT rptr %u wptr %u RBBM_STATUS %08x INT_0_STATUS %08x CP_HW_FAULT %08x", what, rptr, wptr,
	    grd(REG_RBBM_STATUS), grd(REG_RBBM_INT_0_STATUS), grd(REG_CP_HW_FAULT));
	smmu_fault(what);
	step_done();
	return false;
}

/* Room for `need` more dwords in the ring (the CP reads behind us): wait up to ms. */
static bool
ring_room(uint32_t need, uint32_t ms)
{
	for (uint32_t i = 0; i < ms * 100; i++) {
		uint32_t rptr = grd(REG_CP_RB_RPTR), w = ring_wptr % RING_DWORDS;
		if (i == 0) {
			/* 0336: how much the CP has not read yet when more is added, and how often there is no room */
			uint32_t waiting = (w + RING_DWORDS - rptr) % RING_DWORDS;
			if (waiting > gdiag.sub_ring_max) {
				gdiag.sub_ring_max = waiting;
			}
		}
		if ((rptr + RING_DWORDS - w - 1) % RING_DWORDS >= need) {
			return true;
		}
		if (i == 0) {
			gdiag.ring_waits++;
		}
		IODelay(10);
	}
	return false;
}

/* Hand what was emitted to the CP and return at once (the msm client's submissions). */
static void
kick(void)
{
	gmb();
	gwr(REG_CP_RB_WPTR, ring_wptr % RING_DWORDS);
}

/*
 * 0336 (kern.sweet_gpu_marks): the CP copies its always-on counter, 64 bits, to pa when it gets here. A pair of
 * these around a piece of work, each after a CP_WAIT_FOR_IDLE, is the GPU's own time for it.
 */
static void
gdiag_mark(uint64_t pa)
{
	emit(pkt7(CP_REG_TO_MEM, 3));
	emit(REG_CP_ALWAYS_ON_COUNTER | (2u << 18) | (1u << 30));
	emit((uint32_t)pa);
	emit((uint32_t)(pa >> 32));
}

static void
gdiag_gpu_time(uint64_t ticks, const uint32_t *edges, uint64_t *n, uint64_t *sum, uint64_t *max, uint64_t *hist)
{
	unsigned k = 0;

	while (k < GDIAG_HIST - 1 && ticks >= edges[k]) {
		k++;
	}
	hist[k]++;
	(*n)++;
	*sum += ticks;
	if (ticks > *max) {
		*max = ticks;
	}
}

/*
 * The marks of the work that is done by now, into the statistics. The ring's lock held. A submission's end mark is
 * written before its fence; a copy's after it (blit_end), so a copy whose end is not there yet gets another look.
 */
static void
gdiag_harvest(void)
{
	/* ticks of 19.2 MHz: under 1 2 4 8 12 16 24 32 48 ms, more; and under 0.5 1 1.5 2 3 4 6 8 12 ms, more */
	static const uint32_t sub_edges[GDIAG_HIST - 1] = { 19200, 38400, 76800, 153600, 230400, 307200, 460800, 614400, 921600 };
	static const uint32_t copy_edges[GDIAG_HIST - 1] = { 9600, 19200, 28800, 38400, 57600, 76800, 115200, 153600, 230400 };
	uint32_t done_sub, done_copy;

	if (pool == NULL) {
		return;
	}
	done_sub = *(volatile uint32_t *)(pool + OFF_MSM_FENCE);
	done_copy = *(volatile uint32_t *)(pool + OFF_FENCE);
	while (gdiag_sub_tail != gdiag_sub_head) {
		uint32_t i = gdiag_sub_tail % GDIAG_SLOTS;
		if (gdiag_sub_head - gdiag_sub_tail > GDIAG_SLOTS) {
			gdiag.gt_lost++;                        /* its place was used again before it was read */
		} else {
			volatile uint64_t *m = (volatile uint64_t *)(pool + OFF_DIAG + i * 16u);
			if ((int32_t)(done_sub - gdiag_sub_fence[i]) < 0) {
				break;
			}
			if (m[0] != 0 && m[1] > m[0]) {
				gdiag_gpu_time(m[1] - m[0], sub_edges, &gdiag.gt_sub_n, &gdiag.gt_sub_ticks, &gdiag.gt_sub_max,
				    gdiag.gt_sub_hist);
			} else {
				gdiag.gt_lost++;                /* given up at a reset of the GPU */
			}
		}
		gdiag_sub_tail++;
	}
	while (gdiag_copy_tail != gdiag_copy_head) {
		uint32_t i = gdiag_copy_tail % GDIAG_SLOTS;
		if (gdiag_copy_head - gdiag_copy_tail > GDIAG_SLOTS) {
			gdiag.gt_lost++;
		} else {
			volatile uint64_t *m = (volatile uint64_t *)(pool + OFF_DIAG + (GDIAG_SLOTS + i) * 16u);
			if ((int32_t)(done_copy - gdiag_copy_fence[i]) < 0) {
				break;
			}
			if (m[0] != 0 && m[1] > m[0]) {
				gdiag_gpu_time(m[1] - m[0], copy_edges, &gdiag.gt_copy_n, &gdiag.gt_copy_ticks, &gdiag.gt_copy_max,
				    gdiag.gt_copy_hist);
			} else if (gdiag_copy_head - gdiag_copy_tail < 3) {
				break;                          /* the fence is there, the end mark not yet */
			} else {
				gdiag.gt_lost++;
			}
		}
		gdiag_copy_tail++;
	}
}

/*
 * Start the command processor and prove it runs: its microcode (a630_sqe.fw without its
 * first dword, as adreno_fw_create_bo copies it), a 32 KB ring, CP_ME_INIT, then leaving
 * secure mode, and a CP_MEM_WRITE the CPU reads back from RAM.
 */
static bool
cp_start(bool have_zap)
{
	uint8_t *sqe = pool + OFF_SQE;
	int64_t fw_size;

	LOG("--- CP start");
	step_done();
	/* adreno_fw_create_bo copies the firmware from its second dword on */
	fw_size = fw_read("a630_sqe.fw", sqe, OFF_RING - OFF_SQE);
	if (fw_size <= 4) {
		return false;
	}
	/* dst is below src, so a forward word copy is the shift */
	uint32_t *w = (uint32_t *)sqe;
	for (uint32_t i = 0; i < (uint32_t)(fw_size - 4 + 3) / 4; i++) {
		w[i] = w[i + 1];
	}
	test_buf[0] = 0x11111111;
	gmb();
	LOG("  sqe %lld bytes at %llx, ring at %llx, test buffer at %llx", fw_size - 4, POOL_PA + OFF_SQE,
	    POOL_PA + OFF_RING, POOL_PA + OFF_TEST);
	step_done();

	if (!(mmu ? smmu_pagetables() : smmu_identity())) {
		return false;
	}
	step_done();
	hw_init_regs();
	LOG("  hw_init done: RBBM_STATUS %08x", grd(REG_RBBM_STATUS));
	step_done();
	gwr64(REG_CP_SQE_INSTR_BASE, POOL_PA + OFF_SQE);
	gwr64(REG_CP_RB_BASE, POOL_PA + OFF_RING);
	/* 32 KB ring, 32-byte blocks, the CP does not write the read pointer back to memory */
	gwr(REG_CP_RB_CNTL, 12 | (2 << 8) | (1u << 27));
	ring_wptr = 0;
	gwr(REG_CP_RB_WPTR, 0);
	gwr(REG_CP_SQE_CNTL, 1);
	LOG("  SQE started: RBBM_STATUS %08x CP_HW_FAULT %08x rptr %u", grd(REG_RBBM_STATUS), grd(REG_CP_HW_FAULT),
	    grd(REG_CP_RB_RPTR));
	step_done();
	if (stop_here(2, "starting the SQE")) {
		return false;
	}

	emit(pkt7(CP_ME_INIT, 8));
	emit(0x2f);
	emit(3);
	emit(0x20000000);
	emit(0);
	emit(0);
	emit(0);
	emit(0);
	emit(0);
	if (!submit("CP_ME_INIT", 100)) {
		return false;
	}
	smmu_fault("after CP_ME_INIT");
	if (stop_here(3, "CP_ME_INIT")) {
		return false;
	}
	if (have_zap) {
		emit(pkt7(CP_SET_SECURE_MODE, 1));
		emit(0);
		if (!submit("CP_SET_SECURE_MODE 0", 100)) {
			return false;
		}
		smmu_fault("after CP_SET_SECURE_MODE");
		if (stop_here(4, "CP_SET_SECURE_MODE 0")) {
			return false;
		}
	} else {
		/* no zap shader: tell the hardware directly, and hope the bootloader left it open */
		gwr(REG_RBBM_SECVID_TRUST_CNTL, 0);
		LOG("  no zap shader: RBBM_SECVID_TRUST_CNTL = 0, RBBM_STATUS %08x", grd(REG_RBBM_STATUS));
		step_done();
	}
	emit(pkt7(CP_MEM_WRITE, 3));
	emit((uint32_t)(POOL_PA + OFF_TEST));
	emit((uint32_t)((POOL_PA + OFF_TEST) >> 32));
	emit(0xC0FFEE00);
	if (!submit("CP_MEM_WRITE", 100)) {
		return false;
	}
	IODelay(100);
	gmb();
	LOG("  test buffer: %08x (c0ffee00 = the GPU wrote it)", test_buf[0]);
	smmu_fault("after the test");
	step_done();
	return test_buf[0] == 0xC0FFEE00;
}

static void
reg1(uint32_t reg, uint32_t v)
{
	emit(pkt4(reg, 1));
	emit(v);
}

static uint32_t fence_seq;                      /* the last timestamp handed to the GPU */
static uint32_t msm_seq;                        /* 0153: the last msm submission's (OFF_MSM_FENCE) */

static void
event(uint32_t ev, bool ts)
{

	if (!ts) {
		emit(pkt7(CP_EVENT_WRITE, 1));
		emit(ev);
		return;
	}
	emit(pkt7(CP_EVENT_WRITE, 4));
	/*
	 * 0154: the event alone, so the CP writes the seqno below when the event is done. Bit 30 of
	 * this dword is TIMESTAMP, which writes the GPU's always-on counter (64 bits) instead -- what
	 * turnip asks for in its trace timestamps -- and every fence of 0145-0153 was that counter:
	 * "done" while it was under 2^31 (the first ~112 s of a boot at 19.2 MHz), never after.
	 */
	emit(ev);
	emit((uint32_t)(POOL_PA + OFF_FENCE));
	emit((uint32_t)((POOL_PA + OFF_FENCE) >> 32));
	emit(++fence_seq);
}

/*
 * The 2D blitter, as Mesa's freedreno does a buffer clear: blit_begin() emits the a610's
 * batch-start registers, cleans and invalidates the CCU, sets render mode BLIT2DSCALE, the CCU
 * bypass (sysmem) layout, the clear colour and the blit format; blit_rect() emits one
 * destination and one CP_BLIT(SCALE); blit_end() cleans the CCU and the caches, waits for idle
 * and submits, so the pixels are in RAM when it returns.
 */
static void
blit_setup(bool solid, uint32_t rgba)
{
	uint32_t fmt = FMT6_8_8_8_8_UNORM;
	uint32_t blt = (0xFu << 20) | (fmt << 8) | (solid ? (1u << 7) : 0);  /* MASK, COLOR_FORMAT, IFMT UNORM8, SOLID_COLOR */

	/* the a610's batch-start registers (freedreno_devices.py magic_regs / raw_magic_regs) */
	reg1(0x9804, 0xf);                          /* PC_MODE_CNTL */
	reg1(0x9805, 0);                            /* PC_POWER_CNTL */
	reg1(0xa0f8, 0);                            /* VFD_POWER_CNTL */
	reg1(0xb600, 0);                            /* TPL1_DBG_ECO_CNTL */
	reg1(0x8600, 0);                            /* GRAS_DBG_ECO_CNTL */
	reg1(0xae03, 0);                            /* SP_CHICKEN_BITS */
	reg1(0xae00, 0);                            /* SP_DBG_ECO_CNTL */
	reg1(0xbe04, 0);                            /* HLSQ_DBG_ECO_CNTL */
	reg1(0x9600, 0);                            /* VPC_DBG_ECO_CNTL */
	reg1(0x0e12, 0x10000000);                   /* UCHE_UNKNOWN_0E12 */
	reg1(REG_RB_RBP_CNTL, 1);
	reg1(REG_RB_DBG_ECO_CNTL, 0x04100000);      /* the same value for blits on a610: no toggling */

	/* emit_setup */
	event(EV_PC_CCU_FLUSH_COLOR_TS, true);
	event(EV_PC_CCU_FLUSH_DEPTH_TS, true);
	event(EV_PC_CCU_INVALIDATE_COLOR, false);
	event(EV_PC_CCU_INVALIDATE_DEPTH, false);
	emit(pkt7(CP_SET_MARKER, 1));
	emit(RM6_BLIT2DSCALE);
	emit(pkt7(CP_WAIT_FOR_IDLE, 0));
	/* sysmem layout: depth cache at 0, colour at num_ccu (1) x 8 KB; fast clear off on gen1_low */
	reg1(REG_RB_CCU_CNTL, (1u << 0) | ((0x2000u >> 12) << 23));

	/* emit_clear_color (R2D_UNORM8: one byte per dword, in the format's channel order) */
	if (solid) {
		emit(pkt4(REG_RB_A2D_CLEAR_COLOR_DW0, 4));
		emit((rgba >> 24) & 0xff);
		emit((rgba >> 16) & 0xff);
		emit((rgba >> 8) & 0xff);
		emit(rgba & 0xff);
	}
	/* emit_blit_setup */
	reg1(REG_RB_A2D_BLT_CNTL, blt);
	reg1(REG_GRAS_A2D_BLT_CNTL, blt);
	reg1(REG_SP_A2D_OUTPUT_INFO, (fmt << 3) | (0xFu << 12));  /* IFMT_TYPE FLOAT, COLOR_FORMAT, MASK */
	reg1(REG_RB_A2D_PIXEL_CNTL, 0);
}

static void
blit_begin(uint32_t rgba)
{
	blit_setup(true, rgba);
}

/* One rectangle [x0, x1] x [y0, y1] of a linear surface: base 64-byte aligned, pitch a multiple of 64. */
static void
blit_rect(uint64_t base, uint32_t pitch, uint32_t x0, uint32_t y0, uint32_t x1, uint32_t y1)
{
	emit(pkt4(REG_RB_A2D_DEST_BUFFER_INFO, 4));
	emit(FMT6_8_8_8_8_UNORM);                   /* TILE6_LINEAR, WZYX */
	emit((uint32_t)base);
	emit((uint32_t)(base >> 32));
	emit(pitch >> 6);
	emit(pkt4(REG_GRAS_A2D_DEST_TL, 2));
	emit(x0 | (y0 << 16));
	emit(x1 | (y1 << 16));
	/* emit_blit_fini */
	event(EV_DEBUG_LABEL, false);
	emit(pkt7(CP_BLIT, 1));
	emit(BLIT_OP_SCALE);
}

/*
 * boot-arg gpu_copy_async (default 1, 0152): once the service is up, a copy is handed to the CP and
 * not waited for. The CP runs the ring in order, so nothing queued after it (the next frame's
 * drawing into the same surface) overtakes it; waiting only cost the CPU (1.8 s in 5 s, measured)
 * and held the frame up behind the drawing queued before it.
 */
static uint32_t copy_async = 1;

static bool
blit_end(const char *what)
{
	event(EV_PC_CCU_FLUSH_COLOR_TS, true);
	event(EV_PC_CCU_FLUSH_DEPTH_TS, true);
	event(EV_CACHE_FLUSH_TS, true);
	emit(pkt7(CP_WAIT_FOR_IDLE, 0));
	if (gdiag_end_pa != 0) {
		gdiag_mark(gdiag_end_pa);               /* 0336: a panel copy's end, after the wait for idle */
		gdiag_end_pa = 0;
	}
	if (copy_async && service_ready) {
		kick();
		return true;
	}
	return submit(what, 200);
}

/*
 * The first thing the GPU draws: a 256 x 256 RGBA8 solid fill into a buffer the CPU has filled
 * with a pattern first, then read back pixel by pixel.
 */
static bool
blit_fill(uint32_t rgba)
{
	uint32_t *pix = (uint32_t *)(pool + OFF_DRAW);
	uint32_t good = 0, bad_at = 0xFFFFFFFF, bad_val = 0;

	LOG("--- first draw: 2D blitter, %ux%u RGBA8 at %llx, colour %08x", DRAW_W, DRAW_H, POOL_PA + OFF_DRAW, rgba);
	for (uint32_t i = 0; i < DRAW_W * DRAW_H; i++) {
		pix[i] = 0xDEADBEEF;
	}
	gmb();
	blit_begin(rgba);
	blit_rect(POOL_PA + OFF_DRAW, DRAW_PITCH, 0, 0, DRAW_W - 1, DRAW_H - 1);
	if (!blit_end("2D fill")) {
		return false;
	}
	IODelay(100);
	gmb();

	for (uint32_t i = 0; i < DRAW_W * DRAW_H; i++) {
		if (pix[i] == __builtin_bswap32(rgba)) {
			good++;
		} else if (bad_at == 0xFFFFFFFF) {
			bad_at = i;
			bad_val = pix[i];
		}
	}
	LOG("  pixels: first %08x, centre %08x, last %08x; %u of %u as expected (%08x in memory)", pix[0],
	    pix[(DRAW_H / 2) * DRAW_W + DRAW_W / 2], pix[DRAW_W * DRAW_H - 1], good, DRAW_W * DRAW_H,
	    __builtin_bswap32(rgba));
	if (bad_at != 0xFFFFFFFF) {
		LOG("  first wrong pixel: #%u (x %u, y %u) = %08x", bad_at, bad_at % DRAW_W, bad_at / DRAW_W, bad_val);
	}
	LOG("  RBBM_STATUS %08x CP_HW_FAULT %08x RBBM_INT_0_STATUS %08x", grd(REG_RBBM_STATUS), grd(REG_CP_HW_FAULT),
	    grd(REG_RBBM_INT_0_STATUS));
	return good == DRAW_W * DRAW_H;
}

/*
 * A copy of a w x h RGBA8 rectangle between two linear surfaces, as Mesa's emit_blit_buffer does
 * one (TPL1_A2D_SRC_TEXTURE_* for the source, GRAS_A2D_SRC_* its rectangle, then the same
 * destination and CP_BLIT). Both bases must be 64-byte aligned and both pitches multiples of 64,
 * so a pitch that is not (the panel's 4320) is split as the demo does: k interleaved surfaces of
 * pitch k x pitch, each base rounded down to 64 bytes with the difference moved into x.
 */
static bool
blit_copy(uint64_t src, uint32_t spitch, uint64_t dst, uint32_t dpitch, uint32_t w, uint32_t h)
{
	uint32_t k = ((spitch | dpitch) & 63) == 0 ? 1 : 2;

	if (((k * spitch) & 63) != 0 || ((k * dpitch) & 63) != 0 || (src & 3) != 0 || (dst & 3) != 0 ||
	    w + 16 > 0x4000 || h > 2 * 0x3fff) {
		LOG("  blit_copy: pitches %u/%u or size %ux%u do not fit the blitter", spitch, dpitch, w, h);
		return false;
	}
	blit_setup(false, 0);
	for (uint32_t j = 0; j < k && j < h; j++) {
		uint64_t s0 = src + (uint64_t)j * spitch, d0 = dst + (uint64_t)j * dpitch;
		uint32_t sx = (uint32_t)(s0 & 63) / 4, dx = (uint32_t)(d0 & 63) / 4;
		uint32_t n = (h - j + k - 1) / k;
		uint64_t sb = s0 & ~63ull, db = d0 & ~63ull;

		emit(pkt4(REG_TPL1_A2D_SRC_TEXTURE_INFO, 5));        /* INFO, SIZE, BASE lo/hi, PITCH */
		emit(FMT6_8_8_8_8_UNORM | (1u << 20) | (1u << 22));   /* LINEAR, WZYX, UNK20, UNK22 as Mesa */
		emit((sx + w) | (n << 15));                           /* WIDTH, HEIGHT */
		emit((uint32_t)sb);
		emit((uint32_t)(sb >> 32));
		emit(((k * spitch) >> 6) << 9);                       /* PITCH, bits 9..23, in 64-byte units */
		emit(pkt4(REG_GRAS_A2D_SRC_XMIN, 4));
		emit(sx << 8);
		emit((sx + w - 1) << 8);
		emit(0);
		emit((n - 1) << 8);
		blit_rect(db, k * dpitch, dx, 0, dx + w - 1, n - 1);
	}
	return blit_end("copy");
}

/*
 * sapphire 0239: the source rectangle [sx, sx + sw) x [sy, sy + sh) (pixels << 8: the blitter's
 * source coordinates have 8 fraction bits) scaled, bilinear, onto dw x dh pixels at dst. For iOS
 * drawing at 2x (SweetFramebuffer, sweetfb_scale=2): 720 x 1600 of a 750 x 1624 surface onto the
 * 1080 x 2400 panel. The source pitch must be a multiple of 64 bytes; a destination pitch that is
 * not (the panel's 4320) is done as blit_copy does it, in k passes of every k-th row, each pass's
 * source rows k times as far apart and started where its first row's centre falls.
 */
static bool
blit_scale(uint64_t src, uint32_t spitch, uint32_t sx, uint32_t sy, uint32_t sw, uint32_t sh,
    uint64_t dst, uint32_t dpitch, uint32_t dw, uint32_t dh)
{
	uint32_t k = (dpitch & 63) == 0 ? 1 : 2;

	if ((spitch & 63) != 0 || ((k * dpitch) & 63) != 0 || (src & 3) != 0 || (dst & 3) != 0 || dw == 0 || dh == 0 ||
	    sw == 0 || sh == 0 || dw + 16 > 0x4000 || ((sx + sw) >> 8) + 16 > 0x4000 || ((sy + sh) >> 8) + 2 > 0x7fff) {
		LOG("  blit_scale: pitches %u/%u or sizes do not fit the blitter", spitch, dpitch);
		return false;
	}
	/*
	 * 0354: whole 16-pixel groups of the source origin's column (64 bytes) and, for each pass, its whole rows go into the
	 * texture's base address; GRAS_A2D_SRC_* get only what is left (under 16 columns, a fraction of a row). The panel's
	 * window starts at whole 64 bytes of the 2x picture (SweetFramebuffer.cpp, fOffX), so this copy crops by the
	 * address alone, as the scanout does (presentScanout()): the two ways of putting the window on the panel cannot
	 * differ by how the blitter applies its source origin.
	 */
	uint32_t col = (sx >> 8) & ~15u;
	uint64_t sbase = src + (uint64_t)col * 4;
	uint32_t sxr = sx - (col << 8);
	uint64_t sb = sbase & ~63ull;
	uint32_t sadd = (uint32_t)(sbase & 63) / 4;             /* the base's misalignment, as pixels */
	uint32_t swidth = sadd + ((sxr + sw) >> 8) + 1;
	blit_setup(false, 0);
	for (uint32_t j = 0; j < k && j < dh; j++) {
		uint64_t d0 = dst + (uint64_t)j * dpitch;
		uint32_t dx = (uint32_t)(d0 & 63) / 4, n = (dh - j + k - 1) / k;
		/* the pass's n rows sample k * n * (sh / dh) source rows, the first centred on row j's */
		int64_t span = (int64_t)sh * k * n / dh;
		int64_t y0 = (int64_t)sy + ((2 * (int64_t)j + 1 - (int64_t)k) * (int64_t)sh) / (2 * (int64_t)dh);
		if (y0 < 0) {
			y0 = 0;
		}
		uint32_t row = (uint32_t)(y0 >> 8);
		uint32_t yr = (uint32_t)(y0 - ((int64_t)row << 8));
		uint64_t pb = sb + (uint64_t)row * spitch;      /* (spitch is whole 64 bytes: still aligned) */
		uint32_t sheight = ((yr + (uint32_t)span) >> 8) + 2;
		emit(pkt4(REG_TPL1_A2D_SRC_TEXTURE_INFO, 5));        /* INFO, SIZE, BASE lo/hi, PITCH */
		emit(FMT6_8_8_8_8_UNORM | (1u << 16) | (1u << 20) | (1u << 22));  /* LINEAR, WZYX, FILTER, UNK20, UNK22 */
		emit(swidth | (sheight << 15));
		emit((uint32_t)pb);
		emit((uint32_t)(pb >> 32));
		emit((spitch >> 6) << 9);
		emit(pkt4(REG_GRAS_A2D_SRC_XMIN, 4));
		emit((sadd << 8) + sxr);
		emit((sadd << 8) + sxr + sw - 256);
		emit(yr);
		emit((uint32_t)(yr + span - 256));
		blit_rect(d0 & ~63ull, k * dpitch, dx, 0, dx + dw - 1, n - 1);
	}
	return blit_end("scale");
}

/*
 * boot-arg gpu_bench=1: is the GPU worth handing AppleCLCD's per-frame copy? That copy is a
 * full 1080 x 2400 frame (10 MB) from CoreAnimation's surface into the flip buffer, memcpy then a
 * cache clean for the MDP, every frame. The GPU writes RAM behind the caches, so taking it over
 * costs a clean of the source and an invalidate of the destination instead. This times each of
 * those on the real buffers' sizes, the GPU copy of the boot framebuffer into a contiguous 10 MB
 * buffer, and checks that copy against its source, row by row.
 */
static uint32_t bench;
extern "C" void sweet_dcache_clean(vm_offset_t addr, unsigned length);
extern "C" void sweet_dcache_flush(vm_offset_t addr, unsigned length);

static uint64_t
us_since(uint64_t t0)
{
	uint64_t ns;
	absolutetime_to_nanoseconds(mach_absolute_time() - t0, &ns);
	return ns / 1000;
}

static void
bench_copy(void)
{
	PE_Video video;
	IOBufferMemoryDescriptor *a = NULL, *b = NULL;

	PE_current_console(&video);
	uint32_t w = (uint32_t)video.v_width, h = (uint32_t)video.v_height, dpitch = (uint32_t)video.v_rowBytes;
	uint32_t spitch = (w * 4 + 511) & ~511u;        /* CoreAnimation's surfaces: 1125 px in 4608 bytes */
	size_t dsize = (size_t)dpitch * h, ssize = (size_t)spitch * h;

	LOG("--- gpu_bench: one %ux%u frame, source %u bytes a row (%zu), destination %u (%zu)", w, h, spitch,
	    ssize, dpitch, dsize);
	a = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
	    kIODirectionInOut | kIOMemoryPhysicallyContiguous, dsize, 0x00000000fffff000ull);
	b = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
	    kIODirectionInOut | kIOMemoryPhysicallyContiguous, ssize, 0x00000000fffff000ull);
	if (a == NULL || b == NULL || w == 0 || h == 0) {
		LOG("  no memory for the benchmark");
		OSSafeReleaseNULL(a);
		OSSafeReleaseNULL(b);
		return;
	}
	uint8_t *av = (uint8_t *)a->getBytesNoCopy(), *bv = (uint8_t *)b->getBytesNoCopy();
	uint64_t apa = a->getPhysicalSegment(0, NULL, kIOMemoryMapperNone);
	uint64_t bpa = b->getPhysicalSegment(0, NULL, kIOMemoryMapperNone);
	uint64_t t, t_cpy, t_cpy_clean, t_src_clean, t_dst_flush, t_gpu, t_inv;
	uint32_t bad_rows = 0, first_bad = 0xFFFFFFFF;

	/* the flip path today: memcpy cached -> cached, row by row (pitches differ), then clean for the MDP */
	memset(bv, 0x5a, ssize);
	memset(av, 0, dsize);
	t = mach_absolute_time();
	for (uint32_t y = 0; y < h; y++) {
		memcpy(av + (uint64_t)y * dpitch, bv + (uint64_t)y * spitch, (size_t)w * 4);
	}
	t_cpy = us_since(t);
	t = mach_absolute_time();
	sweet_dcache_clean((vm_offset_t)av, (unsigned)dsize);
	t_cpy_clean = us_since(t);

	/* a frame as CoreAnimation leaves one: written by the CPU, in its cache; every pixel unique */
	for (uint32_t y = 0; y < h; y++) {
		uint32_t *row = (uint32_t *)(bv + (uint64_t)y * spitch);
		for (uint32_t x = 0; x < w; x++) {
			row[x] = (y << 12) ^ x ^ 0xA5000000u;
		}
	}
	memset(av, 0x11, dsize);                    /* the destination dirty in the cache too */

	/* the GPU path: clean the source, flush the destination, copy, drop the destination's lines */
	t = mach_absolute_time();
	sweet_dcache_clean((vm_offset_t)bv, (unsigned)ssize);
	t_src_clean = us_since(t);
	t = mach_absolute_time();
	sweet_dcache_flush((vm_offset_t)av, (unsigned)dsize);
	t_dst_flush = us_since(t);
	t = mach_absolute_time();
	bool ok = blit_copy(bpa, spitch, apa, dpitch, w, h);
	t_gpu = us_since(t);
	t = mach_absolute_time();
	sweet_dcache_flush((vm_offset_t)av, (unsigned)dsize);
	t_inv = us_since(t);

	for (uint32_t y = 0; ok && y < h; y++) {
		if (memcmp(av + (uint64_t)y * dpitch, bv + (uint64_t)y * spitch, (size_t)w * 4) != 0) {
			if (first_bad == 0xFFFFFFFF) {
				first_bad = y;
			}
			bad_rows++;
		}
	}
	LOG("  CPU today:  copy %llu us + clean %llu us = %llu us a frame", t_cpy, t_cpy_clean, t_cpy + t_cpy_clean);
	LOG("  GPU copy:   %s in %llu us (submit to idle, the CPU polling), %u of %u rows exactly like the source",
	    ok ? "done" : "FAILED", t_gpu, ok ? h - bad_rows : 0, h);
	if (ok && bad_rows) {
		const uint32_t *gr = (const uint32_t *)(av + (uint64_t)first_bad * dpitch);
		const uint32_t *sr = (const uint32_t *)(bv + (uint64_t)first_bad * spitch);
		LOG("  first row that differs: %u: x0 %08x (want %08x), x%u %08x (want %08x)", first_bad, gr[0], sr[0],
		    w - 1, gr[w - 1], sr[w - 1]);
	}
	LOG("  GPU path:   clean source %llu us + flush dest %llu us + GPU %llu us + drop dest %llu us = %llu us a frame",
	    t_src_clean, t_dst_flush, t_gpu, t_inv, t_src_clean + t_dst_flush + t_gpu + t_inv);
	OSSafeReleaseNULL(a);
	OSSafeReleaseNULL(b);
}

/*
 * The test GPU virtual memory exists for: 256 KB of ordinary kernel memory, allocated without
 * asking for contiguity (so scattered over physical pages the way iOS surfaces are), mapped
 * page by page at one contiguous GPU address, filled by the 2D blitter through that address and
 * read back by the CPU through its own mapping. The identity bank could never do this.
 */
#define MMU_TEST_VA         0x10000000ull       /* below RAM: no overlap with the identity blocks */

static bool
mmu_test(void)
{
	IOBufferMemoryDescriptor *m = IOBufferMemoryDescriptor::inTaskWithOptions(kernel_task, kIODirectionInOut,
	    DRAW_W * DRAW_H * 4, 4096);
	uint32_t runs = 0, good = 0, bad_at = 0xFFFFFFFF, bad_val = 0;
	uint64_t prev = 0, lo = ~0ull, hi = 0;
	const uint32_t rgba = 0x20C0FFFFu;
	bool ok = true;

	LOG("--- gpu_mmu: %u KB of scattered memory at GPU address %llx", DRAW_W * DRAW_H * 4 / 1024, MMU_TEST_VA);
	if (m == NULL) {
		LOG("  no memory");
		return false;
	}
	uint32_t *cpu = (uint32_t *)m->getBytesNoCopy();
	for (uint32_t off = 0; ok && off < DRAW_W * DRAW_H * 4; off += 4096) {
		uint64_t len = 0;
		uint64_t pa = m->getPhysicalSegment(off, &len, kIOMemoryMapperNone);
		if (pa == 0) {
			ok = false;
			break;
		}
		if (pa != prev + 4096) {
			runs++;
		}
		prev = pa;
		lo = pa < lo ? pa : lo;
		hi = pa > hi ? pa : hi;
		ok = map_page_4k(MMU_TEST_VA + off, pa);
	}
	if (!ok || !tlb_flush()) {
		LOG("  mapping failed (%u tables used)", pt_used);
		m->release();
		return false;
	}
	LOG("  %u pages in %u physical runs, between %llx and %llx; %u page tables in use", DRAW_W * DRAW_H * 4 / 4096,
	    runs, lo, hi + 4095, pt_used);
	for (uint32_t i = 0; i < DRAW_W * DRAW_H; i++) {
		cpu[i] = 0xDEADBEEF;
	}
	sweet_dcache_flush((vm_offset_t)cpu, DRAW_W * DRAW_H * 4);  /* in RAM, nothing dirty left behind */

	blit_begin(rgba);
	blit_rect(MMU_TEST_VA, DRAW_PITCH, 0, 0, DRAW_W - 1, DRAW_H - 1);
	ok = blit_end("2D fill through the GPU MMU");
	sweet_dcache_flush((vm_offset_t)cpu, DRAW_W * DRAW_H * 4);  /* drop any line the CPU still holds */
	for (uint32_t i = 0; ok && i < DRAW_W * DRAW_H; i++) {
		if (cpu[i] == __builtin_bswap32(rgba)) {
			good++;
		} else if (bad_at == 0xFFFFFFFF) {
			bad_at = i;
			bad_val = cpu[i];
		}
	}
	LOG("  pixels: %u of %u as expected (%08x); first %08x, last %08x", good, DRAW_W * DRAW_H,
	    __builtin_bswap32(rgba), cpu[0], cpu[DRAW_W * DRAW_H - 1]);
	if (bad_at != 0xFFFFFFFF) {
		LOG("  first wrong pixel: #%u (page %u, x %u, y %u) = %08x", bad_at, bad_at * 4 / 4096, bad_at % DRAW_W,
		    bad_at / DRAW_W, bad_val);
	}
	smmu_fault("after the MMU test");
	m->release();
	return ok && good == DRAW_W * DRAW_H;
}

/*
 * boot-arg gpu_3d=1 (needs gpu_mmu=1): the first 3D draw, replayed. Mesa's Vulkan driver
 * (turnip), run on a PC under drm-shim impersonating this very a610, recorded one submit:
 * a clear to dark blue, one textured quad blended source-over (a CoreAnimation layer, in
 * short) and a blit of the result to a second image. metal/tools/rd2replay.py turned that
 * capture into SapphireReplay3D.h: every buffer at the GPU address turnip gave it (from
 * 0x30000000, below RAM, free in our page tables), the non-zero pages and the command
 * stream. Here those buffers get real pages, the texture is generated (x in red, y in
 * green), the stream runs as an indirect buffer, and the CPU checks every pixel of both
 * images. Then the 2D blitter puts the picture on the panel so it can be seen.
 */
#include "SapphireReplay3D.h"

#define RP_RT       0x30026000u                 /* render target, BGRA8 256 x 256, pitch 1024 */
#define RP_TX       0x30066000u                 /* texture, RGBA8 256 x 256 */
#define RP_DS       0x300a6000u                 /* blit destination */
#define RP_W        256u
#define RP_PITCH    1024u
#define CP_INDIRECT_BUFFER 0x3f

static uint32_t draw3d;

static uint32_t
rp_check(const uint32_t *img, uint32_t *bad_x, uint32_t *bad_y, uint32_t *bad_v, uint32_t *want)
{
	uint32_t good = 0;

	*bad_x = 0xFFFFFFFF;
	for (uint32_t y = 0; y < RP_W; y++) {
		for (uint32_t x = 0; x < RP_W; x++) {
			uint32_t v = img[y * RP_W + x], ok, w;
			if (x >= 64 && x < 192 && y >= 64 && y < 192) {
				/* bilinear at texel 2(x-64)+0.5: red and green within one of 2(x-64) */
				uint32_t r = (v >> 16) & 0xff, g = (v >> 8) & 0xff, er = 2 * (x - 64), eg = 2 * (y - 64);
				w = 0xff000000u | (er << 16) | (eg << 8);
				ok = (v >> 24) == 0xff && (v & 0xff) == 0 && r - er <= 1 && g - eg <= 1;
			} else {
				w = 0xff000080u;                /* the clear: (0, 0, 0.5, 1) as B, G, R, A bytes */
				ok = v == w;
			}
			if (ok) {
				good++;
			} else if (*bad_x == 0xFFFFFFFF) {
				*bad_x = x;
				*bad_y = y;
				*bad_v = v;
				*want = w;
			}
		}
	}
	return good;
}

static bool
replay_3d(void)
{
	const uint32_t span = rp3d_VA_END - rp3d_VA_START;
	IOBufferMemoryDescriptor *m = IOBufferMemoryDescriptor::inTaskWithOptions(kernel_task, kIODirectionInOut,
	    span, 4096);
	uint32_t bx, by, bv, bw;
	bool ok = true;

	LOG("--- gpu_3d: turnip's clear + textured quad + blit, %u KB of buffers at GPU address %x, %u command streams",
	    span / 1024, rp3d_VA_START, (unsigned)(sizeof(rp3d_cmds) / sizeof(rp3d_cmds[0])));
	if (m == NULL) {
		LOG("  no memory");
		return false;
	}
	uint8_t *cpu = (uint8_t *)m->getBytesNoCopy();
	bzero(cpu, span);
	const uint32_t *run = rp3d_page_runs;
	for (uint32_t p = 0; p < sizeof(rp3d_page_va) / sizeof(rp3d_page_va[0]); p++) {
		uint32_t *pg = (uint32_t *)(cpu + (rp3d_page_va[p] - rp3d_VA_START));
		while (*run != 0xFFFFFFFFu) {
			uint32_t at = run[0], n = run[1];
			memcpy(pg + at, run + 2, n * 4);
			run += 2 + n;
		}
		run++;
	}
	uint32_t *tx = (uint32_t *)(cpu + (RP_TX - rp3d_VA_START));
	uint32_t *rt = (uint32_t *)(cpu + (RP_RT - rp3d_VA_START));
	uint32_t *ds = (uint32_t *)(cpu + (RP_DS - rp3d_VA_START));
	for (uint32_t y = 0; y < RP_W; y++) {
		for (uint32_t x = 0; x < RP_W; x++) {
			tx[y * RP_W + x] = 0xff000000u | (y << 8) | x;
			rt[y * RP_W + x] = 0xDEADBEEF;
			ds[y * RP_W + x] = 0xDEADBEEF;
		}
	}
	for (uint32_t off = 0; ok && off < span; off += 4096) {
		uint64_t len = 0, pa = m->getPhysicalSegment(off, &len, kIOMemoryMapperNone);
		ok = pa != 0 && map_page_4k(rp3d_VA_START + off, pa);
	}
	if (!ok || !tlb_flush()) {
		LOG("  mapping failed (%u tables used)", pt_used);
		m->release();
		return false;
	}
	sweet_dcache_flush((vm_offset_t)cpu, span);

	for (uint32_t i = 0; i < sizeof(rp3d_cmds) / sizeof(rp3d_cmds[0]); i++) {
		emit(pkt7(CP_INDIRECT_BUFFER, 3));
		emit(rp3d_cmds[i].va);
		emit(0);
		emit(rp3d_cmds[i].dwords);
	}
	emit(pkt7(CP_WAIT_FOR_IDLE, 0));
	event(EV_CACHE_FLUSH_TS, true);
	emit(pkt7(CP_WAIT_FOR_IDLE, 0));
	uint64_t t0 = mach_absolute_time();
	ok = submit("3D replay", 500);
	uint64_t ns;
	absolutetime_to_nanoseconds(mach_absolute_time() - t0, &ns);
	sweet_dcache_flush((vm_offset_t)cpu, span);
	LOG("  submit %s in %llu us; RBBM_STATUS %08x CP_HW_FAULT %08x RBBM_INT_0_STATUS %08x", ok ? "done" : "STUCK",
	    ns / 1000, grd(REG_RBBM_STATUS), grd(REG_CP_HW_FAULT), grd(REG_RBBM_INT_0_STATUS));
	smmu_fault("after the 3D replay");
	LOG("  render target: (0,0) %08x (128,128) %08x (191,64) %08x (255,255) %08x", rt[0], rt[128 * RP_W + 128],
	    rt[64 * RP_W + 191], rt[RP_W * RP_W - 1]);
	uint32_t good_rt = rp_check(rt, &bx, &by, &bv, &bw);
	LOG("  render target: %u of %u pixels right", good_rt, RP_W * RP_W);
	if (bx != 0xFFFFFFFF) {
		LOG("  first wrong: (%u,%u) = %08x, wanted %08x", bx, by, bv, bw);
	}
	uint32_t good_ds = rp_check(ds, &bx, &by, &bv, &bw);
	LOG("  blit copy:     %u of %u pixels right", good_ds, RP_W * RP_W);
	if (bx != 0xFFFFFFFF) {
		LOG("  first wrong: (%u,%u) = %08x, wanted %08x", bx, by, bv, bw);
	}

	/* on the panel, centred, for ten seconds (SpringBoard's next frame covers it) */
	PE_Video video;
	PE_current_console(&video);
	if (ok && video.v_baseAddr != 0 && video.v_width >= RP_W && video.v_height >= RP_W) {
		uint64_t at = video.v_baseAddr + (uint64_t)((video.v_height - RP_W) / 2) * video.v_rowBytes +
		    ((video.v_width - RP_W) / 2) * 4u;
		for (int i = 0; i < 20; i++) {
			if (!blit_copy(RP_RT, RP_PITCH, at, (uint32_t)video.v_rowBytes, RP_W, RP_W)) {
				break;
			}
			IOSleep(500);
		}
	}
	for (uint32_t off = 0; off < span; off += 4096) {
		unmap_page_4k(rp3d_VA_START + off);     /* no GPU address may outlive its page */
	}
	(void)tlb_flush();
	m->release();
	return ok && good_rt == RP_W * RP_W && good_ds == RP_W * RP_W;
}

/*
 * The GPU as a service (boot-arg gpu_service=1, needs gpu_mmu=1): after its tests the GPU is
 * not parked, and the rest of the kernel may hand it copies through sapphire_gpu_copy(). One
 * lock covers the ring, the page tables and the windows. If a copy ever fails or times out the
 * service turns itself off and every caller falls back to the CPU from then on.
 *
 * Sources are IOMemoryDescriptors (iOS surfaces: scattered pages), mapped page by page into
 * one of four 16 MB windows of GPU addresses from 0x20000000, kept while the same memory comes
 * back (same descriptor, same length, same first and last physical page) and replaced round
 * robin otherwise. Destinations are physically contiguous below 4 GB (the panel's flip
 * buffers) and are mapped at their own physical address, once.
 */
#define WIN_BASE            0x20000000ull
#define MSM_VA_START        0x60000000ull       /* the msm client's buffers (below): 2 GB, nothing else there */
#define MSM_VA_END          0xE0000000ull
#define WIN_SIZE            0x01000000ull       /* 16 MB: a 1125 x 2436 surface is 10.9 MB */
#define WIN_COUNT           4u

static uint32_t service;
/* service_ready: defined at the top (submit() reads it) */
static IOLock *service_lock;
static struct {
	IOMemoryDescriptor *md;
	uint64_t len, first, last;
	uint32_t fence;                         /* 0153: the shared seqno of the last copy that read it */
} win[WIN_COUNT];

/* the shared slot's seqno has reached f (the copies' own events) */
static bool
fence_done(uint32_t f)
{
	return (int32_t)(*(volatile uint32_t *)(pool + OFF_FENCE) - f) >= 0;
}
static uint32_t win_next;
static struct {
	uint64_t base, size;
} ident[4];
static uint32_t ident_used;

static bool
map_identity(uint64_t base, uint64_t size)
{
	for (uint32_t i = 0; i < ident_used; i++) {
		if (base >= ident[i].base && base + size <= ident[i].base + ident[i].size) {
			return true;
		}
	}
	if (ident_used == 4 || base + size > (1ull << 32) || (base < MSM_VA_END && base + size > MSM_VA_START)) {
		return false;                           /* (the msm client's addresses are never identity) */
	}
	for (uint64_t a = base & ~0xFFFull; a < base + size; a += 4096) {
		uint64_t *l2 = pt_next(&pt_l1[(a >> 30) & 3]);
		if (l2 == NULL) {
			return false;
		}
		uint64_t e = l2[(a >> 21) & 511];
		if ((e & 3) == PTE_TYPE_BLOCK && (e & PTE_ADDR_MASK & ~0x1FFFFFull) == (a & ~0x1FFFFFull)) {
			continue;                       /* already inside an identity block */
		}
		if (!map_page_4k(a, a)) {
			return false;
		}
	}
	ident[ident_used].base = base;
	ident[ident_used].size = size;
	ident_used++;
	return tlb_flush();
}

static uint64_t
map_window(IOMemoryDescriptor *md)
{
	uint64_t len = md->getLength(), first, last, seg;

	if (len == 0 || len > WIN_SIZE) {
		return 0;
	}
	/*
	 * 0164: a window holds its memory: retained and prepared (surfaces are pageable now, and the
	 * VM must not move or take the pages the GPU reads) until another surface takes the slot.
	 */
	for (uint32_t i = 0; i < WIN_COUNT; i++) {
		if (win[i].md == md && win[i].len == len) {
			return WIN_BASE + i * WIN_SIZE;
		}
	}
	uint32_t i = win_next++ % WIN_COUNT;
	uint64_t va = WIN_BASE + i * WIN_SIZE;
	/* 0153: copies are not waited for (0152): a window still being read is not remapped under it */
	for (uint32_t t = 0; win[i].fence != 0 && !fence_done(win[i].fence) && t < 20000; t++) {
		if (grd(REG_CP_RB_RPTR) == ring_wptr % RING_DWORDS) {
			break;                          /* (the CP is past it: the slot was overwritten, as above) */
		}
		IODelay(10);
	}
	if (win[i].md != NULL) {
		win[i].md->complete();
		win[i].md->release();
		win[i].md = NULL;
	}
	if (md->prepare() != kIOReturnSuccess) {
		return 0;
	}
	md->retain();
	win[i].md = md;
	first = md->getPhysicalSegment(0, &seg, kIOMemoryMapperNone);
	last = md->getPhysicalSegment((len - 1) & ~0xFFFull, &seg, kIOMemoryMapperNone);
	for (uint64_t off = 0; off < WIN_SIZE; off += 4096) {
		uint64_t pa = off < len ? md->getPhysicalSegment(off, &seg, kIOMemoryMapperNone) : 0;
		if ((off < len && pa == 0) || (pa != 0 && !map_page_4k(va + off, pa))) {
			md->complete();
			md->release();
			win[i].md = NULL;
			return 0;
		}
		if (pa == 0) {
			/* past the end: no stale page from a previous surface stays reachable */
			uint64_t *l2 = pt_next(&pt_l1[((va + off) >> 30) & 3]);
			uint64_t *l3 = l2 != NULL ? pt_next(&l2[((va + off) >> 21) & 511]) : NULL;
			if (l3 != NULL) {
				l3[((va + off) >> 12) & 511] = 0;
			}
		}
	}
	win[i].md = md;
	win[i].len = len;
	win[i].first = first;
	win[i].last = last;
	return tlb_flush() ? va : 0;
}

/*
 * Copy w x h RGBA8 pixels from src (offset src_off, src_pitch bytes a row) to dst_pa (dst_pitch a
 * row), inside the contiguous destination [dst_base, dst_base + dst_size). The caller has cleaned
 * the source rows from the CPU caches and invalidates the destination rows after. Returns false
 * to have the caller copy with the CPU instead.
 */
bool sapphire_gpu_copy(IOMemoryDescriptor *src, uint64_t src_off, uint32_t src_pitch, uint64_t dst_pa,
    uint32_t dst_pitch, uint32_t w, uint32_t h, uint64_t dst_base, uint64_t dst_size);
bool sapphire_gpu_ready(void);
bool sapphire_gpu_copy_pending(void);

/*
 * 0212: whether the last copy into the panel's memory is still in the GPU's queue (SweetFramebuffer:
 * the screen is made black at display off only once it is done, or the copy lands over the black).
 */
static uint32_t last_copy_fence;

bool
sapphire_gpu_copy_pending(void)
{
	return service_ready && last_copy_fence != 0 && pool != NULL && !fence_done(last_copy_fence);
}

/*
 * 0340, for SweetFramebuffer's frame queue, which waits for each copy by its own fence: the fence of the last copy
 * handed to the CP (0: none yet), and whether a given one is done. A GPU that takes no more work has nothing pending
 * (as sapphire_gpu_copy_pending() has it).
 */
uint32_t sapphire_gpu_copy_fence(void);
bool sapphire_gpu_fence_done(uint32_t fence);

uint32_t
sapphire_gpu_copy_fence(void)
{
	return last_copy_fence;
}

bool
sapphire_gpu_fence_done(uint32_t fence)
{
	return !service_ready || fence == 0 || pool == NULL || fence_done(fence);
}

/* So a caller does not pay for cache maintenance before a copy the GPU would refuse. */
bool
sapphire_gpu_ready(void)
{
	return service_ready;
}

/*
 * 0336: a panel copy is about to be emitted (the ring's lock held). Counted; with kern.sweet_gpu_marks its start mark
 * goes in here and blit_end() adds its end mark.
 */
static uint32_t
gdiag_copy_begin(void)
{
	uint32_t mark = ~0u;

	gdiag.copy_n++;
	if (last_copy_fence != 0 && !fence_done(last_copy_fence)) {
		gdiag.copy_behind++;                    /* the copy before this one has not run yet */
	}
	if (msm_seq != 0 && (int32_t)(*(volatile uint32_t *)(pool + OFF_MSM_FENCE) - msm_seq) < 0) {
		gdiag.copy_gpu_busy++;                  /* drawing is still queued or running ahead of it */
	}
	if (gpu_marks) {
		gdiag_harvest();
		mark = gdiag_copy_head % GDIAG_SLOTS;
		volatile uint64_t *m = (volatile uint64_t *)(pool + OFF_DIAG + (GDIAG_SLOTS + mark) * 16u);
		m[0] = m[1] = 0;
		gdiag_mark(POOL_PA + OFF_DIAG + (GDIAG_SLOTS + mark) * 16u);
		gdiag_end_pa = POOL_PA + OFF_DIAG + (GDIAG_SLOTS + mark) * 16u + 8;
	}
	return mark;
}

static void
gdiag_copy_end(uint32_t mark)
{
	gdiag_end_pa = 0;                               /* (blit_end() used it, unless the blit was refused) */
	if (mark != ~0u) {
		gdiag_copy_fence[mark] = fence_seq;
		gdiag_copy_head++;
	}
}

bool
sapphire_gpu_copy(IOMemoryDescriptor *src, uint64_t src_off, uint32_t src_pitch, uint64_t dst_pa,
    uint32_t dst_pitch, uint32_t w, uint32_t h, uint64_t dst_base, uint64_t dst_size)
{
	static uint32_t told;
	bool ok = false;

	if (!service_ready || src == NULL || w == 0 || h == 0) {
		return false;
	}
	uint64_t diag_t0 = mach_absolute_time();        /* 0336: how long the copy waits for the ring's lock */
	IOLockLock(service_lock);
	gdiag_since(&gdiag.copy_lock, diag_t0);
	if (service_ready) {
		uint64_t va = map_window(src);
		bool dst_ok = map_identity(dst_base, dst_size);
		if (!ring_room(1024, 200)) {
			LOG("service: the ring stays full (the GPU stopped?): copy refused");
			IOLockUnlock(service_lock);
			return false;
		}
		if (va != 0 && dst_ok && src_off + (uint64_t)(h - 1) * src_pitch + w * 4u <= src->getLength()) {
			uint32_t mark = gdiag_copy_begin();
			ok = blit_copy(va + src_off, src_pitch, dst_pa, dst_pitch, w, h);
			gdiag_copy_end(mark);
			uint32_t wi = (uint32_t)((va - WIN_BASE) / WIN_SIZE);
			if (wi < WIN_COUNT) {
				win[wi].fence = fence_seq;      /* blit_end's last event */
			}
			last_copy_fence = fence_seq;
			if (!ok) {
				service_ready = false;
				LOG("service: a copy failed, the GPU path is off; RBBM_STATUS %08x", grd(REG_RBBM_STATUS));
				smmu_fault("service");
			}
		} else if (told++ < 4) {
			LOG("service: cannot map this copy (window %llx, destination %s, %ux%u at +%llx of %llu bytes)", va,
			    dst_ok ? "ok" : "NOT mapped", w, h, src_off, src->getLength());
		}
	}
	IOLockUnlock(service_lock);
	return ok;
}

/* 0239: sapphire_gpu_copy() scaled: blit_scale() of the source at src_off (coordinates << 8) */
bool sapphire_gpu_copy_scaled(IOMemoryDescriptor *src, uint64_t src_off, uint32_t src_pitch, uint32_t sx, uint32_t sy,
    uint32_t sw, uint32_t sh, uint64_t dst_pa, uint32_t dst_pitch, uint32_t dw, uint32_t dh, uint64_t dst_base,
    uint64_t dst_size);

bool
sapphire_gpu_copy_scaled(IOMemoryDescriptor *src, uint64_t src_off, uint32_t src_pitch, uint32_t sx, uint32_t sy,
    uint32_t sw, uint32_t sh, uint64_t dst_pa, uint32_t dst_pitch, uint32_t dw, uint32_t dh, uint64_t dst_base,
    uint64_t dst_size)
{
	static uint32_t told;
	bool ok = false;
	uint64_t last = src_off + (uint64_t)(((sy + sh) >> 8) + 1) * src_pitch;

	if (!service_ready || src == NULL || dw == 0 || dh == 0) {
		return false;
	}
	uint64_t diag_t0 = mach_absolute_time();        /* 0336 */
	IOLockLock(service_lock);
	gdiag_since(&gdiag.copy_lock, diag_t0);
	if (service_ready) {
		uint64_t va = map_window(src);
		bool dst_ok = map_identity(dst_base, dst_size);
		if (!ring_room(1024, 200)) {
			LOG("service: the ring stays full (the GPU stopped?): copy refused");
			IOLockUnlock(service_lock);
			return false;
		}
		if (va != 0 && dst_ok && last <= src->getLength()) {
			uint32_t mark = gdiag_copy_begin();
			ok = blit_scale(va + src_off, src_pitch, sx, sy, sw, sh, dst_pa, dst_pitch, dw, dh);
			gdiag_copy_end(mark);
			uint32_t wi = (uint32_t)((va - WIN_BASE) / WIN_SIZE);
			if (wi < WIN_COUNT) {
				win[wi].fence = fence_seq;
			}
			last_copy_fence = fence_seq;
			if (!ok) {
				service_ready = false;
				LOG("service: a scaled copy failed, the GPU path is off; RBBM_STATUS %08x", grd(REG_RBBM_STATUS));
				smmu_fault("service");
			}
		} else if (told++ < 4) {
			LOG("service: cannot map this scaled copy (window %llx, destination %s, to %ux%u at +%llx of %llu bytes)", va,
			    dst_ok ? "ok" : "NOT mapped", dw, dh, src_off, src->getLength());
		}
	}
	IOLockUnlock(service_lock);
	return ok;
}

/*
 * boot-arg gpu_cs=1: the first shader on the Adreno — a compute kernel of 64 invocations, each
 * storing 0xc0de0000 + its local invocation id at buf[id], the buffer's GPU address in c0.xy.
 * Source metal/ir3/shaders/first.asm, assembled for the a610 by metal/ir3/sapphire-ir3as (Mesa's
 * own ir3 assembler and encoder): 128 bytes, max_reg 2 (3 full registers), no half registers,
 * constlen 4 vec4, mergedregs, double_threadsize (THREAD128), branchstack 0, local id in r0.x.
 * The launch is Mesa's computerator a6xx.cc (cs_restore_emit, cs_program_emit_regs,
 * cs_program_emit, emit_const, a6xx_emit_grid) for a GPU without LPAC; offsets and fields from
 * a6xx.xml (metal/tools/a6xx-reg.py).
 */
static const uint32_t first_cs[32] = {
	0x00000000, 0x202cc004, 0x00000001, 0x202cc005, 0xc0de0000, 0x204cc008, 0x00000000, 0x00000500,
	0x00000008, 0x42100008, 0x00000000, 0x00000500, 0x01800010, 0xc0d60900, 0x00000000, 0x03000000,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};
static uint32_t cs_test;

#define REG_SP_PERFCTR_SHADER_MASK  0xae0f
#define REG_SP_NC_MODE_CNTL_2       0xae04
#define REG_SP_MODE_CNTL            0xab00
#define REG_SP_UPDATE_CNTL          0xbb08
#define REG_SP_CS_CONST_CONFIG      0xb987
#define REG_SP_CS_CONFIG            0xa9bb
#define REG_SP_CS_INSTR_SIZE        0xa9bc
#define REG_SP_CS_CNTL_0            0xa9b0
#define REG_SP_CS_CNTL_1            0xa9b1
#define REG_SP_CS_CONST_CONFIG_0    0xb997
#define REG_SP_CS_WGE_CNTL          0xb998
#define REG_SP_CS_BASE              0xa9b4
#define REG_SP_CS_NDRANGE_0         0xb990      /* NDRANGE_0..6 consecutive */
#define REG_SP_CS_KERNEL_GROUP_X    0xb999      /* X, Y, Z consecutive */
#define CP_LOAD_STATE6_FRAG         0x34
#define CP_EXEC_CS                  0x33
#define RM6_COMPUTE                 8
#define INVALID_REG                 0xfcu

static bool
cs_first(void)
{
	uint64_t code = POOL_PA + OFF_CS, out = POOL_PA + OFF_CSOUT;
	volatile uint32_t *res = (volatile uint32_t *)(pool + OFF_CSOUT);
	uint32_t good = 0, bad_at = 0xFFFFFFFF, bad_val = 0;

	LOG("--- gpu_cs: the first shader, 64 invocations writing to %llx", out);
	memcpy(pool + OFF_CS, first_cs, sizeof(first_cs));
	for (int i = 0; i < 64; i++) {
		res[i] = 0xDEADBEEF;
	}
	gmb();

	/* cs_restore_emit: shader mask, NC mode, the a610's magic registers */
	reg1(REG_SP_PERFCTR_SHADER_MASK, 1u << 5);
	reg1(REG_SP_NC_MODE_CNTL_2, 0);
	reg1(0x9804, 0xf);                          /* PC_MODE_CNTL */
	reg1(0x9805, 0);                            /* PC_POWER_CNTL */
	reg1(0xa0f8, 0);                            /* VFD_POWER_CNTL */
	reg1(0xb600, 0);                            /* TPL1_DBG_ECO_CNTL */
	reg1(0x8600, 0);                            /* GRAS_DBG_ECO_CNTL */
	reg1(0xae03, 0);                            /* SP_CHICKEN_BITS */
	reg1(0xae00, 0);                            /* SP_DBG_ECO_CNTL */
	reg1(0xbe04, 0);                            /* HLSQ_DBG_ECO_CNTL */
	reg1(0x9600, 0);                            /* VPC_DBG_ECO_CNTL */
	reg1(0x0e12, 0x10000000);                   /* UCHE_UNKNOWN_0E12 */

	/* cs_program_emit_regs */
	reg1(REG_SP_MODE_CNTL, 1u | (2u << 1));     /* constant demotion, ISAMMODE_GL */
	reg1(REG_SP_UPDATE_CNTL, 0x3Fu | (1u << 7)); /* every stage's state, GFX_UAV */
	reg1(REG_SP_CS_CONST_CONFIG, (4u >> 2) | (1u << 8));  /* constlen 4 vec4 (field shr 2), enabled */
	reg1(REG_SP_CS_CONFIG, 1u << 8);            /* enabled, no textures, samplers or UAVs */
	reg1(REG_SP_CS_INSTR_SIZE, 1);              /* instrlen */
	reg1(REG_SP_CS_CNTL_0, (0u << 1) | (3u << 7) | (0u << 13) | (1u << 20) | (1u << 31));
	                                            /* half 0, full 3, branchstack 0, THREAD128, mergedregs */
	reg1(REG_SP_CS_CNTL_1, 1u);                 /* shared_size 1, CONSTLEN_128 */
	reg1(REG_SP_CS_CONST_CONFIG_0, INVALID_REG | (INVALID_REG << 8) | (INVALID_REG << 16) | (0u << 24));
	                                            /* no wg id / size / offset consts, local id in r0.x */
	reg1(REG_SP_CS_WGE_CNTL, INVALID_REG | (1u << 9));  /* no linear local id, THREAD128 */
	emit(pkt4(REG_SP_CS_BASE, 2));
	emit((uint32_t)code);
	emit((uint32_t)(code >> 32));
	reg1(REG_SP_CS_INSTR_SIZE, 1);

	/* cs_program_emit: preload the code (instrlen 1, within the 64-entry cache) */
	emit(pkt7(CP_LOAD_STATE6_FRAG, 3));
	emit(0u | (0u << 14) | (2u << 16) | (0xDu << 18) | (1u << 22));  /* ST6_SHADER, SS6_INDIRECT, SB6_CS_SHADER, 1 unit */
	emit((uint32_t)code);
	emit((uint32_t)(code >> 32));

	/* emit_const: c0 = the buffer's address */
	emit(pkt7(CP_LOAD_STATE6_FRAG, 3 + 4));
	emit(0u | (1u << 14) | (0u << 16) | (0xDu << 18) | (1u << 22));  /* ST6_CONSTANTS, SS6_DIRECT, SB6_CS_SHADER, 1 vec4 */
	emit(0);
	emit(0);
	emit((uint32_t)out);
	emit((uint32_t)(out >> 32));
	emit(0);
	emit(0);

	/* a6xx_emit_grid: one group of 64 */
	emit(pkt7(CP_SET_MARKER, 1));
	emit(RM6_COMPUTE);
	emit(pkt4(REG_SP_CS_NDRANGE_0, 7));
	emit(1u | (63u << 2) | (0u << 12) | (0u << 22));  /* kerneldim 1, local size 64 x 1 x 1 */
	emit(64);                                   /* global size x */
	emit(0);
	emit(1);                                    /* global size y */
	emit(0);
	emit(1);                                    /* global size z */
	emit(0);
	emit(pkt4(REG_SP_CS_KERNEL_GROUP_X, 3));
	emit(1);
	emit(1);
	emit(1);
	emit(pkt7(CP_EXEC_CS, 4));
	emit(0);
	emit(1);
	emit(1);
	emit(1);
	emit(pkt7(CP_WAIT_FOR_IDLE, 0));
	event(EV_CACHE_FLUSH_TS, true);
	emit(pkt7(CP_WAIT_FOR_IDLE, 0));
	if (!submit("compute", 200)) {
		return false;
	}
	IODelay(100);
	gmb();
	for (uint32_t i = 0; i < 64; i++) {
		if (res[i] == 0xC0DE0000u + i) {
			good++;
		} else if (bad_at == 0xFFFFFFFF) {
			bad_at = i;
			bad_val = res[i];
		}
	}
	LOG("  results: [0] %08x [1] %08x [63] %08x; %u of 64 right", res[0], res[1], res[63], good);
	if (bad_at != 0xFFFFFFFF) {
		LOG("  first wrong: [%u] = %08x", bad_at, bad_val);
	}
	LOG("  RBBM_STATUS %08x CP_HW_FAULT %08x RBBM_INT_0_STATUS %08x", grd(REG_RBBM_STATUS), grd(REG_CP_HW_FAULT),
	    grd(REG_RBBM_INT_0_STATUS));
	return good == 64;
}

/*
 * boot-arg gpu_demo=1: the GPU paints three bars (red, green, blue) straight onto the boot
 * framebuffer the panel scans out, over the Apple logo, where they stay until SpringBoard's first
 * frame replaces them. The framebuffer is 1080 x 2400 BGRA with 4320 bytes a row, which the
 * blitter cannot take as a pitch (it counts in 64-byte units and 4320 is 67.5 of them), but two
 * rows are 8640 = 135 x 64: so the even rows are one surface with an 8640-byte pitch, and the odd
 * rows another one starting 4320 bytes later, whose base is 32 bytes off alignment, so it is
 * rounded down and x shifted right by 8 pixels, the trick fd6_clear_buffer uses for unaligned
 * buffers. Two blits per bar.
 */
static uint32_t demo;

static bool
blit_bar(uint64_t fb, uint32_t stride, uint32_t w, uint32_t y0, uint32_t h, uint32_t r, uint32_t g, uint32_t b)
{
	/* the framebuffer is B,G,R,A in memory: channel order of the blit format is R,G,B,A */
	uint32_t rgba = (b << 24) | (g << 16) | (r << 8) | 0xFF;
	uint64_t even = fb + (uint64_t)y0 * stride;
	uint64_t odd = even + stride;
	uint32_t dx = (uint32_t)(odd & 63) / 4;

	blit_begin(rgba);
	blit_rect(even, 2 * stride, 0, 0, w - 1, h / 2 - 1);
	blit_rect(odd & ~63ull, 2 * stride, dx, 0, dx + w - 1, h / 2 - 1);
	return blit_end("bar");
}

static void
demo_bars(void)
{
	PE_Video video;

	PE_current_console(&video);
	uint64_t fb = video.v_baseAddr;
	uint32_t w = (uint32_t)video.v_width, h = (uint32_t)video.v_height, stride = (uint32_t)video.v_rowBytes;

	LOG("--- gpu_demo: bars onto the framebuffer at %llx, %ux%u, %u bytes a row", fb, w, h, stride);
	if (fb == 0 || (fb & 63) != 0 || ((2 * stride) & 63) != 0 || (stride & 3) != 0 || w > 0x3ff0 || h < 1400) {
		LOG("  this framebuffer does not fit the two-surface trick; no demo");
		return;
	}
	uint32_t bar = 200, top = h / 2 - 3 * bar / 2;
	top &= ~1u;                                  /* even first row */
	bool ok = blit_bar(fb, stride, w, top, bar, 0xFF, 0x20, 0x20) &&
	    blit_bar(fb, stride, w, top + bar, bar, 0x20, 0xE0, 0x20) &&
	    blit_bar(fb, stride, w, top + 2 * bar, bar, 0x20, 0x40, 0xFF);
	LOG("  %s: rows %u-%u red, green, blue", ok ? "drawn" : "FAILED", top, top + 3 * bar - 1);

	/* read a pixel of every bar on an even and an odd row back through the CPU */
	IOMemoryMap *m = map_phys(fb, (uint64_t)stride * h, true);
	if (m == NULL) {
		return;
	}
	const uint8_t *v = (const uint8_t *)m->getVirtualAddress();
	for (uint32_t i = 0; i < 3; i++) {
		uint32_t ye = top + i * bar + 10, yo = ye + 1;
		LOG("  bar %u: pixel (%u,%u) %08x, (%u,%u) %08x, (0,%u) %08x, (%u,%u) %08x", i, w / 2, ye,
		    *(const volatile uint32_t *)(v + (uint64_t)ye * stride + (w / 2) * 4), w / 2, yo,
		    *(const volatile uint32_t *)(v + (uint64_t)yo * stride + (w / 2) * 4), yo,
		    *(const volatile uint32_t *)(v + (uint64_t)yo * stride), w - 1, yo,
		    *(const volatile uint32_t *)(v + (uint64_t)yo * stride + (w - 1) * 4));
	}
	m->release();
}

/* ---------------------------------------------------------------- bring-up thread */

static void
gpu_thread(void *arg __unused, wait_result_t wr __unused)
{
	uint32_t stage = 0, delay_s = 0;

	(void)PE_parse_boot_argn("gpu", &stage, sizeof(stage));
	/*
	 * gpu_delay=N waits N seconds before touching anything. The bring-up runs from
	 * AppleCLCD::start, long before USB networking carries the console, so its log would
	 * only be readable from recovery; with a delay it lands in the live log instead.
	 */
	(void)PE_parse_boot_argn("gpu_slow", &slow_ms, sizeof(slow_ms));
	/* 0152: gpu_div=1 is GPLL0 undivided, 600 MHz -- the rail is the bootloader's, so try it, do not assume it */
	(void)PE_parse_boot_argn("gpu_mhz", &core_mhz, sizeof(core_mhz));
	(void)PE_parse_boot_argn("gpu_vco2", &vco2, sizeof(vco2));           /* 0337 */
	(void)PE_parse_boot_argn("gpu_cx", &cx_arg, sizeof(cx_arg));
	(void)PE_parse_boot_argn("gpu_mx", &mx_arg, sizeof(mx_arg));
	if (PE_parse_boot_argn("gpu_div", &core_div, sizeof(core_div)) && (core_div < 1 || core_div > 16)) {
		core_div = 2;
	}
	(void)PE_parse_boot_argn("gpu_clkdiag", &clkdiag, sizeof(clkdiag));         /* 0343 */
	clk_lock = IOLockAlloc();
	(void)PE_parse_boot_argn("gpu_opp_drain", &opp_drain_ms, sizeof(opp_drain_ms));      /* 0345 */
	(void)PE_parse_boot_argn("gpu_hwcg", &gpu_hwcg, sizeof(gpu_hwcg));                   /* 0348 */
	(void)PE_parse_boot_argn("gpu_ubwc", &gpu_ubwc, sizeof(gpu_ubwc));                   /* 0370 */
	if (gpu_ubwc < 0 || gpu_ubwc > 2) {
		gpu_ubwc = 0;
	}
	(void)PE_parse_boot_argn("gpu_opp_park", &opp_park, sizeof(opp_park));
	if (opp_drain_ms == 0 || opp_drain_ms > 1000) {
		opp_drain_ms = 100;
	}
	opp_lock = IOLockAlloc();
	(void)PE_parse_boot_argn("gpu_copy_async", &copy_async, sizeof(copy_async));
	(void)PE_parse_boot_argn("gpu_smmu_early", &smmu_early, sizeof(smmu_early));
	(void)PE_parse_boot_argn("gpu_irq", &gpu_irq, sizeof(gpu_irq));
	(void)PE_parse_boot_argn("gpu_stop", &stop_at, sizeof(stop_at));
	(void)PE_parse_boot_argn("gpu_keep", &keep_running, sizeof(keep_running));
	(void)PE_parse_boot_argn("gpu_draw", &draw_first, sizeof(draw_first));
	(void)PE_parse_boot_argn("gpu_demo", &demo, sizeof(demo));
	(void)PE_parse_boot_argn("gpu_bench", &bench, sizeof(bench));
	(void)PE_parse_boot_argn("gpu_cs", &cs_test, sizeof(cs_test));
	(void)PE_parse_boot_argn("gpu_3d", &draw3d, sizeof(draw3d));
	(void)PE_parse_boot_argn("gpu_mmu", &mmu, sizeof(mmu));
	(void)PE_parse_boot_argn("gpu_service", &service, sizeof(service));
	service_lock = IOLockAlloc();
	if (PE_parse_boot_argn("gpu_delay", &delay_s, sizeof(delay_s)) && delay_s != 0) {
		LOG("starting in %u s", delay_s);
		IOSleep(delay_s * 1000);
	}
	LOG("stage %u (Adreno 610 v2, regs %llx, GPU CC %llx), core GPLL0/%u = %u MHz, %u ms after each step",
	    stage, GPU_PA, GPUCC_PA, core_div, 600 / core_div, slow_ms);
	if (core_mhz != 0) {
		/* 0337: the line above speaks of gpu_div, which only decides when gpu_mhz does not */
		LOG("gpu_mhz=%u gpu_vco2=%u: %s", core_mhz, vco2, vco2 != 0 ?
		    "gpu_mhz is the core clock; the PLL's VCO runs at twice that, as the stock driver sets it" :
		    "gpu_mhz is the PLL's VCO and the core clock is half of it (gpu_vco2=1: gpu_mhz is the core clock)");
	}

	gcc_map = map_phys(GCC_PA, GCC_SIZE, false);
	gpucc_map = map_phys(GPUCC_PA, GPUCC_SIZE, false);
	if (gcc_map == NULL || gpucc_map == NULL) {
		LOG("the clock controllers could not be mapped");
		thread_terminate(current_thread());
		return;
	}
	gcc = (volatile uint8_t *)gcc_map->getVirtualAddress();
	gpucc = (volatile uint8_t *)gpucc_map->getVirtualAddress();
	gicd_map = map_phys(GICD_PA, GICD_SIZE, false);
	if (gicd_map != NULL) {
		gicd = (volatile uint8_t *)gicd_map->getVirtualAddress();
	}
	gic_report("as the bootloader left it");
	dump_clocks("as the bootloader left it");
	if (stage < 2) {
		thread_terminate(current_thread());
		return;
	}

	if (gpu_irq == 0) {
		gic_mask_gpu();
		step_done();
	}
	/* mapped before the power-up only so gpu_smmu_early has it; the registers need the vote */
	smmu_map = map_phys(SMMU_PA, SMMU_SIZE, false);
	if (smmu_map != NULL) {
		smmu = (volatile uint8_t *)smmu_map->getVirtualAddress();
	}
	if (!power_up()) {
		LOG("power-up failed, the GPU stays off");
		power_down();
		thread_terminate(current_thread());
		return;
	}
	dump_clocks("powered");
	clocks_up = true;                           /* 0343 */
	if (clkdiag != 0) {
		char line[480];
		clock_report(line, sizeof(line));
		LOG("clocks: %s", line);
		thermal_report(line, sizeof(line));
		LOG("temperatures: %s", line);
	}
	gpu_map = map_phys(GPU_PA, GPU_SIZE, false);
	if (gpu_map == NULL) {
		LOG("the GPU registers could not be mapped");
		power_down();
		thread_terminate(current_thread());
		return;
	}
	gpu = (volatile uint8_t *)gpu_map->getVirtualAddress();
	LOG("GPU registers mapped");
	step_done();
	first_reads();
	if (stage < 3) {
		LOG("stage 2: the GPU is powered and answering; stopping here");
		thread_terminate(current_thread());
		return;
	}

	if (fw_count() == 0 && !have_adt_firmware()) {
		LOG("no firmware, in the device tree or in a table: stopping after the power-up");
		thread_terminate(current_thread());
		return;
	}
	pool_map = map_phys(POOL_PA, POOL_SIZE, true);
	if (pool_map == NULL || smmu_map == NULL) {
		LOG("the buffer pool or the SMMU could not be mapped");
		power_down();
		thread_terminate(current_thread());
		return;
	}
	pool = (uint8_t *)pool_map->getVirtualAddress();
	ring = (uint32_t *)(pool + OFF_RING);
	test_buf = (uint32_t *)(pool + OFF_TEST);
	bzero(pool, OFF_META + 0x4000);

	/* a partition carries the firmware: sweet_ufs answers EAGAIN until the root device is up */
	for (int i = 0; fw_count() != 0 && i < 120; i++) {
		uint8_t probe[16];
		int err = sweet_ufs_read_part(gpu_fw[0].part, gpu_fw[0].off, probe, sizeof(probe));
		if (err != EAGAIN) {
			if (err != 0) {
				LOG("partition %s: %d", gpu_fw[0].part, err);
				thread_terminate(current_thread());
				return;
			}
			break;
		}
		IOSleep(500);
	}

	if (stage == 3 && !zap_load()) {
		LOG("the zap shader did not load; gpu=4 tries it without one");
		thread_terminate(current_thread());
		return;
	}
	if (stage == 3 && stop_here(1, "the zap shader")) {
		;
	} else if ((recover_zap = (stage == 3)), cp_start(stage == 3)) {
		LOG("*** the GPU executes commands ***");
		if (draw_first && blit_fill(0xFF8000FF)) {
			LOG("*** the GPU drew: every pixel of the 2D fill is right ***");
			service_ok = true;
			if (demo) {
				demo_bars();
			}
			if (bench) {
				bench_copy();
			}
			if (cs_test && cs_first()) {
				LOG("*** the first shader ran on the Adreno: 64 of 64 invocations wrote their value ***");
			} else if (cs_test) {
				LOG("the compute shader did not come out right");
			}
			if (mmu && mmu_test()) {
				LOG("*** the GPU reached scattered memory through its own page tables ***");
			} else if (mmu) {
				LOG("the GPU MMU test did not come out right");
				service_ok = false;
			}
			if (mmu && draw3d && service_ok && replay_3d()) {
				LOG("*** the first 3D draw on the Adreno: clear, textured quad and blit, every pixel right ***");
			} else if (mmu && draw3d) {
				LOG("the 3D replay did not come out right");
			}
		} else if (draw_first) {
			LOG("the 2D fill did not come out right");
		}
	} else if (!stopped_early) {
		LOG("the command processor did not come up");
	}
	if (service && mmu && service_lock != NULL && service_ok) {
		service_ready = true;
		LOG("*** gpu_service: the GPU stays up and takes copies (%u page tables in use) ***", pt_used);
		msm_publish();
		gov_boot_khz = opp_khz;                 /* 0346 */
		{
			uint32_t ddr_on = 0;                /* 0347 */
			if (PE_parse_boot_argn("gpu_ddr", &ddr_on, sizeof(ddr_on)) && ddr_on != 0 && opp_khz != 0) {
				(void)ddr_mode_set(1);
			}
		}
		if (PE_parse_boot_argn("gpu_gov", &gov_on, sizeof(gov_on)) && gov_on != 0 && opp_khz != 0) {
			gov_start();
		} else {
			gov_on = 0;
		}
	} else if (keep_running || service) {
		LOG("%s: the GPU stays up%s", service ? "gpu_service" : "gpu_keep=1",
		    service ? " but takes no work (needs gpu_mmu=1 and every test passed)" : "");
	} else {
		LOG("parking the GPU: RBBM_STATUS %08x", grd(REG_RBBM_STATUS));
		bus_drain();
		power_down();
	}
	thread_terminate(current_thread());
}

/* ---------------------------------------------------------------- user client */

/*
 * The GPU for user space: what a Metal driver needs from the kernel. IOAcceleratorES (in
 * SweetFramebuffer.cpp) hands IOServiceOpen of type 'SGPU' to sapphire_gpu_new_client(); any
 * other type is refused as before, so Metal's own probes see nothing new. Works only once
 * gpu_service is up (gpu=3 gpu_mmu=1 gpu_service=1 and every test passed), and shares the
 * service lock with the frame copies: one ring, one set of page tables.
 *
 *   sel 0  info            out: gpu id, chip id, user VA base, user VA end, page tables free
 *   sel 1  buffer create   in: size, fixed GPU address (0 = any)    out: handle, GPU address
 *   sel 2  release         in: handle
 *   sel 3  surface import  in: IOSurface id                  out: handle, GPU address, size
 *   sel 4  sync            in: handle   (clean + invalidate the CPU caches over it)
 *   sel 5  submit          in: GPU address, dwords, timeout ms   out: fence seqno, us taken
 *
 * Buffers are wired kernel memory, zeroed and cleaned; map one into the task with
 * IOConnectMapMemory64(conn, handle, ..., kIOMapAnywhere), cacheable, as the IOSurface buffers
 * are: never write-combined, because changing the cache mode of kernel pages for a user
 * mapping panics this kernel (phystokv_range: illegal PA, first try of this interface).
 * Every submit cleans and invalidates all of the client's buffers before it starts and after
 * it ends, so CPU writes reach the GPU and GPU writes reach the CPU. A surface is the task's
 * own IOSurface (by id): its pixels get a GPU address and sel 4 cleans what the CPU wrote, or
 * drops what it holds before reading what the GPU wrote. The GPU sees its buffers through 4 KB pages from
 * 0x01000000 to 0x10000000, below RAM: nothing there aliases the identity blocks.
 *
 * A submit runs one indirect buffer, which must lie inside a buffer of this client, then a
 * wait for idle and a timestamped cache flush, and waits for it (no queue yet). If the GPU
 * does not finish in time, the service stops: every later call and frame copy is refused.
 *
 * This is a development interface: the command processor runs what it is given with the
 * whole GPU address space reachable, and no register protection is set up yet.
 */
#include "SweetSurface.h"

extern "C" void proc_selfname(char *buf, int size);

#define UVA_BASE            0x01000000ull
#define UVA_END             0x10000000ull
#define UVA_PAGES           ((UVA_END - UVA_BASE) >> 12)
#define UC_BOS              256u

static uint8_t uva_bits[UVA_PAGES / 8];

static bool
uva_used(uint64_t p)
{
	return (uva_bits[p >> 3] >> (p & 7)) & 1;
}

static void
uva_mark(uint64_t p0, uint64_t n, bool used)
{
	for (uint64_t p = p0; p < p0 + n; p++) {
		if (used) {
			uva_bits[p >> 3] |= (uint8_t)(1u << (p & 7));
		} else {
			uva_bits[p >> 3] &= (uint8_t)~(1u << (p & 7));
		}
	}
}

/* n pages of GPU addresses, at fixed if non-zero; 0 if there is no room. */
static uint64_t
uva_alloc(uint64_t n, uint64_t fixed)
{
	if (n == 0 || n > UVA_PAGES) {
		return 0;
	}
	if (fixed != 0) {
		if ((fixed & 0xFFF) != 0 || fixed < UVA_BASE || fixed + (n << 12) > UVA_END) {
			return 0;
		}
		uint64_t p0 = (fixed - UVA_BASE) >> 12;
		for (uint64_t i = 0; i < n; i++) {
			if (uva_used(p0 + i)) {
				return 0;
			}
		}
		uva_mark(p0, n, true);
		return fixed;
	}
	for (uint64_t p = 0, run = 0; p < UVA_PAGES; p++) {
		run = uva_used(p) ? 0 : run + 1;
		if (run == n) {
			uva_mark(p + 1 - n, n, true);
			return UVA_BASE + ((p + 1 - n) << 12);
		}
	}
	return 0;
}

struct uc_bo {
	IOMemoryDescriptor *md;                 /* retained; NULL = free slot */
	SweetSurface       *surf;               /* retained when imported */
	uint64_t            va;                 /* GPU address of the first page */
	uint64_t            pages;
	uint64_t            len;                /* bytes from va + delta */
	uint32_t            delta;              /* where byte 0 sits in the first page */
	uintptr_t           kva;                /* kernel address of byte 0, for cache maintenance */
};

/*
 * Map len bytes of md from offset 0 at fresh GPU addresses (fixed if non-zero), page by page,
 * following the physical segments. Fills va, pages and delta. service_lock held.
 */
static bool
uc_map(struct uc_bo *b, uint64_t fixed)
{
	uint64_t seg = 0, pa0 = b->md->getPhysicalSegment(0, &seg, kIOMemoryMapperNone);

	if (pa0 == 0) {
		return false;
	}
	b->delta = (uint32_t)(pa0 & 0xFFF);
	b->pages = (b->delta + b->len + 4095) >> 12;
	b->va = uva_alloc(b->pages, fixed);
	if (b->va == 0) {
		return false;
	}
	bool ok = true;
	for (uint64_t o = 0; ok && o < b->len;) {
		uint64_t pa = b->md->getPhysicalSegment(o, &seg, kIOMemoryMapperNone);
		uint64_t at = b->delta + o;             /* byte offset from the first page */
		ok = pa != 0 && map_page_4k(b->va + (at & ~0xFFFull), pa & ~0xFFFull);
		o += 4096 - (pa & 0xFFF);
	}
	if (!ok) {
		for (uint64_t p = 0; p < b->pages; p++) {
			unmap_page_4k(b->va + (p << 12));
		}
		uva_mark((b->va - UVA_BASE) >> 12, b->pages, false);
	}
	return tlb_flush() && ok;
}

static void
uc_unmap(struct uc_bo *b)
{
	for (uint64_t p = 0; p < b->pages; p++) {
		unmap_page_4k(b->va + (p << 12));
	}
	(void)tlb_flush();
	uva_mark((b->va - UVA_BASE) >> 12, b->pages, false);
}

class SapphireGpuClient : public IOUserClient {
	OSDeclareDefaultStructors(SapphireGpuClient);

public:
	task_t        fTask;
	uint32_t      fCalls;
	char          fName[32];
	struct uc_bo  fBo[UC_BOS];

	bool initWithTask(task_t owningTask, void *securityID, UInt32 type) APPLE_KEXT_OVERRIDE;
	IOReturn clientClose(void) APPLE_KEXT_OVERRIDE;
	IOReturn clientMemoryForType(UInt32 type, IOOptionBits *options, IOMemoryDescriptor **memory) APPLE_KEXT_OVERRIDE;
	IOReturn externalMethod(uint32_t selector, IOExternalMethodArguments *args, IOExternalMethodDispatch *dispatch,
	    OSObject *target, void *reference) APPLE_KEXT_OVERRIDE;

	int  freeSlot(void);
	void flushAll(void);
	void drop(uint32_t h);
	IOReturn create(uint64_t size, uint64_t fixed, uint64_t *out);
	IOReturn import(uint32_t id, uint64_t *out);
	IOReturn run(uint64_t va, uint64_t dwords, uint64_t ms, uint64_t *out);
};

OSDefineMetaClassAndStructors(SapphireGpuClient, IOUserClient);

bool
SapphireGpuClient::initWithTask(task_t owningTask, void *securityID, UInt32 type)
{
	if (!IOUserClient::initWithTask(owningTask, securityID, type)) {
		return false;
	}
	fTask = owningTask;
	fCalls = 0;
	proc_selfname(fName, sizeof(fName));          /* IOServiceOpen runs in the caller */
	bzero(fBo, sizeof(fBo));
	return true;
}

int
SapphireGpuClient::freeSlot(void)
{
	for (uint32_t i = 1; i < UC_BOS; i++) {         /* handle 0 is never used */
		if (fBo[i].md == NULL) {
			return (int)i;
		}
	}
	return -1;
}

/* Clean and invalidate every buffer of this client (not surfaces: sel 4 does those). */
void
SapphireGpuClient::flushAll(void)
{
	for (uint32_t i = 1; i < UC_BOS; i++) {
		if (fBo[i].md != NULL && fBo[i].surf == NULL && fBo[i].kva != 0) {
			sweet_dcache_flush((vm_offset_t)fBo[i].kva, (unsigned)fBo[i].len);
		}
	}
}

/* service_lock held */
void
SapphireGpuClient::drop(uint32_t h)
{
	struct uc_bo *b = &fBo[h];

	if (h == 0 || h >= UC_BOS || b->md == NULL) {
		return;
	}
	uc_unmap(b);
	if (b->surf != NULL) {
		b->md->complete();
	}
	OSSafeReleaseNULL(b->md);
	OSSafeReleaseNULL(b->surf);
	bzero(b, sizeof(*b));
}

IOReturn
SapphireGpuClient::clientClose(void)
{
	if (service_lock != NULL) {
		IOLockLock(service_lock);
		for (uint32_t i = 1; i < UC_BOS; i++) {
			drop(i);
		}
		IOLockUnlock(service_lock);
	}
	LOG("client: %s closed after %u calls", fName, fCalls);
	terminate();
	return kIOReturnSuccess;
}

IOReturn
SapphireGpuClient::clientMemoryForType(UInt32 type, IOOptionBits *options, IOMemoryDescriptor **memory)
{
	IOReturn r = kIOReturnBadArgument;

	IOLockLock(service_lock);
	if (type > 0 && type < UC_BOS && fBo[type].md != NULL && fBo[type].surf == NULL) {
		*memory = fBo[type].md;
		fBo[type].md->retain();
		*options = kIOMapDefaultCache;          /* see above: never write-combined */
		r = kIOReturnSuccess;
	}
	IOLockUnlock(service_lock);
	return r;
}

/* service_lock held */
IOReturn
SapphireGpuClient::create(uint64_t size, uint64_t fixed, uint64_t *out)
{
	int h = freeSlot();

	if (h < 0) {
		return kIOReturnNoResources;
	}
	if (size == 0 || size > (UVA_END - UVA_BASE)) {
		return kIOReturnBadArgument;
	}
	size = (size + 4095) & ~4095ull;
	IOBufferMemoryDescriptor *m = IOBufferMemoryDescriptor::inTaskWithOptions(kernel_task,
	    kIODirectionInOut | kIOMemoryKernelUserShared, size, 4096);
	if (m == NULL) {
		return kIOReturnNoMemory;
	}
	bzero(m->getBytesNoCopy(), size);
	sweet_dcache_flush((vm_offset_t)m->getBytesNoCopy(), (unsigned)size);
	struct uc_bo *b = &fBo[h];
	b->md = m;
	b->len = size;
	b->kva = (uintptr_t)m->getBytesNoCopy();
	if (!uc_map(b, fixed)) {
		m->release();
		bzero(b, sizeof(*b));
		return fixed ? kIOReturnExclusiveAccess : kIOReturnNoSpace;
	}
	out[0] = (uint64_t)h;
	out[1] = b->va + b->delta;
	return kIOReturnSuccess;
}

/* service_lock held */
IOReturn
SapphireGpuClient::import(uint32_t id, uint64_t *out)
{
	int h = freeSlot();

	if (h < 0) {
		return kIOReturnNoResources;
	}
	SweetSurface *s = sweet_surface_copy(id);
	if (s == NULL || s->memory == NULL) {
		OSSafeReleaseNULL(s);
		return kIOReturnNotFound;
	}
	if (s->memory->prepare() != kIOReturnSuccess) {
		s->release();
		return kIOReturnNoMemory;
	}
	struct uc_bo *b = &fBo[h];
	b->md = s->memory;
	b->md->retain();
	b->surf = s;
	b->len = s->memory->getLength();
	b->kva = s->kernelAddress();
	if (!uc_map(b, 0)) {
		b->md->complete();
		OSSafeReleaseNULL(b->md);
		OSSafeReleaseNULL(b->surf);
		bzero(b, sizeof(*b));
		return kIOReturnNoSpace;
	}
	out[0] = (uint64_t)h;
	out[1] = b->va + b->delta + s->wrapDelta;     /* byte 0 of the surface, as IOSurfaceGetBaseAddress */
	out[2] = b->len - s->wrapDelta;
	return kIOReturnSuccess;
}

/* service_lock held */
IOReturn
SapphireGpuClient::run(uint64_t va, uint64_t dwords, uint64_t ms, uint64_t *out)
{
	bool inside = false;

	for (uint32_t i = 1; i < UC_BOS && !inside; i++) {
		struct uc_bo *b = &fBo[i];
		inside = b->md != NULL && va >= b->va + b->delta && va + dwords * 4 <= b->va + b->delta + b->len;
	}
	if (!inside || dwords == 0 || dwords > 0xFFFFF || (va & 3) != 0) {
		return kIOReturnBadArgument;
	}
	ms = ms == 0 ? 100 : (ms > 2000 ? 2000 : ms);
	if (!ring_room(64, 2000)) {
		return kIOReturnTimeout;
	}
	flushAll();                                 /* what the CPU wrote, out to RAM */
	emit(pkt7(CP_INDIRECT_BUFFER, 3));
	emit((uint32_t)va);
	emit((uint32_t)(va >> 32));
	emit((uint32_t)dwords);
	emit(pkt7(CP_WAIT_FOR_IDLE, 0));
	event(EV_CACHE_FLUSH_TS, true);
	emit(pkt7(CP_WAIT_FOR_IDLE, 0));
	uint64_t t0 = mach_absolute_time(), ns;
	bool ok = submit("user", (uint32_t)ms);
	absolutetime_to_nanoseconds(mach_absolute_time() - t0, &ns);
	flushAll();                                 /* and no stale line over what the GPU wrote */
	out[0] = *(volatile uint32_t *)(pool + OFF_FENCE);
	out[1] = ns / 1000;
	if (!ok) {
		service_ready = false;
		LOG("client: %s's submit of %llu dwords at %llx did not finish in %llu ms: the GPU service stops; RBBM_STATUS %08x CP_HW_FAULT %08x",
		    fName, dwords, va, ms, grd(REG_RBBM_STATUS), grd(REG_CP_HW_FAULT));
		smmu_fault("user submit");
		return kIOReturnTimeout;
	}
	return kIOReturnSuccess;
}

IOReturn
SapphireGpuClient::externalMethod(uint32_t selector, IOExternalMethodArguments *a, IOExternalMethodDispatch *,
    OSObject *, void *)
{
	static const uint8_t ins[] = { 0, 2, 1, 1, 1, 3 }, outs[] = { 5, 2, 0, 3, 0, 2 };
	uint64_t out[5] = { 0 };
	IOReturn r = kIOReturnSuccess;

	if (selector >= sizeof(ins) || a->scalarInputCount != ins[selector] || a->scalarOutputCount < outs[selector]) {
		return kIOReturnBadArgument;
	}
	IOLockLock(service_lock);
	if (!service_ready) {
		IOLockUnlock(service_lock);
		return kIOReturnNotReady;
	}
	const uint64_t *in = a->scalarInput;
	switch (selector) {
	case 0:
		out[0] = 610;
		out[1] = 0x06010001;
		out[2] = UVA_BASE;
		out[3] = UVA_END;
		out[4] = PT_PAGES - pt_used;
		break;
	case 1:
		r = create(in[0], in[1], out);
		break;
	case 2:
		if (in[0] == 0 || in[0] >= UC_BOS || fBo[in[0]].md == NULL) {
			r = kIOReturnBadArgument;
		} else {
			drop((uint32_t)in[0]);
		}
		break;
	case 3:
		r = import((uint32_t)in[0], out);
		break;
	case 4:
		if (in[0] == 0 || in[0] >= UC_BOS || fBo[in[0]].md == NULL || fBo[in[0]].kva == 0) {
			r = kIOReturnBadArgument;
		} else {
			sweet_dcache_flush((vm_offset_t)fBo[in[0]].kva, (unsigned)fBo[in[0]].len);
		}
		break;
	case 5:
		r = run(in[0], in[1], in[2], out);
		break;
	}
	IOLockUnlock(service_lock);
	if (fCalls++ < 32 || r != kIOReturnSuccess) {
		LOG("client: %s sel %u (%llx %llx) -> %x, out %llx %llx", fName, selector,
		    ins[selector] > 0 ? in[0] : 0, ins[selector] > 1 ? in[1] : 0, r, out[0], out[1]);
	}
	a->scalarOutputCount = outs[selector];
	for (uint32_t i = 0; i < outs[selector]; i++) {
		a->scalarOutput[i] = out[i];
	}
	return r;
}

IOReturn sapphire_gpu_new_client(IOService *provider, task_t task, void *security, UInt32 type, IOUserClient **handler);

IOReturn
sapphire_gpu_new_client(IOService *provider, task_t task, void *security, UInt32 type, IOUserClient **handler)
{
	char name[32];
	proc_selfname(name, sizeof(name));
	if (!service_ready || service_lock == NULL) {
		LOG("client: %s asked for the GPU, but the service is not up", name);
		return kIOReturnNotReady;
	}
	SapphireGpuClient *c = OSTypeAlloc(SapphireGpuClient);
	if (c == NULL || !c->initWithTask(task, security, type)) {
		OSSafeReleaseNULL(c);
		return kIOReturnNoMemory;
	}
	if (!c->attach(provider)) {
		c->release();
		return kIOReturnError;
	}
	if (!c->start(provider)) {
		c->detach(provider);
		c->release();
		return kIOReturnError;
	}
	LOG("client: %s opened the GPU", name);
	*handler = c;
	return kIOReturnSuccess;
}

/* ---------------------------------------------------------------- msm client (Mesa) */

/*
 * SapphireGPU: the Adreno for Mesa's freedreno, which speaks Linux's msm DRM interface. The
 * interface is HackiOS's (github.com/oosexgang, iokit/Kernel/RedwoodGPU.cpp, whose design
 * this follows; their user-space libdrm stand-in, tools/mesa/redwood-drm, MIT, is what talks
 * to it): every msm request is a call of this user client whose selector is the request's
 * number (the low byte of the ioctl code) and whose structure goes in and comes back as it is;
 * mmap of a buffer is the client memory of the type that is its handle. Request 0x7e, not
 * Linux's, makes an IOSurface's memory a buffer (by surface id).
 *
 * Built on what gpu_service already runs: the same ring, lock and page tables. Buffers get GPU
 * addresses from 0x60000000 to 0xE0000000 (2 GB, no identity mapping may land there) in 4 KB
 * pages, and the kernel chooses them (MSM_INFO_GET_IOVA; VA_START/VA_SIZE refused, as Linux
 * does for a client without its own page tables).
 *
 * Memory is cacheable (never write-combined: changing the cache mode of these pages panics this
 * kernel, see the SGPU client). So: before a submission every buffer it names that the program
 * has mapped is cleaned out of the CPU caches, and every IOSurface whose seed moved (or that was
 * last cleaned more than 4 ms ago: not everyone who writes one says so); GEM_CPU_PREP (the CPU
 * is about to read what the GPU wrote) cleans and invalidates the buffer.
 *
 * Submissions run to completion before the call returns (no queue yet), so every fence is done
 * by the time anyone waits for it; a submission that does not finish in 2 s stops the service.
 * Development quality: one address space for all clients, no register protection.
 */
#define MSM_PAGES           ((MSM_VA_END - MSM_VA_START) >> 12)
#define MSM_BUFFERS         4096u
#define MSM_BUFFER_MAX      (256ull << 20)
#define MSM_SUBMIT_CMDS     256u                /* Mesa sends 68 and more (a frame of CoreAnimation) */
#define MSM_SUBMIT_BOS      4096u

enum {
	kDrmGemClose        = 0x09,
	kMsmGetParam        = 0x40,
	kMsmSetParam        = 0x41,
	kMsmGemNew          = 0x42,
	kMsmGemInfo         = 0x43,
	kMsmGemCpuPrep      = 0x44,
	kMsmGemCpuFini      = 0x45,
	kMsmGemSubmit       = 0x46,
	kMsmWaitFence       = 0x47,
	kMsmGemMadvise      = 0x48,
	kMsmSubmitQueueNew  = 0x4a,
	kMsmSubmitQueueClose = 0x4b,
	kMsmSubmitQueueQuery = 0x4c,
	kMsmSurface         = 0x7e,             /* HackiOS's: an IOSurface's memory as a buffer */
	kMsmUserMemory      = 0x7d,             /* sapphire 0380: the program's own memory as a buffer */
};

struct msm_timespec { int64_t seconds, nanoseconds; };
struct msm_param { uint32_t pipe, param; uint64_t value; uint32_t len, pad; };
struct msm_gem_new { uint64_t size; uint32_t flags, handle; };
struct msm_gem_info { uint32_t handle, info; uint64_t value; uint32_t len, pad; };
struct msm_gem_cpu_prep { uint32_t handle, op; struct msm_timespec timeout; };
struct msm_gem_submit_cmd { uint32_t type, submit_idx, submit_offset, size, pad, nr_relocs; uint64_t relocs; };
struct msm_gem_submit_bo { uint32_t flags, handle; uint64_t presumed; };
struct msm_gem_submit {
	uint32_t flags, fence, nr_bos, nr_cmds;
	uint64_t bos, cmds;
	int32_t fence_fd;
	uint32_t queueid;
	uint64_t in_syncobjs, out_syncobjs;
	uint32_t nr_in_syncobjs, nr_out_syncobjs, syncobj_stride, pad;
};
struct msm_wait_fence { uint32_t fence, flags; struct msm_timespec timeout; uint32_t queueid; };
struct msm_gem_madvise { uint32_t handle, madv, retained; };
struct msm_submitqueue { uint32_t flags, prio, id; };
struct msm_submitqueue_query { uint64_t data; uint32_t id, param, len, pad; };
struct drm_gem_close { uint32_t handle, pad; };
struct msm_surface {                            /* HackiOS's struct redwood_surface */
	uint32_t surface;                       /* the IOSurface id */
	uint32_t handle;                        /* 0xffffffff: only describe it; back: the buffer */
	uint64_t size;
	uint32_t offset, rowBytes;              /* where the pixels start in the buffer, a row's length */
	uint32_t width, height, pixelFormat, planes;
	uint32_t ubwc, metaOffset, metaStride, metaSize, dataOffset, dataStride;
};

#define MSM_INFO_GET_OFFSET 0
#define MSM_INFO_GET_IOVA   1
#define MSM_INFO_SET_NAME   2
#define MSM_INFO_SET_IOVA   4
#define MSM_INFO_GET_FLAGS  5
#define MSM_SUBMIT_CMD_BUF  1
#define MSM_SUBMIT_CMD_CTX_RESTORE_BUF 3
#define MSM_SUBMIT_FENCE_FD_OUT 0x20000000
#define MSM_DESCRIBE        0xffffffffu
struct msm_user_memory {                        /* sapphire 0380 (hakios-metal's gpu.hpp: UserMemory) */
	uint64_t address, size;                 /* the program's range: whole pages */
	uint32_t flags;                         /* 0 */
	uint32_t handle;                        /* back: the buffer */
};

static uint64_t msm_bits[MSM_PAGES / 64];
static uint64_t msm_next;                       /* where the next search starts (next fit) */

static bool
msm_used(uint64_t p)
{
	return (msm_bits[p >> 6] >> (p & 63)) & 1;
}

static void
msm_mark(uint64_t p0, uint64_t n, bool used)
{
	for (uint64_t p = p0; p < p0 + n; p++) {
		if (used) {
			msm_bits[p >> 6] |= 1ull << (p & 63);
		} else {
			msm_bits[p >> 6] &= ~(1ull << (p & 63));
		}
	}
}

/* n pages of addresses, next fit, wrapping once; 0 if none */
static uint64_t
msm_va_alloc(uint64_t n)
{
	for (int pass = 0; pass < 2; pass++) {
		uint64_t start = pass == 0 ? msm_next : 0, end = pass == 0 ? MSM_PAGES : msm_next, run = 0;
		for (uint64_t p = start; p < end; p++) {
			if ((p & 63) == 0 && run == 0 && msm_bits[p >> 6] == ~0ull) {
				p += 63;                /* a full word: skip it */
				continue;
			}
			run = msm_used(p) ? 0 : run + 1;
			if (run == n) {
				msm_mark(p + 1 - n, n, true);
				msm_next = p + 1;
				return MSM_VA_START + ((p + 1 - n) << 12);
			}
		}
	}
	return 0;
}

/*
 * 0153: the msm fences have a slot and a counter of their own. On 0152 the shared slot once held
 * 0x801d229c -- a pixel, not a seqno (the GPU's SMMU had a translation fault at about the same
 * time) -- and a GPU that was idle with the ring read to its end was taken for hung, the service
 * stopped and backboardd crashed (Mesa on the CPU after its restart). Now a fence that looks
 * unfinished while the CP is idle and has read everything counts as done, and that is said.
 */
static bool
msm_fence_done(uint32_t f)
{
	return (int32_t)(*(volatile uint32_t *)(pool + OFF_MSM_FENCE) - f) >= 0;
}

/*
 * sapphire 0350, for SweetFramebuffer's scanout of the window server's surfaces: the fence of the last msm submission
 * handed to the CP, and whether a given one is done. The frame a SwapEnd names was drawn by submissions made before it
 * (HakiosMetal's commit submits before it returns), and the CP runs the one ring in order, so the last fence at that
 * moment covers them; what a submission made later draws into the surface is in its gpuFence (submitIt()). A GPU that
 * takes no more work has nothing pending, as for the copies (sapphire_gpu_fence_done()).
 */
uint32_t sapphire_gpu_msm_fence(void);
bool sapphire_gpu_msm_fence_done(uint32_t fence);

uint32_t
sapphire_gpu_msm_fence(void)
{
	return *(volatile uint32_t *)&msm_seq;          /* (a word, written under service_lock: read as it stands) */
}

bool
sapphire_gpu_msm_fence_done(uint32_t fence)
{
	return !service_ready || fence == 0 || pool == NULL || msm_fence_done(fence);
}

static bool
msm_gpu_idle(void)
{
	return grd(REG_CP_RB_RPTR) == ring_wptr % RING_DWORDS && (grd(REG_RBBM_STATUS) & ~1u) == 0;
}

/*
 * sapphire 0345: kern.sweet_gpu_khz, the GPU's core clock moved between the rows of the stock table while the system
 * runs, as the stock clock driver moves it and with the votes kgsl makes around it (kgsl_pwrctrl.c, the vdd class of
 * gpucc-khaje.c: a higher level before a faster clock, the lower one after a slower clock):
 *   - going up: the CX peak-current votes from 900 and 950 MHz (cx_ipeak_set), the CX rail by rate (rate_max of
 *     gx_gfx3d_clk_src) and the MX rail by VCO (rate_max of gpu_cc_pll0), each never under the 0x1a0 floor of 0337
 *     unless gpu_cx/gpu_mx give the level; then the clock;
 *   - going down: the clock; then the rails; then the peak-current votes the new rate does not need;
 *   - the clock: with the service lock held and the GPU idle (the ring read to its end and RBBM_STATUS idle, waited
 *     for up to gpu_opp_drain ms, 100 by default: no new work can be queued meanwhile), gpu_cc_pll0 takes its new
 *     alpha and L while it runs and the RCG stays on it, as the stock clk_zonda_pll_set_rate() does with a PLL that is
 *     on (alpha, L, 5 us, the lock bit TEST_CTL says to wait for, 100 us). Every row of the stock
 *     ftbl_gpu_cc_gx_gfx3d_clk_src is source 1 divider 1, so a change of rate is a change of the PLL alone.
 *     kern.sweet_gpu_opp_park=1 (boot-arg gpu_opp_park=1) does it the long way: the RCG moves to GPLL0 / 2 (300 MHz,
 *     under every row and valid at any rail level), the PLL is configured and started again from standby at the new
 *     rate as at boot, and the RCG goes back to it.
 * A PLL that reports no lock within 1 ms is brought back the long way at the rate it had. If that fails the core
 * stays on GPLL0 / 2 (300 MHz) and further changes are refused; if the RCG does not take a switch the GPU service is
 * stopped (its copies go back to the CPU) rather than leave the core without a known clock.
 * Needs the stock path of 0337 (gpu_vco2=1 and a gpu_mhz of the table) and a GPU taking work (gpu_service).
 * Read: the rate in kHz (0: no stock path). Write: a row in kHz (or in MHz, 1114 for 1114.8). Errors: EINVAL not a row
 * or above the speed bin's top, ENODEV no stock path or no service, EBUSY the GPU did not go idle in time, EAGAIN the
 * peak-current limit refused, EIO a rail vote refused or the PLL did not lock (kern.sweet_gpu_opp tells which).
 */
#define OPP_ROWS            (sizeof(stock_khz) / sizeof(stock_khz[0]))
#define OPP_CAT(...) do { \
	if (off + 1 < sizeof(buf)) { \
		int n_ = snprintf(buf + off, sizeof(buf) - off, __VA_ARGS__); \
		if (n_ > 0) { \
			off = off + (size_t)n_ < sizeof(buf) ? off + (size_t)n_ : sizeof(buf) - 1; \
		} \
	} \
} while (0)
static bool opp_broken;                         /* a recovery failed: no more changes */
static struct {
	uint64_t up, down, refused_busy, refused_ipeak, refused_rail, pll_fail, recovered, broken;
	uint64_t last_ns, last_drain_ns, last_pll_ns, max_ns, max_drain_ns;
	uint32_t last_from, last_to, last_polls, max_polls;
	int last_err;
	uint64_t row_ns[OPP_ROWS];              /* time spent at each row of stock_khz[] */
	uint64_t since;                         /* when the current rate was set (mach_absolute_time) */
} gopp;

static int
opp_row(uint32_t khz)
{
	for (unsigned i = 0; i < OPP_ROWS; i++) {
		if (stock_khz[i] == khz) {
			return (int)i;
		}
	}
	return -1;
}

static uint64_t
ns_since(uint64_t t0)
{
	uint64_t ns;

	absolutetime_to_nanoseconds(mach_absolute_time() - t0, &ns);
	return ns;
}

/* The time at the current rate into its row's account (opp_lock held). */
static void
opp_account(void)
{
	int r = opp_row(opp_khz);
	uint64_t now = mach_absolute_time();

	if (gopp.since != 0 && r >= 0) {
		uint64_t ns;
		absolutetime_to_nanoseconds(now - gopp.since, &ns);
		gopp.row_ns[r] += ns;
	}
	gopp.since = now;
}

/* gx_gfx3d's RCG to a source and divider, without a word in the log; false if the update does not complete. */
static bool
opp_rcg(uint32_t src, uint32_t div)
{
	rmw(gpucc, 0x1020, 0x71fu, (src << 8) | (2 * div - 1));
	rmw(gpucc, 0x101c, 0, RCG_UPDATE);
	return poll(gpucc, 0x101c, RCG_UPDATE, 0, 500);
}

/*
 * clk_zonda_pll_set_rate() on a PLL that runs: the new alpha and L, 5 us, the lock bit TEST_CTL says to wait for,
 * 100 us. Stock writes alpha first in both directions; for the microseconds between the two writes the PLL then aims
 * at the old L with the new alpha, which on the way down is up to half a step of L (9.6 MHz of core clock) above the
 * old rate, with the rails still at the old rate's level. Here the way down writes L first: in both directions what
 * the PLL aims at between the writes stays at or under the higher of the two rates (the GPU is idle either way).
 */
static bool
opp_pll_dynamic(uint32_t vco_khz, bool up, uint32_t *polls)
{
	uint32_t l, a, lock, i;

	zonda_la_khz(vco_khz, &l, &a);
	if (up) {
		wr(gpucc, ZP_ALPHA, a);
		wr(gpucc, ZP_L, l);
	} else {
		wr(gpucc, ZP_L, l);
		wr(gpucc, ZP_ALPHA, a);
	}
	IODelay(5);                                 /* "Wait before polling for the frequency latch" */
	lock = (rd(gpucc, ZP_TEST) & ZP_STAY_IN_CFA) != 0 ? ZP_FREQ_LOCK : ZP_LOCK;
	for (i = 0; i < 1000 && (rd(gpucc, ZP_MODE) & lock) != lock; i++) {
		IODelay(1);
	}
	*polls = i;
	if (i == 1000) {
		return false;
	}
	IODelay(100);                               /* "Wait for PLL output to stabilize" */
	return (rd(gpucc, ZP_L) & 0xffffu) == l && (rd(gpucc, ZP_ALPHA) & 0xffffu) == a;
}

/*
 * The long way: the RCG to GPLL0 / 2, the PLL started again at vco_khz, the RCG back to it. 1 done, 0 the core was
 * left on GPLL0 / 2 (the PLL did not start), -1 the RCG did not take a switch.
 */
static int
opp_pll_park(uint32_t vco_khz)
{
	if (!opp_rcg(SRC_GPLL0, 2)) {
		return -1;
	}
	if (!gpu_pll_start_vco(vco_khz)) {
		return 0;
	}
	return opp_rcg(SRC_GPU_PLL0, 1) ? 1 : -1;
}

/*
 * sapphire 0347: the DDR voted with the GPU's level, as kgsl votes its path to the DDR at every level change
 * (kgsl_bus.c: ib = qcom,bus-table-ddr[qcom,bus-freq of the level], ab = 25 % of ib; kgsl_pwrctrl.c: before the clock
 * when it goes up, after it when it goes down). The interconnect driver turns that into the BIMC clock it asks the RPM
 * for: max(ab sum, ib peak) / the buswidth of the ebi node, 8 bytes (icc-rpm.c qcom_icc_rpm_set, bengal.c "ebi"), in
 * kHz rounded up by clk-smd-rpm; with ab at 25 % of ib, ib decides. This driver sends that rate itself: RPM "clk2" 0,
 * key "KHz", active set, never under kern.sweet_gpu_ddr_floor (kHz). Rates of the clock's table that are no level of
 * the device tree take the bus level of the next level up (820 and 980 that of 1025, 1100 that of 1114.8).
 *   kern.sweet_gpu_ddr (boot-arg gpu_ddr)  0 (default): no vote; 1: a vote per level, from now on and at every
 *                                          change of rate; writing 0 again votes INT_MAX, what the stock clock
 *                                          driver leaves on the BIMC before the interconnect driver votes;
 *   kern.sweet_gpu_ddr_stats              votes, refusals, the last vote, and the DDR measured (0343's mccc period)
 *                                          before the last vote, 20 ms after the vote made by the sysctl, and now.
 * While the RPM's clock scaling is off -- nothing in this kernel turns it on (clk_smd_rpm_enable_scaling, "Enab" on
 * "clk0" 2) -- a vote is expected to change nothing (report 11, section 10); the before/after reading says whether it
 * does.
 */
static int ddr_mode;                            /* kern.sweet_gpu_ddr */
static int ddr_floor_khz;                       /* kern.sweet_gpu_ddr_floor */
static uint32_t ddr_voted;                      /* the BIMC kHz this driver last asked the RPM for (0: none) */
static struct {
	uint64_t votes, refused;
	uint32_t last_khz, before_khz, after_khz;
	int last_err;
} gddr;

/* The BIMC kHz kgsl's vote for a GPU rate comes to (bin 0 of this phone's tree). */
static uint32_t
ddr_khz_for(uint32_t gpu_khz)
{
	static const uint32_t ib[] = { 0, 1562500, 4273437, 6000000, 7945312, 12148437, 14093750, 16343750 };   /* kB/s */
	unsigned idx = gpu_khz > 1025000 ? 7 : gpu_khz > 785000 ? 6 : gpu_khz > 600000 ? 5 : gpu_khz > 465000 ? 4 :
	    gpu_khz > 320000 ? 3 : 2;
	uint32_t khz = (ib[idx] + 7) / 8;

	return MAX(khz, (uint32_t)MAX(ddr_floor_khz, 0));
}

/*
 * One BIMC vote (active set); 0 or the RPM's answer. 0361: through the DDR manager (TopazModem.cpp), which sends it
 * as it is while the RPM's clock scaling is off, and with scaling on takes it as the GPU's input with the others.
 */
extern "C" int sweet_ddr_gpu_vote(uint32_t khz);

static int
ddr_vote(uint32_t khz)
{
	gddr.before_khz = ddr_khz();
	int r = sweet_ddr_gpu_vote(khz);                /* "clk2" 0 (the BIMC), "KHz" */
	gddr.last_khz = khz;
	gddr.last_err = r;
	if (r == 0) {
		gddr.votes++;
		ddr_voted = khz;
	} else {
		gddr.refused++;
	}
	return r;
}

/* Toward the GPU rate khz, only upward (raise) or only downward, as opp_votes() moves the rails. opp_lock held. */
static void
opp_ddr(uint32_t khz, bool raise)
{
	if (ddr_mode == 0) {
		return;
	}
	uint32_t want = ddr_khz_for(khz);
	if (raise ? want > ddr_voted : want < ddr_voted) {
		(void)ddr_vote(want);                   /* a refusal leaves the old vote; the GPU's rate changes anyway */
	}
}

/*
 * The rails and the peak-current votes toward the rate khz, only upward (raise) or only downward: 0, or EAGAIN (the
 * peak-current limit refused) or EIO (a rail vote refused) when raising. Lowering cannot fail: a vote the RPM refuses
 * there leaves the higher level in place, which is safe. opp_lock held.
 */
static int
opp_votes(uint32_t khz, bool raise)
{
	uint32_t cx = cx_arg != ~0u ? cx_arg : MAX(cx_level_for(khz), 0x1a0u);
	uint32_t mx = mx_arg != ~0u ? mx_arg : MAX(mx_level_for(2 * khz), 0x1a0u);
	int r;

	if (raise && !cx_ipeak_set(khz)) {
		return EAGAIN;
	}
	if (cx != 0 && (raise ? cx > cx_voted : cx < cx_voted)) {
		r = topaz_rpm_vote(0, 0x78637772, 0, 0x6c766c76, cx);          /* "rwcx" 0, "vlvl" */
		if (r == 0) {
			cx_voted = cx;
		} else if (raise) {
			LOG("opp: CX rail (RPM rwcx 0) to level %u for %u kHz refused: %d", cx, khz, r);
			return EIO;
		}
	}
	if (mx != 0 && (raise ? mx > mx_voted : mx < mx_voted)) {
		r = topaz_rpm_vote(0, 0x786d7772, 0, 0x6c766c76, mx);          /* "rwmx" 0, "vlvl" */
		if (r == 0) {
			mx_voted = mx;
		} else if (raise) {
			LOG("opp: MX rail (RPM rwmx 0) to level %u for %u kHz refused: %d", mx, khz, r);
			return EIO;
		}
	}
	opp_ddr(khz, raise);                        /* 0347: the bus, after the rails going up, with them going down */
	if (!raise) {
		(void)cx_ipeak_set(khz);                /* drops the votes the rate does not need */
	}
	return 0;
}

/* Wait, with the service lock held, for the GPU to have nothing queued or running; false after ms. */
static bool
opp_drain(uint32_t ms)
{
	uint64_t deadline;

	clock_interval_to_deadline(ms, kMillisecondScale, &deadline);
	while (!msm_gpu_idle()) {
		if (mach_absolute_time() >= deadline) {
			return false;
		}
		delay_for_interval(20, NSEC_PER_USEC);
	}
	return true;
}

/* The row of the stock table a request names (kHz, or MHz below 10000), or 0. */
static uint32_t
opp_request_khz(uint32_t v)
{
	for (unsigned i = 0; i < OPP_ROWS; i++) {
		if (stock_khz[i] == v || (v < 10000 && stock_khz[i] / 1000 == v)) {
			return stock_khz[i];
		}
	}
	return 0;
}

/* Changes the core clock to the stock row khz; 0 or an errno (see above). who: for the log (NULL: say nothing). */
static int
gpu_opp_set(uint32_t khz, const char *who)
{
	uint32_t old, polls = 0, anybin = 0, actual;
	uint64_t t0 = mach_absolute_time(), t1 = 0, t2 = 0;
	bool ok;
	int err = 0;

	if (opp_lock == NULL) {
		return ENODEV;
	}
	(void)PE_parse_boot_argn("gpu_anybin", &anybin, sizeof(anybin));
	if (opp_row(khz) < 0 || (khz > bin_max_khz(bin_fuse) && anybin == 0)) {
		return EINVAL;
	}
	IOLockLock(opp_lock);
	old = opp_khz;
	if (old == 0 || opp_row(old) < 0 || opp_broken || !service_ready || !clocks_up) {
		IOLockUnlock(opp_lock);
		return ENODEV;
	}
	if (khz == old) {
		IOLockUnlock(opp_lock);
		return 0;
	}
	opp_quiet = true;
	if (khz > old) {
		err = opp_votes(khz, true);
		if (err != 0) {
			(void)opp_votes(old, false);        /* takes back what was raised */
			if (err == EAGAIN) {
				gopp.refused_ipeak++;
			} else {
				gopp.refused_rail++;
			}
			gopp.last_err = err;
			IOLockUnlock(opp_lock);
			return err;
		}
	}
	IOLockLock(service_lock);
	t1 = mach_absolute_time();
	if (!service_ready || !opp_drain(opp_drain_ms)) {
		IOLockUnlock(service_lock);
		(void)opp_votes(old, false);
		gopp.refused_busy++;
		gopp.last_err = EBUSY;
		IOLockUnlock(opp_lock);
		return EBUSY;
	}
	t2 = mach_absolute_time();
	actual = khz;
	if (opp_park != 0) {
		int p = opp_pll_park(2 * khz);
		ok = p == 1;
		if (p == 0) {
			p = gpu_pll_start_vco(2 * old) ? (opp_rcg(SRC_GPU_PLL0, 1) ? 1 : -1) : 0;
			actual = p == 1 ? old : 300000;
		} else if (p < 0) {
			actual = 0;
		}
		if (!ok) {
			gopp.pll_fail++;
			err = EIO;
			opp_broken = p != 1;
			if (p < 0) {
				service_ready = false;
			}
		}
	} else {
		ok = opp_pll_dynamic(2 * khz, khz > old, &polls);
		if (!ok) {
			int p = opp_pll_park(2 * old);
			gopp.pll_fail++;
			err = EIO;
			actual = p == 1 ? old : p == 0 ? 300000 : 0;
			if (p == 1) {
				gopp.recovered++;
			} else {
				opp_broken = true;
				if (p < 0) {
					service_ready = false;
				}
			}
		}
	}
	IOLockUnlock(service_lock);
	if (opp_broken) {
		gopp.broken++;
		LOG("opp: %u -> %u kHz: the PLL %s; %s", old, khz, ok ? "changed" : "did not lock",
		    actual == 300000 ? "the core stays on GPLL0 / 2 (300 MHz), no more changes" :
		    "the RCG did not take a switch: the GPU service is stopped");
	} else if (!ok) {
		LOG("opp: %u -> %u kHz: the PLL did not lock (mode %08x test %08x); back at %u kHz the long way", old, khz,
		    rd(gpucc, ZP_MODE), rd(gpucc, ZP_TEST), actual);
	}
	opp_account();
	opp_khz = actual;
	if (actual != 0) {
		(void)opp_votes(actual, false);         /* the levels the rate it runs at needs, no more (0: unknown, all kept) */
	}
	if (ok) {
		if (khz > old) {
			gopp.up++;
		} else {
			gopp.down++;
		}
	}
	gopp.last_from = old;
	gopp.last_to = khz;
	gopp.last_polls = polls;
	if (polls > gopp.max_polls) {
		gopp.max_polls = polls;
	}
	gopp.last_ns = ns_since(t0);
	absolutetime_to_nanoseconds(t2 - t1, &gopp.last_drain_ns);
	gopp.last_pll_ns = ns_since(t2);
	if (gopp.last_ns > gopp.max_ns) {
		gopp.max_ns = gopp.last_ns;
	}
	if (gopp.last_drain_ns > gopp.max_drain_ns) {
		gopp.max_drain_ns = gopp.last_drain_ns;
	}
	gopp.last_err = err;
	IOLockUnlock(opp_lock);
	if (who != NULL && err == 0) {
		LOG("opp (%s): %u -> %u kHz in %llu us (GPU idle after %llu us, PLL %llu us, %u lock polls)", who, old, khz,
		    gopp.last_ns / 1000, gopp.last_drain_ns / 1000, gopp.last_pll_ns / 1000, polls);
	}
	return err;
}

static int
sweet_gpu_khz_sysctl SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	int v = (int)opp_khz;
	int error = sysctl_io_number(req, v, sizeof(v), &v, NULL);

	if (error != 0 || req->newptr == USER_ADDR_NULL) {
		return error;
	}
	uint32_t khz = v > 0 ? opp_request_khz((uint32_t)v) : 0;
	if (gov_on) {
		return EBUSY;                           /* 0346: the governor sets the rate (kern.sweet_gpu_gov=0 first) */
	}
	return khz == 0 ? EINVAL : gpu_opp_set(khz, "sysctl");
}
SYSCTL_PROC(_kern, OID_AUTO, sweet_gpu_khz, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED,
    NULL, 0, sweet_gpu_khz_sysctl, "I", "sapphire 0345: the GPU's core clock in kHz; write a row of the stock table to change it");

static int
sweet_gpu_opp_sysctl SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	char buf[900];
	size_t off = 0;

	if (opp_lock != NULL) {
		IOLockLock(opp_lock);
		opp_account();
	}
	buf[0] = 0;
	OPP_CAT("now %u kHz%s (%s); up %llu down %llu; refused: GPU busy %llu, peak current %llu, rail %llu; PLL without lock %llu "
	    "(recovered %llu, broken %llu); last %u -> %u kHz err %d: %llu us, idle after %llu us, PLL %llu us, %u lock polls; "
	    "max %llu us, idle after %llu us, %u polls; drain limit %u ms; votes CX %u MX %u\n", opp_khz,
	    opp_broken ? " BROKEN" : "", opp_park ? "the long way (park)" : "dynamic (stock)", gopp.up, gopp.down,
	    gopp.refused_busy, gopp.refused_ipeak, gopp.refused_rail, gopp.pll_fail, gopp.recovered, gopp.broken,
	    gopp.last_from, gopp.last_to, gopp.last_err, gopp.last_ns / 1000, gopp.last_drain_ns / 1000, gopp.last_pll_ns / 1000,
	    gopp.last_polls, gopp.max_ns / 1000, gopp.max_drain_ns / 1000, gopp.max_polls, opp_drain_ms, cx_voted, mx_voted);
	OPP_CAT("time at each rate, ms:");
	for (unsigned i = 0; i < OPP_ROWS; i++) {
		OPP_CAT(" %u %llu", stock_khz[i], gopp.row_ns[i] / 1000000);
	}
	OPP_CAT("\n");
	if (opp_lock != NULL) {
		IOLockUnlock(opp_lock);
	}
	return SYSCTL_OUT(req, buf, off + 1);
}
SYSCTL_PROC(_kern, OID_AUTO, sweet_gpu_opp, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED,
    NULL, 0, sweet_gpu_opp_sysctl, "A", "sapphire 0345: changes of the GPU's core clock: counts, times, refusals, time at each rate");
SYSCTL_INT(_kern, OID_AUTO, sweet_gpu_opp_park, CTLFLAG_RW | CTLFLAG_LOCKED, &opp_park, 0,
    "sapphire 0345: 1 = change the PLL the long way (RCG on GPLL0 / 2 meanwhile) instead of the stock dynamic update");

/*
 * sapphire 0346: a governor for the GPU's core clock, by how busy the GPU is, with a temperature cap. Off unless
 * kern.sweet_gpu_gov=1 (boot-arg gpu_gov=1 starts it once the GPU service is up). Every gpu_gov_ms (50):
 *   - busy: GMU CX power counter 0, which counts XO ticks (19.2 MHz) while the GPU is busy -- the counter kgsl
 *     (a6xx_power_stats: busy_us = ticks * 10 / 192) and Linux (a6xx_gpu_busy, also for the GMU wrapper of the a610)
 *     read, set up as both set it up: GPU_GMU_AO_GPU_CX_BUSY_MASK = 0xff000000, GMU_CX_GMU_POWER_COUNTER_SELECT_0[7:0]
 *     = 0x20, GMU_CX_GMU_POWER_COUNTER_ENABLE = 1 (kgsl a6xx_start; dword offsets 0x23b0e, 0x1f841, 0x1f840 and the
 *     count at 0x1f844 from the GPU's 0x5900000, so 0x598ec38, 0x597e104, 0x597e100 and 0x597e110); load = busy time /
 *     wall time;
 *   - rate: the levels of the stock device tree (320 465 600 785 1025 1114.8 1260 MHz) between kern.sweet_gpu_khz_min
 *     (320000) and kern.sweet_gpu_khz_max (0: the boot rate; a rate of the stock table that is no level of the tree,
 *     820 for one, joins the ladder as its top). Load >= 95 %: the top (Linux's simple_ondemand goes to the top over
 *     its 50 %, kgsl's msm-adreno-tz over 50 ms busy in a window); >= kern.sweet_gpu_gov_up (80 %): at least one level
 *     up, as far as the level that would bring the load to the middle of up and down; < kern.sweet_gpu_gov_down (40 %)
 *     for two periods in a row: at least one level down, as far as that same level; no submission, no panel copy and
 *     under 2 % for kern.sweet_gpu_gov_idle_ms (100): the lowest level (kgsl drops the GPU's clocks at once when
 *     idle and to its lowest level after 80 ms);
 *   - temperature: TSENS 15 (zone "gpu"). The stock zone has a passive trip at 95 C, handled by step_wise with the
 *     kgsl cooling device (state N = level N): one level down for each reading at or over the trip that is higher than
 *     the one before, one level back up for each reading under it that is not higher; and a trip at 100 C whose
 *     cooling map starts at state 4 (600 MHz). Here kern.sweet_gpu_temp_max (95000 mC) is the first trip and 5 C over
 *     it the second, read every period. A GPU sensor that gives no valid value three periods in a row caps the rate at
 *     the lowest level until it reads again.
 * A change goes through gpu_opp_set() (0345). A GPU that does not go idle in time is tried again at the next period;
 * a refused peak-current or rail vote keeps the governor under that level; no stock path, no service, or a failed
 * recovery stop it. A busy counter that does not move while work is submitted for a second stops it too (it would
 * keep the GPU at its lowest level). When it stops it gives back the boot rate. kern.sweet_gpu_khz cannot be written
 * while it runs (EBUSY). kern.sweet_gpu_gov_stats tells what it saw and did; the time at each rate is in
 * kern.sweet_gpu_opp.
 */
#define GMU_PWR_PA          0x0597e000ull
#define GMU_PWR_ENABLE      0x100u
#define GMU_PWR_SELECT_0    0x104u
#define GMU_PWR_XOCLK_0_L   0x110u
#define GMU_AO_PA           0x0598e000ull
#define GMU_AO_CX_BUSY_MASK 0xc38u
#define GOV_LEVELS          8u
static int gov_khz_max, gov_khz_min = 320000, gov_temp_max = 95000;
static int gov_ms = 50, gov_up = 80, gov_down = 40, gov_idle_ms = 100;
static volatile uint8_t *gmu_pwr, *gmu_ao;
static volatile bool gov_running;
static struct {
	uint64_t ticks, up, down, idle_drops, refused_busy, refused_other, capped_ticks, temp_lost, counter_setups;
	uint64_t bad_samples, load_hist[10], load_sum;
	uint32_t last_load, cap_khz, refused_khz, last_busy;
	int32_t last_temp, max_temp;
	const char *stopped;
} ggov;

static const uint32_t gov_dt_levels[] = { 320000, 465000, 600000, 785000, 1025000, 1114800, 1260000 };

/* The ladder the governor moves on, lowest first; its length. */
static unsigned
gov_ladder(uint32_t *lv)
{
	uint32_t top = gov_khz_max > 0 ? (uint32_t)gov_khz_max : gov_boot_khz, low = (uint32_t)MAX(gov_khz_min, 320000);
	uint32_t anybin = 0;
	unsigned n = 0;

	(void)PE_parse_boot_argn("gpu_anybin", &anybin, sizeof(anybin));
	if (anybin == 0 && top > bin_max_khz(bin_fuse)) {
		top = bin_max_khz(bin_fuse);
	}
	for (unsigned i = 0; i < sizeof(gov_dt_levels) / sizeof(gov_dt_levels[0]); i++) {
		if (gov_dt_levels[i] >= low && gov_dt_levels[i] <= top) {
			lv[n++] = gov_dt_levels[i];
		}
	}
	if (opp_row(top) >= 0 && (n == 0 || lv[n - 1] != top) && top >= low) {
		lv[n++] = top;                          /* a rate of the clock's table that is no level of the tree: the top */
	}
	return n;
}

/* The index of the lowest level of the ladder at or above khz (the top if none). */
static unsigned
gov_index_for(const uint32_t *lv, unsigned n, uint64_t khz)
{
	for (unsigned i = 0; i < n; i++) {
		if (lv[i] >= khz) {
			return i;
		}
	}
	return n - 1;
}

/* The busy counter, set up as kgsl and Linux set it (again if it was changed, e.g. by a reset of the GPU). */
static bool
gov_counter(uint32_t *ticks)
{
	if (map_reg_page(&gmu_pwr, GMU_PWR_PA) == NULL || map_reg_page(&gmu_ao, GMU_AO_PA) == NULL) {
		return false;
	}
	if ((rd(gmu_pwr, GMU_PWR_SELECT_0) & 0xffu) != 0x20u || (rd(gmu_pwr, GMU_PWR_ENABLE) & 1u) == 0 ||
	    rd(gmu_ao, GMU_AO_CX_BUSY_MASK) != 0xff000000u) {
		wr(gmu_ao, GMU_AO_CX_BUSY_MASK, 0xff000000u);
		rmw(gmu_pwr, GMU_PWR_SELECT_0, 0xffu, 0x20u);
		wr(gmu_pwr, GMU_PWR_ENABLE, 1);
		ggov.counter_setups++;
	}
	*ticks = rd(gmu_pwr, GMU_PWR_XOCLK_0_L);
	return true;
}

/* Thermal state across periods: the cap is a rate (kHz), so that it keeps its meaning if the ladder changes. */
static struct {
	uint32_t cap;                           /* the rate may not exceed it; ~0: no cap */
	int fails;
	int32_t last;
	bool have_last;
} gtherm;

/* The highest level of the ladder at or under khz (the lowest if none). */
static unsigned
gov_index_under(const uint32_t *lv, unsigned n, uint32_t khz)
{
	unsigned i = 0;

	while (i + 1 < n && lv[i + 1] <= khz) {
		i++;
	}
	return i;
}

/* The cap from the GPU's temperature, step_wise as the stock zone runs it (see above): an index into the ladder. */
static unsigned
gov_thermal(const uint32_t *lv, unsigned n)
{
	int32_t t = 0;
	int e = tsens_read(TSENS_GPU, &t);
	unsigned top = n - 1;

	if (e != 0) {
		ggov.temp_lost++;
		if (++gtherm.fails >= 3) {
			return 0;                       /* no temperature: the lowest level until it reads again */
		}
		return gtherm.cap >= lv[top] ? top : gov_index_under(lv, n, gtherm.cap);
	}
	if (gtherm.fails >= 3) {
		gtherm.cap = ~0u;                       /* it reads again: the step-wise state starts over */
	}
	gtherm.fails = 0;
	int32_t trip = MAX(gov_temp_max, 50000), hard = trip + 5000;
	bool rising = gtherm.have_last && t > gtherm.last;
	unsigned c = gtherm.cap >= lv[top] ? top : gov_index_under(lv, n, gtherm.cap);
	if (t >= trip) {
		if (rising && c > 0) {
			c--;                            /* one cooling state more */
		}
	} else if (!rising && c < top) {
		c++;                                    /* one cooling state less */
	}
	if (t >= hard) {
		unsigned h = gov_index_under(lv, n, 600000);
		if (c > h) {
			c = h;                          /* the stock 100 C map starts at state 4: 600 MHz */
		}
	}
	gtherm.cap = c >= top ? ~0u : lv[c];
	gtherm.last = t;
	gtherm.have_last = true;
	ggov.last_temp = t;
	if (t > ggov.max_temp) {
		ggov.max_temp = t;
	}
	return c;
}

/* Where the governor's state across periods lives (the thread's). */
static struct {
	uint32_t busy, seq;
	uint64_t copies, time;
	uint32_t idle_ms, low;
	bool have;
	uint32_t dead_ms, bad;
} gstate;

/* One period. false: the governor has to stop (ggov.stopped says why). */
static bool
gov_tick(void)
{
	uint32_t lv[GOV_LEVELS], busy, seq = msm_seq;
	uint64_t now = mach_absolute_time(), copies = gdiag.copy_n, wall_ns;
	unsigned n = gov_ladder(lv);

	if (n == 0) {
		ggov.stopped = "no level between kern.sweet_gpu_khz_min and kern.sweet_gpu_khz_max";
		return false;
	}
	if (!gov_counter(&busy)) {
		ggov.stopped = "the GMU's power counter could not be mapped";
		return false;
	}
	unsigned cap = gov_thermal(lv, n);
	if (!gstate.have) {
		gstate.have = true;
		gstate.busy = busy;
		gstate.seq = seq;
		gstate.copies = copies;
		gstate.time = now;
		return true;
	}
	absolutetime_to_nanoseconds(now - gstate.time, &wall_ns);
	uint64_t busy_ns = (uint64_t)(busy - gstate.busy) * 10000ull / 192ull;      /* 19.2 MHz ticks */
	bool work = seq != gstate.seq || copies != gstate.copies;
	gstate.busy = busy;
	gstate.seq = seq;
	gstate.copies = copies;
	gstate.time = now;
	ggov.ticks++;
	if (wall_ns == 0 || busy_ns > wall_ns + wall_ns / 20) {
		ggov.bad_samples++;                     /* a reset of the counter, or a period too short to tell */
		if (++gstate.bad >= 20) {
			ggov.stopped = "the busy counter counts faster than the wall clock";
			return false;
		}
		return true;
	}
	gstate.bad = 0;
	uint32_t load = (uint32_t)(busy_ns * 100 / wall_ns), wall_ms = (uint32_t)(wall_ns / 1000000);
	ggov.last_load = load;
	ggov.last_busy = (uint32_t)(busy_ns / 1000);
	ggov.load_sum += load;
	ggov.load_hist[load >= 100 ? 9 : load / 10]++;
	/* a counter that never moves while work comes would hold the GPU at its lowest level */
	if (work) {
		gstate.dead_ms = busy_ns == 0 ? gstate.dead_ms + wall_ms : 0;
	}
	if (gstate.dead_ms >= 1000) {
		ggov.stopped = "the busy counter does not count while work is submitted";
		return false;
	}
	unsigned cur = gov_index_for(lv, n, opp_khz), target = cur;
	uint32_t up = (uint32_t)MAX(MIN(gov_up, 99), 2), down = (uint32_t)MIN(MAX(gov_down, 1), (int)up - 1);
	uint64_t want = (uint64_t)opp_khz * load / ((up + down) / 2);
	gstate.idle_ms = !work && load < 2 ? gstate.idle_ms + wall_ms : 0;
	if (gstate.idle_ms >= (uint32_t)MAX(gov_idle_ms, 0)) {
		target = 0;
		gstate.low = 0;
	} else if (load >= 95) {
		target = n - 1;
		gstate.low = 0;
	} else if (load >= up) {
		target = MAX(cur + 1 < n ? cur + 1 : cur, gov_index_for(lv, n, want));
		gstate.low = 0;
	} else if (load < down) {
		if (++gstate.low >= 2) {
			unsigned w = gov_index_for(lv, n, want);
			target = cur > 0 ? MIN(cur - 1, w) : 0;
			gstate.low = 0;
		}
	} else {
		gstate.low = 0;
	}
	if (target > cap) {
		target = cap;
		ggov.capped_ticks++;
	}
	if (ggov.refused_khz != 0) {
		while (target > 0 && lv[target] >= ggov.refused_khz) {
			target--;
		}
	}
	ggov.cap_khz = lv[cap];
	if (lv[target] == opp_khz) {
		return true;
	}
	int e = gpu_opp_set(lv[target], NULL);
	if (e == 0) {
		if (target == 0 && gstate.idle_ms != 0) {
			ggov.idle_drops++;
		} else if (lv[target] > opp_khz || target > cur) {
			ggov.up++;
		} else {
			ggov.down++;
		}
	} else if (e == EBUSY) {
		ggov.refused_busy++;
	} else if (e == EAGAIN || e == EIO || e == EINVAL) {
		ggov.refused_other++;
		if (lv[target] > opp_khz && !opp_broken) {
			ggov.refused_khz = lv[target];   /* stay under it from now on */
			LOG("gov: %u kHz refused (%d): the governor stays under it", lv[target], e);
		}
		if (opp_broken) {
			ggov.stopped = "a change failed and could not be recovered";
			return false;
		}
	} else {
		ggov.stopped = "no stock path or no GPU service";
		return false;
	}
	return true;
}

static void
gov_thread(void *arg __unused, wait_result_t wr __unused)
{
	bzero(&gstate, sizeof(gstate));
	bzero(&gtherm, sizeof(gtherm));
	gtherm.cap = ~0u;
	ggov.stopped = NULL;
	ggov.refused_khz = 0;
	LOG("gov: on (%d ms; up %d%%, down %d%%, idle %d ms; %d to %u kHz; temperature trip %d mC)", gov_ms, gov_up, gov_down,
	    gov_idle_ms, gov_khz_min, gov_khz_max > 0 ? (uint32_t)gov_khz_max : gov_boot_khz, gov_temp_max);
	while (gov_on && service_ready && !opp_broken) {
		if (!gov_tick()) {
			break;
		}
		IOSleep((uint32_t)MIN(MAX(gov_ms, 10), 1000));
	}
	if (ggov.stopped == NULL) {
		ggov.stopped = !gov_on ? "kern.sweet_gpu_gov=0" : !service_ready ? "the GPU service stopped" : "a change failed";
	}
	gov_on = 0;
	for (int i = 0; i < 20 && gov_boot_khz != 0 && opp_khz != gov_boot_khz && service_ready && !opp_broken; i++) {
		if (gpu_opp_set(gov_boot_khz, NULL) != EBUSY) {
			break;
		}
		IOSleep(50);
	}
	LOG("gov: off (%s), at %u kHz", ggov.stopped, opp_khz);
	gov_running = false;
	thread_terminate(current_thread());
}

/* Starts the governor's thread unless it runs (gov_on set by the caller). */
static void
gov_start(void)
{
	thread_t thread;

	if (!OSCompareAndSwap8(false, true, (volatile UInt8 *)&gov_running)) {
		return;
	}
	if (kernel_thread_start(gov_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	} else {
		gov_running = false;
		gov_on = 0;
	}
}

static int
sweet_gpu_gov_sysctl SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	int v = gov_on;
	int error = sysctl_io_number(req, v, sizeof(v), &v, NULL);

	if (error != 0 || req->newptr == USER_ADDR_NULL) {
		return error;
	}
	if (v != 0) {
		if (opp_khz == 0 || opp_broken || !service_ready || gov_boot_khz == 0) {
			return ENODEV;
		}
		if (gov_running && !gov_on) {
			return EBUSY;                   /* still stopping: try again in a moment */
		}
		gov_on = 1;
		gov_start();
	} else {
		gov_on = 0;                             /* the thread gives back the boot rate and ends */
	}
	return 0;
}
SYSCTL_PROC(_kern, OID_AUTO, sweet_gpu_gov, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED,
    NULL, 0, sweet_gpu_gov_sysctl, "I", "sapphire 0346: 1 = the GPU's clock follows how busy it is, capped by temperature");

/* kern.sweet_gpu_khz_max / _min: a rate of the stock table (kHz, or MHz), not above the speed bin's top; 0 = default. */
static int
gov_khz_sysctl(struct sysctl_req *req, int *where, int dflt)
{
	int v = *where != 0 ? *where : dflt;
	int error = sysctl_io_number(req, v, sizeof(v), &v, NULL);
	uint32_t anybin = 0;

	if (error != 0 || req->newptr == USER_ADDR_NULL) {
		return error;
	}
	if (v == 0) {
		*where = 0;
		return 0;
	}
	uint32_t khz = v > 0 ? opp_request_khz((uint32_t)v) : 0;
	(void)PE_parse_boot_argn("gpu_anybin", &anybin, sizeof(anybin));
	if (khz == 0 || (khz > bin_max_khz(bin_fuse) && anybin == 0)) {
		return EINVAL;
	}
	*where = (int)khz;
	return 0;
}

static int
sweet_gpu_khz_max_sysctl SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	return gov_khz_sysctl(req, &gov_khz_max, (int)gov_boot_khz);
}
SYSCTL_PROC(_kern, OID_AUTO, sweet_gpu_khz_max, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED,
    NULL, 0, sweet_gpu_khz_max_sysctl, "I", "sapphire 0346: the governor's top rate in kHz (0: the boot rate)");

static int
sweet_gpu_khz_min_sysctl SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	int error = gov_khz_sysctl(req, &gov_khz_min, 320000);

	if (gov_khz_min == 0) {
		gov_khz_min = 320000;
	}
	return error;
}
SYSCTL_PROC(_kern, OID_AUTO, sweet_gpu_khz_min, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED,
    NULL, 0, sweet_gpu_khz_min_sysctl, "I", "sapphire 0346: the governor's lowest rate in kHz (320000)");

static int
sweet_gpu_temp_max_sysctl SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	int v = gov_temp_max;
	int error = sysctl_io_number(req, v, sizeof(v), &v, NULL);

	if (error != 0 || req->newptr == USER_ADDR_NULL) {
		return error;
	}
	if (v < 50000 || v > 95000) {
		return EINVAL;                          /* 50 C to the stock trip, 95 C */
	}
	gov_temp_max = v;
	return 0;
}
SYSCTL_PROC(_kern, OID_AUTO, sweet_gpu_temp_max, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED,
    NULL, 0, sweet_gpu_temp_max_sysctl, "I", "sapphire 0346: the governor's temperature trip, mC (50000-95000; 5 C over it: 600 MHz at most)");
SYSCTL_INT(_kern, OID_AUTO, sweet_gpu_gov_ms, CTLFLAG_RW | CTLFLAG_LOCKED, &gov_ms, 0,
    "sapphire 0346: the governor's period, ms (10-1000)");
SYSCTL_INT(_kern, OID_AUTO, sweet_gpu_gov_up, CTLFLAG_RW | CTLFLAG_LOCKED, &gov_up, 0,
    "sapphire 0346: busy percent from which the governor goes up");
SYSCTL_INT(_kern, OID_AUTO, sweet_gpu_gov_down, CTLFLAG_RW | CTLFLAG_LOCKED, &gov_down, 0,
    "sapphire 0346: busy percent under which (two periods in a row) the governor goes down");
SYSCTL_INT(_kern, OID_AUTO, sweet_gpu_gov_idle_ms, CTLFLAG_RW | CTLFLAG_LOCKED, &gov_idle_ms, 0,
    "sapphire 0346: ms without work after which the governor goes to the lowest level");

static int
sweet_gpu_gov_stats_sysctl SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	char buf[900];
	size_t off = 0;

	buf[0] = 0;
	OPP_CAT("gov %d%s%s; at %u kHz (boot %u), ladder %d-%u kHz, thermal cap %u kHz, refused from %u kHz; period %d ms, "
	    "up %d%% down %d%% idle %d ms; trip %d mC (and %d: 600 MHz)\n", gov_on, ggov.stopped != NULL ? ", stopped: " : "",
	    ggov.stopped != NULL ? ggov.stopped : "", opp_khz, gov_boot_khz, gov_khz_min,
	    gov_khz_max > 0 ? (uint32_t)gov_khz_max : gov_boot_khz, ggov.cap_khz, ggov.refused_khz, gov_ms, gov_up, gov_down,
	    gov_idle_ms, gov_temp_max, gov_temp_max + 5000);
	OPP_CAT("periods %llu; load last %u%% (%u us busy) mean %llu%%; up %llu down %llu to-idle %llu; refused: GPU busy %llu, "
	    "other %llu; capped by temperature %llu periods; gpu temperature last %d max %d mC, unreadable %llu; busy counter set "
	    "up %llu times, bad samples %llu\n", ggov.ticks, ggov.last_load, ggov.last_busy,
	    ggov.ticks != 0 ? ggov.load_sum / ggov.ticks : 0, ggov.up, ggov.down, ggov.idle_drops, ggov.refused_busy,
	    ggov.refused_other, ggov.capped_ticks, ggov.last_temp, ggov.max_temp, ggov.temp_lost, ggov.counter_setups,
	    ggov.bad_samples);
	OPP_CAT("load per period, %% 0-9 10-19 .. 90-100:");
	for (unsigned i = 0; i < 10; i++) {
		OPP_CAT(" %llu", ggov.load_hist[i]);
	}
	OPP_CAT("\n");
	return SYSCTL_OUT(req, buf, off + 1);
}
SYSCTL_PROC(_kern, OID_AUTO, sweet_gpu_gov_stats, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED,
    NULL, 0, sweet_gpu_gov_stats_sysctl, "A", "sapphire 0346: what the GPU governor saw and did");

/* 0347: on, the vote for the current rate at once (and the DDR 20 ms later); off, INT_MAX. */
static int
ddr_mode_set(int on)
{
	if (opp_lock == NULL) {
		return ENODEV;
	}
	IOLockLock(opp_lock);
	int r = 0;
	if (on != 0 && opp_khz != 0) {
		ddr_mode = 1;
		ddr_voted = 0;
		r = ddr_vote(ddr_khz_for(opp_khz));
	} else if (on == 0 && ddr_mode != 0) {
		ddr_mode = 0;
		r = ddr_vote(0x7fffffffu);
		ddr_voted = 0;
	} else if (on != 0) {
		r = ENODEV;                             /* no stock path: no rate to vote for */
	}
	IOLockUnlock(opp_lock);
	if (r == 0 && gddr.last_khz != 0) {
		IOSleep(20);
		gddr.after_khz = ddr_khz();
		LOG("ddr: BIMC vote %u kHz: the DDR at %u kHz before, %u kHz 20 ms after", gddr.last_khz, gddr.before_khz,
		    gddr.after_khz);
	}
	return r > 0 ? r : r < 0 ? EIO : 0;
}

static int
sweet_gpu_ddr_sysctl SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	int v = ddr_mode;
	int error = sysctl_io_number(req, v, sizeof(v), &v, NULL);

	if (error != 0 || req->newptr == USER_ADDR_NULL) {
		return error;
	}
	return ddr_mode_set(v);
}
SYSCTL_PROC(_kern, OID_AUTO, sweet_gpu_ddr, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED,
    NULL, 0, sweet_gpu_ddr_sysctl, "I", "sapphire 0347: 1 = vote the BIMC (the DDR) with the GPU's level, as kgsl does");
SYSCTL_INT(_kern, OID_AUTO, sweet_gpu_ddr_floor, CTLFLAG_RW | CTLFLAG_LOCKED, &ddr_floor_khz, 0,
    "sapphire 0347: the BIMC vote never goes under this, kHz (0)");

static int
sweet_gpu_ddr_stats_sysctl SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	char buf[400];
	size_t off = 0;
	uint32_t now = ddr_khz();

	buf[0] = 0;
	OPP_CAT("ddr vote %d, floor %d kHz; voted %u kHz for the GPU at %u kHz; votes %llu, refused %llu; last vote %u kHz "
	    "(answer %d): DDR %u kHz before, %u kHz 20 ms after a vote by sysctl; DDR now %u kHz (plan row %u)\n", ddr_mode,
	    ddr_floor_khz, ddr_voted, opp_khz, gddr.votes, gddr.refused, gddr.last_khz, gddr.last_err, gddr.before_khz,
	    gddr.after_khz, now, ddr_plan_khz(now));
	return SYSCTL_OUT(req, buf, off + 1);
}
SYSCTL_PROC(_kern, OID_AUTO, sweet_gpu_ddr_stats, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED,
    NULL, 0, sweet_gpu_ddr_stats_sysctl, "A", "sapphire 0347: the BIMC votes and the DDR's clock before and after");

/* 0348: read, whether RBBM_CLOCK_CNTL has gating on (the switch, if the GPU takes no work); write 0/1, now. */
static int
sweet_gpu_hwcg_sysctl SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	int v = gpu_hwcg;
	int error;

	if (req->newptr == USER_ADDR_NULL && service_ready && service_lock != NULL) {
		IOLockLock(service_lock);
		if (service_ready) {
			v = grd(0xae) == 0xaaa8aa82u;
		}
		IOLockUnlock(service_lock);
	}
	error = sysctl_io_number(req, v, sizeof(v), &v, NULL);
	if (error != 0 || req->newptr == USER_ADDR_NULL) {
		return error;
	}
	if (!service_ready || service_lock == NULL) {
		return ENODEV;
	}
	IOLockLock(service_lock);
	if (!service_ready || !opp_drain(opp_drain_ms)) {
		IOLockUnlock(service_lock);
		return EBUSY;
	}
	gpu_hwcg = v != 0;
	hwcg_set(gpu_hwcg != 0);
	uint32_t cntl = grd(0xae);
	IOLockUnlock(service_lock);
	LOG("hwcg: %s (RBBM_CLOCK_CNTL %08x)", gpu_hwcg ? "on" : "off", cntl);
	return 0;
}
SYSCTL_PROC(_kern, OID_AUTO, sweet_gpu_hwcg, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED,
    NULL, 0, sweet_gpu_hwcg_sysctl, "I", "sapphire 0348: 1 = hardware clock gating of the GPU's blocks, as kgsl sets it");

/* 0370: read, the mode; write 0/1/2, now (the GPU idle) and at every start after. */
static int
sweet_gpu_ubwc_sysctl SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	int v = gpu_ubwc;
	int error;

	error = sysctl_io_number(req, v, sizeof(v), &v, NULL);
	if (error != 0 || req->newptr == USER_ADDR_NULL) {
		return error;
	}
	if (v < 0 || v > 2) {
		return EINVAL;
	}
	if (!service_ready || service_lock == NULL) {
		return ENODEV;
	}
	IOLockLock(service_lock);
	if (!service_ready || !opp_drain(opp_drain_ms)) {
		IOLockUnlock(service_lock);
		return EBUSY;
	}
	gpu_ubwc = v;
	ubwc_set(v);
	uint32_t rb = grd(REG_RB_NC_MODE_CNTL), tp = grd(REG_TPL1_NC_MODE_CNTL), sp = grd(REG_SP_NC_MODE_CNTL), uc = grd(REG_UCHE_MODE_CNTL);
	IOLockUnlock(service_lock);
	LOG("ubwc: mode %d (RB_NC_MODE_CNTL %08x TPL1 %08x SP %08x UCHE_MODE_CNTL %08x); restart backboardd", v, rb, tp, sp, uc);
	return 0;
}
SYSCTL_PROC(_kern, OID_AUTO, sweet_gpu_ubwc, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED,
    NULL, 0, sweet_gpu_ubwc_sysctl, "I", "sapphire 0370: UBWC/tiling configuration, 0 as found, 1 kgsl's (bank bit 14), 2 Linux's (13)");

/* 0370: the five registers now, and as hw_init_regs found them before 0370 first wrote them */
static int
sweet_gpu_ubwc_regs_sysctl SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	char line[256];
	uint32_t now[5] = { 0, 0, 0, 0, 0 };
	bool have = false;

	if (service_ready && service_lock != NULL) {
		IOLockLock(service_lock);
		if (service_ready) {
			for (unsigned i = 0; i < 5; i++) {
				now[i] = grd(ubwc_regs[i]);
			}
			have = true;
		}
		IOLockUnlock(service_lock);
	}
	snprintf(line, sizeof(line), "mode %d, bank bit %u; now RB %08x TPL1 %08x SP %08x UCHE %08x RBBM %08x%s; found %sRB %08x TPL1 %08x "
	    "SP %08x UCHE %08x RBBM %08x", gpu_ubwc, ubwc_hbb(), now[0], now[1], now[2], now[3], now[4], have ? "" : " (no GPU)",
	    ubwc_found_ok ? "" : "(not yet) ", ubwc_found[0], ubwc_found[1], ubwc_found[2], ubwc_found[3], ubwc_found[4]);
	return SYSCTL_OUT(req, line, strlen(line) + 1);
}
SYSCTL_PROC(_kern, OID_AUTO, sweet_gpu_ubwc_regs, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED,
    NULL, 0, sweet_gpu_ubwc_regs_sysctl, "A", "sapphire 0370: RB/TPL1/SP_NC_MODE_CNTL, UCHE_MODE_CNTL, RBBM_NC_MODE_CNTL now and as found");

/*
 * GPU memory (0151): the gigabyte from 0x1_0000_0000 to 0x1_4000_0000 is DDR that nobody uses
 * here -- xnu runs from 0x1_4000_0000 (the shim clamps it there), the fixed firmware carve-outs
 * end at 0x65b00000, the shim's payload is below 4 GB -- so Mesa's buffers come from it, mapped
 * write-combined into the program (as the panel's own framebuffer surface is; the memory is not
 * the kernel's, so no cache mode of its pages changes) and seen by the GPU without any cache
 * maintenance: no buffer is cleaned before a submission any more. Contiguous runs of 4 KB pages,
 * next fit. boot-arg gpu_pool=0: the cacheable kernel buffers of 0149 instead. GET_PARAM 0x1000
 * tells the program which (its libdrm then asks for write-combined mappings).
 */
#define GPOOL_PA            0x100000000ull
#define GPOOL_SIZE          0x40000000ull
#define GPOOL_PAGES         (GPOOL_SIZE >> 12)
static uint64_t gpool_bits[GPOOL_PAGES / 64];
static uint64_t gpool_next;
static int gpool_on = -1;

static bool
gpool_enabled(void)
{
	if (gpool_on < 0) {
		uint32_t v = 1;
		(void)PE_parse_boot_argn("gpu_pool", &v, sizeof(v));
		gpool_on = v != 0;
		LOG("msm: buffers %s", gpool_on ? "from the GPU pool 0x100000000-0x140000000, write-combined" :
		    "cacheable kernel memory (gpu_pool=0)");
	}
	return gpool_on;
}

static void
bits_mark(uint64_t *bits, uint64_t p0, uint64_t n, bool used)
{
	for (uint64_t p = p0; p < p0 + n; p++) {
		if (used) {
			bits[p >> 6] |= 1ull << (p & 63);
		} else {
			bits[p >> 6] &= ~(1ull << (p & 63));
		}
	}
}

/* n contiguous pages of the pool: their physical address, or 0. service_lock held. */
static uint64_t
gpool_alloc(uint64_t n)
{
	for (int pass = 0; pass < 2; pass++) {
		uint64_t start = pass == 0 ? gpool_next : 0, end = pass == 0 ? GPOOL_PAGES : gpool_next, run = 0;
		for (uint64_t p = start; p < end; p++) {
			if ((p & 63) == 0 && run == 0 && gpool_bits[p >> 6] == ~0ull) {
				p += 63;
				continue;
			}
			run = ((gpool_bits[p >> 6] >> (p & 63)) & 1) ? 0 : run + 1;
			if (run == n) {
				bits_mark(gpool_bits, p + 1 - n, n, true);
				gpool_next = p + 1;
				return GPOOL_PA + ((p + 1 - n) << 12);
			}
		}
	}
	return 0;
}

/* Wait for fence f, without the service lock, up to ms; false if it never came. */
static bool
msm_wait(uint32_t f, uint64_t ms)
{
	uint64_t deadline, now;

	OSAddAtomic64(1, (volatile SInt64 *)&gdiag.wait_n);             /* 0336: who waits for the GPU, and how long */
	if (msm_fence_done(f)) {
		OSAddAtomic64(1, (volatile SInt64 *)&gdiag.wait_done);
		return true;
	}
	if (ms == 0) {
		return false;
	}
	uint64_t diag_t0 = mach_absolute_time();
	clock_interval_to_deadline((uint32_t)ms, kMillisecondScale, &deadline);
	while (!msm_fence_done(f)) {
		now = mach_absolute_time();
		if (now >= deadline) {
			gdiag_since(&gdiag.wait, diag_t0);
			if (msm_gpu_idle()) {
				static uint32_t said;
				if (said++ < 8) {
					LOG("msm: fence %u not written (slot %08x) but the GPU is idle with the ring read: done", f,
					    *(volatile uint32_t *)(pool + OFF_MSM_FENCE));
				}
				return true;
			}
			return false;
		}
		OSAddAtomic64(1, (volatile SInt64 *)&gdiag.wait_loops);
		delay_for_interval(50, NSEC_PER_USEC);
	}
	gdiag_since(&gdiag.wait, diag_t0);
	return true;
}

/* How long from now a time of msm's is (CLOCK_MONOTONIC = mach_continuous_time here), in ms; at most 2 s. */
static uint64_t
msm_ms_until(const struct msm_timespec *when)
{
	uint64_t now;
	int64_t left;

	absolutetime_to_nanoseconds(mach_continuous_time(), &now);
	left = when->seconds * (int64_t)NSEC_PER_SEC + when->nanoseconds - (int64_t)now;
	if (left <= 0) {
		return 0;
	}
	return (uint64_t)left / NSEC_PER_MSEC > 2000 ? 2000 : (uint64_t)left / NSEC_PER_MSEC;
}

/*
 * 0317: a submission that never finishes. The GPU is reset and the service goes on (Linux
 * a6xx_recover for a GPU without a GMU: drain the bus, RBBM software reset, hw_init, the CP
 * started again on an empty ring, the zap shader resumed and secure mode left), keeping the page
 * tables, the SQE microcode already in the pool and every client's buffers. What was in flight
 * is given up: both fence words jump to the last number handed out, so every wait ends, and
 * GET_PARAM FAULTS counts the resets (a client that sees it rise knows its work did not run, as
 * Linux's msm says with the same parameter). boot-arg gpu_norecover=1: the old behaviour, the
 * service stops for good. Called with service_lock held or not (`locked`).
 */
static uint32_t gpu_faults;

static bool
gpu_recover(void)
{
	uint64_t res = 0;
	long long st;

	bus_drain();
	gwr(REG_RBBM_SW_RESET_CMD, 1);
	(void)grd(REG_RBBM_SW_RESET_CMD);
	IODelay(100);
	gwr(REG_RBBM_SW_RESET_CMD, 0);
	(void)grd(REG_RBBM_SW_RESET_CMD);
	IODelay(100);
	hw_init_regs();
	gwr64(REG_CP_SQE_INSTR_BASE, POOL_PA + OFF_SQE);
	gwr64(REG_CP_RB_BASE, POOL_PA + OFF_RING);
	gwr(REG_CP_RB_CNTL, 12 | (2 << 8) | (1u << 27));
	ring_wptr = 0;
	gwr(REG_CP_RB_WPTR, 0);
	gwr(REG_CP_SQE_CNTL, 1);
	emit(pkt7(CP_ME_INIT, 8));
	emit(0x2f);
	emit(3);
	emit(0x20000000);
	emit(0);
	emit(0);
	emit(0);
	emit(0);
	emit(0);
	if (!submit("recover: CP_ME_INIT", 100)) {
		return false;
	}
	if (recover_zap) {
		st = scm(SCM_FN(SCM_SVC_BOOT, SCM_BOOT_SET_REMOTE_STATE), 2, 0, GPU_PAS_ID, 0, &res);
		LOG("msm: recover: zap resume: ret %llx res %llx", st, res);
		emit(pkt7(CP_SET_SECURE_MODE, 1));
		emit(0);
		if (!submit("recover: CP_SET_SECURE_MODE 0", 100)) {
			return false;
		}
	} else {
		gwr(REG_RBBM_SECVID_TRUST_CNTL, 0);
	}
	*(volatile uint32_t *)(pool + OFF_MSM_FENCE) = msm_seq;
	*(volatile uint32_t *)(pool + OFF_FENCE) = fence_seq;
	gmb();
	smmu_fault("msm: recover");
	return true;
}

static void
msm_hung(const char *who, uint32_t fence, bool locked)
{
	uint32_t norecover = 0;

	if (!locked) {
		IOLockLock(service_lock);
	}
	if (service_ready) {
		service_ready = false;
		LOG("msm: %s: submission %u did not finish in 2 s (done: %u): RBBM_STATUS %08x CP_HW_FAULT %08x rptr %u wptr %u",
		    who, fence, *(volatile uint32_t *)(pool + OFF_MSM_FENCE), grd(REG_RBBM_STATUS), grd(REG_CP_HW_FAULT),
		    grd(REG_CP_RB_RPTR), ring_wptr % RING_DWORDS);
		smmu_fault("msm");
		PE_parse_boot_argn("gpu_norecover", &norecover, sizeof(norecover));
		if (norecover) {
			LOG("msm: gpu_norecover=1: the GPU service stops");
		} else if (gpu_recover()) {
			gpu_faults++;
			service_ready = true;
			LOG("*** msm: the GPU was reset (%u so far); work up to %u given up, the service goes on ***", gpu_faults, msm_seq);
		} else {
			LOG("msm: the GPU did not come back from its reset: the GPU service stops; RBBM_STATUS %08x", grd(REG_RBBM_STATUS));
		}
	}
	if (!locked) {
		IOLockUnlock(service_lock);
	}
}

/*
 * sapphire 0342: buffers of the GPU pool that their program gives back are kept for it, with their memory, their GPU
 * addresses and their page table entries, and handed out again for a GEM_NEW of the same size: no search of the pool,
 * no mapping, and at the other end no unmapping and no TLB invalidation (measured with 0336: 6575 GEM_NEW and 6670
 * frees in 15 minutes, 0.79 s mapping and 0.85 s freeing, beside 1.44 s zeroing). HackiOS's driver keeps them the
 * same way (hakios.gpu.cpp, keep / dropKept).
 * kern.sweet_gpu_bocache (boot-arg gpu_bocache), 0 by default:
 *   0  as before: nothing is kept;
 *   1  kept, and zeroed again when handed out (what GEM_NEW has always answered with);
 *   2  kept, and handed out as it is: the program finds in a new buffer what an older buffer of its own held. Never
 *      another program's: a buffer is only kept for the client that had it. Faster (the zeroing is most of a large
 *      buffer's cost), and to be tried with care: without zeroing, 0205 showed white frames (0215).
 * A client keeps at most MSM_KEEP buffers and 64 MB, none over 16 MB, each for 2 s at most (looked at by every
 * submission); they are freed when it closes, and when the pool has no room for a new buffer.
 */
#define MSM_KEEP            32u
#define MSM_KEEP_BYTES      (64ull << 20)
#define MSM_KEEP_ONE        (16ull << 20)
#define MSM_KEEP_MS         2000u
static int gpu_bocache;
static struct {
	uint64_t kept, reused, reused_bytes, aged, flushed;
} gkeep;

static int
sweet_gpu_bocache_stats_sysctl SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	char line[224];

	snprintf(line, sizeof(line), "buffers kept (kern.sweet_gpu_bocache %d): kept %llu, handed out again %llu (%llu bytes), freed for their age %llu, "
	    "freed to make room %llu\n", gpu_bocache, gkeep.kept, gkeep.reused, gkeep.reused_bytes, gkeep.aged, gkeep.flushed);
	return SYSCTL_OUT(req, line, strlen(line) + 1);
}
SYSCTL_PROC(_kern, OID_AUTO, sweet_gpu_bocache_stats, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED,
    NULL, 0, sweet_gpu_bocache_stats_sysctl, "A", "sapphire 0342: the GPU buffers kept for their programs");
SYSCTL_INT(_kern, OID_AUTO, sweet_gpu_bocache, CTLFLAG_RW | CTLFLAG_LOCKED, &gpu_bocache, 0,
    "sapphire 0342: pool buffers given back are kept for their program: 0 = no (as before), 1 = and zeroed again when reused, 2 = and reused as they are");

struct msm_bo {
	IOMemoryDescriptor *md;                 /* NULL: free handle */
	SweetSurface       *surf;
	uint8_t            *kva;                /* kernel address of byte 0 (cache maintenance) */
	uint64_t            va, size;
	uint32_t            flags, seed;
	uint64_t            cleaned;            /* surfaces: when they were last cleaned */
	bool                mapped;             /* the program has mapped it: it may have written it */
	bool                pooled;             /* GPU pool memory at pa (write-combined, never cleaned) */
	uint64_t            pa;
	uint32_t            fence;              /* the last submission that named it */
	bool                user;               /* 0380: the program's own pages (userMemory), wired, cacheable */
	ppnum_t            *pages;              /* ... their physical pages, size >> PAGE_SHIFT of them */
	uint32_t            wfence;             /* ... the last submission that wrote them, until they are settled (0: none) */
};
#define MSM_LEFT            256u                /* buffers given back while the GPU still had them */

/*
 * sapphire 0339: what the CPU caches hold of a surface, kept account of, so that a submission maintains only what a
 * processor may have touched. kern.sweet_gpu_flush (boot-arg gpu_flush):
 *   0  as before: a submission cleans and invalidates (dc civac) all of every surface it names whose seed moved or
 *      that its program has mapped. HakiosMetal maps every surface it imports (Gpu::surfaceBo), so that is the
 *      frame's own surface, 4.9 MB and 1.5 ms inside the ring's lock, on every submission (measured with 0336).
 *   1  by the surface's own account (SweetSurface.h: cpuEpoch, cleanEpoch, invalEpoch). There are two hazards:
 *        - the GPU reads memory that a processor wrote and still holds dirty in a cache. Cleaned (dc cvac) when
 *          cleanEpoch is not cpuEpoch;
 *        - the GPU writes memory of which a cache holds a line: a dirty line is written back later over what the
 *          GPU drew, a clean one is what a processor reads afterwards in place of it. Cleaned and invalidated (dc
 *          civac) when invalEpoch is not cpuEpoch.
 *      Otherwise nothing: what the GPU writes puts no line in a cache, so a surface that only the GPU touches is
 *      maintained once. Whether a submission writes a buffer is what it says in the buffer's flags
 *      (MSM_SUBMIT_BO_WRITE, which HakiosMetal sets for what it draws into; neither flag counts as writing).
 *      cpuEpoch moves at every access the kernel is told of: IOSurfaceLock and IOSurfaceUnlock (traps here, each
 *      one comes through the kernel), the buffer's client memory being mapped or unmapped, GEM_CPU_PREP and
 *      GEM_CPU_FINI, the kernel's own CPU copies. What it is NOT told of is a program reading or writing the
 *      pixels through the mapping its GPU client gave it without locking the surface: HakiosMetal does that in
 *      Texture::replace and Texture::read (-replaceRegion: and -getBytes: of a texture made straight over an
 *      IOSurface). 1 is as good as the absence of those, which is why 0 is the default, and 2 is how to find out.
 *   2  as 1, and a surface's buffer is not mapped into its program (the client memory is refused; one that is
 *      mapped already stays so). The program then reaches the pixels through IOSurface alone, with its locks, and
 *      the account is complete. HakiosMetal takes a surface it cannot map (Bo::cpu() null) and refuses by name, on
 *      its console, a -replaceRegion: or -getBytes: of it: a session without such a line lost nothing with 1.
 * It may be changed at any time: the account is kept in every mode, so a change costs at most one maintenance more.
 */
#define MSM_SUBMIT_BO_READ  0x0001u
#define MSM_SUBMIT_BO_WRITE 0x0002u
static int gpu_flush_mode;
static struct {
	uint64_t looked, nothing, cleaned, cleaned_bytes, flushed, flushed_bytes, maps_refused;
} gflush;

/* A submission names surface buffer b with these flags (mode 1 or 2). service_lock held. */
static void
msm_surface_sync(struct msm_bo *b, uint32_t flags)
{
	SweetSurface *s = b->surf;
	/* read before the maintenance: an access announced while it runs is maintained again the next time */
	uint32_t epoch = s->cpuEpoch;
	bool writes = (flags & MSM_SUBMIT_BO_WRITE) != 0 || (flags & MSM_SUBMIT_BO_READ) == 0;

	gflush.looked++;
	if (s->mapCache == kIOMapWriteCombineCache || (writes ? s->invalEpoch == epoch : s->cleanEpoch == epoch)) {
		gflush.nothing++;                       /* (write-combined: the panel's own memory, which no cache holds) */
		return;
	}
	uint64_t t0 = mach_absolute_time();
	if (writes) {
		sweet_dcache_flush((vm_offset_t)b->kva, (unsigned)b->size);
		s->invalEpoch = epoch;
		gflush.flushed++;
		gflush.flushed_bytes += b->size;
	} else {
		sweet_dcache_clean((vm_offset_t)b->kva, (unsigned)b->size);
		gflush.cleaned++;
		gflush.cleaned_bytes += b->size;
	}
	s->cleanEpoch = epoch;
	gdiag_since(&gdiag.sub_flush, t0);              /* 0336's "cache flush" counts both kinds, and their bytes */
	gdiag.sub_flush_bytes += b->size;
}

/* For SweetFramebuffer: whether surfaces go by their own account (kern.sweet_gpu_flush 1 or 2). */
int sapphire_gpu_flush_tracking(void);

int
sapphire_gpu_flush_tracking(void)
{
	return gpu_flush_mode != 0;
}

static int
sweet_gpu_flush_stats_sysctl SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	char line[320];

	snprintf(line, sizeof(line), "surface account (kern.sweet_gpu_flush %d): named %llu, nothing to do %llu, cleaned %llu (%llu bytes), "
	    "cleaned and invalidated %llu (%llu bytes), mappings refused %llu\n", gpu_flush_mode, gflush.looked, gflush.nothing,
	    gflush.cleaned, gflush.cleaned_bytes, gflush.flushed, gflush.flushed_bytes, gflush.maps_refused);
	return SYSCTL_OUT(req, line, strlen(line) + 1);
}
SYSCTL_PROC(_kern, OID_AUTO, sweet_gpu_flush_stats, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED,
    NULL, 0, sweet_gpu_flush_stats_sysctl, "A", "sapphire 0339: what submissions did to the surfaces they named, by the surfaces' account");
SYSCTL_INT(_kern, OID_AUTO, sweet_gpu_flush, CTLFLAG_RW | CTLFLAG_LOCKED, &gpu_flush_mode, 0,
    "sapphire 0339: cache maintenance of surfaces at a submission: 0 = every mapped one every time (as before), 1 = by the surface's account, 2 = as 1 and surfaces are not mapped through the GPU client");

/*
 * sapphire 0380: the program's own memory as a GPU buffer (request 0x7d), what Metal's -newBufferWithBytesNoCopy: is:
 * the caller's pages themselves, which the GPU reads where the program wrote them and writes where the program will
 * read them. QuartzCore wraps an image's pages so and keeps a texture over them (its client-storage images:
 * CA::OGL::MetalContext::copy_image_to_texture, 20H392), and never uploads that image again; HakiosMetal made a copy of
 * the bytes when the buffer was made instead, which showed the image as it was then and nothing after
 * (ios-sapphire dev-notes/investigacion/26).
 *
 * The pages: IOMemoryDescriptor::withAddressRange over the client's task, prepared for reading and writing. That wires
 * them where they are, mapped as the program has them, until the buffer is freed (freeBo, after the last submission
 * that named it), and marks each one dirty for the VM (vm_object_iopl_request: SET_PAGE_DIRTY for a request that may
 * write), so that what the VM knows of their contents does not rest on the pmap's modified bit while the buffer holds
 * them. Only pages the pmap manages are taken. Their GPU addresses are given 4 KB at a time, as any buffer's (place).
 *
 * The caches. This GPU reads and writes memory behind the processor's caches (no IO coherence: the stock kgsl SMMU node
 * of this phone has no dma-coherent), and these pages are cacheable in every mapping of them. Cleaning all of them at
 * every submission is what would cost: QuartzCore keeps 7 to 29 MB of images in client storage a command buffer once
 * the alignment is declared (traces of merge-6), 2 to 9 ms of cleaning at the ~3 GB/s 0336 measured for dc civac, at
 * every submission, for pages nobody touched. So each page keeps an account, in the pmap: pmap_test_and_clear_modify
 * (osfmk/arm/pmap/pmap.c, this patch) says whether any mapping wrote the page since it was last asked and makes the
 * next write fault again, in one step under the page's lock, and only those pages are maintained:
 *   - a submission that reads the buffer: each page written since is cleaned (dc cvac; 0339's H1);
 *   - a submission that writes it (MSM_SUBMIT_BO_WRITE, or neither flag): each page written since is cleaned and
 *     invalidated (dc civac; H2); and once that submission is done, before the program reads, the whole buffer is
 *     cleaned and invalidated again (a line a processor fetched while the GPU wrote, by speculation or not, would be
 *     what it reads: H3) -- at the first WAIT_FENCE or CPU_PREP of the client that finds it done, or when it is freed;
 *   - when the buffer is made: every page's account started, then every line cleaned and invalidated.
 * Nothing here maps the pages into the kernel: they are maintained through the kernel's physical map, by line.
 *
 * kern.sweet_gpu_usermem_track 0 (boot-arg gpu_usermem_track=0): every page of every such buffer maintained at every
 * submission instead -- the plain rule, to hold the account to. kern.sweet_gpu_usermem 0 (boot-arg gpu_usermem=0): the
 * request refused and GET_PARAM 0x1001 answers 0, and HakiosMetal answers nil for a no-copy buffer (QuartzCore then
 * uploads its images). kern.sweet_gpu_usermem_stats: what was made and maintained.
 * What this does not cover, as 0339 does not: a processor writing the pages while the GPU reads them (nothing orders
 * the two, in Metal either); and a page with a mapping wired in the pmap, which the account cannot write-protect: it
 * answers written every time and is maintained every time.
 */
extern "C" boolean_t pmap_test_and_clear_modify(ppnum_t pn);   /* osfmk/arm/pmap/pmap.c */
extern "C" void pmap_set_modify(ppnum_t pn);
extern "C" boolean_t pmap_valid_page(ppnum_t pn);
extern "C" vm_offset_t phystokv(uint64_t pa);

#define MSM_USER_BYTES      (512ull << 20)      /* what one client may hold wired for the GPU at a time */
static int gpu_usermem = 1, gpu_usermem_track = 1;
static struct {
	uint64_t made, refused, freed, bytes, peak;
	uint64_t looked, cleaned, flushed, settled;
	struct gdiag_time make, sync, settle;
} guser;

/* the data cache's line, from CTR_EL0 (DminLine) */
static vm_offset_t
msm_user_line(void)
{
	static vm_offset_t line;

	if (line == 0) {
		uint64_t ctr;
		__asm__ volatile ("mrs %0, ctr_el0" : "=r"(ctr));
		line = (vm_offset_t)4 << ((ctr >> 16) & 0xf);
	}
	return line;
}

/* page PN through the kernel's physical map, line by line: cleaned, or cleaned and invalidated. No barrier: the
 * caller's DSB before and after its pages. */
static void
msm_user_lines(ppnum_t pn, bool invalidate)
{
	const vm_offset_t line = msm_user_line();
	vm_offset_t va = phystokv((uint64_t)pn << PAGE_SHIFT);
	const vm_offset_t end = va + PAGE_SIZE;

	for (; va < end; va += line) {
		if (invalidate) {
			__asm__ volatile ("dc civac, %0" :: "r"(va) : "memory");
		} else {
			__asm__ volatile ("dc cvac, %0" :: "r"(va) : "memory");
		}
	}
}

static inline void
msm_user_dsb(void)
{
	__asm__ volatile ("dsb sy" ::: "memory");
}

/* Before a submission that names B runs: the pages a processor wrote since, as the submission's flags for B say.
 * service_lock held. */
static void
msm_user_sync(struct msm_bo *b, uint32_t flags)
{
	const bool writes = (flags & MSM_SUBMIT_BO_WRITE) != 0 || (flags & MSM_SUBMIT_BO_READ) == 0;
	const uint64_t n = b->size >> PAGE_SHIFT, t0 = mach_absolute_time();
	uint64_t done = 0;

	msm_user_dsb();
	for (uint64_t p = 0; p < n; p++) {
		if (gpu_usermem_track == 0 || pmap_test_and_clear_modify(b->pages[p])) {
			msm_user_lines(b->pages[p], writes);
			done++;
		}
	}
	msm_user_dsb();
	guser.looked += n;
	if (writes) {
		guser.flushed += done;
	} else {
		guser.cleaned += done;
	}
	gdiag_since(&guser.sync, t0);
}

/* B was written by a submission that is done: every line of it cleaned and invalidated, so that what the processor
 * reads next is what the GPU wrote. The caller clears b->wfence. service_lock held. */
static void
msm_user_settle(struct msm_bo *b)
{
	const uint64_t n = b->size >> PAGE_SHIFT, t0 = mach_absolute_time();

	msm_user_dsb();
	for (uint64_t p = 0; p < n; p++) {
		msm_user_lines(b->pages[p], true);
	}
	msm_user_dsb();
	guser.settled++;
	gdiag_since(&guser.settle, t0);
}

/* B's pages back to the program, the GPU done with them: settled if it wrote them, the pmap's modified bit given back
 * (the VM's account of the pages was theirs before the buffer took it), unwired. freeBo's, service_lock held. */
static void
msm_user_free(struct msm_bo *b)
{
	const uint64_t n = b->size >> PAGE_SHIFT;

	if (b->wfence != 0) {
		msm_user_settle(b);
	}
	for (uint64_t p = 0; p < n; p++) {
		pmap_set_modify(b->pages[p]);
	}
	b->md->complete();
	IODelete(b->pages, ppnum_t, n);
	b->pages = NULL;
	guser.freed++;
	guser.bytes -= b->size;
}

static int
sweet_gpu_usermem_stats_sysctl SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	char line[512];

	snprintf(line, sizeof(line), "program memory buffers (kern.sweet_gpu_usermem %d, track %d): made %llu, refused %llu, freed %llu, "
	    "now %llu bytes (peak %llu); submissions looked at %llu pages, cleaned %llu, cleaned and invalidated %llu; settled after "
	    "the GPU wrote %llu; made %llu in %llu us (max %llu), synced %llu in %llu us (max %llu), settled %llu in %llu us (max %llu)\n",
	    gpu_usermem, gpu_usermem_track, guser.made, guser.refused, guser.freed, guser.bytes, guser.peak, guser.looked, guser.cleaned,
	    guser.flushed, guser.settled, guser.make.n, guser.make.ns / 1000, guser.make.max / 1000, guser.sync.n, guser.sync.ns / 1000,
	    guser.sync.max / 1000, guser.settle.n, guser.settle.ns / 1000, guser.settle.max / 1000);
	return SYSCTL_OUT(req, line, strlen(line) + 1);
}
SYSCTL_PROC(_kern, OID_AUTO, sweet_gpu_usermem_stats, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED,
    NULL, 0, sweet_gpu_usermem_stats_sysctl, "A", "sapphire 0380: the GPU buffers made of programs' own memory, and their cache maintenance");
SYSCTL_INT(_kern, OID_AUTO, sweet_gpu_usermem, CTLFLAG_RW | CTLFLAG_LOCKED, &gpu_usermem, 0,
    "sapphire 0380: a program's own memory as a GPU buffer (request 0x7d): 1 = served (default), 0 = refused");
SYSCTL_INT(_kern, OID_AUTO, sweet_gpu_usermem_track, CTLFLAG_RW | CTLFLAG_LOCKED, &gpu_usermem_track, 0,
    "sapphire 0380: 1 = only the pages a processor wrote since are maintained at a submission (default), 0 = every page");

/*
 * sapphire 0394: an IOSurfaceLock of a surface the GPU wrote, by any process. The GPU writes memory behind the
 * processor's caches, and 0339's account invalidates a surface only before a submission writes it: a line a processor
 * fetched while the GPU wrote (by speculation, a prefetch, or a read of another word of the line) was what a later
 * read of the surface got in place of what the GPU drew, and a later partial write merged into such a stale line put
 * its old bytes back over the GPU's (ios-sapphire dev-notes/investigacion/27 §2.5.2). Before the kernel flushed the
 * caches at every idle entry, those lines did not live long; with 0344/0351 they do.
 * IOSurfaceLock is where a processor says it is about to touch the pixels, and on Apple's platforms it waits for the
 * GPU's pending writes to the surface unless kIOSurfaceLockAvoidSync says not to. So here: when a submission wrote the
 * surface (gpuWriteFence, set by submitIt) since the last settle (settledFence), wait for it (up to 2 s, as every msm
 * wait), then clean and invalidate the surface's lines through the kernel's mapping and note it in 0339's account
 * (invalEpoch, cleanEpoch: nothing of it is in a cache now). With AvoidSync, a submission not done yet is left to the
 * caller. Write-combined surfaces (the scanout pool's) are skipped: no cache holds them.
 * kern.sweet_gpu_lock_settle (boot-arg gpu_lock_settle): 1 (default) as above, 0 as before. kern.sweet_gpu_lock_settle_stats.
 */
static int gpu_lock_settle = -1;
static struct {
	uint64_t locks, settled, settled_bytes, waited, timeouts, not_waited;
	struct gdiag_time time;
} glock;

void
sapphire_gpu_surface_settle(SweetSurface *s, bool avoidSync)
{
	if (gpu_lock_settle < 0) {
		int on = 1;
		(void)PE_parse_boot_argn("gpu_lock_settle", &on, sizeof(on));
		gpu_lock_settle = on != 0;
	}
	OSAddAtomic64(1, (volatile SInt64 *)&glock.locks);
	const uint32_t f = s->gpuWriteFence;
	if (gpu_lock_settle == 0 || f == 0 || f == s->settledFence || s->lowPool || s->mapCache == kIOMapWriteCombineCache || !service_ready ||
	    pool == NULL || s->memory == NULL) {
		return;
	}
	if (!msm_fence_done(f)) {
		if (avoidSync) {
			OSAddAtomic64(1, (volatile SInt64 *)&glock.not_waited);
			return;
		}
		OSAddAtomic64(1, (volatile SInt64 *)&glock.waited);
		if (!msm_wait(f, 2000)) {
			OSAddAtomic64(1, (volatile SInt64 *)&glock.timeouts);
			return;
		}
	}
	const uintptr_t kva = s->kernelAddress();
	if (kva == 0) {
		return;
	}
	const uint64_t t0 = mach_absolute_time();
	const uint32_t epoch = s->cpuEpoch;             /* (read before: an access announced while it runs is maintained again) */
	const uint64_t length = s->memory->getLength();
	sweet_dcache_flush((vm_offset_t)kva, (unsigned)length);
	s->invalEpoch = epoch;
	s->cleanEpoch = epoch;
	s->settledFence = f;
	OSAddAtomic64(1, (volatile SInt64 *)&glock.settled);
	OSAddAtomic64((SInt64)length, (volatile SInt64 *)&glock.settled_bytes);
	gdiag_since(&glock.time, t0);
}

static int
sweet_gpu_lock_settle_stats_sysctl SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	char line[320];

	snprintf(line, sizeof(line), "surface locks (kern.sweet_gpu_lock_settle %d): %llu, settled after the GPU wrote %llu (%llu bytes, %llu us, "
	    "max %llu us), waited for the GPU %llu (timed out %llu), not waited (AvoidSync) %llu\n", gpu_lock_settle, glock.locks, glock.settled,
	    glock.settled_bytes, glock.time.ns / 1000, glock.time.max / 1000, glock.waited, glock.timeouts, glock.not_waited);
	return SYSCTL_OUT(req, line, strlen(line) + 1);
}
SYSCTL_PROC(_kern, OID_AUTO, sweet_gpu_lock_settle_stats, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED,
    NULL, 0, sweet_gpu_lock_settle_stats_sysctl, "A", "sapphire 0394: IOSurfaceLocks of surfaces the GPU wrote: waited for and invalidated");
SYSCTL_INT(_kern, OID_AUTO, sweet_gpu_lock_settle, CTLFLAG_RW | CTLFLAG_LOCKED, &gpu_lock_settle, 0,
    "sapphire 0394: an IOSurfaceLock waits for the GPU's writes to the surface and invalidates its lines: 1 = yes (default), 0 = as before");

class SapphireGPU : public IOService {
	OSDeclareDefaultStructors(SapphireGPU);
};

OSDefineMetaClassAndStructors(SapphireGPU, IOService);

class SapphireGPUClient : public IOUserClient {
	OSDeclareDefaultStructors(SapphireGPUClient);

public:
	char            fName[32];
	struct msm_bo  *fBo;                    /* handle n is fBo[n - 1] */
	struct msm_bo  *fLeft;                  /* MSM_LEFT of them: kept until their fence */
	uint32_t        fCount, fSubmits, fQueue;
	uint64_t        fBytes;
	struct msm_bo  *fKeep;                  /* 0342: MSM_KEEP buffers given back and kept (service_lock) */
	uint64_t       *fKeepWhen;              /* ... and when each was */
	uint64_t        fKeepBytes;
	task_t          fTask;                  /* 0380: whose memory userMemory takes */
	uint64_t        fUserBytes;             /* ... how much of it this client holds (service_lock) */
	uint32_t        fUnsettled;             /* ... its buffers the GPU wrote and the caches have not been settled for */
	bool keepBo(struct msm_bo *b);
	uint32_t dropKept(uint64_t before);

	bool initWithTask(task_t owningTask, void *securityID, UInt32 type) APPLE_KEXT_OVERRIDE;
	IOReturn clientClose(void) APPLE_KEXT_OVERRIDE;
	IOReturn clientMemoryForType(UInt32 type, IOOptionBits *options, IOMemoryDescriptor **memory) APPLE_KEXT_OVERRIDE;
	IOReturn externalMethod(uint32_t selector, IOExternalMethodArguments *args, IOExternalMethodDispatch *dispatch,
	    OSObject *target, void *reference) APPLE_KEXT_OVERRIDE;

	struct msm_bo *bo(uint32_t handle);
	struct msm_bo *slot(uint32_t *handle);
	bool place(struct msm_bo *b);
	void drop(struct msm_bo *b);
	void freeBo(struct msm_bo *b);
	void reap(void);
	IOReturn gemNew(struct msm_gem_new *r);
	IOReturn gemInfo(struct msm_gem_info *r);
	IOReturn surface(struct msm_surface *r);
	IOReturn userMemory(struct msm_user_memory *r);
	void settleUser(void);
	IOReturn submitIt(struct msm_gem_submit *r);
};

OSDefineMetaClassAndStructors(SapphireGPUClient, IOUserClient);

bool
SapphireGPUClient::initWithTask(task_t owningTask, void *securityID, UInt32 type)
{
	if (!IOUserClient::initWithTask(owningTask, securityID, type)) {
		return false;
	}
	proc_selfname(fName, sizeof(fName));
	fBo = IONew(struct msm_bo, MSM_BUFFERS);
	fLeft = IONew(struct msm_bo, MSM_LEFT);
	if (fBo == NULL || fLeft == NULL) {
		if (fBo != NULL) {
			IODelete(fBo, struct msm_bo, MSM_BUFFERS);
			fBo = NULL;
		}
		if (fLeft != NULL) {
			IODelete(fLeft, struct msm_bo, MSM_LEFT);
			fLeft = NULL;
		}
		return false;
	}
	bzero(fBo, sizeof(struct msm_bo) * MSM_BUFFERS);
	bzero(fLeft, sizeof(struct msm_bo) * MSM_LEFT);
	fCount = fSubmits = fQueue = 0;
	fBytes = 0;
	/* 0342: room for the buffers kept (without it nothing is kept), and the boot-arg, once */
	fKeep = IONew(struct msm_bo, MSM_KEEP);
	fKeepWhen = IONew(uint64_t, MSM_KEEP);
	fKeepBytes = 0;
	if (fKeep != NULL && fKeepWhen != NULL) {
		bzero(fKeep, sizeof(struct msm_bo) * MSM_KEEP);
	}
	static bool keep_arg;
	if (!keep_arg) {
		keep_arg = true;
		(void)PE_parse_boot_argn("gpu_bocache", &gpu_bocache, sizeof(gpu_bocache));
	}
	fTask = owningTask;                             /* 0380 */
	fUserBytes = 0;
	fUnsettled = 0;
	static bool user_arg;
	if (!user_arg) {
		user_arg = true;
		(void)PE_parse_boot_argn("gpu_usermem", &gpu_usermem, sizeof(gpu_usermem));
		(void)PE_parse_boot_argn("gpu_usermem_track", &gpu_usermem_track, sizeof(gpu_usermem_track));
	}
	LOG("msm: %s opened the GPU", fName);
	return true;
}

/* service_lock held */
struct msm_bo *
SapphireGPUClient::bo(uint32_t handle)
{
	if (handle == 0 || handle > MSM_BUFFERS || fBo[handle - 1].md == NULL) {
		return NULL;
	}
	return &fBo[handle - 1];
}

struct msm_bo *
SapphireGPUClient::slot(uint32_t *handle)
{
	for (uint32_t i = 0; i < MSM_BUFFERS; i++) {
		if (fBo[i].md == NULL) {
			*handle = i + 1;
			return &fBo[i];
		}
	}
	return NULL;
}

/* Give b->md [0, b->size) GPU addresses, page by page. service_lock held. */
bool
SapphireGPUClient::place(struct msm_bo *b)
{
	uint64_t n = b->size >> 12, seg;

	b->va = msm_va_alloc(n);
	if (b->va == 0) {
		LOG("msm: %s: no %llu MB of GPU addresses left", fName, b->size >> 20);
		return false;
	}
	for (uint64_t p = 0; p < n; p++) {
		uint64_t pa = b->md->getPhysicalSegment(p << 12, &seg, kIOMemoryMapperNone);
		if (pa == 0 || !map_page_4k(b->va + (p << 12), pa & ~0xFFFull)) {
			LOG("msm: %s: cannot map page %llu of %llu (pa %llx; %u + %u page tables used)", fName, p, n, pa, pt_used,
			    pt2_used);
			for (uint64_t q = 0; q < p; q++) {
				unmap_page_4k(b->va + (q << 12));
			}
			(void)tlb_flush();
			msm_mark((b->va - MSM_VA_START) >> 12, n, false);
			return false;
		}
	}
	if ((gpu_tlb & 1) && !tlb_stale) {
		gtlb.maps_plain++;              /* 0341: only mapped: nothing stale to invalidate (see gpu_tlb) */
		return true;
	}
	gtlb.maps_flushed++;
	return tlb_flush();
}

/*
 * msm buffers of every client, by kind, for the 5 s log (a leak shows as one of them growing):
 * [0] imported IOSurfaces (their memory stays wired while imported), [1] the GPU pool (outside
 * the kernel's RAM), [2] ordinary kernel memory (wired).
 */
static SInt32  msm_census_n[3];
static SInt64  msm_census_bytes[3];

static void
msm_census(const struct msm_bo *b, int delta)
{
	unsigned k = b->surf != NULL ? 0 : b->pooled ? 1 : 2;

	OSAddAtomic(delta, &msm_census_n[k]);
	OSAddAtomic64(delta * (SInt64)b->size, &msm_census_bytes[k]);
}

extern "C" void
sapphire_gpu_census(uint32_t n[3], uint64_t bytes[3])
{
	for (unsigned k = 0; k < 3; k++) {
		n[k] = (uint32_t)msm_census_n[k];
		bytes[k] = (uint64_t)msm_census_bytes[k];
	}
}

/* Free a buffer the GPU is done with. service_lock held. */
void
SapphireGPUClient::freeBo(struct msm_bo *b)
{
	uint64_t n = b->size >> 12;
	uint64_t diag_t0 = mach_absolute_time();        /* 0336 */

	msm_census(b, -1);

	for (uint64_t p = 0; p < n; p++) {
		unmap_page_4k(b->va + (p << 12));
	}
	(void)tlb_flush();
	msm_mark((b->va - MSM_VA_START) >> 12, n, false);
	if (b->surf != NULL) {
		b->md->complete();
		b->surf->release();
	}
	if (b->user) {
		msm_user_free(b);                       /* 0380: settled if the GPU wrote it, the VM's bit given back, unwired */
		if (b->wfence != 0 && fUnsettled != 0) {
			fUnsettled--;
		}
		fUserBytes -= b->size;
	}
	if (b->pooled) {
		bits_mark(gpool_bits, (b->pa - GPOOL_PA) >> 12, n, false);
	}
	b->md->release();
	fCount--;
	fBytes -= b->size;
	bzero(b, sizeof(*b));
	gdiag_since(&gdiag.bo_free, diag_t0);
}

/*
 * 0342: b, a buffer the GPU is done with and that is still counted as its client's, is kept instead of freed, if
 * the cache is on and has room. Then it is counted out, as a buffer left is, and b is an empty handle. service_lock held.
 */
bool
SapphireGPUClient::keepBo(struct msm_bo *b)
{
	if (gpu_bocache == 0 || fKeep == NULL || fKeepWhen == NULL || !b->pooled || b->surf != NULL || !service_ready ||
	    b->size > MSM_KEEP_ONE || fKeepBytes + b->size > MSM_KEEP_BYTES) {
		return false;
	}
	for (uint32_t i = 0; i < MSM_KEEP; i++) {
		if (fKeep[i].md == NULL) {
			fKeep[i] = *b;
			fKeepWhen[i] = mach_absolute_time();
			fKeepBytes += b->size;
			fCount--;
			fBytes -= b->size;
			bzero(b, sizeof(*b));
			gkeep.kept++;
			return true;
		}
	}
	return false;
}

/* The buffers kept since before `before` are freed (~0: all of them); how many. service_lock held. */
uint32_t
SapphireGPUClient::dropKept(uint64_t before)
{
	uint32_t n = 0;

	for (uint32_t i = 0; fKeep != NULL && fKeepWhen != NULL && fKeepBytes != 0 && i < MSM_KEEP; i++) {
		if (fKeep[i].md != NULL && fKeepWhen[i] < before) {
			fKeepBytes -= fKeep[i].size;
			fCount++;                       /* (counted out when it was kept: freeBo counts it out again) */
			fBytes += fKeep[i].size;
			freeBo(&fKeep[i]);
			n++;
		}
	}
	return n;
}

/* Buffers given back earlier whose submissions are done now: freed. service_lock held. */
void
SapphireGPUClient::reap(void)
{
	for (uint32_t i = 0; fLeft != NULL && i < MSM_LEFT; i++) {
		if (fLeft[i].md != NULL && msm_fence_done(fLeft[i].fence)) {
			fCount++;                       /* (counted out when it was left: release counts it again) */
			fBytes += fLeft[i].size;
			if (keepBo(&fLeft[i])) {
				continue;               /* 0342 */
			}
			freeBo(&fLeft[i]);
		}
	}
	if (fKeepBytes != 0) {
		/* 0342: what has been kept for longer than MSM_KEEP_MS goes back to the pool */
		uint64_t age, now = mach_absolute_time();
		nanoseconds_to_absolutetime((uint64_t)MSM_KEEP_MS * NSEC_PER_MSEC, &age);
		if (now > age) {
			gkeep.aged += dropKept(now - age);
		}
	}
}

/*
 * GEM_CLOSE: the handle is free at once; the memory and the GPU addresses stay until the last
 * submission that named the buffer is done. service_lock held.
 */
void
SapphireGPUClient::drop(struct msm_bo *b)
{
	gdiag.close_n++;                                /* 0336 */
	if (b->fence == 0 || msm_fence_done(b->fence) || !service_ready) {
		if (keepBo(b)) {
			return;                         /* 0342: kept for the next GEM_NEW of its size */
		}
		freeBo(b);
		return;
	}
	gdiag.close_left++;                             /* given back while the GPU still has it */
	for (uint32_t i = 0; i < MSM_LEFT; i++) {
		if (fLeft[i].md == NULL) {
			fLeft[i] = *b;
			fCount--;
			fBytes -= b->size;
			bzero(b, sizeof(*b));
			return;
		}
	}
	IOLockUnlock(service_lock);                 /* no room to leave it: wait for it */
	if (!msm_wait(b->fence, 2000)) {
		msm_hung(fName, b->fence, false);
	}
	IOLockLock(service_lock);
	freeBo(b);
}

IOReturn
SapphireGPUClient::clientClose(void)
{
	if (service_ready && !msm_wait(msm_seq, 2000)) {       /* nothing of ours may still be at work */
		msm_hung(fName, msm_seq, false);
	}
	IOLockLock(service_lock);
	(void)dropKept(~0ull);                          /* 0342 */
	if (fKeep != NULL) {
		IODelete(fKeep, struct msm_bo, MSM_KEEP);
		fKeep = NULL;
	}
	if (fKeepWhen != NULL) {
		IODelete(fKeepWhen, uint64_t, MSM_KEEP);
		fKeepWhen = NULL;
	}
	for (uint32_t i = 0; fBo != NULL && i < MSM_BUFFERS; i++) {
		if (fBo[i].md != NULL) {
			freeBo(&fBo[i]);
		}
	}
	for (uint32_t i = 0; fLeft != NULL && i < MSM_LEFT; i++) {
		if (fLeft[i].md != NULL) {
			fCount++;
			fBytes += fLeft[i].size;
			freeBo(&fLeft[i]);
		}
	}
	IOLockUnlock(service_lock);
	LOG("msm: %s closed (%u submissions)", fName, fSubmits);
	if (fBo != NULL) {
		IODelete(fBo, struct msm_bo, MSM_BUFFERS);
		fBo = NULL;
	}
	if (fLeft != NULL) {
		IODelete(fLeft, struct msm_bo, MSM_LEFT);
		fLeft = NULL;
	}
	terminate();
	return kIOReturnSuccess;
}

IOReturn
SapphireGPUClient::clientMemoryForType(UInt32 type, IOOptionBits *options, IOMemoryDescriptor **memory)
{
	IOReturn r = kIOReturnBadArgument;

	IOLockLock(service_lock);
	struct msm_bo *b = fBo != NULL ? bo(type) : NULL;
	if (b != NULL && b->user) {
		r = kIOReturnUnsupported;               /* 0380: the program's own memory, mapped where it is already */
	} else if (b != NULL && b->surf != NULL && !b->mapped && gpu_flush_mode == 2) {
		/* 0339: a surface's pixels are the program's through IOSurface and its locks alone (an unmapping, which
		 * also asks here, is of one mapped before and goes through) */
		gflush.maps_refused++;
		r = kIOReturnUnsupported;
	} else if (b != NULL) {
		b->md->retain();
		b->mapped = true;
		if (b->surf != NULL) {
			OSIncrementAtomic((volatile SInt32 *)&b->surf->cpuEpoch);      /* 0339: mapped, or unmapped after use */
		}
		*memory = b->md;
		/* pool memory write-combined; kernel memory never (see above). libdrm asks for the same. 0350: a surface of the
		 * framebuffer's scanout pool too (mapped by default, memory the kernel does not manage would be Device memory,
		 * and the program's other mapping of it is write-combined: SweetSurface.cpp) */
		*options = b->pooled || (b->surf != NULL && b->surf->lowPool) ? kIOMapWriteCombineCache : kIOMapDefaultCache;
		r = kIOReturnSuccess;
	}
	IOLockUnlock(service_lock);
	return r;
}

IOReturn
SapphireGPUClient::gemNew(struct msm_gem_new *r)
{
	uint64_t size = (r->size + 4095) & ~4095ull;
	uint32_t h;

	if (size == 0 || size > MSM_BUFFER_MAX) {
		return kIOReturnBadArgument;
	}
	{
		/* 0336: how many buffers are made, of what size, and what zeroing and mapping each costs */
		unsigned k = size < 256 * 1024 ? 0 : size < (4ull << 20) ? 1 : 2;
		OSAddAtomic64(1, (volatile SInt64 *)&gdiag.new_n[k]);
		OSAddAtomic64((SInt64)size, (volatile SInt64 *)&gdiag.new_bytes[k]);
	}
	uint64_t diag_t0;
	if (gpool_enabled()) {
		/* 0342: one kept of this size, if there is one, in place of a new one */
		struct msm_bo kept;
		bool reuse = false;
		bzero(&kept, sizeof(kept));
		IOLockLock(service_lock);
		for (uint32_t i = 0; gpu_bocache != 0 && fKeep != NULL && fKeepWhen != NULL && fKeepBytes != 0 && !reuse && i < MSM_KEEP; i++) {
			if (fKeep[i].md != NULL && fKeep[i].size == size) {
				kept = fKeep[i];
				bzero(&fKeep[i], sizeof(fKeep[i]));
				fKeepBytes -= size;
				reuse = true;
			}
		}
		uint64_t pa = reuse ? kept.pa : gpool_alloc(size >> 12);
		if (pa == 0 && fKeepBytes != 0) {
			gkeep.flushed += dropKept(~0ull);       /* no room: what is kept goes back to the pool first */
			pa = gpool_alloc(size >> 12);
		}
		IOLockUnlock(service_lock);
		if (pa == 0) {
			LOG("msm: %s: the GPU pool has no %llu KB in one piece (%u buffers, %llu MB)", fName, size >> 10, fCount,
			    fBytes >> 20);
			return kIOReturnNoMemory;
		}
		IOMemoryDescriptor *pm = reuse ? kept.md : IOMemoryDescriptor::withPhysicalAddress(pa, size, kIODirectionInOut);
		/*
		 * 0205: zeroed only when small. CoreAnimation makes and drops screen-sized textures many
		 * times a second while it animates (render targets it then draws all of); mapping 11 MB into
		 * the kernel and zeroing it write-combined made every such GEM_NEW cost 0.5 ms and more on
		 * Mesa's driver thread. Small buffers (state, queries) are still zeroed; boot-arg gpu_zero=1:
		 * every buffer, as before.
		 * 0215: gpu_zero=1 is the default again: with 0205 folders opening in the dark appearance
		 * showed white frames (and once the whole screen went yellow), which gpu_zero=1 took away.
		 */
		static int zero_all = -1;
		if (zero_all < 0) {
			int z = 1;              /* 0215: on by default again (gpu_zero=0: 0205's way); see below */
			PE_parse_boot_argn("gpu_zero", &z, sizeof(z));
			zero_all = z != 0;
		}
		bool zeroed = pm != NULL;
		/*
		 * 0395: the kernel maps every new buffer write-combined once, zeroed or not. A buffer's pages take their
		 * cache mode from the first mapping; one that the kernel never mapped reached the program as device
		 * memory, where an unaligned store traps (EXC_ARM_DA_ALIGN in the Metal driver with gpu_zero=0).
		 */
		if (pm != NULL && !(reuse && gpu_bocache == 2)) {
			diag_t0 = mach_absolute_time();
			IOMemoryMap *km = pm->map(kIOMapWriteCombineCache);
			if (km != NULL) {
				if (zero_all || size < 256 * 1024) {
					bzero((void *)km->getVirtualAddress(), size);
				}
				km->release();
			} else {
				zeroed = false;
			}
			gdiag_since(&gdiag.new_zero, diag_t0);
		}
		IOLockLock(service_lock);
		struct msm_bo *b = zeroed ? slot(&h) : NULL;
		if (b != NULL && reuse) {
			/* 0342: as it was kept, GPU addresses and all, under a new handle; mapped by nobody now */
			*b = kept;
			b->flags = r->flags;
			b->mapped = false;
			b->fence = 0;
			fCount++;
			fBytes += size;
			r->handle = h;
			gkeep.reused++;
			gkeep.reused_bytes += size;
		} else if (reuse) {
			/* no handle left, or it could not be zeroed: freed as any buffer of the client's is */
			fCount++;
			fBytes += size;
			freeBo(&kept);
			pm = NULL;
		} else if (b != NULL) {
			b->md = pm;
			b->kva = NULL;                  /* never cleaned: write-combined everywhere */
			b->size = size;
			b->flags = r->flags;
			b->pooled = true;
			b->pa = pa;
			diag_t0 = mach_absolute_time();
			bool placed = place(b);
			gdiag_since(&gdiag.new_place, diag_t0);
			if (!placed) {
				bzero(b, sizeof(*b));
				b = NULL;
			}
		}
		if (reuse) {
			/* (counted above) */
		} else if (b != NULL) {
			fCount++;
			fBytes += size;
			msm_census(b, 1);
			r->handle = h;
		} else {
			bits_mark(gpool_bits, (pa - GPOOL_PA) >> 12, size >> 12, false);
		}
		IOLockUnlock(service_lock);
		if (b == NULL) {
			OSSafeReleaseNULL(pm);
			return kIOReturnNoMemory;
		}
		return kIOReturnSuccess;
	}
	IOBufferMemoryDescriptor *m = IOBufferMemoryDescriptor::inTaskWithOptions(kernel_task,
	    kIODirectionInOut | kIOMemoryKernelUserShared, size, 4096);
	if (m == NULL) {
		LOG("msm: %s: no memory for %llu KB (%u buffers, %llu MB)", fName, size >> 10, fCount, fBytes >> 20);
		return kIOReturnNoMemory;
	}
	diag_t0 = mach_absolute_time();
	bzero(m->getBytesNoCopy(), size);
	sweet_dcache_flush((vm_offset_t)m->getBytesNoCopy(), (unsigned)size);
	gdiag_since(&gdiag.new_zero, diag_t0);
	IOLockLock(service_lock);
	struct msm_bo *b = slot(&h);
	if (b != NULL) {
		b->md = m;
		b->kva = (uint8_t *)m->getBytesNoCopy();
		b->size = size;
		b->flags = r->flags;
		diag_t0 = mach_absolute_time();
		bool placed = place(b);
		gdiag_since(&gdiag.new_place, diag_t0);
		if (!placed) {
			bzero(b, sizeof(*b));
			b = NULL;
		}
	}
	if (b != NULL) {
		fCount++;
		fBytes += size;
		msm_census(b, 1);
		r->handle = h;
	}
	IOLockUnlock(service_lock);
	if (b == NULL) {
		m->release();
		return kIOReturnNoMemory;
	}
	return kIOReturnSuccess;
}

IOReturn
SapphireGPUClient::gemInfo(struct msm_gem_info *r)
{
	IOReturn ret = kIOReturnSuccess;

	IOLockLock(service_lock);
	struct msm_bo *b = bo(r->handle);
	if (b == NULL) {
		ret = kIOReturnBadArgument;
	} else if (r->info == MSM_INFO_GET_OFFSET) {
		r->value = (uint64_t)r->handle << 32;   /* nobody maps by it here */
	} else if (r->info == MSM_INFO_GET_IOVA) {
		r->value = b->va;
	} else if (r->info == MSM_INFO_GET_FLAGS) {
		r->value = b->flags;
	} else if (r->info != MSM_INFO_SET_NAME) {
		ret = kIOReturnUnsupported;             /* SET_IOVA among them: addresses are the kernel's */
	}
	IOLockUnlock(service_lock);
	return ret;
}

/* An IOSurface's memory as a buffer (HackiOS request 0x7e). */
IOReturn
SapphireGPUClient::surface(struct msm_surface *r)
{
	SweetSurface *s = sweet_surface_copy(r->surface);
	uint32_t h;

	if (s == NULL || s->memory == NULL) {
		OSSafeReleaseNULL(s);
		return kIOReturnNotFound;
	}
	if (s->wrapped) {
		s->release();                           /* a program's own memory: rows the GPU seldom takes */
		return kIOReturnUnsupported;
	}
	uint64_t size = (s->memory->getLength() + 4095) & ~4095ull;
	r->size = size;
	r->offset = s->offset;
	r->rowBytes = s->bytesPerRow;
	r->width = s->width;
	r->height = s->height;
	r->pixelFormat = s->pixelFormat;
	r->planes = s->planeCount;
	r->ubwc = s->ubwc ? 1 : 0;
	r->metaOffset = s->ubwcMetaOffset;
	r->metaStride = s->ubwcMetaStride;
	r->metaSize = s->ubwcMetaSize;
	r->dataOffset = s->ubwcDataOffset;
	r->dataStride = s->ubwcDataStride;
	if (r->handle == MSM_DESCRIBE) {
		r->handle = 0;
		s->release();
		return kIOReturnSuccess;
	}
	IOLockLock(service_lock);
	for (uint32_t i = 0; i < MSM_BUFFERS; i++) {
		if (fBo[i].md != NULL && fBo[i].surf == s) {
			r->handle = i + 1;              /* the same surface is the same buffer, as Linux's handles */
			gdiag.surf_again++;             /* 0336 */
			IOLockUnlock(service_lock);
			s->release();
			return kIOReturnSuccess;
		}
	}
	struct msm_bo *b = slot(&h);
	IOReturn ret = kIOReturnNoMemory;
	if (b != NULL && s->memory->prepare() == kIOReturnSuccess) {
		b->md = s->memory;
		b->md->retain();
		b->surf = s;
		b->size = size;
		b->kva = (uint8_t *)s->kernelAddress();
		if (place(b)) {
			gdiag.surf_new++;               /* 0336 */
			fCount++;
			fBytes += size;
			msm_census(b, 1);
			r->handle = h;
			s = NULL;                       /* the buffer keeps the reference */
			ret = kIOReturnSuccess;
		} else {
			b->md->complete();
			b->md->release();
			bzero(b, sizeof(*b));
		}
	}
	IOLockUnlock(service_lock);
	OSSafeReleaseNULL(s);
	return ret;
}

/*
 * 0380: the program's memory [address, address + size) as a buffer (see msm_user_sync). The range on whole pages, at
 * most MSM_BUFFER_MAX, flags 0; this client holds at most MSM_USER_BYTES of it at a time. kIOReturnVMError: memory the
 * program does not have (or that cannot be held), or a page the pmap does not manage.
 */
IOReturn
SapphireGPUClient::userMemory(struct msm_user_memory *r)
{
	const uint64_t size = r->size, n = size >> PAGE_SHIFT, t0 = mach_absolute_time();
	IOMemoryDescriptor *md;
	ppnum_t *pages;
	uint32_t h;
	bool ok = true;

	if (gpu_usermem == 0) {
		return kIOReturnUnsupported;
	}
	if (r->address == 0 || (r->address & PAGE_MASK) != 0 || (r->address & 4095) != 0 || size == 0 || (size & PAGE_MASK) != 0 ||
	    (size & 4095) != 0 || size > MSM_BUFFER_MAX || r->flags != 0 || r->address + size < r->address) {
		guser.refused++;
		return kIOReturnBadArgument;
	}
	md = IOMemoryDescriptor::withAddressRange(r->address, size, kIODirectionInOut, fTask);
	pages = md != NULL ? IONew(ppnum_t, n) : NULL;
	if (md == NULL || pages == NULL) {
		OSSafeReleaseNULL(md);
		guser.refused++;
		return kIOReturnNoMemory;
	}
	if (md->prepare(kIODirectionInOut) != kIOReturnSuccess) {
		md->release();
		IODelete(pages, ppnum_t, n);
		guser.refused++;
		return kIOReturnVMError;
	}
	for (uint64_t p = 0; p < n && ok; p++) {
		IOByteCount seg = 0;
		addr64_t pa = md->getPhysicalSegment(p << PAGE_SHIFT, &seg, kIOMemoryMapperNone);

		pages[p] = (ppnum_t)(pa >> PAGE_SHIFT);
		ok = pa != 0 && pmap_valid_page(pages[p]);
	}
	if (ok) {
		/* the account starts: every page's bit taken (a write from here on sets it again), then no line of the
		 * buffer left in any cache */
		for (uint64_t p = 0; p < n; p++) {
			(void)pmap_test_and_clear_modify(pages[p]);
		}
		msm_user_dsb();
		for (uint64_t p = 0; p < n; p++) {
			msm_user_lines(pages[p], true);
		}
		msm_user_dsb();
	}
	IOReturn ret = ok ? kIOReturnNoMemory : kIOReturnVMError;
	IOLockLock(service_lock);
	struct msm_bo *b = ok && fUserBytes + size <= MSM_USER_BYTES ? slot(&h) : NULL;
	if (b != NULL) {
		b->md = md;
		b->size = size;
		b->user = true;
		b->pages = pages;
		if (place(b)) {
			fCount++;
			fBytes += size;
			fUserBytes += size;
			msm_census(b, 1);
			r->handle = h;
			guser.made++;
			guser.bytes += size;
			guser.peak = MAX(guser.peak, guser.bytes);
			ret = kIOReturnSuccess;
		} else {
			bzero(b, sizeof(*b));
		}
	} else if (ok) {
		LOG("msm: %s: %llu KB of its own memory as a GPU buffer refused: it holds %llu KB already, or no handle is left", fName,
		    size >> 10, fUserBytes >> 10);
	}
	IOLockUnlock(service_lock);
	if (ret != kIOReturnSuccess) {
		md->complete();
		md->release();
		IODelete(pages, ppnum_t, n);
		guser.refused++;
	}
	gdiag_since(&guser.make, t0);
	return ret;
}

/* The client's program-memory buffers the GPU wrote: settled, those whose submission is done. With service_lock taken
 * here. */
void
SapphireGPUClient::settleUser(void)
{
	IOLockLock(service_lock);
	for (uint32_t i = 0; fUnsettled != 0 && i < MSM_BUFFERS + MSM_LEFT; i++) {
		struct msm_bo *b = i < MSM_BUFFERS ? (fBo != NULL ? &fBo[i] : NULL) : (fLeft != NULL ? &fLeft[i - MSM_BUFFERS] : NULL);
		if (b != NULL && b->md != NULL && b->user && b->wfence != 0 && msm_fence_done(b->wfence)) {
			msm_user_settle(b);
			b->wfence = 0;
			fUnsettled--;
		}
	}
	IOLockUnlock(service_lock);
}

IOReturn
SapphireGPUClient::submitIt(struct msm_gem_submit *r)
{
	struct msm_gem_submit_bo *bos = NULL;
	struct msm_gem_submit_cmd *cmds = NULL;
	IOReturn ret = kIOReturnBadArgument;
	uint64_t diag_t0 = mach_absolute_time(), diag_t1 = 0;   /* 0336: what a submission costs here, and where */

	if (r->nr_cmds == 0 || r->nr_cmds > MSM_SUBMIT_CMDS || r->nr_bos == 0 || r->nr_bos > MSM_SUBMIT_BOS) {
		LOG("msm: %s: a submission of %u commands in %u buffers: not taken", fName, r->nr_cmds, r->nr_bos);
		return kIOReturnBadArgument;
	}
	bos = IONew(struct msm_gem_submit_bo, r->nr_bos);
	cmds = IONew(struct msm_gem_submit_cmd, r->nr_cmds);
	if (bos == NULL || cmds == NULL) {
		ret = kIOReturnNoMemory;
		goto out;
	}
	if (copyin((user_addr_t)r->bos, bos, r->nr_bos * sizeof(*bos)) != 0 ||
	    copyin((user_addr_t)r->cmds, cmds, r->nr_cmds * sizeof(*cmds)) != 0) {
		ret = kIOReturnVMError;
		goto out;
	}
	diag_t1 = mach_absolute_time();
	IOLockLock(service_lock);
	gdiag_since(&gdiag.sub_lock, diag_t1);
	diag_t1 = mach_absolute_time();
	if (!service_ready) {
		IOLockUnlock(service_lock);
		ret = kIOReturnNotReady;
		goto out;
	}
	reap();
	if (!ring_room(4 * r->nr_cmds + 16 + (gpu_marks ? 8 : 0), 2000)) {
		msm_hung(fName, msm_seq, true);
		IOLockUnlock(service_lock);
		ret = kIOReturnTimeout;
		goto out;
	}
	{
		uint64_t now = mach_absolute_time();
		for (uint32_t i = 0; i < r->nr_bos; i++) {
			struct msm_bo *b = bo(bos[i].handle);
			if (b != NULL && b->user) {
				msm_user_sync(b, bos[i].flags);         /* 0380: the pages a processor wrote since */
				continue;
			}
			if (b == NULL || b->kva == NULL) {
				continue;
			}
			if (b->surf != NULL && b->surf->state != NULL) {
				uint32_t seed = *(volatile uint32_t *)((uint8_t *)b->surf->state->getBytesNoCopy() + 0xc);
				/* the GPU has drawn it since that seed: the display need not clean it (SweetFramebuffer) */
				b->surf->gpuSeed = seed;
				b->surf->gpuDrawn = true;
				if (gpu_flush_mode != 0) {
					msm_surface_sync(b, bos[i].flags);      /* 0339: by the surface's own account */
					b->seed = seed;
					continue;
				}
				/*
				 * 0165: a surface is cleaned only when the CPU may have written it since: its seed
				 * moved (IOSurfaceLock for writing) or it is mapped as a buffer. The 4 ms window that
				 * also cleaned an unchanged surface again cleaned the 11 MB display surface on every
				 * frame (glFlush took 9 ms a frame, most of the CPU's time); the GPU alone writes it.
				 */
				if ((seed == b->seed && !b->mapped) || b->surf->lowPool) {
					continue;               /* (0350: a scanout pool surface is write-combined everywhere: no cache holds it) */
				}
				b->seed = seed;
				b->cleaned = now;
			} else if (!b->mapped) {
				continue;                       /* never written by the CPU since it was cleaned */
			}
			uint64_t flush_t0 = mach_absolute_time();
			uint32_t epoch = b->surf != NULL ? b->surf->cpuEpoch : 0;      /* 0339: the account is kept in every mode */
			sweet_dcache_flush((vm_offset_t)b->kva, (unsigned)b->size);
			if (b->surf != NULL) {
				b->surf->invalEpoch = epoch;
				b->surf->cleanEpoch = epoch;
			}
			gdiag_since(&gdiag.sub_flush, flush_t0);
			gdiag.sub_flush_bytes += b->size;
		}
		if (msm_seq != 0 && !msm_fence_done(msm_seq)) {
			gdiag.sub_behind++;                     /* the submission before this one is not done yet */
		}
		uint32_t mark = ~0u;
		if (gpu_marks) {
			gdiag_harvest();
			mark = gdiag_sub_head % GDIAG_SLOTS;
			volatile uint64_t *m = (volatile uint64_t *)(pool + OFF_DIAG + mark * 16u);
			m[0] = m[1] = 0;
			gdiag_mark(POOL_PA + OFF_DIAG + mark * 16u);    /* the CP begins this submission */
		}
		uint32_t n = 0;
		for (uint32_t i = 0; i < r->nr_cmds; i++) {
			struct msm_bo *b = cmds[i].submit_idx < r->nr_bos ? bo(bos[cmds[i].submit_idx].handle) : NULL;
			if (cmds[i].type != MSM_SUBMIT_CMD_BUF && cmds[i].type != MSM_SUBMIT_CMD_CTX_RESTORE_BUF) {
				continue;                       /* IB_TARGET_BUF: only reached from another */
			}
			if (b == NULL || cmds[i].nr_relocs != 0 || (cmds[i].size & 3) || (cmds[i].submit_offset & 3) ||
			    (uint64_t)cmds[i].submit_offset + cmds[i].size > b->size) {
				LOG("msm: %s: command %u (buffer %u, +%u, %u bytes, %u relocs): not taken", fName, i,
				    cmds[i].submit_idx, cmds[i].submit_offset, cmds[i].size, cmds[i].nr_relocs);
				IOLockUnlock(service_lock);
				goto out;
			}
			uint64_t va = b->va + cmds[i].submit_offset;
			emit(pkt7(CP_INDIRECT_BUFFER, 3));
			emit((uint32_t)va);
			emit((uint32_t)(va >> 32));
			emit(cmds[i].size / 4);
			n++;
		}
		emit(pkt7(CP_WAIT_FOR_IDLE, 0));
		if (mark != ~0u) {
			gdiag_mark(POOL_PA + OFF_DIAG + mark * 16u + 8);        /* all of it drawn */
		}
		emit(pkt7(CP_EVENT_WRITE, 4));          /* CACHE_FLUSH_TS into the msm slot (event() writes the shared one) */
		emit(EV_CACHE_FLUSH_TS);                /* (not TIMESTAMP: see event()) */
		emit((uint32_t)(POOL_PA + OFF_MSM_FENCE));
		emit((uint32_t)((POOL_PA + OFF_MSM_FENCE) >> 32));
		emit(++msm_seq);
		if (mark != ~0u) {
			gdiag_sub_fence[mark] = msm_seq;
			gdiag_sub_head++;
		}
		gdiag.sub++;
		gdiag.sub_cmds += n;
		gdiag.sub_bos += r->nr_bos;
		r->fence = msm_seq;
		kick();                                 /* and back to the program: msm_wait() is how it waits */
		for (uint32_t i = 0; i < r->nr_bos; i++) {
			struct msm_bo *b = bo(bos[i].handle);
			if (b != NULL) {
				b->fence = r->fence;
				if (b->surf != NULL) {
					b->surf->gpuFence = r->fence;   /* 0350: what a scanout of the surface waits for */
					if ((bos[i].flags & MSM_SUBMIT_BO_WRITE) != 0 || (bos[i].flags & MSM_SUBMIT_BO_READ) == 0) {
						b->surf->gpuWriteFence = r->fence;      /* 0394: what an IOSurfaceLock settles */
					}
				}
				if (b->user && ((bos[i].flags & MSM_SUBMIT_BO_WRITE) != 0 || (bos[i].flags & MSM_SUBMIT_BO_READ) == 0)) {
					if (b->wfence == 0) {
						fUnsettled++;
					}
					b->wfence = r->fence;   /* 0380: its caches settled once this is done */
				}
			}
		}
		if (r->flags & MSM_SUBMIT_FENCE_FD_OUT) {
			r->fence_fd = -1;
		}
		if (fSubmits++ < 4) {
			LOG("msm: %s: submission %u, %u command buffers in %u buffers (asynchronous)", fName, r->fence, n,
			    r->nr_bos);
		}
		ret = kIOReturnSuccess;
		gdiag_since(&gdiag.sub_hold, diag_t1);
	}
	IOLockUnlock(service_lock);
out:
	gdiag_since(&gdiag.sub_all, diag_t0);
	if (bos != NULL) {
		IODelete(bos, struct msm_gem_submit_bo, r->nr_bos);
	}
	if (cmds != NULL) {
		IODelete(cmds, struct msm_gem_submit_cmd, r->nr_cmds);
	}
	return ret;
}

/* msm GET_PARAM for this GPU (Linux adreno_get_param; numbers from msm_drm.h) */
static bool
msm_param(uint32_t which, uint64_t *value)
{
	switch (which) {
	case 0x01: *value = 610; return true;                   /* GPU_ID */
	case 0x02: *value = 128 * 1024; return true;            /* GMEM_SIZE */
	case 0x03: *value = 0x06010001; return true;            /* CHIP_ID (a610 v2, read off the GPU) */
	case 0x04: *value = 600000000; return true;             /* MAX_FREQ (nominal) */
	case 0x05: *value = mach_absolute_time(); return true;  /* TIMESTAMP */
	case 0x06: *value = 0x100000; return true;              /* GMEM_BASE: UCHE_GMEM_RANGE_MIN of hw_init */
	case 0x07: *value = 1; return true;                     /* PRIORITIES */
	case 0x09: *value = gpu_faults; return true;           /* FAULTS: the resets of a hung GPU (0317) */
	case 0x08: case 0x0a: case 0x11: *value = 0; return true;  /* PP_PGTABLE SUSPENDS RAYTRACING */
	case 0x10: *value = ubwc_hbb(); return true;            /* HIGHEST_BANK_BIT: the one in force (0370) */
	case 0x1000: *value = gpool_enabled(); return true;     /* sapphire: buffers are write-combined (libdrm) */
	case 0x1001: *value = gpu_usermem != 0; return true;    /* sapphire 0380: request 0x7d, the program's own memory */
	default: return false;                                  /* VA_START/VA_SIZE among them */
	}
}

IOReturn
SapphireGPUClient::externalMethod(uint32_t selector, IOExternalMethodArguments *args, IOExternalMethodDispatch *,
    OSObject *, void *)
{
	union {
		struct msm_param param;
		struct msm_gem_new gemNew;
		struct msm_gem_info gemInfo;
		struct msm_gem_cpu_prep cpuPrep;
		struct msm_gem_submit submit;
		struct msm_wait_fence waitFence;
		struct msm_gem_madvise madvise;
		struct msm_submitqueue queue;
		struct msm_submitqueue_query query;
		struct drm_gem_close close;
		struct msm_surface surface;
		struct msm_user_memory userMemory;
	} r;
	size_t size = args->structureInputSize;
	IOReturn ret = kIOReturnSuccess;
	uint64_t zero = 0;
	static uint32_t unknown;

	if (args->structureInput == NULL || size > sizeof(r) || fBo == NULL) {
		return kIOReturnBadArgument;
	}
	if (!service_ready) {
		return kIOReturnNotReady;
	}
	bzero(&r, sizeof(r));
	memcpy(&r, args->structureInput, size);
	switch (selector) {
	case kMsmGetParam:
		ret = msm_param(r.param.param, &r.param.value) ? kIOReturnSuccess : kIOReturnBadArgument;
		break;
	case kMsmSetParam:
	case kMsmSubmitQueueClose:
		break;
	case kMsmGemCpuFini: {
		/* 0339: the program has finished reading or writing the buffer (its handle comes first, as in CPU_PREP) */
		IOLockLock(service_lock);
		struct msm_bo *b = bo(r.cpuPrep.handle);
		if (b != NULL && b->surf != NULL) {
			OSIncrementAtomic((volatile SInt32 *)&b->surf->cpuEpoch);
		}
		IOLockUnlock(service_lock);
		break;
	}
	case kMsmGemNew:
		ret = gemNew(&r.gemNew);
		break;
	case kMsmGemInfo:
		ret = gemInfo(&r.gemInfo);
		break;
	case kMsmGemCpuPrep: {
		/* the CPU is about to use the buffer: the GPU done with it, and (kernel memory) nothing stale cached */
		IOLockLock(service_lock);
		struct msm_bo *b = bo(r.cpuPrep.handle);
		uint32_t f = b != NULL ? b->fence : 0;
		IOLockUnlock(service_lock);
		if (b == NULL) {
			ret = kIOReturnBadArgument;
			break;
		}
		if (f != 0 && !msm_wait(f, (r.cpuPrep.op & 0x04) ? 0 : msm_ms_until(&r.cpuPrep.timeout))) {
			ret = kIOReturnBusy;            /* MSM_PREP_NOSYNC, or not done in time */
			break;
		}
		IOLockLock(service_lock);
		b = bo(r.cpuPrep.handle);
		gdiag.prep_n++;                         /* 0336 */
		if (b != NULL && b->user) {
			if (b->wfence != 0) {
				msm_user_settle(b);             /* 0380: what the GPU wrote, before the processor reads */
				b->wfence = 0;
				if (fUnsettled != 0) {
					fUnsettled--;
				}
			}
		} else if (b != NULL && b->kva != NULL) {
			uint32_t epoch = b->surf != NULL ? b->surf->cpuEpoch : 0;
			sweet_dcache_flush((vm_offset_t)b->kva, (unsigned)b->size);
			gdiag.prep_flush_bytes += b->size;
			if (b->surf != NULL) {
				/* 0339: nothing of it in a cache now, and the program is about to touch it */
				b->surf->invalEpoch = epoch;
				b->surf->cleanEpoch = epoch;
				OSIncrementAtomic((volatile SInt32 *)&b->surf->cpuEpoch);
			}
		}
		IOLockUnlock(service_lock);
		break;
	}
	case kMsmGemSubmit:
		ret = submitIt(&r.submit);
		break;
	case kMsmWaitFence: {
		uint64_t ms = msm_ms_until(&r.waitFence.timeout);
		if (!msm_wait(r.waitFence.fence, ms)) {
			ret = kIOReturnTimeout;
			if (ms >= 2000) {
				msm_hung(fName, r.waitFence.fence, false);
			}
		} else if (fUnsettled != 0) {
			settleUser();                           /* 0380: the program's buffers the GPU wrote, now done */
		}
		break;
	}
	case kMsmGemMadvise:
		r.madvise.retained = 1;
		break;
	case kMsmSubmitQueueNew:
		r.queue.id = ++fQueue;
		break;
	case kMsmSubmitQueueQuery:
		if (r.query.data != 0 && r.query.len >= sizeof(zero)) {
			(void)copyout(&zero, (user_addr_t)r.query.data, sizeof(zero));
		}
		break;
	case kDrmGemClose: {
		IOLockLock(service_lock);
		struct msm_bo *b = bo(r.close.handle);
		if (b == NULL) {
			ret = kIOReturnBadArgument;
		} else {
			drop(b);
		}
		IOLockUnlock(service_lock);
		break;
	}
	case kMsmSurface:
		ret = surface(&r.surface);
		break;
	case kMsmUserMemory:
		ret = userMemory(&r.userMemory);
		break;
	default:
		if (unknown++ < 16) {
			LOG("msm: %s: request %#x (%zu bytes) is not known here", fName, selector, size);
		}
		ret = kIOReturnUnsupported;
		break;
	}
	if (ret == kIOReturnSuccess && args->structureOutput != NULL) {
		if (args->structureOutputSize > size) {
			args->structureOutputSize = (uint32_t)size;
		}
		memcpy(args->structureOutput, &r, args->structureOutputSize);
	}
	return ret;
}

/* Called once gpu_service is up: publish the service Mesa's libdrm looks for. */
static void
msm_publish(void)
{
	IOService *provider = IOService::getPlatform();
	SapphireGPU *gpu = OSTypeAlloc(SapphireGPU);

	if (gpu == NULL || provider == NULL || !gpu->init()) {
		OSSafeReleaseNULL(gpu);
		LOG("msm: cannot make the SapphireGPU service");
		return;
	}
	gpu->setProperty(kIOUserClientClassKey, "SapphireGPUClient");
	gpu->setName("SapphireGPU");
	(void)PE_parse_boot_argn("gpu_flush", &gpu_flush_mode, sizeof(gpu_flush_mode));     /* 0339 */
	if (gpu->attach(provider)) {
		if (gpu->start(provider)) {
			gpu->registerService();
			LOG("*** msm: SapphireGPU published: Mesa can open the GPU ***");
		} else {
			gpu->detach(provider);
		}
	}
	gpu->release();
}

void
sapphire_gpu_init(void)
{
	uint32_t stage = 0;
	thread_t thread;

	if (!PE_parse_boot_argn("gpu", &stage, sizeof(stage)) || stage == 0) {
		return;
	}
	(void)PE_parse_boot_argn("gpu_tlb", &gpu_tlb, sizeof(gpu_tlb));         /* 0341 */
	if (kernel_thread_start(gpu_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}

/*
 * sapphire 0336: kern.sweet_gpu_diag, the counters of gdiag (near the top of this file). Read only: two readings
 * subtracted give what happened in between. Times are "n total max" in nanoseconds; the GPU's own times are in ticks
 * of 19.2 MHz (19200 a millisecond) and only move with kern.sweet_gpu_marks=1.
 */
#include <sys/sysctl.h>
#include <libkern/libkern.h>

#define GDIAG_CAT(...) do { \
	if (off + 1 < sizeof(buf)) { \
		int n_ = snprintf(buf + off, sizeof(buf) - off, __VA_ARGS__); \
		if (n_ > 0) { \
			off = off + (size_t)n_ < sizeof(buf) ? off + (size_t)n_ : sizeof(buf) - 1; \
		} \
	} \
} while (0)
#define GDIAG_T(t)  (t).n, (t).ns, (t).max

static int
sweet_gpu_diag_sysctl SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	char buf[2560];
	size_t off = 0;

	if (gpu_marks && service_lock != NULL && service_ready) {
		IOLockLock(service_lock);
		gdiag_harvest();
		IOLockUnlock(service_lock);
	}
	buf[0] = 0;
	GDIAG_CAT("submit: %llu, %llu command buffers, %llu buffers named; the one before not done %llu; ring max %llu dwords, no room %llu\n",
	    gdiag.sub, gdiag.sub_cmds, gdiag.sub_bos, gdiag.sub_behind, gdiag.sub_ring_max, gdiag.ring_waits);
	GDIAG_CAT("submit ns: whole call %llu %llu %llu; waiting for the lock %llu %llu %llu; holding it %llu %llu %llu; cache flush %llu %llu %llu, %llu bytes\n",
	    GDIAG_T(gdiag.sub_all), GDIAG_T(gdiag.sub_lock), GDIAG_T(gdiag.sub_hold), GDIAG_T(gdiag.sub_flush), gdiag.sub_flush_bytes);
	GDIAG_CAT("new: under 256K %llu (%llu bytes), under 4M %llu (%llu), larger %llu (%llu); zeroing ns %llu %llu %llu; mapping ns %llu %llu %llu\n",
	    gdiag.new_n[0], gdiag.new_bytes[0], gdiag.new_n[1], gdiag.new_bytes[1], gdiag.new_n[2], gdiag.new_bytes[2],
	    GDIAG_T(gdiag.new_zero), GDIAG_T(gdiag.new_place));
	GDIAG_CAT("close: %llu, still the GPU's %llu; freeing ns %llu %llu %llu; surfaces: new %llu, again %llu; tlb flush ns %llu %llu %llu, slept %llu (poll sleeps %llu)\n",
	    gdiag.close_n, gdiag.close_left, GDIAG_T(gdiag.bo_free), gdiag.surf_new, gdiag.surf_again, GDIAG_T(gdiag.tlb),
	    gdiag.tlb_slept, gdiag.poll_sleeps);
	GDIAG_CAT("wait: %llu, done already %llu; slept ns %llu %llu %llu in %llu looks; cpu prep %llu, %llu bytes flushed\n",
	    gdiag.wait_n, gdiag.wait_done, GDIAG_T(gdiag.wait), gdiag.wait_loops, gdiag.prep_n, gdiag.prep_flush_bytes);
	GDIAG_CAT("copy: %llu, the copy before still queued %llu, drawing still ahead %llu; waiting for the lock ns %llu %llu %llu\n",
	    gdiag.copy_n, gdiag.copy_behind, gdiag.copy_gpu_busy, GDIAG_T(gdiag.copy_lock));
	GDIAG_CAT("gpu time (marks %d), submissions: %llu, %llu ticks, max %llu; ms <1 <2 <4 <8 <12 <16 <24 <32 <48 more:", gpu_marks,
	    gdiag.gt_sub_n, gdiag.gt_sub_ticks, gdiag.gt_sub_max);
	for (unsigned k = 0; k < GDIAG_HIST; k++) {
		GDIAG_CAT(" %llu", gdiag.gt_sub_hist[k]);
	}
	GDIAG_CAT("\ngpu time, panel copies: %llu, %llu ticks, max %llu; ms <0.5 <1 <1.5 <2 <3 <4 <6 <8 <12 more:", gdiag.gt_copy_n,
	    gdiag.gt_copy_ticks, gdiag.gt_copy_max);
	for (unsigned k = 0; k < GDIAG_HIST; k++) {
		GDIAG_CAT(" %llu", gdiag.gt_copy_hist[k]);
	}
	GDIAG_CAT("; marks lost %llu\n", gdiag.gt_lost);
	return SYSCTL_OUT(req, buf, off + 1);
}
SYSCTL_PROC(_kern, OID_AUTO, sweet_gpu_diag, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED,
    NULL, 0, sweet_gpu_diag_sysctl, "A", "sapphire 0336: what submissions, buffers, waits and panel copies cost in the GPU driver");
SYSCTL_INT(_kern, OID_AUTO, sweet_gpu_marks, CTLFLAG_RW | CTLFLAG_LOCKED, &gpu_marks, 0,
    "sapphire 0336: 1 = time every msm submission and panel copy on the GPU (two CP_REG_TO_MEM of the always-on counter each)");

#else  /* !ARM64_BOARD_CONFIG_SWEET */

extern "C" void sapphire_gpu_init(void) {}

#endif /* ARM64_BOARD_CONFIG_SWEET */
