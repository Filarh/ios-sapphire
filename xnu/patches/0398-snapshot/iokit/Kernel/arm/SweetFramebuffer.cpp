/*
 * Built-in IOMobileFramebuffer for SWEET: the ABL's continuous-splash
 * framebuffer (1080x2400 a8r8g8b8 in RAM, the panel scans it out by
 * itself) as the main display, driven by CoreAnimation's CPU renderer.
 *
 * IOMobileFramebuffer.framework (iOS 16.7) counts /arm-io children with
 * device_type "display-subsystem", waits for that many services matching
 * IOServiceMatching("IOMobileFramebuffer"), opens each with type 0 and
 * reads canvas-width/canvas-height from the service's provider. The class
 * here (AppleCLCD, an IOMobileFramebuffer) matches the /arm-io/disp0 nub.
 *
 * User client selectors used by CoreAnimation:
 *   3  GetLayerDefaultSurface [layer, ?] -> [IOSurface port name]
 *   4  SwapBegin -> [swap id]
 *   5  SwapEnd, struct 0x46c: +0x50 swap id, +0x54 IOSurface id per layer
 *      (4 layers), +0x64 rect a / +0xc4 rect b per layer (int x,y,w,h),
 *      +0x104 enabled layer mask
 *   6  SwapWait, 7 GetID, 8 GetDisplaySize -> [w, h],
 *   12 RequestPowerChange [state]: 0 = display off (backboardd, power
 *      button / auto-lock), else on. Off fades the panel out over a few
 *      vsyncs (halving each pixel) and keeps it black until on again;
 *      the panel itself stays powered (AMOLED black pixels are off).
 *   18 IsMainDisplay,
 *   28 GetDotPitch, 72 Enable/DisableNotifications [func, refcon, type, 0]
 * Everything else is accepted and logged once.
 *
 * SwapEnd copies the enabled layers' surfaces into the panel memory (no
 * scaling, CPU copy). Vsync (notification type 5) is a 60 Hz timer that
 * sends (count, mach_absolute_time, 0) as 32-bit halves to clients that
 * enabled it, through the port they registered (IOConnectSetNotificationPort).
 *
 * The panel memory itself is published as an IOSurface and returned as the
 * default framebuffer surface. The kernel text console keeps drawing over
 * the GUI by default: boot-arg sweetfb_noconsole=1 stops drawing it from
 * the first swap on (it still reaches the persistent log; panics draw).
 */
#include "SweetSurface.h"

extern "C" int sweet_console_muted;     /* osfmk/console/serial_console.c */
extern "C" int sweet_splash_stop(void); /* osfmk/console/video_console.c (0299): the boot screen */
extern "C" void sweet_console_retarget(vm_offset_t va);   /* osfmk/console/video_console.c */
extern "C" void (*sweet_console_show_hook)(void);
extern "C" void topaz_modem_init(void);
extern "C" void sweet_amfi_service_init(void);     /* iokit/Kernel/arm/SweetAMFIService.cpp */
extern "C" void sapphire_gpu_census(uint32_t n[3], uint64_t bytes[3]);   /* SapphireGpu.cpp */
extern "C" unsigned int vm_page_wire_count;        /* osfmk/vm (the same kernel) */
extern "C" void sapphire_gpu_init(void);            /* iokit/Kernel/arm/SapphireGpu.cpp (boot-arg gpu=N) */
extern "C" void sweet_busy_init(void);              /* iokit/Kernel/arm/SweetBusy.cpp (boot-arg sweetbusy=N) */             /* iokit/Kernel/arm/TopazModem.cpp (boot-arg modem=1) */
extern "C" int sweet_console_copy_dirty(void *dst);
extern bool sapphire_gpu_copy_pending(void);         /* SapphireGpu.cpp (0212): the last copy is still queued */

/*
 * topaz: a surface the MDP may still be fetching counts as in use, so
 * CoreAnimation does not render into it. IOSurfaceClientIsInUse (iOS 16.7)
 * reads the u64 at +0x18 of the shared state page; held from SwapEnd until
 * one vsync after the next frame replaced it on the panel.
 */
static void
sweet_hold(SweetSurface *s, bool on)
{
	OSAddAtomic64(on ? 1 : -1, (volatile SInt64 *)((uint8_t *)s->state->getBytesNoCopy() + 0x18));
}

static void
sweet_hold_set(SweetSurface **set, bool on)
{
	for (uint32_t i = 0; i < 4; i++) {
		if (set[i] != NULL && !on) {
			sweet_hold(set[i], false);
			set[i]->release();
			set[i] = NULL;
		}
	}
}
class AppleCLCD;
static AppleCLCD *sweet_cons_fb;
static AppleCLCD *sweet_dsi_fb;                 /* 0260: sweet_fb_dsi_seq() */
static AppleCLCD *sweet_frame_fb;               /* 0286: kern.sweet_fb_frame_await/_sent */
/* 0324: the panel's state for tap to wake (bsd/dev/sweet_touch.c): 1 after RequestPowerChange(0), and since when */
extern "C" volatile int sweet_fb_panel_off;
extern "C" volatile uint64_t sweet_fb_panel_off_abs;
volatile int sweet_fb_panel_off;
volatile uint64_t sweet_fb_panel_off_abs;
static void sweet_cons_show(void);
class OSDictionary;
static void sweet_bl_log(const char *who, OSDictionary *d);
extern "C" void sweet_dcache_clean(vm_offset_t addr, unsigned length);  /* osfmk/arm/caches.c */
extern "C" vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);

#if defined(SWEET)
#include <IOKit/IOService.h>
#include <IOKit/IOUserClient.h>
#include <libkern/c++/OSCollectionIterator.h>
#include <kern/thread_call.h>
#include <IOKit/IOKitKeys.h>
#include <IOKit/IOLib.h>
#include <kern/clock.h>
#include <IOKit/IOLocks.h>
#include <IOKit/IOWorkLoop.h>
#include <IOKit/IOTimerEventSource.h>
#include <IOKit/system.h>
#include <pexpert/pexpert.h>
#include <kern/clock.h>
#include <sys/proc.h>

enum {
	kSweetFBGetLayerDefaultSurface  = 3,
	kSweetFBSwapBegin               = 4,
	kSweetFBSwapEnd                 = 5,
	kSweetFBSwapWait                = 6,
	kSweetFBGetID                   = 7,
	kSweetFBGetDisplaySize          = 8,
	kSweetFBRequestPowerChange      = 12,
	kSweetFBIsMainDisplay           = 18,
	kSweetFBGetDotPitch             = 28,
	kSweetFBNotifications           = 72,
	kSweetFBGetBlock                = 79,
};

#define SWEET_FB_SWAP_REC_SIZE          0x46c
#define SWEET_FB_LAYERS                 4
#define SWEET_FB_FADE_STEPS             30      /* ~0.5 s at 60 Hz */
#define SWEET_FB_MAX_NITS               500     /* IOMFBBrightnessLevel at panel DBV 0x7ff */
#define SWEET_FB_NOTIFY_TYPES           8
#define SWEET_FB_NOTIFY_VSYNC           5
#define SWEET_FB_VSYNC_US               16667
#define SWEET_FB_DOT_PITCH              458     /* ppi, as the iPhone X panel */
/*
 * The iOS userland only has status bar / layout data for real iPhone
 * screens (UIKit: "No status bar visual provider for screen", the red
 * rdar://45025538 bar for 360x800 points). The display is therefore
 * presented as the iPhone X panel, 1125x2436 (375x812 points @3x), and the
 * centered 1080x2400 part of it is shown on the 1080x2400 panel.
 * Boot-arg sweetfb_native=1 presents the real panel size instead.
 */
#define SWEET_FB_LOGICAL_W              1125
#define SWEET_FB_LOGICAL_H              2436
#define SWEET_FB_FORMAT_BGRA            0x42475241

class IOMobileFramebuffer : public IOService {
	OSDeclareDefaultStructors(IOMobileFramebuffer);
};

OSDefineMetaClassAndStructors(IOMobileFramebuffer, IOService);

class IOMobileFramebufferUserClient;

/*
 * sapphire 0233: what the MDP scans (flip buffers, console shadow) and the DSI command buffer,
 * from a fixed piece of the middle DDR bank the kernel does not manage. The MDP only reaches
 * below 4 GiB, and with the kernel's RAM above 4 GiB (mkadt.py --highmem) a contiguous allocation
 * below 4 GiB cannot succeed: vm_page_find_contiguous failed, ran zone_gc twice and failed again
 * for every buffer, minutes in which the panel stayed black after the Apple logo. Next to the
 * modem's pool (0xE0000000, 16 MB) and the GPU's (0xE1000000, 5 MB), below 0xF0000000. Mapped
 * write-combined like the modem's pool: the MDP sees every write, the cache cleans become no-ops.
 */
#define SWEET_FB_LOW_PA         0xE2000000ull
/*
 * sapphire 0304: the flip buffers come from the low pool mapped write-combined (Normal non-cacheable,
 * SweetLowBuf::alloc): no CPU cache holds them, the MDP reads what the GPU wrote. The invalidate of the
 * whole back buffer after each scaled GPU copy (0239, from when the flip buffers were cacheable, 0147)
 * cost ~162k dc civac, 10.4 MB, on every frame for nothing. kern.sweet_fb_flipflush=1 puts it back.
 */
static int sweet_fb_flipflush = 0;
/*
 * sapphire 0338 (an experiment, off by default): kern.sweet_fb_hold_ms=N, boot-arg sweetfb_hold=N. A SwapEnd that
 * finds the GPU's copy of the frame before still queued waits for it, up to N ms (200 at most), before it presents its
 * own frame. With 0, as before, the new copy is queued over the same back buffer and the flip then waits for the new
 * one: the frame before is drawn, copied and never shown (present(): flipWhenCopied(false), then fFlipPhys[fFlipIdx ^ 1]
 * again; 0336 counts it as "overwritten"). The wait is what a framebuffer with a full swap queue does to its caller
 * (HackiOS's MiniIOMobileFramebuffer holds SwapEnd the same way, fb_queue): the window server stays at most one frame
 * ahead of the panel, and every frame the GPU finishes is shown.
 */
static int sweet_fb_hold_ms = 0;
/*
 * 0321: a render scale under the 2x one. kern.sweet_fb_rscale (permille, 400 to 1000; boot-arg sweetrscale, default
 * 1000) says how much of the 2x surface the window server's GPU driver drew: HakiosMetal draws the frame into the
 * top-left rscale x rscale part of it (viewport and scissor scaled; the surface sampled back for blurs is that part,
 * as a smaller texture), and the panel copy scales that part instead of the whole surface. iOS keeps its 2x geometry,
 * assets and caches (at 1x Settings, the wallpaper and widgets broke). kern.sweet_fb_ids: the IOSurface ids swapped
 * onto the panel lately, so that the driver scales those passes only.
 */
static int sweet_fb_rscale = 1000;
static uint32_t sweet_fb_ids[4];
static unsigned sweet_fb_ids_next;
#define SWEET_FB_LOW_SIZE       0x03000000ull
static uint64_t sweet_fb_low_used;

/*
 * sapphire 0336: diagnostics of the presentation path, read with sysctl kern.sweet_fb_diag. Counters and histograms
 * only: nothing here changes when or what is presented. What they answer:
 *   - how far apart the SwapEnds are, and how far apart the frames the panel is really given are, counted in the
 *     panel's own refreshes (INTF_FRAME_COUNT at each flip: a flip written during refresh c is latched at c + 1, so
 *     two flips in the same refresh mean the first was never seen);
 *   - how long a frame waits from its SwapEnd to its flip, and who found its copy done (the next swap or a tick);
 *   - how many frames had their copy replaced in the back buffer before any flip: drawn, copied and never shown;
 *   - how late the ticks of the vsync timer run and how often its grid restarts;
 *   - in which eighth of the refresh (INTF_LINE_COUNT) the flips are written;
 *   - how often the window server calls SwapWait (selector 6), which is answered at once.
 * The reader takes two readings and subtracts.
 */
#define SWEET_FBD_N     13
struct sweet_fbd_hist {
	uint64_t n[SWEET_FBD_N];
	uint64_t count, sum, max;
};
/* microseconds; bucket k is k + 1 refreshes of 8333 us: 1 2 3 4 5 6 7 8, 9-10, 11-12, 13-18, more */
static const uint32_t sweet_fbd_iv_us[] = { 12500, 20833, 29167, 37500, 45833, 54167, 62500, 70833, 87500, 104167, 154167 };
/* microseconds: under 2 4 8 12 16 24 32 48 64 100 200 ms, more */
static const uint32_t sweet_fbd_lat_us[] = { 2000, 4000, 8000, 12000, 16000, 24000, 32000, 48000, 64000, 100000, 200000 };
/* microseconds: under 50 100 200 400 800 1600 3200 6400 12800, more */
static const uint32_t sweet_fbd_late_us[] = { 50, 100, 200, 400, 800, 1600, 3200, 6400, 12800 };
static struct {
	struct sweet_fbd_hist swap_iv, flip_iv, swap_to_flip, tick_late;
	uint64_t swaps, flips, flips_by_swap, flips_by_tick, cpu_flips, overwritten, grid_restarts, swap_waits;
	uint64_t cleans, clean_us, presents, present_us, present_max_us;
	uint64_t holds, hold_us;                /* 0338: SwapEnds that waited for the copy before theirs, and for how long */
	uint64_t flip_eighth[8];
	uint64_t last_swap, pend_swap;          /* mach_absolute_time of the last SwapEnd, and of the frame awaiting its flip */
	uint32_t last_flip_frame;               /* INTF_FRAME_COUNT at the last flip */
	bool     have_flip;
} sweet_fbd;

static uint64_t
sweet_fbd_us(uint64_t a, uint64_t b)
{
	uint64_t ns;
	absolutetime_to_nanoseconds(b - a, &ns);
	return ns / 1000;
}

static void
sweet_fbd_bucket(struct sweet_fbd_hist *h, unsigned k, uint64_t v)
{
	OSAddAtomic64(1, (volatile SInt64 *)&h->n[k < SWEET_FBD_N ? k : SWEET_FBD_N - 1]);
	OSAddAtomic64(1, (volatile SInt64 *)&h->count);
	OSAddAtomic64((SInt64)v, (volatile SInt64 *)&h->sum);
	if (v > h->max) {
		h->max = v;
	}
}

static void
sweet_fbd_add(struct sweet_fbd_hist *h, const uint32_t *edges, unsigned n_edges, uint64_t v)
{
	unsigned k = 0;
	while (k < n_edges && v >= edges[k]) {
		k++;
	}
	sweet_fbd_bucket(h, k, v);
}

/* A flip is being written (scanFrom): intf is the INTF_1 mapping (INTF at + 0x800), or NULL. */
static void
sweet_fbd_flip(volatile uint32_t *intf)
{
	uint64_t now = mach_absolute_time(), pend = sweet_fbd.pend_swap;

	OSAddAtomic64(1, (volatile SInt64 *)&sweet_fbd.flips);
	if (pend != 0 && now > pend) {
		sweet_fbd_add(&sweet_fbd.swap_to_flip, sweet_fbd_lat_us, sizeof(sweet_fbd_lat_us) / sizeof(sweet_fbd_lat_us[0]),
		    sweet_fbd_us(pend, now));
	}
	sweet_fbd.pend_swap = 0;
	if (intf != NULL) {
		uint32_t frame = intf[(0x800 + 0x0ac) / 4], line = intf[(0x800 + 0x0b0) / 4];
		uint32_t hsync = intf[(0x800 + 0x008) / 4] >> 16;
		uint32_t vtotal = hsync != 0 ? intf[(0x800 + 0x00c) / 4] / hsync : 0;
		if (vtotal != 0) {
			sweet_fbd.flip_eighth[((uint64_t)line * 8 / vtotal) & 7]++;
		}
		if (sweet_fbd.have_flip) {
			uint32_t d = frame - sweet_fbd.last_flip_frame;
			sweet_fbd_bucket(&sweet_fbd.flip_iv, d <= 8 ? d : d <= 10 ? 9 : d <= 12 ? 10 : d <= 18 ? 11 : 12, d);
		}
		sweet_fbd.last_flip_frame = frame;
		sweet_fbd.have_flip = true;
	}
}

/*
 * sapphire 0340: the frame queue. kern.sweet_fb_queue=1 (boot-arg sweetfb_queue=1; default 0: as before).
 *
 * As it is without it (measured with 0336: "present: 0", "flips by swap 0"): a SwapEnd only records its swap (swap():
 * `now` needs fFadeOrig == NULL, and fFadeOrig comes with the flip buffers, which this phone has had since 0233); the
 * 120 Hz timer hands it to present() at its next tick, replacing one not yet presented; the GPU's copy goes into the
 * one back buffer; and the flip is written by the first tick that finds the fence of the LAST copy queued done. A
 * frame whose copy is still pending when the next one is presented is copied over in that buffer and never shown
 * (13.8 % of them, measured), and a frame waits for a tick twice (up to 8.3 ms each time).
 *
 * With the queue:
 *   - a frame is presented in SwapEnd itself: its copy is queued at once, behind the drawing that made it;
 *   - there are three flip buffers and a queue of (buffer, fence, the frame's surfaces). The copy goes into a buffer
 *     that is neither the one the MDP reads, nor the one it is leaving (it reads that one until it has taken the last
 *     flip, at its vsync), nor one that holds a frame still waiting: no frame is copied over, and nothing is written
 *     into what the panel is showing;
 *   - a thread flips the oldest frame as soon as its copy's fence is done, looking every kern.sweet_fb_queue_poll_us
 *     (500) while there is one to wait for, and not before the MDP has taken the flip before it (INTF_FRAME_COUNT has
 *     moved on since it was written, two refreshes if it was written in the last sixteenth of one): two flips in one
 *     refresh would leave the first unseen. The timer's ticks look too;
 *   - a frame's surfaces stay in use until it is flipped: whoever draws cannot run further ahead than its surfaces;
 *   - a SwapEnd that finds no free buffer (two frames waiting, or one and the buffer being left), or as many frames
 *     waiting as kern.sweet_fb_queue_depth allows (2; 1 keeps whoever draws one frame less ahead of the panel, at the
 *     price of more waits), waits, up to kern.sweet_fb_queue_hold_ms (50; boot-arg sweetfb_queue_hold). After that
 *     the newest waiting frame gives up its buffer to the new one and is never shown (counted in 0336's
 *     "overwritten"): the caller is never held longer, and the last frame of a run is always the one that stays.
 * present() itself is unchanged: the queue puts the buffer it chose where present() looks for the back buffer
 * (slot fFlipIdx ^ 1 of fFlip[]), and takes over the flip that present() leaves pending (fFlipWait). Switching the
 * sysctl while frames are in flight is safe either way: each side takes over what the other left (presentQueued(),
 * queueDrain()).
 */
#include <kern/thread.h>
#define SWEET_FB_BUFS           3
static int sweet_fb_queue = 0;
static int sweet_fb_queue_hold_ms = 50;
static int sweet_fb_queue_poll_us = 500;
static int sweet_fb_queue_depth = 2;
static struct {
	uint64_t frames, flips, flips_thread, holds, hold_us, hold_max_us, timeouts, forced, dropped, drained, adopted;
	uint64_t by_time, late, polls, depth_max;
	struct sweet_fbd_hist lat;              /* from the SwapEnd coming in to the flip, waits included */
} sweet_fbq;
extern uint32_t sapphire_gpu_copy_fence(void);          /* SapphireGpu.cpp: the fence of the last copy queued */
extern bool sapphire_gpu_fence_done(uint32_t fence);    /* ... and whether a given one is done */

/*
 * sapphire 0349: the display engine as it is, read only. Nothing here writes a register.
 *
 * kern.sweet_fb_regs: the registers of every block the panel's picture goes through -- MDP TOP, CTL0, the VIG0 plane
 * with its scaler (QSEED3LITE, +0xa00) and its CSC (+0x1a00), the layer mixer LM0, the ping-pong PP0 and the interface
 * INTF1 -- as they are when read, and the plane and its scaler as the bootloader left them (read in start(), before
 * this kernel's first flip). Blocks and sizes are the phone's device tree's (qcom,sde-*-off and -size, from mdp_phys
 * 0x5e00000) and Linux's catalog's (dpu_6_3_sm6115.h, from 0x5e01000). This kernel writes, of all of it, only VIG0's
 * SRC0_ADDR and SRC_YSTRIDE0, CTL0's CTL_FLUSH and INTF1's FRAME_LINE_COUNT_EN.
 *
 * kern.sweet_fb_latch: whether CTL_FLUSH reads back the bits a flip writes (0x20001: VIG0 and CTL) and when the MDP
 * clears them, against INTF_FRAME_COUNT. Linux and the stock driver take a flip as done when those bits read clear
 * after a vblank (dpu_encoder_phys_vid.c, sde_encoder_phys_vid.c: "double-check with hw that it accepted the flush
 * bits"); this kernel takes it as done when the frame counter has moved (0340's flipTaken()). A plane whose geometry is
 * rewritten (the scaler of a scanout) has to know: what is written while a flush is pending is latched with it. Each
 * flip is followed until its bits read clear: what reads back right after the write, by how many frames the counter
 * had moved when they cleared (by where in the refresh the flip was written), how long that took, and whether the
 * counter moved while they were still set.
 */
static const struct {
	const char *name;
	uint32_t    pa, len;
} sweet_fbr_blocks[] = {
	/* 0393: MDSS's own registers below MDP TOP -- HW_REV (+0x0), UBWC_DEC_HW_VERSION (+0x58), UBWC_STATIC (+0x144),
	 * UBWC_CTRL_2 (+0x150), UBWC_PREDICTION_MODE (+0x154) (Linux v6.12 msm_mdss.c:22-28): how the display decodes UBWC,
	 * which Linux writes for this SoC (sm6115_data: 0x11f, msm_mdss.c:172-177, 649-656) and this kernel does not */
	{ "MDSS", 0x05e00000, 0x160 },
	{ "TOP", 0x05e01000, 0x494 },
	{ "CTL0", 0x05e02000, 0x1dc },
	{ "VIG0", 0x05e05000, 0x1f8 },
	{ "VIG0 scaler", 0x05e05a00, 0xa0 },
	{ "VIG0 csc", 0x05e06a00, 0x100 },
	{ "LM0", 0x05e45000, 0x320 },
	{ "PP0", 0x05e71000, 0xd4 },
	{ "INTF1", 0x05e6b800, 0x2c0 },
};
#define SWEET_FBR_N             (sizeof(sweet_fbr_blocks) / sizeof(sweet_fbr_blocks[0]))
#define SWEET_FBR_VIG0          3       /* VIG0's entry in the table (its scaler the next): 2 before 0393 put MDSS first */
static volatile uint32_t *sweet_fbr_va[SWEET_FBR_N];  /* each block, mapped once in start() (its page, uncached) */
static uint32_t sweet_fbr_boot[(0x1f8 + 0xa0) / 4];  /* VIG0 0x000-0x1f8 and its scaler 0xa00-0xaa0 at start() */
static bool sweet_fbr_have_boot;

static void
sweet_fbr_map(void)
{
	for (unsigned i = 0; i < SWEET_FBR_N; i++) {
		uint32_t page = sweet_fbr_blocks[i].pa & ~0xfffu;
		vm_offset_t va = ml_io_map(page, 0x1000);
		sweet_fbr_va[i] = va != 0 ? (volatile uint32_t *)(va + (sweet_fbr_blocks[i].pa - page)) : NULL;
	}
	if (sweet_fbr_va[SWEET_FBR_VIG0] != NULL && sweet_fbr_va[SWEET_FBR_VIG0 + 1] != NULL) {
		for (uint32_t k = 0; k < 0x1f8 / 4; k++) {
			sweet_fbr_boot[k] = sweet_fbr_va[SWEET_FBR_VIG0][k];
		}
		for (uint32_t k = 0; k < 0xa0 / 4; k++) {
			sweet_fbr_boot[0x1f8 / 4 + k] = sweet_fbr_va[SWEET_FBR_VIG0 + 1][k];
		}
		sweet_fbr_have_boot = true;
	}
}

#define SWEET_FBL_EDGES         7
static const uint32_t sweet_fbl_us[SWEET_FBL_EDGES] = { 1000, 2000, 4000, 8000, 12000, 17000, 34000 };
static struct {
	uint64_t flips, readback_set, readback_clear, set_after_move, gave_up, samples;
	uint64_t at[16][4];                     /* sixteenth of the refresh at the write x frames moved when cleared (0 1 2 3+) */
	struct sweet_fbd_hist clear;            /* write to bits read clear, us */
	uint32_t last_readback, w_frame, w_sixteenth;
	uint64_t w_abs;
	bool     watching, moved_told;
} sweet_fbl;

/* A flip has just written CTL_FLUSH (scanFrom()): what reads back now, and where the panel's scan is. */
static void
sweet_fbl_wrote(volatile uint32_t *ctl, volatile uint32_t *intf)
{
	if (ctl == NULL || intf == NULL) {
		return;
	}
	uint32_t fl = ctl[0x18 / 4];
	uint32_t hsync = intf[(0x800 + 0x008) / 4] >> 16;
	uint32_t vtotal = hsync != 0 ? intf[(0x800 + 0x00c) / 4] / hsync : 0;
	uint32_t line = intf[(0x800 + 0x0b0) / 4];

	sweet_fbl.flips++;
	sweet_fbl.last_readback = fl;
	if ((fl & 0x20001) != 0) {
		sweet_fbl.readback_set++;
	} else {
		sweet_fbl.readback_clear++;
	}
	sweet_fbl.w_frame = intf[(0x800 + 0x0ac) / 4];
	sweet_fbl.w_sixteenth = vtotal != 0 && line < vtotal ? (uint32_t)((uint64_t)line * 16 / vtotal) : 15;
	sweet_fbl.w_abs = mach_absolute_time();
	sweet_fbl.watching = (fl & 0x20001) != 0;
	sweet_fbl.moved_told = false;
}

/* Whenever the frame queue or a tick looks: have the flush bits of the flip being followed been taken? */
static void
sweet_fbl_sample(volatile uint32_t *ctl, volatile uint32_t *intf)
{
	if (!sweet_fbl.watching || ctl == NULL || intf == NULL) {
		return;
	}
	uint32_t fl = ctl[0x18 / 4], d = intf[(0x800 + 0x0ac) / 4] - sweet_fbl.w_frame;
	uint64_t us = sweet_fbd_us(sweet_fbl.w_abs, mach_absolute_time());

	sweet_fbl.samples++;
	if ((fl & 0x20001) == 0) {
		sweet_fbl.at[sweet_fbl.w_sixteenth & 15][d < 3 ? d : 3]++;
		sweet_fbd_add(&sweet_fbl.clear, sweet_fbl_us, SWEET_FBL_EDGES, us);
		sweet_fbl.watching = false;
	} else if (us > 100000) {
		sweet_fbl.gave_up++;                    /* (not taken in 100 ms: the bits may not read back as taken) */
		sweet_fbl.watching = false;
	} else if (d != 0 && !sweet_fbl.moved_told) {
		sweet_fbl.set_after_move++;             /* the frame counter moved, the flush is still pending */
		sweet_fbl.moved_told = true;
	}
}

/*
 * sapphire 0350: the window server's frames scanned out by the display engine itself. kern.sweet_fb_scanout (boot-arg
 * sweetfb_scanout; default 0: as before):
 *   0  as before: every frame is copied by the GPU, scaled, into a flip buffer;
 *   1  the window server's screen surfaces are made in a pool the MDP can scan (SweetSurface.cpp), and a frame of one
 *      such surface goes to the panel as it is: VIG0 is pointed at the surface and its scaler (QSEED3LITE) scales the
 *      panel's window of it onto the panel, the window and the factor the copy used -- no copy, no flip buffer,
 *      nothing of the GPU's. Needs the frame queue (kern.sweet_fb_queue=1), whose thread flips;
 *   2  the pool only: surfaces made in it, frames still copied (to tell the memory apart from the presentation).
 * Surfaces made before it was set stay where they are: the window server makes its screen surfaces when it starts
 * (restarting backboardd makes them again). Anything else goes the copy's way, as before, in the same queue and in
 * order: a frame with more than one layer, a surface not from the pool or of another size or format, one swapped
 * again while the panel may still read it or a frame of it waits, a plane the bootloader did not leave as start()
 * expects (then it never starts: kern.sweet_fb_scanout_diag says why). Setting 0 (or 2) takes effect at the next swap,
 * and a frame of the pool that is on the panel then is copied into a flip buffer at once (the latest swap presented
 * again the copy's way), so that the panel leaves the scaled plane without waiting for the window server.
 *
 * What a scan needs, and how it is had:
 *   - memory the MDP reaches: a physical address below 4 GiB (the plane's address register has 32 bits, and the
 *     display's SMMU stream bypasses, as it does for the flip buffers): the pool, SWEET_FB_SCAN_SLOTS slots of 5 MiB
 *     from 0xe5000000, RAM the kernel does not manage, after the flip buffers' pool (0xe2000000-0xe5000000) and below
 *     the QSEE pool (0xe8000000);
 *   - one way of presenting while the window server's ring is the same (0354): if one of the surfaces it swaps cannot
 *     be scanned out (outside the pool, another size or format), all its frames are copied until that surface is gone
 *     (freed or released by the window server), so that the plane does not change mode frame by frame; and a frame of
 *     more than one layer, which the 2x path cannot compose (present() copies one layer only), shows its layer 0 when
 *     that is the window server's full-screen surface, and nothing otherwise (the panel keeps its frame), in either mode;
 *   - the surface left alone while the panel may read it: in use (sweet_hold) from its SwapEnd until the MDP has taken
 *     the flip of a later frame. CoreAnimation does not look at the use count of its screen surfaces: it learns this
 *     from SwapWait, answered from the same state (0353, sweet_fb_swapwait below), and then draws into another
 *     surface or makes one more; a surface swapped again while still in use goes the copy's way;
 *   - the frame drawn before it is shown: the flip waits for the msm fence of the last submission made before the
 *     SwapEnd (HakiosMetal's commit submits before it returns, and the CP runs one ring in order), and for any later
 *     one that named the surface (its gpuFence); a hung GPU is reset after 2 s and its fences jump ahead (0317);
 *   - the plane rewritten only when it cannot be latched half written: the MDP has taken the last flush (CTL_FLUSH's
 *     bits clear) and the scan is between a sixteenth and three quarters of the refresh, milliseconds from the next
 *     frame start, where the registers are latched; a write takes microseconds. A flip of a flip buffer rewrites the
 *     plane back the same way.
 * The scaled plane is Linux's for an RGB plane on this scaler (dpu_plane.c: _dpu_plane_setup_scaler3,
 * _dpu_plane_setup_pixel_ext; dpu_hw_util.c: dpu_hw_setup_scaler3; dpu_hw_sspp.c: setup_rects, setup_pe_config), which is
 * also the stock driver's when userspace gives no scaler data (sde_plane.c, _sde_plane_setup_scaler3): bilinear, no
 * coefficient table (the tables are userspace's, through the lut_sep property: the kernel has none), phase steps
 * (src << 21) / dst with a zero initial phase, preload 4 x 2 (scaler 3.0), pixel extension the size of the source, PE
 * override on. The unscaled plane is what the bootloader left (read in start()) and goes back at the first flip of a flip
 * buffer.
 */
/*
 * sapphire 0354: eight slots (0xe5000000-0xe7800000, below the QSEE pool). What CoreAnimation keeps (QuartzCore 20H392,
 * dev-notes/investigacion/18 §3.13): its screen surfaces are named "CA Framebuffer" (IOSurfaceName, set after they are
 * made, so the pool cannot tell them by it); three pages hold at most three of them, and when what a page needs changes
 * (current_page_surface(): size, format, protection; at a display on, for one) it makes new ones while the old ones stay
 * until their pages are taken back and collect_buffers() frees them: three and three. A restarted window server makes its
 * own while the framebuffer still holds the surface the panel shows. With four slots the pool runs out (dev-notes 03 M22:
 * "4 of 4 slots in use, 14 refused") and a surface of the ring goes the copy's way for as long as it lives.
 */
#define SWEET_FB_SCAN_PA        0xE5000000ull
#define SWEET_FB_SCAN_SLOTS     8
#define SWEET_FB_SCAN_SLOT      0x00500000u
#define SWEET_FB_FMT_BGRA       0x42475241
static int sweet_fb_scanout = 0;
static int sweet_fb_ubwc = 0;                    /* 0398: disabled unless direct scanout and GPU mode 1 */
static uint64_t sweet_fb_ubwc_flips, sweet_fb_ubwc_errors;
static uint32_t sweet_fb_ubwc_last_error;
static bool sweet_fb_scan_ready;                /* start() found the plane as it should be and set up the pool */
extern "C" int sapphire_gpu_ubwc_mode(void);
/*
 * sapphire 0352: when a flip counts as taken (flipTaken()). CTL_FLUSH holds the bits a flip writes until the MDP takes
 * them, as Linux and the stock driver read it (dpu_encoder_phys_vid.c, sde_encoder_phys_vid.c, vblank): measured on the
 * phone with 0349, the bits read set right after the write in 14 386 of 14 388 flips (the two others were written as
 * the MDP took them), and they clear at the programmable fetch (INTF_PROG_FETCH_START 0x156ad1: line 2420 of 2432), ~12
 * lines before INTF_FRAME_COUNT moves. 0340 takes a flip as taken when the counter has moved (twice if written in the
 * last sixteenth); a flip written later than line ~2420 is taken a refresh later, and the counter alone cannot tell.
 * kern.sweet_fb_flush_taken (boot-arg sweetfb_flush_taken):
 *   0 (default)  as 0340, and with the bits once a frame of a surface has been on the panel (as 0350);
 *   1            by the bits for every flip, once 16 flips have shown that they read back (no more than one in 64 read
 *                clear right after the write); by the counter otherwise.
 */
static int sweet_fb_flush_taken = 0;
static struct {
	uint64_t frames, flips, late_draw, not_one_layer, not_pool, unfit, reused, queue_off, swapwait_off;
	uint64_t held_off, blockers, not_shown, base_only; /* 0354: copied for a surface of the ring that cannot be; blockers seen;
	                                         * frames not shown; frames of several layers shown by their layer 0 */
	uint64_t holds, hold_us, hold_max_us, timeouts, to_scaled, to_unscaled, unscaled_waits, unscaled_wait_max_us;
	uint64_t window_waits, released, stops, stuck;
} sweet_fbs;
extern uint32_t sapphire_gpu_msm_fence(void);           /* SapphireGpu.cpp: the last msm submission's fence */
extern bool sapphire_gpu_msm_fence_done(uint32_t fence); /* ... and whether a given one is done */

/*
 * sapphire 0353: SwapWait (selector 6, scalars {id, flags, timeout}) answered from the frame queue's state.
 *
 * What CoreAnimation does with it (QuartzCore 20H392, CA::WindowServer::IOMFBDisplay; dev-notes/investigacion/18
 * §3.10, with addresses):
 *   - it draws its frames into a ring of three pages (AppleDisplay's constructor: 3, or 1 with page flipping
 *     disabled). A page holds the buffer (an IOSurface) its frame was drawn into and the id of the swap that showed it;
 *   - before each frame (can_update_) it looks at the two pages that are not the latest swap's, oldest first. A page is
 *     taken back once SwapWait(its id, 0x80000000) answers 0; then its buffer is free. The first one is asked through
 *     swap_wait_timeout(): every 100 us for up to 1 ms while the answer is not 0. If the oldest is still busy after
 *     that, the frame is not drawn on this tick of CoreAnimation's own 120 Hz timer (the A11 path does not use the
 *     vsync notifications, selector 72);
 *   - the frame is drawn into the free buffer least out of date, or into a new one made then if none is free
 *     (allocate_buffer). Answered 0 at once, every page but the latest is free at every frame: two buffers are enough,
 *     and the one drawn into next is the one the panel may still be showing;
 *   - flags: 0x80000000 answer now, kIOReturnBusy if not yet (can_update_ compares with it); 1 the swap's frame is on
 *     the panel (the latest page, swapped_frame_displayed_p()); 2 every swap so far is on the panel, id 0 (idle(),
 *     set_all_layers_to_null(), a synchronous finish_update(), is_ready()); neither of 1 and 2: the swap's surfaces are
 *     no longer used (a page taken back, its buffer drawn into next). IOMobileFramebuffer's kern_SwapWait passes
 *     {id, flags, 0}.
 * Here, with the frame queue on:
 *   - a swap's frame is on the panel once the MDP has taken its flip (flipTaken()) or a later one, and nothing of that
 *     swap or before it waits in the queue;
 *   - the surfaces of a swap whose frame is scanned out as it is (0350) are in use while the frame waits in the queue,
 *     while it is on the panel and until the MDP has taken a later flip: what scanoutBusy() says. Those of a frame the
 *     GPU copies are free from its SwapEnd, as before (the copy runs on the GPU's one ring ahead of anything drawn into
 *     them later, 0201); those of one the CPU composes in present() too;
 *   - without 0x80000000 SwapWait waits for it, 100 ms at most (then kIOReturnTimeout); the timeout scalar is not used;
 *   - after a wait for every swap (flags 2 without 0x80000000) CoreAnimation clears the swap ids of all its pages, the
 *     current one's too (idle()): the page of the frame on the panel is then taken back without asking once the next
 *     frame is current. If that frame is one of a surface (scanned out; 0354), the first SwapEnd after such a wait is
 *     held until its frame is on the panel, 250 ms at most (swapEndHold()); a copied frame is in a flip buffer, and its
 *     surface is free.
 * kern.sweet_fb_swapwait (boot-arg sweetfb_swapwait):
 *   1 (default)  as above: CoreAnimation leaves the surface on the panel alone and makes a third one when it needs it;
 *                with the scanout the pool holds them (SWEET_FB_SCAN_SLOTS). The scanout (0350) needs 1 or 2 (0354):
 *                with 0 its frames are copied (counted in kern.sweet_fb_scanout_diag), and setting 0 while scanning
 *                copies the frame on the panel at once, as kern.sweet_fb_scanout=0 does;
 *   2            the surfaces of every swap stay in use until the MDP has taken the flip of a later one, copied frames'
 *                too (as an IOMobileFramebuffer that retains them): CoreAnimation keeps at most two frames ahead of the
 *                panel either way, with a third surface also when copying;
 *   0            SwapWait answers 0 at once, as before 0353 (the swap ids are kept all the same).
 * With the queue off it answers 0 at once. A swap still busy after 1 s of asking is answered 0 (counted): a wait that long
 * is not a frame's (a hung GPU is reset after 2 s, 0317).
 */
static int sweet_fb_swapwait = 1;
#define SWEET_FBW_FREE          0       /* SwapWait(id, 0 or 0x80000000): the swap's surfaces no longer used */
#define SWEET_FBW_SHOWN         1       /* SwapWait(id, 1 or 0x80000001): the swap's frame on the panel */
#define SWEET_FBW_ALL           2       /* SwapWait(0, 2, 3 or 0x80000003): every swap so far on the panel */
static struct {
	uint64_t asked[3], busy[3], waits, wait_us, wait_max_us, timeouts, forced, busy_max_us;
	uint64_t holds, hold_us, hold_max_us, hold_timeouts;    /* swapEndHold() */
} sweet_fbw;

/* The plane's registers the scaled scanout writes (VIG0's offsets; the scaler's at +0xa00), in the order Linux writes them. */
#define SWEET_PLANE_REGS        27
static const uint16_t sweet_plane_off[SWEET_PLANE_REGS] = {
	0x000, 0x008, 0x00c, 0x010,                     /* SSPP_SRC_SIZE, SRC_XY, OUT_SIZE, OUT_XY */
	0x100, 0x104, 0x108,                            /* SSPP_SW_PIX_EXT_C0_LR, _TB, _REQ_PIXELS */
	0x110, 0x114, 0x118,                            /* ... C1C2 */
	0x120, 0x124, 0x128,                            /* ... C3 */
	0xa90, 0xa94, 0xa98, 0xa9c,                     /* QSEED3_PHASE_INIT_Y_H, _Y_V, _UV_H, _UV_V */
	0xa10, 0xa14, 0xa18, 0xa1c,                     /* QSEED3_PHASE_STEP_Y_H, _Y_V, _UV_H, _UV_V */
	0xa20, 0xa40, 0xa44, 0xa48,                     /* QSEED3_PRELOAD, _SRC_SIZE_Y_RGB_A, _SRC_SIZE_UV, _DST_SIZE */
	0xa04,                                          /* QSEED3_OP_MODE */
	0x038,                                          /* SSPP_SRC_OP_MODE */
};

/*
 * The scaled plane's register values, in sweet_plane_off[]'s order: the part (sx, sy, sw, sh) of the surface onto
 * dw x dh of the panel at out_xy (the bootloader's), for the source format in src_format (SSPP_SRC_FORMAT: BIT(8) alpha,
 * BIT(14) 10-bit) and the bootloader's SSPP_SRC_OP_MODE in src_op. Arithmetic only, so that a host program can set it
 * beside Linux's own functions (dpu_plane.c, dpu_hw_util.c, dpu_hw_sspp.c of v6.12).
 */
static void
sweet_plane_scaled_regs(uint32_t *v, uint32_t sx, uint32_t sy, uint32_t sw, uint32_t sh, uint32_t dw, uint32_t dh,
    uint32_t out_xy, uint32_t src_format, uint32_t src_op)
{
	/* _dpu_plane_setup_scaler3: mult_frac(1 << PHASE_STEP_SHIFT, src, dst), the same for the chroma (RGB: no subsampling) */
	uint32_t step_x = (uint32_t)(((uint64_t)sw << 21) / dw), step_y = (uint32_t)(((uint64_t)sh << 21) / dh);
	uint32_t req = (sh << 16) | sw;                 /* _dpu_plane_setup_pixel_ext: the whole source, no extension */
	uint32_t op = 1u | (3u << 16) | (1u << 31);     /* enable, Y/RGB filter DPU_SCALE_BIL, blend_cfg 1 */

	if ((src_format & (1u << 14)) == 0) {
		op |= 1u << 14;                         /* 8-bit data (not DX) */
	}
	if ((src_format & (1u << 8)) != 0) {
		op |= (1u << 10) | (1u << 29);          /* alpha, filtered DPU_SCALE_ALPHA_BIL (scaler version not 0x1002) */
	}
	v[0] = (sh << 16) | sw;                         /* setup_rects */
	v[1] = (sy << 16) | sx;
	v[2] = (dh << 16) | dw;
	v[3] = out_xy;
	for (unsigned c = 0; c < 3; c++) {              /* setup_pe_config: LR, TB 0; C3's TB is written with C3's LR, also 0 */
		v[4 + 3 * c] = 0;
		v[5 + 3 * c] = 0;
		v[6 + 3 * c] = req;
	}
	v[13] = v[14] = v[15] = v[16] = 0;              /* dpu_hw_setup_scaler3: initial phases (left 0 by the plane) */
	v[17] = v[19] = step_x & 0xFFFFFF;
	v[18] = v[20] = step_y & 0xFFFFFF;
	v[21] = 4u | (2u << 8) | (4u << 16) | (2u << 24);       /* DPU_QSEED4_DEFAULT_PRELOAD_H/V (scaler 3.0) */
	v[22] = v[23] = (sw & 0x1FFFF) | ((sh & 0x1FFFF) << 16);
	v[24] = (dw & 0x1FFFF) | ((dh & 0x1FFFF) << 16);
	v[25] = op;
	v[26] = src_op | (1u << 31);                    /* setup_format: MDSS_MDP_OP_PE_OVERRIDE */
}

/*
 * The part of the window server's 2x surface (lw x lh) the panel (pw x ph) shows, in the surface's whole pixels: the
 * panel's window of the 3x picture at (offx, offy) of SWEET_FB_LOGICAL_W x _H, scaled by 2/3, of the part of the surface
 * drawn (0321: rscale permille, 400-1000). present()'s copy takes the same window in 1/256 pixels (sapphire_gpu_copy_scaled:
 * x and y << 8); here the origin is rounded down and the size to the nearest pixel.
 */
static void
sweet_scan_window(uint32_t lw, uint32_t lh, int32_t offx, int32_t offy, uint32_t pw, uint32_t ph, int rscale,
    uint32_t *sx, uint32_t *sy, uint32_t *sw, uint32_t *sh)
{
	int rs = rscale >= 400 && rscale <= 1000 ? rscale : 1000;
	uint64_t kx = ((uint64_t)lw << 8) * (uint64_t)rs / 1000, ky = ((uint64_t)lh << 8) * (uint64_t)rs / 1000;

	*sx = (uint32_t)(((uint64_t)(uint32_t)offx * kx / SWEET_FB_LOGICAL_W) >> 8);
	*sy = (uint32_t)(((uint64_t)(uint32_t)offy * ky / SWEET_FB_LOGICAL_H) >> 8);
	*sw = (uint32_t)((((uint64_t)pw * kx / SWEET_FB_LOGICAL_W) + 128) >> 8);
	*sh = (uint32_t)((((uint64_t)ph * ky / SWEET_FB_LOGICAL_H) + 128) >> 8);
}

struct SweetLowBuf {
	IOMemoryMap *map;
	uint8_t     *va;
	uint64_t     pa;
	size_t       len;
	void        *getBytesNoCopy(void) { return va; }
	size_t       getLength(void) { return len; }
	uint64_t     getPhysicalSegment(uint64_t, void *, int) { return pa; }
	void         release(void) { }      /* the pool is never given back */
	static SweetLowBuf *alloc(size_t len);
};

SweetLowBuf *
SweetLowBuf::alloc(size_t len)
{
	uint64_t size = (len + 0xffffull) & ~0xffffull;
	if (sweet_fb_low_used + size > SWEET_FB_LOW_SIZE) {
		return NULL;
	}
	IOMemoryDescriptor *md = IOMemoryDescriptor::withPhysicalAddress(SWEET_FB_LOW_PA + sweet_fb_low_used, size,
	    kIODirectionInOut);
	IOMemoryMap *m = md ? md->map(kIOMapWriteCombineCache) : NULL;    /* Normal NC: Device would fault on memcpy */
	OSSafeReleaseNULL(md);
	SweetLowBuf *b = m ? IONew(SweetLowBuf, 1) : NULL;
	if (b == NULL) {
		OSSafeReleaseNULL(m);
		return NULL;
	}
	b->map = m;
	b->va = (uint8_t *)m->getVirtualAddress();
	b->pa = SWEET_FB_LOW_PA + sweet_fb_low_used;
	b->len = len;
	sweet_fb_low_used += size;
	bzero(b->va, len);
	return b;
}

class AppleCLCD : public IOMobileFramebuffer {
	OSDeclareDefaultStructors(AppleCLCD);

	IOWorkLoop         *fWorkLoop;
	IOTimerEventSource *fVsync;
	uint64_t            fVsyncCount;

	static void vsyncFired(OSObject *owner, IOTimerEventSource *sender);
	void publishModes(void);
	uint64_t            fNextVsync;     /* 0201: the next vsync, on an absolute 60.000 Hz grid */
	void armVsync(IOTimerEventSource *t);

public:
	uint64_t      fPhys;
	uint32_t      fWidth, fHeight, fRowBytes;
	uint32_t      fLogicalW, fLogicalH;     /* size reported to userland */
	int32_t       fOffX, fOffY;             /* logical origin of the panel */
	bool          fScale2;                  /* 0239: iOS draws at 2x (750 x 1624), the GPU scales it to the panel */
	uintptr_t     fPixels;          /* kernel mapping of the panel memory */
	IOMemoryMap  *fMap;
	SweetSurface *fSurface;         /* the panel memory as an IOSurface */
	IOLock       *fLock;            /* swaps and client list */
	OSArray      *fClients;
	uint32_t      fSwapID;
	bool          fConsolePaused;
	uint8_t       fPending[SWEET_FB_SWAP_REC_SIZE];     /* latest swap, not yet shown */
	bool          fDirty;
	uint32_t      fWakeTicks;     /* 0295: vsync ticks left before the frame from before "off" may be shown again */
	/* topaz direct MDP scanout (VIG0 / CTL0 from TopazGpuW disp.c) */
	volatile uint32_t *fVig0;
	volatile uint32_t *fCtl0;
	volatile uint32_t *fIntf1;      /* INTF_1 (DSI0) timing engine: refresh diagnostics */
	uint32_t      fIntfFrames;
	uint64_t      fScanPhys;      /* what VIG0 currently fetches */
	uint32_t      fScanStride;
	uint32_t      fBootSrc, fBootStride;
	bool          directPresent(const uint8_t *rec);
	void          scanFrom(uint64_t pa, uint32_t stride);
	uint32_t      fFlushTicks;    /* vsync ticks between panel copies */
	uint32_t      fRateHz;        /* vsync and published refresh rate: 60, 90, 120, 180 or 240 (sweetfb_hz) */
	/* 0286: a frame still being sent to the GPU (kern.sweet_fb_frame_await/_sent, backboardd's GL bridge) */
	uint32_t      fAwaitSid;      /* its number, the bridge's (0: none) */
	bool          fAwaitSent;     /* ... reported sent before its swap came */
	uint32_t      fHeldSeq;       /* the number of the frame the held swap waits for */
	uint8_t      *fHeldRec;       /* its swap, held back until then */
	SweetSurface *fHeldHold[4];
	uint32_t      fHeldTicks;
	uint64_t      fHeldCount, fHeldLate, fHeldForced, fSentCount;
	bool          fPowerOff;      /* RequestPowerChange(0) */
	uint32_t      fPowerOns;      /* 0236: display-on requests so far, up to 2 (the second: the UI is up) */
	uint64_t      fStartAbs;      /* mach_absolute_time() at start */
	uint32_t      fFadeStep;      /* fade-out steps done while off */
	/* topaz fade: cached copy of the last frame and a contiguous output frame */
	IOBufferMemoryDescriptor *fFadeOrig;
	/* kernel-drawn frames (console, fade) are double buffered like iOS's:
	 * built in the back buffer, then the MDP flips to it at vsync */
	SweetLowBuf  *fFlip[SWEET_FB_BUFS];     /* 0340: a third one for the frame queue (NULL if the low pool had no room) */
	uint64_t      fFlipPhys[SWEET_FB_BUFS];
	uint32_t      fFlipIdx;
	bool          fConsShown;     /* console frames on the panel (no iOS frame yet) */
	void          flipFrom(const uint8_t *src, uint32_t srcStride);
	void          splashEnd(void);
	uint8_t      *backBuffer(void) { return (uint8_t *)fFlip[fFlipIdx ^ 1]->getBytesNoCopy(); }
	void          flipBack(void);
	bool          fFlipWait;      /* 0235: the back buffer waits for the GPU's copy before the MDP is pointed at it */
	bool          flipWhenCopied(bool wait);
	SweetLowBuf  *fCons;        /* console shadow frame (contiguous, below 4 GiB) */
	uint64_t      fConsPhys;
	const uint8_t *fLastVA;       /* last directly scanned frame (panel window) */
	uint32_t      fLastStride;
	SweetSurface *fLastSurf;      /* keeps fLastVA alive */
	SweetSurface *fPendHold[4], *fShowHold[4], *fRetireHold[4];   /* in-use holds */
	void          fadeStep(const uint8_t *src, uint32_t srcStride, uint32_t step);

	bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	IOReturn setProperties(OSObject *properties) APPLE_KEXT_OVERRIDE;
	/* topaz panel brightness: DCS 0x51 over DSI0 (MakrSas/topaz-woa TopazGpuW disp.c) */
	volatile uint8_t *fDsi;
	SweetLowBuf  *fDsiBuf;
	uint64_t      fDsiBufPhys;
	IOLock       *fBlLock;
	uint32_t      fBlLevel;       /* panel DBV 1..0x7ff, kept while the display is off */
	uint32_t      fBlSent;        /* last level sent to the panel */
	thread_call_t fBlCall;        /* sends the latest level off backboardd's thread */
	static void   blWork(thread_call_param_t p0, thread_call_param_t p1);
	bool          dsiDcsLong(const uint8_t *payload, uint32_t len);
	bool          dsiSend(const uint8_t hdr[4], const uint8_t *payload, uint32_t len);
	void          setPanelOn(bool on);
	void          setPanelLevel(uint32_t level);
	void addClient(IOMobileFramebufferUserClient *client);
	void removeClient(IOMobileFramebufferUserClient *client);
	void swap(const uint8_t *rec);
	bool present(const uint8_t *rec);
	void presentNow(const uint8_t *rec, SweetSurface **held);
	void presentHeld(bool late);
	IOLock       *fPresentLock;   /* 0201: present() from SwapEnd and from the vsync timer */
	bool          fNow;           /* 0201: boot-arg sweetfb_now (default 1): present at SwapEnd */
	/* 0340: the frame queue (see sweet_fb_queue above). All of it with fPresentLock. */
	struct SweetGeo {
		uint32_t      sx, sy, sw, sh; /* 0350: the part of a surface the panel shows, in its pixels */
	};
	struct SweetQFrame {
		uint64_t      pa;             /* the flip buffer the GPU copies the frame into */
		uint32_t      fence;          /* that copy's fence */
		uint64_t      queued, arrived; /* when the copy was queued; when the frame's SwapEnd came in */
		SweetSurface *held[4];        /* the frame's surfaces: in use until it is flipped */
		/* 0350: a frame of a pool surface, scanned out as it is: pa is the surface's, fence an msm fence, and its
		 * surfaces stay in use after its flip, until the MDP has taken a later one */
		bool          scan;
		SweetSurface *surf;
		uint32_t      stride;
		uint32_t      metaStride;
		uint64_t      metaPa;
		bool          ubwc;
		struct SweetGeo geo;
		uint32_t      swap;           /* 0353: the id of the swap it shows (0: none, the timer's or an adopted copy) */
		uint32_t      ox, oy;         /* 0354: a frame of a surface: the window's origin, in pa already */
	};
	struct SweetQFrame fQ[SWEET_FB_BUFS];     /* the frames waiting for the panel, oldest first */
	uint32_t      fQCount;
	uint64_t      fQRetirePa[2];  /* the flip buffers the MDP may still be reading until it takes the last flip (0: none) */
	bool          fQLatchWait;    /* the last flip written has not been seen taken yet */
	bool          fQFlipLate;     /* ... it was written in the last sixteenth of a refresh (taken a refresh later, maybe) */
	uint32_t      fQFlipFrame;    /* ... INTF_FRAME_COUNT when it was written */
	uint64_t      fQFlipAbs;      /* ... and when */
	uint64_t      fQArrived;      /* when the SwapEnd now being presented came in */
	bool          fQThread;       /* the thread that flips is running */
	bool          queueOn(void);
	bool          flipTaken(uint64_t now);
	void          queueFront(uint64_t pa);
	int           queueFreeSlot(void);
	void          queueDrop(void);
	void          queueFlip(void);
	void          queueDrain(void);
	bool          presentQueued(const uint8_t *rec, SweetSurface **held, bool *queued);
	void          queueThread(void) __attribute__((noreturn));
	static void   queueThreadMain(void *self, wait_result_t result) __attribute__((noreturn));
	/* 0350: the scanout of the window server's surfaces (sweet_fb_scanout). All of it with fPresentLock. */
	uint32_t      fPlane0[SWEET_PLANE_REGS];      /* the plane's registers as the bootloader left them (unscaled) */
	uint32_t      fPlaneFormat, fScalerVersion;   /* SSPP_SRC_FORMAT and QSEED3_HW_VERSION at boot */
	const char   *fScanWhy;       /* why there is no scanout (start()), or NULL */
	bool          fPlaneScaled;   /* the plane is set up for a surface (scaled), not for a flip buffer */
	struct SweetGeo fPlaneGeo;    /* ... with this part of it */
	uint32_t      fPlaneStride;
	SweetSurface *fScanShow[4];   /* the surfaces of the frame on the panel, when it is a frame of a surface */
	SweetSurface *fScanLeave[4][4]; /* ... of frames left since the MDP was last seen to take a flip, oldest first */
	bool          fFlushKnown, fFlushReadback;    /* CTL_FLUSH has been read after 16 writes or more; it reads back (0352) */
	uint32_t      fFlushSet, fFlushClear;         /* 0352: what it read right after each write: the bits set, or clear */
	bool          fScanUsed;      /* 0352: a frame of a surface has been on the panel */
	void          flushNote(void);
	bool          fScanStuck;     /* the MDP did not take a flush of the scaled plane in 100 ms */
	void          scanoutInit(void);
	bool          scanoutOn(void);
	bool          scanoutBusy(SweetSurface *s);
	void          scanoutLeave(void);
	void          scanoutRetire(void);
	void          scanoutStop(void);
	bool          planeWritable(void);
	void          planeScaled(uint64_t pa, uint32_t stride, const struct SweetGeo *g);
	void          planeUnscaled(void);
	void          flipNote(uint64_t pa, bool force);
	bool          frameDone(const struct SweetQFrame *f, uint64_t now);
	void          frameShow(struct SweetQFrame *f);
	bool          presentScanout(const uint8_t *rec, SweetSurface **held, bool *queued);
	/* 0353: SwapWait (sweet_fb_swapwait). Swap ids, all with fPresentLock (0: none). */
	uint32_t      fSwapShow;      /* the swap of the flip about to be written (frameShow(), present()'s own flips) */
	uint32_t      fFlipSwap;      /* the swap of the last flip written */
	uint32_t      fPanelSwap;     /* the swap of the last flip seen taken: the newest one on the panel */
	uint32_t      fScanShowSwap;  /* the swap whose surfaces are fScanShow[] */
	uint32_t      fScanLeaveSwap[4]; /* ... fScanLeave[f][] */
	uint32_t      fSwapBusyId[3]; /* by how it is asked (SWEET_FBW_*): the last swap answered busy */
	uint64_t      fSwapBusySince[3]; /* ... since when */
	bool          fSwapHoldNext;  /* a wait for every swap found a frame of a surface on the panel: the next SwapEnd waits */
	IOReturn      swapWait(uint32_t id, uint32_t flags, uint32_t held);
	void          swapEndHold(uint32_t id);
	/* 0354: surfaces of the window server's ring that cannot be scanned out (not retained: id and pointer), fPresentLock */
	struct SweetScanBlock {
		uint32_t      id;
		SweetSurface *s;
		uint8_t       why;            /* 1 not from the pool, 2 another size or format */
		uint64_t      since;
	};
	struct SweetScanBlock fScanBlock[8];
	uint32_t      fPlaneOX, fPlaneOY; /* the window's origin in the surface on the panel, taken through its address */
	bool          scanoutBlocked(void);
	void          scanoutBlock(SweetSurface *s, uint8_t why);
	const uint8_t *shownRecord(const uint8_t *rec, uint8_t **one);
	bool          swapShown(uint32_t id, uint32_t held);
	bool          swapFree(uint32_t id, uint32_t held);
};

/*
 * CoreBrightness (-[CBDisplayContaineriOS findBacklight]) matches
 * {IOPropertyMatch: {backlight-control: true}} and requires the class to
 * conform to AppleARMBacklight; without it there is no display module and
 * the brightness slider is dead. It then sets "brightness-nits" here
 * (SetBLDriverNitsValue) and IOMFBBrightnessLevel on the framebuffer.
 */
class AppleARMBacklight : public IOService {
	OSDeclareDefaultStructors(AppleARMBacklight);
public:
	AppleCLCD *fFB;
	IOReturn setProperties(OSObject *properties) APPLE_KEXT_OVERRIDE;
};

OSDefineMetaClassAndStructors(AppleARMBacklight, IOService);

/*
 * The graphics accelerator's service class. QuartzCore (CAMetalContextCreate,
 * once per process) polls IOServiceMatching("IOAcceleratorES") 300 times,
 * 100 ms apart, before it goes on: thirty seconds at every backboardd start.
 * One without MetalPluginName/MetalPluginClassName is found and then passed
 * over by Metal (MTLRegisterDevices), which leaves the renderers as they were.
 */
class IOAcceleratorES : public IOService {
	OSDeclareDefaultStructors(IOAcceleratorES);

public:
	using IOService::newUserClient;
	IOReturn newUserClient(task_t owningTask, void *securityID, UInt32 type, OSDictionary *properties,
	    IOUserClient **handler) APPLE_KEXT_OVERRIDE;
};

OSDefineMetaClassAndStructors(IOAcceleratorES, IOService);

/*
 * IOServiceOpen type 'SGPU': the Adreno for our Metal driver (SapphireGpu.cpp, user client).
 * Every other type goes where it went before, which for this service is nowhere.
 */
#define SAPPHIRE_GPU_CLIENT_TYPE 0x53475055u
extern IOReturn sapphire_gpu_new_client(IOService *provider, task_t task, void *security, UInt32 type,
    IOUserClient **handler);

IOReturn
IOAcceleratorES::newUserClient(task_t owningTask, void *securityID, UInt32 type, OSDictionary *properties,
    IOUserClient **handler)
{
	if (type == SAPPHIRE_GPU_CLIENT_TYPE) {
		return sapphire_gpu_new_client(this, owningTask, securityID, type, handler);
	}
	return IOService::newUserClient(owningTask, securityID, type, properties, handler);
}
OSDefineMetaClassAndStructors(AppleCLCD, IOMobileFramebuffer);

class IOMobileFramebufferUserClient : public IOUserClient {
	OSDeclareDefaultStructors(IOMobileFramebufferUserClient);

public:
	AppleCLCD          *fFB;
	task_t              fTask;
	mach_port_t         fPort;
	io_user_reference_t fFunc[SWEET_FB_NOTIFY_TYPES];
	io_user_reference_t fRefcon[SWEET_FB_NOTIFY_TYPES];

	bool initWithTask(task_t owningTask, void *securityID, UInt32 type) APPLE_KEXT_OVERRIDE;
	bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	IOReturn clientClose(void) APPLE_KEXT_OVERRIDE;
	IOReturn registerNotificationPort(mach_port_t port, UInt32 type,
	    io_user_reference_t refCon) APPLE_KEXT_OVERRIDE;
	IOReturn externalMethod(uint32_t selector, IOExternalMethodArguments *args,
	    IOExternalMethodDispatch *dispatch, OSObject *target,
	    void *reference) APPLE_KEXT_OVERRIDE;
	void notify(uint32_t type, const uint64_t *values, uint32_t count);
};

OSDefineMetaClassAndStructors(IOMobileFramebufferUserClient, IOUserClient);

/*
 * 0319: the Metal plug-in the accelerator names: 1 SapphireMetal (metal/ in the repository: Metal over the GL bridge),
 * 2 HakiosMetal (HackiOS's hakios-metal, ported to the A610: Metal straight on the GPU). boot-arg sweetmetal=N at boot,
 * sysctl kern.sweet_metal=N at run time (each process that asks for a device afterwards gets that one; backboardd must
 * be restarted to change the one CoreAnimation uses). 0 takes both names away.
 */
static IOAcceleratorES *sweet_accel;
static uint32_t sweet_metal_now;

static void
sweet_metal_name(IOAcceleratorES *accel, uint32_t metal)
{
	if (metal == 2) {
		accel->setProperty("MetalPluginName", "HakiosMetal");
		accel->setProperty("MetalPluginClassName", "HakiosMetalDevice");
	} else if (metal) {
		accel->setProperty("MetalPluginName", "SapphireMetal");
		accel->setProperty("MetalPluginClassName", "SapphireMTLDevice");
	} else {
		accel->removeProperty("MetalPluginName");
		accel->removeProperty("MetalPluginClassName");
	}
	sweet_metal_now = metal;
}

extern "C" int
sweet_metal_plugin(int set)
{
	if (set >= 0 && sweet_accel != NULL) {
		sweet_metal_name(sweet_accel, (uint32_t)set);
		IOLog("SWEET FB: Metal driver now %s\n", set == 2 ? "HakiosMetal" : set ? "SapphireMetal" : "none");
	}
	return (int)sweet_metal_now;
}

bool
AppleCLCD::start(IOService *provider)
{
	PE_Video video;

	if (!IOService::start(provider)) {
		return false;
	}
	topaz_modem_init();
	sweet_amfi_service_init();
	sweet_busy_init();
	sapphire_gpu_init();
	PE_current_console(&video);
	fPhys = video.v_baseAddr;
	fWidth = (uint32_t)video.v_width;
	fHeight = (uint32_t)video.v_height;
	fRowBytes = (uint32_t)video.v_rowBytes;
	if (fPhys == 0 || fWidth == 0 || fHeight == 0 || fRowBytes < fWidth * 4) {
		IOLog("SWEET FB: no boot framebuffer\n");
		return false;
	}
	int native = 0;
	PE_parse_boot_argn("sweetfb_native", &native, sizeof(native));
	fLogicalW = fWidth;
	fLogicalH = fHeight;
	if (!native && fWidth <= SWEET_FB_LOGICAL_W && fHeight <= SWEET_FB_LOGICAL_H) {
		fLogicalW = SWEET_FB_LOGICAL_W;
		fLogicalH = SWEET_FB_LOGICAL_H;
	}
	/*
	 * sapphire 0354: x is the multiple of 24 nearest the centre, (1125 - 1080) / 2 = 22.5: 24. The panel's window then
	 * starts on a 16-byte boundary at 3x (24 pixels, 96 bytes: the MDP fetches from 16-byte aligned addresses) and on a
	 * 64-byte one in the 2x picture (24 x 2/3 = 16 pixels: the GPU's blitter takes its source base 64-byte aligned), so
	 * that the scaled copy (present(), blit_scale()) and the scanout (presentScanout()) both crop by the source address
	 * alone, the same way, and neither depends on how its engine applies an origin (the blitter's GRAS_A2D_SRC_XMIN and
	 * YMIN, the plane's SRC_XY). The touch mapping (SweetHID.cpp) uses the same rule.
	 */
	fOffX = ((((int32_t)(fLogicalW - fWidth) / 2) + 12) / 24) * 24;
	fOffY = (int32_t)(fLogicalH - fHeight) / 2;
	/*
	 * 0239, boot-arg sweetfb_scale=2 (with the ADT's display-scale 2 and canvas 750 x 1624): iOS
	 * draws the same 375 x 812 points at 2x, 750 x 1624, and every frame's copy to the panel (the
	 * GPU's) scales the part of it the panel shows by 1.5. 2.25 times fewer pixels for the GPU to
	 * draw, blend and copy: this phone's GPU is bound by memory bandwidth (~2.5 ms a full-screen
	 * pass at 3x). fOffX/fOffY stay those of the 3x picture (the panel's window, and the touch
	 * mapping, are unchanged).
	 */
	/*
	 * 0240: the 2x picture is that of a real 2x iPhone, the XR's 828 x 1792 (414 x 896 points,
	 * the X's proportions): UIKit has a status bar only for screens it knows by their pixel width
	 * (_UIStatusBarVisualProvider_Split828, Split1125, ...), and 750 x 1624 got the red
	 * "rdar://45025538" bar. Boot-arg sweetfb_w (750 or 828, the default) chooses.
	 */
	int scale = 3, width2x = 828;
	PE_parse_boot_argn("sweetfb_scale", &scale, sizeof(scale));
	PE_parse_boot_argn("sweetfb_w", &width2x, sizeof(width2x));
	PE_parse_boot_argn("sweetrscale", &sweet_fb_rscale, sizeof(sweet_fb_rscale));
	PE_parse_boot_argn("sweetfb_hold", &sweet_fb_hold_ms, sizeof(sweet_fb_hold_ms));       /* 0338 */
	/* 0320: sweetfb_scale=1 too -- iOS draws its points at 1x (375 x 812, 414 x 896), four times fewer pixels
	 * than 2x for a GPU bound by memory bandwidth; fScale2 means scaled onto the panel, whatever the factor */
	fScale2 = (scale == 2 || scale == 1) && fLogicalW == SWEET_FB_LOGICAL_W && fLogicalH == SWEET_FB_LOGICAL_H &&
	    (width2x == 750 || width2x == 828);
	if (fScale2) {
		fLogicalW = (uint32_t)width2x;
		fLogicalH = width2x == 828 ? 1792 : SWEET_FB_LOGICAL_H * 2 / 3;
		if (scale == 1) {
			fLogicalW /= 2;
			fLogicalH /= 2;
		}
		IOLog("SWEET FB: iOS draws at 2x, %ux%u, scaled onto the panel by the GPU\n", fLogicalW, fLogicalH);
	}
	fLock = IOLockAlloc();
	fPresentLock = IOLockAlloc();
	{
		uint32_t now = 1;
		PE_parse_boot_argn("sweetfb_now", &now, sizeof(now));
		fNow = now != 0 && fPresentLock != NULL;
	}
	sweet_frame_fb = this;
	fClients = OSArray::withCapacity(4);
	fSurface = sweet_surface_create_physical(fPhys, fWidth, fHeight, fRowBytes, SWEET_FB_FORMAT_BGRA);
	fPixels = fSurface != NULL ? fSurface->kernelAddress() : 0;
	fWorkLoop = IOWorkLoop::workLoop();
	if (fLock == NULL || fClients == NULL || fPixels == 0 || fWorkLoop == NULL) {
		IOLog("SWEET FB: setup failed\n");
		OSSafeReleaseNULL(fSurface);    /* stays published: also held by the table */
		OSSafeReleaseNULL(fWorkLoop);
		return false;
	}
	fVsync = IOTimerEventSource::timerEventSource(this, &AppleCLCD::vsyncFired);
	if (fVsync == NULL || fWorkLoop->addEventSource(fVsync) != kIOReturnSuccess) {
		IOLog("SWEET FB: no vsync timer\n");
		return false;
	}
	fVig0 = (volatile uint32_t *)ml_io_map(0x05e05000, 0x1000);
	fCtl0 = (volatile uint32_t *)ml_io_map(0x05e02000, 0x1000);
	fDsi = (volatile uint8_t *)ml_io_map(0x05e94000, 0x1000);
	fDsiBuf = SweetLowBuf::alloc(page_size);
	fDsiBufPhys = fDsiBuf ? fDsiBuf->getPhysicalSegment(0, NULL, kIOMemoryMapperNone) : 0;
	fBlLock = IOLockAlloc();
	fBlLevel = 0x133;                       /* panel init level (stock DT) */
	fBlSent = ~0u;
	fBlCall = thread_call_allocate(&AppleCLCD::blWork, this);
	if (fDsi != NULL && fDsiBuf != NULL && fBlLock != NULL) {
		sweet_dsi_fb = this;
	}
	if (fDsi != NULL) {
		IOLog("SWEET FB: DSI0 ctrl %#x, brightness via DCS 0x51\n", *(volatile uint32_t *)(fDsi + 0x004));
		setProperty("IOMFBBrightnessLevel", (unsigned long long)(200u << 16), 32);
		/*
		 * Off unless sweetbl=1: with a backlight service CoreBrightness also
		 * runs digital dimming, a colour filter over every frame, which the
		 * CPU renderer cannot afford (the UI dropped to under 1 fps).
		 */
		/*
		 * 0210: off by default again (sweetbl=1: on). 0209 had it on: CoreBrightness then runs its
		 * digital dimming, a filter over every frame, and the phone lagged; without it, smooth.
		 */
		uint32_t blOn = 0;
		(void)PE_parse_boot_argn("sweetbl", &blOn, sizeof(blOn));
		AppleARMBacklight *bl = blOn ? OSTypeAlloc(AppleARMBacklight) : NULL;
		if (bl != NULL && bl->init()) {
			bl->fFB = this;
			bl->setProperty("backlight-control", true);
			bl->setProperty("brightness-nits", (unsigned long long)200, 32);
			bl->setProperty("max-nits", (unsigned long long)SWEET_FB_MAX_NITS, 32);
			if (bl->attach(this)) {
				bl->registerService();
				IOLog("SWEET FB: AppleARMBacklight published\n");
			}
		}
		OSSafeReleaseNULL(bl);
	}
	uint32_t accelOn = 1;
	(void)PE_parse_boot_argn("sweetaccel", &accelOn, sizeof(accelOn));
	IOAcceleratorES *accel = accelOn ? OSTypeAlloc(IOAcceleratorES) : NULL;
	if (accel != NULL && accel->init()) {
		accel->setName("sgx");
		/*
		 * boot-arg sweetmetal=1: name a Metal driver, so Metal (getMetalPluginClassForService)
		 * loads /System/Library/Extensions/SapphireMetal.bundle and makes its SapphireMTLDevice
		 * the system device (metal/ in the repository). Without it the service stays empty and
		 * Metal passes it over, as before.
		 */
		uint32_t metal = 0;
		(void)PE_parse_boot_argn("sweetmetal", &metal, sizeof(metal));
		if (metal) {
			sweet_metal_name(accel, metal);
		}
		if (accel->attach(this)) {
			sweet_accel = accel;
			sweet_accel->retain();
			accel->registerService();
			IOLog("SWEET FB: IOAcceleratorES published, %s\n", metal == 2 ? "Metal driver HakiosMetal" : metal ? "Metal driver SapphireMetal" :
			    "empty (no 30 s wait in CAMetalContextCreate)");
		}
	}
	OSSafeReleaseNULL(accel);
	/* dpu sm6115 catalog: intf_1 at mdp+0x6a800, mdp at 0x5e01000 */
	fIntf1 = (volatile uint32_t *)ml_io_map(0x05e6b000, 0x1000);
	if (fIntf1 != NULL) {
		volatile uint32_t *r = fIntf1 + 0x800 / 4;
		r[0x0a8 / 4] = 0x3;     /* FRAME_LINE_COUNT_EN */
		IOLog("SWEET INTF1: te_en %#x config %#x hsync %#x vsync_period %#x hctl %#x active_hctl %#x vstart %#x vend %#x fmt %#x cmd_trig %#x\n",
		    r[0x000 / 4], r[0x004 / 4], r[0x008 / 4], r[0x00c / 4], r[0x03c / 4], r[0x040 / 4],
		    r[0x01c / 4], r[0x024 / 4], r[0x090 / 4], r[0x084 / 4]);
	}
	sweet_fbr_map();                /* 0349: before the first flip, so that the plane is read as the bootloader left it */
	if (fVig0 != NULL && fCtl0 != NULL) {
		fBootSrc = fVig0[0x14 / 4];
		fBootStride = fVig0[0x24 / 4];
		fScanPhys = fBootSrc;
		fScanStride = fBootStride;
		IOLog("SWEET FB: MDP VIG0 src %#x stride %u (boot), direct scanout ready\n",
		    fBootSrc, fBootStride);
		sweet_scanout_set_geometry(fWidth, fHeight, fLogicalW, fLogicalH);
		vm_size_t fsz = (vm_size_t)fRowBytes * fHeight;
		for (int i = 0; i < 2; i++) {
			fFlip[i] = SweetLowBuf::alloc(fsz);
			fFlipPhys[i] = fFlip[i] ? fFlip[i]->getPhysicalSegment(0, NULL, kIOMemoryMapperNone) : 0;
		}
		fFadeOrig = IOBufferMemoryDescriptor::inTaskWithOptions(kernel_task, kIODirectionInOut, fsz, page_size);
		fCons = SweetLowBuf::alloc(fsz);
		if (fFlip[0] == NULL || fFlip[1] == NULL || fFadeOrig == NULL) {
			OSSafeReleaseNULL(fFlip[0]);
			OSSafeReleaseNULL(fFlip[1]);
			OSSafeReleaseNULL(fFadeOrig);
			OSSafeReleaseNULL(fCons);
		}
		if (fCons != NULL) {
			uint8_t *c = (uint8_t *)fCons->getBytesNoCopy();
			memcpy(c, (const void *)fPixels, fsz);  /* keep what the console drew */
			sweet_dcache_clean((vm_offset_t)(uintptr_t)c, (unsigned)fsz);   /* for panics */
			fConsPhys = fCons->getPhysicalSegment(0, NULL, kIOMemoryMapperNone);
			sweet_cons_fb = this;
			sweet_console_show_hook = sweet_cons_show;
			sweet_console_retarget((vm_offset_t)c);
			fConsShown = true;
			flipFrom(c, fRowBytes);
			IOLog("SWEET FB: console double buffered (shadow %#llx)\n", fConsPhys);
		} else {
			OSSafeReleaseNULL(fCons);
		}
		if (fFlip[0] != NULL) {
			/* 0340: the frame queue's third flip buffer, after everything else so that nothing moves */
			fFlip[2] = SweetLowBuf::alloc(fsz);
			fFlipPhys[2] = fFlip[2] ? fFlip[2]->getPhysicalSegment(0, NULL, kIOMemoryMapperNone) : 0;
		}
	}
	uint32_t hz = 10;
	PE_parse_boot_argn("sweetfb_hz", &hz, sizeof(hz));
	fFlushTicks = (hz == 0 || hz >= 60) ? 1 : 60 / hz;
	/*
	 * The panel's only timing is 120 Hz (dfps by the vertical front porch, the INTF counts 120
	 * frames a second): with sweetfb_hz=90/120 CoreAnimation is told that rate and the vsync ticks
	 * at it, so frames that are ready sooner are shown sooner. Below 60, ticks stay at 60.
	 *
	 * 0302: 180 and 240 too, a finer grid for CoreAnimation than the panel's. Its render server
	 * starts a frame on a vsync tick, so a frame that takes a little longer than a tick waits for
	 * the next: at 120, every frame of 8.4-16.7 ms (most of backboardd's on the Adreno 610) came
	 * 16.7 ms after the last, 60 a second. Told 240, such a frame takes three ticks of 4.2 ms
	 * (12.5 ms, 80 a second) or two (8.3 ms). The panel still refreshes at 120 and shows the latest
	 * frame at each of its refreshes.
	 */
	fRateHz = hz >= 240 ? 240 : hz >= 180 ? 180 : hz >= 120 ? 120 : hz >= 90 ? 90 : 60;
	fVsync->setTimeoutUS(1000000 / fRateHz);

	publishModes();
	setProperty(kIOUserClientClassKey, "IOMobileFramebufferUserClient");
	setProperty("external", false);
	setProperty("DisplayPipePlaneBaseAlignment", 64ULL, 32);
	registerService();
	fStartAbs = mach_absolute_time();
	IOLog("SWEET FB: AppleCLCD %ux%u stride %u at 0x%llx, default surface %u, presented as %ux%u\n",
	    fWidth, fHeight, fRowBytes, fPhys, fSurface->id, fLogicalW, fLogicalH);
	/* 0340: the frame queue's thread (asleep until a frame is queued), and its boot-args */
	PE_parse_boot_argn("sweetfb_queue", &sweet_fb_queue, sizeof(sweet_fb_queue));
	PE_parse_boot_argn("sweetfb_queue_hold", &sweet_fb_queue_hold_ms, sizeof(sweet_fb_queue_hold_ms));
	if (fFlip[0] != NULL && fPresentLock != NULL) {
		thread_t thread;
		if (kernel_thread_start(&AppleCLCD::queueThreadMain, this, &thread) == KERN_SUCCESS) {
			thread_deallocate(thread);
			fQThread = true;
		}
		IOLog("SWEET FB: frame queue %s (kern.sweet_fb_queue), %u flip buffers, its thread %s\n", sweet_fb_queue ? "on" : "off",
		    fFlip[2] != NULL ? 3u : 2u, fQThread ? "started" : "NOT started (the timer's ticks flip)");
	}
	scanoutInit();                  /* 0350 */
	return true;
}

/*
 * 0350: whether the plane is as the scanout expects it, and the pool. The plane's registers are the bootloader's: this
 * kernel has only pointed it elsewhere (SRC0_ADDR, SRC_YSTRIDE0) so far. The scanout needs iOS drawing at 2x (a surface
 * smaller than the panel, the copy's case), the copy's way to fall back on (two flip buffers, the queue's thread, the
 * interface's counters), a plane that shows the whole panel unscaled from (0, 0) -- source and output both the panel's
 * size, the scaler off -- out of linear 32-bit RGB, and VIG0 in the mixer.
 */
void
AppleCLCD::scanoutInit(void)
{
	PE_parse_boot_argn("sweetfb_scanout", &sweet_fb_scanout, sizeof(sweet_fb_scanout));
	PE_parse_boot_argn("sweetfb_ubwc", &sweet_fb_ubwc, sizeof(sweet_fb_ubwc));
	PE_parse_boot_argn("sweetfb_flush_taken", &sweet_fb_flush_taken, sizeof(sweet_fb_flush_taken));     /* 0352 */
	PE_parse_boot_argn("sweetfb_swapwait", &sweet_fb_swapwait, sizeof(sweet_fb_swapwait));             /* 0353 */
	if (sweet_fb_ubwc != 0 && (sweet_fb_ubwc != 1 || sweet_fb_scanout != 1 || sapphire_gpu_ubwc_mode() != 1)) {
		sweet_fb_ubwc = 0;
		fScanWhy = "UBWC requires sweetfb_scanout=1 and kern.sweet_gpu_ubwc=1";
	}
	if (fVig0 == NULL || fCtl0 == NULL) {
		fScanWhy = "the plane is not mapped";
		return;
	}
	for (unsigned k = 0; k < SWEET_PLANE_REGS; k++) {
		fPlane0[k] = fVig0[sweet_plane_off[k] / 4];
	}
	fPlaneFormat = fVig0[0x30 / 4];
	fScalerVersion = fVig0[0xa00 / 4];
	uint32_t panel = (fHeight << 16) | fWidth, layer = fCtl0[0x00 / 4];
	if (!fScale2 || fLogicalW >= fWidth || fLogicalH >= fHeight) {
		fScanWhy = "iOS does not draw at 2x (sweetfb_scale=2): its surface is not smaller than the panel";
	} else if (fFlip[0] == NULL || fFlip[1] == NULL || !fQThread || fIntf1 == NULL) {
		fScanWhy = "no flip buffers, frame queue thread or interface counters to fall back on";
	} else if (fPlane0[0] != panel || fPlane0[2] != panel || fPlane0[1] != 0 || fPlane0[3] != 0) {
		fScanWhy = "the plane does not show the whole panel unscaled from (0, 0)";
	} else if ((fPlane0[25] & 1) != 0 || fScalerVersion == 0 || fScalerVersion == ~0u) {
		fScanWhy = "the scaler is on already, or does not answer";
	} else if (((fPlaneFormat >> 9) & 3) != 3 || (fPlaneFormat >> 30) != 0 || (fPlaneFormat & (1u << 15)) != 0 ||
	    (fPlane0[26] & 1) != 0) {
		fScanWhy = "the plane's format is not linear 32-bit RGB";
	} else if ((layer & 7) == 0) {
		fScanWhy = "VIG0 is not in the mixer (CTL_LAYER(LM0))";
	} else {
		/* 0398: UBWC is valid only on this direct-scanout path. The copy fallback remains linear because
		 * scanoutOn() selects it whenever a frame is not a pool surface; it never enters planeScaled(). */
		if (sweet_fb_ubwc != 0 && sweet_fbr_va[0] != NULL) {
			uint32_t staticUbwc = sweet_fbr_va[0][0x144 / 4];
			if (staticUbwc != 0x11fu) {
				sweet_fbr_va[0][0x144 / 4] = 0x11fu;
				__asm__ volatile("dsb sy" ::: "memory");
			}
		}
		sweet_scanout_pool_set(SWEET_FB_SCAN_PA, SWEET_FB_SCAN_SLOTS, SWEET_FB_SCAN_SLOT, fLogicalW, fLogicalH);
		sweet_fb_scan_ready = true;
	}
	IOLog("SWEET FB: scanout (kern.sweet_fb_scanout %d): %s; plane at boot: src %#x/%#x out %#x/%#x format %#x op %#x pe %#x, "
	    "scaler %#x op %#x, CTL_LAYER %#x; pool %u x %u KB at %#llx\n", sweet_fb_scanout,
	    fScanWhy != NULL ? fScanWhy : "ready", fPlane0[0], fPlane0[1], fPlane0[2], fPlane0[3], fPlaneFormat, fPlane0[26],
	    fPlane0[6], fScalerVersion, fPlane0[25], layer, (uint32_t)SWEET_FB_SCAN_SLOTS, SWEET_FB_SCAN_SLOT >> 10, SWEET_FB_SCAN_PA);
}

/* SweetSurface.cpp asks: are the window server's screen surfaces to be made in the pool now? */
int
sweet_fb_scanout_pool_on(void)
{
	return sweet_fb_scan_ready && (sweet_fb_scanout == 1 || sweet_fb_scanout == 2);
}

/*
 * The display's modes, the way CoreAnimation reads them (properties TimingElements and
 * ColorElements; the current one is block 0x40 of selector 79): one timing, the presented size
 * at 60 Hz, one colour mode, 8-bit RGB. Without them CoreAnimation ran its frame clock at a rate
 * of its own guessing: 20 frames a second, 50 ms apart. (ios-topaz 0948, after HackiOS
 * MiniIOMobileFramebuffer publishModes(), APSL 2.0.)
 */
#define SWEET_FB_TIMING_ID      1
#define SWEET_FB_COLOR_ID       1

static void
sweet_set_number(OSDictionary *d, const char *key, uint64_t value)
{
	OSNumber *n = OSNumber::withNumber(value, 32);
	if (n != NULL) {
		d->setObject(key, n);
		n->release();
	}
}

void
AppleCLCD::publishModes(void)
{
	OSDictionary *timing = OSDictionary::withCapacity(8);
	OSDictionary *horizontal = OSDictionary::withCapacity(4);
	OSDictionary *vertical = OSDictionary::withCapacity(4);
	OSDictionary *color = OSDictionary::withCapacity(8);
	OSArray *timings = OSArray::withCapacity(1);
	OSArray *colors = OSArray::withCapacity(1);
	const uint32_t rate = fRateHz << 16;    /* 16.16 */

	if (timing != NULL && horizontal != NULL && vertical != NULL && color != NULL && timings != NULL && colors != NULL) {
		sweet_set_number(color, "ID", SWEET_FB_COLOR_ID);
		sweet_set_number(color, "PixelEncoding", 0);
		sweet_set_number(color, "Depth", 8);
		sweet_set_number(color, "Colorimetry", 1);
		sweet_set_number(color, "EOTF", 1);
		sweet_set_number(color, "DynamicRange", 0);
		sweet_set_number(color, "Score", 1);
		color->setObject("IsVirtual", kOSBooleanFalse);
		colors->setObject(color);
		sweet_set_number(horizontal, "Active", fLogicalW);
		sweet_set_number(horizontal, "Total", fLogicalW);
		sweet_set_number(horizontal, "SyncRate", rate);
		sweet_set_number(horizontal, "PreciseSyncRate", rate);
		sweet_set_number(vertical, "Active", fLogicalH);
		sweet_set_number(vertical, "Total", fLogicalH);
		sweet_set_number(vertical, "SyncRate", rate);
		sweet_set_number(vertical, "PreciseSyncRate", rate);
		sweet_set_number(timing, "ID", SWEET_FB_TIMING_ID);
		sweet_set_number(timing, "Score", 1);
		timing->setObject("IsVirtual", kOSBooleanFalse);
		timing->setObject("IsOverscanned", kOSBooleanFalse);
		timing->setObject("HorizontalAttributes", horizontal);
		timing->setObject("VerticalAttributes", vertical);
		timing->setObject("ColorModes", colors);
		timings->setObject(timing);
		setProperty("TimingElements", timings);
		setProperty("ColorElements", colors);
		IOLog("SWEET FB: modes published: %ux%u at %u Hz\n", fLogicalW, fLogicalH, fRateHz);
	}
	/*
	 * sapphire 0301: the refresh the window server clocks its frames by, IOMFBDisplayRefresh: the
	 * shortest interval between refreshes, the longest and the step (QuartzCore's
	 * displayMinRefreshInterval / displayRefreshStepMachTime, CoreMedia's displayMaxRefreshInterval /
	 * displayRefreshStep). Without it CoreAnimation takes a built-in panel for a 60 Hz one whatever its
	 * timings say ("internal_panel (fixed)" at 60, UIScreen's maximumFramesPerSecond 60). The intervals
	 * are in 2^-32 s; only the step named MachTime is in mach_absolute_time ticks. One fixed rate here,
	 * sweetfb_hz's. (After HackiOS MiniIOMobileFramebuffer publishRefresh(), b6c1f82 + 0dedaec, APSL 2.0.)
	 */
	OSDictionary *refresh = OSDictionary::withCapacity(4);
	if (refresh != NULL) {
		uint64_t interval = (1ULL << 32) / fRateHz, ticks = 0;
		nanoseconds_to_absolutetime(NSEC_PER_SEC / fRateHz, &ticks);
		const char *keys[] = { "displayMinRefreshInterval", "displayMaxRefreshInterval", "displayRefreshStep" };
		for (unsigned i = 0; i < 3; i++) {
			OSNumber *n = OSNumber::withNumber(interval, 64);
			if (n != NULL) {
				refresh->setObject(keys[i], n);
				n->release();
			}
		}
		OSNumber *t = OSNumber::withNumber(ticks, 64);
		if (t != NULL) {
			refresh->setObject("displayRefreshStepMachTime", t);
			t->release();
		}
		setProperty("IOMFBDisplayRefresh", refresh);
		refresh->release();
		IOLog("SWEET FB: IOMFBDisplayRefresh: %u Hz, an interval of %llu (2^-32 s), %llu ticks\n",
		    fRateHz, interval, ticks);
	}
	OSSafeReleaseNULL(timings);
	OSSafeReleaseNULL(colors);
	OSSafeReleaseNULL(timing);
	OSSafeReleaseNULL(horizontal);
	OSSafeReleaseNULL(vertical);
	OSSafeReleaseNULL(color);
}

void
AppleCLCD::vsyncFired(OSObject *owner, IOTimerEventSource *sender)
{
	AppleCLCD *fb = (AppleCLCD *)owner;
	uint64_t now = mach_absolute_time();
	uint64_t count = ++fb->fVsyncCount;
	if (fb->fNextVsync != 0 && now >= fb->fNextVsync) {
		/* 0336: how long after its place on the grid this tick runs */
		sweet_fbd_add(&sweet_fbd.tick_late, sweet_fbd_late_us, sizeof(sweet_fbd_late_us) / sizeof(sweet_fbd_late_us[0]),
		    sweet_fbd_us(fb->fNextVsync, now));
	}
	uint64_t values[6] = {
		(uint32_t)count, (uint32_t)(count >> 32),
		(uint32_t)now, (uint32_t)(now >> 32),
		0, 0,
	};

	IOLockLock(fb->fLock);
	for (unsigned int i = 0; i < fb->fClients->getCount(); i++) {
		IOMobileFramebufferUserClient *client =
		    (IOMobileFramebufferUserClient *)fb->fClients->getObject(i);
		client->notify(SWEET_FB_NOTIFY_VSYNC, values, 6);
	}
	if (fb->fIntf1 != NULL && (count % (5 * fb->fRateHz)) == 0) {
		uint32_t frames = fb->fIntf1[(0x800 + 0x0ac) / 4];
		IOLog("SWEET INTF1: %u frames in 5 s (line %u, te_en %#x)\n",
		    frames - fb->fIntfFrames, fb->fIntf1[(0x800 + 0x0b0) / 4], fb->fIntf1[0x800 / 4]);
		fb->fIntfFrames = frames;
	}
	if (fb->fPowerOff) {
		/*
		 * topaz: fade in cached memory and flip whole frames to the MDP.
		 * Fading the uncached boot framebuffer in place crawled down the panel.
		 */
		if (fb->fDsi != NULL) {
			/*
			 * 0209: the panel's display off (DCS 0x28) does not always get through ("NO DONE"), and
			 * the last frame stayed on the screen until the power button again. So the frame the
			 * panel scans is also made black, once (on AMOLED black is off); at "on" the latest
			 * frame is presented again (fDirty).
			 */
			/*
			 * 0211/0212: turned off in the middle of an animation, the GPU is busy and the copy of the
			 * last frame queued before "off" lands later, over the black (0209 blackened twice: the
			 * screen stayed on; 0211 again at ticks 1-30: it blinked two or three times). Now the black
			 * waits for the GPU to have done the last copy (no new one is queued while off), up to half
			 * a second, and is made once, then once more a tick later.
			 */
			uint32_t step = fb->fFadeStep;
			bool pending = sapphire_gpu_copy_pending();
			bool first = false;
			if (step < 30 && pending) {
				fb->fFadeStep = step + 1;       /* waiting for the copy (the frame stays a moment) */
			} else if (step < 32) {
				first = true;
				fb->fFadeStep = step < 30 ? 30 : step + 1;
			}
			IOLockUnlock(fb->fLock);
			if (first && fb->fPixels != 0) {
				if (fb->fVig0 != NULL && fb->fScanPhys != fb->fBootSrc) {
					/* (0350: with fPresentLock, as every flip that may leave a scaled plane or a surface on it) */
					if (fb->fPresentLock != NULL) {
						IOLockLock(fb->fPresentLock);
					}
					fb->scanFrom(fb->fBootSrc, fb->fBootStride);
					if (fb->fPresentLock != NULL) {
						IOLockUnlock(fb->fPresentLock);
					}
				}
				for (uint32_t y = 0; y < fb->fHeight; y++) {
					volatile uint64_t *row = (volatile uint64_t *)(fb->fPixels + (uint64_t)y * fb->fRowBytes);
					for (uint32_t x = 0; x < fb->fWidth / 2; x++) {
						row[x] = 0;
					}
				}
				__asm__ volatile("dsb sy" ::: "memory");
			}
			fb->armVsync(sender);
			return;
		}
		if (fb->fFadeOrig != NULL) {
			uint32_t step = fb->fFadeStep;
			if (step <= SWEET_FB_FADE_STEPS) {
				fb->fFadeStep = step + 1;
			}
			bool front = fb->fFlip[0] != NULL && fb->fScanPhys == fb->fFlipPhys[fb->fFlipIdx];
			const uint8_t *src = front ? (const uint8_t *)fb->fFlip[fb->fFlipIdx]->getBytesNoCopy() :
			    fb->fLastVA ? fb->fLastVA :
			    fb->fConsShown ?
			    (const uint8_t *)fb->fCons->getBytesNoCopy() : (const uint8_t *)fb->fPixels;
			uint32_t srcStride = (!front && fb->fLastVA) ? fb->fLastStride : fb->fRowBytes;
			IOLockUnlock(fb->fLock);
			if (step <= SWEET_FB_FADE_STEPS) {
				fb->fadeStep(src, srcStride, step);
			}
			fb->armVsync(sender);
			return;
		}
		if (fb->fVig0 != NULL && fb->fScanPhys != fb->fBootSrc) {
			if (fb->fPresentLock != NULL) {
				IOLockLock(fb->fPresentLock);   /* (0350) */
			}
			fb->scanFrom(fb->fBootSrc, fb->fBootStride);
			if (fb->fPresentLock != NULL) {
				IOLockUnlock(fb->fPresentLock);
			}
		}
		/* Fade out: halve every channel each vsync, then black, then idle. */
		uint32_t step = fb->fFadeStep;
		if (step <= 6) {
			fb->fFadeStep = step + 1;
		}
		IOLockUnlock(fb->fLock);
		if (step <= 6) {
			for (uint32_t y = 0; y < fb->fHeight; y++) {
				volatile uint32_t *row = (volatile uint32_t *)(fb->fPixels + (uint64_t)y * fb->fRowBytes);
				for (uint32_t x = 0; x < fb->fWidth; x++) {
					row[x] = step < 6 ? (row[x] >> 1) & 0x7f7f7f7f : 0;
				}
			}
		}
		fb->armVsync(sender);
		return;
	}
	if (fb->fWakeTicks != 0 && --fb->fWakeTicks == 0) {
		fb->fDirty = true;              /* 0295: no new frame since "on": the latest one after all */
	}
	bool flush = fb->fDirty && (count % fb->fFlushTicks) == 0;
	uint8_t *rec = NULL;
	SweetSurface *taken[4] = {};
	if (flush) {
		rec = (uint8_t *)IOMallocData(SWEET_FB_SWAP_REC_SIZE);
		if (rec != NULL) {
			memcpy(rec, fb->fPending, SWEET_FB_SWAP_REC_SIZE);
			fb->fDirty = false;
			memcpy(taken, fb->fPendHold, sizeof(taken));
			bzero(fb->fPendHold, sizeof(fb->fPendHold));
		}
	}
	/* frames replaced at the previous vsync have been latched by now */
	SweetSurface *retire[4];
	memcpy(retire, fb->fRetireHold, sizeof(retire));
	bzero(fb->fRetireHold, sizeof(fb->fRetireHold));
	IOLockUnlock(fb->fLock);
	sweet_hold_set(retire, false);
	fb->presentHeld(true);
	if (fb->fPresentLock != NULL) {
		/* 0235: a back buffer whose GPU copy has landed since the last swap goes on the panel now */
		IOLockLock(fb->fPresentLock);
		uint64_t flips_before = sweet_fbd.flips;
		fb->queueFlip();                /* 0340: the frame queue's turn (nothing to do when it is empty) */
		(void)fb->flipWhenCopied(false);
		if (sweet_fbd.flips != flips_before) {
			sweet_fbd.flips_by_tick++;      /* 0336: this tick found the copy done */
		}
		IOLockUnlock(fb->fPresentLock);
	}
	if (fb->fConsShown && fb->fFlip[0] != NULL && fb->flipWhenCopied(true) &&
	    sweet_console_copy_dirty(fb->backBuffer())) {
		fb->flipBack();
	}
	if (rec != NULL && fb->queueOn()) {
		/* 0340: the timer's own frames (the latest one after "on", a swap recorded while off) go the queue's way */
		fb->presentNow(rec, taken);
		IOFreeData(rec, SWEET_FB_SWAP_REC_SIZE);
	} else if (rec != NULL) {
		if (fb->fPresentLock != NULL) {
			IOLockLock(fb->fPresentLock);
			fb->queueDrain();               /* 0340: (the queue was just turned off: what it still holds goes first) */
		}
		fb->present(rec);
		if (fb->fPresentLock != NULL) {
			IOLockUnlock(fb->fPresentLock);
		}
		IOFreeData(rec, SWEET_FB_SWAP_REC_SIZE);
		IOLockLock(fb->fLock);
		memcpy(fb->fRetireHold, fb->fShowHold, sizeof(taken));
		memcpy(fb->fShowHold, taken, sizeof(taken));
		IOLockUnlock(fb->fLock);
	}
	fb->armVsync(sender);
}

/*
 * 0201: the vsync timer re-armed 16667 us after it had run (and after the frame it presented):
 * every period was that much longer -- 57 Hz measured against the panel's frame counter. Now the
 * ticks sit on an absolute 60.000 Hz grid; after a stall longer than a period the grid restarts.
 */
void
AppleCLCD::armVsync(IOTimerEventSource *t)
{
	uint64_t period, now = mach_absolute_time();
	nanoseconds_to_absolutetime(1000000000ull / (fRateHz ? fRateHz : 60), &period);
	if (fNextVsync == 0 || now > fNextVsync + period) {
		if (fNextVsync != 0) {
			sweet_fbd.grid_restarts++;      /* 0336 */
		}
		fNextVsync = now;
	}
	fNextVsync += period;
	t->wakeAtTime(fNextVsync);
}

void
AppleCLCD::addClient(IOMobileFramebufferUserClient *client)
{
	IOLockLock(fLock);
	fClients->setObject(client);
	IOLockUnlock(fLock);
}

void
AppleCLCD::removeClient(IOMobileFramebufferUserClient *client)
{
	IOLockLock(fLock);
	unsigned int i = fClients->getNextIndexOfObject(client, 0);
	if (i != (unsigned int)-1) {
		fClients->removeObject(i);
	}
	IOLockUnlock(fLock);
}

static uint32_t
get32(const uint8_t *p, uint32_t off)
{
	uint32_t v;
	memcpy(&v, p + off, 4);
	return v;
}

static uint32_t sweet_fb_swaps_logged;
static uint64_t sweet_fb_swap_count;

/* Copy each enabled layer's surface into the panel memory (no scaling). */
/*
 * A swap only records the frame: copying 10 MB into the panel on every
 * swap (backboardd presents at 60 Hz) took the whole CPU. The vsync timer
 * shows the latest frame every fFlushTicks ticks (boot-arg sweetfb_hz,
 * default 10 Hz).
 */
void
AppleCLCD::swap(const uint8_t *rec)
{
	SweetSurface *held[4] = {};
	uint32_t mask = get32(rec, 0x104);
	for (uint32_t i = 0; i < 4 && i < SWEET_FB_LAYERS; i++) {
		uint32_t sid = get32(rec, 0x54 + 4 * i);
		if ((mask & (1u << i)) && sid != 0 && (held[i] = sweet_surface_copy(sid)) != NULL) {
			sweet_hold(held[i], true);
		}
	}
	if (held[0] != NULL || held[1] != NULL || held[2] != NULL || held[3] != NULL) {
		/* 0336: SwapEnd to SwapEnd, of the swaps that name a surface */
		uint64_t t = mach_absolute_time(), last = sweet_fbd.last_swap;
		sweet_fbd.swaps++;
		if (last != 0 && t > last) {
			sweet_fbd_add(&sweet_fbd.swap_iv, sweet_fbd_iv_us, sizeof(sweet_fbd_iv_us) / sizeof(sweet_fbd_iv_us[0]),
			    sweet_fbd_us(last, t));
		}
		sweet_fbd.last_swap = t;
	}
	/*
	 * 0201: presented here, at SwapEnd, rather than at the next tick of the vsync timer: frames that
	 * came two to a tick were dropped and every frame waited up to a tick (the panel showed 25-30 of
	 * the 60 a second backboardd drew). A frame the GPU copies into the panel's memory leaves its
	 * surface at once -- the GPU runs one queue in order, so whatever CoreAnimation draws into that
	 * surface next is drawn after the copy -- instead of 2-3 ticks later: CoreAnimation has two or
	 * three surfaces and waited for a free one. A frame the MDP scans directly keeps its surface
	 * until the next one has replaced it on the panel, as before.
	 */
	IOLockLock(fLock);
	bool now = fNow && !fPowerOff && fFadeOrig == NULL;
	if (!now && !fPowerOff && queueOn()) {
		/* 0340: the frame queue presents in SwapEnd, flip buffers or not; the swap stays the latest one for "on" */
		memcpy(fPending, rec, SWEET_FB_SWAP_REC_SIZE);
		now = true;
	}
	IOLockUnlock(fLock);
	if (now) {
		/*
		 * 0286: backboardd's GL bridge no longer waits in glFlush for its frame to reach the
		 * GPU (a quarter of its frame time): it numbers the frame first
		 * (kern.sweet_fb_frame_await) and reports it sent from Mesa's driver thread
		 * (kern.sweet_fb_frame_sent). A swap that comes in between is that frame's
		 * (CoreAnimation: flush, swap, the next frame's flush) and is held until then --
		 * presented now, the GPU's copy into the panel would go ahead of the frame in the
		 * GPU's queue and show what the surface held before. A held swap goes out anyway when
		 * the next swap comes (frames stay in order) or after 3 vsync ticks.
		 */
		presentHeld(false);
		IOLockLock(fLock);
		if (fAwaitSid != 0) {
			if (!fAwaitSent && fHeldRec == NULL &&
			    (fHeldRec = (uint8_t *)IOMallocData(SWEET_FB_SWAP_REC_SIZE)) != NULL) {
				memcpy(fHeldRec, rec, SWEET_FB_SWAP_REC_SIZE);
				memcpy(fHeldHold, held, sizeof(fHeldHold));
				fHeldSeq = fAwaitSid;
				fHeldTicks = 0;
				fHeldCount++;
				IOLockUnlock(fLock);
				return;
			}
			fAwaitSid = 0;
			fAwaitSent = false;
		}
		IOLockUnlock(fLock);
		if (sweet_fb_hold_ms > 0 && fFlipWait && sapphire_gpu_copy_pending()) {
			/* 0338: the copy of the frame before lands (a tick or present() then flips it) before this one is queued */
			uint64_t hold_t0 = mach_absolute_time(), deadline;
			clock_interval_to_deadline((uint32_t)(sweet_fb_hold_ms > 200 ? 200 : sweet_fb_hold_ms), kMillisecondScale,
			    &deadline);
			while (fFlipWait && sapphire_gpu_copy_pending() && mach_absolute_time() < deadline) {
				IODelay(200);
			}
			sweet_fbd.holds++;
			sweet_fbd.hold_us += sweet_fbd_us(hold_t0, mach_absolute_time());
		}
		presentNow(rec, held);
		swapEndHold(get32(rec, 0x50));  /* 0353: (SwapEnd's own thread, never the timer's) */
		return;
	}
	IOLockLock(fLock);
	memcpy(fPending, rec, SWEET_FB_SWAP_REC_SIZE);
	fDirty = true;
	SweetSurface *dropped[4];
	memcpy(dropped, fPendHold, sizeof(dropped));      /* replaced before shown */
	memcpy(fPendHold, held, sizeof(held));
	IOLockUnlock(fLock);
	sweet_hold_set(dropped, false);
	if ((++sweet_fb_swap_count % 1000) == 0) {
		IOLog("SWEET FB: %llu swaps\n", sweet_fb_swap_count);
	}
}

/* SwapEnd's frame on the panel now, and the surfaces it holds and frees as before (0201) */
void
AppleCLCD::presentNow(const uint8_t *rec, SweetSurface **held)
{
	uint64_t diag_t0 = mach_absolute_time();        /* 0336: what a SwapEnd costs its caller here, lock included */
	IOLockLock(fPresentLock);
	uint64_t flips_before = sweet_fbd.flips;
	bool queued = false, copied;
	if (queueOn()) {
		fQArrived = diag_t0;
		for (uint32_t i = 0; held != NULL && i < SWEET_FB_LAYERS; i++) {
			if (held[i] != NULL) {
				held[i]->swaps++;               /* 0354: (the pool's census) */
				held[i]->swapAbs = diag_t0;
			}
		}
		if (sweet_fb_scanout == 1 && sweet_fb_swapwait == 0 && sweet_fb_scan_ready) {
			sweet_fbs.swapwait_off++;       /* 0354: the scanout needs SwapWait answered (kern.sweet_fb_swapwait) */
		}
		/* 0354: a 2x frame goes by the record shownRecord() gives, or is not shown (see there) */
		uint8_t *one = NULL;
		const uint8_t *r = fScale2 ? shownRecord(rec, &one) : rec;
		if (r == NULL) {
			sweet_fbs.not_shown++;
			copied = true;                  /* (its swap is done for SwapWait; its surfaces are free) */
		} else if (scanoutOn() && presentScanout(r, held, &queued)) {
			copied = true;                  /* 0350: scanned out as it is; queued, it keeps its surfaces */
		} else {
			copied = presentQueued(r, held, &queued);       /* 0340: queued, the frame keeps its surfaces until its flip */
		}
		if (one != NULL) {
			IOFreeData(one, SWEET_FB_SWAP_REC_SIZE);
		}
	} else {
		if (sweet_fb_scanout == 1 && sweet_fb_scan_ready) {
			sweet_fbs.queue_off++;          /* 0350: the scanout needs the frame queue (kern.sweet_fb_queue=1) */
		}
		queueDrain();                           /* (the queue was just turned off: what it still holds goes first) */
		copied = present(rec);
	}
	if (sweet_fbd.flips != flips_before) {
		sweet_fbd.flips_by_swap++;              /* this swap found the copy before it done */
	}
	IOLockUnlock(fPresentLock);
	{
		uint64_t us = sweet_fbd_us(diag_t0, mach_absolute_time());
		sweet_fbd.presents++;
		sweet_fbd.present_us += us;
		if (us > sweet_fbd.present_max_us) {
			sweet_fbd.present_max_us = us;
		}
	}
	SweetSurface *release[4] = {};
	IOLockLock(fLock);
	fDirty = false;
	fWakeTicks = 0;
	SweetSurface *pend[4];
	memcpy(pend, fPendHold, sizeof(pend));         /* (a frame of the timer's path, now replaced) */
	bzero(fPendHold, sizeof(fPendHold));
	if (queued) {
		/* 0340: (the queue lets go of them) */
	} else if (copied) {
		memcpy(release, held, sizeof(release));
	} else {
		memcpy(release, fRetireHold, sizeof(release));
		memcpy(fRetireHold, fShowHold, sizeof(fRetireHold));
		memcpy(fShowHold, held, sizeof(fShowHold));
	}
	IOLockUnlock(fLock);
	sweet_hold_set(pend, false);
	sweet_hold_set(release, false);
	if ((++sweet_fb_swap_count % 1000) == 0) {
		IOLog("SWEET FB: %llu swaps; 0286: %llu held, %llu sent, %llu late, %llu forced\n", sweet_fb_swap_count,
		    fHeldCount, fSentCount, fHeldLate, fHeldForced);
	}
}

/* 0286: the held swap out: its frame was sent, or the next swap came (late false), or a vsync tick (late true) */
void
AppleCLCD::presentHeld(bool late)
{
	SweetSurface *hold[4];
	uint8_t *rec;

	IOLockLock(fLock);
	rec = fHeldRec;
	if (rec == NULL || (late && ++fHeldTicks < 3)) {
		IOLockUnlock(fLock);
		return;
	}
	fHeldRec = NULL;
	memcpy(hold, fHeldHold, sizeof(hold));
	bzero(fHeldHold, sizeof(fHeldHold));
	if (fAwaitSid == fHeldSeq) {
		fAwaitSid = 0;
		fAwaitSent = false;
	}
	if (late) {
		fHeldLate++;
	}
	IOLockUnlock(fLock);
	presentNow(rec, hold);
	IOFreeData(rec, SWEET_FB_SWAP_REC_SIZE);
}

void
AppleCLCD::scanFrom(uint64_t pa, uint32_t stride)
{
	/*
	 * 0340: the MDP goes on reading what it reads until it takes this flip at its vsync. Which flip buffer that
	 * is, and where the panel's scan was when the flip was written, are kept for the frame queue: the buffer is
	 * not written and no other flip is made before the flip is seen taken (flipTaken()). If the flip before this
	 * one has not been seen taken, the MDP may be reading either the buffer noted then or that flip's own.
	 * 0350: a flip buffer is shown unscaled, the plane as the bootloader set it up: a scaled plane (a surface on the
	 * panel) is set back first, where it cannot be latched half written; that surface is read until this flip is taken.
	 */
	bool geometry = fPlaneScaled;
	planeUnscaled();                /* 0352: (waits for a writable plane first: the flip is noted after, when it is written) */
	flipNote(pa, geometry);
	scanoutLeave();
	fVig0[0x14 / 4] = (uint32_t)pa;
	fVig0[0x24 / 4] = stride;
	__asm__ volatile("dsb sy" ::: "memory");
	fCtl0[0x18 / 4] = 0x20001;      /* flush VIG0 + CTL: latched at the next vsync */
	__asm__ volatile("dsb sy" ::: "memory");
	flushNote();                    /* 0352 */
	sweet_fbl_wrote(fCtl0, fIntf1); /* 0349: what CTL_FLUSH reads back, and when it clears (read only) */
	fScanPhys = pa;
	fScanStride = stride;
}

/* 0352: what CTL_FLUSH reads right after a flip wrote it (sweet_fb_flush_taken, at the top). */
void
AppleCLCD::flushNote(void)
{
	if (fCtl0 == NULL) {
		return;
	}
	if ((fCtl0[0x18 / 4] & 0x20001) != 0) {
		fFlushSet++;
	} else {
		fFlushClear++;
	}
	fFlushKnown = fFlushSet + fFlushClear >= 16;
	fFlushReadback = fFlushKnown && (uint64_t)fFlushClear * 64 <= (uint64_t)fFlushSet + fFlushClear;
}

/*
 * 0350 (0340's bookkeeping of a flip, from scanFrom(), shared with planeScaled()): a flip to pa is about to be written.
 * Which flip buffer the MDP is leaving, and where the scan is, for flipTaken(); force: a flip that rewrites the plane
 * counts as one even to the same address.
 */
void
AppleCLCD::flipNote(uint64_t pa, bool force)
{
	uint64_t now = mach_absolute_time();
	bool taken = flipTaken(now);
	uint64_t leaving = 0;
	for (unsigned i = 0; i < SWEET_FB_BUFS; i++) {
		if (fFlip[i] != NULL && fFlipPhys[i] == fScanPhys) {
			leaving = fFlipPhys[i];
		}
	}
	if (pa != fScanPhys || force) {
		if (taken) {
			fQRetirePa[0] = leaving;
			fQRetirePa[1] = 0;
		} else if (leaving != fQRetirePa[0]) {
			fQRetirePa[1] = leaving;
		}
		fQFlipLate = true;
		if (fIntf1 != NULL) {
			uint32_t hsync = fIntf1[(0x800 + 0x008) / 4] >> 16;
			uint32_t vtotal = hsync != 0 ? fIntf1[(0x800 + 0x00c) / 4] / hsync : 0;
			fQFlipFrame = fIntf1[(0x800 + 0x0ac) / 4];
			fQFlipLate = vtotal == 0 || (uint64_t)fIntf1[(0x800 + 0x0b0) / 4] * 16 >= (uint64_t)vtotal * 15;
		}
		fQFlipAbs = now;
		fQLatchWait = true;
		fFlipSwap = fSwapShow;          /* 0353: the swap this flip shows (0: none of a swap) */
	}
	fSwapShow = 0;
}

/*
 * DCS long write through the DSI command DMA path (topaz-woa: the TPG FIFO
 * path is a no-op on this DSI 2.4.1). The packet lives in a contiguous
 * buffer below 4 GiB; header = word count, data type 0x39, flags last|long.
 */
static inline uint32_t dsi_rd(volatile uint8_t *b, uint32_t o) { return *(volatile uint32_t *)(b + o); }
static inline void dsi_wr(volatile uint8_t *b, uint32_t o, uint32_t v)
{
	__asm__ volatile ("dsb sy" ::: "memory");
	*(volatile uint32_t *)(b + o) = v;
	__asm__ volatile ("dsb sy" ::: "memory");
}

bool
AppleCLCD::dsiDcsLong(const uint8_t *payload, uint32_t len)
{
	if (fDsi == NULL || fDsiBuf == NULL || len > 60) {
		return false;
	}
	uint8_t hdr[4] = { (uint8_t)len, (uint8_t)(len >> 8), 0x39, 0xc0 };   /* long write, last|long */
	return dsiSend(hdr, payload, len);
}

bool
AppleCLCD::dsiSend(const uint8_t hdr[4], const uint8_t *payload, uint32_t len)
{
	if (fDsi == NULL || fDsiBuf == NULL || len > 60) {
		return false;
	}
	uint8_t *p = (uint8_t *)fDsiBuf->getBytesNoCopy();
	uint32_t size = (4 + len + 3) & ~3u;
	memset(p, 0xff, 64);
	memcpy(p, hdr, 4);
	if (len) {
		memcpy(p + 4, payload, len);
	}
	sweet_dcache_clean((vm_offset_t)(uintptr_t)p, 64);
	uint32_t ctrl = dsi_rd(fDsi, 0x004), trig = dsi_rd(fDsi, 0x084);
	if ((trig & 7) != 4) {
		dsi_wr(fDsi, 0x084, (trig & ~7u) | 4);  /* DMA trigger: software */
	}
	dsi_wr(fDsi, 0x004, ctrl | 4 | 1);              /* CMD_MODE_EN | DSI_EN */
	dsi_wr(fDsi, 0x110, dsi_rd(fDsi, 0x110) | 1);   /* clear CMD_DMA_DONE */
	dsi_wr(fDsi, 0x15c, 0);                          /* no test pattern */
	dsi_wr(fDsi, 0x048, (uint32_t)fDsiBufPhys);
	dsi_wr(fDsi, 0x04c, size);
	uint32_t dma = dsi_rd(fDsi, 0x03c);
	dsi_wr(fDsi, 0x03c, dma & ~(1u << 26));          /* high speed */
	dsi_wr(fDsi, 0x090, 1);                          /* SW trigger */
	bool done = false;
	for (int i = 0; i < 100 && !done; i++) {
		done = (dsi_rd(fDsi, 0x110) & 1) != 0;
		if (!done) {
			IODelay(200);
		}
	}
	dsi_wr(fDsi, 0x110, dsi_rd(fDsi, 0x110) | 1);
	dsi_wr(fDsi, 0x03c, dma);
	dsi_wr(fDsi, 0x004, ctrl);
	if (trig != dsi_rd(fDsi, 0x084)) {
		dsi_wr(fDsi, 0x084, trig);
	}
	return done;
}

/* DCS 0x28 display off / 0x29 display on (short write, no parameter) */
void
AppleCLCD::setPanelOn(bool on)
{
	uint8_t hdr[4] = { (uint8_t)(on ? 0x29 : 0x28), 0, 0x05, 0x80 };
	IOLockLock(fBlLock);
	bool done = false;
	for (int i = 0; i < 4 && !done; i++) {         /* 0209: "NO DONE" now and then (a frame on the link) */
		done = dsiSend(hdr, NULL, 0);
		if (!done) {
			IODelay(2000);
		}
	}
	IOLockUnlock(fBlLock);
	IOLog("SWEET FB: panel display %s: %s\n", on ? "on" : "off", done ? "done" : "NO DONE");
}

void
AppleCLCD::setPanelLevel(uint32_t level)
{
	uint8_t cmd[3] = { 0x51, (uint8_t)(level >> 8), (uint8_t)level };   /* MSB first */
	IOLockLock(fBlLock);
	bool done = false;
	for (int i = 0; i < 4 && !done; i++) {
		done = dsiDcsLong(cmd, sizeof(cmd));
		if (!done) {
			IODelay(2000);
		}
	}
	IOLockUnlock(fBlLock);
	static uint32_t logs;
	if (logs++ < 8 || !done) {
		IOLog("SWEET FB: panel level %#x: %s\n", level, done ? "done" : "NO DONE");
	}
}

/*
 * CoreBrightness (SetIOMFBBrightness) sets IOMFBBrightnessLevel on the
 * framebuffer: nits in 16.16. It also animates the dim before the display
 * turns off, so the panel dims while iOS keeps drawing, as on an iPhone.
 */
IOReturn
AppleCLCD::setProperties(OSObject *properties)
{
	OSDictionary *d = OSDynamicCast(OSDictionary, properties);
	sweet_bl_log("iomfb", d);
	OSNumber *n = d ? OSDynamicCast(OSNumber, d->getObject("IOMFBBrightnessLevel")) : NULL;
	if (n == NULL || fDsi == NULL) {
		return kIOReturnUnsupported;
	}
	uint64_t v = n->unsigned64BitValue();
	setProperty("IOMFBBrightnessLevel", v, 32);
	/* 0..SWEET_FB_MAX_NITS nits -> DBV 1..0x7ff */
	uint64_t level = (v * 0x7ffull) / ((uint64_t)SWEET_FB_MAX_NITS << 16);
	if (level > 0x7ff) {
		level = 0x7ff;
	}
	if (level < 1) {
		level = 1;
	}
	IOLockLock(fLock);
	fBlLevel = (uint32_t)level;
	IOLockUnlock(fLock);
	/*
	 * CoreBrightness runs inside backboardd and ramps the level in many small
	 * synchronous steps; waiting for the DSI here stalled rendering. Only note
	 * the level; a thread call sends the latest one.
	 */
	if (fBlCall != NULL) {
		thread_call_enter(fBlCall);
	}
	return kIOReturnSuccess;
}

void
AppleCLCD::blWork(thread_call_param_t p0, __unused thread_call_param_t p1)
{
	AppleCLCD *fb = (AppleCLCD *)p0;
	IOLockLock(fb->fLock);
	uint32_t level = fb->fBlLevel;
	bool off = fb->fPowerOff;
	IOLockUnlock(fb->fLock);
	if (!off && level != fb->fBlSent) {
		fb->fBlSent = level;
		fb->setPanelLevel(level);
	}
}

static void
sweet_bl_log(const char *who, OSDictionary *d)
{
	static uint32_t logs;
	if (d == NULL || logs >= 24) {
		return;
	}
	OSCollectionIterator *it = OSCollectionIterator::withCollection(d);
	OSSymbol *k;
	while (it != NULL && (k = OSDynamicCast(OSSymbol, it->getNextObject())) != NULL) {
		OSObject *v = d->getObject(k);
		OSNumber *n = OSDynamicCast(OSNumber, v);
		if (logs++ < 24) {
			IOLog("SWEET BL: %s set %s = %s %llu\n", who, k->getCStringNoCopy(),
			    v ? v->getMetaClass()->getClassName() : "null", n ? n->unsigned64BitValue() : 0);
		}
	}
	OSSafeReleaseNULL(it);
}

IOReturn
AppleARMBacklight::setProperties(OSObject *properties)
{
	OSDictionary *d = OSDynamicCast(OSDictionary, properties);
	sweet_bl_log("backlight", d);
	OSNumber *n = d ? OSDynamicCast(OSNumber, d->getObject("brightness-nits")) : NULL;
	if (n != NULL && fFB != NULL) {
		OSDictionary *m = OSDictionary::withCapacity(1);
		/* same 16.16 nits scale as IOMFBBrightnessLevel */
		uint64_t v = n->unsigned64BitValue();
		if (v < 0x10000) {
			v <<= 16;               /* plain nits */
		}
		OSNumber *nn = OSNumber::withNumber(v, 64);
		m->setObject("IOMFBBrightnessLevel", nn);
		fFB->setProperties(m);
		nn->release();
		m->release();
		setProperty("brightness-nits", n);
	}
	return kIOReturnSuccess;
}

/*
 * 0299: iOS's first frame: the boot screen stops being painted, and the boot framebuffer and the
 * console frame it was painted into go black -- the fade when backboardd turns the display off after
 * the Apple logo starts from one of them, and showed the boot screen again for a few seconds.
 */
void
AppleCLCD::splashEnd(void)
{
	if (!sweet_splash_stop()) {
		return;
	}
	for (uint32_t y = 0; fPixels != 0 && y < fHeight; y++) {
		volatile uint64_t *row = (volatile uint64_t *)(fPixels + (uint64_t)y * fRowBytes);
		for (uint32_t x = 0; x < fWidth / 2; x++) {
			row[x] = 0;
		}
	}
	if (fCons != NULL) {
		bzero(fCons->getBytesNoCopy(), (size_t)fRowBytes * fHeight);
	}
	__asm__ volatile("dsb sy" ::: "memory");
}

/* panic: show the console frame (MMIO only, no locks) */
static void
sweet_cons_show(void)
{
	AppleCLCD *fb = sweet_cons_fb;
	if (fb != NULL && fb->fVig0 != NULL && fb->fScanPhys != fb->fConsPhys) {
		fb->scanFrom(fb->fConsPhys, fb->fRowBytes);
	}
}

void
AppleCLCD::fadeStep(const uint8_t *src, uint32_t srcStride, uint32_t step)
{
	uint8_t *orig = (uint8_t *)fFadeOrig->getBytesNoCopy();
	uint8_t *out = backBuffer();
	if (step == 0) {
		/* snapshot what is on screen (one slow read of the frame) */
		for (uint32_t y = 0; y < fHeight; y++) {
			memcpy(orig + (uint64_t)y * fRowBytes, src + (uint64_t)y * srcStride, (size_t)fWidth * 4);
		}
	}
	/* linear fade: each channel * m / 32, m from 32 down to 0 */
	uint32_t m = 32 - (step * 32 + SWEET_FB_FADE_STEPS / 2) / SWEET_FB_FADE_STEPS;
	for (uint32_t y = 0; y < fHeight; y++) {
		const uint32_t *in = (const uint32_t *)(orig + (uint64_t)y * fRowBytes);
		uint32_t *o = (uint32_t *)(out + (uint64_t)y * fRowBytes);
		for (uint32_t x = 0; x < fWidth; x++) {
			uint32_t p = in[x];
			uint32_t lo = (((p & 0x00ff00ffu) * m) >> 5) & 0x00ff00ffu;
			uint32_t hi = ((((p >> 8) & 0x00ff00ffu) * m) >> 5) & 0x00ff00ffu;
			o[x] = lo | (hi << 8);
		}
	}
	flipBack();
}

/*
 * sapphire 0235: the GPU's copy of a frame into the back buffer is only queued when present()
 * returns; pointing the MDP at that buffer right away showed what it held before, the frame of two
 * swaps ago, until the copy landed: animations jumped back and forth by a frame (0233 gave this
 * phone flip buffers for the first time). The flip now waits for the copy's fence: at the next
 * swap or vsync tick (wait false), or, up to 20 ms, before the CPU writes the back buffer (wait
 * true: flipFrom, present's CPU path, the console). With fPresentLock held, but for the console.
 */
bool
AppleCLCD::flipWhenCopied(bool wait)
{
	if (!fFlipWait) {
		return true;
	}
	for (unsigned i = 0; sapphire_gpu_copy_pending(); i++) {
		if (!wait) {
			return false;
		}
		if (i >= 200) {
			break;                  /* the GPU is stuck: flip anyway, as before 0235 */
		}
		IODelay(100);
	}
	uint32_t idx = fFlipIdx ^ 1;
	scanFrom(fFlipPhys[idx], fRowBytes);
	sweet_fbd_flip(fIntf1);                 /* 0336 */
	fFlipIdx = idx;
	fFlipWait = false;
	return true;
}

/* Clean the back buffer and point the MDP at it (latched at its vsync). */
void
AppleCLCD::flipBack(void)
{
	uint32_t idx = fFlipIdx ^ 1;
	sweet_dcache_clean((vm_offset_t)(uintptr_t)fFlip[idx]->getBytesNoCopy(), (unsigned)fFlip[idx]->getLength());
	scanFrom(fFlipPhys[idx], fRowBytes);
	sweet_fbd.cpu_flips++;                  /* 0336: a frame the CPU wrote (console, fade, the CPU copy path) */
	fFlipIdx = idx;
}

/*
 * Where a frame's time goes (boot-arg sweetfb_stats=0 turns it off): the CPU copy of the frame
 * into the back buffer, the cache clean for the MDP, and the composed (several layers) path,
 * summed over 5 s and logged with the frame count, so it is known what handing them to the GPU
 * could save.
 */
static struct {
	uint64_t t0, copy, clean, comp, copy_max, g_clean, g_copy, g_inval;
	uint32_t direct, composed, gpu;
	int on;
} sweet_fb_stats = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1 };

static uint64_t
sweet_fb_us(uint64_t a, uint64_t b)
{
	uint64_t ns;
	absolutetime_to_nanoseconds(b - a, &ns);
	return ns / 1000;
}

static void
sweet_fb_stats_tick(void)
{
	uint64_t now = mach_absolute_time();
	if (sweet_fb_stats.t0 == 0) {
		sweet_fb_stats.t0 = now;
		return;
	}
	if (sweet_fb_us(sweet_fb_stats.t0, now) < 5000000) {
		return;
	}
	uint32_t d = sweet_fb_stats.direct, c = sweet_fb_stats.composed, g = sweet_fb_stats.gpu;
	{
		/* what holds memory, to see a leak grow: surfaces, and the GPU's buffers by kind */
		uint32_t so, gn[3];
		uint64_t sw, gb[3];
		sweet_surface_census(&so, &sw);
		sapphire_gpu_census(gn, gb);
		IOLog("SWEET MEM: %u surfaces (%llu MB of buffers), GPU: %u imported surfaces (%llu MB), %u pool buffers (%llu MB), %u kernel buffers (%llu MB), wired %u pages\n",
		    so, sw >> 20, gn[0], gb[0] >> 20, gn[1], gb[1] >> 20, gn[2], gb[2] >> 20, (unsigned)vm_page_wire_count);
	}
	IOLog("SWEET FB: 5 s: %u direct frames (copy %llu us + clean %llu us avg, copy max %llu us), %u composed by the CPU (%llu us avg), %u by the GPU (clean %llu + GPU %llu + invalidate %llu us avg); %llu ms of CPU\n",
	    d, d ? sweet_fb_stats.copy / d : 0, d ? sweet_fb_stats.clean / d : 0, sweet_fb_stats.copy_max, c,
	    c ? sweet_fb_stats.comp / c : 0, g, g ? sweet_fb_stats.g_clean / g : 0, g ? sweet_fb_stats.g_copy / g : 0,
	    g ? sweet_fb_stats.g_inval / g : 0,
	    (sweet_fb_stats.copy + sweet_fb_stats.clean + sweet_fb_stats.comp + sweet_fb_stats.g_clean +
	    sweet_fb_stats.g_copy + sweet_fb_stats.g_inval) / 1000);
	{
		/*
		 * 0336: the same five seconds as the panel was given them. The line above counts SwapEnds; a frame
		 * whose copy is replaced before its flip is in that count and never on the panel.
		 */
		static uint64_t last[4];
		uint64_t cur[4] = { sweet_fbd.swaps, sweet_fbd.flips, sweet_fbd.overwritten, 0 };
		for (unsigned k = 6; k < SWEET_FBD_N; k++) {
			cur[3] += sweet_fbd.flip_iv.n[k];
		}
		IOLog("SWEET FB: 5 s (0336): %llu swaps, %llu flips, %llu frames overwritten before their flip, %llu flips 6 or more refreshes after the one before\n",
		    cur[0] - last[0], cur[1] - last[1], cur[2] - last[2], cur[3] - last[3]);
		memcpy(last, cur, sizeof(last));
	}
	sweet_fb_stats.copy = sweet_fb_stats.clean = sweet_fb_stats.comp = sweet_fb_stats.copy_max = 0;
	sweet_fb_stats.g_clean = sweet_fb_stats.g_copy = sweet_fb_stats.g_inval = 0;
	sweet_fb_stats.direct = sweet_fb_stats.composed = sweet_fb_stats.gpu = 0;
	sweet_fb_stats.t0 = now;
}

static bool
sweet_fb_stats_on(void)
{
	if (sweet_fb_stats.on < 0) {
		sweet_fb_stats.on = 1;
		PE_parse_boot_argn("sweetfb_stats", &sweet_fb_stats.on, sizeof(sweet_fb_stats.on));
	}
	return sweet_fb_stats.on != 0;
}

/*
 * boot-arg sweetfb_gpucopy=1: a frame with one layer has its copy into the back buffer done by
 * the GPU (SapphireGpu.cpp, gpu_service=1 gpu_mmu=1), which reads the surface's scattered pages
 * through its own page tables. The source rows are cleaned from the CPU caches first; the GPU
 * writes RAM, so the destination rows are invalidated after and the flip skips its clean. Any
 * failure leaves the frame to the CPU path below, unchanged.
 */
extern bool sapphire_gpu_copy(IOMemoryDescriptor *src, uint64_t src_off, uint32_t src_pitch, uint64_t dst_pa,
    uint32_t dst_pitch, uint32_t w, uint32_t h, uint64_t dst_base, uint64_t dst_size);
extern bool sapphire_gpu_copy_scaled(IOMemoryDescriptor *src, uint64_t src_off, uint32_t src_pitch, uint32_t sx,
    uint32_t sy, uint32_t sw, uint32_t sh, uint64_t dst_pa, uint32_t dst_pitch, uint32_t dw, uint32_t dh,
    uint64_t dst_base, uint64_t dst_size);
extern bool sapphire_gpu_ready(void);
extern "C" void sweet_dcache_flush(vm_offset_t addr, unsigned length);  /* osfmk/arm/caches.c */

static int sweet_fb_gpucopy = -1;

static bool
sweet_fb_gpucopy_on(void)
{
	if (sweet_fb_gpucopy < 0) {
		sweet_fb_gpucopy = 0;
		PE_parse_boot_argn("sweetfb_gpucopy", &sweet_fb_gpucopy, sizeof(sweet_fb_gpucopy));
	}
	return sweet_fb_gpucopy != 0;
}

/*
 * sapphire 0339: the GPU is about to read surface s for the panel's copy. The kernel sees the surface's memory at va;
 * [part, part + length) is what the copy reads. What a processor may still hold dirty of it is cleaned out of the
 * caches (dc cvac), and the answer is whether anything was cleaned. With kern.sweet_gpu_flush=0 it is decided as
 * before: cleaned unless the GPU drew the surface at its present seed. Otherwise by the surface's own account
 * (SweetSurface.h): cleaned when a processor may have touched it since it was last cleaned, by this path or by a
 * submission, and then all of its memory, so that the account stands for whoever asks next. A surface around a
 * program's own memory is written without a lock and has no account: as before.
 */
extern int sapphire_gpu_flush_tracking(void);           /* SapphireGpu.cpp */

static bool
sweet_fb_clean_for_gpu(SweetSurface *s, uintptr_t va, vm_offset_t part, uint64_t length)
{
	uint32_t seed_now = s->state != NULL ? *(volatile uint32_t *)((uint8_t *)s->state->getBytesNoCopy() + 0xc) : 0;
	uint32_t epoch = s->cpuEpoch;           /* read first: an access announced meanwhile is cleaned the next time */

	if (s->lowPool) {
		return false;                   /* 0350: a scanout pool surface is write-combined everywhere: no cache holds it */
	}

	if (sapphire_gpu_flush_tracking() && !s->wrapped && s->memory != NULL) {
		if (s->cleanEpoch == epoch) {
			return false;
		}
		sweet_dcache_clean((vm_offset_t)va, (unsigned)s->memory->getLength());
		s->cleanEpoch = epoch;
		return true;
	}
	if (s->gpuDrawn && seed_now == s->gpuSeed) {
		return false;
	}
	sweet_dcache_clean(part, (unsigned)length);
	return true;
}

void
AppleCLCD::flipFrom(const uint8_t *src, uint32_t srcStride)
{
	(void)flipWhenCopied(true);
	uint8_t *dst = backBuffer();
	uint64_t t0 = mach_absolute_time();
	if (srcStride == fRowBytes) {
		memcpy(dst, src, (size_t)fRowBytes * fHeight);
	} else {
		for (uint32_t y = 0; y < fHeight; y++) {
			memcpy(dst + (uint64_t)y * fRowBytes, src + (uint64_t)y * srcStride, (size_t)fWidth * 4);
		}
	}
	uint64_t t1 = mach_absolute_time();
	flipBack();
	if (sweet_fb_stats_on()) {
		uint64_t t2 = mach_absolute_time(), cp = sweet_fb_us(t0, t1);
		sweet_fb_stats.copy += cp;
		sweet_fb_stats.clean += sweet_fb_us(t1, t2);
		if (cp > sweet_fb_stats.copy_max) {
			sweet_fb_stats.copy_max = cp;
		}
		sweet_fb_stats.direct++;
		sweet_fb_stats_tick();
	}
}

/*
 * topaz: a swap that shows one full-screen surface from the scanout pool is
 * presented by pointing the MDP at it (no 10 MB CPU copy). Returns false to
 * fall back to the copy path.
 */
bool
AppleCLCD::directPresent(const uint8_t *rec)
{
	if (fVig0 == NULL) {
		return false;
	}
	uint32_t mask = get32(rec, 0x104), layer = 32, n = 0;
	for (uint32_t i = 0; i < SWEET_FB_LAYERS; i++) {
		if ((mask & (1u << i)) && get32(rec, 0x54 + 4 * i) != 0) {
			layer = i;
			n++;
		}
	}
	if (n != 1) {
		return false;
	}
	SweetSurface *s = sweet_surface_copy(get32(rec, 0x54 + 4 * layer));
	if (s == NULL) {
		return false;
	}
	bool ok = false;
	if (s->scanoutPhys != 0 && s->bytesPerElement == 4 &&
	    s->width >= fWidth && s->height >= fHeight) {
		/* full-screen surface: panel window at the logical offset */
		uint32_t ox = (s->width - fWidth) / 2, oy = (s->height - fHeight) / 2;
		if (s->width == fLogicalW && s->height == fLogicalH) {
			ox = (uint32_t)fOffX;
			oy = (uint32_t)fOffY;
		}
		uint64_t pa = s->scanoutPhys + s->offset + (uint64_t)oy * s->bytesPerRow + (uint64_t)ox * 4;
		if ((pa & 0xf) == 0 && pa + (uint64_t)fHeight * s->bytesPerRow < (1ull << 32)) {
			uintptr_t va = s->kernelAddress();
			if (va != 0) {
				/*
				 * Copy into our back buffer and flip: CoreAnimation may start the
				 * next frame in this surface at once, so scanning it directly let
				 * the panel show frames half drawn. Cached to cached, a few ms.
				 */
				if (fFlip[0] != NULL) {
					flipFrom((const uint8_t *)(va + s->offset + (uint64_t)oy * s->bytesPerRow + (uint64_t)ox * 4),
					    s->bytesPerRow);
				} else {
					sweet_dcache_clean((vm_offset_t)(va + s->offset + (uint64_t)oy * s->bytesPerRow), (unsigned)((uint64_t)fHeight * s->bytesPerRow));
					scanFrom(pa, s->bytesPerRow);
				}
				ok = true;
				OSIncrementAtomic((volatile SInt32 *)&s->cpuEpoch);     /* 0339: the CPU has read it (flipFrom): lines of it are in the caches */
				IOLockLock(fLock);
				if (fLastSurf != s) {
					s->retain();
					if (fLastSurf != NULL) {
						fLastSurf->release();
					}
					fLastSurf = s;
				}
				fLastVA = (const uint8_t *)(va + s->offset + (uint64_t)oy * s->bytesPerRow + (uint64_t)ox * 4);
				fLastStride = s->bytesPerRow;
				IOLockUnlock(fLock);
			}
		}
	}
	s->release();
	return ok;
}

bool
AppleCLCD::present(const uint8_t *rec)
{
	uint32_t mask = get32(rec, 0x104);
	static int console_only = -1;

	/* boot-arg sweetfb_console=1: debugging, the panel keeps showing the kernel console */
	if (console_only < 0) {
		console_only = 0;
		PE_parse_boot_argn("sweetfb_console", &console_only, sizeof(console_only));
	}
	if (console_only > 0 && fConsShown) {
		(void)mask;
		return false;
	}
	fConsShown = false;
	splashEnd();                    /* 0299: iOS draws from here on */
	if (directPresent(rec)) {
		if (!fConsolePaused) {
			int quiet = 0;
			fConsolePaused = true;
			if (PE_parse_boot_argn("sweetfb_noconsole", &quiet, sizeof(quiet)) && quiet != 0) {
				sweet_console_muted = 1;
			}
		}
		return false;           /* the MDP scans the surface itself: it stays held */
	}
	IOLockLock(fLock);
	fLastVA = NULL;
	IOLockUnlock(fLock);
	uint64_t comp_t0 = mach_absolute_time();
	/*
	 * copy path (several layers): compose into the cached back buffer and
	 * flip, like the direct path. Composing into the scanned, uncached boot
	 * framebuffer showed every multi-layer frame (dims, apps) drawn top-down.
	 */
	uint32_t nlayers = 0;
	for (uint32_t layer = 0; layer < SWEET_FB_LAYERS; layer++) {
		if ((mask & (1u << layer)) && get32(rec, 0x54 + 4 * layer) != 0) {
			nlayers++;
		}
	}
	/*
	 * Without flip buffers (they need 2 x 10 MB of contiguous memory, which is not there at
	 * boot on this phone) the frame goes straight into the boot framebuffer the panel scans:
	 * contiguous, already mapped in the GPU, and write-combined for the CPU, so the GPU's
	 * writes need no invalidate.
	 */
	bool gpu_frame = nlayers == 1 && sweet_fb_gpucopy_on() && sapphire_gpu_ready();
	bool gpu_used = false;

	/*
	 * 0235: the last GPU copy, landed, goes on the panel first. A frame the CPU composes waits for
	 * it (it writes the same back buffer); one the GPU copies is queued behind it, in order, and the
	 * flip of both waits for the newer copy.
	 */
	if (fFlip[0] != NULL) {
		(void)flipWhenCopied(!gpu_frame);
	}
	uint8_t *dst = fFlip[0] != NULL ? backBuffer() : (uint8_t *)fPixels;
	if (fFlip[0] == NULL && fVig0 != NULL && fScanPhys != fBootSrc) {
		scanFrom(fBootSrc, fBootStride);
	}


	if (!fConsolePaused) {
		int quiet = 0;
		fConsolePaused = true;
		splashEnd();                    /* 0299: iOS draws from here on */
		if (PE_parse_boot_argn("sweetfb_noconsole", &quiet, sizeof(quiet)) && quiet != 0) {
			/* Not PE_initialize_console(kPEDisableScreen): its console switch
			 * writes the read-only-late cons_ops_index and panics. */
			sweet_console_muted = 1;
		}
	}

	for (uint32_t layer = 0; layer < SWEET_FB_LAYERS; layer++) {
		uint32_t id = get32(rec, 0x54 + 4 * layer);
		const uint8_t *a = rec + 0x64 + 16 * layer;
		const uint8_t *b = rec + 0xc4 + 16 * layer;
		int32_t sx = (int32_t)get32(a, 0), sy = (int32_t)get32(a, 4);
		int32_t dx = (int32_t)get32(b, 0), dy = (int32_t)get32(b, 4);
		int32_t w = (int32_t)get32(b, 8), h = (int32_t)get32(b, 12);

		if (!(mask & (1u << layer)) || id == 0) {
			continue;
		}
		SweetSurface *s = sweet_surface_copy(id);
		if (sweet_fb_swaps_logged < 16) {
			sweet_fb_swaps_logged++;
			IOLog("SWEET FB: swap %u layer %u surface %u %ux%u fmt 0x%x bpr %u a %d,%d %ux%u b %d,%d %dx%d\n",
			    get32(rec, 0x50), layer, id, s ? s->width : 0, s ? s->height : 0,
			    s ? s->pixelFormat : 0, s ? s->bytesPerRow : 0, sx, sy, get32(a, 8),
			    get32(a, 12), dx, dy, w, h);
		}
		if (s == NULL || s == fSurface) {
			OSSafeReleaseNULL(s);
			continue;
		}
		uintptr_t src = s->kernelAddress();
		uint32_t bpe = s->bytesPerElement ? s->bytesPerElement : 4;
		if (fScale2) {
			/*
			 * 0239: a 2x frame (one full-screen layer, as CoreAnimation sends them) goes onto the
			 * panel scaled by the GPU: the panel's window of the 3x picture, at 2/3 of its place
			 * and size. Anything else is not shown (the CPU copies 1:1 and would put it small).
			 */
			static uint32_t scale_told;
			bool shown = false;
			if (gpu_frame && bpe == 4 && src != 0 && s->width >= fLogicalW && s->height >= fLogicalH &&
			    s->elementWidth == 1 && s->elementHeight == 1 && s->memory != NULL) {
				uint64_t dbase = fFlip[0] != NULL ? fFlipPhys[fFlipIdx ^ 1] : fPhys;
				uint64_t clean_t0 = mach_absolute_time();
				if (sweet_fb_clean_for_gpu(s, src, (vm_offset_t)(src + s->offset), (uint64_t)s->height * s->bytesPerRow)) {
					sweet_fbd.cleans++;             /* 0336: the whole surface out of the CPU caches for this frame */
					sweet_fbd.clean_us += sweet_fbd_us(clean_t0, mach_absolute_time());
				}
				/* the panel's window of the 3x picture, in the 2x one's pixels << 8 (0321: of the part drawn) */
				int rs = sweet_fb_rscale >= 400 && sweet_fb_rscale <= 1000 ? sweet_fb_rscale : 1000;
				uint64_t kx = ((uint64_t)fLogicalW << 8) * (uint64_t)rs / 1000, ky = ((uint64_t)fLogicalH << 8) * (uint64_t)rs / 1000;
				{
					uint32_t sid = s->id;
					bool known = false;
					for (unsigned i = 0; i < 4; i++) {
						known |= sweet_fb_ids[i] == sid;
					}
					if (!known) {
						sweet_fb_ids[sweet_fb_ids_next++ & 3] = sid;
					}
				}
				shown = sapphire_gpu_copy_scaled(s->memory, s->offset, s->bytesPerRow,
				    (uint32_t)((uint64_t)fOffX * kx / SWEET_FB_LOGICAL_W), (uint32_t)((uint64_t)fOffY * ky / SWEET_FB_LOGICAL_H),
				    (uint32_t)((uint64_t)fWidth * kx / SWEET_FB_LOGICAL_W), (uint32_t)((uint64_t)fHeight * ky / SWEET_FB_LOGICAL_H),
				    dbase, fRowBytes, fWidth, fHeight, dbase, (uint64_t)fRowBytes * fHeight);
				if (shown) {
					if (fFlip[0] != NULL && sweet_fb_flipflush) {
						sweet_dcache_flush((vm_offset_t)dst, (unsigned)((uint64_t)fHeight * fRowBytes));
					}
					gpu_used = true;
				}
			}
			if (!shown && scale_told++ < 8) {
				IOLog("SWEET FB: 2x frame not shown (layers %u, gpu %d, surface %ux%u bpe %u)\n", nlayers, gpu_frame,
				    s->width, s->height, bpe);
			}
			s->release();
			continue;
		}
		if (w <= 0 || h <= 0) {
			w = (int32_t)s->width;
			h = (int32_t)s->height;
		}
		if (sx < 0 || sy < 0) {
			sx = sy = 0;
		}
		if (dx < 0 || dy < 0) {
			dx = dy = 0;
		}
		/* Logical (presented) display coordinates -> panel coordinates. */
		dx -= fOffX;
		dy -= fOffY;
		if (dx < 0) {
			sx -= dx;
			w += dx;
			dx = 0;
		}
		if (dy < 0) {
			sy -= dy;
			h += dy;
			dy = 0;
		}
		if (w > (int32_t)s->width - sx) {
			w = (int32_t)s->width - sx;
		}
		if (h > (int32_t)s->height - sy) {
			h = (int32_t)s->height - sy;
		}
		if (w > (int32_t)fWidth - dx) {
			w = (int32_t)fWidth - dx;
		}
		if (h > (int32_t)fHeight - dy) {
			h = (int32_t)fHeight - dy;
		}
		uint64_t last = (uint64_t)s->offset + (uint64_t)(sy + h - 1) * s->bytesPerRow +
		    (uint64_t)(sx + w) * 4;
		if (w <= 0 || h <= 0 || s->elementWidth != 1 || s->elementHeight != 1 ||
		    s->memory == NULL || last > s->memory->getLength()) {
			src = 0;
		}
		if (src != 0 && bpe == 4 && gpu_frame) {
			uint64_t soff = s->offset + (uint64_t)sy * s->bytesPerRow + (uint64_t)sx * 4;
			uint64_t dbase = fFlip[0] != NULL ? fFlipPhys[fFlipIdx ^ 1] : fPhys;
			uint64_t t0 = mach_absolute_time();
			/*
			 * sapphire 0151: a surface the GPU drew (msm client) and nobody locked since -- its seed
			 * is the one of that submission -- has nothing of the CPU's in the caches to push out.
			 */
			(void)sweet_fb_clean_for_gpu(s, src, (vm_offset_t)(src + soff), (uint64_t)(h - 1) * s->bytesPerRow + (uint64_t)w * 4);
			uint64_t t1 = mach_absolute_time();
			if (sapphire_gpu_copy(s->memory, soff, s->bytesPerRow,
			    dbase + (uint64_t)dy * fRowBytes + (uint64_t)dx * 4, fRowBytes, (uint32_t)w, (uint32_t)h,
			    dbase, (uint64_t)fRowBytes * fHeight)) {
				uint64_t t2 = mach_absolute_time();
				if (fFlip[0] != NULL) {
					sweet_dcache_flush((vm_offset_t)(dst + (uint64_t)dy * fRowBytes), (unsigned)((uint64_t)h * fRowBytes));
				}
				gpu_used = true;
				if (sweet_fb_stats_on()) {
					sweet_fb_stats.g_clean += sweet_fb_us(t0, t1);
					sweet_fb_stats.g_copy += sweet_fb_us(t1, t2);
					sweet_fb_stats.g_inval += sweet_fb_us(t2, mach_absolute_time());
				}
				src = 0;                /* done: skip the CPU copy */
			}
		}
		if (src != 0 && bpe == 4) {
			/* 0235: a GPU frame whose copy was refused: the earlier copy into this buffer lands first */
			for (unsigned i = 0; fFlipWait && i < 200 && sapphire_gpu_copy_pending(); i++) {
				IODelay(100);
			}
			for (int32_t y = 0; y < h; y++) {
				memcpy((void *)(dst + (uint64_t)(dy + y) * fRowBytes + (uint64_t)dx * 4),
				    (const void *)(src + s->offset + (uint64_t)(sy + y) * s->bytesPerRow + (uint64_t)sx * 4),
				    (size_t)w * 4);
			}
			OSIncrementAtomic((volatile SInt32 *)&s->cpuEpoch);     /* 0339: the CPU has read it: lines of it are in the caches */
		}
		s->release();
	}
	if (gpu_used) {
		/*
		 * the GPU writes RAM: the MDP is pointed at the back buffer once the copy has landed
		 * (0235: flipWhenCopied(), at the next swap or vsync tick), no clean
		 */
		if (fFlip[0] != NULL) {
			if (fFlipWait) {
				/* 0336: the frame before this one is still waiting for its flip, and this copy goes over it */
				sweet_fbd.overwritten++;
			}
			fFlipWait = true;
			sweet_fbd.pend_swap = mach_absolute_time();
		}
		if (sweet_fb_stats_on()) {
			sweet_fb_stats.gpu++;
			sweet_fb_stats_tick();
		}
		return true;
	}
	if (fScale2) {
		/* 0354: nothing of this 2x frame was copied (one full-screen layer, by the GPU, is all it shows): not shown */
		sweet_fbs.not_shown++;
		return true;
	}
	if (fFlip[0] != NULL) {
		flipBack();
		fFlipWait = false;      /* (this frame, the CPU's, replaces one still waiting for its copy) */
	}
	if (sweet_fb_stats_on()) {
		sweet_fb_stats.comp += sweet_fb_us(comp_t0, mach_absolute_time());
		sweet_fb_stats.composed++;
		sweet_fb_stats_tick();
	}
	return true;                    /* copied by the CPU: the surface is free */
}

/* ---------------------------------------------------------------- sapphire 0340: the frame queue */

bool
AppleCLCD::queueOn(void)
{
	return sweet_fb_queue != 0 && fFlip[0] != NULL && fFlip[1] != NULL && fPresentLock != NULL && fVig0 != NULL;
}

/*
 * Whether the MDP has taken the last flip written (scanFrom()), so that the buffer it was reading is free and the
 * next flip may be written. Its frame counter has moved on since the flip was written: once, or twice if the flip was
 * written in the last sixteenth of a refresh, where it may have come too late for the vsync that followed. Without
 * the counter, or if it stands still, 34 ms (two refreshes at 60 Hz) are enough.
 */
bool
AppleCLCD::flipTaken(uint64_t now)
{
	uint64_t limit;

	if (!fQLatchWait) {
		return true;
	}
	bool taken;
	if (fFlushReadback && (sweet_fb_flush_taken != 0 || fScanUsed) && fCtl0 != NULL) {
		taken = (fCtl0[0x18 / 4] & 0x20001) == 0;      /* 0352: the MDP has taken the flush (sweet_fb_flush_taken) */
	} else {
		taken = fIntf1 != NULL && (uint32_t)(fIntf1[(0x800 + 0x0ac) / 4] - fQFlipFrame) >= (fQFlipLate ? 2u : 1u);
	}
	if (!taken) {
		nanoseconds_to_absolutetime(34 * NSEC_PER_MSEC, &limit);
		if (now - fQFlipAbs < limit) {
			return false;
		}
		sweet_fbq.by_time++;
	}
	fQLatchWait = false;
	fQRetirePa[0] = fQRetirePa[1] = 0;
	if (fFlipSwap != 0) {
		fPanelSwap = fFlipSwap;         /* 0353 */
	}
	return true;
}

/* The buffer at pa is the one the MDP is pointed at now: it goes to slot fFlipIdx, where everything else looks for it. */
void
AppleCLCD::queueFront(uint64_t pa)
{
	for (uint32_t k = 0; k < SWEET_FB_BUFS; k++) {
		if (fFlip[k] != NULL && fFlipPhys[k] == pa && k != fFlipIdx) {
			SweetLowBuf *b = fFlip[k];
			fFlip[k] = fFlip[fFlipIdx];
			fFlipPhys[k] = fFlipPhys[fFlipIdx];
			fFlip[fFlipIdx] = b;
			fFlipPhys[fFlipIdx] = pa;
			return;
		}
	}
}

/* A flip buffer nothing is using: not the one the MDP reads, nor the one it is leaving, nor one of a waiting frame. */
int
AppleCLCD::queueFreeSlot(void)
{
	for (int i = 0; i < SWEET_FB_BUFS; i++) {
		bool busy = fFlip[i] == NULL || fFlipPhys[i] == fScanPhys || fFlipPhys[i] == fQRetirePa[0] || fFlipPhys[i] == fQRetirePa[1];
		for (uint32_t k = 0; !busy && k < fQCount; k++) {
			busy = fQ[k].pa == fFlipPhys[i];
		}
		if (!busy) {
			return i;
		}
	}
	return -1;
}

/* The waiting frames are given up: none of them is shown. Their copies still run, each into its own buffer. */
void
AppleCLCD::queueDrop(void)
{
	for (uint32_t k = 0; k < fQCount; k++) {
		sweet_hold_set(fQ[k].held, false);
		sweet_fbq.dropped++;
	}
	if (fQCount != 0) {
		fQCount = 0;
		IOLockWakeup(fPresentLock, &fQCount, false);
	}
}

/*
 * The queue's turn, from its thread, from the timer's ticks and from SwapEnd: the oldest waiting frame goes to the
 * panel if its copy is done and the MDP has taken the flip before it. With the display off the waiting frames are
 * given up (0295: what was drawn before "off" is not what "on" shows).
 */
void
AppleCLCD::queueFlip(void)
{
	sweet_fbl_sample(fCtl0, fIntf1);        /* 0349 (read only) */
	uint64_t now = mach_absolute_time();
	bool waiting = fQLatchWait;
	bool taken = flipTaken(now);
	bool wake = waiting && taken;           /* the buffer the MDP was leaving is free from now on */

	if (taken) {
		scanoutRetire();                        /* 0350: ... and the surfaces it was leaving */
	} else if (fPlaneScaled && fFlushReadback && !fScanStuck && fCtl0 != NULL && (fCtl0[0x18 / 4] & 0x20001) != 0) {
		uint64_t limit;
		nanoseconds_to_absolutetime(100 * NSEC_PER_MSEC, &limit);
		if (now - fQFlipAbs > limit) {
			/* 0350: the MDP has not taken a flush of the scaled plane in 100 ms: back to the copies, for good */
			fScanStuck = true;
			if (sweet_fb_scanout == 1) {
				sweet_fb_scanout = 0;
			}
			sweet_fbs.stuck++;
			IOLog("SWEET FB: scanout: the MDP has not taken a flush in 100 ms (CTL_FLUSH %#x): scanout off\n", fCtl0[0x18 / 4]);
		}
	}
	if (fPowerOff) {
		queueDrop();
	} else if (fQCount != 0 && taken && frameDone(&fQ[0], now)) {
		/* 0350: a flip that rewrites the plane (to a surface, or back from one) only where it cannot be latched half written */
		if ((fQ[0].scan || fPlaneScaled) && !fScanStuck && !planeWritable()) {
			sweet_fbs.window_waits++;
		} else {
			struct SweetQFrame f = fQ[0];
			for (uint32_t k = 1; k < fQCount; k++) {
				fQ[k - 1] = fQ[k];
			}
			fQCount--;
			sweet_fbd.pend_swap = f.queued;         /* 0336's "swap to flip" is this frame's, from its copy being queued */
			frameShow(&f);
			sweet_fbq.late += fQFlipLate;
			if (f.arrived != 0 && now > f.arrived) {
				sweet_fbd_add(&sweet_fbq.lat, sweet_fbd_lat_us, sizeof(sweet_fbd_lat_us) / sizeof(sweet_fbd_lat_us[0]),
				    sweet_fbd_us(f.arrived, now));
			}
			sweet_hold_set(f.held, false);          /* (a frame of a surface keeps its: frameShow() took them) */
			wake = true;                            /* ... and one frame fewer waits */
		}
	}
	if (wake) {
		IOLockWakeup(fPresentLock, &fQCount, false);    /* whoever waits in SwapEnd for a buffer or for its turn */
	}
}

/*
 * Before the CPU writes a flip buffer, and when the queue is switched off with frames still in it: the copies queued
 * are waited for (the GPU runs them in order, so the newest one's fence covers them all; up to 20 ms, as
 * flipWhenCopied(true) waits), the newest frame goes to the panel and the older ones are given up.
 */
void
AppleCLCD::queueDrain(void)
{
	if (fQCount == 0) {
		return;
	}
	struct SweetQFrame f = fQ[fQCount - 1];
	for (unsigned i = 0; i < 200 && !frameDone(&f, mach_absolute_time()); i++) {
		IODelay(100);
	}
	fQCount--;
	queueDrop();
	if (!fPowerOff && (!f.scan || frameDone(&f, mach_absolute_time()))) {
		sweet_fbd.pend_swap = f.queued;
		frameShow(&f);                  /* (0350: a frame of a surface rewrites the plane where it may) */
	} else if (f.scan) {
		sweet_fbq.dropped++;            /* 0352: a frame of a surface still being drawn is not shown half drawn */
	}
	sweet_hold_set(f.held, false);
	sweet_fbq.drained++;
	IOLockWakeup(fPresentLock, &fQCount, false);
}

/*
 * A frame presented through the queue (fPresentLock held; from SwapEnd, or from the timer for its own frames). Says
 * what present() says, and in *queued whether the frame now waits in the queue, which then holds its surfaces.
 */
bool
AppleCLCD::presentQueued(const uint8_t *rec, SweetSurface **held, bool *queued)
{
	uint32_t mask = get32(rec, 0x104), nlayers = 0;

	for (uint32_t layer = 0; layer < SWEET_FB_LAYERS; layer++) {
		if ((mask & (1u << layer)) && get32(rec, 0x54 + 4 * layer) != 0) {
			nlayers++;
		}
	}
	bool gpu_frame = nlayers == 1 && sweet_fb_gpucopy_on() && sapphire_gpu_ready();   /* as present() decides it */

	if (fFlipWait) {
		/* the queue was just turned on: the copy the old path left pending is the queue's first frame */
		if (fQCount < SWEET_FB_BUFS) {
			struct SweetQFrame *f = &fQ[fQCount++];
			bzero(f, sizeof(*f));
			f->pa = fFlipPhys[fFlipIdx ^ 1];
			f->fence = sapphire_gpu_copy_fence();
			f->queued = sweet_fbd.pend_swap;
			sweet_fbq.adopted++;
		}
		fFlipWait = false;
		sweet_fbd.pend_swap = 0;
	}
	queueFlip();
	if (!gpu_frame) {
		queueDrain();           /* the CPU writes its buffer at once: nothing older may still be on its way */
	}

	uint32_t depth = sweet_fb_queue_depth < 1 ? 1 : (uint32_t)sweet_fb_queue_depth;
	int slot = fQCount < depth ? queueFreeSlot() : -1;
	if (slot < 0) {
		/* every buffer is taken, or enough frames wait already: until the oldest one's flip is taken, but not for long */
		uint64_t t0 = mach_absolute_time(), deadline;
		int ms = sweet_fb_queue_hold_ms;
		if (ms > 0) {
			clock_interval_to_deadline((uint32_t)(ms > 200 ? 200 : ms), kMillisecondScale, &deadline);
			do {
				IOLockWakeup(fPresentLock, &fQThread, true);
				int result = IOLockSleepDeadline(fPresentLock, &fQCount, deadline, THREAD_UNINT);
				queueFlip();
				slot = fQCount < depth ? queueFreeSlot() : -1;
				if (result == THREAD_TIMED_OUT) {
					if (fQCount != 0 || !fQLatchWait) {
						break;
					}
					/* no frame waits, only the vsync that frees the buffer the MDP is leaving: 34 ms from its flip at most */
					clock_interval_to_deadline(1, kMillisecondScale, &deadline);
				}
			} while (slot < 0);
		}
		uint64_t us = sweet_fbd_us(t0, mach_absolute_time());
		sweet_fbq.holds++;
		sweet_fbq.hold_us += us;
		if (us > sweet_fbq.hold_max_us) {
			sweet_fbq.hold_max_us = us;
		}
	}
	if (slot < 0 && fQCount != 0) {
		/* still none: the newest waiting frame gives up its buffer to this one and is never shown */
		struct SweetQFrame *f = &fQ[--fQCount];
		for (int i = 0; i < SWEET_FB_BUFS; i++) {
			if (fFlip[i] != NULL && fFlipPhys[i] == f->pa) {
				slot = i;
			}
		}
		sweet_hold_set(f->held, false);
		sweet_fbd.overwritten++;
		sweet_fbq.timeouts++;
	}
	if (slot < 0 && fQLatchWait) {
		/* (no frame waits and nobody is to be kept waiting, kern.sweet_fb_queue_hold_ms=0: a buffer the MDP may
		 * still be reading, as if it had taken the last flip. The one way left to write into what the panel shows.) */
		fQLatchWait = false;
		fQRetirePa[0] = fQRetirePa[1] = 0;
		slot = queueFreeSlot();
		sweet_fbq.forced++;
	}
	uint32_t back = fFlipIdx ^ 1;           /* where present() takes its back buffer from */
	if (slot >= 0 && (uint32_t)slot != back) {
		SweetLowBuf *b = fFlip[slot];
		uint64_t b_pa = fFlipPhys[slot];
		fFlip[slot] = fFlip[back];
		fFlipPhys[slot] = fFlipPhys[back];
		fFlip[back] = b;
		fFlipPhys[back] = b_pa;
	}
	uint64_t pa = fFlipPhys[back];

	fSwapShow = get32(rec, 0x50);           /* 0353: (a frame the CPU writes is flipped in present()) */
	bool copied = present(rec);
	fSwapShow = 0;

	if (fFlipWait) {
		/* the GPU's copy into that buffer is queued: the frame waits its turn, and keeps its surfaces until then */
		fFlipWait = false;
		if (fQCount < SWEET_FB_BUFS) {
			struct SweetQFrame *f = &fQ[fQCount++];
			bzero(f, sizeof(*f));           /* 0352: (a frame of a surface may have had this place: scan, surf) */
			f->pa = pa;
			f->fence = sapphire_gpu_copy_fence();
			f->queued = sweet_fbd.pend_swap;
			f->arrived = fQArrived;
			f->swap = get32(rec, 0x50);     /* 0353 */
			memcpy(f->held, held, sizeof(f->held));
			*queued = true;
			sweet_fbq.frames++;
			if (fQCount > sweet_fbq.depth_max) {
				sweet_fbq.depth_max = fQCount;
			}
		}
		sweet_fbd.pend_swap = 0;
		queueFlip();                            /* (a copy with nothing ahead of it may be done already) */
		IOLockWakeup(fPresentLock, &fQThread, true);
	} else if (fScanPhys == pa) {
		queueDrop();            /* the CPU wrote the frame and flipped to it: whatever still waited is older */
	}
	return copied;
}

void
AppleCLCD::queueThreadMain(void *self, __unused wait_result_t result)
{
	((AppleCLCD *)self)->queueThread();
}

/* The thread that flips: asleep while nothing waits, otherwise looking every kern.sweet_fb_queue_poll_us. */
void
AppleCLCD::queueThread(void)
{
	IOLockLock(fPresentLock);
	for (;;) {
		if (fQCount != 0 || fQLatchWait) {
			uint64_t flips = sweet_fbq.flips;
			queueFlip();
			sweet_fbq.flips_thread += sweet_fbq.flips - flips;
		}
		if (fQCount != 0 || (fQLatchWait && queueOn())) {
			uint64_t deadline;
			int us = sweet_fb_queue_poll_us;
			clock_interval_to_deadline((uint32_t)(us < 100 ? 100 : us > 8000 ? 8000 : us), kMicrosecondScale, &deadline);
			(void)IOLockSleepDeadline(fPresentLock, &fQThread, deadline, THREAD_UNINT);
			sweet_fbq.polls++;
		} else {
			(void)IOLockSleep(fPresentLock, &fQThread, THREAD_UNINT);
		}
	}
}

/* ---------------------------------------------------------------- sapphire 0350: the scanout of the window server's surfaces */

bool
AppleCLCD::scanoutOn(void)
{
	/* 0354: and SwapWait answered (kern.sweet_fb_swapwait): without it the window server draws into the surface on the panel */
	return sweet_fb_scanout == 1 && sweet_fb_swapwait != 0 && sweet_fb_scan_ready && !fScanStuck && fQThread && queueOn();
}

/* Whether the panel may still read surface s, or a frame of it to be scanned out waits: then it is not scanned out again. */
bool
AppleCLCD::scanoutBusy(SweetSurface *s)
{
	for (uint32_t k = 0; k < 4; k++) {
		if (fScanShow[k] == s) {
			return true;
		}
		for (uint32_t f = 0; f < 4; f++) {
			if (fScanLeave[f][k] == s) {
				return true;
			}
		}
		for (uint32_t q = 0; q < fQCount; q++) {
			if (fQ[q].scan && fQ[q].held[k] == s) {
				return true;   /* 0353: (a waiting copy reads it on the GPU's ring before whatever is drawn into it next) */
			}
		}
	}
	return false;
}

/*
 * 0354: whether a surface of the window server's ring that cannot be scanned out is still alive (freed or released by
 * every user, it is no longer of the ring). While one is, every frame goes the copy's way: the plane keeps one mode.
 * fPresentLock held.
 */
bool
AppleCLCD::scanoutBlocked(void)
{
	bool any = false;
	for (uint32_t i = 0; i < sizeof(fScanBlock) / sizeof(fScanBlock[0]); i++) {
		if (fScanBlock[i].s == NULL) {
			continue;
		}
		if (!sweet_surface_alive(fScanBlock[i].id, fScanBlock[i].s)) {
			bzero(&fScanBlock[i], sizeof(fScanBlock[i]));
			continue;
		}
		any = true;
	}
	return any;
}

/* 0354: s, swapped by the window server, cannot be scanned out (why: 1 not from the pool, 2 unfit). fPresentLock held. */
void
AppleCLCD::scanoutBlock(SweetSurface *s, uint8_t why)
{
	uint32_t n = sizeof(fScanBlock) / sizeof(fScanBlock[0]), slot = n, oldest = 0;
	for (uint32_t i = 0; i < n; i++) {
		if (fScanBlock[i].s == s && fScanBlock[i].id == s->id) {
			return;
		}
		if (fScanBlock[i].s == NULL && slot == n) {
			slot = i;
		}
		if (fScanBlock[i].since < fScanBlock[oldest].since) {
			oldest = i;
		}
	}
	if (slot == n) {
		slot = oldest;
	}
	fScanBlock[slot].id = s->id;
	fScanBlock[slot].s = s;
	fScanBlock[slot].why = why;
	fScanBlock[slot].since = mach_absolute_time();
	sweet_fbs.blockers++;
}

/*
 * A flip is being written: the surfaces of the frame on the panel (a frame of a surface) are read until the MDP takes
 * it, and kept in use until then (scanoutRetire()). With four frames left and no flip seen taken since, the oldest is
 * let go: four flips have been written since it was on the panel.
 */
void
AppleCLCD::scanoutLeave(void)
{
	bool any = false;
	for (uint32_t k = 0; k < 4; k++) {
		any |= fScanShow[k] != NULL;
	}
	if (!any) {
		fScanShowSwap = 0;
		return;
	}
	uint32_t f = 0;
	for (; f < 4; f++) {
		bool used = false;
		for (uint32_t k = 0; k < 4; k++) {
			used |= fScanLeave[f][k] != NULL;
		}
		if (!used) {
			break;
		}
	}
	if (f == 4) {
		sweet_hold_set(fScanLeave[0], false);
		sweet_fbs.released++;
		for (uint32_t k = 0; k < 3; k++) {
			memcpy(fScanLeave[k], fScanLeave[k + 1], sizeof(fScanLeave[k]));
			fScanLeaveSwap[k] = fScanLeaveSwap[k + 1];      /* 0353 */
		}
		bzero(fScanLeave[3], sizeof(fScanLeave[3]));
		f = 3;
	}
	memcpy(fScanLeave[f], fScanShow, sizeof(fScanShow));
	fScanLeaveSwap[f] = fScanShowSwap;      /* 0353 */
	bzero(fScanShow, sizeof(fScanShow));
	fScanShowSwap = 0;
}

/* The MDP has taken the last flip written: it reads that frame alone, and the surfaces it left are free. */
void
AppleCLCD::scanoutRetire(void)
{
	for (uint32_t f = 0; f < 4; f++) {
		bool used = false;
		for (uint32_t k = 0; k < 4; k++) {
			used |= fScanLeave[f][k] != NULL;
		}
		if (used) {
			sweet_hold_set(fScanLeave[f], false);
			sweet_fbs.released++;
		}
		fScanLeaveSwap[f] = 0;                  /* 0353 */
	}
}

/*
 * Whether the plane's registers may be rewritten now: the MDP has taken the last flush (CTL_FLUSH's VIG0 and CTL bits
 * clear: what is written while one is pending is latched with it), and the scan is between a sixteenth and three
 * quarters of the refresh (INTF_LINE_COUNT, in lines of vtotal), milliseconds away from the next frame start and from the
 * programmable fetch before it, where the plane is latched; writing it takes microseconds.
 * 0352: once CTL_FLUSH is seen to read back (flushNote()), bits clear are enough, at any line: nothing written then is
 * latched before the flush written after it. Linux does no more (msm_atomic.c, msm_atomic_commit_tail: wait_flush, "now
 * that there is no in-progress flush", the planes' registers, then the flush), and the window cost a refresh to every
 * frame whose drawing ended in the last quarter of one.
 */
bool
AppleCLCD::planeWritable(void)
{
	if (fCtl0 == NULL || fIntf1 == NULL || (fCtl0[0x18 / 4] & 0x20001) != 0) {
		return false;
	}
	if (fFlushReadback) {
		return true;
	}
	uint32_t hsync = fIntf1[(0x800 + 0x008) / 4] >> 16;
	uint32_t vtotal = hsync != 0 ? fIntf1[(0x800 + 0x00c) / 4] / hsync : 0;
	uint64_t line = fIntf1[(0x800 + 0x0b0) / 4];
	return vtotal != 0 && line < vtotal && line * 16 >= vtotal && line * 4 < (uint64_t)vtotal * 3;
}

/* Until the plane may be rewritten, 20 ms at most (a refresh is 8.3): spinning, so that it also serves a panic. */
static void
sweet_plane_wait(AppleCLCD *fb)
{
	uint64_t t0 = mach_absolute_time(), now = t0, limit;
	nanoseconds_to_absolutetime(20 * NSEC_PER_MSEC, &limit);
	while (!fb->fScanStuck && !fb->planeWritable() && (now = mach_absolute_time()) - t0 < limit) {
	}
	if (now != t0) {
		uint64_t us = sweet_fbd_us(t0, now);
		sweet_fbs.unscaled_waits++;
		if (us > sweet_fbs.unscaled_wait_max_us) {
			sweet_fbs.unscaled_wait_max_us = us;
		}
	}
}

/* Back to the plane the bootloader set up (unscaled, the panel's size), before a flip of a flip buffer. */
void
AppleCLCD::planeUnscaled(void)
{
	if (!fPlaneScaled || fVig0 == NULL) {
		return;
	}
	sweet_plane_wait(this);
	for (unsigned k = 0; k < SWEET_PLANE_REGS; k++) {
		fVig0[sweet_plane_off[k] / 4] = fPlane0[k];
	}
	fPlaneScaled = false;
	sweet_fbs.to_unscaled++;
}

/*
 * The panel shows surface memory at pa (rows of stride bytes), part g of it scaled onto the whole panel: the plane is
 * rewritten when it is not set up so already, then pointed at the surface, and flushed. Whether CTL_FLUSH reads back
 * the bits until the MDP takes them is looked at once, here: the write is far from a frame start (planeWritable()), so
 * bits read clear right after it mean that the register does not read back.
 */
void
AppleCLCD::planeScaled(uint64_t pa, uint32_t stride, const struct SweetGeo *g)
{
	bool geometry = !fPlaneScaled || memcmp(g, &fPlaneGeo, sizeof(*g)) != 0;

	if (!planeWritable()) {
		sweet_plane_wait(this);         /* (from queueDrain(); the queue's flips come here writable) */
	}
	flipNote(pa, geometry);                 /* 0352: (after the wait: the frame and line the flip is written in) */
	if (geometry) {
		uint32_t v[SWEET_PLANE_REGS];
		sweet_plane_scaled_regs(v, g->sx, g->sy, g->sw, g->sh, fWidth, fHeight, fPlane0[3], fPlaneFormat, fPlane0[26]);
		for (unsigned k = 0; k < SWEET_PLANE_REGS; k++) {
			fVig0[sweet_plane_off[k] / 4] = v[k];
		}
		if (!fPlaneScaled) {
			sweet_fbs.to_scaled++;
		}
		fPlaneScaled = true;
		fPlaneGeo = *g;
	}
	scanoutLeave();
	fVig0[0x14 / 4] = (uint32_t)pa;
	fVig0[0x24 / 4] = stride;
	if (sweet_fb_ubwc != 0 && fQCount != 0 && fQ[fQCount - 1].ubwc) {
		struct SweetQFrame *q = &fQ[fQCount - 1];
		fVig0[0x1c / 4] = (uint32_t)q->metaPa;
		fVig0[0x2c / 4] = q->metaStride;
		fVig0[0x30 / 4] = fPlaneFormat | 0x80000000u; /* MDP_FETCH_UBWC(2) */
		fVig0[0x38 / 4] = fPlane0[26] | 0x80000001u; /* PE_OVERRIDE | BWC_EN */
		fVig0[0x48 / 4] = 0x00040087u;               /* reset 0x87, DPU HBB 1 */
		fVig0[0x44 / 4] = 0x80000111u;               /* alpha, UBWC1, swizzle 7, HBB 1 */
		fVig0[0x138 / 4] = 0x80000000u;
		sweet_fb_ubwc_flips++;
		sweet_fb_ubwc_last_error = fVig0[0x138 / 4];
		if (sweet_fb_ubwc_last_error != 0) {
			sweet_fb_ubwc_errors++;
			sweet_fb_ubwc = 0;
		}
	}
	__asm__ volatile("dsb sy" ::: "memory");
	fCtl0[0x18 / 4] = 0x20001;      /* flush VIG0 + CTL: latched at the next frame start */
	__asm__ volatile("dsb sy" ::: "memory");
	flushNote();                    /* 0352: (the one-shot look of 0350 is now the count of every flip) */
	if (!fScanUsed) {
		fScanUsed = true;
		IOLog("SWEET FB: scanout: first frame of a surface on the panel (%u,%u %ux%u of it scaled to %ux%u); CTL_FLUSH %s\n",
		    g->sx, g->sy, g->sw, g->sh, fWidth, fHeight, fFlushReadback ? "reads back until taken: flips are taken by it" :
		    fFlushKnown ? "reads clear at once: flips are taken by the frame counter" : "not known yet (fewer than 16 flips)");
	}
	sweet_fbl_wrote(fCtl0, fIntf1); /* 0349 */
	fScanPhys = pa;
	fScanStride = stride;
	fPlaneStride = stride;
}

/*
 * Whether a frame's drawing is done: a copy's fence, or for a frame of a surface the msm fences. 0352: no time limit (0350
 * showed a frame of a surface after 250 ms, half drawn): a GPU that hangs is reset after 2 s and its fences jump to the
 * last number (0317), as for the copies.
 */
bool
AppleCLCD::frameDone(const struct SweetQFrame *f, __unused uint64_t now)
{
	if (!f->scan) {
		return sapphire_gpu_fence_done(f->fence);
	}
	uint32_t want = f->fence, last = f->surf != NULL ? f->surf->gpuFence : 0;
	if (last != 0 && (int32_t)(last - want) > 0) {
		want = last;                    /* a submission after the SwapEnd named the surface: wait for that one too */
	}
	return sapphire_gpu_msm_fence_done(want);
}

/* The frame goes to the panel (fPresentLock held). A frame of a surface keeps its surfaces in use: its held[] is taken. */
void
AppleCLCD::frameShow(struct SweetQFrame *f)
{
	fSwapShow = f->swap;                    /* 0353: (flipNote() notes it with the flip) */
	if (f->scan) {
		uint32_t last = f->surf != NULL ? f->surf->gpuFence : 0;
		if (last != 0 && (int32_t)(last - f->fence) > 0) {
			sweet_fbs.late_draw++;          /* drawn by a submission made after its SwapEnd */
		}
		planeScaled(f->pa, f->stride, &f->geo);
		fPlaneOX = f->ox;                       /* 0354 */
		fPlaneOY = f->oy;
		memcpy(fScanShow, f->held, sizeof(fScanShow));
		fScanShowSwap = f->swap;        /* 0353 */
		bzero(f->held, sizeof(f->held));
		sweet_fbs.flips++;
	} else {
		scanFrom(f->pa, fRowBytes);
	}
	sweet_fbd_flip(fIntf1);
	queueFront(f->pa);
	sweet_fbq.flips++;
}

/*
 * A frame through the scanout (fPresentLock held, the queue on; from SwapEnd, or from the timer for its own frames).
 * Returns false, having changed nothing, for a frame it does not take: then it goes the copy's way.
 */
bool
AppleCLCD::presentScanout(const uint8_t *rec, SweetSurface **held, bool *queued)
{
	uint32_t mask = get32(rec, 0x104), layer = SWEET_FB_LAYERS, n = 0;

	for (uint32_t i = 0; i < SWEET_FB_LAYERS; i++) {
		if ((mask & (1u << i)) && get32(rec, 0x54 + 4 * i) != 0) {
			layer = i;
			n++;
		}
	}
	if (n != 1) {
		sweet_fbs.not_one_layer++;
		return false;
	}
	SweetSurface *own[4] = {};
	SweetSurface *s = held != NULL ? held[layer] : NULL;
	if (s == NULL) {
		if ((s = sweet_surface_copy(get32(rec, 0x54 + 4 * layer))) == NULL) {
			return false;
		}
		own[layer] = s;
	}
	struct SweetGeo g;
	sweet_scan_window(fLogicalW, fLogicalH, fOffX, fOffY, fWidth, fHeight, sweet_fb_rscale, &g.sx, &g.sy, &g.sw, &g.sh);
	bool ok = false;
	uint8_t why = 0;
	if (!s->lowPool || s->scanoutPhys == 0) {
		sweet_fbs.not_pool++;
		why = 1;
	} else if (s->width != fLogicalW || s->height != fLogicalH || s->pixelFormat != SWEET_FB_FMT_BGRA ||
	    s->bytesPerElement != 4 || (s->bytesPerRow & 63) != 0 || s->bytesPerRow > 0xffff || s->offset != 0 || g.sw == 0 ||
	    g.sh == 0 || g.sx + g.sw > s->width || g.sy + g.sh > s->height ||
	    s->scanoutPhys + (uint64_t)s->bytesPerRow * s->height > (1ull << 32)) {
		sweet_fbs.unfit++;
		why = 2;
	} else if (scanoutBlocked()) {
		sweet_fbs.held_off++;                   /* 0354: another surface of the ring cannot be: copied as it is */
	} else if (scanoutBusy(s)) {
		sweet_fbs.reused++;                     /* swapped again while the panel may read it: copied instead */
	} else {
		ok = true;
	}
	if (why != 0) {
		scanoutBlock(s, why);                   /* 0354: the whole ring goes the copy's way while this one lives */
	}
	if (!ok) {
		OSSafeReleaseNULL(own[layer]);
		return false;
	}

	/* as present() at a frame of iOS: the console stops, the boot screen goes; and 0321's ids */
	fConsShown = false;
	splashEnd();
	if (!fConsolePaused) {
		int quiet = 0;
		fConsolePaused = true;
		if (PE_parse_boot_argn("sweetfb_noconsole", &quiet, sizeof(quiet)) && quiet != 0) {
			sweet_console_muted = 1;
		}
	}
	{
		bool known = false;
		for (unsigned i = 0; i < 4; i++) {
			known |= sweet_fb_ids[i] == s->id;
		}
		if (!known) {
			sweet_fb_ids[sweet_fb_ids_next++ & 3] = s->id;
		}
	}
	if (fFlipWait) {
		/* the queue was just turned on: the copy the old path left pending is the queue's first frame (as presentQueued()) */
		if (fQCount < SWEET_FB_BUFS) {
			struct SweetQFrame *f = &fQ[fQCount++];
			bzero(f, sizeof(*f));
			f->pa = fFlipPhys[fFlipIdx ^ 1];
			f->fence = sapphire_gpu_copy_fence();
			f->queued = sweet_fbd.pend_swap;
			sweet_fbq.adopted++;
		}
		fFlipWait = false;
		sweet_fbd.pend_swap = 0;
	}
	queueFlip();

	/* as many frames waiting as kern.sweet_fb_queue_depth allows: until the oldest is flipped, but not for long */
	uint32_t depth = sweet_fb_queue_depth < 1 ? 1 : sweet_fb_queue_depth > SWEET_FB_BUFS - 1 ? SWEET_FB_BUFS - 1 :
	    (uint32_t)sweet_fb_queue_depth;
	if (fQCount >= depth) {
		uint64_t t0 = mach_absolute_time(), deadline;
		int ms = sweet_fb_queue_hold_ms;
		if (ms > 0) {
			clock_interval_to_deadline((uint32_t)(ms > 200 ? 200 : ms), kMillisecondScale, &deadline);
			while (fQCount >= depth) {
				IOLockWakeup(fPresentLock, &fQThread, true);
				int result = IOLockSleepDeadline(fPresentLock, &fQCount, deadline, THREAD_UNINT);
				queueFlip();
				if (result == THREAD_TIMED_OUT) {
					break;
				}
			}
		}
		uint64_t us = sweet_fbd_us(t0, mach_absolute_time());
		sweet_fbs.holds++;
		sweet_fbs.hold_us += us;
		if (us > sweet_fbs.hold_max_us) {
			sweet_fbs.hold_max_us = us;
		}
		if (fQCount >= depth) {
			/* still none: the newest waiting frame gives up its place to this one and is never shown */
			struct SweetQFrame *f = &fQ[--fQCount];
			sweet_hold_set(f->held, false);
			sweet_fbd.overwritten++;
			sweet_fbs.timeouts++;
		}
		if (!scanoutOn() || scanoutBusy(s)) {
			/* (turned off, or the surface taken, while this waited: the copy's way after all) */
			OSSafeReleaseNULL(own[layer]);
			return false;
		}
	}
	if (own[layer] != NULL) {
		sweet_hold(s, true);                    /* (the timer's own frame: no SwapEnd holds it) */
	}
	struct SweetQFrame *f = &fQ[fQCount++];
	bzero(f, sizeof(*f));
	f->scan = true;
	f->surf = s;
	/*
	 * 0354: the window's origin goes into the source address -- whole rows, and whole 16 bytes of the row (the MDP
	 * fetches from a 16-byte aligned address) -- and only what is left into SRC_XY: with the panel's window at 16, 12 of
	 * the 2x picture (fOffX, start()), nothing. The copy crops the same way (blit_scale(), SapphireGpu.cpp).
	 */
		f->ubwc = s->ubwc;
		if (s->ubwc) {
			/* UBWC crops stay in SRC_XY; the compressed data is not row-addressable. */
			f->pa = s->scanoutPhys + s->ubwcDataOffset;
			f->metaPa = s->scanoutPhys + s->ubwcMetaOffset;
			f->metaStride = s->ubwcMetaStride;
			f->ox = g.sx;
			f->oy = g.sy;
		} else {
			uint32_t ax = g.sx & ~3u;
			f->pa = s->scanoutPhys + (uint64_t)g.sy * s->bytesPerRow + (uint64_t)ax * 4;
			f->ox = g.sx;
			f->oy = g.sy;
			g.sx -= ax;
			g.sy = 0;
		}
		f->stride = s->ubwc ? s->ubwcDataStride : s->bytesPerRow;
	f->geo = g;
	f->fence = sapphire_gpu_msm_fence();
	f->queued = mach_absolute_time();
	f->arrived = fQArrived;
	f->swap = get32(rec, 0x50);             /* 0353 */
	memcpy(f->held, own[layer] != NULL ? own : held, sizeof(f->held));
	*queued = true;
	sweet_fbs.frames++;
	sweet_fbq.frames++;
	if (fQCount > sweet_fbq.depth_max) {
		sweet_fbq.depth_max = fQCount;
	}
	queueFlip();                                    /* (drawn already, and the plane writable now: flipped at once) */
	IOLockWakeup(fPresentLock, &fQThread, true);
	return true;
}

/*
 * kern.sweet_fb_scanout set to something else than 1: if the plane shows a surface, or a frame of one waits, the latest
 * swap is presented once more the copy's way, so that the panel goes back to a flip buffer now and not at the window
 * server's next frame (fPresentLock taken here; from the sysctl).
 */
void
AppleCLCD::scanoutStop(void)
{
	uint8_t *rec = (uint8_t *)IOMallocData(SWEET_FB_SWAP_REC_SIZE);
	if (rec == NULL || fPresentLock == NULL) {
		if (rec != NULL) {
			IOFreeData(rec, SWEET_FB_SWAP_REC_SIZE);
		}
		return;
	}
	IOLockLock(fLock);
	memcpy(rec, fPending, SWEET_FB_SWAP_REC_SIZE);
	bool off = fPowerOff;
	IOLockUnlock(fLock);
	IOLockLock(fPresentLock);
	bool scanning = fPlaneScaled;
	for (uint32_t q = 0; q < fQCount; q++) {
		scanning |= fQ[q].scan;
	}
	uint8_t *one = NULL;
	const uint8_t *r = fScale2 ? shownRecord(rec, &one) : rec;     /* 0354: as presentNow() showed it */
	if (scanning && !off && r != NULL) {
		SweetSurface *none[4] = {};
		bool queued = false;
		if (queueOn()) {
			(void)presentQueued(r, none, &queued);
		} else {
			queueDrain();
			(void)present(r);
		}
		sweet_fbs.stops++;
	}
	IOLockUnlock(fPresentLock);
	if (one != NULL) {
		IOFreeData(one, SWEET_FB_SWAP_REC_SIZE);
	}
	IOFreeData(rec, SWEET_FB_SWAP_REC_SIZE);
}

/*
 * 0354: the record a 2x frame is shown by. The 2x path shows one full-screen layer: the GPU's scaled copy (present())
 * and the scanout take nothing else, and flipping the back buffer for a frame they did not take would show an older
 * frame for a refresh. A frame of one layer is shown as it is: rec. One of several whose layer 0 is the window
 * server's full-screen surface is shown by that layer alone, the others left out, from a copy of the record with only
 * layer 0 in its mask (*one, IOMallocData(SWEET_FB_SWAP_REC_SIZE), the caller frees it): the panel moves on and the plane
 * keeps its mode. One of no layer, or without such a layer 0, is not shown (NULL): the panel keeps its frame. The first
 * frames of several layers are logged, with what their layers bring. fPresentLock held (the log's count).
 */
const uint8_t *
AppleCLCD::shownRecord(const uint8_t *rec, uint8_t **one)
{
	uint32_t mask = get32(rec, 0x104), nl = 0;
	static uint32_t layers_told;

	*one = NULL;
	for (uint32_t i = 0; i < SWEET_FB_LAYERS; i++) {
		if ((mask & (1u << i)) && get32(rec, 0x54 + 4 * i) != 0) {
			nl++;
		}
	}
	if (nl == 1) {
		return rec;
	}
	if (nl == 0) {
		return NULL;
	}
	SweetSurface *b = (mask & 1u) != 0 ? sweet_surface_copy(get32(rec, 0x54)) : NULL;
	bool full = b != NULL && b->width == fLogicalW && b->height == fLogicalH;
	OSSafeReleaseNULL(b);
	if (layers_told < 8) {
		layers_told++;
		for (uint32_t i = 0; i < SWEET_FB_LAYERS; i++) {
			if ((mask & (1u << i)) && get32(rec, 0x54 + 4 * i) != 0) {
				const uint8_t *a = rec + 0x64 + 16 * i, *d = rec + 0xc4 + 16 * i;
				IOLog("SWEET FB: 0354: swap %u of %u layers: layer %u surface %u, %u,%u %ux%u onto %u,%u %ux%u%s\n",
				    get32(rec, 0x50), nl, i, get32(rec, 0x54 + 4 * i), get32(a, 0), get32(a, 4), get32(a, 8),
				    get32(a, 12), get32(d, 0), get32(d, 4), get32(d, 8), get32(d, 12),
				    i == 0 && full ? " (shown alone)" : "");
			}
		}
	}
	if (!full || (*one = (uint8_t *)IOMallocData(SWEET_FB_SWAP_REC_SIZE)) == NULL) {
		return NULL;
	}
	uint32_t m = 1;
	memcpy(*one, rec, SWEET_FB_SWAP_REC_SIZE);
	memcpy(*one + 0x104, &m, sizeof(m));
	sweet_fbs.base_only++;
	return *one;
}

/*
 * 0353: whether the frame of swap id has reached the panel (id 0: the frame of every swap so far): no frame of it or of an
 * earlier swap waits in the queue, the MDP has taken the flip written for the last of them, and the swap held back for
 * backboardd's GL bridge (0286, held: its id, 0 if none) is not one of them. A swap whose frame was given up counts once
 * a later one is on the panel. fPresentLock held.
 */
bool
AppleCLCD::swapShown(uint32_t id, uint32_t held)
{
	if (fQLatchWait && fFlipSwap != 0 && (id == 0 || (int32_t)(fFlipSwap - id) <= 0)) {
		return false;
	}
	for (uint32_t k = 0; k < fQCount; k++) {
		if (id == 0 || (fQ[k].swap != 0 && (int32_t)(fQ[k].swap - id) <= 0)) {
			return false;
		}
	}
	return held == 0 || (id != 0 && (int32_t)(held - id) > 0);
}

/*
 * 0353: whether the surfaces of swap id are no longer used, as scanoutBusy() says it of a surface: no frame of it to be
 * scanned out waits in the queue, is on the panel or is being left (fScanShow, fScanLeave), and it is not the held swap.
 * A frame the GPU copies does not count: the copy runs on the GPU's one ring before anything drawn into the surface after
 * its SwapEnd (0201). With kern.sweet_fb_swapwait=2, also the MDP has taken the flip of a later swap. fPresentLock held.
 */
bool
AppleCLCD::swapFree(uint32_t id, uint32_t held)
{
	for (uint32_t k = 0; k < fQCount; k++) {
		if (fQ[k].swap == id && fQ[k].scan) {
			return false;
		}
	}
	if (fScanShowSwap == id || held == id) {
		return false;
	}
	for (uint32_t f = 0; f < 4; f++) {
		if (fScanLeaveSwap[f] == id) {
			return false;
		}
	}
	return sweet_fb_swapwait != 2 || (int32_t)(fPanelSwap - id) > 0;
}

/*
 * 0353: SwapWait(id, flags) from the user client (no lock held; held: the GL bridge's held swap, read under fLock). See
 * sweet_fb_swapwait at the top. The queue's state is brought up to date first (queueFlip(): a flip the MDP has taken
 * since the queue's thread last looked, the frames that may go now), so that the answer is the panel's as of now.
 */
IOReturn
AppleCLCD::swapWait(uint32_t id, uint32_t flags, uint32_t held)
{
	uint32_t kind = (flags & 2) != 0 ? SWEET_FBW_ALL : (flags & 1) != 0 ? SWEET_FBW_SHOWN : SWEET_FBW_FREE;
	bool now = (flags & 0x80000000u) != 0, done = true;

	sweet_fbw.asked[kind]++;
	if (sweet_fb_swapwait == 0 || fPresentLock == NULL || (kind != SWEET_FBW_ALL && id == 0)) {
		return kIOReturnSuccess;
	}
	IOLockLock(fPresentLock);
	if (!queueOn()) {
		IOLockUnlock(fPresentLock);
		return kIOReturnSuccess;
	}
	uint64_t t0 = mach_absolute_time(), t = t0, deadline = 0, step;
	for (;;) {
		queueFlip();
		done = kind == SWEET_FBW_FREE ? swapFree(id, held) : swapShown(kind == SWEET_FBW_ALL ? 0 : id, held);
		t = mach_absolute_time();
		if (done || now) {
			break;
		}
		if (deadline == 0) {
			clock_interval_to_deadline(100, kMillisecondScale, &deadline);
		}
		if (t >= deadline) {
			sweet_fbw.timeouts++;
			break;
		}
		clock_interval_to_deadline(1, kMillisecondScale, &step);
		IOLockWakeup(fPresentLock, &fQThread, true);    /* (the thread looks again, and wakes this up when it flips) */
		(void)IOLockSleepDeadline(fPresentLock, &fQCount, step < deadline ? step : deadline, THREAD_UNINT);
	}
	if (!now) {
		uint64_t us = sweet_fbd_us(t0, t);
		if (kind == SWEET_FBW_ALL) {
			/* (the next SwapEnd waits only if a frame of a surface may be on the panel: a copy's is in a flip buffer) */
			bool scan = fScanShowSwap != 0;
			for (uint32_t k = 0; k < 4; k++) {
				scan |= fScanLeaveSwap[k] != 0 || (k < fQCount && fQ[k].scan);
			}
			fSwapHoldNext = scan;
		}
		sweet_fbw.waits++;
		sweet_fbw.wait_us += us;
		if (us > sweet_fbw.wait_max_us) {
			sweet_fbw.wait_max_us = us;
		}
	} else if (done) {
		fSwapBusyId[kind] = 0;
	} else if (fSwapBusyId[kind] != (id != 0 ? id : ~0u)) {
		fSwapBusyId[kind] = id != 0 ? id : ~0u;   /* (~0: every swap, id 0) */
		fSwapBusySince[kind] = t;
		sweet_fbw.busy[kind]++;
	} else {
		uint64_t us = sweet_fbd_us(fSwapBusySince[kind], t);
		sweet_fbw.busy[kind]++;
		if (us > sweet_fbw.busy_max_us) {
			sweet_fbw.busy_max_us = us;
		}
		if (us >= 1000000) {
			done = true;                    /* (not a frame's wait: answered 0, and counted) */
			fSwapBusyId[kind] = 0;
			sweet_fbw.forced++;
		}
	}
	IOLockUnlock(fPresentLock);
	return done ? kIOReturnSuccess : now ? kIOReturnBusy : kIOReturnTimeout;
}

/*
 * 0353: the first SwapEnd after a wait for every swap that found a frame of a surface on the panel (fSwapHoldNext; see
 * sweet_fb_swapwait at the top) returns once its own frame is on the panel, 250 ms at most: the surface on the panel
 * before it, whose page CoreAnimation no longer asks about, has left the panel when the frame after it is drawn. From
 * swap(), after presentNow(): SwapEnd's thread, no lock held.
 */
void
AppleCLCD::swapEndHold(uint32_t id)
{
	if (fPresentLock == NULL) {
		return;
	}
	IOLockLock(fPresentLock);
	bool hold = fSwapHoldNext && sweet_fb_swapwait != 0 && id != 0 && queueOn();
	fSwapHoldNext = false;
	if (!hold) {
		IOLockUnlock(fPresentLock);
		return;
	}
	uint64_t t0 = mach_absolute_time(), t = t0, deadline, step;
	clock_interval_to_deadline(250, kMillisecondScale, &deadline);         /* 0354: (a first frame can take 200 ms) */
	for (;;) {
		queueFlip();
		t = mach_absolute_time();
		if (swapShown(id, 0)) {
			break;
		}
		if (t >= deadline) {
			sweet_fbw.hold_timeouts++;
			break;
		}
		clock_interval_to_deadline(1, kMillisecondScale, &step);
		IOLockWakeup(fPresentLock, &fQThread, true);
		(void)IOLockSleepDeadline(fPresentLock, &fQCount, step < deadline ? step : deadline, THREAD_UNINT);
	}
	uint64_t us = sweet_fbd_us(t0, t);
	sweet_fbw.holds++;
	sweet_fbw.hold_us += us;
	if (us > sweet_fbw.hold_max_us) {
		sweet_fbw.hold_max_us = us;
	}
	IOLockUnlock(fPresentLock);
}

bool
IOMobileFramebufferUserClient::initWithTask(task_t owningTask, void *securityID, UInt32 type)
{
	if (!IOUserClient::initWithTask(owningTask, securityID, type)) {
		return false;
	}
	fTask = owningTask;
	return true;
}

bool
IOMobileFramebufferUserClient::start(IOService *provider)
{
	fFB = OSDynamicCast(AppleCLCD, provider);
	if (fFB == NULL || !IOUserClient::start(provider)) {
		return false;
	}
	fFB->addClient(this);
	return true;
}

IOReturn
IOMobileFramebufferUserClient::clientClose(void)
{
	if (fFB != NULL) {
		fFB->removeClient(this);
		IOLockLock(fFB->fLock);
		mach_port_t old = fPort;
		fPort = MACH_PORT_NULL;
		IOLockUnlock(fFB->fLock);
		if (old != MACH_PORT_NULL) {
			releaseNotificationPort(old);
		}
	}
	terminate();
	return kIOReturnSuccess;
}

IOReturn
IOMobileFramebufferUserClient::registerNotificationPort(mach_port_t port, __unused UInt32 type,
    __unused io_user_reference_t refCon)
{
	mach_port_t old;
	IOLockLock(fFB->fLock);
	old = fPort;
	fPort = port;
	IOLockUnlock(fFB->fLock);
	if (old != MACH_PORT_NULL) {
		releaseNotificationPort(old);
	}
	return kIOReturnSuccess;
}

/* Called with the framebuffer lock held. */
void
IOMobileFramebufferUserClient::notify(uint32_t type, const uint64_t *values, uint32_t count)
{
	OSAsyncReference64 ref;

	if (type >= SWEET_FB_NOTIFY_TYPES || fFunc[type] == 0 || fPort == MACH_PORT_NULL) {
		return;
	}
	bzero(ref, sizeof(ref));
	setAsyncReference64(ref, fPort, (mach_vm_address_t)fFunc[type], fRefcon[type], fTask);
	sendAsyncResult64WithOptions(ref, kIOReturnSuccess, (io_user_reference_t *)values, count,
	    kIOUserNotifyOptionCanDrop);
}

static uint64_t sweet_fb_logged;        /* selectors already logged (< 64) */
static void sweet_fb_frame(AppleCLCD *fb, bool sent, uint32_t sid);     /* 0286 */

IOReturn
IOMobileFramebufferUserClient::externalMethod(uint32_t selector, IOExternalMethodArguments *args,
    __unused IOExternalMethodDispatch *dispatch, __unused OSObject *target,
    __unused void *reference)
{
	IOReturn ret = kIOReturnSuccess;
	uint64_t out[2] = { 0, 0 };
	uint32_t outCount = 0;

	switch (selector) {
	case kSweetFBGetLayerDefaultSurface:
		out[0] = sweet_surface_port_in_task(fFB->fSurface, fTask);
		outCount = 1;
		break;
	case kSweetFBSwapBegin:
		IOLockLock(fFB->fLock);
		out[0] = ++fFB->fSwapID;
		IOLockUnlock(fFB->fLock);
		outCount = 1;
		break;
	case kSweetFBSwapEnd: {
		uint8_t rec[SWEET_FB_SWAP_REC_SIZE];
		if (args->structureInputDescriptor != NULL ||
		    args->structureInputSize < SWEET_FB_SWAP_REC_SIZE) {
			ret = kIOReturnBadArgument;
			break;
		}
		memcpy(rec, args->structureInput, sizeof(rec));
		fFB->swap(rec);
		break;
	}
	case kSweetFBGetID:
		out[0] = 0;
		outCount = 1;
		break;
	case kSweetFBGetDisplaySize:
		out[0] = fFB->fLogicalW;
		out[1] = fFB->fLogicalH;
		outCount = 2;
		break;
	case kSweetFBIsMainDisplay:
		out[0] = 1;
		outCount = 1;
		break;
	case kSweetFBGetDotPitch:
		out[0] = SWEET_FB_DOT_PITCH;
		outCount = 1;
		break;
	case kSweetFBGetBlock:
		/*
		 * Block 0x40, the current mode: its colour and timing identifiers (publishModes). Zeros
		 * here left CoreAnimation without a mode, and its frame clock without the rate.
		 */
		if (args->structureOutput != NULL && args->structureOutputDescriptor == NULL) {
			bzero(args->structureOutput, args->structureOutputSize);
			if (args->scalarInputCount >= 1 && args->scalarInput[0] == 0x40 && args->structureOutputSize >= 12) {
				uint32_t ids[2] = { SWEET_FB_COLOR_ID, SWEET_FB_TIMING_ID };
				memcpy((uint8_t *)args->structureOutput + 4, ids, sizeof(ids));
			}
			return kIOReturnSuccess;
		}
		break;
	case kSweetFBNotifications:
		if (args->scalarInputCount >= 3 && args->scalarInput[2] < SWEET_FB_NOTIFY_TYPES) {
			uint32_t type = (uint32_t)args->scalarInput[2];
			IOLockLock(fFB->fLock);
			fFunc[type] = args->scalarInput[0];
			fRefcon[type] = args->scalarInput[1];
			IOLockUnlock(fFB->fLock);
			char name[32];
			proc_selfname(name, sizeof(name));
			IOLog("SWEET FB: %s notifications type %u %s\n", name, type,
			    args->scalarInput[0] ? "on" : "off");
		}
		break;
	case kSweetFBRequestPowerChange:
		if (args->scalarInputCount >= 1) {
			bool off = args->scalarInput[0] == 0;
			/*
			 * topaz: sweetfb_stayon=1 ignores display-off requests (auto-lock)
			 * while there are no buttons or touch to wake the panel again.
			 */
			int stayon = 0;
			if (off && PE_parse_boot_argn("sweetfb_stayon", &stayon, sizeof(stayon)) && stayon) {
				IOLog("SWEET FB: display off ignored (sweetfb_stayon)\n");
				break;
			}
			/*
			 * sapphire 0234: backboardd asks for display off right after the Apple logo, long
			 * before the UI can ask for it on again, and the panel stayed black (sweetfb_stayon=1
			 * worked around it, but then the panel never went off). An off before the first on,
			 * in the first 2 minutes, is that one: ignored. Later ones (auto-lock, the power
			 * button) turn the panel off as asked.
			 */
			/*
			 * sapphire 0236: the boot log of 0235 shows the order: backboardd asks for on, draws a
			 * few frames, asks for off (the panel went black there, logo gone), and SpringBoard asks
			 * for on again once the lock screen is ready. So every off before the second on, in the
			 * first 3 minutes, belongs to the boot: ignored.
			 */
			uint64_t since_ns;
			absolutetime_to_nanoseconds(mach_absolute_time() - fFB->fStartAbs, &since_ns);
			if (off && fFB->fPowerOns < 2 && since_ns < 180ull * NSEC_PER_SEC) {
				IOLog("SWEET FB: display off ignored (boot, %llu s, %u on)\n", since_ns / NSEC_PER_SEC, fFB->fPowerOns);
				break;
			}
			if (!off && fFB->fPowerOns < 2) {
				fFB->fPowerOns++;
			}
			IOLockLock(fFB->fLock);
			bool changed = off != fFB->fPowerOff;
			if (changed) {
				fFB->fPowerOff = off;
				sweet_fb_panel_off_abs = mach_absolute_time();  /* 0324: tap to wake */
				sweet_fb_panel_off = off ? 1 : 0;
				fFB->fFadeStep = 0;
				if (off) {
					fFB->fDirty = true;
					fFB->fWakeTicks = 0;
				} else {
					/*
					 * 0295: "on" showed the latest frame again at once -- the frame from before
					 * "off" (the app or home screen that was up), then the lock screen when
					 * backboardd's first new frame came a moment later: a flash of old content at
					 * every wake. The panel keeps the black it scans since "off" until a new frame
					 * is presented; only if none comes in 0.3 s is the old one shown.
					 */
					fFB->fDirty = false;
					fFB->fWakeTicks = fFB->fRateHz * 2 + 1;   /* 0326: 2 s (0.3 s ran out before Metal backboardd sent its first frame after wake: the old frame flashed, then the lock screen) */
				}
			}
			uint32_t level = fFB->fBlLevel;
			IOLockUnlock(fFB->fLock);
			if (changed && fFB->fDsi != NULL) {
				if (off) {
					fFB->setPanelOn(false);
				} else {
					fFB->setPanelOn(true);
					fFB->fBlSent = level;
					fFB->setPanelLevel(level);
				}
			}
			IOLog("SWEET FB: display %s\n", off ? "off" : "on");
		}
		break;
	case 61:
		/*
		 * 0286: [0x53570286, 0 (await) or 1 (sent), frame number] from backboardd's GL bridge, whose
		 * sandbox writes no sysctl (kern.sweet_fb_frame_await/_sent do the same for root).
		 */
		if (args->scalarInputCount == 3 && args->scalarInput[0] == 0x53570286) {
			sweet_fb_frame(fFB, args->scalarInput[1] != 0, (uint32_t)args->scalarInput[2]);
		}
		break;
	case kSweetFBSwapWait:
		sweet_fbd.swap_waits++;         /* 0336: how often the window server asks to wait for a swap */
		if (args->scalarInputCount >= 2) {
			/* 0353: {id, flags, timeout}: answered from the frame queue (sweet_fb_swapwait) */
			uint32_t held = 0;
			IOLockLock(fFB->fLock);
			if (fFB->fHeldRec != NULL) {
				held = get32(fFB->fHeldRec, 0x50);
			}
			IOLockUnlock(fFB->fLock);
			ret = fFB->swapWait((uint32_t)args->scalarInput[0], (uint32_t)args->scalarInput[1], held);
		}
		break;
	default:
		break;
	}

	for (uint32_t i = 0; i < args->scalarOutputCount; i++) {
		args->scalarOutput[i] = i < outCount ? out[i] : 0;
	}
	if (args->structureOutputDescriptor == NULL && args->structureOutput != NULL) {
		bzero(args->structureOutput, args->structureOutputSize);
	}
	if (selector >= 64 || !(sweet_fb_logged & (1ULL << selector))) {
		if (selector < 64) {
			sweet_fb_logged |= 1ULL << selector;
		}
		char name[32];
		proc_selfname(name, sizeof(name));
		IOLog("SWEET FB: %s sel %u in %u/%u out %u/%u -> 0x%x\n", name, selector,
		    args->scalarInputCount, args->structureInputSize,
		    args->scalarOutputCount, args->structureOutputSize, ret);
	}
	return ret;
}
/*
 * 0262: DSI commands without leaving video mode, as Linux's dsi_ctrl does for a video-mode panel
 * (dsi_ctrl_hw_cmn_kickoff_command): command DMA in embedded mode (DSI_COMMAND_MODE_DMA_CTRL bit
 * 28, packet type bit 24 and WC_SEL bit 29 clear, bit 26 clear = high speed), the DMA FIFO
 * watermarks off (DSI_DMA_FIFO_CTRL bits 16/20), the software trigger; DSI_CTRL is not touched,
 * so the controller stays in video mode and sends the packets in the next blanking period; if the
 * DMA does not complete that way, the batch is sent once more with CMD_MODE_EN set beside
 * VID_MODE_EN for its duration (Linux's transfer_prepare/unprepare)
 * (the n7 panel has no qcom,mdss-dsi-dma-schedule-line, so no scheduled line either). The
 * registers it changes are put back afterwards. dsiSend() (0260) switched CMD_MODE_EN on for
 * every packet: the screen went black for a moment at each local-HBM change.
 */
#define DSI_CTRL_REG            0x004
#define DSI_DMA_CTRL            0x03c
#define DSI_DMA_OFFSET          0x048
#define DSI_DMA_LENGTH          0x04c
#define DSI_DMA_FIFO_CTRL       0x050
#define DSI_TRIG_CTRL           0x084
#define DSI_DMA_SW_TRIGGER      0x090
#define DSI_INT_CTRL            0x110
#define DSI_BATCH_MAX           2048u

static bool
dsi_video_dma(AppleCLCD *fb, uint32_t size, uint32_t *waited_us, bool cmd_engine)
{
	volatile uint8_t *d = fb->fDsi;
	uint32_t dma = dsi_rd(d, DSI_DMA_CTRL), fifo = dsi_rd(d, DSI_DMA_FIFO_CTRL), trig = dsi_rd(d, DSI_TRIG_CTRL);
	uint32_t ctrl = dsi_rd(d, DSI_CTRL_REG);

	if (cmd_engine && (ctrl & 4) == 0) {
		dsi_wr(d, DSI_CTRL_REG, ctrl | 4);      /* CMD_MODE_EN beside VID_MODE_EN, as Linux's transfer_prepare */
	}

	if ((trig & 7) != 4) {
		dsi_wr(d, DSI_TRIG_CTRL, (trig & ~7u) | 4);     /* DMA trigger: software */
	}
	dsi_wr(d, DSI_DMA_CTRL, (dma | (1u << 28)) & ~((1u << 24) | (1u << 29) | (1u << 26)));
	dsi_wr(d, DSI_DMA_FIFO_CTRL, fifo | (1u << 20) | (1u << 16));
	dsi_wr(d, DSI_INT_CTRL, dsi_rd(d, DSI_INT_CTRL) | 1);       /* clear CMD_DMA_DONE */
	dsi_wr(d, DSI_DMA_OFFSET, (uint32_t)fb->fDsiBufPhys);
	dsi_wr(d, DSI_DMA_LENGTH, size & 0xffffff);
	dsi_wr(d, DSI_DMA_SW_TRIGGER, 1);
	bool done = false;
	uint32_t us = 0;
	while (!done && us < 60000) {                    /* the next blanking: a frame is 8-17 ms */
		done = (dsi_rd(d, DSI_INT_CTRL) & 1) != 0;
		if (!done) {
			IODelay(100);
			us += 100;
		}
	}
	dsi_wr(d, DSI_INT_CTRL, dsi_rd(d, DSI_INT_CTRL) | 1);
	dsi_wr(d, DSI_DMA_FIFO_CTRL, fifo);
	dsi_wr(d, DSI_DMA_CTRL, dma);
	if (cmd_engine && (ctrl & 4) == 0) {
		dsi_wr(d, DSI_CTRL_REG, ctrl);
	}
	if (trig != dsi_rd(d, DSI_TRIG_CTRL)) {
		dsi_wr(d, DSI_TRIG_CTRL, trig);
	}
	*waited_us = us;
	return done;
}

/* one command the 0260 way (command mode for the packet), with retries */
static bool
dsi_one_cmdmode(AppleCLCD *fb, uint8_t dt, const uint8_t *pl, uint32_t dl, bool *known)
{
	bool done = false;
	*known = true;
	for (int i = 0; i < 4 && !done; i++) {
		if (dt == 0x39 || dt == 0x29) {
			uint8_t hdr[4] = { (uint8_t)dl, (uint8_t)(dl >> 8), dt, 0xc0 };
			done = fb->dsiSend(hdr, pl, dl);
		} else if ((dt == 0x05 || dt == 0x15) && dl >= 1 && dl <= 2) {
			uint8_t hdr[4] = { pl[0], (uint8_t)(dl > 1 ? pl[1] : 0), dt, 0x80 };
			done = fb->dsiSend(hdr, NULL, 0);
		} else {
			*known = false;
			return false;
		}
		if (!done) {
			IODelay(2000);
		}
	}
	return done;
}

/*
 * 0260/0262: a panel command sequence for SweetQsee.cpp (the fingerprint spot's local HBM), in the
 * qcom dtsi format: per command {dtype, last, vc, flags, wait ms, len hi, len lo, payload}. In
 * video mode the commands up to the next wait go out as one batch (packets back to back, each
 * 4-byte padded with 0xff, BIT(7) "last" on the final one, BIT(6) on long ones), in one blanking
 * period like Android's batched local-HBM set; if that batch is not done, or the controller is
 * not in video mode, each command goes the 0260 way. Returns the commands not done, -1 without a
 * DSI path, -2 while the panel is off.
 */
extern "C" int
sweet_fb_dsi_seq(const uint8_t *seq, uint32_t len, const char *what, int quiet)
{
	AppleCLCD *fb = sweet_dsi_fb;
	if (fb == NULL) {
		IOLog("SWEET FB: dsi %s: no DSI command path\n", what);
		return -1;
	}
	if (fb->fPowerOff) {
		IOLog("SWEET FB: dsi %s: the panel is off: not sent\n", what);
		return -2;
	}
	int failed = 0;
	unsigned n = 0, batches = 0, fallbacks = 0;
	IOLockLock(fb->fBlLock);
	uint32_t ctrl = dsi_rd(fb->fDsi, DSI_CTRL_REG);
	bool video = (ctrl & 2) != 0;                   /* VID_MODE_EN */
	if (!quiet) {
		IOLog("SWEET FB: dsi %s: ctrl %#x (%s), dma ctrl %#x, fifo %#x, trigger %#x, int %#x\n", what, ctrl,
		    video ? "video mode: batched in the blanking" : "not video mode: one by one", dsi_rd(fb->fDsi, DSI_DMA_CTRL),
		    dsi_rd(fb->fDsi, DSI_DMA_FIFO_CTRL), dsi_rd(fb->fDsi, DSI_TRIG_CTRL), dsi_rd(fb->fDsi, DSI_INT_CTRL));
	}
	uint32_t o = 0;
	while (o + 7 <= len) {
		/* the batch: from o up to and including the first command with a wait (or the end) */
		uint32_t e = o, cnt = 0, size = 0;
		bool ok = true;
		uint8_t wait = 0;
		while (e + 7 <= len) {
			uint8_t dt = seq[e];
			uint32_t dl = ((uint32_t)seq[e + 5] << 8) | seq[e + 6];
			if (e + 7 + dl > len || dl > 60 || !(dt == 0x39 || dt == 0x29 || ((dt == 0x05 || dt == 0x15) && dl >= 1 && dl <= 2))) {
				ok = false;
				break;
			}
			uint32_t psize = dt == 0x39 || dt == 0x29 ? 4 + ((dl + 3) & ~3u) : 4;
			if (size + psize > DSI_BATCH_MAX) {
				break;
			}
			size += psize;
			cnt++;
			wait = seq[e + 4];
			e += 7 + dl;
			if (wait != 0) {
				break;
			}
		}
		bool sent = false;
		if (video && ok && cnt > 0) {
			uint8_t *p = (uint8_t *)fb->fDsiBuf->getBytesNoCopy();
			uint32_t at = 0, i = 0;
			memset(p, 0xff, size);
			for (uint32_t c = o; c < e; i++) {
				uint8_t dt = seq[c];
				uint32_t dl = ((uint32_t)seq[c + 5] << 8) | seq[c + 6];
				const uint8_t *pl = seq + c + 7;
				uint8_t flags = (i + 1 == cnt) ? 0x80 : 0;
				if (dt == 0x39 || dt == 0x29) {
					p[at] = (uint8_t)dl;
					p[at + 1] = (uint8_t)(dl >> 8);
					p[at + 2] = dt;
					p[at + 3] = flags | 0x40;
					memcpy(p + at + 4, pl, dl);
					at += 4 + ((dl + 3) & ~3u);
				} else {
					p[at] = pl[0];
					p[at + 1] = dl > 1 ? pl[1] : 0;
					p[at + 2] = dt;
					p[at + 3] = flags;
					at += 4;
				}
				c += 7 + dl;
			}
			sweet_dcache_clean((vm_offset_t)(uintptr_t)p, size);
			/* first with the command engine left off; if the DMA never completes, once more with it on */
			uint32_t us = 0, us2 = 0;
			sent = dsi_video_dma(fb, size, &us, false);
			bool engine = false;
			if (!sent) {
				engine = true;
				sent = dsi_video_dma(fb, size, &us2, true);
			}
			batches++;
			if (!quiet || !sent || engine) {
				IOLog("SWEET FB: dsi %s: batch of %u commands (%u bytes) in video mode: %s (%u us; %s)\n", what, cnt, size,
				    sent ? "done" : "NOT DONE, sent one by one instead", us + us2,
				    engine ? "not done with the command engine off: retried with it on" : "command engine left off");
			}
			n += sent ? cnt : 0;
		}
		if (!sent) {
			/* the 0260 way, command by command (also for anything the batch cannot carry) */
			uint32_t stop = ok && cnt > 0 ? e : len;
			fallbacks++;
			for (uint32_t c = o; c + 7 <= stop;) {
				uint8_t dt = seq[c], w = seq[c + 4];
				uint32_t dl = ((uint32_t)seq[c + 5] << 8) | seq[c + 6];
				if (c + 7 + dl > len || dl > 60) {
					IOLog("SWEET FB: dsi %s: bad command at %u (len %u): stop\n", what, c, dl);
					failed++;
					e = len;
					break;
				}
				bool known;
				bool done = dsi_one_cmdmode(fb, dt, seq + c + 7, dl, &known);
				if (!quiet || !done) {
					IOLog("SWEET FB: dsi %s #%u: dt %02x len %u: %02x %02x: %s\n", what, n, dt, dl, dl > 0 ? seq[c + 7] : 0,
					    dl > 1 ? seq[c + 8] : 0, !known ? "UNKNOWN TYPE, skipped" : done ? "done" : "NO DONE");
				}
				failed += done ? 0 : 1;
				n++;
				if (w != 0 && c + 7 + dl < stop) {
					IOSleep(w);
				}
				c += 7 + dl;
				e = c;
			}
		}
		if (wait != 0) {
			IOSleep(wait);
		}
		o = e;
	}
	IOLockUnlock(fb->fBlLock);
	if (!quiet || failed != 0) {
		IOLog("SWEET FB: dsi %s: %u commands, %d not done (%u video batches, %u sent one by one)\n", what, n, failed,
		    batches, fallbacks);
	}
	return failed;
}
#endif /* SWEET */

/*
 * 0286: kern.sweet_fb_frame_await = N: frame N (the bridge's numbers, not 0) is being sent to the
 * GPU; kern.sweet_fb_frame_sent = N: it has been (Mesa's driver thread, after the submission).
 * kern.sweet_fb_frame_stats: swaps held, frames reported sent, held swaps that went out late.
 */
#include <sys/sysctl.h>

static void
sweet_fb_frame(AppleCLCD *fb, bool sent, uint32_t sid)
{
	if (fb == NULL || fb->fLock == NULL) {
		return;
	}
	IOLockLock(fb->fLock);
	if (!sent) {
		fb->fAwaitSid = sid;
		fb->fAwaitSent = false;
		IOLockUnlock(fb->fLock);
		return;
	}
	fb->fSentCount++;
	bool held = fb->fHeldRec != NULL && fb->fHeldSeq == sid;
	if (!held && fb->fAwaitSid == sid) {
		fb->fAwaitSent = true;
	}
	IOLockUnlock(fb->fLock);
	if (held) {
		fb->presentHeld(false);
	}
}

static int
sweet_fb_frame_sysctl SYSCTL_HANDLER_ARGS
{
#pragma unused(arg2)
	int sid = 0;
	int error = sysctl_handle_int(oidp, &sid, 0, req);

	if (error == 0 && req->newptr != USER_ADDR_NULL) {
		sweet_fb_frame(sweet_frame_fb, arg1 != NULL, (uint32_t)sid);
	}
	return error;
}
SYSCTL_INT(_kern, OID_AUTO, sweet_fb_rscale, CTLFLAG_RW | CTLFLAG_LOCKED, &sweet_fb_rscale, 0,
    "sapphire 0321: permille of the 2x surface the GPU driver draws (400-1000)");
static int
sweet_fb_ids_sysctl SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	uint32_t ids[4];
	memcpy(ids, sweet_fb_ids, sizeof(ids));
	return SYSCTL_OUT(req, ids, sizeof(ids));
}
SYSCTL_PROC(_kern, OID_AUTO, sweet_fb_ids, CTLTYPE_OPAQUE | CTLFLAG_RD | CTLFLAG_ANYBODY | CTLFLAG_LOCKED,
    NULL, 0, sweet_fb_ids_sysctl, "S", "0321: the IOSurface ids swapped onto the panel lately");
SYSCTL_INT(_kern, OID_AUTO, sweet_fb_flipflush, CTLFLAG_RW | CTLFLAG_LOCKED, &sweet_fb_flipflush, 0,
    "sapphire 0304: 1 = flush the whole flip buffer after each 2x GPU copy (as before)");
SYSCTL_PROC(_kern, OID_AUTO, sweet_fb_frame_await, CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_ANYBODY | CTLFLAG_LOCKED,
    NULL, 0, sweet_fb_frame_sysctl, "I", "a frame of this surface is being sent to the GPU");
SYSCTL_PROC(_kern, OID_AUTO, sweet_fb_frame_sent, CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_ANYBODY | CTLFLAG_LOCKED,
    (void *)1, 0, sweet_fb_frame_sysctl, "I", "the frame of this surface has been sent to the GPU");

static int
sweet_fb_frame_stats SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	char line[160];
	AppleCLCD *fb = sweet_frame_fb;

	snprintf(line, sizeof(line), "held %llu sent %llu late %llu", fb ? fb->fHeldCount : 0, fb ? fb->fSentCount : 0,
	    fb ? fb->fHeldLate : 0);
	return SYSCTL_OUT(req, line, strlen(line) + 1);
}
SYSCTL_PROC(_kern, OID_AUTO, sweet_fb_frame_stats, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED,
    NULL, 0, sweet_fb_frame_stats, "A", "0286: swaps held until their frame was sent");

/*
 * sapphire 0336: kern.sweet_fb_diag, the presentation path's counters and histograms (sweet_fbd, at the top). Read
 * only; two readings subtracted give what happened in between.
 */
#define SWEET_FBD_CAT(...) do { \
	if (off + 1 < sizeof(buf)) { \
		int n_ = snprintf(buf + off, sizeof(buf) - off, __VA_ARGS__); \
		if (n_ > 0) { \
			off = off + (size_t)n_ < sizeof(buf) ? off + (size_t)n_ : sizeof(buf) - 1; \
		} \
	} \
} while (0)

static int
sweet_fb_diag_sysctl SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	char buf[2048];
	size_t off = 0;
	static const struct {
		const char *name;
		const struct sweet_fbd_hist *h;
		unsigned n;
	} hs[] = {
		{ "swap interval (refreshes 1 2 3 4 5 6 7 8 9-10 11-12 13-18 more; us)", &sweet_fbd.swap_iv, 12 },
		{ "flip interval (panel refreshes 0 1 2 3 4 5 6 7 8 9-10 11-12 13-18 more)", &sweet_fbd.flip_iv, 13 },
		{ "swap to flip (ms <2 <4 <8 <12 <16 <24 <32 <48 <64 <100 <200 more; us)", &sweet_fbd.swap_to_flip, 12 },
		{ "tick late (us <50 <100 <200 <400 <800 <1600 <3200 <6400 <12800 more)", &sweet_fbd.tick_late, 10 },
	};

	buf[0] = 0;
	SWEET_FBD_CAT("swaps %llu flips %llu (by swap %llu, by tick %llu) cpu flips %llu overwritten %llu grid restarts %llu swap waits %llu\n",
	    sweet_fbd.swaps, sweet_fbd.flips, sweet_fbd.flips_by_swap, sweet_fbd.flips_by_tick, sweet_fbd.cpu_flips,
	    sweet_fbd.overwritten, sweet_fbd.grid_restarts, sweet_fbd.swap_waits);
	for (unsigned i = 0; i < sizeof(hs) / sizeof(hs[0]); i++) {
		SWEET_FBD_CAT("%s:", hs[i].name);
		for (unsigned k = 0; k < hs[i].n; k++) {
			SWEET_FBD_CAT(" %llu", hs[i].h->n[k]);
		}
		SWEET_FBD_CAT("; count %llu sum %llu max %llu\n", hs[i].h->count, hs[i].h->sum, hs[i].h->max);
	}
	SWEET_FBD_CAT("flip written in eighth of the refresh:");
	for (unsigned k = 0; k < 8; k++) {
		SWEET_FBD_CAT(" %llu", sweet_fbd.flip_eighth[k]);
	}
	SWEET_FBD_CAT("\npresent: %llu, %llu us in all, max %llu us; surface cleans %llu, %llu us in all\n", sweet_fbd.presents,
	    sweet_fbd.present_us, sweet_fbd.present_max_us, sweet_fbd.cleans, sweet_fbd.clean_us);
	SWEET_FBD_CAT("hold (kern.sweet_fb_hold_ms %d): %llu swaps waited for the copy before theirs, %llu us in all\n",
	    sweet_fb_hold_ms, sweet_fbd.holds, sweet_fbd.hold_us);
	return SYSCTL_OUT(req, buf, off + 1);
}
SYSCTL_PROC(_kern, OID_AUTO, sweet_fb_diag, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED,
    NULL, 0, sweet_fb_diag_sysctl, "A", "sapphire 0336: presentation path counters and histograms");
SYSCTL_INT(_kern, OID_AUTO, sweet_fb_hold_ms, CTLFLAG_RW | CTLFLAG_LOCKED, &sweet_fb_hold_ms, 0,
    "sapphire 0338: ms a SwapEnd waits for the GPU's copy of the frame before its own (0 = off, as before)");

/* sapphire 0340: the frame queue's switches and counters (sweet_fbq, at the top). */
static int
sweet_fb_queue_diag_sysctl SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	char buf[1024];
	int n;

	n = snprintf(buf, sizeof(buf), "queue (kern.sweet_fb_queue %d, depth %d, hold %d ms, poll %d us): frames %llu flips %llu (by its thread %llu) "
	    "adopted %llu dropped %llu drained %llu; swaps that waited %llu, %llu us in all, max %llu; "
	    "timed out %llu forced %llu; flips taken by time %llu, written late %llu; polls %llu; depth max %llu\n"
	    "arrival to flip (ms <2 <4 <8 <12 <16 <24 <32 <48 <64 <100 <200 more; us):", sweet_fb_queue, sweet_fb_queue_depth,
	    sweet_fb_queue_hold_ms, sweet_fb_queue_poll_us, sweet_fbq.frames, sweet_fbq.flips, sweet_fbq.flips_thread, sweet_fbq.adopted, sweet_fbq.dropped,
	    sweet_fbq.drained, sweet_fbq.holds, sweet_fbq.hold_us, sweet_fbq.hold_max_us, sweet_fbq.timeouts, sweet_fbq.forced,
	    sweet_fbq.by_time, sweet_fbq.late, sweet_fbq.polls, sweet_fbq.depth_max);
	for (unsigned k = 0; k < 12 && n > 0 && (size_t)n < sizeof(buf); k++) {
		n += snprintf(buf + n, sizeof(buf) - (size_t)n, " %llu", sweet_fbq.lat.n[k]);
	}
	if (n > 0 && (size_t)n < sizeof(buf)) {
		n += snprintf(buf + n, sizeof(buf) - (size_t)n, "; count %llu sum %llu max %llu\n", sweet_fbq.lat.count, sweet_fbq.lat.sum,
		    sweet_fbq.lat.max);
	}
	buf[sizeof(buf) - 1] = 0;
	return SYSCTL_OUT(req, buf, strlen(buf) + 1);
}
SYSCTL_PROC(_kern, OID_AUTO, sweet_fb_queue_diag, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED,
    NULL, 0, sweet_fb_queue_diag_sysctl, "A", "sapphire 0340: the frame queue's counters");
SYSCTL_INT(_kern, OID_AUTO, sweet_fb_queue, CTLFLAG_RW | CTLFLAG_LOCKED, &sweet_fb_queue, 0,
    "sapphire 0340: 1 = frames presented in SwapEnd through a queue of three flip buffers, flipped by a thread as their copies finish (0 = as before)");
SYSCTL_INT(_kern, OID_AUTO, sweet_fb_queue_hold_ms, CTLFLAG_RW | CTLFLAG_LOCKED, &sweet_fb_queue_hold_ms, 0,
    "sapphire 0340: the longest a SwapEnd waits for a free flip buffer, ms (200 at most; 0 = never)");
SYSCTL_INT(_kern, OID_AUTO, sweet_fb_queue_poll_us, CTLFLAG_RW | CTLFLAG_LOCKED, &sweet_fb_queue_poll_us, 0,
    "sapphire 0340: how often the queue's thread looks at a pending copy's fence, us (100 to 8000)");
SYSCTL_INT(_kern, OID_AUTO, sweet_fb_queue_depth, CTLFLAG_RW | CTLFLAG_LOCKED, &sweet_fb_queue_depth, 0,
    "sapphire 0340: how many frames may wait for the panel before a SwapEnd is held (1 or 2)");

/* sapphire 0349: the display engine's registers and the latch of CTL_FLUSH (sweet_fbr, sweet_fbl, at the top). Read only. */
#define SWEET_FBR_BUF           24576

static size_t
sweet_fbr_words(char *buf, size_t off, const char *title, uint32_t pa, const volatile uint32_t *r, const uint32_t *copy,
    uint32_t len)
{
	int n = snprintf(buf + off, SWEET_FBR_BUF - off, "%s (%#x, %#x bytes):\n", title, pa, len);
	off = n > 0 && off + (size_t)n < SWEET_FBR_BUF ? off + (size_t)n : SWEET_FBR_BUF - 1;
	for (uint32_t k = 0; k < len / 4 && off + 96 < SWEET_FBR_BUF; k += 8) {
		n = snprintf(buf + off, SWEET_FBR_BUF - off, " +%03x:", k * 4);
		off += n > 0 ? (size_t)n : 0;
		for (uint32_t j = k; j < k + 8 && j < len / 4; j++) {
			n = snprintf(buf + off, SWEET_FBR_BUF - off, " %08x", copy != NULL ? copy[j] : r[j]);
			off += n > 0 ? (size_t)n : 0;
		}
		n = snprintf(buf + off, SWEET_FBR_BUF - off, "\n");
		off += n > 0 ? (size_t)n : 0;
	}
	return off;
}

static int
sweet_fb_regs_sysctl SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	char *buf = (char *)IOMallocData(SWEET_FBR_BUF);
	size_t off = 0;
	int error;

	if (buf == NULL) {
		return ENOMEM;
	}
	buf[0] = 0;
	for (unsigned i = 0; i < SWEET_FBR_N; i++) {
		if (sweet_fbr_va[i] != NULL) {
			off = sweet_fbr_words(buf, off, sweet_fbr_blocks[i].name, sweet_fbr_blocks[i].pa, sweet_fbr_va[i], NULL,
			    sweet_fbr_blocks[i].len);
		}
	}
	if (sweet_fbr_have_boot) {
		off = sweet_fbr_words(buf, off, "VIG0 at boot", sweet_fbr_blocks[SWEET_FBR_VIG0].pa, NULL, sweet_fbr_boot, 0x1f8);
		off = sweet_fbr_words(buf, off, "VIG0 scaler at boot", sweet_fbr_blocks[SWEET_FBR_VIG0 + 1].pa, NULL, sweet_fbr_boot + 0x1f8 / 4, 0xa0);
	}
	error = SYSCTL_OUT(req, buf, off + 1);
	IOFreeData(buf, SWEET_FBR_BUF);
	return error;
}
SYSCTL_PROC(_kern, OID_AUTO, sweet_fb_regs, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED,
    NULL, 0, sweet_fb_regs_sysctl, "A", "sapphire 0349/0393: the display engine's registers now (MDSS, TOP, CTL0, VIG0 and its scaler and CSC, LM0, PP0, INTF1) and VIG0's at boot");

static int
sweet_fb_latch_sysctl SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	char buf[2048];
	size_t off = 0;

	buf[0] = 0;
	SWEET_FBD_CAT("latch (0349): flips %llu; CTL_FLUSH right after the write: bits set %llu, clear %llu (last %#x); "
	    "still set after the frame counter moved %llu; not clear in 100 ms %llu; looks %llu\n", sweet_fbl.flips,
	    sweet_fbl.readback_set, sweet_fbl.readback_clear, sweet_fbl.last_readback, sweet_fbl.set_after_move,
	    sweet_fbl.gave_up, sweet_fbl.samples);
	SWEET_FBD_CAT("write to clear (ms <1 <2 <4 <8 <12 <17 <34 more; us):");
	for (unsigned k = 0; k <= SWEET_FBL_EDGES; k++) {
		SWEET_FBD_CAT(" %llu", sweet_fbl.clear.n[k]);
	}
	SWEET_FBD_CAT("; count %llu sum %llu max %llu\n", sweet_fbl.clear.count, sweet_fbl.clear.sum, sweet_fbl.clear.max);
	SWEET_FBD_CAT("by sixteenth of the refresh at the write, cleared with the frame counter moved by 0/1/2/3+:");
	for (unsigned k = 0; k < 16; k++) {
		SWEET_FBD_CAT(" %u:%llu/%llu/%llu/%llu", k, sweet_fbl.at[k][0], sweet_fbl.at[k][1], sweet_fbl.at[k][2], sweet_fbl.at[k][3]);
	}
	SWEET_FBD_CAT("\n");
	return SYSCTL_OUT(req, buf, off + 1);
}
SYSCTL_PROC(_kern, OID_AUTO, sweet_fb_latch, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED,
    NULL, 0, sweet_fb_latch_sysctl, "A", "sapphire 0349: whether CTL_FLUSH reads back a flip's bits and when they clear");

/*
 * sapphire 0350: kern.sweet_fb_scanout (0 copies as before, 1 scanout of the window server's surfaces, 2 their pool only;
 * boot-arg sweetfb_scanout) and kern.sweet_fb_scanout_diag (sweet_fbs, at the top; read only).
 */
static int
sweet_fb_scanout_sysctl SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	int v = sweet_fb_scanout;
	int error = sysctl_handle_int(oidp, &v, 0, req);

	if (error != 0 || req->newptr == USER_ADDR_NULL) {
		return error;
	}
	if (v < 0 || v > 2) {
		return EINVAL;
	}
	int was = sweet_fb_scanout;
	if (v == 1 && sweet_frame_fb != NULL) {
		sweet_frame_fb->fScanStuck = false;     /* (a new try after the MDP once did not take a flush) */
	}
	sweet_fb_scanout = v;
	if (was == 1 && v != 1 && sweet_frame_fb != NULL) {
		sweet_frame_fb->scanoutStop();
	}
	return 0;
}
SYSCTL_PROC(_kern, OID_AUTO, sweet_fb_scanout, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED,
    NULL, 0, sweet_fb_scanout_sysctl, "I",
    "sapphire 0350: 1 = the window server's surfaces scanned out by VIG0's scaler (no copy), 2 = their pool only, 0 = as before");

static int
sweet_fb_scanout_diag_sysctl SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	char buf[3072];                 /* 0354: (eight slots described) */
	size_t off = 0;
	AppleCLCD *fb = sweet_frame_fb;
	uint32_t used = 0, slots = 0;
	uint64_t made = 0, refused = 0;

	sweet_scanout_pool_census(&used, &slots, &made, &refused);
	buf[0] = 0;
	SWEET_FBD_CAT("scanout (kern.sweet_fb_scanout %d): %s; plane now %s, scaler %#x; CTL_FLUSH read its bits back right after "
	    "%u of %u flips: %s (kern.sweet_fb_flush_taken %d)\n", sweet_fb_scanout,
	    fb == NULL ? "no framebuffer" : fb->fScanWhy != NULL ? fb->fScanWhy : fb->fScanStuck ? "stopped: a flush was not taken" :
	    "ready", fb != NULL && fb->fPlaneScaled ? "scaled" : "unscaled", fb != NULL ? fb->fScalerVersion : 0,
	    fb != NULL ? fb->fFlushSet : 0, fb != NULL ? fb->fFlushSet + fb->fFlushClear : 0,
	    fb == NULL || !fb->fFlushKnown ? "not known yet" : !fb->fFlushReadback ? "it does not read back" :
	    sweet_fb_flush_taken != 0 || fb->fScanUsed ? "flips are taken by it" : "flips are taken by the frame counter",
	    sweet_fb_flush_taken);
	SWEET_FBD_CAT("pool: %u of %u slots in use, %llu surfaces made in it, %llu refused (full)\n", used, slots, made, refused);
	if (off + 1 < sizeof(buf)) {
		off += sweet_scanout_pool_describe(buf + off, sizeof(buf) - off);  /* 0354: what is in each slot */
	}
	if (fb != NULL) {
		uint32_t nb = 0;
		for (uint32_t i = 0; i < 8; i++) {
			nb += fb->fScanBlock[i].s != NULL;
		}
		SWEET_FBD_CAT("0354: surfaces of the ring that cannot be scanned out, seen %llu, kept now %u (frames copied for them %llu); "
		    "frames of several layers shown by their layer 0 %llu, frames not shown %llu\n", sweet_fbs.blockers, nb,
		    sweet_fbs.held_off, sweet_fbs.base_only, sweet_fbs.not_shown);
	}
	SWEET_FBD_CAT("frames %llu flips %llu; copied instead: not one layer %llu, not from the pool %llu, unfit %llu, "
	    "swapped again while on the panel or waiting %llu, queue off %llu, kern.sweet_fb_swapwait 0 %llu\n",
	    sweet_fbs.frames, sweet_fbs.flips, sweet_fbs.not_one_layer, sweet_fbs.not_pool, sweet_fbs.unfit, sweet_fbs.reused,
	    sweet_fbs.queue_off, sweet_fbs.swapwait_off);
	SWEET_FBD_CAT("drawn after the SwapEnd %llu; swaps that waited %llu, %llu us in all, max %llu; given up %llu; flips "
	    "waiting for a writable plane %llu\n", sweet_fbs.late_draw, sweet_fbs.holds, sweet_fbs.hold_us, sweet_fbs.hold_max_us,
	    sweet_fbs.timeouts, sweet_fbs.window_waits);
	SWEET_FBD_CAT("plane to scaled %llu, to unscaled %llu (waited %llu times, max %llu us); frames let go after the "
	    "panel left them %llu; stops %llu; flushes not taken %llu\n", sweet_fbs.to_scaled, sweet_fbs.to_unscaled,
	    sweet_fbs.unscaled_waits, sweet_fbs.unscaled_wait_max_us, sweet_fbs.released, sweet_fbs.stops, sweet_fbs.stuck);
	if (fb != NULL && fb->fPlaneScaled) {
		SWEET_FBD_CAT("showing %u,%u %ux%u of a surface (rows of %u bytes) from %#llx: %u,%u of it through the address, "
		    "SRC_XY %u,%u\n", fb->fPlaneOX, fb->fPlaneOY, fb->fPlaneGeo.sw, fb->fPlaneGeo.sh, fb->fPlaneStride, fb->fScanPhys,
		    fb->fPlaneOX - fb->fPlaneGeo.sx, fb->fPlaneOY - fb->fPlaneGeo.sy, fb->fPlaneGeo.sx, fb->fPlaneGeo.sy);
	}
	return SYSCTL_OUT(req, buf, off + 1);
}
SYSCTL_PROC(_kern, OID_AUTO, sweet_fb_scanout_diag, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED,
    NULL, 0, sweet_fb_scanout_diag_sysctl, "A", "sapphire 0350: the scanout's state and counters");

extern "C" int
sweet_fb_ubwc_on(void)
{
	return sweet_fb_ubwc != 0 && sweet_fb_scanout == 1 && sapphire_gpu_ubwc_mode() == 1 && sweet_fb_scan_ready;
}

static int
sweet_fb_ubwc_sysctl SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	int v = sweet_fb_ubwc;
	int error = sysctl_handle_int(oidp, &v, 0, req);
	if (error != 0 || req->newptr == USER_ADDR_NULL) {
		return error;
	}
	if (v != 0 && v != 1) {
		return EINVAL;
	}
	if (v != 0 && (sweet_fb_scanout != 1 || sapphire_gpu_ubwc_mode() != 1 || sweet_frame_fb == NULL || !sweet_fb_scan_ready)) {
		return EINVAL;
	}
	/* Existing pool surfaces cannot change layout in place; a disable stops direct scanout before copy resumes. */
	int was = sweet_fb_ubwc;
	sweet_fb_ubwc = v;
	if (was != 0 && v == 0 && sweet_frame_fb != NULL) {
		sweet_frame_fb->scanoutStop();
	}
	return 0;
}
SYSCTL_PROC(_kern, OID_AUTO, sweet_fb_ubwc, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED,
    NULL, 0, sweet_fb_ubwc_sysctl, "I", "sapphire 0398: 1 = shared UBWC display pool (GPU mode 1 and direct scanout only), 0 = linear");

static int
sweet_fb_ubwc_diag_sysctl SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	char line[192];
	snprintf(line, sizeof(line), "ubwc %d flips %llu errors %llu last %#x mdss_static %#x gpu_mode %d\n", sweet_fb_ubwc,
	    sweet_fb_ubwc_flips, sweet_fb_ubwc_errors, sweet_fb_ubwc_last_error,
	    sweet_fbr_va[0] != NULL ? sweet_fbr_va[0][0x144 / 4] : 0, sapphire_gpu_ubwc_mode());
	return SYSCTL_OUT(req, line, strlen(line) + 1);
}
SYSCTL_PROC(_kern, OID_AUTO, sweet_fb_ubwc_diag, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED,
    NULL, 0, sweet_fb_ubwc_diag_sysctl, "A", "sapphire 0398: SSPP UBWC error status sampled per flip");

SYSCTL_INT(_kern, OID_AUTO, sweet_fb_flush_taken, CTLFLAG_RW | CTLFLAG_LOCKED, &sweet_fb_flush_taken, 0,
    "sapphire 0352: 1 = a flip is taken when CTL_FLUSH's bits read clear (once they are seen to read back), for every flip; 0 = by the frame counter, as 0340");

/* sapphire 0353: kern.sweet_fb_swapwait (0, 1 or 2; boot-arg sweetfb_swapwait) and its counters (sweet_fbw, at the top). */
static int
sweet_fb_swapwait_sysctl SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	int v = sweet_fb_swapwait;
	int error = sysctl_handle_int(oidp, &v, 0, req);

	if (error != 0 || req->newptr == USER_ADDR_NULL) {
		return error;
	}
	if (v < 0 || v > 2) {
		return EINVAL;
	}
	int was = sweet_fb_swapwait;
	sweet_fb_swapwait = v;
	if (was != 0 && v == 0 && sweet_fb_scanout == 1 && sweet_frame_fb != NULL) {
		sweet_frame_fb->scanoutStop();  /* (the scanout goes with it: the frame on the panel is copied now) */
	}
	return 0;
}
SYSCTL_PROC(_kern, OID_AUTO, sweet_fb_swapwait, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED,
    NULL, 0, sweet_fb_swapwait_sysctl, "I",
    "sapphire 0353: 1 = SwapWait answers from the frame queue (a swap's surfaces in use, its frame on the panel), 2 = and a copied frame's surfaces until a later flip, 0 = at once");

static int
sweet_fb_swapwait_diag_sysctl SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	char buf[1024];
	size_t off = 0;
	AppleCLCD *fb = sweet_frame_fb;

	buf[0] = 0;
	SWEET_FBD_CAT("swapwait (kern.sweet_fb_swapwait %d): asked if the surfaces are free %llu (busy %llu), if the frame is "
	    "on the panel %llu (busy %llu), if every frame is %llu (busy %llu)\n", sweet_fb_swapwait,
	    sweet_fbw.asked[SWEET_FBW_FREE], sweet_fbw.busy[SWEET_FBW_FREE], sweet_fbw.asked[SWEET_FBW_SHOWN],
	    sweet_fbw.busy[SWEET_FBW_SHOWN], sweet_fbw.asked[SWEET_FBW_ALL], sweet_fbw.busy[SWEET_FBW_ALL]);
	SWEET_FBD_CAT("waited %llu times, %llu us in all, max %llu, timed out %llu; longest busy %llu us, answered 0 after "
	    "1 s busy %llu\n", sweet_fbw.waits, sweet_fbw.wait_us, sweet_fbw.wait_max_us, sweet_fbw.timeouts,
	    sweet_fbw.busy_max_us, sweet_fbw.forced);
	SWEET_FBD_CAT("SwapEnds held after a wait for every swap %llu, %llu us in all, max %llu, timed out %llu\n",
	    sweet_fbw.holds, sweet_fbw.hold_us, sweet_fbw.hold_max_us, sweet_fbw.hold_timeouts);
	if (fb != NULL) {
		SWEET_FBD_CAT("swaps: last flip %u (%s), on the panel %u, of a surface %u, left %u %u %u %u\n", fb->fFlipSwap,
		    fb->fQLatchWait ? "not taken yet" : "taken", fb->fPanelSwap, fb->fScanShowSwap, fb->fScanLeaveSwap[0],
		    fb->fScanLeaveSwap[1], fb->fScanLeaveSwap[2], fb->fScanLeaveSwap[3]);
	}
	return SYSCTL_OUT(req, buf, off + 1);
}
SYSCTL_PROC(_kern, OID_AUTO, sweet_fb_swapwait_diag, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED,
    NULL, 0, sweet_fb_swapwait_diag_sysctl, "A", "sapphire 0353: what SwapWait was asked and answered");
