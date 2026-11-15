/*
 * SWEET: USB device. sweetusb=1 only reports the controller state, sweetusb=2 also starts the
 * CDC-ECM gadget (sweet_usb_gadget_start).
 *
 * The phone's USB is a Synopsys DWC3 (USB 2.0 only, maximum-speed
 * high-speed) behind Qualcomm's dwc-usb3-msm wrapper; its power domain is
 * the USB30_PRIM GDSC and its clocks come from GCC. sapphire/topaz
 * (SM6225 khaje): DWC3 at 0x04e00000, Synopsys femto HS PHY at 0x01613000
 * (only read, by the USBDIAG dump), GCC offsets from Linux gcc-sm6115.c (the khaje GCC; the
 * USB30_PRIM GDSC at 0x141a004 matches the phone's DT). ABL uses this
 * controller for fastboot, so what the driver finds depends on the boot path: after leaving
 * fastboot (`fastboot continue`) the DWC3 is configured by ABL; on a normal boot of boot_<slot>
 * it is at reset values and the USB3 PIPE clock is off (dev-notes/investigacion/03, E1/E2).
 * sweet_usb_probe() itself never writes a register, and it touches the
 * DWC3 / PHY register blocks only when the GCC status registers say their
 * power domain and clocks are on (an access to a gated block would hang).
 */
#if defined(ARM64_BOARD_CONFIG_SWEET)
#include <sys/param.h>
#include <sys/systm.h>
#include <kern/clock.h>
#include <kern/thread.h>
#include <kern/sched_prim.h>
#include <machine/machine_routines.h>
#include <arm/caches_internal.h>
#include <vm/pmap.h>
#include <vm/vm_kern.h>
#include <pexpert/pexpert.h>
#include <kern/locks.h>
#include <sys/kpi_mbuf.h>
#include <sys/kpi_socket.h>
#include <sys/sockio.h>
#include <net/if.h>
#include <net/if_types.h>
#include <net/ethernet.h>
#include <net/if_ether.h>
#include <net/ether_if_module.h>
#include <net/kpi_interface.h>
#include <net/kpi_protocol.h>
#include <netinet/in.h>
#include <sys/sysctl.h>

extern vm_offset_t ml_io_map(vm_offset_t, vm_size_t);
void sweet_usb_probe(void);
static void sweet_usb_gadget_start(vm_offset_t dwc);
extern ppnum_t pmap_find_phys(pmap_t pmap, addr64_t va);

#define GCC_PA                  0x01400000
#define GCC_SIZE                0x001f0000
#define GCC_USB30_PRIM_GDSCR    0x1a004         /* bit 31: power on */
#define CBCR_CLK_OFF            (1u << 31)

static const struct {
	const char *name;
	uint32_t    off;
} usb_cbcr[] = {
	{ "master", 0x1a010 },
	{ "sleep", 0x1a014 },
	{ "utmi", 0x1a018 },
	{ "cfg-noc-axi", 0x1a084 },
	{ "sys-noc-axi", 0x1a080 },
	{ "ahb2phy", 0x1d008 },
	{ "clkref", 0x9f000 },
};

#define DWC3_PA                 0x04e00000
#define DWC3_SIZE               0x0000d000

static uint32_t
usb_rd(vm_offset_t base, uint32_t off)
{
	uint32_t v = *(volatile uint32_t *)(base + off);
	__asm__ volatile ("dmb oshld" ::: "memory");
	return v;
}

/*
 * 0329 (diagnostic, read-only): dump what the boot path left in the USB block before this driver
 * writes anything. Same image, `fastboot continue` enumerates and set_active + reboot does not
 * (2026-11-14, VBUS present on both: PMIC usb 0x10); the DWC3 summary differs (GUSB2PHYCFG 0x2400 vs
 * 0x2500, DCFG/DCTL/DEVTEN set vs reset values), but the femto HS PHY (0x01613000), QSCRATCH and the
 * GCC resets/RCGs were never read. Lines "SWEET USBDIAG <block> +off w0 w1 w2 w3".
 * sweetusb_reclaim=0 disables 0327's HWO reclaim (default 1 = unchanged), for an A/B under load.
 */
static int ud_reclaim_on = 1;

static void
usbdiag_dump(const char *name, vm_offset_t base, uint32_t from, uint32_t to)
{
	for (uint32_t o = from; o < to; o += 16) {
		printf("SWEET USBDIAG %s +0x%05x %08x %08x %08x %08x\n", name, o,
		    usb_rd(base, o), usb_rd(base, o + 4), usb_rd(base, o + 8), usb_rd(base, o + 12));
	}
}

static void
usbdiag_snapshot(vm_offset_t gcc, vm_offset_t dwc)
{
	vm_offset_t phy = ml_io_map(0x01613000, 0x1000);
	vm_offset_t qs = ml_io_map(0x04ef8800, 0x100);

	usbdiag_dump("GCC", gcc, 0x1a000, 0x1a0a0);     /* USB30_PRIM BCR, GDSC, CBCRs, master/mock_utmi RCGs */
	usbdiag_dump("GCC", gcc, 0x1c000, 0x1c010);     /* HS PHY BCR */
	usbdiag_dump("GCC", gcc, 0x1d000, 0x1d010);     /* ahb2phy */
	usbdiag_dump("DWC3", dwc, 0xc100, 0xc160);      /* GSBUSCFG, GCTL, GSTS, GUCTL1, GSNPSID, GUCTL */
	usbdiag_dump("DWC3", dwc, 0xc200, 0xc210);      /* GUSB2PHYCFG */
	usbdiag_dump("DWC3", dwc, 0xc2c0, 0xc2d0);      /* GUSB3PIPECTL */
	usbdiag_dump("DWC3", dwc, 0xc300, 0xc320);      /* GTXFIFOSIZ */
	usbdiag_dump("DWC3", dwc, 0xc380, 0xc390);      /* GRXFIFOSIZ */
	usbdiag_dump("DWC3", dwc, 0xc600, 0xc640);      /* GUCTL2, GFLADJ */
	usbdiag_dump("DWC3", dwc, 0xc700, 0xc730);      /* DCFG..DALEPENA */
	if (qs != 0) {
		usbdiag_dump("QSCR", qs, 0x00, 0x80);
	} else {
		printf("SWEET USBDIAG QSCRATCH map failed\n");
	}
	if (phy != 0) {
		usbdiag_dump("PHY", phy, 0x000, 0x120);
	} else {
		printf("SWEET USBDIAG PHY map failed\n");
	}
}

void
sweet_usb_probe(void)
{
	int enabled = 0;
	uint32_t gdscr, cbcr[sizeof(usb_cbcr) / sizeof(usb_cbcr[0])];
	bool clocks_on = true;

	/* sweetusb=0 (default) off, 1 report state only, 2 also start the USB device */
	PE_parse_boot_argn("sweetusb", &enabled, sizeof(enabled));
	if (!enabled) {
		return;
	}
	vm_offset_t gcc = ml_io_map(GCC_PA, GCC_SIZE);
	if (gcc == 0) {
		printf("SWEET USB: GCC map failed\n");
		return;
	}
	gdscr = usb_rd(gcc, GCC_USB30_PRIM_GDSCR);
	printf("SWEET USB: GDSC usb30_prim 0x%08x (%s)\n", gdscr, (gdscr & (1u << 31)) ? "on" : "off");
	for (unsigned i = 0; i < sizeof(usb_cbcr) / sizeof(usb_cbcr[0]); i++) {
		cbcr[i] = usb_rd(gcc, usb_cbcr[i].off);
		printf("SWEET USB: clk %-12s CBCR 0x%08x (%s)\n", usb_cbcr[i].name, cbcr[i],
		    (cbcr[i] & CBCR_CLK_OFF) ? "off" : "on");
	}
	/* DWC3 registers need the GDSC, the master clock and the config-NoC path. */
	clocks_on = (gdscr & (1u << 31)) && !(cbcr[0] & CBCR_CLK_OFF) && !(cbcr[3] & CBCR_CLK_OFF);
	if (!clocks_on) {
		printf("SWEET USB: controller unpowered or unclocked; DWC3 not read\n");
		return;
	}
	vm_offset_t dwc = ml_io_map(DWC3_PA, DWC3_SIZE);
	if (dwc == 0) {
		printf("SWEET USB: DWC3 map failed\n");
		return;
	}
	printf("SWEET USB: DWC3 GSNPSID 0x%08x GCTL 0x%08x GSTS 0x%08x GUSB2PHYCFG 0x%08x\n",
	    usb_rd(dwc, 0xc120), usb_rd(dwc, 0xc110), usb_rd(dwc, 0xc118), usb_rd(dwc, 0xc200));
	printf("SWEET USB: DWC3 DCFG 0x%08x DCTL 0x%08x DEVTEN 0x%08x DSTS 0x%08x DALEPENA 0x%08x\n",
	    usb_rd(dwc, 0xc700), usb_rd(dwc, 0xc704), usb_rd(dwc, 0xc708), usb_rd(dwc, 0xc70c),
	    usb_rd(dwc, 0xc720));
	printf("SWEET USB: DWC3 GEVNTADR 0x%08x%08x GEVNTSIZ 0x%08x GEVNTCOUNT 0x%08x GHWPARAMS0 0x%08x\n",
	    usb_rd(dwc, 0xc404), usb_rd(dwc, 0xc400), usb_rd(dwc, 0xc408), usb_rd(dwc, 0xc40c),
	    usb_rd(dwc, 0xc140));
	PE_parse_boot_argn("sweetusb_reclaim", &ud_reclaim_on, sizeof(ud_reclaim_on));
	printf("SWEET USBDIAG reclaim(0327)=%d\n", ud_reclaim_on);
	usbdiag_snapshot(gcc, dwc);
	if (enabled >= 2) {
		sweet_usb_gadget_start(dwc);
	}
}

/*
 * Step 2: USB device mode with only the default control endpoint, enough
 * for the host to enumerate the phone (boot-arg sweetusb=2). On both boot paths the GDSC and the
 * controller clocks are on when the driver starts; the DWC3 itself is only configured by ABL
 * after fastboot, and on a normal boot it is at reset values with the USB3 PIPE clock off
 * (investigacion/03, E1/E2). gadget_start therefore selects UTMI as the PIPE clock
 * (sweetusb_utmipipe, default 1), then: device soft reset, an event
 * buffer in our memory, endpoint 0, run. Events are polled from a kernel
 * thread (no interrupt), like the UFS driver. DMA is not cache coherent
 * (DT qcom,gsi-disable-io-coherency): ud_sync() cleans/invalidates (flush_dcache64 is a real
 * dc civac in this build); data path v2 maps its area non-cacheable instead (R6).
 * Register layout and the control-transfer state machine follow the
 * Synopsys DWC3 databook as used by Linux drivers/usb/dwc3 (gadget.c,
 * ep0.c).
 */
#define GCTL            0xc110
#define GEVNTADRLO      0xc400
#define GEVNTADRHI      0xc404
#define GEVNTSIZ        0xc408
#define GEVNTCOUNT      0xc40c
#define DCFG            0xc700
#define DCTL            0xc704
#define DEVTEN          0xc708
#define DSTS            0xc70c
#define DALEPENA        0xc720
#define DEPCMDPAR2(n)   (0xc800 + 0x10 * (n))
#define DEPCMDPAR1(n)   (0xc804 + 0x10 * (n))
#define DEPCMDPAR0(n)   (0xc808 + 0x10 * (n))
#define DEPCMD(n)       (0xc80c + 0x10 * (n))

/*
 * Qualcomm wrapper registers ("QSCRATCH", core base + 0xf8800). The
 * controller only connects (pull-up on D+) while it sees a valid VBUS
 * session; Linux dwc3-qcom forces that in software when the charger
 * reports a cable (dwc3_qcom_vbus_override_enable).
 */
#define QSCRATCH_PA             0x04ef8800    /* DWC3 + 0xf8800 */
#define QSCRATCH_HS_PHY_CTRL    0x10
#define   UTMI_OTG_VBUS_VALID   (1u << 20)
#define   SW_SESSVLD_SEL        (1u << 28)
#define QSCRATCH_SS_PHY_CTRL    0x30
#define   LANE0_PWR_PRESENT     (1u << 24)

#define DCTL_RUN_STOP   (1u << 31)
#define DCTL_CSFTRST    (1u << 30)
#define DSTS_DEVCTRLHLT (1u << 22)

#define DEPCMD_SETEPCONFIG      1
#define DEPCMD_SETTRANSFRESOURCE 2
#define DEPCMD_SETSTALL         4
#define DEPCMD_CLEARSTALL       5
#define DEPCMD_STARTTRANSFER    6
#define DEPCMD_ENDTRANSFER      8
#define DEPCMD_HIPRI_FORCERM    (1u << 11)
#define DEPCMD_DEPSTARTCFG      9
#define DEPCMD_CMDACT           (1u << 10)

#define TRB_HWO         (1u << 0)
#define TRB_LST         (1u << 1)
#define TRB_ISP_IMI     (1u << 10)
#define TRB_IOC         (1u << 11)
#define TRBCTL_SETUP    (2u << 4)
#define TRBCTL_STATUS2  (3u << 4)
#define TRBCTL_STATUS3  (4u << 4)
#define TRBCTL_DATA     (5u << 4)

/* DMA area layout */
#define UD_EVT          0x0000          /* event buffer, 4 KiB */
#define UD_EVT_SIZE     0x1000
#define UD_TRB_OUT      0x1000          /* ep0 OUT TRB */
#define UD_TRB_IN       0x1010          /* ep0 IN TRB */
#define UD_TRB_RX       0x1020          /* ep1 OUT (bulk, host -> phone) */
#define UD_TRB_TX       0x1030          /* ep1 IN (bulk, phone -> host) */
#define UD_TRB_INT      0x1050          /* ep2 IN (interrupt, notifications) */
#define UD_SETUP        0x1040          /* 8-byte setup packet */
#define UD_DATA         0x1100          /* ep0 data stage, 512 bytes */
#define UD_NOTIFY       0x1300          /* 16-byte notification */
#define UD_RX           0x2000          /* one received frame */
#define UD_TX           0x2800          /* one frame to send */
#define UD_FRAME_MAX    0x800
#define UD_SIZE         0x4000

/* Physical endpoints: 2n = OUT, 2n+1 = IN */
#define EP_RX           2               /* 0x01 bulk OUT */
#define EP_TX           3               /* 0x81 bulk IN */
#define EP_INT          5               /* 0x82 interrupt IN */
#define TRBCTL_NORMAL   (1u << 4)

#define UD_TXQ          32

static struct {
	vm_offset_t dwc;
	vm_offset_t dma;
	uint64_t    dma_pa;
	uint32_t    evt_pos;
	int         sleep_chan;
	enum { EP0_SETUP, EP0_DATA, EP0_WAIT_STATUS2, EP0_WAIT_STATUS3, EP0_STATUS } state;
	bool        data_in;        /* data stage direction */
	uint32_t    events;
	bool        configured;
	uint8_t     data_alt;       /* data interface alternate setting */
	/* network */
	ifnet_t     ifp;
	lck_mtx_t  *lock;           /* TX ring and TX endpoint */
	mbuf_t      txq[UD_TXQ];
	unsigned    txq_head, txq_count;
	bool        tx_busy;
	uint64_t    rx_frames, tx_frames;
	/* every endpoint's transfer (control 0/1 and data), to end them on a bus reset */
	bool        xfer_active[EP_INT + 1];
	uint8_t     xfer_rsc[EP_INT + 1];
	uint32_t    resets;
	/* old data path: transfers ud_watch took on HWO=0 before their event was seen; the event
	 * still arrives later (events arrive late, they are not lost: audit 01 §4) */
	uint32_t    lost_rx, lost_tx, stale_rx;
	uint32_t    rx_rearms;      /* RX armed but silent, ended and armed again (ud_watch) */
	uint32_t    halt_clears;    /* 0289: CLEAR_FEATURE(ENDPOINT_HALT) / SET_INTERFACE: sequence numbers reset */
	uint64_t    watch_rx, watch_tx;
} ud;

/* 0330: data path v2 (below, before ud_thread); sweetusb_dp set to anything but 2 selects the old path */
/*
 * 0332: data path v2 (0330) is the default; sweetusb_dp=0 selects the old path for an A/B.
 * Evidence (dev-notes/investigacion/03, E4 / carga-1): 20 x 30 MB both ways intact, 0 refused
 * Start Transfer, against the old path dying after 192 KiB (130538 refusals).
 */
static int ud_dp;      /* 0 until gadget_start has initialised everything; default chosen there (0333) */
#define DP_TRB_RX       0x1400          /* one data TRB per 64-byte line, away from ep0's (0x1000-0x107f) */
#define DP_TRB_TX       0x1440
#define DP_TRB_INT      0x1480
static void ud_dp_notify(void);
static void ud_dp_reset_seq(unsigned ep);
static errno_t ud_dp_output(ifnet_t ifp, mbuf_t m);
static void ud_dp_bus_reset(void);
static void ud_dp_ep_event(unsigned ep, unsigned type, unsigned status, uint32_t ev);

static inline void
dwc_wr(uint32_t off, uint32_t v)
{
	*(volatile uint32_t *)(ud.dwc + off) = v;
	__asm__ volatile ("dsb sy" ::: "memory");
}

static inline uint32_t
dwc_rd(uint32_t off)
{
	return usb_rd(ud.dwc, off);
}

static void
ud_sync(uint32_t off, uint32_t len)
{
	flush_dcache64((addr64_t)(ud.dma_pa + off), len, TRUE);
}

static void
ud_sleep_ms(unsigned ms)
{
	uint64_t deadline;
	clock_interval_to_deadline(ms, NSEC_PER_MSEC, &deadline);
	(void)assert_wait_deadline(&ud.sleep_chan, THREAD_UNINT, deadline);
	thread_block(THREAD_CONTINUE_NULL);
}

static int
ud_epcmd(unsigned ep, uint32_t cmd, uint32_t p0, uint32_t p1, uint32_t p2)
{
	dwc_wr(DEPCMDPAR0(ep), p0);
	dwc_wr(DEPCMDPAR1(ep), p1);
	dwc_wr(DEPCMDPAR2(ep), p2);
	dwc_wr(DEPCMD(ep), cmd | DEPCMD_CMDACT);
	/* 5000 polls (0328). delay() blocks in this port (cpu_idle_latency is 0, so ml_delay_should_spin
	 * is false): each delay(1) is a thread_block and the bound is not 5 ms. The old data path calls
	 * this with ud.lock held (ud_tx_kick_locked, audit 01 D5); data path v2 spins instead (R5). */
	for (int i = 0; i < 5000; i++) {
		uint32_t v = dwc_rd(DEPCMD(ep));
		if (!(v & DEPCMD_CMDACT)) {
			uint32_t status = (v >> 12) & 0xf;
			if (status) {
				printf("SWEET USB: ep%u cmd 0x%x status %u\n", ep, cmd & 0xf, status);
				return -1;
			}
			return 0;
		}
		delay(1);
	}
	printf("SWEET USB: ep%u cmd 0x%x timeout\n", ep, cmd & 0xf);
	return -1;
}

/* One TRB (at DMA offset trb) on physical endpoint ep: 0, or -1 if the controller refused it. */
static int
ud_start_trb(unsigned ep, uint32_t trb, uint32_t buf, uint32_t len, uint32_t ctl)
{
	volatile uint32_t *t = (volatile uint32_t *)(ud.dma + trb);
	uint64_t bpa = ud.dma_pa + buf;

	t[0] = (uint32_t)bpa;
	t[1] = (uint32_t)(bpa >> 32);
	t[2] = len;
	t[3] = ctl | TRB_HWO | TRB_LST | TRB_ISP_IMI | TRB_IOC;
	ud_sync(trb, 16);
	if (len) {
		ud_sync(buf, len);
	}
	uint64_t tpa = ud.dma_pa + trb;
	if (ud_epcmd(ep, DEPCMD_STARTTRANSFER, (uint32_t)(tpa >> 32), (uint32_t)tpa, 0) != 0) {
		return -1;
	}
	if (ep <= EP_INT) {
		/* the transfer resource index, for ENDTRANSFER (Linux: dwc3_gadget_ep_get_transfer_index) */
		ud.xfer_rsc[ep] = (uint8_t)((dwc_rd(DEPCMD(ep)) >> 16) & 0x7f);
		ud.xfer_active[ep] = true;
	}
	return 0;
}

static int
ud_start(unsigned ep, uint32_t buf, uint32_t len, uint32_t ctl)
{
	return ud_start_trb(ep, ep ? UD_TRB_IN : UD_TRB_OUT, buf, len, ctl);
}

/* End endpoint ep's transfer, if it has one. */
static void
ud_end_transfer(unsigned ep)
{
	if (ep <= EP_INT && ud.xfer_active[ep]) {
		ud_epcmd(ep, DEPCMD_ENDTRANSFER | DEPCMD_HIPRI_FORCERM | ((uint32_t)ud.xfer_rsc[ep] << 16), 0, 0, 0);
		delay(100);
		ud.xfer_active[ep] = false;
	}
}

/*
 * Arm ep0 for the next SETUP. If the controller refuses (a control transfer the host abandoned
 * is still active: "ep0 cmd 0x6 status 1"), end it and try again: otherwise ep0 never listens
 * again, and every enumeration after that fails at the first GET_DESCRIPTOR.
 */
static void
ud_start_setup(void)
{
	ud.state = EP0_SETUP;
	if (ud_start(0, UD_SETUP, 8, TRBCTL_SETUP) != 0) {
		/* the transfer still active on ep0 is ended by the index it was started with */
		ud.xfer_active[0] = true;
		ud_end_transfer(0);
		ud_end_transfer(1);
		if (ud_start(0, UD_SETUP, 8, TRBCTL_SETUP) != 0) {
			printf("SWEET USB: ep0 cannot be armed for SETUP\n");
		}
	}
}

static void
ud_stall(void)
{
	ud_epcmd(0, DEPCMD_SETSTALL, 0, 0, 0);
	ud_start_setup();
}

/*
 * USB descriptors: a CDC-ECM (Ethernet) function, which Linux (cdc_ether)
 * and macOS drive without extra drivers. VID 0x1209 / PID 0x0001 =
 * pid.codes test ID, not a product.
 */
static const uint8_t ud_dev_desc[18] = {
	18, 1, 0x00, 0x02, 0x02, 0x00, 0x00, 64,
	0x09, 0x12, 0x01, 0x00, 0x00, 0x01, 1, 2, 3, 1,
};
static const uint8_t ud_cfg_desc[80] = {
	9, 2, 80, 0, 2, 1, 0, 0x80, 250,                /* config: 2 interfaces, 500 mA */
	9, 4, 0, 0, 1, 0x02, 0x06, 0x00, 0,             /* comm interface 0: CDC ECM, 1 endpoint */
	5, 0x24, 0x00, 0x10, 0x01,                      /* header, CDC 1.10 */
	5, 0x24, 0x06, 0, 1,                            /* union: control 0, data 1 */
	13, 0x24, 0x0f, 4, 0, 0, 0, 0, 0xea, 0x05, 0, 0, 0, /* ethernet: MAC string 4, max segment 1514 */
	7, 5, 0x82, 0x03, 16, 0, 9,                     /* ep 0x82 interrupt IN, 16 bytes, 2^(9-1) uframes */
	9, 4, 1, 0, 0, 0x0a, 0x00, 0x00, 0,             /* data interface 1 alt 0: no endpoints */
	9, 4, 1, 1, 2, 0x0a, 0x00, 0x00, 0,             /* data interface 1 alt 1: two bulk endpoints */
	7, 5, 0x81, 0x02, 0x00, 0x02, 0,                /* ep 0x81 bulk IN, 512 */
	7, 5, 0x01, 0x02, 0x00, 0x02, 0,                /* ep 0x01 bulk OUT, 512 */
};
/* The host side's MAC (ECM iMACAddress); the phone's own is ..:02 */
static const char *const ud_strings[] = {
	NULL, "ios-sweet", "Sweet iOS (USB Ethernet)", "0001", "02005E77EE01",
};
static const uint8_t ud_phone_mac[ETHER_ADDR_LEN] = { 0x02, 0x00, 0x5e, 0x77, 0xee, 0x02 };

static uint32_t
ud_string(unsigned idx, uint8_t *out)
{
	if (idx == 0) {
		out[0] = 4; out[1] = 3; out[2] = 0x09; out[3] = 0x04;   /* en-US */
		return 4;
	}
	if (idx >= sizeof(ud_strings) / sizeof(ud_strings[0])) {
		return 0;
	}
	const char *str = ud_strings[idx];
	uint32_t n = 2;
	for (; *str && n < 126; str++) {
		out[n++] = (uint8_t)*str;
		out[n++] = 0;
	}
	out[0] = (uint8_t)n;
	out[1] = 3;
	return n;
}

static void ud_rx_arm(void);
static void ud_tx_kick_locked(void);
static void ud_ep_reset_seq(unsigned ep);

/*
 * 0290: what the controller and the host did, kept for after a dead link (the console log goes
 * over the same link): 256 entries of (ms since boot, what, a, b). Read: kern.sweet_usb_trace.
 */
#define UT_N 256
static struct ut_ent { uint32_t ms; char what[4]; uint32_t a, b; } ut_ring[UT_N];
static uint32_t ut_pos;
static uint64_t ut_last_rx_ms;

static uint32_t
ut_now_ms(void)
{
	uint64_t ns;
	absolutetime_to_nanoseconds(mach_absolute_time(), &ns);
	return (uint32_t)(ns / 1000000ull);
}

static void
ut(const char *what, uint32_t a, uint32_t b)
{
	struct ut_ent *e = &ut_ring[ut_pos++ % UT_N];
	e->ms = ut_now_ms();
	strlcpy(e->what, what, sizeof(e->what));
	e->a = a;
	e->b = b;
}

/* NETWORK_CONNECTION notification: the host brings its link up on it. */
static void
ud_notify_connected(void)
{
	if (ud_dp) {
		ud_dp_notify();
		return;
	}
	uint8_t *n = (uint8_t *)(ud.dma + UD_NOTIFY);
	n[0] = 0xa1; n[1] = 0x00;       /* NETWORK_CONNECTION */
	n[2] = 1; n[3] = 0;             /* connected */
	n[4] = 0; n[5] = 0;             /* interface 0 */
	n[6] = 0; n[7] = 0;
	ud_start_trb(EP_INT, UD_TRB_INT, UD_NOTIFY, 8, TRBCTL_NORMAL);
}

/* SET_CONFIGURATION(1): bulk IN/OUT and the interrupt endpoint. */
static void
ud_configure(void)
{
	if (ud.configured) {
		return;
	}
	if (ud_epcmd(0, DEPCMD_DEPSTARTCFG | (2u << 16), 0, 0, 0) ||
	    ud_epcmd(EP_RX, DEPCMD_SETEPCONFIG, (2u << 1) | (512u << 3), (1u << 8) | ((uint32_t)EP_RX << 25), 0) ||
	    ud_epcmd(EP_RX, DEPCMD_SETTRANSFRESOURCE, 1, 0, 0) ||
	    ud_epcmd(EP_TX, DEPCMD_SETEPCONFIG, (2u << 1) | (512u << 3) | (1u << 17), (1u << 8) | ((uint32_t)EP_TX << 25), 0) ||
	    ud_epcmd(EP_TX, DEPCMD_SETTRANSFRESOURCE, 1, 0, 0) ||
	    ud_epcmd(EP_INT, DEPCMD_SETEPCONFIG, (3u << 1) | (16u << 3) | (2u << 17),
	    (1u << 8) | (8u << 16) | ((uint32_t)EP_INT << 25), 0) ||
	    ud_epcmd(EP_INT, DEPCMD_SETTRANSFRESOURCE, 1, 0, 0)) {
		printf("SWEET USB: data endpoints setup failed\n");
		return;
	}
	dwc_wr(DALEPENA, dwc_rd(DALEPENA) | (1u << EP_RX) | (1u << EP_TX) | (1u << EP_INT));
	ud.configured = true;
	if (!ud_dp) {
		ud_rx_arm();
	}
	printf("SWEET USB: ECM endpoints configured\n");
}

static void
ud_setup(void)
{
	const uint8_t *st = (const uint8_t *)(ud.dma + UD_SETUP);
	uint8_t *data = (uint8_t *)(ud.dma + UD_DATA);
	uint8_t type = st[0], req = st[1];
	uint16_t value = (uint16_t)(st[2] | st[3] << 8);
	uint16_t index = (uint16_t)(st[4] | st[5] << 8);
	uint16_t length = (uint16_t)(st[6] | st[7] << 8);
	uint32_t n = 0;
	bool ok = true;

	printf("SWEET USB: setup %02x %02x %04x %04x len %u\n", type, req, value, index, length);
	ut("SET", (uint32_t)type << 8 | req, (uint32_t)value << 16 | index);
	ud.data_in = (type & 0x80) != 0;
	if ((type & 0x60) == 0x20) {    /* class request (e.g. SET_ETHERNET_PACKET_FILTER) */
		ok = (length == 0);
	} else if ((type & 0x60) != 0) {
		ok = false;
	} else if (req == 6) {          /* GET_DESCRIPTOR */
		switch (value >> 8) {
		case 1: memcpy(data, ud_dev_desc, sizeof(ud_dev_desc)); n = sizeof(ud_dev_desc); break;
		case 2: memcpy(data, ud_cfg_desc, sizeof(ud_cfg_desc)); n = sizeof(ud_cfg_desc); break;
		case 3: n = ud_string(value & 0xff, data); ok = n != 0; break;
		default: ok = false; break;
		}
	} else if (req == 5) {          /* SET_ADDRESS: takes effect now (DWC3 handles the timing) */
		uint32_t dcfg = dwc_rd(DCFG) & ~(0x7fu << 3);
		dwc_wr(DCFG, dcfg | ((uint32_t)(value & 0x7f) << 3));
		printf("SWEET USB: address %u\n", value & 0x7f);
	} else if (req == 9) {          /* SET_CONFIGURATION */
		if (value == 1) {
			ud_configure();
		}
		printf("SWEET USB: configured (%u)\n", value);
	} else if (req == 8) {          /* GET_CONFIGURATION */
		data[0] = ud.configured ? 1 : 0; n = 1;
	} else if (req == 0) {          /* GET_STATUS */
		data[0] = 0; data[1] = 0; n = 2;
	} else if (req == 11) {         /* SET_INTERFACE */
		if (index == 1) {
			if (value == 1 && ud.configured && ud.lock != NULL) {
				ud_ep_reset_seq(EP_RX);  /* 0289: a new alternate setting starts at DATA0 */
				ud_ep_reset_seq(EP_TX);
			}
			ud.data_alt = (uint8_t)value;
			if (value == 1 && ud.configured) {
				ud_notify_connected();
			}
		}
	} else if (req == 10) {         /* GET_INTERFACE */
		data[0] = index == 1 ? ud.data_alt : 0; n = 1;
	} else if (req == 1 && (type & 0x1f) == 2 && value == 0) { /* CLEAR_FEATURE(ENDPOINT_HALT) */
		unsigned a = index & 0xff;
		if (ud.configured && ud.lock != NULL) {
			if (a == 0x01) {
				ud_ep_reset_seq(EP_RX);
			} else if (a == 0x81) {
				ud_ep_reset_seq(EP_TX);
			} else if (a == 0x82) {
				ud_ep_reset_seq(EP_INT);
			}
		}
	} else if (req == 1 || req == 3) { /* other CLEAR/SET_FEATURE (remote wakeup; SET halt is not used here) */
	} else {
		ok = false;
	}
	if (!ok) {
		ud_stall();
		return;
	}
	if (length == 0) {
		ud.state = EP0_WAIT_STATUS2;    /* status stage on XferNotReady */
		return;
	}
	ud.state = EP0_DATA;
	if (ud.data_in) {
		if (n > length) {
			n = length;
		}
		ud_start(1, UD_DATA, n, TRBCTL_DATA);
	} else {
		ud_start(0, UD_DATA, 64, TRBCTL_DATA);  /* OUT data is not used here */
	}
}

/* ---- network interface (ue0) ---- */

/*
 * 0289: the host reset a data endpoint's data toggle (CLEAR_FEATURE(ENDPOINT_HALT), or SET_INTERFACE,
 * which resets every endpoint of the interface): the controller's sequence number must go back to
 * DATA0 as well (DWC3 ClearStall does that), its transfer ended first and armed again after.
 * usbipd/Windows reset the pipe that way after a transfer it cancelled (a cdc_ether transmit
 * timeout on the PC). What ignoring it does to the link is not measured: a one-packet toggle
 * mismatch resynchronises by itself (audit 01 §4, 0289); the measured dead link under load is
 * 0327's double completion (investigacion/03 E-carga-1).
 */
static void
ud_ep_reset_seq(unsigned ep)
{
	ud.halt_clears++;
	if (ud_dp) {
		ud_dp_reset_seq(ep);
		return;
	}
	if (ep == EP_TX) {
		lck_mtx_lock(ud.lock);
		ud_end_transfer(EP_TX);
		(void)ud_epcmd(EP_TX, DEPCMD_CLEARSTALL, 0, 0, 0);
		ud.tx_busy = false;
		ud_tx_kick_locked();
		lck_mtx_unlock(ud.lock);
	} else if (ep == EP_RX) {
		ud_end_transfer(EP_RX);
		(void)ud_epcmd(EP_RX, DEPCMD_CLEARSTALL, 0, 0, 0);
		ud_rx_arm();
	} else if (ep == EP_INT) {
		ud_end_transfer(EP_INT);
		(void)ud_epcmd(EP_INT, DEPCMD_CLEARSTALL, 0, 0, 0);
	}
}

static void
ud_rx_arm(void)
{
	/* refused (a transfer still held by the controller): end it and arm again, or RX is dead for good */
	if (ud_start_trb(EP_RX, UD_TRB_RX, UD_RX, UD_FRAME_MAX, TRBCTL_NORMAL) != 0) {
		ut("RXR", dwc_rd(DEPCMD(EP_RX)), 0);
		ud.xfer_active[EP_RX] = true;
		ud_end_transfer(EP_RX);
		(void)ud_start_trb(EP_RX, UD_TRB_RX, UD_RX, UD_FRAME_MAX, TRBCTL_NORMAL);
	}
}

static void
ud_rx_done(void)
{
	volatile uint32_t *t = (volatile uint32_t *)(ud.dma + UD_TRB_RX);
	ud_sync(UD_TRB_RX, 16);
	/*
	 * 0291: the XferComplete for RX can find HWO still set in the RX TRB ("late rx"). The cause is
	 * not established: the RX TRB shares a 64-byte cache line with the other TRBs (audit 01 D4),
	 * and 0327's reclaim can consume one completion twice (audit 01 I5); a slow TRB write-back is
	 * not shown. Returning without arming leaves RX unarmed (the event already marked the transfer
	 * inactive, ud_watch skips it) and every bulk OUT from the host is NAKed. So: wait up to
	 * 100 x delay(10) -- each one blocks the thread, delay() does not spin in this port -- and if
	 * HWO is still set, drop that frame and arm RX again (ud_rx_arm ends a transfer the controller
	 * still holds first).
	 */
	for (int i = 0; (t[3] & TRB_HWO) && i < 100; i++) {
		delay(10);
		ud_sync(UD_TRB_RX, 16);
	}
	if (t[3] & TRB_HWO) {
		ud.stale_rx++;
		ut("RXS", t[3], t[2]);
		ud_rx_arm();
		return;
	}
	uint32_t len = UD_FRAME_MAX - (t[2] & 0xffffff);
	uint32_t now = ut_now_ms();
	if ((t[2] >> 28) != 0 || len < ETHER_HDR_LEN || len > ETHER_MAX_LEN || (ut_last_rx_ms && now - ut_last_rx_ms > 1000)) {
		ut("RXd", t[2], now - (uint32_t)ut_last_rx_ms);
	}
	ut_last_rx_ms = now;
	ud_sync(UD_RX, UD_FRAME_MAX);
	if (ud.ifp != NULL && len >= ETHER_HDR_LEN && len <= ETHER_MAX_LEN) {
		/*
		 * One mbuf with a 2 KiB cluster holds any frame. (mbuf_allocpacket
		 * may return a first mbuf of length 0: stripping the header from
		 * it panicked "Failed mbuf validity check: len -14".)
		 */
		mbuf_t m = NULL;
		if (mbuf_gethdr(MBUF_DONTWAIT, MBUF_TYPE_DATA, &m) == 0 &&
		    mbuf_mclget(MBUF_DONTWAIT, MBUF_TYPE_DATA, &m) == 0 &&
		    (mbuf_flags(m) & MBUF_EXT) && mbuf_maxlen(m) >= len) {
			struct ifnet_stat_increment_param stats = {};
			memcpy(mbuf_data(m), (const void *)(ud.dma + UD_RX), len);
			mbuf_setlen(m, len);
			mbuf_pkthdr_setlen(m, len);
			void *frame_header = mbuf_data(m);
			mbuf_pkthdr_setrcvif(m, ud.ifp);
			mbuf_pkthdr_setheader(m, frame_header);
			mbuf_pkthdr_adjustlen(m, -ETHER_HDR_LEN);
			(void)mbuf_setdata(m, (char *)mbuf_data(m) + ETHER_HDR_LEN, mbuf_len(m) - ETHER_HDR_LEN);
			stats.packets_in = 1;
			stats.bytes_in = len;
			ud.rx_frames++;
			ifnet_input(ud.ifp, m, &stats);
		} else if (m != NULL) {
			mbuf_freem(m);
		}
	}
	ud_rx_arm();
}

/* Start the next queued frame if the IN endpoint is idle. ud.lock held. */
static void
ud_tx_kick_locked(void)
{
	while (!ud.tx_busy && ud.txq_count > 0 && ud.configured) {
		mbuf_t m = ud.txq[ud.txq_head];
		ud.txq_head = (ud.txq_head + 1) % UD_TXQ;
		ud.txq_count--;
		size_t len = mbuf_pkthdr_len(m);
		if (len > ETHER_MAX_LEN - ETHER_CRC_LEN) {
			mbuf_freem(m);
			continue;
		}
		mbuf_copydata(m, 0, len, (void *)(ud.dma + UD_TX));
		mbuf_freem(m);
		/* A frame of a multiple of 512 bytes would need a zero-length
		 * packet to end it; one pad byte does the same (IP ignores it). */
		if ((len % 512) == 0) {
			((uint8_t *)(ud.dma + UD_TX))[len++] = 0;
		}
		ud.tx_busy = true;
		ud.tx_frames++;
		if (ud_start_trb(EP_TX, UD_TRB_TX, UD_TX, (uint32_t)len, TRBCTL_NORMAL) != 0) {
			ud.tx_busy = false;     /* refused: that frame is lost, the next ones are not */
			break;
		}
	}
}

static errno_t
ud_if_output(ifnet_t ifp, mbuf_t m)
{
	if (ud_dp) {
		return ud_dp_output(ifp, m);
	}
	lck_mtx_lock(ud.lock);
	if (ud.txq_count == UD_TXQ || !ud.configured) {
		lck_mtx_unlock(ud.lock);
		mbuf_freem(m);
		ifnet_stat_increment_out(ifp, 0, 0, 1);
		return 0;
	}
	ifnet_stat_increment_out(ifp, 1, mbuf_pkthdr_len(m), 0);
	ud.txq[(ud.txq_head + ud.txq_count) % UD_TXQ] = m;
	ud.txq_count++;
	ud_tx_kick_locked();
	lck_mtx_unlock(ud.lock);
	return 0;
}

static errno_t
ud_if_ioctl(ifnet_t ifp, unsigned long cmd, void *data __unused)
{
	switch (cmd) {
	case SIOCSIFADDR:
	case SIOCAIFADDR:
		ifnet_set_flags(ifp, IFF_UP, IFF_UP);
		return 0;
	case SIOCSIFFLAGS:
	case SIOCADDMULTI:
	case SIOCDELMULTI:
		return 0;
	default:
		return EOPNOTSUPP;
	}
}

static void
ud_if_detach(ifnet_t ifp __unused)
{
}

/* ue0, 192.168.7.2/24 (the host side is 192.168.7.1). */
static void
ud_if_create(void)
{
	struct ifnet_init_eparams init;
	static lck_grp_t *grp;

	grp = lck_grp_alloc_init("sweet_usb", LCK_GRP_ATTR_NULL);
	ud.lock = lck_mtx_alloc_init(grp, LCK_ATTR_NULL);

	bzero(&init, sizeof(init));
	init.ver = IFNET_INIT_CURRENT_VERSION;
	init.len = sizeof(init);
	init.flags = IFNET_INIT_LEGACY;
	init.uniqueid = "sweet-usb-ecm";
	init.uniqueid_len = sizeof("sweet-usb-ecm") - 1;
	init.name = "ue";
	init.unit = 0;
	init.family = IFNET_FAMILY_ETHERNET;
	init.type = IFT_ETHER;
	init.output = ud_if_output;
	init.demux = ether_demux;
	init.add_proto = ether_add_proto;
	init.del_proto = ether_del_proto;
	init.check_multi = ether_check_multi;
	init.framer_extended = ether_frameout_extended;
	init.ioctl = ud_if_ioctl;
	init.detach = ud_if_detach;
	init.broadcast_addr = etherbroadcastaddr;
	init.broadcast_len = ETHER_ADDR_LEN;
	if (ifnet_allocate_extended(&init, &ud.ifp) != 0) {
		printf("SWEET USB: ifnet_allocate failed\n");
		ud.ifp = NULL;
		return;
	}
	ifnet_set_addrlen(ud.ifp, ETHER_ADDR_LEN);
	ifnet_set_hdrlen(ud.ifp, sizeof(struct ether_header));
	ifnet_set_mtu(ud.ifp, ETHERMTU);
	ifnet_set_baudrate(ud.ifp, 480000000);
	ifnet_set_flags(ud.ifp, IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST, 0xffff);
	if (ifnet_attach(ud.ifp, NULL) != 0) {
		printf("SWEET USB: ifnet_attach failed\n");
		ud.ifp = NULL;
		return;
	}
	ifnet_set_lladdr(ud.ifp, ud_phone_mac, ETHER_ADDR_LEN);

	errno_t err = proto_plumb(PF_INET, ud.ifp);
	socket_t so = NULL;
	if (err == 0) {
		err = sock_socket(PF_INET, SOCK_DGRAM, 0, NULL, NULL, &so);
	}
	if (err == 0) {
		struct ifaliasreq ifra;
		bzero(&ifra, sizeof(ifra));
		snprintf(ifra.ifra_name, sizeof(ifra.ifra_name), "ue0");
		struct sockaddr_in *a = (struct sockaddr_in *)&ifra.ifra_addr;
		a->sin_len = sizeof(*a);
		a->sin_family = AF_INET;
		a->sin_addr.s_addr = htonl(0xc0a80702);         /* 192.168.7.2 */
		a = (struct sockaddr_in *)&ifra.ifra_mask;
		a->sin_len = sizeof(*a);
		a->sin_family = AF_INET;
		a->sin_addr.s_addr = htonl(0xffffff00);
		a = (struct sockaddr_in *)&ifra.ifra_broadaddr;
		a->sin_len = sizeof(*a);
		a->sin_family = AF_INET;
		a->sin_addr.s_addr = htonl(0xc0a807ff);
		err = sock_ioctl(so, SIOCAIFADDR, &ifra);
		sock_close(so);
	}
	ifnet_set_flags(ud.ifp, IFF_UP | IFF_RUNNING, IFF_UP | IFF_RUNNING);
	printf("SWEET USB: ue0 192.168.7.2/24 %s (err %d)\n", err ? "NOT configured" : "up", err);
}

/*
 * A bus reset (the host's, or usbipd's when the phone was slow to answer) loses the data
 * endpoints' configuration in the controller. Until now only the address went back to 0: the
 * endpoints stayed "configured" here, so the next SET_CONFIGURATION set nothing up, a frame
 * being sent kept tx_busy, and the link was dead until a reboot. As Linux's
 * dwc3_gadget_reset_interrupt: end the data endpoints' transfers, forget the configuration and
 * the queue, restart a control transfer caught in the middle.
 */
static void
ud_bus_reset(void)
{
	if (ud_dp) {
		ud_dp_bus_reset();
		return;
	}
	for (unsigned ep = EP_RX; ep <= EP_INT; ep++) {
		ud_end_transfer(ep);
	}
	dwc_wr(DALEPENA, dwc_rd(DALEPENA) & ~((1u << EP_RX) | (1u << EP_TX) | (1u << EP_INT)));
	if (ud.lock != NULL) {
		lck_mtx_lock(ud.lock);
	}
	ud.configured = false;
	ud.data_alt = 0;
	ud.tx_busy = false;
	while (ud.txq_count > 0) {
		mbuf_freem(ud.txq[ud.txq_head]);
		ud.txq_head = (ud.txq_head + 1) % UD_TXQ;
		ud.txq_count--;
	}
	if (ud.lock != NULL) {
		lck_mtx_unlock(ud.lock);
	}
	/*
	 * ep0: a SETUP already waiting stays; a control transfer caught in the middle is ended on
	 * both directions (Linux: dwc3_ep0_end_control_data) and ep0 armed again.
	 */
	if (ud.state != EP0_SETUP || !ud.xfer_active[0]) {
		ud_end_transfer(0);
		ud_end_transfer(1);
		ud_start_setup();
	}
}

static void
ud_event(uint32_t ev)
{
	if (ev & 1) {                   /* device event */
		uint32_t type = (ev >> 8) & 0xf;
		if (type == 1) {        /* USB reset */
			dwc_wr(DCFG, dwc_rd(DCFG) & ~(0x7fu << 3));
			ut("RST", ud.resets, (uint32_t)ud.rx_frames);
			ud_bus_reset();
			if (ud.resets++ < 20) {
				printf("SWEET USB: bus reset\n");
			}
		} else if (type == 2) { /* connect done */
			uint32_t speed = dwc_rd(DSTS) & 7;
			/* ep0 max packet: 64 at high/full speed (modify action) */
			ud_epcmd(0, DEPCMD_SETEPCONFIG, (0u << 1) | (64u << 3) | (2u << 30),
			    (1u << 8) | (1u << 10) | (0u << 25), 0);
			ud_epcmd(1, DEPCMD_SETEPCONFIG, (0u << 1) | (64u << 3) | (2u << 30),
			    (1u << 8) | (1u << 10) | (1u << 25), 0);
			printf("SWEET USB: connected, speed %u\n", speed);
		} else if (type == 0) {
			printf("SWEET USB: disconnect\n");
		}
		return;
	}
	unsigned ep = (ev >> 1) & 0x1f;
	unsigned type = (ev >> 6) & 0xf;
	unsigned status = (ev >> 12) & 0xf;

	if (ep >= EP_RX && ep <= EP_INT && (type != 1 || (status & 1))) {   /* not a completion, or a bus error */
		ut("EV", ev, (uint32_t)ud.rx_frames);
	}
	if (ud_dp && ep >= EP_RX) {
		ud_dp_ep_event(ep, type, status, ev);
		return;
	}
	if (ep <= EP_INT && type == 1) {
		ud.xfer_active[ep] = false;
	}
	if (ep == EP_RX && type == 1) {
		ud_rx_done();
		return;
	}
	if (ep == EP_TX && type == 1) {
		lck_mtx_lock(ud.lock);
		ud.tx_busy = false;
		ud_tx_kick_locked();
		lck_mtx_unlock(ud.lock);
		return;
	}
	if (ep > 1) {
		return;
	}
	/*
	 * Control transfer state machine (Linux dwc3 ep0.c): SETUP completes ->
	 * request handled; the data stage (if any) is started right away; the
	 * status stage is started when the controller reports XferNotReady
	 * with status "control status"; its completion re-arms SETUP.
	 */
	if (type == 1) {                /* XferComplete */
		switch (ud.state) {
		case EP0_SETUP:
			ud_sync(UD_SETUP, 8);
			ud_setup();
			break;
		case EP0_DATA:
			ud.state = EP0_WAIT_STATUS3;
			break;
		case EP0_STATUS:
			ud_start_setup();
			break;
		default:
			break;
		}
		return;
	}
	if (type == 3 && status == 2) { /* XferNotReady, control status phase */
		if (ud.state == EP0_WAIT_STATUS2 || ud.state == EP0_WAIT_STATUS3) {
			uint32_t ctl = ud.state == EP0_WAIT_STATUS3 ? TRBCTL_STATUS3 : TRBCTL_STATUS2;
			ud.state = EP0_STATUS;
			ud_start(ep, UD_DATA, 0, ctl);
		}
	}
}

/*
 * Old data path only (ud_thread, sweetusb_dp other than 2). Every 500 ms: a data transfer whose
 * TRB has HWO clear while its XferComplete has not been processed, twice in a row with no frame
 * moving, is taken as if the event had come. That is not safe: events arrive late, they are not
 * lost (audit 01 §4), so the late event consumes the same completion a second time (audit 01 I3,
 * D1), and HWO=0 does not make a TRB available (Linux 51f1954ad853). Data path v2 never does
 * this (R2, R7).
 */
static void
ud_watch(void)
{
	static bool rx_seen, tx_seen;
	volatile uint32_t *rt = (volatile uint32_t *)(ud.dma + UD_TRB_RX);
	volatile uint32_t *tt = (volatile uint32_t *)(ud.dma + UD_TRB_TX);

	if (!ud.configured || ud.lock == NULL) {
		rx_seen = tx_seen = false;
		return;
	}
	ud_sync(UD_TRB_RX, 16);
	bool rx_idle = ud.xfer_active[EP_RX] && !(rt[3] & TRB_HWO) && ud.rx_frames == ud.watch_rx;
	if (rx_idle && rx_seen) {
		ut("RXL", ud.lost_rx, 0);
		if (ud.lost_rx++ < 20) {
			printf("SWEET USB: rx transfer done without an event (%u), taking it\n", ud.lost_rx);
		}
		ud.xfer_active[EP_RX] = false;
		ud_rx_done();
		rx_idle = false;
	}
	rx_seen = rx_idle;
	/*
	 * RX armed (HWO still set) and not a frame in 2 s: end the transfer and arm it again. This
	 * also runs on a link that is merely idle and is not free there: each re-arm is a ForceRM End
	 * Transfer with a fixed 100 us wait (audit 01 D6), and if the new Start is refused RX stays
	 * dead until a bus reset (D7, D9).
	 */
	/* 0291: configured and no RX transfer at all (nothing armed): arm it */
	static unsigned rx_unarmed;
	if (!ud.xfer_active[EP_RX] && !(rt[3] & TRB_HWO)) {
		if (++rx_unarmed >= 2) {
			rx_unarmed = 0;
			ut("RXU", (uint32_t)ud.rx_frames, rt[3]);
			ud_rx_arm();
		}
	} else {
		rx_unarmed = 0;
	}
	static unsigned rx_silent;
	if (ud.xfer_active[EP_RX] && (rt[3] & TRB_HWO) && ud.rx_frames == ud.watch_rx) {
		if (++rx_silent >= 4) {
			rx_silent = 0;
			ud.rx_rearms++;
			ut("RRA", ud.rx_rearms, ((volatile uint32_t *)(ud.dma + UD_TRB_RX))[2]);
			ud_end_transfer(EP_RX);
			ud_rx_arm();
		}
	} else {
		rx_silent = 0;
	}
	ud.watch_rx = ud.rx_frames;

	lck_mtx_lock(ud.lock);
	ud_sync(UD_TRB_TX, 16);
	bool tx_idle = ud.tx_busy && !(tt[3] & TRB_HWO) && ud.tx_frames == ud.watch_tx;
	if (tx_idle && tx_seen) {
		if (ud.lost_tx++ < 20) {
			printf("SWEET USB: tx transfer done without an event (%u), going on\n", ud.lost_tx);
		}
		ud.xfer_active[EP_TX] = false;
		ud.tx_busy = false;
		ud_tx_kick_locked();
		tx_idle = false;
	}
	tx_seen = tx_idle;
	ud.watch_tx = ud.tx_frames;
	lck_mtx_unlock(ud.lock);
}

static int
sysctl_sweet_usb_stats SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	char buf[320];
	uint32_t rx3 = 0, tx3 = 0, dsts = 0;
	if (ud.dma != 0) {
		uint32_t rxo = ud_dp ? DP_TRB_RX : UD_TRB_RX, txo = ud_dp ? DP_TRB_TX : UD_TRB_TX;
		ud_sync(rxo, 16);
		ud_sync(txo, 16);
		rx3 = ((volatile uint32_t *)(ud.dma + rxo))[3];
		tx3 = ((volatile uint32_t *)(ud.dma + txo))[3];
		dsts = dwc_rd(DSTS);
	}
	snprintf(buf, sizeof(buf), "configured %d events %u resets %u; rx %llu frames (trb %08x, active %d) tx %llu "
	    "frames (trb %08x, busy %d, queue %u); without event: rx %u tx %u, late rx %u; rx rearms %u; halt clears %u; DSTS %08x",
	    ud.configured, ud.events, ud.resets, ud.rx_frames, rx3, ud.xfer_active[EP_RX], ud.tx_frames, tx3,
	    ud.tx_busy, ud.txq_count, ud.lost_rx, ud.lost_tx, ud.stale_rx, ud.rx_rearms, ud.halt_clears, dsts);
	return SYSCTL_OUT(req, buf, strlen(buf) + 1);
}
static int
sysctl_sweet_usb_trace SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	static char out[UT_N * 40];
	size_t n = 0;
	uint32_t start = ut_pos > UT_N ? ut_pos - UT_N : 0;
	for (uint32_t i = start; i < ut_pos && n + 48 < sizeof(out); i++) {
		struct ut_ent *e = &ut_ring[i % UT_N];
		n += (size_t)snprintf(out + n, sizeof(out) - n, "%u %s %08x %08x\n", e->ms, e->what, e->a, e->b);
	}
	out[n] = 0;
	return SYSCTL_OUT(req, out, n + 1);
}
SYSCTL_PROC(_kern, OID_AUTO, sweet_usb_trace, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED,
    0, 0, sysctl_sweet_usb_trace, "A", "USB gadget trace: ms what a b");

SYSCTL_PROC(_kern, OID_AUTO, sweet_usb_stats, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED,
    0, 0, sysctl_sweet_usb_stats, "A", "USB Ethernet gadget state");

/*
 * 0327, old data path only (sweetusb_reclaim=0 turns it off): on every poll, a bulk transfer
 * whose TRB has HWO clear is taken as complete without waiting for its XferComplete. That is
 * not safe. Events are not lost under load, they arrive late (hwo0_noevt_polls,
 * investigacion/03 carga-1), and the handler does not ignore the late one: ud_event clears
 * xfer_active/tx_busy and kicks again (audit 01 §1.1 I1, I5). The same completion is consumed
 * twice and the next Start Transfer is refused with status 1 (No Resource): the dead link of
 * E-carga-1. HWO=0 does not make a TRB available (Linux 51f1954ad853). Data path v2 (the
 * default) does not reclaim on HWO (R2).
 */
static void
ud_reclaim_fast(void)
{
	volatile uint32_t *rt = (volatile uint32_t *)(ud.dma + UD_TRB_RX);
	volatile uint32_t *tt = (volatile uint32_t *)(ud.dma + UD_TRB_TX);

	if (!ud_reclaim_on || !ud.configured || ud.lock == NULL) {
		return;
	}
	ud_sync(UD_TRB_RX, 16);
	if (ud.xfer_active[EP_RX] && !(rt[3] & TRB_HWO)) {
		ud.xfer_active[EP_RX] = false;
		ud_rx_done();           /* copies the frame and re-arms RX */
	}
	lck_mtx_lock(ud.lock);
	ud_sync(UD_TRB_TX, 16);
	if (ud.tx_busy && !(tt[3] & TRB_HWO)) {
		ud.xfer_active[EP_TX] = false;
		ud.tx_busy = false;
		ud_tx_kick_locked();    /* send the next queued frame */
	}
	lck_mtx_unlock(ud.lock);
}

/*
 * 0330: data path v2, the default (sweetusb_dp unset or 2). With sweetusb_dp set to any other value
 * nothing below runs and the old path (ud_thread) is used (same kernel for the A/B). Audit 01 (2026-11-14): §1 mechanism of the
 * dead link (observation B), §2 defects D1-D9, D13, D14, D16, §5 corrected design.
 *
 *  R1 Only ud_thread (here ud_dp_thread) touches the controller, the data TRBs and the endpoint state.
 *     ud_if_output only queues the mbuf and wakes the thread (D14).
 *  R2 One ownership state per data endpoint (RX 0x01, TX 0x81, INT 0x82), dp.e[ep].st:
 *       IDLE    -> STARTED  Start Transfer issued (accepted; or refused/timed out: then the controller
 *                           holds *some* transfer on the endpoint and it is ours to end)
 *       STARTED -> IDLE     XferComplete for the endpoint whose TRB has HWO=0
 *       STARTED -> ENDING   End Transfer issued
 *       ENDING  -> IDLE     the End Transfer completed (CMDACT clear, GUCTL2[14]) and 1 ms more passed,
 *                           measured from before an event-buffer drain: events of the ended transfer
 *                           are dropped while ENDING, never taken as a later transfer's completion.
 *     HWO=0 alone never frees an endpoint: no 0327 reclaim, no ud_watch (D1; Linux 51f1954ad853).
 *  R3 A TRB or a data buffer is written only while its endpoint is IDLE (D2).
 *  R4 A refused Start keeps the frame, prints nothing, is counted with a snapshot and is followed by an
 *     End Transfer (D2, D3; Linux gadget.c 1653-1660, U-Boot -EBUSY).
 *  R5 Endpoint commands spin on mach_absolute_time, bounded; never delay() (it blocks in this port, D5),
 *     never with ud.lock held. ud.lock only guards the mbuf queue and `configured`.
 *  R6 Each data TRB in its own 64-byte line (D4) and the DMA area mapped Normal non-cacheable, the way
 *     Linux maps dma_alloc_coherent on arm64 (the cacheable physmap/kmem aliases are only cleaned once,
 *     never written again). sweetusb_dpnc=0 keeps it cacheable with clean+invalidate per line.
 *  R7 A transfer whose TRB was written back (HWO=0) while no event came for DP_NOEVT_POLLS polls and
 *     2 ms (the "alpha" signature, audit §1.7) is counted, snapshotted and ended; its frame is then
 *     accounted from the TRB. That is the only way out of an event that never comes, and it goes through End Transfer.
 * No printf on the data path: counters and latched snapshots (first 8 and the last of each kind) are in
 * sysctl kern.sweet_usb_dp.
 */
#define DP_GSNPSID      0xc120
#define DP_GUCTL2       0xc19c
#define DP_GUCTL2_RST_ACTBITLATER (1u << 14)    /* DWC_usb3 >= 3.10a (Linux core.c 1385-1392) */
#define DP_GUSB2PHYCFG  0xc200
#define DP_PHY_SUSP     ((1u << 6) | (1u << 8)) /* SUSPHY | ENBLSLPM (Linux gadget.c 320-345) */
#define DP_NOEVT_POLLS  8
#define DP_NSNAP        8

#ifndef VM_WIMG_WCOMB
#define VM_WIMG_WCOMB   0x6             /* osfmk/arm/memory_types.h: Normal, inner/outer non-cacheable */
#endif
extern vm_offset_t ml_io_map_unmappable(vm_offset_t phys_addr, vm_size_t size, uint32_t flags);

enum { DP_IDLE = 0, DP_STARTED, DP_ENDING };
enum { DPK_START_REFUSED, DPK_START_TIMEOUT, DPK_NOEVT, DPK_EVT_NOT_STARTED, DPK_EVT_HWO_SET,
       DPK_END_TIMEOUT, DPK_EVCOUNT, DPK_N };
static const char *const dp_kind[DPK_N] = {
	"start-refused", "start-timeout", "noevt-alpha", "evt-not-started", "evt-hwo-set",
	"end-timeout", "evcount-anomaly",
};

struct dp_snap {
	uint64_t us;                    /* since boot */
	uint32_t ep, st, depcmd, trb[4], dsts, evcnt, a;
};

struct dp_ep {
	uint8_t  st;
	uint8_t  rsc;                   /* resource index for End Transfer (0 after an End, as Linux) */
	uint8_t  rsc_last;              /* of the last accepted Start */
	bool     end_wanted;            /* STARTED: End Transfer to issue as soon as ep0 allows it */
	bool     end_cmdact;            /* ENDING: End Transfer issued, CMDACT not seen clear yet */
	bool     hwo0_at_end;           /* the TRB had been written back when the End completed */
	bool     evt_seen;              /* an XferComplete for this endpoint in the current drain */
	uint32_t trb, buf;              /* DMA offsets */
	uint32_t noevt_polls;
	uint64_t noevt_since, end_t0, ready_at;
};

static struct {
	bool     nc, actbitlater, notify_pending;
	uint32_t gsnpsid, guctl2;
	int      chan;                  /* wait channel of ud_dp_thread only */
	int      kick;                  /* set by ud_dp_output before waking the thread */
	uint32_t tx_len;                /* frame staged in UD_TX (0: none); only ud_dp_thread */
	uint64_t abs_cmd, abs_end, abs_spin, abs_125us, abs_1ms, abs_2ms, abs_10ms, abs_30s;
	uint64_t last_xfer, last_poll, last_report;
	struct dp_ep e[EP_INT + 1];
	struct {
		uint64_t polls, spins, poll_max_us;
		uint64_t start_ok, start_refused, start_timeout, start_not_idle, cmd_busy, phy_susp_cleared;
		uint64_t end_issued, end_deferred, end_timeout, end_status, end_lat_max_us, end_lat_last_us, end_lat_sum_us;
		uint64_t noevt, hwo0_noevt_polls, evt_not_started, evt_hwo_set, evt_other, evt_buserr;
		uint64_t evcount_over, evcount_bits;
		uint64_t tx_done, tx_done_by_end, tx_qdrop, tx_bad, tx_reset_drop;
		uint64_t rx_done, rx_by_end, rx_bad_len, rx_nombuf, int_done, notify_skipped;
		uint64_t clearstall_fail, bus_resets;
	} c;
	struct {
		uint32_t seq, n;
		struct dp_snap first[DP_NSNAP], last;
	} latch[DPK_N];
} dp;

static void dp_end(unsigned ep, bool force);

static inline bool
dp_is_data(unsigned ep)
{
	return ep == EP_RX || ep == EP_TX || ep == EP_INT;
}

static inline volatile uint32_t *
dp_trb(unsigned ep)
{
	return (volatile uint32_t *)(ud.dma + dp.e[ep].trb);
}

/* cacheable fallback only: every range given here is whole 64-byte lines nobody else uses */
static inline void
dp_sync(uint32_t off, uint32_t len)
{
	if (!dp.nc) {
		ud_sync(off, len);
	}
}

static uint64_t
dp_us(uint64_t abs)
{
	uint64_t ns;
	absolutetime_to_nanoseconds(abs, &ns);
	return ns / 1000;
}

/* counter-side snapshot: the first DP_NSNAP of each kind stay, the last is overwritten (seqlock for the sysctl) */
static void
dp_snap(unsigned k, unsigned ep, uint32_t depcmd, uint32_t a)
{
	struct dp_snap s;

	bzero(&s, sizeof(s));
	s.us = dp_us(mach_absolute_time());
	s.ep = ep;
	s.depcmd = depcmd;
	s.a = a;
	if (dp_is_data(ep)) {
		volatile uint32_t *t = dp_trb(ep);
		dp_sync(dp.e[ep].trb, 16);
		s.st = dp.e[ep].st;
		for (unsigned i = 0; i < 4; i++) {
			s.trb[i] = t[i];
		}
	}
	s.dsts = dwc_rd(DSTS);
	s.evcnt = dwc_rd(GEVNTCOUNT);
	__atomic_add_fetch(&dp.latch[k].seq, 1, __ATOMIC_ACQ_REL);
	if (dp.latch[k].n < DP_NSNAP) {
		dp.latch[k].first[dp.latch[k].n] = s;
	}
	dp.latch[k].last = s;
	dp.latch[k].n++;
	__atomic_add_fetch(&dp.latch[k].seq, 1, __ATOMIC_ACQ_REL);
}

/*
 * One endpoint command, polled by spinning. 0 accepted, 1-15 the command status (1 = No Resource),
 * -1 CMDACT still set after `limit` (still in progress), -2 not issued: the previous command on the
 * endpoint is still active.
 */
static int
dp_cmd(unsigned ep, uint32_t cmd, uint32_t p0, uint32_t p1, uint32_t p2, uint64_t limit, uint32_t *depcmd)
{
	uint32_t v = dwc_rd(DEPCMD(ep));

	if (v & DEPCMD_CMDACT) {
		dp.c.cmd_busy++;
		*depcmd = v;
		return -2;
	}
	uint32_t phy = dwc_rd(DP_GUSB2PHYCFG);
	if (phy & DP_PHY_SUSP) {        /* 0328 clears them once at start; keep them clear for every command */
		dwc_wr(DP_GUSB2PHYCFG, phy & ~DP_PHY_SUSP);
		dp.c.phy_susp_cleared++;
	}
	dwc_wr(DEPCMDPAR0(ep), p0);
	dwc_wr(DEPCMDPAR1(ep), p1);
	dwc_wr(DEPCMDPAR2(ep), p2);
	dwc_wr(DEPCMD(ep), cmd | DEPCMD_CMDACT);
	uint64_t end = mach_absolute_time() + limit;
	do {
		v = dwc_rd(DEPCMD(ep));
	} while ((v & DEPCMD_CMDACT) && mach_absolute_time() < end);
	if (v & DEPCMD_CMDACT) {
		v = dwc_rd(DEPCMD(ep));         /* preempted past the limit right after a read */
	}
	*depcmd = v;
	if (v & DEPCMD_CMDACT) {
		return -1;
	}
	return (int)((v >> 12) & 0xf);
}

/* R3/R4: start the endpoint's only TRB. 0 accepted; otherwise the frame/buffer stays as it is. */
static int
dp_start(unsigned ep, uint32_t len)
{
	struct dp_ep *e = &dp.e[ep];
	volatile uint32_t *t = dp_trb(ep);
	uint64_t bpa = ud.dma_pa + e->buf, tpa = ud.dma_pa + e->trb;
	uint32_t dc = 0;
	int r;

	if (e->st != DP_IDLE) {
		dp.c.start_not_idle++;
		return -1;
	}
	t[0] = (uint32_t)bpa;
	t[1] = (uint32_t)(bpa >> 32);
	t[2] = len;
	t[3] = TRBCTL_NORMAL | TRB_HWO | TRB_LST | TRB_ISP_IMI | TRB_IOC;      /* the same bits as ud_start_trb */
	dp_sync(e->trb, 16);
	dp_sync(e->buf, ep == EP_RX ? UD_FRAME_MAX : len);
	__asm__ volatile ("dsb sy" ::: "memory");      /* TRB and data in memory before the controller reads them */
	r = dp_cmd(ep, DEPCMD_STARTTRANSFER, (uint32_t)(tpa >> 32), (uint32_t)tpa, 0, dp.abs_cmd, &dc);
	if (r == 0) {
		e->rsc = e->rsc_last = (uint8_t)((dc >> 16) & 0x7f);
		e->st = DP_STARTED;
		e->end_wanted = false;
		e->noevt_polls = 0;
		dp.c.start_ok++;
		return 0;
	}
	if (r == -2) {                  /* cannot happen in IDLE (ENDING -> IDLE waits for CMDACT): nothing started */
		return -1;
	}
	/*
	 * Refused (1 = No Resource: the controller still holds a transfer on this endpoint, Linux commit
	 * f7ac582effc6 cause 3) or timed out. From here the controller may own the TRB: the endpoint is
	 * STARTED, the frame stays staged, and what the controller holds is ended. Linux reads the resource
	 * index after a refused Start too (gadget.c 433-434); a timed-out one is read again before the End.
	 */
	if (r > 0) {
		dp.c.start_refused++;
		dp_snap(DPK_START_REFUSED, ep, dc, (uint32_t)r);
		e->rsc = (uint8_t)((dc >> 16) & 0x7f);
		if (e->rsc == 0) {
			e->rsc = e->rsc_last;
		}
	} else {
		dp.c.start_timeout++;
		dp_snap(DPK_START_TIMEOUT, ep, dc, 0);
		e->rsc = 0;
	}
	e->st = DP_STARTED;
	e->end_wanted = true;
	e->noevt_polls = 0;
	dp_end(ep, false);
	return -1;
}

/* The End Transfer has completed: nobody owns the TRB any more. */
static void
dp_end_done(unsigned ep)
{
	struct dp_ep *e = &dp.e[ep];
	volatile uint32_t *t = dp_trb(ep);
	uint64_t now = mach_absolute_time(), lat = dp_us(now - e->end_t0);

	e->end_cmdact = false;
	dp.c.end_lat_last_us = lat;
	dp.c.end_lat_sum_us += lat;
	if (lat > dp.c.end_lat_max_us) {
		dp.c.end_lat_max_us = lat;
	}
	dp_sync(e->trb, 16);
	e->hwo0_at_end = !(t[3] & TRB_HWO);
	t[3] &= ~TRB_HWO;               /* ForceRM leaves HWO set (Linux gadget.c 1687-1695) */
	dp_sync(e->trb, 16);
	e->rsc = 0;
	/* Linux waits 1 ms after a non-IOC End Transfer (gadget.c 441-443); also the window in which a
	 * stale XferComplete of the ended transfer is drained and dropped */
	e->ready_at = now + dp.abs_1ms;
}

/*
 * R2: End Transfer, ForceRM, no IOC; with GUCTL2[14] CMDACT stays set until it has completed
 * (Linux gadget.c 3897-3925). A SETUP waiting to be DMAed blocks every End Transfer (gadget.c
 * 3886-3897): outside a forced call (bus reset, or from inside the SETUP being handled) it is only
 * issued while ep0 is armed for SETUP, else deferred to a later poll.
 */
static void
dp_end(unsigned ep, bool force)
{
	struct dp_ep *e = &dp.e[ep];
	uint32_t dc = 0;
	int r;

	if (e->st != DP_STARTED) {
		return;
	}
	e->end_wanted = true;
	if (!force && ud.state != EP0_SETUP) {
		dp.c.end_deferred++;
		return;
	}
	if (e->rsc == 0) {
		uint32_t v = dwc_rd(DEPCMD(ep));
		e->rsc = (v & DEPCMD_CMDACT) ? 0 : (uint8_t)((v >> 16) & 0x7f);
		if (e->rsc == 0) {
			e->rsc = e->rsc_last;
		}
	}
	e->end_t0 = mach_absolute_time();
	r = dp_cmd(ep, DEPCMD_ENDTRANSFER | DEPCMD_HIPRI_FORCERM | ((uint32_t)e->rsc << 16), 0, 0, 0, dp.abs_end, &dc);
	if (r == -2) {                  /* a timed-out Start is still running: next poll */
		return;
	}
	dp.c.end_issued++;
	e->st = DP_ENDING;
	e->end_wanted = false;
	e->noevt_polls = 0;
	if (r == -1) {
		e->end_cmdact = true;
		dp.c.end_timeout++;
		dp_snap(DPK_END_TIMEOUT, ep, dc, e->rsc);
		return;
	}
	if (r > 0) {
		dp.c.end_status++;
	}
	dp_end_done(ep);
}

/* control path only (bus reset, clear halt): wait, bounded, for an End Transfer issued above */
static void
dp_wait_end(unsigned ep)
{
	struct dp_ep *e = &dp.e[ep];
	uint64_t end = mach_absolute_time() + dp.abs_end;

	while (e->st == DP_ENDING && e->end_cmdact && mach_absolute_time() < end) {
		if (!(dwc_rd(DEPCMD(ep)) & DEPCMD_CMDACT)) {
			dp_end_done(ep);
		}
	}
}

static void
dp_rx_frame(void)
{
	volatile uint32_t *t = dp_trb(EP_RX);
	uint32_t len = UD_FRAME_MAX - (t[2] & 0xffffff);
	mbuf_t m = NULL;

	dp.last_xfer = mach_absolute_time();
	if (ud.ifp == NULL || len < ETHER_HDR_LEN || len > ETHER_MAX_LEN) {
		dp.c.rx_bad_len++;
		return;
	}
	dp_sync(UD_RX, len);
	/* as ud_rx_done: one mbuf with a 2 KiB cluster */
	if (mbuf_gethdr(MBUF_DONTWAIT, MBUF_TYPE_DATA, &m) == 0 &&
	    mbuf_mclget(MBUF_DONTWAIT, MBUF_TYPE_DATA, &m) == 0 &&
	    (mbuf_flags(m) & MBUF_EXT) && mbuf_maxlen(m) >= len) {
		struct ifnet_stat_increment_param stats = {};
		memcpy(mbuf_data(m), (const void *)(ud.dma + UD_RX), len);
		mbuf_setlen(m, len);
		mbuf_pkthdr_setlen(m, len);
		void *frame_header = mbuf_data(m);
		mbuf_pkthdr_setrcvif(m, ud.ifp);
		mbuf_pkthdr_setheader(m, frame_header);
		mbuf_pkthdr_adjustlen(m, -ETHER_HDR_LEN);
		(void)mbuf_setdata(m, (char *)mbuf_data(m) + ETHER_HDR_LEN, mbuf_len(m) - ETHER_HDR_LEN);
		stats.packets_in = 1;
		stats.bytes_in = len;
		ud.rx_frames++;
		ifnet_input(ud.ifp, m, &stats);
	} else {
		dp.c.rx_nombuf++;
		if (m != NULL) {
			mbuf_freem(m);
		}
	}
}

/* STARTED -> IDLE on its XferComplete with HWO=0 */
static void
dp_complete(unsigned ep)
{
	if (ep == EP_TX) {
		dp.tx_len = 0;
		ud.tx_frames++;
		dp.c.tx_done++;
		dp.last_xfer = mach_absolute_time();
	} else if (ep == EP_RX) {
		dp.c.rx_done++;
		dp_rx_frame();
	} else {
		dp.c.int_done++;
	}
}

/* ENDING -> IDLE: a TRB written back before the End is a transfer that did happen */
static void
dp_ended(unsigned ep)
{
	struct dp_ep *e = &dp.e[ep];

	if (!e->hwo0_at_end) {
		return;                 /* not done: TX sends the staged frame again, RX arms again */
	}
	e->hwo0_at_end = false;
	if (ep == EP_TX && dp.tx_len != 0) {
		dp.tx_len = 0;
		ud.tx_frames++;
		dp.c.tx_done_by_end++;
	} else if (ep == EP_RX) {
		dp.c.rx_by_end++;
		dp_rx_frame();
	}
}

/* from ud_event, ud_dp_thread only */
static void
ud_dp_ep_event(unsigned ep, unsigned type, unsigned status, uint32_t ev)
{
	if (!dp_is_data(ep) || type != 1) {     /* only XferComplete is enabled on the data endpoints */
		dp.c.evt_other++;
		return;
	}
	struct dp_ep *e = &dp.e[ep];
	volatile uint32_t *t = dp_trb(ep);

	e->evt_seen = true;
	if (e->st != DP_STARTED) {      /* of a transfer already ended: never a reason to start another */
		dp.c.evt_not_started++;
		dp_snap(DPK_EVT_NOT_STARTED, ep, dwc_rd(DEPCMD(ep)), ev);
		return;
	}
	dp_sync(e->trb, 16);
	if (t[3] & TRB_HWO) {           /* not the completion of the TRB armed now; R7 ends it if none comes */
		dp.c.evt_hwo_set++;
		dp_snap(DPK_EVT_HWO_SET, ep, dwc_rd(DEPCMD(ep)), ev);
		return;
	}
	if (status & 1) {
		dp.c.evt_buserr++;
	}
	e->st = DP_IDLE;
	e->end_wanted = false;
	e->noevt_polls = 0;
	dp_complete(ep);
}

static void
dp_tx_pump(void)
{
	if (dp.e[EP_TX].st != DP_IDLE) {
		return;
	}
	while (dp.tx_len == 0) {        /* R3: UD_TX is only written with the endpoint IDLE and nothing staged */
		mbuf_t m = NULL;
		lck_mtx_lock(ud.lock);
		if (ud.txq_count > 0) {
			m = ud.txq[ud.txq_head];
			ud.txq_head = (ud.txq_head + 1) % UD_TXQ;
			ud.txq_count--;
		}
		lck_mtx_unlock(ud.lock);
		if (m == NULL) {
			return;
		}
		size_t len = mbuf_pkthdr_len(m);
		if (len == 0 || len > ETHER_MAX_LEN - ETHER_CRC_LEN ||
		    mbuf_copydata(m, 0, len, (void *)(ud.dma + UD_TX)) != 0) {
			dp.c.tx_bad++;
			mbuf_freem(m);
			continue;
		}
		mbuf_freem(m);
		if ((len % 512) == 0) {         /* as ud_tx_kick_locked: one pad byte instead of a ZLP */
			((uint8_t *)(ud.dma + UD_TX))[len++] = 0;
		}
		dp.tx_len = (uint32_t)len;
	}
	(void)dp_start(EP_TX, dp.tx_len);       /* refused: the frame stays staged and goes after the End */
}

static void
dp_pump(uint64_t now0)
{
	static const unsigned eps[3] = { EP_RX, EP_TX, EP_INT };

	for (unsigned i = 0; i < 3; i++) {
		unsigned ep = eps[i];
		struct dp_ep *e = &dp.e[ep];

		if (e->st == DP_STARTED && e->end_wanted) {
			dp_end(ep, false);
		} else if (e->st == DP_STARTED) {
			volatile uint32_t *t = dp_trb(ep);
			dp_sync(e->trb, 16);
			if (!(t[3] & TRB_HWO) && !e->evt_seen) {        /* R7 */
				dp.c.hwo0_noevt_polls++;
				if (e->noevt_polls++ == 0) {
					e->noevt_since = now0;
				}
				if (e->noevt_polls >= DP_NOEVT_POLLS && now0 - e->noevt_since >= dp.abs_2ms) {
					dp.c.noevt++;
					dp_snap(DPK_NOEVT, ep, dwc_rd(DEPCMD(ep)), e->noevt_polls);
					dp_end(ep, false);
				}
			} else {
				e->noevt_polls = 0;
			}
		}
		if (e->st == DP_ENDING) {
			if (e->end_cmdact) {
				if (!(dwc_rd(DEPCMD(ep)) & DEPCMD_CMDACT)) {
					dp_end_done(ep);
				}
			} else if (now0 >= e->ready_at) {       /* now0 was taken before this poll's drain */
				e->st = DP_IDLE;
				dp_ended(ep);
			}
		}
		e->evt_seen = false;
	}
	if (!ud.configured) {
		return;
	}
	if (dp.e[EP_RX].st == DP_IDLE) {
		(void)dp_start(EP_RX, UD_FRAME_MAX);
	}
	if (dp.notify_pending && dp.e[EP_INT].st == DP_IDLE) {
		uint8_t *n = (uint8_t *)(ud.dma + UD_NOTIFY);   /* NETWORK_CONNECTION, connected, interface 0 */
		n[0] = 0xa1; n[1] = 0x00; n[2] = 1; n[3] = 0;
		n[4] = 0; n[5] = 0; n[6] = 0; n[7] = 0;
		if (dp_start(EP_INT, 8) == 0) {
			dp.notify_pending = false;
		}
	}
	dp_tx_pump();
}

/* one pass: drain the event buffer, then advance every data endpoint */
static void
dp_poll(uint64_t now0)
{
	uint32_t raw = dwc_rd(GEVNTCOUNT);
	uint32_t count = raw & 0xfffc;          /* Linux core.h 78 */
	uint32_t take = count;

	if ((raw & 0x7fff0003) != 0) {          /* bits outside the count and EVNT_HANDLER_BUSY */
		dp.c.evcount_bits++;
	}
	if (count > UD_EVT_SIZE) {              /* cannot happen with a 4 KiB buffer polled every ms: record it */
		dp.c.evcount_over++;
		dp_snap(DPK_EVCOUNT, 0, raw, ud.evt_pos);
		take = UD_EVT_SIZE;
	}
	if (count) {
		dp_sync(UD_EVT, UD_EVT_SIZE);
		for (uint32_t done = 0; done < take; done += 4) {
			uint32_t ev = *(volatile uint32_t *)(ud.dma + UD_EVT + ud.evt_pos);
			ud.evt_pos = (ud.evt_pos + 4) % UD_EVT_SIZE;
			/* the first 200 control/device events are printed as before; data events never */
			if (((ev & 1) || !dp_is_data((ev >> 1) & 0x1f)) && ud.events < 200) {
				printf("SWEET USB: event 0x%08x\n", ev);
			}
			ud.events++;
			ud_event(ev);
		}
		__asm__ volatile ("dmb sy" ::: "memory");      /* the event words are read before the slots are given back */
		dwc_wr(GEVNTCOUNT, count);
	}
	dp_pump(now0);
}

/* R1: no hardware here, from any thread */
static errno_t
ud_dp_output(ifnet_t ifp, mbuf_t m)
{
	lck_mtx_lock(ud.lock);
	if (ud.txq_count == UD_TXQ || !ud.configured) {
		dp.c.tx_qdrop++;
		lck_mtx_unlock(ud.lock);
		mbuf_freem(m);
		ifnet_stat_increment_out(ifp, 0, 0, 1);
		return ENOBUFS;         /* back-pressure; the log thread stops on it */
	}
	ifnet_stat_increment_out(ifp, 1, mbuf_pkthdr_len(m), 0);
	ud.txq[(ud.txq_head + ud.txq_count) % UD_TXQ] = m;
	ud.txq_count++;
	lck_mtx_unlock(ud.lock);
	__atomic_store_n(&dp.kick, 1, __ATOMIC_RELEASE);
	thread_wakeup(&dp.chan);
	return 0;
}

/* SET_INTERFACE: the CDC notification, sent by dp_pump once the interrupt endpoint is idle */
static void
ud_dp_notify(void)
{
	if (dp.e[EP_INT].st == DP_STARTED) {
		dp.c.notify_skipped++;          /* the one still armed says the same */
		return;
	}
	dp.notify_pending = true;
}

/*
 * CLEAR_FEATURE(ENDPOINT_HALT) / SET_INTERFACE (0289), from inside the SETUP being handled (ep0 is in
 * its SETUP phase for Linux there too): End Transfer, wait for it, then ClearStall (Linux commit
 * d97c78a1908e: CLEAR_STALL after END_TRANSFER completes). dp_pump arms the endpoint again after 1 ms.
 */
static void
ud_dp_reset_seq(unsigned ep)
{
	struct dp_ep *e = &dp.e[ep];
	uint32_t dc = 0;

	if (!dp_is_data(ep)) {
		return;
	}
	if (e->st == DP_STARTED) {
		dp_end(ep, true);
	}
	dp_wait_end(ep);
	if (e->st == DP_STARTED || (e->st == DP_ENDING && e->end_cmdact) ||
	    dp_cmd(ep, DEPCMD_CLEARSTALL, 0, 0, 0, dp.abs_cmd, &dc) != 0) {
		dp.c.clearstall_fail++;
	}
	if (ep == EP_TX && dp.tx_len != 0) {    /* the host cancelled it */
		dp.tx_len = 0;
		dp.c.tx_reset_drop++;
	}
}

/*
 * USB reset (D8): ep0 back to its SETUP phase first (Linux dwc3_ep0_reset_state, gadget.c 4035-4041),
 * then End Transfer for every active transfer (databook 3.30a §4.1.2 Table 4-2, gadget.c 4043-4049),
 * waited for, then ClearStall (gadget.c dwc3_clear_stall_all_ep), then forget the configuration.
 */
static void
ud_dp_bus_reset(void)
{
	static const unsigned eps[3] = { EP_RX, EP_TX, EP_INT };
	bool was_configured = ud.configured;

	dp.c.bus_resets++;
	if (ud.state != EP0_SETUP || !ud.xfer_active[0]) {
		ud_end_transfer(0);
		ud_end_transfer(1);
		ud_start_setup();
	}
	for (unsigned i = 0; i < 3; i++) {
		if (dp.e[eps[i]].st == DP_STARTED) {
			dp_end(eps[i], true);
		}
		dp_wait_end(eps[i]);
	}
	for (unsigned i = 0; was_configured && i < 3; i++) {
		struct dp_ep *e = &dp.e[eps[i]];
		uint32_t dc = 0;
		if (e->st == DP_STARTED || (e->st == DP_ENDING && e->end_cmdact) ||
		    dp_cmd(eps[i], DEPCMD_CLEARSTALL, 0, 0, 0, dp.abs_cmd, &dc) != 0) {
			dp.c.clearstall_fail++;
		}
	}
	dwc_wr(DALEPENA, dwc_rd(DALEPENA) & ~((1u << EP_RX) | (1u << EP_TX) | (1u << EP_INT)));
	lck_mtx_lock(ud.lock);
	ud.configured = false;
	ud.data_alt = 0;
	ud.tx_busy = false;
	while (ud.txq_count > 0) {
		mbuf_freem(ud.txq[ud.txq_head]);
		ud.txq_head = (ud.txq_head + 1) % UD_TXQ;
		ud.txq_count--;
	}
	lck_mtx_unlock(ud.lock);
	dp.tx_len = 0;
	dp.notify_pending = false;
}

static void
ud_dp_thread(void *arg __unused, wait_result_t wr __unused)
{
	for (;;) {
		uint64_t now0 = mach_absolute_time();

		if (dp.last_poll != 0) {
			uint64_t p = dp_us(now0 - dp.last_poll);
			if (p > dp.c.poll_max_us) {
				dp.c.poll_max_us = p;
			}
		}
		dp.last_poll = now0;
		dp.c.polls++;
		__atomic_store_n(&dp.kick, 0, __ATOMIC_RELAXED);
		dp_poll(now0);
		if (now0 - dp.last_report >= dp.abs_30s) {
			dp.last_report = now0;
			printf("SWEET USB: ue0 rx %llu tx %llu frames\n", ud.rx_frames, ud.tx_frames);
		}
		/*
		 * Frames moved in the last 10 ms: spin up to 50 us for the next event (a 1514-byte frame is
		 * ~30 us on the wire, and with one TRB per endpoint each poll moves at most one frame each
		 * way), then sleep 125 us. Otherwise sleep 1 ms; ud_dp_output wakes the thread at once.
		 */
		bool busy = mach_absolute_time() - dp.last_xfer < dp.abs_10ms;
		if (busy) {
			uint64_t until = mach_absolute_time() + dp.abs_spin;
			bool more;
			do {
				more = (dwc_rd(GEVNTCOUNT) & 0xfffc) != 0 || __atomic_load_n(&dp.kick, __ATOMIC_RELAXED);
			} while (!more && mach_absolute_time() < until);
			if (more) {
				dp.c.spins++;
				continue;
			}
		}
		(void)assert_wait_deadline(&dp.chan, THREAD_UNINT,
		    mach_absolute_time() + (busy ? dp.abs_125us : dp.abs_1ms));
		if (__atomic_load_n(&dp.kick, __ATOMIC_ACQUIRE)) {
			(void)clear_wait(current_thread(), THREAD_AWAKENED);    /* woken before the assert_wait */
		} else {
			(void)thread_block(THREAD_CONTINUE_NULL);
		}
	}
}

/* before ud_if_create: DMA mapping (R6), TRB layout, time constants */
static void
ud_dp_init(void)
{
	int nc = 1;

	nanoseconds_to_absolutetime(2 * NSEC_PER_MSEC, &dp.abs_cmd);
	nanoseconds_to_absolutetime(5 * NSEC_PER_MSEC, &dp.abs_end);
	nanoseconds_to_absolutetime(50 * NSEC_PER_USEC, &dp.abs_spin);
	nanoseconds_to_absolutetime(125 * NSEC_PER_USEC, &dp.abs_125us);
	nanoseconds_to_absolutetime(NSEC_PER_MSEC, &dp.abs_1ms);
	nanoseconds_to_absolutetime(2 * NSEC_PER_MSEC, &dp.abs_2ms);
	nanoseconds_to_absolutetime(10 * NSEC_PER_MSEC, &dp.abs_10ms);
	nanoseconds_to_absolutetime(30 * NSEC_PER_SEC, &dp.abs_30s);
	dp.e[EP_RX].trb = DP_TRB_RX;
	dp.e[EP_RX].buf = UD_RX;
	dp.e[EP_TX].trb = DP_TRB_TX;
	dp.e[EP_TX].buf = UD_TX;
	dp.e[EP_INT].trb = DP_TRB_INT;
	dp.e[EP_INT].buf = UD_NOTIFY;
	PE_parse_boot_argn("sweetusb_dpnc", &nc, sizeof(nc));
	if (nc) {
		/* the zeros of KMA_ZERO reach DRAM and no line of the area stays in the cache; from here on
		 * the CPU only uses the non-cacheable alias (the ud_sync() of the ep0 code then cleans and
		 * invalidates clean lines of the physmap alias: harmless) */
		flush_dcache64((addr64_t)ud.dma_pa, UD_SIZE, TRUE);
		vm_offset_t wc = ml_io_map_unmappable((vm_offset_t)ud.dma_pa, UD_SIZE, VM_WIMG_WCOMB);
		if (wc != 0) {
			ud.dma = wc;
			dp.nc = true;
		}
	}
	printf("SWEET USB: data path v2 (sweetusb_dp=2), DMA %s\n", dp.nc ? "non-cacheable" : "cacheable");
}

/* after the device soft reset: End Transfer polling (GUCTL2[14], DWC_usb3 >= 3.10a) */
static void
ud_dp_core_init(void)
{
	dp.gsnpsid = dwc_rd(DP_GSNPSID);
	if ((dp.gsnpsid & 0xffff0000) == 0x55330000 && (dp.gsnpsid & 0xffff) >= 0x310a) {
		dwc_wr(DP_GUCTL2, dwc_rd(DP_GUCTL2) | DP_GUCTL2_RST_ACTBITLATER);
	}
	dp.guctl2 = dwc_rd(DP_GUCTL2);
	dp.actbitlater = (dp.guctl2 & DP_GUCTL2_RST_ACTBITLATER) != 0;
	printf("SWEET USB: GSNPSID 0x%08x GUCTL2 0x%08x (End Transfer polling %s)\n", dp.gsnpsid, dp.guctl2,
	    dp.actbitlater ? "on" : "off");
}

static int
sysctl_sweet_usb_dp SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	static char out[12288];
	static int busy;
	size_t n = 0;
	int err;

#define DPF(...) do { if (n < sizeof(out)) { n += (size_t)snprintf(out + n, sizeof(out) - n, __VA_ARGS__); } } while (0)
	if (__atomic_exchange_n(&busy, 1, __ATOMIC_ACQUIRE)) {
		return EBUSY;
	}
	DPF("dp %d nc %d actbitlater %d gsnpsid %08x guctl2 %08x configured %d ep0state %d tx_len %u queue %u\n",
	    ud_dp, dp.nc, dp.actbitlater, dp.gsnpsid, dp.guctl2, ud.configured, (int)ud.state, dp.tx_len, ud.txq_count);
	for (unsigned ep = EP_RX; ud_dp && ud.dma != 0 && ep <= EP_INT; ep++) {
		if (!dp_is_data(ep)) {
			continue;
		}
		volatile uint32_t *t = dp_trb(ep);
		dp_sync(dp.e[ep].trb, 16);
		DPF("ep%u st %u rsc %u end_wanted %d end_cmdact %d noevt_polls %u trb %08x %08x %08x %08x\n", ep,
		    dp.e[ep].st, dp.e[ep].rsc, dp.e[ep].end_wanted, dp.e[ep].end_cmdact, dp.e[ep].noevt_polls,
		    t[0], t[1], t[2], t[3]);
	}
	DPF("polls %llu spins %llu poll_max_us %llu\n", dp.c.polls, dp.c.spins, dp.c.poll_max_us);
	DPF("start ok %llu refused %llu timeout %llu not_idle %llu cmd_busy %llu phy_susp_cleared %llu\n",
	    dp.c.start_ok, dp.c.start_refused, dp.c.start_timeout, dp.c.start_not_idle, dp.c.cmd_busy,
	    dp.c.phy_susp_cleared);
	DPF("end issued %llu deferred %llu timeout %llu status %llu lat_us max %llu last %llu sum %llu\n",
	    dp.c.end_issued, dp.c.end_deferred, dp.c.end_timeout, dp.c.end_status, dp.c.end_lat_max_us,
	    dp.c.end_lat_last_us, dp.c.end_lat_sum_us);
	DPF("noevt(alpha) %llu hwo0_noevt_polls %llu evt not_started %llu hwo_set %llu other %llu buserr %llu "
	    "evcount over %llu bits %llu\n", dp.c.noevt, dp.c.hwo0_noevt_polls, dp.c.evt_not_started,
	    dp.c.evt_hwo_set, dp.c.evt_other, dp.c.evt_buserr, dp.c.evcount_over, dp.c.evcount_bits);
	DPF("tx done %llu by_end %llu qdrop %llu bad %llu reset_drop %llu; rx done %llu by_end %llu bad_len %llu "
	    "nombuf %llu; int done %llu notify_skipped %llu; clearstall_fail %llu bus_resets %llu\n",
	    dp.c.tx_done, dp.c.tx_done_by_end, dp.c.tx_qdrop, dp.c.tx_bad, dp.c.tx_reset_drop, dp.c.rx_done,
	    dp.c.rx_by_end, dp.c.rx_bad_len, dp.c.rx_nombuf, dp.c.int_done, dp.c.notify_skipped,
	    dp.c.clearstall_fail, dp.c.bus_resets);
	for (unsigned k = 0; k < DPK_N; k++) {
		static struct dp_snap first[DP_NSNAP], last;
		uint32_t cnt = 0, s1, s2;
		for (int tries = 0; tries < 4; tries++) {
			s1 = __atomic_load_n(&dp.latch[k].seq, __ATOMIC_ACQUIRE);
			cnt = dp.latch[k].n;
			memcpy(first, dp.latch[k].first, sizeof(first));
			last = dp.latch[k].last;
			__atomic_thread_fence(__ATOMIC_ACQUIRE);
			s2 = __atomic_load_n(&dp.latch[k].seq, __ATOMIC_ACQUIRE);
			if (s1 == s2 && !(s1 & 1)) {
				break;
			}
		}
		if (cnt == 0) {
			continue;
		}
		DPF("%s n %u (us ep st depcmd trb0-3 dsts evcnt a)\n", dp_kind[k], cnt);
		for (uint32_t i = 0; i < cnt && i < DP_NSNAP; i++) {
			DPF(" #%u %llu %u %u %08x %08x %08x %08x %08x %08x %08x %08x\n", i, first[i].us, first[i].ep,
			    first[i].st, first[i].depcmd, first[i].trb[0], first[i].trb[1], first[i].trb[2], first[i].trb[3],
			    first[i].dsts, first[i].evcnt, first[i].a);
		}
		DPF(" last %llu %u %u %08x %08x %08x %08x %08x %08x %08x %08x\n", last.us, last.ep, last.st, last.depcmd,
		    last.trb[0], last.trb[1], last.trb[2], last.trb[3], last.dsts, last.evcnt, last.a);
	}
#undef DPF
	if (n >= sizeof(out)) {
		n = sizeof(out) - 1;
	}
	out[n] = 0;
	err = SYSCTL_OUT(req, out, n + 1);
	__atomic_store_n(&busy, 0, __ATOMIC_RELEASE);
	return err;
}
SYSCTL_PROC(_kern, OID_AUTO, sweet_usb_dp, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED,
    0, 0, sysctl_sweet_usb_dp, "A", "USB gadget data path v2 (sweetusb_dp=2): state, counters, snapshots");

static void
ud_thread(void *arg __unused, wait_result_t wr __unused)
{
	uint32_t ticks = 0;
	for (;;) {
		uint32_t count = dwc_rd(GEVNTCOUNT) & 0xffff;
		if (count) {
			ud_sync(UD_EVT, UD_EVT_SIZE);
			for (uint32_t done = 0; done < count; done += 4) {
				uint32_t ev = *(volatile uint32_t *)(ud.dma + UD_EVT + ud.evt_pos);
				ud.evt_pos = (ud.evt_pos + 4) % UD_EVT_SIZE;
				if (ud.events++ < 200) {
					printf("SWEET USB: event 0x%08x\n", ev);
				}
				ud_event(ev);
			}
			dwc_wr(GEVNTCOUNT, count);
		}
		ud_reclaim_fast();
		if ((ticks % 500) == 0) {
			ud_watch();
		}
		if ((++ticks % 30000) == 0) {
			printf("SWEET USB: ue0 rx %llu tx %llu frames\n", ud.rx_frames, ud.tx_frames);
		}
		ud_sleep_ms(1);
	}
}

/*
 * Live console log over USB Ethernet: every 100 ms the new bytes of the
 * console ring (bsd/dev/sweet_ufs.c) go as UDP datagrams to
 * 192.168.7.1:5140 (on the PC: nc -ukl 5140). Starts with the last 64 KiB
 * so the boot is included. Nothing here may print (it would feed itself).
 * Boot-arg sweetusblog=0 disables it.
 */
extern uint32_t sweet_log_copy(uint64_t *pos, char *out, uint32_t max);
extern uint64_t sweet_log_position(void);

static void
ud_log_thread(void *arg __unused, wait_result_t wr __unused)
{
	static char chunk[1400];
	socket_t so = NULL;
	struct sockaddr_in to;
	uint64_t pos = sweet_log_position();

	pos = pos > 65536 ? pos - 65536 : 0;
	bzero(&to, sizeof(to));
	to.sin_len = sizeof(to);
	to.sin_family = AF_INET;
	to.sin_port = htons(5140);
	to.sin_addr.s_addr = htonl(0xc0a80701);         /* 192.168.7.1 */
	for (;;) {
		ud_sleep_ms(100);
		if (!ud.configured) {
			continue;
		}
		if (so == NULL && sock_socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP, NULL, NULL, &so) != 0) {
			so = NULL;
			continue;
		}
		uint32_t n, sent_n = 0;
		while ((n = sweet_log_copy(&pos, chunk, sizeof(chunk))) > 0) {
			struct iovec iov = { .iov_base = chunk, .iov_len = n };
			struct msghdr msg = {
				.msg_name = &to, .msg_namelen = sizeof(to),
				.msg_iov = &iov, .msg_iovlen = 1,
			};
			size_t sent = 0;
			if (sock_send(so, &msg, 0, &sent) != 0) {
				break;          /* no route/ARP yet: retry later */
			}
			if (ud_dp && ++sent_n >= 16) {
				break;          /* 0330: at most 16 datagrams per 100 ms, so it never crowds out TX */
			}
		}
	}
}

static void
sweet_usb_gadget_start(vm_offset_t dwc)
{
	thread_t thread;

	ud.dwc = dwc;
	{
		/* sweetusb_dp: default 2, applied here where the boot-arg is read; ud_dp stays 0 until
		 * this point. Any value other than 2 selects the old data path. */
		int want = 2;
		PE_parse_boot_argn("sweetusb_dp", &want, sizeof(want));
		ud_dp = (want == 2) ? 2 : 0;
	}
	if (kmem_alloc_contig(kernel_map, &ud.dma, UD_SIZE, PAGE_MASK, 0, 0,
	    KMA_DATA | KMA_ZERO | KMA_PERMANENT, VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		printf("SWEET USB: no DMA memory\n");
		return;
	}
	ppnum_t pn = pmap_find_phys(kernel_pmap, (addr64_t)ud.dma);
	if (!pn) {
		return;
	}
	ud.dma_pa = ptoa_64(pn);
	if (ud_dp) {
		ud_dp_init();
	}
	ud_if_create();

	/*
	 * 0331 (sweetusb_utmipipe, default 1; 0 disables): use the UTMI clock as the
	 * PIPE clock, as Linux dwc3-qcom does for an HS-only controller (dwc3_qcom_select_utmi_clk,
	 * binding qcom,select-utmi-as-pipe-clk: "disable USB3 pipe_clk requirement"). 0329 showed that
	 * on a normal boot GCC gcc_usb3_prim_phy_pipe_clk (CBCR 0x1a058) reads 0x80000001 (enabled but
	 * off: no USB3 PHY running) while it runs after `fastboot continue` (0x00000001), the femto HS
	 * PHY being identical on both paths, and QSCRATCH GENERAL_CFG 0x4 on both (never selected).
	 */
	{
		/* 0332: on by default (5/5 normal boots enumerate with it, 0/3 without); sweetusb_utmipipe=0 disables */
		int utmipipe = 1;
		PE_parse_boot_argn("sweetusb_utmipipe", &utmipipe, sizeof(utmipipe));
		if (utmipipe) {
			vm_offset_t qsg = ml_io_map(QSCRATCH_PA, 0x100);
			if (qsg != 0) {
				volatile uint32_t *gcfg = (volatile uint32_t *)(qsg + 0x08);    /* QSCRATCH_GENERAL_CFG */
				uint32_t before = *gcfg;
				*gcfg = *gcfg | (1u << 8);                      /* PIPE_UTMI_CLK_DIS */
				__asm__ volatile ("dsb sy" ::: "memory");
				delay(1000);
				*gcfg = *gcfg | (1u << 0) | (1u << 3);          /* PIPE_UTMI_CLK_SEL | PIPE3_PHYSTATUS_SW */
				__asm__ volatile ("dsb sy" ::: "memory");
				delay(1000);
				*gcfg = *gcfg & ~(1u << 8);                     /* clear PIPE_UTMI_CLK_DIS */
				__asm__ volatile ("dsb sy" ::: "memory");
				delay(1000);
				printf("SWEET USBDIAG utmipipe=%d GENERAL_CFG 0x%08x -> 0x%08x\n", utmipipe, before, *gcfg);
			} else {
				printf("SWEET USBDIAG utmipipe: QSCRATCH map failed\n");
			}
		}
	}
	/* Device controller soft reset. The PHY and the global registers are not programmed here (only
	 * GUSB2PHYCFG below and, in v2, GUCTL2): they keep what the boot path left, which on a normal
	 * boot is reset values (investigacion/03 E2). */
	dwc_wr(DCTL, DCTL_CSFTRST);
	int i;
	for (i = 0; i < 500 && (dwc_rd(DCTL) & DCTL_CSFTRST); i++) {
		delay(1000);
	}
	printf("SWEET USB: soft reset %s after %d ms, GCTL 0x%08x DCFG 0x%08x\n",
	    (dwc_rd(DCTL) & DCTL_CSFTRST) ? "STUCK" : "done", i, dwc_rd(GCTL), dwc_rd(DCFG));
	if (dwc_rd(DCTL) & DCTL_CSFTRST) {
		return;
	}

	/* 0328: clear GUSB2PHYCFG SUSPHY (bit 6) + ENBLSLPM (bit 8) once; Linux dwc3 clears them around
	 * every ep command (data path v2 does too, dp_cmd). On a normal boot SUSPHY is already 0 and
	 * ENBLSLPM is 1 (GUSB2PHYCFG 0x2500), but clearing them does not cure the ep0 SETEPCONFIG
	 * timeout there: that comes from the USB3 PIPE clock being off, which the UTMI-as-PIPE
	 * selection above fixes (investigacion/03 E1-E3). */
	dwc_wr(0xc200, dwc_rd(0xc200) & ~((1u << 6) | (1u << 8)));
	if (ud_dp) {
		ud_dp_core_init();
	}

	dwc_wr(GEVNTADRLO, (uint32_t)(ud.dma_pa + UD_EVT));
	dwc_wr(GEVNTADRHI, (uint32_t)((ud.dma_pa + UD_EVT) >> 32));
	dwc_wr(GEVNTSIZ, (1u << 31) | UD_EVT_SIZE);     /* interrupt masked: polled */
	dwc_wr(GEVNTCOUNT, dwc_rd(GEVNTCOUNT) & 0xffff);
	ud.evt_pos = 0;

	/* High speed, address 0 */
	uint32_t dcfg = dwc_rd(DCFG) & ~(7u | (0x7fu << 3));
	dwc_wr(DCFG, dcfg | 0);
	/* Events: disconnect, USB reset, connect done */
	dwc_wr(DEVTEN, 0x7);

	/* Endpoint 0 OUT/IN: start config, control type, max packet 512 until connect done. */
	if (ud_epcmd(0, DEPCMD_DEPSTARTCFG, 0, 0, 0) ||
	    ud_epcmd(0, DEPCMD_SETEPCONFIG, (0u << 1) | (512u << 3), (1u << 8) | (1u << 10) | (0u << 25), 0) ||
	    ud_epcmd(0, DEPCMD_SETTRANSFRESOURCE, 1, 0, 0) ||
	    ud_epcmd(1, DEPCMD_SETEPCONFIG, (0u << 1) | (512u << 3), (1u << 8) | (1u << 10) | (1u << 25), 0) ||
	    ud_epcmd(1, DEPCMD_SETTRANSFRESOURCE, 1, 0, 0)) {
		printf("SWEET USB: endpoint 0 setup failed\n");
		return;
	}
	dwc_wr(DALEPENA, 0x3);
	ud_start_setup();

	vm_offset_t qs = ml_io_map(QSCRATCH_PA, 0x100);
	if (qs != 0) {
		uint32_t hs = usb_rd(qs, QSCRATCH_HS_PHY_CTRL), ss = usb_rd(qs, QSCRATCH_SS_PHY_CTRL);
		printf("SWEET USB: QSCRATCH HS_PHY_CTRL 0x%08x SS_PHY_CTRL 0x%08x GENERAL_CFG 0x%08x\n",
		    hs, ss, usb_rd(qs, 0x08));
		*(volatile uint32_t *)(qs + QSCRATCH_HS_PHY_CTRL) = hs | UTMI_OTG_VBUS_VALID | SW_SESSVLD_SEL;
		*(volatile uint32_t *)(qs + QSCRATCH_SS_PHY_CTRL) = ss | LANE0_PWR_PRESENT;
		__asm__ volatile ("dsb sy" ::: "memory");
		printf("SWEET USB: VBUS override on: HS_PHY_CTRL 0x%08x SS_PHY_CTRL 0x%08x\n",
		    usb_rd(qs, QSCRATCH_HS_PHY_CTRL), usb_rd(qs, QSCRATCH_SS_PHY_CTRL));
	}

	dwc_wr(DCTL, dwc_rd(DCTL) | DCTL_RUN_STOP);
	for (i = 0; i < 500 && (dwc_rd(DSTS) & DSTS_DEVCTRLHLT); i++) {
		delay(1000);
	}
	printf("SWEET USB: running (DSTS 0x%08x after %d ms), DMA at 0x%llx\n", dwc_rd(DSTS), i, ud.dma_pa);
	if (kernel_thread_start(ud_dp ? ud_dp_thread : ud_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
	int livelog = 1;
	PE_parse_boot_argn("sweetusblog", &livelog, sizeof(livelog));
	if (livelog && ud.ifp != NULL && kernel_thread_start(ud_log_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}
#endif /* ARM64_BOARD_CONFIG_SWEET */
