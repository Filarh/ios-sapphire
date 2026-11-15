/*
 * Minimal polled SWEET keys. Call once after the scheduler is running.
 * See hw-report.md: volume-up is PM6150L SID 4, NOT PM6150 SID 0.
 * PMIC access is READ ONLY: the sole MMIO write issues EXT_READL on an
 * observer channel. No PMIC config, IRQ acknowledge or reset writes exist.
 */
#if defined(ARM64_BOARD_CONFIG_SWEET)
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/sysctl.h>
#include <sys/errno.h>
#include <kern/clock.h>
#include <kern/thread.h>
#include <kern/sched_prim.h>
#include <machine/machine_routines.h>
#include <pexpert/pexpert.h>

extern vm_offset_t ml_io_map(vm_offset_t, vm_size_t);
extern void sweet_hid_key(int key, int down);
extern int sweet_splash_active;         /* osfmk/console/video_console.c (0299): the boot screen is up */
extern void sweet_reboot_mode_set(int mode);
extern int reboot_kernel(int howto, char *message);
void sweet_keys_init(void);

/* topaz (SM6225, sm6125.dtsi spmi_bus; PM6125 USID 0): PON 0x800, Vol+ = GPIO5 */
#define KEYS_CORE_PA     0x01c40000u
#define KEYS_OBS_PA      0x03e00000u
#define KEYS_CNFG_PA     0x01c0a000u
#define KEYS_PON_RT      0x0810u
#define KEYS_GPIO_RT     0xc410u
#define KEYS_NO_APID     0xffffu

static vm_offset_t keys_core, keys_obs, keys_cnfg;
static uint16_t keys_pon_apid = KEYS_NO_APID, keys_gpio_apid = KEYS_NO_APID;
static uint32_t keys_version, keys_last_status;
static bool keys_started;
static int keys_sleep_chan;

static uint32_t
keys_rd(vm_offset_t base, uint32_t off)
{
	uint32_t v = *(volatile uint32_t *)(base + off);
	__asm__ volatile("dmb oshld" ::: "memory");
	return v;
}

/* v5 can map one PPID more than once; prefer the apps-owned entry. */
static uint16_t
keys_find_apid(uint16_t ppid, uint32_t count)
{
	uint16_t found = KEYS_NO_APID;
	for (uint32_t i = 0; i < count; ++i) {
		uint32_t map = keys_rd(keys_core, 0x900u + 4u * i);
		if (map == 0 || ((map >> 8) & 0xfffu) != ppid) {
			continue;
		}
		uint32_t owner = keys_rd(keys_cnfg, 0x700u + 4u * i) & 7u;
		if (found == KEYS_NO_APID || owner == 0) {
			found = (uint16_t)i;
		}
	}
	return found;
}

void sweet_spmi_bus_lock(void);         /* sweet_spmi.c (0214): one reader of the observer at a time */
void sweet_spmi_bus_unlock(void);
static bool keys_read_locked(uint16_t apid, uint16_t addr, uint8_t *out);

static bool
keys_read(uint16_t apid, uint16_t addr, uint8_t *out)
{
	if (apid == KEYS_NO_APID) {
		return false;
	}
	sweet_spmi_bus_lock();
	bool ok = keys_read_locked(apid, addr, out);
	sweet_spmi_bus_unlock();
	return ok;
}

static bool
keys_read_locked(uint16_t apid, uint16_t addr, uint8_t *out)
{
	uint32_t off = 0x80u * apid; /* obs + EE(0)*0x10000 */
	uint32_t cmd = (1u << 27) | ((uint32_t)(addr & 0xffu) << 4);
	__asm__ volatile("dmb oshst" ::: "memory");
	*(volatile uint32_t *)(keys_obs + off) = cmd; /* EXT_READL, one byte */
	__asm__ volatile("dsb osh" ::: "memory");
	for (unsigned i = 0; i < 1000; ++i) {
		keys_last_status = keys_rd(keys_obs, off + 0x08u);
		if (keys_last_status & 1u) {
			if (keys_last_status & 0x0eu) {
				return false;
			}
			*out = (uint8_t)keys_rd(keys_obs, off + 0x18u);
			return true;
		}
		delay(1);
	}
	return false;
}

static void
keys_wait(void)
{
	uint64_t deadline;
	clock_interval_to_deadline(20, NSEC_PER_MSEC, &deadline);
	(void)assert_wait_deadline(&keys_sleep_chan, THREAD_UNINT, deadline);
	thread_block(THREAD_CONTINUE_NULL);
}

static void
keys_thread(void *arg, wait_result_t result)
{
	int state[3] = {0, 0, 0}, seen[3] = {0, 0, 0};
	unsigned errors = 0;
	(void)arg;
	(void)result;
	for (;;) {
		uint8_t pon = 0, gpio = 0;
		bool pok = keys_read(keys_pon_apid, KEYS_PON_RT, &pon);
		bool gok = keys_read(keys_gpio_apid, KEYS_GPIO_RT, &gpio);
		int now[3] = {!!(pon & 1u), !(gpio & 1u), !!(pon & 2u)};
		if (!pok || !gok) {
			if ((errors++ % 250u) == 0) {
				printf("SWEET KEYS: read error pon-ok=%d gpio-ok=%d arb-status=%#x\n",
				    pok, gok, keys_last_status);
			}
		} else {
			errors = 0;
		}
		for (int k = 0; k < 3; ++k) {
			/* 0214: a change counts once two polls in a row (40 ms) agree on it */
			bool ok = k == 1 ? gok : pok;
			bool agreed = ok && now[k] == seen[k];
			if (ok) {
				seen[k] = now[k];
			}
			if (agreed && now[k] != state[k]) {
				state[k] = now[k];
				printf("SWEET KEYS: key=%d down=%d pon=%#x gpio4:c110=%#x\n",
				    k, state[k], pon, gpio);
				sweet_hid_key(k, state[k]);
				/* 0299: Volume up during the boot screen restarts into recovery (as Power + Volume up) */
				if (k == 1 && state[k] && sweet_splash_active) {
					printf("SWEET KEYS: Volume up on the boot screen: restarting into recovery\n");
					sweet_reboot_mode_set(1);
					(void)reboot_kernel(0, (char *)"Volume up on the boot screen: recovery");
				}
			}
		}
		keys_wait();
	}
}

void
sweet_keys_init(void)
{
	int enabled = 1;
	thread_t thread;
	if (keys_started) {
		return;
	}
	keys_started = true;
	(void)PE_parse_boot_argn("sweetkeys", &enabled, sizeof(enabled));
	if (!enabled) {
		printf("SWEET KEYS: disabled (sweetkeys=0)\n");
		return;
	}
	keys_core = ml_io_map(KEYS_CORE_PA, 0x2000);
	keys_cnfg = ml_io_map(KEYS_CNFG_PA, 0x1000);
	keys_obs = ml_io_map(KEYS_OBS_PA, 0x10000);
	if (!keys_core || !keys_cnfg || !keys_obs) {
		printf("SWEET KEYS: MMIO mapping failed\n");
		return;
	}
	keys_version = keys_rd(keys_core, 0);
	uint32_t count = keys_rd(keys_core, 4) & 0x7ffu;
	if (keys_version < 0x50000000u || keys_version >= 0x70000000u ||
	    count == 0 || count > 512) {
		printf("SWEET KEYS: unsupported arbiter version=%#x apids=%u\n",
		    keys_version, count);
		return;
	}
	keys_pon_apid = keys_find_apid(0x008, count);
	keys_gpio_apid = keys_find_apid(0x0c4, count);
	uint8_t pon = 0, gpio = 0, mode = 0, en = 0, subtype = 0;
	bool pok = keys_read(keys_pon_apid, KEYS_PON_RT, &pon);
	bool gok = keys_read(keys_gpio_apid, KEYS_GPIO_RT, &gpio);
	bool mok = keys_read(keys_gpio_apid, 0xc440, &mode);
	bool eok = keys_read(keys_gpio_apid, 0xc446, &en);
	bool sok = keys_read(keys_gpio_apid, 0xc405, &subtype);
	printf("SWEET KEYS: arb=%#x apids=%u pon-apid=%u gpio4-apid=%u pon=%#x/%d gpio=%#x/%d mode=%#x/%d en=%#x/%d subtype=%#x/%d\n",
	    keys_version, count, keys_pon_apid, keys_gpio_apid,
	    pon, pok, gpio, gok, mode, mok, en, eok, subtype, sok);
	/* GPIO configuration belongs to firmware. Diagnose, never repair it here. */
	unsigned dir = subtype >= 0x10u && subtype <= 0x13u ?
	    mode & 3u : (mode >> 4) & 7u;
	if (!eok || !mok || !sok || !(en & 0x80u) || (dir != 0 && dir != 2)) {
		printf("SWEET KEYS: GPIO5 not confirmed enabled as input; volume-up withheld, PMIC unchanged\n");
		keys_gpio_apid = KEYS_NO_APID;
	}
	if (!pok && !gok) {
		printf("SWEET KEYS: no readable keys; polling retained for recovery\n");
	}
	kern_return_t kr = kernel_thread_start(keys_thread, NULL, &thread);
	if (kr == KERN_SUCCESS) {
		thread_deallocate(thread);
	} else {
		printf("SWEET KEYS: kernel_thread_start failed %d\n", kr);
	}
}
#endif /* ARM64_BOARD_CONFIG_SWEET */

/* 0319: which Metal plug-in the accelerator names (SweetFramebuffer.cpp sweet_metal_plugin): 1 SapphireMetal, 2 HakiosMetal */
extern int sweet_metal_plugin(int set);

static int
sysctl_sweet_metal SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	int v = sweet_metal_plugin(-1), changed = 0;
	int error = sysctl_io_number(req, v, sizeof(v), &v, &changed);
	if (error || !changed) {
		return error;
	}
	if (v < 0 || v > 2) {
		return EINVAL;
	}
	(void)sweet_metal_plugin(v);
	return 0;
}
SYSCTL_PROC(_kern, OID_AUTO, sweet_metal, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED,
    0, 0, sysctl_sweet_metal, "I", "sapphire: the Metal plug-in the accelerator names: 0 none, 1 SapphireMetal, 2 HakiosMetal");
