/*
 * sapphire 0221: where a restart goes (SweetPlatformExpert::haltRestart). sysctl
 * kern.sweet_reboot_mode = 0 system: a normal boot of the GPT's active slot (boot_<slot>), which is
 * iOS itself when iOS's slot is active (the installed layout flashes iOS into boot_<iOS slot>) and
 * Android only after kern.sweet_boot_android switched slots; 1 recovery; 2 bootloader (fastboot),
 * the default. Boot-arg sweet_reboot=N sets the default.
 */
#if defined(ARM64_BOARD_CONFIG_SWEET)
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/sysctl.h>
#include <pexpert/pexpert.h>

static int sweet_reboot_mode = -1;
int sweet_reboot_mode_get(void);
void sweet_reboot_mode_set(int mode);

/* 0299: Volume up held during the boot screen (sweet_keys.c): the next restart goes to recovery */
void
sweet_reboot_mode_set(int mode)
{
	if (mode >= 0 && mode <= 2) {
		sweet_reboot_mode = mode;
	}
}

int
sweet_reboot_mode_get(void)
{
	if (sweet_reboot_mode < 0) {
		int mode = 2;
		(void)PE_parse_boot_argn("sweet_reboot", &mode, sizeof(mode));
		sweet_reboot_mode = (mode >= 0 && mode <= 2) ? mode : 2;
	}
	return sweet_reboot_mode;
}

static int
sysctl_sweet_reboot_mode SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	int mode = sweet_reboot_mode_get();
	int error = sysctl_io_number(req, mode, sizeof(mode), &mode, NULL);
	if (error == 0 && req->newptr != USER_ADDR_NULL) {
		if (mode < 0 || mode > 2) {
			return EINVAL;
		}
		sweet_reboot_mode = mode;
		printf("SWEET REBOOT: next restart goes to %s\n",
		    mode == 2 ? "the bootloader" : mode == 1 ? "recovery" : "the system");
	}
	return error;
}

SYSCTL_PROC(_kern, OID_AUTO, sweet_reboot_mode, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED,
    0, 0, sysctl_sweet_reboot_mode, "I", "where a restart goes: 0 system, 1 recovery, 2 bootloader");
#endif /* ARM64_BOARD_CONFIG_SWEET */
