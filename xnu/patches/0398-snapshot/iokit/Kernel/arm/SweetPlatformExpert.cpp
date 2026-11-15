/*
 * Minimal device-tree platform expert for Redmi Note 10 Pro (sweet).
 * Uses XNU's normal platform, power-domain and nub initialization. Hardware
 * interrupt and power-management services are supplied by separate drivers.
 */
#include <pexpert/arm64/board_config.h>

#if defined(SWEET)
#include <IOKit/IOPlatformExpert.h>
#include <IOKit/IODeviceTreeSupport.h>
#include <IOKit/IOLib.h>

class SweetPlatformExpert : public IODTPlatformExpert {
	OSDeclareDefaultStructors(SweetPlatformExpert);

public:
	bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	void processTopLevel(IORegistryEntry *root) APPLE_KEXT_OVERRIDE;
	const char *deleteList(void) APPLE_KEXT_OVERRIDE;
	const char *excludeList(void) APPLE_KEXT_OVERRIDE;
	int haltRestart(unsigned int type) APPLE_KEXT_OVERRIDE;
};

/*
 * sapphire 0221: restart and power off. Nothing set PE_halt_restart, so both did nothing and
 * XNU panicked after unmounting. Restart writes the reboot reason ABL reads (PMIC PON spare)
 * and asks PSCI for SYSTEM_RESET; halt is PSCI SYSTEM_OFF. The default is the bootloader
 * (fastboot). kern.sweet_reboot_mode (or boot-arg sweet_reboot): 0 system = a normal boot of the
 * GPT's active slot (iOS when iOS's slot is active, as installed), 1 recovery, 2 bootloader.
 * See bsd/dev/sweet_reboot.c.
 */
extern "C" int sweet_spmi_set_reboot_reason(unsigned mode);
extern "C" int sweet_reboot_mode_get(void);

static void
sweet_psci_call(uint64_t fn)
{
	register uint64_t x0 __asm__("x0") = fn;
	__asm__ volatile("smc #0" : "+r"(x0) : : "x1", "x2", "x3", "x4", "x5", "x6", "x7", "x8", "x9", "x10",
	    "x11", "x12", "x13", "x14", "x15", "x16", "x17", "cc", "memory");
}

int
SweetPlatformExpert::haltRestart(unsigned int type)
{
	if (type == kPERestartCPU) {
		int mode = sweet_reboot_mode_get();
		int rc = sweet_spmi_set_reboot_reason((unsigned)mode);
		IOLog("SWEET: restart, mode %d (%s), reason write %d\n", mode,
		    mode == 2 ? "bootloader" : mode == 1 ? "recovery" : "system", rc);
		IODelay(20000);
		sweet_psci_call(0x84000009ULL);         /* PSCI SYSTEM_RESET */
	} else if (type == kPEHaltCPU || type == kPEUPSDelayHaltCPU) {
		IOLog("SWEET: power off\n");
		IODelay(20000);
		sweet_psci_call(0x84000008ULL);         /* PSCI SYSTEM_OFF */
	}
	return IODTPlatformExpert::haltRestart(type);
}

OSDefineMetaClassAndStructors(SweetPlatformExpert, IODTPlatformExpert);

void SweetIONetworkStackStart(void);   /* SweetIONetworkStack.cpp (0323) */

bool
SweetPlatformExpert::start(IOService *provider)
{
	IOLog("SWEET: starting device-tree platform expert\n");
	if (!IODTPlatformExpert::start(provider)) {
		return false;
	}
	registerService();
	IOLog("SWEET: platform expert registered\n");
	/* 0396: with sweet_panic_reboot=1 a panic restarts the phone; the restart it gets then is to the bootloader */
	{
		int panic_reboot = 0;
		if (PE_parse_boot_argn("sweet_panic_reboot", &panic_reboot, sizeof(panic_reboot)) && panic_reboot != 0) {
			int rc = sweet_spmi_set_reboot_reason(2);
			IOLog("SWEET: sweet_panic_reboot: a panic restarts into the bootloader (reason write %d)\n", rc);
		}
	}
	/* 0323: IOResources/IONetworkStack, which configd's interface namer opens before it says "quiet" */
	SweetIONetworkStackStart();
	return true;
}

const char *
SweetPlatformExpert::deleteList(void)
{
	return "()";
}

const char *
SweetPlatformExpert::excludeList(void)
{
	return "('chosen', 'defaults', 'cpus', 'options')";
}

void
SweetPlatformExpert::processTopLevel(IORegistryEntry *root)
{
	/*
	 * The stock implementation requires an Apple NVRAM /options node.
	 * SWEET has no NVRAM controller; publish real CPU/device nodes without
	 * inventing one. createNubs adds the topology's logical CPU/cluster IDs.
	 */
	IORegistryEntry *cpus = root->childFromPath("cpus", gIODTPlane);
	if (cpus != NULL) {
		createNubs(this, IODTFindMatchingEntries(cpus, kIODTExclusive, NULL));
		cpus->release();
	}
	createNubs(this, IODTFindMatchingEntries(root, kIODTExclusive, excludeList()));

	/*
	 * No AppleARMIO driver publishes the /arm-io children: do it here so
	 * built-in drivers (the display, AppleCLCD) can match them. The GIC is
	 * driven by the platform code and gets no nub. The nubs hang off this
	 * platform expert: IOPlatformDevice::compareName asks its provider.
	 */
	IORegistryEntry *armio = root->childFromPath("arm-io", gIODTPlane);
	if (armio != NULL) {
		createNubs(this, IODTFindMatchingEntries(armio, kIODTExclusive, "('gic')"));
		armio->release();
	}
}

/*
 * hw.target / hw.product come from IODTPlatformExpert: the first root
 * "compatible" string and "model" (D22AP / iPhone10,3 in the ADT), which
 * is what the iPhone userland (MobileGestalt) expects.
 */
#endif /* SWEET */
