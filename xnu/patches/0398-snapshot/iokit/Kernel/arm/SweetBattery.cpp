/*
 * Built-in iOS battery bring-up service. Default: 100%, AC, not charging.
 * sweetbattery=1: read-only PM6150 polling every 30 seconds; voltage-derived
 * SOC is an UNCALIBRATED estimate, not a replacement for the Linux QG algo.
 * See report.md for binary evidence, register provenance and integration.
 */
#if defined(ARM64_BOARD_CONFIG_SWEET)
#include <device/device_types.h> /* io_buf_ptr_t for private OSKext headers */
#include <IOKit/IOLib.h>
#include <IOKit/IOWorkLoop.h>
#include <IOKit/IOTimerEventSource.h>
#include <IOKit/pwr_mgt/IOPMPowerSource.h>
#include <libkern/c++/OSCollectionIterator.h>
#include <pexpert/pexpert.h>
#include <kern/clock.h>
/* bsd/dev/sweet_spmi.c */
extern "C" int sweet_spmi_init(void);
extern "C" int sweet_spmi_read(unsigned sid, unsigned addr, uint8_t *out);
extern "C" int sweet_spmi_read_any(unsigned sid, unsigned addr, uint8_t *out);     /* any PMIC register, read only */
extern "C" int sweet_spmi_write(unsigned sid, unsigned addr, uint8_t val);
/* bsd/dev/sweet_i2c1.c: topaz battery bus (QUP0 SE1) */
extern "C" int sweet_i2c1_init(void);
extern "C" int sweet_cpufreq_max(void);    /* bsd/dev/sweet_cpufreq.c */
extern "C" int sweet_i2c1_read(uint8_t addr, uint8_t reg, uint8_t *buf, unsigned len);

class SweetBattery : public IOPMPowerSource {
	OSDeclareDefaultStructors(SweetBattery);
public:
	bool init(OSDictionary *dictionary = NULL) APPLE_KEXT_OVERRIDE;
	bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	void stop(IOService *provider) APPLE_KEXT_OVERRIDE;
	void free(void) APPLE_KEXT_OVERRIDE;
private:
	IOWorkLoop *fLoop;
	IOTimerEventSource *fTimer;
	bool fTimerAdded, fRunning, fReal;
	unsigned fSid;          /* the PMIC with the charger and the gauge: 0 PM6150 (sweet), 2 PM7250B (sapphire) */
	bool fChargerAuto;      /* 0242: keep the charger's input limit in HW auto mode (sweetchg_auto=0: off) */
	int fLastMode;
	void putNumber(const char *key, UInt32 value);
	void putBool(const char *key, bool value);
	void publish(unsigned percent, unsigned mv, int ma, bool external,
	    bool capable, bool charging, bool full, bool measured, bool currentValid);
	void poll(void);
	bool pollTopaz(void);
	static void timerFired(OSObject *owner, IOTimerEventSource *sender);
	static IOReturn stopPolling(OSObject *owner, void *, void *, void *, void *);
	static bool readWord(unsigned sid, unsigned address, uint16_t *value);
	static unsigned estimatePercent(unsigned mv);
};
OSDefineMetaClassAndStructors(SweetBattery, IOPMPowerSource);

bool
SweetBattery::init(OSDictionary *dictionary)
{
	/* IOCatalogue calls init(dictionary). This XNU's IOPMPowerSource only
	 * overrides init(void): explicitly run it to allocate its PS dictionary
	 * and symbols, then copy the matching personality into the registry. */
	if (!IOPMPowerSource::init()) {
		return false;
	}
	fLoop = NULL;
	fTimer = NULL;
	fTimerAdded = fRunning = fReal = false;
	if (dictionary) {
		OSCollectionIterator *it = OSCollectionIterator::withCollection(dictionary);
		if (!it) {
			return false;
		}
		OSObject *object;
		bool ok = true;
		while ((object = it->getNextObject())) {
			const OSSymbol *key = OSDynamicCast(OSSymbol, object);
			if (!key || !setProperty(key, dictionary->getObject(key))) {
				ok = false;
				break;
			}
		}
		it->release();
		if (!ok) {
			return false;
		}
	}
	fLastMode = -1;
	return true;
}

void
SweetBattery::putNumber(const char *key, UInt32 value)
{
	const OSSymbol *symbol = OSSymbol::withCString(key);
	OSNumber *number = OSNumber::withNumber(value, 32);
	if (symbol && number) {
		setPSProperty(symbol, number);
	}
	OSSafeReleaseNULL(number);
	OSSafeReleaseNULL(symbol);
}

void
SweetBattery::putBool(const char *key, bool value)
{
	const OSSymbol *symbol = OSSymbol::withCString(key);
	if (symbol) {
		setPSProperty(symbol, value ? kOSBooleanTrue : kOSBooleanFalse);
		symbol->release();
	}
}

void
SweetBattery::publish(unsigned percent, unsigned mv, int ma, bool external,
    bool capable, bool charging, bool full, bool measured, bool currentValid)
{
	/* Use consistent mAh units (DT nominal=5020). powerd computes C*100/M. */
	const unsigned capacity = (5020u * percent + 50u) / 100u;
	setBatteryInstalled(true);
	setCurrentCapacity(capacity);
	setMaxCapacity(5020);
	setExternalConnected(external);
	setExternalChargeCapable(capable);
	setIsCharging(charging);
	setAtWarnLevel(false);
	/* An uncalibrated voltage curve must not assert a critical shutdown. */
	setAtCriticalLevel(false);
	setVoltage(mv);
	setAmperage(ma);
	setTimeRemaining(-1); /* Unknown; never publish zero minutes on error. */
	setCycleCount(0);     /* Placeholder: cycle learning is not implemented. */
	putBool("FullyCharged", full);
	putBool("AppleRawExternalConnected", external);
	putNumber("AppleRawCurrentCapacity", capacity);
	putNumber("AppleRawMaxCapacity", 5020);
	putNumber("DesignCapacity", 5020);
	putNumber("InstantAmperage", (UInt32)ma);
	/* iOS battery convention: centidegrees C; fixed 25 C, NOT measured. */
	putNumber("Temperature", 2500);
	putBool("SweetBatteryHardwareSampleValid", measured);
	putBool("SweetBatteryCurrentValid", currentValid);
	putBool("SweetBatterySOCEstimated", measured);
	putBool("SweetBatteryTemperatureEstimated", true);
	putBool("SweetBatteryFallback", !measured);
	putNumber("SweetBatteryPercent", percent);

	/* IOPMCopyBatteryInfo's per-source compatibility dictionary. */
	OSDictionary *legacy = OSDictionary::withCapacity(7);
	if (legacy) {
		const char *keys[] = {kIOBatteryCurrentChargeKey, kIOBatteryCapacityKey,
		    kIOBatteryVoltageKey, kIOBatteryAmperageKey, kIOBatteryCycleCountKey,
		    kIOBatteryFlagsKey};
		UInt32 values[] = {capacity, 5020, mv, (UInt32)ma, 0,
		    (UInt32)(kIOBatteryInstalled | (external ? kIOBatteryChargerConnect : 0) |
		    (charging ? kIOBatteryCharge : 0))};
		for (unsigned i = 0; i < sizeof(values) / sizeof(values[0]); ++i) {
			OSNumber *number = OSNumber::withNumber(values[i], 32);
			if (number) {
				legacy->setObject(keys[i], number);
				number->release();
			}
		}
		setLegacyIOBatteryInfo(legacy);
		legacy->release();
	}
	updateStatus(); /* Flush PS properties and send 0xe0024100 on change. */
	int mode = measured ? 1 : 0;
	if (mode != fLastMode) {
		IOLog("SWEET BATTERY: %s, %u%% %umV external=%d charging=%d\n",
		    measured ? "PM6150 / estimated SOC" : "static fallback",
		    percent, mv, external, charging);
		fLastMode = mode;
	}
}

/* Keep the proven single-byte observer command. Read two matching whole
 * words to reject most torn ADC samples without writing the QG HOLD bit.
 * This is a bounded consistency check, not a hardware snapshot guarantee. */
bool
SweetBattery::readWord(unsigned sid, unsigned address, uint16_t *value)
{
	uint16_t previous = 0;
	for (unsigned attempt = 0; attempt < 4; ++attempt) {
		uint8_t lo, hi;
		if (sweet_spmi_read(sid, address, &lo) ||
		    sweet_spmi_read(sid, address + 1, &hi)) {
			return false;
		}
		uint16_t next = (uint16_t)((unsigned)lo | ((unsigned)hi << 8));
		if (attempt && next == previous) {
			*value = next;
			return true;
		}
		previous = next;
	}
	return false;
}

unsigned
SweetBattery::estimatePercent(unsigned mv)
{
	/*
	 * 0325: this cell's own open-circuit curve, 25 C column of qcom,pc-temp-v1-lut (0.1 mV units) in
	 * sapphire's dtbo, qcom,qbg-battery-profile-guanyu-5000mAh (the nvt-5000mAh profile is within 20 mV).
	 * At 3.93 V under +0.8 A (open circuit ~3.81 V) the generic 3.4-4.35 V curve gave 42 %, this one 29 %.
	 * Still an estimate: load (taken out with 0.15 ohm by the caller), temperature and aging.
	 */
	static const unsigned volts[] = {3000, 3325, 3474, 3563, 3625, 3671, 3696, 3704, 3709, 3720,
	    3736, 3753, 3766, 3778, 3789, 3797, 3806, 3816, 3829, 3844, 3860, 3879, 3892, 3909, 3932,
	    3962, 3985, 4004, 4041, 4083, 4129, 4178, 4230, 4282, 4331, 4377, 4422, 4452, 4495};
	static const unsigned soc[] = {0, 1, 2, 3, 4, 5, 6, 8, 10, 12,
	    14, 16, 18, 20, 22, 24, 28, 32, 36, 40, 44, 48, 50, 52, 54,
	    56, 58, 60, 64, 68, 72, 76, 80, 84, 88, 92, 96, 98, 100};
	static_assert(sizeof(volts) == sizeof(soc), "OCV table");
	if (mv <= volts[0]) {
		return soc[0];
	}
	for (unsigned i = 1; i < sizeof(volts) / sizeof(volts[0]); ++i) {
		if (mv < volts[i]) {
			return soc[i - 1] + (mv - volts[i - 1]) * (soc[i] - soc[i - 1]) /
			       (volts[i] - volts[i - 1]);
		}
	}
	return 100;
}

/*
 * 0325: Android's QG keeps its state in the PM7250B's SDAM (sid 2, 0xb100: qcom,qg-sdam@b100 under
 * qpnp,qg / "qcom,pm7250b-qg" in sapphire's dtbo) and rewrites it at every SoC update. Offsets from
 * Linux qg-reg.h / qg-sdam.c (msm-4.19 drivers/power/supply/qcom): VALID 0x46 (1), SOC 0x47 (1, the
 * shown %), TEMP 0x48 (2, 0.1 C), OCV_UV 0x4c (4), TIME_SEC 0x54 (4, PMIC RTC seconds), MAGIC 0x80 (4,
 * 0x12345678 once qpnp-qg has initialised it). Little endian. Read only, through the observer.
 */
static bool
qgSdam(unsigned offset, unsigned length, uint32_t *value)
{
	uint32_t v = 0;
	for (unsigned i = 0; i < length && i < 4; i++) {
		uint8_t b = 0;
		if (sweet_spmi_read_any(2, 0xb100u + offset + i, &b) != 0) {
			return false;
		}
		v |= (uint32_t)b << (8 * i);
	}
	*value = v;
	return true;
}

/*
 * 0325: PM6125's RTC (sid 0, "qcom,pm8941-rtc" at 0x6000; Linux rtc-pm8xxx pm8941 layout: RDATA0-3 at
 * 0x6048, CTRL 0x6046 bit 7 enable), the clock QG stamps TIME_SEC with. Raw seconds, as Linux 4.19
 * reads them (no offset). Byte 0 read again: if it moved during the read, read again.
 */
static bool
pmicRtc(uint32_t *seconds)
{
	uint8_t ctrl = 0;
	if (sweet_spmi_read_any(0, 0x6046, &ctrl) != 0 || !(ctrl & 0x80u)) {
		return false;
	}
	for (unsigned attempt = 0; attempt < 3; attempt++) {
		uint32_t v = 0;
		bool ok = true;
		for (unsigned i = 0; i < 4 && ok; i++) {
			uint8_t b = 0;
			ok = sweet_spmi_read_any(0, 0x6048u + i, &b) == 0;
			v |= (uint32_t)b << (8 * i);
		}
		uint8_t again = 0;
		if (ok && sweet_spmi_read_any(0, 0x6048, &again) == 0 && again == (uint8_t)v) {
			*seconds = v;
			return true;
		}
	}
	return false;
}

/*
 * topaz (MakrSas/topaz-woa TopazBattery gauge.c): SM5602 fuel gauge at 0x71
 * (16-bit little-endian words) and the bq25890-map charger at 0x6a, REG0B.
 * Read-only: nothing is written to either chip; charging runs in hardware.
 */
bool
SweetBattery::pollTopaz(void)
{
	uint8_t w[2], st = 0;
	uint16_t soc, volt, cur, full;
	if (sweet_i2c1_read(0x71, 0x05, w, 2)) {
		return false;
	}
	soc = (uint16_t)(w[0] | w[1] << 8);
	if (sweet_i2c1_read(0x71, 0x07, w, 2)) {
		return false;
	}
	volt = (uint16_t)(w[0] | w[1] << 8);
	if (sweet_i2c1_read(0x71, 0x08, w, 2)) {
		return false;
	}
	cur = (uint16_t)(w[0] | w[1] << 8);
	bool fullOk = sweet_i2c1_read(0x71, 0x62, w, 2) == 0;
	full = fullOk ? (uint16_t)(w[0] | w[1] << 8) : 0;
	bool chgOk = sweet_i2c1_read(0x6a, 0x0b, &st, 1) == 0;

	unsigned percent = (soc & 0x8000u) ? 0 : ((soc >> 8) & 0x7fu);
	unsigned mv = 1800u * (volt & 0x7fffu) / 19622u + 2700u;
	int ma = (int)((cur & 0x7fffu) * 1000u / 4088u * 2u);
	if (cur & 0x8000u) {
		ma = -ma;               /* bit 15: discharging */
	}
	unsigned fullMah = (full & 0x7fffu) * 1000u / 2048u;
	if (percent > 100 || mv < 2800 || mv > 4600) {
		return false;
	}
	unsigned vbus = (st >> 5) & 7u, chrg = (st >> 3) & 3u;
	bool external = chgOk && vbus >= 1 && vbus <= 6 && (st & 4u);
	bool charging = external && (chrg == 1 || chrg == 2);
	bool done = external && chrg == 3;
	publish(done ? 100 : percent, mv, ma, external, external, charging, done, true, true);
	static unsigned logs;
	if (logs++ % 20 == 0) {
		IOLog("SWEET BATTERY: topaz %u%% %umV %dmA full=%umAh chg=%#x external=%d charging=%d\n",
		    percent, mv, ma, fullMah, st, external, charging);
	}
	return true;
}

void
SweetBattery::poll(void)
{
	/*
	 * sapphire: topaz's gauge (I2C 0x71) is not there; the charger and the gauge are the PM7250B's
	 * (SPMI sid 2: SMB5 at 0x1000-0x13ff, QG at 0x4800), laid out as the PM6150's below. Before,
	 * the failed topaz read published 100%, external power, fully charged, forever.
	 */
	if (fReal && fSid == 0 && pollTopaz()) {
		return;
	}
	const unsigned sid = fSid;
	static bool dumped;
	if (!dumped && sweet_spmi_init() == 0) {
		/* 0208: once, what the QG and the charger hold, for the register choices below */
		dumped = true;
		static const uint16_t bases[] = { 0x4800, 0x4880, 0x1000, 0x1080, 0x1300 };
		for (unsigned b = 0; b < sizeof(bases) / sizeof(bases[0]); b++) {
			char line[400];
			int n = snprintf(line, sizeof(line), "SWEET BATTERY: sid %u %04x:", sid, bases[b]);
			for (unsigned i = 0; i < 0x80 && n < (int)sizeof(line) - 4; i += 1) {
				uint8_t v = 0;
				(void)sweet_spmi_read(sid, bases[b] + i, &v);
				n += snprintf(line + n, sizeof(line) - n, "%s%02x", (i % 16) ? "" : " ", v);
			}
			IOLog("%s\n", line);
		}
	}
	if (sid == 2 && fChargerAuto) {
		/*
		 * 0242: the PM7250B's charger came up in USB 5/1 mode (USBIN_ICL_OPTIONS 0x1366 = 0x02: 500 mA
		 * from any source) and nothing here changed it, so on the 33 W charger the phone took 500 mA,
		 * less than iOS spends, and ran down overnight while showing "charging". Linux's smb5 leaves
		 * HW auto mode (USBIN_MODE_CHG, bit 0; 0x1366 = 0x03): the input limit follows the charger the
		 * hardware detects (a PC's port 500 mA, a wall charger USBIN_CURRENT_LIMIT_CFG 0x1370, 1.5 A,
		 * with AICL backing off if its voltage sags). Checked every poll: a reset of the charger
		 * (hard reset, unplug) puts the default back. sweetchg_auto=0: left alone.
		 */
		uint8_t icl = 0;
		if (!sweet_spmi_read(sid, 0x1366, &icl) && !(icl & 1u)) {
			int r = sweet_spmi_write(sid, 0x1366, (uint8_t)(icl | 1u));
			uint8_t now = 0;
			(void)sweet_spmi_read(sid, 0x1366, &now);
			IOLog("SWEET BATTERY: charger input limit from the hardware: 0x1366 %#x -> %#x (%d)\n", icl, now, r);
			if (r != 0) {
				fChargerAuto = false;   /* not ours to write: do not retry every 2 s */
			}
		}
	}
	uint8_t type = 0, present = 0, usb = 0, path = 0, charger = 0, error = 0, temperature = 0, msoc = 0;
	bool regs =
	    !sweet_spmi_read(sid, 0x4804, &type) && type == 0x0d &&
	    !sweet_spmi_read(sid, 0x4808, &present) &&
	    !sweet_spmi_read(sid, 0x1310, &usb) &&
	    !sweet_spmi_read(sid, 0x110b, &path) &&
	    !sweet_spmi_read(sid, 0x1006, &charger) &&
	    !sweet_spmi_read(sid, 0x1007, &error) &&
	    !sweet_spmi_read(sid, 0x100d, &temperature);
	/*
	 * 0208: LAST_ADC_V (0xc0) stays 0 until Linux's QG driver sets the gauge going. Then, in this
	 * order: the S2 average (0x80), the newest FIFO sample (0x90), the OCV taken at power on (0x70).
	 */
	static const uint16_t vregs[] = { 0x48c0, 0x4880, 0x4890, 0x4870 }, iregs[] = { 0x48c2, 0x4882, 0x48a0, 0x4872 };
	uint16_t rawVoltage = 0, rawCurrent = 0;
	unsigned which = 0, mv = 0;
	for (; regs && which < 4; which++) {
		if (readWord(sid, vregs[which], &rawVoltage) && rawVoltage != 0 && rawVoltage != 0x8000) {
			mv = (unsigned)(((uint64_t)rawVoltage * 194637u) / 1000000u);
			if (mv >= 2800 && mv <= 4600) {
				break;
			}
		}
		mv = 0;
	}
	bool haveV = mv != 0;
	(void)sweet_spmi_read(sid, 0x48bf, &msoc);     /* QG_SOC_MONOTONIC: what the boot's charging code left */
	bool external = regs && (usb & 0x10u) != 0;
	bool capable = external && (path & 0x10u) && (path & 1u) && !(path & 0x40u);
	unsigned state = charger & 7u;
	bool charging = capable && state >= 1 && state <= 4 &&
	    !(error & 2u) && !(temperature & 0x0cu);
	/* Inhibit (0) does not by itself establish full capacity. */
	bool full = external && state == 5 && !(error & 2u);
	int ma = 0;
	uint8_t subtype = 0;
	bool currentValid = haveV && !sweet_spmi_read(sid, 0x4805, &subtype) &&
	    (subtype == 3 || subtype == 4) && readWord(sid, iregs[which < 4 ? which : 0], &rawCurrent) &&
	    rawCurrent != 0x8000;
	if (currentValid) {
		int signedRaw = (rawCurrent & 0x8000u) ? (int)rawCurrent - 65536 : (int)rawCurrent;
		/* Linux: negative=charging; IOPMPowerSource: positive=charging, mA. */
		ma = (int)(-((int64_t)signedRaw * (subtype == 3 ? 152588 : 305176)) / 1000000);
	}
	/*
	 * 0213: what is shown moves slowly and one way. On the phone the percent jumped every 2 s (18,
	 * 16, 19, 17) and now and then to 100: QG_SOC_MONOTONIC is not kept up here (84, 0, other values
	 * from one boot to the next) and is no longer used, and the voltage moves with the load. So:
	 * the voltage with the current's drop taken out (an open-circuit estimate, 0.15 ohm), averaged
	 * (1/8 of each new reading), its percent shown at most 1 point every 30 s, never down while
	 * charging nor up while discharging; "full" only after 3 polls in a row.
	 */
	/*
	 * 0325 (tester bug 9): the voltage alone read 20 points low on this 4.45 V cell. Now:
	 * - start: Android's own percent, from the QG's SDAM (qgSdam), if it is valid, stamped by the
	 *   PMIC RTC no more than sweetbat_sdam_age (600) s ago (Linux's own rule, 360 s by default: the
	 *   phone was just restarted from Android). Otherwise (iOS after iOS, a cold boot) the cell's real
	 *   open-circuit curve (estimatePercent) at the voltage with the current's drop taken out.
	 * - then: the charge counted, the measured current integrated over 5020 mAh every poll.
	 * - the curve still pulls, slowly (~30 min), when the load is light (|I| < 150 mA: the drop is
	 *   small), and at once when there is no live current (the gauge's samples are not moving: no
	 *   Linux ran the QG this power-up): the 0213 behaviour.
	 * - shown: never up while discharging nor down while charging, 99 until the charger says full.
	 * ioreg: SweetBatterySOCSource 1 SDAM, 2 voltage; SweetBatteryChargeUAh; SweetBatteryOCVPercent.
	 */
	static const int64_t kCapUAh = 5020ll * 1000ll;
	static bool seeded;
	static int64_t chargeUAh;               /* charge left, uAh, 0..kCapUAh */
	static uint64_t lastAbs;
	static unsigned shown, fullPolls, source, stillPolls, gaps;
	static uint16_t lastRawV, lastRawI;
	unsigned percent = shown != 0 ? shown : 50;
	unsigned ocvPct = 0;
	int ocvMv = 0;
	fullPolls = (external && state == 5 && !(error & 2u)) ? fullPolls + 1 : 0;
	full = fullPolls >= 3;
	if (haveV) {
		ocvMv = (int)mv - (int)((int64_t)ma * 15 / 100);       /* ma > 0 charging: the voltage stands above the OCV */
		if (ocvMv < 3000) {
			ocvMv = 3000;
		}
		ocvPct = estimatePercent((unsigned)ocvMv);
	}
	if (!seeded && regs && haveV) {
		seeded = true;
		uint32_t valid = 0, soc = 0, when = 0, magic = 0, temp = 0, now = 0;
		bool sdam = qgSdam(0x46, 1, &valid) && qgSdam(0x47, 1, &soc) && qgSdam(0x48, 2, &temp) &&
		    qgSdam(0x54, 4, &when) && qgSdam(0x80, 4, &magic);
		bool rtc = pmicRtc(&now);
		long long age = (sdam && rtc) ? (long long)now - (long long)when : -1;
		unsigned maxAge = 600;
		(void)PE_parse_boot_argn("sweetbat_sdam_age", &maxAge, sizeof(maxAge));
		bool useSdam = sdam && valid == 1 && magic == 0x12345678u && soc <= 100 && age >= 0 &&
		    age <= (long long)maxAge;
		unsigned start = useSdam ? soc : ocvPct;
		source = useSdam ? 1 : 2;
		chargeUAh = kCapUAh * start / 100;
		lastAbs = mach_absolute_time();
		IOLog("SWEET BATTERY: start at %u%% from %s; SDAM %s valid %u soc %u temp %d (0.1 C) magic %#x, age %lld s "
		    "(max %u, rtc %s %u); now %u mV %d mA, open circuit ~%d mV = %u%%\n", start,
		    useSdam ? "Android's QG (SDAM)" : "the voltage", sdam ? "read" : "unreadable", valid, soc,
		    (int)(int16_t)temp, magic, age, maxAge, rtc ? "at" : "unreadable", now, mv, ma, ocvMv, ocvPct);
	}
	if (seeded) {
		uint64_t nowAbs = mach_absolute_time(), ns = 0;
		absolutetime_to_nanoseconds(nowAbs - lastAbs, &ns);
		lastAbs = nowAbs;
		uint64_t ms = ns / NSEC_PER_MSEC;
		/* the same raw V and I for a minute is a gauge that is not sampling, not a measurement */
		stillPolls = (rawVoltage == lastRawV && rawCurrent == lastRawI) ? stillPolls + 1 : 0;
		lastRawV = rawVoltage;
		lastRawI = rawCurrent;
		bool live = currentValid && which < 3 && stillPolls < 30;     /* 3: the power-on OCV sample, fixed */
		if (live && ms <= 30000) {
			chargeUAh += (int64_t)ma * (int64_t)ms / 3600;          /* mA * ms / 3600 = uAh */
		} else if (ms > 30000) {
			gaps++;
		}
		if (haveV && (!live || (ma > -150 && ma < 150))) {
			int64_t target = kCapUAh * ocvPct / 100;
			chargeUAh += (target - chargeUAh) / (live ? 900 : 30);
		}
		if (haveV && !external && mv < 3450 && chargeUAh > kCapUAh * 3 / 100) {
			chargeUAh -= (chargeUAh - kCapUAh * 3 / 100) / 10;      /* sagging near the 3.4 V cutoff: head for 3 % */
		}
		if (full) {
			chargeUAh = kCapUAh;
		}
		if (chargeUAh < 0) {
			chargeUAh = 0;
		} else if (chargeUAh > kCapUAh) {
			chargeUAh = kCapUAh;
		}
		unsigned target = (unsigned)((chargeUAh * 100 + kCapUAh / 2) / kCapUAh);
		if (target >= 100 && !full) {
			target = 99;
		}
		if (target == 0 && haveV && mv > 3500) {
			target = 1;
		}
		if (full) {
			shown = 100;
		} else if (shown == 0) {
			shown = target;
		} else if (target != shown) {
			bool up = target > shown;
			if (!live || (up && ma > 0) || (!up && ma < 0)) {
				shown += up ? 1 : -1;   /* at most a point a poll, and only the way the current goes */
			}
		}
		percent = shown;
		putNumber("SweetBatterySOCSource", source);
		putNumber("SweetBatteryChargeUAh", (UInt32)chargeUAh);
		putNumber("SweetBatteryOCVPercent", ocvPct);
	}
	if (!regs) {
		static unsigned bad;
		if (bad++ % 30 == 0) {
			IOLog("SWEET BATTERY: sid %u: the PMIC does not answer (type %#x)\n", sid, type);
		}
		publish(100, 4200, 0, true, true, false, true, false, false);
		return;
	}
	/* (no voltage: the charger's state is still real; the percent is the boot's, or a neutral 50) */
	publish(full ? 100 : percent, haveV ? mv : 3900, ma, external, capable, charging, full, haveV, currentValid);
	static unsigned logs;
	if (logs++ % 30 == 0) {
		IOLog("SWEET BATTERY: sid %u: %umV from %#x (~%u%%, monotonic soc %u) %dmA usb %#x path %#x chgr %#x err %#x present %#x "
		    "-> external=%d charging=%d full=%d\n", sid, mv, haveV ? vregs[which] : 0, percent, msoc, ma, usb, path, charger,
		    error, present, external, charging, full);
		/*
		 * 0325: the gauge, and what limits the charge (read only; Linux smb5-reg.h, 50 mA steps):
		 * APSD_RESULT 0x1308 (bit 0 SDP, 2 CDP, 3 DCP, 5 QC2, 6 QC3), USBIN_CURRENT_LIMIT_CFG 0x1370,
		 * ICL_STATUS 0x1107 (the limit in use after AICL), FAST_CHARGE_CURRENT_CFG 0x1061, and the
		 * charger's JEITA band from BATTERY_CHARGER_STATUS_7 0x100d (bits 5/4 warm/cool, 3/2 hot/cold).
		 */
		uint8_t apsd = 0, iclCfg = 0, iclNow = 0, fcc = 0;
		(void)sweet_spmi_read(sid, 0x1308, &apsd);
		(void)sweet_spmi_read(sid, 0x1370, &iclCfg);
		(void)sweet_spmi_read(sid, 0x1107, &iclNow);
		(void)sweet_spmi_read(sid, 0x1061, &fcc);
		IOLog("SWEET BATTERY: gauge %u%% (source %u, %lld uAh, ocv %d mV = %u%%, still %u, gaps %u); charger apsd %#x "
		    "icl cfg %u mA in use %u mA, fcc %u mA, jeita %#x\n", percent, source, chargeUAh, ocvMv, ocvPct,
		    stillPolls, gaps, apsd, iclCfg * 50u, iclNow * 50u, fcc * 50u, temperature);
	}
}

void
SweetBattery::timerFired(OSObject *owner, IOTimerEventSource *sender)
{
	SweetBattery *self = OSDynamicCast(SweetBattery, owner);
	if (self && self->fRunning) {
		self->poll();
		sender->setTimeoutMS(2000);     /* 0209: was 10 s: plugging in showed 4-5 s late */
	}
}

bool
SweetBattery::start(IOService *provider)
{
	if (!IOPMPowerSource::start(provider)) {
		return false;
	}
	(void)sweet_cpufreq_max();
	int real = 1;   /* topaz: real gauge by default; sweetbattery=0 for the static fallback */
	(void)PE_parse_boot_argn("sweetbattery", &real, sizeof(real));
	unsigned sid = 2;       /* sapphire: PM7250B; boot-arg sweetbattery_sid=0 for the PM6150 / topaz path */
	(void)PE_parse_boot_argn("sweetbattery_sid", &sid, sizeof(sid));
	fSid = sid;
	int chgAuto = 1;
	(void)PE_parse_boot_argn("sweetchg_auto", &chgAuto, sizeof(chgAuto));
	fChargerAuto = chgAuto != 0;
	fReal = real == 1 && (fSid == 0 ? sweet_i2c1_init() == 0 : sweet_spmi_init() == 0);
	/* Publish a complete source BEFORE registerService wakes powerd. */
	poll();
	if (fReal) {
		fLoop = IOWorkLoop::workLoop();
		fTimer = IOTimerEventSource::timerEventSource(this, &SweetBattery::timerFired);
		if (fLoop && fTimer && fLoop->addEventSource(fTimer) == kIOReturnSuccess) {
			fTimerAdded = true;
			fRunning = true;
			fTimer->setTimeoutMS(2000);
		} else {
			fReal = false;
			poll();
			IOLog("SWEET BATTERY: timer unavailable; static fallback\n");
		}
	}
	registerService();
	return true;
}

void
SweetBattery::stop(IOService *provider)
{
	if (fLoop && fTimerAdded) {
		/* Serialize against a running timer callback, including rearming. */
		(void)fLoop->runAction(&SweetBattery::stopPolling, this);
		fLoop->removeEventSource(fTimer);
		fTimerAdded = false;
	}
	IOPMPowerSource::stop(provider);
}

IOReturn
SweetBattery::stopPolling(OSObject *owner, void *, void *, void *, void *)
{
	SweetBattery *self = OSDynamicCast(SweetBattery, owner);
	if (!self) {
		return kIOReturnBadArgument;
	}
	self->fRunning = false;
	self->fTimer->cancelTimeout();
	self->fTimer->disable();
	return kIOReturnSuccess;
}

void
SweetBattery::free(void)
{
	OSSafeReleaseNULL(fTimer);
	OSSafeReleaseNULL(fLoop);
	IOPMPowerSource::free();
}
#endif /* ARM64_BOARD_CONFIG_SWEET */
