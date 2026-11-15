/*
 * SWEET internal UFS root device.
 *
 * The Qualcomm UFS host (UFSHCI 2.1) is left enabled with the link up by the
 * bootloader, which loaded our boot image from it. This driver reuses that
 * state: it points the transfer request list at its own memory (verifying the
 * host took it before ringing any doorbell) and issues polled SCSI commands,
 * one at a time, to LUN 0. It finds one GPT partition by name (boot-arg
 * ufsroot=, default "userdata") and exposes it as the BSD disk ufsroot0
 * (rufsroot0), with 512-byte logical sectors over the device's 4 KiB blocks.
 * Root on it with rd=ufsroot0. On the dual-boot layout the iOS root is the
 * "iosroot" partition and the boot-args must say ufsroot=iosroot: without it
 * this takes Android's userdata as the iOS root and, with ufsrw=1, writes the
 * console log over userdata's last 4 MiB + 4 KiB.
 *
 * All DMA goes through one physically contiguous allocation (descriptors and
 * a 1 MiB bounce buffer); the host is not cache coherent, so caches are
 * cleaned/invalidated around each command.
 *
 * Writes are refused unless boot-arg ufsrw=1 is set. Every write is checked
 * against the partition bounds again right before it is issued, so nothing
 * outside the named partition can be written.
 */
#if defined(ARM64_BOARD_CONFIG_SWEET)
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/buf.h>
#include <sys/conf.h>
#include <sys/disk.h>
#include <sys/errno.h>
#include <sys/proc.h>
#include <sys/uio.h>
#include <miscfs/devfs/devfs.h>
#include <kern/locks.h>
#include <kern/clock.h>
#include <kern/sched_prim.h>
#include <machine/machine_routines.h>
#include <arm/caches_internal.h>
#include <vm/pmap.h>
#include <vm/vm_kern.h>
#include <pexpert/pexpert.h>
#include <pexpert/device_tree.h>
#include <libkern/libkern.h>
#include <machine/atomic.h>
#include <sys/sysctl.h>
#include <sys/kauth.h>
#include "sweet_slot.h"

extern ppnum_t pmap_find_phys(pmap_t pmap, addr64_t va);
/* osfmk/arm/machine_routines.h, not exported to the BSD layer */
extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
/* osfmk/arm/cpu_data.h, not exported to the BSD layer */
extern int get_preemption_level(void);
extern kern_return_t kernel_thread_start(thread_continue_t, void *, thread_t *);
extern void thread_deallocate(thread_t);
extern int hz;
static void sweet_log_start(void);
extern void sweet_sample_start(void);
extern void sweet_keys_init(void);
extern void sweet_touch_init(void);
extern void sweet_bt_boot_init(void);   /* bsd/dev/sweet_bt.c: device nodes only */
extern void sweet_usb_probe(void);

#define REG_CAP         0x00
#define REG_IS          0x20
#define REG_IE          0x24
#define REG_HCS         0x30
#define REG_HCE         0x34
#define REG_UTRLBA      0x50
#define REG_UTRLBAU     0x54
#define REG_UTRLDBR     0x58
#define REG_UTRLCLR     0x5c
#define REG_UTRLRSR     0x60

#define HCS_DP          (1u << 0)
#define HCS_UTRLRDY     (1u << 1)
#define CAP_64AS        (1u << 24)              /* 64-bit DMA addressing */

#define UFS_BLOCK       4096u
#define BOUNCE_SIZE     (1u << 20)
#define PRDT_CHUNK      (256u << 10)            /* PRDT byte count field: 18 bits */
#define PRDT_MAX        (BOUNCE_SIZE / PRDT_CHUNK)
#define CMD_TIMEOUT_NS  (5ull * NSEC_PER_SEC)
#define CMD_SPIN_NS     (50ull * NSEC_PER_USEC)
#define CMD_SLEEP_NS    (200ull * NSEC_PER_USEC)

struct utrd {
	uint32_t dw0, dw1, dw2, dw3;
	uint32_t ucdba, ucdbau;
	uint32_t resp;          /* [31:16] response UPIU offset, [15:0] length, DWORDs */
	uint32_t prdt;          /* [31:16] PRDT offset (DWORDs), [15:0] entries */
};

struct ucd {
	uint8_t cmd[512];
	uint8_t resp[512];
	struct {
		uint32_t addr_lo, addr_hi, rsvd, dbc;
	} prdt[PRDT_MAX];
};

/* Layout of the contiguous DMA allocation. */
#define DMA_UTRL        0x0000                  /* 1 KiB aligned */
#define DMA_UCD         0x1000                  /* 128 byte aligned */
#define DMA_BOUNCE      0x4000
#define DMA_SIZE        (DMA_BOUNCE + BOUNCE_SIZE)

static struct {
	vm_offset_t regs;
	vm_offset_t dma;                        /* kernel VA of the DMA allocation */
	uint64_t dma_pa;
	uint8_t lun;
	uint8_t tag;
	uint64_t first_block, nblocks;          /* partition, in 4 KiB device blocks */
	uint32_t secsize;                       /* logical sector size for the BSD layer */
	bool writable;                          /* boot-arg ufsrw=1 */
	bool cache;                             /* boot-arg ufscache=1 */
	bool wait;                              /* boot-arg ufswait=1 */
	int bmajor, cmajor;
	dev_t bdev;
	bool ready;
} su;

static LCK_GRP_DECLARE(sweet_ufs_grp, "sweet_ufs");
static LCK_MTX_DECLARE(sweet_ufs_lock, &sweet_ufs_grp);

static inline uint32_t
ufs_rd(uint32_t off)
{
	return *(volatile uint32_t *)(su.regs + off);
}

static inline void
ufs_wr(uint32_t off, uint32_t v)
{
	*(volatile uint32_t *)(su.regs + off) = v;
}

static void
dma_sync(uint32_t off, uint32_t len)
{
	flush_dcache64((addr64_t)(su.dma_pa + off), len, TRUE);
}

static void
dma_clean(uint32_t off, uint32_t len)
{
	clean_dcache64((addr64_t)(su.dma_pa + off), len, TRUE);
}

/*
 * One SCSI command in slot 0, data (if any) to/from the bounce buffer
 * (to the device when write is set). Returns 0 or an errno.
 */
static int
ufs_scsi(const uint8_t *cdb, unsigned cdb_len, uint32_t len, bool write)
{
	struct utrd *utrd = (struct utrd *)(su.dma + DMA_UTRL);
	struct ucd *ucd = (struct ucd *)(su.dma + DMA_UCD);
	uint64_t ucd_pa = su.dma_pa + DMA_UCD, bounce_pa = su.dma_pa + DMA_BOUNCE;

	assert(len <= BOUNCE_SIZE);
	bzero(utrd, sizeof(*utrd));
	bzero(ucd, sizeof(*ucd));

	ucd->cmd[0] = 0x01;                     /* COMMAND UPIU */
	ucd->cmd[1] = len ? (write ? 0x20 : 0x40) : 0x00;      /* W: data out, R: data in */
	ucd->cmd[2] = su.lun;
	ucd->cmd[3] = ++su.tag;
	ucd->cmd[12] = (uint8_t)(len >> 24);
	ucd->cmd[13] = (uint8_t)(len >> 16);
	ucd->cmd[14] = (uint8_t)(len >> 8);
	ucd->cmd[15] = (uint8_t)len;
	memcpy(&ucd->cmd[16], cdb, cdb_len);

	/* DD: 1 = memory to device, 2 = device to memory. */
	utrd->dw0 = (1u << 28) | ((len ? (write ? 1u : 2u) : 0u) << 25) | (1u << 24);
	utrd->dw2 = 0x0f;                       /* OCS invalid until updated */
	utrd->ucdba = (uint32_t)ucd_pa;
	utrd->ucdbau = (uint32_t)(ucd_pa >> 32);
	utrd->resp = (512u / 4) << 16 | (512u / 4);
	if (len) {
		uint32_t n = 0;
		for (uint32_t done = 0; done < len; done += PRDT_CHUNK, n++) {
			uint32_t chunk = MIN(PRDT_CHUNK, len - done);
			ucd->prdt[n].addr_lo = (uint32_t)(bounce_pa + done);
			ucd->prdt[n].addr_hi = (uint32_t)((bounce_pa + done) >> 32);
			ucd->prdt[n].dbc = chunk - 1;
		}
		utrd->prdt = (1024u / 4) << 16 | n;
	}

	if (su.cache) {
		dma_clean(DMA_UTRL, sizeof(*utrd));
		dma_clean(DMA_UCD, 16 + cdb_len);
		dma_sync(DMA_UCD + offsetof(struct ucd, resp), sizeof(ucd->resp));
		if (len) {
			dma_clean(DMA_UCD + offsetof(struct ucd, prdt),
			    (utrd->prdt & 0xffff) * sizeof(ucd->prdt[0]));
		}
	} else {
		dma_sync(DMA_UTRL, sizeof(*utrd));
		dma_sync(DMA_UCD, sizeof(*ucd));
	}
	if (len) {
		if (su.cache && write) {
			dma_clean(DMA_BOUNCE, len);
		} else {
			dma_sync(DMA_BOUNCE, len);
		}
	}
	__builtin_arm_dsb(0xf);

	ufs_wr(REG_UTRLDBR, 1);
	uint64_t deadline;
	nanoseconds_to_absolutetime(CMD_TIMEOUT_NS, &deadline);
	deadline += mach_absolute_time();
	bool sleepable = su.wait && su.ready && ml_get_interrupts_enabled() &&
	    !ml_at_interrupt_context() && get_preemption_level() == 0;
	uint64_t spin_until = 0, sleep_interval = 0;
	if (sleepable) {
		nanoseconds_to_absolutetime(CMD_SPIN_NS, &spin_until);
		spin_until += mach_absolute_time();
		nanoseconds_to_absolutetime(CMD_SLEEP_NS, &sleep_interval);
	}
	while (ufs_rd(REG_UTRLDBR) & 1) {
		uint64_t now = mach_absolute_time();
		if (now > deadline) {
			ufs_wr(REG_UTRLCLR, ~1u);
			printf("SWEET UFS: command 0x%02x timed out\n", cdb[0]);
			return ETIMEDOUT;
		}
		if (sleepable && now >= spin_until) {
			uint64_t wake = now + MIN(sleep_interval, deadline - now);
			/* Keep the mutex: slot 0 and the bounce buffer are still owned. */
			if (assert_wait_deadline_with_leeway((event_t)&su.wait, THREAD_UNINT,
			    TIMEOUT_URGENCY_SYS_CRITICAL, wake, 0) == THREAD_WAITING) {
				thread_block(THREAD_CONTINUE_NULL);
			}
		} else {
			delay(1);
		}
	}
	__builtin_arm_dsb(0xf);
	ufs_wr(REG_IS, ufs_rd(REG_IS));         /* acknowledge completion status */

	dma_sync(DMA_UTRL, sizeof(*utrd));
	if (su.cache) {
		dma_sync(DMA_UCD + offsetof(struct ucd, resp), sizeof(ucd->resp));
	} else {
		dma_sync(DMA_UCD, sizeof(*ucd));
	}
	if (len && !(su.cache && write)) {
		dma_sync(DMA_BOUNCE, len);
	}

	uint32_t ocs = utrd->dw2 & 0xff;
	if (ocs || ucd->resp[0] != 0x21 || ucd->resp[6] || ucd->resp[7]) {
		printf("SWEET UFS: command 0x%02x failed: ocs 0x%x upiu 0x%x response 0x%x status 0x%x\n",
		    cdb[0], ocs, ucd->resp[0], ucd->resp[6], ucd->resp[7]);
		return EIO;
	}
	return 0;
}

/* READ(10) of count device blocks at lba into the bounce buffer. */
static int
ufs_read_blocks(uint64_t lba, uint32_t count)
{
	if (lba > UINT32_MAX || count > BOUNCE_SIZE / UFS_BLOCK) {
		return EINVAL;
	}
	uint8_t cdb[10] = {
		0x28, 0,
		(uint8_t)(lba >> 24), (uint8_t)(lba >> 16), (uint8_t)(lba >> 8), (uint8_t)lba,
		0, (uint8_t)(count >> 8), (uint8_t)count, 0
	};
	return ufs_scsi(cdb, sizeof(cdb), count * UFS_BLOCK, false);
}

/*
 * topaz: on 2026-10-19 the LUN0 GPT (super, userdata, esp, win, ...) was found
 * destroyed after iOS sessions. Remember both GPT copies and fence them off;
 * a watchdog thread re-reads them and panics the moment they change.
 */
static uint64_t gpt_alt;                /* backup header LBA (last LBA of LUN0) */
static uint32_t gpt_hcrc, gpt_ecrc, gpt_bhcrc;
static int gpt_chan;

/*
 * WRITE(10) of count device blocks from the bounce buffer to lba. The range
 * must lie inside the partition: this is the only place data goes out.
 */
static int
ufs_write_blocks(uint64_t lba, uint32_t count)
{
	if (!su.writable || !su.nblocks || count == 0 || count > BOUNCE_SIZE / UFS_BLOCK ||
	    lba < su.first_block || lba - su.first_block >= su.nblocks ||
	    count > su.nblocks - (lba - su.first_block) || lba + count - 1 > UINT32_MAX ||
	    lba < 6 || (gpt_alt && lba + count - 1 >= gpt_alt - 4)) {
		printf("SWEET UFS: refusing write of %u blocks at 0x%llx\n", count, lba);
		return EPERM;
	}
	uint8_t cdb[10] = {
		0x2a, 0,
		(uint8_t)(lba >> 24), (uint8_t)(lba >> 16), (uint8_t)(lba >> 8), (uint8_t)lba,
		0, (uint8_t)(count >> 8), (uint8_t)count, 0
	};
	return ufs_scsi(cdb, sizeof(cdb), count * UFS_BLOCK, true);
}

static int
ufs_sync_cache(void)
{
	static const uint8_t cdb[10] = { 0x35 };       /* SYNCHRONIZE CACHE(10), whole LUN */
	return ufs_scsi(cdb, sizeof(cdb), 0, false);
}

static int
ufs_find_partition(const char *want)
{
	const uint8_t *b = (const uint8_t *)(su.dma + DMA_BOUNCE);
	int error = ufs_read_blocks(1, 1);
	if (error) {
		return error;
	}
	if (memcmp(b, "EFI PART", 8) != 0) {
		printf("SWEET UFS: LUN %u has no GPT\n", su.lun);
		return ENXIO;
	}
	uint64_t entry_lba;
	uint32_t nent, esz;
	memcpy(&gpt_hcrc, b + 16, 4);
	memcpy(&gpt_alt, b + 32, 8);
	memcpy(&gpt_ecrc, b + 88, 4);
	memcpy(&entry_lba, b + 72, 8);
	memcpy(&nent, b + 80, 4);
	memcpy(&esz, b + 84, 4);
	if (esz != 128 || nent == 0 || nent * esz > BOUNCE_SIZE) {
		printf("SWEET UFS: unsupported GPT entries %u x %u\n", nent, esz);
		return ENXIO;
	}
	error = ufs_read_blocks(entry_lba, (nent * esz + UFS_BLOCK - 1) / UFS_BLOCK);
	if (error) {
		return error;
	}
	for (uint32_t i = 0; i < nent; i++) {
		const uint8_t *e = b + i * esz;
		char name[37];
		unsigned c;
		for (c = 0; c < 36; c++) {
			uint16_t ch = (uint16_t)(e[56 + 2 * c] | e[57 + 2 * c] << 8);
			if (ch == 0 || ch > 0x7f) {
				break;
			}
			name[c] = (char)ch;
		}
		name[c] = 0;
		if (strcmp(name, want) != 0) {
			continue;
		}
		uint64_t first, last;
		memcpy(&first, e + 32, 8);
		memcpy(&last, e + 40, 8);
		if (last < first) {
			return ENXIO;
		}
		su.first_block = first;
		su.nblocks = last - first + 1;
		if (ufs_read_blocks(gpt_alt, 1) == 0 && memcmp(b, "EFI PART", 8) == 0) {
			memcpy(&gpt_bhcrc, b + 16, 4);
		}
		printf("SWEET UFS: GPT LUN0 hcrc %#x ecrc %#x backup@%#llx hcrc %#x\n",
		    gpt_hcrc, gpt_ecrc, gpt_alt, gpt_bhcrc);
		return 0;
	}
	printf("SWEET UFS: no GPT partition named \"%s\"\n", want);
	return ENOENT;
}

/*
 * topaz: read-only access to other GPT partitions on LUNs 0-5 (modem firmware in
 * modem_a, later the EFS copies). Never writes; the LUN0 GPT fence state is untouched.
 */
static struct { char name[37]; uint8_t lun; uint64_t first, nblocks; } part_cache[8];
static unsigned part_cached;

static int
ufs_lookup_part(const char *want, unsigned *slot)
{
	const uint8_t *b = (const uint8_t *)(su.dma + DMA_BOUNCE);

	for (unsigned i = 0; i < part_cached; i++) {
		if (strcmp(part_cache[i].name, want) == 0) {
			*slot = i;
			return 0;
		}
	}
	if (part_cached == sizeof(part_cache) / sizeof(part_cache[0]) || strlen(want) > 36) {
		return ENOSPC;
	}
	for (uint8_t lun = 0; lun < 6; lun++) {
		su.lun = lun;
		if (ufs_read_blocks(1, 1) != 0 || memcmp(b, "EFI PART", 8) != 0) {
			continue;
		}
		uint64_t entry_lba;
		uint32_t nent, esz;
		memcpy(&entry_lba, b + 72, 8);
		memcpy(&nent, b + 80, 4);
		memcpy(&esz, b + 84, 4);
		if (esz != 128 || nent == 0 || nent * esz > BOUNCE_SIZE ||
		    ufs_read_blocks(entry_lba, (nent * esz + UFS_BLOCK - 1) / UFS_BLOCK) != 0) {
			continue;
		}
		for (uint32_t i = 0; i < nent; i++) {
			const uint8_t *e = b + i * esz;
			char name[37];
			unsigned c;
			for (c = 0; c < 36; c++) {
				uint16_t ch = (uint16_t)(e[56 + 2 * c] | e[57 + 2 * c] << 8);
				if (ch == 0 || ch > 0x7f) {
					break;
				}
				name[c] = (char)ch;
			}
			name[c] = 0;
			uint64_t first, last;
			memcpy(&first, e + 32, 8);
			memcpy(&last, e + 40, 8);
			if (strcmp(name, want) != 0 || last < first) {
				continue;
			}
			unsigned k = part_cached++;
			strlcpy(part_cache[k].name, want, sizeof(part_cache[k].name));
			part_cache[k].lun = lun;
			part_cache[k].first = first;
			part_cache[k].nblocks = last - first + 1;
			printf("SWEET UFS: partition %s: LUN %u blocks 0x%llx+0x%llx (read-only)\n",
			    want, lun, first, last - first + 1);
			*slot = k;
			return 0;
		}
	}
	return ENOENT;
}

int sweet_ufs_read_part(const char *want, uint64_t off, void *dst, uint64_t len);
int
sweet_ufs_read_part(const char *want, uint64_t off, void *dst, uint64_t len)
{
	unsigned slot = 0;
	int error;

	if (!su.ready) {
		return EAGAIN;
	}
	lck_mtx_lock(&sweet_ufs_lock);
	uint8_t saved = su.lun;
	error = ufs_lookup_part(want, &slot);
	uint64_t bytes = error ? 0 : part_cache[slot].nblocks * UFS_BLOCK;
	if (!error && (off > bytes || len > bytes - off)) {
		error = EINVAL;
	}
	for (uint64_t done = 0; !error && done < len;) {
		uint64_t pos = off + done;
		uint32_t skip = (uint32_t)(pos % UFS_BLOCK);
		uint32_t want_b = (uint32_t)MIN(len - done, (uint64_t)(BOUNCE_SIZE - skip));
		uint32_t blocks = (skip + want_b + UFS_BLOCK - 1) / UFS_BLOCK;
		su.lun = part_cache[slot].lun;
		error = ufs_read_blocks(part_cache[slot].first + pos / UFS_BLOCK, blocks);
		if (!error) {
			memcpy((uint8_t *)dst + done, (const uint8_t *)(su.dma + DMA_BOUNCE) + skip, want_b);
			done += want_b;
		}
	}
	su.lun = saved;
	lck_mtx_unlock(&sweet_ufs_lock);
	return error;
}

static int
ufs_take_over_host(void)
{
	uint32_t hce = ufs_rd(REG_HCE), hcs = ufs_rd(REG_HCS);
	if (!(hce & 1) || !(hcs & HCS_DP) || !(hcs & HCS_UTRLRDY)) {
		printf("SWEET UFS: host not ready (hce 0x%x hcs 0x%x)\n", hce, hcs);
		return ENXIO;
	}
	if (ufs_rd(REG_UTRLDBR)) {
		printf("SWEET UFS: requests outstanding from the bootloader\n");
		return EBUSY;
	}
	/* With RAM above 4 GiB (mkadt.py --highmem) the DMA memory can be there too. */
	if (((su.dma_pa + DMA_SIZE - 1) >> 32) && !(ufs_rd(REG_CAP) & CAP_64AS)) {
		printf("SWEET UFS: DMA memory at 0x%llx but the host has no 64-bit addressing\n",
		    su.dma_pa);
		return ENXIO;
	}
	ufs_wr(REG_IE, 0);                      /* polled operation */
	ufs_wr(REG_IS, ufs_rd(REG_IS));
	ufs_wr(REG_UTRLRSR, 0);
	uint64_t list = su.dma_pa + DMA_UTRL;
	ufs_wr(REG_UTRLBA, (uint32_t)list);
	ufs_wr(REG_UTRLBAU, (uint32_t)(list >> 32));
	ufs_wr(REG_UTRLRSR, 1);
	/* Never ring a doorbell unless the host uses our list. */
	if (ufs_rd(REG_UTRLBA) != (uint32_t)list || ufs_rd(REG_UTRLBAU) != (uint32_t)(list >> 32) ||
	    !(ufs_rd(REG_UTRLRSR) & 1)) {
		printf("SWEET UFS: host did not take our request list\n");
		return EIO;
	}
	return 0;
}

static int
sweet_ufs_open(__unused dev_t dev, int flags, __unused int devtype, __unused struct proc *p)
{
	if (!su.ready) {
		return ENXIO;
	}
	return ((flags & FWRITE) && !su.writable) ? EROFS : 0;
}

static int
sweet_ufs_close(__unused dev_t dev, __unused int flags, __unused int devtype, __unused struct proc *p)
{
	return 0;
}

static void
sweet_ufs_strategy(buf_t bp)
{
	int error = 0;
	caddr_t vaddr = NULL;
	uint64_t off = (uint64_t)buf_blkno(bp) * su.secsize;
	uint32_t count = buf_count(bp);
	uint64_t part_bytes = su.nblocks * UFS_BLOCK;

	bool rd = buf_flags(bp) & B_READ;

	buf_setresid(bp, count);
	/*
	 * sapphire 0334: a physio() buffer (B_PHYS without B_CLUSTER) points at user memory, which
	 * buf_map() hands back as is. The raw device no longer sends any here (sweet_ufs_rw); refuse
	 * one rather than memcpy to it. Cluster direct I/O (B_PHYS | B_CLUSTER, e.g. HFS uncached
	 * reads) is fine: buf_map() maps its UPL into the kernel.
	 */
	if ((buf_flags(bp) & (B_PHYS | B_CLUSTER)) == B_PHYS) {
		error = ENOTSUP;
		goto done;
	}
	if (!rd && !su.writable) {
		error = EROFS;
		goto done;
	}
	if (off > part_bytes || count > part_bytes - off) {
		error = EINVAL;
		goto done;
	}
	if (count == 0) {
		goto done;
	}
	if (buf_map(bp, &vaddr)) {
		error = EIO;
		goto done;
	}

	lck_mtx_lock(&sweet_ufs_lock);
	for (uint32_t done = 0; done < count;) {
		uint64_t pos = off + done;
		uint64_t lba = pos / UFS_BLOCK;
		uint32_t skip = (uint32_t)(pos % UFS_BLOCK);
		uint32_t want = MIN(count - done, BOUNCE_SIZE - skip);
		uint32_t blocks = (skip + want + UFS_BLOCK - 1) / UFS_BLOCK;
		void *bounce = (void *)(su.dma + DMA_BOUNCE);
		if (rd) {
			error = ufs_read_blocks(su.first_block + lba, blocks);
			if (!error) {
				memcpy(vaddr + done, (const char *)bounce + skip, want);
			}
		} else {
			/* Partial device blocks at either end: read, merge, write back. */
			if (skip || (want % UFS_BLOCK)) {
				error = ufs_read_blocks(su.first_block + lba, blocks);
			}
			if (!error) {
				memcpy((char *)bounce + skip, vaddr + done, want);
				error = ufs_write_blocks(su.first_block + lba, blocks);
			}
		}
		if (error) {
			break;
		}
		done += want;
		buf_setresid(bp, count - done);
	}
	lck_mtx_unlock(&sweet_ufs_lock);
	buf_unmap(bp);

done:
	if (error) {
		buf_seterror(bp, error);
	}
	buf_biodone(bp);
}

static int
sweet_ufs_ioctl(__unused dev_t dev, u_long cmd, caddr_t data, __unused int flag,
    __unused struct proc *p, int is_char)
{
	uint32_t *f = (uint32_t *)data;
	uint64_t *o = (uint64_t *)data;

	switch (cmd) {
	case DKIOCGETBLOCKSIZE:
		*f = su.secsize;
		return 0;
	case DKIOCSETBLOCKSIZE:
		if (is_char) {
			return ENODEV;
		}
		if (*f < DEV_BSIZE || *f > UFS_BLOCK || (*f & (*f - 1))) {
			return EINVAL;
		}
		su.secsize = *f;
		return 0;
	case DKIOCGETPHYSICALBLOCKSIZE:
		*f = UFS_BLOCK;
		return 0;
	case DKIOCGETBLOCKCOUNT:
		*o = su.nblocks * UFS_BLOCK / su.secsize;
		return 0;
	case DKIOCISWRITABLE:
		*f = su.writable;
		return 0;
	case DKIOCSYNCHRONIZE:
	case DKIOCSYNCHRONIZECACHE: {
		if (!su.writable) {
			return 0;
		}
		lck_mtx_lock(&sweet_ufs_lock);
		int error = ufs_sync_cache();
		lck_mtx_unlock(&sweet_ufs_lock);
		return error;
	}
	case DKIOCGETMAXBLOCKCOUNTREAD:
		*o = BOUNCE_SIZE / su.secsize;
		return 0;
	case DKIOCGETMAXBLOCKCOUNTWRITE:
		*o = su.writable ? BOUNCE_SIZE / su.secsize : 0;
		return 0;
	case DKIOCGETMAXSEGMENTCOUNTREAD:
	case DKIOCGETMAXSEGMENTCOUNTWRITE:
		*o = 1;
		return 0;
	default:
		return ENOTTY;
	}
}

static int
sweet_ufs_bioctl(dev_t dev, u_long cmd, caddr_t data, int flag, struct proc *p)
{
	return sweet_ufs_ioctl(dev, cmd, data, flag, p, 0);
}

static int
sweet_ufs_cioctl(dev_t dev, u_long cmd, caddr_t data, int flag, struct proc *p)
{
	return sweet_ufs_ioctl(dev, cmd, data, flag, p, 1);
}

static int
sweet_ufs_psize(__unused dev_t dev)
{
	return su.ready ? (int)MIN(su.nblocks * UFS_BLOCK / su.secsize, INT_MAX) : -1;
}

/*
 * sapphire 0334: read/write of the raw device (rufsroot0). These used physio(), whose buf
 * carries the caller's USER address; sweet_ufs_strategy() memcpy'd straight to/from it. That
 * memcpy is not in copyio's recovery table, so any fault on the user page -- even one the VM
 * resolves on every copyout, like the first write to a page mapped read-only to track the
 * modified bit -- is driven against kernel_map and panics (osfmk/arm64/sleh.c,
 * handle_kernel_abort). 2026-11-14: dd if=/dev/rufsroot0 bs=4096 ... -> data abort in memcpy
 * from sweet_ufs_strategy, far 0x1006f4000, esr 0x9600004f.
 *
 * Now: chunks of at most BOUNCE_SIZE. The device I/O goes through the bounce buffer under
 * sweet_ufs_lock exactly as in strategy (partial device blocks of a write are read, merged and
 * written back); the data is staged in a kernel buffer and moved with uiomove() (copyin /
 * copyout, faults handled) with the lock dropped, since such a fault may page in from this
 * same disk. Offsets must be multiples of the sector size (physio silently rounded them down);
 * a read is cut at the end of the partition (0 = end of file); a write that does not fit is
 * refused whole, before anything is written.
 */
static int
sweet_ufs_rw(struct uio *uio, bool rd)
{
	uint64_t part_bytes = su.nblocks * UFS_BLOCK;
	uint32_t secsize = su.secsize;
	off_t off = uio_offset(uio);
	user_ssize_t resid = uio_resid(uio);
	int error = 0;

	if (!su.ready) {
		return ENXIO;
	}
	if (!rd && !su.writable) {
		return EROFS;
	}
	if (off < 0 || (uint64_t)off % secsize) {
		return EINVAL;
	}
	if (resid <= 0 || (rd && (uint64_t)off >= part_bytes)) {
		return 0;
	}
	if ((uint64_t)off > part_bytes) {
		return EINVAL;
	}
	uint64_t total = MIN((uint64_t)resid, part_bytes - (uint64_t)off);
	if (!rd && total < (uint64_t)resid) {
		return EINVAL;
	}

	uint32_t bufsize = (uint32_t)MIN(total, (uint64_t)BOUNCE_SIZE);
	char *kbuf = kalloc_data(bufsize, Z_WAITOK);
	if (kbuf == NULL) {
		return ENOMEM;
	}
	char *bounce = (char *)(su.dma + DMA_BOUNCE);
	for (uint64_t done = 0; done < total;) {
		uint64_t pos = (uint64_t)off + done;
		uint64_t lba = su.first_block + pos / UFS_BLOCK;
		uint32_t skip = (uint32_t)(pos % UFS_BLOCK);
		uint32_t want = (uint32_t)MIN(total - done, (uint64_t)(BOUNCE_SIZE - skip));
		want = MIN(want, bufsize);
		uint32_t blocks = (skip + want + UFS_BLOCK - 1) / UFS_BLOCK;

		if (!rd && (error = uiomove(kbuf, (int)want, uio))) {
			break;
		}
		lck_mtx_lock(&sweet_ufs_lock);
		if (rd) {
			error = ufs_read_blocks(lba, blocks);
			if (!error) {
				memcpy(kbuf, bounce + skip, want);
			}
		} else {
			/* Partial device blocks at either end: read, merge, write back. */
			if (skip || (want % UFS_BLOCK)) {
				error = ufs_read_blocks(lba, blocks);
			}
			if (!error) {
				memcpy(bounce + skip, kbuf, want);
				error = ufs_write_blocks(lba, blocks);
			}
		}
		lck_mtx_unlock(&sweet_ufs_lock);
		if (!error && rd) {
			error = uiomove(kbuf, (int)want, uio);
		}
		if (error) {
			break;
		}
		done += want;
	}
	kfree_data(kbuf, bufsize);
	return error;
}

static int
sweet_ufs_read(__unused dev_t dev, struct uio *uio, __unused int ioflag)
{
	return sweet_ufs_rw(uio, true);
}

static int
sweet_ufs_write(__unused dev_t dev, struct uio *uio, __unused int ioflag)
{
	return sweet_ufs_rw(uio, false);
}

static const struct bdevsw sweet_ufs_bdevsw = {
	.d_open     = sweet_ufs_open,
	.d_close    = sweet_ufs_close,
	.d_strategy = sweet_ufs_strategy,
	.d_ioctl    = sweet_ufs_bioctl,
	.d_dump     = eno_dump,
	.d_psize    = sweet_ufs_psize,
	.d_type     = D_DISK,
};

static const struct cdevsw sweet_ufs_cdevsw = {
	.d_open       = sweet_ufs_open,
	.d_close      = sweet_ufs_close,
	.d_read       = sweet_ufs_read,
	.d_write      = sweet_ufs_write,
	.d_ioctl      = sweet_ufs_cioctl,
	.d_stop       = eno_stop,
	.d_reset      = eno_reset,
	.d_ttys       = NULL,
	.d_select     = eno_select,
	.d_mmap       = eno_mmap,
	.d_strategy   = eno_strat,
	.d_reserved_1 = eno_getc,
	.d_reserved_2 = eno_putc,
	.d_type       = D_DISK,
};

static void
sweet_gpt_watch(__unused void *arg, __unused wait_result_t wr)
{
	const uint8_t *b = (const uint8_t *)(su.dma + DMA_BOUNCE);
	uint64_t start = mach_absolute_time();
	for (;;) {
		uint64_t deadline;
		clock_interval_to_deadline(10, NSEC_PER_SEC, &deadline);
		(void)assert_wait_deadline(&gpt_chan, THREAD_UNINT, deadline);
		thread_block(THREAD_CONTINUE_NULL);

		uint32_t h = 0, e = 0, bh = 0;
		bool ok;
		lck_mtx_lock(&sweet_ufs_lock);
		ok = ufs_read_blocks(1, 1) == 0 && memcmp(b, "EFI PART", 8) == 0;
		if (ok) {
			memcpy(&h, b + 16, 4);
			memcpy(&e, b + 88, 4);
		}
		if (ok && gpt_bhcrc && ufs_read_blocks(gpt_alt, 1) == 0) {
			if (memcmp(b, "EFI PART", 8) == 0) {
				memcpy(&bh, b + 16, 4);
			}
		}
		lck_mtx_unlock(&sweet_ufs_lock);
		if (!ok || h != gpt_hcrc || e != gpt_ecrc || (gpt_bhcrc && bh != gpt_bhcrc)) {
			uint64_t ns;
			absolutetime_to_nanoseconds(mach_absolute_time() - start, &ns);
			panic("SWEET UFS: GPT LUN0 CHANGED after %llu s: primary %s hcrc %#x/%#x ecrc %#x/%#x backup hcrc %#x/%#x",
			    ns / NSEC_PER_SEC, ok ? "present" : "MISSING", h, gpt_hcrc, e, gpt_ecrc, bh, gpt_bhcrc);
		}
	}
}

static int
sweet_ufs_init(void)
{
	DTEntry node;
	uintptr_t const *reg = NULL;
	unsigned int size = 0;
	char want[64] = "userdata";
	int error;

	if (SecureDTLookupEntry(NULL, "/sweet-ufs", &node) != kSuccess ||
	    SecureDTGetProperty(node, "reg", (void const **)&reg, &size) != kSuccess ||
	    size < 2 * sizeof(uintptr_t)) {
		printf("SWEET UFS: no /sweet-ufs node in the device tree\n");
		return ENXIO;
	}
	PE_parse_boot_argn("ufsroot", want, sizeof(want));
	uint32_t rw = 0;
	PE_parse_boot_argn("ufsrw", &rw, sizeof(rw));
	uint32_t cache = 0, wait = 0;
	PE_parse_boot_argn("ufscache", &cache, sizeof(cache));
	PE_parse_boot_argn("ufswait", &wait, sizeof(wait));
	su.cache = (cache == 1);
	su.wait = (wait == 1);

	su.regs = ml_io_map(reg[0], reg[1]);
	if (kmem_alloc_contig(kernel_map, &su.dma, DMA_SIZE, PAGE_MASK, 0, 0,
	    KMA_DATA | KMA_ZERO | KMA_PERMANENT, VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		printf("SWEET UFS: no contiguous DMA memory\n");
		return ENOMEM;
	}
	ppnum_t pn = pmap_find_phys(kernel_pmap, (addr64_t)su.dma);
	if (!pn) {
		return ENOMEM;
	}
	su.dma_pa = ptoa_64(pn);
	su.lun = 0;
	su.secsize = DEV_BSIZE;

	lck_mtx_lock(&sweet_ufs_lock);
	error = ufs_take_over_host();
	if (!error) {
		static const uint8_t tur[6] = { 0x00 };
		/* The first command after the bootloader may report a unit attention. */
		for (int i = 0; i < 3 && (error = ufs_scsi(tur, sizeof(tur), 0, false)); i++) {
		}
	}
	if (!error) {
		error = ufs_find_partition(want);
	}
	lck_mtx_unlock(&sweet_ufs_lock);
	if (error) {
		return error;
	}

	su.bmajor = bdevsw_add(-1, &sweet_ufs_bdevsw);
	if (su.bmajor < 0) {
		return ENXIO;
	}
	su.cmajor = cdevsw_add_with_bdev(-1, &sweet_ufs_cdevsw, su.bmajor);
	if (su.cmajor < 0) {
		return ENXIO;
	}
	su.bdev = makedev(su.bmajor, 0);
	devfs_make_node(su.bdev, DEVFS_BLOCK, UID_ROOT, GID_OPERATOR, 0640, "ufsroot0");
	devfs_make_node(makedev(su.cmajor, 0), DEVFS_CHAR, UID_ROOT, GID_OPERATOR, 0640, "rufsroot0");
	/* Only now, with the partition bounds known, can writes be allowed. */
	su.writable = (rw != 0);
	su.ready = true;
	printf("SWEET UFS: \"%s\" on LUN %u: first block 0x%llx, %llu MiB, %s, DMA at 0x%llx\n",
	    want, su.lun, su.first_block, su.nblocks * UFS_BLOCK >> 20,
	    su.writable ? "read-write" : "read-only", su.dma_pa);
	sweet_log_start();
	sweet_sample_start();
	thread_t gpt_thread;
	if (kernel_thread_start(sweet_gpt_watch, NULL, &gpt_thread) == KERN_SUCCESS) {
		thread_deallocate(gpt_thread);
	}
	/*
	 * Input drivers (HID services in iokit/Kernel/arm/hid/SweetHID.cpp).
	 * Touch (FT5452 on QUP0 SE2) and keys (PM6125 over SPMI) are ported to
	 * topaz; sweettouch=0 / sweetkeys=0 disable them. USB (ported to SM6225)
	 * is opt-in: sweetusb=1 reports its state, sweetusb=2 starts the device.
	 */
	sweet_touch_init();
	sweet_keys_init();
	sweet_bt_boot_init();
	sweet_usb_probe();
	return 0;
}

/*
 * Persistent console log ("sweetlog"). Filming the phone screen is the only
 * other way to see the console, so every character that reaches the
 * console (kernel printf and /dev/console writes of daemons, both through
 * _cnputs) is also kept in a 4 MiB ring, and once the partition is known
 * and writable a thread copies new ring blocks to the LAST 4 MiB + 4 KiB of
 * the partition every ufslogsecs seconds (default 10), far behind the HFS
 * image at its start. Device block 0 of that area is a header:
 *   "SWEETLOG" | u32 version 1 | u32 ring size | u64 total bytes written
 *   | u64 flush sequence
 * ring offset of byte n = n % ring size. Read it from a recovery with
 * tools/sweet-log-pull.sh. Disabled with boot-arg sweetlog=0.
 */
#define SWEET_LOG_SIZE          (4u << 20)
#define SWEET_LOG_BLOCKS        (1 + SWEET_LOG_SIZE / UFS_BLOCK)
#define SWEET_LOG_SECS          10u

static char sweet_log_ring[SWEET_LOG_SIZE];
static uint64_t _Atomic sweet_log_total;
static int sweet_log_chan;
static uint64_t _Atomic sweet_log_flushed;      /* 0274: what the log thread has written */
static uint32_t sweet_log_secs = SWEET_LOG_SECS;

/* Called for all console output, any context: never blocks. */
void sweet_log_append(const char *c, int size);

void
sweet_log_append(const char *c, int size)
{
	if (size <= 0) {
		return;
	}
	uint64_t at = os_atomic_add_orig(&sweet_log_total, (uint64_t)size, relaxed);
	for (int i = 0; i < size; i++) {
		sweet_log_ring[(at + (uint64_t)i) % SWEET_LOG_SIZE] = c[i];
	}
}

/*
 * Copy console bytes [*pos, total) (at most max) for live log streaming
 * (sweet_usb.c). A reader that fell more than the ring behind skips ahead.
 * Returns the byte count; *pos advances.
 */
uint32_t sweet_log_copy(uint64_t *pos, char *out, uint32_t max);

uint32_t
sweet_log_copy(uint64_t *pos, char *out, uint32_t max)
{
	uint64_t total = os_atomic_load(&sweet_log_total, relaxed);
	if (total - *pos > SWEET_LOG_SIZE) {
		*pos = total - SWEET_LOG_SIZE;
	}
	uint32_t n = (uint32_t)MIN((uint64_t)max, total - *pos);
	for (uint32_t i = 0; i < n; i++) {
		out[i] = sweet_log_ring[(*pos + i) % SWEET_LOG_SIZE];
	}
	*pos += n;
	return n;
}

uint64_t sweet_log_position(void);

uint64_t
sweet_log_position(void)
{
	return os_atomic_load(&sweet_log_total, relaxed);
}

struct sweet_log_header {
	char magic[8];
	uint32_t version, size;
	uint64_t total, seq;
};

static void
sweet_log_thread(__unused void *arg, __unused wait_result_t wr)
{
	uint64_t base = su.first_block + su.nblocks - SWEET_LOG_BLOCKS;
	uint64_t flushed = 0, seq = 0;
	extern void IOSleep(unsigned int);
	void *bounce = (void *)(su.dma + DMA_BOUNCE);

	printf("SWEET LOG: console log to the last %u KiB of the partition (block 0x%llx)\n",
	    SWEET_LOG_BLOCKS * UFS_BLOCK >> 10, base);
	for (;;) {
		(void)tsleep(&sweet_log_chan, PZERO, "sweetlog", hz * (int)sweet_log_secs);
		uint64_t total = os_atomic_load(&sweet_log_total, relaxed);
		if (total == flushed) {
			continue;
		}
		uint64_t pos = total - flushed >= SWEET_LOG_SIZE ? total - SWEET_LOG_SIZE : flushed;
		int error = 0;

		lck_mtx_lock(&sweet_ufs_lock);
		while (pos < total && !error) {
			uint32_t off = (uint32_t)(pos % SWEET_LOG_SIZE);
			uint32_t blk = off / UFS_BLOCK;
			uint64_t need = ((uint64_t)off % UFS_BLOCK + (total - pos) + UFS_BLOCK - 1) / UFS_BLOCK;
			uint32_t n = (uint32_t)MIN(MIN(need, (uint64_t)(SWEET_LOG_SIZE / UFS_BLOCK - blk)),
			    (uint64_t)(BOUNCE_SIZE / UFS_BLOCK));
			memcpy(bounce, sweet_log_ring + (uint64_t)blk * UFS_BLOCK, n * UFS_BLOCK);
			error = ufs_write_blocks(base + 1 + blk, n);
			pos += (uint64_t)(blk + n) * UFS_BLOCK - off;
		}
		if (!error) {
			struct sweet_log_header h = {
				.magic = { 'S', 'W', 'E', 'E', 'T', 'L', 'O', 'G' },
				.version = 1, .size = SWEET_LOG_SIZE, .total = total, .seq = ++seq,
			};
			bzero(bounce, UFS_BLOCK);
			memcpy(bounce, &h, sizeof(h));
			error = ufs_write_blocks(base, 1);
		}
		if (!error) {
			error = ufs_sync_cache();
		}
		lck_mtx_unlock(&sweet_ufs_lock);
		if (error) {
			printf("SWEET LOG: write failed (%d), stopping\n", error);
			return;
		}
		flushed = total;
		os_atomic_store(&sweet_log_flushed, total, relaxed);
	}
}

static bool sweet_log_on;

/*
 * 0274: write the console log to the partition now and wait (up to max_ms) until it is there,
 * so that the last lines before a risky step survive a hang (sweet_cam.c).
 */
void sweet_log_sync(unsigned max_ms);
void
sweet_log_sync(unsigned max_ms)
{
	extern void IOSleep(unsigned int);
	if (!sweet_log_on) {
		return;
	}
	uint64_t want = os_atomic_load(&sweet_log_total, relaxed);
	wakeup(&sweet_log_chan);
	for (unsigned ms = 0; ms < max_ms && os_atomic_load(&sweet_log_flushed, relaxed) < want; ms += 5) {
		IOSleep(5);
	}
}

/*
 * Black box. Called from SavePanicInfo() (osfmk/arm/model_dep.c): the panic string has
 * already gone through the console into the ring, every other CPU is halted and
 * interrupts are off. The thread above cannot run any more, so this writes the ring the
 * same way without its mutex (whoever holds it is never coming back) and without
 * sleeping (ufs_scsi spins when interrupts are off). A command that thread left in
 * flight gets a bounded wait for its doorbell first. The header's seq is all ones so the
 * reader knows this flush came from a panic. Without this the only trace of a panic
 * is the screen, and the last ufslogsecs of console before it are gone.
 */
void sweet_log_panic_flush(void);

void
sweet_log_panic_flush(void)
{
	static bool done;
	uint64_t base = su.first_block + su.nblocks - SWEET_LOG_BLOCKS;
	void *bounce = (void *)(su.dma + DMA_BOUNCE);
	int error = 0;

	if (done || !sweet_log_on) {
		return;
	}
	done = true;
	for (int i = 0; i < 3000 && (ufs_rd(REG_UTRLDBR) & 1); i++) {
		delay(1000);
	}
	uint64_t total = os_atomic_load(&sweet_log_total, relaxed);
	uint64_t pos = total >= SWEET_LOG_SIZE ? total - SWEET_LOG_SIZE : 0;

	while (pos < total && !error) {
		uint32_t off = (uint32_t)(pos % SWEET_LOG_SIZE);
		uint32_t blk = off / UFS_BLOCK;
		uint64_t need = ((uint64_t)off % UFS_BLOCK + (total - pos) + UFS_BLOCK - 1) / UFS_BLOCK;
		uint32_t n = (uint32_t)MIN(MIN(need, (uint64_t)(SWEET_LOG_SIZE / UFS_BLOCK - blk)),
		    (uint64_t)(BOUNCE_SIZE / UFS_BLOCK));
		memcpy(bounce, sweet_log_ring + (uint64_t)blk * UFS_BLOCK, n * UFS_BLOCK);
		error = ufs_write_blocks(base + 1 + blk, n);
		pos += (uint64_t)(blk + n) * UFS_BLOCK - off;
	}
	if (!error) {
		struct sweet_log_header h = {
			.magic = { 'S', 'W', 'E', 'E', 'T', 'L', 'O', 'G' },
			.version = 1, .size = SWEET_LOG_SIZE, .total = total, .seq = ~0ull,
		};
		bzero(bounce, UFS_BLOCK);
		memcpy(bounce, &h, sizeof(h));
		error = ufs_write_blocks(base, 1);
	}
	if (!error) {
		error = ufs_sync_cache();
	}
	printf("SWEET PANIC: console log %s to the partition (%llu bytes)\n",
	    error ? "NOT written" : "written", total);
}

void sweet_slot_mark_start(void);

static void
sweet_log_start(void)
{
	sweet_slot_mark_start();
	uint32_t on = 1;
	thread_t thread;

	PE_parse_boot_argn("sweetlog", &on, sizeof(on));
	if (!on || !su.writable || su.nblocks < 2 * SWEET_LOG_BLOCKS) {
		return;
	}
	PE_parse_boot_argn("ufslogsecs", &sweet_log_secs, sizeof(sweet_log_secs));
	if (sweet_log_secs == 0 || sweet_log_secs > 60) {
		sweet_log_secs = SWEET_LOG_SECS;
	}
	if (kernel_thread_start(sweet_log_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
		sweet_log_on = true;
	}
}

/* Root device for rd=ufsroot0 (IOFindBSDRoot). */
int sweet_ufs_rootdev(dev_t *dev);

int
sweet_ufs_rootdev(dev_t *dev)
{
	if (!su.ready) {
		int error = sweet_ufs_init();
		if (error) {
			return error;
		}
	}
	*dev = su.bdev;
	return 0;
}
#endif /* ARM64_BOARD_CONFIG_SWEET */

/*
 * sapphire 0305: the phone handed back to Android. The A/B slot that is booted is kept in the GPT of the
 * LUN holding boot_a/boot_b (sde): the active bit of every _a/_b partition and the type GUIDs, which
 * Qualcomm's bootloader swaps between each X_a/X_b pair when it changes slots (fastboot set_active) and
 * checks before it loads anything. sweet_slot_switch() does what it does (checked against the GPTs this
 * phone's ABL wrote, userland/dualboot/test_slot.c); here both copies are read, checked, changed, written
 * back -- backup first, primary last, only those sectors, only on that LUN -- and read again.
 *
 * kern.sweet_boot_android (anybody may write it; the iOS app does): 1 = the other slot (Android's) active,
 * then reboot; 2 = the same without the reboot (a test). Reads: 'a' or 'b', the active slot, 0 unknown.
 * "The other slot" is the one that is not active in the GPT, not the one iOS was loaded from: it is
 * Android's only when iOS booted from its own active slot (the installed layout). After `fastboot
 * boot` with Android's slot active, writing 1 makes iOS's slot active instead.
 */
static int
ufs_write_slot_blocks(uint8_t lun, uint64_t lba, uint32_t count)
{
	if (!su.writable || su.lun != lun || count == 0 || count > 8 || lba > UINT32_MAX) {
		return EPERM;
	}
	uint8_t cdb[10] = {
		0x2a, 0,
		(uint8_t)(lba >> 24), (uint8_t)(lba >> 16), (uint8_t)(lba >> 8), (uint8_t)lba,
		0, (uint8_t)(count >> 8), (uint8_t)count, 0
	};
	return ufs_scsi(cdb, sizeof(cdb), count * UFS_BLOCK, true);
}

/* LBA's COUNT blocks into DST (COUNT * UFS_BLOCK bytes), on the current LUN */
static int
ufs_read_into(uint64_t lba, uint32_t count, uint8_t *dst)
{
	int error = ufs_read_blocks(lba, count);
	if (!error) {
		memcpy(dst, (const void *)(su.dma + DMA_BOUNCE), count * UFS_BLOCK);
	}
	return error;
}

static int
ufs_write_from(uint8_t lun, uint64_t lba, uint32_t count, const uint8_t *src)
{
	memcpy((void *)(su.dma + DMA_BOUNCE), src, count * UFS_BLOCK);
	return ufs_write_slot_blocks(lun, lba, count);
}

static int sweet_slot_active_cached;

/*
 * sapphire 0306: UFS QUERY REQUEST for a device attribute (UFS 2.1 10.7.8): opcode 3 read, 4 write.
 * IDN 0 is bBootLunEn, the boot well-known LUN the primary boot loader (XBL) is read from: 1 = Boot LU A
 * (xbl_a, slot a), 2 = Boot LU B. Qualcomm's bootloader checks it against the GPT's active slot
 * (FindBootableSlot: "Boot lun ... and BootableSlot ... do not match") and loads nothing when they
 * differ; fastboot set_active and Android's boot HAL change both. With sweet_ufs_lock held.
 */
static int
ufs_query_attr(bool write, uint8_t idn, uint32_t *value)
{
	struct utrd *utrd = (struct utrd *)(su.dma + DMA_UTRL);
	struct ucd *ucd = (struct ucd *)(su.dma + DMA_UCD);
	uint64_t ucd_pa = su.dma_pa + DMA_UCD;

	bzero(utrd, sizeof(*utrd));
	bzero(ucd, sizeof(*ucd));
	ucd->cmd[0] = 0x16;                             /* QUERY REQUEST UPIU */
	ucd->cmd[3] = ++su.tag;
	ucd->cmd[5] = write ? 0x81 : 0x01;              /* standard write / read request */
	ucd->cmd[12] = write ? 0x04 : 0x03;             /* WRITE / READ ATTRIBUTE */
	ucd->cmd[13] = idn;
	if (write) {
		ucd->cmd[20] = (uint8_t)(*value >> 24);
		ucd->cmd[21] = (uint8_t)(*value >> 16);
		ucd->cmd[22] = (uint8_t)(*value >> 8);
		ucd->cmd[23] = (uint8_t)*value;
	}
	utrd->dw0 = (1u << 28) | (1u << 24);            /* UFS storage command type, interrupt; no data */
	utrd->dw2 = 0x0f;
	utrd->ucdba = (uint32_t)ucd_pa;
	utrd->ucdbau = (uint32_t)(ucd_pa >> 32);
	utrd->resp = (512u / 4) << 16 | (512u / 4);
	dma_clean(DMA_UTRL, sizeof(*utrd));
	dma_clean(DMA_UCD, 32);
	dma_sync(DMA_UCD + offsetof(struct ucd, resp), sizeof(ucd->resp));
	__builtin_arm_dsb(0xf);
	ufs_wr(REG_UTRLDBR, 1);
	uint64_t deadline;
	nanoseconds_to_absolutetime(CMD_TIMEOUT_NS, &deadline);
	deadline += mach_absolute_time();
	while (ufs_rd(REG_UTRLDBR) & 1) {
		if (mach_absolute_time() > deadline) {
			ufs_wr(REG_UTRLCLR, ~1u);
			printf("SWEET UFS: query %s attribute %u timed out\n", write ? "write" : "read", idn);
			return ETIMEDOUT;
		}
		delay(10);
	}
	__builtin_arm_dsb(0xf);
	ufs_wr(REG_IS, ufs_rd(REG_IS));
	dma_sync(DMA_UTRL, sizeof(*utrd));
	dma_sync(DMA_UCD + offsetof(struct ucd, resp), sizeof(ucd->resp));
	uint32_t ocs = utrd->dw2 & 0xff;
	if (ocs || ucd->resp[0] != 0x36 || ucd->resp[6] != 0) {
		printf("SWEET UFS: query %s attribute %u failed: ocs 0x%x upiu 0x%x response 0x%x\n",
		    write ? "write" : "read", idn, ocs, ucd->resp[0], ucd->resp[6]);
		return EIO;
	}
	if (!write) {
		*value = (uint32_t)ucd->resp[20] << 24 | (uint32_t)ucd->resp[21] << 16 | (uint32_t)ucd->resp[22] << 8 | ucd->resp[23];
	}
	return 0;
}

static int
ufs_boot_lun(uint32_t *lun)
{
	return ufs_query_attr(false, 0x00, lun);
}

static int
sweet_ufs_slot(int target, bool successful, int *active_out)
{
	unsigned ps = 0;
	int error;
	const uint32_t blk = UFS_BLOCK;
	uint8_t *hdr = kalloc_data(blk, Z_WAITOK | Z_ZERO), *bhdr = kalloc_data(blk, Z_WAITOK | Z_ZERO);
	uint8_t *ents = kalloc_data(8 * blk, Z_WAITOK | Z_ZERO), *bents = kalloc_data(8 * blk, Z_WAITOK | Z_ZERO);
	uint8_t *check = kalloc_data(8 * blk, Z_WAITOK | Z_ZERO);

	if (!su.ready || !hdr || !bhdr || !ents || !bents || !check) {
		error = EAGAIN;
		goto out_free;
	}
	lck_mtx_lock(&sweet_ufs_lock);
	uint8_t saved = su.lun;
	error = ufs_lookup_part("boot_a", &ps);
	if (error) {
		goto out;
	}
	uint8_t lun = part_cache[ps].lun;
	su.lun = lun;
	struct sweet_slot_gpt g, bg;
	if ((error = ufs_read_into(1, 1, hdr)) != 0) {
		goto out;
	}
	if (sweet_slot_check_header(hdr, 1, &g) != 0) {
		error = EFTYPE;
		goto out;
	}
	uint32_t span = (g.count * 128 + blk - 1) / blk;
	if (span > 8 || g.entries_lba < 2 || g.alt_lba <= g.entries_lba + span) {
		error = EFTYPE;
		goto out;
	}
	if ((error = ufs_read_into(g.entries_lba, span, ents)) != 0 || (error = ufs_read_into(g.alt_lba, 1, bhdr)) != 0) {
		goto out;
	}
	if (sweet_slot_check_header(bhdr, g.alt_lba, &bg) != 0 || bg.entries_lba + span > g.alt_lba ||
	    bg.entries_lba <= g.entries_lba + span) {
		error = EFTYPE;
		goto out;
	}
	if ((error = ufs_read_into(bg.entries_lba, span, bents)) != 0) {
		goto out;
	}
	int now = sweet_slot_active(ents, g.count);
	sweet_slot_active_cached = now > 0 ? now : 0;
	if (active_out) {
		*active_out = sweet_slot_active_cached;
	}
	if (target == 0) {
		goto out;                       /* only asked which slot is active */
	}
	int r = sweet_slot_switch(hdr, ents, bhdr, bents, g.alt_lba, (char)target, successful);
	if (r != 0) {
		printf("SWEET SLOT: GPT of LUN %u refused (%d): nothing written\n", lun, r);
		error = EFTYPE;
		goto out;
	}
	printf("SWEET SLOT: LUN %u: slot %c -> %c (entries %llu+%u, backup %llu+%u, header %llu)\n", lun, now, target,
	    g.entries_lba, span, bg.entries_lba, span, g.alt_lba);
	/* 0306: the boot LUN first (the bootloader loads nothing while it and the GPT disagree), read back */
	uint32_t boot_lun_old = 0, boot_lun_new = target == 'a' ? 1 : 2, boot_lun_now = 0;
	if ((error = ufs_boot_lun(&boot_lun_old)) != 0) {
		printf("SWEET SLOT: boot LUN unreadable (%d): nothing written\n", error);
		goto out;
	}
	if (boot_lun_old != boot_lun_new) {
		if ((error = ufs_query_attr(true, 0x00, &boot_lun_new)) != 0 ||
		    (error = ufs_boot_lun(&boot_lun_now)) != 0 || boot_lun_now != boot_lun_new) {
			printf("SWEET SLOT: boot LUN %u -> %u failed (%d, reads %u): GPT not written\n", boot_lun_old,
			    boot_lun_new, error, boot_lun_now);
			if (boot_lun_now != boot_lun_old) {
				(void)ufs_query_attr(true, 0x00, &boot_lun_old);
			}
			error = error ? error : EIO;
			goto out;
		}
	}
	printf("SWEET SLOT: boot LUN %u -> %u\n", boot_lun_old, boot_lun_new);
	if ((error = ufs_write_from(lun, bg.entries_lba, span, bents)) != 0 ||
	    (error = ufs_write_from(lun, g.alt_lba, 1, bhdr)) != 0 ||
	    (error = ufs_write_from(lun, g.entries_lba, span, ents)) != 0 ||
	    (error = ufs_write_from(lun, 1, 1, hdr)) != 0) {
		printf("SWEET SLOT: write failed (%d)\n", error);
		goto out;
	}
	(void)ufs_sync_cache();
	if ((error = ufs_read_into(g.entries_lba, span, check)) == 0 && memcmp(check, ents, span * blk) != 0) {
		error = EIO;
	}
	if (!error && (error = ufs_read_into(1, 1, check)) == 0 && memcmp(check, hdr, blk) != 0) {
		error = EIO;
	}
	if (!error) {
		sweet_slot_active_cached = target;
		if (active_out) {
			*active_out = target;
		}
	}
	printf("SWEET SLOT: %s\n", error ? "read back DIFFERS" : "written and read back");
out:
	su.lun = saved;
	lck_mtx_unlock(&sweet_ufs_lock);
out_free:
	if (hdr) kfree_data(hdr, blk);
	if (bhdr) kfree_data(bhdr, blk);
	if (ents) kfree_data(ents, 8 * blk);
	if (bents) kfree_data(bents, 8 * blk);
	if (check) kfree_data(check, 8 * blk);
	return error;
}

extern int reboot_kernel(int howto, char *message);
extern void sweet_reboot_mode_set(int mode);

static int
sysctl_sweet_boot_android SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	int active = 0;
	(void)sweet_ufs_slot(0, false, &active);
	int v = active, changed = 0;
	int error = sysctl_io_number(req, v, sizeof(v), &v, &changed);
	if (error || !changed) {
		return error;
	}
	if (v != 1 && v != 2 && v != 3) {
		return EINVAL;
	}
	if (active != 'a' && active != 'b') {
		return EFTYPE;
	}
	error = sweet_ufs_slot(active == 'a' ? 'b' : 'a', false, NULL);
	if (!error && v == 1) {
		/* 0307: a restart to the system (the slot just made active), not the default bootloader (0221) */
		sweet_reboot_mode_set(0);
		(void)reboot_kernel(0, (char *)"sweet: to Android");
	}
	if (!error && v == 3) {
		/* 0311: to Android slot recovery (OrangeFox), where the installers apply Sapphire updates */
		sweet_reboot_mode_set(1);
		(void)reboot_kernel(0, (char *)"sweet: to recovery");
	}
	return error;
}
SYSCTL_PROC(_kern, OID_AUTO, sweet_boot_android, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_ANYBODY | CTLFLAG_LOCKED,
    0, 0, sysctl_sweet_boot_android, "I", "sapphire: 1 = make the other A/B slot (Android) active and reboot, 2 = without the reboot, 3 = and reboot into its recovery");

/*
 * sapphire 0306: iOS marks its slot good, as Android's update_verifier does: 90 s after the UFS driver is up
 * (iOS running), if the GPT's active slot is the one the boot LUN says the bootloader came from, boot_X is
 * given its 7 retries and marked successful -- the bootloader then stops counting them down, so iOS stays
 * the system booted (without it, the 7th boot would have fallen back to Android). Boot-arg sweetmarkgood=0
 * leaves it.
 */
static void
sweet_slot_mark_thread(__unused void *arg, __unused wait_result_t wr)
{
	extern void IOSleep(unsigned int);
	IOSleep(90 * 1000);
	unsigned ps = 0;
	uint32_t blun = 0;
	const uint32_t blk = UFS_BLOCK;
	uint8_t *hdr = kalloc_data(blk, Z_WAITOK | Z_ZERO), *bhdr = kalloc_data(blk, Z_WAITOK | Z_ZERO);
	uint8_t *ents = kalloc_data(8 * blk, Z_WAITOK | Z_ZERO), *bents = kalloc_data(8 * blk, Z_WAITOK | Z_ZERO);
	if (!hdr || !bhdr || !ents || !bents) {
		goto free;
	}
	lck_mtx_lock(&sweet_ufs_lock);
	uint8_t saved = su.lun;
	struct sweet_slot_gpt g, bg;
	int r = -1;
	if (ufs_lookup_part("boot_a", &ps) != 0) {
		goto out;
	}
	uint8_t lun = part_cache[ps].lun;
	su.lun = lun;
	if (ufs_boot_lun(&blun) != 0 || ufs_read_into(1, 1, hdr) != 0 || sweet_slot_check_header(hdr, 1, &g) != 0) {
		goto out;
	}
	uint32_t span = (g.count * 128 + blk - 1) / blk;
	if (span > 8 || ufs_read_into(g.entries_lba, span, ents) != 0 || ufs_read_into(g.alt_lba, 1, bhdr) != 0 ||
	    sweet_slot_check_header(bhdr, g.alt_lba, &bg) != 0 || bg.entries_lba + span > g.alt_lba ||
	    ufs_read_into(bg.entries_lba, span, bents) != 0) {
		goto out;
	}
	int act = sweet_slot_active(ents, g.count);
	if (act != (blun == 1 ? 'a' : blun == 2 ? 'b' : 0)) {
		printf("SWEET SLOT: not marking: active slot %c, boot LUN %u\n", act > 0 ? act : '?', blun);
		goto out;
	}
	r = sweet_slot_mark_good(hdr, ents, bhdr, bents, g.alt_lba, (char)act);
	if (r == 0) {
		int e = 0;
		if ((e = ufs_write_from(lun, bg.entries_lba, span, bents)) == 0 &&
		    (e = ufs_write_from(lun, g.alt_lba, 1, bhdr)) == 0 &&
		    (e = ufs_write_from(lun, g.entries_lba, span, ents)) == 0) {
			e = ufs_write_from(lun, 1, 1, hdr);
		}
		(void)ufs_sync_cache();
		printf("SWEET SLOT: slot %c marked good%s\n", act, e ? " -- WRITE FAILED" : "");
	} else {
		printf("SWEET SLOT: slot %c %s (%d)\n", act, r == SWEET_SLOT_ALREADY ? "already good" : "not marked", r);
	}
out:
	su.lun = saved;
	lck_mtx_unlock(&sweet_ufs_lock);
free:
	if (hdr) kfree_data(hdr, blk);
	if (bhdr) kfree_data(bhdr, blk);
	if (ents) kfree_data(ents, 8 * blk);
	if (bents) kfree_data(bents, 8 * blk);
}

void sweet_slot_mark_start(void);
void
sweet_slot_mark_start(void)
{
	uint32_t on = 1;
	thread_t t;
	(void)PE_parse_boot_argn("sweetmarkgood", &on, sizeof(on));
	if (on && su.ready && su.writable && kernel_thread_start(sweet_slot_mark_thread, NULL, &t) == KERN_SUCCESS) {
		thread_deallocate(t);
	}
}

static int
sysctl_sweet_boot_lun SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	uint32_t v = 0;
	int error = EAGAIN;
	if (su.ready) {
		lck_mtx_lock(&sweet_ufs_lock);
		error = ufs_boot_lun(&v);
		lck_mtx_unlock(&sweet_ufs_lock);
	}
	if (error) {
		return error;
	}
	int out = (int)v;
	return SYSCTL_OUT(req, &out, sizeof(out));
}
SYSCTL_PROC(_kern, OID_AUTO, sweet_boot_lun, CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_ANYBODY | CTLFLAG_LOCKED,
    0, 0, sysctl_sweet_boot_lun, "I", "sapphire: the UFS bBootLunEn attribute (1 = slot a's XBL, 2 = slot b's)");

/*
 * sapphire 0310: the inbox, read from iOS. The raw space of the root partition after its HFS volume (the
 * Windows installer and the Android app write there: updates, apps; installer/recovery/sapphire.sh,
 * android/dualboot IpaInbox.java) is read through this, never through /dev/rufsroot0. Read-only, root
 * only, and only inside [V, P - LOG): V = the volume size the installer gives the volume
 * ((P - LOG - INBOX) rounded down to 1 MiB), P = the partition, LOG = the console ring at its end.
 *
 *   kern.sweet_inbox: new = u64 offset from the partition start, old = up to 1 MiB from there
 *   (with no old buffer: the length of the inbox space)
 */
#define INBOX_LOG_BYTES   4198400ull      /* 1025 x 4 KiB */
#define INBOX_BYTES       134217728ull
#define INBOX_MIB         1048576ull

static int
sysctl_sweet_inbox SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	char want[64] = "userdata";
	uint64_t off = 0;
	unsigned slot = 0;
	int error;

	if (!kauth_cred_issuser(kauth_cred_get())) {
		return EPERM;
	}
	if (req->newptr == USER_ADDR_NULL || req->newlen != sizeof(off)) {
		return EINVAL;
	}
	error = SYSCTL_IN(req, &off, sizeof(off));
	if (error) {
		return error;
	}
	if (!su.ready) {
		return EAGAIN;
	}
	PE_parse_boot_argn("ufsroot", want, sizeof(want));
	lck_mtx_lock(&sweet_ufs_lock);
	error = ufs_lookup_part(want, &slot);
	uint64_t p = error ? 0 : part_cache[slot].nblocks * UFS_BLOCK;
	lck_mtx_unlock(&sweet_ufs_lock);
	if (error) {
		return error;
	}
	if (p < INBOX_LOG_BYTES + INBOX_BYTES + 64 * INBOX_MIB) {
		return ENOSPC;
	}
	uint64_t lo = (p - INBOX_LOG_BYTES - INBOX_BYTES) / INBOX_MIB * INBOX_MIB;
	uint64_t hi = p - INBOX_LOG_BYTES;
	if (req->oldptr == USER_ADDR_NULL) {
		req->oldidx = (size_t)(hi - lo);
		return 0;
	}
	uint64_t len = req->oldlen;
	if (len > BOUNCE_SIZE) {
		len = BOUNCE_SIZE;
	}
	if (off < lo || off >= hi) {
		return ERANGE;
	}
	if (len > hi - off) {
		len = hi - off;
	}
	void *buf = kalloc_data((vm_size_t)len, Z_WAITOK);
	if (buf == NULL) {
		return ENOMEM;
	}
	error = sweet_ufs_read_part(want, off, buf, len);
	if (!error) {
		error = SYSCTL_OUT(req, buf, (size_t)len);
	}
	kfree_data(buf, (vm_size_t)len);
	return error;
}
SYSCTL_PROC(_kern, OID_AUTO, sweet_inbox, CTLTYPE_OPAQUE | CTLFLAG_RW | CTLFLAG_LOCKED,
    0, 0, sysctl_sweet_inbox, "S", "sapphire: the inbox after the root volume (new: u64 offset; old: up to 1 MiB)");
