/*
 * topaz (SM6225): boot the modem DSP (MPSS). It runs the WCN3988 WLAN firmware, so this is
 * Wi-Fi step 1. Port of MakrSas/topaz-woa drivers/TopazModem/ModemPas.c + Smem.c, the way
 * Linux qcom_mdt_load() + qcom_q6v5_pas do it:
 *   TrustZone PAS init_image (.mdt metadata) -> mem_setup -> copy the .bNN segments into
 *   the carveout (0x4AB00000, 0x6900000) -> auth_and_reset -> wait for the modem's SMP2P
 *   "slave-kernel" READY bit (SMEM item 435) and answer its IPA power query (item 428).
 *
 * Boot-arg modem=1. Firmware from the root filesystem, /usr/share/firmware/topaz/modem/
 * (stock HyperOS 2.0.208 NON-HLOS: it must match this phone's EFS partitions).
 * Rules (WIFI_RISKS.md): buffers TZ/the modem see are below 0xF0000000 (own pool, not the
 * GPU's 0x90000000-0xD0000000), memory shared with the modem is write-combined (Normal NC:
 * no cache upkeep, unaligned access ok), the carveout is mapped only while copying, the
 * modem must start cold, polling only (no interrupts), no EFS writes.
 */
#include <pexpert/arm64/board_config.h>

#if defined(SWEET)
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <kern/thread.h>
#include <pexpert/pexpert.h>
#include <libkern/crc.h>
#include <pexpert/device_tree.h>
#include <sys/param.h>
#include <sys/errno.h>

extern "C" vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);

#define LOG(fmt, ...)       IOLog("TopazModem: " fmt "\n", ##__VA_ARGS__)

#define PAS_ID_MODEM        4
#define SCM_FN(svc, cmd)    (0x42000000u | ((uint32_t)(svc) << 8) | (uint32_t)(cmd))
#define SCM_SVC_PIL         0x02
#define PIL_INIT_IMAGE      0x01
#define PIL_MEM_SETUP       0x02
#define PIL_AUTH_RESET      0x05
#define PIL_SHUTDOWN        0x06
#define PIL_IS_SUPPORTED    0x07
#define SCM_ARG_RW          2

#define PT_LOAD             1
#define MDT_TYPE_MASK       (7u << 24)
#define MDT_TYPE_HASH       (2u << 24)
#define MDT_RELOCATABLE     (1u << 27)

/* middle DDR bank, RAM the kernel does not manage; below 0xF0000000 for TZ/the modem */
#define MODEM_POOL_BASE     0xE0000000ull
#define MODEM_POOL_SIZE     0x01000000ull       /* 16 MB: metadata, SCM args, rmtfs, CE rings, WMI memory */

#define SMEM_PA             0x46000000ull
#define SMEM_SIZE           0x00200000u
#define SMEM_GLOBAL_HOST    0xFFFE
#define TCSR_MUTEX_PA       0x00340000ull       /* qcom,tcsr-mutex, stride 0x1000 */
#define SMEM_HWLOCK         3
#define HWLOCK_APPS_ID      1
#define APCS_PA             0x0F111000ull
#define APCS_IPC            8                   /* apcs_glb mailbox */
#define MPSS_SMP2P_BIT      14

#define ITEM_SMP2P_OUT      428                 /* apps -> modem */
#define ITEM_SMP2P_IN       435                 /* modem -> apps */
#define ITEM_MSS_CRASH      421
#define SMP2P_MAGIC         0x504D5324u         /* "$SMP" */
#define SMP2P_MAX_ENTRY     16

struct elf32_ehdr {
	uint8_t  e_ident[16];
	uint16_t e_type, e_machine;
	uint32_t e_version, e_entry, e_phoff, e_shoff, e_flags;
	uint16_t e_ehsize, e_phentsize, e_phnum, e_shentsize, e_shnum, e_shstrndx;
} __attribute__((packed));
struct elf32_phdr {
	uint32_t p_type, p_offset, p_vaddr, p_paddr, p_filesz, p_memsz, p_flags, p_align;
} __attribute__((packed));

struct smem_pt_entry { uint32_t offset, size, flags; uint16_t host0, host1; uint32_t cacheline, rsvd[7]; } __attribute__((packed));
struct smem_pt { uint8_t magic[4]; uint32_t version, num_entries, rsvd[5]; } __attribute__((packed));
struct smem_part_hdr { uint8_t magic[4]; uint16_t host0, host1; uint32_t size, free_uncached, free_cached, rsvd[3]; } __attribute__((packed));
struct smem_priv_entry { uint16_t canary, item; uint32_t size; uint16_t pad_data, pad_hdr; uint32_t rsvd; } __attribute__((packed));
struct smp2p_hdr { uint32_t magic; uint8_t version, features[3]; uint16_t local_pid, remote_pid, total, valid; uint32_t flags; } __attribute__((packed));
struct smp2p_entry { char name[16]; uint32_t value; } __attribute__((packed));

static uint8_t *smem;                           /* write-combined */
static volatile uint8_t *apcs, *tcsr_mutex;     /* device */
static uint8_t *pool;
static uint64_t pool_used;

static inline void mb(void) { __asm__ volatile ("dsb sy" ::: "memory"); }

/* ---------------------------------------------------------------- mappings */

/* Large or RAM regions: an IOMemoryDescriptor mapping (ml_io_map space is small). */
static IOMemoryMap *
map_phys(uint64_t pa, uint64_t size, bool wc)
{
	IOMemoryDescriptor *md = IOMemoryDescriptor::withPhysicalAddress(pa, size, kIODirectionInOut);
	IOMemoryMap *m;

	if (md == NULL) {
		return NULL;
	}
	m = md->map(wc ? kIOMapWriteCombineCache : kIOMapInhibitCache);
	md->release();
	return m;
}

static void *
pool_alloc(uint64_t size, uint64_t *pa)
{
	size = (size + 0xfff) & ~0xfffull;
	/* the last 256 KB: the ADSP's audio buffers; 64 KB under them: FastRPC's (0265) */
	if (pool == NULL || pool_used + size > MODEM_POOL_SIZE - 0x50000) {
		return NULL;
	}
	*pa = MODEM_POOL_BASE + pool_used;
	void *va = pool + pool_used;
	pool_used += size;
	bzero(va, size);                            /* Normal NC: DC ZVA is fine */
	return va;
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

static unsigned scm_resumes;

static uint64_t
scm(uint32_t fn, uint64_t arginfo, uint64_t a, uint64_t b, uint64_t c, uint64_t *res1)
{
	uint64_t args[8] = { fn, arginfo, a, b, c, 0, 0, 0 };
	unsigned n;

	smc(args);
	/* long calls return INTERRUPTED (1): resume with x0 = 1 and the session from x6 */
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

/* ---------------------------------------------------------------- firmware files */

/*
 * The firmware is read from the phone's stock modem partition (boot-arg modem_part=, default
 * modem_a), read-only through sweet_ufs (bsd/dev/sweet_ufs.c): nothing has to be put on
 * userdata. Where each file lies in that FAT16 volume was worked out on the host from the
 * same NON-HLOS.bin (HyperOS 2.0.208; every file is one contiguous run); the CRC32 tells
 * whether the phone's partition is that image. sapphire (Redmi Note 13 4G, same SoC and
 * carveouts) has the same file set at other places; fw_pick() chooses the table whose
 * modem.mdt CRC matches the partition.
 */
extern "C" int sweet_ufs_read_part(const char *want, uint64_t off, void *dst, uint64_t len);

static char fw_part[32] = "modem_a";
struct fw_file { const char *name; uint32_t off, size, crc; };
static const struct fw_file fw_topaz[] = {
	{ "modem.b00", 0x1e1d000, 0x414, 0xaacde80c },
	{ "modem.b01", 0x1e21000, 0x1f78, 0x80555ebe },
	{ "modem.b02", 0x1e25000, 0x2930, 0x0858b3a0 },
	{ "modem.b03", 0x1e29000, 0x2518, 0x93a2467e },
	{ "modem.b04", 0x1e2d000, 0x4a3ac, 0xbf85c65c },
	{ "modem.b05", 0x1e79000, 0x119d8, 0xf3637bd9 },
	{ "modem.b06", 0x1e8d000, 0x100c, 0x89d24f27 },
	{ "modem.b07", 0x1e91000, 0x90b4, 0xefeddcd9 },
	{ "modem.b08", 0x1e9d000, 0x4f054, 0xeb2e2bdb },
	{ "modem.b09", 0x1eed000, 0x143ee8, 0xc8344d45 },
	{ "modem.b10", 0x2031000, 0x40000, 0x7e9db699 },
	{ "modem.b11", 0x2071000, 0x540, 0x0030dad4 },
	{ "modem.b12", 0x2075000, 0xe2210, 0x7d37ba08 },
	{ "modem.b14", 0x2159000, 0x2d, 0x7a4002c7 },
	{ "modem.b15", 0x215d000, 0x7c, 0x959b6a58 },
	{ "modem.b16", 0x2161000, 0x9c1a0, 0xd902be24 },
	{ "modem.b17", 0x2201000, 0x61110, 0xd2182ca7 },
	{ "modem.b18", 0x2265000, 0x2d29a9, 0x672e3b04 },
	{ "modem.b19", 0x2539000, 0x2d94cc, 0x916bcc13 },
	{ "modem.b22", 0x2815000, 0x10ec0, 0x444b4456 },
	{ "modem.b23", 0x2829000, 0x1722040, 0x3c3bc2a3 },
	{ "modem.b24", 0x3f4d000, 0xa07900, 0x4001a993 },
	{ "modem.b25", 0x4955000, 0x77a45, 0xda7bfa33 },
	{ "modem.b26", 0x49cd000, 0x15104, 0x33437cee },
	{ "modem.b27", 0x49e5000, 0x1378, 0x161303ae },
	{ "modem.b28", 0x49e9000, 0xb0, 0x1e475201 },
	{ "modem.b29", 0x49ed000, 0x7d52c, 0x7d6f01a3 },
	{ "modem.mdt", 0x4a6d000, 0x238c, 0x8742d22b },
};
static const struct fw_file fw_sapphire[] = {
	{ "modem.b00", 0x24a9000, 0x414, 0xb65bc268 },
	{ "modem.b01", 0x24ad000, 0x1f78, 0x3095d3e5 },
	{ "modem.b02", 0x24b1000, 0x2930, 0x43a8e621 },
	{ "modem.b03", 0x24b5000, 0x2518, 0x93a2467e },
	{ "modem.b04", 0x24b9000, 0x4a3ac, 0x1824a643 },
	{ "modem.b05", 0x2505000, 0x119d8, 0x9b98d3c3 },
	{ "modem.b06", 0x2519000, 0x100c, 0x4cdb81db },
	{ "modem.b07", 0x251d000, 0x90b4, 0x45aafd29 },
	{ "modem.b08", 0x2529000, 0x4f054, 0xa09c4e3a },
	{ "modem.b09", 0x2579000, 0x143ee8, 0xf56ce8eb },
	{ "modem.b10", 0x26bd000, 0x40000, 0xa1d37576 },
	{ "modem.b11", 0x26fd000, 0x540, 0x1055fb61 },
	{ "modem.b12", 0x2701000, 0xe2210, 0x66ec10c8 },
	{ "modem.b14", 0x27e5000, 0x2d, 0x7a4002c7 },
	{ "modem.b15", 0x27e9000, 0x7c, 0x959b6a58 },
	{ "modem.b16", 0x27ed000, 0x9c1a0, 0xd902be24 },
	{ "modem.b17", 0x288d000, 0x61110, 0xd2182ca7 },
	{ "modem.b18", 0x28f1000, 0x2d4079, 0x04103884 },
	{ "modem.b19", 0x2bc9000, 0x2da98c, 0x8f13126c },
	{ "modem.b22", 0x2ea5000, 0x10ec0, 0x8c07d162 },
	{ "modem.b23", 0x2eb9000, 0x1722040, 0x12610ed2 },
	{ "modem.b24", 0x45dd000, 0xa07900, 0x489ef97e },
	{ "modem.b25", 0x4fe5000, 0x778fb, 0xbcb83811 },
	{ "modem.b26", 0x505d000, 0x15104, 0x33437cee },
	{ "modem.b27", 0x5075000, 0x1378, 0x161303ae },
	{ "modem.b28", 0x5079000, 0xb0, 0x1e475201 },
	{ "modem.b29", 0x507d000, 0x7d52c, 0xc2f72f9c },
	{ "modem.mdt", 0x50fd000, 0x238c, 0x8d38e54b },
};
static const struct { const char *phone; const struct fw_file *files; unsigned n; } fw_tables[] = {
	{ "topaz", fw_topaz, sizeof(fw_topaz) / sizeof(fw_topaz[0]) },
	{ "sapphire", fw_sapphire, sizeof(fw_sapphire) / sizeof(fw_sapphire[0]) },
};
static const struct fw_file *fw_files = fw_topaz;
static unsigned fw_phone;                   /* index into fw_tables */
static unsigned fw_count = sizeof(fw_topaz) / sizeof(fw_topaz[0]);
static bool fw_picked;                      /* fw_pick() ran (0248: SweetQsee.cpp may run it) */
#define FW_FAT 2                            /* fw_phone: no table matched, the files were found in the FAT (0308) */
static bool fw_fat_build(void);
static int64_t fw_fat_size(const char *name);
static int64_t fw_fat_read(const char *name, void *dst, uint64_t max);

static int
fw_find(const char *name)
{
	for (unsigned i = 0; i < fw_count; i++) {
		if (strcmp(fw_files[i].name, name) == 0) {
			return (int)i;
		}
	}
	return -1;
}

static int64_t
fw_size(const char *name)
{
	if (fw_phone == FW_FAT) {
		return fw_fat_size(name);
	}
	int i = fw_find(name);
	return i < 0 ? -1 : (int64_t)fw_files[i].size;
}

/* Use the table whose modem.mdt (the last entry) matches the partition. */
static void
fw_pick(void)
{
	const uint32_t max = 0x4000;
	uint8_t *mdt = (uint8_t *)IOMalloc(max);

	if (mdt == NULL) {
		return;
	}
	fw_picked = true;
	for (unsigned t = 0; t < sizeof(fw_tables) / sizeof(fw_tables[0]); t++) {
		const struct fw_file *m = &fw_tables[t].files[fw_tables[t].n - 1];
		if (m->size > max || sweet_ufs_read_part(fw_part, m->off, mdt, m->size) != 0) {
			continue;
		}
		if (crc32(0, mdt, m->size) == m->crc) {
			fw_files = fw_tables[t].files;
			fw_count = fw_tables[t].n;
			fw_phone = t;
			LOG("firmware: %s matches the %s table", fw_part, fw_tables[t].phone);
			IOFree(mdt, max);
			return;
		}
	}
	IOFree(mdt, max);
	if (fw_fat_build()) {
		fw_phone = FW_FAT;
		LOG("firmware: %s matches no table: its files are read by name from its FAT", fw_part);
		return;
	}
	LOG("firmware: %s matches no table, trying the topaz one", fw_part);
}

/* Read the whole file name into dst (max bytes at most). Returns bytes read or -1. */
static int64_t
fw_read(const char *name, void *dst, uint64_t max)
{
	if (fw_phone == FW_FAT) {
		return fw_fat_read(name, dst, max);
	}
	int i = fw_find(name), err;

	if (i < 0 || fw_files[i].size > max) {
		LOG("%s: not in the table or too big (%lld)", name, fw_size(name));
		return -1;
	}
	err = sweet_ufs_read_part(fw_part, fw_files[i].off, dst, fw_files[i].size);
	if (err != 0) {
		LOG("%s: read from %s failed: %d", name, fw_part, err);
		return -1;
	}
	uint32_t crc = crc32(0, dst, fw_files[i].size);
	if (crc != fw_files[i].crc) {
		/* TrustZone checks the signature anyway: say so and go on */
		LOG("%s: crc32 %08x, expected %08x: %s is not the image this was made for", name, crc, fw_files[i].crc, fw_part);
	}
	return fw_files[i].size;
}

/* ---------------------------------------------------------------- SMEM */

static struct smem_part_hdr *
smem_partition(uint16_t a, uint16_t b)
{
	struct smem_pt *pt = (struct smem_pt *)(smem + SMEM_SIZE - 0x1000);
	struct smem_pt_entry *e = (struct smem_pt_entry *)(pt + 1);

	if (memcmp(pt->magic, "$TOC", 4) != 0 || pt->num_entries > 64) {
		return NULL;
	}
	for (uint32_t i = 0; i < pt->num_entries; i++) {
		if (e[i].offset != 0 && e[i].size != 0 && e[i].offset < SMEM_SIZE &&
		    ((e[i].host0 == a && e[i].host1 == b) || (e[i].host0 == b && e[i].host1 == a))) {
			return (struct smem_part_hdr *)(smem + e[i].offset);
		}
	}
	return NULL;
}

static void *
smem_get(struct smem_part_hdr *p, uint16_t item, uint32_t *size)
{
	uint8_t *e, *end;

	if (p == NULL || memcmp(p->magic, "$PRT", 4) != 0 || p->free_uncached > p->size) {
		return NULL;
	}
	e = (uint8_t *)p + sizeof(*p);
	end = (uint8_t *)p + p->free_uncached;
	while (e + sizeof(struct smem_priv_entry) <= end) {
		struct smem_priv_entry *h = (struct smem_priv_entry *)e;
		if (h->canary != 0xA5A5) {
			return NULL;
		}
		if (h->item == item) {
			if (size != NULL) {
				*size = h->size - h->pad_data;
			}
			return e + sizeof(*h) + h->pad_hdr;
		}
		e += sizeof(*h) + h->pad_hdr + h->size;
	}
	return NULL;
}

static bool
smem_lock(void)
{
	volatile uint32_t *reg = (volatile uint32_t *)(tcsr_mutex + SMEM_HWLOCK * 0x1000);

	for (int n = 0; n < 100000; n++) {
		*reg = HWLOCK_APPS_ID;
		mb();
		if (*reg == HWLOCK_APPS_ID) {
			return true;
		}
		IODelay(10);
	}
	LOG("SMEM hwlock busy: owner %u", *reg);
	return false;
}

static void
smem_unlock(void)
{
	mb();
	*(volatile uint32_t *)(tcsr_mutex + SMEM_HWLOCK * 0x1000) = 0;
	mb();
}

/* qcom_smem_alloc_private(): append an uncached entry under hwlock 3 */
static void *
smem_alloc(struct smem_part_hdr *p, uint16_t item, uint32_t size)
{
	uint32_t aligned = (size + 7) & ~7u, need = sizeof(struct smem_priv_entry) + aligned;
	void *v = smem_get(p, item, NULL);

	if (v != NULL || p == NULL || !smem_lock()) {
		return v;
	}
	v = smem_get(p, item, NULL);
	if (v == NULL && p->free_uncached + need <= p->free_cached) {
		struct smem_priv_entry *h = (struct smem_priv_entry *)((uint8_t *)p + p->free_uncached);
		h->canary = 0xA5A5;
		h->item = item;
		h->size = aligned;
		h->pad_data = (uint16_t)(aligned - size);
		h->pad_hdr = 0;
		h->rsvd = 0;
		v = h + 1;
		bzero(v, aligned);
		mb();
		p->free_uncached += need;
		mb();
	} else if (v == NULL) {
		LOG("SMEM partition full: %x..%x need %x", p->free_uncached, p->free_cached, need);
	}
	smem_unlock();
	return v;
}

/* ---------------------------------------------------------------- SMP2P */

static void
smp2p_kick(void)
{
	mb();
	*(volatile uint32_t *)(apcs + APCS_IPC) = 1u << MPSS_SMP2P_BIT;
	mb();
}

static struct smp2p_hdr *
smp2p_create_out(struct smem_part_hdr *p)
{
	struct smp2p_hdr *o = (struct smp2p_hdr *)smem_alloc(p, ITEM_SMP2P_OUT,
	    sizeof(struct smp2p_hdr) + SMP2P_MAX_ENTRY * sizeof(struct smp2p_entry));

	if (o == NULL || o->version != 0) {
		return o;
	}
	o->magic = SMP2P_MAGIC;
	o->local_pid = 0;
	o->remote_pid = 1;
	o->total = SMP2P_MAX_ENTRY;
	o->valid = 0;
	o->features[0] = 1;                         /* SSR_ACK */
	mb();
	o->version = 1;                             /* validates the item */
	smp2p_kick();
	return o;
}

static volatile uint32_t *
smp2p_find(struct smp2p_hdr *h, const char *name)
{
	struct smp2p_entry *e = (struct smp2p_entry *)(h + 1);

	for (uint32_t i = 0; i < h->valid && i < SMP2P_MAX_ENTRY; i++) {
		if (strncmp(e[i].name, name, 16) == 0) {
			return &e[i].value;
		}
	}
	return NULL;
}

static volatile uint32_t *
smp2p_add_out(struct smp2p_hdr *o, const char *name, uint32_t value)
{
	struct smp2p_entry *e = (struct smp2p_entry *)(o + 1);
	volatile uint32_t *v = smp2p_find(o, name);

	if (v != NULL || o->valid >= o->total || o->valid >= SMP2P_MAX_ENTRY) {
		return v;
	}
	bzero(e[o->valid].name, sizeof(e[o->valid].name));
	strlcpy(e[o->valid].name, name, sizeof(e[o->valid].name));
	e[o->valid].value = value;
	mb();
	o->valid++;
	smp2p_kick();
	return &e[o->valid - 1].value;
}

static bool svc_rx(uint32_t src_node, uint32_t src_port, uint32_t dst_port, const uint8_t *data, uint32_t len);
static void svc_announce(void);
static bool adsp_qrtr_send(uint32_t dst_node, const uint8_t *pkt, uint32_t len);
static uint32_t adsp_qrtr_version(uint32_t dst_node);
static uint32_t adsp_node(void);
static void svc_summary(void);
/* sapphire 0220: /dev/redwood-qmi (bsd/dev/sweet_qmi_dev.c), QMI for userland */
extern "C" int sweet_qmi_deliver(uint32_t dst_port, uint32_t node, uint32_t port, const uint8_t *data, uint32_t len);
extern "C" void sweet_qmi_pump(void (*send)(uint32_t src_port, uint32_t node, uint32_t port, const void *data, uint32_t len, int ctrl));
extern "C" void sweet_qmi_dev_init(void);
static void qmi_user_send(uint32_t src_port, uint32_t node, uint32_t port, const void *data, uint32_t len, int ctrl);
static void wlfw_arrive(uint32_t node, uint32_t port);
static void ipa_qmi_arrive(uint32_t node, uint32_t port);
static int ipa_stats(char *buf, size_t len);
static bool wifi_enabled(void);
static bool ce_start(void);
static void radio_poll(unsigned ms);
static void w_rx_native(const uint8_t *f, uint32_t len);
static void htt_tx_compl(const uint8_t *p, uint32_t len);
static void w_assoc_mgmt_rx(const uint8_t *f, uint32_t len);
static void w_join_event(uint32_t id, const uint8_t *t, uint32_t len);
extern "C" void read_random(void *buffer, unsigned int numBytes);

/* ---------------------------------------------------------------- GLINK over SMEM */

/*
 * After READY the modem talks GLINK through two SMEM FIFOs (items 478 descriptor, 479 our TX,
 * 480 its TX), doorbell APCS bit 12. One channel, "IPCRTR", carries QRTR. Port of
 * topaz-woa drivers/TopazModem/Glink.c, polled from the watch thread (no interrupts).
 */
#define SMEM_GLINK_DESC     478
#define SMEM_GLINK_FIFO_TX  479
#define SMEM_GLINK_FIFO_RX  480
#define GLINK_TX_SIZE       0x4000u
#define GLINK_IRQ_BIT       12
#define GLINK_FEATURES      1                   /* INTENT_REUSE */

#define CMD_VERSION         0
#define CMD_VERSION_ACK     1
#define CMD_OPEN            2
#define CMD_CLOSE           3
#define CMD_OPEN_ACK        4
#define CMD_INTENT          5
#define CMD_RX_DONE         6
#define CMD_RX_INTENT_REQ   7
#define CMD_RX_INTENT_REQ_ACK 8
#define CMD_TX_DATA         9
#define CMD_CLOSE_ACK       11
#define CMD_TX_DATA_CONT    12
#define CMD_READ_NOTIF      13
#define CMD_RX_DONE_W_REUSE 14
#define CMD_SIGNALS         15

#define CHAN_NAME           "IPCRTR"
#define CHAN_LCID           1
#define OUR_INTENTS         8
#define OUR_INTENT_SIZE     0x4400u

#define QRTR_NODE_APPS      1
#define QRTR_PORT_CTRL      0xFFFFFFFEu
#define QRTR_NODE_BCAST     0xFFFFFFFFu
#define QRTR_TYPE_DATA      1
#define QRTR_TYPE_HELLO     2
#define QRTR_TYPE_BYE       3
#define QRTR_TYPE_NEW_SERVER 4
#define QRTR_TYPE_DEL_SERVER 5
#define QRTR_TYPE_DEL_CLIENT 6
#define QRTR_TYPE_RESUME_TX 7
#define QMI_SVC_WLFW        0x45

struct glink_msg { uint16_t cmd, p1; uint32_t p2; } __attribute__((packed));
struct glink_data_hdr { struct glink_msg m; uint32_t chunk, left; } __attribute__((packed));
struct qrtr_hdr_v1 { uint32_t version, type, src_node, src_port, confirm_rx, size, dst_node, dst_port; } __attribute__((packed));
struct qrtr_hdr_v2 { uint8_t version, type, flags, optlen; uint32_t size; uint16_t src_node, src_port, dst_node, dst_port; } __attribute__((packed));
struct qrtr_ctrl { uint32_t cmd, a, b, c, d; } __attribute__((packed));

static volatile uint32_t *g_desc;               /* [0] tx tail [1] tx head [2] rx tail [3] rx head */
static uint8_t *g_tx, *g_rx;
static uint32_t g_rx_len, g_features = GLINK_FEATURES, g_rcid, g_next_intent = 1;
static bool g_remote_open, g_open_sent, g_open_acked, g_intents_sent, g_intent_req_sent;
static struct { uint32_t id, size; bool used; } g_ri[256];
static unsigned g_ri_count;
static uint8_t g_asm[OUR_INTENT_SIZE];
static uint32_t g_asm_len;
static uint32_t q_ver = 1, q_modem_node = QRTR_NODE_BCAST;
static bool q_hello_sent, q_hello_rx, q_announced;
static struct { uint32_t svc, inst, node, port; } q_srv[64];
static unsigned q_srv_count, g_rx_msgs, g_tx_msgs, q_resumes;
static unsigned g_ms;                           /* watch time, for the log */
/* 0223: where the modem thread is, for kern.sweet_modem_stats when it stops (it did, under CommCenter) */
static const char *volatile g_where = "start";
static volatile unsigned g_loops;
#define WHERE(s) (g_where = (s))

static uint32_t
rx_avail(void)
{
	uint32_t h = g_desc[3], t = g_desc[2];
	return h >= t ? h - t : g_rx_len - t + h;
}

static void
rx_peek(uint32_t off, void *dst, uint32_t n)
{
	uint32_t t = g_desc[2] + off;
	if (t >= g_rx_len) {
		t -= g_rx_len;
	}
	uint32_t n1 = MIN(n, g_rx_len - t);
	memcpy(dst, g_rx + t, n1);
	if (n > n1) {
		memcpy((uint8_t *)dst + n1, g_rx, n - n1);
	}
}

static void
rx_advance(uint32_t n)
{
	uint32_t t = g_desc[2] + n;
	if (t >= g_rx_len) {
		t -= g_rx_len;
	}
	mb();
	g_desc[2] = t;
}

static uint32_t
tx_put(uint32_t head, const void *src, uint32_t n)
{
	uint32_t n1 = MIN(n, GLINK_TX_SIZE - head);
	memcpy(g_tx + head, src, n1);
	if (n > n1) {
		memcpy(g_tx, (const uint8_t *)src + n1, n - n1);
	}
	head += n;
	return head >= GLINK_TX_SIZE ? head - GLINK_TX_SIZE : head;
}

static bool
tx_write(const void *hdr, uint32_t hlen, const void *data, uint32_t dlen)
{
	uint32_t head = 0, tail = 0, avail = 0;
	int n;

	for (n = 0; n < 1000; n++) {
		head = g_desc[1];
		tail = g_desc[0];
		avail = tail <= head ? GLINK_TX_SIZE - head + tail : tail - head;
		avail = avail < 8 ? 0 : avail - 8;
		if (avail >= ((hlen + dlen + 7) & ~7u)) {
			break;
		}
		IODelay(100);
	}
	if (n == 1000) {
		LOG("glink TX FIFO full (head %x tail %x)", head, tail);
		return false;
	}
	head = tx_put(head, hdr, hlen);
	if (dlen != 0) {
		head = tx_put(head, data, dlen);
	}
	head = (head + 7) & ~7u;
	if (head >= GLINK_TX_SIZE) {
		head -= GLINK_TX_SIZE;
	}
	mb();
	g_desc[1] = head;
	mb();
	*(volatile uint32_t *)(apcs + APCS_IPC) = 1u << GLINK_IRQ_BIT;
	mb();
	g_tx_msgs++;
	return true;
}

static void
send_cmd(uint16_t cmd, uint16_t p1, uint32_t p2)
{
	struct glink_msg m = { cmd, p1, p2 };
	tx_write(&m, sizeof(m), NULL, 0);
}

static void
send_open(void)
{
	struct { struct glink_msg m; char name[8]; } __attribute__((packed)) req = {};
	req.m.cmd = CMD_OPEN;
	req.m.p1 = CHAN_LCID;
	req.m.p2 = sizeof(CHAN_NAME);
	memcpy(req.name, CHAN_NAME, sizeof(CHAN_NAME));
	tx_write(&req, sizeof(req), NULL, 0);
	g_open_sent = true;
}

static void
advertise_intents(uint32_t count, uint32_t size)
{
	struct { struct glink_msg m; uint32_t pair[2 * OUR_INTENTS]; } __attribute__((packed)) m = {};
	m.m.cmd = CMD_INTENT;
	m.m.p1 = CHAN_LCID;
	m.m.p2 = count;
	for (uint32_t i = 0; i < count && i < OUR_INTENTS; i++) {
		m.pair[2 * i] = size;
		m.pair[2 * i + 1] = g_next_intent++;
	}
	tx_write(&m, sizeof(struct glink_msg) + 8 * count, NULL, 0);
}

static bool
chan_up(void)
{
	return g_remote_open && g_open_acked;
}

static bool
glink_try_send(const void *data, uint32_t len)
{
	unsigned i;
	for (i = 0; i < g_ri_count; i++) {
		if (!g_ri[i].used && g_ri[i].size >= len) {
			break;
		}
	}
	if (i == g_ri_count) {
		if (!g_intent_req_sent) {
			send_cmd(CMD_RX_INTENT_REQ, CHAN_LCID, len);
			g_intent_req_sent = true;
		}
		return false;
	}
	struct glink_data_hdr h = { { CMD_TX_DATA, CHAN_LCID, g_ri[i].id }, len, 0 };
	g_ri[i].used = true;
	return tx_write(&h, sizeof(h), data, len);
}

/*
 * packets wait here until the modem has a free RX intent for them. 0222: 64 was too few for
 * Apple's CommCenter (hundreds of requests and RESUME_TX at once): the queue overflowed, a dropped
 * RESUME_TX left the modem's QRTR waiting forever and no QMI answer came any more.
 */
#define TXQ_LEN 1024
static struct { void *p; uint32_t len; } g_txq[TXQ_LEN];
static unsigned g_txq_head, g_txq_tail, g_txq_drops;

static void
txq_pump(void)
{
	while (g_txq_head != g_txq_tail &&
	    glink_try_send(g_txq[g_txq_head % TXQ_LEN].p, g_txq[g_txq_head % TXQ_LEN].len)) {
		IOFree(g_txq[g_txq_head % TXQ_LEN].p, g_txq[g_txq_head % TXQ_LEN].len);
		g_txq_head++;
	}
}

static void
glink_send(const void *data, uint32_t len)
{
	void *p;
	if (g_txq_tail - g_txq_head >= TXQ_LEN || (p = IOMalloc(len)) == NULL) {
		if (g_txq_drops++ < 8) {
			LOG("t=%u ms: glink: TX queue full (%u), packet of %u bytes dropped", g_ms, g_txq_tail - g_txq_head, len);
		}
		return;
	}
	memcpy(p, data, len);
	g_txq[g_txq_tail % TXQ_LEN].p = p;
	g_txq[g_txq_tail % TXQ_LEN].len = len;
	g_txq_tail++;
	txq_pump();
}

/* ---------------------------------------------------------------- QRTR */

static void
qrtr_send(uint32_t type, uint32_t src_port, uint32_t dst_node, uint32_t dst_port, const void *payload, uint32_t len)
{
	uint32_t cap = (uint32_t)sizeof(struct qrtr_hdr_v1) + len + 4;
	uint8_t *buf = (uint8_t *)IOMalloc(cap);
	uint32_t n;

	if (buf == NULL) {
		return;
	}
	bzero(buf, cap);
	if (adsp_qrtr_version(dst_node) == 2) {
		struct qrtr_hdr_v2 *h = (struct qrtr_hdr_v2 *)buf;
		h->version = 2;
		h->type = (uint8_t)type;
		h->flags = type == QRTR_TYPE_DATA ? 1 : 0;
		h->size = len;
		h->src_node = QRTR_NODE_APPS;
		h->src_port = (uint16_t)src_port;
		h->dst_node = (uint16_t)dst_node;
		h->dst_port = (uint16_t)dst_port;
		n = sizeof(*h);
	} else {
		struct qrtr_hdr_v1 *h = (struct qrtr_hdr_v1 *)buf;
		h->version = 1;
		h->type = type;
		h->src_node = QRTR_NODE_APPS;
		h->src_port = src_port;
		h->confirm_rx = type == QRTR_TYPE_DATA ? 1 : 0;   /* flow control: ask for RESUME_TX */
		h->size = len;
		h->dst_node = dst_node;
		h->dst_port = dst_port;
		n = sizeof(*h);
	}
	memcpy(buf + n, payload, len);
	if (!adsp_qrtr_send(dst_node, buf, (n + len + 3) & ~3u)) {
		glink_send(buf, (n + len + 3) & ~3u);  /* the modem's node (the ADSP's has its own edge) */
	}
	IOFree(buf, cap);
}

static const char *
svc_name(uint32_t svc)
{
	switch (svc) {
	case 0x45: return "WLFW";
	case 0x0E: return "rmtfs";
	case 0x40: return "servreg-loc";
	case 0x42: return "servreg-notif";
	case 0x2B: return "ssctl";
	case 0x31: return "ipa";
	case 0x01: return "wds";
	case 0x02: return "dms";
	case 0x03: return "nas";
	case 0x0B: return "uim";
	case 0x1A: return "wda";
	default:   return "";
	}
}

static void
qrtr_rx(const uint8_t *p, uint32_t len)
{
	uint32_t type, src_node, src_port, dst_port, size, hlen, confirm;

	if (len >= sizeof(struct qrtr_hdr_v1) && *(const uint32_t *)p == 1) {
		const struct qrtr_hdr_v1 *h = (const struct qrtr_hdr_v1 *)p;
		type = h->type; src_node = h->src_node; src_port = h->src_port;
		dst_port = h->dst_port; size = h->size; hlen = sizeof(*h); confirm = h->confirm_rx;
		q_ver = 1;
	} else if (len >= sizeof(struct qrtr_hdr_v2) && p[0] == 2) {
		const struct qrtr_hdr_v2 *h = (const struct qrtr_hdr_v2 *)p;
		type = h->type; src_node = h->src_node; src_port = h->src_port;
		dst_port = h->dst_port; size = h->size; hlen = sizeof(*h) + h->optlen * 4;
		src_port = src_port == 0xFFFE ? QRTR_PORT_CTRL : src_port;
		dst_port = dst_port == 0xFFFE ? QRTR_PORT_CTRL : dst_port;
		confirm = h->flags & 1;
		q_ver = 2;
	} else {
		LOG("t=%u ms: qrtr: bad packet, %u bytes", g_ms, len);
		return;
	}
	const struct qrtr_ctrl *c = (const struct qrtr_ctrl *)(p + hlen);

	switch (type) {
	case QRTR_TYPE_HELLO:
		q_modem_node = src_node;
		q_hello_rx = true;
		q_hello_sent = false;                   /* (re)answer with our HELLO */
		LOG("t=%u ms: qrtr v%u HELLO from node %u", g_ms, q_ver, src_node);
		break;
	case QRTR_TYPE_NEW_SERVER:
		if (q_srv_count < sizeof(q_srv) / sizeof(q_srv[0])) {
			q_srv[q_srv_count].svc = c->a;
			q_srv[q_srv_count].inst = c->b;
			q_srv[q_srv_count].node = c->c;
			q_srv[q_srv_count].port = c->d;
			q_srv_count++;
		}
		if (c->a == 0x31 && c->b == 0x201) {
			ipa_qmi_arrive(c->c, c->d);     /* the modem's IPA service, instance 2 */
		}
		if (c->a == QMI_SVC_WLFW) {
			LOG("t=%u ms: *** WLFW service 0x45 up: inst %x node %u port %x ***", g_ms, c->b, c->c, c->d);
			wlfw_arrive(c->c, c->d);
		} else if (c->a != 0) {
			LOG("t=%u ms: service %x:%x port %x %s", g_ms, c->a, c->b, c->d, svc_name(c->a));
		}
		break;
	case QRTR_TYPE_DATA:
		if (confirm) {
			/* the full 20-byte control packet (Linux qrtr_send_resume_tx): 12 bytes stalls the modem */
			struct qrtr_ctrl r = { QRTR_TYPE_RESUME_TX, QRTR_NODE_APPS, dst_port, 0, 0 };
			qrtr_send(QRTR_TYPE_RESUME_TX, dst_port, src_node, src_port, &r, sizeof(r));
		}
		if (hlen + size <= len) {
			(void)svc_rx(src_node, src_port, dst_port, p + hlen, size);
		}
		break;
	case QRTR_TYPE_RESUME_TX:
		q_resumes++;
		break;
	default:
		break;
	}
}

/* sapphire 0222: kern.sweet_modem_stats (bsd/dev/sweet_qmi_dev.c) */
extern "C" int
topaz_modem_stats(char *buf, size_t len)
{
	int n = snprintf(buf, len, "glink rx %u tx %u chan %s modem-intents %u txq %u drops %u; qrtr node %x hello %u services %u resume-rx %u; thread at '%s', %u loops, t=%u ms",
	    g_rx_msgs, g_tx_msgs, chan_up() ? "up" : "down", g_ri_count, g_txq_tail - g_txq_head, g_txq_drops,
	    q_modem_node, q_hello_rx, q_srv_count, q_resumes, g_where, g_loops, g_ms);
	if (n > 0 && (size_t)n < len) {
		n += ipa_stats(buf + n, len - n);
	}
	return n;
}

/* ---------------------------------------------------------------- GLINK RX */

static bool
rx_one(void)
{
	struct glink_msg m;
	uint32_t avail = rx_avail(), n;

	if (avail < sizeof(m)) {
		return false;
	}
	rx_peek(0, &m, sizeof(m));
	g_rx_msgs++;
	switch (m.cmd) {
	case CMD_VERSION:
		rx_advance(8);
		LOG("t=%u ms: glink VERSION %u features %x", g_ms, m.p1, m.p2);
		if (m.p1 != 0) {
			g_features &= m.p2;
			send_cmd(CMD_VERSION_ACK, 1, g_features);
		}
		break;
	case CMD_VERSION_ACK:
		rx_advance(8);
		LOG("t=%u ms: glink VERSION_ACK %u features %x", g_ms, m.p1, m.p2);
		if (m.p1 != 0 && m.p2 != g_features) {
			g_features &= m.p2;
			send_cmd(CMD_VERSION, 1, g_features);
		}
		break;
	case CMD_OPEN: {
		char name[33] = {};
		n = (8 + m.p2 + 7) & ~7u;
		if (avail < n) {
			return false;
		}
		rx_peek(8, name, MIN(m.p2, 32u));
		rx_advance(n);
		LOG("t=%u ms: glink modem OPEN \"%s\" rcid %u", g_ms, name, m.p1);
		if (strcmp(name, CHAN_NAME) == 0) {
			g_rcid = m.p1;
			g_remote_open = true;
			send_cmd(CMD_OPEN_ACK, (uint16_t)g_rcid, 0);
			if (!g_open_sent) {
				send_open();
			}
		}
		break;
	}
	case CMD_OPEN_ACK:
		rx_advance(8);
		LOG("t=%u ms: glink OPEN_ACK lcid %u", g_ms, m.p1);
		if (m.p1 == CHAN_LCID) {
			g_open_acked = true;
		}
		break;
	case CMD_CLOSE:
		rx_advance(8);
		LOG("t=%u ms: glink modem CLOSE rcid %u", g_ms, m.p1);
		send_cmd(CMD_CLOSE_ACK, m.p1, 0);
		if (m.p1 == g_rcid) {
			g_remote_open = false;
		}
		break;
	case CMD_INTENT:
		n = (8 + 8 * m.p2 + 7) & ~7u;
		if (avail < n) {
			return false;
		}
		for (uint32_t i = 0; i < m.p2; i++) {
			uint32_t pair[2];
			rx_peek(8 + 8 * i, pair, 8);
			if (m.p1 == g_rcid && g_ri_count < sizeof(g_ri) / sizeof(g_ri[0])) {
				g_ri[g_ri_count].size = pair[0];
				g_ri[g_ri_count].id = pair[1];
				g_ri[g_ri_count].used = false;
				g_ri_count++;
			}
		}
		rx_advance(n);
		g_intent_req_sent = false;
		break;
	case CMD_RX_DONE:
	case CMD_RX_DONE_W_REUSE:
		rx_advance(8);
		for (unsigned i = 0; i < g_ri_count; i++) {
			if (g_ri[i].id == m.p2) {
				if (m.cmd == CMD_RX_DONE_W_REUSE) {
					g_ri[i].used = false;
				} else {
					g_ri[i] = g_ri[--g_ri_count];
				}
				break;
			}
		}
		break;
	case CMD_RX_INTENT_REQ:
		rx_advance(8);
		advertise_intents(1, MAX(m.p2, OUR_INTENT_SIZE));
		send_cmd(CMD_RX_INTENT_REQ_ACK, CHAN_LCID, 1);
		break;
	case CMD_RX_INTENT_REQ_ACK:
		rx_advance(8);
		if (m.p2 == 0) {
			g_intent_req_sent = false;      /* ask again on the next pump */
		}
		break;
	case CMD_CLOSE_ACK:
	case CMD_SIGNALS:
		rx_advance(8);
		break;
	case CMD_READ_NOTIF:
		rx_advance(8);
		*(volatile uint32_t *)(apcs + APCS_IPC) = 1u << GLINK_IRQ_BIT;
		mb();
		break;
	case CMD_TX_DATA:
	case CMD_TX_DATA_CONT: {
		uint32_t hdr[2];
		if (avail < 16) {
			return false;
		}
		rx_peek(8, hdr, 8);                     /* chunk_size, left_size */
		n = (16 + hdr[0] + 7) & ~7u;
		if (avail < n) {
			return false;
		}
		if (m.cmd == CMD_TX_DATA) {
			g_asm_len = 0;
		}
		if (g_asm_len + hdr[0] <= sizeof(g_asm)) {
			rx_peek(16, g_asm + g_asm_len, hdr[0]);
			g_asm_len += hdr[0];
		}
		rx_advance(n);
		if (hdr[1] == 0) {
			if (m.p1 == g_rcid) {
				qrtr_rx(g_asm, g_asm_len);
			}
			if (g_features & 1) {
				send_cmd(CMD_RX_DONE_W_REUSE, CHAN_LCID, m.p2);
			} else {
				send_cmd(CMD_RX_DONE, CHAN_LCID, m.p2);
				advertise_intents(1, OUR_INTENT_SIZE);
			}
		}
		break;
	}
	default:
		LOG("t=%u ms: glink unknown cmd %u (%x %x), RX stopped", g_ms, m.cmd, m.p1, m.p2);
		g_rx = NULL;
		return false;
	}
	return true;
}

static bool
glink_start(struct smem_part_hdr *part)
{
	bool existed = smem_get(part, SMEM_GLINK_DESC, NULL) != NULL;

	g_desc = (volatile uint32_t *)smem_alloc(part, SMEM_GLINK_DESC, 32);
	g_tx = (uint8_t *)smem_alloc(part, SMEM_GLINK_FIFO_TX, GLINK_TX_SIZE);
	if (g_desc == NULL || g_tx == NULL) {
		LOG("glink: SMEM alloc failed (desc %p tx %p)", g_desc, g_tx);
		return false;
	}
	LOG("glink: desc %s, our TX FIFO %u bytes", existed ? "made by the modem" : "ours", GLINK_TX_SIZE);
	g_desc[2] = 0;                              /* rx tail */
	g_desc[1] = 0;                              /* tx head */
	mb();
	send_cmd(CMD_VERSION, 1, g_features);
	return true;
}

/* one pass, every millisecond from the watch loop */
static void
glink_poll(struct smem_part_hdr *part, unsigned ms)
{
	g_ms = ms;
	if (g_rx == NULL && g_rx_msgs == 0) {
		uint32_t sz = 0;
		g_rx = (uint8_t *)smem_get(part, SMEM_GLINK_FIFO_RX, &sz);
		if (g_rx != NULL) {
			g_rx_len = sz;
			LOG("t=%u ms: modem RX FIFO, %u bytes", ms, sz);
		}
	}
	WHERE("glink rx");
	for (int i = 0; i < 64 && g_rx != NULL && rx_one(); i++) {
	}
	if (q_hello_rx) {
		WHERE("qmi pump");
		sweet_qmi_dev_init();                   /* once; nodes for userland's QMI clients */
		sweet_qmi_pump(qmi_user_send);
	}
	WHERE("txq pump");
	txq_pump();
	WHERE("glink poll end");
	if (chan_up() && !g_intents_sent) {
		advertise_intents(OUR_INTENTS, OUR_INTENT_SIZE);
		g_intents_sent = true;
		LOG("t=%u ms: IPCRTR open both ways, %u intents sent", ms, OUR_INTENTS);
	}
	if (chan_up() && !q_hello_sent) {
		struct qrtr_ctrl c = { QRTR_TYPE_HELLO, 0, 0, 0, 0 };
		qrtr_send(QRTR_TYPE_HELLO, QRTR_PORT_CTRL, q_modem_node, QRTR_PORT_CTRL, &c, sizeof(c));
		q_hello_sent = true;
		LOG("t=%u ms: sent qrtr HELLO to node %x", ms, q_modem_node);
		if (q_hello_rx && !q_announced) {
			svc_announce();
			q_announced = true;
		}
	}
	/* nobody opened IPCRTR after 2 s: open it from our side */
	if (ms >= 2000 && !g_open_sent && g_rx != NULL) {
		LOG("t=%u ms: modem did not open IPCRTR, opening it", ms);
		send_open();
	}
}

static void
glink_summary(void)
{
	LOG("glink: rx %u msgs, tx %u, chan %s, modem intents %u, txq %u drops %u; qrtr node %x hello %u, %u services, %u RESUME_TX",
	    g_rx_msgs, g_tx_msgs, chan_up() ? "UP" : "down", g_ri_count, g_txq_tail - g_txq_head, g_txq_drops,
	    q_modem_node, q_hello_rx, q_srv_count, q_resumes);
	svc_summary();
	for (unsigned i = 0; i < q_srv_count; i++) {
		LOG("  service %x:%x node %u port %x %s", q_srv[i].svc, q_srv[i].inst, q_srv[i].node, q_srv[i].port, svc_name(q_srv[i].svc));
	}
}

/* ---------------------------------------------------------------- host services */

/*
 * What the modem expects from the apps processor (linux-msm userspace daemons), port of
 * topaz-woa drivers/TopazModem/ModemSvc.c:
 *  - pd-mapper: QMI servreg-locator 0x40 v0x101, maps "wlan/fw" etc. to msm/modem/wlan_pd;
 *  - tftp (QRTR service 4096): /readonly/firmware/image/... read-only from modem_a,
 *    /readwrite/... kept in RAM for the session (dropped writes starve modem_cfg);
 *  - rmtfs (QMI 14): the modem's EFS, read-only from modemst1/modemst2/fsc/fsg on UFS, into a
 *    3 MB buffer in our pool assigned to the modem through TrustZone. Writes are not persisted.
 */
#define PORT_PDM            0x4001
#define PORT_TFTP           0x4002
#define PORT_RMTFS          0x4003
#define PORT_SESS           0x4100
#define QMI_REQ             0
#define QMI_RESP            2

/* generated by fwtable_all.py from NON-HLOS.bin (HyperOS 2.0.208 modem_a): 657 files, 657 extents */
struct fw_extent { uint32_t off, len; };
struct fw_all_file { const char *path; uint32_t size; uint16_t ext, next; };
static const struct fw_extent fw_ext_topaz[] = {
	{ 0x25000, 0x4d4 },
	{ 0x29000, 0x2098 },
	{ 0x2d000, 0x14ef4 },
	{ 0x45000, 0x5e9c0 },
	{ 0xa5000, 0x18f388 },
	{ 0x235000, 0x19f628 },
	{ 0x3d5000, 0xb60 },
	{ 0x3d9000, 0x63e4 },
	{ 0x3e1000, 0x6f6c },
	{ 0x3e9000, 0x12dc },
	{ 0x3ed000, 0x104 },
	{ 0x3f1000, 0xf2a0 },
	{ 0x401000, 0x11ae4 },
	{ 0x415000, 0xa4000 },
	{ 0x4b9000, 0x170484 },
	{ 0x62d000, 0x44004 },
	{ 0x675000, 0x2dfb },
	{ 0x679000, 0x1b60 },
	{ 0x67d000, 0x2dc },
	{ 0x681000, 0x1f60 },
	{ 0x685000, 0x6fec },
	{ 0x68d000, 0x3a56c },
	{ 0x6c9000, 0x94000 },
	{ 0x75d000, 0x4db23e },
	{ 0xc39000, 0x15dd6a },
	{ 0xd99000, 0x423c },
	{ 0xda1000, 0x51a74 },
	{ 0xdf5000, 0xb70 },
	{ 0xdf9000, 0xd1f98 },
	{ 0xecd000, 0x50230 },
	{ 0xf21000, 0xe00 },
	{ 0xf25000, 0x13f0 },
	{ 0xf29000, 0x23810 },
	{ 0xf4d000, 0x116400 },
	{ 0x1065000, 0xd4 },
	{ 0x1069000, 0x256c },
	{ 0x15000, 0x193 },
	{ 0x19000, 0x195 },
	{ 0x1d000, 0x22b },
	{ 0x76b1000, 0x66d8 },
	{ 0x76b9000, 0x66d8 },
	{ 0x76c1000, 0x66d8 },
	{ 0x76c9000, 0x66d8 },
	{ 0x76d1000, 0x66d8 },
	{ 0x76d9000, 0x66d8 },
	{ 0x76e1000, 0x66d8 },
	{ 0x76e9000, 0x66d8 },
	{ 0x76f1000, 0x66d8 },
	{ 0x76f9000, 0x66d8 },
	{ 0x7701000, 0x66d8 },
	{ 0x7709000, 0x66d8 },
	{ 0x7711000, 0x66d8 },
	{ 0x7719000, 0x66d8 },
	{ 0x7721000, 0x66d8 },
	{ 0x7729000, 0x66d8 },
	{ 0x7731000, 0x66d8 },
	{ 0x7739000, 0x66d8 },
	{ 0x7741000, 0x66d8 },
	{ 0x7749000, 0x66d8 },
	{ 0x7751000, 0x66d8 },
	{ 0x7759000, 0x66d8 },
	{ 0x7761000, 0x66d8 },
	{ 0x7769000, 0x66d8 },
	{ 0x7771000, 0x66d8 },
	{ 0x7779000, 0x66d8 },
	{ 0x7781000, 0x66d8 },
	{ 0x7789000, 0x66d8 },
	{ 0x7791000, 0x66d8 },
	{ 0x7799000, 0x66d8 },
	{ 0x77a1000, 0x66d8 },
	{ 0x77a9000, 0x66d8 },
	{ 0x77b1000, 0x66d8 },
	{ 0x77b9000, 0x66d8 },
	{ 0x77c1000, 0x66d8 },
	{ 0x77c9000, 0x66d8 },
	{ 0x77d1000, 0x66d8 },
	{ 0x77d9000, 0x66d8 },
	{ 0x77e1000, 0x66d8 },
	{ 0x77e9000, 0x66d8 },
	{ 0x77f1000, 0x66d8 },
	{ 0x77f9000, 0x66d8 },
	{ 0x7801000, 0x66d8 },
	{ 0x7809000, 0x66d8 },
	{ 0x7811000, 0x66d8 },
	{ 0x7819000, 0x66d8 },
	{ 0x7821000, 0x66d8 },
	{ 0x7829000, 0x66d8 },
	{ 0x7831000, 0x66d8 },
	{ 0x7839000, 0x66d8 },
	{ 0x7841000, 0x66d8 },
	{ 0x106d000, 0x134 },
	{ 0x1071000, 0x1b28 },
	{ 0x1075000, 0x97d3 },
	{ 0x1081000, 0x91 },
	{ 0x1085000, 0x2b8 },
	{ 0x1089000, 0x14c },
	{ 0x108d000, 0x98 },
	{ 0x1091000, 0x1cab },
	{ 0x1095000, 0x1c5c },
	{ 0x1099000, 0x1d4 },
	{ 0x109d000, 0x1c18 },
	{ 0x10a1000, 0x39898 },
	{ 0x10dd000, 0x1a07c },
	{ 0x10f9000, 0x44c },
	{ 0x10fd000, 0x20db5c },
	{ 0x130d000, 0x5b60 },
	{ 0x1315000, 0x95378 },
	{ 0x13ad000, 0xa20 },
	{ 0x13b1000, 0xe614 },
	{ 0x13c1000, 0x1dec },
	{ 0x21000, 0x193 },
	{ 0x760d000, 0x9c2df },
	{ 0x13c5000, 0x200 },
	{ 0x13c9000, 0x4e70 },
	{ 0x13d1000, 0xa89d },
	{ 0x13dd000, 0xc4 },
	{ 0x13e1000, 0x391 },
	{ 0x13e5000, 0x1c0 },
	{ 0x13e9000, 0x130 },
	{ 0x13ed000, 0x2231 },
	{ 0x13f1000, 0x5070 },
	{ 0x13f9000, 0x200 },
	{ 0x13fd000, 0x1b28 },
	{ 0x1401000, 0x109ed8 },
	{ 0x150d000, 0x144 },
	{ 0x1511000, 0x103760 },
	{ 0x1615000, 0x468 },
	{ 0x1619000, 0x130 },
	{ 0x161d000, 0x24749 },
	{ 0x1645000, 0x1d28 },
	{ 0x1649000, 0x134 },
	{ 0x164d000, 0x1b28 },
	{ 0x1651000, 0x3b8c68 },
	{ 0x1a0d000, 0x9b },
	{ 0x1a11000, 0x2763a4 },
	{ 0x1c89000, 0x230 },
	{ 0x1c8d000, 0xa8 },
	{ 0x1c91000, 0x268d6 },
	{ 0x1cb9000, 0x1c5c },
	{ 0x1cbd000, 0x134 },
	{ 0x1cc1000, 0x1b28 },
	{ 0x1cc5000, 0x1430b },
	{ 0x1cdd000, 0xd4 },
	{ 0x1ce1000, 0x6bb4 },
	{ 0x1ce9000, 0x430 },
	{ 0x1ced000, 0x98 },
	{ 0x1cf1000, 0x6c49 },
	{ 0x1cf9000, 0x1c5c },
	{ 0x1cfd000, 0x200 },
	{ 0x1d01000, 0x1b28 },
	{ 0x1d05000, 0x541d },
	{ 0x1d0d000, 0xc2 },
	{ 0x1d11000, 0x388 },
	{ 0x1d15000, 0x1c0 },
	{ 0x1d19000, 0x130 },
	{ 0x1d1d000, 0x137d },
	{ 0x1d21000, 0x1d28 },
	{ 0x1d25000, 0x200 },
	{ 0x1d29000, 0x1b28 },
	{ 0x1d2d000, 0xc0f5 },
	{ 0x1d3d000, 0xc5 },
	{ 0x1d41000, 0x4c60 },
	{ 0x1d49000, 0x180 },
	{ 0x1d4d000, 0x130 },
	{ 0x1d51000, 0x10a9 },
	{ 0x1d55000, 0x1d28 },
	{ 0x1d59000, 0x134 },
	{ 0x1d5d000, 0x1b28 },
	{ 0x1d61000, 0x2bb74 },
	{ 0x1d8d000, 0x89 },
	{ 0x1d91000, 0xe47c },
	{ 0x1da1000, 0x188 },
	{ 0x1da5000, 0x98 },
	{ 0x1da9000, 0x68ee },
	{ 0x1db1000, 0x1c5c },
	{ 0x1db5000, 0x200 },
	{ 0x1db9000, 0x1b28 },
	{ 0x1dbd000, 0x1b20a },
	{ 0x1dd9000, 0xbe },
	{ 0x1ddd000, 0xc7c },
	{ 0x1de1000, 0x298 },
	{ 0x1de5000, 0x130 },
	{ 0x1de9000, 0x3931 },
	{ 0x1ded000, 0x1d28 },
	{ 0x1df1000, 0x134 },
	{ 0x1df5000, 0x1b28 },
	{ 0x1df9000, 0xaf99 },
	{ 0x1e05000, 0x8a },
	{ 0x1e09000, 0xa88 },
	{ 0x1e0d000, 0x174 },
	{ 0x1e11000, 0x98 },
	{ 0x1e15000, 0x231b },
	{ 0x1e19000, 0x1c5c },
	{ 0x1e1d000, 0x414 },
	{ 0x1e21000, 0x1f78 },
	{ 0x1e25000, 0x2930 },
	{ 0x1e29000, 0x2518 },
	{ 0x1e2d000, 0x4a3ac },
	{ 0x1e79000, 0x119d8 },
	{ 0x1e8d000, 0x100c },
	{ 0x1e91000, 0x90b4 },
	{ 0x1e9d000, 0x4f054 },
	{ 0x1eed000, 0x143ee8 },
	{ 0x2031000, 0x40000 },
	{ 0x2071000, 0x540 },
	{ 0x2075000, 0xe2210 },
	{ 0x2159000, 0x2d },
	{ 0x215d000, 0x7c },
	{ 0x2161000, 0x9c1a0 },
	{ 0x2201000, 0x61110 },
	{ 0x2265000, 0x2d29a9 },
	{ 0x2539000, 0x2d94cc },
	{ 0x2815000, 0x10ec0 },
	{ 0x2829000, 0x1722040 },
	{ 0x3f4d000, 0xa07900 },
	{ 0x4955000, 0x77a45 },
	{ 0x49cd000, 0x15104 },
	{ 0x49e5000, 0x1378 },
	{ 0x49e9000, 0xb0 },
	{ 0x49ed000, 0x7d52c },
	{ 0x4a6d000, 0x238c },
	{ 0x5351000, 0x588c },
	{ 0x5335000, 0x4404 },
	{ 0x5341000, 0x402c },
	{ 0x5359000, 0x20 },
	{ 0x535d000, 0x232 },
	{ 0x5361000, 0x1 },
	{ 0x537d000, 0xb4ac },
	{ 0x5395000, 0xb838 },
	{ 0x53ad000, 0xb7d8 },
	{ 0x53c1000, 0xbdc8 },
	{ 0x53d9000, 0xb50c },
	{ 0x53f1000, 0xc048 },
	{ 0x5409000, 0xc16c },
	{ 0x5421000, 0xc328 },
	{ 0x5439000, 0xbf28 },
	{ 0x544d000, 0xc418 },
	{ 0x5469000, 0xc2bc },
	{ 0x5481000, 0xbb40 },
	{ 0x5499000, 0xbd1c },
	{ 0x54ad000, 0xbc08 },
	{ 0x54c1000, 0xc464 },
	{ 0x54d9000, 0xbc00 },
	{ 0x5501000, 0xb7cc },
	{ 0x54ed000, 0xb898 },
	{ 0x5515000, 0xeb6c },
	{ 0x552d000, 0xc05c },
	{ 0x5545000, 0xb964 },
	{ 0x5575000, 0xb948 },
	{ 0x555d000, 0xbd34 },
	{ 0x5589000, 0xaff0 },
	{ 0x559d000, 0xbedc },
	{ 0x55b5000, 0xc240 },
	{ 0x55cd000, 0xf490 },
	{ 0x55e5000, 0x1011c },
	{ 0x5601000, 0xb388 },
	{ 0x5615000, 0xb8b8 },
	{ 0x5629000, 0xbc40 },
	{ 0x563d000, 0xb8d8 },
	{ 0x5651000, 0xb80c },
	{ 0x5665000, 0xc2dc },
	{ 0x567d000, 0xb7dc },
	{ 0x5691000, 0xb940 },
	{ 0x56a5000, 0xbcf4 },
	{ 0x56c1000, 0xc4a4 },
	{ 0x56d9000, 0xc794 },
	{ 0x56f1000, 0xc148 },
	{ 0x5711000, 0xd030 },
	{ 0x5725000, 0xe098 },
	{ 0x573d000, 0x6c5c },
	{ 0x5749000, 0xcc88 },
	{ 0x575d000, 0x5eb0 },
	{ 0x5769000, 0x6c64 },
	{ 0x5775000, 0xc664 },
	{ 0x5789000, 0xc7e4 },
	{ 0x579d000, 0x6c64 },
	{ 0x57a9000, 0x5b60 },
	{ 0x57b5000, 0x5e04 },
	{ 0x57f1000, 0x119bc },
	{ 0x57c9000, 0xa698 },
	{ 0x57d9000, 0x11c50 },
	{ 0x580d000, 0x7618 },
	{ 0x5819000, 0xe4c0 },
	{ 0x582d000, 0xe120 },
	{ 0x5841000, 0x9fa0 },
	{ 0x5851000, 0xa060 },
	{ 0x5861000, 0x9cc4 },
	{ 0x5871000, 0xe350 },
	{ 0x5889000, 0xbcc0 },
	{ 0x58a1000, 0x690c },
	{ 0x58ad000, 0xd604 },
	{ 0x58c5000, 0x52f4 },
	{ 0x71a1000, 0xb7f4 },
	{ 0x71b5000, 0xc568 },
	{ 0x71cd000, 0xab94 },
	{ 0x58dd000, 0xc878 },
	{ 0x58f5000, 0xbbc0 },
	{ 0x590d000, 0xbbf0 },
	{ 0x5921000, 0xb824 },
	{ 0x5931000, 0xb90c },
	{ 0x5955000, 0xbbfc },
	{ 0x5945000, 0xc304 },
	{ 0x5965000, 0xbeec },
	{ 0x5975000, 0xbe0c },
	{ 0x5985000, 0xb88c },
	{ 0x599d000, 0xb8f4 },
	{ 0x59c9000, 0xbbe0 },
	{ 0x59d9000, 0xbbf8 },
	{ 0x59f1000, 0xc734 },
	{ 0x59b1000, 0xb854 },
	{ 0x6595000, 0xc7c4 },
	{ 0x5a09000, 0xd034 },
	{ 0x5b31000, 0xc4c8 },
	{ 0x5b45000, 0xbf00 },
	{ 0x5b55000, 0xb448 },
	{ 0x5a25000, 0xcd68 },
	{ 0x5a4d000, 0xcac8 },
	{ 0x5a39000, 0xccfc },
	{ 0x5a61000, 0xd02c },
	{ 0x5a75000, 0xd688 },
	{ 0x5a89000, 0xbff8 },
	{ 0x5a99000, 0xca3c },
	{ 0x5aad000, 0xcdc4 },
	{ 0x5ac1000, 0xc82c },
	{ 0x5ad9000, 0x5b58 },
	{ 0x5ae9000, 0xca60 },
	{ 0x5afd000, 0xca98 },
	{ 0x5b19000, 0xbb48 },
	{ 0x5b69000, 0xc688 },
	{ 0x5b85000, 0xb7d8 },
	{ 0x5b9d000, 0xbdf0 },
	{ 0x5bad000, 0xbfe0 },
	{ 0x5bc1000, 0xbc40 },
	{ 0x5bd9000, 0xb944 },
	{ 0x5be9000, 0xbecc },
	{ 0x5c01000, 0xd440 },
	{ 0x5c1d000, 0xbac8 },
	{ 0x5c35000, 0xba58 },
	{ 0x5c4d000, 0xb954 },
	{ 0x5c5d000, 0xbf58 },
	{ 0x5c6d000, 0xbec4 },
	{ 0x5c7d000, 0xd450 },
	{ 0x5c91000, 0xc0d0 },
	{ 0x5ca5000, 0xbba4 },
	{ 0x5cbd000, 0xc1cc },
	{ 0x5cd5000, 0xc03c },
	{ 0x5cf1000, 0xc018 },
	{ 0x5d25000, 0xbce8 },
	{ 0x5d3d000, 0xc0cc },
	{ 0x5d0d000, 0xb5e4 },
	{ 0x5d55000, 0xc828 },
	{ 0x5df5000, 0xb3fc },
	{ 0x5d71000, 0xb8f4 },
	{ 0x5d89000, 0xc190 },
	{ 0x5d9d000, 0xc94c },
	{ 0x5db9000, 0xbf18 },
	{ 0x5dc9000, 0xbb1c },
	{ 0x5ddd000, 0xb95c },
	{ 0x5e09000, 0xbb00 },
	{ 0x5e1d000, 0xce10 },
	{ 0x5e39000, 0xc678 },
	{ 0x5e4d000, 0xc7e4 },
	{ 0x5e61000, 0xc5dc },
	{ 0x5e7d000, 0xb8b8 },
	{ 0x5e95000, 0xbf04 },
	{ 0x5ea5000, 0xb8fc },
	{ 0x5eb5000, 0xc1d0 },
	{ 0x5ec9000, 0xd5ac },
	{ 0x5edd000, 0x6180 },
	{ 0x5ee9000, 0xb84c },
	{ 0x5ef9000, 0xdc88 },
	{ 0x5f0d000, 0xb8f8 },
	{ 0x5f1d000, 0xc9c0 },
	{ 0x5f31000, 0xc5d0 },
	{ 0x5f45000, 0xcbb4 },
	{ 0x5f59000, 0xaf7c },
	{ 0x5f6d000, 0xcc50 },
	{ 0x5f89000, 0xb8cc },
	{ 0x5f9d000, 0xcf44 },
	{ 0x5fb9000, 0xb9d8 },
	{ 0x5fcd000, 0xbc4c },
	{ 0x5fe1000, 0xb8e8 },
	{ 0x601d000, 0xc568 },
	{ 0x5ff9000, 0xbe88 },
	{ 0x6009000, 0xbd30 },
	{ 0x6035000, 0xc150 },
	{ 0x6051000, 0xbaf8 },
	{ 0x6061000, 0xbcf8 },
	{ 0x6071000, 0xbde0 },
	{ 0x6085000, 0xb8c8 },
	{ 0x609d000, 0xc2d0 },
	{ 0x60b9000, 0xbd74 },
	{ 0x6121000, 0xc7e0 },
	{ 0x6149000, 0xc7dc },
	{ 0x6135000, 0xc800 },
	{ 0x615d000, 0xbf6c },
	{ 0x616d000, 0xb4b8 },
	{ 0x6185000, 0xbff8 },
	{ 0x6195000, 0xc9a0 },
	{ 0x61a9000, 0xbd20 },
	{ 0x61bd000, 0xc8a4 },
	{ 0x61d9000, 0xb288 },
	{ 0x61e9000, 0xbdb0 },
	{ 0x61fd000, 0xb9fc },
	{ 0x620d000, 0xb4a0 },
	{ 0x6225000, 0xbb18 },
	{ 0x623d000, 0xb6c8 },
	{ 0x624d000, 0xc754 },
	{ 0x6261000, 0xb9a8 },
	{ 0x6271000, 0xbda8 },
	{ 0x6281000, 0xb8b8 },
	{ 0x6291000, 0xb524 },
	{ 0x62a1000, 0xbf44 },
	{ 0x62b1000, 0xc224 },
	{ 0x62c5000, 0xbf18 },
	{ 0x62dd000, 0xc478 },
	{ 0x62f1000, 0xc2b0 },
	{ 0x6305000, 0xc7fc },
	{ 0x6319000, 0xc314 },
	{ 0x632d000, 0xcb54 },
	{ 0x6341000, 0xca0c },
	{ 0x6359000, 0xc144 },
	{ 0x60d1000, 0xc8d4 },
	{ 0x60e5000, 0xa828 },
	{ 0x60f5000, 0xa7a8 },
	{ 0x6109000, 0xb924 },
	{ 0x6385000, 0xbb10 },
	{ 0x6371000, 0xb9c4 },
	{ 0x65ad000, 0xc0dc },
	{ 0x6399000, 0xbf50 },
	{ 0x63b1000, 0xbb34 },
	{ 0x63c5000, 0xc2cc },
	{ 0x63e1000, 0xbf60 },
	{ 0x63f1000, 0xb818 },
	{ 0x6401000, 0xc6b4 },
	{ 0x6415000, 0xb91c },
	{ 0x6425000, 0x5f6c },
	{ 0x6435000, 0xc1cc },
	{ 0x6449000, 0xcee0 },
	{ 0x645d000, 0xc6e4 },
	{ 0x6485000, 0xce40 },
	{ 0x6471000, 0xc720 },
	{ 0x6499000, 0xab68 },
	{ 0x64a9000, 0xcca4 },
	{ 0x64bd000, 0xab60 },
	{ 0x64cd000, 0xc118 },
	{ 0x64e1000, 0xc360 },
	{ 0x64f5000, 0xcc98 },
	{ 0x6509000, 0xc384 },
	{ 0x651d000, 0xcec8 },
	{ 0x6539000, 0xbdbc },
	{ 0x654d000, 0xc5e0 },
	{ 0x6565000, 0xb708 },
	{ 0x6579000, 0xc064 },
	{ 0x65c9000, 0xb368 },
	{ 0x65dd000, 0xaee4 },
	{ 0x65f1000, 0xa210 },
	{ 0x6605000, 0x9bf4 },
	{ 0x6621000, 0xc238 },
	{ 0x6635000, 0x393c },
	{ 0x663d000, 0xc638 },
	{ 0x6655000, 0xbd84 },
	{ 0x666d000, 0xbcb4 },
	{ 0x6685000, 0xd9f0 },
	{ 0x6699000, 0xda10 },
	{ 0x66c1000, 0xe430 },
	{ 0x66d5000, 0xebf0 },
	{ 0x66ad000, 0xda28 },
	{ 0x66e9000, 0xda40 },
	{ 0x66fd000, 0xdd08 },
	{ 0x6711000, 0xda34 },
	{ 0x6725000, 0xda2c },
	{ 0x6739000, 0xda30 },
	{ 0x674d000, 0xd9d4 },
	{ 0x6761000, 0xd8b8 },
	{ 0x6775000, 0xeba8 },
	{ 0x6789000, 0xd9cc },
	{ 0x679d000, 0xda2c },
	{ 0x67b1000, 0xd980 },
	{ 0x67cd000, 0xfb3c },
	{ 0x67e1000, 0xefa0 },
	{ 0x67fd000, 0xbc00 },
	{ 0x6829000, 0xb4b8 },
	{ 0x6839000, 0xb7fc },
	{ 0x6849000, 0xb578 },
	{ 0x6859000, 0xb5fc },
	{ 0x6869000, 0xbd6c },
	{ 0x6879000, 0xb508 },
	{ 0x6811000, 0xe110 },
	{ 0x68a9000, 0xbd1c },
	{ 0x68b9000, 0xe0c8 },
	{ 0x68d5000, 0xbb24 },
	{ 0x68e1000, 0xba68 },
	{ 0x6891000, 0xa2c0 },
	{ 0x68f9000, 0xbf00 },
	{ 0x6911000, 0xc10c },
	{ 0x692d000, 0xc114 },
	{ 0x693d000, 0xb014 },
	{ 0x6955000, 0xc7d0 },
	{ 0x696d000, 0xb7f0 },
	{ 0x697d000, 0xb7f0 },
	{ 0x6991000, 0xb79c },
	{ 0x69a5000, 0xb79c },
	{ 0x69b9000, 0xbab4 },
	{ 0x69cd000, 0xb8a0 },
	{ 0x69e5000, 0xb9f0 },
	{ 0x69f9000, 0xb958 },
	{ 0x6a39000, 0xbb10 },
	{ 0x6a11000, 0xbd94 },
	{ 0x6a25000, 0xbd28 },
	{ 0x6a51000, 0xb798 },
	{ 0x6a65000, 0xb8dc },
	{ 0x6a79000, 0xc2fc },
	{ 0x6a91000, 0xb8b0 },
	{ 0x6aa5000, 0xb420 },
	{ 0x6ad5000, 0xbc18 },
	{ 0x6ae9000, 0xb61c },
	{ 0x6afd000, 0xb7e4 },
	{ 0x6abd000, 0xb8d4 },
	{ 0x6b15000, 0xb948 },
	{ 0x6b29000, 0xb928 },
	{ 0x6b41000, 0xd5fc },
	{ 0x6b55000, 0xd4a0 },
	{ 0x6b69000, 0xd5e4 },
	{ 0x6b7d000, 0x61b8 },
	{ 0x6b89000, 0xd490 },
	{ 0x6ba5000, 0xc9c8 },
	{ 0x6bc1000, 0x5454 },
	{ 0x6bd5000, 0xbb7c },
	{ 0x6bed000, 0xbff0 },
	{ 0x6c01000, 0xf0dc },
	{ 0x6c15000, 0xf0d8 },
	{ 0x6c59000, 0xc434 },
	{ 0x6c2d000, 0xd210 },
	{ 0x6c41000, 0xba04 },
	{ 0x6c71000, 0xc850 },
	{ 0x6c89000, 0x116e8 },
	{ 0x6cb1000, 0x132dc },
	{ 0x6ca1000, 0x93f0 },
	{ 0x6cd1000, 0xcec8 },
	{ 0x6d11000, 0xc97c },
	{ 0x6d29000, 0xb79c },
	{ 0x6cf9000, 0xb8d8 },
	{ 0x6ced000, 0xb964 },
	{ 0x6d3d000, 0xbdc8 },
	{ 0x6d51000, 0xc778 },
	{ 0x6d6d000, 0x32cc },
	{ 0x6d81000, 0xbed0 },
	{ 0x7151000, 0xb904 },
	{ 0x6d99000, 0xb850 },
	{ 0x6dad000, 0xbaa4 },
	{ 0x6dd5000, 0xbdec },
	{ 0x6ded000, 0xc99c },
	{ 0x6dc1000, 0xb3dc },
	{ 0x6e49000, 0xca88 },
	{ 0x6e09000, 0xc0b0 },
	{ 0x6e25000, 0xbdd0 },
	{ 0x6e35000, 0xbdd0 },
	{ 0x6e61000, 0xbee4 },
	{ 0x6e79000, 0xbdc0 },
	{ 0x6e91000, 0xb8f0 },
	{ 0x6ea5000, 0xba58 },
	{ 0x6ebd000, 0xc208 },
	{ 0x6f09000, 0xb250 },
	{ 0x6f1d000, 0xbba4 },
	{ 0x6f31000, 0xc720 },
	{ 0x6ed9000, 0xb428 },
	{ 0x6ef1000, 0xbbb8 },
	{ 0x6f4d000, 0xc364 },
	{ 0x6f69000, 0xbb8c },
	{ 0x6f7d000, 0xd70c },
	{ 0x6f99000, 0xc6e4 },
	{ 0x7165000, 0xb950 },
	{ 0x6fc9000, 0xb938 },
	{ 0x6ff1000, 0xc2a4 },
	{ 0x6fe1000, 0xc254 },
	{ 0x6fb5000, 0xb9a4 },
	{ 0x700d000, 0xb9c8 },
	{ 0x7065000, 0xc388 },
	{ 0x7021000, 0xbd54 },
	{ 0x7039000, 0xbcb8 },
	{ 0x707d000, 0xbb04 },
	{ 0x7095000, 0xc010 },
	{ 0x70a9000, 0xc030 },
	{ 0x7051000, 0xa408 },
	{ 0x70c5000, 0xc5f0 },
	{ 0x70e1000, 0xb8f0 },
	{ 0x70f9000, 0xbb30 },
	{ 0x7111000, 0xbdbc },
	{ 0x7125000, 0xb990 },
	{ 0x7139000, 0xbb84 },
	{ 0x7179000, 0xb7b0 },
	{ 0x7189000, 0xb744 },
	{ 0x71d9000, 0x20 },
	{ 0x71dd000, 0x4a2c },
	{ 0x71e5000, 0x378d },
	{ 0x71e9000, 0x3ece },
	{ 0x5309000, 0x2cf },
	{ 0x76ad000, 0x2c9 },
	{ 0x4a71000, 0x134 },
	{ 0x4a75000, 0x1b28 },
	{ 0x4a79000, 0x4c0c0 },
	{ 0x4ac9000, 0x8f },
	{ 0x4acd000, 0x3b80 },
	{ 0x4ad1000, 0x238 },
	{ 0x4ad5000, 0x98 },
	{ 0x4ad9000, 0xb3c3 },
	{ 0x4ae5000, 0x1c5c },
	{ 0x4f29000, 0x3df2b6 },
	{ 0x71ed000, 0x493ba },
	{ 0x4ae9000, 0x134 },
	{ 0x4aed000, 0x1b28 },
	{ 0x4af1000, 0x55b1 },
	{ 0x4af9000, 0x98 },
	{ 0x4afd000, 0x460 },
	{ 0x4b01000, 0xdc },
	{ 0x4b05000, 0x98 },
	{ 0x4b09000, 0x102c },
	{ 0x4b0d000, 0x1c5c },
	{ 0x4b11000, 0x134 },
	{ 0x4b15000, 0x1b28 },
	{ 0x4b19000, 0x43c7c },
	{ 0x4b5d000, 0xcc },
	{ 0x4b61000, 0x1688bc },
	{ 0x4ccd000, 0x4c0 },
	{ 0x4cd1000, 0x98 },
	{ 0x4cd5000, 0xb1a5 },
	{ 0x4ce1000, 0x1c5c },
	{ 0x4ce5000, 0x200 },
	{ 0x4ce9000, 0x1b28 },
	{ 0x4ced000, 0x4ff70 },
	{ 0x4d3d000, 0xf8 },
	{ 0x4d41000, 0x16b250 },
	{ 0x4ead000, 0x980 },
	{ 0x4eb1000, 0x130 },
	{ 0x4eb5000, 0x1452b },
	{ 0x4ecd000, 0x1d28 },
	{ 0x4ed1000, 0x200 },
	{ 0x4ed5000, 0x1b28 },
	{ 0x4ed9000, 0xcecd },
	{ 0x4ee9000, 0xc2 },
	{ 0x4eed000, 0x840 },
	{ 0x4ef1000, 0x288 },
	{ 0x4ef5000, 0x130 },
	{ 0x4ef9000, 0x1b48 },
	{ 0x4efd000, 0x1d28 },
	{ 0x4f01000, 0x134 },
	{ 0x4f05000, 0x1b28 },
	{ 0x4f09000, 0x7d3b },
	{ 0x4f11000, 0xbb },
	{ 0x4f15000, 0x334 },
	{ 0x4f19000, 0x10c },
	{ 0x4f1d000, 0x98 },
	{ 0x4f21000, 0x1510 },
	{ 0x4f25000, 0x1c5c },
	{ 0x7239000, 0x2ecfa },
	{ 0x7269000, 0x3a39c0 },
};
static const struct fw_all_file fw_all_topaz[] = {
	{ "image/adsp.b00", 0x4d4, 0, 1 },
	{ "image/adsp.b01", 0x2098, 1, 1 },
	{ "image/adsp.b02", 0x14ef4, 2, 1 },
	{ "image/adsp.b03", 0x5e9c0, 3, 1 },
	{ "image/adsp.b04", 0x18f388, 4, 1 },
	{ "image/adsp.b05", 0x19f628, 5, 1 },
	{ "image/adsp.b06", 0xb60, 6, 1 },
	{ "image/adsp.b07", 0x63e4, 7, 1 },
	{ "image/adsp.b08", 0x6f6c, 8, 1 },
	{ "image/adsp.b09", 0x12dc, 9, 1 },
	{ "image/adsp.b10", 0x104, 10, 1 },
	{ "image/adsp.b12", 0xf2a0, 11, 1 },
	{ "image/adsp.b13", 0x11ae4, 12, 1 },
	{ "image/adsp.b14", 0xa4000, 13, 1 },
	{ "image/adsp.b15", 0x170484, 14, 1 },
	{ "image/adsp.b16", 0x44004, 15, 1 },
	{ "image/adsp.b17", 0x2dfb, 16, 1 },
	{ "image/adsp.b18", 0x1b60, 17, 1 },
	{ "image/adsp.b19", 0x2dc, 18, 1 },
	{ "image/adsp.b20", 0x1f60, 19, 1 },
	{ "image/adsp.b21", 0x6fec, 20, 1 },
	{ "image/adsp.b22", 0x3a56c, 21, 1 },
	{ "image/adsp.b23", 0x94000, 22, 1 },
	{ "image/adsp.b24", 0x4db23e, 23, 1 },
	{ "image/adsp.b25", 0x15dd6a, 24, 1 },
	{ "image/adsp.b26", 0x423c, 25, 1 },
	{ "image/adsp.b27", 0x51a74, 26, 1 },
	{ "image/adsp.b28", 0xb70, 27, 1 },
	{ "image/adsp.b29", 0xd1f98, 28, 1 },
	{ "image/adsp.b30", 0x50230, 29, 1 },
	{ "image/adsp.b31", 0xe00, 30, 1 },
	{ "image/adsp.b32", 0x13f0, 31, 1 },
	{ "image/adsp.b33", 0x23810, 32, 1 },
	{ "image/adsp.b34", 0x116400, 33, 1 },
	{ "image/adsp.b35", 0xd4, 34, 1 },
	{ "image/adsp.mdt", 0x256c, 35, 1 },
	{ "image/adspr.jsn", 0x193, 36, 1 },
	{ "image/adsps.jsn", 0x195, 37, 1 },
	{ "image/adspua.jsn", 0x22b, 38, 1 },
	{ "image/bdm7eu.bin", 0x66d8, 39, 1 },
	{ "image/bdm7eut.bin", 0x66d8, 40, 1 },
	{ "image/bdm7euu.bin", 0x66d8, 41, 1 },
	{ "image/bdm7gl.bin", 0x66d8, 42, 1 },
	{ "image/bdm7glt.bin", 0x66d8, 43, 1 },
	{ "image/bdm7glu.bin", 0x66d8, 44, 1 },
	{ "image/bdm7in.bin", 0x66d8, 45, 1 },
	{ "image/bdm7int.bin", 0x66d8, 46, 1 },
	{ "image/bdm7inu.bin", 0x66d8, 47, 1 },
	{ "image/bdm7mx.bin", 0x66d8, 48, 1 },
	{ "image/bdm7mxt.bin", 0x66d8, 49, 1 },
	{ "image/bdm7mxu.bin", 0x66d8, 50, 1 },
	{ "image/bdwlan.102", 0x66d8, 51, 1 },
	{ "image/bdwlan.104", 0x66d8, 52, 1 },
	{ "image/bdwlan.105", 0x66d8, 53, 1 },
	{ "image/bdwlan.106", 0x66d8, 54, 1 },
	{ "image/bdwlan.107", 0x66d8, 55, 1 },
	{ "image/bdwlan.108", 0x66d8, 56, 1 },
	{ "image/bdwlan.109", 0x66d8, 57, 1 },
	{ "image/bdwlan.10e", 0x66d8, 58, 1 },
	{ "image/bdwlan.10f", 0x66d8, 59, 1 },
	{ "image/bdwlan.111", 0x66d8, 60, 1 },
	{ "image/bdwlan.202", 0x66d8, 61, 1 },
	{ "image/bdwlan.203", 0x66d8, 62, 1 },
	{ "image/bdwlan.204", 0x66d8, 63, 1 },
	{ "image/bdwlan.206", 0x66d8, 64, 1 },
	{ "image/bdwlan.b04", 0x66d8, 65, 1 },
	{ "image/bdwlan.b07", 0x66d8, 66, 1 },
	{ "image/bdwlan.b09", 0x66d8, 67, 1 },
	{ "image/bdwlan.b0a", 0x66d8, 68, 1 },
	{ "image/bdwlan.b0b", 0x66d8, 69, 1 },
	{ "image/bdwlan.b0d", 0x66d8, 70, 1 },
	{ "image/bdwlan.b0e", 0x66d8, 71, 1 },
	{ "image/bdwlan.b0f", 0x66d8, 72, 1 },
	{ "image/bdwlan.b14", 0x66d8, 73, 1 },
	{ "image/bdwlan.b15", 0x66d8, 74, 1 },
	{ "image/bdwlan.b30", 0x66d8, 75, 1 },
	{ "image/bdwlan.b31", 0x66d8, 76, 1 },
	{ "image/bdwlan.b32", 0x66d8, 77, 1 },
	{ "image/bdwlan.b33", 0x66d8, 78, 1 },
	{ "image/bdwlan.b34", 0x66d8, 79, 1 },
	{ "image/bdwlan.b35", 0x66d8, 80, 1 },
	{ "image/bdwlan.b36", 0x66d8, 81, 1 },
	{ "image/bdwlan.b37", 0x66d8, 82, 1 },
	{ "image/bdwlan.b38", 0x66d8, 83, 1 },
	{ "image/bdwlan.bin", 0x66d8, 84, 1 },
	{ "image/bdwlans.202", 0x66d8, 85, 1 },
	{ "image/bdwlant.202", 0x66d8, 86, 1 },
	{ "image/bdwlanu.202", 0x66d8, 87, 1 },
	{ "image/bdwlanu.206", 0x66d8, 88, 1 },
	{ "image/bdwlanu.206", 0x66d8, 89, 1 },
	{ "image/cardapp.b00", 0x134, 90, 1 },
	{ "image/cardapp.b01", 0x1b28, 91, 1 },
	{ "image/cardapp.b02", 0x97d3, 92, 1 },
	{ "image/cardapp.b03", 0x91, 93, 1 },
	{ "image/cardapp.b04", 0x2b8, 94, 1 },
	{ "image/cardapp.b05", 0x14c, 95, 1 },
	{ "image/cardapp.b06", 0x98, 96, 1 },
	{ "image/cardapp.b07", 0x1cab, 97, 1 },
	{ "image/cardapp.mdt", 0x1c5c, 98, 1 },
	{ "image/cdsp.b00", 0x1d4, 99, 1 },
	{ "image/cdsp.b01", 0x1c18, 100, 1 },
	{ "image/cdsp.b02", 0x39898, 101, 1 },
	{ "image/cdsp.b03", 0x1a07c, 102, 1 },
	{ "image/cdsp.b04", 0x44c, 103, 1 },
	{ "image/cdsp.b05", 0x20db5c, 104, 1 },
	{ "image/cdsp.b06", 0x5b60, 105, 1 },
	{ "image/cdsp.b08", 0x95378, 106, 1 },
	{ "image/cdsp.b09", 0xa20, 107, 1 },
	{ "image/cdsp.b11", 0xe614, 108, 1 },
	{ "image/cdsp.mdt", 0x1dec, 109, 1 },
	{ "image/cdspr.jsn", 0x193, 110, 1 },
	{ "image/data.msc", 0x9c2df, 111, 1 },
	{ "image/featenab.b00", 0x200, 112, 1 },
	{ "image/featenab.b01", 0x4e70, 113, 1 },
	{ "image/featenab.b02", 0xa89d, 114, 1 },
	{ "image/featenab.b03", 0xc4, 115, 1 },
	{ "image/featenab.b04", 0x391, 116, 1 },
	{ "image/featenab.b05", 0x1c0, 117, 1 },
	{ "image/featenab.b06", 0x130, 118, 1 },
	{ "image/featenab.b07", 0x2231, 119, 1 },
	{ "image/featenab.mdt", 0x5070, 120, 1 },
	{ "image/fpctzapp.b00", 0x200, 121, 1 },
	{ "image/fpctzapp.b01", 0x1b28, 122, 1 },
	{ "image/fpctzapp.b02", 0x109ed8, 123, 1 },
	{ "image/fpctzapp.b03", 0x144, 124, 1 },
	{ "image/fpctzapp.b04", 0x103760, 125, 1 },
	{ "image/fpctzapp.b05", 0x468, 126, 1 },
	{ "image/fpctzapp.b06", 0x130, 127, 1 },
	{ "image/fpctzapp.b07", 0x24749, 128, 1 },
	{ "image/fpctzapp.mdt", 0x1d28, 129, 1 },
	{ "image/goodixfp.b00", 0x134, 130, 1 },
	{ "image/goodixfp.b01", 0x1b28, 131, 1 },
	{ "image/goodixfp.b02", 0x3b8c68, 132, 1 },
	{ "image/goodixfp.b03", 0x9b, 133, 1 },
	{ "image/goodixfp.b04", 0x2763a4, 134, 1 },
	{ "image/goodixfp.b05", 0x230, 135, 1 },
	{ "image/goodixfp.b06", 0xa8, 136, 1 },
	{ "image/goodixfp.b07", 0x268d6, 137, 1 },
	{ "image/goodixfp.mdt", 0x1c5c, 138, 1 },
	{ "image/gptest.b00", 0x134, 139, 1 },
	{ "image/gptest.b01", 0x1b28, 140, 1 },
	{ "image/gptest.b02", 0x1430b, 141, 1 },
	{ "image/gptest.b03", 0xd4, 142, 1 },
	{ "image/gptest.b04", 0x6bb4, 143, 1 },
	{ "image/gptest.b05", 0x430, 144, 1 },
	{ "image/gptest.b06", 0x98, 145, 1 },
	{ "image/gptest.b07", 0x6c49, 146, 1 },
	{ "image/gptest.mdt", 0x1c5c, 147, 1 },
	{ "image/hdcpsrm.b00", 0x200, 148, 1 },
	{ "image/hdcpsrm.b01", 0x1b28, 149, 1 },
	{ "image/hdcpsrm.b02", 0x541d, 150, 1 },
	{ "image/hdcpsrm.b03", 0xc2, 151, 1 },
	{ "image/hdcpsrm.b04", 0x388, 152, 1 },
	{ "image/hdcpsrm.b05", 0x1c0, 153, 1 },
	{ "image/hdcpsrm.b06", 0x130, 154, 1 },
	{ "image/hdcpsrm.b07", 0x137d, 155, 1 },
	{ "image/hdcpsrm.mdt", 0x1d28, 156, 1 },
	{ "image/loadalgo.b00", 0x200, 157, 1 },
	{ "image/loadalgo.b01", 0x1b28, 158, 1 },
	{ "image/loadalgo.b02", 0xc0f5, 159, 1 },
	{ "image/loadalgo.b03", 0xc5, 160, 1 },
	{ "image/loadalgo.b04", 0x4c60, 161, 1 },
	{ "image/loadalgo.b05", 0x180, 162, 1 },
	{ "image/loadalgo.b06", 0x130, 163, 1 },
	{ "image/loadalgo.b07", 0x10a9, 164, 1 },
	{ "image/loadalgo.mdt", 0x1d28, 165, 1 },
	{ "image/mfido.b00", 0x134, 166, 1 },
	{ "image/mfido.b01", 0x1b28, 167, 1 },
	{ "image/mfido.b02", 0x2bb74, 168, 1 },
	{ "image/mfido.b03", 0x89, 169, 1 },
	{ "image/mfido.b04", 0xe47c, 170, 1 },
	{ "image/mfido.b05", 0x188, 171, 1 },
	{ "image/mfido.b06", 0x98, 172, 1 },
	{ "image/mfido.b07", 0x68ee, 173, 1 },
	{ "image/mfido.mdt", 0x1c5c, 174, 1 },
	{ "image/miriskm.b00", 0x200, 175, 1 },
	{ "image/miriskm.b01", 0x1b28, 176, 1 },
	{ "image/miriskm.b02", 0x1b20a, 177, 1 },
	{ "image/miriskm.b03", 0xbe, 178, 1 },
	{ "image/miriskm.b04", 0xc7c, 179, 1 },
	{ "image/miriskm.b05", 0x298, 180, 1 },
	{ "image/miriskm.b06", 0x130, 181, 1 },
	{ "image/miriskm.b07", 0x3931, 182, 1 },
	{ "image/miriskm.mdt", 0x1d28, 183, 1 },
	{ "image/mlipay.b00", 0x134, 184, 1 },
	{ "image/mlipay.b01", 0x1b28, 185, 1 },
	{ "image/mlipay.b02", 0xaf99, 186, 1 },
	{ "image/mlipay.b03", 0x8a, 187, 1 },
	{ "image/mlipay.b04", 0xa88, 188, 1 },
	{ "image/mlipay.b05", 0x174, 189, 1 },
	{ "image/mlipay.b06", 0x98, 190, 1 },
	{ "image/mlipay.b07", 0x231b, 191, 1 },
	{ "image/mlipay.mdt", 0x1c5c, 192, 1 },
	{ "image/modem.b00", 0x414, 193, 1 },
	{ "image/modem.b01", 0x1f78, 194, 1 },
	{ "image/modem.b02", 0x2930, 195, 1 },
	{ "image/modem.b03", 0x2518, 196, 1 },
	{ "image/modem.b04", 0x4a3ac, 197, 1 },
	{ "image/modem.b05", 0x119d8, 198, 1 },
	{ "image/modem.b06", 0x100c, 199, 1 },
	{ "image/modem.b07", 0x90b4, 200, 1 },
	{ "image/modem.b08", 0x4f054, 201, 1 },
	{ "image/modem.b09", 0x143ee8, 202, 1 },
	{ "image/modem.b10", 0x40000, 203, 1 },
	{ "image/modem.b11", 0x540, 204, 1 },
	{ "image/modem.b12", 0xe2210, 205, 1 },
	{ "image/modem.b14", 0x2d, 206, 1 },
	{ "image/modem.b15", 0x7c, 207, 1 },
	{ "image/modem.b16", 0x9c1a0, 208, 1 },
	{ "image/modem.b17", 0x61110, 209, 1 },
	{ "image/modem.b18", 0x2d29a9, 210, 1 },
	{ "image/modem.b19", 0x2d94cc, 211, 1 },
	{ "image/modem.b22", 0x10ec0, 212, 1 },
	{ "image/modem.b23", 0x1722040, 213, 1 },
	{ "image/modem.b24", 0xa07900, 214, 1 },
	{ "image/modem.b25", 0x77a45, 215, 1 },
	{ "image/modem.b26", 0x15104, 216, 1 },
	{ "image/modem.b27", 0x1378, 217, 1 },
	{ "image/modem.b28", 0xb0, 218, 1 },
	{ "image/modem.b29", 0x7d52c, 219, 1 },
	{ "image/modem.mdt", 0x238c, 220, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_hw/generic/common/divar/cmcc_sub/sr_dsds/mcfg_hw.mbn", 0x588c, 221, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_hw/generic/common/divar/la/7+7_mode/sr_dsds/mcfg_hw.mbn", 0x4404, 222, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_hw/generic/common/divar/la/ss/mcfg_hw.mbn", 0x402c, 223, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_hw/mbn_hw.dig", 0x20, 224, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_hw/mbn_hw.txt", 0x232, 225, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_hw/oem_hw.txt", 0x1, 226, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/af/cellc/commerci/safrica/mcfg_sw.mbn", 0xb4ac, 227, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/af/kenya/airtel/commerci/mcfg_sw.mbn", 0xb838, 228, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/af/morocco/inwi/commerci/mcfg_sw.mbn", 0xb7d8, 229, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/af/morocco/orange/commerci/mcfg_sw.mbn", 0xbdc8, 230, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/af/moroccot/commerci/ma/mcfg_sw.mbn", 0xb50c, 231, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/airtel/commerci/mcfg_sw.mbn", 0xc048, 232, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/asiacell/commerci/mcfg_sw.mbn", 0xc16c, 233, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/bakcell/commerci/mcfg_sw.mbn", 0xc328, 234, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/beeline/commerci/mcfg_sw.mbn", 0xbf28, 235, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/beeline/uzbe/commerci/mcfg_sw.mbn", 0xc418, 236, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/cellcom/commerci/israel/mcfg_sw.mbn", 0xc2bc, 237, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/dcm/commerci/mcfg_sw.mbn", 0xbb40, 238, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/dialog/commerci/sl/mcfg_sw.mbn", 0xbd1c, 239, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/grameenp/commerci/mcfg_sw.mbn", 0xbc08, 240, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/htmobile/commerci/mcfg_sw.mbn", 0xc464, 241, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/idea/commerci/mcfg_sw.mbn", 0xbc00, 242, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/jazz/commerci/mcfg_sw.mbn", 0xb7cc, 243, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/jtl/commerci/mcfg_sw.mbn", 0xb898, 244, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/kddi/commerci/mcfg_sw.mbn", 0xeb6c, 245, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/kenya/commerci/mcfg_sw.mbn", 0xc05c, 246, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/mobitel/commerci/mcfg_sw.mbn", 0xb964, 247, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/ncell/commerci/nepal/mcfg_sw.mbn", 0xb948, 248, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/ntc/commerci/nepal/mcfg_sw.mbn", 0xbd34, 249, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/ooredoo/commerci/mcfg_sw.mbn", 0xaff0, 250, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/partner/commerci/mcfg_sw.mbn", 0xbedc, 251, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/pelephon/commerci/israel/mcfg_sw.mbn", 0xc240, 252, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/rakuten/commerci/mcfg_sw.mbn", 0xf490, 253, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/reliance/commerci/mcfg_sw.mbn", 0x1011c, 254, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/robi/commerci/mcfg_sw.mbn", 0xb388, 255, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/sbm/commerci/mcfg_sw.mbn", 0xb8b8, 256, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/tele2/commerci/mcfg_sw.mbn", 0xbc40, 257, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/telenor/commerci/mcfg_sw.mbn", 0xb8d8, 258, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/ucell/commerci/mcfg_sw.mbn", 0xb80c, 259, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/ufone/commerci/mcfg_sw.mbn", 0xc2dc, 260, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/uzmobile/commerci/mcfg_sw.mbn", 0xb7dc, 261, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/zain/commerci/mcfg_sw.mbn", 0xb940, 262, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/zong/commerci/mcfg_sw.mbn", 0xbcf4, 263, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/aunz/optus/commerci/au/mcfg_sw.mbn", 0xc4a4, 264, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/aunz/telstra/commerci/mcfg_sw.mbn", 0xc794, 265, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/aunz/vodafone/commerci/mcfg_sw.mbn", 0xc148, 266, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/cmcc/commerci/volte_cm/mcfg_sw.mbn", 0xd030, 267, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/cmcc/commerci/volte_op/mcfg_sw.mbn", 0xe098, 268, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/cmcc/lab/agnss_lo/mcfg_sw.mbn", 0x6c5c, 269, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/cmcc/lab/conf_vol/mcfg_sw.mbn", 0xcc88, 270, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/cmcc/lab/eps_only/mcfg_sw.mbn", 0x5eb0, 271, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/cmcc/lab/lpp_loct/mcfg_sw.mbn", 0x6c64, 272, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/cmcc/lab/nsiot_4r/mcfg_sw.mbn", 0xc664, 273, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/cmcc/lab/nsiot_vo/mcfg_sw.mbn", 0xc7e4, 274, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/cmcc/lab/rrlp_loc/mcfg_sw.mbn", 0x6c64, 275, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/cmcc/lab/tgl_comb/mcfg_sw.mbn", 0x5b60, 276, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/cmcc/lab/w_irat_c/mcfg_sw.mbn", 0x5e04, 277, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/ct/commerci/hvolte_o/mcfg_sw.mbn", 0x119bc, 278, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/ct/commerci/openmkt/mcfg_sw.mbn", 0xa698, 279, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/ct/commerci/volte_op/mcfg_sw.mbn", 0x11c50, 280, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/ct/lab/cta/mcfg_sw.mbn", 0x7618, 281, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/ct/lab/eps_only/mcfg_sw.mbn", 0xe4c0, 282, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/ct/lab/noapn_vo/mcfg_sw.mbn", 0xe120, 283, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/ct/lab/test/mcfg_sw.mbn", 0x9fa0, 284, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/ct/lab/test_eps/mcfg_sw.mbn", 0xa060, 285, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/ct/lab/test_no_/mcfg_sw.mbn", 0x9cc4, 286, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/ct/lab/volte_co/mcfg_sw.mbn", 0xe350, 287, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/ctm/commerci/mcfg_sw.mbn", 0xbcc0, 288, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/cu/commerci/openmkt/mcfg_sw.mbn", 0x690c, 289, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/cu/commerci/volte/mcfg_sw.mbn", 0xd604, 290, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/cu/lab/test/mcfg_sw.mbn", 0x52f4, 291, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/common/row/commerci/mcfg_sw.mbn", 0xb7f4, 292, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/common/test/commerci/mcfg_sw.mbn", 0xc568, 293, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/common/w_one/commerci/mcfg_sw.mbn", 0xab94, 294, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/1and1/commerci/de/mcfg_sw.mbn", 0xc878, 295, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/2degress/commerci/mcfg_sw.mbn", 0xbbc0, 296, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/4ka/commerci/slovenia/mcfg_sw.mbn", 0xbbf0, 297, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/a1/at_comme/mcfg_sw.mbn", 0xb824, 298, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/a1/bulgaria/mcfg_sw.mbn", 0xb90c, 299, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/a1/commerci/mcfg_sw.mbn", 0xbbfc, 300, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/a1/commerci/slovenia/mcfg_sw.mbn", 0xc304, 301, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/a1/croatia_/mcfg_sw.mbn", 0xbeec, 302, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/a1/macedoni/mcfg_sw.mbn", 0xbe0c, 303, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/a1/serbia_c/mcfg_sw.mbn", 0xb88c, 304, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/bh/commerci/ba/mcfg_sw.mbn", 0xb8f4, 305, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/bite/commerci/latvia/mcfg_sw.mbn", 0xbbe0, 306, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/bite/commerci/lithuani/mcfg_sw.mbn", 0xbbf8, 307, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/bouygues/commerci/france/mcfg_sw.mbn", 0xc734, 308, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/btbd/commerci/mcfg_sw.mbn", 0xb854, 309, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/coop/commerci/italy/mcfg_sw.mbn", 0xc7c4, 310, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/cosmote/commerci/mcfg_sw.mbn", 0xd034, 311, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/digi/commerci/hungary/mcfg_sw.mbn", 0xc4c8, 312, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/digi/commerci/spain/mcfg_sw.mbn", 0xbf00, 313, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/digi/romania_/mcfg_sw.mbn", 0xb448, 314, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/dt/commerci/austria/mcfg_sw.mbn", 0xcd68, 315, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/dt/commerci/croatia/mcfg_sw.mbn", 0xcac8, 316, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/dt/commerci/cz/mcfg_sw.mbn", 0xccfc, 317, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/dt/commerci/greece/mcfg_sw.mbn", 0xd02c, 318, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/dt/commerci/hungary/mcfg_sw.mbn", 0xd688, 319, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/dt/commerci/macedoni/mcfg_sw.mbn", 0xbff8, 320, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/dt/commerci/nl/mcfg_sw.mbn", 0xca3c, 321, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/dt/commerci/pl/mcfg_sw.mbn", 0xcdc4, 322, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/dt/commerci/slovakia/mcfg_sw.mbn", 0xc82c, 323, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/dt/non_volt/commerci/mcfg_sw.mbn", 0x5b58, 324, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/dt/volte/commerci/mcfg_sw.mbn", 0xca60, 325, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/dt/volte/romania_/mcfg_sw.mbn", 0xca98, 326, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/du/commerci/uae/mcfg_sw.mbn", 0xbb48, 327, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/ee/commerci/mcfg_sw.mbn", 0xc688, 328, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/eir/commerci/ireland/mcfg_sw.mbn", 0xb7d8, 329, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/elisa/commerci/estonia/mcfg_sw.mbn", 0xbdf0, 330, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/elisa/commerci/fi/mcfg_sw.mbn", 0xbfe0, 331, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/enetwork/commerci/mcfg_sw.mbn", 0xbc40, 332, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/etisalat/commerci/egypt/mcfg_sw.mbn", 0xb944, 333, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/etisalat/commerci/uae/mcfg_sw.mbn", 0xbecc, 334, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/fastweb/commerci/italy/mcfg_sw.mbn", 0xd440, 335, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/free/volte/france/mcfg_sw.mbn", 0xbac8, 336, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/gibtel/commerci/gibra/mcfg_sw.mbn", 0xba58, 337, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/h3g/commerci/austria/mcfg_sw.mbn", 0xb954, 338, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/h3g/commerci/denmark/mcfg_sw.mbn", 0xbf58, 339, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/h3g/commerci/ireland/mcfg_sw.mbn", 0xbec4, 340, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/h3g/commerci/italy/mcfg_sw.mbn", 0xd450, 341, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/h3g/commerci/se/mcfg_sw.mbn", 0xc0d0, 342, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/h3g/commerci/uk/mcfg_sw.mbn", 0xbba4, 343, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/ice/commerci/norway/mcfg_sw.mbn", 0xc1cc, 344, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/kpn/commerci/mcfg_sw.mbn", 0xc03c, 345, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/kyvistar/volte/ukraine/mcfg_sw.mbn", 0xc018, 346, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/lebara/commerci/france/mcfg_sw.mbn", 0xbce8, 347, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/lifecell/volte/ukraine/mcfg_sw.mbn", 0xc0cc, 348, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/lmt/commerci/latvia/mcfg_sw.mbn", 0xb5e4, 349, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/meo/commerci/mcfg_sw.mbn", 0xc828, 350, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/moldcell/commerci/moldova/mcfg_sw.mbn", 0xb3fc, 351, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/mtel/commerci/ba/mcfg_sw.mbn", 0xb8f4, 352, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/mtn/commerci/ng/mcfg_sw.mbn", 0xc190, 353, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/mtn/commerci/safrica/mcfg_sw.mbn", 0xc94c, 354, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/mts/commerci/armenia/mcfg_sw.mbn", 0xbf18, 355, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/mts/commerci/belarus/mcfg_sw.mbn", 0xbb1c, 356, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/mts/volte/serbia/mcfg_sw.mbn", 0xb95c, 357, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/nos/commerci/mcfg_sw.mbn", 0xbb00, 358, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/nova/commerci/mcfg_sw.mbn", 0xce10, 359, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/o2/commerci/czech/mcfg_sw.mbn", 0xc678, 360, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/o2/commerci/slovakia/mcfg_sw.mbn", 0xc7e4, 361, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/o2/uk_comme/mcfg_sw.mbn", 0xc5dc, 362, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/ora/commerci/france/mcfg_sw.mbn", 0xb8b8, 363, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/orange/commerci/belgium/mcfg_sw.mbn", 0xbf04, 364, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/orange/commerci/caraibes/mcfg_sw.mbn", 0xb8fc, 365, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/orange/commerci/egypt/mcfg_sw.mbn", 0xc1d0, 366, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/orange/commerci/france/mcfg_sw.mbn", 0xd5ac, 367, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/orange/commerci/group_no/mcfg_sw.mbn", 0x6180, 368, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/orange/commerci/luxembou/mcfg_sw.mbn", 0xb84c, 369, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/orange/commerci/poland/mcfg_sw.mbn", 0xdc88, 370, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/orange/commerci/reunion/mcfg_sw.mbn", 0xb8f8, 371, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/orange/commerci/romania/mcfg_sw.mbn", 0xc9c0, 372, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/orange/commerci/slovakia/mcfg_sw.mbn", 0xc5d0, 373, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/orange/commerci/spain/mcfg_sw.mbn", 0xcbb4, 374, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/orange/moldova_/mcfg_sw.mbn", 0xaf7c, 375, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/play/poland_c/mcfg_sw.mbn", 0xcc50, 376, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/plintron/commerci/italy/mcfg_sw.mbn", 0xb8cc, 377, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/plus/poland_c/mcfg_sw.mbn", 0xcf44, 378, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/post/commerci/luxembou/mcfg_sw.mbn", 0xb9d8, 379, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/proximus/commerci/mcfg_sw.mbn", 0xbc4c, 380, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/proximus/volte/luxembou/mcfg_sw.mbn", 0xb8e8, 381, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/salt/commerci/mcfg_sw.mbn", 0xc568, 382, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/sfr/commerci/altice/mcfg_sw.mbn", 0xbe88, 383, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/sfr/commerci/fr/mcfg_sw.mbn", 0xbd30, 384, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/spark/commerci/mcfg_sw.mbn", 0xc150, 385, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/spusu/commerci/at/mcfg_sw.mbn", 0xbaf8, 386, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/spusu/commerci/it/mcfg_sw.mbn", 0xbcf8, 387, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/spusu/commerci/uk/mcfg_sw.mbn", 0xbde0, 388, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/sunrise/commerci/mcfg_sw.mbn", 0xb8c8, 389, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/swisscom/commerci/swiss/mcfg_sw.mbn", 0xc2d0, 390, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/tdc/commerci/denmark/mcfg_sw.mbn", 0xbd74, 391, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/tele2/commerci/estonia/mcfg_sw.mbn", 0xc7e0, 392, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/tele2/commerci/latvia/mcfg_sw.mbn", 0xc7dc, 393, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/tele2/commerci/lithuani/mcfg_sw.mbn", 0xc800, 394, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/tele2/commerci/nl/mcfg_sw.mbn", 0xbf6c, 395, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/tele2/commerci/sweden/mcfg_sw.mbn", 0xb4b8, 396, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telefoni/commerci/de/mcfg_sw.mbn", 0xbff8, 397, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telefoni/commerci/spain/mcfg_sw.mbn", 0xc9a0, 398, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telefoni/commerci/uk/mcfg_sw.mbn", 0xbd20, 399, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telefoni/non_volt/spain/mcfg_sw.mbn", 0xc8a4, 400, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telekom/commerci/argentin/mcfg_sw.mbn", 0xb288, 401, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telekom/commerci/slovenia/mcfg_sw.mbn", 0xbdb0, 402, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telemach/commerci/mcfg_sw.mbn", 0xb9fc, 403, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telemach/hr_comme/mcfg_sw.mbn", 0xb4a0, 404, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telenet/volte/belgium/mcfg_sw.mbn", 0xbb18, 405, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telenor/commerci/bulgaria/mcfg_sw.mbn", 0xb6c8, 406, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telenor/commerci/denmark/mcfg_sw.mbn", 0xc754, 407, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telenor/commerci/finland/mcfg_sw.mbn", 0xb9a8, 408, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telenor/commerci/hungary/mcfg_sw.mbn", 0xbda8, 409, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telenor/commerci/monteneg/mcfg_sw.mbn", 0xb8b8, 410, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telenor/commerci/myanmar/mcfg_sw.mbn", 0xb524, 411, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telenor/commerci/norway/mcfg_sw.mbn", 0xbf44, 412, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telenor/commerci/serbia/mcfg_sw.mbn", 0xc224, 413, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telenor/commerci/sweden/mcfg_sw.mbn", 0xbf18, 414, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telia/commerci/denmark/mcfg_sw.mbn", 0xc478, 415, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telia/commerci/estonia/mcfg_sw.mbn", 0xc2b0, 416, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telia/commerci/finland/mcfg_sw.mbn", 0xc7fc, 417, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telia/commerci/lithuani/mcfg_sw.mbn", 0xc314, 418, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telia/commerci/norway/mcfg_sw.mbn", 0xcb54, 419, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telia/commerci/sweden/mcfg_sw.mbn", 0xca0c, 420, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telkom/commerci/mcfg_sw.mbn", 0xc144, 421, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/tim/commerci/italy/mcfg_sw.mbn", 0xc8d4, 422, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/tim/commerci/kena/mcfg_sw.mbn", 0xa828, 423, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/tim/commerci/welcome/mcfg_sw.mbn", 0xa7a8, 424, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/tmo/poland_c/mcfg_sw.mbn", 0xb924, 425, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/turkcell/commerci/mcfg_sw.mbn", 0xbb10, 426, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/turktele/commerci/mcfg_sw.mbn", 0xb9c4, 427, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vee/commerci/mcfg_sw.mbn", 0xc0dc, 428, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vimla/commerci/mcfg_sw.mbn", 0xbf50, 429, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/virgin/commerci/uk/mcfg_sw.mbn", 0xbb34, 430, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vivacom/commerci/mcfg_sw.mbn", 0xc2cc, 431, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/commerci/albania/mcfg_sw.mbn", 0xbf60, 432, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/commerci/hungary/mcfg_sw.mbn", 0xb818, 433, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/commerci/ireland/mcfg_sw.mbn", 0xc6b4, 434, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/india_vo/mcfg_sw.mbn", 0xb91c, 435, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/non_volt/mcfg_sw.mbn", 0x5f6c, 436, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/volte/ctc/mcfg_sw.mbn", 0xc1cc, 437, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/volte/cz/mcfg_sw.mbn", 0xcee0, 438, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/volte/egypt/mcfg_sw.mbn", 0xc6e4, 439, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/volte/germany/mcfg_sw.mbn", 0xce40, 440, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/volte/gr/mcfg_sw.mbn", 0xc720, 441, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/volte/italy/mcfg_sw.mbn", 0xab68, 442, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/volte/netherla/mcfg_sw.mbn", 0xcca4, 443, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/volte/portugal/mcfg_sw.mbn", 0xab60, 444, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/volte/ro/mcfg_sw.mbn", 0xc118, 445, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/volte/safrica/mcfg_sw.mbn", 0xc360, 446, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/volte/spain/mcfg_sw.mbn", 0xcc98, 447, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/volte/turkey/mcfg_sw.mbn", 0xc384, 448, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/volte/uk/mcfg_sw.mbn", 0xcec8, 449, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/we/commerci/egypt/mcfg_sw.mbn", 0xbdbc, 450, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/wind/commerci/mcfg_sw.mbn", 0xc5e0, 451, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/yoigo/commerci/mcfg_sw.mbn", 0xb708, 452, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/zeop/commerci/mcfg_sw.mbn", 0xc064, 453, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/korea/kt/commerci/mcfg_sw.mbn", 0xb368, 454, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/korea/lgu/commerci/mcfg_sw.mbn", 0xaee4, 455, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/korea/skt/commerci/mcfg_sw.mbn", 0xa210, 456, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/korea/tta/commerci/mcfg_sw.mbn", 0x9bf4, 457, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/amx/commerci/mx/mcfg_sw.mbn", 0xc238, 458, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/amx/non_volt/mcfg_sw.mbn", 0x393c, 459, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/amx/volte/mcfg_sw.mbn", 0xc638, 460, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/antel/commerci/mcfg_sw.mbn", 0xbd84, 461, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/brisanet/commerci/brazil/mcfg_sw.mbn", 0xbcb4, 462, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/claro/commerci/arg/mcfg_sw.mbn", 0xd9f0, 463, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/claro/commerci/brazil/mcfg_sw.mbn", 0xda10, 464, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/claro/commerci/chile/mcfg_sw.mbn", 0xe430, 465, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/claro/commerci/colombia/mcfg_sw.mbn", 0xebf0, 466, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/claro/commerci/cri/mcfg_sw.mbn", 0xda28, 467, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/claro/commerci/dominica/mcfg_sw.mbn", 0xda40, 468, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/claro/commerci/ecuador/mcfg_sw.mbn", 0xdd08, 469, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/claro/commerci/guatemal/mcfg_sw.mbn", 0xda34, 470, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/claro/commerci/honduras/mcfg_sw.mbn", 0xda2c, 471, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/claro/commerci/nicaragu/mcfg_sw.mbn", 0xda30, 472, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/claro/commerci/panama/mcfg_sw.mbn", 0xd9d4, 473, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/claro/commerci/paraguay/mcfg_sw.mbn", 0xd8b8, 474, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/claro/commerci/peru/mcfg_sw.mbn", 0xeba8, 475, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/claro/commerci/puerto/mcfg_sw.mbn", 0xd9cc, 476, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/claro/commerci/salvador/mcfg_sw.mbn", 0xda2c, 477, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/claro/commerci/uruguay/mcfg_sw.mbn", 0xd980, 478, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/entel/commerci/chile/mcfg_sw.mbn", 0xfb3c, 479, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/entel/commerci/peru/mcfg_sw.mbn", 0xefa0, 480, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/liberty/commerci/pa/mcfg_sw.mbn", 0xbc00, 481, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/movistar/argentin/mcfg_sw.mbn", 0xb4b8, 482, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/movistar/chile_co/mcfg_sw.mbn", 0xb7fc, 483, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/movistar/ecuador_/mcfg_sw.mbn", 0xb578, 484, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/movistar/mx_comme/mcfg_sw.mbn", 0xb5fc, 485, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/movistar/panama_c/mcfg_sw.mbn", 0xbd6c, 486, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/movistar/uruguay_/mcfg_sw.mbn", 0xb508, 487, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/mx-att/commerci/mcfg_sw.mbn", 0xe110, 488, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/telefoni/commerci/colombia/mcfg_sw.mbn", 0xbd1c, 489, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/telefoni/commerci/peru/mcfg_sw.mbn", 0xe0c8, 490, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/tigo/commerci/guatemal/mcfg_sw.mbn", 0xbb24, 491, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/tigo/commerci/mcfg_sw.mbn", 0xba68, 492, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/tim/commerci/brazil/mcfg_sw.mbn", 0xa2c0, 493, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/unifique/commerci/brazil/mcfg_sw.mbn", 0xbf00, 494, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/vivo/commerci/brazil/mcfg_sw.mbn", 0xc10c, 495, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/wom/commerci/colombia/mcfg_sw.mbn", 0xc114, 496, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/wom/commerci/mcfg_sw.mbn", 0xb014, 497, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/azerbaij/commerci/mcfg_sw.mbn", 0xc7d0, 498, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/bahrain/bahrain_/mcfg_sw.mbn", 0xb7f0, 499, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/bahrain/betelco_/mcfg_sw.mbn", 0xb7f0, 500, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/bahrain/viva/commerci/mcfg_sw.mbn", 0xb79c, 501, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/bahrain/zain/commerci/mcfg_sw.mbn", 0xb79c, 502, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/cyta/commerci/mcfg_sw.mbn", 0xbab4, 503, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/epic/commerci/mcfg_sw.mbn", 0xb8a0, 504, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/jordan/orange/commerci/mcfg_sw.mbn", 0xb9f0, 505, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/jordan/umniah/commerci/mcfg_sw.mbn", 0xb958, 506, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/kuwait/ooredoo/commerci/mcfg_sw.mbn", 0xbb10, 507, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/kuwait/viva/commerci/mcfg_sw.mbn", 0xbd94, 508, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/kuwait/zain/commerci/mcfg_sw.mbn", 0xbd28, 509, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/oman/omantel/commerci/mcfg_sw.mbn", 0xb798, 510, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/oman/ooredoo/commerci/mcfg_sw.mbn", 0xb8dc, 511, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/oman/vodafone/commerci/mcfg_sw.mbn", 0xc2fc, 512, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/primetel/commerci/mcfg_sw.mbn", 0xb8b0, 513, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/qatar/commerci/mcfg_sw.mbn", 0xb420, 514, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/saudi/mobily/commerci/mcfg_sw.mbn", 0xbc18, 515, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/saudi/salam/commerci/mcfg_sw.mbn", 0xb61c, 516, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/saudi/zain/commerci/mcfg_sw.mbn", 0xb7e4, 517, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/stc/commerci/sa/mcfg_sw.mbn", 0xb8d4, 518, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mexico/altan/commerci/mcfg_sw.mbn", 0xb948, 519, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mexico/axtel/commerci/mcfg_sw.mbn", 0xb928, 520, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/att/american/mcfg_sw.mbn", 0xd5fc, 521, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/att/firstnet/mcfg_sw.mbn", 0xd4a0, 522, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/att/guam_cnm/mcfg_sw.mbn", 0xd5e4, 523, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/att/non_volt/mcfg_sw.mbn", 0x61b8, 524, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/att/volte/mcfg_sw.mbn", 0xd490, 525, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/bell/commerci/ca/mcfg_sw.mbn", 0xc9c8, 526, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/cbrs/commerci/us/mcfg_sw.mbn", 0x5454, 527, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/dish/commerci/us/mcfg_sw.mbn", 0xbb7c, 528, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/rogers/commerci/ca/mcfg_sw.mbn", 0xbff0, 529, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/sprint/commerci/mcfg_sw.mbn", 0xf0dc, 530, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/sprint/vowifi/mcfg_sw.mbn", 0xf0d8, 531, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/telus/commerci/ca/mcfg_sw.mbn", 0xc434, 532, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/tmo/commerci/mcfg_sw.mbn", 0xd210, 533, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/tmo/poland_c/mcfg_sw.mbn", 0xba04, 534, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/uscc/commerci/mcfg_sw.mbn", 0xc850, 535, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/verizon/cdmaless/mcfg_sw.mbn", 0x116e8, 536, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/verizon/hvolte/mcfg_sw.mbn", 0x132dc, 537, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/verizon/imsless/mcfg_sw.mbn", 0x93f0, 538, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/russia/beeline/gen_3gpp/mcfg_sw.mbn", 0xcec8, 539, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/russia/megafon/commerci/ru/mcfg_sw.mbn", 0xc97c, 540, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/russia/motiv/commerci/mcfg_sw.mbn", 0xb79c, 541, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/russia/mts/commerci/mcfg_sw.mbn", 0xb8d8, 542, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/russia/mts/commerci/ru/mcfg_sw.mbn", 0xb964, 543, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/russia/sberbank/commerci/mcfg_sw.mbn", 0xbdc8, 544, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/russia/tele2/commerci/mcfg_sw.mbn", 0xc778, 545, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sa/brazil/commerci/mcfg_sw.mbn", 0x32cc, 546, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/3hk/commerci/hk/mcfg_sw.mbn", 0xbed0, 547, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/airtel/commerci/lanka/mcfg_sw.mbn", 0xb904, 548, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/ais/commerci/thailand/mcfg_sw.mbn", 0xb850, 549, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/apt/commerci/mcfg_sw.mbn", 0xbaa4, 550, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/celcom/commerci/mcfg_sw.mbn", 0xbdec, 551, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/chunghwa/commerci/tw/mcfg_sw.mbn", 0xc99c, 552, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/cuhk/commerci/mcfg_sw.mbn", 0xb3dc, 553, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/digi/commerci/mcfg_sw.mbn", 0xca88, 554, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/dito/commerci/ph/mcfg_sw.mbn", 0xc0b0, 555, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/dtac/commerci/th/mcfg_sw.mbn", 0xbdd0, 556, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/dtac/commerci/volte/mcfg_sw.mbn", 0xbdd0, 557, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/fareasto/commerci/mcfg_sw.mbn", 0xbee4, 558, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/globe/commerci/ph/mcfg_sw.mbn", 0xbdc0, 559, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/hkt/commerci/hk/mcfg_sw.mbn", 0xb8f0, 560, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/indosat/commerci/mcfg_sw.mbn", 0xba58, 561, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/m1/commerci/sg/mcfg_sw.mbn", 0xc208, 562, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/maxis/commerci/malaysia/mcfg_sw.mbn", 0xb250, 563, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/metfone/commerci/mcfg_sw.mbn", 0xbba4, 564, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/mobifone/commerci/mcfg_sw.mbn", 0xc720, 565, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/mpt/commerci/myanmar/mcfg_sw.mbn", 0xb428, 566, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/mytel/commerci/myanmar/mcfg_sw.mbn", 0xbbb8, 567, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/nt/commerci/thailand/mcfg_sw.mbn", 0xc364, 568, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/p1/commerci/malaysia/mcfg_sw.mbn", 0xbb8c, 569, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/seatel/commerci/mcfg_sw.mbn", 0xd70c, 570, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/singtel/commerci/singapor/mcfg_sw.mbn", 0xc6e4, 571, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/smart/commerci/mcfg_sw.mbn", 0xb950, 572, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/smart_ph/commerci/mcfg_sw.mbn", 0xb938, 573, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/smartfre/commerci/mcfg_sw.mbn", 0xc2a4, 574, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/smartfre/commerci/vowifi/mcfg_sw.mbn", 0xc254, 575, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/smartone/commerci/hk/mcfg_sw.mbn", 0xb9a4, 576, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/starhub/commerci/sg/mcfg_sw.mbn", 0xb9c8, 577, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/telkomse/commerci/mcfg_sw.mbn", 0xc388, 578, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/tm/commerci/mcfg_sw.mbn", 0xbd54, 579, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/tpg/commerci/tpg/mcfg_sw.mbn", 0xbcb8, 580, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/tri/commerci/mcfg_sw.mbn", 0xbb04, 581, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/truemove/commerci/thailand/mcfg_sw.mbn", 0xc010, 582, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/truemove/commerci/volte/mcfg_sw.mbn", 0xc030, 583, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/tstar/commerci/tw/mcfg_sw.mbn", 0xa408, 584, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/umobile/commerci/malaysia/mcfg_sw.mbn", 0xc5f0, 585, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/vietnamo/commerci/vietnamo/mcfg_sw.mbn", 0xb8f0, 586, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/viettel/commerci/vietnam/mcfg_sw.mbn", 0xbb30, 587, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/vinaphon/commerci/vietnam/mcfg_sw.mbn", 0xbdbc, 588, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/xl/commerci/mcfg_sw.mbn", 0xb990, 589, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/ytl/commerci/mcfg_sw.mbn", 0xbb84, 590, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/test/er_iot/mcfg_sw.mbn", 0xb7b0, 591, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/test/nk_iot/mcfg_sw.mbn", 0xb744, 592, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/mbn_sw.dig", 0x20, 593, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/mbn_sw.txt", 0x4a2c, 594, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/oem_eea.txt", 0x378d, 595, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/oem_sw.txt", 0x3ece, 596, 1 },
	{ "image/modemr.jsn", 0x2cf, 597, 1 },
	{ "image/modemuw.jsn", 0x2c9, 598, 1 },
	{ "image/otrpapp.b00", 0x134, 599, 1 },
	{ "image/otrpapp.b01", 0x1b28, 600, 1 },
	{ "image/otrpapp.b02", 0x4c0c0, 601, 1 },
	{ "image/otrpapp.b03", 0x8f, 602, 1 },
	{ "image/otrpapp.b04", 0x3b80, 603, 1 },
	{ "image/otrpapp.b05", 0x238, 604, 1 },
	{ "image/otrpapp.b06", 0x98, 605, 1 },
	{ "image/otrpapp.b07", 0xb3c3, 606, 1 },
	{ "image/otrpapp.mdt", 0x1c5c, 607, 1 },
	{ "image/qdsp6m.qdb", 0x3df2b6, 608, 1 },
	{ "image/rtic.mbn", 0x493ba, 609, 1 },
	{ "image/secauth.b00", 0x134, 610, 1 },
	{ "image/secauth.b01", 0x1b28, 611, 1 },
	{ "image/secauth.b02", 0x55b1, 612, 1 },
	{ "image/secauth.b03", 0x98, 613, 1 },
	{ "image/secauth.b04", 0x460, 614, 1 },
	{ "image/secauth.b05", 0xdc, 615, 1 },
	{ "image/secauth.b06", 0x98, 616, 1 },
	{ "image/secauth.b07", 0x102c, 617, 1 },
	{ "image/secauth.mdt", 0x1c5c, 618, 1 },
	{ "image/smplap32.b00", 0x134, 619, 1 },
	{ "image/smplap32.b01", 0x1b28, 620, 1 },
	{ "image/smplap32.b02", 0x43c7c, 621, 1 },
	{ "image/smplap32.b03", 0xcc, 622, 1 },
	{ "image/smplap32.b04", 0x1688bc, 623, 1 },
	{ "image/smplap32.b05", 0x4c0, 624, 1 },
	{ "image/smplap32.b06", 0x98, 625, 1 },
	{ "image/smplap32.b07", 0xb1a5, 626, 1 },
	{ "image/smplap32.mdt", 0x1c5c, 627, 1 },
	{ "image/smplap64.b00", 0x200, 628, 1 },
	{ "image/smplap64.b01", 0x1b28, 629, 1 },
	{ "image/smplap64.b02", 0x4ff70, 630, 1 },
	{ "image/smplap64.b03", 0xf8, 631, 1 },
	{ "image/smplap64.b04", 0x16b250, 632, 1 },
	{ "image/smplap64.b05", 0x980, 633, 1 },
	{ "image/smplap64.b06", 0x130, 634, 1 },
	{ "image/smplap64.b07", 0x1452b, 635, 1 },
	{ "image/smplap64.mdt", 0x1d28, 636, 1 },
	{ "image/soter64.b00", 0x200, 637, 1 },
	{ "image/soter64.b01", 0x1b28, 638, 1 },
	{ "image/soter64.b02", 0xcecd, 639, 1 },
	{ "image/soter64.b03", 0xc2, 640, 1 },
	{ "image/soter64.b04", 0x840, 641, 1 },
	{ "image/soter64.b05", 0x288, 642, 1 },
	{ "image/soter64.b06", 0x130, 643, 1 },
	{ "image/soter64.b07", 0x1b48, 644, 1 },
	{ "image/soter64.mdt", 0x1d28, 645, 1 },
	{ "image/vsimapp.b00", 0x134, 646, 1 },
	{ "image/vsimapp.b01", 0x1b28, 647, 1 },
	{ "image/vsimapp.b02", 0x7d3b, 648, 1 },
	{ "image/vsimapp.b03", 0xbb, 649, 1 },
	{ "image/vsimapp.b04", 0x334, 650, 1 },
	{ "image/vsimapp.b05", 0x10c, 651, 1 },
	{ "image/vsimapp.b06", 0x98, 652, 1 },
	{ "image/vsimapp.b07", 0x1510, 653, 1 },
	{ "image/vsimapp.mdt", 0x1c5c, 654, 1 },
	{ "image/widevine.mbn", 0x2ecfa, 655, 1 },
	{ "image/wlanmdsp.mbn", 0x3a39c0, 656, 1 },
};

static const struct fw_extent fw_ext_sapphire[] = {
	{ 0x15000, 0x193 },
	{ 0x19000, 0x195 },
	{ 0x1d000, 0x22b },
	{ 0x21000, 0x193 },
	{ 0x25000, 0x4d4 },
	{ 0x29000, 0x2098 },
	{ 0x2d000, 0x14ef4 },
	{ 0x45000, 0x3e9c0 },
	{ 0x85000, 0x18f288 },
	{ 0x215000, 0x19f620 },
	{ 0x3b5000, 0xb60 },
	{ 0x3b9000, 0x6354 },
	{ 0x3c1000, 0x6f6c },
	{ 0x3c9000, 0x12dc },
	{ 0x3cd000, 0x104 },
	{ 0x3d1000, 0xf2a0 },
	{ 0x3e1000, 0x11b58 },
	{ 0x3f5000, 0xc4000 },
	{ 0x4b9000, 0x1837e4 },
	{ 0x63d000, 0x44004 },
	{ 0x685000, 0x342b },
	{ 0x689000, 0x1b60 },
	{ 0x68d000, 0x2dc },
	{ 0x691000, 0x2140 },
	{ 0x695000, 0x6fec },
	{ 0x69d000, 0x3a0ec },
	{ 0x6d9000, 0x80000 },
	{ 0x759000, 0x55a23e },
	{ 0xcb5000, 0x1c513a },
	{ 0xe7d000, 0x561c },
	{ 0xe85000, 0x53a74 },
	{ 0xed9000, 0xc08 },
	{ 0xedd000, 0xd6e28 },
	{ 0xfb5000, 0x50230 },
	{ 0x1009000, 0xe00 },
	{ 0x100d000, 0x13f0 },
	{ 0x1011000, 0x23880 },
	{ 0x1035000, 0x173a70 },
	{ 0x11a9000, 0xbc },
	{ 0x11ad000, 0x256c },
	{ 0x11b1000, 0x134 },
	{ 0x11b5000, 0x1b28 },
	{ 0x11b9000, 0x97d3 },
	{ 0x11c5000, 0x91 },
	{ 0x11c9000, 0x2b8 },
	{ 0x11cd000, 0x14c },
	{ 0x11d1000, 0x98 },
	{ 0x11d5000, 0x1cab },
	{ 0x11d9000, 0x1c5c },
	{ 0x11dd000, 0x1d4 },
	{ 0x11e1000, 0x1c18 },
	{ 0x11e5000, 0x39898 },
	{ 0x1221000, 0x1a07c },
	{ 0x123d000, 0x44c },
	{ 0x1241000, 0x20db5c },
	{ 0x1451000, 0x5b60 },
	{ 0x1459000, 0x95378 },
	{ 0x14f1000, 0xa20 },
	{ 0x14f5000, 0xe614 },
	{ 0x1505000, 0x1dec },
	{ 0x1509000, 0x200 },
	{ 0x150d000, 0x4e70 },
	{ 0x1515000, 0xa88d },
	{ 0x1521000, 0xc4 },
	{ 0x1525000, 0x391 },
	{ 0x1529000, 0x1c0 },
	{ 0x152d000, 0x130 },
	{ 0x1531000, 0x2231 },
	{ 0x1535000, 0x5070 },
	{ 0x153d000, 0x200 },
	{ 0x1541000, 0x1b28 },
	{ 0x1545000, 0x1a3a43 },
	{ 0x16e9000, 0xc3 },
	{ 0x16ed000, 0x14f6dc },
	{ 0x183d000, 0x4e0 },
	{ 0x1841000, 0x130 },
	{ 0x1845000, 0x1ad3d },
	{ 0x1861000, 0x1d28 },
	{ 0x1865000, 0x200 },
	{ 0x1869000, 0x1b28 },
	{ 0x186d000, 0x6c26cc },
	{ 0x1f31000, 0xc7 },
	{ 0x1f35000, 0x3d6d8c },
	{ 0x230d000, 0x6c0 },
	{ 0x2311000, 0x150 },
	{ 0x2315000, 0x2e943 },
	{ 0x2345000, 0x1d28 },
	{ 0x2349000, 0x134 },
	{ 0x234d000, 0x1b28 },
	{ 0x2351000, 0x1430b },
	{ 0x2369000, 0xd4 },
	{ 0x236d000, 0x6bb4 },
	{ 0x2375000, 0x430 },
	{ 0x2379000, 0x98 },
	{ 0x237d000, 0x6c49 },
	{ 0x2385000, 0x1c5c },
	{ 0x2389000, 0x200 },
	{ 0x238d000, 0x1b28 },
	{ 0x2391000, 0x541d },
	{ 0x2399000, 0xc2 },
	{ 0x239d000, 0x388 },
	{ 0x23a1000, 0x1c0 },
	{ 0x23a5000, 0x130 },
	{ 0x23a9000, 0x137d },
	{ 0x23ad000, 0x1d28 },
	{ 0x23b1000, 0x200 },
	{ 0x23b5000, 0x1b28 },
	{ 0x23b9000, 0xc0f5 },
	{ 0x23c9000, 0xc5 },
	{ 0x23cd000, 0x4c60 },
	{ 0x23d5000, 0x180 },
	{ 0x23d9000, 0x130 },
	{ 0x23dd000, 0x10a9 },
	{ 0x23e1000, 0x1d28 },
	{ 0x23e5000, 0x134 },
	{ 0x23e9000, 0x1b28 },
	{ 0x23ed000, 0x2bb74 },
	{ 0x2419000, 0x89 },
	{ 0x241d000, 0xe47c },
	{ 0x242d000, 0x188 },
	{ 0x2431000, 0x98 },
	{ 0x2435000, 0x68ee },
	{ 0x243d000, 0x1c5c },
	{ 0x2441000, 0x200 },
	{ 0x2445000, 0x1b28 },
	{ 0x2449000, 0x1b20a },
	{ 0x2465000, 0xbe },
	{ 0x2469000, 0xc7c },
	{ 0x246d000, 0x298 },
	{ 0x2471000, 0x130 },
	{ 0x2475000, 0x3931 },
	{ 0x2479000, 0x1d28 },
	{ 0x247d000, 0x134 },
	{ 0x2481000, 0x1b28 },
	{ 0x2485000, 0xaf99 },
	{ 0x2491000, 0x8a },
	{ 0x2495000, 0xa88 },
	{ 0x2499000, 0x174 },
	{ 0x249d000, 0x98 },
	{ 0x24a1000, 0x231b },
	{ 0x24a5000, 0x1c5c },
	{ 0x24a9000, 0x414 },
	{ 0x24ad000, 0x1f78 },
	{ 0x24b1000, 0x2930 },
	{ 0x24b5000, 0x2518 },
	{ 0x24b9000, 0x4a3ac },
	{ 0x2505000, 0x119d8 },
	{ 0x2519000, 0x100c },
	{ 0x251d000, 0x90b4 },
	{ 0x2529000, 0x4f054 },
	{ 0x2579000, 0x143ee8 },
	{ 0x26bd000, 0x40000 },
	{ 0x26fd000, 0x540 },
	{ 0x2701000, 0xe2210 },
	{ 0x27e5000, 0x2d },
	{ 0x27e9000, 0x7c },
	{ 0x27ed000, 0x9c1a0 },
	{ 0x288d000, 0x61110 },
	{ 0x28f1000, 0x2d4079 },
	{ 0x2bc9000, 0x2da98c },
	{ 0x2ea5000, 0x10ec0 },
	{ 0x2eb9000, 0x1722040 },
	{ 0x45dd000, 0xa07900 },
	{ 0x4fe5000, 0x778fb },
	{ 0x505d000, 0x15104 },
	{ 0x5075000, 0x1378 },
	{ 0x5079000, 0xb0 },
	{ 0x507d000, 0x7d52c },
	{ 0x50fd000, 0x238c },
	{ 0x5101000, 0x134 },
	{ 0x5105000, 0x1b28 },
	{ 0x5109000, 0x4c0c0 },
	{ 0x5159000, 0x8f },
	{ 0x515d000, 0x3b80 },
	{ 0x5161000, 0x238 },
	{ 0x5165000, 0x98 },
	{ 0x5169000, 0xb3c3 },
	{ 0x5175000, 0x1c5c },
	{ 0x5179000, 0x134 },
	{ 0x517d000, 0x1b28 },
	{ 0x5181000, 0x55b1 },
	{ 0x5189000, 0x98 },
	{ 0x518d000, 0x460 },
	{ 0x5191000, 0xdc },
	{ 0x5195000, 0x98 },
	{ 0x5199000, 0x102c },
	{ 0x519d000, 0x1c5c },
	{ 0x51a1000, 0x134 },
	{ 0x51a5000, 0x1b28 },
	{ 0x51a9000, 0x43c7c },
	{ 0x51ed000, 0xcc },
	{ 0x51f1000, 0x1688bc },
	{ 0x535d000, 0x4c0 },
	{ 0x5361000, 0x98 },
	{ 0x5365000, 0xb1a5 },
	{ 0x5371000, 0x1c5c },
	{ 0x5375000, 0x200 },
	{ 0x5379000, 0x1b28 },
	{ 0x537d000, 0x4ff70 },
	{ 0x53cd000, 0xf8 },
	{ 0x53d1000, 0x16b250 },
	{ 0x553d000, 0x980 },
	{ 0x5541000, 0x130 },
	{ 0x5545000, 0x1452b },
	{ 0x555d000, 0x1d28 },
	{ 0x5561000, 0x200 },
	{ 0x5565000, 0x1b28 },
	{ 0x5569000, 0xcecd },
	{ 0x5579000, 0xc2 },
	{ 0x557d000, 0x840 },
	{ 0x5581000, 0x288 },
	{ 0x5585000, 0x130 },
	{ 0x5589000, 0x1b48 },
	{ 0x558d000, 0x1d28 },
	{ 0x5591000, 0x134 },
	{ 0x5595000, 0x1b28 },
	{ 0x5599000, 0x7d3b },
	{ 0x55a1000, 0xbb },
	{ 0x55a5000, 0x334 },
	{ 0x55a9000, 0x10c },
	{ 0x55ad000, 0x98 },
	{ 0x55b1000, 0x1510 },
	{ 0x55b5000, 0x1c5c },
	{ 0x55b9000, 0x3df1e1 },
	{ 0x5999000, 0x2cf },
	{ 0x59c5000, 0x4404 },
	{ 0x59d1000, 0x402c },
	{ 0x59e1000, 0x588c },
	{ 0x59e9000, 0x20 },
	{ 0x59ed000, 0x232 },
	{ 0x59f1000, 0x1 },
	{ 0x5a0d000, 0xb4ac },
	{ 0x5a25000, 0xb838 },
	{ 0x5a3d000, 0xb7d8 },
	{ 0x5a51000, 0xbdc8 },
	{ 0x5a69000, 0xb50c },
	{ 0x5a81000, 0xb804 },
	{ 0x5a99000, 0xc048 },
	{ 0x5ab1000, 0xc264 },
	{ 0x5ac9000, 0xc328 },
	{ 0x5ae1000, 0xb8f4 },
	{ 0x5af5000, 0xb958 },
	{ 0x5b09000, 0xc4ac },
	{ 0x5b25000, 0xc2c0 },
	{ 0x5b3d000, 0xbb40 },
	{ 0x5b55000, 0xbd1c },
	{ 0x5b69000, 0xbc08 },
	{ 0x5b7d000, 0xc464 },
	{ 0x5b95000, 0xbb8c },
	{ 0x5ba9000, 0xb898 },
	{ 0x5bbd000, 0xb7cc },
	{ 0x5bd1000, 0xeb6c },
	{ 0x5be9000, 0xc24c },
	{ 0x5c01000, 0xb964 },
	{ 0x5c19000, 0xbd34 },
	{ 0x5c31000, 0xb948 },
	{ 0x5c45000, 0xaffc },
	{ 0x5c59000, 0xbfb4 },
	{ 0x5c71000, 0xc248 },
	{ 0x5c89000, 0xf490 },
	{ 0x5ca1000, 0xff90 },
	{ 0x5cb9000, 0xb388 },
	{ 0x5ccd000, 0xb8b8 },
	{ 0x5ce1000, 0xbc40 },
	{ 0x5cf5000, 0xb8d8 },
	{ 0x5d09000, 0xb80c },
	{ 0x5d1d000, 0xc2e0 },
	{ 0x5d35000, 0xb7dc },
	{ 0x5d49000, 0xb940 },
	{ 0x5d5d000, 0xbd3c },
	{ 0x5d79000, 0xc4a4 },
	{ 0x5d91000, 0xc794 },
	{ 0x5da9000, 0xc148 },
	{ 0x5dc9000, 0xec94 },
	{ 0x5ddd000, 0xe56c },
	{ 0x5df5000, 0x7180 },
	{ 0x5e01000, 0xd1ac },
	{ 0x5e15000, 0x63d0 },
	{ 0x5e21000, 0x7184 },
	{ 0x5e2d000, 0xcb88 },
	{ 0x5e41000, 0xcd04 },
	{ 0x5e55000, 0x7184 },
	{ 0x5e61000, 0x5b60 },
	{ 0x5e6d000, 0x6324 },
	{ 0x5e81000, 0xa698 },
	{ 0x5e91000, 0x121b8 },
	{ 0x5ea9000, 0x11f24 },
	{ 0x5ec5000, 0x7618 },
	{ 0x5ed1000, 0xe9e0 },
	{ 0x5ee5000, 0xe640 },
	{ 0x5ef9000, 0x9fa0 },
	{ 0x5f09000, 0xa060 },
	{ 0x5f19000, 0x9cc4 },
	{ 0x5f29000, 0xe874 },
	{ 0x5f41000, 0xc068 },
	{ 0x5f5d000, 0x6e2c },
	{ 0x5f69000, 0xdb68 },
	{ 0x5f81000, 0x5814 },
	{ 0x5f91000, 0xb7f0 },
	{ 0x5fad000, 0xc838 },
	{ 0x5fc5000, 0xbbc0 },
	{ 0x5fdd000, 0xbbf0 },
	{ 0x5ff1000, 0xb824 },
	{ 0x6001000, 0xb914 },
	{ 0x6015000, 0xc2bc },
	{ 0x6025000, 0xbbdc },
	{ 0x6035000, 0xbf28 },
	{ 0x6045000, 0xbe0c },
	{ 0x6055000, 0xb88c },
	{ 0x606d000, 0xb954 },
	{ 0x6081000, 0xb854 },
	{ 0x6099000, 0xbbe0 },
	{ 0x60a9000, 0xbbf8 },
	{ 0x60c1000, 0xc668 },
	{ 0x60d9000, 0xb8a8 },
	{ 0x60ed000, 0xd050 },
	{ 0x6109000, 0xcd88 },
	{ 0x611d000, 0xcd54 },
	{ 0x6131000, 0xcae8 },
	{ 0x6145000, 0xd048 },
	{ 0x6159000, 0xd6a4 },
	{ 0x616d000, 0xc070 },
	{ 0x6181000, 0xca14 },
	{ 0x6195000, 0xcef4 },
	{ 0x61a9000, 0xc884 },
	{ 0x61c1000, 0x6090 },
	{ 0x61d1000, 0xcf20 },
	{ 0x61e5000, 0xcec8 },
	{ 0x6201000, 0xb914 },
	{ 0x6219000, 0xc570 },
	{ 0x622d000, 0xbf34 },
	{ 0x623d000, 0xb498 },
	{ 0x6251000, 0xc688 },
	{ 0x626d000, 0xb7d8 },
	{ 0x6285000, 0xbdf0 },
	{ 0x6295000, 0xbfe0 },
	{ 0x62a9000, 0xbc44 },
	{ 0x62c1000, 0xb9cc },
	{ 0x62d1000, 0xbecc },
	{ 0x62e9000, 0xd440 },
	{ 0x6305000, 0xbb7c },
	{ 0x631d000, 0xba58 },
	{ 0x6335000, 0xb954 },
	{ 0x6345000, 0xbf58 },
	{ 0x6355000, 0xbec4 },
	{ 0x6365000, 0xd474 },
	{ 0x6379000, 0xc09c },
	{ 0x638d000, 0xbba4 },
	{ 0x63a5000, 0xbb9c },
	{ 0x63b9000, 0xc110 },
	{ 0x63d5000, 0xca08 },
	{ 0x63f1000, 0xb65c },
	{ 0x6409000, 0xc138 },
	{ 0x6421000, 0xbfb0 },
	{ 0x6439000, 0xb920 },
	{ 0x6451000, 0xc190 },
	{ 0x6465000, 0xc948 },
	{ 0x6481000, 0xbf18 },
	{ 0x6491000, 0xbb1c },
	{ 0x64a5000, 0xb97c },
	{ 0x64bd000, 0xb3fc },
	{ 0x64d1000, 0xb9ec },
	{ 0x64e5000, 0xc894 },
	{ 0x6501000, 0xc678 },
	{ 0x6515000, 0xc260 },
	{ 0x6529000, 0xc5dc },
	{ 0x6545000, 0xb840 },
	{ 0x655d000, 0xbf04 },
	{ 0x656d000, 0xbf94 },
	{ 0x657d000, 0xc224 },
	{ 0x6591000, 0xd7a8 },
	{ 0x65a5000, 0x6180 },
	{ 0x65b1000, 0xb84c },
	{ 0x65c1000, 0xdd20 },
	{ 0x65d5000, 0xbbcc },
	{ 0x65e5000, 0xc9e4 },
	{ 0x65f9000, 0xc588 },
	{ 0x660d000, 0xcc44 },
	{ 0x6621000, 0xaf7c },
	{ 0x6635000, 0xcc54 },
	{ 0x6651000, 0xb8cc },
	{ 0x6665000, 0xd00c },
	{ 0x6681000, 0xb9d8 },
	{ 0x6695000, 0xbc74 },
	{ 0x66a9000, 0xb8e8 },
	{ 0x66c1000, 0xbe88 },
	{ 0x66d1000, 0xc298 },
	{ 0x66e9000, 0xc4c4 },
	{ 0x6701000, 0xbc74 },
	{ 0x6715000, 0xc150 },
	{ 0x6731000, 0xbaf8 },
	{ 0x6741000, 0xbcf8 },
	{ 0x6751000, 0xbde0 },
	{ 0x6765000, 0xb8cc },
	{ 0x677d000, 0xc2d0 },
	{ 0x6799000, 0xbde8 },
	{ 0x67b1000, 0xc88c },
	{ 0x67c5000, 0xa7f0 },
	{ 0x67d5000, 0xa770 },
	{ 0x67e9000, 0xb924 },
	{ 0x6801000, 0xcdf8 },
	{ 0x681d000, 0xc7e0 },
	{ 0x6831000, 0xc800 },
	{ 0x6845000, 0xc7dc },
	{ 0x6859000, 0xbf6c },
	{ 0x6869000, 0xc098 },
	{ 0x6885000, 0xc030 },
	{ 0x6899000, 0xca70 },
	{ 0x68ad000, 0xc2dc },
	{ 0x68c5000, 0xc8e4 },
	{ 0x68e1000, 0xb288 },
	{ 0x68f1000, 0xbdfc },
	{ 0x6905000, 0xb9fc },
	{ 0x6915000, 0xb4a0 },
	{ 0x692d000, 0xbfe8 },
	{ 0x6945000, 0xb6c8 },
	{ 0x6955000, 0xc794 },
	{ 0x6969000, 0xb9a8 },
	{ 0x6979000, 0xbe6c },
	{ 0x6989000, 0xb518 },
	{ 0x6999000, 0xb4f4 },
	{ 0x69a9000, 0xbf44 },
	{ 0x69b9000, 0xc224 },
	{ 0x69cd000, 0xc4e8 },
	{ 0x69e9000, 0xc4b8 },
	{ 0x69fd000, 0xc32c },
	{ 0x6a11000, 0xc7fc },
	{ 0x6a25000, 0xc314 },
	{ 0x6a39000, 0xcb54 },
	{ 0x6a4d000, 0xca0c },
	{ 0x6a65000, 0xc180 },
	{ 0x6a7d000, 0xba28 },
	{ 0x6a91000, 0xbb3c },
	{ 0x6aa5000, 0xc4a8 },
	{ 0x6ac1000, 0xc0e4 },
	{ 0x6ad9000, 0xc27c },
	{ 0x6af5000, 0xbfc4 },
	{ 0x6b05000, 0xb84c },
	{ 0x6b15000, 0xbdfc },
	{ 0x6b25000, 0xc6b4 },
	{ 0x6b39000, 0xb91c },
	{ 0x6b49000, 0x5f6c },
	{ 0x6b59000, 0xc1cc },
	{ 0x6b6d000, 0xcee4 },
	{ 0x6b81000, 0xc8c0 },
	{ 0x6b95000, 0xc76c },
	{ 0x6ba9000, 0xce40 },
	{ 0x6bbd000, 0xab68 },
	{ 0x6bcd000, 0xcbe4 },
	{ 0x6be1000, 0xabd0 },
	{ 0x6bf1000, 0xc0dc },
	{ 0x6c05000, 0xc360 },
	{ 0x6c19000, 0xce40 },
	{ 0x6c2d000, 0xc264 },
	{ 0x6c41000, 0xce9c },
	{ 0x6c55000, 0xd06c },
	{ 0x6c71000, 0xc320 },
	{ 0x6c89000, 0xcb38 },
	{ 0x6ca1000, 0xb708 },
	{ 0x6cb5000, 0xc1ec },
	{ 0x6cd1000, 0xc7f4 },
	{ 0x6ce9000, 0xc0dc },
	{ 0x6d05000, 0xb3f8 },
	{ 0x6d19000, 0xafec },
	{ 0x6d2d000, 0xa27c },
	{ 0x6d41000, 0x9bf4 },
	{ 0x6d5d000, 0xc280 },
	{ 0x6d71000, 0x393c },
	{ 0x6d79000, 0xc680 },
	{ 0x6d91000, 0xbd84 },
	{ 0x6da9000, 0xda8c },
	{ 0x6dbd000, 0xdef4 },
	{ 0x6dd1000, 0xda28 },
	{ 0x6de5000, 0xe458 },
	{ 0x6df9000, 0xebf0 },
	{ 0x6e0d000, 0xda90 },
	{ 0x6e21000, 0xdd08 },
	{ 0x6e35000, 0xda34 },
	{ 0x6e49000, 0xda2c },
	{ 0x6e5d000, 0xda30 },
	{ 0x6e71000, 0xd9d4 },
	{ 0x6e85000, 0xd8b8 },
	{ 0x6e99000, 0xec08 },
	{ 0x6ead000, 0xd9cc },
	{ 0x6ec1000, 0xda2c },
	{ 0x6ed5000, 0xd980 },
	{ 0x6ef1000, 0xfaf4 },
	{ 0x6f05000, 0xf594 },
	{ 0x6f21000, 0xbc00 },
	{ 0x6f35000, 0xe58c },
	{ 0x6f4d000, 0xb830 },
	{ 0x6f5d000, 0xbaa0 },
	{ 0x6f6d000, 0xb824 },
	{ 0x6f7d000, 0xb96c },
	{ 0x6f8d000, 0xbdb4 },
	{ 0x6f9d000, 0xb7b4 },
	{ 0x6fb5000, 0xb794 },
	{ 0x6fcd000, 0xc010 },
	{ 0x6fe1000, 0xe368 },
	{ 0x6ffd000, 0xbb24 },
	{ 0x7009000, 0xc01c },
	{ 0x7025000, 0xc26c },
	{ 0x7041000, 0xc114 },
	{ 0x7051000, 0xb014 },
	{ 0x706d000, 0xc8ac },
	{ 0x7089000, 0xc240 },
	{ 0x70a1000, 0xc7d0 },
	{ 0x70b9000, 0xb7f0 },
	{ 0x70c9000, 0xb7f0 },
	{ 0x70dd000, 0xb79c },
	{ 0x70f1000, 0xb7a4 },
	{ 0x7105000, 0xbae0 },
	{ 0x7119000, 0xb8c8 },
	{ 0x7131000, 0xb9f0 },
	{ 0x7145000, 0xb958 },
	{ 0x715d000, 0xbd94 },
	{ 0x7171000, 0xbd60 },
	{ 0x7185000, 0xc0ac },
	{ 0x71a1000, 0xb798 },
	{ 0x71b5000, 0xb8dc },
	{ 0x71c9000, 0xc2fc },
	{ 0x71e1000, 0xb8d8 },
	{ 0x71f5000, 0xb420 },
	{ 0x720d000, 0xb8d4 },
	{ 0x7225000, 0xbc18 },
	{ 0x7239000, 0xb61c },
	{ 0x724d000, 0xb808 },
	{ 0x7265000, 0xb804 },
	{ 0x727d000, 0xb804 },
	{ 0x7295000, 0xb948 },
	{ 0x72a9000, 0xb928 },
	{ 0x72c1000, 0xd5fc },
	{ 0x72d5000, 0xd4a0 },
	{ 0x72e9000, 0xd5e4 },
	{ 0x72fd000, 0x61b8 },
	{ 0x7309000, 0xd490 },
	{ 0x7325000, 0xc9c8 },
	{ 0x7341000, 0x5454 },
	{ 0x7355000, 0xbb7c },
	{ 0x736d000, 0xbff0 },
	{ 0x7381000, 0xf0dc },
	{ 0x7395000, 0xf0d8 },
	{ 0x73ad000, 0xd210 },
	{ 0x73c1000, 0xb9e4 },
	{ 0x73d9000, 0xc434 },
	{ 0x73f1000, 0xc850 },
	{ 0x7409000, 0x116e8 },
	{ 0x7421000, 0x93f0 },
	{ 0x7431000, 0x132dc },
	{ 0x7451000, 0xcec8 },
	{ 0x746d000, 0xb964 },
	{ 0x7479000, 0xb8d8 },
	{ 0x7491000, 0xc97c },
	{ 0x74a9000, 0xb79c },
	{ 0x74bd000, 0xbdc8 },
	{ 0x74d1000, 0xc778 },
	{ 0x74ed000, 0x32cc },
	{ 0x7501000, 0xc474 },
	{ 0x751d000, 0xb850 },
	{ 0x7531000, 0xbb98 },
	{ 0x7545000, 0xb3dc },
	{ 0x7559000, 0xbe6c },
	{ 0x7571000, 0xc9b0 },
	{ 0x758d000, 0xc0d8 },
	{ 0x75a9000, 0xbdd0 },
	{ 0x75b9000, 0xbdd0 },
	{ 0x75cd000, 0xc9ec },
	{ 0x75e5000, 0xc598 },
	{ 0x7601000, 0xbe70 },
	{ 0x7619000, 0xbeb4 },
	{ 0x762d000, 0xb9ec },
	{ 0x7645000, 0xc208 },
	{ 0x7661000, 0xb428 },
	{ 0x7679000, 0xbbb8 },
	{ 0x7691000, 0xb2e4 },
	{ 0x76a5000, 0xbba4 },
	{ 0x76b9000, 0xc770 },
	{ 0x76d5000, 0xc374 },
	{ 0x76f1000, 0xbb8c },
	{ 0x7705000, 0xd70c },
	{ 0x7721000, 0xc6f0 },
	{ 0x773d000, 0xb9a4 },
	{ 0x7751000, 0xb9c8 },
	{ 0x7769000, 0xbe0c },
	{ 0x7775000, 0xc1d4 },
	{ 0x7791000, 0xbf00 },
	{ 0x77a5000, 0xc308 },
	{ 0x77c1000, 0xbc48 },
	{ 0x77d9000, 0xb6b4 },
	{ 0x77ed000, 0xc2f4 },
	{ 0x7805000, 0xbacc },
	{ 0x781d000, 0xc010 },
	{ 0x7831000, 0xc030 },
	{ 0x784d000, 0xc584 },
	{ 0x7869000, 0xb8a4 },
	{ 0x7875000, 0xb8a4 },
	{ 0x788d000, 0xbb58 },
	{ 0x78a5000, 0xb988 },
	{ 0x78b9000, 0xb958 },
	{ 0x78cd000, 0xc418 },
	{ 0x78e9000, 0xb904 },
	{ 0x78fd000, 0xb9b0 },
	{ 0x7911000, 0xb7b0 },
	{ 0x7921000, 0xb744 },
	{ 0x7939000, 0xb7f4 },
	{ 0x794d000, 0xc568 },
	{ 0x7965000, 0xab94 },
	{ 0x7971000, 0x20 },
	{ 0x7975000, 0x4c1e },
	{ 0x797d000, 0x378d },
	{ 0x7981000, 0x407b },
	{ 0x7989000, 0x493ba },
	{ 0x79d5000, 0x2ecfa },
	{ 0x7a05000, 0x3a39c0 },
	{ 0x7da9000, 0x9c2df },
	{ 0x7e49000, 0x2c9 },
	{ 0x7e4d000, 0x66d8 },
	{ 0x7e55000, 0x66d8 },
	{ 0x7e5d000, 0x66d8 },
	{ 0x7e65000, 0x66d8 },
	{ 0x7e6d000, 0x66d8 },
	{ 0x7e75000, 0x66d8 },
	{ 0x7e7d000, 0x66d8 },
	{ 0x7e85000, 0x66d8 },
	{ 0x7e8d000, 0x66d8 },
	{ 0x7e95000, 0x66d8 },
	{ 0x7e9d000, 0x66d8 },
	{ 0x7ea5000, 0x66d8 },
	{ 0x7ead000, 0x66d8 },
	{ 0x7eb5000, 0x66d8 },
	{ 0x7ebd000, 0x66d8 },
	{ 0x7ec5000, 0x66d8 },
	{ 0x7ecd000, 0x66d8 },
	{ 0x7ed5000, 0x66d8 },
	{ 0x7edd000, 0x66d8 },
	{ 0x7ee5000, 0x66d8 },
	{ 0x7eed000, 0x66d8 },
	{ 0x7ef5000, 0x66d8 },
	{ 0x7efd000, 0x66d8 },
	{ 0x7f05000, 0x66d8 },
	{ 0x7f0d000, 0x66d8 },
	{ 0x7f15000, 0x66d8 },
	{ 0x7f1d000, 0x66d8 },
	{ 0x7f25000, 0x66d8 },
	{ 0x7f2d000, 0x66d8 },
	{ 0x7f35000, 0x66d8 },
	{ 0x7f3d000, 0x66d8 },
	{ 0x7f45000, 0x66d8 },
	{ 0x7f4d000, 0x66d8 },
	{ 0x7f55000, 0x66d8 },
	{ 0x7f5d000, 0x66d8 },
	{ 0x7f65000, 0x66d8 },
	{ 0x7f6d000, 0x66d8 },
	{ 0x7f75000, 0x66d8 },
	{ 0x7f7d000, 0x66d8 },
	{ 0x7f85000, 0x66d8 },
	{ 0x7f8d000, 0x66d8 },
	{ 0x7f95000, 0x66d8 },
};
static const struct fw_all_file fw_all_sapphire[] = {
	{ "image/adspr.jsn", 0x193, 0, 1 },
	{ "image/adsps.jsn", 0x195, 1, 1 },
	{ "image/adspua.jsn", 0x22b, 2, 1 },
	{ "image/cdspr.jsn", 0x193, 3, 1 },
	{ "image/adsp.b00", 0x4d4, 4, 1 },
	{ "image/adsp.b01", 0x2098, 5, 1 },
	{ "image/adsp.b02", 0x14ef4, 6, 1 },
	{ "image/adsp.b03", 0x3e9c0, 7, 1 },
	{ "image/adsp.b04", 0x18f288, 8, 1 },
	{ "image/adsp.b05", 0x19f620, 9, 1 },
	{ "image/adsp.b06", 0xb60, 10, 1 },
	{ "image/adsp.b07", 0x6354, 11, 1 },
	{ "image/adsp.b08", 0x6f6c, 12, 1 },
	{ "image/adsp.b09", 0x12dc, 13, 1 },
	{ "image/adsp.b10", 0x104, 14, 1 },
	{ "image/adsp.b12", 0xf2a0, 15, 1 },
	{ "image/adsp.b13", 0x11b58, 16, 1 },
	{ "image/adsp.b14", 0xc4000, 17, 1 },
	{ "image/adsp.b15", 0x1837e4, 18, 1 },
	{ "image/adsp.b16", 0x44004, 19, 1 },
	{ "image/adsp.b17", 0x342b, 20, 1 },
	{ "image/adsp.b18", 0x1b60, 21, 1 },
	{ "image/adsp.b19", 0x2dc, 22, 1 },
	{ "image/adsp.b20", 0x2140, 23, 1 },
	{ "image/adsp.b21", 0x6fec, 24, 1 },
	{ "image/adsp.b22", 0x3a0ec, 25, 1 },
	{ "image/adsp.b23", 0x80000, 26, 1 },
	{ "image/adsp.b24", 0x55a23e, 27, 1 },
	{ "image/adsp.b25", 0x1c513a, 28, 1 },
	{ "image/adsp.b26", 0x561c, 29, 1 },
	{ "image/adsp.b27", 0x53a74, 30, 1 },
	{ "image/adsp.b28", 0xc08, 31, 1 },
	{ "image/adsp.b29", 0xd6e28, 32, 1 },
	{ "image/adsp.b30", 0x50230, 33, 1 },
	{ "image/adsp.b31", 0xe00, 34, 1 },
	{ "image/adsp.b32", 0x13f0, 35, 1 },
	{ "image/adsp.b33", 0x23880, 36, 1 },
	{ "image/adsp.b34", 0x173a70, 37, 1 },
	{ "image/adsp.b35", 0xbc, 38, 1 },
	{ "image/adsp.mdt", 0x256c, 39, 1 },
	{ "image/cardapp.b00", 0x134, 40, 1 },
	{ "image/cardapp.b01", 0x1b28, 41, 1 },
	{ "image/cardapp.b02", 0x97d3, 42, 1 },
	{ "image/cardapp.b03", 0x91, 43, 1 },
	{ "image/cardapp.b04", 0x2b8, 44, 1 },
	{ "image/cardapp.b05", 0x14c, 45, 1 },
	{ "image/cardapp.b06", 0x98, 46, 1 },
	{ "image/cardapp.b07", 0x1cab, 47, 1 },
	{ "image/cardapp.mdt", 0x1c5c, 48, 1 },
	{ "image/cdsp.b00", 0x1d4, 49, 1 },
	{ "image/cdsp.b01", 0x1c18, 50, 1 },
	{ "image/cdsp.b02", 0x39898, 51, 1 },
	{ "image/cdsp.b03", 0x1a07c, 52, 1 },
	{ "image/cdsp.b04", 0x44c, 53, 1 },
	{ "image/cdsp.b05", 0x20db5c, 54, 1 },
	{ "image/cdsp.b06", 0x5b60, 55, 1 },
	{ "image/cdsp.b08", 0x95378, 56, 1 },
	{ "image/cdsp.b09", 0xa20, 57, 1 },
	{ "image/cdsp.b11", 0xe614, 58, 1 },
	{ "image/cdsp.mdt", 0x1dec, 59, 1 },
	{ "image/featenab.b00", 0x200, 60, 1 },
	{ "image/featenab.b01", 0x4e70, 61, 1 },
	{ "image/featenab.b02", 0xa88d, 62, 1 },
	{ "image/featenab.b03", 0xc4, 63, 1 },
	{ "image/featenab.b04", 0x391, 64, 1 },
	{ "image/featenab.b05", 0x1c0, 65, 1 },
	{ "image/featenab.b06", 0x130, 66, 1 },
	{ "image/featenab.b07", 0x2231, 67, 1 },
	{ "image/featenab.mdt", 0x5070, 68, 1 },
	{ "image/fpctzapp.b00", 0x200, 69, 1 },
	{ "image/fpctzapp.b01", 0x1b28, 70, 1 },
	{ "image/fpctzapp.b02", 0x1a3a43, 71, 1 },
	{ "image/fpctzapp.b03", 0xc3, 72, 1 },
	{ "image/fpctzapp.b04", 0x14f6dc, 73, 1 },
	{ "image/fpctzapp.b05", 0x4e0, 74, 1 },
	{ "image/fpctzapp.b06", 0x130, 75, 1 },
	{ "image/fpctzapp.b07", 0x1ad3d, 76, 1 },
	{ "image/fpctzapp.mdt", 0x1d28, 77, 1 },
	{ "image/goodixfp.b00", 0x200, 78, 1 },
	{ "image/goodixfp.b01", 0x1b28, 79, 1 },
	{ "image/goodixfp.b02", 0x6c26cc, 80, 1 },
	{ "image/goodixfp.b03", 0xc7, 81, 1 },
	{ "image/goodixfp.b04", 0x3d6d8c, 82, 1 },
	{ "image/goodixfp.b05", 0x6c0, 83, 1 },
	{ "image/goodixfp.b06", 0x150, 84, 1 },
	{ "image/goodixfp.b07", 0x2e943, 85, 1 },
	{ "image/goodixfp.mdt", 0x1d28, 86, 1 },
	{ "image/gptest.b00", 0x134, 87, 1 },
	{ "image/gptest.b01", 0x1b28, 88, 1 },
	{ "image/gptest.b02", 0x1430b, 89, 1 },
	{ "image/gptest.b03", 0xd4, 90, 1 },
	{ "image/gptest.b04", 0x6bb4, 91, 1 },
	{ "image/gptest.b05", 0x430, 92, 1 },
	{ "image/gptest.b06", 0x98, 93, 1 },
	{ "image/gptest.b07", 0x6c49, 94, 1 },
	{ "image/gptest.mdt", 0x1c5c, 95, 1 },
	{ "image/hdcpsrm.b00", 0x200, 96, 1 },
	{ "image/hdcpsrm.b01", 0x1b28, 97, 1 },
	{ "image/hdcpsrm.b02", 0x541d, 98, 1 },
	{ "image/hdcpsrm.b03", 0xc2, 99, 1 },
	{ "image/hdcpsrm.b04", 0x388, 100, 1 },
	{ "image/hdcpsrm.b05", 0x1c0, 101, 1 },
	{ "image/hdcpsrm.b06", 0x130, 102, 1 },
	{ "image/hdcpsrm.b07", 0x137d, 103, 1 },
	{ "image/hdcpsrm.mdt", 0x1d28, 104, 1 },
	{ "image/loadalgo.b00", 0x200, 105, 1 },
	{ "image/loadalgo.b01", 0x1b28, 106, 1 },
	{ "image/loadalgo.b02", 0xc0f5, 107, 1 },
	{ "image/loadalgo.b03", 0xc5, 108, 1 },
	{ "image/loadalgo.b04", 0x4c60, 109, 1 },
	{ "image/loadalgo.b05", 0x180, 110, 1 },
	{ "image/loadalgo.b06", 0x130, 111, 1 },
	{ "image/loadalgo.b07", 0x10a9, 112, 1 },
	{ "image/loadalgo.mdt", 0x1d28, 113, 1 },
	{ "image/mfido.b00", 0x134, 114, 1 },
	{ "image/mfido.b01", 0x1b28, 115, 1 },
	{ "image/mfido.b02", 0x2bb74, 116, 1 },
	{ "image/mfido.b03", 0x89, 117, 1 },
	{ "image/mfido.b04", 0xe47c, 118, 1 },
	{ "image/mfido.b05", 0x188, 119, 1 },
	{ "image/mfido.b06", 0x98, 120, 1 },
	{ "image/mfido.b07", 0x68ee, 121, 1 },
	{ "image/mfido.mdt", 0x1c5c, 122, 1 },
	{ "image/miriskm.b00", 0x200, 123, 1 },
	{ "image/miriskm.b01", 0x1b28, 124, 1 },
	{ "image/miriskm.b02", 0x1b20a, 125, 1 },
	{ "image/miriskm.b03", 0xbe, 126, 1 },
	{ "image/miriskm.b04", 0xc7c, 127, 1 },
	{ "image/miriskm.b05", 0x298, 128, 1 },
	{ "image/miriskm.b06", 0x130, 129, 1 },
	{ "image/miriskm.b07", 0x3931, 130, 1 },
	{ "image/miriskm.mdt", 0x1d28, 131, 1 },
	{ "image/mlipay.b00", 0x134, 132, 1 },
	{ "image/mlipay.b01", 0x1b28, 133, 1 },
	{ "image/mlipay.b02", 0xaf99, 134, 1 },
	{ "image/mlipay.b03", 0x8a, 135, 1 },
	{ "image/mlipay.b04", 0xa88, 136, 1 },
	{ "image/mlipay.b05", 0x174, 137, 1 },
	{ "image/mlipay.b06", 0x98, 138, 1 },
	{ "image/mlipay.b07", 0x231b, 139, 1 },
	{ "image/mlipay.mdt", 0x1c5c, 140, 1 },
	{ "image/modem.b00", 0x414, 141, 1 },
	{ "image/modem.b01", 0x1f78, 142, 1 },
	{ "image/modem.b02", 0x2930, 143, 1 },
	{ "image/modem.b03", 0x2518, 144, 1 },
	{ "image/modem.b04", 0x4a3ac, 145, 1 },
	{ "image/modem.b05", 0x119d8, 146, 1 },
	{ "image/modem.b06", 0x100c, 147, 1 },
	{ "image/modem.b07", 0x90b4, 148, 1 },
	{ "image/modem.b08", 0x4f054, 149, 1 },
	{ "image/modem.b09", 0x143ee8, 150, 1 },
	{ "image/modem.b10", 0x40000, 151, 1 },
	{ "image/modem.b11", 0x540, 152, 1 },
	{ "image/modem.b12", 0xe2210, 153, 1 },
	{ "image/modem.b14", 0x2d, 154, 1 },
	{ "image/modem.b15", 0x7c, 155, 1 },
	{ "image/modem.b16", 0x9c1a0, 156, 1 },
	{ "image/modem.b17", 0x61110, 157, 1 },
	{ "image/modem.b18", 0x2d4079, 158, 1 },
	{ "image/modem.b19", 0x2da98c, 159, 1 },
	{ "image/modem.b22", 0x10ec0, 160, 1 },
	{ "image/modem.b23", 0x1722040, 161, 1 },
	{ "image/modem.b24", 0xa07900, 162, 1 },
	{ "image/modem.b25", 0x778fb, 163, 1 },
	{ "image/modem.b26", 0x15104, 164, 1 },
	{ "image/modem.b27", 0x1378, 165, 1 },
	{ "image/modem.b28", 0xb0, 166, 1 },
	{ "image/modem.b29", 0x7d52c, 167, 1 },
	{ "image/modem.mdt", 0x238c, 168, 1 },
	{ "image/otrpapp.b00", 0x134, 169, 1 },
	{ "image/otrpapp.b01", 0x1b28, 170, 1 },
	{ "image/otrpapp.b02", 0x4c0c0, 171, 1 },
	{ "image/otrpapp.b03", 0x8f, 172, 1 },
	{ "image/otrpapp.b04", 0x3b80, 173, 1 },
	{ "image/otrpapp.b05", 0x238, 174, 1 },
	{ "image/otrpapp.b06", 0x98, 175, 1 },
	{ "image/otrpapp.b07", 0xb3c3, 176, 1 },
	{ "image/otrpapp.mdt", 0x1c5c, 177, 1 },
	{ "image/secauth.b00", 0x134, 178, 1 },
	{ "image/secauth.b01", 0x1b28, 179, 1 },
	{ "image/secauth.b02", 0x55b1, 180, 1 },
	{ "image/secauth.b03", 0x98, 181, 1 },
	{ "image/secauth.b04", 0x460, 182, 1 },
	{ "image/secauth.b05", 0xdc, 183, 1 },
	{ "image/secauth.b06", 0x98, 184, 1 },
	{ "image/secauth.b07", 0x102c, 185, 1 },
	{ "image/secauth.mdt", 0x1c5c, 186, 1 },
	{ "image/smplap32.b00", 0x134, 187, 1 },
	{ "image/smplap32.b01", 0x1b28, 188, 1 },
	{ "image/smplap32.b02", 0x43c7c, 189, 1 },
	{ "image/smplap32.b03", 0xcc, 190, 1 },
	{ "image/smplap32.b04", 0x1688bc, 191, 1 },
	{ "image/smplap32.b05", 0x4c0, 192, 1 },
	{ "image/smplap32.b06", 0x98, 193, 1 },
	{ "image/smplap32.b07", 0xb1a5, 194, 1 },
	{ "image/smplap32.mdt", 0x1c5c, 195, 1 },
	{ "image/smplap64.b00", 0x200, 196, 1 },
	{ "image/smplap64.b01", 0x1b28, 197, 1 },
	{ "image/smplap64.b02", 0x4ff70, 198, 1 },
	{ "image/smplap64.b03", 0xf8, 199, 1 },
	{ "image/smplap64.b04", 0x16b250, 200, 1 },
	{ "image/smplap64.b05", 0x980, 201, 1 },
	{ "image/smplap64.b06", 0x130, 202, 1 },
	{ "image/smplap64.b07", 0x1452b, 203, 1 },
	{ "image/smplap64.mdt", 0x1d28, 204, 1 },
	{ "image/soter64.b00", 0x200, 205, 1 },
	{ "image/soter64.b01", 0x1b28, 206, 1 },
	{ "image/soter64.b02", 0xcecd, 207, 1 },
	{ "image/soter64.b03", 0xc2, 208, 1 },
	{ "image/soter64.b04", 0x840, 209, 1 },
	{ "image/soter64.b05", 0x288, 210, 1 },
	{ "image/soter64.b06", 0x130, 211, 1 },
	{ "image/soter64.b07", 0x1b48, 212, 1 },
	{ "image/soter64.mdt", 0x1d28, 213, 1 },
	{ "image/vsimapp.b00", 0x134, 214, 1 },
	{ "image/vsimapp.b01", 0x1b28, 215, 1 },
	{ "image/vsimapp.b02", 0x7d3b, 216, 1 },
	{ "image/vsimapp.b03", 0xbb, 217, 1 },
	{ "image/vsimapp.b04", 0x334, 218, 1 },
	{ "image/vsimapp.b05", 0x10c, 219, 1 },
	{ "image/vsimapp.b06", 0x98, 220, 1 },
	{ "image/vsimapp.b07", 0x1510, 221, 1 },
	{ "image/vsimapp.mdt", 0x1c5c, 222, 1 },
	{ "image/qdsp6m.qdb", 0x3df1e1, 223, 1 },
	{ "image/modemr.jsn", 0x2cf, 224, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_hw/generic/common/divar/la/7+7_mode/sr_dsds/mcfg_hw.mbn", 0x4404, 225, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_hw/generic/common/divar/la/ss/mcfg_hw.mbn", 0x402c, 226, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_hw/generic/common/divar/cmcc_sub/sr_dsds/mcfg_hw.mbn", 0x588c, 227, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_hw/mbn_hw.dig", 0x20, 228, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_hw/mbn_hw.txt", 0x232, 229, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_hw/oem_hw.txt", 0x1, 230, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/af/cellc/commerci/safrica/mcfg_sw.mbn", 0xb4ac, 231, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/af/kenya/airtel/commerci/mcfg_sw.mbn", 0xb838, 232, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/af/morocco/inwi/commerci/mcfg_sw.mbn", 0xb7d8, 233, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/af/morocco/orange/commerci/mcfg_sw.mbn", 0xbdc8, 234, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/af/moroccot/commerci/ma/mcfg_sw.mbn", 0xb50c, 235, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/af/ooredoo/commerci/algeria/mcfg_sw.mbn", 0xb804, 236, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/airtel/commerci/mcfg_sw.mbn", 0xc048, 237, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/asiacell/commerci/mcfg_sw.mbn", 0xc264, 238, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/bakcell/commerci/mcfg_sw.mbn", 0xc328, 239, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/banglali/commerci/mcfg_sw.mbn", 0xb8f4, 240, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/beeline/commerci/mcfg_sw.mbn", 0xb958, 241, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/beeline/uzbe/commerci/mcfg_sw.mbn", 0xc4ac, 242, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/cellcom/commerci/israel/mcfg_sw.mbn", 0xc2c0, 243, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/dcm/commerci/mcfg_sw.mbn", 0xbb40, 244, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/dialog/commerci/sl/mcfg_sw.mbn", 0xbd1c, 245, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/grameenp/commerci/mcfg_sw.mbn", 0xbc08, 246, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/htmobile/commerci/mcfg_sw.mbn", 0xc464, 247, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/idea/commerci/mcfg_sw.mbn", 0xbb8c, 248, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/jtl/commerci/mcfg_sw.mbn", 0xb898, 249, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/jazz/commerci/mcfg_sw.mbn", 0xb7cc, 250, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/kddi/commerci/mcfg_sw.mbn", 0xeb6c, 251, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/kenya/commerci/mcfg_sw.mbn", 0xc24c, 252, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/mobitel/commerci/mcfg_sw.mbn", 0xb964, 253, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/ntc/commerci/nepal/mcfg_sw.mbn", 0xbd34, 254, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/ncell/commerci/nepal/mcfg_sw.mbn", 0xb948, 255, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/ooredoo/commerci/mcfg_sw.mbn", 0xaffc, 256, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/partner/commerci/mcfg_sw.mbn", 0xbfb4, 257, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/pelephon/commerci/israel/mcfg_sw.mbn", 0xc248, 258, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/rakuten/commerci/mcfg_sw.mbn", 0xf490, 259, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/reliance/commerci/mcfg_sw.mbn", 0xff90, 260, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/robi/commerci/mcfg_sw.mbn", 0xb388, 261, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/sbm/commerci/mcfg_sw.mbn", 0xb8b8, 262, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/tele2/commerci/mcfg_sw.mbn", 0xbc40, 263, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/telenor/commerci/mcfg_sw.mbn", 0xb8d8, 264, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/ucell/commerci/mcfg_sw.mbn", 0xb80c, 265, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/ufone/commerci/mcfg_sw.mbn", 0xc2e0, 266, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/uzmobile/commerci/mcfg_sw.mbn", 0xb7dc, 267, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/zain/commerci/mcfg_sw.mbn", 0xb940, 268, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/zong/commerci/mcfg_sw.mbn", 0xbd3c, 269, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/aunz/optus/commerci/au/mcfg_sw.mbn", 0xc4a4, 270, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/aunz/telstra/commerci/mcfg_sw.mbn", 0xc794, 271, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/aunz/vodafone/commerci/mcfg_sw.mbn", 0xc148, 272, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/cmcc/commerci/volte_cm/mcfg_sw.mbn", 0xec94, 273, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/cmcc/commerci/volte_op/mcfg_sw.mbn", 0xe56c, 274, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/cmcc/lab/agnss_lo/mcfg_sw.mbn", 0x7180, 275, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/cmcc/lab/conf_vol/mcfg_sw.mbn", 0xd1ac, 276, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/cmcc/lab/eps_only/mcfg_sw.mbn", 0x63d0, 277, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/cmcc/lab/lpp_loct/mcfg_sw.mbn", 0x7184, 278, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/cmcc/lab/nsiot_4r/mcfg_sw.mbn", 0xcb88, 279, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/cmcc/lab/nsiot_vo/mcfg_sw.mbn", 0xcd04, 280, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/cmcc/lab/rrlp_loc/mcfg_sw.mbn", 0x7184, 281, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/cmcc/lab/tgl_comb/mcfg_sw.mbn", 0x5b60, 282, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/cmcc/lab/w_irat_c/mcfg_sw.mbn", 0x6324, 283, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/ct/commerci/openmkt/mcfg_sw.mbn", 0xa698, 284, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/ct/commerci/volte_op/mcfg_sw.mbn", 0x121b8, 285, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/ct/commerci/hvolte_o/mcfg_sw.mbn", 0x11f24, 286, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/ct/lab/cta/mcfg_sw.mbn", 0x7618, 287, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/ct/lab/eps_only/mcfg_sw.mbn", 0xe9e0, 288, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/ct/lab/noapn_vo/mcfg_sw.mbn", 0xe640, 289, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/ct/lab/test/mcfg_sw.mbn", 0x9fa0, 290, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/ct/lab/test_eps/mcfg_sw.mbn", 0xa060, 291, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/ct/lab/test_no_/mcfg_sw.mbn", 0x9cc4, 292, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/ct/lab/volte_co/mcfg_sw.mbn", 0xe874, 293, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/ctm/commerci/mcfg_sw.mbn", 0xc068, 294, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/cu/commerci/openmkt/mcfg_sw.mbn", 0x6e2c, 295, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/cu/commerci/volte/mcfg_sw.mbn", 0xdb68, 296, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/cu/lab/test/mcfg_sw.mbn", 0x5814, 297, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/macau3/commerci/mcfg_sw.mbn", 0xb7f0, 298, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/1and1/commerci/de/mcfg_sw.mbn", 0xc838, 299, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/2degress/commerci/mcfg_sw.mbn", 0xbbc0, 300, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/4ka/commerci/slovenia/mcfg_sw.mbn", 0xbbf0, 301, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/a1/at_comme/mcfg_sw.mbn", 0xb824, 302, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/a1/bulgaria/mcfg_sw.mbn", 0xb914, 303, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/a1/commerci/slovenia/mcfg_sw.mbn", 0xc2bc, 304, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/a1/commerci/mcfg_sw.mbn", 0xbbdc, 305, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/a1/croatia_/mcfg_sw.mbn", 0xbf28, 306, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/a1/macedoni/mcfg_sw.mbn", 0xbe0c, 307, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/a1/serbia_c/mcfg_sw.mbn", 0xb88c, 308, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/bh/commerci/ba/mcfg_sw.mbn", 0xb954, 309, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/btbd/commerci/mcfg_sw.mbn", 0xb854, 310, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/bite/commerci/latvia/mcfg_sw.mbn", 0xbbe0, 311, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/bite/commerci/lithuani/mcfg_sw.mbn", 0xbbf8, 312, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/bouygues/commerci/france/mcfg_sw.mbn", 0xc668, 313, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/cr/commerci/mcfg_sw.mbn", 0xb8a8, 314, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/cosmote/commerci/mcfg_sw.mbn", 0xd050, 315, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/dt/commerci/austria/mcfg_sw.mbn", 0xcd88, 316, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/dt/commerci/cz/mcfg_sw.mbn", 0xcd54, 317, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/dt/commerci/croatia/mcfg_sw.mbn", 0xcae8, 318, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/dt/commerci/greece/mcfg_sw.mbn", 0xd048, 319, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/dt/commerci/hungary/mcfg_sw.mbn", 0xd6a4, 320, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/dt/commerci/macedoni/mcfg_sw.mbn", 0xc070, 321, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/dt/commerci/nl/mcfg_sw.mbn", 0xca14, 322, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/dt/commerci/pl/mcfg_sw.mbn", 0xcef4, 323, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/dt/commerci/slovakia/mcfg_sw.mbn", 0xc884, 324, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/dt/non_volt/commerci/mcfg_sw.mbn", 0x6090, 325, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/dt/volte/commerci/mcfg_sw.mbn", 0xcf20, 326, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/dt/volte/romania_/mcfg_sw.mbn", 0xcec8, 327, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/du/commerci/uae/mcfg_sw.mbn", 0xb914, 328, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/digi/commerci/hungary/mcfg_sw.mbn", 0xc570, 329, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/digi/commerci/spain/mcfg_sw.mbn", 0xbf34, 330, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/digi/romania_/mcfg_sw.mbn", 0xb498, 331, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/ee/commerci/mcfg_sw.mbn", 0xc688, 332, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/eir/commerci/ireland/mcfg_sw.mbn", 0xb7d8, 333, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/elisa/commerci/estonia/mcfg_sw.mbn", 0xbdf0, 334, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/elisa/commerci/fi/mcfg_sw.mbn", 0xbfe0, 335, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/enetwork/commerci/mcfg_sw.mbn", 0xbc44, 336, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/etisalat/commerci/egypt/mcfg_sw.mbn", 0xb9cc, 337, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/etisalat/commerci/uae/mcfg_sw.mbn", 0xbecc, 338, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/fastweb/commerci/italy/mcfg_sw.mbn", 0xd440, 339, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/free/volte/france/mcfg_sw.mbn", 0xbb7c, 340, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/gibtel/commerci/gibra/mcfg_sw.mbn", 0xba58, 341, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/h3g/commerci/austria/mcfg_sw.mbn", 0xb954, 342, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/h3g/commerci/denmark/mcfg_sw.mbn", 0xbf58, 343, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/h3g/commerci/ireland/mcfg_sw.mbn", 0xbec4, 344, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/h3g/commerci/italy/mcfg_sw.mbn", 0xd474, 345, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/h3g/commerci/se/mcfg_sw.mbn", 0xc09c, 346, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/h3g/commerci/uk/mcfg_sw.mbn", 0xbba4, 347, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/ice/commerci/norway/mcfg_sw.mbn", 0xbb9c, 348, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/kpn/commerci/mcfg_sw.mbn", 0xc110, 349, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/kyvistar/volte/ukraine/mcfg_sw.mbn", 0xca08, 350, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/lmt/commerci/latvia/mcfg_sw.mbn", 0xb65c, 351, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/lifecell/volte/ukraine/mcfg_sw.mbn", 0xc138, 352, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/meo/commerci/mcfg_sw.mbn", 0xbfb0, 353, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/mtel/commerci/ba/mcfg_sw.mbn", 0xb920, 354, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/mtn/commerci/ng/mcfg_sw.mbn", 0xc190, 355, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/mtn/commerci/safrica/mcfg_sw.mbn", 0xc948, 356, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/mts/commerci/armenia/mcfg_sw.mbn", 0xbf18, 357, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/mts/commerci/belarus/mcfg_sw.mbn", 0xbb1c, 358, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/mts/volte/serbia/mcfg_sw.mbn", 0xb97c, 359, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/moldcell/commerci/moldova/mcfg_sw.mbn", 0xb3fc, 360, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/nos/commerci/mcfg_sw.mbn", 0xb9ec, 361, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/nova/commerci/mcfg_sw.mbn", 0xc894, 362, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/o2/commerci/czech/mcfg_sw.mbn", 0xc678, 363, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/o2/commerci/slovakia/mcfg_sw.mbn", 0xc260, 364, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/o2/uk_comme/mcfg_sw.mbn", 0xc5dc, 365, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/ora/commerci/france/mcfg_sw.mbn", 0xb840, 366, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/orange/commerci/belgium/mcfg_sw.mbn", 0xbf04, 367, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/orange/commerci/caraibes/mcfg_sw.mbn", 0xbf94, 368, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/orange/commerci/egypt/mcfg_sw.mbn", 0xc224, 369, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/orange/commerci/france/mcfg_sw.mbn", 0xd7a8, 370, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/orange/commerci/group_no/mcfg_sw.mbn", 0x6180, 371, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/orange/commerci/luxembou/mcfg_sw.mbn", 0xb84c, 372, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/orange/commerci/poland/mcfg_sw.mbn", 0xdd20, 373, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/orange/commerci/reunion/mcfg_sw.mbn", 0xbbcc, 374, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/orange/commerci/romania/mcfg_sw.mbn", 0xc9e4, 375, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/orange/commerci/slovakia/mcfg_sw.mbn", 0xc588, 376, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/orange/commerci/spain/mcfg_sw.mbn", 0xcc44, 377, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/orange/moldova_/mcfg_sw.mbn", 0xaf7c, 378, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/play/poland_c/mcfg_sw.mbn", 0xcc54, 379, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/plintron/commerci/italy/mcfg_sw.mbn", 0xb8cc, 380, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/plus/poland_c/mcfg_sw.mbn", 0xd00c, 381, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/post/commerci/luxembou/mcfg_sw.mbn", 0xb9d8, 382, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/proximus/commerci/mcfg_sw.mbn", 0xbc74, 383, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/proximus/volte/luxembou/mcfg_sw.mbn", 0xb8e8, 384, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/sfr/commerci/altice/mcfg_sw.mbn", 0xbe88, 385, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/sfr/commerci/fr/mcfg_sw.mbn", 0xc298, 386, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/salt/commerci/mcfg_sw.mbn", 0xc4c4, 387, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/siminn/commerci/mcfg_sw.mbn", 0xbc74, 388, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/spark/commerci/mcfg_sw.mbn", 0xc150, 389, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/spusu/commerci/at/mcfg_sw.mbn", 0xbaf8, 390, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/spusu/commerci/it/mcfg_sw.mbn", 0xbcf8, 391, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/spusu/commerci/uk/mcfg_sw.mbn", 0xbde0, 392, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/sunrise/commerci/mcfg_sw.mbn", 0xb8cc, 393, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/swisscom/commerci/swiss/mcfg_sw.mbn", 0xc2d0, 394, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/tdc/commerci/denmark/mcfg_sw.mbn", 0xbde8, 395, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/tim/commerci/italy/mcfg_sw.mbn", 0xc88c, 396, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/tim/commerci/kena/mcfg_sw.mbn", 0xa7f0, 397, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/tim/commerci/welcome/mcfg_sw.mbn", 0xa770, 398, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/tmo/poland_c/mcfg_sw.mbn", 0xb924, 399, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telavox/commerci/se/mcfg_sw.mbn", 0xcdf8, 400, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/tele2/commerci/estonia/mcfg_sw.mbn", 0xc7e0, 401, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/tele2/commerci/lithuani/mcfg_sw.mbn", 0xc800, 402, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/tele2/commerci/latvia/mcfg_sw.mbn", 0xc7dc, 403, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/tele2/commerci/nl/mcfg_sw.mbn", 0xbf6c, 404, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/tele2/commerci/sweden/mcfg_sw.mbn", 0xc098, 405, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telefoni/commerci/de/mcfg_sw.mbn", 0xc030, 406, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telefoni/commerci/spain/mcfg_sw.mbn", 0xca70, 407, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telefoni/commerci/uk/mcfg_sw.mbn", 0xc2dc, 408, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telefoni/non_volt/spain/mcfg_sw.mbn", 0xc8e4, 409, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telekom/commerci/argentin/mcfg_sw.mbn", 0xb288, 410, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telekom/commerci/slovenia/mcfg_sw.mbn", 0xbdfc, 411, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telemach/commerci/mcfg_sw.mbn", 0xb9fc, 412, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telemach/hr_comme/mcfg_sw.mbn", 0xb4a0, 413, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telenet/volte/belgium/mcfg_sw.mbn", 0xbfe8, 414, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telenor/commerci/bulgaria/mcfg_sw.mbn", 0xb6c8, 415, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telenor/commerci/denmark/mcfg_sw.mbn", 0xc794, 416, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telenor/commerci/finland/mcfg_sw.mbn", 0xb9a8, 417, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telenor/commerci/hungary/mcfg_sw.mbn", 0xbe6c, 418, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telenor/commerci/monteneg/mcfg_sw.mbn", 0xb518, 419, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telenor/commerci/myanmar/mcfg_sw.mbn", 0xb4f4, 420, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telenor/commerci/norway/mcfg_sw.mbn", 0xbf44, 421, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telenor/commerci/serbia/mcfg_sw.mbn", 0xc224, 422, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telenor/commerci/sweden/mcfg_sw.mbn", 0xc4e8, 423, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telia/commerci/denmark/mcfg_sw.mbn", 0xc4b8, 424, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telia/commerci/estonia/mcfg_sw.mbn", 0xc32c, 425, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telia/commerci/finland/mcfg_sw.mbn", 0xc7fc, 426, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telia/commerci/lithuani/mcfg_sw.mbn", 0xc314, 427, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telia/commerci/norway/mcfg_sw.mbn", 0xcb54, 428, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telia/commerci/sweden/mcfg_sw.mbn", 0xca0c, 429, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telkom/commerci/mcfg_sw.mbn", 0xc180, 430, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/turktele/commerci/mcfg_sw.mbn", 0xba28, 431, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/turkcell/commerci/mcfg_sw.mbn", 0xbb3c, 432, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vimla/commerci/mcfg_sw.mbn", 0xc4a8, 433, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/virgin/commerci/uk/mcfg_sw.mbn", 0xc0e4, 434, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vivacom/commerci/mcfg_sw.mbn", 0xc27c, 435, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/commerci/albania/mcfg_sw.mbn", 0xbfc4, 436, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/commerci/hungary/mcfg_sw.mbn", 0xb84c, 437, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/commerci/is/mcfg_sw.mbn", 0xbdfc, 438, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/commerci/ireland/mcfg_sw.mbn", 0xc6b4, 439, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/india_vo/mcfg_sw.mbn", 0xb91c, 440, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/non_volt/mcfg_sw.mbn", 0x5f6c, 441, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/volte/ctc/mcfg_sw.mbn", 0xc1cc, 442, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/volte/cz/mcfg_sw.mbn", 0xcee4, 443, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/volte/egypt/mcfg_sw.mbn", 0xc8c0, 444, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/volte/gr/mcfg_sw.mbn", 0xc76c, 445, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/volte/germany/mcfg_sw.mbn", 0xce40, 446, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/volte/italy/mcfg_sw.mbn", 0xab68, 447, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/volte/netherla/mcfg_sw.mbn", 0xcbe4, 448, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/volte/portugal/mcfg_sw.mbn", 0xabd0, 449, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/volte/ro/mcfg_sw.mbn", 0xc0dc, 450, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/volte/safrica/mcfg_sw.mbn", 0xc360, 451, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/volte/spain/mcfg_sw.mbn", 0xce40, 452, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/volte/turkey/mcfg_sw.mbn", 0xc264, 453, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/volte/uk/mcfg_sw.mbn", 0xce9c, 454, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/volte/ukraine/mcfg_sw.mbn", 0xd06c, 455, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/we/commerci/egypt/mcfg_sw.mbn", 0xc320, 456, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/wind/commerci/mcfg_sw.mbn", 0xcb38, 457, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/yoigo/commerci/mcfg_sw.mbn", 0xb708, 458, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/zeop/commerci/mcfg_sw.mbn", 0xc1ec, 459, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/coop/commerci/italy/mcfg_sw.mbn", 0xc7f4, 460, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vee/commerci/mcfg_sw.mbn", 0xc0dc, 461, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/korea/kt/commerci/mcfg_sw.mbn", 0xb3f8, 462, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/korea/lgu/commerci/mcfg_sw.mbn", 0xafec, 463, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/korea/skt/commerci/mcfg_sw.mbn", 0xa27c, 464, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/korea/tta/commerci/mcfg_sw.mbn", 0x9bf4, 465, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/amx/commerci/mx/mcfg_sw.mbn", 0xc280, 466, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/amx/non_volt/mcfg_sw.mbn", 0x393c, 467, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/amx/volte/mcfg_sw.mbn", 0xc680, 468, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/antel/commerci/mcfg_sw.mbn", 0xbd84, 469, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/claro/commerci/arg/mcfg_sw.mbn", 0xda8c, 470, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/claro/commerci/brazil/mcfg_sw.mbn", 0xdef4, 471, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/claro/commerci/cri/mcfg_sw.mbn", 0xda28, 472, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/claro/commerci/chile/mcfg_sw.mbn", 0xe458, 473, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/claro/commerci/colombia/mcfg_sw.mbn", 0xebf0, 474, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/claro/commerci/dominica/mcfg_sw.mbn", 0xda90, 475, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/claro/commerci/ecuador/mcfg_sw.mbn", 0xdd08, 476, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/claro/commerci/guatemal/mcfg_sw.mbn", 0xda34, 477, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/claro/commerci/honduras/mcfg_sw.mbn", 0xda2c, 478, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/claro/commerci/nicaragu/mcfg_sw.mbn", 0xda30, 479, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/claro/commerci/panama/mcfg_sw.mbn", 0xd9d4, 480, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/claro/commerci/paraguay/mcfg_sw.mbn", 0xd8b8, 481, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/claro/commerci/peru/mcfg_sw.mbn", 0xec08, 482, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/claro/commerci/puerto/mcfg_sw.mbn", 0xd9cc, 483, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/claro/commerci/salvador/mcfg_sw.mbn", 0xda2c, 484, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/claro/commerci/uruguay/mcfg_sw.mbn", 0xd980, 485, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/entel/commerci/chile/mcfg_sw.mbn", 0xfaf4, 486, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/entel/commerci/peru/mcfg_sw.mbn", 0xf594, 487, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/liberty/commerci/pa/mcfg_sw.mbn", 0xbc00, 488, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/mx-att/commerci/mcfg_sw.mbn", 0xe58c, 489, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/movistar/argentin/mcfg_sw.mbn", 0xb830, 490, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/movistar/chile_co/mcfg_sw.mbn", 0xbaa0, 491, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/movistar/ecuador_/mcfg_sw.mbn", 0xb824, 492, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/movistar/mx_comme/mcfg_sw.mbn", 0xb96c, 493, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/movistar/panama_c/mcfg_sw.mbn", 0xbdb4, 494, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/movistar/uruguay_/mcfg_sw.mbn", 0xb7b4, 495, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/tim/commerci/brazil/mcfg_sw.mbn", 0xb794, 496, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/telefoni/commerci/colombia/mcfg_sw.mbn", 0xc010, 497, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/telefoni/commerci/peru/mcfg_sw.mbn", 0xe368, 498, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/tigo/commerci/guatemal/mcfg_sw.mbn", 0xbb24, 499, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/tigo/commerci/mcfg_sw.mbn", 0xc01c, 500, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/vivo/commerci/brazil/mcfg_sw.mbn", 0xc26c, 501, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/wom/commerci/colombia/mcfg_sw.mbn", 0xc114, 502, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/wom/commerci/mcfg_sw.mbn", 0xb014, 503, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/019mobil/commerci/il/mcfg_sw.mbn", 0xc8ac, 504, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/annatel/commerci/il/mcfg_sw.mbn", 0xc240, 505, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/azerbaij/commerci/mcfg_sw.mbn", 0xc7d0, 506, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/bahrain/bahrain_/mcfg_sw.mbn", 0xb7f0, 507, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/bahrain/betelco_/mcfg_sw.mbn", 0xb7f0, 508, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/bahrain/viva/commerci/mcfg_sw.mbn", 0xb79c, 509, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/bahrain/zain/commerci/mcfg_sw.mbn", 0xb7a4, 510, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/cyta/commerci/mcfg_sw.mbn", 0xbae0, 511, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/epic/commerci/mcfg_sw.mbn", 0xb8c8, 512, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/jordan/orange/commerci/mcfg_sw.mbn", 0xb9f0, 513, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/jordan/umniah/commerci/mcfg_sw.mbn", 0xb958, 514, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/kuwait/viva/commerci/mcfg_sw.mbn", 0xbd94, 515, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/kuwait/zain/commerci/mcfg_sw.mbn", 0xbd60, 516, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/kuwait/ooredoo/commerci/mcfg_sw.mbn", 0xc0ac, 517, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/oman/omantel/commerci/mcfg_sw.mbn", 0xb798, 518, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/oman/ooredoo/commerci/mcfg_sw.mbn", 0xb8dc, 519, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/oman/vodafone/commerci/mcfg_sw.mbn", 0xc2fc, 520, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/primetel/commerci/mcfg_sw.mbn", 0xb8d8, 521, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/qatar/commerci/mcfg_sw.mbn", 0xb420, 522, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/stc/commerci/sa/mcfg_sw.mbn", 0xb8d4, 523, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/saudi/mobily/commerci/mcfg_sw.mbn", 0xbc18, 524, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/saudi/salam/commerci/mcfg_sw.mbn", 0xb61c, 525, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/saudi/zain/commerci/mcfg_sw.mbn", 0xb808, 526, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/wecom/commerci/il/mcfg_sw.mbn", 0xb804, 527, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/xphone/commerci/il/mcfg_sw.mbn", 0xb804, 528, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mexico/altan/commerci/mcfg_sw.mbn", 0xb948, 529, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mexico/axtel/commerci/mcfg_sw.mbn", 0xb928, 530, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/att/american/mcfg_sw.mbn", 0xd5fc, 531, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/att/firstnet/mcfg_sw.mbn", 0xd4a0, 532, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/att/guam_cnm/mcfg_sw.mbn", 0xd5e4, 533, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/att/non_volt/mcfg_sw.mbn", 0x61b8, 534, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/att/volte/mcfg_sw.mbn", 0xd490, 535, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/bell/commerci/ca/mcfg_sw.mbn", 0xc9c8, 536, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/cbrs/commerci/us/mcfg_sw.mbn", 0x5454, 537, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/dish/commerci/us/mcfg_sw.mbn", 0xbb7c, 538, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/rogers/commerci/ca/mcfg_sw.mbn", 0xbff0, 539, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/sprint/commerci/mcfg_sw.mbn", 0xf0dc, 540, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/sprint/vowifi/mcfg_sw.mbn", 0xf0d8, 541, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/tmo/commerci/mcfg_sw.mbn", 0xd210, 542, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/tmo/poland_c/mcfg_sw.mbn", 0xb9e4, 543, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/telus/commerci/ca/mcfg_sw.mbn", 0xc434, 544, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/uscc/commerci/mcfg_sw.mbn", 0xc850, 545, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/verizon/cdmaless/mcfg_sw.mbn", 0x116e8, 546, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/verizon/imsless/mcfg_sw.mbn", 0x93f0, 547, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/verizon/hvolte/mcfg_sw.mbn", 0x132dc, 548, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/russia/beeline/gen_3gpp/mcfg_sw.mbn", 0xcec8, 549, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/russia/mts/commerci/ru/mcfg_sw.mbn", 0xb964, 550, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/russia/mts/commerci/mcfg_sw.mbn", 0xb8d8, 551, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/russia/megafon/commerci/ru/mcfg_sw.mbn", 0xc97c, 552, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/russia/motiv/commerci/mcfg_sw.mbn", 0xb79c, 553, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/russia/sberbank/commerci/mcfg_sw.mbn", 0xbdc8, 554, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/russia/tele2/commerci/mcfg_sw.mbn", 0xc778, 555, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sa/brazil/commerci/mcfg_sw.mbn", 0x32cc, 556, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/3hk/commerci/hk/mcfg_sw.mbn", 0xc474, 557, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/ais/commerci/thailand/mcfg_sw.mbn", 0xb850, 558, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/apt/commerci/mcfg_sw.mbn", 0xbb98, 559, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/cuhk/commerci/mcfg_sw.mbn", 0xb3dc, 560, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/celcom/commerci/mcfg_sw.mbn", 0xbe6c, 561, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/chunghwa/commerci/tw/mcfg_sw.mbn", 0xc9b0, 562, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/dito/commerci/ph/mcfg_sw.mbn", 0xc0d8, 563, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/dtac/commerci/th/mcfg_sw.mbn", 0xbdd0, 564, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/dtac/commerci/volte/mcfg_sw.mbn", 0xbdd0, 565, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/digi/commerci/mcfg_sw.mbn", 0xc9ec, 566, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/fareasto/commerci/mcfg_sw.mbn", 0xc598, 567, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/globe/commerci/ph/mcfg_sw.mbn", 0xbe70, 568, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/hkt/commerci/hk/mcfg_sw.mbn", 0xbeb4, 569, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/indosat/commerci/mcfg_sw.mbn", 0xb9ec, 570, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/m1/commerci/sg/mcfg_sw.mbn", 0xc208, 571, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/mpt/commerci/myanmar/mcfg_sw.mbn", 0xb428, 572, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/mytel/commerci/myanmar/mcfg_sw.mbn", 0xbbb8, 573, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/maxis/commerci/malaysia/mcfg_sw.mbn", 0xb2e4, 574, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/metfone/commerci/mcfg_sw.mbn", 0xbba4, 575, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/mobifone/commerci/mcfg_sw.mbn", 0xc770, 576, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/nt/commerci/thailand/mcfg_sw.mbn", 0xc374, 577, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/p1/commerci/malaysia/mcfg_sw.mbn", 0xbb8c, 578, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/seatel/commerci/mcfg_sw.mbn", 0xd70c, 579, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/singtel/commerci/singapor/mcfg_sw.mbn", 0xc6f0, 580, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/smartone/commerci/hk/mcfg_sw.mbn", 0xb9a4, 581, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/smart_ph/commerci/mcfg_sw.mbn", 0xb9c8, 582, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/smartfre/commerci/vowifi/mcfg_sw.mbn", 0xbe0c, 583, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/smartfre/commerci/mcfg_sw.mbn", 0xc1d4, 584, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/starhub/commerci/sg/mcfg_sw.mbn", 0xbf00, 585, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/tm/commerci/mcfg_sw.mbn", 0xc308, 586, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/tpg/commerci/tpg/mcfg_sw.mbn", 0xbc48, 587, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/tstar/commerci/tw/mcfg_sw.mbn", 0xb6b4, 588, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/telkomse/commerci/mcfg_sw.mbn", 0xc2f4, 589, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/tri/commerci/mcfg_sw.mbn", 0xbacc, 590, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/truemove/commerci/thailand/mcfg_sw.mbn", 0xc010, 591, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/truemove/commerci/volte/mcfg_sw.mbn", 0xc030, 592, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/umobile/commerci/malaysia/mcfg_sw.mbn", 0xc584, 593, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/vietnamo/commerci/vietnamo/mcfg_sw.mbn", 0xb8a4, 594, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/vietnamo/commerci/mcfg_sw.mbn", 0xb8a4, 595, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/viettel/commerci/vietnam/mcfg_sw.mbn", 0xbb58, 596, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/vinaphon/commerci/vietnam/mcfg_sw.mbn", 0xb988, 597, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/xl/commerci/mcfg_sw.mbn", 0xb958, 598, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/ytl/commerci/mcfg_sw.mbn", 0xc418, 599, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/airtel/commerci/lanka/mcfg_sw.mbn", 0xb904, 600, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/smart/commerci/mcfg_sw.mbn", 0xb9b0, 601, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/test/er_iot/mcfg_sw.mbn", 0xb7b0, 602, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/test/nk_iot/mcfg_sw.mbn", 0xb744, 603, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/common/row/commerci/mcfg_sw.mbn", 0xb7f4, 604, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/common/test/commerci/mcfg_sw.mbn", 0xc568, 605, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/common/w_one/commerci/mcfg_sw.mbn", 0xab94, 606, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/mbn_sw.dig", 0x20, 607, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/mbn_sw.txt", 0x4c1e, 608, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/oem_eea.txt", 0x378d, 609, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/oem_sw.txt", 0x407b, 610, 1 },
	{ "image/rtic.mbn", 0x493ba, 611, 1 },
	{ "image/widevine.mbn", 0x2ecfa, 612, 1 },
	{ "image/wlanmdsp.mbn", 0x3a39c0, 613, 1 },
	{ "image/data.msc", 0x9c2df, 614, 1 },
	{ "image/modemuw.jsn", 0x2c9, 615, 1 },
	{ "image/bdn7gl.bin", 0x66d8, 616, 1 },
	{ "image/bdn7lmx.bin", 0x66d8, 617, 1 },
	{ "image/bdn7neu.bin", 0x66d8, 618, 1 },
	{ "image/bdwlan.102", 0x66d8, 619, 1 },
	{ "image/bdwlan.104", 0x66d8, 620, 1 },
	{ "image/bdwlan.105", 0x66d8, 621, 1 },
	{ "image/bdwlan.106", 0x66d8, 622, 1 },
	{ "image/bdwlan.107", 0x66d8, 623, 1 },
	{ "image/bdwlan.108", 0x66d8, 624, 1 },
	{ "image/bdwlan.109", 0x66d8, 625, 1 },
	{ "image/bdwlan.10e", 0x66d8, 626, 1 },
	{ "image/bdwlan.10f", 0x66d8, 627, 1 },
	{ "image/bdwlan.111", 0x66d8, 628, 1 },
	{ "image/bdwlan.202", 0x66d8, 629, 1 },
	{ "image/bdwlan.203", 0x66d8, 630, 1 },
	{ "image/bdwlan.204", 0x66d8, 631, 1 },
	{ "image/bdwlan.206", 0x66d8, 632, 1 },
	{ "image/bdwlan.b04", 0x66d8, 633, 1 },
	{ "image/bdwlan.b07", 0x66d8, 634, 1 },
	{ "image/bdwlan.b09", 0x66d8, 635, 1 },
	{ "image/bdwlan.b0a", 0x66d8, 636, 1 },
	{ "image/bdwlan.b0b", 0x66d8, 637, 1 },
	{ "image/bdwlan.b0d", 0x66d8, 638, 1 },
	{ "image/bdwlan.b0e", 0x66d8, 639, 1 },
	{ "image/bdwlan.b0f", 0x66d8, 640, 1 },
	{ "image/bdwlan.b14", 0x66d8, 641, 1 },
	{ "image/bdwlan.b15", 0x66d8, 642, 1 },
	{ "image/bdwlan.b30", 0x66d8, 643, 1 },
	{ "image/bdwlan.b31", 0x66d8, 644, 1 },
	{ "image/bdwlan.b32", 0x66d8, 645, 1 },
	{ "image/bdwlan.b33", 0x66d8, 646, 1 },
	{ "image/bdwlan.b34", 0x66d8, 647, 1 },
	{ "image/bdwlan.b35", 0x66d8, 648, 1 },
	{ "image/bdwlan.b36", 0x66d8, 649, 1 },
	{ "image/bdwlan.b37", 0x66d8, 650, 1 },
	{ "image/bdwlan.b38", 0x66d8, 651, 1 },
	{ "image/bdwlan.bin", 0x66d8, 652, 1 },
	{ "image/bdwlans.202", 0x66d8, 653, 1 },
	{ "image/bdwlant.202", 0x66d8, 654, 1 },
	{ "image/bdwlanu.202", 0x66d8, 655, 1 },
	{ "image/bdwlanu.206", 0x66d8, 656, 1 },
	{ "image/bdwlanu.206", 0x66d8, 657, 1 },
};

/* the tables of the phone fw_pick() found (same order as fw_tables) */
static const struct fw_extent *fw_ext_tab[3] = { fw_ext_topaz, fw_ext_sapphire, NULL };
static const struct fw_all_file *fw_all_tab[3] = { fw_all_topaz, fw_all_sapphire, NULL };
static unsigned fw_all_count[3] = {
	sizeof(fw_all_topaz) / sizeof(fw_all_topaz[0]), sizeof(fw_all_sapphire) / sizeof(fw_all_sapphire[0]), 0,
};
#define fw_ext (fw_ext_tab[fw_phone])
#define fw_all (fw_all_tab[fw_phone])

static int
fw_all_find(const char *path)
{
	for (unsigned i = 0; i < fw_all_count[fw_phone]; i++) {
		if (strcmp(fw_all[i].path, path) == 0) {
			return (int)i;
		}
		if (fw_phone == FW_FAT) {
			/* the FAT's names were lower-cased: FAT itself ignores case */
			const char *a = fw_all[i].path, *b = path;
			while (*a && *b && *a == ((*b >= 'A' && *b <= 'Z') ? *b + 32 : *b)) {
				a++;
				b++;
			}
			if (*a == 0 && *b == 0) {
				return (int)i;
			}
		}
	}
	return -1;
}

/* read n bytes at off of file i from modem_a (extent by extent) */
static bool
fw_all_read(int i, uint64_t off, void *dst, uint64_t n)
{
	uint64_t base = 0, done = 0;
	for (unsigned k = 0; k < fw_all[i].next && done < n; k++) {
		uint32_t eo = fw_ext[fw_all[i].ext + k].off, el = fw_ext[fw_all[i].ext + k].len;
		if (off + done < base + el) {
			uint64_t in = off + done - base, take = MIN((uint64_t)el - in, n - done);
			if (sweet_ufs_read_part(fw_part, eo + in, (uint8_t *)dst + done, take) != 0) {
				return false;
			}
			done += take;
		}
		base += el;
	}
	return done == n;
}

/*
 * sapphire 0308: firmware no table knows (a newer HyperOS than 2.0.208, any phone's): the partition is a
 * FAT16/FAT32 volume, so its files are found by name. Every file under it becomes an fw_all entry
 * (lower-case path, extents from its cluster chain), the modem's own files ("modem.mdt", "modem.bNN")
 * are read through them, and the QSEE / TFTP users get the same table. Read-only, as before.
 */
#define FW_FAT_MAX_FILES    2048
#define FW_FAT_MAX_EXT      8192

static struct fw_extent *fat_ext;
static struct fw_all_file *fat_all;
static unsigned fat_next, fat_nfiles;
static uint8_t *fat_tab;                        /* the whole FAT */
static uint32_t fat_bits, fat_bps, fat_clus_bytes, fat_nclus;
static uint64_t fat_data_off, fat_root_off;
static uint32_t fat_root_bytes, fat_root_clus;

static uint32_t
fat_le(const uint8_t *p, int n)
{
	uint32_t v = 0;
	for (int i = n - 1; i >= 0; i--) {
		v = v << 8 | p[i];
	}
	return v;
}

static uint32_t
fat_get(uint32_t c)
{
	if (fat_bits == 16) {
		return fat_le(fat_tab + 2 * c, 2);
	}
	return fat_le(fat_tab + 4 * c, 4) & 0x0FFFFFFF;
}

static bool
fat_eoc(uint32_t c)
{
	return c < 2 || c >= (fat_bits == 16 ? 0xFFF8u : 0x0FFFFFF8u) || c >= fat_nclus + 2;
}

static uint64_t
fat_clus_off(uint32_t c)
{
	return fat_data_off + (uint64_t)(c - 2) * fat_clus_bytes;
}

/* a file's clusters as extents in fat_ext; false when the tables are full or the chain is broken */
static bool
fat_add_file(const char *path, uint32_t first, uint32_t size)
{
	if (fat_nfiles == FW_FAT_MAX_FILES) {
		return false;
	}
	unsigned start = fat_next, guard = 0;
	uint32_t left = size, c = first;
	while (left > 0) {
		if (fat_eoc(c) || fat_next == FW_FAT_MAX_EXT || ++guard > fat_nclus) {
			return false;
		}
		uint32_t run = 1;
		while (run * fat_clus_bytes < left && !fat_eoc(fat_get(c + run - 1)) && fat_get(c + run - 1) == c + run) {
			run++;
		}
		uint32_t len = (uint32_t)MIN((uint64_t)run * fat_clus_bytes, (uint64_t)left);
		fat_ext[fat_next].off = (uint32_t)fat_clus_off(c);
		fat_ext[fat_next].len = len;
		fat_next++;
		left -= len;
		c = fat_get(c + run - 1);
	}
	size_t pl = strlen(path) + 1;
	char *p = (char *)IOMalloc(pl);
	if (p == NULL) {
		return false;
	}
	memcpy(p, path, pl);
	fat_all[fat_nfiles].path = p;
	fat_all[fat_nfiles].size = size;
	fat_all[fat_nfiles].ext = (uint16_t)start;
	fat_all[fat_nfiles].next = (uint16_t)(fat_next - start);
	fat_nfiles++;
	return true;
}

/* one directory (the FAT16 root when clus == 0) into the tables, sub-directories after */
static void
fat_walk(uint32_t clus, const char *prefix, int depth)
{
	uint8_t *buf = (uint8_t *)IOMalloc(fat_clus_bytes > fat_root_bytes ? fat_clus_bytes : fat_root_bytes);
	if (buf == NULL || depth > 16) {
		if (buf) IOFree(buf, fat_clus_bytes > fat_root_bytes ? fat_clus_bytes : fat_root_bytes);
		return;
	}
	const uint32_t bsz = fat_clus_bytes > fat_root_bytes ? fat_clus_bytes : fat_root_bytes;
	char lfn[256];
	bool have_lfn = false;
	uint32_t c = clus, guard = 0;
	bool root16 = clus == 0;
	for (;;) {
		uint32_t n = root16 ? fat_root_bytes : fat_clus_bytes;
		uint64_t off = root16 ? fat_root_off : fat_clus_off(c);
		if (sweet_ufs_read_part(fw_part, off, buf, n) != 0) {
			break;
		}
		for (uint32_t e = 0; e + 32 <= n; e += 32) {
			const uint8_t *d = buf + e;
			if (d[0] == 0) {
				goto done;
			}
			if (d[0] == 0xE5) {
				have_lfn = false;
				continue;
			}
			if (d[11] == 0x0F) {                    /* long name piece: 13 UCS-2 characters */
				int seq = (d[0] & 0x1F) - 1;
				static const uint8_t at[13] = { 1, 3, 5, 7, 9, 14, 16, 18, 20, 22, 24, 28, 30 };
				if (d[0] & 0x40) {
					memset(lfn, 0, sizeof(lfn));
					have_lfn = true;
				}
				for (int k = 0; k < 13 && seq >= 0 && seq * 13 + k < 255; k++) {
					uint16_t ch = (uint16_t)(d[at[k]] | d[at[k] + 1] << 8);
					if (ch == 0 || ch == 0xFFFF) {
						break;
					}
					lfn[seq * 13 + k] = ch < 0x80 ? (char)((ch >= 'A' && ch <= 'Z') ? ch + 32 : ch) : '_';
				}
				continue;
			}
			if (d[11] & 0x08) {                     /* the volume label */
				have_lfn = false;
				continue;
			}
			char name[256];
			if (have_lfn) {
				strlcpy(name, lfn, sizeof(name));
			} else {
				unsigned k = 0;
				for (int i = 0; i < 8 && d[i] != ' '; i++) {
					name[k++] = (char)((d[i] >= 'A' && d[i] <= 'Z') ? d[i] + 32 : d[i]);
				}
				if (d[8] != ' ') {
					name[k++] = '.';
					for (int i = 8; i < 11 && d[i] != ' '; i++) {
						name[k++] = (char)((d[i] >= 'A' && d[i] <= 'Z') ? d[i] + 32 : d[i]);
					}
				}
				name[k] = 0;
			}
			have_lfn = false;
			if (strcmp(name, ".") == 0 || strcmp(name, "..") == 0) {
				continue;
			}
			uint32_t first = fat_le(d + 26, 2) | (fat_bits == 32 ? fat_le(d + 20, 2) << 16 : 0);
			char path[300];
			snprintf(path, sizeof(path), "%s%s%s", prefix, *prefix ? "/" : "", name);
			if (d[11] & 0x10) {
				if (first >= 2) {
					fat_walk(first, path, depth + 1);
				}
			} else if (!fat_add_file(path, first, fat_le(d + 28, 4))) {
				LOG("firmware: FAT: %s not added (tables full or broken chain)", path);
			}
		}
		if (root16) {
			break;
		}
		c = fat_get(c);
		if (fat_eoc(c) || ++guard > fat_nclus) {
			break;
		}
	}
done:
	IOFree(buf, bsz);
}

static bool
fw_fat_build(void)
{
	uint8_t bs[512];
	if (sweet_ufs_read_part(fw_part, 0, bs, sizeof(bs)) != 0 || bs[510] != 0x55 || bs[511] != 0xAA) {
		return false;
	}
	fat_bps = fat_le(bs + 11, 2);
	uint32_t spc = bs[13], rsvd = fat_le(bs + 14, 2), nfats = bs[16], rootent = fat_le(bs + 17, 2);
	uint32_t total = fat_le(bs + 19, 2) ? fat_le(bs + 19, 2) : fat_le(bs + 32, 4);
	uint32_t fatsz = fat_le(bs + 22, 2) ? fat_le(bs + 22, 2) : fat_le(bs + 36, 4);
	if (fat_bps < 512 || fat_bps > 4096 || spc == 0 || nfats == 0 || fatsz == 0 || total == 0) {
		return false;
	}
	fat_root_bytes = rootent * 32;
	uint32_t root_secs = (fat_root_bytes + fat_bps - 1) / fat_bps;
	uint32_t data_sec = rsvd + nfats * fatsz + root_secs;
	fat_nclus = (total - data_sec) / spc;
	fat_bits = fat_nclus < 4085 ? 12 : fat_nclus < 65525 ? 16 : 32;
	if (fat_bits == 12) {
		return false;
	}
	fat_clus_bytes = spc * fat_bps;
	fat_root_off = (uint64_t)(rsvd + nfats * fatsz) * fat_bps;
	fat_data_off = (uint64_t)data_sec * fat_bps;
	fat_root_clus = fat_bits == 32 ? fat_le(bs + 44, 4) : 0;
	uint32_t fat_bytes = fatsz * fat_bps;
	fat_tab = (uint8_t *)IOMalloc(fat_bytes);
	fat_ext = (struct fw_extent *)IOMalloc(sizeof(*fat_ext) * FW_FAT_MAX_EXT);
	fat_all = (struct fw_all_file *)IOMalloc(sizeof(*fat_all) * FW_FAT_MAX_FILES);
	if (fat_tab == NULL || fat_ext == NULL || fat_all == NULL ||
	    sweet_ufs_read_part(fw_part, (uint64_t)rsvd * fat_bps, fat_tab, fat_bytes) != 0) {
		LOG("firmware: FAT: no memory or the FAT is not readable");
		return false;
	}
	fat_walk(fat_root_clus, "", 0);
	IOFree(fat_tab, fat_bytes);
	fat_tab = NULL;
	if (fat_nfiles == 0) {
		return false;
	}
	fw_ext_tab[FW_FAT] = fat_ext;
	fw_all_tab[FW_FAT] = fat_all;
	fw_all_count[FW_FAT] = fat_nfiles;
	LOG("firmware: FAT%u volume, %u files, %u extents, cluster %u bytes", fat_bits, fat_nfiles, fat_next, fat_clus_bytes);
	return true;
}

/* the modem's own files ("modem.mdt", "modem.b03") when the tables came from the FAT */
static int64_t
fw_fat_size(const char *name)
{
	char path[64];
	snprintf(path, sizeof(path), "image/%s", name);
	int i = fw_all_find(path);
	return i < 0 ? -1 : (int64_t)fw_all[i].size;
}

static int64_t
fw_fat_read(const char *name, void *dst, uint64_t max)
{
	char path[64];
	snprintf(path, sizeof(path), "image/%s", name);
	int i = fw_all_find(path);
	if (i < 0 || fw_all[i].size > max) {
		LOG("%s: not in %s or too big", name, fw_part);
		return -1;
	}
	if (!fw_all_read(i, 0, dst, fw_all[i].size)) {
		LOG("%s: read from %s failed", name, fw_part);
		return -1;
	}
	return fw_all[i].size;
}

/*
 * 0248: the modem partition's files for SweetQsee.cpp (the goodixfp TrustZone app). fw_pick()
 * runs here first when the modem thread has not run it (no modem= / audio=). Size: bytes, -1 not
 * in the table, -2 the partition not readable (yet).
 */
extern "C" int64_t
topaz_fw_file_size(const char *path)
{
	if (!fw_picked) {
		uint8_t probe[16];
		if (sweet_ufs_read_part(fw_part, 0, probe, sizeof(probe)) != 0) {
			return -2;
		}
		fw_pick();
	}
	int i = fw_all_find(path);
	return i < 0 ? -1 : (int64_t)fw_all[i].size;
}

extern "C" int
topaz_fw_file_read(const char *path, uint64_t off, void *dst, uint64_t n)
{
	int i = fw_picked ? fw_all_find(path) : -1;
	if (i < 0 || off > fw_all[i].size || n > fw_all[i].size - off) {
		return EINVAL;
	}
	return fw_all_read(i, off, dst, n) ? 0 : EIO;
}

extern "C" const char *
topaz_fw_phone(void)
{
	return !fw_picked ? "(not picked)" : fw_phone == FW_FAT ? "FAT" : fw_tables[fw_phone].phone;
}

/* QMI */
struct qmsg { uint8_t b[512]; uint32_t n; };

static void
qm_init(struct qmsg *m, uint16_t txn, uint16_t msg)
{
	m->b[0] = QMI_RESP;
	memcpy(m->b + 1, &txn, 2);
	memcpy(m->b + 3, &msg, 2);
	m->n = 7;
}

static void
qm_tlv(struct qmsg *m, uint8_t type, const void *v, uint16_t len)
{
	if (m->n + 3 + len > sizeof(m->b)) {
		return;
	}
	m->b[m->n] = type;
	memcpy(m->b + m->n + 1, &len, 2);
	memcpy(m->b + m->n + 3, v, len);
	m->n += 3 + len;
}

static void
qm_result(struct qmsg *m, bool ok)
{
	uint16_t r[2] = { (uint16_t)(ok ? 0 : 1), (uint16_t)(ok ? 0 : 1) };
	qm_tlv(m, 2, r, 4);
}

static void
qm_send(struct qmsg *m, uint32_t src_port, uint32_t node, uint32_t port)
{
	uint16_t l = (uint16_t)(m->n - 7);
	memcpy(m->b + 5, &l, 2);
	qrtr_send(QRTR_TYPE_DATA, src_port, node, port, m->b, m->n);
}

static const uint8_t *
qm_find(const uint8_t *d, uint32_t len, uint8_t type, uint16_t *tlen)
{
	uint32_t o = 7;
	while (o + 3 <= len) {
		uint16_t l;
		memcpy(&l, d + o + 1, 2);
		if (o + 3 + l > len) {
			break;
		}
		if (d[o] == type) {
			*tlen = l;
			return d + o + 3;
		}
		o += 3 + l;
	}
	return NULL;
}

static void
qm_string(const uint8_t *d, uint32_t len, uint8_t type, char *out, size_t max)
{
	uint16_t l = 0;
	const uint8_t *v = qm_find(d, len, type, &l);
	out[0] = 0;
	if (v != NULL) {
		l = (uint16_t)MIN((size_t)l, max - 1);
		memcpy(out, v, l);
		out[l] = 0;
	}
}

/* pd-mapper */
static const struct { const char *svc, *domain; uint32_t inst; } pd_map[] = {
	{ "tms/servreg",       "msm/modem/root_pd", 180 },
	{ "tms/pdr_enabled",   "msm/modem/root_pd", 180 },
	{ "gps/gps_service",   "msm/modem/root_pd", 180 },
	{ "kernel/elf_loader", "msm/modem/wlan_pd", 180 },
	{ "tms/servreg",       "msm/modem/wlan_pd", 180 },
	{ "wlan/fw",           "msm/modem/wlan_pd", 180 },
	/* the ADSP's (image/adspr.jsn, adsps.jsn, adspua.jsn) */
	{ "tms/servreg",       "msm/adsp/root_pd", 74 },
	{ "tms/servreg",       "msm/adsp/sensor_pd", 74 },
	{ "tms/servreg",       "msm/adsp/audio_pd", 74 },
	{ "avs/audio",         "msm/adsp/audio_pd", 74 },
};
static unsigned pd_reqs;

static void
pdm_rx(uint32_t node, uint32_t port, uint16_t txn, uint16_t msg, const uint8_t *d, uint32_t len)
{
	struct qmsg m;
	char name[80];
	uint8_t list[256];
	uint32_t n = 1, cnt = 0;
	uint16_t v;

	qm_init(&m, txn, msg);
	if (msg != 0x21) {                      /* PFR etc.: just say OK */
		LOG("t=%u ms: pd-mapper: msg %x", g_ms, msg);
		qm_result(&m, true);
		qm_send(&m, PORT_PDM, node, port);
		return;
	}
	qm_string(d, len, 1, name, sizeof(name));
	for (unsigned i = 0; i < sizeof(pd_map) / sizeof(pd_map[0]); i++) {
		if (strcmp(pd_map[i].svc, name) == 0) {
			uint32_t dl = (uint32_t)strlen(pd_map[i].domain), inst = pd_map[i].inst, zero = 0;
			list[n++] = (uint8_t)dl;
			memcpy(list + n, pd_map[i].domain, dl);
			n += dl;
			memcpy(list + n, &inst, 4);
			n += 4;
			list[n++] = 0;                  /* service_data_valid */
			memcpy(list + n, &zero, 4);
			n += 4;
			cnt++;
		}
	}
	list[0] = (uint8_t)cnt;
	qm_result(&m, true);
	v = (uint16_t)cnt;
	qm_tlv(&m, 0x10, &v, 2);
	v = 1;
	qm_tlv(&m, 0x11, &v, 2);
	if (cnt != 0) {
		qm_tlv(&m, 0x12, list, (uint16_t)n);
	}
	qm_send(&m, PORT_PDM, node, port);
	pd_reqs++;
	LOG("t=%u ms: pd-mapper: \"%s\" -> %u domain(s)", g_ms, name, cnt);
}

/* tftp */
#define OP_RRQ   1
#define OP_WRQ   2
#define OP_DATA  3
#define OP_ACK   4
#define OP_ERROR 5
#define OP_OACK  6

struct tftp_sess {
	bool used, write;
	uint32_t port, node, rport;
	int fi;                                 /* fw_all index, -1 = none */
	int rw;                                 /* /readwrite file in RAM, -1 = none */
	uint64_t size, sent;
	uint32_t blk, rsize, wsize, seek;
};
static struct tftp_sess sess[16];
static uint32_t next_sess_port = PORT_SESS;
static unsigned tftp_reqs, tftp_missing, tftp_writes;

struct rw_file { char path[96]; uint8_t *data; uint32_t size, cap; };
static struct rw_file rw_files[12];

static int
rw_find(const char *path, bool create)
{
	for (int i = 0; i < 12; i++) {
		if (rw_files[i].path[0] != 0 && strcmp(rw_files[i].path, path) == 0) {
			return i;
		}
	}
	for (int i = 0; create && i < 12; i++) {
		if (rw_files[i].path[0] == 0) {
			strlcpy(rw_files[i].path, path, sizeof(rw_files[i].path));
			return i;
		}
	}
	return -1;
}

static bool
rw_write(int r, uint64_t off, const uint8_t *src, uint32_t n)
{
	struct rw_file *f = &rw_files[r];
	uint64_t end = off + n;
	if (off > 0x100000 || n > 0x100000) {
		return false;
	}
	if (end > f->cap) {
		uint32_t cap = (uint32_t)MAX(end, (uint64_t)f->cap * 2);
		uint8_t *d = (uint8_t *)IOMalloc(cap);
		if (d == NULL) {
			return false;
		}
		bzero(d, cap);
		if (f->data != NULL) {
			memcpy(d, f->data, f->size);
			IOFree(f->data, f->cap);
		}
		f->data = d;
		f->cap = cap;
	}
	memcpy(f->data + off, src, n);
	f->size = (uint32_t)MAX((uint64_t)f->size, end);
	return true;
}

static void
tftp_send(struct tftp_sess *s, const void *p, uint32_t len)
{
	qrtr_send(QRTR_TYPE_DATA, s->port, s->node, s->rport, p, len);
}

static void
tftp_error(struct tftp_sess *s, uint16_t code, const char *msg)
{
	uint8_t b[64] = { 0, OP_ERROR, (uint8_t)(code >> 8), (uint8_t)code };
	uint32_t l = (uint32_t)strlcpy((char *)b + 4, msg, sizeof(b) - 4) + 1;
	tftp_send(s, b, 4 + l);
}

static void
tftp_close(struct tftp_sess *s)
{
	bzero(s, sizeof(*s));
}

/* tqftpserv tftp_send_data() */
static bool
tftp_data(struct tftp_sess *s, uint32_t block, uint64_t offset, uint32_t resp_size)
{
	uint8_t *b = (uint8_t *)IOMalloc(s->blk + 4);
	uint32_t n = s->blk, send;

	if (b == NULL) {
		return false;
	}
	b[0] = 0; b[1] = OP_DATA; b[2] = (uint8_t)(block >> 8); b[3] = (uint8_t)block;
	if (offset >= s->size) {
		n = 0;
	} else {
		n = (uint32_t)MIN((uint64_t)n, s->size - offset);
		if (s->rw >= 0) {
			memcpy(b + 4, rw_files[s->rw].data + offset, n);
		} else if (!fw_all_read(s->fi, offset, b + 4, n)) {
			LOG("tftp sess %x: read FAILED at %llu", s->port, offset);
			IOFree(b, s->blk + 4);
			tftp_error(s, 0, "Read error");
			return false;
		}
	}
	send = 4 + n;
	if (resp_size != 0) {
		if (4 + resp_size > send) {
			IOFree(b, s->blk + 4);
			return false;
		}
		send = 4 + resp_size;
	}
	tftp_send(s, b, send);
	s->sent += send - 4;
	IOFree(b, s->blk + 4);
	return true;
}

static char *
tftp_opt(char *p, char *end, const char *name, uint64_t val)
{
	char num[24];
	snprintf(num, sizeof(num), "%llu", val);
	size_t a = strlen(name) + 1, b = strlen(num) + 1;
	if (p + a + b > end) {
		return p;
	}
	memcpy(p, name, a);
	memcpy(p + a, num, b);
	return p + a + b;
}

/* the modem's path -> fw_all path: /readonly/firmware/image/x -> image/x */
static bool
tftp_map(const char *path, char *out, size_t max)
{
	static const char *pre[] = {
		"/readonly/firmware/image/", "/readonly/vendor/firmware_mnt/image/", "/readonly/vendor/firmware/",
	};
	const char *rest = NULL;
	size_t k;

	for (unsigned i = 0; i < 3; i++) {
		if (strncmp(path, pre[i], strlen(pre[i])) == 0) {
			rest = path + strlen(pre[i]);
			strlcpy(out, "image/", max);
			break;
		}
	}
	if (rest == NULL && strncmp(path, "/readonly/firmware/", 19) == 0) {
		rest = path + 19;
		out[0] = 0;
	}
	if (rest == NULL) {
		return false;
	}
	k = strlen(out);
	for (; *rest != 0 && k + 1 < max; rest++) {
		char c = *rest;
		out[k++] = (c >= 'A' && c <= 'Z') ? (char)(c - 'A' + 'a') : c;
	}
	out[k] = 0;
	return true;
}

static uint32_t
dec(const char *p)
{
	uint32_t v = 0;
	while (*p >= '0' && *p <= '9') {
		v = v * 10 + (uint32_t)(*p++ - '0');
	}
	return v;
}

static void
tftp_request(uint32_t node, uint32_t port, const uint8_t *d, uint32_t len)
{
	char buf[512], *p, *end, *file, *mode, oack[256], *o, path[128];
	uint16_t op = (uint16_t)(d[0] << 8 | d[1]);
	struct tftp_sess *s = NULL;
	bool opts, want_tsize = false;
	uint32_t timeoutms = 1000;

	if (len < 4 || len >= sizeof(buf)) {
		return;
	}
	memcpy(buf, d, len);
	buf[len] = 0;
	end = buf + len;
	file = buf + 2;
	mode = file + strlen(file) + 1;
	if (mode >= end) {
		return;
	}
	p = mode + strlen(mode) + 1;
	opts = p < end;
	for (unsigned i = 0; i < 16; i++) {
		if (!sess[i].used) {
			s = &sess[i];
			break;
		}
	}
	if (s == NULL) {
		LOG("tftp: out of sessions");
		return;
	}
	bzero(s, sizeof(*s));
	s->used = true;
	s->write = op == OP_WRQ;
	s->port = next_sess_port++;
	s->node = node;
	s->rport = port;
	s->blk = 512;
	s->wsize = 1;
	s->rw = -1;
	s->fi = -1;
	while (p < end) {
		char *name = p, *val = p + strlen(p) + 1;
		if (val >= end) {
			break;
		}
		uint32_t v = dec(val);
		if (strcmp(name, "blksize") == 0) {
			s->blk = MAX(8u, MIN(v, 16384u));
		} else if (strcmp(name, "timeoutms") == 0) {
			timeoutms = v;
		} else if (strcmp(name, "tsize") == 0) {
			want_tsize = true;
		} else if (strcmp(name, "rsize") == 0) {
			s->rsize = v;
		} else if (strcmp(name, "wsize") == 0) {
			s->wsize = MAX(1u, v);
		} else if (strcmp(name, "seek") == 0) {
			s->seek = v;
		}
		p = val + strlen(val) + 1;
	}
	tftp_reqs++;
	if (strncmp(file, "/readwrite/", 11) == 0) {
		s->rw = rw_find(file, s->write);
	}
	if (s->write) {
		tftp_writes++;
		LOG("t=%u ms: tftp WRQ %s seek %u%s", g_ms, file, s->seek, s->rw < 0 ? " (no RAM slot, dropped)" : "");
		if (s->rw >= 0 && s->seek == 0) {
			rw_files[s->rw].size = 0;       /* a write from the start replaces the file */
		}
	} else if (s->rw >= 0) {
		s->size = rw_files[s->rw].size;
		LOG("t=%u ms: tftp RRQ %s (RAM, %llu B)", g_ms, file, s->size);
	} else {
		if (!tftp_map(file, path, sizeof(path)) || (s->fi = fw_all_find(path)) < 0) {
			tftp_missing++;
			LOG("t=%u ms: tftp RRQ %s: not found", g_ms, file);
			tftp_error(s, 1, "file not found");
			tftp_close(s);
			return;
		}
		s->size = fw_all[s->fi].size;
		LOG("t=%u ms: tftp RRQ %s seek %u rsize %u blk %u w %u (%llu B)", g_ms, path, s->seek, s->rsize, s->blk, s->wsize, s->size);
	}
	if (!opts) {
		if (s->write) {
			uint8_t ack[4] = { 0, OP_ACK, 0, 0 };
			tftp_send(s, ack, 4);
		} else {
			tftp_data(s, 1, 0, 0);
		}
		return;
	}
	o = oack;
	*o++ = 0;
	*o++ = OP_OACK;
	o = tftp_opt(o, oack + sizeof(oack), "blksize", s->blk);
	o = tftp_opt(o, oack + sizeof(oack), "timeoutms", timeoutms);
	if (want_tsize && !s->write) {
		o = tftp_opt(o, oack + sizeof(oack), "tsize", s->size);
	}
	o = tftp_opt(o, oack + sizeof(oack), "wsize", s->wsize);
	if (s->rsize != 0) {
		o = tftp_opt(o, oack + sizeof(oack), "rsize", s->rsize);
	}
	if (s->seek != 0) {
		o = tftp_opt(o, oack + sizeof(oack), "seek", s->seek);
	}
	tftp_send(s, oack, (uint32_t)(o - oack));
}

/* traffic on a session port: ACKs for reads, DATA for writes (tqftpserv handle_reader) */
static bool
tftp_session(uint32_t dst_port, const uint8_t *d, uint32_t len)
{
	struct tftp_sess *s = NULL;
	for (unsigned i = 0; i < 16; i++) {
		if (sess[i].used && sess[i].port == dst_port) {
			s = &sess[i];
			break;
		}
	}
	if (s == NULL || len < 4) {
		return s != NULL;
	}
	uint16_t op = (uint16_t)(d[0] << 8 | d[1]), blk = (uint16_t)(d[2] << 8 | d[3]);
	if (op == OP_ERROR) {
		/* rsize 0 + tsize is a stat: the modem ends it with ERROR 9 "End of Transfer" */
		if (blk != 9) {
			LOG("t=%u ms: tftp sess %x: modem ERROR %u after %llu B", g_ms, s->port, blk, s->sent);
		}
		tftp_close(s);
		return true;
	}
	if (s->write) {
		if (op == OP_DATA) {
			uint32_t payload = len - 4;
			if (s->rw >= 0 && blk >= 1) {
				(void)rw_write(s->rw, s->seek + (uint64_t)(blk - 1) * s->blk, d + 4, payload);
			}
			if ((blk % s->wsize) == 0 || payload < s->blk) {
				uint8_t ack[4] = { 0, OP_ACK, (uint8_t)(blk >> 8), (uint8_t)blk };
				tftp_send(s, ack, 4);
			}
			if (payload < s->blk) {
				tftp_close(s);
			}
		}
		return true;
	}
	if (op != OP_ACK) {
		tftp_error(s, 4, "Expected ACK opcode");
		tftp_close(s);
		return true;
	}
	if (s->rsize == 0) {
		/* the whole file: blocks until a short one */
		uint64_t total = s->size > s->seek ? s->size - s->seek : 0;
		uint32_t last = (uint32_t)(total / s->blk) + 1;
		if ((uint32_t)blk >= last) {
			tftp_close(s);
			return true;
		}
		for (uint32_t i = blk + 1u; i <= blk + s->wsize && i <= last; i++) {
			if (!tftp_data(s, i, s->seek + (uint64_t)(i - 1) * s->blk, 0)) {
				break;
			}
		}
		return true;
	}
	if ((uint64_t)blk * s->blk > s->rsize) {
		tftp_close(s);
		return true;
	}
	for (uint32_t i = blk; i < blk + s->wsize; i++) {
		uint64_t off = s->seek + (uint64_t)i * s->blk;
		uint32_t rs = (i + 1) * s->blk > s->rsize ? s->rsize % s->blk : 0;
		if (!tftp_data(s, i + 1, off, rs) || (i + 1) * s->blk > s->rsize) {
			break;
		}
	}
	return true;
}

/* rmtfs */
#define RMTFS_BUF_SIZE      0x300000            /* qcom,rmtfs_sharedmem size */
#define VMID_HLOS           3
#define VMID_MSS_MSA        15
#define VMID_NAV            0x2B
#define PERM_RW             6

static const struct { const char *path, *part; } rmtfs_parts[] = {
	{ "/boot/modem_fs1", "modemst1" },
	{ "/boot/modem_fs2", "modemst2" },
	{ "/boot/modem_fsc", "fsc" },
	{ "/boot/modem_fsg", "fsg" },
};
static uint64_t rmtfs_pa;
static uint8_t *rmtfs_va;
static unsigned rmtfs_reads, rmtfs_writes, rmtfs_err;

/* hyp_assign_phys (SCM 0x0C/0x16): from HLOS to the given VMIDs, read-write */
static uint32_t scm_perm = PERM_RW;            /* what scm_assign() gives each destination */

static uint64_t
scm_assign(uint64_t addr, uint64_t size, const uint32_t *vmids, unsigned count)
{
	uint64_t pa = 0, res = 0;
	uint64_t *p = (uint64_t *)pool_alloc(0x1000, &pa);

	if (p == NULL) {
		return ~0ull;
	}
	p[0] = addr;                                /* 0x000: mem map {addr, size} */
	p[1] = size;
	uint32_t *src = (uint32_t *)(p + 8);        /* 0x040: src vmids */
	src[0] = VMID_HLOS;
	uint32_t *dst = (uint32_t *)(p + 16);       /* 0x080: {vmid, perm, ctx(8), ctx_size, unused} x3 */
	for (unsigned i = 0; i < count && i < 8; i++) {
		dst[6 * i] = vmids[i];
		dst[6 * i + 1] = scm_perm;
	}
	p[32] = 4;                                  /* 0x100: extended args: src size */
	p[33] = pa + 0x80;                          /* dest */
	p[34] = count * 24;                         /* dest size */
	p[35] = 0;
	mb();
	uint64_t args[8] = { SCM_FN(0x0C, 0x16), 7 | (1 << 4) | (1 << 8) | (1 << 12), pa, 16, pa + 0x40, pa + 0x100, 0, 0 };
	smc(args);
	for (unsigned n = 0; args[0] == 1 && n < 100000; n++) {
		uint64_t again[8] = { 1, 7 | (1 << 4) | (1 << 8) | (1 << 12), pa, 16, pa + 0x40, pa + 0x100, args[6], 0 };
		memcpy(args, again, sizeof(args));
		smc(args);
	}
	res = args[1];
	LOG("SCM assign %llx+%llx -> %u vmids: ret=%llx res=%llx", addr, size, count, args[0], res);
	return args[0] != 0 ? args[0] : res;
}

static void
rmtfs_rx(uint32_t node, uint32_t port, uint16_t txn, uint16_t msg, const uint8_t *d, uint32_t len)
{
	struct qmsg m;
	char path[64];
	uint16_t l = 0;
	const uint8_t *v;
	uint32_t caller = 0, i;
	bool ok = true;

	qm_init(&m, txn, msg);
	v = qm_find(d, len, 1, &l);
	if (msg != 1 && v != NULL && l >= 4) {
		memcpy(&caller, v, 4);
	}
	switch (msg) {
	case 1:                                     /* OPEN */
		qm_string(d, len, 1, path, sizeof(path));
		for (i = 0; i < 4; i++) {
			if (strcmp(path, rmtfs_parts[i].path) == 0) {
				break;
			}
		}
		if (i < 4) {
			uint8_t probe[16];
			ok = sweet_ufs_read_part(rmtfs_parts[i].part, 0, probe, sizeof(probe)) == 0;
		} else {
			ok = false;
		}
		LOG("t=%u ms: rmtfs: open %s -> %s", g_ms, path, ok ? rmtfs_parts[i].part : "FAILED");
		qm_result(&m, ok);
		if (ok) {
			caller = i + 1;
			qm_tlv(&m, 0x10, &caller, 4);
		}
		break;
	case 2:                                     /* CLOSE */
		qm_result(&m, true);
		break;
	case 3: {                                   /* RW_IOVEC */
		uint16_t l2 = 0;
		const uint8_t *dir = qm_find(d, len, 2, &l2);
		const uint8_t *iov = qm_find(d, len, 3, &l2);
		bool write = dir != NULL && *dir != 0;
		ok = caller >= 1 && caller <= 4 && iov != NULL && rmtfs_pa != 0;
		for (i = 0; ok && i < iov[0]; i++) {
			uint32_t e[3];                      /* sector, phys, count */
			memcpy(e, iov + 1 + 12 * i, 12);
			uint64_t bytes = (uint64_t)e[2] * 512;
			if (e[1] < rmtfs_pa || e[1] + bytes > rmtfs_pa + RMTFS_BUF_SIZE) {
				ok = false;
				break;
			}
			if (write) {
				rmtfs_writes++;                 /* never written to the phone's EFS */
			} else {
				ok = sweet_ufs_read_part(rmtfs_parts[caller - 1].part, (uint64_t)e[0] * 512,
				    rmtfs_va + (e[1] - rmtfs_pa), bytes) == 0;
				rmtfs_reads++;
			}
		}
		if (!ok && ++rmtfs_err <= 3) {
			LOG("t=%u ms: rmtfs: iovec caller %u %s FAILED", g_ms, caller, write ? "write" : "read");
		}
		qm_result(&m, ok);
		break;
	}
	case 4: {                                   /* ALLOC_BUFF */
		if (rmtfs_pa == 0) {
			rmtfs_va = (uint8_t *)pool_alloc(RMTFS_BUF_SIZE, &rmtfs_pa);   /* the modem owns it from now on */
			if (rmtfs_va == NULL) {
				rmtfs_pa = 0;
			} else {
				static const uint32_t v[] = { VMID_HLOS, VMID_MSS_MSA, VMID_NAV };
				(void)scm_assign(rmtfs_pa, RMTFS_BUF_SIZE, v, 3);
			}
		}
		LOG("t=%u ms: rmtfs: alloc for caller %u -> %llx", g_ms, caller, rmtfs_pa);
		ok = rmtfs_pa != 0;
		qm_result(&m, ok);
		if (ok) {
			uint64_t a = rmtfs_pa;
			qm_tlv(&m, 0x10, &a, 8);
		}
		break;
	}
	case 5: {                                   /* GET_DEV_ERROR */
		uint8_t st = 0;
		qm_result(&m, true);
		qm_tlv(&m, 0x10, &st, 1);
		break;
	}
	default:
		LOG("t=%u ms: rmtfs: msg %x", g_ms, msg);
		qm_result(&m, false);
		break;
	}
	qm_send(&m, PORT_RMTFS, node, port);
}

/* ---------------------------------------------------------------- WLFW (QMI 0x45) */

/*
 * The WLAN firmware service, the way ath10k qmi.c talks to it (port of topaz-woa Wlfw.c):
 * ind_register -> host_cap -> msa_info (+ assign the MSA regions) -> msa_ready -> cap ->
 * MSA_READY ind -> board data (bdwlan.*) -> cal_report -> FW_READY ind -> wlan_cfg (CE
 * layout) -> wlan_mode mission. Nothing touches the WLAN registers here (CE comes next and
 * only after mission mode: before it a register read hangs the bus).
 */
#define PORT_WLFWC          0x4004              /* our QMI client port towards WLFW */
#define QMI_IND             4
#define WLFW_IND_REGISTER   0x20
#define WLFW_FW_READY_IND   0x21
#define WLFW_WLAN_MODE      0x22
#define WLFW_WLAN_CFG       0x23
#define WLFW_CAP            0x24
#define WLFW_BDF_DOWNLOAD   0x25
#define WLFW_CAL_REPORT     0x26
#define WLFW_MSA_READY_IND  0x2B
#define WLFW_MSA_INFO       0x2D
#define WLFW_MSA_READY      0x2E
#define WLFW_HOST_CAP       0x34
#define WLFW_MSA_BASE       0x51900000u         /* DTB wlan_msa_region, inside "PIL Reserved" */
#define WLFW_MSA_SIZE       0x00100000u
#define WLFW_CLIENT_ID      0x4b4e454c          /* ath10k ATH10K_QMI_CLIENT_ID */
#define WLFW_SEG            6144                /* QMI_WLFW_MAX_DATA_SIZE_V01 */
#define VMID_WLAN           0x18
#define VMID_WLAN_CE        0x19

static uint32_t w_node, w_port, w_board = 0xFF, w_bdf_seg;
static uint16_t w_txn = 1;
static bool w_msa_ind, w_cap_done, w_bdf_started, w_fw_ready, w_mission;
static int w_bdf_fi = -1;
static uint32_t w_bdf_len, w_bdf_off;
static const char *w_step = "not started";

struct wmsg { uint8_t *b; uint32_t n; };

static bool
wm_new(struct wmsg *m, uint16_t msg)
{
	m->b = (uint8_t *)IOMalloc(WLFW_SEG + 128);
	if (m->b == NULL) {
		return false;
	}
	bzero(m->b, WLFW_SEG + 128);
	m->b[0] = QMI_REQ;
	uint16_t t = w_txn++;
	memcpy(m->b + 1, &t, 2);
	memcpy(m->b + 3, &msg, 2);
	m->n = 7;
	return true;
}

static void
wm_tlv(struct wmsg *m, uint8_t type, const void *v, uint16_t len)
{
	m->b[m->n] = type;
	memcpy(m->b + m->n + 1, &len, 2);
	if (len != 0) {
		memcpy(m->b + m->n + 3, v, len);
	}
	m->n += 3 + len;
}

static void wm_u8(struct wmsg *m, uint8_t t, uint8_t v) { wm_tlv(m, t, &v, 1); }
static void wm_u32(struct wmsg *m, uint8_t t, uint32_t v) { wm_tlv(m, t, &v, 4); }

/* TLV with a u8 element count in front of an array */
static void
wm_array(struct wmsg *m, uint8_t type, uint8_t count, const void *v, uint16_t bytes)
{
	uint16_t l = (uint16_t)(1 + bytes);
	m->b[m->n] = type;
	memcpy(m->b + m->n + 1, &l, 2);
	m->b[m->n + 3] = count;
	memcpy(m->b + m->n + 4, v, bytes);
	m->n += 4 + bytes;
}

static void
wm_send(struct wmsg *m)
{
	uint16_t l = (uint16_t)(m->n - 7);
	memcpy(m->b + 5, &l, 2);
	qrtr_send(QRTR_TYPE_DATA, PORT_WLFWC, w_node, w_port, m->b, m->n);
	IOFree(m->b, WLFW_SEG + 128);
}

static bool
w_result(const uint8_t *d, uint32_t len, uint16_t *err)
{
	uint16_t l = 0;
	const uint8_t *r = qm_find(d, len, 2, &l);
	uint16_t res = 0xFFFF;
	*err = 0xFFFF;
	if (r != NULL && l >= 4) {
		memcpy(&res, r, 2);
		memcpy(err, r + 2, 2);
	}
	return res == 0;
}

static void
w_simple(uint16_t msg, const char *step)
{
	struct wmsg m;
	if (wm_new(&m, msg)) {
		wm_send(&m);
		w_step = step;
	}
}

static void
w_bdf_segment(void)
{
	struct wmsg m;
	uint32_t chunk = MIN(w_bdf_len - w_bdf_off, (uint32_t)WLFW_SEG);
	bool end = w_bdf_off + chunk >= w_bdf_len;

	if (!wm_new(&m, WLFW_BDF_DOWNLOAD)) {
		return;
	}
	wm_u8(&m, 0x01, 1);                         /* valid */
	wm_u32(&m, 0x10, 0);                        /* file_id */
	wm_u32(&m, 0x11, w_bdf_len);                /* total_size */
	wm_u32(&m, 0x12, w_bdf_seg);                /* seg_id */
	uint8_t *t = m.b + m.n;                     /* data: u16 len + bytes */
	uint16_t tl = (uint16_t)(2 + chunk), dl = (uint16_t)chunk;
	t[0] = 0x13;
	memcpy(t + 1, &tl, 2);
	memcpy(t + 3, &dl, 2);
	if (!fw_all_read(w_bdf_fi, w_bdf_off, t + 5, chunk)) {
		LOG("wlfw: board data read failed");
		IOFree(m.b, WLFW_SEG + 128);
		return;
	}
	m.n += 5 + chunk;
	wm_u8(&m, 0x14, end ? 1 : 0);               /* end */
	wm_send(&m);
	w_bdf_off += chunk;
	w_bdf_seg++;
	w_step = "bdf download";
}

/* bdwlan.bNN for board ids < 0x100, bdwlan.NNN otherwise, bdwlan.bin as fallback */
static void
w_start_bdf(void)
{
	char name[40];
	if (w_bdf_started || !w_msa_ind || !w_cap_done) {
		return;
	}
	w_bdf_started = true;
	if (w_board == 0xFF) {
		snprintf(name, sizeof(name), "image/bdwlan.bin");
	} else if (w_board < 0x100) {
		snprintf(name, sizeof(name), "image/bdwlan.b%02x", w_board);
	} else {
		snprintf(name, sizeof(name), "image/bdwlan.%03x", w_board);
	}
	w_bdf_fi = fw_all_find(name);
	if (w_bdf_fi < 0) {
		LOG("wlfw: %s missing, using bdwlan.bin", name);
		snprintf(name, sizeof(name), "image/bdwlan.bin");
		w_bdf_fi = fw_all_find(name);
	}
	if (w_bdf_fi < 0) {
		LOG("wlfw: no board data file");
		return;
	}
	w_bdf_len = fw_all[w_bdf_fi].size;
	w_bdf_off = 0;
	w_bdf_seg = 0;
	LOG("t=%u ms: wlfw: board data %s, %u bytes", g_ms, name, w_bdf_len);
	w_bdf_segment();
}

/*
 * ath10k_snoc_wlan_enable(): the copy-engine layout (Linux snoc.c target_ce_config_wlan,
 * target_service_to_ce_map_wlan, target_shadow_reg_cfg_map). PIPEDIR 1 in, 2 out, 3 inout;
 * flags bit 3 = DIS_INTR.
 */
static const uint32_t w_tgt_ce[12][5] = {
	{ 0, 2, 32, 2048, 0 }, { 1, 1, 32, 2048, 0 }, { 2, 1, 64, 2048, 0 }, { 3, 2, 32, 2048, 0 },
	{ 4, 2, 256, 256, 8 }, { 5, 2, 1024, 64, 8 }, { 6, 3, 32, 16384, 0 }, { 7, 4, 0, 0, 8 },
	{ 8, 1, 32, 2048, 0 }, { 9, 1, 32, 2048, 0 }, { 10, 1, 32, 2048, 0 }, { 11, 1, 32, 2048, 0 },
};
static const uint32_t w_svc_ce[][3] = {
	{ 0x104, 2, 3 }, { 0x104, 1, 2 }, { 0x102, 2, 3 }, { 0x102, 1, 2 }, { 0x101, 2, 3 }, { 0x101, 1, 2 },
	{ 0x103, 2, 3 }, { 0x103, 1, 2 }, { 0x100, 2, 3 }, { 0x100, 1, 2 }, { 0x001, 2, 0 }, { 0x001, 1, 2 },
	{ 0xFE00, 2, 0 }, { 0xFE00, 1, 2 }, { 0x300, 2, 4 }, { 0x300, 1, 1 }, { 0xFE00, 2, 5 }, { 0x301, 1, 9 },
	{ 0x302, 1, 10 }, { 0x600, 1, 11 }, { 0, 0, 0 },
};
static const uint16_t w_shadow[12][2] = {
	{ 0, 0x3C }, { 3, 0x3C }, { 4, 0x3C }, { 5, 0x3C }, { 7, 0x3C }, { 1, 0x40 },
	{ 2, 0x40 }, { 7, 0x40 }, { 8, 0x40 }, { 9, 0x40 }, { 10, 0x40 }, { 11, 0x40 },
};

static void
w_on_cap(const uint8_t *d, uint32_t len)
{
	uint16_t l = 0;
	const uint8_t *v;
	char s[80];
	uint32_t a = 0, b = 0;

	if ((v = qm_find(d, len, 0x10, &l)) != NULL && l >= 8) {
		memcpy(&a, v, 4);
		memcpy(&b, v + 4, 4);
		LOG("wlfw: chip_id %x family %x", a, b);
	}
	if ((v = qm_find(d, len, 0x11, &l)) != NULL && l >= 4) {
		memcpy(&w_board, v, 4);
	}
	LOG("wlfw: board_id %x", w_board);
	if ((v = qm_find(d, len, 0x13, &l)) != NULL && l >= 5) {
		size_t n = MIN((size_t)v[4], MIN((size_t)(l - 5), sizeof(s) - 1));
		memcpy(s, v + 5, n);
		s[n] = 0;
		memcpy(&a, v, 4);
		LOG("wlfw: fw_version %x built %s", a, s);
	}
	if ((v = qm_find(d, len, 0x14, &l)) != NULL) {
		size_t n = MIN((size_t)l, sizeof(s) - 1);
		memcpy(s, v, n);
		s[n] = 0;
		LOG("wlfw: fw_build_id %s", s);
	}
}

static void
w_on_msa_info(const uint8_t *d, uint32_t len)
{
	uint16_t l = 0;
	const uint8_t *v = qm_find(d, len, 0x03, &l);
	uint32_t n = (v != NULL && l >= 1) ? v[0] : 0;

	LOG("wlfw: %u MSA region(s)", n);
	IOSleep(20);                                /* ath10k: give the modem time before the XPU update */
	for (uint32_t i = 0; i < n && 1 + 13 * (i + 1) <= l; i++) {
		const uint8_t *e = v + 1 + 13 * i;      /* u64 addr, u32 size, u8 secure */
		uint64_t addr;
		uint32_t size;
		memcpy(&addr, e, 8);
		memcpy(&size, e + 8, 4);
		uint8_t sec = e[12];
		static const uint32_t vm[3] = { VMID_MSS_MSA, VMID_WLAN, VMID_WLAN_CE };
		LOG(" MSA region %u: %llx + %x secure %u", i, addr, size, sec);
		if (addr < WLFW_MSA_BASE || addr + size > (uint64_t)WLFW_MSA_BASE + WLFW_MSA_SIZE) {
			LOG(" out of the MSA window, skipped");
			continue;
		}
		(void)scm_assign(addr, size, vm, sec ? 2 : 3);
	}
}

static unsigned w_arrivals, w_cap_tries, w_cap_retry_ms;

static void
wlfw_arrive(uint32_t node, uint32_t port)
{
	if ((w_node != 0 || w_port != 0) && w_node == node && w_port == port) {
		return;
	}
	if (w_arrivals++ != 0) {
		/* the WLAN PD restarted (new server port): start the handshake again */
		LOG("t=%u ms: WLFW back on port %x (restart %u, last step: %s): handshake again", g_ms, port, w_arrivals - 1, w_step);
		w_msa_ind = w_cap_done = w_bdf_started = w_fw_ready = w_mission = false;
		w_board = 0xFF;
		w_cap_tries = 0;
		w_cap_retry_ms = 0;
	}
	w_node = node;
	w_port = port;
	struct wmsg m;
	if (wm_new(&m, WLFW_IND_REGISTER)) {
		wm_u8(&m, 0x10, 1);                     /* fw_ready_enable */
		wm_u8(&m, 0x13, 1);                     /* msa_ready_enable */
		wm_u32(&m, 0x15, WLFW_CLIENT_ID);       /* client_id */
		wm_send(&m);
		w_step = "ind_register sent";
	}
}

static void
wlfw_rx(const uint8_t *d, uint32_t len)
{
	uint16_t msg, err = 0, l = 0;
	struct wmsg m;

	if (len < 7) {
		return;
	}
	memcpy(&msg, d + 3, 2);
	if (d[0] == QMI_IND) {
		if (msg == WLFW_MSA_READY_IND) {
			LOG("t=%u ms: wlfw: MSA_READY indication", g_ms);
			w_msa_ind = true;
			w_start_bdf();
		} else if (msg == WLFW_FW_READY_IND) {
			w_fw_ready = true;
			LOG("t=%u ms: *** WLAN FIRMWARE READY ***", g_ms);
			if (wm_new(&m, WLFW_WLAN_CFG)) {
				wm_array(&m, 0x11, 12, w_tgt_ce, sizeof(w_tgt_ce));
				wm_array(&m, 0x12, (uint8_t)(sizeof(w_svc_ce) / sizeof(w_svc_ce[0])), w_svc_ce, sizeof(w_svc_ce));
				wm_array(&m, 0x13, 12, w_shadow, sizeof(w_shadow));
				wm_send(&m);
				w_step = "wlan_cfg sent";
			}
		} else {
			LOG("wlfw: indication %x", msg);
		}
		return;
	}
	if (d[0] != QMI_RESP) {
		return;
	}
	bool ok = w_result(d, len, &err);
	switch (msg) {
	case WLFW_IND_REGISTER: {
		const uint8_t *v = qm_find(d, len, 0x10, &l);
		uint64_t st = 0;
		if (v != NULL && l >= 8) {
			memcpy(&st, v, 8);
		}
		LOG("t=%u ms: wlfw: ind_register %s err %u, fw_status %llx", g_ms, ok ? "ok" : "FAILED", err, st);
		if (wm_new(&m, WLFW_HOST_CAP)) {
			wm_u32(&m, 0x10, 0);                /* daemon_support = 0 */
			wm_send(&m);
			w_step = "host_cap sent";
		}
		break;
	}
	case WLFW_HOST_CAP:
		LOG("wlfw: host_cap %s err %u", ok ? "ok" : "rejected (not fatal)", err);
		if (wm_new(&m, WLFW_MSA_INFO)) {
			uint64_t a = WLFW_MSA_BASE;
			wm_tlv(&m, 0x01, &a, 8);            /* msa_addr */
			wm_u32(&m, 0x02, WLFW_MSA_SIZE);
			wm_send(&m);
			w_step = "msa_info sent";
		}
		break;
	case WLFW_MSA_INFO:
		LOG("wlfw: msa_info %s err %u", ok ? "ok" : "FAILED", err);
		if (ok) {
			w_on_msa_info(d, len);
			w_simple(WLFW_MSA_READY, "msa_ready sent");
		}
		break;
	case WLFW_MSA_READY:
		LOG("wlfw: msa_ready %s err %u", ok ? "ok" : "FAILED", err);
		w_simple(WLFW_CAP, "cap sent");
		break;
	case WLFW_CAP:
		LOG("t=%u ms: wlfw: cap %s err %u (try %u)", g_ms, ok ? "ok" : "FAILED", err, w_cap_tries + 1);
		if (ok) {
			w_on_cap(d, len);
		} else {
			char hx[3 * 24 + 1];
			uint32_t n = MIN(len, 24u);
			for (uint32_t i = 0; i < n; i++) {
				snprintf(hx + 3 * i, 4, "%02x ", d[i]);
			}
			hx[3 * n] = 0;
			LOG("wlfw: cap response %u bytes: %s", len, hx);
			if (++w_cap_tries < 5) {
				w_cap_retry_ms = g_ms + 500;    /* ask again from the poll loop */
				break;
			}
		}
		w_cap_done = true;
		w_step = "waiting MSA_READY ind";
		w_start_bdf();
		break;
	case WLFW_BDF_DOWNLOAD:
		if (!ok) {
			LOG("wlfw: bdf segment %u FAILED err %u", w_bdf_seg - 1, err);
			break;
		}
		if (w_bdf_off < w_bdf_len) {
			w_bdf_segment();
		} else {
			LOG("wlfw: board data sent in %u segment(s)", w_bdf_seg);
			if (wm_new(&m, WLFW_CAL_REPORT)) {
				uint8_t none = 0;
				wm_tlv(&m, 0x01, &none, 1);     /* meta_data_len = 0 */
				wm_send(&m);
				w_step = "cal_report sent";
			}
		}
		break;
	case WLFW_CAL_REPORT:
		LOG("wlfw: cal_report %s err %u, waiting for FW_READY", ok ? "ok" : "FAILED", err);
		w_step = "waiting FW_READY";
		break;
	case WLFW_WLAN_CFG:
		LOG("wlfw: wlan_cfg %s err %u", ok ? "ok" : "FAILED", err);
		if (ok && wm_new(&m, WLFW_WLAN_MODE)) {
			wm_u32(&m, 0x01, 0);                /* mission */
			wm_u8(&m, 0x10, 0);                 /* hw_debug */
			wm_send(&m);
			w_step = "wlan_mode sent";
		}
		break;
	case WLFW_WLAN_MODE:
		LOG("t=%u ms: wlfw: wlan_mode mission %s err %u", g_ms, ok ? "ok" : "FAILED", err);
		if (ok) {
			w_mission = true;
			w_step = "WLAN ON (mission)";
			LOG("*** WLAN ON (mission mode) ***");
			if (wifi_enabled()) {
				IOSleep(100);
				(void)ce_start();               /* only now: before mission a CE access hangs the bus */
			}
		}
		break;
	default:
		LOG("wlfw: response %x %s err %u", msg, ok ? "ok" : "FAILED", err);
		break;
	}
}

/* ---------------------------------------------------------------- WLAN radio (wifi=1) */

/*
 * After WLAN_MODE mission (port of topaz-woa wlanprobe.c, ce.c, htc.c, wmi.c, scan.c):
 *  - apps SMMU: the WLAN stream 0x1A0 gets an identity context bank (SCTLR.M=0); a global
 *    bypass S2CR is turned into a fault by this SMMU. Done before the modem boots.
 *  - copy engines at 0xC800000+0x240000, polled; rings and buffers from our pool (Normal NC).
 *  - HTC: READY on ep0, connect HTT_DATA and WMI_CONTROL, SETUP_COMPLETE_EX.
 *  - WMI (TLV): SERVICE_READY -> INIT -> READY, then a passive scan (nothing is transmitted);
 *    each BSS seen is logged once, its BSSID masked.
 */
#define SMMU_PA             0x0C600000ull
#define SMMU_SIZE           0x00080000u
#define WLAN_SID            0x1A0
#define WLAN_SID_MASK       0x1
#define CB_CLONE_CBAR       0x0001f000u         /* S1_TRANS_S2_FAULT, S1_MEMATTR WB */
#define CB_CLONE_CBA2R      0x00000001u         /* VA64 */
#define CB_CLONE_SCTLR      0x000000e0u         /* CFCFG|CFIE|CFRE, M=0: no translation */

static int wifi_on = -1;

static bool
wifi_enabled(void)
{
	if (wifi_on < 0) {
		uint32_t v = 0;
		wifi_on = PE_parse_boot_argn("wifi", &v, sizeof(v)) && v != 0;
	}
	return wifi_on > 0;
}

static inline uint32_t r32(volatile uint8_t *b, uint32_t o) { return *(volatile uint32_t *)(b + o); }
static inline void w32(volatile uint8_t *b, uint32_t o, uint32_t v) { mb(); *(volatile uint32_t *)(b + o) = v; }

static bool
smmu_sid_match(uint32_t smr, uint32_t sid, uint32_t mask)
{
	uint32_t id = smr & 0xFFFF, m = (smr >> 16) & 0x7FFF;
	return ((id ^ sid) & ~(m | mask) & 0x7FFF) == 0;
}

static bool
smmu_identity(uint32_t sid, uint32_t mask, const char *name)
{
	volatile uint8_t *s = (volatile uint8_t *)ml_io_map(SMMU_PA, SMMU_SIZE);
	if (s == NULL) {
		LOG("smmu: map failed");
		return false;
	}
	uint32_t cr0 = r32(s, 0x0), id0 = r32(s, 0x20), id1 = r32(s, 0x24);
	uint32_t nsmr = id0 & 0xFF, ncb = id1 & 0xFF;
	uint32_t psize = (id1 & (1u << 31)) ? 0x10000 : 0x1000, npage = 1u << (((id1 >> 28) & 7) + 1);
	uint32_t slot = ~0u, cb = ~0u, smr, s2cr;
	uint64_t used = 0;

	LOG("smmu: sCR0 %08x ID0 %08x ID1 %08x: %u SMRs, %u CBs, page %x x%u", cr0, id0, id1, nsmr, ncb, psize, npage);
	for (uint32_t i = 0; i < nsmr && i < 128; i++) {
		smr = r32(s, 0x800 + 4 * i);
		s2cr = r32(s, 0xC00 + 4 * i);
		if ((smr >> 31) != 0 && smmu_sid_match(smr, sid, mask)) {
			slot = i;                           /* already there: reuse */
		} else if ((smr >> 31) == 0 && slot == ~0u) {
			slot = i;
		}
		if ((smr >> 31) != 0 && ((s2cr >> 16) & 3) == 0) {
			used |= 1ull << (s2cr & 0x3F);
		}
	}
	for (uint32_t i = 4; i < ncb && i < 64; i++) {  /* keep clear of the loader's cb0..3 */
		if ((used & (1ull << i)) == 0) {
			cb = i;
			break;
		}
	}
	if (slot == ~0u || cb == ~0u || (uint64_t)(npage + cb + 1) * psize > SMMU_SIZE) {
		LOG("smmu: no free SMR/CB (slot %u cb %u)", slot, cb);
		return false;
	}
	volatile uint8_t *gr1 = s + psize, *cbp = s + (uint64_t)npage * psize + (uint64_t)cb * psize;
	w32(cbp, 0x58, 0xFFFFFFFF);                 /* FSR: clear */
	w32(cbp, 0x20, 0);                          /* TTBR0 */
	w32(cbp, 0x24, 0);
	w32(cbp, 0x30, 0);                          /* TCR */
	w32(cbp, 0x10, 0);                          /* TCR2 */
	w32(cbp, 0x38, 0);                          /* MAIR */
	w32(cbp, 0x3C, 0);
	w32(gr1, 0x800 + 4 * cb, CB_CLONE_CBA2R);
	w32(gr1, 0x000 + 4 * cb, CB_CLONE_CBAR);
	w32(cbp, 0x0, CB_CLONE_SCTLR);              /* SCTLR last */
	w32(s, 0xC00 + 4 * slot, cb);               /* S2CR: type 0 (translate), this bank */
	w32(s, 0x800 + 4 * slot, (1u << 31) | (mask << 16) | sid);   /* SMR valid last */
	w32(s, 0x70, 0);                            /* sTLBGSYNC */
	uint32_t n;
	for (n = 0; n < 100000 && (r32(s, 0x74) & 1) != 0; n++) {
		IODelay(1);
	}
	smr = r32(s, 0x800 + 4 * slot);
	s2cr = r32(s, 0xC00 + 4 * slot);
	bool ok = (smr >> 31) != 0 && smmu_sid_match(smr, sid, mask) && ((s2cr >> 16) & 3) == 0 && (s2cr & 0x3F) == cb;
	LOG("smmu: %s identity (sid %x mask %x) CB%u SMR%u: SMR %08x S2CR %08x SCTLR %08x -> %s", name, sid, mask, cb,
	    slot, smr, s2cr, r32(cbp, 0x0), ok ? "OK" : "NOT as written");
	return ok;
}

/* 0272: for bsd/dev/sweet_cam.c (the camera's TFE stream 0x400 and its frame pool) */
extern "C" int
topaz_smmu_identity(uint32_t sid, uint32_t mask, const char *name)
{
	return smmu_identity(sid, mask, name) ? 1 : 0;
}

extern "C" void *
topaz_map_wc(uint64_t pa, uint64_t size)
{
	IOMemoryMap *m = map_phys(pa, size, true);     /* kept for good */
	return m != NULL ? (void *)m->getVirtualAddress() : NULL;
}

/*
 * A stream with real stage-1 translation (the ADSP's, sapphire audio): the ADSP issues 32-bit
 * addresses -- Android's IOVA pool for it is 0x10000000-0x20000000 -- while this XNU's memory is
 * all above 4 GB, so a bypass (smmu_identity()) cannot reach it. `n` 4 KB pages from `pa` appear
 * at `iova` (inside one 2 MB block), through three tables in `tbl` (12 KB, write-combined: the
 * walks are non-cacheable). LPAE stage 1, 4 KB granule, T0SZ 32 (the walk starts at level 1).
 */
static volatile uint8_t *s1_cbp, *s1_smmu;       /* the bank smmu_s1() set up, and the SMMU: fault registers for the log */

static bool __attribute__((unused))
smmu_s1(uint32_t sid, uint32_t mask, const char *name, uint64_t iova, uint64_t pa, unsigned n, uint64_t *tbl,
    uint64_t tbl_pa)
{
	volatile uint8_t *s = (volatile uint8_t *)ml_io_map(SMMU_PA, SMMU_SIZE);
	if (s == NULL || ((iova >> 21) != ((iova + n * 0x1000ull - 1) >> 21))) {
		LOG("smmu: %s: map failed or the range crosses a 2 MB block", name);
		return false;
	}
	uint32_t id0 = r32(s, 0x20), id1 = r32(s, 0x24);
	uint32_t nsmr = id0 & 0xFF, ncb = id1 & 0xFF;
	uint32_t psize = (id1 & (1u << 31)) ? 0x10000 : 0x1000, npage = 1u << (((id1 >> 28) & 7) + 1);
	uint32_t slot = ~0u, cb = ~0u, smr, s2cr;
	uint64_t used = 0;

	for (uint32_t i = 0; i < nsmr && i < 128; i++) {
		smr = r32(s, 0x800 + 4 * i);
		s2cr = r32(s, 0xC00 + 4 * i);
		if ((smr >> 31) != 0 && smmu_sid_match(smr, sid, mask)) {
			slot = i;
		} else if ((smr >> 31) == 0 && slot == ~0u) {
			slot = i;
		}
		if ((smr >> 31) != 0 && ((s2cr >> 16) & 3) == 0) {
			used |= 1ull << (s2cr & 0x3F);
		}
	}
	/*
	 * 0181: the ADSP's first read through this bank reset the phone. Android maps the audio
	 * streams with mask 0xf (msm-audio-ion: qcom,smmu-sid-mask 0xf, 0x1c0-0x1cf): the ADSP may read
	 * through a neighbour of 0x1c1. Every valid SMR that matches any of them is listed (and later
	 * pointed at this bank too); ours then takes the whole mask.
	 */
	uint32_t near[8], nnear = 0;
	for (uint32_t i = 0; i < nsmr && i < 128; i++) {
		smr = r32(s, 0x800 + 4 * i);
		s2cr = r32(s, 0xC00 + 4 * i);
		bool hit = false;
		for (uint32_t k = 0; k < 16 && !hit; k++) {
			hit = (smr >> 31) != 0 && smmu_sid_match(smr, (sid & ~0xFu) | k, 0);
		}
		if (hit) {
			LOG("smmu: %s: SMR%u %08x (id %x mask %x) S2CR %08x matches the audio streams", name, i, smr, smr & 0x7FFF,
			    (smr >> 16) & 0x7FFF, s2cr);
			if (nnear < 8 && i != slot) {
				near[nnear++] = i;
			}
		}
	}
	LOG("smmu: sCR0 %08x sGFSR %08x sGFSYNR0 %08x", r32(s, 0x0), r32(s, 0x48), r32(s, 0x50));
	/* a stream already ours (an earlier identity) keeps its bank */
	if (slot != ~0u && (r32(s, 0x800 + 4 * slot) >> 31) != 0 && ((r32(s, 0xC00 + 4 * slot) >> 16) & 3) == 0) {
		cb = r32(s, 0xC00 + 4 * slot) & 0x3F;
	}
	for (uint32_t i = 4; cb == ~0u && i < ncb && i < 64; i++) {
		if ((used & (1ull << i)) == 0) {
			cb = i;
		}
	}
	if (slot == ~0u || cb == ~0u || (uint64_t)(npage + cb + 1) * psize > SMMU_SIZE) {
		LOG("smmu: %s: no free SMR/CB (slot %u cb %u)", name, slot, cb);
		return false;
	}
	/* tables: level 1 at tbl[0..], level 2 at tbl[512..], level 3 at tbl[1024..] */
	bzero(tbl, 3 * 0x1000);
	tbl[(iova >> 30) & 3] = (tbl_pa + 0x1000) | 3;
	tbl[512 + ((iova >> 21) & 511)] = (tbl_pa + 0x2000) | 3;
	for (unsigned i = 0; i < n; i++) {
		/* page, AF, AP read-write, attribute 0 (normal non-cacheable), non-shareable */
		tbl[1024 + (((iova >> 12) + i) & 511)] = (pa + i * 0x1000ull) | (1u << 10) | (1u << 6) | 3;
	}
	mb();
	volatile uint8_t *gr1 = s + psize, *cbp = s + (uint64_t)npage * psize + (uint64_t)cb * psize;
	w32(cbp, 0x0, 0);                           /* SCTLR off while it changes */
	w32(cbp, 0x58, 0xFFFFFFFF);                 /* FSR */
	w32(cbp, 0x20, (uint32_t)tbl_pa);           /* TTBR0, ASID 0 */
	w32(cbp, 0x24, (uint32_t)(tbl_pa >> 32));
	w32(cbp, 0x30, 32u | (1u << 23));           /* TCR: T0SZ 32, walks non-cacheable non-shareable, 4 KB, EPD1 */
	w32(cbp, 0x10, (7u << 15) | 2u);            /* TCR2: SEP upstream, 40-bit output */
	w32(cbp, 0x38, 0x44);                       /* MAIR0: attribute 0 normal non-cacheable */
	w32(cbp, 0x3C, 0);
	w32(gr1, 0x800 + 4 * cb, CB_CLONE_CBA2R);
	w32(gr1, 0x000 + 4 * cb, CB_CLONE_CBAR);
	w32(cbp, 0x610, 0);                         /* TLBIASID 0 */
	w32(cbp, 0x7F0, 0);                         /* TLBSYNC */
	for (uint32_t k = 0; k < 100000 && (r32(cbp, 0x7F4) & 1) != 0; k++) {
		IODelay(1);
	}
	/* SCTLR: translate; faults reported and terminated (no CFCFG stall: a stalled ADSP read is a hung bus) */
	w32(cbp, 0x0, (1u << 6) | (1u << 5) | 1u);
	s1_cbp = cbp;
	s1_smmu = s;
	for (uint32_t i = 0; i < nnear; i++) {
		w32(s, 0xC00 + 4 * near[i], cb);
		LOG("smmu: %s: SMR%u now through CB%u too", name, near[i], cb);
	}
	w32(s, 0xC00 + 4 * slot, cb);
	w32(s, 0x800 + 4 * slot, (1u << 31) | ((nnear == 0 ? 0xFu : mask) << 16) | (nnear == 0 ? (sid & ~0xFu) : sid));
	w32(s, 0x70, 0);
	for (uint32_t k = 0; k < 100000 && (r32(s, 0x74) & 1) != 0; k++) {
		IODelay(1);
	}
	LOG("smmu: %s: sid %x CB%u SMR%u, %u pages %llx at iova %llx, tables %llx: SCTLR %08x TCR %08x FSR %08x", name, sid,
	    cb, slot, n, pa, iova, tbl_pa, r32(cbp, 0x0), r32(cbp, 0x30), r32(cbp, 0x58));
	return true;
}

/* ---------------- copy engines (ath10k ce.c, WCN3990: 64-bit descriptors, shadow regs, RRI in DDR) */

#define CE_PA               (0x0C800000ull + 0x240000)
#define CE_COUNT            12
#define SHADOW_PA           (0x0C800000ull + 0x32000)
#define SR_BASE_LO          0x00
#define SR_BASE_HI          0x04
#define SR_SIZE             0x08
#define DR_BASE_LO          0x0C
#define DR_BASE_HI          0x10
#define DR_SIZE             0x14
#define CTRL1               0x18
#define SR_WR_IDX           0x3C
#define DR_WR_IDX           0x40
#define CUR_SRRI            0x44
#define CUR_DRRI            0x48
#define SRC_WM              0x4C
#define DST_WM              0x50
#define WRAP_RRI_LO         0xC004
#define WRAP_RRI_HI         0xC008
#define CTRL1_DMAX          0x0000FFFFu
#define CTRL1_SRC_SWAP      (1u << 17)
#define CTRL1_DST_SWAP      (1u << 18)
#define CTRL1_RRI_UPD       (1u << 19)
#define ADDR_HI_MASK        0x1Fu
#define ATTR_DIS_INTR       0x08
#define RX_BUF_SIZE         2048
#define RX_POST             32
#define TX_SLOTS_MAX        64

struct ce_desc { uint64_t addr; uint16_t nbytes, flags; uint32_t toeplitz; } __attribute__((packed));
struct ce_attr { uint16_t src_n, src_max, dst_n, flags; };

/* host_ce_config_wlan (ath10k snoc.c) */
static const struct ce_attr ce_attr[CE_COUNT] = {
	{ 16, 2048, 0, 0 }, { 0, 2048, 512, 0 }, { 0, 2048, 64, 0 }, { 32, 2048, 0, 0 },
	{ 2048, 256, 0, ATTR_DIS_INTR }, { 0, 512, 512, 0 }, { 0, 0, 0, 0 }, { 2, 2048, 2, 0 },
	{ 0, 2048, 128, 0 }, { 0, 2048, 512, 0 }, { 0, 2048, 512, 0 }, { 0, 2048, 512, 0 },
};

struct ce_state {
	volatile struct ce_desc *src, *dst;
	uint64_t src_pa, dst_pa, rx_pa, tx_pa;
	uint32_t src_n, dst_n, src_wr, dst_wr, dst_sw;
	uint8_t *rx_buf, *tx_buf;
	uint32_t rx, tx, tx_full;
};
static struct ce_state ce[CE_COUNT];
static volatile uint8_t *ce_regs, *ce_shadow;
static volatile uint32_t *ce_rri;
static uint64_t ce_rri_pa;
static bool ce_up;

static bool ce_host_rx(uint32_t c) { return ce_attr[c].dst_n != 0 && c != 7; }
static bool ce_host_tx(uint32_t c) { return c == 0 || c == 3 || c == 4; }
static uint32_t ce_tx_slots(uint32_t c) { return MIN(ce[c].src_n, (uint32_t)TX_SLOTS_MAX); }
static uint32_t ce_tx_sz(uint32_t c) { return ce_attr[c].src_max; }
static uint32_t pow2(uint32_t n) { uint32_t p = 1; while (p < n) { p <<= 1; } return p; }
static uint32_t ce_rd(uint32_t c, uint32_t o) { return r32(ce_regs, c * 0x1000 + o); }
static void ce_wr(uint32_t c, uint32_t o, uint32_t v) { w32(ce_regs, c * 0x1000 + o, v); }

static void
ce_post(uint32_t c)
{
	struct ce_state *s = &ce[c];
	uint32_t slot = s->dst_wr;
	s->dst[slot].addr = s->rx_pa + (uint64_t)(slot % RX_POST) * RX_BUF_SIZE;
	s->dst[slot].nbytes = 0;
	s->dst_wr = (slot + 1) & (s->dst_n - 1);
}

static bool
ce_start(void)
{
	uint64_t off = 0x1000, pa = 0;              /* RRI in the first page */

	if (ce_up) {
		return true;
	}
	ce_regs = (volatile uint8_t *)ml_io_map(CE_PA, (CE_COUNT + 1) * 0x1000);
	ce_shadow = (volatile uint8_t *)ml_io_map(SHADOW_PA, 0x1000);
	for (uint32_t c = 0; c < CE_COUNT; c++) {
		ce[c].src_n = ce_attr[c].src_n ? pow2(ce_attr[c].src_n) : 0;
		ce[c].dst_n = ce_attr[c].dst_n ? pow2(ce_attr[c].dst_n) : 0;
		off += ((uint64_t)ce[c].src_n * sizeof(struct ce_desc) + 0xfff) & ~0xfffull;
		off += ((uint64_t)ce[c].dst_n * sizeof(struct ce_desc) + 0xfff) & ~0xfffull;
		off += ce_host_rx(c) ? RX_POST * RX_BUF_SIZE : 0;
		off += ce_host_tx(c) ? ce_tx_slots(c) * ce_tx_sz(c) : 0;
	}
	uint8_t *va = (uint8_t *)pool_alloc(off, &pa);
	if (ce_regs == NULL || ce_shadow == NULL || va == NULL) {
		LOG("ce: map/alloc failed (%llu bytes)", off);
		return false;
	}
	LOG("ce: host memory %llx + %llx", pa, off);
	ce_rri = (volatile uint32_t *)va;
	ce_rri_pa = pa;
	off = 0x1000;
	for (uint32_t c = 0; c < CE_COUNT; c++) {
		struct ce_state *s = &ce[c];
		if (s->src_n != 0) {
			s->src = (volatile struct ce_desc *)(va + off);
			s->src_pa = pa + off;
			off += ((uint64_t)s->src_n * sizeof(struct ce_desc) + 0xfff) & ~0xfffull;
		}
		if (s->dst_n != 0) {
			s->dst = (volatile struct ce_desc *)(va + off);
			s->dst_pa = pa + off;
			off += ((uint64_t)s->dst_n * sizeof(struct ce_desc) + 0xfff) & ~0xfffull;
		}
		if (ce_host_rx(c)) {
			s->rx_buf = va + off;
			s->rx_pa = pa + off;
			off += RX_POST * RX_BUF_SIZE;
		}
		if (ce_host_tx(c)) {
			s->tx_buf = va + off;
			s->tx_pa = pa + off;
			off += ce_tx_slots(c) * ce_tx_sz(c);
		}
	}
	/* the CE writes its read indices to ce_rri[ce] (src [15:0], dst [31:16]) */
	w32(ce_regs, WRAP_RRI_LO, (uint32_t)ce_rri_pa);
	w32(ce_regs, WRAP_RRI_HI, (uint32_t)(ce_rri_pa >> 32) & ADDR_HI_MASK);
	for (uint32_t c = 0; c < CE_COUNT; c++) {
		ce_wr(c, CTRL1, ce_rd(c, CTRL1) | CTRL1_RRI_UPD);
	}
	for (uint32_t c = 0; c < CE_COUNT; c++) {
		struct ce_state *s = &ce[c];
		if (s->src_n != 0) {                    /* ath10k_ce_init_src_ring */
			s->src_wr = ce_rd(c, SR_WR_IDX) & (s->src_n - 1);
			ce_wr(c, SR_BASE_LO, (uint32_t)s->src_pa);
			ce_wr(c, SR_BASE_HI, (uint32_t)(s->src_pa >> 32) & ADDR_HI_MASK);
			ce_wr(c, SR_SIZE, s->src_n);
			ce_wr(c, CTRL1, (ce_rd(c, CTRL1) & ~(CTRL1_DMAX | CTRL1_SRC_SWAP)) | ce_attr[c].src_max);
			ce_wr(c, SRC_WM, s->src_n);
		}
		if (s->dst_n != 0) {                    /* ath10k_ce_init_dest_ring */
			s->dst_sw = ce_rd(c, CUR_DRRI) & (s->dst_n - 1);
			s->dst_wr = ce_rd(c, DR_WR_IDX) & (s->dst_n - 1);
			ce_wr(c, DR_BASE_LO, (uint32_t)s->dst_pa);
			ce_wr(c, DR_BASE_HI, (ce_rd(c, DR_BASE_HI) & ~ADDR_HI_MASK) | ((uint32_t)(s->dst_pa >> 32) & ADDR_HI_MASK));
			ce_wr(c, DR_SIZE, s->dst_n);
			ce_wr(c, CTRL1, ce_rd(c, CTRL1) & ~CTRL1_DST_SWAP);
			ce_wr(c, DST_WM, s->dst_n);
		}
	}
	for (uint32_t c = 0; c < CE_COUNT; c++) {   /* ath10k_snoc_rx_post */
		if (!ce_host_rx(c)) {
			continue;
		}
		for (uint32_t i = 0; i < RX_POST; i++) {
			ce_post(c);
		}
		mb();
		ce_wr(c, DR_WR_IDX, ce[c].dst_wr);
	}
	ce_up = true;
	LOG("t=%u ms: ce: %u rings programmed, rx posted", g_ms, CE_COUNT);
	return true;
}

static void htc_rx(uint32_t c, const uint8_t *d, uint32_t len);

static bool
ce_poll(void)
{
	bool any = false;
	if (!ce_up) {
		return false;
	}
	for (uint32_t c = 0; c < CE_COUNT; c++) {
		struct ce_state *s = &ce[c];
		if (!ce_host_rx(c)) {
			continue;
		}
		for (int k = 0; k < 16; k++) {
			uint32_t slot = s->dst_sw, n = s->dst[slot].nbytes;
			if (n == 0) {
				break;
			}
			mb();                               /* payload after the descriptor */
			s->dst[slot].nbytes = 0;
			s->dst_sw = (slot + 1) & (s->dst_n - 1);
			s->rx++;
			htc_rx(c, s->rx_buf + (slot % RX_POST) * RX_BUF_SIZE, n);
			ce_post(c);
			mb();
			ce_wr(c, DR_WR_IDX, s->dst_wr);
			any = true;
		}
	}
	return any;
}

static bool
ce_send(uint32_t c, const void *data, uint32_t len, uint32_t transfer_id)
{
	if (!ce_up || c >= CE_COUNT || !ce_host_tx(c) || len > ce_tx_sz(c)) {
		return false;
	}
	struct ce_state *s = &ce[c];
	uint32_t mask = s->src_n - 1, sw = ce_rri[c] & 0xFFFF;
	if (((sw - 1 - s->src_wr) & mask) == 0 || ((s->src_wr - sw) & mask) >= ce_tx_slots(c) - 1) {
		s->tx_full++;
		return false;
	}
	uint32_t slot = s->src_wr;
	memcpy(s->tx_buf + (slot % ce_tx_slots(c)) * ce_tx_sz(c), data, len);
	volatile struct ce_desc *d = &s->src[slot];
	d->addr = s->tx_pa + (uint64_t)(slot % ce_tx_slots(c)) * ce_tx_sz(c);
	d->nbytes = (uint16_t)len;
	d->flags = (uint16_t)((transfer_id << 4) & 0xFFF0);
	d->toeplitz = 0;
	s->src_wr = (slot + 1) & mask;
	s->tx++;
	mb();
	*(volatile uint32_t *)(ce_shadow + 4 * c) = s->src_wr;
	mb();
	return true;
}

/* ---------------- HTC (ath10k htc.c) */

#define HTC_MSG_READY       1
#define HTC_MSG_CONNECT     2
#define HTC_MSG_CONNECT_RESP 3
#define HTC_MSG_SETUP_COMPLETE_EX 5
#define HTC_FLAG_NEED_CREDIT_UPDATE 0x01
#define HTC_FLAG_TRAILER    0x02
#define HTC_CONN_NO_CREDIT_FLOW 0x0008
#define SVC_RSVD_CTRL       0x0001
#define SVC_WMI_CONTROL     0x0100
#define SVC_HTT_DATA        0x0300
#define EP_MAX              8

struct htc_hdr { uint8_t eid, flags; uint16_t len; uint8_t trailer, seq; uint16_t pad; } __attribute__((packed));
struct htc_ep { uint16_t svc, max_msg; uint8_t ul, dl, seq, credits; bool up, credit_flow; uint32_t rx; };
static struct htc_ep ep[EP_MAX];
static uint8_t wmi_eid = 0xFF, htt_eid = 0xFF;
static bool htc_ready, htc_setup;

static bool
htc_send(uint8_t eid, const void *payload, uint32_t len)
{
	static uint8_t buf[2048];
	struct htc_hdr *h = (struct htc_hdr *)buf;
	if (eid >= EP_MAX || len + sizeof(*h) > sizeof(buf)) {
		return false;
	}
	struct htc_ep *e = &ep[eid];
	if (e->credit_flow) {
		if (e->credits == 0) {
			return false;
		}
		e->credits--;
	}
	bzero(h, sizeof(*h));
	h->eid = eid;
	h->len = (uint16_t)len;
	h->flags = e->credit_flow ? HTC_FLAG_NEED_CREDIT_UPDATE : 0;
	h->seq = e->seq++;
	memcpy(buf + sizeof(*h), payload, len);
	if (!ce_send(e->ul, buf, (uint32_t)sizeof(*h) + len, eid)) {
		LOG("htc: send on ep %u (ce%u) FAILED", eid, e->ul);
		return false;
	}
	return true;
}

static void
htc_connect(uint16_t svc)
{
	uint8_t m[8] = {};
	uint16_t id = HTC_MSG_CONNECT, flags = svc == SVC_WMI_CONTROL ? (uint16_t)(1u << 8) : HTC_CONN_NO_CREDIT_FLOW;
	memcpy(m, &id, 2);
	memcpy(m + 2, &svc, 2);
	memcpy(m + 4, &flags, 2);
	htc_send(0, m, sizeof(m));
}

/* WMI waits here while the endpoint has no credit */
#define WMIQ_LEN 6
#define WMIQ_MAX 2040
static uint8_t wmiq[WMIQ_LEN][WMIQ_MAX];
static uint16_t wmiq_len[WMIQ_LEN];
static uint32_t wmiq_head, wmiq_tail;

static void
wmiq_pump(void)
{
	while (wmiq_head != wmiq_tail && wmi_eid != 0xFF && ep[wmi_eid].credits != 0) {
		uint32_t i = wmiq_head % WMIQ_LEN;
		if (!htc_send(wmi_eid, wmiq[i], wmiq_len[i])) {
			break;
		}
		wmiq_head++;
	}
}

static bool
htc_wmi_send(const void *data, uint32_t len)
{
	if (wmi_eid == 0xFF || len > WMIQ_MAX) {
		return false;
	}
	if (wmiq_head == wmiq_tail && ep[wmi_eid].credits != 0) {
		return htc_send(wmi_eid, data, len);
	}
	if (wmiq_tail - wmiq_head >= WMIQ_LEN) {
		LOG("htc: wmi queue full, message dropped");
		return false;
	}
	uint32_t i = wmiq_tail++ % WMIQ_LEN;
	memcpy(wmiq[i], data, len);
	wmiq_len[i] = (uint16_t)len;
	return true;
}

static void wmi_rx(const uint8_t *d, uint32_t len);
static void htt_rx(const uint8_t *p, uint32_t len);

static void
htc_rx(uint32_t c, const uint8_t *d, uint32_t len)
{
	const struct htc_hdr *h = (const struct htc_hdr *)d;
	const uint8_t *p = d + sizeof(*h);
	if (len < sizeof(*h)) {
		return;
	}
	uint32_t plen = MIN((uint32_t)h->len, len - (uint32_t)sizeof(*h));
	if ((h->flags & HTC_FLAG_TRAILER) != 0 && h->trailer <= plen) {
		const uint8_t *t = p + plen - h->trailer;   /* records {id, len, data}; id 1 = credits */
		for (uint32_t o = 0; o + 2 <= h->trailer && o + 2 + t[o + 1] <= h->trailer; o += 2 + t[o + 1]) {
			if (t[o] == 1) {
				for (uint32_t i = 0; i + 1 < t[o + 1]; i += 2) {
					if (t[o + 2 + i] < EP_MAX) {
						ep[t[o + 2 + i]].credits = (uint8_t)(ep[t[o + 2 + i]].credits + t[o + 3 + i]);
					}
				}
			}
		}
		plen -= h->trailer;
		wmiq_pump();
	}
	if (h->eid < EP_MAX) {
		ep[h->eid].rx++;
	}
	if (h->eid == 0) {
		uint16_t msg = 0;
		if (plen >= 2) {
			memcpy(&msg, p, 2);
		}
		if (msg == HTC_MSG_READY && plen >= 8) {
			uint16_t cc, cs;
			memcpy(&cc, p + 2, 2);
			memcpy(&cs, p + 4, 2);
			htc_ready = true;
			ep[0].svc = SVC_RSVD_CTRL;
			ep[0].ul = 0;
			ep[0].dl = 2;
			ep[0].up = true;
			LOG("t=%u ms: *** HTC READY: credits %u x %u, %u endpoints ***", g_ms, cc, cs, p[6]);
			htc_connect(SVC_HTT_DATA);          /* ath10k_core_start: htt, then wmi */
		} else if (msg == HTC_MSG_CONNECT_RESP && plen >= 8) {
			uint16_t svc, max;
			memcpy(&svc, p + 2, 2);
			memcpy(&max, p + 6, 2);
			uint8_t status = p[4], eid = p[5];
			LOG("t=%u ms: htc: connect resp svc %x status %u eid %u max_msg %u", g_ms, svc, status, eid, max);
			if (status != 0 || eid >= EP_MAX) {
				return;
			}
			ep[eid].svc = svc;
			ep[eid].max_msg = max;
			ep[eid].ul = svc == SVC_WMI_CONTROL ? 3 : 4;
			ep[eid].dl = svc == SVC_WMI_CONTROL ? 2 : 1;
			ep[eid].up = true;
			ep[eid].credit_flow = svc == SVC_WMI_CONTROL;
			ep[eid].credits = ep[eid].credit_flow ? 1 : 0;
			if (svc == SVC_HTT_DATA) {
				htt_eid = eid;
				htc_connect(SVC_WMI_CONTROL);
			} else if (svc == SVC_WMI_CONTROL) {
				wmi_eid = eid;
				uint8_t m[12] = {};
				uint16_t id = HTC_MSG_SETUP_COMPLETE_EX;
				memcpy(m, &id, 2);
				htc_setup = htc_send(0, m, sizeof(m));
				LOG("t=%u ms: htc: SETUP_COMPLETE_EX %s", g_ms, htc_setup ? "sent" : "FAILED");
			}
		}
	} else if (h->eid == wmi_eid) {
		wmi_rx(p, plen);
	} else if (h->eid == htt_eid) {
		htt_rx(p, plen);
	}
}

/* ---------------- WMI (TLV, ath10k wmi-tlv.c) and the passive scan */

#define WMI_EV_SERVICE_READY 0x1
#define WMI_EV_READY        0x2
#define WMI_EV_SERVICE_AVAILABLE 0x3
#define WMI_EV_SCAN         0x3001
#define WMI_EV_MGMT_RX      0x7001
#define WMI_CMD_INIT        0x1
#define CMD_START_SCAN      0x3001
#define CMD_SCAN_CHAN_LIST  0x3003
#define CMD_VDEV_CREATE     0x5001
#define TAG_ARRAY_UINT32    0x10
#define TAG_ARRAY_BYTE      0x11
#define TAG_ARRAY_STRUCT    0x12
#define TAG_ARRAY_FIXED_STRUCT 0x13
#define TAG_SERVICE_READY   0x20
#define TAG_HOST_MEM_REQ    0x22
#define TAG_READY_EVENT     0x23
#define TAG_SCAN_EVENT      0x24
#define TAG_MGMT_RX_HDR     0x2C
#define TAG_INIT_CMD        0x4A
#define TAG_RESOURCE_CONFIG 0x4B
#define TAG_HOST_MEM_CHUNK  0x4C
#define TAG_START_SCAN_CMD  0x4D
#define TAG_SCAN_CHAN_LIST_CMD 0x4F
#define TAG_CHANNEL         0x50
#define TAG_VDEV_CREATE_CMD 0x56
#define TAG_SERVICE_AVAILABLE 0x22F
#define ABI_VER0            0x01000000u
#define ABI_VER1            53
#define ABI_NS0             0x5F414351u         /* "QCA_" */
#define ABI_NS1             0x00004C4Du         /* "ML" */
#define SVC_RX_FULL_REORDER 65
#define SVC_EXTEND_ADDRESS  179
#define NUM_PEERS           33
#define NUM_VDEVS           4
#define MAX_MEM_REQS        8
#define MAX_MEM_TOTAL       (4u << 20)

struct wmi_mem_req { uint32_t req_id, unit_size, num_unit_info, num_units; };
static uint32_t wmi_svc_map[32], wmi_svc_ext[4], wmi_nreq;
static struct wmi_mem_req wmi_req[MAX_MEM_REQS];
static uint64_t wmi_chunk_pa[MAX_MEM_REQS];
static uint32_t wmi_chunk_len[MAX_MEM_REQS];
static bool wmi_init_sent, wmi_ready, wmi_seen_bmap, wmi_seen_reqs;
static uint8_t wmi_mac[6];

static bool
wmi_svc(uint32_t bit)
{
	if (bit < 128) {
		return (wmi_svc_map[bit / 4] & (1u << (bit % 4))) != 0;
	}
	bit -= 128;
	return bit < 128 && (wmi_svc_ext[bit / 32] & (1u << (bit % 32))) != 0;
}

static uint8_t *
wmi_put_tlv(uint8_t *p, uint16_t tag, uint32_t len)
{
	uint16_t l = (uint16_t)len;
	memcpy(p, &l, 2);
	memcpy(p + 2, &tag, 2);
	return p + 4;
}

static uint8_t *
put32(uint8_t *p, uint32_t v)
{
	memcpy(p, &v, 4);
	return p + 4;
}

static bool
wmi_send(uint32_t cmd, const void *tlvs, uint32_t len)
{
	static uint8_t m[2048];
	if (len + 4 > sizeof(m)) {
		return false;
	}
	put32(m, cmd & 0xFFFFFF);
	memcpy(m + 4, tlvs, len);
	return htc_wmi_send(m, len + 4);
}

/* walk the TLVs of SERVICE_READY (ath10k_wmi_tlv_svc_rdy_parse) */
static void
wmi_svc_ready(const uint8_t *p, uint32_t len)
{
	for (uint32_t o = 0; o + 4 <= len;) {
		uint16_t l, tag;
		memcpy(&l, p + o, 2);
		memcpy(&tag, p + o + 2, 2);
		if (o + 4 + l > len) {
			break;
		}
		const uint8_t *v = p + o + 4;
		if (tag == TAG_SERVICE_READY && l >= 16) {
			uint32_t fw, a0, rf;
			memcpy(&fw, v, 4);
			memcpy(&a0, v + 4, 4);
			memcpy(&rf, v + 4 + 24 + 8, 4);
			LOG("wmi: fw %08x abi %08x rf_chains %u", fw, a0, rf);
		} else if (tag == TAG_ARRAY_UINT32 && !wmi_seen_bmap) {
			wmi_seen_bmap = true;
			memcpy(wmi_svc_map, v, MIN((uint32_t)l, (uint32_t)sizeof(wmi_svc_map)));
		} else if (tag == TAG_ARRAY_STRUCT && !wmi_seen_reqs) {
			wmi_seen_reqs = true;
			for (uint32_t q = 0; q + 4 <= l;) {
				uint16_t ql, qt;
				memcpy(&ql, v + q, 2);
				memcpy(&qt, v + q + 2, 2);
				if (q + 4 + ql > l) {
					break;
				}
				if (qt == TAG_HOST_MEM_REQ && ql >= sizeof(struct wmi_mem_req) && wmi_nreq < MAX_MEM_REQS) {
					memcpy(&wmi_req[wmi_nreq], v + q + 4, sizeof(struct wmi_mem_req));
					LOG("wmi: mem_req %u: id %u unit %u info %x units %u", wmi_nreq, wmi_req[wmi_nreq].req_id,
					    wmi_req[wmi_nreq].unit_size, wmi_req[wmi_nreq].num_unit_info, wmi_req[wmi_nreq].num_units);
					wmi_nreq++;
				}
				q += 4 + ql;
			}
		}
		o += 4 + l;
	}
}

static void
wmi_send_init(void)
{
	static uint8_t m[1024];
	uint8_t *p = m;
	uint32_t total = 0, n = wmi_nreq;
	uint64_t pa = 0;

	for (uint32_t i = 0; i < n; i++) {
		uint32_t units = wmi_req[i].num_units;
		if (wmi_req[i].num_unit_info & 6) {
			units = NUM_PEERS + 1;
		} else if (wmi_req[i].num_unit_info & 1) {
			units = NUM_VDEVS + 1;
		}
		wmi_chunk_len[i] = units * ((wmi_req[i].unit_size + 3) & ~3u);
		total += (wmi_chunk_len[i] + 0xfff) & ~0xfffu;
	}
	if (total != 0 && (total > MAX_MEM_TOTAL || pool_alloc(total, &pa) == NULL)) {
		LOG("wmi: host memory %u bytes not given", total);
		n = 0;
	}
	for (uint32_t i = 0, off = 0; i < n; i++) {
		wmi_chunk_pa[i] = pa + off;
		off += (wmi_chunk_len[i] + 0xfff) & ~0xfffu;
	}
	bzero(m, sizeof(m));
	p = wmi_put_tlv(p, TAG_INIT_CMD, 24 + 4);
	p = put32(p, ABI_VER0);
	p = put32(p, ABI_VER1);
	p = put32(p, ABI_NS0);
	p = put32(p, ABI_NS1);
	p = put32(p, 0);
	p = put32(p, 0);
	p = put32(p, n);
	/* wmi_tlv_resource_config, 44 x u32, ath10k values for wcn3990 */
	uint32_t rc[44] = {};
	rc[0] = NUM_VDEVS;
	rc[1] = NUM_PEERS;
	if (wmi_svc(SVC_RX_FULL_REORDER)) {
		rc[2] = NUM_VDEVS;
		rc[3] = NUM_VDEVS;
	}
	rc[4] = 2;                                  /* peer keys */
	rc[5] = NUM_PEERS * 2;                      /* tids */
	rc[6] = 16;                                 /* ast skid limit */
	rc[7] = 0x7;                                /* tx chain mask */
	rc[8] = 0x7;                                /* rx chain mask */
	rc[9] = 0x64; rc[10] = 0x64; rc[11] = 0x64; rc[12] = 0x28;
	rc[13] = 1;                                 /* rx decap: native wifi */
	rc[14] = 4;                                 /* scan max pending */
	rc[15] = NUM_VDEVS;                         /* bmiss offload */
	rc[16] = NUM_VDEVS;                         /* roam offload */
	rc[17] = 8;                                 /* roam ap profiles */
	rc[21] = 0x400;                             /* tx dbglog size */
	rc[22] = 2;                                 /* wds entries */
	rc[27] = 2;                                 /* gtk offload */
	rc[28] = 1024 + 32;                         /* msdu descs */
	rc[29] = 2;                                 /* frag entries */
	rc[30] = 1;                                 /* tdls vdevs */
	rc[31] = 0x20;                              /* tdls conn table */
	rc[32] = 2;                                 /* beacon tx offload */
	rc[33] = 5;                                 /* mcast filter entries */
	rc[34] = 22;                                /* wow filters */
	rc[35] = 6;                                 /* keepalive patterns */
	rc[37] = 1;
	rc[38] = 1;
	rc[43] = 1u << 9;                           /* MGMT_BUNDLE_TX_COMPL */
	p = wmi_put_tlv(p, TAG_RESOURCE_CONFIG, sizeof(rc));
	memcpy(p, rc, sizeof(rc));
	p += sizeof(rc);
	p = wmi_put_tlv(p, TAG_ARRAY_STRUCT, n * 20);
	for (uint32_t i = 0; i < n; i++) {
		uint8_t *c = wmi_put_tlv(p, TAG_HOST_MEM_CHUNK, 16);
		c = put32(c, wmi_req[i].req_id);
		c = put32(c, (uint32_t)wmi_chunk_pa[i]);
		c = put32(c, wmi_chunk_len[i]);
		put32(c, wmi_svc(SVC_EXTEND_ADDRESS) ? (uint32_t)(wmi_chunk_pa[i] >> 32) : 0);
		p += 20;
	}
	wmi_init_sent = wmi_send(WMI_CMD_INIT, m, (uint32_t)(p - m));
	LOG("t=%u ms: wmi: INIT (%u chunks, %u bytes host memory) %s", g_ms, n, total, wmi_init_sent ? "sent" : "FAILED");
}

/* scan: 2.4 GHz 1-13, 5 GHz UNII-1/2/2e/3 */
static const uint16_t scan_freqs[] = {
	2412, 2417, 2422, 2427, 2432, 2437, 2442, 2447, 2452, 2457, 2462, 2467, 2472,
	5180, 5200, 5220, 5240, 5260, 5280, 5300, 5320,
	5500, 5520, 5540, 5560, 5580, 5600, 5620, 5640, 5660, 5680, 5700,
	5745, 5765, 5785, 5805, 5825,
};
#define NFREQ ((uint32_t)(sizeof(scan_freqs) / sizeof(scan_freqs[0])))
static const uint8_t our_mac[6] = { 0x02, 0x54, 0x4F, 0x50, 0x41, 0x5A };   /* locally administered */
struct bss {
	uint8_t bssid[6]; char ssid[33]; uint8_t chan; int8_t rssi; uint16_t seen;
	uint8_t raw_ssid[32], raw_ssid_len; uint16_t freq, bintval, capab, ie_len; uint32_t last_ms; uint8_t ie[768];
};
static struct bss bss[64];
static uint32_t nbss, scans, scan_frames;
static volatile uint32_t api_scans_done;
static bool scan_started, scanning;
static unsigned next_scan_ms;

/*
 * 0322: access points not heard for BSS_MAX_AGE_MS leave the table when the next scan starts (the scan
 * fills it again with what is still there). Before, the table only ever grew: networks that had gone
 * stayed listed and, with 64 entries, new ones were dropped. The network joined is always kept.
 */
#define BSS_MAX_AGE_MS 120000u
static bool bss_is_ours(const uint8_t *bssid);  /* (below, next to the join's state) */
static void
bss_age(void)
{
	uint32_t keep = 0, gone = 0;
	for (uint32_t i = 0; i < nbss; i++) {
		if (g_ms - bss[i].last_ms <= BSS_MAX_AGE_MS || bss_is_ours(bss[i].bssid)) {
			if (keep != i) {
				bss[keep] = bss[i];
			}
			keep++;
		} else {
			gone++;
		}
	}
	nbss = keep;
	if (gone != 0) {
		LOG("t=%u ms: scan: %u networks not heard for %u s forgotten, %u kept", g_ms, gone, BSS_MAX_AGE_MS / 1000, keep);
	}
}

static bool
scan_send_start(void)
{
	bss_age();
	static uint8_t m[512];
	uint8_t *p = m, *c;
	bzero(m, sizeof(m));
	c = wmi_put_tlv(p, TAG_START_SCAN_CMD, 100);
	c = put32(c, 0xA000 | (scans + 1));         /* scan_id */
	c = put32(c, 0xA000 | 1);                   /* scan_req_id */
	c = put32(c, 0);                            /* vdev */
	c = put32(c, 1);                            /* priority low */
	c = put32(c, 0x001 | 0x002 | 0x004 | 0x008 | 0x100 | 0x010);   /* events */
	c = put32(c, 50);                           /* dwell active */
	c = put32(c, 150);                          /* dwell passive */
	c = put32(c, 50);                           /* min rest */
	c = put32(c, 500);                          /* max rest */
	c = put32(c, 0);
	c = put32(c, 0);
	c = put32(c, 0);
	c = put32(c, 20000);                        /* max scan time */
	c = put32(c, 5);                            /* probe delay */
	c = put32(c, (0x01 | 0x10) ^ 0x20);         /* passive, chan stat; probe-req filter inverted on TLV */
	c = put32(c, 0);
	c = put32(c, NFREQ);
	c = put32(c, 1);                            /* bssids */
	c = put32(c, 0);                            /* ssids */
	c = put32(c, 0);                            /* ie_len */
	c = put32(c, 3);                            /* num_probes */
	p = c + 16;                                 /* mac addr + mask: zero */
	c = wmi_put_tlv(p, TAG_ARRAY_UINT32, NFREQ * 4);
	for (uint32_t i = 0; i < NFREQ; i++) {
		c = put32(c, scan_freqs[i]);
	}
	p = wmi_put_tlv(c, TAG_ARRAY_FIXED_STRUCT, 0);
	c = wmi_put_tlv(p, TAG_ARRAY_FIXED_STRUCT, 8);  /* bssid: broadcast */
	memset(c, 0xFF, 6);
	p = wmi_put_tlv(c + 8, TAG_ARRAY_BYTE, 0);
	return wmi_send(CMD_START_SCAN, m, (uint32_t)(p - m));
}

static void
scan_start(void)
{
	static uint8_t m[1536];
	uint8_t *p = m, *c;

	if (scan_started) {
		return;
	}
	scan_started = true;
	/* channel list, every channel passive: nothing is transmitted */
	p = put32(wmi_put_tlv(p, TAG_SCAN_CHAN_LIST_CMD, 4), NFREQ);
	p = wmi_put_tlv(p, TAG_ARRAY_STRUCT, NFREQ * (4 + 24));
	for (uint32_t i = 0; i < NFREQ; i++) {
		uint32_t mode = scan_freqs[i] < 4000 ? 1 : 0;
		uint32_t flags = mode | (1u << 7) | (1u << 11) | (scan_freqs[i] < 4000 ? 0 : (1u << 12));
		c = wmi_put_tlv(p, TAG_CHANNEL, 24);
		c = put32(c, scan_freqs[i]);
		c = put32(c, scan_freqs[i]);
		c = put32(c, 0);
		c = put32(c, flags);
		c = put32(c, (40u << 8) | (40u << 16));
		p = put32(c, 40u << 8);
	}
	bool a = wmi_send(CMD_SCAN_CHAN_LIST, m, (uint32_t)(p - m));
	uint8_t v[4 + 20] = {};
	c = wmi_put_tlv(v, TAG_VDEV_CREATE_CMD, 20);
	c = put32(c, 0);                            /* vdev 0 */
	c = put32(c, 2);                            /* station */
	c = put32(c, 0);
	memcpy(c, our_mac, 6);
	bool b = wmi_send(CMD_VDEV_CREATE, v, sizeof(v));
	scanning = scan_send_start();
	LOG("t=%u ms: scan: chan list (%u ch) %s, vdev %s, passive scan %s", g_ms, NFREQ, a ? "ok" : "FAILED",
	    b ? "ok" : "FAILED", scanning ? "requested" : "FAILED");
}

/* ---------------- HTT setup (ath10k htt.c ath10k_htt_setup, 64-bit, WCN3990), after WMI READY */

/*
 * Without the host rx ring the radio receives (pdev rx_frame grows) but no frame reaches the
 * firmware's rx path: no beacon, no scan result (topaz-woa v0.12). So, before the scan:
 * VERSION_REQ -> VERSION_CONF (or 2 s), FRAG_DESC_BANK_CFG, RX_RING_CFG (host ring of buffer
 * addresses + host-written alloc index), AGGR_CFG v2. All on the HTT endpoint (CE4).
 */
#define H2T_VERSION_REQ     0
#define H2T_RX_RING_CFG     2
#define H2T_AGGR_CFG        5
#define H2T_FRAG_DESC_BANK_CFG 6
#define T2H_VERSION_CONF    0x00
#define T2H_RX_IN_ORD_PADDR_IND 0x12
#define HTT_RING_SIZE       256
#define HTT_BUF_SIZE        2048
#define HTT_MAX_PENDING_TX  (1024 + 32)
#define HTT_EXT_DESC_SIZE   72
/* wcn3990 htt_rx_desc_v2 offsets in 4-byte words */
#define OFF_ATTENTION       1
#define OFF_FRAG_INFO       2
#define OFF_MPDU_START      4
#define OFF_MSDU_START      7
#define OFF_MSDU_END        12
#define OFF_MPDU_END        26
#define OFF_PPDU_START      27
#define OFF_PPDU_END        37
#define OFF_HDR_STATUS      74
#define OFF_MSDU_PAYLOAD    90

static uint8_t *htt_blk, *htt_bank, *htt_buf;
static uint64_t htt_ring_pa, htt_idx_pa, htt_bank_pa, htt_buf_pa;
static volatile uint64_t *htt_ring;
static volatile uint32_t *htt_idx;
static uint32_t htt_alloc, htt_state, htt_msgs, htt_msdus, htt_req_ms;

static bool htt_send(const void *d, uint32_t len) { return htt_eid != 0xFF && htc_send(htt_eid, d, len); }

static void
htt_post(uint64_t pa)
{
	*(volatile uint32_t *)(htt_buf + (pa - htt_buf_pa) + 4 * OFF_ATTENTION) = 0;
	htt_ring[htt_alloc] = pa;
	htt_alloc = (htt_alloc + 1) & (HTT_RING_SIZE - 1);
}

static void
htt_publish(void)
{
	mb();                                       /* ring entries before the index */
	*htt_idx = htt_alloc;
	mb();
}

static uint8_t *put16(uint8_t *p, uint16_t v) { memcpy(p, &v, 2); return p + 2; }
static uint8_t *put64(uint8_t *p, uint64_t v) { memcpy(p, &v, 8); return p + 8; }

static void
htt_configure(const char *why)
{
	uint8_t m[64] = {}, *p = m;

	htt_state = 2;
	/* ath10k_htt_send_frag_desc_bank_cfg_64 */
	*p++ = H2T_FRAG_DESC_BANK_CFG;
	*p++ = 0;
	*p++ = 1;                                   /* num_banks */
	*p++ = HTT_EXT_DESC_SIZE;
	p = put64(p, htt_bank_pa);
	p += 3 * 8;
	p = put16(p, 0);
	p = put16(p, HTT_MAX_PENDING_TX - 1);
	p += 3 * 4;
	p += 4 + 2 + 2;
	*p++ = 1;
	*p++ = 0;
	bool a = htt_send(m, sizeof(m));
	/* ath10k_htt_send_rx_ring_cfg_64 */
	for (uint32_t i = 0; i < HTT_RING_SIZE - 1; i++) {
		htt_post(htt_buf_pa + (uint64_t)i * HTT_BUF_SIZE);
	}
	uint8_t r[48] = {};
	p = r;
	*p++ = H2T_RX_RING_CFG;
	*p++ = 1;
	p = put16(p, 0);
	p = put64(p, htt_idx_pa);
	p = put64(p, htt_ring_pa);
	p = put16(p, HTT_RING_SIZE);
	p = put16(p, HTT_BUF_SIZE);
	p = put16(p, 0xFFFF);                       /* every frame type, management included */
	p = put16(p, 0);                            /* fw_idx_init_val */
	p = put16(p, OFF_HDR_STATUS);
	p = put16(p, OFF_MSDU_PAYLOAD);
	p = put16(p, OFF_PPDU_START);
	p = put16(p, OFF_PPDU_END);
	p = put16(p, OFF_MPDU_START);
	p = put16(p, OFF_MPDU_END);
	p = put16(p, OFF_MSDU_START);
	p = put16(p, OFF_MSDU_END);
	p = put16(p, OFF_ATTENTION);
	p = put16(p, OFF_FRAG_INFO);
	htt_publish();
	bool b = htt_send(r, sizeof(r));
	uint8_t g[4] = { H2T_AGGR_CFG, 64, 3, 0 };
	bool c = htt_send(g, sizeof(g));
	LOG("t=%u ms: htt (%s): frag bank %s, rx ring cfg %s (%u bufs), aggr cfg %s", g_ms, why, a ? "ok" : "FAILED",
	    b ? "ok" : "FAILED", HTT_RING_SIZE - 1, c ? "ok" : "FAILED");
	scan_start();
}

static void
htt_start(void)
{
	uint64_t pa = 0;
	uint32_t ring = (HTT_RING_SIZE * 8 + 64 + 0xfff) & ~0xfffu;
	uint32_t bank = (HTT_MAX_PENDING_TX * HTT_EXT_DESC_SIZE + 0xfff) & ~0xfffu;

	if (htt_state != 0) {
		return;
	}
	htt_blk = (uint8_t *)pool_alloc(ring + bank + HTT_RING_SIZE * HTT_BUF_SIZE, &pa);
	if (htt_blk == NULL) {
		LOG("htt: no memory, scanning without it");
		scan_start();
		return;
	}
	htt_ring = (volatile uint64_t *)htt_blk;
	htt_ring_pa = pa;
	htt_idx = (volatile uint32_t *)(htt_blk + HTT_RING_SIZE * 8);
	htt_idx_pa = pa + HTT_RING_SIZE * 8;
	htt_bank = htt_blk + ring;
	htt_bank_pa = pa + ring;
	htt_buf = htt_blk + ring + bank;
	htt_buf_pa = pa + ring + bank;
	htt_state = 1;
	htt_req_ms = g_ms;
	uint8_t m[4] = { H2T_VERSION_REQ, 0, 0, 0 };
	LOG("t=%u ms: htt: VERSION_REQ %s", g_ms, htt_send(m, sizeof(m)) ? "sent" : "FAILED");
}

static void
htt_rx(const uint8_t *p, uint32_t len)
{
	uint8_t type = len != 0 ? p[0] : 0xFF;
	htt_msgs++;
	if (type == T2H_VERSION_CONF && len >= 3) {
		LOG("t=%u ms: *** HTT VERSION_CONF %u.%u ***", g_ms, p[2], p[1]);
		if (htt_state == 1) {
			htt_configure("version conf");
		}
		return;
	}
	if (type == 0x07 && len >= 4) {            /* TX_COMPL_IND */
		htt_tx_compl(p, len);
		return;
	}
	if (type == T2H_RX_IN_ORD_PADDR_IND && len >= 8) {
		uint16_t n;
		memcpy(&n, p + 6, 2);
		const uint8_t *d = p + 8;
		for (uint32_t i = 0; i < n && 8 + (i + 1) * 12 <= len; i++, d += 12) {
			uint64_t pa;
			memcpy(&pa, d, 8);
			if (pa < htt_buf_pa || pa >= htt_buf_pa + (uint64_t)HTT_RING_SIZE * HTT_BUF_SIZE ||
			    ((pa - htt_buf_pa) % HTT_BUF_SIZE) != 0) {
				continue;
			}
			htt_msdus++;
			uint16_t mlen;
			memcpy(&mlen, d + 8, 2);
			if (mlen <= HTT_BUF_SIZE - 4 * OFF_MSDU_PAYLOAD) {
				w_rx_native(htt_buf + (pa - htt_buf_pa) + 4 * OFF_MSDU_PAYLOAD, mlen);
			}
			htt_post(pa);                       /* recycle the same buffer */
		}
		htt_publish();
	}
}

static void
htt_poll(void)
{
	if (htt_state == 1 && g_ms - htt_req_ms > 2000) {
		LOG("t=%u ms: htt: no VERSION_CONF in 2 s, configuring anyway", g_ms);
		htt_configure("timeout");
	}
}

static void
scan_event(const uint8_t *t, uint32_t len)
{
	uint16_t tag;
	uint32_t e[6];
	if (len < 4 + 24) {
		return;
	}
	memcpy(&tag, t + 2, 2);
	if (tag != TAG_SCAN_EVENT) {
		return;
	}
	memcpy(e, t + 4, sizeof(e));
	if (e[0] == 0x008 || e[0] == 0x100 || e[0] == 0x004) {
		return;                                 /* foreign channel / exit / bss channel */
	}
	if (e[0] == 0x001) {
		LOG("t=%u ms: scan: STARTED", g_ms);
	}
	if (e[0] == 0x002 || e[0] == 0x040 || e[0] == 0x010) {
		scanning = false;
		scans++;
		LOG("t=%u ms: *** scan %u done (event %x): %u networks, %u beacons/probe responses, %u htt msdus ***", g_ms, scans, e[0], nbss, scan_frames, htt_msdus);
		next_scan_ms = scans < 3 ? g_ms + 10000 : 0;
		api_scans_done++;
	}
}

static void
scan_mgmt_rx(const uint8_t *t, uint32_t len)
{
	uint32_t hdr[6];
	const uint8_t *f = NULL;
	uint32_t flen = 0;
	bool have_hdr = false;

	for (uint32_t o = 0; o + 4 <= len;) {
		uint16_t l, tag;
		memcpy(&l, t + o, 2);
		memcpy(&tag, t + o + 2, 2);
		if (o + 4 + l > len) {
			break;
		}
		if (tag == TAG_MGMT_RX_HDR && l >= 24 && !have_hdr) {
			memcpy(hdr, t + o + 4, sizeof(hdr));
			have_hdr = true;
		} else if (tag == TAG_ARRAY_BYTE && f == NULL) {
			f = t + o + 4;
			flen = l;
		}
		o += 4 + l;
	}
	if (!have_hdr || f == NULL) {
		return;
	}
	flen = MIN(hdr[4], flen);
	if (flen >= 24 && (f[0] & 0x0C) == 0 && (f[0] >> 4) != 8 && (f[0] >> 4) != 5) {
		w_assoc_mgmt_rx(f, flen);               /* auth, assoc response, deauth, disassoc */
		return;
	}
	if (flen < 36 || (f[0] & 0x0C) != 0 || ((f[0] >> 4) != 8 && (f[0] >> 4) != 5)) {
		return;                                 /* beacons and probe responses only */
	}
	scan_frames++;
	int32_t rssi = (int32_t)hdr[1] - 95;
	uint32_t chan = hdr[0];
	char ssid[33] = "";
	for (uint32_t o = 36; o + 2 <= flen && o + 2 + f[o + 1] <= flen; o += 2 + f[o + 1]) {
		if (f[o] == 0) {
			uint32_t n = MIN((uint32_t)f[o + 1], 32u);
			for (uint32_t i = 0; i < n; i++) {
				uint8_t ch = f[o + 2 + i];
				ssid[i] = (ch >= 0x20 && ch != 0x7F) ? (char)ch : '?';
			}
			ssid[n] = 0;
			if (n == 0 || f[o + 2] == 0) {
				strlcpy(ssid, "<hidden>", sizeof(ssid));
			}
		} else if (f[o] == 3 && f[o + 1] == 1) {
			chan = f[o + 2];
		}
	}
	struct bss *b = NULL;
	bool fresh = false;
	for (uint32_t i = 0; i < nbss; i++) {
		if (memcmp(bss[i].bssid, f + 16, 6) == 0) {
			b = &bss[i];
			break;
		}
	}
	if (b == NULL) {
		if (nbss < 64) {
			b = &bss[nbss++];
		} else {
			/*
			 * 0322: the table is full: the access point heard longest ago makes room. Before, once 64
			 * BSSIDs had been heard (a campus or a block of flats: every AP has one per band and per
			 * name) no new network was ever listed again, nor could it be joined ("no such network heard").
			 */
			uint32_t old = 0;
			for (uint32_t i = 1; i < nbss; i++) {
				if (bss[i].last_ms < bss[old].last_ms) {
					old = i;
				}
			}
			if (g_ms - bss[old].last_ms < 5000 || bss_is_ours(bss[old].bssid)) {
				return;                 /* all heard in the last 5 s (or it is ours): wait for the next scan */
			}
			b = &bss[old];
		}
		bzero(b, sizeof(*b));
		memcpy(b->bssid, f + 16, 6);
		fresh = true;
	}
	b->seen++;
	b->rssi = (int8_t)rssi;
	b->chan = (uint8_t)chan;
	b->freq = (uint16_t)(chan == 14 ? 2484 : chan < 14 ? 2407 + 5 * chan : 5000 + 5 * chan);
	b->last_ms = g_ms;
	memcpy(&b->bintval, f + 32, 2);
	memcpy(&b->capab, f + 34, 2);
	b->ie_len = (uint16_t)MIN(flen - 36, (uint32_t)sizeof(b->ie));
	memcpy(b->ie, f + 36, b->ie_len);
	strlcpy(b->ssid, ssid, sizeof(b->ssid));
	b->raw_ssid_len = 0;
	for (uint32_t o = 36; o + 2 <= flen && o + 2 + f[o + 1] <= flen; o += 2 + f[o + 1]) {
		if (f[o] == 0) {
			b->raw_ssid_len = (uint8_t)MIN((uint32_t)f[o + 1], 32u);
			memcpy(b->raw_ssid, f + o + 2, b->raw_ssid_len);
			break;
		}
	}
	if (!fresh) {
		return;
	}
	LOG("t=%u ms: net %2u: %02x:%02x:%02x:**:**:** ch %3u %4d dBm '%s'", g_ms, nbss, b->bssid[0], b->bssid[1],
	    b->bssid[2], b->chan, b->rssi, b->ssid);
}

/* ---------------- the radio for the rest of the kernel (sys-independent C API) */

/*
 * What another part of the kernel may ask of the radio once it is up: a scan, and what the
 * scans heard (each access point with its beacon's information elements, from which a reader
 * tells the network's protection). The modem thread does the work; a caller's scan waits.
 */
extern "C" {
struct topaz_wlan_bss {
	uint8_t bssid[6];
	uint8_t ssid[32], ssid_length;
	uint16_t frequency;                         /* MHz */
	int16_t rssi;                               /* dBm */
	uint16_t beacon_interval, capability;
	uint32_t age_ms;
	uint16_t elements_length;
	uint8_t elements[768];
};
void (*topaz_wlan_ready_hook)(void);
bool topaz_wlan_ready(void);
int topaz_wlan_scan(void);
bool topaz_wlan_heard(unsigned index, struct topaz_wlan_bss *bss);
void topaz_wlan_address(uint8_t address[6]);
}

static volatile bool api_scan_asked;
static bool api_ready;

bool
topaz_wlan_ready(void)
{
	return api_ready;
}

void
topaz_wlan_address(uint8_t address[6])
{
	memcpy(address, our_mac, 6);
}

int
topaz_wlan_scan(void)
{
	if (!api_ready) {
		return ENXIO;
	}
	uint32_t before = api_scans_done;
	api_scan_asked = true;
	for (int i = 0; i < 300 && api_scans_done == before; i++) {   /* 30 s */
		IOSleep(100);
	}
	return api_scans_done != before ? 0 : ETIMEDOUT;
}

bool
topaz_wlan_heard(unsigned index, struct topaz_wlan_bss *out)
{
	if (index >= nbss) {
		return false;
	}
	const struct bss *b = &bss[index];
	bzero(out, sizeof(*out));
	memcpy(out->bssid, b->bssid, 6);
	memcpy(out->ssid, b->raw_ssid, b->raw_ssid_len);
	out->ssid_length = b->raw_ssid_len;
	out->frequency = b->freq;
	out->rssi = b->rssi;
	out->beacon_interval = b->bintval;
	out->capability = b->capab;
	out->age_ms = g_ms - b->last_ms;
	out->elements_length = MIN(b->ie_len, (uint16_t)sizeof(out->elements));
	memcpy(out->elements, b->ie, out->elements_length);
	return true;
}

/* ---------------- WPA2-PSK crypto, self-contained (SHA-1, HMAC, PBKDF2, PRF, AES-128 unwrap) */

/*
 * The handshake is done here, in the kernel: wifid hands over the passphrase as typed. Small and
 * self-contained on purpose: libkern's SHA-1/AES go through corecrypto, which this kernel may not
 * have registered.
 */
struct w_sha1 { uint32_t h[5]; uint64_t n; uint8_t b[64]; uint32_t bl; };

static inline uint32_t rol32(uint32_t v, int s) { return (v << s) | (v >> (32 - s)); }

static void
w_sha1_block(struct w_sha1 *c, const uint8_t *p)
{
	uint32_t w[80], a = c->h[0], b = c->h[1], d = c->h[2], e = c->h[3], f = c->h[4];
	for (int i = 0; i < 16; i++) {
		w[i] = (uint32_t)p[4 * i] << 24 | (uint32_t)p[4 * i + 1] << 16 | (uint32_t)p[4 * i + 2] << 8 | p[4 * i + 3];
	}
	for (int i = 16; i < 80; i++) {
		w[i] = rol32(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
	}
	for (int i = 0; i < 80; i++) {
		uint32_t k, x;
		if (i < 20) {
			x = (b & d) | (~b & e); k = 0x5A827999;
		} else if (i < 40) {
			x = b ^ d ^ e; k = 0x6ED9EBA1;
		} else if (i < 60) {
			x = (b & d) | (b & e) | (d & e); k = 0x8F1BBCDC;
		} else {
			x = b ^ d ^ e; k = 0xCA62C1D6;
		}
		uint32_t t = rol32(a, 5) + x + f + k + w[i];
		f = e; e = d; d = rol32(b, 30); b = a; a = t;
	}
	c->h[0] += a; c->h[1] += b; c->h[2] += d; c->h[3] += e; c->h[4] += f;
}

static void
w_sha1_init(struct w_sha1 *c)
{
	c->h[0] = 0x67452301; c->h[1] = 0xEFCDAB89; c->h[2] = 0x98BADCFE; c->h[3] = 0x10325476; c->h[4] = 0xC3D2E1F0;
	c->n = 0;
	c->bl = 0;
}

static void
w_sha1_update(struct w_sha1 *c, const uint8_t *p, size_t len)
{
	c->n += len;
	while (len > 0) {
		uint32_t take = (uint32_t)MIN((size_t)(64 - c->bl), len);
		memcpy(c->b + c->bl, p, take);
		c->bl += take;
		p += take;
		len -= take;
		if (c->bl == 64) {
			w_sha1_block(c, c->b);
			c->bl = 0;
		}
	}
}

static void
w_sha1_final(struct w_sha1 *c, uint8_t out[20])
{
	uint64_t bits = c->n * 8;
	uint8_t pad = 0x80, z = 0;
	w_sha1_update(c, &pad, 1);
	while (c->bl != 56) {
		w_sha1_update(c, &z, 1);
	}
	uint8_t l[8];
	for (int i = 0; i < 8; i++) {
		l[i] = (uint8_t)(bits >> (56 - 8 * i));
	}
	w_sha1_update(c, l, 8);
	for (int i = 0; i < 5; i++) {
		out[4 * i] = (uint8_t)(c->h[i] >> 24);
		out[4 * i + 1] = (uint8_t)(c->h[i] >> 16);
		out[4 * i + 2] = (uint8_t)(c->h[i] >> 8);
		out[4 * i + 3] = (uint8_t)c->h[i];
	}
}

/* HMAC-SHA1 over up to three pieces */
static void
w_hmac_sha1(const uint8_t *key, size_t klen, const uint8_t *a, size_t al, const uint8_t *b, size_t bl,
    const uint8_t *c3, size_t cl, uint8_t out[20])
{
	uint8_t k[64] = {}, ipad[64], opad[64], inner[20];
	struct w_sha1 s;
	if (klen > 64) {
		w_sha1_init(&s);
		w_sha1_update(&s, key, klen);
		w_sha1_final(&s, k);
	} else {
		memcpy(k, key, klen);
	}
	for (int i = 0; i < 64; i++) {
		ipad[i] = k[i] ^ 0x36;
		opad[i] = k[i] ^ 0x5c;
	}
	w_sha1_init(&s);
	w_sha1_update(&s, ipad, 64);
	if (al) w_sha1_update(&s, a, al);
	if (bl) w_sha1_update(&s, b, bl);
	if (cl) w_sha1_update(&s, c3, cl);
	w_sha1_final(&s, inner);
	w_sha1_init(&s);
	w_sha1_update(&s, opad, 64);
	w_sha1_update(&s, inner, 20);
	w_sha1_final(&s, out);
}

/* PMK = PBKDF2-HMAC-SHA1(passphrase, ssid, 4096, 32) */
static void
w_pbkdf2_pmk(const uint8_t *pass, size_t plen, const uint8_t *ssid, size_t slen, uint8_t pmk[32])
{
	for (uint32_t blk = 1; blk <= 2; blk++) {
		uint8_t cnt[4] = { (uint8_t)(blk >> 24), (uint8_t)(blk >> 16), (uint8_t)(blk >> 8), (uint8_t)blk };
		uint8_t u[20], t[20];
		w_hmac_sha1(pass, plen, ssid, slen, cnt, 4, NULL, 0, u);
		memcpy(t, u, 20);
		for (int i = 1; i < 4096; i++) {
			w_hmac_sha1(pass, plen, u, 20, NULL, 0, NULL, 0, u);
			for (int j = 0; j < 20; j++) {
				t[j] ^= u[j];
			}
		}
		memcpy(pmk + 20 * (blk - 1), t, blk == 1 ? 20 : 12);
	}
}

/* IEEE 802.11 PRF-SHA1 */
static void
w_prf(const uint8_t *key, size_t klen, const char *label, const uint8_t *data, size_t dlen, uint8_t *out, size_t olen)
{
	uint8_t buf[128], h[20];
	size_t ll = strlen(label);
	for (uint8_t i = 0; olen > 0; i++) {
		memcpy(buf, label, ll);
		buf[ll] = 0;
		memcpy(buf + ll + 1, data, dlen);
		buf[ll + 1 + dlen] = i;
		w_hmac_sha1(key, klen, buf, ll + 2 + dlen, NULL, 0, NULL, 0, h);
		size_t take = MIN(olen, (size_t)20);
		memcpy(out, h, take);
		out += take;
		olen -= take;
	}
}

/* AES-128 decryption of one block (FIPS-197), for the key unwrap of the GTK */
static const uint8_t aes_sbox[256] = {
	0x63,0x7c,0x77,0x7b,0xf2,0x6b,0x6f,0xc5,0x30,0x01,0x67,0x2b,0xfe,0xd7,0xab,0x76,0xca,0x82,0xc9,0x7d,0xfa,0x59,0x47,0xf0,0xad,0xd4,0xa2,0xaf,0x9c,0xa4,0x72,0xc0,
	0xb7,0xfd,0x93,0x26,0x36,0x3f,0xf7,0xcc,0x34,0xa5,0xe5,0xf1,0x71,0xd8,0x31,0x15,0x04,0xc7,0x23,0xc3,0x18,0x96,0x05,0x9a,0x07,0x12,0x80,0xe2,0xeb,0x27,0xb2,0x75,
	0x09,0x83,0x2c,0x1a,0x1b,0x6e,0x5a,0xa0,0x52,0x3b,0xd6,0xb3,0x29,0xe3,0x2f,0x84,0x53,0xd1,0x00,0xed,0x20,0xfc,0xb1,0x5b,0x6a,0xcb,0xbe,0x39,0x4a,0x4c,0x58,0xcf,
	0xd0,0xef,0xaa,0xfb,0x43,0x4d,0x33,0x85,0x45,0xf9,0x02,0x7f,0x50,0x3c,0x9f,0xa8,0x51,0xa3,0x40,0x8f,0x92,0x9d,0x38,0xf5,0xbc,0xb6,0xda,0x21,0x10,0xff,0xf3,0xd2,
	0xcd,0x0c,0x13,0xec,0x5f,0x97,0x44,0x17,0xc4,0xa7,0x7e,0x3d,0x64,0x5d,0x19,0x73,0x60,0x81,0x4f,0xdc,0x22,0x2a,0x90,0x88,0x46,0xee,0xb8,0x14,0xde,0x5e,0x0b,0xdb,
	0xe0,0x32,0x3a,0x0a,0x49,0x06,0x24,0x5c,0xc2,0xd3,0xac,0x62,0x91,0x95,0xe4,0x79,0xe7,0xc8,0x37,0x6d,0x8d,0xd5,0x4e,0xa9,0x6c,0x56,0xf4,0xea,0x65,0x7a,0xae,0x08,
	0xba,0x78,0x25,0x2e,0x1c,0xa6,0xb4,0xc6,0xe8,0xdd,0x74,0x1f,0x4b,0xbd,0x8b,0x8a,0x70,0x3e,0xb5,0x66,0x48,0x03,0xf6,0x0e,0x61,0x35,0x57,0xb9,0x86,0xc1,0x1d,0x9e,
	0xe1,0xf8,0x98,0x11,0x69,0xd9,0x8e,0x94,0x9b,0x1e,0x87,0xe9,0xce,0x55,0x28,0xdf,0x8c,0xa1,0x89,0x0d,0xbf,0xe6,0x42,0x68,0x41,0x99,0x2d,0x0f,0xb0,0x54,0xbb,0x16,
};

static uint8_t aes_xt(uint8_t x) { return (uint8_t)((x << 1) ^ ((x & 0x80) ? 0x1b : 0)); }
static uint8_t aes_mul(uint8_t a, uint8_t b) { uint8_t r = 0; while (b) { if (b & 1) r ^= a; a = aes_xt(a); b >>= 1; } return r; }

static void
aes128_expand(const uint8_t key[16], uint8_t rk[176])
{
	uint8_t rcon = 1;
	memcpy(rk, key, 16);
	for (int i = 16; i < 176; i += 4) {
		uint8_t t[4];
		memcpy(t, rk + i - 4, 4);
		if (i % 16 == 0) {
			uint8_t x = t[0];
			t[0] = aes_sbox[t[1]] ^ rcon; t[1] = aes_sbox[t[2]]; t[2] = aes_sbox[t[3]]; t[3] = aes_sbox[x];
			rcon = aes_xt(rcon);
		}
		for (int j = 0; j < 4; j++) {
			rk[i + j] = rk[i - 16 + j] ^ t[j];
		}
	}
}

static void
aes128_decrypt(const uint8_t rk[176], uint8_t s[16])
{
	uint8_t inv[256];
	for (int i = 0; i < 256; i++) {
		inv[aes_sbox[i]] = (uint8_t)i;
	}
	for (int j = 0; j < 16; j++) {
		s[j] ^= rk[160 + j];
	}
	for (int r = 9; r >= 0; r--) {
		uint8_t t[16];
		for (int c = 0; c < 4; c++) {           /* inverse shift rows + inverse sub bytes */
			for (int row = 0; row < 4; row++) {
				t[4 * ((c + row) % 4) + row] = inv[s[4 * c + row]];
			}
		}
		for (int j = 0; j < 16; j++) {
			t[j] ^= rk[16 * r + j];
		}
		if (r == 0) {
			memcpy(s, t, 16);
			break;
		}
		for (int c = 0; c < 4; c++) {           /* inverse mix columns */
			uint8_t *q = t + 4 * c, a0 = q[0], a1 = q[1], a2 = q[2], a3 = q[3];
			s[4 * c] = aes_mul(a0, 14) ^ aes_mul(a1, 11) ^ aes_mul(a2, 13) ^ aes_mul(a3, 9);
			s[4 * c + 1] = aes_mul(a0, 9) ^ aes_mul(a1, 14) ^ aes_mul(a2, 11) ^ aes_mul(a3, 13);
			s[4 * c + 2] = aes_mul(a0, 13) ^ aes_mul(a1, 9) ^ aes_mul(a2, 14) ^ aes_mul(a3, 11);
			s[4 * c + 3] = aes_mul(a0, 11) ^ aes_mul(a1, 13) ^ aes_mul(a2, 9) ^ aes_mul(a3, 14);
		}
	}
}

/* RFC 3394 key unwrap with a 128-bit KEK; out gets len-8 bytes. False if the check value is wrong. */
static bool
aes_unwrap(const uint8_t kek[16], const uint8_t *in, size_t len, uint8_t *out)
{
	uint8_t rk[176], a[8], b[16];
	size_t n = len / 8 - 1;
	if (len < 24 || len % 8 != 0) {
		return false;
	}
	aes128_expand(kek, rk);
	memcpy(a, in, 8);
	memcpy(out, in + 8, len - 8);
	for (int j = 5; j >= 0; j--) {
		for (size_t i = n; i >= 1; i--) {
			uint64_t t = (uint64_t)n * (uint64_t)j + i;
			memcpy(b, a, 8);
			for (int k = 0; k < 8; k++) {
				b[7 - k] ^= (uint8_t)(t >> (8 * k));
			}
			memcpy(b + 8, out + 8 * (i - 1), 8);
			aes128_decrypt(rk, b);
			memcpy(a, b, 8);
			memcpy(out + 8 * (i - 1), b + 8, 8);
		}
	}
	for (int k = 0; k < 8; k++) {
		if (a[k] != 0xA6) {
			return false;
		}
	}
	return true;
}

static bool
w_crypto_selftest(void)
{
	static const uint8_t kek[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };
	static const uint8_t ct[24] = { 0x1F,0xA6,0x8B,0x0A,0x81,0x12,0xB4,0x47,0xAE,0xF3,0x4B,0xD8,0xFB,0x5A,0x7B,0x82,
		0x9D,0x3E,0x86,0x23,0x71,0xD2,0xCF,0xE5 };
	static const uint8_t pt[16] = { 0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xAA,0xBB,0xCC,0xDD,0xEE,0xFF };
	static const uint8_t pmk_ref[4] = { 0xf4, 0x2c, 0x6f, 0xc5 };     /* "password"/"IEEE" (802.11 annex) */
	uint8_t out[16], pmk[32];
	bool a = aes_unwrap(kek, ct, 24, out) && memcmp(out, pt, 16) == 0;
	w_pbkdf2_pmk((const uint8_t *)"password", 8, (const uint8_t *)"IEEE", 4, pmk);
	bool b = memcmp(pmk, pmk_ref, 4) == 0;
	LOG("wpa: crypto self-test: key unwrap %s, pbkdf2 %s", a ? "ok" : "FAILED", b ? "ok" : "FAILED");
	return a && b;
}

/* ---------------- station: HTT data TX, association, WPA2-PSK, Ethernet bridge */

/*
 * Port of topaz-woa assoc.c and the TX half of htt.c, plus the 4-way handshake that Windows did
 * there: VDEV_START on the AP's channel, PEER_CREATE, open-system AUTH and ASSOC REQ as WMI
 * management frames, PEER_ASSOC + VDEV_UP, then EAPOL over the data path, keys by
 * VDEV_INSTALL_KEY. Frames to and from the rest of the kernel are Ethernet; the radio's are
 * native wifi (802.11 header without QoS + LLC/SNAP).
 */
#define CMD_VDEV_START          0x5003
#define CMD_VDEV_UP             0x5005
#define CMD_VDEV_STOP           0x5006
#define CMD_VDEV_DOWN           0x5007
#define CMD_VDEV_INSTALL_KEY    0x5009
#define CMD_PEER_CREATE         0x6001
#define CMD_PEER_DELETE         0x6002
#define CMD_PEER_SET_PARAM      0x6004
#define CMD_PEER_ASSOC          0x6005
#define CMD_MGMT_TX_SEND        0x7008
#define CMD_STA_PS_MODE         0x9001
#define TAG_VDEV_START          0x58
#define TAG_VDEV_UP             0x5c
#define TAG_VDEV_STOP           0x5d
#define TAG_VDEV_DOWN           0x5e
#define TAG_VDEV_INSTALL_KEY    0x60
#define TAG_PEER_CREATE         0x61
#define TAG_PEER_DELETE         0x62
#define TAG_PEER_SET_PARAM      0x64
#define TAG_PEER_ASSOC          0x65
#define TAG_VHT_RATE_SET        0x66
#define TAG_STA_PS_MODE         0x6c
#define TAG_MGMT_TX             0x1a6
#define PEER_FLAG_AUTH          0x1
#define PEER_FLAG_NEED_PTK_4WAY 0x4
#define PEER_PARAM_AUTHORIZE    3
#define WMI_CIPHER_TKIP         2
#define WMI_CIPHER_CCMP         4
#define FC_ASSOC_REQ            0x00
#define FC_ASSOC_RESP           0x10
#define FC_DISASSOC             0xA0
#define FC_AUTH                 0xB0
#define FC_DEAUTH               0xC0

enum sta_state { S_IDLE, S_START, S_AUTH, S_ASSOC, S_4WAY, S_UP };
static volatile enum sta_state s_state;
static volatile int s_result;                   /* of the last join: 0 joined, else an errno (EAUTH: passphrase) */
static volatile bool s_join_asked, s_leave_asked;
static volatile uint32_t s_join_done;           /* bumped when a join ends either way */
static uint8_t s_ssid[32], s_ssid_len, s_bssid[6], s_pmk[32];

/* 0322: the access point being joined or joined (never aged out of the scan table) */
static bool
bss_is_ours(const uint8_t *bssid)
{
	static const uint8_t none[6] = {};
	return memcmp(s_bssid, none, 6) != 0 && memcmp(bssid, s_bssid, 6) == 0;
}
static uint8_t s_body[768];                     /* the AP's beacon IEs */
static uint32_t s_body_len, s_freq, s_aid, s_tries, s_deadline, s_desc, s_msg1s;
static bool s_rsn, s_start_resp, s_ptk_set;
static int32_t s_start_status;
static uint8_t s_assoc_req[512];
static uint32_t s_assoc_req_len;
static uint8_t s_rsn_ie[22];
static uint8_t s_ptk[48], s_anonce[32], s_snonce[32], s_group_cipher;
static uint64_t s_replay;
static uint8_t *s_mgmt_va;
static uint64_t s_mgmt_pa;

void (*topaz_wlan_rx_hook)(const uint8_t *frame, size_t length);
void (*topaz_wlan_link_hook)(bool up);

static uint8_t *w_p32(uint8_t *p, uint32_t v) { memcpy(p, &v, 4); return p + 4; }
static uint8_t *w_pmac(uint8_t *p, const uint8_t *mac) { bzero(p, 8); memcpy(p, mac, 6); return p + 8; }

static uint8_t *
w_channel(uint8_t *p, uint32_t freq)
{
	uint8_t *c = wmi_put_tlv(p, TAG_CHANNEL, 24);
	c = w_p32(c, freq);
	c = w_p32(c, freq);
	c = w_p32(c, 0);
	c = w_p32(c, freq < 4000 ? 1 : 0);
	c = w_p32(c, (40u << 8) | (40u << 16));
	return w_p32(c, 40u << 8);
}

static const uint8_t *
w_find_ie(const uint8_t *ies, uint32_t len, uint8_t id)
{
	for (uint32_t o = 0; o + 2 <= len && o + 2 + ies[o + 1] <= len; o += 2 + ies[o + 1]) {
		if (ies[o] == id) {
			return ies + o;
		}
	}
	return NULL;
}

/* ---- HTT data TX (ath10k_htt_tx_64, native wifi) */
#define TX_SLOTS        64
#define TX_SLOT_SIZE    2048
#define H2T_TX_FRM      1
#define TXF0_MAC_HDR    0x01
#define TXF0_NO_ENCRYPT 0x04
#define TXF0_NATIVE     (1u << 5)
#define TXF1_TID_NONQOS (16u << 6)
#define TXF1_POSTPONED  (1u << 11)
#define PREFETCH_LEN    50
#define T2H_TX_COMPL_IND 0x07

static uint8_t *tx_pool;
static uint64_t tx_pool_pa, tx_busy;
static uint32_t tx_sent, tx_done, tx_fail, tx_drop, rx_up;

static bool
htt_tx_frame(const uint8_t *f, uint32_t len, bool clear)
{
	uint8_t m[20 + 52], *p = m;
	uint32_t id, pf = (MIN(len, (uint32_t)PREFETCH_LEN) + 3) & ~3u;
	if (tx_pool == NULL) {
		tx_pool = (uint8_t *)pool_alloc(TX_SLOTS * TX_SLOT_SIZE, &tx_pool_pa);
	}
	if (tx_pool == NULL || htt_bank == NULL || htt_state != 2 || len > TX_SLOT_SIZE) {
		return false;
	}
	for (id = 0; id < TX_SLOTS && (tx_busy & (1ull << id)) != 0; id++) {
	}
	if (id == TX_SLOTS) {
		return false;
	}
	uint64_t pa = tx_pool_pa + (uint64_t)id * TX_SLOT_SIZE;
	memcpy(tx_pool + id * TX_SLOT_SIZE, f, len);
	uint8_t *ext = htt_bank + id * HTT_EXT_DESC_SIZE;  /* frags[0] at +24: paddr_lo, paddr_hi16, len16 */
	bzero(ext, HTT_EXT_DESC_SIZE);
	uint32_t lo = (uint32_t)pa;
	uint16_t hi = (uint16_t)(pa >> 32), l16 = (uint16_t)len;
	memcpy(ext + 24, &lo, 4);
	memcpy(ext + 28, &hi, 2);
	memcpy(ext + 30, &l16, 2);
	mb();
	bzero(m, sizeof(m));
	*p++ = H2T_TX_FRM;
	*p++ = (uint8_t)(TXF0_MAC_HDR | TXF0_NATIVE | (clear ? TXF0_NO_ENCRYPT : 0));
	p = put16(p, (uint16_t)(TXF1_TID_NONQOS | TXF1_POSTPONED));   /* vdev 0 */
	p = put16(p, (uint16_t)len);
	p = put16(p, (uint16_t)id);
	p = put64(p, htt_bank_pa + (uint64_t)id * HTT_EXT_DESC_SIZE);
	p = w_p32(p, 0xFFFF);                       /* peer id: invalid */
	memcpy(p, f, MIN(len, pf));
	if (!htt_send(m, 20 + pf)) {
		return false;
	}
	tx_busy |= 1ull << id;
	tx_sent++;
	return true;
}

static void
htt_tx_compl(const uint8_t *p, uint32_t len)
{
	uint32_t n = p[2], st = p[1] & 7;
	for (uint32_t i = 0; i < n && 4 + 2 * (i + 1) <= len; i++) {
		uint16_t id;
		memcpy(&id, p + 4 + 2 * i, 2);
		if (id < TX_SLOTS) {
			tx_busy &= ~(1ull << id);
		}
	}
	tx_done += n;
	if (st != 0) {
		tx_fail += n;
		if (tx_fail <= 4) {
			LOG("t=%u ms: htt tx: %u frame(s) status %u", g_ms, n, st);
		}
	}
}

/* ---- WMI commands for the join (layouts: ath10k wmi-tlv) */
static bool
w_vdev_start(uint32_t freq, uint32_t bcn, uint32_t dtim)
{
	uint8_t m[4 + 72 + 4 + 24 + 4] = {}, *p;
	p = wmi_put_tlv(m, TAG_VDEV_START, 72);
	p = w_p32(p, 0);
	p = w_p32(p, 0);
	p = w_p32(p, bcn);
	p = w_p32(p, dtim);
	p = w_p32(p, 0);
	p += 36 + 16;                               /* ssid (empty for a station), tx rate/power, noa, hw ack */
	p = w_channel(p, freq);
	wmi_put_tlv(p, TAG_ARRAY_STRUCT, 0);
	return wmi_send(CMD_VDEV_START, m, sizeof(m));
}

static bool
w_vdev_simple(uint32_t cmd, uint16_t tag)
{
	uint8_t m[8], *p = wmi_put_tlv(m, tag, 4);
	w_p32(p, 0);
	return wmi_send(cmd, m, sizeof(m));
}

static bool
w_peer(uint32_t cmd, uint16_t tag, bool create)
{
	uint8_t m[4 + 16] = {}, *p = wmi_put_tlv(m, tag, create ? 16 : 12);
	p = w_p32(p, 0);
	p = w_pmac(p, s_bssid);
	if (create) {
		w_p32(p, 0);
	}
	return wmi_send(cmd, m, create ? 20 : 16);
}

static bool
w_peer_param(uint32_t param, uint32_t value)
{
	uint8_t m[4 + 20], *p = wmi_put_tlv(m, TAG_PEER_SET_PARAM, 20);
	p = w_p32(p, 0);
	p = w_pmac(p, s_bssid);
	p = w_p32(p, param);
	w_p32(p, value);
	return wmi_send(CMD_PEER_SET_PARAM, m, sizeof(m));
}

static bool
w_ps_off(void)
{
	uint8_t m[12], *p = wmi_put_tlv(m, TAG_STA_PS_MODE, 8);
	p = w_p32(p, 0);
	w_p32(p, 0);
	return wmi_send(CMD_STA_PS_MODE, m, sizeof(m));
}

static bool
w_mgmt_tx(const uint8_t *f, uint32_t len)
{
	uint8_t m[4 + 28 + 4 + 64] = {}, *p;
	uint32_t buf = (MIN(len, 64u) + 3) & ~3u;
	if (s_mgmt_va == NULL) {
		s_mgmt_va = (uint8_t *)pool_alloc(0x1000, &s_mgmt_pa);
	}
	if (s_mgmt_va == NULL || len > 0x1000 - 8) {
		return false;
	}
	bzero(s_mgmt_va, ((len + 3) & ~3u) + 4);
	memcpy(s_mgmt_va, f, len);
	mb();
	p = wmi_put_tlv(m, TAG_MGMT_TX, 28);
	p = w_p32(p, 0);
	p = w_p32(p, ++s_desc);
	p = w_p32(p, 0);
	p = put64(p, s_mgmt_pa);
	p = w_p32(p, len);
	p = w_p32(p, buf);
	p = wmi_put_tlv(p, TAG_ARRAY_BYTE, buf);
	memcpy(p, s_mgmt_va, buf);
	return wmi_send(CMD_MGMT_TX_SEND, m, 4 + 28 + 4 + buf);
}

static uint32_t
w_ap_rates(uint8_t *out)
{
	static const uint8_t ids[2] = { 1, 50 };
	uint32_t n = 0;
	for (int k = 0; k < 2; k++) {
		const uint8_t *ie = w_find_ie(s_body, s_body_len, ids[k]);
		for (uint32_t i = 0; ie != NULL && i < ie[1] && n < 16; i++) {
			out[n++] = ie[2 + i];
		}
	}
	return n;
}

static bool
w_peer_assoc(uint32_t caps)
{
	uint8_t m[4 + 76 + 4 + 16 + 4 + 4 + 16] = {}, *p, *r, rates[16];
	uint32_t n = w_ap_rates(rates), rl = (MIN(n, 16u) + 3) & ~3u;
	p = wmi_put_tlv(m, TAG_PEER_ASSOC, 76);
	p = w_pmac(p, s_bssid);
	p = w_p32(p, 0);
	p = w_p32(p, 1);                            /* new assoc */
	p = w_p32(p, s_aid);
	p = w_p32(p, PEER_FLAG_AUTH | (s_rsn ? PEER_FLAG_NEED_PTK_4WAY : 0));
	p = w_p32(p, caps);
	p = w_p32(p, 1);                            /* listen interval */
	p += 4 * 6;                                 /* ht caps, max mpdu, density, rate caps; nss below */
	p -= 4 * 2;
	p = w_p32(p, 1);                            /* nss */
	p = w_p32(p, 0);                            /* vht caps */
	p = w_p32(p, s_freq < 4000 ? 1 : 0);        /* phy mode: 11g / 11a */
	p += 8;
	p = w_p32(p, MIN(n, 16u));
	p = w_p32(p, 0);
	r = wmi_put_tlv(p, TAG_ARRAY_BYTE, rl);
	for (uint32_t i = 0; i < n && i < 16; i++) {
		uint8_t v = rates[i] & 0x7F;
		r[i] = (v == 2 || v == 4 || v == 11 || v == 22) ? (uint8_t)(v | 0x80) : v;
	}
	p = r + rl;
	p = wmi_put_tlv(p, TAG_ARRAY_BYTE, 0);
	wmi_put_tlv(p, TAG_VHT_RATE_SET, 16);
	return wmi_send(CMD_PEER_ASSOC, m, 4 + 76 + 4 + rl + 4 + 4 + 16);
}

static bool
w_install_key(bool group, uint32_t idx, uint32_t cipher, const uint8_t *key, uint32_t klen)
{
	uint8_t m[4 + 92 + 4 + 32] = {}, *p;
	uint32_t kl = (klen + 3) & ~3u;
	p = wmi_put_tlv(m, TAG_VDEV_INSTALL_KEY, 92);
	p = w_p32(p, 0);
	p = w_pmac(p, s_bssid);
	p = w_p32(p, idx);
	p = w_p32(p, group ? 1 : 0);
	p = w_p32(p, cipher);
	p += 3 * 8 + 16 + 16;
	p = w_p32(p, klen);
	p = w_p32(p, cipher == WMI_CIPHER_TKIP ? 8 : 0);
	p = w_p32(p, cipher == WMI_CIPHER_TKIP ? 8 : 0);
	p = wmi_put_tlv(p, TAG_ARRAY_BYTE, kl);
	memcpy(p, key, klen);
	return wmi_send(CMD_VDEV_INSTALL_KEY, m, 4 + 92 + 4 + kl);
}

/* ---- 802.11 management frames */
static uint8_t *
w_mgmt_hdr(uint8_t *f, uint8_t fc)
{
	bzero(f, 24);
	f[0] = fc;
	memcpy(f + 4, s_bssid, 6);
	memcpy(f + 10, our_mac, 6);
	memcpy(f + 16, s_bssid, 6);
	return f + 24;
}

static bool
w_send_auth(void)
{
	uint8_t f[30], *b = w_mgmt_hdr(f, FC_AUTH);
	bzero(b, 6);
	b[2] = 1;                                   /* open system, transaction 1 */
	return w_mgmt_tx(f, sizeof(f));
}

static void
w_build_assoc_req(void)
{
	uint8_t *f = s_assoc_req, *p = w_mgmt_hdr(f, FC_ASSOC_REQ), rates[16];
	uint32_t n = w_ap_rates(rates), cap = 0x0001 | 0x0020 | 0x0400;   /* ESS, short preamble, short slot */
	if (s_rsn) {
		cap |= 0x0010;
	}
	*p++ = (uint8_t)cap;
	*p++ = (uint8_t)(cap >> 8);
	*p++ = 10;
	*p++ = 0;
	*p++ = 0;
	*p++ = s_ssid_len;
	memcpy(p, s_ssid, s_ssid_len);
	p += s_ssid_len;
	*p++ = 1;
	*p++ = (uint8_t)MIN(n, 8u);
	for (uint32_t k = 0; k < n && k < 8; k++) {
		*p++ = rates[k];
	}
	if (n > 8) {
		*p++ = 50;
		*p++ = (uint8_t)(n - 8);
		for (uint32_t k = 8; k < n; k++) {
			*p++ = rates[k];
		}
	}
	if (s_rsn) {
		memcpy(p, s_rsn_ie, sizeof(s_rsn_ie));
		p += sizeof(s_rsn_ie);
	}
	s_assoc_req_len = (uint32_t)(p - f);
}

/* ---- Ethernet <-> native wifi */
static void
w_link(bool up)
{
	if (topaz_wlan_link_hook != NULL) {
		topaz_wlan_link_hook(up);
	}
}

/* an Ethernet frame (dst, src, type, payload) to the AP as a to-DS data frame */
static bool
w_tx_ether(const uint8_t *e, size_t len, bool clear)
{
	uint8_t f[TX_SLOT_SIZE];
	if (len < 14 || len - 14 + 24 + 8 > sizeof(f)) {
		return false;
	}
	bzero(f, 24);
	f[0] = 0x08;                                /* data */
	f[1] = (uint8_t)(0x01 | (s_ptk_set && !clear ? 0x40 : 0));   /* to DS (+ protected once keyed) */
	memcpy(f + 4, s_bssid, 6);                  /* addr1: the AP */
	memcpy(f + 10, our_mac, 6);                 /* addr2: us */
	memcpy(f + 16, e, 6);                       /* addr3: the destination */
	static const uint8_t snap[6] = { 0xAA, 0xAA, 0x03, 0x00, 0x00, 0x00 };
	memcpy(f + 24, snap, 6);
	memcpy(f + 30, e + 12, 2);                  /* ethertype */
	memcpy(f + 32, e + 14, len - 14);
	return htt_tx_frame(f, (uint32_t)(len - 14 + 32), clear);
}

/* ---- WPA2 4-way handshake (EAPOL-Key, descriptor version 2: HMAC-SHA1-128 MIC, AES key wrap) */
static void w_join_end(int result);

static void
w_eapol_send(uint16_t info, const uint8_t *nonce, const uint8_t *data, uint16_t dlen)
{
	uint8_t e[14 + 4 + 95 + 64] = {}, *k = e + 14 + 4, mic[20];
	memcpy(e, s_bssid, 6);
	memcpy(e + 6, our_mac, 6);
	e[12] = 0x88;
	e[13] = 0x8E;
	uint16_t body = (uint16_t)(95 + dlen);
	e[14] = 1;                                  /* EAPOL version */
	e[15] = 3;                                  /* key */
	e[16] = (uint8_t)(body >> 8);
	e[17] = (uint8_t)body;
	k[0] = 2;                                   /* RSN key descriptor */
	k[1] = (uint8_t)(info >> 8);
	k[2] = (uint8_t)info;
	k[3] = 0;                                   /* key length 0 (from the STA) */
	k[4] = 0;
	for (int i = 0; i < 8; i++) {
		k[5 + i] = (uint8_t)(s_replay >> (56 - 8 * i));
	}
	if (nonce != NULL) {
		memcpy(k + 13, nonce, 32);
	}
	k[93] = (uint8_t)(dlen >> 8);
	k[94] = (uint8_t)dlen;
	if (dlen) {
		memcpy(k + 95, data, dlen);
	}
	w_hmac_sha1(s_ptk, 16, e + 14, 4 + body, NULL, 0, NULL, 0, mic);
	memcpy(k + 77, mic, 16);
	(void)w_tx_ether(e, 14 + 4 + body, true);
}

static bool
w_eapol_mic_ok(uint8_t *eapol, size_t len)
{
	uint8_t want[16], got[20];
	if (len < 4 + 95) {
		return false;
	}
	memcpy(want, eapol + 4 + 77, 16);
	bzero(eapol + 4 + 77, 16);
	w_hmac_sha1(s_ptk, 16, eapol, len, NULL, 0, NULL, 0, got);
	memcpy(eapol + 4 + 77, want, 16);
	return memcmp(want, got, 16) == 0;
}

static void
w_eapol_rx(const uint8_t *e, size_t elen)
{
	uint8_t frame[512];
	const uint8_t *src = e + 6;
	if (s_state != S_4WAY || elen < 14 + 4 + 95 || elen - 14 > sizeof(frame) || memcmp(src, s_bssid, 6) != 0) {
		return;
	}
	size_t len = elen - 14;
	memcpy(frame, e + 14, len);
	uint8_t *k = frame + 4;
	if (frame[1] != 3 || k[0] != 2) {
		return;
	}
	uint16_t info = (uint16_t)(k[1] << 8 | k[2]), dlen = (uint16_t)(k[93] << 8 | k[94]);
	uint64_t replay = 0;
	for (int i = 0; i < 8; i++) {
		replay = replay << 8 | k[5 + i];
	}
	if (95 + (size_t)dlen > len - 4) {
		return;
	}
	bool pairwise = info & 0x0008, ack = info & 0x0080, has_mic = info & 0x0100, install = info & 0x0040;
	if (pairwise && ack && !has_mic) {
		/* message 1: ANonce -> PTK, answer with message 2 */
		if (++s_msg1s > 3) {
			LOG("t=%u ms: wpa: the AP keeps sending message 1: wrong passphrase", g_ms);
			w_join_end(EAUTH);
			return;
		}
		s_replay = replay;
		memcpy(s_anonce, k + 13, 32);
		read_random(s_snonce, 32);
		uint8_t data[76];
		const uint8_t *aa = s_bssid, *spa = our_mac;
		bool amin = memcmp(aa, spa, 6) < 0, nmin = memcmp(s_anonce, s_snonce, 32) < 0;
		memcpy(data, amin ? aa : spa, 6);
		memcpy(data + 6, amin ? spa : aa, 6);
		memcpy(data + 12, nmin ? s_anonce : s_snonce, 32);
		memcpy(data + 44, nmin ? s_snonce : s_anonce, 32);
		w_prf(s_pmk, 32, "Pairwise key expansion", data, sizeof(data), s_ptk, 48);
		w_eapol_send(0x010A, s_snonce, s_rsn_ie, sizeof(s_rsn_ie));
		LOG("t=%u ms: wpa: message 1 -> message 2 sent", g_ms);
		return;
	}
	if (pairwise && ack && has_mic && install) {
		/* message 3: check, unwrap the GTK, answer with message 4, install the keys */
		if (!w_eapol_mic_ok(frame, len)) {
			LOG("t=%u ms: wpa: message 3 MIC wrong: wrong passphrase", g_ms);
			w_join_end(EAUTH);
			return;
		}
		s_replay = replay;
		uint8_t plain[256];
		uint8_t gtk[32], gtk_len = 0, gtk_idx = 1;
		if ((info & 0x1000) && dlen >= 24 && dlen <= sizeof(plain) + 8 && aes_unwrap(s_ptk + 16, k + 95, dlen, plain)) {
			for (uint32_t o = 0; o + 2 <= (uint32_t)dlen - 8 && o + 2 + plain[o + 1] <= (uint32_t)dlen - 8; o += 2 + plain[o + 1]) {
				if (plain[o] == 0xDD && plain[o + 1] >= 6 && plain[o + 2] == 0x00 && plain[o + 3] == 0x0F &&
				    plain[o + 4] == 0xAC && plain[o + 5] == 0x01) {
					gtk_idx = plain[o + 6] & 3;
					gtk_len = (uint8_t)MIN((uint32_t)plain[o + 1] - 6, 32u);
					memcpy(gtk, plain + o + 8, gtk_len);
				}
			}
		}
		w_eapol_send(0x030A, NULL, NULL, 0);
		IOSleep(5);                             /* message 4 out in the clear before the keys go in */
		w_install_key(false, 0, WMI_CIPHER_CCMP, s_ptk + 32, 16);
		if (gtk_len != 0) {
			w_install_key(true, gtk_idx, s_group_cipher == 2 ? WMI_CIPHER_TKIP : WMI_CIPHER_CCMP, gtk, gtk_len);
		}
		w_peer_param(PEER_PARAM_AUTHORIZE, 1);
		s_ptk_set = true;
		bzero(gtk, sizeof(gtk));
		LOG("t=%u ms: *** WPA2 keys installed (group key %s, idx %u) ***", g_ms, gtk_len ? "yes" : "NO", gtk_idx);
		w_join_end(0);
		return;
	}
}

/* ---- the join, step by step from the modem thread */
static void
w_teardown(void)
{
	if (s_state >= S_START) {
		if (s_state >= S_4WAY) {
			w_vdev_simple(CMD_VDEV_DOWN, TAG_VDEV_DOWN);
		}
		if (s_state >= S_AUTH) {
			w_peer(CMD_PEER_DELETE, TAG_PEER_DELETE, false);
		}
		w_vdev_simple(CMD_VDEV_STOP, TAG_VDEV_STOP);
	}
	if (s_state == S_UP) {
		w_link(false);
	}
	s_state = S_IDLE;
	s_ptk_set = false;
}

static void
w_join_end(int result)
{
	if (result != 0) {
		LOG("t=%u ms: join: failed (%d) in state %u", g_ms, result, s_state);
		w_teardown();
	} else {
		s_state = S_UP;
		LOG("t=%u ms: *** joined %.*s ***", g_ms, (int)s_ssid_len, s_ssid);
		w_link(true);
	}
	bzero(s_pmk, sizeof(s_pmk));
	s_result = result;
	s_join_done++;
}

static void
w_join_begin(void)
{
	/* the AP: the strongest one heard with that name (or the BSSID asked for) */
	int best = -1;
	for (uint32_t i = 0; i < nbss; i++) {
		if (bss[i].raw_ssid_len == s_ssid_len && memcmp(bss[i].raw_ssid, s_ssid, s_ssid_len) == 0 &&
		    (best < 0 || bss[i].rssi > bss[best].rssi)) {
			best = (int)i;
		}
	}
	bool want_bssid = false;
	for (int i = 0; i < 6; i++) {
		want_bssid |= s_bssid[i] != 0;
	}
	if (want_bssid) {
		for (uint32_t i = 0; i < nbss; i++) {
			if (memcmp(bss[i].bssid, s_bssid, 6) == 0) {
				best = (int)i;
			}
		}
	}
	if (best < 0) {
		LOG("join: no such network heard");
		w_join_end(ENOENT);
		return;
	}
	memcpy(s_bssid, bss[best].bssid, 6);
	s_freq = bss[best].freq;
	s_body_len = bss[best].ie_len;
	memcpy(s_body, bss[best].ie, s_body_len);
	const uint8_t *rsn = w_find_ie(s_body, s_body_len, 48);
	s_rsn = rsn != NULL;
	s_group_cipher = (rsn != NULL && rsn[1] >= 8) ? rsn[2 + 2 + 3] : 4;
	static const uint8_t hdr[4] = { 48, 20, 1, 0 };
	memcpy(s_rsn_ie, hdr, 4);
	uint8_t *q = s_rsn_ie + 4;
	*q++ = 0x00; *q++ = 0x0F; *q++ = 0xAC; *q++ = s_group_cipher;
	*q++ = 1; *q++ = 0; *q++ = 0x00; *q++ = 0x0F; *q++ = 0xAC; *q++ = 4;   /* pairwise CCMP */
	*q++ = 1; *q++ = 0; *q++ = 0x00; *q++ = 0x0F; *q++ = 0xAC; *q++ = 2;   /* AKM PSK */
	*q++ = 0; *q++ = 0;
	uint32_t bcn = bss[best].bintval ? bss[best].bintval : 100, dtim = 1;
	const uint8_t *tim = w_find_ie(s_body, s_body_len, 5);
	if (tim != NULL && tim[1] >= 2 && tim[3] != 0) {
		dtim = tim[3];
	}
	s_msg1s = 0;
	s_start_resp = false;
	LOG("t=%u ms: join: %.*s %02x:%02x:%02x:**:**:** %u MHz %s", g_ms, (int)s_ssid_len, s_ssid, s_bssid[0], s_bssid[1],
	    s_bssid[2], s_freq, s_rsn ? "WPA2" : "open");
	w_ps_off();
	if (!w_vdev_start(s_freq, bcn, dtim)) {
		w_join_end(EIO);
		return;
	}
	s_state = S_START;
	s_deadline = g_ms + 1500;
}

static void
w_assoc_mgmt_rx(const uint8_t *f, uint32_t len)
{
	if (len < 26 || s_state == S_IDLE || memcmp(f + 10, s_bssid, 6) != 0) {
		return;
	}
	uint8_t fc = f[0];
	uint16_t v = (uint16_t)(f[24] | f[25] << 8);
	if (fc == FC_AUTH && s_state == S_AUTH && len >= 30 && f[26] == 2) {
		v = (uint16_t)(f[28] | f[29] << 8);
		LOG("t=%u ms: join: auth reply status %u", g_ms, v);
		if (v != 0) {
			w_join_end(ECONNREFUSED);
			return;
		}
		s_state = S_ASSOC;
		s_tries = 1;
		w_build_assoc_req();
		w_mgmt_tx(s_assoc_req, s_assoc_req_len);
		s_deadline = g_ms + 400;
	} else if (fc == FC_ASSOC_RESP && s_state == S_ASSOC && len >= 30) {
		v = (uint16_t)(f[26] | f[27] << 8);
		s_aid = (uint32_t)(f[28] | f[29] << 8) & 0x3FFF;
		LOG("t=%u ms: join: assoc reply status %u aid %u", g_ms, v, s_aid);
		if (v != 0) {
			w_join_end(ECONNREFUSED);
			return;
		}
		w_peer_assoc((uint32_t)(f[24] | f[25] << 8));
		uint8_t m[20], *p = wmi_put_tlv(m, TAG_VDEV_UP, 16);
		p = w_p32(p, 0);
		p = w_p32(p, s_aid);
		w_pmac(p, s_bssid);
		wmi_send(CMD_VDEV_UP, m, sizeof(m));
		if (!s_rsn) {
			w_peer_param(PEER_PARAM_AUTHORIZE, 1);
			s_state = S_4WAY;                   /* (teardown order) */
			w_join_end(0);
		} else {
			s_state = S_4WAY;
			s_deadline = g_ms + 8000;
			LOG("t=%u ms: join: associated, waiting for the 4-way handshake", g_ms);
		}
	} else if (fc == FC_DEAUTH || fc == FC_DISASSOC) {
		LOG("t=%u ms: join: the AP ended the link, reason %u (state %u)", g_ms, v, s_state);
		if (s_state == S_UP) {
			w_teardown();
		} else {
			w_join_end(s_state == S_4WAY ? EAUTH : ECONNREFUSED);
		}
	}
}

static void
w_join_event(uint32_t id, const uint8_t *t, uint32_t len)
{
	uint16_t l = 0;
	uint32_t v[4] = {};
	if (len >= 4) {
		memcpy(&l, t, 2);
	}
	if (len < 4u + l) {
		return;
	}
	memcpy(v, t + 4, MIN((uint32_t)l, (uint32_t)sizeof(v)));
	if (id == 0x5001 && l >= 16) {             /* VDEV_START_RESP {vdev, req, type, status} */
		s_start_status = (int32_t)v[3];
		s_start_resp = true;
	} else if (id == 0x6001 && s_state == S_UP) {   /* PEER_STA_KICKOUT */
		LOG("t=%u ms: join: peer kickout", g_ms);
		w_teardown();
	}
}

/* every pass of the modem loop */
static void
w_join_poll(void)
{
	if (s_leave_asked) {
		s_leave_asked = false;
		w_teardown();
	}
	if (s_join_asked && !scanning) {
		s_join_asked = false;
		w_teardown();
		w_join_begin();
		return;
	}
	if (s_state == S_START) {
		if (s_start_resp) {
			if (s_start_status != 0) {
				w_join_end(EIO);
				return;
			}
			s_state = S_AUTH;
			s_tries = 1;
			w_peer(CMD_PEER_CREATE, TAG_PEER_CREATE, true);
			w_send_auth();
			s_deadline = g_ms + 400;
		} else if (g_ms > s_deadline) {
			w_join_end(ETIMEDOUT);
		}
	} else if ((s_state == S_AUTH || s_state == S_ASSOC) && g_ms > s_deadline) {
		if (s_tries >= 4) {
			w_join_end(ETIMEDOUT);
		} else {
			s_tries++;
			if (s_state == S_AUTH) {
				w_send_auth();
			} else {
				w_mgmt_tx(s_assoc_req, s_assoc_req_len);
			}
			s_deadline = g_ms + 400;
		}
	} else if (s_state == S_4WAY && g_ms > s_deadline) {
		LOG("t=%u ms: join: no 4-way handshake (%u message 1s)", g_ms, s_msg1s);
		w_join_end(s_msg1s != 0 ? EAUTH : ETIMEDOUT);
	}
}

/* a received data frame (native wifi, from the HTT rx ring) */
static void
w_rx_native(const uint8_t *f, uint32_t len)
{
	uint8_t e[14 + 1600];
	if (len < 24 + 8 || (f[0] & 0x0C) != 0x08) {
		return;                                 /* data frames only */
	}
	uint32_t hl = ((f[1] & 3) == 3) ? 30 : 24;
	const uint8_t *da = f + 4, *sa = (f[1] & 2) ? f + 16 : f + 10;   /* from DS: addr3 is the source */
	const uint8_t *llc = f + hl;
	if (len < hl + 8 || llc[0] != 0xAA || llc[1] != 0xAA || llc[2] != 0x03) {
		return;
	}
	uint32_t plen = len - hl - 8;
	if (plen > 1600 - 0) {
		return;
	}
	memcpy(e, da, 6);
	memcpy(e + 6, sa, 6);
	memcpy(e + 12, llc + 6, 2);
	memcpy(e + 14, llc + 8, plen);
	if (e[12] == 0x88 && e[13] == 0x8E) {
		w_eapol_rx(e, 14 + plen);
		return;
	}
	if (s_state == S_UP && topaz_wlan_rx_hook != NULL) {
		rx_up++;
		topaz_wlan_rx_hook(e, 14 + plen);
	}
}

/* ---- the C API */
extern "C" {
int topaz_wlan_join(const uint8_t *ssid, unsigned ssid_len, const uint8_t *bssid, const uint8_t *pass, unsigned pass_len);
int topaz_wlan_join_pmk(const uint8_t *ssid, unsigned ssid_len, const uint8_t *bssid, const uint8_t pmk[32]);
int topaz_wlan_leave(void);
bool topaz_wlan_joined(void);
bool topaz_wlan_transmit(const uint8_t *frame, size_t length);
}

static int w_join_with(const uint8_t *ssid, unsigned ssid_len, const uint8_t *bssid, const uint8_t *pmk);

/* Blocks until joined or failed (EAUTH: wrong passphrase; ENOENT: not heard; ETIMEDOUT). */
int
topaz_wlan_join(const uint8_t *ssid, unsigned ssid_len, const uint8_t *bssid, const uint8_t *pass, unsigned pass_len)
{
	static bool tested, crypto_ok;
	if (!api_ready || ssid_len == 0 || ssid_len > 32) {
		return api_ready ? EINVAL : ENXIO;
	}
	if (!tested) {
		tested = true;
		crypto_ok = w_crypto_selftest();
	}
	if (pass_len != 0 && (!crypto_ok || pass_len < 8 || pass_len > 63)) {
		return crypto_ok ? EINVAL : EIO;
	}
	uint8_t pmk[32] = {};
	if (pass_len != 0) {
		w_pbkdf2_pmk(pass, pass_len, ssid, ssid_len, pmk);
	}
	int r = w_join_with(ssid, ssid_len, bssid, pass_len != 0 ? pmk : NULL);
	bzero(pmk, sizeof(pmk));
	return r;
}

/* the same with the PMK already made of the passphrase (what wifid hands over) */
int
topaz_wlan_join_pmk(const uint8_t *ssid, unsigned ssid_len, const uint8_t *bssid, const uint8_t pmk[32])
{
	if (!api_ready || ssid_len == 0 || ssid_len > 32) {
		return api_ready ? EINVAL : ENXIO;
	}
	return w_join_with(ssid, ssid_len, bssid, pmk);
}

static int
w_join_with(const uint8_t *ssid, unsigned ssid_len, const uint8_t *bssid, const uint8_t *pmk)
{
	while (s_state != S_IDLE && s_state != S_UP && s_join_asked) {
		IOSleep(50);
	}
	memcpy(s_ssid, ssid, ssid_len);
	s_ssid_len = (uint8_t)ssid_len;
	if (bssid != NULL) {
		memcpy(s_bssid, bssid, 6);
	} else {
		bzero(s_bssid, 6);
	}
	if (pmk != NULL) {
		memcpy(s_pmk, pmk, 32);
	} else {
		bzero(s_pmk, 32);
	}
	uint32_t before = s_join_done;
	s_join_asked = true;
	for (int i = 0; i < 300 && s_join_done == before; i++) {   /* 30 s */
		IOSleep(100);
	}
	return s_join_done != before ? s_result : ETIMEDOUT;
}

int
topaz_wlan_leave(void)
{
	s_leave_asked = true;
	return 0;
}

bool
topaz_wlan_joined(void)
{
	return s_state == S_UP;
}

/* frames from the network stack wait here for the modem thread */
#define ETXQ 32
static struct { uint16_t len; uint8_t d[1514]; } etxq[ETXQ];
static volatile uint32_t etxq_head, etxq_tail;
static IOSimpleLock *etxq_lock;

bool
topaz_wlan_transmit(const uint8_t *frame, size_t length)
{
	if (s_state != S_UP || length < 14 || length > 1514 || etxq_lock == NULL) {
		return false;
	}
	bool ok = false;
	IOSimpleLockLock(etxq_lock);
	if (etxq_tail - etxq_head < ETXQ) {
		uint32_t i = etxq_tail % ETXQ;
		etxq[i].len = (uint16_t)length;
		memcpy(etxq[i].d, frame, length);
		etxq_tail++;
		ok = true;
	}
	IOSimpleLockUnlock(etxq_lock);
	if (!ok) {
		tx_drop++;
	}
	return ok;
}

static void
w_tx_pump(void)
{
	static uint8_t one[1514];
	for (int n = 0; n < 16 && etxq_head != etxq_tail; n++) {
		uint16_t len;
		IOSimpleLockLock(etxq_lock);
		uint32_t i = etxq_head % ETXQ;
		len = etxq[i].len;
		memcpy(one, etxq[i].d, len);
		IOSimpleLockUnlock(etxq_lock);
		if (s_state != S_UP || !w_tx_ether(one, len, false)) {
			if (s_state != S_UP) {
				IOSimpleLockLock(etxq_lock);
				etxq_head = etxq_tail;          /* not joined: drop them */
				IOSimpleLockUnlock(etxq_lock);
			}
			break;
		}
		IOSimpleLockLock(etxq_lock);
		etxq_head++;
		IOSimpleLockUnlock(etxq_lock);
	}
}

static void
wmi_rx(const uint8_t *d, uint32_t len)
{
	uint32_t id;
	if (len < 4) {
		return;
	}
	memcpy(&id, d, 4);
	id &= 0xFFFFFF;
	switch (id) {
	case WMI_EV_SERVICE_AVAILABLE: {
		uint16_t tag;
		memcpy(&tag, d + 6, 2);
		if (len >= 12 + sizeof(wmi_svc_ext) && tag == TAG_SERVICE_AVAILABLE) {
			memcpy(wmi_svc_ext, d + 12, sizeof(wmi_svc_ext));
		}
		break;
	}
	case WMI_EV_SERVICE_READY:
		LOG("t=%u ms: wmi: SERVICE_READY, %u bytes", g_ms, len);
		wmi_svc_ready(d + 4, len - 4);
		if (!wmi_init_sent) {
			wmi_send_init();
		}
		break;
	case WMI_EV_READY:
		for (uint32_t o = 4; o + 4 <= len;) {
			uint16_t l, tag;
			memcpy(&l, d + o, 2);
			memcpy(&tag, d + o + 2, 2);
			if (o + 4 + l > len) {
				break;
			}
			if (tag == TAG_READY_EVENT && l >= 24 + 8 + 4) {
				memcpy(wmi_mac, d + o + 4 + 24, 6);
				wmi_ready = true;
			}
			o += 4 + l;
		}
		LOG("t=%u ms: *** WMI READY (fw MAC %02x:%02x:%02x:%02x:%02x:%02x) ***", g_ms, wmi_mac[0], wmi_mac[1],
		    wmi_mac[2], wmi_mac[3], wmi_mac[4], wmi_mac[5]);
		if (wmi_ready) {
			htt_start();                        /* the host rx ring first, then the scan */
		}
		break;
	case WMI_EV_SCAN:
		scan_event(d + 4, len - 4);
		break;
	case WMI_EV_MGMT_RX:
		scan_mgmt_rx(d + 4, len - 4);
		break;
	case 0x5001:                                /* VDEV_START_RESP */
	case 0x6001:                                /* PEER_STA_KICKOUT */
		w_join_event(id, d + 4, len - 4);
		break;
	default:
		break;
	}
}

/*
 * sapphire: join the network named by boot-args wifi_ssid= / wifi_pass= (no spaces) once the
 * radio is up, for testing until wifid can ask. topaz_wlan_join waits for the modem thread, so
 * it runs on a thread of its own. The passphrase is never logged and is wiped after the join.
 */
static void
w_bootarg_join_thread(void *arg __unused, wait_result_t wr __unused)
{
	char ssid[33] = "", pass[64] = "";

	if (PE_parse_boot_arg_str("wifi_ssid", ssid, sizeof(ssid)) && ssid[0] != 0) {
		(void)PE_parse_boot_arg_str("wifi_pass", pass, sizeof(pass));
		size_t plen = strnlen(pass, sizeof(pass));
		LOG("t=%u ms: boot-arg join: '%s' (%s)", g_ms, ssid, plen != 0 ? "WPA2-PSK" : "open");
		int r = topaz_wlan_join((const uint8_t *)ssid, (unsigned)strnlen(ssid, 32), NULL, (const uint8_t *)pass,
		    (unsigned)plen);
		bzero(pass, sizeof(pass));
		LOG("t=%u ms: boot-arg join: '%s' -> %d%s", g_ms, ssid, r, r == 0 ? " (joined)" : "");
	}
	thread_terminate(current_thread());
}

static void
w_bootarg_join_start(void)
{
	char probe[2];
	thread_t t;

	if (!PE_parse_boot_arg_str("wifi_ssid", probe, sizeof(probe))) {
		return;
	}
	if (kernel_thread_start(w_bootarg_join_thread, NULL, &t) == KERN_SUCCESS) {
		thread_deallocate(t);
	}
}

/* from the watch loop */
static void
radio_poll(unsigned ms)
{
	(void)ms;
	if (w_cap_retry_ms != 0 && g_ms >= w_cap_retry_ms) {
		w_cap_retry_ms = 0;
		w_simple(WLFW_CAP, "cap sent again");
	}
	if (!ce_up) {
		return;
	}
	WHERE("wlan ce poll");
	(void)ce_poll();
	WHERE("wlan htt poll");
	htt_poll();
	WHERE("wlan join/tx");
	if (scan_started && !api_ready) {
		if (etxq_lock == NULL) {
			etxq_lock = IOSimpleLockAlloc();
		}
		api_ready = true;
		LOG("t=%u ms: wlan: radio ready for the rest of the kernel", g_ms);
		if (topaz_wlan_ready_hook != NULL) {
			topaz_wlan_ready_hook();
		}
		w_bootarg_join_start();
	}
	if (api_ready) {
		w_join_poll();
		w_tx_pump();
	}
	if (scan_started && !scanning && api_scan_asked && (s_state == S_IDLE || s_state == S_UP)) {
		api_scan_asked = false;
		next_scan_ms = 0;
		scanning = scan_send_start();
		if (!scanning) {
			api_scans_done++;
		}
	}
	if (scan_started && !scanning && next_scan_ms != 0 && g_ms >= next_scan_ms) {
		next_scan_ms = 0;
		scanning = scan_send_start();
		LOG("t=%u ms: scan: passive scan %u %s", g_ms, scans + 1, scanning ? "requested" : "FAILED");
	}
}

/* ---------------------------------------------------------------- IPA v4.2 (ipa=1) */

/*
 * sapphire 0225: the IP accelerator. The modem hands its data (the LTE default bearer, IMS) to the
 * AP through the IPA. With no IPA driver here the modem was told "AP IPA power off" and its data
 * and IMS tasks hung until its watchdog killed it ("Task starvation: dcc", "... ims",
 * "ipa_dl_opt_lte.c: IPA Assert"). A port of Linux drivers/net/ipa for IPA v4.2 (SC7180's data and
 * registers; SM6225's downstream tables give the same endpoints and channels), polled from the
 * modem thread, no interrupts:
 *   RPM "ipa" clock -> registers (base 0x5800000 + 0x40000, GSI at +0x4000) -> hardware config,
 *   SRAM canaries, resource groups -> GSI firmware ipa_fws (PAS 15, carried by the device tree:
 *   tools/mkadt.py --ipa-mdt/--ipa-elf) -> GSI event rings and channels (AP command TX 1, LAN RX 2,
 *   modem TX 0, modem RX 3; the modem's 0-3 allocated on its behalf, a v4.2 quirk) -> endpoints ->
 *   SRAM regions, filter and route tables by immediate commands -> default route to LAN RX ->
 *   QMI handshake (INIT_DRIVER to the modem's 0x31:2; our server 0x31:1 answers
 *   DRIVER_INIT_COMPLETE and INDICATION_REGISTER, then sends INIT_COMPLETE) -> modem TX/RX started.
 * Received QMAP frames are counted (kern.sweet_modem_stats) and their buffers given back.
 * Rings and buffers come from the modem pool (below 4 GB, uncached: no cache maintenance).
 */
#define IPA_REG_PA          0x05840000ull
#define IPA_REG_SIZE        0x10000
#define IPA_GSI_PA          0x05804000ull
#define IPA_GSI_SIZE        0x28000
#define IPA_SRAM_OFF        0x7000              /* SW_AREA_RAM_DIRECT_ACCESS */
#define IPA_PAS_ID          15
#define IPA_FW_PA           0x55a00000ull       /* ipa_fw_region */
#define IPA_FW_SIZE         0x10000
#define IPA_SID             0x140               /* ipa_smmu_ap */
#define IPA_SMEM_ITEM       497
#define IPA_SMEM_SIZE       0x2000
#define IPA_RX_BUF          8192
#define IPA_RX_NBUF         32

/* IPA registers (Linux reg/ipa_reg-v4.2.c), endpoint registers every 0x70 */
#define IPA_COMP_CFG        0x003c
#define IPA_CLKON_CFG       0x0044
#define IPA_ROUTE           0x0048
#define IPA_SHARED_MEM_SIZE 0x0054
#define IPA_QSB_MAX_WRITES  0x0074
#define IPA_QSB_MAX_READS   0x0078
#define IPA_AGGR_ACTIVE     0x00b4
#define IPA_HASH_EN         0x0148
#define IPA_BCR             0x01d0
#define IPA_PKT_PROC_CNTXT  0x01e8
#define IPA_COUNTER_CFG     0x01f0
#define IPA_TX_CFG          0x01fc
#define IPA_FLAVOR_0        0x0210
#define IPA_IDLE_CFG        0x0240
#define IPA_SRC_RSRC(t)     (0x0400 + 0x20 * (t))
#define IPA_DST_RSRC(t)     (0x0500 + 0x20 * (t))
#define IPA_EP_CFG          0x0808
#define IPA_EP_NAT          0x080c
#define IPA_EP_HDR          0x0810
#define IPA_EP_HDR_EXT      0x0814
#define IPA_EP_MDMASK       0x0818
#define IPA_EP_MODE         0x0820
#define IPA_EP_AGGR         0x0824
#define IPA_EP_HOL_EN       0x082c
#define IPA_EP_DEAGGR       0x0834
#define IPA_EP_RSRC         0x0838
#define IPA_EP_SEQ          0x083c
#define IPA_EP_STATUS       0x0840
#define IPA_EP(reg, ep)     ((reg) + 0x70 * (ep))
#define IPA_IRQ_EN          0x300c

/* GSI registers, EE 0 (Linux reg/gsi_reg-v4.0.c) */
#define GSI_INTER_EE_CH_MSK 0xc020
#define GSI_INTER_EE_EV_MSK 0xc024
#define GSI_CH(n, ch)       (0x1c000 + 0x80 * (ch) + 4 * (n))  /* CNTXT_0..3 */
#define GSI_CH_QOS(ch)      (0x1c05c + 0x80 * (ch))
#define GSI_CH_SCR(n, ch)   (0x1c060 + 0x80 * (ch) + 4 * (n))
#define GSI_EV(n, ev)       (0x1d000 + 0x80 * (ev) + 4 * (n))  /* CNTXT_0..13 */
#define GSI_CH_DB(ch)       (0x1e000 + 8 * (ch))
#define GSI_EV_DB(ev)       (0x1e100 + 8 * (ev))
#define GSI_STATUS          0x1f000
#define GSI_CH_CMD          0x1f008
#define GSI_EV_CMD          0x1f010
#define GSI_GENERIC_CMD     0x1f018
#define GSI_HW_PARAM_2      0x1f040
#define GSI_TYPE_IRQ_MSK    0x1f088
#define GSI_SRC_CH_IRQ_MSK  0x1f098
#define GSI_SRC_EV_IRQ_MSK  0x1f09c
#define GSI_IEOB_IRQ_MSK    0x1f0b8
#define GSI_GLOB_IRQ_STTS   0x1f100
#define GSI_GLOB_IRQ_EN     0x1f108
#define GSI_GLOB_IRQ_CLR    0x1f110
#define GSI_GSI_IRQ_EN      0x1f120
#define GSI_INTSET          0x1f180
#define GSI_ERROR_LOG       0x1f200
#define GSI_ERROR_LOG_CLR   0x1f210
#define GSI_SCRATCH_0       0x1f400

/* IPA-local memory regions (Linux data/ipa_data-v4.2.c), offsets from the memory base */
#define IPA_MEM_V4_FILTER   0x0290
#define IPA_MEM_V6_FILTER   0x0318
#define IPA_MEM_V4_ROUTE    0x03a0
#define IPA_MEM_V6_ROUTE    0x0428
#define IPA_MEM_TABLE_SIZE  0x0078
#define IPA_MEM_MODEM_HDR   0x04a8              /* 0x140 */
#define IPA_MEM_MODEM_PROC  0x05f0              /* 0x200 */
#define IPA_MEM_AP_PROC     0x07f0              /* 0x200 */
#define IPA_MEM_QUOTA       0x0a50              /* 0x60 */
#define IPA_MEM_MODEM       0x0bf0              /* 0x140c */
#define IPA_MODEM_ROUTES    8
#define IPA_FILTERED        ((1u << 1) | (1u << 4)) /* endpoints 1 (AP->modem) and 4 (modem->AP) filter */

/* immediate commands (Linux ipa_cmd.h) */
#define IPA_CMD_V4_FILTER_INIT  0x3
#define IPA_CMD_V6_FILTER_INIT  0x4
#define IPA_CMD_V4_ROUTE_INIT   0x7
#define IPA_CMD_V6_ROUTE_INIT   0x8
#define IPA_CMD_HDR_INIT_LOCAL  0x9
#define IPA_CMD_DMA_SHARED_MEM  0x13

#define PORT_IPAC           0x4005              /* our QMI client towards the modem's IPA service */
#define PORT_IPAS           0x4006              /* our IPA QMI service (0x31, version 1, instance 1) */

struct ipa_chan {
	const char *name;
	uint8_t ch, ev, ep;
	bool tx, cmd;
	uint16_t tres, tlv;
	uint8_t *ring, *evr, *buf;
	uint64_t ring_pa, evr_pa, buf_pa;
	uint32_t widx, eidx;                    /* free running: next TRE to fill, next event to read */
	uint8_t slot_buf[512];                  /* RX: the buffer a TRE slot holds */
	uint64_t frames, bytes, pkts, cmds;
	uint32_t last_code;
};

static struct ipa_chan ipa_ch[4] = {
	{ "command TX", 1, 0, 6, true,  true,  256, 20 },
	{ "LAN RX",     2, 1, 8, false, false, 256, 6 },
	{ "modem TX",   0, 2, 1, true,  false, 512, 8 },
	{ "modem RX",   3, 3, 9, false, false, 256, 6 },
};
#define IPA_CMD             (&ipa_ch[0])
#define IPA_LAN             (&ipa_ch[1])
#define IPA_MTX             (&ipa_ch[2])
#define IPA_MRX             (&ipa_ch[3])

static int ipa_on;
static bool ipa_up, ipa_running;
static volatile uint8_t *ipa_r, *gsi_r;
static uint32_t ipa_mo, ipa_msize;              /* IPA-local memory offset and size */
static uint8_t *ipa_pay, *ipa_zero;             /* command payloads (32 bytes a TRE slot), zeroes */
static uint64_t ipa_pay_pa, ipa_zero_pa, ipa_table_pa;
static const char *ipa_step = "off";
static uint32_t ipa_m_node, ipa_m_port, ipa_c_node, ipa_c_port;
static bool ipa_modem_ready, ipa_uc_ready, ipa_ind_req, ipa_ind_sent;
static uint16_t ipa_txn = 1;
static uint32_t ipa_gsi_errors, ipa_last_error, ipa_mux_seen[8];

static inline uint32_t ir(uint32_t o) { return *(volatile uint32_t *)(ipa_r + o); }
static inline void iw(uint32_t o, uint32_t v) { *(volatile uint32_t *)(ipa_r + o) = v; mb(); }
static inline uint32_t gr(uint32_t o) { return *(volatile uint32_t *)(gsi_r + o); }
static inline void gw(uint32_t o, uint32_t v) { *(volatile uint32_t *)(gsi_r + o) = v; mb(); }

extern "C" int topaz_rpm_vote(uint32_t set, uint32_t type, uint32_t id, uint32_t key, uint32_t value);

/* pool memory aligned to its size (GSI rings are 2^n bytes, aligned on their size) */
static void *
ipa_pool(uint64_t size, uint64_t align, uint64_t *pa)
{
	uint64_t pad = (align - ((MODEM_POOL_BASE + pool_used) & (align - 1))) & (align - 1);
	uint64_t dummy;
	if (pad != 0 && pool_alloc(pad, &dummy) == NULL) {
		return NULL;
	}
	return pool_alloc(size, pa);
}

static bool
ipa_chosen(const char *prop, const void **blob, unsigned *size)
{
	DTEntry chosen;
	return SecureDTLookupEntry(NULL, "/chosen", &chosen) == kSuccess &&
	       SecureDTGetProperty(chosen, prop, blob, size) == kSuccess;
}

/* ---- GSI */

static unsigned
gsi_state(uint32_t reg)
{
	return (gr(reg) >> 20) & 0xf;
}

static bool
gsi_cmd(uint32_t cmd_reg, uint32_t val, uint32_t state_reg, unsigned want, const char *what, unsigned id)
{
	gw(cmd_reg, val);
	for (unsigned i = 0; i < 500; i++) {
		if (gsi_state(state_reg) == want) {
			return true;
		}
		IODelay(100);
	}
	LOG("ipa: GSI %s %u: state %u, wanted %u (error log %08x)", what, id, gsi_state(state_reg), want, gr(GSI_ERROR_LOG));
	return false;
}

/* a GENERIC command on a modem channel; the result comes in SCRATCH_0 with global interrupt GP_INT1 */
static int
gsi_generic(unsigned opcode, unsigned ch)
{
	gw(GSI_GLOB_IRQ_EN, 3);                 /* ERROR_INT | GP_INT1 */
	gw(GSI_GLOB_IRQ_CLR, 2);
	gw(GSI_SCRATCH_0, gr(GSI_SCRATCH_0) & ~(7u << 5));
	gw(GSI_GENERIC_CMD, opcode | (ch << 5) | (1u << 10));  /* EE 1: the modem */
	unsigned i;
	for (i = 0; i < 500 && !(gr(GSI_GLOB_IRQ_STTS) & 2); i++) {
		IODelay(100);
	}
	unsigned result = (gr(GSI_SCRATCH_0) >> 5) & 7;
	gw(GSI_GLOB_IRQ_CLR, 2);
	gw(GSI_GLOB_IRQ_EN, 1);
	if (i == 500) {
		LOG("ipa: GSI generic %u on modem channel %u: no answer (result %u)", opcode, ch, result);
		return -1;
	}
	return (result == 1 || result == 2) ? 0 : (int)result;   /* SUCCESS, INCORRECT_CHANNEL_STATE */
}

static void
gsi_tre(struct ipa_chan *c, uint64_t pa, uint16_t len_op, uint32_t flags)
{
	uint8_t *t = c->ring + (c->widx % c->tres) * 16;
	uint16_t z = 0;
	memcpy(t, &pa, 8);
	memcpy(t + 8, &len_op, 2);
	memcpy(t + 10, &z, 2);
	memcpy(t + 12, &flags, 4);
	c->widx++;
}

static void
gsi_doorbell(struct ipa_chan *c)
{
	mb();
	gw(GSI_CH_DB(c->ch), (uint32_t)(c->ring_pa + (c->widx % c->tres) * 16));
}

/* events filled since last time: fn for each, then the event ring doorbell */
static unsigned
gsi_events(struct ipa_chan *c, void (*fn)(struct ipa_chan *, const uint8_t *))
{
	uint32_t wp = gr(GSI_EV(4, c->ev));
	uint32_t idx = (wp - (uint32_t)c->evr_pa) / 16;
	unsigned n = 0;

	if (idx >= c->tres) {
		return 0;
	}
	while (c->eidx % c->tres != idx && n < c->tres) {
		const uint8_t *e = c->evr + (c->eidx % c->tres) * 16;
		c->last_code = e[11];
		fn(c, e);
		c->eidx++;
		n++;
	}
	if (n != 0) {
		mb();
		gw(GSI_EV_DB(c->ev), (uint32_t)(c->evr_pa + ((c->eidx - 1) % c->tres) * 16));
	}
	return n;
}

static void ipa_ev_count(struct ipa_chan *c, const uint8_t *e) { (void)e; c->cmds++; }

static bool
gsi_setup_chan(struct ipa_chan *c)
{
	uint32_t rsize = c->tres * 16u;

	c->ring = (uint8_t *)ipa_pool(rsize, rsize, &c->ring_pa);
	c->evr = (uint8_t *)ipa_pool(rsize, rsize, &c->evr_pa);
	if (c->ring == NULL || c->evr == NULL) {
		LOG("ipa: no pool memory for the %s rings", c->name);
		return false;
	}
	if (gsi_state(GSI_EV(0, c->ev)) != 0 || gsi_state(GSI_CH(0, c->ch)) != 0) {
		LOG("ipa: %s: event ring %u state %u, channel %u state %u before allocation", c->name, c->ev,
		    gsi_state(GSI_EV(0, c->ev)), c->ch, gsi_state(GSI_CH(0, c->ch)));
	}
	if (!gsi_cmd(GSI_EV_CMD, c->ev, GSI_EV(0, c->ev), 1, "event ring allocate", c->ev)) {
		return false;
	}
	gw(GSI_EV(0, c->ev), 2u | (1u << 16) | (16u << 24));          /* GPI, interrupt type IRQ, 16-byte elements */
	gw(GSI_EV(1, c->ev), rsize);
	gw(GSI_EV(2, c->ev), (uint32_t)c->evr_pa);
	gw(GSI_EV(3, c->ev), (uint32_t)(c->evr_pa >> 32));
	gw(GSI_EV(8, c->ev), 32u | (1u << 16));                         /* moderation: 1 ms, from the channel */
	for (unsigned n = 9; n <= 13; n++) {
		gw(GSI_EV(n, c->ev), 0);
	}
	gw(GSI_EV_DB(c->ev), (uint32_t)(c->evr_pa + (c->tres - 1) * 16u));
	if (!gsi_cmd(GSI_CH_CMD, c->ch, GSI_CH(0, c->ch), 1, "channel allocate", c->ch)) {
		return false;
	}
	gw(GSI_CH(0, c->ch), 2u | (c->tx ? (1u << 3) : 0) | ((uint32_t)c->ev << 14) | (16u << 24));
	gw(GSI_CH(1, c->ch), rsize);
	gw(GSI_CH(2, c->ch), (uint32_t)c->ring_pa);
	gw(GSI_CH(3, c->ch), (uint32_t)(c->ring_pa >> 32));
	gw(GSI_CH_QOS(c->ch), c->cmd ? 0xfu : (1u << 10));             /* command: low WRR priority; others: escape buffer only */
	gw(GSI_CH_SCR(0, c->ch), 0);
	gw(GSI_CH_SCR(1, c->ch), 0);
	gw(GSI_CH_SCR(2, c->ch), (uint32_t)(c->tlv * 16u) << 16);      /* max outstanding TRE bytes */
	gw(GSI_CH_SCR(3, c->ch), (32u << 16) | (gr(GSI_CH_SCR(3, c->ch)) & 0xffff)); /* doorbell threshold: 2 TREs */
	return true;
}

static bool
gsi_start(struct ipa_chan *c)
{
	return gsi_cmd(GSI_CH_CMD, c->ch | (1u << 24), GSI_CH(0, c->ch), 2, "channel start", c->ch);
}

/* ---- endpoints (Linux ipa_endpoint_program() for each of the AP's four) */

static void
ipa_ep_program(void)
{
	/* command TX 6: DMA mode into LAN RX 8, sequencer DMA */
	iw(IPA_EP(IPA_EP_CFG, 6), 0);
	iw(IPA_EP(IPA_EP_NAT, 6), 0);
	iw(IPA_EP(IPA_EP_HDR, 6), 0);
	iw(IPA_EP(IPA_EP_HDR_EXT, 6), 0);
	iw(IPA_EP(IPA_EP_MODE, 6), 3u | (8u << 4));
	iw(IPA_EP(IPA_EP_AGGR, 6), 0);
	iw(IPA_EP(IPA_EP_DEAGGR, 6), 0);
	iw(IPA_EP(IPA_EP_RSRC, 6), 0);
	iw(IPA_EP(IPA_EP_SEQ, 6), 0x00);
	iw(IPA_EP(IPA_EP_STATUS, 6), 0);
	/* LAN RX 8: generic aggregation (6 KB, 500 us), 4-byte pad alignment, status */
	iw(IPA_EP(IPA_EP_CFG, 8), 0);
	iw(IPA_EP(IPA_EP_HDR, 8), 0);
	iw(IPA_EP(IPA_EP_HDR_EXT, 8), 2u << 10);
	iw(IPA_EP(IPA_EP_MDMASK, 8), 0);
	iw(IPA_EP(IPA_EP_AGGR, 8), 1u | (4u << 2) | (6u << 5) | (1u << 10));
	iw(IPA_EP(IPA_EP_HOL_EN, 8), 0);
	iw(IPA_EP(IPA_EP_RSRC, 8), 0);
	iw(IPA_EP(IPA_EP_STATUS, 8), 1);
	/* modem TX 1: QMAP header only (4 bytes), no uplink checksum offload, sequencer 1 pass skip uC
	 * with DMA parser replication, status to modem->AP RX 10. (sapphire 0231: with Linux's UL
	 * checksum offload and its 8-byte header the IPA left our header on the packet, the modem
	 * counted it sent and nothing ever came back; QMAP alone, as rmnet without the checksum
	 * flag, and the network answers.) */
	iw(IPA_EP(IPA_EP_CFG, 1), 0);
	iw(IPA_EP(IPA_EP_NAT, 1), 0);
	iw(IPA_EP(IPA_EP_HDR, 1), 4u | (1u << 6));
	iw(IPA_EP(IPA_EP_HDR_EXT, 1), 1);
	iw(IPA_EP(IPA_EP_MODE, 1), 0);
	iw(IPA_EP(IPA_EP_AGGR, 1), 0);
	iw(IPA_EP(IPA_EP_DEAGGR, 1), 0);
	iw(IPA_EP(IPA_EP_RSRC, 1), 0);
	iw(IPA_EP(IPA_EP_SEQ, 1), 0x06u | (0x08u << 8));
	iw(IPA_EP(IPA_EP_STATUS, 1), 1u | (10u << 1));
	/* modem RX 9: DL checksum offload, QMAP header filled by the IPA (mux id at 1, length at 2),
	 * big endian, length includes padding, metadata mask 0xff (big endian), generic aggregation
	 * closed at end of frame */
	iw(IPA_EP(IPA_EP_CFG, 9), 2u << 1);
	iw(IPA_EP(IPA_EP_HDR, 9), 4u | (1u << 6) | (1u << 7) | (1u << 19) | (2u << 20));
	iw(IPA_EP(IPA_EP_HDR_EXT, 9), 1u | (1u << 1) | (1u << 3));
	iw(IPA_EP(IPA_EP_MDMASK, 9), 0xff000000u);
	iw(IPA_EP(IPA_EP_AGGR, 9), 1u | (4u << 2) | (6u << 5) | (1u << 10) | (1u << 21));
	iw(IPA_EP(IPA_EP_HOL_EN, 9), 0);
	iw(IPA_EP(IPA_EP_RSRC, 9), 0);
	iw(IPA_EP(IPA_EP_STATUS, 9), 0);
}

/* ---- immediate commands */

static uint8_t *
ipa_cmd_add(uint16_t opcode, bool last)
{
	struct ipa_chan *c = IPA_CMD;
	uint32_t slot = c->widx % c->tres;
	uint8_t *p = ipa_pay + 32 * slot;
	bzero(p, 32);
	gsi_tre(c, ipa_pay_pa + 32 * slot, opcode, (3u << 16) | (last ? (1u << 9) : 1u));
	return p;
}

static void
ipa_cmd_dma(uint32_t off, uint16_t size, bool last)
{
	uint8_t *p = ipa_cmd_add(IPA_CMD_DMA_SHARED_MEM, last);
	uint16_t local = (uint16_t)(ipa_mo + off);
	memcpy(p + 2, &size, 2);
	memcpy(p + 4, &local, 2);
	memcpy(p + 8, &ipa_zero_pa, 8);         /* flags 0: toward the IPA */
}

static void
ipa_cmd_table(uint16_t opcode, uint32_t off, uint16_t size, uint64_t addr, bool last)
{
	uint8_t *p = ipa_cmd_add(opcode, last);
	uint64_t flags = ((uint64_t)(ipa_mo + off) << 40) | ((uint64_t)size << 28);
	memcpy(p + 8, &flags, 8);
	memcpy(p + 16, &addr, 8);
}

/* send what was added, wait for its completion event */
static bool
ipa_cmd_commit(const char *what)
{
	struct ipa_chan *c = IPA_CMD;
	uint64_t before = c->cmds;
	gsi_doorbell(c);
	for (unsigned i = 0; i < 1000; i++) {
		gsi_events(c, ipa_ev_count);
		if (c->cmds != before) {
			LOG("ipa: %s: done (event code %u)", what, c->last_code);
			return true;
		}
		IODelay(100);
	}
	LOG("ipa: %s: no completion (channel state %u, error log %08x, glob %08x)", what,
	    gsi_state(GSI_CH(0, c->ch)), gr(GSI_ERROR_LOG), gr(GSI_GLOB_IRQ_STTS));
	return false;
}

/* ---- RX buffers */

static void
ipa_rx_post(struct ipa_chan *c, unsigned b)
{
	c->slot_buf[c->widx % c->tres] = (uint8_t)b;
	gsi_tre(c, c->buf_pa + (uint64_t)b * IPA_RX_BUF, IPA_RX_BUF, (2u << 16) | (1u << 9));
}

static bool
ipa_rx_init(struct ipa_chan *c)
{
	c->buf = (uint8_t *)ipa_pool((uint64_t)IPA_RX_NBUF * IPA_RX_BUF, 0x1000, &c->buf_pa);
	if (c->buf == NULL) {
		LOG("ipa: no pool memory for %s buffers", c->name);
		return false;
	}
	for (unsigned b = 0; b < IPA_RX_NBUF; b++) {
		ipa_rx_post(c, b);
	}
	gsi_doorbell(c);
	return true;
}

static void ipa_rx_keep(uint8_t mux, const uint8_t *p, uint16_t len);
static void ipa_raw_keep(uint8_t chan, const uint8_t *p, uint16_t len);
static void ipa_qmap_cmd(const uint8_t *frame, uint16_t len);
extern "C" void sweet_pdp_create(void);
extern "C" void sweet_pdp_input(const uint8_t *pkt, uint32_t len);

/* QMAP frames from the modem: [flags/pad, mux id, length (big endian)] + packet, back to back */
static void
ipa_rx_event(struct ipa_chan *c, const uint8_t *e)
{
	uint64_t tre;
	uint16_t len;
	memcpy(&tre, e, 8);
	memcpy(&len, e + 8, 2);
	uint32_t slot = (uint32_t)((tre - c->ring_pa) / 16) % c->tres;
	unsigned b = c->slot_buf[slot];
	const uint8_t *d = c->buf + (uint64_t)b * IPA_RX_BUF;

	c->frames++;
	c->bytes += len;
	ipa_raw_keep(c == IPA_MRX ? 3 : 2, d, len);
	if (c == IPA_MRX) {
		for (uint32_t off = 0; off + 4 <= len;) {
			uint8_t pad = d[off] & 0x3f, mux = d[off + 1];
			bool cmd = (d[off] & 0x80) != 0;
			uint16_t plen = (uint16_t)((d[off + 2] << 8) | d[off + 3]);
			if (plen == 0 || off + 4 + plen > len) {
				break;
			}
			c->pkts++;
			ipa_mux_seen[mux & 7]++;
			if (!cmd) {
				ipa_rx_keep(mux, d + off + 4, (uint16_t)(plen - pad));
				if (mux == 1) {
					sweet_pdp_input(d + off + 4, (uint32_t)(plen - pad));
				}
			} else {
				ipa_qmap_cmd(d + off, (uint16_t)(4 + plen));
			}
			if (c->pkts <= 12) {
				const uint8_t *ip = d + off + 4;
				LOG("ipa: modem RX packet %llu: mux %u%s, %u bytes (pad %u), first bytes %02x %02x %02x %02x .. proto %u",
				    c->pkts, mux, cmd ? " (QMAP command)" : "", plen, pad, ip[0], ip[1], ip[2], ip[3],
				    plen > 9 ? ip[9] : 0);
			}
			off += 4 + plen;
		}
	} else if (c->frames <= 8) {
		LOG("ipa: %s frame %llu: %u bytes, status opcode %02x endpoint %u", c->name, c->frames, len, d[0],
		    len > 3 ? d[3] & 0x1f : 0);
	}
	ipa_rx_post(c, b);
}

static void
ipa_tx_event(struct ipa_chan *c, const uint8_t *e)
{
	uint16_t len;
	memcpy(&len, e + 8, 2);
	c->cmds++;
	if (c->cmds <= 8) {
		LOG("t=%u ms: ipa: %s completion %llu: code %u, %u bytes, type %u, channel %u", g_ms, c->name, c->cmds, e[11], len,
		    e[14], e[15]);
	}
}

/*
 * sapphire 0226: one IP packet at a time towards the modem (kern.sweet_ipa_tx, bsd/dev/sweet_qmi_dev.c)
 * and the last packets from it (kern.sweet_ipa_rx): enough for qmitool's ping over a data call
 * started with qmitool datacall. The packet goes out on modem TX behind a QMAP header (mux id) and
 * an empty uplink checksum header (endpoint 1's 8-byte header, no checksum offload asked).
 */
#define IPA_TX_SLOTS        64              /* DMA buffers on modem TX, one a TRE in flight */
#define IPA_TXQ             64              /* packets waiting for the modem thread (0232: pdp_ip0's output) */
#define IPA_RXLOG           8
static struct { uint16_t len; uint8_t mux; uint8_t data[1600]; } ipa_txq[IPA_TXQ];
static uint32_t ipa_tx_hdr = 4;                 /* 4 = QMAP only (0231 default), 8 = QMAP + UL checksum header (peek "h8") */
static volatile uint32_t ipa_txq_head, ipa_txq_tail;    /* written by the senders (under the lock) / the modem thread */
static IOSimpleLock *ipa_txq_lock;
static uint8_t *ipa_txbuf;
static uint64_t ipa_txbuf_pa, ipa_tx_sent, ipa_tx_posted, ipa_tx_full;
static struct { uint32_t seq; uint8_t mux, pad; uint16_t len; uint8_t data[248]; } ipa_rxlog[IPA_RXLOG];
static uint32_t ipa_rxlog_seq;

extern "C" int
topaz_ipa_tx(const uint8_t *pkt, uint32_t len, uint8_t mux)
{
	if (!ipa_running || ipa_txbuf == NULL || ipa_txq_lock == NULL) {
		return ENXIO;
	}
	if (len < 20 || len > sizeof(ipa_txq[0].data)) {
		return EINVAL;
	}
	IOSimpleLockLock(ipa_txq_lock);
	uint32_t h = ipa_txq_head;
	if (h - ipa_txq_tail >= IPA_TXQ) {
		IOSimpleLockUnlock(ipa_txq_lock);
		ipa_tx_full++;
		return EBUSY;
	}
	memcpy(ipa_txq[h % IPA_TXQ].data, pkt, len);
	ipa_txq[h % IPA_TXQ].len = (uint16_t)len;
	ipa_txq[h % IPA_TXQ].mux = mux;
	mb();
	ipa_txq_head = h + 1;
	IOSimpleLockUnlock(ipa_txq_lock);
	return 0;
}

/* sapphire 0232: pdp_ip0's output (bsd/dev/sweet_pdp.c) */
extern "C" int
topaz_ipa_send(const uint8_t *pkt, uint32_t len, uint8_t mux)
{
	return topaz_ipa_tx(pkt, len, mux);
}

extern "C" int
topaz_ipa_rx(uint8_t *out, uint32_t max)
{
	uint32_t n = MIN(max, (uint32_t)sizeof(ipa_rxlog));
	memcpy(out, ipa_rxlog, n);
	return (int)n;
}

static void
ipa_rx_keep(uint8_t mux, const uint8_t *p, uint16_t len)
{
	unsigned i = ipa_rxlog_seq % IPA_RXLOG;
	ipa_rxlog[i].len = len;
	ipa_rxlog[i].mux = mux;
	memcpy(ipa_rxlog[i].data, p, MIN((uint32_t)len, (uint32_t)sizeof(ipa_rxlog[i].data)));
	mb();
	ipa_rxlog[i].seq = ++ipa_rxlog_seq;
}

/* kern.sweet_ipa_raw: the first 64 bytes of the last 8 frames received, as they came */
static struct { uint32_t seq; uint8_t chan, pad; uint16_t len; uint8_t data[64]; } ipa_raw[8];
static uint32_t ipa_raw_seq;

static void
ipa_raw_keep(uint8_t chan, const uint8_t *p, uint16_t len)
{
	unsigned i = ipa_raw_seq % 8;
	ipa_raw[i].chan = chan;
	ipa_raw[i].len = len;
	memcpy(ipa_raw[i].data, p, MIN((uint32_t)len, (uint32_t)sizeof(ipa_raw[i].data)));
	mb();
	ipa_raw[i].seq = ++ipa_raw_seq;
}

extern "C" int
topaz_ipa_raw(uint8_t *out, uint32_t max)
{
	uint32_t n = MIN(max, (uint32_t)sizeof(ipa_raw));
	memcpy(out, ipa_raw, n);
	return (int)n;
}

/*
 * A QMAP command from the modem ([cd=1, mux, length] + name, type, reserved, transaction id):
 * a request (type 0) is answered with the same command as an ACK (type 1), as Linux rmnet's
 * rmnet_map_send_ack() does for flow control.
 */
static uint8_t ipa_ackq[64];
static volatile uint32_t ipa_ackq_len;
static uint32_t ipa_qmap_cmds;

static void
ipa_qmap_cmd(const uint8_t *frame, uint16_t len)
{
	if (len < 12) {
		return;
	}
	uint8_t name = frame[4], type = frame[5] & 3;
	uint32_t txn = (uint32_t)frame[8] << 24 | (uint32_t)frame[9] << 16 | (uint32_t)frame[10] << 8 | frame[11];
	if (++ipa_qmap_cmds <= 16) {
		LOG("t=%u ms: ipa: QMAP command %u type %u mux %u transaction %u (%u bytes)%s", g_ms, name, type, frame[1], txn, len,
		    type == 0 ? ": ACK sent" : "");
	}
	if (type == 0 && ipa_ackq_len == 0 && len <= sizeof(ipa_ackq)) {
		memcpy(ipa_ackq, frame, len);
		ipa_ackq[5] = (uint8_t)((ipa_ackq[5] & ~3u) | 1);
		mb();
		ipa_ackq_len = len;
	}
}

static void
ipa_tx_pump(void)
{
	struct ipa_chan *c = IPA_MTX;

	if (ipa_ackq_len != 0 && ipa_tx_posted - c->cmds < IPA_TX_SLOTS) {     /* a QMAP command ACK goes out as it is */
		uint32_t slot = c->widx % IPA_TX_SLOTS;
		uint8_t *b = ipa_txbuf + 2048 * slot;
		uint32_t l = ipa_ackq_len;
		memcpy(b, ipa_ackq, 4);
		bzero(b + 4, 4);                /* the endpoint's 8-byte header: QMAP + checksum */
		memcpy(b + 8, ipa_ackq + 4, l - 4);
		gsi_tre(c, ipa_txbuf_pa + 2048 * slot, (uint16_t)(l + 4), (2u << 16) | (1u << 9));
		gsi_doorbell(c);
		ipa_tx_posted++;
		mb();
		ipa_ackq_len = 0;
	}
	/* the queued packets, as many as there are free buffers (a buffer is free once its TRE completed) */
	bool rang = false;
	while (ipa_txq_tail != ipa_txq_head && ipa_tx_posted - c->cmds < IPA_TX_SLOTS) {
		uint32_t t = ipa_txq_tail;
		mb();
		uint32_t len = ipa_txq[t % IPA_TXQ].len;
		uint8_t mux = ipa_txq[t % IPA_TXQ].mux;
		const uint8_t *pkt = ipa_txq[t % IPA_TXQ].data;
		uint32_t slot = c->widx % IPA_TX_SLOTS;
		uint8_t *b = ipa_txbuf + 2048 * slot;
		b[0] = 0;                               /* pad 0, data */
		b[1] = mux;
		b[2] = (uint8_t)(len >> 8);
		b[3] = (uint8_t)len;
		bzero(b + 4, 4);                        /* uplink checksum header: nothing asked */
		memcpy(b + ipa_tx_hdr, pkt, len);
		gsi_tre(c, ipa_txbuf_pa + 2048 * slot, (uint16_t)(ipa_tx_hdr + len), (2u << 16) | (1u << 9));
		rang = true;
		ipa_tx_posted++;
		ipa_tx_sent++;
		if (ipa_tx_sent <= 8) {
			LOG("t=%u ms: ipa: packet %llu to the modem: mux %u, %u bytes, proto %u", g_ms, ipa_tx_sent, mux, len, pkt[9]);
		}
		mb();
		ipa_txq_tail = t + 1;
	}
	if (rang) {
		gsi_doorbell(c);
	}
}

/* ---- firmware: GSI microcode, authenticated by TrustZone (Linux ipa_firmware_load) */

static bool
ipa_fw_load(void)
{
	const void *mdt_v, *elf_v;
	unsigned mdt_n, elf_n;
	uint64_t meta_pa, st, res = 0;

	if (!ipa_chosen("ipa-fws-metadata", &mdt_v, &mdt_n) || !ipa_chosen("ipa-fws-image", &elf_v, &elf_n) || mdt_n < 52) {
		LOG("ipa: no ipa_fws in the device tree (tools/mkadt.py --ipa-mdt/--ipa-elf)");
		return false;
	}
	const uint8_t *mdt = (const uint8_t *)mdt_v, *elf = (const uint8_t *)elf_v;
	uint8_t *meta = (uint8_t *)pool_alloc(mdt_n, &meta_pa);
	if (meta == NULL) {
		return false;
	}
	memcpy(meta, mdt, mdt_n);
	uint32_t phoff, min_a = ~0u, max_a = 0;
	uint16_t phnum;
	memcpy(&phoff, mdt + 28, 4);
	memcpy(&phnum, mdt + 44, 2);
	for (unsigned i = 0; i < phnum && phoff + 32u * (i + 1) <= mdt_n; i++) {
		uint32_t ph[8];
		memcpy(ph, mdt + phoff + 32 * i, 32);
		if (ph[0] == PT_LOAD && ph[5] != 0 && (ph[6] & MDT_TYPE_MASK) != MDT_TYPE_HASH) {
			min_a = MIN(min_a, ph[3]);
			max_a = MAX(max_a, (ph[3] + ph[5] + 0xfffu) & ~0xfffu);
		}
	}
	if (min_a == ~0u || max_a - min_a > IPA_FW_SIZE) {
		LOG("ipa: ipa_fws: unexpected layout %x..%x", min_a, max_a);
		return false;
	}
	st = scm(SCM_FN(SCM_SVC_PIL, PIL_SHUTDOWN), 1, IPA_PAS_ID, 0, 0, &res);
	st = scm(SCM_FN(SCM_SVC_PIL, PIL_INIT_IMAGE), 2 | (SCM_ARG_RW << 6), IPA_PAS_ID, meta_pa, 0, &res);
	LOG("ipa: ipa_fws init_image: ret %llx res %llx (%u bytes of metadata)", st, res, mdt_n);
	if (st != 0 || res != 0) {
		return false;
	}
	st = scm(SCM_FN(SCM_SVC_PIL, PIL_MEM_SETUP), 3, IPA_PAS_ID, IPA_FW_PA, max_a - min_a, &res);
	LOG("ipa: ipa_fws mem_setup %llx+%x: ret %llx res %llx", IPA_FW_PA, max_a - min_a, st, res);
	if (st != 0 || res != 0) {
		return false;
	}
	IOMemoryMap *m = map_phys(IPA_FW_PA, IPA_FW_SIZE, true);
	if (m == NULL) {
		return false;
	}
	uint8_t *dst = (uint8_t *)m->getVirtualAddress();
	bzero(dst, IPA_FW_SIZE);
	for (unsigned i = 0; i < phnum && phoff + 32u * (i + 1) <= mdt_n; i++) {
		uint32_t ph[8];
		memcpy(ph, mdt + phoff + 32 * i, 32);
		if (ph[0] != PT_LOAD || ph[5] == 0 || (ph[6] & MDT_TYPE_MASK) == MDT_TYPE_HASH) {
			continue;
		}
		if (ph[1] + ph[4] > elf_n || ph[3] - min_a + ph[4] > IPA_FW_SIZE) {
			LOG("ipa: ipa_fws segment %u out of range", i);
			m->release();
			return false;
		}
		memcpy(dst + (ph[3] - min_a), elf + ph[1], ph[4]);
	}
	mb();
	m->release();
	st = scm(SCM_FN(SCM_SVC_PIL, PIL_AUTH_RESET), 1, IPA_PAS_ID, 0, 0, &res);
	LOG("ipa: ipa_fws auth_and_reset: ret %llx res %llx", st, res);
	return st == 0 && res == 0;
}

/* ---- bring-up, before the modem boots (it asks for the IPA clock state early) */

static bool
ipa_hw_init(void)
{
	int r = topaz_rpm_vote(0, 0x00617069, 0, 0x007a484b, 100000);     /* "ipa" clock, active set, 100 MHz */
	int r2 = topaz_rpm_vote(1, 0x00617069, 0, 0x007a484b, 100000);    /* and while the AP sleeps */
	LOG("ipa: RPM ipa clock 100 MHz: active %d, sleep %d", r, r2);
	if (r != 0) {
		ipa_step = "no clock";
		return false;
	}
	if (!smmu_identity(IPA_SID, 0, "IPA (AP)")) {
		ipa_step = "no SMMU stream";
		return false;
	}
	ipa_r = (volatile uint8_t *)ml_io_map(IPA_REG_PA, IPA_REG_SIZE);
	gsi_r = (volatile uint8_t *)ml_io_map(IPA_GSI_PA, IPA_GSI_SIZE);
	if (ipa_r == NULL || gsi_r == NULL) {
		ipa_step = "no mapping";
		return false;
	}
	uint32_t smem_v = ir(IPA_SHARED_MEM_SIZE), flavor = ir(IPA_FLAVOR_0);
	LOG("ipa: COMP_CFG %08x CLKON %08x SHARED_MEM_SIZE %08x FLAVOR_0 %08x (tx %u, rx %u from %u); GSI status %08x",
	    ir(IPA_COMP_CFG), ir(IPA_CLKON_CFG), smem_v, flavor, (flavor >> 8) & 0x1f, (flavor >> 16) & 0x1f,
	    (flavor >> 24) & 0xf, gr(GSI_STATUS));

	/* Linux ipa_hardware_config() */
	iw(IPA_BCR, 0);
	iw(IPA_TX_CFG, ir(IPA_TX_CFG) & ~(1u << 12));                     /* no PA mask: HOLB drop works */
	iw(IPA_CLKON_CFG, (1u << 28) | (1u << 29));                        /* CLKON_GLOBAL, GLOBAL_2X_CLK */
	iw(IPA_COMP_CFG, ir(IPA_COMP_CFG) | (1u << 15) | (1u << 7) | (1u << 8));
	iw(IPA_QSB_MAX_WRITES, 8);
	iw(IPA_QSB_MAX_READS, 12);
	iw(IPA_COUNTER_CFG, 15u << 4);                                     /* aggregation granularity 500 us */
	iw(IPA_HASH_EN, 0);                                                /* v4.2: no hashed tables */
	iw(IPA_IDLE_CFG, 256);
	iw(IPA_IRQ_EN, 0);

	/* Linux ipa_mem_config(): where the IPA-local memory is, canaries before its regions */
	ipa_mo = 8 * (smem_v >> 16);
	ipa_msize = 8 * (smem_v & 0xffff);
	if (ipa_msize < 0x2000 || ipa_mo + 0x2000 > IPA_REG_SIZE - IPA_SRAM_OFF) {
		LOG("ipa: IPA memory %x + %x, the v4.2 layout wants 0x2000", ipa_mo, ipa_msize);
		ipa_step = "memory too small";
		return false;
	}
	static const uint16_t canary[][2] = {
		{ 0x0288, 2 }, { 0x0290, 2 }, { 0x0310, 2 }, { 0x0318, 2 }, { 0x0398, 2 }, { 0x03a0, 2 },
		{ 0x0420, 2 }, { 0x0428, 2 }, { 0x04a8, 2 }, { 0x05f0, 2 }, { 0x09f8, 2 }, { 0x0a50, 2 },
		{ 0x2000, 1 },
	};
	for (unsigned i = 0; i < sizeof(canary) / sizeof(canary[0]); i++) {
		volatile uint32_t *p = (volatile uint32_t *)(ipa_r + IPA_SRAM_OFF + ipa_mo + canary[i][0]);
		for (unsigned k = 0; k < canary[i][1]; k++) {
			*--p = 0xdeadbeef;
		}
	}
	/* Linux ipa_resource_config(): one source group, one destination group */
	static const uint32_t src[5][2] = { { 3, 63 }, { 3, 3 }, { 10, 10 }, { 1, 1 }, { 5, 5 } };
	static const uint32_t dst[2][2] = { { 3, 3 }, { 1, 63 } };
	for (unsigned t = 0; t < 5; t++) {
		iw(IPA_SRC_RSRC(t), src[t][0] | (src[t][1] << 8));
	}
	for (unsigned t = 0; t < 2; t++) {
		iw(IPA_DST_RSRC(t), dst[t][0] | (dst[t][1] << 8));
	}
	/* the modem's IPA memory in SMEM (Linux ipa_smem_init) */
	if (smem_alloc(smem_partition(0, 1), IPA_SMEM_ITEM, IPA_SMEM_SIZE) == NULL) {
		LOG("ipa: SMEM item %u not allocated", IPA_SMEM_ITEM);
	}

	ipa_step = "GSI firmware";
	if (!ipa_fw_load()) {
		return false;
	}
	uint32_t gs = 0;
	for (unsigned i = 0; i < 100 && !((gs = gr(GSI_STATUS)) & 1); i++) {
		IOSleep(1);
	}
	uint32_t hw2 = gr(GSI_HW_PARAM_2);
	LOG("ipa: GSI status %08x, %u channels and %u event rings per EE", gs, (hw2 >> 3) & 0x1f, (hw2 >> 8) & 0x1f);
	if (!(gs & 1)) {
		ipa_step = "GSI not enabled";
		return false;
	}

	/* Linux gsi_setup(): no interrupts (polled), error log cleared */
	gw(GSI_INTSET, 1);
	gw(GSI_TYPE_IRQ_MSK, 0);
	gw(GSI_SRC_CH_IRQ_MSK, 0);
	gw(GSI_SRC_EV_IRQ_MSK, 0);
	gw(GSI_GLOB_IRQ_EN, 1);
	gw(GSI_IEOB_IRQ_MSK, 0);
	gw(GSI_INTER_EE_CH_MSK, 0);
	gw(GSI_INTER_EE_EV_MSK, 0);
	gw(GSI_GSI_IRQ_EN, 0);
	gw(GSI_ERROR_LOG, 0);

	ipa_pay = (uint8_t *)pool_alloc(32 * 256, &ipa_pay_pa);
	ipa_zero = (uint8_t *)pool_alloc(0x2000, &ipa_zero_pa);
	uint64_t *table = (uint64_t *)pool_alloc(0x1000, &ipa_table_pa);
	if (ipa_pay == NULL || ipa_zero == NULL || table == NULL) {
		ipa_step = "no pool memory";
		return false;
	}
	/* zero rule, filter bitmap (endpoint n is bit n + 1), then 15 entries all naming the zero rule */
	table[0] = 0;
	table[1] = (uint64_t)IPA_FILTERED << 1;
	for (unsigned i = 0; i < 15; i++) {
		table[2 + i] = ipa_table_pa;
	}

	ipa_step = "GSI channels";
	for (unsigned i = 0; i < 4; i++) {
		if (!gsi_setup_chan(&ipa_ch[i])) {
			return false;
		}
	}
	for (unsigned ch = 0; ch < 4; ch++) {   /* modem channels 0-3: the AP allocates them on IPA v4.2 */
		int g = gsi_generic(2, ch);
		if (g != 0) {
			LOG("ipa: modem channel %u allocation: %d", ch, g);
		}
	}
	ipa_ep_program();
	ipa_step = "command channel";
	if (!gsi_start(IPA_CMD)) {
		return false;
	}

	/* Linux ipa_mem_setup(): header table, zeroed processing contexts and modem memory */
	ipa_step = "IPA memory setup";
	uint8_t *p = ipa_cmd_add(IPA_CMD_HDR_INIT_LOCAL, false);
	uint32_t hflags = 0x140u | ((ipa_mo + IPA_MEM_MODEM_HDR) << 12);
	memcpy(p, &ipa_zero_pa, 8);
	memcpy(p + 8, &hflags, 4);
	ipa_cmd_dma(IPA_MEM_MODEM_PROC, 0x200, false);
	ipa_cmd_dma(IPA_MEM_AP_PROC, 0x200, false);
	ipa_cmd_dma(IPA_MEM_MODEM, 0x140c, true);
	if (!ipa_cmd_commit("memory setup")) {
		return false;
	}
	iw(IPA_PKT_PROC_CNTXT, (ipa_mo + IPA_MEM_MODEM_PROC) & 0x1ffff);

	/* Linux ipa_table_setup(): 15 route entries, filter bitmap + 2 entries, the rest zeroed */
	ipa_step = "tables";
	ipa_cmd_table(IPA_CMD_V4_ROUTE_INIT, IPA_MEM_V4_ROUTE, 15 * 8, ipa_table_pa + 16, false);
	ipa_cmd_table(IPA_CMD_V6_ROUTE_INIT, IPA_MEM_V6_ROUTE, 15 * 8, ipa_table_pa + 16, false);
	ipa_cmd_table(IPA_CMD_V4_FILTER_INIT, IPA_MEM_V4_FILTER, 3 * 8, ipa_table_pa + 8, false);
	ipa_cmd_dma(IPA_MEM_V4_FILTER + 3 * 8, IPA_MEM_TABLE_SIZE - 3 * 8, false);
	ipa_cmd_table(IPA_CMD_V6_FILTER_INIT, IPA_MEM_V6_FILTER, 3 * 8, ipa_table_pa + 8, false);
	ipa_cmd_dma(IPA_MEM_V6_FILTER + 3 * 8, IPA_MEM_TABLE_SIZE - 3 * 8, true);
	if (!ipa_cmd_commit("tables")) {
		return false;
	}

	ipa_step = "LAN RX";
	if (!gsi_start(IPA_LAN) || !ipa_rx_init(IPA_LAN)) {
		return false;
	}
	iw(IPA_ROUTE, (8u << 1) | (1u << 6) | (8u << 17) | (1u << 24));  /* default route: LAN RX, keep headers */
	ipa_up = true;
	ipa_step = "up, waiting for the modem's IPA service";
	LOG("ipa: up (memory %x + %x, pool used %llx)", ipa_mo, ipa_msize, pool_used);
	return true;
}

/* ---- QMI handshake (Linux ipa_qmi.c) */

static uint32_t
ipa_qmi_hdr(uint8_t *b, uint8_t type, uint16_t txn, uint16_t msg)
{
	b[0] = type;
	memcpy(b + 1, &txn, 2);
	memcpy(b + 3, &msg, 2);
	return 7;
}

static uint32_t
ipa_tlv(uint8_t *b, uint32_t n, uint8_t type, const void *v, uint16_t len)
{
	b[n] = type;
	memcpy(b + n + 1, &len, 2);
	memcpy(b + n + 3, v, len);
	return n + 3 + len;
}

static void
ipa_qmi_send(uint8_t *b, uint32_t n, uint32_t src_port, uint32_t node, uint32_t port)
{
	uint16_t l = (uint16_t)(n - 7);
	memcpy(b + 5, &l, 2);
	qrtr_send(QRTR_TYPE_DATA, src_port, node, port, b, n);
}

static void
ipa_modem_start(void)
{
	if (ipa_running) {
		return;
	}
	if (ipa_txq_lock == NULL) {
		ipa_txq_lock = IOSimpleLockAlloc();
	}
	ipa_txbuf = (uint8_t *)pool_alloc(2048 * IPA_TX_SLOTS, &ipa_txbuf_pa);
	if (ipa_txbuf == NULL || !gsi_start(IPA_MTX) || !gsi_start(IPA_MRX) || !ipa_rx_init(IPA_MRX)) {
		ipa_step = "modem endpoints did not start";
		return;
	}
	ipa_running = true;
	ipa_step = "running";
	LOG("t=%u ms: ipa: *** modem TX/RX endpoints started: the modem's data can flow ***", g_ms);
	sweet_pdp_create();
}

static void
ipa_qmi_ready(void)
{
	if (!ipa_modem_ready || !ipa_uc_ready) {
		return;
	}
	if (ipa_ind_req && !ipa_ind_sent) {
		uint8_t b[32];
		uint32_t n = ipa_qmi_hdr(b, QMI_IND, 0, 0x22);
		uint32_t ok = 0;
		n = ipa_tlv(b, n, 0x02, &ok, 4);
		ipa_qmi_send(b, n, PORT_IPAS, ipa_c_node, ipa_c_port);
		ipa_ind_sent = true;
		LOG("t=%u ms: ipa: INIT_COMPLETE indication sent", g_ms);
	}
	if (ipa_ind_sent) {
		ipa_modem_start();
	}
}

/* the modem's IPA service 0x31 instance 2 came up: INIT_DRIVER with our memory layout */
static void
ipa_qmi_arrive(uint32_t node, uint32_t port)
{
	ipa_m_node = node;
	ipa_m_port = port;
	if (!ipa_up) {
		LOG("t=%u ms: ipa: modem IPA service up, but our IPA is not (%s)", g_ms, ipa_step);
		return;
	}
	struct { uint32_t a, b; } __attribute__((packed)) v;
	uint8_t b[256];
	uint32_t n = ipa_qmi_hdr(b, QMI_REQ, ipa_txn++, 0x21), u;
	uint8_t skip = 0;

	u = 3; n = ipa_tlv(b, n, 0x10, &u, 4);                                       /* platform: MSM Android */
	v.a = ipa_mo + IPA_MEM_MODEM_HDR; v.b = v.a + 0x140 - 1; n = ipa_tlv(b, n, 0x11, &v, 8);
	v.a = ipa_mo + IPA_MEM_V4_ROUTE; v.b = IPA_MODEM_ROUTES - 1; n = ipa_tlv(b, n, 0x12, &v, 8);
	v.a = ipa_mo + IPA_MEM_V6_ROUTE; v.b = IPA_MODEM_ROUTES - 1; n = ipa_tlv(b, n, 0x13, &v, 8);
	u = ipa_mo + IPA_MEM_V4_FILTER; n = ipa_tlv(b, n, 0x14, &u, 4);
	u = ipa_mo + IPA_MEM_V6_FILTER; n = ipa_tlv(b, n, 0x15, &u, 4);
	v.a = ipa_mo + IPA_MEM_MODEM; v.b = 0x140c; n = ipa_tlv(b, n, 0x16, &v, 8);
	u = 9; n = ipa_tlv(b, n, 0x17, &u, 4);                                       /* control: AP modem RX */
	n = ipa_tlv(b, n, 0x18, &skip, 1);                                           /* the modem loads the uC */
	v.a = ipa_mo + IPA_MEM_MODEM_PROC; v.b = v.a + 0x200 - 1; n = ipa_tlv(b, n, 0x19, &v, 8);
	u = ipa_mo + IPA_MEM_QUOTA; n = ipa_tlv(b, n, 0x1f, &u, 4);
	u = ipa_mo + 0x60; n = ipa_tlv(b, n, 0x20, &u, 4);                           /* (Linux: mem_offset + size) */
	ipa_qmi_send(b, n, PORT_IPAC, node, port);
	ipa_step = "INIT_DRIVER sent";
	LOG("t=%u ms: ipa: modem IPA service up (node %u port %x): INIT_DRIVER sent", g_ms, node, port);
}

/* a response to our client, or a request to our service */
static bool
ipa_svc_rx(uint32_t src_node, uint32_t src_port, uint32_t dst_port, const uint8_t *d, uint32_t len)
{
	if (dst_port != PORT_IPAC && dst_port != PORT_IPAS) {
		return false;
	}
	if (len < 7) {
		return true;
	}
	uint16_t txn, msg, l = 0, res = 0xffff, err = 0xffff;
	memcpy(&txn, d + 1, 2);
	memcpy(&msg, d + 3, 2);
	const uint8_t *r = qm_find(d, len, 2, &l);
	if (r != NULL && l >= 4) {
		memcpy(&res, r, 2);
		memcpy(&err, r + 2, 2);
	}
	if (dst_port == PORT_IPAC) {
		LOG("t=%u ms: ipa: INIT_DRIVER answer: result %u error %u", g_ms, res, err);
		if (msg == 0x21 && res == 0) {
			ipa_modem_ready = true;
			ipa_step = "modem driver ready";
			ipa_qmi_ready();
		} else {
			ipa_step = "INIT_DRIVER refused";
		}
		return true;
	}
	if (d[0] != QMI_REQ) {
		return true;
	}
	uint8_t b[32];
	uint32_t n = ipa_qmi_hdr(b, QMI_RESP, txn, msg), ok = 0;
	n = ipa_tlv(b, n, 0x02, &ok, 4);
	ipa_qmi_send(b, n, PORT_IPAS, src_node, src_port);
	ipa_c_node = src_node;
	ipa_c_port = src_port;
	if (msg == 0x20) {
		LOG("t=%u ms: ipa: modem INDICATION_REGISTER", g_ms);
		ipa_ind_req = true;
	} else if (msg == 0x35) {
		LOG("t=%u ms: ipa: modem DRIVER_INIT_COMPLETE (the IPA microcontroller is up)", g_ms);
		ipa_uc_ready = true;
	} else {
		LOG("t=%u ms: ipa: modem request %04x (%u bytes) answered", g_ms, msg, len);
		for (uint32_t o = 7; o < len && o < 7 + 192; o += 32) {
			char hex[100];
			uint32_t k = 0;
			for (uint32_t i = o; i < len && i < o + 32; i++) {
				k += (uint32_t)snprintf(hex + k, sizeof(hex) - k, "%02x", d[i]);
			}
			LOG("ipa:   %04x: %s", o, hex);
		}
	}
	ipa_qmi_ready();
	return true;
}

/* every millisecond from the modem thread */
static void
ipa_poll(unsigned ms)
{
	if (!ipa_up) {
		return;
	}
	gsi_events(IPA_CMD, ipa_ev_count);
	if (gsi_events(IPA_LAN, ipa_rx_event) != 0) {
		gsi_doorbell(IPA_LAN);
	}
	if (ipa_running) {
		gsi_events(IPA_MTX, ipa_tx_event);
		ipa_tx_pump();
		if (gsi_events(IPA_MRX, ipa_rx_event) != 0) {
			gsi_doorbell(IPA_MRX);
		}
	}
	if (ms % 1000 == 0) {
		uint32_t e = gr(GSI_ERROR_LOG);
		if (e != 0) {
			ipa_gsi_errors++;
			ipa_last_error = e;
			if (ipa_gsi_errors <= 10) {
				LOG("t=%u ms: ipa: GSI error log %08x (ee %u type %u index %u code %u)", ms, e, e >> 28,
				    (e >> 24) & 0xf, (e >> 19) & 0x1f, (e >> 12) & 0xf);
			}
			gw(GSI_ERROR_LOG, 0);
			gw(GSI_ERROR_LOG_CLR, ~0u);
		}
	}
}

/*
 * sapphire 0229: kern.sweet_ipa_peek, any IPA or GSI register (every EE: the modem's channels too)
 * or the IPA-local memory, read and written from user space: "i OFF [N]" IPA registers, "g OFF [N]"
 * GSI registers, "s OFF [N]" IPA memory (N words, default 16); "wi|wg|ws OFF VALUE" writes one.
 * Reading the sysctl returns the words of the last request.
 */
static char ipa_peek_out[1024];
extern "C" unsigned long strtoul(const char *nptr, char **endptr, int base);

extern "C" int
topaz_ipa_peek(const char *req)
{
	char sp[4] = {};
	unsigned long off = 0, arg = 16;
	int n = 0;
	if (ipa_r == NULL || gsi_r == NULL) {
		return ENXIO;
	}
	while (req[n] == ' ') n++;
	int k = 0;
	while (req[n] && req[n] != ' ' && k < 3) sp[k++] = req[n++];
	const char *p = req + n;
	off = strtoul(p, (char **)&p, 16);
	if (*p) {
		arg = strtoul(p, NULL, 0);
	}
	if (sp[0] == 'h') {                     /* "h4" / "h8": the uplink header length */
		ipa_tx_hdr = sp[1] == '4' ? 4 : 8;
		snprintf(ipa_peek_out, sizeof(ipa_peek_out), "uplink header %u bytes\n", ipa_tx_hdr);
		return 0;
	}
	if (sp[0] == 'p') {                     /* "p PA [N]": physical memory, one page at most */
		uint64_t pa = strtoul(p, (char **)&p, 16);    /* unsigned long: 64 bits */
		unsigned long words = *p ? strtoul(p, NULL, 0) : 16;
		if ((pa & 3) || words == 0 || words > 64 || ((pa & 0xfff) + 4 * words) > 0x1000) {
			return EINVAL;
		}
		IOMemoryMap *m = map_phys(pa & ~0xfffull, 0x1000, false);
		if (m == NULL) {
			return ENOMEM;
		}
		volatile uint32_t *v = (volatile uint32_t *)(m->getVirtualAddress() + (pa & 0xfff));
		int o = 0;
		for (unsigned long i = 0; i < words; i++) {
			if (i % 8 == 0) {
				o += snprintf(ipa_peek_out + o, sizeof(ipa_peek_out) - o, "%sp %llx:", i ? "\n" : "", (unsigned long long)(pa + 4 * i));
			}
			o += snprintf(ipa_peek_out + o, sizeof(ipa_peek_out) - o, " %08x", v[i]);
		}
		snprintf(ipa_peek_out + o, sizeof(ipa_peek_out) - o, "\n");
		m->release();
		return 0;
	}
	bool write = sp[0] == 'w';
	char kind = write ? sp[1] : sp[0];
	uint32_t limit = kind == 'i' ? IPA_REG_SIZE : kind == 'g' ? IPA_GSI_SIZE : kind == 's' ? IPA_REG_SIZE - IPA_SRAM_OFF : 0;
	volatile uint8_t *base = kind == 'i' ? ipa_r : kind == 'g' ? gsi_r : ipa_r + IPA_SRAM_OFF;
	if (limit == 0 || (off & 3) != 0 || off + 4 > limit) {
		return EINVAL;
	}
	if (write) {
		*(volatile uint32_t *)(base + off) = (uint32_t)arg;
		mb();
		snprintf(ipa_peek_out, sizeof(ipa_peek_out), "%c %lx <- %08lx, reads %08x\n", kind, off, arg,
		    *(volatile uint32_t *)(base + off));
		return 0;
	}
	if (arg == 0 || arg > 64) {
		arg = 16;
	}
	int o = 0;
	for (unsigned long i = 0; i < arg && off + 4 * i + 4 <= limit; i++) {
		if (i % 8 == 0) {
			o += snprintf(ipa_peek_out + o, sizeof(ipa_peek_out) - o, "%s%c %05lx:", i ? "\n" : "", kind, off + 4 * i);
		}
		o += snprintf(ipa_peek_out + o, sizeof(ipa_peek_out) - o, " %08x", *(volatile uint32_t *)(base + off + 4 * i));
	}
	snprintf(ipa_peek_out + o, sizeof(ipa_peek_out) - o, "\n");
	return 0;
}

extern "C" const char *
topaz_ipa_peek_out(void)
{
	return ipa_peek_out;
}

/* kern.sweet_ipa_stats: each AP channel's ring positions and states, the IPA's routing registers */
extern "C" int
topaz_ipa_stats(char *buf, size_t len)
{
	int n = snprintf(buf, len, "ipa %s; mem %x+%x; ROUTE %08x AGGR_ACTIVE %08x; GSI status %08x error %08x glob %08x\n", ipa_step,
	    ipa_mo, ipa_msize, ipa_r ? ir(IPA_ROUTE) : 0, ipa_r ? ir(IPA_AGGR_ACTIVE) : 0, gsi_r ? gr(GSI_STATUS) : 0,
	    gsi_r ? gr(GSI_ERROR_LOG) : 0, gsi_r ? gr(GSI_GLOB_IRQ_STTS) : 0);
	for (unsigned i = 0; i < 4 && gsi_r != NULL && n > 0 && (size_t)n < len; i++) {
		struct ipa_chan *c = &ipa_ch[i];
		n += snprintf(buf + n, len - n, "%s: ch %u state %u ev %u state %u; filled %u read %u; hw ev write %x (ring %llx); frames %llu bytes %llu pkts %llu events %llu code %u\n",
		    c->name, c->ch, gsi_state(GSI_CH(0, c->ch)), c->ev, gsi_state(GSI_EV(0, c->ev)), c->widx, c->eidx,
		    gr(GSI_EV(4, c->ev)), c->evr_pa, c->frames, c->bytes, c->pkts, c->cmds, c->last_code);
	}
	if (n > 0 && (size_t)n < len) {
		n += snprintf(buf + n, len - n, "tx queue: head %u tail %u, posted %llu sent %llu, full %llu\n", ipa_txq_head,
		    ipa_txq_tail, ipa_tx_posted, ipa_tx_sent, ipa_tx_full);
	}
	return n;
}

static int
ipa_stats(char *buf, size_t len)
{
	return snprintf(buf, len, "; ipa %s: lan rx %llu frames, modem rx %llu frames %llu packets %llu bytes (mux 0-7: %u %u %u %u %u %u %u %u), modem tx %llu, GSI errors %u (last %08x)",
	    ipa_step, IPA_LAN->frames, IPA_MRX->frames, IPA_MRX->pkts, IPA_MRX->bytes, ipa_mux_seen[0], ipa_mux_seen[1],
	    ipa_mux_seen[2], ipa_mux_seen[3], ipa_mux_seen[4], ipa_mux_seen[5], ipa_mux_seen[6], ipa_mux_seen[7],
	    IPA_MTX->cmds, ipa_gsi_errors, ipa_last_error);
}

static void
svc_announce(void)
{
	static const uint32_t svc[][3] = {
		{ 0x40, 0x101, PORT_PDM },              /* servreg-locator, version 0x101 instance 0 */
		{ 4096, 1,     PORT_TFTP },             /* tftp */
		{ 14,   1,     PORT_RMTFS },            /* rmtfs */
	};
	for (unsigned i = 0; i < 3; i++) {
		struct qrtr_ctrl c = { QRTR_TYPE_NEW_SERVER, svc[i][0], svc[i][1], QRTR_NODE_APPS, svc[i][2] };
		qrtr_send(QRTR_TYPE_NEW_SERVER, QRTR_PORT_CTRL, q_modem_node, QRTR_PORT_CTRL, &c, sizeof(c));
	}
	LOG("t=%u ms: announced pd-mapper 0x40, tftp 0x1000, rmtfs 0x0e", g_ms);
	if (ipa_up) {
		struct qrtr_ctrl c = { QRTR_TYPE_NEW_SERVER, 0x31, 0x101, QRTR_NODE_APPS, PORT_IPAS };
		qrtr_send(QRTR_TYPE_NEW_SERVER, QRTR_PORT_CTRL, q_modem_node, QRTR_PORT_CTRL, &c, sizeof(c));
		LOG("t=%u ms: announced the IPA service 0x31:1", g_ms);
	}
}

static bool
svc_rx(uint32_t src_node, uint32_t src_port, uint32_t dst_port, const uint8_t *data, uint32_t len)
{
	WHERE("qmi deliver");
	if (sweet_qmi_deliver(dst_port, src_node, src_port, data, len)) {
		WHERE("svc rx");
		return true;                            /* a userland client's port (0x3000..) */
	}
	if (ipa_svc_rx(src_node, src_port, dst_port, data, len)) {
		return true;
	}
	WHERE("svc rx (kernel services)");
	if (dst_port == PORT_TFTP) {
		WHERE("tftp");
		if (len >= 2 && (data[1] == OP_RRQ || data[1] == OP_WRQ)) {
			tftp_request(src_node, src_port, data, len);
		}
		return true;
	}
	if (dst_port == PORT_WLFWC) {
		wlfw_rx(data, len);
		return true;
	}
	if (dst_port >= PORT_SESS && dst_port < PORT_SESS + 0x10000) {
		return tftp_session(dst_port, data, len);
	}
	if (len < 7 || data[0] != QMI_REQ) {
		return false;
	}
	uint16_t txn, msg;
	memcpy(&txn, data + 1, 2);
	memcpy(&msg, data + 3, 2);
	if (dst_port == PORT_PDM) {
		pdm_rx(src_node, src_port, txn, msg, data, len);
		return true;
	}
	if (dst_port == PORT_RMTFS) {
		WHERE("rmtfs");
		rmtfs_rx(src_node, src_port, txn, msg, data, len);
		return true;
	}
	return false;
}

/*
 * sapphire 0220: a userland client's message (sweet_qmi_pump, modem thread), or, with ctrl,
 * QRTR DEL_CLIENT for its port after close() (Linux's qrtr_port_remove does the same).
 */
static void
qmi_user_send(uint32_t src_port, uint32_t node, uint32_t port, const void *data, uint32_t len, int ctrl)
{
	uint32_t an = adsp_node();
	if (ctrl) {
		struct qrtr_ctrl c = { QRTR_TYPE_DEL_CLIENT, QRTR_NODE_APPS, src_port, 0, 0 };
		qrtr_send(QRTR_TYPE_DEL_CLIENT, QRTR_PORT_CTRL, q_modem_node, QRTR_PORT_CTRL, &c, sizeof(c));
		if (an != QRTR_NODE_BCAST) {
			/* 0264: the ADSP's services (SNS_CLIENT) drop the client's streams too */
			qrtr_send(QRTR_TYPE_DEL_CLIENT, QRTR_PORT_CTRL, an, QRTR_PORT_CTRL, &c, sizeof(c));
		}
		return;
	}
	/* 0264: the modem's node, or the ADSP's (qrtr_send() picks its edge by node) */
	if (node != q_modem_node && (an == QRTR_NODE_BCAST || node != an)) {
		static unsigned dropped;
		if (dropped++ < 8) {
			LOG("qmi user: port %x to node %u: no such node (modem %u, adsp %u), dropped", src_port, node,
			    q_modem_node, an);
		}
		return;
	}
	qrtr_send(QRTR_TYPE_DATA, src_port, node, port, data, len);
}

/* the index-th server of a QMI service the modem announced (inst ~0: any instance) */
extern "C" int
topaz_qmi_lookup(uint32_t svc, uint32_t inst, uint32_t index, uint32_t *node, uint32_t *port, uint32_t *found_inst)
{
	for (uint32_t i = 0, seen = 0; i < q_srv_count; i++) {
		if (q_srv[i].svc != svc || (inst != ~0u && q_srv[i].inst != inst)) {
			continue;
		}
		if (seen++ == index) {
			*node = q_srv[i].node;
			*port = q_srv[i].port;
			*found_inst = q_srv[i].inst;
			return 1;
		}
	}
	return 0;
}

static void
svc_summary(void)
{
	LOG("WLFW: %s%s", w_step, w_fw_ready ? " (firmware ready)" : "");
	LOG("services: pd-mapper %u reqs; tftp %u reqs (%u missing, %u writes); rmtfs reads %u writes %u (not kept) errors %u",
	    pd_reqs, tftp_reqs, tftp_missing, tftp_writes, rmtfs_reads, rmtfs_writes, rmtfs_err);
}

/*
 * After auth_and_reset: watch the modem's "slave-kernel" entry (bit0 fatal, bit1 ready,
 * bit2 handover) and answer its IPA power query ("valid, AP IPA power off"): unanswered,
 * modem_cfg starves and the modem watchdog fires once data services come up.
 */
static void
modem_watch(struct smem_part_hdr *part, unsigned seconds)
{
	struct smp2p_hdr *in = NULL, *out = smp2p_create_out(part);
	volatile uint32_t *sk = NULL, *ipa_in = NULL, *ipa_out = NULL;
	uint32_t sk_last = ~0u, ipa_last = ~0u;
	bool ready = false;

	if (out == NULL) {
		LOG("cannot create SMP2P item %u", ITEM_SMP2P_OUT);
		return;
	}
	ipa_out = smp2p_add_out(out, "ipa", 0);
	bool glink = false;
	for (unsigned ms = 0; seconds == 0 || ms < seconds * 1000; ms++) {
		if (ready && !glink) {
			glink = glink_start(part);
			if (!glink) {
				ready = false;  /* no retry */
			}
		}
		g_loops++;
		if (glink) {
			glink_poll(part, ms);
			WHERE("radio poll");
			radio_poll(ms);
			WHERE("ipa poll");
			ipa_poll(ms);
			WHERE("watch loop");
		}
		if (glink && (ms == 3000 || ms == 20000)) {
			glink_summary();                /* early, before the iOS UI covers the console */
		}
		if (in == NULL) {
			uint32_t sz = 0;
			in = (struct smp2p_hdr *)smem_get(part, ITEM_SMP2P_IN, &sz);
			if (in != NULL && in->magic != SMP2P_MAGIC) {
				in = NULL;
			}
			if (in != NULL) {
				LOG("t=%u ms: modem SMP2P item %u up (v%u, %u entries)", ms, ITEM_SMP2P_IN, in->version, in->valid);
			}
		}
		if (in != NULL && sk == NULL) {
			sk = smp2p_find(in, "slave-kernel");
		}
		if (in != NULL && ipa_in == NULL) {
			ipa_in = smp2p_find(in, "ipa");
		}
		if (sk != NULL && *sk != sk_last) {
			sk_last = *sk;
			LOG("t=%u ms: slave-kernel %08x%s%s%s", ms, sk_last, (sk_last & 1) ? " FATAL" : "",
			    (sk_last & 2) ? " READY" : "", (sk_last & 4) ? " HANDOVER" : "");
			if ((sk_last & 2) && !ready) {
				ready = true;
				LOG("modem is up (MODEM READY)");
			}
			if (sk_last & 1) {
				uint32_t sz = 0;
				const char *why = (const char *)smem_get(smem_partition(SMEM_GLOBAL_HOST, SMEM_GLOBAL_HOST), ITEM_MSS_CRASH, &sz);
				char buf[128] = {};
				if (why != NULL) {
					memcpy(buf, why, MIN(sz, (uint32_t)sizeof(buf) - 1));
				}
				LOG("modem crash reason: %s", why != NULL ? buf : "(none)");
				if (glink) {
					glink_summary();
				}
				return;
			}
		}
		if (ipa_in != NULL && ipa_out != NULL && *ipa_in != ipa_last) {
			uint32_t v = *ipa_in;
			if ((v & 1) && !(ipa_last != ~0u && (ipa_last & 1))) {
				*ipa_out = ipa_up ? 3 : 1;      /* valid, and the IPA clock on (ipa=1) or off */
				smp2p_kick();
				LOG("t=%u ms: answered IPA power query (%08x): IPA power %s", ms, v, ipa_up ? "on" : "off");
			}
			ipa_last = v;
		}
		IOSleep(1);
	}
	LOG("watch done: slave-kernel %08x, %s", sk_last, ready ? "READY" : "NOT ready");
	if (glink) {
		glink_summary();
	}
}

/* ---------------------------------------------------------------- PAS boot */

static bool
loadable(const struct elf32_phdr *p)
{
	return p->p_type == PT_LOAD && p->p_memsz != 0 && (p->p_flags & MDT_TYPE_MASK) != MDT_TYPE_HASH;
}

static bool
modem_boot(void)
{
	uint8_t *mdt = NULL, *meta;
	int64_t mdt_size;
	uint64_t meta_pa = 0, res = 0, st;
	uint32_t min_a = ~0u, max_a = 0, hash = ~0u;
	bool relocate = false, ok = false;

	struct smem_part_hdr *part = smem_partition(0, 1);
	LOG("SMEM apps<->modem partition %p", part);
	if (part == NULL) {
		return false;
	}
	if (smem_get(part, ITEM_SMP2P_IN, NULL) != NULL) {
		LOG("modem SMP2P item %u already exists: modem is not cold, not touching it", ITEM_SMP2P_IN);
		return false;
	}
	st = scm(SCM_FN(SCM_SVC_PIL, PIL_IS_SUPPORTED), 1, PAS_ID_MODEM, 0, 0, &res);
	LOG("PAS id %u supported: ret=%llx res=%llx", PAS_ID_MODEM, st, res);
	/* informational only: on topaz TZ answers res=0 here, init_image decides */

	mdt_size = fw_size("modem.mdt");
	if (mdt_size <= (int64_t)sizeof(struct elf32_ehdr) || mdt_size > 0x100000) {
		LOG("modem.mdt: size %lld", mdt_size);
		return false;
	}
	mdt = (uint8_t *)IOMalloc(mdt_size);
	if (mdt == NULL || fw_read("modem.mdt", mdt, mdt_size) != mdt_size) {
		goto out;
	}
	{
		struct elf32_ehdr *eh = (struct elf32_ehdr *)mdt;
		if (memcmp(eh->e_ident, "\x7f" "ELF", 4) != 0 ||
		    eh->e_phoff + (uint64_t)eh->e_phnum * sizeof(struct elf32_phdr) > (uint64_t)mdt_size) {
			LOG("modem.mdt: bad ELF header");
			goto out;
		}
		struct elf32_phdr *ph = (struct elf32_phdr *)(mdt + eh->e_phoff);
		for (uint32_t i = 0; i < eh->e_phnum; i++) {
			if ((ph[i].p_flags & MDT_TYPE_MASK) == MDT_TYPE_HASH) {
				hash = i;
			}
			if (!loadable(&ph[i])) {
				continue;
			}
			if (ph[i].p_flags & MDT_RELOCATABLE) {
				relocate = true;
			}
			min_a = MIN(min_a, ph[i].p_paddr);
			max_a = MAX(max_a, ph[i].p_paddr + ph[i].p_memsz);
		}
		LOG("phdrs=%u hash=%u region %08x..%08x relocate=%u", eh->e_phnum, hash, min_a, max_a, relocate);
		if (hash == ~0u || min_a < 0x4AB00000u || max_a > 0x4AB00000u + 0x6900000u) {
			LOG("unexpected layout (carveout is 0x4ab00000+0x6900000)");
			goto out;
		}

		/* metadata = ELF header + phdrs + hash segment, contiguous, below 0xF0000000 */
		uint64_t hdr = eh->e_phoff + eh->e_phnum * sizeof(struct elf32_phdr);
		uint64_t meta_size = hdr + ph[hash].p_filesz;
		meta = (uint8_t *)pool_alloc(meta_size, &meta_pa);
		if (meta == NULL) {
			LOG("metadata: pool allocation of %llu bytes failed", meta_size);
			goto out;
		}
		memcpy(meta, mdt, hdr);
		if (ph[hash].p_offset + ph[hash].p_filesz <= (uint64_t)mdt_size) {
			memcpy(meta + hdr, mdt + ph[hash].p_offset, ph[hash].p_filesz);
		} else {
			/* split firmware: the hash segment is its own .bNN file (qcom_mdt_read_metadata) */
			char hname[16];
			snprintf(hname, sizeof(hname), "modem.b%02u", hash);
			int64_t got = fw_read(hname, meta + hdr, ph[hash].p_filesz);
			LOG("hash from %s: %lld of %u bytes", hname, got, ph[hash].p_filesz);
			if (got != (int64_t)ph[hash].p_filesz) {
				goto out;
			}
		}
		mb();

		st = scm(SCM_FN(SCM_SVC_PIL, PIL_INIT_IMAGE), 2 | (SCM_ARG_RW << 6), PAS_ID_MODEM, meta_pa, 0, &res);
		LOG("init_image: ret=%llx res=%llx resumes=%u (meta %llu bytes @%llx)", st, res, scm_resumes, meta_size, meta_pa);
		if (st != 0 || res != 0) {
			goto out;
		}
		if (relocate) {
			st = scm(SCM_FN(SCM_SVC_PIL, PIL_MEM_SETUP), 3, PAS_ID_MODEM, min_a,
			    ((uint64_t)(max_a - min_a) + 0xfff) & ~0xfffull, &res);
			LOG("mem_setup: ret=%llx res=%llx", st, res);
			if (st != 0 || res != 0) {
				goto shutdown;
			}
		}

		for (uint32_t i = 0; i < eh->e_phnum; i++) {
			char name[16];
			if (!loadable(&ph[i])) {
				continue;
			}
			/* map only while copying: TZ locks the carveout at auth_and_reset */
			uint64_t pa = ph[i].p_paddr & ~0x3fffull, off = ph[i].p_paddr - pa;
			uint64_t len = (off + ph[i].p_memsz + 0x3fff) & ~0x3fffull;
			IOMemoryMap *m = map_phys(pa, len, true);
			if (m == NULL) {
				LOG("segment %u: cannot map %08x+%x", i, ph[i].p_paddr, ph[i].p_memsz);
				goto shutdown;
			}
			uint8_t *dst = (uint8_t *)m->getVirtualAddress() + off;
			int64_t got = 0;
			if (ph[i].p_filesz != 0) {
				snprintf(name, sizeof(name), "modem.b%02u", i);
				got = fw_read(name, dst, ph[i].p_filesz);
				if (got != (int64_t)ph[i].p_filesz) {
					LOG("%s: %lld of %u bytes", name, got, ph[i].p_filesz);
					m->release();
					goto shutdown;
				}
			}
			if (ph[i].p_memsz > got) {
				bzero(dst + got, ph[i].p_memsz - got);
			}
			mb();
			m->release();
		}
		LOG("segments loaded");

		st = scm(SCM_FN(SCM_SVC_PIL, PIL_AUTH_RESET), 1, PAS_ID_MODEM, 0, 0, &res);
		LOG("auth_and_reset: ret=%llx res=%llx resumes=%u", st, res, scm_resumes);
		if (st != 0 || res != 0) {
			goto shutdown;
		}
		ok = true;
		goto out;
	}
shutdown:
	st = scm(SCM_FN(SCM_SVC_PIL, PIL_SHUTDOWN), 1, PAS_ID_MODEM, 0, 0, &res);
	LOG("shutdown after failure: ret=%llx res=%llx", st, res);
out:
	if (mdt != NULL) {
		IOFree(mdt, mdt_size);
	}
	return ok;
}

/* ---------------------------------------------------------------- ADSP: the audio DSP (audio=1) */

/*
 * sapphire: the LPASS Q6 (the audio DSP), the first step towards sound. Booted the way the modem
 * is (TrustZone PAS, id 1: image/adsp.mdt + .bNN from the same modem_a partition, carveout
 * 0x53800000 + 0x2200000 from the device tree's pil_adsp_region), then its own SMP2P (items
 * 429 out / 443 in, doorbell APCS bit 10) and its own GLINK edge (SMEM host 2, items 478-480,
 * doorbell bit 8). Two channels: "IPCRTR" (QRTR: its services, our pd-mapper) and "adsp_apps"
 * (GPR, the AudioReach packets: APM, the graph manager, and PRM, its resources: Linux
 * drivers/soc/qcom/apr.c, sound/soc/qcom/qdsp6/q6apm.c). The test here: ask APM whether the
 * signal processing framework is up (APM_CMD_GET_SPF_STATE).
 */
#define PAS_ID_ADSP         1
#define ADSP_CARVEOUT       0x53800000u
#define ADSP_CARVEOUT_SIZE  0x02200000u
#define ADSP_HOST           2                   /* SMEM host, SMP2P remote pid */
#define ADSP_ITEM_SMP2P_OUT 429
#define ADSP_ITEM_SMP2P_IN  443
#define ADSP_SMP2P_BIT      10
#define ADSP_GLINK_BIT      8

#define GPR_DOMAIN_ADSP     2
#define GPR_DOMAIN_APPS     3
#define GPR_PORT_SPF_CORE   3                   /* the device tree's spf_core, reg 3 */
#define APM_PORT            1                   /* APM_MODULE_INSTANCE_ID */
#define APM_CMD_GET_SPF_STATE     0x01001021u
#define APM_CMD_RSP_GET_SPF_STATE 0x02001007u
#define GPR_BASIC_RSP_RESULT      0x02001005u

static unsigned audio_on, modem_on;
static unsigned a_ms;
static bool a_verbose;                          /* audio_test(): every GLINK command logged */

static void
adsp_kick(unsigned bit)
{
	mb();
	*(volatile uint32_t *)(apcs + APCS_IPC) = 1u << bit;
	mb();
}

static bool
adsp_boot(void)
{
	int fi = fw_all_find("image/adsp.mdt");
	uint8_t *mdt = NULL, *meta;
	uint64_t mdt_size, meta_pa = 0, res = 0, st;
	uint32_t min_a = ~0u, max_a = 0, hash = ~0u;
	bool relocate = false, ok = false;
	struct smem_part_hdr *part = smem_partition(0, ADSP_HOST);

	LOG("adsp: SMEM apps<->adsp partition %p, adsp.mdt %d", part, fi);
	if (part == NULL || fi < 0) {
		return false;
	}
	if (smem_get(part, ADSP_ITEM_SMP2P_IN, NULL) != NULL) {
		LOG("adsp: SMP2P item %u already exists: the ADSP is not cold, not touching it", ADSP_ITEM_SMP2P_IN);
		return false;
	}
	mdt_size = fw_all[fi].size;
	if (mdt_size <= sizeof(struct elf32_ehdr) || mdt_size > 0x100000 ||
	    (mdt = (uint8_t *)IOMalloc(mdt_size)) == NULL || !fw_all_read(fi, 0, mdt, mdt_size)) {
		LOG("adsp: adsp.mdt (%llu bytes) cannot be read", mdt_size);
		goto out;
	}
	{
		struct elf32_ehdr *eh = (struct elf32_ehdr *)mdt;
		if (memcmp(eh->e_ident, "\x7f" "ELF", 4) != 0 ||
		    eh->e_phoff + (uint64_t)eh->e_phnum * sizeof(struct elf32_phdr) > mdt_size) {
			LOG("adsp: adsp.mdt: bad ELF header");
			goto out;
		}
		struct elf32_phdr *ph = (struct elf32_phdr *)(mdt + eh->e_phoff);
		for (uint32_t i = 0; i < eh->e_phnum; i++) {
			if ((ph[i].p_flags & MDT_TYPE_MASK) == MDT_TYPE_HASH) {
				hash = i;
			}
			if (!loadable(&ph[i])) {
				continue;
			}
			relocate |= (ph[i].p_flags & MDT_RELOCATABLE) != 0;
			min_a = MIN(min_a, ph[i].p_paddr);
			max_a = MAX(max_a, ph[i].p_paddr + ph[i].p_memsz);
		}
		LOG("adsp: phdrs=%u hash=%u region %08x..%08x relocate=%u", eh->e_phnum, hash, min_a, max_a, relocate);
		if (hash == ~0u || min_a < ADSP_CARVEOUT || max_a > ADSP_CARVEOUT + ADSP_CARVEOUT_SIZE) {
			LOG("adsp: unexpected layout (carveout is %08x+%x)", ADSP_CARVEOUT, ADSP_CARVEOUT_SIZE);
			goto out;
		}
		uint64_t hdr = eh->e_phoff + eh->e_phnum * sizeof(struct elf32_phdr);
		uint64_t meta_size = hdr + ph[hash].p_filesz;
		meta = (uint8_t *)pool_alloc(meta_size, &meta_pa);
		if (meta == NULL) {
			LOG("adsp: metadata: no %llu bytes in the pool", meta_size);
			goto out;
		}
		memcpy(meta, mdt, hdr);
		if (ph[hash].p_offset + ph[hash].p_filesz <= mdt_size) {
			memcpy(meta + hdr, mdt + ph[hash].p_offset, ph[hash].p_filesz);
		} else {
			char hname[24];
			snprintf(hname, sizeof(hname), "image/adsp.b%02u", hash);
			int hi = fw_all_find(hname);
			if (hi < 0 || fw_all[hi].size < ph[hash].p_filesz || !fw_all_read(hi, 0, meta + hdr, ph[hash].p_filesz)) {
				LOG("adsp: hash segment %s cannot be read", hname);
				goto out;
			}
		}
		mb();
		st = scm(SCM_FN(SCM_SVC_PIL, PIL_INIT_IMAGE), 2 | (SCM_ARG_RW << 6), PAS_ID_ADSP, meta_pa, 0, &res);
		LOG("adsp: init_image: ret=%llx res=%llx (meta %llu bytes @%llx)", st, res, meta_size, meta_pa);
		if (st != 0 || res != 0) {
			goto out;
		}
		if (relocate) {
			st = scm(SCM_FN(SCM_SVC_PIL, PIL_MEM_SETUP), 3, PAS_ID_ADSP, min_a,
			    ((uint64_t)(max_a - min_a) + 0xfff) & ~0xfffull, &res);
			LOG("adsp: mem_setup: ret=%llx res=%llx", st, res);
			if (st != 0 || res != 0) {
				goto shutdown;
			}
		}
		for (uint32_t i = 0; i < eh->e_phnum; i++) {
			if (!loadable(&ph[i])) {
				continue;
			}
			uint64_t pa = ph[i].p_paddr & ~0x3fffull, off = ph[i].p_paddr - pa;
			uint64_t len = (off + ph[i].p_memsz + 0x3fff) & ~0x3fffull;
			IOMemoryMap *m = map_phys(pa, len, true);
			if (m == NULL) {
				LOG("adsp: segment %u: cannot map %08x+%x", i, ph[i].p_paddr, ph[i].p_memsz);
				goto shutdown;
			}
			uint8_t *dst = (uint8_t *)m->getVirtualAddress() + off;
			uint64_t got = 0;
			if (ph[i].p_filesz != 0) {
				char name[24];
				snprintf(name, sizeof(name), "image/adsp.b%02u", i);
				int si = fw_all_find(name);
				if (si < 0 || fw_all[si].size < ph[i].p_filesz || !fw_all_read(si, 0, dst, ph[i].p_filesz)) {
					LOG("adsp: %s cannot be read (%u bytes)", name, ph[i].p_filesz);
					m->release();
					goto shutdown;
				}
				got = ph[i].p_filesz;
			}
			if (ph[i].p_memsz > got) {
				bzero(dst + got, ph[i].p_memsz - got);
			}
			mb();
			m->release();
		}
		LOG("adsp: segments loaded");
		st = scm(SCM_FN(SCM_SVC_PIL, PIL_AUTH_RESET), 1, PAS_ID_ADSP, 0, 0, &res);
		LOG("adsp: auth_and_reset: ret=%llx res=%llx resumes=%u", st, res, scm_resumes);
		if (st != 0 || res != 0) {
			goto shutdown;
		}
		ok = true;
		goto out;
	}
shutdown:
	st = scm(SCM_FN(SCM_SVC_PIL, PIL_SHUTDOWN), 1, PAS_ID_ADSP, 0, 0, &res);
	LOG("adsp: shutdown after failure: ret=%llx res=%llx", st, res);
out:
	if (mdt != NULL) {
		IOFree(mdt, mdt_size);
	}
	return ok;
}

/* ---------------- the ADSP's GLINK edge: the modem's protocol, with more than one channel */

struct a_chan {
	const char *name;
	uint16_t lcid;
	uint32_t rcid;
	bool remote_open, open_sent, open_acked, intents_sent, intent_req_sent;
	struct { uint32_t id, size; bool used; } ri[32];
	unsigned ri_count;
	uint8_t buf[OUR_INTENT_SIZE];
	uint32_t len;
	struct { void *p; uint32_t len; } q[128];   /* 0263: 128 (was 32): sensor streams */
	unsigned qh, qt, drops;
	void (*rx)(const uint8_t *p, uint32_t len);
};
static void a_qrtr_rx(const uint8_t *p, uint32_t len);
static void a_gpr_rx(const uint8_t *p, uint32_t len);
static void a_frpc_rx(const uint8_t *p, uint32_t len);
static struct a_chan a_ch[] = {
	{ "IPCRTR", 1 }, { "adsp_apps", 2 }, { "fastrpcglink-apps-dsp", 3 },  /* 0265: FastRPC (the sensors PD) */
};
#define A_NCH (sizeof(a_ch) / sizeof(a_ch[0]))
static volatile uint32_t *a_desc;
static uint8_t *a_tx, *a_rx;
static uint32_t a_rx_len, a_features = GLINK_FEATURES, a_next_intent = 1;
static unsigned a_rx_msgs, a_tx_msgs;
static bool a_version_done;

static uint32_t
a_rx_avail(void)
{
	uint32_t h = a_desc[3], t = a_desc[2];
	return h >= t ? h - t : a_rx_len - t + h;
}

static void
a_rx_peek(uint32_t off, void *dst, uint32_t n)
{
	uint32_t t = a_desc[2] + off;
	if (t >= a_rx_len) {
		t -= a_rx_len;
	}
	uint32_t n1 = MIN(n, a_rx_len - t);
	memcpy(dst, a_rx + t, n1);
	if (n > n1) {
		memcpy((uint8_t *)dst + n1, a_rx, n - n1);
	}
}

static void
a_rx_advance(uint32_t n)
{
	uint32_t t = a_desc[2] + n;
	if (t >= a_rx_len) {
		t -= a_rx_len;
	}
	mb();
	a_desc[2] = t;
}

static uint32_t
a_tx_put(uint32_t head, const void *src, uint32_t n)
{
	uint32_t n1 = MIN(n, GLINK_TX_SIZE - head);
	memcpy(a_tx + head, src, n1);
	if (n > n1) {
		memcpy(a_tx, (const uint8_t *)src + n1, n - n1);
	}
	head += n;
	return head >= GLINK_TX_SIZE ? head - GLINK_TX_SIZE : head;
}

/*
 * 0263: the ADSP edge is written from two threads (the ADSP thread: GPR, RESUME_TX, servreg; the
 * modem thread: user QMI through qrtr_send) -> a lock for the TX FIFO, and one for the send
 * queues and the ADSP's receive-buffer table (taken first when both are needed).
 */
static IOLock *a_fifo_lock, *a_q_lock;

static IOLock *
a_lock_get(IOLock **l)
{
	if (*l == NULL) {
		IOLock *n = IOLockAlloc();
		if (!OSCompareAndSwapPtr(NULL, n, (void * volatile *)l)) {
			IOLockFree(n);
		}
	}
	return *l;
}

static bool a_tx_write_locked(const void *hdr, uint32_t hlen, const void *data, uint32_t dlen);

static bool
a_tx_write(const void *hdr, uint32_t hlen, const void *data, uint32_t dlen)
{
	IOLock *l = a_lock_get(&a_fifo_lock);
	IOLockLock(l);
	bool r = a_tx_write_locked(hdr, hlen, data, dlen);
	IOLockUnlock(l);
	return r;
}

static bool
a_tx_write_locked(const void *hdr, uint32_t hlen, const void *data, uint32_t dlen)
{
	uint32_t head = 0, tail = 0, avail = 0;
	int n;

	for (n = 0; n < 1000; n++) {
		head = a_desc[1];
		tail = a_desc[0];
		avail = tail <= head ? GLINK_TX_SIZE - head + tail : tail - head;
		avail = avail < 8 ? 0 : avail - 8;
		if (avail >= ((hlen + dlen + 7) & ~7u)) {
			break;
		}
		IODelay(100);
	}
	if (n == 1000) {
		LOG("adsp: glink TX FIFO full (head %x tail %x)", head, tail);
		return false;
	}
	head = a_tx_put(head, hdr, hlen);
	if (dlen != 0) {
		head = a_tx_put(head, data, dlen);
	}
	head = (head + 7) & ~7u;
	if (head >= GLINK_TX_SIZE) {
		head -= GLINK_TX_SIZE;
	}
	mb();
	a_desc[1] = head;
	adsp_kick(ADSP_GLINK_BIT);
	a_tx_msgs++;
	return true;
}

static void
a_cmd(uint16_t cmd, uint16_t p1, uint32_t p2)
{
	struct glink_msg m = { cmd, p1, p2 };
	a_tx_write(&m, sizeof(m), NULL, 0);
}

static void
a_open(struct a_chan *c)
{
	struct { struct glink_msg m; char name[32]; } __attribute__((packed)) req = {};
	uint32_t n = (uint32_t)strlen(c->name) + 1;

	req.m.cmd = CMD_OPEN;
	req.m.p1 = c->lcid;
	req.m.p2 = n;
	memcpy(req.name, c->name, n);
	a_tx_write(&req, (uint32_t)sizeof(struct glink_msg) + ((n + 7) & ~7u), NULL, 0);
	c->open_sent = true;
}

static void
a_intents(struct a_chan *c, uint32_t count, uint32_t size)
{
	struct { struct glink_msg m; uint32_t pair[2 * OUR_INTENTS]; } __attribute__((packed)) m = {};
	m.m.cmd = CMD_INTENT;
	m.m.p1 = c->lcid;
	m.m.p2 = count;
	for (uint32_t i = 0; i < count && i < OUR_INTENTS; i++) {
		m.pair[2 * i] = size;
		m.pair[2 * i + 1] = a_next_intent++;
	}
	a_tx_write(&m, sizeof(struct glink_msg) + 8 * count, NULL, 0);
}

static bool
a_up(const struct a_chan *c)
{
	return c->remote_open && c->open_acked;
}

static bool
a_try_send(struct a_chan *c, const void *data, uint32_t len)
{
	unsigned i;
	for (i = 0; i < c->ri_count; i++) {
		if (!c->ri[i].used && c->ri[i].size >= len) {
			break;
		}
	}
	if (i == c->ri_count) {
		if (!c->intent_req_sent) {
			LOG("adsp: t=%u ms: \"%s\": no receive buffer of %u bytes among the ADSP's %u, asking for one", a_ms, c->name,
			    len, c->ri_count);
			a_cmd(CMD_RX_INTENT_REQ, c->lcid, len);
			c->intent_req_sent = true;
		}
		return false;
	}
	struct glink_data_hdr h = { { CMD_TX_DATA, c->lcid, c->ri[i].id }, len, 0 };
	if (len > 128 || a_verbose) {
		LOG("adsp: t=%u ms: \"%s\": %u bytes into the ADSP's buffer %u (%u bytes)", a_ms, c->name, len, c->ri[i].id,
		    c->ri[i].size);
	}
	c->ri[i].used = true;
	return a_tx_write(&h, sizeof(h), data, len);
}

#define A_TXQ 128

/* a_q_lock held */
static void
a_pump_locked(struct a_chan *c)
{
	while (c->qh != c->qt && a_up(c) && a_try_send(c, c->q[c->qh % A_TXQ].p, c->q[c->qh % A_TXQ].len)) {
		IOFree(c->q[c->qh % A_TXQ].p, c->q[c->qh % A_TXQ].len);
		c->qh++;
	}
}

static void
a_pump(struct a_chan *c)
{
	IOLock *l = a_lock_get(&a_q_lock);
	IOLockLock(l);
	a_pump_locked(c);
	IOLockUnlock(l);
}

/* queued, then sent as the ADSP's receive buffers allow; full: the oldest packet goes (counted) */
static void
a_send(struct a_chan *c, const void *data, uint32_t len)
{
	void *p = IOMalloc(len);
	if (p == NULL) {
		c->drops++;
		return;
	}
	memcpy(p, data, len);
	IOLock *l = a_lock_get(&a_q_lock);
	IOLockLock(l);
	if (c->qt - c->qh >= A_TXQ) {
		IOFree(c->q[c->qh % A_TXQ].p, c->q[c->qh % A_TXQ].len);
		c->qh++;
		c->drops++;
		if (c->drops == 1 || (c->drops % 100) == 0) {
			LOG("adsp: \"%s\": send queue full (%u): the oldest packet dropped, %u so far", c->name, A_TXQ, c->drops);
		}
	}
	c->q[c->qt % A_TXQ].p = p;
	c->q[c->qt % A_TXQ].len = len;
	c->qt++;
	a_pump_locked(c);
	IOLockUnlock(l);
}

static struct a_chan *
a_by_lcid(uint32_t lcid)
{
	for (unsigned i = 0; i < A_NCH; i++) {
		if (a_ch[i].lcid == lcid) {
			return &a_ch[i];
		}
	}
	return NULL;
}

static struct a_chan *
a_by_rcid(uint32_t rcid)
{
	for (unsigned i = 0; i < A_NCH; i++) {
		if (a_ch[i].remote_open && a_ch[i].rcid == rcid) {
			return &a_ch[i];
		}
	}
	return NULL;
}

static bool
a_rx_one(void)
{
	struct glink_msg m;
	uint32_t avail = a_rx_avail(), n;
	struct a_chan *c;

	if (avail < sizeof(m)) {
		return false;
	}
	a_rx_peek(0, &m, sizeof(m));
	a_rx_msgs++;
	if (a_verbose && m.cmd != CMD_TX_DATA) {
		LOG("adsp: t=%u ms: glink rx cmd %u %x %x", a_ms, m.cmd, m.p1, m.p2);
	}
	switch (m.cmd) {
	case CMD_VERSION:
		a_rx_advance(8);
		LOG("adsp: t=%u ms: glink VERSION %u features %x", a_ms, m.p1, m.p2);
		if (m.p1 != 0) {
			a_features &= m.p2;
			a_cmd(CMD_VERSION_ACK, 1, a_features);
			a_version_done = true;
		}
		break;
	case CMD_VERSION_ACK:
		a_rx_advance(8);
		LOG("adsp: t=%u ms: glink VERSION_ACK %u features %x", a_ms, m.p1, m.p2);
		if (m.p1 != 0 && m.p2 != a_features) {
			a_features &= m.p2;
			a_cmd(CMD_VERSION, 1, a_features);
		}
		a_version_done = true;
		break;
	case CMD_OPEN: {
		char name[33] = {};
		n = (8 + m.p2 + 7) & ~7u;
		if (avail < n) {
			return false;
		}
		a_rx_peek(8, name, MIN(m.p2, 32u));
		a_rx_advance(n);
		c = NULL;
		for (unsigned i = 0; i < A_NCH; i++) {
			if (strcmp(name, a_ch[i].name) == 0) {
				c = &a_ch[i];
			}
		}
		LOG("adsp: t=%u ms: glink OPEN \"%s\" rcid %u%s", a_ms, name, m.p1, c != NULL ? "" : " (not ours)");
		if (c != NULL) {
			c->rcid = m.p1;
			c->remote_open = true;
			a_cmd(CMD_OPEN_ACK, (uint16_t)c->rcid, 0);
			if (!c->open_sent) {
				a_open(c);
			}
		}
		break;
	}
	case CMD_OPEN_ACK:
		a_rx_advance(8);
		if ((c = a_by_lcid(m.p1)) != NULL) {
			c->open_acked = true;
			LOG("adsp: t=%u ms: glink OPEN_ACK \"%s\"", a_ms, c->name);
		}
		break;
	case CMD_CLOSE:
		a_rx_advance(8);
		LOG("adsp: t=%u ms: glink CLOSE rcid %u", a_ms, m.p1);
		a_cmd(CMD_CLOSE_ACK, m.p1, 0);
		if ((c = a_by_rcid(m.p1)) != NULL) {
			c->remote_open = false;
		}
		break;
	case CMD_INTENT:
		n = (8 + 8 * m.p2 + 7) & ~7u;
		if (avail < n) {
			return false;
		}
		c = a_by_rcid(m.p1);
		IOLockLock(a_lock_get(&a_q_lock));
		for (uint32_t i = 0; i < m.p2; i++) {
			uint32_t pair[2];
			a_rx_peek(8 + 8 * i, pair, 8);
			if (c != NULL && c->ri_count < 32) {
				c->ri[c->ri_count].size = pair[0];
				c->ri[c->ri_count].id = pair[1];
				c->ri[c->ri_count].used = false;
				c->ri_count++;
			}
		}
		IOLockUnlock(a_lock_get(&a_q_lock));
		a_rx_advance(n);
		if (c != NULL) {
			c->intent_req_sent = false;
		}
		break;
	case CMD_RX_DONE:
	case CMD_RX_DONE_W_REUSE:
		a_rx_advance(8);
		if ((c = a_by_rcid(m.p1)) != NULL) {
			IOLockLock(a_lock_get(&a_q_lock));
			for (unsigned i = 0; i < c->ri_count; i++) {
				if (c->ri[i].id == m.p2) {
					if (m.cmd == CMD_RX_DONE_W_REUSE) {
						c->ri[i].used = false;
					} else {
						c->ri[i] = c->ri[--c->ri_count];
					}
					break;
				}
			}
			IOLockUnlock(a_lock_get(&a_q_lock));
		}
		break;
	case CMD_RX_INTENT_REQ:
		a_rx_advance(8);
		if ((c = a_by_rcid(m.p1)) != NULL) {
			a_intents(c, 1, MAX(m.p2, OUR_INTENT_SIZE));
			a_cmd(CMD_RX_INTENT_REQ_ACK, c->lcid, 1);
		}
		break;
	case CMD_RX_INTENT_REQ_ACK:
		a_rx_advance(8);
		LOG("adsp: t=%u ms: glink intent request %s (rcid %u)", a_ms, m.p2 ? "granted" : "refused", m.p1);
		if (m.p2 == 0 && (c = a_by_rcid(m.p1)) != NULL) {
			c->intent_req_sent = false;
		}
		break;
	case CMD_CLOSE_ACK:
	case CMD_SIGNALS:
		a_rx_advance(8);
		break;
	case CMD_READ_NOTIF:
		a_rx_advance(8);
		adsp_kick(ADSP_GLINK_BIT);
		break;
	case CMD_TX_DATA:
	case CMD_TX_DATA_CONT: {
		uint32_t hdr[2];
		if (avail < 16) {
			return false;
		}
		a_rx_peek(8, hdr, 8);
		n = (16 + hdr[0] + 7) & ~7u;
		if (avail < n) {
			return false;
		}
		c = a_by_rcid(m.p1);
		if (c != NULL && m.cmd == CMD_TX_DATA) {
			c->len = 0;
		}
		if (c != NULL && c->len + hdr[0] <= sizeof(c->buf)) {
			a_rx_peek(16, c->buf + c->len, hdr[0]);
			c->len += hdr[0];
		}
		a_rx_advance(n);
		if (hdr[1] == 0 && c != NULL) {
			if (c->rx != NULL) {
				c->rx(c->buf, c->len);
			}
			if (a_features & 1) {
				a_cmd(CMD_RX_DONE_W_REUSE, c->lcid, m.p2);
			} else {
				a_cmd(CMD_RX_DONE, c->lcid, m.p2);
				a_intents(c, 1, OUR_INTENT_SIZE);
			}
		}
		break;
	}
	default:
		LOG("adsp: t=%u ms: glink unknown cmd %u (%x %x), RX stopped", a_ms, m.cmd, m.p1, m.p2);
		a_rx = NULL;
		return false;
	}
	return true;
}

static bool
a_glink_start(struct smem_part_hdr *part)
{
	bool existed = smem_get(part, SMEM_GLINK_DESC, NULL) != NULL;

	a_ch[0].rx = a_qrtr_rx;
	a_ch[1].rx = a_gpr_rx;
	a_ch[2].rx = a_frpc_rx;
	a_desc = (volatile uint32_t *)smem_alloc(part, SMEM_GLINK_DESC, 32);
	a_tx = (uint8_t *)smem_alloc(part, SMEM_GLINK_FIFO_TX, GLINK_TX_SIZE);
	if (a_desc == NULL || a_tx == NULL) {
		LOG("adsp: glink: SMEM alloc failed (desc %p tx %p)", a_desc, a_tx);
		return false;
	}
	LOG("adsp: glink: desc %s, our TX FIFO %u bytes", existed ? "made by the ADSP" : "ours", GLINK_TX_SIZE);
	a_desc[2] = 0;
	a_desc[1] = 0;
	mb();
	a_cmd(CMD_VERSION, 1, a_features);
	return true;
}

/* ---------------- QRTR towards the ADSP (its own node; qrtr_send() routes by node) */

static uint32_t a_node = QRTR_NODE_BCAST, a_qver = 1;
static bool a_hello_rx, a_hello_sent, a_announced;
static unsigned a_srv_count;

static bool
adsp_qrtr_send(uint32_t dst_node, const uint8_t *pkt, uint32_t len)
{
	if (a_node == QRTR_NODE_BCAST || dst_node != a_node) {
		return false;
	}
	a_send(&a_ch[0], pkt, len);
	return true;
}

static void
a_qrtr_rx(const uint8_t *p, uint32_t len)
{
	uint32_t type, src_node, src_port, dst_port, size, hlen, confirm;

	if (len >= sizeof(struct qrtr_hdr_v1) && *(const uint32_t *)p == 1) {
		const struct qrtr_hdr_v1 *h = (const struct qrtr_hdr_v1 *)p;
		type = h->type; src_node = h->src_node; src_port = h->src_port;
		dst_port = h->dst_port; size = h->size; hlen = sizeof(*h); confirm = h->confirm_rx;
		a_qver = 1;
	} else if (len >= sizeof(struct qrtr_hdr_v2) && p[0] == 2) {
		const struct qrtr_hdr_v2 *h = (const struct qrtr_hdr_v2 *)p;
		type = h->type; src_node = h->src_node; src_port = h->src_port;
		dst_port = h->dst_port; size = h->size; hlen = sizeof(*h) + h->optlen * 4;
		src_port = src_port == 0xFFFE ? QRTR_PORT_CTRL : src_port;
		dst_port = dst_port == 0xFFFE ? QRTR_PORT_CTRL : dst_port;
		confirm = h->flags & 1;
		a_qver = 2;
	} else {
		LOG("adsp: t=%u ms: qrtr: bad packet, %u bytes", a_ms, len);
		return;
	}
	const struct qrtr_ctrl *c = (const struct qrtr_ctrl *)(p + hlen);

	switch (type) {
	case QRTR_TYPE_HELLO:
		a_node = src_node;
		a_hello_rx = true;
		a_hello_sent = false;
		LOG("adsp: t=%u ms: qrtr v%u HELLO from node %u", a_ms, a_qver, src_node);
		break;
	case QRTR_TYPE_NEW_SERVER:
		a_srv_count++;
		if (c->a != 0) {
			/* 0263: in the lookup table too (SWEET_QMI_LOOKUP), like the modem's: e.g. SNS_CLIENT 0x190 */
			bool known = false;
			for (unsigned k = 0; k < q_srv_count; k++) {
				known |= q_srv[k].svc == c->a && q_srv[k].inst == c->b && q_srv[k].node == c->c && q_srv[k].port == c->d;
			}
			if (!known && q_srv_count < sizeof(q_srv) / sizeof(q_srv[0])) {
				q_srv[q_srv_count].svc = c->a;
				q_srv[q_srv_count].inst = c->b;
				q_srv[q_srv_count].node = c->c;
				q_srv[q_srv_count].port = c->d;
				q_srv_count++;
			}
			LOG("adsp: t=%u ms: service %x:%x node %u port %x %s", a_ms, c->a, c->b, c->c, c->d, svc_name(c->a));
		}
		break;
	case QRTR_TYPE_DATA:
		if (confirm) {
			struct qrtr_ctrl r = { QRTR_TYPE_RESUME_TX, QRTR_NODE_APPS, dst_port, 0, 0 };
			qrtr_send(QRTR_TYPE_RESUME_TX, dst_port, src_node, src_port, &r, sizeof(r));
		}
		if (hlen + size <= len && !svc_rx(src_node, src_port, dst_port, p + hlen, size)) {
			LOG("adsp: t=%u ms: qrtr data %u bytes from %u:%x to port %x, not served", a_ms, size, src_node, src_port, dst_port);
		}
		break;
	case 8:                                     /* NEW_LOOKUP: somebody on the ADSP looks for a service */
		LOG("adsp: t=%u ms: qrtr lookup for service %x:%x", a_ms, c->a, c->b);
		break;
	default:
		break;
	}
}

/* ---------------- GPR (AudioReach) */

static unsigned a_gpr_rx_count;
static bool a_spf_ready;
static volatile uint32_t *a_sk;                 /* the ADSP's SMP2P slave-kernel entry (bit 0: fatal) */
static struct {
	bool full;
	uint32_t opcode, token, n, p[8];
} a_box;                                        /* the last answer (audio_test() waits for it) */
static unsigned a_writes_done;
static void a_mic_done(const uint32_t *pl, uint32_t pn);

static void
a_gpr_send(uint32_t dst_port, uint32_t opcode, uint32_t token, const void *payload, uint32_t len)
{
	uint8_t buf[256];
	uint32_t w[6];

	if (len > sizeof(buf) - sizeof(w)) {
		return;
	}
	w[0] = 0 | (6u << 4) | ((uint32_t)(sizeof(w) + len) << 8);     /* version 0, 6 header words, size */
	w[1] = GPR_DOMAIN_ADSP | (GPR_DOMAIN_APPS << 8);
	w[2] = GPR_PORT_SPF_CORE;
	w[3] = dst_port;
	w[4] = token;
	w[5] = opcode;
	memcpy(buf, w, sizeof(w));
	if (len != 0) {
		memcpy(buf + sizeof(w), payload, len);
	}
	a_send(&a_ch[1], buf, sizeof(w) + len);
}

static void
a_gpr_rx(const uint8_t *p, uint32_t len)
{
	uint32_t w[6], hsize;

	if (len < sizeof(w)) {
		LOG("adsp: t=%u ms: gpr: short packet, %u bytes", a_ms, len);
		return;
	}
	memcpy(w, p, sizeof(w));
	hsize = ((w[0] >> 4) & 0xf) * 4;
	const uint32_t *pl = (const uint32_t *)(p + MIN(hsize, len));
	uint32_t pn = len > hsize ? (len - hsize) / 4 : 0;
	a_gpr_rx_count++;
	if (a_gpr_rx_count <= 40) {
		LOG("adsp: t=%u ms: gpr from %u:%x to %u:%x token %x opcode %08x, %u bytes: %08x %08x %08x %08x", a_ms,
		    (w[1] >> 8) & 0xff, w[2], w[1] & 0xff, w[3], w[4], w[5], len, pn > 0 ? pl[0] : 0, pn > 1 ? pl[1] : 0,
		    pn > 2 ? pl[2] : 0, pn > 3 ? pl[3] : 0);
	}
	if (w[5] == 0x05001005u) {                              /* a buffer the record graph filled (mic_service()) */
		a_mic_done(pl, pn);
		return;
	}
	if (w[5] == 0x05001004u || w[5] == 0x05001000u) {       /* a buffer back from the shared memory endpoint */
		a_writes_done++;
		if (a_writes_done <= 2 || (pn > 3 && pl[3] != 0)) {
			LOG("adsp: t=%u ms: buffer %08x back, status %x", a_ms, pn > 0 ? pl[0] : 0, pn > 3 ? pl[3] : 0);
		}
		return;
	}
	a_box.opcode = w[5];
	a_box.token = w[4];
	a_box.n = MIN(pn, 8u);
	for (uint32_t i = 0; i < a_box.n; i++) {
		a_box.p[i] = pl[i];
	}
	a_box.full = true;
	if (w[5] == APM_CMD_RSP_GET_SPF_STATE && pn > 0 && pl[0] == 1 && !a_spf_ready) {
		a_spf_ready = true;
		LOG("adsp: t=%u ms: *** AudioReach SPF is ready ***", a_ms);
	}
}

/* ---------------- audio=2: the playback path, step by step, with everything logged */

/*
 * What Linux's q6prm / q6apm / audioreach.c do for a playback, written out for one graph:
 *   PRM: vote the LPASS core and the digital codec, ask for the RX macro's clocks (22.5792 MHz,
 *        the device tree's rx_core_clk / rx_npl_clk, ids 0x30e / 0x30f);
 *   the codec's registers read back (RX macro 0x0a600000, its SoundWire master 0x0a610000);
 *   the speaker amplifiers looked for on I2C (QUP0 SE1: fs16xx 0x34/0x35, aw87xxx 0x58/0x5b,
 *        reset GPIOs 99 and 105 raised);
 *   APM: a buffer in our pool mapped (SMMU stream 0x1c1 identity), one graph opened --
 *        stream: WR_SHARED_MEM_EP -> PCM_DEC -> PCM_CNV, device: MFC -> CODEC_DMA_SINK on
 *        RX_CODEC_DMA_RX_1 (RXTX type 1, index 2), the Android speaker path's backend --
 *        formats set, prepared, started, and 3 s of a 1 kHz tone written; the ADSP returning
 *        the buffers at 48 kHz says the DMA runs. The codec itself (RX macro paths, SoundWire,
 *        WCD937x, the amplifier) is the next step: nothing is heard yet.
 */
#define GPR_PORT_APM_CLIENT 1                   /* GPR_APM_MODULE_IID: Linux's source for APM commands */
#define GPR_PORT_PRM_CLIENT 7                   /* the device tree's q6prm, reg 7 */
#define GPR_PORT_GRAPH      0x10000001u         /* a graph's own port (Linux: dynamic, from 0x10000000) */
#define PRM_PORT            2
#define PRM_CMD_REQUEST_HW_RSC     0x0100100Fu
#define PRM_CMD_RSP_REQUEST_HW_RSC 0x02001002u
#define PARAM_ID_RSC_HW_CORE       0x08001032u
#define PARAM_ID_RSC_AUDIO_HW_CLK  0x0800102Cu
#define APM_CMD_GRAPH_OPEN         0x01001000u
#define APM_CMD_GRAPH_PREPARE      0x01001001u
#define APM_CMD_GRAPH_START        0x01001002u
#define APM_CMD_GRAPH_STOP         0x01001003u
#define APM_CMD_GRAPH_CLOSE        0x01001004u
#define APM_CMD_SET_CFG            0x01001006u
#define APM_CMD_SHARED_MEM_MAP_REGIONS     0x0100100Cu
#define APM_CMD_RSP_SHARED_MEM_MAP_REGIONS 0x02001001u
#define DATA_CMD_WR_SH_MEM_EP_DATA_BUFFER_V2      0x0400100Au
#define DATA_CMD_RSP_WR_SH_MEM_EP_DATA_BUFFER_DONE_V2 0x05001004u
#define A_SG        0x4001u
#define A_SG_DEV    0x4002u
#define A_IOVA      0x10000000ull               /* where the ADSP sees our buffers (smmu_s1) */
#define A_SID_BITS  (1ull << 32)                /* (0x1c1 & 0xf) << 32: msm_audio_ion's smmu_sid_bits */
#define A_CONT_STR  0x5001u
#define A_CONT_DEV  0x5002u
#define A_WRSH      0x6001u
#define A_PCMDEC    0x6002u
#define A_PCMCNV    0x6003u
#define A_MFC       0x6004u
#define A_CDMA      0x6005u
#define A_RATE      48000u
#define A_PERIOD    480u                        /* samples: 10 ms */
#define A_NBUF      8u

static uint32_t a_mem_handle;

/* one pass over the edge: what came in, what waits to go out */
static void
a_service(void)
{
	for (int i = 0; i < 64 && a_rx != NULL && a_rx_one(); i++) {
	}
	for (unsigned i = 0; i < A_NCH; i++) {
		struct a_chan *c = &a_ch[i];
		if (a_version_done && !c->open_sent) {
			a_open(c);
		}
		if (a_up(c) && !c->intents_sent) {
			a_intents(c, OUR_INTENTS, OUR_INTENT_SIZE);
			c->intents_sent = true;
			LOG("adsp: t=%u ms: \"%s\" open both ways", a_ms, c->name);
		}
		a_pump(c);
	}
}

/* any GPR packet: header + payload, src and dst ports given */
static void
a_gpr_pkt(uint32_t src, uint32_t dst, uint32_t opcode, uint32_t token, const void *payload, uint32_t len)
{
	uint32_t w[6];
	uint8_t *buf = (uint8_t *)IOMalloc(sizeof(w) + len);

	if (buf == NULL) {
		return;
	}
	w[0] = 0 | (6u << 4) | ((uint32_t)(sizeof(w) + len) << 8);
	w[1] = GPR_DOMAIN_ADSP | (GPR_DOMAIN_APPS << 8);
	w[2] = src;
	w[3] = dst;
	w[4] = token;
	w[5] = opcode;
	memcpy(buf, w, sizeof(w));
	if (len != 0) {
		memcpy(buf + sizeof(w), payload, len);
	}
	a_send(&a_ch[1], buf, (uint32_t)sizeof(w) + len);
	IOFree(buf, sizeof(w) + len);
}

/*
 * Wait for the answer to `cmd`: a GPR_BASIC_RSP_RESULT naming it (returns its status), or a
 * packet with opcode `rsp` (returns 0, payload in a_box). -1 after `ms` without one.
 */
/*
 * 0181/0182: the ADSP's first read of our memory resets the phone before the USB log gets out.
 * Breadcrumbs, then, in the IMEM (0183's place in our pool did not survive: the next boot's
 * download goes there, it seems): while a
 * command waits, every change of the SMMU bank's and the SMMU's fault status and of the ADSP's
 * state is written there, and the next boot's audio test prints what the last one left.
 */
#define IMEM_PA     0x0C125000ull               /* qcom,msm-imem: 4 KB of on-chip RAM a reset keeps */
#define CRUMB_OFF   0xC00                       /* its last KB, which no listed user takes */
#define CRUMB_MAGIC 0x44524241u                 /* "ABRD" */
struct crumb { uint32_t ms, stage, fsr, far_lo, far_hi, gfsr, sk, cmd; };
struct crumbs { uint32_t magic, boot, n, pad; struct crumb e[31]; };
static volatile struct crumbs *a_crumbs;
static uint32_t a_stage;

static void
crumb(bool force)
{
	if (a_crumbs == NULL || a_crumbs->n >= 31) {
		return;
	}
	struct crumb c = { a_ms, a_stage, s1_cbp ? r32(s1_cbp, 0x58) : 0, s1_cbp ? r32(s1_cbp, 0x60) : 0,
		s1_cbp ? r32(s1_cbp, 0x64) : 0, s1_smmu ? r32(s1_smmu, 0x48) : 0, a_sk ? *a_sk : 0, 0 };
	uint32_t n = a_crumbs->n;
	if (!force && n != 0) {
		volatile struct crumb *l = &a_crumbs->e[n - 1];
		if (l->stage == c.stage && l->fsr == c.fsr && l->gfsr == c.gfsr && l->sk == c.sk && c.ms - l->ms < 100) {
			return;
		}
	}
	a_crumbs->e[n].ms = c.ms;
	a_crumbs->e[n].stage = c.stage;
	a_crumbs->e[n].fsr = c.fsr;
	a_crumbs->e[n].far_lo = c.far_lo;
	a_crumbs->e[n].far_hi = c.far_hi;
	a_crumbs->e[n].gfsr = c.gfsr;
	a_crumbs->e[n].sk = c.sk;
	mb();
	a_crumbs->n = n + 1;
	mb();
}

static void
crumbs_start(void)
{
	volatile uint8_t *imem = (volatile uint8_t *)ml_io_map(IMEM_PA, 0x1000);
	if (imem == NULL) {
		return;
	}
	LOG("adsp: IMEM restart reason %08x, dload type %08x", *(volatile uint32_t *)(imem + 0x65c),
	    *(volatile uint32_t *)(imem + 0x1c));
	a_crumbs = (volatile struct crumbs *)(imem + CRUMB_OFF);
	if (a_crumbs->magic == CRUMB_MAGIC && a_crumbs->n <= 31) {
		LOG("adsp: the last boot's breadcrumbs (%u):", a_crumbs->n);
		for (uint32_t i = 0; i < a_crumbs->n; i++) {
			volatile struct crumb *c = &a_crumbs->e[i];
			LOG("adsp:   t=%u ms stage %u: bank FSR %08x FAR %08x%08x, SMMU GFSR %08x, ADSP %08x", c->ms, c->stage, c->fsr,
			    c->far_hi, c->far_lo, c->gfsr, c->sk);
		}
		IOSleep(300);
	} else {
		LOG("adsp: no breadcrumbs from the last boot");
	}
	a_crumbs->magic = CRUMB_MAGIC;
	a_crumbs->n = 0;
	mb();
}

static int64_t
a_wait(uint32_t cmd, uint32_t rsp, unsigned ms)
{
	for (unsigned t = 0; t < ms; t++) {
		a_service();
		crumb(t == 0);
		if (a_sk != NULL && (*a_sk & 1)) {
			uint32_t sz = 0;
			const char *why = (const char *)smem_get(smem_partition(SMEM_GLOBAL_HOST, SMEM_GLOBAL_HOST), 423, &sz);
			char reason[160] = {};
			if (why != NULL) {
				memcpy(reason, why, MIN(sz, (uint32_t)sizeof(reason) - 1));
			}
			LOG("adsp: *** the ADSP crashed waiting for %08x: %s ***", cmd, why != NULL ? reason : "(no reason)");
			IOSleep(500);
			return -3;
		}
		if (a_box.full) {
			a_box.full = false;
			if (rsp != 0 && a_box.opcode == rsp) {
				return 0;
			}
			if (a_box.opcode == GPR_BASIC_RSP_RESULT && a_box.n >= 2 && a_box.p[0] == cmd) {
				return a_box.p[1];
			}
			LOG("adsp: waiting for %08x: got %08x (%08x %08x)", cmd, a_box.opcode, a_box.p[0], a_box.p[1]);
		}
		IOSleep(1);
		a_ms++;
	}
	while (a_ch[1].qh != a_ch[1].qt) {                      /* what never went out stays out */
		LOG("adsp: dropping a %u-byte packet that found no receive buffer", a_ch[1].q[a_ch[1].qh % 32].len);
		IOFree(a_ch[1].q[a_ch[1].qh % 32].p, a_ch[1].q[a_ch[1].qh % 32].len);
		a_ch[1].qh++;
	}
	a_ch[1].intent_req_sent = false;
	return -1;
}

/* an in-band APM-style command: GPR header, apm_cmd_header (payload size), payload */
/*
 * An APM-style command: GPR header, apm_cmd_header, payload. On adsp_apps the ADSP's receive
 * buffers are 128 bytes (0168: 32 of them), and a 392-byte graph open was never answered even
 * with a bigger buffer granted; the header can instead name the payload in shared memory
 * (payload address, the map's handle, size), which is how anything bigger than a few
 * parameters goes once the command area is mapped (a_cmd_area, after the tone's buffers).
 */
static uint8_t *a_cmd_area;
/*
 * 0170: a bigger receive buffer asked for on adsp_apps is granted (RX_INTENT_REQ_ACK) but never
 * advertised, so a packet over 128 bytes waits for ever (and everything queued behind it):
 * payloads that would not fit go out of band.
 */
static bool a_oob, a_oob_force;
static uint64_t a_cmd_pa;

static int64_t
a_cmd_sync(uint32_t src, uint32_t dst, uint32_t opcode, const void *payload, uint32_t len, uint32_t rsp)
{
	uint32_t n = 16 + len;
	uint8_t *b;
	int64_t r;

	if ((a_oob_force || (a_oob && 24 + 16 + len > 128)) && a_cmd_area != NULL && a_mem_handle != 0 && len <= 4096) {
		uint32_t h[4] = { (uint32_t)a_cmd_pa, (uint32_t)(a_cmd_pa >> 32), a_mem_handle, len };
		memcpy(a_cmd_area, payload, len);
		mb();
		a_stage = opcode & 0xFFFF;              /* (in the breadcrumbs: which command was out) */
		crumb(true);
		a_gpr_pkt(src, dst, opcode, 0, h, sizeof(h));
		return a_wait(opcode, rsp, 2000);
	}
	if ((b = (uint8_t *)IOMalloc(n)) == NULL) {
		return -1;
	}
	bzero(b, 16);
	memcpy(b + 12, &len, 4);
	memcpy(b + 16, payload, len);
	a_gpr_pkt(src, dst, opcode, 0, b, n);
	IOFree(b, n);
	r = a_wait(opcode, rsp, 2000);
	return r;
}

/* builder for parameter blocks: module_instance_id, param_id, param_size, error_code + data, 8-aligned */
struct a_pb { uint8_t b[1536]; uint32_t n; };

static uint8_t *
a_param(struct a_pb *pb, uint32_t iid, uint32_t pid, uint32_t size)
{
	uint32_t total = (16 + size + 7) & ~7u;
	uint32_t h[4] = { iid, pid, total - 16, 0 };
	uint8_t *p;

	if (pb->n + total > sizeof(pb->b)) {
		return NULL;
	}
	p = pb->b + pb->n;
	bzero(p, total);
	memcpy(p, h, 16);
	pb->n += total;
	return p + 16;
}

static void
a_put32(uint8_t **p, uint32_t v)
{
	memcpy(*p, &v, 4);
	*p += 4;
}

static void
a_put16(uint8_t **p, uint16_t v)
{
	memcpy(*p, &v, 2);
	*p += 2;
}

/*
 * 0177: the ADSP cannot read our pool (the out-of-band graph open reset the phone from its end
 * too), and kernel memory is above 4 GB. Everything goes in band, then, one parameter block a
 * packet so that each fits the ADSP's 128-byte receive buffers: APM takes a graph open (or a
 * SET_CFG) in pieces, each naming what an earlier one created.
 */
static int64_t __attribute__((unused))
a_cmd_each(uint32_t src, uint32_t dst, uint32_t opcode, const struct a_pb *pb, const char *what)
{
	int64_t first = 0;
	unsigned n = 0;

	for (uint32_t at = 0; at + 16 <= pb->n; n++) {
		uint32_t h[4];
		memcpy(h, pb->b + at, 16);
		uint32_t len = 16 + h[2];
		int64_t r = a_cmd_sync(src, dst, opcode, pb->b + at, len, 0);
		LOG("adsp: %s, block %u (module %x, param %08x, %u bytes) -> %lld", what, n, h[0], h[1], len, r);
		if (r != 0 && first == 0) {
			first = r;
		}
		at += len;
	}
	return first;
}

static int64_t
a_prm(uint32_t pid, const uint32_t *v, unsigned n)
{
	struct a_pb pb = {};
	uint8_t *p = a_param(&pb, PRM_PORT, pid, 4 * n);
	for (unsigned i = 0; i < n; i++) {
		a_put32(&p, v[i]);
	}
	int64_t r = a_cmd_sync(GPR_PORT_PRM_CLIENT, PRM_PORT, PRM_CMD_REQUEST_HW_RSC, pb.b, pb.n, PRM_CMD_RSP_REQUEST_HW_RSC);
	LOG("adsp: PRM %08x %x: %lld, answer %08x: %08x %08x %08x", pid, v[n > 1 ? 1 : 0], r, a_box.opcode, a_box.p[0],
	    a_box.p[1], a_box.p[2]);
	return r;
}

/*
 * A graph is opened in two parts, as Linux does (the stream's subgraph, then the device's with the
 * connection between them): all of it at once is ~600 bytes, more than the 512-byte receive
 * buffers the ADSP offers on adsp_apps, and that open was never answered.
 */
static const struct { uint32_t sg, cont, mid, iid, ip, op; } a_mods[] = {
	{ A_SG,     A_CONT_STR, 0x07001000, A_WRSH, 0, 1 },        /* WR_SHARED_MEM_EP */
	{ A_SG,     A_CONT_STR, 0x07001005, A_PCMDEC, 1, 1 },      /* PCM_DEC */
	{ A_SG,     A_CONT_STR, 0x07001003, A_PCMCNV, 1, 1 },      /* PCM_CNV */
	{ A_SG_DEV, A_CONT_DEV, 0x07001015, A_MFC, 1, 1 },         /* MFC */
	{ A_SG_DEV, A_CONT_DEV, 0x07001023, A_CDMA, 1, 0 },        /* CODEC_DMA_SINK */
};

static uint32_t a_mask = ~0u;                   /* which of a_mods a graph open takes (a_graph_probe()) */

static bool
a_in(unsigned i, uint32_t sg)
{
	return a_mods[i].sg == sg && ((a_mask >> i) & 1) != 0;
}

/*
 * A graph open in band, in packets that fit the ADSP's 128-byte receive buffers (0178/0179:
 * the subgraph alone is taken; a container sent alone, with no module, makes the ADSP's audio
 * process die in that container's thread):
 *   1. the subgraph's configuration;
 *   2. the container (its type only: 48 bytes) with the first module's list entry (40 bytes);
 *   3. each further module's list entry into the same container;
 *   4. each module's properties (its ports);
 *   5. the connections.
 */
/*
 * One subgraph a graph open, its payload in the shared command page (the ADSP reaches it now, at
 * its IOVA). The container as Android's ACDB has it (e0000123 there): type GC, graph position,
 * stack 4 KB, domain ADSP, heap 1, no parent (0xffffffff), and 0x0800124D = 1 -- 0178-0180: one
 * property alone (or four) made the ADSP's audio process die in the container's thread.
 */
static int64_t
a_graph_open_part(uint32_t sg, uint32_t direction, uint32_t cont, uint32_t cap, uint32_t link_from)
{
	const unsigned all = sizeof(a_mods) / sizeof(a_mods[0]);
	struct a_pb *pb = (struct a_pb *)IOMalloc(sizeof(*pb));
	unsigned nm = 0, nc = 0, last = ~0u;
	uint8_t *p;
	int64_t r;

	if (pb == NULL) {
		return -1;
	}
	for (unsigned i = 0; i < all; i++) {
		if (a_in(i, sg)) {
			nc += last != ~0u || link_from != 0;
			last = i;
			nm++;
		}
	}
	pb->n = 0;
	p = a_param(pb, APM_PORT, 0x08001001, 4 + 44);
	a_put32(&p, 1);
	a_put32(&p, sg); a_put32(&p, 3);
	a_put32(&p, 0x0800100E); a_put32(&p, 4); a_put32(&p, 2);    /* perf mode: low latency */
	a_put32(&p, 0x0800100F); a_put32(&p, 4); a_put32(&p, direction);
	a_put32(&p, 0x08001010); a_put32(&p, 4); a_put32(&p, 1);    /* scenario: playback */
	p = a_param(pb, APM_PORT, 0x08001000, 4 + 8 + 16 + 6 * 12);
	a_put32(&p, 1);
	a_put32(&p, cont); a_put32(&p, 7);
	a_put32(&p, 0x08001011); a_put32(&p, 8); a_put32(&p, 1); a_put32(&p, cap);
	a_put32(&p, 0x08001012); a_put32(&p, 4); a_put32(&p, 1);    /* graph position: stream */
	a_put32(&p, 0x08001013); a_put32(&p, 4); a_put32(&p, 0x1000);
	a_put32(&p, 0x08001014); a_put32(&p, 4); a_put32(&p, 2);    /* processing domain: ADSP */
	a_put32(&p, 0x08001174); a_put32(&p, 4); a_put32(&p, 1);    /* heap: default */
	a_put32(&p, 0x080010CB); a_put32(&p, 4); a_put32(&p, 0xFFFFFFFF);   /* no parent container */
	a_put32(&p, 0x0800124D); a_put32(&p, 4); a_put32(&p, 1);
	p = a_param(pb, APM_PORT, 0x08001002, 4 + 12 + nm * 8);
	a_put32(&p, 1);
	a_put32(&p, sg); a_put32(&p, cont); a_put32(&p, nm);
	for (unsigned i = 0; i < all; i++) {
		if (a_in(i, sg)) {
			a_put32(&p, a_mods[i].mid); a_put32(&p, a_mods[i].iid);
		}
	}
	p = a_param(pb, APM_PORT, 0x08001003, 4 + nm * 24);
	a_put32(&p, nm);
	for (unsigned i = 0; i < all; i++) {
		if (a_in(i, sg)) {
			a_put32(&p, a_mods[i].iid); a_put32(&p, 1);
			a_put32(&p, 0x08001015); a_put32(&p, 8); a_put32(&p, a_mods[i].ip); a_put32(&p, a_mods[i].op);
		}
	}
	p = a_param(pb, APM_PORT, 0x08001004, 4 + nc * 16);
	a_put32(&p, nc);
	last = ~0u;
	for (unsigned i = 0; i < all; i++) {
		if (a_in(i, sg)) {
			uint32_t from = last != ~0u ? a_mods[last].iid : link_from;
			if (from != 0) {
				a_put32(&p, from); a_put32(&p, 1); a_put32(&p, a_mods[i].iid); a_put32(&p, 2);
			}
			last = i;
		}
	}
	r = a_cmd_sync(GPR_PORT_APM_CLIENT, APM_PORT, APM_CMD_GRAPH_OPEN, pb->b, pb->n, 0);
	LOG("adsp: graph open of subgraph %x (%u modules, %u connections, direction %u, container %x, %u bytes) -> %lld", sg,
	    nm, nc, direction, cap, pb->n, r);
	IOFree(pb, sizeof(*pb));
	return r;
}

static int64_t
a_graph_mgmt_list(uint32_t opcode, const uint32_t *sgs, unsigned n)
{
	struct a_pb pb = {};
	uint8_t *p = a_param(&pb, APM_PORT, 0x08001005, 4 + 4 * n);   /* APM_PARAM_ID_SUB_GRAPH_LIST */
	a_put32(&p, n);
	for (unsigned i = 0; i < n; i++) {
		a_put32(&p, sgs[i]);
	}
	return a_cmd_sync(GPR_PORT_APM_CLIENT, APM_PORT, opcode, pb.b, pb.n, 0);
}

static int64_t
a_graph_mgmt(uint32_t opcode)
{
	static const uint32_t both[] = { A_SG, A_SG_DEV };
	return a_graph_mgmt_list(opcode, both, 2);
}

/*
 * 0174 on the phone: the graph open read from shared memory is answered with status 1
 * (AR_EFAILED) for both kinds of container. Which part is refused: the stream's modules one by
 * one, then the device's, each with both container kinds, each closed again when it opens.
 */
static void __attribute__((unused))
a_graph_probe(void)
{
	static const uint32_t caps[][2] = { { 2, 3 }, { 0x0B001001, 0x0B001001 } };
	static const struct { uint32_t mask, sg, dir, cont; } steps[] = {
		{ 0x01, A_SG, 2, A_CONT_STR },          /* WR_SHARED_MEM_EP alone */
		{ 0x03, A_SG, 2, A_CONT_STR },          /* + PCM_DEC */
		{ 0x07, A_SG, 2, A_CONT_STR },          /* + PCM_CNV */
		{ 0x10, A_SG_DEV, 1, A_CONT_DEV },      /* CODEC_DMA_SINK alone, TX */
		{ 0x10, A_SG_DEV, 2, A_CONT_DEV },      /* ... RX */
		{ 0x18, A_SG_DEV, 1, A_CONT_DEV },      /* MFC + CODEC_DMA_SINK */
	};
	for (unsigned c = 0; c < 2; c++) {
		for (unsigned k = 0; k < sizeof(steps) / sizeof(steps[0]); k++) {
			a_mask = steps[k].mask;
			int64_t r = a_graph_open_part(steps[k].sg, steps[k].dir, steps[k].cont,
			    steps[k].sg == A_SG ? caps[c][0] : caps[c][1], 0);
			if (r == 0) {
				(void)a_graph_mgmt_list(APM_CMD_GRAPH_CLOSE, &steps[k].sg, 1);
			}
			IOSleep(150);
		}
	}
	a_mask = ~0u;
}

static int64_t
a_graph_open(uint32_t cap_str, uint32_t cap_dev)
{
	/* the stream's subgraph, then the device's (the template's direction for it: TX) with the link into it */
	int64_t r = a_graph_open_part(A_SG, 2, A_CONT_STR, cap_str, 0);
	if (r != 0) {
		return r;
	}
	return a_graph_open_part(A_SG_DEV, 1, A_CONT_DEV, cap_dev, A_PCMCNV);
}

static int64_t
a_formats(void)
{
	struct a_pb *pb = (struct a_pb *)IOMalloc(sizeof(*pb));
	uint8_t *p;
	int64_t r, r2;

	if (pb == NULL) {
		return -1;
	}
	pb->n = 0;
	/* PCM_DEC (interleaved in) and PCM_CNV (deinterleaved unpacked out): PCM_OUTPUT_FORMAT_CFG */
	for (unsigned m = 0; m < 2; m++) {
		p = a_param(pb, m == 0 ? A_PCMDEC : A_PCMCNV, 0x08001008, 12 + 16 + 1);
		a_put32(&p, 1); a_put32(&p, 0x09001000); a_put32(&p, 20);  /* fixed point, PCM, payload size */
		a_put16(&p, 16); a_put16(&p, 1); a_put16(&p, 16); a_put16(&p, 15);   /* width, LSB aligned, bits, Q15 */
		a_put16(&p, 1); a_put16(&p, m == 0 ? 1 : 3); a_put16(&p, 0); a_put16(&p, 1);   /* LE, interleave, -, 1 ch */
		*p = 1;                                             /* FL */
	}
	/* MFC: 48 kHz, 16 bits, 1 channel */
	p = a_param(pb, A_MFC, 0x08001024, 8 + 2);
	a_put32(&p, A_RATE); a_put16(&p, 16); a_put16(&p, 1); a_put16(&p, 1);
	/* CODEC_DMA_SINK: media format, frame size factor, RX_CODEC_DMA_RX_1, power mode */
	p = a_param(pb, A_CDMA, 0x08001017, 12);
	a_put32(&p, A_RATE); a_put16(&p, 16); a_put16(&p, 1); a_put32(&p, 1);
	p = a_param(pb, A_CDMA, 0x08001018, 4);
	a_put32(&p, 1);
	p = a_param(pb, A_CDMA, 0x08001063, 12);
	a_put32(&p, 1); a_put32(&p, 2); a_put32(&p, 1);              /* RXTX, RX1, channel 0 */
	p = a_param(pb, A_CDMA, 0x08001176, 4);
	a_put32(&p, 0);
	r = a_cmd_sync(GPR_PORT_APM_CLIENT, APM_PORT, APM_CMD_SET_CFG, pb->b, pb->n, 0);
	/* the shared memory endpoint's input format: SET_CFG sent to the module itself */
	pb->n = 0;
	p = a_param(pb, A_WRSH, 0x0800100C, 12 + 16 + 1);
	a_put32(&p, 1); a_put32(&p, 0x09001000); a_put32(&p, 20);
	a_put32(&p, A_RATE); a_put16(&p, 16); a_put16(&p, 1); a_put16(&p, 16); a_put16(&p, 15); a_put16(&p, 1); a_put16(&p, 1);
	*p = 1;
	r2 = a_cmd_sync(GPR_PORT_GRAPH, A_WRSH, APM_CMD_SET_CFG, pb->b, pb->n, 0);
	IOFree(pb, sizeof(*pb));
	LOG("adsp: formats: modules %lld, shared memory endpoint %lld", r, r2);
	return r != 0 ? r : r2;
}

static void
a_write(uint64_t pa, uint32_t bytes, uint32_t token)
{
	uint32_t w[11] = { (uint32_t)pa, (uint32_t)(pa >> 32), a_mem_handle, bytes, 0, 0, 0, 0, 0, 0, 0 };
	a_gpr_pkt(GPR_PORT_GRAPH, A_WRSH, DATA_CMD_WR_SH_MEM_EP_DATA_BUFFER_V2, token, w, sizeof(w));
}

extern "C" int sweet_i2c1_init(void);
extern "C" int sweet_i2c1_read(uint8_t addr, uint8_t reg, uint8_t *buf, unsigned len);

static void
a_amp_probe(void)
{
	static const uint8_t addrs[] = { 0x34, 0x35, 0x58, 0x5b };
	volatile uint8_t *south = (volatile uint8_t *)ml_io_map(0x00900000, 0x70000);
	int err = sweet_i2c1_init();

	if (south != NULL) {
		static const unsigned gpios[] = { 99, 105 };
		for (unsigned i = 0; i < 2; i++) {
			volatile uint32_t *ctl = (volatile uint32_t *)(south + gpios[i] * 0x1000u);
			uint32_t was = ctl[0], io = ctl[1];
			ctl[1] = 2;                             /* output high */
			mb();
			/*
			 * function 0 (GPIO), output enabled, 2 mA, no pull -- and bit 12, EGPIO_ENABLE:
			 * 99 and 105 are eGPIOs (pinctrl-sm6115.c egpio_present 11 / egpio_enable 12);
			 * with it clear (0192-0197, ctl 0xa00) the pin is not ours to drive
			 */
			ctl[0] = (was & ~0x13ffu) | (1u << 9) | (1u << 12);
			mb();
			LOG("adsp: amp reset GPIO %u: ctl %08x io %08x -> %08x", gpios[i], was, io, ctl[0]);
		}
		IOSleep(5);
	}
	for (unsigned i = 0; i < sizeof(addrs); i++) {
		uint8_t v[2] = {};
		int e = err != 0 ? err : sweet_i2c1_read(addrs[i], 0x00, v, 2);
		uint8_t v3[2] = {};
		int e3 = err != 0 ? err : sweet_i2c1_read(addrs[i], 0x03, v3, 2);
		LOG("adsp: I2C 0x%02x: reg 0 -> %d (%02x %02x), reg 3 -> %d (%02x %02x)", addrs[i], e, v[0], v[1], e3, v3[0], v3[1]);
	}
}

/* ---------------- the RPM: the codec's buck supply (pm6125 LDO14, cdc-vdd-buck) */

/*
 * 0194 on the phone: every stage of the speaker path powered and silent. The WCD937x's buck and
 * flyback, which feed every one of its outputs, run from cdc-vdd-buck -- the PMIC's LDO14, an
 * "on demand" supply Android's wcd937x driver asks the RPM for only while it plays (the static
 * ones, LDO9, keep SoundWire alive). Nobody asks for it here. Mainline's qcom_glink_rpm.c (the
 * transport: two FIFOs in the RPM's message RAM, found through its table of contents, and the
 * APCS doorbell) and smd-rpm.c / qcom_smd-regulator.c (an intentless "rpm_requests" channel,
 * "req" packets of key-value pairs), written out.
 */
#define RPM_MSG_RAM_PA  0x045F0000ull
#define RPM_MSG_RAM_SZ  0x7000u
#define APCS_IPC_PA     0x0F111000ull           /* +8: the IPC register, bit 0 the RPM */

struct rpm_pipe { volatile uint32_t *tail, *head; volatile uint8_t *fifo; uint32_t len; };
static struct rpm_pipe rpm_rx, rpm_tx;
static volatile uint8_t *rpm_apcs;
static uint32_t rpm_rcid, rpm_msg_id = 1;
static bool rpm_version, rpm_remote_open, rpm_open_sent, rpm_open_acked;
static int rpm_status = 1;                      /* the last answer: 1 none yet, 0 ok, < 0 refused */
static uint8_t rpm_buf[256];
static uint32_t rpm_len;
/*
 * 0360: the answer belongs to the request in flight. The RPM answers every request with a "msg#" message holding the
 * request's msg_id (smd-rpm.c qcom_smd_rpm_callback, RPM_MSG_TYPE_MSG_ID). An answer that comes after the wait gave up
 * (-4) used to be taken for the next request's, and every later status was one request late; now an answer for
 * another msg_id is counted and dropped.
 */
extern "C" uint64_t mach_absolute_time(void);
extern "C" void absolutetime_to_nanoseconds(uint64_t abstime, uint64_t *result);
static uint32_t rpm_wait_id;                    /* the msg_id whose answer rpm_status waits for (0: any) */
static struct {
	uint64_t requests, refused, timeouts, stale, wait_us_max;
	uint32_t last_stale_id;
} rpm_st;

static void
rpm_peek(uint32_t off, void *dst, uint32_t n)   /* whole words */
{
	uint32_t t = *rpm_rx.tail + off;
	while (t >= rpm_rx.len) {
		t -= rpm_rx.len;
	}
	for (uint32_t i = 0; i < n; i += 4) {
		((uint32_t *)dst)[i / 4] = *(volatile uint32_t *)(rpm_rx.fifo + t);
		t += 4;
		if (t >= rpm_rx.len) {
			t = 0;
		}
	}
}

static void
rpm_advance(uint32_t n)
{
	uint32_t t = *rpm_rx.tail + n;
	while (t >= rpm_rx.len) {
		t -= rpm_rx.len;
	}
	mb();
	*rpm_rx.tail = t;
}

static bool
rpm_write(const void *hdr, uint32_t hlen, const void *data, uint32_t dlen)
{
	uint32_t w[80] = {};
	uint32_t total = (hlen + dlen + 7) & ~7u;
	if (total > sizeof(w)) {
		return false;
	}
	memcpy(w, hdr, hlen);
	if (dlen != 0) {
		memcpy((uint8_t *)w + hlen, data, dlen);
	}
	uint32_t head = *rpm_tx.head, tail = *rpm_tx.tail;
	uint32_t avail = tail <= head ? rpm_tx.len - head + tail : tail - head;
	if (avail <= total) {
		LOG("adsp: RPM: TX FIFO full (head %x tail %x)", head, tail);
		return false;
	}
	for (uint32_t i = 0; i < total; i += 4) {
		*(volatile uint32_t *)(rpm_tx.fifo + head) = w[i / 4];
		head += 4;
		if (head >= rpm_tx.len) {
			head = 0;
		}
	}
	mb();
	*rpm_tx.head = head;
	mb();
	*(volatile uint32_t *)(rpm_apcs + 8) = 1;
	return true;
}

static void
rpm_cmd(uint16_t cmd, uint16_t p1, uint32_t p2)
{
	struct glink_msg m = { cmd, p1, p2 };
	(void)rpm_write(&m, sizeof(m), NULL, 0);
}

static void
rpm_open(void)
{
	struct { struct glink_msg m; char name[16]; } __attribute__((packed)) req = {};
	req.m.cmd = CMD_OPEN;
	req.m.p1 = 1;                                   /* our lcid */
	req.m.p2 = sizeof("rpm_requests");
	memcpy(req.name, "rpm_requests", sizeof("rpm_requests"));
	(void)rpm_write(&req, sizeof(req), NULL, 0);
	rpm_open_sent = true;
}

/* an answer: "req" holding "msg#" and, when refused, "err" with a string */
static void
rpm_answer(const uint8_t *p, uint32_t len)
{
	const uint32_t *h = (const uint32_t *)p;
	int st = 0;
	if (len < 8 || h[0] != 0x00716572) {
		LOG("adsp: RPM: answer of %u bytes, not \"req\" (%08x)", len, len >= 4 ? h[0] : 0);
		rpm_status = -1;
		return;
	}
	uint32_t off = 8, end = MIN(len, 8 + h[1]), id = 0;
	bool has_id = false;
	char s[40] = {};
	while (off + 8 <= end) {
		uint32_t type = *(const uint32_t *)(p + off), mlen = *(const uint32_t *)(p + off + 4);
		if (type == 0x2367736d && mlen >= 4 && off + 12 <= end) {     /* 0360: "msg#", the request's msg_id */
			id = *(const uint32_t *)(p + off + 8);
			has_id = true;
		} else if (type == 0x00727265) {                       /* "err" */
			memcpy(s, p + off + 8, MIN(mlen, (uint32_t)sizeof(s) - 1));
			st = -2;
		}
		off += (8 + mlen + 3) & ~3u;
	}
	if (has_id && rpm_wait_id != 0 && id != rpm_wait_id) {
		rpm_st.stale++;
		rpm_st.last_stale_id = id;
		if (rpm_st.stale <= 8) {
			LOG("adsp: RPM: an answer for msg %u (%s) while msg %u waits: dropped", id, st == 0 ? "taken" : s,
			    rpm_wait_id);
		}
		return;
	}
	if (st != 0) {
		LOG("adsp: RPM: refused: \"%s\"", s);
	}
	rpm_status = st;
}

static bool
rpm_rx_one(void)
{
	uint32_t h = *rpm_rx.head, t = *rpm_rx.tail;
	uint32_t avail = h >= t ? h - t : rpm_rx.len - t + h, n;
	struct glink_msg m;

	if (avail < 8) {
		return false;
	}
	rpm_peek(0, &m, 8);
	switch (m.cmd) {
	case CMD_VERSION:
		rpm_advance(8);
		LOG("adsp: RPM: glink VERSION %u features %x", m.p1, m.p2);
		rpm_cmd(CMD_VERSION_ACK, 1, 0);
		rpm_version = true;
		break;
	case CMD_VERSION_ACK:
		rpm_advance(8);
		LOG("adsp: RPM: glink VERSION_ACK %u features %x", m.p1, m.p2);
		rpm_version = true;
		break;
	case CMD_OPEN: {
		uint32_t name[8] = {};
		n = (8 + m.p2 + 7) & ~7u;
		if (avail < n) {
			return false;
		}
		rpm_peek(8, name, MIN((m.p2 + 3) & ~3u, (uint32_t)sizeof(name) - 4));
		rpm_advance(n);
		bool ours = strcmp((const char *)name, "rpm_requests") == 0;
		LOG("adsp: RPM: glink OPEN \"%s\" rcid %u%s", (const char *)name, m.p1, ours ? "" : " (not ours)");
		if (ours) {
			rpm_rcid = m.p1;
			rpm_remote_open = true;
			rpm_cmd(CMD_OPEN_ACK, m.p1, 0);
			if (!rpm_open_sent) {
				rpm_open();
			}
		}
		break;
	}
	case CMD_OPEN_ACK:
		rpm_advance(8);
		LOG("adsp: RPM: glink OPEN_ACK lcid %u", m.p1);
		rpm_open_acked = true;
		break;
	case CMD_CLOSE:
		rpm_advance(8);
		LOG("adsp: RPM: glink CLOSE rcid %u", m.p1);
		rpm_cmd(CMD_CLOSE_ACK, m.p1, 0);
		rpm_remote_open = false;
		break;
	case CMD_INTENT:
		n = (8 + 8 * m.p2 + 7) & ~7u;
		if (avail < n) {
			return false;
		}
		rpm_advance(n);
		break;
	case CMD_TX_DATA:
	case CMD_TX_DATA_CONT: {
		uint32_t cl[2];
		if (avail < 16) {
			return false;
		}
		rpm_peek(8, cl, 8);
		n = (16 + cl[0] + 7) & ~7u;
		if (avail < n) {
			return false;
		}
		if (m.cmd == CMD_TX_DATA) {
			rpm_len = 0;
		}
		if (rpm_len + cl[0] <= sizeof(rpm_buf) - 4) {
			rpm_peek(16, rpm_buf + rpm_len, (cl[0] + 3) & ~3u);
			rpm_len += cl[0];
		}
		rpm_advance(n);
		if (cl[1] == 0) {
			rpm_answer(rpm_buf, rpm_len);
		}
		break;
	}
	case CMD_READ_NOTIF:
		rpm_advance(8);
		*(volatile uint32_t *)(rpm_apcs + 8) = 1;
		break;
	default:
		rpm_advance(8);
		LOG("adsp: RPM: glink cmd %u (%x %x)", m.cmd, m.p1, m.p2);
		break;
	}
	return true;
}

static void
rpm_poll(unsigned ms, bool (*done)(void))
{
	/* 0360: the first 200 us in 10 us steps, then 1 ms ones: an answer in tens of us is not waited for a whole ms */
	for (unsigned i = 0; i < 20 && !done(); i++) {
		while (rpm_rx_one()) {
		}
		if (!done()) {
			IODelay(10);
		}
	}
	for (unsigned i = 0; i < ms && !done(); i++) {
		while (rpm_rx_one()) {
		}
		if (!done()) {
			IOSleep(1);
		}
	}
}

static bool rpm_is_up(void) { return rpm_remote_open && rpm_open_acked; }
static bool rpm_answered(void) { return rpm_status != 1; }

static bool
rpm_start(void)
{
	volatile uint8_t *ram = (volatile uint8_t *)ml_io_map(RPM_MSG_RAM_PA, RPM_MSG_RAM_SZ);
	rpm_apcs = (volatile uint8_t *)ml_io_map(APCS_IPC_PA, 0x1000);
	if (ram == NULL || rpm_apcs == NULL) {
		LOG("adsp: RPM: mapping failed");
		return false;
	}
	volatile uint32_t *toc = (volatile uint32_t *)(ram + RPM_MSG_RAM_SZ - 256);
	if (toc[0] != 0x67727430) {                     /* "grt0" */
		LOG("adsp: RPM: no table of contents (magic %08x)", toc[0]);
		return false;
	}
	for (uint32_t i = 0; i < toc[1] && i < 20; i++) {
		uint32_t id = toc[2 + 3 * i], off = toc[3 + 3 * i], size = toc[4 + 3 * i];
		struct rpm_pipe *p = id == 0x72326170 ? &rpm_rx : id == 0x61703272 ? &rpm_tx : NULL;   /* r2ap, ap2r */
		if (p != NULL && off + size <= RPM_MSG_RAM_SZ) {
			p->tail = (volatile uint32_t *)(ram + off);
			p->head = (volatile uint32_t *)(ram + off + 4);
			p->fifo = ram + off + 8;
			p->len = size;
		}
	}
	if (rpm_rx.fifo == NULL || rpm_tx.fifo == NULL) {
		LOG("adsp: RPM: FIFOs not in the table (%u entries)", toc[1]);
		return false;
	}
	LOG("adsp: RPM: RX FIFO %u bytes (head %x tail %x), TX %u bytes (head %x tail %x)", rpm_rx.len, *rpm_rx.head,
	    *rpm_rx.tail, rpm_tx.len, *rpm_tx.head, *rpm_tx.tail);
	*rpm_tx.head = 0;                               /* as Linux: a fresh link */
	*rpm_rx.tail = 0;
	mb();
	rpm_cmd(CMD_VERSION, 1, 0);
	rpm_poll(1000, rpm_is_up);
	if (!rpm_is_up() && rpm_version && !rpm_open_sent) {
		/* (the RPM may still hold the channel open from the boot loader) */
		rpm_open();
		rpm_poll(500, rpm_is_up);
	}
	LOG("adsp: RPM: version %s, rpm_requests %s (remote open %d, ack %d)", rpm_version ? "agreed" : "NO ANSWER",
	    rpm_is_up() ? "open" : "NOT open", rpm_remote_open, rpm_open_acked);
	return rpm_open_acked;
}

/*
 * 0360: one request out and its own answer back: 0, -2 refused ("err"), -3 the TX FIFO full, -4 no answer for it in
 * about a second. The caller holds rpm_lock.
 */
static int
rpm_request(const void *pkt, uint32_t size, uint32_t msg_id)
{
	uint64_t t0 = mach_absolute_time(), ns;

	rpm_wait_id = msg_id;
	rpm_status = 1;
	rpm_st.requests++;
	if (!rpm_write(pkt, size, NULL, 0)) {
		return -3;
	}
	rpm_poll(1000, rpm_answered);
	absolutetime_to_nanoseconds(mach_absolute_time() - t0, &ns);
	if (ns / 1000 > rpm_st.wait_us_max) {
		rpm_st.wait_us_max = ns / 1000;
	}
	if (rpm_status == 1) {
		rpm_st.timeouts++;
		return -4;
	}
	if (rpm_status != 0) {
		rpm_st.refused++;
	}
	return rpm_status;
}

/* an LDO: on, at uv, for ma; set 0 the active set, 1 the sleep set */
static int
rpm_ldo(uint32_t id, uint32_t uv, uint32_t ma, uint32_t set)
{
	struct {
		struct glink_msg m;
		uint32_t chunk, left;
		uint32_t svc, len, msg_id, flags, type, rid, dlen;
		uint32_t kvp[9];
	} __attribute__((packed)) pkt = {};
	pkt.m.cmd = CMD_TX_DATA;
	pkt.m.p1 = 1;
	pkt.m.p2 = 0;                                   /* intentless */
	pkt.chunk = 8 + 20 + sizeof(pkt.kvp);
	pkt.svc = 0x00716572;                           /* "req" */
	pkt.len = 20 + sizeof(pkt.kvp);
	pkt.msg_id = rpm_msg_id++;
	pkt.flags = set;
	pkt.type = 0x616f646c;                          /* "ldoa" */
	pkt.rid = id;
	pkt.dlen = sizeof(pkt.kvp);
	pkt.kvp[0] = 0x6e657773; pkt.kvp[1] = 4; pkt.kvp[2] = 1;       /* "swen" */
	pkt.kvp[3] = 0x00007675; pkt.kvp[4] = 4; pkt.kvp[5] = uv;      /* "uv" */
	pkt.kvp[6] = 0x0000616d; pkt.kvp[7] = 4; pkt.kvp[8] = ma;      /* "ma" */
	return rpm_request(&pkt, sizeof(pkt), pkt.msg_id);
}

/*
 * One key-value pair to an RPM resource (0202, kern.sweet_audio op 6): the buses and the DDR.
 * smd-rpm types: "clk0" misc (key "Enab": RPM clock scaling), "clk1" bus (SNOC 2, CNOC 1),
 * "clk2" memory (BIMC 0, the DDR), key "KHz" (Xiaomi topaz clk-smd-rpm.c, khaje = holi's clocks).
 */
static int
rpm_kvp(uint32_t set, uint32_t type, uint32_t id, uint32_t key, uint32_t value)
{
	struct {
		struct glink_msg m;
		uint32_t chunk, left;
		uint32_t svc, len, msg_id, flags, type, rid, dlen;
		uint32_t kvp[3];
	} __attribute__((packed)) pkt = {};
	pkt.m.cmd = CMD_TX_DATA;
	pkt.m.p1 = 1;
	pkt.chunk = 8 + 20 + sizeof(pkt.kvp);
	pkt.svc = 0x00716572;
	pkt.len = 20 + sizeof(pkt.kvp);
	pkt.msg_id = rpm_msg_id++;
	pkt.flags = set;
	pkt.type = type;
	pkt.rid = id;
	pkt.dlen = sizeof(pkt.kvp);
	pkt.kvp[0] = key; pkt.kvp[1] = 4; pkt.kvp[2] = value;
	return rpm_request(&pkt, sizeof(pkt), pkt.msg_id);
}

/*
 * 0204: the RPM for the rest of the kernel (SapphireGpu: the CX rail's level before a faster GPU
 * clock). The first caller starts the client; one lock for every user of it.
 */
static IOLock *rpm_lock;

/* 0360: the lock, made by whoever comes first (the audio path took it only if it existed already) */
static IOLock *
rpm_lock_get(void)
{
	if (rpm_lock == NULL) {
		IOLock *l = IOLockAlloc();
		if (!OSCompareAndSwapPtr(NULL, l, (void * volatile *)&rpm_lock)) {
			IOLockFree(l);
		}
	}
	return rpm_lock;
}

static bool
rpm_ensure(void)
{
	return rpm_is_up() || rpm_start();
}

extern "C" int
topaz_rpm_vote(uint32_t set, uint32_t type, uint32_t id, uint32_t key, uint32_t value)
{
	IOLock *l = rpm_lock_get();
	IOLockLock(l);
	int r = rpm_ensure() ? rpm_kvp(set, type, id, key, value) : -5;
	IOLockUnlock(l);
	return r;
}

/* 0248: an LDO on at uv for ma (SweetQsee.cpp: the fingerprint sensor's L9); set 0 active, 1 sleep */
extern "C" int
topaz_rpm_ldo(uint32_t id, uint32_t uv, uint32_t ma, uint32_t set)
{
	IOLock *l = rpm_lock_get();
	IOLockLock(l);
	int r = rpm_ensure() ? rpm_ldo(id, uv, ma, set) : -5;
	IOLockUnlock(l);
	return r;
}

/* 0360: the client's counters, for kern.sweet_bus_stats */
extern "C" void
topaz_rpm_stats(uint64_t *requests, uint64_t *refused, uint64_t *timeouts, uint64_t *stale, uint64_t *wait_us_max)
{
	*requests = rpm_st.requests;
	*refused = rpm_st.refused;
	*timeouts = rpm_st.timeouts;
	*stale = rpm_st.stale;
	*wait_us_max = rpm_st.wait_us_max;
}

/* ---------------- 0361: the RPM's clock scaling and the DDR's clock (report 24) */

/*
 * The DDR's clock belongs to the RPM: the BIMC clock, RPM "clk2" 0, key "KHz" (stock clk-smd-rpm.c bimc_clk; this
 * phone's clk-smd-rpm.ko holi_bimc_clk). The RPM picks the row of the DDR's plan that covers the vote (200, 547.2,
 * 768, 1017.6, 1555.2, 1804.8, 2092.8 MHz: ddr-freq-table of the stock device tree and xbl_a's table). It applies no
 * rate vote until the OS turns its clock scaling on: "Enab" = 1 on "clk0" 2, sleep set then active set
 * (clk_smd_rpm_enable_scaling()). Until then the DDR and the buses stay where the boot chain left them: 1555.2 MHz
 * here, and votes the RPM takes change nothing (03 M24). With scaling on, every bus clock of the RPM is what the
 * masters (APSS, modem, ADSP...) vote, and one nobody votes is gated; a bus the CPU reaches registers through, gated,
 * hangs the next access without a panic. So before "Enab" every bus clock of khaje's table is held, as Linux's
 * clk_smd_rpm_handoff() does (INT_MAX, or 1 for a branch, in both sets), and the BIMC at the row it runs at, so the
 * DDR does not move at the switch.
 *
 * Linux (v6.12 clk-smd-rpm.c rpm_smd_clk_probe): handoff of every clock, then "Enab". Stock khaje (this phone's
 * clk-smd-rpm.ko): the BIMC to INT_MAX in the active set, "Enab", then XO on and CNOC, SNOC and QUP at 19.2 MHz in
 * the active set; its interconnect driver keeps the six bus providers at their maximum until every consumer has
 * probed. Here every consumer runs already when scaling comes on, and none votes: hence the holds.
 *
 *   kern.sweet_bus_clocks   (read) the buses and the DDR measured now (GCC debug mux and mccc, SapphireGpu 0343):
 *                           no RPM message, available with scaling off;
 *   kern.sweet_bus_scale    write 1: the holds, then "Enab" (sleep set, active set), measured before and after;
 *                           write 0: the DDR manager stops and the BIMC goes back to the row it had at the switch
 *                           (scaling stays on in the RPM: Linux never sends "Enab" 0 and what it does is not known);
 *                           read: 0 never on, 1 on, 2 on with the DDR back at the row of the switch (1 again resumes),
 *                           -1 a step before "Enab" failed (nothing changed; 1 tries again), -2 "Enab" in the active
 *                           set got no good answer (state unknown until a reboot);
 *   kern.sweet_bus_hold     0 (default): the holds at INT_MAX (Linux's handoff); 1: CNOC, SNOC and MMRT at the rate
 *                           measured before the switch plus 2 %, the others INT_MAX. Read at the switch;
 *   kern.sweet_bus_vote     write "<resource> <kHz>": one hold voted again, both sets (scaling on only); the DDR has
 *                           its own controls below;
 *   kern.sweet_ddr_set      write a row (kHz, or MHz under 10000): the BIMC to it now (scaling on, manager off);
 *                           0 back to the row of the switch; measured until the DDR moves;
 *   kern.sweet_ddr_gov      1: the DDR follows its inputs (the GPU's level through 0347 with kern.sweet_gpu_ddr=1, the
 *                           CPU's traffic from 0362), between kern.sweet_ddr_floor and kern.sweet_ddr_cap (kHz),
 *                           up at once, down after kern.sweet_ddr_down_ms below; capped at the row of the switch while
 *                           a sensor is at kern.sweet_ddr_temp_max (m°C) or above, until 5 °C under it;
 *   kern.sweet_bus_stats    (read) state, every vote and answer, the measurements, the DDR's rows and time in each,
 *                           the switch latencies, the inputs, and the RPM client's counters (0360).
 * The DDR manager votes the BIMC in the active set only; the sleep set keeps the hold of the switch. This kernel never
 * lets the APSS sleep (no PSCI CPU_SUSPEND or CPU_OFF), so the RPM never applies the APSS's sleep set while it runs.
 */
#include <sys/sysctl.h>
#include <libkern/libkern.h>

extern "C" uint32_t sapphire_clk_measure_khz(uint32_t sel);   /* SapphireGpu.cpp 0361: a GCC debug mux input */
extern "C" uint32_t sapphire_ddr_khz(void);                   /* SapphireGpu.cpp 0361: mccc's period (0343) */
extern "C" int sapphire_tsens_max_mc(int32_t *mc);            /* SapphireGpu.cpp 0361: the hottest TSENS sensor */

#define RPM_CLK0        0x306b6c63u                 /* "clk0" misc */
#define RPM_CLK1        0x316b6c63u                 /* "clk1" bus */
#define RPM_CLK2        0x326b6c63u                 /* "clk2" memory */
#define RPM_QUP         0x00707571u                 /* "qup" */
#define RPM_MMXI        0x69786d6du                 /* "mmxi" */
#define RPM_IPA         0x00617069u                 /* "ipa" */
#define RPM_CE          0x00006563u                 /* "ce" */
#define RPM_KEY_KHZ     0x007a484bu                 /* "KHz" */
#define RPM_KEY_ENAB    0x62616e45u                 /* "Enab" */
#define RPM_RATE_MAX    0x7fffffffu                 /* INT_MAX: the top level of any rate */
#define RPM_SET_ACTIVE  0u
#define RPM_SET_SLEEP   1u

/*
 * khaje's RPM clocks with a rate (stock clk-smd-rpm.c khaje_clks, holi's definitions; Linux sm6115_clks and
 * sm_qnoc_icc_clks). meas: the GCC debug mux input that measures the clock itself (debugcc-khaje.c:
 * measure_only_cnoc_clk 0x1c, measure_only_snoc_clk 0x7, gcc_disp_hf_axi_clk 0x40, the MMRT's branch to the display);
 * floor: what another driver of this kernel votes for it (TopazModem ipa_hw_init 100 MHz, SweetQsee 192 MHz).
 */
struct bus_res {
	const char *name;
	uint32_t type, id, meas, floor;
};
static const struct bus_res bus_res[] = {
	{ "snoc_periph", RPM_CLK1, 0, 0,    0 },
	{ "cnoc",        RPM_CLK1, 1, 0x1c, 0 },
	{ "snoc",        RPM_CLK1, 2, 0x07, 0 },
	{ "snoc_lpass",  RPM_CLK1, 5, 0,    0 },
	{ "qup",         RPM_QUP,  0, 0,    0 },
	{ "mmnrt",       RPM_MMXI, 0, 0,    0 },
	{ "mmrt",        RPM_MMXI, 1, 0x40, 0 },
	{ "ipa",         RPM_IPA,  0, 0,    100000 },
	{ "ce1",         RPM_CE,   0, 0,    192000 },
};
#define BUS_N           (sizeof(bus_res) / sizeof(bus_res[0]))

/*
 * The DDR's rows and the vote that selects each: kgsl's bus table of this phone (qcom,bus-table-ddr, kB/s) over the
 * 8-byte buswidth of the ebi node, rounded up as clk-smd-rpm does: 97.7 % of 8 bytes a cycle of the row, which the
 * RPM covers with that row.
 */
static const uint32_t ddr_row_khz[] = { 200000, 547200, 768000, 1017600, 1555200, 1804800, 2092800 };
static const uint32_t ddr_row_vote[] = { 195313, 534180, 750000, 993164, 1518555, 1761719, 2042969 };
#define DDR_ROWS        7u

/* the GCC debug mux inputs kern.sweet_bus_clocks measures (debugcc-khaje.c gcc_debug_mux_sels) */
static const struct { const char *name; uint32_t sel; } bus_meas[] = {
	{ "cnoc", 0x1c }, { "snoc", 0x07 }, { "ipa_2x", 0xcd }, { "cpuss_gnoc", 0xb1 }, { "disp_hf_axi(mmrt)", 0x40 },
	{ "qup_core_2x", 0x71 }, { "ufs_axi", 0x116 }, { "usb30_master", 0x60 }, { "bimc_gpu_axi", 0x97 },
};
#define BUS_MEAS_N      (sizeof(bus_meas) / sizeof(bus_meas[0]))

static IOLock *bus_lock;
static int bus_state;                               /* kern.sweet_bus_scale: 0 off, 1 on, -1 a step failed */
static int bus_hold_mode;                           /* kern.sweet_bus_hold */
static uint32_t bus_hold[BUS_N];                    /* the kHz held for each, both sets */
static int ddr_gov;                                 /* kern.sweet_ddr_gov */
static int ddr_floor = 1555200, ddr_cap = 2092800;  /* kHz */
static int ddr_down_ms = 100, ddr_gov_ms = 20, ddr_temp_max = 95000;
static bool ddr_gov_running;
static unsigned ddr_boot_row;                       /* the row at the switch */
static unsigned ddr_row;                            /* the row the BIMC is voted at */
static uint32_t ddr_in_gpu, ddr_in_cpu;             /* inputs, kHz of BIMC vote (0: none) */
static uint32_t ddr_manual;                         /* kern.sweet_ddr_set, kHz of a row (0: none) */
static bool ddr_hot;                                /* the temperature cap is on */
static uint64_t ddr_low_since;                      /* when the inputs first asked for a lower row (0: they do not) */
static uint64_t ddr_row_since;
static struct {
	const char *failed;                             /* the step that failed, or NULL */
	int failed_err;
	uint32_t before[BUS_MEAS_N + 1], after[BUS_MEAS_N + 1];  /* measured around the switch; the last is the DDR */
	int hold_err[BUS_N], xo_err, enab_sleep, enab_active;
	uint64_t votes, refused, ups, downs, hot_caps;
	uint64_t row_ns[DDR_ROWS];
	uint32_t last_vote, last_seen_khz;
	int last_err;
	uint32_t sw_us_last, sw_us_max, sw_unseen;      /* from the vote until mccc shows the new row */
	uint32_t ack_us_last, ack_us_max;
	int32_t temp_mc;
} gbus;

static uint64_t
bus_ns(uint64_t abs)
{
	uint64_t ns;
	absolutetime_to_nanoseconds(abs, &ns);
	return ns;
}

static IOLock *
bus_lock_get(void)
{
	if (bus_lock == NULL) {
		IOLock *l = IOLockAlloc();
		if (!OSCompareAndSwapPtr(NULL, l, (void * volatile *)&bus_lock)) {
			IOLockFree(l);
		}
	}
	return bus_lock;
}

/* The row that covers khz (a BIMC vote or a DDR rate): the first whose vote is not under it; the top row above all. */
static unsigned
ddr_row_for(uint32_t khz)
{
	for (unsigned i = 0; i < DDR_ROWS; i++) {
		if (ddr_row_vote[i] >= khz) {
			return i;
		}
	}
	return DDR_ROWS - 1;
}

/* The first row whose rate is not under khz (a DDR rate: kern.sweet_ddr_floor and _cap); the top row above all. */
static unsigned
ddr_row_of_rate(uint32_t khz)
{
	for (unsigned i = 0; i < DDR_ROWS; i++) {
		if (ddr_row_khz[i] >= khz) {
			return i;
		}
	}
	return DDR_ROWS - 1;
}

/* The row a measured DDR rate is (the nearest), or DDR_ROWS if it is none within 2 %. */
static unsigned
ddr_row_measured(uint32_t khz)
{
	for (unsigned i = 0; i < DDR_ROWS; i++) {
		uint32_t d = khz > ddr_row_khz[i] ? khz - ddr_row_khz[i] : ddr_row_khz[i] - khz;
		if (d <= ddr_row_khz[i] / 50) {
			return i;
		}
	}
	return DDR_ROWS;
}

/* One RPM vote, both sets as Linux's handoff (active, then sleep): 0 or the first answer that is not. */
static int
bus_vote_both(uint32_t type, uint32_t id, uint32_t key, uint32_t value)
{
	int r = topaz_rpm_vote(RPM_SET_ACTIVE, type, id, key, value);
	if (r == 0) {
		r = topaz_rpm_vote(RPM_SET_SLEEP, type, id, key, value);
	}
	return r;
}

static void
bus_measure(uint32_t *out)
{
	for (unsigned i = 0; i < BUS_MEAS_N; i++) {
		out[i] = sapphire_clk_measure_khz(bus_meas[i].sel);
	}
	out[BUS_MEAS_N] = sapphire_ddr_khz();
}

/*
 * The BIMC to row r, active set. The time the RPM takes to answer, and the time until mccc's period shows the row
 * (polled every 5 us for 3 ms after the answer), go to the statistics. bus_lock held. 0 or the RPM's answer.
 */
static int
ddr_vote_row(unsigned r)
{
	uint64_t t0 = mach_absolute_time(), t1, now = t0;
	int e = topaz_rpm_vote(RPM_SET_ACTIVE, RPM_CLK2, 0, RPM_KEY_KHZ, ddr_row_vote[r]);

	t1 = mach_absolute_time();
	gbus.votes++;
	gbus.last_vote = ddr_row_vote[r];
	gbus.last_err = e;
	gbus.ack_us_last = (uint32_t)(bus_ns(t1 - t0) / 1000);
	gbus.ack_us_max = MAX(gbus.ack_us_max, gbus.ack_us_last);
	if (e != 0) {
		gbus.refused++;
		LOG("bus: the BIMC to %u kHz (row %u kHz) refused: %d", ddr_row_vote[r], ddr_row_khz[r], e);
		return e;
	}
	if (r != ddr_row) {
		bool seen = false;
		for (unsigned i = 0; i < 600 && !seen; i++) {
			gbus.last_seen_khz = sapphire_ddr_khz();
			unsigned m = ddr_row_measured(gbus.last_seen_khz);
			seen = m == r || (r > ddr_row && m < DDR_ROWS && m > r);
			if (!seen) {
				IODelay(5);
			}
		}
		now = mach_absolute_time();
		if (seen) {
			gbus.sw_us_last = (uint32_t)(bus_ns(now - t0) / 1000);
			gbus.sw_us_max = MAX(gbus.sw_us_max, gbus.sw_us_last);
		} else {
			gbus.sw_unseen++;
		}
		if (ddr_row_since != 0) {
			gbus.row_ns[ddr_row] += bus_ns(now - ddr_row_since);
		}
		ddr_row_since = now;
		if (r > ddr_row) {
			gbus.ups++;
		} else {
			gbus.downs++;
		}
		ddr_row = r;
	}
	return 0;
}

/*
 * The row the inputs ask for now, within floor and cap, and under the temperature cap. The inputs are BIMC votes (a
 * row covers them); floor and cap are DDR rates (rows). bus_lock held.
 */
static unsigned
ddr_want(void)
{
	uint32_t in = MAX(ddr_in_gpu, ddr_in_cpu);
	unsigned r = MAX(in != 0 ? ddr_row_for(in) : 0u, ddr_row_of_rate((uint32_t)MAX(ddr_floor, 0)));

	r = MIN(r, ddr_row_of_rate((uint32_t)MAX(ddr_cap, 0)));
	if (ddr_hot) {
		r = MIN(r, ddr_boot_row);
	}
	return r;
}

/* Applies what the inputs ask: up at once, down after ddr_down_ms of asking for less. bus_lock held. */
static void
ddr_apply(bool tick)
{
	if (bus_state != 1 || ddr_gov == 0) {
		return;
	}
	unsigned r = ddr_want();
	uint64_t now = mach_absolute_time();
	if (r > ddr_row) {
		ddr_low_since = 0;
		(void)ddr_vote_row(r);
	} else if (r < ddr_row) {
		if (ddr_low_since == 0) {
			ddr_low_since = now;
		}
		if (bus_ns(now - ddr_low_since) >= (uint64_t)MAX(ddr_down_ms, 0) * NSEC_PER_MSEC || (!tick && ddr_down_ms == 0)) {
			ddr_low_since = 0;
			(void)ddr_vote_row(r);
		}
	} else {
		ddr_low_since = 0;
	}
}

/*
 * 0347's vote for the GPU's level (SapphireGpu ddr_vote, opp_lock held there). With scaling off it goes to the RPM
 * as before. With scaling on it is the GPU's input to the manager; INT_MAX (0347 switched off) withdraws it.
 */
extern "C" int
sweet_ddr_gpu_vote(uint32_t khz)
{
	IOLock *l = bus_lock_get();
	IOLockLock(l);
	if (bus_state == 0 || bus_state == -1) {
		IOLockUnlock(l);
		return topaz_rpm_vote(RPM_SET_ACTIVE, RPM_CLK2, 0, RPM_KEY_KHZ, khz);
	}
	ddr_in_gpu = khz == RPM_RATE_MAX ? 0 : khz;
	uint64_t votes = gbus.votes;
	ddr_apply(false);
	int e = gbus.votes != votes ? gbus.last_err : 0;
	IOLockUnlock(l);
	return e;
}

/* The governor's tick: the temperature cap, the down hysteresis (0362 adds the CPU's traffic here). bus_lock held. */
static void ddr_cpu_sample(void);

static void
ddr_tick(void)
{
	int32_t mc = 0;
	if (sapphire_tsens_max_mc(&mc) == 0) {
		gbus.temp_mc = mc;
		if (!ddr_hot && mc >= ddr_temp_max) {
			ddr_hot = true;
			gbus.hot_caps++;
			LOG("bus: a sensor at %d mC: the DDR capped at %u kHz", mc, ddr_row_khz[ddr_boot_row]);
		} else if (ddr_hot && mc < ddr_temp_max - 5000) {
			ddr_hot = false;
		}
	}
	ddr_cpu_sample();
	ddr_apply(true);
}

static void
ddr_gov_thread(void *arg __unused, wait_result_t wr __unused)
{
	LOG("bus: DDR manager on (%d ms; floor %d, cap %d kHz; down after %d ms; temperature cap %d mC)", ddr_gov_ms,
	    ddr_floor, ddr_cap, ddr_down_ms, ddr_temp_max);
	IOLock *l = bus_lock_get();
	IOLockLock(l);
	while (ddr_gov != 0 && bus_state == 1) {
		ddr_tick();
		IOLockUnlock(l);
		IOSleep((uint32_t)MIN(MAX(ddr_gov_ms, 5), 1000));
		IOLockLock(l);
	}
	ddr_gov_running = false;
	ddr_in_cpu = 0;
	ddr_hot = false;
	ddr_low_since = 0;
	if (bus_state == 1) {
		(void)ddr_vote_row(ddr_manual != 0 ? ddr_row_for(ddr_manual) : ddr_boot_row);
	}
	LOG("bus: DDR manager off, the DDR at row %u kHz", ddr_row_khz[ddr_row]);
	IOLockUnlock(l);
	thread_terminate(current_thread());
}

/*
 * The switch: XO, then every hold in both sets, then "Enab" in the sleep set and the active set. Nothing is sent
 * after a step that fails. bus_lock held. 0 or an errno.
 */
static int
bus_scale_on(void)
{
	uint32_t ddr = sapphire_ddr_khz();
	unsigned row = ddr_row_measured(ddr);

	gbus.failed = NULL;
	if (row >= DDR_ROWS) {
		gbus.failed = "the DDR's clock is no row of the plan (mccc)";
		gbus.failed_err = (int)ddr;
		return ENODEV;
	}
	bus_measure(gbus.before);
	ddr_boot_row = row;
	/* XO: the stock keepalive, an active-set vote (clk-smd-rpm.c bi_tcxo_ao) */
	gbus.xo_err = topaz_rpm_vote(RPM_SET_ACTIVE, RPM_CLK0, 0, RPM_KEY_ENAB, 1);
	if (gbus.xo_err != 0) {
		gbus.failed = "XO (clk0 0)";
		gbus.failed_err = gbus.xo_err;
		return EIO;
	}
	for (unsigned i = 0; i < BUS_N; i++) {
		uint32_t v = RPM_RATE_MAX;
		if (bus_hold_mode == 1 && bus_res[i].meas != 0) {
			uint32_t m = 0;
			for (unsigned j = 0; j < BUS_MEAS_N; j++) {
				if (bus_meas[j].sel == bus_res[i].meas) {
					m = gbus.before[j];
				}
			}
			v = m != 0 ? MAX(m + m / 50, bus_res[i].floor) : RPM_RATE_MAX;
		}
		bus_hold[i] = v;
		gbus.hold_err[i] = bus_vote_both(bus_res[i].type, bus_res[i].id, RPM_KEY_KHZ, v);
		if (gbus.hold_err[i] != 0) {
			gbus.failed = bus_res[i].name;
			gbus.failed_err = gbus.hold_err[i];
			return EIO;
		}
	}
	int e = bus_vote_both(RPM_CLK2, 0, RPM_KEY_KHZ, ddr_row_vote[row]);
	if (e != 0) {
		gbus.failed = "bimc";
		gbus.failed_err = e;
		return EIO;
	}
	ddr_row = row;
	ddr_row_since = mach_absolute_time();
	gbus.enab_sleep = topaz_rpm_vote(RPM_SET_SLEEP, RPM_CLK0, 2, RPM_KEY_ENAB, 1);
	if (gbus.enab_sleep != 0) {
		gbus.failed = "Enab, sleep set";
		gbus.failed_err = gbus.enab_sleep;
		return EIO;
	}
	gbus.enab_active = topaz_rpm_vote(RPM_SET_ACTIVE, RPM_CLK0, 2, RPM_KEY_ENAB, 1);
	if (gbus.enab_active != 0) {
		gbus.failed = "Enab, active set";
		gbus.failed_err = gbus.enab_active;
		return EIO;
	}
	bus_state = 1;
	IOSleep(20);
	bus_measure(gbus.after);
	LOG("bus: RPM clock scaling on; DDR %u -> %u kHz, cnoc %u -> %u, snoc %u -> %u, mmrt %u -> %u (hold mode %d)",
	    gbus.before[BUS_MEAS_N], gbus.after[BUS_MEAS_N], gbus.before[0], gbus.after[0], gbus.before[1], gbus.after[1],
	    gbus.before[4], gbus.after[4], bus_hold_mode);
	return 0;
}

static int
sweet_bus_scale_sysctl SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	int v = bus_state;
	int error = sysctl_io_number(req, v, sizeof(v), &v, NULL);

	if (error != 0 || req->newptr == USER_ADDR_NULL) {
		return error;
	}
	IOLock *l = bus_lock_get();
	IOLockLock(l);
	if (v == 1 && (bus_state == 0 || bus_state == -1)) {
		error = bus_scale_on();
		if (error != 0) {
			/* -2: "Enab" went out and its answer says it failed: whether the RPM took it is not known */
			bus_state = gbus.enab_sleep == 0 && gbus.failed != NULL && strcmp(gbus.failed, "Enab, active set") == 0 ?
			    -2 : -1;
			LOG("bus: scaling not turned on: %s (%d)", gbus.failed, gbus.failed_err);
		}
	} else if (v == 1 && bus_state == 2) {
		bus_state = 1;                              /* the holds and "Enab" are in place already */
	} else if (v == 0 && bus_state == 1) {
		ddr_gov = 0;                                /* the thread sees it and stops */
		ddr_manual = 0;
		ddr_in_gpu = 0;
		bus_state = 2;
		error = ddr_vote_row(ddr_boot_row) == 0 ? 0 : EIO;
		LOG("bus: DDR manager off, the BIMC back at %u kHz; RPM scaling stays on", ddr_row_khz[ddr_boot_row]);
	} else if (v != 0 && v != 1) {
		error = EINVAL;
	} else if (v == 1 && bus_state == -2) {
		error = EIO;                                /* a reboot clears it */
	}
	IOLockUnlock(l);
	return error;
}
SYSCTL_PROC(_kern, OID_AUTO, sweet_bus_scale, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED,
    NULL, 0, sweet_bus_scale_sysctl, "I", "sapphire 0361: 1 = hold the RPM's bus clocks and turn its clock scaling on");
SYSCTL_INT(_kern, OID_AUTO, sweet_bus_hold, CTLFLAG_RW | CTLFLAG_LOCKED, &bus_hold_mode, 0,
    "sapphire 0361: 0 = hold the buses at INT_MAX, 1 = at the rate measured before the switch");

/* "<resource> <kHz>": one hold again, both sets */
static int
sweet_bus_vote_sysctl SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	char in[48] = {};
	size_t len = 0;
	int error;

	if (req->newptr == USER_ADDR_NULL) {
		return SYSCTL_OUT(req, "", 1);
	}
	len = MIN(req->newlen, sizeof(in) - 1);
	error = SYSCTL_IN(req, in, len);
	if (error != 0) {
		return error;
	}
	char *sp = strchr(in, ' ');
	if (sp == NULL) {
		return EINVAL;
	}
	*sp = 0;
	uint32_t khz = (uint32_t)strtoul(sp + 1, NULL, 0);
	IOLock *l = bus_lock_get();
	IOLockLock(l);
	error = bus_state == 1 ? ENOENT : ENXIO;
	for (unsigned i = 0; i < BUS_N && bus_state == 1; i++) {
		if (strcmp(in, bus_res[i].name) == 0) {
			khz = MAX(khz, MAX(bus_res[i].floor, 19200u));
			int e = bus_vote_both(bus_res[i].type, bus_res[i].id, RPM_KEY_KHZ, khz);
			if (e == 0) {
				bus_hold[i] = khz;
			}
			LOG("bus: %s held at %u kHz: %d", bus_res[i].name, khz, e);
			error = e == 0 ? 0 : EIO;
			break;
		}
	}
	IOLockUnlock(l);
	return error;
}
SYSCTL_PROC(_kern, OID_AUTO, sweet_bus_vote, CTLTYPE_STRING | CTLFLAG_RW | CTLFLAG_LOCKED,
    NULL, 0, sweet_bus_vote_sysctl, "A", "sapphire 0361: \"<resource> <kHz>\", one bus hold voted again (scaling on)");

static int
sweet_ddr_set_sysctl SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	int v = (int)ddr_manual;
	int error = sysctl_io_number(req, v, sizeof(v), &v, NULL);

	if (error != 0 || req->newptr == USER_ADDR_NULL) {
		return error;
	}
	uint32_t khz = v > 0 && v < 10000 ? (uint32_t)v * 1000 : (uint32_t)MAX(v, 0);
	unsigned r = DDR_ROWS;
	for (unsigned i = 0; i < DDR_ROWS && khz != 0; i++) {
		if (ddr_row_khz[i] / 1000 == khz / 1000) {
			r = i;
		}
	}
	if (khz != 0 && r == DDR_ROWS) {
		return EINVAL;
	}
	IOLock *l = bus_lock_get();
	IOLockLock(l);
	if (bus_state != 1) {
		error = ENXIO;
	} else if (ddr_gov != 0) {
		error = EBUSY;
	} else {
		ddr_manual = khz != 0 ? ddr_row_khz[r] : 0;
		error = ddr_vote_row(khz != 0 ? r : ddr_boot_row) == 0 ? 0 : EIO;
		LOG("bus: DDR set to row %u kHz by hand: %d (seen %u kHz, %u us from the vote; answer in %u us)",
		    ddr_row_khz[ddr_row], error, gbus.last_seen_khz, gbus.sw_us_last, gbus.ack_us_last);
	}
	IOLockUnlock(l);
	return error;
}
SYSCTL_PROC(_kern, OID_AUTO, sweet_ddr_set, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED,
    NULL, 0, sweet_ddr_set_sysctl, "I", "sapphire 0361: the DDR to a row of its plan (kHz or MHz), by hand");

static int
sweet_ddr_gov_sysctl SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	int v = ddr_gov;
	int error = sysctl_io_number(req, v, sizeof(v), &v, NULL);

	if (error != 0 || req->newptr == USER_ADDR_NULL) {
		return error;
	}
	IOLock *l = bus_lock_get();
	IOLockLock(l);
	if (v != 0 && bus_state != 1) {
		error = ENXIO;
	} else if (v != 0) {
		ddr_gov = 1;
		ddr_manual = 0;
		if (!ddr_gov_running) {                     /* a thread on its way out runs its exit under this lock */
			thread_t thread;
			ddr_gov_running = true;
			if (kernel_thread_start(ddr_gov_thread, NULL, &thread) == KERN_SUCCESS) {
				thread_deallocate(thread);
			} else {
				ddr_gov_running = false;
				ddr_gov = 0;
				error = ENOMEM;
			}
		}
	} else {
		ddr_gov = 0;
	}
	IOLockUnlock(l);
	return error;
}
SYSCTL_PROC(_kern, OID_AUTO, sweet_ddr_gov, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED,
    NULL, 0, sweet_ddr_gov_sysctl, "I", "sapphire 0361: 1 = the DDR follows the GPU's level and the CPU's traffic");
SYSCTL_INT(_kern, OID_AUTO, sweet_ddr_floor, CTLFLAG_RW | CTLFLAG_LOCKED, &ddr_floor, 0,
    "sapphire 0361: the DDR manager never goes under this, kHz (1555200)");
SYSCTL_INT(_kern, OID_AUTO, sweet_ddr_cap, CTLFLAG_RW | CTLFLAG_LOCKED, &ddr_cap, 0,
    "sapphire 0361: the DDR manager never goes over this, kHz (2092800)");
SYSCTL_INT(_kern, OID_AUTO, sweet_ddr_down_ms, CTLFLAG_RW | CTLFLAG_LOCKED, &ddr_down_ms, 0,
    "sapphire 0361: ms of asking for less before the DDR goes down (100)");
SYSCTL_INT(_kern, OID_AUTO, sweet_ddr_gov_ms, CTLFLAG_RW | CTLFLAG_LOCKED, &ddr_gov_ms, 0,
    "sapphire 0361: the DDR manager's period, ms (20)");
SYSCTL_INT(_kern, OID_AUTO, sweet_ddr_temp_max, CTLFLAG_RW | CTLFLAG_LOCKED, &ddr_temp_max, 0,
    "sapphire 0361: from this (m°C, any sensor) the DDR stays at the row of the switch (95000)");

#define BUS_CAT(...) do { if (off < sizeof(buf)) { off += (size_t)snprintf(buf + off, sizeof(buf) - off, __VA_ARGS__); } } while (0)

static int
sweet_bus_clocks_sysctl SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	char buf[512];
	size_t off = 0;
	uint32_t m[BUS_MEAS_N + 1];

	bus_measure(m);
	buf[0] = 0;
	BUS_CAT("measured, kHz (0: off, or the GPU's clock pages not mapped):");
	for (unsigned i = 0; i < BUS_MEAS_N; i++) {
		BUS_CAT(" %s %u,", bus_meas[i].name, m[i]);
	}
	BUS_CAT(" ddr %u (row %u)", m[BUS_MEAS_N], ddr_row_measured(m[BUS_MEAS_N]) < DDR_ROWS ?
	    ddr_row_khz[ddr_row_measured(m[BUS_MEAS_N])] : 0);
	return SYSCTL_OUT(req, buf, MIN(off, sizeof(buf) - 1) + 1);
}
SYSCTL_PROC(_kern, OID_AUTO, sweet_bus_clocks, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED,
    NULL, 0, sweet_bus_clocks_sysctl, "A", "sapphire 0361: the RPM's buses and the DDR, measured at the read");

static int
sweet_bus_stats_sysctl SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	char buf[1536];
	size_t off = 0;
	uint64_t rq, rf, to, st, wmax;

	IOLock *l = bus_lock_get();
	IOLockLock(l);
	buf[0] = 0;
	BUS_CAT("scaling %d (hold mode %d)%s%s (%d); xo %d, Enab sleep %d active %d\nholds:", bus_state, bus_hold_mode,
	    gbus.failed != NULL ? ", failed at " : "", gbus.failed != NULL ? gbus.failed : "", gbus.failed_err, gbus.xo_err,
	    gbus.enab_sleep, gbus.enab_active);
	for (unsigned i = 0; i < BUS_N; i++) {
		BUS_CAT(" %s %u (%d)", bus_res[i].name, bus_hold[i], gbus.hold_err[i]);
	}
	BUS_CAT("\nmeasured before -> after the switch, kHz:");
	for (unsigned i = 0; i < BUS_MEAS_N; i++) {
		BUS_CAT(" %s %u -> %u,", bus_meas[i].name, gbus.before[i], gbus.after[i]);
	}
	BUS_CAT(" ddr %u -> %u\n", gbus.before[BUS_MEAS_N], gbus.after[BUS_MEAS_N]);
	BUS_CAT("ddr: manager %d (running %d), row %u kHz (boot %u), manual %u; inputs gpu %u cpu %u, floor %d cap %d, "
	    "hot %d (%d mC, caps %llu); votes %llu refused %llu ups %llu downs %llu; last vote %u (answer %d); "
	    "answer us last %u max %u; switch us last %u max %u, not seen %u; now %u kHz\ntime at each row, ms:",
	    ddr_gov, ddr_gov_running, ddr_row_khz[ddr_row], ddr_row_khz[ddr_boot_row], ddr_manual, ddr_in_gpu, ddr_in_cpu,
	    ddr_floor, ddr_cap, ddr_hot, gbus.temp_mc, gbus.hot_caps, gbus.votes, gbus.refused, gbus.ups, gbus.downs,
	    gbus.last_vote, gbus.last_err, gbus.ack_us_last, gbus.ack_us_max, gbus.sw_us_last, gbus.sw_us_max,
	    gbus.sw_unseen, sapphire_ddr_khz());
	uint64_t now = mach_absolute_time();
	for (unsigned i = 0; i < DDR_ROWS; i++) {
		uint64_t ns = gbus.row_ns[i] + (bus_state >= 1 && i == ddr_row && ddr_row_since != 0 ?
		    bus_ns(now - ddr_row_since) : 0);
		BUS_CAT(" %u:%llu", ddr_row_khz[i] / 1000, ns / NSEC_PER_MSEC);
	}
	IOLockUnlock(l);
	topaz_rpm_stats(&rq, &rf, &to, &st, &wmax);
	BUS_CAT("\nrpm client: requests %llu refused %llu timeouts %llu stale answers %llu, longest wait %llu us\n", rq, rf,
	    to, st, wmax);
	return SYSCTL_OUT(req, buf, MIN(off, sizeof(buf) - 1) + 1);
}
SYSCTL_PROC(_kern, OID_AUTO, sweet_bus_stats, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED,
    NULL, 0, sweet_bus_stats_sysctl, "A", "sapphire 0361: the RPM's clock scaling and the DDR manager");

/*
 * 0362: the CPUs' input, as stock memlat's compute monitors do it (MiCode topaz-t-oss memlat.c; this phone's
 * memlat.ko; device tree DT:7712-7726, monitors "silver-compute" and "gold-compute"): per CPU, the effective frequency
 * is its frequency times the busy share of the window; the first row of its cluster's qcom,cpufreq-memfreq-tbl that
 * is not under it gives a DDR rate (past the last row, the last); the vote is the highest over the CPUs, and never
 * under 547000, the floor of every stock monitor (so with any CPU awake Android's DDR is not under 547.2 MHz). The
 * compute monitors ignore cache misses; the other two (instructions per L2 refill, PMU event 0x17) are not reproduced.
 * In this kernel the CPUs 0-3 are the A73s (Linux numbers them 4-7) at 2803.2 MHz and 4-7 the A53s at 1900.8 MHz, both
 * fixed (report 11 section 12; 16 section 1); Android caps the A73s at 2400 MHz, and memlat asks 2092000 for either.
 * The busy share comes from the scheduler's accounting (processor_info PROCESSOR_CPU_LOAD_INFO, 10 ms ticks), so the
 * window is kern.sweet_ddr_cpu_ms (100: one tick of error is 10 %) instead of stock's 8 ms PMU samples.
 *   kern.sweet_ddr_cpu   1 (default): the CPUs' input counts when the manager runs; 0: it does not.
 */
extern "C" processor_t cpu_to_processor(int cpu);
extern "C" int ml_get_max_cpu_number(void);
extern "C" kern_return_t processor_info(processor_t processor, processor_flavor_t flavor, host_t *host,
    processor_info_t info, mach_msg_type_number_t *count);

static const uint32_t memlat_gold[][2] = {          /* gold-compute, kHz -> kHz (DT:7720-7726) */
	{ 1056000, 547000 }, { 1344000, 768000 }, { 1766400, 1017000 }, { 2208000, 1804000 }, { 2803200, 2092000 },
};
static const uint32_t memlat_silver[][2] = {        /* silver-compute (DT:7712-7718) */
	{ 1190400, 547000 }, { 1516800, 768000 }, { 1804800, 1017000 },
};
#define MEMLAT_FLOOR_KHZ    547000u
#define CPU_GOLD_KHZ        2803200u                /* this kernel's fixed rates (11 section 12) */
#define CPU_SILVER_KHZ      1900800u
#define DDR_CPUS            8

static int ddr_cpu_on = 1, ddr_cpu_ms = 100;
static uint32_t ddr_cpu_ticks[DDR_CPUS][2];         /* busy, idle ticks at the window's start */
static uint64_t ddr_cpu_t0;
static struct { uint32_t busy_pct[DDR_CPUS], eff_mhz_max, want_khz; uint64_t windows; } gcpu;

static uint32_t
memlat_lookup(const uint32_t (*tbl)[2], unsigned n, uint32_t eff_khz)
{
	for (unsigned i = 0; i < n; i++) {
		if (tbl[i][0] >= eff_khz) {
			return tbl[i][1];
		}
	}
	return tbl[n - 1][1];
}

static bool
ddr_cpu_ticks_now(int cpu, uint32_t *busy, uint32_t *idle)
{
	processor_cpu_load_info_data_t li;
	mach_msg_type_number_t count = PROCESSOR_CPU_LOAD_INFO_COUNT;
	host_t host;

	if (cpu > ml_get_max_cpu_number()) {
		return false;
	}
	processor_t p = cpu_to_processor(cpu);
	if (p == NULL || processor_info(p, PROCESSOR_CPU_LOAD_INFO, &host, (processor_info_t)&li, &count) != KERN_SUCCESS) {
		return false;
	}
	*busy = li.cpu_ticks[CPU_STATE_USER] + li.cpu_ticks[CPU_STATE_SYSTEM] + li.cpu_ticks[CPU_STATE_NICE];
	*idle = li.cpu_ticks[CPU_STATE_IDLE];
	return true;
}

/* Every ddr_cpu_ms: the CPUs' vote (BIMC kHz of the row memlat asks) into ddr_in_cpu. bus_lock held. */
static void
ddr_cpu_sample(void)
{
	uint64_t now = mach_absolute_time();

	if (ddr_cpu_on == 0) {
		ddr_in_cpu = 0;
		ddr_cpu_t0 = 0;
		return;
	}
	if (ddr_cpu_t0 != 0 && bus_ns(now - ddr_cpu_t0) < (uint64_t)MAX(ddr_cpu_ms, 20) * NSEC_PER_MSEC) {
		return;
	}
	uint32_t want = MEMLAT_FLOOR_KHZ, eff_max = 0;
	for (int c = 0; c < DDR_CPUS; c++) {
		uint32_t busy = 0, idle = 0;
		if (!ddr_cpu_ticks_now(c, &busy, &idle)) {
			continue;
		}
		uint32_t db = busy - ddr_cpu_ticks[c][0], di = idle - ddr_cpu_ticks[c][1];
		ddr_cpu_ticks[c][0] = busy;
		ddr_cpu_ticks[c][1] = idle;
		if (ddr_cpu_t0 == 0 || db + di == 0) {
			continue;
		}
		uint32_t pct = (uint32_t)((uint64_t)db * 100 / (db + di));
		bool gold = c < 4;
		uint32_t eff = (uint32_t)((uint64_t)(gold ? CPU_GOLD_KHZ : CPU_SILVER_KHZ) * pct / 100);
		uint32_t m = gold ? memlat_lookup(memlat_gold, 5, eff) : memlat_lookup(memlat_silver, 3, eff);
		gcpu.busy_pct[c] = pct;
		eff_max = MAX(eff_max, eff);
		want = MAX(want, m);
	}
	if (ddr_cpu_t0 != 0) {
		gcpu.windows++;
		gcpu.eff_mhz_max = eff_max / 1000;
		gcpu.want_khz = want;
		/* a DDR rate of the device tree's table (547000 ... 2092000) to the BIMC vote of its row */
		ddr_in_cpu = ddr_row_vote[ddr_row_of_rate(want)];
	}
	ddr_cpu_t0 = now;
}

static int
sweet_ddr_cpu_stats_sysctl SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	char buf[256];
	size_t off = 0;

	buf[0] = 0;
	BUS_CAT("cpu input %d (%d ms windows, %llu): busy %%", ddr_cpu_on, ddr_cpu_ms, gcpu.windows);
	for (int c = 0; c < DDR_CPUS; c++) {
		BUS_CAT(" %u", gcpu.busy_pct[c]);
	}
	BUS_CAT("; highest effective %u MHz; memlat asks %u kHz (BIMC vote %u)", gcpu.eff_mhz_max, gcpu.want_khz, ddr_in_cpu);
	return SYSCTL_OUT(req, buf, MIN(off, sizeof(buf) - 1) + 1);
}
SYSCTL_PROC(_kern, OID_AUTO, sweet_ddr_cpu_stats, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED,
    NULL, 0, sweet_ddr_cpu_stats_sysctl, "A", "sapphire 0362: the CPUs' input to the DDR manager");
SYSCTL_INT(_kern, OID_AUTO, sweet_ddr_cpu, CTLFLAG_RW | CTLFLAG_LOCKED, &ddr_cpu_on, 0,
    "sapphire 0362: 1 = the CPUs' busy share counts, as stock memlat's compute monitors (1)");
SYSCTL_INT(_kern, OID_AUTO, sweet_ddr_cpu_ms, CTLFLAG_RW | CTLFLAG_LOCKED, &ddr_cpu_ms, 0,
    "sapphire 0362: the CPUs' window, ms (100)");

/* ---------------- the codec: from the RX macro to the speaker (audio=2) */

/*
 * The speaker path Android sets up (mixer_paths_bengal_qrd.xml "speaker"), written out from
 * Linux's lpass-rx-macro.c / wcd937x.c / wcd-clsh-v2.c, Qualcomm's swr-mstr-ctrl.c and
 * bolero-clk-rsc.c (ref/techpack-audio), and Xiaomi's fs1815 module (disassembled):
 *   RX_CODEC_DMA_RX_1 -> RX macro RX2 -> interpolator INT2 (AUX) -> SoundWire (RX master, its
 *   port 4 "LO" + port 2 "CLSH") -> WCD937x RX3 -> AUX DAC -> AUX PA -> the FS1815 amplifiers
 *   (I2C 0x34/0x35) -> the speakers.
 * The WCD937x's registers are reached over its TX SoundWire slave, on the VA macro's master.
 */
#define LPI_PA          0x0A7C0000ull           /* LPASS LPI pins, 0x1000 a pin */
#define LPI_SLEW_PA     0x0A95A000ull
#define RXM_PA          0x0A600000ull           /* RX macro */
#define RXSWR_PA        0x0A610000ull           /* its SoundWire master */
#define RXSWR_HCTL_PA   0x0A6A9098ull
#define RX_MUXSEL_PA    0x0A5640D8ull           /* qcom,rx_mclk_mode_muxsel */
#define VAM_PA          0x0A730000ull           /* VA macro */
#define TXM_PA          0x0A620000ull           /* TX macro */
#define VASWR_PA        0x0A740000ull
#define VASWR_HCTL_PA   0x0A7EC100ull
#define VA_WAKEUP_PA    0x003CA04Cull           /* qcom,chip-wakeup-reg of the VA SoundWire pins */
#define TLMM_WEST_PA    0x00500000ull           /* GPIO 92: the WCD937x's reset */

static volatile uint8_t *c_rx, *c_va, *c_tx, *c_rxswr, *c_vaswr;

static inline uint32_t c_rd(volatile uint8_t *b, uint32_t o) { return *(volatile uint32_t *)(b + o); }
static inline void c_wr(volatile uint8_t *b, uint32_t o, uint32_t v) { mb(); *(volatile uint32_t *)(b + o) = v; mb(); }
static inline void c_upd(volatile uint8_t *b, uint32_t o, uint32_t m, uint32_t v) { c_wr(b, o, (c_rd(b, o) & ~m) | (v & m)); }

/* ---- SoundWire master (swr-mstr-ctrl.c) */
struct swrm {
	volatile uint8_t *b;
	const char *name;
	uint8_t wid, rid;
	unsigned errors;
};
static struct swrm s_rx = { NULL, "RX", 0, 0, 0 }, s_va = { NULL, "VA", 0, 0, 0 };

static uint32_t
swr_pack(uint8_t data, uint8_t dev, uint8_t id, uint16_t reg)
{
	return (uint32_t)reg | ((uint32_t)id << 16) | ((uint32_t)dev << 20) | ((uint32_t)data << 24);
}

static void
swr_write(struct swrm *m, uint8_t dev, uint16_t reg, uint8_t val)
{
	uint8_t id;
	if (dev == 0xF) {
		id = 0xF;                               /* broadcast */
	} else {
		m->wid = m->wid < 14 ? m->wid + 1 : 1;
		id = m->wid;
	}
	c_wr(m->b, 0x300, swr_pack(val, dev, id, reg));   /* CMD_FIFO_WR_CMD */
	IODelay(dev == 0xF ? 2000 : 160);
}

static int
swr_read(struct swrm *m, uint8_t dev, uint16_t reg, uint8_t *val)
{
	m->rid = m->rid < 14 ? m->rid + 1 : 1;
	uint32_t cmd = swr_pack(1, dev, m->rid, reg), d = 0;
	IODelay(100);
	c_wr(m->b, 0x304, cmd);                     /* CMD_FIFO_RD_CMD */
	IODelay(260);
	for (int tries = 0; tries < 4; tries++) {
		d = c_rd(m->b, 0x318);                  /* CMD_FIFO_RD_FIFO */
		if (((d >> 8) & 0xF) == m->rid) {
			*val = (uint8_t)d;
			return 0;
		}
		IODelay(500);
		if (tries == 2) {
			c_wr(m->b, 0x308, 1);           /* CMD_FIFO_CMD: flush */
			c_wr(m->b, 0x304, cmd);
		}
	}
	m->errors++;
	*val = 0;
	return -1;
}

/* master init after its macro's SoundWire clock is running: Qualcomm's runtime resume */
static bool
swrm_start(struct swrm *m)
{
	uint32_t v;

	c_wr(m->b, 0x008, 1);                       /* COMP_SW_RESET */
	c_wr(m->b, 0x008, 1);
	IODelay(100);
	/* frame: 50 rows, 2 columns, SSP period 48 (9.6 MHz bus, 4 kHz frame sync) */
	c_wr(m->b, 0x101C, (1u << 3) | (0u << 0) | (47u << 16));
	c_wr(m->b, 0x500, 1);                       /* ENUMERATOR_CFG: auto enumeration */
	v = c_rd(m->b, 0x1048);                     /* MCP_CFG: no pings 0x1f */
	c_wr(m->b, 0x1048, (v & ~0x3E0000u) | (0x1Fu << 17));
	c_wr(m->b, 0x314, 3);                       /* CMD_FIFO_CFG: 3 retries */
	c_wr(m->b, 0x1044, 2);                      /* MCP_BUS_CTRL: clock start */
	c_wr(m->b, 0x004, 2);                       /* COMP_CFG: IRQ pulse, then enable */
	c_wr(m->b, 0x004, 3);
	c_wr(m->b, 0x208, 0xFFFFFFFF);              /* INTERRUPT_CLEAR */
	c_wr(m->b, 0x204, 0x1FDFD);                 /* mask (we poll) */
	c_wr(m->b, 0x210, 0x1FDFD);
	bool up = false;
	for (int i = 0; i < 100 && !up; i++) {
		up = (c_rd(m->b, 0x014) & 1) != 0;      /* COMP_STATUS: link */
		if (!up) {
			IODelay(500);
		}
	}
	c_wr(m->b, 0x314, c_rd(m->b, 0x314) | 0x80000000u);   /* >= 1.5.1: go on after a command is ignored */
	LOG("adsp: SoundWire %s master version %08x, params %08x, link %s", m->name, c_rd(m->b, 0x0), c_rd(m->b, 0x100),
	    up ? "up" : "DOWN");
	swr_write(m, 0xF, 0x41, 0x4);                /* SCP_INT_STATUS_MASK_1 (broadcast) */
	return up;
}

/* the logical device number the auto enumeration gave the slave with this 48-bit id; 0: none */
static uint8_t
swrm_find(struct swrm *m, uint64_t want)
{
	for (int t = 0; t < 50; t++) {
		uint32_t st = c_rd(m->b, 0x1090);       /* MCP_SLV_STATUS, 2 bits a device */
		for (uint32_t i = 1; i <= 11; i++) {
			uint64_t id = ((uint64_t)c_rd(m->b, 0x534 + 8 * i) << 32) | c_rd(m->b, 0x530 + 8 * i);
			uint32_t s = (st >> (2 * i)) & 3;
			if ((id & 0xFFFFFFFFFFFFull) == want && (s == 1 || s == 2)) {
				return (uint8_t)i;
			}
		}
		IOSleep(10);
	}
	uint32_t st = c_rd(m->b, 0x1090);
	LOG("adsp: SoundWire %s: no slave %llx (status %08x, ids %08x%08x %08x%08x)", m->name, want, st,
	    c_rd(m->b, 0x53C), c_rd(m->b, 0x538), c_rd(m->b, 0x544), c_rd(m->b, 0x540));
	return 0;
}

/* ---- WCD937x registers, over its TX slave on the VA master */
static uint8_t w_tx_dev, w_rx_dev;

static uint8_t
w_rd(uint16_t reg)
{
	uint8_t v = 0;
	(void)swr_read(&s_va, w_tx_dev, reg, &v);
	return v;
}

static void
w_wr(uint16_t reg, uint8_t v)
{
	swr_write(&s_va, w_tx_dev, reg, v);
}

static void
w_upd(uint16_t reg, uint8_t mask, uint8_t val)
{
	w_wr(reg, (uint8_t)((w_rd(reg) & ~mask) | (val & mask)));
}

/* ---- the FS1815 amplifiers (Xiaomi's fs1815_dlkm.ko: fs1815_reg_init / start_up, fs1815.fsm) */
extern "C" int sweet_i2c1_write(uint8_t addr, const uint8_t *buf, unsigned len);

static int
fs_wr(uint8_t a, uint8_t reg, uint16_t v)
{
	uint8_t b[3] = { reg, (uint8_t)(v >> 8), (uint8_t)v };
	return sweet_i2c1_write(a, b, 3);
}

static int
fs_rd(uint8_t a, uint8_t reg, uint16_t *v)
{
	uint8_t b[2] = {};
	int e = sweet_i2c1_read(a, reg, b, 2);
	*v = (uint16_t)((b[0] << 8) | b[1]);
	return e;
}

/* one table entry: addr bits 7-0 register, 11-8 field position, 15-12 field width - 1 */
static void
fs_entry(uint8_t a, uint16_t addr, uint16_t val)
{
	uint8_t reg = (uint8_t)addr, pos = (addr >> 8) & 0xF, w = (uint8_t)((addr >> 12) + 1);
	if (addr >= 0x1000 && pos == 0xF) {
		if ((addr >> 12) == 1 && val != 0) {
			IOSleep(val);                   /* a delay */
		}
		return;                                 /* (the polls of fsm_reg_case: not in this preset) */
	}
	if (w == 16 && pos == 0) {
		(void)fs_wr(a, reg, val);
		return;
	}
	uint16_t old = 0, mask = (uint16_t)((((1u << w) - 1) << pos) & 0xFFFF);
	(void)fs_rd(a, reg, &old);
	(void)fs_wr(a, reg, (uint16_t)((old & ~mask) | ((val << pos) & mask)));
}

static const uint16_t fs_common[][2] = {        /* fs1815.fsm, both presets */
	{ 0xf03e, 0x8500 }, { 0xf03f, 0x8110 }, { 0xf04c, 0x7008 }, { 0xf02a, 0x0030 }, { 0xf058, 0x827a },
	{ 0xf059, 0xd821 }, { 0xf05a, 0xd841 }, { 0xf05b, 0x007b }, { 0xf05c, 0x0000 }, { 0xf088, 0xa000 },
	{ 0xf089, 0x0032 }, { 0x44bc, 0x0002 }, { 0x034c, 0x0000 }, { 0xf0b8, 0x0002 }, { 0xf05d, 0x2005 },
};
static const uint16_t fs_scene1[][2] = {        /* scene 0x0001 (music); 0x4019 differs between the two */
	{ 0xf020, 0x1722 }, { 0xf022, 0x0208 }, { 0xf023, 0x0027 }, { 0xf018, 0x000a }, { 0x0f3f, 0x0000 },
	{ 0x0fbc, 0x0000 }, { 0xf01f, 0xca91 }, { 0xf058, 0x827a }, { 0x3c59, 0x0000 }, { 0x4019, 0x0000 },
	{ 0xf05a, 0xfcab }, { 0xf01f, 0x0000 },
};

static bool
fs_init(uint8_t a, uint16_t scene_vol)
{
	uint16_t id = 0;
	for (int t = 0; t < 5; t++) {
		(void)fs_wr(a, 0x10, 2);                /* reset */
		IOSleep(15);
		(void)fs_wr(a, 0x10, 1);                /* power down */
		(void)fs_rd(a, 0x0E, &id);
		if (id == 0x0003 || id == 0x0300) {
			break;
		}
	}
	(void)fs_wr(a, 0x14, 0x00e5 << 1);          /* info table +4, shifted: fs1815_reg_init */
	for (unsigned i = 0; i < sizeof(fs_common) / sizeof(fs_common[0]); i++) {
		fs_entry(a, fs_common[i][0], fs_common[i][1]);
	}
	for (unsigned i = 0; i < sizeof(fs_scene1) / sizeof(fs_scene1[0]); i++) {
		fs_entry(a, fs_scene1[i][0], fs_scene1[i][0] == 0x4019 ? scene_vol : fs_scene1[i][1]);
	}
	/* start up: dev type 0x11 (info table [0]) */
	(void)fs_wr(a, 0xbc, 0x9433);
	(void)fs_wr(a, 0x11, 0x8f);
	(void)fs_wr(a, 0x10, 0x0000);
	/* fsm_config_vol (0192 left it out): volume 0xff (0 dB) << 8 into 0x06, or into 0x16 with 0x80 */
	uint16_t v06 = 0, v16 = 0;
	(void)fs_rd(a, 0x06, &v06);
	(void)fs_rd(a, 0x16, &v16);
	(void)fs_wr(a, 0x06, 0xff00);
	(void)fs_wr(a, 0x16, 0xff80);
	IOSleep(10);
	LOG("adsp: FS1815 0x%02x: volume registers were 06 %04x 16 %04x", a, v06, v16);
	uint16_t st = 0, s11 = 0;
	(void)fs_rd(a, 0x00, &st);
	(void)fs_rd(a, 0x11, &s11);
	LOG("adsp: FS1815 0x%02x: id %04x, after start status %04x, reg 11 %04x", a, id, st, s11);
	return id == 0x0003 || id == 0x0300;
}

/*
 * a_flags (kern.sweet_audio op 5): 1 the RX clocks and RX MCLK mux 1 as 0192-0195 had them (stock
 * asks only for the TX ones: topaz-woa P9_audio.md, "root cause"), 2 without the stock RX macro
 * table, 4 the headphones (INT0/INT1 to HPHL/HPHR) instead of the top speaker (INT0 to EAR), 8 without
 * the FS1815 init.
 */
static uint32_t a_flags;                        /* 0199: the speakers (EAR + AUX); 4 the headphones instead of EAR */

/* RX macro registers from the stock Android dump while playing (topaz-woa TopazSpeaker rxtab.h) */
static const uint16_t rx_stock[][2] = {
	{ 0x070, 0x11 }, { 0x074, 0x20 }, { 0x080, 0x11 }, { 0x084, 0x20 }, { 0x090, 0x0c }, { 0x094, 0x0c },
	{ 0x098, 0x0c }, { 0x110, 0x08 }, { 0x144, 0x38 }, { 0x190, 0x07 }, { 0x200, 0x01 }, { 0x204, 0x03 },
	{ 0x208, 0x02 }, { 0x20c, 0x1c }, { 0x210, 0x39 }, { 0x214, 0x0c }, { 0x218, 0x0c }, { 0x21c, 0x01 },
	{ 0x228, 0x80 }, { 0x238, 0x07 }, { 0x244, 0x02 }, { 0x248, 0x9a }, { 0x24c, 0x10 }, { 0x284, 0x10 },
	{ 0x290, 0x04 }, { 0x294, 0xe0 }, { 0x298, 0x01 }, { 0x29c, 0x40 }, { 0x2a0, 0x2a }, { 0x2ac, 0x18 },
	{ 0x2b0, 0x18 }, { 0x2b4, 0x03 }, { 0x2b8, 0x01 }, { 0x2c4, 0x64 }, { 0x2c8, 0x01 }, { 0x2d8, 0x0c },
	{ 0x2e0, 0x77 }, { 0x2e4, 0x01 }, { 0x2ec, 0x4b }, { 0x2f4, 0x01 }, { 0x300, 0x04 }, { 0x304, 0x08 },
	{ 0x308, 0x0c }, { 0x30c, 0xe2 }, { 0x314, 0x04 }, { 0x318, 0x3e }, { 0x360, 0xff }, { 0x380, 0xff },
	{ 0x408, 0x64 }, { 0x40c, 0x8f }, { 0x410, 0x02 }, { 0x418, 0x04 }, { 0x41c, 0x7e }, { 0x424, 0x08 },
	{ 0x434, 0x02 }, { 0x438, 0x08 }, { 0x440, 0x08 }, { 0x450, 0x55 }, { 0x454, 0x55 }, { 0x458, 0x55 },
	{ 0x488, 0x64 }, { 0x48c, 0x8f }, { 0x490, 0x02 }, { 0x498, 0x04 }, { 0x49c, 0x7e }, { 0x4a4, 0x08 },
	{ 0x4b4, 0x02 }, { 0x4b8, 0x08 }, { 0x4c0, 0x08 }, { 0x4d0, 0x55 }, { 0x4d4, 0x55 }, { 0x4d8, 0x55 },
	{ 0x504, 0x18 }, { 0x508, 0x64 }, { 0x50c, 0x8f }, { 0x510, 0x02 }, { 0x518, 0x04 }, { 0x51c, 0x7e },
	{ 0x524, 0x04 }, { 0x528, 0x08 }, { 0x540, 0x02 }, { 0x544, 0x08 }, { 0x54c, 0x01 }, { 0x784, 0x07 },
	{ 0x788, 0x3c }, { 0x800, 0x60 }, { 0x804, 0xdb }, { 0x808, 0xff }, { 0x80c, 0x35 }, { 0x810, 0xff },
	{ 0x818, 0x01 }, { 0x81c, 0x28 }, { 0x840, 0x60 }, { 0x844, 0xdb }, { 0x848, 0xff }, { 0x84c, 0x35 },
	{ 0x850, 0xff }, { 0x858, 0x01 }, { 0x85c, 0x28 }, { 0xa24, 0x40 }, { 0xaa4, 0x40 }, { 0xb40, 0x04 },
	{ 0xb50, 0x04 }, { 0xc04, 0x01 }, { 0xc44, 0x01 }, { 0xc84, 0x01 }, { 0xd0c, 0xa8 }, { 0xd4c, 0xa8 },
	{ 0xd8c, 0xa8 }, { 0xf08, 0x62 }, { 0xf0c, 0x96 }, { 0xf88, 0x62 }, { 0xf8c, 0x96 },
};

static void
lpi_pin(volatile uint8_t *lpi, unsigned gpio, unsigned pull)
{
	/* LPI cfg: pull [1:0], function [5:2] = 1, drive [8:6] = 10 mA, bits 10/11 as stock (0xD04 / 0xD06) */
	c_wr(lpi, gpio * 0x1000u, (pull & 3) | (1u << 2) | (4u << 6) | (3u << 10));
}

/* everything from the clocks to the amplifiers; returns whether the codec is there */
static bool
codec_up(void)
{
	static const uint32_t tx_core[] = { 1, 0x30c, 19200000, 1, 0 }, tx_npl[] = { 1, 0x30d, 19200000, 1, 0 };
	int64_t r1 = a_prm(PARAM_ID_RSC_AUDIO_HW_CLK, tx_core, 5), r2 = a_prm(PARAM_ID_RSC_AUDIO_HW_CLK, tx_npl, 5);
	LOG("adsp: codec clocks: TX core %lld, TX NPL %lld (19.2 MHz; the VA SoundWire master runs on TX MCLK)", r1, r2);

	volatile uint8_t *lpi = (volatile uint8_t *)ml_io_map(LPI_PA, 0x13000);
	volatile uint8_t *slew = (volatile uint8_t *)ml_io_map(LPI_SLEW_PA, 0x1000);
	volatile uint8_t *muxsel = (volatile uint8_t *)ml_io_map(RX_MUXSEL_PA & ~0xFFFull, 0x1000);
	volatile uint8_t *wake = (volatile uint8_t *)ml_io_map(VA_WAKEUP_PA & ~0xFFFull, 0x1000);
	volatile uint8_t *west = (volatile uint8_t *)ml_io_map(TLMM_WEST_PA + 92 * 0x1000u, 0x1000);
	c_rx = (volatile uint8_t *)ml_io_map(RXM_PA, 0x1000);
	c_va = (volatile uint8_t *)ml_io_map(VAM_PA, 0x1000);
	c_tx = (volatile uint8_t *)ml_io_map(TXM_PA, 0x1000);
	c_rxswr = (volatile uint8_t *)ml_io_map(RXSWR_PA, 0x2000);
	c_vaswr = (volatile uint8_t *)ml_io_map(VASWR_PA, 0x2000);
	volatile uint8_t *rxh = (volatile uint8_t *)ml_io_map(RXSWR_HCTL_PA & ~0xFFFull, 0x1000);
	volatile uint8_t *vah = (volatile uint8_t *)ml_io_map(VASWR_HCTL_PA & ~0xFFFull, 0x1000);
	if (!lpi || !slew || !muxsel || !wake || !west || !c_rx || !c_va || !c_tx || !c_rxswr || !c_vaswr || !rxh || !vah) {
		LOG("adsp: codec: a mapping failed");
		return false;
	}
	s_rx.b = c_rxswr;
	s_va.b = c_vaswr;

	/* RX macro clock: from the TX MCLK as stock (mux 0); 0192-0195 had the RX core clock (mux 1) */
	if (a_flags & 1) {
		c_wr(muxsel, RX_MUXSEL_PA & 0xFFF, 1);
	}
	/* SoundWire pins: VA 0 clk, 1/2 data; RX 3 clk, 4/5 data (bus keeper on data), slew 3 */
	lpi_pin(lpi, 0, 0); lpi_pin(lpi, 1, 2); lpi_pin(lpi, 2, 2);
	lpi_pin(lpi, 3, 0); lpi_pin(lpi, 4, 2); lpi_pin(lpi, 5, 2);
	{
		c_wr(slew, 0, 0x3F3F);                  /* as stock */
	}
	c_upd(wake, VA_WAKEUP_PA & 0xFFF, 1, 1);
	/* topaz-woa TopazSpeaker hw.c CodecBringUp(), the stock sequence: VA fs-gen + SWR clock */
	c_wr(c_va, 0x000, 1); c_wr(c_va, 0x004, 3); c_wr(c_va, 0x004, 1); c_wr(c_va, 0x080, 2);
	c_wr(c_va, 0x008, 2); c_wr(c_va, 0x008, 3); c_wr(c_va, 0x008, 1);
	/* RX macro probe-time settings, MCLK, FS counter, SWR clock */
	c_upd(c_rx, 0x434, 0x7, 0x2); c_upd(c_rx, 0x4b4, 0x7, 0x2); c_upd(c_rx, 0x540, 0x7, 0x2);
	c_upd(c_rx, 0x410, 0x3, 0x2); c_upd(c_rx, 0x490, 0x3, 0x2); c_upd(c_rx, 0x510, 0x3, 0x2);
	c_wr(c_rx, 0x100, 3); c_wr(c_rx, 0x104, 3); c_wr(c_rx, 0x104, 1);
	c_wr(c_rx, 0x108, 2); c_wr(c_rx, 0x108, 3); c_wr(c_rx, 0x108, 1);
	/* TX macro MCLK, FS, SWR clock: RX and VA run on the TX MCLK (0192-0195 left it off) */
	c_upd(c_tx, 0x090, 0x01, 0x01);             /* FREQ_MCLK 9.6 MHz, before the MCLK (vendor tx_macro; the microphones) */
	c_wr(c_tx, 0x000, 1); c_wr(c_tx, 0x004, 3); c_wr(c_tx, 0x004, 1);
	c_wr(c_tx, 0x008, 2); c_wr(c_tx, 0x008, 3); c_wr(c_tx, 0x008, 1);
	LOG("adsp: RX macro MCLK %08x FS %08x SWR %08x; VA MCLK %08x FS %08x SWR %08x TOP %08x", c_rd(c_rx, 0x100),
	    c_rd(c_rx, 0x104), c_rd(c_rx, 0x108), c_rd(c_va, 0x0), c_rd(c_va, 0x4), c_rd(c_va, 0x8), c_rd(c_va, 0x80));
	/* the SoundWire CGCRs: clock on with a reset pulse (stock) */
	c_wr(rxh, RXSWR_HCTL_PA & 0xFFF, 3); IOSleep(1); c_wr(rxh, RXSWR_HCTL_PA & 0xFFF, 1);
	c_wr(vah, VASWR_HCTL_PA & 0xFFF, 3); IOSleep(1); c_wr(vah, VASWR_HCTL_PA & 0xFFF, 1);

	/* WCD937x out of reset (active low, GPIO 92 as an output) */
	{
		volatile uint32_t *g = (volatile uint32_t *)west;
		g[1] = 0;                               /* low */
		mb();
		g[0] = (g[0] & ~0x3ffu) | (1u << 9) | (7u << 6);   /* GPIO, output, 16 mA */
		mb();
		IODelay(30);
		g[1] = 2;                               /* high: out of reset */
		mb();
		IOSleep(5);
	}
	bool va_up = swrm_start(&s_va), rx_up = swrm_start(&s_rx);
	IOSleep(20);
	w_tx_dev = va_up ? swrm_find(&s_va, 0x000A01170223ull) : 0;
	w_rx_dev = rx_up ? swrm_find(&s_rx, 0x000A01170224ull) : 0;
	LOG("adsp: WCD937x slaves: TX dev %u (VA master), RX dev %u (RX master)", w_tx_dev, w_rx_dev);
	IOSleep(300);
	if (w_tx_dev == 0 || w_rx_dev == 0) {
		return false;
	}
	uint8_t e0 = w_rd(0x34B0), e16 = w_rd(0x34C0), e17 = w_rd(0x34C1), e29 = w_rd(0x34CD);
	LOG("adsp: WCD937x efuse 0 %02x (chip id %u: 0 WCD9370, 1 WCD9375), 16 %02x 17 %02x 29 %02x; read errors %u",
	    e0, (e0 & 0x1e) >> 1, e16, e17, e29, s_va.errors);
	/* wcd937x_io_init */
	if (e16 == 0x02 || e17 > 0x09) {
		w_upd(0x3103, 0x0E, e29 & 0x0F);
	} else {
		w_upd(0x3103, 0x0E, 0x0E);
	}
	w_upd(0x3103, 0x80, 0x80);
	IOSleep(1);
	w_upd(0x3103, 0x40, 0x40);
	IOSleep(1);
	w_upd(0x312A, 0x10, 0x00);                  /* LDORXTX_CONFIG */
	w_upd(0x3029, 0xF0, 0x80);                  /* BIAS_VBG_FINE_ADJ */
	w_upd(0x3001, 0x80, 0x80);                  /* ANA_BIAS */
	w_upd(0x3001, 0x40, 0x40);
	IOSleep(10);
	w_upd(0x3001, 0x40, 0x00);
	w_wr(0x30E2, 0xD9);                         /* HPH_SURGE_HPHLR_SURGE_EN */
	w_wr(0x306B, 0xFA); w_wr(0x306E, 0xFA); w_wr(0x3071, 0xFA);   /* MICBn_TEST_CTL_1 */
	if (e16 == 0x01 || e16 == 0x02) {
		w_upd(0x3029, 0xF0, 0xB0);
	}
	/* the stock values topaz-woa wcd_aux.ps1 / wcd_auxpa.ps1 write */
	w_wr(0x30A5, 0xEB);                         /* FLYBACK_VNEG_CTRL_1 */
	w_wr(0x340D, 0xBC); w_wr(0x340E, 0xBC); w_wr(0x340F, 0xBC);   /* CDC_RX0..2_CTL */
	w_wr(0x300A, 0x40); w_wr(0x300B, 0x12);     /* ANA_EAR, ANA_EAR_COMPANDER_CTL */
	w_wr(0x346B, 0x4C); w_wr(0x346C, 0x7F); w_wr(0x346D, 0x0F);
	if (!(a_flags & 2)) {
		for (unsigned i = 0; i < sizeof(rx_stock) / sizeof(rx_stock[0]); i++) {
			c_wr(c_rx, rx_stock[i][0], rx_stock[i][1]);
		}
		c_wr(c_rx, 0x400, 4); c_wr(c_rx, 0x480, 4); c_wr(c_rx, 0x500, 4);
	}
	LOG("adsp: WCD937x init: SLEEP_CTL %02x ANA_BIAS %02x; RX macro stock table %s, flags %x", w_rd(0x3103), w_rd(0x3001),
	    (a_flags & 2) ? "skipped" : "written", a_flags);
	return true;
}

/* SoundWire ports for the AUX path on the RX master and the WCD937x's RX slave, then a bank switch */
/* dp1: the channels of port 1 (HPH L = RX1, also EAR's data through RDAC3; HPH R) */
static void
swr_ports_on(uint8_t dp1)
{
	struct swrm *m = &s_rx;
	uint8_t dev = w_rx_dev;
	uint32_t bank = (c_rd(m->b, 0x104C) & 1) ? 0 : 1;   /* MCP_STATUS: the inactive bank */

	for (unsigned pass = 0; pass < 2; pass++) {
		/* slave DP2 (CLSH): si 31, off1 0, off2 0, hstart 3, hstop 6, word length 7, block pack 0, lane 0 */
		uint32_t o2 = 0x100 * 1, o4 = 0x100 * 3, bk = 0x10 * bank;
		if (pass == 0) {
			swr_write(m, 0xF, (uint16_t)(0xE0 + 0x10 * bank), 0x01);   /* SCP_HOST_CLK_DIV2_CTL (broadcast) */
		}
		swr_write(m, dev, (uint16_t)(0x120 + o2 + bk), 0x1);      /* channel enable */
		swr_write(m, dev, (uint16_t)(0x122 + o2 + bk), 31);       /* sample interval */
		swr_write(m, dev, (uint16_t)(0x124 + o2 + bk), 0);        /* offset 1 */
		swr_write(m, dev, (uint16_t)(0x125 + o2 + bk), 0);        /* offset 2 */
		swr_write(m, dev, (uint16_t)(0x126 + o2 + bk), (3 << 4) | 6);
		swr_write(m, dev, (uint16_t)(0x103 + o2), 7);             /* block control 1: word length */
		swr_write(m, dev, (uint16_t)(0x127 + o2 + bk), 0);        /* block control 3: pack mode */
		swr_write(m, dev, (uint16_t)(0x128 + o2 + bk), 0);        /* lane */
		/* slave DP4 (LO = AUX): si 7, off1 1, off2 0, lane 0 */
		swr_write(m, dev, (uint16_t)(0x120 + o4 + bk), 0x1);
		swr_write(m, dev, (uint16_t)(0x122 + o4 + bk), 7);
		swr_write(m, dev, (uint16_t)(0x124 + o4 + bk), 1);
		swr_write(m, dev, (uint16_t)(0x125 + o4 + bk), 0);
		swr_write(m, dev, (uint16_t)(0x128 + o4 + bk), 0);
		/* slave DP1 (HPH L+R, swrm_port_config.h rx_frame_params): si 3, off1 0, off2 0, word length 1, lane 1 */
		swr_write(m, dev, (uint16_t)(0x120 + bk), dp1);
		swr_write(m, dev, (uint16_t)(0x122 + bk), 3);
		swr_write(m, dev, (uint16_t)(0x124 + bk), 0);
		swr_write(m, dev, (uint16_t)(0x125 + bk), 0);
		swr_write(m, dev, (uint16_t)0x103, 1);
		swr_write(m, dev, (uint16_t)(0x128 + bk), 1);
		c_wr(m->b, 0x1124 + 0x40 * bank, ((uint32_t)dp1 << 24) | (0 << 16) | (0 << 8) | 3);
		c_wr(m->b, 0x1128 + 0x40 * bank, 1);                      /* lane 1 */
		c_wr(m->b, 0x112C, 1);
		c_wr(m->b, 0x1134 + 0x40 * bank, (0xF << 4) | 0);
		/* master port 2 and 4 (DP_PORT_CTRL: channels << 24 | off2 << 16 | off1 << 8 | si) */
		c_wr(m->b, 0x1124 + 0x100 * 1 + 0x40 * bank, (1u << 24) | (0 << 16) | (0 << 8) | 31);
		c_wr(m->b, 0x1128 + 0x100 * 1 + 0x40 * bank, 0);          /* lane */
		c_wr(m->b, 0x112C + 0x100 * 1, 7);                        /* word length */
		c_wr(m->b, 0x1134 + 0x100 * 1 + 0x40 * bank, (6 << 4) | 3);   /* hstop << 4 | hstart */
		c_wr(m->b, 0x1138 + 0x100 * 1 + 0x40 * bank, 0);          /* block pack mode */
		c_wr(m->b, 0x1124 + 0x100 * 3 + 0x40 * bank, (1u << 24) | (0 << 16) | (1 << 8) | 7);
		c_wr(m->b, 0x1128 + 0x100 * 3 + 0x40 * bank, 0);
		c_wr(m->b, 0x1134 + 0x100 * 3 + 0x40 * bank, 0);             /* stock: 0 */
		if (pass == 0) {
			/* frame for data: 50 rows, 16 columns, SSP period 6, divider 1; switch to that bank */
			uint32_t v = c_rd(m->b, 0x101C + 0x40 * bank) & ~0x00FF07FFu;
			v |= (1u << 3) | 7u | (0u << 8) | (5u << 16);
			c_wr(m->b, 0x101C + 0x40 * bank, v);
			swr_write(m, 0xF, (uint16_t)(0x60 + 0x10 * bank), (1 << 3) | 7);   /* SCP_FRAME_CTRL bank: switch */
			IOSleep(2);
			bank ^= 1;                      /* and the same into the other bank */
		}
	}
	uint32_t nb = c_rd(m->b, 0x104C) & 1;
	uint8_t d1 = 0, d4 = 0;
	(void)swr_read(m, dev, (uint16_t)(0x120 + 0x10 * nb), &d1);
	(void)swr_read(m, dev, (uint16_t)(0x420 + 0x10 * nb), &d4);
	LOG("adsp: SoundWire RX: bank now %u, port 1 %08x port 2 %08x port 4 %08x, frame %08x/%08x; slave DP1 ch %02x DP4 ch %02x, errors %u",
	    nb, c_rd(m->b, 0x1124 + 0x40 * nb), c_rd(m->b, 0x1124 + 0x100 + 0x40 * nb), c_rd(m->b, 0x1124 + 0x300 + 0x40 * nb),
	    c_rd(m->b, 0x101C), c_rd(m->b, 0x105C), d1, d4, m->errors);
}

/*
 * The AUX output (to the FS1815s) and both headphone outputs at once, all fed from RX2 -- mainline
 * lpass-rx-macro.c / wcd937x.c / wcd-clsh-v2.c (v3) in mixer_paths_bengal_idp.xml's mode, CLS_AB
 * ("RX HPH Mode"), compander off; the headphones as its "headphones" path (INT0/INT1, CLSH_DSM_OUT)
 * but 20 dB down (the tone is near full scale).
 */
static void
codec_rx_on(void)
{
	/* RX macro: INT0 (HPHL), INT1 (HPHR), INT2 (AUX), each from RX2 at 48 kHz, muted while they start */
	/*
	 * The bottom speaker is AUX (INT2), the top one EAR (INT0 with "RX_EAR Mode" ON, through RDAC3), each
	 * through its FS1815 (mixer_paths_bengal_idp.xml "speaker"; heard on the phone 2026-10-24). The
	 * headphones take INT0 back, so they replace EAR.
	 */
	const bool hph = (a_flags & 4) != 0, ear = !hph;
	c_upd(c_rx, 0x190, 0x0F, 7);                /* INT2_CFG0: INP0 = RX2 */
	c_upd(c_rx, 0x200, 0x01, 0x01);             /* class-H block clock */
	if (ear) {
		c_upd(c_rx, 0x180, 0x0F, 7);        /* INT0_CFG0: INP0 = RX2 */
		c_upd(c_rx, 0x400, 0x1F, 0x14);     /* PGA mute, 48 kHz */
		c_upd(c_rx, 0x440, 0x01, 0x01);     /* DSM clock */
		c_upd(c_rx, 0x40C, 0x03, 0x03);     /* HPF cut */
		c_upd(c_rx, 0x404, 0x40, 0x40);     /* CFG0: class-H */
		c_upd(c_rx, 0x408, 0x03, 0x03);     /* CFG1: DEM CLSH_DSM_OUT, HPH_L_EAR_SEL */
		c_wr(c_rx, 0x414, 0x00);            /* volume 0 dB */
		c_upd(c_rx, 0x400, 0x20, 0x20);     /* path clock */
	}
	if (hph) {
		c_upd(c_rx, 0x180, 0x0F, 7);        /* INT0_CFG0: INP0 = RX2 */
		c_upd(c_rx, 0x188, 0x0F, 7);        /* INT1_CFG0 */
		c_wr(c_rx, 0x220, 0xC0);            /* CLSH_K1_LSB, K1_MSB 0 (16 ohm) */
		c_upd(c_rx, 0x21C, 0x0F, 0x00);
		c_upd(c_rx, 0x20C, 0x3F, 0x1C);     /* CLSH_HPH_V_PA */
		c_upd(c_rx, 0x208, 0x07, 0x00);     /* CLSH_DECAY_CTRL */
	}
	for (unsigned k = 0; hph && k < 2; k++) {
		uint32_t b = 0x400 + 0x80 * k;
		c_upd(c_rx, b + 0x00, 0x1F, 0x14);      /* PGA mute, 48 kHz */
		c_upd(c_rx, b + 0x40, 0x01, 0x01);      /* DSM clock */
		c_upd(c_rx, b + 0x0C, 0x03, 0x03);      /* HPF cut */
		c_upd(c_rx, k == 0 ? 0x28 : 0x3C, 0x80, 0x80);   /* TOP_HPHx_COMP_LUT: bypass */
		c_upd(c_rx, b + 0x04, 0x40, 0x40);      /* CFG0: class-H */
		c_upd(c_rx, b + 0x08, 0x01, 0x01);      /* CFG1: DEM mux CLSH_DSM_OUT */
		c_wr(c_rx, b + 0x14, 0xEC);             /* volume -20 dB */
		c_upd(c_rx, b + 0x00, 0x20, 0x20);      /* path clock */
	}
	c_upd(c_rx, 0x500, 0x1F, 0x14);             /* RX2 path: PGA mute, 48 kHz */
	c_upd(c_rx, 0x54C, 0x01, 0x01);             /* DSM clock (0x440 + 2 * 0x80 + 0xc) */
	c_upd(c_rx, 0x50C, 0x03, 0x03);             /* HPF cut */
	c_upd(c_rx, 0x504, 0x18, 0x18);             /* INT2 delay zone + class-H */
	c_wr(c_rx, 0x514, 0x00);                    /* volume 0 dB */
	c_wr(c_rx, 0x518, 0x04);                    /* RX2 mix path: 48 kHz (stock) */
	c_upd(c_rx, 0x500, 0x20, 0x20);             /* path clock */

	/* WCD937x: RX clocks (wcd937x_rx_clk_enable) */
	w_upd(0x3409, 0x08, 0x08);                  /* DIG_CLK_CTL */
	w_upd(0x3408, 0x01, 0x01);                  /* ANA_CLK_CTL */
	w_upd(0x3008, 0x01, 0x01);                  /* RX_SUPPLIES */
	w_upd(0x340D, 0x40, 0x00); w_upd(0x340E, 0x40, 0x00); w_upd(0x340F, 0x40, 0x00);
	w_upd(0x3408, 0x02, 0x02);
	/* RDAC1 / RDAC2 (HPHL / HPHR DACs), PRE_PMU */
	if (hph) {
		w_upd(0x3409, 0x03, 0x03);          /* DIG_CLK_CTL: RX0, RX1 */
		w_upd(0x344E, 0x0C, 0x0C);          /* CDC_HPH_GAIN_CTL */
		w_upd(0x30D9, 0x80, 0x00);          /* HPH_RDAC_CLK_CTL1 */
	}
	/* RDAC3 (EAR DAC) from RX1, PRE_PMU */
	if (ear) {
		w_wr(0x3450, 0x00);                 /* CDC_EAR_PATH_CTL: RDAC3_MUX RX1 */
		w_upd(0x344E, 0x04, 0x04);          /* CDC_HPH_GAIN_CTL */
		w_upd(0x3409, 0x01, 0x01);          /* DIG_CLK_CTL: RX0 */
		w_upd(0x30A4, 0x04, 0x00);          /* FLYBACK_EN */
	}
	/* RDAC4 (AUX DAC), PRE_PMU */
	w_upd(0x3408, 0x04, 0x04);
	/* Xiaomi's "AUX PATH Mode" HP_MODE (enum index 0, mixer_paths_bengal_idp speaker): AUX_AUXPA bit 4 and
	 * CDC_PATH_MODE bit 6 clear (mainline's AUX DAC sets bit 4; stock topaz reads 0x80 too) */
	w_upd(0x3128, 0x10, 0x00);
	w_upd(0x340B, 0x40, 0x00);
	w_upd(0x3409, 0x04, 0x04);
	w_upd(0x344F, 0x01, 0x01);                  /* CDC_AUX_GAIN_CTL */
	/* class-H v3, AUX then HPH, CLS_AB: buck and flyback (once), the HPH quiescent current */
	w_upd(0x3008, 0x08, 0x00);                  /* buck mode: default */
	w_upd(0x3008, 0x04, 0x00);                  /* flyback mode: default */
	w_upd(0x30A8, 0xF0, 0x70);                  /* FLYBACK_VNEG_CTRL_4 */
	w_upd(0x30A5, 0xE0, 0xE0);                  /* FLYBACK_VNEG_CTRL_1 */
	w_upd(0x3008, 0x40, 0x40);                  /* flyback on */
	IODelay(110);
	w_upd(0x30AF, 0xE0, 0xE0);                  /* FLYBACK_VNEGDAC_CTRL_2 */
	IODelay(550);
	w_upd(0x30C7, 0x0F, 0x0A);                  /* RX_BIAS_FLYB_BUFF */
	w_upd(0x30C7, 0xF0, 0xA0);
	IODelay(110);
	w_upd(0x3008, 0x80, 0x80);                  /* buck on */
	IODelay(510);
	w_wr(0x3098, 0x3A);                         /* CLASSH_MODE_2 */
	IODelay(550);
	/* the SoundWire data path */
	swr_ports_on(hph ? 3 : ear ? 1 : 0);
	w_upd(0x3009, 0x0C, 0x0C);                  /* ANA_HPH: CLS_AB (stock writes it for AUX too) */
	if (hph) {
		/* RDAC1 / RDAC2 POST_PMU, compander off */
		w_upd(0x3414, 0x03, 0x00);                  /* CDC_COMP_CTL_0 */
		w_upd(0x30D3, 0x20, 0x20);                  /* HPH_L_EN */
		w_upd(0x30D6, 0x20, 0x20);                  /* HPH_R_EN */
		w_upd(0x313A, 0x02, 0x00);                  /* HPH_NEW_INT_HPH_TIMER1 */
		/* HPHL / HPHR PGA, PRE_PMU: class-H HPH state (v3, CLS_AB) */
		w_upd(0x3008, 0x02, 0x00);                  /* buck regulator mode */
		w_upd(0x30AF, 0xE0, 0xA0);                  /* force IQ */
		IODelay(110);
		w_upd(0x3099, 0x02, 0x02);                  /* CLASSH_MODE_3 */
		w_wr(0x3098, 0x1C);                         /* CLASSH_MODE_2 */
		w_upd(0x30C7, 0x0F, 0x0A);
		w_upd(0x30C7, 0xF0, 0xA0);
		IODelay(110);
		w_upd(0x3009, 0x0C, 0x0C);                  /* ANA_HPH: CLS_AB */
		w_upd(0x3009, 0x30, 0x30);                  /* ANA_HPH: HPHL, HPHR references */
		IODelay(110);
		w_upd(0x3465, 0x07, 0x03);                  /* PDM watchdogs HPHL, HPHR */
		w_upd(0x3466, 0x07, 0x03);
		w_upd(0x3009, 0xC0, 0xC0);                  /* ANA_HPH: HPHL, HPHR PA */
	}
	/* the EAR PA: compander off, PDM watchdog of HPHL (its RX1 data), the PA */
	if (ear) {
		w_upd(0x300B, 0x80, 0x80);
		w_upd(0x3465, 0x07, 0x03);
		w_upd(0x300A, 0x80, 0x80);
	}
	/* the AUX PA (topaz-woa wcd_auxpa.ps1) */
	w_wr(0x3467, 0x05);                         /* PDM watchdog AUX */
	w_upd(0x3128, 0x80, 0x80);                  /* AUX PA */
	IOSleep(21);
	/* POST_PMU */
	if (hph) {
		w_upd(0x313A, 0x02, 0x02);
		w_upd(0x3008, 0x02, 0x02);          /* CLS_AB regulator mode */
	}
	w_upd(0x3008, 0x40, 0x40);                  /* AUX PA supplies */
	w_upd(0x3008, 0x80, 0x80);
	/* unmute */
	if (hph || ear) {
		c_upd(c_rx, 0x400, 0x10, 0x00);
	}
	if (hph) {
		c_upd(c_rx, 0x480, 0x10, 0x00);
	}
	c_upd(c_rx, 0x500, 0x10, 0x00);
	LOG("adsp: RX on (%s): WCD ANA_EAR %02x RX_SUPPLIES %02x ANA_HPH %02x AUXPA %02x HPH_L_EN %02x ANA_CLK %02x DIG_CLK %02x; RX0 %08x RX1 %08x RX2 %08x; SWR VA errors %u",
	    hph ? "headphones + bottom speaker" : "both speakers", w_rd(0x300A), w_rd(0x3008), w_rd(0x3009), w_rd(0x3128), w_rd(0x30D3), w_rd(0x3408), w_rd(0x3409), c_rd(c_rx, 0x400),
	    c_rd(c_rx, 0x480), c_rd(c_rx, 0x500), s_va.errors);
}

/* ---------------- the codec: the handset microphones (AMIC1 + AMIC3) to the TX macro's AIF1 capture */

/*
 * Android's "handset-mic" path (mixer_paths_bengal_idp.xml:2524-2540), written out as register writes:
 *   AMIC1 (main, bottom mic; "main-mic" = amic1, :2543) -> WCD937x ADC1 -> slave DP1 ch 1 -> VA SoundWire master
 *     port 2 ch 1 (SWRM_TX2_CH1) -> TX macro SWR_MIC4 -> TX SMIC MUX1 -> DEC1
 *   AMIC3 (top mic; "top-mic" = amic3, :2551) -> ADC2 MUX INP3 -> ADC2 -> slave DP2 ch 1 -> master port 3 ch 1
 *     (SWRM_TX3_CH1) -> SWR_MIC8 -> TX SMIC MUX0 -> DEC0
 *   DEC0 + DEC1 -> TX_AIF1_CAP -> ADSP TX_CODEC_DMA_TX_3 (2 channels, mask 0x3: ch 1 = DEC0 = top mic,
 *   ch 2 = DEC1 = main mic; the order is the decimator bit order of tx_macro_get_channel_map()).
 * There is no register for "TX_AIF1_CAP Mixer DECn": it only sets the channel mask the DMA reads.
 *
 * Sources (all checked, file:line or module:.text offset):
 *  [MP]  /home/angel/ios-build/vendor/extract/etc/mixer_paths_bengal_idp.xml
 *  [DT]  lunaris dtbo.img, overlay #45 (model "Qualcomm Technologies, Inc. Khaje IDP sapphire"), decompiled
 *        with dtc; line numbers are those of that .dts
 *  [SWR] swr_ctrl_dlkm.ko, [WCD] wcd937x_dlkm.ko, [TXM] tx_macro_dlkm.ko, [VAM] va_macro_dlkm.ko:
 *        /home/angel/ios-build/fox/rd/vendor/lib/modules/1.1/ (Xiaomi's own vendor build, disassembled)
 *  [ML]  /home/angel/linux-ref/{wcd937x.c,wcd937x.h,wcd937x-sdw.c,lpass-tx-macro.c,lpass-va-macro.c,qcom.c}
 *
 * Values and where they come from (vendor preferred over mainline where they differ):
 *  - MIC BIAS: AMIC1 <- MIC BIAS1, AMIC3 <- MIC BIAS3 ([DT] sound/qcom,audio-routing :3221);
 *    2700 mV for all three ([DT]:3178-3180 qcom,cdc-micbiasN-mv = 0xa8c) -> VOUT (2700-1000)/50 = 34 = 0x22
 *    ([ML] wcd937x.c:876 wcd937x_get_micb_vout_ctl_val; same formula in [WCD]).
 *  - Enable sequences: micbias [WCD] wcd937x_micbias_control .text 0xd2c (= [ML] wcd937x.c:1026),
 *    ADC [WCD] wcd937x_codec_enable_adc 0x310c, REQ [WCD] wcd937x_enable_req 0x32d0 (= [ML] wcd937x.c:902-990).
 *    Mainline's DAPM order: supplies (micbias) -> muxes (TX DEC MUX = tx_macro_enable_dec) -> mixers (ADC REQ)
 *    -> ADCs (whose PRE_PMU starts the SoundWire data path: swr_slvdev_datapath_control).
 *  - ADC1/ADC2 Volume 6: ANA_TX_CH1/CH2 bits 4:0 = 6 (6 dB) ([WCD] .data compound literal: reg 0x300e/0x300f,
 *    shift 0, max 20; [ML] wcd937x.c:2050). ADC2 MUX INP3: TX_NEW_TX_CH2_SEL (0x3127) bit 7 = 1
 *    ([WCD] .rodata 0x7d60 enum: reg 0x3127 shift 7, 2 items; [ML] wcd937x.c:2135-2145).
 *  - Slave TX ports: ADC1 = slave port idx 0 = DP1 ch mask 0x1, ADC2 = idx 1 = DP2 ch mask 0x1, ch rate 4.8 MHz
 *    ([DT]:3194 qcom,tx_swr_ch_map <0 ADC1 0x1 4800000 SWRM_TX1_CH1>, <1 ADC2 0x1 4800000 ...>; [ML] wcd937x-sdw.c:33,
 *    wcd937x.h WCD937X_ADC_1_PORT = 1, WCD937X_ADC_2_3_PORT = 2). "ADCn ChMap" only picks the MASTER port
 *    (no WCD register): SWRM_TX2_CH1 (0x32) -> master port 2 mask 0x1, SWRM_TX3_CH1 (0x36) -> port 3 mask 0x1
 *    ([DT]:3150 qcom,swr-port-mapping of va_swr_master).
 *  - Port parameters, use case 0 (bus clock 9.6 MHz): [SWR] swrm_copy_data_port_config 0x6dd8, TX-master branch
 *    0x6f6c-0x706c: si = 2 * bus_clk / ch_rate - 1 = 3; offset1 and lane from the slave's DT table, offset2 0,
 *    hstart/hstop/word length/block packing/block group = 0xff (= not written); [DT]:3193 qcom,swr-tx-port-params
 *    UC0 = DP1 (off1 1, lane 0), DP2 (off1 0, lane 1), DP3 (1, 0), DP4 (1, 0).
 *    Slave writes: 0x120+0x100*(n-1)+0x10*bank ch_en, +2 si, +4 off1, +5 off2, +8 lane (0x7150-0x7514).
 *    Master: offset1 = min over slaves = slave's; PORT_CTRL = ch<<24|off2<<16|off1<<8|si, SAMPLECTRL2 = si>>8,
 *    PORT_CTRL_2 = lane, HCTRL = 0xF0 because the TX master's hstart/hstop stay 0xff from probe
 *    ([SWR] swrm_probe 0x2e74-0x2edc and 0x7768-0x7788); BLOCK_CTRL_1/2/3 are left alone (0xff).
 *  - Frame for data on the VA bus: [SWR] swrm_slvdev_datapath_control 0x4850-0x4ad8 for master id 3 (TX):
 *    SCP_HOST_CLK_DIV2_CTL(new bank) = 1, ports into the inactive bank, MCP_FRAME_CTRL(new bank) = 50 rows
 *    (ctrl 1), 16 columns (ctrl 7), divider 0, SSP 6 (field 5), broadcast SCP_FRAME_CTRL(new bank) = 0x0F.
 *    That is exactly the shape swr_ports_on() gives the RX bus.
 *  - TX macro: SMIC MUX on ADC_MUXn_CFG0 bits 3:0, 13 items ZERO, SWR_MIC0..11 -> SWR_MIC8 = 9, SWR_MIC4 = 5;
 *    TX DECn MUX on ADC_MUXn_CFG1 bits 1:0, SWR_MIC = 1 ([TXM] .rodata enums 0xe660/0xe888).
 *    [TXM] tx_macro_put_dec_enum 0x2da0 for SMIC values >= 5: TXn_TX_PATH_CFG0 bit 7 = 1, VA 0x94 bit 7 = 0
 *    (register 0x3094 in the bolero map: VA base 0x3000, [DT] fs-gen-sequence 0x3000/0x3004/0x3080), and
 *    TX_TOP_CSR_SWR_DMICn_CTL ((2*val+0xb6)&~3: 0xC0 for val 5, 0xC8 for val 9) bits 3:1 = dmic_clk_div = 2
 *    ([DT]:3085 qcom,tx-dmic-sample-rate 2400000; [TXM] probe 0x6a4: 9.6 MHz / 2.4 MHz = 4 -> .rodata 0x17568[2] = 2).
 *    NOTE mainline (v9.2, lpass-tx-macro.c tx_macro_update_smic_sel_v9_2) would leave bit 7 at 0: vendor wins.
 *  - Decimator: [TXM] tx_macro_enable_dec 0x25d8 + hpf callback 0x22e8 + mute callback 0x2494, with
 *    bolero-version 5 ([DT]:3070) so is_amic_enabled() is true (0x2fa8: version 5 -> any SWR_MIC is "amic"):
 *    unmute after 100 ms, HPF 150 Hz while settling, back to the saved cut-off (CFG0 reset 0x10 -> 4 Hz) after
 *    300 ms (75 jiffies), SEC2 bits 1:0 = 2 then 1 (no 0 for amic). Rate 48 kHz = 4 in TX_PATH_CTL bits 3:0
 *    ([TXM] tx_macro_hw_params 0x3cf8, [ML] lpass-tx-macro.c:1124).
 *  - TX_DEC0/1 Volume 84: kcontrol min -84, max 40, sign bit 7 ([TXM] .data compound literal reg 0x40c/0x48c)
 *    -> register = 84 + (-84) = 0x00 = 0 dB (the default 90 would be +6 dB).
 *  - TX_TOP_CSR_FREQ_MCLK (0x90) bit 0 = 1: [TXM] tx_macro_mclk_enable 0x1610, before MCLK_CONTROL/FS_CNT
 *    (codec_up() does not write it). TX0_TX_PATH_SEC7 bits 5:0 = 0x0A: [TXM] tx_macro_init 0x1b34.
 *  - MICBn_TEST_CTL_2 bits 5:3 = 0: [ML] wcd937x.c:285-287 (io_init; codec_up() skips them).
 *
 * NOT verified (ranked by risk, highest first):
 *  1. The VA bus clock is 9.6 MHz (codec_up()/swrm_start() comment: "50 rows, 2 columns, SSP 48 (9.6 MHz bus,
 *     4 kHz frame sync)"). Use case 0 port params and si = 3 depend on it. With the vendor's dynamic bus clock the
 *     two 4.8 MHz ADCs add up to 9.6 MHz, so Android also ends up in use case 0 (connect_port 0x41ec-0x42e0).
 *     If only ONE ADC were used, Android would drop to 4.8 MHz / use case 1 (DP1 off1 0 lane 1) - not done here.
 *  2. The master-side registers vendor leaves untouched (BLOCK_CTRL_1 word length, BLOCK_CTRL2/3) are assumed
 *     to hold reset values; the slave's DPn hstart/hstop/word length/block ctrl are not written either (as vendor).
 *  3. Writing TX FREQ_MCLK while the TX MCLK (and the VA SoundWire bus) already runs: Android writes it before
 *     the MCLK enable. Better move it into codec_up() before c_wr(c_tx, 0x000, 1); here it is only written if
 *     still 0, and the VA bus is checked right after (chip id read + error count logged).
 *  4. "SWR DMIC" treatment of SWR_MIC4/8 (TX_PATH_CFG0 bit 7, SWR_DMIC0/2 clock 2.4 MHz) - copied from the vendor
 *     binary; it contradicts mainline v9.2. If the capture is silent/garbled, try bit 7 = 0 first.
 *  5. The ADSP's channel order (ch 1 = DEC0) and that the ADSP CODEC_DMA TX-3 needs nothing from the macro.
 *  6. Unmute/HPF timings are done synchronously (IOSleep 100 + 200 ms) instead of delayed work.
 */

static bool c_tx_on;

/* the WCD937x TX data ports (slave DP1 = ADC1, DP2 = ADC2) and VA master ports 2 and 3, then a bank switch */
static void
va_tx_ports(bool on)
{
	struct swrm *m = &s_va;
	uint8_t dev = w_tx_dev;
	uint32_t bank = (c_rd(m->b, 0x104C) & 1) ? 0 : 1;   /* MCP_STATUS: the inactive bank */
	uint32_t ch = on ? 1u : 0u;

	for (unsigned pass = 0; pass < 2; pass++) {
		uint32_t bk = 0x10 * bank, mb = 0x40 * bank;
		if (pass == 0) {
			swr_write(m, 0xF, (uint16_t)(0xE0 + bk), 0x01);    /* SCP_HOST_CLK_DIV2_CTL (broadcast) */
		}
		/* slave DP1 (ADC1): ch mask 1, si 3 (4.8 MHz at 9.6 MHz DDR), offset1 1, offset2 0, lane 0 */
		swr_write(m, dev, (uint16_t)(0x120 + bk), (uint8_t)ch);
		if (on) {
			swr_write(m, dev, (uint16_t)(0x122 + bk), 3);
			swr_write(m, dev, (uint16_t)(0x124 + bk), 1);
			swr_write(m, dev, (uint16_t)(0x125 + bk), 0);
			swr_write(m, dev, (uint16_t)(0x128 + bk), 0);
		}
		/* slave DP2 (ADC2): ch mask 1, si 3, offset1 0, offset2 0, lane 1 (the second data line, LPI pin 2) */
		swr_write(m, dev, (uint16_t)(0x220 + bk), (uint8_t)ch);
		if (on) {
			swr_write(m, dev, (uint16_t)(0x222 + bk), 3);
			swr_write(m, dev, (uint16_t)(0x224 + bk), 0);
			swr_write(m, dev, (uint16_t)(0x225 + bk), 0);
			swr_write(m, dev, (uint16_t)(0x228 + bk), 1);
		}
		/* master port 2 = SWRM_TX2 (SWR_MIC4..7): DP_PORT_CTRL ch << 24 | off2 << 16 | off1 << 8 | si */
		c_wr(m->b, 0x1124 + 0x100 * 1 + mb, (ch << 24) | (0u << 16) | (1u << 8) | 3u);
		c_wr(m->b, 0x113C + 0x100 * 1 + mb, 0);           /* SAMPLECTRL2: si >> 8 */
		c_wr(m->b, 0x1128 + 0x100 * 1 + mb, 0);           /* PORT_CTRL_2: lane 0 */
		c_wr(m->b, 0x1134 + 0x100 * 1 + mb, 0xF0);        /* HCTRL: hstop 15, hstart 0 (vendor default) */
		/* master port 3 = SWRM_TX3 (SWR_MIC8..11) */
		c_wr(m->b, 0x1124 + 0x100 * 2 + mb, (ch << 24) | (0u << 16) | (0u << 8) | 3u);
		c_wr(m->b, 0x113C + 0x100 * 2 + mb, 0);
		c_wr(m->b, 0x1128 + 0x100 * 2 + mb, 1);           /* lane 1 */
		c_wr(m->b, 0x1134 + 0x100 * 2 + mb, 0xF0);
		/* this bank's frame: 50 rows, 16 columns, divider 0, SSP period 6 (both banks, so a later switch keeps it) */
		uint32_t v = c_rd(m->b, 0x101C + mb) & ~0x00FF07FFu;
		c_wr(m->b, 0x101C + mb, v | (1u << 3) | 7u | (0u << 8) | (5u << 16));
		if (pass == 0) {
			swr_write(m, 0xF, (uint16_t)(0x60 + bk), (1 << 3) | 7);   /* SCP_FRAME_CTRL bank: switch */
			IOSleep(2);
			bank ^= 1;                              /* and the same into the other bank */
		}
	}
}

static void
codec_tx_diag(const char *when)
{
	struct swrm *m = &s_va;
	uint32_t nb = c_rd(m->b, 0x104C) & 1;
	uint8_t d1 = 0, d2 = 0, i1 = 0, i2 = 0;
	(void)swr_read(m, w_tx_dev, (uint16_t)(0x120 + 0x10 * nb), &d1);
	(void)swr_read(m, w_tx_dev, (uint16_t)(0x220 + 0x10 * nb), &d2);
	(void)swr_read(m, w_tx_dev, 0x100, &i1);           /* DP1_INTSTAT */
	(void)swr_read(m, w_tx_dev, 0x200, &i2);           /* DP2_INTSTAT */
	LOG("adsp: TX %s: WCD MICB1 %02x MICB3 %02x TX_CH1 %02x TX_CH2 %02x CH2_SEL %02x ANA_CLK %02x DIG_CLK %02x REQ %02x; slave DP1 ch %02x int %02x DP2 ch %02x int %02x",
	    when, w_rd(0x3022), w_rd(0x3025), w_rd(0x300E), w_rd(0x300F), w_rd(0x3127), w_rd(0x3408), w_rd(0x3409), w_rd(0x3457),
	    d1, i1, d2, i2);
	LOG("adsp: TX %s: DEC0 CTL %08x CFG0 %08x VOL %08x MUX %08x/%08x; DEC1 CTL %08x CFG0 %08x VOL %08x MUX %08x/%08x; FREQ_MCLK %08x DMIC0 %08x DMIC2 %08x",
	    when, c_rd(c_tx, 0x400), c_rd(c_tx, 0x404), c_rd(c_tx, 0x40C), c_rd(c_tx, 0x100), c_rd(c_tx, 0x104),
	    c_rd(c_tx, 0x480), c_rd(c_tx, 0x484), c_rd(c_tx, 0x48C), c_rd(c_tx, 0x108), c_rd(c_tx, 0x10C),
	    c_rd(c_tx, 0x090), c_rd(c_tx, 0x0C0), c_rd(c_tx, 0x0C8));
	/* 2nd round: the VA-side mic clock selects and DMIC cfg */
	LOG("adsp: TX %s: VA SWR_MIC_CTL %02x %02x %02x DMIC_CFG %02x SWR_CTRL %02x TOP_CFG0 %02x SWR_CONTROL %02x",
	    when, c_rd(c_va, 0x0D0), c_rd(c_va, 0x0D4), c_rd(c_va, 0x0D8), c_rd(c_va, 0x094), c_rd(c_va, 0x0DC),
	    c_rd(c_va, 0x080), c_rd(c_va, 0x008));
	LOG("adsp: TX %s: SoundWire VA bank %u frame %08x/%08x, port 2 %08x lane %08x, port 3 %08x lane %08x, intr %08x slaves %08x, errors %u",
	    when, nb, c_rd(m->b, 0x101C), c_rd(m->b, 0x105C), c_rd(m->b, 0x1224 + 0x40 * nb), c_rd(m->b, 0x1228 + 0x40 * nb),
	    c_rd(m->b, 0x1324 + 0x40 * nb), c_rd(m->b, 0x1328 + 0x40 * nb), c_rd(m->b, 0x200), c_rd(m->b, 0x1090), m->errors);
}

/*
 * The handset mics into TX_AIF1_CAP (DEC0 = AMIC3 via ADC2, DEC1 = AMIC1 via ADC1), 48 kHz. Android starts the
 * ADSP's CODEC_DMA TX-3 port (dai prepare) before this DAPM power-up; either order works for the codec.
 */
static void
codec_tx_on(void)
{
	if (w_tx_dev == 0 || c_tx == NULL || c_va == NULL || c_tx_on) {
		return;
	}
	unsigned e0 = s_va.errors;

	/* TX macro: MCLK frequency 9.6 MHz (vendor writes it before the MCLK enable; see assumption 3) */
	uint32_t fm = c_rd(c_tx, 0x090);
	if ((fm & 1) == 0) {
		c_upd(c_tx, 0x090, 0x01, 0x01);
		IODelay(100);
	}
	uint8_t id = w_rd(0x3401);                  /* DIGITAL_CHIP_ID0: is the VA bus still fine? */
	LOG("adsp: TX: FREQ_MCLK was %08x now %08x; WCD chip id0 %02x, VA errors %u -> %u", fm, c_rd(c_tx, 0x090), id, e0, s_va.errors);
	c_upd(c_tx, 0x42C, 0x3F, 0x0A);             /* TX0_TX_PATH_SEC7 (tx_macro_init) */
	/*
	 * FIX (2nd round, prime suspect for the all-zero capture): VA_TOP_CSR_SWR_MIC_CTL0/1/2 clock select of the
	 * SoundWire mic inputs. Reset value 0xEE (no clock, [ML] lpass-va-macro.c:286-288); the vendor writes
	 * mask 0xEE = 0xCC (DIV1) in va_macro_init when bolero-version == 5 ([VAM] .text 0x1854-0x189c; mainline
	 * va_macro probe, lpass-va-macro.c:1601-1611 "has_swr_master"). Without it the TX decimators see no
	 * SWR_MIC data. Belongs in codec_up() next to the VA fs-gen writes; idempotent here.
	 */
	{
		uint32_t m0 = c_rd(c_va, 0x0D0), m1 = c_rd(c_va, 0x0D4), m2 = c_rd(c_va, 0x0D8);
		c_upd(c_va, 0x0D0, 0xEE, 0xCC);
		c_upd(c_va, 0x0D4, 0xEE, 0xCC);
		c_upd(c_va, 0x0D8, 0xEE, 0xCC);
		LOG("adsp: TX: VA SWR_MIC_CTL0/1/2 were %02x %02x %02x now %02x %02x %02x", m0, m1, m2,
		    c_rd(c_va, 0x0D0), c_rd(c_va, 0x0D4), c_rd(c_va, 0x0D8));
	}

	/* WCD937x: io_init leftovers, micbias voltages 2700 mV (MICB1, MICB3) */
	w_upd(0x306C, 0x38, 0x00); w_upd(0x306F, 0x38, 0x00); w_upd(0x3072, 0x38, 0x00);   /* MICBn_TEST_CTL_2 */
	w_upd(0x3022, 0x3F, 0x22);                  /* ANA_MICB1 VOUT */
	w_upd(0x3025, 0x3F, 0x22);                  /* ANA_MICB3 VOUT */

	/* the mixer controls of "handset-mic" */
	w_upd(0x3127, 0x80, 0x80);                  /* ADC2 MUX = INP3 (AMIC3) */
	w_upd(0x300E, 0x1F, 16);                    /* ADC1 16 dB (Android: 6; heard too low in a call) */
	w_upd(0x300F, 0x1F, 16);                    /* ADC2 16 dB */
	/* TX SMIC MUX0 = SWR_MIC8 (9), MUX1 = SWR_MIC4 (5): tx_macro_put_dec_enum side effects, then the enum */
	c_upd(c_tx, 0x404, 0x80, 0x80);             /* TX0_TX_PATH_CFG0: ADC_DMIC_SEL (vendor, val >= 5) */
	c_upd(c_va, 0x094, 0x80, 0x00);             /* VA_TOP_CSR_DMIC_CFG: no "reset all DMICs" */
	c_upd(c_tx, 0x0C8, 0x0E, 2 << 1);           /* TX_TOP_CSR_SWR_DMIC2_CTL: clock 9.6 MHz / 4 */
	c_upd(c_tx, 0x100, 0x0F, 9);                /* ADC_MUX0_CFG0: SWR_MIC8 */
	c_upd(c_tx, 0x484, 0x80, 0x80);             /* TX1_TX_PATH_CFG0 */
	c_upd(c_va, 0x094, 0x80, 0x00);
	c_upd(c_tx, 0x0C0, 0x0E, 2 << 1);           /* TX_TOP_CSR_SWR_DMIC0_CTL */
	c_upd(c_tx, 0x108, 0x0F, 5);                /* ADC_MUX1_CFG0: SWR_MIC4 */
	c_upd(c_tx, 0x104, 0x03, 1);                /* TX DEC0 MUX = SWR_MIC (ADC_MUX0_CFG1) */
	c_upd(c_tx, 0x10C, 0x03, 1);                /* TX DEC1 MUX = SWR_MIC */
	c_wr(c_tx, 0x40C, 0x00);                    /* TX_DEC0 Volume 84 = 0 dB */
	c_wr(c_tx, 0x48C, 0x00);                    /* TX_DEC1 Volume 84 */
	/* hw_params: 48 kHz */
	c_upd(c_tx, 0x400, 0x0F, 4);
	c_upd(c_tx, 0x480, 0x0F, 4);

	/* MIC BIAS1 and MIC BIAS3 (supplies): wcd937x_micbias_control(MICB_ENABLE), then 1 ms (POST_PMU) */
	w_upd(0x3409, 0xF0, 0xF0);                  /* DIG_CLK_CTL: TX clocks */
	w_upd(0x3408, 0x10, 0x10);                  /* ANA_CLK_CTL */
	w_upd(0x306C, 0x01, 0x01); w_upd(0x306F, 0x01, 0x01); w_upd(0x3072, 0x01, 0x01);
	w_upd(0x3022, 0xC0, 0x40);                  /* MICB1 enable */
	IOSleep(1);
	w_upd(0x3025, 0xC0, 0x40);                  /* MICB3 enable */
	IOSleep(1);

	/* TX DEC0/DEC1 MUX (tx_macro_enable_dec): PRE_PMU, then POST_PMU */
	uint8_t hpf[2];
	for (unsigned k = 0; k < 2; k++) {
		uint32_t b = 0x400 + 0x80 * k;
		c_upd(c_tx, b + 0x04, 0x06, 0x00);      /* CFG0: ADC mode default (DEC MODE not set by the path) */
		c_upd(c_tx, b + 0x00, 0x10, 0x10);      /* PGA mute */
	}
	for (unsigned k = 0; k < 2; k++) {
		uint32_t b = 0x400 + 0x80 * k;
		c_upd(c_tx, b + 0x00, 0x20, 0x20);      /* decimator clock */
		hpf[k] = (uint8_t)((c_rd(c_tx, b + 0x04) >> 5) & 3);
		if (hpf[k] != 2) {
			c_upd(c_tx, b + 0x04, 0x60, 2 << 5);   /* HPF 150 Hz while it settles */
			c_upd(c_tx, b + 0x18, 0x03, 0x02);     /* SEC2: HPF frequency change */
			c_upd(c_tx, b + 0x18, 0x03, 0x01);
			IOSleep(6);
		}
		c_wr(c_tx, b + 0x0C, c_rd(c_tx, b + 0x0C));   /* gain after the decimator is on */
	}

	/* ADC1 REQ / ADC2 REQ (wcd937x_enable_req PRE_PMU, the same writes for both) */
	w_upd(0x3457, 0x02, 0x02);                  /* CDC_REQ_CTL */
	w_upd(0x3457, 0x01, 0x00);
	w_upd(0x300F, 0x40, 0x40);                  /* ANA_TX_CH2 */
	w_upd(0x3011, 0x40, 0x40);                  /* ANA_TX_CH3_HPF */
	w_upd(0x3409, 0x70, 0x70);
	w_upd(0x300E, 0x80, 0x80);                  /* ADC1 on */
	w_upd(0x300F, 0x40, 0x00);
	w_upd(0x300F, 0x80, 0x80);                  /* ADC2 on */
	w_upd(0x3010, 0x80, 0x80);                  /* ANA_TX_CH3 (vendor sets it too) */

	/* ADC1 / ADC2 PRE_PMU: clocks, then the SoundWire data path (swr_slvdev_datapath_control) */
	w_upd(0x3409, 0x80, 0x80);
	w_upd(0x3408, 0x08, 0x08);
	w_upd(0x3408, 0x10, 0x10);
	va_tx_ports(true);
	c_tx_on = true;

	/* unmute after 100 ms (amic), HPF back to its own cut-off after 300 ms */
	IOSleep(100);
	c_upd(c_tx, 0x400, 0x10, 0x00);
	c_upd(c_tx, 0x480, 0x10, 0x00);
	IOSleep(200);
	for (unsigned k = 0; k < 2; k++) {
		uint32_t b = 0x400 + 0x80 * k;
		if (hpf[k] != 2) {
			c_upd(c_tx, b + 0x04, 0x60, (uint32_t)hpf[k] << 5);
			c_upd(c_tx, b + 0x18, 0x03, 0x02);
			IOSleep(1);
			c_upd(c_tx, b + 0x18, 0x03, 0x01);
		}
	}
	codec_tx_diag("on");
}

static void
codec_tx_off(void)
{
	if (!c_tx_on) {
		return;
	}
	/* decimators: PRE_PMD mute */
	c_upd(c_tx, 0x400, 0x10, 0x10);
	c_upd(c_tx, 0x480, 0x10, 0x10);
	/* ADC POST_PMD, the data path off, REQ POST_PMD */
	w_upd(0x3408, 0x08, 0x00);
	va_tx_ports(false);
	w_upd(0x300E, 0x80, 0x00);
	w_upd(0x300F, 0x80, 0x00);
	w_upd(0x3010, 0x80, 0x00);
	w_upd(0x3409, 0x10, 0x00);
	w_upd(0x3408, 0x10, 0x00);
	w_upd(0x3409, 0x80, 0x00);
	/* micbias off */
	w_upd(0x3022, 0xC0, 0x00);
	w_upd(0x3025, 0xC0, 0x00);
	/* decimators: POST_PMD (clock off, mode 0, path reset pulse, mute cleared as vendor leaves it) */
	for (unsigned k = 0; k < 2; k++) {
		uint32_t b = 0x400 + 0x80 * k;
		c_upd(c_tx, b + 0x00, 0x20, 0x00);
		c_upd(c_tx, b + 0x04, 0x06, 0x00);
		c_upd(c_tx, b + 0x00, 0x40, 0x40);
		c_upd(c_tx, b + 0x00, 0x40, 0x00);
		c_upd(c_tx, b + 0x00, 0x10, 0x00);
	}
	c_tx_on = false;
	codec_tx_diag("off");
}

extern "C" int sweet_spmi_init(void);
extern "C" int sweet_spmi_read_any(unsigned sid, unsigned addr, uint8_t *out);

/* what each stage of the speaker path says while the tone plays */
static void
audio_diag(const char *when)
{
	char line[200];
	unsigned n = 0;
	for (unsigned a = 0x34; a <= 0x35; a++) {
		n = (unsigned)snprintf(line, sizeof(line), "FS1815 0x%02x:", a);
		static const uint8_t regs[] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x0e, 0x10, 0x11, 0x14, 0x16, 0x18, 0x19, 0x20, 0x22, 0x23 };
		for (unsigned i = 0; i < sizeof(regs) && n < sizeof(line) - 12; i++) {
			uint16_t v = 0;
			(void)fs_rd((uint8_t)a, regs[i], &v);
			n += (unsigned)snprintf(line + n, sizeof(line) - n, " %02x=%04x", regs[i], v);
		}
		LOG("adsp: %s: %s", when, line);
	}
	if (w_tx_dev != 0) {
		LOG("adsp: %s: WCD intr status %02x %02x %02x, RX_SUPPLIES %02x ANA_HPH %02x AUXPA %02x PDM_WD %02x %02x %02x; SWR RX intr %08x status %08x, VA intr %08x",
		    when, w_rd(0x346E), w_rd(0x346F), w_rd(0x3470), w_rd(0x3008), w_rd(0x3009), w_rd(0x3128), w_rd(0x3465), w_rd(0x3466), w_rd(0x3467),
		    c_rxswr ? c_rd(c_rxswr, 0x200) : 0, c_rxswr ? c_rd(c_rxswr, 0x1090) : 0, c_vaswr ? c_rd(c_vaswr, 0x200) : 0);
	}
	/* (0193: reading the PMIC over SPMI from here hung this thread: left out) */
}

static bool a_live;
static uint64_t a_live_buf, a_stream_pa;
static int16_t *a_stream_va;
extern "C" {
uint32_t sweet_audio_avail(void);
uint32_t sweet_audio_take(int16_t *mono, uint32_t frames);
void sweet_audio_flush(void);
void sweet_audio_dev_init(void);
void sweet_gpr_dev_init(void);
extern volatile uint32_t sweet_audio_vol_below, sweet_audio_vol_muted, sweet_audio_vol_gen;
extern volatile uint64_t sweet_audio_frames_in, sweet_audio_frames_dropped;
}

static void
audio_test(void)
{
	static const uint32_t macro[] = { 1 }, dcodec[] = { 2 };
	static const uint32_t rx_core[] = { 1, 0x30e, 19200000, 1, 0 }, rx_npl[] = { 1, 0x30f, 19200000, 1, 0 };
	int64_t r1, r2, r3, r4, r;
	uint64_t buf_pa = 0;
	int16_t *buf;
	const uint32_t bytes = A_PERIOD * 2;

	LOG("adsp: audio test begins (t=%u ms)", a_ms);
	crumbs_start();
	r1 = a_prm(PARAM_ID_RSC_HW_CORE, macro, 1);
	r2 = a_prm(PARAM_ID_RSC_HW_CORE, dcodec, 1);
	r3 = r4 = 0;
	if (a_flags & 1) {                          /* stock asks only for the TX clocks (codec_up()) */
		r3 = a_prm(PARAM_ID_RSC_AUDIO_HW_CLK, rx_core, 5);
		r4 = a_prm(PARAM_ID_RSC_AUDIO_HW_CLK, rx_npl, 5);
	}
	LOG("adsp: PRM: LPASS core vote %lld, digital codec vote %lld, RX core clock %lld, RX NPL clock %lld (box %08x %08x %08x)",
	    r1, r2, r3, r4, a_box.opcode, a_box.p[0], a_box.p[1]);
	if (r1 == 0 && r3 == 0) {
		volatile uint32_t *rx = (volatile uint32_t *)ml_io_map(0x0a600000, 0x1000);
		volatile uint32_t *swr = (volatile uint32_t *)ml_io_map(0x0a610000, 0x1000);
		if (rx != NULL && swr != NULL) {
			LOG("adsp: RX macro: %08x %08x %08x %08x; SoundWire master version %08x (device tree: 01060000)",
			    rx[0], rx[1], rx[2], rx[3], swr[0]);
		}
	} else {
		LOG("adsp: the codec's registers are not read without its clocks");
	}
	IOSleep(300);
	a_amp_probe();
	IOSleep(300);
	static bool rpm_done;
	if (!rpm_done) {
		rpm_done = true;
		IOLock *rl = rpm_lock_get();                    /* 0360: always under the lock */
		IOLockLock(rl);
		bool rpm = rpm_ensure();
		int act = rpm ? rpm_ldo(14, 1800000, 650, 0) : -9, slp = rpm ? rpm_ldo(14, 1800000, 650, 1) : -9;
		IOLockUnlock(rl);
		LOG("adsp: RPM: LDO14 (cdc-vdd-buck) on at 1.8 V: active set %d, sleep set %d (0: taken)", act, slp);
		IOSleep(10);
	}
	bool codec = codec_up();
	LOG("adsp: codec %s", codec ? "up" : "NOT up: the tone will go nowhere");
	IOSleep(300);

	const uint32_t tone = (A_NBUF * bytes + 0xfff) & ~0xfffu;   /* 0x2000: with the command page, 12 KB */
	/*
	 * 0173: hyp_assign of pool memory refused for every VM set (ret -2 res 0x56), while the
	 * modem's rmtfs buffer in the same pool is taken: the pool is not plain HLOS memory to the
	 * hypervisor everywhere. Android gives the ADSP ordinary kernel memory (msm_audio_ion, through
	 * SMMU stream 0x1c1, no assignment): so does this now -- physically contiguous below 4 GB,
	 * written through a write-combined mapping (the ADSP does not snoop our caches).
	 */
	/*
	 * Where the ADSP's buffers go: not our pool (0171 and 0177: the ADSP's first read of it reset
	 * the phone, wherever in the pool), but kernel memory, as Android does (that never reset
	 * anything), through SMMU stream 0x1c1; this XNU has it only above 4 GB.
	 */
	/*
	 * 0189: with the stream bits right (0x1_10000000) the ADSP's read through our stage-1 tables
	 * into kernel memory above 4 GB still reset the phone. What works for the WLAN engine works
	 * here too, it is to be hoped: the stream in bypass (smmu_identity(), as WLAN's 0x1a0) and
	 * memory below 4 GB, our pool's last 64 KB -- and 0174/0175, by chance stream bits 1 and a
	 * low address, read without a reset. The ADSP is given pa | 1 << 32.
	 */
	IOMemoryMap *bmap = NULL;
	buf_pa = MODEM_POOL_BASE + MODEM_POOL_SIZE - 0x10000;
	if (pool_used + 0x10000 <= MODEM_POOL_SIZE) {
		bmap = map_phys(buf_pa, 0x6000, true);    /* tone 8 KB, command page, iOS's stream 12 KB */
	}
	buf = bmap != NULL ? (int16_t *)bmap->getVirtualAddress() : NULL;
	if (buf == NULL) {
		LOG("adsp: no buffer (map %p pa %llx)", bmap, buf_pa);
		return;
	}
	bzero(buf, tone + 0x1000);
	LOG("adsp: %u bytes of buffers at %llx (the pool's end), write-combined", tone + 0x1000, buf_pa);
	/* 0190: an SMR with mask 0xf reads back 0 (refused): 0182-0189 had no stream for the ADSP at all */
	if (!smmu_identity(0x1c1, 0, "audio (ADSP buffers, stream 0x1c1)")) {
		LOG("adsp: no SMMU stream for the ADSP: the test stops here");
		return;
	}
	buf_pa |= A_SID_BITS;                       /* from here on, the address the ADSP uses */
	IOSleep(300);
	/* kept for the rest of the session: the ADSP may hold on to the mapping */
	for (unsigned i = 0; i < A_NBUF * A_PERIOD; i++) {
		/* 1 kHz at 48 kHz: 48 samples a period; a triangle near full scale (the buffers alternate tone / silence below) */
		int ph = (int)(i % 48);
		int tri = ph < 24 ? ph * 2 - 24 : 72 - ph * 2;   /* -24..24 */
		buf[i] = (int16_t)(tri * 1200);
	}
	bzero(buf + 4 * A_PERIOD, 4 * A_PERIOD * sizeof(int16_t));   /* buffers 4-7 silent: beeps */
	mb();
	if (a_mem_handle != 0) {
		/* 0196: a second map of the same region is refused (status 1): keep the first */
		LOG("adsp: shared memory %llx already mapped, handle %x", buf_pa, a_mem_handle);
	} else {
		uint8_t m[20];
		uint8_t *p = m;
		a_put16(&p, 3);                                       /* APM_MEMORY_MAP_SHMEM8_4K_POOL */
		a_put16(&p, 1);
		a_put32(&p, 0);                                       /* physical addresses */
		a_put32(&p, (uint32_t)buf_pa); a_put32(&p, (uint32_t)(buf_pa >> 32));
		a_put32(&p, tone + 0x4000);
		a_gpr_pkt(GPR_PORT_APM_CLIENT, APM_PORT, APM_CMD_SHARED_MEM_MAP_REGIONS, 1, m, sizeof(m));
		r = a_wait(APM_CMD_SHARED_MEM_MAP_REGIONS, APM_CMD_RSP_SHARED_MEM_MAP_REGIONS, 2000);
		a_mem_handle = r == 0 ? a_box.p[0] : 0;
		a_cmd_area = (uint8_t *)buf + tone;
		a_cmd_pa = buf_pa + tone;
		LOG("adsp: shared memory %llx (%u bytes): map %lld, handle %x", buf_pa, A_NBUF * bytes, r, a_mem_handle);
		if (r != 0) {
			return;
		}
	}
	a_verbose = false;
	a_oob = true;
	IOSleep(300);                                           /* (the USB log goes out every 100 ms) */
	/*
	 * 0178: the subgraph was taken; a container with "capability" 2 was answered status 1 and
	 * then the ADSP's audio process died in that container's thread (err_qdi.c EX:audio_process
	 * GC_5001): this SPF takes the field as a container type, as Android's ACDB has it -- the
	 * generic container 0x0B001001 throughout.
	 */
	if (s1_cbp != NULL) {
		LOG("adsp: SMMU bank before the first read: FSR %08x", r32(s1_cbp, 0x58));
		IOSleep(300);
	}
	r = a_graph_open(0x0B001001, 0x0B001001);
	a_verbose = false;
	LOG("adsp: graph open -> %lld", r);
	if (s1_cbp != NULL) {
		LOG("adsp: SMMU bank: FSR %08x FAR %08x%08x FSYNR0 %08x", r32(s1_cbp, 0x58), r32(s1_cbp, 0x64), r32(s1_cbp, 0x60),
		    r32(s1_cbp, 0x68));
	}
	if (r != 0) {
		return;
	}
	r = a_formats();
	r1 = a_graph_mgmt(APM_CMD_GRAPH_PREPARE);
	r2 = a_graph_mgmt(APM_CMD_GRAPH_START);
	LOG("adsp: formats %lld, prepare %lld, start %lld", r, r1, r2);
	if (r1 != 0 || r2 != 0) {
		(void)a_graph_mgmt(APM_CMD_GRAPH_CLOSE);
		return;
	}
	if (codec) {
		codec_rx_on();
		IOSleep(200);
		bool a = (a_flags & 8) || fs_init(0x34, 0x000b), b = (a_flags & 8) || fs_init(0x35, 0x000e);
		LOG("adsp: speaker amplifiers: 0x34 %s, 0x35 %s", a ? "on" : "not found", b ? "on" : "not found");
		IOSleep(200);
	}
	unsigned sent = 0, t0 = a_ms;
	a_writes_done = 0;
	/* 0200: the beeps only with audio=3; audio=2 leaves everything ready for iOS's sound */
	for (; audio_on >= 3 && sent < 4; sent++) {
		a_write(buf_pa + (sent % A_NBUF) * bytes, bytes, sent % A_NBUF);
	}
	for (unsigned ms = 0; audio_on >= 3 && ms < 15000; ms++) {
		a_service();
		while (sent < a_writes_done + 4 && sent < 1500) {
			unsigned k = (sent / 50) % 2 == 0 ? sent % 4 : 4 + sent % 4;   /* 0.5 s tone, 0.5 s silence */
			a_write(buf_pa + k * bytes, bytes, k);
			sent++;
		}
		if (ms % 1000 == 999) {
			LOG("adsp: t+%u ms: %u buffers written, %u returned", a_ms - t0, sent, a_writes_done);
		}
		if (ms == 8000 && codec) {
			audio_diag("playing");
		}
		IOSleep(1);
		a_ms++;
	}
	LOG("adsp: tone done: %u of %u buffers came back in %u ms (10 ms each)", a_writes_done, sent, a_ms - t0);
	if (s1_cbp != NULL) {
		LOG("adsp: SMMU bank: FSR %08x FAR %08x%08x FSYNR0 %08x", r32(s1_cbp, 0x58), r32(s1_cbp, 0x64), r32(s1_cbp, 0x60),
		    r32(s1_cbp, 0x68));
	}
	r1 = a_graph_mgmt(APM_CMD_GRAPH_STOP);
	a_live = true;                              /* open and stopped: kern.sweet_audio beeps start it again */
	a_live_buf = buf_pa;
	a_stream_va = (int16_t *)((uint8_t *)buf + tone + 0x1000);
	a_stream_pa = buf_pa + tone + 0x1000;
	LOG("adsp: stop %lld (the graph stays open for iOS's sound and kern.sweet_audio)", r1);
	sweet_audio_dev_init();
	sweet_gpr_dev_init();
}

/*
 * iOS's sound: what RedwoodAudio.driver writes to /dev/redwood-audio (bsd/dev/sweet_audio_dev.c),
 * 10 ms at a time into the open graph, 4 periods ahead of the DSP. The graph starts when 20 ms
 * have come and stops after 0.5 s with nothing; the volume iOS sets goes to the RX macro's
 * digital gains (EAR INT0, AUX INT2; the headphones 20 dB under).
 */
static bool s_playing;
static unsigned s_sent, s_quiet;
static uint32_t s_vol_gen = ~0u;

static void
stream_volume(void)
{
	int db = sweet_audio_vol_muted ? -84 : -(int)(sweet_audio_vol_below / 8);
	if (db < -84) {
		db = -84;
	}
	s_vol_gen = sweet_audio_vol_gen;
	if (c_rx == NULL) {
		return;
	}
	int hdb = MAX(db - 20, -84);
	c_wr(c_rx, 0x414, (uint32_t)((a_flags & 4) ? hdb : db) & 0xFF);
	c_wr(c_rx, 0x494, (uint32_t)hdb & 0xFF);
	c_wr(c_rx, 0x514, (uint32_t)db & 0xFF);
}

static void
stream_service(void)
{
	const uint32_t bytes = A_PERIOD * 2;
	if (!a_live || a_stream_va == NULL) {
		return;
	}
	if (!s_playing) {
		if (sweet_audio_avail() < 2 * A_PERIOD) {
			return;
		}
		int64_t r1 = a_graph_mgmt(APM_CMD_GRAPH_PREPARE), r2 = a_graph_mgmt(APM_CMD_GRAPH_START);
		if (r2 != 0) {
			LOG("adsp: iOS sound: start failed (prepare %lld, start %lld)", r1, r2);
			sweet_audio_flush();
			return;
		}
		s_playing = true;
		s_sent = 0;
		s_quiet = 0;
		a_writes_done = 0;
		stream_volume();
		LOG("adsp: iOS sound: playing (%llu frames so far, %llu dropped)", sweet_audio_frames_in, sweet_audio_frames_dropped);
	}
	if (s_vol_gen != sweet_audio_vol_gen) {
		stream_volume();
	}
	while (s_sent < a_writes_done + 4) {
		unsigned k = s_sent % A_NBUF;
		int16_t *d = a_stream_va + k * A_PERIOD;
		uint32_t n = sweet_audio_take(d, A_PERIOD);
		if (n < A_PERIOD) {
			bzero(d + n, (A_PERIOD - n) * sizeof(int16_t));
		}
		s_quiet = n == 0 ? s_quiet + 1 : 0;
		mb();
		a_write(a_stream_pa + k * bytes, bytes, 0x100 + k);
		s_sent++;
	}
	if (s_quiet >= 50) {                        /* 0.5 s of nothing */
		int64_t r = a_graph_mgmt(APM_CMD_GRAPH_STOP);
		s_playing = false;
		LOG("adsp: iOS sound: stopped (stop %lld; %llu frames in, %llu dropped)", r, sweet_audio_frames_in,
		    sweet_audio_frames_dropped);
	}
}

/* kern.sweet_audio op 1: N seconds of beeps on the open graph */
static void
audio_beep(unsigned seconds)
{
	const uint32_t bytes = A_PERIOD * 2;
	int64_t r1 = a_graph_mgmt(APM_CMD_GRAPH_PREPARE), r2 = a_graph_mgmt(APM_CMD_GRAPH_START);
	LOG("adsp: beep %u s: prepare %lld, start %lld", seconds, r1, r2);
	if (r2 != 0) {
		return;
	}
	unsigned sent = 0, n = seconds * 100, t0 = a_ms;
	a_writes_done = 0;
	for (; sent < 4; sent++) {
		a_write(a_live_buf + (sent % A_NBUF) * bytes, bytes, sent % A_NBUF);
	}
	for (unsigned ms = 0; ms < seconds * 1000 + 100; ms++) {
		a_service();
		while (sent < a_writes_done + 4 && sent < n) {
			unsigned k = (sent / 50) % 2 == 0 ? sent % 4 : 4 + sent % 4;
			a_write(a_live_buf + k * bytes, bytes, k);
			sent++;
		}
		if (ms == 1500 && w_tx_dev != 0) {
			audio_diag("beep");
		}
		IOSleep(1);
		a_ms++;
	}
	r1 = a_graph_mgmt(APM_CMD_GRAPH_STOP);
	LOG("adsp: beep done: %u of %u buffers back in %u ms, stop %lld", a_writes_done, sent, a_ms - t0, r1);
}

/*
 * ---------------- /dev/redwood-gpr (bsd/dev/sweet_gpr_dev.c): AudioReach packets from user space
 *
 * A voice call's graphs are what Android's GSL builds from the ACDB (graph opens of 4 KB, VCPM
 * calibration tables of up to 45 KB that the DSP reads for as long as the graph is open): a tool
 * replays them (userland/audio/arplay). Their memory is the 192 KB under the playback's 64 KB at
 * the pool's end, given to the DSP by a second map (the same stream 0x1c1, in bypass).
 */
struct sweet_gpr_cmd {                          /* as bsd/dev/sweet_gpr_dev.c */
	uint32_t op, arg;
	uint32_t src_port, dst_port, opcode, token, rsp_opcode, wait_ms;
	uint32_t flags, oob_offset, oob_len, len;
	int32_t status;
	uint32_t got_opcode, got_n, got[8];
	uint8_t payload[512];
};
extern "C" {
extern struct sweet_gpr_cmd sweet_gpr_req;
extern volatile uint32_t sweet_gpr_seq, sweet_gpr_done;
extern uint8_t *volatile sweet_gpr_va;
extern volatile uint32_t sweet_gpr_size;
}
#define G_SIZE  0x30000u
static uint64_t g_pa;                           /* as the DSP sees it (A_SID_BITS) */
static uint32_t g_handle;

static int32_t
gpr_setup(void)
{
	if (g_handle != 0) {
		return 0;
	}
	if (!a_spf_ready || a_mem_handle == 0) {
		return -2;                              /* not before the playback's test has mapped its memory */
	}
	uint64_t pa = MODEM_POOL_BASE + MODEM_POOL_SIZE - 0x10000 - G_SIZE;
	if (pool_used > pa - MODEM_POOL_BASE) {
		LOG("adsp: gpr: the pool reaches %llx, over %llx", MODEM_POOL_BASE + pool_used, pa);
		return -4;
	}
	IOMemoryMap *m = map_phys(pa, G_SIZE, true);
	uint8_t *va = m != NULL ? (uint8_t *)m->getVirtualAddress() : NULL;
	if (va == NULL) {
		return -5;
	}
	bzero(va, G_SIZE);
	mb();
	pa |= A_SID_BITS;
	uint8_t b[20];
	uint8_t *p = b;
	a_put16(&p, 3);                                 /* APM_MEMORY_MAP_SHMEM8_4K_POOL, one region */
	a_put16(&p, 1);
	a_put32(&p, 0);
	a_put32(&p, (uint32_t)pa); a_put32(&p, (uint32_t)(pa >> 32));
	a_put32(&p, G_SIZE);
	a_gpr_pkt(GPR_PORT_APM_CLIENT, APM_PORT, APM_CMD_SHARED_MEM_MAP_REGIONS, 2, b, sizeof(b));
	int64_t r = a_wait(APM_CMD_SHARED_MEM_MAP_REGIONS, APM_CMD_RSP_SHARED_MEM_MAP_REGIONS, 2000);
	LOG("adsp: gpr: %u KB at %llx for the DSP: map %lld, handle %x", G_SIZE / 1024, pa, r, r == 0 ? a_box.p[0] : 0);
	if (r != 0 || a_box.p[0] == 0) {
		return r != 0 ? (int32_t)r : -6;
	}
	g_pa = pa;
	g_handle = a_box.p[0];
	sweet_gpr_size = G_SIZE;
	mb();
	sweet_gpr_va = va;
	return 0;
}

static void
gpr_cmd(struct sweet_gpr_cmd *c)
{
	uint8_t pl[512];

	if (c->len > sizeof(pl) || ((c->flags & 1) && (c->len < 16 || g_handle == 0 || c->oob_offset > G_SIZE ||
	    c->oob_len > G_SIZE - c->oob_offset))) {
		c->status = -7;
		return;
	}
	memcpy(pl, c->payload, c->len);
	if (c->flags & 1) {
		uint64_t a = g_pa + c->oob_offset;
		uint32_t h[4] = { (uint32_t)a, (uint32_t)(a >> 32), g_handle, c->oob_len };
		memcpy(pl, h, sizeof(h));
	}
	mb();
	a_box.full = false;
	a_gpr_pkt(c->src_port, c->dst_port, c->opcode, c->token, pl, c->len);
	int64_t r = a_wait(c->opcode, c->rsp_opcode, c->wait_ms != 0 ? MIN(c->wait_ms, 8000u) : 2000);
	c->status = (int32_t)r;
	c->got_opcode = a_box.opcode;
	c->got_n = a_box.n;
	for (unsigned i = 0; i < 8; i++) {
		c->got[i] = i < a_box.n ? a_box.p[i] : 0;
	}
	LOG("adsp: gpr: %08x to %x (%u bytes%s) -> %d", c->opcode, c->dst_port, c->len, (c->flags & 1) ? ", out of band" : "", c->status);
}

/* iOS's playback graph off while a call has the codec DMA interface, back on after */
static int32_t
gpr_playback(bool on)
{
	if (!on) {
		if (s_playing) {
			(void)a_graph_mgmt(APM_CMD_GRAPH_STOP);
			s_playing = false;
		}
		if (a_live) {
			int64_t r = a_graph_mgmt(APM_CMD_GRAPH_CLOSE);
			a_live = false;
			LOG("adsp: gpr: iOS's playback graph closed: %lld", r);
			return (int32_t)r;
		}
		return 0;
	}
	if (a_live || a_live_buf == 0) {
		return 0;
	}
	int64_t r = a_graph_open(0x0B001001, 0x0B001001);
	if (r == 0) {
		r = a_formats();
	}
	a_live = r == 0;
	sweet_audio_flush();
	s_vol_gen = ~0u;
	LOG("adsp: gpr: iOS's playback graph open again: %lld", r);
	return (int32_t)r;
}

/*
 * /dev/redwood-mic (bsd/dev/sweet_gpr_dev.c): the record graph arplay opened and prepared (op 5:
 * its subgraph list, its RD_SHARED_MEM_EP, where its buffers go), started while the device is
 * open: 4 buffers of 20 ms read in turn, each one's two channels (the handset's two microphones,
 * DEC0 and DEC1) summed into the device's one.
 */
extern "C" {
extern volatile uint32_t sweet_mic_want;
void sweet_mic_put(const int16_t *s, uint32_t n);
}
#define M_NBUF  4u
#define M_BYTES 3840u
static struct { uint32_t src, ep, off, len; uint8_t start[128]; } m_cfg;
static bool m_running;
static uint32_t m_back;                         /* bit i: buffer i came back */
static unsigned m_filled, m_bad;

static void
a_mic_done(const uint32_t *pl, uint32_t pn)
{
	if (pn < 5 || !m_running || sweet_gpr_va == NULL) {
		return;
	}
	uint64_t a = ((uint64_t)pl[2] << 32) | pl[1], base = g_pa + m_cfg.off;
	if (a < base || (a - base) % M_BYTES != 0 || (a - base) / M_BYTES >= M_NBUF) {
		m_bad++;
		return;
	}
	unsigned i = (unsigned)((a - base) / M_BYTES);
	if (pl[0] == 0) {
		const int16_t *s = (const int16_t *)(sweet_gpr_va + m_cfg.off + i * M_BYTES);
		uint32_t frames = MIN(pl[4], M_BYTES) / 4;
		int16_t mono[M_BYTES / 4];
		for (uint32_t j = 0; j < frames; j++) {
			int32_t v = (int32_t)s[2 * j] + s[2 * j + 1];
			mono[j] = (int16_t)(v > 32767 ? 32767 : v < -32768 ? -32768 : v);
		}
		sweet_mic_put(mono, frames);
		m_filled++;
	} else {
		m_bad++;
	}
	m_back |= 1u << i;
}

static void
mic_read(unsigned i)
{
	uint64_t a = g_pa + m_cfg.off + i * M_BYTES;
	uint32_t w[8] = { (uint32_t)a, (uint32_t)(a >> 32), g_handle, M_BYTES, 0, 0, 0, 0 };
	a_gpr_pkt(m_cfg.src, m_cfg.ep, 0x0400100Bu, i, w, sizeof(w));   /* DATA_CMD_RD_SH_MEM_EP_DATA_BUFFER_V2 */
}

static int64_t
mic_graph(uint32_t opcode)
{
	a_box.full = false;
	a_gpr_pkt(m_cfg.src, APM_PORT, opcode, 0, m_cfg.start, m_cfg.len);
	return a_wait(opcode, 0, 2000);
}

static void
mic_service(void)
{
	if (m_cfg.len == 0) {
		return;
	}
	if (sweet_mic_want && !m_running) {
		int64_t r = mic_graph(APM_CMD_GRAPH_START);
		LOG("adsp: mic: record graph started: %lld", r);
		if (r != 0) {
			m_cfg.len = 0;                  /* not again until arplay sets it up again */
			return;
		}
		m_running = true;
		m_back = 0;
		m_filled = m_bad = 0;
		for (unsigned i = 0; i < M_NBUF; i++) {
			mic_read(i);
		}
	} else if (!sweet_mic_want && m_running) {
		int64_t r = mic_graph(APM_CMD_GRAPH_STOP);
		m_running = false;
		LOG("adsp: mic: stopped (%lld): %u buffers filled, %u refused", r, m_filled, m_bad);
	}
	if (m_running && m_back != 0) {
		uint32_t b = m_back;
		m_back = 0;
		for (unsigned i = 0; i < M_NBUF; i++) {
			if (b & (1u << i)) {
				mic_read(i);
			}
		}
	}
}

static void codec_tx_on(void);
static void codec_tx_off(void);

static void
gpr_request(void)
{
	if (sweet_gpr_seq == sweet_gpr_done) {
		return;
	}
	mb();
	struct sweet_gpr_cmd *c = &sweet_gpr_req;
	switch (c->op) {
	case 1:
		c->status = gpr_setup();
		c->got[0] = g_handle;
		c->got[1] = G_SIZE;
		break;
	case 2:
		gpr_cmd(c);
		break;
	case 3:
		c->status = gpr_playback(c->arg != 0);
		break;
	case 4:
		if (c_rx != NULL) {
			c_wr(c_rx, 0x414, c->arg & 0xFF);
			c_wr(c_rx, 0x514, (c->arg >> 8) & 0xFF);
			c->status = 0;
		} else {
			c->status = -2;
		}
		break;
	case 5:
		if (m_running) {
			(void)mic_graph(APM_CMD_GRAPH_STOP);
			m_running = false;
		}
		m_cfg.len = 0;
		if (c->len > sizeof(m_cfg.start) || c->arg > G_SIZE - M_NBUF * M_BYTES || g_handle == 0) {
			c->status = -7;
			break;
		}
		m_cfg.src = c->src_port;
		m_cfg.ep = c->dst_port;
		m_cfg.off = c->arg;
		memcpy(m_cfg.start, c->payload, c->len);
		m_cfg.len = c->len;
		c->status = 0;
		LOG("adsp: mic: record graph %s (endpoint %x, buffers at %x)", c->len ? "set up" : "gone", c->dst_port, c->arg);
		break;
	case 6:
		if (w_tx_dev == 0) {
			c->status = -2;
			break;
		}
		if (c->arg) {
			codec_tx_on();
		} else {
			codec_tx_off();
		}
		c->status = 0;
		break;
	default:
		c->status = -22;
		break;
	}
	mb();
	sweet_gpr_done = sweet_gpr_seq;
}

extern "C" {
extern volatile uint64_t sweet_audio_req, sweet_audio_rsp;
extern volatile uint32_t sweet_audio_seq, sweet_audio_done;
}

/* the TLMM tile of each GPIO (pinctrl-sm6115.c): S south 0x900000, E east 0xd00000, W west 0x500000 */
static const char tlmm_tile[] = "WWWWWWWWEEEEWWWWWWEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEWWWWWWWSSSSSSSSWWWWWWWEEWEEWWWWWWSSSSSSSSSSSSSSS";

/* one kern.sweet_audio request (sweet_audio_ctl.c), on this thread */
static void
audio_request(void)
{
	if (sweet_audio_seq == sweet_audio_done) {
		return;
	}
	mb();
	uint64_t q = sweet_audio_req, rsp = 0;
	uint32_t op = (uint32_t)(q >> 56), tg = (uint32_t)(q >> 48) & 0xFF, ad = (uint32_t)(q >> 32) & 0xFFFF, v = (uint32_t)q;
	bool wr = op == 2;
	volatile uint8_t *mm = tg == 2 ? c_rx : tg == 3 ? c_va : tg == 4 ? c_rxswr : tg == 5 ? c_vaswr : NULL;
	switch (op) {
	case 1:
		if (a_live && !s_playing) {
			audio_beep(MIN(MAX(v, 1u), 20u));
		} else {
			LOG("adsp: beep: no graph (the test at 45 s did not get that far)");
			rsp = ~0ull;
		}
		break;
	case 2:
	case 3:
		if (tg == 1 && w_tx_dev != 0) {
			if (wr) {
				w_wr((uint16_t)ad, (uint8_t)v);
			}
			rsp = w_rd((uint16_t)ad);
		} else if (tg == 7 && w_rx_dev != 0) {
			uint8_t b = 0;
			if (wr) {
				swr_write(&s_rx, w_rx_dev, (uint16_t)ad, (uint8_t)v);
			}
			rsp = swr_read(&s_rx, w_rx_dev, (uint16_t)ad, &b) == 0 ? b : ~0ull;
		} else if (mm != NULL && ad < (tg >= 4 ? 0x2000u : 0x1000u)) {
			if (wr) {
				c_wr(mm, ad & ~3u, v);
			}
			rsp = c_rd(mm, ad & ~3u);
		} else if ((tg == 8 || tg == 9) && ad < sizeof(tlmm_tile) - 1) {
			/* TLMM pin ad: 8 writes its ctl, 9 its io (bit 1 the output); the answer io << 32 | ctl */
			uint64_t base = tlmm_tile[ad] == 'S' ? 0x00900000ull : tlmm_tile[ad] == 'E' ? 0x00D00000ull : 0x00500000ull;
			volatile uint32_t *g = (volatile uint32_t *)ml_io_map(base + ad * 0x1000ull, 0x1000);
			if (g != NULL) {
				if (wr) {
					g[tg == 8 ? 0 : 1] = v;
					mb();
				}
				rsp = ((uint64_t)g[1] << 32) | g[0];
			} else {
				rsp = ~0ull;
			}
		} else if (tg == 6) {
			uint16_t x = 0;
			if (wr) {
				(void)fs_wr((uint8_t)(ad >> 8), (uint8_t)ad, (uint16_t)v);
			}
			rsp = fs_rd((uint8_t)(ad >> 8), (uint8_t)ad, &x) == 0 ? x : ~0ull;
		} else {
			rsp = ~0ull;
		}
		LOG("adsp: sweet_audio %s target %u %04x%s -> %llx", wr ? "write" : "read", tg, ad, wr ? "" : "", rsp);
		break;
	case 4:
		if (a_live) {
			(void)a_graph_mgmt(APM_CMD_GRAPH_CLOSE);
			a_live = false;
		}
		audio_test();
		rsp = a_live;
		break;
	case 6: {
		/*
		 * an RPM vote: target = resource type (1 "clk2" memory, 2 "clk1" bus, 3 "clk0" misc, 4 "qup",
		 * 5 "ipa", 6 "mmxi", 7 "ce"), addr = id | set << 15, value: kHz (misc: "Enab" value)
		 */
		static const uint32_t types[] = { 0, 0x326b6c63, 0x316b6c63, 0x306b6c63, 0x00707571, 0x00617069, 0x69786d6d,
			0x00006563, 0x78637772 };
		if (tg >= 1 && tg <= 8) {
			/* (8 "rwcx": the CX rail, key "vlvl": an RPM level, 0x180 TURBO, 0x1a0 TURBO_L1) */
			uint32_t key = tg == 3 ? 0x62616e45 : tg == 8 ? 0x6c766c76 : 0x007a484b;
			int r = topaz_rpm_vote((ad >> 15) & 1, types[tg], ad & 0x7fff, key, v);
			LOG("adsp: RPM vote %.4s %u set %u = %u: %d", (const char *)&types[tg], ad & 0x7fff, (ad >> 15) & 1, v, r);
			rsp = (uint64_t)(int64_t)r;
		} else {
			rsp = ~0ull;
		}
		break;
	}
	case 5:
		a_flags = v;
		rsp = a_flags;
		LOG("adsp: sweet_audio flags %x", a_flags);
		break;
	default:
		rsp = ~0ull;
		break;
	}
	sweet_audio_rsp = rsp;
	mb();
	sweet_audio_done = sweet_audio_seq;
}

/* ---------------- FastRPC to the ADSP's sensors PD (0265-0267) */

#include <stdarg.h>

/*
 * The ADSP's sensor core (SEE) lives in a static protection domain, "sensor_pd". Its SNS_CLIENT
 * service (QMI 0x190) answers, but only the timer sensor exists until an HLOS process attaches to
 * that PD over FastRPC and serves its reverse calls: on Android that is `sscrpcd sensorspd`
 * (libssc_default_listener: FASTRPC_INIT_ATTACH_SNS, then the adsp_listener loop through which
 * the PD's registry reads /vendor/etc/sensors/config and /mnt/vendor/persist/sensors/registry).
 *
 * Step 1, here: the GLINK channel "fastrpcglink-apps-dsp", the attach, adsp_listener init2 and
 * next2 (the reverse-call loop), adsp_default_listener register; every reverse call the PD makes
 * is logged and answered "unsupported" for now. Protocol as Linux drivers/misc/fastrpc.c:
 *   message  {pid, tid, ctx = index << 4 | pd (2: sensors), handle, sc, addr, size} -> {ctx, retval}
 *   the buffer at addr: remote args {pv, len} x n, {num, pgidx} x n, pages {addr, size} x n,
 *   fd list (16 x u64), crc list (64 x u32), then each argument 128-byte aligned.
 * Buffers: 64 KB of the modem pool below 4 GB, just under the audio buffers (the ADSP's reads of
 * memory above 4 GB through stage-1 tables reset the phone, 0189), stream 0x1c3 (compute cb10,
 * "adsprpc-smd") in bypass, the context bank's number in address bits 32+ (Linux: sid << 32), as
 * the audio buffers do with 0x1c1.
 * Started by kern.sweet_frpc = 1 (or boot-arg frpc=1, 45 s into the ADSP's life); log in
 * kern.sweet_frpc_log.
 */
#define FR_SID          0x1c3u
#define FR_SID_BITS     ((uint64_t)(FR_SID & 0xf) << 32)
#define FR_MEM_OFF      (MODEM_POOL_SIZE - 0x50000)
#define FR_SLOTS        2
#define FR_SLOT_OFF(s)  ((s) == 1 ? 0u : 0x4000u)     /* 0267: slot 1 16 KB, slot 2 (next2) 48 KB */
#define FR_SLOT_LEN(s)  ((s) == 1 ? 0x4000u : 0xC000u)
#define FR_PD_SENSORS   2
#define FR_TGID         0x5e0
#define FR_BUFS         0x4000u                 /* next2's receive buffer */
#define FR_PREV_MAX     0x6000u                 /* next2's answer (the out-buffers) */
#define FR_SC(m, in, out) ((((uint32_t)(m) & 0x1f) << 24) | (((uint32_t)(in) & 0xff) << 16) | (((uint32_t)(out) & 0xff) << 8))
#define FR_UNSUPPORTED  20                      /* AEE_EUNSUPPORTED */

struct fr_msg { int32_t pid, tid; uint64_t ctx; uint32_t handle, sc; uint64_t addr, size; } __attribute__((packed));
struct fr_rarg { uint64_t pv, len; };
struct fr_ibuf { uint32_t num, pgidx; };
struct fr_page { uint64_t addr, size; };

static struct {
	volatile bool busy, done;
	volatile int32_t retval;
	uint32_t off[16], len[16];
} fr_slot[FR_SLOTS];
static uint8_t *fr_va;
static uint64_t fr_pa;
static volatile int fr_state;                   /* 0 idle, 1 running, 2 listening, < 0 stopped at that step */
static volatile bool fr_started;
static unsigned fr_auto, fr_unmatched, fr_calls;
static IOLock *fr_log_lock;
static char fr_log[32768];
static uint32_t fr_log_n;

static void
fr_logf(const char *fmt, ...) __printflike(1, 2);
static void
fr_logf(const char *fmt, ...)
{
	char line[256];
	va_list ap;
	va_start(ap, fmt);
	int n = vsnprintf(line, sizeof(line) - 1, fmt, ap);
	va_end(ap);
	if (n < 0) {
		return;
	}
	n = MIN(n, (int)sizeof(line) - 2);
	line[n++] = '\n';
	line[n] = 0;
	IOLog("TopazModem: frpc: %s", line);
	IOLock *l = a_lock_get(&fr_log_lock);
	IOLockLock(l);
	if (fr_log_n + n >= sizeof(fr_log)) {
		uint32_t drop = MAX((uint32_t)sizeof(fr_log) / 2, fr_log_n + n - (uint32_t)sizeof(fr_log) + 1);
		drop = MIN(drop, fr_log_n);
		memmove(fr_log, fr_log + drop, fr_log_n - drop);
		fr_log_n -= drop;
	}
	memcpy(fr_log + fr_log_n, line, n);
	fr_log_n += n;
	IOLockUnlock(l);
}

static void
fr_dump(const char *what, const uint8_t *p, uint32_t n)
{
	fr_logf("%s: %u bytes", what, n);
	for (uint32_t o = 0; o < n && o < 512; o += 32) {
		char hex[100], asc[40];
		uint32_t k, h = 0;
		for (k = 0; k < 32 && o + k < n; k++) {
			h += snprintf(hex + h, sizeof(hex) - h, "%02x", p[o + k]);
			asc[k] = p[o + k] >= 0x20 && p[o + k] < 0x7f ? (char)p[o + k] : '.';
		}
		asc[k] = 0;
		fr_logf("  %04x %-64s %s", o, hex, asc);
	}
}

/* responses (the ADSP thread) */
static void
a_frpc_rx(const uint8_t *p, uint32_t len)
{
	uint64_t ctx = 0;
	int32_t rv = 0;
	if (len >= 12) {
		memcpy(&ctx, p, 8);
		memcpy(&rv, p + 8, 4);
	}
	unsigned s = (unsigned)((ctx >> 4) & 0xff);
	if (len < 12 || s == 0 || s > FR_SLOTS || !fr_slot[s - 1].busy) {
		if (fr_unmatched++ < 16) {
			fr_dump("unmatched message", p, len);
		}
		return;
	}
	fr_slot[s - 1].retval = rv;
	mb();
	fr_slot[s - 1].done = true;
}

/* one invoke into slot s (1..FR_SLOTS); in[i] / inlen[i] for the in-arguments, outlen[] for the out */
static bool
fr_post(unsigned s, bool kernel, uint32_t handle, uint32_t sc, const void *const *in, const uint32_t *inlen,
    const uint32_t *outlen)
{
	uint32_t nin = (sc >> 16) & 0xff, nout = (sc >> 8) & 0xff, n = nin + nout;
	uint8_t *b = fr_va + FR_SLOT_OFF(s);
	uint64_t pa = fr_pa + FR_SLOT_OFF(s);
	uint32_t meta = n * (uint32_t)(sizeof(struct fr_rarg) + sizeof(struct fr_ibuf) + sizeof(struct fr_page)) + 16 * 8 + 64 * 4;
	struct fr_rarg *ra = (struct fr_rarg *)b;
	struct fr_ibuf *il = (struct fr_ibuf *)(b + sizeof(struct fr_rarg) * n);
	struct fr_page *pg = (struct fr_page *)(b + (sizeof(struct fr_rarg) + sizeof(struct fr_ibuf)) * n);
	uint32_t at = meta;

	if (n > 16 || s == 0 || s > FR_SLOTS || fr_slot[s - 1].busy) {
		return false;
	}
	bzero(b, meta);
	for (uint32_t i = 0; i < n; i++) {
		uint32_t len = i < nin ? inlen[i] : outlen[i - nin];
		ra[i].len = len;
		il[i].num = len ? 1 : 0;
		il[i].pgidx = i;
		fr_slot[s - 1].len[i] = len;
		fr_slot[s - 1].off[i] = 0;
		if (len == 0) {
			continue;
		}
		at = (at + 127) & ~127u;
		if (at + len > FR_SLOT_LEN(s)) {
			fr_logf("invoke %x: %u bytes do not fit", sc, at + len);
			return false;
		}
		ra[i].pv = pa + at;                     /* only its offset in the page counts */
		pg[i].addr = ((pa + at) & ~0xfffull) | FR_SID_BITS;
		pg[i].size = ((((pa + at + len - 1) >> 12) - ((pa + at) >> 12)) + 1) << 12;
		if (i < nin) {
			memcpy(b + at, in[i], len);
		} else {
			bzero(b + at, len);
		}
		fr_slot[s - 1].off[i] = at;
		at += len;
	}
	struct fr_msg m;
	m.pid = kernel ? 0 : FR_TGID;
	m.tid = FR_TGID + (int32_t)s;
	m.ctx = ((uint64_t)s << 4) | FR_PD_SENSORS;
	m.handle = handle;
	m.sc = sc;
	m.addr = pa | FR_SID_BITS;
	m.size = (at + 0xfff) & ~0xfffu;
	fr_slot[s - 1].done = false;
	fr_slot[s - 1].retval = 0;
	fr_slot[s - 1].busy = true;
	mb();
	fr_calls++;
	if (s != 2 || fr_calls <= 4) {             /* 0266: what goes out (next2's only at first) */
		fr_logf("-> pid %d tid %d ctx %llx handle %u sc %08x addr %llx size %llx", m.pid, m.tid,
		    (unsigned long long)m.ctx, m.handle, m.sc, (unsigned long long)m.addr, (unsigned long long)m.size);
		fr_dump("   buffer", b, MIN(at, 0x300u));
	}
	a_send(&a_ch[2], &m, sizeof(m));
	return true;
}

static void
fr_release(unsigned s)
{
	fr_slot[s - 1].busy = false;
}

static const uint8_t *
fr_out(unsigned s, uint32_t i)
{
	return fr_va + FR_SLOT_OFF(s) + fr_slot[s - 1].off[i];
}

/* 0266: what the channel and the SMMU say when a call gets no answer */
static void
fr_diag(const char *when)
{
	static volatile uint8_t *smmu;
	struct a_chan *c = &a_ch[2];
	unsigned used = 0;
	for (unsigned i = 0; i < c->ri_count; i++) {
		used += c->ri[i].used;
	}
	fr_logf("%s: channel %s, the ADSP's receive buffers %u (%u in use), queue %u, drops %u; slave-kernel %08x", when,
	    a_up(c) ? "up" : "down", c->ri_count, used, c->qt - c->qh, c->drops, a_sk ? *a_sk : 0);
	if (smmu == NULL) {
		smmu = (volatile uint8_t *)ml_io_map(SMMU_PA, 0x1000);
	}
	if (smmu != NULL) {
		fr_logf("%s: SMMU sGFSR %08x sGFSYNR0 %08x sGFSYNR1 %08x sGFSYNR2 %08x", when, r32(smmu, 0x48), r32(smmu, 0x50),
		    r32(smmu, 0x54), r32(smmu, 0x58));
	}
}

/*
 * adsp_listener next2 on its static handle 3 (libadsprpc: remote_handle_invoke(3, 0x04020200, ...)):
 * in {prevCtx, prevResult, len(prevbufs), len(bufs)} + prevbufs, out {ctx, handle, sc, bufsLenReq} + bufs
 */
#define FR_LISTENER     3
static uint32_t fr_prev_ctx;
static int32_t fr_prev_result;
static unsigned fr_reverse, fr_next2_posts;

static uint8_t fr_rbuf[FR_BUFS], fr_pbuf[FR_PREV_MAX];
static uint32_t fr_plen;

static bool
fr_next2(void)
{
	uint32_t prim_in[4] = { fr_prev_ctx, (uint32_t)fr_prev_result, fr_plen, FR_BUFS };
	const void *in[2] = { prim_in, fr_pbuf };
	uint32_t inlen[2] = { sizeof(prim_in), fr_plen }, outlen[2] = { 16, FR_BUFS };
	fr_next2_posts++;
	return fr_post(2, false, FR_LISTENER, FR_SC(4, 2, 2), in, inlen, outlen);
}

/*
 * 0267: the files the sensors PD reads through reverse calls (apps_std), from a RAM store filled
 * from user space (kern.sweet_frpc_file = "path\0bytes", full paths such as
 * /vendor/etc/sensors/sns_reg_config or /mnt/vendor/persist/sensors/registry/registry/<group>).
 * Directories are what the stored paths imply. The PD's writes (registry updates) stay in RAM:
 * no partition is ever touched.
 */
#define FS_MAX          1536
#define FS_PATH         160
#define FS_BYTES_MAX    (16u << 20)
#define FS_FILES        64
#define FS_DIRS         16
#define FR_H_STD        0x5d0                   /* our handle for apps_std */
#define FR_EFAIL        1                       /* AEE_EFAILED */

struct fs_ent { char path[FS_PATH]; uint8_t *data; uint32_t size, cap; bool written; };
static struct fs_ent *fs_tab;
static unsigned fs_n;
static uint32_t fs_bytes;
static IOLock *fs_lock;
static struct { int ent; uint32_t pos; bool used, wr; } fs_fh[FS_FILES];
static struct { char path[FS_PATH]; unsigned cursor; bool used; } fs_dh[FS_DIRS];
static unsigned fs_reads, fs_writes, fs_misses;

static int
fs_find(const char *path)
{
	for (unsigned i = 0; i < fs_n; i++) {
		if (strcmp(fs_tab[i].path, path) == 0) {
			return (int)i;
		}
	}
	return -1;
}

/* fs_lock held: a file's bytes replaced (or a new file) */
static int
fs_put_locked(const char *path, const uint8_t *data, uint32_t len, bool written)
{
	if (fs_tab == NULL) {
		fs_tab = (struct fs_ent *)IOMalloc(FS_MAX * sizeof(struct fs_ent));
		if (fs_tab == NULL) {
			return ENOMEM;
		}
		bzero(fs_tab, FS_MAX * sizeof(struct fs_ent));
	}
	if (strlen(path) >= FS_PATH || path[0] != '/') {
		return EINVAL;
	}
	int i = fs_find(path);
	if (i < 0) {
		if (fs_n >= FS_MAX) {
			return ENOSPC;
		}
		i = (int)fs_n++;
		strlcpy(fs_tab[i].path, path, FS_PATH);
	}
	struct fs_ent *e = &fs_tab[i];
	uint32_t cap = MAX(len, 64u);
	if (e->cap < len || e->data == NULL) {
		if (fs_bytes + cap > FS_BYTES_MAX) {
			return EFBIG;
		}
		uint8_t *d = (uint8_t *)IOMalloc(cap);
		if (d == NULL) {
			return ENOMEM;
		}
		if (e->data != NULL) {
			IOFree(e->data, e->cap);
			fs_bytes -= e->cap;
		}
		e->data = d;
		e->cap = cap;
		fs_bytes += cap;
	}
	if (len != 0) {
		memcpy(e->data, data, len);
	}
	e->size = len;
	e->written = written;
	return 0;
}

/* fs_lock held: grow a file for a write at pos */
static bool
fs_grow_locked(struct fs_ent *e, uint32_t need)
{
	if (need <= e->cap) {
		return true;
	}
	uint32_t cap = MAX(need, e->cap * 2);
	if (fs_bytes + cap - e->cap > FS_BYTES_MAX) {
		return false;
	}
	uint8_t *d = (uint8_t *)IOMalloc(cap);
	if (d == NULL) {
		return false;
	}
	bzero(d, cap);
	if (e->data != NULL) {
		memcpy(d, e->data, e->size);
		IOFree(e->data, e->cap);
	}
	fs_bytes += cap - e->cap;
	e->data = d;
	e->cap = cap;
	return true;
}

extern "C" int
topaz_frpc_file_put(const uint8_t *buf, uint32_t len)
{
	uint32_t nl = (uint32_t)strnlen((const char *)buf, MIN(len, (uint32_t)FS_PATH));
	if (nl == 0 || nl >= len || nl >= FS_PATH) {
		return EINVAL;
	}
	IOLock *l = a_lock_get(&fs_lock);
	IOLockLock(l);
	int err = fs_put_locked((const char *)buf, buf + nl + 1, len - nl - 1, false);
	IOLockUnlock(l);
	return err;
}

/* kern.sweet_frpc_files: "size path" lines (w: written by the PD) */
extern "C" int
topaz_frpc_files(char *out, size_t max)
{
	size_t n = 0;
	IOLock *l = a_lock_get(&fs_lock);
	IOLockLock(l);
	n += snprintf(out + n, max - n, "%u files, %u bytes; PD reads %u writes %u misses %u\n", fs_n, fs_bytes, fs_reads,
	    fs_writes, fs_misses);
	for (unsigned i = 0; i < fs_n && n + FS_PATH + 24 < max; i++) {
		n += snprintf(out + n, max - n, "%u%s %s\n", fs_tab[i].size, fs_tab[i].written ? "w" : "", fs_tab[i].path);
	}
	IOLockUnlock(l);
	return (int)n;
}

/* a path as the PD gives it: no trailing slash, no double slashes */
static void
fs_norm(char *dst, const uint8_t *src, uint32_t len)
{
	uint32_t n = 0;
	for (uint32_t i = 0; i < len && src[i] != 0 && n + 1 < FS_PATH; i++) {
		if (src[i] == '/' && n > 0 && dst[n - 1] == '/') {
			continue;
		}
		dst[n++] = (char)src[i];
	}
	while (n > 1 && dst[n - 1] == '/') {
		n--;
	}
	dst[n] = 0;
}

/* is there anything under dir/ ? */
static bool
fs_is_dir_locked(const char *dir)
{
	size_t dl = strlen(dir);
	for (unsigned i = 0; i < fs_n; i++) {
		if (strncmp(fs_tab[i].path, dir, dl) == 0 && fs_tab[i].path[dl] == '/') {
			return true;
		}
	}
	return dl == 1 && fs_n > 0;                 /* "/" */
}

/* the child of dir that entry i is under (a file or a subdirectory), or NULL */
static const char *
fs_child(const char *dir, unsigned i, size_t *clen)
{
	size_t dl = strcmp(dir, "/") == 0 ? 0 : strlen(dir);
	const char *p = fs_tab[i].path;
	if (strncmp(p, dir, dl) != 0 || p[dl] != '/') {
		return NULL;
	}
	p += dl + 1;
	const char *slash = strchr(p, '/');
	*clen = slash != NULL ? (size_t)(slash - p) : strlen(p);
	return p;
}

static uint32_t u32_at(const uint8_t *p, unsigned i) { uint32_t v; memcpy(&v, p + 4 * i, 4); return v; }
static void put32(uint8_t *p, unsigned i, uint32_t v) { memcpy(p + 4 * i, &v, 4); }

/* a reverse call, its arguments unpacked */
struct fr_rev {
	uint32_t handle, sc, method, nin, nout;
	const uint8_t *in[16];
	uint32_t inl[16];
	uint8_t *out[16];
	uint32_t outl[16];
};

static unsigned fr_std_logged;

#define STDLOG(...) do { if (fr_std_logged++ < 400) fr_logf(__VA_ARGS__); } while (0)

/* apps_remotectl (handle 0): open / close */
static int
fr_remotectl(struct fr_rev *r)
{
	char name[64] = "";
	if (r->method == 0 && r->nin >= 2 && r->nout >= 1 && r->outl[0] >= 8) {
		fs_norm(name, r->in[1], MIN(r->inl[1], (uint32_t)sizeof(name)));
		bool std = strcmp(name, "apps_std") == 0;
		put32(r->out[0], 0, std ? FR_H_STD : 0);
		put32(r->out[0], 1, std ? 0 : FR_UNSUPPORTED);
		STDLOG("remotectl open \"%s\" -> %s", name, std ? "apps_std" : "not here");
		return 0;
	}
	if (r->method == 1) {
		STDLOG("remotectl close 0x%x", r->nin >= 1 && r->inl[0] >= 4 ? u32_at(r->in[0], 0) : 0);
		return 0;
	}
	fr_logf("remotectl method %u: not here", r->method);
	return FR_UNSUPPORTED;
}

static int
fs_open_locked(const char *path, const char *mode, uint32_t *fh)
{
	bool wr = strchr(mode, 'w') != NULL || strchr(mode, 'a') != NULL || strchr(mode, '+') != NULL;
	int e = fs_find(path);
	if (e < 0 && !wr) {
		fs_misses++;
		return FR_EFAIL;
	}
	if (e < 0 || strchr(mode, 'w') != NULL) {
		if (fs_put_locked(path, NULL, 0, true) != 0) {
			return FR_EFAIL;
		}
		e = fs_find(path);
	}
	for (unsigned k = 0; k < FS_FILES; k++) {
		if (!fs_fh[k].used) {
			fs_fh[k].used = true;
			fs_fh[k].ent = e;
			fs_fh[k].wr = wr;
			fs_fh[k].pos = strchr(mode, 'a') != NULL ? fs_tab[e].size : 0;
			*fh = k + 1;
			return 0;
		}
	}
	return FR_EFAIL;
}

static struct fs_ent *
fs_fh_ent(uint32_t fh, unsigned *k)
{
	if (fh == 0 || fh > FS_FILES || !fs_fh[fh - 1].used) {
		return NULL;
	}
	*k = fh - 1;
	return &fs_tab[fs_fh[fh - 1].ent];
}

/* apps_std (Lunaris libadsprpc's skel: methods 0-30, 31+ through method 31 with the id first) */
static int
fr_std_locked(struct fr_rev *r)
{
	char path[FS_PATH], mode[8], path2[FS_PATH];
	const uint8_t *pin = r->nin >= 1 ? r->in[0] : NULL;
	uint32_t pl = r->nin >= 1 ? r->inl[0] : 0, m = r->method;
	unsigned k;
	struct fs_ent *e;

	if (m == 31 && pl >= 4) {                   /* extended: the method id is the first word */
		m = u32_at(pin, 0);
		pin += 4;
		pl -= 4;
	}
#define NEED(ni, no) do { if (r->nin < (ni) || r->nout < (no)) { fr_logf("apps_std %u: in %u out %u", m, r->nin, r->nout); return FR_EFAIL; } } while (0)
	switch (m) {
	case 0:                                     /* fopen(name, mode, rout FILE) */
	case 19: {                                  /* fopen_with_env(env, delim, name, mode, rout FILE) */
		unsigned ni = m == 0 ? 1 : 3;
		NEED(ni + 2, 1);
		uint32_t fh = 0;
		fs_norm(path, r->in[ni], r->inl[ni]);
		fs_norm(mode, r->in[ni + 1], MIN(r->inl[ni + 1], (uint32_t)sizeof(mode)));
		int rv = fs_open_locked(path, mode, &fh);
		put32(r->out[0], 0, fh);
		STDLOG("std fopen%s \"%s\" \"%s\" -> %s", m == 19 ? "_with_env" : "", path, mode, rv == 0 ? "ok" : "missing");
		return rv;
	}
	case 2:                                     /* fflush */
	case 23:                                    /* fsync */
		return 0;
	case 3:                                     /* fclose(FILE) */
		if (pl >= 4 && (e = fs_fh_ent(u32_at(pin, 0), &k)) != NULL) {
			if (fs_fh[k].wr) {
				STDLOG("std fclose \"%s\": %u bytes written (RAM)", e->path, e->size);
			}
			fs_fh[k].used = false;
		}
		return 0;
	case 4: {                                   /* fread(FILE, rout buf, rout bytesRead, rout eof) */
		NEED(1, 2);
		if (pl < 8 || (e = fs_fh_ent(u32_at(pin, 0), &k)) == NULL || r->outl[0] < 8) {
			return FR_EFAIL;
		}
		uint32_t want = MIN(u32_at(pin, 1), r->outl[1]), pos = fs_fh[k].pos;
		uint32_t n = pos < e->size ? MIN(want, e->size - pos) : 0;
		memcpy(r->out[1], e->data + pos, n);
		fs_fh[k].pos += n;
		put32(r->out[0], 0, n);
		put32(r->out[0], 1, fs_fh[k].pos >= e->size);
		fs_reads++;
		return 0;
	}
	case 5: {                                   /* fwrite(FILE, buf, rout written, rout error) */
		NEED(2, 1);
		if (pl < 4 || (e = fs_fh_ent(u32_at(pin, 0), &k)) == NULL || !fs_fh[k].wr || r->outl[0] < 8) {
			return FR_EFAIL;
		}
		uint32_t n = r->inl[1], pos = fs_fh[k].pos;
		if (!fs_grow_locked(e, pos + n)) {
			return FR_EFAIL;
		}
		memcpy(e->data + pos, r->in[1], n);
		fs_fh[k].pos += n;
		e->size = MAX(e->size, pos + n);
		e->written = true;
		put32(r->out[0], 0, n);
		put32(r->out[0], 1, 0);
		fs_writes++;
		return 0;
	}
	case 8:                                     /* ftell(FILE, rout pos) */
		NEED(1, 1);
		if (pl < 4 || (e = fs_fh_ent(u32_at(pin, 0), &k)) == NULL) {
			return FR_EFAIL;
		}
		put32(r->out[0], 0, fs_fh[k].pos);
		return 0;
	case 9: {                                   /* fseek(FILE, offset, whence) */
		if (pl < 12 || (e = fs_fh_ent(u32_at(pin, 0), &k)) == NULL) {
			return FR_EFAIL;
		}
		int32_t off = (int32_t)u32_at(pin, 1);
		uint32_t wh = u32_at(pin, 2);
		int64_t base = wh == 0 ? 0 : wh == 1 ? (int64_t)fs_fh[k].pos : (int64_t)e->size;
		if (base + off < 0) {
			return FR_EFAIL;
		}
		fs_fh[k].pos = (uint32_t)(base + off);
		return 0;
	}
	case 10: {                                  /* flen(FILE, rout uint64 len) */
		NEED(1, 1);
		if (pl < 4 || (e = fs_fh_ent(u32_at(pin, 0), &k)) == NULL || r->outl[0] < 8) {
			return FR_EFAIL;
		}
		uint64_t len = e->size;
		memcpy(r->out[0], &len, 8);
		return 0;
	}
	case 11:                                    /* rewind(FILE) */
		if (pl >= 4 && (e = fs_fh_ent(u32_at(pin, 0), &k)) != NULL) {
			fs_fh[k].pos = 0;
		}
		return 0;
	case 12:                                    /* feof(FILE, rout eof) */
		NEED(1, 1);
		if (pl < 4 || (e = fs_fh_ent(u32_at(pin, 0), &k)) == NULL) {
			return FR_EFAIL;
		}
		put32(r->out[0], 0, fs_fh[k].pos >= e->size);
		return 0;
	case 13:                                    /* ferror */
		NEED(1, 1);
		put32(r->out[0], 0, 0);
		return 0;
	case 14:                                    /* clearerr */
		return 0;
	case 20: {                                  /* fgets(FILE, rout buf, rout eof) */
		NEED(1, 2);
		if (pl < 8 || (e = fs_fh_ent(u32_at(pin, 0), &k)) == NULL || r->outl[1] == 0) {
			return FR_EFAIL;
		}
		uint32_t cap = r->outl[1], pos = fs_fh[k].pos, n = 0;
		while (n + 1 < cap && pos < e->size) {
			uint8_t c = e->data[pos++];
			r->out[1][n++] = c;
			if (c == '\n') {
				break;
			}
		}
		r->out[1][n] = 0;
		fs_fh[k].pos = pos;
		put32(r->out[0], 0, pos >= e->size);
		fs_reads++;
		return 0;
	}
	case 22: {                                  /* fileExists(path, rout exists) */
		NEED(2, 1);
		fs_norm(path, r->in[1], r->inl[1]);
		bool ex = fs_find(path) >= 0 || fs_is_dir_locked(path);
		put32(r->out[0], 0, ex);
		STDLOG("std fileExists \"%s\" -> %u", path, ex);
		return 0;
	}
	case 24:                                    /* fremove(path) */
		if (r->nin >= 2) {
			fs_norm(path, r->in[1], r->inl[1]);
			int i = fs_find(path);
			STDLOG("std fremove \"%s\" (%s, RAM only)", path, i >= 0 ? "there" : "missing");
			if (i >= 0) {
				fs_tab[i].size = 0;
				fs_tab[i].path[0] = '#';        /* hidden */
			}
		}
		return 0;
	case 26: {                                  /* opendir(path, rout DIR (u64)) */
		NEED(2, 1);
		fs_norm(path, r->in[1], r->inl[1]);
		if (!fs_is_dir_locked(path) || r->outl[0] < 8) {
			STDLOG("std opendir \"%s\" -> missing", path);
			fs_misses++;
			return FR_EFAIL;
		}
		for (k = 0; k < FS_DIRS; k++) {
			if (!fs_dh[k].used) {
				fs_dh[k].used = true;
				fs_dh[k].cursor = 0;
				strlcpy(fs_dh[k].path, path, FS_PATH);
				uint64_t h = k + 1;
				memcpy(r->out[0], &h, 8);
				STDLOG("std opendir \"%s\" -> dir %u", path, k + 1);
				return 0;
			}
		}
		return FR_EFAIL;
	}
	case 27: {                                  /* closedir(DIR) */
		uint64_t h = 0;
		if (pl >= 8) {
			memcpy(&h, pin, 8);
		}
		if (h >= 1 && h <= FS_DIRS) {
			fs_dh[h - 1].used = false;
		}
		return 0;
	}
	case 28: {                                  /* readdir(DIR, rout {int ino; char name[255]}, rout eof) */
		NEED(1, 1);
		uint64_t h = 0;
		if (pl >= 8) {
			memcpy(&h, pin, 8);
		}
		if (h < 1 || h > FS_DIRS || !fs_dh[h - 1].used || r->outl[0] < 264) {
			return FR_EFAIL;
		}
		bzero(r->out[0], 264);
		const char *dir = fs_dh[h - 1].path;
		for (unsigned i = fs_dh[h - 1].cursor; i < fs_n; i++) {
			size_t cl;
			const char *c = fs_child(dir, i, &cl);
			if (c == NULL || cl == 0 || cl > 254) {
				continue;
			}
			bool seen = false;                  /* a subdirectory: only its first entry counts */
			for (unsigned j = 0; j < i && !seen; j++) {
				size_t cl2;
				const char *c2 = fs_child(dir, j, &cl2);
				seen = c2 != NULL && cl2 == cl && strncmp(c, c2, cl) == 0;
			}
			if (seen) {
				continue;
			}
			fs_dh[h - 1].cursor = i + 1;
			put32(r->out[0], 0, i + 1);
			memcpy(r->out[0] + 4, c, cl);
			return 0;
		}
		fs_dh[h - 1].cursor = fs_n;
		put32(r->out[0], 65, 1);                    /* eof at byte 260 */
		return 0;
	}
	case 29:                                    /* mkdir */
	case 32:                                    /* ftrunc */
		return 0;
	case 31: {                                  /* stat(path, rout apps_std_STAT, 96 bytes) */
		NEED(2, 1);
		fs_norm(path, r->in[1], r->inl[1]);
		int i = fs_find(path);
		bool dir = i < 0 && fs_is_dir_locked(path);
		STDLOG("std stat \"%s\" -> %s", path, i >= 0 ? "file" : dir ? "dir" : "missing");
		if ((i < 0 && !dir) || r->outl[0] < 96) {
			fs_misses++;
			return FR_EFAIL;
		}
		uint8_t *s = r->out[0];
		bzero(s, 96);
		uint64_t ino = (uint64_t)(i + 1), size = i >= 0 ? fs_tab[i].size : 0;
		uint32_t mod = dir ? 040755 : 0100644, nlink = 1;
		memcpy(s + 16, &ino, 8);
		memcpy(s + 24, &mod, 4);
		memcpy(s + 28, &nlink, 4);
		memcpy(s + 40, &size, 8);
		return 0;
	}
	case 33:                                    /* frename(old, new): primIn {lenOld, lenNew} */
		if (r->nin >= 3) {
			fs_norm(path, r->in[1], r->inl[1]);
			fs_norm(path2, r->in[2], r->inl[2]);
			int i = fs_find(path), j = fs_find(path2);
			STDLOG("std frename \"%s\" -> \"%s\" (RAM only)", path, path2);
			if (i < 0) {
				return FR_EFAIL;
			}
			if (j >= 0 && j != i) {
				fs_tab[j].path[0] = '#';
			}
			strlcpy(fs_tab[i].path, path2, FS_PATH);
			return 0;
		}
		return FR_EFAIL;
	default:
		break;
	}
#undef NEED
	fr_logf("apps_std method %u (in %u out %u): not here", m, r->nin, r->nout);
	return FR_UNSUPPORTED;
}

/* unpack (listener_android: per in-buffer u32 length, then the data 8-aligned; per out-buffer u32 length) */
static bool
fr_unpack(struct fr_rev *r, const uint8_t *b, uint32_t n)
{
	uint32_t off = 0;
	if (r->nin > 16 || r->nout > 16) {
		return false;
	}
	for (uint32_t i = 0; i < r->nin; i++) {
		if (off + 4 > n) {
			return false;
		}
		uint32_t len = u32_at(b + off, 0);
		off += 4;
		if (len != 0) {
			off = (off + 7) & ~7u;
		}
		if (off + len > n) {
			return false;
		}
		r->in[i] = b + off;
		r->inl[i] = len;
		off += len;
	}
	for (uint32_t i = 0; i < r->nout; i++) {
		if (off + 4 > n) {
			return false;
		}
		r->outl[i] = u32_at(b + off, 0);
		off += 4;
	}
	return off == n;
}

/* lay the out-buffers out the same way (u32 length, data 8-aligned); returns the packed length, ~0 if it does not fit */
static uint32_t
fr_pack_outs(struct fr_rev *r, uint8_t *p, uint32_t cap)
{
	uint32_t off = 0;
	for (uint32_t i = 0; i < r->nout; i++) {
		uint32_t len = r->outl[i];
		if (off + 4 > cap) {
			return ~0u;
		}
		put32(p + off, 0, len);
		off += 4;
		if (len != 0) {
			off = (off + 7) & ~7u;
		}
		if (off + len > cap) {
			return ~0u;
		}
		r->out[i] = p + off;
		bzero(p + off, len);
		off += len;
	}
	return off;
}

/* a finished next2: the reverse call served (apps_remotectl, apps_std), its answer goes with the next next2 */
static void
fr_service(void)
{
	if (!fr_slot[1].busy || !fr_slot[1].done) {
		return;
	}
	mb();
	int32_t rv = fr_slot[1].retval;
	uint32_t po[4];
	memcpy(po, fr_out(2, 2), 16);
	uint32_t blen = MIN(po[3], FR_BUFS);
	if (rv == 0) {
		memcpy(fr_rbuf, fr_out(2, 3), blen);
	}
	fr_release(2);
	if (rv == 0x2e) {                           /* interrupted: the same call again (listener_android) */
		(void)fr_next2();
		return;
	}
	if (rv != 0) {
		fr_logf("next2: retval %d (0x%x)", rv, (unsigned)rv);
		if (fr_next2_posts > 8 && fr_reverse == 0) {
			fr_logf("next2 keeps failing: the listener stops");
			fr_state = -7;
			return;
		}
		fr_prev_ctx = 0;
		fr_prev_result = 0;
		fr_plen = 0;
		(void)fr_next2();
		return;
	}
	struct fr_rev r;
	bzero(&r, sizeof(r));
	r.handle = po[1];
	r.sc = po[2];
	r.method = (po[2] >> 24) & 0x1f;
	r.nin = (po[2] >> 16) & 0xff;
	r.nout = (po[2] >> 8) & 0xff;
	int result = FR_UNSUPPORTED;
	fr_plen = 0;
	if (po[3] > FR_BUFS || !fr_unpack(&r, fr_rbuf, po[3])) {
		fr_logf("reverse call %u: handle 0x%x sc 0x%08x: %u bytes of arguments not understood", fr_reverse, po[1], po[2],
		    po[3]);
		fr_dump("  in-args", fr_rbuf, blen);
	} else if ((fr_plen = fr_pack_outs(&r, fr_pbuf, sizeof(fr_pbuf))) == ~0u) {
		fr_logf("reverse call %u: handle 0x%x sc 0x%08x: out-buffers too big", fr_reverse, po[1], po[2]);
		fr_plen = 0;
	} else if (r.handle == 0) {
		result = fr_remotectl(&r);
	} else if (r.handle == FR_H_STD) {
		IOLock *l = a_lock_get(&fs_lock);
		IOLockLock(l);
		result = fs_tab != NULL ? fr_std_locked(&r) : FR_EFAIL;
		IOLockUnlock(l);
		if (fs_tab == NULL) {
			fr_logf("apps_std: no files loaded (kern.sweet_frpc_file)");
		}
	} else {
		fr_logf("reverse call %u: handle 0x%x sc 0x%08x (method %u, in %u, out %u): not here", fr_reverse, po[1], po[2],
		    r.method, r.nin, r.nout);
		fr_dump("  in-args", fr_rbuf, blen);
	}
	fr_reverse++;
	fr_prev_ctx = po[0];
	fr_prev_result = result;
	(void)fr_next2();
}

/* waits for slot s, serving reverse calls meanwhile (a call may need them: a skel loaded through apps_std) */
static bool
fr_wait(unsigned s, unsigned ms)
{
	for (unsigned i = 0; i < ms && !fr_slot[s - 1].done; i++) {
		fr_service();
		IOSleep(1);
	}
	mb();
	return fr_slot[s - 1].done;
}

/* a call that answers within ms: its retval, or -1 on a timeout (the slot then stays taken) */
static int
fr_call(unsigned s, bool kernel, uint32_t handle, uint32_t sc, const void *const *in, const uint32_t *inlen,
    const uint32_t *outlen, unsigned ms, const char *what)
{
	if (!fr_post(s, kernel, handle, sc, in, inlen, outlen)) {
		fr_logf("%s: not sent", what);
		return -2;
	}
	if (!fr_wait(s, ms)) {
		fr_logf("%s: no answer in %u ms", what, ms);
		fr_diag(what);
		return -1;
	}
	fr_logf("%s: retval %d (0x%x)", what, fr_slot[s - 1].retval, (unsigned)fr_slot[s - 1].retval);
	return fr_slot[s - 1].retval;
}

/* remotectl_open(name): static handle 0, method 0, in {len(name), len(dlerror)} + name, out {handle, nErr} + dlerror */
static int
fr_open(const char *name, uint32_t *h, unsigned ms)
{
	uint32_t prim_in[2] = { (uint32_t)strlen(name) + 1, 128 };
	const void *in[2] = { prim_in, name };
	uint32_t inlen[2] = { sizeof(prim_in), prim_in[0] }, outlen[2] = { 8, 128 };
	char what[64];
	snprintf(what, sizeof(what), "open \"%s\"", name);
	int r = fr_call(1, false, 0, FR_SC(0, 2, 2), in, inlen, outlen, ms, what);
	if (r == -1 || r == -2) {
		return r;
	}
	uint32_t prim_out[2];
	memcpy(prim_out, fr_out(1, 2), 8);
	char dlerr[129];
	memcpy(dlerr, fr_out(1, 3), 128);
	dlerr[128] = 0;
	fr_release(1);
	fr_logf("open \"%s\": handle 0x%x nErr 0x%x dlerror \"%s\"", name, prim_out[0], prim_out[1], dlerr);
	*h = prim_out[0];
	return r != 0 ? r : (int)prim_out[1];
}

static void
fr_thread(void *arg __unused, wait_result_t wr __unused)
{
	uint32_t tgid = FR_TGID, dh = 0;
	int r;

	fr_state = 1;
	fr_logf("start: channel %s, pool used %llu of %llu", a_up(&a_ch[2]) ? "up" : "down", (unsigned long long)pool_used,
	    (unsigned long long)FR_MEM_OFF);
	if (pool == NULL || pool_used > FR_MEM_OFF) {
		fr_logf("no room below the audio buffers");
		fr_state = -1;
		goto out;
	}
	fr_va = pool + FR_MEM_OFF;
	fr_pa = MODEM_POOL_BASE + FR_MEM_OFF;
	bzero(fr_va, 0x10000);
	if (!smmu_identity(FR_SID, 0, "fastrpc (ADSP compute cb 0x1c3)")) {
		fr_logf("no SMMU stream 0x%x", FR_SID);
		fr_state = -2;
		goto out;
	}
	fr_logf("buffers at %llx (+%llx stream bits), %u slots", (unsigned long long)fr_pa, (unsigned long long)FR_SID_BITS,
	    FR_SLOTS);
	fr_diag("before the attach");

	/* INIT_ATTACH_SNS: handle 1, method 0, in: the tgid; a kernel message (pid 0), ctx pd 2 */
	{
		const void *in[1] = { &tgid };
		uint32_t inlen[1] = { 4 };
		r = fr_call(1, true, 1, FR_SC(0, 1, 0), in, inlen, NULL, 5000, "attach sensors PD");
		if (r != -1) {
			fr_release(1);
		}
		if (r != 0) {
			fr_state = -3;
			goto out;
		}
	}
	/*
	 * 0266: the listener as libadsprpc runs it: no open (0265's remotectl open of "adsp_listener" got no
	 * answer), static handle 3: init2 = remote_handle_invoke(3, 0x03000000, NULL), then next2 (0x04020200)
	 * kept posted.
	 */
	r = fr_call(1, false, FR_LISTENER, FR_SC(3, 0, 0), NULL, NULL, NULL, 5000, "adsp_listener init2 (handle 3)");
	if (r != -1) {
		fr_release(1);
	}
	if (r != 0) {
		fr_state = -5;
		goto out;
	}
	if (!fr_next2()) {
		fr_state = -6;
		goto out;
	}
	fr_state = 2;
	/* adsp_default_listener.c: remote_handle_open("adsp_default_listener"), then register = invoke(h, 0, NULL) */
	if (fr_open("adsp_default_listener", &dh, 20000) == 0 && dh != 0) {
		r = fr_call(1, false, dh, FR_SC(0, 0, 0), NULL, NULL, NULL, 20000, "adsp_default_listener register");
		if (r != -1) {
			fr_release(1);
		}
	}
	fr_logf("serving reverse calls (apps_std from the RAM file store)");
	for (unsigned t = 0; fr_state == 2; t++) {
		fr_service();
		IOSleep(2);
		if (t == 30000) {
			fr_diag("60 s");
		}
	}
out:
	fr_logf("stopped, state %d; %u calls, %u reverse, %u unmatched messages", fr_state, fr_calls, fr_reverse, fr_unmatched);
	thread_terminate(current_thread());
}

static void
fr_start(void)
{
	thread_t t;
	if (fr_started) {
		return;
	}
	fr_started = true;
	if (kernel_thread_start(fr_thread, NULL, &t) == KERN_SUCCESS) {
		thread_deallocate(t);
	}
}

/* the ADSP thread, every pass: boot-arg frpc=1 starts it 45 s into the ADSP's life */
static void
fr_poll(void)
{
	if (fr_auto == 0) {
		unsigned v = 0;
		fr_auto = PE_parse_boot_argn("frpc", &v, sizeof(v)) && v != 0 ? 1 : 2;
	}
	if (fr_auto == 1 && !fr_started && a_ms >= 45000 && a_up(&a_ch[2])) {
		fr_start();
	}
}

/* kern.sweet_frpc (bsd/dev/sweet_qmi_dev.c): write 1 starts it (EAGAIN: the channel is not up); reads the state */
extern "C" int
topaz_frpc_start(void)
{
	if (!a_up(&a_ch[2])) {
		return EAGAIN;
	}
	fr_start();
	return 0;
}

extern "C" int
topaz_frpc_state(void)
{
	return fr_state;
}

extern "C" int
topaz_frpc_log(char *buf, size_t len)
{
	IOLock *l = a_lock_get(&fr_log_lock);
	IOLockLock(l);
	size_t n = MIN((size_t)fr_log_n, len - 1);
	memcpy(buf, fr_log + fr_log_n - n, n);
	buf[n] = 0;
	IOLockUnlock(l);
	return (int)n;
}

/* ---------------- the ADSP's watch thread */

static void
adsp_thread(void *arg __unused, wait_result_t wr __unused)
{
	struct smem_part_hdr *part = smem_partition(0, ADSP_HOST);
	struct smp2p_hdr *out, *in = NULL;
	volatile uint32_t *sk = NULL;
	uint32_t sk_last = ~0u;
	bool ready = false, glink = false, tested = false;
	unsigned spf_asks = 0, next_ask = 0;

	out = (struct smp2p_hdr *)smem_alloc(part, ADSP_ITEM_SMP2P_OUT,
	    sizeof(struct smp2p_hdr) + SMP2P_MAX_ENTRY * sizeof(struct smp2p_entry));
	if (out != NULL && out->version == 0) {
		out->magic = SMP2P_MAGIC;
		out->local_pid = 0;
		out->remote_pid = ADSP_HOST;
		out->total = SMP2P_MAX_ENTRY;
		out->valid = 0;
		out->features[0] = 1;                   /* SSR_ACK */
		mb();
		out->version = 1;
		adsp_kick(ADSP_SMP2P_BIT);
	}
	LOG("adsp: SMP2P out item %u %p", ADSP_ITEM_SMP2P_OUT, out);
	for (a_ms = 0;; a_ms++) {
		if (in == NULL && (a_ms % 10) == 0) {
			in = (struct smp2p_hdr *)smem_get(part, ADSP_ITEM_SMP2P_IN, NULL);
			if (in != NULL && in->magic != SMP2P_MAGIC) {
				in = NULL;
			}
			if (in != NULL) {
				LOG("adsp: t=%u ms: SMP2P item %u up (v%u, %u entries)", a_ms, ADSP_ITEM_SMP2P_IN, in->version, in->valid);
			}
		}
		if (in != NULL && sk == NULL) {
			sk = smp2p_find(in, "slave-kernel");
			a_sk = sk;
		}
		if (sk != NULL && *sk != sk_last) {
			sk_last = *sk;
			LOG("adsp: t=%u ms: slave-kernel %08x%s%s%s", a_ms, sk_last, (sk_last & 1) ? " FATAL" : "",
			    (sk_last & 2) ? " READY" : "", (sk_last & 4) ? " HANDOVER" : "");
			if (sk_last & 1) {
				uint32_t sz = 0;
				const char *why = (const char *)smem_get(smem_partition(SMEM_GLOBAL_HOST, SMEM_GLOBAL_HOST), 423, &sz);
				char buf[128] = {};
				if (why != NULL) {
					memcpy(buf, why, MIN(sz, (uint32_t)sizeof(buf) - 1));
				}
				LOG("adsp: crash reason: %s", why != NULL ? buf : "(none)");
				break;
			}
			ready |= (sk_last & 2) != 0;
		}
		if (ready && !glink) {
			glink = a_glink_start(part);
			if (!glink) {
				break;
			}
		}
		if (glink) {
			if (a_rx == NULL && a_rx_msgs == 0) {
				uint32_t sz = 0;
				a_rx = (uint8_t *)smem_get(part, SMEM_GLINK_FIFO_RX, &sz);
				if (a_rx != NULL) {
					a_rx_len = sz;
					LOG("adsp: t=%u ms: ADSP RX FIFO, %u bytes", a_ms, sz);
				}
			}
			a_service();
			fr_poll();
			/* after the USB log is up (it starts ~40 s into the boot), so that all of it is seen */
			if (a_spf_ready && audio_on >= 2 && !tested && a_ms >= 45000) {
				tested = true;
				audio_test();
			}
			if (tested) {
				audio_request();
				gpr_request();
				mic_service();
				stream_service();
			}
			if (a_up(&a_ch[0]) && a_hello_rx && !a_hello_sent) {
				struct qrtr_ctrl c = { QRTR_TYPE_HELLO, 0, 0, 0, 0 };
				qrtr_send(QRTR_TYPE_HELLO, QRTR_PORT_CTRL, a_node, QRTR_PORT_CTRL, &c, sizeof(c));
				a_hello_sent = true;
				if (!a_announced) {
					struct qrtr_ctrl s = { QRTR_TYPE_NEW_SERVER, 0x40, 0x101, QRTR_NODE_APPS, PORT_PDM };
					qrtr_send(QRTR_TYPE_NEW_SERVER, QRTR_PORT_CTRL, a_node, QRTR_PORT_CTRL, &s, sizeof(s));
					a_announced = true;
					LOG("adsp: t=%u ms: HELLO answered, pd-mapper announced to node %u", a_ms, a_node);
				}
			}
			if (a_up(&a_ch[1]) && !a_spf_ready && spf_asks < 60 && a_ms >= next_ask) {
				a_gpr_send(APM_PORT, APM_CMD_GET_SPF_STATE, ++spf_asks, NULL, 0);
				next_ask = a_ms + 500;
			}
		}
		if (a_ms == 44000) {
			for (unsigned k = 0; k < A_NCH; k++) {
				uint32_t most = 0;
				for (unsigned i = 0; i < a_ch[k].ri_count; i++) {
					most = MAX(most, a_ch[k].ri[i].size);
				}
				LOG("adsp: \"%s\": the ADSP gave %u receive buffers, the biggest %u bytes", a_ch[k].name, a_ch[k].ri_count, most);
			}
		}
		if (a_ms == 5000 || a_ms == 30000 || a_ms == 44000) {
			LOG("adsp: t=%u ms: slave-kernel %08x, glink rx %u tx %u, IPCRTR %s, adsp_apps %s, qrtr node %u, %u services, "
			    "gpr %u packets, SPF %s after %u asks", a_ms, sk_last, a_rx_msgs, a_tx_msgs, a_up(&a_ch[0]) ? "up" : "down",
			    a_up(&a_ch[1]) ? "up" : "down", a_node, a_srv_count, a_gpr_rx_count, a_spf_ready ? "READY" : "not ready",
			    spf_asks);
		}
		IOSleep(1);
	}
	LOG("adsp: watch stopped at t=%u ms", a_ms);
	thread_terminate(current_thread());
}

static uint32_t
adsp_node(void)
{
	return a_node;
}

static uint32_t
adsp_qrtr_version(uint32_t dst_node)
{
	return a_node != QRTR_NODE_BCAST && dst_node == a_node ? a_qver : q_ver;
}

static void
topaz_modem_thread(void *arg __unused, wait_result_t wr __unused)
{
	IOMemoryMap *smem_map, *pool_map;
	unsigned watch = 0;                         /* 0: serve the modem for the whole session */

	(void)PE_parse_boot_argn("modem_watch", &watch, sizeof(watch));
	(void)PE_parse_boot_argn("modem_part", fw_part, sizeof(fw_part));
	/* sweet_ufs comes up with the root device; the modem is not needed before that */
	{
		uint8_t probe[16];
		int err = EAGAIN;
		for (int i = 0; i < 120 && err == EAGAIN; i++) {
			err = sweet_ufs_read_part(fw_part, fw_files[0].off, probe, sizeof(probe));
			if (err == EAGAIN) {
				IOSleep(500);
			}
		}
		if (err != 0) {
			LOG("partition %s: %d", fw_part, err);
			thread_terminate(current_thread());
			return;
		}
	}
	fw_pick();
	smem_map = map_phys(SMEM_PA, SMEM_SIZE, true);
	pool_map = map_phys(MODEM_POOL_BASE, MODEM_POOL_SIZE, true);
	apcs = (volatile uint8_t *)ml_io_map(APCS_PA, 0x1000);
	tcsr_mutex = (volatile uint8_t *)ml_io_map(TCSR_MUTEX_PA, 0x4000);
	if (smem_map == NULL || pool_map == NULL || apcs == NULL || tcsr_mutex == NULL) {
		LOG("mappings failed");
		thread_terminate(current_thread());
		return;
	}
	smem = (uint8_t *)smem_map->getVirtualAddress();
	pool = (uint8_t *)pool_map->getVirtualAddress();
	LOG("SMEM version %u", *(volatile uint32_t *)(smem + 64 + 7 * 4) >> 16);
	if (audio_on) {
		thread_t at;
		if (adsp_boot() && kernel_thread_start(adsp_thread, NULL, &at) == KERN_SUCCESS) {
			thread_deallocate(at);
		}
	}
	if (!modem_on) {
		thread_terminate(current_thread());
		return;
	}
	if (wifi_enabled() && !smmu_identity(WLAN_SID, WLAN_SID_MASK, "WLAN")) {
		LOG("wifi: no SMMU stream for WLAN, the radio stays off");
		wifi_on = 0;
	}
	(void)PE_parse_boot_argn("ipa", &ipa_on, sizeof(ipa_on));
	if (ipa_on && !ipa_hw_init()) {
		LOG("ipa: not up (step: %s): the modem is told \"AP IPA power off\"", ipa_step);
	}
	if (modem_boot()) {
		modem_watch(smem_partition(0, 1), watch);
	}
	/* keep the mappings: the modem uses SMEM from now on */
	thread_terminate(current_thread());
}

extern "C" void
topaz_modem_init(void)
{
	static bool started;
	thread_t thread;

	(void)PE_parse_boot_argn("modem", &modem_on, sizeof(modem_on));
	(void)PE_parse_boot_argn("audio", &audio_on, sizeof(audio_on));
	if (started || (modem_on == 0 && audio_on == 0)) {
		return;
	}
	started = true;
	if (kernel_thread_start(topaz_modem_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}

#else
extern "C" void topaz_modem_init(void) {}
#endif
