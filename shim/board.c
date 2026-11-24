/*
 * The unit's identity from the ABL, for xnu (board.h).
 *
 * For a boot image v4 the ABL builds Android's bootconfig (the androidboot.* parameters) and appends it to the
 * initrd it hands over: the parameters' text, then u32 size, u32 checksum (sum of the text's bytes) and the
 * 12-byte magic "#BOOTCONFIG\n", ending at /chosen/linux,initrd-end. Linux finds it there the same way
 * (init/main.c get_boot_config_from_initrd: the magic may sit up to 3 bytes before the end, for alignment), and
 * that is what Android shows as /proc/bootconfig. An ABL that puts the parameters on the command line instead
 * leaves them in /chosen/bootargs; that is the second place looked at.
 *
 * MMU off: everything is Device memory, read a byte at a time (or aligned words).
 */
#include "board.h"
#include "fdt.h"

#define BOOTCONFIG_MAGIC	"#BOOTCONFIG\n"
#define BOOTCONFIG_MAGIC_LEN	12
#define BOOTCONFIG_ALIGN_SLACK	3	/* Linux: "Grub may align the size of initrd to 4" */
#define ADT_NAME_LEN		32	/* ADT property: char name[32], u32 length, value padded to 4 */
#define ADT_LEN_MASK		0x7fffffffU	/* bit 31: placeholder flag */
#define ADT_MAX_DEPTH		16

static const char KEY_HWVERSION[] = "androidboot.hwversion";
static const char KEY_SKU[] = "androidboot.product.hardware.sku";
static const char KEY_HWC[] = "androidboot.hwc";

static inline uint8_t rd8(uintptr_t a) { return *(volatile uint8_t *)a; }
static inline uint32_t rd32(uintptr_t a) { return *(volatile uint32_t *)a; }
static inline void wr8(uintptr_t a, uint8_t v) { *(volatile uint8_t *)a = v; }
static inline void wr32(uintptr_t a, uint32_t v) { *(volatile uint32_t *)a = v; }

static uint32_t rd32_le_bytes(uintptr_t a)
{
	return rd8(a) | (uint32_t)rd8(a + 1) << 8 | (uint32_t)rd8(a + 2) << 16 | (uint32_t)rd8(a + 3) << 24;
}

static int is_space(uint8_t c)
{
	return c == ' ' || c == '\t' || c == '\r';
}

/* Copy [v, end) without quotes and surrounding blanks into out; returns 1 if anything is left. */
static int take_value(uintptr_t v, uintptr_t end, char *out)
{
	int n = 0;

	while (v < end && (is_space(rd8(v)) || rd8(v) == '"'))
		v++;
	while (end > v && (is_space(rd8(end - 1)) || rd8(end - 1) == '"'))
		end--;
	for (; v < end && n < BOARD_VALUE_MAX - 1; v++)
		out[n++] = (char)rd8(v);
	out[n] = 0;
	return n != 0;
}

/* Does [k, end) (blanks trimmed) equal key? */
static int key_is(uintptr_t k, uintptr_t end, const char *key)
{
	while (k < end && is_space(rd8(k)))
		k++;
	while (end > k && is_space(rd8(end - 1)))
		end--;
	for (; k < end && *key; k++, key++)
		if (rd8(k) != (uint8_t)*key)
			return 0;
	return k == end && *key == 0;
}

/*
 * key=value entries separated by sep1 or sep2 (bootconfig: newlines or ';'; a command line: blanks). Fills the three
 * values found; returns how many.
 */
static int scan_pairs(uintptr_t p, uintptr_t end, char sep1, char sep2, struct board *b)
{
	int found = 0;

	while (p < end) {
		uintptr_t e = p, eq = 0;
		while (e < end && rd8(e) != (uint8_t)sep1 && rd8(e) != (uint8_t)sep2 && rd8(e) != 0) {
			if (rd8(e) == '=' && !eq)
				eq = e;
			e++;
		}
		if (eq) {
			if (!b->hwversion[0] && key_is(p, eq, KEY_HWVERSION))
				found += take_value(eq + 1, e, b->hwversion);
			else if (!b->sku[0] && key_is(p, eq, KEY_SKU))
				found += take_value(eq + 1, e, b->sku);
			else if (!b->hwc[0] && key_is(p, eq, KEY_HWC))
				found += take_value(eq + 1, e, b->hwc);
		}
		if (e < end && rd8(e) == 0)
			break;
		p = e + 1;
	}
	return found;
}

/* The bootconfig block at the end of [start, end): its text in [*text, *text + *size). */
static const char *bootconfig_at(uintptr_t start, uintptr_t end, uintptr_t *text, uint32_t *size)
{
	uintptr_t m = 0;

	if (end - start < BOOTCONFIG_MAGIC_LEN + 8)
		return "initrd too small for a bootconfig";
	for (int back = 0; back <= BOOTCONFIG_ALIGN_SLACK && !m; back++) {
		uintptr_t at = end - BOOTCONFIG_MAGIC_LEN - back;
		int i = 0;
		while (i < BOOTCONFIG_MAGIC_LEN && rd8(at + i) == (uint8_t)BOOTCONFIG_MAGIC[i])
			i++;
		if (i == BOOTCONFIG_MAGIC_LEN)
			m = at;
	}
	if (!m)
		return "no #BOOTCONFIG at the end of the initrd";
	uint32_t sz = rd32_le_bytes(m - 8), csum = rd32_le_bytes(m - 4), sum = 0;
	if (sz > m - 8 - start)
		return "bootconfig size runs past the initrd";
	uintptr_t t = m - 8 - sz;
	for (uint32_t i = 0; i < sz; i++)
		sum += rd8(t + i);
	if (sum != csum)
		return "bootconfig checksum wrong";
	*text = t;
	*size = sz;
	return 0;
}

void board_from_abl(uintptr_t dtb, struct board *b)
{
	uint64_t rd_start, rd_end;
	uintptr_t text, val;
	uint32_t size, len;
	const char *why;

	b->hwversion[0] = b->sku[0] = b->hwc[0] = 0;
	if (!fdt_chosen_u64(dtb, "linux,initrd-start", &rd_start) || !fdt_chosen_u64(dtb, "linux,initrd-end", &rd_end) ||
	    rd_end <= rd_start)
		why = "no initrd in /chosen";
	else if ((why = bootconfig_at(rd_start, rd_end, &text, &size)) == 0 &&
		 scan_pairs(text, text + size, '\n', ';', b) && b->hwversion[0]) {
		b->source = "abl-bootconfig";
		return;
	} else if (!why)
		why = "no androidboot.hwversion in the bootconfig";
	if (fdt_chosen_prop(dtb, "bootargs", &val, &len) && scan_pairs(val, val + len, ' ', '\t', b) && b->hwversion[0]) {
		b->source = "abl-cmdline";
		return;
	}
	b->source = why;
}

/* ---- ADT */

static uintptr_t adt_skip_props(uintptr_t p, uintptr_t end, uint32_t n)
{
	for (uint32_t i = 0; i < n && p; i++) {
		if (p + ADT_NAME_LEN + 4 > end)
			return 0;
		uint32_t len = rd32(p + ADT_NAME_LEN) & ADT_LEN_MASK;
		p += ADT_NAME_LEN + 4 + ((len + 3) & ~3U);
	}
	return p <= end ? p : 0;
}

static uintptr_t adt_skip_node(uintptr_t p, uintptr_t end, int depth)
{
	if (!p || depth > ADT_MAX_DEPTH || p + 8 > end)
		return 0;
	uint32_t np = rd32(p), nc = rd32(p + 4);
	p = adt_skip_props(p + 8, end, np);
	for (uint32_t i = 0; i < nc && p; i++)
		p = adt_skip_node(p, end, depth + 1);
	return p;
}

/* Is the "name" property of the node at p equal to want? */
static int adt_node_named(uintptr_t p, uintptr_t end, const char *want)
{
	uint32_t np = rd32(p);
	uintptr_t q = p + 8;
	static const char name[] = "name";

	for (uint32_t i = 0; i < np && q + ADT_NAME_LEN + 4 <= end; i++) {
		uint32_t len = rd32(q + ADT_NAME_LEN) & ADT_LEN_MASK;
		int j = 0;
		while (j < (int)sizeof(name) && rd8(q + j) == (uint8_t)name[j])
			j++;
		if (j == (int)sizeof(name)) {
			uintptr_t v = q + ADT_NAME_LEN + 4;
			int k = 0;
			while (want[k] && k < (int)len && rd8(v + k) == (uint8_t)want[k])
				k++;
			return want[k] == 0 && (k == (int)len || rd8(v + k) == 0);
		}
		q += ADT_NAME_LEN + 4 + ((len + 3) & ~3U);
	}
	return 0;
}

int adt_chosen_add_str(uintptr_t adt, uint64_t *size, const char *name, const char *value)
{
	uintptr_t end = adt + *size, p, chosen = 0, at;
	uint32_t vlen = 0, nlen = 0;

	if (*size < 8)
		return 0;
	while (value[vlen])
		vlen++;
	vlen++;					/* with its NUL */
	while (name[nlen])
		nlen++;
	if (nlen >= ADT_NAME_LEN)
		return 0;
	p = adt_skip_props(adt + 8, end, rd32(adt));
	for (uint32_t i = 0, nc = rd32(adt + 4); i < nc && p && !chosen; i++) {
		if (adt_node_named(p, end, "chosen"))
			chosen = p;
		else
			p = adt_skip_node(p, end, 1);
	}
	if (!chosen || !(at = adt_skip_props(chosen + 8, end, rd32(chosen))))
		return 0;
	uint32_t grow = ADT_NAME_LEN + 4 + ((vlen + 3) & ~3U);
	for (uintptr_t q = end; q-- > at;)	/* move what follows up, from the top down */
		wr8(q + grow, rd8(q));
	for (uint32_t i = 0; i < ADT_NAME_LEN; i++)
		wr8(at + i, i < nlen ? (uint8_t)name[i] : 0);
	wr32(at + ADT_NAME_LEN, vlen);
	for (uint32_t i = 0; i < ((vlen + 3) & ~3U); i++)
		wr8(at + ADT_NAME_LEN + 4 + i, i < vlen - 1 ? (uint8_t)value[i] : 0);
	wr32(chosen, rd32(chosen) + 1);
	*size += grow;
	return 1;
}
