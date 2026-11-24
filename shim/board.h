#pragma once
#include <stdint.h>

/*
 * The unit's hardware identity as the ABL gives it to Android (androidboot.hwversion, androidboot.product.hardware.sku,
 * androidboot.hwc), handed on to xnu in /chosen of the ADT, as iBoot puts a unit's syscfg (WSKU, Regn...) there.
 */
#define BOARD_VALUE_MAX	24

struct board {
	char hwversion[BOARD_VALUE_MAX], sku[BOARD_VALUE_MAX], hwc[BOARD_VALUE_MAX];
	const char *source;	/* where they were found, or why not */
};

/* Read them from the ABL's bootconfig (end of the initrd) or, failing that, from /chosen/bootargs. */
void board_from_abl(uintptr_t dtb, struct board *b);

/*
 * Add a string property to /chosen of the ADT at adt (*size bytes, grown in place: the memory behind it must be
 * free). Returns 0 if /chosen is not there or the ADT is malformed.
 */
int adt_chosen_add_str(uintptr_t adt, uint64_t *size, const char *name, const char *value);
