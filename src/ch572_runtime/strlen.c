#include "ch32fun.h"

// Slightly more optimal strlen implementation.
// Default library uses signed char for this loop. QingKe V3[BCFV] cores include
// the vendor-specific XW extension, which adds compressed unsigned byte loads.
// This implementation saves two bytes. Make of that what you will.

size_t strlen(const char *s) {
    const unsigned char *src = (const unsigned char *) s;
	const unsigned char *a = src;
	for (; *src; src++);
	return src-a;
}