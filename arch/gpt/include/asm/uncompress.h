/*
 * linux/arch/gpt/include/mach/uncompress.h
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __MACH_GPT_UNCOMPRESS_H__
#define __MACH_GPT_UNCOMPRESS_H__


extern char input_data[];
extern char input_data_end[];
static inline void putc(int c) {}

static void arch_decomp_puts(const char *ptr)
{
	char c;

	while ((c = *ptr++) != '\0') {
		if (c == '\n')
			putc('\r');
		putc(c);
	}
}
#define ARCH_HAVE_DECOMP_PUTS

#endif /* __MACH_GPT_UNCOMPRESS_H__ */
