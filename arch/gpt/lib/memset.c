/*
 * Copyright (C) 2018, General Processor Techologies Inc.
 * scxie <scxie@hxgpt.com>
 *
 * It is based on demo code originally Copyright 2001 by Intel Corp, taken from
 * http://www.embedded.com/showArticle.jhtml?articleID=19205567
 *
 * Attempts were made, unsuccessfully, to contact the original
 * author of this code (Michael Morrow, Intel).  Below is the original
 * copyright notice.
 *
 * This software has been developed by Intel Corporation.
 * Intel specifically disclaims all warranties, express or
 * implied, and all liability, including consequential and
 * other indirect damages, for the use of this program, including
 * liability for infringement of any proprietary rights,
 * and including the warranties of merchantability and fitness
 * for a particular purpose. Intel does not assume any
 * responsibility for and errors which may appear in this program
 * not any responsibility to update it.
 */

#include <linux/export.h>
#include <linux/types.h>
#include <linux/stddef.h>
#include <linux/compiler.h>
#include <linux/string.h>

void *memset(void *v_src, int c, __kernel_size_t n)
{
	char *src = v_src;
	uint64_t *i_src;
	uint64_t w64 = 0;

	/* Truncate c to 8 bits */
	c = (c & 0xFF);

	if (unlikely(c)) {
		/* Make a repeating word out of it */
		w64 = c;
		w64 |= w64 << 8;
		w64 |= w64 << 16;
		w64 |= w64 << 32;
	}

	if (likely(n >= 8)) {
		/* Align the destination to a double word boundary */
		/* This is done in an endian independent manner */
		switch ((unsigned long) src & 7) {
		case 1:
			*src++ = c;
			--n;
		case 2:
			*src++ = c;
			--n;
		case 3:
			*src++ = c;
			--n;
		case 4:
			*src++ = c;
			--n;
		case 5:
			*src++ = c;
			--n;
		case 6:
			*src++ = c;
			--n;
		case 7:
			*src++ = c;
			--n;
		}

		i_src  = (void *)src;

		/* Do as many full-word copies as we can */
		for (; n >= 8; n -= 8)
			*i_src++ = w64;

		src  = (void *)i_src;
	}

	/* Simple, byte oriented memset or the rest of count. */
	while (n--)
		*src++ = c;

	return v_src;
}

EXPORT_SYMBOL(memset);
