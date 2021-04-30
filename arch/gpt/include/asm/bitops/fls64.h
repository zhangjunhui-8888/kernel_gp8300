/*
 * arch/gpt/include/asm/bitops/fls64.h
 *
 * find last set bit in a 64-bit word.
 *
 * Copyright (C) 2015-2017, Optimum Semiconductor Technologies
 *  Enrique Barria <ebarria@optimumsemi.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see the file COPYING, or write
 * to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef _ASM_GPT_BITOPS_FLS64_H_
#define _ASM_GPT_BITOPS_FLS64_H_

#include <asm/types.h>

/**
 *  fls64 - find last set bit in a 64-bit word
 *  @x: the word to search
 *
 *  This is defined in a similar way as the libc and compiler builtin
 *  ffsll, but returns the position of the most significant set bit.
 *
 *  fls64(value) returns 0 if value is 0 or the position of the last
 *  set bit if value is nonzero. The last (most significant) bit is
 *  at position 64.
 *
 */

static __always_inline int fls64(__u64 x)
{
        if (x == 0)
                return 0;

        return __fls(x) + 1;
}

#endif /* _ASM_GPT_BITOPS_FLS64_H_ */

