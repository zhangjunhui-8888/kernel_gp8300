/*
 * arch/gpt/include/asm/bitops/__fls.h
 *
 * find last (most-significant) set bit in a long word.
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

#ifndef _ASM_GPT_BITOPS___FLS_H_
#define _ASM_GPT_BITOPS___FLS_H_

#include <asm/types.h>

/**
 *  __fls - find last (most-significant) set bit in a long word
 *  @word: the word to search
 *
 *  Undefined if no set bit exists, so code should check against 0 first.
 *
 */
static __always_inline unsigned long __fls(unsigned long word)
{
        int num;

        __asm__ __volatile__ (
        "\n   countlz   %0, %1"
        : "=r"(num)
        : "r" (word)
        :
        );

        return BITS_PER_LONG - 1 - num;
}

#endif /* _ASM_GPT_BITOPS___FLS_H_ */

