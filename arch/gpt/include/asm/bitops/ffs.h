/*
 * arch/gpt/include/asm/bitops/ffs.h
 *
 * find first bit set.
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

#ifndef _ASM_GPT_BITOPS_FFS_H_
#define _ASM_GPT_BITOPS_FFS_H_

/**
 *  ffs - find first bit set
 *  @x: the word to search
 *
 *  This is defined the same way as
 *  the libc and compiler builtin ffs routines, therefore
 *  differs in spirit from the above ffz (man ffs).
 *
 */
static inline int ffs(int x)
{
        int num;
        unsigned long xul = (unsigned int)x;

        if (!x)
                return 0;

        __asm__ __volatile__ (
        "\n   counttz   %0, %1"
        : "=r"(num)
        : "r" (xul)
        :
        );

        return num + 1;
}

#endif /* _ASM_GPT_BITOPS_FFS_H_ */

