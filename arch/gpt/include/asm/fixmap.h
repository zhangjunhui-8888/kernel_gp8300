/*
 * arch/gpt/include/asm/fixmap.h
 *
 * We use these special fixed_addresses for doing ioremap early
 * in the boot process before memory initialization is complete.
 *
 * Copyright (C) 2015, Optimum Semiconductor Technologies
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

#ifndef __ASM_GPT_FIXMAP_H
#define __ASM_GPT_FIXMAP_H

/*
 * From arch/openrisc/include/asm/fixmap.h:
 *
 * Why exactly do we need 2 empty pages between the top of the fixed
 * addresses and the top of virtual memory?  Something is using that
 * memory space but not sure what right now... If you find it, leave
 * a comment here.
 */
#define FIXADDR_TOP     ((unsigned long) (-2*PAGE_SIZE))

#include <linux/kernel.h>
#include <asm/page.h>

enum fixed_addresses {
	FIX_EARLYCON_MEM_BASE,
	__end_of_permanent_fixed_addresses,
        /*
         * FIX_IOREMAP entries are useful for mapping physical address
         * space before ioremap() is useable, e.g. really early in boot
         * before kmalloc() is working.
         */
#define FIX_N_IOREMAPS  32

#define NR_FIX_BTMAPS           64
#define FIX_BTMAPS_SLOTS        7
#define TOTAL_FIX_BTMAPS        (NR_FIX_BTMAPS * FIX_BTMAPS_SLOTS)

	FIX_BTMAP_END = __end_of_permanent_fixed_addresses,
	FIX_BTMAP_BEGIN = FIX_BTMAP_END + TOTAL_FIX_BTMAPS - 1,
	__end_of_fixed_addresses

};

#define FIXADDR_SIZE            (__end_of_fixed_addresses << PAGE_SHIFT)
#define FIXADDR_START           (FIXADDR_TOP - FIXADDR_SIZE)

#define FIXMAP_PAGE_NORMAL	__pgprot(_PAGE_KERNEL)
#define	FIXMAP_PAGE_IO		__pgprot(_PAGE_DEVICE)
#define FIXMAP_PAGE_CLEAR	__pgprot(0)

extern void __early_set_fixmap(enum fixed_addresses idx,
			       phys_addr_t phys, pgprot_t flags);

#define __set_fixmap __early_set_fixmap

#include <asm-generic/fixmap.h>

#endif /* __ASM_GPT_FIXMAP_H */
