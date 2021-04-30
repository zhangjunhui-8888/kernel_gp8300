/*
 * arch/gpt/mm/cache.c
 *
 * Based on arch/sh/include/asm/cacheflush.h
 *
 * GPT cache initilization code.
 *
 * Copyright (C) 2016, General Processor Techologies Inc.
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

#ifndef __GPT_ASM_CACHEFLUSH_H
#define __GPT_ASM_CACHEFLUSH_H

#include <linux/mm.h>
#include <asm/cache.h>

/*
 * This flag is used to indicate that the page pointed to by a pte is clean
 * and does not require cleaning before returning it to the user.
 */

#define PG_dcache_clean PG_arch_1

/* Cache flushing:
 *	See Documentation/cachetlb.txt for more information. Please note that
 *	the implementation assumes PIPT D-cache and ASID-tagged VIVT I-cache.
 *
 *  - flush_cache_all() flushes entire cache
 *  - flush_cache_range(vma, start, end) flushes a range of pages
 *  - flush_cache_page(vma, user_addr, pfn) flushes a page
 *  - flush_icache_range(start, end) flush a range of instructions
 *  - flush_icache_page(vma, page) flush a page of instructions
 *
 */

extern void flush_cache_all(void);

extern void flush_cache_range(struct vm_area_struct *vma,
			      unsigned long start, unsigned long end);
extern void flush_cache_page(struct vm_area_struct *vma,
                                    unsigned long user_addr, unsigned long pfn);

extern void flush_icache_range(unsigned long start, unsigned long end);

extern void copy_to_user_page(struct vm_area_struct *vma,
				     struct page *page,
				     unsigned long vaddr,
				     void *dst, void *src, int len);


#define ARCH_IMPLEMENTS_FLUSH_DCACHE_PAGE 1

extern void flush_dcache_page(struct page *page);

/* Our dcache is PIPT and icache is ASID-tagged, 
 * so do not need flush all cache when exit or exec */
#define flush_cache_mm(mm)			do { } while (0)
#define flush_cache_dup_mm(mm)			do { } while (0)

#define flush_dcache_mmap_lock(mapping) \
        spin_lock_irq(&(mapping)->tree_lock)
#define flush_dcache_mmap_unlock(mapping) \
        spin_unlock_irq(&(mapping)->tree_lock)
#define flush_icache_page(vma,pg)		do { } while (0)
/* Not required on GPT (PIPT D-cache). */
#define flush_cache_vmap(start, end)		do { } while (0)
#define flush_cache_vunmap(start, end)		do { } while (0)
#define copy_from_user_page(vma, page, vaddr, dst, src, len) \
	memcpy(dst, src, len)

extern void __flush_dcache_all(void);
extern void __invalidate_icache_all(void);
extern void __sync_dcache_range(unsigned long start, unsigned long end);
extern void __flush_dcache_page(struct address_space *mapping, struct page *page);

extern void __dma_sync_cache_range(unsigned long start, unsigned long end);
#endif /* __GPT_ASM_CACHEFLUSH_H */
