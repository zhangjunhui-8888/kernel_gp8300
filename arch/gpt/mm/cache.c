/*
 * arch/gpt/mm/cache-chip2.c
 *
 * GPT cache initilization code.
 *
 * Copyright (C) 2016, General Processor Techologies Inc.
 * scxie <scxie@hxgpt.com>
 * Nick Wu <fwu@hxgpt.com>
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
#include <linux/mm.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/fs.h>
#include <linux/smp.h>
#include <linux/module.h>
#include <asm/mmu_context.h>
#include <asm/cache.h>
#include <asm/cacheflush.h>

void flush_cache_all(void)
{
	unsigned long flag;
	
	local_irq_save(flag);
	
	__flush_dcache_all();
	__invalidate_icache_all();
	
	local_irq_restore(flag);
}
EXPORT_SYMBOL(flush_cache_all);

void flush_cache_range(	struct vm_area_struct *vma, unsigned long start, unsigned long end)
{
	unsigned long flag;
	local_irq_save(flag);
	if (vma->vm_flags & VM_EXEC) {
		__invalidate_icache_all();
	}
	local_irq_restore(flag);
}

void flush_cache_page(struct vm_area_struct *vma,
                                    unsigned long user_addr, unsigned long pfn)
{
	unsigned long flag;
	local_irq_save(flag);
        if (vma->vm_flags & VM_EXEC) {
		__invalidate_icache_all();
        }
	local_irq_restore(flag);
	
}


void __flush_dcache_page(struct address_space *mapping, struct page *page)

{
        unsigned long addr = (unsigned long)page_address(page);
        /*
         * Writeback any data associated with the kernel mapping of this
         * page.  This ensures that data in the physical page is mutually
         * coherent with the kernels mapping.
         */
        __sync_dcache_range(addr, addr + PAGE_SIZE);
}

/*

 * Ensure cache coherency between kernel mapping and userspace mapping

 * of this page.

 */
 
void __sync_icache_dcache(pte_t pte, unsigned long addr)
{
	struct page *page = pte_page(pte);
	unsigned long flags;
	addr = (unsigned long)page_address(page);


	local_irq_save(flags);
	if (!test_and_set_bit(PG_dcache_clean, &page->flags)) {
		__sync_dcache_range(addr, addr + (PAGE_SIZE << compound_order(page)));
	}
	__invalidate_icache_all();
	local_irq_restore(flags);
	
}

void flush_dcache_page(struct page *page)
{
	unsigned long flags;
	local_irq_save(flags);
	if (test_bit(PG_dcache_clean, &page->flags))
		clear_bit(PG_dcache_clean, &page->flags);
	local_irq_restore(flags);
#if 0
        struct address_space *mapping;
        /*
         * The zero page is never written to, so never has any dirty
         * cache lines, and therefore never needs to be flushed.
         */

        if (page == ZERO_PAGE(0))
                return;

        mapping = page_mapping(page);
        if (mapping && !mapping_mapped(mapping))
                clear_bit(PG_dcache_clean, &page->flags);
        else {
                __flush_dcache_page(mapping, page);
                if (mapping)
                        __flush_icache_all();

                set_bit(PG_dcache_clean, &page->flags);
        }
#endif
}

EXPORT_SYMBOL(flush_dcache_page);

/*
 * Copy user data from/to a page which is mapped into a different processes
 * address space.
 *
 * Note that this code needs to run on the current CPU.
 */
void copy_to_user_page(struct vm_area_struct *vma,
				     struct page *page,
				     unsigned long vaddr,
				     void *dst, void *src, int len)
{
	unsigned long flags;
	local_irq_save(flags);
#ifdef CONFIG_SMP
	preempt_disable();
#endif
	memcpy(dst, src, len);
	if (vma->vm_flags & VM_EXEC) {			   
	  __sync_dcache_range((unsigned long)dst, (unsigned long)dst + len);
	  __invalidate_icache_all();
	}
#ifdef CONFIG_SMP
	preempt_enable();
#endif	
	local_irq_restore(flags);
}
