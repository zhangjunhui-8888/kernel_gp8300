/*
 * arch/gpt/mm/tlb.c
 *
 * GPT translation table management.
 *
 * Copyright (C) 2015, Optimum Semiconductor Technologies
 *  Enrique Barria <ebarria@optimumsemi.com>
 *
 * Copyright (C) 2018, General Processor Techologies Inc.
 * scxie <scxie@hxgpt.com>
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

#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/ptrace.h>
#include <linux/mman.h>
#include <linux/mm.h>
#include <linux/init.h>

#include <asm/segment.h>
#include <asm/tlbflush.h>
#include <asm/mmu_context.h>
#include <asm/tlbhw.h>
#include <asm/cacheflush.h>
/* Assembly function, refer to tlb-xx.S. */
extern void invalidate_tlb_and_switch(void);

#define MAX_TLB_RANGE	(NUM_TLB_ENTRY * PAGE_SIZE)

#define refresh_shadow_aregs() 				\
	do {asm volatile (					\
	".irp regno,1,3,4,5,6,7,8,9,10,15\n"		\
	"aaddai	$a\\regno, $a\\regno, 0\n"			\
	".endr\n"					\
	:::"$a1","$a3","$a4","$a5","$a6","$a7","$a8","$a9","$a10","$a15");} while (0)

/* Invalidate range of dtlb entries with a specified ASID and 0 is for kernel page. */
static inline void __invalidate_dtlb_range(unsigned long start, unsigned long end, unsigned long procid)
{
	unsigned long tag;
	asm volatile (
	"fence\n"
	"barrier\n"
	"taddpci	$t0, 1f\n"
	"taddpci	$t1, 2f\n"
	"rseti		%1, 0\n"
	"sprsetr	%1, %7\n"
	".align 5\n"
	"1:\n"
	"sprsetr	%0, %4\n"
	"rsetspr	%1, %5\n"
	"ext_u_b	%1, %1\n"
	"jc_ne		$t1, %1, %2\n"
	"sprsetr	%0, %6\n"
	"2:\n"
	"addi		%0, %0, 1\n"
	"jc_ne		$t0, %0, %3\n"
	"fence\n"
	"barrier\n"
	: "+&r"(start), "=&r"(tag)
	: "r"(procid), "r"(end), "i"(DRB), "i"(DRT), "i"(DWB), "i"(DWV)
	: "memory", "$t0", "$t1");
}

/* Invalidate range of dtlb entries with a specified ASID and 0 is for kernel page. */
static inline void __invalidate_itlb_range(unsigned long start, unsigned long end, unsigned long procid)
{
	unsigned long tag;
	asm volatile (
	"fence\n"
	"barrier\n"
	"taddpci	$t0, 1f\n"
	"taddpci	$t1, 2f\n"
	"rseti		%1, 0\n"
	"sprsetr	%1, %7\n"
	".align 5\n"
	"1:\n"
	"sprsetr	%0, %4\n"
	"rsetspr	%1, %5\n"
	"ext_u_b	%1, %1\n"
	"jc_ne		$t1, %1, %2\n"
	"sprsetr	%0, %6\n"
	"2:\n"
	"addi		%0, %0, 1\n"
	"jc_ne		$t0, %0, %3\n"
	"fence\n"
	"barrier\n"
	: "+&r"(start), "=&r"(tag)
	: "r"(procid), "r"(end), "i"(IRB), "i"(IRT), "i"(IWB), "i"(IWV)
	: "memory", "$t0", "$t1");
}

static inline void invalidate_tlb_user_entries(unsigned long start, unsigned long end)
{
	unsigned long tag;
	asm volatile (
	"fence\n"
	"barrier\n"
	"taddpci	$t0, 1f\n"
	"rseti		%1, 0\n"
	"sprsetr	%1, %6\n"
	"sprsetr	%1, %10\n"
	".align 5\n"
	"1:\n"
	"taddpci	$t1, 2f\n"
	"sprsetr	%0, %3\n"
	"rsetspr	%1, %4\n"
	"ext_u_b	%1, %1\n"
	"jci_eq		$t1, %1, 0\n"
	"sprsetr	%0, %5\n"
	"2:\n"
	"sprsetr	%0, %7\n"
	"taddpci	$t1, 3f\n"
	"rsetspr	%1, %8\n"
	"ext_u_b	%1, %1\n"
	"jci_eq		$t1, %1, 0\n"
	"sprsetr	%0, %9\n"
	"3:\n"
	"addi		%0, %0, 1\n"
	"jc_ne		$t0, %0, %2\n"
	"fence\n"
	"barrier\n"
	: "+&r"(start), "=&r"(tag)
	: "r"(end), 
          "i"(IRB), "i"(IRT), "i"(IWB), "i"(IWV),
	  "i"(DRB), "i"(DRT), "i"(DWB), "i"(DWV)
	: "memory", "$t0", "$t1");

}
static inline void __invalidate_tlb_page(unsigned long addr, unsigned long procid)
{
	unsigned long index;

	addr >>= PAGE_SHIFT;
	
	index = addr & (BIT(IWB_WIDTH_index)-1);
	index <<= IWB_WIDTH_way;
	__invalidate_itlb_range(index, index + NUM_TLB_WAY, procid);

	index = addr & (BIT(DWB_WIDTH_index)-1);
	index <<= DWB_WIDTH_way;
	__invalidate_dtlb_range(index, index + NUM_TLB_WAY, procid);

}

/*
 * Invalidate all TLB entries.
 *
 */
void local_flush_tlb_all(void)
{
	unsigned long flags;

	local_irq_save(flags);
	invalidate_tlb_user_entries(0, NUM_TLB_ENTRY);
	__invalidate_icache_all();
	local_irq_restore(flags);
}
/*
 * Invalidate the selected mm context only.
 *
 */
void local_flush_tlb_mm(struct mm_struct *mm)
{
	unsigned long max;
	unsigned long flags;
	unsigned long procid = ASID(mm);
	/* The child process may exit(such as do_execve) before running,
	 * the initial asid value is 0, so DO NOT use BUG_ON here.
	 */
	if (procid == 0)
	  return ;

	local_irq_save(flags);

	//__invalidate_icache_all();
	__invalidate_itlb_range(0, NUM_TLB_ENTRY, procid);
	__invalidate_dtlb_range(0, NUM_TLB_ENTRY, procid);
	refresh_shadow_aregs();	

	local_irq_restore(flags);
}
/*
 * Invalidate a single page.
 *
 */
void local_flush_tlb_page(struct vm_area_struct *vma, unsigned long addr)
{
	unsigned long flags;

	struct mm_struct *mm = vma->vm_mm;
	unsigned long procid = ASID(mm);
	if(procid == 0)
		return;

	local_irq_save(flags);

	__invalidate_tlb_page(addr, procid);
//	__flush_icache_all();
	refresh_shadow_aregs();	

	local_irq_restore(flags);
}

void local_flush_tlb_range(struct vm_area_struct *vma, unsigned long start, unsigned long end)
{
	unsigned long flags;

	struct mm_struct *mm = vma->vm_mm;
	unsigned long procid = ASID(mm);
	if(procid == 0)
		return;

	start = start & ~(PAGE_SIZE - 1);
	if ((end - start) > MAX_TLB_RANGE)
		end = start + MAX_TLB_RANGE;
	
	local_irq_save(flags);

	while (start < end)
	{
		__invalidate_tlb_page(start, procid);
		start += PAGE_SIZE;
	}
//	__flush_icache_all();
	refresh_shadow_aregs();	

	local_irq_restore(flags);
}

void local_flush_tlb_kernel_range(unsigned long start, unsigned long end)
{
	unsigned long flags;

	start = start & ~(PAGE_SIZE - 1);
	if ((end - start) > MAX_TLB_RANGE)
		end = start + MAX_TLB_RANGE;

	local_irq_save(flags);

	while (start < end)
	{	
		__invalidate_tlb_page(start, 0);
		start += PAGE_SIZE;
	}
//	__flush_icache_all();
	refresh_shadow_aregs();	

	local_irq_restore(flags);
}

/*
 * Invalidate tlb and switch to small mode.
 */
void __init switch_tlb(void)
{
	invalidate_tlb_and_switch();	
}

/*
 * Called at the end of pagefault, for a userspace mapped page
 *  -pre-install the corresponding TLB entry into MMU
 *  -Finalize the delayed D-cache flush of kernel mapping of page due to
 *      flush_dcache_page(), copy_user_page()
 *
 */
void update_mmu_cache(struct vm_area_struct *vma, unsigned long addr, pte_t *ptep)
{
	mb();
}

