/*
 * gpt Linux
 *
 * Linux architectural port borrowing liberally from similar works of
 * others.  All original copyrights apply as per the original source
 * declaration.
 *
 * gpt implementation:
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __ASM_GPT_TLBFLUSH_H
#define __ASM_GPT_TLBFLUSH_H

#include <linux/mm.h>
#include <asm/processor.h>
#include <asm/pgtable.h>
#include <asm/pgalloc.h>
#include <asm/current.h>
#include <linux/sched.h>

/*
 * TLB flushing:
 *
 *  - flush_tlb() flushes the current mm struct TLBs
 *  - flush_tlb_all() flushes all processes TLBs
 *  - flush_tlb_mm(mm) flushes the specified mm context TLB's
 *  - flush_tlb_page(vma, vmaddr) flushes one page
 *  - flush_tlb_range(vma, start, end) flushes a range of pages
 *  - flush_tlb_kernel_range(start, end) flushes a range of kernel pages
 */

extern void local_flush_tlb_all(void);
extern void local_flush_tlb_mm(struct mm_struct *);
extern void local_flush_tlb_page(struct vm_area_struct *, unsigned long);
extern void local_flush_tlb_range(struct vm_area_struct *, unsigned long,
	unsigned long);
extern void local_flush_tlb_kernel_range(unsigned long start, unsigned long end);

#ifndef CONFIG_SMP

#define flush_tlb_all()			local_flush_tlb_all()
#define flush_tlb_mm(mm)		local_flush_tlb_mm(mm)
#define flush_tlb_page(vma, page)	local_flush_tlb_page(vma, page)
#define flush_tlb_range(vma, start, end)	\
	local_flush_tlb_range(vma, start, end)
#define flush_tlb_kernel_range(start, end)	local_flush_tlb_kernel_range(start, end)

#else	/* CONFIG_SMP */
extern void flush_tlb_all(void);
extern void flush_tlb_mm(struct mm_struct *mm);
extern void flush_tlb_page(struct vm_area_struct *vma, unsigned long page);
extern void flush_tlb_range(struct vm_area_struct *vma, unsigned long start, unsigned long end);
extern void flush_tlb_kernel_range(unsigned long start, unsigned long end);
#endif	/* CONFIG_SMP */

extern void update_mmu_cache(struct vm_area_struct *vma, unsigned long addr, pte_t *ptep);

extern void switch_tlb(void);

#endif /* __ASM_GPT_TLBFLUSH_H */
