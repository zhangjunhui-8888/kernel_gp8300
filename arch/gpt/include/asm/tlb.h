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

#ifndef __ASM_GPT_TLB_H__
#define __ASM_GPT_TLB_H__

#define	DWT_OFFSE_BIG_va	(DWT_OFFSE_va + (KERNEL_PAGE_BIGREG_ln - KERNEL_PAGE_SIZE_ln))
#define	DWT_WIDTH_BIG_va	(DWT_WIDTH_va - (KERNEL_PAGE_BIGREG_ln - KERNEL_PAGE_SIZE_ln))
#define	DWV_OFFSE_BIG_ra	(DWV_OFFSE_ra + (KERNEL_PAGE_BIGREG_ln - KERNEL_PAGE_SIZE_ln))
#define	DWV_WIDTH_BIG_ra	(DWV_WIDTH_ra - (KERNEL_PAGE_BIGREG_ln - KERNEL_PAGE_SIZE_ln))

#define	IWT_OFFSE_BIG_va	(IWT_OFFSE_va + (KERNEL_PAGE_BIGREG_ln - KERNEL_PAGE_SIZE_ln))
#define	IWT_WIDTH_BIG_va	(IWT_WIDTH_va - (KERNEL_PAGE_BIGREG_ln - KERNEL_PAGE_SIZE_ln))
#define	IWV_OFFSE_BIG_ra	(IWV_OFFSE_ra + (KERNEL_PAGE_BIGREG_ln - KERNEL_PAGE_SIZE_ln))
#define	IWV_WIDTH_BIG_ra	(IWV_WIDTH_ra - (KERNEL_PAGE_BIGREG_ln - KERNEL_PAGE_SIZE_ln))

#define tlb_start_vma(tlb, vma)						\
do {									\
	if (!tlb->fullmm)						\
		flush_cache_range(vma, vma->vm_start, vma->vm_end);	\
} while(0)

#define tlb_end_vma(tlb, vma)						\
do {									\
	if (!tlb->fullmm)						\
		flush_tlb_range(vma, vma->vm_start, vma->vm_end);	\
} while (0)

#define __tlb_remove_tlb_entry(tlb, ptep, address) do { } while (0)

#define tlb_flush(tlb) flush_tlb_mm((tlb)->mm)

#ifndef __ASSEMBLY__
#include <linux/pagemap.h>
#include <asm-generic/tlb.h>

pte_t* get_ptep(pgd_t* pgd, unsigned long addr);

#endif

#endif /* __ASM_GPT_TLB_H__ */
