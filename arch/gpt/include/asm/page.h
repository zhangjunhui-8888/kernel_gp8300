/*
 * arch/gpt/include/asm/page.h
 *
 * Page table definitions.
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

#ifndef __ASM_GPT_PAGE_H
#define __ASM_GPT_PAGE_H

#include <asm/bitsperlong.h>

/* Configure the number of page table levels.
   Supported 2, 3 or 4 levels. 4 levels by default.
   Select one of the following to reduce levels.
 */
#undef  CONFIG_PGTABLE_2
#undef  CONFIG_PGTABLE_3

/* Assume 4 levels by default */
#undef PMD_FOLDED
#undef PUD_FOLDED

/* CONFIG_PGTABLE_2 overides CONFIG_PGTABLE_3 */
#if defined(CONFIG_PGTABLE_2)
   #define PMD_FOLDED
   #define PUD_FOLDED
#elif defined(CONFIG_PGTABLE_3)
   #define PUD_FOLDED
#endif


/* PAGE_SHIFT determines the page size */

#define PAGE_SHIFT      15
#ifdef __ASSEMBLY__
#define PAGE_SIZE       (1 << PAGE_SHIFT)
#else
#define PAGE_SIZE       (1UL << PAGE_SHIFT)
#endif
#define PAGE_MASK       (~(PAGE_SIZE-1))

/*
 * virtual memory layout from kernel's point of view
 * PAGE_OFFSET -- the first address of the first page of memory.
 */
/* Top 1TB of Virtual Space memory is where kernel resides */
/*#define PAGE_OFFSET   0xFFFFFF0000000000 */
/* Express PAGE_OFFSET as a shift operation */

#define PAGE_OFFSET_sft 40
#ifndef __ASSEMBLY__
#define PAGE_OFFSET     ((-1UL) << PAGE_OFFSET_sft)
#else
#define PAGE_OFFSET     ((-1) << PAGE_OFFSET_sft)
#endif

#if BITS_PER_LONG == 64
#define BITS_PER_LONG_SFT  6
#define BYTES_PER_LONG_SFT  3
#else
#error Not supported for BITS_PER_LONG != 64
#endif

#define PHYS_PFN_OFFSET		(CONFIG_PHYSICAL_START >> PAGE_SHIFT)	
#define ARCH_PFN_OFFSET		((unsigned long)PHYS_PFN_OFFSET)

/*
 * Highest possible physical address supported.
 */
#define PHYS_MASK_SHIFT		(PAGE_OFFSET_sft)
#define PHYS_MASK		(((1UL) << PHYS_MASK_SHIFT) - 1)


/* This is not necessarily the right place for this, but it's needed by
 * drivers/of/fdt.c
 */
#include <linux/pfn.h>
#include <asm/setup.h>

#ifndef __ASSEMBLY__

#define clear_page(page)        memset((page), 0, PAGE_SIZE)
#define copy_page(to, from)     memcpy((to), (from), PAGE_SIZE)
#define clear_user_page(addr, vaddr, page)	\
do {						\
	clear_page(addr);			\
	flush_dcache_page(page);		\
} while (0)

#define copy_user_page(to, from, vaddr, page)	\
do {						\
	copy_page((to), (from));		\
	flush_dcache_page(page);		\
} while (0)

/*
 * These are used to make use of C type-checking..
 */
typedef struct { unsigned long pte; } pte_t;
#define pte_val(x)      ((x).pte)
#define __pte(x)        ((pte_t) { (x) } )

#ifndef PMD_FOLDED
typedef struct { unsigned long pmd; } pmd_t;
#define pmd_val(x)      ((x).pmd)
#define __pmd(x)        ((pmd_t) { (x) } )
#endif

#ifndef PUD_FOLDED
typedef struct { unsigned long pud; } pud_t;
#define pud_val(x)      ((x).pud)
#define __pud(x)        ((pud_t) { (x) } )
#endif

typedef struct { unsigned long pgd; } pgd_t;
#define pgd_val(x)      ((x).pgd)
#define __pgd(x)        ((pgd_t) { (x) } )

typedef struct { unsigned long pgprot; } pgprot_t;
#define pgprot_val(x)   ((x).pgprot)
#define __pgprot(x)     ((pgprot_t) { (x) } )

typedef struct page *pgtable_t;

#define __va(x) ((void *)((unsigned long)(x) + PAGE_OFFSET))
#define __pa(x) ((unsigned long) (x) - PAGE_OFFSET)

#define phys_to_pfn(phys)       ((unsigned long) ((phys) >> PAGE_SHIFT))
#define pfn_to_phys(pfn)        ((phys_addr_t) ((pfn) << PAGE_SHIFT))
#define virt_to_pfn(virt)	(phys_to_pfn(__pa(virt)))
#define pfn_to_virt(pfn)        (__va(pfn_to_phys(pfn)))

/* page_to_pfn() and pfn_to_page() from memory model */
#define phys_to_page(phys)	(pfn_to_page(phys_to_pfn(phys)))
#define page_to_phys(page)      (pfn_to_phys((unsigned long)page_to_pfn(page)))
#define virt_to_page(virt)	(pfn_to_page(virt_to_pfn(virt)))
#define page_to_virt(page)      (pfn_to_virt(page_to_pfn((const unsigned long)page)))

#ifdef CONFIG_HAVE_ARCH_PFN_VALID
extern int pfn_valid(unsigned long pfn);
#endif

#define virt_addr_valid(kaddr)  (pfn_valid(virt_to_pfn(kaddr)))

#endif /* __ASSEMBLY__ */



#define VM_DATA_DEFAULT_FLAGS \
	(((current->personality & READ_IMPLIES_EXEC) ? VM_EXEC : 0) | \
	 VM_READ | VM_WRITE | VM_MAYREAD | VM_MAYWRITE | VM_MAYEXEC)

#include <asm-generic/memory_model.h>
#include <asm-generic/getorder.h>

#endif /* __ASM_GPT_PAGE_H */
