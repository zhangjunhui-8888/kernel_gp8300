/*
 * arch/gpt/include/asm/pgtable.h
 *
 * macros and functions to manipulate page tables.
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
#ifndef __ASM_GPT_PGTABLE_H
#define __ASM_GPT_PGTABLE_H
#include <linux/sizes.h>

#define __PAGE_VALID    (1 << IWV_OFFSE_status_valid)   /* Valid entry */
#define __PAGE_FILE     (1 << IWV_OFFSE_status_lock)    /* lock bit is used for _PAGE_FILE */
#define __PAGE_REF      (1 << IWV_OFFSE_status_ref)     /* page was referenced */
#define __PAGE_CHG      (1 << IWV_OFFSE_status_chg)     /* page was changed */

#define __PAGE_USER_X   (1 << IWV_OFFSE_prot_user_x)    /* User execute */
#define __PAGE_USER_RD  (1 << IWV_OFFSE_prot_user_r)    /* User read */
#define __PAGE_USER_WR  (1 << IWV_OFFSE_prot_user_w)    /* User write */
#define __PAGE_PRIV_X   (1 << IWV_OFFSE_prot_priv_x)    /* Privilege execute */
#define __PAGE_PRIV_RD  (1 << IWV_OFFSE_prot_priv_r)    /* Privilege read */
#define __PAGE_PRIV_WR  (1 << IWV_OFFSE_prot_priv_w)    /* Privilege write */

#define __PAGE_CACHE_IC (1 << IWV_OFFSE_cache_ic)       /* ic cache */
#define __PAGE_CACHE_DC (1 << IWV_OFFSE_cache_dc)       /* dc cache */
#define __PAGE_CACHE_L2C (1 << IWV_OFFSE_cache_l2c)     /* l2c cache */
#define __PAGE_MEM_IO	(1 << IWV_OFFSE_mem_io)		/* mem io in order */

#define __PAGE_SHARE    (1UL << 41)    			/* software  shared */
#define __HAVE_ARCH_PTE_SPECIAL
#ifdef __HAVE_ARCH_PTE_SPECIAL
#define __PAGE_SPECIAL    (1UL << 42)    		/* software special */
#endif

/* Define some higher level generic page attributes. */
#ifdef CONFIG_SMP
#define _PAGE_PRESENT   (__PAGE_VALID|__PAGE_REF|__PAGE_SHARE)
#else
#define _PAGE_PRESENT   (__PAGE_VALID)
#endif

#define _PAGE_ACCESSED  __PAGE_REF
#define _PAGE_DIRTY     __PAGE_CHG

#define _PAGE_CE        (__PAGE_CACHE_IC | __PAGE_CACHE_DC | __PAGE_CACHE_L2C)
#define _PAGE_CHG_MASK  (PAGE_MASK | _PAGE_ACCESSED | _PAGE_DIRTY | _PAGE_CE)
#define _PAGE_EXEC      (__PAGE_USER_X  | __PAGE_PRIV_X)
#define _PAGE_READ      (__PAGE_USER_RD | __PAGE_PRIV_RD)
#define _PAGE_WRITE     (__PAGE_USER_WR | __PAGE_PRIV_WR)
#define _PAGE_USER	__PAGE_USER_RD
#define _PAGE_SHARED	__PAGE_SHARE
#define _PAGE_FILE	__PAGE_FILE

#define _PAGE_KERNEL		(__PAGE_VALID | __PAGE_PRIV_X | __PAGE_PRIV_RD | __PAGE_PRIV_WR | _PAGE_CE)
#define _PAGE_KERNEL_RO		(__PAGE_VALID | __PAGE_PRIV_X | __PAGE_PRIV_RD |  _PAGE_CE)
#define _PAGE_KERNEL_NC		(__PAGE_VALID | __PAGE_PRIV_X | __PAGE_PRIV_RD | __PAGE_PRIV_WR )
#define _PAGE_DEVICE		(__PAGE_VALID | __PAGE_PRIV_X | __PAGE_PRIV_RD | __PAGE_PRIV_WR | __PAGE_MEM_IO)

#define PAGE_NONE		__pgprot(_PAGE_PRESENT | _PAGE_CE)
#define PAGE_READONLY		__pgprot(_PAGE_PRESENT | _PAGE_READ | _PAGE_CE)
#define PAGE_READONLY_X		__pgprot(_PAGE_PRESENT | _PAGE_READ | _PAGE_EXEC | _PAGE_CE)
#define PAGE_COPY		__pgprot(_PAGE_PRESENT | _PAGE_READ | _PAGE_CE)
#define PAGE_COPY_X		__pgprot(_PAGE_PRESENT | _PAGE_READ | _PAGE_EXEC | _PAGE_CE)
#define PAGE_SHARED		__pgprot(_PAGE_PRESENT | _PAGE_READ | _PAGE_WRITE | _PAGE_SHARED | _PAGE_CE)
#define PAGE_SHARED_X		__pgprot(_PAGE_PRESENT | _PAGE_READ | _PAGE_WRITE | _PAGE_SHARED | _PAGE_EXEC | _PAGE_CE)

#define PAGE_KERNEL		__pgprot(_PAGE_KERNEL|_PAGE_DIRTY)
#define PAGE_KERNEL_NC		__pgprot(_PAGE_KERNEL_NC|_PAGE_DIRTY)
#define PAGE_KERNEL_RO		__pgprot(_PAGE_KERNEL_RO|_PAGE_DIRTY)
#define PAGE_DEVICE		__pgprot(_PAGE_DEVICE)

#define PAGE_INVALID		__pgprot(0)

#define pgprot_noncached(prot) __pgprot(((pgprot_val(prot) & (~_PAGE_CE))))

/*
 * How many pointers will a page table level hold expressed in shift
 * PAGE_SHIFT in page.h.
 */
#define PTRS_PER_PTD_SHIFT      (PAGE_SHIFT - BYTES_PER_LONG_SFT)

/*
 * Definitions for fourth level:
 */
#define PTRS_PER_PTE    (1UL << (PTRS_PER_PTD_SHIFT))

/*
 * Definitions for third level:
 *
 * PMD_SHIFT determines the size of the area a third-level page table
 * can map.
 */
#ifndef PMD_FOLDED
#define PMD_SHIFT       (PAGE_SHIFT + (PTRS_PER_PTD_SHIFT))
#define PMD_SIZE        (1UL << PMD_SHIFT)
#define PMD_MASK        (~(PMD_SIZE-1))
#define PTRS_PER_PMD    PTRS_PER_PTE
#endif

/*
 * Definitions for second level:
 *
 * PUD_SHIFT determines the size of the area a second-level page table
 * can map.
 */
#ifndef PUD_FOLDED
#define PUD_SHIFT       (PMD_SHIFT + (PTRS_PER_PTD_SHIFT))
#define PUD_SIZE        (1UL << PUD_SHIFT)
#define PUD_MASK        (~(PUD_SIZE-1))
#define PTRS_PER_PUD    PTRS_PER_PTE
#endif

/*
 * Definitions for first level:
 *
 * PGDIR_SHIFT determines what a first-level page table entry can map.
 */
#if defined(PMD_FOLDED)
#define PGDIR_SHIFT             (PAGE_SHIFT + (PTRS_PER_PTD_SHIFT))
#elif defined(PUD_FOLDED)
#define PGDIR_SHIFT             (PMD_SHIFT + (PTRS_PER_PTD_SHIFT))
#else
#define PGDIR_SHIFT             (PUD_SHIFT + (PTRS_PER_PTD_SHIFT))
#endif
#define PGDIR_SIZE              (1UL << PGDIR_SHIFT)
#define PGDIR_MASK              (~(PGDIR_SIZE-1))
/*#define PTRS_PER_PGD            (1UL << (64 - PGDIR_SHIFT))*/
#define PTRS_PER_PGD            (1UL << (PTRS_PER_PTD_SHIFT))

#define	BOOT_MEMORY_LIMIT_SIZE_ln	29
#define	BOOT_MEMORY_LIMIT_SIZE		(1UL << BOOT_MEMORY_LIMIT_SIZE_ln)

# ifndef __ASSEMBLY__
#include <asm/mmu.h>

/*         xwr */
#define __P000  PAGE_NONE
#define __P001  PAGE_READONLY
#define __P010  PAGE_COPY
#define __P011  PAGE_COPY
#define __P100  PAGE_READONLY_X
#define __P101  PAGE_READONLY_X
#define __P110  PAGE_COPY_X
#define __P111  PAGE_COPY_X

#define __S000  PAGE_NONE
#define __S001  PAGE_READONLY
#define __S010  PAGE_SHARED
#define __S011  PAGE_SHARED
#define __S100  PAGE_READONLY_X
#define __S101  PAGE_READONLY_X
#define __S110  PAGE_SHARED_X
#define __S111  PAGE_SHARED_X

/*
 * 64 MB of vmalloc area is comparable to what's available on other architectures.
 */

#define VMALLOC_START   (PAGE_OFFSET << 1)
#define VMALLOC_END     (PAGE_OFFSET-1)
#define VMALLOC_VMADDR(x) ((unsigned long)(x))

#define MODULE_START	((PAGE_OFFSET+CONFIG_PHYSICAL_START) - SZ_64M)
#define MODULE_END	((PAGE_OFFSET+CONFIG_PHYSICAL_START) - 1)

#define pte_none(x)     (!pte_val(x))
#define pte_present(x)  (pte_val(x) & _PAGE_PRESENT)
#define pte_clear(mm, addr, xp) do { pte_val(*(xp)) = 0; mb(); } while (0)
#define pte_page(pte)   (pfn_to_page((pte_val(pte)&PHYS_MASK) >> PAGE_SHIFT))

#ifndef PMD_FOLDED
#define pmd_none(x)     (!pmd_val(x))
#define pmd_bad(x)      (pmd_val(x) & ~PAGE_MASK)
#define pmd_present(x)  (pmd_val(x) & _PAGE_PRESENT)
#define pmd_clear(xp)   do { pmd_val(*(xp)) = 0; mb();} while (0)
#define pmd_page(pmd)   (pfn_to_page(pmd_val(pmd) >> PAGE_SHIFT))
#define pmd_populate(mm, pmd, pte)  do {pmd_val(*(pmd)) = page_to_phys(pte); mb();}while(0)
#define pmd_populate_kernel(mm, pmd, pte)  do {pmd_val(*(pmd)) = __pa(pte); mb();}while(0)
#define pmd_pgtable(pmd) pmd_page(pmd)
#endif

#ifndef PUD_FOLDED
#define pud_none(x)     (!pud_val(x))
#define pud_bad(x)      (pud_val(x) & ~PAGE_MASK)
#define pud_present(x)  (pud_val(x) & _PAGE_PRESENT)
#define pud_clear(xp)   do { pud_val(*(xp)) = 0; mb();} while (0)
#define pud_page(pud)   (pfn_to_page(pud_val(pud) >> PAGE_SHIFT))
#define pud_populate(mm, pud, pmd) do {pud_val(*(pud)) = __pa(pmd); mb();}while(0)
#endif

#define pgd_none(x)     (!pgd_val(x))
#define pgd_bad(x)      (pgd_val(x) & ~PAGE_MASK)
#define pgd_present(x)  (pgd_val(x) & _PAGE_PRESENT)
#define pgd_clear(xp)   do { pgd_val(*(xp)) = 0; mb(); } while (0)
#define pgd_page(pgd)   (pfn_to_page(pgd_val(pgd) >> PAGE_SHIFT))
#define pgd_populate(mm, pgd, pud) do{  pgd_val(*(pgd)) = __pa(pud); mb(); }while(0)

/*
 * The following only work if pte_present() is true.
 * Undefined behaviour if not..
 */
#define pte_write(pte)          (pte_val(pte) & ((pte_val(pte) & _PAGE_USER) ? __PAGE_USER_WR : __PAGE_PRIV_WR))
#define pte_exec(pte)           (pte_val(pte) & ((pte_val(pte) & _PAGE_USER) ? __PAGE_USER_X : __PAGE_PRIV_X))
#define pte_dirty(pte)          (pte_val(pte) & _PAGE_DIRTY)
#define pte_young(pte)          (pte_val(pte) & _PAGE_ACCESSED)
#define pte_file(pte)		(pte_val(pte) & _PAGE_FILE)

#ifdef __HAVE_ARCH_PTE_SPECIAL
#define pte_special(pte)        (pte_val(pte) & __PAGE_SPECIAL)
#else
#define pte_special(pte)        (0)
#endif

#define pte_wrprotect(pte)      (__pte(pte_val(pte) & ~((pte_val(pte) & _PAGE_USER) ? _PAGE_WRITE : __PAGE_PRIV_WR)))
#define pte_mkwrite(pte)        (__pte(pte_val(pte) | ((pte_val(pte) & _PAGE_USER) ? _PAGE_WRITE : __PAGE_PRIV_WR)))
#define pte_mkold(pte)          (__pte(pte_val(pte) & ~(_PAGE_ACCESSED)))
#define pte_mkyoung(pte)        (__pte(pte_val(pte) | _PAGE_ACCESSED))
#define pte_mkclean(pte)        (__pte(pte_val(pte) & ~(_PAGE_DIRTY)))
#define pte_mkdirty(pte)        (__pte(pte_val(pte) |  _PAGE_DIRTY))
#ifdef __HAVE_ARCH_PTE_SPECIAL
#define pte_mkspecial(pte)      (__pte(pte_val(pte) |  __PAGE_SPECIAL))
#else
#define pte_mkspecial(pte)      (pte)
#endif



/* Page entry (a physical address) to virtual address */
#define page_vaddr(p)           ((unsigned long)__va(p))


/* Certain architectures need to do special things when pte's
 * within a page table are directly modified.  Thus, the following
 * hook is made available.
 */
#define set_pte(pteptr, pteval) do {((*(pteptr)) = (pte_t)(pteval)); mb();}while(0)
#define pte_valid_exec_user(pte) \
	((pte_val(pte) & __PAGE_VALID) && (pte_val(pte) & _PAGE_USER) && (pte_val(pte) & __PAGE_USER_X))

/* Page frame number from entry in PTE table */
#define pte_pfn(pte)		((pte_val(pte) & PHYS_MASK) >> PAGE_SHIFT)
extern void __sync_icache_dcache(pte_t pteval, unsigned long addr);

static inline void set_pte_at(struct mm_struct *mm, unsigned long addr,
			      pte_t *ptep, pte_t pte)
{
#ifdef __HAVE_ARCH_PTE_SPECIAL
	if (pte_valid_exec_user(pte)) {
	    if(!pte_special(pte))
			__sync_icache_dcache(pte, addr);
	}
#else
	/* page is present && page is user  && page is executable
	 * && (page swapin or new page or page migraton
	 *	|| copy_on_write with page copying.)
	 */
	if (pte_valid_exec_user(pte)) {
	    if(!pte_present(*ptep) || pte_pfn(*ptep) != pte_pfn(pte))
			__sync_icache_dcache(pte, addr);
	}
#endif
	set_pte(ptep, pte);
}

/*
 * (pmds are folded into pgds so this doesn't get actually called,
 * but the define is needed for a generic inline function.)
 */
#define set_pmd(pmdptr, pmdval) (*(pmdptr) = pmdval)

static inline pte_t *pmd_page_vaddr(pmd_t pmd)
{
	return __va(pmd_val(pmd) & (s32)PAGE_MASK);
}

/*
 * the pte page can be thought of an array like this: pte_t[PTRS_PER_PTE]
 */
/* Index of the entry in the pte page which control the given virtual address */
#define pte_index(address) (((address) >> PAGE_SHIFT) & (PTRS_PER_PTE - 1))

#define pte_offset_kernel(pmd,address)	(pmd_page_vaddr(*(pmd)) + pte_index(address))
#define pte_offset_map(dir, address)	pte_offset_kernel(dir,address)
#define __pte_free_tlb(tlb, pte, addr) tlb_remove_page((tlb), (pte))

#define pte_unmap(pte)          do { } while (0)

/* Computes entry to PTE table given page frame number */
#define pfn_pte(pfn, prot)      __pte(pfn_to_phys(pfn) | pgprot_val(prot))

/* Third level page table entries */
#ifndef PMD_FOLDED
#define pmd_index(addr)         (((addr) >> PMD_SHIFT) & (PTRS_PER_PMD - 1))
#define pmd_offset(dir, addr)   ((pmd_t *)(page_vaddr(pud_val(*(dir)) & PAGE_MASK)) + pmd_index(addr))
#define __pmd_free_tlb(tlb, pmd, address)       pmd_free((tlb)->mm, pmd)
#endif

/* Second level page table entries */
#ifndef PUD_FOLDED
#define pud_index(addr)         (((addr) >> PUD_SHIFT) & (PTRS_PER_PUD - 1))
#define pud_offset(dir, addr)   ((pud_t *)(page_vaddr(pgd_val(*(dir)) & PAGE_MASK)) + pud_index(addr))
#define __pud_free_tlb(tlb, pud, address)       pud_free((tlb)->mm, pud)
#endif

/* First level page table entries */
#define pgd_index(address)      (((address) >> PGDIR_SHIFT) & (PTRS_PER_PGD-1))
#define pgd_offset(mm, address) ((mm)->pgd + pgd_index(address))

/* a shortcut which implies the use of the kernel's pgd, instead of a process's */
#define pgd_offset_k(address) pgd_offset(&init_mm, address)

/*
 * Conversion functions:
 *
 * What actually goes as arguments to the various functions is less than
 * obvious, but a rule of thumb is that struct page's goes as struct page *,
 * really physical DRAM addresses are unsigned long's, and DRAM "virtual"
 * addresses (the 0xFFFFFFxxxxxxxxxx's) goes as void *'s.
 */
/* Computes entry to PTE table given a page physical address */
//#define mk_pte(page, pgprot)            pfn_pte(page_to_pfn(page), (pgprot))

#define mk_pte(page, pgprot) \
({	\
	pte_t pte;	\
	pte_val(pte) = page_to_phys(page) | pgprot_val(pgprot);	\
	pte;	\
})

/* Computes entry to PTE table given a physical address */
#define mk_pte_phys(p, pgprot)          (__pte(p | pgprot_val(pgprot)))
/* Computes entry to PTE table with modified protections */
#define pte_modify(pteval, newprot)     (__pte((pte_val(pteval) & _PAGE_CHG_MASK) | pgprot_val(newprot)))


extern pgd_t swapper_pg_dir[PTRS_PER_PGD]; /* in mm/init.c. Make sure it is paged aligned */

#define pgd_ERROR(e) printk(KERN_ERR "%s:%d: bad pgd %p(%08lx).\n", __FILE__, __LINE__, &(e), pgd_val(e))
#ifndef PUD_FOLDED
#define pud_ERROR(e) printk(KERN_ERR "%s:%d: bad pud %p(%08lx).\n", __FILE__, __LINE__, &(e), pud_val(e))
#endif
#ifndef PMD_FOLDED
#define pmd_ERROR(e) printk(KERN_ERR "%s:%d: bad pmd %p(%08lx).\n", __FILE__, __LINE__, &(e), pmd_val(e))
#endif
#define pte_ERROR(e) printk(KERN_ERR "%s:%d: bad pte %p(%08lx).\n", __FILE__, __LINE__, &(e), pte_val(e))

#define FIRST_USER_ADDRESS      0

/*
 * Encode and decode a swap entry:
 *	bits 0:		_PAGE_PRESENT (must be zero)
 *	bit  1:		_PAGE_FILE
 *	bits 2-8:	swap type
 *	bits 9-63:	swap offset
 */

#define __SWP_TYPE_SHIFT	2
#define __SWP_TYPE_BITS		7
#define __SWP_OFFSET_BITS	55
#define __SWP_TYPE_MASK		((1 << __SWP_TYPE_BITS) - 1)
#define __SWP_OFFSET_SHIFT	(__SWP_TYPE_BITS + __SWP_TYPE_SHIFT)
#define __SWP_OFFSET_MASK	((1UL << __SWP_OFFSET_BITS) - 1)

#define __swp_type(x)		(((x).val >> __SWP_TYPE_SHIFT) & __SWP_TYPE_MASK)
#define __swp_offset(x)		(((x).val >> __SWP_OFFSET_SHIFT) & __SWP_OFFSET_MASK)
#define __swp_entry(type,offset) ((swp_entry_t) { ((type) << __SWP_TYPE_SHIFT) | ((offset) << __SWP_OFFSET_SHIFT) })

#define __pte_to_swp_entry(pte)	((swp_entry_t) { pte_val(pte) })
#define __swp_entry_to_pte(swp)	((pte_t) { (swp).val })

/*
 * Ensure that there are not more swap files than can be encoded in the kernel
 * PTEs.
 */
#define MAX_SWAPFILES_CHECK() BUILD_BUG_ON(MAX_SWAPFILES_SHIFT > __SWP_TYPE_BITS)

/*
 * Encode and decode a file entry:
 *	bits 0:		_PAGE_PRESENT (must be zero)
 *	bit  1:		_PAGE_FILE
 *	bits 2-63:	file offset / PAGE_SIZE
 */
#define pte_to_pgoff(x)		(pte_val(x) >> 2)
#define pgoff_to_pte(x)		__pte(((x) << 2) | _PAGE_FILE)

#define PTE_FILE_MAX_BITS	62


/*
 * ZERO_PAGE is a global shared page that is always zero: used
 * for zero-mapped memory areas etc..
 */
/* allocated in paging_init, zeroed in mem_init, and unchanged thereafter */
extern struct page * empty_zero_page;
#define ZERO_PAGE(vaddr) (empty_zero_page)

extern void gpt_memblock_init(void);
extern void paging_init(void);
/*
 * No page table caches to initialise
 */
#define pgtable_cache_init()                    do { } while (0)

#define kern_addr_valid(addr)           (1)

# endif /* !__ASSEMBLY__ */

#if defined(PMD_FOLDED)
#include <asm-generic/pgtable-nopmd.h>
#elif defined(PUD_FOLDED)
#include <asm-generic/pgtable-nopud.h>
#endif

#include <asm-generic/pgtable.h>

#endif /* __ASM_GPT_PGTABLE_H */
