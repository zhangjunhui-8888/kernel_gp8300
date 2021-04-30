/*
 * arch/gpt/mm/ioremap.c
 *
 * This file remap an arbitrary physical address space into the kernel virtual address space.
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

#include <linux/vmalloc.h>
#include <linux/io.h>
#include <asm/pgalloc.h>
#include <asm/kmap_types.h>
#include <asm/fixmap.h>
#include <asm/bug.h>
#include <asm/pgtable.h>
#include <linux/sched.h>
#include <asm/tlbflush.h>

/*
 * Generic mapping function (not visible outside):
 */

/*
 * Remap an arbitrary physical address space into the kernel virtual
 * address space. Needed when the kernel wants to access high addresses
 * directly.
 *
 * NOTE! We need to allow non-page-aligned mappings too: we will obviously
 * have to convert them into an offset in a page-aligned mapping, but the
 * caller shouldn't need to know that small detail.
 */
void __iomem *__init_refok __ioremap(phys_addr_t addr, unsigned long size, pgprot_t prot)
{
        unsigned long v;
        unsigned long offset, last_addr;
        struct vm_struct *area = NULL;

        /*
         * Mappings have to be page-aligned
         */
        offset = addr & ~PAGE_MASK;
        addr = addr & PAGE_MASK;
        size = PAGE_ALIGN(size + offset);


        /* Don't allow wraparound or zero size */
	last_addr = addr + size - 1;
	if (!size || last_addr < addr || last_addr & (~PHYS_MASK))
		return NULL;

        /*
         * Ok, go for it..
         */
        area = get_vm_area_caller(size, VM_IOREMAP, __builtin_return_address(0));
        if (!area)
                return NULL;
        v = (unsigned long)area->addr;
        if (ioremap_page_range(v, v + size, addr, prot)) {

                vunmap(area->addr);
                return NULL;
        }

        return (void __iomem *)(offset + v);
}
EXPORT_SYMBOL(__ioremap);

void __iounmap(volatile void __iomem *io_addr)
{
	unsigned long addr = (unsigned long)io_addr & PAGE_MASK;

	/*
	 * We could get an address outside vmalloc range in case
	 * of ioremap_cache() reusing a RAM mapping.
	 */
	if (VMALLOC_START <= addr && addr < VMALLOC_END)
		vunmap((void *)addr);
}
EXPORT_SYMBOL(__iounmap);

static pte_t bm_pte[PTRS_PER_PTE] __page_aligned_bss;
static pmd_t bm_pmd[PTRS_PER_PMD] __page_aligned_bss;
static pud_t bm_pud[PTRS_PER_PUD] __page_aligned_bss;
static inline pud_t * __init early_ioremap_pud(unsigned long addr)
{
        pgd_t *pgd;

        pgd = pgd_offset_k(addr);
        BUG_ON(pgd_none(*pgd) || pgd_bad(*pgd));

        return pud_offset(pgd, addr);
}

static inline pmd_t * __init early_ioremap_pmd(unsigned long addr)
{
        pud_t *pud = early_ioremap_pud(addr);

        BUG_ON(pud_none(*pud) || pud_bad(*pud));

        return pmd_offset(pud, addr);
}

static inline pte_t * __init early_ioremap_pte(unsigned long addr)
{
        pmd_t *pmd = early_ioremap_pmd(addr);

        BUG_ON(pmd_none(*pmd) || pmd_bad(*pmd));

        return pte_offset_kernel(pmd, addr);
}

void __init early_ioremap_init(void)
{
        pgd_t *pgd;
        pud_t *pud;
        pmd_t *pmd;
        unsigned long addr = fix_to_virt(FIX_BTMAP_BEGIN);

        pgd = pgd_offset_k(addr);
        pgd_populate(&init_mm, pgd, bm_pud);
        pud = pud_offset(pgd, addr);
        pud_populate(&init_mm, pud, bm_pmd);
        pmd = pmd_offset(pud, addr);
        pmd_populate_kernel(&init_mm, pmd, bm_pte);

        /*
         * The boot-ioremap range spans multiple pmds, for which
         * we are not prepared:
         */
        BUILD_BUG_ON((__fix_to_virt(FIX_BTMAP_BEGIN) >> PMD_SHIFT)
                     != (__fix_to_virt(FIX_BTMAP_END) >> PMD_SHIFT));

        if (pmd != early_ioremap_pmd(fix_to_virt(FIX_BTMAP_END))) {
                WARN_ON(1);
                pr_warn("pmd %p != %p\n",
                        pmd, early_ioremap_pmd(fix_to_virt(FIX_BTMAP_END)));
                pr_warn("fix_to_virt(FIX_BTMAP_BEGIN): %08lx\n",
                        fix_to_virt(FIX_BTMAP_BEGIN));
                pr_warn("fix_to_virt(FIX_BTMAP_END):   %08lx\n",
                        fix_to_virt(FIX_BTMAP_END));

                pr_warn("FIX_BTMAP_END:       %d\n", FIX_BTMAP_END);
                pr_warn("FIX_BTMAP_BEGIN:     %d\n",
                        FIX_BTMAP_BEGIN);
        }

        early_ioremap_setup();
}

void __init __early_set_fixmap(enum fixed_addresses idx,
                               phys_addr_t phys, pgprot_t flags)
{
        unsigned long addr = __fix_to_virt(idx);
        pte_t *pte;

        if (idx >= __end_of_fixed_addresses) {
                BUG();
                return;
        }

        pte = early_ioremap_pte(addr);

        if (pgprot_val(flags)) {
                set_pte(pte, pfn_pte(phys >> PAGE_SHIFT, flags));
        } else {
                pte_clear(&init_mm, addr, pte);
                flush_tlb_kernel_range(addr, addr+PAGE_SIZE);
        }
}

void __iomem *ioport_map(unsigned long port, unsigned int nr)
{
	return ioremap(port, nr);
}
EXPORT_SYMBOL(ioport_map);

void ioport_unmap(void __iomem *addr)
{
	iounmap(addr);
}
EXPORT_SYMBOL(ioport_unmap);

