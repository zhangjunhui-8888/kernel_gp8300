/*
 * arch/gpt/mm/pgtable.c
 *
 * This file implements handful of procedures to handle kernel pages. Rest are
 * defined as macros in pgtable.h
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


#define USER_PTRS_PER_PGD       (PTRS_PER_PGD - 1)

/*
 * Allocate and free page tables.
 */

/* Allocate PGD page. Copies PGD entries of kernel space,
 * USER_PTRS_PER_PGD is user space 's pgd entries*/
pgd_t *pgd_alloc(struct mm_struct *mm)
{
        pgd_t *ret = (pgd_t *)get_zeroed_page(GFP_KERNEL|__GFP_REPEAT);
        /*if (ret) {
                memcpy(ret + USER_PTRS_PER_PGD, swapper_pg_dir + USER_PTRS_PER_PGD, (PTRS_PER_PGD - USER_PTRS_PER_PGD) * sizeof(pgd_t));
        }*/
        return ret;
}

void pgd_free(struct mm_struct *mm, pgd_t *pgd)
{
        free_page((unsigned long)pgd);
}

#ifndef PUD_FOLDED
pud_t __init_refok *pud_alloc_one(struct mm_struct *mm, unsigned long addr)
{
        return  (pud_t *)get_zeroed_page(GFP_KERNEL|__GFP_REPEAT);
}
void pud_free(struct mm_struct *mm, pud_t *pud)
{
	BUG_ON((unsigned long)pud & (PAGE_SIZE-1));
        free_page((unsigned long)pud);
}
#endif

#ifndef PMD_FOLDED
pmd_t __init_refok *pmd_alloc_one(struct mm_struct *mm, unsigned long addr)
{
	return (pmd_t *)get_zeroed_page(GFP_KERNEL|__GFP_REPEAT);         
}
void pmd_free(struct mm_struct *mm, pmd_t *pmd)
{
	BUG_ON((unsigned long)pmd & (PAGE_SIZE-1));
        free_page((unsigned long)pmd);
}
#endif

/* called from main tree mm/memory.c */
pte_t __init_refok *pte_alloc_one_kernel(struct mm_struct *mm,
                                         unsigned long address)
{
        pte_t *pte;

	pte = (pte_t *)get_zeroed_page(GFP_KERNEL|__GFP_REPEAT);

        return pte;
}

void pte_free_kernel(struct mm_struct *mm, pte_t *pte)
{
        free_page((unsigned long)pte);
}


