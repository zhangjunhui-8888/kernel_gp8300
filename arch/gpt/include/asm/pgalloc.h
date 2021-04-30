/*
 * arch/gpt/include/asm/pgalloc.h
 *
 * Page table manipulation procedures.
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

#ifndef __ASM_GPT_PGALLOC_H
#define __ASM_GPT_PGALLOC_H

#include <asm/page.h>
#include <linux/threads.h>
#include <linux/mm.h>
#include <linux/memblock.h>
#include <linux/bootmem.h>

pgd_t *pgd_alloc(struct mm_struct *mm);
void pgd_free(struct mm_struct *mm, pgd_t *pgd);

#ifndef PMD_FOLDED
pmd_t *pmd_alloc_one(struct mm_struct *mm, unsigned long addr);
void pmd_free(struct mm_struct *mm, pmd_t *pmd);
#endif

#ifndef PUD_FOLDED
pud_t *pud_alloc_one(struct mm_struct *mm, unsigned long addr);
void pud_free(struct mm_struct *mm, pud_t *pud);
#endif

extern pte_t *pte_alloc_one_kernel(struct mm_struct *mm, unsigned long address);
extern void pte_free_kernel(struct mm_struct *mm, pte_t *pte);

static inline struct page *pte_alloc_one(struct mm_struct *mm,
                                         unsigned long address)
{
        struct page *pte;
        pte = alloc_pages(GFP_KERNEL|__GFP_REPEAT, 0);
        if (!pte)
                return NULL;
        clear_page(page_address(pte));
        if (!pgtable_page_ctor(pte)) {
                __free_page(pte);
                return NULL;
        }
        return pte;
}

static inline void pte_free(struct mm_struct *mm, struct page *pte)
{
        pgtable_page_dtor(pte);
        __free_page(pte);
}

#define check_pgt_cache()          do { } while (0)

#endif /* __ASM_GPT_PGALLOC_H */
