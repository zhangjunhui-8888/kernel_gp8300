/*
 * arch/gpt/mm/init.c, refer to arm64
 *
 * This file handles the memory management initialization.
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

#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/ptrace.h>
#include <linux/mman.h>
#include <linux/mm.h>
#include <linux/swap.h>
#include <linux/smp.h>
#include <linux/bootmem.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/blkdev.h>       /* for initrd_* */
#include <linux/pagemap.h>
#include <linux/memblock.h>
#include <linux/initrd.h>
#include <linux/of_fdt.h>
#include <linux/dma-mapping.h>
#include <linux/dma-contiguous.h>

#include <asm/segment.h>
#include <asm/pgalloc.h>
#include <asm/dma.h>
#include <asm/io.h>
#include <asm/tlb.h>
#include <asm/mmu_context.h>
#include <asm/kmap_types.h>
#include <asm/tlbflush.h>
#include <asm/sections.h>

#include <asm/spr.h>
#include <asm/fixmap.h>


/* Page aligned */
pgd_t swapper_pg_dir[PTRS_PER_PGD] __attribute__((__aligned__(PAGE_SIZE)));
/* Page aligned */
struct page * empty_zero_page;
EXPORT_SYMBOL(empty_zero_page);

#ifdef CONFIG_BLK_DEV_INITRD
static int __init early_initrd(char *p)
{
	unsigned long start, size;
	char *endp;

	start = memparse(p, &endp);
	if (*endp == ',') {
		size = memparse(endp + 1, NULL);

		initrd_start = (unsigned long)__va(start);
		initrd_end = (unsigned long)__va(start + size);
	}
	return 0;
}
early_param("initrd", early_initrd);
#endif

/*
 * Limit the memory size that was specified via FDT.
 */
static int __init early_mem(char *p)
{
        phys_addr_t limit;

        if (!p)
                return 1;

        limit = memparse(p, &p) & PAGE_MASK;
        pr_notice("Memory limited to %lldMB\n", limit >> 20);

        memblock_enforce_memory_limit(limit);

        return 0;
}
early_param("mem", early_mem);


#ifdef CONFIG_ZONE_DMA
static phys_addr_t max_zone_dma_phys(void)
{
	phys_addr_t offset = memblock_start_of_DRAM() & GENMASK_ULL(63, 32);
	return min(offset + (1ULL << 32), memblock_end_of_DRAM());
}
#endif

static void __init zone_sizes_init(unsigned long min, unsigned long max)
{
	struct memblock_region *reg;
	unsigned long zone_size[MAX_NR_ZONES], zhole_size[MAX_NR_ZONES];
	unsigned long max_dma = min;

	memset(zone_size, 0, sizeof(zone_size));

	/* 4GB maximum for 32-bit only capable devices */
	if (IS_ENABLED(CONFIG_ZONE_DMA)) {
		max_dma = PFN_DOWN(max_zone_dma_phys());
		zone_size[ZONE_DMA] = max_dma - min;
	}
	zone_size[ZONE_NORMAL] = max - max_dma;

	memcpy(zhole_size, zone_size, sizeof(zhole_size));

	for_each_memblock(memory, reg) {
		unsigned long start = memblock_region_memory_base_pfn(reg);
		unsigned long end = memblock_region_memory_end_pfn(reg);

		if (start >= max)
			continue;

		if (IS_ENABLED(CONFIG_ZONE_DMA) && start < max_dma) {
			unsigned long dma_end = min(end, max_dma);
			zhole_size[ZONE_DMA] -= dma_end - start;
		}

		if (end > max_dma) {
			unsigned long normal_end = min(end, max);
			unsigned long normal_start = max(start, max_dma);
			zhole_size[ZONE_NORMAL] -= normal_end - normal_start;
		}
	}

	free_area_init_node(0, zone_size, min, zhole_size);
}

#ifdef CONFIG_HAVE_ARCH_PFN_VALID
int pfn_valid(unsigned long pfn)
{
	return memblock_is_memory(pfn << PAGE_SHIFT);
}
EXPORT_SYMBOL(pfn_valid);
#endif

#ifndef CONFIG_SPARSEMEM
static void gpt_memory_present(void)
{
}
#else
static void gpt_memory_present(void)
{
	struct memblock_region *reg;

	for_each_memblock(memory, reg){
		memory_present(0, memblock_region_memory_base_pfn(reg),
			       memblock_region_memory_end_pfn(reg));
	}
}
#endif

void __init gpt_memblock_init(void)
{
	phys_addr_t dma_phys_limit = 0;
	/*
	 * Register the kernel text, kernel data, initrd, and initial
	 * pagetables with memblock.
	 */
	memblock_reserve(__pa(_text), _end - _text);

#ifdef CONFIG_BLK_DEV_INITRD
	if (initrd_start)
		memblock_reserve(__pa(initrd_start), initrd_end - initrd_start);
#endif
	early_init_fdt_scan_reserved_mem();

#ifdef CONFIG_ZONE_DMA
	dma_phys_limit = max_zone_dma_phys();
#endif
	dma_contiguous_reserve(dma_phys_limit);
		
	memblock_allow_resize();
	memblock_dump_all();				
}

static void __init bootmem_init(void)
{
	unsigned long min, max;

	min = PFN_UP(memblock_start_of_DRAM());
	max = PFN_DOWN(memblock_end_of_DRAM());	

	/*
	 * Sparsemem tries to allocate bootmem in memory_present(), so must be
	 * done after the fixed reservations.
	 */
	gpt_memory_present();

	sparse_init();
	zone_sizes_init(min, max);
	
	high_memory = __va((max << PAGE_SHIFT) - 1) + 1;			
	max_pfn = max_low_pfn = max;
}


static void __init *early_alloc(unsigned long sz)
{
	void *ptr = __va(memblock_alloc(sz, sz));
	memset(ptr, 0, sz);
	return ptr;
}

extern const char _s_kernel_ro[], _e_kernel_ro[];

int __init create_mem_map_page_table(unsigned long start, unsigned long end)
{
        unsigned long address, start_page, end_page;
        pgd_t *pgd;
        pud_t *pud;
        pmd_t *pmd;
        pte_t *pte;

        start_page = (unsigned long)__va(start & PAGE_MASK);
        end_page = (unsigned long)__va(PAGE_ALIGN((unsigned long) end));

        for (address = start_page; address < end_page; address += PAGE_SIZE)
        {
                pgd = pgd_offset_k(address);
                if (pgd_none(*pgd))
                {
                        pud = (pud_t *) early_alloc(PAGE_SIZE);
                        clear_page(pud);
                        pgd_populate(&init_mm, pgd, pud);
                }

                pud = pud_offset(pgd, address);
                if (pud_none(*pud))
                {
                        pmd = (pmd_t *) early_alloc(PAGE_SIZE);
                        clear_page(pmd);
                        pud_populate(&init_mm, pud, pmd);
                }

                pmd = pmd_offset(pud, address);
                if (pmd_none(*pmd))
                {
                        pte = (pte_t *) early_alloc(PAGE_SIZE);
                        clear_page(pte);
                        pmd_populate_kernel(&init_mm, pmd, pte);
                }

                pte = pte_offset_kernel(pmd, address);
                if (pte_none(*pte))
                {
                        pgprot_t prot;

                        if (address < (unsigned long) _s_kernel_ro)
                                prot = PAGE_INVALID;
#ifndef CONFIG_FTRACE
                        else if (address >= (unsigned long) _e_kernel_ro)
                                prot = PAGE_KERNEL;
                        else
                                prot = PAGE_KERNEL_RO;
#else
			else
				prot = PAGE_KERNEL;			
#endif
#if 0
			if (start == (0x0F0007400ULL & PAGE_MASK))
				prot = PAGE_DEVICE;
#endif
                        set_pte(pte, mk_pte_phys(__pa(address), prot));
                }
        }
        return 0;
}


/*
 * Map all physical memory into kernel's address space.
 *
 * This is explicitly coded for two-level page tables, so if you need
 * something else then this needs to change.
 */
static void __init map_mem(void)
{
        unsigned long p, e;
        struct memblock_region *region;

        printk(KERN_INFO "Setting up paging and PTEs.\n");

        asm volatile ("sseta "CURRPGD", %0\n"::"a"(__pa(init_mm.pgd)) : "memory", CURRPGD);

        for_each_memblock(memory, region) {
                p = (unsigned long) region->base & PAGE_MASK;
                e = p + (unsigned long) region->size;
		
                create_mem_map_page_table(p, e);

                printk(KERN_INFO "Memory: 0x%llx-0x%llx\n",
                       region->base, region->base + region->size);
        }
#if 0
	p = 0x0F0007400ULL & PAGE_MASK;
	e = p + PAGE_SIZE;
	create_mem_map_page_table(p, e);
#endif
}

void __init paging_init(void)
{
	void *zero_page;
		
	/* Build page tables */
	map_mem();
/*
 * The address is limited to BOOT_MEMORY_LIMIT_SIZE in setup_machine_fdt(),
 * and reset here.
 */
	memblock_set_current_limit(MEMBLOCK_ALLOC_ANYWHERE);

	switch_tlb();
	flush_cache_all();

	bootmem_init();
	
	zero_page=early_alloc(PAGE_SIZE);
	empty_zero_page=virt_to_page(zero_page);
	
}

/* References to section boundaries */

/* Called from init/main.c */
void __init mem_init(void)
{
        set_max_mapnr(pfn_to_page(max_pfn) - mem_map);

        /* this will put all low memory onto the freelists */
        free_all_bootmem();

        mem_init_print_info(NULL);

#define MLK(b, t) b, t, ((t) - (b)) >> 10
#define MLM(b, t) b, t, ((t) - (b)) >> 20
#define MLG(b, t) b, t, ((t) - (b)) >> 30
#define MLK_ROUNDUP(b, t) b, t, DIV_ROUND_UP(((t) - (b)), SZ_1K)

	pr_notice("Virtual kernel memory layout:\n"
		  "    vmalloc : 0x%16lx - 0x%16lx   (%6ld GB)\n"

		  "    fixed   : 0x%16lx - 0x%16lx   (%6ld KB)\n"
		  "    modules : 0x%16lx - 0x%16lx   (%6ld MB)\n"
		  "    memory  : 0x%16lx - 0x%16lx   (%6ld MB)\n"
		  "      .init : 0x%p" " - 0x%p" "   (%6ld KB)\n"
		  "      .text : 0x%p" " - 0x%p" "   (%6ld KB)\n"
		  "      .data : 0x%p" " - 0x%p" "   (%6ld KB)\n",
		  MLG(VMALLOC_START, VMALLOC_END),

		  MLK(FIXADDR_START, FIXADDR_TOP),
		  MLM(MODULE_START, MODULE_END),
		  MLM(PAGE_OFFSET, (unsigned long)high_memory),
		  MLK_ROUNDUP(__init_begin, __init_end),
		  MLK_ROUNDUP(_text, _etext),
		  MLK_ROUNDUP(_sdata, _edata));

#undef MLK
#undef MLM
#undef MLK_ROUNDUP
	BUILD_BUG_ON(TASK_SIZE				> MODULE_START);
	BUG_ON(TASK_SIZE				> MODULE_START);
        return;
}

#ifdef CONFIG_BLK_DEV_INITRD
/* Called from init/initramfs.c */
void free_initrd_mem(unsigned long start, unsigned long end)
{
        free_reserved_area((void *)start, (void *)end, -1, "initrd");
}
#endif

/* Called from init/main.c */
void free_initmem(void)
{
        free_initmem_default(-1);
}

