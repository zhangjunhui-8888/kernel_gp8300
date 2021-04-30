/*
 * arch/gpt/kernel/setup.c
 *
 * This file handles the architecture-dependent parts of initialization.
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

#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/stddef.h>
#include <linux/unistd.h>
#include <linux/ptrace.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/console.h>
#include <linux/init.h>
#include <linux/bootmem.h>
#include <linux/serial.h>
#include <linux/initrd.h>
#include <linux/of_fdt.h>
#include <linux/of.h>
#include <linux/memblock.h>
#include <linux/device.h>
#include <linux/of_platform.h>

#include <machine/gpt_kernel.h>
#include <asm/sections.h>
#include <asm/segment.h>
#include <asm/pgtable.h>
#include <asm/page.h>
#include <asm/types.h>
#include <asm/setup.h>
#include <asm/io.h>
#include <asm/delay.h>
#include <asm/cpuinfo.h>

#include "vmlinux.h"

phys_addr_t __fdt_pointer __initdata;
EXPORT_SYMBOL(__fdt_pointer);

phys_addr_t __fdt_end __initdata;
EXPORT_SYMBOL(__fdt_end);

// Init routine for device driver
static int __init gpt_device_probe(void)
{
        of_platform_populate(NULL, NULL, NULL, NULL);
        return 0;
}

device_initcall(gpt_device_probe);

static void __init setup_machine_fdt(phys_addr_t dt_phys)
{
#ifdef CONFIG_OF
	if (!dt_phys || !early_init_dt_scan(__va(dt_phys))) {
		pr_info("\n"
			"Error: invalid device tree blob at physical address 0x%p (virtual address 0x%p)\n"
			"\nPlease check your bootloader.\n",
			(void *)dt_phys, (void *)__va(dt_phys));

		while (true)
			cpu_relax();
	}
#endif
	memblock_set_current_limit(rounddown(CONFIG_PHYSICAL_START + BOOT_MEMORY_LIMIT_SIZE, 1UL << KERNEL_PAGE_BIGREG_ln));
#ifdef CONFIG_OF
	unflatten_device_tree();
#endif
	pr_info("FDT at 0x%p\n", (void*)dt_phys);
}

/*
 * Standard memory resources
 */
static struct resource mem_res[] = {
	{
		.name = "Kernel code",
		.start = 0,
		.end = 0,
		.flags = IORESOURCE_MEM
	},
	{
		.name = "Kernel data",
		.start = 0,
		.end = 0,
		.flags = IORESOURCE_MEM
	}
};

#define kernel_code mem_res[0]
#define kernel_data mem_res[1]

void __init smp_setup_processor_id(void)
{
        /*
         * clear __my_cpu_offset on boot CPU to avoid hang caused by
         * using percpu variable early, for example, lockdep will
         * access percpu variable inside lock_release
         */
        set_my_cpu_offset(0);
}

static void __init request_standard_resources(void)
{
	struct memblock_region *region;
	struct resource *res;

	kernel_code.start   = virt_to_phys(_text);
	kernel_code.end     = virt_to_phys(_etext - 1);
	kernel_data.start   = virt_to_phys(_sdata);
	kernel_data.end     = virt_to_phys(_end - 1);

	for_each_memblock(memory, region) {
		res = alloc_bootmem_low(sizeof(*res));
		res->name  = "System RAM";
		res->start = pfn_to_phys(memblock_region_memory_base_pfn(region));
		res->end = pfn_to_phys(memblock_region_memory_end_pfn(region)) - 1;
		res->flags = IORESOURCE_MEM | IORESOURCE_BUSY;

		request_resource(&iomem_resource, res);

		if (kernel_code.start >= res->start &&
		    kernel_code.end <= res->end)
			request_resource(res, &kernel_code);
		if (kernel_data.start >= res->start &&
		    kernel_data.end <= res->end)
			request_resource(res, &kernel_data);
	}
}

void __init setup_arch(char **cmdline_p)
{
	if(IS_ENABLED(CONFIG_GPT_BUILTIN_DTB))
		__fdt_pointer = __pa(__dtb_start);
	setup_machine_fdt(__fdt_pointer);

	early_ioremap_init();

        setup_cpuinfo();

        /* process 1's initial memory region is the kernel code/data */
        init_mm.start_code = (unsigned long)_stext;
        init_mm.end_code = (unsigned long)_etext;
        init_mm.end_data = (unsigned long)_edata;
        init_mm.brk = (unsigned long)_end;

        *cmdline_p = boot_command_line;

	parse_early_param();

	/* setup bootmem allocator */
	gpt_memblock_init();
	/* paging_init() sets up the MMU and marks all pages as reserved */
        paging_init();
//	request_standard_resources();

#ifdef CONFIG_SMP
	smp_init_cpus();
#endif

#if defined(CONFIG_VT) && defined(CONFIG_DUMMY_CONSOLE)
        if (!conswitchp)
                conswitchp = &dummy_con;
#endif

        printk(KERN_INFO "GPT setup done\n");
}

static int __init gpt_device_init(void)
{
	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);
	return 0;
}
arch_initcall_sync(gpt_device_init);
