/*
 * arch/gpt/include/asm/cpuinfo.h
 *
 * GPT data structure to describe processor.
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

#ifndef __ASM_GPT_CPUINFO_H
#define __ASM_GPT_CPUINFO_H

#include <linux/cpu.h>
#include <linux/init.h>
#include <linux/percpu.h>

struct cpuinfo_gpt {
	struct cpu cpu;
        /* i/d cache info */
        u32 icache_line_size;
        u32 icache_no_lines;
        u32 dcache_line_size;
        u32 dcache_no_lines;

        /* i/d tlb info */
        u32 immu_entries;
        u32 immu_ways;
        u32 dmmu_entries;
        u32 dmmu_ways;

        unsigned long cpuc;
        /*asid info*/
};

DECLARE_PER_CPU(struct cpuinfo_gpt, cpudata_gpt);

void setup_cpuinfo(void);

#endif /* __ASM_GPT_CPUINFO_H */
