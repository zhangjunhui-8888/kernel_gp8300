/*
 * arch/gpt/mm/cache-chip2.S
 *
 * GPT cache initilization code.
 *
 * Copyright (C) 2016, General Processor Techologies Inc.
 * scxie <scxie@hxgpt.com>
 * Nick Wu <fwu@hxgpt.com>
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

#include <asm/unistd.h>
#include <asm/page.h>
#include <asm/thread_info.h>

#include <asm/asm-defs.h>
#include <linux/linkage.h>
#include <asm/cache.h>

/* void __invalidate_icache_range(unsigned long start, unsigned long end) 
 */
ENTRY(__invalidate_icache_all)
	fence
	barrier
	taddpci		$t1, 1f
/* The start address must be cache line aligned */
	rseti		$r8, 0
	rseti		$r9, GPT_ICACHE_count_max
1:
	sprsetr		$r8, ICCTL
	addi		$r8, $r8, ICACHE_LINE_BYTES
        jdecnz		$t1, $r9, $r9
	fence
	barrier
        ret
ENDPROC(__invalidate_icache_all)

ENTRY(flush_icache_range)
	fence
	barrier
	taddpci		$t1, 1f
	andi		$r8, $r8, ~(DCACHE_LINE_BYTES - 1)
	aaddri		$a4, $r8, 0
	rseti		$r10, ICACHE_LINE_BYTES * ICACHE_LINE_COUNT
	add		$r10, $r8, $r10
	max_u		$r10, $r9, $r10
	aaddri		$a5, $r9, 0
	taddpci		$t0, 2f
	.align 5
1:
USER(9f,	dctl_sync_l1	$a4)		// flush dcache
	sprsetr		$r8, ICCTL		// invalidate icache, No exceptions.
	aaddai		$a4, $a4, DCACHE_LINE_BYTES
	addi		$r8, $r8, ICACHE_LINE_BYTES
	jcu_lt		$t1, $r8, $r10
	jc_ne		$t0, $r9, $r10
	fence
	barrier
	ret
	.align 4
2:
USER(9f,	dctl_sync_l1	$a4)		// flush dcache
	aaddai		$a4, $a4, DCACHE_LINE_BYTES
	jcau_lt		$t0, $a4, $a5

	ret
	fence
	barrier
9:	ret
ENDPROC(flush_icache_range)

#if 0
dctl_invalid cannot be used for invalidate perticular address range.
/* void __invalidate_dcache_range(unsigned long start, unsigned long end) 
 */
ENTRY(__invalidate_dcache_range)
	taddpci		$t1, 1f	

	andi		$r8, $r8, ~(DCACHE_LINE_BYTES - 1)
	aaddri		$a5, $r8, 0
	aaddri		$a4, $r9, 0		

1:	
	dctl_invalid_l1	$a5	// invalidate l1 dcache
	aaddai		$a5, $a5, DCACHE_LINE_BYTES
	jcau_lt		$t1, $a5, $a4	

	ret
ENDPROC(__invalidate_dcache_range)
#endif

/* void __flush_dcache_all()
 */
	.align 5
ENTRY(__flush_dcache_all)
	fence
	barrier
	taddpci		$t1, 1f	

	aaddpci		$a5, __flush_dcache_all
	rseti		$r8, GPT_DCACHE_count_max
	.align 5
1:	
	dctl_flush_l1	$a5		
	aaddai		$a5, $a5, 1
	dctl_flush_l1	$a5		
	aaddai		$a5, $a5, 1
	dctl_flush_l1	$a5
	aaddai		$a5, $a5, 1
	dctl_flush_l1	$a5	
	aaddai		$a5, $a5, DCACHE_LINE_BYTES - 3
	jdecnz		$t1, $r8, $r8	
	fence
	barrier
	ret
ENDPROC(__flush_dcache_all)

/* void __sync_dcache_range(unsigned long start, unsigned long end)
 */
ENTRY(__sync_dcache_range)
	fence
	barrier
	taddpci		$t1, 1f

	andi		$r8, $r8, ~(DCACHE_LINE_BYTES - 1)
	aaddri		$a5, $r8, 0
	aaddri		$a4, $r9, 0
1:
	dctl_sync_l1	$a5		// clean l1 dcache
	aaddai		$a5, $a5, DCACHE_LINE_BYTES
	jcau_lt		$t1, $a5, $a4
	fence
	barrier
	ret
ENDPROC(__sync_dcache_range)

/* void __dma_sync_cache_range(unsigned long start, unsigned long end) */
ENTRY(__dma_sync_cache_range)
	taddpci		$t1, 1f	
	
	aaddri		$a7, $r8, 0
	aaddri		$a6, $r9, 0		

1:	
	dctl_sync_l1	$a7				// flush l1 dcache
	aaddai		$a7, $a7, DCACHE_LINE_BYTES
	jcau_lt		$t1, $a7, $a6	

	taddpci		$t1, 2f	
	
	aaddri		$a7, $r8, 0
2:
	dctl_sync_l2	$a7				// flush l2 dcache
	aaddai		$a7, $a7, L2CACHE_LINE_BYTES
	jcau_lt		$t1, $a7, $a6	

	ret
ENDPROC(__dma_sync_cache_range)

/*
 * 
 * Invalidate the whole L1Cache and L2Cache respectively.
 *
 */
	.align 5
ENTRY(l1cache_init)
	// Dcache first.
	taddpci		$t1, 1f
	aaddpcilb	$a4, l1cache_init
	rseta		$r8, $a4
	rseti		$r9, DCACHE_LINE_COUNT
1:
	dctl_invalid_l1	$a4
	aaddai		$a4, $a4, DCACHE_LINE_BYTES	
	jdecnz		$t1, $r9, $r9	

	taddpci		$t1, 1f
	rseti		$r9, ICACHE_LINE_COUNT
1:
	sprsetr		$r8, ICCTL
	addi		$r8, $r8, ICACHE_LINE_BYTES
        jdecnz		$t1, $r9, $r9
        ret
ENDPROC(l1cache_init)

	.align 5
ENTRY(l2cache_init)
	taddpci		$t1, 1f
	aaddpcilb	$a4, l2cache_init
	rseti		$r9, L2CACHE_LINE_COUNT
1:
	dctl_invalid_l2	$a4
	aaddai		$a4, $a4, L2CACHE_LINE_BYTES	
	jdecnz		$t1, $r9, $r9	
	ret
ENDPROC(l2cache_init)

