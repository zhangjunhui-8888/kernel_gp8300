/*
 * arch/gpt/mm/trap-chip2.S
 *
 * GPT trap code for user.
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

#include <linux/linkage.h>

#include <asm/unistd.h>
#include <asm/param.h>
#include <asm/page.h>
#include <asm/thread_info.h>
#include <asm/errno.h>
#include <asm/asm-offsets.h>

#include <asm/pgtable.h>
#include <asm/irqflags.h>
#include <asm/asm-defs.h>
#include <asm/cache.h>

/*
 *	$a5 $r6 $r14 $r15 $t0 are reserved in glibc
 *	$r8	- mem
 *	$r9	- new
 *	$r10	- old 
 */

/*Now we will re-execute the compare_and_exchange_32/64 when exception occurs,
and we need to make sure the registers are unchanged,
so we use dctl_lock_l1 to trigger the exception beforehand */
#if 1
.macro compare_and_exchange_32
        andi            $r6, $r8, ((-(1<< GPT_DCACHE_line_ln)) | 3)
        aaddri          $a5, $r6, 0
        taddpci         $t0, 1f
	fence
	.align          GPT_ICACHE_line_ln
        dctl_lock_l1    $a5
        aaddri          $a5, $r8, 0
        ldw             $r8, $a5
        jcw_ne           $t0, $r8, $r10
        stw             $r9, $a5
1:
        aaddri          $a5, $r6, 0
        dctl_unlock_l1  $a5
        retfi_trap
.endm

.macro compare_and_exchange_64
        andi            $r6, $r8, ((-(1<< GPT_DCACHE_line_ln)) | 7)
        aaddri          $a5, $r6, 0
        taddpci         $t0, 1f
	fence
	.align          GPT_ICACHE_line_ln
        dctl_lock_l1    $a5
        aaddri          $a5, $r8, 0
        ldl             $r8, $a5
        jc_ne           $t0, $r8, $r10
        stl             $r9, $a5
1:
        aaddri          $a5, $r6, 0
        dctl_unlock_l1  $a5
        retfi_trap
.endm
#else
.macro compare_and_exchange_32
	fence
        aaddri          $a5, $r8, 0
        ldw             $r8, $a5
        taddpci         $t0, 1f
        jcw_ne		$t0, $r8, $r10
        stw             $r9, $a5
1:
        retfi_trap
.endm

.macro compare_and_exchange_64
	fence
        aaddri          $a5, $r8, 0
        ldl             $r8, $a5
        taddpci         $t0, 1f
        jc_ne           $t0, $r8, $r10
        stl             $r9, $a5
1:
        retfi_trap
.endm
#endif

.macro trap_get_timer
	rsetspr		$r8, TIM
        retfi_trap
.endm

.macro do_trap_default
	retfi_trap
.endm

/*
 * Note: 
 *      These registers $r8, $r9, $a4, $t1 are clobbered by glibc,
 *      so DO NOT use other registers and DO NOT clobber $r9.
 *	$r8	- start
 *	$r9	- end
 */
.macro flush_cache
	taddpci		$t1, 1f
	andi		$r8, $r8, ~(ICACHE_LINE_BYTES - 1)
	aaddri		$a4, $r8, 0
1:
	dctl_sync_l1	$a4		// flush dcache
	sprsetr		$r8, ICCTL	// invalidate icache
	aaddai		$a4, $a4, DCACHE_LINE_BYTES
	addi		$r8, $r8, ICACHE_LINE_BYTES
	jcu_lt		$t1, $r8, $r9	
.endm

.macro trap_entry from, to, handler
	.org trp_vectors + \from * 256
trap_entry_\@_\handler:
	\handler
trap_entry_end_\@_\handler:        
        .if             \to - \from
        trap_entry    "(\from+1)", \to, \handler
        .endif
.endm

        .section ".vector_tables.text","ax"
        .align KERNEL_PAGE_SIZE_ln + 1
/* trap vector table */
ENTRY(trp_vectors)
        trap_entry  0,	5,  do_trap_default
        trap_entry  6,	6,  compare_and_exchange_32
        trap_entry  7,	7,  compare_and_exchange_64
	trap_entry  8,	8,  trap_get_timer
        trap_entry  9,	29, do_trap_default
        trap_entry  30,	30, flush_cache
        trap_entry  31,	63, do_trap_default
END(trp_vectors)

