/*
 * arch/gpt/include/asm/asm-defs.h
 *
 * GPT low level macro definitions.
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

#ifndef __ASM_DEFS_H__
#define __ASM_DEFS_H__

/* generate the list of gpt sp registers.
   Aternative approach in file ~/arch/gpt/kernel/asm-offsets.c */
#define SPR__(a,b)      .equ a,b
#include <machine/gpt_spr.h>
#undef SPR__

#include <machine/gpt_kernel.h>
#include <machine/gpt_mach.h>

#define LEVEL_0H   0
#define LEVEL_1H   1
#define LEVEL_0    2
#define LEVEL_1    3
#define LEVEL_TRAP 4
#define LEVEL_MC   5
#define LEVEL_ALT  6

#define ITLB       0
#define DTLB       1

#define CURRPGD		$s0		/* scratch for current_pgd */
#define SYSTAB		$s1		/* scratch for sys_call_table */
#define CPUOFFSET	$s2		/* scratch for per cpu offset*/
#define L0S1		$s3
#define L1S0		$s4		/* scratch for level 1 */
#define L1S1		$s5
#define MCS0		$s6		/* scratch for mc */
#define MCS1		$s7
#define MCS2		$s8
#define MCS3		$s9
#define MCS4		$s10

.macro ldtpci td, label, t4=$t4
        taddpcil        \t4, \label
        taddti          \td, \t4, \label
.endm

.macro ldapci ad, label
        aaddpcilb       \ad, \label
        aadduib         \ad, \label
.endm

.macro rsetui rd, offset, val
        .if (\val < 0)
        rseti           \rd, ((\val) >> \offset) | 0xffffffffffff0000
        .else
        rseti           \rd, ((\val) >> \offset)
        .endif
.endm
/* set long immediate number */
.macro rsetli rd, val
        .if ((\val) == -1)
        rseti           \rd, (\val)
        .elseif ((\val) >> 47)
        rsetui          \rd, 48, (\val)
        insui           \rd, ((\val) >> 32) & 0xffff
        insui           \rd, ((\val) >> 16) & 0xffff
        insui           \rd, ((\val) & 0xffff)
        .elseif ((\val) >> 31)
        rsetui          \rd, 32, (\val)
        insui           \rd, ((\val) >> 16) & 0xffff
        insui           \rd, ((\val) & 0xffff)
        .elseif ((\val) >> 15)
        rsetui          \rd, 16, (\val)
        insui           \rd, (\val) & 0xffff
        .else
        rseti           \rd, (\val)
        .endif
.endm

.macro cal td, label
        ldtpci          \td, \label
        call            \td
.endm

.macro jmp td, label
        ldtpci          \td, \label
        j               \td
.endm

/* Implements _pa(va) to convert virtual address to real address. */
#define PHYS(label) (label - ((-1) << PAGE_OFFSET_sft))

.macro tophysa ad, rs
        rseti           \rs, 1
        shli            \rs, \rs, PAGE_OFFSET_sft
        aaddar_b        \ad, \ad, \rs
.endm

/* Implements _va(pa) to transform real address to virtual address. */
.macro tovirt rd, rs
        rseti           \rd, -1
        shli            \rd, \rd, PAGE_OFFSET_sft
        add             \rd, \rd, \rs
.endm

.macro tovirta ad, rs
        rseti           \rs, -1
        shli            \rs, \rs, PAGE_OFFSET_sft
        aaddar_b        \ad, \ad, \rs
.endm

.macro tovirtt td, rs
        ori             \rs, \rs, -1
        shli            \rs, \rs, PAGE_OFFSET_sft
        taddtr_b        \td, \td, \rs
.endm

.macro  roundup		rreg,align
	shri		\rreg,\rreg,\align
	addi		\rreg,\rreg,1
	shli		\rreg,\rreg,\align	
.endm

.macro  rounddown	rreg,align
	shri		\rreg,\rreg,\align
	shli		\rreg,\rreg,\align	
.endm

.macro lshli rd, offset
        rseti           \rd, 1
        shli            \rd, \rd, \offset
.endm

.macro 	get_cpuid rd
	rsetspr		\rd, THID
	extrui          \rd, \rd, THID_OFFSE_cpu, THID_WIDTH_cpu
.endm

.macro 	get_procid rd, spr
	rsetspr		\rd, \spr
	extrui          \rd, \rd, PSC_OFFSE_procid, PSC_WIDTH_procid
.endm

.macro get_ixi rd, lv
        .if \lv == LEVEL_MC
        rsetspr         \rd, IXIM
        .elseif \lv == LEVEL_0
        rsetspr         \rd, IXI0
        .elseif \lv == LEVEL_1
        rsetspr         \rd, IXI1
        .elseif \lv == LEVEL_0H
        rsetspr         \rd, IXI0H
        .elseif \lv == LEVEL_1H
        rsetspr         \rd, IXI1H
        .else
        rseti           \rd, 0
        .endif
.endm

.macro get_ixa rd, lv
        .if \lv == LEVEL_MC
        rsetspr         \rd, IXAM
        .elseif \lv == LEVEL_0
        rsetspr         \rd, IXA0
        .elseif \lv == LEVEL_1
        rsetspr         \rd, IXA1
        .elseif \lv == LEVEL_0H
        rsetspr         \rd, IXA0H
        .elseif \lv == LEVEL_1H
        rsetspr         \rd, IXA1H
        .else
        rseti           \rd, 0
        .endif
.endm

.macro get_xs rd, lv
        .if \lv == LEVEL_TRAP
        rsetspr         \rd, XST
        .elseif \lv == LEVEL_MC
        rsetspr         \rd, XSM
        .elseif \lv == LEVEL_0
        rsetspr         \rd, XS0
        .elseif \lv == LEVEL_1
        rsetspr         \rd, XS1
        .elseif \lv == LEVEL_0H
        rsetspr         \rd, XS0H
        .elseif \lv == LEVEL_1H
        rsetspr         \rd, XS1H
        .elseif \lv == LEVEL_ALT
        rsetspr         \rd, XSA
        .endif
.endm

.macro get_xr rd, lv
        .if \lv == LEVEL_TRAP
        rsetspr         \rd, XRT
        .elseif \lv == LEVEL_MC
        rsetspr         \rd, XRM
        .elseif \lv == LEVEL_0
        rsetspr         \rd, XR0
        .elseif \lv == LEVEL_1
        rsetspr         \rd, XR1
        .elseif \lv == LEVEL_0H
        rsetspr         \rd, XR0H
        .elseif \lv == LEVEL_1H
        rsetspr         \rd, XR1H
        .elseif \lv == LEVEL_ALT
        rsetspr         \rd, XRA
        .endif
.endm

.macro set_xs rs, lv
        .if \lv == LEVEL_TRAP
        sprsetr         \rs, XST
        .elseif \lv == LEVEL_MC
        sprsetr         \rs, XSM
        .elseif \lv == LEVEL_0
        sprsetr         \rs, XS0
        .elseif \lv == LEVEL_1
        sprsetr         \rs, XS1
        .elseif \lv == LEVEL_0H
        sprsetr         \rs, XS0H
        .elseif \lv == LEVEL_1H
        sprsetr         \rs, XS1H
        .elseif \lv == LEVEL_ALT
        sprsetr         \rs, XSA
        .endif
.endm

.macro set_xr rs, lv
        .if \lv == LEVEL_TRAP
        sprsetr         \rs, XRT
        .elseif \lv == LEVEL_MC
        sprsetr         \rs, XRM
        .elseif \lv == LEVEL_0
        sprsetr         \rs, XR0
        .elseif \lv == LEVEL_1
        sprsetr         \rs, XR1
        .elseif \lv == LEVEL_0H
        sprsetr         \rs, XR0H
        .elseif \lv == LEVEL_1H
        sprsetr         \rs, XR1H
        .elseif \lv == LEVEL_ALT
        sprsetr         \rs, XRA
        .endif
.endm

.macro get_ca	rr
	subfb	\rr, \rr, \rr	
.endm

.macro set_ca	rr
	sat_u_b	\rr, \rr
.endm

.macro init_all_regs val
	rseti		$r15, \val
	rseti		$r14, \val
	rseti		$r13, \val
	rseti		$r12, \val
	rseti		$r11, \val
	rseti		$r10, \val
	rseti		$r9, \val
	rseti		$r8, \val
	rseti		$r7, \val
	rseti		$r6, \val
	rseti		$r5, \val
	rseti		$r4, \val
	rseti		$r3, \val
	rseti		$r2, \val
	rseti		$r1, \val
	rseti		$r0, \val

        aaddri          $a0, $r15, 0
        aaddri          $a1, $r15, 0
        aaddri          $a2, $r15, 0
        aaddri          $a3, $r15, 0
        aaddri          $a4, $r15, 0
        aaddri          $a5, $r15, 0
        aaddri          $a6, $r15, 0
        aaddri          $a7, $r15, 0
        aaddri          $a8, $r15, 0
        aaddri          $a9, $r15, 0
        aaddri          $a10, $r15, 0
        aaddri          $a11, $r15, 0
        aaddri          $a12, $r15, 0
        aaddri          $a13, $r15, 0
        aaddri          $a14, $r15, 0
        aaddri          $a15, $r15, 0
        ssetr           $s0, $r15
        ssetr           $s1, $r15
        ssetr           $s2, $r15
        ssetr           $s3, $r15
        ssetr           $s4, $r15
        ssetr           $s5, $r15
        ssetr           $s6, $r15
        ssetr           $s7, $r15
        tsetr           $t0, $r15
        tsetr           $t1, $r15
        tsetr           $t4, $r15
        tsetr           $t7, $r15
.endm

/* Refresh hardware shadow registers, 
   DPAGE has been set during mm switch */
.macro refresh_shadow_regs
	atrans		$a0, $a0
	atrans		$a1, $a1
	atrans		$a2, $a2
	atrans		$a3, $a3
	atrans		$a4, $a4
	atrans		$a5, $a5
	atrans		$a6, $a6
	atrans		$a7, $a7
	atrans		$a8, $a8
	atrans		$a9, $a9
	atrans		$a10, $a10
	atrans		$a15, $a15
.endm

/* Load PSC indirectly through XSA/XRA and retfi instead of through sprset */
.macro load_psc rr, tr=$t1
        sprsetr         \rr, XSA
        taddpci         \tr, 2f
        rsett           \rr, \tr
        sprsetr         \rr, XRA
        taddpci         \tr, 1f
        call            \tr
1:
        retfi_alt
2:
.endm

.macro or_spr rd, rs, name
        rsetspr         \rs, \name
        or              \rs, \rs, \rd
        sprsetr         \rs, \name
.endm

.macro ra2va_switch rpsc, rr, tr, lb = 1f
        sprsetr         \rpsc, XSA
        ldtpci		\tr, \lb
        rsett           \rpsc, \tr
        tovirt          \rr, \rpsc
        sprsetr         \rr, XRA

        refresh_shadow_regs
        retfi_alt
1:
.endm

.macro va2ra_switch rpsc, rr, tr, lb = 1f
        sprsetr         \rpsc, XSA
        ldtpci		\tr, \lb
        rsett           \rpsc, \tr
        tophys          \rr, \rpsc
        sprsetr         \rr, XRA
        retfi_alt
1:
.endm

.macro tlb_way_gen rway, rlru, rr, tr
        /*
         * way = (lru & A) == A'
         *  0  <- (lru & 0x38) == 0x00
         *  1  <- (lru & 0x26) == 0x20
         *  2  <- (lru & 0x15) == 0x14
         *  3  <- (lru & 0x0b) == 0x0b
         */
	taddpci		\tr, 1f
	andi		\rr, \rlru, 0x38
	xori		\rr, \rr, 0x00
	jci_eq		\tr, \rr, 0
	andi		\rr, \rlru, 0x26
	xori		\rr, \rr, 0x20
	addi		\rway, \rway, 1
	jci_eq		\tr, \rr, 0
	addi		\rway, \rway, 1
	andi		\rr, \rlru, 0x15
	xori		\rr, \rr, 0x14
	countnzb	\rr, \rr, \rr
	add		\rway, \rway, \rr
1:
.endm

.macro tlb_get_base ridx, rway, rr, tr, type
        .if \type == ITLB
	sprsetr         \ridx, IRB
        rsetspr         \rway, IRL
        .else
	sprsetr         \ridx, DRB
        rsetspr         \rway, DRL
        .endif

        tlb_way_gen     \ridx, \rway, \rr, \tr

.endm

#define USER(l, x...)				\
9999:	x;					\
	.section __ex_table,"a";		\
	.align	3;				\
	.dword	9999b,l;			\
	.previous

#endif
