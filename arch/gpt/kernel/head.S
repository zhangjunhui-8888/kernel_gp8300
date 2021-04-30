/*
 * arch/gpt/kernel/head.S
 *
 * GPT kernel startup code.
 *
 * Copyright (C) 2015, Optimum Semiconductor Technologies
 *  Enrique Barria <ebarria@optimumsemi.com>
 * Copyright (C) 2018, General Processor Techologies Inc.
 *  Feng Wu <fwu@hxgpt.com>
 *  scxie <scxie@hxgpt.com>
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
#include <asm/asm-offsets.h>

#include <machine/gpt_kernel.h>
#include <asm/pgtable.h>
#include <asm/asm-defs.h>
#include <asm/irqflags.h>
#include <asm/cache.h>
#include <uapi/asm/ptrace.h>

#include <linux/of_fdt.h>
#include <linux/linkage.h>

        __HEAD
        .globl _start
_start:
#ifndef CONFIG_GPT_BUILTIN_DTB
      /* save fdt pointer from bootloader*/
       	ldapci          $a7, __fdt_pointer
       	stl             $r9, $a7
      /* save fdt end */
	addi		$r10, $r9, 0 
	aaddri		$a7, $r10, 4 
	ldw		$r10, $a7
	swap		$r10, $r10
	shri		$r10, $r10, 32 
	add 		$r9, $r9, $r10				
       	ldapci          $a7, __fdt_end
       	stl             $r9, $a7
#endif

#ifdef  CONFIG_GPT_SIMULATOR_ERRTA2
      /* clear all registers */
        init_all_regs  0
#endif
        rseti           $r5, 0
	// Disable all features in CPU
        sprsetr         $r5, CPUC
	// Disable TIM and TCD
	rseti		$r0, 1
	sprsetr		$r0, TIMC
	sprsetr		$r5, TCD0C
	// Clear any spurious interrupts
	sprsetr		$r5, XPEN
	rseti		$r0, -1
        sprsetr         $r0, XPS
	// Reset PSC and PSCM, disable cache coherency and memory speculative access
	rseti		$r0, 0x3 << PSC_OFFSE_mem
        sprsetr         $r0, PSC
        sprsetr         $r0, PSCM

        taddpci         $t0, start_entry
#ifdef CONFIG_SMP
        get_cpuid       $r5
        jci_eq          $t0, $r5, 0
        taddpci         $t0, secondary_entry
#endif
        j               $t0

/* Should never be here... */
	.align 5
ENTRY(fatal_error)
        taddpci         $t0, 1f
1:
        j               $t0

#ifdef	CONFIG_DBG_MONITOR
#define	IX_ONE_EXCEP_SIZE 256
ENTRY(setup_mc_monitor)
	rsetspr         $r15, MXB
	taddpci         $t1, 5f
	jci_eq          $t1, $r15, 0

    aaddri          $a5, $r15, IX_MACHINE_CHECK *  IX_ONE_EXCEP_SIZE

    ldapci          $a7, ixm_vectors
    aaddai          $a6, $a7, IX_MACHINE_CHECK *  IX_ONE_EXCEP_SIZE
    aaddai          $a7, $a6, IX_ONE_EXCEP_SIZE
    taddpci         $t1, 1f
1:
    ldul            $r14, $a5, 8
    stul            $r14, $a6, 8
    jcau_lt         $t1, $a6, $a7

    rsetspr         $r15, MXB
    rsetli          $r14, IX_DBG_STEP *  IX_ONE_EXCEP_SIZE
    aaddri          $a5, $r15, 0
    ldapci          $a7, ixm_vectors

    aaddar_b        $a6, $a7, $r14
    aaddai          $a7, $a6, (IX_DBG_DATA - IX_DBG_STEP + 1) * IX_ONE_EXCEP_SIZE
    taddpci         $t1, 2f
2:
    ldul            $r14, $a5, 8
    stul            $r14, $a6, 8
    jcau_lt         $t1, $a6, $a7

    rsetspr     $r15, TXB
    rsetli      $r14, 63 * IX_ONE_EXCEP_SIZE
    add         $r15, $r14, $r15
    aaddri      $a5, $r15, 0
    ldapci      $a6, trp_vectors
    aaddar_b    $a6, $a6, $r14
    aaddai      $a7, $a6, IX_ONE_EXCEP_SIZE
    taddpci     $t1, 3f
3:
    ldul        $r14, $a5, 8
    stul        $r14, $a6, 8
    jcau_lt     $t1, $a6, $a7

5:
    ret
ENDPROC(setup_mc_monitor)
#endif

	/* void start_entry(unsigned int cpu_id"$r5") */
	.align 5
ENTRY(start_entry)
	// Disable TCM
        sprsetr         $r5, L2CCTLC
	/* First of all setup a stack and prepare machine check vector
	   so that we can capture any early exceptions. */
	/* Set stack to (unsigned long)(&init_thread_union) + THREAD_SIZE*/
        ldapci          $asp, init_thread_union + THREAD_SIZE - 16
#ifdef	CONFIG_DBG_MONITOR
    /* Setup debug monitor*/
        cal            $t0, setup_mc_monitor
1:
#endif
	/* Setup machine check handlers. */
        ldapci          $a0, ixm_vectors
        rseta           $r0, $a0
        sprsetr         $r0, MXB
	/* All exceptions go to mc. */
	rseti           $r0, -1
        sprsetr         $r0, XMS

	// Enable default CPU features
        rsetli          $r0, CPUC_DEFAULT
	sprsetr		$r0, CPUC

	// Invalid all caches.
        cal		$t0, l2cache_init
        cal		$t0, l1cache_init

        /* Turn on l2sync for the first core and turn off
           the others. */
	/* BUG: L2CCTLC sync_en is not imlemented, sync_mask is used as both mask and enable.*/
        rseti           $r0, (1 << L2CCTLC_OFFSE_sync_en) | (0x1 << L2CCTLC_OFFSE_sync_mask) | (0x1 << L2CCTLC_OFFSE_sync_set)
        sprsetr         $r0, L2CCTLC

	// Initialize PSC
	// PROCID = 0 
	// Enable ic, dc and l2c
	// Disable FP, VPU and MMU
	// Disable intx, int0, int1, intmc
	// Enable memory speculative access and cache coherency
        rsetli          $r0, CACHE_MODE | (1 << PSC_OFFSE_interrupt_mc)
	sprsetr         $r0, PSC
	// This profile is good for PSCM in the whole life of kernel
	sprsetr		$r0, PSCM
	// NOTE: Before io remap we should not access any IO device.

	/* Clear _bss, i.e. region from __bss_start to _end */
	ldapci		$a0, __bss_start
	ldapci		$a1, __bss_stop
	rsubaa_l	$r1, $a1, $a0

        taddpci         $t0, 1f
	.align 3
1:
        stul            $r5, $a0, 8
        jdecnz          $t0, $r1, $r1

	/* Enable MMU
           1. Clear TLB buffer
           2. Prepare page table (early usage)
           3. Setup page size configuration
           ??? Do we have to redirect MC handlers for TLB refilling? 
           No TLB refilling is allowed until we have a real page table.
           4. Enable MMU in PSC and other PSC copies
         */
        // Invalidate tlb
	taddpci		$t0, 1f
	sprsetr		$r5, IWV
	sprsetr		$r5, DWV
        rseti		$r2, (1 << (IWB_WIDTH_way + IWB_WIDTH_index)) - 1
	.align 3
1:
	sprsetr		$r2, IWB
	jdecnz		$t0, $r2, $r2
	taddpci		$t0, 1f
        rseti		$r2, (1 << (DWB_WIDTH_way + DWB_WIDTH_index)) - 1
1:
	sprsetr		$r2, DWB
	jdecnz		$t0, $r2, $r2
	sprsetr		$r2, IWB
	sprsetr		$r2, DWB

	/* Prepare early DPAGE and IPAGE */
	// Set way 3 as large page and others small
	rseti		$r1, 0x8
	sprsetr         $r1, IPAGE
	rseti		$r2, 3
	// Enable MMU in both PSC and DPAGE
	shli		$r2, $r2, PSC_OFFSE_trans
	or		$r0, $r0, $r2  // Keep $r0 for later enabling MMU in PSC
	// Set DPAGE as: way 3 large parge, way 0-2 small page, enable DT and PROCID from PSC
	andi		$r2, $r0, ~(0xFF)  // Should we clear all lower 32 bits of PSC for DPAGE?
	or		$r1, $r2, $r1
	sprsetr         $r1, DPAGE

        // Create early page table
        cal		$t0, create_kernel_idmap
#ifndef CONFIG_GPT_BUILTIN_DTB
	// Map the FDT blob
	cal		$t0, create_kernel_fdtmap
#endif
        /* Initilize current_pgd here for early ioremap */
        ldapci          $a0, swapper_pg_dir 
        // The right place for setting CURRPGD should be in invalidate_tlb_and_switch,
        // but we do this here to save instructions translating virtual to physical
        sseta		CURRPGD, $a0
        // We are ready to going to virtual mode
	sprsetr		$r0, XSA
        tovirta		$asp, $r1
	aaddpci		$a0, virt_entry
	tovirta		$a0, $r1
	rseta		$r1, $a0
	sprsetr		$r1, XRA
	retfi_alt	
ENDPROC(start_entry)

	/* void virt_entry (unsigned int cpu_id"$r5", unsigned long psc"$r0", void (*fun)()"$t1" */
        .align 5
ENTRY(virt_entry)
        // We are in virtual mode now. We need to get the
        // kernel's exception handlers working.
        // We don't have kernel in use for now.
	rseti           $r1, 0
        sprsetr         $r1, XHS
        // Disable IRQ
        sprsetr		$r1, XEN
        // Setup exception level
        rsetli          $r1, IX_LEV1_EXCEPTIONS
        sprsetr         $r1, XLV
        rsetli          $r1, IX_MC_EXCEPTIONS
        sprsetr         $r1, XMS
	// Initialize syscall table
	ldapci          $a0, sys_call_table
        sseta		SYSTAB, $a0 // ??? Are we really ready for syscall?
        // Setup hypervisor exception handler base.
        ldapci          $a0, ixh_vectors
        rseta           $r1, $a0
        sprsetr         $r1, HXB

#if 0   // No need to take care of kernel mode
        rseti           $r15, 0
        sprsetr         $r15, KXB
#endif

        // Setup traps
        ldapci          $a0, trp_vectors
        rseta           $r1, $a0
        sprsetr         $r1, TXB
        rsetli          $r1, TRAP_MASK
        sprsetr         $r1, TRAPM
        // Setup PSCH
        sprsetr		$r0, PSCH

        // Now turn on interrupt_1 and interrupt_x
        ori		$r0, $r0, (1 << PSC_OFFSE_interrupt_1) | (1 << PSC_OFFSE_interrupt_x)
	// PROCID = 1
	rseti		$r1, 1
	shli		$r1, $r1, PSC_OFFSE_procid
	or		$r0, $r0, $r1
        sprsetr		$r0, PSC
	taddpci		$t7, fatal_error
	ldtpci		$t0, start_kernel
#ifdef CONFIG_SMP
	jci_eq		$t0, $r5, 0
	// void secondary_start_kernel (unsigned int cpu)
        addi		$r8, $r5, 0
	jmp		$t0, secondary_start_kernel
#else
	j		$t0
#endif
ENDPROC(virt_entry)

#ifdef CONFIG_SMP
ENTRY(secondary_entry)
        /* First of all setup a stack and prepare machine check vector
           so that we can capture any early exceptions. */
	/* Setup machine check handlers. */
        ldapci          $a0, ixm_vectors
        rseta           $r0, $a0
        sprsetr         $r0, MXB
	/* All exceptions go to mc. */
	rseti           $r15, -1
        sprsetr         $r15, XMS

	// Enable default CPU features
        rsetli          $r0, CPUC_DEFAULT
	sprsetr		$r0, CPUC
	// Invalid l1cache caches, secondary cpus do not need to invalid l2
        cal		$t0, l1cache_init

        /* Turn on l2sync for this core and leave others unchanged */
	/* BUG: sync_set is not implemented in real chip. */
        rseti           $r0, (1 << L2CCTLC_OFFSE_sync_mask) | (0x1 << L2CCTLC_OFFSE_sync_set)
        shl             $r0, $r0, $r5
        ori             $r0, $r0, (1 << L2CCTLC_OFFSE_sync_en)
        sprsetr         $r0, L2CCTLC

	// Initialize PSC
	// PROCID = 1 
	// Enable ic, dc and l2c
	// Disable FP, VPU and MMU
	// Disable intx, int0, int1, intmc
	// Enable memory speculative access and cache coherency
        rsetli          $r0, CACHE_MODE | (1 << PSC_OFFSE_interrupt_mc)
	sprsetr         $r0, PSC
	// This profile is good for PSCM in the whole life of kernel
	sprsetr		$r0, PSCM
        /* Set stack to (unsigned long)(&secondary_data) + THREAD_SIZE*/
	// Do not fetch this pointer before cache coherence is done.
	ldapci          $asp, secondary_data
	lda		$asp, $asp
	// NOTE: Before io remap we should not access any IO device.

	/* Enable MMU
           1. Clear TLB buffer
           2. Prepare page table (early usage)
           3. Setup page size configuration
           ??? Do we have to redirect MC handlers for TLB refilling? 
           No TLB refilling is allowed until we have a real page table.
           4. Enable MMU in PSC
         */
        // Invalidate tlb
	taddpci		$t0, 1f
        rseti		$r2, 0
	sprsetr		$r2, IWV
	sprsetr		$r2, DWV
        rseti		$r2, (1 << (IWB_WIDTH_way + IWB_WIDTH_index)) - 1
	.align 3
1:
	sprsetr		$r2, IWB
	jdecnz		$t0, $r2, $r2
	taddpci		$t0, 1f
        rseti		$r2, (1 << (DWB_WIDTH_way + DWB_WIDTH_index)) - 1
1:
	sprsetr		$r2, DWB
	jdecnz		$t0, $r2, $r2
	sprsetr		$r2, IWB
	sprsetr		$r2, DWB

	/* Initialize DPAGE and IPAGE */
	rseti		$r1, 0
	sprsetr         $r1, IPAGE
	rseti		$r2, 3
	// Enable MMU in both PSC and DPAGE
	shli		$r2, $r2, PSC_OFFSE_trans
	or		$r0, $r0, $r2  // Keep $r0 for later enabling MMU in PSC
	// Set DPAGE as: all small page, enable DT and PROCID from PSC
	andi		$r2, $r0, ~(0xFF)  // Should we clear all lower 32 bits of PSC for DPAGE?
	or		$r1, $r2, $r1
	sprsetr         $r1, DPAGE
        /* Initilize current_pgd. Unlike the first cpu, the secondary cpus do need table walking
           to refill the TLB buffer. */
        ldapci          $a0, swapper_pg_dir
        sseta		CURRPGD, $a0
        // We are ready to going to virtual mode
	sprsetr		$r0, XSA
	aaddpci		$a0, virt_entry
	tovirta		$a0, $r1
	rseta		$r1, $a0
	sprsetr		$r1, XRA
	retfi_alt	
ENDPROC(secondary_entry)
#endif
