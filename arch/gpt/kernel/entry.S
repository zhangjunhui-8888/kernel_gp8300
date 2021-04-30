/*
 * arch/gpt/kernel/entry.S
 *
 * GPT System calls, fault low-level handling, etc. entry points.
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
#include <linux/of_fdt.h>
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

#ifndef CONFIG_PREEMPT
#define resume_kernel	restore_all
#endif

	.macro mc2l1_jump rpsc, rr, tr, lb
        rsetspr         \rpsc, PSCH
	rsetspr		\rr, XSM
	extrui		\rr, \rr, PSC_OFFSE_procid, PSC_WIDTH_procid
	shli		\rr, \rr, PSC_OFFSE_procid
	// PROCID is always 0 in PSCH
	or		\rpsc, \rpsc, \rr
	sprsetr         \rpsc, XSM
	taddpci         \tr, \lb
	rsett           \rpsc, \tr
	tovirt          \rr, \rpsc
	sprsetr         \rr, XRM
	retfi_mc
	.endm

	.macro	get_thread_info ra, rr, offset
	rseta		\rr, $asp
	shri		\rr, \rr, PAGE_SHIFT + THREAD_SIZE_ORDER
	shli		\rr, \rr, PAGE_SHIFT + THREAD_SIZE_ORDER
	aaddri		\ra, \rr, \offset
	.endm

	.macro get_current_procid rr, where
        rsetspr         \rr, \where
        extrui          \rr, \rr, PSC_OFFSE_procid, PSC_WIDTH_procid
	.endm
	
	.macro	save_tregs POST_OFFSET:req
	stut		$t7, $asp, -PT_t7 + PT_t4
	stut		$t4, $asp, -PT_t4 + PT_t1
	stut		$t1, $asp, -8
	stut		$t0, $asp, \POST_OFFSET
	.endm

	.macro	restore_tregs POST_OFFSET:req
	ldut		$t0, $asp, 8
	ldut		$t1, $asp, -PT_t1 + PT_t4	
	ldut		$t4, $asp, -PT_t4 + PT_t7
	ldut		$t7, $asp, \POST_OFFSET
	.endm

	.macro kernel_entry lv, type /*type: 0: general exception. 1: syscall. 2: interrupt. 3: Data access exception. 4: Wake up. */
	/* Construct pt_gregs frame */
	aaddai		$asp, $asp, -(PT_regs_size - PT_t7)

	save_tregs	-PT_t0 + PT_a10
	
	taddpci		$t0, save_general_regs
	call		$t0
	
	.if \type == 1  // Syscall requires to save r7 and r8 in another place
	aaddai		$a7, $asp, PT_syscallno
	stul		$r7, $a7, -PT_syscallno + PT_r8_orig
	stul		$r8, $a7, -PT_r8_orig + PT_xen
	.else
	aaddai		$a7, $asp, PT_xen
	  .if \lv == LEVEL_MC
	tovirta		$asp, $r14	// set $asp to virtual address
	  .endif
	.endif

	rsetspr		$r14, XEN
	stul		$r14, $a7, -8   /* save xen */
	get_ixi		$r15, \lv
	stul		$r15, $a7, -8   /* save ixi */
	get_ca		$r15
	stul		$r15, $a7, -8	/* save ca */
	get_xs		$r5, \lv
	get_xr		$r15, \lv	
	.if \type == 1			/* In SWI exception, the PC = xr+4 */
	stul		$r5, $a7, -8	/* save sr */
	addi		$r15, $r15, 4
	stul		$r15, $a7, -8	/* save pc */
	sta		$a0, $a7	/* save sp */
	.elseif \type == 4	/*wake up from idle*/
	taddpci		$t7, resume_kernel
	stul		$r5, $a7, -8	/* save sr */
	addi		$r15, $r15, 4 // Skip idle
	stul		$r15, $a7, -8	/* save pc */
	aaddai		$a0, $asp, PT_regs_size // Previous kernel sp
	rseti		$r14, 0
	sta		$a0, $a7	/* save sp */
	// We saved all what we need to save. Enable the interrupt if allowed.
	// TODO: Exception should allow interrupt?
	sprsetr		$r14, XEN
	.else		/*interrupt(type=2) and data access exception(type=3)*/
	taddpci		$t0, 1f
	taddpci		$t7, resume_userspace
	extrui          $r10, $r5, PSC_OFFSE_mem_priv_user, PSC_WIDTH_mem_priv_user
	taddtr_l	$t0, $t0, $r10
	  .if \type == 3
	extrui          $r11, $r5, PSC_OFFSE_all_priv, PSC_WIDTH_all_priv
	// trap: 0b0001: priv_user = 0, mem_priv_user = 1
	// user: 0b0101: priv_user = 1, mem_priv_user = 1
	// kernel: 0b0000: priv_user = 0, mem_priv_user = 0
	jci_ne		$t0, $r11, 1
	get_xs		$r5, LEVEL_TRAP
	get_xr		$r15, LEVEL_TRAP
	addi		$r15, $r15, -4 /* Backward to the trap instruction */
	rseti		$r14, 0
	sprsetr		$r14, XEN
	  .endif
	j		$t0
1:
	// TODO: Exception should also allow preemption? ARM64 does not allow it.
	    .if \type == 2
	taddpci		$t7, resume_kernel
	    .else
	taddpci		$t7, restore_all
	  .endif
	// We came from kernel, so user asp should be saved already.
	aaddai		$a0, $asp, PT_regs_size // Previous kernel sp
	// Condition for allowing irq enabling
	// 1. swi
	// 2. not an interrupt and
	//   a. from user space
	//   b. from kernel space and interrupt was enable before exception
	stul		$r5, $a7, -8	/* save sr */
	stul		$r15, $a7, -8	/* save pc */
	sta		$a0, $a7	/* save sp */
	// We saved all what we need to save. Enable the interrupt if allowed.
	// TODO: Exception should allow interrupt?
	rseti		$r14, 0
	sprsetr		$r14, XEN
	
	.endif
	get_thread_info $a8, $r15, TI_FLAGS
	.endm
		
	.macro kernel_exit lv
	disable_exception  $r5
	/* The procid maybe changed, update it here */
	rseti		$r13, 0xff
	shli		$r13, $r13, PSC_OFFSE_procid
	rsetspr		$r14, PSC
	and		$r14, $r14, $r13
	taddpci		$t0, restore_general_regs
	aaddai		$a7, $asp, PT_xen
	ldul		$r12, $a7, -16	/* restore xen */
	sprsetr		$r12, XEN
	ldul		$r12, $a7, -8	/* restore ca */
	set_ca		$r12
	ldul		$r15, $a7, -8	/* restore sr */
	.if \lv == 0 /* Back to user */
	ldul		$r12, $a7, -8	/* restore pc */
	.else
	ldl		$r12, $a7	/* restore pc */
	.endif
	cand		$r15, $r13, $r15
	or		$r15, $r15, $r14
	set_xs		$r15, LEVEL_1H
	set_xr		$r12, LEVEL_1H
	.if \lv == 0 /* Back to user */
	lda		$a0, $a7	/* restore sp */
	.endif
	call		$t0
		
	restore_tregs	PT_regs_size - PT_t7
	.endm


	.macro mc_tlb_entry
	ssetr		MCS0, $r8
	ssetr		MCS1, $r9
	ssetr		MCS2, $r14
	ssetr		MCS3, $r15
	rsett		$r14, $t1
	.endm

	.macro mc_tlb_exit
	tsetr		$t1, $r14
	rsets		$r15, MCS3
	rsets		$r14, MCS2
	rsets		$r9,  MCS1
	rsets		$r8,  MCS0
	.endm

	.macro do_mc_default
	taddpci		$t1, 1f
1:
	j		$t1	
	retfi_mc
	.endm

	.macro do_mc_wakeup
	sprsetr		$r7, XEN
	get_xr		$r15, LEVEL_MC
	addi		$r15, $r15, 4
	set_xr		$r15, LEVEL_MC
	.endm

        .macro mc_entry from, to, handler
	.org ixm_vectors + \from * 256
ENTRY(mc_entry_\handler\@)
	.if	\from == IX_SGI_0
	ssetr		MCS0, $r15
	\handler
	rsets		$r15, MCS0
	retfi_mc
	.else
	mc_tlb_entry
		.if	(\from==IX_INSN_TLB || \from == IX_DATA_TLB)
			taddpci         $t1, \handler
        		j               $t1
		.else
			\handler
		.endif	

	.endif
ENDPROC(mc_entry_\handler\@)

        .if             \to - \from
        mc_entry    "(\from+1)", \to, \handler
        .endif
        .endm

/*
static inline pte_t get_ptep(pgd_t *pgd, unsigned long addr)
{
	pte_t ptep;
	if (pgd && !pgd_none(*pgd)) {
	        pud_t *pud = pud_offset(pgd, addr);
	        if (pud && !pud_none(*pud)) {
	                pmd_t *pmd = pmd_offset(pud, addr);
	                if (pmd && !pmd_none(*pmd))
				ptep = *pte_offset_map(pmd, addr);
				if (pte_none(&ptep))
					goto lb;
			else
			  goto lb;
	        } else
		goto lb;
	} else
		goto lb;
}
*/
	PGDIR_ln = PGDIR_SHIFT
	PUD_ln = PUD_SHIFT
	PMD_ln = PMD_SHIFT
	.macro get_ptep_one_stage P, pgd, va, rr, tr
        extrui          \rr, \va, \P\()_ln, PTRS_PER_PTD_SHIFT
        aaddar          \pgd, \pgd, \rr
        lda             \pgd, \pgd
        jcaz_eq		\tr, \pgd
	.endm

	.macro get_ptep pgd, va, rr, tr, lb
	taddpci		\tr, \lb
       	get_ptep_one_stage PGDIR, \pgd, \va, \rr, \tr	/* PGD Table entry */
#ifndef PUD_FOLDED
	get_ptep_one_stage PUD, \pgd, \va, \rr, \tr	/* PUD Table entry */
#endif
#ifndef PMD_FOLDED
       	get_ptep_one_stage PMD, \pgd, \va, \rr, \tr	/* PMD Table entry */
#endif
        /* Offset to PTE table */
        extrui          \rr, \va, PAGE_SHIFT, PTRS_PER_PTD_SHIFT
        aaddar          \pgd, \pgd, \rr
	ldl		\rr, \pgd		/* get pte.val */
	jci_eq		\tr, \rr, 0
	.endm

	.macro get_ptep_unsafe_one_stage P, pgd, va, rr, tr
        extrui          \rr, \va, \P\()_ln, PTRS_PER_PTD_SHIFT
        aaddar          \pgd, \pgd, \rr
        lda             \pgd, \pgd
	.endm

	.macro get_ptep_unsafe pgd, va, rr, tr, lb
	taddpci		\tr, \lb
       	get_ptep_one_stage PGDIR, \pgd, \va, \rr, \tr	/* PGD Table entry */
#ifndef PUD_FOLDED
	get_ptep_one_stage PUD, \pgd, \va, \rr, \tr	/* PUD Table entry */
#endif
#ifndef PMD_FOLDED
       	get_ptep_one_stage PMD, \pgd, \va, \rr, \tr	/* PMD Table entry */
#endif
        /* Offset to PTE table */
        extrui          \rr, \va, PAGE_SHIFT, PTRS_PER_PTD_SHIFT
        aaddar          \pgd, \pgd, \rr
	ldl		\rr, \pgd		/* get pte.val */
	jci_eq		\tr, \rr, 0
	.endm

	Ira = IWV_OFFSE_ra + IWV_WIDTH_ra
	Ioffse_prot_priv_x = IWV_OFFSE_prot_priv_x
	Iwidth_prot = IWV_WIDTH_prot
	Ioffse_prot_hyper = IWT_OFFSE_prot_hyper
	Ioffse_va = IWT_OFFSE_va
	Iwidth_va = IWT_WIDTH_va
	Iwidth_index = IWB_WIDTH_index
	Ioffse_index = IWB_OFFSE_index
	Dra = DWV_OFFSE_ra + DWV_WIDTH_ra
	Doffse_prot_priv_x = DWV_OFFSE_prot_priv_x
	Dwidth_prot = DWV_WIDTH_prot
	Doffse_prot_hyper = DWT_OFFSE_prot_hyper
	Doffse_va = DWT_OFFSE_va
	Dwidth_va = DWT_WIDTH_va
	Dwidth_index = DWB_WIDTH_index
	Doffse_index = DWB_OFFSE_index

	.macro do_tlb_miss T:req
        /* Base of PGD table */
	rseta		$r15, $a7		/* save $a7 */
	asets		$a7, CURRPGD		/* get pgd address */

	.if \T == I
        get_xr      	$r9, LEVEL_MC	        /* get fault address */
	.else
        get_ixa      	$r9, LEVEL_MC	        /* get fault address */
	.endif
#if 1
	extrui		$r8, $r9, PGDIR_SHIFT, 12
	taddpci		$t1, 1f
	jci_eq		$t1, $r8, 0
	ldapci		$a7, swapper_pg_dir
1:
#endif
	.if \T == I
	get_ptep	$a7, $r9, $r8, $t1, itlb_page_fault_mc
	.else
	get_ptep	$a7, $r9, $r8, $t1, dtlb_page_fault_mc
	.endif
	aaddri		$a7, $r15, 0		/* restore $a7 */

        /* Load WV with ra and page settings from PTE entry */
	extrui          $r8, $r8, 0, \T\()ra
        sprsetr         $r8, \T\()WV

	taddpci		$t1, 1f

        /* Build Hypervisor's permissions value */
        extrui          $r8, $r8, \T\()offse_prot_priv_x, \T\()width_prot / 2
	xori		$r8, $r8, (1 << (\T\()width_prot / 2)) - 1
        shli            $r8, $r8, \T\()offse_prot_hyper
        /* Build VA value */
	extrui		$r15, $r9, \T\()offse_va, \T\()width_va
	shli		$r15, $r15, \T\()offse_va
	or		$r15, $r15, $r8
	/* Update PROCID if it's a user page, 
	 * otherwise PROCID=0 for kernel page,
	 * it means a wildcard entry which ignore process's ASID.
	 */
	extrui          $r8, $r9, 40, 24 // Get top 24 bits of the virtual address
	jcui_ge         $t1, $r8, 6  // Space above 0x00000600 00000000 is kernel space.
	get_current_procid $r8, XSM
	or		$r15, $r15, $r8
1:
        sprsetr         $r15, \T\()WT

        /* Build index/way and load WB */
        extrui          $r15, $r9, GPT_PAGE_SIZE_ln, \T\()width_index
        shli            $r15, $r15, \T\()offse_index
        tlb_get_base    $r15, $r9, $r8, $t1, \T\()TLB
        sprsetr         $r15, \T\()WB
	.if \T == I
	taddpci		$t1, itlb_page_found
	/* Normally kernel only invalidats the ITLB when MC tlb miss could interrupt it.*/
	rseti		$r15, 0
	sprsetr		$r15, IWV
	.else
	taddpci		$t1, dtlb_page_found
	.endif
	j		$t1
	.endm

ENTRY(do_itlb_miss)
	do_tlb_miss I
ENDPROC(do_itlb_miss) 

ENTRY(do_dtlb_miss)
	do_tlb_miss D
ENDPROC(do_dtlb_miss) 



/* Place all macros before real codes, so we can precisely arrange the layout. */
	// The first entire page is for machine check vectors.	
        .align KERNEL_PAGE_SIZE_ln
ENTRY(ixm_vectors)
        mc_entry IX_MACHINE_CHECK, IX_INSN_ALIGN, do_mc_default
        mc_entry IX_INSN_TLB, IX_INSN_TLB, do_itlb_miss
        mc_entry IX_INSN_NOEXEC, IX_DATA_ALIGN, do_mc_default
        mc_entry IX_DATA_TLB, IX_DATA_TLB, do_dtlb_miss
        mc_entry IX_DATA_NOREAD, IX_unused_1, do_mc_default
        mc_entry IX_SGI_0, IX_SGI_0, do_mc_wakeup
	mc_entry IX_SGI_1, /*IX_MPINT_IRQ, do_mc_wakeup
        mc_entry IX_BURST_DMA0,*/ (IX_XPEN_MAX - 1), do_mc_default
ENDPROC(ixm_vectors)

	.macro hyper_do_default
	jmp		$t1, do_hyper_exception
	.endm

	.macro hyper_do_data_align
	jmp		$t1, do_alignment
	.endm

	.macro hyper_do_irq
#ifdef CONFIG_TRACE_IRQFLAGS
	taddpci		$t1, 1f
#endif
	ldapci		$a7, handle_arch_irq
	aaddar_l	$a7, $a7, $r9
	ldt		$t0, $a7
#ifdef CONFIG_TRACE_IRQFLAGS
	taddpci		$t7, L.trace_hardirqs_on
	jci_ne		$t1, $r5, 0
	taddpci		$t7, resume_kernel
1:	
#endif
	j		$t0

#ifdef CONFIG_TRACE_IRQFLAGS
L.trace_hardirqs_on\@:
	taddpci		$t7, resume_userspace
	jmp		$t0, trace_hardirqs_on
#endif	
	.endm

	.macro hyper_do_swi
/*
 * Since swi is used to do software breakpoint by gdb,
 * we have to check the argument filed here,
 * 0      - syscall
 * 999    - gdb
 * others - reserved
 */
	taddpci		$t0, do_syscall
	jci_eq		$t0, $r5, 0

	taddpci		$t0, do_brk
	jci_eq		$t0, $r5, 999
/*
 * TODO: add other swi handler here just like this
 *	taddpci		$t0, do_xxx
 *	jci_eq		$t0, $r5, $swi_id
 */

/*
 * 	Handle an undefined swi exception
 */
	rseta		$r8, $asp
	rseti		$r9, IX_SWI_0
	rseti		$r10, 1
	hyper_do_default
	.endm

/*
 * Please note that the size of the entry should be < 256 bytes.
 */
        .macro ix_entry from, to, handler
	.org ixh_vectors + \from * 256
ENTRY(ix_entry_\handler\@)
	// All entries invoke corresponding \handler macros and the 
	// return address for $t7 is determined in kernel_entry.
	.if \from == IX_SWI_0
	kernel_entry	LEVEL_1H, 1
	get_ixi		$r5, LEVEL_1H
	extrui		$r5, $r5, 0, 12
	.elseif (\from >= IX_TIMER_BASE && \from <= IX_TIMER_COUNT1) || \
                (\from >= IX_SGI_1 && \from <= IX_BURST_DMA_7)
	kernel_entry	LEVEL_1H, 2
#ifdef CONFIG_TRACE_IRQFLAGS
	addi		$r5, $r10, 0
        cal             $t0, trace_hardirqs_off
#endif
	rseta		$r8, $asp
	rseti		$r9, \from	
	.elseif \from == IX_DATA_ALIGN
	kernel_entry	LEVEL_1H, 3
	rseta		$r8, $asp
	get_ixi		$r9, LEVEL_1H	
	.elseif \from == IX_DATA_NOREAD || \from == IX_DATA_NOWRITE
	kernel_entry	LEVEL_1H, 3
	rseta		$r8, $asp
	rseti		$r9, \from
	.else
	kernel_entry	LEVEL_1H, 0
	rseta		$r8, $asp
	rseti		$r9, \from	
	.endif

	enable_exception	$r15
	\handler
ENDPROC(ix_entry_\handler\@)

        .if             \to - \from
        ix_entry    "(\from+1)", \to, \handler
        .endif
        .endm

        .macro ix_tlb_entry from, to, handler
	.org ixh_vectors + \from * 256
ENTRY(ix_entry_\handler\@)
	
	enable_exception	$r15
	\handler
ENDPROC(ix_entry_\handler\@)

        .if             \to - \from
        ix_tlb_entry    "(\from+1)", \to, \handler
        .endif
	.endm

	// The second entire page is for hypervisor vector table
        .align KERNEL_PAGE_SIZE_ln
ENTRY(ixh_vectors)
        ix_entry IX_MACHINE_CHECK, IX_MACHINE_CHECK,  hyper_do_default
        ix_entry IX_TIMER_BASE, IX_TIMER_COUNT1,  hyper_do_irq
        ix_entry IX_UNAVAIL_VEC, IX_INSN_ALIGN, hyper_do_default
	ix_entry IX_INSN_TLB, IX_INSN_TLB, hyper_do_default //do_itlb_miss
        ix_entry IX_INSN_NOEXEC, IX_INSN_NOEXEC, hyper_do_default
        ix_entry IX_DATA_ALIGN, IX_DATA_ALIGN, hyper_do_data_align
        ix_entry IX_DATA_TLB, IX_DATA_TLB, hyper_do_default //do_dtlb_miss
        ix_entry IX_DATA_NOREAD, IX_VEC_TYPE, hyper_do_default
        ix_entry IX_SWI_0, IX_SWI_0, hyper_do_swi
        ix_entry IX_unused_0, IX_unused_1, hyper_do_default
		ix_entry IX_SGI_0, IX_GPTDMA, hyper_do_irq
        ix_entry (IX_GPTDMA + 1), (IX_XPEN_MAX - 1), hyper_do_default        
ENDPROC(ixh_vectors)

       	.macro ix_wake_up_entry from, to, handler
ENTRY(ix_wake_up_\handler\@)
	.org ixh_wake_up_vectors + \from * 256
	.if (\from >= IX_TIMER_BASE && \from <= IX_TIMER_COUNT1) || (\from >= IX_SGI_0 && \from <= IX_GPTDMA)
	kernel_entry	LEVEL_1H, 4
	taddpcil	$t4, ixh_vectors
	rsett		$r8, $t4
	sprsetr		$r8, HXB
#ifdef CONFIG_TRACE_IRQFLAGS
	rseti		$r5, 0
        cal             $t0, trace_hardirqs_off
#endif
	rseta		$r8, $asp
	rseti		$r9, \from
	enable_exception	$r15
	\handler
	.endif
	// There should not have any other options.
ENDPROC(ix_wake_up_\handler\@)
        .if             \to - \from
        ix_wake_up_entry    "(\from+1)", \to, \handler
        .endif

	.endm
	
	.macro code_segment1
ENTRY(itlb_page_fault_mc)
	aaddri		$a7, $r15, 0	/* restore $a7 */
	tophysa		$asp, $r15
	mc_tlb_exit
	kernel_entry	LEVEL_MC, 3
	/* set paramters */
	rseta		$r8, $asp
	get_xr		$r9, LEVEL_MC
	rseti		$r10, IX_INSN_TLB
                	
	mc2l1_jump      $r14, $r15, $t1, tlb_do_fault_virt
ENDPROC(itlb_page_fault_mc)
ENTRY(dtlb_page_fault_mc)
	aaddri		$a7, $r15, 0	/* restore $a7 */
	tophysa		$asp, $r15
	mc_tlb_exit
	kernel_entry	LEVEL_MC, 3
	/* set paramters */
	rseta		$r8, $asp
	get_ixa		$r9, LEVEL_MC
	rseti		$r10, IX_DATA_TLB
                	
	mc2l1_jump      $r14, $r15, $t1, tlb_do_fault_virt
ENDPROC(dtlb_page_fault_mc)

ENTRY(tlb_do_fault_virt)
	enable_exception $r15
	aaddai		$a8, $a8, 0
	aaddai		$asp, $asp, 0
	tovirtt		$t7, $r15
        jmp             $t0, do_page_fault
ENDPROC(tlb_do_fault_virt)
	.endm
	
	.macro code_segment2
	.align 8
ENTRY(itlb_page_found)
	mc_tlb_exit
	atrans		$a7, $a7 
        retfi_mc
	.align 5
ENDPROC(itlb_page_found)
ENTRY(dtlb_page_found)
	mc_tlb_exit
	refresh_shadow_regs
        retfi_mc
ENDPROC(dtlb_page_found)

	.align 5
ENTRY(save_general_regs)
	/* skip $a15-$a11, $a15 is kernel sp,
	 * $a11-$a14 are scratch in kernel.
	 */
	.irp	regno,10,9,8,7,6,5,4
	sta		$a\regno, $asp
	aaddai		$asp, $asp, -8
	.endr
	sta		$a3, $asp
	aaddai		$asp, $asp, -16
	sta		$a1, $asp
	aaddai		$asp, $asp, -PT_a1 + PT_r15
		
	.irp	regno,15,14,13,12,11,10,9,8,7,6,5,4,3,2,1
	stul		$r\regno, $asp, -8
	.endr
	stl		$r0, $asp

	ret
ENDPROC(save_general_regs)
	.align 5
ENTRY(restore_general_regs)
	.irp	regno,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14
	ldul		$r\regno, $asp, 8
	.endr
	ldul		$r15, $asp, -PT_r15 + PT_a1

	lda		$a1, $asp
	aaddai		$asp, $asp, 16
	aaddai		$a2, $a2, 0
	.irp	regno,3,4,5,6,7,8,9
	lda		$a\regno, $asp
	aaddai		$asp, $asp, 8
	.endr
	lda		$a10, $asp
	aaddai		$asp, $asp, -PT_a10 + PT_t0
	ret
ENDPROC(restore_general_regs)

	.align 5
ENTRY(ret_from_fork)
        cal		$t0, schedule_tail
        taddpci         $t1, 1f
        jci_eq          $t1, $r0, 0                // not a kernel thread
        addi		$r8, $r1, 0                // args
        tsetr           $t0, $r0                   // kfn
        call               $t0
1: 
	taddpci		$t0, resume_userspace
	rseta		$r14, $asp
	rsetli		$r15, THREAD_SIZE - 1
	cand		$r15, $r15, $r14
        aaddri          $a8, $r15, TI_FLAGS     
	j		$t0
ENDPROC(ret_from_fork)

	.align 5
ENTRY(resume_brk)
	disable_irq	$r15
	kernel_exit	0				// Back to user
	retfi_l1h
ENDPROC(resume_brk)

	.align 5
ENTRY(do_brk)
	taddpci		$t7, resume_userspace
	rseta		$r8, $asp
	jmp		$t0, brk_handler
ENDPROC(do_brk)

	.align 5
ENTRY(do_syscall)
        asets		$a3, SYSTAB

        taddpci         $t0, L.syscall_trace_enter

        /* Check _TIF_SYSCALL_TRACE in current thread */
        /* $r15 set with TI_FLAGS */
        ldl             $r15, $a8
        andi            $r15, $r15, _TIF_SYSCALL_TRACE
        jci_ne          $t0, $r15, 0
        taddpci         $t0, L.ni_sys
	taddpci		$t7, syscall_exit
        jcui_gt         $t0, $r7, __NR_syscalls - 1

        /* $r7 is syscall number */
L.syscall_call:
        aaddar          $a3, $a3, $r7
        ldt             $t0, $a3
        j               $t0

L.ni_sys:
        ldtpci          $t0, sys_ni_syscall
        j               $t0

        /* Perform syscall entry tracing */
L.syscall_trace_enter:
        /* asmlinkage long syscall_trace_enter(struct pt_regs *regs, long syscallno) */
	taddpcil	$t4, syscall_trace_enter
	taddti		$t0, $t4, syscall_trace_enter
	rseta		$r8, $asp
        call		$t0
        taddpci         $t1, L.ni_sys
        /* return value $r8 is syscall number, so check again. */
        ori             $r7, $r8, 0
        taddpci         $t7, L.syscall_trace_exit

        jcui_gt         $t1, $r8, __NR_syscalls - 1

        aaddar          $a3, $a3, $r7
        ldt             $t0, $a3

        /* restore the syscall args */
        aaddai          $a7, $asp, PT_r8_orig
        ldul            $r8, $a7, -PT_r8_orig + PT_r9
        ldul            $r9, $a7, 8
        ldul            $r10, $a7, 8
        ldul            $r11, $a7, 8
        ldul            $r12, $a7, 8
        ldl             $r13, $a7

        /* call sys_call* routine */
        j               $t0

L.syscall_trace_exit:
        /* call asmlinkage void syscall_trace_exit(struct pt_regs *regs) */
        
        taddpci		$t7, resume_userspace
	stl		$r8, $asp
        rseta           $r8, $asp
        jmp		$t0, syscall_trace_exit
ENDPROC(do_syscall)
	.endm

	// The third page is for hypervisor idle wake up vector table
        .align KERNEL_PAGE_SIZE_ln
ENTRY(ixh_wake_up_vectors)
	// We have many holes in this vectors, so place some codes in it.
	code_segment1
        ix_wake_up_entry IX_TIMER_COUNT0, IX_TIMER_COUNT0, hyper_do_irq
	code_segment2
		ix_wake_up_entry IX_SGI_0, IX_BURST_DMA_7, hyper_do_irq
		ix_wake_up_entry IX_GPTDMA, IX_GPTDMA, hyper_do_irq

ENDPROC(ixh_wake_up_vectors)
	
	.align 5
ENTRY(syscall_exit)
	disable_irq	$r15
        taddpci         $t1, save_r8_work_pending
        ldl             $r9, $a8			// $r9 = ti->flags
	extrui		$r10, $r9, TIF_NOTIFY_RESUME, 5 // _TIF_NOTIFY_RESUME | _TIF_SIGPENDING | _TIF_NEED_RESCHED | _TIF_FOREIGN_FPUSTATE| _TIF_FOREIGN_VPUSTATE
        jci_ne          $t1, $r10, 0
	aaddai		$a7, $asp, PT_r8
	stl		$r8, $a7			// save syscall return value
	kernel_exit	0				// Back to user
	retfi_l1h
ENDPROC(syscall_exit)

	.align 5
save_r8_work_pending:
	aaddai		$a7, $asp, PT_r8
	stl		$r8, $a7			// save syscall return value
work_pending:
        taddpci         $t1, work_resched
        extrui          $r14, $r9, TIF_NEED_RESCHED, 1
        taddpci         $t7, resume_userspace
        jci_ne          $t1, $r14, 0

        /* call do_notify_resume asmlinkage void do_notify_resume(struct pt_regs *regs, u32 thread_info_flags) */
	rseta           $r8, $asp
	enable_irq	$r15			// enable interrupts for do_notify_resume()
        jmp		$t0, do_notify_resume		// slow path 1
		
work_resched:						
        cal		$t0, schedule			// slow path 2
	.align 5
ENTRY(resume_userspace)
	disable_irq	$r15
        taddpci         $t1, work_pending
        ldl             $r9, $a8			// $r9 = ti->flags
	extrui		$r10, $r9, TIF_NOTIFY_RESUME, 5 // _TIF_NOTIFY_RESUME | _TIF_SIGPENDING | _TIF_NEED_RESCHED | _TIF_FOREIGN_FPUSTATE| _TIF_FOREIGN_VPUSTATE
        jci_ne          $t1, $r10, 0
no_work_pending:
	kernel_exit	0		// Back to user
	retfi_l1h
ENDPROC(resume_userspace)
			
#ifdef CONFIG_PREEMPT
	.align 5
ENTRY(resume_kernel)
	disable_irq	$r15
        taddpci         $t1, 1f
	taddpci		$t7, need_resched
	get_thread_info $a8, $r14, TI_PREEMPT
        ldusw            $r15, $a8, -TI_PREEMPT + TI_FLAGS// get preempt count
        jci_ne          $t1, $r15, 0			// preempt count != 0
need_resched:
        taddpci         $t1, 1f
        ldl             $r8, $a8
        andi            $r8, $r8, _TIF_NEED_RESCHED
        jci_eq          $t1, $r8, 0			// no need to resched
        cal             $t0, preempt_schedule_irq	
1:
#ifdef CONFIG_TRACE_IRQFLAGS
        cal             $t0, trace_hardirqs_on
#endif	
	disable_irq	$r15
	kernel_exit	1 /* back to kernel */
	retfi_l1h
ENDPROC(resume_kernel)

#endif
	.align 5
ENTRY(restore_all)
	disable_irq	$r15
	kernel_exit	1 /* back to kernel */
	retfi_l1h
ENDPROC(restore_all)

	.align 5	
ENTRY(_switch)
        /* Save stack ptr of old process */
        aaddri          $a5, $r8, TASK_THREAD_CPU_CONTEXT
        aaddri          $a6, $r9, TASK_THREAD_CPU_CONTEXT

        /* Save process registers ($r0-$r5, $a1-$a3, $t7 in pt_regs.
         * $a0 saved elsewhere. )
         */
        stul            $r0, $a5, 8             // store $r0 ~ $r5
        stul            $r1, $a5, 8
        stul            $r2, $a5, 8
        stul            $r3, $a5, 8
        stul            $r4, $a5, 8
        stul            $r5, $a5, 8
        sta             $a1, $a5                // store $a1 ~ $a3
        aaddai          $a5, $a5, 16
        sta             $a3, $a5
        aaddai          $a5, $a5, 8
        sta             $a8, $a5                // store $a8 ~ $a10
        aaddai          $a5, $a5, 8
        sta             $a9, $a5
        aaddai          $a5, $a5, 8
        sta             $a10, $a5
        aaddai          $a5, $a5, 8
        stut            $t7, $a5, 8             // store $t7
        sta             $asp, $a5               // store sp

        ldul            $r0, $a6, 8             // restore $r0 ~ $r5
        ldul            $r1, $a6, 8
        ldul            $r2, $a6, 8
        ldul            $r3, $a6, 8
        ldul            $r4, $a6, 8
        ldul            $r5, $a6, 8
        lda             $a1, $a6                // restore $a1 ~ $a3
        aaddai          $a6, $a6, 16
        lda             $a3, $a6
        aaddai          $a6, $a6, 8
        lda             $a8, $a6                // restore $a8 ~ $a10
        aaddai          $a6, $a6, 8
        lda             $a9, $a6
        aaddai          $a6, $a6, 8
        lda             $a10, $a6
        aaddai          $a6, $a6, 8
        ldut            $t7, $a6, 8             // restore $t7
        lda             $asp, $a6               // restore sp

        ret
ENDPROC(_switch)
	.align 5
ENTRY(_do_cpu_idle)
#ifdef CONFIG_GPT_SIMULATOR_ERRTA1
	taddpci		$t0, pc_idle
#endif
	taddpcil	$t4, ixh_wake_up_vectors
	rsett		$r7, $t4
	sprsetr		$r7, HXB
pc_idle:
#ifdef CONFIG_GPT_SIMULATOR_ERRTA1
	j		$t0
#else
	idle
#endif
	ret
ENDPROC(_do_cpu_idle)

ENTRY(sys_rt_sigreturn)
	rseta		$r8, $asp
        jmp             $t0, _sys_rt_sigreturn
ENDPROC(sys_rt_sigreturn)

        .section ".text","ax"
        .align 5
        .global __rt_sigreturn_start
        .global __rt_sigreturn_end
__rt_sigreturn_start:
	rseti		$r7, __NR_rt_sigreturn
	swi		0
__rt_sigreturn_end:

