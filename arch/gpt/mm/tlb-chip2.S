#include <asm/page.h>
#include <asm/pgtable.h>
#include <asm/asm-defs.h>
#include <asm/tlb.h>

#include <machine/gpt_kernel.h>

#include <linux/linkage.h>        

/*
 * Clear boot table map and switch to small page
 */
	.align 5  
ENTRY(invalidate_tlb_and_switch)
        // Switch DPAGE and IPAGE to close big page mode.
	taddpci		$t0, 1f
	rsetspr         $r8, DPAGE
	// TODO: We should use XOR function to enhance TLB performance	
        andi		$r8, $r8, ~0xFF
	sprsetr		$r8, DPAGE
        rseti		$r9, 0
	sprsetr		$r9, IWV
	sprsetr		$r9, DWV
	// Only way 3 needs invalidation.
	rseti		$r8, 3
	// ---Cache boundary---
1:
	sprsetr		$r8, DWB
	sprsetr		$r8, IWB
	addi		$r8, $r8, 4
	// The first time will cause the ITLB miss and refill the right entry.
	jcui_le		$t0, $r8, (1 << (DWB_WIDTH_way + DWB_WIDTH_index)) - 1

#ifndef CONFIG_GPT_BUILTIN_DTB
	//way 2 needs invalidation.
	taddpci		$t0, 1f
	rseti		$r8, 2 
1:
	sprsetr		$r8, DWB
	sprsetr		$r8, IWB
	addi		$r8, $r8, 4
	// The first time will cause the ITLB miss and refill the right entry.
	jcui_le		$t0, $r8, (1 << (DWB_WIDTH_way + DWB_WIDTH_index)) - 1
#endif

	// Put IPAGE change at last in case of ITLB access during previous code
	sprsetr		$r9, IPAGE

        ret
ENDPROC(invalidate_tlb_and_switch)

kernel_image_start:	.dword	_s_kernel_ro
kernel_image_end:	.dword	__init_end

ENTRY(create_kernel_idmap)
	taddpci		$t0, 1f
	// Map _s_kernel_ro to __init_end in ITLB and DTLB in BIG page mode
	aaddpci		$a7, kernel_image_end
	ldl		$r9, $a7
	aaddpci		$a7, kernel_image_start
	ldl		$r8, $a7
	rounddown	$r8, GPT_PAGE_BIGREG_ln
	roundup		$r9, GPT_PAGE_BIGREG_ln
	rsetli		$r14, 1 << GPT_PAGE_BIGREG_ln
	shli		$r10, $r14, (BOOT_MEMORY_LIMIT_SIZE_ln - GPT_PAGE_BIGREG_ln)
	add		$r6, $r8, $r10

        extrui          $r10, $r8, 0, DWV_OFFSE_BIG_ra + DWV_WIDTH_BIG_ra
        rseti           $r11, _PAGE_KERNEL
        or              $r11, $r10, $r11
#if DWV_OFFSE_BIG_ra != IWV_OFFSE_BIG_ra || DWV_WIDTH_BIG_ra != IWV_WIDTH_BIG_ra
        extrui          $r10, $r8, 0, IWV_OFFSE_BIG_ra + IWV_WIDTH_BIG_ra
#endif
        rseti           $r12, _PAGE_KERNEL_RO
        or              $r10, $r10, $r12
	// Extract the index + 2 more bits for adding way later.
	extrui          $r12, $r8, KERNEL_PAGE_BIGREG_ln - 2, DWB_WIDTH_index + 2
	ori		$r12, $r12, 3
#if DWB_WIDTH_index != IWB_WIDTH_index
	extrui          $r13, $r8, KERNEL_PAGE_BIGREG_ln - 2, IWB_WIDTH_index + 2
	ori		$r13, $r13, 3
#endif
	// In BIG page mode the virtual address 63:29 bits are used which covers
	// 256MB range per tag (Enough for BOOT_MEMORY_LIMIT_SIZE), so we set it once for all.
	sprsetr		$r8, IWT
        sprsetr         $r8, DWT
1:
	sprsetr		$r10, IWV
#if DWB_WIDTH_index != IWB_WIDTH_index
	sprsetr		$r13, IWB
	addi		$r13, $r13, 4
#else
	sprsetr		$r12, IWB
#endif
	add		$r10, $r10, $r14
2:
	sprsetr		$r11, DWV
        sprsetr		$r12, DWB
	addi		$r12, $r12, 4
	add		$r8, $r8, $r14
	add		$r11, $r11, $r14
	jc_ne		$t0, $r8, $r9
	// Done for the kernel executable zone. Continue on to BOOT_MEMORY_LIMIT_SIZE
	// for DTLB only.
	taddpci		$t0, 2b
	addi		$r9, $r6, 0
	jc_ne		$t0, $r8, $r9
#if 0
/*map page for mpic sgi*/
	rsetli		$r8, 0xf0007400 & ~(PAGE_SIZE - 1)
        extrui          $r10, $r8, 0, DWV_OFFSE_BIG_ra + DWV_WIDTH_BIG_ra
	rseti		$r9, _PAGE_DEVICE
        or              $r10, $r10, $r9
	sprsetr		$r10, DWV
	rsetli		$r9, 0xffffff00f0000000
	sprsetr		$r9, DWT
	extrui          $r8, $r8, KERNEL_PAGE_SIZE_ln - 2, DWB_WIDTH_index + 2
	ori		$r8, $r8, 2
	sprsetr		$r8, DWB
#endif
	ret
ENDPROC(create_kernel_idmap)

ENTRY(create_kernel_fdtmap)
#define FDT_MEMORY_LIMIT_SIZE_ln  18
	taddpci		$t0, 1f
	/* Map from __fdt_pointer to __fdt_end in DTLB in Small page mode */
	rseti           $r15, -1
	shli		$r15, $r15, PAGE_OFFSET_sft
	ldapci		$a7, __fdt_end
	ldl		$r9, $a7
 	add		$r9, $r9, $r15		
	ldapci		$a7, __fdt_pointer
	ldl		$r8, $a7
 	add		$r8, $r8, $r15		
	rounddown	$r8, GPT_PAGE_SIZE_ln
	roundup		$r9, GPT_PAGE_SIZE_ln
	rsetli		$r14, 1 << GPT_PAGE_SIZE_ln
	shli		$r10, $r14, (FDT_MEMORY_LIMIT_SIZE_ln  - GPT_PAGE_SIZE_ln)
	add		$r6, $r8, $r10

        extrui          $r10, $r8, 0, DWV_OFFSE_BIG_ra + DWV_WIDTH_BIG_ra
        rseti           $r11, _PAGE_KERNEL
        or              $r11, $r10, $r11
#if DWV_OFFSE_BIG_ra != IWV_OFFSE_BIG_ra || DWV_WIDTH_BIG_ra != IWV_WIDTH_BIG_ra
        extrui          $r10, $r8, 0, IWV_OFFSE_BIG_ra + IWV_WIDTH_BIG_ra
#endif
        rseti           $r12, _PAGE_KERNEL_RO
        or              $r10, $r10, $r12
	// Extract the index + 2 more bits for adding way later.
	extrui          $r12, $r8, KERNEL_PAGE_SIZE_ln - 2, DWB_WIDTH_index + 2
	ori		$r12, $r12, 2 
#if DWB_WIDTH_index != IWB_WIDTH_index
	extrui          $r13, $r8, KERNEL_PAGE_SIZE_ln - 2, IWB_WIDTH_index + 2
	ori		$r13, $r13, 2 
#endif
	// In Small page mode the virtual address 63:19 bits are used which covers
	// 256KB range per tag (Enough for FDT_MEMORY_LIMIT_SIZE), so we set it once for all.
	sprsetr		$r8, IWT
        sprsetr         $r8, DWT
1:
	sprsetr		$r10, IWV
#if DWB_WIDTH_index != IWB_WIDTH_index
	sprsetr		$r13, IWB
	addi		$r13, $r13, 4
#else
	sprsetr		$r12, IWB
#endif
	add		$r10, $r10, $r14
2:
	sprsetr		$r11, DWV
        sprsetr		$r12, DWB
	addi		$r12, $r12, 4
	add		$r8, $r8, $r14
	add		$r11, $r11, $r14
	jc_ne		$t0, $r8, $r9
	// Done for the kernel executable zone. Continue on to FDT_MEMORY_LIMIT_SIZE
	// for DTLB only.
	taddpci		$t0, 2b
	addi		$r9, $r6, 0
	jc_ne		$t0, $r8, $r9
	ret
ENDPROC(create_kernel_fdtmap)
