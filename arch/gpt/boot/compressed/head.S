/*
 * linux/arch/gpt/boot/compressed/head.S
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/linkage.h>
#include <asm/asm-defs.h>
#include <linux/of_fdt.h>
#include <asm/asm-offsets.h>
#include <asm/ptrace.h>
#include <asm/cache.h>

#define ENL1CACHE ((1 << PSC_OFFSE_cache_ic) | (1 << PSC_OFFSE_cache_dc)) 

		.align	5
		.section ".start"
		.text
start:
        	rseti           $r15, 0
        	sprsetr         $r15, XLV
        	sprsetr         $r15, XHS
        	sprsetr         $r15, XMS
        	sprsetr         $r15, XEN
        	sprsetr         $r15, L2CCTLC
        	sprsetr         $r15, CPUC
        	sprsetr         $r15, PSC
        	sprsetr         $r15, PSCH
        	sprsetr         $r15, PSCK
        	sprsetr         $r15, PSCM
        	sprsetr         $r15, DPAGE
        	sprsetr         $r15, IPAGE
        	rsetli          $r14, CPUC_DEFAULT
        	or_spr          $r14, $r15, CPUC

		rsetspr		$r15, PSC
		andi		$r15, $r15, ENL1CACHE
		taddpci		$t0, cache_is_enabled
		jci_ne		$t0, $r15, 0
	        cal             $t0, init_cache
	        taddpci         $t0, enable_cache
		call            $t0

cache_is_enabled:
		ldapci	$a2, LC0
		lda	$a3, $a2
		rseta	$r0, $a2
		ldul	$r1, $a2, 8
		ldul	$r2, $a2, 8
		ldul	$r3, $a2, 8
		ldul	$r4, $a2, 8
		ldul	$r5, $a2, 8
		ldul	$r6, $a2, 8
		ldul	$r7, $a2, 8
		ldul	$r8, $a2, 8
		ldapci  $asp,  decompress_stack_end
		subf	$r0, $r0, $r1

		/*
		 * if delta is zero, we are running at the address
		 * we were linked at.
		 */
		taddpci         $t0, not_relocated
		jci_eq		$t0, $r0, 0			

		/*
		 * We're running at a different address.  We need to fix
		 * up various pointers:
		 *   r5 - zImage base address (_start)
		 *   r7 - GOT start
		 *   r8 - GOT end
		 */
		add	$r5, $r5, $r0	
		add	$r7, $r7, $r0	
		add	$r8, $r8, $r0	

		/*
		 * we need to fix up pointers into the BSS region.
		 *   r2 - BSS start
		 *   r3 - BSS end
		 *   sp - stack pointer
		 */
		add	$r2, $r2, $r0
		add	$r3, $r3, $r0
		aaddar	$asp, $asp, $r0

		/*
		 * Relocate all entries in the GOT table.
		 * This fixes up the C references.
		 *   r7 - GOT start
		 *   r8 - GOT end
		 */
		aaddri	$a2, $r7, 0
		aaddri	$a3, $r8, 0
		taddpci   $t0, 1001f
		
1001:		ldl	$r1, $a2
		add	$r1, $r1, $r0
		stul	$r1, $a2, 8
		jca_ne  $t0, $a2, $a3	

not_relocated:
		/*
		 * Clear BSS region.
		 *   r2 - BSS start
		 *   r3 - BSS end
		 */
	 	rseti           $r15, 0
		aaddri		$a2, $r2, 0
		aaddri		$a3, $r3, 0
		taddpci   	$t0, 1002f
1002:
        	stul            $r15, $a2, 8
        	jca_ne          $t0, $a2, $a3

		/*
		 * Set up some pointers, for starting decompressing.
		 */
		rseta	$r9,  $asp			# malloc space above stack
	        rsetli  $r10, 0x10000
       		add     $r10, $r9, $r10			#64K max


		/*
		 * Check to see if we will overwrite ourselves.
		 *   r4 = final kernel address
		 *   r5 = start of this image
		 *   r6 = size of decompressed image
		 *   r10 = end of malloc space (and therefore this image)
		 * We basically want:
		 *   r4 >= r10 -> OK
		 *   r4 + image length <= r5 -> OK
		 */
		
		taddpci		$t0, wont_overwrite
		rsetli		$r4, KERNEL_LOAD
		jcsw_ge		$t0, $r4, $r10
		add		$r0, $r4, $r6
		jcsw_ge		$t0, $r0, $r5

		/*
		 * If overwrite, just print error message
		 */
		
		jmp		$t0, __error_overwrite

		/*
		 * We're not in danger of overwriting ourselves.
		 * Do this the simple way.
		 */
wont_overwrite:
		/*
		 * decompress_kernel:
		 *   $r8: output_start
		 *   $r9: free_mem_ptr_p
		 *   $r10: free_mem_ptr_end_p
		 */
		rsetli		$r8, KERNEL_LOAD
		cal         	$t0, decompress_kernel
		rsetli		$r15, KERNEL_LOAD
		tsetr		$t0, $r15
        	j            	$t0
		
		.align	4
LC0:		.dword	LC0			#r1
		.dword	__bss_start		# r2
		.dword	_end			# r3
		.dword	_start			# r5
		.dword	_image_size		# r6
		.dword	_got_start		# r7
		.dword	_got_end		# r8
		.dword	decompress_stack_end	# sp
		.size	LC0, . - LC0

__error_overwrite:
		taddpci         $t0, 2001f
2001:		nop
		j		$t0

	.align	5
ENTRY(init_cache)
	ldapci          $a7, _text
__inv_dcache:
        taddpci         $t1, 1f
        aaddai          $a6, $a7, 0
        rseti           $r9, DCACHE_LINE_COUNT
1:
        dctl_invalid_l1 $a6
        aaddai          $a6, $a6, DCACHE_LINE_BYTES
        jdecnz          $t1, $r9, $r9

__inv_icache:
        taddpci         $t1, 2f
        rseta           $r8, $a7
        rseti           $r9, ICACHE_LINE_COUNT
2:
        sprsetr         $r8, ICCTL
        addi            $r8, $r8, ICACHE_LINE_BYTES
        jdecnz          $t1, $r9, $r9

        ret
ENDPROC(init_cache)
	
	.align	5
ENTRY(enable_cache)
        /* activate icache, dcache */
        rseti           $r14, ENL1CACHE
        or_spr          $r14, $r15, PSC
        or_spr          $r14, $r15, PSCM

        ret
ENDPROC(enable_cache)

		.align	4	
		.section ".stack", "aw", %nobits
decompress_stack:	.space	32768
decompress_stack_end:
