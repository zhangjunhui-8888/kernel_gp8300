/*
 * arch/gpt/kernel/entry-vfp.S
 *
 * VFP state saving and restoring.
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
#include <asm/asm-defs.h>
#include <asm/asm-offsets.h>

.macro fpu_save
	aaddri          $a5, $r8, VFP_f0
	
	stuf		$f0, $a5, 8
	stuf		$f1, $a5, 8
	stuf		$f2, $a5, 8
	stuf		$f3, $a5, 8
	stuf		$f4, $a5, 8
	stuf		$f5, $a5, 8
	stuf		$f6, $a5, 8
	stuf		$f7, $a5, 8
	stuf		$f8, $a5, 8
	stuf		$f9, $a5, 8
	stuf		$f10, $a5, 8
	stuf		$f11, $a5, 8
	stuf		$f12, $a5, 8
	stuf		$f13, $a5, 8
	stuf		$f14, $a5, 8
	stuf		$f15, $a5, 8	
	
	rsetfs		$r15
	stul		$r15, $a5, 8
	rsetfc		$r15		
	stl		$r15, $a5
.endm

.macro fpu_restore
	aaddri          $a5, $r8, VFP_f0
	
	lduf		$f0, $a5, 8
	lduf		$f1, $a5, 8
	lduf		$f2, $a5, 8
	lduf		$f3, $a5, 8
	lduf		$f4, $a5, 8
	lduf		$f5, $a5, 8					
	lduf		$f6, $a5, 8
	lduf		$f7, $a5, 8
	lduf		$f8, $a5, 8
	lduf		$f9, $a5, 8
	lduf		$f10, $a5, 8
	lduf		$f11, $a5, 8					
	lduf		$f12, $a5, 8
	lduf		$f13, $a5, 8
	lduf		$f14, $a5, 8
	lduf		$f15, $a5, 8
	
	ldul		$r15, $a5, 8
	fssetr		$r15
	ldl		$r15, $a5
	fcsetr		$r15		
.endm

.macro vpu_save
	aaddri          $a5, $r8, VFP_n0
	rsetn		$r15, $n0
	stuh		$r15, $a5, 2
	rsetn		$r15, $n1	
	stuh		$r15, $a5, 2	
	rsetn		$r15, $n2	
	stuh		$r15, $a5, 2	
	rsetn		$r15, $n3	
	stuh		$r15, $a5, 2	
	rsetn		$r15, $n4	
	stuh		$r15, $a5, 2	
	rsetn		$r15, $n5	
	stuh		$r15, $a5, 2	
	rsetn		$r15, $n6	
	stuh		$r15, $a5, 2	
	rsetn		$r15, $n7	
	stuh		$r15, $a5, 2	
	
	rseti		$r15, 1024/4
	nsetr		$n0, $r15
	
	rsetli		$r12, VFP_v0
	add		$r12, $r12, $r8	
	aaddri		$a6, $r12, 0
	
	stuv_w		$v0, $a6, $n0
	stuv_w		$v1, $a6, $n0
	stuv_w		$v2, $a6, $n0		
	stuv_w		$v3, $a6, $n0 		
	stuv_w		$v4, $a6, $n0		
	stuv_w		$v5, $a6, $n0
	stuv_w		$v6, $a6, $n0
	stuv_w		$v7, $a6, $n0

	rseti		$r12, 0
	rseti		$r13, 1
	rseti		$r14, 512
	nsetr		$n0, $r14
	rseti		$r14, 256
	nsetr		$n1, $r14

	vbrd_h		$n0, $v6, $r12
	vbrd_h		$n0, $v7, $r13

	aaddri		$a7, $r8, VFP_m0
	vsel_h		$n0, $v0, $v6, $v7,$m0
	vsel_h		$n0, $v1, $v6, $v7,$m1
	vshl_h		$n0, $v0, $v0, $v7
	vor_h		$n0, $v0, $v0, $v1

	vsel_h		$n0, $v1, $v6, $v7,$m2
	vshl_h		$n0, $v0, $v0, $v7
	vor_h		$n0, $v0, $v0, $v1

	vsel_h		$n0, $v1, $v6, $v7,$m3
	vshl_h		$n0, $v0, $v0, $v7
	vor_h		$n0, $v0, $v0, $v1

	rseti		$r15, 4
	vbrd_h		$n1, $v5, $r15
	vtail_h		$n1, $v2, $v0, $n1
	vshl_h		$n1, $v2, $v2, $v5
	vor_h		$n1, $v0, $v0, $v2

	vpack_h_lower	$n1, $v0, $v0
	stv_b		$v0, $a7, $n1

	rsetvfs		$r15
	stul		$r15, $a6, 8				
	rsetvfc		$r15
	stl		$r15, $a6				
.endm

.macro vpu_restore
	rseti		$r13, 0
	rseti		$r14, 512
	nsetr		$n0, $r14
	rseti		$r14, 256
	nsetr		$n1, $r14
	vbrd_h		$n0, $v6, $r13	
	
	rseti		$r14, 4
	vbrd_h		$n1, $v5, $r14
	aaddri		$a7, $r8, VFP_m0	
	ldv_b		$v2, $a7, $n1
	vunpack_b_upper	$n1, $v2, $v2
	vshl_h		$n1, $v1, $v2, $v5
	vconcat_h	$n0, $v0, $v2, $v1, $n1 

	rseti		$r14, 1
	vbrd_h		$n0, $v5, $r14
	vcmps_h_lt	$n0, $m0,$v0,$v6
	vshl_h		$n0, $v0, $v0, $v5
	vcmps_h_lt	$n0, $m1,$v0,$v6
	vshl_h		$n0, $v0, $v0, $v5
	vcmps_h_lt	$n0, $m2,$v0,$v6
	vshl_h		$n0, $v0, $v0, $v5
	vcmps_h_lt	$n0, $m3,$v0,$v6

	rseti		$r15, 1024/4
	nsetr		$n0, $r15
	
	rsetli		$r12, VFP_v0
	add		$r12, $r12, $r8	
	aaddri		$a6, $r12, 0
					
	lduv_w		$v0, $a6, $n0
	lduv_w		$v1, $a6, $n0	
	lduv_w		$v2, $a6, $n0		
	lduv_w		$v3, $a6, $n0		
	lduv_w		$v4, $a6, $n0		
	lduv_w		$v5, $a6, $n0			
	lduv_w		$v6, $a6, $n0
	lduv_w		$v7, $a6, $n0						
	
	aaddri		$a5, $r8, VFP_n0
	lduh		$r15, $a5, 2
	nsetr		$n0, $r15
	lduh		$r15, $a5, 2
	nsetr		$n1, $r15
	lduh		$r15, $a5, 2
	nsetr		$n2, $r15
	lduh		$r15, $a5, 2
	nsetr		$n3, $r15
	lduh		$r15, $a5, 2
	nsetr		$n4, $r15
	lduh		$r15, $a5, 2
	nsetr		$n5, $r15
	lduh		$r15, $a5, 2
	nsetr		$n6, $r15
	lduh		$r15, $a5, 2
	nsetr		$n7, $r15
				
	ldul		$r15, $a6, 8				
	vfssetr		$r15
	ldl		$r15, $a6
	vfcsetr		$r15
.endm


/*
 * Save the FPU registers.
 *
 * r8 - pointer to struct vfp_state
 */
ENTRY(fpu_save_state)
	fpu_save 
	ret
ENDPROC(fpu_save_state)

/*
 * Load the FPU registers.
 *
 * r8 - pointer to struct vfp_state
 */
ENTRY(fpu_load_state)
	fpu_restore 
	ret
ENDPROC(fpu_load_state)

/*
 * Save the VPU registers.
 *
 * r8 - pointer to struct vfp_state
 */
ENTRY(vpu_save_state)
	vpu_save 
	ret
ENDPROC(vpu_save_state)

/*
 * Load the VPU registers.
 *
 * r8 - pointer to struct vfp_state
 */
ENTRY(vpu_load_state)
	vpu_restore 
	ret
ENDPROC(vpu_load_state)

/*
 * Get the FPU FCR register.
 *
 * r8 - return value
 */
ENTRY(fpu_get_fcr)
	rsetfc		$r8 
	ret
ENDPROC(fpu_get_fcr)

/*
 * Set the FPU FCR register.
 *
 * r8 - argument
 */
ENTRY(fpu_set_fcr)
	fcsetr		$r8 
	ret
ENDPROC(fpu_set_fcr)

/*
 * Get the FPU FSR register.
 *
 * r8 - return value
 */
ENTRY(fpu_get_fsr)
	rsetfs		$r8 
	ret
ENDPROC(fpu_get_fsr)


/*
 * Get the VPU FCR register.
 *
 * r8 - return value
 */
ENTRY(vpu_get_fcr)
	rsetvfc		$r8 
	ret
ENDPROC(vpu_get_fcr)

/*
 * Set the VPU FCR register.
 *
 * r8 - argument
 */
ENTRY(vpu_set_fcr)
	vfcsetr		$r8 
	ret
ENDPROC(vpu_set_fcr)


/*
 * Get the VPU FSR register.
 *
 * r8 - return value
 */
ENTRY(vpu_get_fsr)
	rsetvfs		$r8 
	ret
ENDPROC(vpu_get_fsr)
