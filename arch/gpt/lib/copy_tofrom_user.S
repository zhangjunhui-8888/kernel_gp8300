/*
 * Copyright (C) 2017-2018, General Processor Techologies Inc.
 * scxie <scxie@hxgpt.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
 
#include <linux/linkage.h>
#include <asm/asm-defs.h>

/*
 * unsigned long __copy_tofrom_user(void *to, const void *from, unsigned long size);
 * $r8	- to
 * $r9	- from
 * $r10	- size
 * ret	- data size NOT copied
 */
ENTRY(__copy_tofrom_user)
	taddpci		$t1, 8f
	or		$r14, $r8, $r9
	aaddri		$a5, $r8, 0 
	andi		$r14, $r14, 7		
	taddpci		$t0, 1f
	aaddri		$a6, $r9, 0 
	jci_eq		$t1, $r14, 0		// 8 bytes aligned
	taddpci		$t1, 4f	
	andi		$r14, $r14, 3		
	jci_eq		$t1, $r14, 0		// 4 bytes aligned
	j		$t0			// non-aligned
  .align 5	
8:
	jcsi_lt		$t0, $r10, 8		// the rest bytes
111:	USER(99f, ldul $r15, $a6, 8)
222:	USER(99f, stul $r15, $a5, 8)
	addi		$r10, $r10, -8
	j		$t1	
  .align 5	
4:
	jcsi_lt		$t0, $r10, 4		// the rest bytes
333:	USER(99f, lduw $r15, $a6, 4)
444:	USER(99f, stuw $r15, $a5, 4)
	addi		$r10, $r10, -4
	j		$t1	
  .align 5	
1:						
	taddpci		$t1, 33f
	jcsi_le		$t1, $r10, 0
555:	USER(99f, ldub $r15, $a6, 1)
666:	USER(99f, stub $r15, $a5, 1)
	jdecnz		$t0, $r10, $r10	

33:	addi		$r8, $r10, 0
	ret


	.section .fixup,"ax"
	.align 3
99:	addi		$r8, $r10, 0
	ret
	.previous
		
ENDPROC(__copy_tofrom_user)
