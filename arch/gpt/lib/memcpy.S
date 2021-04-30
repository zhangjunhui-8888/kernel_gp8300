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

/**
 * memcpy - Copy one area of memory to another
 * @dest: Where to copy to
 * @src: Where to copy from
 * @count: The size of the area.
 *
 */
ENTRY(memcpy)
	aaddri		$a5, $r8, 0 
	aaddri		$a6, $r9, 0 

	or		$r14, $r8, $r9

	andi		$r14, $r14, 7		
	taddpci		$t1, 8f
	jci_eq		$t1, $r14, 0		// 8 bytes aligned

	andi		$r14, $r14, 3		
	taddpci		$t1, 4f	
	jci_eq		$t1, $r14, 0		// 4 bytes aligned

	taddpci		$t1, 1f			// non-aligned
	j		$t1			
8:
	addi		$r14, $r10, -8
	taddpci		$t1, 1f			// the rest bytes
	jcsi_le		$t1, $r14, 0	
	ldul		$r15, $a6, 8
	stul		$r15, $a5, 8
	addi		$r10, $r14, 0
	taddpci		$t1, 8b
	j		$t1	
4:
	addi		$r13, $r10, -4
	taddpci		$t1, 1f			// the rest bytes
	jcsi_le		$t1, $r13, 0	
	lduw		$r15, $a6, 4
	stuw		$r15, $a5, 4
	addi		$r10, $r13, 0
	taddpci		$t1, 4b
	j		$t1	
1:						
	taddpci		$t1, 33f
	jcsi_le		$t1, $r10, 0
	ldub		$r15, $a6, 1
	stub		$r15, $a5, 1
	taddpci		$t1, 1b
	jdecnz		$t1, $r10, $r10	
33:	
	ret
ENDPROC(memcpy)
