/*
 * Copyright (C) 2018, General Processor Techologies Inc.
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

/*
 * Find the last occurrence of a character in a string.
 *
 * Parameters:
 *	$r8 - str
 *	$r9 - c
 * Returns:
 *	$r8 - address of last occurrence of 'c' or 0
 */
ENTRY(strrchr)
	aaddri	$a7, $r8, 0
	rseti	$r8, 1
	andi	$r9, $r9, 0xff
	taddpci	$t0, 1f
	taddpci	$t1, 2f
1:
	ldub	$r14, $a7, 1
	jci_eq	$t1, $r14, 0		// if reach end
	jc_ne	$t0, $r9, $r14		// if not equal to c
	rseta	$r8, $a7
	j	$t0
2:	
	addi	$r8, $r8, -1
	ret
ENDPROC(strrchr)
