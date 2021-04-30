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
* compare memory areas
*
* Parameters:
*  $r8 - const memory area 1 pointer
*  $r9 - const memory area 2 pointer
*  $r10 - the maximal compare byte length
* Returns:
*  $r8 - a compare result, maybe less than, equal to, or greater than ZERO
*/
ENTRY(memcmp)
	aaddri		$a5, $r8, 0 
	aaddri		$a6, $r9, 0 

	taddpci		$t0, 33f
	taddpci		$t1, 1f
1:						
	ldub		$r15, $a6, 1
	ldub		$r12, $a5, 1
	subf		$r8, $r15, $r12
	jci_ne		$t0, $r8, 0
	jdecnz		$t1, $r10, $r10	
33:	
	ret
ENDPROC(memcmp)
