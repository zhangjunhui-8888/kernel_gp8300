/*
 * Copyright (C) 2019, General Processor Techologies Inc.
 * lwang <lwang@hxgpt.com>
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
/* int strcmp (const char *s1, const char *s2);
 * Compare two strings using unsigned comparison.
 */
ENTRY(strcmp)
	taddpci		$t0, Lnot_aligned8
	aaddri		$a5, $r8, 0
	andi		$r11, $r8, 7
	aaddri		$a6, $r9, 0
	andi		$r12, $r9, 7
	or		$r13, $r11, $r12
	jci_ne		$t0, $r13, 0

# inputs aligned on an 8 byte boundary
Laligned8_loop_setup:
	taddpci		$t0, Llong_compare8
	taddpci		$t1, Laligned8_loop

Laligned8_loop:
	ldul		$r8, $a5, 8
	ldul		$r9, $a6, 8
	countnzb	$r6, $r8, $r9
	jci_ne		$t0, $r6, 8
	jc_eq		$t1, $r8, $r9

# no null found but byte mismatch found

	swap		$r8, $r8
	swap		$r9, $r9
	subf		$r8, $r9, $r8
	sat_s_w		$r8, $r8
	ret

# Return the result of comparing two aligned long words.
# A terminating null has been found, and $r6 contains the countnzb value.
Llong_compare8:
	shli		$r6, $r6, 3
	subfi		$r6, $r6, 56
	shl		$r8, $r8, $r6
	shl		$r9, $r9, $r6
	swap		$r8, $r8
	swap		$r9, $r9
	subf		$r8, $r9, $r8
	sat_s_w		$r8, $r8
	ret

# inputs not aligned on an 8 byte boundary
Lnot_aligned8:
	taddpci		$t0, Lalign_right_setup
	jcs_lt		$t0, $r12, $r11

# Compare bytes till left is aligned on a long word boundary.
Lalign_left_setup:
	taddpci		$t0, Lcheck_right_alignment
	jci_eq		$t0, $r11, 0
	taddpci		$t0, Lchar_compare
	taddpci		$t1, Lalign_left_loop
	subfi		$r7, $r11, 8

Lalign_left_loop:
	ldub		$r8, $a5, 1
	ldub		$r9, $a6, 1
	jc_ne		$t0, $r8, $r9
	jci_eq		$t0, $r8, 0
	jdecnz		$t1, $r7, $r7

# Left is aligned on a long word boundary.
# Check right's alignment.
Lcheck_right_alignment:

	taddpci		$t0, Laligned_loop_setup
	jc_eq		$t0, $r11, $r12

# Compare long words.
Lmisaligned_right_setup:
	taddpci		$t1, Lmisaligned_right_loop
	rseta		$r8, $a6
	andi		$r8, $r8, -8
	aaddri		$a6, $r8, 0
	ldul		$r10, $a6, 8
	subf		$r11, $r11, $r12
	subfi		$r15, $r11, 8
	shli		$r11, $r11, 3
	subfi		$r12, $r11, 64

Lmisaligned_right_loop:
	ldul		$r8, $a5, 8
	addi		$r9, $r10, 0
	shr		$r9, $r9, $r11

# a NULL is detected in $r9 do not load from next word
	countnzb	$r14, $r9, $r9
	taddpci		$t0, Lskip_right_load_word
	jc_ne		$t0, $r14, $r15

	ldul		$r10, $a6, 8
	shl		$r13, $r10, $r12
	or		$r9, $r9, $r13

Lskip_right_load_word:
	countnzb	$r6, $r8, $r9
	taddpci		$t0, Llong_compare
	jci_ne		$t0, $r6, 8
	jc_eq		$t1, $r8, $r9

# no null found but byte mismatch found
	swap		$r8, $r8
	swap		$r9, $r9
	subf		$r8, $r9, $r8
	sat_s_w		$r8, $r8
	ret

# Compare bytes till right is aligned on a long word boundary.
Lalign_right_setup:
	taddpci		$t0, Lmisaligned_left_setup
	jci_eq		$t0, $r12, 0
	taddpci		$t0, Lchar_compare
	taddpci		$t1, Lalign_right_loop
	subfi		$r7, $r12, 8

Lalign_right_loop:
	ldub		$r8, $a5, 1
	ldub		$r9, $a6, 1
	jc_ne		$t0, $r8, $r9
	jci_eq		$t0, $r8, 0
	jdecnz		$t1, $r7, $r7

# Right is aligned on a long word boundary.
# Compare long words.
Lmisaligned_left_setup:
	taddpci		$t1, Lmisaligned_left_loop
	rseta		$r9, $a5
	andi		$r9, $r9, -8
	aaddri		$a5, $r9, 0
	ldul		$r10, $a5, 8
	subf		$r11, $r12, $r11
	subfi		$r15, $r11, 8
	shli		$r11, $r11, 3
	subfi		$r12, $r11, 64

Lmisaligned_left_loop:
	ldul		$r9, $a6, 8
	addi		$r8, $r10, 0
	shr		$r8, $r8, $r11

# a NULL is detected in $r8 do not load from next word
	countnzb	$r14, $r8, $r8
	taddpci		$t0, Lskip_left_load_word
	jc_ne		$t0, $r14, $r15

	ldul		$r10, $a5, 8
	shl		$r13, $r10, $r12
	or		$r8, $r8, $r13

Lskip_left_load_word:
	countnzb	$r6, $r8, $r9
	taddpci		$t0, Llong_compare
	jci_ne		$t0, $r6, 8
	jc_eq		$t1, $r8, $r9

# no null found but byte mismatch found
	swap		$r8, $r8
	swap		$r9, $r9
	subf		$r8, $r9, $r8
	sat_s_w		$r8, $r8
	ret

# Left and right are both aligned.
# Compare long words.
Laligned_loop_setup:
	taddpci		$t0, Llong_compare
	taddpci		$t1, Laligned_loop

Laligned_loop:
	ldul		$r8, $a5, 8
	ldul		$r9, $a6, 8
	countnzb	$r6, $r8, $r9
	jci_ne		$t0, $r6, 8
	jc_eq		$t1, $r8, $r9

	swap		$r8, $r8
	swap		$r9, $r9
	subf		$r8, $r9, $r8
	sat_s_w		$r8, $r8
	ret

# Return the result of comparing two aligned long words.
# A terminating null has been found, and $r6 contains the countnzb value.
Llong_compare:
	shli		$r6, $r6, 3
	subfi		$r6, $r6, 56
	shl		$r8, $r8, $r6
	shl		$r9, $r9, $r6
	swap		$r8, $r8
	swap		$r9, $r9
	subf		$r8, $r9, $r8
	sat_s_w		$r8, $r8
	ret

# Return the result of comparing a single byte
Lchar_compare:
	subf		$r8, $r9, $r8
	sat_s_w		$r8, $r8
	ret
ENDPROC(strcmp)
