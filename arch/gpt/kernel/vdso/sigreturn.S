/*
 * Sigreturn trampoline for returning from a signal when the SA_RESTORER
 * flag is not set.
 *
 * Copyright (C) 2019 GPT Limited
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
 *
 */

#include <linux/linkage.h>
#include <asm/unistd.h>

	.text

	nop
ENTRY(__kernel_rt_sigreturn)
	rseti	$r7, __NR_rt_sigreturn
	swi	0
ENDPROC(__kernel_rt_sigreturn)
