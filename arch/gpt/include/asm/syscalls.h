/*
 * arch/gpt/include/asm/syscalls.h
 *
 * Handle system calls specs.
 *
 * Copyright (C) 2015, Optimum Semiconductor Technologies
 *  Enrique Barria <ebarria@optimumsemi.com>
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


#ifndef __ASM_GPT_SYSCALLS_H
#define __ASM_GPT_SYSCALLS_H

#include <asm-generic/syscalls.h>

/* From line 385. tls is 5th parameter
   #include <linux/syscalls.h>
*/
#ifdef	CONFIG_CLONE_BACKWARDS
asmlinkage long sys_clone(unsigned long clone_flags, unsigned long newsp,
			int __user *parent_tid, int tls, int __user *child_tid);
#else
asmlinkage long sys_clone(unsigned long clone_flags, unsigned long newsp,
                        int __user *parent_tid, int __user *child_tid, int tls);
#endif

void sys_do_abort(void);

#endif /* __ASM_GPT_SYSCALLS_H */
