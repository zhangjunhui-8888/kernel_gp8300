/*
 * Copyright (C) 2016 GPT Ltd.
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
#ifndef __ASM_PARAM_H
#define __ASM_PARAM_H

#define EXEC_PAGESIZE	32768

/* TRAP numbers for user programs to access kernel */
#define SYSCALL_TRAP_INT        4
#define SYSCALL_TRAP_ATOMIC_32	6
#define SYSCALL_TRAP_ATOMIC_64	7

#include <asm-generic/param.h>

#endif /* __ASM_PARAM_H */
