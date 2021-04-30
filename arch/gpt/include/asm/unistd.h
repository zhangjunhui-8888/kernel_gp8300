/*
 * gpt Linux
 *
 * Linux architectural port borrowing liberally from similar works of
 * others.  All original copyrights apply as per the original source
 * declaration.
 *
 * gpt implementation:
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#define __ARCH_HAVE_MMU
#define __ARCH_WANT_SYS_FORK
#define __ARCH_WANT_SYS_CLONE
#define __ARCH_WANT_SYSCALL_NO_AT
#define __ARCH_WANT_SYSCALL_NO_FLAGS
#define __ARCH_WANT_SYSCALL_DEPRECATED
#define __ARCH_WANT_SYS_VFORK

#include <uapi/asm/unistd.h>

#define NR_syscalls (__NR_syscalls)
