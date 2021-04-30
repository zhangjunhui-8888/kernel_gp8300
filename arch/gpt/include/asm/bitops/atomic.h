/*
 * arch/gpt/include/asm/bitops/atomic.h
 *
 * Atomic bit operations.
 *
 * Copyright (C) 2015-2017, Optimum Semiconductor Technologies
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

#ifndef __ASM_GPT_BITOPS_ATOMIC_H
#define __ASM_GPT_BITOPS_ATOMIC_H

#include <asm/processor.h>

static inline void set_bit(int nr, volatile unsigned long *addr)
{
	unsigned long flags;
        unsigned long mask = BIT_MASK(nr);
        unsigned long *p = ((unsigned long *)addr) + BIT_WORD(nr);
	unsigned long p_align=DCACHE_ALIGNED(p);
	unsigned long tmp=0;

	raw_local_irq_save(flags);
	asm volatile (
	"fence\n"
        ".align 5\n"
	"dctl_lock_l1    %3\n"
	"ldl             %0,%1\n"
	"or              %0,%0,%2\n"
	"stl             %0,%1\n"
	"dctl_unlock_l1  %3\n"
        : "=&r"(tmp),"+m"(*p)
        : "r"(mask), "a"(p_align)
        : "memory");
	raw_local_irq_restore(flags);
}

static inline void clear_bit(int nr, volatile unsigned long *addr)
{
	unsigned long flags;
        unsigned long mask = BIT_MASK(nr);
        unsigned long *p = ((unsigned long *)addr) + BIT_WORD(nr);
	unsigned long p_align=DCACHE_ALIGNED(p);
	unsigned long tmp=0;

	raw_local_irq_save(flags);

	asm volatile (
	"fence\n"
        ".align 5\n"
	"dctl_lock_l1    %3\n"
	"ldl             %0,%1\n"
	"cand            %0,%2,%0\n"
	"stl             %0,%1\n"
	"dctl_unlock_l1  %3\n"
        : "=&r"(tmp),"+m"(*p)
	: "r"(mask), "a"(p_align)
        : "memory");

	raw_local_irq_restore(flags);
}

static inline void change_bit(int nr, volatile unsigned long *addr)
{
	unsigned long flags;
        unsigned long mask = BIT_MASK(nr);
        unsigned long *p = ((unsigned long *)addr) + BIT_WORD(nr);
	unsigned long p_align=DCACHE_ALIGNED(p);
	unsigned long tmp=0;

	raw_local_irq_save(flags);

	asm volatile (
	"fence\n"
        ".align 5\n"
	"dctl_lock_l1    %3\n"
	"ldl             %0,%1\n"
	"xor             %0,%0,%2\n"
	"stl             %0,%1\n"
        "dctl_unlock_l1  %3\n"
        : "=&r"(tmp),"+m"(*p)
        : "r"(mask), "a"(p_align)
        : "memory");

	raw_local_irq_restore(flags);
}

static inline int test_and_set_bit(int nr, volatile unsigned long *addr)
{
	unsigned long flags;
        unsigned long mask = BIT_MASK(nr);
        unsigned long *p = ((unsigned long *)addr) + BIT_WORD(nr);
	unsigned long p_align=DCACHE_ALIGNED(p);
        unsigned long old=0, tmp=0;

	raw_local_irq_save(flags);
	asm volatile (
	"fence\n"
        ".align 5\n"
	"dctl_lock_l1    %4\n"
	"ldl             %0,%2\n"
	"or              %1,%0,%3\n"
	"stl             %1,%2\n"
	"dctl_unlock_l1  %4\n"
        : "=&r"(old),"=r"(tmp),"+m"(*p)
        : "r"(mask), "a"(p_align)
        : "memory");
	raw_local_irq_restore(flags);

        return (old & mask) != 0;
}

static inline int test_and_clear_bit(int nr, volatile unsigned long *addr)
{
	unsigned long flags;
        unsigned long mask = BIT_MASK(nr);
        unsigned long *p = ((unsigned long *)addr) + BIT_WORD(nr);
	unsigned long p_align=DCACHE_ALIGNED(p);
        unsigned long old=0, tmp=0;

	raw_local_irq_save(flags);
	asm volatile (
	"fence\n"
        ".align 5\n"
	"dctl_lock_l1    %4\n"
	"ldl             %0,%2\n"
	"cand             %1,%3,%0\n"
	"stl             %1,%2\n"
	"dctl_unlock_l1  %4\n"
        : "=&r"(old),"=r"(tmp), "+m"(*p)
        : "r"(mask), "a"(p_align)
        : "memory");
	raw_local_irq_restore(flags);

        return (old & mask) != 0;
}

#endif /* __ASM_GPT_BITOPS_ATOMIC_H */
