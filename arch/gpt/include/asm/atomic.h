/*
 * arch/gpt/include/asm/atomic.h, refer to arm64
 *
 * Atomic operations 32 and 64 bits.
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
#ifndef __ASM_ATOMIC_H
#define __ASM_ATOMIC_H

#include <linux/types.h>
#include <linux/irqflags.h>
#include <asm/processor.h>
#include <asm/cmpxchg.h>
#include <asm/barrier.h>

#define ATOMIC_INIT(i)	{ (i) }

#ifdef __KERNEL__

#define atomic_read(v)  ACCESS_ONCE((v)->counter)
#define atomic_set(v, i) (((v)->counter) = (i))

#define ATOMIC_OP(op, asm_op)						\
static inline void atomic_##op(int i, atomic_t *v)			\
{									\
	int result=0;							\
	unsigned long flags;						\
	unsigned long v_align=DCACHE_ALIGNED(v);			\
									\
	raw_local_irq_save(flags);					\
	asm volatile (							\
	"fence\n"							\
	".align	5\n"							\
	"dctl_lock_l1    %2\n"						\
	"ldsw            %0,%1\n"					\
	"" #asm_op "    %0,%3,%0\n"					\
	"stw             %0,%1\n"					\
	"dctl_unlock_l1  %2\n"						\
	: "+&r"(result), "+m"(ACCESS_ONCE(v->counter)), "+a"(v_align)	\
	: "r"(i)							\
	: "memory");							\
	raw_local_irq_restore(flags);					\
}									\

#define ATOMIC_OP_RETURN(op, asm_op)					\
static inline int atomic_##op##_return(int i, atomic_t *v)		\
{									\
	int result=0;							\
	unsigned long flags;						\
	unsigned long v_align=DCACHE_ALIGNED(v);			\
									\
	raw_local_irq_save(flags);					\
	asm volatile (							\
	"fence\n"							\
	".align	5\n"							\
	"dctl_lock_l1    %2\n"						\
	"ldsw            %0,%1\n"					\
	"" #asm_op "    %0,%3,%0\n"					\
	"stw             %0,%1\n"					\
	"dctl_unlock_l1  %2\n"						\
	: "+&r"(result), "+m"(ACCESS_ONCE(v->counter)), "+a"(v_align)	\
	: "r"(i)							\
	: "memory");							\
	raw_local_irq_restore(flags);					\
									\
	smp_mb();							\
	return result;							\
}

#define ATOMIC_OPS(op, asm_op)						\
	ATOMIC_OP(op, asm_op)						\
	ATOMIC_OP_RETURN(op, asm_op)

ATOMIC_OPS(add, add)
ATOMIC_OPS(sub, subf)

#undef ATOMIC_OPS
#undef ATOMIC_OP_RETURN
#undef ATOMIC_OP

static inline int atomic_cmpxchg(atomic_t *ptr, int old, int new)
{
	unsigned long tmp=0;
        unsigned long flags;
	unsigned long ptr_a=DCACHE_ALIGNED(ptr);
	int oldval;

	smp_mb();

        raw_local_irq_save(flags);
	asm volatile(
	"	.align          5\n"
	"	dctl_lock_l1    %3\n"
	"	ldw          	%1, %2\n"
	"	rc_w_u_ne	%0, %1, %4\n"
	"	sel		%0, %1, %5\n"
	"	stw          	%0, %2\n"
	"	dctl_unlock_l1  %3\n"
	: "=&r"(tmp), "=&r"(oldval),"+&a"(ptr), "+a"(ptr_a)
        : "r"(old), "r"(new)
        : "memory", "$CA");
        raw_local_irq_restore(flags);

	smp_mb();
	return oldval;
}

#define atomic_xchg(v, new) (xchg(&((v)->counter), new))

static inline int __atomic_add_unless(atomic_t *v, int a, int u)
{
	int c, old;

	c = atomic_read(v);
	while (c != u && (old = atomic_cmpxchg((v), c, c + a)) != c)
		c = old;
	return c;
}

#define atomic_inc(v)		atomic_add(1, v)
#define atomic_dec(v)		atomic_sub(1, v)

#define atomic_inc_and_test(v)	(atomic_add_return(1, v) == 0)
#define atomic_dec_and_test(v)	(atomic_sub_return(1, v) == 0)
#define atomic_inc_return(v)    (atomic_add_return(1, v))
#define atomic_dec_return(v)    (atomic_sub_return(1, v))
#define atomic_sub_and_test(i, v) (atomic_sub_return(i, v) == 0)

#define atomic_add_negative(i,v) (atomic_add_return(i, v) < 0)

/*
 * 64-bit atomic operations.
 */
#define ATOMIC64_INIT(i) { (i) }

#define atomic64_read(v)	ACCESS_ONCE((v)->counter)
#define atomic64_set(v,i)	(((v)->counter) = (i))

#define ATOMIC64_OP(op, asm_op)						\
static inline void atomic64_##op(long i, atomic64_t *v)			\
{									\
	long result=0;							\
	unsigned long flags;						\
	unsigned long v_align=DCACHE_ALIGNED(v);			\
									\
	raw_local_irq_save(flags);					\
	asm volatile (							\
	"fence\n"							\
	".align 5\n"							\
	"dctl_lock_l1    %2\n"						\
	"ldl             %0,%1\n"					\
	"" #asm_op "     %0,%3,%0\n"					\
	"stl             %0,%1\n"					\
	"dctl_unlock_l1  %2\n"						\
	: "=&r"(result), "+m"(ACCESS_ONCE(v->counter)), "+a"(v_align)	\
	: "r"(i)							\
	: "memory");							\
	raw_local_irq_restore(flags);					\
}									\

#define ATOMIC64_OP_RETURN(op, asm_op)					\
static inline long atomic64_##op##_return(long i, atomic64_t *v)	\
{									\
	long result=0;							\
	unsigned long flags;						\
	unsigned long v_align=DCACHE_ALIGNED(v);			\
									\
	raw_local_irq_save(flags);					\
	asm volatile (							\
	"fence\n"							\
	".align 5\n"							\
	"dctl_lock_l1    %2\n"						\
	"ldl             %0,%1\n"					\
	"" #asm_op "     %0,%3,%0\n"					\
	"stl             %0,%1\n"					\
	"dctl_unlock_l1  %2\n"						\
	: "=&r"(result), "+m"(ACCESS_ONCE(v->counter)), "+a"(v_align)	\
	: "r"(i)							\
	: "memory");							\
	raw_local_irq_restore(flags);					\
									\
	smp_mb();							\
	return result;							\
}

#define ATOMIC64_OPS(op, asm_op)					\
	ATOMIC64_OP(op, asm_op)						\
	ATOMIC64_OP_RETURN(op, asm_op)

ATOMIC64_OPS(add, add)
ATOMIC64_OPS(sub, subf)

#undef ATOMIC64_OPS
#undef ATOMIC64_OP_RETURN
#undef ATOMIC64_OP

static inline long atomic64_cmpxchg(atomic64_t *ptr, long old, long new)
{
	long oldval;
	unsigned long tmp=0;
        unsigned long flags;
	unsigned long ptr_a=DCACHE_ALIGNED(ptr);

	smp_mb();

        raw_local_irq_save(flags);
	asm volatile(
	"	.align          5\n"
	"	dctl_lock_l1    %3\n"
	"	ldl          	%1, %2\n"
	"	rc_l_u_ne	%0, %1, %4\n"
	"	sel		%0, %1, %5\n"
	"	stl          	%0, %2\n"
	"	dctl_unlock_l1  %3\n"
	: "=&r"(tmp), "=&r"(oldval),"+&a"(ptr), "+a"(ptr_a)
        : "r"(old), "r"(new)
        : "memory", "$CA");
        raw_local_irq_restore(flags);

	smp_mb();
	return oldval;
}

#define atomic64_xchg(v, new) (xchg(&((v)->counter), new))

static inline long atomic64_dec_if_positive(atomic64_t *v)
{
	long result=0;
	unsigned long flags;
	unsigned long v_align=DCACHE_ALIGNED(v);

	raw_local_irq_save(flags);
	asm volatile (
	"	fence\n"
	"	taddpci         $t1,1f\n"
	"	.align          5\n"
	"	dctl_lock_l1    %2\n"
	"	ldl             %0,%1\n"
	"	addi            %0,%0,-1\n"
	"	jcsi_lt         $t1,%0,0\n"
	"	stl             %0,%1\n"
	"1:	dctl_unlock_l1  %2\n"
	: "=r"(result), "+m"(v->counter), "+a"(v_align)
	:
	: "memory", "$t1");
	raw_local_irq_restore(flags);

	return result;
}

static inline int atomic64_add_unless(atomic64_t *v, long a, long u)
{
	long c, old;

	c = atomic64_read(v);
	while (c != u && (old = atomic64_cmpxchg((v), c, c + a)) != c)
		c = old;

	return c != u;
}

#define atomic64_add_negative(a, v)	(atomic64_add_return((a), (v)) < 0)
#define atomic64_inc(v)			atomic64_add(1LL, (v))
#define atomic64_inc_return(v)		atomic64_add_return(1LL, (v))
#define atomic64_inc_and_test(v)	(atomic64_inc_return(v) == 0)
#define atomic64_sub_and_test(a, v)	(atomic64_sub_return((a), (v)) == 0)
#define atomic64_dec(v)			atomic64_sub(1LL, (v))
#define atomic64_dec_return(v)		atomic64_sub_return(1LL, (v))
#define atomic64_dec_and_test(v)	(atomic64_dec_return((v)) == 0)
#define atomic64_inc_not_zero(v)	atomic64_add_unless((v), 1LL, 0LL)

#endif
#endif
