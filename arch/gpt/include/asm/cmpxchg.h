/*
 * arch/gpt/include/asm/cmpxchg.h
 *
 * Atomic compare and exchange procedures.
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

#ifndef __ASM_GPT_CMPXCHG_H
#define __ASM_GPT_CMPXCHG_H

#ifdef CONFIG_SMP
#include <linux/bug.h>
/*
 * This function doesn't exist, so you'll get a linker error if
 * something tries to do an invalidly-sized xchg().
 */
extern void __xchg_called_with_bad_pointer(void);

#define __xchg_asm(sz, x, ptr, ptr_a, ret)			\
	__asm__ __volatile__(					\
	"fence\n"						\
	".align	5\n"						\
	"dctl_lock_l1	%3\n"					\
	"ld"sz"		%0, %1\n"				\
	"st"sz"		%2, %1\n"				\
	"dctl_unlock_l1 %3\n"					\
	: "=&r"(ret),"+&a"(ptr)		                         	\
        : "r"(x), "a"(ptr_a)                        	\
        : "memory");


static inline
unsigned long __xchg(unsigned long x, volatile void *ptr, int size)
{
	unsigned long flags, ret = 0;
	unsigned long ptr_align=DCACHE_ALIGNED(ptr);

	raw_local_irq_save(flags);
        switch (size) {
        case 1:
		__xchg_asm("b", x, ptr, ptr_align, ret);
		break;
        case 2:
		__xchg_asm("h", x, ptr, ptr_align, ret);
		break;
        case 4:
		__xchg_asm("w", x, ptr, ptr_align, ret);
		break;
        case 8:
		__xchg_asm("l", x, ptr, ptr_align, ret);
		break;
        default:
		BUILD_BUG();
        }
	raw_local_irq_restore(flags);
	return ret;
}

#define xchg(ptr,x) \
({ \
	__typeof__(*(ptr)) __ret; \
	__ret = (__typeof__(*(ptr))) \
		__xchg((unsigned long)(x), (ptr), sizeof(*(ptr))); \
	__ret; \
})

#define __HAVE_ARCH_CMPXCHG 1

extern unsigned long wrong_size_cmpxchg(volatile void *ptr)
        __noreturn;

/*
 * There is a compiler bug here if we use branch register in clobber list
 * so we don't compare old value and orignal value, instead we use
 * rc_"sz"_u_ne	%0, %1, %2
 * sel		%0, %1, %3
 */
#define __cmpxchg_asm(sz, ptr, old, new, ptr_a, oldval, tmp, c)		\
	__asm__ __volatile__(						\
	"	fence\n"						\
	"	.align          5\n"					\
	"	dctl_lock_l1    %5\n"					\
	"	ld"sz"          %1, %2\n"				\
	"	rc_"c"_u_ne	%0, %1, %3\n"				\
	"	sel		%0, %1, %4\n"				\
	"	st"sz"          %0, %2\n"				\
	"	dctl_unlock_l1  %5\n"					\
	: "=&r"(tmp), "=&r"(oldval),"+&a"(ptr)                       		\
        : "r"(old), "r"(new), "a"(ptr_a)			\
        : "memory", "$CA");

static inline unsigned long __cmpxchg_local_generic(volatile void *ptr,
                unsigned long old, unsigned long new, int size)
{

	unsigned long flags, tmp = 0, oldval = 0;
	unsigned long ptr_align=DCACHE_ALIGNED(ptr);

        /*
         * Sanity checking, compile-time.
         */
        if (size == 8 && sizeof(unsigned long) != 8)
                wrong_size_cmpxchg(ptr);

	raw_local_irq_save(flags);
        switch (size) {
        case 1:
		__cmpxchg_asm("b", ptr, (u8)old, new, ptr_align, oldval, tmp, "w");
		break;
        case 2:
		__cmpxchg_asm("h", ptr, (u16)old, new, ptr_align, oldval, tmp, "w");
		break;
        case 4:
		__cmpxchg_asm("w", ptr, (u32)old, new, ptr_align, oldval, tmp, "w");
		break;
        case 8:
		__cmpxchg_asm("l", ptr, old, new, ptr_align, oldval, tmp, "l");
		break;
        default:
		BUILD_BUG();
        }
	raw_local_irq_restore(flags);
	return oldval;
}

#define cmpxchg_local(ptr, o, n) \
({ \
	__typeof__(*(ptr)) __ret; \
	__ret = (__typeof__(*(ptr))) \
		__cmpxchg_local_generic((ptr), (unsigned long)(o), \
			  (unsigned long)(n), sizeof(*(ptr))); \
	__ret; \
})

#define cmpxchg(ptr, o, n)          cmpxchg_local((ptr), (o), (n))

#define cmpxchg64(ptr,o,n)		cmpxchg((ptr),(o),(n))
#define cmpxchg64_local(ptr,o,n)	cmpxchg_local((ptr),(o),(n))
#define cmpxchg64_relaxed(ptr,o,n)	cmpxchg_local((ptr),(o),(n))

#else

static inline unsigned long ext_to_size(unsigned long o, int size)
{
    unsigned long result;
    switch (size) {
        case 1:
            result = (u8)o;
	    break;
        case 2:
            result = (u16)o;
            break;
	case 4:
            result = (u32)o;
	    break;
	case 8:
            result = (u64)o;
            break;
        default:
            break;
    }
    return result;
}

#define cmpxchg_local(ptr, o, n)				  	       \
	((__typeof__(*(ptr)))__cmpxchg_local_generic((ptr), ext_to_size((unsigned long)(o), sizeof(*(ptr))),\
			(unsigned long)(n), sizeof(*(ptr))))

#include <asm-generic/cmpxchg.h>
#endif
#endif /* __ASM_GPT_CMPXCHG_H */
