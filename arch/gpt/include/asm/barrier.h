#ifndef __GPT_BARRIER_H
#define __GPT_BARRIER_H

/* On GPT there is 'barrier' instruction to interrupt CPU pipeline,
 * And 'fence' instruction to order load/store.
 */
#define mb()	__asm__ __volatile__("fence\n":::"memory")
#define rmb()	__asm__ __volatile__("fence\n":::"memory")
#define wmb()	__asm__ __volatile__("fence\n":::"memory")

#define set_mb(__var, __value) \
	do { __var = __value; smp_mb(); } while(0)

#ifdef CONFIG_SMP
#define smp_mb()	mb()
#define smp_rmb()	rmb()
#define smp_wmb()	wmb()
#else
#define smp_mb()	barrier()
#define smp_rmb()	barrier()
#define smp_wmb()	barrier()
#endif

#include <asm-generic/barrier.h>

#endif /* !(__GPT_BARRIER_H) */
