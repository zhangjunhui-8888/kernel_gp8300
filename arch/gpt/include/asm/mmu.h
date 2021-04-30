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

#ifndef __ASM_GPT_MMU_H
#define __ASM_GPT_MMU_H

#include <linux/smp.h>

#ifndef __ASSEMBLY__
typedef struct{
	unsigned long asid;
	raw_spinlock_t id_lock;
	unsigned long vdso_base;
	unsigned int cpu;
} mm_context_t;
#endif

#endif /* __ASM_GPT_MMU_H */
