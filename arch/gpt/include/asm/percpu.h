/*
 * Copyright (C) 2020 GPT Ltd.
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
#ifndef __ASM_PERCPU_H
#define __ASM_PERCPU_H

#ifdef CONFIG_SMP

static inline void set_my_cpu_offset(unsigned long off)
{
	asm volatile("ssetr $s2, %0\n" :: "r" (off) : "memory", "$s2");
}

static inline unsigned long __my_cpu_offset(void)
{
	unsigned long off;

	/*
	 * We want to allow caching the value, so avoid using volatile and
	 * instead use a fake stack read to hazard against barrier().
	 */
	asm("rsets %0, $s2\n" : "=r" (off) :
		"m" (*(const unsigned long *)current_stack_pointer): "memory");

	return off;
}
#define __my_cpu_offset __my_cpu_offset()

#else	/* !CONFIG_SMP */

#define set_my_cpu_offset(x)	do { } while (0)

#endif /* CONFIG_SMP */

#include <asm-generic/percpu.h>

#endif /* __ASM_PERCPU_H */
