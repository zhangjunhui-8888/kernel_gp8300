/*
 * arch/gpt/asm/smp.h
 *
 * Based on arch/arm64/include/asm/smp.h
 *
 * Copyright (C) 2017, General Processor Techologies Inc. & HXGPT Tech Ltd.
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
#ifndef __ASM_SMP_H
#define __ASM_SMP_H

#include <linux/threads.h>
#include <linux/cpumask.h>
#include <linux/thread_info.h>

#ifndef CONFIG_SMP
# error "<asm/smp.h> included in non-SMP build"
#endif

#define raw_smp_processor_id() (current_thread_info()->cpu)

struct seq_file;

/*
 * generate IPI list text
 */
extern void show_ipi_list(struct seq_file *p, int prec);

/*
 * Called from C code, this handles an IPI.
 */
extern void handle_IPI(u8 ipi_msg);

/*
 * Setup the set of possible CPUs (via set_cpu_possible)
 */
extern void smp_init_cpus(void);

/*
 * Provide a function to raise an IPI cross call on CPUs in callmap.
 */
extern void set_smp_cross_call(void (*)(const struct cpumask *, unsigned int));

extern void (*__smp_cross_call)(const struct cpumask *, unsigned int);

/*
 * Called from the secondary holding pen, this is the secondary CPU entry point.
 */
asmlinkage void _start(void);

/*
 * Initial data for bringing up a secondary CPU.
 */
struct secondary_data {
	volatile void *stack;
};
extern struct secondary_data secondary_data;
extern void secondary_entry(void);

extern void arch_send_call_function_single_ipi(int cpu);
extern void arch_send_call_function_ipi_mask(const struct cpumask *mask);

extern int __cpu_disable(void);

extern void __cpu_die(unsigned int cpu);
extern void cpu_die(void);

struct plat_smp_ops {
        void (*send_ipi_single)(unsigned int cpu, unsigned int action);
        void (*send_ipi_mask)(const struct cpumask *mask, unsigned int action);
        int (*cpu_kick)(unsigned int cpu, struct task_struct *idle);
#ifdef CONFIG_HOTPLUG_CPU
        int (*cpu_disable)(void);
        void (*cpu_die)(unsigned int cpu);
        int (*cpu_kill)(unsigned int cpu);
#endif
};

void register_smp_ops(struct plat_smp_ops *ops);
extern void __init plat_smp_init(void);
extern void __init plat_smp_finish(void);
#endif /* ifndef __ASM_SMP_H */
