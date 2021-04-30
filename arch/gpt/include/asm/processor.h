/*
 * arch/gpt/include/asm/processor.h
 *
 * Processor specific procedures.
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

#ifndef __ASM_GPT_PROCESSOR_H
#define __ASM_GPT_PROCESSOR_H

#include <asm/ptrace.h>
#include <asm/vfp.h>
#include <asm/hw_breakpoint.h>

#define STACK_TOP       TASK_SIZE
#define STACK_TOP_MAX   STACK_TOP

#define GPT_KUSER_BASE	0xffff0000
/*
 * Default implementation of macro that returns current
 * instruction pointer ("program counter").
 */
#define current_text_addr() ({ __label__ _l; _l: &&_l; })

/*
 * User space process size. 6 TB
 */

#define TASK_SIZE       (0x0000060000000000UL)
#define ARCH_LOW_ADDRESS_LIMIT (CONFIG_PHYSICAL_START + (1UL << 28))
/* This decides where the kernel will search for a free chunk of vm
 * space during mmap's.
 */
#define TASK_UNMAPPED_BASE      PAGE_ALIGN(TASK_SIZE / 8 * 3)

#ifndef __ASSEMBLY__

struct task_struct;

struct cpu_context {
	unsigned long r0;       /* kfn */
	unsigned long r1;       /* karg */
	unsigned long r2;
	unsigned long r3;
	unsigned long r4;
	unsigned long r5;
	unsigned long a1;
	unsigned long a2;
	unsigned long a3;
	unsigned long a8;
	unsigned long a9;
	unsigned long a10;
	unsigned long pc;       /* $t7 */
	unsigned long sp;
};

struct debug_info {
	/* Have we suspended stepping by a debugger? */
	int suspended_step;
	/* Allow breakpoints and watchpoints to be disabled for this thread. */
	int bps_disabled;
	int wps_disabled;
	/* wp triggered and implement single-step, so we must backup the address
	 * to send signal(SIGTRAP) */
	unsigned long wp_addr;
	/* Hardware breakpoints pinned to this task. */
	struct perf_event *hbp_breakp[GPTX_MAX_BRP];
	struct perf_event *hbp_watchp[GPTX_MAX_WTP];
};

struct thread_struct {
	struct cpu_context cpu_context;
	unsigned long tp_value; /* Store sys_clone arg 5 */
	struct vfp_state vfp_state;
	unsigned long fault_address;
	unsigned long fault_code;
	struct debug_info debug;
};

/*
 * At user->kernel entry, the pt_regs struct is stacked on the top of the
 * kernel-stack.  This macro allows us to find those regs for a task.
 * Notice that subsequent pt_regs stackings, like recursive interrupts
 * occurring while we're in the kernel, won't affect this - only the first
 * user->kernel transition registers are reached by this (i.e. not regs
 * for running signal handler)
 */
#define user_regs(thread_info)  (((struct pt_regs *)((unsigned long)(thread_info) + THREAD_SIZE - 16)) - 1)

/*
 * Dito but for the currently running task
 */

#define task_pt_regs(task) user_regs(task_thread_info(task))

//#define INIT_SP         (sizeof(init_stack) + (unsigned long) &init_stack)

#define INIT_THREAD  { }


#define KSTK_EIP(tsk)   (task_pt_regs(tsk)->pc)
#define KSTK_ESP(tsk)   (task_pt_regs(tsk)->sp)

void start_thread(struct pt_regs *regs, unsigned long nip, unsigned long sp);
void release_thread(struct task_struct *);
unsigned long get_wchan(struct task_struct *p);

/*
 * Free current thread data structures etc..
 */

extern inline void exit_thread(void)
{
	/* Nothing needs to be done.  */
}

#define init_stack      (init_thread_union.stack)

#define cpu_relax()     barrier()
#define cpu_relax_lowlatency() cpu_relax()

#include <machine/gpt_kernel.h>
#define DCACHE_ALIGNED(ptr)     ((unsigned long)ptr & (-(1UL << GPT_DCACHE_line_ln)))

#define HAVE_ARCH_PICK_MMAP_LAYOUT

#endif /* __ASSEMBLY__ */
#endif /* __ASM_GPT_PROCESSOR_H */
