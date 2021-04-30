/*
 * gpt process.c
 *
 * Linux architectural port borrowing liberally from similar works of
 * others.  All original copyrights apply as per the original source
 * declaration.
 *
 * Modifications for the gpt architecture:
 *
 *      This program is free software; you can redistribute it and/or
 *      modify it under the terms of the GNU General Public License
 *      as published by the Free Software Foundation; either version
 *      2 of the License, or (at your option) any later version.
 *
 * This file handles the architecture-dependent parts of process handling...
 */

#define __KERNEL_SYSCALLS__
#include <stdarg.h>

#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/stddef.h>
#include <linux/unistd.h>
#include <linux/ptrace.h>
#include <linux/slab.h>
#include <linux/elfcore.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/init_task.h>
#include <linux/mqueue.h>
#include <linux/fs.h>
#include <linux/smp.h>

#include <asm/uaccess.h>
#include <asm/pgtable.h>
#include <asm/io.h>
#include <asm/processor.h>
#include <asm/vfp.h>
#include <asm/stacktrace.h>

/*
* If a TLS pointer was passed to sys_clone, use it for the new thread.
*
* The kernel entry is:
*      long sys_clone(unsigned long clone_flags, unsigned long newsp,
*              int __user *parent_tid, int __user *child_tid, int tls);
*
* CONFIG_CLONE_BACKWARDS
* asmlinkage long sys_clone(unsigned long clone_flags, unsigned long newsp,
*		int __user *parent_tid, int,tls);
*                int __user *);
* */
#ifdef	CONFIG_CLONE_BACKWARDS
#define	TSL_ARG_INDEX	11
#else
#define	TSL_ARG_INDEX	12
#endif	/* CONFIG_CLONE_BACKWARDS */

/* defined in arch/gpt/kernel/entry.S */
extern void _do_cpu_idle(void);

void arch_cpu_idle(void)
{
        local_irq_enable();
       _do_cpu_idle();
}

static void tls_flush_thread(void)
{
	unsigned long tp_value = 0;
	asm("aaddri	$a2, %0, 0" ::"r"(tp_value));
}
/*
 * When a process does an "exec", machine state like FPU and debug
 * registers need to be reset.  This is a hook function for that.
 * Currently we don't have any such state to reset, so this is empty.
 */
/* Called by kernel in fs/exec.c */
void flush_thread(void)
{
	vfp_flush_thread();
	tls_flush_thread();
}

/* Called by kernel */
void show_regs(struct pt_regs *regs)
{
        extern void show_registers(struct pt_regs *regs);

        /* In printk.c */
        show_regs_print_info(KERN_DEFAULT);
        /* In gpt/kernel/traps.c */
        show_registers(regs);
}

/* Called from kernel: kernel/exit.c */
void release_thread(struct task_struct *dead_task)
{
}

/*
 * Copy the thread-specific (arch specific) info from the current
 * process to the new one p
 */
int arch_dup_task_struct(struct task_struct *dst, struct task_struct *src)
{
	/* Save fpu/vpu state to orig tsk for child */
	fpu_save_current_state(&src->thread.vfp_state);
	vpu_save_current_state(&src->thread.vfp_state);

	*dst = *src;
	return 0;
}

/*
 * copy_thread
 * @clone_flags: flags
 * @usp: user stack pointer or fn for kernel thread
 * @arg: arg to fn for kernel thread; always NULL for userspace thread
 * @p: the newly created task
 * @regs: CPU context to copy for userspace thread; always NULL for kthread
 *
 * At the top of a newly initialized kernel stack are two stacked pt_reg
 * structures.  The first (topmost) is the userspace context of the thread.
 * The second is the kernelspace context of the thread.
 *
 * A kernel thread will not be returning to userspace, so the topmost pt_regs
 * struct can be uninitialized; it _does_ need to exist, though, because
 * a kernel thread can become a userspace thread by doing a kernel_execve, in
 * which case the topmost context will be initialized and used for 'returning'
 * to userspace.
 *
 * A kernel thread 'fn' may return; this is effectively what happens when
 * kernel_execve is called.  In that case, the userspace pt_regs must have
 * been initialized (which kernel_execve takes care of, see start_thread
 * below); ret_from_fork will then continue its execution causing the
 * 'kernel thread' to return to userspace as a userspace thread.
 */
asmlinkage void ret_from_fork(void) __asm__("ret_from_fork");

/* Called from kernel: kernel/fork.c */
int copy_thread(unsigned long clone_flags, unsigned long usp, unsigned long arg, struct task_struct *p)
{
	struct pt_regs *childregs = task_pt_regs(p);
	unsigned long tls = p->thread.tp_value;
	
	p->set_child_tid = p->clear_child_tid = NULL;
	
	memset(&p->thread.cpu_context, 0, sizeof(struct cpu_context));
	
	if (likely(!(p->flags & PF_KTHREAD))) {
		/* Transfer pt_regs from current stack to new stack */
		*childregs = *current_pt_regs();
		childregs->r_regs[8] = 0;  /* Result from fork() */

		if (usp) {
			childregs->sp = usp;
		}
		/* Sync $a2 to tls, $a2 is TLS pointer for user thread */
		asm volatile ("rseta %0, $a2" : "=r" (tls));

		if (clone_flags & CLONE_SETTLS) {
			tls = childregs->r_regs[TSL_ARG_INDEX];
		}

	} else {
		memset(childregs, 0, sizeof(struct pt_regs));
		childregs->sr = PSC_MODE_KERNEL|CACHE_MODE| PSC_FPU_EN | PSC_VPU_EN;

		/* kernel thread callback and argument. */
		p->thread.cpu_context.r0 = usp;
		p->thread.cpu_context.r1 = arg;
	}

	p->thread.cpu_context.pc = (unsigned long)ret_from_fork;
	p->thread.cpu_context.sp = (unsigned long)childregs;
	p->thread.tp_value = tls;

	ptrace_hw_copy_thread(p);
	return 0;
}

/*
 * Set up a thread for executing a new program
 */
/* Called from kernel */
void start_thread(struct pt_regs *regs, unsigned long pc, unsigned long sp)
{
        memset(regs, 0, sizeof(struct pt_regs));
        
	set_fs(USER_DS);
	
	regs->syscallno = ~0UL; /* avoid deal the system call again */
        regs->pc = pc;
        regs->sp = sp;
	regs->xen = DEFAULT_XEN;        
        regs->sr = PSC_MODE_USER | CACHE_MODE | PSC_FPU_EN | PSC_VPU_EN;
}

static void tls_thread_switch(struct task_struct *next)
{
        asm volatile ("sta $a2, %0\n"
             "lda $a2, %1\n"
	 : "=m"(current->thread.tp_value)
	 : "m"(next->thread.tp_value)
	 : "memory", "$a2");
}

/* in ./arch/gpt/kernel/entry.S */
extern struct task_struct *_switch(struct task_struct *old, struct task_struct *new);

/* kernel switch_to maps to this fcn through include/asm-generic/switch_to.h */
struct task_struct *__switch_to(struct task_struct *old,
                                struct task_struct *new)
{
        struct task_struct *last;
        
	vfp_thread_switch(new);
	tls_thread_switch(new);

	mb();
        last = _switch(old, new);

        return last;
}

unsigned long get_wchan(struct task_struct *p)
{
	struct stackframe frame;
	unsigned long stack_page;
	int count = 0;
	if (!p || p == current || p->state == TASK_RUNNING)
		return 0;

	frame.fp = thread_saved_fp(p);
	frame.sp = thread_saved_sp(p);
	frame.pc = thread_saved_pc(p);
	stack_page = (unsigned long)task_stack_page(p);
	do {
		if (frame.sp < stack_page ||
		    frame.sp >= stack_page + THREAD_SIZE ||
		    unwind_frame(&frame))
			return 0;
		if (!in_sched_functions(frame.pc))
			return frame.pc;
	} while (count ++ < 16);

        return 0;
}

