/*
 * gpt traps.c
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
 *  Here we handle the break vectors not used by the system call
 *  mechanism, as well as some general stack/register dumping
 *  things.
 *
 */


#include <linux/kallsyms.h>
#include <linux/spinlock.h>
//#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/kdebug.h>
#include <linux/module.h>
#include <linux/kexec.h>
#include <linux/kmod.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/ptrace.h>
#include <linux/timer.h>
#include <linux/mm.h>
#include <asm/bug.h>

#include <asm/stacktrace.h>
#include <asm/segment.h>
#include <asm/io.h>
#include <asm/pgtable.h>

static int kstack_depth_to_print = 0x100;

int show_unhandled_signals = 1;

static void dump_instr(const char *lvl, struct pt_regs *regs)
{
	unsigned long addr = instruction_pointer(regs);
	mm_segment_t fs;
	char str[sizeof("00000000 ") * 5 + 2 + 1], *p = str;
	int i;

	/*
	 * We need to switch to kernel mode so that we can use __get_user
	 * to safely read from kernel space.  Note that we now dump the
	 * code first, just in case the backtrace kills us.
	 */
	fs = get_fs();
	set_fs(KERNEL_DS);

	for (i = -4; i < 1; i++) {
		unsigned int val, bad;

		bad = __get_user(val, &((u32 *)addr)[i]);

		if (!bad)
			p += sprintf(p, i == 0 ? "(%08x) " : "%08x ", val);
		else {
			p += sprintf(p, "bad PC value");
			break;
		}
	}
	printk("%sCode: %s\n", lvl, str);

	set_fs(fs);
}


static void dump_backtrace(struct pt_regs *regs, struct task_struct *tsk)
{
	struct stackframe frame;

	pr_debug("%s(regs = %p tsk = %p)\n", __func__, regs, tsk);

	if (!tsk)
		tsk = current;

	if (regs) {
		frame.fp = regs->a_regs[1];
		frame.sp = regs->sp;
		frame.pc = regs->pc;
	} else if (tsk == current) {
		frame.fp = (unsigned long)__builtin_frame_address(0);
		frame.sp = current_stack_pointer;
		frame.pc = (unsigned long)dump_backtrace;
	} else {
		/*
		 * task blocked in __switch_to
		 */
		frame.fp = thread_saved_fp(tsk);
		frame.sp = thread_saved_sp(tsk);
		frame.pc = thread_saved_pc(tsk);
	}

	pr_emerg("Call trace:\n");
	while (1) {
		unsigned long where = frame.pc;
		int ret;

		ret = unwind_frame(&frame);
		if (ret < 0)
			break;
		print_ip_sym(where);
	}
}

static void show_trace(struct task_struct *task, unsigned long *stack)
{
	dump_backtrace(NULL, task);
}

/* displays a short stack trace */
void show_stack(struct task_struct *task, unsigned long *esp)
{
        unsigned long addr, *stack;
        int i;

        if (esp == NULL)
                esp = (unsigned long *)&esp;

        stack = esp;

        printk("Stack dump [0x%016lx]:\n", (unsigned long)esp);
        for (i = 0; i < kstack_depth_to_print; i++) {
                if (kstack_end(stack))
                        break;
                if (__get_user(addr, stack)) {
                        printk("Failing address 0x%016lx\n", (unsigned long)stack);
                        break;
                }
                stack++;

                printk("sp + %03d: 0x%016lx\n", i * 8, addr);
        }
        printk("\n");

        show_trace(task, esp);

        return;
}

void show_registers(struct pt_regs *regs)
{
	unsigned long long curpgd;
	asm volatile ("rsets %0, $s0":"=r"(curpgd)::"memory");
        printk("CPU #%lx\n:"
               "   PC: %016llx    SR: %016llx    SP: %016llx    LR: %016llx\n"
               "   XEN: %016llx    XRT: %016lx    XST: %016lx    CA: %016llx\n"
               "   IXI: %016lx    IXA: %016lx\n"
               "   CURPDG: %016llx\n",
               sprget_gpt(THID), regs->pc, regs->sr, regs->sp, regs->t_regs[7],
               regs->xen, sprget_gpt(XRT), sprget_gpt(XST), regs->ca, sprget_gpt(IXI1H), sprget_gpt(IXA1H), curpgd);

        printk("\n");
        printk("a01: %016llx a02: %016llx a03: %016llx\n",
                regs->a_regs[1], regs->a_regs[2], regs->a_regs[3]);
        printk("a04: %016llx a05: %016llx a06: %016llx a07: %016llx\n",
                regs->a_regs[4], regs->a_regs[5], regs->a_regs[6], regs->a_regs[7]);
        printk("a08: %016llx a09: %016llx a10: %016llx\n",
                regs->a_regs[8], regs->a_regs[9], regs->a_regs[10]);
        printk("r00: %016llx r01: %016llx r02: %016llx r03: %016llx\n",
                regs->r_regs[0], regs->r_regs[1], regs->r_regs[2], regs->r_regs[3]);
        printk("r04: %016llx r05: %016llx r06: %016llx r07: %016llx\n",
                regs->r_regs[4], regs->r_regs[5], regs->r_regs[6], regs->r_regs[7]);
        printk("r08: %016llx r09: %016llx r10: %016llx r11: %016llx\n",
                regs->r_regs[8], regs->r_regs[9], regs->r_regs[10], regs->r_regs[11]);
        printk("r12: %016llx r13: %016llx r14: %016llx r15: %016llx\n",
                regs->r_regs[12], regs->r_regs[13], regs->r_regs[14], regs->r_regs[15]);
        printk("t0: %016llx t1: %016llx t4: %016llx\n",
                regs->t_regs[0], regs->t_regs[1], regs->t_regs[4]);


        printk("\n");        
}

#ifdef CONFIG_PREEMPT
#define S_PREEMPT " PREEMPT"
#else
#define S_PREEMPT ""
#endif
#ifdef CONFIG_SMP
#define S_SMP " SMP"
#else
#define S_SMP ""
#endif

static int __die(const char *str, int err, struct thread_info *thread,
		 struct pt_regs *regs)
{
	struct task_struct *tsk = thread->task;
	static int die_counter;
	int ret;

	pr_emerg("Internal error: %s: %x [#%d]" S_PREEMPT S_SMP "\n",
		 str, err, ++die_counter);

	/* trap and error numbers are mostly meaningless */
	ret = notify_die(DIE_OOPS, str, regs, err, 0, SIGSEGV);
	if (ret == NOTIFY_STOP)
		return ret;

	print_modules();
	printk("\n%s#: %04x\n", str, err & 0xffff);
	show_registers(regs);
	pr_emerg("Process %.*s (pid: %d, stack limit = 0x%p)\n",
		 TASK_COMM_LEN, tsk->comm, task_pid_nr(tsk), thread + 1);

	if (!user_mode(regs) || in_interrupt()) {
		dump_instr(KERN_EMERG, regs);
		dump_backtrace(regs, tsk);
	}

	return ret;
}

static DEFINE_RAW_SPINLOCK(die_lock);

/*
 * This function is protected against re-entrancy.
 */
void die(const char *str, struct pt_regs *regs, int err)
{
	struct thread_info *thread = current_thread_info();
	int ret;

	oops_enter();

	raw_spin_lock_irq(&die_lock);
	console_verbose();
	bust_spinlocks(1);
	ret = __die(str, err, thread, regs);

	if (regs && kexec_should_crash(thread->task))
		crash_kexec(regs);

	bust_spinlocks(0);
	add_taint(TAINT_DIE, LOCKDEP_NOW_UNRELIABLE);
	raw_spin_unlock_irq(&die_lock);
	oops_exit();

	if (in_interrupt())
		panic("Fatal exception in interrupt");
	if (panic_on_oops)
		panic("Fatal exception");
	if (ret != NOTIFY_STOP)
		do_exit(SIGSEGV);
}
void die_if_kernel(const char *str, struct pt_regs *regs, int err)
{
	if (!user_mode(regs))
		die(str, regs, err);
}
void gpt_notify_die(const char *str, struct pt_regs *regs,
		      struct siginfo *info, int err)
{
	if (user_mode(regs)) {
		current->thread.fault_address = 0;
		current->thread.fault_code = err;
		force_sig_info(info->si_signo, info, current);
	} else {
		die(str, regs, err);
	}
}

void __init trap_init(void)
{
}

