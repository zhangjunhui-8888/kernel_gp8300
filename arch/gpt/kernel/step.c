#include <linux/cpu.h>
#include <linux/debugfs.h>
#include <linux/hardirq.h>
#include <linux/init.h>
#include <linux/ptrace.h>
#include <linux/stat.h>
#include <linux/uaccess.h>

#include <asm/cacheflush.h>

static void set_regs_single_step(struct pt_regs *reg)
{
	reg->sr = reg->sr | (1UL << PSC_OFFSE_debug_ss);
}

static void set_regs_disable_single_step(struct pt_regs *reg)
{
	reg->sr = reg->sr & (~(1UL << PSC_OFFSE_debug_ss));
}

void user_enable_single_step(struct task_struct *task)
{
	set_ti_thread_flag(task_thread_info(task), TIF_SINGLESTEP);
	set_regs_single_step(task_pt_regs(task));
}

void user_disable_single_step(struct task_struct *task)
{
	clear_ti_thread_flag(task_thread_info(task), TIF_SINGLESTEP);
	set_regs_disable_single_step(task_pt_regs(task));
}


/* Re-enable single step for syscall restarting. */
void user_rewind_single_step(struct task_struct *task)
{
	if (test_ti_thread_flag(task_thread_info(task), TIF_SINGLESTEP))
		user_enable_single_step(task);
}

extern int wp_flag;
static int single_step_handler(struct pt_regs *regs, unsigned long ixi)
{
	siginfo_t info;
	struct debug_info *debug_info = &current->thread.debug;

	if (user_mode(regs)) {
		info.si_signo = SIGTRAP;
		info.si_errno = 0;
		info.si_code  = TRAP_HWBKPT;
		if (debug_info->wps_disabled) {
			info.si_addr = (void __user *)debug_info->wp_addr;
			debug_info->wps_disabled = 0;
		}else
			info.si_addr  = (void __user *)instruction_pointer(regs);
		force_sig_info(SIGTRAP, &info, current);

		/*
		 * ptrace will disable single step unless explicitly
		 * asked to re-enable it. For other clients, it makes
		 * sense to leave it enabled (i.e. rewind the controls
		 * to the active-not-pending state).
		 */
		user_rewind_single_step(current);
	}

	return 0;
}

static int __init arch_hw_breakpoint_init(void)
{
	hook_fault_code(IX_DBG_STEP, single_step_handler, SIGTRAP,
			      TRAP_HWBKPT, "single-step handler");
	return 0;
}
arch_initcall(arch_hw_breakpoint_init)
