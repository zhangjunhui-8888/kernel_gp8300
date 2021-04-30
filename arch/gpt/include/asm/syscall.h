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

#ifndef __ASM_GPT_SYSCALL_H__
#define __ASM_GPT_SYSCALL_H__

#include <uapi/linux/audit.h>
#include <linux/err.h>
#include <linux/sched.h>

extern const void *sys_call_table[];

static inline int
syscall_get_nr(struct task_struct *task, struct pt_regs *regs)
{
	return regs->syscallno;
}

static inline void
syscall_rollback(struct task_struct *task, struct pt_regs *regs)
{
	regs->r_regs[8] = regs->r8_orig;
}

static inline long
syscall_get_error(struct task_struct *task, struct pt_regs *regs)
{
	unsigned long error = regs->r_regs[8];
        return IS_ERR_VALUE(error) ? error : 0;
}

static inline long
syscall_get_return_value(struct task_struct *task, struct pt_regs *regs)
{
        return regs->r_regs[8];
}

static inline void
syscall_set_return_value(struct task_struct *task, struct pt_regs *regs,
                         int error, long val)
{
	regs->r_regs[8] = (long) error ? error : val;
}

#define SYSCALL_MAX_ARGS 6

static inline void
syscall_get_arguments(struct task_struct *task, struct pt_regs *regs,
                      unsigned int i, unsigned int n, unsigned long *args)
{
	if (n == 0)
		return;

	if (i + n > SYSCALL_MAX_ARGS) {
		unsigned long *args_bad = args + SYSCALL_MAX_ARGS - i;
		unsigned int n_bad = n + i - SYSCALL_MAX_ARGS;
		pr_warning("%s called with max args %d, handling only %d\n",
			   __func__, i + n, SYSCALL_MAX_ARGS);
		memset(args_bad, 0, n_bad * sizeof(args[0]));
	}

	if (i == 0) {
		args[0] = regs->r8_orig;
		args++;
		i++;
		n--;
	}

	memcpy(args, &regs->r_regs[8+i], n * sizeof(args[0]));
}

static inline void
syscall_set_arguments(struct task_struct *task, struct pt_regs *regs,
                      unsigned int i, unsigned int n, const unsigned long *args)
{
	if (n == 0)
		return;

	if (i + n > SYSCALL_MAX_ARGS) {
		pr_warning("%s called with max args %d, handling only %d\n",
			   __func__, i + n, SYSCALL_MAX_ARGS);
		n = SYSCALL_MAX_ARGS - i;
	}

	if (i == 0) {
		regs->r8_orig = args[0];
		args++;
		i++;
		n--;
	}

	memcpy(&regs->r_regs[8+i], args, n * sizeof(args[0]));
}

static inline int syscall_get_arch(void)
{
        return AUDIT_ARCH_GPT;
}
#endif /* __ASM_GPT_SYSCALL_H__ */
