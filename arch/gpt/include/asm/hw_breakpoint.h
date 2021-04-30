/*
 * Copyright (C) 2019 GPT Ltd.
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

#ifndef __ASM_HW_BREAKPOINT_H
#define __ASM_HW_BREAKPOINT_H

#ifdef __KERNEL__

struct arch_hw_breakpoint_ctrl {
	u32 __reserved  : 20,
	len		: 8,
	type		: 2,
	level		: 1,
	enabled		: 1;
};

struct arch_hw_breakpoint {
	u64 address;
	u64 trigger;
	struct arch_hw_breakpoint_ctrl ctrl;
};

static inline u32 encode_ctrl_reg(struct arch_hw_breakpoint_ctrl ctrl)
{
	return ctrl.type;
}

static inline void decode_ctrl_reg(u32 reg,
				   struct arch_hw_breakpoint_ctrl *ctrl)
{
	ctrl->enabled   = reg & 0x1;
	reg >>= 1;
	ctrl->level	= reg & 0x1;
	reg >>= 1;
	ctrl->type      = reg & 0x3;
	reg >>= 2;
	ctrl->len       = reg & 0xff;
}


#define	GPTX_MAX_BRP	2
#define	GPTX_MAX_WTP	2

/* Breakpoint */
#define GPTX_BREAKPOINT_EXECUTE	0

/* Watchpoints */
#define	GPTX_BREAKPOINT_LOAD	1
#define	GPTX_BREAKPOINT_STORE	2


/* Lengths */
#define GPTX_BREAKPOINT_LEN_1    0x1
#define GPTX_BREAKPOINT_LEN_2    0x3
#define GPTX_BREAKPOINT_LEN_4    0xf
#define GPTX_BREAKPOINT_LEN_8    0xff

struct task_struct;
struct notifier_block;
struct perf_event;
struct pmu;

int arch_install_hw_breakpoint(struct perf_event *bp);
extern int arch_bp_generic_type(u32 is_wp, u64 ctrl,
			   int *gen_type);
extern int arch_check_bp_in_kernelspace(struct perf_event *bp);
extern int arch_validate_hwbkpt_settings(struct perf_event *bp);
extern int hw_breakpoint_exceptions_notify(struct notifier_block *unused,
		                                           unsigned long val, void *data);
extern void arch_uninstall_hw_breakpoint(struct perf_event *bp);
extern void hw_breakpoint_pmu_read(struct perf_event *bp);
extern int hw_breakpoint_slots(int type);

#ifdef CONFIG_HAVE_HW_BREAKPOINT
extern void hw_breakpoint_thread_switch(struct task_struct *next);
extern void ptrace_hw_copy_thread(struct task_struct *task);
#else
static inline void hw_breakpoint_thread_switch(struct task_struct *next)
{
}
static inline void ptrace_hw_copy_thread(struct task_struct *task)
{
}
#endif


#endif /* __KERNEL__ */
#endif /* __ASM_HW_BREAKPOINT_H */
