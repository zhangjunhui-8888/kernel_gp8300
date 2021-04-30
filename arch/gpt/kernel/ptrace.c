/*
 * gpt ptrace.c
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
 */

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/string.h>

#include <linux/mm.h>
#include <linux/errno.h>
#include <linux/ptrace.h>
#include <linux/audit.h>
#include <linux/regset.h>
#include <linux/tracehook.h>
#include <linux/elf.h>
#include <linux/ftrace.h>
#include <linux/perf_event.h>
#include <linux/hw_breakpoint.h>
#include <linux/elf.h>

#include <asm/thread_info.h>
#include <asm/segment.h>
#include <asm/page.h>
#include <asm/pgtable.h>

#define CREATE_TRACE_POINTS
#include <trace/events/syscalls.h>

void ptrace_disable(struct task_struct *child){}
/*
 * TODO: optimize struct pt_regs.
 */
static int gpr_get(struct task_struct *target,
		   const struct user_regset *regset,
		   unsigned int pos, unsigned int count,
		   void *kbuf, void __user *ubuf)
{
	struct user_pt_regs *uregs = &task_pt_regs(target)->user_regs;
	/* GDB will use a0 as cfa when the debug function is a leaf,
	 * but exception entry not saved a0, so here retore a0(a0 == sp) */
	uregs->a_regs[0] = uregs->sp;
	return  user_regset_copyout(&pos, &count, &kbuf, &ubuf, uregs, 0, -1);
}

static int gpr_set(struct task_struct *target, const struct user_regset *regset,
		   unsigned int pos, unsigned int count,
		   const void *kbuf, const void __user *ubuf)
{
	int ret;
	struct user_pt_regs newregs;
	struct pt_regs *regs= task_pt_regs(target);

	ret = user_regset_copyin(&pos, &count, &kbuf, &ubuf, &newregs, 0, -1);
	if (ret)
		return ret;

	memcpy((struct user_pt_regs*)&regs, &newregs, sizeof(struct user_pt_regs));

	return 0;
}
/*
 * TODO: update fp accessors for lazy context switching (sync/flush hwstate)
 */
static int fpr_get(struct task_struct *target, const struct user_regset *regset,
		   unsigned int pos, unsigned int count,
		   void *kbuf, void __user *ubuf)
{
	struct user_fpu_state *uregs;
	uregs = &target->thread.vfp_state.user_fpu_state;
	return user_regset_copyout(&pos, &count, &kbuf, &ubuf, uregs, 0, -1);
}

static int fpr_set(struct task_struct *target, const struct user_regset *regset,
		   unsigned int pos, unsigned int count,
		   const void *kbuf, const void __user *ubuf)
{
	int ret;
	struct user_fpu_state newstate;

	ret = user_regset_copyin(&pos, &count, &kbuf, &ubuf, &newstate, 0, -1);
	if (ret)
		return ret;

	target->thread.vfp_state.user_fpu_state = newstate;
	vfp_flush_task_state(target);
	return ret;
}

static int tls_get(struct task_struct *target, const struct user_regset *regset,
		   unsigned int pos, unsigned int count,
		   void *kbuf, void __user *ubuf)
{
	unsigned long *tls = &target->thread.tp_value;
	return user_regset_copyout(&pos, &count, &kbuf, &ubuf, tls, 0, -1);
}

static int tls_set(struct task_struct *target, const struct user_regset *regset,
		   unsigned int pos, unsigned int count,
		   const void *kbuf, const void __user *ubuf)
{
	int ret;
	unsigned long tls;

	ret = user_regset_copyin(&pos, &count, &kbuf, &ubuf, &tls, 0, -1);
	if (ret)
		return ret;

	target->thread.tp_value = tls;
	return ret;
}

static int vpr_get(struct task_struct *target, const struct user_regset *regset,
		unsigned int pos, unsigned int count,
		void *kbuf, void __user *ubuf)
{
	struct user_vpu_state *vregs;
	vregs = &target->thread.vfp_state.user_vpu_state;
	return user_regset_copyout(&pos, &count, &kbuf, &ubuf, vregs, 0, -1);
}

static int vpr_set(struct task_struct *target, const struct user_regset *regset,
		unsigned int pos, unsigned int count,
		const void *kbuf, const void __user *ubuf)
{
	int ret;
	struct user_vpu_state *newstate;

	newstate = kmalloc(sizeof(*newstate), GFP_KERNEL);
	ret = user_regset_copyin(&pos, &count, &kbuf, &ubuf, &newstate, 0, -1);
	if (ret)
		return ret;

	target->thread.vfp_state.user_vpu_state = *newstate;
	vfp_flush_task_state(target);
	return ret;
}

enum gpt_regset {
	REGSET_GPR,
	REGSET_FPR,
	REGSET_TLS,
	REGSET_VPR,
#ifdef	CONFIG_HAVE_HW_BREAKPOINT
	REGSET_HW_BRK,
	REGSET_HW_WTP,
#endif
};

#ifdef CONFIG_HAVE_HW_BREAKPOINT
/*
 * Handle hitting a HW-breakpoint/HW-watchpoint.
 */
static void ptrace_hbptriggered(struct perf_event *bp,
		struct perf_sample_data *data,
		struct pt_regs *regs)
{
	struct arch_hw_breakpoint *bkpt = counter_arch_bp(bp);
	siginfo_t info = {
		.si_signo       = SIGTRAP,
		.si_errno       = 0,
		.si_code        = TRAP_HWBKPT,
		.si_addr        = (void __user *)(bkpt->trigger),
	};
	force_sig_info(SIGTRAP, &info, current);
}

/*
 * Unregister breakpoints from this task and reset the pointers in
 * the thread_struct.
 */
void flush_ptrace_hw_breakpoint(struct task_struct *tsk)
{
	int i;
	struct thread_struct *t = &tsk->thread;

	for (i = 0; i < GPTX_MAX_BRP; i++) {
		if (t->debug.hbp_breakp[i]) {
			unregister_hw_breakpoint(t->debug.hbp_breakp[i]);
			t->debug.hbp_breakp[i] = NULL;
		}
	}

	for (i = 0; i < GPTX_MAX_WTP; i++) {
		if (t->debug.hbp_watchp[i]) {
			unregister_hw_breakpoint(t->debug.hbp_watchp[i]);
			t->debug.hbp_watchp[i] = NULL;
		}
	}
}

void ptrace_hw_copy_thread(struct task_struct *tsk)
{
	memset(&tsk->thread.debug, 0, sizeof(struct debug_info));
}

static struct perf_event *ptrace_hbp_get_event(unsigned int note_type,
				struct task_struct *tsk, unsigned long idx)
{
	struct perf_event *bp = ERR_PTR(-EINVAL);

	switch (note_type) {
		case NT_GPTX_HW_BREAK:
			if (idx < GPTX_MAX_BRP)
				bp = tsk->thread.debug.hbp_breakp[idx];
			break;
		case NT_GPTX_HW_WATCH:
			if (idx < GPTX_MAX_WTP)
				bp = tsk->thread.debug.hbp_watchp[idx];
			break;
	}

	return bp;
}

static int ptrace_hbp_set_event(unsigned int note_type, struct task_struct *tsk,
				unsigned long idx, struct perf_event *bp)
{
	int err = -EINVAL;

	switch (note_type) {
		case NT_GPTX_HW_BREAK:
			if (idx < GPTX_MAX_BRP) {
				tsk->thread.debug.hbp_breakp[idx] = bp;
				err = 0;
			}
			break;
		case NT_GPTX_HW_WATCH:
			if (idx < GPTX_MAX_WTP) {
				tsk->thread.debug.hbp_watchp[idx] = bp;
				err = 0;
			}
			break;
	}

	return err;
}

static struct perf_event *ptrace_hbp_create(unsigned int note_type,
				struct task_struct *tsk, unsigned long idx)
{
	struct perf_event *bp;
	struct perf_event_attr attr;
	int err, type;

	switch (note_type) {
		case NT_GPTX_HW_BREAK:
			type = HW_BREAKPOINT_X;
			break;
		case NT_GPTX_HW_WATCH:
			type = HW_BREAKPOINT_RW;
			break;
		default:
			return ERR_PTR(-EINVAL);
	}

	ptrace_breakpoint_init(&attr);

	/*
	 * Initialise fields to sane defaults
	 * (i.e. values that will pass validation).
	 */
	attr.bp_addr    = 0;
	attr.bp_len     = HW_BREAKPOINT_LEN_4;
	attr.bp_type    = type;
	attr.disabled   = 1;
	/* current not support */
	attr.exclude_kernel = 0;

	bp = register_user_hw_breakpoint(&attr, ptrace_hbptriggered, NULL, tsk);
	if (IS_ERR(bp))
		return bp;

	err = ptrace_hbp_set_event(note_type, tsk, idx, bp);
	if (err)
		return ERR_PTR(err);

	return bp;
}

static struct perf_event *ptrace_hbp_get_initialised_bp(unsigned int note_type,
					struct task_struct *tsk, unsigned long idx)
{
	struct perf_event *bp = ptrace_hbp_get_event(note_type, tsk, idx);

	if (!bp)
		bp = ptrace_hbp_create(note_type, tsk, idx);

	return bp;
}

static int ptrace_hbp_fill_attr_ctrl(unsigned int note_type,
				     u64 ctrl, u64 mask, struct perf_event_attr *attr)
{
	int err, type, disabled = !ctrl;

	attr->disabled = disabled;
	if (disabled)
		return 0;

	err = arch_bp_generic_type(note_type == NT_GPTX_HW_BREAK ? 1 : 0, ctrl, &type);
	if (err)
		return err;

	switch (note_type) {
		case NT_GPTX_HW_BREAK:
			if ((type & HW_BREAKPOINT_X) != type)
				return -EINVAL;
			break;
		case NT_GPTX_HW_WATCH:
			if ((type & HW_BREAKPOINT_RW) != type)
				return -EINVAL;
			break;
		default:
			return -EINVAL;
	}

	attr->bp_len    = mask;
	attr->bp_type   = type;

	return 0;
}

static int ptrace_hbp_get_ctrl(unsigned int note_type,
						struct task_struct *tsk,
						unsigned long idx, u32 *ctrl)
{
	struct perf_event *bp = ptrace_hbp_get_event(note_type, tsk, idx);

	if (IS_ERR(bp))
		return PTR_ERR(bp);

	*ctrl = bp ? (counter_arch_bp(bp)->ctrl).type : 0;

	return 0;
}

static int ptrace_hbp_get_mask(unsigned int note_type,
						struct task_struct *tsk,
						unsigned long idx, u64 *ctrl)
{
	struct perf_event *bp = ptrace_hbp_get_event(note_type, tsk, idx);

	if (IS_ERR(bp))
		return PTR_ERR(bp);

	*ctrl = bp ? (counter_arch_bp(bp)->ctrl).len: 0;

	return 0;
}


static int ptrace_hbp_set_ctrl(unsigned int note_type,
				struct task_struct *tsk,
				unsigned long idx,
				u64 ctrl, u64 mask)
{
	int err;
	struct perf_event *bp;
	struct perf_event_attr attr;

	bp = ptrace_hbp_get_initialised_bp(note_type, tsk, idx);
	if (IS_ERR(bp)) {
		err = PTR_ERR(bp);
		return err;
	}

	attr = bp->attr;
	err = ptrace_hbp_fill_attr_ctrl(note_type, ctrl, mask, &attr);
	if (err)
		return err;

	return modify_user_hw_breakpoint(bp, &attr);
}

static int ptrace_hbp_get_addr(unsigned int note_type,
			       struct task_struct *tsk,
			       unsigned long idx,
			       u64 *addr)
{
	struct perf_event *bp = ptrace_hbp_get_event(note_type, tsk, idx);

	if (IS_ERR(bp))
		return PTR_ERR(bp);

	*addr = bp ? bp->attr.bp_addr : 0;
	return 0;
}

static int ptrace_hbp_set_addr(unsigned int note_type,
				struct task_struct *tsk,
				unsigned long idx,
				u64 addr)
{
	int err;
	struct perf_event *bp;
	struct perf_event_attr attr;

	bp = ptrace_hbp_get_initialised_bp(note_type, tsk, idx);
	if (IS_ERR(bp)) {
		err = PTR_ERR(bp);
		return err;
	}

	attr = bp->attr;
	attr.bp_addr = addr;
	err = modify_user_hw_breakpoint(bp, &attr);
	return err;
}

#define	PTRACE_HBP_CTRL_SZ	(sizeof(u64))
#define	PTRACE_HBP_ADDR_SZ	(sizeof(u64))
#define	PTRACE_HBP_MASK_SZ	(sizeof(u64))
#define	PTRACE_HBP_PAD_SZ	(sizeof(u32))

static int hw_break_get(struct task_struct *target, const struct user_regset *regset,
			unsigned int pos, unsigned int count,
			void *kbuf, void __user *ubuf)
{
	unsigned int note_type = regset->core_note_type;
	int ret, idx = 0, offset, limit;
	struct user_hwdebug_state  *state;
	u32 info, ctrl;
	u64 addr, mask;

	/* bp/wp numbers, here is hard code */
	info = 2;
	ret = user_regset_copyout(&pos, &count, &kbuf, &ubuf, &info, 0,
			sizeof(info));
	if (ret)
		return ret;

	offset = offsetof(struct user_hwdebug_state, pad);
	ret = user_regset_copyout_zero(&pos, &count, &kbuf, &ubuf, offset,
			offset + PTRACE_HBP_PAD_SZ);
	if (ret)
		return ret;
	limit = sizeof(*(state->dm_regs));
	if (note_type == NT_GPTX_HW_BREAK) {
		offset = offsetof(struct user_hwdebug_state, im_regs);
		limit = sizeof(*(state->im_regs));
	} else
		offset = offsetof(struct user_hwdebug_state, dm_regs);

	limit = limit * GPTX_MAX_BRP;
	while (count && offset < limit) {
		ret = ptrace_hbp_get_ctrl(note_type, target, idx, &ctrl);
		if (ret)
			return ret;
		ret = user_regset_copyout(&pos, &count, &kbuf, &ubuf, &ctrl,
				offset, offset + PTRACE_HBP_CTRL_SZ);
		if (ret)
			return ret;
		offset += PTRACE_HBP_CTRL_SZ;

		ret = ptrace_hbp_get_mask(note_type, target, idx, &mask);
		if (ret)
			return ret;

		ret = user_regset_copyout(&pos, &count, &kbuf, &ubuf, &mask,
				offset, offset + PTRACE_HBP_MASK_SZ);
		if (ret)
			return ret;
		offset += PTRACE_HBP_MASK_SZ;

		ret = ptrace_hbp_get_addr(note_type, target, idx, &addr);
		if (ret)
			return ret;
		ret = user_regset_copyout(&pos, &count, &kbuf, &ubuf, &addr,
				offset, offset + PTRACE_HBP_ADDR_SZ);
		if (ret)
			return ret;
		offset += PTRACE_HBP_ADDR_SZ;

		idx++;
	}
	return 0;
}

static int hw_break_set(struct task_struct *target, const struct user_regset *regset,
			unsigned int pos, unsigned int count,
			const void *kbuf, const void __user *ubuf)
{
	unsigned int note_type = regset->core_note_type;
	int ret, idx = 0, offset, limit;
	u64 addr, ctrl, mask;
	struct user_hwdebug_state  *state;

	/* Resource info and pad */
	limit = sizeof(*(state));
	if (note_type == NT_GPTX_HW_BREAK) {
		offset = offsetof(struct user_hwdebug_state, im_regs);
		limit = sizeof(*(state->im_regs));
	} else
		offset = offsetof(struct user_hwdebug_state, dm_regs);
	ret = user_regset_copyin_ignore(&pos, &count, &kbuf, &ubuf, 0, offset);
	if (ret)
		return ret;

	/* (address, ctrl) registers */
	limit = limit * GPTX_MAX_BRP;
	while (count && offset < limit) {
		/* get ctrl */
		ret = user_regset_copyin(&pos, &count, &kbuf, &ubuf, &ctrl,
				offset, offset + PTRACE_HBP_CTRL_SZ);
		if (ret)
			return ret;
		offset += PTRACE_HBP_CTRL_SZ;

		/* get mask */
		ret = user_regset_copyin(&pos, &count, &kbuf, &ubuf, &mask,
				offset, offset + PTRACE_HBP_MASK_SZ);
		if (ret)
			return ret;

		ret = ptrace_hbp_set_ctrl(note_type, target, idx, ctrl, mask);
		if (ret)
			return ret;
		offset += PTRACE_HBP_MASK_SZ;

		ret = user_regset_copyin(&pos, &count, &kbuf, &ubuf, &addr,
				offset, offset + sizeof(u64));
		if (ret)
			return ret;
		ret = ptrace_hbp_set_addr(note_type, target, idx, addr);
		if (ret)
			return ret;
		offset += PTRACE_HBP_ADDR_SZ;

		idx++;
	}

	return 0;
}
#endif

static const struct user_regset gpt_regsets[] = {
	[REGSET_GPR] = {
		.core_note_type = NT_PRSTATUS,
		.n = sizeof(struct user_pt_regs) / sizeof(u64),
		.size = sizeof(u64),
		.align = sizeof(u64),
		.get = gpr_get,
		.set = gpr_set
	},
	[REGSET_FPR] = {
		.core_note_type = NT_PRFPREG,
		.n = sizeof(struct user_fpu_state) / sizeof(u64),
		.size = sizeof(u64),
		.align = sizeof(u64),
		.get = fpr_get,
		.set = fpr_set
	},
	[REGSET_TLS] = {
		.core_note_type = NT_GPTX_TLS,
		.n = 1,
		.size = sizeof(void *),
		.align = sizeof(void *),
		.get = tls_get,
		.set = tls_set,
	},
#if 1
	[REGSET_VPR] = {
		.core_note_type = NT_GPTX_VFP,
		.n = sizeof(struct user_vpu_state) / sizeof(u64),
		.size = sizeof(u64),
		.align = sizeof(u64),
		.get = vpr_get,
		.set = vpr_set
	},
#endif
#ifdef	CONFIG_HAVE_HW_BREAKPOINT
	[REGSET_HW_BRK] = {
		.core_note_type = NT_GPTX_HW_BREAK,
		.n = sizeof(struct user_hwdebug_state) / sizeof(u64),
		.size = sizeof(u64),
		.align = sizeof(u64),
		.get = hw_break_get,
		.set = hw_break_set,
	},
	[REGSET_HW_WTP] = {
		.core_note_type = NT_GPTX_HW_WATCH,
		.n = sizeof(struct user_hwdebug_state) / sizeof(u64),
		.size = sizeof(u64),
		.align = sizeof(u64),
		.get = hw_break_get,
		.set = hw_break_set,
	},
#endif
};

static const struct user_regset_view user_gpt_view = {
	.name = "gpt", .e_machine = EM_GPT,
	.regsets = gpt_regsets, .n = ARRAY_SIZE(gpt_regsets)
};

const struct user_regset_view *task_user_regset_view(struct task_struct *task)
{
	return &user_gpt_view;
}
long arch_ptrace(struct task_struct *child, long request,
		 unsigned long addr, unsigned long data)
{
	return ptrace_request(child, request, addr, data);
}

/*
 * handle tracing of system call entry
 * - return the revised system call number or ULONG_MAX to cause ENOSYS
 */
asmlinkage long syscall_trace_enter(struct pt_regs *regs)
{
	if (test_thread_flag(TIF_SYSCALL_TRACE) &&
	    tracehook_report_syscall_entry(regs)) {
		regs->syscallno = -1;
	}

#ifdef CONFIG_HAVE_SYSCALL_TRACEPOINTS
	if (unlikely(test_thread_flag(TIF_SYSCALL_TRACEPOINT))) {
		trace_sys_enter(regs, regs->syscallno);
	}
#endif

	audit_syscall_entry(regs->syscallno, regs->r8_orig, regs->r_regs[9],
			    regs->r_regs[10], regs->r_regs[11]);

	return regs->syscallno;
}

/*
 * handle tracing of system call exit
 */
asmlinkage void syscall_trace_exit(struct pt_regs *regs)
{
	audit_syscall_exit(regs);

#ifdef CONFIG_HAVE_SYSCALL_TRACEPOINTS
	if (unlikely(test_thread_flag(TIF_SYSCALL_TRACEPOINT))) {
		trace_sys_exit(regs, regs_return_value(regs));
	}
#endif

	if (test_thread_flag(TIF_SYSCALL_TRACE)) {
		tracehook_report_syscall_exit(regs, 0);
	}
}
