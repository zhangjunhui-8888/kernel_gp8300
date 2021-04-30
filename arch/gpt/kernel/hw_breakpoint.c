/*
 * HW_breakpoint: a unified kernel/user-space hardware breakpoint facility,
 * using the CPU's debug registers.
 *
 * Copyright (C) 2019 GPT Limited
 * Author: Ron Ren <jjren@hxgpt.com>
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

#include <linux/compat.h>
#include <linux/cpu_pm.h>
#include <linux/errno.h>
#include <linux/hw_breakpoint.h>
#include <linux/perf_event.h>
#include <linux/ptrace.h>
#include <linux/smp.h>

#include <asm/current.h>
#include <asm/hw_breakpoint.h>

#define	GPTX_BRP_NUM	2
#define	GPTX_WRP_NUM	2

/* Breakpoint currently in use for each breakpoint. */
static DEFINE_PER_CPU(struct perf_event *, bp_reg[GPTX_BRP_NUM]);

/* Watchpoint currently in use for each watchpoint. */
static DEFINE_PER_CPU(struct perf_event *, wp_reg[GPTX_WRP_NUM]);

/* Currently stepping a per-CPU kernel breakpoint. */
static DEFINE_PER_CPU(int, stepping_kernel_bp);

/* Number of BRP/WRP registers on this CPU. */
static int core_num_brps;
static int core_num_wrps;

/* Determine number of breakpoint registers available. */
static int get_num_brps(void)
{
	return GPTX_BRP_NUM;
}

/* Determine number of watchpoint registers available. */
static int get_num_wrps(void)
{
	return GPTX_WRP_NUM;
}

int hw_breakpoint_slots(int type)
{
	/*
	 * We can be called early, so don't rely on
	 * our static variables being initialised.
	 */
	switch (type) {
	case TYPE_INST:
		return get_num_brps();
	case TYPE_DATA:
		return get_num_wrps();
	default:
		pr_warning("unknown slot type: %d\n", type);
		return 0;
	}
}

enum hw_breakpoint_ops {
	HW_BREAKPOINT_INSTALL,
	HW_BREAKPOINT_UNINSTALL,
	HW_BREAKPOINT_RESTORE
};

/**
 * hw_breakpoint_slot_setup - Find and setup a perf slot according to
 *			      operations
 *
 * @slots: pointer to array of slots
 * @max_slots: max number of slots
 * @bp: perf_event to setup
 * @ops: operation to be carried out on the slot
 *
 * Return:
 *	slot index on success
 *	-ENOSPC if no slot is available/matches
 *	-EINVAL on wrong operations parameter
 */
static int hw_breakpoint_slot_setup(struct perf_event **slots, int max_slots,
				    struct perf_event *bp,
				    enum hw_breakpoint_ops ops)
{
	int i;
	struct perf_event **slot;

	for (i = 0; i < max_slots; ++i) {
		slot = &slots[i];
		switch (ops) {
		case HW_BREAKPOINT_INSTALL:
			if (!*slot) {
				*slot = bp;
				return i;
			}
			break;
		case HW_BREAKPOINT_UNINSTALL:
			if (*slot == bp) {
				*slot = NULL;
				return i;
			}
			break;
		case HW_BREAKPOINT_RESTORE:
			if (*slot == bp)
				return i;
			break;
		default:
			pr_warn_once("Unhandled hw breakpoint ops %d\n", ops);
			return -EINVAL;
		}
	}
	return -ENOSPC;
}

static int hw_breakpoint_control(struct perf_event *bp,
				 enum hw_breakpoint_ops ops)
{
	struct arch_hw_breakpoint *info = counter_arch_bp(bp);
	struct perf_event **slots;
	struct debug_info *debug_info = &current->thread.debug;
	int i, max_slots, reg_enable;
	u64 spr;
	struct pt_regs *regs = current_pt_regs();

	if (bp->attr.bp_type == HW_BREAKPOINT_X) {
		/* Breakpoint */
		slots = this_cpu_ptr(bp_reg);
		max_slots = core_num_brps;
		reg_enable = !debug_info->bps_disabled;
	} else {
		/* Watchpoint */
		slots = this_cpu_ptr(wp_reg);
		max_slots = core_num_wrps;
		reg_enable = !debug_info->wps_disabled;
	}

	i = hw_breakpoint_slot_setup(slots, max_slots, bp, ops);

	if (WARN_ONCE(i < 0, "Can't find any breakpoint slot"))
		return i;

	switch (ops) {
	case HW_BREAKPOINT_INSTALL:
		/*
		 * Ensure debug monitors are enabled at the correct exception
		 * level.
		 */
		if (bp->attr.bp_type == HW_BREAKPOINT_X) {
			spr = sprget_gpt(CPUC);
			spr |= (1UL << PSC_WIDTH_debug_im);
			regs->sr |= (1UL << PSC_OFFSE_debug_im);
		} else {
			spr = sprget_gpt(CPUC);
			spr |= (1UL << PSC_WIDTH_debug_dm);
			regs->sr |= (1UL << PSC_OFFSE_debug_dm);
		}
		sprset_gpt(CPUC, spr);
		/* Fall through */
	case HW_BREAKPOINT_RESTORE:
		/* Setup the address register. */
		if (bp->attr.bp_type == HW_BREAKPOINT_X) {
			if (i == 0) {
				/* IMxC */
				sprset_gpt(IM0C, info->address);
				sprset_gpt(IM0M, -1);
				sprset_gpt(IM0E, reg_enable ? 1 : 0);
			} else {
				/* IMxC */
				sprset_gpt(IM1C, info->address);
				sprset_gpt(IM1M, -1);
				sprset_gpt(IM1E, reg_enable ? 1 : 0);
			}
		} else {
			if (i == 0) {
				/* DMxC */
				sprset_gpt(DM0C, info->address);
				sprset_gpt(DM0M, -1);
				if (reg_enable)
					/* load/store */
					sprset_gpt(DM0E, info->ctrl.type);
				else
					sprset_gpt(DM0E, 0);
			} else {
				/* DMxC */
				sprset_gpt(DM1C, info->address);
				sprset_gpt(DM1M, -1);
				if (reg_enable)
					/* load/store */
					sprset_gpt(DM1E, info->ctrl.type);
				else
					sprset_gpt(DM1E, 0);
			}
		}
		break;
	case HW_BREAKPOINT_UNINSTALL:
#if 1
		/* Reset the control register. */
		if (bp->attr.bp_type == HW_BREAKPOINT_X) {
			if (i == 0)
				sprset_gpt(IM0E, 0);
			else
				sprset_gpt(IM1E, 0);
			regs->sr = regs->sr & (~(1UL << PSC_OFFSE_debug_im));
		} else {
			if (i == 0)
				sprset_gpt(DM0E, 0);
			else
				sprset_gpt(DM1E, 0);
			regs->sr = regs->sr & (~(1UL << PSC_OFFSE_debug_dm));
		}
#endif
		break;
	}

	return 0;
}

/*
 * Install a perf counter breakpoint.
 */
int arch_install_hw_breakpoint(struct perf_event *bp)
{
	return hw_breakpoint_control(bp, HW_BREAKPOINT_INSTALL);
}

void arch_uninstall_hw_breakpoint(struct perf_event *bp)
{
	hw_breakpoint_control(bp, HW_BREAKPOINT_UNINSTALL);
}

static int get_hbp_len(u8 hbp_len)
{
	unsigned int len_in_bytes = 0;

	switch (hbp_len) {
	case GPTX_BREAKPOINT_LEN_1:
		len_in_bytes = 1;
		break;
	case GPTX_BREAKPOINT_LEN_2:
		len_in_bytes = 2;
		break;
	case GPTX_BREAKPOINT_LEN_4:
		len_in_bytes = 4;
		break;
	case GPTX_BREAKPOINT_LEN_8:
		len_in_bytes = 8;
		break;
	}

	return len_in_bytes;
}

/*
 * Check whether bp virtual address is in kernel space.
 */
int arch_check_bp_in_kernelspace(struct perf_event *bp)
{
	unsigned int len;
	unsigned long va;
	struct arch_hw_breakpoint *info = counter_arch_bp(bp);

	va = info->address;
	len = get_hbp_len(info->ctrl.len);

	return (va >= TASK_SIZE) && ((va + len - 1) >= TASK_SIZE);
}

/*
 * Extract generic type and length encodings from an arch_hw_breakpoint_ctrl.
 * Hopefully this will disappear when ptrace can bypass the conversion
 * to generic breakpoint descriptions.
 */
int arch_bp_generic_type(u32 is_bp, u64 ctrl, int *gen_type)
{
	/* Type */
	switch (ctrl) {
	case GPTX_BREAKPOINT_EXECUTE:
		*gen_type = HW_BREAKPOINT_X;
		break;
	case GPTX_BREAKPOINT_LOAD:
		*gen_type = HW_BREAKPOINT_R;
		break;
	case GPTX_BREAKPOINT_STORE:
		*gen_type = HW_BREAKPOINT_W;
		break;
	case GPTX_BREAKPOINT_LOAD | GPTX_BREAKPOINT_STORE:
		*gen_type = HW_BREAKPOINT_RW;
		break;
	default:
		return -EINVAL;
	}
	if (is_bp)
		*gen_type = HW_BREAKPOINT_X;

	return 0;
}

/*
 * Construct an arch_hw_breakpoint from a perf_event.
 */
static int arch_build_bp_info(struct perf_event *bp)
{
	struct arch_hw_breakpoint *info = counter_arch_bp(bp);

	/* Type */
	switch (bp->attr.bp_type) {
	case HW_BREAKPOINT_X:
		info->ctrl.type = GPTX_BREAKPOINT_EXECUTE;
		break;
	case HW_BREAKPOINT_R:
		info->ctrl.type = GPTX_BREAKPOINT_LOAD;
		break;
	case HW_BREAKPOINT_W:
		info->ctrl.type = GPTX_BREAKPOINT_STORE;
		break;
	case HW_BREAKPOINT_RW:
		info->ctrl.type = GPTX_BREAKPOINT_LOAD | GPTX_BREAKPOINT_STORE;
		break;
	default:
		return -EINVAL;
	}

	/* Len */
	switch (bp->attr.bp_len) {
	case HW_BREAKPOINT_LEN_1:
		info->ctrl.len = GPTX_BREAKPOINT_LEN_1;
		break;
	case HW_BREAKPOINT_LEN_2:
		info->ctrl.len = GPTX_BREAKPOINT_LEN_2;
		break;
	case HW_BREAKPOINT_LEN_4:
		info->ctrl.len = GPTX_BREAKPOINT_LEN_4;
		break;
	case HW_BREAKPOINT_LEN_8:
		info->ctrl.len = GPTX_BREAKPOINT_LEN_8;
		break;
	default:
		return -EINVAL;
	}

	/* Address */
	info->address = bp->attr.bp_addr;

	/* Enabled? */
	info->ctrl.enabled = !bp->attr.disabled;

	return 0;
}

/*
 * Validate the arch-specific HW Breakpoint register settings.
 */
int arch_validate_hwbkpt_settings(struct perf_event *bp)
{
	struct arch_hw_breakpoint *info = counter_arch_bp(bp);
	int ret;

	/* Build the arch_hw_breakpoint. */
	ret = arch_build_bp_info(bp);
	if (ret)
		return ret;

	if (info->address & 0x3)
		return -EINVAL;

	return 0;
}

/*
 * Enable/disable all of the breakpoints active at the specified
 * exception level at the register level.
 * This is used when single-stepping after a breakpoint exception.
 */
static void toggle_bp_registers(int enable)
{
	int i, max_slots;
	u64 en;
	struct perf_event **slots;
	struct debug_info *debug_info;

	debug_info = &current->thread.debug;
	slots = this_cpu_ptr(bp_reg);
	max_slots = core_num_brps;

	for (i = 0; i < max_slots; ++i) {
		if (!slots[i])
			continue;

		en = counter_arch_bp(slots[i])->ctrl.enabled;
		if (en & (1UL << IM0E_OFFSE_en)) {
			if (enable) {
				if (i ==0) {
					sprset_gpt(IM0E, 1 << IM0E_OFFSE_en);
				} else {
					sprset_gpt(IM1E, 1 << IM0E_OFFSE_en);
				}
			} else {
				if (i == 0)
					sprset_gpt(IM0E, 0 << IM0E_OFFSE_en);
				else
					sprset_gpt(IM1E, 0 << IM0E_OFFSE_en);
			}
		}
	}
}

/*
 * Enable/disable all of the breakpoints active at the specified
 * exception level at the register level.
 * This is used when single-stepping after a breakpoint exception.
 */
static void toggle_wp_registers(int enable)
{
	int i, max_slots;
	u64 en;
	struct perf_event **slots;

	slots = this_cpu_ptr(bp_reg);
	max_slots = core_num_brps;

	for (i = 0; i < max_slots; ++i) {
		if (!slots[i])
			continue;

		en = counter_arch_bp(slots[i])->ctrl.enabled;
		if (en) {
			if (enable) {
				/* 01: load, 10: store, 11: load and store */
				if (i == 0) {
					sprset_gpt(DM0E, en);
				}else {
					sprset_gpt(DM1E, en);
				}
			} else {
				if (i == 0)
					sprset_gpt(DM0E, 0);
				else
					sprset_gpt(DM1E, 0);
			}
		}
	}
}

/*
 * Debug exception handlers.
 */
static int breakpoint_handler(struct pt_regs *regs, unsigned long user)
{
	int i, step = 0;
	u64 addr, val;
	struct perf_event *bp, **slots;
	struct debug_info *debug_info;

	slots = this_cpu_ptr(bp_reg);
	addr = instruction_pointer(regs);
	debug_info = &current->thread.debug;

	for (i = 0; i < core_num_brps; ++i) {
		rcu_read_lock();

		bp = slots[i];

		if (bp == NULL)
			goto unlock;

		/* Check if the breakpoint value matches. */
		if (i == 0) {
			val = sprget_gpt(IM0C);
		} else
			val = sprget_gpt(IM1C);
		if (val != (addr & ~0x3))
			goto unlock;

		counter_arch_bp(bp)->trigger = addr;
		perf_bp_event(bp, regs);

		/* Do we need to handle the stepping? */
		if (!bp->overflow_handler)
			step = 1;
unlock:
		rcu_read_unlock();
	}

	if (!step)
		return 0;

	if (user) {
		debug_info->bps_disabled = 1;
		toggle_bp_registers(0);
		regs->sr = regs->sr & (~(1UL << PSC_OFFSE_debug_im));

		/* If we're already stepping a watchpoint, just return. */
		if (debug_info->wps_disabled)
			return 0;

		if (test_thread_flag(TIF_SINGLESTEP))
			debug_info->suspended_step = 1;
		else
			user_enable_single_step(current);
	} else {
		return -1;
	}

	return 0;
}

static u64 dbg_virt2phys(u64 addr)
{
	u64 phys;
	pgd_t *pgd;
	struct mm_struct *mm = current->mm;
	pud_t *pud;
	pmd_t *pmd;
	pte_t *pte;

	if (!addr)
		return 0;

	if (!mm)
		mm = &init_mm;

	pgd = pgd_offset(mm, addr);

	if (pgd_none(*pgd) || pgd_bad(*pgd))
		return 0;

	pud = pud_offset(pgd, addr);
	if (pud_none(*pud) || pud_bad(*pud))
		return 0;

	pmd = pmd_offset(pud, addr);
	if (pmd_none(*pmd) || pmd_bad(*pmd))
		return 0;

	pte = pte_offset_map(pmd, addr);
	if (pte_none(*pte))
		return 0;
	phys = pte_val(*pte) & PAGE_MASK;
	phys |= addr & (~PAGE_MASK);

	return phys;
}
static int watchpoint_handler(struct pt_regs *regs, unsigned long user)
{
	int i;
	struct perf_event *wp, **slots;
	struct debug_info *debug_info;
	struct arch_hw_breakpoint *info;
	u64 addr, val = sprget_gpt(IXA1H);

	if (val == dbg_virt2phys(sprget_gpt(DM0C))) {
		addr = sprget_gpt(DM0C);
		i = 0;
	} else if (val == dbg_virt2phys(sprget_gpt(DM1C))) {
		addr = sprget_gpt(DM1C);
		i = 1;
	} else
		return -1;

	slots = this_cpu_ptr(wp_reg);
	debug_info = &current->thread.debug;

	for (i = 0; i < core_num_wrps; ++i) {
		rcu_read_lock();
		wp = slots[i];

		if (wp == NULL)
			goto unlock;

		info = counter_arch_bp(wp);
		if (val != (addr & ~7))
			goto unlock;
		info->trigger = addr;
		perf_bp_event(wp, regs);

unlock:
		rcu_read_unlock();
	}

	/*
	 * We always disable EL0 watchpoints because the kernel can
	 * cause these to fire via an unprivileged access.
	 */
	toggle_wp_registers(0);

	if (user) {
		debug_info->wps_disabled = 1;
		debug_info->wp_addr = addr;

		if (test_thread_flag(TIF_SINGLESTEP))
			debug_info->suspended_step = 1;
		else {
			user_enable_single_step(current);
		}
	} else
		return -1;

	return 0;
}

/*
 * Handle single-step exception.
 */
int reinstall_suspended_bps(struct pt_regs *regs)
{
	struct debug_info *debug_info = &current->thread.debug;
	int handled_exception = 0, *kernel_step;

	kernel_step = this_cpu_ptr(&stepping_kernel_bp);

	/*
	 * Called from single-step exception handler.
	 * Return 0 if execution can resume, 1 if a SIGTRAP should be
	 * reported.
	 */
	if (user_mode(regs)) {
		if (debug_info->bps_disabled) {
			debug_info->bps_disabled = 0;
			toggle_bp_registers(1);
			handled_exception = 1;
		}

		if (debug_info->wps_disabled) {
			debug_info->wps_disabled = 0;
			toggle_wp_registers(1);
			handled_exception = 1;
		}

		if (handled_exception) {
			if (debug_info->suspended_step) {
				debug_info->suspended_step = 0;
				/* Allow exception handling to fall-through. */
				handled_exception = 0;
			} else {
				user_disable_single_step(current);
			}
		}
	} else {
		handled_exception = 0;
	}

	return !handled_exception;
}

/*
 * Context-switcher for restoring suspended breakpoints.
 */
void hw_breakpoint_thread_switch(struct task_struct *next)
{
	/*
	 *           current        next
	 * disabled: 0              0     => The usual case, NOTIFY_DONE
	 *           0              1     => Disable the registers
	 *           1              0     => Enable the registers
	 *           1              1     => NOTIFY_DONE. per-task bps will
	 *                                   get taken care of by perf.
	 */

	struct debug_info *current_debug_info, *next_debug_info;

	current_debug_info = &current->thread.debug;
	next_debug_info = &next->thread.debug;

	/* Update breakpoints. */
	if (current_debug_info->bps_disabled != next_debug_info->bps_disabled)
		toggle_bp_registers(!next_debug_info->bps_disabled);

	/* Update watchpoints. */
	if (current_debug_info->wps_disabled != next_debug_info->wps_disabled)
		toggle_bp_registers(!next_debug_info->wps_disabled);
}

/*
 * CPU initialisation.
 */
static void hw_breakpoint_reset(void *unused)
{
	int i;
	/*
	 * When a CPU goes through cold-boot, it does not have any installed
	 * slot, so it is safe to share the same function for restoring and
	 * resetting breakpoints; when a CPU is hotplugged in, it goes
	 * through the slots, which are all empty, hence it just resets control
	 * and value for debug registers.
	 * When this function is triggered on warm-boot through a CPU PM
	 * notifier some slots might be initialized; if so they are
	 * reprogrammed according to the debug slots content.
	 */
	for (i = 0; i < core_num_brps; ++i) {
		if (i == 0) {
			sprset_gpt(IM0C, 0);
			sprset_gpt(IM0E, 0);
			sprset_gpt(IM0M, 0);
		} else {
			sprset_gpt(IM1C, 0);
			sprset_gpt(IM1E, 0);
			sprset_gpt(IM1M, 0);
		}
	}

	for (i = 0; i < core_num_wrps; ++i) {
		if (i == 0) {
			sprset_gpt(DM0C, 0);
			sprset_gpt(DM0E, 0);
			sprset_gpt(DM0M, 0);
		} else {
			sprset_gpt(DM1C, 0);
			sprset_gpt(DM1E, 0);
			sprset_gpt(DM1M, 0);
		}
	}
}

static int hw_breakpoint_reset_notify(struct notifier_block *self,
						unsigned long action,
						void *hcpu)
{
	int cpu = (long)hcpu;
	if (action == CPU_ONLINE)
		smp_call_function_single(cpu, hw_breakpoint_reset, NULL, 1);
	return NOTIFY_OK;
}

static struct notifier_block hw_breakpoint_reset_nb = {
	.notifier_call = hw_breakpoint_reset_notify,
};

/*
 * One-time initialisation.
 */
static int __init arch_hw_breakpoint_init(void)
{
	core_num_brps = get_num_brps();
	core_num_wrps = get_num_wrps();

	pr_info("found %d breakpoint and %d watchpoint registers.\n",
		core_num_brps, core_num_wrps);

	cpu_notifier_register_begin();

	/*
	 * Reset the breakpoint resources. We assume that a halting
	 * debugger will leave the world in a nice state for us.
	 */
	smp_call_function(hw_breakpoint_reset, NULL, 1);
	hw_breakpoint_reset(NULL);

	/* Register debug fault handlers. */
	hook_fault_code(IX_DBG_INSN, breakpoint_handler, SIGTRAP,
			      TRAP_HWBKPT, "hw-breakpoint handler");
	hook_fault_code(IX_DBG_DATA, watchpoint_handler, SIGTRAP,
			      TRAP_HWBKPT, "hw-watchpoint handler");

	/* Register hotplug notifier. */
	__register_cpu_notifier(&hw_breakpoint_reset_nb);

	cpu_notifier_register_done();

	return 0;
}
#if 1
arch_initcall(arch_hw_breakpoint_init);
#endif

void hw_breakpoint_pmu_read(struct perf_event *bp)
{
}

/*
 * Dummy function to register with die_notifier.
 */
int hw_breakpoint_exceptions_notify(struct notifier_block *unused,
				    unsigned long val, void *data)
{
	return NOTIFY_DONE;
}
