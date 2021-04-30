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

#ifndef __ASM_GPT_MMU_CONTEXT_H
#define __ASM_GPT_MMU_CONTEXT_H

#include <linux/sched.h>
#include <asm-generic/mm_hooks.h>
#include <asm/cacheflush.h>
#include <asm/tlbflush.h>
#include <asm/tlbhw.h>


#define ASID_BITS	8
#define ASID_MASK	(BIT(ASID_BITS)-1)
#define ASID(mm)	(mm->context.asid & ASID_MASK)
#define NO_CONTEXT      0UL
#define CURRPGD 	"$s0"

extern unsigned long cpu_last_asid;

void __init_new_context(struct task_struct *tsk, struct mm_struct *mm);
void __new_context(struct mm_struct *mm);
void cpu_switch_mm(struct mm_struct *mm);

static inline void switch_new_context(struct mm_struct *mm)
{
	unsigned long flags;

	__new_context(mm);
	local_irq_save(flags);

	cpu_switch_mm(mm);
	local_irq_restore(flags);
}

static inline void check_and_switch_context(struct mm_struct *mm,
					    struct task_struct *tsk)
{

	if (!((mm->context.asid ^ cpu_last_asid) >> ASID_BITS))
		/*
		 * The ASID is from the current generation, just switch to the
		 * new pgd. This condition is only true for calls from
		 * context_switch() and interrupts are already disabled.
		 */
		cpu_switch_mm(mm);
	else if (irqs_disabled())
		/*
		 * Defer the new ASID allocation until after the context
		 * switch critical region since __new_context() cannot be
		 * called with interrupts disabled.
		 */
		set_ti_thread_flag(task_thread_info(tsk), TIF_SWITCH_MM);
	else
		/*
		 * That is a direct call to switch_mm() or activate_mm() with
		 * interrupts enabled and a new context.
		 */
		switch_new_context(mm);
}

static inline int init_new_context(struct task_struct *tsk, struct mm_struct *mm)
{
	__init_new_context(tsk,mm);
	mm->context.cpu=-1;
	return 0;
}

#define destroy_context(mm)		do { } while(0)


#define finish_arch_post_lock_switch \
	finish_arch_post_lock_switch
static inline void finish_arch_post_lock_switch(void)
{
	if (test_and_clear_thread_flag(TIF_SWITCH_MM)) {
		struct mm_struct *mm = current->mm;
		unsigned long flags;

		__new_context(mm);

		local_irq_save(flags);
		cpu_switch_mm(mm);
		local_irq_restore(flags);
	}
}

/*
 * This is called when "tsk" is about to enter lazy TLB mode.
 *
 * mm:  describes the currently active mm context
 * tsk: task which is entering lazy tlb
 * cpu: cpu number which is entering lazy tlb
 *
 * tsk->mm will be NULL
 */
static inline void
enter_lazy_tlb(struct mm_struct *mm, struct task_struct *tsk)
{
}

/*
 * This is the actual mm switch as far as the scheduler
 * is concerned.  No registers are touched.  We avoid
 * calling the CPU specific function when the mm hasn't
 * actually changed.
 */
static inline void
switch_mm(struct mm_struct *prev, struct mm_struct *next,
	  struct task_struct *tsk)
{
	unsigned int cpu = smp_processor_id();
	unsigned long flags;
	/* Flush the icache if we migrated to a new core. */
	if (next->context.cpu != cpu) {
		local_irq_save(flags);
		local_flush_tlb_all();
		next->context.cpu = cpu;
		local_irq_restore(flags);
	}

	/*
	 * init_mm.pgd does not contain any user mappings
	 */
	if (next == &init_mm) {
		return;
	}

	if (!cpumask_test_and_set_cpu(cpu, mm_cpumask(next)) || prev != next)
		check_and_switch_context(next, tsk);
}

#define deactivate_mm(tsk,mm)	do { } while (0)
#define activate_mm(prev,next)	switch_mm(prev, next, NULL)

#endif
