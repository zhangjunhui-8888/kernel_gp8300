/*
 *  linux/arch/gpt/mm/context.c, refer to arm
 *
 *  Copyright (C) 2018, General Processor Techologies Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/ptrace.h>
#include <linux/mman.h>
#include <linux/mm.h>
#include <linux/init.h>

#include <asm/cpuinfo.h>
#include <asm/segment.h>
#include <asm/tlbflush.h>
#include <asm/cacheflush.h>
#include <asm/tlbhw.h>
#include <asm/mmu_context.h>

#define ASID_FIRST_VERSION	(1ULL << ASID_BITS)


static DEFINE_RAW_SPINLOCK(cpu_asid_lock);
unsigned long cpu_last_asid = ASID_FIRST_VERSION;


void cpu_switch_mm(struct mm_struct *mm)
{
	unsigned long flags;
	unsigned long asid=ASID(mm);

	BUG_ON(!asid);
	local_irq_save(flags);
	sprset_gpt(PSC, (sprget_gpt(PSC) &
          ~(0xFFUL << PSC_OFFSE_procid)) | (asid << PSC_OFFSE_procid));
        sprset_gpt(PSCH, (sprget_gpt(PSCH) &
          ~(0xFFUL << PSC_OFFSE_procid)) | (asid << PSC_OFFSE_procid));
        sprset_gpt(PSCM, (sprget_gpt(PSCM) &
          ~(0xFFUL << PSC_OFFSE_procid)) | (asid << PSC_OFFSE_procid));
        sprset_gpt(DPAGE, (sprget_gpt(DPAGE) &
          ~(0xFFUL << PSC_OFFSE_procid)) | (asid << PSC_OFFSE_procid));


        asm volatile ("sseta "CURRPGD", %0\n"
          ::"a"(__pa(mm->pgd)) : "memory", CURRPGD);

        local_irq_restore(flags);
}

/*
 * We fork()ed a process, and we need a new context for the child to run in.
 */
void __init_new_context(struct task_struct *tsk, struct mm_struct *mm)
{
	mm->context.asid = NO_CONTEXT;
	raw_spin_lock_init(&mm->context.id_lock);
}

static void flush_context(void)
{
	unsigned long flags;

        local_irq_save(flags);
	local_flush_tlb_all();
	__invalidate_icache_all();
        local_irq_restore(flags);
}

#ifdef CONFIG_SMP

static void set_mm_context(struct mm_struct *mm, unsigned long asid)
{
	unsigned long flags;

	/*
	 * Locking needed for multi-threaded applications where the same
	 * mm->context.id could be set from different CPUs during the
	 * broadcast. This function is also called via IPI so the
	 * mm->context.id_lock has to be IRQ-safe.
	 */
	raw_spin_lock_irqsave(&mm->context.id_lock, flags);
	if (likely((mm->context.asid ^ cpu_last_asid) >> ASID_BITS)) {
		/*
		 * Old version of ASID found. Set the new one and reset
		 * mm_cpumask(mm).
		 */
		mm->context.asid = asid;
		cpumask_clear(mm_cpumask(mm));
	}

	raw_spin_unlock_irqrestore(&mm->context.id_lock, flags);

	/*
	 * Set the mm_cpumask(mm) bit for the current CPU.
	 */
	cpumask_set_cpu(smp_processor_id(), mm_cpumask(mm));
}

/*
 * Reset the ASID on the current CPU. This function call is broadcast from the
 * CPU handling the ASID rollover and holding cpu_asid_lock.
 */
static void reset_context(void *info)
{
	unsigned int asid;
	unsigned int cpu = smp_processor_id();
	struct mm_struct *mm = current->active_mm;

	/*
	 * current->active_mm could be init_mm for the idle thread immediately
	 * after secondary CPU boot or hotplug. 
	 */
	if (mm == &init_mm)
		return;

	smp_rmb();
	asid = cpu_last_asid + cpu + 1;

	flush_context();
	set_mm_context(mm, asid);

	/* set the new ASID */
	cpu_switch_mm(mm);
	
}

#else

static inline void set_mm_context(struct mm_struct *mm, unsigned int asid)
{
	mm->context.asid = asid;
	cpumask_copy(mm_cpumask(mm), cpumask_of(smp_processor_id()));
}

#endif
void __new_context(struct mm_struct *mm)
{
	unsigned long asid;

	raw_spin_lock(&cpu_asid_lock);
#ifdef CONFIG_SMP
	/*
	 * Check the ASID again, in case the change was broadcast from another
	 * CPU before we acquired the lock.
	 */
	if (!unlikely((mm->context.asid ^ cpu_last_asid) >> ASID_BITS)) {
		cpumask_set_cpu(smp_processor_id(), mm_cpumask(mm));
		raw_spin_unlock(&cpu_asid_lock);
		return;
	}
#endif
	/*
	 * At this point, it is guaranteed that the current mm (with an old
	 * ASID) isn't active on any other CPU since the ASIDs are changed
	 * simultaneously via IPI.
	 */
	asid = ++cpu_last_asid;

	/*
	 * If we've used up all our ASIDs, we need to start a new version and
	 * flush the TLB.
	 */
	if (unlikely((asid & (ASID_MASK)) == 0)) {
		if (cpu_last_asid == 0)
			cpu_last_asid = ASID_FIRST_VERSION;
		asid = cpu_last_asid + smp_processor_id() + 1;
		flush_context();
#ifdef CONFIG_SMP
		smp_wmb();
		smp_call_function(reset_context, NULL, 1);
#endif
		cpu_last_asid += NR_CPUS;
	}

	set_mm_context(mm, asid);
	raw_spin_unlock(&cpu_asid_lock);
}
