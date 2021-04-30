/*
 * arch/gpt/asm/smp.c
 *
 * SMP initialisation and IPI support
 * Based on arch/arm64/kernel/smp.c
 *
 * Copyright (C) 2017, General Processor Techologies Inc. & HXGPT Tech Ltd.
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

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/cache.h>
#include <linux/profile.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/err.h>
#include <linux/cpu.h>
#include <linux/smp.h>
#include <linux/seq_file.h>
#include <linux/irq.h>
#include <linux/percpu.h>
#include <linux/clockchips.h>
#include <linux/completion.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/irq_work.h>
#include <linux/irqchip/gpt-mpic.h>

#include <asm/atomic.h>
#include <asm/cacheflush.h>
#include <asm/cpuinfo.h>
#include <asm/mmu_context.h>
#include <asm/pgtable.h>
#include <asm/pgalloc.h>
#include <asm/processor.h>
#include <asm/sections.h>
#include <asm/tlbflush.h>
#include <asm/ptrace.h>
#include <asm/time.h>
#include <asm/mach-chip2/apiu.h>

#define CREATE_TRACE_POINTS
#include <trace/events/ipi.h>


/*
 * as from 2.5, kernels no longer have an init_tasks structure
 * so we need some other way of telling a new secondary core
 * where to place its SVC stack
 */
struct secondary_data secondary_data;

enum ipi_msg_type {
	IPI_RESCHEDULE,
	IPI_CALL_FUNC,
	IPI_CALL_FUNC_SINGLE,
	IPI_CPU_STOP,
	IPI_TIMER,
	IPI_IRQ_WORK,
};
struct ipi_data {
        unsigned long bits;
};

const struct plat_smp_ops *smp_ops = NULL;

void register_smp_ops(struct plat_smp_ops *ops)
{
        if (smp_ops)
                printk(KERN_WARNING "Overriding previously set SMP ops\n");

        smp_ops = ops;
}

static DECLARE_COMPLETION(cpu_running);

static int plat_boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	int ret;
	if(smp_ops->cpu_kick)
		ret = smp_ops->cpu_kick(cpu, idle);
	else
		ret = -EOPNOTSUPP;

	return ret;
}
/*
 * Called from kernel_init( ) -> smp_init( ) - for each CPU
 *
 * At this point, Secondary Processor  is "HALT"ed:
 *  -It booted, but was halted in head.S
 *  -It was configured to halt-on-reset
 *  So need to wake it up.
 *
 * Essential requirements being where to run from (PC) and stack (SP)
*/
int __cpu_up(unsigned int cpu, struct task_struct *idle)
{
	int ret;

	/*
	 * We need to tell the secondary core where to find its stack and the
	 * page tables.
	 */
	secondary_data.stack = task_stack_page(idle) + (THREAD_SIZE - 16);
	task_thread_info(idle)->cpu = cpu;
	//asm volatile ("sseta "CURRPGD", %0\n"::"a"(__pa((pgd_t *)swapper_pg_dir)) : "memory", CURRPGD);

	ret = plat_boot_secondary(cpu, idle);
	if (ret == 0) {
		/*
		 * CPU was successfully started, wait for it to come online or
		 * time out.
		 */
		ret = wait_for_completion_timeout(&cpu_running,
					    msecs_to_jiffies(1000));
		if(!ret)
		{
			pr_crit("CPU%u: failed to come online\n", cpu);
			return -EIO;
		}
	} else {
		pr_err("CPU%u: failed to boot: %d\n", cpu, ret);
		return ret;
	}

	synchronise_count_master(cpu);
	secondary_data.stack = NULL;

	return 0;
}

static void smp_store_cpu_info(unsigned int cpuid)
{
	store_cpu_topology(cpuid);
}
/*
 * This is the secondary CPU boot entry.  We're using this CPUs
 * idle thread stack, but a set of temporary page tables.
 */

asmlinkage void secondary_start_kernel(unsigned int cpu)
{
	struct mm_struct *mm = &init_mm;

	/*
	 * All kernel threads share the same mm context; grab a
	 * reference and switch to it.
	 */
	atomic_inc(&mm->mm_count);
	current->active_mm = mm;
	cpumask_set_cpu(cpu, mm_cpumask(mm));

	set_my_cpu_offset(per_cpu_offset(smp_processor_id()));
	printk("CPU%u: Booted secondary processor\n", cpu);

	local_flush_tlb_all();
	preempt_disable();
	trace_hardirqs_off();

	/*
	 * Log the CPU info before it is marked online and might get read.
	 */
	setup_cpuinfo();
	/*
	 * Enable SGI and timers.
	 */
	notify_cpu_starting(cpu);

	smp_store_cpu_info(cpu);

	/*
	 * OK, now it's safe to let the boot CPU continue.  Wait for
	 * the CPU migration code to notice that the CPU is online
	 * before we continue.
	 */

	complete(&cpu_running);
	set_cpu_online(cpu, true);

	synchronise_count_slave(cpu);

	local_irq_enable();

	/*
	 * OK, it's off to the idle thread for us
	 */
	cpu_startup_entry(CPUHP_ONLINE);
}

#ifdef CONFIG_HOTPLUG_CPU
static int plat_cpu_disable(unsigned int cpu)
{
	if (cpu == 0)
		return -EOPNOTSUPP;

	if(smp_ops->cpu_disable)
		return smp_ops->cpu_disable();

	return 0;
}

/*
 * __cpu_disable runs on the processor to be shutdown.
 */
int __cpu_disable(void)
{
	unsigned int cpu = smp_processor_id();
	int ret;

	ret = plat_cpu_disable(cpu);
	if (ret) {
		pr_crit("CPU%u: cpu didn't disabled\n", cpu);
		return ret;
	}

	/*
	 * Take this CPU offline.  Once we clear this, we can't return,
	 * and we must not schedule until we're ready to give up the cpu.
	 */
	set_cpu_online(cpu, false);

	/*
	 * OK - migrate IRQs away from this CPU
	 */
	migrate_irqs();

        /*
         * Flush user cache and TLB mappings, and then remove this CPU
         * from the vm mask set of all processes.
         */
        flush_cache_all();
        local_flush_tlb_all();

	clear_tasks_mm_cpumask(cpu);

	return 0;
}

static int plat_cpu_kill(unsigned int cpu)
{
        if (smp_ops->cpu_kill)
                return smp_ops->cpu_kill(cpu);

        return 1;
}

static DECLARE_COMPLETION(cpu_died);

/*
 * called on the thread which is asking for a CPU to be shutdown -
 * waits until shutdown has completed, or it is timed out.
 */
void __cpu_die(unsigned int cpu)
{
	if (!wait_for_completion_timeout(&cpu_died, msecs_to_jiffies(5000))) {
		pr_crit("CPU%u: cpu didn't die\n", cpu);
		return;
	}
	pr_notice("CPU%u: shutdown\n", cpu);

	/*
	 * Now that the dying CPU is beyond the point of no return w.r.t.
	 * in-kernel synchronisation, try to get the firwmare to help us to
	 * verify that it has really left the kernel before we consider
	 * clobbering anything it might still be using.
	 */
	if (!plat_cpu_kill(cpu)) {
		pr_warn("CPU%d may not have shut down cleanly\n", cpu);
	}
}

static void plat_cpu_die(unsigned int cpu)
{
        if (smp_ops->cpu_die)
                smp_ops->cpu_die(cpu);
}

/*
 * Called from the idle thread for the CPU which has been shutdown.
 *
 * Note that we disable IRQs here, but do not re-enable them
 * before returning to the caller. This is also the behaviour
 * of the other hotplug-cpu capable cores, so presumably coming
 * out of idle fixes this.
 */
void cpu_die(void)
{
	unsigned int cpu = smp_processor_id();

	idle_task_exit();

	local_irq_disable();

	/* Tell __cpu_die() that this CPU is now safe to dispose of */
	complete(&cpu_died);

	/*
	 * Actually shutdown the CPU. This must never fail. The specific hotplug
	 * mechanism must perform all required cache maintenance to ensure that
	 * no dirty lines are lost in the process of shutting down the CPU.
	 */

	plat_cpu_die(cpu);
}

void arch_cpu_idle_dead(void)
{
        cpu_die();
}
#endif

/* Called at end of smp_init for any cleanup work */
void __init smp_cpus_done(unsigned int max_cpus)
{
	pr_info("SMP: Total of %d processors activated.\n", num_online_cpus());
	plat_smp_finish();
}

/* Called from start_kernel after setup_arch(). */
/* Called very early during startup to mark boot cpu as online */
/*
 * Mark the boot cpu "online" so that it can call console drivers in
 * printk() and can access its per-cpu storage.
 */
void __init smp_prepare_boot_cpu(void)
{
	set_my_cpu_offset(per_cpu_offset(smp_processor_id()));
	set_cpu_online(0, true);
}

/*
 * Called from setup_arch() before calling setup_processor()
 * Initialise the CPU possible map early - this describes the CPUs
 * which may be present or become present in the system.
 *
 */
/* Called from ./arch/gpt/kernel/setup.c */
void __init smp_init_cpus(void)
{
	struct device_node *dn = NULL;
	unsigned int i, cpu = 1;

	while ((dn = of_find_node_by_type(dn, "cpu"))) {
		const u32 *cell;

		/* check cpu register? */
		cell = of_get_property(dn, "reg", NULL);
		if (!cell) {
			pr_err("%s: missing reg property\n", dn->full_name);
			goto next;
		}

		if (cpu >= NR_CPUS) {
			break;
		}

next:
		cpu++;
	}

	/* sanity check */
	if (cpu > NR_CPUS) {
		pr_warning("no. of cores (%d) greater than configured maximum of %d - clipping\n",
			   cpu, NR_CPUS);
	}

	for (i = 0; i < NR_CPUS; i++) {
		set_cpu_possible(i, true);
	}
}

/* called from init ( ) =>  process 1 */
/*
 * Called just before smp_init to prepare the secondaries
 */
void __init smp_prepare_cpus(unsigned int max_cpus)
{
	unsigned int cpu, ncores = num_possible_cpus();

	init_cpu_topology();

	smp_store_cpu_info(smp_processor_id());

	/*
	 * are we trying to boot more cores than exist?
	 */
	if (max_cpus > ncores) {
		max_cpus = ncores;
	}

	/* Don't bother if we're effectively UP */
	if (max_cpus <= 1) {
		return;
	}

	/*
	 * Initialise the present map (which describes the set of CPUs
	 * actually populated at the present time) and release the
	 * secondaries from the bootloader.
	 *
	 * Make sure we online at most (max_cpus - 1) additional CPUs.
	 */
	max_cpus--;
	for_each_possible_cpu(cpu) {
		if (max_cpus == 0) {
			break;
		}

		if (cpu == smp_processor_id()) {
			continue;
		}

		set_cpu_present(cpu, true);
		max_cpus--;
	}

	plat_smp_init();
}

/*****************************************************************************/
/*              Inter Processor Interrupt Handling                           */
/*****************************************************************************/

void (*__smp_cross_call)(const struct cpumask *, unsigned int);

void __init set_smp_cross_call(void (*fn)(const struct cpumask *, unsigned int))
{
	__smp_cross_call = fn;
}

static const char *ipi_types[NR_IPI_TYPE] __tracepoint_string = {
#define S(x,s)	[x] = s
	S(IPI_RESCHEDULE, "Rescheduling interrupts"),
	S(IPI_CALL_FUNC, "Function call interrupts"),
	S(IPI_CALL_FUNC_SINGLE, "Single function call interrupts"),
	S(IPI_CPU_STOP, "CPU stop interrupts"),
	S(IPI_TIMER, "Timer broadcast interrupts"),
	S(IPI_IRQ_WORK, "IRQ work interrupts"),
};

static void smp_cross_call(const struct cpumask *target, unsigned int ipi_type)
{
	trace_ipi_raise(target, ipi_types[ipi_type]);
	__smp_cross_call(target, ipi_type);
}

void show_ipi_list(struct seq_file *p, int prec)
{
	unsigned int cpu, i;

	for (i = 0; i < NR_IPI_TYPE; i++) {
		seq_printf(p, "%*s%u:%s", prec - 1, "IPI_TYPE", i,
			   prec >= 4 ? " " : "");
		for_each_online_cpu(cpu) {
			seq_printf(p, "%10u ",
				   __get_irq_stat(cpu, ipi_type_cnt[i]));
		}
		seq_printf(p, "      %s\n", ipi_types[i]);
	}
}

u64 smp_irq_stat_cpu(unsigned int cpu)
{
	u64 sum = 0;
	int i;

	for (i = 0; i < NR_IPI_TYPE; i++) {
		sum += __get_irq_stat(cpu, ipi_type_cnt[i]);
	}

	return sum;
}

void arch_send_call_function_ipi_mask(const struct cpumask *mask)
{
	smp_cross_call(mask, IPI_CALL_FUNC);
}

void arch_send_call_function_single_ipi(int cpu)
{
	smp_cross_call(cpumask_of(cpu), IPI_CALL_FUNC_SINGLE);
}

#ifdef CONFIG_IRQ_WORK
void arch_irq_work_raise(void)
{
	smp_cross_call(cpumask_of(smp_processor_id()), IPI_IRQ_WORK);
}
#endif

static DEFINE_RAW_SPINLOCK(stop_lock);

/*
 * ipi_cpu_stop - handle IPI from smp_send_stop()
 */
static void ipi_cpu_stop(unsigned int cpu)
{
	if (system_state == SYSTEM_BOOTING ||
	    system_state == SYSTEM_RUNNING) {
		raw_spin_lock(&stop_lock);
		pr_crit("CPU%u: stopping\n", cpu);
		dump_stack();
		raw_spin_unlock(&stop_lock);
	}

	set_cpu_online(cpu, false);

	local_irq_disable();

	while (1) {
		cpu_relax();
	}
}

/*
 * Main handler for inter-processor interrupts
 */
void handle_IPI(u8 ipi_type)
{
	unsigned int cpu = smp_processor_id();
				
	if (ipi_type < NR_IPI_TYPE) {
		trace_ipi_entry(ipi_types[ipi_type]);
		__inc_irq_stat(cpu, ipi_type_cnt[ipi_type]);
	}

	switch (ipi_type) {
		case IPI_RESCHEDULE:
			scheduler_ipi();
			break;

				case IPI_CALL_FUNC:
					irq_enter();
					generic_smp_call_function_interrupt();
					irq_exit();
					break;

				case IPI_CALL_FUNC_SINGLE:
					irq_enter();
					generic_smp_call_function_single_interrupt();
					irq_exit();
					break;

				case IPI_CPU_STOP:
					irq_enter();
					ipi_cpu_stop(cpu);
					irq_exit();
					break;

#ifdef CONFIG_GENERIC_CLOCKEVENTS_BROADCAST
				case IPI_TIMER:
					irq_enter();
					tick_receive_broadcast();
					irq_exit();
					break;
#endif

#ifdef CONFIG_IRQ_WORK
				case IPI_IRQ_WORK:
					irq_enter();
					irq_work_run();
					irq_exit();
					break;
#endif

				default:
					break;
			}
		smp_mb();

	if (ipi_type < NR_IPI_TYPE) {
		trace_ipi_exit(ipi_types[ipi_type]);
	}
}

void smp_send_reschedule(int cpu)
{
	smp_cross_call(cpumask_of(cpu), IPI_RESCHEDULE);
}

#ifdef CONFIG_GENERIC_CLOCKEVENTS_BROADCAST
void tick_broadcast(const struct cpumask *mask)
{
	smp_cross_call(mask, IPI_TIMER);
}
#endif

void smp_send_stop(void)
{
	unsigned long timeout;

	if (num_online_cpus() > 1) {
		cpumask_t mask;

		cpumask_copy(&mask, cpu_online_mask);
		cpu_clear(smp_processor_id(), mask);

		smp_cross_call(&mask, IPI_CPU_STOP);
	}

	/* Wait up to one second for other CPUs to stop */
	timeout = USEC_PER_SEC;
	while (num_online_cpus() > 1 && timeout--) {
		udelay(1);
	}

	if (num_online_cpus() > 1) {
		pr_warning("SMP: failed to stop secondary CPUs\n");
	}
}

/*
 * not supported here
 */
int setup_profiling_timer(unsigned int multiplier)
{
	return -EINVAL;
}
