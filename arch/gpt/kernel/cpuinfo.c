#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/bitops.h>
#include <linux/bug.h>
#include <linux/init.h>
#include <linux/preempt.h>
#include <linux/printk.h>
#include <linux/smp.h>
#include <linux/delay.h>

#include <asm/cpuinfo.h>

DEFINE_PER_CPU(struct cpuinfo_gpt, cpudata_gpt);

static void print_cpuinfo(int cpu_id)
{
        struct cpuinfo_gpt *cpuinfo = &per_cpu(cpudata_gpt, cpu_id);
        unsigned int version;
        unsigned int revision;
/*
 * CPU should provide ways to get version info.
 */
        version = 2;
        revision = 0;

        pr_info("CPU %d: GPT-%x (revision %d)\n", cpu_id, version, revision);

        if (cpuinfo->cpuc & (1UL << PSC_OFFSE_cache_ic))
                pr_info(
                       "-- icache: %4d cache lines, %2d bytes/line\n",
                       cpuinfo->icache_no_lines, cpuinfo->icache_line_size);
        else
                pr_info("-- icache disabled\n");

        if (cpuinfo->cpuc & (1UL << PSC_OFFSE_cache_dc))
                pr_info(
                       "-- dcache: %4d cache lines, %2d bytes/line\n",
                       cpuinfo->dcache_no_lines, cpuinfo->dcache_line_size);
        else
                pr_info("-- dcache disabled\n");


        if (cpuinfo->cpuc & (1UL << PSC_OFFSE_trans_it))
                pr_info(
                       "-- itlb: %4d entries, %2d ways\n",
                       cpuinfo->immu_entries, cpuinfo->immu_ways);
        else
                pr_info("-- itlb disabled\n");

        if (cpuinfo->cpuc & (1UL << PSC_OFFSE_trans_dt))
                pr_info(
                       "-- dtlb: %4d entries, %2d ways\n",
                       cpuinfo->dmmu_entries, cpuinfo->dmmu_ways);
        else
                pr_info("-- dtlb disabled\n");

}

static void __setup_cpuinfo(int cpu_id)
{
        struct cpuinfo_gpt *cpuinfo = &per_cpu(cpudata_gpt, cpu_id);

        cpuinfo->icache_line_size = 1UL << GPT_ICACHE_line_ln;
        cpuinfo->icache_no_lines = 1UL << GPT_ICACHE_count_ln;
        cpuinfo->dcache_line_size = 1UL << GPT_DCACHE_line_ln;
        cpuinfo->dcache_no_lines = 1UL << GPT_DCACHE_count_ln;

        cpuinfo->immu_entries = 1UL << IWB_WIDTH_index;
        cpuinfo->immu_ways = 1UL << IWB_WIDTH_way;
        cpuinfo->dmmu_entries = 1UL << DWB_WIDTH_index;
        cpuinfo->dmmu_ways = 1UL << DWB_WIDTH_way;

        cpuinfo->cpuc = sprget_gpt(CPUC);

        print_cpuinfo(cpu_id);
}

void setup_cpuinfo()
{
	__setup_cpuinfo(smp_processor_id());
}

/* Following procedures are for proc interface */
static int c_show(struct seq_file *m, void *v)
{
#ifdef CONFIG_SMP
	unsigned long cpu_id = (unsigned long)v - 1;
/*
 * glibc reads /proc/cpuinfo to determine the number of
 * online processors, looking for lines beginning with
 * "processor".  Give glibc what it expects.
 */
	seq_printf(m, "processor\t: %lu\n", cpu_id);
#endif
	seq_printf(m, "BogoMIPS\t: %lu.%02lu\n",
		   loops_per_jiffy / (500000UL/HZ),
		   loops_per_jiffy / (5000UL/HZ) % 100);

	seq_puts(m, "Features\t:");
	seq_puts(m, " fpu vpu");
	seq_puts(m, "\n");

	seq_printf(m, "CPU architecture: GPT64-CHIP2\n");
	seq_printf(m, "CPU revision\t: %d.%d\n\n", 3, 0);

        return 0;
}

static void *c_start(struct seq_file *m, loff_t * pos)
{
        unsigned long i = *pos;

        return i < NR_CPUS ? (void *) (i + 1) : NULL;
}

static void *c_next(struct seq_file *m, void *v, loff_t * pos)
{
        ++*pos;
        return NULL;
}

static void c_stop(struct seq_file *m, void *v)
{
}

const struct seq_operations cpuinfo_op = {
        .start = c_start,
        .next = c_next,
        .stop = c_stop,
        .show = c_show,
};
