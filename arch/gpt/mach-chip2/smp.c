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

#include <asm/mach-chip2/sysctrl.h>

struct cpulaunch {
	unsigned long       jump_addr;
	unsigned long       cpu_id;
	unsigned long       err_flag;
};

static struct cpulaunch * mem_smpboot = NULL;
static struct cpulaunch *launch = NULL;
#ifdef CONFIG_GPT_SIMULATOR_ERRTA3
static int sim_boot = 0;
#endif

static void cpu_set_start_addr(unsigned long bootaddr, unsigned int cpu)
{
        /* set cpux jump address */
        launch = mem_smpboot + cpu;
        
	launch->jump_addr = bootaddr;
	launch->cpu_id = cpu;
	launch->err_flag = 0;
	mb();	
}

static void cpu_release(unsigned int cpu)
{
	enum sysreg_off off = cpu * 4;
	/* Use BP's control value for APs */
	u32 val = gpt_sysctrl_read(SYSREG_CORE0_CCR);
	val |= 1 << 4;
	/* reset cpux */
	gpt_sysctrl_write(off, val);
}

int gpt_cpun_start(unsigned long bootaddr, int cpu)
{
	cpu_set_start_addr(bootaddr, cpu);
#ifdef CONFIG_GPT_SIMULATOR_ERRTA3
	sim_boot = 1;
#else		
	cpu_release(cpu);
#endif	
	return 0;
}

int gpt_cpu_disable(void)
{
        unsigned int cpu = smp_processor_id();

        if (cpu == 0)
                return -EBUSY;
        
        return 0;
}

#ifdef CONFIG_HOTPLUG_CPU
static void gpt_cpu_die(unsigned int cpu)
{	

	/* Should never be here */
        while (1) {
#ifdef CONFIG_GPT_SIMULATOR_ERRTA3
                if(sim_boot) {
	                unsigned long _bootaddr, _bootval = 0;

                        sim_boot = 0;			
                        _bootaddr = launch->jump_addr;
                        __asm__ __volatile__ (                                  \
                        "sprsetr        %0, %2\n"                               \
                        "sprsetr        %1, %3\n"                               \
                        "retfi_alt\n"                                           \
                        :                                                       \
                        : "r" (_bootval), "r" (_bootaddr), "i" (XSA), "i" (XRA) \
                        : "memory");
                }
#endif        
                cpu_relax();
        }
}

static int gpt_cpu_kill(unsigned int cpu)
{
#ifndef CONFIG_GPT_SIMULATOR_ERRTA3
	/* disable cpux */
	enum sysreg_off off = cpu * 4;
	/* Use BP's control value for APs */
	u32 val = gpt_sysctrl_read(SYSREG_CORE0_CCR);
	val &= ~(1 << 4);
	/* reset cpux */
	gpt_sysctrl_write(off, val);
#endif
	return 1;
}
#endif

static int gpt_cpu_kick(unsigned int cpu, struct task_struct *idle)
{
	unsigned long bootaddr = virt_to_phys(_start);
	
	cpu_set_start_addr(bootaddr, cpu);
	cpu_release(cpu);
	
	return 0;
}

static int gpt_smp_setup(void)
{
	struct device_node *np;

	np = of_find_node_by_name(NULL, "smpboot");
	if(!np){
		pr_err("Device node %s is not found\n", np->full_name);
		return -ENODEV;
	}
	mem_smpboot = of_iomap(np, 0);
	if(!mem_smpboot){
		pr_err("%s can not be remaped\n", np->full_name);
		return -ENOMEM;
	}
	
	return 0;
}

struct plat_smp_ops gpt_smp_ops ={
	.cpu_kick = gpt_cpu_kick,
#ifdef CONFIG_HOTPLUG_CPU
	.cpu_disable = gpt_cpu_disable,
	.cpu_die = gpt_cpu_die,
	.cpu_kill = gpt_cpu_kill
#endif
};

void __init plat_smp_finish(void)
{
#ifndef CONFIG_HOTPLUG_CPU
	if(mem_smpboot)
		iounmap(mem_smpboot);
#endif
}

void __init plat_smp_init(void)
{
	int ret;		
	ret = gpt_smp_setup();
	if(ret)
		panic("%s failed\n",__func__);
		
	register_smp_ops(&gpt_smp_ops);
}
