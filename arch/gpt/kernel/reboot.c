/*
 * GPT machine reboot support, refer to arm64
 *
 * Copyright (C) 2018, General Processor Techologies Inc.
 * scxie <scxie@hxgpt.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/smp.h>
#include <linux/cpu.h>
#include <linux/reboot.h>
#include <linux/io.h>

#include <asm/reboot.h>

void (*pm_power_off)(void);
EXPORT_SYMBOL_GPL(pm_power_off);
void gpt_pm_restart(const char *cmd)
{
	unsigned int val;
	val = readl(ioremap(0xf0000034, 4));
	val &=~0xffff0000;
	writel(val|0xa0000000, ioremap(0xf0000034, 4));	
}
/*
 * Restart requires that the secondary CPUs stop performing any activity
 * while the primary CPU resets the system. Systems with multiple CPUs must
 * provide a HW restart implementation, to ensure that all CPUs reset at once.
 * This is required so that any code running after reset on the primary CPU
 * doesn't have to co-ordinate with other CPUs to ensure they aren't still
 * executing pre-reset code, and using RAM that the primary CPU's code wishes
 * to use. Implementing such co-ordination would be essentially impossible.
 */
void machine_restart(char *cmd)
{
       /* Disable interrupts first */
        local_irq_disable();
        smp_send_stop();

        /* Now call the architecture specific reboot code. */
        do_kernel_restart(cmd);
        gpt_pm_restart(cmd);

	printk("Reboot failed -- System halted\n");
        while (1);
}

/*
 * Halting simply requires that the secondary CPUs stop performing any
 * activity (executing tasks, handling interrupts). smp_send_stop()
 * achieves this.
 */
void machine_halt(void)
{
        local_irq_disable();
        smp_send_stop();
        while(1);
}

/* If or when software power-off is implemented, add code here.  */
/* Called from kernel */
void machine_power_off(void)
{
        local_irq_disable();
        smp_send_stop();
        if (pm_power_off)
                pm_power_off();
}

/*
 * Called by kexec, immediately prior to machine_kexec().
 *
 * This must completely disable all secondary CPUs; simply causing those CPUs
 * to execute e.g. a RAM-based pin loop is not sufficient. This allows the
 * kexec'd kernel to use any and all RAM as it sees fit, without having to
 * avoid any code or data used by any SW CPU pin loop. The CPU hotplug
 * functionality embodied in disable_nonboot_cpus() to achieve this.
 */
void machine_shutdown(void)
{
        disable_nonboot_cpus();
}
