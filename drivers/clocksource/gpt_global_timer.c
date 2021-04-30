/*
 * drivers/clocksource/arm_global_timer.c
 *
 * Copyright (C) 2013 STMicroelectronics (R&D) Limited.
 * Author: Stuart Menefy <stuart.menefy@st.com>
 * Author: Srinivas Kandagatla <srinivas.kandagatla@st.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/cpu.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/sched_clock.h>
#include <linux/delay.h>

#include <asm/mach-chip2/apiu.h>

#define APIU_TIMER_CLOCK_DIV	0x00
#define APIU_TIMER_TARGET_VAL	0x04
#define APIU_TIMER_CUR_VAL	0x08
#define APIU_TIMER_PWM_VAL	0x0C
#define APIU_TIMER_CTRL		0x10
#define APIU_TIMER_INTR_RAW	0x14
#define APIU_TIMER_INTR_MASK	0x18
#define APIU_TIMER_INTR_STS	0x1C
#define APIU_TIMER_INTR_CLR	0x20
#define APIU_TIMER_RELOAD_EN	0x24
#define APIU_TIMER_RELOAD_VAL	0x28
#define APIU_TIMER_EN_OFFSET		(0)
#define APIU_TIMER_MODE_OFFSET		(1)

/*
 * We are expecting to be clocked by the ARM peripheral clock.
 *
 * Note: it is assumed we are using a prescaler value of zero, so this is
 * the units for all operations.
 */
static void __iomem *gt_base;
static unsigned long gt_clk_rate;

static void dump_timer_reg(void)
{
	int i;
	for(i=0; i < 0x2c; i=i+4){
		pr_debug("func=%s, -------APIU_TIMER[0x%x]=0x%x\n", __func__, i, readl(gt_base + i));
	}
}

unsigned long long global_counter;
unsigned long long last_counter;
unsigned long long update_cnt;

static DEFINE_SPINLOCK(global_timer_lock);
static u64 gt_counter_read(void)
{
	unsigned long long cur_counter;
	unsigned long long ret_val;
	unsigned long flags;

	spin_lock_irqsave(&global_timer_lock, flags);
	cur_counter=readl(gt_base + APIU_TIMER_CUR_VAL);
	if (last_counter >= cur_counter){
		update_cnt += (1ULL<<32);
	}
	last_counter=cur_counter;
	ret_val=(global_counter + cur_counter + update_cnt);
	spin_unlock_irqrestore(&global_timer_lock, flags);
	return ret_val;
}	


static cycle_t gt_clocksource_read(struct clocksource *cs)
{
	return gt_counter_read();
}

static cycle_t arch_counter_read_cc(const struct cyclecounter *cc)
{
	return gt_counter_read();
}

static struct clocksource gt_clocksource = {
	.name	= "gpt_global_timer",
	.rating	= 350,
	.read	= gt_clocksource_read,
	.mask	= CLOCKSOURCE_MASK(64),
	.flags	= CLOCK_SOURCE_IS_CONTINUOUS,
};


static struct cyclecounter cyclecounter = {
	.read	= arch_counter_read_cc,
	.mask	= CLOCKSOURCE_MASK(64),
};


static struct timecounter timecounter;

static u64 notrace gt_sched_clock_read(void)
{
	return gt_counter_read();
}

static void __init gt_clocksource_init(void)
{
	u64 start_count;
	writel(0, gt_base + APIU_TIMER_CUR_VAL);
	writel(0xffffffff, gt_base + APIU_TIMER_TARGET_VAL);
	writel(0, gt_base + APIU_TIMER_INTR_MASK);

	writel((1<<APIU_TIMER_MODE_OFFSET)|(1<<APIU_TIMER_EN_OFFSET), gt_base + APIU_TIMER_CTRL);
	dump_timer_reg();

	clocksource_register_hz(&gt_clocksource, gt_clk_rate);
	start_count = gt_counter_read();
	cyclecounter.mult = gt_clocksource.mult;
	cyclecounter.shift = gt_clocksource.shift;
	timecounter_init(&timecounter, &cyclecounter, start_count);

	sched_clock_register(gt_sched_clock_read, 64, gt_clk_rate);
}

static irqreturn_t gpt_gt_handler(int irq, void *dev_id)
{
	if(readl(gt_base + APIU_TIMER_INTR_STS)){
		writel(1, gt_base + APIU_TIMER_INTR_CLR);
		spin_lock(&global_timer_lock);
		global_counter += (update_cnt +(1ULL<<32));
		last_counter=update_cnt=0;
		spin_unlock(&global_timer_lock);
	}
        return IRQ_HANDLED;
}


static void __init global_timer_of_register(struct device_node *np)
{
	struct clk *gt_clk;
	int err = 0;
	int gt_irq;

	gt_base = of_iomap(np, 0);
	if (!gt_base) {
		pr_warn("global-timer: invalid base address\n");
		return;
	}
	gt_irq=irq_of_parse_and_map(np, 0);
	err=request_irq(gt_irq, gpt_gt_handler, IRQF_TIMER, "gpt_global_timer_irq", NULL);
        if (err) {
                pr_err("gpt_global_timer: can't register interrupt %d (%d)\n",
                       gt_irq, err);
                goto out_unmap;
        }

	gt_clk = of_clk_get(np, 0);
	if (!IS_ERR(gt_clk)) {
		err = clk_prepare_enable(gt_clk);
		if (err)
			goto out_free;
	} else {
		pr_warn("global-timer: clk not found\n");
		err = -EINVAL;
		goto out_free;
	}

	gt_clk_rate = clk_get_rate(gt_clk);
	writel(1, gt_base + APIU_TIMER_INTR_CLR);
	writel(0, gt_base + APIU_TIMER_CLOCK_DIV);
	gt_clk_rate = gt_clk_rate/2;

	gt_clocksource_init();
	lpj_fine = gt_clk_rate / HZ;	

	return;
out_free:
	free_irq(gt_irq, NULL);
out_unmap:
	iounmap(gt_base);
	WARN(err, "GPT Global timer register failed (%d)\n", err);
}

CLOCKSOURCE_OF_DECLARE(gpt_gt, "gpt,gpt-global-timer",
			global_timer_of_register);
