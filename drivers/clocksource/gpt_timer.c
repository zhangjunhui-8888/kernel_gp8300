/*
 * linux/drivers/clocksource/gpt_timer.c
 *
 * GPT tick timer support.
 *
 * Copyright (C) 2017, Optimum Semiconductor Technologies
 * Copyright (C) 2017, HXGPT Inc.
 *  Nick Wu <fwu@hxgpt.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see the file COPYING, or write
 * to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/smp.h>
#include <linux/cpu.h>
#include <linux/cpu_pm.h>
#include <linux/clockchips.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/sched_clock.h>
#include <linux/clk.h>

#include <asm/time.h>

static int gpt_timer_irq;
static unsigned long gpt_timer_freq;
static struct clock_event_device __percpu *gpt_timer_evt;

/* Set the timer to trigger in delta cycles */
inline void gpt_timer_set_next(unsigned long delta)
{
        sprset_gpt(TCD0C, sprget_gpt(TCD0C) & ~(1 << TCD0C_OFFSE_en));
	sprset_gpt(TCD0, delta);
        /* Set counter and enable interrupt. */
        sprset_gpt(TCD0C, sprget_gpt(TCD0C) | (1 << TCD0C_OFFSE_en));
}

static int gpt_clkevent_set_next_event(unsigned long evt, struct clock_event_device *clk)
{
        /* Load count down register. */
	gpt_timer_set_next(evt);
        return 0;
}

static void gpt_clkevent_set_mode(enum clock_event_mode mode,
			       struct clock_event_device *evt)
{
	switch (mode) {
		case CLOCK_EVT_MODE_PERIODIC:
			pr_debug(KERN_INFO "%s: periodic\n", __func__);
			BUG();
			break;
		case CLOCK_EVT_MODE_ONESHOT:
			break;
		case CLOCK_EVT_MODE_UNUSED:
		case CLOCK_EVT_MODE_SHUTDOWN:
        	  sprset_gpt(TCD0C, sprget_gpt(TCD0C) & ~(1 << TCD0C_OFFSE_en));
		  break;
		case CLOCK_EVT_MODE_RESUME:
        	  sprset_gpt(TCD0C, sprget_gpt(TCD0C) | (1 << TCD0C_OFFSE_ie));
		  break;
		default:
			pr_debug(KERN_INFO "%s: default\n", __func__);
			break;
        }
}

static inline void timer_ack(void)
{
        /* Disable further interrupts */
	sprset_gpt(XCLR, 1UL << 3);
}

static irqreturn_t gpt_timer_handler(int irq, void *dev_id)
{
	struct clock_event_device *evt = dev_id;

	timer_ack();
        evt->event_handler(evt);

	return IRQ_HANDLED;
}

int gpt_timer_clockevent_register(struct clock_event_device *evt, unsigned long freq, int irq)
{
	unsigned int cpu = smp_processor_id();

	evt->name = "gpt_timer_evt";
	evt->rating = 300;
	evt->cpumask = cpumask_of(cpu);
	evt->irq = irq;
	evt->set_mode = gpt_clkevent_set_mode;
	evt->set_next_event = gpt_clkevent_set_next_event;
	evt->features = CLOCK_EVT_FEAT_ONESHOT;

	//enable_percpu_irq(irq, 0);
	clockevents_config_and_register(evt, freq,
                                        1, 0xffffffff);

	sprset_gpt(XCLR, 1UL << 3);
        sprset_gpt(TCD0C, sprget_gpt(TCD0C) | (1 << TCD0C_OFFSE_ie));
        	return 0;
}

/* Test the timer ticks to count, used in sync routine */
inline void gpt_timer_set(unsigned long count)
{
	sprset_gpt(TIM, count);
}

static void gpt_timer_enable(void)
{
        /* Enable the incrementer: 'continuous' mode with interrupt disabled */
	sprset_gpt(TIMC, 1<<TIMC_OFFSE_en);
}

static void gpt_timer_disable(void)
{
        /* Disable the incrementer: 'continuous' mode with interrupt disabled */
	sprset_gpt(TIMC, 0);
}

static void gpt_timer_resume(struct clocksource *cs)
{
	sprset_gpt(TIMC, 0);
	sprset_gpt(TIM, 0);
	sprset_gpt(TIMC, 1 << TIMC_OFFSE_en);
}

static u64 gpt_timer_read(struct clocksource *cs)
{
        return (u64) sprget_gpt(TIM);
}

static u64 gpt_read_sched_clock(void)
{
        return (u64) sprget_gpt(TIM);
}


static struct clocksource gpt_timer = {
        .name = "gpt_timer",
        .rating = 300,
        .read = gpt_timer_read,
        .mask = CLOCKSOURCE_MASK(64),
        .flags = CLOCK_SOURCE_IS_CONTINUOUS,
	.resume = gpt_timer_resume,
};

int __init gpt_clocksource_setup(void)
{
	unsigned long rate = gpt_timer_freq >> 3;
	
        if (clocksource_register_hz(&gpt_timer, rate))
                panic("failed to register clocksource");

	gpt_timer_disable();
	sprset_gpt(TIM, 0);
	gpt_timer_enable();

	sched_clock_register(gpt_read_sched_clock, 64, rate);
        return 0;
}

/**
 * This is the clock event device based on the gpt count down timer.
 */
static void gpt_timer_cpu_start(struct clock_event_device *evt)
{
	gpt_timer_clockevent_register(evt, gpt_timer_freq, gpt_timer_irq);
	gpt_timer_disable();
	sprset_gpt(TIM, 0);
	gpt_timer_enable();
}

static void gpt_timer_cpu_stop(struct clock_event_device *evt)
{
        sprset_gpt(TCD0C, 0);
	gpt_timer_disable();
}

static int gpt_timer_cpu_notify(struct notifier_block *self,
				unsigned long action, void *hcpu)
{
	/*
	 * Grab cpu pointer in each case to avoid spurious
	 * preemptible warnings
	 */
	switch (action & ~CPU_TASKS_FROZEN) {
	case CPU_STARTING:
		gpt_timer_cpu_start(this_cpu_ptr(gpt_timer_evt));
		break;
	case CPU_DYING:
		gpt_timer_cpu_stop(this_cpu_ptr(gpt_timer_evt));
		break;
	default:
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block gpt_timer_cpu_nb = {
	.notifier_call = gpt_timer_cpu_notify,
};

static int __init gpt_clockevent_setup(struct device_node *np)
{
	int ret;
	struct clock_event_device *evt;

	ret = irq_of_parse_and_map(np, 0);
        if (ret <= 0) {
                pr_err("gpt-clockevent: missing irq, ret=%d\n",ret);
                return ret;
        }
	gpt_timer_irq = ret;

        ret = gpt_get_cpu_clk();
        if (!ret) {
                pr_err("gpt-clockevent: missing clk, ret=%d\n",ret);
                return ret;
        }
	gpt_timer_freq = ret;

	gpt_timer_evt = alloc_percpu(struct clock_event_device);
	if(gpt_timer_evt == NULL){
		ret = -ENOMEM;
		goto out;
	}
	evt = this_cpu_ptr(gpt_timer_evt);

        ret = request_percpu_irq(gpt_timer_irq, gpt_timer_handler,
                                 "gpt_timer_evt", gpt_timer_evt);
        if (ret) {
                pr_err("clockevent: unable to request irq, ret=%d\n",ret);
                goto out_free;
        }

	ret = register_cpu_notifier(&gpt_timer_cpu_nb);
	if (ret) {
		goto out_free_irq;
	}

	gpt_timer_clockevent_register(evt, gpt_timer_freq, gpt_timer_irq);

	return 0;
out_free_irq:
	free_percpu_irq(gpt_timer_irq, gpt_timer_evt);
out_free:
	free_percpu(gpt_timer_evt);
out:
	return ret;
}

static void __init gpt_of_timer_init(struct device_node *np)
{
	gpt_clockevent_setup(np);
	gpt_clocksource_setup();
}

CLOCKSOURCE_OF_DECLARE(gpt_timer, "gpt,gpt-timer", gpt_of_timer_init);
