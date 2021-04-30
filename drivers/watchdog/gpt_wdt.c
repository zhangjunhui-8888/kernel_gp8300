/*
 * Gpt Watchdog Driver
 *
 * (c) Copyright 2020 HXGPT, Inc.
 *
 * auth: yangyang.liu@hxgpt.com
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ":" fmt
#define DRV_NAME	"gpt_watchdog"
#define DRV_VERSION	"0.01"

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/timer.h>
#include <linux/watchdog.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/reboot.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <linux/threads.h>
#include <linux/smp.h>

#define WDT_START		BIT(0)
#define CPU0_WDT_ACTIVE		BIT(4)
#define CPU1_WDT_ACTIVE		BIT(5)
#define CPU2_WDT_ACTIVE		BIT(6)
#define CPU3_WDT_ACTIVE		BIT(7)
#define WDT_STOP		(0)

#define CPU0_FEED	0x1ACCE770
#define CPU1_FEED	(CPU0_FEED + 1)
#define CPU2_FEED	(CPU0_FEED + 2)
#define CPU3_FEED	(CPU0_FEED + 3)

#define DBG(fmt, ...)				\
do {						\
	if (debug)				\
		pr_info(fmt, ##__VA_ARGS__);	\
} while (0)

struct gpt_wdt_regs {
	u32 clkdiv;
	u32 init_val;
	u32 curent_val;
	u32 wdt_rcc;
	u32 wdt_ctrl;
	u32 res1;
	u32 res2;
	u32 intr_st;
};

static struct gpt_wdt_regs drv_data_gp8300;

struct gpt_wdt {
	struct device		*dev;
	struct clk		*clock;
	void __iomem		*reg_base;
	/*unsigned int		count;*/
	spinlock_t		lock;
	unsigned long		wtcon_save;
	unsigned long		wtdat_save;
	struct watchdog_device	wdt_device;
	struct notifier_block	freq_transition;
	struct notifier_block	restart_handler;
	struct gpt_wdt_regs	*drv_data;
	struct regmap		*ppmureg;

};

struct gpt_wdt_conf {
	u32 timeout;
	u64 mmio;
	u32 initno;
};
static struct gpt_wdt_conf wdt_conf;

/*
static struct platform_device *platfrom_device;
static DEFINE_SPINLOCK(wdt_lock);
static struct gpt_wdt wdt;
static __kernel_time_t wdt_expires;
static bool is_active, expect_release;
*/
#ifndef CONFIG_GPT_WATCHDOG_ATBOOT
#define CONFIG_GPT_WATCHDOG_ATBOOT	(0) /* 0:disable at boot; 1:enable at boot.*/
#endif

#define CONFIG_GPT_WATCHDOG_DEFAULT_TIME (30)/*in seconds*/

static bool nowayout	= WATCHDOG_NOWAYOUT;
static int tmr_margin;
static int tmr_atboot = CONFIG_GPT_WATCHDOG_ATBOOT;
//static int soft_noboot;
static int debug = 0;

module_param(tmr_margin,	int, 0);
module_param(tmr_atboot,	int, 0);
module_param(nowayout,		bool, 0);
//module_param(soft_noboot,	int 0);
module_param(debug,		int, 0);

MODULE_PARM_DESC(tmr_margin, "Watchdog tmr_margin in seconds. (default=" __MODULE_STRING(CONFIG_GPT_WATCHDOG_DEFAULT_TIME) ")");

MODULE_PARM_DESC(tmr_atboot, "Watchdog is started at boot time if set to 1, default=" __MODULE_STRING(CONFIG_GPT_WATCHDOG_ATBOOT));

MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started (default=" __MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

//MODULE_PARM_DESC(soft_noboot, "Watchdog action, set to 1 to ignore reboots, " "0 to reboot (default 0)");

MODULE_PARM_DESC(debug, "Watchdog debug, set to > 1 for debug (default 0)");

/* functions */
static void __gpt_wdt_keepalive(void *wdd)
{
	int i;
	struct gpt_wdt *wdt = watchdog_get_drvdata((struct watchdog_device *)wdd);
	struct gpt_wdt_regs *wdt_regs = (struct gpt_wdt_regs *)wdt->reg_base;

	spin_lock(&wdt->lock);
	i = smp_processor_id();
	writel(CPU0_FEED + i, &wdt_regs->wdt_rcc);
	DBG("%d:%s CPU%d, 0x%x => 0x%p\n", __LINE__, __func__,
		i, CPU0_FEED + i, &wdt_regs->wdt_rcc);
	spin_unlock(&wdt->lock);
}

static int gpt_wdt_keepalive(struct watchdog_device *wdd)
{
	on_each_cpu(__gpt_wdt_keepalive, (void *)wdd, 0);

	return 0;
}

static void __gpt_wdt_stop(struct gpt_wdt *wdt)
{
	//unsigned long wdt_intr;
	int i;

	struct gpt_wdt_regs *wdt_regs = (struct gpt_wdt_regs *)wdt->reg_base;

	if (0 != readl(&wdt_regs->intr_st)){
		for (i = 0; i < NR_CPUS; i++){
			writel (~(CPU0_FEED + i), &wdt_regs->wdt_rcc);
			DBG("%d:%s 0x%x => 0x%p\n", __LINE__, __func__,
				~(CPU0_FEED + i), &wdt_regs->wdt_rcc);
		}
	}

	writel (WDT_STOP, &wdt_regs->wdt_ctrl);
	DBG("%d:%s 0x%x => 0x%p\n", __LINE__, __func__,
		WDT_STOP, &wdt_regs->wdt_ctrl);
}

static int gpt_wdt_stop(struct watchdog_device *wdd)
{
	struct gpt_wdt *wdt = watchdog_get_drvdata(wdd);

	spin_lock(&wdt->lock);
	__gpt_wdt_stop(wdt);
	spin_unlock(&wdt->lock);

	return 0;
}

static int gpt_wdt_start(struct watchdog_device *wdd)
{
	int i;
	unsigned long wtcon;
	struct gpt_wdt *wdt = watchdog_get_drvdata(wdd);

	struct gpt_wdt_regs *wdt_regs = (struct gpt_wdt_regs *)wdt->reg_base;

	spin_lock(&wdt->lock);
	__gpt_wdt_stop(wdt);
	wtcon = readl(&wdt_regs->wdt_ctrl);
	for (i = 0; i < NR_CPUS; i++){
		wtcon |= CPU0_WDT_ACTIVE << i;
	}
	wtcon |= WDT_START;
	DBG("read_wdt_ctrl=0x%08x, write_wdt_ctrl=0x%08lx\n", wdt_regs->wdt_ctrl, wtcon);
	writel(wtcon, &wdt_regs->wdt_ctrl);
	spin_unlock(&wdt->lock);

	return 0;
}

static int gpt_wdt_set_heartbeat(struct watchdog_device *wdd, unsigned timeout)
{
	struct gpt_wdt *wdt = watchdog_get_drvdata(wdd);
	struct gpt_wdt_regs *wdt_regs = (struct gpt_wdt_regs *) wdt->reg_base;
	unsigned long freq = clk_get_rate(wdt->clock);
	unsigned int count;
	unsigned int divisor = 1;
	//unsigned int wtcon;

	if(timeout < 1)
		return -EINVAL;

	divisor = freq; //set divisor equl freq

	count = timeout * freq / divisor;

	DBG("%s: timeout=%d, divisor=0x%x, init_val count=%d (%08x)\n", __func__,
			timeout, divisor, count, DIV_ROUND_UP(count, divisor));

	writel(divisor, &wdt_regs->clkdiv);
	writel(count, &wdt_regs->init_val);
	wdd->timeout = (count * divisor) / freq;

	return 0;
}

#define OPTIONS (WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE)

static const struct watchdog_info gpt_wdt_ident = {
	.options		= OPTIONS,
	.firmware_version	= 0,
	.identity		= "GPT Watchdog",
};

static const struct watchdog_ops gpt_wdt_ops = {
	.owner			= THIS_MODULE,
	.start			= gpt_wdt_start,
	.stop			= gpt_wdt_stop,
	.ping			= gpt_wdt_keepalive,
	.set_timeout		= gpt_wdt_set_heartbeat,
};

static struct watchdog_device gpt_wdd = {
	.info = &gpt_wdt_ident,
	.ops = &gpt_wdt_ops,
	.timeout = CONFIG_GPT_WATCHDOG_DEFAULT_TIME,
};

#if 0
/* interrupt handler code */
static irqreturn_t gpt_wdt_irq(int irqno, void *param)
{
	struct gpt_wdt *wdt = platform_get_drvdata(param);

	dev_info(wdt->dev, "watchdog timer expired (irq)\n");

	//TODO... judge some options.
	gpt_wdt_keepalive(&wdt->wdt_device);

	return IRQ_HANDLED;
}
#endif

static int gpt_wdt_parse_dt(struct platform_device *pdev, struct gpt_wdt_conf *params)
{
	int ret;
	struct device_node *np = pdev->dev.of_node;

	ret = of_property_read_u32(np, "timeout-sec", &params->timeout);
	if(ret) {
		dev_err(&pdev->dev, "Can't parse timeout-sec property\n");
		return ret;
	}
/*
	ret = of_property_read_u64(np, "reg", &params->mmio);
	if(reg) {
		dev_err(&pdev->dev, "Can't parse reg property\n");
		return reg;
	}
*/
#if 0
	ret = of_property_read_u32(np, "interrupts", &params->initno);
	if(ret) {
		dev_err(&pdev->dev, "Can't parse interupts property\n");
		return ret;
	}
#endif

	return 0;
}

static inline void gpt_wdt_cpufreq_register(struct gpt_wdt *wdt)
{
}

static inline void gpt_wdt_cpufreq_deregister(struct gpt_wdt *wdt)
{
}

static int gpt_wdt_restart (struct notifier_block *this, unsigned long mode, void *cmd)
{
	struct gpt_wdt *wdt = container_of(this, struct gpt_wdt,  restart_handler);

	struct gpt_wdt_regs *wdt_regs = (struct gpt_wdt_regs *)wdt->reg_base;

	//TODO...

	/* disable watchdog, to be safe */
	__gpt_wdt_stop(wdt);

	/* put initial values into init_bal and clk_div */
	writel(1, &wdt_regs->clkdiv);
	writel(0x80, &wdt_regs->init_val);

	/* set the watchdog to go and reset */
	writel(WDT_START | CPU0_WDT_ACTIVE, &wdt_regs->wdt_ctrl);

	mdelay(500);

	return NOTIFY_DONE;

}

static int gpt_wdt_probe(struct platform_device *pdev)
{
	struct device *dev;
	struct gpt_wdt	*wdt;
	struct gpt_wdt_regs *wdt_regs;
	struct resource	*wdt_mem;
	//struct resource *wdt_irq;
	unsigned int wtcon;
	int started = 0;
	int ret = -1;

	DBG("%s: probe=%p\n", __func__, pdev);

	dev = &pdev->dev;

	wdt = devm_kzalloc(dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt)
		return -ENOMEM;

	wdt->dev = &pdev->dev;
	spin_lock_init(&wdt->lock);
	wdt->wdt_device = gpt_wdd;

	//wdt->drv_data = get_wdt_drv_data(pdev);//not use

	ret = gpt_wdt_parse_dt(pdev, &wdt_conf);
	if(ret) {
		printk("%s,%d: gpt_wdt_parse_dt error\n", __func__, __LINE__);
	}

#if 0
	wdt_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (wdt_irq == NULL) {
		dev_err(dev, "no irq resource specified\n");
		ret = -ENOENT;
		goto err;
	}
#endif

	/* get the memory region for the watchdog timer */
	wdt_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(wdt_mem == NULL) {
		printk("%s:%d --> get memory/io resource failed\n", __func__, __LINE__);
		goto err;
	}

	wdt->reg_base = devm_ioremap_resource(&pdev->dev, wdt_mem);
	if (IS_ERR(wdt->reg_base)) {
		ret = PTR_ERR(wdt->reg_base);
		goto err;
	}
	wdt_regs = (struct gpt_wdt_regs *)wdt->reg_base;	
	DBG("probe: mapped reg_base=%p\n", wdt->reg_base);

	wdt->clock = devm_clk_get(dev, NULL);
	if (IS_ERR(wdt->clock)) {
		dev_err(dev, "failed to find watchdog clock source\n");
		ret = PTR_ERR(wdt->clock);
		goto err;
	}

	ret = clk_prepare_enable(wdt->clock);
	if (ret < 0) {
		dev_err(dev, "failed to enable clock\n");
		return ret;
	}

	watchdog_set_drvdata(&wdt->wdt_device, wdt);

	watchdog_init_timeout(&wdt->wdt_device, tmr_margin, &pdev->dev); //wdd->timeout <= tmr_magin > 0 ? tmr_magin : timeout-sec(dts)
	ret = gpt_wdt_set_heartbeat(&wdt->wdt_device, wdt->wdt_device.timeout);
	if (ret) {
		started = gpt_wdt_set_heartbeat(&wdt->wdt_device, CONFIG_GPT_WATCHDOG_DEFAULT_TIME);
		if (started == 0) {
			dev_info(dev, "tmr_magin value out of range, default %d used\n", CONFIG_GPT_WATCHDOG_DEFAULT_TIME);
		} else {
			dev_info(dev, "default timer value is out of range, " "cannot start\n");
		}
	}

#if 0
	//TODO... enable irq handler.
	ret = devm_request_irq(dev, wdt_irq->start, gpt_wdt_irq, 0, pdev->name, pdev);
	if (ret != 0) {
		dev_err(dev, "failed to install irq (%d)\n", ret);
		goto err_cpufreq;
	}
#endif

	watchdog_set_nowayout(&wdt->wdt_device, nowayout);

	//wdt->wdt_device.bootstatus = gpt_wdt_get_bootstatus(wdt);

	ret = watchdog_register_device(&wdt->wdt_device);
	if(ret){
		dev_err(dev, "cannot register watchdog (%d)\n", ret);
		goto err_cpufreq;
	}

	if (tmr_atboot && started == 0) {
		dev_info(dev, "starting watchdog timer\n");
		gpt_wdt_start(&wdt->wdt_device);
	} else if (!tmr_atboot) {
		gpt_wdt_stop(&wdt->wdt_device);
	}

	platform_set_drvdata(pdev, wdt);//?

	wdt->restart_handler.notifier_call = gpt_wdt_restart;
	wdt->restart_handler.priority = 128;
	ret = register_restart_handler(&wdt->restart_handler);
	if (ret){
		pr_err("cannot register restart handler, %d\n", ret);
		goto err_unregister;
	}
	/* print out a statement of readiness */
	wtcon = readl(&wdt_regs->wdt_ctrl);

	dev_info(dev, "Watchdog %sactive, cpu0 %sabled, cpu1 %sabled, cpu2 %sabled, cpu3 %sabled\n", (wtcon & WDT_START) ? "" : "in",
			(wtcon & CPU0_WDT_ACTIVE) ? "en" : "dis",
			(wtcon & CPU1_WDT_ACTIVE) ? "en" : "dis",
			(wtcon & CPU2_WDT_ACTIVE) ? "en" : "dis",
			(wtcon & CPU3_WDT_ACTIVE) ? "en" : "dis");

	return 0;

err_unregister:
	watchdog_unregister_device(&wdt->wdt_device);

err_cpufreq:
	gpt_wdt_cpufreq_deregister(wdt);

//err_clk:
	clk_disable_unprepare(wdt->clock);

err:
	return ret;
}

static int gpt_wdt_remove(struct platform_device *pdev)
{

	return 0;
}

static void gpt_wdt_shutdown(struct platform_device *pdev)
{

}

#ifdef CONFIG_PM_SLEEP
static int gpt_wdt_suspend(struct platform_device *pdev)
{

	return 0;
}

static int gpt_wdt_resume(struct platform_device *pdev)
{

	return 0;
}
#endif

static const struct of_device_id gpt_wdt_match[] = {
	{ .compatible = "gpt,gpt-wdt",
	  .data = &drv_data_gp8300 },
	{},
};

MODULE_DEVICE_TABLE(of, gpt_wdt_match);

static const struct platform_device_id gpt_wdt_ids[] = {
	{
		.name = "gpt-wdt",
		.driver_data = (unsigned long)&drv_data_gp8300,
	},
	{}
};
MODULE_DEVICE_TABLE(platform, gpt_wdt_ids);

static SIMPLE_DEV_PM_OPS(gpt_wdt_pm_ops, gpt_wdt_suspend, gpt_wdt_resume);

static struct platform_driver gpt_wdt_driver = {
	.probe		= gpt_wdt_probe,
	.remove		= gpt_wdt_remove,
	.shutdown	= gpt_wdt_shutdown,
	.id_table	= gpt_wdt_ids,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= DRV_NAME,
		.pm	= &gpt_wdt_pm_ops,
		.of_match_table = of_match_ptr(gpt_wdt_match),
	},
};

module_platform_driver(gpt_wdt_driver);

MODULE_AUTHOR("Jasion <yangyang.liu@hxgpt.com>");
MODULE_DESCRIPTION("Gpt WatchDog Timer Driver");
MODULE_VERSION(DRV_VERSION);
MODULE_LICENSE("GPL");
