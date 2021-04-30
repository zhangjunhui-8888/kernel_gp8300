/*
 * GPT  I2C adapter driver (slave only).
 *
 * Based on the TI DAVINCI I2C adapter driver.
 *
 * Copyright (C) 2006 Texas Instruments.
 * Copyright (C) 2007 MontaVista Software Inc.
 * Copyright (C) 2009 Provigent Ltd.
 *
 * ----------------------------------------------------------------------------
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
 * ----------------------------------------------------------------------------
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/dmi.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/acpi.h>
#include <linux/platform_data/i2c-gpt.h>
#include "i2c-gpt.h"
#include <asm/mach-chip2/apiu.h>
#include <linux/export.h>


#ifdef	CONFIG_APIU
#define gpt_readl(addr)		apiu_readl(addr)
#define gpt_writel(val,addr)	apiu_writel(val,addr)
#endif


//#define AF_PRINT
#ifdef  AF_PRINT
#define  SP_PRINTK(format,arg...) do { \
                                          printk(KERN_INFO format,##arg);\
                                  }while(0)
#else
#define  SP_PRINTK(format,arg...) do { \
                                          }while(0)
#endif


#define GPT_IC_INTR_DEFAULT_MASK		(GPT_IC_INTR_RX_FULL | \
					 GPT_IC_INTR_RD_REQ | \
					 GPT_IC_INTR_TX_ABRT)


typedef struct student
{
	int data;
	struct student *next;
}node;

typedef struct linkqueue 
{
	node *first, *rear;	
}queue;


queue *headque;

static char *abort_sources[] = {
	[ABRT_7B_ADDR_NOACK] =
		"slave address not acknowledged (7bit mode)",
	[ABRT_10ADDR1_NOACK] =
		"first address byte not acknowledged (10bit mode)",
	[ABRT_10ADDR2_NOACK] =
		"second address byte not acknowledged (10bit mode)",
	[ABRT_TXDATA_NOACK] =
		"data not acknowledged",
	[ABRT_GCALL_NOACK] =
		"no acknowledgement for a general call",
	[ABRT_GCALL_READ] =
		"read after general call",
	[ABRT_SBYTE_ACKDET] =
		"start byte acknowledged",
	[ABRT_SBYTE_NORSTRT] =
		"trying to send start byte when restart is disabled",
	[ABRT_10B_RD_NORSTRT] =
		"trying to read when restart is disabled (10bit mode)",
	[ABRT_MASTER_DIS] =
		"trying to use disabled adapter",
	[ARB_LOST] =
		"lost arbitration",
};


static void slave_insert(int x)
{
	int tmp = 0;
	if(headque==NULL)
	{
		tmp = 1;
		headque = (queue*)kmalloc(sizeof(queue), GFP_KERNEL);
	}
	
	node *s = NULL;

	s = (node*)kmalloc(sizeof(node),GFP_KERNEL);
	s->data = x;
	s->next = NULL;

//	if(headque->rear == NULL)
	if(tmp == 1 || headque->rear==NULL)
	{
		headque->first = s;
		headque->rear = s;
	}
	else
	{	
		headque->rear->next = s;
		headque->rear = s;
	}
}

static int slave_del(void)
{

	node *p = NULL;
	int x = 0;

	if(headque->first == NULL)
	{
		return 0;
	}
	else
	{
		x = headque->first->data;

		p = headque->first;
		if(headque->first == headque->rear)
		{
			headque->first = NULL;
			headque->rear = NULL;
		}
		else
		{
			headque->first = headque->first->next;
			kfree(p);
			p = NULL;
		}
	}

	return x;
}

static void show(void)
{
	node *p = NULL;
	int x= 0;

	p = headque->first;
	while(p != NULL)
	{
		x = p->data;
		printk("val:%d\n",x);
		p = p->next;
	}
}



static void __i2c_gpt_enable(struct gpt_i2c_dev *dev, bool enable)
{
	int timeout = 100;

	do {
		gpt_writel(enable, dev->base + GPT_IC_ENABLE);
		if ((gpt_readl(dev->base + GPT_IC_ENABLE_STATUS) & 1) == enable)
			return;

		/*
		 * Wait 10 times the signaling period of the highest I2C
		 * transfer supported by the driver (for 400KHz this is
		 * 25us) as described in the DesignWare I2C databook.
		 */
		usleep_range(25, 250);
	} while (timeout--);

	dev_warn(dev->dev, "timeout in %sabling adapter\n",
		 enable ? "en" : "dis");
}

int i2c_gpt_acquire_lock(struct gpt_i2c_dev *dev)
{
	int ret;

	if(!dev->acquire_lock)
		return 0;
	
	ret = dev->acquire_lock();
	if(!ret)
		return 0;

	dev_err(dev->dev,"could't acuire bus ownership\n");

	return ret;
}

/*
int i2c_gpt_set_sda_hold(struct gpt_i2c_dev *dev)
{
	u32 reg;
	int ret;

	ret = i2c_gpt_acquire_lock(dev);
	if(ret)
		return ret;
	
	reg = gpt_readl(dev->base + GPT_IC_COMP_VERSION);

	if(reg >= GPT_IC_SDA_HOLD_MIN_VERS){
		if(!dev->sda_hold_time){
			dev->sda_hold_time = gpt_readl(dev->base + GPT_IC_ADA_HOLD);			
		}	
		if(!(dev->sda_hold_time & GPT_IC_SDA_HOLD_RX_MASK))
			dev->sda_hold_time | = 1 << GPT_IC_SDA_HOLD_RX_SHIFT;

		dev_dbg(dev->dev, "SDA Hold Time Tx:Rx = %d:%d\n",dev->sda_hold_time & ~(u32)GPT_IC_SDA_HOLD_RX_MASK, dev->sda_hold_time >> GPT_IC_SDA_HOLD_RX_SHIFT);
	}
	else if(dev->set_sda_hold_time){
		dev->set_sda_hold_time(dev);	
	}
	else if(dev->set_sda_hold_time){
		dev_warn(dev->dev,"Hardware too old to adjust SDA hold time.\n");	
		dev->sda_hold_time = 0;
	}

	return ret;
}
*/


static u32 i2c_gpt_read_clear_intrbits_slave(struct gpt_i2c_dev *dev)
{
	u32 stat;

	/*
	 * The IC_INTR_STAT register just indicates "enabled" interrupts.
	 * Ths unmasked raw version of interrupt status bits are available
	 * in the IC_RAW_INTR_STAT register.
	 *
	 * That is,
	 *   stat = gpt_readl(IC_INTR_STAT);
	 * equals to,
	 *   stat = gpt_readl(IC_RAW_INTR_STAT) & gpt_readl(IC_INTR_MASK);
	 *
	 * The raw version might be useful for debugging purposes.
	 */
	stat = gpt_readl(dev->base + GPT_IC_INTR_STAT);

	/*
	 * Do not use the IC_CLR_INTR register to clear interrupts, or
	 * you'll miss some interrupts, triggered during the period from
	 * gpt_readl(IC_INTR_STAT) to gpt_readl(IC_CLR_INTR).
	 *
	 * Instead, use the separately-prepared IC_CLR_* registers.
	 */
	
	if (stat & GPT_IC_INTR_TX_ABRT)
		gpt_readl(dev->base + GPT_IC_CLR_TX_ABRT);
	if (stat & GPT_IC_INTR_RX_UNDER)
		gpt_readl(dev->base + GPT_IC_CLR_RX_UNDER);
	if (stat & GPT_IC_INTR_RX_OVER)
		gpt_readl(dev->base + GPT_IC_CLR_RX_OVER);
	if (stat & GPT_IC_INTR_TX_OVER)
		gpt_readl(dev->base + GPT_IC_CLR_TX_OVER);
	if (stat & GPT_IC_INTR_RX_DONE)
		gpt_readl(dev->base + GPT_IC_CLR_RX_DONE);

	if (stat & GPT_IC_INTR_ACTIVITY)
		gpt_readl(dev->base + GPT_IC_CLR_ACTIVITY);
	if (stat & GPT_IC_INTR_STOP_DET)
		gpt_readl(dev->base + GPT_IC_CLR_STOP_DET);
	if (stat & GPT_IC_INTR_START_DET)
		gpt_readl(dev->base + GPT_IC_CLR_START_DET);
	if (stat & GPT_IC_INTR_GEN_CALL)
		gpt_readl(dev->base + GPT_IC_CLR_GEN_CALL);

	return stat;
}

/**
 * i2c_gpt_init() - initialize the designware i2c master hargptare
 * @dev: device private data
 *
 * This functions configures and enables the I2C master.
 * This function is called during I2C init function, and in case of timeout at
 * run time.
 */
int i2c_gpt_init(struct gpt_i2c_dev *dev)
{
	

	/* Disable the i2c adapter */
	__i2c_gpt_enable(dev, false);


	/*define slave 7 bit address:0x21 */
	gpt_writel(0x21, dev->base + GPT_IC_SAR);

	/* configure the i2c slave */
	gpt_writel(dev->slave_cfg , dev->base + GPT_IC_CON);
	return 0;
}


static void i2c_gpt_xfer_init(struct gpt_i2c_dev *dev)
{


	printk("i2c gpt xfer init======\n");

	/* Disable the adapter */
	__i2c_gpt_enable(dev, false);

#if 0
	/* if the slave address is ten bit iddress, enable 10BITADDR */
	ic_con = gpt_readl(dev->base + GPT_IC_CON);
	if (msgs[dev->msg_write_idx].flags & I2C_M_TEN) {
		ic_con |= GPT_IC_CON_10BITADDR_MASTER;
		/*
		 * If I2C_DYNAMIC_TAR_UPDATE is set, the 10-bit addressing
		 * mode has to be enabled via bit 12 of IC_TAR register.
		 * We set it always as I2C_DYNAMIC_TAR_UPDATE can't be
		 * detected from registers.
		 */
		ic_tar = GPT_IC_TAR_10BITADDR_MASTER;
	} else {
		ic_con &= ~GPT_IC_CON_10BITADDR_MASTER;
	}

	gpt_writel(ic_con,dev->base + GPT_IC_CON);

	/*
	 * Set the slave (target) address and enable 10-bit addressing mode
	 * if applicable.
	 */
	gpt_writel(msgs[dev->msg_write_idx].addr | ic_tar, dev->base + GPT_IC_TAR);
#endif

	//set Slave address; 7bit
	gpt_writel(0x21, dev->base + GPT_IC_SAR);

	gpt_writel(dev->slave_cfg , dev->base + GPT_IC_CON);
	printk("ic_con:%x\n",dev->slave_cfg);
	/* enforce disabled interrupts (due to HW issues) */
	i2c_gpt_disable_int(dev);

	/* Enable the adapter */
	__i2c_gpt_enable(dev, true);

	/* Clear and enable interrupts */
	i2c_gpt_clear_int(dev);
	gpt_writel(GPT_IC_INTR_DEFAULT_MASK, dev->base + GPT_IC_INTR_MASK);
}



static int i2c_gpt_irq_handler_slave(struct gpt_i2c_dev *dev)
{


	printk("i2c gpt irq handler slave\n");

	u32  stat, enabled, tmp;
	int  val = 0, slave_activity;
	enabled = gpt_readl(dev->base + GPT_IC_ENABLE);
	stat = gpt_readl(dev->base + GPT_IC_RAW_INTR_STAT);
	tmp = gpt_readl(dev->base + GPT_IC_STATUS);
	slave_activity = ((tmp & GPT_IC_STATUS_SLAVE_ACTIVITY)>>6);

	if(stat & GPT_IC_INTR_RX_FULL){

		tmp = gpt_readl(dev->base + GPT_IC_DATA_CMD);
		val = tmp;
		slave_insert(val);
		printk("val:%x\n",val);
	}
	if(stat & GPT_IC_INTR_RD_REQ){

		val = slave_del();
		printk("write val:%x\n", val);
		gpt_writel(val, dev->base + GPT_IC_DATA_CMD);			
		gpt_readl(dev->base + GPT_IC_CLR_RD_REQ);
		gpt_readl(dev->base + GPT_IC_CLR_RX_UNDER);
		stat = i2c_gpt_read_clear_intrbits_slave(dev);	
	

	}

	return 1;

}

int i2c_gpt_reg_slave(struct i2c_client *slave)
{

	struct gpt_i2c_dev *dev = i2c_get_adapdata(slave->adapter);
	if(dev->slave)
		return -EBUSY;
	if(slave->flags & I2C_CLIENT_TEN);
		return -EAFNOSUPPORT;	

	pm_runtime_get_sync(dev->dev);

	i2c_gpt_xfer_init(dev);
	dev->slave = slave;

    __i2c_gpt_enable(dev, false);

	dev->cmd_err = 0;
	dev->msg_write_idx = 0;
	dev->msg_read_idx = 0;
	dev->msg_err = 0;
	dev->status = STATUS_IDLE;
	dev->abort_source = 0;
	dev->rx_outstanding = 0;

//	__i2c_gpt_enable(dev, false);

	return 0;
}

static int i2c_gpt_unreg_slave(struct i2c_client *slave)
{
	struct gpt_i2c_dev *dev = i2c_get_adapdata(slave->adapter);
	i2c_gpt_disable_int(dev);
	i2c_gpt_disable(dev);
	synchronize_irq(dev->irq);
	dev->slave = NULL;
	pm_runtime_put(dev->dev);

	return 0;

}

u32 i2c_gpt_func(struct i2c_adapter *adap)
{
	struct gpt_i2c_dev *dev = i2c_get_adapdata(adap);
	return dev->functionality;
}
	

/*
 * Interrupt service routine. This gets called whenever an I2C interrupt
 * occurs.
 */

irqreturn_t i2c_gpt_isr_slave(int this_irq, void *dev_id)
{

	struct gpt_i2c_dev *dev = dev_id;
	int ret;

	i2c_gpt_read_clear_intrbits_slave(dev);
	
	i2c_gpt_irq_handler_slave(dev);

	if(ret > 0)
		complete(&dev->cmd_complete);

	return IRQ_RETVAL(ret);


}


void i2c_gpt_enable(struct gpt_i2c_dev *dev)
{
       /* Enable the adapter */
	__i2c_gpt_enable(dev, true);
}

u32 i2c_gpt_is_enabled(struct gpt_i2c_dev *dev)
{
	return gpt_readl(dev->base + GPT_IC_ENABLE);
}

void i2c_gpt_disable(struct gpt_i2c_dev *dev)
{
	/* Disable controller */
	__i2c_gpt_enable(dev, false);

	/* Disable all interupts */
	gpt_writel(0, dev->base + GPT_IC_INTR_MASK);
	gpt_readl(dev->base + GPT_IC_CLR_INTR);
}

void i2c_gpt_clear_int(struct gpt_i2c_dev *dev)
{
	gpt_readl(dev->base + GPT_IC_CLR_INTR);
}

void i2c_gpt_disable_int(struct gpt_i2c_dev *dev)
{
	gpt_writel(0, dev->base + GPT_IC_INTR_MASK);
}

u32 i2c_gpt_read_comp_param(struct gpt_i2c_dev *dev)
{
	return gpt_readl(dev->base + GPT_IC_COMP_PARAM_1);
}

static struct i2c_algorithm i2c_gpt_algo = {
	.reg_slave = i2c_gpt_reg_slave,
	.unreg_slave = i2c_gpt_unreg_slave,
	.functionality	= i2c_gpt_func,
};

static void enable_i2c_ctrl(void __iomem *addr)
{
	writel(0x1fff5503, (void *)(addr + APIU_UCR));
}

static void gpt_inithw(struct gpt_i2c_dev *dev)
{
	apiu_unit_init(dev->base,0);
}


static int gpt_i2c_probe(struct platform_device *pdev)
{
	struct gpt_i2c_dev *dev;
	struct i2c_adapter *adap;
	struct resource *mem;
	int irq, r;

	printk("gpt_i2c_probe-------------\n");

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "no irq resource?\n");
		return irq; 
	}

	dev = devm_kzalloc(&pdev->dev, sizeof(struct gpt_i2c_dev), GFP_KERNEL);

	if (!dev)
		return -ENOMEM;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dev->base = devm_ioremap_resource(&pdev->dev, mem);

	gpt_inithw(dev);

	if (IS_ERR(dev->base))
		return PTR_ERR(dev->base);

	enable_i2c_ctrl(dev->base);
	init_completion(&dev->cmd_complete);
	mutex_init(&dev->lock);
	dev->dev = &pdev->dev;
	dev->irq = irq;
	platform_set_drvdata(pdev, dev);

	dev->functionality =
		I2C_FUNC_SLAVE |
		I2C_FUNC_10BIT_ADDR |
		I2C_FUNC_SMBUS_BYTE |
		I2C_FUNC_SMBUS_BYTE_DATA |
		I2C_FUNC_SMBUS_WORD_DATA |
		I2C_FUNC_SMBUS_I2C_BLOCK;

	//dev->master_cfg = 0x22;

	dev->slave_cfg = 0x24;

	//dev->master_cfg = 0x26;

	//dev->master_cfg = 0x3A;

	//dev->master_cfg = 0x3c;

	//dev->master_cfg = 0x3e;


	if (!dev->tx_fifo_depth) {
		u32 param1 = i2c_gpt_read_comp_param(dev);

		dev->tx_fifo_depth = ((param1 >> 16) & 0xff) + 1;
		dev->rx_fifo_depth = ((param1 >> 8)  & 0xff) + 1;
		dev->adapter.nr = pdev->id;
	}

	//init reg
	r = i2c_gpt_init(dev);
	if (r)
		return r;

	//disable INT
	i2c_gpt_disable_int(dev);

	r = devm_request_irq(&pdev->dev, dev->irq, i2c_gpt_isr_slave, IRQF_SHARED | IRQF_NO_THREAD,
			pdev->name, dev);
	if (r) {
		dev_err(&pdev->dev, "failure requesting irq %i\n", dev->irq);
		return r;
	}

	adap = &dev->adapter;
	i2c_set_adapdata(adap, dev);
	adap->owner = THIS_MODULE;
	adap->class = I2C_CLASS_DEPRECATED;
	strlcpy(adap->name, "gpt I2C adapter",
			sizeof(adap->name));
	adap->algo = &i2c_gpt_algo;
	adap->dev.parent = &pdev->dev;
	adap->dev.of_node = pdev->dev.of_node;

	r = i2c_add_numbered_adapter(adap);
	if (r) {
		dev_err(&pdev->dev, "failure adding adapter\n");
		return r;
	}
	

	i2c_gpt_clear_int(dev);//clear INT
	gpt_writel(GPT_IC_INTR_DEFAULT_MASK, dev->base + GPT_IC_INTR_MASK);//enable INT
	i2c_gpt_enable(dev);//enable i2c adapter


	pm_runtime_set_autosuspend_delay(&pdev->dev, 1000);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	return 0;
}



static int gpt_i2c_remove(struct platform_device *pdev)
{
	struct gpt_i2c_dev *dev = platform_get_drvdata(pdev);

	pm_runtime_get_sync(&pdev->dev);

	i2c_del_adapter(&dev->adapter);

	i2c_gpt_disable(dev);

	pm_runtime_put(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	return 0;
}

static int gpt_i2c_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gpt_i2c_dev *i_dev = platform_get_drvdata(pdev);

	i2c_gpt_disable(i_dev);
	clk_disable_unprepare(i_dev->clk);

	return 0;
}

static int gpt_i2c_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gpt_i2c_dev *i_dev = platform_get_drvdata(pdev);

	clk_prepare_enable(i_dev->clk);
	i2c_gpt_init(i_dev);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id gpt_i2c_of_match[] = {
	{ .compatible = "gpt,i2c0.0", },
	{ .compatible = "gpt,i2c0.1", },
	{ .compatible = "gpt,i2c0.2", },
	{},
};
MODULE_DEVICE_TABLE(of, gpt_i2c_of_match);
#endif


static SIMPLE_DEV_PM_OPS(gpt_i2c_pm, gpt_i2c_suspend, gpt_i2c_resume);

/* work with hotplug and coldplug */
MODULE_ALIAS("platform:i2c_gpt");

static struct platform_driver gpt_i2c_driver = {
	.probe = gpt_i2c_probe,
	.remove = gpt_i2c_remove,
	.driver		= {
		.name	= "i2c_gpt",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(gpt_i2c_of_match),
		.pm    = &gpt_i2c_pm,
	},
};

static int __init gpt_i2c_init_driver(void)
{
	return platform_driver_register(&gpt_i2c_driver);
}
subsys_initcall(gpt_i2c_init_driver);

static void __exit gpt_i2c_exit_driver(void)
{
	platform_driver_unregister(&gpt_i2c_driver);
}
module_exit(gpt_i2c_exit_driver);

MODULE_AUTHOR("GPT Inc.");
MODULE_DESCRIPTION("GPT I2C bus slave adapter");
MODULE_LICENSE("GPL");
		

