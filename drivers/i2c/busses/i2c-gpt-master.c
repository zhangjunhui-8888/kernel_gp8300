/*
 * GPT  I2C adapter driver (master only).
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
					 GPT_IC_INTR_TX_EMPTY | \
					 GPT_IC_INTR_TX_ABRT | \
					 GPT_IC_INTR_STOP_DET)


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


static u32
i2c_gpt_scl_hcnt(u32 ic_clk, u32 tSYMBOL, u32 tf, int cond, int offset)
{
	/*
	 * DesignWare I2C core doesn't seem to have solid strategy to meet
	 * the tHD;STA timing spec.  Configuring _HCNT based on tHIGH spec
	 * will result in violation of the tHD;STA spec.
	 */
	if (cond)
		/*
		 * Conditional expression:
		 *
		 *   IC_[FS]S_SCL_HCNT + (1+4+3) >= IC_CLK * tHIGH
		 *
		 * This is based on the GPT manuals, and represents an ideal
		 * configuration.  The resulting I2C bus speed will be
		 * faster than any of the others.
		 *
		 * If your hargptare is free from tHD;STA issue, try this one.
		 */
		return (ic_clk * tSYMBOL + 500000) / 1000000 - 8 + offset;
	else
		/*
		 * Conditional expression:
		 *
		 *   IC_[FS]S_SCL_HCNT + 3 >= IC_CLK * (tHD;STA + tf)
		 *
		 * This is just experimental rule; the tHD;STA period turned
		 * out to be proportinal to (_HCNT + 3).  With this setting,
		 * we could meet both tHIGH and tHD;STA timing specs.
		 *
		 * If unsure, you'd better to take this alternative.
		 *
		 * The reason why we need to take into account "tf" here,
		 * is the same as described in i2c_gpt_scl_lcnt().
		 */
		return (ic_clk * (tSYMBOL + tf) + 500000) / 1000000
			- 3 + offset;
}

static u32 i2c_gpt_scl_lcnt(u32 ic_clk, u32 tLOW, u32 tf, int offset)
{
	/*
	 * Conditional expression:
	 *
	 *   IC_[FS]S_SCL_LCNT + 1 >= IC_CLK * (tLOW + tf)
	 *
	 * GPT I2C core starts counting the SCL CNTs for the LOW period
	 * of the SCL clock (tLOW) as soon as it pulls the SCL line.
	 * In order to meet the tLOW timing spec, we need to take into
	 * account the fall time of SCL signal (tf).  Default tf value
	 * should be 0.3 us, for safety.
	 */
	return ((ic_clk * (tLOW + tf) + 500000) / 1000000) - 1 + offset;
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
	u32 input_clock_khz;
	u32 hcnt, lcnt;
	//u32 reg;
	u32 sda_falling_time, scl_falling_time;

	input_clock_khz = dev->get_clk_rate_khz(dev);

	//reg = gpt_readl(dev->base + GPT_IC_COMP_TYPE);
	/*
	reg = 0x44570140;
	if (reg == ___constant_swab32(GPT_IC_COMP_TYPE_VALUE)) {
		// Configure register endianess access
		dev->accessor_flags |= ACCESS_SWAP;
	} else if (reg == (GPT_IC_COMP_TYPE_VALUE & 0x0000ffff)) {
		// Configure register access mode 16bit
		dev->accessor_flags |= ACCESS_16BIT;
	} else if (reg != GPT_IC_COMP_TYPE_VALUE) {
		dev_err(dev->dev, "Unknown GPT component type: "
			"0x%08x\n", reg);
		return -ENODEV;
	}
	*/

	/* Disable the adapter */
	__i2c_gpt_enable(dev, false);

	/* set standard and fast speed deviders for high/low periods */

	sda_falling_time = dev->sda_falling_time ?: 300; /* ns */
	scl_falling_time = dev->scl_falling_time ?: 300; /* ns */

	/* Standard-mode */
	hcnt = i2c_gpt_scl_hcnt(input_clock_khz,
				4000,	/* tHD;STA = tHIGH = 4.0 us */
				sda_falling_time,
				0,	/* 0: GPT default, 1: Ideal */
				0);	/* No offset */
	lcnt = i2c_gpt_scl_lcnt(input_clock_khz,
				4700,	/* tLOW = 4.7 us */
				scl_falling_time,
				0);	/* No offset */

	/* Allow platforms to specify the ideal HCNT and LCNT values */
	if (dev->ss_hcnt && dev->ss_lcnt) {
		hcnt = dev->ss_hcnt;
		lcnt = dev->ss_lcnt;
	}
	gpt_writel(hcnt, dev->base + GPT_IC_SS_SCL_HCNT);
	gpt_writel(lcnt, dev->base + GPT_IC_SS_SCL_LCNT);

	/* Fast-mode */
	hcnt = i2c_gpt_scl_hcnt(input_clock_khz,
				600,	/* tHD;STA = tHIGH = 0.6 us */
				sda_falling_time,
				0,	/* 0: GPT default, 1: Ideal */
				0);	/* No offset */
	lcnt = i2c_gpt_scl_lcnt(input_clock_khz,
				1300,	/* tLOW = 1.3 us */
				scl_falling_time,
				0);	/* No offset */

	if (dev->fs_hcnt && dev->fs_lcnt) {
		hcnt = dev->fs_hcnt;
		lcnt = dev->fs_lcnt;
	}
	gpt_writel(hcnt,dev->base + GPT_IC_FS_SCL_HCNT);
	gpt_writel(lcnt,dev->base + GPT_IC_FS_SCL_LCNT);

	/* high-speed-mode */
	hcnt = i2c_gpt_scl_hcnt(input_clock_khz,
				60,	/* tHD;STA = tHIGH = 0.06 us */
				sda_falling_time,
				0,	/* 0: GPT default, 1: Ideal */
				0);	/* No offset */
	lcnt = i2c_gpt_scl_lcnt(input_clock_khz,
				160,	/* tLOW = 0.16 us */
				scl_falling_time,
				0);	/* No offset */

	if (dev->fs_hcnt && dev->fs_lcnt) {
		hcnt = dev->hs_hcnt;
		lcnt = dev->hs_lcnt;
	}
	gpt_writel(hcnt,dev->base + GPT_IC_HS_SCL_HCNT);
	gpt_writel(lcnt,dev->base + GPT_IC_HS_SCL_LCNT);


	/* Configure SDA Hold Time if required */
	if (dev->sda_hold_time) {
		/*
		reg = gpt_readl(dev->base + GPT_IC_COMP_VERSION);
		if (reg >= GPT_IC_SDA_HOLD_MIN_VERS)
			gpt_writel(dev->sda_hold_time,dev->base + GPT_IC_SDA_HOLD);
		else
			dev_warn(dev->dev,
				"Hargptare too old to adjust SDA hold time.");
				*/
		gpt_writel(dev->sda_hold_time,dev->base + GPT_IC_SDA_HOLD);
	}

	/* Configure Tx/Rx FIFO threshold levels */
	gpt_writel(dev->tx_fifo_depth / 2,dev->base + GPT_IC_TX_TL);
	gpt_writel(0,dev->base + GPT_IC_RX_TL);

	/* configure the i2c master */
	gpt_writel(dev->master_cfg , dev->base + GPT_IC_CON);
	return 0;
}

/*
 * Waiting for bus not busy
 */
static int i2c_gpt_wait_bus_not_busy(struct gpt_i2c_dev *dev)
{
	int timeout = TIMEOUT;

	while (gpt_readl(dev->base + GPT_IC_STATUS) & GPT_IC_STATUS_ACTIVITY) {
		if (timeout <= 0) {
			dev_warn(dev->dev, "timeout waiting for bus ready\n");
			return -ETIMEDOUT;
		}
		timeout--;
		usleep_range(1000, 1100);
	}

	return 0;
}

static void i2c_gpt_xfer_init(struct gpt_i2c_dev *dev)
{
	struct i2c_msg *msgs = dev->msgs;
	u32 ic_con, ic_tar = 0;

	/* Disable the adapter */
	__i2c_gpt_enable(dev, false);

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

	/* enforce disabled interrupts (due to HW issues) */
	i2c_gpt_disable_int(dev);

	/* Enable the adapter */
	__i2c_gpt_enable(dev, true);

	/* Clear and enable interrupts */
	i2c_gpt_clear_int(dev);
	gpt_writel(GPT_IC_INTR_DEFAULT_MASK, dev->base + GPT_IC_INTR_MASK);
}

/*
 * Initiate (and continue) low level master read/write transaction.
 * This function is only called from i2c_gpt_isr, and pumping i2c_msg
 * messages into the tx buffer.  Even if the size of i2c_msg data is
 * longer than the size of the tx buffer, it handles everything.
 */
static void
i2c_gpt_xfer_msg(struct gpt_i2c_dev *dev)
{
	struct i2c_msg *msgs = dev->msgs;
	u32 intr_mask;
	int tx_limit, rx_limit;
	u32 addr = msgs[dev->msg_write_idx].addr;
	u32 buf_len = dev->tx_buf_len;
	u8 *buf = dev->tx_buf;
	bool need_restart = false;

	intr_mask = GPT_IC_INTR_DEFAULT_MASK;

	for (; dev->msg_write_idx < dev->msgs_num; dev->msg_write_idx++) {
		/*
		 * if target address has changed, we need to
		 * reprogram the target address in the i2c
		 * adapter when we are done with this transfer
		 */
		if (msgs[dev->msg_write_idx].addr != addr) {
			dev_err(dev->dev,
				"%s: invalid target address\n", __func__);
			dev->msg_err = -EINVAL;
			break;
		}

		if (msgs[dev->msg_write_idx].len == 0) {
			dev_err(dev->dev,
				"%s: invalid message length\n", __func__);
			dev->msg_err = -EINVAL;
			break;
		}

		if (!(dev->status & STATUS_WRITE_IN_PROGRESS)) {
			/* new i2c_msg */
			buf = msgs[dev->msg_write_idx].buf;
			buf_len = msgs[dev->msg_write_idx].len;

			/* If both IC_EMPTYFIFO_HOLD_MASTER_EN and
			 * IC_RESTART_EN are set, we must manually
			 * set restart bit between messages.
			 */
			if ((dev->master_cfg & GPT_IC_CON_RESTART_EN) &&
					(dev->msg_write_idx > 0))
				need_restart = true;
		}

		tx_limit = dev->tx_fifo_depth - gpt_readl(dev->base + GPT_IC_TXFLR);
		rx_limit = dev->rx_fifo_depth - gpt_readl(dev->base + GPT_IC_RXFLR);

		while (buf_len > 0 && tx_limit > 0 && rx_limit > 0) {
			u32 cmd = 0;

			/*
			 * If IC_EMPTYFIFO_HOLD_MASTER_EN is set we must
			 * manually set the stop bit. However, it cannot be
			 * detected from the registers so we set it always
			 * when writing/reading the last byte.
			 */
			if (dev->msg_write_idx == dev->msgs_num - 1 &&
			    buf_len == 1)
				cmd |= BIT(9);

			if (need_restart) {
				cmd |= BIT(10);
				need_restart = false;
			}

			if (msgs[dev->msg_write_idx].flags & I2C_M_RD) {

				/* avoid rx buffer overrun */
				if (rx_limit - dev->rx_outstanding <= 0)
					break;

				gpt_writel(cmd | 0x100, dev->base + GPT_IC_DATA_CMD);
				rx_limit--;
				dev->rx_outstanding++;
			} else
				gpt_writel(cmd | *buf++, dev->base + GPT_IC_DATA_CMD);
			tx_limit--; buf_len--;
		}

		dev->tx_buf = buf;
		dev->tx_buf_len = buf_len;

		if (buf_len > 0) {
			/* more bytes to be written */
			dev->status |= STATUS_WRITE_IN_PROGRESS;
			break;
		} else
			dev->status &= ~STATUS_WRITE_IN_PROGRESS;
	}

	/*
	 * If i2c_msg index search is completed, we don't need TX_EMPTY
	 * interrupt any more.
	 */
	if (dev->msg_write_idx == dev->msgs_num)
		intr_mask &= ~GPT_IC_INTR_TX_EMPTY;

	if (dev->msg_err)
		intr_mask = 0;

	gpt_writel(intr_mask,  dev->base + GPT_IC_INTR_MASK);
}

static void
i2c_gpt_read(struct gpt_i2c_dev *dev)
{
	struct i2c_msg *msgs = dev->msgs;
	int rx_valid;
	int tt;

	for (; dev->msg_read_idx < dev->msgs_num; dev->msg_read_idx++) {
		u32 len;
		u8 *buf;

		if (!(msgs[dev->msg_read_idx].flags & I2C_M_RD))
			continue;

		if (!(dev->status & STATUS_READ_IN_PROGRESS)) {
			len = msgs[dev->msg_read_idx].len;
			buf = msgs[dev->msg_read_idx].buf;
		} else {
			len = dev->rx_buf_len;
			buf = dev->rx_buf;
		}

		rx_valid = gpt_readl(dev->base +GPT_IC_RXFLR);

		for (; len > 0 && rx_valid > 0; len--, rx_valid--) {
			tt = gpt_readl(dev->base + GPT_IC_DATA_CMD);
			*buf = (char)tt;
			buf++;
			dev->rx_outstanding--;
		}

		if (len > 0) {
			dev->status |= STATUS_READ_IN_PROGRESS;
			dev->rx_buf_len = len;
			dev->rx_buf = buf;
			return;
		} else
			dev->status &= ~STATUS_READ_IN_PROGRESS;
	}
}

static int i2c_gpt_handle_tx_abort(struct gpt_i2c_dev *dev)
{
	unsigned long abort_source = dev->abort_source;
	int i;


	if (abort_source & GPT_IC_TX_ABRT_NOACK) {
		for_each_set_bit(i, &abort_source, ARRAY_SIZE(abort_sources))
			dev_dbg(dev->dev,"%s: %s\n", __func__, abort_sources[i]);

		return -EREMOTEIO;
	}

	for_each_set_bit(i, &abort_source, ARRAY_SIZE(abort_sources))
		dev_err(dev->dev, "%s: %s\n", __func__, abort_sources[i]);

	if (abort_source & GPT_IC_TX_ARB_LOST)
		return -EAGAIN;
	else if (abort_source & GPT_IC_TX_ABRT_GCALL_READ)
		return -EINVAL; /* wrong msgs[] data */
	else
		return -EIO;
}

/*
 * Prepare controller for a transaction and call i2c_gpt_xfer_msg
 * send or receive data
 * the data information is in the i2c_msg struct
 */
int i2c_gpt_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[], int num)
{
	struct gpt_i2c_dev *dev = i2c_get_adapdata(adap);
	int ret;
	dev_dbg(dev->dev, "%s: msgs: %d\n", __func__, num);

	mutex_lock(&dev->lock);
	pm_runtime_get_sync(dev->dev);

	reinit_completion(&dev->cmd_complete);
	dev->msgs = msgs;
	dev->msgs_num = num;
	dev->cmd_err = 0;
	dev->msg_write_idx = 0;
	dev->msg_read_idx = 0;
	dev->msg_err = 0;
	dev->status = STATUS_IDLE;
	dev->abort_source = 0;
	dev->rx_outstanding = 0;

	msleep(1);/* for gp8300 demo board i2c through fpga, must be del nornal board*/
	ret = i2c_gpt_wait_bus_not_busy(dev);
	if (ret < 0)
		goto done;

	/* start the transfers */
	i2c_gpt_xfer_init(dev);

	/* wait for tx to complete */
	ret = wait_for_completion_timeout(&dev->cmd_complete, HZ);
	if (ret == 0) {
		dev_err(dev->dev, "controller timed out\n");
		/* i2c_gpt_init implicitly disables the adapter */
		i2c_gpt_init(dev);
		ret = -ETIMEDOUT;
		goto done;
	}

	/*
	 * We must disable the adapter before unlocking the &dev->lock mutex
	 * below. Otherwise the hargptare might continue generating interrupts
	 * which in turn causes a race condition with the following transfer.
	 * Needs some more investigation if the additional interrupts are
	 * a hargptare bug or this driver doesn't handle them correctly yet.
	 */
	__i2c_gpt_enable(dev, false);

	if (dev->msg_err) {
		ret = dev->msg_err;
		goto done;
	}

	/* no error */
	if (likely(!dev->cmd_err)) {
		ret = num;
		goto done;
	}

	/* We have an error */
	if (dev->cmd_err == GPT_IC_ERR_TX_ABRT) {
		ret = i2c_gpt_handle_tx_abort(dev);
		goto done;
	}
	ret = -EIO;

done:
	pm_runtime_mark_last_busy(dev->dev);
	pm_runtime_put_autosuspend(dev->dev);
	mutex_unlock(&dev->lock);

	return ret;
}

u32 i2c_gpt_func(struct i2c_adapter *adap)
{
	struct gpt_i2c_dev *dev = i2c_get_adapdata(adap);
	return dev->functionality;
}

static u32 i2c_gpt_read_clear_intrbits(struct gpt_i2c_dev *dev)
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
	if (stat & GPT_IC_INTR_RX_UNDER)
		gpt_readl(dev->base + GPT_IC_CLR_RX_UNDER);
	if (stat & GPT_IC_INTR_RX_OVER)
		gpt_readl(dev->base + GPT_IC_CLR_RX_OVER);
	if (stat & GPT_IC_INTR_TX_OVER)
		gpt_readl(dev->base + GPT_IC_CLR_TX_OVER);
	if (stat & GPT_IC_INTR_RD_REQ)
		gpt_readl(dev->base + GPT_IC_CLR_RD_REQ);
	if (stat & GPT_IC_INTR_TX_ABRT) {
		/*
		 * The IC_TX_ABRT_SOURCE register is cleared whenever
		 * the IC_CLR_TX_ABRT is read.  Preserve it beforehand.
		 */
		SP_PRINTK("func:clear abrt = %x\n",gpt_readl(dev->base + GPT_IC_TX_ABRT_SOURCE));
		dev->abort_source = gpt_readl(dev->base + GPT_IC_TX_ABRT_SOURCE);
		gpt_readl(dev->base + GPT_IC_CLR_TX_ABRT);
	}
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

/*
 * Interrupt service routine. This gets called whenever an I2C interrupt
 * occurs.
 */
irqreturn_t i2c_gpt_isr(int this_irq, void *dev_id)
{
	struct gpt_i2c_dev *dev = dev_id;
	unsigned long flags = 0;
	u32 stat, enabled;

	local_irq_save(flags);
	enabled = gpt_readl(dev->base + GPT_IC_ENABLE);
	stat = gpt_readl(dev->base + GPT_IC_RAW_INTR_STAT);

	dev_dbg(dev->dev, "%s:  %s enabled= 0x%x stat=0x%x\n", __func__,
		dev->adapter.name, enabled, stat);
	if (!enabled || !(stat & ~GPT_IC_INTR_ACTIVITY))
		return IRQ_NONE;


	stat = i2c_gpt_read_clear_intrbits(dev);
	if (stat & GPT_IC_INTR_TX_ABRT) {
		dev->cmd_err |= GPT_IC_ERR_TX_ABRT;
		dev->status = STATUS_IDLE;

		/*
		 * Anytime TX_ABRT is set, the contents of the tx/rx
		 * buffers are flushed.  Make sure to skip them.
		 */
		gpt_writel(0, dev->base + GPT_IC_INTR_MASK);
		goto tx_aborted;
	}

	if (stat & GPT_IC_INTR_RX_FULL)
		i2c_gpt_read(dev);

	if (stat & GPT_IC_INTR_TX_EMPTY)
		i2c_gpt_xfer_msg(dev);

	/*
	 * No need to modify or disable the interrupt mask here.
	 * i2c_gpt_xfer_msg() will take care of it according to
	 * the current transmit status.
	 */

tx_aborted:
	if ((stat & (GPT_IC_INTR_TX_ABRT | GPT_IC_INTR_STOP_DET)) || dev->msg_err)
		complete(&dev->cmd_complete);
	local_irq_restore(flags);
	return IRQ_HANDLED;
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
	.master_xfer	= i2c_gpt_xfer,
	.functionality	= i2c_gpt_func,
};
static u32 i2c_gpt_get_clk_rate_khz(struct gpt_i2c_dev *dev)
{
	return clk_get_rate(dev->clk)/1000;
}

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
	struct gpt_i2c_platform_data *pdata;
	int irq, r;
	u32 clk_freq, ht = 0;

	printk("entry %s %d-----------------------------\n",__func__,__LINE__);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "no irq resource?\n");
		return irq; /* -ENXIO */
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

	/* stand mode */
	//clk_freq = 100000;

	/* fast mode by default because of legacy reasons */
	clk_freq = 400000;

	/* high speed mode */
	//clk_freq = 3400000;

	if (pdev->dev.of_node) {
		of_property_read_u32(pdev->dev.of_node,
					"i2c-sda-hold-time-ns", &ht);

		of_property_read_u32(pdev->dev.of_node,
				     "i2c-sda-falling-time-ns",
				     &dev->sda_falling_time);
		of_property_read_u32(pdev->dev.of_node,
				     "i2c-scl-falling-time-ns",
				     &dev->scl_falling_time);

		of_property_read_u32(pdev->dev.of_node, "clock-frequency",
				     &clk_freq);

	} else {
		pdata = dev_get_platdata(&pdev->dev);
		if (pdata)
			clk_freq = pdata->i2c_scl_freq;
	}


	dev->functionality =
		I2C_FUNC_I2C |
		I2C_FUNC_10BIT_ADDR |
		I2C_FUNC_SMBUS_BYTE |
		I2C_FUNC_SMBUS_BYTE_DATA |
		I2C_FUNC_SMBUS_WORD_DATA |
		I2C_FUNC_SMBUS_I2C_BLOCK;


	if (clk_freq <= 100000)
		dev->master_cfg =  GPT_IC_CON_MASTER | GPT_IC_CON_SLAVE_DISABLE |
			GPT_IC_CON_RESTART_EN | GPT_IC_CON_SPEED_STD;
	else if((clk_freq <= 400000) && (clk_freq >= 100000))
		dev->master_cfg =  GPT_IC_CON_MASTER | GPT_IC_CON_SLAVE_DISABLE |
			GPT_IC_CON_RESTART_EN | GPT_IC_CON_SPEED_FAST;
	else if((clk_freq <= 3400000) && (clk_freq >= 400000))
		dev->master_cfg =  GPT_IC_CON_MASTER | GPT_IC_CON_SLAVE_DISABLE |
			GPT_IC_CON_RESTART_EN | GPT_IC_CON_SPEED_HIGH;



	dev->clk = devm_clk_get(&pdev->dev, NULL);
	dev->get_clk_rate_khz = i2c_gpt_get_clk_rate_khz;
	if (IS_ERR(dev->clk))
		return PTR_ERR(dev->clk);
	clk_prepare_enable(dev->clk);

	if (!dev->sda_hold_time && ht) {
		u32 ic_clk = dev->get_clk_rate_khz(dev);

		dev->sda_hold_time = div_u64((u64)ic_clk * ht + 500000,
					     1000000);
	}

	if (!dev->tx_fifo_depth) {
		u32 param1 = i2c_gpt_read_comp_param(dev);

		dev->tx_fifo_depth = ((param1 >> 16) & 0xff) + 1;
		dev->rx_fifo_depth = ((param1 >> 8)  & 0xff) + 1;
		dev->adapter.nr = pdev->id;
	}
	r = i2c_gpt_init(dev);
	if (r)
		return r;

	i2c_gpt_disable_int(dev);
	r = devm_request_irq(&pdev->dev, dev->irq, i2c_gpt_isr, IRQF_SHARED | IRQF_NO_THREAD,
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

#ifdef CONFIG_OF
static const struct of_device_id gpt_i2c_of_match[] = {
	{ .compatible = "gpt,i2c0.0", },
	{ .compatible = "gpt,i2c0.1", },
	{ .compatible = "gpt,i2c0.2", },
	{},
};
MODULE_DEVICE_TABLE(of, gpt_i2c_of_match);
#endif

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
MODULE_DESCRIPTION("GPT I2C bus adapter");
MODULE_LICENSE("GPL");
