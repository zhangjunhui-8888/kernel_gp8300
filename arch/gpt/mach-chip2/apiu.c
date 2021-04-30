/*
 * linux/arch/gpt/mach-chip2/apiu.c
 * 
 *
 * APB Peripheral Interface Unit (APIU)
 *
 * Copyright (C) 2016, General Processor Techologies Inc.
 * 
 * scxie <scxie@hxgpt.com>
 * Nick Wu <fwu@hxgpt.com>
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

/**
 * The APB_Peripheral Interface Unit (APIU) provides access to two UARTs and two 8-bit
 * GPIO blocks. The APIU is accessed through a direct memory path, with internal
 * translation (onto an APB bus) performed for communicating with the APB compliant
 * peripherals.
 *
 * register format:
 * 31------------------16-----12---------2----------1----0
 * |	   F000        | unit | reg addr |uart burst| w/r|
 * +-------------------+------+----------+----------+----+
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/string.h>
#include <linux/io.h>
#include <linux/amba/bus.h>
#include <linux/sizes.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <asm/irq.h>
#include <asm/mach-chip2/apiu.h>
#include <asm/delay.h>

static DEFINE_SPINLOCK(lock);

static inline void apiu_tx_fifo_full(void __iomem *addr)
{
#if 1
	u32 status;
	do{
		status = readl((void*)(((u64)addr & DEV_MASK) | APIU_USFR));
	}while(status & UNIT_RAW_RQST_FULL);
#else
	udelay(1);
#endif
}
static inline void apiu_rx_fifo_not_empty(void __iomem *addr)
{
#if 1
	u32 status;
	do{
		status = readl((void*)(((u64)addr & DEV_MASK) | APIU_USFR));
	}while(!(status & UNIT_RAW_RESP_DAVAIL));
#else
	udelay(1);
#endif
}

u32 apiu_readl(void __iomem *addr)
{
	u32 ret;
	unsigned long flags;
	spin_lock_irqsave(&lock, flags);
	apiu_tx_fifo_full(addr);
	writeb(0, (void*)((u64)addr | APIU_RQST_FIFO_READ));
	apiu_rx_fifo_not_empty(addr);
	ret = readl(addr);
	spin_unlock_irqrestore(&lock, flags);
	return ret;
}
EXPORT_SYMBOL(apiu_readl);

u16 apiu_readw(void __iomem *addr)
{
	u16 ret;
	unsigned long flags;
	spin_lock_irqsave(&lock, flags);
	apiu_tx_fifo_full(addr);
	writeb(0, (void*)((u64)addr | APIU_RQST_FIFO_READ));
	apiu_rx_fifo_not_empty(addr);
	ret = readw(addr);
	spin_unlock_irqrestore(&lock, flags);
	return ret & 0xffff;
}
EXPORT_SYMBOL(apiu_readw);

void apiu_writel(u32 val, void __iomem *addr)
{ 
	unsigned long flags;
	spin_lock_irqsave(&lock, flags);
	apiu_tx_fifo_full(addr);
	writel(val, addr);
	spin_unlock_irqrestore(&lock, flags);
}
EXPORT_SYMBOL(apiu_writel);

void apiu_writeb(u8 val, void __iomem *addr)
{
	unsigned long flags;
	spin_lock_irqsave(&lock, flags);
	apiu_tx_fifo_full(addr);
	writeb(val, addr);
	spin_unlock_irqrestore(&lock, flags);
}
EXPORT_SYMBOL(apiu_writeb);

void apiu_writew(u16 val, void __iomem *addr)
{
	unsigned long flags;
	spin_lock_irqsave(&lock, flags);
	apiu_tx_fifo_full(addr);
	writew(val, addr);
	spin_unlock_irqrestore(&lock, flags);
}
EXPORT_SYMBOL(apiu_writew);

u32 apiu_burst_readl(void __iomem *addr)
{ 
	u32 status;
#if 1
	do{
		status = readl((void*)(((u64)addr & DEV_MASK) | APIU_USFR));
	} while (status & UNIT_RAW_BRX_EPTY);
#else
	udealy(10);
#endif
	return readl((void*)(((u64)addr & DEV_MASK) | APIU_BURST_ADDR));
}
EXPORT_SYMBOL(apiu_burst_readl);

void apiu_burst_writel(u32 val, void __iomem *addr)
{ 
	u32 status;
#if 1
	do{
		status = readl((void*)(((u64)addr & DEV_MASK) | APIU_USFR));
	} while (status & UNIT_RAW_BTX_FULL);
#else
	udelay(10);
#endif
	writel(val, (void*)(((u64)addr & DEV_MASK) | APIU_BURST_ADDR));
}
EXPORT_SYMBOL(apiu_burst_writel);

u8 apiu_burst_readb(void __iomem *addr)
{
	return apiu_burst_readl(addr) & 0xff;
}
EXPORT_SYMBOL(apiu_burst_readb);

void apiu_unit_init(void __iomem *addr, int burst)
{
	u32 value;

	value = UNIT_IRQ_EN | UNIT_BURST_DFLT | UNIT_EN;
	if(!burst)
		value |= UNIT_BQ_DIS;
	writel(value, (void *)((u64)addr | APIU_UCR));
}
EXPORT_SYMBOL(apiu_unit_init);

void apiu_unit_uninit(void __iomem *addr)
{
	writel(0, (void *)((u64)addr | APIU_UCR));	
}
EXPORT_SYMBOL(apiu_unit_uninit);

static int __init apiu_init(void)
{
	return 0;
}
arch_initcall(apiu_init);
