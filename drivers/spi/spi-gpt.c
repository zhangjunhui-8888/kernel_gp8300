/*
 * GPT SPI controller driver (master mode only)
 *
 * Author: MontaVista Software, Inc.
 *	source@mvista.com
 *
 * Copyright (c) 2010 Secret Lab Technologies, Ltd.
 * Copyright (c) 2009 Intel Corporation
 * 2002-2007 (c) MontaVista Software, Inc.

 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define DEBUG
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/gpt_spi.h>
#include <asm/mach-chip2/apiu.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/clk.h>

#ifdef CONFIG_APIU
#define ioread32(addr) apiu_readl(addr)
#define iowrite32(val,addr) apiu_writel(val,addr)
#endif

#define AF_PRINT
#ifdef  AF_PRINT
#define  SP_PRINTK(format,arg...) do { \
	printk(format,##arg);\
}while(0)
#else
#define  SP_PRINTK(format,arg...) do { \
}while(0)
#endif


#define GPT_SPI_NAME "gptspi"
#define CONFIG_SYS_MHZ  50
#define MHz (1000*1000)
/* Register offsets */
#define GPT_SPI_CFG			0x00
#define	GPT_SPI_DIV_CNT			0x04
#define	GPT_SPI_BIT_MODE		0x08
#define	GPT_SPI_DLY_CNT			0x0c
#define	GPT_SPI_TRANSFER_CNT		0x10
#define	GPT_SPI_FIFO_CFG		0x14
#define	GPT_SPI_FIFO_CLEAR		0x18
#define	GPT_SPI_IRQ_RAW_STATUS		0x1c
#define	GPT_SPI_IRQ_STATUS		0x20
#define	GPT_SPI_IRQ_MASK		0x24
#define	GPT_SPI_IRQ_CLEAR		0x28
#define	GPT_SPI_RX_DATA			0x2c
#define	GPT_SPI_TX_DATA			0x30
#define	GPT_SPI_FIFO_CUR_CNT		0x34
#define	GPT_SPI_FIFO_OP			0x38
#define	GPT_SPI_SSN_MANUAL	0x3c

/* spi config register */
#define	GPT_SPI_MODE_MASTER		1 << 0
#define	GPT_SPI_MODE_SLAVE		0 << 0
#define	GPT_SPI_CPOL			1 << 1
#define	GPT_SPI_CPHA			1 << 2
#define	GPT_SPI_MSB			1 << 3
#define	GPT_SPI_LSB			0 << 3
#define	GPT_SPI_SSN_SEL	 1 << 4
#define GPT_SPI_SLV_XFER_SAME_EDGE 1 << 5
#define	GPT_SPI_TRANSFER_EN		1 << 29
#define	GPT_SPI_TX_DMA_EN		1 << 30
#define	GPT_SPI_RX_DMA_EN		1 << 31

#define	GPT_SPI_BIT_32			0
#define	GPT_SPI_BIT_16			2
#define	GPT_SPI_BIT_8			  1

/* spi fifo config register */
#define	GPT_SPI_TX_ALMOST_FULL_WIDTH	8
#define	GPT_SPI_TX_ALMOST_FULL_OFFSET	0
#define	GPT_SPI_TX_ALMOST_EMPTY_WIDTH	8
#define	GPT_SPI_TX_ALMOST_EMPTY_OFFSET	8
#define	GPT_SPI_RX_ALMOST_FULL_WIDTH	8
#define	GPT_SPI_RX_ALMOST_FULL_OFFSET	16
#define	GPT_SPI_RX_ALMOST_EMPTY_WIDTH	8
#define	GPT_SPI_RX_ALMOST_EMPTY_OFFSET	24

/* spi fifo clear register */
#define	GPT_SPI_TX_FLUSH			1 << 0
#define	GPT_SPI_RX_FLUSH			1 << 1

/* spi irq raw status register */
#define	GPT_SPI_TX_FIFO_ALMOST_FULL_CLR		1 << 0
#define	GPT_SPI_TX_FIFO_ALMOST_EMPTY_CLR	1 << 1
#define	GPT_SPI_TX_FIFO_FULL_CLR		1 << 2
#define	GPT_SPI_TX_FIFO_EMPTY_CLR		1 << 3
#define	GPT_SPI_TX_FIFO_OVERFLOW_CLR		1 << 4
#define	GPT_SPI_TX_FIFO_UNDERFLOW_CLR		1 << 5
#define	GPT_SPI_RX_FIFO_ALMOST_FULL_CLR		1 << 6
#define	GPT_SPI_RX_FIFO_ALMOST_EMPTY_CLR	1 << 7
#define	GPT_SPI_RX_FIFO_FULL_CLR		1 << 8
#define	GPT_SPI_RX_FIFO_EMPTY_CLR		1 << 9
#define	GPT_SPI_RX_FIFO_OVERFLOW_CLR		1 << 10
#define	GPT_SPI_RX_FIFO_UNDERFLOW_CLR		1 << 11
#define	GPT_SPI_TRANSFER_END_CLR		1 << 12

/* spi irq status register*/
#define	GPT_SPI_TX_FIFO_ALMOST_FULL		1 << 0
#define	GPT_SPI_TX_FIFO_ALMOST_EMPTY		1 << 1
#define	GPT_SPI_TX_FIFO_FULL			1 << 2
#define	GPT_SPI_TX_FIFO_EMPTY			1 << 3
#define	GPT_SPI_TX_FIFO_OVERFLOW		1 << 4
#define	GPT_SPI_TX_FIFO_UNDERFLOW		1 << 5
#define	GPT_SPI_RX_FIFO_ALMOST_FULL		1 << 6
#define	GPT_SPI_RX_FIFO_ALMOST_EMPTY		1 << 7
#define	GPT_SPI_RX_FIFO_FULL			1 << 8
#define	GPT_SPI_RX_FIFO_EMPTY			1 << 9
#define	GPT_SPI_RX_FIFO_OVERFLOW		1 << 10
#define	GPT_SPI_RX_FIFO_UNDERFLOW		1 << 11
#define	GPT_SPI_TRANSFER_END			1 << 12

/* spi irq mask register */
#define	GPT_SPI_TX_FIFO_ALMOST_FULL_MASK	1 << 0
#define	GPT_SPI_TX_FIFO_ALMOST_EMPTY_MASK	1 << 1
#define	GPT_SPI_TX_FIFO_FULL_MASK		1 << 2
#define	GPT_SPI_TX_FIFO_EMPTY_MASK		1 << 3
#define	GPT_SPI_TX_FIFO_OVERFLOW_MASK		1 << 4
#define	GPT_SPI_TX_FIFO_UNDERFLOW_MASK		1 << 5
#define	GPT_SPI_RX_FIFO_ALMOST_FULL_MASK	1 << 6
#define	GPT_SPI_RX_FIFO_ALMOST_EMPTY_MASK	1 << 7
#define	GPT_SPI_RX_FIFO_FULL_MASK		1 << 8
#define	GPT_SPI_RX_FIFO_EMPTY_MASK		1 << 9
#define	GPT_SPI_RX_FIFO_OVERFLOW_MASK		1 << 10
#define	GPT_SPI_RX_FIFO_UNDERFLOW_MASK		1 << 11
#define	GPT_SPI_TRANSFER_END_MASK		1 << 12
#define	GPT_SPI_MASK_WIDTH			13

/* spi irq clear register */
#define	GPT_SPI_TX_FIFO_ALMOST_FULL_CLR		1 << 0
#define	GPT_SPI_TX_FIFO_ALMOST_EMPTY_CLR	1 << 1
#define	GPT_SPI_TX_FIFO_FULL_CLR		1 << 2
#define	GPT_SPI_TX_FIFO_EMPTY_CLR		1 << 3
#define	GPT_SPI_TX_FIFO_OVERFLOW_CLR		1 << 4
#define	GPT_SPI_TX_FIFO_UNDERFLOW_CLR		1 << 5
#define	GPT_SPI_RX_FIFO_ALMOST_FULL_CLR		1 << 6
#define	GPT_SPI_RX_FIFO_ALMOST_EMPTY_CLR	1 << 7
#define	GPT_SPI_RX_FIFO_FULL_CLR		1 << 8
#define	GPT_SPI_RX_FIFO_EMPTY_CLR		1 << 9
#define	GPT_SPI_RX_FIFO_OVERFLOW_CLR		1 << 10
#define	GPT_SPI_RX_FIFO_UNDERFLOW_CLR		1 << 11
#define	GPT_SPI_TRANSFER_END_CLR		1 << 12

#define	GPT_SPI_TX_ENABLE			1 << 0
#define	GPT_SPI_TX_DISABLE			0 << 0
#define	GPT_SPI_RX_ENABLE			1 << 1
#define	GPT_SPI_RX_DISABLE			0 << 0

/* spi ssn manual register */
#define	GPT_SPI_SSN_MANUAL_DISABLE		1 << 0

#define GPT_SPI_FIFO_DEPTH 64

#define GPT_SPI_TIMEOUT_MS 30000

struct gpt_spi {
	struct platform_device		*dev;
	void __iomem		*regs;	/* virt. address of the control registers */
	struct completion	 done;
	struct mutex		 lock;
	int			 irq;
	struct clk		*clk;
	u8			*rx_buf;		/* pointer in the Tx buffer */
	const u8		*tx_buf;	/* pointer in the Rx buffer */

	u32			 cur_speed;
	u8			 cur_bpw;
	u16			 cur_mod;
	u8			 cur_lsb;

	unsigned int (*read_fn)(void __iomem *);
	void (*write_fn)(u32, void __iomem *);
	int			 len;

};

static void gspi_write32(u32 val, void __iomem *addr)
{
	iowrite32(val, addr);
}

static unsigned int gspi_read32(void __iomem *addr)
{
	return ioread32(addr);
}

static void dump_all_register(struct gpt_spi* gs)
{

	int value  = 0;
	SP_PRINTK("----------------------------------------------------\n");
	value = gspi_read32(gs->regs + GPT_SPI_CFG);
	SP_PRINTK("cfg= 0x%x\n",value);
	value = gspi_read32(gs->regs + GPT_SPI_DIV_CNT);
	SP_PRINTK("clk_div = 0x%x\n",value);
	value = gspi_read32(gs->regs + GPT_SPI_DLY_CNT);
	SP_PRINTK("dly_cnt = 0x%x\n",value);
	value = gspi_read32(gs->regs + GPT_SPI_SSN_MANUAL);
	SP_PRINTK("ssn_manual = 0x%x\n",value);
	value = gspi_read32(gs->regs + GPT_SPI_BIT_MODE);
	SP_PRINTK("bit_mode = 0x%x\n",value);
	value = gspi_read32(gs->regs + GPT_SPI_FIFO_CFG);
	SP_PRINTK("fifo_cfg = 0x%x\n",value);
	value = gspi_read32(gs->regs + GPT_SPI_FIFO_CLEAR);
	SP_PRINTK("fifo_clear = 0x%x\n",value);
	value = gspi_read32(gs->regs + GPT_SPI_TRANSFER_CNT);
	SP_PRINTK("transfer_cnt = 0x%x\n",value);
	value = gspi_read32(gs->regs + GPT_SPI_IRQ_RAW_STATUS);
	SP_PRINTK("IRQ_RAW_STATUS = 0x%x\n",value);
	value = gspi_read32(gs->regs + GPT_SPI_IRQ_STATUS);
	SP_PRINTK("irq_status = 0x%x\n",value);
	value = gspi_read32(gs->regs + GPT_SPI_IRQ_CLEAR);
	SP_PRINTK("IRQ_CLEAR = 0x%x\n",value);
	value = gspi_read32(gs->regs + GPT_SPI_IRQ_MASK);
	SP_PRINTK("IRQ_MASK = 0x%x\n",value);
	value = gspi_read32(gs->regs + GPT_SPI_FIFO_OP);
	SP_PRINTK("fifo_OP = 0x%x\n",value);
	value = gspi_read32(gs->regs + GPT_SPI_FIFO_CUR_CNT);
	SP_PRINTK("fifo_cur_cnt = 0x%x\n",value);
	SP_PRINTK("----------------------------------------------------\n");

}

static void gspi_init_hwinit(struct gpt_spi *gspi)
{
	void __iomem *regs_base = gspi->regs;

	/*enable spi*/
	apiu_unit_init(gspi->regs,0);

	gspi_write32(GPT_SPI_MODE_MASTER | GPT_SPI_MSB | GPT_SPI_SSN_SEL,regs_base + GPT_SPI_CFG);
	gspi_write32(5,regs_base + GPT_SPI_DIV_CNT);
	gspi_write32(GPT_SPI_BIT_8,regs_base + GPT_SPI_BIT_MODE);
	gspi_write32(0x1000,regs_base + GPT_SPI_IRQ_MASK);
	gspi_write32(0x1fff,regs_base + GPT_SPI_IRQ_CLEAR);

}

/* This driver supports single master mode only. Hence Tx FIFO Empty
 * is the only interrupt we care about.
 * Receive FIFO Overrun, Transmit FIFO Underrun, Mode Fault, and Slave Mode
 * Fault are not to happen.
 */
static irqreturn_t gpt_spi_irq(int irq, void *dev_id)
{
	struct gpt_spi *gspi = dev_id;
	u32 value = 0;

	gspi_write32(0, gspi->regs + GPT_SPI_FIFO_OP);


	value = gspi_read32(gspi->regs + GPT_SPI_IRQ_RAW_STATUS);


	if(GPT_SPI_TX_FIFO_OVERFLOW_CLR & value)
	{
		dev_err(&gspi->dev->dev,"err:tx fifo overflow\n");
	}

	value = gspi_read32(gspi->regs + GPT_SPI_IRQ_RAW_STATUS);
	if(GPT_SPI_TX_FIFO_UNDERFLOW_CLR & value)
	{
		dev_err(&gspi->dev->dev,"err:tx fifo underflow\n");
	}

	value = gspi_read32(gspi->regs + GPT_SPI_IRQ_RAW_STATUS);
	if(GPT_SPI_RX_FIFO_OVERFLOW_CLR & value)
	{
		dev_err(&gspi->dev->dev,"err:rx fifo overflow\n");
	}

	value = gspi_read32(gspi->regs + GPT_SPI_IRQ_RAW_STATUS);
	if(GPT_SPI_RX_FIFO_UNDERFLOW_CLR & value)
	{
		dev_err(&gspi->dev->dev,"err:rx fifo underflow\n");
	}

	value = gspi_read32(gspi->regs + GPT_SPI_IRQ_RAW_STATUS);
	if(GPT_SPI_TRANSFER_END_CLR & value)
	{
		complete(&gspi->done);
		gspi_write32(0x1fff,gspi->regs + GPT_SPI_IRQ_CLEAR);
		return IRQ_HANDLED;
	}
	return IRQ_NONE;
}


static void gpt_spi_cleanup(struct spi_device *spi)
{

}


static int gptspi_transfer_one(struct spi_master *master,
		struct spi_device *spi,
		struct spi_transfer *tfr)
{
	struct gpt_spi *gs = spi_master_get_devdata(master);
	unsigned char value  = 0;
	unsigned int cfg = 0;
	unsigned int count = 0;
	unsigned int inter_cnt= 0;
	unsigned int cnt = 0;
	unsigned char*buf = NULL;
	unsigned int  tt =0;
	unsigned int tx_count=0;

	gs->rx_buf = tfr->rx_buf;
	gs->tx_buf = tfr->tx_buf;
	gs->len = tfr->len;

	buf = gs->rx_buf;

	mutex_lock(&gs->lock);

	if(gs->tx_buf)
	{
		count = (gs->len)/GPT_SPI_FIFO_DEPTH;
		cnt = (gs->len)%GPT_SPI_FIFO_DEPTH;

		if(0 != count)
		{
			gspi_write32(GPT_SPI_FIFO_DEPTH,gs->regs + GPT_SPI_TRANSFER_CNT);
			cfg = gspi_read32(gs->regs + GPT_SPI_CFG);
			cfg |= GPT_SPI_TRANSFER_EN;
			gspi_write32(cfg,gs->regs + GPT_SPI_CFG);

			while(count--)
			{

				inter_cnt = GPT_SPI_FIFO_DEPTH;
				gspi_write32(0, gs->regs + GPT_SPI_FIFO_OP);
				while(inter_cnt--)
				{
					value = *(gs->tx_buf++);
					//SP_PRINTK("1tx value = %x\n",value);
					gspi_write32(value,gs->regs + GPT_SPI_TX_DATA);
				}

				reinit_completion(&gs->done);
				gspi_write32(0x3, gs->regs + GPT_SPI_FIFO_OP);
				wait_for_completion_interruptible(&gs->done);

				// i = gspi_read32(gs->regs +  GPT_SPI_FIFO_CUR_CNT);
				// i = i>>8;
				inter_cnt = GPT_SPI_FIFO_DEPTH;
				while(inter_cnt--)
				{
					tt = gspi_read32(gs->regs + GPT_SPI_RX_DATA);
					*buf = (char)tt;
					//SP_PRINTK("rx value = %x\n",*buf);
					buf++;
				}

			}
		}

		if(cnt){
			gspi_write32(cnt,gs->regs + GPT_SPI_TRANSFER_CNT);
			cfg = gspi_read32(gs->regs + GPT_SPI_CFG);
			cfg |= GPT_SPI_TRANSFER_EN;
			gspi_write32(cfg,gs->regs + GPT_SPI_CFG);

			gspi_write32(0, gs->regs + GPT_SPI_FIFO_OP);

			for(tx_count=0;tx_count<cnt;tx_count++)
			{
				value = *(gs->tx_buf++);
				//SP_PRINTK("2tx value = %x\n",value);
				gspi_write32(value,gs->regs + GPT_SPI_TX_DATA);
				// printk("cnt is %x\n",cnt);
			}

			reinit_completion(&gs->done);
			gspi_write32(0x3, gs->regs + GPT_SPI_FIFO_OP);
			wait_for_completion_interruptible(&gs->done);

			//i = gspi_read32(gs->regs +  GPT_SPI_FIFO_CUR_CNT);
			//i = i >> 8;
			//if(cnt != i)
			//      return -EINVAL;
			while(cnt--)
			{
				tt = gspi_read32(gs->regs + GPT_SPI_RX_DATA);
				*buf = (char)tt;
				//SP_PRINTK("2rx value = %x\n",*buf);
				buf++;
			}
		}
	}

	//clr fifo
	gspi_write32(GPT_SPI_RX_FLUSH | GPT_SPI_TX_FLUSH,gs->regs + GPT_SPI_FIFO_CLEAR);

	mutex_unlock(&gs->lock);
	return 0;
}


static int gpt_spi_config(struct gpt_spi*gs)
{
	void __iomem *regs_base = gs->regs;
	unsigned int value  = 0;

	value = gspi_read32(regs_base + GPT_SPI_CFG);
	if(gs->cur_mod & SPI_CPOL)
		value |= GPT_SPI_CPOL;
	if(gs->cur_mod & SPI_CPHA)
		value |= GPT_SPI_CPHA;
	if(gs->cur_mod & SPI_LSB_FIRST)
		value |= GPT_SPI_LSB;
	gspi_write32(value,regs_base + GPT_SPI_CFG);

	value = gspi_read32(regs_base + GPT_SPI_BIT_MODE);
	switch(gs->cur_bpw){
		case 32:
			value = 0;
			break;
		case 16:
			value = GPT_SPI_BIT_16;
			break;
		default:
			value = GPT_SPI_BIT_8;
			break;
	}

	gspi_write32(value,regs_base + GPT_SPI_BIT_MODE);

	value  = (CONFIG_SYS_MHZ * MHz) / (gs->cur_speed);
	gspi_write32(value,regs_base + GPT_SPI_DIV_CNT);

	return 0;
}

static int gpt_spi_prepare_message(struct spi_master *sm,struct spi_message *smg)
{
	struct gpt_spi*gs =  spi_master_get_devdata(sm);
	struct spi_device *spi = smg->spi;
	unsigned int val = 0;

	//clr fifo
	gspi_write32(0x3,gs->regs + GPT_SPI_FIFO_CLEAR);

	//disable rx/tx channel
	gspi_write32(0,gs->regs + GPT_SPI_FIFO_OP);

	//clr irq
	gspi_write32(GPT_SPI_TRANSFER_END_CLR,gs->regs + GPT_SPI_IRQ_CLEAR);

	gs->cur_bpw = spi->bits_per_word;
	gs->cur_mod = spi->mode;
	gs->cur_speed = spi->max_speed_hz;

	val = gspi_read32(gs->regs);
	val |= GPT_SPI_MODE_MASTER | GPT_SPI_SLV_XFER_SAME_EDGE | GPT_SPI_SSN_SEL;
	gspi_write32(val, gs->regs);

	//do not mask tranferend INT
	gspi_write32(0x1000,gs->regs + GPT_SPI_IRQ_MASK);
	gspi_write32(0xa060408,gs->regs + GPT_SPI_FIFO_CFG);
	gspi_write32(0,gs->regs + GPT_SPI_DLY_CNT);
	gspi_write32(0,gs->regs + GPT_SPI_TRANSFER_CNT);

	gpt_spi_config(gs);

	return 0;
}

int gpt_spi_setup(struct spi_device*spi)
{
	struct gpt_spi*gs = spi_master_get_devdata(spi->master);

	gs->cur_bpw = spi->bits_per_word;
	gs->cur_mod = spi->mode;
	gs->cur_speed = spi->max_speed_hz;
	gpt_spi_config(gs);

	return 0;
}
EXPORT_SYMBOL(gpt_spi_setup);

void gpt_spi_set_cs(struct spi_device *spi,bool enable)
{
	struct gpt_spi *gs  = spi_master_get_devdata(spi->master);
	u32  value ;

	if(!enable)
	{
		value = gspi_read32(gs->regs + GPT_SPI_SSN_MANUAL );
		value &= ~GPT_SPI_SSN_MANUAL_DISABLE;

	}else{
		value = gspi_read32(gs->regs + GPT_SPI_SSN_MANUAL );
		value |= GPT_SPI_SSN_MANUAL_DISABLE;
	}

	gspi_write32(value, gs->regs + GPT_SPI_SSN_MANUAL);

}
EXPORT_SYMBOL(gpt_spi_set_cs);

static const struct of_device_id gpt_spi_of_match[] = {
	{ .compatible = "gpt,spi0.0", },
	{ .compatible = "gpt,spi0.1", },
	{}
};
MODULE_DEVICE_TABLE(of, gpt_spi_of_match);

static int gpt_spi_parse_dt(struct platform_device *pdev)
{
	int ret;
	struct pinctrl *pinctrl;
	const char *pctrl_state;
	struct pinctrl_state *states;
	struct device_node *np = pdev->dev.of_node;

	pinctrl = devm_pinctrl_get(&pdev->dev);
	pctrl_state = devm_kzalloc(&pdev->dev, sizeof(pctrl_state), GFP_KERNEL);
	if (!pctrl_state) {
		dev_err(&pdev->dev, "Cannot allocate pctrl_state\n");
		return -ENOMEM;
	}

	states = devm_kzalloc(&pdev->dev, sizeof(states), GFP_KERNEL);
	if (!states) {
		dev_err(&pdev->dev, "Cannot allocate states\n");
		return -ENOMEM;
	}

	ret = of_property_read_string_index(np, "pinctrl-names", 0,
							&pctrl_state);
	if (ret < 0) {
		dev_err(&pdev->dev, "Cannot parse pinctrl-names %d\n", ret);
		return ret;
	}

	states = pinctrl_lookup_state(pinctrl, pctrl_state);
	if (IS_ERR(states)) {
		dev_err(&pdev->dev, "Lookup state failed\n");
		return IS_ERR(states);
	}

	ret = pinctrl_select_state(pinctrl, states);
	if (ret < 0) {
		dev_err(&pdev->dev, "Select state failed\n");
		return ret;
	}

	return 0;
}

static int gpt_ext_apb_ccr(void)
{
	uint32_t *extsrc = ioremap(0xf0000010,0x40);
	if(extsrc == NULL){
		printk("%s:%d-->get memory/io resource filed\n", __func__, __LINE__);
		return -ENXIO;
	}
	/*config clk*/
	writel((readl(extsrc) & 0xFFFFFF89) | 0x81, extsrc);

	iounmap(extsrc);
	
	return 0;
}

static int gpt_spi_probe(struct platform_device *pdev)
{
	struct gpt_spi *gspi;
	struct gspi_platform_data *pdata;
	struct resource *res;
	int ret;
	struct spi_master *master;
	int num_cs = 0;
	int bits_per_word = 8;

	//SP_PRINTK("func: gpt_spi_probe function!!\n");

	gpt_spi_parse_dt(pdev);

	//gpt_ext_apb_ccr();

	pdata = dev_get_platdata(&pdev->dev);

	if(!pdata && pdev->dev.of_node)
	{
		if(of_property_read_u32(pdev->dev.of_node,"gpt,num-ss-bits",&num_cs))
		{
			dev_warn(&pdev->dev,"number of chip select lines not specified\n");
		}
		if(of_property_read_u32(pdev->dev.of_node,"gpt,num-transfer-bits",&bits_per_word))
		{
			dev_warn(&pdev->dev,"num-transfer-bits not specified\n");
		}

	}

	master = spi_alloc_master(&pdev->dev, sizeof(struct gpt_spi));
	if (!master)
	{
		return -ENODEV;
	}

	master->bus_num = pdev->id;
	// master->bus_num = 0;
	master->num_chipselect = num_cs?num_cs :2;
	master->dev.of_node = pdev->dev.of_node;
	master->mode_bits = SPI_CPOL | SPI_CPHA;

	gspi = spi_master_get_devdata(master);

	init_completion(&gspi->done);
	mutex_init(&gspi->lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	gspi->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(gspi->regs)) {
		ret = PTR_ERR(gspi->regs);
		goto put_master;
	}

	apiu_unit_init(&gspi->regs,0);
	/*
	 * Detect endianess on the IP via loop bit in CR. Detection
	 * must be done before reset is sent because incorrect reset
	 * value generates error interrupt.
	 * Setup little endian helper functions first and try to use them
	 * and check if bit was correctly setup or not.
	 */
	gspi->read_fn = gspi_read32;
	gspi->write_fn = gspi_write32;

	gspi->dev =pdev;
	gspi->cur_bpw= bits_per_word;

	/* SPI controller initializations */
	gspi_init_hwinit(gspi);

	/*irq number*/
	gspi->irq = platform_get_irq(pdev, 0);
	if (gspi->irq < 0) {
		dev_err(&pdev->dev,"no spi IRQ specified!!");
		ret = gspi->irq;
		goto put_master;
	}

	// Register for SPI Interrupt
	ret = devm_request_irq(&pdev->dev, gspi->irq, gpt_spi_irq, 0,
			dev_name(&pdev->dev), gspi);
	if (ret)
		goto put_master;

	master->bits_per_word_mask = SPI_BPW_RANGE_MASK(8, 32);
	master->transfer_one = gptspi_transfer_one;
	master->prepare_message = gpt_spi_prepare_message;
	master->setup = gpt_spi_setup;
	master->set_cs = gpt_spi_set_cs;
	master->cleanup = gpt_spi_cleanup;

	ret = devm_spi_register_master(&pdev->dev,master);
	if(ret){
		dev_err(&pdev->dev,"could not register SPI master: %d\n",ret);
		goto put_master;
	}


	dev_info(&pdev->dev, "at 0x%08llX mapped to 0x%p, irq=%d\n",
			(unsigned long long)res->start, gspi->regs, gspi->irq);

	platform_set_drvdata(pdev, master);
	return 0;

put_master:
	spi_master_put(master);

	return ret;
}

static int gpt_spi_remove(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct gpt_spi *gspi = spi_master_get_devdata(master);

	iounmap((void __iomem*)gspi->regs);

	return 0;
}

/* work with hotplug and coldplug */
MODULE_ALIAS("platform:" GPT_SPI_NAME);

static struct platform_driver gpt_spi_driver = {
	.probe = gpt_spi_probe,
	.remove = gpt_spi_remove,
	.driver = {
		.name = GPT_SPI_NAME,
		.of_match_table = gpt_spi_of_match,
	},
};
module_platform_driver(gpt_spi_driver);

MODULE_AUTHOR("GPT Software");
MODULE_DESCRIPTION("GPT SPI driver");
MODULE_LICENSE("GPL");
