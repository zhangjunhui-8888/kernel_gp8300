/*
 * GPT SPI controller driver (slave mode only)
 * support SCLK(20K - 8M), BIT(8/16/32)
 *
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
#include <asm/mach-chip2/apiu.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/device.h> 
#include <linux/fs.h> 
#include <linux/poll.h>
#include <linux/kthread.h>
#include "spi-gpt-slave.h"
#include <linux/list.h>

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

#define DRV_NAME "spi-gpt-slave"

#define TIME_DELAY 1000

#define SPI_MODE_MASK		(SPI_CPHA | SPI_CPOL)

int spi_gpt_major;
static struct class *spi_class;

struct gpt_spi_slave {
	struct device		*dev;
	void __iomem		*regs;
	struct mutex		 lock;

	int			 irq;
	struct clk	*clk;

	int bus_num;

	u8	tx_buf_num;
	u16 tx_buf_size;

	u8 flag_completion;

	u8		 cur_bpw;
	u16		 cur_mode;
	u16      r_len;
	u16      s_len;

	u8 *tx_buf8;
	u16 *tx_buf16;
	u32 *tx_buf32;

	unsigned users;

	unsigned int (*read_fn)(void __iomem *);
	void		 (*write_fn)(u32, void __iomem *);

	wait_queue_head_t   w_wait;
	wait_queue_head_t   r_wait;

	int poll_r_flag; //user read data from kernel
	int poll_w_flag; //user write data to kernel

	void *rx_buf;
	struct tx_buf *tx_buf[10];
};

static LIST_HEAD(queued_list);

struct tx_buf {
	u8 id;
	u8 flag;
	u16 len;
	u8 *data; 
	struct list_head list;
};

struct gpt_spi_slave *gspi;

static void gspi_write32(u32 val, void __iomem *addr)
{
	iowrite32(val, addr);
}

static unsigned int gspi_read32(void __iomem *addr)
{
	return ioread32(addr);
}

static int spi_slave_setup(void)
{
	unsigned int value  = 0;

	value = gspi_read32(gspi->regs + GPT_SPI_CFG);
	value &= 0xfffffff9; 
	if(gspi->cur_mode & SPI_CPOL)
		value |= GPT_SPI_CPOL;
	if(gspi->cur_mode & SPI_CPHA)
		value |= GPT_SPI_CPHA;
	gspi_write32(value, gspi->regs + GPT_SPI_CFG);

	value = gspi_read32(gspi->regs + GPT_SPI_BIT_MODE);
	switch(gspi->cur_bpw){
		case 32:
			value = GPT_SPI_BIT_32;
			break;
		case 16:
			value = GPT_SPI_BIT_16;
			break;
		default:
			value = GPT_SPI_BIT_8;
			break;
	}
	gspi_write32(value, gspi->regs + GPT_SPI_BIT_MODE);

	return 0;
}

static int spi_slave_transfer(void)
{
	u16 len = 0;
	u16 count = 0;
	u8  cnt = 0;
	u16 inter_cnt = 0;
	u8 id = 0;
	struct tx_buf *tx;

	if(!list_empty(&queued_list)) {
		list_for_each_entry(tx, &queued_list, list)	{
			id = tx->id;
			break;
		}

		if(gspi->flag_completion == 0) {
			if(gspi->cur_bpw == 8) {
				gspi->tx_buf8 = (u8 *)gspi->tx_buf[id]->data;
			} else if (gspi->cur_bpw == 16) {
				gspi->tx_buf16 = (u16 *)gspi->tx_buf[id]->data;
			} else if (gspi->cur_bpw == 32) {
				gspi->tx_buf32 = (u32 *)gspi->tx_buf[id]->data;
			}
		}

		gspi->s_len = gspi->tx_buf[id]->len;

		if(gspi->cur_bpw == 8) {
			len = gspi->s_len;
		} else if (gspi->cur_bpw == 16) {
			len = gspi->s_len / 2;
		} else if (gspi->cur_bpw == 32) {
			len = gspi->s_len / 4;
		}

		count = len / GPT_SPI_FIFO_DEPTH;
		cnt = len % GPT_SPI_FIFO_DEPTH;
		if(count) {
			inter_cnt = GPT_SPI_FIFO_DEPTH;
			gspi_write32(gspi_read32(gspi->regs + GPT_SPI_CFG) | GPT_SPI_TRANSFER_EN, gspi->regs + GPT_SPI_CFG);

			if(gspi->cur_bpw == 8) {
				while(inter_cnt --)
					gspi_write32(*gspi->tx_buf8 ++, gspi->regs + GPT_SPI_TX_DATA);
				gspi->tx_buf[id]->len = gspi->tx_buf[id]->len - GPT_SPI_FIFO_DEPTH;

			} else if (gspi->cur_bpw == 16) {
				while(inter_cnt --)
					gspi_write32(*gspi->tx_buf16 ++, gspi->regs + GPT_SPI_TX_DATA);
				gspi->tx_buf[id]->len = gspi->tx_buf[id]->len - GPT_SPI_FIFO_DEPTH * 2;

			} else if (gspi->cur_bpw == 32) {
				while (inter_cnt --)
					gspi_write32(*gspi->tx_buf32 ++, gspi->regs + GPT_SPI_TX_DATA);
				gspi->tx_buf[id]->len = gspi->tx_buf[id]->len - GPT_SPI_FIFO_DEPTH * 4;
			}

			//enable TX/RX QUEUE
			gspi_write32(0x3, gspi->regs + GPT_SPI_FIFO_OP);

		} else if(cnt) {
			inter_cnt = cnt;

			gspi_write32(gspi_read32(gspi->regs + GPT_SPI_CFG) | GPT_SPI_TRANSFER_EN, gspi->regs + GPT_SPI_CFG);

			if(gspi->cur_bpw == 8) {
				while(inter_cnt --)
					gspi_write32(*gspi->tx_buf8 ++, gspi->regs + GPT_SPI_TX_DATA);
				gspi->tx_buf[id]->len = gspi->tx_buf[id]->len - cnt;

			} else if (gspi->cur_bpw == 16) {
				while(inter_cnt --)
					gspi_write32(*gspi->tx_buf16 ++, gspi->regs + GPT_SPI_TX_DATA);
				gspi->tx_buf[id]->len = gspi->tx_buf[id]->len - cnt * 2;

			} else if (gspi->cur_bpw == 32) {
				while(inter_cnt --)
					gspi_write32(*gspi->tx_buf32 ++, gspi->regs + GPT_SPI_TX_DATA);
				gspi->tx_buf[id]->len = gspi->tx_buf[id]->len - cnt * 4;
			}
			//enable TX/RX QUEUE
			gspi_write32(0x3, gspi->regs + GPT_SPI_FIFO_OP);

		}//cnt end

		if(gspi->tx_buf[id]->len == 0) {
			list_del(&gspi->tx_buf[id]->list);
			gspi->tx_buf[id]->flag = 0;
			gspi->flag_completion = 0;
		} else {
			gspi->flag_completion = 1;
		}
	} else {
		gspi_write32(0x2, gspi->regs + GPT_SPI_FIFO_OP);
	}

	return 0;
}

static int gspi_init_hwinit(void)
{
	apiu_unit_init(gspi->regs,0);

	//salve mode; CPOL=1,CPHA=0; MSB first; 
	//gspi_write32(GPT_SPI_MSB & 0xffffffdf, gspi->regs + GPT_SPI_CFG);
	gspi_write32(GPT_SPI_MSB | GPT_SPI_SLV_XFER_SAME_EDGE, gspi->regs + GPT_SPI_CFG);

	//8bit
	gspi_write32(GPT_SPI_BIT_8, gspi->regs + GPT_SPI_BIT_MODE);

	gspi_write32(0, gspi->regs + GPT_SPI_DIV_CNT);

	//CLR FIFO	
	gspi_write32(GPT_SPI_RX_FLUSH | GPT_SPI_TX_FLUSH, gspi->regs + GPT_SPI_FIFO_CLEAR);

	gspi_write32(0x0, gspi->regs + GPT_SPI_FIFO_OP);

	//tx fifo: alm_ampty TH=0; rx fifo: alm_full TH=1
	gspi_write32(0x0, gspi->regs + GPT_SPI_FIFO_CFG);

	//CLR INT
	gspi_write32(0x1fff, gspi->regs + GPT_SPI_IRQ_CLEAR);

	return 0;
}

static long get_timeus(void)
{
	struct timeval tv;

	do_gettimeofday(&tv);
	return tv.tv_sec * 1000000 + tv.tv_usec;
}

/* This driver supports single slave mode only.
 * The first word is the num of bytes in this frame data (except for the first word).
 */
static irqreturn_t gpt_spi_irq(int irq, void *dev_id)
{
	u32 value = 0;
	long t0 = 0, t1 = 0; 
	int num;
	u16 len = 0;
	int i = 0;
	u8  *buf8  = NULL;
	u16 *buf16 = NULL;
	u32 *buf32 = NULL;

	struct gpt_spi_slave *gspi = dev_id;

	value = gspi_read32(gspi->regs + GPT_SPI_IRQ_STATUS);

	if(GPT_SPI_RX_FIFO_ALMOST_FULL & value)	{
		if (gspi->cur_bpw == 8)	{
			buf8 = gspi->rx_buf;

			while (1) {
				i = gspi_read32(gspi->regs + GPT_SPI_FIFO_CUR_CNT);
				num = i >> 8;
				if (num > 0) {
					while (num-- > 0) {
						*buf8 ++ = gspi_read32(gspi->regs + GPT_SPI_RX_DATA);
						len ++;
					}
					t0 = get_timeus();
				} else {
					t1 = get_timeus();
					if (t1 - t0 > TIME_DELAY / 2)
						break;
				}
			}

		} else if (gspi->cur_bpw == 16) {
			buf16 = gspi->rx_buf;

			while (1) {
				i = gspi_read32(gspi->regs + GPT_SPI_FIFO_CUR_CNT);
				num = i >> 8;
				if (num > 0) {
					while (num-- > 0) {
						*buf16++ = gspi_read32(gspi->regs + GPT_SPI_RX_DATA);
						len = len + 2;
					}
					t0 = get_timeus();
				} else {
					t1 = get_timeus();
					if (t1 - t0 > TIME_DELAY)
						break;
				}
			}

		} else if (gspi->cur_bpw == 32) {
			buf32 = gspi->rx_buf;

			while (1) {
				i = gspi_read32(gspi->regs + GPT_SPI_FIFO_CUR_CNT);
				num = i >> 8;
				if (num > 0) {
					while (num-- > 0) {
						*buf32++ = gspi_read32(gspi->regs + GPT_SPI_RX_DATA);
						len = len + 4;
					}
					t0 = get_timeus();
				} else {
					t1 = get_timeus();
					if ( t1 - t0 > TIME_DELAY * 2)
						break;
				}
			}
		}

		gspi_write32(0x0, gspi->regs + GPT_SPI_FIFO_OP);		

		gspi_write32(GPT_SPI_RX_FLUSH | GPT_SPI_TX_FLUSH, gspi->regs + GPT_SPI_FIFO_CLEAR);

		gspi->r_len = len;

		gspi->poll_r_flag = 1;
		wake_up(&gspi->r_wait);

		//spi_slave_transfer();

		gspi_write32(0x1fff, gspi->regs + GPT_SPI_IRQ_CLEAR);

		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static int spi_request_buf(void)
{
	int i = 0;
	int status = 0;

	for(i = 0; i < gspi->tx_buf_num; i++) {
		gspi->tx_buf[i] = kzalloc(sizeof(struct tx_buf), GFP_KERNEL);
		if (gspi->tx_buf[i] == NULL) {
			status = -ENOMEM;
			return status;
		}

		gspi->tx_buf[i]->data = kzalloc(gspi->tx_buf_size, GFP_KERNEL);
		if (gspi->tx_buf[i]->data == NULL) {
			status = -ENOMEM;
			return status;
		}

		gspi->tx_buf[i]->id = i;
		gspi->tx_buf[i]->len = 0;
		gspi->tx_buf[i]->flag = 0;
	}

	gspi->rx_buf = kzalloc(280, GFP_KERNEL);
	if (gspi->rx_buf == NULL) {
		status = -ENOMEM;
		return status;
	}

	return status;
}

static int spi_release_buf(void)
{
	int i = 0;

	for(i = 0; i < gspi->tx_buf_num; i++) {
		kfree(gspi->tx_buf[i]);
		kfree(gspi->tx_buf[i]->data);
	}

	kfree(gspi->rx_buf);
	return 0;
}

static long spi_gpt_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int retval = 0;
	struct gpt_spi_slave *gspi;
	u32 tmp;
	int err = 0;

	/* Check type and command number */
	if (_IOC_TYPE(cmd) != SPI_IOC_MAGIC)
		return -ENOTTY;

	/* Check access direction once here; don't repeat below.
	 * IOC_DIR is from the user perspective, while access_ok is
	 * from the kernel perspective; so they look reversed.
	 */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,
				(void __user *)arg, _IOC_SIZE(cmd));
	if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ,
				(void __user *)arg, _IOC_SIZE(cmd));
	if (err)
		return -EFAULT;

	gspi = filp->private_data;

	mutex_lock(&gspi->lock);

	switch (cmd) {
		/* read requests */
		case SPI_IOC_RD_MODE32:
			retval = __put_user(gspi->cur_mode & SPI_MODE_MASK, (__u32 __user *)arg);
			break;
		case SPI_IOC_RD_BITS_PER_WORD:
			retval = __put_user(gspi->cur_bpw, (__u8 __user *)arg);
			break;
		case SPI_IOC_RD_DATA_LEN:
			retval = __put_user(gspi->r_len, (__u16 __user *)arg);
			break;

			/* write requests */
		case SPI_IOC_WR_MODE32:
			retval = __get_user(tmp, (u32 __user *)arg);
			if (retval == 0) {
				if (tmp & ~SPI_MODE_MASK) {
					retval = -EINVAL;
					break;
				}

				tmp |= gspi->cur_mode & ~SPI_MODE_MASK;
				gspi->cur_mode = (u16)tmp;
				spi_slave_setup();
			}
			break;
		case SPI_IOC_WR_BITS_PER_WORD:
			retval = __get_user(tmp, (__u8 __user *)arg);
			if (retval == 0) {
				gspi->cur_bpw = tmp;
				spi_slave_setup();
			}
			break;
		case SPI_IOC_WR_TX_BUF_NUM:
			retval = __get_user(tmp, (__u8 __user *)arg);
			if (retval == 0) {
				gspi->tx_buf_num = tmp;
				if(tmp > 10) {
					retval = -EINVAL;
					break;
				}
			}
			break;
		case SPI_IOC_WR_TX_BUF_SIZE:
			retval = __get_user(tmp, (__u16 __user *)arg);
			if (retval == 0) {
				gspi->tx_buf_size = tmp;
				if(tmp > 65536)	{
					retval = -EINVAL;
					break;
				}
			}
			break;
		case SPI_IOC_WR_REQUEST_BUF:
			spi_request_buf();
			break;
		case SPI_IOC_WR_RELEASE_BUF:
			spi_release_buf();
			break;
		case SPI_IOC_WR_TRANSFER_EN:
			spi_slave_transfer();
			break;
		default:
			break;
	}
	mutex_unlock(&gspi->lock);

	return retval;
}

static int spi_gpt_open(struct inode *inode, struct file *filp)
{
	int status = 0;

	filp->private_data = gspi;

	mutex_lock(&gspi->lock);

	gspi->users++;
	nonseekable_open(inode, filp);

	INIT_LIST_HEAD(&queued_list);

	//CLR FIFO	
	gspi_write32(GPT_SPI_RX_FLUSH | GPT_SPI_TX_FLUSH, gspi->regs + GPT_SPI_FIFO_CLEAR);

	//CLR INT
	gspi_write32(0x1fff, gspi->regs + GPT_SPI_IRQ_CLEAR);

	//irq: RX almost full  
	gspi_write32(0x40, gspi->regs + GPT_SPI_IRQ_MASK);

	//enable TX/RX QUEUE
	gspi_write32(0x2, gspi->regs + GPT_SPI_FIFO_OP);

	mutex_unlock(&gspi->lock);

	return status;
}

static ssize_t spi_gpt_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	ssize_t	status = 0;
	unsigned long ret = 0;
	struct gpt_spi_slave *gspi;
	u8 i = 0;
	u8 buf_id = 0;

	gspi = filp->private_data;

	mutex_lock(&gspi->lock);
	for(i = 0; i < gspi->tx_buf_num; i++) {
		if(gspi->tx_buf[i]->flag == 0) {
			buf_id = i;
			break;
		}
	}

	if(i == gspi->tx_buf_num) {
		status = -EFAULT;
	} else {
		gspi->tx_buf[buf_id]->len = count;
		ret = copy_from_user(gspi->tx_buf[buf_id]->data, buf, count);
		if (ret != 0)
			goto err;

		list_add_tail(&gspi->tx_buf[buf_id]->list, &queued_list);
		gspi->tx_buf[buf_id]->flag = 1;
	}
	mutex_unlock(&gspi->lock);

	return status;
err:
	mutex_unlock(&gspi->lock);
	return -EFAULT;
}

static ssize_t spi_gpt_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	struct gpt_spi_slave *gspi;
	ssize_t	status = 0;
	unsigned long ret = 0;

	gspi = filp->private_data;

	mutex_lock(&gspi->lock);

	ret = copy_to_user(buf, gspi->rx_buf, gspi->r_len);
	if (ret)
		status = -EFAULT;

	mutex_unlock(&gspi->lock);

	return status;
}

static int spi_gpt_release(struct inode *inode, struct file *filp)
{
	struct gpt_spi_slave *gspi;
	int i = 0;

	gspi = filp->private_data;

	gspi->users--;
	filp->private_data = NULL;

	//disable irq  
	gspi_write32(0x0, gspi->regs + GPT_SPI_IRQ_MASK);

	//disable TX/RX QUEUE
	gspi_write32(0x0, gspi->regs + GPT_SPI_FIFO_OP);

	//CLR FIFO	
	gspi_write32(GPT_SPI_RX_FLUSH | GPT_SPI_TX_FLUSH, gspi->regs + GPT_SPI_FIFO_CLEAR);

	//CLR INT
	gspi_write32(0x1fff, gspi->regs + GPT_SPI_IRQ_CLEAR);

	for(i = 0; i < gspi->tx_buf_num; i++)
		gspi->tx_buf[i]->flag = 0;

	gspi->flag_completion = 0;

	return 0;
}

unsigned int spi_gpt_poll(struct file *filp, poll_table *wait)
{
	struct gpt_spi_slave *dev;
	unsigned int mask = 0;

	dev = filp->private_data;

	poll_wait(filp, &dev->r_wait, wait);
	poll_wait(filp, &dev->w_wait, wait);

	/*
	 *user can write data to kernel
	 */
	if(dev->poll_w_flag == 1) {
		dev->poll_w_flag = 0;
		mask = POLLOUT | POLLWRNORM; 
	}

	/*
	 *user can read data from kernel
	 */
	if(dev->poll_r_flag == 1) {
		dev->poll_r_flag = 0;
		mask = POLLIN | POLLRDNORM; 
	}

	return mask;
}

static const struct file_operations spi_ops = {
	.owner			= THIS_MODULE,
	.open			= spi_gpt_open,
	.write			= spi_gpt_write,
	.read			= spi_gpt_read,
	.unlocked_ioctl = spi_gpt_ioctl,
	.release		= spi_gpt_release,
	.poll			= spi_gpt_poll
};

static const struct of_device_id gpt_spi_of_match[] = {
	{ .compatible = "gpt,spi0.0", },
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


static int gpt_spi_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret;
	struct device_node *node;
	node = pdev->dev.of_node;

	gpt_spi_parse_dt(pdev);

	gspi = devm_kzalloc(&pdev->dev, sizeof(struct gpt_spi_slave), GFP_KERNEL);
	if (!gspi) {
		dev_err(&pdev->dev, "Can't allocate for gpt_spi\n");
		ret = -ENOMEM;
		return ret;
	}

	mutex_init(&gspi->lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	gspi->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(gspi->regs)) {
		ret = PTR_ERR(gspi->regs);
	}

	apiu_unit_init(&gspi->regs, 0);
	gspi->read_fn = gspi_read32;
	gspi->write_fn = gspi_write32;

	init_waitqueue_head(&gspi->r_wait);
	init_waitqueue_head(&gspi->w_wait);

	gspi->poll_r_flag = 0;
	gspi->flag_completion = 0;

	gspi->cur_bpw = 8;
	gspi->cur_mode = SPI_MODE_0;
	gspi->r_len = 1;
	gspi->s_len = 1;
	gspi->tx_buf_num = 1;
	gspi->tx_buf_size = 1028;

	gspi->dev = &pdev->dev;
	platform_set_drvdata(pdev, gspi);

	/* SPI controller initializations */
	gspi_init_hwinit();


	gspi->bus_num = of_alias_get_id(node, "spi");
	device_create(spi_class, &pdev->dev, MKDEV(spi_gpt_major, 0),
			gspi, "spi%d-gpt-slave", gspi->bus_num);

	/*irq number*/
	gspi->irq = platform_get_irq(pdev, 0);
	if (gspi->irq < 0) {
		dev_err(&pdev->dev,"no spi IRQ specified!!");
		ret = gspi->irq;
		return ret;
	}

	/*Register for SPI Interrupt*/
	ret = devm_request_irq(&pdev->dev, gspi->irq, gpt_spi_irq, 0,
			dev_name(&pdev->dev), gspi);
	if (ret)
		return ret;

	return 0;
}

static int gpt_spi_remove(struct platform_device *pdev)
{
	struct gpt_spi_slave *gspi = platform_get_drvdata(pdev);

	iounmap((void __iomem*)gspi->regs);
	device_destroy(spi_class,MKDEV(spi_gpt_major, 0));

	return 0;
}

MODULE_ALIAS("platform:" GPT_SPI_NAME);

static struct platform_driver gpt_spi_driver = {
	.probe = gpt_spi_probe,
	.remove = gpt_spi_remove,
	.driver = {
		.name = DRV_NAME,
		.of_match_table = gpt_spi_of_match,
	},
};
module_platform_driver(gpt_spi_driver);

static int __init spi_init(void)
{
	spi_gpt_major = register_chrdev(0, DRV_NAME, &spi_ops);
	if(spi_gpt_major < 0)
	{
		printk("fail to register one chardev!\n");
		return spi_gpt_major;
	}

	spi_class = class_create(THIS_MODULE, DRV_NAME);
	if (IS_ERR(spi_class)) {
		unregister_chrdev(spi_gpt_major, DRV_NAME);
		return PTR_ERR(spi_class);
	}

	return 0;
}
module_init(spi_init);

static void __exit spi_exit(void)
{
	device_destroy(spi_class, MKDEV(spi_gpt_major, 0));
	class_destroy(spi_class);
	unregister_chrdev(spi_gpt_major, DRV_NAME);
}
module_exit(spi_exit);

MODULE_AUTHOR("GPT Software");
MODULE_DESCRIPTION("Gpt Spi Slave driver");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_LICENSE("GPL");
