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

#define DRV_NAME "spi-gpt-slave"

#define TIME_DELAY 1000
#define TX_FIFO_TH 32
#define SPI_FULL_DUPLEX 0
#define SPI_HALF_DUPLEX 1
#define ONLY_TRANSFER 0
#define ONLY_RECEIVE 1

#define SPI_MODE_MASK  (SPI_CPHA | SPI_CPOL)

static int spi_gpt_major;
static struct class *spi_class;
static LIST_HEAD(device_list);
static LIST_HEAD(queued_list);
static DEFINE_MUTEX(device_list_lock);

#define N_SPI_MINORS 10	/* ... up to 256 */
static DECLARE_BITMAP(minors, N_SPI_MINORS);

struct gpt_spi_slave {
	dev_t devt;
	int	  irq;
	struct device		*dev;
	struct list_head    device_entry;
	void __iomem		*regs;
	struct mutex		 lock;
	struct clk			*clk;

	int bus_num;
	u8	tx_buf_num;
	u32 tx_buf_size;
	u8  flag_completion;

	u8	 cur_bpw;
	u16	 cur_mode;
	u32  r_len;
	u32  s_len;

	u8  *tx_buf8;
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
	unsigned char spi_transfer_type;
	unsigned char spi_half_duplex_type;
	u32 spi_will_rx_len;
};

struct tx_buf {
	u8 id;
	u8 flag;
	u16 len;
	u8 *data; 
	struct list_head list;
};

static void gspi_write32(u32 val, void __iomem *addr)
{
	iowrite32(val, addr);
}

static unsigned int gspi_read32(void __iomem *addr)
{
	return ioread32(addr);
}

static int spi_slave_setup(struct gpt_spi_slave *gspi)
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

static void spi_slave_transfer(struct gpt_spi_slave *gspi)
{
	u32 len = 0;
	u32 count = 0;
	int i = 0;
	u8 id = 0;
	struct tx_buf *tx;
	u32 tx_fifo_th = 0;

	if (!list_empty(&queued_list)) {
		list_for_each_entry(tx, &queued_list, list)	{
			id = tx->id;
			break;
		}

		if (gspi->spi_transfer_type == SPI_HALF_DUPLEX) {
			tx_fifo_th = TX_FIFO_TH;
		} else if (gspi->spi_transfer_type == SPI_FULL_DUPLEX) {
			tx_fifo_th = GPT_SPI_FIFO_DEPTH; 
		}

		if (gspi->flag_completion == 0) {
			if (gspi->cur_bpw == 8) {
				gspi->tx_buf8 = (u8 *)gspi->tx_buf[id]->data;
			} else if (gspi->cur_bpw == 16) {
				gspi->tx_buf16 = (u16 *)gspi->tx_buf[id]->data;
			} else if (gspi->cur_bpw == 32) {
				gspi->tx_buf32 = (u32 *)gspi->tx_buf[id]->data;
			}
		}

		gspi->s_len = gspi->tx_buf[id]->len;

		if (gspi->cur_bpw == 8) {
			len = gspi->s_len;
		} else if (gspi->cur_bpw == 16) {
			len = gspi->s_len / 2;
		} else if (gspi->cur_bpw == 32) {
			len = gspi->s_len / 4;
		}

		if (len >= tx_fifo_th) {
			count = tx_fifo_th;

			if(gspi->cur_bpw == 8) {
				for (i = 0; i < count; i ++)
					gspi_write32(*gspi->tx_buf8 ++, gspi->regs + GPT_SPI_TX_DATA);
				gspi->tx_buf[id]->len -= count;

			} else if (gspi->cur_bpw == 16) {
				for (i = 0; i < count; i ++)
					gspi_write32(*gspi->tx_buf16 ++, gspi->regs + GPT_SPI_TX_DATA);
				gspi->tx_buf[id]->len -= count * 2;

			} else if (gspi->cur_bpw == 32) {
				for (i = 0; i < count; i ++)
					gspi_write32(*gspi->tx_buf32 ++, gspi->regs + GPT_SPI_TX_DATA);
				gspi->tx_buf[id]->len -= count * 4;
			}

		} else {
			count = len;

			if (gspi->cur_bpw == 8) {
				for (i = 0; i < count; i ++)
					gspi_write32(*gspi->tx_buf8 ++, gspi->regs + GPT_SPI_TX_DATA);
				gspi->tx_buf[id]->len -= count;

			} else if (gspi->cur_bpw == 16) {
				for (i = 0; i < count; i ++)
					gspi_write32(*gspi->tx_buf16 ++, gspi->regs + GPT_SPI_TX_DATA);
				gspi->tx_buf[id]->len -= count * 2;

			} else if (gspi->cur_bpw == 32) {
				for (i = 0; i < count; i ++)
					gspi_write32(*gspi->tx_buf32 ++, gspi->regs + GPT_SPI_TX_DATA);
				gspi->tx_buf[id]->len -= count * 4;
			}
		}

		if (gspi->tx_buf[id]->len == 0) 
		{
			list_del(&gspi->tx_buf[id]->list);
			gspi->tx_buf[id]->flag = 0;
			gspi->flag_completion = 0;

			//there is empty buffer for user to write data to kernel
			gspi->poll_w_flag = 1;
			wake_up(&gspi->w_wait);
		} else {
			gspi->flag_completion = 1;
		}

	} else {
		if (gspi->spi_transfer_type == SPI_HALF_DUPLEX && 
				gspi->spi_half_duplex_type == ONLY_TRANSFER) 
			gspi_write32(0x8, gspi->regs + GPT_SPI_IRQ_MASK);//IRQ: TX empty
	}
}

static int gspi_init_hwinit(struct gpt_spi_slave *gspi)
{
	apiu_unit_init(gspi->regs, 0);

	/*salve mode; CPOL=1,CPHA=0; MSB first; 8bit*/
	gspi_write32(GPT_SPI_MSB | GPT_SPI_SLV_XFER_SAME_EDGE, gspi->regs + GPT_SPI_CFG);
	gspi_write32(GPT_SPI_BIT_8, gspi->regs + GPT_SPI_BIT_MODE);
	gspi_write32(0, gspi->regs + GPT_SPI_DIV_CNT);
	gspi_write32(GPT_SPI_RX_FLUSH | GPT_SPI_TX_FLUSH, gspi->regs + GPT_SPI_FIFO_CLEAR);
	gspi_write32(0x0, gspi->regs + GPT_SPI_FIFO_OP);
	/*tx fifo: alm_ampty TH=32; rx fifo: alm_full TH=1*/
	gspi_write32(0x1f00, gspi->regs + GPT_SPI_FIFO_CFG);
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
	u32 len = 0;
	int i = 0;
	u8  *buf8  = NULL;
	u16 *buf16 = NULL;
	u32 *buf32 = NULL;

	struct gpt_spi_slave *gspi = dev_id;

	value = gspi_read32(gspi->regs + GPT_SPI_IRQ_STATUS);

	if (GPT_SPI_RX_FIFO_ALMOST_FULL & value) {
		if (gspi->cur_bpw == 8)	{
			buf8 = gspi->rx_buf;

#if 0
			i = gspi_read32(gspi->regs + GPT_SPI_FIFO_CUR_CNT);
			num = i >> 8;

			for (i = 0; i < num; i ++)
				*buf8 ++ = gspi_read32(gspi->regs + GPT_SPI_RX_DATA);
#endif

#if 1
			while (1) {
				i = gspi_read32(gspi->regs + GPT_SPI_FIFO_CUR_CNT);
				num = i >> 8;
				if (num > 0) {
					while (num-- > 0) {
						*buf8 ++ = gspi_read32(gspi->regs + GPT_SPI_RX_DATA);
						len ++;
					}

					if (len == gspi->spi_will_rx_len)
						break;

					t0 = get_timeus();
				} else {
					t1 = get_timeus();
					//if (t1 - t0 > TIME_DELAY / 2)
					/*if slave don`t get the data within 1 second, exit the acceptance program. This transfer fail.*/
					if (t1 - t0 > 1000000)
						break;
				}
			}
#endif

		} else if (gspi->cur_bpw == 16) {
			buf16 = gspi->rx_buf;

#if 0
			i = gspi_read32(gspi->regs + GPT_SPI_FIFO_CUR_CNT);
			num = i >> 8;

			for (i = 0; i < num; i ++)
				*buf16 ++ = gspi_read32(gspi->regs + GPT_SPI_RX_DATA);
#endif

#if 1
			while (1) {
				i = gspi_read32(gspi->regs + GPT_SPI_FIFO_CUR_CNT);
				num = i >> 8;
				if (num > 0) {
					while (num-- > 0) {
						*buf16++ = gspi_read32(gspi->regs + GPT_SPI_RX_DATA);
						len = len + 2;
					}
					if (len == gspi->spi_will_rx_len)
						break;

					t0 = get_timeus();
				} else {
					t1 = get_timeus();
					//if (t1 - t0 > TIME_DELAY)
					if (t1 - t0 > 1000000)
						break;
				}
			}
#endif

		} else if (gspi->cur_bpw == 32) {
			buf32 = gspi->rx_buf;

#if 0
			i = gspi_read32(gspi->regs + GPT_SPI_FIFO_CUR_CNT);
			num = i >> 8;

			for (i = 0; i < num; i ++)
				*buf32 ++ = gspi_read32(gspi->regs + GPT_SPI_RX_DATA);
#endif

#if 1
			while (1) {
				i = gspi_read32(gspi->regs + GPT_SPI_FIFO_CUR_CNT);
				num = i >> 8;
				if (num > 0) {
					while (num-- > 0) {
						*buf32++ = gspi_read32(gspi->regs + GPT_SPI_RX_DATA);
						len = len + 4;
					}
					if (len == gspi->spi_will_rx_len)
						break;

					t0 = get_timeus();
				} else {
					t1 = get_timeus();
					//if ( t1 - t0 > TIME_DELAY * 2)
					if (t1 - t0 > 1000000)
						break;
				}
			}
#endif
		}

		gspi->r_len = len;
		gspi->poll_r_flag = 1;
		wake_up(&gspi->r_wait);

		gspi_write32(GPT_SPI_RX_FLUSH | GPT_SPI_TX_FLUSH, gspi->regs + GPT_SPI_FIFO_CLEAR);
		gspi_write32(0x2, gspi->regs + GPT_SPI_FIFO_OP);
		gspi_write32(0x1fff, gspi->regs + GPT_SPI_IRQ_CLEAR);

		return IRQ_HANDLED;

	} else if (GPT_SPI_TX_FIFO_ALMOST_EMPTY & value) {
		gspi_write32(0x1fff, gspi->regs + GPT_SPI_IRQ_CLEAR);
		spi_slave_transfer(gspi);

		return IRQ_HANDLED;

	} else if (GPT_SPI_TX_FIFO_EMPTY & value) {
		gspi_write32(0x1fff, gspi->regs + GPT_SPI_IRQ_CLEAR);
#if 0
		gspi_write32(0x0, gspi->regs + GPT_SPI_FIFO_OP);
		for (i = 0; i < 64; i ++) 
			gspi_read32(gspi->regs + GPT_SPI_TX_DATA);
#endif
		gspi_write32(GPT_SPI_TX_FLUSH, gspi->regs + GPT_SPI_FIFO_CLEAR);
		gspi_write32(0x2, gspi->regs + GPT_SPI_FIFO_OP);
		gspi_write32(0x40, gspi->regs + GPT_SPI_IRQ_MASK);//IRQ: RX almost full

		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static int spi_request_buf(struct gpt_spi_slave *gspi)
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
			printk("%s: alloc spi rx memory failed\n", __func__);
			return -ENOMEM;
		}

		gspi->tx_buf[i]->id = i;
		gspi->tx_buf[i]->len = 0;
		gspi->tx_buf[i]->flag = 0;
	}

	/*
	 *alloc 20M for rx buffer.
	 *max memory is 32M.
	 */
	gspi->rx_buf = kzalloc(20 * 1024 * 1024, GFP_KERNEL);
	if (gspi->rx_buf == NULL) {
		printk("%s: alloc spi rx memory failed\n", __func__);
		return -ENOMEM;
	}

	return status;
}

static int spi_release_buf(struct gpt_spi_slave *gspi)
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
	u32 rx_th;

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
				spi_slave_setup(gspi);
			}
			break;
		case SPI_IOC_WR_BITS_PER_WORD:
			retval = __get_user(tmp, (__u8 __user *)arg);
			if (retval == 0) {
				gspi->cur_bpw = tmp;
				spi_slave_setup(gspi);
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
			retval = __get_user(tmp, (__u32 __user *)arg);
			if (retval == 0) {
				gspi->tx_buf_size = tmp;
				if(tmp > 1UL<<32) {
					retval = -EINVAL;
					break;
				}
			}
			break;
		case SPI_IOC_WR_REQUEST_BUF:
			spi_request_buf(gspi);
			break;
		case SPI_IOC_WR_RELEASE_BUF:
			spi_release_buf(gspi);
			break;
		case SPI_IOC_WR_FULL_DUPLEX:
			gspi->spi_transfer_type = SPI_FULL_DUPLEX;
			gspi_write32(0x40, gspi->regs + GPT_SPI_IRQ_MASK);
			break;
		case SPI_IOC_WR_HALF_DUPLEX:
			gspi->spi_transfer_type = SPI_HALF_DUPLEX;

			retval = __get_user(tmp, (__u8 __user *)arg);
			if (retval == 0) {
				if (tmp == ONLY_TRANSFER) {
					gspi->spi_half_duplex_type = ONLY_TRANSFER;
				} else if (tmp == ONLY_RECEIVE) {
					gspi->spi_half_duplex_type = ONLY_RECEIVE;
					gspi_write32(0x2, gspi->regs + GPT_SPI_FIFO_OP);
					gspi_write32(0x40, gspi->regs + GPT_SPI_IRQ_MASK);
				}
			}
			break;
		case SPI_IOC_WR_WILL_RX_LEN:
			retval = __get_user(tmp, (__u32 __user *)arg);
			if (retval == 0) {
				gspi->spi_will_rx_len = tmp;
#if 0
				/*set RX FIFO threshold*/
				if (tmp > 64)
					tmp = 63;
				else 
					tmp -= 1;

				rx_th = gspi_read32(gspi->regs + GPT_SPI_FIFO_CFG);
				rx_th &= 0xffff;
				rx_th |= (tmp << 16);
				gspi_write32(rx_th, gspi->regs + GPT_SPI_FIFO_CFG);
#endif
			}
			break;
		case SPI_IOC_WR_TRANSFER_EN:
			if (gspi->spi_transfer_type == SPI_HALF_DUPLEX) {
				if (gspi->spi_half_duplex_type == ONLY_TRANSFER) {
					spi_slave_transfer(gspi);
					gspi_write32(0x2, gspi->regs + GPT_SPI_IRQ_MASK);//IRQ: TX almost empty
					gspi_write32(0x1, gspi->regs + GPT_SPI_FIFO_OP);
				}

			} else if (gspi->spi_transfer_type == SPI_FULL_DUPLEX) {
				spi_slave_transfer(gspi);
				gspi_write32(0x3, gspi->regs + GPT_SPI_FIFO_OP);
			}
			break;
		default:
			break;
	}
	mutex_unlock(&gspi->lock);

	return retval;
}

static int spi_gpt_open(struct inode *inode, struct file *filp)
{
	int status = -ENXIO;
	struct gpt_spi_slave *gspi;

	mutex_lock(&device_list_lock);

	list_for_each_entry(gspi, &device_list, device_entry) {
		if (gspi->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}

	if (status) {
		pr_debug("spidev: nothing for minor %d\n", iminor(inode));
		goto err_find_dev;
	}

	filp->private_data = gspi;

	gspi->users ++;
	nonseekable_open(inode, filp);

	INIT_LIST_HEAD(&queued_list);

	gspi_write32(GPT_SPI_RX_FLUSH | GPT_SPI_TX_FLUSH, gspi->regs + GPT_SPI_FIFO_CLEAR);
	gspi_write32(0x1fff, gspi->regs + GPT_SPI_IRQ_CLEAR);
	gspi_write32(0x40, gspi->regs + GPT_SPI_IRQ_MASK);
	gspi_write32(0x2, gspi->regs + GPT_SPI_FIFO_OP);

	mutex_unlock(&device_list_lock);

	return 0;

err_find_dev:
	mutex_unlock(&device_list_lock);
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
		if (ret)
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

	mutex_lock(&device_list_lock);
	gspi->users--;

	gspi_write32(0x0, gspi->regs + GPT_SPI_IRQ_MASK); /*disable irq*/
	gspi_write32(0x0, gspi->regs + GPT_SPI_FIFO_OP); /*disable TX/RX QUEUE*/
	gspi_write32(GPT_SPI_RX_FLUSH | GPT_SPI_TX_FLUSH, gspi->regs + GPT_SPI_FIFO_CLEAR); /*CLR FIFO*/
	gspi_write32(0x1fff, gspi->regs + GPT_SPI_IRQ_CLEAR); /*CLR INT*/

	for(i = 0; i < gspi->tx_buf_num; i++)
		gspi->tx_buf[i]->flag = 0;

	gspi->flag_completion = 0;
	filp->private_data = NULL;
	mutex_unlock(&device_list_lock);

	return 0;
}

unsigned int spi_gpt_poll(struct file *filp, poll_table *wait)
{
	struct gpt_spi_slave *gspi;
	unsigned int mask = 0;

	gspi = filp->private_data;

	poll_wait(filp, &gspi->r_wait, wait);
	poll_wait(filp, &gspi->w_wait, wait);

	/*
	 *user can write data to kernel
	 */
	if(gspi->poll_w_flag == 1) {
		gspi->poll_w_flag = 0;
		mask = POLLOUT | POLLWRNORM; 
	}

	/*
	 *user can read data from kernel
	 */
	if(gspi->poll_r_flag == 1) {
		gspi->poll_r_flag = 0;
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
	{ .compatible = "gpt,spi1.0", },
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
	unsigned long minor;
	struct device_node *node;
	struct gpt_spi_slave *gspi;
	node = pdev->dev.of_node;

	gpt_spi_parse_dt(pdev);

	gspi = devm_kzalloc(&pdev->dev, sizeof(struct gpt_spi_slave), GFP_KERNEL);
	if (!gspi) {
		dev_err(&pdev->dev, "Can't allocate for gpt_spi\n");
		return -ENOMEM;
	}

	mutex_init(&gspi->lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	gspi->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(gspi->regs)) {
		return PTR_ERR(gspi->regs);
	}

	apiu_unit_init(&gspi->regs, 0);
	gspi->read_fn = gspi_read32;
	gspi->write_fn = gspi_write32;

	init_waitqueue_head(&gspi->r_wait);
	init_waitqueue_head(&gspi->w_wait);

	gspi->poll_r_flag = 0;
	gspi->poll_w_flag = 0;
	gspi->flag_completion = 0;

	gspi->cur_bpw = 8;
	gspi->cur_mode = SPI_MODE_0;
	gspi->r_len = 1;
	gspi->s_len = 1;
	gspi->tx_buf_num = 1;
	gspi->tx_buf_size = 1028;
	gspi->spi_transfer_type = SPI_HALF_DUPLEX;
	gspi->spi_will_rx_len = 8;

	gspi->dev = &pdev->dev;
	platform_set_drvdata(pdev, gspi);

	/* SPI controller initializations */
	gspi_init_hwinit(gspi);

	INIT_LIST_HEAD(&gspi->device_entry);

	gspi->bus_num = of_alias_get_id(node, "spi");

	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		struct device *dev;

		gspi->devt = MKDEV(spi_gpt_major, 0);
		dev = device_create(spi_class, &pdev->dev, gspi->devt,
				gspi, "spi%d-gpt-slave", gspi->bus_num);

		ret = PTR_ERR_OR_ZERO(dev);
		if (ret == 0) {
			set_bit(minor, minors);
			list_add(&gspi->device_entry, &device_list);
		} else {
			dev_err(gspi->dev, "create dev failed!\n");
			mutex_unlock(&device_list_lock);
			return ret;
		}

	} else {
		dev_err(gspi->dev, "no minor number available!\n");
		mutex_unlock(&device_list_lock);
		return -ENODEV;
	}
	mutex_unlock(&device_list_lock);

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

	mutex_lock(&device_list_lock);
	list_del(&gspi->device_entry);
	clear_bit(MINOR(gspi->devt), minors);
	mutex_unlock(&device_list_lock);

	return 0;
}

MODULE_ALIAS("platform:" GPT_SPI_NAME);

static struct platform_driver gpt_spi_driver = {
	.probe = gpt_spi_probe,
	.remove = gpt_spi_remove,
	.driver = {
		.name = DRV_NAME,
		.of_match_table = of_match_ptr(gpt_spi_of_match),
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
