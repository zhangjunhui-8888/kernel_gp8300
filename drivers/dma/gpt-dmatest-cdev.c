#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <asm/cacheflush.h> 
#include <linux/syscalls.h>
#include <asm/cacheflush.h>
#include <asm/spr.h>
#include <asm/mach-chip2/dma.h>
#include "gpt-dma.h"
#include <linux/delay.h> 
#include <linux/string.h>
#include <linux/list.h>
#include <linux/device.h>

static int dma_gpt_major;
static struct class *dma_class;
static DEFINE_MUTEX(device_list_lock);

#define DRV_NAME "gpt-dma-cdev"

#define GDMA_IOC_MAGIC   'm'
#define DMA_IOC_WR_TRANSFER    _IOW(GDMA_IOC_MAGIC, 1, __u8)
#define DMA_IOC_WR_BUF_LEN     _IOW(GDMA_IOC_MAGIC, 2, __u16)

struct gpt_dma {
	dev_t devt;
	struct mutex lock;
	struct dma_chan  *chan;
	bool done_flag;
	wait_queue_head_t   dma_wait;

	u8 *src_buf[4];
	u8 *dst_buf[4];
	u16 src_len;
	u16 dst_len;
	dma_cookie_t cookie;
	dma_addr_t srcs[4];
	dma_addr_t dsts[4];
};
static struct gpt_dma *gdma;

static void dmatest_callback(void *arg)
{
	u32 i;
	u16 src_len = gdma->src_len;
	u16 dst_len = gdma->dst_len;
	enum dma_status  status;
	u8 *rx_buf_test = NULL;
	u8 *tx_buf_test = NULL;
	struct dma_device *dev = gdma->chan->device;

	//printk("%s\n", __func__);
	status = dma_async_is_tx_complete(gdma->chan, gdma->cookie, NULL, NULL);

	dmaengine_terminate_all(gdma->chan);

	for(i = 0; i < 1; i ++) {
		gpt_cmda_cache(gdma->src_buf[i], gdma->src_len + 10);
		gpt_cmda_cache(gdma->dst_buf[i], gdma->dst_len + 10);
	}

	/********test************/
	tx_buf_test = gdma->src_buf[0];
	rx_buf_test = gdma->dst_buf[0];

	for(i = 0; i < src_len; i ++) {
		if(*tx_buf_test++ != *rx_buf_test++){
			printk("DMA transfer failed! i = %d\n", i);
			break;
		}
	}

	if (i == src_len) 
		printk("DMA transfer success!\n");

	for(i = 0; i < 1; i ++) 
		dma_unmap_page(dev->dev, gdma->srcs[i], src_len, DMA_TO_DEVICE);

	for(i = 0; i < 1; i ++) 
		dma_unmap_page(dev->dev, gdma->dsts[i], dst_len, DMA_BIDIRECTIONAL);

//	dma_release_channel(gdma->chan);
	gdma->done_flag = 1;
	wake_up(&gdma->dma_wait);
}

static int gdma_request_buf(struct gpt_dma *gdma)
{
	u8 *buf = NULL;
	u64 i = 0;

	gdma->src_buf[0] = kzalloc(70000, GFP_KERNEL);
	if (!gdma->src_buf[0]) {
		printk("request tx memory failed!\n");
		return -ENOMEM;
	}

	gdma->dst_buf[0] = kzalloc(70000, GFP_KERNEL);
	if (!gdma->dst_buf[0]) {
		printk("request rx memory failed!\n");
		return -ENOMEM;
	}

	buf = gdma->src_buf[0]; 

	for(i = 0; i < 70000; i ++) 
		*buf ++ = 0x12 + i;

	return 0;
}

static int start_dma_transfer(struct gpt_dma *gdma)
{
	struct dma_chan *chan = gdma->chan;
	struct dma_device *dev = chan->device;
	dma_cookie_t cookie;
	enum dma_ctrl_flags flags;
	struct dma_async_tx_descriptor *tx = NULL;
	dma_addr_t srcs[4];
	dma_addr_t dsts[4];
	u16 src_len = gdma->src_len;
	u16 dst_len = gdma->dst_len;
	int i = 0, ret = 0;

	flags = DMA_CTRL_ACK | DMA_PREP_INTERRUPT;
	//gpt_cdma_l2_en();

	for (i = 0; i < 1; i++) {
		void *buf = gdma->src_buf[i];
		struct page *pg = virt_to_page(buf);
		unsigned pg_off = (unsigned long) buf & ~PAGE_MASK;

		srcs[i] = dma_map_page(dev->dev, pg, pg_off,
				src_len, DMA_TO_DEVICE);

		ret = dma_mapping_error(dev->dev, srcs[i]);
		if (ret) {
			printk("src dma mapping failed!\n");
		}

		gdma->srcs[i] = srcs[i];
		gpt_cmda_cache(gdma->src_buf[i], src_len + 10);
	}

	for (i = 0; i < 1; i++) {
		void *buf = gdma->dst_buf[i];
		struct page *pg = virt_to_page(buf);
		unsigned pg_off = (unsigned long) buf & ~PAGE_MASK;

		dsts[i] = dma_map_page(dev->dev, pg, pg_off, dst_len,
				DMA_BIDIRECTIONAL);

		ret = dma_mapping_error(dev->dev, dsts[i]);
		if (ret) {
			printk("dst dma mapping failed!\n");
		}

		gdma->dsts[i] = dsts[i];
		gpt_cmda_cache(gdma->dst_buf[i], dst_len + 10);
	}

	gpt_cdma_opsetn_mode(chan, 0, GPT_COPY, GPT_RD_SINGAL, GPT_WR_SINGAL, GPT_RB_NUM1, GPT_SIZE_1B, GPT_PAD_1B);
	gpt_cdma_group_para(chan, 0, src_len, 1, 0, dst_len, 1, 0);

	do {
		tx = dev->device_prep_dma_memcpy(chan, dsts[0], srcs[0], src_len, flags);
	} while (!tx);

	tx->callback = dmatest_callback;
	tx->callback_param = gdma;
	cookie = tx->tx_submit(tx);
	gdma->cookie = cookie;
	if (dma_submit_error(cookie)) {
		printk("%s :submit error\n", __func__);
	}

	dma_async_issue_pending(chan);

	return 0;
}

static bool filter(struct dma_chan *chan, void *param)
{
	if (gpt_strcmp(dma_chan_name(chan), "dma0chan0") == 0)
		return true;

	return false;
}

static int gpt_dma_transfer(struct gpt_dma *gdma)
{
	dma_cap_mask_t mask;
	struct dma_chan *chan;
	enum dma_transaction_type type = DMA_MEMCPY;

	dma_cap_zero(mask);
	dma_cap_set(type, mask);

	/*request a channel: struct dma_chan*/
	do {
	  //chan = dma_request_channel(mask, filter, NULL);
	  chan = dma_request_channel(mask, 0, NULL);
	} while (!chan);

	gdma->chan = chan;
	start_dma_transfer(gdma);

	return 0;
}

static int gdma_open(struct inode *inode, struct file *filp)
{
	//mutex_lock(&device_list_lock);
	gdma_request_buf(gdma);	
	nonseekable_open(inode, filp);
	//mutex_unlock(&device_list_lock);

	return 0;
}

static long gdma_ioctl(struct file *filp, unsigned int cmd,
		unsigned long arg)
{
	int retval = 0;
	u32 tmp;
	int err = 0;

	/* Check type and command number */
	if (_IOC_TYPE(cmd) != GDMA_IOC_MAGIC)
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

	mutex_lock(&gdma->lock);
	switch (cmd) {
		/* read requests */
		case DMA_IOC_WR_TRANSFER:
			gpt_dma_transfer(gdma);
			break;
		case DMA_IOC_WR_BUF_LEN:
			retval = __get_user(tmp, (u16 __user *)arg);
			if (retval == 0) { 
				gdma->src_len = gdma->dst_len = tmp;
			}
			break;
		default:
			break;
	}
	mutex_unlock(&gdma->lock);

	return retval;
}

static unsigned int gdma_poll(struct file *filp, poll_table *wait)
{
	unsigned int mask = 0;

	poll_wait(filp, &gdma->dma_wait, wait);

	if(gdma->done_flag == 1) {
		gdma->done_flag = 0;
		mask = POLLIN | POLLRDNORM; // user read data from mmap
	}

	return mask;
}

static int gdma_release(struct inode *inode, struct file *filp)
{
	mutex_lock(&device_list_lock);
	kfree(gdma->src_buf[0]);
	kfree(gdma->dst_buf[0]);
	dma_release_channel(gdma->chan);
	mutex_unlock(&device_list_lock);

	return 0;
}

static const struct file_operations gdma_ops = {
	.owner			= THIS_MODULE,
	.open			= gdma_open,
	.unlocked_ioctl = gdma_ioctl,
	.release		= gdma_release,
	.poll			= gdma_poll
};

static int __init gdma_init(void)
{
	dma_gpt_major = register_chrdev(0, DRV_NAME, &gdma_ops);
	if(dma_gpt_major < 0) {
		printk("fail to register one chardev!\n");
		return dma_gpt_major;
	}

	dma_class = class_create(THIS_MODULE, DRV_NAME);
	if (IS_ERR(dma_class)) {
		unregister_chrdev(dma_gpt_major, DRV_NAME);
		return PTR_ERR(dma_class);
	}

	gdma = kzalloc(sizeof(struct gpt_dma), GFP_KERNEL);
	if (!gdma) {
		return -ENOMEM;
	}

	gdma->done_flag = 0;

	gdma->devt = MKDEV(dma_gpt_major, 0);
	device_create(dma_class, NULL, gdma->devt, gdma, DRV_NAME);

	mutex_init(&gdma->lock);
	init_waitqueue_head(&gdma->dma_wait);

	return 0;
}

static void __exit gdma_exit(void)
{
	device_destroy(dma_class, MKDEV(dma_gpt_major, 0));
	class_destroy(dma_class);
	unregister_chrdev(dma_gpt_major, DRV_NAME);
	kfree(gdma);
}

module_init(gdma_init);
module_exit(gdma_exit);

MODULE_LICENSE("GPL v2");
