/*
 * i2s code driver for cs4334 and cs5343
 *
 * playback: cs5343    capture: cs4334
 *
 * only support sample size = 16
 *
 * Copyright (C) 2020 by GPT
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation.
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <sound/soc.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/list.h>
#include <linux/fs.h>
//#include <asm/mach-chip2/apiu.h>
#include "i2s-gpt-core.h"
#include "i2s-cs4334-cs5343.h"

#define DRV_NAME "cs4334_cs5343"

//#define AF_PRINT
#ifdef  AF_PRINT
#define  SP_PRINTK(format,arg...) do { \
	printk(KERN_INFO format,##arg);\
}while(0)
#else
#define  SP_PRINTK(format,arg...) do { \
}while(0)
#endif

//size for mmap: 32768 * n -- half for playback; half for capture
#define mmap_size 32768 * 500 
#define buf_num   5

static int i2s_major;
static struct class *i2s_class;
u8 *mmap_buffer;

struct i2s_dev {
	//	dev_t       devt; 
	struct device *dev;
	const char  *name;

	unsigned int sclk_freq;
	u8   mode;
	u8   align_mode;
	u16  sample_size;
	u16  frame_size;

	unsigned users;

	struct mutex lock;

	struct i2s_playback_buf *p_buf[buf_num];
	struct i2s_capture_buf *c_buf[buf_num];

	int poll_p_flag;
	int poll_c_flag;

	wait_queue_head_t   p_wait;
	wait_queue_head_t   c_wait;

	int en_playback_flag; //enable playback
	int en_capture_flag; //enable capture

	u8   playback_buf_num;
	u8   capture_buf_num;
	u32  playback_buf_size;
	u32  capture_buf_size;
	u8   playback_buf_id;
	u8   capture_buf_id;

	int stop_flag;

	struct completion    done;

	struct task_struct *task; //thread: playback and capture
};

struct i2s_dev *i2sdev;

static LIST_HEAD(queued_list);
static LIST_HEAD(dqueued_list);

/*
 * playback
 *@id: id of request buf
 *@flag; buf is available(1) or unavailable(0)
 *@data_buf: buf for request
 */
struct i2s_playback_buf {
	u8 id;
	u8 flag;
	u8 *data_buf; 
	struct list_head list;
};

/*
 * capture
 *@id: id of request buf
 *@flag; buf is available(1) or unavailable(0)
 *@data_buf: buf for request
 */
struct i2s_capture_buf {
	u8 id;
	u8 flag;
	u8 *data_buf; 
	struct list_head list;
};

/*
 *stop thread;disable I2S
 */
static int i2sdev_disable(struct i2s_dev *dev)
{
	dev->en_playback_flag = 0;
	dev->en_capture_flag = 0;
	reinit_completion(&dev->done);

	mdelay(10);
	i2s_gpt_enable(0);
	dev->stop_flag = 1;
	wait_for_completion(&dev->done);
	return 0;
}

/*
 *start playback and capture
 */
static int thread_i2s_start_playback_capture(void *data)
{
	u32 p_count = 0;
	u32 c_count = 0;
	u8 data_temp = 0;
	u8 byte_count = 0;
	u8 *p_data_buf = NULL; 
	u8 *c_data_buf = NULL; 
	u32 L_p_data = 0, R_p_data = 0;
	u32 L_c_data = 0, R_c_data = 0;
	struct i2s_playback_buf *p;
	struct i2s_capture_buf *c;
	u8 p_id = 0;
	u8 c_id = 0;
	u8 id = 0;
	u64 capture_data = 0;
	int i = 0;

	if (i2sdev->sample_size == 16) {
		byte_count = 2;
	} else if (i2sdev->sample_size == 24) {
		byte_count = 3;
	} else if (i2sdev->sample_size == 32) {
		byte_count = 4;
	}

	while(1) {
		/******************playback******************/
		if(i2sdev->en_playback_flag == 1) {
			if(p_count == 0) {
				if(!list_empty(&queued_list)) {
					list_for_each_entry(p, &queued_list, list) {
						p_id = p->id;
						break;
					}

					p_data_buf = i2sdev->p_buf[p_id]->data_buf;

				} else {
					p_data_buf = NULL;
				}
			}

			if(p_data_buf != NULL) {
				L_p_data = R_p_data = 0;

				for(i = 0; i < byte_count; i++)	{
					data_temp = *p_data_buf ++;
					L_p_data = L_p_data + (data_temp << (8 * i));
					p_count++;
				}

				for(i = 0; i < byte_count; i++)	{
					data_temp = *p_data_buf ++;
					R_p_data = R_p_data + (data_temp << (8 * i));
					p_count++;
				}

				i2s_gpt_playback(L_p_data, R_p_data);

				//when data in playback_buf is NULL, user write data to mmap
				if(p_count == i2sdev->playback_buf_size) {
					p_count = 0;

					list_del(&i2sdev->p_buf[p_id]->list);
					i2sdev->p_buf[p_id]->flag = 1;

					i2sdev->poll_p_flag = 1;
					wake_up(&i2sdev->p_wait);
				}
			}
		}

		/******************capture******************/
		if(i2sdev->en_capture_flag == 1) {
			if(!list_empty(&dqueued_list)) {
				list_for_each_entry(c, &dqueued_list, list)	{
					id = c->id;
					break;
				}

				memcpy(mmap_buffer + mmap_size / 4, i2sdev->c_buf[id]->data_buf, i2sdev->capture_buf_size);

				list_del(&i2sdev->c_buf[id]->list);
				i2sdev->c_buf[id]->flag = 1;

				i2sdev->poll_c_flag = 1;
				wake_up(&i2sdev->c_wait);
			}

			if(c_count == 0) {
				for(i = 0; i < i2sdev-> capture_buf_num; i++)
					if(i2sdev->c_buf[i]->flag == 1)
						break;

				c_id = i;
				c_data_buf = i2sdev->c_buf[c_id]->data_buf;
			}

			capture_data = i2s_gpt_capture();
			L_c_data = capture_data / (2 ^ 32);
			R_c_data = capture_data % (2 ^ 32);

			for(i = 0; i < byte_count; i++)	{
				data_temp = L_c_data >> (8 * i);
				*c_data_buf ++ = data_temp;
				c_count ++;
			}

			for(i = 0; i < byte_count; i++)	{
				data_temp = R_c_data >> (8 * i);
				*c_data_buf ++ = data_temp;
				c_count ++;
			}

			if(c_count == i2sdev->capture_buf_size)	{
				c_count = 0;

				list_add_tail(&i2sdev->c_buf[c_id]->list, &dqueued_list);
				i2sdev->c_buf[c_id]->flag = 0;
			}

		}

		if(i2sdev->stop_flag == 1) {
			i2sdev->stop_flag = 0;
			complete(&i2sdev->done);
			break;
		}

	}
	kthread_stop(i2sdev->task);

	return 0;
}

/*
 *request n buffer for list
 */
static int i2s_playback_request_buf(struct i2s_dev *dev)
{
	int status = 0;
	int i = 0;

	for(i = 0; i < dev->playback_buf_num; i++) {
		dev->p_buf[i] = kzalloc(sizeof(struct i2s_playback_buf), GFP_KERNEL);
		if (dev->p_buf[i] == NULL) {
			status = -ENOMEM;
			return status;
		}
	}

	for(i = 0; i < dev->playback_buf_num; i++) {
		dev->p_buf[i]->data_buf = kzalloc(dev->playback_buf_size, GFP_KERNEL);
		if (dev->p_buf[i]->data_buf == NULL) {
			status = -ENOMEM;
			return status;
		}

		dev->p_buf[i]->id = i;
		dev->p_buf[i]->flag = 1; 
	}

	return 0;
}

/*
 *request n buffer for list
 */
static int i2s_capture_request_buf(struct i2s_dev *dev)
{
	int status = 0;
	int i = 0;

	for(i = 0; i < dev->capture_buf_num; i++) {
		dev->c_buf[i] = kzalloc(sizeof(struct i2s_capture_buf), GFP_KERNEL);
		if (dev->c_buf[i] == NULL) {
			status = -ENOMEM;
			return status;
		}
	}

	for(i = 0; i < dev->capture_buf_num; i++) {
		dev->c_buf[i]->data_buf = kzalloc(dev->capture_buf_size, GFP_KERNEL);
		if (dev->c_buf[i]->data_buf == NULL) {
			status = -ENOMEM;
			return status;
		}

		dev->c_buf[i]->id = i;

		dev->c_buf[i]->flag = 1;
	}

	return 0;
}

static int i2s_playback_release_buf(struct i2s_dev *dev)
{
	int i = 0;

	for(i = 0; i < dev->playback_buf_num; i++) {
		kfree(dev->p_buf[i]->data_buf);
		kfree(dev->p_buf[i]);
	}

	return 0;
}

static int i2s_capture_release_buf(struct i2s_dev *dev)
{
	int i = 0;

	for(i = 0; i < dev->capture_buf_num; i++) {
		kfree(dev->c_buf[i]->data_buf);
		kfree(dev->c_buf[i]);
	}

	return 0;
}

static long i2sdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int retval = 0;
	int err = 0;
	u32 tmp = 0;
	int count = 0;

	struct i2s_dev *dev;

	/* Check type and command number */
	if (_IOC_TYPE(cmd) != I2S_IOC_MAGIC)
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

	dev = filp->private_data;

	mutex_lock(&dev->lock);

	switch (cmd) {
		/* read requests */
		case I2S_IOC_RD_MODE:
			retval = __put_user(dev->mode, (__u8 __user *)arg);
			break;
		case I2S_IOC_RD_ALIGN_MODE:
			retval = __put_user(dev->align_mode, (__u8 __user *)arg);
			break;
		case I2S_IOC_RD_SAMPLE_SIZE:
			retval = __put_user(dev->sample_size, (__u16 __user *)arg);
			break;
		case I2S_IOC_RD_FRAME_SIZE:
			retval = __put_user(dev->frame_size, (__u16 __user *)arg);
			break;
		case I2S_IOC_RD_MAX_BUF_SIZE:
			retval = __put_user(mmap_size / 2, (__u32 __user *)arg);
			break;
		case I2S_IOC_RD_MAX_BUF_NUM:
			retval = __put_user(buf_num, (__u8 __user *)arg);
			break;
		case I2S_IOC_RD_SCLK_FREQ:
			retval = __put_user(dev->sclk_freq, (__u32 __user *)arg);
			break;

			/* write requests */
		case I2S_IOC_WR_MODE:
			retval = __get_user(tmp, (__u8 __user *)arg);
			if (retval == 0) {
				dev->mode = tmp;
				retval = i2s_gpt_mode_setup(dev->mode, dev->align_mode, dev->sample_size, 
						dev->frame_size);
				if(retval != 0)
					return -EINVAL;
			}
			break;
		case I2S_IOC_WR_ALIGN_MODE:
			retval = __get_user(tmp, (__u8 __user *)arg);
			if (retval == 0) {
				dev->align_mode = tmp;
				retval = i2s_gpt_mode_setup(dev->mode, dev->align_mode, dev->sample_size, 
						dev->frame_size);
				if(retval != 0)
					return -EINVAL;
			}
			break;
		case I2S_IOC_WR_SAMPLE_SIZE:
			retval = __get_user(tmp, (__u16 __user *)arg);
			if (retval == 0) {
				dev->sample_size = tmp;
				i2s_gpt_mode_setup(dev->mode, dev->align_mode, dev->sample_size, 
						dev->frame_size);
				if(retval != 0)
					return -EINVAL;
			}
			break;
		case I2S_IOC_WR_FRAME_SIZE:
			retval = __get_user(tmp, (__u16 __user *)arg);
			if (retval == 0) {
				dev->frame_size = tmp;
				i2s_gpt_mode_setup(dev->mode, dev->align_mode, dev->sample_size, 
						dev->frame_size);
				if(retval != 0)
					return -EINVAL;
			}
			break;
		case I2S_IOC_WR_SCLK_FREQ:
			retval = __get_user(tmp, (__u32 __user *)arg);
			if (retval == 0) {
				dev->sclk_freq = tmp;
				i2s_gpt_sclk_setup(dev->sclk_freq);
			}
			break;
		case I2S_IOC_WR_PLAYBACK_BUF_SIZE:
			retval = __get_user(tmp, (__u32 __user *)arg);
			if (retval == 0) {
				dev->playback_buf_size =tmp;

				if(tmp > mmap_size / 2){
					return -EINVAL;
				}
			}
			break;
		case I2S_IOC_WR_CAPTURE_BUF_SIZE:
			retval = __get_user(tmp, (__u32 __user *)arg);
			if (retval == 0) {
				dev->capture_buf_size = tmp;

				if(tmp > mmap_size / 2){
					return -EINVAL;
				}
			}
			break;
		case I2S_IOC_WR_PLAYBACK_BUF_NUM:
			retval = __get_user(tmp, (__u8 __user *)arg);
			if (retval == 0) {
				dev->playback_buf_num = tmp;
				if(tmp > buf_num)
					return -EINVAL;
			}
			break;
		case I2S_IOC_WR_CAPTURE_BUF_NUM:
			retval = __get_user(tmp, (__u8 __user *)arg);
			if (retval == 0) {
				dev->capture_buf_num = tmp;

				if(tmp > buf_num)
					return -EINVAL;
			}
			break;
		case I2S_IOC_WR_PLAYBACK_REQUEST_BUF:
			i2s_playback_request_buf(dev);
			break;
		case I2S_IOC_WR_CAPTURE_REQUEST_BUF:
			i2s_capture_request_buf(dev);
			break;
		case I2S_IOC_WR_PLAYBACK_RELEASE_BUF:
			i2s_playback_release_buf(dev);
			break;
		case I2S_IOC_WR_CAPTURE_RELEASE_BUF:
			i2s_capture_release_buf(dev);
			break;
		case I2S_IOC_WR_I2S_ENABLE:
			retval = __get_user(tmp, (__u8 __user *)arg);
			if (retval == 0) {
				if (tmp == 1) { //enable
					i2s_gpt_enable(1);

					dev->task = kthread_run(thread_i2s_start_playback_capture, NULL, "thread: i2s_dev");
					if (IS_ERR(dev->task)) {
						retval = PTR_ERR(dev->task);
						printk("thread_start fail to create\n");
						dev->task = NULL;
					}
				} else if (tmp == 0){ //disable
					i2sdev_disable(dev);

				} else {
					return -EINVAL;
				}
			}
			break;
		case I2S_IOC_WR_PLAYBACK:
			retval = __get_user(tmp, (__u8 __user *)arg);
			if (retval == 0) {
				if(tmp == 1) { //start
					dev->en_playback_flag = 1;

				} else if (tmp == 0) { //suspend
					dev->en_playback_flag = 0;

				} else {
					return -EINVAL;
				}
			}
			break;
		case I2S_IOC_WR_CAPTURE:
			retval = __get_user(tmp, (__u8 __user *)arg);
			if (retval == 0) {
				if(tmp == 1) { //start
					dev->en_capture_flag = 1;

				} else if (tmp == 0) { //suspend
					dev->en_capture_flag = 0;

				} else {
					return -EINVAL;
				}
			}
			break;
		case I2S_IOC_WR_PLAYBACK_QUEUE:
			for(count = 0; count < dev->playback_buf_num; count++)
				if(dev->p_buf[count]->flag == 1)
					break;

			if(count == dev->playback_buf_num) {
				return -EINVAL;
			} else {
				memcpy(dev->p_buf[count]->data_buf, mmap_buffer, dev->playback_buf_size);	
				list_add_tail(&dev->p_buf[count]->list, &queued_list);
				dev->p_buf[count]->flag = 0;
			}
			break;
		default:
			return -EINVAL;
			break;
	}

	mutex_unlock(&dev->lock);

	return 0;
}

static int i2sdev_open(struct inode *inode, struct file *filp)
{
	u32 ret = 0;

	filp->private_data = i2sdev;

	mutex_lock(&i2sdev->lock);
	i2sdev->users++;
	nonseekable_open(inode, filp);

	mmap_buffer = kzalloc(mmap_size, GFP_KERNEL);
	if(!mmap_buffer)
		ret = -ENOMEM;

	INIT_LIST_HEAD(&queued_list);
	INIT_LIST_HEAD(&dqueued_list);

	mutex_unlock(&i2sdev->lock);

	return ret;
}

static int i2sdev_release(struct inode *inode, struct file *filp)
{
	struct i2s_dev *dev;

	dev = filp->private_data;

	dev->users--;
	filp->private_data = NULL;

	kfree(mmap_buffer);
	return 0;
}

/*
 *alloc buffer for user to driver (driver to user)
 *
 */
static int i2sdev_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct i2s_dev *dev;
	unsigned long page;

	unsigned long start = (unsigned long)(vma->vm_start);
	unsigned long size = (unsigned long)(vma->vm_end - vma->vm_start);

	dev = filp->private_data;

	page = virt_to_phys(mmap_buffer);
	if(remap_pfn_range(vma, start, page >> PAGE_SHIFT, size, PAGE_SHARED))
		return -1;

	return 0;
}

static unsigned int i2sdev_poll(struct file *filp, poll_table *wait)
{
	struct i2s_dev *dev;
	unsigned int mask = 0;

	dev = filp->private_data;

	poll_wait(filp, &dev->p_wait, wait);
	poll_wait(filp, &dev->c_wait, wait);

	if(dev->poll_p_flag == 1) {
		dev->poll_p_flag = 0;
		mask = POLLOUT | POLLWRNORM; // user write data to mmap
	}


	if(dev->poll_c_flag == 1) {
		dev->poll_c_flag = 0;
		mask = POLLIN | POLLRDNORM; // user read data from mmap
	}

	return mask;
}

static const struct file_operations i2sdev_ops = {
	.owner = THIS_MODULE,
	.open = i2sdev_open,
	.unlocked_ioctl = i2sdev_ioctl,
	.release = i2sdev_release,
	.mmap = i2sdev_mmap,
	.poll = i2sdev_poll
};

static int __init i2sdev_init(void)
{
	int ret;
	i2sdev = kzalloc(sizeof(struct i2s_dev), GFP_KERNEL);
	if (!i2sdev) {
		printk("Can't allocate for i2sdev\n");
		return -ENOMEM;
	}

	i2sdev->poll_p_flag = 0;
	i2sdev->poll_c_flag = 0;

	init_waitqueue_head(&i2sdev->p_wait);
	init_waitqueue_head(&i2sdev->c_wait);

	i2sdev->en_playback_flag = 0;
	i2sdev->en_capture_flag = 0;

	i2sdev->playback_buf_num = 1;
	i2sdev->capture_buf_num = 1;
	i2sdev->playback_buf_size = mmap_size / 2;
	i2sdev->capture_buf_size = mmap_size / 2;
	i2sdev->playback_buf_id = 0;
	i2sdev->capture_buf_id = 0;
	i2sdev->stop_flag = 0;

	i2sdev->mode = 0;
	i2sdev->align_mode = 0;
	i2sdev->sample_size = 16;
	i2sdev->frame_size = 32;
	i2s_gpt_mode_setup(i2sdev->mode, i2sdev->align_mode, i2sdev->sample_size, 
			i2sdev->frame_size);

	i2sdev->sclk_freq = 1536000;
	i2s_gpt_sclk_setup(i2sdev->sclk_freq);

	init_completion(&i2sdev->done);
	mutex_init(&i2sdev->lock);

	i2s_major = register_chrdev(0, DRV_NAME, &i2sdev_ops);
	if(i2s_major < 0)
	{
		printk("fail to register one chardev!\n");
		return i2s_major;
	}

	i2s_class = class_create(THIS_MODULE, DRV_NAME);
	if (IS_ERR(i2s_class)) {
		unregister_chrdev(i2s_major, DRV_NAME);
		return PTR_ERR(i2s_class);
	}

	device_create(i2s_class, NULL, MKDEV(i2s_major, 0),
			NULL, "i2s0-dev");

	return 0;
}
module_init(i2sdev_init);

static void __exit i2sdev_exit(void)
{
	device_destroy(i2s_class, MKDEV(i2s_major, 0));
	class_destroy(i2s_class);
	unregister_chrdev(i2s_major, DRV_NAME);

	kfree(i2sdev);
}
module_exit(i2sdev_exit);

MODULE_DESCRIPTION("i2s code driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
