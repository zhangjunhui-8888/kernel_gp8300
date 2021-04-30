/*
 * DMA Engine test module
 *
 * Copyright (C) 2007 Atmel Corporation
 * Copyright (C) 2013 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/freezer.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/random.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <asm/cacheflush.h>
#include "gpt-dma.h"

static char test_channel[20];
module_param_string(channel, test_channel, sizeof(test_channel),
		S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(channel, "Bus ID of the channel to test (default: any)");

static char test_device[32];
module_param_string(device, test_device, sizeof(test_device),
		S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(device, "Bus ID of the DMA Engine to test (default: any)");

static unsigned char memcpy_type = 0;
module_param(memcpy_type, byte, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(memcpy_type, "memory copy type (default: 0)");

static unsigned long tx_buf_len = 65536;
module_param(tx_buf_len, ulong, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(tx_buf_len, "tx_buf_len (default: 32)");

static unsigned long rx_buf_len = 65536;
module_param(rx_buf_len, ulong, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(rx_buf_len, "rx_buf_len (default: 32)");

static unsigned char src_nents = 1;
module_param(src_nents, byte, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(src_nents, "src_nents (default: 1)");

static unsigned char dst_nents = 1;
module_param(dst_nents, byte, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(dst_nents, "dst_nents (default: 1)");

static unsigned char test_mode = 1;
module_param(test_mode, byte, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(test_mode, "test_mode (default: 1)");

static int timeout = 3000;
module_param(timeout, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(timeout, "Transfer Timeout in msec (default: 3000), "
		"Pass -1 for infinite timeout");

u8 *tx_buf[4];
u8 *rx_buf[4];

/**
 * struct dmatest_params - test parameters.
 * @channel:		bus ID of the channel to test
 * @device:		bus ID of the DMA Engine to test
 * @threads_per_chan:	number of threads to start per channel
 * @iterations:		iterations before stopping test
 */
struct dmatest_params {
	char		channel[20];
	char		device[32];
	unsigned int	threads_per_chan;
	unsigned int	iterations;
	int     timeout;
};

/**
 * struct dmatest_info - test information.
 * @params:		test parameters
 * @lock:		access protection to the fields of this structure
 */
static struct dmatest_info {
	/* Test parameters */
	struct dmatest_params	params;

	/* Internal state */
	struct list_head	channels;
	unsigned int		nr_channels;
	struct mutex		lock;
	bool			did_init;
} test_info = {
	.channels = LIST_HEAD_INIT(test_info.channels),
	.lock = __MUTEX_INITIALIZER(test_info.lock),
};

static int dmatest_run_set(const char *val, const struct kernel_param *kp);
static int dmatest_run_get(char *val, const struct kernel_param *kp);
static struct kernel_param_ops run_ops = {
	.set = dmatest_run_set,
	.get = dmatest_run_get,
};

static bool dmatest_run;
module_param_cb(run, &run_ops, &dmatest_run, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(run, "Run the test (default: false)");

/*alloc mem in func: dmatest_add_threads()*/
struct dmatest_thread {
	struct list_head	node;
	struct dmatest_info	*info;
	struct task_struct	*task;
	struct dma_chan		*chan;
	u8			**srcs;
	u8			**dsts;
	enum dma_transaction_type type;
	bool			done;
};

/*alloc mem in func: dmatest_add_channel()*/
struct dmatest_chan {
	struct list_head	node;
	struct dma_chan		*chan;
	struct list_head	threads;
};

static DECLARE_WAIT_QUEUE_HEAD(thread_wait);
static bool wait;

static long get_timeus(void)
{
	struct timeval tv;

	do_gettimeofday(&tv);
	return tv.tv_sec * 1000000 + tv.tv_usec;
}

static bool is_threaded_test_run(struct dmatest_info *info)
{
	struct dmatest_chan *dtc;

	list_for_each_entry(dtc, &info->channels, node) {
		struct dmatest_thread *thread;

		list_for_each_entry(thread, &dtc->threads, node) {
			if (!thread->done)
				return true;
		}
	}

	return false;
}

static int dmatest_wait_get(char *val, const struct kernel_param *kp)
{
	struct dmatest_info *info = &test_info;
	struct dmatest_params *params = &info->params;

	if (params->iterations)
		wait_event(thread_wait, !is_threaded_test_run(info));
	wait = true;
	return param_get_bool(val, kp);
}

static struct kernel_param_ops wait_ops = {
	.get = dmatest_wait_get,
	.set = param_set_bool,
};
module_param_cb(wait, &wait_ops, &wait, S_IRUGO);
MODULE_PARM_DESC(wait, "Wait for tests to complete (default: false)");

struct dmatest_done {
	bool			done;
	wait_queue_head_t	*wait;
};

static void dmatest_callback(void *arg)
{
	struct dmatest_done *done = arg;

	done->done = true;
	wake_up_all(done->wait);
}

static void data_test1(void)
{
	u64 i = 0;
	u8 *rx_buf_test = NULL;
	u8 *tx_buf_test = NULL;

#if 1
	printk("***************rx data***************\n");
	rx_buf_test = rx_buf[0];
	for(i = 1; i <= tx_buf_len; i ++) {
		if(i % 32){
			printk("%02x ", *rx_buf_test++);
		}else {
			printk("%02x\n", *rx_buf_test++);
		}
	}
	printk("\n");
#endif

	tx_buf_test = tx_buf[0];
	rx_buf_test = rx_buf[0];

	for(i = 0; i < tx_buf_len; i ++) {
		if(*tx_buf_test++ != *rx_buf_test++){
			printk("DMA transfer failed!\n");
			break;
		}
	}

	if (i == tx_buf_len)
		printk("DMA transfer success!\n");

	printk("\n");
}

static void data_test2(void)
{
	u64 i = 0;
	u8 *rx_buf_test = NULL;
	u8 *tx_buf_test = NULL;

#if 1
	printk("***************rx data***************\n");
	rx_buf_test = rx_buf[0];

	for(i = 1; i <= tx_buf_len + 2; i ++) {
		if(i % 32){
			printk("%02x ", *rx_buf_test++);
		} else {
			printk("%02x\n", *rx_buf_test++);
		}
	}
	printk("\n");
#endif

	tx_buf_test = tx_buf[0];
	rx_buf_test = rx_buf[0] + 1;

	for(i = 0; i < tx_buf_len; i ++) {
		if(*tx_buf_test++ != *rx_buf_test++){
			printk("DMA transfer failed!\n");
			break;
		}
	} 

	if (i == tx_buf_len) 
		printk("DMA transfer success!\n");
	printk("\n");	
}

static void data_test3(void)
{
	u64 i = 0;
	u8 *rx_buf_test = NULL;
	u8 *tx_buf_test = NULL;

	tx_buf_test = tx_buf[0];
	rx_buf_test = rx_buf[0];

	for(i = 0; i < tx_buf_len; i ++) {
		if(*tx_buf_test++ != *rx_buf_test++){
			printk("DMA transfer failed!\n");
			break;
		}
	}

	if (i == tx_buf_len)
		printk("DMA transfer success!\n");
	printk("\n");
}

static void data_test4(void)
{
	u64 i = 0, m = 0, n = 0;
	u8 *rx_buf_test = NULL;
	u8 data;
#if 1
	printk("***************rx data***************\n");
	rx_buf_test = rx_buf[0];
	for(i = 1; i <= rx_buf_len; i ++) {
		if(i % 32){
			printk("%02x ", *rx_buf_test++);
		}else {
			printk("%02x\n", *rx_buf_test++);
		}
	}
	printk("\n");
#endif

	rx_buf_test = rx_buf[0];
	for(m = 0; m < rx_buf_len / 8; m ++) {
		data = 0x11;

		for(i = 0; i < 8; i ++) {
			if(data != *rx_buf_test++){
				printk("DMA transfer failed!\n");
				break;
			}
			n ++;
			data += 0x11;
		}
	}

	if (n == rx_buf_len) 
		printk("DMA transfer success!\n");

	printk("\n");
}

static void data_test5(void)
{
	u64 i = 0, x = 0, n = 0;
	u8 *rx_buf_test = NULL;
	u8 *tx_buf_test = NULL;

#if 1
	printk("***************rx data***************\n");
	rx_buf_test = rx_buf[0];
	for(i = 1; i <= tx_buf_len * 3; i ++) {
		if(i % 32){
			printk("%02x ", *rx_buf_test++);
		}else {
			printk("%02x\n", *rx_buf_test++);
		}
	}
	printk("\n");
#endif

	for(x = 0; x < src_nents; x++) {
		tx_buf_test = tx_buf[x];
		rx_buf_test = rx_buf[0] + x;

		for(i = 0; i < tx_buf_len; i ++) {
			if(*tx_buf_test++ != *rx_buf_test){
				printk("DMA transfer failed!\n");
				break;
			}
			rx_buf_test += 3;
			n ++;
		}
	}

	if(n == tx_buf_len * 3)
		printk("DMA transfer success!\n");
	printk("\n");
}

static void data_test6(void)
{
	u8 x = 0;
	u64 i = 0;
	u8 *rx_buf_test = NULL;
	u8 *tx_buf_test = NULL;

#if 1
	printk("***************rx data***************\n");
	for(x = 0; x < dst_nents; x++) {
		rx_buf_test = rx_buf[x];
		printk("**************rx buf[%d]**************\n", x);
		for(i = 1; i <= tx_buf_len + 2; i ++) {
			if(i % 32){
				printk("%02x ", *rx_buf_test++);
			}else {
				printk("%02x\n", *rx_buf_test++);
			}
		}
		printk("\n");
	}
#endif

	for(x = 0; x < src_nents; x ++) {

		if (x == 0) {
			tx_buf_test = tx_buf[x];
			rx_buf_test = rx_buf[x];

			for(i = 0; i < tx_buf_len; i ++) {
				if(*tx_buf_test++ != *rx_buf_test++){
					printk("DMA transfer failed!\n");
					break;
				}
			}
		} else {
			tx_buf_test = tx_buf[x];
			rx_buf_test = rx_buf[x] + 1;

			for(i = 0; i < tx_buf_len; i ++) {
				if(*tx_buf_test++ != *rx_buf_test++){
					printk("DMA transfer failed!\n");
					break;
				}
			} 

		}
	}

	if ((x == src_nents) && (i == tx_buf_len))
		printk("DMA transfer success!\n");
	printk("\n");
}

static void data_test7(void)
{
	u64 i = 0;
	u8 *rx_buf_test = NULL;
	u8 *tx_buf_test = NULL;
	u64 len = tx_buf_len + tx_buf_len / 32;

#if 1
	printk("***************rx data***************\n");
	rx_buf_test = rx_buf[0];
	for(i = 1; i <= len; i ++) {
		if(i % 33){
			printk("%02x ", *rx_buf_test++);
		}else {
			printk("%02x\n", *rx_buf_test++);
		}
	}
	printk("\n");
#endif

	tx_buf_test = tx_buf[0];
	rx_buf_test = rx_buf[0];

	for(i = 1; i <= len; i ++) {
		if (i % 33) {
			if(*tx_buf_test++ != *rx_buf_test++){
				printk("DMA transfer failed!\n");
				break;
			}
		}else
			rx_buf_test += 1;
	}

	if (i == len + 1)
		printk("DMA transfer success!\n");
	printk("\n");
}

static int dmatest_func(void *data)
{
	DECLARE_WAIT_QUEUE_HEAD_ONSTACK(done_wait);
	struct dmatest_done done = { .wait = &done_wait };
	struct dmatest_thread	*thread = data;
	struct dmatest_info	*info;
	struct dmatest_params	*params;
	struct dma_chan		*chan;
	struct dma_device	*dev;
	unsigned int		src_off, dst_off;
	unsigned int		total_tests = 0;
	dma_cookie_t		cookie;
	enum dma_status		status;
	enum dma_ctrl_flags 	flags;
	int	ret = 0;
	u64 i = 0;
	unsigned long t0 = 0, t1 = 0;
	s64	runtime = 0;
	unsigned long long	total_len = 0;
	struct sg_table *dst_sgt = NULL, *src_sgt = NULL;	//for memcpy_sg
	struct dma_async_tx_descriptor *tx = NULL;
	dma_addr_t srcs[src_nents];
	dma_addr_t dsts[dst_nents];

	ret = -ENOMEM;

	smp_rmb();

	info = thread->info;
	params = &info->params;
	chan = thread->chan;
	dev = chan->device;

	src_sgt = kzalloc(sizeof(struct sg_table), GFP_KERNEL);
	if (!src_sgt) {
		pr_warn("request tx memory failed!\n");
		return -ENOMEM;
	}

	dst_sgt = kzalloc(sizeof(struct sg_table), GFP_KERNEL);
	if (!dst_sgt) {
		pr_warn("request tx memory failed!\n");
		return -ENOMEM;
	}

	ret = sg_alloc_table(src_sgt, src_nents, GFP_KERNEL);
	if (ret != 0)
		return ret;

	ret = sg_alloc_table(dst_sgt, dst_nents, GFP_KERNEL);
	if (ret != 0)
		return ret;

	//set nice value for thread
	set_user_nice(current, 10);

	/*
	 * src and dst buffers are freed by ourselves below
	 */
	flags = DMA_CTRL_ACK | DMA_PREP_INTERRUPT;

	gpt_cdma_tlb_dis();

#if 0
	/*TLB enable*/
	gpt_cdma_l2_en();
	gpt_cdma_tlb_en();
#endif

	//When someone calls kthread_stop() on your kthread, it will be woken and return 1
	while (!kthread_should_stop()
			&& !(params->iterations && total_tests >= params->iterations)) 
	{
		total_tests++;

		src_off = 0;
		dst_off = 0;

		if(thread->type == DMA_MEMCPY)
		{
			for (i = 0; i < src_nents; i++) {
				void *buf = tx_buf[i];
				struct page *pg = virt_to_page(buf);
				unsigned pg_off = (unsigned long) buf & ~PAGE_MASK;

				srcs[i] = dma_map_page(dev->dev, pg, pg_off,
						tx_buf_len, DMA_TO_DEVICE);

				ret = dma_mapping_error(dev->dev, srcs[i]);
				if (ret) {
					printk("src mapping error\n");
				}
				srcs[i] += src_off;
				total_len += tx_buf_len;

				gpt_cmda_cache(tx_buf[i], tx_buf_len + 10);
			}

			for (i = 0; i < dst_nents; i++) {
				void *buf = rx_buf[i];
				struct page *pg = virt_to_page(buf);
				unsigned pg_off = (unsigned long) buf & ~PAGE_MASK;

				dsts[i] = dma_map_page(dev->dev, pg, pg_off, rx_buf_len,
						DMA_BIDIRECTIONAL);

				ret = dma_mapping_error(dev->dev, dsts[i]);
				if (ret) {
					printk("dst mapping error\n");
				}
				dsts[i] += dst_off;
				gpt_cmda_cache(rx_buf[i], rx_buf_len + 10);
			}
		}

		if (thread->type == DMA_SG)
		{
			int i, count;
			struct scatterlist *sg;

			for (i = 0; i < src_nents; i++) {
				sg_set_buf(&src_sgt->sgl[i], tx_buf[i], tx_buf_len);
				sg ++; 
				total_len += tx_buf_len;
				gpt_cmda_cache(tx_buf[i], tx_buf_len + 10);
			}

			count = dma_map_sg(dev->dev, src_sgt->sgl, src_nents, DMA_TO_DEVICE);
			if (!count)
				count = -ENOMEM;
			if (count < 0) {
				sg_free_table(src_sgt);
				return count;
			}
			src_sgt->nents = ret;

			for (i = 0; i < dst_nents; i++) {
				sg_set_buf(&dst_sgt->sgl[i], rx_buf[i], rx_buf_len);
				sg ++; 
				gpt_cmda_cache(rx_buf[i], rx_buf_len + 10);
			}

			count = dma_map_sg(dev->dev, dst_sgt->sgl, dst_nents, DMA_BIDIRECTIONAL);
			if (!count)
				count = -ENOMEM;
			if (count < 0) {
				sg_free_table(dst_sgt);
				return count;
			}
			dst_sgt->nents = ret;
		}

		t0 = get_timeus();

#if 0
		/*mask the high 24 bit of addr to make TLB not miss*/
		//	gpt_cdma_virt(chan, 0, (u64)tx_buf[0] & 0xffffffffff, (u64)rx_buf[0] & 0xffffffffff);

		/*
		 * test for TLB
		 * test data: 33 bytes
		 * if tlb miss and the miss address is not virt address, the TLB is questionable
		 * */
		gpt_cdma_virt(chan, 0, (u64)tx_buf[0], (u64)rx_buf[0]);
#endif

		switch (test_mode) {
			case 1:
				//TEST: 1D-to-1D; COPY; S-to-S  size <= 65535
				gpt_cdma_opsetn_mode(chan, 0, GPT_COPY, GPT_RD_SINGAL, GPT_WR_SINGAL, GPT_RB_NUM1, GPT_SIZE_1B, GPT_PAD_1B);
				gpt_cdma_group_para(chan, 0, tx_buf_len, 1, 0, tx_buf_len, 1, 0);
				break;

			case 2:
				//TEST:1D-to-1D; ZERO_PAD
				gpt_cdma_opsetn_mode(chan, 0, GPT_ZERO_PAD, GPT_RD_SINGAL, GPT_WR_SINGAL, GPT_RB_NUM1, GPT_SIZE_1B, GPT_PAD_1B);
				gpt_cdma_group_para(chan, 0, tx_buf_len, 1, 0, rx_buf_len, 1, 0);
				break;

			case 3:
				//TEST:1D-to-1D; COPY_PAD
				gpt_cdma_opsetn_mode(chan, 0, GPT_COPY_PAD, GPT_RD_SINGAL, GPT_WR_SINGAL, GPT_RB_NUM1, GPT_SIZE_1B, GPT_PAD_1B);
				gpt_cdma_group_para(chan, 0, tx_buf_len, 1, 0, tx_buf_len, 1, 0);
				break;

			case 4:
				//TEST: 1D-to-1D; COPY; S-to-S  size > 65535
				gpt_cdma_opsetn_mode(chan, 0, GPT_COPY, GPT_RD_MULTIPLE, GPT_WR_MULTIPLE, GPT_RB_NUM1, GPT_SIZE_1B, GPT_PAD_1B);
				gpt_cdma_group_para(chan, 0, 1024, tx_buf_len / 1024, 0, 1024, rx_buf_len / 1024, 0);

#if 0
				// for test: 1D data with 2D transfer
				gpt_cdma_opsetn_mode(chan, 0, GPT_COPY, GPT_RD_MULTIPLE, GPT_WR_MULTIPLE, GPT_RB_NUM1, GPT_SIZE_1B, GPT_PAD_1B);
				gpt_cdma_opsetn_mode(chan, 1, GPT_COPY, GPT_RD_MULTIPLE, GPT_WR_MULTIPLE, GPT_RB_NUM1, GPT_SIZE_1B, GPT_PAD_1B);
				gpt_cdma_group_para(chan, 0, 32, tx_buf_len / 64, 32, 32, rx_buf_len / 64, 32);
				gpt_cdma_group_para(chan, 1, 32, tx_buf_len / 64, 32, 32, rx_buf_len / 64, 32);
#endif
				break;

			case 5:
				//TEST: FILL; S-to-S
				gpt_cdma_opsetn_mode(chan, 0, GPT_FILL, GPT_RD_SINGAL, GPT_WR_SINGAL, GPT_RB_NUM1, GPT_SIZE_1B, GPT_PAD_1B);
				gpt_cdma_fill_withsetn(chan, 0, 0x8877665544332211);
				gpt_cdma_group_para(chan, 0, tx_buf_len, 1, 0, rx_buf_len, 1, 0);
				break;

			case 6:
				//TEST: 1D-to-1D; COPY; M-to-S
				gpt_cdma_opsetn_mode(chan, 0, GPT_COPY, GPT_RD_SINGAL, GPT_WR_SINGAL, GPT_RB_NUM3, GPT_SIZE_1B, GPT_PAD_1B);
				gpt_cdma_group_para(chan, 0, tx_buf_len, 1, 0, rx_buf_len, 1, 0);
				break;

			case 7:
				//TEST: 1D-to-1D; COPY; M-to-M
				gpt_cdma_opsetn_mode(chan, 0, GPT_COPY, GPT_RD_SINGAL, GPT_WR_SINGAL, GPT_RB_NUM1, GPT_SIZE_1B, GPT_PAD_1B);
				gpt_cdma_opsetn_mode(chan, 1, GPT_ZERO_PAD, GPT_RD_SINGAL, GPT_WR_SINGAL, GPT_RB_NUM1, GPT_SIZE_1B, GPT_PAD_1B);
				gpt_cdma_opsetn_mode(chan, 2, GPT_COPY_PAD, GPT_RD_SINGAL, GPT_WR_SINGAL, GPT_RB_NUM1, GPT_SIZE_1B, GPT_PAD_1B);
				gpt_cdma_group_para(chan, 0, tx_buf_len, 1, 0, rx_buf_len, 1, 0);
				gpt_cdma_group_para(chan, 1, tx_buf_len, 1, 0, rx_buf_len, 1, 0);
				gpt_cdma_group_para(chan, 2, tx_buf_len, 1, 0, rx_buf_len, 1, 0);
				break;

			case 8:
				//TEST: 1D-to-2D; COPY; S-to-S
				gpt_cdma_opsetn_mode(chan, 0, GPT_COPY, GPT_RD_MULTIPLE, GPT_WR_MULTIPLE, GPT_RB_NUM1, GPT_SIZE_1B, GPT_PAD_1B);
				gpt_cdma_group_para(chan, 0, tx_buf_len, 1, 0, 32, rx_buf_len / 32, 1);
				break;
		}

		if (thread->type == DMA_MEMCPY)
		{
			do {
				tx = dev->device_prep_dma_memcpy(chan,
						dsts[0],
						srcs[0], tx_buf_len, flags);
			} while (!tx);
		} else if (thread->type == DMA_SG){
			do {
				tx = dev->device_prep_dma_sg(chan,
						dst_sgt->sgl, dst_nents,
						src_sgt->sgl, src_nents,
						flags);
			} while (!tx);
		}

		done.done = false;
		tx->callback = dmatest_callback;
		tx->callback_param = &done;
		cookie = tx->tx_submit(tx);

		if (dma_submit_error(cookie)) {
			printk("%s :submit error\n", __func__);
			continue;
		}

		dma_async_issue_pending(chan);

		wait_event_freezable_timeout(done_wait, done.done,
				msecs_to_jiffies(params->timeout));

		status = dma_async_is_tx_complete(chan, cookie, NULL, NULL);
		if (!done.done)  
			printk("test time out\n");
		else if (status != DMA_COMPLETE) 
			printk("DMA ERR!\n");
	}

#if 0
	t1 = get_timeus();
	runtime = t1 - t0;
#endif

	dmaengine_terminate_all(chan);

	for(i = 0; i < src_nents; i ++) 
		gpt_cmda_cache(tx_buf[i], tx_buf_len + 10);

	for(i = 0; i < dst_nents; i ++) 
		gpt_cmda_cache(rx_buf[i], rx_buf_len + 10);

	switch (test_mode) {
		case 1:
			printk("******1D-to-1D; COPY; S-to-S  size <= 65535******\n");
			data_test1();
			break;
		case 2:
			printk("******1D-to-1D; ZERO_PAD; S-to-S******\n");
			data_test2();
			break;
		case 3:
			printk("******1D-to-1D; COPY_PAD; S-to-S******\n");
			data_test2();
			break;
		case 4:
			printk("******1D-to-1D; COPY; S-to-S  size > 65535******\n");
			data_test3();
			break;
		case 5:
			printk("******FILL: 0x8877665544332211******\n");
			data_test4();
			break;
		case 6:
			printk("******1D-to-1D; COPY; M-to-S******\n");
			data_test5();
			break;
		case 7:
			printk("******1D-to-1D; COPY; M-to-M******\n"
					"  1-COPY  2-ZERO_PAD  3-COPY_PAD\n");
			data_test6();
			break;
		case 8:
			printk("******1D-to-2D; COPY; S-to-S******\n");
			data_test7();
			break;
	}

	if(thread->type == DMA_MEMCPY) {
		for (i = 0; i < src_nents; i ++)
			dma_unmap_page(dev->dev, srcs[i], tx_buf_len, DMA_TO_DEVICE);
		for (i = 0; i < dst_nents; i ++)
			dma_unmap_page(dev->dev, dsts[i], tx_buf_len, DMA_BIDIRECTIONAL);
	}

	if(thread->type == DMA_SG) {
		dma_unmap_sg(dev->dev, src_sgt->sgl, src_nents, DMA_TO_DEVICE);
		kfree(src_sgt);

		dma_unmap_sg(dev->dev, dst_sgt->sgl, dst_nents, DMA_BIDIRECTIONAL);
		kfree(dst_sgt);
	}

	dma_release_channel(chan);

	thread->done = true;
	wake_up(&thread_wait);

	return 0;
}

static void dmatest_cleanup_channel(struct dmatest_chan *dtc)
{
	struct dmatest_thread	*thread;
	struct dmatest_thread	*_thread;
	int	ret;

	list_for_each_entry_safe(thread, _thread, &dtc->threads, node) {
		ret = kthread_stop(thread->task);
		pr_debug("thread %s exited with status %d\n",
				thread->task->comm, ret);
		list_del(&thread->node);
		put_task_struct(thread->task);
		kfree(thread);
	}

	/* terminate all transfers on specified channels */
	//	dmaengine_terminate_all(dtc->chan);

	kfree(dtc);
}

static int dmatest_add_threads(struct dmatest_info *info,
		struct dmatest_chan *dtc, enum dma_transaction_type type)
{
	struct dmatest_params *params = &info->params;
	struct dmatest_thread *thread;
	struct dma_chan *chan = dtc->chan;
	char *op;
	unsigned int i;

	if (type == DMA_MEMCPY)
		op = "copy";
	else if (type == DMA_SG)
		op = "sg";
	else
		return -EINVAL;

	for (i = 0; i < params->threads_per_chan; i++) {
		thread = kzalloc(sizeof(struct dmatest_thread), GFP_KERNEL);
		if (!thread) {
			pr_warn("No memory for %s-%s%u\n",
					dma_chan_name(chan), op, i);
			break;
		}
		thread->info = info;
		thread->chan = dtc->chan;
		thread->type = type;
		smp_wmb();

		thread->task = kthread_create(dmatest_func, thread, "%s-%s%u",
				dma_chan_name(chan), op, i);
		if (IS_ERR(thread->task)) {
			pr_warn("Failed to create thread %s-%s%u\n",
					dma_chan_name(chan), op, i);
			kfree(thread);
			break;
		}

		/* srcbuf and dstbuf are allocated by the thread itself */
		get_task_struct(thread->task);
		list_add_tail(&thread->node, &dtc->threads);
		wake_up_process(thread->task);
	}

	return i;
}

static int dmatest_add_channel(struct dmatest_info *info,
		struct dma_chan *chan)
{
	struct dmatest_chan	*dtc;
	struct dma_device	*dma_dev = chan->device;
	unsigned int		thread_count = 0;
	int cnt;

	/*alloc mem for struct dmatest_chan*/
	dtc = kmalloc(sizeof(struct dmatest_chan), GFP_KERNEL);
	if (!dtc) {
		pr_warn("No memory for %s\n", dma_chan_name(chan));
		return -ENOMEM;
	}

	dtc->chan = chan;
	INIT_LIST_HEAD(&dtc->threads);

	if (memcpy_type == 0){
		if (dma_has_cap(DMA_MEMCPY, dma_dev->cap_mask)) {
			cnt = dmatest_add_threads(info, dtc, DMA_MEMCPY);
			thread_count += cnt > 0 ? cnt : 0;
		}
	} else if (memcpy_type == 1) {
		if (dma_has_cap(DMA_SG, dma_dev->cap_mask)) {
			cnt = dmatest_add_threads(info, dtc, DMA_SG);
			thread_count += cnt > 0 ? cnt : 0;
		}
	}

#if 1
	pr_info("Started %u threads using %s\n",
			thread_count, dma_chan_name(chan));
#endif

	list_add_tail(&dtc->node, &info->channels);
	info->nr_channels++;

	return 0;
}

static void request_channels(struct dmatest_info *info,
		enum dma_transaction_type type)
{
	dma_cap_mask_t mask;
	struct dmatest_params *params = &info->params;
	struct dma_chan *chan;

	dma_cap_zero(mask);
	dma_cap_set(type, mask);

	/*request a channel: struct dma_chan*/
	chan = dma_request_channel(mask, 0, NULL);
	if (chan) {
		if (dmatest_add_channel(info, chan)) {
			dma_release_channel(chan);
			printk("dma add chanel failed!\n");
		}
	}
}

static int dmatest_request_buf(void)
{
	int i = 0;

	for(i = 0; i < 4; i ++)	{
		tx_buf[i] = kzalloc(tx_buf_len + 20, GFP_KERNEL);
		if (!tx_buf[i]) {
			pr_warn("request tx memory failed!\n");
			return -ENOMEM;
		}
	}

	for(i = 0; i < 4; i ++)	{
		rx_buf[i] = kzalloc(rx_buf_len + 20, GFP_KERNEL);
		if (!rx_buf[i]) {
			pr_warn("request rx memory failed!\n");
			return -ENOMEM;
		}
	}

	return 0;
}

static void run_threaded_test(struct dmatest_info *info)
{
	struct dmatest_params *params = &info->params;
	u64 i = 0;
	u8 *buf0, *buf1, *buf2, *buf3;

	/* Copy test parameters */
	strlcpy(params->channel, strim(test_channel), sizeof(params->channel));
	strlcpy(params->device, strim(test_device), sizeof(params->device));
	params->threads_per_chan = 1;
	params->iterations = 1;
	params->timeout = timeout;

	dmatest_request_buf();

	buf0 = tx_buf[0]; 
	buf1 = tx_buf[1]; 
	buf2 = tx_buf[2]; 
	buf3 = tx_buf[3];

	for(i = 0; i < tx_buf_len; i ++) {
		*buf0 ++ = 0x11 + i;
		*buf1 ++ = 0x12 + i;
		*buf2 ++ = 0x13 + i;
		*buf3 ++ = 0x14 + i;
	}

#if 1
	if ((test_mode != 4) && (test_mode != 5)) {
		buf0 = tx_buf[0]; 
		printk("*************tx raw data**************\n");
		for(i = 1; i <= tx_buf_len; i ++) {
			if(i % 32){
				printk("%02x ", *buf0 ++);
			}else {
				printk("%02x\n", *buf0 ++);
			}
		}
		printk("\n");
	}
#endif

	if (memcpy_type == 0)
		request_channels(info, DMA_MEMCPY);
	else if (memcpy_type == 1)
		request_channels(info, DMA_SG);
}

static void stop_threaded_test(struct dmatest_info *info)
{
	struct dmatest_chan *dtc, *_dtc;
	struct dma_chan *chan;

	list_for_each_entry_safe(dtc, _dtc, &info->channels, node) {
		list_del(&dtc->node);
		chan = dtc->chan;
		dmatest_cleanup_channel(dtc);
		pr_debug("dropped channel %s\n", dma_chan_name(chan));
		//		dma_release_channel(chan);
	}

	info->nr_channels = 0;
}

static void restart_threaded_test(struct dmatest_info *info, bool run)
{
	int i = 0;
	/* we might be called early to set run=, defer running until all
	 * parameters have been evaluated
	 */
	if (!info->did_init)
		return;

	/* Stop any running test first */
	stop_threaded_test(info);

	for(i = 0; i < 4; i ++)	{
		kfree(tx_buf[i]);
		kfree(rx_buf[i]);
	}

	/* Run test with new parameters */
	run_threaded_test(info);
}

static int dmatest_run_get(char *val, const struct kernel_param *kp)
{
	struct dmatest_info *info = &test_info;

	mutex_lock(&info->lock);
	if (is_threaded_test_run(info)) {
		dmatest_run = true;
	} else {
		stop_threaded_test(info);
		dmatest_run = false;
	}
	mutex_unlock(&info->lock);

	return param_get_bool(val, kp);
}

static int dmatest_run_set(const char *val, const struct kernel_param *kp)
{
	struct dmatest_info *info = &test_info;
	int ret;

	mutex_lock(&info->lock);
	ret = param_set_bool(val, kp);
	if (ret) {
		mutex_unlock(&info->lock);
		return ret;
	}

	if (is_threaded_test_run(info))
		ret = -EBUSY;
	else if (dmatest_run)
		restart_threaded_test(info, dmatest_run);

	mutex_unlock(&info->lock);

	return ret;
}

static int __init dmatest_init(void)
{
	struct dmatest_info *info = &test_info;
	struct dmatest_params *params = &info->params;

	if (dmatest_run) {
		mutex_lock(&info->lock);
		run_threaded_test(info);
		mutex_unlock(&info->lock);
	}

	if (params->iterations && wait)
		wait_event(thread_wait, !is_threaded_test_run(info));

	/* module parameters are stable, inittime tests are started,
	 * let userspace take over 'run' control
	 */
	info->did_init = true;

	return 0;
}
/* when compiled-in wait for drivers to load first */
late_initcall(dmatest_init);

static void __exit dmatest_exit(void)
{
	struct dmatest_info *info = &test_info;

	mutex_lock(&info->lock);
	stop_threaded_test(info);
	mutex_unlock(&info->lock);
}
module_exit(dmatest_exit);

MODULE_AUTHOR("GPT");
MODULE_LICENSE("GPL v2");
