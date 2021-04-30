/*
 * tegra_pcm.c - Tegra PCM driver
 *
 * Author: Stephen Warren <swarren@nvidia.com>
 * Copyright (C) 2010,2012 - NVIDIA, Inc.
 *
 * Based on code copyright/by:
 *
 * Copyright (c) 2009-2010, NVIDIA Corporation.
 * Scott Peterson <speterson@nvidia.com>
 * Vijay Mali <vmali@nvidia.com>
 *
 * Copyright (C) 2010 Google, Inc.
 * Iliyan Malchev <malchev@google.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/atmel_pdc.h>
#include <linux/atmel-ssc.h>
#include <linux/kthread.h>
#include <linux/sched.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <asm/mach-chip2/apiu.h>

#include "gpt-pcm.h"
#include "gpt_i2s.h"


#define   sw2Bytes(val) (((val)>>8) & 0x00ff) | (((val)<<8)&0xff00)

#ifdef	CONFIG_APIU
#define i2s_readl(addr)		apiu_readl(addr)
#define i2s_writel(val,addr)	apiu_writel(val,addr)
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

void __iomem *i2s_regs;

/*--------------------------------------------------------------------------*\
 * Hardware definition
\*--------------------------------------------------------------------------*/
/* TODO: These values were taken from the AT91 platform driver, check
 *	 them against real values for AT32
 */
static const struct snd_pcm_hardware atmel_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_MMAP_VALID |
				  SNDRV_PCM_INFO_INTERLEAVED |
				  SNDRV_PCM_INFO_PAUSE,
	.period_bytes_min	= 32,
	.period_bytes_max	= 8192,
	.periods_min		= 2,
	.periods_max		= 1024,
	.buffer_bytes_max	= ATMEL_SSC_DMABUF_SIZE,
};

/*--------------------------------------------------------------------------*\
 * Data types
\*--------------------------------------------------------------------------*/
struct gpt_runtime_data {
	struct gpt_pcm_dma_params *params;
	dma_addr_t dma_buffer;		/* physical address of dma buffer */
	dma_addr_t dma_buffer_end;	/* first address beyond DMA buffer */
	dma_addr_t dma_every_end;
	size_t period_size;

	dma_addr_t period_ptr;		/* physical address of next period */
};


void i2s_burst_writel(u32 val, void __iomem *addr,unsigned int lr)
{
	u32 status;

	if (lr ==0) {
		do {
			status = i2s_readl(i2s_regs + APIU_UNIT_USFR);
		} while ((status & I2S_RAW_TXR_FULL) != 0); // wait for not full
		i2s_writel(val, addr);
	}else if (lr == 1) {
		do {
			status = i2s_readl(i2s_regs + APIU_UNIT_USFR);
		} while ((status & I2S_RAW_TXL_FULL) != 0); // wait for not full
		i2s_writel(val, addr);
	}else{
		;
	}
}

static int gpt_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct gpt_runtime_data *prtd;
	int ret = 0;
	SP_PRINTK("file:%s, func: %s,line=%d \n",__FILE__,__func__,__LINE__);
	snd_soc_set_runtime_hwparams(substream, &atmel_pcm_hardware);

	/* ensure that buffer size is a multiple of period size */
	ret = snd_pcm_hw_constraint_integer(runtime,
						SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		goto out;

	prtd = kzalloc(sizeof(struct gpt_runtime_data), GFP_KERNEL);
	if (prtd == NULL) {
		ret = -ENOMEM;
		goto out;
	}
	runtime->private_data = prtd;

 out:
	return ret;
}

static int gpt_pcm_close(struct snd_pcm_substream *substream)
{
	struct gpt_runtime_data *prtd = substream->runtime->private_data;

	SP_PRINTK("file:%s, func: %s,line=%d \n",__FILE__,__func__,__LINE__);
	kfree(prtd);
	return 0;
}

/*--------------------------------------------------------------------------*\
 * PCM operations
\*--------------------------------------------------------------------------*/
static int gpt_pcm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct gpt_runtime_data *prtd = runtime->private_data;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;

	/* this may get called several times by oss emulation
	 * with different params */
	// update substream->runtime->dma_buffer  aread size and so on
	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	runtime->dma_bytes = params_buffer_bytes(params);

	prtd->params = snd_soc_dai_get_dma_data(rtd->cpu_dai, substream);

	prtd->dma_buffer = runtime->dma_addr; //physical
	prtd->dma_buffer_end = runtime->dma_addr + runtime->dma_bytes;
	prtd->period_size = params_period_bytes(params);

	SP_PRINTK("file:%s, func: %s,line=%d,prtd->period_size = %x \n",__FILE__,__func__,__LINE__,prtd->period_size);
	SP_PRINTK("		prtd->dma_buffer_end = %x \n",prtd->dma_buffer_end);

	pr_debug("atmel-pcm: "
		"hw_params: DMA for %s initialized "
		"(dma_bytes=%zu, period_size=%zu)\n",
		prtd->params->name,
		runtime->dma_bytes,
		prtd->period_size);
	return 0;
}

static int gpt_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct gpt_runtime_data *prtd = substream->runtime->private_data;
	struct gpt_pcm_dma_params *params = prtd->params;

	SP_PRINTK("file:%s, func: %s,line=%d \n",__FILE__,__func__,__LINE__);
	if (params != NULL) {
		prtd->params->dma_intr_handler = NULL;
	}

	return 0;
}

static void  i2s_init(void){

	unsigned int value;
	i2s_writel(0, i2s_regs + APIU_UNIT_UCR);
	udelay(1);
	SP_PRINTK("file:%s, func: %s,line=%d\n",__FILE__,__func__,__LINE__);
	value = I2S_CHSIZE_16 | I2S_FRMSIZE_64 |(1 << I2S_MASTER_sft) | I2S_UMASK_STD | I2S_UMASK_BST;

	SP_PRINTK("	value =%x\n",value);
	i2s_writel(value, i2s_regs + APIU_UNIT_UCR);
	udelay(2);
	i2s_writel(I2S_EN | value, i2s_regs + APIU_UNIT_UCR);
	udelay(2);
}
static int gpt_pcm_prepare(struct snd_pcm_substream *substream)
{
//	struct gpt_runtime_data *prtd = substream->runtime->private_data;
	//struct gpt_pcm_dma_params *params = prtd->params;

	i2s_init();
	return 0;
}
/*
static struct task_struct*test_task;

int threadfunc(void*data){
	int i =0;
	unsigned int*tmp =data;
	SP_PRINTK("	========data = %x\n", *tmp);

	while(1){
		set_current_state(TASK_INTERRUPTIBLE);
		if(kthread_should_stop()) break;
		if(data > 0){
			SP_PRINTK("	=======i>0\n");
		}
		else{
			SP_PRINTK("condition is  not satisfied\n");
			//msleep(100);
			//schedule_timeout(HZ);
		}

	}

	return 0;
}
*/

static int gpt_pcm_trigger(struct snd_pcm_substream *substream,
	int cmd)
{
	struct snd_pcm_runtime *rtd = substream->runtime;
	struct gpt_runtime_data *prtd = rtd->private_data;
	//struct gpt_pcm_dma_params *params = prtd->params;
	dma_addr_t every_end;
	unsigned short *tmp = (unsigned short*) rtd->dma_area; /// virtural
	int ret = 0;
	int i =0 ;
/*
	int err;
	unsigned int data = 0x09;

	test_task = kthread_create(threadfunc, &data, "test_task");
	if(IS_ERR(test_task)){

		SP_PRINTK("	unable to start kernel thread\n");
		err =PTR_ERR(test_task);
		test_task = NULL;
		return err;
	}
	wake_up_process(test_task);
*/
	switch(cmd){
		case SNDRV_PCM_TRIGGER_START:
			SP_PRINTK("file:%s, func: %s,line=%d \n",__FILE__,__func__,__LINE__);
			SP_PRINTK("	rtd->dma_area = %p, dma_bytes = %d\n", rtd->dma_area, rtd->dma_bytes);
			SP_PRINTK("	rtd->buffer_size = %x\n", rtd->buffer_size);//0x4000
			SP_PRINTK("	prtd->period_size = %x\n", prtd->period_size);//0x400
			SP_PRINTK("	prtd->period_ptr = %p\n", prtd->period_ptr);//0x0

			prtd->period_size = 0x40;
			if(prtd->period_ptr == 0x0){
				prtd->period_ptr = prtd->dma_buffer; //physical
				every_end = (dma_addr_t)rtd->dma_area;///
			}
			else{
				tmp = (unsigned short*)prtd->dma_every_end; ///
				every_end = prtd->dma_every_end;
			}
			SP_PRINTK("	prtd->dma_every_end = %p\n", prtd->dma_every_end);
			SP_PRINTK("	every_end = %p\n", every_end);
			SP_PRINTK("	prtd->period_ptr = %p\n", prtd->period_ptr); //physical

			for(i = 0; i < prtd->period_size; i++){
					if( i%4 ==0){
						SP_PRINTK("\n");
					}
					SP_PRINTK("%x", *tmp);
					i2s_burst_writel(*tmp++, i2s_regs + I2S_TXL, 0);
					SP_PRINTK("%x ", *tmp);
					i2s_burst_writel(*tmp++, i2s_regs + I2S_TXR, 1);
			}

					SP_PRINTK("\n");
			prtd->period_ptr += prtd->period_size*4;
			prtd->dma_every_end = every_end + prtd->period_size *4;

			if(prtd->period_ptr>=prtd->dma_buffer_end){
				prtd->period_ptr = prtd->dma_buffer;
				prtd->dma_every_end = (dma_addr_t)rtd->dma_area; ///
			}

			SP_PRINTK("	prtd->period_ptr = %llx\n", prtd->period_ptr);
			SP_PRINTK("	prtd->dma_every_end = %llx\n", prtd->dma_every_end);

			snd_pcm_period_elapsed(substream);

			break;
		case SNDRV_PCM_TRIGGER_STOP:
		case SNDRV_PCM_TRIGGER_SUSPEND:
		case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
			break;

		case SNDRV_PCM_TRIGGER_RESUME:
		case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
			break;

		default:
			ret = -EINVAL;
	}

	SP_PRINTK("file:%s, func: %s,line=%d \n",__FILE__,__func__,__LINE__);
	return ret;
}

static snd_pcm_uframes_t gpt_pcm_pointer(
	struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct gpt_runtime_data *prtd = runtime->private_data;
	//struct gpt_pcm_dma_params *params = prtd->params;
	dma_addr_t ptr;
	snd_pcm_uframes_t x;

	SP_PRINTK("file:%s, func: %s,line=%d \n",__FILE__,__func__,__LINE__);
	ptr = prtd->period_ptr;
	SP_PRINTK("	prtd->period_ptr =%p\n",prtd->period_ptr);
	x = bytes_to_frames(runtime, ptr - prtd->dma_buffer);
	if (x == runtime->buffer_size)
		x = 0;
	SP_PRINTK("	xxxxxxxxxxxxxxxx =%x\n",x);
       return x;
}

int gpt_pcm_mmap(struct snd_pcm_substream *substream,
	struct vm_area_struct *vma)
{
	return remap_pfn_range(vma, vma->vm_start,
		       substream->dma_buffer.addr >> PAGE_SHIFT,
		       vma->vm_end - vma->vm_start, vma->vm_page_prot);
}
EXPORT_SYMBOL_GPL(atmel_pcm_mmap);


static struct snd_pcm_ops gpt_pcm_ops = {
	.open		= gpt_pcm_open,
	.close		= gpt_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= gpt_pcm_hw_params,
	.hw_free	= gpt_pcm_hw_free,
	.prepare	= gpt_pcm_prepare,
	.trigger	= gpt_pcm_trigger,
	.pointer	= gpt_pcm_pointer,
	.mmap		= gpt_pcm_mmap,
};


static struct snd_soc_platform_driver gpt_soc_platform = {
	.ops		= &gpt_pcm_ops,
	.pcm_new	= atmel_pcm_new,
	.pcm_free	= atmel_pcm_free,
};

int gpt_pcm_platform_register(struct device *dev)
{
	struct gpt_i2s *i2s = dev_get_drvdata(dev);
	i2s_regs = i2s->base;
	return snd_soc_register_platform(dev, &gpt_soc_platform);
}
EXPORT_SYMBOL_GPL(gpt_pcm_platform_register);

void gpt_pcm_platform_unregister(struct device *dev)
{
	return snd_soc_unregister_platform(dev);
}
EXPORT_SYMBOL_GPL(gpt_pcm_platform_unregister);

MODULE_AUTHOR("Stephen Warren <swarren@nvidia.com>");
MODULE_DESCRIPTION("Tegra PCM ASoC driver");
MODULE_LICENSE("GPL");
