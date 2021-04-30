#include <linux/of.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/of_irq.h>
#include <linux/kthread.h>
#include <linux/cache.h>
#include <linux/sched.h>
#include <asm/cacheflush.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/of_device.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/completion.h>
#include <asm/atomic.h>

#include "gpt-proto.h"
#include "gpt-gsnn.h"

#define NUM_OF_DEVICES	2

static struct class *gpt_gsnn_class = NULL;
//spinlock_t gpt_gsnn_lock;

static struct gpt_gsnn *gsnn_dev[NUM_OF_DEVICES];
static dev_t devno_gsnn[2] =  {MKDEV(GPT_GSNN_MAJOR, GPT_GSNN0_MINOR), \
	MKDEV(GPT_GSNN_MAJOR, GPT_GSNN1_MINOR)};
static atomic_t gsnn_atomic_available[NUM_OF_DEVICES] = {ATOMIC_INIT(1), ATOMIC_INIT(1)};

int debug_level = 1;
module_param(debug_level, uint, 0644);
MODULE_PARM_DESC(debug_level, "enable 1, disable 0, default disable");

static int timeout = 5000;
module_param(timeout , uint, 0644);
MODULE_PARM_DESC(timeout , "xx msec\n");

//#define GPT_GSNN_DEBUG
#ifdef GPT_GSNN_DEBUG
void gpt_generate_rand(unsigned char *buff, int length)
{
        int i;
        unsigned char rd = jiffies & 0xff;
        PDEBUG("random data is %x\n", rd);

        for (i = 0; i < length; i++)
                buff[i] = rd++;
}

static void gpt_gsnn_dumpdata(void *addr, int length)
{
        int i;
        unsigned int *buff = addr;
        int count = length / 4;
        count += ((length % 4 > 0) ? 1 : 0);

        for (i = 0; i < (length / 4); i++) {
                if (i % 4 == 0)
                        PDEBUG("%.8x: ", i * 4);
                PDEBUG("%.8x%c", buff[i], ((i % 4 == 3) ? '\n' : ' '));
        }
        PDEBUG("\n");
}

static int gpt_gsnn_datacmp(unsigned char *src, unsigned char *dst, int length)
{
	int i;

	for (i = 0; i < length; i++) {
		if (src[i] != dst[i]) {
			PDEBUG("#####offset 0x%x, 0x%x--> 0x%x\n", i, src[i], dst[i]);
			return -1;
		}
	}

	return 0;
}
#endif

static int gpt_extsrc_cfg(void)
{
	uint32_t *extsrc = ioremap_nocache(0xf0000100, 0x40);

	if (extsrc == NULL) {
		PERROR("%s:%d--> get memory/io resource failed\n",
					__func__, __LINE__);
		return -ENXIO;
	}

	writel(readl(extsrc) | (0x10f | (0x10f << 16)), extsrc);
	//PDEBUG("GPT EXTSRC address(%x) remap to %p, 0x%x\n", 0xf0000100, extsrc, readl(extsrc));
	iounmap(extsrc);

	return 0;
}

static int gsnn_device_config(void)
{
	uint32_t reg_value;
	uint32_t *extsrc;
	uint32_t *extsrc_high;
	uint32_t *ext_apb_ccr;

	extsrc = ioremap_nocache(0xf0000100, 0x40);
	if (extsrc == NULL) {
		PERROR("%s:%d--> get memory/io resource failed\n",
					__func__, __LINE__);
		return -ENXIO;
	}	
	reg_value = readl(extsrc);
	//printk("before write, extsrc=0x%x\n", reg_value);	
	writel(0xf0005, extsrc);  
	//PDEBUG("GPT EXTSRC address(%x) remap to %p, 0x%x\n", 0xf0000100, extsrc, readl(extsrc));
	iounmap(extsrc);		

	extsrc_high= ioremap_nocache(0xf0000104, 0x40);
	if (extsrc_high == NULL) {
		PERROR("%s:%d--> get memory/io resource failed\n",
					__func__, __LINE__);
		return -ENXIO;
	}			
	writel(0xf00000, extsrc_high);
	//printk("GPT EXTSRC_HIGH address(%x) remap to %p, 0x%x\n", 0xf0000104, extsrc_high, readl(extsrc_high));
	iounmap(extsrc_high);		

	ext_apb_ccr= ioremap_nocache(0xF0000010, 0x40);
	if (ext_apb_ccr == NULL) {
		PERROR("%s:%d--> get memory/io resource failed\n",
					__func__, __LINE__);
		return -ENXIO;
	}
	//printk("before write ,ext_apb_ccr=0x%x\n ", readl(ext_apb_ccr));
	reg_value = readl(ext_apb_ccr);
	reg_value &= (~(0xf << 8));
	//printk("reg_value=0x%x\n", reg_value);
	writel(reg_value, ext_apb_ccr);
	//printk("GPT EXT_APB_CCR address(%x) remap to %p, 0x%x\n", 0xF0000010, ext_apb_ccr, readl(ext_apb_ccr));
	iounmap(ext_apb_ccr);			
}


static int gpt_pxi_regmap(struct gpt_gsnn *gsnn)
{
	unsigned long tag = gsnn->hwregs + GPT_PXI_SPR_EN;
	unsigned long mask = GPT_PXI_MASK;

	//printk("addr: %lx, mask:%lx\n", tag, mask);
	asm volatile (
		"sprsetr %0, %2\n\t"
		"sprsetr %1, %3\n\t"
		:
		: "r"(tag), "r"(mask), "i"(PXITAG), "i"(PXIMASK)
		:
	);

        return 0;
}

static void gpt_pxi_get_spr(void)
{
        uint64_t address = 0;
        uint64_t mask = 0;

        __asm__ volatile (
                "rsetspr   %0,%1\n\t"
                : "=r" (address)
                : "i"(PXITAG)
                );

        __asm__ volatile (
                "rsetspr   %0,%1\n\t"
                : "=r" (mask)
                : "i"(PXIMASK)
        );
	//printk("PXITAG:%llx, PXIMSK:%llx\n", address, mask);

        __asm__ volatile (
                "rsetspr   %0,%1\n\t"
                : "=r" (address)
                : "i"(XPEN)
                );

        __asm__ volatile (
                "rsetspr   %0,%1\n\t"
                : "=r" (mask)
                : "i"(XEN)
        );
	//printk("XPEN:%llx, XEN:%llx\n", address, mask);
}


static int gpt_gsnn_open(struct inode *inode, struct file *filp)
{
	struct gpt_gsnn *drvdata;
	
	if (MINOR(inode->i_cdev->dev) == GPT_GSNN0_MINOR)
	{
		//printk("@@@gpt_gsnn_open: GPT_GSNN0\n");
		if (!atomic_dec_and_test(&gsnn_atomic_available[0]))
		{
			atomic_inc(&gsnn_atomic_available[0]);
			return -EBUSY;
		}
	}
	else if (MINOR(inode->i_cdev->dev) == GPT_GSNN1_MINOR)
	{
		//printk("@@@gpt_gsnn_open: GPT_GSNN1\n");
		if (!atomic_dec_and_test(&gsnn_atomic_available[1]))
		{
			atomic_inc(&gsnn_atomic_available[1]);
			return -EBUSY;
		}
	}

	drvdata = container_of(inode->i_cdev, struct gpt_gsnn, gsnn_cdev);

	filp->private_data = drvdata;
	
	return 0;
}

static int gpt_gsnn_release(struct inode *inode, struct file *filp)
{
	if (MINOR(inode->i_cdev->dev) == GPT_GSNN0_MINOR)
	{		
		//printk("---gpt_gsnn_release: GPT_GSNN0\n");
		atomic_inc(&gsnn_atomic_available[0]);
	}
	else if (MINOR(inode->i_cdev->dev) == GPT_GSNN1_MINOR)
	{		
		//printk("---gpt_gsnn_release: GPT_GSNN1\n");
		atomic_inc(&gsnn_atomic_available[1]);
	}
	filp->private_data = NULL;
	return 0;
}

static set_bitn(int *num, int n)
{
        *num |= (1 << n);
}

static clear_bitn(int *num, int n)
{
        *num &= ~(1 << n);
}

static long gpt_gsnn_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int i = 0;
	int ret = 0;
	int status = 0;
	struct gsnn_header *gsnnhd;
	struct gpt_gsnn_param param;
	unsigned int *cmdbuf = NULL;
	struct gpt_gsnn *gsnn = filp->private_data;
	struct gnx_table *gnxtb = (struct gnx_table *)arg;
	struct gpt_gsnn_reg *reg;
	struct gpt_gsnn_mem *mem;
	uint32_t reg_value;
	uint32_t *extsrc;
	uint32_t *extsrc_high;
	uint32_t *core1_ienh;
	uint32_t *core1_mirqh;
	uint32_t *core3_ienh;
	uint32_t *core3_mirqh;
	uint32_t *ext_apb_ccr;
	uint32_t *mem_addr;
	

	mutex_lock(&gsnn->lock);
	switch (cmd) {
	case  IOCTL_GSNN_REG_R:
		reg = (struct gpt_gsnn_reg *)arg;
		reg_value = gsnn_read(gsnn, reg->reg_addr);
		reg->value = reg_value;
		status = copy_to_user((void *)arg, (void *)reg, sizeof(struct gpt_gsnn_reg)) ? -EFAULT : 0;
		break;

	case  IOCTL_GSNN_REG_W:
		reg = (struct gpt_gsnn_reg *)arg;
		gsnn_write(gsnn, reg->reg_addr, reg->value);				
		break;
	case IOCTL_GSNN_REG_EN:
		//PDEBUG("**IOCTL_GSNN_RES_EN irq: 0x%x\n", gsnn_read(gsnn, GSNN_REG_INTCTL));

		gsnn_write(gsnn, GSNN_REG_INTCTL, 0xf1);
		ret = wait_for_completion_timeout(&gsnn->irq_detect, HZ*60);  //msecs_to_jiffies(timeout)
		if (ret == 0) {
			PERROR("gsnn: wait timeout\n");
			status = -1;
		}

		/*PDEBUG("**pc0(0x%x): 0x%x, pc1(0x%x): 0x%x, pc2(0x%x): 0x%x, \
				pc3(0x%x): 0x%x, error(0x%x), irq(0x%x)\n",
				GSNN_REG_PC(0), gsnn_read(gsnn, GSNN_REG_PC(0)),
				GSNN_REG_PC(1), gsnn_read(gsnn, GSNN_REG_PC(1)),
				GSNN_REG_PC(2), gsnn_read(gsnn, GSNN_REG_PC(2)),
				GSNN_REG_PC(3), gsnn_read(gsnn, GSNN_REG_PC(3)),
				gsnn_read(gsnn, GSNN_REG_ERRST),
				gsnn_read(gsnn, GSNN_REG_INTCTL));*/

		break;
	case  IOCTL_GSNN_S_REGMAP:
		//PDEBUG("IOCTL_GSNN_REGMAP\n");
		gpt_pxi_regmap(gsnn);
		break;
	case  IOCTL_GSNN_G_REGMAP:
		//PDEBUG("IOCTL_GSNN_G_REGMAP\n");
		gpt_pxi_get_spr();
		break;
	case IOCTL_GSNN_SET_EXTSRC_CFG:
		gsnn_device_config();
#if 0
		core1_ienh= ioremap_nocache(0x00F0007204, 0x40);
		if (core1_ienh == NULL) {
			printk("%s:%d--> get memory/io resource failed\n",
						__func__, __LINE__);
			return -ENXIO;
		}			
		printk("before modify, core1_ienh= 0x%x\n", readl(core1_ienh));
		//writel(readl(core1_ienh) | 0xC0000000, core1_ienh);
		writel(readl(core1_ienh) | (0x1 << 30), core1_ienh);
		printk("GPT core1_ienh address(%x) remap to %p, 0x%x\n", 0x00F0007204, core1_ienh, readl(core1_ienh));
		iounmap(core1_ienh);	
		
		
		core1_mirqh= ioremap_nocache(0x00F0007274, 0x40);
		if (core1_mirqh == NULL) {
			printk("%s:%d--> get memory/io resource failed\n",
						__func__, __LINE__);
			return -ENXIO;
		}			
		printk("before modify, core1_mirqh= 0x%x\n", readl(core1_mirqh));
		//writel(readl(core1_mirqh) | 0xC0000000, core1_mirqh);
		writel(readl(core1_mirqh) | (0x1 << 30), core1_mirqh);
		printk("GPT core1_mirqh address(%x) remap to %p, 0x%x\n", 0x00F0007274, core1_mirqh, readl(core1_mirqh));
		iounmap(core1_mirqh);

		core3_ienh= ioremap_nocache(0x00F0007214, 0x40);
		if (core3_ienh == NULL) {
			printk("%s:%d--> get memory/io resource failed\n",
						__func__, __LINE__);
			return -ENXIO;
		}			
		printk("before modify, core3_ienh= 0x%x\n", readl(core3_ienh));
		writel(readl(core3_ienh) | (0x3 << 30), core3_ienh);
		printk("GPT core3_ienh address(%x) remap to %p, 0x%x\n", 0x00F0007214, core3_ienh, readl(core3_ienh));
		iounmap(core3_ienh);	


		core3_mirqh= ioremap_nocache(0x00F0007284, 0x40);
		if (core3_mirqh == NULL) {
			printk("%s:%d--> get memory/io resource failed\n",
						__func__, __LINE__);
			return -ENXIO;
		}			
		printk("before modify, core3_mirqh= 0x%x\n", readl(core3_mirqh));
		writel(readl(core3_mirqh) | (0x3 << 30), core3_mirqh);
		printk("GPT core3_mirqh address(%x) remap to %p, 0x%x\n", 0x00F0007284, core3_mirqh, readl(core3_mirqh));
		iounmap(core3_mirqh);

#endif		
		break;

	case IOCTL_GSNN_GETPARAM:
		param.hwdata_base = gsnn->hwdata_base;
		param.hwdata_size = gsnn->hwdata_size;
		status = copy_to_user((void *)arg, (void *)&param, sizeof(param)) ? -EFAULT : 0;
		break;
#if 0		
	case  IOCTL_GSNN_MEM_DUMP:
		 mem = (struct gpt_gsnn_mem *)arg;
		
		//PDEBUG("\nIOCTL_GSNN_REG_W, reg_addr=0x%x, value=0x%x\n", reg->reg_addr, reg->value);
		printk("----phys_add: 0x%p\n", mem->phys_addr);
		for (i = 0; i < mem->len ; i++)
		{	
			mem_addr = ioremap_nocache((mem->phys_addr + i ), 0x40);
			if (mem_addr == NULL) {
				printk("%s:%d--> get memory/io resource failed\n",
							__func__, __LINE__);
				return -ENXIO;
			}	
			
			if (i % 16 == 0 ) printk("\n");
			printk("%2x ", (unsigned char)*mem_addr); 
			iounmap(mem_addr);
		}
		printk("\n");
				
		break;		
		
	case  IOCTL_GSNN_REGREAD:
		PDEBUG("IOCTL_GSNN_REGREAD addr: 0x%x, size: 0x%x\n",
					gnxtb->s_addr, gnxtb->s_size);

		if (gnxtb->s_size % 4) {
			PERROR("section data size must be align 4\n");
			status = -1;
			goto gsnn_out;
		}

		cmdbuf = kmalloc(gnxtb->s_size, GFP_KERNEL);
		if (cmdbuf == NULL) {
			PERROR("alloc buffer failed\n");
			status = -1;
			goto gsnn_out;
		}

		for (i = 0; i < (gnxtb->s_size / 4); i++)
			cmdbuf[i] = gsnn_read(gsnn, gnxtb->s_addr + i * 4);

		if (copy_to_user((void *)gnxtb + sizeof(struct gnx_table),
				(void *)cmdbuf, gnxtb->s_size)) {
			PERROR("#####set data section failed");
			status = -EFAULT;
			goto gsnn_out;
		}
		kfree(cmdbuf);
		break;

	case  IOCTL_GSNN_REGWRITE:
		PDEBUG("IOCTL_GSNN_REGWRITE addr: 0x%x, size: 0x%x\n",
					gnxtb->s_addr, gnxtb->s_size);

		if (gnxtb->s_size % 4) {
			PERROR("section data size must be align 4\n");
			status = -1;
			goto gsnn_out;
		}

		cmdbuf = kmalloc(gnxtb->s_size, GFP_KERNEL);
		if (cmdbuf == NULL) {
			PERROR("alloc buffer failed\n");
			status = -1;
			goto gsnn_out;
		}

		if (copy_from_user((void *)cmdbuf,
				(void *)gnxtb + sizeof(struct gnx_table),
				gnxtb->s_size)) {
			PERROR("#####set data section failed");
			status = -EFAULT;
			goto gsnn_out;
		}

		for (i = 0; i < (gnxtb->s_size / 4); i++)
			gsnn_write(gsnn, gnxtb->s_addr + i * 4, cmdbuf[i]);
		kfree(cmdbuf);
		break;

	case  IOCTL_GSNN_MEMREAD:
		PDEBUG("IOCTL_GSNN_MEMREAD addr: 0x%x, size: 0x%x\n",
					gnxtb->s_addr, gnxtb->s_size);

		if (gnxtb->s_size > gsnn->hwdata_size) {
			PERROR("gnx table data length must less 0x%lx\n", gsnn->hwdata_size);
			status = -EFAULT;
			goto gsnn_out;
		}

		if (gsnn->hwdata_base + gnxtb->s_addr - gsnn->hwdata_base < 0) {
			PERROR("address of ddr must be hwaddr\n");
			status = -EFAULT;
			goto gsnn_out;
		}

		if (copy_to_user((void *)gnxtb + sizeof(struct gnx_table),
				(void *)gsnn->mem + gnxtb->s_addr,
				gnxtb->s_size)) {
			PERROR("read data to user failed\n");
			status = -EFAULT;
			goto gsnn_out;
		}
		break;

	case  IOCTL_GSNN_MEMWRITE:
		PDEBUG("IOCTL_GSNN_MEMWRITE addr: 0x%x, size: 0x%x\n",
					gnxtb->s_addr, gnxtb->s_size);

		if (gnxtb->s_size > gsnn->hwdata_size) {
			PERROR("gnx table data length must less 0x%lx\n", gsnn->hwdata_size);
			status = -EFAULT;
			goto gsnn_out;
		}

		if (gsnn->hwdata_base + gnxtb->s_addr - gsnn->hwdata_base < 0) {
			PERROR("address of ddr must be hwaddr\n");
			status = -EFAULT;
			goto gsnn_out;
		}

		if (copy_from_user((void *)gsnn->mem + gnxtb->s_addr,
				(void *)gnxtb + sizeof(struct gnx_table),
				gnxtb->s_size)) {
			PERROR("write data section failed\n");
			status = -EFAULT;
			goto gsnn_out;
		}
		break;



	case IOCTL_GSNN_LOADNET:
		gsnnhd = (struct gsnn_header *)arg;
		PDEBUG("IOCTL_GSNN_LOADNET length: 0x%x\n", gsnnhd->length);
		if (gsnnhd->length > GPT_GSNN_CODE_SIZE) {
			PERROR("lenth must < 0x%x\n", GPT_GSNN_CODE_SIZE);
			status = -1;
			goto gsnn_out;
		}

		cmdbuf = kmalloc(gsnnhd->length, GFP_KERNEL);
		if (cmdbuf == NULL) {
			PERROR("alloc buffer failed\n");
			status = -1;
			goto gsnn_out;
		}

		if (copy_from_user(cmdbuf,
			(void *)arg + sizeof(struct gsnn_header), gsnnhd->length)) {
			PERROR("#####set data section failed");
			status = -EFAULT;
			goto gsnn_out;
		}

		//snapshot_from_bin(gsnn, (void *)cmdbuf, &gsnnhd->length);
		kfree(cmdbuf);
		break;

	case IOCTL_GSNN_LOADCFG:
		PDEBUG("IOCTL_GSNN_LOADCFG addr: 0x%x, size: 0x%x\n",
						gnxtb->s_addr, gnxtb->s_size);
		if (copy_to_user((void *)gnxtb + sizeof(struct gnx_table),
				(void *)gsnn->regs + gnxtb->s_addr,
				 gnxtb->s_size)) {
			PERROR("enable data section setting failed\n");
			status = -EFAULT;
			goto gsnn_out;
		}
		break;

	case IOCTL_GSNN_SECT:
		PDEBUG("IOCTL_GSNN_SECT addr: 0x%x, size: 0x%x\n",
						gnxtb->s_addr, gnxtb->s_size);
		if (gnxtb->s_size % 4) {
			PERROR("section data size must be align 4\n");
			status = -1;
			goto gsnn_out;
		}

		cmdbuf = kmalloc(gnxtb->s_size, GFP_KERNEL);
		if (cmdbuf == NULL) {
			PERROR("alloc buffer failed\n");
			status = -1;
			goto gsnn_out;
		}

		if (copy_from_user((void *)cmdbuf,
				(void *)gnxtb + sizeof(struct gnx_table),
				gnxtb->s_size)) {
			PERROR("#####set data section failed");
			status = -EFAULT;
			goto gsnn_out;
		}

		for (i = 0; i < (gnxtb->s_size / 4); i++)
			gsnn_write(gsnn, gnxtb->s_addr + i * 4, cmdbuf[i]);

		kfree(cmdbuf);
		break;

	case IOCTL_GSNN_GLOBAL:
		PDEBUG("IOCTL_GSNN_SECT addr: 0x%x, size: 0x%x\n",
					gnxtb->s_addr, gnxtb->s_size);
		if (gnxtb->s_size % 4) {
			PERROR("section data size must be align 4\n");
			status = -1;
			goto gsnn_out;
		}

		cmdbuf = kmalloc(gnxtb->s_size, GFP_KERNEL);
		if (cmdbuf == NULL) {
			PERROR("alloc buffer failed\n");
			status = -1;
			goto gsnn_out;
		}

		if (copy_from_user((void *)cmdbuf,
			(void *)gnxtb + sizeof(struct gnx_table), gnxtb->s_size)) {
			PERROR("#####set data section failed");
			status = -EFAULT;
			goto gsnn_out;
		}

		for (i = 0; i < (gnxtb->s_size / 4); i++)
			gsnn_write(gsnn, gnxtb->s_addr + i * 4, cmdbuf[i]);

		kfree(cmdbuf);
		break;

	case IOCTL_GSNN_ENABLE:
		PDEBUG("IOCTL_GSNN_ENABLE addr: 0x%x, size: 0x%x, irq: 0x%x\n",
			gnxtb->s_addr, gnxtb->s_size, gsnn_read(gsnn, GSNN_REG_INTCTL));

		reinit_completion(&gsnn->irq_detect);

		if (gnxtb->s_size % 4) {
			PERROR("section data size must be align 4\n");
			status = -1;
			goto gsnn_out;
		}

		cmdbuf = kmalloc(gnxtb->s_size, GFP_KERNEL);
		if (cmdbuf == NULL) {
			PERROR("#####alloc buffer failed\n");
			status = -1;
			goto gsnn_out;
		}

		if (copy_from_user((void *)cmdbuf,
			(void *)gnxtb + sizeof(struct gnx_table), gnxtb->s_size)) {
			PERROR("#####set data section failed");
			status = -EFAULT;
			goto gsnn_out;
		}

		for (i = 0; i < (gnxtb->s_size / 4); i++) {
			gsnn_write(gsnn, gnxtb->s_addr + i * 4, cmdbuf[i]);
		}

		ret = wait_for_completion_timeout(&gsnn->irq_detect, msecs_to_jiffies(timeout));
		if (ret == 0) {
			PERROR("gsnn: wait timeout\n");
			status = -1;
		}

		PDEBUG("pc0(0x%x): 0x%x, pc1(0x%x): 0x%x, pc2(0x%x): 0x%x, \
				pc3(0x%x): 0x%x, error state: 0x%x, irq: 0x%x\n",
				GSNN_REG_PC(0), gsnn_read(gsnn, GSNN_REG_PC(0)),
				GSNN_REG_PC(1), gsnn_read(gsnn, GSNN_REG_PC(1)),
				GSNN_REG_PC(2), gsnn_read(gsnn, GSNN_REG_PC(2)),
				GSNN_REG_PC(3), gsnn_read(gsnn, GSNN_REG_PC(3)),
				gsnn_read(gsnn, GSNN_REG_ERRST),
				gsnn_read(gsnn, GSNN_REG_INTCTL));
		kfree(cmdbuf);
		break;

	case IOCTL_GSNN_RDATA:
	case IOCTL_GSNN_GETRES:
		PDEBUG("IOCTL_GSNN_GETRES addr: 0x%x, size: 0x%x\n", gnxtb->s_addr, gnxtb->s_size);

		if (gnxtb->s_size > gsnn->hwdata_size) {
			PERROR("length data transfer must less 0x%lx\n", gsnn->hwdata_size);
			status = -EFAULT;
			goto gsnn_out;
		}

		if (gnxtb->s_addr - (gsnn->hwdata_base & 0xffffffff) < 0) {
			PERROR("address of ddr must be hwaddr\n");
			status = -EFAULT;
			goto gsnn_out;
		}

		if (copy_to_user((void *)gnxtb + sizeof(struct gnx_table),
				(void *)gsnn->mem + gnxtb->s_addr - (gsnn->hwdata_base & 0xffffffff),
				gnxtb->s_size)) {
			PERROR("get result section setting failed\n");
			status = -EFAULT;
			goto gsnn_out;
		}
		break;

	case IOCTL_GSNN_WRDATA:
		PDEBUG("IOCTL_GSNN_WRDATA addr: 0x%x, size: 0x%x\n", gnxtb->s_addr, gnxtb->s_size);
		if (gnxtb->s_size > gsnn->hwdata_size) {
			PERROR("gnx table data length must less 0x%lx\n", gsnn->hwdata_size);
			status = -EFAULT;
			goto gsnn_out;
		}

		if (gnxtb->s_addr - (gsnn->hwdata_base & 0xffffffff) < 0) {
			PERROR("address of ddr must be hwaddr\n");
			status = -EFAULT;
			goto gsnn_out;
		}
//		printk("lqq-> write data offset %lx\n", gnxtb->s_addr - (gsnn->hwdata_base & 0xffffffff));
		if (copy_from_user((void *)gsnn->mem + (gnxtb->s_addr - (gsnn->hwdata_base & 0xffffffff)),
//		if (copy_from_user((void *)gsnn->mem,
			(void *)gnxtb + sizeof(struct gnx_table), gnxtb->s_size)) {
			PERROR("write data section failed\n");
			status = -EFAULT;
			goto gsnn_out;
		}
		break;
#endif
	default:
		PERROR("cmd(0x%x) is not supported\n", cmd);
	}

gsnn_out:
//	spin_unlock(&gpt_gsnn_lock);
	mutex_unlock(&gsnn->lock);
	return status;
}

#ifdef GPT_GSNN_THREAD
static int gsnn_control_thread(void * data)
{
	struct gpt_gsnn *gsnn = (struct gpt_gsnn *)data;

	for (;;) {
		if (kthread_should_stop()) {
			PDEBUG("gsnn control thread exit\n");
			break;
		}
		msleep(1000);
		PDEBUG("gsnn thread id: %d\n", gsnn->ts->pid);
	}

	return 0;
}
#endif

irqreturn_t gpt_gsnn_irq_handler(int irq, void *data)
{
	unsigned long flags;
	struct gpt_gsnn *gsnn =  (struct gpt_gsnn *)data;
	unsigned int state = gsnn_read(gsnn, GSNN_REG_INTCTL);
	unsigned int state1 = gsnn_read(gsnn, GSNN_REG_G15);

	/* PDEBUG("gpt_gsnn_irq_handler GSNN_REG_INTCTL: 0x%x, GSNN_REG_G15=0x%x\n",
					state, state1); */
	
	if (!((state >> 8) & 0xf))
		return IRQ_NONE;

	local_irq_save(flags);

	if (state & GSNN_OF_INT_REQ) {
		//PDEBUG("gpt_gsnn_irq_handler , GSNN_OF_INT_REQ\n");
		gsnn_write(gsnn, GSNN_REG_INTCLR, GSNN_OF_ERR_EN
				| GSNN_OF_REQ_EN | GSNN_OF_NOPREF_EN
				| GSNN_OF_CPYDN_EN | GSNN_OF_RUNING);
		gsnn_write(gsnn, GSNN_REG_G15,
			(~(0x1 << 30)) & gsnn_read(gsnn, GSNN_REG_G15));
		//PDEBUG("###irq: before complete\n");
		complete(&gsnn->irq_detect);
#if 0		
		gsnn->gsnn_run_status  = 1;
		wake_up(&gsnn->gsnn_wq); 
#endif		
		//PDEBUG("###irq: after complete\n");
	}
	
	if (state & GSNN_OF_INT_ERR) {
		PERROR("pc0(0x%x), pc1(0x%x), pc2(0x%x), pc3(0x%x), error(0x%x)",
				gsnn_read(gsnn, GSNN_REG_PC(0)),
				gsnn_read(gsnn, GSNN_REG_PC(1)),
				gsnn_read(gsnn, GSNN_REG_PC(2)),
				gsnn_read(gsnn, GSNN_REG_PC(3)),
				gsnn_read(gsnn, GSNN_REG_ERRST));
	}

	local_irq_restore(flags);
	
	return IRQ_HANDLED;
}

static int gpt_gsnn_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct gpt_gsnn *gsnn = filp->private_data;

	vma->vm_flags |= VM_IO;
	vma->vm_flags |= (VM_DONTEXPAND | VM_DONTDUMP);
	vma->vm_flags |= VM_MAYSHARE | VM_SHARED;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	
	if (remap_pfn_range(vma, vma->vm_start, gsnn->hwdata_base >> PAGE_SHIFT, gsnn->hwdata_size, \
		vma->vm_page_prot))   
	{
		return -EAGAIN;
	}
	return 0;
}


static struct file_operations gpt_gsnn_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= gpt_gsnn_ioctl,
	.open		= gpt_gsnn_open,
	.release	= gpt_gsnn_release,
	.mmap		= gpt_gsnn_mmap,
};


static int gpt_gsnn_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct resource *res;
	struct device_node *devnode = pdev->dev.of_node;
	struct gpt_gsnn *gsnn;
	int gsnn_id = -1;
	static dev_t  devno;
	char node_name[30];
	int cpu_num = num_possible_cpus();
	
	if (pdev->name)
	{
		if (strstr(pdev->name, "gpt-gsnn0"))
		{
			if (cpu_num < 2)
			{
				PERROR("cpus num=%d, GSNN0 should run on core1, cannot create gsnn0 device!\n", cpu_num);
				return -1;
			}		
			gsnn = gsnn_dev[0];
			gsnn_id = 0;
			devno = devno_gsnn[0];
			strcpy(node_name, "gpt-gsnn0");
		}
		else if (strstr(pdev->name, "gpt-gsnn1"))
		{
			if (cpu_num < 4)
			{
				PERROR("cpus num=%d, GSNN1 should run on core3, cannot create gsnn1 device!\n", cpu_num);
				return -1;
			}
			gsnn = gsnn_dev[1];
			gsnn_id = 1;
			devno = devno_gsnn[1];
			strcpy(node_name, "gpt-gsnn1");
		}
		else
		{
			PERROR("No  correct device name found!\n");
			return -1;
		}
	}
	else
	{
		PERROR("No  gsnn device  found!\n");
		return -1;
	}

	gpt_extsrc_cfg();
	
	gsnn = kmalloc(sizeof(struct gpt_gsnn), GFP_KERNEL);
	if (gsnn == NULL) {
		PERROR("alloc device failed\n");
		return -1;
	}
	platform_set_drvdata(pdev, gsnn);

	gpt_pxi_regmap(gsnn);
	gpt_pxi_get_spr();
	gsnn_device_config();

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		PERROR("no memory resource defination\n");
		ret = -ENODEV;
		goto kmalloc_fail;
	}

	gsnn->hwregs = res->start;
	gpt_pxi_regmap(gsnn);

	gsnn->regs = ioremap(res->start, resource_size(res));
	if (gsnn->regs == NULL) {
		PERROR("register map failed\n");
		ret = -ENODEV;
		goto kmalloc_fail;
	}
	
	PERROR("gsnn register 0x%llx-0x%llx remap to 0x%p\n",
				res->start, res->end, gsnn->regs);	
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (res == NULL) {
		PERROR("no memory resource defination\n");
		ret = -ENODEV;
		goto regmap_fail;
	}
	gsnn->hwdata_base = res->start;
	gsnn->hwdata_size = resource_size(res);
	gsnn->mem = ioremap_nocache(res->start, resource_size(res));
	if (gsnn->mem == NULL) {
		PERROR("memory map failed\n");
		ret = -ENODEV;
		goto regmap_fail;
	}
	PERROR("memory 0x%llx - 0x%llx rmap to 0x%p\n",
				res->start, res->end, gsnn->mem);
#if 1
	gsnn->irq = platform_get_irq(pdev, 0);
	if (gsnn->irq < 0) {
		PERROR("irq has not been detected\n");
		goto memap_fail;
	}
        ret = request_irq(gsnn->irq,
			gpt_gsnn_irq_handler, IRQF_SHARED, "gsnn-irq", gsnn);
        if (ret) {
		PERROR("can't register irq\n");
		ret = -1;
		goto memap_fail;
        }
#else
	gsnn->irq = irq_of_parse_and_map(devnode, 0);

	printk("lqq-> irq %d\n", gsnn->irq);
	ret = request_percpu_irq(gsnn->irq, gpt_gsnn_irq_handler, "gsnn-irq", gsnn);
        if (ret) {
                PERROR("gsnn: can't register interrupt %d (%d)\n", gsnn->irq, ret);
		ret = -1;
		goto memap_fail;
        }
#endif
	ret = register_chrdev_region(devno, 1, node_name);
	if( ret < 0) {
		PERROR("can't register chardev region\n");
		goto irq_fail;
	}
	cdev_init(&gsnn->gsnn_cdev, &gpt_gsnn_fops);
	ret = cdev_add(&gsnn->gsnn_cdev, devno, 1);
	if (ret) {
		PERROR("register cdev failed\n");
		ret = -1;
		goto region_fail;
	}
	gsnn->gsnn_dev = device_create(gpt_gsnn_class, NULL, devno, NULL, node_name);
	if(IS_ERR(gsnn->gsnn_dev)) {
		PERROR("can't create /dev/%s\n", node_name);
		ret = -1;
		goto cdev_fail;
	}
	else
	{
		PDEBUG("create device %s successfully!\n", node_name);
	}

	gsnn->codemem = kmalloc(GPT_GSNN_CODE_SIZE, GFP_KERNEL);
	if (gsnn->codemem == NULL) {
		PERROR("alloc device failed\n");
		ret = -1;
		goto dev_fail;
	}

	init_completion(&gsnn->irq_detect);
	mutex_init(&gsnn->lock);
	init_waitqueue_head(&gsnn->gsnn_wq);
	gsnn->gsnn_run_status = 0;

#ifdef GPT_GSNN_THREAD
	/* Start up our control thread */
	gsnn->ts = kthread_run(gsnn_control_thread, gsnn, "gsnn");
	if (IS_ERR(gsnn->ts)) {
		PERROR("Unable to start gsnn thread\n");
		ret = PTR_ERR(gsnn->ts);
		goto codemem;
	}
#endif

	return ret;

#ifdef GPT_GSNN_THREAD
codemem:
	kfree(gsnn->codemem);
#endif

dev_fail:
	device_del(gsnn->gsnn_dev);
cdev_fail:
	cdev_del(&gsnn->gsnn_cdev);
region_fail:
	unregister_chrdev_region(devno, 1);
irq_fail:
	free_irq(gsnn->irq, gsnn);
memap_fail:
	iounmap(gsnn->mem);
regmap_fail:
	iounmap(gsnn->regs);
kmalloc_fail:
	kfree(gsnn);
	return ret;
}

static int gpt_gsnn_remove(struct platform_device *pdev)
{
	int i = -1;

	if (pdev->name)
	{
		if (strstr(pdev->name, "gpt-gsnn0"))
		{
			i = 0;
		}
		else if (strstr(pdev->name, "gpt-gsnn1"))
		{
			i = 1;
		}		
	}

	if (i < 0 )
	{
		printk("No gsnn dev found!\n");
		return -1;
	}
	
	if (gsnn_dev[i] == NULL) {
		PERROR("%s:%d  gsnn is null\n", __func__, __LINE__);
	}	

#ifdef GPT_GSNN_THREAD
	kthread_stop(gsnn_dev[i]->ts);
#endif

	if (gsnn_dev[i]->codemem) {
		kfree(gsnn_dev[i]->codemem);
		gsnn_dev[i]->codemem = NULL;
	}

	if (gsnn_dev[i]->irq) {
		free_irq(gsnn_dev[i]->irq, gsnn_dev[i]);
		gsnn_dev[i]->irq = 0;
	}

	if (gsnn_dev[i]->mem)
		iounmap(gsnn_dev[i]->mem);

	if (gsnn_dev[i]->regs)
		iounmap(gsnn_dev[i]->regs);

	if (gsnn_dev[i]->gsnn_dev)
		device_del(gsnn_dev[i]->gsnn_dev);

	cdev_del(&gsnn_dev[i]->gsnn_cdev);

	unregister_chrdev_region(devno_gsnn[i], 1);
	kfree(gsnn_dev[i]);
	
	return 0;
}

static const struct of_device_id gpt_gsnn_match[] = {
	{.compatible = "gpt,gpt-gsnn.0", },
	{.compatible = "gpt,gpt-gsnn.1", },
	{},
};

static struct platform_driver gpt_gsnn_driver = {
	.probe = gpt_gsnn_probe,
	.remove = gpt_gsnn_remove,
	.driver = {
		.name = "gpt-gsnn",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(gpt_gsnn_match),
	},
};

static int __init gpt_gsnn_init(void)
{
	gpt_gsnn_class = class_create(THIS_MODULE, "GSNN_CLASS");
	if (IS_ERR(gpt_gsnn_class)) {
		PERROR("can't create GSNN CLASS\n");
		return -1;
	}
	PDEBUG("--%s \n", __FUNCTION__);

	struct platform_device *pdev;
/*	
	pdev = platform_device_register_simple("gpt-gsnn", 0, NULL, 0);
	if (IS_ERR(pdev))
	{
		PERROR("platform_device_register_simple failed!\n");
		return -1;
	}
*/
	int ret = platform_driver_register(&gpt_gsnn_driver);
	return ret;
}

static void __exit gpt_gsnn_exit(void)
{
	PDEBUG("--%s \n", __FUNCTION__);
	platform_driver_unregister(&gpt_gsnn_driver);
	cdev_del(&gsnn_dev[0]->gsnn_cdev);
	cdev_del(&gsnn_dev[1]->gsnn_cdev);
	device_destroy(gpt_gsnn_class, devno_gsnn[0]);
	device_destroy(gpt_gsnn_class, devno_gsnn[1]);		
	if (gpt_gsnn_class) {
		class_destroy(gpt_gsnn_class);
		gpt_gsnn_class = NULL;
	}
	unregister_chrdev_region(devno_gsnn[0], 1);
	unregister_chrdev_region(devno_gsnn[1], 1);	
}

module_init(gpt_gsnn_init);
module_exit(gpt_gsnn_exit);
MODULE_AUTHOR("qqliang");
MODULE_LICENSE("Dual BSD/GPL");
