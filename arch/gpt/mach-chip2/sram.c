/*
 * Generic on-chip sram allocation driver
 *
 * Copyright (C) 2012 Philipp Zabel, Pengutronix
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/list.h>
#include <linux/list_sort.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/cpu.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/kthread.h>

#include <asm/spr.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/mach-chip2/sram.h>

#define OFFSET_VIDDMA_SIZE	0x8
#define OFFSET_VIDMEM_ADDR	0x20
#define OFFSET_DDR_LOWER_ADDR	0x24
#define OFFSET_DDR_UPPER_ADDR	0x28
#define OFFSET_VIDDMA_CTRL	0x30
#define OFFSET_VIDDMA_STATUS	0x40

/* Set default burst size and length */
#define MASK_VIDDMA_SIZE	0x30000011

/* DMA status */
#define VIDDMA_STATUS_ACTIVE	(1 << 0)
#define VIDDMA_STATUS_PEND	(1 << 1)
#define VIDDMA_STATUS_DONE	(1 << 2)
#define VIDDMA_STATUS_IDLE	(1 << 3)
/* DMA control*/
#define VIDDMA_CTRL_GO		(1 << 0)
#define VIDDMA_CTRL_WRITE	(0 << 1)
#define VIDDMA_CTRL_READ	(1 << 1)

#define SRAM_GRANULARITY	32

struct sram_dev {
        phys_addr_t phys;
	unsigned long start;
	unsigned long end;
	unsigned long size;
	struct gen_pool *pool;
	void __iomem *reg;
	struct miscdevice dev;
};

struct sram_client {
	void *start;
	size_t size;
};

static struct sram_dev gpt_sram_dev;

static void sram_pxi_setup(void)
{
        unsigned long tag, mask;
	struct sram_dev *sram = &gpt_sram_dev;
	/* PXITAG = ADDRESS & ~PXIMASK
 	 */
	mask = (sram->size & ~(sram->size - 1)) -1;
	sprset_gpt(PXIMASK, mask);

	tag = sram->phys | 1;
        sprset_gpt(PXITAG, tag);

	pr_debug("CPU%d setup PXI mask(0x%lx) tag(0x%lx)\n", smp_processor_id(), mask, tag);
}

static inline bool sram_dma_addr_check(unsigned long start, unsigned long addr)
{
	/* Due to dma bug, the start address here must be
	 * within low 4MB range.
	 */
	return (addr >= (start + SZ_4M));
}

static void sram_dma_setup(unsigned long dst,
					unsigned long src, size_t size)
{
	struct sram_dev *sram = &gpt_sram_dev;
	void *reg = sram->reg;
	unsigned long ddr_addr, sram_addr, ctrl;

	if(dst < sram->phys || (dst > sram->phys + sram->size)) {
		ddr_addr = dst;
		sram_addr = src;
		ctrl = VIDDMA_CTRL_GO | VIDDMA_CTRL_READ;
	} else {
                ddr_addr = src;
                sram_addr = dst;
		ctrl = VIDDMA_CTRL_GO & ~VIDDMA_CTRL_READ;
	}

	BUG_ON(sram_dma_addr_check(sram->start, sram_addr));

	writel((size << 5) | MASK_VIDDMA_SIZE, 
			reg + OFFSET_VIDDMA_SIZE);

	writel(sram_addr, reg + OFFSET_VIDMEM_ADDR);
	/* physical addresses are 40 bits */
	writel(ddr_addr, reg + OFFSET_DDR_LOWER_ADDR);
	writel((ddr_addr >> 20), reg + OFFSET_DDR_UPPER_ADDR);
	/* start dma transfer */
	writel(ctrl, reg + OFFSET_VIDDMA_CTRL);
}

static inline bool sram_wait_dma_status(unsigned long st)
{
	u32 val;
	int count = 2000;

	struct sram_dev *sram = &gpt_sram_dev;
	void *reg = sram->reg;

	do {
		val = readl(reg + OFFSET_VIDDMA_STATUS);
		if(val & st) {
			return true;
		} else
			udelay(1);
	}while(count--);
	
	return false;
}

struct gen_pool *sram_get_gen_pool(void)
{
	struct sram_dev *sram = &gpt_sram_dev;
	return sram->pool;
}
EXPORT_SYMBOL(sram_get_gen_pool);

void *sram_alloc(size_t size)
{
	struct sram_dev *sram = &gpt_sram_dev;

	if(!sram->pool)
		return NULL;

	return gen_pool_dma_alloc(sram->pool, size, NULL);
}
EXPORT_SYMBOL(sram_alloc);

void *sram_dma_alloc(size_t size, dma_addr_t *dma)
{
	struct sram_dev *sram = &gpt_sram_dev;

        if(!sram->pool || !dma)
                return NULL;

        return gen_pool_dma_alloc(sram->pool, size, dma);
}
EXPORT_SYMBOL(sram_dma_alloc);

void sram_free(void *addr, size_t len)
{
	struct sram_dev *sram = &gpt_sram_dev;
	if(sram->pool)
        	gen_pool_free(sram->pool, (unsigned long)addr, len);
}
EXPORT_SYMBOL(sram_free);

static DEFINE_MUTEX(sram_mutex);

bool sram_dma_copy(dma_addr_t dst, dma_addr_t src, size_t size)
{
	struct sram_dev *sram = &gpt_sram_dev;

	if(size > sram->size) {
		pr_err("requested size(%lx) is too big!\n", size);
		return false;
	}

	mutex_lock(&sram_mutex);

	sram_dma_setup((unsigned long)dst, (unsigned long)src, size);
	sram_wait_dma_status(VIDDMA_STATUS_ACTIVE);
	if(!sram_wait_dma_status(VIDDMA_STATUS_IDLE)) {
		mutex_unlock(&sram_mutex);
		pr_err("sram dma is timeout\n");
		return false;
	}

	mutex_unlock(&sram_mutex);

	return true;
}
EXPORT_SYMBOL(sram_dma_copy);

static unsigned long sram_uservirt_to_phys(unsigned long virtp)
{
        unsigned long physp = 0;
        struct vm_area_struct *vma;
        struct mm_struct *mm = current->mm;

        /* For kernel direct-mapped memory, take the easy way */
        if (virtp >= PAGE_OFFSET)
                return virt_to_phys((void *) virtp);

        down_read(&current->mm->mmap_sem);
        vma = find_vma(mm, virtp);
        if (vma && (vma->vm_flags & VM_IO) && vma->vm_pgoff) {
                /* this will catch, kernel-allocated, mmaped-to-usermode
                   addresses */
                physp = (vma->vm_pgoff << PAGE_SHIFT) + (virtp - vma->vm_start);
                up_read(&current->mm->mmap_sem);
        } else {
                /* otherwise, use get_user_pages() for general userland pages */
                int res, nr_pages = 1;
                struct page *pages;

                res = get_user_pages(current, current->mm, virtp, nr_pages, 1,
                                0, &pages, NULL);
                up_read(&current->mm->mmap_sem);

                if (res == nr_pages) {
                        physp =  __pa(page_address(&pages[0]) +
                                        (virtp & ~PAGE_MASK));
                } else {
                        pr_err("get_user_pages failed\n");
                        return 0;
                }
        }

        return physp;
}


static long sram_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	size_t size = arg;
	struct sram_client *client = file->private_data;

        switch (cmd) {
		case SRAM_IOCTL_ALLOC:
		{
			client->start = sram_alloc(size);
			if(!client->start)
				return -ENOMEM;
			client->size = size;

		        break;
		}
		case SRAM_IOCTL_FREE:
		{
			if(client->start)
				sram_free(client->start, client->size);
			client->start = NULL;

		        break;
		}
		default:
			break;
	}

	return 0;
}

static loff_t sram_lseek(struct file *file, loff_t offset, int orig)
{
        loff_t ret;

        mutex_lock(&file_inode(file)->i_mutex);
        switch (orig) {
        case SEEK_CUR:
                offset += file->f_pos;
        case SEEK_SET:
                /* to avoid userland mistaking f_pos=-9 as -EBADF=-9 */
                if (IS_ERR_VALUE((unsigned long long)offset)) {
                        ret = -EOVERFLOW;
                        break;
                }
                file->f_pos = offset;
                ret = file->f_pos;
                break;
        default:
                ret = -EINVAL;
        }
        mutex_unlock(&file_inode(file)->i_mutex);
        return ret;
}

static ssize_t sram_read(struct file *file, char __user *buf,
                        size_t count, loff_t *ppos)
{
	loff_t offset;
	unsigned long src, dst;
	cycles_t start_cycles, end_cycles;
	struct sram_dev *sram = &gpt_sram_dev;
	struct sram_client *client = file->private_data;

	if(!client->start)
		return -ENODEV;
	if(count > client->size)
		return -EINVAL;

	start_cycles = get_cycles();

	dst = sram_uservirt_to_phys((unsigned long)buf);
	if(!dst)
		return -EFAULT;

	offset = (loff_t)(client->start - sram->start) + *ppos;
	src = offset + (unsigned long)sram->phys;

	if(!sram_dma_copy((dma_addr_t)dst, (dma_addr_t)src, count)) {
		pr_err("sram read dma error\n");
		return -EFAULT;
	}
	end_cycles = get_cycles();

	*ppos += count;

	pr_debug("read: cycles %lld %lld %lld\n", start_cycles, end_cycles, end_cycles - start_cycles);

	return count;
}

static ssize_t sram_write(struct file *file, const char __user *buf,
                        size_t count, loff_t *ppos)
{
	loff_t offset;
	unsigned long src, dst;
	cycles_t start_cycles, end_cycles;
	struct sram_dev *sram = &gpt_sram_dev;
	struct sram_client *client = file->private_data;

	if(!client->start)
		return -ENODEV;
	if(count > client->size)
		return -EINVAL;

	start_cycles = get_cycles();

	src = sram_uservirt_to_phys((unsigned long)buf);
	if(!src) {
		pr_err("sram write convert user address error\n");
		return -EFAULT;
	}

	offset = (loff_t)(client->start - sram->start) + *ppos;
	dst = offset + (unsigned long)sram->phys;
	if(!sram_dma_copy((dma_addr_t)dst, (dma_addr_t)src, count)) {
		pr_err("sram write dma error\n");
		return -EFAULT;
	}
	end_cycles = get_cycles();

	*ppos += count;

	pr_debug("write: cycles %lld %lld %lld\n", start_cycles, end_cycles, end_cycles - start_cycles);

	return count;
}

static int sram_release(struct inode *inode, struct file *file)
{
	struct sram_client *client = file->private_data;

	if(client) {
		kfree(client);
	}

        return 0;
}

static int sram_open(struct inode *inode, struct file *file)
{
	struct sram_client *client;

	client = kmalloc(sizeof(struct sram_client), GFP_KERNEL);
	if(!client) {
		pr_err("failed to alloc sram_client\n");
		return -ENOMEM;
	}
	memset(client, 0, sizeof(struct sram_client));

	file->private_data = client;

	return 0;
}

static const struct file_operations sram_fops = {
        .owner          = THIS_MODULE,
        .open           = sram_open,
        .release        = sram_release,
	.llseek         = sram_lseek,
	.read		= sram_read,
	.write		= sram_write,
        .unlocked_ioctl	= sram_ioctl,
};

#if (CONFIG_NR_CPUS >= 4)
static int sram_nb(struct notifier_block *nfb,
			      unsigned long action, void *hcpu)
{
	if ((action == CPU_STARTING) && 
			(smp_processor_id() == 2)) {
		sram_pxi_setup();
	}
		
	return NOTIFY_OK;
}
static struct notifier_block sram_cpu_notifier = {
	.notifier_call = sram_nb,
	.priority = 100,
};
#endif

const struct of_device_id sram_dt_ids[] = {
	{ .compatible = "mmio-sram" },
	{}
};

int __init early_sram_init(void)
{
	struct device_node *np;
	struct gen_pool *sram_pool;
	const u32 *prop_val;
	u64 prop_size = 0;
	phys_addr_t phys;
	unsigned long size;
	void __iomem *addr;
	int ret = 0;
	struct sram_dev *sdev = &gpt_sram_dev;

	np = of_find_matching_node(NULL, sram_dt_ids);
	if (!np)
		return -ENODEV;
        prop_val = of_get_address(np, 0, &prop_size, NULL);
        if (!prop_val)
                return -EINVAL;

	phys = (phys_addr_t)of_translate_address(np, prop_val);
	size = prop_size;

        of_node_put(np);
		
        sram_pool = gen_pool_create(ilog2(SRAM_GRANULARITY), -1);
        if (!sram_pool)
		return -ENOMEM;

        addr = ioremap(phys, size);
        if (!addr)
                return -ENOMEM;
        ret = gen_pool_add_virt(sram_pool, (unsigned long) addr,
                                   phys, size, -1);
	if(ret < 0) {
                iounmap(addr);
		return ret; 
	}

	sdev->pool = sram_pool;
	sdev->phys = phys;
	sdev->start = (unsigned long)addr;
	sdev->end = (unsigned long)addr + size;
	sdev->size = size;

/* Only core0 and core2 are allowed on chip2 */
        cpumask_set_cpu(0, &cpus_allowed);
#if (CONFIG_NR_CPUS >= 4)
        cpumask_set_cpu(2, &cpus_allowed);
	register_cpu_notifier(&sram_cpu_notifier);
#endif
	sram_pxi_setup();

	return 0;
}
early_initcall(early_sram_init);

static int sram_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int ret = 0;
	struct sram_dev *sdev = &gpt_sram_dev;

	if(!sdev->pool) {
		dev_err(&pdev->dev,"sram pool was not created\n");
		return -ENODEV;
	}

	sdev->reg = of_iomap(np, 0);
	if(!sdev->reg) {
		dev_err(&pdev->dev,"failed to map reg range\n");
		return -ENODEV;
	}

        sdev->dev.minor = MISC_DYNAMIC_MINOR;
        sdev->dev.name = "sram";
        sdev->dev.fops = &sram_fops;
        sdev->dev.parent = NULL;
        ret = misc_register(&sdev->dev);
        if (ret) {
                dev_err(&pdev->dev,"failed to register misc device, ret=%d.\n", ret);
                goto unmap;
        }

	platform_set_drvdata(pdev, sdev);

	dev_info(&pdev->dev,"reg %p, virt 0x%lx-0x%lx, phys 0x%lx\n",
				sdev->reg,sdev->start,sdev->end, (unsigned long)sdev->phys);

	return 0;
unmap:
	iounmap(sdev->reg);

	return ret;
}

static int sram_remove(struct platform_device *pdev)
{
	struct sram_dev *sdev = platform_get_drvdata(pdev);

	if (gen_pool_avail(sdev->pool) < gen_pool_size(sdev->pool))
		dev_dbg(&pdev->dev, "removed while sram allocated\n");

	misc_deregister(&sdev->dev);

	iounmap(sdev->reg);

	return 0;
}

static struct of_device_id gpt_sram_dt_ids[] = {
	{ .compatible = "gpt,sram" },
	{}
};

static struct platform_driver sram_driver = {
	.driver = {
		.name = "gpt_sram",
		.of_match_table = of_match_ptr(gpt_sram_dt_ids),
	},
	.probe = sram_probe,
	.remove = sram_remove,
};

static int __init sram_init(void)
{
	return platform_driver_register(&sram_driver);
}

fs_initcall(sram_init);
