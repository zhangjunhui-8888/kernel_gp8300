/*
 * Generic on-chip vidmem allocation driver
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
#include <linux/dma-mapping.h>
#include <linux/kfifo.h>
#include <asm/cacheflush.h>

#include <asm/spr.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/mach-chip2/sram.h>

#define OFFSET_VIDDMA_SIZE	0x8
#define OFFSET_VIDMEM_ADDR	0x20
#define OFFSET_DDR_LOWER_ADDR	0x24
#define OFFSET_DDR_UPPER_ADDR	0x28
#define OFFSET_VIDDMA_CTRL	0x30
#define OFFSET_VIDDMA_STATUS	0x58

#define MASK_VIDDMA_STATUS	0xff

/* Set default burst size and length */
#define MASK_VIDDMA_SIZE	0x60000011

/* DMA status */
#define VIDDMA_STATUS_ACTIVE	(1 << 0)
#define VIDDMA_STATUS_PEND	(1 << 1)
#define VIDDMA_STATUS_DONE	(1 << 2)
#define VIDDMA_STATUS_IDLE	(1 << 3)
/* DMA control*/
#define VIDDMA_CTRL_GO		(1 << 0)
#define VIDDMA_CTRL_WRITE	(0 << 1)
#define VIDDMA_CTRL_READ	(1 << 1)

#define VIDMEM_IOCTL_ALLOC	SRAM_IOCTL_ALLOC
#define VIDMEM_IOCTL_FREE	SRAM_IOCTL_FREE
#define VIDMEM_IOCTL_COPY	SRAM_IOCTL_COPY
#define VIDMEM_IOCTL_PAGECOPY	_IOW('S', 5, size_t)

#define VIDMEM_GRANULARITY	32
#define VIDMEM_RESERVER_SIZE	0

#define SRAM_PAGE_SIZE			PAGE_SIZE
#define VIDMEM_PAGE_MASK		(PAGE_SIZE - 1)
#define GET_SRAM_PAGES(len)			\
	((len / SRAM_PAGE_SIZE + ((len % SRAM_PAGE_SIZE) ? 1 : 0)) * PAGE_SIZE)


#ifdef CONFIG_DDR2DDR_COPY
#undef VIDMEM_RESERVER_SIZE
#define VIDMEM_RESERVER_SIZE	0x8000
#endif

struct vidmem_dev {
        phys_addr_t phys;
	unsigned long start;
	unsigned long end;
	unsigned long size;
	struct gen_pool *pool;
	void __iomem *reg;
	struct miscdevice dev;
};

struct vidmem_client {
	void *start;
	size_t size;
	struct page *pages[1];
};

static struct vidmem_dev gpt_vidmem_dev;

static void vidmem_pxi_setup(void)
{
        unsigned long tag, mask;
	struct vidmem_dev *vidmem = &gpt_vidmem_dev;
	/* PXITAG = ADDRESS & ~PXIMASK
 	 */
	mask = (vidmem->size & ~(vidmem->size - 1)) -1;
	sprset_gpt(PXIMASK, mask);

	tag = vidmem->phys | 1;
        sprset_gpt(PXITAG, tag);

	pr_debug("CPU%d setup PXI mask(0x%lx) tag(0x%lx)\n", smp_processor_id(), mask, tag);
}

static inline bool vidmem_dma_addr_check(unsigned long start, unsigned long addr)
{
	/* Due to dma bug, the start address here must be
	 * within low 4MB range.
	 */
	return (addr >= (start + SZ_4M));
}

static void vidmem_dma_setup(unsigned long dst,
					unsigned long src, size_t size)
{
	struct vidmem_dev *vidmem = &gpt_vidmem_dev;
	void *reg = vidmem->reg;
	unsigned long ddr_addr, vidmem_addr, ctrl;

	if(dst < vidmem->phys || (dst > vidmem->phys + vidmem->size)) {
		ddr_addr = dst;
		vidmem_addr = src;
		ctrl = VIDDMA_CTRL_GO | VIDDMA_CTRL_READ;
	} else {
                ddr_addr = src;
                vidmem_addr = dst;
		ctrl = VIDDMA_CTRL_GO & ~VIDDMA_CTRL_READ;
	}

	BUG_ON(vidmem_dma_addr_check(vidmem->start, vidmem_addr));

	writel(((size - 1) << 5) | MASK_VIDDMA_SIZE,
			reg + OFFSET_VIDDMA_SIZE);

	writel(vidmem_addr, reg + OFFSET_VIDMEM_ADDR);
	/* physical addresses are 40 bits */
	writel(ddr_addr, reg + OFFSET_DDR_LOWER_ADDR);
	writel((ddr_addr >> 20), reg + OFFSET_DDR_UPPER_ADDR);
	/* start dma transfer */

	writel(ctrl, reg + OFFSET_VIDDMA_CTRL);
}

static inline bool vidmem_wait_dma_status(unsigned long st)
{
	u32 val;
	int count = 20000;

	struct vidmem_dev *vidmem = &gpt_vidmem_dev;
	void *reg = vidmem->reg;

	do {
		val = readl(reg + OFFSET_VIDDMA_STATUS);
		if(val & st) {
			if (st & VIDDMA_STATUS_DONE)
				writel(MASK_VIDDMA_STATUS,		\
					reg + OFFSET_VIDDMA_STATUS);

			return true;
		} else
			udelay(1);
	} while(count--);
	
	return false;
}

struct gen_pool *vidmem_get_gen_pool(void)
{
	struct vidmem_dev *vidmem = &gpt_vidmem_dev;
	return vidmem->pool;
}
EXPORT_SYMBOL(vidmem_get_gen_pool);

void *vidmem_alloc(size_t size)
{
	struct vidmem_dev *vidmem = &gpt_vidmem_dev;

	if(!vidmem->pool)
		return NULL;

	return gen_pool_dma_alloc(vidmem->pool, size, NULL);
}
EXPORT_SYMBOL(vidmem_alloc);

void *vidmem_dma_alloc(size_t size, dma_addr_t *dma)
{
	struct vidmem_dev *vidmem = &gpt_vidmem_dev;

        if(!vidmem->pool || !dma)
                return NULL;

        return gen_pool_dma_alloc(vidmem->pool, size, dma);
}
EXPORT_SYMBOL(vidmem_dma_alloc);

void vidmem_free(void *addr, size_t len)
{
	struct vidmem_dev *vidmem = &gpt_vidmem_dev;
	if(vidmem->pool)
		gen_pool_free(vidmem->pool, (unsigned long)addr, len);
}
EXPORT_SYMBOL(vidmem_free);

static DEFINE_MUTEX(vidmem_mutex);

static unsigned long vidmem_uservirt_to_phys(unsigned long virtp,
							struct page **pages)
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

                res = get_user_pages(current, current->mm, virtp, nr_pages, 1,
                                0, pages, NULL);
                up_read(&current->mm->mmap_sem);

                if (res == nr_pages) {
                        physp =  __pa(page_address(pages[0]) +
                                        (virtp & ~PAGE_MASK));
                } else {
                        pr_err("get_user_pages failed\n");
                        return 0;
                }
        }

        return physp;
}

bool vidmem_dma_copy(dma_addr_t dst, dma_addr_t src, size_t size)
{
	struct vidmem_dev *vidmem = &gpt_vidmem_dev;

	if(size > vidmem->size) {
		pr_err("requested size(%lx) is too big!\n", size);
		return false;
	}

	mutex_lock(&vidmem_mutex);

	vidmem_dma_setup((unsigned long)dst, (unsigned long)src, size);
	if(!vidmem_wait_dma_status(VIDDMA_STATUS_DONE)) {
		mutex_unlock(&vidmem_mutex);
		pr_err("vidmem dma is timeout\n");
		return false;
	}

	mutex_unlock(&vidmem_mutex);

	return true;
}
EXPORT_SYMBOL(vidmem_dma_copy);

#ifdef CONFIG_DDR2DDR_COPY
bool vidmem_dma_user_copy(struct vdma_desc *vdma_desc)
{
	bool res = true;
	int loop, i;
	size_t length;
	struct vidmem_dev *vidmem = &gpt_vidmem_dev;

	loop = vdma_desc->size / VIDMEM_RESERVER_SIZE;
	length = VIDMEM_RESERVER_SIZE;
	for (i = 0; i < loop && res; i++) {
		res = vidmem_dma_copy(vidmem->phys,			\
				vdma_desc->src + i * length, length);
		res = vidmem_dma_copy(vdma_desc->dst + i * length,
						vidmem->phys, length);
	}
	length = vdma_desc->size % VIDMEM_RESERVER_SIZE;
	if (length) {
		res = vidmem_dma_copy(vidmem->phys,			\
			vdma_desc->src + i * VIDMEM_RESERVER_SIZE, length);
		res = vidmem_dma_copy(vdma_desc->dst +		\
			i * VIDMEM_RESERVER_SIZE, vidmem->phys, length);
	}

	return res;
}
#endif

static int vidmem_dma_page_copy(struct file *file, struct vdma_desc desc)
{
	size_t len;
	unsigned long src, dst;
	unsigned long user_virt;
	struct vidmem_dev *vidmem = &gpt_vidmem_dev;
	struct vidmem_client *client = file->private_data;
	int count = desc.size;
	struct page **pages = client->pages;

	if(!client->start)
		return -ENODEV;

	if(count > client->size)
		return -EINVAL;

	if(desc.dst > vidmem->size) {
		user_virt = desc.dst;
		//get offset
		src = (unsigned long)(client->start - vidmem->start);
		//src = offset + user_offset + base
		src = src + desc.src + (unsigned long)vidmem->phys;
		while(count) {
			dst = vidmem_uservirt_to_phys(user_virt, pages);
			if(!dst) {
				pr_err("vidmem page copy convert "
						"user address error\n");
				return -EFAULT;
			}
			len = PAGE_SIZE - (dst & VIDMEM_PAGE_MASK);
			if(len > count)
				len = count;

			__dma_sync_cache_range((unsigned long)phys_to_virt(dst),
					(unsigned long)phys_to_virt(dst + len));

			if(!vidmem_dma_copy((dma_addr_t)dst,
						(dma_addr_t)src, len)) {
				pr_err("vidmem dma vidmem to ddr err\n");
				return -EFAULT;
			}

			user_virt += len;
			src += len;
			count -= len;
			if(pages[0]) {
				put_page(pages[0]);
				pages[0] = NULL;
			}
		}
	} else {
		user_virt  = desc.src;
		//get offset
		dst = (unsigned long)(client->start - vidmem->start);
		//dst = offset + user_offset + base
		dst = dst + desc.dst + (unsigned long)vidmem->phys;

		while(count) {
			src = vidmem_uservirt_to_phys(user_virt, pages);
			if(!src) {
				pr_err("vidmem page copy convert "
						"user address error\n");
				return -EFAULT;
			}

			len = PAGE_SIZE - (src & VIDMEM_PAGE_MASK);
			if(len > count)
				len = count;

			__dma_sync_cache_range((unsigned long)phys_to_virt(src),
					(unsigned long)phys_to_virt(src + len));
			if(!vidmem_dma_copy((dma_addr_t)dst,
						(dma_addr_t)src, len)) {
				pr_err("vidmem dma vidmem to ddr err\n");
				return -EFAULT;
			}
			user_virt += len;
			dst += len;
			count -= len;
			if(pages[0]) {
				put_page(pages[0]);
				pages[0] = NULL;
			}

		}
	}

	return 0;
}

static ssize_t vidmem_read(struct file *file, char __user *buf,
                        size_t count, loff_t *ppos)
{
	loff_t offset;
	unsigned long src, dst;
	cycles_t start_cycles, end_cycles;
	struct vidmem_dev *vidmem = &gpt_vidmem_dev;
	struct vidmem_client *client = file->private_data;
	struct page **pages = client->pages;


	if(!client->start)
		return -ENODEV;

	if(count > client->size)
		return -EINVAL;

	dst = vidmem_uservirt_to_phys((unsigned long)buf, pages);
	if(!dst)
		return -EFAULT;

	offset = (loff_t)(client->start - vidmem->start) + *ppos;
	src = offset + (unsigned long)vidmem->phys;

	start_cycles = get_cycles();

	__dma_sync_cache_range((unsigned long)phys_to_virt(dst),
			       (unsigned long)phys_to_virt(dst + count));
	if(!vidmem_dma_copy((dma_addr_t)dst, (dma_addr_t)src, count)) {
		pr_err("vidmem read dma error\n");
		return -EFAULT;
	}

	end_cycles = get_cycles();

	*ppos += count;

	pr_debug("read: cycles %lld %lld %lld\n", start_cycles, end_cycles, end_cycles - start_cycles);

	return count;
}

static ssize_t vidmem_write(struct file *file, const char __user *buf,
                        size_t count, loff_t *ppos)
{
	loff_t offset;
	unsigned long src, dst;
	cycles_t start_cycles, end_cycles;
	struct vidmem_dev *vidmem = &gpt_vidmem_dev;
	struct vidmem_client *client = file->private_data;
	struct page **pages = client->pages;

	if(!client->start)
		return -ENODEV;

	if(count > client->size)
		return -EINVAL;

	src = vidmem_uservirt_to_phys((unsigned long)buf, pages);
	if(!src) {
		pr_err("vidmem write convert user address error\n");
		return -EFAULT;
	}

	offset = (loff_t)(client->start - vidmem->start) + *ppos;
	dst = offset + (unsigned long)vidmem->phys;
	start_cycles = get_cycles();
	__dma_sync_cache_range((unsigned long)phys_to_virt(src),
			       (unsigned long)phys_to_virt(src + count));

	if(!vidmem_dma_copy((dma_addr_t)dst, (dma_addr_t)src, count)) {
		pr_err("vidmem write dma error\n");
		return -EFAULT;
	}
	end_cycles = get_cycles();

	*ppos += count;

	pr_debug("write: cycles %lld %lld %lld\n", start_cycles, end_cycles, end_cycles - start_cycles);

	return count;
}


static long vidmem_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	size_t size = arg;
	struct vidmem_client *client = file->private_data;

        switch (cmd) {
		case VIDMEM_IOCTL_ALLOC:
		{
			size = GET_SRAM_PAGES(size);
			if(client->start) {
				pr_err("File desc realloc\n");
				return -EFAULT;
			}
			client->start = vidmem_alloc(size);
			if(!client->start)
				return -ENOMEM;
			client->size = size;

		        break;
		}
		case VIDMEM_IOCTL_FREE:
		{
			if(client->start)
				vidmem_free(client->start, client->size);
			client->start = NULL;

		        break;
		}
		case VIDMEM_IOCTL_PAGECOPY:
		{
			int ret = 0;
			struct vdma_desc vdma_desc;

			copy_from_user(&vdma_desc, (void *)arg,
						sizeof(struct vdma_desc));

			ret = vidmem_dma_page_copy(file, vdma_desc);
			return ret;
		}
#ifdef CONFIG_DDR2DDR_COPY
		case VIDMEM_IOCTL_COPY:
		{
			struct vdma_desc vdma_desc;

			copy_from_user(&vdma_desc, (void *)arg,
						sizeof(struct vdma_desc));
			vdma_desc.dst = vidmem_uservirt_to_phys(vdma_desc.dst,
							client->page);
			vdma_desc.src = vidmem_uservirt_to_phys(vdma_desc.src,
							client->page);
			if(!vdma_desc.dst || !vdma_desc.src)
				return -EFAULT;

			if (!vidmem_dma_user_copy(&vdma_desc))
				return -EINVAL;

			break;
		}
#endif
		default:
		{
			pr_err("ioctl command does not exist\n");
			return -EFAULT;
		}
	}

	return 0;
}

int vidmem_mmap(struct file *file, struct vm_area_struct *vma)
{
	phys_addr_t phys;
	struct vidmem_dev *vidmem = &gpt_vidmem_dev;
	struct vidmem_client *client = file->private_data;
	size_t size = vma->vm_end - vma->vm_start;
	unsigned long off = (unsigned long)(client->start - vidmem->start);
	unsigned long pfn;

	if(size > client->size)
		return -EINVAL;

	phys = vidmem->phys + off;
	pfn = phys >> PAGE_SHIFT;
	vma->vm_pgoff += pfn;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	/* Remap-pfn-range will mark the range VM_IO*/

	if (remap_pfn_range(vma,
			    vma->vm_start,
			    vma->vm_pgoff,
			    size,
			    vma->vm_page_prot)) {
		return -EAGAIN;
	}

	return 0;
}

static loff_t vidmem_lseek(struct file *file, loff_t offset, int orig)
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


static int vidmem_release(struct inode *inode, struct file *file)
{
	struct vidmem_client *client = file->private_data;

        mutex_lock(&file_inode(file)->i_mutex);
	if(client) {
		if(client->start)
			vidmem_free(client->start, client->size);
		kfree(client);
	}
        mutex_unlock(&file_inode(file)->i_mutex);

        return 0;
}

static int vidmem_open(struct inode *inode, struct file *file)
{
	struct vidmem_client *client;

        mutex_lock(&file_inode(file)->i_mutex);
	client = kmalloc(sizeof(struct vidmem_client), GFP_KERNEL);
	if(!client) {
		pr_err("failed to alloc vidmem_client\n");
		return -ENOMEM;
	}
	memset(client, 0, sizeof(struct vidmem_client));

	file->private_data = client;
        mutex_unlock(&file_inode(file)->i_mutex);

	return 0;
}

static const struct file_operations vidmem_fops = {
        .owner          = THIS_MODULE,
        .open           = vidmem_open,
        .release        = vidmem_release,
        .unlocked_ioctl	= vidmem_ioctl,
	.mmap		= vidmem_mmap,
	.llseek         = vidmem_lseek,
	.read		= vidmem_read,
	.write		= vidmem_write,
};

#if (CONFIG_NR_CPUS >= 4)
static int vidmem_nb(struct notifier_block *nfb,
			      unsigned long action, void *hcpu)
{
	if ((action == CPU_STARTING) && 
			(smp_processor_id() == 2)) {
		vidmem_pxi_setup();
	}
		
	return NOTIFY_OK;
}
static struct notifier_block vidmem_cpu_notifier = {
	.notifier_call = vidmem_nb,
	.priority = 100,
};
#endif

const struct of_device_id vidmem_dt_ids[] = {
	{ .compatible = "mmio-vidmem" },
	{}
};

int __init early_vidmem_init(void)
{
	struct device_node *np;
	struct gen_pool *vidmem_pool;
	const u32 *prop_val;
	u64 prop_size = 0;
	phys_addr_t phys;
	unsigned long size;
	void __iomem *addr;
	int ret = 0;
	struct vidmem_dev *sdev = &gpt_vidmem_dev;

	np = of_find_matching_node(NULL, vidmem_dt_ids);
	if (!np)
		return -ENODEV;
        prop_val = of_get_address(np, 0, &prop_size, NULL);
        if (!prop_val)
                return -EINVAL;

	phys = (phys_addr_t)of_translate_address(np, prop_val);
	size = prop_size;

        of_node_put(np);
		
        vidmem_pool = gen_pool_create(ilog2(VIDMEM_GRANULARITY), -1);
        if (!vidmem_pool)
		return -ENOMEM;

        addr = ioremap(phys, size);
        if (!addr)
                return -ENOMEM;
        ret = gen_pool_add_virt(vidmem_pool,
			(unsigned long)addr + VIDMEM_RESERVER_SIZE, phys +
			      VIDMEM_RESERVER_SIZE, size - VIDMEM_RESERVER_SIZE, -1);
	if(ret < 0) {
                iounmap(addr);
		return ret; 
	}

	sdev->pool = vidmem_pool;
	sdev->phys = phys;
	sdev->start = (unsigned long)addr;
	sdev->end = (unsigned long)addr + size;
	sdev->size = size;

/* Only core0 and core2 are allowed on chip2 */
        cpumask_set_cpu(0, &cpus_allowed);
#if (CONFIG_NR_CPUS >= 4)
        cpumask_set_cpu(2, &cpus_allowed);
	register_cpu_notifier(&vidmem_cpu_notifier);
#endif
	vidmem_pxi_setup();

	return 0;
}
early_initcall(early_vidmem_init);

static int vidmem_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int ret = 0;
	struct vidmem_dev *sdev = &gpt_vidmem_dev;

	if(!sdev->pool) {
		dev_err(&pdev->dev,"vidmem pool was not created\n");
		return -ENODEV;
	}

	sdev->reg = of_iomap(np, 0);
	if(!sdev->reg) {
		dev_err(&pdev->dev,"failed to map reg range\n");
		return -ENODEV;
	}

        sdev->dev.minor = MISC_DYNAMIC_MINOR;
        sdev->dev.name = "vidmem";
        sdev->dev.fops = &vidmem_fops;
        sdev->dev.parent = NULL;
        ret = misc_register(&sdev->dev);
        if(ret) {
                dev_err(&pdev->dev,"failed to register misc device, ret=%d.\n",
									ret);
                goto unmap;
        }

	platform_set_drvdata(pdev, sdev);

	dev_info(&pdev->dev,"reg %p, virt 0x%lx-0x%lx, phys 0x%lx\n",
				sdev->reg,sdev->start,sdev->end,
				(unsigned long)sdev->phys);


	return 0;
unmap:
	iounmap(sdev->reg);

	return ret;
}

static int vidmem_remove(struct platform_device *pdev)
{
	struct vidmem_dev *sdev = platform_get_drvdata(pdev);

	if (gen_pool_avail(sdev->pool) < gen_pool_size(sdev->pool))
		dev_dbg(&pdev->dev, "removed while vidmem allocated\n");

	misc_deregister(&sdev->dev);

	iounmap(sdev->reg);

	return 0;
}

static struct of_device_id gpt_vidmem_dt_ids[] = {
	{ .compatible = "gpt,vidmem" },
	{}
};

static struct platform_driver vidmem_driver = {
	.driver = {
		.name = "gpt_vidmem",
		.of_match_table = of_match_ptr(gpt_vidmem_dt_ids),
	},
	.probe = vidmem_probe,
	.remove = vidmem_remove,
};

static int __init vidmem_init(void)
{
	return platform_driver_register(&vidmem_driver);
}

fs_initcall(vidmem_init);
