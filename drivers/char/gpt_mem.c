#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/uaccess.h>
#include <asm/io.h>
#include <asm/mach-chip2/apiu.h>

#define GPT_MEM_IOC_MAGIC 'k'
#define GPT_MEM_IOC_ADDR			_IOW(GPT_MEM_IOC_MAGIC, 1, __u32)

static int gpt_mem_major;
#define DRV_NAME "gpt_mem"
static struct class *gpt_mem_class; 
static DEFINE_MUTEX(device_list_lock);
static unsigned long mem_phy_addr = 0;

static ssize_t gpt_mem_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	unsigned long ret = 0;
	u32 status = 0;
	void __iomem *virt_addr;
	u32 value = 0;

	mutex_lock(&device_list_lock);
	ret = copy_from_user(&value, buf, 4);
	if (ret)
		status = -EFAULT;

	virt_addr = ioremap(mem_phy_addr, 4);

	if (mem_phy_addr >= 0xf0008000 && mem_phy_addr <= 0xf001efff)
		apiu_writel(value, virt_addr);
	else
		writel(value, virt_addr);

	mutex_unlock(&device_list_lock);
	return status;
}

static ssize_t gpt_mem_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	unsigned long ret = 0;
	ssize_t status = 0;
	void __iomem *virt_addr;
	u32 value = 0;

	mutex_lock(&device_list_lock);
	virt_addr = ioremap(mem_phy_addr, 4);

	if (mem_phy_addr >= 0xf0008000 && mem_phy_addr <= 0xf001efff) 
		value = apiu_readl(virt_addr);
	else 
		value = readl(virt_addr);

	ret = copy_to_user(buf, &value, 4);
	if (ret)
		status = -EFAULT;

	mutex_unlock(&device_list_lock);
	return status;
}

static long gpt_mem_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int retval = 0;
	u32 tmp;
	int err = 0;

	/* Check type and command number */
	if (_IOC_TYPE(cmd) != GPT_MEM_IOC_MAGIC)
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


	mutex_lock(&device_list_lock);

	switch (cmd) {
		case GPT_MEM_IOC_ADDR:
			retval = __get_user(tmp, (u32 __user *)arg);
			if (retval == 0) {
				if (tmp < 0xf0007000 || tmp > 0xf044ffff) {
					printk("Invalid argument!\n");
					retval = -EINVAL;
				} else {
					mem_phy_addr = tmp;
				}
			}
			break;
		default:
			break;
	}
	mutex_unlock(&device_list_lock);

	return retval;
}

static int gpt_mem_open(struct inode *inode, struct file *filp)
{
	nonseekable_open(inode, filp);
	return 0;
}

static int gpt_mem_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static const struct file_operations gpt_mem_fops = {
	.owner			= THIS_MODULE,
	.open			= gpt_mem_open,
	.write			= gpt_mem_write,
	.read			= gpt_mem_read,
	.unlocked_ioctl = gpt_mem_ioctl,
	.release		= gpt_mem_release,
};

static int __init gpt_mem_init(void)
{
	gpt_mem_major = register_chrdev(0, DRV_NAME, &gpt_mem_fops);
	if (gpt_mem_major < 0) {
		printk("%s: fail to register one chardev!\n", __func__);
		return gpt_mem_major;
	}

	gpt_mem_class = class_create(THIS_MODULE, "gpt_mem_dev");
	if (IS_ERR(gpt_mem_class)) {
		unregister_chrdev(gpt_mem_major, DRV_NAME);
		return PTR_ERR(gpt_mem_class);
	}

	device_create(gpt_mem_class, NULL, MKDEV(gpt_mem_major, 0), NULL, DRV_NAME);

	return 0;
}
module_init(gpt_mem_init);

static void __exit gpt_mem_exit(void)
{
	device_destroy(gpt_mem_class, MKDEV(gpt_mem_major, 0));
	class_destroy(gpt_mem_class);
	unregister_chrdev(gpt_mem_major, DRV_NAME);
}
module_exit(gpt_mem_exit);

MODULE_LICENSE("GPL");
