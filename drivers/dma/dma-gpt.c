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
#include <linux/syscalls.h>
#include <asm/cacheflush.h>
#include <asm/spr.h>
#include <asm/mach-chip2/dma.h>

#undef 	MODULE
#define CPU_DMA_MAJOR			0

#define PXI_EN				(0x1 << 0)
#define CPU_DMA_BASE			0xe2000000
#define CPU_DMA_MASK			0x0000ffff
#define CDMA_PAGE_SIZE			0x10000
#define CDMA_STATUS			0x000
#define CDMA_MISS			0x001
#define CDMA_SCTL			0x002
#define CDMA_ASYNC			0x003
#define CDMA_CMD			0x005
#define CDMA_TIMEOUT			0x100000
#define CDMA_REG(reg)			(reg_base + reg)
#define ADDR_CALCULATE(base, mult)	(base + mult * CDMA_PAGE_SIZE)

#define CDMA_OP_SET(n)			(0x006 + (n) * 0x20)
#define CDMA_WITH_SET(n)		(0x007 + (n) * 0x20)
#define CDMA_RDBASE0_SET(n)		(0x008 + (n) * 0x20)
#define CDMA_RDBASE1_SET(n)		(0x009 + (n) * 0x20)
#define CDMA_RDBASE2_SET(n)		(0x00a + (n) * 0x20)
#define CDMA_RDBASE3_SET(n)		(0x00b + (n) * 0x20)
#define CDMA_WRBASE_SET(n)		(0x00c + (n) * 0x20)
#define CDMA_RDCOUNT0_SET(n)		(0x010 + (n) * 0x20)
#define CDMA_RDCOUNT1_SET(n)		(0x012 + (n) * 0x20)
#define CDMA_RDSTRIDE1_SET(n)		(0x013 + (n) * 0x20)
#define CDMA_WRCOUNT0_SET(n)		(0x018 + (n) * 0x20)
#define CDMA_WRCOUNT1_SET(n)		(0x019 + (n) * 0x20)
#define CDMA_WRSTRIDE1_SET(n)		(0x01a + (n) * 0x20)
#define CDMA_RDMASK_TLB(n)		(0x080 + (n) * 0x08)
#define CDMA_RDTAG_TLB(n)		(0x081 + (n) * 0x08)
#define CDMA_RDREAL_TLB(n)		(0x082 + (n) * 0x08)
#define CDMA_WRMASK_TLB(n)		(0x084 + (n) * 0x08)
#define CDMA_WRTAG_TLB(n)		(0x085 + (n) * 0x08)
#define CDMA_WRREAL_TLB(n)		(0x086 + (n) * 0x08)


#define CDMA_OP_SET0			0x006
#define CDMA_WITH_SET0			0x007
#define CDMA_RDBASE0_SET0		0x008
#define CDMA_RDBASE1_SET0		0x009
#define CDMA_RDBASE2_SET0		0x00a
#define CDMA_RDBASE3_SET0		0x00b
#define CDMA_WRBASE_SET0		0x00c
#define CDMA_RDCOUNT0_SET0		0x010
#define CDMA_RDCOUNT1_SET0		0x012
#define CDMA_RDSTRIDE1_SET0		0x013
#define CDMA_WRCOUNT0_SET0		0x018
#define CDMA_WRCOUNT1_SET0		0x019
#define CDMA_WRSTRIDE1_SET0		0x01a

#define CDMA_OP_SET1			0x027
#define CDMA_RDBASE0_SET1		0x028
#define CDMA_RDBASE1_SET1		0x029
#define CDMA_RDBASE2_SET1		0x02a
#define CDMA_RDBASE3_SET1		0x02b
#define CDMA_WRBASE_SET1		0x02c
#define CDMA_RDCOUNT0_SET1		0x030
#define CDMA_RDCOUNT1_SET1		0x032
#define CDMA_RDSTRIDE1_SET1		0x033
#define CDMA_WRCOUNT0_SET1		0x038
#define CDMA_WRCOUNT1_SET1		0x039
#define CDMA_WRSTRIDE1_SET1		0x03a

#define CDMA_OP_SET2			0x047
#define CDMA_RDBASE0_SET2		0x048
#define CDMA_RDBASE1_SET2		0x049
#define CDMA_RDBASE2_SET2		0x04a
#define CDMA_RDBASE3_SET2		0x04b
#define CDMA_WRBASE_SET2		0x04c
#define CDMA_RDCOUNT0_SET2		0x050
#define CDMA_RDCOUNT1_SET2		0x052
#define CDMA_RDSTRIDE1_SET2		0x053
#define CDMA_WRCOUNT0_SET2		0x058
#define CDMA_WRCOUNT1_SET2		0x059
#define CDMA_WRSTRIDE1_SET2		0x05a

#define CDMA_OP_SET3			0x067
#define CDMA_RDBASE0_SET3		0x068
#define CDMA_RDBASE1_SET3		0x069
#define CDMA_RDBASE2_SET3		0x06a
#define CDMA_RDBASE3_SET3		0x06b
#define CDMA_WRBASE_SET3		0x06c
#define CDMA_RDCOUNT0_SET3		0x070
#define CDMA_RDCOUNT1_SET3		0x072
#define CDMA_RDSTRIDE1_SET3		0x073
#define CDMA_WRCOUNT0_SET3		0x078
#define CDMA_WRCOUNT1_SET3		0x079
#define CDMA_WRSTRIDE1_SET3		0x07a

#define CDMA_RDMASK_TLB0		0x080
#define CDMA_RDTAG_TLB0			0x081
#define CDMA_RDREAL_TLB0		0x082
#define CDMA_WRMASK_TLB0		0x084
#define CDMA_WRTAG_TLB0			0x085
#define CDMA_WRREAL_TLB0		0x086

#define CDMA_RDMASK_TLB1		0x088
#define CDMA_RDTAG_TLB1			0x089
#define CDMA_RDREAL_TLB1		0x08a
#define CDMA_WRMASK_TLB1		0x08c
#define CDMA_WRTAG_TLB1			0x08d
#define CDMA_WRREAL_TLB1		0x08e

#define CDMA_RDMASK_TLB2		0x090
#define CDMA_RDTAG_TLB2			0x091
#define CDMA_RDREAL_TLB2		0x092
#define CDMA_WRMASK_TLB2		0x094
#define CDMA_WRTAG_TLB2			0x095
#define CDMA_WRREAL_TLB2		0x096

#define CDMA_RDMASK_TLB3		0x0a8
#define CDMA_RDTAG_TLB3			0x0a9
#define CDMA_RDREAL_TLB3		0x0aa
#define CDMA_WRMASK_TLB3		0x0ac
#define CDMA_WRTAG_TLB3			0x0ad
#define CDMA_WRREAL_TLB3		0x0ae

#define CDMA_A_CMD			0x105
#define CDMA_A_OP_SET0			0x106
#define CDMA_A_WITH_SET0		0x107
#define CDMA_A_RDBASE0_SET0		0x108
#define CDMA_A_RDBASE1_SET0		0x109
#define CDMA_A_RDBASE2_SET0		0x10a
#define CDMA_A_RDBASE3_SET0		0x10b
#define CDMA_A_WRBASE_SET0		0x10c
#define CDMA_A_RDCOUNT0_SET0		0x110
#define CDMA_A_RDCOUNT1_SET0		0x112
#define CDMA_A_RDSTRIDE1_SET0		0x112
#define CDMA_A_WRCOUNT0_SET0		0x118
#define CDMA_A_WRCOUNT1_SET0		0x119
#define CDMA_A_WRSTRIDE1_SET0		0x11a

#define CDMA_A_OP_SET1			0x127
#define CDMA_A_RDBASE0_SET1		0x128
#define CDMA_A_RDBASE1_SET1		0x129
#define CDMA_A_RDBASE2_SET1		0x12a
#define CDMA_A_RDBASE3_SET1		0x12b
#define CDMA_A_WRBASE_SET1		0x12c
#define CDMA_A_RDCOUNT0_SET1		0x130
#define CDMA_A_RDCOUNT1_SET1		0x132
#define CDMA_A_RDSTRIDE1_SET1		0x133
#define CDMA_A_WRCOUNT0_SET1		0x138
#define CDMA_A_WRCOUNT1_SET1		0x139
#define CDMA_A_WRSTRIDE1_SET1		0x13a

#define CDMA_A_OP_SET2			0x147
#define CDMA_A_RDBASE0_SET2		0x148
#define CDMA_A_RDBASE1_SET2		0x149
#define CDMA_A_RDBASE2_SET2		0x14a
#define CDMA_A_RDBASE3_SET2		0x14b
#define CDMA_A_WRBASE_SET2		0x14c
#define CDMA_A_RDCOUNT0_SET2		0x150
#define CDMA_A_RDCOUNT1_SET2		0x152
#define CDMA_A_RDSTRIDE1_SET2		0x153
#define CDMA_A_WRCOUNT0_SET2		0x158
#define CDMA_A_WRCOUNT1_SET2		0x159
#define CDMA_A_WRSTRIDE1_SET2		0x15a

#define CDMA_A_OP_SET3			0x167
#define CDMA_A_RDBASE0_SET3		0x168
#define CDMA_A_RDBASE1_SET3		0x169
#define CDMA_A_RDBASE2_SET3		0x16a
#define CDMA_A_RDBASE3_SET3		0x16b
#define CDMA_A_WRBASE_SET3		0x16c
#define CDMA_A_RDCOUNT0_SET3		0x170
#define CDMA_A_RDCOUNT1_SET3		0x172
#define CDMA_A_RDSTRIDE1_SET3		0x173
#define CDMA_A_WRCOUNT0_SET3		0x178
#define CDMA_A_WRCOUNT1_SET3		0x179
#define CDMA_A_WRSTRIDE1_SET3		0x17a

/* status register offset */
#define CDMA_STATUS_A			(0x1 << 0)
#define CDMA_STATUS_D			(0x1 << 1)
#define CDMA_STATUS_W			(0x1 << 2)
#define CDMA_STATUS_P			(0x1 << 3)
#define CDMA_STATUS_R			(0x1 << 4)
#define CDMA_STATUS_E			(0x1 << 5)
#define CDMA_STATUS_CODE		(0x3 << 6)
#define CDMA_STATUS_OLD			(0x3 << 8)
/*sctl register offset*/
#define CDMA_SCTL_VIRT			(0x01 << 0)
#define CDMA_SCTL_RL2			(0x01 << 1)
#define CDMA_SCTL_SYNC			(0x01 << 2)
/* async register offset */
#define CDMA_ASYNC_RESET_OFF		(0x4)
#define CDMA_ASYNC_PAUSE_OFF		(0x3)
#define CDMA_ASYNC_CLEAR_OFF		(0x2)
#define CDMA_ASYNC_RESUM_OFF		(0x1)
/* cmd register offset */
#define CDMA_CMD_ACTIVE			(0x1 << 0)
#define CDMA_CMD_SET0			(0x0 << 5)
#define CDMA_CMD_SET0_EN		(0x1 << 4)
#define CDMA_CMD_SET1			(0x1 << 9)
#define CDMA_CMD_SET1_EN		(0x1 << 8)
#define CDMA_CMD_SET2			(0x2 << 13)
#define CDMA_CMD_SET2_EN		(0x1 << 12)
#define CDMA_CMD_SET3			(0x3 << 17)
#define CDMA_CMD_SET3_EN		(0x1 << 16)
/* tlb mask register offset */
#define CDMA_TLBMASK_EN			(0x1 << 0)
#define CDMA_TLBMASK_L2			(0x1 << 1)
#define CDMA_TLBMASK_SYNC		(0x1 << 2)
#define CDMA_TLBMASK_MK(n)		((n) & (~0x7))

#define NSETS_DMA   4
#define BASES_DMA   4
#define DEPTH_DMA   2
#define NRDREQS_DMA 4
#define NBUFS_DMA   5
#define WITH_DMA    8
#define RDTLBS_DMA  4
#define WRTLBS_DMA  4

#define ACTIVE_CTL			(1)
#define REGSET0_CTL(v__)		((((v__)&0x3)<<5)|1<<4)
#define REGSET1_CTL(v__)		((((v__)&0x3)<<9)|(1<<8))
#define REGSET2_CTL(v__)		((((v__)&0x3)<<13)|(1<<12))
#define REGSET3_CTL(v__)		((((v__)&0x3)<<17)|(1<<16))

/* op_set register offset */
#define CDMA_OP_TY_COPY			((0x0 & 0x3) << 0)
#define CDMA_OP_TY_FILL			((0x1 & 0x3) << 0)
#define CDMA_OP_TY_ZERO_PAD		((0x2 & 0x3) << 0)
#define CDMA_OP_TY_COPY_PAD		((0x3 & 0x3) << 0)
#define CDMA_OP_RB_NUM0			((0x0 & 0x3) << 8)
#define CDMA_OP_RB_NUM1			((0x1 & 0x3) << 8)
#define CDMA_OP_RB_NUM2			((0x2 & 0x3) << 8)
#define CDMA_OP_RB_NUM3			((0x3 & 0x3) << 8)
#define CDMA_OP_SZ_1B			((0x0 & 0x3) << 12)
#define CDMA_OP_SZ_2B			((0x1 & 0x3) << 12)
#define CDMA_OP_SZ_4B			((0x2 & 0x3) << 12)
#define CDMA_OP_SZ_8B			((0x3 & 0x3) << 12)
#define CDMA_OP_CN_1B			((0x0 & 0xF) << 14)
#define CDMA_OP_CN_2B			((0x1 & 0xF) << 14)
#define CDMA_OP_CN_4B			((0x3 & 0xF) << 14)
#define CDMA_OP_CN_8B			((0x7 & 0xF) << 14)
#define CDMA_OP_RD_MULTIPLE		(0x1 << 4)
#define CDMA_OP_WR_MULTIPLE		(0x1 << 6)


static int cdma_major = CPU_DMA_MAJOR;
uint64_t *reg_base;
struct device dev;
struct cdev cdev;

struct dma_addr_desc *dma_addr_desc;

static void cdma_enable(uint64_t base_addr)
{
	uint64_t tag = base_addr | PXI_EN;
	uint64_t mask = CPU_DMA_MASK;

	sprset_gpt(DMATAG, tag);
	sprset_gpt(DMAMASK, mask);
}

static void cdma_get_spr(void)
{
	printk("DMATAG: %lx\n", sprget_gpt(DMATAG));
	printk("DMAMASK: %lx\n", sprget_gpt(DMAMASK));
}

uint64_t cdma_writeq(uint64_t *addr, uint64_t value)
{
	__asm__ volatile (
		"stl	%0, %1; barrier;\n\t"
		:
		: "r"(value), "a"(addr)
		:
	);
	return value;
}

uint64_t cdma_readq(uint64_t *addr)
{
	uint64_t value;

	__asm__ volatile (
		"ldl	%0,%1;	barrier;\n\t"
		: "=r"(value)
		: "a"(addr)
	);

	return value;
}

void cdma_reset(void)
{
	uint64_t status;

	cdma_writeq(CDMA_REG(CDMA_ASYNC), CDMA_ASYNC_RESET_OFF);
	do {
		status = cdma_readq(CDMA_REG(CDMA_STATUS));
	} while(!(status & CDMA_STATUS_R));

	cdma_writeq(CDMA_REG(CDMA_CMD), 0x0);
	do {
		status = cdma_readq(CDMA_REG(CDMA_STATUS));
	} while((status & CDMA_STATUS_W));

	cdma_writeq(CDMA_REG(CDMA_ASYNC), CDMA_ASYNC_CLEAR_OFF);
	do {
		status = cdma_readq(CDMA_REG(CDMA_STATUS));
	} while((status & CDMA_STATUS_W));

}

static void cdma_alloc(struct dma_addr_desc *desc)
{
	desc->cpu_addr = dma_alloc_writecombine(&dev, desc->len,
				&desc->phy_addr, GFP_KERNEL);
	if(!desc->cpu_addr)
		printk("cpu-dma alloc failed\n");
}

static void cdma_free(struct dma_addr_desc *desc)
{
	dma_free_writecombine(&dev, desc->len, desc->cpu_addr,
					desc->phy_addr);
}

int cdma_mmap(struct file *filp, struct vm_area_struct *vma)
{
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	dma_addr_desc->user_virt = vm_iomap_memory(vma,
				dma_addr_desc->phy_addr, dma_addr_desc->len);
	return dma_addr_desc->user_virt;
}

void cdma_rd_setx(int setn, dma_addr_t src0, dma_addr_t src1, dma_addr_t src2,
			dma_addr_t src3, uint64_t columns, uint64_t rows,
							uint64_t rstr)
{
	cdma_writeq(CDMA_REG(CDMA_RDBASE0_SET(setn)), src0);
	cdma_writeq(CDMA_REG(CDMA_RDBASE1_SET(setn)), src1);
	cdma_writeq(CDMA_REG(CDMA_RDBASE2_SET(setn)), src2);
	cdma_writeq(CDMA_REG(CDMA_RDBASE3_SET(setn)), src3);
	cdma_writeq(CDMA_REG(CDMA_RDCOUNT0_SET(setn)), columns - 1);
	cdma_writeq(CDMA_REG(CDMA_RDCOUNT1_SET(setn)), rows - 1);
	cdma_writeq(CDMA_REG(CDMA_RDSTRIDE1_SET(setn)), rstr);
}

void cdma_wr_setx(int setn, uint64_t columns, uint64_t rows,
				uint64_t stride, dma_addr_t dst)
{
	cdma_writeq(CDMA_REG(CDMA_WRBASE_SET(setn)), dst);
	cdma_writeq(CDMA_REG(CDMA_WRCOUNT0_SET(setn)), columns - 1);
	cdma_writeq(CDMA_REG(CDMA_WRCOUNT1_SET(setn)), rows - 1);
	cdma_writeq(CDMA_REG(CDMA_WRSTRIDE1_SET(setn)), stride);
}

int cdma_setn_op(int setn, uint64_t mode)
{
	if(setn > 4 || setn < 0) {
		printk("1 < nset < 4\n");
		return -EINVAL;
	}

	cdma_writeq(CDMA_REG(CDMA_OP_SET(setn)), mode);

	return 0;
}

static int cdma_xset_xbase(int setx, int basex, int mode, dma_addr_t r0,
			dma_addr_t r1, dma_addr_t r2, dma_addr_t r3, int rstr,
			uint64_t rcol, int rrow, dma_addr_t d0, int dstr,
						int dcol, int drow)
{
	cdma_rd_setx(setx, r0, r1, r2, r3, rcol, rrow, rstr);
	cdma_wr_setx(setx, dcol, drow, dstr, d0);
	cdma_setn_op(setx, mode);
	return 0;
}

static int cdma_cmd_enable(int nset)
{
	uint64_t reg = 0;

	switch (nset) {
	case 3:
		reg |= (CDMA_CMD_SET3_EN | CDMA_CMD_SET3);
	case 2:
		reg |= (CDMA_CMD_SET2_EN | CDMA_CMD_SET2);
	case 1:
		reg |= (CDMA_CMD_SET1_EN | CDMA_CMD_SET1);
	case 0:
		reg |= (CDMA_CMD_SET0_EN | CDMA_CMD_SET0);
		break;
	}

	reg |= CDMA_CMD_ACTIVE;
	cdma_writeq(CDMA_REG(CDMA_CMD), reg);

	return 0;
}

uint64_t cdma_nbase_cfg(int n)
{
	uint64_t mode = 0;

	switch (n) {
	case 1:
		mode = CDMA_OP_RB_NUM0;
		break;
	case 2:
		mode = CDMA_OP_RB_NUM1;
		break;
	case 3:
		mode = CDMA_OP_RB_NUM2;
		break;
	case 4:
		mode = CDMA_OP_RB_NUM3;
		break;
	default:
		mode = CDMA_OP_RB_NUM0;
		printk("use defaule READ BASE\n");
		break;
	}

	return mode;
}

int cdma_wait_done(void)
{
	int timeout = CDMA_TIMEOUT;
	uint64_t status;

	do {
		status = cdma_readq(CDMA_REG(CDMA_STATUS));
		if(status & CDMA_STATUS_E) {
			printk("error status: %llxx, mis address: %llx\n",
				status & CDMA_STATUS_CODE,
					cdma_readq(CDMA_REG(CDMA_MISS)));
		}
		timeout--;
	} while((!(status & CDMA_STATUS_D)) && (timeout > 0));

	if(timeout <= 0) {
		printk("cdma wait timeout\n");
		return -1;
	}

	return 0;
}

int cdma_phys_copy(dma_addr_t dst, dma_addr_t src, int length)
{
	int len;
	int count;
	int nset = 1;
	int rstr = 0x0;
	int dstr = 0x0;
	int row = 0x1;
	int nbase = 0x1;
	int num = length / CDMA_PAGE_SIZE;
	int offset = length % CDMA_PAGE_SIZE;
	uint64_t mode = CDMA_OP_TY_COPY | cdma_nbase_cfg(nbase);

	num += (offset > 0) ? 1 : 0;
	len = CDMA_PAGE_SIZE;
	for( count = 0; count < num; count++) {
		if((offset > 0) && (count == (num - 1)))
			len = offset;
		cdma_xset_xbase(0, nbase, mode, ADDR_CALCULATE(src, count), 0,
				0, 0, rstr, len, row, ADDR_CALCULATE(dst, count),
								dstr, len, row);
		cdma_cmd_enable(nset - 1);
		cdma_wait_done();
	}

	return 0;
}

static int cdma_copy(struct cdma_desc *desc)
{
	switch (desc->cmd) {
		case CDMA_PHYS_1D_1D:
			cdma_phys_copy(desc->dst->phy_addr,
					desc->src->phy_addr, desc->len);
			break;

		default:
			printk("Unkown cpu-dma ops\n");
			break;
	}

	return 0;
}

static long cdma_ioctl(struct file *filp, unsigned int cmd,
					unsigned long arg)
{
	struct cdma_desc desc;

	switch (cmd) {
		case CDMA_ADDR_ALLOC:
			copy_from_user(dma_addr_desc,
					(struct dma_addr_desc *)arg,
						sizeof(struct dma_addr_desc));
			cdma_alloc(dma_addr_desc);
			copy_to_user((struct dma_addr_desc *)arg,
				dma_addr_desc, sizeof(struct dma_addr_desc));
			break;

		case CDMA_ADDR_FREE:
			copy_from_user(dma_addr_desc,
					(struct dma_addr_desc *)arg,
						sizeof(struct dma_addr_desc));
			cdma_free(dma_addr_desc);
			break;

		case CDMA_ADDR_MMAP:
			copy_from_user(dma_addr_desc,
					(struct dma_addr_desc *)arg,
						sizeof(struct dma_addr_desc));
			break;

		case CDMA_COPY:
			copy_from_user(&desc, (struct cdma_desc *)arg,
						sizeof(struct dma_addr_desc));
			cdma_copy(&desc);
			break;
	}

	return 0;
}


static const struct file_operations cdma_fops = {
	.owner = 0,
	.unlocked_ioctl = cdma_ioctl,
	.mmap = cdma_mmap,
};

static dev_t devno;
static void cdma_setup_cdev(void)
{
	int err;

	cdev_init(&cdev, &cdma_fops);
	cdev.owner = 0;
	err = cdev_add(&cdev, devno, 1);
	if(err)
		dev_err(&dev, "Add cpu-dma error\n");
}

static struct class *cdma_class;

static int cdma_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct resource *res;

	devno = MKDEV(cdma_major, 0);

	if(cdma_major) {
		ret = register_chrdev_region(devno, 1, "cpu-dma");
	} else {
		ret = alloc_chrdev_region(&devno, 0, 1, "cpu-dma");
		cdma_major = MAJOR(devno);
	}

	if(ret < 0){
		dev_err(&pdev->dev, "alloc chrdev region failed\n");
		return ret;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "core0");
	if(res == NULL) {
		dev_err(&pdev->dev, "get cpu dma resource failed\n");
		goto fail_alloc;
	}
	reg_base = ioremap_nocache(res->start, resource_size(res));
	if(reg_base == NULL) {
		dev_err(&pdev->dev, "ioremap failed\n");
		goto fail_alloc;
	}
	dma_addr_desc = kzalloc(sizeof(struct dma_addr_desc), GFP_KERNEL);
	if(!dma_addr_desc)
		goto fail_alloc;
	dev = pdev->dev;
	cdma_enable(res->start);
	cdma_get_spr();
	cdma_reset();
	cdma_setup_cdev();
	cdma_class = class_create(THIS_MODULE, "cpu-dma");
	device_create(cdma_class, NULL, devno, NULL, "cpu-dma");
	printk("cpu dma character device register\n");
	return 0;

fail_alloc:
	unregister_chrdev_region(devno,1);
	return -EINVAL;
}

static int cdma_remove(struct platform_device *pdev)
{
	kfree(dma_addr_desc);
	device_destroy(cdma_class, MKDEV(cdma_major, 0));
	class_destroy(cdma_class);
	cdev_del(&cdev);
	iounmap((void *)reg_base);
	unregister_chrdev_region(MKDEV(cdma_major, 0), 1);

	return 0;
}

static const struct of_device_id cdma_match[] = {
	{ .compatible = "gpt,cpu-dma"},
	{},
};

static struct platform_driver cdma_driver = {
	.driver = {
		.name = "cpu-dma",
		.owner = 0,
		.of_match_table = of_match_ptr(cdma_match),
	},
	.probe = cdma_probe,
	.remove = cdma_remove,
};

static int __init cdma_driver_init(void)
{
	return platform_driver_register(&cdma_driver);
}

static void __exit cdma_driver_exit(void)
{
	platform_driver_unregister(&cdma_driver);
}

module_init(cdma_driver_init);
module_exit(cdma_driver_exit);

MODULE_DESCRIPTION("GPT cdma Controller driver");
MODULE_LICENSE("GPL v2");
