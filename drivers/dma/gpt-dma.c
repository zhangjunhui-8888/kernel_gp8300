/*
 * GPT DMA handling
 *
 * based on s3c24xx-dma.c
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * The DMA controllers in gpt SoCs have a varying number of DMA signals
 * that can be routed to any of the 4 to 8 hardware-channels.
 *
 * Therefore on these DMA controllers the number of channels
 * and the number of incoming DMA signals are two totally different things.
 * It is usually not possible to theoretically handle all physical signals,
 * so a multiplexing scheme with possible denial of use is necessary.
 *
 * Open items:
 * - bursts
 */

#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/dmaengine.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/of_dma.h>
#include <linux/syscalls.h>
#include <asm/cacheflush.h>
#include <asm/cache.h>
#include <asm/spr.h>
#include <linux/mm.h>
#include <linux/cpu.h>
#include <asm/mach-chip2/dma.h>
#include <asm/mach-chip2/sram.h>
#include <linux/delay.h>
#include <linux/percpu-defs.h>
#include <linux/percpu.h>
#include "dmaengine.h"
#include "virt-dma.h"
#include "gpt-dma.h"
#include <linux/string.h>

/*
 *if want to open DMA IRQ
 *(1) arch/gpt/include/asm/irqflags: add "IX_GPTDMA" in "#define DEFAULT_XEN" 
 *(2) arch/gpt/kernel/entry.S:  
    modify "ix_entry IX_SGI_0, (IX_GPTDMA - 1), hyper_do_irq" -> "ix_entry IX_SGI_0, IX_GPTDMA, hyper_do_irq";
    "ix_entry IX_GPTDMA, (IX_XPEN_MAX - 1), hyper_do_default" -> "ix_entry (IX_GPTDMA + 1), (IX_XPEN_MAX - 1), hyper_do_default"
 *(3)arch/gpt/kernel/entry.S: 
    in the last of "ENTRY(ixh_wake_up_vectors)" add "ix_wake_up_entry IX_GPTDMA, IX_GPTDMA, hyper_do_irq"
    in "ENTRY(ix_wake_up_\handler\@)" modify ".if (\from >= IX_TIMER_BASE && \from <= IX_TIMER_COUNT1) || (\from >= IX_SGI_0 && \from <= IX_GPTDMA)"
 *(4)open GPT_DMA_IRQ_EN
 */

#define NUM_PHY_CHANNELS    4 
#define GPT_DMA_IRQ_EN
static struct gpt_cpu_dma __percpu *cpu_dma;

uint64_t  *reg_base;
uint64_t phy_addr;
u8 irq_cdma;

struct gpt_cpu_dma {
	struct dma_async_tx_descriptor *tx;
	u64 dma_status;
	bool cpu_dma_valid;
	spinlock_t  lock;
};

struct gpt_dma_parameter {
	u8 copy_mode;
	u64 src_virt[4];
	u64 dst_virt[4];
	struct gpt_opset_para *opset[4];
	struct gpt_data_info *data_info[4];
};

struct gpt_opset_para {
	u8 type; 
	u8 rddepth;
	u8 wrdepth; 
	u8 rdbases; 
	u8 count;
	u8 size;
	u64 fill_data;
};

struct gpt_data_info {
	u8 data_mode;
	u16 src_rows;
	u16 src_columns;
	u32 src_stride;
	u16 dst_rows;
	u16 dst_columns;
	u32 dst_stride;
};

/*
 * struct gpt_sg - structure containing data per sg (the information of data for transfer)
 * struct gpt_sg will add list of struct gpt_txd
 * @src_addr: src address of sg
 * @dst_addr: dst address of sg
 * len: transfer len in bytes
 * @node: node for txd's dsg_list
 */
struct gpt_sg {
	dma_addr_t src_addr;
	dma_addr_t dst_addr;
	size_t len;
	struct list_head node;
};

/*
 * struct gpt_txd - wrapper for struct dma_async_tx_descriptor
 * @vd: virtual DMA descriptor
 * @dsg_list: list of children sg's
 * @at: sg currently being transfered
 * @cyclic: indicate cyclic transfer
 */
struct gpt_txd {
	struct virt_dma_desc vd;
	struct list_head dsg_list;
	struct list_head *at;
	u8 dst_nents;
	u8 src_nents;
	bool cyclic;
};

/*
 * struct gpt_dma_phy - holder for the physical channels
 * @id: physical index to this channel
 * @avail: whether the channel is available
 * @lock: a lock to use when altering an instance of this struct
 */
struct gpt_dma_phy {
	unsigned int id;
	bool  avail;
};

/*
 * struct gpt_dma_chan - this structure wraps a DMA ENGINE channel
 * @name: name of the channel
 * @vc: wrappped virtual channel
 * @phy: the physical channel utilized by this channel, if there is one
 * @at: active transaction on this channel
 * @host: a pointer to the host (internal use)
 * @state: whether the channel is idle, running etc
 */
struct gpt_dma_chan {
	const char *name;
	struct virt_dma_chan vc;
	struct gpt_dma_phy *phy[4];
	struct dma_slave_config cfg;
	struct gpt_txd *at;
	struct gpt_dma_engine *host;
	struct gpt_dma_parameter *para;
	struct gpt_dma_wtlb *gpt_wtlb[4];
	struct gpt_dma_rtlb *gpt_rtlb[4];
	u64 dma_status;
	int cpu_id;
};

struct gpt_dma_wtlb {
	u8 id;
	bool avail;
};

struct gpt_dma_rtlb {
	u8 id;
	bool avail;
};

/*
 * struct gpt_dma_engine - the local state holder for the gpt
 * @pdev: the corresponding platform device
 * @memcpy: memcpy engine for this instance
 * @phy_chans: array of data for the physical channels
 * @channels: array of virtual channel descriptions
 */
struct gpt_dma_engine {
	struct platform_device			*pdev;
	struct dma_device			memcpy;
};

static long get_timeus(void)
{
	struct timeval tv;

	do_gettimeofday(&tv);
	return tv.tv_sec * 1000000 + tv.tv_usec;
}

uint64_t gpt_cdma_read64(uint64_t *addr)
{
	uint64_t value;
	__asm__ volatile (
			"ldl     %0,%1; barrier;\n\t"
			: "=r"(value)
			: "a"(addr)
			:
			);
	return value;
}


static void cdma_enable(uint64_t base_addr)
{
	uint64_t tag = base_addr | GPT_PXI_EN;
	uint64_t mask = GPT_CPU_DMA_MASK;
	sprset_gpt(DMATAG, tag);
	sprset_gpt(DMAMASK, mask);
}

uint64_t gpt_cdma_write64(uint64_t *addr, uint64_t value)
{
	__asm__ volatile (
			"stl     %0, %1; barrier;\n\t"
			:
			: "r"(value), "a"(addr)
			:
			);

	return  value;
}

static void gpt_cdma_get_spr(void)
{
	printk("DMATAG: %lx\n", sprget_gpt(DMATAG));
	printk("DMAMASK: %lx\n", sprget_gpt(DMAMASK));
}

/*
 * Virtual channel handling
 */
static inline struct gpt_dma_chan *to_gpt_dma_chan(struct dma_chan *chan)
{
	return container_of(chan, struct gpt_dma_chan, vc.chan);
}

/*
 *reset DMA 
 */
static void gpt_cdma_reset(void)
{
	u64 status = 0;
	/*
	 *reset the DMA. When this command is executed, the STATUS register RESET bit is set to 1. 
	 *The WAIT bit may be 1 till the DMA quiesces. 
	 *The DMA will start executing the next command only after the ASYNC register is written with a CLEAR command
	 */
	gpt_cdma_write64(reg_base + GPT_CDMA_ASYNC, GPT_ASYNC_RESET);
	do {
		status = gpt_cdma_read64(reg_base + GPT_CDMA_STATUS);
	} while (status & GPT_STATUS_W);

	gpt_cdma_write64(reg_base + GPT_CDMA_ASYNC, GPT_ASYNC_CLEAR);
	do {
		status = gpt_cdma_read64(reg_base + GPT_CDMA_STATUS);
	} while (status & GPT_STATUS_W);

}

int gpt_cdma_group_para(struct dma_chan *chan, u8 group_id, u16 src_columns, u16 src_rows, u32 src_stride,
		u16 dst_columns, u16 dst_rows, u32 dst_stride)
{
	struct gpt_dma_chan *gptchan = to_gpt_dma_chan(chan);
	struct gpt_dma_parameter *para = gptchan->para;

	if (group_id > 3) {
		printk("Invalid argument for group_id: 0/1/2/3\n");
		return -EINVAL;
	}

	para->data_info[group_id]->src_columns = src_columns;
	para->data_info[group_id]->src_rows = src_rows;
	para->data_info[group_id]->src_stride = src_stride;
	para->data_info[group_id]->dst_columns = dst_columns;
	para->data_info[group_id]->dst_rows = dst_rows;
	para->data_info[group_id]->dst_stride = dst_stride;

	return 0;
}
EXPORT_SYMBOL_GPL(gpt_cdma_group_para);

int gpt_cdma_virt(struct dma_chan *chan, u8 virt_id, u64 src_virt, u64 dst_virt)
{
	struct gpt_dma_chan *gptchan = to_gpt_dma_chan(chan);
	struct gpt_dma_parameter *para = gptchan->para;

	if (virt_id > 3) {
		printk("Invalid argument for virt_id: 0/1/2/3\n");
		return -EINVAL;
	}

	para->src_virt[virt_id] = src_virt;
	para->dst_virt[virt_id] = dst_virt;

	return 0;
}
EXPORT_SYMBOL_GPL(gpt_cdma_virt);

/*
 *set the 64-bit base address for READ stream s; size of raw, columns and stride 
 *RDBASEs_SETm (Read Base, Stream s, Register Set m)
 *RDCOUNTd_SETm (Read Count, Depth d, Register Set m): d-0:columns   d-1:raw
 *RDSTRIDEd_SETm (Read Count, Depth d, Register Set m): RDSTRIDE0 is always 1   d-0:columns   d-1:raw
 *setn: number of set
 *src0: base0 address
 *src1: base1 address
 *src2: base2 address
 *src3: base3 address
 *rows: if 1d to 1d rows is count0, if multiple d rows is number of rows
 *columns: number of columns
 *rstr: stride of raw
 */
static void gpt_cdma_rd_setx(int setn, uint64_t columns, uint64_t rows, uint64_t rstr)
{
	gpt_cdma_write64(reg_base + GPT_CDMA_RDCOUNT0_SET(setn), columns - 1);
	gpt_cdma_write64(reg_base + GPT_CDMA_RDCOUNT1_SET(setn), rows - 1);
	gpt_cdma_write64(reg_base + GPT_CDMA_RDSTRIDE1_SET(setn), rstr);
}

/*
 *set the 64-bit base address for WRITE stream s;
 *RDBASE(0/1/2/3)_SET --> WRBASE_SET
 */
static void gpt_cdma_wr_setx(int setn, uint64_t columns,
		uint64_t rows, uint64_t stride, uint64_t dst)
{
	gpt_cdma_write64(reg_base + GPT_CDMA_WRCOUNT0_SET(setn), columns - 1);
	gpt_cdma_write64(reg_base + GPT_CDMA_WRCOUNT1_SET(setn), rows - 1);
	gpt_cdma_write64(reg_base + GPT_CDMA_WRSTRIDE1_SET(setn), stride);
	gpt_cdma_write64(reg_base + GPT_CDMA_WRBASE_SET(setn), dst);
}

/*
 *get parameter of OP_SETm register: used to specify the DMA operation to be executed
 *@type: the type of operation to be executed
 *	COPY (=0): copy data from input to output
 *	FILLL (=1): fill the output with the contents of the WITH register
 *	ZERO_PAD(=2): copy data from the input to output padding the output with zeroes before and after each output row
 *	COPY_PAD(=3): copy data from the input to output padding the output with first/last element in row before and after each output row
 *@rddepth: controls the depth of iteration for reads
 *  0: only one row is read   1: multiple rows are read
 *@wrdepth: controls the depth of iteration for writes
 *  0: only one row is write   1: multiple rows are write
 *@rdbases: controls the number of read streams,The TYPE must be COPY;The output is the interleaved elements of the streams
 *@size: This field can be 0,1,2,3 for 1B,2B,4B or 8B values
 *@count: this is 1 less than the number of bytes to be padded
 */
int gpt_cdma_opsetn_mode(struct dma_chan *chan, u8 setn, u8 type, u8 rddepth, u8 wrdepth, u8 rdbases, u8 size, u8 count)
{
	struct gpt_dma_chan *gptchan = to_gpt_dma_chan(chan);
	struct gpt_dma_parameter *para = gptchan->para;

	if (setn > 3) {
		printk("Invalid argument! set: 0 - 3\n");
		return -EINVAL;
	}

	if (type > 3) {
		printk("Invalid argument! type: 0 - 3\n");
		return -EINVAL;
	} else {
		para->opset[setn]->type = type;	
	}

	if (rddepth > 1) {
		printk("Invalid argument! rddepth: 0/1\n");
		return -EINVAL;
	} else {
		para->opset[setn]->rddepth = rddepth;	
	}

	if (wrdepth > 1) {
		printk("Invalid argument! wrdepth: 0/1\n");
		return -EINVAL;
	} else {
		para->opset[setn]->wrdepth = wrdepth;	
	}

	if (rdbases > 4) {
		printk("Invalid argument! rdbases: 1 - 4\n");
		return -EINVAL;
	} else {
		para->opset[setn]->rdbases = rdbases;	
	}

	if ((size == 1) || (size == 2) || (size == 4) || (size == 8)) {
		para->opset[setn]->size = size;
	} else {
		printk("Invalid argument! size: 1/2/4/8\n");
		return -EINVAL;
	}

	if ((count > 4) || (count == 3)) {
		printk("Invalid argument! count: 1/2/4\n");
		return -EINVAL;
	} else {
		para->opset[setn]->count = count;	
	}

	return 0;
}
EXPORT_SYMBOL_GPL(gpt_cdma_opsetn_mode);

/*
 *set OP_SETm register: used to specify the DMA operation to be executed
 *@setn: real channel id: 0-3
 *@set_id: user set channel id: 0-3
 */
static int gpt_cdma_setn_op(u8 setn, u8 set_id, struct gpt_dma_parameter *para)
{
	u64 value = 0;
	u8 type = para->opset[set_id]->type; 
	u8 rddepth = para->opset[set_id]->rddepth;
	u8 wrdepth = para->opset[set_id]->wrdepth;
	u8 rdbases = para->opset[set_id]->rdbases;
	u8 size = para->opset[set_id]->size;
	u8 count = para->opset[set_id]->count;

	if (setn > 3 || set_id > 3) {
		printk("Invalid argument! setn: 0 - 3\n");
		return -EINVAL;
	}

	switch (type) {
		case 0:
			value |= GPT_OP_TY_COPY;
			break;
		case 1:
			value |= GPT_OP_TY_FILL;
			break;
		case 2:
			value |= GPT_OP_TY_ZERO_PAD;
			break;
		case 3:
			value |= GPT_OP_TY_COPY_PAD;
			break;
	}

	switch (rddepth) {
		case 0:
			value |= GPT_OP_RD_SINGAL;
			break;
		case 1:
			value |= GPT_OP_RD_MULTIPLE;
			break;
	}

	switch (wrdepth) {
		case 0:
			value |= GPT_OP_WR_SINGAL;
			break;
		case 1:
			value |= GPT_OP_WR_MULTIPLE;
			break;
	}

	switch (rdbases) {
		case 1:
			value |= GPT_OP_RB_NUM1;
			break;
		case 2:
			value |= GPT_OP_RB_NUM2;
			break;
		case 3:
			value |= GPT_OP_RB_NUM3;
			break;
		case 4:
			value |= GPT_OP_RB_NUM4;
			break;
	}

	switch (size) {
		case 1:
			value |= GPT_OP_SZ_1B;
			break;
		case 2:
			value |= GPT_OP_SZ_2B;
			break;
		case 4:
			value |= GPT_OP_SZ_4B;
			break;
		case 8:
			value |= GPT_OP_SZ_8B;
			break;
	}

	switch (count) {
		case 1:
			value |= GPT_OP_CN_1B;
			break;
		case 2:
			value |= GPT_OP_CN_2B;
			break;
		case 4:
			value |= GPT_OP_CN_4B;
			break;
	}

	gpt_cdma_write64(reg_base + GPT_CDMA_OP_SET(setn), value);

	return 0;
}

/*
 *if type is FILL, fill the WITH register
 */
int gpt_cdma_fill_withsetn(struct dma_chan *chan, u8 setn, u64 data)
{
	struct gpt_dma_chan *gptchan = to_gpt_dma_chan(chan);
	struct gpt_dma_parameter *para = gptchan->para;

	if (sizeof(data) != 8) {
		printk("Invalid argument! data must be 64 bit!\n");
		return -EINVAL;
	}
	para->opset[setn]->fill_data = data;

	return 0;
}
EXPORT_SYMBOL_GPL(gpt_cdma_fill_withsetn);

/*
 *The SCTL register is used to control functionality of the DMA that is reserved for the super-user
 */
void gpt_cdma_tlb_en(void)
{
	u64 value = 0;

	value = gpt_cdma_read64(reg_base + GPT_CDMA_SCTL);
	value |= GPT_SCTL_VIRT;
	gpt_cdma_write64(reg_base + GPT_CDMA_SCTL, value);
}
EXPORT_SYMBOL_GPL(gpt_cdma_tlb_en);

void gpt_cdma_tlb_dis(void)
{
	u64 value = 0;

	value = gpt_cdma_read64(reg_base + GPT_CDMA_SCTL) & 0x6;
	gpt_cdma_write64(reg_base + GPT_CDMA_SCTL, value);	
}
EXPORT_SYMBOL_GPL(gpt_cdma_tlb_dis);

void gpt_cdma_l2_en(void)
{
	u64 value = 0;

	value = gpt_cdma_read64(reg_base + GPT_CDMA_SCTL);
	value |= GPT_TLBMASK_L2 | GPT_TLBMASK_SYNC;
	gpt_cdma_write64(reg_base + GPT_CDMA_SCTL, value);
}
EXPORT_SYMBOL_GPL(gpt_cdma_l2_en);

void gpt_cdma_l2_dis(void)
{
	u64 value = 0;

	value = gpt_cdma_read64(reg_base + GPT_CDMA_SCTL) & 0x1;
	gpt_cdma_write64(reg_base + GPT_CDMA_SCTL, value);	
}
EXPORT_SYMBOL_GPL(gpt_cdma_l2_dis);

static void gpt_cdma_tlb_r(int item, u64 phy, u64 virt, u64 mask)
{
	gpt_cdma_write64(reg_base + GPT_CDMA_RDREAL_TLB(item), phy);
	gpt_cdma_write64(reg_base + GPT_CDMA_RDMASK_TLB(item), mask);
	gpt_cdma_write64(reg_base + GPT_CDMA_RDTAG_TLB(item), virt & (~mask));
}

static void gpt_cdma_tlb_w(int item, u64 phy, u64 virt, u64 mask)
{
	gpt_cdma_write64(reg_base + GPT_CDMA_WRREAL_TLB(item), phy);
	gpt_cdma_write64(reg_base + GPT_CDMA_WRMASK_TLB(item), mask);
	gpt_cdma_write64(reg_base + GPT_CDMA_WRTAG_TLB(item), virt & (~mask));
}

static int gpt_cdma_get_wtlb(struct gpt_dma_chan *gptchan)
{
	u8 i = 0;

	for (i = 0; i < 4; i ++) {
		if (gptchan->gpt_wtlb[i]->avail == true)
			return i;
	}

	if (i == 4) {
		printk("get wtlb failed!\n");
		return -EBUSY;
	}

	return 0;
}

static int gpt_cdma_get_rtlb(struct gpt_dma_chan *gptchan)
{
	u8 i = 0;

	for (i = 0; i < 4; i ++) {
		if (gptchan->gpt_rtlb[i]->avail == true)
			return i;
	}

	if (i == 4) {
		printk("get rtlb failed!\n");
		return -EBUSY;
	}

	return 0;
}

/*
 *select SETn for transfer
 */
static int gpt_cdma_cmd_enable(int setn)
{
	uint64_t value = 0;

	value = gpt_cdma_read64(reg_base + GPT_CDMA_CMD);
	switch (setn) {
		case 0:
			value |= (GPT_CMD_SET0_EN | GPT_CMD_SET0);
			break;
		case 1:
			value |= (GPT_CMD_SET1_EN | GPT_CMD_SET1);
			break;
		case 2:
			value |= (GPT_CMD_SET2_EN | GPT_CMD_SET2);
			break;
		case 3:
			value |= (GPT_CMD_SET3_EN | GPT_CMD_SET3);
			break;
		default:
			printk("Invalid argument\n");
			return -EINVAL;
	}
	gpt_cdma_write64(reg_base + GPT_CDMA_CMD, value);

	return 0;
}

static void gpt_cdma_start_transfer(void)
{
	u64 value = 0;

	value = gpt_cdma_read64(reg_base + GPT_CDMA_CMD);
	value |= GPT_CMD_ACTIVE;

	gpt_cdma_write64(reg_base + GPT_CDMA_CMD, value);
}

int gpt_strcmp(const char *str1,const char *str2)
{
	return strcmp(str1, str2);
}
EXPORT_SYMBOL_GPL(gpt_strcmp);

void gpt_cmda_cache(u8 *src, u64 len)
{
	unsigned long start, end;

	start = (unsigned long)src & (~(L2_CACHE_BYTES - 1));
	end = (unsigned long)src + len + 1;
	__dma_sync_cache_range(start, end);
}
EXPORT_SYMBOL_GPL(gpt_cmda_cache);

/*
 * Check whether a certain channel is busy or not.
 */
static int gpt_dma_phy_busy(void)
{
	unsigned int value = gpt_cdma_read64(reg_base + GPT_CDMA_STATUS);
	return value & (GPT_STATUS_A | GPT_STATUS_W | GPT_STATUS_P | GPT_STATUS_R | GPT_STATUS_E);

}

/*
 *whether phy channel is avail
 */
static bool gpt_dma_phy_avail(struct gpt_dma_chan *gptchan,
		struct gpt_dma_phy *phy)
{
	return (phy->avail == true) ? true : false;
}

/*
 * Allocate a physical channel for a virtual channel: SETn
 *
 * Try to locate a physical channel to be used for this transfer. If all
 * are taken return NULL and the requester will have to cope by using
 * some fallback PIO mode or retrying later.
 */
static struct gpt_dma_phy *gpt_dma_get_phy(struct gpt_dma_chan *gptchan)
{
	struct gpt_dma_engine *gptdma = gptchan->host;
	struct gpt_dma_phy *phy = NULL;
	int i;

	for (i = 0; i < 4; i++) {
		phy = gptchan->phy[i];

		if (gpt_dma_phy_avail(gptchan, phy))
			break;
	}

	/* No physical channel available, cope with it */
	if (i == 4) {
		dev_warn(&gptdma->pdev->dev, "no phy channel available\n");
		return NULL;
	}

	return phy;
}

static int gpt_dma_set_runtime_config(struct gpt_dma_chan *gptchan,
		struct dma_slave_config *config)
{
	gptchan->cfg = *config;

	return 0;
}

/*
 * Transfer handling
 */
static inline struct gpt_txd *to_gpt_txd(struct dma_async_tx_descriptor *tx)
{
	return container_of(tx, struct gpt_txd, vd.tx);
}

/*
 *free struct gpt_sg of list in struct gpt_txd
 *free struct gpt_txd
 */
static void gpt_dma_free_txd(struct gpt_txd *txd)
{
	struct gpt_sg *dsg, *_dsg;

	list_for_each_entry_safe(dsg, _dsg, &txd->dsg_list, node) {
		list_del(&dsg->node);
		kfree(dsg);
	}

	kfree(txd);
}

static void gpt_dma_free_txd_list(struct gpt_dma_chan *gptchan)
{
	LIST_HEAD(head);

	vchan_get_all_descriptors(&gptchan->vc, &head);
	vchan_dma_desc_free_list(&gptchan->vc, &head);
}


/*
 * Try to allocate a physical channel.  When successful, assign it to
 * this virtual channel, and initiate the next descriptor.  The
 * virtual channel lock must be held at this point.
 */
static int gpt_dma_phy_alloc_and_start(struct gpt_dma_chan *gptchan)
{
	struct gpt_dma_parameter *para = gptchan->para;
	struct gpt_dma_phy *phy;
	struct virt_dma_desc *vd = vchan_next_desc(&gptchan->vc);
	struct gpt_txd *txd = to_gpt_txd(&vd->tx);
	struct dma_async_tx_descriptor *tx = &vd->tx;
	struct gpt_sg *dsg;
	dma_addr_t src_addr[4], dst_addr[4];
	u8 i = 0, src_count = 0, dst_count = 0;
	u8 src_nents = 0, dst_nents = 0;
	u8 tlb_id = 0, set_id = 0, group_id = 0, virt_id = 0;
	int ret = 0;

#ifndef GPT_DMA_IRQ_EN
	u8 timeout = 0;
#endif

#ifdef GPT_DMA_IRQ_EN
	this_cpu_ptr(cpu_dma)->tx = tx;
	gptchan->cpu_id = smp_processor_id();
#endif

	gptchan->at = txd;

	phy = gpt_dma_get_phy(gptchan);
	if (!phy) {
		printk("no physical channel available for xfer on %s\n", gptchan->name);
		return -ENODEV;
	}
	phy->avail = false;

	src_nents = txd->src_nents;
	dst_nents = txd->dst_nents;

	list_del(&txd->vd.node);
	
	if (para->copy_mode == GPT_DMA_MEMCPY) {
		ret = gpt_cdma_setn_op(phy->id, set_id, para); 
		if (ret < 0) {
			printk("set DMA OP_SETn failed!\n");
			return -EINVAL;
		}
	} else if (para->copy_mode == GPT_DMA_MEMCPY_SG_INTERLEAVE) {
		ret = gpt_cdma_setn_op(phy->id, set_id, para); 
		if (ret < 0) {
			printk("set DMA OP_SETn failed!\n");
			return -EINVAL;
		}
	} else if (para->copy_mode == GPT_DMA_MEMCPY_SG_NO_INTERLEAVE) {
		ret = gpt_cdma_setn_op(phy->id, set_id, para); 
		if (ret < 0) {
			printk("set DMA OP_SETn failed!\n");
			return -EINVAL;
		}
		set_id ++;
	}

	list_for_each_entry(dsg, &txd->dsg_list, node) {
		if (para->copy_mode == GPT_DMA_MEMCPY) {
			if (para->opset[set_id]->type == GPT_FILL){
				dst_addr[dst_count] = dsg->dst_addr;
			} else { 
				dst_addr[dst_count] = dsg->dst_addr;
				src_addr[src_count] = dsg->src_addr;
			}
			break;
		} else if (para->copy_mode == GPT_DMA_MEMCPY_SG_INTERLEAVE) {
			if (src_nents --) {
				src_addr[src_count] = dsg->src_addr;
				src_count ++;
			} else if (dst_nents --) {
				src_nents = 0;
				dst_addr[dst_count] = dsg->dst_addr;
			} else 
				break;
		} else if (para->copy_mode == GPT_DMA_MEMCPY_SG_NO_INTERLEAVE) {
			if (src_nents --) {
				src_addr[src_count] = dsg->src_addr;
				src_count ++;
			} else if(dst_nents --) {
				src_nents = 0;
				dst_addr[dst_count] = dsg->dst_addr;
				dst_count ++;
			} else 
				break;
		}
	}

	src_count = dst_count = 0;
	src_nents = txd->src_nents;
	dst_nents = txd->dst_nents;

	if (para->copy_mode == GPT_DMA_MEMCPY) {
		if (para->opset[set_id]->type == GPT_FILL){
			if (gpt_cdma_read64(reg_base + GPT_CDMA_SCTL) & 0x1) {
				gpt_cdma_write64(reg_base + GPT_CDMA_WITH_SET(phy->id), para->opset[set_id]->fill_data);	
				gpt_cdma_wr_setx(phy->id, para->data_info[group_id]->dst_columns, para->data_info[group_id]->dst_rows,
						para->data_info[group_id]->dst_stride, para->dst_virt[virt_id]);
				tlb_id = gpt_cdma_get_wtlb(gptchan);
				if(tlb_id >= 0){
					gpt_cdma_tlb_w(tlb_id, dsg->dst_addr, para->dst_virt[virt_id], 
							para->data_info[group_id]->dst_columns * para->data_info[group_id]->dst_rows);
					gptchan->gpt_wtlb[tlb_id]->avail = false;
				}

			} else {
				gpt_cdma_write64(reg_base + GPT_CDMA_WITH_SET(phy->id), para->opset[set_id]->fill_data);	
				gpt_cdma_wr_setx(phy->id, para->data_info[group_id]->dst_columns, para->data_info[group_id]->dst_rows,
						para->data_info[group_id]->dst_stride, dst_addr[dst_count]);
			}

		} else {
			if (gpt_cdma_read64(reg_base + GPT_CDMA_SCTL) & 0x1) {
				gpt_cdma_write64(reg_base + GPT_CDMA_RDBASE_SET(i, phy->id), para->src_virt[virt_id]);
				gpt_cdma_rd_setx(phy->id, para->data_info[group_id]->src_columns, para->data_info[group_id]->src_rows,
						para->data_info[group_id]->src_stride);
				gpt_cdma_wr_setx(phy->id, para->data_info[group_id]->dst_columns, para->data_info[group_id]->dst_rows,
						para->data_info[group_id]->dst_stride, para->dst_virt[virt_id]);

				tlb_id = gpt_cdma_get_rtlb(gptchan);
				if(tlb_id >= 0){
					gpt_cdma_tlb_r(tlb_id, src_addr[src_count], para->src_virt[virt_id], 0xffffffffffffffff);
					gptchan->gpt_rtlb[tlb_id]->avail = false;
				}
				tlb_id = gpt_cdma_get_wtlb(gptchan);
				if(tlb_id >= 0){
					gpt_cdma_tlb_w(tlb_id, dst_addr[dst_count], para->dst_virt[virt_id], 0xffffffffffffffff);
					gptchan->gpt_wtlb[tlb_id]->avail = false;
				}

			} else {
				gpt_cdma_write64(reg_base + GPT_CDMA_RDBASE_SET(i, phy->id), src_addr[src_count]);
				gpt_cdma_rd_setx(phy->id, para->data_info[group_id]->src_columns, para->data_info[group_id]->src_rows,
						para->data_info[group_id]->src_stride);
				gpt_cdma_wr_setx(phy->id, para->data_info[group_id]->dst_columns, para->data_info[group_id]->dst_rows,
						para->data_info[group_id]->dst_stride, dst_addr[dst_count]);
			}
		}
		gpt_cdma_cmd_enable(phy->id);	

	} else if (para->copy_mode == GPT_DMA_MEMCPY_SG_INTERLEAVE) {
		while (src_nents --) {
			gpt_cdma_write64(reg_base + GPT_CDMA_RDBASE_SET(i++, phy->id), src_addr[src_count++]);
			gpt_cdma_rd_setx(phy->id, para->data_info[group_id]->src_columns, 0, 0);
		}

		while (dst_nents --)
			gpt_cdma_wr_setx(phy->id, para->data_info[group_id]->dst_columns, 1, 0, dst_addr[dst_count]);

		gpt_cdma_cmd_enable(phy->id);

	} else if (para->copy_mode == GPT_DMA_MEMCPY_SG_NO_INTERLEAVE) {
		while (src_nents --) {
			if (gpt_cdma_read64(reg_base + GPT_CDMA_SCTL) & 0x1) {
				gpt_cdma_write64(reg_base + GPT_CDMA_RDBASE_SET(0, phy->id), para->src_virt[virt_id]);
				gpt_cdma_rd_setx(phy->id, para->data_info[group_id]->src_columns, para->data_info[group_id]->src_rows,
						para->data_info[group_id]->src_stride);
				gpt_cdma_wr_setx(phy->id, para->data_info[group_id]->dst_columns, para->data_info[group_id]->dst_rows,
						para->data_info[group_id]->dst_stride, para->dst_virt[virt_id]);

				tlb_id = gpt_cdma_get_rtlb(gptchan);
				if (tlb_id >= 0) {
					gpt_cdma_tlb_r(tlb_id, src_addr[src_count], para->src_virt[virt_id], 0xffffffffffffffff);
					gptchan->gpt_rtlb[tlb_id]->avail = false;
				}
				tlb_id = gpt_cdma_get_wtlb(gptchan);
				if (tlb_id >= 0) {
					gpt_cdma_tlb_w(tlb_id, dst_addr[dst_count], para->dst_virt[virt_id], 0xffffffffffffffff);
					gptchan->gpt_wtlb[tlb_id]->avail = false;
				}
				virt_id ++;

			} else {
				gpt_cdma_write64(reg_base + GPT_CDMA_RDBASE_SET(0, phy->id), src_addr[src_count]);
				gpt_cdma_rd_setx(phy->id, para->data_info[group_id]->src_columns, para->data_info[group_id]->src_rows,
						para->data_info[group_id]->src_stride);
				gpt_cdma_wr_setx(phy->id, para->data_info[group_id]->dst_columns, para->data_info[group_id]->dst_rows,
						para->data_info[group_id]->dst_stride, dst_addr[dst_count]);
			}

			src_count ++;
			dst_count ++;
			group_id ++;

			gpt_cdma_cmd_enable(phy->id);

			phy = gpt_dma_get_phy(gptchan);
			if (!phy) {
				printk("no physical channel available for xfer on %s\n", gptchan->name);
				return -ENODEV;
			}
			phy->avail = false;

			ret = gpt_cdma_setn_op(phy->id, set_id, para); 
			if (ret < 0) {
				printk("set DMA OP_SETn failed!\n");
				return -EINVAL;
			}

			set_id ++;
		} 
	}

	gpt_cdma_start_transfer();

#ifndef GPT_DMA_IRQ_EN
	/* Wait for channel inactive */
	while (gpt_dma_phy_busy()) {
		if (gpt_cdma_read64(reg_base + GPT_CDMA_STATUS) & GPT_STATUS_E)
		{
			if (gpt_cdma_read64(reg_base + GPT_CDMA_STATUS) & GPT_STATUS_CODE)
			{
				if (timeout ++ == 10) {
					timeout = 0;
					break;
				}

				gpt_cdma_write64(reg_base + GPT_CDMA_ASYNC, GPT_ASYNC_RESUM);
				gpt_cdma_start_transfer();

			} else {
				printk("CPU DMA ERROR (not TLB miss)!\n");
				break;
			}
		} 
	}

	gptchan->dma_status = gpt_cdma_read64(reg_base + GPT_CDMA_STATUS);

	if (tx->callback)
		tx->callback(tx->callback_param);
#endif

	return ret;
}

/*
 * Free a physical DMA channel, potentially reallocating it to another
 * virtual channel if we have any pending.
 */
static void gpt_dma_phy_free(struct gpt_dma_chan *gptchan)
{
	struct gpt_dma_phy *phy;
	u8 i = 0;

	gpt_cdma_reset();

	/* No more jobs, so free up the physical channel */
	for (i = 0; i < 4; i++) {
		phy = gptchan->phy[i];
		phy->avail = true;

		gptchan->gpt_rtlb[i]->avail = true;
		gptchan->gpt_wtlb[i]->avail = true;
	}
}

/*
 *free transfer desc when transfer complete
 */
static void gpt_dma_desc_free(struct virt_dma_desc *vd)
{
	struct gpt_txd *txd = to_gpt_txd(&vd->tx);

	gpt_dma_free_txd(txd);
}

#ifdef GPT_DMA_IRQ_EN
static irqreturn_t gpt_dma_irq(int irq, void *data)
{
	struct dma_async_tx_descriptor *tx;
	unsigned long flags;
	u64 status;

	spin_lock_irqsave(&(this_cpu_ptr(cpu_dma)->lock), flags);

	status = gpt_cdma_read64(reg_base + GPT_CDMA_STATUS);

	if (status & GPT_STATUS_CODE)
	{
		gpt_cdma_write64(reg_base + GPT_CDMA_ASYNC, GPT_ASYNC_RESUM);
		gpt_cdma_start_transfer();

	} else { 
		tx = this_cpu_ptr(cpu_dma)->tx;
		this_cpu_ptr(cpu_dma)->dma_status = status;
		gpt_cdma_write64(reg_base + GPT_CDMA_ASYNC, GPT_ASYNC_CLEAR);

		if (tx->callback)
			tx->callback(tx->callback_param);
	}

	spin_unlock_irqrestore(&(this_cpu_ptr(cpu_dma)->lock), flags);

	return IRQ_HANDLED;
}
#endif

/*
 * called by func: dmaengine_terminate_all() or dmaengine_device_control() or dmaengine_pause()
 *				or dmaengine_resume()
 */
static int gpt_dma_control(struct dma_chan *chan, enum dma_ctrl_cmd cmd,
		unsigned long arg)
{
	struct gpt_dma_chan *gptchan = to_gpt_dma_chan(chan);
	struct gpt_dma_engine *gptdma = gptchan->host;
	unsigned long flags;
	int ret = 0;
	u64 status = 0;

	spin_lock_irqsave(&gptchan->vc.lock, flags);

	switch (cmd) {
		case DMA_SLAVE_CONFIG:
			ret = gpt_dma_set_runtime_config(gptchan,
					(struct dma_slave_config *)arg);
			break;
		case DMA_PAUSE:
			gpt_cdma_write64(reg_base + GPT_CDMA_ASYNC, GPT_ASYNC_PAUSE);
			do {
				status = gpt_cdma_read64(reg_base + GPT_CDMA_STATUS);
			} while (status & GPT_STATUS_P);
			break;
		case DMA_RESUME:
			gpt_cdma_write64(reg_base + GPT_CDMA_ASYNC, GPT_ASYNC_RESUM);
			do {
				status = gpt_cdma_read64(reg_base + GPT_CDMA_STATUS);
			} while (status & GPT_STATUS_A);
			break;
		case DMA_TERMINATE_ALL:
#if 0
			if (!gptchan->at) {
				dev_err(&gptdma->pdev->dev, "trying to terminate already stopped channel %d\n",
						gptchan->id);
				ret = -EINVAL;
				break;
			}
#endif

			/* Mark physical channel as free */
			gpt_dma_phy_free(gptchan);

			/* Dequeue current job */
			if (gptchan->at) {
				gpt_dma_desc_free(&gptchan->at->vd);
				gptchan->at = NULL;
			}

			/* Dequeue jobs not yet fired as well */
			gpt_dma_free_txd_list(gptchan);

#ifdef GPT_DMA_IRQ_EN
			this_cpu_ptr(cpu_dma)->dma_status = 0;
#endif
			this_cpu_ptr(cpu_dma)->cpu_dma_valid = true;
			break;
		default:
			ret = -ENXIO;
			break;
	}

	spin_unlock_irqrestore(&gptchan->vc.lock, flags);

	return ret;
}

/*
 *called by func: dma_request_channel()
 */
static int gpt_dma_alloc_chan_resources(struct dma_chan *chan)
{
	return 0;
}

/*
 *called by func: dma_release_channel()
 */
static void gpt_dma_free_chan_resources(struct dma_chan *chan)
{
	/* Ensure all queued descriptors are freed */
	vchan_free_chan_resources(to_virt_chan(chan));
}

/*
 *called by func: dma_async_is_tx_complete() or dmaengine_tx_status()
 */
static enum dma_status gpt_dma_tx_status(struct dma_chan *chan,
		dma_cookie_t cookie, struct dma_tx_state *txstate)
{
	enum dma_status ret = DMA_IN_PROGRESS;
	u64 status;
	struct gpt_dma_chan *gptchan = to_gpt_dma_chan(chan);
	unsigned long flags;
	int cpu_id;

	spin_lock_irqsave(&gptchan->vc.lock, flags);

#ifdef GPT_DMA_IRQ_EN
	cpu_id = gptchan->cpu_id;
	status = per_cpu_ptr(cpu_dma, cpu_id)->dma_status;
#else
	status = gptchan->dma_status;
#endif

	if(status & GPT_STATUS_D){
		ret = DMA_COMPLETE; 
	} else if (status & GPT_STATUS_A) {
		ret = DMA_IN_PROGRESS; 
	} else if (status & GPT_STATUS_P) {
		ret = DMA_PAUSED; 
	} else if (status & GPT_STATUS_E){
		ret = DMA_ERROR; 
	}
	spin_unlock_irqrestore(&gptchan->vc.lock, flags);	

	return ret;
}

/*
 * Initialize a descriptor to be used by memcpy submit
 * @flags: enum dma_ctrl_flags
 */
static struct dma_async_tx_descriptor *gpt_dma_prep_memcpy(
		struct dma_chan *chan, dma_addr_t dest, dma_addr_t src,
		size_t len, unsigned long flags)
{
	struct gpt_dma_chan *gptchan = to_gpt_dma_chan(chan);
	struct gpt_txd *txd;
	struct gpt_sg *dsg;
	struct gpt_dma_parameter *para = gptchan->para;
	struct dma_async_tx_descriptor *tx;

	spin_lock_irqsave(&gptchan->vc.lock, flags);

	if (this_cpu_ptr(cpu_dma)->cpu_dma_valid == true) {
		this_cpu_ptr(cpu_dma)->cpu_dma_valid = false;

		para->copy_mode = GPT_DMA_MEMCPY;

		txd = kzalloc(sizeof(*txd), GFP_NOWAIT);
		if (!txd)
			goto err;

		INIT_LIST_HEAD(&txd->dsg_list);

		dsg = kzalloc(sizeof(*dsg), GFP_NOWAIT);
		if (!dsg) {
			gpt_dma_free_txd(txd);
			goto err;
		}

		list_add_tail(&dsg->node, &txd->dsg_list);

		dsg->src_addr = src;
		dsg->dst_addr = dest;

		/*
		 * vchan_tx_prep - prepare a descriptor
		 * vc: virtual channel allocating this descriptor
		 * vd: virtual descriptor to prepare
		 * tx_flags: flags argument passed in to prepare function
		 * return vd->tx
		 * &gptchan->vc->chan = &txd->vd->tx->chan
		 */
		tx = vchan_tx_prep(&gptchan->vc, &txd->vd, flags);
	} else {
		tx = NULL;
	}

	spin_unlock_irqrestore(&gptchan->vc.lock, flags);

	return tx;

err:
	spin_unlock_irqrestore(&gptchan->vc.lock, flags);
	return NULL;
}

static struct dma_async_tx_descriptor *gpt_prep_memcpy_sg(
		struct dma_chan *chan,
		struct scatterlist *dst_sg, unsigned int dst_nents,
		struct scatterlist *src_sg, unsigned int src_nents,
		unsigned long flags)
{
	struct gpt_dma_chan *gptchan = to_gpt_dma_chan(chan);
	struct gpt_dma_parameter *para = gptchan->para;
	struct scatterlist *sg;
	struct gpt_txd *txd;
	struct gpt_sg *dsg;
	int temp;
	struct dma_async_tx_descriptor *tx;

	spin_lock_irqsave(&gptchan->vc.lock, flags);

	if (this_cpu_ptr(cpu_dma)->cpu_dma_valid == true) {
		this_cpu_ptr(cpu_dma)->cpu_dma_valid = false;

		if(dst_nents == src_nents)
			para->copy_mode = GPT_DMA_MEMCPY_SG_NO_INTERLEAVE;
		else
			para->copy_mode = GPT_DMA_MEMCPY_SG_INTERLEAVE;

		txd = kzalloc(sizeof(*txd), GFP_NOWAIT);
		if (!txd) 
			goto err;

		INIT_LIST_HEAD(&txd->dsg_list);

		txd->src_nents = src_nents; 
		txd->dst_nents = dst_nents;

		dsg = kzalloc(sizeof(*dsg), GFP_NOWAIT);
		if (!dsg) {
			gpt_dma_free_txd(txd);
			goto err;
		}

		for_each_sg(src_sg, sg, src_nents, temp) {
			dsg = kzalloc(sizeof(*dsg), GFP_NOWAIT);
			if (!dsg) {
				gpt_dma_free_txd(txd);
				goto err;
			}
			list_add_tail(&dsg->node, &txd->dsg_list);

			dsg->src_addr = sg_dma_address(sg);
		}

		for_each_sg(dst_sg, sg, dst_nents, temp) {
			dsg = kzalloc(sizeof(*dsg), GFP_NOWAIT);
			if (!dsg) {
				gpt_dma_free_txd(txd);
				goto err;
			}
			list_add_tail(&dsg->node, &txd->dsg_list);

			dsg->dst_addr = sg_dma_address(sg);
		}
		tx = vchan_tx_prep(&gptchan->vc, &txd->vd, flags);
	} else {
		tx = NULL;
	}

	spin_unlock_irqrestore(&gptchan->vc.lock, flags);
	return tx;

err:
	spin_unlock_irqrestore(&gptchan->vc.lock, flags);
	return NULL;
}

static struct dma_async_tx_descriptor *gpt_dma_prep_dma_cyclic(
		struct dma_chan *chan, dma_addr_t addr, size_t size, size_t period,
		enum dma_transfer_direction direction, unsigned long flags)
{
	struct gpt_dma_chan *gptchan = to_gpt_dma_chan(chan);
	struct gpt_dma_engine *gptdma = gptchan->host;
	struct gpt_dma_parameter *para = gptchan->para;
	struct gpt_txd *txd;
	struct gpt_sg *dsg;
	unsigned sg_len;
	dma_addr_t slave_addr;
	int i;
	struct dma_async_tx_descriptor *tx;

	spin_lock_irqsave(&gptchan->vc.lock, flags);

	if (this_cpu_ptr(cpu_dma)->cpu_dma_valid == true) {
		this_cpu_ptr(cpu_dma)->cpu_dma_valid = false;

		para->copy_mode = GPT_DMA_CYCLIC;

		dev_dbg(&gptdma->pdev->dev,
				"prepare cyclic transaction of %zu bytes with period %zu from %s\n",
				size, period, gptchan->name);

		if (!is_slave_direction(direction)) {
			dev_err(&gptdma->pdev->dev,
					"direction %d unsupported\n", direction);
			goto err;
		}

		txd = kzalloc(sizeof(*txd), GFP_NOWAIT);
		if (!txd) 
			goto err;

		INIT_LIST_HEAD(&txd->dsg_list);

		txd->cyclic = 1;

		if (direction == DMA_MEM_TO_DEV)
			slave_addr = gptchan->cfg.dst_addr;
		else 
			slave_addr = gptchan->cfg.src_addr;

		sg_len = size / period;

		for (i = 0; i < sg_len; i++) {
			dsg = kzalloc(sizeof(*dsg), GFP_NOWAIT);
			if (!dsg) {
				gpt_dma_free_txd(txd);
				goto err;
			}
			list_add_tail(&dsg->node, &txd->dsg_list);

			dsg->len = period;

			/* Check last period length */
			if (i == sg_len - 1)
				dsg->len = size - period * i;
			if (direction == DMA_MEM_TO_DEV) {
				dsg->src_addr = addr + period * i;
				dsg->dst_addr = slave_addr;
			} else { /* DMA_DEV_TO_MEM */
				dsg->src_addr = slave_addr;
				dsg->dst_addr = addr + period * i;
			}
		}
		tx = vchan_tx_prep(&gptchan->vc, &txd->vd, flags);
	} else {
		tx = NULL;
	}

	spin_unlock_irqrestore(&gptchan->vc.lock, flags);
	return tx;

err:
	spin_unlock_irqrestore(&gptchan->vc.lock, flags);
	return NULL;
}

/*
 * Slave transactions callback to the slave device to allow
 * synchronization of slave DMA signals with the DMAC enable
 */
static void gpt_dma_issue_pending(struct dma_chan *chan)
{
	struct gpt_dma_chan *gptchan = to_gpt_dma_chan(chan);
	unsigned long flags;

	spin_lock_irqsave(&gptchan->vc.lock, flags);

	if (vchan_issue_pending(&gptchan->vc)) 
		gpt_dma_phy_alloc_and_start(gptchan);

	spin_unlock_irqrestore(&gptchan->vc.lock, flags);
}

int gpt_cmda_preload_l2(struct dma_chan *chan, u8 *src, u64 columns, u64 rows)
{
	unsigned long dst_addr = 0x20780000;
	unsigned long src_addr;
	struct gpt_dma_chan *gptchan = to_gpt_dma_chan(chan);
	struct gpt_dma_phy *phy;
	int status = 0;
	u64 len = columns * rows;
	unsigned long flags;

	if (len >= 524288){
		printk("len is Invalid argument! len < 524288\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&gptchan->vc.lock, flags);

	if (this_cpu_ptr(cpu_dma)->cpu_dma_valid == true) {
		this_cpu_ptr(cpu_dma)->cpu_dma_valid = false;

		gpt_cdma_reset();
		gpt_cdma_l2_en();

		phy = gpt_dma_get_phy(gptchan);
		if (!phy) {
			printk("no physical channel available for xfer\n");
			goto dev_err;
		}
		phy->avail = false;

		src_addr = virt_to_phys(src);

		gpt_cdma_write64(reg_base + GPT_CDMA_OP_SET(phy->id), 
				GPT_OP_TY_COPY | GPT_OP_RD_MULTIPLE | GPT_OP_WR_MULTIPLE | GPT_OP_SZ_1B | GPT_OP_CN_1B);
		gpt_cdma_write64(reg_base + GPT_CDMA_RDBASE_SET(0, phy->id), src_addr);
		gpt_cdma_rd_setx(phy->id, columns, rows, 0);
		gpt_cdma_wr_setx(phy->id, columns, rows, 0, dst_addr);

		gpt_cdma_cmd_enable(phy->id);
		gpt_cdma_start_transfer();

		/* Wait for channel inactive */
		while (gpt_dma_phy_busy()) {
			if (gpt_cdma_read64(reg_base + GPT_CDMA_STATUS) & GPT_STATUS_E)
				break;
		}

		if (gpt_cdma_read64(reg_base + GPT_CDMA_STATUS) & 0x20) {
			status = -1;
			printk("transfer failed!\n\n");
		}

		gpt_dma_phy_free(gptchan);

		gpt_cdma_l2_dis();

		gpt_cdma_reset();
		this_cpu_ptr(cpu_dma)->cpu_dma_valid = true;
	} else {
		goto err;
	}

	spin_unlock_irqrestore(&gptchan->vc.lock, flags);

	return status;

dev_err:
	this_cpu_ptr(cpu_dma)->cpu_dma_valid = true;
	spin_unlock_irqrestore(&gptchan->vc.lock, flags);
	return -ENODEV; 

err:
	spin_unlock_irqrestore(&gptchan->vc.lock, flags);
	return -EBUSY;
}
EXPORT_SYMBOL_GPL(gpt_cmda_preload_l2);

/*
 * Initialise the DMAC memcpy/slave channels.
 * Make a local wrapper to hold required data
 */
static int gpt_dma_init_virtual_channels(struct gpt_dma_engine *gptdma,
		struct dma_device *dmadev)
{
	int i, cpu, ret;
	struct gpt_dma_chan *chan;

	INIT_LIST_HEAD(&dmadev->channels);

	cpu_dma = alloc_percpu(struct gpt_cpu_dma);
	if(cpu_dma == NULL){
		return -ENOMEM;
	}

	/*
	 * Register as many many memcpy as we have physical channels,
	 * we won't always be able to use all but the code will have
	 * to cope with that situation.
	 */
	for (cpu = 0; cpu < 4; cpu++) {
		struct gpt_dma_parameter *para;

		chan = devm_kzalloc(dmadev->dev, sizeof(*chan), GFP_KERNEL);
		if (!chan) {
			dev_err(dmadev->dev,
					"%s no memory for channel\n", __func__);
			return -ENOMEM;
		}

		chan->host = gptdma;

		chan->name = kasprintf(GFP_KERNEL, "cpu%d_chan", cpu);
		if (!chan->name)
			return -ENOMEM;

		dev_info(dmadev->dev,
				"initialize virtual channel \"%s\"\n",
				chan->name);

		for (i = 0; i < 4; i ++) {
			chan->phy[i] = devm_kzalloc(dmadev->dev, sizeof(struct gpt_dma_phy), GFP_KERNEL);
			if (chan->phy[i] == NULL) {
				dev_err(dmadev->dev,
						"%s no memory for phy\n", __func__);
				return -ENOMEM;
			}

			chan->phy[i]->id = i;
			chan->phy[i]->avail = true;
		}

		para = devm_kzalloc(dmadev->dev, sizeof(struct gpt_dma_parameter), GFP_KERNEL);
		if (!para) {
			dev_err(dmadev->dev,
					"%s no memory for channel para\n", __func__);
			return -ENOMEM;
		}
		chan->para = para;

		for(i = 0; i < 4; i ++) {
			para->data_info[i] = devm_kzalloc(dmadev->dev, sizeof(struct gpt_data_info), GFP_KERNEL);
			if (para->data_info[i] == NULL) {
				dev_err(dmadev->dev,
						"%s no memory for channel para\n", __func__);
				return -ENOMEM;
			}

			para->opset[i] = devm_kzalloc(dmadev->dev, sizeof(struct gpt_opset_para), GFP_KERNEL);
			if (para->opset[i] == NULL) {
				dev_err(dmadev->dev,
						"%s no memory for channel para\n", __func__);
				return -ENOMEM;
			}
		}

		for(i = 0; i < 4; i ++) {
			chan->gpt_wtlb[i] = devm_kzalloc(dmadev->dev, sizeof(struct gpt_dma_wtlb), GFP_KERNEL);
			if (chan->gpt_wtlb[i] == NULL) {
				dev_err(dmadev->dev,
						"%s no memory for channel wtlb\n", __func__);
				return -ENOMEM;
			}
			chan->gpt_wtlb[i]->id = i;
			chan->gpt_wtlb[i]->avail = true;

			chan->gpt_rtlb[i] = devm_kzalloc(dmadev->dev, sizeof(struct gpt_dma_rtlb), GFP_KERNEL);
			if (chan->gpt_rtlb[i] == NULL) {
				dev_err(dmadev->dev,
						"%s no memory for channel rtlb\n", __func__);
				return -ENOMEM;
			}
			chan->gpt_rtlb[i]->id = i;
			chan->gpt_rtlb[i]->avail = true;
		}

		//virt chan desc
		chan->vc.desc_free = gpt_dma_desc_free;
		chan->vc.chan.chan_id = cpu;
		//dma_cookie_init
		vchan_init(&chan->vc, dmadev);
	}
	dev_info(dmadev->dev, "initialized %d virtual %s channels\n", i, "memcpy");

#ifdef GPT_DMA_IRQ_EN
	ret = request_percpu_irq(irq_cdma, gpt_dma_irq, "gpt_dma_irq", cpu_dma);
	if (ret) {
		printk("Unable to request irq, error %d\n", ret);
		free_percpu(cpu_dma);
		return ret;
	}
#endif

	for_each_present_cpu(cpu) {
		spinlock_t lock;
		lock = per_cpu_ptr(cpu_dma, cpu)->lock; 
		spin_lock_init(&lock);
		per_cpu_ptr(cpu_dma, cpu)->dma_status = 0;
		per_cpu_ptr(cpu_dma, cpu)->cpu_dma_valid = true;
	}

	return i;
}

static void gpt_dma_free_virtual_channels(struct dma_device *dmadev)
{
	struct gpt_dma_chan *chan = NULL;
	struct gpt_dma_chan *next;

	list_for_each_entry_safe(chan,
			next, &dmadev->channels, vc.chan.device_node)
		list_del(&chan->vc.chan.device_node);
}

static const struct of_device_id gpt_dma_of_match[] = {
	{ .compatible = "gpt,cpu-dma", },
	{}
};
MODULE_DEVICE_TABLE(of, gpt_spi_of_match);

static int gptdma_nb(struct notifier_block *nfb,
		unsigned long action, void *hcpu)
{
	if ((action == CPU_STARTING) && 
			(smp_processor_id() > 0)) {
		cdma_enable(phy_addr);
		gpt_cdma_get_spr();
		gpt_cdma_reset();
	}
	return NOTIFY_OK;
}

static struct notifier_block gptdma_cpu_notifier = {
	.notifier_call = gptdma_nb,
	.priority = 100,
};

int __init early_gptdma_init(void)
{
	struct device_node *np;
	const u32 *prop_val;
	u64 prop_size = 0;
	phys_addr_t phys;
	void __iomem *addr;
	u8 i = 0;

	np = of_find_matching_node(NULL, gpt_dma_of_match);
	if (!np)
		return -ENODEV;

	prop_val = of_get_address(np, 0, &prop_size, NULL);
	if (!prop_val)
		return -EINVAL;

	phys = (phys_addr_t)of_translate_address(np, prop_val);
	phy_addr = phys;

	of_node_put(np);

	addr = ioremap_nocache(phys, prop_size);
	if (!addr)
		return -ENOMEM;

	reg_base = addr;

	cpumask_set_cpu(0, &cpus_allowed);
	cdma_enable(phy_addr);
	gpt_cdma_get_spr();
	gpt_cdma_reset();

	if (CONFIG_NR_CPUS > 1) {
		for(i = 1; i < 4; i ++)
			cpumask_set_cpu(i, &cpus_allowed);
		register_cpu_notifier(&gptdma_cpu_notifier);
	}

	return 0;
}
early_initcall(early_gptdma_init);

static int gpt_dma_probe(struct platform_device *pdev)
{
	struct gpt_dma_engine *gptdma;
	struct device_node *node = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	int ret = 0;

	printk("entry %s\n", __func__);

	irq_cdma = platform_get_irq(pdev, 0);
	if (irq_cdma < 0) {
		dev_err(dev, "failed to get irq, err %d\n",
				irq_cdma);
	}

	gptdma = devm_kzalloc(dev, sizeof(*gptdma), GFP_KERNEL);
	if (!gptdma)
		return -ENOMEM;

	gptdma->pdev = pdev;

	/* Initialize memcpy engine */
	dma_cap_set(DMA_MEMCPY, gptdma->memcpy.cap_mask);
	dma_cap_set(DMA_PRIVATE, gptdma->memcpy.cap_mask);
	dma_cap_set(DMA_SG, gptdma->memcpy.cap_mask);
	dma_cap_set(DMA_CYCLIC, gptdma->memcpy.cap_mask);
	gptdma->memcpy.dev = dev;
	gptdma->memcpy.device_alloc_chan_resources = gpt_dma_alloc_chan_resources;
	gptdma->memcpy.device_free_chan_resources =	gpt_dma_free_chan_resources;
	gptdma->memcpy.device_prep_dma_memcpy = gpt_dma_prep_memcpy;
	gptdma->memcpy.device_prep_dma_sg = gpt_prep_memcpy_sg;
	gptdma->memcpy.device_prep_dma_cyclic = gpt_dma_prep_dma_cyclic;
	gptdma->memcpy.device_tx_status = gpt_dma_tx_status;
	gptdma->memcpy.device_issue_pending = gpt_dma_issue_pending;
	gptdma->memcpy.device_control = gpt_dma_control;

	/* Register as many memcpy channels as there are physical channels */
	ret = gpt_dma_init_virtual_channels(gptdma, &gptdma->memcpy);
	if (ret < 0) {
		dev_warn(dev, "%s failed to enumerate memcpy channels - %d\n", __func__, ret);
	}

	ret = dma_async_device_register(&gptdma->memcpy);
	if (ret) {
		dev_warn(dev, "%s failed to register memcpy as an async device - %d\n", __func__, ret);
	}

	platform_set_drvdata(pdev, gptdma);
	dev_info(dev, "Loaded dma driver with %d physical channels\n", NUM_PHY_CHANNELS);

	/* Register with OF helpers for DMA lookups (nonfatal) */
	if (node) {
		ret = of_dma_controller_register(node,
				of_dma_xlate_by_chan_id, gptdma);
		if (ret)
			dev_warn(dev, "Could not register for OF lookup\n");
	}

	return ret;
}

static int gpt_dma_remove(struct platform_device *pdev)
{
	struct gpt_dma_engine *gptdma = platform_get_drvdata(pdev);

	dma_async_device_unregister(&gptdma->memcpy);

	gpt_dma_free_virtual_channels(&gptdma->memcpy);

	iounmap((void *)reg_base);

#ifdef GPT_DMA_IRQ_EN
	free_percpu_irq(irq_cdma, cpu_dma);
#endif
	free_percpu(cpu_dma);

	return 0;
}

static struct platform_driver gpt_dma_driver = {
	.driver		= {
		.name	= "gpt,cpu-dma",
		.owner	= THIS_MODULE,
		.of_match_table = gpt_dma_of_match,
	},
	.probe		= gpt_dma_probe,
	.remove		= gpt_dma_remove,
};

module_platform_driver(gpt_dma_driver);

MODULE_DESCRIPTION("GPT DMA Driver");
MODULE_AUTHOR("GPT");
MODULE_LICENSE("GPL v2");
