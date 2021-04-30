#ifndef __GPT_CPU_DMA_H__
#define __GPT_CPU_DMA_H__


#define GPT_PXI_EN	(0x1 << 0)
#define GPT_PXI_MK(x)	((x) << 15)

#define GPT_CPU_DMA_MASK	0x0000ffff


/*DMA register */
#define GPT_CDMA_STATUS			0x000
#define GPT_CDMA_MISS			0x001
#define GPT_CDMA_SCTL			0x002
#define GPT_CDMA_ASYNC			0x003
#define GPT_CDMA_CMD			0x005

#define GPT_CDMA_OP_SET(n)  		(0x006 + (n) * 0x20)
#define GPT_CDMA_WITH_SET(n)		(0x007 + (n) * 0x20)
#define GPT_CDMA_RDBASE_SET(m,n)    ((0x008 + m * 0x1) + (n) * 0x20)
#define GPT_CDMA_RDBASE0_SET(n)		(0x008 + (n) * 0x20)
#define GPT_CDMA_RDBASE1_SET(n)		(0x009 + (n) * 0x20)
#define GPT_CDMA_RDBASE2_SET(n)		(0x00a + (n) * 0x20)
#define GPT_CDMA_RDBASE3_SET(n)		(0x00b + (n) * 0x20)
#define GPT_CDMA_WRBASE_SET(n)		(0x00c + (n) * 0x20)
#define GPT_CDMA_RDCOUNT0_SET(n)	(0x010 + (n) * 0x20)
#define GPT_CDMA_RDCOUNT1_SET(n)	(0x012 + (n) * 0x20)
#define GPT_CDMA_RDSTRIDE1_SET(n)	(0x013 + (n) * 0x20)
#define GPT_CDMA_WRCOUNT0_SET(n)	(0x018 + (n) * 0x20)
#define GPT_CDMA_WRCOUNT1_SET(n)	(0x019 + (n) * 0x20)
#define GPT_CDMA_WRSTRIDE1_SET(n)	(0x01a + (n) * 0x20)
#define GPT_CDMA_RDMASK_TLB(n)		(0x080 + (n) * 0x08)
#define GPT_CDMA_RDTAG_TLB(n)		(0x081 + (n) * 0x08)
#define GPT_CDMA_RDREAL_TLB(n)		(0x082 + (n) * 0x08)
#define GPT_CDMA_WRMASK_TLB(n)		(0x084 + (n) * 0x08)
#define GPT_CDMA_WRTAG_TLB(n)		(0x085 + (n) * 0x08)
#define GPT_CDMA_WRREAL_TLB(n)		(0x086 + (n) * 0x08)

/* status register offset */
#define GPT_STATUS_A			(0x1 << 0)
#define GPT_STATUS_D			(0x1 << 1)
#define GPT_STATUS_W			(0x1 << 2)
#define GPT_STATUS_P			(0x1 << 3)
#define GPT_STATUS_R			(0x1 << 4)
#define GPT_STATUS_E			(0x1 << 5)
#define GPT_STATUS_CODE			(0x3 << 6)
#define GPT_STATUS_OLD			(0x3 << 8)

/*sctl register offset*/
#define GPT_SCTL_VIRT			(0x01 << 0)
#define GPT_TLBMASK_L2			(0x01 << 1)
#define GPT_TLBMASK_SYNC		(0x01 << 2)
#define GPT_SCTL_RDL2_0			(0x01 << 4)
#define GPT_SCTL_RDL2_1         (0x01 << 5)
#define GPT_SCTL_RDL2_2         (0x01 << 6)
#define GPT_SCTL_RDL2_3         (0x01 << 7)
#define GPT_SCTL_RDSYN_0    	(0x01 << 8)
#define GPT_SCTL_RDSYN_1        (0x01 << 9)
#define GPT_SCTL_RDSYN_2        (0x01 << 10)
#define GPT_SCTL_RDSYN_3        (0x01 << 11)
#define GPT_SCTL_WRL2_0         (0x01 << 12)
#define GPT_SCTL_WRL2_1         (0x01 << 13)
#define GPT_SCTL_WRL2_2         (0x01 << 14)
#define GPT_SCTL_WRL2_3         (0x01 << 15)
#define GPT_SCTL_WRSYN_0        (0x01 << 16)
#define GPT_SCTL_WRSYN_1        (0x01 << 17)
#define GPT_SCTL_WRSYN_2        (0x01 << 18)
#define GPT_SCTL_WRSYN_3        (0x01 << 19)

/* async register offset */
#define GPT_ASYNC_RESET		(0x4)
#define GPT_ASYNC_PAUSE		(0x3)
#define GPT_ASYNC_CLEAR		(0x2)
#define GPT_ASYNC_RESUM		(0x1)

/* cmd register offset */
#define GPT_CMD_ACTIVE			(0x1 << 0)
#define GPT_CMD_SET0			(0x0 << 5)
#define GPT_CMD_SET0_EN			(0x1 << 4)
#define GPT_CMD_SET1			(0x1 << 9)
#define GPT_CMD_SET1_EN			(0x1 << 8)
#define GPT_CMD_SET2			(0x2 << 13)
#define GPT_CMD_SET2_EN			(0x1 << 12)
#define GPT_CMD_SET3			(0x3 << 17)
#define GPT_CMD_SET3_EN			(0x1 << 16)

#define ACTIVE_CTL			(1)
#define REGSET0_CTL(v__)		((((v__)&0x3)<<5)|1<<4)
#define REGSET1_CTL(v__)		((((v__)&0x3)<<9)|(1<<8))
#define REGSET2_CTL(v__)		((((v__)&0x3)<<13)|(1<<12))
#define REGSET3_CTL(v__)		((((v__)&0x3)<<17)|(1<<16))

/* op_set register offset */
#define GPT_OP_TY_COPY			(0x0 << 0)
#define GPT_OP_TY_FILL			(0x1 << 0)
#define GPT_OP_TY_ZERO_PAD		(0x2 << 0)
#define GPT_OP_TY_COPY_PAD		(0x3 << 0)
#define GPT_OP_RD_SINGAL        (0x0 << 4)
#define GPT_OP_RD_MULTIPLE		(0x1 << 4)
#define GPT_OP_WR_SINGAL        (0x0 << 6)
#define GPT_OP_WR_MULTIPLE		(0x1 << 6)
#define GPT_OP_RB_NUM1			(0x0 << 8)
#define GPT_OP_RB_NUM2			(0x1 << 8)
#define GPT_OP_RB_NUM3			(0x2 << 8)
#define GPT_OP_RB_NUM4			(0x3 << 8)
#define GPT_OP_SZ_1B			(0x0 << 12)
#define GPT_OP_SZ_2B			(0x1 << 12)
#define GPT_OP_SZ_4B			(0x2 << 12)
#define GPT_OP_SZ_8B			(0x3 << 12)
#define GPT_OP_CN_1B			(0x0 << 14)
#define GPT_OP_CN_2B			(0x1 << 14)
#define GPT_OP_CN_4B			(0x3 << 14)

#define GPT_COPY			0x0
#define GPT_FILL			0x1
#define GPT_ZERO_PAD		0x2
#define GPT_COPY_PAD		0x3
#define GPT_RD_SINGAL       0x0
#define GPT_RD_MULTIPLE		0x1
#define GPT_WR_SINGAL       0x0
#define GPT_WR_MULTIPLE		0x1
#define GPT_RB_NUM1			0x1
#define GPT_RB_NUM2			0x2
#define GPT_RB_NUM3			0x3
#define GPT_RB_NUM4			0x4
#define GPT_SIZE_1B         0x1
#define GPT_SIZE_2B         0x2
#define GPT_SIZE_4B         0x4
#define GPT_SIZE_8B         0x8
#define GPT_PAD_1B			0x1
#define GPT_PAD_2B			0x2
#define GPT_PAD_4B			0x4

/*transfer style*/
#define GPT_DMA_MEMCPY                     0x1
#define GPT_DMA_MEMCPY_SG_NO_INTERLEAVE    0x2
#define GPT_DMA_MEMCPY_SG_INTERLEAVE       0x3
#define GPT_DMA_CYCLIC                     0x4

extern void gpt_cdma_tlb_dis(void);
extern void gpt_cdma_tlb_en(void);
extern int gpt_cdma_opsetn_mode(struct dma_chan *chan, u8 setn, u8 type, u8 rddepth, u8 wrdepth, u8 rdbases, u8 size, u8 count); 
extern int gpt_cdma_fill_withsetn(struct dma_chan *chan, u8 setn, u64 data);
extern int gpt_cdma_sctl(bool tlb_en, u8 tlb_num); 
extern	int gpt_cdma_group_para(struct dma_chan *chan, u8 group_id, u16 src_columns, u16 src_rows, u32 src_stride,
		u16 dst_columns, u16 dst_rows, u32 dst_stride);
extern int gpt_cdma_virt(struct dma_chan *chan, u8 virt_id, u64 tx_virt, u64 rx_virt);
extern void gpt_cdma_l2_en(void);
extern void gpt_cdma_l2_dis(void);
extern void gpt_cmda_cache(u8 *src, u64 len);
extern int gpt_cmda_preload_l2(struct dma_chan *chan, u8 *src, u64 columns, u64 rows); 
extern int gpt_strcmp(const char *str1,const char *str2);

#endif
