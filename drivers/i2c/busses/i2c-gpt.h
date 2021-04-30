/*
 * Synopsys DesignWare I2C adapter driver (master only).
 *
 * Based on the TI DAVINCI I2C adapter driver.
 *
 * Copyright (C) 2006 Texas Instruments.
 * Copyright (C) 2007 MontaVista Software Inc.
 * Copyright (C) 2009 Provigent Ltd.
 *
 * ----------------------------------------------------------------------------
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * ----------------------------------------------------------------------------
 *
 */

#include <linux/i2c.h>


#define GPT_IC_CON_MASTER		0x1
#define GPT_IC_CON_SPEED_STD		0x2
#define GPT_IC_CON_SPEED_FAST		0x4
#define GPT_IC_CON_SPEED_HIGH           0x6
#define GPT_IC_CON_10BITADDR_MASTER	0x10
#define GPT_IC_CON_RESTART_EN		0x20
#define GPT_IC_CON_SLAVE_DISABLE		0x40
/*
 * Registers offset
 */
#define GPT_IC_CON		0x0
#define GPT_IC_TAR		0x4
#define GPT_IC_SAR              0x8
#define GPT_IC_DATA_CMD		0x10
#define GPT_IC_SS_SCL_HCNT	0x14
#define GPT_IC_SS_SCL_LCNT	0x18
#define GPT_IC_FS_SCL_HCNT	0x1c
#define GPT_IC_FS_SCL_LCNT	0x20
#define GPT_IC_HS_SCL_HCNT      0x24
#define GPT_IC_HS_SCL_LCNT      0x28
#define GPT_IC_INTR_STAT		0x2c
#define GPT_IC_INTR_MASK		0x30
#define GPT_IC_RAW_INTR_STAT	0x34
#define GPT_IC_RX_TL		0x38
#define GPT_IC_TX_TL		0x3c
#define GPT_IC_CLR_INTR		0x40
#define GPT_IC_CLR_RX_UNDER	0x44
#define GPT_IC_CLR_RX_OVER	0x48
#define GPT_IC_CLR_TX_OVER	0x4c
#define GPT_IC_CLR_RD_REQ	0x50
#define GPT_IC_CLR_TX_ABRT	0x54
#define GPT_IC_CLR_RX_DONE	0x58
#define GPT_IC_CLR_ACTIVITY	0x5c
#define GPT_IC_CLR_STOP_DET	0x60
#define GPT_IC_CLR_START_DET	0x64
#define GPT_IC_CLR_GEN_CALL	0x68
#define GPT_IC_ENABLE		0x6c
#define GPT_IC_STATUS		0x70
#define GPT_IC_TXFLR		0x74
#define GPT_IC_RXFLR		0x78
#define GPT_IC_SDA_HOLD		0x7c
#define GPT_IC_TX_ABRT_SOURCE	0x80
#define GPT_IC_ENABLE_STATUS	0x9c
#define GPT_IC_COMP_PARAM_1	0xf4
#define GPT_IC_COMP_VERSION	0xf8
#define GPT_IC_SDA_HOLD_MIN_VERS	0x3131312A
#define GPT_IC_COMP_TYPE		0xfc
#define GPT_IC_COMP_TYPE_VALUE	0x44570140

#define GPT_IC_INTR_RX_UNDER	0x001
#define GPT_IC_INTR_RX_OVER	0x002
#define GPT_IC_INTR_RX_FULL	0x004
#define GPT_IC_INTR_TX_OVER	0x008
#define GPT_IC_INTR_TX_EMPTY	0x010
#define GPT_IC_INTR_RD_REQ	0x020
#define GPT_IC_INTR_TX_ABRT	0x040
#define GPT_IC_INTR_RX_DONE	0x080
#define GPT_IC_INTR_ACTIVITY	0x100
#define GPT_IC_INTR_STOP_DET	0x200
#define GPT_IC_INTR_START_DET	0x400
#define GPT_IC_INTR_GEN_CALL	0x800


#define GPT_IC_STATUS_ACTIVITY	0x1

#define GPT_IC_ERR_TX_ABRT	0x1

#define GPT_IC_TAR_10BITADDR_MASTER BIT(12)

#define GPT_IC_STATUS_SLAVE_ACTIVITY  BIT(6)

/*
 * status codes
 */
#define STATUS_IDLE			0x0
#define STATUS_WRITE_IN_PROGRESS	0x1
#define STATUS_READ_IN_PROGRESS		0x2

#define TIMEOUT			20 /* ms */

/*
 * hargptare abort codes from the GPT_IC_TX_ABRT_SOURCE register
 *
 * only expected abort codes are listed here
 * refer to the datasheet for the full list
 */
#define ABRT_7B_ADDR_NOACK	0
#define ABRT_10ADDR1_NOACK	1
#define ABRT_10ADDR2_NOACK	2
#define ABRT_TXDATA_NOACK	3
#define ABRT_GCALL_NOACK	4
#define ABRT_GCALL_READ		5
#define ABRT_SBYTE_ACKDET	7
#define ABRT_SBYTE_NORSTRT	9
#define ABRT_10B_RD_NORSTRT	10
#define ABRT_MASTER_DIS		11
#define ARB_LOST		12

#define GPT_IC_TX_ABRT_7B_ADDR_NOACK	(1UL << ABRT_7B_ADDR_NOACK)
#define GPT_IC_TX_ABRT_10ADDR1_NOACK	(1UL << ABRT_10ADDR1_NOACK)
#define GPT_IC_TX_ABRT_10ADDR2_NOACK	(1UL << ABRT_10ADDR2_NOACK)
#define GPT_IC_TX_ABRT_TXDATA_NOACK	(1UL << ABRT_TXDATA_NOACK)
#define GPT_IC_TX_ABRT_GCALL_NOACK	(1UL << ABRT_GCALL_NOACK)
#define GPT_IC_TX_ABRT_GCALL_READ	(1UL << ABRT_GCALL_READ)
#define GPT_IC_TX_ABRT_SBYTE_ACKDET	(1UL << ABRT_SBYTE_ACKDET)
#define GPT_IC_TX_ABRT_SBYTE_NORSTRT	(1UL << ABRT_SBYTE_NORSTRT)
#define GPT_IC_TX_ABRT_10B_RD_NORSTRT	(1UL << ABRT_10B_RD_NORSTRT)
#define GPT_IC_TX_ABRT_MASTER_DIS	(1UL << ABRT_MASTER_DIS)
#define GPT_IC_TX_ARB_LOST		(1UL << ARB_LOST)

#define GPT_IC_TX_ABRT_NOACK		(GPT_IC_TX_ABRT_7B_ADDR_NOACK | \
					 GPT_IC_TX_ABRT_10ADDR1_NOACK | \
					 GPT_IC_TX_ABRT_10ADDR2_NOACK | \
					 GPT_IC_TX_ABRT_TXDATA_NOACK | \
					 GPT_IC_TX_ABRT_GCALL_NOACK)


/**
 * struct gpt_i2c_dev - private i2c-designware data
 * @dev: driver model device node
 * @base: IO registers pointer
 * @cmd_complete: tx completion indicator
 * @lock: protect this struct and IO registers
 * @clk: input reference clock
 * @cmd_err: run time hardware error code
 * @msgs: points to an array of messages currently being transfered
 * @msgs_num: the number of elements in msgs
 * @msg_write_idx: the element index of the current tx message in the msgs
 *	array
 * @tx_buf_len: the length of the current tx buffer
 * @tx_buf: the current tx buffer
 * @msg_read_idx: the element index of the current rx message in the msgs
 *	array
 * @rx_buf_len: the length of the current rx buffer
 * @rx_buf: the current rx buffer
 * @msg_err: error status of the current transfer
 * @status: i2c master status, one of STATUS_*
 * @abort_source: copy of the TX_ABRT_SOURCE register
 * @irq: interrupt number for the i2c master
 * @adapter: i2c subsystem adapter node
 * @tx_fifo_depth: depth of the hardware tx fifo
 * @rx_fifo_depth: depth of the hardware rx fifo
 * @rx_outstanding: current master-rx elements in tx fifo
 * @ss_hcnt: standard speed HCNT value
 * @ss_lcnt: standard speed LCNT value
 * @fs_hcnt: fast speed HCNT value
 * @fs_lcnt: fast speed LCNT value
 *
 * HCNT and LCNT parameters can be used if the platform knows more accurate
 * values than the one computed based only on the input clock frequency.
 * Leave them to be %0 if not used.
 */
struct gpt_i2c_dev {
	struct device		*dev;
	void __iomem		*base;
	struct completion	cmd_complete;
	struct mutex		lock;
	struct clk		*clk;
	struct i2c_client	*slave;
	u32			(*get_clk_rate_khz) (struct gpt_i2c_dev *dev);
	struct gpt_pci_controller *controller;
	int			cmd_err;
	struct i2c_msg		*msgs;
	int			msgs_num;
	int			msg_write_idx;
	u32			tx_buf_len;
	u8			*tx_buf;
	int			msg_read_idx;
	u32			rx_buf_len;
	u8			*rx_buf;
	int			msg_err;
	unsigned int		status;
	u32			abort_source;
	int			irq;
	u32			accessor_flags;
	struct i2c_adapter	adapter;
	u32			functionality;
	u32			master_cfg;
	u32			slave_cfg;
	unsigned int		tx_fifo_depth;
	unsigned int		rx_fifo_depth;
	int			rx_outstanding;
	u32			sda_hold_time;
	u32			sda_falling_time;
	u32			scl_falling_time;
	u16			ss_hcnt;
	u16			ss_lcnt;
	u16			fs_hcnt;
	u16			fs_lcnt;
	u16                     hs_hcnt;
	u16                     hs_lcnt;
	int			(*acquire_lock)(void);
	void		(*release_lock)(void);
	int			(*set_sda_hold_time)(struct gpt_i2c_dev *dev);
	u8			buffer[];
};

#define ACCESS_SWAP		0x00000001
#define ACCESS_16BIT		0x00000002

extern u32 gpt_readl(struct gpt_i2c_dev *dev, int offset);
extern void gpt_writel(struct gpt_i2c_dev *dev, u32 b, int offset);
extern int i2c_gpt_init(struct gpt_i2c_dev *dev);
extern int i2c_gpt_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[],
		int num);
extern u32 i2c_gpt_func(struct i2c_adapter *adap);
extern irqreturn_t i2c_gpt_isr(int this_irq, void *dev_id);
extern void i2c_gpt_enable(struct gpt_i2c_dev *dev);
extern u32 i2c_gpt_is_enabled(struct gpt_i2c_dev *dev);
extern void i2c_gpt_disable(struct gpt_i2c_dev *dev);
extern void i2c_gpt_clear_int(struct gpt_i2c_dev *dev);
extern void i2c_gpt_disable_int(struct gpt_i2c_dev *dev);
extern u32 i2c_gpt_read_comp_param(struct gpt_i2c_dev *dev);
