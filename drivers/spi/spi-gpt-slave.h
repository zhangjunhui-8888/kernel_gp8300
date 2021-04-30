/*
 * spi-gpt-slave.h - Definitions for gpt spi slave driver
 */

#ifndef __GPT_SPI_H__
#define __GPT_SPI_H__

#include <linux/types.h>

#define CONFIG_SYS_MHZ  50
#define MHz (1000*1000)
/* Register offsets */
#define GPT_SPI_CFG			0x00
#define	GPT_SPI_DIV_CNT			0x04
#define	GPT_SPI_BIT_MODE		0x08
#define	GPT_SPI_DLY_CNT			0x0c
#define	GPT_SPI_TRANSFER_CNT		0x10
#define	GPT_SPI_FIFO_CFG		0x14
#define	GPT_SPI_FIFO_CLEAR		0x18
#define	GPT_SPI_IRQ_RAW_STATUS		0x1c
#define	GPT_SPI_IRQ_STATUS		0x20
#define	GPT_SPI_IRQ_MASK		0x24
#define	GPT_SPI_IRQ_CLEAR		0x28
#define	GPT_SPI_RX_DATA			0x2c
#define	GPT_SPI_TX_DATA			0x30
#define	GPT_SPI_FIFO_CUR_CNT		0x34
#define	GPT_SPI_FIFO_OP			0x38
#define	GPT_SPI_SSN_MANUAL	0x3c

/* spi config register */
#define	GPT_SPI_MODE_MASTER		     1 << 0
#define	GPT_SPI_MODE_SLAVE		     0 << 0
#define	GPT_SPI_CPOL		         1 << 1
#define	GPT_SPI_CPHA			     1 << 2
#define	GPT_SPI_MSB				     1 << 3
#define	GPT_SPI_LSB				     0 << 3
#define	GPT_SPI_SSN_SEL				 1 << 4
#define GPT_SPI_SLV_XFER_SAME_EDGE	 1 << 5
#define	GPT_SPI_TRANSFER_EN		     1 << 29
#define	GPT_SPI_TX_DMA_EN		     1 << 30
#define	GPT_SPI_RX_DMA_EN		     1 << 31

#define	GPT_SPI_BIT_32			  0
#define	GPT_SPI_BIT_16		      2
#define	GPT_SPI_BIT_8			  1

/* spi fifo config register */
#define	GPT_SPI_TX_ALMOST_FULL_WIDTH	8
#define	GPT_SPI_TX_ALMOST_FULL_OFFSET	0
#define	GPT_SPI_TX_ALMOST_EMPTY_WIDTH	8
#define	GPT_SPI_TX_ALMOST_EMPTY_OFFSET	8
#define	GPT_SPI_RX_ALMOST_FULL_WIDTH	8
#define	GPT_SPI_RX_ALMOST_FULL_OFFSET	16
#define	GPT_SPI_RX_ALMOST_EMPTY_WIDTH	8
#define	GPT_SPI_RX_ALMOST_EMPTY_OFFSET	24

/* spi fifo clear register */
#define	GPT_SPI_TX_FLUSH			1 << 0
#define	GPT_SPI_RX_FLUSH			1 << 1

/* spi irq raw status register */
#define	GPT_SPI_TX_FIFO_ALMOST_FULL_CLR		1 << 0
#define	GPT_SPI_TX_FIFO_ALMOST_EMPTY_CLR	1 << 1
#define	GPT_SPI_TX_FIFO_FULL_CLR		1 << 2
#define	GPT_SPI_TX_FIFO_EMPTY_CLR		1 << 3
#define	GPT_SPI_TX_FIFO_OVERFLOW_CLR		1 << 4
#define	GPT_SPI_TX_FIFO_UNDERFLOW_CLR		1 << 5
#define	GPT_SPI_RX_FIFO_ALMOST_FULL_CLR		1 << 6
#define	GPT_SPI_RX_FIFO_ALMOST_EMPTY_CLR	1 << 7
#define	GPT_SPI_RX_FIFO_FULL_CLR		1 << 8
#define	GPT_SPI_RX_FIFO_EMPTY_CLR		1 << 9
#define	GPT_SPI_RX_FIFO_OVERFLOW_CLR		1 << 10
#define	GPT_SPI_RX_FIFO_UNDERFLOW_CLR		1 << 11
#define	GPT_SPI_TRANSFER_END_CLR		1 << 12

/* spi irq status register*/
#define	GPT_SPI_TX_FIFO_ALMOST_FULL		1 << 0
#define	GPT_SPI_TX_FIFO_ALMOST_EMPTY		1 << 1
#define	GPT_SPI_TX_FIFO_FULL			1 << 2
#define	GPT_SPI_TX_FIFO_EMPTY			1 << 3
#define	GPT_SPI_TX_FIFO_OVERFLOW		1 << 4
#define	GPT_SPI_TX_FIFO_UNDERFLOW		1 << 5
#define	GPT_SPI_RX_FIFO_ALMOST_FULL		1 << 6
#define	GPT_SPI_RX_FIFO_ALMOST_EMPTY		1 << 7
#define	GPT_SPI_RX_FIFO_FULL			1 << 8
#define	GPT_SPI_RX_FIFO_EMPTY			1 << 9
#define	GPT_SPI_RX_FIFO_OVERFLOW		1 << 10
#define	GPT_SPI_RX_FIFO_UNDERFLOW		1 << 11
#define	GPT_SPI_TRANSFER_END			1 << 12

/* spi irq mask register */
#define	GPT_SPI_TX_FIFO_ALMOST_FULL_MASK	1 << 0
#define	GPT_SPI_TX_FIFO_ALMOST_EMPTY_MASK	1 << 1
#define	GPT_SPI_TX_FIFO_FULL_MASK		1 << 2
#define	GPT_SPI_TX_FIFO_EMPTY_MASK		1 << 3
#define	GPT_SPI_TX_FIFO_OVERFLOW_MASK		1 << 4
#define	GPT_SPI_TX_FIFO_UNDERFLOW_MASK		1 << 5
#define	GPT_SPI_RX_FIFO_ALMOST_FULL_MASK	1 << 6
#define	GPT_SPI_RX_FIFO_ALMOST_EMPTY_MASK	1 << 7
#define	GPT_SPI_RX_FIFO_FULL_MASK		1 << 8
#define	GPT_SPI_RX_FIFO_EMPTY_MASK		1 << 9
#define	GPT_SPI_RX_FIFO_OVERFLOW_MASK		1 << 10
#define	GPT_SPI_RX_FIFO_UNDERFLOW_MASK		1 << 11
#define	GPT_SPI_TRANSFER_END_MASK		1 << 12
#define	GPT_SPI_MASK_WIDTH			13

/* spi irq clear register */
#define	GPT_SPI_TX_FIFO_ALMOST_FULL_CLR		1 << 0
#define	GPT_SPI_TX_FIFO_ALMOST_EMPTY_CLR	1 << 1
#define	GPT_SPI_TX_FIFO_FULL_CLR		1 << 2
#define	GPT_SPI_TX_FIFO_EMPTY_CLR		1 << 3
#define	GPT_SPI_TX_FIFO_OVERFLOW_CLR		1 << 4
#define	GPT_SPI_TX_FIFO_UNDERFLOW_CLR		1 << 5
#define	GPT_SPI_RX_FIFO_ALMOST_FULL_CLR		1 << 6
#define	GPT_SPI_RX_FIFO_ALMOST_EMPTY_CLR	1 << 7
#define	GPT_SPI_RX_FIFO_FULL_CLR		1 << 8
#define	GPT_SPI_RX_FIFO_EMPTY_CLR		1 << 9
#define	GPT_SPI_RX_FIFO_OVERFLOW_CLR		1 << 10
#define	GPT_SPI_RX_FIFO_UNDERFLOW_CLR		1 << 11
#define	GPT_SPI_TRANSFER_END_CLR		1 << 12

#define	GPT_SPI_TX_ENABLE			1 << 0
#define	GPT_SPI_TX_DISABLE			0 << 0
#define	GPT_SPI_RX_ENABLE			1 << 1
#define	GPT_SPI_RX_DISABLE			0 << 0

/* spi ssn manual register */
#define	GPT_SPI_SSN_MANUAL_DISABLE		1 << 0

#define GPT_SPI_FIFO_DEPTH 64

#define GPT_SPI_TIMEOUT_MS 30000

#define SPI_CPHA		0x01
#define SPI_CPOL		0x02

#define SPI_MODE_0		(0|0)
#define SPI_MODE_1		(0|SPI_CPHA)
#define SPI_MODE_2		(SPI_CPOL|0)
#define SPI_MODE_3		(SPI_CPOL|SPI_CPHA)

/* IOCTL commands */

#define SPI_IOC_MAGIC			'k'

/* Read*/
#define SPI_IOC_RD_BITS_PER_WORD	_IOR(SPI_IOC_MAGIC, 1, __u8)
#define SPI_IOC_RD_MODE32		    _IOR(SPI_IOC_MAGIC, 2, __u32)
#define SPI_IOC_RD_DATA_LEN         _IOR(SPI_IOC_MAGIC, 3, __u16)

/*Write*/
#define SPI_IOC_WR_BITS_PER_WORD	_IOW(SPI_IOC_MAGIC, 1, __u8)
#define SPI_IOC_WR_MODE32		    _IOW(SPI_IOC_MAGIC, 2, __u32)
#define SPI_IOC_WR_TX_BUF_NUM       _IOW(SPI_IOC_MAGIC, 3, __u8)
#define SPI_IOC_WR_TX_BUF_SIZE      _IOW(SPI_IOC_MAGIC, 4, __u16)
#define SPI_IOC_WR_REQUEST_BUF      _IOW(SPI_IOC_MAGIC, 5, __u8)
#define SPI_IOC_WR_RELEASE_BUF      _IOW(SPI_IOC_MAGIC, 6, __u8)
#define SPI_IOC_WR_TRANSFER_EN      _IOW(SPI_IOC_MAGIC, 7, __u8)
#endif
