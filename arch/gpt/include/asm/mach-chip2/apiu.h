#ifndef _APIU_H_
#define _APIU_H_

#define GPIO_UNIT_CFG     0xFFC
#define GPIO_DATA_ALL     0x000
#define GPIO_DATA_BITS    0x004
#define GPIO_DIR          0x400
#define GPIO_IT           0x404
#define GPIO_IR           0x408
#define GPIO_IF           0x40C
#define GPIO_IE           0x410
#define GPIO_IS           0x414
#define GPIO_IC           0x41C

/* "mask" defines the bits to be accessed; mask = 0x0001, will write bit 0 */ 
#define GPIO_WDATA(mask, data)  (((mask & 0x00FF) << 8) | (data & 0x00FF))

#define GPIO_PERIPHID0    0x0FE0
#define GPIO_PERIPHID1    0x0FE4
#define GPIO_PERIPHID2    0x0FE8
#define GPIO_PERIPHID3    0x0FEC


/* 
 * Interface Unit Specific constants
 */

/*  GPIO Status Flags */
#define GPIO_MASKED_B0            (0x1 << 0)
#define GPIO_MASKED_B1            (0x1 << 1)
#define GPIO_MASKED_B2            (0x1 << 2)
#define GPIO_MASKED_B3            (0x1 << 3)
#define GPIO_MASKED_B4            (0x1 << 4)
#define GPIO_MASKED_B5            (0x1 << 5)
#define GPIO_MASKED_B6            (0x1 << 6)
#define GPIO_MASKED_B7            (0x1 << 7)
#define GPIO_MASKED_COMP          (0x1 << 8)

#define GPIO_RAW_B0               (GPIO_MASKED_B0         << 16)
#define GPIO_RAW_B1               (GPIO_MASKED_B1         << 16)
#define GPIO_RAW_B2               (GPIO_MASKED_B2         << 16)
#define GPIO_RAW_B3               (GPIO_MASKED_B3         << 16)
#define GPIO_RAW_B4               (GPIO_MASKED_B4         << 16)
#define GPIO_RAW_B5               (GPIO_MASKED_B5         << 16)
#define GPIO_RAW_B6               (GPIO_MASKED_B6         << 16)
#define GPIO_RAW_B7               (GPIO_MASKED_B7         << 16)
#define GPIO_RAW_COMP             (GPIO_MASKED_COMP       << 16)

/*  GPIO Control Register Definitions */
#define GPIO_EN                   (0x1)
#define GPIO_B0_EN                (GPIO_MASKED_B0         << 16)
#define GPIO_B1_EN                (GPIO_MASKED_B1         << 16)
#define GPIO_B2_EN                (GPIO_MASKED_B2         << 16)
#define GPIO_B3_EN                (GPIO_MASKED_B3         << 16)
#define GPIO_B4_EN                (GPIO_MASKED_B4         << 16)
#define GPIO_B5_EN                (GPIO_MASKED_B5         << 16)
#define GPIO_B6_EN                (GPIO_MASKED_B6         << 16)
#define GPIO_B7_EN                (GPIO_MASKED_B7         << 16)
#define GPIO_COMP_EN              (GPIO_MASKED_COMP       << 16)

/*  GPIO Data Direction Register Definitions */
#define GPIO_BIT0_INPUT             (0x1 << 0)
#define GPIO_BIT1_INPUT             (0x1 << 1)
#define GPIO_BIT2_INPUT             (0x1 << 2)
#define GPIO_BIT3_INPUT             (0x1 << 3)
#define GPIO_BIT4_INPUT             (0x1 << 4)
#define GPIO_BIT5_INPUT             (0x1 << 5)
#define GPIO_BIT6_INPUT             (0x1 << 6)
#define GPIO_BIT7_INPUT             (0x1 << 7)
#define GPIO_INPUT                   (0x1)
#define GPIO_OUTPUT                  (0x0)

/*  GPIO Interrupt Type Register Definitions */
#define GPIO_BIT0_EDGE_INT          (0x1 << 0)
#define GPIO_BIT1_EDGE_INT          (0x1 << 1)
#define GPIO_BIT2_EDGE_INT          (0x1 << 2)
#define GPIO_BIT3_EDGE_INT          (0x1 << 3)
#define GPIO_BIT4_EDGE_INT          (0x1 << 4)
#define GPIO_BIT5_EDGE_INT          (0x1 << 5)
#define GPIO_BIT6_EDGE_INT          (0x1 << 6)
#define GPIO_BIT7_EDGE_INT          (0x1 << 7) 
#define GPIO_EDGE_INT                (0x1)
#define GPIO_LEVEL_INT               (0x0)

/*  GPIO Rising/High-Level Enable Register Definitions */
#define GPIO_RISING_EDGE             (0x1)
#define GPIO_HIGH_LEVEL              (0x0)

/*  GPIO Falling/Low-Level Enable Register Definitions */
#define GPIO_BIT0_FALLING_EDGE         (0x1 << 0)
#define GPIO_BIT1_FALLING_EDGE         (0x1 << 1)
#define GPIO_BIT2_FALLING_EDGE         (0x1 << 2)
#define GPIO_BIT3_FALLING_EDGE         (0x1 << 3)
#define GPIO_BIT4_FALLING_EDGE         (0x1 << 4)
#define GPIO_BIT5_FALLING_EDGE         (0x1 << 5)
#define GPIO_BIT6_FALLING_EDGE         (0x1 << 6)
#define GPIO_BIT7_FALLING_EDGE         (0x1 << 7) 
#define GPIO_FALLING_EDGE            (0x1)
#define GPIO_LOW_LEVEL               (0x0)

/*  GPIO Interrupt Enable Register Definitions */
#define GPIO_BIT0_INT_EN         (0x1 << 0)
#define GPIO_BIT1_INT_EN         (0x1 << 1)
#define GPIO_BIT2_INT_EN         (0x1 << 2)
#define GPIO_BIT3_INT_EN         (0x1 << 3)
#define GPIO_BIT4_INT_EN         (0x1 << 4)
#define GPIO_BIT5_INT_EN         (0x1 << 5)
#define GPIO_BIT6_INT_EN         (0x1 << 6)
#define GPIO_BIT7_INT_EN         (0x1 << 7) 
#define GPIO_INT_EN                  (0x1)
#define GPIO_INT_DIS                 (0x0)


#define GPIO_INT_CLR                  (0x1)

/*  GPIO DATA write/read Register Definitions */
#define GPIO_BIT0_WR         (0x1 << 0)
#define GPIO_BIT1_WR         (0x1 << 1)
#define GPIO_BIT2_WR         (0x1 << 2)
#define GPIO_BIT3_WR         (0x1 << 3)
#define GPIO_BIT4_WR         (0x1 << 4)
#define GPIO_BIT5_WR         (0x1 << 5)
#define GPIO_BIT6_WR         (0x1 << 6)
#define GPIO_BIT7_WR         (0x1 << 7)

#define APIU_UNIT_UCR			0xFFC
#define APIU_UNIT_USFR			0xFF8

/**
 * operations for peripherals
 */

#define APIU_RQST_FIFO_WRITE    0
#define APIU_RQST_FIFO_READ     1

/* I2C/SPI/UART Status Flags */
#define UNIT_MASKED_IRQ           (0x1 << 0)
#define UNIT_MASKED_RESP_DAVAIL   (0x1 << 1)
#define UNIT_MASKED_RQST_QEPTY    (0x1 << 2)
#define UNIT_MASKED_RQST_HFULL    (0x1 << 3)
#define UNIT_MASKED_RQST_FULL     (0x1 << 4)
#define UNIT_MASKED_BRX_HIGH      (0x1 << 5)
#define UNIT_MASKED_BRX_LOW       (0x1 << 6)
#define UNIT_MASKED_BRX_EPTY      (0x1 << 7)
#define UNIT_MASKED_BRX_FULL      (0x1 << 8)
#define UNIT_MASKED_BTX_HIGH      (0x1 << 9)
#define UNIT_MASKED_BTX_LOW       (0x1 << 10)
#define UNIT_MASKED_BTX_EPTY      (0x1 << 11)
#define UNIT_MASKED_BTX_FULL      (0x1 << 12)

#define UNIT_RAW_IRQ              (UNIT_MASKED_IRQ         << 16)
#define UNIT_RAW_RESP_DAVAIL      (UNIT_MASKED_RESP_DAVAIL << 16)
#define UNIT_RAW_RQST_QEPTY       (UNIT_MASKED_RQST_QEPTY  << 16)
#define UNIT_RAW_RQST_HFULL       (UNIT_MASKED_RQST_HFULL  << 16)
#define UNIT_RAW_RQST_FULL        (UNIT_MASKED_RQST_FULL   << 16)
#define UNIT_RAW_BRX_HIGH         (UNIT_MASKED_BRX_HIGH    << 16)
#define UNIT_RAW_BRX_LOW          (UNIT_MASKED_BRX_LOW     << 16)
#define UNIT_RAW_BRX_EPTY         (UNIT_MASKED_BRX_EPTY    << 16)
#define UNIT_RAW_BRX_FULL         (UNIT_MASKED_BRX_FULL    << 16)
#define UNIT_RAW_BTX_HIGH         (UNIT_MASKED_BTX_HIGH    << 16)
#define UNIT_RAW_BTX_LOW          (UNIT_MASKED_BTX_LOW     << 16)
#define UNIT_RAW_BTX_EPTY         (UNIT_MASKED_BTX_EPTY    << 16)
#define UNIT_RAW_BTX_FULL         (UNIT_MASKED_BTX_FULL    << 16)

#define UNIT_IRQ_EN               (UNIT_RAW_IRQ)
#define UNIT_RESP_DAVAIL_EN       (UNIT_RAW_RESP_DAVAIL)
#define UNIT_RQST_QEPTY_EN        (UNIT_RAW_RQST_QEPTY)
#define UNIT_RQST_HFULL_EN        (UNIT_RAW_RQST_HFULL)
#define UNIT_RQST_FULL_EN         (UNIT_RAW_RQST_FULL)
#define UNIT_BRX_HIGH_EN          (UNIT_RAW_BRX_HIGH)
#define UNIT_BRX_LOW_EN           (UNIT_RAW_BRX_LOW)
#define UNIT_BRX_EPTY_EN          (UNIT_RAW_BRX_EPTY)
#define UNIT_BRX_FULL_EN          (UNIT_RAW_BRX_FULL)
#define UNIT_BTX_HIGH_EN          (UNIT_RAW_BTX_HIGH)
#define UNIT_BTX_LOW_EN           (UNIT_RAW_BTX_LOW)
#define UNIT_BTX_EPTY_EN          (UNIT_RAW_BTX_EPTY)
#define UNIT_BTX_FULL_EN          (UNIT_RAW_BTX_FULL)

#define UNIT_EN					  (0x1)
#define UNIT_BQ_DIS               (0x2)
#define UART_IRDA_EN              (0x4)
#define UNIT_BTXL_1Q              (0x0 << 8)
#define UNIT_BTXL_2Q              (0x1 << 8)
#define UNIT_BTXL_3Q              (0x2 << 8)
#define UNIT_BTXH_1Q              (0x0 << 10)
#define UNIT_BTXH_2Q              (0x1 << 10)
#define UNIT_BTXH_3Q              (0x2 << 10)
#define UNIT_BRXL_1Q              (0x0 << 12)
#define UNIT_BRXL_2Q              (0x1 << 12)
#define UNIT_BRXL_3Q              (0x2 << 12)
#define UNIT_BRXH_1Q              (0x0 << 14)
#define UNIT_BRXH_2Q              (0x1 << 14)
#define UNIT_BRXH_3Q              (0x2 << 14)

#define UNIT_BURST_DFLT           (0x8800)
#define DEV_MASK		   (~0xFFFul)

#define	APIU_CCCR1_OFFSET				0x00000000
#define	APIU_CCCR2_OFFSET				0x00000004
#define	APIU_CCCR3_OFFSET				0x00000008
#define	APIU_CCCR4_OFFSET				0x0000000C
#define	APIU_ICCR_OFFSET				0x00000010
#define	APIU_IOCR_OFFSET				0x00000014
#define	APIU_ISLR1_OFFSET				0x00000020
#define	APIU_ISLR2_OFFSET				0x00000024
#define	APIU_ISLR3_OFFSET				0x00000028
#define	APIU_ISLR4_OFFSET				0x0000002C

enum APIU_DEV{
	APIU_MPINT		=			0xF0007000,
	APIU_GPIO1		=			0xF0008000,
	APIU_GPIO2		=			0xF0009000,
	APIU_GPIO3		=			0xF000A000,
	APIU_GPIO4		=			0xF000B000,
	APIU_GPIO5		=			0xF000C000,
	APIU_GPIO6		=			0xF000D000,
	APIU_GPIO7		=			0xF000E000,
	APIU_GPIO8		=			0xF000F000,

	APIU_UART1		=			0xF0010000,
	APIU_UART2		=			0xF0011000,
	APIU_UART3		=			0xF0012000,
	APIU_UART4		=			0xF0013000,

	APIU_I2S1		=			0xF0014000,
#define APIU_I2S1_TXL		   (APIU_I2S1 | 0x10)
#define APIU_I2S1_TXR  		   (APIU_I2S1 | 0x14)
#define APIU_I2S1_RXL  		   (APIU_I2S1 | 0x20)
#define APIU_I2S1_RXR  		   (APIU_I2S1 | 0x24)
	APIU_I2S2		=			0xF0015000,
#define APIU_I2S2_TXL		   (APIU_I2S2 | 0x10)
#define APIU_I2S2_TXR  		   (APIU_I2S2 | 0x14)
#define APIU_I2S2_RXL  		   (APIU_I2S2 | 0x20)
#define APIU_I2S2_RXR  		   (APIU_I2S2 | 0x24)

	APIU_GPTIMER1	=			0xF0016000,
	APIU_GPTIMER2	=			0xF0016100,
	APIU_GPTIMER3	=			0xF0016200,
	APIU_GPTIMER4	=			0xF0016300,
	APIU_WDTIMER	=			0xF0017000,

	APIU_SPI1		=			0xF0018000,
	APIU_SPI2		=			0xF0019000,
	APIU_SPI3		=			0xF001A000,
	APIU_SPI4		=			0xF001B000,

	APIU_I2C1		=			0xF001C000,
	APIU_I2C2		=			0xF001D000,
	APIU_I2C3		=			0xF001E000,
	APIU_I2C4		=			0xF001F000,
};

#define APIU_BURST_ADDR			0x0FF0
#define APIU_USFR				0x0FF8
#define APIU_UCR				0x0FFC

struct _sysctl{
	unsigned long ctrl;
	unsigned long mpint;
};

struct _gpio{
	unsigned long gpio1;
	unsigned long gpio2;
	unsigned long gpio3;
	unsigned long gpio4;
	unsigned long gpio5;
	unsigned long gpio6;
	unsigned long gpio7;
	unsigned long gpio8;
};

struct _uart{
	unsigned long uart1;
	unsigned long uart2;
	unsigned long uart3;
	unsigned long uart4;
};

struct _i2s{
	unsigned long i2s1;
	unsigned long i2s2;
};

struct _timer{
	unsigned long timer1;
	unsigned long timer2;
	unsigned long timer3;
	unsigned long timer4;
	unsigned long wdtimer;
};

struct _spi{
	unsigned long spi1;
	unsigned long spi2;
};

struct _i2c{
	unsigned long i2c1;
	unsigned long i2c2;
	unsigned long i2c3;
	unsigned long i2c4;
};

typedef struct apiu_regs {
	struct _sysctl sysctl;
	struct _gpio gpio;
	struct _uart uart;
	struct _i2s i2s;
	struct _timer timer;
	struct _spi spi;
	struct _i2c i2c;
} apiu_regs_t;

extern u16 apiu_readw(void __iomem *addr);
extern u32 apiu_readl(void __iomem *addr);
extern void apiu_writeb(u8 val, void __iomem *addr);
extern void apiu_writew(u16 val, void __iomem *addr);
extern void apiu_writel(u32 val, void __iomem *addr);
extern u8 apiu_burst_readb(void __iomem *addr);
extern void apiu_burst_writeb(u8 val, void __iomem *addr);
extern void apiu_unit_init(void __iomem *addr, int burst);
extern void apiu_unit_uninit(void __iomem *addr);

extern void __iomem *gpio_base_addr;
#endif /* _APIU_H_ */
