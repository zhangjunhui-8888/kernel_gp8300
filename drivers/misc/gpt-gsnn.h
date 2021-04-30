#ifndef __GPT_GSNN_H__
#define __GPT_GSNN_H__

#include <linux/io.h>
#include <linux/cdev.h>
#include <linux/completion.h>

#define GPT_GSNN_MAJOR		200
#define GPT_GSNN0_MINOR		0
#define GPT_GSNN1_MINOR		1

#define GPT_GSNN_CODE_SIZE	(2 * 1024 * 1024)

#define GSNN_REG_G00		0x20000
#define GSNN_REG_G01		0x20004
#define GSNN_REG_G02		0x20008
#define GSNN_REG_G03		0x2000C
#define GSNN_REG_G04		0x20010
#define GSNN_REG_G05		0x20014
#define GSNN_REG_G06		0x20018
#define GSNN_REG_G07		0x2001C
#define GSNN_REG_G08		0x20020
#define GSNN_REG_G09		0x20024
#define GSNN_REG_G10		0x20028
#define GSNN_REG_G11		0x2002C
#define GSNN_REG_G12		0x20030
#define GSNN_REG_G13		0x20034
#define GSNN_REG_G14		0x20038
#define GSNN_REG_G15		0x2003C
#define GSNN_REG_INTCTL		0x200C0
#define GSNN_REG_INTSET		0x200C4
#define GSNN_REG_INTCLR		0x200C8
#define GSNN_REG_NEXT		0x200D0
#define GSNN_REG_QOS		0x200E0
#define GSNN_REG_ERRST		0x200F0
#define GSNN_REG_PC(n)		(0x20300 | ((n) << 6))

#define GSNN_OF_INT_ERR		(0x1 << 11)
#define GSNN_OF_INT_REQ		(0x1 << 10)
#define GSNN_OF_REQ_NOPERF	(0x1 << 9)
#define GSNN_OF_CPY_DONE	(0x1 << 8)
#define GSNN_OF_ERR_EN		(0x1 << 7)
#define GSNN_OF_REQ_EN		(0x1 << 6)
#define GSNN_OF_NOPREF_EN	(0x1 << 5)
#define GSNN_OF_CPYDN_EN	(0x1 << 4)
#define GSNN_OF_ERR_CLR		(0x1 << 3)
#define GSNN_OF_EM_ERR		(0x1 << 2)
#define GSNN_OF_RUNING		(0x1 << 0)


#define GSNN_OF_INPUT_UNALIGN	(0x1 << 0)
#define GSNN_OF_AXI_4K		(0x1 << 1)
#define GSNN_OF_OUTPUT_UNALIGN	(0x1 << 2)

#define GSNN_G15_NOINT_NOCPY	0x0
#define GSNN_G15_INT_NOCPY	0x1
#define GSNN_G15_NOINT_CPY	0x2
#define GSNN_G15_INT_CPY	0x3
#define GSNN_OF_G15_TYPE(n)	(((n) & 0x3) << 30)

#define GPT_PXI_SPR_EN		0x1
#define GPT_PXI_MASK		0xfffffff

#define gsnn_write(gsnn, offset, data)	writel(data, ((void *)gsnn->regs + (offset)))
#define gsnn_read(gsnn, offset)  readl(((void *)gsnn->regs + (offset)))


#define PDEBUG(fmt, args...)			\
	do {					\
		if (debug_level)		\
			printk("GSNN: "fmt, ## args);	\
	} while (0)

#define PERROR(fmt,args...)			\
	do {					\
		printk("GSNN: "fmt, ## args);		\
	} while (0)

extern int debug_level;

struct gpt_gsnn {
	struct task_struct *ts;
	struct cdev gsnn_cdev;
	struct device *gsnn_dev;
	unsigned char *regs;
	unsigned char *mem;
	unsigned char *codemem;
	struct completion irq_detect;
	struct mutex	lock;
	spinlock_t irq_spinlock;
	wait_queue_head_t gsnn_wq;
	int gsnn_run_status;
	int irq;
	unsigned long hwregs;
	unsigned long hwdata_base;
	unsigned long hwdata_size;
};

#endif
