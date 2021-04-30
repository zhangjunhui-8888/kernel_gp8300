/*
 * OHS900 register declarations and HCD data structures
 *
 * Copyright (C) 2005 Steve Fielding
 * Copyright (C) 2004 Psion Teklogix
 * Copyright (C) 2004 David Brownell
 */
#include <linux/ohs900.h>
//#include <asm/arch/sam255.h>

#define DEVELOP
#define DEBUGING
// base address of OHS900

#define OHS900_BASE		0x9c000000	//(DIMI_USB_HOST_PHYS)
#define OHS900_IRQ		20	//(SAM255_INTERRUPT_USB_HOST)
#define OHS900SLAVE_BASE	0x9c000000	//(DIMI_USB_HOST_PHYS)
#define OHS900SLAVE_IRQ		21	//(SAM255_INTERRUPT_USB_HOST)
#define OHS900_IO_EXTENT 0x100
#define OHS900_SLAVE_ADDRESS		0x54

/*
 * OHS900 has transfer registers, and control registers.  In host/master
 * mode one set of registers is used; in peripheral/slave mode, another.
 */

/* TRANSFER REGISTERS
 */
#define OHS900_HOST_TX_CTLREG	0x00
#	define OHS900_HCTLMASK_TRANS_REQ		0x01
#	define OHS900_HCTLMASK_SOF_SYNC		0x02
#	define OHS900_HCTLMASK_PREAMBLE_EN	0x04
#	define OHS900_HCTLMASK_ISO_EN		0x08

#define OHS900_HRXSTATREG	0x0a	/* read */
#	define OHS900_STATMASK_CRC_ERROR		0x01
#	define OHS900_STATMASK_BS_ERROR		0x02
#	define OHS900_STATMASK_RX_OVF		0x04
#	define OHS900_STATMASK_RX_TMOUT		0x08
#	define OHS900_STATMASK_NAK_RXED		0x10
#	define OHS900_STATMASK_STALL_RXED	0x20
#	define OHS900_STATMASK_ACK_RXED		0x40
#	define OHS900_STATMASK_DATA_SEQ		0x80

#define OHS900_TXTRANSTYPEREG		0x01	/* write */
#	define	OHS900_SETUP	0x00
#	define	OHS900_IN	0x01
#	define	OHS900_OUT_DATA0	0x02
#	define	OHS900_OUT_DATA1	0x03

#define OHS900_TXADDRREG	0x04
#define OHS900_TXENDPREG	0x05

/* CONTROL REGISTERS:  
 */
#define OHS900_SOFENREG		0x03
#define OHS900_MASK_SOF_ENA	0x01

#define OHS900_TXLINECTLREG 0x02
#define OHS900_TXLCTL_MASK_FORCE	0x4
#define OHS900_TXLCTL_MASK_LINE_CTRL_BITS 0x7
#define OHS900_TXLCTL_MASK_NORMAL	0x00
#define OHS900_TXLCTL_MASK_SE0	0x04
#define OHS900_TXLCTL_MASK_FS_J	0x06
#define OHS900_TXLCTL_MASK_FS_K	0x05
#define OHS900_TXLCTL_MASK_LSPD	0x00
#define OHS900_TXLCTL_MASK_FSPD	0x18
#define OHS900_TXLCTL_MASK_FS_POL	0x08
#define OHS900_TXLCTL_MASK_FS_RATE 0x10

#define OHS900_IRQ_ENABLE 0x09
#define OHS900_INTMASK_TRANS_DONE	0x01
#define OHS900_INTMASK_SOFINTR	0x08
#define OHS900_INTMASK_INSRMV	0x04
#define OHS900_INTMASK_RESUME_DET	0x02

#define OHS900_RXCONNSTATEREG 0x0e
#define   OHS900_DISCONNECT_STATE 0x00
#define   OHS900_LS_CONN_STATE 0x01
#define   OHS900_FS_CONN_STATE 0x02

#define OHS900_SLAVE_ADDRESS		0x54

#define OHS900_IRQ_STATUS	0x08	/* write to ack */
#define OHS900_HWREVREG		0xe1	/* read */

#define OHS900_SOFTMRREG		0x0F

#define OHS900_HOSTSLAVECTLREG 			0xe0
#define OHS900_HSCTLREG_HOST_EN_MASK	0x01
#define OHS900_HSCTLREG_RESET_CORE	0x02

#define OHS900_HS_CTL_INIT OHS900_HSCTLREG_HOST_EN_MASK

/* 64-byte FIFO control and status
 */
#define H_MAXPACKET	64	/* bytes in fifos */

#define OHS900_HOST_TXFIFO_DATA	0x30
#define OHS900_TXFIFOCNTMSBREG	0x32
#define OHS900_TXFIFOCNTLSBREG	0x33
#define OHS900_TXFIFOCONTROLREG	0x34
#define OHS900_HOST_RXFIFO_DATA	0x20
#define OHS900_RXFIFOCNTMSBREG	0x22
#define OHS900_RXFIFOCNTLSBREG	0x23
#define OHS900_RXFIFOCONTROLREG	0x24
#define		OHS900_FIFO_FORCE_EMPTY 0x01

#define OHS900_IO_EXTENT 0x100

/*-------------------------------------------------------------------------*/

#define	LOG2_PERIODIC_SIZE	5	/* arbitrary; this matches OHCI */
#define	PERIODIC_SIZE		(1 << LOG2_PERIODIC_SIZE)

struct ohs900 {
	spinlock_t lock;
	void __iomem *addr_reg;

	struct ohs900_platform_data *board;
	struct proc_dir_entry *pde;

	unsigned long stat_insrmv;
	unsigned long stat_wake;
	unsigned long stat_sof;
	unsigned long stat_a;
	unsigned long stat_b;
	unsigned long stat_lost;
	unsigned long stat_overrun;
	unsigned int setup_stage;
	unsigned int setup_len;
	/* sw model */
	struct timer_list timer;
	int debug_timer;
	int fifo_op;

	struct ohs900h_ep *next_periodic;
	struct ohs900h_ep *next_async;

	struct ohs900h_ep *active_a;
	unsigned long jiffies_a;
	struct ohs900h_ep *active_b;
	unsigned long jiffies_b;
	unsigned int irq_stat;
	u32 port1;
	u8 ctrl1, ctrl2, irq_enable;
	u16 frame;

	/* async schedule: control, bulk */
	struct list_head async;

	/* periodic schedule: interrupt, iso */
	u16 load[PERIODIC_SIZE];
	struct ohs900h_ep *periodic[PERIODIC_SIZE];
	unsigned periodic_count;
};

static inline struct ohs900 *hcd_to_ohs900(struct usb_hcd *hcd)
{
	return (struct ohs900 *)(hcd->hcd_priv);
}

static inline struct usb_hcd *ohs900_to_hcd(struct ohs900 *s_ohs900)
{
	return container_of((void *)s_ohs900, struct usb_hcd, hcd_priv);
}

struct ohs900h_ep {
	struct usb_host_endpoint *hep;
	struct usb_device *udev;

	u8 defctrl;
	u8 maxpacket;
	u8 epnum;
	u8 nextpid;

	u16 error_count;
	u16 nak_count;
	u16 length;		/* of current packet */

	/* periodic schedule */
	u16 period;
	u16 branch;
	u16 load;
	struct ohs900h_ep *next;

	/* async schedule */
	struct list_head schedule;
};

/*-------------------------------------------------------------------------*/

/* Register utilities
 * NOTE:  caller must hold ohs900->lock.
 */

static inline u8 ohs900_read(struct ohs900 *s_ohs900, int reg)
{
	u8 temp;

	temp = ((volatile unsigned char *)s_ohs900->addr_reg)[reg];

	return temp;
}

static inline void ohs900_write(struct ohs900 *s_ohs900, int reg, u8 val)
{
	((volatile unsigned char *)s_ohs900->addr_reg)[reg] = val;

}

static inline void
ohs900_write_buf(struct ohs900 *s_ohs900, int addr, const void *buf,
		 size_t count)
{
	void __iomem *addr_reg = s_ohs900->addr_reg;
	const u8 *data;

	if (!count)
		return;

	data = buf;
	do {
		((volatile unsigned char *)addr_reg)[addr] = *data++;
	} while (--count);

}

static inline void
ohs900_read_buf(struct ohs900 *s_ohs900, int addr, void *buf, size_t count,
		unsigned long urb_tot)
{
	void __iomem *addr_reg = s_ohs900->addr_reg;

	u8 *data;

	if (!count)
		return;
	data = buf;

	do {

		*data++ = ((volatile unsigned char *)addr_reg)[addr];
	} while (--count);

}

/*-------------------------------------------------------------------------*/

#ifdef DEBUGING
#define DBG(stuff...)		printk(KERN_DEBUG "ohs900: " stuff)
#else
#define DBG(stuff...)		do{}while(0)
#endif

#ifdef DEVELOP
#define INFO(stuff...) 		printk(KERN_INFO "ohs900: " stuff)
#else
#define INFO(stuff...)		do{}while(0)
#endif

#ifdef VERBOSE
#    define VDBG		DBG
#else
#    define VDBG(stuff...)	do{}while(0)
#endif

#ifdef PACKET_TRACE
#    define PACKET		VDBG
#else
#    define PACKET(stuff...)	do{}while(0)
#endif

#define ERR(stuff...)		printk(KERN_ERR "ohs900: " stuff)
#define WARNING(stuff...)	printk(KERN_WARNING "ohs900: " stuff)
