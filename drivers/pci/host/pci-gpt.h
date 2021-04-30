#ifndef __GPT_PCIE_HOST_H__
#define __GPT_PCIE_HOST_H__

#include <linux/of.h>
#include <linux/list.h>
#include <linux/of_pci.h>
#include <linux/of_gpio.h>

/* regsiter map section */
#define GPT_PCIE_EPRC			0x0000
#define GPT_PCIE_MNG			0x1000
#define GPT_PCIE_RCW			0x2000
#define GPT_PCIE_AXI			0x4000
#define GPT_PCIE_DMA			0x6000
#define GPT_PCIE_EXT			0x8000

/* function register */
#define GPT_PCIE_FUNC_CLASS			(GPT_PCIE_EPRC + 0x008)
#define GPT_PCIE_BAR0				(GPT_PCIE_EPRC + 0x010)
#define GPT_PCIE_BAR1				(GPT_PCIE_EPRC + 0x014)
#define GPT_PCIE_MSI_CTL			(GPT_PCIE_EPRC + 0x090)
#define GPT_PCIE_MSI_LADDR			(GPT_PCIE_EPRC + 0x094)
#define GPT_PCIE_MSI_HADDR			(GPT_PCIE_EPRC + 0x098)
#define GPT_PCIE_MSI_DATA			(GPT_PCIE_EPRC + 0x09C)
#define GPT_PCIE_MSI_MASK			(GPT_PCIE_EPRC + 0x0a0)
#define GPT_PCIE_MSI_PEN			(GPT_PCIE_EPRC + 0x0a4)
#define GPT_PCIE_UNCORR_ERR_OF			(GPT_PCIE_EPRC + 0x104)
#define GPT_PCIE_CORR_ERR_OF			(GPT_PCIE_EPRC + 0x110)
#define GPT_PCIE_SERIAL_NUM0			(GPT_PCIE_EPRC + 0x154)
#define GPT_PCIE_SERIAL_NUM1			(GPT_PCIE_EPRC + 0x158)
/* local manager regsiter */
#define GPT_PCIE_TX_CNT				(GPT_PCIE_MNG + 0x028)
#define GPT_PCIE_TX_DWCNT			(GPT_PCIE_MNG + 0x02c)
#define GPT_PCIE_RX_CNT				(GPT_PCIE_MNG + 0x030)
#define GPT_PCIE_RX_DWCNT			(GPT_PCIE_MNG + 0x034)
#define GPT_PCIE_SRIS_CTL			(GPT_PCIE_MNG + 0x074)
#define GPT_PCIE_DBG_MUX_CTL			(GPT_PCIE_MNG + 0x208)
#define GPT_PCIE_LERR_ST			(GPT_PCIE_MNG + 0x20c)
#define GPT_PCIE_LCRC_CNT			(GPT_PCIE_MNG + 0x214)
#define GPT_PCIE_ECC_CNT			(GPT_PCIE_MNG + 0x218)
#define GPT_PCIE_BUSDEV				(GPT_PCIE_MNG + 0x22c)
#define GPT_PCIE_BAR_CFG0			(GPT_PCIE_MNG + 0x240)
#define GPT_PCIE_BAR_CFG1			(GPT_PCIE_MNG + 0x244)
#define GPT_PCIE_RC_BAR_CFG			(GPT_PCIE_MNG + 0x300)
/* axi register*/
#define GPT_OUT_RE_PCIE_ADDR0(region)		(GPT_PCIE_AXI + (0x20 * region) + 0x00)
#define GPT_OUT_RE_PCIE_ADDR1(region)		(GPT_PCIE_AXI + (0x20 * region) + 0x04)
#define GPT_OUT_RE_DESC_ADDR0(region)		(GPT_PCIE_AXI + (0x20 * region) + 0x08)
#define GPT_OUT_RE_DESC_ADDR1(region)		(GPT_PCIE_AXI + (0x20 * region) + 0x0c)
#define GPT_OUT_RE_DESC_ADDR2(region)		(GPT_PCIE_AXI + (0x20 * region) + 0x10)
#define GPT_OUT_RE_DESC_ADDR3(region)		(GPT_PCIE_AXI + (0x20 * region) + 0x14)
#define GPT_OUT_RE_AXI_ADDR0(region)		(GPT_PCIE_AXI + (0x20 * region) + 0x18)
#define GPT_OUT_RE_AXI_ADDR1(region)		(GPT_PCIE_AXI + (0x20 * region) + 0x1c)
#define GPT_RC_IN_AXI_ADDR0(bar)		(GPT_PCIE_AXI + 0x800 + (0x08 * bar) + 0x0)
#define GPT_RC_IN_AXI_ADDR1(bar)		(GPT_PCIE_AXI + 0x800 + (0x08 * bar) + 0x4)
#define GPT_EP_IN_AXI_ADDR0(bar)		(GPT_PCIE_AXI + 0x840 + (0x08 * bar) + 0x0)
#define GPT_EP_IN_AXI_ADDR1(bar)		(GPT_PCIE_AXI + 0x840 + (0x08 * bar) + 0x4)
/* dma register */
#define GPT_PDMA_CTL(ch)			(GPT_PCIE_DMA + 0x14 * (ch) + 0x00)
#define GPT_PDMA_ADDR0(ch)			(GPT_PCIE_DMA + 0x14 * (ch) + 0x04)
#define GPT_PDMA_ADDR1(ch)			(GPT_PCIE_DMA + 0x14 * (ch) + 0x08)
#define GPT_PDMA_ATTR0(ch)			(GPT_PCIE_DMA + 0x14 * (ch) + 0x0c)
#define GPT_PDMA_ATTR1(ch)			(GPT_PCIE_DMA + 0x14 * (ch) + 0x10)
#define GPT_PDMA_IRQST				(GPT_PCIE_DMA + 0xa0)
#define GPT_PDMA_IRQEN				(GPT_PCIE_DMA + 0xa4)
#define GPT_PDMA_IRQDIS				(GPT_PCIE_DMA + 0xa8)
#define GPT_PDMA_IN_UNC_ECC			(GPT_PCIE_DMA + 0xac)
#define GPT_PDMA_IN_C_ECC			(GPT_PCIE_DMA + 0xb0)
#define GPT_PDMA_OUT_UNC_ECC			(GPT_PCIE_DMA + 0xb4)
#define GPT_PDMA_OUT_C_ECC			(GPT_PCIE_DMA + 0xb8)
#define GPT_PDMA_VERID				(GPT_PCIE_DMA + 0xf8)
#define GPT_PDMA_CONFIG				(GPT_PCIE_DMA + 0xfc)
/* pcie extern register */
#define GPT_PCIE_CFG				(GPT_PCIE_EXT + 0x000)
#define GPT_PCIE_CTRL				(GPT_PCIE_EXT + 0x004)
#define GPT_PCIE_LPM_CTRL			(GPT_PCIE_EXT + 0x008)
#define GPT_PCIE_IRQ_STA			(GPT_PCIE_EXT + 0x00c)
#define GPT_PCIE_SDBND_STA0			(GPT_PCIE_EXT + 0x010)
#define GPT_PCIE_SDBND_STA1			(GPT_PCIE_EXT + 0x014)
#define GPT_PCIE_MSI_MSK_STA			(GPT_PCIE_EXT + 0x018)
#define GPT_PCIE_MSI_PEND_STA			(GPT_PCIE_EXT + 0x01c)
#define GPT_PCIE_STA				(GPT_PCIE_EXT + 0x020)
#define GPT_PCIE_IRQ_MASK			(GPT_PCIE_EXT + 0x024)
#define GPT_PCIE_RST_CTL			(GPT_PCIE_EXT + 0x028)
#define GPT_PCIE_DBG_CTL			(GPT_PCIE_EXT + 0x050)
#define GPT_PCIE_DBG_BUF			(GPT_PCIE_EXT + 0x054)
#define GPT_PCIE_DBG_BUFADDR			(GPT_PCIE_EXT + 0x058)
#define GPT_PCIE_LTSSM_ST_CTL			(GPT_PCIE_EXT + 0x080)
#define GPT_PCIE_LTSSM_ST_BUF			(GPT_PCIE_EXT + 0x084)
#define GPT_PCIE_MSI_EHADDR			(GPT_PCIE_EXT + 0x090)
#define GPT_PCIE_MSI_ELADDR			(GPT_PCIE_EXT + 0x094)
#define GPT_PCIE_MSI_EDATA			(GPT_PCIE_EXT + 0x098)
#define GPT_PCIE_MSI_IRQ_EN			(GPT_PCIE_EXT + 0x200)
#define GPT_PCIE_MSI_STA			(GPT_PCIE_EXT + 0x204)
#define GPT_PCIE_MSIX_EHADDR(ch)		(GPT_PCIE_EXT + 0xc *(ch) + 0x9c)
#define GPT_PCIE_MSIX_ELADDR(ch)		(GPT_PCIE_EXT + 0xc *(ch) + 0xa0)
#define GPT_PCIE_MSIX_EDATA(ch)			(GPT_PCIE_EXT + 0xc *(ch) + 0xa4)
#define GPT_PCIE_MSIX_IRQ_EN			(GPT_PCIE_EXT + 0x208)
#define GPT_PCIE_MSIX_STA			(GPT_PCIE_EXT + 0x20c)

/*configuration register*/
#define GPT_PCIE_MSI_CTRL			0x90
#define GPT_PCIE_MSIX_CTRL			0xb0
#define GPT_PCIE_MSIX_TABLE			0xb4

/* type defination  of debug */
#define GPT_DEBUG_PHY_LTSSM0			0x00
#define GPT_DEBUG_PHY_LTSSM1			0x01
#define GPT_DEBUG_PHY_LTSSM2			0x04
#define GPT_DEBUG_PHY_RX			0x05
#define GPT_DEBUG_PHY_TX			0x06
#define GPT_DEBUG_DATALINK_TX			0x08
#define GPT_DEBUG_DATALINK_RX			0x09
#define GPT_DEBUG_DATALINK_RTX			0x0a
#define GPT_DEBUG_TRANSACT_TX0			0x10
#define GPT_DEBUG_TRANSACT_RX0			0x11
#define GPT_DEBUG_TRANSACT_TX1			0x13
#define GPT_DEBUG_TRANSACT_RX1			0x12
#define GPT_DEBUG_MODE(x,d)			(((x) & ~(0x3f)) | (d))
/* end point offset */
#define GPT_CTL_SERR_EN				(0x1 << 8)
#define GPT_CTL_INTX_DIS			(0x1 << 10)
#define GPT_CTL_MSI_EN				(0x1 << 16)
#define GPT_CTL_MM_EN(x, y)			(((x) & (~(7 << 20))) | y)
#define GPT_MSI_MASK				(0x1 << 0)
#define GPT_MSI_PEND				(0x1 << 0)

/* local manage offset */
#define GPT_RC_BAR_4B				(0x00)
#define GPT_RC_BAR_8B				(0x01)
#define GPT_RC_BAR_16B				(0x02)
#define GPT_RC_BAR_32B				(0x03)
#define GPT_RC_BAR_64B				(0x04)
#define GPT_RC_BAR_128B				(0x05)
#define GPT_RC_BAR_256B				(0x06)
#define GPT_RC_BAR_512B				(0x07)
#define GPT_RC_BAR_1KB				(0x08)
#define GPT_RC_BAR_2KB				(0x09)
#define GPT_RC_BAR_4KB				(0x0a)
#define GPT_RC_BAR_8KB				(0x0b)
#define GPT_RC_BAR_16KB				(0x0c)
#define GPT_RC_BAR_32KB				(0x0d)
#define GPT_RC_BAR_64KB				(0x0e)
#define GPT_RC_BAR_128K				(0x0f)
#define GPT_RC_BAR_256K				(0x10)
#define GPT_RC_BAR_512K				(0x11)
#define GPT_RC_BAR_1M				(0x12)
#define GPT_RC_BAR_2M				(0x13)
#define GPT_RC_BAR_4M				(0x14)
#define GPT_RC_BAR_8M				(0x15)
#define GPT_RC_BAR_16M				(0x16)
#define GPT_RC_BAR_32M				(0x17)
#define GPT_RC_BAR_64M				(0x18)
#define GPT_RC_BAR_128M				(0x19)
#define GPT_RC_BAR_256M				(0x1a)
#define GPT_RC_BAR_512M				(0x1b)
#define GPT_RC_BAR_1G				(0x1c)
#define GPT_RC_BAR_2G				(0x1d)
#define GPT_RC_BAR_4G				(0x1e)
#define GPT_RC_BAR_8G				(0x1f)
#define GPT_RC_BAR_16G				(0x20)
#define GPT_RC_BAR_32G				(0x21)
#define GPT_RC_BAR_64G				(0x22)
#define GPT_RC_BAR_128G				(0x23)
#define GPT_RC_BAR_256G				(0x24)
#define GPT_EP_BAR_128B				(0x00)
#define GPT_EP_BAR_256B				(0x00)
#define GPT_EP_BAR_512B				(0x02)
#define GPT_EP_BAR_1KB				(0x03)
#define GPT_EP_BAR_2KB				(0x04)
#define GPT_EP_BAR_4KB				(0x05)
#define GPT_EP_BAR_8KB				(0x06)
#define GPT_EP_BAR_16KB				(0x07)
#define GPT_EP_BAR_32KB				(0x08)
#define GPT_EP_BAR_64KB				(0x09)
#define GPT_EP_BAR_128K				(0x0a)
#define GPT_EP_BAR_256K				(0x0b)
#define GPT_EP_BAR_512K				(0x0c)
#define GPT_EP_BAR_1M				(0x0d)
#define GPT_EP_BAR_2M				(0x0e)
#define GPT_EP_BAR_4M				(0x0f)
#define GPT_EP_BAR_8M				(0x10)
#define GPT_EP_BAR_16M				(0x11)
#define GPT_EP_BAR_32M				(0x12)
#define GPT_EP_BAR_64M				(0x13)
#define GPT_EP_BAR_128M				(0x14)
#define GPT_EP_BAR_256M				(0x15)
#define GPT_EP_BAR_512M				(0x16)
#define GPT_EP_BAR_1G				(0x17)
#define GPT_EP_BAR_2G				(0x18)
#define GPT_EP_BAR_4G				(0x19)
#define GPT_EP_BAR_8G				(0x1a)
#define GPT_EP_BAR_16G				(0x1b)
#define GPT_EP_BAR_32G				(0x1c)
#define GPT_EP_BAR_64G				(0x1d)
#define GPT_EP_BAR_128G				(0x1e)
#define GPT_EP_BAR_256G				(0x1f)
#define GPT_EP_BAR_DISABLE			(0x0)
#define GPT_EP_BAR_IO32				(0x1)
#define GPT_EP_BAR_MEM32			(0x4)
#define GPT_EP_BAR_PREF_MEM32			(0x5)
#define GPT_EP_BAR_MEM64			(0x6)
#define GPT_EP_BAR_PREF_MEM64			(0x7)

#define GPT_EP_BAR0_ENABLE(type, size)		((((size) & 0x1f) | ((type & 0x7) << 5)) << 0)
#define GPT_EP_BAR1_ENABLE(type, size)		((((size) & 0x1f) | ((type & 0x7) << 5)) << 8)
#define GPT_EP_BAR2_ENABLE(type, size)		((((size) & 0x1f) | ((type & 0x7) << 5)) << 16)
#define GPT_EP_BAR3_ENABLE(type, size)		((((size) & 0x1f) | ((type & 0x7) << 5)) << 24)
#define GPT_EP_BAR4_ENABLE(type, size)		((((size) & 0x1f) | ((type & 0x7) << 5)) << 0)
#define GPT_EP_BAR5_ENABLE(type, size)		((((size) & 0x1f) | ((type & 0x7) << 5)) << 8)

#define GPT_RC_BAR_DISABLE			(0x0)
#define GPT_RC_BAR_IO32				(0x1)
#define GPT_RC_BAR_MEM32			(0x4)
#define GPT_RC_BAR_PREF_MEM32			(0x5)
#define GPT_RC_BAR_MEM64			(0x6)
#define GPT_RC_BAR_PREF_MEM64			(0x7)

#define GPT_RC_BAR0_ENABLE(type, size)		((((size) & 0x3f) | ((type & 0x7) << 6)) << 0)
#define GPT_RC_BAR1_ENABLE(type, size)		((((size) & 0x1f) | ((type & 0x7) << 5)) << 9)
#define GPT_RC_CHECK_EN				(0x1 << 31)
/* axi offset */
#define GPT_OUT_DESC_TP_MEM			(0x02)
#define GPT_OUT_DESC_TP_IO			(0x06)
#define GPT_OUT_DESC_TP_CFG0			(0x0a)
#define GPT_OUT_DESC_TP_CFG1			(0x0b)
#define GPT_OUT_DESC_TP_MSG			(0x0c)
#define GPT_OUT_DESC_TP_MSGV			(0x0d)
#define GPT_OUT_CAPT_WITH_RID			(0x1 << 23)
#define GPT_OUT_DEV_NUM(n)			(((n) & 0x1f) << 27)
#define GPT_OUT_BUS_NUM(n)			(((n) & 0xff) << 0)
/* dma offset */
#define GPT_PDMA_ENABLE				(0x1 << 0)
#define GPT_PDMA_OTBOUND			(0x1 << 1)
#define GPT_PDMA_INBOUND			(0x0 << 1)
#define GPT_PDMA_BULK				(0x0 << 25)
#define GPT_PDMA_SG				(0x1 << 25)
#define GPT_PDMA_PRE				(0x2 << 25)
#define GPT_PDMA_LEN(len)			((len) & 0xffffff)
#define GPT_PDMA_TYPE(data, type)		(((data) & (~(0x3 << 25))) | (type))
#define GPT_PDMA_CONTINUE			(0x1 << 29)
#define GPT_PDMA_INT				(0x1 << 24)
#define GPT_PDMA_ARBAR(n)			((0x2 | ((n) & 0x3)) << 22)
#define GPT_PDMA_ARDOMAIN(n)			(((n) & 0x3) << 20)
#define GPT_PDMA_ARSNOOP(n)			(((n) & 0x7) << 16)
#define GPT_PDMA_ARREGION(n)			(((n) & 0xf) << 12)
#define GPT_PDMA_ARQOS(n)			(((n) & 0xf) << 8)
#define GPT_PDMA_ARLOCK(n)			(((n) & 0x1) << 7)
#define GPT_PDMA_ARCACHE(n)			(((n) & 0xf) << 3)
#define GPT_PDMA_ARPROT(n)			(((n) & 0x7) << 0)
#define GPT_PDMA_VERMAJOR(id)			((id >> 8) & 0xf)
#define GPT_PDMA_VERMINOR(id)			((id) & 0xf)
#define GPT_PDMA_CHNUM(state)			((state) & 0xf)
#define GPT_PDMA_PARTNUM(state)			(((state) >> 0x4) & 0xf)
#define GPT_PDMA_PARTSIZE(state)		(((state) >> 0x8) & 0xf)
/* extern offset */
/* ctl offset*/
#define GPT_PCIE_CTL_LINK_EN			(0x1 << 0)
#define GPT_PCIE_CTL_LTSSM_EN			(0x1 << 1)
#define GPT_PCIE_LINK				(0x3 << 7)
#define GPT_PCIE_LTSSM_ST_CLR			(0x1 << 30)
#define GPT_PCIE_LTSSM_ST_FULL			(0x1 << 31)
/* config offset */
#define GPT_PCIE_RCMD				(0x1 << 0)
#define GPT_PCIE_GEN2				(0x1 << 1)
#define GPT_PCIE_LKX1(num)			(((num) & (~(0x3 << 3))) | (0x00 << 3))
#define GPT_PCIE_LKX2(num)			(((num) & (~(0x3 << 3))) | (0x01 << 3))
#define GPT_PCIE_LKX4(num)			(((num) & (~(0x3 << 3))) | (0x02 << 3))
/* reset offset */
#define GPT_PCIE_RST_PHY_OF			(0x1 << 0)
#define GPT_PCIE_RST_CTL_OF			(0x1 << 1)
#define GPT_PCIE_RST_RESET_OF			(0x1 << 2)
#define GPT_PCIE_RST_MGMT_OF			(0x1 << 3)
#define GPT_PCIE_RST_MGMTST_OF			(0x1 << 4)
#define GPT_PCIE_RST_AXI_OF			(0x1 << 5)
#define GPT_PCIE_RST_PM_OF			(0x1 << 6)
#define GPT_PCIE_RST_CLK_STAB_OF		(0x1 << 31)
/* dbg ctrl offset */
#define GPT_PCIE_DBG_CAP_VEC(x, d)		(((x) & (~(0xffff << 0))) | (d))
#define GPT_PCIE_DBG_CAP_INTV(x, d)		(((x) & (~(0xfff << 16))) | (d))
#define GPT_PCIE_DBG_EN				(0x1 << 28)
#define GPT_PCIE_DBG_OVER			(0x1 << 29)
#define GPT_PCIE_PFM_EN				(0x1 << 30)
/* pcie state offset*/
#define GPT_PCIE_ST_SPEED5G			(0x1 << 21)
/* pcie irq offset */
#define GPT_PCIE_IRQ_WAKE			(0x1 << 12)
#define GPT_PCIE_IRQ_PFM			(0x1 << 11)
#define GPT_PCIE_IRQ_MSG			(0x1 << 10)
#define GPT_PCIE_IRQ_PHY			(0x1 << 9)
#define GPT_PCIE_IRQ_ACK			(0x1 << 8)
#define GPT_PCIE_IRQ_INTD			(0x1 << 7)
#define GPT_PCIE_IRQ_INTC			(0x1 << 6)
#define GPT_PCIE_IRQ_INTB			(0x1 << 5)
#define GPT_PCIE_IRQ_INTA			(0x1 << 4)
#define GPT_PCIE_IRQ_LOCL			(0x1 << 3)
#define GPT_PCIE_IRQ_DMA			(0x1 << 2)
#define GPT_PCIE_IRQ_DPA			(0x1 << 1)
#define GPT_PCIE_IRQ_PWR			(0x1 << 0)

#define GPT_PCIE_MASK_DEFAULT			GPT_PCIE_IRQ_MSG
#define GPT_PCIE_INT_ALL_MASK			0xFFFFFFFF
#define GPT_PCIE_INT_BITS			0xFFFFF000
#define GPT_PCIE_IRQ_INTx_MASK			(GPT_PCIE_IRQ_INTA | \
						GPT_PCIE_IRQ_INTB | \
						GPT_PCIE_IRQ_INTC | \
						GPT_PCIE_IRQ_INTD)
#define GPT_PCIE_INT_MASK			0xFFFFFF0F
#define GPT_PCI_INT_NR				(12)
#define GPT_NUM_MSI_IRQS			(32)

#pragma pack(push, 1)
struct gpt_pdma_desc {
	uint64_t bus_addr;
	uint32_t axi_ctrl;
	uint64_t pci_addr;
	uint32_t tlp_header;
	uint32_t len_ctl;
	uint8_t  axibus_st;
	uint8_t  pcibus_st;
	uint8_t  ch_stat;
	uint8_t  reserved;
	struct gpt_pdma_desc *next;
};
#pragma pack(pop)

struct gpt_pciec_res {
	struct resource res;
};

#define INT_PCI_MSI_NR 32
#define INT_PCI_MSIX_NR 8

struct gpt_msi {
        struct msi_chip chip;
        DECLARE_BITMAP(used, INT_PCI_MSI_NR);
        struct irq_domain *domain;
        unsigned long pages;
        struct mutex lock; 
        int irq;
};

struct gpt_pciec {
	struct device *dev;
	struct pci_bus *root_bus;
	u8 root_busno;
	void __iomem *regs;
	void __iomem *phy;
	void __iomem *cfg;
	struct device_node *node;
	struct resource *cfg_res;
	struct resource bus_res;
	struct gpt_pciec_res mem;
	struct gpt_pciec_res io;
	struct list_head list;
	uint32_t irq;
	uint64_t msi_pages;
	struct irq_chip *int_chip;
	struct irq_chip *msi_chip;
	struct gpt_msi msi;
	struct irq_domain *int_domain;
//}__attribute__((packed));
};

#define GPT_PCIE_INT_BIT 		0x01

#endif
