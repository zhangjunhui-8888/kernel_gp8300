/*
 * pcie-host.c: a pcie host controller driver
 *
 * Copyright (C) 2018
 *
 * This file is subject to the terms and conditions of the GUN General Public
 * License. See the file COPYING in the main directory of this archive
 * for more details.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/stat.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>

#include "pci-gpt.h"

spinlock_t pcie_lock;
static int debug_level = 0;
module_param(debug_level, uint, 0644);
MODULE_PARM_DESC(debug_level, "debug ctl 1:en, 0:disen, default disable");

void gpt_vdo_dumpdata(void *addr, int length);

static inline struct gpt_pciec *sys_to_pcie(struct pci_sys_data *sys)
{
        return sys->private_data;
}

static inline void pciec_reg_write(struct gpt_pciec *pcie, u32 addr, u32 data)
{
	
	writel(data, pcie->regs + addr);
}

static inline unsigned int pciec_rcw_read(struct gpt_pciec *pcie, u32 addr)
{
	return readl(pcie->regs + GPT_PCIE_RCW + addr);
}

static inline void pciec_rcw_write(struct gpt_pciec *pcie, u32 addr, u32 data)
{
	writel(data, pcie->regs + GPT_PCIE_RCW + addr);
}

static inline unsigned int pciec_reg_read(struct gpt_pciec *pcie, u32 addr)
{
	return readl(pcie->regs + addr);
}

static inline void pciec_phy_write(struct gpt_pciec *pcie, u32 addr, u32 data)
{
	writel(data, pcie->phy + addr);
}

static inline unsigned int pciec_phy_read(struct gpt_pciec *pcie, u32 addr)
{
	return readl(pcie->phy + addr);
}

static int gpt_pciec_outbd_region(struct gpt_pciec *pcie, int func,
				uint32_t region, int mode, uint64_t cpu_addr,
				uint64_t pci_addr, size_t len)
{
	uint32_t addr0, addr1, desc0, desc1;
	int passbts = ilog2(1ULL << fls64(len - 1));

	dev_dbg(pcie->dev, "ob region:%d, cpuaddr:%llx, pciaddr:%llx, pass:%d\n",
			region, cpu_addr, pci_addr, passbts);

	if (passbts > 64 || passbts < 8) {
		dev_err(pcie->dev, "outbound region 8 < pass  < 64\n");
		passbts = 8;
	}

	if (region > 31) {
		dev_err(pcie->dev, "region must less 32\n");
		return -1;
	}

	addr0 = (lower_32_bits(pci_addr) & GENMASK(31, 8)) | ((passbts - 1) & GENMASK(5,0));
	addr1 = upper_32_bits(pci_addr);
	pciec_reg_write(pcie, GPT_OUT_RE_PCIE_ADDR0(region), addr0);
	pciec_reg_write(pcie, GPT_OUT_RE_PCIE_ADDR1(region), addr1);

	switch (mode) {
	case 1:
		desc0 = GPT_OUT_DESC_TP_MEM;
		break;

	case 2:
		desc0 = GPT_OUT_DESC_TP_IO;
		break;
	default:
		desc0 = GPT_OUT_DESC_TP_MEM;
	}

	desc1 = 0x0;
	pciec_reg_write(pcie, GPT_OUT_RE_DESC_ADDR0(region), desc0);
	pciec_reg_write(pcie, GPT_OUT_RE_DESC_ADDR1(region), desc1);

	addr0 = (lower_32_bits(cpu_addr) & GENMASK(31, 8)) | (passbts - 1);
	addr1 = 0; /* axi only pass 32bit to pcie! */
	pciec_reg_write(pcie, GPT_OUT_RE_AXI_ADDR0(region), addr0);
	pciec_reg_write(pcie, GPT_OUT_RE_AXI_ADDR1(region), addr1);

	return 0;
}

static int gpt_pciec_inbd_region(struct gpt_pciec *pcie,
			int bars, uint64_t bus_addr, size_t len)
{
	int pass = ilog2(1ULL << fls64(len - 1));
	u32 regs = (bus_addr & (~0x3f)) | (pass - 1);

	dev_dbg(pcie->dev, "inbound bar:%d, addr:%llx, pass:%d\n", bars, bus_addr, pass);

	if (bars > 2)
		return -1;

	pciec_reg_write(pcie, GPT_RC_IN_AXI_ADDR0(bars), regs);
	pciec_reg_write(pcie, GPT_RC_IN_AXI_ADDR1(bars), bus_addr >> 32);

	return 0;
}


static int gpt_pciec_hwinit(struct gpt_pciec *pcie)
{
	uint32_t reg;

	/*check pcie link status*/
	reg = pciec_reg_read(pcie, GPT_PCIE_STA);
	if ((reg & GPT_PCIE_LINK) != GPT_PCIE_LINK) {
		dev_err(pcie->dev, "PCIE link error, state=0x%x\n", reg);
		return -1;
	}

	/*set rc mode */
	reg = pciec_reg_read(pcie, GPT_PCIE_CFG);
	pciec_reg_write(pcie, GPT_PCIE_CFG, reg | GPT_PCIE_RCMD);

	reg = pciec_rcw_read(pcie, PCI_CLASS_DEVICE & (~0x3)) | (PCI_CLASS_BRIDGE_PCI << 16);
	pciec_rcw_write(pcie, PCI_CLASS_DEVICE & (~0x3), reg);

	reg = GPT_RC_BAR0_ENABLE(GPT_RC_BAR_MEM64, GPT_RC_BAR_4G)
		| GPT_RC_BAR1_ENABLE(GPT_RC_BAR_DISABLE, GPT_RC_BAR_4G)
		| GPT_RC_CHECK_EN
		| (0x1 <<17) | (0x1 << 18) | (0x1 << 19);
	pciec_reg_write(pcie, GPT_PCIE_RC_BAR_CFG, reg);

	pciec_rcw_write(pcie, GPT_PCIE_BAR0, 0x00000000);
	pciec_rcw_write(pcie, GPT_PCIE_BAR1, 0x00000002);

	/* intr mask */
	pciec_reg_write(pcie, GPT_PCIE_IRQ_MASK, GPT_PCIE_MASK_DEFAULT);

	return 0;
}

static void *gpt_pcie_map_cfg(struct pci_bus *bus,
			unsigned int devfn, int offset)
{
	struct gpt_pciec *pcie = sys_to_pcie(bus->sysdata);
	unsigned int busn = bus->number;
	u32 addr0, desc0;


	dev_dbg(pcie->dev, "bus:%d, devfn:%x, offset:%x\n", bus->number, devfn, offset);
	if (bus->number == pcie->bus_res.start) {
		if (devfn) {
			return NULL;
		}

		return pcie->regs + (offset & 0xfff);
	}
	/*check link is up*/
       /* Update Output registers for AXI region 0. */
        addr0 = ((12 -1) & GENMASK(5, 0)) | ((devfn << 12) & GENMASK(19, 12)) |
		((busn << 20) & GENMASK(27, 20));
	pciec_reg_write(pcie, GPT_OUT_RE_PCIE_ADDR0(0), addr0);

        /* Configuration Type 0 or Type 1 access. */
        desc0 = (0x1 << 23) | ((0 << 24) & GENMASK(31, 24));

        if (busn == pcie->bus_res.start + 1)
                desc0 |= GPT_OUT_DESC_TP_CFG0;
        else
                desc0 |= GPT_OUT_DESC_TP_CFG1;

	pciec_reg_write(pcie, GPT_OUT_RE_DESC_ADDR0(0), desc0);

	return pcie->cfg + (offset & 0xfff);
}

static int gpt_pciec_read_config(struct pci_bus *bus,
			unsigned int devfn, int offset, int len, u32 *val)
{
	void __iomem *addr = gpt_pcie_map_cfg(bus, devfn, offset);
	if (!addr) {
		*val = ~0x0;
		dev_dbg(&bus->dev, "bus:%d, devfn:0x%x, offset:0x%x, len:0x%x, val:0x%x\n",
			bus->number, devfn, offset, len, *val);
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	switch (len) {
	case 1:
		*val = readb(addr);
		break;
	case 2:
		*val = readw(addr);
		break;
	default:
		*val = readl(addr);
	}

	dev_dbg(&bus->dev, "read: bus:%d, devfn:0x%x, offset:0x%x, len:0x%x, val:0x%x\n",
			bus->number, devfn, offset,  len, *val);
	return PCIBIOS_SUCCESSFUL;
}

static int gpt_pciec_write_config(struct pci_bus *bus,
			unsigned int devfn, int offset, int len, u32 val)
{
	void __iomem *addr = gpt_pcie_map_cfg(bus, devfn, offset);
	if (!addr) {
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	dev_dbg(&bus->dev, "write: bus:%d, devfn:%x, offset:%x, len:%x, val:0x%x\n",
			bus->number, devfn, offset,  len, val);
	switch (len) {
	case 1:
		writeb(val, addr);
		break;
	case 2:
		writew(val, addr);
		break;
	default:
		writel(val, addr);
	}

	return PCIBIOS_SUCCESSFUL;
}

static struct pci_ops gpt_pciec_ops = {
	.read = gpt_pciec_read_config,
	.write = gpt_pciec_write_config
};

static int pci_dma_range_parser_init(struct of_pci_range_parser *parser,
                                     struct device_node *node)
{
	const int na = 3, ns = 2;
	int rlen;

	parser->node = node;
	parser->pna = of_n_addr_cells(node);
	parser->np = parser->pna + na + ns;

	parser->range = of_get_property(node, "dma-ranges", &rlen);
	if (!parser->range)
		return -ENOENT;

	parser->end = parser->range + rlen / sizeof(__be32);

	return 0;
}

static int gpt_pcie_parse_dma_ranges(struct gpt_pciec *pcie)
{
	struct device_node *np = pcie->node;
	struct of_pci_range range;
	struct of_pci_range_parser parser;
	int cnt = 0; /*bar0 inbound*/

	if (pci_dma_range_parser_init(&parser, np)) {
		dev_err(pcie->dev, "missing dma-ranges property\n");
		return -EINVAL;
	}

	/* Get the dma-ranges from DT */
	for_each_of_pci_range(&parser, &range) {
		u64 end = range.cpu_addr + range.size - 1;

		dev_dbg(pcie->dev, "dma-ranges flags:0x%08x, cpuaddr:0x%016llx--0x%016llx, pciaddr:0x%016llx\n",
					range.flags, range.cpu_addr, end, range.pci_addr);
		gpt_pciec_inbd_region(pcie, cnt, range.cpu_addr, range.size);
	}

	return 0;
}

static int gpt_pcie_parse_ranges(struct gpt_pciec *pcie)
{
	int mem = 0;
	int region = 1;

	struct of_pci_range_parser parser;
	struct of_pci_range range;
	uint32_t addr0, addr1, desc1;
	uint64_t cpu_addr;


	/*
 	* Reserve region 0 for PCI configure space accesses:
        * OB_REGION_PCI_ADDR0 and OB_REGION_DESC0 are updated dynamically by
        * gpt_pcie_map_cfg(), other region registers are set here once for all.
        */

        addr1 = 0; /* Should be programmed to zero. */
        desc1 = (pcie->bus_res.start) & GENMASK(7, 0);
	pciec_reg_write(pcie, GPT_OUT_RE_PCIE_ADDR1(0), addr1);
	pciec_reg_write(pcie, GPT_OUT_RE_DESC_ADDR1(0), desc1);

        cpu_addr = pcie->cfg_res->start;
        addr0 = ((12 -1) & GENMASK(5,0)) | (lower_32_bits(cpu_addr) & GENMASK(31, 8));
        addr1 = 0;
	pciec_reg_write(pcie, GPT_OUT_RE_AXI_ADDR0(0), addr0);
	pciec_reg_write(pcie, GPT_OUT_RE_AXI_ADDR1(0), addr1);

	if (of_pci_range_parser_init(&parser, pcie->node)) {
		dev_err(pcie->dev, "missing \"ranges\" property\n");
		return -EINVAL;
	}

	for_each_of_pci_range(&parser, &range) {

		switch (range.flags & IORESOURCE_TYPE_BITS) {
		case IORESOURCE_MEM:
			mem = 0x1;
			dev_dbg(pcie->dev, "MEM: ");
			break;

		case IORESOURCE_IO:
			mem = 0x2;
			dev_dbg(pcie->dev, "IO: ");
			break;
		default:
			dev_err(pcie->dev, "pciec ignore unsupported range\n");
			continue;
		}

		dev_dbg(pcie->dev, "pci(0x%llx)-->cpu(0x%llx), len(0x%llx)\n",
				range.pci_addr, range.cpu_addr, range.size);

		gpt_pciec_outbd_region(pcie, 0,
				region++, mem, range.cpu_addr,
				range.pci_addr, range.size);
	}

	return 0;
}

static int gpt_pciec_map_ranges(struct gpt_pciec *pcie,
				 struct list_head *res,
				 resource_size_t io_base)
{
	int err;
	struct pci_host_bridge_window *window;

	list_for_each_entry(window, res, list) {
		struct resource *res = window->res;
		u64 restype = resource_type(res);

		switch (restype) {
		case IORESOURCE_IO:
			err = devm_request_resource(pcie->dev,
					&ioport_resource, res);
			break;

		case IORESOURCE_MEM:
			err = devm_request_resource(pcie->dev,
					&iomem_resource, res);
			break;

		case IORESOURCE_BUS:
			break;

		default:
			dev_err(pcie->dev, "invalid resource %pR\n", res);
			return -EINVAL;
		}

		if (err < 0) {
			dev_err(pcie->dev, "request resource failed\n");
			return err;
		}
	}

	return 0;
}

static irqreturn_t gpt_pciec_isr(int irq, void *data)
{
	struct gpt_pciec *pcie = (struct gpt_pciec*)data;
	uint32_t irqno;
	unsigned int offset;
	unsigned long flags, state;	

	local_irq_save(flags);

	state = pciec_reg_read(pcie, GPT_PCIE_IRQ_STA);
	pciec_reg_write(pcie, GPT_PCIE_IRQ_MASK, GPT_PCIE_INT_ALL_MASK);
	dev_dbg(pcie->dev, "extirq:%lx \n", state);

	if ((state = pciec_reg_read(pcie, GPT_PCIE_IRQ_STA) &  GPT_PCIE_IRQ_WAKE)) {
		pciec_reg_write(pcie, GPT_PCIE_IRQ_STA, GPT_PCIE_IRQ_WAKE);
	}

	/*handle INTx interrupt*/
	while ((state = pciec_reg_read(pcie, GPT_PCIE_IRQ_STA) & GPT_PCIE_IRQ_INTx_MASK)) {
		for_each_set_bit(offset, &state, 8) {
			pciec_reg_write(pcie, GPT_PCIE_IRQ_STA, BIT(offset));
			irqno = irq_find_mapping(pcie->int_domain, offset);
			if (irqno) {
				generic_handle_irq(irqno);
			} else {
				dev_err(pcie->dev, "unexpected state 0x%lx, offset:%d\n", state, offset);
			}
		}
	}

	pciec_reg_write(pcie, GPT_PCIE_IRQ_MASK, GPT_PCIE_INT_MASK);

	local_irq_restore(flags);

	return IRQ_HANDLED;
}

void gpt_pciec_irq_mask(struct irq_data *data)
{
	struct gpt_pciec *pcie = irq_data_get_irq_chip_data(data);
	irq_hw_number_t hwirq = irqd_to_hwirq(data);
	uint32_t val;

	val = pciec_reg_read(pcie, GPT_PCIE_IRQ_MASK) | (1UL << hwirq);
	pciec_reg_write(pcie, GPT_PCIE_IRQ_MASK, val);
}

void gpt_pciec_irq_unmask(struct irq_data *data)
{
	struct gpt_pciec *pcie = irq_data_get_irq_chip_data(data);
	irq_hw_number_t hwirq = irqd_to_hwirq(data);
	uint32_t val;

	val = pciec_reg_read(pcie, GPT_PCIE_IRQ_MASK) & ~(1UL << hwirq);
	pciec_reg_write(pcie, GPT_PCIE_IRQ_MASK, val);
}

static struct irq_chip gpt_pciec_int_chip = {
	.name = "pciec-intr",
	.irq_mask = gpt_pciec_irq_mask,
	.irq_unmask = gpt_pciec_irq_unmask,
};

static int gpt_pciec_int_map(struct irq_domain *domain, unsigned int irq,
			 irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &gpt_pciec_int_chip, handle_simple_irq);
	irq_set_chip_data(irq, domain->host_data);
	set_irq_flags(irq, IRQF_VALID);

	return 0;
}
static const struct irq_domain_ops gpt_pciec_domain = {
	.map = gpt_pciec_int_map,
};

int gpt_pciec_irq_map(struct pci_dev *pdev)
{
	int irq;
	struct pci_bus *bus = pdev->bus;
	struct gpt_pciec *pcie = (struct gpt_pciec*)pdev->sysdata;
	if (bus == NULL || pcie == NULL) {
		dev_err(pcie->dev, "no pcie host found\n");
		return 0;
	}

	irq = irq_create_mapping(pcie->int_domain, pdev->pin + 3);

	dev_dbg(pcie->dev, "bus:%d- slot:%d, irq:%d\n",
		pdev->bus->number, pdev->pin, irq);

	return irq;
}

static inline struct gpt_msi *to_gpt_msi(struct msi_chip *chip)
{
        return container_of(chip, struct gpt_msi, chip);
}

static int gpt_msi_alloc(struct gpt_msi *chip)
{
        int msi;

        mutex_lock(&chip->lock);

        msi = find_first_zero_bit(chip->used, INT_PCI_MSI_NR);
        if (msi < INT_PCI_MSI_NR)
                set_bit(msi, chip->used);
        else
                msi = -ENOSPC;

        mutex_unlock(&chip->lock);

        return msi;
}

static void gpt_msi_free(struct gpt_msi *chip, unsigned long irq)
{
        struct device *dev = chip->chip.dev;

        mutex_lock(&chip->lock);

        if (!test_bit(irq, chip->used))
                dev_err(dev, "trying to free unused MSI#%lu\n", irq);
        else
                clear_bit(irq, chip->used);

        mutex_unlock(&chip->lock);
}

static irqreturn_t gpt_pcie_msi_irq(int irq, void *data)
{
	unsigned long reg;
	unsigned long flags;	
	unsigned int offset;
        unsigned int irqno;
        struct gpt_pciec *pcie = data;
        struct gpt_msi *msi = &pcie->msi;

	local_irq_save(flags);

        while ((reg = pciec_reg_read(pcie, GPT_PCIE_MSI_STA))) {
		for_each_set_bit(offset, &reg, INT_PCI_MSI_NR) {
			pciec_reg_write(pcie, GPT_PCIE_MSI_STA, BIT(offset));
                	irqno = irq_find_mapping(msi->domain, offset);
                	if (irqno) {
				if (test_bit(offset, msi->used))
                               		generic_handle_irq(irqno);
				else
					dev_err(pcie->dev, "unhandled MSI:%d\n", irqno);
                	} else {
                        	dev_err(pcie->dev, "unexpected MSI, irqno:%d\n", irqno);
                	}
		}
        }

	local_irq_restore(flags);

        return IRQ_HANDLED;
}

static int gpt_msi_setup_irq(struct msi_chip *chip, struct pci_dev *pdev,
                               struct msi_desc *desc)
{
        struct gpt_msi *msi = to_gpt_msi(chip);
        struct msi_msg msg;
        unsigned int irq;
	phys_addr_t msg_addr;
        int hwirq;

        hwirq = gpt_msi_alloc(msi);
        if (hwirq < 0)
                return hwirq;

        irq = irq_create_mapping(msi->domain, hwirq);
        if (!irq) {
                gpt_msi_free(msi, hwirq);
                return -EINVAL;
        }

        irq_set_msi_desc(irq, desc);

        msg_addr = virt_to_phys((void *)msi->pages);
        /* 32 bit address only */
        msg.address_hi = (msg_addr >> 32);
        msg.address_lo = msg_addr & 0xffffffff;
        msg.data = hwirq;

        write_msi_msg(irq, &msg);

        return 0;
}

static void gpt_msi_teardown_irq(struct msi_chip *chip, unsigned int irq)
{
        struct gpt_msi *msi = to_gpt_msi(chip);
        struct irq_data *d = irq_get_irq_data(irq);
        irq_hw_number_t hwirq = irqd_to_hwirq(d);

        irq_dispose_mapping(irq);
        gpt_msi_free(msi, hwirq);
}

static struct irq_chip gpt_msi_irq_chip = {
        .name = "GPT PCIe MSI",
        .irq_enable = unmask_msi_irq,
        .irq_disable = mask_msi_irq,
        .irq_mask = mask_msi_irq,
        .irq_unmask = unmask_msi_irq,
};

static int gpt_msi_map(struct irq_domain *domain, unsigned int irq,
                         irq_hw_number_t hwirq)
{
        irq_set_chip_and_handler(irq, &gpt_msi_irq_chip, handle_simple_irq);
        irq_set_chip_data(irq, domain->host_data);
        set_irq_flags(irq, IRQF_VALID);

        //tegra_cpuidle_pcie_irqs_in_use();

        return 0;
}

static const struct irq_domain_ops msi_domain_ops = {
        .map = gpt_msi_map,
};

static int gpt_pcie_enable_msi(struct gpt_pciec *pcie)
{
        struct platform_device *pdev = to_platform_device(pcie->dev);
        struct gpt_msi *msi = &pcie->msi;
        unsigned long base;
        int err;
        u32 reg;

        mutex_init(&msi->lock);

        msi->chip.dev = pcie->dev;
        msi->chip.setup_irq = gpt_msi_setup_irq;
        msi->chip.teardown_irq = gpt_msi_teardown_irq;

        msi->domain = irq_domain_add_linear(NULL, INT_PCI_MSI_NR,
                                            &msi_domain_ops, &msi->chip);
        if (!msi->domain) {
                dev_err(&pdev->dev, "failed to create IRQ domain\n");
                return -ENOMEM;
        }

        err = platform_get_irq_byname(pdev, "msi");
        if (err < 0) {
                dev_err(&pdev->dev, "failed to get MSI IRQ: %d\n", err);
                goto err;
        }

        msi->irq = err;
        dev_dbg(&pdev->dev, "MSI to request IRQ: %d\n", err);
        err = request_irq(msi->irq, gpt_pcie_msi_irq, IRQF_SHARED | IRQF_NO_THREAD,
                          gpt_msi_irq_chip.name, pcie);
        if (err < 0) {
                dev_err(&pdev->dev, "failed to request MSI IRQ: %d\n", err);
                goto err;
        }

        /* setup MSI data target */
        msi->pages = __get_free_pages(GFP_KERNEL, 0);
        base = virt_to_phys((void *)msi->pages);

        pciec_reg_write(pcie, GPT_PCIE_MSI_EHADDR, base >> 32);
        pciec_reg_write(pcie, GPT_PCIE_MSI_ELADDR, base & 0xffffffff);

        /* enable MSI  32 vector */
        pciec_reg_write(pcie, GPT_PCIE_MSI_IRQ_EN, 0xffffffff);

        reg = pciec_rcw_read(pcie, GPT_PCIE_MSI_CTRL);
	/*message capable/enable : 32*/
        pciec_rcw_write(pcie, GPT_PCIE_MSI_CTRL, reg | (0x5 << 17) | (0x5 << 20));

        /* and unmask the MSI interrupt */

        return 0;

err:
        irq_domain_remove(msi->domain);
        return err;
}

static int gpt_pcie_disable_msi(struct gpt_pciec *pcie)
{
        struct gpt_msi *msi = &pcie->msi;
        unsigned int i, irq;

        /* disable all MSI*/
        pciec_reg_write(pcie, GPT_PCIE_MSI_IRQ_EN, 0x0);

        free_pages(msi->pages, 0);

        if (msi->irq > 0)
                free_irq(msi->irq, pcie);

        for (i = 0; i < INT_PCI_MSI_NR; i++) {
                irq = irq_find_mapping(msi->domain, i);
                if (irq > 0)
                        irq_dispose_mapping(irq);
        }

        irq_domain_remove(msi->domain);

        return 0;
}

static int gpt_pcie_setup(int nr, struct pci_sys_data *sys)
{
        struct gpt_pciec *pcie = sys_to_pcie(sys);
        list_splice_init(&pcie->list, &sys->resources);

        return 1;
}

static void gpt_pcie_add_bus(struct pci_bus *bus)
{
        if (IS_ENABLED(CONFIG_PCI_MSI)) {
                struct gpt_pciec *pcie  = sys_to_pcie(bus->sysdata);

                pcie->msi.chip.dev = pcie->dev;
                bus->msi = &pcie->msi.chip;
        }
}

static struct pci_bus *gpt_pcie_scan_bus(int nr, struct pci_sys_data *sys)
{
        struct gpt_pciec *pcie = sys_to_pcie(sys);
        struct pci_bus *bus;

        pcie->root_busno = sys->busnr;
        bus = pci_scan_root_bus(pcie->dev, sys->busnr, &gpt_pciec_ops,
                                sys, &sys->resources);
        return bus;
}


static int gpt_pciec_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct gpt_pciec *pcie;
	struct device_node *pcie_intc_node;
	struct hw_pci hw;
	resource_size_t iobase = 0;
	int ret = 0;



	pcie = devm_kzalloc(&pdev->dev, sizeof(struct gpt_pciec), GFP_KERNEL);
	if (!pcie)
		return -ENOMEM;
	
	spin_lock_init(&pcie_lock);

	pcie->node = of_node_get(pdev->dev.of_node);
	pcie->dev = &pdev->dev;

	platform_set_drvdata(pdev, pcie);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "reg");
	pcie->regs = devm_ioremap_resource(pcie->dev, res);
	if (IS_ERR(pcie->regs)) {
	        dev_err(pcie->dev, "missing \"reg\"\n");
		return PTR_ERR(pcie->regs);
	}
	dev_dbg(pcie->dev, "regs map 0x%llx --> 0x%p, len(0x%llx)\n",
			res->start, pcie->regs, res->end - res->start);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "phy");
	pcie->phy = devm_ioremap_resource(pcie->dev, res);
	if (IS_ERR(pcie->phy)) {
	        dev_err(pcie->dev, "missing \"phy\"\n");
		return PTR_ERR(pcie->phy);
	}
	dev_dbg(pcie->dev, "phy map 0x%llx --> 0x%p, len(0x%llx)\n",
			res->start, pcie->phy, res->end - res->start);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "cfg");
	pcie->cfg = devm_ioremap_resource(pcie->dev, res);
	if (IS_ERR(pcie->cfg)) {
	        dev_err(pcie->dev, "missing \"cfg\"\n");
		return PTR_ERR(pcie->cfg);
	}
	pcie->cfg_res = res;
	dev_dbg(pcie->dev, "cfg map 0x%llx --> 0x%p, len(0x%llx)\n",
			res->start, pcie->cfg, res->end - res->start);



	if (of_pci_parse_bus_range(pdev->dev.of_node, &pcie->bus_res)) {
		dev_err(pcie->dev, "failed to parse bus-range property\n");
		return -EINVAL;
	}
	dev_dbg(pcie->dev, "bus range 0x%llx --> 0x%llx\n", pcie->bus_res.start, pcie->bus_res.end);

	/* Setup INTx */
	pcie_intc_node = of_get_next_child(pcie->node, NULL);
	if (!pcie_intc_node) {
		dev_err(pcie->dev, "No PCIe Intc node found\n");
		return PTR_ERR(pcie_intc_node);
	}

	pcie->int_domain = irq_domain_add_linear(pcie_intc_node, GPT_PCI_INT_NR,
					    &gpt_pciec_domain, pcie);
	if (!pcie->int_domain) {
		dev_err(pcie->dev, "failed to create IRQ domain\n");
		return -1;
	}

	pcie->int_chip = &gpt_pciec_int_chip;

       if (IS_ENABLED(CONFIG_PCI_MSI)) {
                ret = gpt_pcie_enable_msi(pcie);
                if (ret < 0) {
                        dev_err(&pdev->dev, "failed to enable MSI support: %d\n", ret);
			return ret;
                }
        }else {
			
		pcie->irq = platform_get_irq_byname(pdev, "intr");
		if (pcie->irq < 0) {
		        dev_err(pcie->dev, "request irq failed\n");
			return -1;
		}
		dev_dbg(pcie->dev, "pcie: request irq(%d)\n", pcie->irq);
	
		ret = request_irq(pcie->irq, gpt_pciec_isr, IRQF_SHARED | IRQF_NO_THREAD, "pciec", pcie);
		if (ret) {
			dev_err(pcie->dev, "failed to register IRQ: %d\n", ret);
			return ret;
		}

	}

	if (gpt_pciec_hwinit(pcie)) {
		return -1;
	}

	INIT_LIST_HEAD(&pcie->list);
	ret = of_pci_get_host_bridge_resources(pcie->node,
					0, 0xff, &pcie->list, &iobase);
	if (ret) {
		dev_err(pcie->dev, "parser host resource failed:%d\n", ret);
		return ret;
	}

	gpt_pcie_parse_ranges(pcie);
	gpt_pcie_parse_dma_ranges(pcie);

	gpt_pciec_map_ranges(pcie, &pcie->list, iobase);

        memset(&hw, 0, sizeof(hw)); 
        hw = (struct hw_pci) {
                .nr_controllers = 1,
                .private_data   = (void **)&pcie,
                .setup          = gpt_pcie_setup,
                .map_irq        = of_irq_parse_and_map_pci,
                .add_bus        = gpt_pcie_add_bus,
                .scan           = gpt_pcie_scan_bus,
                .ops            = &gpt_pciec_ops,
        };
        pci_common_init_dev(pcie->dev, &hw);

	return 0;
}

static void gpt_pcie_free_irq_domain(struct gpt_pciec *pcie)
{
	int i;
	u32 irq, num_irqs;

	/* Free IRQ Domain */
	if (IS_ENABLED(CONFIG_PCI_MSI)) {

		free_pages(pcie->msi_pages, 0);

		num_irqs = GPT_NUM_MSI_IRQS;
	} else {
		/* INTx */
		num_irqs = 4;
	}

	for (i = 0; i < num_irqs; i++) {
		irq = irq_find_mapping(pcie->int_domain, i);
		if (irq > 0)
			irq_dispose_mapping(irq);
	}

	irq_domain_remove(pcie->int_domain);
}

static int gpt_pcie_remove(struct platform_device *pdev)
{
	struct gpt_pciec *pcie = platform_get_drvdata(pdev);

	gpt_pcie_free_irq_domain(pcie);

	return 0;
}

static const struct of_device_id gpt_pcie_match_table[] = {
	{.compatible = "gpt,gpt-pciec",},
	{},
};

static struct platform_driver gpt_pcie_driver = {
	.driver = {
		   .name = "gpt-pciec",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(gpt_pcie_match_table),
	},
	.probe = gpt_pciec_probe,
	.remove = gpt_pcie_remove,
};
module_platform_driver(gpt_pcie_driver);

MODULE_DESCRIPTION("GPT pcie host controller driver");
MODULE_AUTHOR("qqliang <qqliang@gpt.com>");
MODULE_LICENSE("GPL");
