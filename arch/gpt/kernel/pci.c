/*
 * Copyright (C) hxgpt.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 */

#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/of_pci.h>
#include <linux/pci.h>
#include <linux/of_platform.h>
#include <linux/slab.h>

#if defined(CONFIG_PCI_GPT)
int gpt_pciec_irq_map(struct pci_dev *pdev);
#else
int gpt_pciec_irq_map(struct pci_dev *pdev)
{
	return 0;
}
#endif

int pcibios_enable_device(struct pci_dev *dev, int mask)
{
	u16 cmd, oldcmd;
	int i;

	pci_read_config_word(dev, PCI_COMMAND, &cmd);
	oldcmd = cmd;

	for (i = 0; i < PCI_NUM_RESOURCES; i++) {
		struct resource *res = &dev->resource[i];

		/* Only set up the requested stuff */
		if (!(mask & (1 << i)))
			continue;

		if (res->flags & IORESOURCE_IO)
			cmd |= PCI_COMMAND_IO;
		if (res->flags & IORESOURCE_MEM)
			cmd |= PCI_COMMAND_MEMORY;
	}

	if (cmd != oldcmd) {
		printk(KERN_DEBUG "PCI: Enabling device: (%s), cmd %x\n",
		       pci_name(dev), cmd);
                /* Enable the appropriate bits in the PCI command register.  */
		pci_write_config_word(dev, PCI_COMMAND, cmd);
	}
	return 0;
}

void pcibios_fixup_resources(struct pci_dev *dev, int start, int limit)
{
	int i;

	if (dev->bus->number == 0)
		return;

	for (i = start; i < limit; i++) {
		if (!dev->resource[i].flags)
			continue;

		pci_claim_resource(dev, i);
	}
}

void pcibios_fixup_device_resources(struct pci_dev *dev)
{

}

/*
 * Called after each bus is probed, but before its children are examined
 */
void pcibios_fixup_bus(struct pci_bus *bus)
{
	struct pci_dev *dev;

	list_for_each_entry(dev, &bus->devices, bus_list)
		pcibios_fixup_device_resources(dev);

}

/*
 * We don't have to worry about legacy ISA devices, so nothing to do here
 */
resource_size_t pcibios_align_resource(void *data, const struct resource *res,
				resource_size_t size, resource_size_t align)
{
	return res->start;
}

/*
 * Try to assign the IRQ number from DT when adding a new device
 */
int pcibios_add_device(struct pci_dev *dev)
{
	//dev->irq = of_irq_parse_and_map_pci(dev, 0, 0);
	return 0;
}

void pcibios_add_bus(struct pci_bus *bus)
{
        struct pci_sys_data *sys = bus->sysdata;
        if (sys->add_bus)
                sys->add_bus(bus);
}

void pcibios_remove_bus(struct pci_bus *bus)
{
        struct pci_sys_data *sys = bus->sysdata;
        if (sys->remove_bus)
                sys->remove_bus(bus);
}

/*
 * Map a slot/pin to an IRQ.
 */
static int pcibios_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
{
        struct pci_sys_data *sys = dev->sysdata;
        int irq = -1;

        if (sys->map_irq)
                irq = sys->map_irq(dev, slot, pin);

        printk(KERN_DEBUG "PCI: %s mapping slot %d pin %d => irq %d\n",
                pci_name(dev), slot, pin, irq);

        return irq;
}

static int pcibios_init_resources(int busnr, struct pci_sys_data *sys)
{
        int ret;
        struct pci_host_bridge_window *window;

        if (list_empty(&sys->resources)) {
                pci_add_resource_offset(&sys->resources,
                         &iomem_resource, sys->mem_offset);
        }

        list_for_each_entry(window, &sys->resources, list) {
                if (resource_type(window->res) == IORESOURCE_IO)
                        return 0;
        }

        sys->io_res.start = (busnr * SZ_64K) ?  : 0x1000;
        sys->io_res.end = (busnr + 1) * SZ_64K - 1;
        sys->io_res.flags = IORESOURCE_IO;
        sys->io_res.name = sys->io_res_name;
        sprintf(sys->io_res_name, "PCI%d I/O", busnr);

        ret = request_resource(&ioport_resource, &sys->io_res);
        if (ret) {
                pr_err("PCI: unable to allocate I/O port region (%d)\n", ret);
                return ret;
        }
        pci_add_resource_offset(&sys->resources, &sys->io_res,
                                sys->io_offset);

        return 0;
}
static void pcibios_init_hw(struct device *parent, struct hw_pci *hw,
                            struct list_head *head)
{
        struct pci_sys_data *sys = NULL;
        int ret;
        int nr, busnr;

        for (nr = busnr = 0; nr < hw->nr_controllers; nr++) {
                sys = kzalloc(sizeof(struct pci_sys_data), GFP_KERNEL);
                if (!sys)
                        panic("PCI: unable to allocate sys data!");

#ifdef CONFIG_PCI_DOMAINS
                sys->domain  = hw->domain;
#endif
                sys->busnr   = busnr;
                sys->swizzle = hw->swizzle;
                sys->map_irq = hw->map_irq;
                sys->align_resource = hw->align_resource;
                sys->add_bus = hw->add_bus;
                sys->remove_bus = hw->remove_bus;
                INIT_LIST_HEAD(&sys->resources);

                if (hw->private_data)
                        sys->private_data = hw->private_data[nr];

                ret = hw->setup(nr, sys);
                if (ret > 0) {
                        ret = pcibios_init_resources(nr, sys);
                        if (ret)  {
                                kfree(sys);
                                break;
                        }

                        if (hw->scan)
                                sys->bus = hw->scan(nr, sys);
                        else
                                sys->bus = pci_scan_root_bus(parent, sys->busnr,
                                                hw->ops, sys, &sys->resources);

                        if (!sys->bus)
                                panic("PCI: unable to scan bus!");

                        busnr = sys->bus->busn_res.end + 1;

                        list_add(&sys->node, head);
                } else {
                        kfree(sys);
                        if (ret < 0)
                                break;
                }
        }
}

void pci_common_init_dev(struct device *parent, struct hw_pci *hw)
{
        struct pci_sys_data *sys;
        LIST_HEAD(head);

        pci_add_flags(PCI_REASSIGN_ALL_RSRC);
        if (hw->preinit)
                hw->preinit();
        pcibios_init_hw(parent, hw, &head);
        if (hw->postinit)
                hw->postinit();

        pci_fixup_irqs(pci_common_swizzle, pcibios_map_irq);

        list_for_each_entry(sys, &head, node) {
                struct pci_bus *bus = sys->bus;

                if (!pci_has_flag(PCI_PROBE_ONLY)) {
                       /*
                             * Size the bridge windows.
                       */
                        pci_bus_size_bridges(bus);

                        /*
                             * Assign resources.
                         */
                        pci_bus_assign_resources(bus);
                }

                /*
                    * Tell drivers about devices found.
                  */
                pci_bus_add_devices(bus);
        }
       list_for_each_entry(sys, &head, node) {
                struct pci_bus *bus = sys->bus;

                /* Configure PCI Express settings */
                if (bus && !pci_has_flag(PCI_PROBE_ONLY)) {
                        struct pci_bus *child;

                        list_for_each_entry(child, &bus->children, node)
                                pcie_bus_configure_settings(child);
                }
        }
}

