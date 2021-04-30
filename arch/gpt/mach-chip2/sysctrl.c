#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/string.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>

#include <asm/mach-chip2/sysctrl.h>

/* Mask of SYSCTRL_EXTCR_LO register */
#define EXTCR_CLK_MASK  (GSNN0_CLK_EN | GSNN1_CLK_EN | \
			DDR_CLK_EN)
#define EXTCR_SUBSYS_MASK (0xFFFF & (~EXTCR_CLK_MASK))

static u32 *sysctrl_base = NULL;

void gpt_sysctrl_write(enum sysreg_off offset, u32 val)
{
	u32* reg;
	if(offset > SYSREG_IOCNF)
		return ;
	reg = (u32*)((unsigned long)sysctrl_base + offset);
	writel(val, reg);
}

u32 gpt_sysctrl_read(enum sysreg_off offset)
{
	u32* reg;
	if(offset > SYSREG_IOCNF)
		return -ENOMEM;
	reg = (u32*)((unsigned long)sysctrl_base + offset);
	
	return readl(reg);
}

void gpt_sysctrl_subsys_enable(u32 val)
{
        u32 tmp, bitmask;
	u32 *reg = (u32*)((unsigned long)sysctrl_base + SYSREG_EXTCR_LO);
	
        val &= EXTCR_SUBSYS_MASK;
        bitmask = (val << 16);
        tmp = val | bitmask;
        writel(tmp, reg);
}
EXPORT_SYMBOL(gpt_sysctrl_subsys_enable);

void gpt_sysctrl_subsys_disable(u32 val)
{
        u32 tmp, bitmask;
	u32 *reg = (u32*)((unsigned long)sysctrl_base + SYSREG_EXTCR_LO);
	
        val &= EXTCR_SUBSYS_MASK;
        bitmask = (val << 16);
        tmp = (~val) & bitmask;
        writel(tmp, reg);
}
EXPORT_SYMBOL(gpt_sysctrl_subsys_disable);

void gpt_sysctrl_mem_enable(u32 val)
{
        u32 tmp, bitmask;
	u32 *reg = (u32*)((unsigned long)sysctrl_base + SYSREG_EXTCR_HI);
	
        bitmask = val << 16;
        tmp = (~val) & bitmask;
        writel(tmp, reg);
}
EXPORT_SYMBOL(gpt_sysctrl_mem_enable);

void gpt_sysctrl_mem_disable(u32 val)
{
        u32 tmp, bitmask;
	u32 *reg = (u32*)((unsigned long)sysctrl_base + SYSREG_EXTCR_HI);
	
        bitmask = val << 16;
        tmp = val | bitmask;
        writel(tmp, reg);
}
EXPORT_SYMBOL(gpt_sysctrl_mem_disable);

static int __init gpt_sysctrl_init(void)
{
	struct device_node *np;	
	np = of_find_node_by_name(NULL, "system-ctrl");
	if(!np){		
		pr_err("Device node %s is not found\n", np->full_name);
		return -ENODEV;
	}
	sysctrl_base = of_iomap(np, 0);
	if(!sysctrl_base){
		pr_err("%s can not be remaped\n", np->full_name);
		return -ENOMEM;
	}

#ifdef CONFIG_MMC_SDHCI_OF_GPT 
	gpt_sysctrl_subsys_enable(SD_EN);
#endif

#ifdef CONFIG_GPT_VIDMEM
	gpt_sysctrl_subsys_enable(VID_EN);
        gpt_sysctrl_mem_enable(VIDMEM_MEM_SD | VIDMEM_MEM_SLP);
        gpt_sysctrl_subsys_enable(VIDMEM_EN | VID_EN);
#endif

	return 0;
}
early_initcall(gpt_sysctrl_init);
