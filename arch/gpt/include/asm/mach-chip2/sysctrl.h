#ifndef __GPT_SYSCTRL_H__
#define __GPT_SYSCTRL_H__

/* Bits of SYSCTRL_EXTCR_LO register */
#define	GSNN0_EN	(1 << 0)
#define	GSNN0_CLK_EN	(1 << 1)
#define	GSNN1_EN	(1 << 2)
#define	GSNN1_CLK_EN	(1 << 3)
#define	VCAP0_EN	(1 << 4)
#define	VCAP1_EN	(1 << 5)
#define	VCAP2_EN	(1 << 6)
#define	VOUT_EN		(1 << 7)
#define	VIDMEM_EN	(1 << 8)
#define	SPINOR_EN	(1 << 9)
#define	SD_EN		(1 << 10)
#define	PCIE_EN		(1 << 11)
#define	DDR_EN		(1 << 12)
#define	VID_EN		(1 << 13)
#define DDR_CLK_EN	(1 << 14)

/* Bits of SYSCTRL_EXTCR_HI register */
#define	L2LO_MEM_SLP	(1 << 0)
#define	L2LO_MEM_SD	(1 << 1)
#define	L2HI_MEM_SLP	(1 << 2)
#define	L2HI_MEM_SD	(1 << 3)
#define GSNN0_MEM_SLP	(1 << 4)
#define GSNN0_MEM_SD	(1 << 5)
#define GSNN1_MEM_SLP	(1 << 6)
#define GSNN1_MEM_SD	(1 << 7)
#define VIDMEM_MEM_SLP	(1 << 8)
#define VIDMEM_MEM_SD	(1 << 9)

enum sysreg_off {
	SYSREG_CORE0_CCR	= 0x00000000,
	SYSREG_CORE1_CCR	= 0x00000004,
	SYSREG_CORE2_CCR	= 0x00000008,
	SYSREG_CORE3_CCR	= 0x0000000C,
	SYSREG_EXTAPB_CCR	= 0x00000010,
	SYSREG_ACC_DEL		= 0x0000001C,
	SYSREG_CORE0_ISLR	= 0x00000020,
	SYSREG_CORE1_ISLR	= 0x00000024,
	SYSREG_CORE2_ISLR	= 0x00000028,
	SYSREG_CORE3_ISLR	= 0x0000002C,
	SYSREG_PLL_CFG_RD	= 0x00000030,
	SYSREG_CPLL_CNTL	= 0x00000034,
	SYSREG_DPLL_CNTL	= 0x00000038,
	SYSREG_EXTCR_LO		= 0x00000100,
	SYSREG_EXTCR_HI		= 0x00000104,
	SYSREG_IOCNF		= 0x00000200
};

extern void gpt_sysctrl_write(enum sysreg_off offset, u32 val);
extern u32 gpt_sysctrl_read(enum sysreg_off offset);
extern void gpt_sysctrl_cpu_disable(u32 val);
extern void gpt_sysctrl_subsys_enable(u32 val);
extern void gpt_sysctrl_subsys_disable(uint32_t val);
extern void gpt_sysctrl_mem_enable(u32 val);
extern void gpt_sysctrl_mem_disable(u32 val);

#endif
