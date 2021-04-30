#
# GPT Clock specific Makefile
#

obj-y	+= clk.o clk_pll.o

obj-$(CONFIG_PLAT_GPT_CHIP2_SIM)		+= clk-chip2dummy.o
obj-$(CONFIG_PLAT_GPT_CHIP2_CHASE_FPGA)		+= clk-chip2chase.o
obj-$(CONFIG_PLAT_GPT_CHIP2_POLARIS_FPGA)	+= clk-chip2dummy.o
obj-$(CONFIG_PLAT_GPT_CHIP2_POLARIS_EVK)	+= clk-chip2polaris.o
obj-$(CONFIG_PLAT_GPT_CHIP2_POLARIS_DVB)	+= clk-chip2polaris.o
obj-$(CONFIG_PLAT_GPT_CHIP2_POLARIS_CHAOKONG)	+= clk-chip2polaris.o
