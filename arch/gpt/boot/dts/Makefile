dtb-$(CONFIG_ARCH_GPT_CHIP2) += gpt_chip2_sim.dtb  gpt_chip2_chase.dtb  gpt_polaris_fpga_xc7v2000.dtb  gpt_chip2_polaris.dtb gpt_gp8300_dvb.dtb gpt_gp8300_evk.dtb gpt_gp8300_chaokong.dtb 


targets += dtbs
targets += $(dtb-y)

dtbs: $(addprefix $(obj)/, $(dtb-y))

ifneq '$(CONFIG_GPT_BUILTIN_DTB_NAME)' '""'
BUILTIN_DTB := $(patsubst "%",%,$(CONFIG_GPT_BUILTIN_DTB_NAME)).dtb.o
else
BUILTIN_DTB :=
endif
obj-y += $(BUILTIN_DTB)

clean-files := *.dtb.S *.dtb
