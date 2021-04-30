#
# arch/gpt/boot/Makefile
#

targets := Image zImage vmlinux.bin  uImage 

OBJCOPYFLAGS= -O binary -R .note -R .note.gnu.build-id -R .comment -S

UIMAGE_LOADADDR  = $(VMLINUX_LOAD_ADDRESS)
UIMAGE_ENTRYADDR = $(VMLINUX_ENTRY_ADDRESS)

suffix-y := bin
suffix-$(CONFIG_KERNEL_GZIP)	:= gz


$(obj)/Image: vmlinux FORCE
	$(call if_changed,objcopy)
	@echo '  Kernel: $@ is ready'

$(obj)/compressed/vmlinux: $(obj)/Image FORCE
	$(Q)$(MAKE) $(build)=$(obj)/compressed $@

$(obj)/zImage: $(obj)/compressed/vmlinux FORCE
	$(call if_changed,objcopy)
	@echo '  Kernel: $@ is ready'

$(obj)/uImage: $(obj)/zImage FORCE
	$(call if_changed,uimage)
	@echo '  Image $@ is ready'

$(obj)/vmlinux.bin: vmlinux FORCE
	$(call if_changed,objcopy)
	@$(kecho) '  Kernel: $@ is ready'
