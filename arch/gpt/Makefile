# BK Id: %F% %I% %G% %U% %#%
#
# This file is included by the global makefile so that you can add your own
# architecture-specific flags and dependencies. Remember to do have actions
# for "archclean" and "archdep" for cleaning up and making dependencies for
# this architecture
#
# This file is subject to the terms and conditions of the GNU General Public
# License.  See the file "COPYING" in the main directory of this archive
# for more details.
#
# Copyright (C) 1994 by Linus Torvalds
# Modifications for the gpt architecture:
#
# Based on:
# arch/i386/Makefile

UTS_MACHINE := gpt

ifeq ($(CROSS_COMPILE),)
CROSS_COMPILE := gpt-
endif

LDFLAGS         :=
OBJCOPYFLAGS    := -O binary -R .note -R .comment -S
LDFLAGS_vmlinux :=
LIBGCC 		:= $(shell $(CC) $(KBUILD_CFLAGS) -print-libgcc-file-name)

KBUILD_CFLAGS	+= -pipe -D__linux__

MARCH_FLAGS     := -DMARCH_CHIP1=1 -DMARCH_CHIP2=2 -DMARCH_GPGPU=4
ifdef CONFIG_ARCH_GPT_CHIP2
MARCH_FLAGS     += -DUNIMPLEMENTED_MARCH_VERSION=MARCH_CHIP2 
MARCH_FLAGS     += -Wa,--stack=a15 -Wno-attributes
MARCH_FLAGS	+= -I$(GPTXROOT)/tools/gptx-linux-gnu/include
endif
KBUILD_CFLAGS   += $(MARCH_FLAGS)
KBUILD_AFLAGS   += $(MARCH_FLAGS)
KBUILD_CPPFLAGS   += $(MARCH_FLAGS)

KBUILD_CFLAGS   += -mdisable-floating-varargs
ifdef CONFIG_DYNAMIC_FTRACE
KBUILD_CFLAGS   += -fno-schedule-insns -fno-schedule-insns2 -fno-schedule-fusion
endif

KBUILD_AFLAGS   += -DNEW_CALLING_CONVENTIONS

ifdef CONFIG_PHYSICAL_START
load-y				= $(CONFIG_PHYSICAL_START)
endif
entry-y				= $$(readelf -h vmlinux | \
				grep "Entry point address" | grep -o 0x.*)

KBUILD_CPPFLAGS += -DVMLINUX_LOAD_ADDRESS=$(load-y)
bootvars-y	= VMLINUX_LOAD_ADDRESS=$(load-y) \
		  VMLINUX_ENTRY_ADDRESS=$(entry-y)

head-y 		:= arch/gpt/kernel/head.o

core-y		+= arch/gpt/kernel/ \
		   arch/gpt/mm/
libs-y		+= arch/gpt/lib/ $(LIBGCC)

core-$(CONFIG_GPT_BUILTIN_DTB)	+= arch/gpt/boot/dts/
core-$(CONFIG_ARCH_GPT_CHIP2)	+= arch/gpt/mach-chip2/


KBUILD_IMAGE := vmlinux.bin dtbs

all: $(KBUILD_IMAGE)

boot := arch/gpt/boot

BOOT_TARGETS	= zImage uImage uImage.gz vmlinux.bin
INSTALL_TARGETS	= zinstall uinstall install

PHONY += $(BOOT_TARGETS) $(INSTALL_TARGETS)

$(BOOT_TARGETS): vmlinux
	$(Q)$(MAKE) $(build)=$(boot) MACHINE=$(MACHINE) $(bootvars-y) $(boot)/$@

$(INSTALL_TARGETS):
	$(Q)$(MAKE) $(build)=$(boot) MACHINE=$(MACHINE) $@

%.dtb: | scripts
	$(Q)$(MAKE) $(build)=$(boot)/dts MACHINE=$(MACHINE) $(boot)/dts/$@

PHONY += dtbs dtbs_install
dtbs dtbs_install: prepare scripts
	$(Q)$(MAKE) $(build)=$(boot)/dts MACHINE=$(MACHINE) $@

PHONY += vdso_install
vdso_install:
	$(Q)$(MAKE) $(build)=arch/gpt/kernel/vdso $@


archclean:
	$(Q)$(MAKE) $(clean)=$(boot)

define archhelp
	echo '  install              - install kernel into $(INSTALL_PATH)'
	echo '* vmlinux.bin          - Raw binary boot image'
	echo '* uImage               - U-Boot image'
	echo '  uImage.gz            - U-Boot image (gzip)'
	echo '  dtbs                 - Device-tree blobs for enabled boards'
	echo
	echo '  These will be default as appropriate for a configured platform.'
endef
