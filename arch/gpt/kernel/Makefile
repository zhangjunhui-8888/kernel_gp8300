#
# Makefile for the linux kernel.
#

extra-y	:= head.o vmlinux.lds

CFLAGS_REMOVE_ftrace.o = -pg
CFLAGS_REMOVE_insn.o = -pg
CFLAGS_REMOVE_return_address.o = -pg

obj-y	:= entry.o setup.o process.o stacktrace.o insn.o\
	   traps.o time.o irq.o ptrace.o signal.o cpuinfo.o\
	   sys_call_table.o reboot.o vdso.o entry-vfp.o vfp.o \
	   return_address.o

obj-$(CONFIG_MODULES)			+= gptksyms.o module.o	   
obj-$(CONFIG_CPU_GPT_CHIP2)		+= trap-chip2.o
obj-$(CONFIG_SMP)			+= smp.o topology.o smp-sync-timer.o
obj-$(CONFIG_FUNCTION_TRACER)    	+= mcount.o ftrace.o
obj-$(CONFIG_FUNCTION_GRAPH_TRACER)     += ftrace.o
obj-$(CONFIG_FTRACE_SYSCALLS)   	+= ftrace.o
obj-$(CONFIG_VDSO)			+= vdso/
obj-$(CONFIG_HAVE_HW_BREAKPOINT)	+= hw_breakpoint.o step.o
obj-$(CONFIG_PCI)			+= pci.o
# vDSO - this must be built first to generate the symbol offsets
$(call objectify,$(obj-y)): $(obj)/vdso/vdso-offsets.h
$(obj)/vdso/vdso-offsets.h: $(obj)/vdso
