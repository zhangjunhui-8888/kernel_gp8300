#
# Makefile for the linux gpt-specific parts of the memory manager.
#

obj-y	:= fault.o tlb.o init.o ioremap.o pgtable.o cache.o\
		dma-mapping.o context.o extable.o mmap.o alignment.o

obj-$(CONFIG_SMP)               += smp_tlb.o  
obj-$(CONFIG_CPU_GPT_CHIP2)	+=cache-chip2.o tlb-chip2.o

