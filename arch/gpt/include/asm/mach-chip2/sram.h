#ifndef __ASM_SRAM_H
#define __ASM_SRAM_H

#include <linux/sched.h>
#include <linux/cpumask.h>
#include <linux/genalloc.h>

#include <uapi/asm/sram.h>

static cpumask_t cpus_allowed = CPU_MASK_NONE;

#define sram_save_cpu_affinity()	\
	do {	\
		cpumask_copy(&old_cpus_allowed, tsk_cpus_allowed(current));	\
		set_cpus_allowed_ptr(current, &cpus_allowed);	\
	}while(0)

#define sram_restore_cpu_affinity()	\
	do {	\
		set_cpus_allowed_ptr(current, &old_cpus_allowed);	\
	}while(0)

static inline u8 sram_readb(void *addr)
{
	u8 val;
        cpumask_t old_cpus_allowed;

	sram_save_cpu_affinity();
	val = readb(addr);
	sram_restore_cpu_affinity();

	return val;
}

static inline u16 sram_readw(void *addr)
{
	u16 val;
        cpumask_t old_cpus_allowed;

	sram_save_cpu_affinity();
	val = readw(addr);
	sram_restore_cpu_affinity();

	return val;
}

static inline u32 sram_readl(void *addr)
{
	u32 val;
        cpumask_t old_cpus_allowed;

	sram_save_cpu_affinity();
	val = readl(addr);
	sram_restore_cpu_affinity();

	return val;
}

static inline u64 sram_readq(void *addr)
{
	u64 val;
        cpumask_t old_cpus_allowed;

	sram_save_cpu_affinity();
	val = readq(addr);
	sram_restore_cpu_affinity();

	return val;
}

static inline void sram_writeb(u8 val, void *addr)
{
        cpumask_t old_cpus_allowed;

	sram_save_cpu_affinity();
	writeb(val, addr);
	sram_restore_cpu_affinity();	
}

static inline void sram_writew(u16 val, void *addr)
{
        cpumask_t old_cpus_allowed;

	sram_save_cpu_affinity();
	writew(val, addr);
	sram_restore_cpu_affinity();	
}

static inline void sram_writel(u32 val, void *addr)
{
        cpumask_t old_cpus_allowed;

	sram_save_cpu_affinity();
	writel(val, addr);
	sram_restore_cpu_affinity();
}

static inline void sram_writeq(u64 val, void *addr)
{
        cpumask_t old_cpus_allowed;

	sram_save_cpu_affinity();
	writeq(val, addr);
	sram_restore_cpu_affinity();	
}

static inline void sram_copy(void *dst, void *src, size_t size)
{
        cpumask_t old_cpus_allowed;

	sram_save_cpu_affinity();
        memcpy(dst, src, size);
	sram_restore_cpu_affinity();
}

/* arch/gpt/mach-chip2/sramm.c */
extern struct gen_pool *sram_get_gen_pool(void);
extern void *sram_alloc(size_t size);
extern void *sram_dma_alloc(size_t size, dma_addr_t *dma);
extern void sram_free(void *addr, size_t len);
extern bool sram_dma_copy(dma_addr_t dst, dma_addr_t src, size_t size);

#endif /* __ASM_SRAM_H */

