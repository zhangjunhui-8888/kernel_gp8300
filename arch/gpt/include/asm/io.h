/*
 * gpt Linux
 *
 * Linux architectural port borrowing liberally from similar works of
 * others.  All original copyrights apply as per the original source
 * declaration.
 *
 * gpt implementation:
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __ASM_GPT_IO_H
#define __ASM_GPT_IO_H

#include <linux/types.h>
#include <linux/mm_types.h>

#include <asm/sizes.h>
#include <asm/byteorder.h>
#include <asm/barrier.h>
#include <asm/pgtable.h>
#include <asm/early_ioremap.h>

/*
 * Generic IO read/write.  These perform native-endian accesses.
 */
static inline void __raw_writeb(u8 val, volatile void __iomem *addr)
{
	asm volatile("stb %0, %1" : : "r" (val), "a" (addr));
}

static inline void __raw_writew(u16 val, volatile void __iomem *addr)
{
	asm volatile("sth %0, %1" : : "r" (val), "a" (addr));
}

static inline void __raw_writel(u32 val, volatile void __iomem *addr)
{
	asm volatile("stw %0, %1" : : "r" (val), "a" (addr));
}

static inline void __raw_writeq(u64 val, volatile void __iomem *addr)
{
	asm volatile("stl %0, %1" : : "r" (val), "a" (addr));
}

static inline u8 __raw_readb(const volatile void __iomem *addr)
{
	u8 val;
	asm volatile("ldb %0, %1\n"
		     : "=r" (val) : "a" (addr));
	return val;
}

static inline u16 __raw_readw(const volatile void __iomem *addr)
{
	u16 val;

	asm volatile("ldh %0, %1\n"
		     : "=r" (val) : "a" (addr));
	return val;
}

static inline u32 __raw_readl(const volatile void __iomem *addr)
{
	u32 val;
	asm volatile("ldw %0, %1\n"
		     : "=r" (val) : "a" (addr));
	return val;
}

static inline u64 __raw_readq(const volatile void __iomem *addr)
{
	u64 val;
	asm volatile("ldl %0, %1\n"
		     : "=r" (val) : "a" (addr));
	return val;
}

/* IO barriers, MEM_IO is enabled so we don't need mb() any more */
#define __iormb()
#define __iowmb()

#define mmiowb()		do { } while (0)

static inline void memcpy_fromio(void * to, const volatile void __iomem *from,
                                 unsigned long count)
{
        memcpy(to, (const void __force *)from, count);
}

static inline void  memcpy_toio(volatile void __iomem *to, const void * from,
                                unsigned long count)
{
        memcpy((void __force *)to, from, count);
}

static inline void memset_io(volatile void __iomem *addr, unsigned char val,
                             unsigned long count)
{
        memset((void __force *)addr, val, count);
}

/* IO barriers, MEM_IO is enabled so we don't need mb() any more */
#define __iormb()
#define __iowmb()

#define mmiowb()		do { } while (0)

/*
 * Relaxed I/O memory access primitives. These follow the Device memory
 * ordering rules but do not guarantee any ordering relative to Normal memory
 * accesses.
 */
#define readb_relaxed(c)	({ u8  __v = __raw_readb(c); __v; })
#define readw_relaxed(c)	({ u16 __v = le16_to_cpu((__force __le16)__raw_readw(c)); __v; })
#define readl_relaxed(c)	({ u32 __v = le32_to_cpu((__force __le32)__raw_readl(c)); __v; })
#define readq_relaxed(c)	({ u64 __v = le64_to_cpu((__force __le64)__raw_readq(c)); __v; })

#define writeb_relaxed(v,c)	((void)__raw_writeb((v),(c)))
#define writew_relaxed(v,c)	((void)__raw_writew((__force u16)cpu_to_le16(v),(c)))
#define writel_relaxed(v,c)	((void)__raw_writel((__force u32)cpu_to_le32(v),(c)))
#define writeq_relaxed(v,c)	((void)__raw_writeq((__force u64)cpu_to_le64(v),(c)))

/*
 * I/O memory access primitives. Reads are ordered relative to any
 * following Normal memory access. Writes are ordered relative to any prior
 * Normal memory access.
 */
#define readb(c)		({ u8  __v = readb_relaxed(c); __iormb(); __v; })
#define readw(c)		({ u16 __v = readw_relaxed(c); __iormb(); __v; })
#define readl(c)		({ u32 __v = readl_relaxed(c); __iormb(); __v; })
#define readq(c)		({ u64 __v = readq_relaxed(c); __iormb(); __v; })

#define writeb(v,c)		({ __iowmb(); writeb_relaxed((v),(c)); })
#define writew(v,c)		({ __iowmb(); writew_relaxed((v),(c)); })
#define writel(v,c)		({ __iowmb(); writel_relaxed((v),(c)); })
#define writeq(v,c)		({ __iowmb(); writeq_relaxed((v),(c)); })

/*
 *  I/O port access primitives.
 */
#define arch_has_dev_port()	(1)
#define IO_SPACE_LIMIT		0xffffffffffUL
#define IOBASE		((void __iomem *)(0))


static inline u8 inb(unsigned long addr)
{
	return readb(addr + IOBASE);
}

static inline u16 inw(unsigned long addr)
{
	return readw(addr + IOBASE);
}

static inline u32 inl(unsigned long addr)
{
	return readl(addr + IOBASE);
}

static inline void outb(u8 b, unsigned long addr)
{
	writeb(b, addr + IOBASE);
}

static inline void outw(u16 b, unsigned long addr)
{
	writew(b, addr + IOBASE);
}

static inline void outl(u32 b, unsigned long addr)
{
	writel(b, addr + IOBASE);
}

#define inb_p(addr)	inb(addr)
#define inw_p(addr)	inw(addr)
#define inl_p(addr)	inl(addr)

#define outb_p(x, addr)	outb((x), (addr))
#define outw_p(x, addr)	outw((x), (addr))
#define outl_p(x, addr)	outl((x), (addr))

static inline void insb(unsigned long addr, void *buffer, int count)
{
	u8 *buf = buffer;
	while (count--)
		*buf++ = __raw_readb(addr + IOBASE);
}

static inline void insw(unsigned long addr, void *buffer, int count)
{
	u16 *buf = buffer;
	while (count--)
		*buf++ = __raw_readw(addr + IOBASE);
}

static inline void insl(unsigned long addr, void *buffer, int count)
{
	u32 *buf = buffer;
	while (count--)
		*buf++ = __raw_readl(addr + IOBASE);
}

static inline void outsb(unsigned long addr, const void *buffer, int count)
{
	const u8 *buf = buffer;
	while (count--)
		__raw_writeb(*buf++, addr + IOBASE);
}

static inline void outsw(unsigned long addr, const void *buffer, int count)
{
	const u16 *buf = buffer;
	while (count--)
		__raw_writew(*buf++, addr + IOBASE);
}

static inline void outsl(unsigned long addr, const void *buffer, int count)
{
	const u32 *buf = buffer;
	while (count--)
		__raw_writel(*buf++, addr + IOBASE);
}

#define insb_p(port,to,len)	insb(port,to,len)
#define insw_p(port,to,len)	insw(port,to,len)
#define insl_p(port,to,len)	insl(port,to,len)

#define outsb_p(port,from,len)	outsb(port,from,len)
#define outsw_p(port,from,len)	outsw(port,from,len)
#define outsl_p(port,from,len)	outsl(port,from,len)


extern void __iomem *ioport_map(unsigned long port, unsigned int nr);
extern void ioport_unmap(void __iomem *addr);

static inline unsigned long virt_to_phys(volatile void *address)
{
        return __pa((unsigned long)address);
}

static inline void *phys_to_virt(unsigned long address)
{
        return __va(address);
}

#include <asm-generic/iomap.h>

/*
 * Convert a physical pointer to a virtual kernel pointer for /dev/mem
 * access
 */
#define xlate_dev_mem_ptr(p)	__va(p)

/*
 * Convert a virtual cached pointer to an uncached pointer
 */
#define xlate_dev_kmem_ptr(p)	p


extern void __iomem *__ioremap(phys_addr_t offset, unsigned long size,
                                pgprot_t prot);
extern void __iounmap(volatile void __iomem *io_addr);

#define ioremap(addr, size)		__ioremap((addr), (size), __pgprot(_PAGE_DEVICE))
#define ioremap_nocache(addr, size)	__ioremap((addr), (size), __pgprot(_PAGE_DEVICE))
#define iounmap				__iounmap

#define __io_virt(x) ((void __force *) (x))

#ifndef memcpy_fromio
#define memcpy_fromio(a, b, c)	memcpy((a), __io_virt(b), (c))
#endif
#ifndef memcpy_toio
#define memcpy_toio(a, b, c)	memcpy(__io_virt(a), (b), (c))
#endif
#endif /* __ASM_GPT_IO_H */
