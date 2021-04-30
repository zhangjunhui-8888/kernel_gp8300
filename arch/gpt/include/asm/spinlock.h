/*
 * arch/gpt/include/asm/spinlock.h
 *
 * Spinlocks for SMP operations.
 *
 * Copyright (C) 2015-2017, Optimum Semiconductor Technologies
 *  Enrique Barria <ebarria@optimumsemi.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see the file COPYING, or write
 * to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */
/*
 * Copyright (C) 2004, 2007-2010, 2011-2012 Synopsys, Inc. (www.synopsys.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_GPT_SPINLOCK_H
#define __ASM_GPT_SPINLOCK_H

#include <linux/smp.h>
#include <asm/spinlock_types.h>
#include <asm/processor.h>
#include <asm/barrier.h>
#include <machine/gpt_mach.h>

#define arch_spin_lock_flags(lock, flags)       arch_spin_lock(lock)
#define arch_spin_unlock_wait(x) \
        do { while (arch_spin_is_locked(x)) cpu_relax(); } while (0)
#define XEN_NUM "17"
extern void __iomem *gpt_mpic_sgi_clear;
extern void __iomem *gpt_mpic_sgi_set;

static inline void arch_spin_lock(arch_spinlock_t *lock)
{
	unsigned long lock_align = DCACHE_ALIGNED(lock);
	u32 oldval, newval;
	volatile unsigned int *mpic_clear = 
	  ((unsigned int *)gpt_mpic_sgi_clear) + ((sprget_gpt(THID) >> 8) & 0x3);
	unsigned long flags = 1UL << IX_SGI_0;

	// TODO: use fix up section to avoid the conditional branch.
	asm volatile (
	"taddpci	$t0, 1f\n"
	"rseti		%0, 0\n"
	"rsetspr	$r7, "XEN_NUM"\n"
	"sprsetr	%0, "XEN_NUM"\n"
	"rseti		%1, 1\n"
	"4:stw		%1, %7\n"
	"or		%3, $r7, %3\n"
	".align 5\n"
	"dctl_lock_l1	%5\n"
	"ldl		%0, %2\n"
	"addi		%1, %0, 1\n"
	"stw		%1, %2\n"
	"dctl_unlock_l1	%5\n"
	"shri		%1, %0, %6\n"
	"jcw_eq		$t0, %0, %1\n" // If we have the lock, skip the loop
	"taddpci	$t0, 2f\n"
	"2:\n"
#ifdef CONFIG_GPT_SIMULATOR_ERRTA1
	"taddpci	$t0, 3f\n"
#endif
	"sprsetr	%3, "XEN_NUM"\n"
	// The idle instruction must immediately follow irq enable instruction
	// which also performs a barrier.
#ifdef CONFIG_GPT_SIMULATOR_ERRTA1
	"3:j		$t0\n"
	"taddpci	$t0, 2b\n"
#else
	"idle\n"
#endif
	"rseti		%1, 1\n"
	"stw		%1, %7\n"
	"rseti		%1, 0\n"
	"sprsetr	%1, "XEN_NUM"\n"
	"ldw		%1, %4\n"
	"jcw_ne		$t0, %0, %1\n"
	"1:\n"
	"sprsetr	$r7, "XEN_NUM"\n"
	"7:\n"
	".section .fixup,\"ax\"\n"                       
	".align 5\n"                                    
	"5:\n"
	"dctl_lock_l1	%5\n"
	"ldw		%0, %2\n"
	"addi		%1, %0, 1\n"
	"stw		%1, %2\n"
	"dctl_unlock_l1	%5\n"
	"sprsetr	$r7, "XEN_NUM"\n"
	"taddpci	$t0, 6f\n"
	"taddpcil	$t4, 7b\n"
	// --- cache alignment ---
	"6:\n"
	"ldw		%1, %4\n"
	"jcw_ne		$t0, %0, %1\n"
	"taddti		$t0, $t4, 7b\n"
	"j		$t0\n"
	".previous\n"                              
	".section __ex_table,\"a\"\n"             
	".align 3\n"                             
	".dword          4b,5b\n"               
	".previous\n"                              
	: "=&r"(oldval), "=&r"(newval), "+m"(lock->next), "+&r"(flags)
	: "m"(lock->owner), "a"(lock_align), "i"(TICKET_SHIFT), "a"(mpic_clear)
        : "memory", "$t0", "$t4", "$r7");
//	BUG_ON(lock->owner != lock->unlock);
	//lock->cpu = smp_processor_id();
	//lock->cpu1 = smp_processor_id();
}

static inline int arch_spin_trylock(arch_spinlock_t *lock)
{
	unsigned int unlocked;
	unsigned long lock_align = DCACHE_ALIGNED(lock);
	unsigned long oldval, newval;
	unsigned long flags;
	local_irq_save(flags);
	asm volatile (
	".align 5\n"
    	"dctl_lock_l1	%4\n"
	"ldl		%1, %3\n"
	"shri		%2, %1, %5\n"
	"rc_w_eq	%0, %1, %2\n"
	"add		%2, %1, %0\n"
	"stw		%2, %3\n"
	"dctl_unlock_l1	%4\n"
        : "=r"(unlocked), "=r"(oldval), "=r"(newval), "+m"(*lock)
        : "a"(lock_align), "i"(TICKET_SHIFT)
        : "memory", "$CA");
	local_irq_restore(flags);

        return unlocked;
}

static inline void arch_spin_unlock(arch_spinlock_t *lock)
{
	volatile unsigned int *mpic_set = ((unsigned int *)gpt_mpic_sgi_set) + ((sprget_gpt(THID) >> 8) & 0x3);
	unsigned long flags;
	local_irq_save(flags);

	//lock->unlock = lock->owner + 1;
	//lock->unlock1 = lock->owner + 1;
//	BUG_ON(lock->cpu != smp_processor_id());
	asm volatile (
	".align 5\n"
	"fence\n"
	"stw		%2, %0\n"
	"fence\n"
	"1:\n"
	"stw		%3, %1\n"
	"2:\n"
	".section __ex_table,\"a\"\n"             
	".align 3\n"                             
	".dword          1b,2b\n"
	".previous\n"  
        : "=m" (lock->owner), "=m"(*mpic_set)
        : "r"(lock->owner+1), "r"((0xF-(1<<smp_processor_id())) << 24)
        : "memory");
	local_irq_restore(flags);
}

static inline int arch_spin_value_unlocked(arch_spinlock_t lock)
{
        return lock.owner == lock.next;
}

static inline int arch_spin_is_locked(arch_spinlock_t *lock)
{
        return !arch_spin_value_unlocked(ACCESS_ONCE(*lock));
}

static inline int arch_spin_is_contended(arch_spinlock_t *lock)
{
        arch_spinlock_t lockval = ACCESS_ONCE(*lock);
        return (lockval.next - lockval.owner) > 1;
}

/*
 * Read-write spinlocks, allowing multiple readers but only one writer.
 *
 * The spinlock itself is contained in @counter and access to it is
 * serialized with @lock_mutex.
 *
 * Unfair locking as Writers could be starved indefinitely by Reader(s)
 */

/* Would read_trylock() succeed? */
#define arch_read_can_lock(x)   ((x)->lock < 0x80000000)

/* Would write_trylock() succeed? */
#define arch_write_can_lock(x)  ((x)->lock == 0)

/* 1 - lock taken successfully */
static inline int arch_read_trylock(arch_rwlock_t *rwlock)
{
	arch_rwlock_t lockval;
        int ret = 0;
	unsigned long lock_align = DCACHE_ALIGNED(rwlock);
	unsigned long flags;
	local_irq_save(flags);
	asm volatile (
	".align 5\n"
    	"dctl_lock_l1	%3\n"
	"ldw		%0, %2\n"
	"rc_w_s_ge	%1, %0, %1\n"
	"add		%0, %0, %1\n"
	"stw		%0, %2\n"
	"dctl_unlock_l1  %3\n"
        : "=&r"(lockval), "+&r"(ret), "+m"(rwlock->lock)
	: "a"(lock_align)
        : "memory", "$CA");
	local_irq_restore(flags);

        return ret;
}

static inline void arch_read_lock(arch_rwlock_t *rwlock)
{
	arch_rwlock_t lockval;
	unsigned long lock_align = DCACHE_ALIGNED(rwlock);
	unsigned long flags;
	unsigned long zero = 0;
	asm volatile (
    	"taddpci	$t0, 1f\n"	
	"rsetspr	%2, "XEN_NUM"\n"
	".align 5\n"
	"1:		   \n"
	"sprsetr	%3, "XEN_NUM"\n"
    	"dctl_lock_l1	%4\n"
	"ldw		%0, %1\n"
	"rc_w_s_ge	%3, %0, %3\n"
	"add		%0, %0, %3\n"
	"stw		%0, %1\n"
	"dctl_unlock_l1 %4\n"
	"sprsetr	%2, "XEN_NUM"\n"	
	"jci_eq		$t0, %3, 0\n"
        : "=&r"(lockval), "+m"(rwlock->lock), "=&r"(flags), "+&r"(zero)
	: "a"(lock_align)
        : "memory", "$t0", "$CA");

}

static inline void arch_read_unlock(arch_rwlock_t *rwlock)
{
	arch_rwlock_t lockval;
	unsigned long lock_align = DCACHE_ALIGNED(rwlock);
	unsigned long flags;
	local_irq_save(flags);

	asm volatile (
	".align 5\n"
    	"dctl_lock_l1    %2\n"
	"ldw             %0, %1\n"
	"addi		 %0, %0, -1\n"
	"stw		 %0, %1\n"
	"dctl_unlock_l1  %2\n"
        : "=r"(lockval), "+m"(rwlock->lock)
	: "a"(lock_align)
        : "memory");
	local_irq_restore(flags);
}

static inline int arch_write_trylock(arch_rwlock_t *rwlock)
{
	arch_rwlock_t lockval;
        int ret = 0;
	unsigned long lock_align = DCACHE_ALIGNED(rwlock);
	unsigned long flags;
	local_irq_save(flags);

	asm volatile (
	".align 5\n"
    	"dctl_lock_l1	%3\n"
	"ldw		%0, %2\n"
	"rc_w_eq	%1, %0, %1\n"
	"sel		%0, %4, %0\n"
	"stw		%0, %2\n"
	"dctl_unlock_l1 %3\n"
        : "=&r"(lockval), "+&r"(ret), "+m"(rwlock->lock)
	: "a"(lock_align), "r"(0x80000000)
	: "memory", "$CA");
	local_irq_restore(flags);
        return ret;
}

static inline void arch_write_lock(arch_rwlock_t *rwlock)
{
	arch_rwlock_t lockval;
	unsigned long lock_align = DCACHE_ALIGNED(rwlock);
	unsigned long flags;
	unsigned long zero = 0;

	asm volatile (
    	"taddpci	$t0, 1f\n"	
	"rsetspr	%2, "XEN_NUM"\n"
	".align 5\n"
	"1:		   \n"
	"sprsetr	%3, "XEN_NUM"\n"
    	"dctl_lock_l1	%4\n"
	"ldw		%0, %1\n"
	"rc_w_eq	%3, %0, %3\n"
	"sel		%0, %5, %0\n"
	"stw		%0, %1\n"
	"dctl_unlock_l1 %4\n"
	"sprsetr	%2, "XEN_NUM"\n"
	"jci_eq		$t0, %3, 0\n"
        : "=&r"(lockval), "+m"(rwlock->lock), "=&r"(flags), "+&r"(zero)
	: "a"(lock_align), "r"(0x80000000)
	: "memory", "$t0", "$CA");
}

static inline void arch_write_unlock(arch_rwlock_t *rwlock)
{
	asm volatile (
	"fence\n"
	"stw		 %1, %0\n"
        : "=m"(rwlock->lock)
        : "r"(0)
        : "memory");
}

#define arch_read_lock_flags(lock, flags)       arch_read_lock(lock)
#define arch_write_lock_flags(lock, flags)      arch_write_lock(lock)

#define arch_spin_relax(lock)   cpu_relax()
#define arch_read_relax(lock)   cpu_relax()
#define arch_write_relax(lock)  cpu_relax()

#endif /* __ASM_GPT_SPINLOCK_H */
