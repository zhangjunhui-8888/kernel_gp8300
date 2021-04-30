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


#define arch_spin_lock_flags(lock, flags)       arch_spin_lock(lock)
#define arch_spin_unlock_wait(x) \
        do { while (arch_spin_is_locked(x)) cpu_relax(); } while (0)
#if 1
#define SPIN_LOCK_DISABLE_IRQ "rsetspr $r8, 16\nandi $r7, $r8, -9\nsprsetr $r7, 16\n"
#define SPIN_LOCK_ENABLE_IRQ "sprsetr	$r8, 16\n"
#define SPIN_LOCK_IRQ_SCRATCH , "$r7","$r8"
//#define SPIN_LOCK_DISABLE_IRQ "rseti	$r7, -9\nsprsetr  $r7, 16\nrseti  $r7, -1\n"
//#define SPIN_LOCK_ENABLE_IRQ "sprsetr	$r7, 16\n"
//#define SPIN_LOCK_IRQ_SCRATCH , "$r7"
#else
#define SPIN_LOCK_DISABLE_IRQ
#define SPIN_LOCK_DISABLE_IRQ
#define SPIN_LOCK_IRQ_SCRATCH
#endif
#define TEST0 (1)
#define TEST1 (1)
#define TEST2 (1)
static inline void arch_spin_lock(arch_spinlock_t *lock)
{
	volatile unsigned long lock_align = DCACHE_ALIGNED(lock);
	u32 oldval, newval;

	if(lock->owner > lock->next)
		printk("func=%s-----------owner=%x, next=%x\n", __func__, lock->owner, lock->next);
		

	smp_mb();
	asm volatile (
	SPIN_LOCK_DISABLE_IRQ
	"fence\n"
	"barrier\n"
	"taddpci	$t0, 1f\n"
	".align 5\n"
	"dctl_lock_l1	%4\n"
	"ldw		%0, %2\n"
	"addi		%1, %0, 1\n"
	"stw		%1, %2\n"
	"fence\n"
	"barrier\n"
	"dctl_unlock_l1	%4\n"
	"fence\n"
	"barrier\n"
	SPIN_LOCK_ENABLE_IRQ
	"fence\n"
	"barrier\n"
	"1:\n"
	SPIN_LOCK_DISABLE_IRQ
	"fence\n"
	"barrier\n"
	".align 5\n"
	"dctl_lock_l1	%4\n"
	"ldw		%1, %3\n"
	"stw		%1, %3\n"
	"fence\n"
	"barrier\n"
	"dctl_unlock_l1	%4\n"
	"fence\n"
	"barrier\n"
	SPIN_LOCK_ENABLE_IRQ
	"jcw_ne		$t0, %0, %1\n"
	: "=&r"(oldval), "=&r"(newval), "+m"(ACCESS_ONCE(lock->next)), "+m"(ACCESS_ONCE(lock->owner)), "+a"(lock_align)
	:
        : "memory", "$t0"SPIN_LOCK_IRQ_SCRATCH);
	smp_mb();

#if (TEST0)
	if(ACCESS_ONCE(lock->owner_c)){
		printk("func=%s----------lock owner_c=%x, cur_cpu=%d\n", __func__, ACCESS_ONCE(lock->owner_c), smp_processor_id());
	//	BUG_ON(1);
	}
	lock->owner_c|=1<<smp_processor_id();
#endif
#if (TEST1)
	if(ACCESS_ONCE(lock->owner) != ACCESS_ONCE(lock->unlock) || ACCESS_ONCE(lock->unlock) != ACCESS_ONCE(lock->unlock1)){
		printk("func=%s----getlocked-------lock=%p, lock->next=%x,=%p,  lock->owner=%x, lock->unlock=%x,=%p, lock->unlock1=%x, lock->cpu=%x, lock->cpu1=%x, cur_cpu=%d, last_cpu=%x\n", __func__, lock, ACCESS_ONCE(lock->next), &lock->next, ACCESS_ONCE(lock->owner), ACCESS_ONCE(lock->unlock), &lock->unlock, ACCESS_ONCE(lock->unlock1), ACCESS_ONCE(lock->cpu), ACCESS_ONCE(lock->cpu1), smp_processor_id(), ACCESS_ONCE(lock->last_cpu));
	}
#endif
//	BUG_ON(lock->owner != lock->unlock);
#if (TEST2)
	smp_mb();
	ACCESS_ONCE(lock->cpu1)=ACCESS_ONCE(lock->cpu) = smp_processor_id();
#endif
}

static inline int arch_spin_trylock(arch_spinlock_t *lock)
{
	unsigned int unlocked;
	volatile unsigned long lock_align = DCACHE_ALIGNED(lock);
	u64 oldval, newval;
	
	smp_mb();
	asm volatile (
	SPIN_LOCK_DISABLE_IRQ
	"fence\n"
	"barrier\n"
	".align 5\n"
    	"dctl_lock_l1	%4\n"
	"ldl		%1, %3\n"
	"shri		%2, %1, %5\n"
	"rc_w_eq	%0, %1, %2\n"
	"add		%2, %1, %0\n"
	"stw		%2, %3\n"
	"barrier\n"
	"dctl_unlock_l1	%4\n"
	"fence\n"
	"barrier\n"
	SPIN_LOCK_ENABLE_IRQ
        : "=&r"(unlocked), "=&r"(oldval), "=&r"(newval), "+m"(ACCESS_ONCE(lock->next)), "+a"(lock_align)
        : "i"(TICKET_SHIFT)
        : "memory", "$CA"SPIN_LOCK_IRQ_SCRATCH);
	smp_mb();
#if 1
	if(unlocked){
#if (TEST0)
		if(lock->owner_c){
			printk("func=%s----------lock owner_c=%x, cur_cpu=%d\n", __func__, ACCESS_ONCE(lock->owner_c), smp_processor_id());
	//		BUG_ON(1);
		}
		lock->owner_c|=1<<smp_processor_id();
#endif
#if (TEST1)
		if(ACCESS_ONCE(lock->owner) != ACCESS_ONCE(lock->unlock)|| ACCESS_ONCE(lock->unlock) != ACCESS_ONCE(lock->unlock1))
			printk("func=%s--------lock=%p, lock->next=%x,  lock->owner=%x, lock->unlock=%x, lock->unlock1=%x, lock->cpu=%x, cur_cpu=%d\n", __func__, lock, lock->next, lock->owner, lock->unlock, lock->unlock1, lock->cpu, smp_processor_id());
#endif
//		BUG_ON(lock->owner != lock->unlock);
#if (TEST2)
		smp_mb();
		ACCESS_ONCE(lock->cpu1)=ACCESS_ONCE(lock->cpu) = smp_processor_id();
#endif
		smp_mb();
	}
#endif
        return unlocked;
}
static inline void arch_spin_unlock(arch_spinlock_t *lock)
{
	volatile unsigned long lock_align = DCACHE_ALIGNED(lock);
	u32 owner;

#if (TEST0)
	if(lock->owner_c &= ~(1<<smp_processor_id()))
		printk("func=%s---22-----lock=%p, lock->next=%x,  lock->owner=%x, lock->cpu=%d, lock->cpu1=%d, cur_cpu=%d, owner_c=%x\n", __func__, lock, lock->next, lock->owner, lock->cpu, lock->cpu1, smp_processor_id(), lock->owner_c);
#endif

#if (TEST1)
	ACCESS_ONCE(lock->unlock1) = ACCESS_ONCE(lock->owner) + 1;
	ACCESS_ONCE(lock->unlock) = ACCESS_ONCE(lock->unlock1);
	smp_mb();
#endif

#if (TEST2)
	if(ACCESS_ONCE(lock->cpu) != smp_processor_id()/* && (unsigned long)lock == DCACHE_ALIGNED(lock)*/  /*|| (lock->owner_c != 1<<smp_processor_id())  || lock->unlock != lock->owner+1 || lock->unlock != lock->unlock1*/){
		printk("func=%s---11-----lock=%p, lock->next=%x,  lock->owner=%x, lock->cpu=0x%x, lock->cpu1=0x%x, cur_cpu=0x%x, owner_c=0x%x, last_cpu=0x%x\n", __func__, lock, lock->next, lock->owner, ACCESS_ONCE(lock->cpu), ACCESS_ONCE(lock->cpu1), smp_processor_id(), ACCESS_ONCE(lock->owner_c), ACCESS_ONCE(lock->last_cpu));
	//	dump_stack();
	}
	ACCESS_ONCE(lock->cpu) = ACCESS_ONCE(lock->cpu1) =smp_processor_id()<<4 ;
	ACCESS_ONCE(lock->last_cpu)=smp_processor_id()<<4;
	if(ACCESS_ONCE(lock->unlock1) != ACCESS_ONCE(lock->unlock))
		printk("func=%s-----unlock=%x, unlock1=%x", __func__, lock->unlock, lock->unlock1);
#endif
	
//	BUG_ON(lock->cpu != smp_processor_id());
	smp_mb();
        asm volatile(
	SPIN_LOCK_DISABLE_IRQ
	"fence\n"
	"barrier\n"
	".align 5\n"
	"dctl_lock_l1	%2\n"
	"ldw		%1, %0\n"
	"addi		%1, %1, 1\n"
	"stw		%1, %0\n"
	"fence\n"
	"barrier\n"
	"dctl_unlock_l1	%2\n"
	"fence\n"
	"barrier\n"
	SPIN_LOCK_ENABLE_IRQ
        : "+m" (ACCESS_ONCE(lock->owner)),"=&r" (owner), "+a"(lock_align)
	:
        : "memory" SPIN_LOCK_IRQ_SCRATCH);
	smp_mb();
}

static inline int arch_spin_value_unlocked(arch_spinlock_t lock)
{
        return lock.owner == lock.next;
}

static inline int arch_spin_is_locked(arch_spinlock_t *lock)
{
#if 0
	unsigned int unlocked;
	volatile unsigned long lock_align = DCACHE_ALIGNED(lock);
	u64 oldval, newval;
	
	smp_mb();
	asm volatile (
	SPIN_LOCK_DISABLE_IRQ
	"fence\n"
	"barrier\n"
	".align 5\n"
    	"dctl_lock_l1	%4\n"
	"ldl		%1, %3\n"
	"shri		%2, %1, %5\n"
	"rc_w_eq	%0, %1, %2\n"
	"fence\n"
	"barrier\n"
	"dctl_unlock_l1	%4\n"
	"fence\n"
	"barrier\n"
	SPIN_LOCK_ENABLE_IRQ
        : "=&r"(unlocked), "=&r"(oldval), "=&r"(newval), "+m"(ACCESS_ONCE(lock->next)), "+a"(lock_align)
        : "i"(TICKET_SHIFT)
        : "memory", "$CA"SPIN_LOCK_IRQ_SCRATCH);
	smp_mb();

	return !unlocked;
#else
        return !arch_spin_value_unlocked(ACCESS_ONCE(*lock));
#endif
}

static inline int arch_spin_is_contended(arch_spinlock_t *lock)
{
#if 0
	volatile unsigned long lock_align = DCACHE_ALIGNED(lock);
        arch_spinlock_t lockval;
	
	smp_mb();
	asm volatile (
	SPIN_LOCK_DISABLE_IRQ
	"fence\n"
	"barrier\n"
	".align 5\n"
    	"dctl_lock_l1	%2\n"
	"ldl		%0, %1\n"
	"fence\n"
	"barrier\n"
	"dctl_unlock_l1	%2\n"
	"fence\n"
	"barrier\n"
	SPIN_LOCK_ENABLE_IRQ
        : "=&r"(lockval), "+m"(ACCESS_ONCE(lock->next)), "+a"(lock_align)
        : 
        : "memory" SPIN_LOCK_IRQ_SCRATCH);
	smp_mb();
#else
        arch_spinlock_t lockval = ACCESS_ONCE(*lock);
#endif
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
	volatile unsigned long lock_align = DCACHE_ALIGNED(rwlock);

	smp_mb();
	asm volatile (
	SPIN_LOCK_DISABLE_IRQ
	"fence\n"
	"barrier\n"
	".align 5\n"
    	"dctl_lock_l1	%3\n"
	"ldw		%0, %2\n"
	"rc_w_s_ge	%1, %0, %1\n"
	"add		%0, %0, %1\n"
	"stw		%0, %2\n"
	"fence\n"
	"barrier\n"
	"dctl_unlock_l1  %3\n"
	"fence\n"
	"barrier\n"
	SPIN_LOCK_ENABLE_IRQ
        : "=&r"(lockval), "+&r"(ret), "+m"(ACCESS_ONCE(rwlock->lock)), "+a"(lock_align)
	:
        : "memory", "$CA" SPIN_LOCK_IRQ_SCRATCH);
	smp_mb();

        return ret;
}

static inline void arch_read_lock(arch_rwlock_t *rwlock)
{
	arch_rwlock_t lockval;
	volatile unsigned long lock_align = DCACHE_ALIGNED(rwlock);
	unsigned long zero = 0;
	
	smp_mb();
	asm volatile (
    	"taddpci	$t0, 1f\n"	
	"1:		   \n"
	SPIN_LOCK_DISABLE_IRQ
	"fence\n"
	"barrier\n"
	".align 5\n"
    	"dctl_lock_l1	%2\n"
	"ldw		%0, %1\n"
	"rc_w_s_ge	%3, %0, %3\n"
	"add		%0, %0, %3\n"
	"stw		%0, %1\n"
	"fence\n"
	"barrier\n" 
	"dctl_unlock_l1 %2\n"
	"fence\n"
	"barrier\n"
	SPIN_LOCK_ENABLE_IRQ
    	"jci_eq		$t0, %3, 0\n"
        : "=&r"(lockval), "+m"(ACCESS_ONCE(rwlock->lock)), "+a"(lock_align), "+&r"(zero)
	:
        : "memory", "$t0", "$CA" SPIN_LOCK_IRQ_SCRATCH);
	smp_mb();

}

static inline void arch_read_unlock(arch_rwlock_t *rwlock)
{
	arch_rwlock_t lockval;
	volatile unsigned long lock_align = DCACHE_ALIGNED(rwlock);

	smp_mb();
	asm volatile (
	SPIN_LOCK_DISABLE_IRQ
	"fence\n"
	"barrier\n"
	".align 5\n"
    	"dctl_lock_l1    %2\n"
	"ldw             %0, %1\n"
	"addi		 %0, %0, -1\n"
	"stw		 %0, %1\n"
	"fence\n"
	"barrier\n" 
	"dctl_unlock_l1  %2\n"
	"fence\n"
	"barrier\n"
	SPIN_LOCK_ENABLE_IRQ
        : "=&r"(lockval), "+m"(ACCESS_ONCE(rwlock->lock)), "+a"(lock_align)
	:
        : "memory" SPIN_LOCK_IRQ_SCRATCH);
	smp_mb();
}

/* 1 - lock taken successfully */
static inline int arch_write_trylock(arch_rwlock_t *rwlock)
{
	arch_rwlock_t lockval;
        int ret = 0;
	volatile unsigned long lock_align = DCACHE_ALIGNED(rwlock);
	smp_mb();
	asm volatile (
	SPIN_LOCK_DISABLE_IRQ
	"fence\n"
	"barrier\n"
	".align 5\n"
    	"dctl_lock_l1	%3\n"
	"ldw		%0, %2\n"
	"rc_w_eq	%1, %0, %1\n"
	"sel		%0, %4, %0\n"
	"stw		%0, %2\n"
	"fence\n"
	"barrier\n"
	"dctl_unlock_l1 %3\n"
	"fence\n"
	"barrier\n"
	SPIN_LOCK_ENABLE_IRQ
        : "=&r"(lockval), "+&r"(ret), "+m"(ACCESS_ONCE(rwlock->lock)), "+a"(lock_align)
	: "r"(0x80000000)
	: "memory", "$CA" SPIN_LOCK_IRQ_SCRATCH);
	smp_mb();
        return ret;
}
static inline void arch_write_lock(arch_rwlock_t *rwlock)
{
	arch_rwlock_t lockval;
	volatile unsigned long lock_align = DCACHE_ALIGNED(rwlock);
	unsigned long zero = 0;

	smp_mb();
	asm volatile (
    	"taddpci	$t0, 1f\n"		
	"1:"
	SPIN_LOCK_DISABLE_IRQ
	"fence\n"
	"barrier\n"
	".align 5\n"
    	"dctl_lock_l1	%2\n"
	"ldw		%0, %1\n"
	"rc_w_eq	%3, %0, %3\n"
	"sel		%0, %4, %0\n"
	"stw		%0, %1\n"
	"fence\n"
	"barrier\n"
	"dctl_unlock_l1 %2\n"
	"fence\n"
	"barrier\n"
	SPIN_LOCK_ENABLE_IRQ
	"jci_eq		$t0, %3, 0\n"
        : "=&r"(lockval), "+m"(ACCESS_ONCE(rwlock->lock)),"+a"(lock_align), "+&r"(zero)
	: "r"(0x80000000)
	: "memory", "$t0", "$CA" SPIN_LOCK_IRQ_SCRATCH);
	smp_mb();
}

static inline void arch_write_unlock(arch_rwlock_t *rwlock)
{
	volatile unsigned long lock_align = DCACHE_ALIGNED(rwlock);
	unsigned int tmp=0;

	smp_mb();
        asm volatile(
	SPIN_LOCK_DISABLE_IRQ
	"fence\n"
	"barrier\n"
	".align 5\n"
	"dctl_lock_l1	%2\n"
	"ldw		%1, %0\n"
	"rseti		%1, 0\n"
	"stw		%1, %0\n"
	"fence\n"
	"barrier\n" 
	"dctl_unlock_l1	%2\n"
	"fence\n"
	"barrier\n"
	SPIN_LOCK_ENABLE_IRQ
        : "+m" (ACCESS_ONCE(rwlock->lock)),"+&r" (tmp), "+a"(lock_align)
	:
        : "memory" SPIN_LOCK_IRQ_SCRATCH);
	smp_mb();

}

#define arch_read_lock_flags(lock, flags)       arch_read_lock(lock)
#define arch_write_lock_flags(lock, flags)      arch_write_lock(lock)

#define arch_spin_relax(lock)   cpu_relax()
#define arch_read_relax(lock)   cpu_relax()
#define arch_write_relax(lock)  cpu_relax()

#endif /* __ASM_GPT_SPINLOCK_H */
