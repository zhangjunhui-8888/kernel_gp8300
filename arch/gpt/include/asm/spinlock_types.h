/*
 * arch/gpt/include/asm/spinlock_types.h
 *
 * Spinlocks and r/w locks data structures.
 *
 * Copyright (C) 2015-2016, Optimum Semiconductor Technologies
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
 *  * Copyright (C) 2004, 2007-2010, 2011-2012 Synopsys, Inc. (www.synopsys.com)
 *   *
 *    * This program is free software; you can redistribute it and/or modify
 *     * it under the terms of the GNU General Public License version 2 as
 *      * published by the Free Software Foundation.
 *       */

#ifndef __ASM_SPINLOCK_TYPES_H
#define __ASM_SPINLOCK_TYPES_H

#ifndef __LINUX_SPINLOCK_TYPES_H
# error "please don't include this file directly"
#endif

#define TICKET_SHIFT 32
typedef struct {
      u32 next;
      u32 owner;
} __aligned(8) arch_spinlock_t;

#define __ARCH_SPIN_LOCK_UNLOCKED       { 0} //, 0, 0 }

typedef struct {
        volatile unsigned int lock;
} arch_rwlock_t;

#define __ARCH_RW_LOCK_UNLOCKED         { 0 }

#endif
