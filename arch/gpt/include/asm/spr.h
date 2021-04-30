/*
 * arch/gpt/include/asm/spr.h
 *
 * Related to gpt processor's SPR registers.
 *
 * Copyright (C) 2015, Optimum Semiconductor Technologies
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

#ifndef __ASM_GPT_SPR_H
#define __ASM_GPT_SPR_H

/* List of SPR values. */
enum {
#define SPR__(a,b)      a=b,
#include <machine/gpt_spr.h>
#undef SPR__
};

#ifndef __ASSEMBLY__
/*
 * asm inline code
 */
#define sprset_gpt(_spr, _val)						\
	({								\
		__asm__ __volatile__ ("sprsetr %0, %1"			\
				      :					\
				      : "r" (_val), "i" (_spr)		\
				      : "memory");			\
	})

#define sprget_gpt(_spr)                                                \
	({								\
		unsigned long _val;					\
		__asm__ __volatile__ ("rsetspr %0, %1"			\
				      : "=r" (_val)			\
				      : "i" (_spr)			\
				      : "memory");			\
		_val;							\
	})

#define get_psc()	sprget_gpt(PSC)

#endif /* __ASSEMBLY__ */
#endif /* __ASM_GPT_SPR_H */

