/* Copyright (C) 2005-2015 Free Software Foundation, Inc.

   This file is part of the GNU C Library.

   The GNU C Library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public License as
   published by the Free Software Foundation; either version 2.1 of the
   License, or (at your option) any later version.

   The GNU C Library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with the GNU C Library; if not, see
   <http://www.gnu.org/licenses/>.  */

#ifndef	_LINK_H
# error "Never include <bits/link.h> directly; use <link.h> instead."
#endif

/* Registers for entry into PLT on GPTX.  */
typedef struct La_gptx_regs
{
  uint64_t lr_greg[8];   //$r8 ~ $r15
  uint64_t lr_freg[8];   //$f8 ~ $f15
  uint64_t lr_sp;        //$asp
  uint64_t lr_ra;        //$t7
} La_gptx_regs;

/* Return values for calls from PLT on GPTX.  */
typedef struct La_gptx_retval
{
  /* Up to two integer registers can be used for a return value.  */
  uint64_t lrv_greg[2];  //$r8 ~ $r9
  /* Up to four D registers can be used for a return value.  */
  uint64_t lrv_freg[4];  //$f8 ~ $f11
} La_gptx_retval;
__BEGIN_DECLS

extern ElfW(Addr)
la_gptx_gnu_pltenter (ElfW(Sym) *__sym, unsigned int __ndx,
			 uintptr_t *__refcook,
			 uintptr_t *__defcook,
			 La_gptx_regs *__regs,
			 unsigned int *__flags,
			 const char *__symname,
			 long int *__framesizep);

extern unsigned int
la_gptx_gnu_pltexit (ElfW(Sym) *__sym, unsigned int __ndx,
			uintptr_t *__refcook,
			uintptr_t *__defcook,
			const La_gptx_regs *__inregs,
			La_gptx_retval *__outregs,
			const char *__symname);

__END_DECLS
