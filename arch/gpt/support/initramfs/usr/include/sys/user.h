/* Copyright (C) 2009-2015 Free Software Foundation, Inc.

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

#ifndef _SYS_USER_H
#define _SYS_USER_H	1

struct user_regs_struct
{
  unsigned long r_regs[16];
  // unsigned long t_regs[8];  /* Now only $t0/1/4/7 are used. */
  unsigned long ra;	/* There is no callee saved $t regs except for $t7. */
  unsigned long pc;
  unsigned long ca;         /* It is used to save carry_flag*/
  unsigned long sp;
  unsigned long a_regs[16];
};

struct user_fpu_struct
{
  unsigned long fpsr;
  unsigned long fpcr;
  unsigned long f_regs[16];
};

struct user_vpu_struct
{
  unsigned long  vfsr;
  unsigned long  vfcr;
  unsigned short n_regs[8];
  unsigned long  m_regs[32];
  unsigned long  v_regs[1024];
};

#endif
