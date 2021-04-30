/* Define the machine-dependent type `jmp_buf'.  GPTX version.
   Copyright (C) 1992-2015 Free Software Foundation, Inc.
   This file is part of the GNU C Library.

   The GNU C Library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   The GNU C Library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with the GNU C Library.  If not, see
   <http://www.gnu.org/licenses/>.  */

#ifndef _GPTX_BITS_SETJMP_H
#define _GPTX_BITS_SETJMP_H 1

#if !defined(_SETJMP_H) && !defined(_PTHREAD_H)
# error "Never include <bits/setjmp.h> directly; use <setjmp.h> instead."
#endif

typedef struct __jmp_buf_internal_tag
  {
    /* Program counter  */
    unsigned long __pc;

    /* Callee-saved general registers $r0 ~ $r5 */
    unsigned long __gregs[6];

    /* Note: carry flag is not saved, because its use is only a few. */
    /* unsigned long carry_flag;  */

    /* Note: Floating point control and status register can not be saved. */
    /* unsigned long __fpcsr[2];  */

    /* Callee-saved floating point registers $f0 ~ $f7  */
    double __fpregs[8];

    /* Callee-saved address register $a3, $a8 - $a10*/
    unsigned long __aregs[4];

    /* Stack pointer $asp */
    unsigned long __sp;

    /* Frame pointer $a1 */
    unsigned long __fp;
  } __jmp_buf[1];

#endif /* _GPTX_BITS_SETJMP_H */
