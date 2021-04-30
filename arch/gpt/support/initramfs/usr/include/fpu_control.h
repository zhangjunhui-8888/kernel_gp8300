/* Copyright (C) 1996-2015 Free Software Foundation, Inc.

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

#ifndef _GPTX_FPU_CONTROL_H
#define _GPTX_FPU_CONTROL_H

/* Macros for accessing the FPCR and FPSR.  */

#define _FPU_GETCW(fpcr) \
  __asm__ __volatile__ ("rsetfc	%0" : "=r" (fpcr))

#define _FPU_SETCW(fpcr) \
  __asm__ __volatile__ ("fcsetr	%0" : : "r" (fpcr))

#define _FPU_GETFPSR(fpsr) \
  __asm__ __volatile__ ("rsetfs	%0" : "=r" (fpsr))

#define _FPU_SETFPSR(fpsr) \
  __asm__ __volatile__ ("fssetr	%0" : : "r" (fpsr))

/* Reserved bits should be preserved when modifying register
   contents. These two masks indicate which bits in each of FPCR and
   FPSR should not be changed.  */

#define _FPU_RESERVED		(~0x0000031fULL)
#define _FPU_FPSR_RESERVED	(~0x0000001fULL)

#define _FPU_DEFAULT		0x00000000
#define _FPU_FPSR_DEFAULT	0x00000000

/* Layout of FPCR and FPSR:
   Bit 0: IOE Invalid Operation exception enable
   Bit 1: DZE Divide by Zero exception enable
   Bit 2: OFE Overflow exception enable
   Bit 3: UFE Underflow exception enable
   Bit 4: IXE Inexact exception enable
   Bit 8,9: RMC Rounding mode control. Only in FPCR   */

#define _FPU_FPCR_RM_MASK  0x0300

#define _FPU_FPCR_MASK_IXE 0x0010
#define _FPU_FPCR_MASK_UFE 0x0008
#define _FPU_FPCR_MASK_OFE 0x0004
#define _FPU_FPCR_MASK_DZE 0x0002
#define _FPU_FPCR_MASK_IOE 0x0001

#define _FPU_FPCR_IEEE                       \
  (_FPU_DEFAULT  | _FPU_FPCR_MASK_IXE |	     \
   _FPU_FPCR_MASK_UFE | _FPU_FPCR_MASK_OFE | \
   _FPU_FPCR_MASK_DZE | _FPU_FPCR_MASK_IOE)

#define _FPU_FPSR_IEEE 0

typedef unsigned long fpu_control_t;
typedef unsigned long fpu_fpsr_t;

/* Default control word set at startup.  */
extern fpu_control_t __fpu_control;

#endif
