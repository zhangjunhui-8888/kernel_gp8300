/*
 * Copyright (C) 2018, General Processor Techologies Inc.
 * scxie <scxie@hxgpt.com>
 *
 * It is based on demo code taken from
 * http://www.cppblog.com/ant/archive/2007/10/12/32886.html
 *
 * This software has been developed by Intel Corporation.
 * Intel specifically disclaims all warranties, express or
 * implied, and all liability, including consequential and
 * other indirect damages, for the use of this program, including
 * liability for infringement of any proprietary rights,
 * and including the warranties of merchantability and fitness
 * for a particular purpose. Intel does not assume any
 * responsibility for and errors which may appear in this program
 * not any responsibility to update it.
 */

#include <linux/types.h>

size_t strlen(const char *str)
{ 
	const char *char_ptr;
	const unsigned long *longword_ptr;
	register unsigned long longword, himagic, lomagic;

	for (char_ptr = str; ((unsigned long)char_ptr 
	& (sizeof(unsigned long) - 1)) != 0;
	++char_ptr) {
		if (*char_ptr == '\0')
			return char_ptr - str;
	}

	longword_ptr = (unsigned long*)char_ptr;

	himagic = 0x8080808080808080L;
	lomagic = 0x0101010101010101L;

	while (1)
	{
		longword = *longword_ptr++;
/*
 *	Since a ascii code is in 0-127, so longword is:
 *         b7       b6	             ......	       b1        b0
 *	63----------------------------------------------------------->0
 *	0XXXXXXX  0XXXXXXX           ......         0XXXXXXX   0XXXXXXX
 *	So if longword - lomagic, the highest bit of each byte should 1;
 */		
		if (((longword - lomagic) & himagic) != 0) {
			int i;
			const char *cp ;
			cp = (const char*)(longword_ptr - 1);
			for(i=0;i<sizeof(unsigned long);i++)
				if (cp[i] == '\0')
					return cp - str + i;
		}
	}
}
