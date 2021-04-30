/*
 *
 * Linux architectural port borrowing liberally from similar works of
 * others.  All original copyrights apply as per the original source
 * declaration.
 *
 * Modifications for the gpt architecture:
 *
 *      This program is free software; you can redistribute it and/or
 *      modify it under the terms of the GNU General Public License
 *      as published by the Free Software Foundation; either version
 *      2 of the License, or (at your option) any later version.
 */

#include <linux/linkage.h>
#include <asm/errno.h>
#include <asm-generic/posix_types.h>
#include <asm/uaccess.h>

/*
 * unsigned long clear_user(void *addr, unsigned long size) ;
 *
 * NOTE: it returns number of bytes NOT cleared !!!
 */
unsigned long __clear_user(void *addr, unsigned long size)
{
	unsigned char *dst = (unsigned char *)addr;
	unsigned long n = size;
	int retval = 0;

	if (n > 0 && ((unsigned long) dst & 1)) {
		__put_user_size(0, dst, 1, retval);
		if(retval)
			return n;

		dst++;
		n--;
	}
	if (n >= 2 && ((unsigned long) dst & 2)) {
		__put_user_size(0, dst, 2, retval);
		if(retval)
			return n;

		dst += 2;
		n -= 2;
	}
	if (n >= 4 && ((unsigned long) dst & 4)) {
		__put_user_size(0, dst, 4, retval);
		if(retval)
			return n;

		dst += 4;
		n -= 4;
	}
	/* 64 bit copy loop */
	if (!((__force unsigned long) dst & 7)) {
		while (n >= 8) {
			__put_user_size(0, dst, 8, retval);
			if(retval)
				return n;
			dst += 8;
			n -= 8;
		}
	}

	while (n >= 4) {
		__put_user_size(0, dst, 4, retval);
		if(retval)
			break;
		
		dst += 4;
		n -= 4;
	}

	while (n >= 2) {
		__put_user_size(0, dst, 2, retval);
		if(retval)
			break;

		dst += 2;
		n -= 2;
	}

	while (n >= 1) {
		__put_user_size(0, dst, 1, retval);
		if(retval)
			break;

		dst++;
		n--;
	}

	return n;
}

