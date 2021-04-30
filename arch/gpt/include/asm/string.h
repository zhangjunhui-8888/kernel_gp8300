#ifndef __ASM_GPT_STRING_H
#define __ASM_GPT_STRING_H

/*
 * We don't do inline string functions, since the
 * optimised inline asm versions are not small.
 */

#define __HAVE_ARCH_MEMCPY
extern void * memcpy(void *, const void *, __kernel_size_t);
#define __HAVE_ARCH_MEMSET
extern void *memset(void *, int, __kernel_size_t);
#define __HAVE_ARCH_MEMCMP
extern int memcmp(const void *cs, const void *ct, size_t count);

#define __HAVE_ARCH_STRLEN
extern size_t strlen(const char *str);
#define __HAVE_ARCH_STRNLEN
extern size_t strnlen(const char *s, size_t count);

#define __HAVE_ARCH_STRRCHR
extern char *strrchr(const char *s, int c);

#define __HAVE_ARCH_STRCMP
extern int strcmp(const char *s1, const char *s2);

#endif
