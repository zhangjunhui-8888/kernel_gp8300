/*
 * linux/arch/gpt/boot/compressed/misc.c
 */

#include <asm/unaligned.h>
#include <asm/uncompress.h>

/*
 * gzip delarations
 */
unsigned char *output_data;
unsigned long long output_ptr;

unsigned long long free_mem_ptr;
unsigned long long  free_mem_end_ptr;

#define STATIC static
#define STATIC_RW_DATA	/* non-static please */
#define GPT_IMAGE_MAX_SIZE    0x10000000
/*
 * arch-dependent implementations
 */
#ifndef ARCH_HAVE_DECOMP_ERROR
#define arch_decomp_error(x)
#endif

#ifndef ARCH_HAVE_DECOMP_SETUP
#define arch_decomp_setup()
#endif

#ifndef ARCH_HAVE_DECOMP_PUTS
#define arch_decomp_puts(p)
#endif

void *memcpy(void *dest, const void *src, size_t n)
{
	int i = 0;
	unsigned char *d = (unsigned char *)dest, *s = (unsigned char *)src;

	for (i = n >> 3; i > 0; i--) {
		*d++ = *s++;
		*d++ = *s++;
		*d++ = *s++;
		*d++ = *s++;
		*d++ = *s++;
		*d++ = *s++;
		*d++ = *s++;
		*d++ = *s++;
	}

	if (n & 1 << 2) {
		*d++ = *s++;
		*d++ = *s++;
		*d++ = *s++;
		*d++ = *s++;
	}

	if (n & 1 << 1) {
		*d++ = *s++;
		*d++ = *s++;
	}

	if (n & 1)
		*d++ = *s++;

	return dest;
}

void error(char *x)
{
	arch_decomp_puts("\n\n");
	arch_decomp_puts(x);
	arch_decomp_puts("\n\n -- System halted");

	arch_decomp_error(x);

	for (;;)
		; /* Halt */
}

void flush_dcache_range(unsigned long start, unsigned long end)
{
	asm volatile (
        "taddpci         $t1, 1f\n"
	"rseta		 $r8, %0\n"
	"andi		 $r8, $r8, %3\n"
	"aaddri		 %0, $r8, 0\n"
	"1:\n"
	"dctl_sync_l1    %0\n"
        "aaddai          %0, %0, %2\n"
        "jcau_lt         $t1, %0, %1\n"
	: "+a"(start)
	: "a"(end), "i"(DCACHE_LINE_BYTES), "i"(~(DCACHE_LINE_BYTES - 1))
	: "memory", "$t1");
}


/* Heap size should be adjusted for different decompress method */
#ifdef CONFIG_KERNEL_GZIP
#include "../../../../lib/decompress_inflate.c"
#endif

#ifdef CONFIG_KERNEL_BZIP2
#include "../../../../lib/decompress_bunzip2.c"
#endif

#ifdef CONFIG_KERNEL_LZO
#include "../../../../lib/decompress_unlzo.c"
#endif

#ifdef CONFIG_KERNEL_LZMA
#include "../../../../lib/decompress_unlzma.c"
#endif

unsigned long decompress_kernel(unsigned long long output_start,
		unsigned long long free_mem_ptr_p,
		unsigned long long free_mem_ptr_end_p)
{
	unsigned char *tmp;

	output_data		= (unsigned char *)output_start;
	free_mem_ptr		= free_mem_ptr_p;
	free_mem_end_ptr	= free_mem_ptr_end_p;

	tmp = (unsigned char *) (((unsigned long)input_data_end) - 4);

	decompress(input_data, input_data_end - input_data, NULL, NULL,
			output_data, NULL, error);

	flush_dcache_range(output_start, output_start+GPT_IMAGE_MAX_SIZE);

	return output_ptr;
}
