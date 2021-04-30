/*
 * gpt Linux
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
 *
 * DMA mapping callbacks...
 * As alloc_coherent is the only DMA callback being used currently, that's
 * the only thing implemented properly.  The rest need looking into...
 */
#include <linux/dma-mapping.h>
#include <linux/dma-debug.h>
#include <linux/export.h>
#include <linux/dma-attrs.h>
#include <linux/dma-contiguous.h>
#include <linux/highmem.h>
#include <linux/genalloc.h>
#include <linux/vmalloc.h>

#include <asm/cache.h>
#include <asm/cacheflush.h>
#include <linux/sizes.h>

#ifndef CONFIG_NO_DMA
static struct gen_pool *atomic_pool;
#define DEFAULT_DMA_COHERENT_POOL_SIZE  SZ_256K
static size_t atomic_pool_size = DEFAULT_DMA_COHERENT_POOL_SIZE;

static int __init early_coherent_pool(char *p)
{
        atomic_pool_size = memparse(p, &p);
        return 0;
}
early_param("coherent_pool", early_coherent_pool);

static void * gpt_alloc_from_pool(size_t size, struct page **ret_page, gfp_t flags)
{
	unsigned long val;
	void *ptr = NULL;

	if (!atomic_pool) {
		WARN(1, "coherent pool not initialised!\n");
		return NULL;
	}

	val = gen_pool_alloc(atomic_pool, size);
	if (val) {
		phys_addr_t phys = gen_pool_virt_to_phys(atomic_pool, val);
		*ret_page = phys_to_page(phys);
		ptr = (void *)val;
	}

	return ptr;
}

static bool __in_atomic_pool(void *start, size_t size)
{
	return addr_in_gen_pool(atomic_pool, (unsigned long)start, size);
}

static int gpt_free_from_pool(void *start, size_t size)
{
	if (!__in_atomic_pool(start, size))
		return 0;

	gen_pool_free(atomic_pool, (unsigned long)start, size);

	return 1;
}

/*
 * Alloc "coherent" memory, which here means simply uncached.
 *
 * This function effectively just calls __get_free_pages, sets the
 * cache-inhibit bit on those pages, and makes sure that the pages are
 * flushed out of the cache before they are used.
 *
 */

static void *gpt_dma_alloc(struct device *dev, size_t size,
			dma_addr_t *dma_handle, gfp_t gfp,
			struct dma_attrs *attrs)
{
	struct page *page = NULL;
	void *ptr, *coherent_ptr;

	if (dma_alloc_from_coherent(dev, size, dma_handle, &ptr))
		return ptr;

	size = PAGE_ALIGN(size);

	if (!(gfp & __GFP_WAIT)) {
		ptr = gpt_alloc_from_pool(size, &page, gfp);
		if (ptr)
			*dma_handle = phys_to_dma(dev, page_to_phys(page));
		return ptr;
		
	}else{
		if (IS_ENABLED(CONFIG_DMA_CMA)) 
			page = dma_alloc_from_contiguous(dev, size >> PAGE_SHIFT,
							 get_order(size));

	}
	if (!page)
		page = alloc_pages(gfp|GFP_DMA, get_order(size));

	if (!page)
		return NULL;

	/* dma addr */
	*dma_handle = phys_to_dma(dev, page_to_phys(page));
	ptr = page_address(page);
	if (!ptr)
		goto no_mem;

	memset(ptr, 0, size);
	__dma_sync_cache_range((unsigned long)ptr, (unsigned long)ptr+size);
	
	page = virt_to_page(ptr);
	coherent_ptr = dma_common_contiguous_remap(page, size, VM_USERMAP, 
		pgprot_writecombine(PAGE_KERNEL_NC), NULL);
	if (!coherent_ptr)
		goto no_map;

	return coherent_ptr;

no_map:
	dma_release_from_contiguous(dev, page, size >> PAGE_SHIFT);
no_mem:
	*dma_handle = DMA_ERROR_CODE;
	return NULL;
}

static void
gpt_dma_free(struct device *dev, size_t size, void *vaddr,
              dma_addr_t dma_handle, struct dma_attrs *attrs)
{
        phys_addr_t paddr = dma_to_phys(dev, dma_handle);

        if (dev == NULL) {
                WARN_ONCE(1, "Use an actual device structure for DMA allocation\n");
                return;
        }

        if (dma_release_from_coherent(dev, get_order(size), vaddr))
                return;

	if (gpt_free_from_pool(vaddr, size))
		return;

	vunmap(vaddr);

        if(!dma_release_from_contiguous(dev, phys_to_page(paddr),
						size >> PAGE_SHIFT))
		__free_pages(phys_to_page(paddr), get_order(size));
}

static inline void dma_sync(void *vaddr, size_t size, enum dma_data_direction dir)
{
	unsigned long start = (unsigned long)vaddr & (~(L2_CACHE_BYTES - 1));
	 unsigned long end = (unsigned long)vaddr+size;

	switch (dir) {
		case DMA_TO_DEVICE:
			/* Flush the dcache for the requested range */
			__dma_sync_cache_range(start, end);
			break;
		case DMA_BIDIRECTIONAL:
			/* Flush the dcache for the requested range */
			__dma_sync_cache_range(start, end);
			break;
		case DMA_FROM_DEVICE:
			__dma_sync_cache_range(start, end);
			break;
		default:
			BUG();
			break;
	}
}

static dma_addr_t
gpt_map_page(struct device *dev, struct page *page,
              unsigned long offset, size_t size,
              enum dma_data_direction dir,
              struct dma_attrs *attrs)
{
	void *vaddr = page_address(page) + offset;
	dma_sync(vaddr, size, dir);
	return phys_to_dma(dev, virt_to_phys(vaddr));
}

static void
gpt_unmap_page(struct device *dev, dma_addr_t dma_handle,
                size_t size, enum dma_data_direction dir,
                struct dma_attrs *attrs)
{
	dma_sync(phys_to_virt(dma_to_phys(dev, dma_handle)), size, dir);
}

static int
gpt_map_sg(struct device *dev, struct scatterlist *sg,
            int nents, enum dma_data_direction dir,
            struct dma_attrs *attrs)
{
        struct scatterlist *s;
        int i;

        for_each_sg(sg, s, nents, i) {
                s->dma_address = gpt_map_page(dev, sg_page(s), s->offset,
                                               s->length, dir, NULL);
        }

        return nents;
}

static void
gpt_unmap_sg(struct device *dev, struct scatterlist *sg,
              int nents, enum dma_data_direction dir,
              struct dma_attrs *attrs)
{
        struct scatterlist *s;
        int i;

        for_each_sg(sg, s, nents, i) {
                gpt_unmap_page(dev, sg_dma_address(s), sg_dma_len(s), dir, NULL);
        }
}

static void
gpt_sync_single_for_cpu(struct device *dev,
                         dma_addr_t dma_handle, size_t size,
                         enum dma_data_direction dir)
{
	void* ptr = phys_to_virt(dma_to_phys(dev, dma_handle));
	dma_sync(ptr, size, dir);
}

static void
gpt_sync_single_for_device(struct device *dev,
                            dma_addr_t dma_handle, size_t size,
                            enum dma_data_direction dir)
{
	void* ptr = phys_to_virt(dma_to_phys(dev, dma_handle));
	dma_sync(ptr, size, dir);
}

static int gpt_dma_mmap(struct device *dev,
		struct vm_area_struct *vma,
		void *cpu_addr, dma_addr_t dma_addr, size_t size,
		struct dma_attrs *attrs)
{
	int ret = -ENXIO;
	unsigned long nr_vma_pages = (vma->vm_end - vma->vm_start) >> PAGE_SHIFT;
	unsigned long nr_pages = PAGE_ALIGN(size) >> PAGE_SHIFT;
	unsigned long pfn = dma_to_phys(dev, dma_addr) >> PAGE_SHIFT;
	unsigned long off = vma->vm_pgoff;

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	if (dma_mmap_from_coherent(dev, vma, cpu_addr, size, &ret))
		return ret;

	if (off < nr_pages && nr_vma_pages <= (nr_pages - off)) {
		ret = remap_pfn_range(vma, vma->vm_start,
				      pfn + off,
				      vma->vm_end - vma->vm_start,
				      vma->vm_page_prot);
	}

	return ret;
}

struct dma_map_ops gpt_dma_map_ops = {
        .alloc = gpt_dma_alloc,
        .free = gpt_dma_free,
        .mmap = gpt_dma_mmap,
        .map_page = gpt_map_page,
        .unmap_page = gpt_unmap_page,
        .map_sg = gpt_map_sg,
        .unmap_sg = gpt_unmap_sg,
        .sync_single_for_cpu = gpt_sync_single_for_cpu,
        .sync_single_for_device = gpt_sync_single_for_device,
};
EXPORT_SYMBOL(gpt_dma_map_ops);

static int __init atomic_pool_init(void)
{
	pgprot_t prot = PAGE_KERNEL_NC;
	unsigned long nr_pages = atomic_pool_size >> PAGE_SHIFT;
	struct page *page;
	void *addr;
	unsigned int pool_size_order = get_order(atomic_pool_size);

	if (dev_get_cma_area(NULL))
		page = dma_alloc_from_contiguous(NULL, nr_pages,
							pool_size_order);
	else
		page = alloc_pages(GFP_DMA, pool_size_order);

	if (page) {
		int ret;

		atomic_pool = gen_pool_create(PAGE_SHIFT, -1);
		if (!atomic_pool)
			goto free_page;

		addr = dma_common_contiguous_remap(page, atomic_pool_size,
					VM_USERMAP, prot, atomic_pool_init);

		if (!addr)
			goto destroy_genpool;

		ret = gen_pool_add_virt(atomic_pool, (unsigned long)addr,
					page_to_phys(page),
					atomic_pool_size, -1);
		if (ret)
			goto remove_mapping;

		gen_pool_set_algo(atomic_pool,
				  gen_pool_first_fit_order_align,
				  (void *)PAGE_SHIFT);

		pr_info("DMA: preallocated %zu KiB pool for atomic allocations\n",
			atomic_pool_size / 1024);
		return 0;
	}
	goto out;

remove_mapping:
	dma_common_free_remap(addr, atomic_pool_size, VM_USERMAP);
destroy_genpool:
	gen_pool_destroy(atomic_pool);
	atomic_pool = NULL;
free_page:
	if (!dma_release_from_contiguous(NULL, page, nr_pages))
		__free_pages(page, pool_size_order);
out:
	pr_err("DMA: failed to allocate %zu KiB pool for atomic coherent allocation\n",
		atomic_pool_size / 1024);
	return -ENOMEM;
}



/* Number of entries preallocated for DMA-API debugging */
#define PREALLOC_DMA_DEBUG_ENTRIES (1 << 16)

static int __init dma_init(void)
{
	int ret = 0;
        dma_debug_init(PREALLOC_DMA_DEBUG_ENTRIES);

	ret |= atomic_pool_init();

        return ret;
}
fs_initcall(dma_init);



#endif
