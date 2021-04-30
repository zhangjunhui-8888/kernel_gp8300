/*
 * VDSO implementation for GPT
 *
 * Copyright (C) 2019 GPT Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <linux/kernel.h>
#include <linux/clocksource.h>
#include <linux/elf.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/gfp.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/signal.h>
#include <linux/slab.h>
#include <linux/timekeeper_internal.h>
#include <linux/vmalloc.h>

#include <asm/cacheflush.h>
#include <asm/signal.h>
#include <asm/vdso.h>
#include <asm/vdso_datapage.h>

extern char vdso_start, vdso_end;
static unsigned long vdso_pages;
static struct page **vdso_pagelist;

/*
 * The vDSO data page.
 */
static union {
	struct vdso_data	data;
	u8			page[PAGE_SIZE];
} vdso_data_store __page_aligned_data;
struct vdso_data *vdso_data = &vdso_data_store.data;

static struct vm_special_mapping vdso_spec[2];

static int __init vdso_init(void)
{
	int i;

	if (memcmp(&vdso_start, "\177ELF", 4)) {
		pr_err("vDSO is not a valid ELF object!\n");
		return -EINVAL;
	}

	vdso_pages = (&vdso_end - &vdso_start) >> PAGE_SHIFT;
	pr_info("vdso: %ld pages (%ld code @ %p, %ld data @ %p)\n",
		vdso_pages + 1, vdso_pages, &vdso_start, 1L, vdso_data);

	/* Allocate the vDSO pagelist, plus a page for the data. */
	vdso_pagelist = kcalloc(vdso_pages + 1, sizeof(struct page *),
				GFP_KERNEL);
	if (vdso_pagelist == NULL)
		return -ENOMEM;

	/* Grab the vDSO data page. */
	vdso_pagelist[0] = virt_to_page(vdso_data);

	/* Grab the vDSO code pages. */
	for (i = 0; i < vdso_pages; i++)
		vdso_pagelist[i + 1] = virt_to_page(&vdso_start + i * PAGE_SIZE);

	/* Populate the special mapping structures */
	vdso_spec[0] = (struct vm_special_mapping) {
		.name	= "[vvar]",
		.pages	= vdso_pagelist,
	};

	vdso_spec[1] = (struct vm_special_mapping) {
		.name	= "[vdso]",
		.pages	= &vdso_pagelist[1],
	};

	return 0;
}
arch_initcall(vdso_init);

int arch_setup_additional_pages(struct linux_binprm *bprm,
				int uses_interp)
{
	struct mm_struct *mm = current->mm;
	unsigned long vdso_base, vdso_text_len, vdso_mapping_len;
	void *ret;

	vdso_text_len = vdso_pages << PAGE_SHIFT;
	/* Be sure to map the data page */
	vdso_mapping_len = vdso_text_len + PAGE_SIZE;

	down_write(&mm->mmap_sem);
	vdso_base = get_unmapped_area(NULL, 0, vdso_mapping_len, 0, 0);
	if (IS_ERR_VALUE(vdso_base)) {
		ret = ERR_PTR(vdso_base);
		goto up_fail;
	}
	ret = _install_special_mapping(mm, vdso_base, PAGE_SIZE,
				       VM_READ|VM_MAYREAD,
				       &vdso_spec[0]);
	if (IS_ERR(ret))
		goto up_fail;

	vdso_base += PAGE_SIZE;
	mm->context.vdso_base = vdso_base;
	ret = _install_special_mapping(mm, vdso_base, vdso_text_len,
				       VM_READ|VM_EXEC|
				       VM_MAYREAD|VM_MAYWRITE|VM_MAYEXEC,
				       &vdso_spec[1]);
	if (IS_ERR(ret))
		goto up_fail;


	up_write(&mm->mmap_sem);
	return 0;

up_fail:
	printk("func=%s---------up_fail\n",__func__);
	mm->context.vdso_base = 0UL;
	up_write(&mm->mmap_sem);
	return PTR_ERR(ret);
}

/*
 * Update the vDSO data page to keep in sync with kernel timekeeping.
 */
void update_vsyscall(struct timekeeper *tk)
{
	struct timespec xtime_coarse;
	u32 use_syscall = strcmp(tk->tkr.clock->name, "arch_sys_counter");

	++vdso_data->tb_seq_count;
	smp_wmb();

	xtime_coarse = __current_kernel_time();
	vdso_data->use_syscall			= use_syscall;
	vdso_data->xtime_coarse_sec		= xtime_coarse.tv_sec;
	vdso_data->xtime_coarse_nsec		= xtime_coarse.tv_nsec;
	vdso_data->wtm_clock_sec		= tk->wall_to_monotonic.tv_sec;
	vdso_data->wtm_clock_nsec		= tk->wall_to_monotonic.tv_nsec;

	if (!use_syscall) {
		vdso_data->cs_cycle_last	= tk->tkr.cycle_last;
		vdso_data->xtime_clock_sec	= tk->xtime_sec;
		vdso_data->xtime_clock_nsec	= tk->tkr.xtime_nsec;
		vdso_data->cs_mult		= tk->tkr.mult;
		vdso_data->cs_shift		= tk->tkr.shift;
	}

	smp_wmb();
	++vdso_data->tb_seq_count;
}

void update_vsyscall_tz(void)
{
	vdso_data->tz_minuteswest	= sys_tz.tz_minuteswest;
	vdso_data->tz_dsttime		= sys_tz.tz_dsttime;
}
