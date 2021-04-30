/*
 * gpt vgettimeofday.c
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

#include <linux/time.h>
#include <linux/printk.h>
#include <asm/timex.h>
#include <asm/unistd.h>
#include <asm/vdso.h>

/*This mean kernel mode or user mode can read timer register */
#define	USER_CLOCK

extern struct vdso_data *__get_datapage(void);

struct syscall_return_value {
	long value;
	long error;
};

/*
 * Find out the vDSO data page address in the process address space.
 */
inline unsigned long get_datapage(void)
{
	return (unsigned long)__get_datapage();
}

static inline u64 vgetsns(struct vdso_data *vdso)
{
	register u64 cycles asm("$r8");
	u64 cycle_delta;
	u64 nsec = 0;

	asm volatile(
		"trap	8\n\t"
		:
		:
		:"memory", "$r8");

	cycle_delta = (cycles - vdso->cycle_last) & vdso->mask;

	nsec = (cycle_delta * vdso->mult) + vdso->wall_time_snsec;
	nsec >>= vdso->shift;

	return nsec;
}

static inline int do_realtime(struct vdso_data *vdso, struct timespec *ts)
{
	register unsigned count asm("$r15");
	u64 ns = 0;

	do {
		count = raw_read_seqcount(&vdso->tb_seq);
		ts->tv_sec = vdso->wall_time_sec;
		ns = vgetsns(vdso);
	} while(0);// (unlikely(read_seqcount_retry(&vdso->tb_seq, count)));

	ts->tv_sec += __iter_div_u64_rem(ns, NSEC_PER_SEC, &ns);
	ts->tv_nsec = ns;

	return 0;
}

static inline int do_monotonic(struct vdso_data *vdso, struct timespec *ts)
{
	struct timespec tomono;
	unsigned count;
	u64 ns;

	do {
		count = raw_read_seqcount_begin(&vdso->tb_seq);
		ts->tv_sec = vdso->wall_time_sec;
		ns = vgetsns(vdso);

		tomono.tv_sec = vdso->monotonic_time_sec;
		tomono.tv_nsec = vdso->monotonic_time_snsec;

	} while (unlikely(read_seqcount_retry(&vdso->tb_seq, count)));

	ts->tv_sec += tomono.tv_sec;

	ts->tv_sec += __iter_div_u64_rem(ns, NSEC_PER_SEC, &ns);
	ts->tv_nsec = ns;

	return 0;
}

static inline int do_realtime_coarse(struct vdso_data *vdso,
				     struct timespec *ts)
{
	unsigned count;

	do {
		count = raw_read_seqcount_begin(&vdso->tb_seq);
		ts->tv_sec = vdso->wall_time_coarse_sec;
		ts->tv_nsec = vdso->wall_time_coarse_nsec;
	} while (unlikely(read_seqcount_retry(&vdso->tb_seq, count)));

	return 0;
}

static inline int do_monotonic_coarse(struct vdso_data *vdso,
				      struct timespec *ts)
{
	struct timespec tomono;
	unsigned count;

	do {
		count = raw_read_seqcount_begin(&vdso->tb_seq);
		ts->tv_sec = vdso->monotonic_time_coarse_sec;
		ts->tv_nsec = vdso->monotonic_time_coarse_nsec;

		tomono.tv_sec = vdso->wall_time_sec;
		tomono.tv_nsec = vdso->wall_time_snsec;
	} while (unlikely(read_seqcount_retry(&vdso->tb_seq, count)));

	ts->tv_sec += tomono.tv_sec;
	ts->tv_sec += __iter_div_u64_rem(ts->tv_nsec, NSEC_PER_SEC, (u64 *)&tomono.tv_nsec);
	ts->tv_nsec = tomono.tv_nsec;

	return 0;
}

notrace int __kernel_gettimeofday(struct timeval *_tv,
						struct timezone *_tz)
{
	/* In current design architecture if want read TIM register MUST NEED Hypervisor Mode
	 * so in current when just use system call instead, the comment part is used for future */
#ifndef	USER_CLOCK
	register struct timezone *tz asm("$r8") = _tz;
	register struct timeval *tv asm("$r9") = _tv;
	register long ret asm("$r8");
	register long nr asm("$r7") = __NR_clock_gettime;

	asm volatile(
		"trap   4\n"
		: "=r" (ret)
		: "r" (tv), "r" (tz), "r" (nr)
		: "memory");

	return ret;
#else
	struct timespec ts;
	struct vdso_data *vdso = (struct vdso_data *)get_datapage();

	/* The use of the timezone is obsolete, normally tz is NULL. */
	do_realtime(vdso, &ts);
	if (_tv) {
		_tv->tv_sec = ts.tv_sec;
		_tv->tv_usec = ts.tv_nsec / 1000;
	}
	if (_tz) {
		_tz->tz_minuteswest = vdso->tz_minuteswest;
		_tz->tz_dsttime = vdso->tz_dsttime;
	}
	return 0;
#endif
}

static notrace long clock_gettime_fallback(clockid_t _clkid, struct timespec *_ts)
{
	register struct timespec *ts asm("$r9") = _ts;
	register clockid_t clkid asm("$r8") = _clkid;
	register long ret asm ("$r8");
	register long nr asm("$r7") = __NR_clock_gettime;

	asm volatile(
		"trap	4\n"
		: "=r" (ret)
		: "r" (clkid), "r" (ts), "r" (nr)
		: "memory");

	return ret;
}

notrace int __kernel_clock_gettime(clockid_t clock,
				struct timespec *ts)
{
	/* In current design architecture if want read TIM register MUST NEED Hypervisor Mode
	 * so in current when just use system call instead, the comment part is used for future */
	int ret = -1;
#ifndef	USER_CLOCK
	struct vdso_data *vdso = (struct vdso_data *)get_datapage();

	switch (clock) {
	case CLOCK_REALTIME:
		ret = do_realtime(vdso, ts);
		break;
	case CLOCK_MONOTONIC:
		ret = do_monotonic(vdso, ts);
		break;;
	case CLOCK_REALTIME_COARSE:
		ret = do_realtime_coarse(vdso, ts);
		break;
	case CLOCK_MONOTONIC_COARSE:
		ret = do_monotonic_coarse(vdso, ts);
		break;
	default:
		ret = clock_gettime_fallback(clock, ts);
		break;
	}
#else

	ret = clock_gettime_fallback(clock, ts);
#endif
	return ret;
}
