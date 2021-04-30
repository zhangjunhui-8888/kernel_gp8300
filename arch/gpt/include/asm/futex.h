#ifndef _ASM_FUTEX_H
#define _ASM_FUTEX_H

#include <linux/futex.h>
#include <linux/uaccess.h>
#include <asm/atomic.h>
#include <asm/errno.h>

#ifdef CONFIG_SMP
#define __futex_atomic_op(insn, ret, oldval, uaddr, tmp, oparg, uaddr_align)		\
	__asm__ __volatile__(		                                       \
"	.align    5\n"                                                         \
"1:	dctl_lock_l1 %5\n"                                                     \
" 	ldw	%1, %2\n"						       \
 	".if " insn "== addi\n"								\
	insn"   %3, %4, 0\n"							       \
	".else\n"						       \
	insn"   %3, %4, %1\n"							       \
	".endif\n"									\
"	stw	%3, %2\n"						       \
"	dctl_unlock_l1 %5\n"						       \
"	barrier\n"							       \
"3:\n" 									       \
"	.section .fixup,\"ax\"\n"			\
"	.align 3\n"					\
"4:	rseti		%0, %6\n"			\
"	taddpcil	$t4, 3b\n"			\
"	taddti		$t1, $t4, 3b\n"			\
"	j		$t1\n"				\
"	.previous\n"                                   	\
"	.section __ex_table,\"a\"\n"                   	\
"       .align 3\n"                             	\
"       .dword		1b,4b\n"                       	\
"	.previous\n"                                     	\
	: "+r"(ret), "=&r" (oldval), "+m" (*uaddr), "=r" (tmp)			\
	: "r" (oparg), "a"(uaddr_align),"i" (-EFAULT) 			        \
	: "memory", "$t1", "$t4")						

static inline int
futex_atomic_op_inuser (int encoded_op, u32 __user *uaddr)
{
	int op = (encoded_op >> 28) & 7;
	int cmp = (encoded_op >> 24) & 15;
	int oparg = (encoded_op << 8) >> 20;
	int cmparg = (encoded_op << 20) >> 20;
	int oldval, ret = 0, tmp;
	unsigned long  uaddr_align;
	unsigned long flags;


	if (encoded_op & (FUTEX_OP_OPARG_SHIFT << 28))
		oparg = 1 << oparg;

	if (!access_ok(VERIFY_WRITE, uaddr, sizeof(u32)))
		return -EFAULT;

	pagefault_disable();	/* implies preempt_disable() */

	uaddr_align = DCACHE_ALIGNED(uaddr);
        uaddr_align |= (unsigned long )uaddr & 0x3;
	raw_local_irq_save(flags);
	switch (op) {
	case FUTEX_OP_SET:
		__futex_atomic_op("addi",
				 ret, oldval, uaddr, tmp, oparg, uaddr_align);
		break;
	case FUTEX_OP_ADD:
		__futex_atomic_op("add",
				 ret, oldval, uaddr, tmp, oparg, uaddr_align);
		break;
	case FUTEX_OP_OR:
		__futex_atomic_op("or",
				  ret, oldval, uaddr, tmp, oparg, uaddr_align);
		break;
	case FUTEX_OP_ANDN:
		__futex_atomic_op("cand",
				  ret,  oldval, uaddr, tmp, oparg, uaddr_align);
		break;
	case FUTEX_OP_XOR:
		__futex_atomic_op("xor",
				  ret, oldval, uaddr, tmp, oparg, uaddr_align);
		break;
	default:
		ret = -ENOSYS;
	}
	raw_local_irq_restore(flags);

	pagefault_enable();	/* subsumes preempt_enable() */

	if (!ret) {
		switch (cmp) {
		case FUTEX_OP_CMP_EQ: ret = (oldval == cmparg); break;
		case FUTEX_OP_CMP_NE: ret = (oldval != cmparg); break;
		case FUTEX_OP_CMP_LT: ret = (oldval < cmparg); break;
		case FUTEX_OP_CMP_GE: ret = (oldval >= cmparg); break;
		case FUTEX_OP_CMP_LE: ret = (oldval <= cmparg); break;
		case FUTEX_OP_CMP_GT: ret = (oldval > cmparg); break;
		default: ret = -ENOSYS;
		}
	}
	return ret;
}




static inline int
futex_atomic_cmpxchg_inatomic(u32 *uval, u32 __user *uaddr,
			      u32 oldval, u32 newval)
{
	int ret = 0;
	u32 val;
	unsigned long uaddr_align;
	unsigned long flags;

	if (!access_ok(VERIFY_WRITE, uaddr, sizeof(u32)))
		return -EFAULT;

	uaddr_align = DCACHE_ALIGNED(uaddr);
        uaddr_align |= (unsigned long )uaddr & 0x3;
	raw_local_irq_save(flags);
	asm volatile(
"	taddpci         $t1, 3f\n"
"	.align          5\n"
"1:	dctl_lock_l1    %5\n"
"	ldw             %1, %2\n"
"	jcw_ne          $t1, %1, %3\n"
"	stw             %4, %2\n"
"3:\n"
"	dctl_unlock_l1  %5\n"
"	barrier\n"
"4:\n"
"	.section .fixup,\"ax\"\n"
"	.align 3\n"
"5:	rseti		%0, %6\n"
"	taddpcil	$t4, 4b\n"
"	taddti		$t1, $t4, 4b\n"
"	j		$t1\n"
"	.previous\n"
"	.section __ex_table,\"a\"\n"
"       .align 3\n"
"       .dword		1b,5b\n"
"	.previous\n"
	: "+r" (ret), "=&r" (val), "+m" (*uaddr)
	: "r" (oldval), "r" (newval), "a"(uaddr_align), "i"(-EFAULT)
	: "memory", "$t1", "$t4");

	raw_local_irq_restore(flags);
	*uval = val;
	return ret;
}
#else

/* This is a temporary solution for single core, since we have no spinlock yet.
 * TODO: support spinlock on SMP
 */
static inline void
_futex_spin_lock_irqsave(u32 __user *uaddr, unsigned long int *flags)
{
	local_irq_save(*flags);
}

static inline void
_futex_spin_unlock_irqrestore(u32 __user *uaddr, unsigned long int *flags)
{
	local_irq_restore(*flags);
}

static inline int
futex_atomic_op_inuser (int encoded_op, u32 __user *uaddr)
{
	unsigned long int flags;
	u32 val;
	int op = (encoded_op >> 28) & 7;
	int cmp = (encoded_op >> 24) & 15;
	int oparg = (encoded_op << 8) >> 20;
	int cmparg = (encoded_op << 20) >> 20;
	int oldval = 0, ret;
	if (encoded_op & (FUTEX_OP_OPARG_SHIFT << 28))
		oparg = 1 << oparg;

	if (!access_ok(VERIFY_WRITE, uaddr, sizeof(*uaddr)))
		return -EFAULT;

	pagefault_disable();

	_futex_spin_lock_irqsave(uaddr, &flags);

	switch (op) {
	case FUTEX_OP_SET:
		/* *(int *)UADDR2 = OPARG; */
		ret = get_user(oldval, uaddr);
		if (!ret)
			ret = put_user(oparg, uaddr);
		break;
	case FUTEX_OP_ADD:
		/* *(int *)UADDR2 += OPARG; */
		ret = get_user(oldval, uaddr);
		if (!ret) {
			val = oldval + oparg;
			ret = put_user(val, uaddr);
		}
		break;
	case FUTEX_OP_OR:
		/* *(int *)UADDR2 |= OPARG; */
		ret = get_user(oldval, uaddr);
		if (!ret) {
			val = oldval | oparg;
			ret = put_user(val, uaddr);
		}
		break;
	case FUTEX_OP_ANDN:
		/* *(int *)UADDR2 &= ~OPARG; */
		ret = get_user(oldval, uaddr);
		if (!ret) {
			val = oldval & ~oparg;
			ret = put_user(val, uaddr);
		}
		break;
	case FUTEX_OP_XOR:
		/* *(int *)UADDR2 ^= OPARG; */
		ret = get_user(oldval, uaddr);
		if (!ret) {
			val = oldval ^ oparg;
			ret = put_user(val, uaddr);
		}
		break;
	default:
		ret = -ENOSYS;
	}

	_futex_spin_unlock_irqrestore(uaddr, &flags);

	pagefault_enable();

	if (!ret) {
		switch (cmp) {
		case FUTEX_OP_CMP_EQ: ret = (oldval == cmparg); break;
		case FUTEX_OP_CMP_NE: ret = (oldval != cmparg); break;
		case FUTEX_OP_CMP_LT: ret = (oldval < cmparg); break;
		case FUTEX_OP_CMP_GE: ret = (oldval >= cmparg); break;
		case FUTEX_OP_CMP_LE: ret = (oldval <= cmparg); break;
		case FUTEX_OP_CMP_GT: ret = (oldval > cmparg); break;
		default: ret = -ENOSYS;
		}
	}
	return ret;
}

/* Non-atomic version */
static inline int
futex_atomic_cmpxchg_inatomic(u32 *uval, u32 __user *uaddr,
			      u32 oldval, u32 newval)
{
	int ret;
	u32 val;
	unsigned long flags;

	if (!access_ok(VERIFY_WRITE, uaddr, sizeof(u32)))
		return -EFAULT;

	_futex_spin_lock_irqsave(uaddr, &flags);

	ret = get_user(val, uaddr);

	if (!ret && val == oldval)
		ret = put_user(newval, uaddr);

	*uval = val;

	_futex_spin_unlock_irqrestore(uaddr, &flags);

	return ret;
}
#endif
#endif /*_ASM_FUTEX_H*/
