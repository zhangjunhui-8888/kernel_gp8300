/*
 * This file is taken from arm64.
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
 */
#include <linux/bitops.h>
#include <linux/compiler.h>
#include <linux/kernel.h>
#include <linux/smp.h>
#include <linux/stop_machine.h>
#include <linux/uaccess.h>

#include <asm/cacheflush.h>
#include <asm/insn.h>

/*
 * The instructions in GPT Unity instruction set have a fixed length of 32 bits and are always
 * little-endian.
 */
int __kprobes gpt_insn_read(void *addr, u32 *insnp, int size)
{
	int ret;

	if (((uintptr_t)addr & 0x3) || (size & 0x3))
		return -EINVAL;

	ret = probe_kernel_read(insnp, addr, size);

	return ret;
}

int __kprobes gpt_insn_write(void *addr, u32 *insnp, int size)
{
	if (((uintptr_t)addr & 0x3) || (size & 0x3))
		return -EINVAL;

	return probe_kernel_write(addr, insnp, size);
}

u32* __kprobes gpt_insn_gen_branch(unsigned long pc, unsigned long addr, u32 insn[3])
{	
	long upper, lower;
	/* 
	 *	taddpcil	$t4, high
	 *	taddti		$t0, $t4, low
	 *	call		$t0
	 */
	BUG_ON((pc & 0x3) || (addr & 0x3));
		
	upper = (long)(addr>>14) - (long)(pc>>14);
	lower = (addr & (BIT(14)-1))>>2;
	
	insn[0] = insn_gen1_imm(GPTXISA_OPCODE_TADDPCIL, 4, upper); 
	insn[1] = insn_gen2_imm(GPTXISA_OPCODE_TADDTI, 0, 4, lower); 
	insn[2] = insn_gen1t(GPTXISA_OPCODE_CALL,0); 	
	
	return insn;
}

u32 __kprobes gpt_insn_gen_nop(void)
{
	return insn_gen0(GPTXISA_OPCODE_NOP);
}
u32 __kprobes gpt_insn_gen_call(void)
{
	return insn_gen1t(GPTXISA_OPCODE_CALL,0);
}
