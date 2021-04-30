/*
 * GPT specific _mcount support, refer to arm64
 *
 * Copyright (C) 2017 scxie <scxie@hxgpt.com>
 * 
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive for
 * more details.
 *
 */
#include <linux/linkage.h>
#include <asm/asm-defs.h>
#include <asm/ftrace.h>

/*
 * Gcc with -pg will put the following code in the beginning of each function:
 *      rsett $r8, $t7
 *	taddpci	$t0, _mcount
 *      call $t0
 *      [function's body ...]
 * "call _mcount" may be replaced to "call ftrace_caller" or NOP if dynamic
 * ftrace is enabled.
 * stack layout after mcount_enter in _mcount():
 *
 * current sp/fp =>  0:+-----+
 * in _mcount()        | $t7 | -> _mcount()'s lr (= instrumented function's pc)
 *                     +-----+
 *                     | $afp| -> instrumented function's fp
 * old sp       => +16:+-----+
 * when instrumented   |     |
 * function calls      | ... |
 * _mcount()           |     |
 *                     |     |
 * instrumented => +xx:+-----+
 * function's fp       | $afp| -> parent's fp
 *                     +-----+
 *                     | $t7 | -> instrumented function's lr (= parent's pc)
 *                     +-----+
 *                     | ... |
 */ 
 
        .macro mcount_enter
        aaddai  $asp, $asp, -8
        stut    $t7, $asp, -8
        sta	$afp, $asp
        aaddai  $afp, $asp, 0
        .endm

        .macro mcount_exit
        lda	$afp, $asp
        aaddai  $asp, $asp, 8
        ldut    $t7, $asp, 8
        ret
        .endm

        .macro mcount_adjust_addr rd, rn
        addi     \rd, \rn, -3*GPT_INSN_SIZE
        .endm
        
	/* for instrumented function */
	.macro mcount_get_pc0 rreg
	rsett	\rreg, $t7
	mcount_adjust_addr	\rreg, \rreg
	.endm	

        .macro mcount_get_lr areg, rreg
        lda	\areg, $afp
        aaddai	\areg, \areg, 8
        ldl	\rreg, \areg
        mcount_adjust_addr	\rreg, \rreg
        .endm
        
        .macro mcount_get_pc areg, rreg
        aaddai	\areg, $afp, 8
        ldl	\rreg, \areg
        mcount_adjust_addr	\rreg, \rreg
        .endm

        .macro mcount_get_lr_addr rreg
        ldl     \rreg, $afp
        addi    \rreg, \rreg, 8
        .endm

        /* for instrumented function's parent */
        .macro mcount_get_parent_fp areg, rreg
        lda     \areg, $afp
        ldl	\rreg, \areg
        .endm

#ifndef CONFIG_DYNAMIC_FTRACE
ENTRY(_mcount)
        mcount_enter
        ldapci	$a7, ftrace_trace_function
        lda	$a7, $a7
        ldapci	$a5, ftrace_stub
        taddpci	$t1, skip_ftrace_call		
        jca_eq	$t1, $a5, $a7			//	if (ftrace_trace_function!= ftrace_stub) {
        
        mcount_get_pc	$a6, $r8              	//       function's pc
        mcount_get_lr	$a6, $r9       		//       function's lr (= parent's pc)
        tseta	$t1, $a7			
	call	$t1				//   	(*ftrace_trace_function)(pc, lr);

skip_ftrace_call:	
#ifndef CONFIG_FUNCTION_GRAPH_TRACER
	ldapci	$a6, ftrace_graph_return
	ldtpci	$t1, ftrace_graph_caller	// if ((ftrace_graph_return
        jca_ne	$t1, $a5, $a6			//	!= ftrace_stub)
        
        ldapci	$a6, ftrace_graph_entry
        ldapci	$a5, ftrace_graph_entry_stub
        jca_ne	$t1, $a5, $a6
#endif /* CONFIG_FUNCTION_GRAPH_TRACER */ 
        mcount_exit       
ENDPROC(_mcount)	
#else	/* CONFIG_DYNAMIC_FTRACE */
ENTRY(_mcount)
	ret        
ENDPROC(_mcount)	

/*
 * void ftrace_caller(unsigned long return_address)
 * @return_address: return address to instrumented function
 *
 * This function is a counterpart of _mcount() in 'static' ftrace, and
 * makes calls to:
 *     - tracer function to probe instrumented function's entry,
 *     - ftrace_graph_caller to set up an exit hook
 */
ENTRY(ftrace_caller)
	mcount_enter

	mcount_get_pc0	$r8		// function's pc
	mcount_get_lr	$a7, $r9	// function's lr

	.global ftrace_call
ftrace_call:				// tracer(pc, lr);
/*
 * This will be replaced with 
 * "taddpcil $t4, xxx"
 * "taddti $t0, $t4, xxx"
 * "call $0"
 */
	nop				
	nop
	nop

#ifdef CONFIG_FUNCTION_GRAPH_TRACER
	.global ftrace_graph_call
ftrace_graph_call:			// ftrace_graph_caller();
	nop				// If enabled, this will be replaced "call ftrace_graph_caller"
	nop				
	nop
#endif

	mcount_exit
ENDPROC(ftrace_caller)

#endif	/* CONFIG_DYNAMIC_FTRACE */

ENTRY(ftrace_stub)
	ret
ENDPROC(ftrace_stub)

#ifdef CONFIG_FUNCTION_GRAPH_TRACER
        /* save return value regs*/
        .macro save_return_regs
        aaddai	$asp, $asp, -8
        stul	$r8,  $asp, -8
        stul	$r9,  $asp, -8
        stul	$r10, $asp, -8
        stul	$r11, $asp, -8
        stul	$r12, $asp, -8
        stul	$r13, $asp, -8
        stul	$r14, $asp, -8
        stl	$r15, $asp
        .endm

        /* restore return value regs*/
        .macro restore_return_regs
        ldul	$r15, $asp, 8
	ldul	$r14, $asp, 8
	ldul	$r13, $asp, 8
	ldul	$r12, $asp, 8
	ldul	$r11, $asp, 8
	ldul	$r10, $asp, 8
	ldul	$r9,  $asp, 8
	ldul	$r8,  $asp, 8
        .endm
/*
 * void ftrace_graph_caller(void)
 *
 * Called from _mcount() or ftrace_caller() when function_graph tracer is
 * selected.
 * This function w/ prepare_ftrace_return() fakes link register's value on
 * the call stack in order to intercept instrumented function's return path
 * and run return_to_handler() later on its exit.
 */
ENTRY(ftrace_graph_caller)
        mcount_get_lr_addr	     $r8    			// pointer to function's saved lr
        mcount_get_pc		$a7, $r9			// function's pc
        mcount_get_parent_fp	$a7, $r10   			// parent's fp
        cal			$t0, prepare_ftrace_return  	// prepare_ftrace_return(&lr, pc, fp)
        
        mcount_exit
ENDPROC(ftrace_graph_caller)

/*
 * void return_to_handler(void)
 *
 * Run ftrace_return_to_handler() before going back to parent.
 * @fp is checked against the value passed by ftrace_graph_caller()
 * only when CONFIG_HAVE_FUNCTION_GRAPH_FP_TEST is enabled.
 */
ENTRY(return_to_handler)
        save_return_regs
        rseta   $r8, $afp                	// parent's fp
        cal	$t0, ftrace_return_to_handler	// addr = ftrace_return_to_hander(fp);
        tsetr	$t7, $r8                 	// restore the original return address
        restore_return_regs
        ret
END(return_to_handler)
#endif /* CONFIG_FUNCTION_GRAPH_TRACER */
