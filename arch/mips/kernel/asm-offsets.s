	.file	1 "asm-offsets.c"
	.section .mdebug.abi32
	.previous

 # -G value = 0, Arch = mips32, ISA = 32
 # GNU C version 4.1.2 (mipsel-linux)
 #	compiled by GNU C version 3.4.2 20041017 (Red Hat 3.4.2-6.fc3).
 # GGC heuristics: --param ggc-min-expand=100 --param ggc-min-heapsize=131072
 # options passed:  -nostdinc -Iinclude
 # -I/home/ros/rg300_abdulwaz/arch/mips/include
 # -I/home/ros/rg300_abdulwaz/arch/mips/include/asm/mach-jz4760b
 # -I/home/ros/rg300_abdulwaz/arch/mips/include/asm/mach-generic -iprefix
 # -D__KERNEL__ -DVMLINUX_LOAD_ADDRESS=0xffffffff80010000
 # -DKBUILD_STR(s)=#s -DKBUILD_BASENAME=KBUILD_STR(asm_offsets)
 # -DKBUILD_MODNAME=KBUILD_STR(asm_offsets) -isystem -include -MD
 # -mno-check-zero-division -mabi=32 -mno-abicalls -msoft-float
 # -march=mips32 -auxbase-strip -O2 -Wall -Wundef -Wstrict-prototypes
 # -Wno-trigraphs -Werror-implicit-function-declaration
 # -Wno-format-security -Wdeclaration-after-statement -Wno-pointer-sign
 # -fno-strict-aliasing -fno-common -fno-delete-null-pointer-checks
 # -ffunction-sections -fno-pic -ffreestanding -fno-stack-protector
 # -fomit-frame-pointer -fverbose-asm
 # options enabled:  -falign-loops -fargument-alias -fbranch-count-reg
 # -fcaller-saves -fcprop-registers -fcrossjumping -fcse-follow-jumps
 # -fcse-skip-blocks -fdefer-pop -fearly-inlining
 # -feliminate-unused-debug-types -fexpensive-optimizations -ffunction-cse
 # -ffunction-sections -fgcse -fgcse-lm -fguess-branch-probability -fident
 # -fif-conversion -fif-conversion2 -finline-functions-called-once
 # -fipa-pure-const -fipa-reference -fipa-type-escape -fivopts
 # -fkeep-static-consts -fleading-underscore -floop-optimize
 # -floop-optimize2 -fmath-errno -fmerge-constants -fomit-frame-pointer
 # -foptimize-register-move -foptimize-sibling-calls -fpcc-struct-return
 # -fpeephole -fpeephole2 -fregmove -freorder-blocks -freorder-functions
 # -frerun-cse-after-loop -frerun-loop-opt -fsched-interblock -fsched-spec
 # -fsched-stalled-insns-dep -fschedule-insns -fschedule-insns2
 # -fshow-column -fsplit-ivs-in-unroller -fstrength-reduce -fthread-jumps
 # -ftrapping-math -ftree-ccp -ftree-ch -ftree-copy-prop -ftree-copyrename
 # -ftree-dce -ftree-dominator-opts -ftree-dse -ftree-fre -ftree-loop-im
 # -ftree-loop-ivcanon -ftree-loop-optimize -ftree-lrs -ftree-pre
 # -ftree-salias -ftree-sink -ftree-sra -ftree-store-ccp
 # -ftree-store-copy-prop -ftree-ter -ftree-vect-loop-version -ftree-vrp
 # -funit-at-a-time -fverbose-asm -fzero-initialized-in-bss -mdivide-traps
 # -mdouble-float -mel -mexplicit-relocs -mfp-exceptions -mfp32
 # -mfused-madd -mgp32 -mlong32 -mno-mips16 -mno-mips3d -msoft-float
 # -msplit-addresses

 # Compiler executable checksum: df4e7dfca3127c6920acfc37cac090fc

#APP
	.macro _ssnop; sll $0, $0, 1; .endm
	.macro _ehb; sll $0, $0, 3; .endm
	.macro mtc0_tlbw_hazard; _ssnop; _ssnop; _ehb; .endm
	.macro tlbw_use_hazard; _ssnop; _ssnop; _ssnop; _ehb; .endm
	.macro tlb_probe_hazard; _ssnop; _ssnop; _ssnop; _ehb; .endm
	.macro irq_enable_hazard; _ssnop; _ssnop; _ssnop; _ehb; .endm
	.macro irq_disable_hazard; _ssnop; _ssnop; _ssnop; _ehb; .endm
	.macro back_to_back_c0_hazard; _ssnop; _ssnop; _ssnop; _ehb; .endm
	.macro enable_fpu_hazard; nop; nop; nop; nop; .endm
	.macro disable_fpu_hazard; _ehb; .endm
		.macro	raw_local_irq_enable				
	.set	push						
	.set	reorder						
	.set	noat						
	mfc0	$1,$12						
	ori	$1,0x1f						
	xori	$1,0x1e						
	mtc0	$1,$12						
	irq_enable_hazard					
	.set	pop						
	.endm
		.macro	raw_local_irq_disable
	.set	push						
	.set	noat						
	mfc0	$1,$12						
	ori	$1,0x1f						
	xori	$1,0x1f						
	.set	noreorder					
	mtc0	$1,$12						
	irq_disable_hazard					
	.set	pop						
	.endm							

		.macro	raw_local_save_flags flags			
	.set	push						
	.set	reorder						
	mfc0	\flags, $12					
	.set	pop						
	.endm							

		.macro	raw_local_irq_save result			
	.set	push						
	.set	reorder						
	.set	noat						
	mfc0	\result, $12					
	ori	$1, \result, 0x1f				
	xori	$1, 0x1f					
	.set	noreorder					
	mtc0	$1, $12						
	irq_disable_hazard					
	.set	pop						
	.endm							

		.macro	raw_local_irq_restore flags			
	.set	push						
	.set	noreorder					
	.set	noat						
	mfc0	$1, $12						
	andi	\flags, 1					
	ori	$1, 0x1f					
	xori	$1, 0x1f					
	or	\flags, $1					
	mtc0	\flags, $12					
	irq_disable_hazard					
	.set	pop						
	.endm							

#NO_APP
	.section	.text.output_ptreg_defines,"ax",@progbits
	.align	2
	.globl	output_ptreg_defines
	.ent	output_ptreg_defines
	.type	output_ptreg_defines, @function
output_ptreg_defines:
	.frame	$sp,0,$31		# vars= 0, regs= 0/0, args= 0, gp= 0
	.mask	0x00000000,0
	.fmask	0x00000000,0
#APP
	
->#MIPS pt_regs offsets.
	
->PT_R0 24 offsetof(struct pt_regs, regs[0])	 #
	
->PT_R1 28 offsetof(struct pt_regs, regs[1])	 #
	
->PT_R2 32 offsetof(struct pt_regs, regs[2])	 #
	
->PT_R3 36 offsetof(struct pt_regs, regs[3])	 #
	
->PT_R4 40 offsetof(struct pt_regs, regs[4])	 #
	
->PT_R5 44 offsetof(struct pt_regs, regs[5])	 #
	
->PT_R6 48 offsetof(struct pt_regs, regs[6])	 #
	
->PT_R7 52 offsetof(struct pt_regs, regs[7])	 #
	
->PT_R8 56 offsetof(struct pt_regs, regs[8])	 #
	
->PT_R9 60 offsetof(struct pt_regs, regs[9])	 #
	
->PT_R10 64 offsetof(struct pt_regs, regs[10])	 #
	
->PT_R11 68 offsetof(struct pt_regs, regs[11])	 #
	
->PT_R12 72 offsetof(struct pt_regs, regs[12])	 #
	
->PT_R13 76 offsetof(struct pt_regs, regs[13])	 #
	
->PT_R14 80 offsetof(struct pt_regs, regs[14])	 #
	
->PT_R15 84 offsetof(struct pt_regs, regs[15])	 #
	
->PT_R16 88 offsetof(struct pt_regs, regs[16])	 #
	
->PT_R17 92 offsetof(struct pt_regs, regs[17])	 #
	
->PT_R18 96 offsetof(struct pt_regs, regs[18])	 #
	
->PT_R19 100 offsetof(struct pt_regs, regs[19])	 #
	
->PT_R20 104 offsetof(struct pt_regs, regs[20])	 #
	
->PT_R21 108 offsetof(struct pt_regs, regs[21])	 #
	
->PT_R22 112 offsetof(struct pt_regs, regs[22])	 #
	
->PT_R23 116 offsetof(struct pt_regs, regs[23])	 #
	
->PT_R24 120 offsetof(struct pt_regs, regs[24])	 #
	
->PT_R25 124 offsetof(struct pt_regs, regs[25])	 #
	
->PT_R26 128 offsetof(struct pt_regs, regs[26])	 #
	
->PT_R27 132 offsetof(struct pt_regs, regs[27])	 #
	
->PT_R28 136 offsetof(struct pt_regs, regs[28])	 #
	
->PT_R29 140 offsetof(struct pt_regs, regs[29])	 #
	
->PT_R30 144 offsetof(struct pt_regs, regs[30])	 #
	
->PT_R31 148 offsetof(struct pt_regs, regs[31])	 #
	
->PT_LO 160 offsetof(struct pt_regs, lo)	 #
	
->PT_HI 156 offsetof(struct pt_regs, hi)	 #
	
->PT_EPC 172 offsetof(struct pt_regs, cp0_epc)	 #
	
->PT_BVADDR 164 offsetof(struct pt_regs, cp0_badvaddr)	 #
	
->PT_STATUS 152 offsetof(struct pt_regs, cp0_status)	 #
	
->PT_CAUSE 168 offsetof(struct pt_regs, cp0_cause)	 #
	
->PT_SIZE 176 sizeof(struct pt_regs)	 #
	
->
#NO_APP
	j	$31
	.end	output_ptreg_defines
	.section	.text.output_task_defines,"ax",@progbits
	.align	2
	.globl	output_task_defines
	.ent	output_task_defines
	.type	output_task_defines, @function
output_task_defines:
	.frame	$sp,0,$31		# vars= 0, regs= 0/0, args= 0, gp= 0
	.mask	0x00000000,0
	.fmask	0x00000000,0
#APP
	
->#MIPS task_struct offsets.
	
->TASK_STATE 0 offsetof(struct task_struct, state)	 #
	
->TASK_THREAD_INFO 4 offsetof(struct task_struct, stack)	 #
	
->TASK_FLAGS 12 offsetof(struct task_struct, flags)	 #
	
->TASK_MM 216 offsetof(struct task_struct, mm)	 #
	
->TASK_PID 252 offsetof(struct task_struct, pid)	 #
	
->TASK_STRUCT_SIZE 1152 sizeof(struct task_struct)	 #
	
->
#NO_APP
	j	$31
	.end	output_task_defines
	.section	.text.output_thread_info_defines,"ax",@progbits
	.align	2
	.globl	output_thread_info_defines
	.ent	output_thread_info_defines
	.type	output_thread_info_defines, @function
output_thread_info_defines:
	.frame	$sp,0,$31		# vars= 0, regs= 0/0, args= 0, gp= 0
	.mask	0x00000000,0
	.fmask	0x00000000,0
#APP
	
->#MIPS thread_info offsets.
	
->TI_TASK 0 offsetof(struct thread_info, task)	 #
	
->TI_EXEC_DOMAIN 4 offsetof(struct thread_info, exec_domain)	 #
	
->TI_FLAGS 8 offsetof(struct thread_info, flags)	 #
	
->TI_TP_VALUE 12 offsetof(struct thread_info, tp_value)	 #
	
->TI_CPU 16 offsetof(struct thread_info, cpu)	 #
	
->TI_PRE_COUNT 20 offsetof(struct thread_info, preempt_count)	 #
	
->TI_ADDR_LIMIT 24 offsetof(struct thread_info, addr_limit)	 #
	
->TI_RESTART_BLOCK 32 offsetof(struct thread_info, restart_block)	 #
	
->TI_REGS 72 offsetof(struct thread_info, regs)	 #
	
->_THREAD_SIZE 8192 THREAD_SIZE	 #
	
->_THREAD_MASK 8191 THREAD_MASK	 #
	
->
#NO_APP
	j	$31
	.end	output_thread_info_defines
	.section	.text.output_thread_defines,"ax",@progbits
	.align	2
	.globl	output_thread_defines
	.ent	output_thread_defines
	.type	output_thread_defines, @function
output_thread_defines:
	.frame	$sp,0,$31		# vars= 0, regs= 0/0, args= 0, gp= 0
	.mask	0x00000000,0
	.fmask	0x00000000,0
#APP
	
->#MIPS specific thread_struct offsets.
	
->THREAD_REG16 520 offsetof(struct task_struct, thread.reg16)	 #
	
->THREAD_REG17 524 offsetof(struct task_struct, thread.reg17)	 #
	
->THREAD_REG18 528 offsetof(struct task_struct, thread.reg18)	 #
	
->THREAD_REG19 532 offsetof(struct task_struct, thread.reg19)	 #
	
->THREAD_REG20 536 offsetof(struct task_struct, thread.reg20)	 #
	
->THREAD_REG21 540 offsetof(struct task_struct, thread.reg21)	 #
	
->THREAD_REG22 544 offsetof(struct task_struct, thread.reg22)	 #
	
->THREAD_REG23 548 offsetof(struct task_struct, thread.reg23)	 #
	
->THREAD_REG29 552 offsetof(struct task_struct, thread.reg29)	 #
	
->THREAD_REG30 556 offsetof(struct task_struct, thread.reg30)	 #
	
->THREAD_REG31 560 offsetof(struct task_struct, thread.reg31)	 #
	
->THREAD_STATUS 564 offsetof(struct task_struct, thread.cp0_status)	 #
	
->THREAD_FPU 568 offsetof(struct task_struct, thread.fpu)	 #
	
->THREAD_BVADDR 884 offsetof(struct task_struct, thread.cp0_badvaddr)	 #
	
->THREAD_BUADDR 888 offsetof(struct task_struct, thread.cp0_baduaddr)	 #
	
->THREAD_ECODE 892 offsetof(struct task_struct, thread.error_code)	 #
	
->THREAD_TRAPNO 896 offsetof(struct task_struct, thread.trap_no)	 #
	
->THREAD_TRAMP 900 offsetof(struct task_struct, thread.irix_trampoline)	 #
	
->THREAD_OLDCTX 904 offsetof(struct task_struct, thread.irix_oldctx)	 #
	
->
#NO_APP
	j	$31
	.end	output_thread_defines
	.section	.text.output_thread_fpu_defines,"ax",@progbits
	.align	2
	.globl	output_thread_fpu_defines
	.ent	output_thread_fpu_defines
	.type	output_thread_fpu_defines, @function
output_thread_fpu_defines:
	.frame	$sp,0,$31		# vars= 0, regs= 0/0, args= 0, gp= 0
	.mask	0x00000000,0
	.fmask	0x00000000,0
#APP
	
->THREAD_FPR0 568 offsetof(struct task_struct, thread.fpu.fpr[0])	 #
	
->THREAD_FPR1 576 offsetof(struct task_struct, thread.fpu.fpr[1])	 #
	
->THREAD_FPR2 584 offsetof(struct task_struct, thread.fpu.fpr[2])	 #
	
->THREAD_FPR3 592 offsetof(struct task_struct, thread.fpu.fpr[3])	 #
	
->THREAD_FPR4 600 offsetof(struct task_struct, thread.fpu.fpr[4])	 #
	
->THREAD_FPR5 608 offsetof(struct task_struct, thread.fpu.fpr[5])	 #
	
->THREAD_FPR6 616 offsetof(struct task_struct, thread.fpu.fpr[6])	 #
	
->THREAD_FPR7 624 offsetof(struct task_struct, thread.fpu.fpr[7])	 #
	
->THREAD_FPR8 632 offsetof(struct task_struct, thread.fpu.fpr[8])	 #
	
->THREAD_FPR9 640 offsetof(struct task_struct, thread.fpu.fpr[9])	 #
	
->THREAD_FPR10 648 offsetof(struct task_struct, thread.fpu.fpr[10])	 #
	
->THREAD_FPR11 656 offsetof(struct task_struct, thread.fpu.fpr[11])	 #
	
->THREAD_FPR12 664 offsetof(struct task_struct, thread.fpu.fpr[12])	 #
	
->THREAD_FPR13 672 offsetof(struct task_struct, thread.fpu.fpr[13])	 #
	
->THREAD_FPR14 680 offsetof(struct task_struct, thread.fpu.fpr[14])	 #
	
->THREAD_FPR15 688 offsetof(struct task_struct, thread.fpu.fpr[15])	 #
	
->THREAD_FPR16 696 offsetof(struct task_struct, thread.fpu.fpr[16])	 #
	
->THREAD_FPR17 704 offsetof(struct task_struct, thread.fpu.fpr[17])	 #
	
->THREAD_FPR18 712 offsetof(struct task_struct, thread.fpu.fpr[18])	 #
	
->THREAD_FPR19 720 offsetof(struct task_struct, thread.fpu.fpr[19])	 #
	
->THREAD_FPR20 728 offsetof(struct task_struct, thread.fpu.fpr[20])	 #
	
->THREAD_FPR21 736 offsetof(struct task_struct, thread.fpu.fpr[21])	 #
	
->THREAD_FPR22 744 offsetof(struct task_struct, thread.fpu.fpr[22])	 #
	
->THREAD_FPR23 752 offsetof(struct task_struct, thread.fpu.fpr[23])	 #
	
->THREAD_FPR24 760 offsetof(struct task_struct, thread.fpu.fpr[24])	 #
	
->THREAD_FPR25 768 offsetof(struct task_struct, thread.fpu.fpr[25])	 #
	
->THREAD_FPR26 776 offsetof(struct task_struct, thread.fpu.fpr[26])	 #
	
->THREAD_FPR27 784 offsetof(struct task_struct, thread.fpu.fpr[27])	 #
	
->THREAD_FPR28 792 offsetof(struct task_struct, thread.fpu.fpr[28])	 #
	
->THREAD_FPR29 800 offsetof(struct task_struct, thread.fpu.fpr[29])	 #
	
->THREAD_FPR30 808 offsetof(struct task_struct, thread.fpu.fpr[30])	 #
	
->THREAD_FPR31 816 offsetof(struct task_struct, thread.fpu.fpr[31])	 #
	
->THREAD_FCR31 824 offsetof(struct task_struct, thread.fpu.fcr31)	 #
	
->
#NO_APP
	j	$31
	.end	output_thread_fpu_defines
	.section	.text.output_mm_defines,"ax",@progbits
	.align	2
	.globl	output_mm_defines
	.ent	output_mm_defines
	.type	output_mm_defines, @function
output_mm_defines:
	.frame	$sp,0,$31		# vars= 0, regs= 0/0, args= 0, gp= 0
	.mask	0x00000000,0
	.fmask	0x00000000,0
#APP
	
->#Size of struct page
	
->STRUCT_PAGE_SIZE 32 sizeof(struct page)	 #
	
->
	
->#Linux mm_struct offsets.
	
->MM_USERS 40 offsetof(struct mm_struct, mm_users)	 #
	
->MM_PGD 36 offsetof(struct mm_struct, pgd)	 #
	
->MM_CONTEXT 328 offsetof(struct mm_struct, context)	 #
	
->
	
->_PAGE_SIZE 4096 PAGE_SIZE	 #
	
->_PAGE_SHIFT 12 PAGE_SHIFT	 #
	
->
	
->_PGD_T_SIZE 4 sizeof(pgd_t)	 #
	
->_PMD_T_SIZE 4 sizeof(pmd_t)	 #
	
->_PTE_T_SIZE 4 sizeof(pte_t)	 #
	
->
	
->_PGD_T_LOG2 2 PGD_T_LOG2	 #
	
->_PMD_T_LOG2 2 PMD_T_LOG2	 #
	
->_PTE_T_LOG2 2 PTE_T_LOG2	 #
	
->
	
->_PGD_ORDER 0 PGD_ORDER	 #
	
->_PMD_ORDER 1 PMD_ORDER	 #
	
->_PTE_ORDER 0 PTE_ORDER	 #
	
->
	
->_PMD_SHIFT 22 PMD_SHIFT	 #
	
->_PGDIR_SHIFT 22 PGDIR_SHIFT	 #
	
->
	
->_PTRS_PER_PGD 1024 PTRS_PER_PGD	 #
	
->_PTRS_PER_PMD 1 PTRS_PER_PMD	 #
	
->_PTRS_PER_PTE 1024 PTRS_PER_PTE	 #
	
->
#NO_APP
	j	$31
	.end	output_mm_defines
	.section	.text.output_sc_defines,"ax",@progbits
	.align	2
	.globl	output_sc_defines
	.ent	output_sc_defines
	.type	output_sc_defines, @function
output_sc_defines:
	.frame	$sp,0,$31		# vars= 0, regs= 0/0, args= 0, gp= 0
	.mask	0x00000000,0
	.fmask	0x00000000,0
#APP
	
->#Linux sigcontext offsets.
	
->SC_REGS 16 offsetof(struct sigcontext, sc_regs)	 #
	
->SC_FPREGS 272 offsetof(struct sigcontext, sc_fpregs)	 #
	
->SC_ACX 528 offsetof(struct sigcontext, sc_acx)	 #
	
->SC_MDHI 552 offsetof(struct sigcontext, sc_mdhi)	 #
	
->SC_MDLO 560 offsetof(struct sigcontext, sc_mdlo)	 #
	
->SC_PC 8 offsetof(struct sigcontext, sc_pc)	 #
	
->SC_FPC_CSR 532 offsetof(struct sigcontext, sc_fpc_csr)	 #
	
->SC_FPC_EIR 536 offsetof(struct sigcontext, sc_fpc_eir)	 #
	
->SC_HI1 568 offsetof(struct sigcontext, sc_hi1)	 #
	
->SC_LO1 572 offsetof(struct sigcontext, sc_lo1)	 #
	
->SC_HI2 576 offsetof(struct sigcontext, sc_hi2)	 #
	
->SC_LO2 580 offsetof(struct sigcontext, sc_lo2)	 #
	
->SC_HI3 584 offsetof(struct sigcontext, sc_hi3)	 #
	
->SC_LO3 588 offsetof(struct sigcontext, sc_lo3)	 #
	
->
#NO_APP
	j	$31
	.end	output_sc_defines
	.section	.text.output_signal_defined,"ax",@progbits
	.align	2
	.globl	output_signal_defined
	.ent	output_signal_defined
	.type	output_signal_defined, @function
output_signal_defined:
	.frame	$sp,0,$31		# vars= 0, regs= 0/0, args= 0, gp= 0
	.mask	0x00000000,0
	.fmask	0x00000000,0
#APP
	
->#Linux signal numbers.
	
->_SIGHUP 1 SIGHUP	 #
	
->_SIGINT 2 SIGINT	 #
	
->_SIGQUIT 3 SIGQUIT	 #
	
->_SIGILL 4 SIGILL	 #
	
->_SIGTRAP 5 SIGTRAP	 #
	
->_SIGIOT 6 SIGIOT	 #
	
->_SIGABRT 6 SIGABRT	 #
	
->_SIGEMT 7 SIGEMT	 #
	
->_SIGFPE 8 SIGFPE	 #
	
->_SIGKILL 9 SIGKILL	 #
	
->_SIGBUS 10 SIGBUS	 #
	
->_SIGSEGV 11 SIGSEGV	 #
	
->_SIGSYS 12 SIGSYS	 #
	
->_SIGPIPE 13 SIGPIPE	 #
	
->_SIGALRM 14 SIGALRM	 #
	
->_SIGTERM 15 SIGTERM	 #
	
->_SIGUSR1 16 SIGUSR1	 #
	
->_SIGUSR2 17 SIGUSR2	 #
	
->_SIGCHLD 18 SIGCHLD	 #
	
->_SIGPWR 19 SIGPWR	 #
	
->_SIGWINCH 20 SIGWINCH	 #
	
->_SIGURG 21 SIGURG	 #
	
->_SIGIO 22 SIGIO	 #
	
->_SIGSTOP 23 SIGSTOP	 #
	
->_SIGTSTP 24 SIGTSTP	 #
	
->_SIGCONT 25 SIGCONT	 #
	
->_SIGTTIN 26 SIGTTIN	 #
	
->_SIGTTOU 27 SIGTTOU	 #
	
->_SIGVTALRM 28 SIGVTALRM	 #
	
->_SIGPROF 29 SIGPROF	 #
	
->_SIGXCPU 30 SIGXCPU	 #
	
->_SIGXFSZ 31 SIGXFSZ	 #
	
->
#NO_APP
	j	$31
	.end	output_signal_defined
	.section	.text.output_irq_cpustat_t_defines,"ax",@progbits
	.align	2
	.globl	output_irq_cpustat_t_defines
	.ent	output_irq_cpustat_t_defines
	.type	output_irq_cpustat_t_defines, @function
output_irq_cpustat_t_defines:
	.frame	$sp,0,$31		# vars= 0, regs= 0/0, args= 0, gp= 0
	.mask	0x00000000,0
	.fmask	0x00000000,0
#APP
	
->#Linux irq_cpustat_t offsets.
	
->IC_SOFTIRQ_PENDING 0 offsetof(irq_cpustat_t, __softirq_pending)	 #
	
->IC_IRQ_CPUSTAT_T 32 sizeof(irq_cpustat_t)	 #
	
->
#NO_APP
	j	$31
	.end	output_irq_cpustat_t_defines
	.ident	"GCC: (GNU) 4.1.2"
