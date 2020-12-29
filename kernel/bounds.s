	.file	1 "bounds.c"
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
 # -DKBUILD_STR(s)=#s -DKBUILD_BASENAME=KBUILD_STR(bounds)
 # -DKBUILD_MODNAME=KBUILD_STR(bounds) -isystem -include -MD
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

	.section	.text.foo,"ax",@progbits
	.align	2
	.globl	foo
	.ent	foo
	.type	foo, @function
foo:
	.frame	$sp,0,$31		# vars= 0, regs= 0/0, args= 0, gp= 0
	.mask	0x00000000,0
	.fmask	0x00000000,0
#APP
	
->NR_PAGEFLAGS 23 __NR_PAGEFLAGS	 #
	
->MAX_NR_ZONES 2 __MAX_NR_ZONES	 #
#NO_APP
	j	$31
	.end	foo
	.ident	"GCC: (GNU) 4.1.2"
