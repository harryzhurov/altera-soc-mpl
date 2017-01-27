/******************************************************************************
*
* Copyright 2014 Altera Corporation. All Rights Reserved.
* 
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* 
* 1. Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
* 
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
* 
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software without
* specific prior written permission.
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
* 
******************************************************************************/

/*
 * $Id: //acds/rel/16.1/embedded/examples/software/Altera-SoCFPGA-HardwareLib-MPL/core/startup.S#4 $
 */

#include "asm.h"

    PRESERVE8
    AREA(MPLinit, CODE, READONLY)

	/* =====  Mode bit definitions, and interrupt flag masks */
EQU(	Mode_USR,       0x10 )  /* un-privileged mode */
EQU(	Mode_FIQ,	0x11 )	/* entered as result of fast interrupt */
EQU(	Mode_IRQ,	0x12 )	/* entered as result of normal interrupt */
EQU(	Mode_SVC,	0x13 )	/* entered on reset, or execution of SVC instruction */
EQU(	Mode_MON,	0x16 )	/* mode to swith between secure and non-secure state, or SMC call */
EQU(	Mode_ABT,	0x17 )	/* entered as result of a data abort exception or prefetch abort exception */
EQU(	Mode_UNDEF,	0x1B )	/* entered as a result of an instruction-related error */
EQU(	Mode_SYS,	0x1F )	/* priveleged accesses  */
EQU(	IRQ_disable,	0x80 )	/* mask to disable IRQ bit in CPSR */
EQU(	FIQ_disable,	0x40 )	/* mask to disable FIQ bit in CPSR */

	/* ===== CR1 bits  */
EQU(	CP15_CR_V,	(1 << 13) )

    ENTRY
    EXPORT(mpl_vectors)
    IMPORT(main)
    IMPORT(debug_memory_ptr)

LABEL(mpl_vectors)
	LDR PC, mpl_reset_addr
	LDR PC, mpl_undef_addr
	LDR PC, mpl_svc_addr
	LDR PC, mpl_prefetch_addr
	LDR PC, mpl_abort_addr
	LDR PC, mpl_reserve_addr
	LDR PC, mpl_irq_addr
	LDR PC, mpl_fiq_addr

LABEL(mpl_reset_addr)    DCD mpl_reset
LABEL(mpl_undef_addr)    DCD mpl_hdlr_undef
LABEL(mpl_svc_addr)      DCD mpl_hdlr_svc
LABEL(mpl_prefetch_addr) DCD mpl_hdlr_prefetch
LABEL(mpl_abort_addr)    DCD mpl_hdlr_abort
LABEL(mpl_reserve_addr)  DCD mpl_hdlr_reserve
LABEL(mpl_irq_addr)      DCD mpl_hdlr_irq
LABEL(mpl_fiq_addr)      DCD mpl_hdlr_fiq


/*===== Program Header
 the header required by BootROM, should start at Ox0040 
 this is only a place holder outlining required format
 The final header is to be added by mkimage tool 	*/

#ifdef soc_cv_av
LABEL(mpl_header)
	DCD	0x31305341	/* Magic   */
	DCB	0x00		/* Version */
	DCB	0x00		/* Flags   */
	DCW	0x0000   	/* crc     */
	DCW	0x0000		/* Length  */
	DCW	0x0000		/* Spare   */
	DCD	0x0000		/* Checksum */
	/* entry point */
	B mpl_reset
#else
LABEL(mpl_header)
	DCD	0x31305341	/* Magic   */
	DCB	0x00		/* Version */
	DCB	0x00		/* Flags   */
	DCW 0x0000      /* PH Length */
	DCD	0x00000000  /* Length  */
	DCD 0x00000000  /* Start Offset */
	DCW	0x0000		/* Spare   */
	DCW	0x0000		/* Checksum */
	/* entry point */
	B mpl_reset
#endif

LABEL(version)
	DCD	0x12340001

	BALIGN(4)

#ifdef GNU
#define data_loadto	0xffff0000
#else
#define __bss_start	||Image$$MPL_BSS$$ZI$$Base||
#define __bss_end	||Image$$MPL_BSS$$ZI$$Limit||

#define user_stack_start ||Image$$ARM_LIB_STACKHEAP$$ZI$$Base||
#define user_stack_end	||Image$$ARM_LIB_STACKHEAP$$ZI$$Limit||

#define endof_consts	||Image$$MPL_CODE$$RO$$Limit||	/* This is where the data section is loaded */

#define data_loadto	||Image$$MPL_DATA$$Base||
#define data_loadto_end	||Image$$MPL_DATA$$Limit||

	IMPORT __bss_start
	IMPORT __bss_end
	IMPORT user_stack_end
	IMPORT endof_consts
	IMPORT data_loadto
	IMPORT data_loadto_end
#endif

/* import symbols from linker script */
LABEL(_user_stack)
	DCD	user_stack_end

	BALIGN(4)

	EXPORT(mpl_reset)
/* ===== Reset Handler and true entry point */
LABEL(mpl_reset)
#if defined(BOOT_FROM_FPGA) || !defined(GNU)
#if !defined(GNU)
	IF :DEF:BOOT_FROM_FPGA
#endif
        /* FPGA Boot:Relocate DATA to OCR */
        ldr     r4, =endof_consts
        ldr     r5, =data_loadto
	ldr	r7, =data_loadto_end
LABEL(copy_loop)
        ldr     r6, [r4]
        str     r6, [r5]
        cmp     r5, r7          /* done copying?  */
        bge     mpl_reset2      /* yes-> jump to mpl_reset2 */
        add     r4, r4, #4
        add     r5, r5, #4
        b       copy_loop

#if !defined(GNU)
	ENDIF
#endif
#endif

LABEL(mpl_reset2)
	/******************************************************************
	@ After BootROM executes, r0-r3 contain useful infomation on entry. 
	@ As a first step we need to preserve that information.
	@ write the debug memory header and value of R0 - R3 (which passed
	@ by BootROM) into debug memory region (which is on ocram)
	******************************************************************/

	ldr r4, =debug_memory_ptr	/* r4= address of global var*/
	ldr	r5, [r4]	/* read value of preloader debub memory pointer*/
	ldr	r4, =0x01444D	/* version id (0x01) and Magic # (0x444d)*/
	stmia	r5!, {r4}	
	stmia	r5!, {r0 -r3}   /* r0-r3 from BootROM */
	ldr	r4, =debug_memory_ptr   /* update debug memory pointer */
	str	r5, [r4]

	/* switch to SVC32 mode with disabled interrupts */
	mrs	r0, cpsr
	bic	r0, r0, #0x1F
	orr	r0, r0, #(Mode_SVC + IRQ_disable + FIQ_disable) /*	0xD3 */
	msr	cpsr_cxsf,r0
	    
	/* check which cpu is executing (just in-case) */
	mrc		p15, 0, r0, c0, c0, 5		/* read arm affinity register */
	ands	r0, r0, #3				/* AND with 3 since only 2 cpus available*/
	beq		primary_cpu_init

	/* @ if here one of secondary cpu's reached this point */
LABEL(secondary_cpu_loop)
	/* wfe */
	b secondary_cpu_loop

	/* primary cpu init
	   must invalidate instruction cache left-on/used by BootROM */
LABEL(primary_cpu_init)

	/* Set bit V of SCTRL Register */
	mrc	p15, 0, r0, c1, c0, 0
	bic	r0, #CP15_CR_V
	mcr	p15, 0, r0, c1, c0, 0

	/* Set VBAR */
	ldr	r0, =mpl_vectors
	mcr	p15, 0, r0, c12, c0, 0    
	
	/* Set up stack pointer with symbol from linker script */
	LDR     sp, _user_stack

	mov	r0, #0
#ifdef soc_cv_av
	/* invalidate TLBs */
	mcr	p15, 0, r0, c8, c7, 0

	/* invalidate icache */
	mcr	p15, 0, r0, c7, c5, 0
  	
	/* invalidate branch predictor */
	mcr	p15, 0, r0, c7, c5, 6
#endif
#ifdef soc_a10
	bl	alt_cache_system_disable 
#endif
	/* DSB instruction */
	mcr     p15, 0, r0, c7, c10, 4
	/* ISB instruction */
	mcr     p15, 0, r0, c7, c5, 4

	/* MMU and cache setup:  clear V, C, A, M bits */
	mrc	p15, 0, r0, c1, c0, 0
	bic	r0, r0, #0x00000007
	bic	r0, r0, #0x00002000

	/* set A, Z, I bits */
	orr	r0, r0, #0x00000002
	orr	r0, r0, #0x00001800
	mcr	p15, 0, r0, c1, c0, 0
    
	/* clear bss section (+ZI in scatter, zero-init) */
	ldr     r0, =__bss_start
	ldr     r1, =__bss_end
	mov     r2, #0
LABEL(zi_loop)
	str     r2, [r0]    /* zi location */
	cmp     r0, r1      /* compare addres */
	add     r0, r0, #4  /* ptr+=4 */
	bne     zi_loop     /* loop until section end */

    /* branch to ARM main to finish scatter loading, ZI init, etc..
     once ARM libs finish scatter loading, will jump to mpl main function in mpl.c  */
	bl main
LABEL(inf_loop)
	wfi
	b inf_loop

/* =====  dummy exceptions, mpl should not get exceptions */
LABEL(mpl_hdlr_undef)
	B mpl_hdlr_undef
LABEL(mpl_hdlr_svc)
	B mpl_hdlr_svc
LABEL(mpl_hdlr_prefetch)
	B mpl_hdlr_prefetch
LABEL(mpl_hdlr_abort)
	B mpl_hdlr_abort
LABEL(mpl_hdlr_reserve)
	B mpl_hdlr_reserve
LABEL(mpl_hdlr_irq)
	B mpl_hdlr_irq
LABEL(mpl_hdlr_fiq)
	B mpl_hdlr_fiq

/* @ Configure the fpga2sdram register */
	EXPORT(sdram_applycfg_ocram)
LABEL(sdram_applycfg_ocram)
	mrc	p15, 0, r0, c1, c0, 0
	bic	r0, #(1 << 11)	 	/* Disable branch predictor (bit Z) */
	bic	r0, #(1 << 12)		/* Disable i-cache (bit I) */
	mcr	p15, 0, r0, c1, c0, 0
	
	mcr  p15, 0, r0, c7, c10, 4	/* DSB */
	mcr  p15, 0, r0, c7, c5, 4    	/* ISB */

	ldr	r1, =0xFFC2505C		/* SOCFPGA_SDR_ADDRESS */
	ldr	r0, [r1]
	mov	r3, #8			/* SDR_CTRLGRP_STATICCFG_APPLYCFG_MASK */
	orr	r0, r0, r3
	str	r0, [r1]

	mcr     p15, 0, r0, c7, c10, 4	/* DSB */
	mcr     p15, 0, r0, c7, c5, 4   /* ISB */

	mrc	p15, 0, r0, c1, c0, 0
	orr	r0, r0, #(1 << 11)	/* Enable back branch predictor */
	orr	r0, r0, #(1 << 12)	/* Enable back i-cache */
	mcr	p15, 0, r0, c1, c0, 0

	mov	pc, lr

	EXPORT(reset_clock_manager)
/* Relocate the sdram_applycfg_ocram function to OCRAM and call it */
LABEL(reset_clock_manager)
	/* Put Main PLL and Peripheral PLL in bypass */
	ldr     r0, SOCFPGA_CLKMGR
	mov     r1, #0x4	/* CLKMGR_BYPASS_ADDRESS */
	mov     r2, #0x19	/* CLKMGR_BYPASS_MAIN_PER_PLL_MASK */
	add     r3, r0, r1
	ldr     r4, [r3]
	orr     r5, r4, r2
	str     r5, [r3]
	dsb
	isb
	mov     r1, #0x54	/*CLKMGR_MAINPLLGRP_MAINQSPICLK_ADDRESS */
	mov     r2, #0x3	/*CLKMGR_MAINQSPICLK_RESET_VALUE */
	add     r3, r0, r1
	str     r2, [r3]
	mov     r1, #0x58	/*CLKMGR_MAINPLLGRP_MAINNANDSDMMCCLK_ADDRESS */
	mov     r2, #0x3	/*CLKMGR_MAINNANDSDMMCCLK_RESET_VALUE */
	add     r3, r0, r1
	str     r2, [r3]
	mov     r1, #0x90	/*CLKMGR_PERPLLGRP_PERQSPICLK_ADDRESS */
	mov     r2, #0x1	/*CLKMGR_PERQSPICLK_RESET_VALUE */
	add     r3, r0, r1
	str     r2, [r3]
	mov     r1, #0x94	/*CLKMGR_PERPLLGRP_PERNANDSDMMCCLK_ADDRESS */
	mov     r2, #0x1	/*CLKMGR_PERNANDSDMMCCLK_RESET_VALUE */
	add     r3, r0, r1
	str     r2, [r3]

	/* Disable the RAM boot */
	ldr     r0, SOCFPGA_RSTMGR
	ldr     r1, SYSMGR_WARMRAMGRP_ENABLE
	mov     r2, #0
	str     r2, [r1]

	/* Trigger warm reset to continue boot normally */
	mov     r1, #0x00000004	/*RSTMGR_CTRL_OFFSET */
	add     r2, r0, r1
	mov     r3, #1
	mov     r3, r3, LSL #1	/*RSTMGR_CTRL_SWWARMRSTREQ_LSB */
	str     r3, [r2]

LABEL(reset_clock_manager_loop)
	dsb
	isb
	b	reset_clock_manager_loop

LABEL(SOCFPGA_CLKMGR)
	DCD 	0xffd04000	/* SOCFPGA_CLKMGR_ADDRESS */
LABEL(SOCFPGA_RSTMGR)
	DCD	0xffd05000	/* SOCFPGA_RSTMGR_ADDRESS */
LABEL(SYSMGR_WARMRAMGRP_ENABLE)
	DCD	0xffd080e0	/* CONFIG_SYSMGR_WARMRAMGRP_ENABLE =>  (SOCFPGA_SYSMGR_ADDRESS + 0xe0) */

	EXPORT(reset_clock_manager_size)
LABEL(reset_clock_manager_size)
	DCD	(reset_clock_manager_size-reset_clock_manager)

/*=====  mark end of file*/
    END
