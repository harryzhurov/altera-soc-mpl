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
 * $Id: //acds/rel/16.1/embedded/examples/software/Altera-SoCFPGA-HardwareLib-MPL/core/altera_ip/sdram.c#2 $
 */

/*
 * Altera SDRAM controller driver
 * For MMR initialization
 * baremetal_fw/drivers/sdram/sdram.c
 *
 * Copyright (C) 2011 Altera Corporation
 * Maintainer: Chin Liang, See <clsee@altera.com>
 *
 */

#include "sdram.h"
#include <alt_types.h>
#include <sequencer_defines.h>
#include <sequencer.h>

#include "socal/socal.h"
#include "socal/hps.h"
#include "socal/alt_rstmgr.h"
#include "socal/alt_sdr.h"

#include "build.h"
#include "alt_dma.h"
#include "alt_dma_program.h"
#include "mpl_common.h"
#include "mpl_config.h"

/* SDRAM debug support */
/*#define SDRAM_DEBUG_ENABLE*/

/*
 * SDRAM MMR init skip read back/verify steps
 * Define to speed up the MMR init process by just write without verifying
 */
#define SDRAM_MMR_SKIP_VERIFY

//#define COMPARE_FAIL_ACTION   ;
#define COMPARE_FAIL_ACTION return 1;


/******************************************************************************
 Function:  SDRAM Calibration Full
 Return:    0 for pass, 1 for fail
******************************************************************************/
int altr_sdram_calibration_full(void)
{
    if (sdram_calibration()==0)  //sdram_calibration return value is flipped
        return 1;
    return 0;
}

//int altr_sdram_calibration_full(void)
//{
//  return sdram_calibration();
//}



/******************************************************************************
< Standard function provided by io.h >
- uint32_t read_register(uint32_t base_addr, uint32_t register_offset)
- void write_register(uint32_t base_addr, uint32_t register_offset,
    uint32_t register_value)
- void clear_mask_register(uint32_t base_addr, uint32_t register_offset,
    uint32_t mask)
- void set_mask_register(uint32_t base_addr, uint32_t register_offset,
    uint32_t mask)
******************************************************************************/
unsigned long sdram_write_register_field (unsigned long masked_value,
    unsigned long data, unsigned long shift, unsigned long mask)
{
    masked_value &= ~ ( mask );
    masked_value |= ( data << shift ) & mask;
    return masked_value;
}

uint32_t sdram_write_verify (uint32_t reg_addr, uint32_t reg_value)
{
#ifndef SDRAM_MMR_SKIP_VERIFY
    uint32_t reg_value1;
#endif
//#ifdef SDRAM_DEBUG_ENABLE
//  puts("Write - Address ");
//  printf("0x%08x Data 0x%08x\n",
//         (unsigned int)((HPS_SDRAM_BASE+register_offset)),(unsigned int)reg_value);
//#endif
    /* Write to register */
    //write_register(HPS_SDRAM_BASE, register_offset,   reg_value);
    alt_write_word(reg_addr, reg_value);
    
#ifndef SDRAM_MMR_SKIP_VERIFY
//#ifdef SDRAM_DEBUG_ENABLE
//  puts("Read and verify...");
//#endif
    /* Read back the wrote value */
    reg_value1 = alt_read_word(reg_addr);
    /* Indicate failure if value not matched */
    if( reg_value1 != reg_value ) {
//#ifdef SDRAM_DEBUG_ENABLE
//      printf("FAIL - Address 0x%08x Expected 0x%08x Data 0x%08x\n",
//          (HPS_SDRAM_BASE+register_offset),
//          reg_value, reg_value1);
//#endif
        return 1;
    }
    //puts("correct!");
#endif  /* SDRAM_MMR_SKIP_VERIFY */
    return 0;
}

/******************************************************************************
 Function:  SDRAM Initialization Full
 Return:    0 for pass, 1 for fail
******************************************************************************/
int altr_sdram_mmr_init_full(void)
{
    unsigned long reg_value;
    unsigned long status=0;

    // Take the SDR peripheral out of reset.
    alt_clrbits_word(ALT_RSTMGR_PERMODRST_ADDR, ALT_RSTMGR_PERMODRST_SDR_SET_MSK);
    
    /***** CTRLCFG *****/
#if defined(CONFIG_HPS_SDR_CTRLCFG_CTRLCFG_MEMTYPE)|\
    defined(CONFIG_HPS_SDR_CTRLCFG_CTRLCFG_MEMBL)|\
    defined(CONFIG_HPS_SDR_CTRLCFG_CTRLCFG_ADDRORDER)|\
    defined(CONFIG_HPS_SDR_CTRLCFG_CTRLCFG_ECCEN)|\
    defined(CONFIG_HPS_SDR_CTRLCFG_CTRLCFG_ECCORREN)|\
    defined(CONFIG_HPS_SDR_CTRLCFG_CTRLCFG_REORDEREN)|\
    defined(CONFIG_HPS_SDR_CTRLCFG_CTRLCFG_STARVELIMIT)|\
    defined(CONFIG_HPS_SDR_CTRLCFG_CTRLCFG_DQSTRKEN)|\
    defined(CONFIG_HPS_SDR_CTRLCFG_CTRLCFG_NODMPINS)
#ifdef SDRAM_DEBUG_ENABLE
    puts("\nConfiguring CTRLCFG\n");
#endif
    //register_offset = SDR_CTRLGRP_CTRLCFG_ADDRESS;
    /* Read original register value */
    //reg_value = read_register(HPS_SDRAM_BASE, register_offset);
    
    reg_value = alt_read_word(ALT_SDR_CTL_CTLCFG_ADDR);
#ifdef CONFIG_HPS_SDR_CTRLCFG_CTRLCFG_MEMTYPE
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_CTRLCFG_MEMTYPE,
            ALT_SDR_CTL_CTLCFG_MEMTYPE_LSB,
            ALT_SDR_CTL_CTLCFG_MEMTYPE_SET_MSK);
#endif
#ifdef CONFIG_HPS_SDR_CTRLCFG_CTRLCFG_MEMBL
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_CTRLCFG_MEMBL,
            ALT_SDR_CTL_CTLCFG_MEMBL_LSB,
            ALT_SDR_CTL_CTLCFG_MEMBL_SET_MSK);
#endif
#ifdef CONFIG_HPS_SDR_CTRLCFG_CTRLCFG_ADDRORDER

    /* FBz 223741: Access to non-existant SDRAM.
     * The SDRAM controller is programmed to use
     * the full 4GB address space, and a protection
     * rule is used to abort non-existant memory space.
     *
     * SDRAM addressing must use Row-Chip-Bank-Column mode.
     */
    reg_value = sdram_write_register_field( reg_value,
            2,//Row-Chip-Bank-Column
            ALT_SDR_CTL_CTLCFG_ADDRORDER_LSB,
            ALT_SDR_CTL_CTLCFG_ADDRORDER_SET_MSK);

#endif
#ifdef CONFIG_HPS_SDR_CTRLCFG_CTRLCFG_ECCEN
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_CTRLCFG_ECCEN,
            ALT_SDR_CTL_CTLCFG_ECCEN_LSB,
            ALT_SDR_CTL_CTLCFG_ECCEN_SET_MSK);
#endif
#ifdef CONFIG_HPS_SDR_CTRLCFG_CTRLCFG_ECCORREN
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_CTRLCFG_ECCORREN,
            ALT_SDR_CTL_CTLCFG_ECCCORREN_LSB,
            ALT_SDR_CTL_CTLCFG_ECCCORREN_SET_MSK);
#endif
#ifdef CONFIG_HPS_SDR_CTRLCFG_CTRLCFG_REORDEREN
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_CTRLCFG_REORDEREN,
            ALT_SDR_CTL_CTLCFG_REORDEREN_LSB,
            ALT_SDR_CTL_CTLCFG_REORDEREN_SET_MSK);
#endif
#ifdef CONFIG_HPS_SDR_CTRLCFG_CTRLCFG_STARVELIMIT
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_CTRLCFG_STARVELIMIT,
            ALT_SDR_CTL_CTLCFG_STARVELIMIT_LSB,
            ALT_SDR_CTL_CTLCFG_STARVELIMIT_SET_MSK);
#endif
#ifdef CONFIG_HPS_SDR_CTRLCFG_CTRLCFG_DQSTRKEN
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_CTRLCFG_DQSTRKEN,
            ALT_SDR_CTL_CTLCFG_DQSTRKEN_LSB,
            ALT_SDR_CTL_CTLCFG_DQSTRKEN_SET_MSK);
#endif
#ifdef CONFIG_HPS_SDR_CTRLCFG_CTRLCFG_NODMPINS
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_CTRLCFG_NODMPINS,
            ALT_SDR_CTL_CTLCFG_NODMPINS_LSB,
            ALT_SDR_CTL_CTLCFG_NODMPINS_SET_MSK);
#endif
    if( sdram_write_verify((uint32_t)ALT_SDR_CTL_CTLCFG_ADDR,   reg_value) == 1 ) {
        status = 1;
        COMPARE_FAIL_ACTION
    }
#endif


    /***** DRAMTIMING1 *****/
#if defined(CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING1_TCWL)|\
    defined(CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING1_AL)|\
    defined(CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING1_TCL)|\
    defined(CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING1_TRRD)|\
    defined(CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING1_TFAW)|\
    defined(CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING1_TRFC)
#ifdef SDRAM_DEBUG_ENABLE
    puts("Configuring DRAMTIMING1\n");
#endif
    //register_offset = SDR_CTRLGRP_DRAMTIMING1_ADDRESS;
    /* Read original register value */
    reg_value = alt_read_word(ALT_SDR_CTL_DRAMTIMING1_ADDR);
#ifdef CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING1_TCWL
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING1_TCWL,
            ALT_SDR_CTL_DRAMTIMING1_TCWL_LSB,
            ALT_SDR_CTL_DRAMTIMING1_TCWL_SET_MSK);
#endif
#ifdef CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING1_AL
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING1_AL,
            ALT_SDR_CTL_DRAMTIMING1_TAL_LSB,
            ALT_SDR_CTL_DRAMTIMING1_TAL_SET_MSK);
#endif
#ifdef CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING1_TCL
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING1_TCL,
            ALT_SDR_CTL_DRAMTIMING1_TCL_LSB,
            ALT_SDR_CTL_DRAMTIMING1_TCL_SET_MSK);
#endif
#ifdef CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING1_TRRD
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING1_TRRD,
            ALT_SDR_CTL_DRAMTIMING1_TRRD_LSB,
            ALT_SDR_CTL_DRAMTIMING1_TRRD_SET_MSK);
#endif
#ifdef CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING1_TFAW
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING1_TFAW,
            ALT_SDR_CTL_DRAMTIMING1_TFAW_LSB,
            ALT_SDR_CTL_DRAMTIMING1_TFAW_SET_MSK);
#endif
#ifdef CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING1_TRFC
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING1_TRFC,
            ALT_SDR_CTL_DRAMTIMING1_TRFC_LSB,
            ALT_SDR_CTL_DRAMTIMING1_TRFC_SET_MSK);
#endif
    if( sdram_write_verify((uint32_t)ALT_SDR_CTL_DRAMTIMING1_ADDR, reg_value) == 1 ) {
        status = 1;
        COMPARE_FAIL_ACTION
    }
#endif


    /***** DRAMTIMING2 *****/
#if defined(CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING2_IF_TREFI)|\
    defined(CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING2_IF_TRCD)|\
    defined(CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING2_IF_TRP)|\
    defined(CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING2_IF_TWR)|\
    defined(CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING2_IF_TWTR)
#ifdef SDRAM_DEBUG_ENABLE
    puts("Configuring DRAMTIMING2\n");
#endif
    //register_offset = SDR_CTRLGRP_DRAMTIMING2_ADDRESS;
    /* Read original register value */
    reg_value = alt_read_word(ALT_SDR_CTL_DRAMTIMING2_ADDR);
#ifdef CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING2_IF_TREFI
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING2_IF_TREFI,
            ALT_SDR_CTL_DRAMTIMING2_TREFI_LSB,
            ALT_SDR_CTL_DRAMTIMING2_TREFI_SET_MSK);
#endif
#ifdef CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING2_IF_TRCD
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING2_IF_TRCD,
            ALT_SDR_CTL_DRAMTIMING2_TRCD_LSB,
            ALT_SDR_CTL_DRAMTIMING2_TRCD_SET_MSK);
#endif
#ifdef CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING2_IF_TRP
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING2_IF_TRP,
            ALT_SDR_CTL_DRAMTIMING2_TRP_LSB,
            ALT_SDR_CTL_DRAMTIMING2_TRP_SET_MSK);
#endif
#ifdef CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING2_IF_TWR
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING2_IF_TWR,
            ALT_SDR_CTL_DRAMTIMING2_TWR_LSB,
            ALT_SDR_CTL_DRAMTIMING2_TWR_SET_MSK);
#endif
#ifdef CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING2_IF_TWTR
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING2_IF_TWTR,
            ALT_SDR_CTL_DRAMTIMING2_TWTR_LSB,
            ALT_SDR_CTL_DRAMTIMING2_TWTR_SET_MSK);
#endif
    if( sdram_write_verify((uint32_t)ALT_SDR_CTL_DRAMTIMING2_ADDR,  reg_value) == 1 ) {
        status = 1;
        COMPARE_FAIL_ACTION
    }
#endif


    /***** DRAMTIMING3 *****/
#if defined(CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING3_TRTP)|\
    defined(CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING3_TRAS)|\
    defined(CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING3_TRC)|\
    defined(CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING3_TMRD)|\
    defined(CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING3_TCCD)
#ifdef SDRAM_DEBUG_ENABLE
    puts("Configuring DRAMTIMING3\n");
#endif
    //register_offset = SDR_CTRLGRP_DRAMTIMING3_ADDRESS;
    /* Read original register value */
    reg_value = alt_read_word(ALT_SDR_CTL_DRAMTIMING3_ADDR);
#ifdef CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING3_TRTP
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING3_TRTP,
            ALT_SDR_CTL_DRAMTIMING3_TRTP_LSB,
            ALT_SDR_CTL_DRAMTIMING3_TRTP_SET_MSK);
#endif
#ifdef CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING3_TRAS
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING3_TRAS,
            ALT_SDR_CTL_DRAMTIMING3_TRAS_LSB,
            ALT_SDR_CTL_DRAMTIMING3_TRAS_SET_MSK);
#endif
#ifdef CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING3_TRC
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING3_TRC,
            ALT_SDR_CTL_DRAMTIMING3_TRC_LSB,
            ALT_SDR_CTL_DRAMTIMING3_TRC_SET_MSK);
#endif
#ifdef CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING3_TMRD
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING3_TMRD,
            ALT_SDR_CTL_DRAMTIMING3_TMRD_LSB,
            ALT_SDR_CTL_DRAMTIMING3_TMRD_SET_MSK);
#endif
#ifdef CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING3_TCCD
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING3_TCCD,
            ALT_SDR_CTL_DRAMTIMING3_TCCD_LSB,
            ALT_SDR_CTL_DRAMTIMING3_TCCD_SET_MSK);
#endif
    if( sdram_write_verify((uint32_t)ALT_SDR_CTL_DRAMTIMING3_ADDR,  reg_value) == 1 ) {
        status = 1;
        COMPARE_FAIL_ACTION
    }
#endif


    /***** DRAMTIMING4 *****/
#if defined(CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING4_SELFRFSHEXIT)|\
    defined(CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING4_PWRDOWNEXIT)
#ifdef SDRAM_DEBUG_ENABLE
    puts("Configuring DRAMTIMING4\n");
#endif
    //register_offset = SDR_CTRLGRP_DRAMTIMING4_ADDRESS;
    /* Read original register value */
    reg_value = alt_read_word(ALT_SDR_CTL_DRAMTIMING4_ADDR);
#ifdef CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING4_SELFRFSHEXIT
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING4_SELFRFSHEXIT,
            ALT_SDR_CTL_DRAMTIMING4_SELFRFSHEXIT_LSB,
            ALT_SDR_CTL_DRAMTIMING4_SELFRFSHEXIT_SET_MSK);
#endif
#ifdef CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING4_PWRDOWNEXIT
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING4_PWRDOWNEXIT,
            ALT_SDR_CTL_DRAMTIMING4_PWRDOWNEXIT_LSB,
            ALT_SDR_CTL_DRAMTIMING4_PWRDOWNEXIT_SET_MSK);
#endif
    if( sdram_write_verify((uint32_t)ALT_SDR_CTL_DRAMTIMING4_ADDR,  reg_value) == 1 ) {
        status = 1;
        COMPARE_FAIL_ACTION
    }
#endif


    /***** LOWPWRTIMING *****/
#ifdef CONFIG_HPS_SDR_CTRLCFG_LOWPWRTIMING_AUTOPDCYCLES
#ifdef SDRAM_DEBUG_ENABLE
    puts("Configuring LOWPWRTIMING\n");
#endif
    //register_offset = SDR_CTRLGRP_LOWPWRTIMING_ADDRESS;
    /* Read original register value */
    reg_value = alt_read_word(ALT_SDR_CTL_LOWPWRTIMING_ADDR);
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_LOWPWRTIMING_AUTOPDCYCLES,
            ALT_SDR_CTL_LOWPWRTIMING_AUTOPDCYCLES_LSB,
            ALT_SDR_CTL_LOWPWRTIMING_AUTOPDCYCLES_SET_MSK);
    if( sdram_write_verify((uint32_t)ALT_SDR_CTL_LOWPWRTIMING_ADDR, reg_value) == 1 ) {
        status = 1;
        COMPARE_FAIL_ACTION
    }
#endif

    /***** DRAMADDRW *****/
#if defined(CONFIG_HPS_SDR_CTRLCFG_DRAMADDRW_COLBITS)|\
    defined(CONFIG_HPS_SDR_CTRLCFG_DRAMADDRW_ROWBITS)|\
    defined(CONFIG_HPS_SDR_CTRLCFG_DRAMADDRW_BANKBITS)|\
    defined(CONFIG_HPS_SDR_CTRLCFG_DRAMADDRW_CSBITS)
#ifdef SDRAM_DEBUG_ENABLE
    puts("Configuring DRAMADDRW\n");
#endif
    //register_offset = SDR_CTRLGRP_DRAMADDRW_ADDRESS;
    /* Read original register value */
    reg_value = alt_read_word(ALT_SDR_CTL_DRAMADDRW_ADDR);
#ifdef CONFIG_HPS_SDR_CTRLCFG_DRAMADDRW_COLBITS
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_DRAMADDRW_COLBITS,
            ALT_SDR_CTL_DRAMADDRW_COLBITS_LSB,
            ALT_SDR_CTL_DRAMADDRW_COLBITS_SET_MSK);
#endif
#ifdef CONFIG_HPS_SDR_CTRLCFG_DRAMADDRW_ROWBITS

    /* FBz 223741: Access to non-existant SDRAM.
     * The SDRAM controller is programmed to use
     * the full 4GB address space, and a protection
     * rule is used to abort non-existant memory space.
     *
     * Rowbits are calculated as the number of bits
     * needed to fill the entire 4GB address space.
     */
    reg_value = sdram_write_register_field( reg_value,
            32 - ( CONFIG_HPS_SDR_CTRLCFG_DRAMADDRW_BANKBITS
                 + CONFIG_HPS_SDR_CTRLCFG_DRAMADDRW_COLBITS
                 + CONFIG_HPS_SDR_CTRLCFG_DRAMADDRW_CSBITS-1
                 + CONFIG_HPS_SDR_CTRLCFG_CTRLWIDTH_CTRLWIDTH),
            ALT_SDR_CTL_DRAMADDRW_ROWBITS_LSB,
            ALT_SDR_CTL_DRAMADDRW_ROWBITS_SET_MSK);
#endif
#ifdef CONFIG_HPS_SDR_CTRLCFG_DRAMADDRW_BANKBITS
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_DRAMADDRW_BANKBITS,
            ALT_SDR_CTL_DRAMADDRW_BANKBITS_LSB,
            ALT_SDR_CTL_DRAMADDRW_BANKBITS_SET_MSK);
#endif
#ifdef CONFIG_HPS_SDR_CTRLCFG_DRAMADDRW_CSBITS

    /* The CSBITS value from Qsys is the
     * number of CS wires used on the board.
     * For ArriaV/CycloneV, this is either 1 or 2.
     *
     * The SDRAM controller needs the number of bits
     * used to build the address. Generally, this is
     * LOG2(CSBITS), or in this case, CSBITS-1.
     */
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_DRAMADDRW_CSBITS-1,
            ALT_SDR_CTL_DRAMADDRW_CSBITS_LSB,
            ALT_SDR_CTL_DRAMADDRW_CSBITS_SET_MSK);

#endif
    if( sdram_write_verify((uint32_t)ALT_SDR_CTL_DRAMADDRW_ADDR,    reg_value) == 1 ) {
        status = 1;
        COMPARE_FAIL_ACTION
    }
#endif

    /***** DRAMIFWIDTH *****/
#ifdef CONFIG_HPS_SDR_CTRLCFG_DRAMIFWIDTH_IFWIDTH
#ifdef SDRAM_DEBUG_ENABLE
    puts("Configuring DRAMIFWIDTH\n");
#endif
    //register_offset = SDR_CTRLGRP_DRAMIFWIDTH_ADDRESS;
    /* Read original register value */
    reg_value = alt_read_word(ALT_SDR_CTL_DRAMIFWIDTH_ADDR);
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_DRAMIFWIDTH_IFWIDTH,
            ALT_SDR_CTL_DRAMIFWIDTH_IFWIDTH_LSB,
            ALT_SDR_CTL_DRAMIFWIDTH_IFWIDTH_SET_MSK);
    if( sdram_write_verify((uint32_t)ALT_SDR_CTL_DRAMIFWIDTH_ADDR,  reg_value) == 1 ) {
        status = 1;
        COMPARE_FAIL_ACTION
    }
#endif


    /***** DRAMDEVWIDTH *****/
#ifdef CONFIG_HPS_SDR_CTRLCFG_DRAMDEVWIDTH_DEVWIDTH
#ifdef SDRAM_DEBUG_ENABLE
    puts("Configuring DRAMDEVWIDTH\n");
#endif
    //register_offset = SDR_CTRLGRP_DRAMDEVWIDTH_ADDRESS;
    /* Read original register value */
    reg_value = alt_read_word(ALT_SDR_CTL_DRAMDEVWIDTH_ADDR);
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_DRAMDEVWIDTH_DEVWIDTH,
            ALT_SDR_CTL_DRAMDEVWIDTH_DEVWIDTH_LSB,
            ALT_SDR_CTL_DRAMDEVWIDTH_DEVWIDTH_SET_MSK);
    if( sdram_write_verify((uint32_t)ALT_SDR_CTL_DRAMDEVWIDTH_ADDR, reg_value) == 1 ) {
        status = 1;
        COMPARE_FAIL_ACTION
    }
#endif


    /***** DRAMINTR *****/
#ifdef CONFIG_HPS_SDR_CTRLCFG_DRAMINTR_INTREN
#ifdef SDRAM_DEBUG_ENABLE
    puts("Configuring DRAMINTR\n");
#endif
    //register_offset = SDR_CTRLGRP_DRAMINTR_ADDRESS;
    /* Read original register value */
    reg_value = alt_read_word(ALT_SDR_CTL_DRAMINTR_ADDR);
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_DRAMINTR_INTREN,
            ALT_SDR_CTL_DRAMINTR_INTREN_LSB,
            ALT_SDR_CTL_DRAMINTR_INTREN_SET_MSK);
    if( sdram_write_verify((uint32_t)ALT_SDR_CTL_DRAMINTR_ADDR, reg_value) == 1 ) {
        status = 1;
        COMPARE_FAIL_ACTION
    }
#endif


    /***** STATICCFG *****/
#if defined(CONFIG_HPS_SDR_CTRLCFG_STATICCFG_MEMBL)|\
    defined(CONFIG_HPS_SDR_CTRLCFG_STATICCFG_USEECCASDATA)
#ifdef SDRAM_DEBUG_ENABLE
    puts("Configuring STATICCFG\n");
#endif
    //register_offset = SDR_CTRLGRP_STATICCFG_ADDRESS;
    /* Read original register value */
    reg_value = alt_read_word(ALT_SDR_CTL_STATICCFG_ADDR);
#ifdef CONFIG_HPS_SDR_CTRLCFG_STATICCFG_MEMBL
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_STATICCFG_MEMBL,
            ALT_SDR_CTL_STATICCFG_MEMBL_LSB,
            ALT_SDR_CTL_STATICCFG_MEMBL_SET_MSK);
#endif
#ifdef CONFIG_HPS_SDR_CTRLCFG_STATICCFG_USEECCASDATA
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_STATICCFG_USEECCASDATA,
            ALT_SDR_CTL_STATICCFG_USEECCASDATA_LSB,
            ALT_SDR_CTL_STATICCFG_USEECCASDATA_SET_MSK);
#endif
    if( sdram_write_verify((uint32_t)ALT_SDR_CTL_STATICCFG_ADDR,    reg_value) == 1 ) {
        status = 1;
        COMPARE_FAIL_ACTION
    }
#endif


    /***** CTRLWIDTH *****/
#ifdef CONFIG_HPS_SDR_CTRLCFG_CTRLWIDTH_CTRLWIDTH
#ifdef SDRAM_DEBUG_ENABLE
    puts("Configuring CTRLWIDTH\n");
#endif
    //register_offset = SDR_CTRLGRP_CTRLWIDTH_ADDRESS;
    /* Read original register value */
    reg_value = alt_read_word(ALT_SDR_CTL_CTLWIDTH_ADDR);
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_CTRLWIDTH_CTRLWIDTH,
            ALT_SDR_CTL_CTLWIDTH_CTLWIDTH_LSB,
            ALT_SDR_CTL_CTLWIDTH_CTLWIDTH_SET_MSK);
    if( sdram_write_verify((uint32_t)ALT_SDR_CTL_CTLWIDTH_ADDR, reg_value) == 1 ) {
        status = 1;
        COMPARE_FAIL_ACTION
    }
#endif


    /***** PORTCFG *****/
#ifdef CONFIG_HPS_SDR_CTRLCFG_PORTCFG_AUTOPCHEN
#ifdef SDRAM_DEBUG_ENABLE
    puts("Configuring PORTCFG\n");
#endif
    //register_offset = SDR_CTRLGRP_PORTCFG_ADDRESS;
    /* Read original register value */
    reg_value = alt_read_word(ALT_SDR_CTL_PORTCFG_ADDR);
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_PORTCFG_AUTOPCHEN,
            ALT_SDR_CTL_PORTCFG_AUTOPCHEN_LSB,
            ALT_SDR_CTL_PORTCFG_AUTOPCHEN_SET_MSK);
    if( sdram_write_verify((uint32_t)ALT_SDR_CTL_PORTCFG_ADDR,  reg_value) == 1 ) {
        status = 1;
        COMPARE_FAIL_ACTION
    }
#endif


    /***** FIFOCFG *****/
#if defined(CONFIG_HPS_SDR_CTRLCFG_FIFOCFG_SYNCMODE)|\
    defined(CONFIG_HPS_SDR_CTRLCFG_FIFOCFG_INCSYNC)
#ifdef SDRAM_DEBUG_ENABLE
    puts("Configuring FIFOCFG\n");
#endif
    //register_offset = SDR_CTRLGRP_FIFOCFG_ADDRESS;
    /* Read original register value */
    reg_value = alt_read_word(ALT_SDR_OFST+0x5088);
#ifdef CONFIG_HPS_SDR_CTRLCFG_FIFOCFG_SYNCMODE
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_FIFOCFG_SYNCMODE,
            0, //SDR_CTRLGRP_FIFOCFG_SYNCMODE_LSB,
            0x000003ff); //SDR_CTRLGRP_FIFOCFG_SYNCMODE_MASK);
#endif
#ifdef CONFIG_HPS_SDR_CTRLCFG_FIFOCFG_INCSYNC
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_FIFOCFG_INCSYNC,
            10, //SDR_CTRLGRP_FIFOCFG_INCSYNC_LSB,
            0x0400); //SDR_CTRLGRP_FIFOCFG_INCSYNC_MASK);
#endif
    if( sdram_write_verify((ALT_SDR_OFST+0x5088),   reg_value) == 1 ) {
        status = 1;
        COMPARE_FAIL_ACTION
    }
#endif


    /***** MPPRIORITY *****/
#ifdef CONFIG_HPS_SDR_CTRLCFG_MPPRIORITY_USERPRIORITY
#ifdef SDRAM_DEBUG_ENABLE
    puts("Configuring MPPRIORITY\n");
#endif
    //register_offset = SDR_CTRLGRP_MPPRIORITY_ADDRESS;
    /* Read original register value */
    reg_value = alt_read_word(ALT_SDR_CTL_MPPRIORITY_ADDR);
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_MPPRIORITY_USERPRIORITY,
            ALT_SDR_CTL_MPPRIORITY_USERPRIORITY_LSB,
            ALT_SDR_CTL_MPPRIORITY_USERPRIORITY_SET_MSK);
    if( sdram_write_verify((uint32_t)ALT_SDR_CTL_MPPRIORITY_ADDR,   reg_value) == 1 ) {
        status = 1;
        COMPARE_FAIL_ACTION
    }
#endif


    /***** MPWEIGHT_MPWEIGHT_0 *****/
#ifdef CONFIG_HPS_SDR_CTRLCFG_MPWIEIGHT_0_STATICWEIGHT_31_0
#ifdef SDRAM_DEBUG_ENABLE
    puts("Configuring MPWEIGHT_MPWEIGHT_0\n");
#endif
    //register_offset = SDR_CTRLGRP_MPWEIGHT_MPWEIGHT_0_ADDRESS;
    /* Read original register value */
    reg_value = alt_read_word(ALT_SDR_CTL_CTL_MPWEIGHT_MPWEIGHT_0_4_ADDR);
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_MPWIEIGHT_0_STATICWEIGHT_31_0,
            ALT_SDR_CTL_MPWT_MPWEIGHT_0_4_STATICWEIGHT_31_0_LSB,
            ALT_SDR_CTL_MPWT_MPWEIGHT_0_4_STATICWEIGHT_31_0_SET_MSK);
    if( sdram_write_verify((uint32_t)ALT_SDR_CTL_CTL_MPWEIGHT_MPWEIGHT_0_4_ADDR,    reg_value) == 1 ) {
        status = 1;
        COMPARE_FAIL_ACTION
    }
#endif


    /***** MPWEIGHT_MPWEIGHT_1 *****/
#if defined(CONFIG_HPS_SDR_CTRLCFG_MPWIEIGHT_1_STATICWEIGHT_49_32)|\
    defined(CONFIG_HPS_SDR_CTRLCFG_MPWIEIGHT_1_SUMOFWEIGHT_13_0)
#ifdef SDRAM_DEBUG_ENABLE
    puts("Configuring MPWEIGHT_MPWEIGHT_1\n");
#endif
    //register_offset = SDR_CTRLGRP_MPWEIGHT_MPWEIGHT_1_ADDRESS;
    /* Read original register value */
    reg_value = alt_read_word(ALT_SDR_CTL_CTL_MPWEIGHT_MPWEIGHT_1_4_ADDR);
#ifdef CONFIG_HPS_SDR_CTRLCFG_MPWIEIGHT_1_STATICWEIGHT_49_32
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_MPWIEIGHT_1_STATICWEIGHT_49_32,
            ALT_SDR_CTL_MPWT_MPWEIGHT_1_4_STATICWEIGHT_49_32_LSB,
            ALT_SDR_CTL_MPWT_MPWEIGHT_1_4_STATICWEIGHT_49_32_SET_MSK);
#endif
#ifdef CONFIG_HPS_SDR_CTRLCFG_MPWIEIGHT_1_SUMOFWEIGHT_13_0
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_MPWIEIGHT_1_SUMOFWEIGHT_13_0,
            ALT_SDR_CTL_MPWT_MPWEIGHT_1_4_SUMOFWEIGHTS_13_0_LSB,
            ALT_SDR_CTL_MPWT_MPWEIGHT_1_4_SUMOFWEIGHTS_13_0_SET_MSK);
#endif
    if( sdram_write_verify((uint32_t)ALT_SDR_CTL_CTL_MPWEIGHT_MPWEIGHT_1_4_ADDR,    reg_value) == 1 ) {
        status = 1;
        COMPARE_FAIL_ACTION
    }
#endif


    /***** MPWEIGHT_MPWEIGHT_2 *****/
#ifdef CONFIG_HPS_SDR_CTRLCFG_MPWIEIGHT_2_SUMOFWEIGHT_45_14
#ifdef SDRAM_DEBUG_ENABLE
    puts("Configuring MPWEIGHT_MPWEIGHT_2\n");
#endif
    //register_offset = SDR_CTRLGRP_MPWEIGHT_MPWEIGHT_2_ADDRESS;
    /* Read original register value */
    reg_value = alt_read_word(ALT_SDR_CTL_CTL_MPWEIGHT_MPWEIGHT_2_4_ADDR);
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_MPWIEIGHT_2_SUMOFWEIGHT_45_14,
            ALT_SDR_CTL_MPWT_MPWEIGHT_2_4_SUMOFWEIGHTS_45_14_LSB,
            ALT_SDR_CTL_MPWT_MPWEIGHT_2_4_SUMOFWEIGHTS_45_14_SET_MSK);
    if( sdram_write_verify((uint32_t)ALT_SDR_CTL_CTL_MPWEIGHT_MPWEIGHT_2_4_ADDR,    reg_value) == 1 ) {
        status = 1;
        COMPARE_FAIL_ACTION
    }
#endif


    /***** MPWEIGHT_MPWEIGHT_3 *****/
#ifdef CONFIG_HPS_SDR_CTRLCFG_MPWIEIGHT_3_SUMOFWEIGHT_63_46
#ifdef SDRAM_DEBUG_ENABLE
    puts("Configuring MPWEIGHT_MPWEIGHT_3\n");
#endif
    //register_offset = SDR_CTRLGRP_MPWEIGHT_MPWEIGHT_3_ADDRESS;
    /* Read original register value */
    reg_value = alt_read_word(ALT_SDR_CTL_CTL_MPWEIGHT_MPWEIGHT_3_4_ADDR);
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_MPWIEIGHT_3_SUMOFWEIGHT_63_46,
            ALT_SDR_CTL_MPWT_MPWEIGHT_3_4_SUMOFWEIGHTS_63_46_LSB,
            ALT_SDR_CTL_MPWT_MPWEIGHT_3_4_SUMOFWEIGHTS_63_46_SET_MSK);
    if( sdram_write_verify((uint32_t)ALT_SDR_CTL_CTL_MPWEIGHT_MPWEIGHT_3_4_ADDR,    reg_value) == 1 ) {
        status = 1;
        COMPARE_FAIL_ACTION
    }
#endif


    /***** MPPACING_MPPACING_0 *****/
#ifdef CONFIG_HPS_SDR_CTRLCFG_MPPACING_0_THRESHOLD1_31_0
#ifdef SDRAM_DEBUG_ENABLE
    puts("Configuring MPPACING_MPPACING_0\n");
#endif
    //register_offset = SDR_CTRLGRP_MPPACING_MPPACING_0_ADDRESS;
    /* Read original register value */
    reg_value = alt_read_word(ALT_SDR_OFST+0x50C0);
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_MPPACING_0_THRESHOLD1_31_0,
            0, //SDR_CTRLGRP_MPPACING_MPPACING_0_THRESHOLD1_31_0_LSB,
            0xffffffff); //SDR_CTRLGRP_MPPACING_MPPACING_0_THRESHOLD1_31_0_MASK);
    if( sdram_write_verify((ALT_SDR_OFST+0x50C0),   reg_value) == 1 ) {
        status = 1;
        COMPARE_FAIL_ACTION
    }
#endif

    /***** MPPACING_MPPACING_1 *****/
#if defined(CONFIG_HPS_SDR_CTRLCFG_MPPACING_1_THRESHOLD1_59_32)|\
    defined(CONFIG_HPS_SDR_CTRLCFG_MPPACING_1_THRESHOLD2_3_0)
#ifdef SDRAM_DEBUG_ENABLE
    puts("Configuring MPPACING_MPPACING_1\n");
#endif
    //register_offset = SDR_CTRLGRP_MPPACING_MPPACING_1_ADDRESS;
    /* Read original register value */
    reg_value = alt_read_word(ALT_SDR_OFST+0x50C4);
#ifdef CONFIG_HPS_SDR_CTRLCFG_MPPACING_1_THRESHOLD1_59_32
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_MPPACING_1_THRESHOLD1_59_32,
            0, //SDR_CTRLGRP_MPPACING_MPPACING_1_THRESHOLD1_59_32_LSB,
            0x0fffffff); // SDR_CTRLGRP_MPPACING_MPPACING_1_THRESHOLD1_59_32_MASK);
#endif
#ifdef CONFIG_HPS_SDR_CTRLCFG_MPPACING_1_THRESHOLD2_3_0
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_MPPACING_1_THRESHOLD2_3_0,
            28, //SDR_CTRLGRP_MPPACING_MPPACING_1_THRESHOLD2_3_0_LSB,
            0xf0000000); //SDR_CTRLGRP_MPPACING_MPPACING_1_THRESHOLD2_3_0_MASK);
#endif
    if( sdram_write_verify((ALT_SDR_OFST+0x50C4),   reg_value) == 1 ) {
        status = 1;
        COMPARE_FAIL_ACTION
    }
#endif

    /***** MPPACING_MPPACING_2 *****/
#ifdef CONFIG_HPS_SDR_CTRLCFG_MPPACING_2_THRESHOLD2_35_4
#ifdef SDRAM_DEBUG_ENABLE
    puts("Configuring MPPACING_MPPACING_2\n");
#endif
    //register_offset = SDR_CTRLGRP_MPPACING_MPPACING_2_ADDRESS;
    /* Read original register value */
    reg_value = alt_read_word(ALT_SDR_OFST+0x50C8);
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_MPPACING_2_THRESHOLD2_35_4,
            0, //SDR_CTRLGRP_MPPACING_MPPACING_2_THRESHOLD2_35_4_LSB,
            0xffffffff); //SDR_CTRLGRP_MPPACING_MPPACING_2_THRESHOLD2_35_4_MASK);
    if( sdram_write_verify((ALT_SDR_OFST+0x50C8),   reg_value) == 1 ) {
        status = 1;
        COMPARE_FAIL_ACTION
    }
#endif


    /***** MPPACING_MPPACING_3 *****/
#ifdef CONFIG_HPS_SDR_CTRLCFG_MPPACING_3_THRESHOLD2_59_36
#ifdef SDRAM_DEBUG_ENABLE
    puts("Configuring MPPACING_MPPACING_3\n");
#endif
    //register_offset = SDR_CTRLGRP_MPPACING_MPPACING_3_ADDRESS;
    /* Read original register value */
    reg_value = alt_read_word(ALT_SDR_OFST+0x50CC);
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_MPPACING_3_THRESHOLD2_59_36,
            0, //SDR_CTRLGRP_MPPACING_MPPACING_3_THRESHOLD2_59_36_LSB,
            0x00ffffff); //SDR_CTRLGRP_MPPACING_MPPACING_3_THRESHOLD2_59_36_MASK);
    if( sdram_write_verify((ALT_SDR_OFST+0x50CC),   reg_value) == 1 ) {
        status = 1;
        COMPARE_FAIL_ACTION
    }
#endif


    /***** MPTHRESHOLDRST_MPTHRESHOLDRST_0 *****/
#ifdef CONFIG_HPS_SDR_CTRLCFG_MPTHRESHOLDRST_0_THRESHOLDRSTCYCLES_31_0
#ifdef SDRAM_DEBUG_ENABLE
    puts("Configuring MPTHRESHOLDRST_MPTHRESHOLDRST_0\n");
#endif
    //register_offset = SDR_CTRLGRP_MPTHRESHOLDRST_MPTHRESHOLDRST_0_ADDRESS;
    /* Read original register value */
    reg_value = alt_read_word(ALT_SDR_OFST+0x50D0);
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_MPTHRESHOLDRST_0_THRESHOLDRSTCYCLES_31_0,
            0, // SDR_CTRLGRP_MPTHRESHOLDRST_MPTHRESHOLDRST_0_THRESHOLDRSTCYCLES_31_0_LSB,
            0xffffffff); //SDR_CTRLGRP_MPTHRESHOLDRST_MPTHRESHOLDRST_0_THRESHOLDRSTCYCLES_31_0_MASK);
    if( sdram_write_verify((ALT_SDR_OFST+0x50D0),   reg_value) == 1 ) {
        status = 1;
        COMPARE_FAIL_ACTION
    }
#endif


    /***** MPTHRESHOLDRST_MPTHRESHOLDRST_1 *****/
#ifdef CONFIG_HPS_SDR_CTRLCFG_MPTHRESHOLDRST_1_THRESHOLDRSTCYCLES_63_32
#ifdef SDRAM_DEBUG_ENABLE
    puts("Configuring MPTHRESHOLDRST_MPTHRESHOLDRST_1\n");
#endif
    //register_offset = SDR_CTRLGRP_MPTHRESHOLDRST_MPTHRESHOLDRST_1_ADDRESS;
    /* Read original register value */
    reg_value = alt_read_word((ALT_SDR_OFST+0x50D4));
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_MPTHRESHOLDRST_1_THRESHOLDRSTCYCLES_63_32,
            0, //SDR_CTRLGRP_MPTHRESHOLDRST_MPTHRESHOLDRST_1_THRESHOLDRSTCYCLES_63_32_LSB,
            0xffffffff); //SDR_CTRLGRP_MPTHRESHOLDRST_MPTHRESHOLDRST_1_THRESHOLDRSTCYCLES_63_32_MASK);
    if( sdram_write_verify((ALT_SDR_OFST+0x50D4),   reg_value) == 1 ) {
        status = 1;
        COMPARE_FAIL_ACTION
    }
#endif


    /***** MPTHRESHOLDRST_MPTHRESHOLDRST_2 *****/
#ifdef CONFIG_HPS_SDR_CTRLCFG_MPTHRESHOLDRST_2_THRESHOLDRSTCYCLES_79_64
#ifdef SDRAM_DEBUG_ENABLE
    puts("Configuring MPTHRESHOLDRST_MPTHRESHOLDRST_2\n");
#endif
    //register_offset = SDR_CTRLGRP_MPTHRESHOLDRST_MPTHRESHOLDRST_2_ADDRESS;
    /* Read original register value */
    reg_value = alt_read_word((ALT_SDR_OFST+0x50D8));
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_MPTHRESHOLDRST_2_THRESHOLDRSTCYCLES_79_64,
            0, //SDR_CTRLGRP_MPTHRESHOLDRST_MPTHRESHOLDRST_2_THRESHOLDRSTCYCLES_79_64_LSB,
            0x0000ffff); //SDR_CTRLGRP_MPTHRESHOLDRST_MPTHRESHOLDRST_2_THRESHOLDRSTCYCLES_79_64_MASK);
    if( sdram_write_verify((ALT_SDR_OFST+0x50D8),   reg_value) == 1 ) {
        status = 1;
        COMPARE_FAIL_ACTION
    }
#endif

    /***** PHYCTRL_PHYCTRL_0 *****/
#ifdef CONFIG_HPS_SDR_CTRLCFG_PHYCTRL_PHYCTRL_0
#ifdef SDRAM_DEBUG_ENABLE
    puts("Configuring PHYCTRL_PHYCTRL_0\n");
#endif
    //register_offset = SDR_CTRLGRP_PHYCTRL_PHYCTRL_0_ADDRESS;
    /* Read original register value */
    reg_value = CONFIG_HPS_SDR_CTRLCFG_PHYCTRL_PHYCTRL_0;
    if( sdram_write_verify((ALT_SDR_OFST+0x5150),   reg_value) == 1 ) {
        status = 1;
        COMPARE_FAIL_ACTION
    }
#endif

/* newly added registers */
    /***** CPORTWIDTH_CPORTWIDTH *****/
#ifdef CONFIG_HPS_SDR_CTRLCFG_CPORTWIDTH_CPORTWIDTH
#ifdef SDRAM_DEBUG_ENABLE
    puts("Configuring CPORTWIDTH\n");
#endif
    //register_offset = SDR_CTRLGRP_CPORTWIDTH_ADDRESS;
    /* Read original register value */
    reg_value = alt_read_word((ALT_SDR_OFST+0x5064));
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_CPORTWIDTH_CPORTWIDTH,
            0, // SDR_CTRLGRP_CPORTWIDTH_CMDPORTWIDTH_LSB,
            0x000fffff); //SDR_CTRLGRP_CPORTWIDTH_CMDPORTWIDTH_MASK);
    alt_write_word((ALT_SDR_OFST+0x5064),reg_value);
#endif

    /***** CPORTWMAP_CPORTWMAP *****/
#ifdef CONFIG_HPS_SDR_CTRLCFG_CPORTWMAP_CPORTWMAP
#ifdef SDRAM_DEBUG_ENABLE
    puts("Configuring CPORTWMAP\n");
#endif
    //register_offset = SDR_CTRLGRP_CPORTWMAP_ADDRESS;
    /* Read original register value */
    reg_value = alt_read_word(ALT_SDR_OFST+0x5068);
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_CPORTWMAP_CPORTWMAP,
            0, //SDR_CTRLGRP_CPORTWMAP_CPORTWFIFOMAP_LSB,
            0x3fffffff); //SDR_CTRLGRP_CPORTWMAP_CPORTWFIFOMAP_MASK);
    alt_write_word((ALT_SDR_OFST+0x5068), reg_value);
#endif

    /***** CPORTRMAP_CPORTRMAP *****/
#ifdef CONFIG_HPS_SDR_CTRLCFG_CPORTRMAP_CPORTRMAP
#ifdef SDRAM_DEBUG_ENABLE
    puts("Configuring CPORTRMAP\n");
#endif
    //register_offset = SDR_CTRLGRP_CPORTRMAP_ADDRESS;
    /* Read original register value */
    reg_value = alt_read_word(ALT_SDR_OFST+0x506C);
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_CPORTRMAP_CPORTRMAP,
            0, //SDR_CTRLGRP_CPORTRMAP_CPORTRFIFOMAP_LSB,
            0x3fffffff); //SDR_CTRLGRP_CPORTRMAP_CPORTRFIFOMAP_MASK);
    alt_write_word((ALT_SDR_OFST+0x506C), reg_value);
#endif

    /***** RFIFOCMAP_RFIFOCMAP *****/
#ifdef CONFIG_HPS_SDR_CTRLCFG_RFIFOCMAP_RFIFOCMAP
#ifdef SDRAM_DEBUG_ENABLE
    puts("Configuring RFIFOCMAP\n");
#endif
    //register_offset = SDR_CTRLGRP_RFIFOCMAP_ADDRESS;
    /* Read original register value */
    reg_value = alt_read_word(ALT_SDR_OFST+0x5070);
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_RFIFOCMAP_RFIFOCMAP,
            0, //SDR_CTRLGRP_RFIFOCMAP_RFIFOCPORTMAP_LSB,
            0x00ffffff); //SDR_CTRLGRP_RFIFOCMAP_RFIFOCPORTMAP_MASK);
    alt_write_word((ALT_SDR_OFST+0x5070), reg_value);
#endif

    /***** WFIFOCMAP_WFIFOCMAP *****/
#ifdef CONFIG_HPS_SDR_CTRLCFG_WFIFOCMAP_WFIFOCMAP
#ifdef SDRAM_DEBUG_ENABLE
    puts("Configuring WFIFOCMAP\n");
#endif
    //register_offset = SDR_CTRLGRP_WFIFOCMAP_ADDRESS;
    /* Read original register value */
    reg_value = alt_read_word(ALT_SDR_OFST+0x5074);
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_WFIFOCMAP_WFIFOCMAP,
            0, // SDR_CTRLGRP_WFIFOCMAP_WFIFOCPORTMAP_LSB,
            0x00ffffff); //SDR_CTRLGRP_WFIFOCMAP_WFIFOCPORTMAP_MASK);
    alt_write_word((ALT_SDR_OFST+0x5074), reg_value);
#endif

    /***** CPORTRDWR_CPORTRDWR *****/
#ifdef CONFIG_HPS_SDR_CTRLCFG_CPORTRDWR_CPORTRDWR
#ifdef SDRAM_DEBUG_ENABLE
    puts("Configuring CPORTRDWR\n");
#endif
    // register_offset = SDR_CTRLGRP_CPORTRDWR_ADDRESS;
    /* Read original register value */
    reg_value = alt_read_word(ALT_SDR_OFST+0x5078);
    reg_value = sdram_write_register_field( reg_value,
            CONFIG_HPS_SDR_CTRLCFG_CPORTRDWR_CPORTRDWR,
            0, //SDR_CTRLGRP_CPORTRDWR_CPORTRDWR_LSB,
            0x000fffff); //SDR_CTRLGRP_CPORTRDWR_CPORTRDWR_MASK);
    alt_write_word((ALT_SDR_OFST+0x5078), reg_value);
#endif

    /***** DRAMODT  *****/
#if defined(CONFIG_HPS_SDR_CTRLCFG_DRAMODT_READ) || defined(CONFIG_HPS_SDR_CTRLCFG_DRAMODT_WRITE)
    reg_value = 0;
#ifdef CONFIG_HPS_SDR_CTRLCFG_DRAMODT_READ
    reg_value |= ALT_SDR_CTL_DRAMODT_CFG_RD_ODT_CHIP_SET(CONFIG_HPS_SDR_CTRLCFG_DRAMODT_READ);
#endif
#ifdef CONFIG_HPS_SDR_CTRLCFG_DRAMODT_WRITE
    reg_value |= ALT_SDR_CTL_DRAMODT_CFG_WR_ODT_CHIP_SET(CONFIG_HPS_SDR_CTRLCFG_DRAMODT_WRITE);
#endif
    alt_write_word(ALT_SDR_CTL_DRAMODT_ADDR, reg_value);
#endif

/* end of newly added registers */
    
/***** Final step - apply configuration changes *****/
#ifdef SDRAM_DEBUG_ENABLE
//  puts("Applying configuration changes\n");
#endif
    //register_offset = SDR_CTRLGRP_STATICCFG_ADDRESS;
    /* Read original register value */
    reg_value = alt_read_word(ALT_SDR_CTL_STATICCFG_ADDR);
    reg_value = sdram_write_register_field( reg_value, 1,
            ALT_SDR_CTL_STATICCFG_APPLYCFG_LSB,
            ALT_SDR_CTL_STATICCFG_APPLYCFG_SET_MSK);
    alt_write_word(ALT_SDR_CTL_STATICCFG_ADDR, reg_value);


#if defined(CONFIG_HPS_SDR_CTRLCFG_DRAMADDRW_CSBITS)\
  & defined(CONFIG_HPS_SDR_CTRLCFG_DRAMADDRW_BANKBITS)\
  & defined(CONFIG_HPS_SDR_CTRLCFG_DRAMADDRW_ROWBITS)\
  & defined(CONFIG_HPS_SDR_CTRLCFG_DRAMADDRW_COLBITS)\
  & defined(CONFIG_HPS_SDR_CTRLCFG_CTRLWIDTH_CTRLWIDTH)

    /* FBz 223741: Access to non-existant SDRAM.
     * The SDRAM controller is programmed to use
     * the full 4GB address space, and a protection
     * rule is used to abort non-existant memory space.
     */

    /* Clear port defaults and prepare rule 0 */
    alt_write_word(ALT_SDR_CTL_PROTPORTDEFAULT_ADDR,0);
    alt_write_word(ALT_SDR_CTL_PROTRULERDWR_ADDR,0);

    /* Create valid memory window */
    alt_write_word(ALT_SDR_CTL_PROTRULEADDR_ADDR,((sdram_size()-1)>>10)<<12);

    /* All ID values are valid */
    alt_write_word(ALT_SDR_CTL_PROTRULEID_ADDR,0xfff<<12);

    /* Applies to all ports and security types */
    alt_write_word(ALT_SDR_CTL_PROTRULEDATA_ADDR,0x1fff);

    /* Write Protection Rule */
    alt_write_word(ALT_SDR_CTL_PROTRULERDWR_ADDR,1<<5);
    alt_write_word(ALT_SDR_CTL_PROTRULERDWR_ADDR,0);

    /* All ports reject transactions by default */
    alt_write_word(ALT_SDR_CTL_PROTPORTDEFAULT_ADDR,0x3ff);

#endif

    return status;
}


/* Calculates the size of DRAM using
 * configuration parameters from QSys.
 *
 * Returns size in 1KB blocks,
 * or 0 if parameters are missing.
 */
uint32_t sdram_size() {

#if defined(CONFIG_HPS_SDR_CTRLCFG_DRAMADDRW_CSBITS)\
  & defined(CONFIG_HPS_SDR_CTRLCFG_DRAMADDRW_BANKBITS)\
  & defined(CONFIG_HPS_SDR_CTRLCFG_DRAMADDRW_ROWBITS)\
  & defined(CONFIG_HPS_SDR_CTRLCFG_DRAMADDRW_COLBITS)\
  & defined(CONFIG_HPS_SDR_CTRLCFG_CTRLWIDTH_CTRLWIDTH)

    /* Add the Bank, Row, and Column bits to get addresses per-chip.*/
    uint32_t value = 1 << (CONFIG_HPS_SDR_CTRLCFG_DRAMADDRW_BANKBITS
                         + CONFIG_HPS_SDR_CTRLCFG_DRAMADDRW_ROWBITS
                         + CONFIG_HPS_SDR_CTRLCFG_DRAMADDRW_COLBITS);

    /* Multiply by the number of chips and divide by 1K */
    value = value * CONFIG_HPS_SDR_CTRLCFG_DRAMADDRW_CSBITS / 1024;

    /* Add number of bytes returned per address */
    value = value << CONFIG_HPS_SDR_CTRLCFG_CTRLWIDTH_CTRLWIDTH;

    return value;
#else
    return 0;
#endif
}


/* DMA Programs for SDRAM Clearing. */
#if (CONFIG_PRELOADER_SDRAM_SCRUBBING==1)
static ALT_DMA_PROGRAM_t dmach0;
static ALT_DMA_PROGRAM_t dmach1;
#endif


/* Use DMA to clear SDRAM address space.
 *
 * According to CONFIG_PRELOADER_SDRAM_SCRUB*
 * parameters, use DMA channels 0 and 1 to clear
 * a small window and possibly all addressable
 * SDRAM space.
 */
int sdram_ecc_clear() {
#if (CONFIG_PRELOADER_SDRAM_SCRUBBING == 1)

    ALT_STATUS_CODE rv;
    ALT_DMA_CHANNEL_STATE_t state;

    uint32_t i;

    /* dma config */
    ALT_DMA_CFG_t dma_cfg = {
      .manager_sec = ALT_DMA_SECURITY_DEFAULT,
      .irq_sec = {0,0,0,0,0,0,0,0},
      .periph_sec = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                     0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
      .periph_mux = {0,0,0,0},
    };

    /* init dma */
    rv = alt_dma_init(&dma_cfg);
    if (rv!=ALT_E_SUCCESS) return rv;

    /* allocate 2 channels */
    rv = alt_dma_channel_alloc(ALT_DMA_CHANNEL_0);
    if (rv!=ALT_E_SUCCESS) return rv;
    rv = alt_dma_channel_alloc(ALT_DMA_CHANNEL_1);
    if (rv!=ALT_E_SUCCESS) return rv;

    /* clear boot region with DMA */
    rv = alt_dma_zero_to_memory(ALT_DMA_CHANNEL_0, &dmach0,
                      (void*)CONFIG_PRELOADER_SDRAM_SCRUB_BOOT_REGION_START,
                      (  CONFIG_PRELOADER_SDRAM_SCRUB_BOOT_REGION_END
                       - CONFIG_PRELOADER_SDRAM_SCRUB_BOOT_REGION_START),
                      false, ALT_DMA_EVENT_0);

    /* check error */
    if (rv!=ALT_E_SUCCESS) return rv;

    /* polling loop */
    for (i=0xFFFFFF;i>0;i--) {

        /* get DMA state */
        rv = alt_dma_channel_state_get(ALT_DMA_CHANNEL_0, &state);
        if (rv!=ALT_E_SUCCESS) return rv;

        /* check DMA state is stopped */
        if (state==ALT_DMA_CHANNEL_STATE_STOPPED) break;
    }

    /* timeout */
    if (i==0) return ALT_E_TMO;

#if (CONFIG_PRELOADER_SDRAM_SCRUB_REMAIN_REGION==1)

    /* get SDRAM size in KB */
    uint32_t size = sdram_size();
    if (size==0) return ALT_E_ERROR;

    /* Convert size to bytes.
     * DMA can only address up to 2GB of SDRAM.
     * Limit the address region to 2GB.
     */
    if (size>(2<<20)) size = 2UL<<30;
    else              size = size<<10;

    /* clear lower region with DMA 0 */
    rv = alt_dma_zero_to_memory(ALT_DMA_CHANNEL_0, &dmach0,
                      (void*)0x0000,
                      CONFIG_PRELOADER_SDRAM_SCRUB_BOOT_REGION_START,
                      false, ALT_DMA_EVENT_0);

    /* check error */
    if (rv!=ALT_E_SUCCESS) return rv;

    /* clear upper region with DMA 1 */
    rv = alt_dma_zero_to_memory(ALT_DMA_CHANNEL_1, &dmach1,
                      (void*)CONFIG_PRELOADER_SDRAM_SCRUB_BOOT_REGION_END,
                      (size - CONFIG_PRELOADER_SDRAM_SCRUB_BOOT_REGION_END),
                      false, ALT_DMA_EVENT_0);

    /* check error */
    if (rv!=ALT_E_SUCCESS) return rv;

#endif
#endif
    return ALT_E_SUCCESS;
}


/* Wait for sdram_ecc_clear() to finish.
 *
 * Polls DMA 0 and 1 until they are both stopped.
 * This could block for a few seconds.
 */
int sdram_ecc_clear_wait() {

#if (CONFIG_PRELOADER_SDRAM_SCRUBBING==1)
    ALT_STATUS_CODE rv;
 #if (CONFIG_PRELOADER_SDRAM_SCRUB_REMAIN_REGION==1)

    uint32_t i;
    ALT_DMA_CHANNEL_STATE_t state;

    /* polling loop */
    for (i=0xFFFFFF;i>0;i--) {

        /* get DMA state */
        rv = alt_dma_channel_state_get(ALT_DMA_CHANNEL_0, &state);
        if (rv!=ALT_E_SUCCESS) return rv;

        /* check DMA state is stopped */
        if (state==ALT_DMA_CHANNEL_STATE_STOPPED) break;
    }

    /* timeout */
    if (i==0) return ALT_E_TMO;

    MPL_WATCHDOG();

    /* polling loop */
    for (i=0xFFFFFF;i>0;i--) {

        /* get DMA state */
        rv = alt_dma_channel_state_get(ALT_DMA_CHANNEL_1, &state);
        if (rv!=ALT_E_SUCCESS) return rv;

        /* check DMA state is stopped */
        if (state==ALT_DMA_CHANNEL_STATE_STOPPED) break;
    }

    /* timeout */
    if (i==0) return ALT_E_TMO;

 #endif

    /* free all channels */
    rv = alt_dma_uninit();
    if (rv!=ALT_E_SUCCESS) return rv;

#endif
    return 0;
}

