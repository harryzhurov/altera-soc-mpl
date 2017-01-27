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
 * $Id: //acds/rel/16.1/embedded/examples/software/Altera-SoCFPGA-HardwareLib-MPL/core/board_init.c#1 $
 */

#include <stdio.h>
#include <string.h> // memcpy
// include socal headers
#include <socal/socal.h>
#include <socal/hps.h>
#include <socal/alt_rstmgr.h>
#include <socal/alt_clkmgr.h>
#include <socal/alt_l3.h>
#include <socal/alt_sysmgr.h>
// include needed hwlib headers
#include <hwlib.h>
#include <alt_timers.h>
#include <alt_clock_manager.h>

// include altera_ip headers
#include <freeze_controller.h>
#include <scan_manager.h>
#include <sdram.h>

#include <mpl_common.h>
#include <mpl_config.h>

// passdown file headers to include
#include <sdram/sdram_config.h>  // SDRAM configuration
#include <sequencer_defines.h>  // SDRAM calibration files
#include <pinmux_config.h>  // Pin muxing configuration
#include <pll_config.h>     // PLL configuration

#ifdef CONFIG_SOCFPGA_ARRIA5
#include <iocsr_config_arria5.h>    // IO bank configuration
#else
#include <iocsr_config_cyclone5.h>  // IO bank configuration
#endif

#include <build.h>  // Build and miscellaneous settings



// utilities macros
#define CLKMGR_SDRPLLGRP_VCO_SSRC_GET(x)        (((x) & 0x00c00000) >> 22)
#define CLKMGR_MAINPLLGRP_VCO_DENOM_GET(x)      (((x) & 0x003f0000) >> 16)
#define CLKMGR_SDRPLLGRP_VCO_NUMER_GET(x)       (((x) & 0x0000fff8) >> 3)
#define CLKMGR_SDRPLLGRP_DDRDQSCLK_CNT_GET(x)   (((x) & 0x000001ff) >> 0)
#define CLKMGR_VCO_SSRC_EOSC1           0x0
#define CLKMGR_VCO_SSRC_EOSC2           0x1
#define CLKMGR_VCO_SSRC_F2S             0x2
#define CLKMGR_SDRPLLGRP_VCO_DENOM_GET(x)       (((x) & 0x003f0000) >> 16)
#define CLKMGR_SDRPLLGRP_VCO_NUMER_GET(x)       (((x) & 0x0000fff8) >> 3)
#define CLKMGR_MAINPLLGRP_VCO_DENOM_GET(x)      (((x) & 0x003f0000) >> 16)
#define CLKMGR_MAINPLLGRP_VCO_NUMER_GET(x)      (((x) & 0x0000fff8) >> 3)


// function and variable prototypes
ALT_STATUS_CODE basic_clocks_init(void);  // defined below
void sysmgr_pinmux_init(void); // defined in system_manager_pinmux.c
// for use in scan manager --- defined in iocsr_config_"board" source file
extern const unsigned long iocsr_scan_chain0_table[((CONFIG_HPS_IOCSR_SCANCHAIN0_LENGTH / 32) + 1)];
extern const unsigned long iocsr_scan_chain1_table[((CONFIG_HPS_IOCSR_SCANCHAIN1_LENGTH / 32) + 1)];
extern const unsigned long iocsr_scan_chain2_table[((CONFIG_HPS_IOCSR_SCANCHAIN2_LENGTH / 32) + 1)];
extern const unsigned long iocsr_scan_chain3_table[((CONFIG_HPS_IOCSR_SCANCHAIN3_LENGTH / 32) + 1)];

// defined below
void print_clock_info(void);
void ram_boot_setup(void);



// boart init function is responsible for doing the board/chip init
// configure re-map, reset_manager, clock manager (pll), and I/Os
// also init and calibrate sdram
ALT_STATUS_CODE board_init(void)
{
    ALT_STATUS_CODE ret;
    // board init sequence from the MPL_Boot_Flow
    // Re-setup L4 watchdog
    // if WD not enabled assert WD reset to disable it.
#if (CONFIG_PRELOADER_WATCHDOG_ENABLE == 0)
    alt_setbits_word(ALT_RSTMGR_PERMODRST_ADDR, ALT_RSTMGR_PERMODRST_L4WD0_SET_MSK);
#endif

    MPL_WATCHDOG();

    // ensure all IO banks are in frozen state via Freeze Controller
    sys_mgr_frzctrl_freeze_req(FREEZE_CHANNEL_0, FREEZE_CONTROLLER_FSM_SW);
    sys_mgr_frzctrl_freeze_req(FREEZE_CHANNEL_1, FREEZE_CONTROLLER_FSM_SW);
    sys_mgr_frzctrl_freeze_req(FREEZE_CHANNEL_2, FREEZE_CONTROLLER_FSM_SW);
    sys_mgr_frzctrl_freeze_req(FREEZE_CHANNEL_3, FREEZE_CONTROLLER_FSM_SW);

    // Assert reset to peripherals except L4wd0 and all bridges
    alt_write_word(ALT_RSTMGR_PERMODRST_ADDR, ALT_RSTMGR_PERMODRST_L4WD0_CLR_MSK);
    //alt_write_word(ALT_RSTMGR_BRGMODRST_ADDR, 0x7);

    // Enable and init OSC1 Timer0 (required during PLL-config)
    alt_clrbits_word(ALT_RSTMGR_PERMODRST_ADDR, ALT_RSTMGR_PERMODRST_OSC1TMR0_SET_MSK);  // release reset bit
    ret = alt_gpt_mode_set(ALT_GPT_OSC1_TMR0, ALT_GPT_RESTART_MODE_PERIODIC); // set-mode to restart
    if (ret != ALT_E_SUCCESS){ return ret;}
    ret = alt_gpt_counter_set(ALT_GPT_OSC1_TMR0, 0xFFFFFFFF); // set loadreg value to max
    if (ret != ALT_E_SUCCESS){ return ret;}
    ret = alt_gpt_tmr_start(ALT_GPT_OSC1_TMR0); // start timer
    if (ret != ALT_E_SUCCESS){ return ret;}

    alt_clk_ext_clk_freq_set(ALT_CLK_OSC1, CONFIG_HPS_CLK_OSC1_HZ);

    MPL_WATCHDOG();

    // Set up the PLL RAM Boot fix, if needed
    ram_boot_setup();

    // Re-configuration of all PLLs
    ret = alt_clk_clkmgr_init();
    if (ret != ALT_E_SUCCESS){ return ret;}

    // init Scan Manager to configure HPS IOCSR
    ret = scan_mgr_io_scan_chain_prg(IO_SCAN_CHAIN_0, CONFIG_HPS_IOCSR_SCANCHAIN0_LENGTH, iocsr_scan_chain0_table);
    if (ret != ALT_E_SUCCESS){ return ret;}
    ret = scan_mgr_io_scan_chain_prg(IO_SCAN_CHAIN_1, CONFIG_HPS_IOCSR_SCANCHAIN1_LENGTH, iocsr_scan_chain1_table);
    if (ret != ALT_E_SUCCESS){ return ret;}
    ret = scan_mgr_io_scan_chain_prg(IO_SCAN_CHAIN_2, CONFIG_HPS_IOCSR_SCANCHAIN2_LENGTH, iocsr_scan_chain2_table);
    if (ret != ALT_E_SUCCESS){ return ret;}
    ret = scan_mgr_io_scan_chain_prg(IO_SCAN_CHAIN_3, CONFIG_HPS_IOCSR_SCANCHAIN3_LENGTH, iocsr_scan_chain3_table);
    if (ret != ALT_E_SUCCESS){ return ret;}

    MPL_WATCHDOG();
    // config pin mux via system manager
    sysmgr_pinmux_init();

    // thaw (un-freeze) all IO banks
    sys_mgr_frzctrl_thaw_req(FREEZE_CHANNEL_0, FREEZE_CONTROLLER_FSM_SW);
    sys_mgr_frzctrl_thaw_req(FREEZE_CHANNEL_1, FREEZE_CONTROLLER_FSM_SW);
    sys_mgr_frzctrl_thaw_req(FREEZE_CHANNEL_2, FREEZE_CONTROLLER_FSM_SW);
    sys_mgr_frzctrl_thaw_req(FREEZE_CHANNEL_3, FREEZE_CONTROLLER_FSM_SW);

    MPL_WATCHDOG();

    ALT_PRINTF("INIT: MPL build: " __DATE__ " " __TIME__ "\n");

    // console init (if serial support enabled)
    ALT_PRINTF("INIT: Initializing board.\n");

    // SDRAM init and calibration
    // sdram mmr init
    ret = altr_sdram_mmr_init_full();
    if (ret != ALT_E_SUCCESS){ return ret;}
    // sdram calibration
    ret = altr_sdram_calibration_full();
    if (ret != ALT_E_SUCCESS){ return ret;}

    MPL_WATCHDOG();

    // Interconnect config
    alt_write_word(ALT_L3_SEC_LWH2F_ADDR, 0x1);  // allow sec + non-sec accesses to lw-bridge
    alt_write_word(ALT_L3_SEC_H2F_ADDR, 0x1);  // allow sec + non-sec accesses to h2f-bridge
    alt_write_word(ALT_L3_SEC_ACP_ADDR, 0x1);  // allow sec + non-sec accesses to acp port
    alt_write_word(ALT_L3_SEC_ROM_ADDR, 0x1);  // allow sec + non-sec accesses to ROM
    alt_write_word(ALT_L3_SEC_OCRAM_ADDR, 0x1);  // allow sec + non-sec accesses to OCRAM
    alt_write_word(ALT_L3_SEC_SDRDATA_ADDR, 0x1);  // allow sec + non-sec accesses to SDRAM
#ifdef BOOT_FROM_FPGA
    alt_write_word(ALT_L3_REMAP_ADDR, 0x18 | 0x1); // remap 0x0 to OCRAM, Maintain FPGA connection
#else
    alt_write_word(ALT_L3_REMAP_ADDR, 0x1); // remap 0x0 to OCRAM
#endif

#if (CONFIG_PRELOADER_SDRAM_SCRUBBING==1)

    /* Run SDRAM clearing */
    ret = sdram_ecc_clear();
    if (ret != ALT_E_SUCCESS) { return ret; }

    MPL_WATCHDOG();
#endif

    print_clock_info();

    ALT_PRINTF("INIT: Initializing successful.\n");
    return ret;
}

void print_clock_info(void)
{
    alt_freq_t freq;
    alt_clk_freq_get(ALT_CLK_MPU, &freq);
    ALT_PRINTF("INIT: MPU clock = %ld MHz\n", (freq / 1000000));
    alt_clk_freq_get(ALT_CLK_DDR_DQS, &freq);
    ALT_PRINTF("INIT: DDR clock = %ld MHz\n", (freq / 1000000));
}

/*
 * Setup RAM boot to ensure the clock are reset under CSEL = 0
 */
extern uint32_t reset_clock_manager_size;
extern void reset_clock_manager(void);

#define CONFIG_SYSMGR_WARMRAMGRP_ENABLE_MAGIC   0xae9efebc

/*
 * A future version of the bsp-editor will include a new value that tells
 * the code to include the warm reset PLL fix.  This fix is required under certain
 * conditions if warm and cold reset are not tied together.
 *
 * For more information,see the errata that describes this fix:
 *
 *   For Cyclone V: http://www.altera.com/literature/es/es_cyclone_v.pdf
 *   For Arria V:   http://www.altera.com/literature/es/es_arria_V.pdf
 *
 * Include the define here until it is in the handoff information and
 * by default, enable the warm boot fix.
 */

#ifndef CONFIG_PRELOADER_RAMBOOT_PLLRESET
#define CONFIG_PRELOADER_RAMBOOT_PLLRESET 1
#endif

void ram_boot_setup(void)
{
    /* We need the fix if we have been asked to put it in and we are not
     * booting from the FPGA and we are using CSEL_0 */

#if ((CONFIG_PRELOADER_RAMBOOT_PLLRESET == 1) && (CONFIG_PRELOADER_EXE_ON_FPGA == 0))

    unsigned csel, ramboot_addr;
    unsigned tmp_daddr;
    unsigned tmp_saddr;
    unsigned int bootinfo;

    bootinfo = alt_read_word(ALT_SYSMGR_BOOT_ADDR);

    csel = (bootinfo & ALT_SYSMGR_BOOT_CSEL_SET_MSK) >> ALT_SYSMGR_BOOT_CSEL_LSB;

    if (csel == ALT_SYSMGR_BOOT_CSEL_E_CSEL_0)
    {
        /*
         * Copy the ramboot program at offset 60 KiB in the on-chip RAM.
         * The copied program must be less than 4 KiB in size.
	 * The customer can modify any content within the first
	 * 60kB of on-chip RAM at any boot stage.
         */

        ramboot_addr = (uint32_t)ALT_OCRAM_ADDR + 0xf000;

        tmp_daddr=(unsigned)ramboot_addr;
        tmp_saddr=(unsigned)reset_clock_manager;
        memcpy((void*)tmp_daddr, (void*)tmp_saddr, reset_clock_manager_size);

        /* tell BootROM where the RAM boot code starts */
        alt_write_word(ALT_SYSMGR_ROMCODE_ROMCODE_WARMRAM_EXECUTION_ADDR, ramboot_addr & ALT_SYSMGR_ROMCODE_WARMRAM_EXECUTION_OFFSET_SET_MSK);

        /* Enable RAM boot */
        alt_write_word(ALT_SYSMGR_ROMCODE_ROMCODE_WARMRAM_EN_ADDR, CONFIG_SYSMGR_WARMRAMGRP_ENABLE_MAGIC);
    }

#endif /* PLLRESET fix is enabled and system did not boot from the FPGA */

}
