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
 * $Id: //acds/rel/16.1/embedded/examples/software/Altera-SoCFPGA-HardwareLib-MPL/core/board_init_a10.c#3 $
 */

#include <stdio.h>
#include <string.h> // memcpy
// include socal headers
#include <socal/socal.h>
#include <socal/hps.h>
#include <socal/alt_rstmgr.h>
#include <socal/alt_clkmgr.h>
#include <socal/alt_sysmgr.h>
// include needed hwlib headers
#include <hwlib.h>
#include <alt_timers.h>
#include <alt_clock_manager.h>

// include altera_ip headers
#include <mpl_common.h>
#include <mpl_config.h>
#include <build.h>  // Build and miscellaneous settings
#include <alt_noc_fw_ocram_scr.h>

extern int altr_sdram_calibration_full(void);
void get_clockdata(CLOCK_SOURCE_CONFIG *psrc, CLOCK_MANAGER_CONFIG *pMgr);

// function and variable prototypes
void sysmgr_pinmux_init(void); // defined in system_manager_pinmux.c

// defined below
void print_clock_info(void);

extern char *user_stack_end;

// boart init function is responsible for doing the board/chip init
// configure re-map, reset_manager, clock manager (pll), and I/Os
// also init and calibrate sdram
ALT_STATUS_CODE board_init(void)
{
    ALT_STATUS_CODE ret;
    CLOCK_MANAGER_CONFIG  cfg;
    CLOCK_SOURCE_CONFIG inclks;

    MPL_WATCHDOG();

    // Need to call the parsing code to get cfg, inclks
    get_clockdata( &inclks, &cfg);

    // Re-configuration of all PLLs
    ret = alt_clkmgr_config(&cfg, &inclks);
    if (ret != ALT_E_SUCCESS)
      { return ret;}

    MPL_WATCHDOG();
    // config pin mux via system manager
    sysmgr_pinmux_init();

    MPL_WATCHDOG();

    // Mark the OCRAM as non-secure so that the qspi/sdmmc/fpga can write to it

    alt_write_word(ALT_NOC_FW_OCRAM_SCR_REG0ADDR_ADDR, 
                     ALT_NOC_FW_OCRAM_SCR_REG0ADDR_BASE_SET(0) |
                     ALT_NOC_FW_OCRAM_SCR_REG0ADDR_LIMIT_SET(0x3f));

    alt_write_word(ALT_NOC_FW_OCRAM_SCR_EN_ADDR,
                     ALT_NOC_FW_OCRAM_SCR_EN_REG0EN_SET_MSK);

    ALT_PRINTF("INIT: MPL build: " __DATE__ " " __TIME__ "\n");

    // console init (if serial support enabled)
    ALT_PRINTF("INIT: Initializing board.\n");    

    return ALT_E_SUCCESS;
}
ALT_STATUS_CODE sdram_init(void)
{
    ALT_STATUS_CODE ret;

    print_clock_info();
    // SDRAM init and calibration
    // sdram calibration
    ret = altr_sdram_calibration_full();
    if (ret != ALT_E_SUCCESS){ return ret;}
    MPL_WATCHDOG();

    ALT_PRINTF("INIT: Initializing successful.\n");
    return ret;
}

void print_clock_info(void)
{
    alt_freq_t freq;
    alt_clk_freq_get(ALT_CLK_MPU, &freq);
    ALT_PRINTF("INIT: MPU clock = %ld MHz\n", (freq / 1000000));

    #ifdef SDMMC_BOOT
        alt_clk_freq_get(ALT_CLK_SDMMC, &freq);
        ALT_PRINTF("INIT: SDMMC clock = %ld MHz\n", (freq / 1000000));
    #elif QSPI_BOOT
        alt_clk_freq_get(ALT_CLK_QSPI, &freq);
        ALT_PRINTF("INIT: QSPI clock = %ld MHz\n", (freq / 1000000));
    #endif
}

