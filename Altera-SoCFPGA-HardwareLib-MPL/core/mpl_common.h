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
 * $Id: //acds/rel/16.1/embedded/examples/software/Altera-SoCFPGA-HardwareLib-MPL/core/mpl_common.h#1 $
 */

/*! \file
 *  Altera - MPL Common Header definitions
 */

#ifndef _MPL_COMMON_H
#define _MPL_COMMON_H

#include "build.h"

#if CONFIG_PRELOADER_WATCHDOG_ENABLE
#include "alt_watchdog.h"
#define MPL_WATCHDOG()    alt_wdog_reset(ALT_WDOG0)
#else
#define MPL_WATCHDOG()
#endif

//
// Structure which contains all relevant image information
//
typedef struct launch_info
{
    uint32_t entry_addr; // Entry point of next stage.
    
    uint32_t image_addr; // Image memory address
    uint32_t image_size; // Image size
#if CONFIG_PRELOADER_CHECKSUM_NEXT_IMAGE
    uint32_t image_crc;  // Image CRC checksum
#endif

#if CONFIG_MPL_FPGA_LOAD
    uint32_t fpga_addr; // FPGA memory address
    uint32_t fpga_size; // FPGA size
#if CONFIG_PRELOADER_CHECKSUM_NEXT_IMAGE
    uint32_t fpga_crc;  // FPGA CRC checksum
#endif
#endif
    
} launch_info_t;

//
// Function pointer to launch the next stage.
//
typedef void (*FXN_PTR)(void);

//
// Function to configure the FPGA2SDRAM registers. Function definition in startup.S.
//
void sdram_applycfg_ocram(void);

#endif
