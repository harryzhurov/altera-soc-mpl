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
 * $Id: //acds/rel/16.1/embedded/examples/software/Altera-SoCFPGA-HardwareLib-MPL/core/sdmmc_init.c#1 $
 */

// include socal headers
#include <socal/socal.h>
#include "alt_printf.h"
#include "alt_sdmmc.h"
#include "header.h"
#include "mpl_config.h"
#include "mpl_common.h"
#include "sdmmc_common.h"

// include passdown info for next image location
#include "build.h"

ALT_SDMMC_CARD_INFO_t card_info;

ALT_STATUS_CODE sdmmc_initialize()
{
    static bool sdmmc_initialized = false;
    ALT_STATUS_CODE status = ALT_E_SUCCESS;

    if(sdmmc_initialized)
    {
       return status;
    }
    sdmmc_initialized = true;

    //
    // Initialize SDMMC
    //

    if (status == ALT_E_SUCCESS)
    {
        status = alt_sdmmc_init();
    }

    if (status == ALT_E_SUCCESS)
    {
        status = alt_sdmmc_card_pwr_on();
    }

    if (status == ALT_E_SUCCESS)
    {
        status = alt_sdmmc_card_identify(&card_info);
    }

    if (status == ALT_E_SUCCESS)
    {
        status = alt_sdmmc_card_bus_width_set(&card_info, ALT_SDMMC_BUS_WIDTH_4);
    }

    if (status == ALT_E_SUCCESS)
    {
        status = alt_sdmmc_fifo_param_set((ALT_SDMMC_FIFO_NUM_ENTRIES >> 3) - 1,
                                          (ALT_SDMMC_FIFO_NUM_ENTRIES >> 3), (ALT_SDMMC_MULT_TRANS_t)0);
    }

    if (status == ALT_E_SUCCESS)
    {
        status = alt_sdmmc_dma_enable();
    }

    if (status == ALT_E_SUCCESS)
    {
        uint32_t speed = card_info.xfer_speed;
        if (card_info.high_speed)
        {
            speed *= 2;
        }
        
        status = alt_sdmmc_card_speed_set(&card_info, speed);
    }

    MPL_WATCHDOG();

    return status;
}

