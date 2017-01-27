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
 * $Id: //acds/rel/16.1/embedded/examples/software/Altera-SoCFPGA-HardwareLib-MPL/core/sdmmc_load_fpga.c#5 $
 */

// include socal headers
#include <socal/socal.h>
#include "alt_printf.h"
#include "alt_sdmmc.h"
#include "header.h"
#include "mpl_config.h"
#include "mpl_common.h"
#include "sdmmc_common.h"
#include "alt_fpga_manager.h"

// include passdown info for next image location
#include "build.h"

#define SDMMC_BLOCK_SZ	512
#define SDMMC_BLK_ALIGN_MSK 0xFFFFFE00

// This is the largest chunk read before handling the watchdog.
#define SDMMC_CHUNK_SZ  (512 * 1024)


/* Information shared between sdmmc_load_next and sdmmc_load_fpga */
uint32_t offset, total_loaded;
uint64_t sd_addr_copy;
char sdmmc_buf[SDMMC_BLOCK_SZ];
uint32_t file_size_copy;

/* Defined in sdmmc_load.c */
extern ALT_SDMMC_CARD_INFO_t card_info;
ALT_STATUS_CODE sdmmc_initialize(void);

/*#define DIRECT_TO_BUFFER*/

int32_t sdmmc_load_next(void * buf, size_t len, void * user_data)
{
    uint32_t *sdmmc_buf_copy, *buf_copy, count;
    ALT_STATUS_CODE status = ALT_E_SUCCESS;
    MPL_WATCHDOG();

    if(offset >= SDMMC_BLOCK_SZ)
    {
        /* Need to load more data */
        if(file_size_copy < len + total_loaded)
        {
          len = file_size_copy - total_loaded;
          if(len == 0)
            return 0;/* Were done! */
        }
        status = alt_sdmmc_read(&card_info, buf, (void *)(uint32_t) sd_addr_copy,
            /* Round off to the nearest sdmmc block size */
            (len + SDMMC_BLOCK_SZ - 1) & SDMMC_BLK_ALIGN_MSK);
        if(status != ALT_E_SUCCESS)
          return 0;
        total_loaded += len;
        sd_addr_copy += len;
        if(status != ALT_E_SUCCESS)
            len = 0;
    }
    else
    {
        if(len > SDMMC_BLOCK_SZ - offset)
            len = SDMMC_BLOCK_SZ - offset;

        /*memcpy(buf, sdmmc_buf_copy + offset, len);*/
        sdmmc_buf_copy = (uint32_t *)(sdmmc_buf + offset);
        buf_copy = (uint32_t *) buf;
        for(count = 0; count < len; count+=4)
            *(buf_copy++) = *(sdmmc_buf_copy++);
        offset += len;
    }
    MPL_WATCHDOG();
    return len;
}

ALT_STATUS_CODE sdmmc_load_fpga(uint64_t sd_addr)
{
    ALT_STATUS_CODE status = ALT_E_SUCCESS;

    img_header_t * const img_hdr_p = (img_header_t *)sdmmc_buf;

    //
    // Initialize SDMMC
    //
    status = sdmmc_initialize();
    if (status != ALT_E_SUCCESS)
    {
        ALT_PRINTF("SDMMC: Error initializing SDMMC.\n");
    }

    MPL_WATCHDOG();

    //
    // Read FPGA header and load FPGA into memory
    //

    if (status == ALT_E_SUCCESS)
    {
        // Read the FPGA image header using sd_base_addr found above as the base address
        status = alt_sdmmc_read(&card_info, sdmmc_buf, (void *)(uint32_t) sd_addr, SDMMC_BLOCK_SZ);
        if (status != ALT_E_SUCCESS)
        {
            ALT_PRINTF("SDMMC: Error reading SDMMC.\n");
        }
    }

    if (status == ALT_E_SUCCESS)
    {
        // img_hdr_p always points to sdmmc_buf.
        status = header_validate(img_hdr_p);

        if (status != ALT_E_SUCCESS)
        {
            ALT_PRINTF("SDMMC: Invalid FPGA header.\n");
        }
    }
    file_size_copy = img_hdr_p->img_size;
    sd_addr_copy = sd_addr + SDMMC_BLOCK_SZ;
    offset = IMG_HDR_SZ;
    total_loaded = SDMMC_BLOCK_SZ;

/* Time to start the fpga */
    if (status == ALT_E_SUCCESS)
    {
        status = alt_fpga_init();
    }

    MPL_WATCHDOG();

    // Take control of the FPGA CB
    if (status == ALT_E_SUCCESS)
    {
        status = alt_fpga_control_enable(ALT_FPGA_CFG_MODE_PP32_FAST_NOAES_DC);
    }

    MPL_WATCHDOG();

    if (status == ALT_E_SUCCESS)
    {
        status = alt_fpga_istream_configure(sdmmc_load_next, NULL);
    }

    MPL_WATCHDOG();

    //
    // Cleanup SDMMC
    //

    alt_sdmmc_uninit();

    /////

    return status;
}
