/******************************************************************************
*
* Copyright 2015 Altera Corporation. All Rights Reserved.
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
 * $Id: //acds/rel/16.1/embedded/examples/software/Altera-SoCFPGA-HardwareLib-MPL/core/fat_load_fpga.c#2 $
 */

#include "alt_printf.h"
#include "alt_sdmmc.h"
#include "diskio.h"
#include "ff.h"
#include "header.h"
#include "mpl_config.h"
#include "mpl_common.h"
#include "alt_fpga_manager.h"
#include "sdmmc_common.h"

// include passdown info for next image location
#include "build.h"

/* Information shared between sdmmc_load_next and sdmmc_load_fpga */
uint32_t offset, total_loaded;
char sdmmc_buf[SDMMC_BLOCK_SZ];
uint32_t file_size_copy;

/* Defined in sdmmc_load.c */
FIL   fil;
ALT_STATUS_CODE sdmmc_initialize();

/*#define DIRECT_TO_BUFFER*/

int32_t fpga_load_next(void * buf, size_t len, void * user_data)
{
    uint32_t *sdmmc_buf_copy, *buf_copy, count;
    ALT_STATUS_CODE status = ALT_E_SUCCESS;
    UINT  br = 0;

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
        status = f_read(&fil, buf, (len + SDMMC_BLOCK_SZ - 1) & SDMMC_BLK_ALIGN_MSK,
            /* Round off to the nearest sdmmc block size */
            &br);
        if(status != ALT_E_SUCCESS)
          return 0;
        total_loaded += len;
        if(status != ALT_E_SUCCESS)
            len = 0;
    } else {
        if(len > SDMMC_BLOCK_SZ - offset)
            len = SDMMC_BLOCK_SZ - offset;
        /*memcpy(buf, sdmmc_buf + offset, len);*/
        sdmmc_buf_copy = (uint32_t *)(sdmmc_buf + offset);
        buf_copy = (uint32_t *) buf;
        for(count = 0; count < len; count+=4)
            *(buf_copy++) = *(sdmmc_buf_copy++);
        offset += len;
    }
    MPL_WATCHDOG();
    return len;
}

ALT_STATUS_CODE fat_load_fpga(char *filename)
{
    ALT_STATUS_CODE status = ALT_E_SUCCESS;
    img_header_t * const img_hdr_p = (img_header_t *)sdmmc_buf;

    FATFS fatfs;
    UINT  br = 0;
    UINT  offset = 0;
    /////

    //
    // Mount disk. This will first initialize the disk.
    //

    if (status == ALT_E_SUCCESS)
    {
        // TODO [Fred Hsueh]: Use CONFIG_PRELOADER_FAT_BOOT_PARTITION to switch partitions.
        
        // Parameter 3: 1 => Force mount the volume now; don't do a lazy mount.
        if (FR_OK != f_mount(&fatfs, "", 1))
        {
            ALT_PRINTF("FAT: Unable to mount FAT partition.\n");
            status = ALT_E_ERROR;
        }
    }

    MPL_WATCHDOG();

    if (status == ALT_E_SUCCESS)
    {
        ALT_PRINTF("FAT: Read fpga image [%s]\n", filename);
        if (FR_OK != f_open(&fil, filename, FA_READ))
            status = ALT_E_ERROR;
    }

    //
    // Read FPGA header and load FPGA into memory
    //

    if (status == ALT_E_SUCCESS)
    {
        // Read the FPGA image header using sd_base_addr found above as the base address
        status = f_read(&fil, sdmmc_buf, SDMMC_BLOCK_SZ, &br);
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
        status = alt_fpga_istream_configure(fpga_load_next, NULL);
    }

    MPL_WATCHDOG();

    //
    // Close file handle
    // Report the entry point and image information
    //

    if (status == ALT_E_SUCCESS)
    {
        ALT_PRINTF("FAT: Image written to FPGA\n");
        f_close(&fil);
    }

    MPL_WATCHDOG();

    /////

    return status;
}
