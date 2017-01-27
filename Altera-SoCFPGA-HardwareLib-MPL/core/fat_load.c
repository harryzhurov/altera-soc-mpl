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
 * $Id: //acds/rel/16.1/embedded/examples/software/Altera-SoCFPGA-HardwareLib-MPL/core/fat_load.c#1 $
 */

#include "alt_printf.h"
#include "alt_sdmmc.h"
#include "diskio.h"
#include "ff.h"
#include "header.h"
#include "mpl_config.h"
#include "mpl_common.h"
#include "sdmmc_common.h"

// include passdown info for next image location
#include "build.h"

//
// Helper functions needed by FatFs
//

DSTATUS disk_status(
    BYTE pdrv               /* Physical drive nmuber to identify the drive */
    )
{
    return RES_OK;
}

/* Defined in sdmmc_load.c */
ALT_STATUS_CODE sdmmc_initialize();

DSTATUS disk_initialize (
    BYTE pdrv               /* Physical drive nmuber to identify the drive */
    )
{
    ALT_STATUS_CODE status = sdmmc_initialize();

    if (status == ALT_E_SUCCESS)
    {
        return 0;
    }
    else
    {
        ALT_PRINTF("FAT: Error initializing SDMMC.\n");
        return STA_NOINIT;
    }
}

DRESULT disk_read(
    BYTE pdrv,      /* Physical drive nmuber to identify the drive */
    BYTE * buff,    /* Data buffer to store read data */
    DWORD sector,   /* Sector address in LBA */
    UINT count      /* Number of sectors to read */
    )

{
    ALT_STATUS_CODE status = ALT_E_SUCCESS;

    status = alt_sdmmc_read(&card_info, buff, (void *)(sector * SDMMC_BLOCK_SZ), (count * SDMMC_BLOCK_SZ));
    if (status != ALT_E_SUCCESS)
    {
        ALT_PRINTF("FAT: Error reading SDMMC.\n");
        return RES_ERROR;
    }

    return RES_OK;
}

/////

ALT_STATUS_CODE fat_load(launch_info_t * launch)
{
    ALT_STATUS_CODE status = ALT_E_SUCCESS;

    img_header_t img_hdr;

    FATFS fatfs;
    FIL   fil;

    /////

    //
    // Mount disk. This will first initialize the disk.
    //

    if (status == ALT_E_SUCCESS)
    {
        ALT_PRINTF("FAT: Mounting FAT.\n");

        // TODO [Fred Hsueh]: Use CONFIG_PRELOADER_FAT_BOOT_PARTITION to switch partitions.
        
        // Parameter 3: 1 => Force mount the volume now; don't do a lazy mount.
        if (FR_OK != f_mount(&fatfs, "", 1))
        {
            ALT_PRINTF("FAT: Unable to mount FAT partition.\n");
            status = ALT_E_ERROR;
        }
    }

    /////

    //
    // Read image header and load image into memory
    //

    if (status == ALT_E_SUCCESS)
    {
        const char * filename = CONFIG_PRELOADER_FAT_LOAD_PAYLOAD_NAME;
        ALT_PRINTF("FAT: Read image [%s] header.\n", filename);

        UINT br = 0;

        if (FR_OK != f_open(&fil, filename, FA_READ))
        {
            status = ALT_E_ERROR;
        }
        else if (FR_OK != f_read(&fil, &img_hdr, IMG_HDR_SZ, &br))
        {
            status = ALT_E_ERROR;
        }
        else if (br != IMG_HDR_SZ)
        {
            status = ALT_E_ERROR;
        }
    }

    if (status == ALT_E_SUCCESS)
    {
        status = header_validate(&img_hdr);

        if (status != ALT_E_SUCCESS)
        {
            ALT_PRINTF("FAT: Invalid image header.\n");
        }
    }

    if (status == ALT_E_SUCCESS)
    {
        ALT_PRINTF("FAT: Read remaining image data.\n");

        char * data_dst_p = (char *)img_hdr.load_addr;

        uint32_t img_block_sz = img_hdr.img_size;

        while (img_block_sz)
        {
            UINT br = 0;
            uint32_t chunk_sz = SDMMC_CHUNK_SZ;

            if (chunk_sz > img_block_sz)
            {
                chunk_sz = img_block_sz;
            }

            MPL_WATCHDOG();

            if (FR_OK != f_read(&fil, data_dst_p, chunk_sz, &br))
            {
                status = ALT_E_ERROR;
            }
            else if (br != chunk_sz)
            {
                status = ALT_E_ERROR;
            }

            if (status != ALT_E_SUCCESS)
            {
                break;
            }

            data_dst_p   += chunk_sz;
            img_block_sz -= chunk_sz;
        }
    }

    //
    // Close file handle
    // Report the entry point and image information
    //

    if (status == ALT_E_SUCCESS)
    {
        ALT_PRINTF("FAT: Image loaded to memory.\n");

        f_close(&fil);

        launch->entry_addr = img_hdr.load_addr + img_hdr.entry_point;

        launch->image_addr = img_hdr.load_addr;
        launch->image_size = img_hdr.img_size;
#if (CONFIG_PRELOADER_CHECKSUM_NEXT_IMAGE == 1)
        launch->image_crc  = img_hdr.dcrc;
#endif
    }

    MPL_WATCHDOG();

    /////

#if (CONFIG_MPL_FPGA_LOAD == 1)

    //
    // Read FPGA header and load FPGA into memory
    //

    if (status == ALT_E_SUCCESS)
    {
        const char * filename = CONFIG_MPL_FAT_LOAD_FPGA_NAME;
        ALT_PRINTF("FAT: Read FPGA [%s] header.\n", filename);

        UINT br = 0;

        if (FR_OK != f_open(&fil, filename, FA_READ))
        {
            status = ALT_E_ERROR;
        }
        else if (FR_OK != f_read(&fil, &img_hdr, IMG_HDR_SZ, &br))
        {
            status = ALT_E_ERROR;
        }
        else if (br != IMG_HDR_SZ)
        {
            status = ALT_E_ERROR;
        }
    }

    if (status == ALT_E_SUCCESS)
    {
        status = header_validate(&img_hdr);

        if (status != ALT_E_SUCCESS)
        {
            ALT_PRINTF("FAT: Invalid FPGA header.\n");
        }
    }

    if (status == ALT_E_SUCCESS)
    {
        ALT_PRINTF("FAT: Read remaining FPGA data.\n");

        char * data_dst_p = (char *)img_hdr.load_addr;
        uint32_t img_block_sz = img_hdr.img_size;

        while (img_block_sz)
        {
            UINT br = 0;
            uint32_t chunk_sz = SDMMC_CHUNK_SZ;

            if (chunk_sz > img_block_sz)
            {
                chunk_sz = img_block_sz;
            }

            MPL_WATCHDOG();

            if (FR_OK != f_read(&fil, data_dst_p, chunk_sz, &br))
            {
                status = ALT_E_ERROR;
            }
            else if (br != chunk_sz)
            {
                status = ALT_E_ERROR;
            }

            if (status != ALT_E_SUCCESS)
            {
                break;
            }

            data_dst_p   += chunk_sz;
            img_block_sz -= chunk_sz;
        }
    }

    //
    // Close file handle
    // Report the FPGA information
    //

    if (status == ALT_E_SUCCESS)
    {
        ALT_PRINTF("FAT: FPGA loaded to memory.\n");

        f_close(&fil);

        launch->fpga_addr = img_hdr.load_addr;
        launch->fpga_size = img_hdr.img_size;
#if (CONFIG_PRELOADER_CHECKSUM_NEXT_IMAGE == 1)
        launch->fpga_crc  = img_hdr.dcrc;
#endif
    }

    MPL_WATCHDOG();

#endif

    //
    // Cleanup SDMMC
    //

    alt_sdmmc_uninit();

    /////

    return status;
}
