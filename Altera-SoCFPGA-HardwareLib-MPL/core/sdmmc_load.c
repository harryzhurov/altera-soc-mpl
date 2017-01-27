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
 * $Id: //acds/rel/16.1/embedded/examples/software/Altera-SoCFPGA-HardwareLib-MPL/core/sdmmc_load.c#1 $
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

// default value
#ifndef CONFIG_PRELOADER_SDMMC_NEXT_BOOT_IMAGE
#define CONFIG_PRELOADER_SDMMC_NEXT_BOOT_IMAGE	(0x40000)
#endif

ALT_STATUS_CODE sdmmc_load(launch_info_t * launch)
{
    ALT_STATUS_CODE status = ALT_E_SUCCESS;
    uint32_t sd_addr = 0;
    uint32_t* data_src_p;
    uint32_t* data_dst_p;
    uint32_t img_block_sz;
    uint32_t sd_base_addr = 0;

    uint8_t sdmmc_buf[SDMMC_BLOCK_SZ];
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

    if (card_info.card_type == ALT_SDMMC_CARD_TYPE_SDHC)
    {
        ALT_PRINTF("SDHC card detected\n");
    }
    else
    {
        ALT_PRINTF("SD card detected\n");
    }
    /////
    //
    // Locate Altera A2 Partition
    //

    if (status == ALT_E_SUCCESS)
    {
        // read first block check for MBR Table
        status = alt_sdmmc_read(&card_info, sdmmc_buf, (void *)0, SDMMC_BLOCK_SZ);
        if (status != ALT_E_SUCCESS)
        {
            ALT_PRINTF("SDMMC: Error reading SDMMC.\n");
        }
    }

    if (status == ALT_E_SUCCESS)
    {
        if (alt_read_hword(&sdmmc_buf[MBR_SIG_ADDR]) == MBR_SIGNATURE)
        {
            mbr_partition_entry_t * mbr_entry;
            uint32_t raw = 0;
            
            ALT_PRINTF("SDMMC: Found MBR table\n");
            for (int i = 0; i < 4; i++)
            {
                mbr_entry = (mbr_partition_entry_t*)&sdmmc_buf[(MBR_P1_ADDR + (MBR_PENTRY_SZ * i))];
                if (0xA2 == mbr_entry->p_type)  // 0xA2 is custom-partition type
                {
                    raw++;
                    sd_base_addr = (((mbr_entry->lba_hw2) << 16) | mbr_entry->lba_hw1) * SDMMC_BLOCK_SZ; // get block address and convert to byte address.
                    ALT_PRINTF("SDMMC: Using custom partition %d\n", i);
                }
            }
            if (raw == 0)
            {
                ALT_PRINTF("SDMMC: No custom partition found. Using Raw Mode\n");
                sd_base_addr = 0;
            }
        }
        else
        {
            ALT_PRINTF("SDMMC: No MBR found. Using Raw Mode\n");
            sd_base_addr = 0;
        }
    }

    /////

    //
    // Read image header and load image into memory
    //

    if (status == ALT_E_SUCCESS)
    {
        ALT_PRINTF("SDMMC: Start loading Image\n");
        
        // Read next image header using sd_base_addr found above as the base address
        
        sd_addr = sd_base_addr + CONFIG_PRELOADER_SDMMC_NEXT_BOOT_IMAGE;
        status = alt_sdmmc_read(&card_info, sdmmc_buf, (void *)sd_addr, SDMMC_BLOCK_SZ);
        if (status != ALT_E_SUCCESS)
        {
            ALT_PRINTF("SDMMC: Error reading SDMMC.\n");
        }
    }

    if (status == ALT_E_SUCCESS)
    {
        // img_hdr_p always points to sdmmc_buf.
        // img_hdr_p = (img_header_t *)sdmmc_buf;
        
        status = header_validate(img_hdr_p);

        if (status != ALT_E_SUCCESS)
        {
            ALT_PRINTF("SDMMC: Invalid image header.\n");
        }
    }

    if (status == ALT_E_SUCCESS)
    {
        data_src_p = (uint32_t *)&sdmmc_buf[IMG_HDR_SZ];  // Start copying after the image header
        data_dst_p = (uint32_t *)img_hdr_p->load_addr;
        
        if (img_hdr_p->img_size > (SDMMC_BLOCK_SZ - IMG_HDR_SZ))
        {
            // The image is contained in this block and adjacent blocks.
            
            // First copy data contained in the current SDMMC buffer
            for (int i = IMG_HDR_SZ; i < SDMMC_BLOCK_SZ; i += sizeof(uint32_t))
            {
                alt_write_word(data_dst_p, alt_read_word(data_src_p));
                data_src_p++;
                data_dst_p++;
            }
            
            // Now copy the read rest of image
            img_block_sz = (img_hdr_p->img_size & SDMMC_BLK_ALIGN_MSK) + SDMMC_BLOCK_SZ; // round-up to nearest block boundary

            sd_addr += SDMMC_BLOCK_SZ;
            while (img_block_sz)
            {
                uint32_t chunk_sz = SDMMC_CHUNK_SZ;
                if (chunk_sz > img_block_sz)
                {
                    chunk_sz = img_block_sz;
                }
                
                MPL_WATCHDOG();

                status = alt_sdmmc_read(&card_info, data_dst_p, (void *)sd_addr, chunk_sz);
                if (status != ALT_E_SUCCESS)
                {
                    ALT_PRINTF("SDMMC: Error reading SDMMC.\n");
                    break;
                }
                
                data_dst_p   += chunk_sz >> 2; // Divide by 4 because data_dst_p is type (uint32_t *).
                sd_addr      += chunk_sz;
                img_block_sz -= chunk_sz;
            }
        }
        else
        {
            // The complete image is already in the SDMMC buffer.
            
            for (int i = IMG_HDR_SZ; i < (img_hdr_p->img_size); i += sizeof(uint32_t))
            {
                alt_write_word(data_dst_p, alt_read_word(data_src_p));
                data_src_p++;
                data_dst_p++;
            }
        }
    }

    MPL_WATCHDOG();

    //
    // Report the entry point and image information
    //

    if (status == ALT_E_SUCCESS)
    {
        ALT_PRINTF("SDMMC: Image loaded successfully.\n");
        launch->entry_addr = img_hdr_p->load_addr + img_hdr_p->entry_point;
    
        launch->image_addr = img_hdr_p->load_addr;
        launch->image_size = img_hdr_p->img_size;
#if (CONFIG_PRELOADER_CHECKSUM_NEXT_IMAGE == 1)
        launch->image_crc = img_hdr_p->dcrc;
#endif
    }

    /////

#if (CONFIG_MPL_FPGA_LOAD == 1)

    //
    // Read FPGA header and load FPGA into memory
    //

    if (status == ALT_E_SUCCESS)
    {
        ALT_PRINTF("SDMMC: Start loading FPGA\n");
        
        // Read the FPGA image header using sd_base_addr found above as the base address
        
        sd_addr = sd_base_addr + CONFIG_PRELOADER_FPGA_IMAGE_SDMMC_ADDR;
        status = alt_sdmmc_read(&card_info, sdmmc_buf, (void *)sd_addr, SDMMC_BLOCK_SZ);
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

    if (status == ALT_E_SUCCESS)
    {
        data_src_p = (uint32_t *)&sdmmc_buf[IMG_HDR_SZ];  // start copying after the image header
        data_dst_p = (uint32_t *)img_hdr_p->load_addr;

        // FPGA images are always larger than the SDMMC buffer.
        
        // The image is contained in this block and adjacent blocks.

        // First copy data contained in the current SDMMC buffer            
        for (int i = IMG_HDR_SZ; i < SDMMC_BLOCK_SZ; i += sizeof(uint32_t))
        {
            alt_write_word(data_dst_p, alt_read_word(data_src_p));
            data_src_p++;
            data_dst_p++;
        }

        // Now copy the read rest of image
        img_block_sz = (img_hdr_p->img_size & SDMMC_BLK_ALIGN_MSK) + SDMMC_BLOCK_SZ; // round-up to nearest block boundary

        sd_addr += SDMMC_BLOCK_SZ;
        while (img_block_sz)
        {
            uint32_t chunk_sz = SDMMC_CHUNK_SZ;
            if (chunk_sz > img_block_sz)
            {
                chunk_sz = img_block_sz;
            }

            MPL_WATCHDOG();
            
            status = alt_sdmmc_read(&card_info, data_dst_p, (void *)sd_addr, chunk_sz);
            if (status != ALT_E_SUCCESS)
            {
                ALT_PRINTF("SDMMC: Error reading SDMMC.\n");
                break;
            }

            data_dst_p   += chunk_sz >> 2; // Divide by 4 because data_dst_p is type (uint32_t *).
            sd_addr      += chunk_sz;
            img_block_sz -= chunk_sz;
        }
    }

    MPL_WATCHDOG();

    //
    // Report the FPGA information
    //

    if (status == ALT_E_SUCCESS)
    {
        ALT_PRINTF("SDMMC: FPGA loaded successfully.\n");
        
        launch->fpga_addr = img_hdr_p->load_addr;
        launch->fpga_size = img_hdr_p->img_size;
#if (CONFIG_PRELOADER_CHECKSUM_NEXT_IMAGE == 1)
        launch->fpga_crc  = img_hdr_p->dcrc;
#endif
    }

#endif

    //
    // Cleanup SDMMC
    //

    alt_sdmmc_uninit();

    /////

    return status;
}
