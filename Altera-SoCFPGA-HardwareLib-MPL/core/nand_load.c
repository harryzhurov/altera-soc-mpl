/******************************************************************************
*
* Copyright 2016 Altera Corporation. All Rights Reserved.
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
 * $Id: //acds/rel/16.1/embedded/examples/software/Altera-SoCFPGA-HardwareLib-MPL/core/nand_load.c#1 $
 */

// include socal headers
#include "socal/socal.h"
#include "alt_nand.h"
#include "header.h"
#include "mpl_config.h"
#include "mpl_common.h"

// include passdown info for next image location
#include "build.h"

// default value
#ifndef CONFIG_PRELOADER_NAND_NEXT_BOOT_IMAGE
#define CONFIG_PRELOADER_NAND_NEXT_BOOT_IMAGE (0xC0000)
#endif

#define NAND_PAGE_ALIGN_MSK 0xFFFFF800

#define NAND_PAGE_SZ        2048
#define NAND_SPARE_SZ       64

ALT_STATUS_CODE alt_nand_flash_init_manual(void *);

ALT_STATUS_CODE alt_nand_flash_init_manual(void *user_arg)
{
    return ALT_E_RESERVED;
}

ALT_STATUS_CODE check_ecc(void)
{
    ALT_STATUS_CODE status = ALT_E_SUCCESS;
    ALT_NAND_FLASH_ECC_STATUS_t *ecc_status;

    status = alt_nand_ecc_status_get(ecc_status);

    if ((ecc_status->corrected_errors[0] != 0) || (ecc_status->corrected_errors[1] != 0) ||
            (ecc_status->corrected_errors[2] != 0) || (ecc_status->corrected_errors[3] != 0))
    {
        ALT_PRINTF("NAND: An ECC correction occurred.\n");
    }

    if ((ecc_status->uncorrected_error[0] == true) || (ecc_status->uncorrected_error[1] == true) ||
            (ecc_status->uncorrected_error[2] == true) || (ecc_status->uncorrected_error[3] == true))
    {
        ALT_PRINTF("NAND: An uncorrectable ECC error occurred on NAND read, aborting boot.\n");
        status = ALT_E_ERROR;
    }

    return status;
}

void write_nand_data_to_mem(uint32_t init_loop_val, uint32_t page_size, uint32_t spare_size, uint8_t * src_p, uint8_t * dst_p)
{
    for (uint32_t i = init_loop_val; i < (page_size + spare_size); i++)
    {
        // We need to handle the ECC bytes injected by the NAND controller
        if ((i == 512) || (i == 1038) || (i == 1564))
        {
            //Skip over the 14 bytes of the ECC stripes
            i += 13;
            src_p += 14;
        }
        else if (i == 2048) //Also skip over the bad block marker
        {
            i += 1;
            src_p += 2;
        }
        else if (i == 2092)
        {
            //Break out, we've hit the end of the data
            //There is no data stored after the last ECC byte in the spare area
            break;
        }
        else
        {
            alt_write_byte(dst_p, alt_read_byte(src_p));
            src_p++;
            dst_p++;
        }
    }
}

ALT_STATUS_CODE nand_load(launch_info_t * launch)
{
    ALT_STATUS_CODE status = ALT_E_SUCCESS;
    alt_nand_flash_custom_init_t init_nand = &alt_nand_flash_init_manual;

    uint32_t param;
    uint32_t block, page, end_block;
    uint32_t nand_block_size;
    uint32_t nand_max_num_pages = 0;
    uint8_t * data_src_p;
    uint8_t * data_dst_p;
    uint32_t img_block_sz;
    uint32_t nand_page_size;
    uint32_t nand_spare_size;
    uint32_t nand_pages_per_blk;
    uint32_t nand_num_blks;
    bool good_block_flag = false;

    uint8_t nand_buf[NAND_PAGE_SZ + NAND_SPARE_SZ];
    img_header_t * const img_hdr_p = (img_header_t *)nand_buf;

    //
    // Initialize NAND
    //
    if (status == ALT_E_SUCCESS)
    {
        ALT_PRINTF("NAND: Initializing NAND.\n");
        status = alt_nand_init(ALT_NAND_BOOTSTRAP_INHIBIT_B0P0_LOAD_DISABLE,
                                ALT_NAND_BOOTSTRAP_512B_DEVICE_DISABLE,
                                init_nand,
                                (void *)&param);
    }

    if (status != ALT_E_SUCCESS)
    {
        ALT_PRINTF("NAND: Error initializing NAND.\n");
    }

    if (status == ALT_E_SUCCESS)
    {
        //Enable the ECC with 14 byte stripes
        alt_nand_ecc_enable(ALT_NAND_ECC_8_BIT_CORRECTION);
    }

    MPL_WATCHDOG();

    //
    // Read image header and load image into memory
    //
    if (status == ALT_E_SUCCESS)
    {
        ALT_PRINTF("NAND: Start loading image.\n");

        //Read the page that contains the image header
        nand_block_size = alt_nand_page_size_get() * alt_nand_num_pages_per_block_get();

        // Figure out which block/page has the image
        block = CONFIG_PRELOADER_NAND_NEXT_BOOT_IMAGE / nand_block_size;
        page = (CONFIG_PRELOADER_NAND_NEXT_BOOT_IMAGE % nand_block_size) / alt_nand_page_size_get();

        while (good_block_flag != true)
        {
            status = alt_nand_read_page_and_spare_byte(block, page, nand_buf);

            MPL_WATCHDOG();
            status = check_ecc();
            if (status != ALT_E_SUCCESS)
            {
                ALT_PRINTF("NAND: Error reading NAND.\n");
                break;
            }

            if ((nand_buf[NAND_PAGE_SZ] != 0xFF) || (nand_buf[NAND_PAGE_SZ + 1] != 0xFF))
            {
                ALT_PRINTF("NAND: Bad block detected, skipping the block.\n");
                block++;
                if (block > nand_num_blks)
                {
                    ALT_PRINTF("NAND: Error, hit max number of blocks on chip\n");
                    status = ALT_E_ERROR;
                    break;
                }
            }
            else
            {
                //We read a good block, move on
                good_block_flag = true;
            }
        }
    }

    //
    // Read image header and load image into memory
    //
    if (status == ALT_E_SUCCESS)
    {
        status = header_validate(img_hdr_p);

        if (status != ALT_E_SUCCESS)
        {
            ALT_PRINTF("NAND: Invalid image header.\n");
        }
    }

    //
    // Set the entry point and image information before losing nand_buf
    //
    if (status == ALT_E_SUCCESS)
    {
        launch->entry_addr = img_hdr_p->load_addr + img_hdr_p->entry_point;
        launch->image_addr = img_hdr_p->load_addr;
        launch->image_size = img_hdr_p->img_size;
        #if (CONFIG_PRELOADER_CHECKSUM_NEXT_IMAGE == 1)
            launch->image_crc = img_hdr_p->dcrc;
        #endif
    }

    if (status == ALT_E_SUCCESS)
    {
        nand_page_size = alt_nand_page_size_get();
        nand_spare_size = alt_nand_spare_size_get();

        data_src_p = (uint8_t *)&nand_buf[IMG_HDR_SZ];  // Start copying after the image header
        data_dst_p = (uint8_t *)img_hdr_p->load_addr;

        if (img_hdr_p->img_size > (nand_page_size - IMG_HDR_SZ))
        {
            // The image is contained in this page and adjacent pages

            // First copy data contained in the current NAND page buffer
            write_nand_data_to_mem(IMG_HDR_SZ, nand_page_size, nand_spare_size, data_src_p, data_dst_p);

            // Now copy the rest of the image
            img_block_sz = (img_hdr_p->img_size & NAND_PAGE_ALIGN_MSK) + nand_page_size; // round-up to nearest page boundary

            // Go to the next page after the initial load
            page++;

            nand_pages_per_blk = alt_nand_num_pages_per_block_get();
            nand_num_blks = alt_nand_num_blocks_get();

            while(img_block_sz)
            {
                //kick the watchdog every page read for now
                MPL_WATCHDOG();

                good_block_flag = false;
                while (good_block_flag != true)
                {
                    status = alt_nand_read_page_and_spare_byte(block, page, nand_buf);

                    MPL_WATCHDOG();
                    status = check_ecc();
                    if (status != ALT_E_SUCCESS)
                    {
                        ALT_PRINTF("NAND: Error reading NAND.\n");
                        break;
                    }

                    if ((nand_buf[NAND_PAGE_SZ] != 0xFF) || (nand_buf[NAND_PAGE_SZ + 1] != 0xFF))
                    {
                        ALT_PRINTF("NAND: Bad block detected, skipping the block.\n");
                        block++;
                        if (block > nand_num_blks)
                        {
                            ALT_PRINTF("NAND: Error, hit max number of blocks on chip\n");
                            status = ALT_E_ERROR;
                            break;
                        }
                    }
                    else
                    {
                        //We read a good block, move on
                        good_block_flag = true;
                    }
                }

                data_src_p = (uint8_t *)&nand_buf; //Reset data_src_p

                write_nand_data_to_mem(0, nand_page_size, nand_spare_size, data_src_p, data_dst_p);
 
                //Handle page boundary
                page++;
                if (page >= nand_pages_per_blk)
                {

                    page = 0;
                    block++;

                    if (block > nand_num_blks)
                    {
                        ALT_PRINTF("NAND: Error, hit max number of blocks on chip\n");
                        status = ALT_E_ERROR;
                        break;
                    }
                }

                img_block_sz -= nand_page_size;
            }
        }
        else
        {
            // The complete image is already in the NAND buffer.

            for (int i = IMG_HDR_SZ; i < (img_hdr_p->img_size); i++)
            {
                // We need to handle the ECC bytes injected by the NAND controller
                if ((i == 512) || (i == 1038) || (i == 1564))
                {
                    //Skip over the 14 bytes of the ECC stripes
                    i += 13;
                    data_src_p += 14;
                }
                else if (i == 2048) //Also skip over the bad block marker
                {
                    i += 1;
                    data_src_p += 2;
                }
                else if (i == 2092)
                {
                    //Break out, we've hit the end of the data
                    //There is no data stored after the last ECC byte in the spare area
                    break;
                }
                else
                {
                    alt_write_byte(data_dst_p, alt_read_byte(data_src_p));
                    data_src_p++;
                    data_dst_p++;
                }
            }
        }

        if (status != ALT_E_SUCCESS)
        {
            ALT_PRINTF("NAND: Error reading NAND.\n");
        }

        MPL_WATCHDOG();
    }

    if (status == ALT_E_SUCCESS)
    {
        ALT_PRINTF("NAND: Image loaded successfully.\n");
    }

    //
    // Cleanup NAND
    //
    alt_nand_ecc_disable();
    alt_nand_uninit();

    /////

    return status;
}

