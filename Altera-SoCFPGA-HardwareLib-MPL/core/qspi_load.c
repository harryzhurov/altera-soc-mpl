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
 * $Id: //acds/rel/16.1/embedded/examples/software/Altera-SoCFPGA-HardwareLib-MPL/core/qspi_load.c#1 $
 */

// include socal headers
#include "socal/socal.h"
#include "alt_qspi.h"
#include "header.h"
#include "mpl_config.h"
#include "mpl_common.h"

// include passdown info for next image location
#include "build.h"

// default value
#ifndef CONFIG_PRELOADER_QSPI_NEXT_BOOT_IMAGE
#define CONFIG_PRELOADER_QSPI_NEXT_BOOT_IMAGE (0x60000)
#endif

#ifdef WINCE
// NOTE: Special provisions for WEC7 QSPI Boot Path support for 512 KiB QSPI devices.
static ALT_STATUS_CODE qspi_load_512K(void)
{
    ALT_STATUS_CODE status;

    // Load the .bin file sans header stored at the (next boot image) / 2 address.
    uint32_t addr = CONFIG_PRELOADER_QSPI_NEXT_BOOT_IMAGE / 2;

    struct BIN_HEADER
    {
        uint32_t Start;
        uint32_t Length;
    } bin_header;

    struct BIN_BLOCK
    {
        uint32_t Address;  // Address where the block should be copied.
        uint32_t Length;   // Length of the block that is being flashed.
        uint32_t Checksum; // Checksum (CRC32) of the block data.
    } bin_block;

    status = alt_qspi_read(&bin_header, addr, sizeof(bin_header));
    if (status != ALT_E_SUCCESS) { return status; }

    addr += sizeof(bin_header);

    while (true)
    {
        MPL_WATCHDOG;

        status = alt_qspi_read(&bin_block, addr, sizeof(bin_block));
        if (status != ALT_E_SUCCESS) { return status; }

        addr += sizeof(bin_block);

        // Detect the end of the block.
        if (!bin_block.Address && !bin_block.Checksum)
        {
            break;
        }

        // NOTE: 0x140000 constant determined by several factors:
        //  - EBoot is compiled to start at 0x80040000 (virtual address) [see eboot.bib, RAM, ROMSTART]
        //  - 0x00040000 will contain an instruction to jump to 0x00041000.
        //  - Virtual address mapping is VA:0x80000000 -> PA 0x00100000 [see oemaddrtab_cfg.inc, g_oalAddr
essTable]
        //  - Thus if we load the image offsetting 0x141000 before VA is enabled,
        //    this will align to 0x80041000 after VA is enabled.
        status = alt_qspi_read((void *)((bin_block.Address - bin_header.Start) + 0x140000),
                     addr, bin_block.Length);
        if (status != ALT_E_SUCCESS) { return status; }

        addr += bin_block.Length;
    }

    MPL_WATCHDOG;
    // jump to entry point
    (*(FXN_PTR)(0x140000))();

    return ALT_E_SUCCESS;
}
#endif

ALT_STATUS_CODE qspi_load(launch_info_t * launch)
{
    ALT_STATUS_CODE status = ALT_E_SUCCESS;
    img_header_t img_hdr;

    //
    // Initialize QSPI
    //

    if (status == ALT_E_SUCCESS)
    {
        ALT_PRINTF("QSPI: Initializing QSPI.\n");
        status = alt_qspi_init();
    }
    
    if (status == ALT_E_SUCCESS)
    {
        status = alt_qspi_enable();
    }

    if (status == ALT_E_SUCCESS)
    {
        if (!alt_qspi_is_idle())
        {
            status = ALT_E_ERROR;
        }
    }

    if (status != ALT_E_SUCCESS)
    {
        ALT_PRINTF("QSPI: Error initializing QSPI.\n");
    }

    MPL_WATCHDOG();

#ifdef WINCE
    // NOTE: Special provisions for WEC7 QSPI Boot Path support for 512 KiB QSPI devices.
    if (alt_qspi_get_device_size() <= (512 * 1024))
    {
        return qspi_load_512K();
    }
#endif

    /////

    //
    // Read image header and load image into memory
    //

    if (status == ALT_E_SUCCESS)
    {
        ALT_PRINTF("QSPI: Start loading image.\n");
        status = alt_qspi_read((void *)&img_hdr, CONFIG_PRELOADER_QSPI_NEXT_BOOT_IMAGE, IMG_HDR_SZ);
        if (status != ALT_E_SUCCESS)
        {
            ALT_PRINTF("QSPI: Error reading QSPI.\n");
        }
    }

    if (status == ALT_E_SUCCESS)
    {
        status = header_validate(&img_hdr);

        if (status != ALT_E_SUCCESS)
        {
            ALT_PRINTF("QSPI: Invalid image header.\n");
        }
    }

    if (status == ALT_E_SUCCESS)
    {
        status = alt_qspi_read((void *)img_hdr.load_addr, 
                       (CONFIG_PRELOADER_QSPI_NEXT_BOOT_IMAGE + IMG_HDR_SZ),
                       img_hdr.img_size);
        if (status != ALT_E_SUCCESS)
        {
            ALT_PRINTF("QSPI: Error reading QSPI.\n");
        }

        MPL_WATCHDOG();
    }


    //
    // Report the entry point and image information
    //

    if (status == ALT_E_SUCCESS)
    {
        ALT_PRINTF("QSPI: Image loaded successfully.\n");
        
        launch->entry_addr = img_hdr.load_addr+img_hdr.entry_point;

        launch->image_addr = img_hdr.load_addr;
        launch->image_size = img_hdr.img_size;
#if (CONFIG_PRELOADER_CHECKSUM_NEXT_IMAGE == 1)
        launch->image_crc  = img_hdr.dcrc;
#endif
    }

    /////
    
#if (CONFIG_MPL_FPGA_LOAD == 1)

    //
    // Read FPGA header and load FPGA into memory
    //

    if (status == ALT_E_SUCCESS)
    {
        ALT_PRINTF("QSPI: Start loading FPGA.\n");
        status = alt_qspi_read((void *)&img_hdr,
                      CONFIG_PRELOADER_FPGA_IMAGE_QSPI_ADDR, IMG_HDR_SZ);
        if (status != ALT_E_SUCCESS)
        {
            ALT_PRINTF("QSPI: Error reading QSPI.\n");
        }
    }

    if (status == ALT_E_SUCCESS)
    {
        status = header_validate(&img_hdr);

        if (status != ALT_E_SUCCESS)
        {
            ALT_PRINTF("QSPI: Invalid FPGA header.\n");
            status = ALT_E_ERROR;
        }
    }

    if (status == ALT_E_SUCCESS)
    {
        status = alt_qspi_read((void *)img_hdr.load_addr, 
                     (CONFIG_PRELOADER_FPGA_IMAGE_QSPI_ADDR + IMG_HDR_SZ), 
                     img_hdr.img_size);
        if (status != ALT_E_SUCCESS)
        {
            ALT_PRINTF("QSPI: Error reading QSPI.\n");
        }

        MPL_WATCHDOG();
    }

    //
    // Report the FPGA information
    //

    if (status == ALT_E_SUCCESS)
    {
        ALT_PRINTF("QSPI: FPGA loaded successfully.\n");
        
        launch->fpga_addr = img_hdr.load_addr;
        launch->fpga_size = img_hdr.img_size;
#if (CONFIG_PRELOADER_CHECKSUM_NEXT_IMAGE == 1)
        launch->fpga_crc  = img_hdr.dcrc;
#endif
    }

#endif

    //
    // Cleanup QSPI
    //

    alt_qspi_uninit();

    /////

    return status;
}
