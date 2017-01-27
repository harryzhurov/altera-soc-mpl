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
 * $Id: //acds/rel/16.1/embedded/examples/software/Altera-SoCFPGA-HardwareLib-MPL/core/qspi_load_a10.c#4 $
 */

// include socal headers
#include "socal/socal.h"
#include "alt_qspi.h"
#include "header.h"
#include "mpl_config.h"
#include "mpl_common.h"
#include "alt_fpga_manager.h"

// default value
#ifndef CONFIG_QSPI_NEXT_BOOT_IMAGE
#define CONFIG_QSPI_NEXT_BOOT_IMAGE (0x120000)
#endif

#ifndef CONFIG_QSPI_RBF_IMAGE
#define CONFIG_QSPI_RBF_IMAGE (0x720000)
#endif

/* Information shared between fpga_istream and qspi_fpga_program */
uint32_t qspi_fpga_img_size;
uint32_t qspi_addr;


#if CONFIG_MPL_FPGA_LOAD
static int32_t fpga_istream_callback(void *buf, size_t buf_len, void * user_data)
{
    uint32_t *qspi_buf_copy_copy, *buf_copy, count;
    ALT_STATUS_CODE status = ALT_E_SUCCESS;
    uint32_t device_size = 0;

    if (qspi_fpga_img_size != 0)
    {
        //Load some data from the qspi flash
        if (buf_len <= qspi_fpga_img_size)
        {
            device_size = alt_qspi_get_device_size();
            status = alt_qspi_read(buf, qspi_addr, buf_len);
            if (status != ALT_E_SUCCESS)
            {
                status = ALT_E_ERROR;
            }
            qspi_addr += buf_len;
            qspi_fpga_img_size -= buf_len;
        }
        else if (buf_len > qspi_fpga_img_size)
        {
            status = alt_qspi_read(buf, qspi_addr, qspi_fpga_img_size);
            if (status != ALT_E_SUCCESS)
            {
                status = ALT_E_ERROR;
            }
            buf_len = qspi_fpga_img_size;
            qspi_fpga_img_size = 0;
        }
    }
    else
    {
        buf_len = 0;
    }

    return buf_len;
}

ALT_STATUS_CODE qspi_fpga_program(void)
{
    ALT_STATUS_CODE status = ALT_E_SUCCESS;
    img_header_t img_hdr;

    //
    // Initialize QSPI
    //

    /* First put the QSPI controller into reset to cleanup BootROM */
    if (status == ALT_E_SUCCESS)
    {
        status = alt_qspi_uninit();
    }

    if (status == ALT_E_SUCCESS)
    {
        ALT_PRINTF("QSPI: Initializing QSPI.\n");
        status = alt_qspi_init();
    }

    if (status == ALT_E_SUCCESS)
    {
        status = alt_qspi_enable();
    }

    /* QSPI needs a delay here before reading. There is probably a register that
     * needs to be checked rather than delay, but I don't have time to find it right now.
     */
    for (volatile int i = 0; i < 40000000; i++);

    if (status == ALT_E_SUCCESS)
    {
        if (!alt_qspi_is_idle())
        {
            status = ALT_E_ERROR;
        }
    }

    if (status == ALT_E_SUCCESS)
    {
        ALT_PRINTF("QSPI: Idling.\n");
    }

    if (status == ALT_E_SUCCESS)
    {
        ALT_PRINTF("QSPI: Loading FPGA Image.\n");
    }

    //
    // Read image header and load image chunk into memory
    //

    if (status == ALT_E_SUCCESS)
    {
        status = alt_qspi_read((void *)&img_hdr, CONFIG_QSPI_RBF_IMAGE, IMG_HDR_SZ);
    }


    if (status == ALT_E_SUCCESS)
    {
        status = header_validate(&img_hdr);
    }
    else
    {
        ALT_PRINTF("ERROR: Unable to read QSPI.\n");
    }

    qspi_fpga_img_size = img_hdr.img_size;
    qspi_addr = (CONFIG_QSPI_RBF_IMAGE + IMG_HDR_SZ);

    //Program the FPGA

    if (status == ALT_E_SUCCESS)
    {
        status = alt_fpga_init();
    }
    else
    {
        ALT_PRINTF("ERROR: FPGA Image Header was Invalid.\n");
    }

    // Take control of the FPGA CB
    if (status == ALT_E_SUCCESS)
    {
        status = alt_fpga_control_enable(ALT_FPGA_CFG_MODE_PP32_FAST_NOAES_DC);
    }
    else
    {
        ALT_PRINTF("ERROR: Unable to init FPGA Manager.\n");
    }

    if (status == ALT_E_SUCCESS)
    {
        status = alt_fpga_istream_configure(fpga_istream_callback, NULL);
    }

    if (status == ALT_E_SUCCESS)
    {
        ALT_PRINTF("QSPI: FPGA Image Loaded.\n");
    }
    else
    {
        ALT_PRINTF("ERROR: FPGA Image failed to load.\n");
    }

    if (status == ALT_E_SUCCESS)
    {
        status = alt_fpga_control_disable();
    }

    return status;
}
#endif

ALT_STATUS_CODE qspi_load(launch_info_t * launch)
{
    ALT_STATUS_CODE status = ALT_E_SUCCESS;
    img_header_t img_hdr;

    /* QSPI needs a delay here before reading. There is probably a register that
     * needs to be checked rather than delay, but I don't have time to find it right now.
     */
    for (volatile int i = 0; i < 40000000; i++);

    //
    // Make sure the qspi is idle
    //
    if (status == ALT_E_SUCCESS)
    {
        if (!alt_qspi_is_idle())
        {
            status = ALT_E_ERROR;
        }
    }

    if (status != ALT_E_SUCCESS)
    {
        ALT_PRINTF("ERROR: QSPI not idle.\n");
    }

    /////
    //
    // Read image header and load image into memory
    //

    if (status == ALT_E_SUCCESS)
    {
        ALT_PRINTF("QSPI: Start loading image.\n");
        status = alt_qspi_read((void *)&img_hdr, CONFIG_QSPI_NEXT_BOOT_IMAGE, IMG_HDR_SZ);
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
                       (CONFIG_QSPI_NEXT_BOOT_IMAGE + IMG_HDR_SZ),
                       img_hdr.img_size);
        if (status != ALT_E_SUCCESS)
        {
            ALT_PRINTF("QSPI: Error reading QSPI.\n");
        }
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
    //
    // Cleanup QSPI
    //

    alt_qspi_uninit();

    return status;
}

