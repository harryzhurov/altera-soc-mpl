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
 * $Id: //acds/rel/16.1/embedded/examples/software/Altera-SoCFPGA-HardwareLib-MPL/mpl_config.h#4 $
 */

/*! \file
 *  Altera - MPL Configuration Header definitions
 */

#ifndef _MPL_CONFIG_H
#define _MPL_CONFIG_H

#include "build.h"

/* to enable printf to uart you must also make mpl with PRINTF_UART defined */
#if (CONFIG_PRELOADER_SERIAL_SUPPORT == 1)
#include "alt_printf.h"
#else
#ifndef ALT_PRINTF
#define ALT_PRINTF(...) (void)(0)
#endif
#endif

#ifdef PRINTF_UART
#define LOG_DONE() alt_log_done(DEFAULT_TERM)
#else
#define LOG_INIT()
#define LOG_DONE()
#endif

#if (CONFIG_PRELOADER_STATE_REG_ENABLE == 1)
#define CONFIG_PRELOADER_STATE_REG      (0xFFD080C8)
#define CONFIG_PRELOADER_STATE_VALID    (0x49535756)
#endif

/*
 * When FAT partition support when booting from SDMMC is enabled and FPGA
 * support is also enabled, this specifies the FPGA image filename within a FAT
 * partition to be used as the FPGA image.
 * Note that the name must conform to the 8.3 format (maximum of 8 characters,
 * with optional 3 character extension)
 */
#ifndef CONFIG_MPL_FAT_LOAD_FPGA_NAME
#define CONFIG_MPL_FAT_LOAD_FPGA_NAME           "fpga.img"
#endif

/*
 * If MPL is to program the FPGA and the FPGA resides on the raw QSPI flash,
 * this specifies the absolute location of the FPGA RBF image within the flash
 * in bytes.
 */
#define CONFIG_PRELOADER_FPGA_IMAGE_QSPI_ADDR   (0x800000)

/*
 * If MPL is to program the FPGA and the FPGA resides on the raw SDMMC flash,
 * this specifies the location of of the FPGA RBF image within the flash
 * relative to the start of the preloader in bytes.
 */
#define CONFIG_PRELOADER_FPGA_IMAGE_SDMMC_ADDR  (0x100000)

#endif
