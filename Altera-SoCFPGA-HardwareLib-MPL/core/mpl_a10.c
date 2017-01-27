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
 * $Id: //acds/rel/16.1/embedded/examples/software/Altera-SoCFPGA-HardwareLib-MPL/core/mpl_a10.c#7 $
 */

#include <stdio.h>
#include "alt_fpga_manager.h"
#include "mpl_config.h"
#include "mpl_common.h"
#include "socal/alt_rstmgr.h"
#include "socal/alt_sysmgr.h"
#include "socal/socal.h"
#include "socal/hps.h"
#include "header.h"


#define CONFIG_HPS_RESET_EMAC0          0
#define CONFIG_HPS_RESET_EMAC1          0
#define CONFIG_HPS_RESET_EMAC2          0
#define CONFIG_HPS_RESET_USB0           0
#define CONFIG_HPS_RESET_USB1           0
#define CONFIG_HPS_RESET_NAND           0
#define CONFIG_HPS_RESET_QSPI           0
#define CONFIG_HPS_RESET_SDMMC          0
#define CONFIG_HPS_RESET_EMAC0OCP       0
#define CONFIG_HPS_RESET_EMAC1OCP       0
#define CONFIG_HPS_RESET_EMAC2OCP       0
#define CONFIG_HPS_RESET_USB0OCP        0
#define CONFIG_HPS_RESET_USB1OCP        0
#define CONFIG_HPS_RESET_NANDOCP        0
#define CONFIG_HPS_RESET_QSPIOCP        0
#define CONFIG_HPS_RESET_SDMMCOCP       0
#define CONFIG_HPS_RESET_DMA            0
#define CONFIG_HPS_RESET_SPIM0          0
#define CONFIG_HPS_RESET_SPIM1          0
#define CONFIG_HPS_RESET_SPIS0          0
#define CONFIG_HPS_RESET_SPIS1          0
#define CONFIG_HPS_RESET_DMAOCP         0
#define CONFIG_HPS_RESET_EMACPTP        0
#define CONFIG_HPS_RESET_DMAIF0         0
#define CONFIG_HPS_RESET_DMAIF1         0
#define CONFIG_HPS_RESET_DMAIF2         0
#define CONFIG_HPS_RESET_DMAIF3         0
#define CONFIG_HPS_RESET_DMAIF4         0
#define CONFIG_HPS_RESET_DMAIF5         0
#define CONFIG_HPS_RESET_DMAIF6         0
#define CONFIG_HPS_RESET_DMAIF7         0
#define CONFIG_HPS_RESET_WD0_SET        0
#define CONFIG_HPS_RESET_WD1_SET        0
#define CONFIG_HPS_RESET_L4SYSTMR0_SET  0
#define CONFIG_HPS_RESET_L4SYSTMR1_SET  0
#define CONFIG_HPS_RESET_SPTMR0_SET     0
#define CONFIG_HPS_RESET_SPTMR1_SET     0
#define CONFIG_HPS_RESET_I2C0_SET       0
#define CONFIG_HPS_RESET_I2C1_SET       0
#define CONFIG_HPS_RESET_I2C2_SET       0
#define CONFIG_HPS_RESET_I2C3_SET       0
#define CONFIG_HPS_RESET_I2C4_SET       0
#define CONFIG_HPS_RESET_UART0_SET      0
#define CONFIG_HPS_RESET_UART1_SET      0
#define CONFIG_HPS_RESET_GPIO0_SET      0
#define CONFIG_HPS_RESET_GPIO1_SET      0
#define CONFIG_HPS_RESET_GPIO2_SET      0





// If you want to build your app into MPL, you have to undefine the boot source specified in build.h
#ifdef NO_BOOT
#undef SDMMC_BOOT
#undef QSPI_BOOT
#undef FAT_BOOT
#endif
uint32_t handoff_data[16];

// For ARMCOMPILER, ensure semihosting functions are not being linked.
// And, resolved the ones that are absolutely needed locally.
#if defined(ARMCOMPILER) && defined(PRINTF_UART)
#pragma import(__use_no_semihosting)
#endif

#ifdef ARMCOMPILER
void _sys_exit(int return_code)
{
    while (1)
        ;
}
int _sbrk = 0;
#endif

#ifndef CONFIG_PRELOADER_DEBUG_MEMORY_ADDR  // default needed for startup.s
#define CONFIG_PRELOADER_DEBUG_MEMORY_ADDR	0xffe3fd00
#define CONFIG_PRELOADER_DEBUG_MEMORY_SIZE	(0x200)
#endif
uint32_t debug_memory_ptr = CONFIG_PRELOADER_DEBUG_MEMORY_ADDR;

/////

uint32_t crc32(uint32_t crc, const uint8_t *buf, uint32_t size);

ALT_STATUS_CODE board_init(void);
ALT_STATUS_CODE sdram_init(void);

ALT_STATUS_CODE qspi_fpga_program(void);
ALT_STATUS_CODE qspi_load(launch_info_t * launch);
ALT_STATUS_CODE sdmmc_load_fpga(uint64_t card_loc);
ALT_STATUS_CODE sdmmc_load(launch_info_t * launch);
ALT_STATUS_CODE fat_load_fpga(char *filename);
ALT_STATUS_CODE fat_load(launch_info_t * launch);

/////

void update_handoff(void)
{
    uint32_t brgmodrst   = 0;

    // Disable BootROM mapped to address 0. 
    alt_write_word(ALT_SYSMGR_NOC_ADDR_REMAP_SET_ADDR, ALT_SYSMGR_NOC_ADDR_REMAP_VALUE_REMAP0_SET_MSK);
    // Map OCRam to address 0
    alt_write_word(ALT_SYSMGR_NOC_ADDR_REMAP_CLR_ADDR, ALT_SYSMGR_NOC_ADDR_REMAP_CLR_REMAP1_SET_MSK);
#if CONFIG_HPS_RESET_ASSERT_HPS2FPGA
    brgmodrst |= ALT_RSTMGR_BRGMODRST_H2F_SET_MSK; // Keep bridge in reset
#endif
#if CONFIG_HPS_RESET_ASSERT_LWHPS2FPGA
    bgrmodrst |= ALT_RSTMGR_BRGMODRST_LWH2F_SET_MSK;
#endif
#if CONFIG_HPS_RESET_ASSERT_FPGA2HPS
    brgmodrst |= ALT_RSTMGR_BRGMODRST_F2H_SET_MSK;
#endif
    alt_write_word(ALT_RSTMGR_BRGMODRST_ADDR,       brgmodrst);
}

ALT_STATUS_CODE prepare_launch(launch_info_t * launch)
{
    ALT_STATUS_CODE status = ALT_E_SUCCESS;
    
#if CONFIG_PRELOADER_CHECKSUM_NEXT_IMAGE
    
    MPL_WATCHDOG();
    //
    // Verify image CRC
    //

    if (status == ALT_E_SUCCESS)
    {
        uint32_t crc_val = crc32(0, (uint8_t *)launch->image_addr, launch->image_size);
        if (crc_val != launch->image_crc)
        {
            ALT_PRINTF("ERROR: Image CRC mismatch.\n");
            status = ALT_E_ERROR;
        }
        
    }
#endif

    /////

    if (status == ALT_E_SUCCESS)
    {
        // Update the peripherals that needs to be taken out of reset.
        // Bridge peripherals are handled in update_handoff().
        alt_write_word(ALT_RSTMGR_PER0MODRST_ADDR,
            ALT_RSTMGR_PER0MODRST_EMAC0_SET(CONFIG_HPS_RESET_EMAC0) |
            ALT_RSTMGR_PER0MODRST_EMAC1_SET(CONFIG_HPS_RESET_EMAC1) |
            ALT_RSTMGR_PER0MODRST_EMAC2_SET(CONFIG_HPS_RESET_EMAC2) |
            ALT_RSTMGR_PER0MODRST_USB0_SET(CONFIG_HPS_RESET_USB0) |
            ALT_RSTMGR_PER0MODRST_USB1_SET(CONFIG_HPS_RESET_USB1) |
            ALT_RSTMGR_PER0MODRST_NAND_SET(CONFIG_HPS_RESET_NAND) |
            ALT_RSTMGR_PER0MODRST_QSPI_SET(CONFIG_HPS_RESET_QSPI) |
            ALT_RSTMGR_PER0MODRST_SDMMC_SET(CONFIG_HPS_RESET_SDMMC) |
            ALT_RSTMGR_PER0MODRST_EMAC0OCP_SET(CONFIG_HPS_RESET_EMAC0OCP) |
            ALT_RSTMGR_PER0MODRST_EMAC1OCP_SET(CONFIG_HPS_RESET_EMAC1OCP) |
            ALT_RSTMGR_PER0MODRST_EMAC2OCP_SET(CONFIG_HPS_RESET_EMAC2OCP) |
            ALT_RSTMGR_PER0MODRST_USB0OCP_SET(CONFIG_HPS_RESET_USB0OCP) |
            ALT_RSTMGR_PER0MODRST_USB1OCP_SET(CONFIG_HPS_RESET_USB1OCP) |
            ALT_RSTMGR_PER0MODRST_NANDOCP_SET(CONFIG_HPS_RESET_NANDOCP) |
            ALT_RSTMGR_PER0MODRST_QSPIOCP_SET(CONFIG_HPS_RESET_QSPIOCP) |
            ALT_RSTMGR_PER0MODRST_SDMMCOCP_SET(CONFIG_HPS_RESET_SDMMCOCP) |
            ALT_RSTMGR_PER0MODRST_DMA_SET(CONFIG_HPS_RESET_DMA) |
            ALT_RSTMGR_PER0MODRST_SPIM0_SET(CONFIG_HPS_RESET_SPIM0) |
            ALT_RSTMGR_PER0MODRST_SPIM1_SET(CONFIG_HPS_RESET_SPIM1) |
            ALT_RSTMGR_PER0MODRST_SPIS0_SET(CONFIG_HPS_RESET_SPIS0) |
            ALT_RSTMGR_PER0MODRST_SPIS1_SET(CONFIG_HPS_RESET_SPIS1) |
            ALT_RSTMGR_PER0MODRST_DMAOCP_SET(CONFIG_HPS_RESET_DMAOCP) |
            ALT_RSTMGR_PER0MODRST_EMACPTP_SET(CONFIG_HPS_RESET_EMACPTP) |
            ALT_RSTMGR_PER0MODRST_DMAIF0_SET(CONFIG_HPS_RESET_DMAIF0) |
            ALT_RSTMGR_PER0MODRST_DMAIF1_SET(CONFIG_HPS_RESET_DMAIF1) |
            ALT_RSTMGR_PER0MODRST_DMAIF2_SET(CONFIG_HPS_RESET_DMAIF2) |
            ALT_RSTMGR_PER0MODRST_DMAIF3_SET(CONFIG_HPS_RESET_DMAIF3) |
            ALT_RSTMGR_PER0MODRST_DMAIF4_SET(CONFIG_HPS_RESET_DMAIF4) |
            ALT_RSTMGR_PER0MODRST_DMAIF5_SET(CONFIG_HPS_RESET_DMAIF5) |
            ALT_RSTMGR_PER0MODRST_DMAIF6_SET(CONFIG_HPS_RESET_DMAIF6) |
            ALT_RSTMGR_PER0MODRST_DMAIF7_SET(CONFIG_HPS_RESET_DMAIF7));
       
        alt_write_word(ALT_RSTMGR_PER1MODRST_ADDR,
            ALT_RSTMGR_PER1MODRST_WD0_SET(CONFIG_HPS_RESET_WD0_SET) |
            ALT_RSTMGR_PER1MODRST_WD1_SET(CONFIG_HPS_RESET_WD1_SET) |
            ALT_RSTMGR_PER1MODRST_L4SYSTMR0_SET(CONFIG_HPS_RESET_L4SYSTMR0_SET) |
            ALT_RSTMGR_PER1MODRST_L4SYSTMR1_SET(CONFIG_HPS_RESET_L4SYSTMR1_SET) |
            ALT_RSTMGR_PER1MODRST_SPTMR0_SET(CONFIG_HPS_RESET_SPTMR0_SET) |
            ALT_RSTMGR_PER1MODRST_SPTMR1_SET(CONFIG_HPS_RESET_SPTMR1_SET) |
            ALT_RSTMGR_PER1MODRST_I2C0_SET(CONFIG_HPS_RESET_I2C0_SET) |
            ALT_RSTMGR_PER1MODRST_I2C1_SET(CONFIG_HPS_RESET_I2C1_SET) |
            ALT_RSTMGR_PER1MODRST_I2C2_SET(CONFIG_HPS_RESET_I2C2_SET) |
            ALT_RSTMGR_PER1MODRST_I2C3_SET(CONFIG_HPS_RESET_I2C3_SET) |
            ALT_RSTMGR_PER1MODRST_I2C4_SET(CONFIG_HPS_RESET_I2C4_SET) |
            ALT_RSTMGR_PER1MODRST_UART0_SET(CONFIG_HPS_RESET_UART0_SET) |
            ALT_RSTMGR_PER1MODRST_UART1_SET(CONFIG_HPS_RESET_UART1_SET) |
            ALT_RSTMGR_PER1MODRST_GPIO0_SET(CONFIG_HPS_RESET_GPIO0_SET) |
            ALT_RSTMGR_PER1MODRST_GPIO1_SET(CONFIG_HPS_RESET_GPIO1_SET) |
            ALT_RSTMGR_PER1MODRST_GPIO2_SET(CONFIG_HPS_RESET_GPIO2_SET));
    }

    return status;
}

int main()
{
    ALT_STATUS_CODE status = ALT_E_SUCCESS;
    launch_info_t launch;

    MPL_WATCHDOG();
    // do board initialization (plls, pinmux/io, etc.. )
    if (status == ALT_E_SUCCESS)
    {
        status = board_init();
    }

#ifdef CONFIG_MPL_FPGA_LOAD
    /* We do not have ram yet so we have to load to OCRAM */
    if (status == ALT_E_SUCCESS)
    {
 #ifdef QSPI_BOOT
        alt_wdog_uninit();
        status = qspi_fpga_program();
 #elif FAT_BOOT
        status = fat_load_fpga(CONFIG_MPL_FAT_LOAD_FPGA_NAME);
 #elif SDMMC_BOOT
        status = sdmmc_load_fpga(CONFIG_PRELOADER_FPGA_IMAGE_SDMMC_ADDR);
 #endif
    }
#endif

    MPL_WATCHDOG();

    if (status == ALT_E_SUCCESS)
    {
        status = sdram_init();
    }

    if (status == ALT_E_SUCCESS)
    {
#ifdef QSPI_BOOT
        ALT_PRINTF("MPL: Booting from QSPI.\n");
        alt_wdog_uninit();
        status = qspi_load(&launch);
#elif FAT_BOOT
        ALT_PRINTF("MPL: Booting from FAT.\n");
        status = fat_load(&launch);
#elif SDMMC_BOOT
        ALT_PRINTF("MPL: Booting from SDMMC.\n");
        status = sdmmc_load(&launch);
#elif !defined(NO_BOOT)
        // #elif NAND_BOOT
        // ALT_PRINTF("MPL: Booting from NAND.\n");
        // status = nand_load(&launch);
#error "No valid boot source set in alt_config.h It must be QSPI or SDMMC"
#endif
    }

    // Process the launch information.
    if (status == ALT_E_SUCCESS)
    {
        status = prepare_launch(&launch);
    }

    if (status == ALT_E_SUCCESS)
    {
        ALT_PRINTF("MPL: Launching next stage.\n");
        
        LOG_DONE();

        // Update the ISW Handoff registers.
        update_handoff();
#ifdef NO_BOOT
#warning For MPL based apps, this is where you would place your code

#else
        // Jump to entry point
        (*(FXN_PTR)(launch.entry_addr))();
#endif
    }

    ALT_PRINTF("ERROR: Error in boot process. Halting.\n");
    return status;
}
