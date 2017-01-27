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
 * $Id: //acds/rel/16.1/embedded/examples/software/Altera-SoCFPGA-HardwareLib-MPL/alt_config.h#1 $
 */

#ifndef ALT_CONFIG_H

#ifdef soc_cv_av
// configuration from handoff
 #include <sdram/sdram_config.h>  // SDRAM configuration
 #include <sequencer_defines.h>  // SDRAM calibration files
 #include <pinmux_config.h>  // Pin muxing configuration
 #include <pll_config.h>     // PLL configuration
 #ifdef CONFIG_SOCFPGA_ARRIA5
  #include <iocsr_config_arria5.h>    // IO bank configuration
 #else
  #include <iocsr_config_cyclone5.h>  // IO bank configuration
 #endif
#else
#define CONFIG_EXT_OSC1_FREQ    25000000
#define CONFIG_INTOSC_HS_DIV2   100000000
#define CONFIG_FREE_CLK         100000000
#endif


/* List of qspi devices that are enabled
   Disabling devices reduces program and data sizes */

#define ALT_QSPI_SUPPORT_MICRON_M25P40          0 // aka ALTERA EPCS4
#define ALT_QSPI_SUPPORT_MICRON_M25P16          0 // aka ALTERA EPCS16
#define ALT_QSPI_SUPPORT_MICRON_M25PX16         0
#define ALT_QSPI_SUPPORT_SPANSION_S25FL116K     0
#define ALT_QSPI_SUPPORT_SPANSION_S25FL256S     0
#define ALT_QSPI_SUPPORT_SPANSION_S25FL512S     0
#define ALT_QSPI_SUPPORT_MACRONIX_MX25L25635    0
#define ALT_QSPI_SUPPORT_MACRONIX_MX66L51235    0
#define ALT_QSPI_SUPPORT_MICRON_N25Q128         0
#define ALT_QSPI_SUPPORT_MICRON_N25Q512A        1
#define ALT_QSPI_SUPPORT_MICRON_N25Q00AA        0

#endif /* ifndef alt_config.h */
