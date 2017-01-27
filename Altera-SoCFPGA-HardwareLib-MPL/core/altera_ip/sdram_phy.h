/******************************************************************************
*
* Copyright 2012 Altera Corporation. All Rights Reserved.
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
 * $Id: //acds/rel/16.1/embedded/examples/software/Altera-SoCFPGA-HardwareLib-MPL/core/altera_ip/sdram_phy.h#1 $
 */

#ifndef _SDRAM_PHY_H_
#define _SDRAM_PHY_H_

#include "socal/socal.h"
#include "socal/hps.h"

#define SDR_PHYGRP_SCCGRP_ADDRESS       0x0
#define SDR_PHYGRP_PHYMGRGRP_ADDRESS    0x1000
#define SDR_PHYGRP_RWMGRGRP_ADDRESS     0x2000
#define SDR_PHYGRP_DATAMGRGRP_ADDRESS   0x4000
#define SDR_PHYGRP_REGFILEGRP_ADDRESS   0x4800
#define SDR_CTRLGRP_ADDRESS     ALT_SDR_CTL_OFST

// other needed defines from csr
#define SDR_CTRLGRP_PHYCTRL_PHYCTRL_0_OFFSET 0x150
#define SDR_CTRLGRP_PHYCTRL_PHYCTRL_0_SAMPLECOUNT_19_0_WIDTH 20
#define SDR_CTRLGRP_PHYCTRL_PHYCTRL_1_OFFSET 0x154
#define SDR_CTRLGRP_PHYCTRL_PHYCTRL_1_LONGIDLESAMPLECOUNT_19_0_WIDTH 20
#define SDR_CTRLGRP_PHYCTRL_PHYCTRL_2_OFFSET 0x158
#define SDR_CTRLGRP_PHYCTRL_PHYCTRL_0_ACDELAYEN_SET(x) ((x) & 0x00000003)
#define SDR_CTRLGRP_PHYCTRL_PHYCTRL_0_ADDLATSEL_SET(x) \
 (((x) << 10) & 0x00000c00)
#define SDR_CTRLGRP_PHYCTRL_PHYCTRL_0_DQDELAYEN_SET(x) \
 (((x) << 2) & 0x0000000c)
#define SDR_CTRLGRP_PHYCTRL_PHYCTRL_0_DQSDELAYEN_SET(x) \
 (((x) << 4) & 0x00000030)
#define SDR_CTRLGRP_PHYCTRL_PHYCTRL_0_DQSLOGICDELAYEN_SET(x) \
 (((x) << 6) & 0x000000c0)
#define SDR_CTRLGRP_PHYCTRL_PHYCTRL_0_LPDDRDIS_SET(x) \
 (((x) << 9) & 0x00000200)
#define SDR_CTRLGRP_PHYCTRL_PHYCTRL_0_RESETDELAYEN_SET(x) \
 (((x) << 8) & 0x00000100)
#define SDR_CTRLGRP_PHYCTRL_PHYCTRL_0_SAMPLECOUNT_19_0_SET(x) \
 (((x) << 12) & 0xfffff000)
#define SDR_CTRLGRP_PHYCTRL_PHYCTRL_1_LONGIDLESAMPLECOUNT_19_0_SET(x) \
 (((x) << 12) & 0xfffff000)
#define SDR_CTRLGRP_PHYCTRL_PHYCTRL_1_SAMPLECOUNT_31_20_SET(x) \
 ((x) & 0x00000fff)
#define SDR_CTRLGRP_PHYCTRL_PHYCTRL_2_LONGIDLESAMPLECOUNT_31_20_SET(x) \
 ((x) & 0x00000fff)




#define HPS_SDR_BASE    ALT_SDR_OFST

#define write_register(BASE, OFFSET, DATA) \
    alt_write_word(((BASE) + (OFFSET)), DATA)

#define read_register(BASE, OFFSET) \
    alt_read_word((BASE) + (OFFSET))


#endif /* _SDRAM_PHY_H_ */
