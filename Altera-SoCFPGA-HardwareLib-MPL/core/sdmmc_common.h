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
 * $Id: //acds/rel/16.1/embedded/examples/software/Altera-SoCFPGA-HardwareLib-MPL/core/sdmmc_common.h#2 $
 */

/*! \file
 *  Altera - SDMMC Common Header definitions
 */

#ifndef _SDMMC_COMMON_H
#define _SDMMC_COMMON_H

// common MBR 
#define MBR_SIGNATURE   0xAA55
#define MBR_SIG_ADDR    0x1FE
#define MBR_P1_ADDR     0x1BE
#define MBR_P2_ADDR     0x1CE
#define MBR_P3_ADDR     0x1DE
#define MBR_P4_ADDR     0x1EE
#define MBR_PENTRY_SZ   0x10


// matching MBR (master boot record) partition struct
typedef struct mbr_partition_entry {
    uint8_t     bootable;   // Boot Indicator (0x80 = bootable)
    uint8_t     schs_b1;    // starting chs value byte1
    uint8_t     schs_b2;    // starting chs value byte2
    uint8_t     schs_b3;    // starting chs value byte3
    uint8_t     p_type;     // Partition-Type
    uint8_t     echs_b1;    // ending chs value byte1
    uint8_t     echs_b2;    // ending chs value byte2
    uint8_t     echs_b3;    // ending chs value byte3
    uint16_t    lba_hw1;    // logical block address of first sector in partition
    uint16_t    lba_hw2;    // split into half-words since partition addresses are hot aligned on word boundary
    uint16_t    size_hw1;   // number of sectors in partition
    uint16_t    size_hw2;
} mbr_partition_entry_t;

ALT_STATUS_CODE sdmmc_initialize(void);
extern ALT_SDMMC_CARD_INFO_t card_info;

#define SDMMC_BLOCK_SZ  512
#define SDMMC_BLK_ALIGN_MSK 0xFFFFFE00
// This is the largest chunk read before handling the watchdog.
#define SDMMC_CHUNK_SZ  (512 * 1024)


#endif
