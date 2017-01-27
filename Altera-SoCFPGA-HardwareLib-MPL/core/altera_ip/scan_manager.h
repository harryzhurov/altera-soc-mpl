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
 * $Id: //acds/rel/16.1/embedded/examples/software/Altera-SoCFPGA-HardwareLib-MPL/core/altera_ip/scan_manager.h#1 $
 */

#ifndef SCAN_MANAGER_H_
#define SCAN_MANAGER_H_

#include <socal/socal.h>
#include <socal/hps.h>
#include <socal/alt_scanmgr.h>
#include <hwlib.h>

/**
 * @enum IOScanChainSelect
 * @brief Definition of enum for IO scan chain ID
 *
 */
typedef enum  {
    IO_SCAN_CHAIN_0 = 0,    /* EMAC_IO and MIXED2_IO */
    IO_SCAN_CHAIN_1,        /* MIXED1_IO and FLASH_IO */
    IO_SCAN_CHAIN_2,        /* General IO */
    IO_SCAN_CHAIN_3,        /* DDR IO */
    IO_SCAN_CHAIN_UNDEFINED
} IOScanChainSelect;

#define IO_SCAN_CHAIN_NUM (4)               /**< Maximum number of IO scan chains 
                                             */
#define IO_SCAN_CHAIN_128BIT_SHIFT (7)      /**< Shift count to get number of IO
                                             * scan chain data in granularity 
                                             * of 128-bit ( N / 128 )
                                             */
#define IO_SCAN_CHAIN_128BIT_MASK (0x7F)    /**< Mask to get residual IO scan 
                                             * chain data in granularity 
                                             * of 128-bit ( N mod 128 )
                                             */
#define IO_SCAN_CHAIN_32BIT_SHIFT (5)       /**< Shift count to get number of IO
                                             * scan chain data in granularity 
                                             * of 32-bit ( N / 32 )
                                             */
#define IO_SCAN_CHAIN_32BIT_MASK (0x1F)     /**< Mask to get residual IO scan 
                                             * chain data in granularity 
                                             * of 32-bit ( N mod 32 )
                                             */
#define IO_SCAN_CHAIN_BYTE_MASK (0xFF)      /**< Byte mask */                                             
#define IO_SCAN_CHAIN_PAYLOAD_24BIT (24)    /**< 24-bits (3 bytes) IO scan chain 
                                             * payload definition
                                             */
#define TDI_TDO_MAX_PAYLOAD (127)           /**< Maximum length of TDI_TDO 
                                             * packet payload is 128 bits, 
                                             * represented by (length - 1) in 
                                             * TDI_TDO header
                                             */
#define TDI_TDO_HEADER_FIRST_BYTE (0x80)    /**< TDI_TDO packet header for IO 
                                             * scan chain program
                                             */
#define TDI_TDO_HEADER_SECOND_BYTE_SHIFT (8)/**< Position of second command byte
                                             * for TDI_TDO packet
                                             */
#define SCAN_MGR_IO_SCAN_ENGINE_STATUS_IDLE (0)     /**< IO scan chain engine is
                                                     * is idle
                                                     */
#define SCAN_MGR_IO_SCAN_ENGINE_STATUS_ACTIVE (1)   /**< IO scan chain engine is
                                                     * is active
                                                     */
#define MAX_WAITING_DELAY_IO_SCAN_ENGINE (100)/**< Maximum polling loop to wait 
                                               * for IO scan chain engine 
                                               * becomes idle to prevent 
                                               * infinite loop
                                               */

extern uint32_t iocsr_scan_config(void);

/**
 * @fn scan_mgr_io_scan_chain_prg
 * @brief Program HPS IO Scan Chain
 *  @param io_scan_chain_id @ref IOScanChainSelect [in] - IO scan chain ID with 
 *        range of enumIOScanChainSelect *        
 *  @param io_scan_chain_len_in_bits uint32_t [in] - IO scan chain length in bits
 *  @param *iocsr_scan_chain @ref Scan_mgr_entry_t [in] - IO scan chain table
 */
extern ALT_STATUS_CODE scan_mgr_io_scan_chain_prg (
    IOScanChainSelect io_scan_chain_id,
    uint32_t io_scan_chain_len_in_bits, 
    const unsigned long *iocsr_scan_chain);

/** 
 * @fn scan_mgr_io_scan_chain_engine_is_idle
 * @brief Inline function to check IO scan chain engine status and wait if the 
 *        engine is active. Poll the IO scan chain engine till maximum iteration 
 *        reached.  
 *  @param max_iter uint32_t [in] - maximum polling loop to revent infinite loop
 */
static inline uint32_t scan_mgr_io_scan_chain_engine_is_idle (uint32_t max_iter)
{
    uint32_t scanmgr_status;
    
    scanmgr_status = alt_read_word(ALT_SCANMGR_STAT_ADDR);

    /* Poll the engine until the scan engine is inactive */                            
    while (ALT_SCANMGR_STAT_ACT_GET(scanmgr_status) || (ALT_SCANMGR_STAT_WFIFOCNT_GET(scanmgr_status) > 0)) {
       max_iter--;
       if (max_iter > 0) {
            scanmgr_status = alt_read_word(ALT_SCANMGR_STAT_ADDR);
        } else {
            return SCAN_MGR_IO_SCAN_ENGINE_STATUS_ACTIVE;
        }
    }   
    return SCAN_MGR_IO_SCAN_ENGINE_STATUS_IDLE;
}

#endif
