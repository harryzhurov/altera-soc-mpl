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
 * $Id: //acds/rel/16.1/embedded/examples/software/Altera-SoCFPGA-HardwareLib-MPL/core/altera_ip/scan_manager.c#1 $
 */

#include <freeze_controller.h>
#include <scan_manager.h>

/**
 * @fn scan_mgr_io_scan_chain_prg
 * @brief Program HPS IO Scan Chain
 */
ALT_STATUS_CODE scan_mgr_io_scan_chain_prg (
    IOScanChainSelect io_scan_chain_id,
    uint32_t io_scan_chain_len_in_bits, 
    const unsigned long *iocsr_scan_chain)
{
        
    uint16_t tdi_tdo_header;
    uint32_t io_program_iter;
    uint32_t io_scan_chain_data_residual;
    uint32_t residual;
    uint32_t i;
    uint32_t index = 0;

#ifdef SCANMGR_DEBUG                                            
    printf ("Entering scan_mgr_io_scan_chain_prg.\n");
    printf ("Program IO scan chain %x, length = %d bits\n", 
        io_scan_chain_id,
        io_scan_chain_len_in_bits);
#endif //SCANMGR_DEBUG 

    /* Check if IO bank is in frozen state before proceed to program IO 
     * scan chain. 
     * Note: IO scan chain ID is 1:1 mapping to freeze channel ID
     */
    if (sys_mgr_frzctrl_frzchn_is_frozen ((FreezeChannelSelect)io_scan_chain_id)) {
        
        /* De-assert reinit if the IO scan chain is intended for HIO */
        if (IO_SCAN_CHAIN_3 == io_scan_chain_id) {

            alt_clrbits_word(ALT_SYSMGR_FRZCTL_HIOCTL_ADDR, ALT_SYSMGR_FRZCTL_HIOCTL_DLLRST_SET_MSK);
        } /* if (HIO) */
        
        /* Check if the scan chain engine is inactive and the WFIFO is empty
         * before enabling the IO scan chain
         */
        if (SCAN_MGR_IO_SCAN_ENGINE_STATUS_IDLE != 
                scan_mgr_io_scan_chain_engine_is_idle (MAX_WAITING_DELAY_IO_SCAN_ENGINE)) {
           //printf ("scan_mgr_io_scan_chain_prg : " \
                "Unable to program IO due to IO scan chain engine is active\n");
            return ALT_E_ERROR;                    
        }
        
        /* Enable IO Scan chain based on scan chain id
         * Note: only one chain can be enabled at a time 
         */
        alt_setbits_word(ALT_SCANMGR_EN_ADDR, (1 << io_scan_chain_id));
        
        /* Calculate number of iteration needed for full 128-bit (4 x32-bits)
           bits shifting. Each TDI_TDO packet can shift in maximum 128-bits */
        io_program_iter = io_scan_chain_len_in_bits >> IO_SCAN_CHAIN_128BIT_SHIFT;
        io_scan_chain_data_residual = io_scan_chain_len_in_bits & IO_SCAN_CHAIN_128BIT_MASK;

#ifdef SCANMGR_DEBUG                                            
        printf ("128-bit IO scan chain iter: %d, residual bits: %d\n", 
            io_program_iter,
            io_scan_chain_data_residual);
#endif //SCANMGR_DEBUG 

        /* Construct TDI_TDO packet for 128-bit IO scan chain (2 bytes) */
        tdi_tdo_header = TDI_TDO_HEADER_FIRST_BYTE | (TDI_TDO_MAX_PAYLOAD << TDI_TDO_HEADER_SECOND_BYTE_SHIFT);
          
        /* Program IO scan chain in 128-bit iteration */
        for (i = 0; i < io_program_iter; i++) {
       
            /* write TDI_TDO packet header to scan manager */
            alt_write_word(ALT_SCANMGR_FIFODOUBLEBYTE_ADDR, tdi_tdo_header);                            

            /* calculate array index */
            index = i * 4;
            
            /* write 4 successive 32-bit IO scan chain data into WFIFO */
            alt_write_word(ALT_SCANMGR_FIFOQUADBYTE_ADDR, iocsr_scan_chain[index]);
            alt_write_word(ALT_SCANMGR_FIFOQUADBYTE_ADDR, iocsr_scan_chain[index + 1]);
            alt_write_word(ALT_SCANMGR_FIFOQUADBYTE_ADDR, iocsr_scan_chain[index + 2]);
            alt_write_word(ALT_SCANMGR_FIFOQUADBYTE_ADDR, iocsr_scan_chain[index + 3]);

#ifdef SCANMGR_DEBUG                                            
            printf ("%d : Write TDI_TDO_header = %04x, " \
                "iocsr_scan_chain_table index = %d\n", 
                i, tdi_tdo_header, index);
                                        
            printf ("Writing %08x %08x %08x %08x from table[%d..%d]\n", 
                iocsr_scan_chain[index],
                iocsr_scan_chain[index + 1], 
                iocsr_scan_chain[index + 2], 
                iocsr_scan_chain[index + 3], 
                index, index + 3);
#endif //SCANMGR_DEBUG 
                            
            /* Check if the scan chain engine has completed the IO scan chain 
             * data shifting 
             */
            if (SCAN_MGR_IO_SCAN_ENGINE_STATUS_IDLE 
                != scan_mgr_io_scan_chain_engine_is_idle (MAX_WAITING_DELAY_IO_SCAN_ENGINE)) {
                
                /* Disable IO Scan chain when error detected */
                alt_clrbits_word(ALT_SCANMGR_EN_ADDR, (1 << io_scan_chain_id));
                
                //puts ("scan_mgr_io_scan_chain_prg : " \
                    "Unable to complete IO scan chain programming.\n");
                return ALT_E_ERROR;
            }
        }

        /* Calculate array index for final TDI_TDO packet */
        index = io_program_iter * 4;
        
        /* Final TDI_TDO packet if any */
        if (0 != io_scan_chain_data_residual) {
            
            /* Calculate number of quad bytes FIFO write needed for the
             * final TDI_TDO packet 
             */
            io_program_iter = io_scan_chain_data_residual >> IO_SCAN_CHAIN_32BIT_SHIFT;
            
            /* Construct TDI_TDO packet for remaining IO scan chain (2 bytes) */
            tdi_tdo_header = TDI_TDO_HEADER_FIRST_BYTE | 
                    ((io_scan_chain_data_residual - 1) << TDI_TDO_HEADER_SECOND_BYTE_SHIFT);
                            
            /* Program the last part of IO scan chain */            
            /* write TDI_TDO packet header (2 bytes) to scan manager */
            alt_write_word(ALT_SCANMGR_FIFODOUBLEBYTE_ADDR, tdi_tdo_header);  

#ifdef SCANMGR_DEBUG                                            
            printf ("Final TDI_TDO_header = %04x, " \
                "iocsr_scan_chain_table index = %d, iter = %d\n", 
                tdi_tdo_header, index, io_program_iter);
#endif //SCANMGR_DEBUG 
            
            for (i = 0; i < io_program_iter; i++) {
                /* write remaining scan chain data into scan manager WFIFO 
                 * with 4 bytes write
                 */
                alt_write_word(ALT_SCANMGR_FIFOQUADBYTE_ADDR, iocsr_scan_chain[index + i]);

#ifdef SCANMGR_DEBUG                                            
                printf ("%d: Writing %08x from table[%d]\n", 
                    i, iocsr_scan_chain[index + i], index + i);
#endif //SCANMGR_DEBUG  
            }
            
            index += io_program_iter;
            residual = io_scan_chain_data_residual & IO_SCAN_CHAIN_32BIT_MASK;
            
            if (IO_SCAN_CHAIN_PAYLOAD_24BIT < residual) {
                /* write the last 4B scan chain data into scan manager WFIFO */
                alt_write_word(ALT_SCANMGR_FIFOQUADBYTE_ADDR, iocsr_scan_chain[index]);

#ifdef SCANMGR_DEBUG                                            
                printf ("%d: Writing %08x from table[%d]\n", 
                    i, iocsr_scan_chain[index], index);
#endif //SCANMGR_DEBUG  

            } else {
                /* write the remaining 1 - 3 bytes scan chain data into scan
                 * manager WFIFO byte by byte to prevent JTAG engine shifting 
                 * unused data from the FIFO and mistaken the data as a valid 
                 * command (even though unused bits are set to 0, but just to 
                 * prevent hardware glitch)
                 */
                for (i = 0; i < residual; i += 8) {
                    alt_write_word(ALT_SCANMGR_FIFOSINGLEBYTE_ADDR,
                    ((iocsr_scan_chain[index] >> i) & IO_SCAN_CHAIN_BYTE_MASK));

#ifdef SCANMGR_DEBUG                                            
                printf ("%d: Writing %02x from table[%d]\n", i, 
                    ((iocsr_scan_chain[index] >> i) & IO_SCAN_CHAIN_BYTE_MASK), 
                    index);
#endif //SCANMGR_DEBUG  
                }
            }
                                    
            /* Check if the scan chain engine has completed the IO scan chain 
             * data shifting 
             */
            if (SCAN_MGR_IO_SCAN_ENGINE_STATUS_IDLE 
                !=  scan_mgr_io_scan_chain_engine_is_idle (MAX_WAITING_DELAY_IO_SCAN_ENGINE)) {
                
                /* Disable IO Scan chain when error detected */
                alt_clrbits_word(ALT_SCANMGR_EN_ADDR, (1 << io_scan_chain_id));

                //puts ("scan_mgr_io_scan_chain_prg : " \
                    "Unable to complete IO scan chain programming.\n");
                return ALT_E_ERROR;
            }                           
        } /* if (io_scan_chain_data_residual) */
        
        /* Disable IO Scan chain when configuration done*/
        alt_clrbits_word(ALT_SCANMGR_EN_ADDR, (1 << io_scan_chain_id));        
    } else {
        /* IOCSR configuration failed as IO bank is not in frozen state */
        //puts ("scan_mgr_io_scan_chain_prg: FAIL, IO bank not in frozen state!\n");
        return ALT_E_ERROR;
    } /* if-else (frozen) */

#ifdef SCANMGR_DEBUG                                            
    printf ("Exiting scan_mgr_io_scan_chain_prg: IO chain %x is programmed\n", 
        io_scan_chain_id);
#endif //SCANMGR_DEBUG 
           
    return ALT_E_SUCCESS;
}
