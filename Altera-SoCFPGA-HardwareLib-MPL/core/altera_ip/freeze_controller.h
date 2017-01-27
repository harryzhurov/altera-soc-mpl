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
 * $Id: //acds/rel/16.1/embedded/examples/software/Altera-SoCFPGA-HardwareLib-MPL/core/altera_ip/freeze_controller.h#1 $
 */

#ifndef FREEZE_CONTROLLER_H_
#define FREEZE_CONTROLLER_H_

#include <socal/socal.h>
#include <socal/alt_sysmgr.h>
#include <socal/hps.h>


/* Freeze controller register mapping */
/*
Register        Offset  RegisterField  Corresponding Freeze Controller port
vioctrl[0]      0x20    cfg         iochannel0_csrdone
                        bushold     iochannel0_bhniotri_n
                        tristate    iochannel0_niotri_n
                        wkpullup    iochannel0_plniotri_n
                        slew        iochannel0_enrnsl_n
vioctrl[1]      0x24    cfg         iochannel1_csrdone
                        bushold     iochannel1_bhniotri_n
                        tristate    iochannel1_niotri_n
                        wkpullup    iochannel1_plniotri_n
                        slew        iochannel1_enrnsl_n
vioctrl[2]      0x28    cfg         iochannel2_csrdone
                        bushold     iochannel2_bhniotri_n
                        tristate    iochannel2_niotri_n
                        wkpullup    iochannel2_plniotri_n
                        slew        iochannel2_enrnsl_n
hioctrl         0x30    cfg         iochannel3_csrdone
                        bushold     iochannel3_bhniotri_n
                        tristate    iochannel3_niotri_n
                        wkpullup    iochannel3_plniotri_n
                        slew        iochannel3_enrnsl_n
                        dllrst      iochannel3_reinit
                        octrst      iochannel3_nfrzdrv_n
                        regrst      iochannel3_frzreg
                        oct_cfgen_calstart   iochannel3_pllbiasen
src             0x34    vio1        iochannel1_src_sel
hwctrl          0x38    vio1req     iochannel1_frz_thaw_request
                        vio1state   {iochannel1_frz_thaw_state_1,
                                    iochannel1_frz_thaw_state_0}
*/


/**
 * @enum FreezeChannelSelect
 * @brief Definition of enum for freeze channel
 *
 */
typedef enum  {
    FREEZE_CHANNEL_0 = 0,   /* EMAC_IO & MIXED2_IO */
    FREEZE_CHANNEL_1,   /* MIXED1_IO and FLASH_IO */
    FREEZE_CHANNEL_2,   /* General IO */
    FREEZE_CHANNEL_3,   /* DDR IO */
    FREEZE_CHANNEL_UNDEFINED
} FreezeChannelSelect;

/* Maximum number of freeze channel */
#define FREEZE_CHANNEL_NUM  (4)

/* Boolean value for TRUE/FALSE */
#define TRUE        (1)
#define FALSE       (0)

                
/**
 * @enum FreezeControllerFSMSelect
 * @brief Definition of enum for freeze controller state machine
 *
 */
typedef enum {
    FREEZE_CONTROLLER_FSM_SW = 0,
    FREEZE_CONTROLLER_FSM_HW,
    FREEZE_CONTROLLER_FSM_UNDEFINED
} FreezeControllerFSMSelect;

/* Shift count needed to calculte for FRZCTRL VIO control register offset */
#define SYSMGR_FRZCTRL_VIOCTRL_SHIFT    (2)

/**
 * @fn sys_mgr_frzctrl_freeze_req
 *
 * @ brief Freeze HPS IOs
 *
 * @param channel_id @ref FreezeChannelSelect [in] - Freeze channel ID 
 * @param fsm_select @ref FreezeControllerFSMSelect [in] - To use hardware or 
 *        software state machine
 * @note If FREEZE_CONTROLLER_FSM_HW is selected for FSM select then the 
 *       the freeze channel id is input is ignored. It is default to channel 1
 */
extern uint32_t 
sys_mgr_frzctrl_freeze_req (
    FreezeChannelSelect channel_id,
    FreezeControllerFSMSelect fsm_select);

/**
 * @fn sys_mgr_frzctrl_thaw_req
 *
 * @brief Unfreeze/Thaw HPS IOs
 *
 * @param channel_id @ref FreezeChannelSelect [in] - Freeze channel ID 
 * @param fsm_select @ref FreezeControllerFSMSelect [in] - To use hardware or 
 *        software state machine
 * @note If FREEZE_CONTROLLER_FSM_HW is selected for FSM select then the 
 *       the freeze channel id is input is ignored. It is default to channel 1
 */
extern uint32_t 
sys_mgr_frzctrl_thaw_req (
    FreezeChannelSelect channel_id,
    FreezeControllerFSMSelect fsm_select);

/**
 * @fn sys_mgr_frzctrl_frzchn_is_frozen
 * 
 * @brief Get current state of freeze channel 
 *
 * @param channel_id @ref FreezeChannelSelect [in] - Freeze channel ID 
 * @return uint32_t [out] - Current freeze channel status, TRUE if frozen, 
 *         0 otherwise
 */
extern uint32_t sys_mgr_frzctrl_frzchn_is_frozen (FreezeChannelSelect channel_id); 


/**
 * Get current state of freeze channel 1 (VIO)
 * @return uint32_t Current freeze channel 1 status
 */
static inline uint32_t sys_mgr_frzctrl_frzchn1_state_get (void) {
    uint32_t frzchn1_state;

    frzchn1_state = alt_read_word(ALT_SYSMGR_FRZCTL_HWCTL_ADDR);

    frzchn1_state = ALT_SYSMGR_FRZCTL_HWCTL_VIO1STATE_GET(frzchn1_state);
                       
    return frzchn1_state;
}

#endif
