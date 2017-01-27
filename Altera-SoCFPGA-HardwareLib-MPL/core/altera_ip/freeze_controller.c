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
 * $Id: //acds/rel/16.1/embedded/examples/software/Altera-SoCFPGA-HardwareLib-MPL/core/altera_ip/freeze_controller.c#1 $
 */

#include "hwlib.h"
#include "socal/alt_tmr.h"
#include <freeze_controller.h>

#define SYSMGR_FRZCTRL_LOOP_PARAM       (1000)
#define SYSMGR_FRZCTRL_DELAY_LOOP_PARAM (10)
#define SYSMGR_FRZCTRL_INTOSC_33	(33) 
#define SYSMGR_FRZCTRL_INTOSC_1000	(1000) 

typedef enum
{
    FREEZE_CTRL_FROZEN = 0,
    FREEZE_CTRL_THAWED = 1
} FREEZE_CTRL_CHAN_STATE; 
/* Default state from cold reset is FREEZE_ALL; the global 
   flag is set to TRUE to indicate the IO banks are frozen */
static uint32_t frzctrl_channel_freeze[FREEZE_CHANNEL_NUM] 
	= { FREEZE_CTRL_FROZEN, FREEZE_CTRL_FROZEN, 
	    FREEZE_CTRL_FROZEN, FREEZE_CTRL_FROZEN};


/**
 * @fn sys_mgr_frzctrl_freeze_req
 *
 * @ brief Freeze HPS IOs
 *
 */
uint32_t 
sys_mgr_frzctrl_freeze_req (
    FreezeChannelSelect channel_id,
    FreezeControllerFSMSelect fsm_select) 
{
    uint32_t frzctrl_ioctrl_reg_offset;
    uint32_t frzctrl_reg_value;
    uint32_t frzctrl_reg_cfg_mask;
    uint32_t i;

#ifdef FRZCTRL_DEBUG    
    printf ("Send Freeze Request to channel %d FSMselect %d\n", 
        channel_id, fsm_select);
#endif //FRZCTRL_DEBUG    

    if (FREEZE_CONTROLLER_FSM_SW == fsm_select) {

#ifdef FRZCTRL_DEBUG 
        puts ("FrzReq: Software FSM selected\n");
#endif //FRZCTRL_DEBUG    
        
        // select software FSM
        alt_write_word(ALT_SYSMGR_FRZCTL_SRC_ADDR, ALT_SYSMGR_FRZCTL_SRC_VIO1_E_SW);
        
        // Freeze channel ID checking and base address
        switch (channel_id){
            case FREEZE_CHANNEL_0:
            case FREEZE_CHANNEL_1:
            case FREEZE_CHANNEL_2:
                frzctrl_ioctrl_reg_offset
                    = ALT_SYSMGR_FRZCTL_OFST + ALT_SYSMGR_FRZCTL_VIOCTL_OFST +
                            (channel_id << SYSMGR_FRZCTRL_VIOCTRL_SHIFT);

#ifdef FRZCTRL_DEBUG
                printf ("Frzreq: IOCTRL addr %x\n", 
                		ALT_SYSMGR_OFST + frzctrl_ioctrl_reg_offset);
#endif //FRZCTRL_DEBUG 
                
                // Assert active low enrnsl, plniotri and niotri signals 
                frzctrl_reg_cfg_mask 
                    = ALT_SYSMGR_FRZCTL_VIOCTL_SLEW_SET_MSK 
                        | ALT_SYSMGR_FRZCTL_VIOCTL_WKPULLUP_SET_MSK
                        | ALT_SYSMGR_FRZCTL_VIOCTL_TRISTATE_SET_MSK;
                alt_clrbits_word( ALT_SYSMGR_OFST + frzctrl_ioctrl_reg_offset, frzctrl_reg_cfg_mask);

                // Note: Delay for 20ns at min
                // Assert active low bhniotri signal and de-assert active high
                // csrdone 
                frzctrl_reg_cfg_mask 
                    = ALT_SYSMGR_FRZCTL_VIOCTL_BUSHOLD_SET_MSK 
                        | ALT_SYSMGR_FRZCTL_VIOCTL_CFG_SET_MSK;
                alt_clrbits_word (ALT_SYSMGR_OFST + frzctrl_ioctrl_reg_offset,
                    frzctrl_reg_cfg_mask);

                // Insert short delay to allow freeze signal propagation
                //delay (SYSMGR_FRZCTRL_DELAY_LOOP_PARAM);                   
                for(i=SYSMGR_FRZCTRL_DELAY_LOOP_PARAM; i>0; i--); 
                
                // Set global flag to indicate channel is frozen
                frzctrl_channel_freeze[channel_id] = FREEZE_CTRL_FROZEN;
                
                break;
                
            case FREEZE_CHANNEL_3:

                // Assert active low enrnsl, plniotri and niotri signals 
                frzctrl_reg_cfg_mask 
                    = ALT_SYSMGR_FRZCTL_HIOCTL_SLEW_SET_MSK
                        | ALT_SYSMGR_FRZCTL_HIOCTL_WKPULLUP_SET_MSK
                        | ALT_SYSMGR_FRZCTL_HIOCTL_TRISTATE_SET_MSK;
                alt_clrbits_word(ALT_SYSMGR_FRZCTL_HIOCTL_ADDR, frzctrl_reg_cfg_mask);                                

                // Note: Delay for 40ns at min
                // assert active low bhniotri, de-assert active high csrdone 
                // and assert active high frzreg and nfrzdrv signals
                frzctrl_reg_value = alt_read_word(ALT_SYSMGR_FRZCTL_HIOCTL_ADDR);
		
                frzctrl_reg_cfg_mask 
                    = ALT_SYSMGR_FRZCTL_HIOCTL_BUSHOLD_SET_MSK
                        | ALT_SYSMGR_FRZCTL_HIOCTL_CFG_SET_MSK;
                frzctrl_reg_value 
                    = (frzctrl_reg_value & ~frzctrl_reg_cfg_mask)
                        | ALT_SYSMGR_FRZCTL_HIOCTL_REGRST_SET_MSK
                        | ALT_SYSMGR_FRZCTL_HIOCTL_OCTRST_SET_MSK;
                alt_write_word (ALT_SYSMGR_FRZCTL_HIOCTL_ADDR, frzctrl_reg_value);
                
                // Note: Delay for 40ns at min
                // assert active high reinit signal and de-assert active high
                // pllbiasen signals
		        frzctrl_reg_value = alt_read_word (ALT_SYSMGR_FRZCTL_HIOCTL_ADDR);
                frzctrl_reg_value 
                    = (frzctrl_reg_value
                        & ~ALT_SYSMGR_FRZCTL_HIOCTL_OCT_CFGEN_CALSTART_SET_MSK)
                        | ALT_SYSMGR_FRZCTL_HIOCTL_DLLRST_SET_MSK;
                alt_write_word (ALT_SYSMGR_FRZCTL_HIOCTL_ADDR, frzctrl_reg_value);
                    
                // Insert short delay to allow freeze signal propagation,
                //delay (SYSMGR_FRZCTRL_DELAY_LOOP_PARAM);
                for(i=SYSMGR_FRZCTRL_DELAY_LOOP_PARAM; i>0; i--); 
                
                // Set global flag to indicate channel is frozen
                frzctrl_channel_freeze[channel_id] = FREEZE_CTRL_FROZEN;
                
                break;
                
            default:
                //puts ("sys_mgr_frzctrl_freeze_req: invalid channel ID\n");
                return 1;
        }
    } else if (FREEZE_CONTROLLER_FSM_HW == fsm_select){

#ifdef FRZCTRL_DEBUG 
        puts ("FrzReq: HW FSM selected\n");
#endif //FRZCTRL_DEBUG         

        // select hardware FSM
        alt_write_word (ALT_SYSMGR_FRZCTL_SRC_ADDR, ALT_SYSMGR_FRZCTL_SRC_VIO1_E_HW);

        // write to hwctrl reg to enable freeze req
        alt_setbits_word(ALT_SYSMGR_FRZCTL_HWCTL_ADDR,
        		ALT_SYSMGR_FRZCTL_HWCTL_VIO1REQ_SET_MSK);
		
		i = 0;
		while (ALT_SYSMGR_FRZCTL_HWCTL_VIO1STATE_E_FROZEN
		        != sys_mgr_frzctrl_frzchn1_state_get()){
			
			i++;
			
			if (SYSMGR_FRZCTRL_LOOP_PARAM < i) {
			    //puts ("sys_mgr_frzctrl_freeze_req: unable to freeze\n");
				return 1;
			}
		} // while ( not frozen)
		
		// Set global flag to indicate channel is frozen
        frzctrl_channel_freeze[channel_id] = FREEZE_CTRL_FROZEN;     
    } else {
        //puts ("sys_mgr_frzctrl_freeze_req: invalid request\n");
        return 1;
    }// if-else (fsm_select)

    return 0;
}

/**
 * @fn sys_mgr_frzctrl_thaw_req
 *
 * @brief Unfreeze/Thaw HPS IOs
 *
 */
uint32_t 
sys_mgr_frzctrl_thaw_req (
    FreezeChannelSelect channel_id,
    FreezeControllerFSMSelect fsm_select) 
{
    uint32_t frzctrl_ioctrl_reg_offset;
    uint32_t frzctrl_reg_cfg_mask;
    uint32_t frzctrl_reg_value;
    uint32_t i;
    uint32_t start_count, clk_cycles_count;

#ifdef FRZCTRL_DEBUG    
    printf ("Send Thaw Request to channel %d FSMselect %d\n", channel_id, fsm_select);
#endif //FRZCTRL_DEBUG  

    if (FREEZE_CONTROLLER_FSM_SW == fsm_select) {

#ifdef FRZCTRL_DEBUG
        puts ("ThawReq: Software FSM selected\n");
#endif //FRZCTRL_DEBUG        
		
		// select software FSM
        alt_write_word(ALT_SYSMGR_FRZCTL_SRC_ADDR, ALT_SYSMGR_FRZCTL_SRC_VIO1_E_SW);

		// Freeze channel ID checking and base address
        switch (channel_id){
            case FREEZE_CHANNEL_0:
            case FREEZE_CHANNEL_1:
            case FREEZE_CHANNEL_2:
            	
            	frzctrl_ioctrl_reg_offset
                	= ALT_SYSMGR_FRZCTL_OFST + ALT_SYSMGR_FRZCTL_VIOCTL_OFST +
                	(channel_id << SYSMGR_FRZCTRL_VIOCTRL_SHIFT);    
                
#ifdef FRZCTRL_DEBUG
                printf ("ThawReq: IOCTRL addr %x\n", 
                    ALT_SYSMGR_OFST + frzctrl_ioctrl_reg_offset);
#endif //FRZCTRL_DEBUG 
                
                // Assert active low bhniotri signal and de-assert active high
                // csrdone
                frzctrl_reg_cfg_mask 
                    = ALT_SYSMGR_FRZCTL_VIOCTL_BUSHOLD_SET_MSK 
                       | ALT_SYSMGR_FRZCTL_VIOCTL_CFG_SET_MSK;
                
                alt_setbits_word(ALT_SYSMGR_OFST + frzctrl_ioctrl_reg_offset,
                                    frzctrl_reg_cfg_mask);
                 
				// Note: Delay for 20ns at min
                // de-assert active low plniotri and niotri signals
                frzctrl_reg_cfg_mask  = ALT_SYSMGR_FRZCTL_VIOCTL_WKPULLUP_SET_MSK
                        | ALT_SYSMGR_FRZCTL_VIOCTL_TRISTATE_SET_MSK;
                alt_setbits_word( ALT_SYSMGR_OFST + frzctrl_ioctrl_reg_offset, frzctrl_reg_cfg_mask);

                // Note: Delay for 20ns at min
                // de-assert active low enrnsl signal
                alt_setbits_word( ALT_SYSMGR_OFST + frzctrl_ioctrl_reg_offset, ALT_SYSMGR_FRZCTL_VIOCTL_SLEW_SET_MSK);

                // Insert short delay to allow freeze signal propagation,
                // delay (SYSMGR_FRZCTRL_DELAY_LOOP_PARAM);
                for(i=SYSMGR_FRZCTRL_DELAY_LOOP_PARAM; i>0; i--); 
                
                // Set global flag to indicate channel is thawed
                frzctrl_channel_freeze[channel_id] = FREEZE_CTRL_THAWED;
                
                break;

            case FREEZE_CHANNEL_3:
         	
                 //de-assert active high reinit signal
                alt_clrbits_word(ALT_SYSMGR_FRZCTL_HIOCTL_ADDR,
                		ALT_SYSMGR_FRZCTL_HIOCTL_DLLRST_SET_MSK);                                  

				// Note: Delay for 40ns at min
				// assert active high pllbiasen signals
                alt_setbits_word (ALT_SYSMGR_FRZCTL_HIOCTL_ADDR,
                		ALT_SYSMGR_FRZCTL_HIOCTL_OCT_CFGEN_CALSTART_SET_MSK);

                // Delay 1000 intosc. intosc is based on eosc1
                //start_l4_timer (ALT_OSC1TMR0_OFST, 0xFFFFFFFF, 0x03);
                alt_write_word(ALT_OSC1TMR0_TMR1CTLREG_ADDR, 0);	// disable
                alt_write_word(ALT_OSC1TMR0_TMR1LDCOUNT_ADDR, 0xFFFFFFFF);  // Load value
                alt_write_word(ALT_OSC1TMR0_TMR1CTLREG_ADDR, 0x3);	// enable, re-start mode      		
                start_count = alt_read_word(ALT_OSC1TMR0_TMR1CURVAL_ADDR);  // read current count
                            		
        		do {
            		clk_cycles_count = start_count - alt_read_word(ALT_OSC1TMR0_TMR1CURVAL_ADDR);
               	} while (clk_cycles_count < SYSMGR_FRZCTRL_INTOSC_1000);

                
                // de-assert active low bhniotri signal, assert active high
                // csrdone and nfrzdrv signals
                frzctrl_reg_value = alt_read_word(ALT_SYSMGR_FRZCTL_HIOCTL_ADDR);
                frzctrl_reg_value 
                    = (frzctrl_reg_value 
                        | ALT_SYSMGR_FRZCTL_HIOCTL_BUSHOLD_SET_MSK
                        | ALT_SYSMGR_FRZCTL_HIOCTL_CFG_SET_MSK)
                        & ~ALT_SYSMGR_FRZCTL_HIOCTL_OCTRST_SET_MSK;
                alt_write_word(ALT_SYSMGR_FRZCTL_HIOCTL_ADDR, frzctrl_reg_value);

                // Delay 33 intosc
                //reset_timer_count (ALT_OSC1TMR0_OFST);
                //start_count = get_timer_count (0, ALT_OSC1TMR0_OFST);
                alt_write_word(ALT_OSC1TMR0_TMR1LDCOUNT_ADDR, 0xFFFFFFFF);  // load value -> reset_count
                start_count = alt_read_word(ALT_OSC1TMR0_TMR1CURVAL_ADDR);  // read current count
        		do {
            		clk_cycles_count = start_count - alt_read_word(ALT_OSC1TMR0_TMR1CURVAL_ADDR);
               	} while (clk_cycles_count < SYSMGR_FRZCTRL_INTOSC_33);


	            // Stop L4 timer
	            //disable_l4_timer(ALT_OSC1TMR0_OFST, 0); 
        		alt_write_word(ALT_OSC1TMR0_TMR1CTLREG_ADDR, 0);	// disable
        		
                // de-assert active low plniotri and niotri signals
                frzctrl_reg_cfg_mask 
                    = ALT_SYSMGR_FRZCTL_HIOCTL_WKPULLUP_SET_MSK
                        | ALT_SYSMGR_FRZCTL_HIOCTL_TRISTATE_SET_MSK;

                alt_setbits_word (ALT_SYSMGR_FRZCTL_HIOCTL_ADDR, frzctrl_reg_cfg_mask);
    
                // Note: Delay for 40ns at min
                // de-assert active high frzreg signal
                alt_clrbits_word (ALT_SYSMGR_FRZCTL_HIOCTL_ADDR,
                		ALT_SYSMGR_FRZCTL_HIOCTL_REGRST_SET_MSK);                                

                // Note: Delay for 40ns at min
                // de-assert active low enrnsl signal
                alt_setbits_word (ALT_SYSMGR_FRZCTL_HIOCTL_ADDR,
                		ALT_SYSMGR_FRZCTL_HIOCTL_SLEW_SET_MSK);      
                    
                // Insert short delay to allow freeze signal propagation,
                //delay (SYSMGR_FRZCTRL_DELAY_LOOP_PARAM);
                for(i=SYSMGR_FRZCTRL_DELAY_LOOP_PARAM; i>0; i--);
                
                // Set global flag to indicate channel is thawed
                frzctrl_channel_freeze[channel_id] = FREEZE_CTRL_THAWED;                                

                break;

            default:
                //puts ("sys_mgr_frzctrl_thaw_req: invalid channel ID\n");
                return 1;
        }
    } else if (FREEZE_CONTROLLER_FSM_HW == fsm_select){

#ifdef FRZCTRL_DEBUG
        puts ("ThawReq: HW FSM selected\n");
#endif //FRZCTRL_DEBUG  
       
        // select hardware FSM
        alt_write_word(ALT_SYSMGR_FRZCTL_SRC_ADDR, 
        		ALT_SYSMGR_FRZCTL_SRC_VIO1_E_HW);

        // write to hwctrl reg to enable thaw req; 0: thaw
        alt_clrbits_word (ALT_SYSMGR_FRZCTL_HWCTL_ADDR,
        		ALT_SYSMGR_FRZCTL_HWCTL_VIO1REQ_SET_MSK);
                                    
		i = 0;
		while (ALT_SYSMGR_FRZCTL_HWCTL_VIO1STATE_E_THAWED 
		    != sys_mgr_frzctrl_frzchn1_state_get()){
			
			i++;
			
			if (SYSMGR_FRZCTRL_LOOP_PARAM < i) {
			    //puts ("sys_mgr_frzctrl_thaw_req: unable to thaw\n");
				return 1;
			}
		} // while (not thaw)  
			
		// Set global flag to indicate channel is thawed
        frzctrl_channel_freeze[channel_id] = FREEZE_CTRL_THAWED;                                  
    } else {
        
        //puts ("sys_mgr_frzctrl_thaw_req: invalid request\n");
        return 1;
    }// if-else (fsm_select)

    return 0;
}

/**
 * Get current state of freeze channel 
 * @param channel_id Freeze channel ID with range of enum
 *            FreezeChannelEnum
 * @return bool Current freeze channel status, TRUE if frozen, 0 otherwise
 */
uint32_t  sys_mgr_frzctrl_frzchn_is_frozen (FreezeChannelSelect channel_id) 
{
    if (FREEZE_CTRL_FROZEN == frzctrl_channel_freeze[channel_id])
	return 1;
    else
	return 0;
}
