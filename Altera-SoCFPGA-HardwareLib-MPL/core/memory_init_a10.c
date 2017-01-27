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
 * $Id: 
 */

// include socal headers
#include <socal/socal.h>
#include <socal/hps.h>
#include <socal/alt_fpgamgr.h>
#include <socal/alt_rstmgr.h>
#include <socal/alt_ecc_hmc_ocp.h>
#include <socal/alt_sysmgr.h>
#include <hwlib.h>
#include <alt_timers.h>
#include "mpl_config.h"

#define BIT(x) (1U<<x)
#define ALT_FPGAMGR_GPO_31_HPS2EMIF_RST_OVR_ACT_LOW      BIT(31)
#define ALT_FPGAMGR_GPO_30_HPS2OCT_OCT_CAL_REQ_ACT_HIGH  BIT(30)
#define ALT_FPGAMGR_GPI_31_OCT2HPS_OCT_CAL_RDY_ACT_HIGH  BIT(31)
#define ALT_RSTMGR_HMCGPI_7_SEQ2CORE_OCT_REQ_ACT_HIGH    BIT(7)
#define ALT_RSTMGR_HMCGPO_7_CORE2SEQ_OCT_RDY_ACT_HIGH    BIT(7)
#define MAX_MEM_CAL_RETRY                                3
#define OCT_FSM_DELAY                                    1000
#define RESET_DELAY_OCT                                  5000
#define RESET_DELAY_EMIF                                 10000
#define TIMEOUT_OCT                                      10000
#define TIMEOUT_EMIF_HAND_SHAKE                          10000
#define TIMEOUT_EMIF_CALIBRATION                         20000000
#define ALT_SYSMGR_SILICONID1_BRING_UP                   0x00010001

ALT_STATUS_CODE altr_sdram_calibration_workaround(void)
{
  uint32_t time;
  uint32_t TimeOutCount, Data32;

  // 1. Assert EMIF Reset signal
  alt_clrbits_word(ALT_FPGAMGR_GPO_ADDR,
        ALT_FPGAMGR_GPO_31_HPS2EMIF_RST_OVR_ACT_LOW);
  time = alt_gpt_counter_get(ALT_GPT_OSC1_TMR0);
  alt_gpt_delay_ns(ALT_GPT_OSC1_TMR0, time, RESET_DELAY_EMIF);

  // 2. Clear HPS to EMIF NIOS II communication register
  alt_write_word(ALT_RSTMGR_HMCGPOUT_ADDR,
        ALT_RSTMGR_HMCGPOUT_RESET);

  // 3. De-assert HPS to OCT FSM calibration request signal
  alt_clrbits_word(ALT_FPGAMGR_GPO_ADDR,
        ALT_FPGAMGR_GPO_30_HPS2OCT_OCT_CAL_REQ_ACT_HIGH);
  time = alt_gpt_counter_get(ALT_GPT_OSC1_TMR0);
  alt_gpt_delay_ns(ALT_GPT_OSC1_TMR0, time, RESET_DELAY_OCT);

  // 4. De-assert EMIF Reset signal
  alt_setbits_word(ALT_FPGAMGR_GPO_ADDR,
        ALT_FPGAMGR_GPO_31_HPS2EMIF_RST_OVR_ACT_LOW);
  time = alt_gpt_counter_get(ALT_GPT_OSC1_TMR0);
  alt_gpt_delay_ns(ALT_GPT_OSC1_TMR0, time, RESET_DELAY_EMIF);

  // 5. Wait for OCT FSM to send HPS an OCT Ready = 1
  TimeOutCount = 0;
  while(1)
  {
    Data32 = alt_read_word(ALT_FPGAMGR_GPI_ADDR);
    if((Data32 & ALT_FPGAMGR_GPI_31_OCT2HPS_OCT_CAL_RDY_ACT_HIGH) != 0)
      break;
    time = alt_gpt_counter_get(ALT_GPT_OSC1_TMR0);
    alt_gpt_delay_ns(ALT_GPT_OSC1_TMR0, time, OCT_FSM_DELAY);
    if(++TimeOutCount > TIMEOUT_OCT)
    {
      ALT_PRINTF("\t\t Timeout during HPS/EMIF/OCT Handshake\n");
      return ALT_E_ERROR;
    }
  }
  
  // 6. Inform EMIF NIOS II to get ready for OCT calibration
  alt_setbits_word(ALT_RSTMGR_HMCGPOUT_ADDR,
                ALT_RSTMGR_HMCGPO_7_CORE2SEQ_OCT_RDY_ACT_HIGH);

  // 7. Wait for EMIF NIOS II to send HPS an OCT request = 1
  TimeOutCount = 0;
  while(1)
  {
    Data32 = alt_read_word(ALT_RSTMGR_HMCGPIN_ADDR);
    if((Data32 & ALT_RSTMGR_HMCGPI_7_SEQ2CORE_OCT_REQ_ACT_HIGH) != 0)
      break;
    time = alt_gpt_counter_get(ALT_GPT_OSC1_TMR0);
    alt_gpt_delay_ns(ALT_GPT_OSC1_TMR0, time, OCT_FSM_DELAY);
    if(++TimeOutCount > TIMEOUT_EMIF_HAND_SHAKE)
    {
      ALT_PRINTF("\t\t Timeout during HPS/EMIF/OCT Handshake\n");
      return ALT_E_ERROR;
    }
  }

  // 8. Acknowledge to EMIF NIOS II on receive of the OCT request signal
  alt_clrbits_word(ALT_RSTMGR_HMCGPOUT_ADDR,
      ALT_RSTMGR_HMCGPO_7_CORE2SEQ_OCT_RDY_ACT_HIGH);

  // 9. Wait for EMIF NIOS II to response with OCT request = 0
  TimeOutCount = 0;
  while(1)
  {
    Data32 = alt_read_word(ALT_RSTMGR_HMCGPIN_ADDR);
    if((Data32 & ALT_RSTMGR_HMCGPI_7_SEQ2CORE_OCT_REQ_ACT_HIGH) == 0)
      break;
    time = alt_gpt_counter_get(ALT_GPT_OSC1_TMR0);
    alt_gpt_delay_ns(ALT_GPT_OSC1_TMR0, time, OCT_FSM_DELAY);
    if(++TimeOutCount > TIMEOUT_EMIF_HAND_SHAKE)
    {
      ALT_PRINTF("\t\t Timeout during HPS/EMIF/OCT Handshake\n");
      return ALT_E_ERROR;
    }
  }

  // 10. Assert HPS to OCT FSM calibration request signal
  alt_setbits_word(ALT_FPGAMGR_GPO_ADDR,
       ALT_FPGAMGR_GPO_30_HPS2OCT_OCT_CAL_REQ_ACT_HIGH);
  time = alt_gpt_counter_get(ALT_GPT_OSC1_TMR0);
  alt_gpt_delay_ns(ALT_GPT_OSC1_TMR0, time, RESET_DELAY_OCT);

  // 11. Wait for OCT FSM's OCT Ready = 0
  TimeOutCount = 0;
  while(1)
  {
    Data32 = alt_read_word(ALT_FPGAMGR_GPI_ADDR);
    if((Data32 & ALT_FPGAMGR_GPI_31_OCT2HPS_OCT_CAL_RDY_ACT_HIGH) == 0)
      break;
    time = alt_gpt_counter_get(ALT_GPT_OSC1_TMR0);
    alt_gpt_delay_ns(ALT_GPT_OSC1_TMR0, time, OCT_FSM_DELAY);
    if(++TimeOutCount > TIMEOUT_OCT)
    {
      ALT_PRINTF("\t\t Timeout during HPS/EMIF/OCT Handshake\n");
      return ALT_E_ERROR;
    }
  }

  // 12. De-assert HPS to OCT FSM calibration request signal
  alt_clrbits_word(ALT_FPGAMGR_GPO_ADDR,
        ALT_FPGAMGR_GPO_30_HPS2OCT_OCT_CAL_REQ_ACT_HIGH);
  time = alt_gpt_counter_get(ALT_GPT_OSC1_TMR0);
  alt_gpt_delay_ns(ALT_GPT_OSC1_TMR0, time, RESET_DELAY_OCT);

  // 13. Wait for OCT FSM to send HPS an OCT Ready = 1
  TimeOutCount = 0;
  while(1)
  {
    Data32 = alt_read_word(ALT_FPGAMGR_GPI_ADDR);
    if((Data32 & ALT_FPGAMGR_GPI_31_OCT2HPS_OCT_CAL_RDY_ACT_HIGH) != 0)
      break;
    time = alt_gpt_counter_get(ALT_GPT_OSC1_TMR0);
    alt_gpt_delay_ns(ALT_GPT_OSC1_TMR0, time, OCT_FSM_DELAY);
    if(++TimeOutCount > TIMEOUT_OCT)
    {
      ALT_PRINTF("\t\t Timeout during HPS/EMIF/OCT Handshake\n");
      return ALT_E_ERROR;
    }
  }

  // 14. Allow EMIF NIOS II to start OCT calibration
  alt_setbits_word(ALT_RSTMGR_HMCGPOUT_ADDR,
        ALT_RSTMGR_HMCGPO_7_CORE2SEQ_OCT_RDY_ACT_HIGH);
  time = alt_gpt_counter_get(ALT_GPT_OSC1_TMR0);
  alt_gpt_delay_ns(ALT_GPT_OSC1_TMR0, time, RESET_DELAY_EMIF);
  return ALT_E_SUCCESS;
}

ALT_STATUS_CODE altr_sdram_calibration_common(void)
{
  uint32_t time;
  uint32_t TimeOutCount, Data32;
  TimeOutCount = 0;
  while(1)
  {
    Data32 = alt_read_word(ALT_ECC_HMC_OCP_DDRCALSTAT_ADDR);
    if (ALT_ECC_HMC_OCP_DDRCALSTAT_CAL_GET(Data32) == 1)
      break;
    time = alt_gpt_counter_get(ALT_GPT_OSC1_TMR0);
    alt_gpt_delay_ns(ALT_GPT_OSC1_TMR0, time, OCT_FSM_DELAY);
    if(++TimeOutCount > TIMEOUT_EMIF_CALIBRATION)
    {
      ALT_PRINTF("t\t Timeout during DDR calibration\r\n");
      return ALT_E_ERROR;
    }
  }
  return ALT_E_SUCCESS;
}
ALT_STATUS_CODE altr_sdram_calibration_full(void)
{
  uint32_t Data32;
  uint32_t RetryCount;

  /* Unmap ROM from address 0 */
  alt_write_word(0xfffffc00, 1);
  /* Take the HMC out of reset */
  alt_clrbits_word(ALT_RSTMGR_BRGMODRST_ADDR, ALT_RSTMGR_BRGMODRST_DDRSCH_SET_MSK);

  Data32 = alt_read_word(ALT_SYSMGR_SILICONID1_ADDR);
  if (Data32 == ALT_SYSMGR_SILICONID1_BRING_UP)
  {
    while(ALT_E_SUCCESS != altr_sdram_calibration_workaround() ||
          ALT_E_SUCCESS != altr_sdram_calibration_common())
    {
      if(RetryCount > MAX_MEM_CAL_RETRY)
      {
        ALT_PRINTF("\t\t DRAM calibration fail\n");
        return ALT_E_ERROR;
      }
      ALT_PRINTF("\t\t Retrying DRAM calibration\r\n");
    }
  }
  ALT_PRINTF("\t\t DRAM calibration successful\n");
  return ALT_E_SUCCESS;
}







