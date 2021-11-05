/* -----------------------------------------------------------------------------
 * Copyright (c) 2013-2020 Arm Limited (or its affiliates). All 
 * rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 *
 * $Date:        28. September 2020
 * $Revision:    V1.3
 *
 * Driver:       Driver_MCI0
 * Configured:   via STM32CubeMX
 * Project:      MCI Driver for ST STM32L4xx
 * --------------------------------------------------------------------------
 * Use the following configuration settings in the middleware component
 * to connect to this driver.
 *
 *   Configuration Setting                 Value
 *   ---------------------                 -----
 *   Connect to hardware via Driver_MCI# = 0
 * -------------------------------------------------------------------------- */


/*! \page stm32l4_mci CMSIS-Driver MCI Setup 

The CMSIS-Driver MCI requires:
  - Setup of SDIO with DMA for Rx DMA transfers.

Valid settings for various evaluation boards are listed in the table below:

Peripheral Resource     | STM32L496G-DISCO   | STM32L476G-EVAL   |
:-----------------------|:-------------------|:------------------|
SDMMC1 Mode             | <b>SD 4 bits</b>   | n/a               |

For different boards, refer to the hardware schematics to reflect correct setup values.

The STM32CubeMX configuration for STM32L496G-DISCO with steps for Pinout, Clock, and System Configuration are 
listed below. Enter the values that are marked \b bold.
   
Pinout tab
----------
  1. Configure SDIO mode
     - Peripherals \b SDMMC1: Mode=<b>SD 4 bits Wide bus</b>
          
Clock Configuration tab
-----------------------
  1. Configure SDIO Clock: "48MHz clocks (MHz)": 48
  
Configuration tab
-----------------
  1. Under Connectivity open \b SDMMC1 Configuration:
     - <b>DMA Settings</b>: setup DMA transfers for Rx\n
       \b Add - Select \b SDMMC1_RX: Stream=DMA2 Channel 5, Direction=Peripheral to Memory, Priority=Low
       \b Add - Select \b SDMMC1_TX: Stream=DMA2 Channel 4, Direction=Memory to Peripheral, Priority=Low

     - <b>GPIO Settings</b>: review settings, no changes required
          Pin Name | Signal on Pin | GPIO mode | GPIO Pull-up/Pull..| Maximum out | User Label
          :--------|:--------------|:----------|:-------------------|:------------|:----------
          PC8      | SDMMC1_D0     | Alternate | No pull-up and no..| Very High   |.
          PC9      | SDMMC1_D1     | Alternate | No pull-up and no..| Very High   |.
          PC10     | SDMMC1_D2     | Alternate | No pull-up and no..| Very High   |.
          PC11     | SDMMC1_D3     | Alternate | No pull-up and no..| Very High   |.
          PC12     | SDMMC1_CK     | Alternate | No pull-up and no..| Very High   |.
          PD2      | SDMMC1_CMD    | Alternate | No pull-up and no..| Very High   |.

     - <b>NVIC Settings</b>: enable interrupts
          Interrupt Table                      | Enable | Preemption Priority | Sub Priority
          :------------------------------------|:-------|:--------------------|:--------------
          SDMMC1 global interrupt              |\b ON   | 0                   | 0
          DMA2 channel5 global interrupt       |   ON   | 0                   | 0
          DMA2 channel4 global interrupt       |   ON   | 0                   | 0

     - Parameter Settings: not used
     - User Constants: not used
     - Click \b OK to close the SDIO Configuration dialog

  2. Under System open \b NVIC Configuration:
     - Disable generation of SDMMC1 interrupt handler	 
     - Click \b OK to close the NVIC Configuration dialog

  3. Open <b>Project - Settings - Advanced Settings</b> from the menu and enable "Not Generate Function call" for 
     MX_SDMMC1_SD_Init
*/

/* History:
 *  Version 1.3
 *    Replaced empty delay loops with _NOP()
 *  Version 1.2
 *    Added handling for separate SD and MMC HAL layers
 *  Version 1.1
 *    Added DMA support for Transmit
 *  Version 1.0
 *    Initial release
 */

#include "MCI_STM32L4xx.h"
#include "stm32L4xx_ll_dma.h"

#define ARM_MCI_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1,3)  /* driver version */

/* Enable High Speed bus mode */
#if defined(MemoryCard_Bus_Mode_HS_Enable)
  #define MCI_BUS_MODE_HS     1U
#else
  #define MCI_BUS_MODE_HS     0U
#endif

/* Define Card Detect pin active state */
#if !defined(MemoryCard_CD_Pin_Active)
  #define MemoryCard_CD_Pin_Active GPIO_PIN_RESET
#endif

/* Define Write Protect pin active state */
#if !defined(MemoryCard_WP_Pin_Active)
  #define MemoryCard_WP_Pin_Active GPIO_PIN_SET
#endif

/* Define MemoryCard_MMC0 if SDMMC1 is configured for MMC device */
#if !defined(MemoryCard_MMC0)
  #define MCI0_HANDLE_TYPE    0U
#else
  #define MCI0_HANDLE_TYPE    1U
#endif

#if (MCI0_HANDLE_TYPE == 0)
#define MCI0_HANDLE      hsd1
extern SD_HandleTypeDef  hsd1;
#else
#define MCI0_HANDLE      hmmc1
extern MMC_HandleTypeDef hmmc1;
#endif
extern DMA_HandleTypeDef hdma_sdmmc1_rx;
extern DMA_HandleTypeDef hdma_sdmmc1_tx;

static MCI_INFO MCI;

/* DMA callback function */
static void RX_DMA_Complete(struct __DMA_HandleTypeDef *hdma);

/* IRQ Handler prototype */
void SDMMC1_IRQHandler (void);


/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion = {
  ARM_MCI_API_VERSION,
  ARM_MCI_DRV_VERSION
};

/* Driver Capabilities */
static const ARM_MCI_CAPABILITIES DriverCapabilities = {
  MCI_CD_PIN,                                     /* cd_state          */
  0U,                                             /* cd_event          */
  MCI_WP_PIN,                                     /* wp_state          */
  0U,                                             /* vdd               */
  0U,                                             /* vdd_1v8           */
  0U,                                             /* vccq              */
  0U,                                             /* vccq_1v8          */
  0U,                                             /* vccq_1v2          */
  MCI_BUS_WIDTH_4,                                /* data_width_4      */
  MCI_BUS_WIDTH_8,                                /* data_width_8      */
  0U,                                             /* data_width_4_ddr  */
  0U,                                             /* data_width_8_ddr  */
  MCI_BUS_MODE_HS,                                /* high_speed        */
  0U,                                             /* uhs_signaling     */
  0U,                                             /* uhs_tuning        */
  0U,                                             /* uhs_sdr50         */
  0U,                                             /* uhs_sdr104        */
  0U,                                             /* uhs_ddr50         */
  0U,                                             /* uhs_driver_type_a */
  0U,                                             /* uhs_driver_type_c */
  0U,                                             /* uhs_driver_type_d */
  1U,                                             /* sdio_interrupt    */
  1U,                                             /* read_wait         */
  0U,                                             /* suspend_resume    */
  0U,                                             /* mmc_interrupt     */
  0U,                                             /* mmc_boot          */
  0U,                                             /* rst_n             */
  0U,                                             /* ccs               */
  0U                                              /* ccs_timeout       */
#if (defined(ARM_MCI_API_VERSION) && (ARM_MCI_API_VERSION >= 0x203U))
, 0U                                              /* reserved bits     */
#endif
};

/**
  \fn          void Assign_SDMMC_Instance (uint32_t set)
  \brief       Set/Reset instance variable inside HAL handle structure
*/
static void Assign_SDMMC_Instance (uint32_t set) {
#if defined(MX_SDMMC1)
  #if (MCI0_HANDLE_TYPE == 0)
    /* Instance is the coresponding peripheral register inteface */
    if (set == 0) {
      hsd1.Instance = NULL;
    } else {
      hsd1.Instance = SDMMC1;
    }
  #else
    /* Instance is the coresponding peripheral register inteface */
    if (set == 0) {
      hmmc1.Instance = NULL;
    } else {
      hmmc1.Instance = SDMMC1;
    }
  #endif
#endif
}

/**
  \fn          void Config_SDMMC_Msp (uint32_t init)
  \brief       Init/DeInit MSP layer
*/
static void Config_SDMMC_Msp (uint32_t init) {
#if defined(MX_SDMMC1)
  #if (MCI0_HANDLE_TYPE == 0)
    if (hsd1.Instance != NULL) {
      if (init == 0) {
        HAL_SD_MspDeInit (&hsd1);
      } else {
        HAL_SD_MspInit (&hsd1);
      }
    }
  #else
    if (hmmc1.Instance != NULL) {
      if (init == 0) {
        HAL_MMC_MspDeInit (&hmmc1);
      } else {
        HAL_MMC_MspInit (&hmmc1);
      }
    }
  #endif
#endif
}

/**
  \fn          ARM_DRV_VERSION GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRV_VERSION
*/
static ARM_DRIVER_VERSION GetVersion (void) {
  return DriverVersion;
}


/**
  \fn          ARM_MCI_CAPABILITIES MCI_GetCapabilities (void)
  \brief       Get driver capabilities.
  \return      \ref ARM_MCI_CAPABILITIES
*/
static ARM_MCI_CAPABILITIES GetCapabilities (void) {
  return DriverCapabilities;
}


/**
  \fn            int32_t Initialize (ARM_MCI_SignalEvent_t cb_event)
  \brief         Initialize the Memory Card Interface
  \param[in]     cb_event  Pointer to \ref ARM_MCI_SignalEvent
  \return        \ref execution_status
*/
static int32_t Initialize (ARM_MCI_SignalEvent_t cb_event) {

  if (MCI.flags & MCI_INIT) { return ARM_DRIVER_OK; }

  /* Set SDMMC instance pointer */
  Assign_SDMMC_Instance (1U);

  /* Set DMA callback function */
  hdma_sdmmc1_rx.XferCpltCallback  = &RX_DMA_Complete;

  /* Clear control structure */
  memset (&MCI, 0, sizeof (MCI_INFO));

  MCI.cb_event = cb_event;
  MCI.flags    = MCI_INIT;

  return ARM_DRIVER_OK;
}


/**
  \fn            int32_t Uninitialize (void)
  \brief         De-initialize Memory Card Interface.
  \return        \ref execution_status
*/
static int32_t Uninitialize (void) {
  MCI.flags = 0U;

  /* Reset SDMMC instance pointer */
  Assign_SDMMC_Instance (0U);

  #if defined (MX_MemoryCard_CD_Pin)
    /* Unconfigure CD (Card Detect) Pin */
    HAL_GPIO_DeInit (MX_MemoryCard_CD_GPIOx, MX_MemoryCard_CD_GPIO_Pin);
  #endif

  #if defined (MX_MemoryCard_WP_Pin)
    /* Unconfigure WP (Write Protect) Pin */
    HAL_GPIO_DeInit (MX_MemoryCard_WP_GPIOx, MX_MemoryCard_WP_GPIO_Pin);
  #endif

  return ARM_DRIVER_OK;
}


/**
  \fn            int32_t PowerControl (ARM_POWER_STATE state)
  \brief         Control Memory Card Interface Power.
  \param[in]     state   Power state \ref ARM_POWER_STATE
  \return        \ref execution_status
*/
static int32_t PowerControl (ARM_POWER_STATE state) {
  int32_t status;

  if ((state != ARM_POWER_OFF)  && 
      (state != ARM_POWER_FULL) && 
      (state != ARM_POWER_LOW)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  status = ARM_DRIVER_OK;

  switch (state) {
    case ARM_POWER_OFF:
      /* Reset/Dereset SDIO peripheral */
      __HAL_RCC_SDIO_FORCE_RESET();
      __NOP(); __NOP(); __NOP(); __NOP();
      __HAL_RCC_SDIO_RELEASE_RESET();

      Config_SDMMC_Msp(0U);

      /* Clear status */
      MCI.status.command_active   = 0U;
      MCI.status.command_timeout  = 0U;
      MCI.status.command_error    = 0U;
      MCI.status.transfer_active  = 0U;
      MCI.status.transfer_timeout = 0U;
      MCI.status.transfer_error   = 0U;
      MCI.status.sdio_interrupt   = 0U;
      MCI.status.ccs              = 0U;

      MCI.flags &= ~MCI_POWER;
      break;

    case ARM_POWER_FULL:
      if ((MCI.flags & MCI_INIT)  == 0U) {
        return ARM_DRIVER_ERROR;
      }
      if ((MCI.flags & MCI_POWER) != 0U) {
        return ARM_DRIVER_OK;
      }

      Config_SDMMC_Msp(1U);

      /* Set DMA callback function */
      hdma_sdmmc1_rx.XferCpltCallback  = &RX_DMA_Complete;

      /* Clear response and transfer variables */
      MCI.response = NULL;
      MCI.xfer.cnt = 0U;

      /* Enable SDMMC peripheral interrupts */
      SDMMC1->MASK = SDMMC_MASK_DATAENDIE  |
                     SDMMC_MASK_CMDSENTIE  |
                     SDMMC_MASK_CMDRENDIE  |
                     SDMMC_MASK_DTIMEOUTIE |
                     SDMMC_MASK_CTIMEOUTIE |
                     SDMMC_MASK_DCRCFAILIE |
                     SDMMC_MASK_CCRCFAILIE ;

      /* Set max data timeout */
      SDMMC1->DTIMER = 0xFFFFFFFF;

      /* Enable clock to the card (SDIO_CK) */
      SDMMC1->POWER = SDMMC_POWER_PWRCTRL_1 | SDMMC_POWER_PWRCTRL_0;

      MCI.flags |= MCI_POWER;
      break;

    case ARM_POWER_LOW:
      return ARM_DRIVER_ERROR_UNSUPPORTED;
  }
  return status;
}


/**
  \fn            int32_t CardPower (uint32_t voltage)
  \brief         Set Memory Card supply voltage.
  \param[in]     voltage  Memory Card supply voltage
  \return        \ref execution_status
*/
static int32_t CardPower (uint32_t voltage) {
  (void)voltage;

  if ((MCI.flags & MCI_POWER) == 0U) { return ARM_DRIVER_ERROR; }
  return ARM_DRIVER_ERROR_UNSUPPORTED;
}


/**
  \fn            int32_t ReadCD (void)
  \brief         Read Card Detect (CD) state.
  \return        1:card detected, 0:card not detected, or error
*/
static int32_t ReadCD (void) {

  if ((MCI.flags & MCI_POWER) == 0U) { return ARM_DRIVER_ERROR; }

  /* Read CD (Card Detect) Pin */
  #if defined (MX_MemoryCard_CD_Pin)
  if (HAL_GPIO_ReadPin (MX_MemoryCard_CD_GPIOx, MX_MemoryCard_CD_GPIO_Pin) == MemoryCard_CD_Pin_Active) {
    /* Card Detect switch is active */
    return (1);
  }
  #endif
  return (0);
}


/**
  \fn            int32_t ReadWP (void)
  \brief         Read Write Protect (WP) state.
  \return        1:write protected, 0:not write protected, or error
*/
static int32_t ReadWP (void) {

  if ((MCI.flags & MCI_POWER) == 0U) { return ARM_DRIVER_ERROR; }

  /* Read WP (Write Protect) Pin */
  #if defined (MX_MemoryCard_WP_Pin)
  if (HAL_GPIO_ReadPin (MX_MemoryCard_WP_GPIOx, MX_MemoryCard_WP_GPIO_Pin) == MemoryCard_WP_Pin_Active) {
    /* Write protect switch is active */
    return (1);
  }
  #endif
  return (0);
}


/**
  \fn            int32_t SendCommand (uint32_t  cmd,
                                      uint32_t  arg,
                                      uint32_t  flags,
                                      uint32_t *response)
  \brief         Send Command to card and get the response.
  \param[in]     cmd       Memory Card command
  \param[in]     arg       Command argument
  \param[in]     flags     Command flags
  \param[out]    response  Pointer to buffer for response
  \return        \ref execution_status
*/
static int32_t SendCommand (uint32_t cmd, uint32_t arg, uint32_t flags, uint32_t *response) {
  uint32_t i, clkcr;

  if (((flags & MCI_RESPONSE_EXPECTED_Msk) != 0U) && (response == NULL)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  if ((MCI.flags & MCI_SETUP) == 0U) {
    return ARM_DRIVER_ERROR;
  }
  if (MCI.status.command_active) {
    return ARM_DRIVER_ERROR_BUSY;
  }
  MCI.status.command_active   = 1U;
  MCI.status.command_timeout  = 0U;
  MCI.status.command_error    = 0U;
  MCI.status.transfer_timeout = 0U;
  MCI.status.transfer_error   = 0U;
  MCI.status.ccs              = 0U;

  if (flags & ARM_MCI_CARD_INITIALIZE) {
    clkcr = SDMMC1->CLKCR;

    if (((clkcr & SDMMC_CLKCR_CLKEN) == 0) || ((clkcr & SDMMC_CLKCR_PWRSAV) != 0)) {
      SDMMC1->CLKCR = (SDMMC1->CLKCR & ~SDMMC_CLKCR_PWRSAV) | SDMMC_CLKCR_CLKEN;

      i = HAL_RCC_GetHCLKFreq();
      for (i = (i/5000000U)*1000U; i; i--) {
        __NOP(); /* Wait for approximate 1000us */
      }
      SDMMC1->CLKCR = clkcr;
    }
  }

  /* Set command register value */
  cmd = SDMMC_CMD_CPSMEN | (cmd & 0xFFU);

  MCI.response = response;
  MCI.flags   &= ~(MCI_RESP_CRC | MCI_RESP_LONG);

  switch (flags & ARM_MCI_RESPONSE_Msk) {
    case ARM_MCI_RESPONSE_NONE:
      /* No response expected (wait CMDSENT) */
      break;

    case ARM_MCI_RESPONSE_SHORT:
    case ARM_MCI_RESPONSE_SHORT_BUSY:
      /* Short response expected (wait CMDREND or CCRCFAIL) */
      cmd |= SDMMC_CMD_WAITRESP_0;
      break;

    case ARM_MCI_RESPONSE_LONG:
      MCI.flags |= MCI_RESP_LONG;
      /* Long response expected (wait CMDREND or CCRCFAIL) */
      cmd |= SDMMC_CMD_WAITRESP_1 | SDMMC_CMD_WAITRESP_0;
      break;

    default:
      return ARM_DRIVER_ERROR;
  }
  if (flags & ARM_MCI_RESPONSE_CRC) {
    MCI.flags |= MCI_RESP_CRC;
  }
  if (flags & ARM_MCI_TRANSFER_DATA) {
    MCI.flags |= MCI_DATA_XFER;
  }

  /* Clear all interrupt flags */
  SDMMC1->ICR = SDMMC_ICR_BIT_Msk;

  /* Send the command */
  SDMMC1->ARG = arg;
  SDMMC1->CMD = cmd;

  return ARM_DRIVER_OK;
}


/**
  \fn            int32_t SetupTransfer (uint8_t *data,
                                        uint32_t block_count,
                                        uint32_t block_size,
                                        uint32_t mode)
  \brief         Setup read or write transfer operation.
  \param[in,out] data         Pointer to data block(s) to be written or read
  \param[in]     block_count  Number of blocks
  \param[in]     block_size   Size of a block in bytes
  \param[in]     mode         Transfer mode
  \return        \ref execution_status
*/
static int32_t SetupTransfer (uint8_t *data, uint32_t block_count, uint32_t block_size, uint32_t mode) {
  uint32_t sz, cnt, dctrl;

  if ((data == NULL) || (block_count == 0U) || (block_size == 0U)) { return ARM_DRIVER_ERROR_PARAMETER; }

  if ((MCI.flags & MCI_SETUP) == 0U) {
    return ARM_DRIVER_ERROR;
  }
  if (MCI.status.transfer_active) {
    return ARM_DRIVER_ERROR_BUSY;
  }

  MCI.xfer.buf = data;
  MCI.xfer.cnt = block_count * block_size;

  cnt = MCI.xfer.cnt;
  if (cnt > 0xFFFFU) {
    cnt = 0xFFFFU;
  }

  MCI.xfer.cnt -= cnt;
  MCI.xfer.buf += cnt;

  dctrl = 0U;

  if ((mode & ARM_MCI_TRANSFER_WRITE) == 0) {
    /* Direction: From card to controller */
    MCI.flags |= MCI_DATA_READ;
    dctrl |= SDMMC_DCTRL_DTDIR;
  }
  else {
    MCI.flags &= ~MCI_DATA_READ;
  }

  if (mode & ARM_MCI_TRANSFER_STREAM) {
    /* Stream or SDIO multibyte data transfer enable */
    dctrl |= SDMMC_DCTRL_DTMODE;
  }

  /* Set data block size */
  if (block_size == 512U) {
    sz = 9U;
  }
  else {
    if (block_size > 16384U) {
      return ARM_DRIVER_ERROR_UNSUPPORTED;
    }
    for (sz = 0U; sz < 14U; sz++) {
      if (block_size & (1UL << sz)) {
        break;
      }
    }
  }

  if (mode & ARM_MCI_TRANSFER_WRITE) {
    /* Enable TX DMA stream */
    if (hdma_sdmmc1_rx.State == HAL_DMA_STATE_READY) {
      LL_DMA_SetPeriphRequest(hdma_sdmmc1_rx.DmaBaseAddress, 4, LL_DMA_REQUEST_0);
      LL_DMA_SetPeriphRequest(hdma_sdmmc1_tx.DmaBaseAddress, 3, hdma_sdmmc1_tx.Init.Request);
    } else {
      return ARM_DRIVER_ERROR;
    }
    if (HAL_DMA_Start_IT (&hdma_sdmmc1_tx, (uint32_t)data, (uint32_t)&(SDMMC1->FIFO), cnt / 4) != HAL_OK) {
      return ARM_DRIVER_ERROR;
    }
  }
  else {
    /* Enable RX DMA stream */
    if (hdma_sdmmc1_tx.State == HAL_DMA_STATE_READY) {
      LL_DMA_SetPeriphRequest(hdma_sdmmc1_tx.DmaBaseAddress, 3, LL_DMA_REQUEST_0);
      LL_DMA_SetPeriphRequest(hdma_sdmmc1_rx.DmaBaseAddress, 4, hdma_sdmmc1_rx.Init.Request);
    } else {
      return ARM_DRIVER_ERROR;
    }
    if (HAL_DMA_Start_IT (&hdma_sdmmc1_rx, (uint32_t)&(SDMMC1->FIFO), (uint32_t)data, cnt / 4) != HAL_OK) {
      return ARM_DRIVER_ERROR;
    }
  }

  MCI.dlen   = cnt;
  MCI.dctrl  = dctrl | (sz << 4) | SDMMC_DCTRL_DMAEN;

  return (ARM_DRIVER_OK);
}


/**
  \fn            int32_t AbortTransfer (void)
  \brief         Abort current read/write data transfer.
  \return        \ref execution_status
*/
static int32_t AbortTransfer (void) {
  int32_t  status;
  uint32_t mask;

  if ((MCI.flags & MCI_SETUP) == 0U) { return ARM_DRIVER_ERROR; }

  status = ARM_DRIVER_OK;

  /* Disable SDIO interrupts */
  mask = SDMMC1->MASK;
  SDMMC1->MASK = 0U;

  /* Disable DMA and clear data transfer bit */
  SDMMC1->DCTRL &= ~(SDMMC_DCTRL_DMAEN | SDMMC_DCTRL_DTEN);

  if (HAL_DMA_Abort (&hdma_sdmmc1_rx) != HAL_OK) {
    status = ARM_DRIVER_ERROR;
  }
  if (HAL_DMA_Abort (&hdma_sdmmc1_tx) != HAL_OK) {
    status = ARM_DRIVER_ERROR;
  }

  /* Clear SDIO FIFO */
  while (SDMMC1->FIFOCNT) {
    SDMMC1->FIFO;
  }

  MCI.status.command_active  = 0U;
  MCI.status.transfer_active = 0U;
  MCI.status.sdio_interrupt  = 0U;
  MCI.status.ccs             = 0U;

  /* Clear pending SDIO interrupts */
  SDMMC1->ICR = SDMMC_ICR_BIT_Msk;

  /* Enable SDIO interrupts */
  SDMMC1->MASK = mask;

  return status;
}


/**
  \fn            int32_t Control (uint32_t control, uint32_t arg)
  \brief         Control MCI Interface.
  \param[in]     control  Operation
  \param[in]     arg      Argument of operation (optional)
  \return        \ref execution_status
*/
static int32_t Control (uint32_t control, uint32_t arg) {
  GPIO_InitTypeDef GPIO_InitStruct;
  uint32_t val, clkdiv, bps;

  if ((MCI.flags & MCI_POWER) == 0U) { return ARM_DRIVER_ERROR; }

  switch (control) {
    case ARM_MCI_BUS_SPEED:
      /* Determine clock divider and set bus speed */
      bps = arg;

      if ((bps < SDIOCLK) || (MCI_BUS_MODE_HS == 0U)) {
        /* bps = SDIOCLK / (clkdiv + 2) */
        clkdiv = (SDIOCLK + bps - 1U) / bps;

        if (clkdiv < 2) { clkdiv  = 0U; }
        else            { clkdiv -= 2U; }

        if (clkdiv > SDMMC_CLKCR_CLKDIV) {
          clkdiv  = SDMMC_CLKCR_CLKDIV;
        }

        SDMMC1->CLKCR = (SDMMC1->CLKCR & ~(SDMMC_CLKCR_CLKDIV | SDMMC_CLKCR_BYPASS)) |
                         SDMMC_CLKCR_CLKEN | clkdiv;
        bps = SDIOCLK / (clkdiv + 2U);
      }
      else {
        /* Max output clock is SDIOCLK */
        SDMMC1->CLKCR |= SDMMC_CLKCR_BYPASS | SDMMC_CLKCR_CLKEN;

        bps = SDIOCLK;
      }

      for (val = (SDIOCLK/5000000U)*20U; val; val--) {
        __NOP(); /* Wait a bit to get stable clock */
      }

      /* Bus speed configured */
      MCI.flags |= MCI_SETUP;
      return ((int32_t)bps);

    case ARM_MCI_BUS_SPEED_MODE:
      switch (arg) {
        case ARM_MCI_BUS_DEFAULT_SPEED:
          /* Speed mode up to 25MHz */
          SDMMC1->CLKCR &= ~SDMMC_CLKCR_NEGEDGE;
          break;
        case ARM_MCI_BUS_HIGH_SPEED:
          /* Speed mode up to 50MHz */
          /* Errata: configuration with the NEGEDGE bit set should not be used. */
          break;
        default: return ARM_DRIVER_ERROR_UNSUPPORTED;
      }
      break;

    case ARM_MCI_BUS_CMD_MODE:
      switch (arg) {
        case ARM_MCI_BUS_CMD_OPEN_DRAIN:
          /* Configure command line in open-drain mode */
          val = GPIO_MODE_AF_OD;
          break;
        case ARM_MCI_BUS_CMD_PUSH_PULL:
          /* Configure command line in push-pull mode */
          val = GPIO_MODE_AF_PP;
          break;
        default:
          return ARM_DRIVER_ERROR_UNSUPPORTED;
      }

      GPIO_InitStruct.Pin = GPIO_PIN_2;
      GPIO_InitStruct.Mode = val;
      GPIO_InitStruct.Pull = GPIO_PULLUP;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
      HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
      break;

    case ARM_MCI_BUS_DATA_WIDTH:
      switch (arg) {
        case ARM_MCI_BUS_DATA_WIDTH_1:
          SDMMC1->CLKCR &= ~SDMMC_CLKCR_WIDBUS;
          break;
        case ARM_MCI_BUS_DATA_WIDTH_4:
          SDMMC1->CLKCR = (SDMMC1->CLKCR & ~SDMMC_CLKCR_WIDBUS) | SDMMC_CLKCR_WIDBUS_0;
          break;
        case ARM_MCI_BUS_DATA_WIDTH_8:
          SDMMC1->CLKCR = (SDMMC1->CLKCR & ~SDMMC_CLKCR_WIDBUS) | SDMMC_CLKCR_WIDBUS_1;
          break;
        default:
          return ARM_DRIVER_ERROR_UNSUPPORTED;
      }
      break;

    case ARM_MCI_CONTROL_CLOCK_IDLE:
      if (arg) {
        /* Clock generation enabled when idle */
        SDMMC1->CLKCR &= ~SDMMC_CLKCR_PWRSAV;
      }
      else {
        /* Clock generation disabled when idle */
        SDMMC1->CLKCR |= SDMMC_CLKCR_PWRSAV;
      }
      break;

    case ARM_MCI_DATA_TIMEOUT:
      SDMMC1->DTIMER = arg;
      break;

    case ARM_MCI_MONITOR_SDIO_INTERRUPT:
      MCI.status.sdio_interrupt = 0U;
      SDMMC1->MASK |= SDMMC_MASK_SDIOITIE;
      break;

    case ARM_MCI_CONTROL_READ_WAIT:
      if (arg) {
        /* Assert read wait */
        MCI.flags |= MCI_READ_WAIT;
      }
      else {
        /* Clear read wait */
        MCI.flags &= ~MCI_READ_WAIT;
        SDMMC1->DCTRL &= ~SDMMC_DCTRL_RWSTOP;
      }
      break;

    default: return ARM_DRIVER_ERROR_UNSUPPORTED;
  }

  return ARM_DRIVER_OK;
}


/**
  \fn            ARM_MCI_STATUS GetStatus (void)
  \brief         Get MCI status.
  \return        MCI status \ref ARM_MCI_STATUS
*/
static ARM_MCI_STATUS GetStatus (void) {
  return MCI.status;
}


/* SDMMC IRQ Handler */
void SDMMC1_IRQHandler (void) {
  uint32_t sta, icr, event, mask;

  event = 0U;
  icr   = 0U;

  /* Read SDIO interrupt status */
  sta = SDMMC1->STA;

  if (sta & SDMMC_STA_ERR_BIT_Msk) {
    /* Check error interrupts */
    if (sta & SDMMC_STA_CCRCFAIL) {
      icr |= SDMMC_ICR_CCRCFAILC;
      /* Command response CRC check failed */
      if (MCI.flags & MCI_RESP_CRC) {
        MCI.status.command_error = 1U;

        event |= ARM_MCI_EVENT_COMMAND_ERROR;
      }
      else {
        /* Ignore CRC error and read the response */
        sta |= SDMMC_STA_CMDREND;
      }
    }
    if (sta & SDMMC_STA_DCRCFAIL) {
      icr |= SDMMC_ICR_DCRCFAILC;
      /* Data block CRC check failed */
      MCI.status.transfer_error = 1U;

      event |= ARM_MCI_EVENT_TRANSFER_ERROR;
    }
    if (sta & SDMMC_STA_CTIMEOUT) {
      icr |= SDMMC_ICR_CTIMEOUTC;
      /* Command response timeout */
      MCI.status.command_timeout = 1U;

      event |= ARM_MCI_EVENT_COMMAND_TIMEOUT;
    }
    if (sta & SDMMC_STA_DTIMEOUT) {
      icr |= SDMMC_ICR_DTIMEOUTC;
      /* Data timeout */
      MCI.status.transfer_timeout = 1U;

      event |= ARM_MCI_EVENT_TRANSFER_TIMEOUT;
    }
    if (sta & SDMMC_STA_STBITERR) {
      icr |= SDMMC_ICR_STBITERRC;
      /* Start bit not detected on all data signals */
      event |= ARM_MCI_EVENT_TRANSFER_ERROR;
    }
  }

  if (sta & SDMMC_STA_CMDREND) {
    icr |= SDMMC_ICR_CMDRENDC;
    /* Command response received */
    event |= ARM_MCI_EVENT_COMMAND_COMPLETE;

    if (MCI.response) {
      /* Read response registers */
      if (MCI.flags & MCI_RESP_LONG) {
        MCI.response[0] = SDMMC1->RESP4;
        MCI.response[1] = SDMMC1->RESP3;
        MCI.response[2] = SDMMC1->RESP2;
        MCI.response[3] = SDMMC1->RESP1;
      }
      else {
        MCI.response[0] = SDMMC1->RESP1;
      }
    }
    if (MCI.flags & MCI_DATA_XFER) {
      MCI.flags &= ~MCI_DATA_XFER;

      if (MCI.flags & MCI_READ_WAIT) {
        MCI.dctrl |= SDMMC_DCTRL_RWSTART;
      }

      /* Start data transfer */
      SDMMC1->DLEN   = MCI.dlen;
      SDMMC1->DCTRL  = MCI.dctrl | SDMMC_DCTRL_DTEN;

      MCI.status.transfer_active = 1U;
    }
  }
  if (sta & SDMMC_STA_CMDSENT) {
    icr |= SDMMC_ICR_CMDSENTC;
    /* Command sent (no response required) */
    event |= ARM_MCI_EVENT_COMMAND_COMPLETE;
  }
  if (sta & SDMMC_STA_DATAEND) {
    icr |= SDMMC_ICR_DATAENDC;
    /* Data end (DCOUNT is zero) */
    if ((MCI.flags & MCI_DATA_READ) == 0) {
    /* Write transfer */
      SDMMC1->MASK |= SDMMC_MASK_DBCKENDIE;
    }
  }
  if (sta & SDMMC_STA_DBCKEND) {
    icr |= SDMMC_ICR_DBCKENDC;
    /* Data block sent/received (CRC check passed) */
    if ((MCI.flags & MCI_DATA_READ) == 0) {
      /* Write transfer */
      if (MCI.xfer.cnt == 0) {
        event |= ARM_MCI_EVENT_TRANSFER_COMPLETE;
      }
    }
    SDMMC1->MASK &= ~SDMMC_MASK_DBCKENDIE;
  }
  if (sta & SDMMC_STA_SDIOIT) {
    icr |= SDMMC_ICR_SDIOITC;
    /* Disable interrupt (must be re-enabled using Control) */
    SDMMC1->MASK &= SDMMC_MASK_SDIOITIE;

    event |= ARM_MCI_EVENT_SDIO_INTERRUPT;
  }

  /* Clear processed interrupts */
  SDMMC1->ICR = icr;

  if (event) {
    /* Check for transfer events */
    mask = ARM_MCI_EVENT_TRANSFER_ERROR   |
           ARM_MCI_EVENT_TRANSFER_TIMEOUT |
           ARM_MCI_EVENT_TRANSFER_COMPLETE;
    if (event & mask) {
      MCI.status.transfer_active = 0U;

      if (MCI.cb_event) {
        if (event & ARM_MCI_EVENT_TRANSFER_ERROR) {
          (MCI.cb_event)(ARM_MCI_EVENT_TRANSFER_ERROR);
        }
        else if (event & ARM_MCI_EVENT_TRANSFER_TIMEOUT) {
          (MCI.cb_event)(ARM_MCI_EVENT_TRANSFER_TIMEOUT);
        }
        else {
          (MCI.cb_event)(ARM_MCI_EVENT_TRANSFER_COMPLETE);
        }
      }
    }
    /* Check for command events */
    mask = ARM_MCI_EVENT_COMMAND_ERROR   |
           ARM_MCI_EVENT_COMMAND_TIMEOUT |
           ARM_MCI_EVENT_COMMAND_COMPLETE;
    if (event & mask) {
      MCI.status.command_active = 0U;

      if (MCI.cb_event) {
        if (event & ARM_MCI_EVENT_COMMAND_ERROR) {
          (MCI.cb_event)(ARM_MCI_EVENT_COMMAND_ERROR);
        }
        else if (event & ARM_MCI_EVENT_COMMAND_TIMEOUT) {
          (MCI.cb_event)(ARM_MCI_EVENT_COMMAND_TIMEOUT);
        }
        else {
          (MCI.cb_event)(ARM_MCI_EVENT_COMMAND_COMPLETE);
        }
      }
    }
    /* Check for SDIO INT event */
    if (event & ARM_MCI_EVENT_SDIO_INTERRUPT) {
      MCI.status.sdio_interrupt = 1U;

      if (MCI.cb_event) {
        (MCI.cb_event)(ARM_MCI_EVENT_SDIO_INTERRUPT);
      }
    }
  }
}


/* Rx DMA Callback */
static void RX_DMA_Complete(struct __DMA_HandleTypeDef *hdma) {

  (void)hdma;

  MCI.status.transfer_active = 0U;

  if (MCI.cb_event) {
    (MCI.cb_event)(ARM_MCI_EVENT_TRANSFER_COMPLETE);
  }
}

/* MCI Driver Control Block */
ARM_DRIVER_MCI Driver_MCI0 = {
  GetVersion,
  GetCapabilities,
  Initialize,
  Uninitialize,
  PowerControl,
  CardPower,
  ReadCD,
  ReadWP,
  SendCommand,
  SetupTransfer,
  AbortTransfer,
  Control,
  GetStatus
};
