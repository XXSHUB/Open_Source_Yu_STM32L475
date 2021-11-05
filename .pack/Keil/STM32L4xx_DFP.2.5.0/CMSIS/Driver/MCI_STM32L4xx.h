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
 * $Date:        28. July 2020
 * $Revision:    V1.2
 *
 * Project:      MCI Driver Definitions for ST STM32l4xx
 * -------------------------------------------------------------------------- */

#ifndef __MCI_STM32L4XX_H
#define __MCI_STM32L4XX_H

#include "Driver_MCI.h"
#include "stm32l4xx_hal.h"

#include "MX_Device.h"

#include <string.h>

/* Define 4-bit data bus width */
#if defined(MX_SDIO_D0_Pin) && defined(MX_SDIO_D1_Pin) && defined(MX_SDIO_D2_Pin) && defined(MX_SDIO_D3_Pin)
  #define MCI_BUS_WIDTH_4   1U
#else
  #define MCI_BUS_WIDTH_4   0U
#endif

/* Define 8-bit data bus width */
#if defined(MX_SDIO_D0_Pin) && defined(MX_SDIO_D1_Pin) && defined(MX_SDIO_D2_Pin) && defined(MX_SDIO_D3_Pin) && \
    defined(MX_SDIO_D4_Pin) && defined(MX_SDIO_D5_Pin) && defined(MX_SDIO_D6_Pin) && defined(MX_SDIO_D7_Pin)
  #define MCI_BUS_WIDTH_8   1U
#else
  #define MCI_BUS_WIDTH_8   0U
#endif

/* Define Card Detect pin existence */
#if defined(MX_MemoryCard_CD_Pin)
  #define MCI_CD_PIN        1U
#else
  #define MCI_CD_PIN        0U
#endif

/* Define Write Protect pin existence */
#if defined(MX_MemoryCard_WP_Pin)
  #define MCI_WP_PIN        1U
#else
  #define MCI_WP_PIN        0U
#endif

/* SDIO Adapter Clock definition */
#define SDIOCLK                  50000000              /* SDIO adapter clock */

/* Interrupt clear Mask */
#define SDMMC_ICR_BIT_Msk       (SDMMC_ICR_CCRCFAILC | \
                                 SDMMC_ICR_DCRCFAILC | \
                                 SDMMC_ICR_CTIMEOUTC | \
                                 SDMMC_ICR_DTIMEOUTC | \
                                 SDMMC_ICR_TXUNDERRC | \
                                 SDMMC_ICR_RXOVERRC  | \
                                 SDMMC_ICR_CMDRENDC  | \
                                 SDMMC_ICR_CMDSENTC  | \
                                 SDMMC_ICR_DATAENDC  | \
                                 SDMMC_ICR_DBCKENDC  | \
                                 SDMMC_ICR_SDIOITC)

/* Error interrupt mask */
#define SDMMC_STA_ERR_BIT_Msk   (SDMMC_STA_CCRCFAIL | \
                                 SDMMC_STA_DCRCFAIL | \
                                 SDMMC_STA_CTIMEOUT | \
                                 SDMMC_STA_DTIMEOUT)

/* Driver flag definitions */
#define MCI_INIT      ((uint8_t)0x01)   /* MCI initialized           */
#define MCI_POWER     ((uint8_t)0x02)   /* MCI powered on            */
#define MCI_SETUP     ((uint8_t)0x04)   /* MCI configured            */
#define MCI_RESP_LONG ((uint8_t)0x08)   /* Long response expected    */
#define MCI_RESP_CRC  ((uint8_t)0x10)   /* Check response CRC error  */
#define MCI_DATA_XFER ((uint8_t)0x20)   /* Transfer data             */
#define MCI_DATA_READ ((uint8_t)0x40)   /* Read transfer             */
#define MCI_READ_WAIT ((uint8_t)0x80)   /* Read wait operation start */

#define MCI_RESPONSE_EXPECTED_Msk (ARM_MCI_RESPONSE_SHORT      | \
                                   ARM_MCI_RESPONSE_SHORT_BUSY | \
                                   ARM_MCI_RESPONSE_LONG)

/* MCI Transfer Information Definition */
typedef struct _MCI_XFER {
  uint8_t *buf;                         /* Data buffer                        */
  uint32_t cnt;                         /* Data bytes to transfer             */
} MCI_XFER;

/* MCI Driver State Definition */
typedef struct _MCI_INFO {
  ARM_MCI_SignalEvent_t cb_event;       /* Driver event callback function     */
  ARM_MCI_STATUS        status;         /* Driver status                      */
  uint32_t             *response;       /* Pointer to response buffer         */
  MCI_XFER              xfer;           /* Data transfer description          */
  uint32_t              dctrl;          /* Data control register value        */
  uint32_t              dlen;           /* Data length register value         */
  uint8_t volatile      flags;          /* Driver state flags                 */
  uint8_t               reserved[3];    /* Reserved                           */
} MCI_INFO;

// Global functions and variables exported by driver .c module
extern ARM_DRIVER_MCI Driver_MCI0;

#endif /* __MCI_STM32L4XX_H */
