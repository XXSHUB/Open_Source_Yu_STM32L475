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
 * $Date:        20. October 2020
 * $Revision:    V1.3
 *
 * Driver:       Driver_USBD0
 * Configured:   via CubeMX
 * Project:      USB Full/Low-Speed Device Driver for ST STM32L4xx
 * --------------------------------------------------------------------------
 * Use the following configuration settings in the middleware component
 * to connect to this driver.
 *
 *   Configuration Setting                  Value   USBD Interface
 *   ---------------------                  -----   -------------
 *   Connect to hardware via Driver_USBD# = 0       USB Device
 * --------------------------------------------------------------------------
 * Defines used for driver configuration (at compile time):
 *   USBD0_MAX_ENDPOINT_NUM: defines maximum number of IN/OUT Endpoint pairs
 *                           that driver will support with Control Endpoint 0
 *                           not included, this value impacts driver memory
 *                           requirements
 *     - default value:      7
 *     - maximum value:      7
 * -------------------------------------------------------------------------- */

/* History:
 *  Version 1.3
 *    Removed include of stm32l4xx_hal_pcd.h header
 *    Corrected documentation of STM32CubeMx configuration
 *  Version 1.0
 *    Initial release
 */

/*! \page stm32l4_usbd_only CMSIS-Driver USBD Setup

The CMSIS-Driver USBD requires:
  - Setup of USB clock to 48MHz
  - Configuration of USB Device (FS) Mode
 
For different boards, refer to the hardware schematics to reflect correct setup values.
 
The STM32CubeMX configuration steps for Pinout, Clock, and System Configuration are
listed below. Enter the values that are marked \b bold.
 
Pinout & Configuration tab
--------------------------
  1. Under Connectivity select \b USB and under <b>Mode and Configuration</b>:
     - Mode: enable <b>Device (FS)</b>
     - Configuration: Parameter Settings: not used
     - Configuration: User Constants: not used
     - Configuration: <b>NVIC Settings</b>: enable interrupts
          NVIC Interrupt Table                     | Enabled | Preemption Priority | Sub Priority
          :----------------------------------------|:--------|:--------------------|:--------------
          USB event interrupt through EXTI line 17 |\b ON    | 0                   | 0
     - Configuration: <b>GPIO Settings</b>: review settings, no changes required
          Pin Name | Signal on Pin | GPIO output level | GPIO mode | GPIO Pull-up/Pull-down | Maximum out..| Fast Mode | User Label | Modified
          :--------|:--------------|:------------------|:----------|:-----------------------|:-------------|:----------|:-----------|:---------
          PA11     | USB_DM        | n/a               | n/a       | n/a                    | n/a          | n/a       |.           | OFF
          PA12     | USB_DP        | n/a               | n/a       | n/a                    | n/a          | n/a       |.           | OFF
 
Clock Configuration tab
-----------------------
  1. Configure USB Clock: "To USB (MHz)": \b 48
 
Generate source code by click on Generate source code tool on toolbar, or select project Generate Code, or
or press keys Ctrl + Shift + G.
*/

/*! \cond */

#include "USBD_STM32L4xx.h"

#include <string.h>

#include "stm32l4xx.h"


#ifndef USBD0_MAX_ENDPOINT_NUM
#define USBD0_MAX_ENDPOINT_NUM         (7U)
#endif
#if    (USBD0_MAX_ENDPOINT_NUM > 7U)
#error  Too many Endpoints, maximum IN/OUT Endpoint pairs (without EP0) that this driver supports is 7 !!!
#endif

extern PCD_HandleTypeDef hpcd_USB_FS;
#define HPCD                            hpcd_USB_FS
#define PTR_HPCD                       &hpcd_USB_FS
#define USBD0_BASE                      USB_BASE
#define USBD0_INSTANCE                  USB
#define USBD0_IRQ_NUM                   USB_IRQn

// USBD Driver *****************************************************************

#define ARM_USBD_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1,3)

// Driver Version
static const ARM_DRIVER_VERSION usbd_driver_version = { ARM_USBD_API_VERSION, ARM_USBD_DRV_VERSION };

// Driver Capabilities
static const ARM_USBD_CAPABILITIES usbd_driver_capabilities = {
  0U,   // VBUS Detection
  0U,   // Event VBUS On
  0U    // Event VBUS Off
#if (defined(ARM_USBD_API_VERSION) && (ARM_USBD_API_VERSION >= 0x202U))
, 0U    // Reserved
#endif
};

#define EP_NUM(ep_addr)         ((ep_addr) & ARM_USB_ENDPOINT_NUMBER_MASK)
#define EP_ID(ep_addr)          ((EP_NUM(ep_addr) * 2U) + (((ep_addr) >> 7) & 1U))

typedef struct {                        // Endpoint structure definition
           uint8_t  *data;
           uint32_t  num;
  volatile uint32_t  num_transferred_total;
  volatile uint32_t  num_transferring;
           uint16_t  max_packet_size;
  volatile uint16_t  active;
} ENDPOINT_t;


static ARM_USBD_SignalDeviceEvent_t   SignalDeviceEvent;
static ARM_USBD_SignalEndpointEvent_t SignalEndpointEvent;

static bool                hw_powered     = false;
static bool                hw_initialized = false;
static ARM_USBD_STATE      usbd_state;

static uint32_t            setup_packet[2];
static volatile uint8_t    setup_received = 0U;     // Setup packet received

// Endpoints runtime information
static volatile ENDPOINT_t ep[(USBD0_MAX_ENDPOINT_NUM + 1U) * 2U];


// USBD Driver functions

/**
  \fn          ARM_DRIVER_VERSION USBD_GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRIVER_VERSION
*/
static ARM_DRIVER_VERSION USBD_GetVersion (void) { return usbd_driver_version; }

/**
  \fn          ARM_USBD_CAPABILITIES USBD_GetCapabilities (void)
  \brief       Get driver capabilities.
  \return      \ref ARM_USBD_CAPABILITIES
*/
static ARM_USBD_CAPABILITIES USBD_GetCapabilities (void) { return usbd_driver_capabilities; }

/**
  \fn          int32_t USBD_Initialize (ARM_USBD_SignalDeviceEvent_t   cb_device_event,
                                        ARM_USBD_SignalEndpointEvent_t cb_endpoint_event)
  \brief       Initialize USB Device Interface.
  \param[in]   cb_device_event    Pointer to \ref ARM_USBD_SignalDeviceEvent
  \param[in]   cb_endpoint_event  Pointer to \ref ARM_USBD_SignalEndpointEvent
  \return      \ref execution_status
*/
static int32_t USBD_Initialize (ARM_USBD_SignalDeviceEvent_t   cb_device_event,
                                ARM_USBD_SignalEndpointEvent_t cb_endpoint_event) {

  if (hw_initialized == true) {
    return ARM_DRIVER_OK;
  }

  SignalDeviceEvent   = cb_device_event;
  SignalEndpointEvent = cb_endpoint_event;

  HPCD.Instance = USBD0_INSTANCE;

  hw_initialized = true;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBD_Uninitialize (void)
  \brief       De-initialize USB Device Interface.
  \return      \ref execution_status
*/
static int32_t USBD_Uninitialize (void) {

  HPCD.Instance = NULL;

  hw_initialized = false;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBD_PowerControl (ARM_POWER_STATE state)
  \brief       Control USB Device Interface Power.
  \param[in]   state  Power state
  \return      \ref execution_status
*/
static int32_t USBD_PowerControl (ARM_POWER_STATE state) {

  if ((state != ARM_POWER_OFF)  &&
      (state != ARM_POWER_FULL) &&
      (state != ARM_POWER_LOW)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  switch (state) {
    case ARM_POWER_OFF:
      NVIC_DisableIRQ (USBD0_IRQ_NUM);   // Disable USB interrupt

      // Deinitialize
      HAL_PCD_DeInit(PTR_HPCD);

      // Clear powered flag
      hw_powered =  false;
      break;

    case ARM_POWER_FULL:
      if (hw_initialized == false) {
        return ARM_DRIVER_ERROR;
      }
      if (hw_powered == true) {
        return ARM_DRIVER_OK;
      }

      // Set powered flag
      hw_powered     = true;

      // Initialize
      HAL_PCD_Init (PTR_HPCD);

      NVIC_EnableIRQ (USBD0_IRQ_NUM);    // Enable USB interrupt
      break;

    case ARM_POWER_LOW:
      return ARM_DRIVER_ERROR_UNSUPPORTED;
  }

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBD_DeviceConnect (void)
  \brief       Connect USB Device.
  \return      \ref execution_status
*/
static int32_t USBD_DeviceConnect (void) {

  if (hw_powered == false) { return ARM_DRIVER_ERROR; }

  HAL_PCD_Start(PTR_HPCD);

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBD_DeviceDisconnect (void)
  \brief       Disconnect USB Device.
  \return      \ref execution_status
*/
static int32_t USBD_DeviceDisconnect (void) {

  if (hw_powered == false) { return ARM_DRIVER_ERROR; }

  HAL_PCD_DevDisconnect(PTR_HPCD);

  return ARM_DRIVER_OK;
}

/**
  \fn          ARM_USBD_STATE USBD_DeviceGetState (void)
  \brief       Get current USB Device State.
  \return      Device State \ref ARM_USBD_STATE
*/
static ARM_USBD_STATE USBD_DeviceGetState (void) {
  return usbd_state;
}

/**
  \fn          int32_t USBD_DeviceRemoteWakeup (void)
  \brief       Trigger USB Remote Wakeup.
  \return      \ref execution_status
*/
static int32_t USBD_DeviceRemoteWakeup (void) {

  if (hw_powered == false) { return ARM_DRIVER_ERROR; }

  HAL_PCD_ActivateRemoteWakeup(PTR_HPCD);

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBD_DeviceSetAddress (uint8_t dev_addr)
  \brief       Set USB Device Address.
  \param[in]   dev_addr  Device Address
  \return      \ref execution_status
*/
static int32_t USBD_DeviceSetAddress (uint8_t dev_addr) {

  if (hw_powered == false) { return ARM_DRIVER_ERROR; }

  HAL_PCD_SetAddress(PTR_HPCD, dev_addr);

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBD_ReadSetupPacket (uint8_t *setup)
  \brief       Read setup packet received over Control Endpoint.
  \param[out]  setup  Pointer to buffer for setup packet
  \return      \ref execution_status
*/
static int32_t USBD_ReadSetupPacket (uint8_t *setup) {

  if (hw_powered == false)  { return ARM_DRIVER_ERROR; }
  if (setup_received == 0U) { return ARM_DRIVER_ERROR; }

  setup_received = 0U;
  memcpy(setup, setup_packet, 8);

  if (setup_received != 0U) {           // If new setup packet was received while this was being read
    return ARM_DRIVER_ERROR;
  }

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBD_EndpointConfigure (uint8_t  ep_addr,
                                               uint8_t  ep_type,
                                               uint16_t ep_max_packet_size)
  \brief       Configure USB Endpoint.
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \param[in]   ep_type  Endpoint Type (ARM_USB_ENDPOINT_xxx)
  \param[in]   ep_max_packet_size Endpoint Maximum Packet Size
  \return      \ref execution_status
*/
static int32_t USBD_EndpointConfigure (uint8_t  ep_addr,
                                       uint8_t  ep_type,
                                       uint16_t ep_max_packet_size) {
  uint8_t              ep_num;
  uint16_t             ep_mps;
  volatile ENDPOINT_t *ptr_ep;

  ep_num = EP_NUM(ep_addr);
  ep_mps = ep_max_packet_size & ARM_USB_ENDPOINT_MAX_PACKET_SIZE_MASK;

  if (ep_num > USBD0_MAX_ENDPOINT_NUM) { return ARM_DRIVER_ERROR_PARAMETER; }
  if (hw_powered == false)             { return ARM_DRIVER_ERROR; }

  ptr_ep = &ep[EP_ID(ep_addr)];
  if (ptr_ep->active != 0U)            { return ARM_DRIVER_ERROR_BUSY; }

  // Clear Endpoint transfer and configuration information
  memset((void *)((uint32_t)ptr_ep), 0, sizeof (ENDPOINT_t));

  // Set maximum packet size to requested
  ptr_ep->max_packet_size = ep_mps;

  HAL_PCD_EP_Open(PTR_HPCD, ep_addr, ep_mps, ep_type);

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBD_EndpointUnconfigure (uint8_t ep_addr)
  \brief       Unconfigure USB Endpoint.
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \return      \ref execution_status
*/
static int32_t USBD_EndpointUnconfigure (uint8_t ep_addr) {
  uint8_t              ep_num;
  volatile ENDPOINT_t *ptr_ep;

  ep_num = EP_NUM(ep_addr);
  if (ep_num > USBD0_MAX_ENDPOINT_NUM) { return ARM_DRIVER_ERROR_PARAMETER; }
  if (hw_powered == false)             { return ARM_DRIVER_ERROR; }

  ptr_ep = &ep[EP_ID(ep_addr)];
  if (ptr_ep->active != 0U)            { return ARM_DRIVER_ERROR_BUSY; }

  // Clear Endpoint transfer and configuration information
  memset((void *)((uint32_t)ptr_ep), 0, sizeof (ENDPOINT_t));

  HAL_PCD_EP_Close(PTR_HPCD, ep_addr);

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBD_EndpointStall (uint8_t ep_addr, bool stall)
  \brief       Set/Clear Stall for USB Endpoint.
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \param[in]   stall  Operation
                - \b false Clear
                - \b true Set
  \return      \ref execution_status
*/
static int32_t USBD_EndpointStall (uint8_t ep_addr, bool stall) {
  uint8_t ep_num;

  ep_num = EP_NUM(ep_addr);
  if (ep_num > USBD0_MAX_ENDPOINT_NUM) { return ARM_DRIVER_ERROR_PARAMETER; }
  if (hw_powered == false)             { return ARM_DRIVER_ERROR; }

  if (stall != 0U) {
    // Set STALL
    HAL_PCD_EP_SetStall(PTR_HPCD, ep_addr);
  } else {
    // Clear STALL
    HAL_PCD_EP_ClrStall(PTR_HPCD, ep_addr);
  }

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBD_EndpointTransfer (uint8_t ep_addr, uint8_t *data, uint32_t num)
  \brief       Read data from or Write data to USB Endpoint.
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \param[out]  data Pointer to buffer for data to read or with data to write
  \param[in]   num  Number of data bytes to transfer
  \return      \ref execution_status
*/
static int32_t USBD_EndpointTransfer (uint8_t ep_addr, uint8_t *data, uint32_t num) {
  uint8_t              ep_num;
  bool                 ep_dir;
  volatile ENDPOINT_t *ptr_ep;

  ep_num = EP_NUM(ep_addr);
  if (ep_num > USBD0_MAX_ENDPOINT_NUM) { return ARM_DRIVER_ERROR_PARAMETER; }
  if (hw_powered == false)             { return ARM_DRIVER_ERROR; }

  ptr_ep = &ep[EP_ID(ep_addr)];
  if (ptr_ep->active != 0U)            { return ARM_DRIVER_ERROR_BUSY; }

  ep_dir = (ep_addr & ARM_USB_ENDPOINT_DIRECTION_MASK) == ARM_USB_ENDPOINT_DIRECTION_MASK;

  ptr_ep->active = 1U;

  ptr_ep->data                  = data;
  ptr_ep->num                   = num;
  ptr_ep->num_transferred_total = 0U;
  ptr_ep->num_transferring      = num;

  if ((ep_addr & 0x7F) == 0) {
    // Only for EP0
    if (ptr_ep->max_packet_size < num) {
      ptr_ep->num_transferring = ptr_ep->max_packet_size;
    }
  }

  if (ep_dir != 0U) {
    // IN Endpoint
    HAL_PCD_EP_Transmit(PTR_HPCD, ep_addr, (uint8_t *)data, ptr_ep->num_transferring);
  } else {
    // OUT Endpoint
    HAL_PCD_EP_Receive(PTR_HPCD, ep_addr, (uint8_t *)data, ptr_ep->num_transferring);
  }
  

  return ARM_DRIVER_OK;
}

/**
  \fn          uint32_t USBD_EndpointTransferGetResult (uint8_t ep_addr)
  \brief       Get result of USB Endpoint transfer.
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \return      number of successfully transferred data bytes
*/
static uint32_t USBD_EndpointTransferGetResult (uint8_t ep_addr) {
  volatile ENDPOINT_t *ptr_ep;

  ptr_ep = &ep[EP_ID(ep_addr)];
  return (ptr_ep->num_transferred_total);
}

/**
  \fn          int32_t USBD_EndpointTransferAbort (uint8_t ep_addr)
  \brief       Abort current USB Endpoint transfer.
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \return      \ref execution_status
*/
static int32_t USBD_EndpointTransferAbort (uint8_t ep_addr) {
           uint8_t       ep_num;
  volatile ENDPOINT_t   *ptr_ep;
  volatile uint32_t      tout;

  ep_num = EP_NUM(ep_addr);
  if (ep_num > USBD0_MAX_ENDPOINT_NUM) { return ARM_DRIVER_ERROR_PARAMETER; }
  if (hw_powered == false)             { return ARM_DRIVER_ERROR; }

  HAL_PCD_EP_Flush(PTR_HPCD, ep_addr);

  ptr_ep = &ep[EP_ID(ep_addr)];
  ptr_ep->active = 0U;

  return ARM_DRIVER_OK;
}

/**
  \fn          uint16_t USBD_GetFrameNumber (void)
  \brief       Get current USB Frame Number.
  \return      Frame Number
*/
static uint16_t USBD_GetFrameNumber (void) {

  if (hw_powered == false) { return 0U; }
  return (USBD0_INSTANCE->FNR & 0x07FFU);
}

// Callbacks from HAL
/**
  * @brief  Data OUT stage callback.
  * @param  hpcd: PCD handle
  * @param  epnum: endpoint number
  * @retval None
  */
void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum) {
  uint16_t             cnt;
  volatile ENDPOINT_t *ptr_ep;

  /* Prevent unused argument(s) compilation warning */
  UNUSED(hpcd);

  ptr_ep = &ep[EP_ID(epnum)];

  if (epnum != 0) {
    if (SignalEndpointEvent != NULL) {
      ptr_ep->active = 0U;
      ptr_ep->num_transferred_total = HAL_PCD_EP_GetRxCount(hpcd, epnum);
      SignalEndpointEvent(epnum, ARM_USBD_EVENT_OUT);
    }
  } else {
    cnt = HAL_PCD_EP_GetRxCount(hpcd, epnum);
    ptr_ep->num_transferred_total += cnt;
    if ((cnt < ptr_ep->max_packet_size) || (ptr_ep->num_transferred_total == ptr_ep->num)) {
      if (SignalEndpointEvent != NULL) {
        ptr_ep->active = 0U;
        SignalEndpointEvent(epnum, ARM_USBD_EVENT_OUT);
      }
    } else {
      ptr_ep->num_transferring = ptr_ep->num - ptr_ep->num_transferred_total;
      if (ptr_ep->num_transferring > ptr_ep->max_packet_size) {
        ptr_ep->num_transferring = ptr_ep->max_packet_size;
      }
      HAL_PCD_EP_Receive(PTR_HPCD, epnum, (uint8_t *)(ptr_ep->data + ptr_ep->num_transferred_total), ptr_ep->num_transferring);
    }
  }
}

/**
  * @brief  Data IN stage callback.
  * @param  hpcd: PCD handle
  * @param  epnum: endpoint number
  * @retval None
  */
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum) {
  volatile ENDPOINT_t *ptr_ep;
  
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hpcd);

  ptr_ep = &ep[EP_ID(epnum | 0x80)];

  ptr_ep->num_transferred_total += ptr_ep->num_transferring;
  if (epnum != 0) {
    if (SignalEndpointEvent != NULL) {
      ptr_ep->active = 0U;
      SignalEndpointEvent(epnum | ARM_USB_ENDPOINT_DIRECTION_MASK, ARM_USBD_EVENT_IN);
    }
  } else {
    // EP0
    if(ptr_ep->num_transferred_total == ptr_ep->num) {
      if (SignalEndpointEvent != NULL) {
        ptr_ep->active = 0U;
        SignalEndpointEvent(epnum | ARM_USB_ENDPOINT_DIRECTION_MASK, ARM_USBD_EVENT_IN);
      }
    } else {
      ptr_ep->num_transferring = ptr_ep->num - ptr_ep->num_transferred_total;
      if (ptr_ep->num_transferring > ptr_ep->max_packet_size) {
        ptr_ep->num_transferring = ptr_ep->max_packet_size;
      }
        HAL_PCD_EP_Transmit(PTR_HPCD, epnum | 0x80, (uint8_t *)(ptr_ep->data + ptr_ep->num_transferred_total), ptr_ep->num_transferring);
    }
  }
}
/**
  * @brief  Setup stage callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd) {
  memcpy(setup_packet, hpcd->Setup, 8);
  setup_received = 1;

  // Analyze Setup packet for SetAddress
  if ((setup_packet[0] & 0xFFFFU) == 0x0500U) {
    USBD_DeviceSetAddress((setup_packet[0] >> 16) & 0xFFU);
  }

  if (SignalEndpointEvent != NULL) {
    SignalEndpointEvent(0, ARM_USBD_EVENT_SETUP);
  }
}

/**
  * @brief  USB Reset callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd) {
  uint8_t i;

  UNUSED(hpcd);

  for (i = 0U; i < ((USBD0_MAX_ENDPOINT_NUM + 1U) * 2U); i++) {
    if (ep[i].max_packet_size != 0U) {
      if ((i & 1) == 0U) {      // If OUT endpoint
        (void)HAL_PCD_EP_Close(PTR_HPCD,  i >> 1);
      } else {                  // If IN  endpoint
        (void)HAL_PCD_EP_Close(PTR_HPCD, (i >> 1) | 0x80U);
      }
    }
  }

  memset((void *)((uint32_t)ep), 0U, sizeof(ep));

  if (SignalDeviceEvent != NULL) {
    SignalDeviceEvent(ARM_USBD_EVENT_RESET);
  }
}

/**
  * @brief  Suspend event callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd) {
  UNUSED(hpcd);

  if (SignalDeviceEvent != NULL) {
    SignalDeviceEvent(ARM_USBD_EVENT_SUSPEND);
  }
}

/**
  * @brief  Resume event callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd) {
  UNUSED(hpcd);

  if (SignalDeviceEvent != NULL) {
    SignalDeviceEvent(ARM_USBD_EVENT_RESUME);
  }
}

/**
  * @brief  Incomplete ISO OUT callback.
  * @param  hpcd: PCD handle
  * @param  epnum: endpoint number
  * @retval None
  */
void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum) {
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hpcd);
  UNUSED(epnum);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_PCD_ISOOUTIncompleteCallback could be implemented in the user file
   */ 
}

/**
  * @brief  Incomplete ISO IN callback.
  * @param  hpcd: PCD handle
  * @param  epnum: endpoint number
  * @retval None
  */
void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum) {
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hpcd);
  UNUSED(epnum);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_PCD_ISOINIncompleteCallback could be implemented in the user file
   */ 
}

/**
  * @brief  Connection event callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd) {
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hpcd);

  if (SignalDeviceEvent != NULL) {
    SignalDeviceEvent(ARM_USBD_EVENT_VBUS_ON);
  } 
}

/**
  * @brief  Disconnection event callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd) {
  UNUSED(hpcd);

  if (SignalDeviceEvent != NULL) {
    SignalDeviceEvent(ARM_USBD_EVENT_VBUS_OFF);
  }
}


// Structure exported by driver
ARM_DRIVER_USBD Driver_USBD0 = {
  USBD_GetVersion,
  USBD_GetCapabilities,
  USBD_Initialize,
  USBD_Uninitialize,
  USBD_PowerControl,
  USBD_DeviceConnect,
  USBD_DeviceDisconnect,
  USBD_DeviceGetState,
  USBD_DeviceRemoteWakeup,
  USBD_DeviceSetAddress,
  USBD_ReadSetupPacket,
  USBD_EndpointConfigure,
  USBD_EndpointUnconfigure,
  USBD_EndpointStall,
  USBD_EndpointTransfer,
  USBD_EndpointTransferGetResult,
  USBD_EndpointTransferAbort,
  USBD_GetFrameNumber
};

/*! \endcond */
