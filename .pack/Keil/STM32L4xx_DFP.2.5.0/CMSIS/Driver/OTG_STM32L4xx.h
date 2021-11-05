/* -----------------------------------------------------------------------------
 * Copyright (c) 2013-2019 Arm Limited (or its affiliates). All 
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
 * $Date:        11. December 2019
 * $Revision:    V1.1
 *
 * Project:      OTG Full/Low-Speed Driver Header for ST STM32L4xx
 * -------------------------------------------------------------------------- */

#ifndef __OTG_STM32L4XX_H
#define __OTG_STM32L4XX_H

#include "stm32l4xx.h"
#include "stm32l4xx_hal.h"

// Global functions and variables exported by driver .c module
extern uint8_t otg_fs_role;

extern void OTG_FS_IRQHandler (void);

#endif /* __OTG_STM32L4XX_H */
