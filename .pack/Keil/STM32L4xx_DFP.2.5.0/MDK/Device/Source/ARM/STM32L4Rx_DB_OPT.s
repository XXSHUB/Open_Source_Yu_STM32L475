;/*******************************************************************************/
;/* Copyright (c) 2015 - 2019 Arm Limited (or its affiliates). All              */
;/* rights reserved.                                                            */
;/*                                                                             */
;/* SPDX-License-Identifier: BSD-3-Clause                                       */
;/*                                                                             */
;/* Redistribution and use in source and binary forms, with or without          */
;/* modification, are permitted provided that the following conditions are met: */
;/*   1.Redistributions of source code must retain the above copyright          */
;/*     notice, this list of conditions and the following disclaimer.           */
;/*   2.Redistributions in binary form must reproduce the above copyright       */
;/*     notice, this list of conditions and the following disclaimer in the     */
;/*     documentation and/or other materials provided with the distribution.    */
;/*   3.Neither the name of Arm nor the names of its contributors may be used   */
;/*     to endorse or promote products derived from this software without       */
;/*     specific prior written permission.                                      */
;/*                                                                             */
;/* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" */
;/* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE   */
;/* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE  */
;/* ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE     */
;/* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR         */
;/* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF        */
;/* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS    */
;/* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN     */
;/* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)     */
;/* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE  */
;/* POSSIBILITY OF SUCH DAMAGE.                                                 */
;/*******************************************************************************/
;/* STM32L4Rx_DB_OPT.s: STM32L4Rx/STM32L4Rx Dual Bank Flash Option Bytes        */
;/*******************************************************************************/
;/* <<< Use Configuration Wizard in Context Menu >>>                            */
;/*******************************************************************************/


;// <e> Flash Option Bytes
FLASH_OPT       EQU     1

;// <h> Flash Read Protection
;//     <i> Read protection is used to protect the software code stored in Flash memory
;//   <o0.0..7> Read Protection Level
;//     <i> Level 0: No Protection
;//     <i> Level 1: Read Protection of Memories (debug features limited)
;//     <i> Level 2: Chip Protection (debug and boot in RAM features disabled)
;//          <0xAA=> Level 0 (No Protection)
;//          <0x00=> Level 1 (Read Protection of Memories)
;//          <0xCC=> Level 2 (Chip Protection)
;// </h>

;// <h> User Configuration
;//   <o0.27> nBOOT0
;//     <i> nBOOT0 option bit
;//   <o0.26> nSWBOOT0
;//     <i> Software BOOT0
;//          <0=> Option bit nBOOT0
;//          <1=> BOOT0 pin
;//   <o0.25> SRAM2_RST
;//     <i> SRAM2 Erase on system reset
;//          <0=> SRAM2 erased
;//          <1=> SRAM2 not erased
;//   <o0.24> SRAM2_PE
;//     <i> SRAM2 parity check 
;//          <0=> SRAM2 parity check enabled
;//          <1=> SRAM2 parity check disabled
;//   <o0.23> nBOOT1
;//     <i> Boot configuration
;//     <i> Together with the BOOT0 pin, this bit selects boot mode from 
;//     <i> the Flash main memory, SRAM1 or the System memory
;//          <0=> System memory
;//          <1=> Embedded SRAM1
;//   <o0.22> DBANK
;//     <i> can only be written when PCROPA/B is disabled
;//          <0=> Single bank mode
;//          <1=> Dual bank mode
;//   <o0.20> BFB2
;//     <i> Dual-bank boot (Boot from Bank2)
;//          <0=> Boot from Bank1
;//          <1=> Boot from Bank2
;//   <o0.19> WWDG_SW
;//     <i> Window watchdog selection
;//          <0=> Hardware window watchdog
;//          <1=> Software window watchdog
;//   <o0.18> IWDG_STDBY
;//     <i> Independent watchdog counter freeze in Standby mode
;//          <0=> Freeze IWDG counter in Standby mode
;//          <1=> IWDG counter active in Standby mode
;//   <o0.17> IWDG_STOP
;//     <i> Independent watchdog counter freeze in Stop mode
;//          <0=> Freeze IWDG counter in STOP mode
;//          <1=> IWDG counter active in STOP mode
;//   <o0.16> IWDG_SW
;//     <i> Independent watchdog selection
;//          <0=> Hardware independant watchdog
;//          <1=> Software independant watchdog
;//   <o0.14> nRST_SHDW
;//     <i> Generate Reset when entering Shutdown Mode
;//          <0=> Reset generated
;//          <1=> Reset not generated
;//   <o0.13> nRST_STDBY
;//     <i> Generate Reset when entering Standby Mode
;//          <0=> Reset generated
;//          <1=> Reset not generated
;//   <o0.12> nRST_STOP
;//     <i> Generate Reset when entering STOP Mode
;//          <0=> Reset generated
;//          <1=> Reset not generated
;//   <o0.8..10> BOR_LEV
;//     <i>These bits contain the VDD supply level threshold that activates/releases the reset.
;//          <0=> BOR Level 0 (Reset level threshold is around 1.7 V)
;//          <1=> BOR Level 1 (Reset level threshold is around 2.0 V)
;//          <2=> BOR Level 2 (Reset level threshold is around 2.2 V)
;//          <3=> BOR Level 3 (Reset level threshold is around 2.5 V)
;//          <4=> BOR Level 4 (Reset level threshold is around 2.8 V)
;// </h>
FLASH_OPTR     EQU     0xFFCFF8AA       ; reset value 0xFFEFF8AA

;// <h> PCROP / WRP Configuration (Bank1)
;//   <o1.31> PCROP_RDP
;//     <i>   Bit is set only! Bit is reset when changing RDP level from 1 to 0
;//     <i>   checked: PCROP area erased when RDP level is decreased from 1 to 0 (full mass erase)
;//     <i>   unchecked: PCROP area is not erased when RDP level is decreased from 1 to 0
;//   <o0.0..16> PCROP_STRT
;//     <i> PCROP area start offset
;//     <i> DBANK=1: PCROP1_STRT contains the first double-word of the PCROP area for bank1
;//     <i> DBANK=0: PCROP1_STRT contains the first 2xdouble-word of the PCROP area for all memory
;//   <o1.0..16> PCROP_END
;//     <i> PCROP area end offset
;//     <i> DBANK=1: PCROP_END contains the last double-word of the bank1 PCROP area
;//     <i> DBANK=0: PCROP_END contains the last 2x double-word PCROP area for all memory
;//   <o2.0..7> WRP_A_STRT
;//     <i> WRP first area "A" start offset
;//     <i> DBANK=1: WRP_A_STRT contains the first page of WRP first area for bank1
;//     <i> DBANK=0: WRP_A_STRT contains the first page of WRP first area for all memory
;//   <o2.16..23> WRP_A_END
;//     <i> WRP first area "A" end offset
;//     <i> DBANK=1: WRP_A_END contains the last page of WRP first area in bank1
;//     <i> DBANK=0: WRP_A_END contains the last page of WRP first area for all memory
;//   <o3.0..7> WRP_B_STRT
;//     <i> WRP second area "B" start offset
;//     <i> DBANK=1: WRP_B_STRT contains the first page of WRP second area in bank1
;//     <i> DBANK=0: WRP_B_STRT contains the first page of WRP second area for all memory
;//   <o3.16..23> WRP_B_END
;//     <i> WRP second area "B" end offset
;//     <i> DBANK=1: WRP_B_END contains the last page of WRP second area in bank1
;//     <i> DBANK=0: WRP_B_END contains the last page of WRP second area for all memory
;// </h>
FLASH_PCROP1SR EQU     0x0001FFFF       ; reset value 0xFFFFFFFF
FLASH_PCROP1ER EQU     0x00000000       ; reset value 0x00000000
FLASH_WRP1AR   EQU     0x000000FF       ; reset value 0x000000FF
FLASH_WRP1BR   EQU     0x000000FF       ; reset value 0x000000FF

;// <h> PCROP / WRP Configuration (Bank2)
;//   <o0.0..16> PCROP_STRT
;//     <i> PCROP area start offset
;//     <i> DBANK=1: PCROP1_STRT contains the first double-word of the PCROP area for bank2
;//     <i> DBANK=0: PCROP1_STRT contains the first 2xdouble-word of the PCROP area for all memory
;//   <o1.0..16> PCROP_END
;//     <i> PCROP area end offset
;//     <i> DBANK=1: PCROP_END contains the last double-word of the bank2 PCROP area
;//     <i> DBANK=0: PCROP_END contains the last 2x double-word PCROP area for all memory
;//   <o2.0..7> WRP_A_STRT
;//     <i> WRP first area "A" start offset
;//     <i> DBANK=1: WRP_A_STRT contains the first page of WRP first area for bank2
;//     <i> DBANK=0: WRP_A_STRT contains the first page of WRP first area for all memory
;//   <o2.16..23> WRP_A_END
;//     <i> WRP first area "A" end offset
;//     <i> DBANK=1: WRP_A_END contains the last page of WRP first area in bank2
;//     <i> DBANK=0: WRP_A_END contains the last page of WRP first area for all memory
;//   <o3.0..7> WRP_B_STRT
;//     <i> WRP second area "B" start offset
;//     <i> DBANK=1: WRP_B_STRT contains the first page of WRP second area in bank2
;//     <i> DBANK=0: WRP_B_STRT contains the first page of WRP second area for all memory
;//   <o3.16..23> WRP_B_END
;//     <i> WRP second area "B" end offset
;//     <i> DBANK=1: WRP_B_END contains the last page of WRP second area in bank2
;//     <i> DBANK=0: WRP_B_END contains the last page of WRP second area for all memory
;// </h>
FLASH_PCROP2SR EQU     0x0001FFFF       ; reset value 0xFFFFFFFF
FLASH_PCROP2ER EQU     0x00000000       ; reset value 0x00000000
FLASH_WRP2AR   EQU     0x000000FF       ; reset value 0xFF00FFFF
FLASH_WRP2BR   EQU     0x000000FF       ; reset value 0xFF00FFFF

;// </e>


                IF      FLASH_OPT <> 0
                AREA    |.ARM.__AT_0x1FF00000|, CODE, READONLY
                DCD     FLASH_OPTR
                DCD     FLASH_PCROP1SR
                DCD     FLASH_PCROP1ER
                DCD     FLASH_WRP1AR
                DCD     FLASH_WRP1BR
                DCD     FLASH_PCROP2SR
                DCD     FLASH_PCROP2ER
                DCD     FLASH_WRP2AR
                DCD     FLASH_WRP2BR
                ENDIF

                END
