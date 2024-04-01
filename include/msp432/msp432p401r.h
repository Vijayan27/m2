/******************************************************************************
*
* Copyright (C) 2012 - 2016 Texas Instruments Incorporated - http://www.ti.com/
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
*  Redistributions of source code must retain the above copyright
*  notice, this list of conditions and the following disclaimer.
*
*  Redistributions in binary form must reproduce the above copyright
*  notice, this list of conditions and the following disclaimer in the
*  documentation and/or other materials provided with the
*  distribution.
*
*  Neither the name of Texas Instruments Incorporated nor the names of
*  its contributors may be used to endorse or promote products derived
*  from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
* A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
* OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
* THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* MSP432P401R Register Definitions
*
* This file includes CMSIS compliant component and register definitions
*
* For legacy components the definitions that are compatible with MSP430 code,
* are included with msp432p401r_classic.h
* 
* With CMSIS definitions, the register defines have been reformatted:
*     ModuleName[ModuleInstance]->RegisterName
*
* Writing to CMSIS bit fields can be done through register level
* or via bitband area access:
*  - ADC14->CTL0 |= ADC14_CTL0_ENC;
*  - BITBAND_PERI(ADC14->CTL0, ADC14_CTL0_ENC_OFS) = 1;
*
* File creation date: 2016-05-09
*
******************************************************************************/

#ifndef __MSP432P401R_H__
#define __MSP432P401R_H__

/* Use standard integer types with explicit width */
#include <stdint.h>

#ifdef __cplusplus
 extern "C" {
#endif

#define __MSP432_HEADER_VERSION__ 2200

/* Remap MSP432 intrinsics to ARM equivalents */
#include "msp_compatibility.h"

#ifndef __CMSIS_CONFIG__
#define __CMSIS_CONFIG__

/** @addtogroup MSP432P401R_Definitions MSP432P401R Definitions
  This file defines all structures and symbols for MSP432P401R:
    - components and registers
    - peripheral base address
    - peripheral ID
    - Peripheral definitions
  @{
*/

/******************************************************************************
*                Processor and Core Peripherals                               *
******************************************************************************/
/** @addtogroup MSP432P401R_CMSIS Device CMSIS Definitions
  Configuration of the Cortex-M4 Processor and Core Peripherals
  @{
*/

/******************************************************************************
* CMSIS-compatible Interrupt Number Definition                                *
******************************************************************************/
typedef enum IRQn
{
  /* Cortex-M4 Processor Exceptions Numbers */
  NonMaskableInt_IRQn         = -14,    /*  2 Non Maskable Interrupt */
  HardFault_IRQn              = -13,    /*  3 Hard Fault Interrupt */
  MemoryManagement_IRQn       = -12,    /*  4 Memory Management Interrupt */
  BusFault_IRQn               = -11,    /*  5 Bus Fault Interrupt */
  UsageFault_IRQn             = -10,    /*  6 Usage Fault Interrupt */
  SVCall_IRQn                 = -5,     /* 11 SV Call Interrupt */
  DebugMonitor_IRQn           = -4,     /* 12 Debug Monitor Interrupt */
  PendSV_IRQn                 = -2,     /* 14 Pend SV Interrupt */
  SysTick_IRQn                = -1,     /* 15 System Tick Interrupt */
  /*  Peripheral Exceptions Numbers */
  PSS_IRQn                    =  0,     /* 16 PSS Interrupt */
  CS_IRQn                     =  1,     /* 17 CS Interrupt */
  PCM_IRQn                    =  2,     /* 18 PCM Interrupt */
  WDT_A_IRQn                  =  3,     /* 19 WDT_A Interrupt */
  FPU_IRQn                    =  4,     /* 20 FPU Interrupt */
  FLCTL_IRQn                  =  5,     /* 21 FLCTL Interrupt */
  COMP_E0_IRQn                =  6,     /* 22 COMP_E0 Interrupt */
  COMP_E1_IRQn                =  7,     /* 23 COMP_E1 Interrupt */
  TA0_0_IRQn                  =  8,     /* 24 TA0_0 Interrupt */
  TA0_N_IRQn                  =  9,     /* 25 TA0_N Interrupt */
  TA1_0_IRQn                  = 10,     /* 26 TA1_0 Interrupt */
  TA1_N_IRQn                  = 11,     /* 27 TA1_N Interrupt */
  TA2_0_IRQn                  = 12,     /* 28 TA2_0 Interrupt */
  TA2_N_IRQn                  = 13,     /* 29 TA2_N Interrupt */
  TA3_0_IRQn                  = 14,     /* 30 TA3_0 Interrupt */
  TA3_N_IRQn                  = 15,     /* 31 TA3_N Interrupt */
  EUSCIA0_IRQn                = 16,     /* 32 EUSCIA0 Interrupt */
  EUSCIA1_IRQn                = 17,     /* 33 EUSCIA1 Interrupt */
  EUSCIA2_IRQn                = 18,     /* 34 EUSCIA2 Interrupt */
  EUSCIA3_IRQn                = 19,     /* 35 EUSCIA3 Interrupt */
  EUSCIB0_IRQn                = 20,     /* 36 EUSCIB0 Interrupt */
  EUSCIB1_IRQn                = 21,     /* 37 EUSCIB1 Interrupt */
  EUSCIB2_IRQn                = 22,     /* 38 EUSCIB2 Interrupt */
  EUSCIB3_IRQn                = 23,     /* 39 EUSCIB3 Interrupt */
  ADC14_IRQn                  = 24,     /* 40 ADC14 Interrupt */
  T32_INT1_IRQn               = 25,     /* 41 T32_INT1 Interrupt */
  T32_INT2_IRQn               = 26,     /* 42 T32_INT2 Interrupt */
  T32_INTC_IRQn               = 27,     /* 43 T32_INTC Interrupt */
  AES256_IRQn                 = 28,     /* 44 AES256 Interrupt */
  RTC_C_IRQn                  = 29,     /* 45 RTC_C Interrupt */
  DMA_ERR_IRQn                = 30,     /* 46 DMA_ERR Interrupt */
  DMA_INT3_IRQn               = 31,     /* 47 DMA_INT3 Interrupt */
  DMA_INT2_IRQn               = 32,     /* 48 DMA_INT2 Interrupt */
  DMA_INT1_IRQn               = 33,     /* 49 DMA_INT1 Interrupt */
  DMA_INT0_IRQn               = 34,     /* 50 DMA_INT0 Interrupt */
  PORT1_IRQn                  = 35,     /* 51 PORT1 Interrupt */
  PORT2_IRQn                  = 36,     /* 52 PORT2 Interrupt */
  PORT3_IRQn                  = 37,     /* 53 PORT3 Interrupt */
  PORT4_IRQn                  = 38,     /* 54 PORT4 Interrupt */
  PORT5_IRQn                  = 39,     /* 55 PORT5 Interrupt */
  PORT6_IRQn                  = 40      /* 56 PORT6 Interrupt */

} IRQn_Type;

/******************************************************************************
* Processor and Core Peripheral Section                                       *
******************************************************************************/
#define __CM4_REV               0x0001    /* Core revision r0p1 */
#define __MPU_PRESENT           1         /* MPU present or not */
#define __NVIC_PRIO_BITS        3         /* Number of Bits used for Prio Levels */
#define __Vendor_SysTickConfig  0         /* Set to 1 if different SysTick Config is used */
#define __FPU_PRESENT           1         /* FPU present or not */

/******************************************************************************
* Available Peripherals                                                       *
******************************************************************************/
#define __MCU_HAS_ADC14__                                                        /*!< Module ADC14 is available */
#define __MCU_HAS_AES256__                                                       /*!< Module AES256 is available */
#define __MCU_HAS_CAPTIO0__                                                      /*!< Module CAPTIO0 is available */
#define __MCU_HAS_CAPTIO1__                                                      /*!< Module CAPTIO1 is available */
#define __MCU_HAS_COMP_E0__                                                      /*!< Module COMP_E0 is available */
#define __MCU_HAS_COMP_E1__                                                      /*!< Module COMP_E1 is available */
#define __MCU_HAS_CRC32__                                                        /*!< Module CRC32 is available */
#define __MCU_HAS_CS__                                                           /*!< Module CS is available */
#define __MCU_HAS_DIO__                                                          /*!< Module DIO is available */
#define __MCU_HAS_DMA__                                                          /*!< Module DMA is available */
#define __MCU_HAS_EUSCI_A0__                                                     /*!< Module EUSCI_A0 is available */
#define __MCU_HAS_EUSCI_A1__                                                     /*!< Module EUSCI_A1 is available */
#define __MCU_HAS_EUSCI_A2__                                                     /*!< Module EUSCI_A2 is available */
#define __MCU_HAS_EUSCI_A3__                                                     /*!< Module EUSCI_A3 is available */
#define __MCU_HAS_EUSCI_B0__                                                     /*!< Module EUSCI_B0 is available */
#define __MCU_HAS_EUSCI_B1__                                                     /*!< Module EUSCI_B1 is available */
#define __MCU_HAS_EUSCI_B2__                                                     /*!< Module EUSCI_B2 is available */
#define __MCU_HAS_EUSCI_B3__                                                     /*!< Module EUSCI_B3 is available */
#define __MCU_HAS_FLCTL__                                                        /*!< Module FLCTL is available */
#define __MCU_HAS_FL_BOOTOVER_MAILBOX__                                          /*!< Module FL_BOOTOVER_MAILBOX is available */
#define __MCU_HAS_PCM__                                                          /*!< Module PCM is available */
#define __MCU_HAS_PMAP__                                                         /*!< Module PMAP is available */
#define __MCU_HAS_PSS__                                                          /*!< Module PSS is available */
#define __MCU_HAS_REF_A__                                                        /*!< Module REF_A is available */
#define __MCU_HAS_RSTCTL__                                                       /*!< Module RSTCTL is available */
#define __MCU_HAS_RTC_C__                                                        /*!< Module RTC_C is available */
#define __MCU_HAS_SYSCTL__                                                       /*!< Module SYSCTL is available */
#define __MCU_HAS_TIMER32__                                                      /*!< Module TIMER32 is available */
#define __MCU_HAS_TIMER_A0__                                                     /*!< Module TIMER_A0 is available */
#define __MCU_HAS_TIMER_A1__                                                     /*!< Module TIMER_A1 is available */
#define __MCU_HAS_TIMER_A2__                                                     /*!< Module TIMER_A2 is available */
#define __MCU_HAS_TIMER_A3__                                                     /*!< Module TIMER_A3 is available */
#define __MCU_HAS_TLV__                                                          /*!< Module TLV is available */
#define __MCU_HAS_WDT_A__                                                        /*!< Module WDT_A is available */

/* Definitions to show that specific ports are available */

#define __MSP432_HAS_PORTA_R__
#define __MSP432_HAS_PORTB_R__
#define __MSP432_HAS_PORTC_R__
#define __MSP432_HAS_PORTD_R__
#define __MSP432_HAS_PORTE_R__
#define __MSP432_HAS_PORTJ_R__

#define __MSP432_HAS_PORT1_R__
#define __MSP432_HAS_PORT2_R__
#define __MSP432_HAS_PORT3_R__
#define __MSP432_HAS_PORT4_R__
#define __MSP432_HAS_PORT5_R__
#define __MSP432_HAS_PORT6_R__
#define __MSP432_HAS_PORT7_R__
#define __MSP432_HAS_PORT8_R__
#define __MSP432_HAS_PORT9_R__
#define __MSP432_HAS_PORT10_R__


/*@}*/ /* end of group MSP432P401R_CMSIS */

/* Include CMSIS Cortex-M4 Core Peripheral Access Layer Header File */
#ifdef __TI_ARM__
/* disable the TI ULP advisor check for the core header file definitions */
#pragma diag_push
#pragma CHECK_ULP("none")
#include "core_cm4.h"
#pragma diag_pop
#else
#include "core_cm4.h"
#endif

/* System Header */
#include "system_msp432p401r.h"

/******************************************************************************
* Definition of standard bits                                                 *
******************************************************************************/
#define BIT0                                     (uint16_t)(0x0001)
#define BIT1                                     (uint16_t)(0x0002)
#define BIT2                                     (uint16_t)(0x0004)
#define BIT3                                     (uint16_t)(0x0008)
#define BIT4                                     (uint16_t)(0x0010)
#define BIT5                                     (uint16_t)(0x0020)
#define BIT6                                     (uint16_t)(0x0040)
#define BIT7                                     (uint16_t)(0x0080)
#define BIT8                                     (uint16_t)(0x0100)
#define BIT9                                     (uint16_t)(0x0200)
#define BITA                                     (uint16_t)(0x0400)
#define BITB                                     (uint16_t)(0x0800)
#define BITC                                     (uint16_t)(0x1000)
#define BITD                                     (uint16_t)(0x2000)
#define BITE                                     (uint16_t)(0x4000)
#define BITF                                     (uint16_t)(0x8000)
#define BIT(x)                                 ((uint16_t)1 << (x))

/******************************************************************************
* Device and peripheral memory map                                            *
******************************************************************************/
/** @addtogroup MSP432P401R_MemoryMap MSP432P401R Memory Mapping
  @{
*/

#define FLASH_BASE                               ((uint32_t)0x00000000)          /*!< Main Flash memory start address */
#define SRAM_BASE                                ((uint32_t)0x20000000)          /*!< SRAM memory start address */
#define PERIPH_BASE                              ((uint32_t)0x40000000)          /*!< Peripherals start address */
#define PERIPH_BASE2                             ((uint32_t)0xE0000000)          /*!< Peripherals start address */

#define ADC14_BASE                            (PERIPH_BASE +0x00012000)          /*!< Base address of module ADC14 registers */
#define AES256_BASE                           (PERIPH_BASE +0x00003C00)          /*!< Base address of module AES256 registers */
#define CAPTIO0_BASE                          (PERIPH_BASE +0x00005400)          /*!< Base address of module CAPTIO0 registers */
#define CAPTIO1_BASE                          (PERIPH_BASE +0x00005800)          /*!< Base address of module CAPTIO1 registers */
#define COMP_E0_BASE                          (PERIPH_BASE +0x00003400)          /*!< Base address of module COMP_E0 registers */
#define COMP_E1_BASE                          (PERIPH_BASE +0x00003800)          /*!< Base address of module COMP_E1 registers */
#define CRC32_BASE                            (PERIPH_BASE +0x00004000)          /*!< Base address of module CRC32 registers */
#define CS_BASE                               (PERIPH_BASE +0x00010400)          /*!< Base address of module CS registers */
#define DIO_BASE                              (PERIPH_BASE +0x00004C00)          /*!< Base address of module DIO registers */
#define DMA_BASE                              (PERIPH_BASE +0x0000E000)          /*!< Base address of module DMA registers */
#define EUSCI_A0_BASE                         (PERIPH_BASE +0x00001000)          /*!< Base address of module EUSCI_A0 registers */
#define EUSCI_A0_SPI_BASE                     (PERIPH_BASE +0x00001000)          /*!< Base address of module EUSCI_A0 registers */
#define EUSCI_A1_BASE                         (PERIPH_BASE +0x00001400)          /*!< Base address of module EUSCI_A1 registers */
#define EUSCI_A1_SPI_BASE                     (PERIPH_BASE +0x00001400)          /*!< Base address of module EUSCI_A1 registers */
#define EUSCI_A2_BASE                         (PERIPH_BASE +0x00001800)          /*!< Base address of module EUSCI_A2 registers */
#define EUSCI_A2_SPI_BASE                     (PERIPH_BASE +0x00001800)          /*!< Base address of module EUSCI_A2 registers */
#define EUSCI_A3_BASE                         (PERIPH_BASE +0x00001C00)          /*!< Base address of module EUSCI_A3 registers */
#define EUSCI_A3_SPI_BASE                     (PERIPH_BASE +0x00001C00)          /*!< Base address of module EUSCI_A3 registers */
#define EUSCI_B0_BASE                         (PERIPH_BASE +0x00002000)          /*!< Base address of module EUSCI_B0 registers */
#define EUSCI_B0_SPI_BASE                     (PERIPH_BASE +0x00002000)          /*!< Base address of module EUSCI_B0 registers */
#define EUSCI_B1_BASE                         (PERIPH_BASE +0x00002400)          /*!< Base address of module EUSCI_B1 registers */
#define EUSCI_B1_SPI_BASE                     (PERIPH_BASE +0x00002400)          /*!< Base address of module EUSCI_B1 registers */
#define EUSCI_B2_BASE                         (PERIPH_BASE +0x00002800)          /*!< Base address of module EUSCI_B2 registers */
#define EUSCI_B2_SPI_BASE                     (PERIPH_BASE +0x00002800)          /*!< Base address of module EUSCI_B2 registers */
#define EUSCI_B3_BASE                         (PERIPH_BASE +0x00002C00)          /*!< Base address of module EUSCI_B3 registers */
#define EUSCI_B3_SPI_BASE                     (PERIPH_BASE +0x00002C00)          /*!< Base address of module EUSCI_B3 registers */
#define FLCTL_BASE                            (PERIPH_BASE +0x00011000)          /*!< Base address of module FLCTL registers */
#define FL_BOOTOVER_MAILBOX_BASE                 ((uint32_t)0x00200000)          /*!< Base address of module FL_BOOTOVER_MAILBOX registers */
#define PCM_BASE                              (PERIPH_BASE +0x00010000)          /*!< Base address of module PCM registers */
#define PMAP_BASE                             (PERIPH_BASE +0x00005000)          /*!< Base address of module PMAP registers */
#define PSS_BASE                              (PERIPH_BASE +0x00010800)          /*!< Base address of module PSS registers */
#define REF_A_BASE                            (PERIPH_BASE +0x00003000)          /*!< Base address of module REF_A registers */
#define RSTCTL_BASE                           (PERIPH_BASE2+0x00042000)          /*!< Base address of module RSTCTL registers */
#define RTC_C_BASE                            (PERIPH_BASE +0x00004400)          /*!< Base address of module RTC_C registers */
#define RTC_C_BCD_BASE                        (PERIPH_BASE +0x00004400)          /*!< Base address of module RTC_C registers */
#define SYSCTL_BASE                           (PERIPH_BASE2+0x00043000)          /*!< Base address of module SYSCTL registers */
#define TIMER32_BASE                          (PERIPH_BASE +0x0000C000)          /*!< Base address of module TIMER32 registers */
#define TIMER_A0_BASE                         (PERIPH_BASE +0x00000000)          /*!< Base address of module TIMER_A0 registers */
#define TIMER_A1_BASE                         (PERIPH_BASE +0x00000400)          /*!< Base address of module TIMER_A1 registers */
#define TIMER_A2_BASE                         (PERIPH_BASE +0x00000800)          /*!< Base address of module TIMER_A2 registers */
#define TIMER_A3_BASE                         (PERIPH_BASE +0x00000C00)          /*!< Base address of module TIMER_A3 registers */
#define TLV_BASE                                 ((uint32_t)0x00201000)          /*!< Base address of module TLV registers */
#define WDT_A_BASE                            (PERIPH_BASE +0x00004800)          /*!< Base address of module WDT_A registers */


/*@}*/ /* end of group MSP432P401R_MemoryMap */

/******************************************************************************
* Definitions for bit band access                                             *
******************************************************************************/
#define BITBAND_SRAM_BASE                     ((uint32_t)(0x22000000))
#define BITBAND_PERI_BASE                     ((uint32_t)(0x42000000))

/* SRAM allows 32 bit bit band access */
#define BITBAND_SRAM(x, b)  (*((__IO uint32_t *) (BITBAND_SRAM_BASE +  (((uint32_t)(uint32_t *)&x) - SRAM_BASE  )*32 + b*4)))
/* peripherals with 8 bit or 16 bit register access allow only 8 bit or 16 bit bit band access, so cast to 8 bit always */
#define BITBAND_PERI(x, b)  (*((__IO  uint8_t *) (BITBAND_PERI_BASE +  (((uint32_t)(uint32_t *)&x) - PERIPH_BASE)*32 + b*4)))

/******************************************************************************
* Peripheral register definitions                                             *
******************************************************************************/
/** @addtogroup MSP432P401R_Peripherals MSP432P401R Peripherals
  MSP432P401R Device Specific Peripheral registers structures
  @{
*/

#if defined ( __CC_ARM )
#pragma anon_unions
#endif


/******************************************************************************
* ADC14 Registers
******************************************************************************/
/** @addtogroup ADC14 MSP432P401R (ADC14)
  @{
*/
typedef struct {
  __IO uint32_t CTL0;                                                            /*!< Control 0 Register */
  __IO uint32_t CTL1;                                                            /*!< Control 1 Register */
  __IO uint32_t LO0;                                                             /*!< Window Comparator Low Threshold 0 Register */
  __IO uint32_t HI0;                                                             /*!< Window Comparator High Threshold 0 Register */
  __IO uint32_t LO1;                                                             /*!< Window Comparator Low Threshold 1 Register */
  __IO uint32_t HI1;                                                             /*!< Window Comparator High Threshold 1 Register */
  __IO uint32_t MCTL[32];                                                        /*!< Conversion Memory Control Register */
  __IO uint32_t MEM[32];                                                         /*!< Conversion Memory Register */
       uint32_t RESERVED0[9];
  __IO uint32_t IER0;                                                            /*!< Interrupt Enable 0 Register */
  __IO uint32_t IER1;                                                            /*!< Interrupt Enable 1 Register */
  __I  uint32_t IFGR0;                                                           /*!< Interrupt Flag 0 Register */
  __I  uint32_t IFGR1;                                                           /*!< Interrupt Flag 1 Register */
  __O  uint32_t CLRIFGR0;                                                        /*!< Clear Interrupt Flag 0 Register */
  __IO uint32_t CLRIFGR1;                                                        /*!< Clear Interrupt Flag 1 Register */
  __IO uint32_t IV;                                                              /*!< Interrupt Vector Register */
} ADC14_Type;

/*@}*/ /* end of group ADC14 */


/******************************************************************************
* AES256 Registers
******************************************************************************/
/** @addtogroup AES256 MSP432P401R (AES256)
  @{
*/
typedef struct {
  __IO uint16_t CTL0;                                                            /*!< AES Accelerator Control Register 0 */
  __IO uint16_t CTL1;                                                            /*!< AES Accelerator Control Register 1 */
  __IO uint16_t STAT;                                                            /*!< AES Accelerator Status Register */
  __O  uint16_t KEY;                                                             /*!< AES Accelerator Key Register */
  __O  uint16_t DIN;                                                             /*!< AES Accelerator Data In Register */
  __O  uint16_t DOUT;                                                            /*!< AES Accelerator Data Out Register */
  __O  uint16_t XDIN;                                                            /*!< AES Accelerator XORed Data In Register */
  __O  uint16_t XIN;                                                             /*!< AES Accelerator XORed Data In Register */
} AES256_Type;

/*@}*/ /* end of group AES256 */


/******************************************************************************
* CAPTIO Registers
******************************************************************************/
/** @addtogroup CAPTIO MSP432P401R (CAPTIO)
  @{
*/
typedef struct {
       uint16_t RESERVED0[7];
  __IO uint16_t CTL;                                                             /*!< Capacitive Touch IO x Control Register */
} CAPTIO_Type;

/*@}*/ /* end of group CAPTIO */


/******************************************************************************
* COMP_E Registers
******************************************************************************/
/** @addtogroup COMP_E MSP432P401R (COMP_E)
  @{
*/
typedef struct {
  __IO uint16_t CTL0;                                                            /*!< Comparator Control Register 0 */
  __IO uint16_t CTL1;                                                            /*!< Comparator Control Register 1 */
  __IO uint16_t CTL2;                                                            /*!< Comparator Control Register 2 */
  __IO uint16_t CTL3;                                                            /*!< Comparator Control Register 3 */
       uint16_t RESERVED0[2];
  __IO uint16_t INT;                                                             /*!< Comparator Interrupt Control Register */
  __I  uint16_t IV;                                                              /*!< Comparator Interrupt Vector Word Register */
} COMP_E_Type;

/*@}*/ /* end of group COMP_E */


/******************************************************************************
* CRC32 Registers
******************************************************************************/
/** @addtogroup CRC32 MSP432P401R (CRC32)
  @{
*/
typedef struct {
  __IO uint16_t DI32;                                                            /*!< Data Input for CRC32 Signature Computation */
       uint16_t RESERVED0;
  __IO uint16_t DIRB32;                                                          /*!< Data In Reverse for CRC32 Computation */
       uint16_t RESERVED1;
  __IO uint16_t INIRES32_LO;                                                     /*!< CRC32 Initialization and Result, lower 16 bits */
  __IO uint16_t INIRES32_HI;                                                     /*!< CRC32 Initialization and Result, upper 16 bits */
  __IO uint16_t RESR32_LO;                                                       /*!< CRC32 Result Reverse, lower 16 bits */
  __IO uint16_t RESR32_HI;                                                       /*!< CRC32 Result Reverse, Upper 16 bits */
  __IO uint16_t DI16;                                                            /*!< Data Input for CRC16 computation */
       uint16_t RESERVED2;
  __IO uint16_t DIRB16;                                                          /*!< CRC16 Data In Reverse */
       uint16_t RESERVED3;
  __IO uint16_t INIRES16;                                                        /*!< CRC16 Initialization and Result register */
       uint16_t RESERVED4[2];
  __IO uint16_t RESR16;                                                          /*!< CRC16 Result Reverse */
} CRC32_Type;

/*@}*/ /* end of group CRC32 */


/******************************************************************************
* CS Registers
******************************************************************************/
/** @addtogroup CS MSP432P401R (CS)
  @{
*/
typedef struct {
  __IO uint32_t KEY;                                                             /*!< Key Register */
  __IO uint32_t CTL0;                                                            /*!< Control 0 Register */
  __IO uint32_t CTL1;                                                            /*!< Control 1 Register */
  __IO uint32_t CTL2;                                                            /*!< Control 2 Register */
  __IO uint32_t CTL3;                                                            /*!< Control 3 Register */
       uint32_t RESERVED0[7];
  __IO uint32_t CLKEN;                                                           /*!< Clock Enable Register */
  __I  uint32_t STAT;                                                            /*!< Status Register */
       uint32_t RESERVED1[2];
  __IO uint32_t IE;                                                              /*!< Interrupt Enable Register */
       uint32_t RESERVED2;
  __I  uint32_t IFG;                                                             /*!< Interrupt Flag Register */
       uint32_t RESERVED3;
  __O  uint32_t CLRIFG;                                                          /*!< Clear Interrupt Flag Register */
       uint32_t RESERVED4;
  __O  uint32_t SETIFG;                                                          /*!< Set Interrupt Flag Register */
       uint32_t RESERVED5;
  __IO uint32_t DCOERCAL0;                                                       /*!< DCO External Resistor Cailbration 0 Register */
  __IO uint32_t DCOERCAL1;                                                       /*!< DCO External Resistor Calibration 1 Register */
} CS_Type;

/*@}*/ /* end of group CS */


/******************************************************************************
* DIO Registers
******************************************************************************/
/** @addtogroup DIO MSP432P401R (DIO)
  @{
*/
typedef struct {
  union {
    __I uint16_t IN;                                                              /*!< Port Pair Input */
    struct {
      __I uint8_t IN_L;                                                           /*!< Low Port Input */
      __I uint8_t IN_H;                                                           /*!< High Port Input */
    };
  };
  union {
    __IO uint16_t OUT;                                                            /*!< Port Pair Output */
    struct {
      __IO uint8_t OUT_L;                                                         /*!< Low Port Output */
      __IO uint8_t OUT_H;                                                         /*!< High Port Output */
    };
  };
  union {
    __IO uint16_t DIR;                                                            /*!< Port Pair Direction */
    struct {
      __IO uint8_t DIR_L;                                                         /*!< Low Port Direction */
      __IO uint8_t DIR_H;                                                         /*!< High Port Direction */
    };
  };
  union {
    __IO uint16_t REN;                                                            /*!< Port Pair Resistor Enable */
    struct {
      __IO uint8_t REN_L;                                                         /*!< Low Port Resistor Enable */
      __IO uint8_t REN_H;                                                         /*!< High Port Resistor Enable */
    };
  };
  union {
    __IO uint16_t DS;                                                             /*!< Port Pair Drive Strength */
    struct {
      __IO uint8_t DS_L;                                                          /*!< Low Port Drive Strength */
      __IO uint8_t DS_H;                                                          /*!< High Port Drive Strength */
    };
  };
  union {
    __IO uint16_t SEL0;                                                           /*!< Port Pair Select 0 */
    struct {
      __IO uint8_t SEL0_L;                                                        /*!< Low Port Select 0 */
      __IO uint8_t SEL0_H;                                                        /*!< High Port Select 0 */
    };
  };
  union {
    __IO uint16_t SEL1;                                                           /*!< Port Pair Select 1 */
    struct {
      __IO uint8_t SEL1_L;                                                        /*!< Low Port Select 1 */
      __IO uint8_t SEL1_H;                                                        /*!< High Port Select 1 */
    };
  };
  __I  uint16_t IV_L;                                                             /*!< Low Port Interrupt Vector Value */
  uint16_t  RESERVED0[3];
  union {
    __IO uint16_t SELC;                                                           /*!< Port Pair Complement Select */
    struct {
      __IO uint8_t SELC_L;                                                        /*!< Low Port Complement Select */
      __IO uint8_t SELC_H;                                                        /*!< High Port Complement Select */
    };
  };
  union {
    __IO uint16_t IES;                                                            /*!< Port Pair Interrupt Edge Select */
    struct {
      __IO uint8_t IES_L;                                                         /*!< Low Port Interrupt Edge Select */
      __IO uint8_t IES_H;                                                         /*!< High Port Interrupt Edge Select */
    };
  };
  union {
    __IO uint16_t IE;                                                             /*!< Port Pair Interrupt Enable */
    struct {
      __IO uint8_t IE_L;                                                          /*!< Low Port Interrupt Enable */
      __IO uint8_t IE_H;                                                          /*!< High Port Interrupt Enable */
    };
  };
  union {
    __IO uint16_t IFG;                                                            /*!< Port Pair Interrupt Flag */
    struct {
      __IO uint8_t IFG_L;                                                         /*!< Low Port Interrupt Flag */
      __IO uint8_t IFG_H;                                                         /*!< High Port Interrupt Flag */
    };
  };
  __I uint16_t IV_H;                                                              /*!< High Port Interrupt Vector Value */
} DIO_PORT_Interruptable_Type;

typedef struct {
  union {
    __I uint16_t IN;                                                              /*!< Port Pair Input */
    struct {
      __I uint8_t IN_L;                                                           /*!< Low Port Input */
      __I uint8_t IN_H;                                                           /*!< High Port Input */
    };
  };
  union {
    __IO uint16_t OUT;                                                            /*!< Port Pair Output */
    struct {
      __IO uint8_t OUT_L;                                                         /*!< Low Port Output */
      __IO uint8_t OUT_H;                                                         /*!< High Port Output */
    };
  };
  union {
    __IO uint16_t DIR;                                                            /*!< Port Pair Direction */
    struct {
      __IO uint8_t DIR_L;                                                         /*!< Low Port Direction */
      __IO uint8_t DIR_H;                                                         /*!< High Port Direction */
    };
  };
  union {
    __IO uint16_t REN;                                                            /*!< Port Pair Resistor Enable */
    struct {
      __IO uint8_t REN_L;                                                         /*!< Low Port Resistor Enable */
      __IO uint8_t REN_H;                                                         /*!< High Port Resistor Enable */
    };
  };
  union {
    __IO uint16_t DS;                                                             /*!< Port Pair Drive Strength */
    struct {
      __IO uint8_t DS_L;                                                          /*!< Low Port Drive Strength */
      __IO uint8_t DS_H;                                                          /*!< High Port Drive Strength */
    };
  };
  union {
    __IO uint16_t SEL0;                                                           /*!< Port Pair Select 0 */
    struct {
      __IO uint8_t SEL0_L;                                                        /*!< Low Port Select 0 */
      __IO uint8_t SEL0_H;                                                        /*!< High Port Select 0 */
    };
  };
  union {
    __IO uint16_t SEL1;                                                           /*!< Port Pair Select 1 */
    struct {
      __IO uint8_t SEL1_L;                                                        /*!< Low Port Select 1 */
      __IO uint8_t SEL1_H;                                                        /*!< High Port Select 1 */
    };
  };
  uint16_t  RESERVED0[4];
  union {
    __IO uint16_t SELC;                                                           /*!< Port Pair Complement Select */
    struct {
      __IO uint8_t SELC_L;                                                        /*!< Low Port Complement Select */
      __IO uint8_t SELC_H;                                                        /*!< High Port Complement Select */
    };
  };
} DIO_PORT_Not_Interruptable_Type;


typedef struct {
  __I uint8_t IN;                                                                 /*!< Port Input */
  uint8_t RESERVED0;
  __IO uint8_t OUT;                                                               /*!< Port Output */
  uint8_t RESERVED1;
  __IO uint8_t DIR;                                                               /*!< Port Direction */
  uint8_t RESERVED2;
  __IO uint8_t REN;                                                               /*!< Port Resistor Enable */
  uint8_t RESERVED3;
  __IO uint8_t DS;                                                                /*!< Port Drive Strength */
  uint8_t RESERVED4;
  __IO uint8_t SEL0;                                                              /*!< Port Select 0 */
  uint8_t RESERVED5;
  __IO uint8_t SEL1;                                                              /*!< Port Select 1 */
  uint8_t RESERVED6;
  __I  uint16_t IV;                                                               /*!< Port Interrupt Vector Value */
  uint8_t RESERVED7[6];
  __IO uint8_t SELC;                                                              /*!< Port Complement Select */
  uint8_t RESERVED8;
  __IO uint8_t IES;                                                               /*!< Port Interrupt Edge Select */
  uint8_t RESERVED9;
  __IO uint8_t IE;                                                                /*!< Port Interrupt Enable */
  uint8_t RESERVED10;
  __IO uint8_t IFG;                                                               /*!< Port Interrupt Flag */
} DIO_PORT_Odd_Interruptable_Type;

typedef struct {
  uint8_t RESERVED0;
  __I uint8_t IN;                                                                 /*!< Port Input */
  uint8_t RESERVED1;
  __IO uint8_t OUT;                                                               /*!< Port Output */
  uint8_t RESERVED2;
  __IO uint8_t DIR;                                                               /*!< Port Direction */
  uint8_t RESERVED3;
  __IO uint8_t REN;                                                               /*!< Port Resistor Enable */
  uint8_t RESERVED4;
  __IO uint8_t DS;                                                                /*!< Port Drive Strength */
  uint8_t RESERVED5;
  __IO uint8_t SEL0;                                                              /*!< Port Select 0 */
  uint8_t RESERVED6;
  __IO uint8_t SEL1;                                                              /*!< Port Select 1 */
  uint8_t RESERVED7[9];
  __IO uint8_t SELC;                                                              /*!< Port Complement Select */
  uint8_t RESERVED8;
  __IO uint8_t IES;                                                               /*!< Port Interrupt Edge Select */
  uint8_t RESERVED9;
  __IO uint8_t IE;                                                                /*!< Port Interrupt Enable */
  uint8_t RESERVED10;
  __IO uint8_t IFG;                                                               /*!< Port Interrupt Flag */
  __I uint16_t IV;                                                                /*!< Port Interrupt Vector Value */
} DIO_PORT_Even_Interruptable_Type;

/*@}*/ /* end of group MSP432P401R_DIO */


/******************************************************************************
* DMA Registers
******************************************************************************/
/** @addtogroup DMA MSP432P401R (DMA)
  @{
*/
typedef struct {
  __I  uint32_t DEVICE_CFG;                                                      /*!< Device Configuration Status */
  __IO uint32_t SW_CHTRIG;                                                       /*!< Software Channel Trigger Register */
       uint32_t RESERVED0[2];
  __IO uint32_t CH_SRCCFG[32];                                                   /*!< Channel n Source Configuration Register */
       uint32_t RESERVED1[28];
  __IO uint32_t INT1_SRCCFG;                                                     /*!< Interrupt 1 Source Channel Configuration */
  __IO uint32_t INT2_SRCCFG;                                                     /*!< Interrupt 2 Source Channel Configuration Register */
  __IO uint32_t INT3_SRCCFG;                                                     /*!< Interrupt 3 Source Channel Configuration Register */
       uint32_t RESERVED2;
  __I  uint32_t INT0_SRCFLG;                                                     /*!< Interrupt 0 Source Channel Flag Register */
  __O  uint32_t INT0_CLRFLG;                                                     /*!< Interrupt 0 Source Channel Clear Flag Register */
} DMA_Channel_Type;

typedef struct {
  __I  uint32_t STAT;                                                            /*!< Status Register */
  __O  uint32_t CFG;                                                             /*!< Configuration Register */
  __IO uint32_t CTLBASE;                                                         /*!< Channel Control Data Base Pointer Register */
  __I  uint32_t ATLBASE;                                                         /*!< Channel Alternate Control Data Base Pointer Register */
  __I  uint32_t WAITSTAT;                                                        /*!< Channel Wait on Request Status Register */
  __O  uint32_t SWREQ;                                                           /*!< Channel Software Request Register */
  __IO uint32_t USEBURSTSET;                                                     /*!< Channel Useburst Set Register */
  __O  uint32_t USEBURSTCLR;                                                     /*!< Channel Useburst Clear Register */
  __IO uint32_t REQMASKSET;                                                      /*!< Channel Request Mask Set Register */
  __O  uint32_t REQMASKCLR;                                                      /*!< Channel Request Mask Clear Register */
  __IO uint32_t ENASET;                                                          /*!< Channel Enable Set Register */
  __O  uint32_t ENACLR;                                                          /*!< Channel Enable Clear Register */
  __IO uint32_t ALTSET;                                                          /*!< Channel Primary-Alternate Set Register */
  __O  uint32_t ALTCLR;                                                          /*!< Channel Primary-Alternate Clear Register */
  __IO uint32_t PRIOSET;                                                         /*!< Channel Priority Set Register */
  __O  uint32_t PRIOCLR;                                                         /*!< Channel Priority Clear Register */
       uint32_t RESERVED4[3];
  __IO uint32_t ERRCLR;                                                          /*!< Bus Error Clear Register */
} DMA_Control_Type;

/*@}*/ /* end of group DMA */


/******************************************************************************
* EUSCI_A Registers
******************************************************************************/
/** @addtogroup EUSCI_A MSP432P401R (EUSCI_A)
  @{
*/
typedef struct {
  __IO uint16_t CTLW0;                                                           /*!< eUSCI_Ax Control Word Register 0 */
  __IO uint16_t CTLW1;                                                           /*!< eUSCI_Ax Control Word Register 1 */
       uint16_t RESERVED0;
  __IO uint16_t BRW;                                                             /*!< eUSCI_Ax Baud Rate Control Word Register */
  __IO uint16_t MCTLW;                                                           /*!< eUSCI_Ax Modulation Control Word Register */
  __IO uint16_t STATW;                                                           /*!< eUSCI_Ax Status Register */
  __I  uint16_t RXBUF;                                                           /*!< eUSCI_Ax Receive Buffer Register */
  __IO uint16_t TXBUF;                                                           /*!< eUSCI_Ax Transmit Buffer Register */
  __IO uint16_t ABCTL;                                                           /*!< eUSCI_Ax Auto Baud Rate Control Register */
  __IO uint16_t IRCTL;                                                           /*!< eUSCI_Ax IrDA Control Word Register */
       uint16_t RESERVED1[3];
  __IO uint16_t IE;                                                              /*!< eUSCI_Ax Interrupt Enable Register */
  __IO uint16_t IFG;                                                             /*!< eUSCI_Ax Interrupt Flag Register */
  __I  uint16_t IV;                                                              /*!< eUSCI_Ax Interrupt Vector Register */
} EUSCI_A_Type;

/*@}*/ /* end of group EUSCI_A */

/** @addtogroup EUSCI_A_SPI MSP432P401R (EUSCI_A_SPI)
  @{
*/
typedef struct {
  __IO uint16_t CTLW0;                                                           /*!< eUSCI_Ax Control Word Register 0 */
       uint16_t RESERVED0[2];
  __IO uint16_t BRW;                                                             /*!< eUSCI_Ax Bit Rate Control Register 1 */
       uint16_t RESERVED1;
  __IO uint16_t STATW;
  __I  uint16_t RXBUF;                                                           /*!< eUSCI_Ax Receive Buffer Register */
  __IO uint16_t TXBUF;                                                           /*!< eUSCI_Ax Transmit Buffer Register */
       uint16_t RESERVED2[5];
  __IO uint16_t IE;                                                              /*!< eUSCI_Ax Interrupt Enable Register */
  __IO uint16_t IFG;                                                             /*!< eUSCI_Ax Interrupt Flag Register */
  __I  uint16_t IV;                                                              /*!< eUSCI_Ax Interrupt Vector Register */
} EUSCI_A_SPI_Type;

/*@}*/ /* end of group EUSCI_A_SPI */


/******************************************************************************
* EUSCI_B Registers
******************************************************************************/
/** @addtogroup EUSCI_B MSP432P401R (EUSCI_B)
  @{
*/
typedef struct {
  __IO uint16_t CTLW0;                                                           /*!< eUSCI_Bx Control Word Register 0 */
  __IO uint16_t CTLW1;                                                           /*!< eUSCI_Bx Control Word Register 1 */
       uint16_t RESERVED0;
  __IO uint16_t BRW;                                                             /*!< eUSCI_Bx Baud Rate Control Word Register */
  __IO uint16_t STATW;                                                           /*!< eUSCI_Bx Status Register */
  __IO uint16_t TBCNT;                                                           /*!< eUSCI_Bx Byte Counter Threshold Register */
  __I  uint16_t RXBUF;                                                           /*!< eUSCI_Bx Receive Buffer Register */
  __IO uint16_t TXBUF;                                                           /*!< eUSCI_Bx Transmit Buffer Register */
       uint16_t RESERVED1[2];
  __IO uint16_t I2COA0;                                                          /*!< eUSCI_Bx I2C Own Address 0 Register */
  __IO uint16_t I2COA1;                                                          /*!< eUSCI_Bx I2C Own Address 1 Register */
  __IO uint16_t I2COA2;                                                          /*!< eUSCI_Bx I2C Own Address 2 Register */
  __IO uint16_t I2COA3;                                                          /*!< eUSCI_Bx I2C Own Address 3 Register */
  __I  uint16_t ADDRX;                                                           /*!< eUSCI_Bx I2C Received Address Register */
  __IO uint16_t ADDMASK;                                                         /*!< eUSCI_Bx I2C Address Mask Register */
  __IO uint16_t I2CSA;                                                           /*!< eUSCI_Bx I2C Slave Address Register */
       uint16_t RESERVED2[4];
  __IO uint16_t IE;                                                              /*!< eUSCI_Bx Interrupt Enable Register */
  __IO uint16_t IFG;                                                             /*!< eUSCI_Bx Interrupt Flag Register */
  __I  uint16_t IV;                                                              /*!< eUSCI_Bx Interrupt Vector Register */
} EUSCI_B_Type;

/*@}*/ /* end of group EUSCI_B */

/** @addtogroup EUSCI_B_SPI MSP432P401R (EUSCI_B_SPI)
  @{
*/
typedef struct {
  __IO uint16_t CTLW0;                                                           /*!< eUSCI_Bx Control Word Register 0 */
       uint16_t RESERVED0[2];
  __IO uint16_t BRW;                                                             /*!< eUSCI_Bx Bit Rate Control Register 1 */
  __IO uint16_t STATW;
       uint16_t RESERVED1;
  __I  uint16_t RXBUF;                                                           /*!< eUSCI_Bx Receive Buffer Register */
  __IO uint16_t TXBUF;                                                           /*!< eUSCI_Bx Transmit Buffer Register */
       uint16_t RESERVED2[13];
  __IO uint16_t IE;                                                              /*!< eUSCI_Bx Interrupt Enable Register */
  __IO uint16_t IFG;                                                             /*!< eUSCI_Bx Interrupt Flag Register */
  __I  uint16_t IV;                                                              /*!< eUSCI_Bx Interrupt Vector Register */
} EUSCI_B_SPI_Type;

/*@}*/ /* end of group EUSCI_B_SPI */


/******************************************************************************
* FLCTL Registers
******************************************************************************/
/** @addtogroup FLCTL MSP432P401R (FLCTL)
  @{
*/
typedef struct {
  __I  uint32_t POWER_STAT;                                                      /*!< Power Status Register */
       uint32_t RESERVED0[3];
  __IO uint32_t BANK0_RDCTL;                                                     /*!< Bank0 Read Control Register */
  __IO uint32_t BANK1_RDCTL;                                                     /*!< Bank1 Read Control Register */
       uint32_t RESERVED1[2];
  __IO uint32_t RDBRST_CTLSTAT;                                                  /*!< Read Burst/Compare Control and Status Register */
  __IO uint32_t RDBRST_STARTADDR;                                                /*!< Read Burst/Compare Start Address Register */
  __IO uint32_t RDBRST_LEN;                                                      /*!< Read Burst/Compare Length Register */
       uint32_t RESERVED2[4];
  __IO uint32_t RDBRST_FAILADDR;                                                 /*!< Read Burst/Compare Fail Address Register */
  __IO uint32_t RDBRST_FAILCNT;                                                  /*!< Read Burst/Compare Fail Count Register */
       uint32_t RESERVED3[3];
  __IO uint32_t PRG_CTLSTAT;                                                     /*!< Program Control and Status Register */
  __IO uint32_t PRGBRST_CTLSTAT;                                                 /*!< Program Burst Control and Status Register */
  __IO uint32_t PRGBRST_STARTADDR;                                               /*!< Program Burst Start Address Register */
       uint32_t RESERVED4;
  __IO uint32_t PRGBRST_DATA0_0;                                                 /*!< Program Burst Data0 Register0 */
  __IO uint32_t PRGBRST_DATA0_1;                                                 /*!< Program Burst Data0 Register1 */
  __IO uint32_t PRGBRST_DATA0_2;                                                 /*!< Program Burst Data0 Register2 */
  __IO uint32_t PRGBRST_DATA0_3;                                                 /*!< Program Burst Data0 Register3 */
  __IO uint32_t PRGBRST_DATA1_0;                                                 /*!< Program Burst Data1 Register0 */
  __IO uint32_t PRGBRST_DATA1_1;                                                 /*!< Program Burst Data1 Register1 */
  __IO uint32_t PRGBRST_DATA1_2;                                                 /*!< Program Burst Data1 Register2 */
  __IO uint32_t PRGBRST_DATA1_3;                                                 /*!< Program Burst Data1 Register3 */
  __IO uint32_t PRGBRST_DATA2_0;                                                 /*!< Program Burst Data2 Register0 */
  __IO uint32_t PRGBRST_DATA2_1;                                                 /*!< Program Burst Data2 Register1 */
  __IO uint32_t PRGBRST_DATA2_2;                                                 /*!< Program Burst Data2 Register2 */
  __IO uint32_t PRGBRST_DATA2_3;                                                 /*!< Program Burst Data2 Register3 */
  __IO uint32_t PRGBRST_DATA3_0;                                                 /*!< Program Burst Data3 Register0 */
  __IO uint32_t PRGBRST_DATA3_1;                                                 /*!< Program Burst Data3 Register1 */
  __IO uint32_t PRGBRST_DATA3_2;                                                 /*!< Program Burst Data3 Register2 */
  __IO uint32_t PRGBRST_DATA3_3;                                                 /*!< Program Burst Data3 Register3 */
  __IO uint32_t ERASE_CTLSTAT;                                                   /*!< Erase Control and Status Register */
  __IO uint32_t ERASE_SECTADDR;                                                  /*!< Erase Sector Address Register */
       uint32_t RESERVED5[2];
  __IO uint32_t BANK0_INFO_WEPROT;                                               /*!< Information Memory Bank0 Write/Erase Protection Register */
  __IO uint32_t BANK0_MAIN_WEPROT;                                               /*!< Main Memory Bank0 Write/Erase Protection Register */
       uint32_t RESERVED6[2];
  __IO uint32_t BANK1_INFO_WEPROT;                                               /*!< Information Memory Bank1 Write/Erase Protection Register */
  __IO uint32_t BANK1_MAIN_WEPROT;                                               /*!< Main Memory Bank1 Write/Erase Protection Register */
       uint32_t RESERVED7[2];
  __IO uint32_t BMRK_CTLSTAT;                                                    /*!< Benchmark Control and Status Register */
  __IO uint32_t BMRK_IFETCH;                                                     /*!< Benchmark Instruction Fetch Count Register */
  __IO uint32_t BMRK_DREAD;                                                      /*!< Benchmark Data Read Count Register */
  __IO uint32_t BMRK_CMP;                                                        /*!< Benchmark Count Compare Register */
       uint32_t RESERVED8[4];
  __IO uint32_t IFG;                                                             /*!< Interrupt Flag Register */
  __IO uint32_t IE;                                                              /*!< Interrupt Enable Register */
  __IO uint32_t CLRIFG;                                                          /*!< Clear Interrupt Flag Register */
  __IO uint32_t SETIFG;                                                          /*!< Set Interrupt Flag Register */
  __I  uint32_t READ_TIMCTL;                                                     /*!< Read Timing Control Register */
  __I  uint32_t READMARGIN_TIMCTL;                                               /*!< Read Margin Timing Control Register */
  __I  uint32_t PRGVER_TIMCTL;                                                   /*!< Program Verify Timing Control Register */
  __I  uint32_t ERSVER_TIMCTL;                                                   /*!< Erase Verify Timing Control Register */
  __I  uint32_t LKGVER_TIMCTL;                                                   /*!< Leakage Verify Timing Control Register */
  __I  uint32_t PROGRAM_TIMCTL;                                                  /*!< Program Timing Control Register */
  __I  uint32_t ERASE_TIMCTL;                                                    /*!< Erase Timing Control Register */
  __I  uint32_t MASSERASE_TIMCTL;                                                /*!< Mass Erase Timing Control Register */
  __I  uint32_t BURSTPRG_TIMCTL;                                                 /*!< Burst Program Timing Control Register */
} FLCTL_Type;

/*@}*/ /* end of group FLCTL */


/******************************************************************************
* FL_BOOTOVER_MAILBOX Registers
******************************************************************************/
/** @addtogroup SEC_ZONE_PARAMS MSP432P401R (FL_BOOTOVER_MAILBOX)
  @{
*/
typedef struct {
  __IO uint32_t SEC_ZONE_SECEN;                                                  /*!< IP Protection Secure Zone Enable. */
  __IO uint32_t SEC_ZONE_START_ADDR;                                             /*!< Start address of IP protected secure zone. */
  __IO uint32_t SEC_ZONE_LENGTH;                                                 /*!< Length of IP protected secure zone in number of bytes. */
  __IO uint32_t SEC_ZONE_AESINIT_VECT[4];                                        /*!< IP protected secure zone 0 AES initialization vector */
  __IO uint32_t SEC_ZONE_SECKEYS[8];                                             /*!< AES-CBC security keys. */
  __IO uint32_t SEC_ZONE_UNENC_PWD[4];                                           /*!< Unencrypted password for authentication. */
  __IO uint32_t SEC_ZONE_ENCUPDATE_EN;                                           /*!< IP Protected Secure Zone Encrypted In-field Update Enable */
  __IO uint32_t SEC_ZONE_DATA_EN;                                                /*!< IP Protected Secure Zone Data Access Enable */
  __IO uint32_t SEC_ZONE_ACK;                                                    /*!< Acknowledgment for IP Protection Secure Zone Enable Command. */
       uint32_t RESERVED0[2];
} SEC_ZONE_PARAMS_Type;

/*@}*/ /* end of group SEC_ZONE_PARAMS */

/** @addtogroup SEC_ZONE_UPDATE MSP432P401R (FL_BOOTOVER_MAILBOX)
  @{
*/
typedef struct {
  __IO uint32_t SEC_ZONE_PAYLOADADDR;                                            /*!< Start address where the payload is loaded in the device. */
  __IO uint32_t SEC_ZONE_PAYLOADLEN;                                             /*!< Length of the payload in bytes. */
  __IO uint32_t SEC_ZONE_UPDATE_ACK;                                             /*!< Acknowledgment for the IP Protected Secure Zone Update Command */
       uint32_t RESERVED0;
} SEC_ZONE_UPDATE_Type;

/*@}*/ /* end of group SEC_ZONE_UPDATE */

/** @addtogroup FL_BOOTOVER_MAILBOX MSP432P401R (FL_BOOTOVER_MAILBOX)
  @{
*/
typedef struct {
  __IO uint32_t MB_START;                                                        /*!< Flash MailBox start: 0x0115ACF6 */
  __IO uint32_t CMD;                                                             /*!< Command for Boot override operations. */
       uint32_t RESERVED0[2];
  __IO uint32_t JTAG_SWD_LOCK_SECEN;                                             /*!< JTAG and SWD Lock Enable */
  __IO uint32_t JTAG_SWD_LOCK_AES_INIT_VECT[4];                                  /*!< JTAG and SWD lock AES initialization vector for AES-CBC */
  __IO uint32_t JTAG_SWD_LOCK_AES_SECKEYS[8];                                    /*!< JTAG and SWD lock AES CBC security Keys 0-7. */
  __IO uint32_t JTAG_SWD_LOCK_UNENC_PWD[4];                                      /*!< JTAG and SWD lock unencrypted password */
  __IO uint32_t JTAG_SWD_LOCK_ACK;                                               /*!< Acknowledgment for JTAG and SWD Lock command */
       uint32_t RESERVED1[2];
  SEC_ZONE_PARAMS_Type SEC_ZONE_PARAMS[4];
  __IO uint32_t BSL_ENABLE;                                                      /*!< BSL Enable. */
  __IO uint32_t BSL_START_ADDRESS;                                               /*!< Contains the pointer to the BSL function. */
  __IO uint32_t BSL_PARAMETERS;                                                  /*!< BSL hardware invoke conifguration field. */
       uint32_t RESERVED2[2];
  __IO uint32_t BSL_ACK;                                                         /*!< Acknowledgment for the BSL Configuration Command */
  __IO uint32_t JTAG_SWD_LOCK_ENCPAYLOADADD;                                     /*!< Start address where the payload is loaded in the device. */
  __IO uint32_t JTAG_SWD_LOCK_ENCPAYLOADLEN;                                     /*!< Length of the encrypted payload in bytes */
  __IO uint32_t JTAG_SWD_LOCK_DST_ADDR;                                          /*!< Destination address where the final data needs to be stored into the device. */
  __IO uint32_t ENC_UPDATE_ACK;                                                  /*!< Acknowledgment for JTAG and SWD Lock Encrypted Update Command */
       uint32_t RESERVED3;
  SEC_ZONE_UPDATE_Type SEC_ZONE_UPDATE[4];                                       /*!< IP Protection Secure Zone Update */
       uint32_t RESERVED4;
  __IO uint32_t FACTORY_RESET_ENABLE;                                            /*!< Enable/Disable Factory Reset */
  __IO uint32_t FACTORY_RESET_PWDEN;                                             /*!< Factory reset password enable */
  __IO uint32_t FACTORY_RESET_PWD[4];                                            /*!< 128-bit Password for factory reset to be saved into the device. */
  __IO uint32_t FACTORY_RESET_PARAMS_ACK;                                        /*!< Acknowledgment for the Factory Reset Params Command */
       uint32_t RESERVED5;
  __IO uint32_t FACTORY_RESET_PASSWORD[4];                                       /*!< 128-bit Password for factory reset. */
  __IO uint32_t FACTORY_RESET_ACK;                                               /*!< Acknowledgment for the Factory Reset Command */
       uint32_t RESERVED6[2];
  __IO uint32_t MB_END;                                                          /*!< Mailbox end */
} FL_BOOTOVER_MAILBOX_Type;

/*@}*/ /* end of group FL_BOOTOVER_MAILBOX */


/******************************************************************************
* PCM Registers
******************************************************************************/
/** @addtogroup PCM MSP432P401R (PCM)
  @{
*/
typedef struct {
  __IO uint32_t CTL0;                                                            /*!< Control 0 Register */
  __IO uint32_t CTL1;                                                            /*!< Control 1 Register */
  __IO uint32_t IE;                                                              /*!< Interrupt Enable Register */
  __I  uint32_t IFG;                                                             /*!< Interrupt Flag Register */
  __O  uint32_t CLRIFG;                                                          /*!< Clear Interrupt Flag Register */
} PCM_Type;

/*@}*/ /* end of group PCM */


/******************************************************************************
* PMAP Registers
******************************************************************************/
/** @addtogroup PMAP MSP432P401R (PMAP)
  @{
*/
typedef struct {
  __IO uint16_t KEYID;
  __IO uint16_t CTL;
} PMAP_COMMON_Type;

typedef struct {
  union {
    __IO uint16_t PMAP_REGISTER[4];
    struct {
      __IO uint8_t PMAP_REGISTER0;
      __IO uint8_t PMAP_REGISTER1;
      __IO uint8_t PMAP_REGISTER2;
      __IO uint8_t PMAP_REGISTER3;
      __IO uint8_t PMAP_REGISTER4;
      __IO uint8_t PMAP_REGISTER5;
      __IO uint8_t PMAP_REGISTER6;
      __IO uint8_t PMAP_REGISTER7;
    };
  };
} PMAP_REGISTER_Type;

/*@}*/ /* end of group PMAP */


/******************************************************************************
* PSS Registers
******************************************************************************/
/** @addtogroup PSS MSP432P401R (PSS)
  @{
*/
typedef struct {
  __IO uint32_t KEY;                                                             /*!< Key Register */
  __IO uint32_t CTL0;                                                            /*!< Control 0 Register */
       uint32_t RESERVED0[11];
  __IO uint32_t IE;                                                              /*!< Interrupt Enable Register */
  __I  uint32_t IFG;                                                             /*!< Interrupt Flag Register */
  __IO uint32_t CLRIFG;                                                          /*!< Clear Interrupt Flag Register */
} PSS_Type;

/*@}*/ /* end of group PSS */


/******************************************************************************
* REF_A Registers
******************************************************************************/
/** @addtogroup REF_A MSP432P401R (REF_A)
  @{
*/
typedef struct {
  __IO uint16_t CTL0;                                                            /*!< REF Control Register 0 */
} REF_A_Type;

/*@}*/ /* end of group REF_A */


/******************************************************************************
* RSTCTL Registers
******************************************************************************/
/** @addtogroup RSTCTL MSP432P401R (RSTCTL)
  @{
*/
typedef struct {
  __IO uint32_t RESET_REQ;                                                       /*!< Reset Request Register */
  __I  uint32_t HARDRESET_STAT;                                                  /*!< Hard Reset Status Register */
  __IO uint32_t HARDRESET_CLR;                                                   /*!< Hard Reset Status Clear Register */
  __IO uint32_t HARDRESET_SET;                                                   /*!< Hard Reset Status Set Register */
  __I  uint32_t SOFTRESET_STAT;                                                  /*!< Soft Reset Status Register */
  __IO uint32_t SOFTRESET_CLR;                                                   /*!< Soft Reset Status Clear Register */
  __IO uint32_t SOFTRESET_SET;                                                   /*!< Soft Reset Status Set Register */
       uint32_t RESERVED0[57];
  __I  uint32_t PSSRESET_STAT;                                                   /*!< PSS Reset Status Register */
  __IO uint32_t PSSRESET_CLR;                                                    /*!< PSS Reset Status Clear Register */
  __I  uint32_t PCMRESET_STAT;                                                   /*!< PCM Reset Status Register */
  __IO uint32_t PCMRESET_CLR;                                                    /*!< PCM Reset Status Clear Register */
  __I  uint32_t PINRESET_STAT;                                                   /*!< Pin Reset Status Register */
  __IO uint32_t PINRESET_CLR;                                                    /*!< Pin Reset Status Clear Register */
  __I  uint32_t REBOOTRESET_STAT;                                                /*!< Reboot Reset Status Register */
  __IO uint32_t REBOOTRESET_CLR;                                                 /*!< Reboot Reset Status Clear Register */
  __I  uint32_t CSRESET_STAT;                                                    /*!< CS Reset Status Register */
  __IO uint32_t CSRESET_CLR;                                                     /*!< CS Reset Status Clear Register */
} RSTCTL_Type;

/*@}*/ /* end of group RSTCTL */


/******************************************************************************
* RTC_C Registers
******************************************************************************/
/** @addtogroup RTC_C MSP432P401R (RTC_C)
  @{
*/
typedef struct {
  __IO uint16_t CTL0;                                                            /*!< RTCCTL0 Register */
  __IO uint16_t CTL13;                                                           /*!< RTCCTL13 Register */
  __IO uint16_t OCAL;                                                            /*!< RTCOCAL Register */
  __IO uint16_t TCMP;                                                            /*!< RTCTCMP Register */
  __IO uint16_t PS0CTL;                                                          /*!< Real-Time Clock Prescale Timer 0 Control Register */
  __IO uint16_t PS1CTL;                                                          /*!< Real-Time Clock Prescale Timer 1 Control Register */
  __IO uint16_t PS;                                                              /*!< Real-Time Clock Prescale Timer Counter Register */
  __I  uint16_t IV;                                                              /*!< Real-Time Clock Interrupt Vector Register */
  __IO uint16_t TIM0;                                                            /*!< RTCTIM0 Register ? Hexadecimal Format */
  __IO uint16_t TIM1;                                                            /*!< Real-Time Clock Hour, Day of Week */
  __IO uint16_t DATE;                                                            /*!< RTCDATE - Hexadecimal Format */
  __IO uint16_t YEAR;                                                            /*!< RTCYEAR Register ? Hexadecimal Format */
  __IO uint16_t AMINHR;                                                          /*!< RTCMINHR - Hexadecimal Format */
  __IO uint16_t ADOWDAY;                                                         /*!< RTCADOWDAY - Hexadecimal Format */
  __IO uint16_t BIN2BCD;                                                         /*!< Binary-to-BCD Conversion Register */
  __IO uint16_t BCD2BIN;                                                         /*!< BCD-to-Binary Conversion Register */
} RTC_C_Type;

/*@}*/ /* end of group RTC_C */

/** @addtogroup RTC_C_BCD MSP432P401R (RTC_C_BCD)
  @{
*/
typedef struct {
       uint16_t RESERVED0[8];
  __IO uint16_t TIM0;                                                            /*!< RTCTIM0 Register ? BCD Format */
  __IO uint16_t TIM1;                                                            /*!< RTCTIM1 Register ? BCD Format */
  __IO uint16_t DATE;                                                            /*!< Real-Time Clock Date - BCD Format */
  __IO uint16_t YEAR;                                                            /*!< RTCYEAR Register ? BCD Format */
  __IO uint16_t AMINHR;                                                          /*!< RTCMINHR - BCD Format */
  __IO uint16_t ADOWDAY;                                                         /*!< RTCADOWDAY - BCD Format */
} RTC_C_BCD_Type;

/*@}*/ /* end of group RTC_C_BCD */


/******************************************************************************
* SYSCTL Registers
******************************************************************************/
/** @addtogroup SYSCTL MSP432P401R (SYSCTL)
  @{
*/
typedef struct {
  __IO uint32_t REBOOT_CTL;                                                      /*!< Reboot Control Register */
  __IO uint32_t NMI_CTLSTAT;                                                     /*!< NMI Control and Status Register */
  __IO uint32_t WDTRESET_CTL;                                                    /*!< Watchdog Reset Control Register */
  __IO uint32_t PERIHALT_CTL;                                                    /*!< Peripheral Halt Control Register */
  __I  uint32_t SRAM_SIZE;                                                       /*!< SRAM Size Register */
  __IO uint32_t SRAM_BANKEN;                                                     /*!< SRAM Bank Enable Register */
  __IO uint32_t SRAM_BANKRET;                                                    /*!< SRAM Bank Retention Control Register */
       uint32_t RESERVED0;
  __I  uint32_t FLASH_SIZE;                                                      /*!< Flash Size Register */
       uint32_t RESERVED1[3];
  __IO uint32_t DIO_GLTFLT_CTL;                                                  /*!< Digital I/O Glitch Filter Control Register */
       uint32_t RESERVED2[3];
  __IO uint32_t SECDATA_UNLOCK;                                                  /*!< IP Protected Secure Zone Data Access Unlock Register */
} SYSCTL_Type;

typedef struct {
  __IO uint32_t MASTER_UNLOCK;                                                   /*!< Master Unlock Register */
  __IO uint32_t BOOTOVER_REQ[2];                                                 /*!< Boot Override Request Register */
  __IO uint32_t BOOTOVER_ACK;                                                    /*!< Boot Override Acknowledge Register */
  __IO uint32_t RESET_REQ;                                                       /*!< Reset Request Register */
  __IO uint32_t RESET_STATOVER;                                                  /*!< Reset Status and Override Register */
       uint32_t RESERVED7[2];
  __I  uint32_t SYSTEM_STAT;                                                     /*!< System Status Register */
} SYSCTL_Boot_Type;

/*@}*/ /* end of group SYSCTL */


/******************************************************************************
* Timer32 Registers
******************************************************************************/
/** @addtogroup Timer32 MSP432P401R (Timer32)
  @{
*/
typedef struct {
  __IO uint32_t LOAD;                                                            /*!< Timer Load Register */
  __I  uint32_t VALUE;                                                           /*!< Timer Current Value Register */
  __IO uint32_t CONTROL;                                                         /*!< Timer Control Register */
  __O  uint32_t INTCLR;                                                          /*!< Timer Interrupt Clear Register */
  __I  uint32_t RIS;                                                             /*!< Timer Raw Interrupt Status Register */
  __I  uint32_t MIS;                                                             /*!< Timer Interrupt Status Register */
  __IO uint32_t BGLOAD;                                                          /*!< Timer Background Load Register */
} Timer32_Type;

/*@}*/ /* end of group Timer32 */


/******************************************************************************
* Timer_A Registers
******************************************************************************/
/** @addtogroup Timer_A MSP432P401R (Timer_A)
  @{
*/
typedef struct {
  __IO uint16_t CTL;                                                             /*!< TimerAx Control Register */
  __IO uint16_t CCTL[7];                                                         /*!< Timer_A Capture/Compare Control Register */
  __IO uint16_t R;                                                               /*!< TimerA register */
  __IO uint16_t CCR[7];                                                          /*!< Timer_A Capture/Compare  Register */
  __IO uint16_t EX0;                                                             /*!< TimerAx Expansion 0 Register */
       uint16_t RESERVED0[6];
  __I  uint16_t IV;                                                              /*!< TimerAx Interrupt Vector Register */
} Timer_A_Type;

/*@}*/ /* end of group Timer_A */


/******************************************************************************
* TLV Registers
******************************************************************************/
/** @addtogroup TLV MSP432P401R (TLV)
  @{
*/
typedef struct {
  __I  uint32_t TLV_CHECKSUM;                                                    /*!< TLV Checksum */
  __I  uint32_t DEVICE_INFO_TAG;                                                 /*!< Device Info Tag */
  __I  uint32_t DEVICE_INFO_LEN;                                                 /*!< Device Info Length */
  __I  uint32_t DEVICE_ID;                                                       /*!< Device ID */
  __I  uint32_t HWREV;                                                           /*!< HW Revision */
  __I  uint32_t BCREV;                                                           /*!< Boot Code Revision */
  __I  uint32_t ROM_DRVLIB_REV;                                                  /*!< ROM Driver Library Revision */
  __I  uint32_t DIE_REC_TAG;                                                     /*!< Die Record Tag */
  __I  uint32_t DIE_REC_LEN;                                                     /*!< Die Record Length */
  __I  uint32_t DIE_XPOS;                                                        /*!< Die X-Position */
  __I  uint32_t DIE_YPOS;                                                        /*!< Die Y-Position */
  __I  uint32_t WAFER_ID;                                                        /*!< Wafer ID */
  __I  uint32_t LOT_ID;                                                          /*!< Lot ID */
  __I  uint32_t RESERVED0;                                                       /*!< Reserved */
  __I  uint32_t RESERVED1;                                                       /*!< Reserved */
  __I  uint32_t RESERVED2;                                                       /*!< Reserved */
  __I  uint32_t TEST_RESULTS;                                                    /*!< Test Results */
  __I  uint32_t CS_CAL_TAG;                                                      /*!< Clock System Calibration Tag */
  __I  uint32_t CS_CAL_LEN;                                                      /*!< Clock System Calibration Length */
  __I  uint32_t DCOIR_FCAL_RSEL04;                                               /*!< DCO IR mode: Frequency calibration for DCORSEL 0 to 4 */
  __I  uint32_t DCOIR_FCAL_RSEL5;                                                /*!< DCO IR mode: Frequency calibration for DCORSEL 5 */
  __I  uint32_t RESERVED3;                                                       /*!< Reserved */
  __I  uint32_t RESERVED4;                                                       /*!< Reserved */
  __I  uint32_t RESERVED5;                                                       /*!< Reserved */
  __I  uint32_t RESERVED6;                                                       /*!< Reserved */
  __I  uint32_t DCOIR_CONSTK_RSEL04;                                             /*!< DCO IR mode: DCO Constant (K) for DCORSEL 0 to 4 */
  __I  uint32_t DCOIR_CONSTK_RSEL5;                                              /*!< DCO IR mode: DCO Constant (K) for DCORSEL 5 */
  __I  uint32_t DCOER_FCAL_RSEL04;                                               /*!< DCO ER mode: Frequency calibration for DCORSEL 0 to 4 */
  __I  uint32_t DCOER_FCAL_RSEL5;                                                /*!< DCO ER mode: Frequency calibration for DCORSEL 5 */
  __I  uint32_t RESERVED7;                                                       /*!< Reserved */
  __I  uint32_t RESERVED8;                                                       /*!< Reserved */
  __I  uint32_t RESERVED9;                                                       /*!< Reserved */
  __I  uint32_t RESERVED10;                                                      /*!< Reserved */
  __I  uint32_t DCOER_CONSTK_RSEL04;                                             /*!< DCO ER mode: DCO Constant (K) for DCORSEL 0 to 4 */
  __I  uint32_t DCOER_CONSTK_RSEL5;                                              /*!< DCO ER mode: DCO Constant (K) for DCORSEL 5 */
  __I  uint32_t ADC14_CAL_TAG;                                                   /*!< ADC14 Calibration Tag */
  __I  uint32_t ADC14_CAL_LEN;                                                   /*!< ADC14 Calibration Length */
  __I  uint32_t ADC_GAIN_FACTOR;                                                 /*!< ADC Gain Factor */
  __I  uint32_t ADC_OFFSET;                                                      /*!< ADC Offset */
  __I  uint32_t RESERVED11;                                                      /*!< Reserved */
  __I  uint32_t RESERVED12;                                                      /*!< Reserved */
  __I  uint32_t RESERVED13;                                                      /*!< Reserved */
  __I  uint32_t RESERVED14;                                                      /*!< Reserved */
  __I  uint32_t RESERVED15;                                                      /*!< Reserved */
  __I  uint32_t RESERVED16;                                                      /*!< Reserved */
  __I  uint32_t RESERVED17;                                                      /*!< Reserved */
  __I  uint32_t RESERVED18;                                                      /*!< Reserved */
  __I  uint32_t RESERVED19;                                                      /*!< Reserved */
  __I  uint32_t RESERVED20;                                                      /*!< Reserved */
  __I  uint32_t RESERVED21;                                                      /*!< Reserved */
  __I  uint32_t RESERVED22;                                                      /*!< Reserved */
  __I  uint32_t RESERVED23;                                                      /*!< Reserved */
  __I  uint32_t RESERVED24;                                                      /*!< Reserved */
  __I  uint32_t RESERVED25;                                                      /*!< Reserved */
  __I  uint32_t RESERVED26;                                                      /*!< Reserved */
  __I  uint32_t ADC14_REF1P2V_TS30C;                                             /*!< ADC14 1.2V Reference Temp. Sensor 30°C */
  __I  uint32_t ADC14_REF1P2V_TS85C;                                             /*!< ADC14 1.2V Reference Temp. Sensor 85°C */
  __I  uint32_t ADC14_REF1P45V_TS30C;                                            /*!< ADC14 1.45V Reference Temp. Sensor 30°C */
  __I  uint32_t ADC14_REF1P45V_TS85C;                                            /*!< ADC14 1.45V Reference Temp. Sensor 85°C */
  __I  uint32_t ADC14_REF2P5V_TS30C;                                             /*!< ADC14 2.5V Reference Temp. Sensor 30°C */
  __I  uint32_t ADC14_REF2P5V_TS85C;                                             /*!< ADC14 2.5V Reference Temp. Sensor 85°C */
  __I  uint32_t REF_CAL_TAG;                                                     /*!< REF Calibration Tag */
  __I  uint32_t REF_CAL_LEN;                                                     /*!< REF Calibration Length */
  __I  uint32_t REF_1P2V;                                                        /*!< REF 1.2V Reference */
  __I  uint32_t REF_1P45V;                                                       /*!< REF 1.45V Reference */
  __I  uint32_t REF_2P5V;                                                        /*!< REF 2.5V Reference */
  __I  uint32_t FLASH_INFO_TAG;                                                  /*!< Flash Info Tag */
  __I  uint32_t FLASH_INFO_LEN;                                                  /*!< Flash Info Length */
  __I  uint32_t FLASH_MAX_PROG_PULSES;                                           /*!< Flash Maximum Programming Pulses */
  __I  uint32_t FLASH_MAX_ERASE_PULSES;                                          /*!< Flash Maximum Erase Pulses */
  __I  uint32_t RANDOM_NUM_TAG;                                                  /*!< 128-bit Random Number Tag */
  __I  uint32_t RANDOM_NUM_LEN;                                                  /*!< 128-bit Random Number Length */
  __I  uint32_t RANDOM_NUM_1;                                                    /*!< 32-bit Random Number 1 */
  __I  uint32_t RANDOM_NUM_2;                                                    /*!< 32-bit Random Number 2 */
  __I  uint32_t RANDOM_NUM_3;                                                    /*!< 32-bit Random Number 3 */
  __I  uint32_t RANDOM_NUM_4;                                                    /*!< 32-bit Random Number 4 */
  __I  uint32_t BSL_CFG_TAG;                                                     /*!< BSL Configuration Tag */
  __I  uint32_t BSL_CFG_LEN;                                                     /*!< BSL Configuration Length */
  __I  uint32_t BSL_PERIPHIF_SEL;                                                /*!< BSL Peripheral Interface Selection */
  __I  uint32_t BSL_PORTIF_CFG_UART;                                             /*!< BSL Port Interface Configuration for UART */
  __I  uint32_t BSL_PORTIF_CFG_SPI;                                              /*!< BSL Port Interface Configuration for SPI */
  __I  uint32_t BSL_PORTIF_CFG_I2C;                                              /*!< BSL Port Interface Configuration for I2C */
  __I  uint32_t TLV_END;                                                         /*!< TLV End Word */
} TLV_Type;

/*@}*/ /* end of group TLV */


/******************************************************************************
* WDT_A Registers
******************************************************************************/
/** @addtogroup WDT_A MSP432P401R (WDT_A)
  @{
*/
typedef struct {
       uint16_t RESERVED0[6];
  __IO uint16_t CTL;                                                             /*!< Watchdog Timer Control Register */
} WDT_A_Type;

/*@}*/ /* end of group WDT_A */


#if defined ( __CC_ARM )
#pragma no_anon_unions
#endif

/*@}*/ /* end of group MSP432P401R_Peripherals */

/******************************************************************************
* Peripheral declaration                                                      *
******************************************************************************/
/** @addtogroup MSP432P401R_PeripheralDecl MSP432P401R Peripheral Declaration
  @{
*/

#define ADC14                            ((ADC14_Type *) ADC14_BASE)   
#define AES256                           ((AES256_Type *) AES256_BASE) 
#define CAPTIO0                          ((CAPTIO_Type *) CAPTIO0_BASE)
#define CAPTIO1                          ((CAPTIO_Type *) CAPTIO1_BASE)
#define COMP_E0                          ((COMP_E_Type *) COMP_E0_BASE)
#define COMP_E1                          ((COMP_E_Type *) COMP_E1_BASE)
#define CRC32                            ((CRC32_Type *) CRC32_BASE)   
#define CS                               ((CS_Type *) CS_BASE)         
#define PA                               ((DIO_PORT_Interruptable_Type*) (DIO_BASE + 0x0000))
#define PB                               ((DIO_PORT_Interruptable_Type*) (DIO_BASE + 0x0020))
#define PC                               ((DIO_PORT_Interruptable_Type*) (DIO_BASE + 0x0040))
#define PD                               ((DIO_PORT_Interruptable_Type*) (DIO_BASE + 0x0060))
#define PE                               ((DIO_PORT_Interruptable_Type*) (DIO_BASE + 0x0080))
#define PJ                               ((DIO_PORT_Not_Interruptable_Type*) (DIO_BASE + 0x0120))
#define P1                               ((DIO_PORT_Odd_Interruptable_Type*)  (DIO_BASE + 0x0000))
#define P2                               ((DIO_PORT_Even_Interruptable_Type*) (DIO_BASE + 0x0000))
#define P3                               ((DIO_PORT_Odd_Interruptable_Type*)  (DIO_BASE + 0x0020))
#define P4                               ((DIO_PORT_Even_Interruptable_Type*) (DIO_BASE + 0x0020))
#define P5                               ((DIO_PORT_Odd_Interruptable_Type*)  (DIO_BASE + 0x0040))
#define P6                               ((DIO_PORT_Even_Interruptable_Type*) (DIO_BASE + 0x0040))
#define P7                               ((DIO_PORT_Odd_Interruptable_Type*)  (DIO_BASE + 0x0060))
#define P8                               ((DIO_PORT_Even_Interruptable_Type*) (DIO_BASE + 0x0060))
#define P9                               ((DIO_PORT_Odd_Interruptable_Type*)  (DIO_BASE + 0x0080))
#define P10                              ((DIO_PORT_Even_Interruptable_Type*) (DIO_BASE + 0x0080))
#define DMA_Channel                      ((DMA_Channel_Type *) DMA_BASE)
#define DMA_Control                      ((DMA_Control_Type *) (DMA_BASE + 0x1000))
#define EUSCI_A0                         ((EUSCI_A_Type *) EUSCI_A0_BASE)
#define EUSCI_A0_SPI                     ((EUSCI_A_SPI_Type *) EUSCI_A0_SPI_BASE)
#define EUSCI_A1                         ((EUSCI_A_Type *) EUSCI_A1_BASE)
#define EUSCI_A1_SPI                     ((EUSCI_A_SPI_Type *) EUSCI_A1_SPI_BASE)
#define EUSCI_A2                         ((EUSCI_A_Type *) EUSCI_A2_BASE)
#define EUSCI_A2_SPI                     ((EUSCI_A_SPI_Type *) EUSCI_A2_SPI_BASE)
#define EUSCI_A3                         ((EUSCI_A_Type *) EUSCI_A3_BASE)
#define EUSCI_A3_SPI                     ((EUSCI_A_SPI_Type *) EUSCI_A3_SPI_BASE)
#define EUSCI_B0                         ((EUSCI_B_Type *) EUSCI_B0_BASE)
#define EUSCI_B0_SPI                     ((EUSCI_B_SPI_Type *) EUSCI_B0_SPI_BASE)
#define EUSCI_B1                         ((EUSCI_B_Type *) EUSCI_B1_BASE)
#define EUSCI_B1_SPI                     ((EUSCI_B_SPI_Type *) EUSCI_B1_SPI_BASE)
#define EUSCI_B2                         ((EUSCI_B_Type *) EUSCI_B2_BASE)
#define EUSCI_B2_SPI                     ((EUSCI_B_SPI_Type *) EUSCI_B2_SPI_BASE)
#define EUSCI_B3                         ((EUSCI_B_Type *) EUSCI_B3_BASE)
#define EUSCI_B3_SPI                     ((EUSCI_B_SPI_Type *) EUSCI_B3_SPI_BASE)
#define FLCTL                            ((FLCTL_Type *) FLCTL_BASE)   
#define FL_BOOTOVER_MAILBOX              ((FL_BOOTOVER_MAILBOX_Type *) FL_BOOTOVER_MAILBOX_BASE)
#define PCM                              ((PCM_Type *) PCM_BASE)       
#define PMAP                             ((PMAP_COMMON_Type*) PMAP_BASE)
#define P1MAP                            ((PMAP_REGISTER_Type*) (PMAP_BASE + 0x0008))
#define P2MAP                            ((PMAP_REGISTER_Type*) (PMAP_BASE + 0x0010))
#define P3MAP                            ((PMAP_REGISTER_Type*) (PMAP_BASE + 0x0018))
#define P4MAP                            ((PMAP_REGISTER_Type*) (PMAP_BASE + 0x0020))
#define P5MAP                            ((PMAP_REGISTER_Type*) (PMAP_BASE + 0x0028))
#define P6MAP                            ((PMAP_REGISTER_Type*) (PMAP_BASE + 0x0030))
#define P7MAP                            ((PMAP_REGISTER_Type*) (PMAP_BASE + 0x0038))
#define PSS                              ((PSS_Type *) PSS_BASE)       
#define REF_A                            ((REF_A_Type *) REF_A_BASE)   
#define RSTCTL                           ((RSTCTL_Type *) RSTCTL_BASE) 
#define RTC_C                            ((RTC_C_Type *) RTC_C_BASE)   
#define RTC_C_BCD                        ((RTC_C_BCD_Type *) RTC_C_BCD_BASE)
#define SYSCTL                           ((SYSCTL_Type *) SYSCTL_BASE)
#define SYSCTL_Boot                      ((SYSCTL_Boot_Type *) (SYSCTL_BASE + 0x1000))
#define TIMER32_1                        ((Timer32_Type *) TIMER32_BASE)
#define TIMER32_2                        ((Timer32_Type *) (TIMER32_BASE + 0x00020))
#define TIMER_A0                         ((Timer_A_Type *) TIMER_A0_BASE)
#define TIMER_A1                         ((Timer_A_Type *) TIMER_A1_BASE)
#define TIMER_A2                         ((Timer_A_Type *) TIMER_A2_BASE)
#define TIMER_A3                         ((Timer_A_Type *) TIMER_A3_BASE)
#define TLV                              ((TLV_Type *) TLV_BASE)       
#define WDT_A                            ((WDT_A_Type *) WDT_A_BASE)   


/*@}*/ /* end of group MSP432P401R_PeripheralDecl */

/*@}*/ /* end of group MSP432P401R_Definitions */

#endif /* __CMSIS_CONFIG__ */

/******************************************************************************
* Peripheral register control bits                                            *
******************************************************************************/

/******************************************************************************
* ADC14 Bits
******************************************************************************/
/* ADC14_CTL0[SC] Bits */
#define ADC14_CTL0_SC_OFS                        ( 0)                            /*!< ADC14SC Bit Offset */
#define ADC14_CTL0_SC                            ((uint32_t)0x00000001)          /*!< ADC14 start conversion */
/* ADC14_CTL0[ENC] Bits */
#define ADC14_CTL0_ENC_OFS                       ( 1)                            /*!< ADC14ENC Bit Offset */
#define ADC14_CTL0_ENC                           ((uint32_t)0x00000002)          /*!< ADC14 enable conversion */
/* ADC14_CTL0[ON] Bits */
#define ADC14_CTL0_ON_OFS                        ( 4)                            /*!< ADC14ON Bit Offset */
#define ADC14_CTL0_ON                            ((uint32_t)0x00000010)          /*!< ADC14 on */
/* ADC14_CTL0[MSC] Bits */
#define ADC14_CTL0_MSC_OFS                       ( 7)                            /*!< ADC14MSC Bit Offset */
#define ADC14_CTL0_MSC                           ((uint32_t)0x00000080)          /*!< ADC14 multiple sample and conversion */
/* ADC14_CTL0[SHT0] Bits */
#define ADC14_CTL0_SHT0_OFS                      ( 8)                            /*!< ADC14SHT0 Bit Offset */
#define ADC14_CTL0_SHT0_MASK                     ((uint32_t)0x00000F00)          /*!< ADC14SHT0 Bit Mask */
#define ADC14_CTL0_SHT00                         ((uint32_t)0x00000100)          /*!< SHT0 Bit 0 */
#define ADC14_CTL0_SHT01                         ((uint32_t)0x00000200)          /*!< SHT0 Bit 1 */
#define ADC14_CTL0_SHT02                         ((uint32_t)0x00000400)          /*!< SHT0 Bit 2 */
#define ADC14_CTL0_SHT03                         ((uint32_t)0x00000800)          /*!< SHT0 Bit 3 */
#define ADC14_CTL0_SHT0_0                        ((uint32_t)0x00000000)          /*!< 4 */
#define ADC14_CTL0_SHT0_1                        ((uint32_t)0x00000100)          /*!< 8 */
#define ADC14_CTL0_SHT0_2                        ((uint32_t)0x00000200)          /*!< 16 */
#define ADC14_CTL0_SHT0_3                        ((uint32_t)0x00000300)          /*!< 32 */
#define ADC14_CTL0_SHT0_4                        ((uint32_t)0x00000400)          /*!< 64 */
#define ADC14_CTL0_SHT0_5                        ((uint32_t)0x00000500)          /*!< 96 */
#define ADC14_CTL0_SHT0_6                        ((uint32_t)0x00000600)          /*!< 128 */
#define ADC14_CTL0_SHT0_7                        ((uint32_t)0x00000700)          /*!< 192 */
#define ADC14_CTL0_SHT0__4                       ((uint32_t)0x00000000)          /*!< 4 */
#define ADC14_CTL0_SHT0__8                       ((uint32_t)0x00000100)          /*!< 8 */
#define ADC14_CTL0_SHT0__16                      ((uint32_t)0x00000200)          /*!< 16 */
#define ADC14_CTL0_SHT0__32                      ((uint32_t)0x00000300)          /*!< 32 */
#define ADC14_CTL0_SHT0__64                      ((uint32_t)0x00000400)          /*!< 64 */
#define ADC14_CTL0_SHT0__96                      ((uint32_t)0x00000500)          /*!< 96 */
#define ADC14_CTL0_SHT0__128                     ((uint32_t)0x00000600)          /*!< 128 */
#define ADC14_CTL0_SHT0__192                     ((uint32_t)0x00000700)          /*!< 192 */
/* ADC14_CTL0[SHT1] Bits */
#define ADC14_CTL0_SHT1_OFS                      (12)                            /*!< ADC14SHT1 Bit Offset */
#define ADC14_CTL0_SHT1_MASK                     ((uint32_t)0x0000F000)          /*!< ADC14SHT1 Bit Mask */
#define ADC14_CTL0_SHT10                         ((uint32_t)0x00001000)          /*!< SHT1 Bit 0 */
#define ADC14_CTL0_SHT11                         ((uint32_t)0x00002000)          /*!< SHT1 Bit 1 */
#define ADC14_CTL0_SHT12                         ((uint32_t)0x00004000)          /*!< SHT1 Bit 2 */
#define ADC14_CTL0_SHT13                         ((uint32_t)0x00008000)          /*!< SHT1 Bit 3 */
#define ADC14_CTL0_SHT1_0                        ((uint32_t)0x00000000)          /*!< 4 */
#define ADC14_CTL0_SHT1_1                        ((uint32_t)0x00001000)          /*!< 8 */
#define ADC14_CTL0_SHT1_2                        ((uint32_t)0x00002000)          /*!< 16 */
#define ADC14_CTL0_SHT1_3                        ((uint32_t)0x00003000)          /*!< 32 */
#define ADC14_CTL0_SHT1_4                        ((uint32_t)0x00004000)          /*!< 64 */
#define ADC14_CTL0_SHT1_5                        ((uint32_t)0x00005000)          /*!< 96 */
#define ADC14_CTL0_SHT1_6                        ((uint32_t)0x00006000)          /*!< 128 */
#define ADC14_CTL0_SHT1_7                        ((uint32_t)0x00007000)          /*!< 192 */
#define ADC14_CTL0_SHT1__4                       ((uint32_t)0x00000000)          /*!< 4 */
#define ADC14_CTL0_SHT1__8                       ((uint32_t)0x00001000)          /*!< 8 */
#define ADC14_CTL0_SHT1__16                      ((uint32_t)0x00002000)          /*!< 16 */
#define ADC14_CTL0_SHT1__32                      ((uint32_t)0x00003000)          /*!< 32 */
#define ADC14_CTL0_SHT1__64                      ((uint32_t)0x00004000)          /*!< 64 */
#define ADC14_CTL0_SHT1__96                      ((uint32_t)0x00005000)          /*!< 96 */
#define ADC14_CTL0_SHT1__128                     ((uint32_t)0x00006000)          /*!< 128 */
#define ADC14_CTL0_SHT1__192                     ((uint32_t)0x00007000)          /*!< 192 */
/* ADC14_CTL0[BUSY] Bits */
#define ADC14_CTL0_BUSY_OFS                      (16)                            /*!< ADC14BUSY Bit Offset */
#define ADC14_CTL0_BUSY                          ((uint32_t)0x00010000)          /*!< ADC14 busy */
/* ADC14_CTL0[CONSEQ] Bits */
#define ADC14_CTL0_CONSEQ_OFS                    (17)                            /*!< ADC14CONSEQ Bit Offset */
#define ADC14_CTL0_CONSEQ_MASK                   ((uint32_t)0x00060000)          /*!< ADC14CONSEQ Bit Mask */
#define ADC14_CTL0_CONSEQ0                       ((uint32_t)0x00020000)          /*!< CONSEQ Bit 0 */
#define ADC14_CTL0_CONSEQ1                       ((uint32_t)0x00040000)          /*!< CONSEQ Bit 1 */
#define ADC14_CTL0_CONSEQ_0                      ((uint32_t)0x00000000)          /*!< Single-channel, single-conversion */
#define ADC14_CTL0_CONSEQ_1                      ((uint32_t)0x00020000)          /*!< Sequence-of-channels */
#define ADC14_CTL0_CONSEQ_2                      ((uint32_t)0x00040000)          /*!< Repeat-single-channel */
#define ADC14_CTL0_CONSEQ_3                      ((uint32_t)0x00060000)          /*!< Repeat-sequence-of-channels */
/* ADC14_CTL0[SSEL] Bits */
#define ADC14_CTL0_SSEL_OFS                      (19)                            /*!< ADC14SSEL Bit Offset */
#define ADC14_CTL0_SSEL_MASK                     ((uint32_t)0x00380000)          /*!< ADC14SSEL Bit Mask */
#define ADC14_CTL0_SSEL0                         ((uint32_t)0x00080000)          /*!< SSEL Bit 0 */
#define ADC14_CTL0_SSEL1                         ((uint32_t)0x00100000)          /*!< SSEL Bit 1 */
#define ADC14_CTL0_SSEL2                         ((uint32_t)0x00200000)          /*!< SSEL Bit 2 */
#define ADC14_CTL0_SSEL_0                        ((uint32_t)0x00000000)          /*!< MODCLK */
#define ADC14_CTL0_SSEL_1                        ((uint32_t)0x00080000)          /*!< SYSCLK */
#define ADC14_CTL0_SSEL_2                        ((uint32_t)0x00100000)          /*!< ACLK */
#define ADC14_CTL0_SSEL_3                        ((uint32_t)0x00180000)          /*!< MCLK */
#define ADC14_CTL0_SSEL_4                        ((uint32_t)0x00200000)          /*!< SMCLK */
#define ADC14_CTL0_SSEL_5                        ((uint32_t)0x00280000)          /*!< HSMCLK */
#define ADC14_CTL0_SSEL__MODCLK                  ((uint32_t)0x00000000)          /*!< MODCLK */
#define ADC14_CTL0_SSEL__SYSCLK                  ((uint32_t)0x00080000)          /*!< SYSCLK */
#define ADC14_CTL0_SSEL__ACLK                    ((uint32_t)0x00100000)          /*!< ACLK */
#define ADC14_CTL0_SSEL__MCLK                    ((uint32_t)0x00180000)          /*!< MCLK */
#define ADC14_CTL0_SSEL__SMCLK                   ((uint32_t)0x00200000)          /*!< SMCLK */
#define ADC14_CTL0_SSEL__HSMCLK                  ((uint32_t)0x00280000)          /*!< HSMCLK */
/* ADC14_CTL0[DIV] Bits */
#define ADC14_CTL0_DIV_OFS                       (22)                            /*!< ADC14DIV Bit Offset */
#define ADC14_CTL0_DIV_MASK                      ((uint32_t)0x01C00000)          /*!< ADC14DIV Bit Mask */
#define ADC14_CTL0_DIV0                          ((uint32_t)0x00400000)          /*!< DIV Bit 0 */
#define ADC14_CTL0_DIV1                          ((uint32_t)0x00800000)          /*!< DIV Bit 1 */
#define ADC14_CTL0_DIV2                          ((uint32_t)0x01000000)          /*!< DIV Bit 2 */
#define ADC14_CTL0_DIV_0                         ((uint32_t)0x00000000)          /*!< /1 */
#define ADC14_CTL0_DIV_1                         ((uint32_t)0x00400000)          /*!< /2 */
#define ADC14_CTL0_DIV_2                         ((uint32_t)0x00800000)          /*!< /3 */
#define ADC14_CTL0_DIV_3                         ((uint32_t)0x00C00000)          /*!< /4 */
#define ADC14_CTL0_DIV_4                         ((uint32_t)0x01000000)          /*!< /5 */
#define ADC14_CTL0_DIV_5                         ((uint32_t)0x01400000)          /*!< /6 */
#define ADC14_CTL0_DIV_6                         ((uint32_t)0x01800000)          /*!< /7 */
#define ADC14_CTL0_DIV_7                         ((uint32_t)0x01C00000)          /*!< /8 */
#define ADC14_CTL0_DIV__1                        ((uint32_t)0x00000000)          /*!< /1 */
#define ADC14_CTL0_DIV__2                        ((uint32_t)0x00400000)          /*!< /2 */
#define ADC14_CTL0_DIV__3                        ((uint32_t)0x00800000)          /*!< /3 */
#define ADC14_CTL0_DIV__4                        ((uint32_t)0x00C00000)          /*!< /4 */
#define ADC14_CTL0_DIV__5                        ((uint32_t)0x01000000)          /*!< /5 */
#define ADC14_CTL0_DIV__6                        ((uint32_t)0x01400000)          /*!< /6 */
#define ADC14_CTL0_DIV__7                        ((uint32_t)0x01800000)          /*!< /7 */
#define ADC14_CTL0_DIV__8                        ((uint32_t)0x01C00000)          /*!< /8 */
/* ADC14_CTL0[ISSH] Bits */
#define ADC14_CTL0_ISSH_OFS                      (25)                            /*!< ADC14ISSH Bit Offset */
#define ADC14_CTL0_ISSH                          ((uint32_t)0x02000000)          /*!< ADC14 invert signal sample-and-hold */
/* ADC14_CTL0[SHP] Bits */
#define ADC14_CTL0_SHP_OFS                       (26)                            /*!< ADC14SHP Bit Offset */
#define ADC14_CTL0_SHP                           ((uint32_t)0x04000000)          /*!< ADC14 sample-and-hold pulse-mode select */
/* ADC14_CTL0[SHS] Bits */
#define ADC14_CTL0_SHS_OFS                       (27)                            /*!< ADC14SHS Bit Offset */
#define ADC14_CTL0_SHS_MASK                      ((uint32_t)0x38000000)          /*!< ADC14SHS Bit Mask */
#define ADC14_CTL0_SHS0                          ((uint32_t)0x08000000)          /*!< SHS Bit 0 */
#define ADC14_CTL0_SHS1                          ((uint32_t)0x10000000)          /*!< SHS Bit 1 */
#define ADC14_CTL0_SHS2                          ((uint32_t)0x20000000)          /*!< SHS Bit 2 */
#define ADC14_CTL0_SHS_0                         ((uint32_t)0x00000000)          /*!< ADC14SC bit */
#define ADC14_CTL0_SHS_1                         ((uint32_t)0x08000000)          /*!< See device-specific data sheet for source */
#define ADC14_CTL0_SHS_2                         ((uint32_t)0x10000000)          /*!< See device-specific data sheet for source */
#define ADC14_CTL0_SHS_3                         ((uint32_t)0x18000000)          /*!< See device-specific data sheet for source */
#define ADC14_CTL0_SHS_4                         ((uint32_t)0x20000000)          /*!< See device-specific data sheet for source */
#define ADC14_CTL0_SHS_5                         ((uint32_t)0x28000000)          /*!< See device-specific data sheet for source */
#define ADC14_CTL0_SHS_6                         ((uint32_t)0x30000000)          /*!< See device-specific data sheet for source */
#define ADC14_CTL0_SHS_7                         ((uint32_t)0x38000000)          /*!< See device-specific data sheet for source */
/* ADC14_CTL0[PDIV] Bits */
#define ADC14_CTL0_PDIV_OFS                      (30)                            /*!< ADC14PDIV Bit Offset */
#define ADC14_CTL0_PDIV_MASK                     ((uint32_t)0xC0000000)          /*!< ADC14PDIV Bit Mask */
#define ADC14_CTL0_PDIV0                         ((uint32_t)0x40000000)          /*!< PDIV Bit 0 */
#define ADC14_CTL0_PDIV1                         ((uint32_t)0x80000000)          /*!< PDIV Bit 1 */
#define ADC14_CTL0_PDIV_0                        ((uint32_t)0x00000000)          /*!< Predivide by 1 */
#define ADC14_CTL0_PDIV_1                        ((uint32_t)0x40000000)          /*!< Predivide by 4 */
#define ADC14_CTL0_PDIV_2                        ((uint32_t)0x80000000)          /*!< Predivide by 32 */
#define ADC14_CTL0_PDIV_3                        ((uint32_t)0xC0000000)          /*!< Predivide by 64 */
#define ADC14_CTL0_PDIV__1                       ((uint32_t)0x00000000)          /*!< Predivide by 1 */
#define ADC14_CTL0_PDIV__4                       ((uint32_t)0x40000000)          /*!< Predivide by 4 */
#define ADC14_CTL0_PDIV__32                      ((uint32_t)0x80000000)          /*!< Predivide by 32 */
#define ADC14_CTL0_PDIV__64                      ((uint32_t)0xC0000000)          /*!< Predivide by 64 */
/* ADC14_CTL1[PWRMD] Bits */
#define ADC14_CTL1_PWRMD_OFS                     ( 0)                            /*!< ADC14PWRMD Bit Offset */
#define ADC14_CTL1_PWRMD_MASK                    ((uint32_t)0x00000003)          /*!< ADC14PWRMD Bit Mask */
#define ADC14_CTL1_PWRMD0                        ((uint32_t)0x00000001)          /*!< PWRMD Bit 0 */
#define ADC14_CTL1_PWRMD1                        ((uint32_t)0x00000002)          /*!< PWRMD Bit 1 */
#define ADC14_CTL1_PWRMD_0                       ((uint32_t)0x00000000)          /*!< Regular power mode for use with any resolution setting. Sample rate can be up  */
                                                                                 /* to 1 Msps. */
#define ADC14_CTL1_PWRMD_2                       ((uint32_t)0x00000002)          /*!< Low-power mode for 12-bit, 10-bit, and 8-bit resolution settings. Sample rate  */
                                                                                 /* must not exceed 200 ksps. */
/* ADC14_CTL1[REFBURST] Bits */
#define ADC14_CTL1_REFBURST_OFS                  ( 2)                            /*!< ADC14REFBURST Bit Offset */
#define ADC14_CTL1_REFBURST                      ((uint32_t)0x00000004)          /*!< ADC14 reference buffer burst */
/* ADC14_CTL1[DF] Bits */
#define ADC14_CTL1_DF_OFS                        ( 3)                            /*!< ADC14DF Bit Offset */
#define ADC14_CTL1_DF                            ((uint32_t)0x00000008)          /*!< ADC14 data read-back format */
/* ADC14_CTL1[RES] Bits */
#define ADC14_CTL1_RES_OFS                       ( 4)                            /*!< ADC14RES Bit Offset */
#define ADC14_CTL1_RES_MASK                      ((uint32_t)0x00000030)          /*!< ADC14RES Bit Mask */
#define ADC14_CTL1_RES0                          ((uint32_t)0x00000010)          /*!< RES Bit 0 */
#define ADC14_CTL1_RES1                          ((uint32_t)0x00000020)          /*!< RES Bit 1 */
#define ADC14_CTL1_RES_0                         ((uint32_t)0x00000000)          /*!< 8 bit (9 clock cycle conversion time) */
#define ADC14_CTL1_RES_1                         ((uint32_t)0x00000010)          /*!< 10 bit (11 clock cycle conversion time) */
#define ADC14_CTL1_RES_2                         ((uint32_t)0x00000020)          /*!< 12 bit (14 clock cycle conversion time) */
#define ADC14_CTL1_RES_3                         ((uint32_t)0x00000030)          /*!< 14 bit (16 clock cycle conversion time) */
#define ADC14_CTL1_RES__8BIT                     ((uint32_t)0x00000000)          /*!< 8 bit (9 clock cycle conversion time) */
#define ADC14_CTL1_RES__10BIT                    ((uint32_t)0x00000010)          /*!< 10 bit (11 clock cycle conversion time) */
#define ADC14_CTL1_RES__12BIT                    ((uint32_t)0x00000020)          /*!< 12 bit (14 clock cycle conversion time) */
#define ADC14_CTL1_RES__14BIT                    ((uint32_t)0x00000030)          /*!< 14 bit (16 clock cycle conversion time) */
/* ADC14_CTL1[CSTARTADD] Bits */
#define ADC14_CTL1_CSTARTADD_OFS                 (16)                            /*!< ADC14CSTARTADD Bit Offset */
#define ADC14_CTL1_CSTARTADD_MASK                ((uint32_t)0x001F0000)          /*!< ADC14CSTARTADD Bit Mask */
/* ADC14_CTL1[BATMAP] Bits */
#define ADC14_CTL1_BATMAP_OFS                    (22)                            /*!< ADC14BATMAP Bit Offset */
#define ADC14_CTL1_BATMAP                        ((uint32_t)0x00400000)          /*!< Controls 1/2 AVCC ADC input channel selection */
/* ADC14_CTL1[TCMAP] Bits */
#define ADC14_CTL1_TCMAP_OFS                     (23)                            /*!< ADC14TCMAP Bit Offset */
#define ADC14_CTL1_TCMAP                         ((uint32_t)0x00800000)          /*!< Controls temperature sensor ADC input channel selection */
/* ADC14_CTL1[CH0MAP] Bits */
#define ADC14_CTL1_CH0MAP_OFS                    (24)                            /*!< ADC14CH0MAP Bit Offset */
#define ADC14_CTL1_CH0MAP                        ((uint32_t)0x01000000)          /*!< Controls internal channel 0 selection to ADC input channel MAX-2 */
/* ADC14_CTL1[CH1MAP] Bits */
#define ADC14_CTL1_CH1MAP_OFS                    (25)                            /*!< ADC14CH1MAP Bit Offset */
#define ADC14_CTL1_CH1MAP                        ((uint32_t)0x02000000)          /*!< Controls internal channel 1 selection to ADC input channel MAX-3 */
/* ADC14_CTL1[CH2MAP] Bits */
#define ADC14_CTL1_CH2MAP_OFS                    (26)                            /*!< ADC14CH2MAP Bit Offset */
#define ADC14_CTL1_CH2MAP                        ((uint32_t)0x04000000)          /*!< Controls internal channel 2 selection to ADC input channel MAX-4 */
/* ADC14_CTL1[CH3MAP] Bits */
#define ADC14_CTL1_CH3MAP_OFS                    (27)                            /*!< ADC14CH3MAP Bit Offset */
#define ADC14_CTL1_CH3MAP                        ((uint32_t)0x08000000)          /*!< Controls internal channel 3 selection to ADC input channel MAX-5 */
/* ADC14_LO0[LO0] Bits */
#define ADC14_LO0_LO0_OFS                        ( 0)                            /*!< ADC14LO0 Bit Offset */
#define ADC14_LO0_LO0_MASK                       ((uint32_t)0x0000FFFF)          /*!< ADC14LO0 Bit Mask */
/* ADC14_HI0[HI0] Bits */
#define ADC14_HI0_HI0_OFS                        ( 0)                            /*!< ADC14HI0 Bit Offset */
#define ADC14_HI0_HI0_MASK                       ((uint32_t)0x0000FFFF)          /*!< ADC14HI0 Bit Mask */
/* ADC14_LO1[LO1] Bits */
#define ADC14_LO1_LO1_OFS                        ( 0)                            /*!< ADC14LO1 Bit Offset */
#define ADC14_LO1_LO1_MASK                       ((uint32_t)0x0000FFFF)          /*!< ADC14LO1 Bit Mask */
/* ADC14_HI1[HI1] Bits */
#define ADC14_HI1_HI1_OFS                        ( 0)                            /*!< ADC14HI1 Bit Offset */
#define ADC14_HI1_HI1_MASK                       ((uint32_t)0x0000FFFF)          /*!< ADC14HI1 Bit Mask */
/* ADC14_MCTLN[INCH] Bits */
#define ADC14_MCTLN_INCH_OFS                     ( 0)                            /*!< ADC14INCH Bit Offset */
#define ADC14_MCTLN_INCH_MASK                    ((uint32_t)0x0000001F)          /*!< ADC14INCH Bit Mask */
#define ADC14_MCTLN_INCH0                        ((uint32_t)0x00000001)          /*!< INCH Bit 0 */
#define ADC14_MCTLN_INCH1                        ((uint32_t)0x00000002)          /*!< INCH Bit 1 */
#define ADC14_MCTLN_INCH2                        ((uint32_t)0x00000004)          /*!< INCH Bit 2 */
#define ADC14_MCTLN_INCH3                        ((uint32_t)0x00000008)          /*!< INCH Bit 3 */
#define ADC14_MCTLN_INCH4                        ((uint32_t)0x00000010)          /*!< INCH Bit 4 */
#define ADC14_MCTLN_INCH_0                       ((uint32_t)0x00000000)          /*!< If ADC14DIF = 0: A0; If ADC14DIF = 1: Ain+ = A0, Ain- = A1 */
#define ADC14_MCTLN_INCH_1                       ((uint32_t)0x00000001)          /*!< If ADC14DIF = 0: A1; If ADC14DIF = 1: Ain+ = A0, Ain- = A1 */
#define ADC14_MCTLN_INCH_2                       ((uint32_t)0x00000002)          /*!< If ADC14DIF = 0: A2; If ADC14DIF = 1: Ain+ = A2, Ain- = A3 */
#define ADC14_MCTLN_INCH_3                       ((uint32_t)0x00000003)          /*!< If ADC14DIF = 0: A3; If ADC14DIF = 1: Ain+ = A2, Ain- = A3 */
#define ADC14_MCTLN_INCH_4                       ((uint32_t)0x00000004)          /*!< If ADC14DIF = 0: A4; If ADC14DIF = 1: Ain+ = A4, Ain- = A5 */
#define ADC14_MCTLN_INCH_5                       ((uint32_t)0x00000005)          /*!< If ADC14DIF = 0: A5; If ADC14DIF = 1: Ain+ = A4, Ain- = A5 */
#define ADC14_MCTLN_INCH_6                       ((uint32_t)0x00000006)          /*!< If ADC14DIF = 0: A6; If ADC14DIF = 1: Ain+ = A6, Ain- = A7 */
#define ADC14_MCTLN_INCH_7                       ((uint32_t)0x00000007)          /*!< If ADC14DIF = 0: A7; If ADC14DIF = 1: Ain+ = A6, Ain- = A7 */
#define ADC14_MCTLN_INCH_8                       ((uint32_t)0x00000008)          /*!< If ADC14DIF = 0: A8; If ADC14DIF = 1: Ain+ = A8, Ain- = A9 */
#define ADC14_MCTLN_INCH_9                       ((uint32_t)0x00000009)          /*!< If ADC14DIF = 0: A9; If ADC14DIF = 1: Ain+ = A8, Ain- = A9 */
#define ADC14_MCTLN_INCH_10                      ((uint32_t)0x0000000A)          /*!< If ADC14DIF = 0: A10; If ADC14DIF = 1: Ain+ = A10, Ain- = A11 */
#define ADC14_MCTLN_INCH_11                      ((uint32_t)0x0000000B)          /*!< If ADC14DIF = 0: A11; If ADC14DIF = 1: Ain+ = A10, Ain- = A11 */
#define ADC14_MCTLN_INCH_12                      ((uint32_t)0x0000000C)          /*!< If ADC14DIF = 0: A12; If ADC14DIF = 1: Ain+ = A12, Ain- = A13 */
#define ADC14_MCTLN_INCH_13                      ((uint32_t)0x0000000D)          /*!< If ADC14DIF = 0: A13; If ADC14DIF = 1: Ain+ = A12, Ain- = A13 */
#define ADC14_MCTLN_INCH_14                      ((uint32_t)0x0000000E)          /*!< If ADC14DIF = 0: A14; If ADC14DIF = 1: Ain+ = A14, Ain- = A15 */
#define ADC14_MCTLN_INCH_15                      ((uint32_t)0x0000000F)          /*!< If ADC14DIF = 0: A15; If ADC14DIF = 1: Ain+ = A14, Ain- = A15 */
#define ADC14_MCTLN_INCH_16                      ((uint32_t)0x00000010)          /*!< If ADC14DIF = 0: A16; If ADC14DIF = 1: Ain+ = A16, Ain- = A17 */
#define ADC14_MCTLN_INCH_17                      ((uint32_t)0x00000011)          /*!< If ADC14DIF = 0: A17; If ADC14DIF = 1: Ain+ = A16, Ain- = A17 */
#define ADC14_MCTLN_INCH_18                      ((uint32_t)0x00000012)          /*!< If ADC14DIF = 0: A18; If ADC14DIF = 1: Ain+ = A18, Ain- = A19 */
#define ADC14_MCTLN_INCH_19                      ((uint32_t)0x00000013)          /*!< If ADC14DIF = 0: A19; If ADC14DIF = 1: Ain+ = A18, Ain- = A19 */
#define ADC14_MCTLN_INCH_20                      ((uint32_t)0x00000014)          /*!< If ADC14DIF = 0: A20; If ADC14DIF = 1: Ain+ = A20, Ain- = A21 */
#define ADC14_MCTLN_INCH_21                      ((uint32_t)0x00000015)          /*!< If ADC14DIF = 0: A21; If ADC14DIF = 1: Ain+ = A20, Ain- = A21 */
#define ADC14_MCTLN_INCH_22                      ((uint32_t)0x00000016)          /*!< If ADC14DIF = 0: A22; If ADC14DIF = 1: Ain+ = A22, Ain- = A23 */
#define ADC14_MCTLN_INCH_23                      ((uint32_t)0x00000017)          /*!< If ADC14DIF = 0: A23; If ADC14DIF = 1: Ain+ = A22, Ain- = A23 */
#define ADC14_MCTLN_INCH_24                      ((uint32_t)0x00000018)          /*!< If ADC14DIF = 0: A24; If ADC14DIF = 1: Ain+ = A24, Ain- = A25 */
#define ADC14_MCTLN_INCH_25                      ((uint32_t)0x00000019)          /*!< If ADC14DIF = 0: A25; If ADC14DIF = 1: Ain+ = A24, Ain- = A25 */
#define ADC14_MCTLN_INCH_26                      ((uint32_t)0x0000001A)          /*!< If ADC14DIF = 0: A26; If ADC14DIF = 1: Ain+ = A26, Ain- = A27 */
#define ADC14_MCTLN_INCH_27                      ((uint32_t)0x0000001B)          /*!< If ADC14DIF = 0: A27; If ADC14DIF = 1: Ain+ = A26, Ain- = A27 */
#define ADC14_MCTLN_INCH_28                      ((uint32_t)0x0000001C)          /*!< If ADC14DIF = 0: A28; If ADC14DIF = 1: Ain+ = A28, Ain- = A29 */
#define ADC14_MCTLN_INCH_29                      ((uint32_t)0x0000001D)          /*!< If ADC14DIF = 0: A29; If ADC14DIF = 1: Ain+ = A28, Ain- = A29 */
#define ADC14_MCTLN_INCH_30                      ((uint32_t)0x0000001E)          /*!< If ADC14DIF = 0: A30; If ADC14DIF = 1: Ain+ = A30, Ain- = A31 */
#define ADC14_MCTLN_INCH_31                      ((uint32_t)0x0000001F)          /*!< If ADC14DIF = 0: A31; If ADC14DIF = 1: Ain+ = A30, Ain- = A31 */
/* ADC14_MCTLN[EOS] Bits */
#define ADC14_MCTLN_EOS_OFS                      ( 7)                            /*!< ADC14EOS Bit Offset */
#define ADC14_MCTLN_EOS                          ((uint32_t)0x00000080)          /*!< End of sequence */
/* ADC14_MCTLN[VRSEL] Bits */
#define ADC14_MCTLN_VRSEL_OFS                    ( 8)                            /*!< ADC14VRSEL Bit Offset */
#define ADC14_MCTLN_VRSEL_MASK                   ((uint32_t)0x00000F00)          /*!< ADC14VRSEL Bit Mask */
#define ADC14_MCTLN_VRSEL0                       ((uint32_t)0x00000100)          /*!< VRSEL Bit 0 */
#define ADC14_MCTLN_VRSEL1                       ((uint32_t)0x00000200)          /*!< VRSEL Bit 1 */
#define ADC14_MCTLN_VRSEL2                       ((uint32_t)0x00000400)          /*!< VRSEL Bit 2 */
#define ADC14_MCTLN_VRSEL3                       ((uint32_t)0x00000800)          /*!< VRSEL Bit 3 */
#define ADC14_MCTLN_VRSEL_0                      ((uint32_t)0x00000000)          /*!< V(R+) = AVCC, V(R-) = AVSS */
#define ADC14_MCTLN_VRSEL_1                      ((uint32_t)0x00000100)          /*!< V(R+) = VREF buffered, V(R-) = AVSS */
#define ADC14_MCTLN_VRSEL_14                     ((uint32_t)0x00000E00)          /*!< V(R+) = VeREF+, V(R-) = VeREF- */
#define ADC14_MCTLN_VRSEL_15                     ((uint32_t)0x00000F00)          /*!< V(R+) = VeREF+ buffered, V(R-) = VeREF */
/* ADC14_MCTLN[DIF] Bits */
#define ADC14_MCTLN_DIF_OFS                      (13)                            /*!< ADC14DIF Bit Offset */
#define ADC14_MCTLN_DIF                          ((uint32_t)0x00002000)          /*!< Differential mode */
/* ADC14_MCTLN[WINC] Bits */
#define ADC14_MCTLN_WINC_OFS                     (14)                            /*!< ADC14WINC Bit Offset */
#define ADC14_MCTLN_WINC                         ((uint32_t)0x00004000)          /*!< Comparator window enable */
/* ADC14_MCTLN[WINCTH] Bits */
#define ADC14_MCTLN_WINCTH_OFS                   (15)                            /*!< ADC14WINCTH Bit Offset */
#define ADC14_MCTLN_WINCTH                       ((uint32_t)0x00008000)          /*!< Window comparator threshold register selection */
/* ADC14_MEMN[CONVRES] Bits */
#define ADC14_MEMN_CONVRES_OFS                   ( 0)                            /*!< Conversion_Results Bit Offset */
#define ADC14_MEMN_CONVRES_MASK                  ((uint32_t)0x0000FFFF)          /*!< Conversion_Results Bit Mask */
/* ADC14_IER0[IE0] Bits */
#define ADC14_IER0_IE0_OFS                       ( 0)                            /*!< ADC14IE0 Bit Offset */
#define ADC14_IER0_IE0                           ((uint32_t)0x00000001)          /*!< Interrupt enable */
/* ADC14_IER0[IE1] Bits */
#define ADC14_IER0_IE1_OFS                       ( 1)                            /*!< ADC14IE1 Bit Offset */
#define ADC14_IER0_IE1                           ((uint32_t)0x00000002)          /*!< Interrupt enable */
/* ADC14_IER0[IE2] Bits */
#define ADC14_IER0_IE2_OFS                       ( 2)                            /*!< ADC14IE2 Bit Offset */
#define ADC14_IER0_IE2                           ((uint32_t)0x00000004)          /*!< Interrupt enable */
/* ADC14_IER0[IE3] Bits */
#define ADC14_IER0_IE3_OFS                       ( 3)                            /*!< ADC14IE3 Bit Offset */
#define ADC14_IER0_IE3                           ((uint32_t)0x00000008)          /*!< Interrupt enable */
/* ADC14_IER0[IE4] Bits */
#define ADC14_IER0_IE4_OFS                       ( 4)                            /*!< ADC14IE4 Bit Offset */
#define ADC14_IER0_IE4                           ((uint32_t)0x00000010)          /*!< Interrupt enable */
/* ADC14_IER0[IE5] Bits */
#define ADC14_IER0_IE5_OFS                       ( 5)                            /*!< ADC14IE5 Bit Offset */
#define ADC14_IER0_IE5                           ((uint32_t)0x00000020)          /*!< Interrupt enable */
/* ADC14_IER0[IE6] Bits */
#define ADC14_IER0_IE6_OFS                       ( 6)                            /*!< ADC14IE6 Bit Offset */
#define ADC14_IER0_IE6                           ((uint32_t)0x00000040)          /*!< Interrupt enable */
/* ADC14_IER0[IE7] Bits */
#define ADC14_IER0_IE7_OFS                       ( 7)                            /*!< ADC14IE7 Bit Offset */
#define ADC14_IER0_IE7                           ((uint32_t)0x00000080)          /*!< Interrupt enable */
/* ADC14_IER0[IE8] Bits */
#define ADC14_IER0_IE8_OFS                       ( 8)                            /*!< ADC14IE8 Bit Offset */
#define ADC14_IER0_IE8                           ((uint32_t)0x00000100)          /*!< Interrupt enable */
/* ADC14_IER0[IE9] Bits */
#define ADC14_IER0_IE9_OFS                       ( 9)                            /*!< ADC14IE9 Bit Offset */
#define ADC14_IER0_IE9                           ((uint32_t)0x00000200)          /*!< Interrupt enable */
/* ADC14_IER0[IE10] Bits */
#define ADC14_IER0_IE10_OFS                      (10)                            /*!< ADC14IE10 Bit Offset */
#define ADC14_IER0_IE10                          ((uint32_t)0x00000400)          /*!< Interrupt enable */
/* ADC14_IER0[IE11] Bits */
#define ADC14_IER0_IE11_OFS                      (11)                            /*!< ADC14IE11 Bit Offset */
#define ADC14_IER0_IE11                          ((uint32_t)0x00000800)          /*!< Interrupt enable */
/* ADC14_IER0[IE12] Bits */
#define ADC14_IER0_IE12_OFS                      (12)                            /*!< ADC14IE12 Bit Offset */
#define ADC14_IER0_IE12                          ((uint32_t)0x00001000)          /*!< Interrupt enable */
/* ADC14_IER0[IE13] Bits */
#define ADC14_IER0_IE13_OFS                      (13)                            /*!< ADC14IE13 Bit Offset */
#define ADC14_IER0_IE13                          ((uint32_t)0x00002000)          /*!< Interrupt enable */
/* ADC14_IER0[IE14] Bits */
#define ADC14_IER0_IE14_OFS                      (14)                            /*!< ADC14IE14 Bit Offset */
#define ADC14_IER0_IE14                          ((uint32_t)0x00004000)          /*!< Interrupt enable */
/* ADC14_IER0[IE15] Bits */
#define ADC14_IER0_IE15_OFS                      (15)                            /*!< ADC14IE15 Bit Offset */
#define ADC14_IER0_IE15                          ((uint32_t)0x00008000)          /*!< Interrupt enable */
/* ADC14_IER0[IE16] Bits */
#define ADC14_IER0_IE16_OFS                      (16)                            /*!< ADC14IE16 Bit Offset */
#define ADC14_IER0_IE16                          ((uint32_t)0x00010000)          /*!< Interrupt enable */
/* ADC14_IER0[IE17] Bits */
#define ADC14_IER0_IE17_OFS                      (17)                            /*!< ADC14IE17 Bit Offset */
#define ADC14_IER0_IE17                          ((uint32_t)0x00020000)          /*!< Interrupt enable */
/* ADC14_IER0[IE19] Bits */
#define ADC14_IER0_IE19_OFS                      (19)                            /*!< ADC14IE19 Bit Offset */
#define ADC14_IER0_IE19                          ((uint32_t)0x00080000)          /*!< Interrupt enable */
/* ADC14_IER0[IE18] Bits */
#define ADC14_IER0_IE18_OFS                      (18)                            /*!< ADC14IE18 Bit Offset */
#define ADC14_IER0_IE18                          ((uint32_t)0x00040000)          /*!< Interrupt enable */
/* ADC14_IER0[IE20] Bits */
#define ADC14_IER0_IE20_OFS                      (20)                            /*!< ADC14IE20 Bit Offset */
#define ADC14_IER0_IE20                          ((uint32_t)0x00100000)          /*!< Interrupt enable */
/* ADC14_IER0[IE21] Bits */
#define ADC14_IER0_IE21_OFS                      (21)                            /*!< ADC14IE21 Bit Offset */
#define ADC14_IER0_IE21                          ((uint32_t)0x00200000)          /*!< Interrupt enable */
/* ADC14_IER0[IE22] Bits */
#define ADC14_IER0_IE22_OFS                      (22)                            /*!< ADC14IE22 Bit Offset */
#define ADC14_IER0_IE22                          ((uint32_t)0x00400000)          /*!< Interrupt enable */
/* ADC14_IER0[IE23] Bits */
#define ADC14_IER0_IE23_OFS                      (23)                            /*!< ADC14IE23 Bit Offset */
#define ADC14_IER0_IE23                          ((uint32_t)0x00800000)          /*!< Interrupt enable */
/* ADC14_IER0[IE24] Bits */
#define ADC14_IER0_IE24_OFS                      (24)                            /*!< ADC14IE24 Bit Offset */
#define ADC14_IER0_IE24                          ((uint32_t)0x01000000)          /*!< Interrupt enable */
/* ADC14_IER0[IE25] Bits */
#define ADC14_IER0_IE25_OFS                      (25)                            /*!< ADC14IE25 Bit Offset */
#define ADC14_IER0_IE25                          ((uint32_t)0x02000000)          /*!< Interrupt enable */
/* ADC14_IER0[IE26] Bits */
#define ADC14_IER0_IE26_OFS                      (26)                            /*!< ADC14IE26 Bit Offset */
#define ADC14_IER0_IE26                          ((uint32_t)0x04000000)          /*!< Interrupt enable */
/* ADC14_IER0[IE27] Bits */
#define ADC14_IER0_IE27_OFS                      (27)                            /*!< ADC14IE27 Bit Offset */
#define ADC14_IER0_IE27                          ((uint32_t)0x08000000)          /*!< Interrupt enable */
/* ADC14_IER0[IE28] Bits */
#define ADC14_IER0_IE28_OFS                      (28)                            /*!< ADC14IE28 Bit Offset */
#define ADC14_IER0_IE28                          ((uint32_t)0x10000000)          /*!< Interrupt enable */
/* ADC14_IER0[IE29] Bits */
#define ADC14_IER0_IE29_OFS                      (29)                            /*!< ADC14IE29 Bit Offset */
#define ADC14_IER0_IE29                          ((uint32_t)0x20000000)          /*!< Interrupt enable */
/* ADC14_IER0[IE30] Bits */
#define ADC14_IER0_IE30_OFS                      (30)                            /*!< ADC14IE30 Bit Offset */
#define ADC14_IER0_IE30                          ((uint32_t)0x40000000)          /*!< Interrupt enable */
/* ADC14_IER0[IE31] Bits */
#define ADC14_IER0_IE31_OFS                      (31)                            /*!< ADC14IE31 Bit Offset */
#define ADC14_IER0_IE31                          ((uint32_t)0x80000000)          /*!< Interrupt enable */
/* ADC14_IER1[INIE] Bits */
#define ADC14_IER1_INIE_OFS                      ( 1)                            /*!< ADC14INIE Bit Offset */
#define ADC14_IER1_INIE                          ((uint32_t)0x00000002)          /*!< Interrupt enable for ADC14MEMx within comparator window */
/* ADC14_IER1[LOIE] Bits */
#define ADC14_IER1_LOIE_OFS                      ( 2)                            /*!< ADC14LOIE Bit Offset */
#define ADC14_IER1_LOIE                          ((uint32_t)0x00000004)          /*!< Interrupt enable for ADC14MEMx below comparator window */
/* ADC14_IER1[HIIE] Bits */
#define ADC14_IER1_HIIE_OFS                      ( 3)                            /*!< ADC14HIIE Bit Offset */
#define ADC14_IER1_HIIE                          ((uint32_t)0x00000008)          /*!< Interrupt enable for ADC14MEMx above comparator window */
/* ADC14_IER1[OVIE] Bits */
#define ADC14_IER1_OVIE_OFS                      ( 4)                            /*!< ADC14OVIE Bit Offset */
#define ADC14_IER1_OVIE                          ((uint32_t)0x00000010)          /*!< ADC14MEMx overflow-interrupt enable */
/* ADC14_IER1[TOVIE] Bits */
#define ADC14_IER1_TOVIE_OFS                     ( 5)                            /*!< ADC14TOVIE Bit Offset */
#define ADC14_IER1_TOVIE                         ((uint32_t)0x00000020)          /*!< ADC14 conversion-time-overflow interrupt enable */
/* ADC14_IER1[RDYIE] Bits */
#define ADC14_IER1_RDYIE_OFS                     ( 6)                            /*!< ADC14RDYIE Bit Offset */
#define ADC14_IER1_RDYIE                         ((uint32_t)0x00000040)          /*!< ADC14 local buffered reference ready interrupt enable */
/* ADC14_IFGR0[IFG0] Bits */
#define ADC14_IFGR0_IFG0_OFS                     ( 0)                            /*!< ADC14IFG0 Bit Offset */
#define ADC14_IFGR0_IFG0                         ((uint32_t)0x00000001)          /*!< ADC14MEM0 interrupt flag */
/* ADC14_IFGR0[IFG1] Bits */
#define ADC14_IFGR0_IFG1_OFS                     ( 1)                            /*!< ADC14IFG1 Bit Offset */
#define ADC14_IFGR0_IFG1                         ((uint32_t)0x00000002)          /*!< ADC14MEM1 interrupt flag */
/* ADC14_IFGR0[IFG2] Bits */
#define ADC14_IFGR0_IFG2_OFS                     ( 2)                            /*!< ADC14IFG2 Bit Offset */
#define ADC14_IFGR0_IFG2                         ((uint32_t)0x00000004)          /*!< ADC14MEM2 interrupt flag */
/* ADC14_IFGR0[IFG3] Bits */
#define ADC14_IFGR0_IFG3_OFS                     ( 3)                            /*!< ADC14IFG3 Bit Offset */
#define ADC14_IFGR0_IFG3                         ((uint32_t)0x00000008)          /*!< ADC14MEM3 interrupt flag */
/* ADC14_IFGR0[IFG4] Bits */
#define ADC14_IFGR0_IFG4_OFS                     ( 4)                            /*!< ADC14IFG4 Bit Offset */
#define ADC14_IFGR0_IFG4                         ((uint32_t)0x00000010)          /*!< ADC14MEM4 interrupt flag */
/* ADC14_IFGR0[IFG5] Bits */
#define ADC14_IFGR0_IFG5_OFS                     ( 5)                            /*!< ADC14IFG5 Bit Offset */
#define ADC14_IFGR0_IFG5                         ((uint32_t)0x00000020)          /*!< ADC14MEM5 interrupt flag */
/* ADC14_IFGR0[IFG6] Bits */
#define ADC14_IFGR0_IFG6_OFS                     ( 6)                            /*!< ADC14IFG6 Bit Offset */
#define ADC14_IFGR0_IFG6                         ((uint32_t)0x00000040)          /*!< ADC14MEM6 interrupt flag */
/* ADC14_IFGR0[IFG7] Bits */
#define ADC14_IFGR0_IFG7_OFS                     ( 7)                            /*!< ADC14IFG7 Bit Offset */
#define ADC14_IFGR0_IFG7                         ((uint32_t)0x00000080)          /*!< ADC14MEM7 interrupt flag */
/* ADC14_IFGR0[IFG8] Bits */
#define ADC14_IFGR0_IFG8_OFS                     ( 8)                            /*!< ADC14IFG8 Bit Offset */
#define ADC14_IFGR0_IFG8                         ((uint32_t)0x00000100)          /*!< ADC14MEM8 interrupt flag */
/* ADC14_IFGR0[IFG9] Bits */
#define ADC14_IFGR0_IFG9_OFS                     ( 9)                            /*!< ADC14IFG9 Bit Offset */
#define ADC14_IFGR0_IFG9                         ((uint32_t)0x00000200)          /*!< ADC14MEM9 interrupt flag */
/* ADC14_IFGR0[IFG10] Bits */
#define ADC14_IFGR0_IFG10_OFS                    (10)                            /*!< ADC14IFG10 Bit Offset */
#define ADC14_IFGR0_IFG10                        ((uint32_t)0x00000400)          /*!< ADC14MEM10 interrupt flag */
/* ADC14_IFGR0[IFG11] Bits */
#define ADC14_IFGR0_IFG11_OFS                    (11)                            /*!< ADC14IFG11 Bit Offset */
#define ADC14_IFGR0_IFG11                        ((uint32_t)0x00000800)          /*!< ADC14MEM11 interrupt flag */
/* ADC14_IFGR0[IFG12] Bits */
#define ADC14_IFGR0_IFG12_OFS                    (12)                            /*!< ADC14IFG12 Bit Offset */
#define ADC14_IFGR0_IFG12                        ((uint32_t)0x00001000)          /*!< ADC14MEM12 interrupt flag */
/* ADC14_IFGR0[IFG13] Bits */
#define ADC14_IFGR0_IFG13_OFS                    (13)                            /*!< ADC14IFG13 Bit Offset */
#define ADC14_IFGR0_IFG13                        ((uint32_t)0x00002000)          /*!< ADC14MEM13 interrupt flag */
/* ADC14_IFGR0[IFG14] Bits */
#define ADC14_IFGR0_IFG14_OFS                    (14)                            /*!< ADC14IFG14 Bit Offset */
#define ADC14_IFGR0_IFG14                        ((uint32_t)0x00004000)          /*!< ADC14MEM14 interrupt flag */
/* ADC14_IFGR0[IFG15] Bits */
#define ADC14_IFGR0_IFG15_OFS                    (15)                            /*!< ADC14IFG15 Bit Offset */
#define ADC14_IFGR0_IFG15                        ((uint32_t)0x00008000)          /*!< ADC14MEM15 interrupt flag */
/* ADC14_IFGR0[IFG16] Bits */
#define ADC14_IFGR0_IFG16_OFS                    (16)                            /*!< ADC14IFG16 Bit Offset */
#define ADC14_IFGR0_IFG16                        ((uint32_t)0x00010000)          /*!< ADC14MEM16 interrupt flag */
/* ADC14_IFGR0[IFG17] Bits */
#define ADC14_IFGR0_IFG17_OFS                    (17)                            /*!< ADC14IFG17 Bit Offset */
#define ADC14_IFGR0_IFG17                        ((uint32_t)0x00020000)          /*!< ADC14MEM17 interrupt flag */
/* ADC14_IFGR0[IFG18] Bits */
#define ADC14_IFGR0_IFG18_OFS                    (18)                            /*!< ADC14IFG18 Bit Offset */
#define ADC14_IFGR0_IFG18                        ((uint32_t)0x00040000)          /*!< ADC14MEM18 interrupt flag */
/* ADC14_IFGR0[IFG19] Bits */
#define ADC14_IFGR0_IFG19_OFS                    (19)                            /*!< ADC14IFG19 Bit Offset */
#define ADC14_IFGR0_IFG19                        ((uint32_t)0x00080000)          /*!< ADC14MEM19 interrupt flag */
/* ADC14_IFGR0[IFG20] Bits */
#define ADC14_IFGR0_IFG20_OFS                    (20)                            /*!< ADC14IFG20 Bit Offset */
#define ADC14_IFGR0_IFG20                        ((uint32_t)0x00100000)          /*!< ADC14MEM20 interrupt flag */
/* ADC14_IFGR0[IFG21] Bits */
#define ADC14_IFGR0_IFG21_OFS                    (21)                            /*!< ADC14IFG21 Bit Offset */
#define ADC14_IFGR0_IFG21                        ((uint32_t)0x00200000)          /*!< ADC14MEM21 interrupt flag */
/* ADC14_IFGR0[IFG22] Bits */
#define ADC14_IFGR0_IFG22_OFS                    (22)                            /*!< ADC14IFG22 Bit Offset */
#define ADC14_IFGR0_IFG22                        ((uint32_t)0x00400000)          /*!< ADC14MEM22 interrupt flag */
/* ADC14_IFGR0[IFG23] Bits */
#define ADC14_IFGR0_IFG23_OFS                    (23)                            /*!< ADC14IFG23 Bit Offset */
#define ADC14_IFGR0_IFG23                        ((uint32_t)0x00800000)          /*!< ADC14MEM23 interrupt flag */
/* ADC14_IFGR0[IFG24] Bits */
#define ADC14_IFGR0_IFG24_OFS                    (24)                            /*!< ADC14IFG24 Bit Offset */
#define ADC14_IFGR0_IFG24                        ((uint32_t)0x01000000)          /*!< ADC14MEM24 interrupt flag */
/* ADC14_IFGR0[IFG25] Bits */
#define ADC14_IFGR0_IFG25_OFS                    (25)                            /*!< ADC14IFG25 Bit Offset */
#define ADC14_IFGR0_IFG25                        ((uint32_t)0x02000000)          /*!< ADC14MEM25 interrupt flag */
/* ADC14_IFGR0[IFG26] Bits */
#define ADC14_IFGR0_IFG26_OFS                    (26)                            /*!< ADC14IFG26 Bit Offset */
#define ADC14_IFGR0_IFG26                        ((uint32_t)0x04000000)          /*!< ADC14MEM26 interrupt flag */
/* ADC14_IFGR0[IFG27] Bits */
#define ADC14_IFGR0_IFG27_OFS                    (27)                            /*!< ADC14IFG27 Bit Offset */
#define ADC14_IFGR0_IFG27                        ((uint32_t)0x08000000)          /*!< ADC14MEM27 interrupt flag */
/* ADC14_IFGR0[IFG28] Bits */
#define ADC14_IFGR0_IFG28_OFS                    (28)                            /*!< ADC14IFG28 Bit Offset */
#define ADC14_IFGR0_IFG28                        ((uint32_t)0x10000000)          /*!< ADC14MEM28 interrupt flag */
/* ADC14_IFGR0[IFG29] Bits */
#define ADC14_IFGR0_IFG29_OFS                    (29)                            /*!< ADC14IFG29 Bit Offset */
#define ADC14_IFGR0_IFG29                        ((uint32_t)0x20000000)          /*!< ADC14MEM29 interrupt flag */
/* ADC14_IFGR0[IFG30] Bits */
#define ADC14_IFGR0_IFG30_OFS                    (30)                            /*!< ADC14IFG30 Bit Offset */
#define ADC14_IFGR0_IFG30                        ((uint32_t)0x40000000)          /*!< ADC14MEM30 interrupt flag */
/* ADC14_IFGR0[IFG31] Bits */
#define ADC14_IFGR0_IFG31_OFS                    (31)                            /*!< ADC14IFG31 Bit Offset */
#define ADC14_IFGR0_IFG31                        ((uint32_t)0x80000000)          /*!< ADC14MEM31 interrupt flag */
/* ADC14_IFGR1[INIFG] Bits */
#define ADC14_IFGR1_INIFG_OFS                    ( 1)                            /*!< ADC14INIFG Bit Offset */
#define ADC14_IFGR1_INIFG                        ((uint32_t)0x00000002)          /*!< Interrupt flag for ADC14MEMx within comparator window */
/* ADC14_IFGR1[LOIFG] Bits */
#define ADC14_IFGR1_LOIFG_OFS                    ( 2)                            /*!< ADC14LOIFG Bit Offset */
#define ADC14_IFGR1_LOIFG                        ((uint32_t)0x00000004)          /*!< Interrupt flag for ADC14MEMx below comparator window */
/* ADC14_IFGR1[HIIFG] Bits */
#define ADC14_IFGR1_HIIFG_OFS                    ( 3)                            /*!< ADC14HIIFG Bit Offset */
#define ADC14_IFGR1_HIIFG                        ((uint32_t)0x00000008)          /*!< Interrupt flag for ADC14MEMx above comparator window */
/* ADC14_IFGR1[OVIFG] Bits */
#define ADC14_IFGR1_OVIFG_OFS                    ( 4)                            /*!< ADC14OVIFG Bit Offset */
#define ADC14_IFGR1_OVIFG                        ((uint32_t)0x00000010)          /*!< ADC14MEMx overflow interrupt flag */
/* ADC14_IFGR1[TOVIFG] Bits */
#define ADC14_IFGR1_TOVIFG_OFS                   ( 5)                            /*!< ADC14TOVIFG Bit Offset */
#define ADC14_IFGR1_TOVIFG                       ((uint32_t)0x00000020)          /*!< ADC14 conversion time overflow interrupt flag */
/* ADC14_IFGR1[RDYIFG] Bits */
#define ADC14_IFGR1_RDYIFG_OFS                   ( 6)                            /*!< ADC14RDYIFG Bit Offset */
#define ADC14_IFGR1_RDYIFG                       ((uint32_t)0x00000040)          /*!< ADC14 local buffered reference ready interrupt flag */
/* ADC14_CLRIFGR0[CLRIFG0] Bits */
#define ADC14_CLRIFGR0_CLRIFG0_OFS               ( 0)                            /*!< CLRADC14IFG0 Bit Offset */
#define ADC14_CLRIFGR0_CLRIFG0                   ((uint32_t)0x00000001)          /*!< clear ADC14IFG0 */
/* ADC14_CLRIFGR0[CLRIFG1] Bits */
#define ADC14_CLRIFGR0_CLRIFG1_OFS               ( 1)                            /*!< CLRADC14IFG1 Bit Offset */
#define ADC14_CLRIFGR0_CLRIFG1                   ((uint32_t)0x00000002)          /*!< clear ADC14IFG1 */
/* ADC14_CLRIFGR0[CLRIFG2] Bits */
#define ADC14_CLRIFGR0_CLRIFG2_OFS               ( 2)                            /*!< CLRADC14IFG2 Bit Offset */
#define ADC14_CLRIFGR0_CLRIFG2                   ((uint32_t)0x00000004)          /*!< clear ADC14IFG2 */
/* ADC14_CLRIFGR0[CLRIFG3] Bits */
#define ADC14_CLRIFGR0_CLRIFG3_OFS               ( 3)                            /*!< CLRADC14IFG3 Bit Offset */
#define ADC14_CLRIFGR0_CLRIFG3                   ((uint32_t)0x00000008)          /*!< clear ADC14IFG3 */
/* ADC14_CLRIFGR0[CLRIFG4] Bits */
#define ADC14_CLRIFGR0_CLRIFG4_OFS               ( 4)                            /*!< CLRADC14IFG4 Bit Offset */
#define ADC14_CLRIFGR0_CLRIFG4                   ((uint32_t)0x00000010)          /*!< clear ADC14IFG4 */
/* ADC14_CLRIFGR0[CLRIFG5] Bits */
#define ADC14_CLRIFGR0_CLRIFG5_OFS               ( 5)                            /*!< CLRADC14IFG5 Bit Offset */
#define ADC14_CLRIFGR0_CLRIFG5                   ((uint32_t)0x00000020)          /*!< clear ADC14IFG5 */
/* ADC14_CLRIFGR0[CLRIFG6] Bits */
#define ADC14_CLRIFGR0_CLRIFG6_OFS               ( 6)                            /*!< CLRADC14IFG6 Bit Offset */
#define ADC14_CLRIFGR0_CLRIFG6                   ((uint32_t)0x00000040)          /*!< clear ADC14IFG6 */
/* ADC14_CLRIFGR0[CLRIFG7] Bits */
#define ADC14_CLRIFGR0_CLRIFG7_OFS               ( 7)                            /*!< CLRADC14IFG7 Bit Offset */
#define ADC14_CLRIFGR0_CLRIFG7                   ((uint32_t)0x00000080)          /*!< clear ADC14IFG7 */
/* ADC14_CLRIFGR0[CLRIFG8] Bits */
#define ADC14_CLRIFGR0_CLRIFG8_OFS               ( 8)                            /*!< CLRADC14IFG8 Bit Offset */
#define ADC14_CLRIFGR0_CLRIFG8                   ((uint32_t)0x00000100)          /*!< clear ADC14IFG8 */
/* ADC14_CLRIFGR0[CLRIFG9] Bits */
#define ADC14_CLRIFGR0_CLRIFG9_OFS               ( 9)                            /*!< CLRADC14IFG9 Bit Offset */
#define ADC14_CLRIFGR0_CLRIFG9                   ((uint32_t)0x00000200)          /*!< clear ADC14IFG9 */
/* ADC14_CLRIFGR0[CLRIFG10] Bits */
#define ADC14_CLRIFGR0_CLRIFG10_OFS              (10)                            /*!< CLRADC14IFG10 Bit Offset */
#define ADC14_CLRIFGR0_CLRIFG10                  ((uint32_t)0x00000400)          /*!< clear ADC14IFG10 */
/* ADC14_CLRIFGR0[CLRIFG11] Bits */
#define ADC14_CLRIFGR0_CLRIFG11_OFS              (11)                            /*!< CLRADC14IFG11 Bit Offset */
#define ADC14_CLRIFGR0_CLRIFG11                  ((uint32_t)0x00000800)          /*!< clear ADC14IFG11 */
/* ADC14_CLRIFGR0[CLRIFG12] Bits */
#define ADC14_CLRIFGR0_CLRIFG12_OFS              (12)                            /*!< CLRADC14IFG12 Bit Offset */
#define ADC14_CLRIFGR0_CLRIFG12                  ((uint32_t)0x00001000)          /*!< clear ADC14IFG12 */
/* ADC14_CLRIFGR0[CLRIFG13] Bits */
#define ADC14_CLRIFGR0_CLRIFG13_OFS              (13)                            /*!< CLRADC14IFG13 Bit Offset */
#define ADC14_CLRIFGR0_CLRIFG13                  ((uint32_t)0x00002000)          /*!< clear ADC14IFG13 */
/* ADC14_CLRIFGR0[CLRIFG14] Bits */
#define ADC14_CLRIFGR0_CLRIFG14_OFS              (14)                            /*!< CLRADC14IFG14 Bit Offset */
#define ADC14_CLRIFGR0_CLRIFG14                  ((uint32_t)0x00004000)          /*!< clear ADC14IFG14 */
/* ADC14_CLRIFGR0[CLRIFG15] Bits */
#define ADC14_CLRIFGR0_CLRIFG15_OFS              (15)                            /*!< CLRADC14IFG15 Bit Offset */
#define ADC14_CLRIFGR0_CLRIFG15                  ((uint32_t)0x00008000)          /*!< clear ADC14IFG15 */
/* ADC14_CLRIFGR0[CLRIFG16] Bits */
#define ADC14_CLRIFGR0_CLRIFG16_OFS              (16)                            /*!< CLRADC14IFG16 Bit Offset */
#define ADC14_CLRIFGR0_CLRIFG16                  ((uint32_t)0x00010000)          /*!< clear ADC14IFG16 */
/* ADC14_CLRIFGR0[CLRIFG17] Bits */
#define ADC14_CLRIFGR0_CLRIFG17_OFS              (17)                            /*!< CLRADC14IFG17 Bit Offset */
#define ADC14_CLRIFGR0_CLRIFG17                  ((uint32_t)0x00020000)          /*!< clear ADC14IFG17 */
/* ADC14_CLRIFGR0[CLRIFG18] Bits */
#define ADC14_CLRIFGR0_CLRIFG18_OFS              (18)                            /*!< CLRADC14IFG18 Bit Offset */
#define ADC14_CLRIFGR0_CLRIFG18                  ((uint32_t)0x00040000)          /*!< clear ADC14IFG18 */
/* ADC14_CLRIFGR0[CLRIFG19] Bits */
#define ADC14_CLRIFGR0_CLRIFG19_OFS              (19)                            /*!< CLRADC14IFG19 Bit Offset */
#define ADC14_CLRIFGR0_CLRIFG19                  ((uint32_t)0x00080000)          /*!< clear ADC14IFG19 */
/* ADC14_CLRIFGR0[CLRIFG20] Bits */
#define ADC14_CLRIFGR0_CLRIFG20_OFS              (20)                            /*!< CLRADC14IFG20 Bit Offset */
#define ADC14_CLRIFGR0_CLRIFG20                  ((uint32_t)0x00100000)          /*!< clear ADC14IFG20 */
/* ADC14_CLRIFGR0[CLRIFG21] Bits */
#define ADC14_CLRIFGR0_CLRIFG21_OFS              (21)                            /*!< CLRADC14IFG21 Bit Offset */
#define ADC14_CLRIFGR0_CLRIFG21                  ((uint32_t)0x00200000)          /*!< clear ADC14IFG21 */
/* ADC14_CLRIFGR0[CLRIFG22] Bits */
#define ADC14_CLRIFGR0_CLRIFG22_OFS              (22)                            /*!< CLRADC14IFG22 Bit Offset */
#define ADC14_CLRIFGR0_CLRIFG22                  ((uint32_t)0x00400000)          /*!< clear ADC14IFG22 */
/* ADC14_CLRIFGR0[CLRIFG23] Bits */
#define ADC14_CLRIFGR0_CLRIFG23_OFS              (23)                            /*!< CLRADC14IFG23 Bit Offset */
#define ADC14_CLRIFGR0_CLRIFG23                  ((uint32_t)0x00800000)          /*!< clear ADC14IFG23 */
/* ADC14_CLRIFGR0[CLRIFG24] Bits */
#define ADC14_CLRIFGR0_CLRIFG24_OFS              (24)                            /*!< CLRADC14IFG24 Bit Offset */
#define ADC14_CLRIFGR0_CLRIFG24                  ((uint32_t)0x01000000)          /*!< clear ADC14IFG24 */
/* ADC14_CLRIFGR0[CLRIFG25] Bits */
#define ADC14_CLRIFGR0_CLRIFG25_OFS              (25)                            /*!< CLRADC14IFG25 Bit Offset */
#define ADC14_CLRIFGR0_CLRIFG25                  ((uint32_t)0x02000000)          /*!< clear ADC14IFG25 */
/* ADC14_CLRIFGR0[CLRIFG26] Bits */
#define ADC14_CLRIFGR0_CLRIFG26_OFS              (26)                            /*!< CLRADC14IFG26 Bit Offset */
#define ADC14_CLRIFGR0_CLRIFG26                  ((uint32_t)0x04000000)          /*!< clear ADC14IFG26 */
/* ADC14_CLRIFGR0[CLRIFG27] Bits */
#define ADC14_CLRIFGR0_CLRIFG27_OFS              (27)                            /*!< CLRADC14IFG27 Bit Offset */
#define ADC14_CLRIFGR0_CLRIFG27                  ((uint32_t)0x08000000)          /*!< clear ADC14IFG27 */
/* ADC14_CLRIFGR0[CLRIFG28] Bits */
#define ADC14_CLRIFGR0_CLRIFG28_OFS              (28)                            /*!< CLRADC14IFG28 Bit Offset */
#define ADC14_CLRIFGR0_CLRIFG28                  ((uint32_t)0x10000000)          /*!< clear ADC14IFG28 */
/* ADC14_CLRIFGR0[CLRIFG29] Bits */
#define ADC14_CLRIFGR0_CLRIFG29_OFS              (29)                            /*!< CLRADC14IFG29 Bit Offset */
#define ADC14_CLRIFGR0_CLRIFG29                  ((uint32_t)0x20000000)          /*!< clear ADC14IFG29 */
/* ADC14_CLRIFGR0[CLRIFG30] Bits */
#define ADC14_CLRIFGR0_CLRIFG30_OFS              (30)                            /*!< CLRADC14IFG30 Bit Offset */
#define ADC14_CLRIFGR0_CLRIFG30                  ((uint32_t)0x40000000)          /*!< clear ADC14IFG30 */
/* ADC14_CLRIFGR0[CLRIFG31] Bits */
#define ADC14_CLRIFGR0_CLRIFG31_OFS              (31)                            /*!< CLRADC14IFG31 Bit Offset */
#define ADC14_CLRIFGR0_CLRIFG31                  ((uint32_t)0x80000000)          /*!< clear ADC14IFG31 */
/* ADC14_CLRIFGR1[CLRINIFG] Bits */
#define ADC14_CLRIFGR1_CLRINIFG_OFS              ( 1)                            /*!< CLRADC14INIFG Bit Offset */
#define ADC14_CLRIFGR1_CLRINIFG                  ((uint32_t)0x00000002)          /*!< clear ADC14INIFG */
/* ADC14_CLRIFGR1[CLRLOIFG] Bits */
#define ADC14_CLRIFGR1_CLRLOIFG_OFS              ( 2)                            /*!< CLRADC14LOIFG Bit Offset */
#define ADC14_CLRIFGR1_CLRLOIFG                  ((uint32_t)0x00000004)          /*!< clear ADC14LOIFG */
/* ADC14_CLRIFGR1[CLRHIIFG] Bits */
#define ADC14_CLRIFGR1_CLRHIIFG_OFS              ( 3)                            /*!< CLRADC14HIIFG Bit Offset */
#define ADC14_CLRIFGR1_CLRHIIFG                  ((uint32_t)0x00000008)          /*!< clear ADC14HIIFG */
/* ADC14_CLRIFGR1[CLROVIFG] Bits */
#define ADC14_CLRIFGR1_CLROVIFG_OFS              ( 4)                            /*!< CLRADC14OVIFG Bit Offset */
#define ADC14_CLRIFGR1_CLROVIFG                  ((uint32_t)0x00000010)          /*!< clear ADC14OVIFG */
/* ADC14_CLRIFGR1[CLRTOVIFG] Bits */
#define ADC14_CLRIFGR1_CLRTOVIFG_OFS             ( 5)                            /*!< CLRADC14TOVIFG Bit Offset */
#define ADC14_CLRIFGR1_CLRTOVIFG                 ((uint32_t)0x00000020)          /*!< clear ADC14TOVIFG */
/* ADC14_CLRIFGR1[CLRRDYIFG] Bits */
#define ADC14_CLRIFGR1_CLRRDYIFG_OFS             ( 6)                            /*!< CLRADC14RDYIFG Bit Offset */
#define ADC14_CLRIFGR1_CLRRDYIFG                 ((uint32_t)0x00000040)          /*!< clear ADC14RDYIFG */


/******************************************************************************
* AES256 Bits
******************************************************************************/
/* AES256_CTL0[OP] Bits */
#define AES256_CTL0_OP_OFS                       ( 0)                            /*!< AESOPx Bit Offset */
#define AES256_CTL0_OP_MASK                      ((uint16_t)0x0003)              /*!< AESOPx Bit Mask */
#define AES256_CTL0_OP0                          ((uint16_t)0x0001)              /*!< OP Bit 0 */
#define AES256_CTL0_OP1                          ((uint16_t)0x0002)              /*!< OP Bit 1 */
#define AES256_CTL0_OP_0                         ((uint16_t)0x0000)              /*!< Encryption */
#define AES256_CTL0_OP_1                         ((uint16_t)0x0001)              /*!< Decryption. The provided key is the same key used for encryption */
#define AES256_CTL0_OP_2                         ((uint16_t)0x0002)              /*!< Generate first round key required for decryption */
#define AES256_CTL0_OP_3                         ((uint16_t)0x0003)              /*!< Decryption. The provided key is the first round key required for decryption */
/* AES256_CTL0[KL] Bits */
#define AES256_CTL0_KL_OFS                       ( 2)                            /*!< AESKLx Bit Offset */
#define AES256_CTL0_KL_MASK                      ((uint16_t)0x000C)              /*!< AESKLx Bit Mask */
#define AES256_CTL0_KL0                          ((uint16_t)0x0004)              /*!< KL Bit 0 */
#define AES256_CTL0_KL1                          ((uint16_t)0x0008)              /*!< KL Bit 1 */
#define AES256_CTL0_KL_0                         ((uint16_t)0x0000)              /*!< AES128. The key size is 128 bit */
#define AES256_CTL0_KL_1                         ((uint16_t)0x0004)              /*!< AES192. The key size is 192 bit. */
#define AES256_CTL0_KL_2                         ((uint16_t)0x0008)              /*!< AES256. The key size is 256 bit */
#define AES256_CTL0_KL__128BIT                   ((uint16_t)0x0000)              /*!< AES128. The key size is 128 bit */
#define AES256_CTL0_KL__192BIT                   ((uint16_t)0x0004)              /*!< AES192. The key size is 192 bit. */
#define AES256_CTL0_KL__256BIT                   ((uint16_t)0x0008)              /*!< AES256. The key size is 256 bit */
/* AES256_CTL0[CM] Bits */
#define AES256_CTL0_CM_OFS                       ( 5)                            /*!< AESCMx Bit Offset */
#define AES256_CTL0_CM_MASK                      ((uint16_t)0x0060)              /*!< AESCMx Bit Mask */
#define AES256_CTL0_CM0                          ((uint16_t)0x0020)              /*!< CM Bit 0 */
#define AES256_CTL0_CM1                          ((uint16_t)0x0040)              /*!< CM Bit 1 */
#define AES256_CTL0_CM_0                         ((uint16_t)0x0000)              /*!< ECB */
#define AES256_CTL0_CM_1                         ((uint16_t)0x0020)              /*!< CBC */
#define AES256_CTL0_CM_2                         ((uint16_t)0x0040)              /*!< OFB */
#define AES256_CTL0_CM_3                         ((uint16_t)0x0060)              /*!< CFB */
#define AES256_CTL0_CM__ECB                      ((uint16_t)0x0000)              /*!< ECB */
#define AES256_CTL0_CM__CBC                      ((uint16_t)0x0020)              /*!< CBC */
#define AES256_CTL0_CM__OFB                      ((uint16_t)0x0040)              /*!< OFB */
#define AES256_CTL0_CM__CFB                      ((uint16_t)0x0060)              /*!< CFB */
/* AES256_CTL0[SWRST] Bits */
#define AES256_CTL0_SWRST_OFS                    ( 7)                            /*!< AESSWRST Bit Offset */
#define AES256_CTL0_SWRST                        ((uint16_t)0x0080)              /*!< AES software reset */
/* AES256_CTL0[RDYIFG] Bits */
#define AES256_CTL0_RDYIFG_OFS                   ( 8)                            /*!< AESRDYIFG Bit Offset */
#define AES256_CTL0_RDYIFG                       ((uint16_t)0x0100)              /*!< AES ready interrupt flag */
/* AES256_CTL0[ERRFG] Bits */
#define AES256_CTL0_ERRFG_OFS                    (11)                            /*!< AESERRFG Bit Offset */
#define AES256_CTL0_ERRFG                        ((uint16_t)0x0800)              /*!< AES error flag */
/* AES256_CTL0[RDYIE] Bits */
#define AES256_CTL0_RDYIE_OFS                    (12)                            /*!< AESRDYIE Bit Offset */
#define AES256_CTL0_RDYIE                        ((uint16_t)0x1000)              /*!< AES ready interrupt enable */
/* AES256_CTL0[CMEN] Bits */
#define AES256_CTL0_CMEN_OFS                     (15)                            /*!< AESCMEN Bit Offset */
#define AES256_CTL0_CMEN                         ((uint16_t)0x8000)              /*!< AES cipher mode enable */
/* AES256_CTL1[BLKCNT] Bits */
#define AES256_CTL1_BLKCNT_OFS                   ( 0)                            /*!< AESBLKCNTx Bit Offset */
#define AES256_CTL1_BLKCNT_MASK                  ((uint16_t)0x00FF)              /*!< AESBLKCNTx Bit Mask */
#define AES256_CTL1_BLKCNT0                      ((uint16_t)0x0001)              /*!< BLKCNT Bit 0 */
#define AES256_CTL1_BLKCNT1                      ((uint16_t)0x0002)              /*!< BLKCNT Bit 1 */
#define AES256_CTL1_BLKCNT2                      ((uint16_t)0x0004)              /*!< BLKCNT Bit 2 */
#define AES256_CTL1_BLKCNT3                      ((uint16_t)0x0008)              /*!< BLKCNT Bit 3 */
#define AES256_CTL1_BLKCNT4                      ((uint16_t)0x0010)              /*!< BLKCNT Bit 4 */
#define AES256_CTL1_BLKCNT5                      ((uint16_t)0x0020)              /*!< BLKCNT Bit 5 */
#define AES256_CTL1_BLKCNT6                      ((uint16_t)0x0040)              /*!< BLKCNT Bit 6 */
#define AES256_CTL1_BLKCNT7                      ((uint16_t)0x0080)              /*!< BLKCNT Bit 7 */
/* AES256_STAT[BUSY] Bits */
#define AES256_STAT_BUSY_OFS                     ( 0)                            /*!< AESBUSY Bit Offset */
#define AES256_STAT_BUSY                         ((uint16_t)0x0001)              /*!< AES accelerator module busy */
/* AES256_STAT[KEYWR] Bits */
#define AES256_STAT_KEYWR_OFS                    ( 1)                            /*!< AESKEYWR Bit Offset */
#define AES256_STAT_KEYWR                        ((uint16_t)0x0002)              /*!< All 16 bytes written to AESAKEY */
/* AES256_STAT[DINWR] Bits */
#define AES256_STAT_DINWR_OFS                    ( 2)                            /*!< AESDINWR Bit Offset */
#define AES256_STAT_DINWR                        ((uint16_t)0x0004)              /*!< All 16 bytes written to AESADIN, AESAXDIN or AESAXIN */
/* AES256_STAT[DOUTRD] Bits */
#define AES256_STAT_DOUTRD_OFS                   ( 3)                            /*!< AESDOUTRD Bit Offset */
#define AES256_STAT_DOUTRD                       ((uint16_t)0x0008)              /*!< All 16 bytes read from AESADOUT */
/* AES256_STAT[KEYCNT] Bits */
#define AES256_STAT_KEYCNT_OFS                   ( 4)                            /*!< AESKEYCNTx Bit Offset */
#define AES256_STAT_KEYCNT_MASK                  ((uint16_t)0x00F0)              /*!< AESKEYCNTx Bit Mask */
#define AES256_STAT_KEYCNT0                      ((uint16_t)0x0010)              /*!< KEYCNT Bit 0 */
#define AES256_STAT_KEYCNT1                      ((uint16_t)0x0020)              /*!< KEYCNT Bit 1 */
#define AES256_STAT_KEYCNT2                      ((uint16_t)0x0040)              /*!< KEYCNT Bit 2 */
#define AES256_STAT_KEYCNT3                      ((uint16_t)0x0080)              /*!< KEYCNT Bit 3 */
/* AES256_STAT[DINCNT] Bits */
#define AES256_STAT_DINCNT_OFS                   ( 8)                            /*!< AESDINCNTx Bit Offset */
#define AES256_STAT_DINCNT_MASK                  ((uint16_t)0x0F00)              /*!< AESDINCNTx Bit Mask */
#define AES256_STAT_DINCNT0                      ((uint16_t)0x0100)              /*!< DINCNT Bit 0 */
#define AES256_STAT_DINCNT1                      ((uint16_t)0x0200)              /*!< DINCNT Bit 1 */
#define AES256_STAT_DINCNT2                      ((uint16_t)0x0400)              /*!< DINCNT Bit 2 */
#define AES256_STAT_DINCNT3                      ((uint16_t)0x0800)              /*!< DINCNT Bit 3 */
/* AES256_STAT[DOUTCNT] Bits */
#define AES256_STAT_DOUTCNT_OFS                  (12)                            /*!< AESDOUTCNTx Bit Offset */
#define AES256_STAT_DOUTCNT_MASK                 ((uint16_t)0xF000)              /*!< AESDOUTCNTx Bit Mask */
#define AES256_STAT_DOUTCNT0                     ((uint16_t)0x1000)              /*!< DOUTCNT Bit 0 */
#define AES256_STAT_DOUTCNT1                     ((uint16_t)0x2000)              /*!< DOUTCNT Bit 1 */
#define AES256_STAT_DOUTCNT2                     ((uint16_t)0x4000)              /*!< DOUTCNT Bit 2 */
#define AES256_STAT_DOUTCNT3                     ((uint16_t)0x8000)              /*!< DOUTCNT Bit 3 */
/* AES256_KEY[KEY0] Bits */
#define AES256_KEY_KEY0_OFS                      ( 0)                            /*!< AESKEY0x Bit Offset */
#define AES256_KEY_KEY0_MASK                     ((uint16_t)0x00FF)              /*!< AESKEY0x Bit Mask */
#define AES256_KEY_KEY00                         ((uint16_t)0x0001)              /*!< KEY0 Bit 0 */
#define AES256_KEY_KEY01                         ((uint16_t)0x0002)              /*!< KEY0 Bit 1 */
#define AES256_KEY_KEY02                         ((uint16_t)0x0004)              /*!< KEY0 Bit 2 */
#define AES256_KEY_KEY03                         ((uint16_t)0x0008)              /*!< KEY0 Bit 3 */
#define AES256_KEY_KEY04                         ((uint16_t)0x0010)              /*!< KEY0 Bit 4 */
#define AES256_KEY_KEY05                         ((uint16_t)0x0020)              /*!< KEY0 Bit 5 */
#define AES256_KEY_KEY06                         ((uint16_t)0x0040)              /*!< KEY0 Bit 6 */
#define AES256_KEY_KEY07                         ((uint16_t)0x0080)              /*!< KEY0 Bit 7 */
/* AES256_KEY[KEY1] Bits */
#define AES256_KEY_KEY1_OFS                      ( 8)                            /*!< AESKEY1x Bit Offset */
#define AES256_KEY_KEY1_MASK                     ((uint16_t)0xFF00)              /*!< AESKEY1x Bit Mask */
#define AES256_KEY_KEY10                         ((uint16_t)0x0100)              /*!< KEY1 Bit 0 */
#define AES256_KEY_KEY11                         ((uint16_t)0x0200)              /*!< KEY1 Bit 1 */
#define AES256_KEY_KEY12                         ((uint16_t)0x0400)              /*!< KEY1 Bit 2 */
#define AES256_KEY_KEY13                         ((uint16_t)0x0800)              /*!< KEY1 Bit 3 */
#define AES256_KEY_KEY14                         ((uint16_t)0x1000)              /*!< KEY1 Bit 4 */
#define AES256_KEY_KEY15                         ((uint16_t)0x2000)              /*!< KEY1 Bit 5 */
#define AES256_KEY_KEY16                         ((uint16_t)0x4000)              /*!< KEY1 Bit 6 */
#define AES256_KEY_KEY17                         ((uint16_t)0x8000)              /*!< KEY1 Bit 7 */
/* AES256_DIN[DIN0] Bits */
#define AES256_DIN_DIN0_OFS                      ( 0)                            /*!< AESDIN0x Bit Offset */
#define AES256_DIN_DIN0_MASK                     ((uint16_t)0x00FF)              /*!< AESDIN0x Bit Mask */
#define AES256_DIN_DIN00                         ((uint16_t)0x0001)              /*!< DIN0 Bit 0 */
#define AES256_DIN_DIN01                         ((uint16_t)0x0002)              /*!< DIN0 Bit 1 */
#define AES256_DIN_DIN02                         ((uint16_t)0x0004)              /*!< DIN0 Bit 2 */
#define AES256_DIN_DIN03                         ((uint16_t)0x0008)              /*!< DIN0 Bit 3 */
#define AES256_DIN_DIN04                         ((uint16_t)0x0010)              /*!< DIN0 Bit 4 */
#define AES256_DIN_DIN05                         ((uint16_t)0x0020)              /*!< DIN0 Bit 5 */
#define AES256_DIN_DIN06                         ((uint16_t)0x0040)              /*!< DIN0 Bit 6 */
#define AES256_DIN_DIN07                         ((uint16_t)0x0080)              /*!< DIN0 Bit 7 */
/* AES256_DIN[DIN1] Bits */
#define AES256_DIN_DIN1_OFS                      ( 8)                            /*!< AESDIN1x Bit Offset */
#define AES256_DIN_DIN1_MASK                     ((uint16_t)0xFF00)              /*!< AESDIN1x Bit Mask */
#define AES256_DIN_DIN10                         ((uint16_t)0x0100)              /*!< DIN1 Bit 0 */
#define AES256_DIN_DIN11                         ((uint16_t)0x0200)              /*!< DIN1 Bit 1 */
#define AES256_DIN_DIN12                         ((uint16_t)0x0400)              /*!< DIN1 Bit 2 */
#define AES256_DIN_DIN13                         ((uint16_t)0x0800)              /*!< DIN1 Bit 3 */
#define AES256_DIN_DIN14                         ((uint16_t)0x1000)              /*!< DIN1 Bit 4 */
#define AES256_DIN_DIN15                         ((uint16_t)0x2000)              /*!< DIN1 Bit 5 */
#define AES256_DIN_DIN16                         ((uint16_t)0x4000)              /*!< DIN1 Bit 6 */
#define AES256_DIN_DIN17                         ((uint16_t)0x8000)              /*!< DIN1 Bit 7 */
/* AES256_DOUT[DOUT0] Bits */
#define AES256_DOUT_DOUT0_OFS                    ( 0)                            /*!< AESDOUT0x Bit Offset */
#define AES256_DOUT_DOUT0_MASK                   ((uint16_t)0x00FF)              /*!< AESDOUT0x Bit Mask */
#define AES256_DOUT_DOUT00                       ((uint16_t)0x0001)              /*!< DOUT0 Bit 0 */
#define AES256_DOUT_DOUT01                       ((uint16_t)0x0002)              /*!< DOUT0 Bit 1 */
#define AES256_DOUT_DOUT02                       ((uint16_t)0x0004)              /*!< DOUT0 Bit 2 */
#define AES256_DOUT_DOUT03                       ((uint16_t)0x0008)              /*!< DOUT0 Bit 3 */
#define AES256_DOUT_DOUT04                       ((uint16_t)0x0010)              /*!< DOUT0 Bit 4 */
#define AES256_DOUT_DOUT05                       ((uint16_t)0x0020)              /*!< DOUT0 Bit 5 */
#define AES256_DOUT_DOUT06                       ((uint16_t)0x0040)              /*!< DOUT0 Bit 6 */
#define AES256_DOUT_DOUT07                       ((uint16_t)0x0080)              /*!< DOUT0 Bit 7 */
/* AES256_DOUT[DOUT1] Bits */
#define AES256_DOUT_DOUT1_OFS                    ( 8)                            /*!< AESDOUT1x Bit Offset */
#define AES256_DOUT_DOUT1_MASK                   ((uint16_t)0xFF00)              /*!< AESDOUT1x Bit Mask */
#define AES256_DOUT_DOUT10                       ((uint16_t)0x0100)              /*!< DOUT1 Bit 0 */
#define AES256_DOUT_DOUT11                       ((uint16_t)0x0200)              /*!< DOUT1 Bit 1 */
#define AES256_DOUT_DOUT12                       ((uint16_t)0x0400)              /*!< DOUT1 Bit 2 */
#define AES256_DOUT_DOUT13                       ((uint16_t)0x0800)              /*!< DOUT1 Bit 3 */
#define AES256_DOUT_DOUT14                       ((uint16_t)0x1000)              /*!< DOUT1 Bit 4 */
#define AES256_DOUT_DOUT15                       ((uint16_t)0x2000)              /*!< DOUT1 Bit 5 */
#define AES256_DOUT_DOUT16                       ((uint16_t)0x4000)              /*!< DOUT1 Bit 6 */
#define AES256_DOUT_DOUT17                       ((uint16_t)0x8000)              /*!< DOUT1 Bit 7 */
/* AES256_XDIN[XDIN0] Bits */
#define AES256_XDIN_XDIN0_OFS                    ( 0)                            /*!< AESXDIN0x Bit Offset */
#define AES256_XDIN_XDIN0_MASK                   ((uint16_t)0x00FF)              /*!< AESXDIN0x Bit Mask */
#define AES256_XDIN_XDIN00                       ((uint16_t)0x0001)              /*!< XDIN0 Bit 0 */
#define AES256_XDIN_XDIN01                       ((uint16_t)0x0002)              /*!< XDIN0 Bit 1 */
#define AES256_XDIN_XDIN02                       ((uint16_t)0x0004)              /*!< XDIN0 Bit 2 */
#define AES256_XDIN_XDIN03                       ((uint16_t)0x0008)              /*!< XDIN0 Bit 3 */
#define AES256_XDIN_XDIN04                       ((uint16_t)0x0010)              /*!< XDIN0 Bit 4 */
#define AES256_XDIN_XDIN05                       ((uint16_t)0x0020)              /*!< XDIN0 Bit 5 */
#define AES256_XDIN_XDIN06                       ((uint16_t)0x0040)              /*!< XDIN0 Bit 6 */
#define AES256_XDIN_XDIN07                       ((uint16_t)0x0080)              /*!< XDIN0 Bit 7 */
/* AES256_XDIN[XDIN1] Bits */
#define AES256_XDIN_XDIN1_OFS                    ( 8)                            /*!< AESXDIN1x Bit Offset */
#define AES256_XDIN_XDIN1_MASK                   ((uint16_t)0xFF00)              /*!< AESXDIN1x Bit Mask */
#define AES256_XDIN_XDIN10                       ((uint16_t)0x0100)              /*!< XDIN1 Bit 0 */
#define AES256_XDIN_XDIN11                       ((uint16_t)0x0200)              /*!< XDIN1 Bit 1 */
#define AES256_XDIN_XDIN12                       ((uint16_t)0x0400)              /*!< XDIN1 Bit 2 */
#define AES256_XDIN_XDIN13                       ((uint16_t)0x0800)              /*!< XDIN1 Bit 3 */
#define AES256_XDIN_XDIN14                       ((uint16_t)0x1000)              /*!< XDIN1 Bit 4 */
#define AES256_XDIN_XDIN15                       ((uint16_t)0x2000)              /*!< XDIN1 Bit 5 */
#define AES256_XDIN_XDIN16                       ((uint16_t)0x4000)              /*!< XDIN1 Bit 6 */
#define AES256_XDIN_XDIN17                       ((uint16_t)0x8000)              /*!< XDIN1 Bit 7 */
/* AES256_XIN[XIN0] Bits */
#define AES256_XIN_XIN0_OFS                      ( 0)                            /*!< AESXIN0x Bit Offset */
#define AES256_XIN_XIN0_MASK                     ((uint16_t)0x00FF)              /*!< AESXIN0x Bit Mask */
#define AES256_XIN_XIN00                         ((uint16_t)0x0001)              /*!< XIN0 Bit 0 */
#define AES256_XIN_XIN01                         ((uint16_t)0x0002)              /*!< XIN0 Bit 1 */
#define AES256_XIN_XIN02                         ((uint16_t)0x0004)              /*!< XIN0 Bit 2 */
#define AES256_XIN_XIN03                         ((uint16_t)0x0008)              /*!< XIN0 Bit 3 */
#define AES256_XIN_XIN04                         ((uint16_t)0x0010)              /*!< XIN0 Bit 4 */
#define AES256_XIN_XIN05                         ((uint16_t)0x0020)              /*!< XIN0 Bit 5 */
#define AES256_XIN_XIN06                         ((uint16_t)0x0040)              /*!< XIN0 Bit 6 */
#define AES256_XIN_XIN07                         ((uint16_t)0x0080)              /*!< XIN0 Bit 7 */
/* AES256_XIN[XIN1] Bits */
#define AES256_XIN_XIN1_OFS                      ( 8)                            /*!< AESXIN1x Bit Offset */
#define AES256_XIN_XIN1_MASK                     ((uint16_t)0xFF00)              /*!< AESXIN1x Bit Mask */
#define AES256_XIN_XIN10                         ((uint16_t)0x0100)              /*!< XIN1 Bit 0 */
#define AES256_XIN_XIN11                         ((uint16_t)0x0200)              /*!< XIN1 Bit 1 */
#define AES256_XIN_XIN12                         ((uint16_t)0x0400)              /*!< XIN1 Bit 2 */
#define AES256_XIN_XIN13                         ((uint16_t)0x0800)              /*!< XIN1 Bit 3 */
#define AES256_XIN_XIN14                         ((uint16_t)0x1000)              /*!< XIN1 Bit 4 */
#define AES256_XIN_XIN15                         ((uint16_t)0x2000)              /*!< XIN1 Bit 5 */
#define AES256_XIN_XIN16                         ((uint16_t)0x4000)              /*!< XIN1 Bit 6 */
#define AES256_XIN_XIN17                         ((uint16_t)0x8000)              /*!< XIN1 Bit 7 */


/******************************************************************************
* CAPTIO Bits
******************************************************************************/
/* CAPTIO_CTL[PISEL] Bits */
#define CAPTIO_CTL_PISEL_OFS                     ( 1)                            /*!< CAPTIOPISELx Bit Offset */
#define CAPTIO_CTL_PISEL_MASK                    ((uint16_t)0x000E)              /*!< CAPTIOPISELx Bit Mask */
#define CAPTIO_CTL_PISEL0                        ((uint16_t)0x0002)              /*!< PISEL Bit 0 */
#define CAPTIO_CTL_PISEL1                        ((uint16_t)0x0004)              /*!< PISEL Bit 1 */
#define CAPTIO_CTL_PISEL2                        ((uint16_t)0x0008)              /*!< PISEL Bit 2 */
#define CAPTIO_CTL_PISEL_0                       ((uint16_t)0x0000)              /*!< Px.0 */
#define CAPTIO_CTL_PISEL_1                       ((uint16_t)0x0002)              /*!< Px.1 */
#define CAPTIO_CTL_PISEL_2                       ((uint16_t)0x0004)              /*!< Px.2 */
#define CAPTIO_CTL_PISEL_3                       ((uint16_t)0x0006)              /*!< Px.3 */
#define CAPTIO_CTL_PISEL_4                       ((uint16_t)0x0008)              /*!< Px.4 */
#define CAPTIO_CTL_PISEL_5                       ((uint16_t)0x000A)              /*!< Px.5 */
#define CAPTIO_CTL_PISEL_6                       ((uint16_t)0x000C)              /*!< Px.6 */
#define CAPTIO_CTL_PISEL_7                       ((uint16_t)0x000E)              /*!< Px.7 */
/* CAPTIO_CTL[POSEL] Bits */
#define CAPTIO_CTL_POSEL_OFS                     ( 4)                            /*!< CAPTIOPOSELx Bit Offset */
#define CAPTIO_CTL_POSEL_MASK                    ((uint16_t)0x00F0)              /*!< CAPTIOPOSELx Bit Mask */
#define CAPTIO_CTL_POSEL0                        ((uint16_t)0x0010)              /*!< POSEL Bit 0 */
#define CAPTIO_CTL_POSEL1                        ((uint16_t)0x0020)              /*!< POSEL Bit 1 */
#define CAPTIO_CTL_POSEL2                        ((uint16_t)0x0040)              /*!< POSEL Bit 2 */
#define CAPTIO_CTL_POSEL3                        ((uint16_t)0x0080)              /*!< POSEL Bit 3 */
#define CAPTIO_CTL_POSEL_0                       ((uint16_t)0x0000)              /*!< Px = PJ */
#define CAPTIO_CTL_POSEL_1                       ((uint16_t)0x0010)              /*!< Px = P1 */
#define CAPTIO_CTL_POSEL_2                       ((uint16_t)0x0020)              /*!< Px = P2 */
#define CAPTIO_CTL_POSEL_3                       ((uint16_t)0x0030)              /*!< Px = P3 */
#define CAPTIO_CTL_POSEL_4                       ((uint16_t)0x0040)              /*!< Px = P4 */
#define CAPTIO_CTL_POSEL_5                       ((uint16_t)0x0050)              /*!< Px = P5 */
#define CAPTIO_CTL_POSEL_6                       ((uint16_t)0x0060)              /*!< Px = P6 */
#define CAPTIO_CTL_POSEL_7                       ((uint16_t)0x0070)              /*!< Px = P7 */
#define CAPTIO_CTL_POSEL_8                       ((uint16_t)0x0080)              /*!< Px = P8 */
#define CAPTIO_CTL_POSEL_9                       ((uint16_t)0x0090)              /*!< Px = P9 */
#define CAPTIO_CTL_POSEL_10                      ((uint16_t)0x00A0)              /*!< Px = P10 */
#define CAPTIO_CTL_POSEL_11                      ((uint16_t)0x00B0)              /*!< Px = P11 */
#define CAPTIO_CTL_POSEL_12                      ((uint16_t)0x00C0)              /*!< Px = P12 */
#define CAPTIO_CTL_POSEL_13                      ((uint16_t)0x00D0)              /*!< Px = P13 */
#define CAPTIO_CTL_POSEL_14                      ((uint16_t)0x00E0)              /*!< Px = P14 */
#define CAPTIO_CTL_POSEL_15                      ((uint16_t)0x00F0)              /*!< Px = P15 */
#define CAPTIO_CTL_POSEL__PJ                     ((uint16_t)0x0000)              /*!< Px = PJ */
#define CAPTIO_CTL_POSEL__P1                     ((uint16_t)0x0010)              /*!< Px = P1 */
#define CAPTIO_CTL_POSEL__P2                     ((uint16_t)0x0020)              /*!< Px = P2 */
#define CAPTIO_CTL_POSEL__P3                     ((uint16_t)0x0030)              /*!< Px = P3 */
#define CAPTIO_CTL_POSEL__P4                     ((uint16_t)0x0040)              /*!< Px = P4 */
#define CAPTIO_CTL_POSEL__P5                     ((uint16_t)0x0050)              /*!< Px = P5 */
#define CAPTIO_CTL_POSEL__P6                     ((uint16_t)0x0060)              /*!< Px = P6 */
#define CAPTIO_CTL_POSEL__P7                     ((uint16_t)0x0070)              /*!< Px = P7 */
#define CAPTIO_CTL_POSEL__P8                     ((uint16_t)0x0080)              /*!< Px = P8 */
#define CAPTIO_CTL_POSEL__P9                     ((uint16_t)0x0090)              /*!< Px = P9 */
#define CAPTIO_CTL_POSEL__P10                    ((uint16_t)0x00A0)              /*!< Px = P10 */
#define CAPTIO_CTL_POSEL__P11                    ((uint16_t)0x00B0)              /*!< Px = P11 */
#define CAPTIO_CTL_POSEL__P12                    ((uint16_t)0x00C0)              /*!< Px = P12 */
#define CAPTIO_CTL_POSEL__P13                    ((uint16_t)0x00D0)              /*!< Px = P13 */
#define CAPTIO_CTL_POSEL__P14                    ((uint16_t)0x00E0)              /*!< Px = P14 */
#define CAPTIO_CTL_POSEL__P15                    ((uint16_t)0x00F0)              /*!< Px = P15 */
/* CAPTIO_CTL[EN] Bits */
#define CAPTIO_CTL_EN_OFS                        ( 8)                            /*!< CAPTIOEN Bit Offset */
#define CAPTIO_CTL_EN                            ((uint16_t)0x0100)              /*!< Capacitive Touch IO enable */
/* CAPTIO_CTL[STATE] Bits */
#define CAPTIO_CTL_STATE_OFS                     ( 9)                            /*!< CAPTIOSTATE Bit Offset */
#define CAPTIO_CTL_STATE                         ((uint16_t)0x0200)              /*!< Capacitive Touch IO state */


/******************************************************************************
* COMP_E Bits
******************************************************************************/
/* COMP_E_CTL0[IPSEL] Bits */
#define COMP_E_CTL0_IPSEL_OFS                    ( 0)                            /*!< CEIPSEL Bit Offset */
#define COMP_E_CTL0_IPSEL_MASK                   ((uint16_t)0x000F)              /*!< CEIPSEL Bit Mask */
#define COMP_E_CTL0_IPSEL0                       ((uint16_t)0x0001)              /*!< IPSEL Bit 0 */
#define COMP_E_CTL0_IPSEL1                       ((uint16_t)0x0002)              /*!< IPSEL Bit 1 */
#define COMP_E_CTL0_IPSEL2                       ((uint16_t)0x0004)              /*!< IPSEL Bit 2 */
#define COMP_E_CTL0_IPSEL3                       ((uint16_t)0x0008)              /*!< IPSEL Bit 3 */
#define COMP_E_CTL0_IPSEL_0                      ((uint16_t)0x0000)              /*!< Channel 0 selected */
#define COMP_E_CTL0_IPSEL_1                      ((uint16_t)0x0001)              /*!< Channel 1 selected */
#define COMP_E_CTL0_IPSEL_2                      ((uint16_t)0x0002)              /*!< Channel 2 selected */
#define COMP_E_CTL0_IPSEL_3                      ((uint16_t)0x0003)              /*!< Channel 3 selected */
#define COMP_E_CTL0_IPSEL_4                      ((uint16_t)0x0004)              /*!< Channel 4 selected */
#define COMP_E_CTL0_IPSEL_5                      ((uint16_t)0x0005)              /*!< Channel 5 selected */
#define COMP_E_CTL0_IPSEL_6                      ((uint16_t)0x0006)              /*!< Channel 6 selected */
#define COMP_E_CTL0_IPSEL_7                      ((uint16_t)0x0007)              /*!< Channel 7 selected */
#define COMP_E_CTL0_IPSEL_8                      ((uint16_t)0x0008)              /*!< Channel 8 selected */
#define COMP_E_CTL0_IPSEL_9                      ((uint16_t)0x0009)              /*!< Channel 9 selected */
#define COMP_E_CTL0_IPSEL_10                     ((uint16_t)0x000A)              /*!< Channel 10 selected */
#define COMP_E_CTL0_IPSEL_11                     ((uint16_t)0x000B)              /*!< Channel 11 selected */
#define COMP_E_CTL0_IPSEL_12                     ((uint16_t)0x000C)              /*!< Channel 12 selected */
#define COMP_E_CTL0_IPSEL_13                     ((uint16_t)0x000D)              /*!< Channel 13 selected */
#define COMP_E_CTL0_IPSEL_14                     ((uint16_t)0x000E)              /*!< Channel 14 selected */
#define COMP_E_CTL0_IPSEL_15                     ((uint16_t)0x000F)              /*!< Channel 15 selected */
/* COMP_E_CTL0[IPEN] Bits */
#define COMP_E_CTL0_IPEN_OFS                     ( 7)                            /*!< CEIPEN Bit Offset */
#define COMP_E_CTL0_IPEN                         ((uint16_t)0x0080)              /*!< Channel input enable for the V+ terminal */
/* COMP_E_CTL0[IMSEL] Bits */
#define COMP_E_CTL0_IMSEL_OFS                    ( 8)                            /*!< CEIMSEL Bit Offset */
#define COMP_E_CTL0_IMSEL_MASK                   ((uint16_t)0x0F00)              /*!< CEIMSEL Bit Mask */
#define COMP_E_CTL0_IMSEL0                       ((uint16_t)0x0100)              /*!< IMSEL Bit 0 */
#define COMP_E_CTL0_IMSEL1                       ((uint16_t)0x0200)              /*!< IMSEL Bit 1 */
#define COMP_E_CTL0_IMSEL2                       ((uint16_t)0x0400)              /*!< IMSEL Bit 2 */
#define COMP_E_CTL0_IMSEL3                       ((uint16_t)0x0800)              /*!< IMSEL Bit 3 */
#define COMP_E_CTL0_IMSEL_0                      ((uint16_t)0x0000)              /*!< Channel 0 selected */
#define COMP_E_CTL0_IMSEL_1                      ((uint16_t)0x0100)              /*!< Channel 1 selected */
#define COMP_E_CTL0_IMSEL_2                      ((uint16_t)0x0200)              /*!< Channel 2 selected */
#define COMP_E_CTL0_IMSEL_3                      ((uint16_t)0x0300)              /*!< Channel 3 selected */
#define COMP_E_CTL0_IMSEL_4                      ((uint16_t)0x0400)              /*!< Channel 4 selected */
#define COMP_E_CTL0_IMSEL_5                      ((uint16_t)0x0500)              /*!< Channel 5 selected */
#define COMP_E_CTL0_IMSEL_6                      ((uint16_t)0x0600)              /*!< Channel 6 selected */
#define COMP_E_CTL0_IMSEL_7                      ((uint16_t)0x0700)              /*!< Channel 7 selected */
#define COMP_E_CTL0_IMSEL_8                      ((uint16_t)0x0800)              /*!< Channel 8 selected */
#define COMP_E_CTL0_IMSEL_9                      ((uint16_t)0x0900)              /*!< Channel 9 selected */
#define COMP_E_CTL0_IMSEL_10                     ((uint16_t)0x0A00)              /*!< Channel 10 selected */
#define COMP_E_CTL0_IMSEL_11                     ((uint16_t)0x0B00)              /*!< Channel 11 selected */
#define COMP_E_CTL0_IMSEL_12                     ((uint16_t)0x0C00)              /*!< Channel 12 selected */
#define COMP_E_CTL0_IMSEL_13                     ((uint16_t)0x0D00)              /*!< Channel 13 selected */
#define COMP_E_CTL0_IMSEL_14                     ((uint16_t)0x0E00)              /*!< Channel 14 selected */
#define COMP_E_CTL0_IMSEL_15                     ((uint16_t)0x0F00)              /*!< Channel 15 selected */
/* COMP_E_CTL0[IMEN] Bits */
#define COMP_E_CTL0_IMEN_OFS                     (15)                            /*!< CEIMEN Bit Offset */
#define COMP_E_CTL0_IMEN                         ((uint16_t)0x8000)              /*!< Channel input enable for the - terminal */
/* COMP_E_CTL1[OUT] Bits */
#define COMP_E_CTL1_OUT_OFS                      ( 0)                            /*!< CEOUT Bit Offset */
#define COMP_E_CTL1_OUT                          ((uint16_t)0x0001)              /*!< Comparator output value */
/* COMP_E_CTL1[OUTPOL] Bits */
#define COMP_E_CTL1_OUTPOL_OFS                   ( 1)                            /*!< CEOUTPOL Bit Offset */
#define COMP_E_CTL1_OUTPOL                       ((uint16_t)0x0002)              /*!< Comparator output polarity */
/* COMP_E_CTL1[F] Bits */
#define COMP_E_CTL1_F_OFS                        ( 2)                            /*!< CEF Bit Offset */
#define COMP_E_CTL1_F                            ((uint16_t)0x0004)              /*!< Comparator output filter */
/* COMP_E_CTL1[IES] Bits */
#define COMP_E_CTL1_IES_OFS                      ( 3)                            /*!< CEIES Bit Offset */
#define COMP_E_CTL1_IES                          ((uint16_t)0x0008)              /*!< Interrupt edge select for CEIIFG and CEIFG */
/* COMP_E_CTL1[SHORT] Bits */
#define COMP_E_CTL1_SHORT_OFS                    ( 4)                            /*!< CESHORT Bit Offset */
#define COMP_E_CTL1_SHORT                        ((uint16_t)0x0010)              /*!< Input short */
/* COMP_E_CTL1[EX] Bits */
#define COMP_E_CTL1_EX_OFS                       ( 5)                            /*!< CEEX Bit Offset */
#define COMP_E_CTL1_EX                           ((uint16_t)0x0020)              /*!< Exchange */
/* COMP_E_CTL1[FDLY] Bits */
#define COMP_E_CTL1_FDLY_OFS                     ( 6)                            /*!< CEFDLY Bit Offset */
#define COMP_E_CTL1_FDLY_MASK                    ((uint16_t)0x00C0)              /*!< CEFDLY Bit Mask */
#define COMP_E_CTL1_FDLY0                        ((uint16_t)0x0040)              /*!< FDLY Bit 0 */
#define COMP_E_CTL1_FDLY1                        ((uint16_t)0x0080)              /*!< FDLY Bit 1 */
#define COMP_E_CTL1_FDLY_0                       ((uint16_t)0x0000)              /*!< Typical filter delay of TBD (450) ns */
#define COMP_E_CTL1_FDLY_1                       ((uint16_t)0x0040)              /*!< Typical filter delay of TBD (900) ns */
#define COMP_E_CTL1_FDLY_2                       ((uint16_t)0x0080)              /*!< Typical filter delay of TBD (1800) ns */
#define COMP_E_CTL1_FDLY_3                       ((uint16_t)0x00C0)              /*!< Typical filter delay of TBD (3600) ns */
/* COMP_E_CTL1[PWRMD] Bits */
#define COMP_E_CTL1_PWRMD_OFS                    ( 8)                            /*!< CEPWRMD Bit Offset */
#define COMP_E_CTL1_PWRMD_MASK                   ((uint16_t)0x0300)              /*!< CEPWRMD Bit Mask */
#define COMP_E_CTL1_PWRMD0                       ((uint16_t)0x0100)              /*!< PWRMD Bit 0 */
#define COMP_E_CTL1_PWRMD1                       ((uint16_t)0x0200)              /*!< PWRMD Bit 1 */
#define COMP_E_CTL1_PWRMD_0                      ((uint16_t)0x0000)              /*!< High-speed mode */
#define COMP_E_CTL1_PWRMD_1                      ((uint16_t)0x0100)              /*!< Normal mode */
#define COMP_E_CTL1_PWRMD_2                      ((uint16_t)0x0200)              /*!< Ultra-low power mode */
/* COMP_E_CTL1[ON] Bits */
#define COMP_E_CTL1_ON_OFS                       (10)                            /*!< CEON Bit Offset */
#define COMP_E_CTL1_ON                           ((uint16_t)0x0400)              /*!< Comparator On */
/* COMP_E_CTL1[MRVL] Bits */
#define COMP_E_CTL1_MRVL_OFS                     (11)                            /*!< CEMRVL Bit Offset */
#define COMP_E_CTL1_MRVL                         ((uint16_t)0x0800)              /*!< This bit is valid of CEMRVS is set to 1 */
/* COMP_E_CTL1[MRVS] Bits */
#define COMP_E_CTL1_MRVS_OFS                     (12)                            /*!< CEMRVS Bit Offset */
#define COMP_E_CTL1_MRVS                         ((uint16_t)0x1000)
/* COMP_E_CTL2[REF0] Bits */
#define COMP_E_CTL2_REF0_OFS                     ( 0)                            /*!< CEREF0 Bit Offset */
#define COMP_E_CTL2_REF0_MASK                    ((uint16_t)0x001F)              /*!< CEREF0 Bit Mask */
/* COMP_E_CTL2[RSEL] Bits */
#define COMP_E_CTL2_RSEL_OFS                     ( 5)                            /*!< CERSEL Bit Offset */
#define COMP_E_CTL2_RSEL                         ((uint16_t)0x0020)              /*!< Reference select */
/* COMP_E_CTL2[RS] Bits */
#define COMP_E_CTL2_RS_OFS                       ( 6)                            /*!< CERS Bit Offset */
#define COMP_E_CTL2_RS_MASK                      ((uint16_t)0x00C0)              /*!< CERS Bit Mask */
#define COMP_E_CTL2_RS0                          ((uint16_t)0x0040)              /*!< RS Bit 0 */
#define COMP_E_CTL2_RS1                          ((uint16_t)0x0080)              /*!< RS Bit 1 */
#define COMP_E_CTL2_RS_0                         ((uint16_t)0x0000)              /*!< No current is drawn by the reference circuitry */
#define COMP_E_CTL2_RS_1                         ((uint16_t)0x0040)              /*!< VCC applied to the resistor ladder */
#define COMP_E_CTL2_RS_2                         ((uint16_t)0x0080)              /*!< Shared reference voltage applied to the resistor ladder */
#define COMP_E_CTL2_RS_3                         ((uint16_t)0x00C0)              /*!< Shared reference voltage supplied to V(CREF). Resistor ladder is off */
/* COMP_E_CTL2[REF1] Bits */
#define COMP_E_CTL2_REF1_OFS                     ( 8)                            /*!< CEREF1 Bit Offset */
#define COMP_E_CTL2_REF1_MASK                    ((uint16_t)0x1F00)              /*!< CEREF1 Bit Mask */
/* COMP_E_CTL2[REFL] Bits */
#define COMP_E_CTL2_REFL_OFS                     (13)                            /*!< CEREFL Bit Offset */
#define COMP_E_CTL2_REFL_MASK                    ((uint16_t)0x6000)              /*!< CEREFL Bit Mask */
#define COMP_E_CTL2_REFL0                        ((uint16_t)0x2000)              /*!< REFL Bit 0 */
#define COMP_E_CTL2_REFL1                        ((uint16_t)0x4000)              /*!< REFL Bit 1 */
#define COMP_E_CTL2_CEREFL_0                     ((uint16_t)0x0000)              /*!< Reference amplifier is disabled. No reference voltage is requested */
#define COMP_E_CTL2_CEREFL_1                     ((uint16_t)0x2000)              /*!< 1.2 V is selected as shared reference voltage input */
#define COMP_E_CTL2_CEREFL_2                     ((uint16_t)0x4000)              /*!< 2.0 V is selected as shared reference voltage input */
#define COMP_E_CTL2_CEREFL_3                     ((uint16_t)0x6000)              /*!< 2.5 V is selected as shared reference voltage input */
#define COMP_E_CTL2_REFL__OFF                    ((uint16_t)0x0000)              /*!< Reference amplifier is disabled. No reference voltage is requested */
#define COMP_E_CTL2_REFL__1P2V                   ((uint16_t)0x2000)              /*!< 1.2 V is selected as shared reference voltage input */
#define COMP_E_CTL2_REFL__2P0V                   ((uint16_t)0x4000)              /*!< 2.0 V is selected as shared reference voltage input */
#define COMP_E_CTL2_REFL__2P5V                   ((uint16_t)0x6000)              /*!< 2.5 V is selected as shared reference voltage input */
/* COMP_E_CTL2[REFACC] Bits */
#define COMP_E_CTL2_REFACC_OFS                   (15)                            /*!< CEREFACC Bit Offset */
#define COMP_E_CTL2_REFACC                       ((uint16_t)0x8000)              /*!< Reference accuracy */
/* COMP_E_CTL3[PD0] Bits */
#define COMP_E_CTL3_PD0_OFS                      ( 0)                            /*!< CEPD0 Bit Offset */
#define COMP_E_CTL3_PD0                          ((uint16_t)0x0001)              /*!< Port disable */
/* COMP_E_CTL3[PD1] Bits */
#define COMP_E_CTL3_PD1_OFS                      ( 1)                            /*!< CEPD1 Bit Offset */
#define COMP_E_CTL3_PD1                          ((uint16_t)0x0002)              /*!< Port disable */
/* COMP_E_CTL3[PD2] Bits */
#define COMP_E_CTL3_PD2_OFS                      ( 2)                            /*!< CEPD2 Bit Offset */
#define COMP_E_CTL3_PD2                          ((uint16_t)0x0004)              /*!< Port disable */
/* COMP_E_CTL3[PD3] Bits */
#define COMP_E_CTL3_PD3_OFS                      ( 3)                            /*!< CEPD3 Bit Offset */
#define COMP_E_CTL3_PD3                          ((uint16_t)0x0008)              /*!< Port disable */
/* COMP_E_CTL3[PD4] Bits */
#define COMP_E_CTL3_PD4_OFS                      ( 4)                            /*!< CEPD4 Bit Offset */
#define COMP_E_CTL3_PD4                          ((uint16_t)0x0010)              /*!< Port disable */
/* COMP_E_CTL3[PD5] Bits */
#define COMP_E_CTL3_PD5_OFS                      ( 5)                            /*!< CEPD5 Bit Offset */
#define COMP_E_CTL3_PD5                          ((uint16_t)0x0020)              /*!< Port disable */
/* COMP_E_CTL3[PD6] Bits */
#define COMP_E_CTL3_PD6_OFS                      ( 6)                            /*!< CEPD6 Bit Offset */
#define COMP_E_CTL3_PD6                          ((uint16_t)0x0040)              /*!< Port disable */
/* COMP_E_CTL3[PD7] Bits */
#define COMP_E_CTL3_PD7_OFS                      ( 7)                            /*!< CEPD7 Bit Offset */
#define COMP_E_CTL3_PD7                          ((uint16_t)0x0080)              /*!< Port disable */
/* COMP_E_CTL3[PD8] Bits */
#define COMP_E_CTL3_PD8_OFS                      ( 8)                            /*!< CEPD8 Bit Offset */
#define COMP_E_CTL3_PD8                          ((uint16_t)0x0100)              /*!< Port disable */
/* COMP_E_CTL3[PD9] Bits */
#define COMP_E_CTL3_PD9_OFS                      ( 9)                            /*!< CEPD9 Bit Offset */
#define COMP_E_CTL3_PD9                          ((uint16_t)0x0200)              /*!< Port disable */
/* COMP_E_CTL3[PD10] Bits */
#define COMP_E_CTL3_PD10_OFS                     (10)                            /*!< CEPD10 Bit Offset */
#define COMP_E_CTL3_PD10                         ((uint16_t)0x0400)              /*!< Port disable */
/* COMP_E_CTL3[PD11] Bits */
#define COMP_E_CTL3_PD11_OFS                     (11)                            /*!< CEPD11 Bit Offset */
#define COMP_E_CTL3_PD11                         ((uint16_t)0x0800)              /*!< Port disable */
/* COMP_E_CTL3[PD12] Bits */
#define COMP_E_CTL3_PD12_OFS                     (12)                            /*!< CEPD12 Bit Offset */
#define COMP_E_CTL3_PD12                         ((uint16_t)0x1000)              /*!< Port disable */
/* COMP_E_CTL3[PD13] Bits */
#define COMP_E_CTL3_PD13_OFS                     (13)                            /*!< CEPD13 Bit Offset */
#define COMP_E_CTL3_PD13                         ((uint16_t)0x2000)              /*!< Port disable */
/* COMP_E_CTL3[PD14] Bits */
#define COMP_E_CTL3_PD14_OFS                     (14)                            /*!< CEPD14 Bit Offset */
#define COMP_E_CTL3_PD14                         ((uint16_t)0x4000)              /*!< Port disable */
/* COMP_E_CTL3[PD15] Bits */
#define COMP_E_CTL3_PD15_OFS                     (15)                            /*!< CEPD15 Bit Offset */
#define COMP_E_CTL3_PD15                         ((uint16_t)0x8000)              /*!< Port disable */
/* COMP_E_INT[IFG] Bits */
#define COMP_E_INT_IFG_OFS                       ( 0)                            /*!< CEIFG Bit Offset */
#define COMP_E_INT_IFG                           ((uint16_t)0x0001)              /*!< Comparator output interrupt flag */
/* COMP_E_INT[IIFG] Bits */
#define COMP_E_INT_IIFG_OFS                      ( 1)                            /*!< CEIIFG Bit Offset */
#define COMP_E_INT_IIFG                          ((uint16_t)0x0002)              /*!< Comparator output inverted interrupt flag */
/* COMP_E_INT[RDYIFG] Bits */
#define COMP_E_INT_RDYIFG_OFS                    ( 4)                            /*!< CERDYIFG Bit Offset */
#define COMP_E_INT_RDYIFG                        ((uint16_t)0x0010)              /*!< Comparator ready interrupt flag */
/* COMP_E_INT[IE] Bits */
#define COMP_E_INT_IE_OFS                        ( 8)                            /*!< CEIE Bit Offset */
#define COMP_E_INT_IE                            ((uint16_t)0x0100)              /*!< Comparator output interrupt enable */
/* COMP_E_INT[IIE] Bits */
#define COMP_E_INT_IIE_OFS                       ( 9)                            /*!< CEIIE Bit Offset */
#define COMP_E_INT_IIE                           ((uint16_t)0x0200)              /*!< Comparator output interrupt enable inverted polarity */
/* COMP_E_INT[RDYIE] Bits */
#define COMP_E_INT_RDYIE_OFS                     (12)                            /*!< CERDYIE Bit Offset */
#define COMP_E_INT_RDYIE                         ((uint16_t)0x1000)              /*!< Comparator ready interrupt enable */


/******************************************************************************
* COREDEBUG Bits
******************************************************************************/


/******************************************************************************
* CRC32 Bits
******************************************************************************/


/******************************************************************************
* CS Bits
******************************************************************************/
/* CS_KEY[KEY] Bits */
#define CS_KEY_KEY_OFS                           ( 0)                            /*!< CSKEY Bit Offset */
#define CS_KEY_KEY_MASK                          ((uint32_t)0x0000FFFF)          /*!< CSKEY Bit Mask */
/* CS_CTL0[DCOTUNE] Bits */
#define CS_CTL0_DCOTUNE_OFS                      ( 0)                            /*!< DCOTUNE Bit Offset */
#define CS_CTL0_DCOTUNE_MASK                     ((uint32_t)0x000003FF)          /*!< DCOTUNE Bit Mask */
/* CS_CTL0[DCORSEL] Bits */
#define CS_CTL0_DCORSEL_OFS                      (16)                            /*!< DCORSEL Bit Offset */
#define CS_CTL0_DCORSEL_MASK                     ((uint32_t)0x00070000)          /*!< DCORSEL Bit Mask */
#define CS_CTL0_DCORSEL0                         ((uint32_t)0x00010000)          /*!< DCORSEL Bit 0 */
#define CS_CTL0_DCORSEL1                         ((uint32_t)0x00020000)          /*!< DCORSEL Bit 1 */
#define CS_CTL0_DCORSEL2                         ((uint32_t)0x00040000)          /*!< DCORSEL Bit 2 */
#define CS_CTL0_DCORSEL_0                        ((uint32_t)0x00000000)          /*!< Nominal DCO Frequency Range (MHz): 1 to 2 */
#define CS_CTL0_DCORSEL_1                        ((uint32_t)0x00010000)          /*!< Nominal DCO Frequency Range (MHz): 2 to 4 */
#define CS_CTL0_DCORSEL_2                        ((uint32_t)0x00020000)          /*!< Nominal DCO Frequency Range (MHz): 4 to 8 */
#define CS_CTL0_DCORSEL_3                        ((uint32_t)0x00030000)          /*!< Nominal DCO Frequency Range (MHz): 8 to 16 */
#define CS_CTL0_DCORSEL_4                        ((uint32_t)0x00040000)          /*!< Nominal DCO Frequency Range (MHz): 16 to 32 */
#define CS_CTL0_DCORSEL_5                        ((uint32_t)0x00050000)          /*!< Nominal DCO Frequency Range (MHz): 32 to 64 */
/* CS_CTL0[DCORES] Bits */
#define CS_CTL0_DCORES_OFS                       (22)                            /*!< DCORES Bit Offset */
#define CS_CTL0_DCORES                           ((uint32_t)0x00400000)          /*!< Enables the DCO external resistor mode */
/* CS_CTL0[DCOEN] Bits */
#define CS_CTL0_DCOEN_OFS                        (23)                            /*!< DCOEN Bit Offset */
#define CS_CTL0_DCOEN                            ((uint32_t)0x00800000)          /*!< Enables the DCO oscillator */
/* CS_CTL1[SELM] Bits */
#define CS_CTL1_SELM_OFS                         ( 0)                            /*!< SELM Bit Offset */
#define CS_CTL1_SELM_MASK                        ((uint32_t)0x00000007)          /*!< SELM Bit Mask */
#define CS_CTL1_SELM0                            ((uint32_t)0x00000001)          /*!< SELM Bit 0 */
#define CS_CTL1_SELM1                            ((uint32_t)0x00000002)          /*!< SELM Bit 1 */
#define CS_CTL1_SELM2                            ((uint32_t)0x00000004)          /*!< SELM Bit 2 */
#define CS_CTL1_SELM_0                           ((uint32_t)0x00000000)          /*!< when LFXT available, otherwise REFOCLK */
#define CS_CTL1_SELM_1                           ((uint32_t)0x00000001)
#define CS_CTL1_SELM_2                           ((uint32_t)0x00000002)
#define CS_CTL1_SELM_3                           ((uint32_t)0x00000003)
#define CS_CTL1_SELM_4                           ((uint32_t)0x00000004)
#define CS_CTL1_SELM_5                           ((uint32_t)0x00000005)          /*!< when HFXT available, otherwise DCOCLK */
#define CS_CTL1_SELM_6                           ((uint32_t)0x00000006)          /*!< when HFXT2 available, otherwise DCOCLK */
#define CS_CTL1_SELM__LFXTCLK                    ((uint32_t)0x00000000)          /*!< when LFXT available, otherwise REFOCLK */
#define CS_CTL1_SELM__VLOCLK                     ((uint32_t)0x00000001)
#define CS_CTL1_SELM__REFOCLK                    ((uint32_t)0x00000002)
#define CS_CTL1_SELM__DCOCLK                     ((uint32_t)0x00000003)
#define CS_CTL1_SELM__MODOSC                     ((uint32_t)0x00000004)
#define CS_CTL1_SELM__HFXTCLK                    ((uint32_t)0x00000005)          /*!< when HFXT available, otherwise DCOCLK */
#define CS_CTL1_SELM__HFXT2CLK                   ((uint32_t)0x00000006)          /*!< when HFXT2 available, otherwise DCOCLK */
#define CS_CTL1_SELM_7                           ((uint32_t)0x00000007)          /*!< for future use. Defaults to DCOCLK. Not recommended for use to ensure future  */
                                                                                 /* compatibilities. */
/* CS_CTL1[SELS] Bits */
#define CS_CTL1_SELS_OFS                         ( 4)                            /*!< SELS Bit Offset */
#define CS_CTL1_SELS_MASK                        ((uint32_t)0x00000070)          /*!< SELS Bit Mask */
#define CS_CTL1_SELS0                            ((uint32_t)0x00000010)          /*!< SELS Bit 0 */
#define CS_CTL1_SELS1                            ((uint32_t)0x00000020)          /*!< SELS Bit 1 */
#define CS_CTL1_SELS2                            ((uint32_t)0x00000040)          /*!< SELS Bit 2 */
#define CS_CTL1_SELS_0                           ((uint32_t)0x00000000)          /*!< when LFXT available, otherwise REFOCLK */
#define CS_CTL1_SELS_1                           ((uint32_t)0x00000010)
#define CS_CTL1_SELS_2                           ((uint32_t)0x00000020)
#define CS_CTL1_SELS_3                           ((uint32_t)0x00000030)
#define CS_CTL1_SELS_4                           ((uint32_t)0x00000040)
#define CS_CTL1_SELS_5                           ((uint32_t)0x00000050)          /*!< when HFXT available, otherwise DCOCLK */
#define CS_CTL1_SELS_6                           ((uint32_t)0x00000060)          /*!< when HFXT2 available, otherwise DCOCLK */
#define CS_CTL1_SELS__LFXTCLK                    ((uint32_t)0x00000000)          /*!< when LFXT available, otherwise REFOCLK */
#define CS_CTL1_SELS__VLOCLK                     ((uint32_t)0x00000010)
#define CS_CTL1_SELS__REFOCLK                    ((uint32_t)0x00000020)
#define CS_CTL1_SELS__DCOCLK                     ((uint32_t)0x00000030)
#define CS_CTL1_SELS__MODOSC                     ((uint32_t)0x00000040)
#define CS_CTL1_SELS__HFXTCLK                    ((uint32_t)0x00000050)          /*!< when HFXT available, otherwise DCOCLK */
#define CS_CTL1_SELS__HFXT2CLK                   ((uint32_t)0x00000060)          /*!< when HFXT2 available, otherwise DCOCLK */
#define CS_CTL1_SELS_7                           ((uint32_t)0x00000070)          /*!< for furture use. Defaults to DCOCLK. Do not use to ensure future  */
                                                                                 /* compatibilities. */
/* CS_CTL1[SELA] Bits */
#define CS_CTL1_SELA_OFS                         ( 8)                            /*!< SELA Bit Offset */
#define CS_CTL1_SELA_MASK                        ((uint32_t)0x00000700)          /*!< SELA Bit Mask */
#define CS_CTL1_SELA0                            ((uint32_t)0x00000100)          /*!< SELA Bit 0 */
#define CS_CTL1_SELA1                            ((uint32_t)0x00000200)          /*!< SELA Bit 1 */
#define CS_CTL1_SELA2                            ((uint32_t)0x00000400)          /*!< SELA Bit 2 */
#define CS_CTL1_SELA_0                           ((uint32_t)0x00000000)          /*!< when LFXT available, otherwise REFOCLK */
#define CS_CTL1_SELA_1                           ((uint32_t)0x00000100)
#define CS_CTL1_SELA_2                           ((uint32_t)0x00000200)
#define CS_CTL1_SELA__LFXTCLK                    ((uint32_t)0x00000000)          /*!< when LFXT available, otherwise REFOCLK */
#define CS_CTL1_SELA__VLOCLK                     ((uint32_t)0x00000100)
#define CS_CTL1_SELA__REFOCLK                    ((uint32_t)0x00000200)
#define CS_CTL1_SELA_3                           ((uint32_t)0x00000300)          /*!< for future use. Defaults to REFOCLK. Not recommended for use to ensure future  */
                                                                                 /* compatibilities. */
#define CS_CTL1_SELA_4                           ((uint32_t)0x00000400)          /*!< for future use. Defaults to REFOCLK. Not recommended for use to ensure future  */
                                                                                 /* compatibilities. */
#define CS_CTL1_SELA_5                           ((uint32_t)0x00000500)          /*!< for future use. Defaults to REFOCLK. Not recommended for use to ensure future  */
                                                                                 /* compatibilities. */
#define CS_CTL1_SELA_6                           ((uint32_t)0x00000600)          /*!< for future use. Defaults to REFOCLK. Not recommended for use to ensure future  */
                                                                                 /* compatibilities. */
#define CS_CTL1_SELA_7                           ((uint32_t)0x00000700)          /*!< for future use. Defaults to REFOCLK. Not recommended for use to ensure future  */
                                                                                 /* compatibilities. */
/* CS_CTL1[SELB] Bits */
#define CS_CTL1_SELB_OFS                         (12)                            /*!< SELB Bit Offset */
#define CS_CTL1_SELB                             ((uint32_t)0x00001000)          /*!< Selects the BCLK source */
/* CS_CTL1[DIVM] Bits */
#define CS_CTL1_DIVM_OFS                         (16)                            /*!< DIVM Bit Offset */
#define CS_CTL1_DIVM_MASK                        ((uint32_t)0x00070000)          /*!< DIVM Bit Mask */
#define CS_CTL1_DIVM0                            ((uint32_t)0x00010000)          /*!< DIVM Bit 0 */
#define CS_CTL1_DIVM1                            ((uint32_t)0x00020000)          /*!< DIVM Bit 1 */
#define CS_CTL1_DIVM2                            ((uint32_t)0x00040000)          /*!< DIVM Bit 2 */
#define CS_CTL1_DIVM_0                           ((uint32_t)0x00000000)          /*!< f(MCLK)/1 */
#define CS_CTL1_DIVM_1                           ((uint32_t)0x00010000)          /*!< f(MCLK)/2 */
#define CS_CTL1_DIVM_2                           ((uint32_t)0x00020000)          /*!< f(MCLK)/4 */
#define CS_CTL1_DIVM_3                           ((uint32_t)0x00030000)          /*!< f(MCLK)/8 */
#define CS_CTL1_DIVM_4                           ((uint32_t)0x00040000)          /*!< f(MCLK)/16 */
#define CS_CTL1_DIVM_5                           ((uint32_t)0x00050000)          /*!< f(MCLK)/32 */
#define CS_CTL1_DIVM_6                           ((uint32_t)0x00060000)          /*!< f(MCLK)/64 */
#define CS_CTL1_DIVM_7                           ((uint32_t)0x00070000)          /*!< f(MCLK)/128 */
#define CS_CTL1_DIVM__1                          ((uint32_t)0x00000000)          /*!< f(MCLK)/1 */
#define CS_CTL1_DIVM__2                          ((uint32_t)0x00010000)          /*!< f(MCLK)/2 */
#define CS_CTL1_DIVM__4                          ((uint32_t)0x00020000)          /*!< f(MCLK)/4 */
#define CS_CTL1_DIVM__8                          ((uint32_t)0x00030000)          /*!< f(MCLK)/8 */
#define CS_CTL1_DIVM__16                         ((uint32_t)0x00040000)          /*!< f(MCLK)/16 */
#define CS_CTL1_DIVM__32                         ((uint32_t)0x00050000)          /*!< f(MCLK)/32 */
#define CS_CTL1_DIVM__64                         ((uint32_t)0x00060000)          /*!< f(MCLK)/64 */
#define CS_CTL1_DIVM__128                        ((uint32_t)0x00070000)          /*!< f(MCLK)/128 */
/* CS_CTL1[DIVHS] Bits */
#define CS_CTL1_DIVHS_OFS                        (20)                            /*!< DIVHS Bit Offset */
#define CS_CTL1_DIVHS_MASK                       ((uint32_t)0x00700000)          /*!< DIVHS Bit Mask */
#define CS_CTL1_DIVHS0                           ((uint32_t)0x00100000)          /*!< DIVHS Bit 0 */
#define CS_CTL1_DIVHS1                           ((uint32_t)0x00200000)          /*!< DIVHS Bit 1 */
#define CS_CTL1_DIVHS2                           ((uint32_t)0x00400000)          /*!< DIVHS Bit 2 */
#define CS_CTL1_DIVHS_0                          ((uint32_t)0x00000000)          /*!< f(HSMCLK)/1 */
#define CS_CTL1_DIVHS_1                          ((uint32_t)0x00100000)          /*!< f(HSMCLK)/2 */
#define CS_CTL1_DIVHS_2                          ((uint32_t)0x00200000)          /*!< f(HSMCLK)/4 */
#define CS_CTL1_DIVHS_3                          ((uint32_t)0x00300000)          /*!< f(HSMCLK)/8 */
#define CS_CTL1_DIVHS_4                          ((uint32_t)0x00400000)          /*!< f(HSMCLK)/16 */
#define CS_CTL1_DIVHS_5                          ((uint32_t)0x00500000)          /*!< f(HSMCLK)/32 */
#define CS_CTL1_DIVHS_6                          ((uint32_t)0x00600000)          /*!< f(HSMCLK)/64 */
#define CS_CTL1_DIVHS_7                          ((uint32_t)0x00700000)          /*!< f(HSMCLK)/128 */
#define CS_CTL1_DIVHS__1                         ((uint32_t)0x00000000)          /*!< f(HSMCLK)/1 */
#define CS_CTL1_DIVHS__2                         ((uint32_t)0x00100000)          /*!< f(HSMCLK)/2 */
#define CS_CTL1_DIVHS__4                         ((uint32_t)0x00200000)          /*!< f(HSMCLK)/4 */
#define CS_CTL1_DIVHS__8                         ((uint32_t)0x00300000)          /*!< f(HSMCLK)/8 */
#define CS_CTL1_DIVHS__16                        ((uint32_t)0x00400000)          /*!< f(HSMCLK)/16 */
#define CS_CTL1_DIVHS__32                        ((uint32_t)0x00500000)          /*!< f(HSMCLK)/32 */
#define CS_CTL1_DIVHS__64                        ((uint32_t)0x00600000)          /*!< f(HSMCLK)/64 */
#define CS_CTL1_DIVHS__128                       ((uint32_t)0x00700000)          /*!< f(HSMCLK)/128 */
/* CS_CTL1[DIVA] Bits */
#define CS_CTL1_DIVA_OFS                         (24)                            /*!< DIVA Bit Offset */
#define CS_CTL1_DIVA_MASK                        ((uint32_t)0x07000000)          /*!< DIVA Bit Mask */
#define CS_CTL1_DIVA0                            ((uint32_t)0x01000000)          /*!< DIVA Bit 0 */
#define CS_CTL1_DIVA1                            ((uint32_t)0x02000000)          /*!< DIVA Bit 1 */
#define CS_CTL1_DIVA2                            ((uint32_t)0x04000000)          /*!< DIVA Bit 2 */
#define CS_CTL1_DIVA_0                           ((uint32_t)0x00000000)          /*!< f(ACLK)/1 */
#define CS_CTL1_DIVA_1                           ((uint32_t)0x01000000)          /*!< f(ACLK)/2 */
#define CS_CTL1_DIVA_2                           ((uint32_t)0x02000000)          /*!< f(ACLK)/4 */
#define CS_CTL1_DIVA_3                           ((uint32_t)0x03000000)          /*!< f(ACLK)/8 */
#define CS_CTL1_DIVA_4                           ((uint32_t)0x04000000)          /*!< f(ACLK)/16 */
#define CS_CTL1_DIVA_5                           ((uint32_t)0x05000000)          /*!< f(ACLK)/32 */
#define CS_CTL1_DIVA_6                           ((uint32_t)0x06000000)          /*!< f(ACLK)/64 */
#define CS_CTL1_DIVA_7                           ((uint32_t)0x07000000)          /*!< f(ACLK)/128 */
#define CS_CTL1_DIVA__1                          ((uint32_t)0x00000000)          /*!< f(ACLK)/1 */
#define CS_CTL1_DIVA__2                          ((uint32_t)0x01000000)          /*!< f(ACLK)/2 */
#define CS_CTL1_DIVA__4                          ((uint32_t)0x02000000)          /*!< f(ACLK)/4 */
#define CS_CTL1_DIVA__8                          ((uint32_t)0x03000000)          /*!< f(ACLK)/8 */
#define CS_CTL1_DIVA__16                         ((uint32_t)0x04000000)          /*!< f(ACLK)/16 */
#define CS_CTL1_DIVA__32                         ((uint32_t)0x05000000)          /*!< f(ACLK)/32 */
#define CS_CTL1_DIVA__64                         ((uint32_t)0x06000000)          /*!< f(ACLK)/64 */
#define CS_CTL1_DIVA__128                        ((uint32_t)0x07000000)          /*!< f(ACLK)/128 */
/* CS_CTL1[DIVS] Bits */
#define CS_CTL1_DIVS_OFS                         (28)                            /*!< DIVS Bit Offset */
#define CS_CTL1_DIVS_MASK                        ((uint32_t)0x70000000)          /*!< DIVS Bit Mask */
#define CS_CTL1_DIVS0                            ((uint32_t)0x10000000)          /*!< DIVS Bit 0 */
#define CS_CTL1_DIVS1                            ((uint32_t)0x20000000)          /*!< DIVS Bit 1 */
#define CS_CTL1_DIVS2                            ((uint32_t)0x40000000)          /*!< DIVS Bit 2 */
#define CS_CTL1_DIVS_0                           ((uint32_t)0x00000000)          /*!< f(SMCLK)/1 */
#define CS_CTL1_DIVS_1                           ((uint32_t)0x10000000)          /*!< f(SMCLK)/2 */
#define CS_CTL1_DIVS_2                           ((uint32_t)0x20000000)          /*!< f(SMCLK)/4 */
#define CS_CTL1_DIVS_3                           ((uint32_t)0x30000000)          /*!< f(SMCLK)/8 */
#define CS_CTL1_DIVS_4                           ((uint32_t)0x40000000)          /*!< f(SMCLK)/16 */
#define CS_CTL1_DIVS_5                           ((uint32_t)0x50000000)          /*!< f(SMCLK)/32 */
#define CS_CTL1_DIVS_6                           ((uint32_t)0x60000000)          /*!< f(SMCLK)/64 */
#define CS_CTL1_DIVS_7                           ((uint32_t)0x70000000)          /*!< f(SMCLK)/128 */
#define CS_CTL1_DIVS__1                          ((uint32_t)0x00000000)          /*!< f(SMCLK)/1 */
#define CS_CTL1_DIVS__2                          ((uint32_t)0x10000000)          /*!< f(SMCLK)/2 */
#define CS_CTL1_DIVS__4                          ((uint32_t)0x20000000)          /*!< f(SMCLK)/4 */
#define CS_CTL1_DIVS__8                          ((uint32_t)0x30000000)          /*!< f(SMCLK)/8 */
#define CS_CTL1_DIVS__16                         ((uint32_t)0x40000000)          /*!< f(SMCLK)/16 */
#define CS_CTL1_DIVS__32                         ((uint32_t)0x50000000)          /*!< f(SMCLK)/32 */
#define CS_CTL1_DIVS__64                         ((uint32_t)0x60000000)          /*!< f(SMCLK)/64 */
#define CS_CTL1_DIVS__128                        ((uint32_t)0x70000000)          /*!< f(SMCLK)/128 */
/* CS_CTL2[LFXTDRIVE] Bits */
#define CS_CTL2_LFXTDRIVE_OFS                    ( 0)                            /*!< LFXTDRIVE Bit Offset */
#define CS_CTL2_LFXTDRIVE_MASK                   ((uint32_t)0x00000003)          /*!< LFXTDRIVE Bit Mask */
#define CS_CTL2_LFXTDRIVE0                       ((uint32_t)0x00000001)          /*!< LFXTDRIVE Bit 0 */
#define CS_CTL2_LFXTDRIVE1                       ((uint32_t)0x00000002)          /*!< LFXTDRIVE Bit 1 */
#define CS_CTL2_LFXTDRIVE_0                      ((uint32_t)0x00000000)          /*!< Lowest drive strength and current consumption LFXT oscillator. */
#define CS_CTL2_LFXTDRIVE_1                      ((uint32_t)0x00000001)          /*!< Increased drive strength LFXT oscillator. */
#define CS_CTL2_LFXTDRIVE_2                      ((uint32_t)0x00000002)          /*!< Increased drive strength LFXT oscillator. */
#define CS_CTL2_LFXTDRIVE_3                      ((uint32_t)0x00000003)          /*!< Maximum drive strength and maximum current consumption LFXT oscillator. */
/* CS_CTL2[LFXTAGCOFF] Bits */
#define CS_CTL2_LFXTAGCOFF_OFS                   ( 7)                            /*!< LFXTAGCOFF Bit Offset */
#define CS_CTL2_LFXTAGCOFF                       ((uint32_t)0x00000080)          /*!< Disables the automatic gain control of the LFXT crystal */
/* CS_CTL2[LFXT_EN] Bits */
#define CS_CTL2_LFXT_EN_OFS                      ( 8)                            /*!< LFXT_EN Bit Offset */
#define CS_CTL2_LFXT_EN                          ((uint32_t)0x00000100)          /*!< Turns on the LFXT oscillator regardless if used as a clock resource */
/* CS_CTL2[LFXTBYPASS] Bits */
#define CS_CTL2_LFXTBYPASS_OFS                   ( 9)                            /*!< LFXTBYPASS Bit Offset */
#define CS_CTL2_LFXTBYPASS                       ((uint32_t)0x00000200)          /*!< LFXT bypass select */
/* CS_CTL2[HFXTDRIVE] Bits */
#define CS_CTL2_HFXTDRIVE_OFS                    (16)                            /*!< HFXTDRIVE Bit Offset */
#define CS_CTL2_HFXTDRIVE                        ((uint32_t)0x00010000)          /*!< HFXT oscillator drive selection */
/* CS_CTL2[HFXTFREQ] Bits */
#define CS_CTL2_HFXTFREQ_OFS                     (20)                            /*!< HFXTFREQ Bit Offset */
#define CS_CTL2_HFXTFREQ_MASK                    ((uint32_t)0x00700000)          /*!< HFXTFREQ Bit Mask */
#define CS_CTL2_HFXTFREQ0                        ((uint32_t)0x00100000)          /*!< HFXTFREQ Bit 0 */
#define CS_CTL2_HFXTFREQ1                        ((uint32_t)0x00200000)          /*!< HFXTFREQ Bit 1 */
#define CS_CTL2_HFXTFREQ2                        ((uint32_t)0x00400000)          /*!< HFXTFREQ Bit 2 */
#define CS_CTL2_HFXTFREQ_0                       ((uint32_t)0x00000000)          /*!< 1 MHz to 4 MHz */
#define CS_CTL2_HFXTFREQ_1                       ((uint32_t)0x00100000)          /*!< >4 MHz to 8 MHz */
#define CS_CTL2_HFXTFREQ_2                       ((uint32_t)0x00200000)          /*!< >8 MHz to 16 MHz */
#define CS_CTL2_HFXTFREQ_3                       ((uint32_t)0x00300000)          /*!< >16 MHz to 24 MHz */
#define CS_CTL2_HFXTFREQ_4                       ((uint32_t)0x00400000)          /*!< >24 MHz to 32 MHz */
#define CS_CTL2_HFXTFREQ_5                       ((uint32_t)0x00500000)          /*!< >32 MHz to 40 MHz */
#define CS_CTL2_HFXTFREQ_6                       ((uint32_t)0x00600000)          /*!< >40 MHz to 48 MHz */
#define CS_CTL2_HFXTFREQ_7                       ((uint32_t)0x00700000)          /*!< Reserved for future use. */
/* CS_CTL2[HFXT_EN] Bits */
#define CS_CTL2_HFXT_EN_OFS                      (24)                            /*!< HFXT_EN Bit Offset */
#define CS_CTL2_HFXT_EN                          ((uint32_t)0x01000000)          /*!< Turns on the HFXT oscillator regardless if used as a clock resource */
/* CS_CTL2[HFXTBYPASS] Bits */
#define CS_CTL2_HFXTBYPASS_OFS                   (25)                            /*!< HFXTBYPASS Bit Offset */
#define CS_CTL2_HFXTBYPASS                       ((uint32_t)0x02000000)          /*!< HFXT bypass select */
/* CS_CTL3[FCNTLF] Bits */
#define CS_CTL3_FCNTLF_OFS                       ( 0)                            /*!< FCNTLF Bit Offset */
#define CS_CTL3_FCNTLF_MASK                      ((uint32_t)0x00000003)          /*!< FCNTLF Bit Mask */
#define CS_CTL3_FCNTLF0                          ((uint32_t)0x00000001)          /*!< FCNTLF Bit 0 */
#define CS_CTL3_FCNTLF1                          ((uint32_t)0x00000002)          /*!< FCNTLF Bit 1 */
#define CS_CTL3_FCNTLF_0                         ((uint32_t)0x00000000)          /*!< 4096 cycles */
#define CS_CTL3_FCNTLF_1                         ((uint32_t)0x00000001)          /*!< 8192 cycles */
#define CS_CTL3_FCNTLF_2                         ((uint32_t)0x00000002)          /*!< 16384 cycles */
#define CS_CTL3_FCNTLF_3                         ((uint32_t)0x00000003)          /*!< 32768 cycles */
#define CS_CTL3_FCNTLF__4096                     ((uint32_t)0x00000000)          /*!< 4096 cycles */
#define CS_CTL3_FCNTLF__8192                     ((uint32_t)0x00000001)          /*!< 8192 cycles */
#define CS_CTL3_FCNTLF__16384                    ((uint32_t)0x00000002)          /*!< 16384 cycles */
#define CS_CTL3_FCNTLF__32768                    ((uint32_t)0x00000003)          /*!< 32768 cycles */
/* CS_CTL3[RFCNTLF] Bits */
#define CS_CTL3_RFCNTLF_OFS                      ( 2)                            /*!< RFCNTLF Bit Offset */
#define CS_CTL3_RFCNTLF                          ((uint32_t)0x00000004)          /*!< Reset start fault counter for LFXT */
/* CS_CTL3[FCNTLF_EN] Bits */
#define CS_CTL3_FCNTLF_EN_OFS                    ( 3)                            /*!< FCNTLF_EN Bit Offset */
#define CS_CTL3_FCNTLF_EN                        ((uint32_t)0x00000008)          /*!< Enable start fault counter for LFXT */
/* CS_CTL3[FCNTHF] Bits */
#define CS_CTL3_FCNTHF_OFS                       ( 4)                            /*!< FCNTHF Bit Offset */
#define CS_CTL3_FCNTHF_MASK                      ((uint32_t)0x00000030)          /*!< FCNTHF Bit Mask */
#define CS_CTL3_FCNTHF0                          ((uint32_t)0x00000010)          /*!< FCNTHF Bit 0 */
#define CS_CTL3_FCNTHF1                          ((uint32_t)0x00000020)          /*!< FCNTHF Bit 1 */
#define CS_CTL3_FCNTHF_0                         ((uint32_t)0x00000000)          /*!< 2048 cycles */
#define CS_CTL3_FCNTHF_1                         ((uint32_t)0x00000010)          /*!< 4096 cycles */
#define CS_CTL3_FCNTHF_2                         ((uint32_t)0x00000020)          /*!< 8192 cycles */
#define CS_CTL3_FCNTHF_3                         ((uint32_t)0x00000030)          /*!< 16384 cycles */
#define CS_CTL3_FCNTHF__2048                     ((uint32_t)0x00000000)          /*!< 2048 cycles */
#define CS_CTL3_FCNTHF__4096                     ((uint32_t)0x00000010)          /*!< 4096 cycles */
#define CS_CTL3_FCNTHF__8192                     ((uint32_t)0x00000020)          /*!< 8192 cycles */
#define CS_CTL3_FCNTHF__16384                    ((uint32_t)0x00000030)          /*!< 16384 cycles */
/* CS_CTL3[RFCNTHF] Bits */
#define CS_CTL3_RFCNTHF_OFS                      ( 6)                            /*!< RFCNTHF Bit Offset */
#define CS_CTL3_RFCNTHF                          ((uint32_t)0x00000040)          /*!< Reset start fault counter for HFXT */
/* CS_CTL3[FCNTHF_EN] Bits */
#define CS_CTL3_FCNTHF_EN_OFS                    ( 7)                            /*!< FCNTHF_EN Bit Offset */
#define CS_CTL3_FCNTHF_EN                        ((uint32_t)0x00000080)          /*!< Enable start fault counter for HFXT */
/* CS_CTL3[FCNTHF2] Bits */
#define CS_CTL3_FCNTHF2_OFS                      ( 8)                            /*!< FCNTHF2 Bit Offset */
#define CS_CTL3_FCNTHF2_MASK                     ((uint32_t)0x00000300)          /*!< FCNTHF2 Bit Mask */
#define CS_CTL3_FCNTHF20                         ((uint32_t)0x00000100)          /*!< FCNTHF2 Bit 0 */
#define CS_CTL3_FCNTHF21                         ((uint32_t)0x00000200)          /*!< FCNTHF2 Bit 1 */
#define CS_CTL3_FCNTHF2_0                        ((uint32_t)0x00000000)          /*!< 2048 cycles */
#define CS_CTL3_FCNTHF2_1                        ((uint32_t)0x00000100)          /*!< 4096 cycles */
#define CS_CTL3_FCNTHF2_2                        ((uint32_t)0x00000200)          /*!< 8192 cycles */
#define CS_CTL3_FCNTHF2_3                        ((uint32_t)0x00000300)          /*!< 16384 cycles */
#define CS_CTL3_FCNTHF2__2048                    ((uint32_t)0x00000000)          /*!< 2048 cycles */
#define CS_CTL3_FCNTHF2__4096                    ((uint32_t)0x00000100)          /*!< 4096 cycles */
#define CS_CTL3_FCNTHF2__8192                    ((uint32_t)0x00000200)          /*!< 8192 cycles */
#define CS_CTL3_FCNTHF2__16384                   ((uint32_t)0x00000300)          /*!< 16384 cycles */
/* CS_CTL3[RFCNTHF2] Bits */
#define CS_CTL3_RFCNTHF2_OFS                     (10)                            /*!< RFCNTHF2 Bit Offset */
#define CS_CTL3_RFCNTHF2                         ((uint32_t)0x00000400)          /*!< Reset start fault counter for HFXT2 */
/* CS_CTL3[FCNTHF2_EN] Bits */
#define CS_CTL3_FCNTHF2_EN_OFS                   (11)                            /*!< FCNTHF2_EN Bit Offset */
#define CS_CTL3_FCNTHF2_EN                       ((uint32_t)0x00000800)          /*!< Enable start fault counter for HFXT2 */
/* CS_CLKEN[ACLK_EN] Bits */
#define CS_CLKEN_ACLK_EN_OFS                     ( 0)                            /*!< ACLK_EN Bit Offset */
#define CS_CLKEN_ACLK_EN                         ((uint32_t)0x00000001)          /*!< ACLK system clock conditional request enable */
/* CS_CLKEN[MCLK_EN] Bits */
#define CS_CLKEN_MCLK_EN_OFS                     ( 1)                            /*!< MCLK_EN Bit Offset */
#define CS_CLKEN_MCLK_EN                         ((uint32_t)0x00000002)          /*!< MCLK system clock conditional request enable */
/* CS_CLKEN[HSMCLK_EN] Bits */
#define CS_CLKEN_HSMCLK_EN_OFS                   ( 2)                            /*!< HSMCLK_EN Bit Offset */
#define CS_CLKEN_HSMCLK_EN                       ((uint32_t)0x00000004)          /*!< HSMCLK system clock conditional request enable */
/* CS_CLKEN[SMCLK_EN] Bits */
#define CS_CLKEN_SMCLK_EN_OFS                    ( 3)                            /*!< SMCLK_EN Bit Offset */
#define CS_CLKEN_SMCLK_EN                        ((uint32_t)0x00000008)          /*!< SMCLK system clock conditional request enable */
/* CS_CLKEN[VLO_EN] Bits */
#define CS_CLKEN_VLO_EN_OFS                      ( 8)                            /*!< VLO_EN Bit Offset */
#define CS_CLKEN_VLO_EN                          ((uint32_t)0x00000100)          /*!< Turns on the VLO oscillator */
/* CS_CLKEN[REFO_EN] Bits */
#define CS_CLKEN_REFO_EN_OFS                     ( 9)                            /*!< REFO_EN Bit Offset */
#define CS_CLKEN_REFO_EN                         ((uint32_t)0x00000200)          /*!< Turns on the REFO oscillator */
/* CS_CLKEN[MODOSC_EN] Bits */
#define CS_CLKEN_MODOSC_EN_OFS                   (10)                            /*!< MODOSC_EN Bit Offset */
#define CS_CLKEN_MODOSC_EN                       ((uint32_t)0x00000400)          /*!< Turns on the MODOSC oscillator */
/* CS_CLKEN[REFOFSEL] Bits */
#define CS_CLKEN_REFOFSEL_OFS                    (15)                            /*!< REFOFSEL Bit Offset */
#define CS_CLKEN_REFOFSEL                        ((uint32_t)0x00008000)          /*!< Selects REFO nominal frequency */
/* CS_STAT[DCO_ON] Bits */
#define CS_STAT_DCO_ON_OFS                       ( 0)                            /*!< DCO_ON Bit Offset */
#define CS_STAT_DCO_ON                           ((uint32_t)0x00000001)          /*!< DCO status */
/* CS_STAT[DCOBIAS_ON] Bits */
#define CS_STAT_DCOBIAS_ON_OFS                   ( 1)                            /*!< DCOBIAS_ON Bit Offset */
#define CS_STAT_DCOBIAS_ON                       ((uint32_t)0x00000002)          /*!< DCO bias status */
/* CS_STAT[HFXT_ON] Bits */
#define CS_STAT_HFXT_ON_OFS                      ( 2)                            /*!< HFXT_ON Bit Offset */
#define CS_STAT_HFXT_ON                          ((uint32_t)0x00000004)          /*!< HFXT status */
/* CS_STAT[HFXT2_ON] Bits */
#define CS_STAT_HFXT2_ON_OFS                     ( 3)                            /*!< HFXT2_ON Bit Offset */
#define CS_STAT_HFXT2_ON                         ((uint32_t)0x00000008)          /*!< HFXT2 status */
/* CS_STAT[MODOSC_ON] Bits */
#define CS_STAT_MODOSC_ON_OFS                    ( 4)                            /*!< MODOSC_ON Bit Offset */
#define CS_STAT_MODOSC_ON                        ((uint32_t)0x00000010)          /*!< MODOSC status */
/* CS_STAT[VLO_ON] Bits */
#define CS_STAT_VLO_ON_OFS                       ( 5)                            /*!< VLO_ON Bit Offset */
#define CS_STAT_VLO_ON                           ((uint32_t)0x00000020)          /*!< VLO status */
/* CS_STAT[LFXT_ON] Bits */
#define CS_STAT_LFXT_ON_OFS                      ( 6)                            /*!< LFXT_ON Bit Offset */
#define CS_STAT_LFXT_ON                          ((uint32_t)0x00000040)          /*!< LFXT status */
/* CS_STAT[REFO_ON] Bits */
#define CS_STAT_REFO_ON_OFS                      ( 7)                            /*!< REFO_ON Bit Offset */
#define CS_STAT_REFO_ON                          ((uint32_t)0x00000080)          /*!< REFO status */
/* CS_STAT[ACLK_ON] Bits */
#define CS_STAT_ACLK_ON_OFS                      (16)                            /*!< ACLK_ON Bit Offset */
#define CS_STAT_ACLK_ON                          ((uint32_t)0x00010000)          /*!< ACLK system clock status */
/* CS_STAT[MCLK_ON] Bits */
#define CS_STAT_MCLK_ON_OFS                      (17)                            /*!< MCLK_ON Bit Offset */
#define CS_STAT_MCLK_ON                          ((uint32_t)0x00020000)          /*!< MCLK system clock status */
/* CS_STAT[HSMCLK_ON] Bits */
#define CS_STAT_HSMCLK_ON_OFS                    (18)                            /*!< HSMCLK_ON Bit Offset */
#define CS_STAT_HSMCLK_ON                        ((uint32_t)0x00040000)          /*!< HSMCLK system clock status */
/* CS_STAT[SMCLK_ON] Bits */
#define CS_STAT_SMCLK_ON_OFS                     (19)                            /*!< SMCLK_ON Bit Offset */
#define CS_STAT_SMCLK_ON                         ((uint32_t)0x00080000)          /*!< SMCLK system clock status */
/* CS_STAT[MODCLK_ON] Bits */
#define CS_STAT_MODCLK_ON_OFS                    (20)                            /*!< MODCLK_ON Bit Offset */
#define CS_STAT_MODCLK_ON                        ((uint32_t)0x00100000)          /*!< MODCLK system clock status */
/* CS_STAT[VLOCLK_ON] Bits */
#define CS_STAT_VLOCLK_ON_OFS                    (21)                            /*!< VLOCLK_ON Bit Offset */
#define CS_STAT_VLOCLK_ON                        ((uint32_t)0x00200000)          /*!< VLOCLK system clock status */
/* CS_STAT[LFXTCLK_ON] Bits */
#define CS_STAT_LFXTCLK_ON_OFS                   (22)                            /*!< LFXTCLK_ON Bit Offset */
#define CS_STAT_LFXTCLK_ON                       ((uint32_t)0x00400000)          /*!< LFXTCLK system clock status */
/* CS_STAT[REFOCLK_ON] Bits */
#define CS_STAT_REFOCLK_ON_OFS                   (23)                            /*!< REFOCLK_ON Bit Offset */
#define CS_STAT_REFOCLK_ON                       ((uint32_t)0x00800000)          /*!< REFOCLK system clock status */
/* CS_STAT[ACLK_READY] Bits */
#define CS_STAT_ACLK_READY_OFS                   (24)                            /*!< ACLK_READY Bit Offset */
#define CS_STAT_ACLK_READY                       ((uint32_t)0x01000000)          /*!< ACLK Ready status */
/* CS_STAT[MCLK_READY] Bits */
#define CS_STAT_MCLK_READY_OFS                   (25)                            /*!< MCLK_READY Bit Offset */
#define CS_STAT_MCLK_READY                       ((uint32_t)0x02000000)          /*!< MCLK Ready status */
/* CS_STAT[HSMCLK_READY] Bits */
#define CS_STAT_HSMCLK_READY_OFS                 (26)                            /*!< HSMCLK_READY Bit Offset */
#define CS_STAT_HSMCLK_READY                     ((uint32_t)0x04000000)          /*!< HSMCLK Ready status */
/* CS_STAT[SMCLK_READY] Bits */
#define CS_STAT_SMCLK_READY_OFS                  (27)                            /*!< SMCLK_READY Bit Offset */
#define CS_STAT_SMCLK_READY                      ((uint32_t)0x08000000)          /*!< SMCLK Ready status */
/* CS_STAT[BCLK_READY] Bits */
#define CS_STAT_BCLK_READY_OFS                   (28)                            /*!< BCLK_READY Bit Offset */
#define CS_STAT_BCLK_READY                       ((uint32_t)0x10000000)          /*!< BCLK Ready status */
/* CS_IE[LFXTIE] Bits */
#define CS_IE_LFXTIE_OFS                         ( 0)                            /*!< LFXTIE Bit Offset */
#define CS_IE_LFXTIE                             ((uint32_t)0x00000001)          /*!< LFXT oscillator fault flag interrupt enable */
/* CS_IE[HFXTIE] Bits */
#define CS_IE_HFXTIE_OFS                         ( 1)                            /*!< HFXTIE Bit Offset */
#define CS_IE_HFXTIE                             ((uint32_t)0x00000002)          /*!< HFXT oscillator fault flag interrupt enable */
/* CS_IE[HFXT2IE] Bits */
#define CS_IE_HFXT2IE_OFS                        ( 2)                            /*!< HFXT2IE Bit Offset */
#define CS_IE_HFXT2IE                            ((uint32_t)0x00000004)          /*!< HFXT2 oscillator fault flag interrupt enable */
/* CS_IE[DCOR_OPNIE] Bits */
#define CS_IE_DCOR_OPNIE_OFS                     ( 6)                            /*!< DCOR_OPNIE Bit Offset */
#define CS_IE_DCOR_OPNIE                         ((uint32_t)0x00000040)          /*!< DCO external resistor open circuit fault flag interrupt enable. */
/* CS_IE[FCNTLFIE] Bits */
#define CS_IE_FCNTLFIE_OFS                       ( 8)                            /*!< FCNTLFIE Bit Offset */
#define CS_IE_FCNTLFIE                           ((uint32_t)0x00000100)          /*!< Start fault counter interrupt enable LFXT */
/* CS_IE[FCNTHFIE] Bits */
#define CS_IE_FCNTHFIE_OFS                       ( 9)                            /*!< FCNTHFIE Bit Offset */
#define CS_IE_FCNTHFIE                           ((uint32_t)0x00000200)          /*!< Start fault counter interrupt enable HFXT */
/* CS_IE[FCNTHF2IE] Bits */
#define CS_IE_FCNTHF2IE_OFS                      (10)                            /*!< FCNTHF2IE Bit Offset */
#define CS_IE_FCNTHF2IE                          ((uint32_t)0x00000400)          /*!< Start fault counter interrupt enable HFXT2 */
/* CS_IE[PLLOOLIE] Bits */
#define CS_IE_PLLOOLIE_OFS                       (12)                            /*!< PLLOOLIE Bit Offset */
#define CS_IE_PLLOOLIE                           ((uint32_t)0x00001000)          /*!< PLL out-of-lock interrupt enable */
/* CS_IE[PLLLOSIE] Bits */
#define CS_IE_PLLLOSIE_OFS                       (13)                            /*!< PLLLOSIE Bit Offset */
#define CS_IE_PLLLOSIE                           ((uint32_t)0x00002000)          /*!< PLL loss-of-signal interrupt enable */
/* CS_IE[PLLOORIE] Bits */
#define CS_IE_PLLOORIE_OFS                       (14)                            /*!< PLLOORIE Bit Offset */
#define CS_IE_PLLOORIE                           ((uint32_t)0x00004000)          /*!< PLL out-of-range interrupt enable */
/* CS_IE[CALIE] Bits */
#define CS_IE_CALIE_OFS                          (15)                            /*!< CALIE Bit Offset */
#define CS_IE_CALIE                              ((uint32_t)0x00008000)          /*!< REFCNT period counter interrupt enable */
/* CS_IFG[LFXTIFG] Bits */
#define CS_IFG_LFXTIFG_OFS                       ( 0)                            /*!< LFXTIFG Bit Offset */
#define CS_IFG_LFXTIFG                           ((uint32_t)0x00000001)          /*!< LFXT oscillator fault flag */
/* CS_IFG[HFXTIFG] Bits */
#define CS_IFG_HFXTIFG_OFS                       ( 1)                            /*!< HFXTIFG Bit Offset */
#define CS_IFG_HFXTIFG                           ((uint32_t)0x00000002)          /*!< HFXT oscillator fault flag */
/* CS_IFG[HFXT2IFG] Bits */
#define CS_IFG_HFXT2IFG_OFS                      ( 2)                            /*!< HFXT2IFG Bit Offset */
#define CS_IFG_HFXT2IFG                          ((uint32_t)0x00000004)          /*!< HFXT2 oscillator fault flag */
/* CS_IFG[DCOR_SHTIFG] Bits */
#define CS_IFG_DCOR_SHTIFG_OFS                   ( 5)                            /*!< DCOR_SHTIFG Bit Offset */
#define CS_IFG_DCOR_SHTIFG                       ((uint32_t)0x00000020)          /*!< DCO external resistor short circuit fault flag. */
/* CS_IFG[DCOR_OPNIFG] Bits */
#define CS_IFG_DCOR_OPNIFG_OFS                   ( 6)                            /*!< DCOR_OPNIFG Bit Offset */
#define CS_IFG_DCOR_OPNIFG                       ((uint32_t)0x00000040)          /*!< DCO external resistor open circuit fault flag. */
/* CS_IFG[FCNTLFIFG] Bits */
#define CS_IFG_FCNTLFIFG_OFS                     ( 8)                            /*!< FCNTLFIFG Bit Offset */
#define CS_IFG_FCNTLFIFG                         ((uint32_t)0x00000100)          /*!< Start fault counter interrupt flag LFXT */
/* CS_IFG[FCNTHFIFG] Bits */
#define CS_IFG_FCNTHFIFG_OFS                     ( 9)                            /*!< FCNTHFIFG Bit Offset */
#define CS_IFG_FCNTHFIFG                         ((uint32_t)0x00000200)          /*!< Start fault counter interrupt flag HFXT */
/* CS_IFG[FCNTHF2IFG] Bits */
#define CS_IFG_FCNTHF2IFG_OFS                    (11)                            /*!< FCNTHF2IFG Bit Offset */
#define CS_IFG_FCNTHF2IFG                        ((uint32_t)0x00000800)          /*!< Start fault counter interrupt flag HFXT2 */
/* CS_IFG[PLLOOLIFG] Bits */
#define CS_IFG_PLLOOLIFG_OFS                     (12)                            /*!< PLLOOLIFG Bit Offset */
#define CS_IFG_PLLOOLIFG                         ((uint32_t)0x00001000)          /*!< PLL out-of-lock interrupt flag */
/* CS_IFG[PLLLOSIFG] Bits */
#define CS_IFG_PLLLOSIFG_OFS                     (13)                            /*!< PLLLOSIFG Bit Offset */
#define CS_IFG_PLLLOSIFG                         ((uint32_t)0x00002000)          /*!< PLL loss-of-signal interrupt flag */
/* CS_IFG[PLLOORIFG] Bits */
#define CS_IFG_PLLOORIFG_OFS                     (14)                            /*!< PLLOORIFG Bit Offset */
#define CS_IFG_PLLOORIFG                         ((uint32_t)0x00004000)          /*!< PLL out-of-range interrupt flag */
/* CS_IFG[CALIFG] Bits */
#define CS_IFG_CALIFG_OFS                        (15)                            /*!< CALIFG Bit Offset */
#define CS_IFG_CALIFG                            ((uint32_t)0x00008000)          /*!< REFCNT period counter expired */
/* CS_CLRIFG[CLR_LFXTIFG] Bits */
#define CS_CLRIFG_CLR_LFXTIFG_OFS                ( 0)                            /*!< CLR_LFXTIFG Bit Offset */
#define CS_CLRIFG_CLR_LFXTIFG                    ((uint32_t)0x00000001)          /*!< Clear LFXT oscillator fault interrupt flag */
/* CS_CLRIFG[CLR_HFXTIFG] Bits */
#define CS_CLRIFG_CLR_HFXTIFG_OFS                ( 1)                            /*!< CLR_HFXTIFG Bit Offset */
#define CS_CLRIFG_CLR_HFXTIFG                    ((uint32_t)0x00000002)          /*!< Clear HFXT oscillator fault interrupt flag */
/* CS_CLRIFG[CLR_HFXT2IFG] Bits */
#define CS_CLRIFG_CLR_HFXT2IFG_OFS               ( 2)                            /*!< CLR_HFXT2IFG Bit Offset */
#define CS_CLRIFG_CLR_HFXT2IFG                   ((uint32_t)0x00000004)          /*!< Clear HFXT2 oscillator fault interrupt flag */
/* CS_CLRIFG[CLR_DCOR_OPNIFG] Bits */
#define CS_CLRIFG_CLR_DCOR_OPNIFG_OFS            ( 6)                            /*!< CLR_DCOR_OPNIFG Bit Offset */
#define CS_CLRIFG_CLR_DCOR_OPNIFG                ((uint32_t)0x00000040)          /*!< Clear DCO external resistor open circuit fault interrupt flag. */
/* CS_CLRIFG[CLR_CALIFG] Bits */
#define CS_CLRIFG_CLR_CALIFG_OFS                 (15)                            /*!< CLR_CALIFG Bit Offset */
#define CS_CLRIFG_CLR_CALIFG                     ((uint32_t)0x00008000)          /*!< REFCNT period counter clear interrupt flag */
/* CS_CLRIFG[CLR_FCNTLFIFG] Bits */
#define CS_CLRIFG_CLR_FCNTLFIFG_OFS              ( 8)                            /*!< CLR_FCNTLFIFG Bit Offset */
#define CS_CLRIFG_CLR_FCNTLFIFG                  ((uint32_t)0x00000100)          /*!< Start fault counter clear interrupt flag LFXT */
/* CS_CLRIFG[CLR_FCNTHFIFG] Bits */
#define CS_CLRIFG_CLR_FCNTHFIFG_OFS              ( 9)                            /*!< CLR_FCNTHFIFG Bit Offset */
#define CS_CLRIFG_CLR_FCNTHFIFG                  ((uint32_t)0x00000200)          /*!< Start fault counter clear interrupt flag HFXT */
/* CS_CLRIFG[CLR_FCNTHF2IFG] Bits */
#define CS_CLRIFG_CLR_FCNTHF2IFG_OFS             (10)                            /*!< CLR_FCNTHF2IFG Bit Offset */
#define CS_CLRIFG_CLR_FCNTHF2IFG                 ((uint32_t)0x00000400)          /*!< Start fault counter clear interrupt flag HFXT2 */
/* CS_CLRIFG[CLR_PLLOOLIFG] Bits */
#define CS_CLRIFG_CLR_PLLOOLIFG_OFS              (12)                            /*!< CLR_PLLOOLIFG Bit Offset */
#define CS_CLRIFG_CLR_PLLOOLIFG                  ((uint32_t)0x00001000)          /*!< PLL out-of-lock clear interrupt flag */
/* CS_CLRIFG[CLR_PLLLOSIFG] Bits */
#define CS_CLRIFG_CLR_PLLLOSIFG_OFS              (13)                            /*!< CLR_PLLLOSIFG Bit Offset */
#define CS_CLRIFG_CLR_PLLLOSIFG                  ((uint32_t)0x00002000)          /*!< PLL loss-of-signal clear interrupt flag */
/* CS_CLRIFG[CLR_PLLOORIFG] Bits */
#define CS_CLRIFG_CLR_PLLOORIFG_OFS              (14)                            /*!< CLR_PLLOORIFG Bit Offset */
#define CS_CLRIFG_CLR_PLLOORIFG                  ((uint32_t)0x00004000)          /*!< PLL out-of-range clear interrupt flag */
/* CS_SETIFG[SET_LFXTIFG] Bits */
#define CS_SETIFG_SET_LFXTIFG_OFS                ( 0)                            /*!< SET_LFXTIFG Bit Offset */
#define CS_SETIFG_SET_LFXTIFG                    ((uint32_t)0x00000001)          /*!< Set LFXT oscillator fault interrupt flag */
/* CS_SETIFG[SET_HFXTIFG] Bits */
#define CS_SETIFG_SET_HFXTIFG_OFS                ( 1)                            /*!< SET_HFXTIFG Bit Offset */
#define CS_SETIFG_SET_HFXTIFG                    ((uint32_t)0x00000002)          /*!< Set HFXT oscillator fault interrupt flag */
/* CS_SETIFG[SET_HFXT2IFG] Bits */
#define CS_SETIFG_SET_HFXT2IFG_OFS               ( 2)                            /*!< SET_HFXT2IFG Bit Offset */
#define CS_SETIFG_SET_HFXT2IFG                   ((uint32_t)0x00000004)          /*!< Set HFXT2 oscillator fault interrupt flag */
/* CS_SETIFG[SET_DCOR_OPNIFG] Bits */
#define CS_SETIFG_SET_DCOR_OPNIFG_OFS            ( 6)                            /*!< SET_DCOR_OPNIFG Bit Offset */
#define CS_SETIFG_SET_DCOR_OPNIFG                ((uint32_t)0x00000040)          /*!< Set DCO external resistor open circuit fault interrupt flag. */
/* CS_SETIFG[SET_CALIFG] Bits */
#define CS_SETIFG_SET_CALIFG_OFS                 (15)                            /*!< SET_CALIFG Bit Offset */
#define CS_SETIFG_SET_CALIFG                     ((uint32_t)0x00008000)          /*!< REFCNT period counter set interrupt flag */
/* CS_SETIFG[SET_FCNTHFIFG] Bits */
#define CS_SETIFG_SET_FCNTHFIFG_OFS              ( 9)                            /*!< SET_FCNTHFIFG Bit Offset */
#define CS_SETIFG_SET_FCNTHFIFG                  ((uint32_t)0x00000200)          /*!< Start fault counter set interrupt flag HFXT */
/* CS_SETIFG[SET_FCNTHF2IFG] Bits */
#define CS_SETIFG_SET_FCNTHF2IFG_OFS             (10)                            /*!< SET_FCNTHF2IFG Bit Offset */
#define CS_SETIFG_SET_FCNTHF2IFG                 ((uint32_t)0x00000400)          /*!< Start fault counter set interrupt flag HFXT2 */
/* CS_SETIFG[SET_FCNTLFIFG] Bits */
#define CS_SETIFG_SET_FCNTLFIFG_OFS              ( 8)                            /*!< SET_FCNTLFIFG Bit Offset */
#define CS_SETIFG_SET_FCNTLFIFG                  ((uint32_t)0x00000100)          /*!< Start fault counter set interrupt flag LFXT */
/* CS_SETIFG[SET_PLLOOLIFG] Bits */
#define CS_SETIFG_SET_PLLOOLIFG_OFS              (12)                            /*!< SET_PLLOOLIFG Bit Offset */
#define CS_SETIFG_SET_PLLOOLIFG                  ((uint32_t)0x00001000)          /*!< PLL out-of-lock set interrupt flag */
/* CS_SETIFG[SET_PLLLOSIFG] Bits */
#define CS_SETIFG_SET_PLLLOSIFG_OFS              (13)                            /*!< SET_PLLLOSIFG Bit Offset */
#define CS_SETIFG_SET_PLLLOSIFG                  ((uint32_t)0x00002000)          /*!< PLL loss-of-signal set interrupt flag */
/* CS_SETIFG[SET_PLLOORIFG] Bits */
#define CS_SETIFG_SET_PLLOORIFG_OFS              (14)                            /*!< SET_PLLOORIFG Bit Offset */
#define CS_SETIFG_SET_PLLOORIFG                  ((uint32_t)0x00004000)          /*!< PLL out-of-range set interrupt flag */
/* CS_DCOERCAL0[DCO_TCCAL] Bits */
#define CS_DCOERCAL0_DCO_TCCAL_OFS               ( 0)                            /*!< DCO_TCCAL Bit Offset */
#define CS_DCOERCAL0_DCO_TCCAL_MASK              ((uint32_t)0x00000003)          /*!< DCO_TCCAL Bit Mask */
/* CS_DCOERCAL0[DCO_FCAL_RSEL04] Bits */
#define CS_DCOERCAL0_DCO_FCAL_RSEL04_OFS         (16)                            /*!< DCO_FCAL_RSEL04 Bit Offset */
#define CS_DCOERCAL0_DCO_FCAL_RSEL04_MASK        ((uint32_t)0x03FF0000)          /*!< DCO_FCAL_RSEL04 Bit Mask */
/* CS_DCOERCAL1[DCO_FCAL_RSEL5] Bits */
#define CS_DCOERCAL1_DCO_FCAL_RSEL5_OFS          ( 0)                            /*!< DCO_FCAL_RSEL5 Bit Offset */
#define CS_DCOERCAL1_DCO_FCAL_RSEL5_MASK         ((uint32_t)0x000003FF)          /*!< DCO_FCAL_RSEL5 Bit Mask */

/* Pre-defined bitfield values */
#define CS_KEY_VAL                               ((uint32_t)0x0000695A)          /*!< CS control key value */

/******************************************************************************
* DIO Bits
******************************************************************************/
/* DIO_IV[IV] Bits */
#define DIO_PORT_IV_OFS                          ( 0)                            /*!< DIO Port IV Bit Offset */
#define DIO_PORT_IV_MASK                         ((uint16_t)0x001F)              /*!< DIO Port IV Bit Mask */
#define DIO_PORT_IV0                             ((uint16_t)0x0001)              /*!< DIO Port IV Bit 0 */
#define DIO_PORT_IV1                             ((uint16_t)0x0002)              /*!< DIO Port IV Bit 1 */
#define DIO_PORT_IV2                             ((uint16_t)0x0004)              /*!< DIO Port IV Bit 2 */
#define DIO_PORT_IV3                             ((uint16_t)0x0008)              /*!< DIO Port IV Bit 3 */
#define DIO_PORT_IV4                             ((uint16_t)0x0010)              /*!< DIO Port IV Bit 4 */
#define DIO_PORT_IV_0                            ((uint16_t)0x0000)              /*!< No interrupt pending */
#define DIO_PORT_IV_2                            ((uint16_t)0x0002)              /*!< Interrupt Source: Port x.0 interrupt; Interrupt Flag: IFG0; Interrupt  */
                                                                                 /* Priority: Highest */
#define DIO_PORT_IV_4                            ((uint16_t)0x0004)              /*!< Interrupt Source: Port x.1 interrupt; Interrupt Flag: IFG1 */
#define DIO_PORT_IV_6                            ((uint16_t)0x0006)              /*!< Interrupt Source: Port x.2 interrupt; Interrupt Flag: IFG2 */
#define DIO_PORT_IV_8                            ((uint16_t)0x0008)              /*!< Interrupt Source: Port x.3 interrupt; Interrupt Flag: IFG3 */
#define DIO_PORT_IV_10                           ((uint16_t)0x000A)              /*!< Interrupt Source: Port x.4 interrupt; Interrupt Flag: IFG4 */
#define DIO_PORT_IV_12                           ((uint16_t)0x000C)              /*!< Interrupt Source: Port x.5 interrupt; Interrupt Flag: IFG5 */
#define DIO_PORT_IV_14                           ((uint16_t)0x000E)              /*!< Interrupt Source: Port x.6 interrupt; Interrupt Flag: IFG6 */
#define DIO_PORT_IV_16                           ((uint16_t)0x0010)              /*!< Interrupt Source: Port x.7 interrupt; Interrupt Flag: IFG7; Interrupt  */
                                                                                 /* Priority: Lowest */
#define DIO_PORT_IV__NONE                        ((uint16_t)0x0000)              /*!< No interrupt pending */
#define DIO_PORT_IV__IFG0                        ((uint16_t)0x0002)              /*!< Interrupt Source: Port x.0 interrupt; Interrupt Flag: IFG0; Interrupt  */
                                                                                 /* Priority: Highest */
#define DIO_PORT_IV__IFG1                        ((uint16_t)0x0004)              /*!< Interrupt Source: Port x.1 interrupt; Interrupt Flag: IFG1 */
#define DIO_PORT_IV__IFG2                        ((uint16_t)0x0006)              /*!< Interrupt Source: Port x.2 interrupt; Interrupt Flag: IFG2 */
#define DIO_PORT_IV__IFG3                        ((uint16_t)0x0008)              /*!< Interrupt Source: Port x.3 interrupt; Interrupt Flag: IFG3 */
#define DIO_PORT_IV__IFG4                        ((uint16_t)0x000A)              /*!< Interrupt Source: Port x.4 interrupt; Interrupt Flag: IFG4 */
#define DIO_PORT_IV__IFG5                        ((uint16_t)0x000C)              /*!< Interrupt Source: Port x.5 interrupt; Interrupt Flag: IFG5 */
#define DIO_PORT_IV__IFG6                        ((uint16_t)0x000E)              /*!< Interrupt Source: Port x.6 interrupt; Interrupt Flag: IFG6 */
#define DIO_PORT_IV__IFG7                        ((uint16_t)0x0010)              /*!< Interrupt Source: Port x.7 interrupt; Interrupt Flag: IFG7; Interrupt  */
                                                                                 /* Priority: Lowest */


/******************************************************************************
* DMA Bits
******************************************************************************/
/* DMA_DEVICE_CFG[NUM_DMA_CHANNELS] Bits */
#define DMA_DEVICE_CFG_NUM_DMA_CHANNELS_OFS      ( 0)                            /*!< NUM_DMA_CHANNELS Bit Offset */
#define DMA_DEVICE_CFG_NUM_DMA_CHANNELS_MASK     ((uint32_t)0x000000FF)          /*!< NUM_DMA_CHANNELS Bit Mask */
/* DMA_DEVICE_CFG[NUM_SRC_PER_CHANNEL] Bits */
#define DMA_DEVICE_CFG_NUM_SRC_PER_CHANNEL_OFS   ( 8)                            /*!< NUM_SRC_PER_CHANNEL Bit Offset */
#define DMA_DEVICE_CFG_NUM_SRC_PER_CHANNEL_MASK  ((uint32_t)0x0000FF00)          /*!< NUM_SRC_PER_CHANNEL Bit Mask */
/* DMA_SW_CHTRIG[CH0] Bits */
#define DMA_SW_CHTRIG_CH0_OFS                    ( 0)                            /*!< CH0 Bit Offset */
#define DMA_SW_CHTRIG_CH0                        ((uint32_t)0x00000001)          /*!< Write 1, triggers DMA_CHANNEL0 */
/* DMA_SW_CHTRIG[CH1] Bits */
#define DMA_SW_CHTRIG_CH1_OFS                    ( 1)                            /*!< CH1 Bit Offset */
#define DMA_SW_CHTRIG_CH1                        ((uint32_t)0x00000002)          /*!< Write 1, triggers DMA_CHANNEL1 */
/* DMA_SW_CHTRIG[CH2] Bits */
#define DMA_SW_CHTRIG_CH2_OFS                    ( 2)                            /*!< CH2 Bit Offset */
#define DMA_SW_CHTRIG_CH2                        ((uint32_t)0x00000004)          /*!< Write 1, triggers DMA_CHANNEL2 */
/* DMA_SW_CHTRIG[CH3] Bits */
#define DMA_SW_CHTRIG_CH3_OFS                    ( 3)                            /*!< CH3 Bit Offset */
#define DMA_SW_CHTRIG_CH3                        ((uint32_t)0x00000008)          /*!< Write 1, triggers DMA_CHANNEL3 */
/* DMA_SW_CHTRIG[CH4] Bits */
#define DMA_SW_CHTRIG_CH4_OFS                    ( 4)                            /*!< CH4 Bit Offset */
#define DMA_SW_CHTRIG_CH4                        ((uint32_t)0x00000010)          /*!< Write 1, triggers DMA_CHANNEL4 */
/* DMA_SW_CHTRIG[CH5] Bits */
#define DMA_SW_CHTRIG_CH5_OFS                    ( 5)                            /*!< CH5 Bit Offset */
#define DMA_SW_CHTRIG_CH5                        ((uint32_t)0x00000020)          /*!< Write 1, triggers DMA_CHANNEL5 */
/* DMA_SW_CHTRIG[CH6] Bits */
#define DMA_SW_CHTRIG_CH6_OFS                    ( 6)                            /*!< CH6 Bit Offset */
#define DMA_SW_CHTRIG_CH6                        ((uint32_t)0x00000040)          /*!< Write 1, triggers DMA_CHANNEL6 */
/* DMA_SW_CHTRIG[CH7] Bits */
#define DMA_SW_CHTRIG_CH7_OFS                    ( 7)                            /*!< CH7 Bit Offset */
#define DMA_SW_CHTRIG_CH7                        ((uint32_t)0x00000080)          /*!< Write 1, triggers DMA_CHANNEL7 */
/* DMA_SW_CHTRIG[CH8] Bits */
#define DMA_SW_CHTRIG_CH8_OFS                    ( 8)                            /*!< CH8 Bit Offset */
#define DMA_SW_CHTRIG_CH8                        ((uint32_t)0x00000100)          /*!< Write 1, triggers DMA_CHANNEL8 */
/* DMA_SW_CHTRIG[CH9] Bits */
#define DMA_SW_CHTRIG_CH9_OFS                    ( 9)                            /*!< CH9 Bit Offset */
#define DMA_SW_CHTRIG_CH9                        ((uint32_t)0x00000200)          /*!< Write 1, triggers DMA_CHANNEL9 */
/* DMA_SW_CHTRIG[CH10] Bits */
#define DMA_SW_CHTRIG_CH10_OFS                   (10)                            /*!< CH10 Bit Offset */
#define DMA_SW_CHTRIG_CH10                       ((uint32_t)0x00000400)          /*!< Write 1, triggers DMA_CHANNEL10 */
/* DMA_SW_CHTRIG[CH11] Bits */
#define DMA_SW_CHTRIG_CH11_OFS                   (11)                            /*!< CH11 Bit Offset */
#define DMA_SW_CHTRIG_CH11                       ((uint32_t)0x00000800)          /*!< Write 1, triggers DMA_CHANNEL11 */
/* DMA_SW_CHTRIG[CH12] Bits */
#define DMA_SW_CHTRIG_CH12_OFS                   (12)                            /*!< CH12 Bit Offset */
#define DMA_SW_CHTRIG_CH12                       ((uint32_t)0x00001000)          /*!< Write 1, triggers DMA_CHANNEL12 */
/* DMA_SW_CHTRIG[CH13] Bits */
#define DMA_SW_CHTRIG_CH13_OFS                   (13)                            /*!< CH13 Bit Offset */
#define DMA_SW_CHTRIG_CH13                       ((uint32_t)0x00002000)          /*!< Write 1, triggers DMA_CHANNEL13 */
/* DMA_SW_CHTRIG[CH14] Bits */
#define DMA_SW_CHTRIG_CH14_OFS                   (14)                            /*!< CH14 Bit Offset */
#define DMA_SW_CHTRIG_CH14                       ((uint32_t)0x00004000)          /*!< Write 1, triggers DMA_CHANNEL14 */
/* DMA_SW_CHTRIG[CH15] Bits */
#define DMA_SW_CHTRIG_CH15_OFS                   (15)                            /*!< CH15 Bit Offset */
#define DMA_SW_CHTRIG_CH15                       ((uint32_t)0x00008000)          /*!< Write 1, triggers DMA_CHANNEL15 */
/* DMA_SW_CHTRIG[CH16] Bits */
#define DMA_SW_CHTRIG_CH16_OFS                   (16)                            /*!< CH16 Bit Offset */
#define DMA_SW_CHTRIG_CH16                       ((uint32_t)0x00010000)          /*!< Write 1, triggers DMA_CHANNEL16 */
/* DMA_SW_CHTRIG[CH17] Bits */
#define DMA_SW_CHTRIG_CH17_OFS                   (17)                            /*!< CH17 Bit Offset */
#define DMA_SW_CHTRIG_CH17                       ((uint32_t)0x00020000)          /*!< Write 1, triggers DMA_CHANNEL17 */
/* DMA_SW_CHTRIG[CH18] Bits */
#define DMA_SW_CHTRIG_CH18_OFS                   (18)                            /*!< CH18 Bit Offset */
#define DMA_SW_CHTRIG_CH18                       ((uint32_t)0x00040000)          /*!< Write 1, triggers DMA_CHANNEL18 */
/* DMA_SW_CHTRIG[CH19] Bits */
#define DMA_SW_CHTRIG_CH19_OFS                   (19)                            /*!< CH19 Bit Offset */
#define DMA_SW_CHTRIG_CH19                       ((uint32_t)0x00080000)          /*!< Write 1, triggers DMA_CHANNEL19 */
/* DMA_SW_CHTRIG[CH20] Bits */
#define DMA_SW_CHTRIG_CH20_OFS                   (20)                            /*!< CH20 Bit Offset */
#define DMA_SW_CHTRIG_CH20                       ((uint32_t)0x00100000)          /*!< Write 1, triggers DMA_CHANNEL20 */
/* DMA_SW_CHTRIG[CH21] Bits */
#define DMA_SW_CHTRIG_CH21_OFS                   (21)                            /*!< CH21 Bit Offset */
#define DMA_SW_CHTRIG_CH21                       ((uint32_t)0x00200000)          /*!< Write 1, triggers DMA_CHANNEL21 */
/* DMA_SW_CHTRIG[CH22] Bits */
#define DMA_SW_CHTRIG_CH22_OFS                   (22)                            /*!< CH22 Bit Offset */
#define DMA_SW_CHTRIG_CH22                       ((uint32_t)0x00400000)          /*!< Write 1, triggers DMA_CHANNEL22 */
/* DMA_SW_CHTRIG[CH23] Bits */
#define DMA_SW_CHTRIG_CH23_OFS                   (23)                            /*!< CH23 Bit Offset */
#define DMA_SW_CHTRIG_CH23                       ((uint32_t)0x00800000)          /*!< Write 1, triggers DMA_CHANNEL23 */
/* DMA_SW_CHTRIG[CH24] Bits */
#define DMA_SW_CHTRIG_CH24_OFS                   (24)                            /*!< CH24 Bit Offset */
#define DMA_SW_CHTRIG_CH24                       ((uint32_t)0x01000000)          /*!< Write 1, triggers DMA_CHANNEL24 */
/* DMA_SW_CHTRIG[CH25] Bits */
#define DMA_SW_CHTRIG_CH25_OFS                   (25)                            /*!< CH25 Bit Offset */
#define DMA_SW_CHTRIG_CH25                       ((uint32_t)0x02000000)          /*!< Write 1, triggers DMA_CHANNEL25 */
/* DMA_SW_CHTRIG[CH26] Bits */
#define DMA_SW_CHTRIG_CH26_OFS                   (26)                            /*!< CH26 Bit Offset */
#define DMA_SW_CHTRIG_CH26                       ((uint32_t)0x04000000)          /*!< Write 1, triggers DMA_CHANNEL26 */
/* DMA_SW_CHTRIG[CH27] Bits */
#define DMA_SW_CHTRIG_CH27_OFS                   (27)              
