/**
  ******************************************************************************
  * @file    stm32l5xx.h
  * @author  MCD Application Team
  * @brief   CMSIS STM32L5xx Device Peripheral Access Layer Header File.
  *
  *          The file is the unique include file that the application programmer
  *          is using in the C source code, usually in main.c. This file contains:
  *           - Configuration section that allows to select:
  *              - The STM32L5xx device used in the target application
  *              - To use or not the peripheral�s drivers in application code(i.e.
  *                code will be based on direct access to peripheral�s registers
  *                rather than drivers API), this option is controlled by
  *                "#define USE_HAL_DRIVER"
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup stm32l5xx
  * @{
  */

#ifndef STM32L5xx_H
#define STM32L5xx_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/** @addtogroup Library_configuration_section
  * @{
  */

/**
  * @brief STM32 Family
  */
#if !defined (STM32L5)
#define STM32L5
#endif /* STM32L5 */

/* Uncomment the line below according to the target STM32L5 device used in your
   application
  */

#if !defined (STM32L552xx) && !defined (STM32L562xx)
  /* #define STM32L552xx */   /*!< STM32L552xx Devices */
  /* #define STM32L562xx */   /*!< STM32L562xx Devices */
#endif

/*  Tip: To avoid modifying this file each time you need to switch between these
        devices, you can define the device in your toolchain compiler preprocessor.
  */
#if !defined  (USE_HAL_DRIVER)
/**
 * @brief Comment the line below if you will not use the peripherals drivers.
   In this case, these drivers will not be included and the application code will
   be based on direct access to peripherals registers
   */
  /*#define USE_HAL_DRIVER */
#endif /* USE_HAL_DRIVER */

/**
  * @brief CMSIS Device version number
  */
#define __STM32L5_CMSIS_VERSION_MAIN   (0x01U) /*!< [31:24] main version */
#define __STM32L5_CMSIS_VERSION_SUB1   (0x00U) /*!< [23:16] sub1 version */
#define __STM32L5_CMSIS_VERSION_SUB2   (0x02U) /*!< [15:8]  sub2 version */
#define __STM32L5_CMSIS_VERSION_RC     (0x00U) /*!< [7:0]  release candidate */
#define __STM32L5_CMSIS_VERSION        ((__STM32L5_CMSIS_VERSION_MAIN << 24U)\
                                       |(__STM32L5_CMSIS_VERSION_SUB1 << 16U)\
                                       |(__STM32L5_CMSIS_VERSION_SUB2 << 8U )\
                                       |(__STM32L5_CMSIS_VERSION_RC))

/**
  * @}
  */

/** @addtogroup Device_Included
  * @{
  */


#define __CM33_REV                0x0000U   /* Core revision r0p1 */
#define __SAUREGION_PRESENT       1U        /* SAU regions present */
#define __MPU_PRESENT             1U        /* MPU present */
#define __VTOR_PRESENT            1U        /* VTOR present */
#define __NVIC_PRIO_BITS          3U        /* Number of Bits used for Priority Levels */
#define __Vendor_SysTickConfig    0U        /* Set to 1 if different SysTick Config is used */
#define __FPU_PRESENT             1U        /* FPU present */
#define __DSP_PRESENT             1U        /* DSP extension present */

typedef enum
{
/* =======================================  ARM Cortex-M33 Specific Interrupt Numbers  ======================================= */
  Reset_IRQn                = -15,    /*!< -15 Reset Vector, invoked on Power up and warm reset             */
  NonMaskableInt_IRQn       = -14,    /*!< -14 Non maskable Interrupt, cannot be stopped or preempted       */
  HardFault_IRQn            = -13,    /*!< -13 Hard Fault, all classes of Fault                             */
  MemoryManagement_IRQn     = -12,    /*!< -12 Memory Management, MPU mismatch, including Access Violation
                                               and No Match                                                  */
  BusFault_IRQn             = -11,    /*!< -11 Bus Fault, Pre-Fetch-, Memory Access Fault, other address/memory
                                               related Fault                                                 */
  UsageFault_IRQn           = -10,    /*!< -10 Usage Fault, i.e. Undef Instruction, Illegal State Transition */
  SecureFault_IRQn          =  -9,    /*!< -9  Secure Fault                                                  */
  SVCall_IRQn               =  -5,    /*!< -5  System Service Call via SVC instruction                       */
  DebugMonitor_IRQn         =  -4,    /*!< -4  Debug Monitor                                                 */
  PendSV_IRQn               =  -2,    /*!< -2  Pendable request for system service                           */
  SysTick_IRQn              =  -1,    /*!< -1  System Tick Timer                                             */

/* ===========================================  STM32L552xx Specific Interrupt Numbers  ========================================= */
  WWDG_IRQn                 = 0,      /*!< Window WatchDog interrupt                                         */
  PVD_PVM_IRQn              = 1,      /*!< PVD/PVM1/PVM2/PVM3/PVM4 through EXTI Line detection interrupts    */
  RTC_IRQn                  = 2,      /*!< RTC non-secure interrupts through the EXTI line 17                */
  RTC_S_IRQn                = 3,      /*!< RTC secure interrupts through the EXTI line 18                    */
  TAMP_IRQn                 = 4,      /*!< Tamper non-secure interrupts through the EXTI line 19             */
  TAMP_S_IRQn               = 5,      /*!< Tamper and TimeStamp interrupts through the EXTI line 20          */
  FLASH_IRQn                = 6,      /*!< FLASH non-secure global interrupt                                 */
  FLASH_S_IRQn              = 7,      /*!< FLASH secure global interrupt                                     */
  GTZC_IRQn                 = 8,      /*!< Global TrustZone controller global interrupt                      */
  RCC_IRQn                  = 9,      /*!< RCC non secure global interrupt                                   */
  RCC_S_IRQn                = 10,     /*!< RCC secure global interrupt                                       */
  EXTI0_IRQn                = 11,     /*!< EXTI Line0 interrupt                                              */
  EXTI1_IRQn                = 12,     /*!< EXTI Line1 interrupt                                              */
  EXTI2_IRQn                = 13,     /*!< EXTI Line2 interrupt                                              */
  EXTI3_IRQn                = 14,     /*!< EXTI Line3 interrupt                                              */
  EXTI4_IRQn                = 15,     /*!< EXTI Line4 interrupt                                              */
  EXTI5_IRQn                = 16,     /*!< EXTI Line5 interrupt                                              */
  EXTI6_IRQn                = 17,     /*!< EXTI Line6 interrupt                                              */
  EXTI7_IRQn                = 18,     /*!< EXTI Line7 interrupt                                              */
  EXTI8_IRQn                = 19,     /*!< EXTI Line8 interrupt                                              */
  EXTI9_IRQn                = 20,     /*!< EXTI Line9 interrupt                                              */
  EXTI10_IRQn               = 21,     /*!< EXTI Line10 interrupt                                             */
  EXTI11_IRQn               = 22,     /*!< EXTI Line11 interrupt                                             */
  EXTI12_IRQn               = 23,     /*!< EXTI Line12 interrupt                                             */
  EXTI13_IRQn               = 24,     /*!< EXTI Line13 interrupt                                             */
  EXTI14_IRQn               = 25,     /*!< EXTI Line14 interrupt                                             */
  EXTI15_IRQn               = 26,     /*!< EXTI Line15 interrupt                                             */
  DMAMUX1_IRQn              = 27,     /*!< DMAMUX1 non-secure interrupt                                      */
  DMAMUX1_S_IRQn            = 28,     /*!< DMAMUX1 secure interrupt                                          */
  DMA1_Channel1_IRQn        = 29,     /*!< DMA1 Channel 1 global interrupt                                   */
  DMA1_Channel2_IRQn        = 30,     /*!< DMA1 Channel 2 global interrupt                                   */
  DMA1_Channel3_IRQn        = 31,     /*!< DMA1 Channel 3 global interrupt                                   */
  DMA1_Channel4_IRQn        = 32,     /*!< DMA1 Channel 4 global interrupt                                   */
  DMA1_Channel5_IRQn        = 33,     /*!< DMA1 Channel 5 global interrupt                                   */
  DMA1_Channel6_IRQn        = 34,     /*!< DMA1 Channel 6 global interrupt                                   */
  DMA1_Channel7_IRQn        = 35,     /*!< DMA1 Channel 7 global interrupt                                   */
  DMA1_Channel8_IRQn        = 36,     /*!< DMA1 Channel 8 global interrupt                                   */
  ADC1_2_IRQn               = 37,     /*!< ADC1 & ADC2 global interrupts                                     */
  DAC_IRQn                  = 38,     /*!< DAC global interrupts                                             */
  FDCAN1_IT0_IRQn           = 39,     /*!< FDCAN1 interrupt 0                                                */
  FDCAN1_IT1_IRQn           = 40,     /*!< FDCAN1 interrupt 1                                                */
  TIM1_BRK_IRQn             = 41,     /*!< TIM1 Break interrupt                                              */
  TIM1_UP_IRQn              = 42,     /*!< TIM1 Update interrupt                                             */
  TIM1_TRG_COM_IRQn         = 43,     /*!< TIM1 Trigger and Commutation interrupt                            */
  TIM1_CC_IRQn              = 44,     /*!< TIM1 Capture Compare interrupt                                    */
  TIM2_IRQn                 = 45,     /*!< TIM2 global interrupt                                             */
  TIM3_IRQn                 = 46,     /*!< TIM3 global interrupt                                             */
  TIM4_IRQn                 = 47,     /*!< TIM4 global interrupt                                             */
  TIM5_IRQn                 = 48,     /*!< TIM5 global interrupt                                             */
  TIM6_IRQn                 = 49,     /*!< TIM6 global interrupt                                             */
  TIM7_IRQn                 = 50,     /*!< TIM7 global interrupt                                             */
  TIM8_BRK_IRQn             = 51,     /*!< TIM8 Break interrupt                                              */
  TIM8_UP_IRQn              = 52,     /*!< TIM8 Update interrupt                                             */
  TIM8_TRG_COM_IRQn         = 53,     /*!< TIM8 Trigger and Commutation interrupt                            */
  TIM8_CC_IRQn              = 54,     /*!< TIM8 Capture Compare interrupt                                    */
  I2C1_EV_IRQn              = 55,     /*!< I2C1 Event interrupt                                              */
  I2C1_ER_IRQn              = 56,     /*!< I2C1 Error interrupt                                              */
  I2C2_EV_IRQn              = 57,     /*!< I2C2 Event interrupt                                              */
  I2C2_ER_IRQn              = 58,     /*!< I2C2 Error interrupt                                              */
  SPI1_IRQn                 = 59,     /*!< SPI1 global interrupt                                             */
  SPI2_IRQn                 = 60,     /*!< SPI2 global interrupt                                             */
  USART1_IRQn               = 61,     /*!< USART1 global interrupt                                           */
  USART2_IRQn               = 62,     /*!< USART2 global interrupt                                           */
  USART3_IRQn               = 63,     /*!< USART3 global interrupt                                           */
  UART4_IRQn                = 64,     /*!< UART4 global interrupt                                            */
  UART5_IRQn                = 65,     /*!< UART5 global interrupt                                            */
  LPUART1_IRQn              = 66,     /*!< LPUART1 global interrupt                                          */
  LPTIM1_IRQn               = 67,     /*!< LPTIM1 global interrupt                                           */
  LPTIM2_IRQn               = 68,     /*!< LPTIM2 global interrupt                                           */
  TIM15_IRQn                = 69,     /*!< TIM15 global interrupt                                            */
  TIM16_IRQn                = 70,     /*!< TIM16 global interrupt                                            */
  TIM17_IRQn                = 71,     /*!< TIM17 global interrupt                                            */
  COMP_IRQn                 = 72,     /*!< COMP1 and COMP2 through EXTI Lines interrupts                     */
  USB_FS_IRQn               = 73,     /*!< USB FS global interrupt                                           */
  CRS_IRQn                  = 74,     /*!< CRS global interrupt                                              */
  FMC_IRQn                  = 75,     /*!< FMC global interrupt                                              */
  OCTOSPI1_IRQn             = 76,     /*!< OctoSPI1 global interrupt                                         */
  SDMMC1_IRQn               = 78,     /*!< SDMMC1 global interrupt                                           */
  DMA2_Channel1_IRQn        = 80,     /*!< DMA2 Channel 1 global interrupt                                   */
  DMA2_Channel2_IRQn        = 81,     /*!< DMA2 Channel 2 global interrupt                                   */
  DMA2_Channel3_IRQn        = 82,     /*!< DMA2 Channel 3 global interrupt                                   */
  DMA2_Channel4_IRQn        = 83,     /*!< DMA2 Channel 4 global interrupt                                   */
  DMA2_Channel5_IRQn        = 84,     /*!< DMA2 Channel 5 global interrupt                                   */
  DMA2_Channel6_IRQn        = 85,     /*!< DMA2 Channel 6 global interrupt                                   */
  DMA2_Channel7_IRQn        = 86,     /*!< DMA2 Channel 7 global interrupt                                   */
  DMA2_Channel8_IRQn        = 87,     /*!< DMA2 Channel 8 global interrupt                                   */
  I2C3_EV_IRQn              = 88,     /*!< I2C3 event interrupt                                              */
  I2C3_ER_IRQn              = 89,     /*!< I2C3 error interrupt                                              */
  SAI1_IRQn                 = 90,     /*!< Serial Audio Interface 1 global interrupt                         */
  SAI2_IRQn                 = 91,     /*!< Serial Audio Interface 2 global interrupt                         */
  TSC_IRQn                  = 92,     /*!< Touch Sense Controller global interrupt                           */
  RNG_IRQn                  = 94,     /*!< RNG global interrupt                                              */
  FPU_IRQn                  = 95,     /*!< FPU global interrupt                                              */
  HASH_IRQn                 = 96,     /*!< HASH global interrupt                                             */
  LPTIM3_IRQn               = 98,     /*!< LPTIM3 global interrupt                                           */
  SPI3_IRQn                 = 99,     /*!< SPI3 global interrupt                                             */
  I2C4_EV_IRQn              = 100,    /*!< I2C4 Event interrupt                                              */
  I2C4_ER_IRQn              = 101,    /*!< I2C4 Error interrupt                                              */
  DFSDM1_FLT0_IRQn          = 102,    /*!< DFSDM1 Filter 0 global interrupt                                  */
  DFSDM1_FLT1_IRQn          = 103,    /*!< DFSDM1 Filter 1 global interrupt                                  */
  DFSDM1_FLT2_IRQn          = 104,    /*!< DFSDM1 Filter 2 global interrupt                                  */
  DFSDM1_FLT3_IRQn          = 105,    /*!< DFSDM1 Filter 3 global interrupt                                  */
  UCPD1_IRQn                = 106,    /*!< UCPD1 global interrupt                                            */
  ICACHE_IRQn               = 107,    /*!< Instruction cache global interrupt                                */
} IRQn_Type;




#include <core_cm33.h>                       /*!< ARM Cortex-M33 processor and core peripherals */

#if defined(STM32L552xx)
  // #include "stm32l552xx.h"
#elif defined(STM32L562xx)
  #include "stm32l562xx.h"
#else
 #error "Please select first the target STM32L5xx device used in your application (in stm32l5xx.h file)"
#endif

/**
  * @}
  */

/** @addtogroup Exported_types
  * @{
  */
typedef enum
{
  RESET = 0,
  SET = !RESET
} FlagStatus, ITStatus;

typedef enum
{
  DISABLE = 0,
  ENABLE = !DISABLE
} FunctionalState;
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))

typedef enum
{
  SUCCESS = 0,
  ERROR = !SUCCESS
} ErrorStatus;

/**
  * @}
  */


/** @addtogroup Exported_macros
  * @{
  */
#define SET_BIT(REG, BIT)     ((REG) |= (BIT))

#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

#define READ_BIT(REG, BIT)    ((REG) & (BIT))

#define CLEAR_REG(REG)        ((REG) = (0x0))

#define WRITE_REG(REG, VAL)   ((REG) = (VAL))

#define READ_REG(REG)         ((REG))

#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

#define POSITION_VAL(VAL)     (__CLZ(__RBIT(VAL)))


/**
  * @}
  */

#if defined (USE_HAL_DRIVER)
 #include "stm32l5xx_hal.h"
#endif /* USE_HAL_DRIVER */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STM32L5xx_H */
/**
  * @}
  */

/**
  * @}
  */




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
