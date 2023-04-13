/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    hal_serial_lld.c
 * @brief   PLATFORM serial subsystem low level driver source.
 *
 * @addtogroup SERIAL
 * @{
 */

#include "hal.h"

#if (HAL_USE_SERIAL == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief USART1 serial driver identifier.*/
#if (PLATFORM_SERIAL_USE_USART1 == TRUE) || defined(__DOXYGEN__)
SerialDriver SD1;
#endif

#if (STM32_SERIAL_USE_USART0 == TRUE)
SerialDriver SD0;
#endif
#if (STM32_SERIAL_USE_USART1 == TRUE)
SerialDriver SD1;
#endif
#if (STM32_SERIAL_USE_USART3 == TRUE)
SerialDriver SD3;
#endif
#if (STM32_SERIAL_USE_UART4 == TRUE)
SerialDriver SD4;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/**
 * @brief   Driver default configuration.
 */
static const SerialConfig default_config = {
  115200
};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level serial driver initialization.
 *
 * @notapi
 */
#if STM32_SERIAL_USE_USART0 || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(VectorD4) {

  OSAL_IRQ_PROLOGUE();

  sd_lld_serve_interrupt(&SD0);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if STM32_SERIAL_USE_USART1 || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(VectorD8) {

  OSAL_IRQ_PROLOGUE();

  sd_lld_serve_interrupt(&SD1);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if STM32_SERIAL_USE_USART3 || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(Vector110) {

  OSAL_IRQ_PROLOGUE();

  sd_lld_serve_interrupt(&SD3);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if STM32_SERIAL_USE_UART4 || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(Vector114) {

  OSAL_IRQ_PROLOGUE();

  sd_lld_serve_interrupt(&SD4);
#if 1
  if(RESET != usart_interrupt_flag_get(UART4,USART_INT_FLAG_RT)){
    chnAddFlagsI(&SD4, CHN_BREAK_DETECTED);

    usart_flag_clear(UART4,USART_FLAG_RT);
  }
#endif

  OSAL_IRQ_EPILOGUE();
}
#endif

#if STM32_SERIAL_USE_USART0 || defined(__DOXYGEN__)
static void notify0(io_queue_t *qp) {

  (void)qp;
  USART_CTL0(USART0) |= 0x80;
}
#endif

#if STM32_SERIAL_USE_USART1 || defined(__DOXYGEN__)
static void notify1(io_queue_t *qp) {

  (void)qp;
  USART_CTL0(USART1) |= 0x80;
}
#endif

#if STM32_SERIAL_USE_USART3 || defined(__DOXYGEN__)
static void notify3(io_queue_t *qp) {

  (void)qp;
  USART_CTL0(UART3) |= 0x80;
}
#endif

#if STM32_SERIAL_USE_UART4 || defined(__DOXYGEN__)
static void notify4(io_queue_t *qp) {

  (void)qp;
  USART_CTL0(UART4) |= 0x80;
}
#endif


void sd_lld_init(void) {

#if STM32_SERIAL_USE_USART0 == TRUE
  sdObjectInit(&SD0, NULL, notify0);
  SD0.uart_basic = USART0;
#endif

#if STM32_SERIAL_USE_USART1 == TRUE
  sdObjectInit(&SD1, NULL, notify1);
  SD1.uart_basic = USART1;
#endif

#if STM32_SERIAL_USE_USART3
  sdObjectInit(&SD3, NULL, notify3);
  SD3.uart_basic = UART3;
#endif
#if STM32_SERIAL_USE_UART4
  sdObjectInit(&SD4, NULL, notify4);
  SD4.uart_basic = UART4;
#endif
}

/**
 * @brief   Low level serial driver configuration and (re)start.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] config    the architecture-dependent serial driver configuration.
 *                      If this parameter is set to @p NULL then a default
 *                      configuration is used.
 *
 * @notapi
 */
void sd_lld_start(SerialDriver *sdp, const SerialConfig *config) {

  if (config == NULL) {
    config = &default_config;
  }

  if (sdp->state == SD_STOP) {
#if PLATFORM_SERIAL_USE_USART1 == TRUE
    if (&SD1 == sdp) {

    }
#endif

#if STM32_SERIAL_USE_USART0
    rcu_periph_clock_enable(RCU_USART0);
    usart_deinit(USART0);
    usart_word_length_set(USART0, USART_WL_8BIT);
    usart_stop_bit_set(USART0, USART_STB_1BIT);
    usart_parity_config(USART0, USART_PM_NONE);
    usart_baudrate_set(USART0, SERIAL_DEFAULT_BITRATE);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    usart_enable(USART0);
    nvicEnableVector(USART0_IRQn, STM32_IRQ_USART0_PRIORITY);
    usart_interrupt_enable(USART0, USART_INT_RBNE);
#endif

#if STM32_SERIAL_USE_USART1
    rcu_periph_clock_enable(RCU_USART1);
    usart_deinit(USART1);
    usart_word_length_set(USART1, USART_WL_8BIT);
    usart_stop_bit_set(USART1, USART_STB_1BIT);
    usart_parity_config(USART1, USART_PM_NONE);
    usart_baudrate_set(USART1, SERIAL_DEFAULT_BITRATE);
    usart_receive_config(USART1, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART1, USART_TRANSMIT_ENABLE);
    usart_enable(USART1);
    nvicEnableVector(USART1_IRQn, STM32_IRQ_USART1_PRIORITY);
    usart_interrupt_enable(USART1, USART_INT_RBNE);
#endif

#if STM32_SERIAL_USE_USART3
    rcu_periph_clock_enable(RCU_UART3);
    usart_deinit(UART3);
    usart_word_length_set(UART3, USART_WL_8BIT);
    usart_stop_bit_set(UART3, USART_STB_1BIT);
    usart_parity_config(UART3, USART_PM_NONE);
    usart_baudrate_set(UART3, SERIAL_DEFAULT_BITRATE);
    usart_receive_config(UART3, USART_RECEIVE_ENABLE);
    usart_transmit_config(UART3, USART_TRANSMIT_ENABLE);
    usart_enable(UART3);
    nvicEnableVector(UART3_IRQn, STM32_IRQ_USART3_PRIORITY);
    usart_interrupt_enable(UART3, USART_INT_RBNE);
#endif
#if STM32_SERIAL_USE_UART4
    rcu_periph_clock_enable(RCU_UART4);
    usart_deinit(UART4);
    usart_word_length_set(UART4, USART_WL_8BIT);
    usart_stop_bit_set(UART4, USART_STB_1BIT);
    usart_parity_config(UART4, USART_PM_NONE);
    usart_baudrate_set(UART4, SERIAL_DEFAULT_BITRATE);
    usart_receive_config(UART4, USART_RECEIVE_ENABLE);
    usart_transmit_config(UART4, USART_TRANSMIT_ENABLE);
    usart_enable(UART4);
    nvicEnableVector(UART4_IRQn, STM32_IRQ_USART3_PRIORITY);
    usart_interrupt_enable(UART4, USART_INT_RBNE);
#endif



  }
  /* Configures the peripheral.*/
  (void)config; /* Warning suppression, remove this.*/
}

/**
 * @brief   Low level serial driver stop.
 * @details De-initializes the USART, stops the associated clock, resets the
 *          interrupt vector.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 *
 * @notapi
 */
void sd_lld_stop(SerialDriver *sdp) {

  if (sdp->state == SD_READY) {
#if PLATFORM_SERIAL_USE_USART1 == TRUE
    if (&SD1 == sdp) {

    }
#endif
  }
}

void sd_lld_serve_interrupt(SerialDriver *sdp){
    while(USART_STAT0(sdp->uart_basic) & 0x20){
      osalSysLockFromISR();
      sdIncomingDataI(sdp, USART_DATA(sdp->uart_basic) & 0xFF);
      osalSysUnlockFromISR();
    }

    if(USART_CTL0(sdp->uart_basic) & 0x80){           //trans buff empty
      while(USART_STAT0(sdp->uart_basic) & 0x80){   
        msg_t b;
        osalSysLockFromISR();
        b = oqGetI(&sdp->oqueue);
        if (b < MSG_OK) {
          // chnAddFlagsI(sdp, CHN_OUTPUT_EMPTY);
          USART_CTL0(sdp->uart_basic) &= ~ 0x80;
          osalSysUnlockFromISR();
          break;
        }

        USART_DATA(sdp->uart_basic) = b;            //发送数据

        osalSysUnlockFromISR();
      }
    }
}

void sd5_lld_serve_interrupt(SerialDriver *sdp){
    while(USART5_STAT(sdp->uart_basic) & 0x20){
      osalSysLockFromISR();
      sdIncomingDataI(sdp, USART5_RDATA(sdp->uart_basic) & 0xFF);
      osalSysUnlockFromISR();
    }

    if(USART5_CTL0(sdp->uart_basic) & 0x80){           //trans buff empty
      while(USART5_STAT(sdp->uart_basic) & 0x80){
        msg_t b;
        osalSysLockFromISR();
        b = oqGetI(&sdp->oqueue);
        if (b < MSG_OK) {
          // chnAddFlagsI(sdp, CHN_OUTPUT_EMPTY);
          USART5_CTL0(sdp->uart_basic) &= ~ 0x80;
          osalSysUnlockFromISR();
          break;
        }

        USART5_TDATA(sdp->uart_basic) = b;

        osalSysUnlockFromISR();
      }
    }
}

#endif /* HAL_USE_SERIAL == TRUE */

/** @} */
