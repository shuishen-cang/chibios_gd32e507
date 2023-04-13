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
 * @file    hal_uart_lld.c
 * @brief   PLATFORM UART subsystem low level driver source.
 *
 * @addtogroup UART
 * @{
 */

#include "hal.h"

#if (HAL_USE_UART == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   UART1 driver identifier.
 */
#if (PLATFORM_UART_USE_UART1 == TRUE) || defined(__DOXYGEN__)
UARTDriver UARTD1;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

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
 * @brief   Low level UART driver initialization.
 *
 * @notapi
 */
void uart_lld_init(void) {
#if PLATFORM_UART_USE_UART1 == TRUE
  /* Driver initialization.*/
  uartObjectInit(&UARTD1);
  UARTD1.uart_basic   = USART1;
#endif
}

/**
 * @brief   RX DMA common service routine.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 * @param[in] flags     pre-shifted content of the ISR register
 */
static void uart_lld_serve_rx_end_irq(UARTDriver *uartp, uint32_t flags) {

//   /* DMA errors handling.*/
// #if defined(STM32_UART_DMA_ERROR_HOOK)
//   if ((flags & (STM32_DMA_ISR_TEIF | STM32_DMA_ISR_DMEIF)) != 0) {
//     STM32_UART_DMA_ERROR_HOOK(uartp);
//   }
// #else
  (void)flags;
  (void)uartp;
// #endif

//   if (uartp->rxstate == UART_RX_IDLE) {
//     /* Receiver in idle state, a callback is generated, if enabled, for each
//        received character and then the driver stays in the same state.*/
//     _uart_rx_idle_code(uartp);
//   }
//   else {
//     /* Receiver in active state, a callback is generated, if enabled, after
//        a completed transfer.*/
//     dmaStreamDisable(uartp->dmarx);
//     _uart_rx_complete_isr_code(uartp);
//   }
}

/**
 * @brief   TX DMA common service routine.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 * @param[in] flags     pre-shifted content of the ISR register
 */
static void uart_lld_serve_tx_end_irq(UARTDriver *uartp, uint32_t flags) {
    if(flags & 0x02){
        uint32_t channelx = uartp->dmatx->shift >> 2;
        DMA_CHCTL(uartp->dmatx->basic, channelx) &= ~(1 << 0);
        _uart_tx1_isr_code(uartp);
    }
}


/**
 * @brief   Configures and activates the UART peripheral.
 *
 * @param[in] uartp      pointer to the @p UARTDriver object
 *
 * @notapi
 */
void uart_lld_start(UARTDriver *uartp) {

  if (uartp->state == UART_STOP) {
    /* Enables the peripheral.*/
#if PLATFORM_UART_USE_UART1 == TRUE
    if (&UARTD1 == uartp) {
        rcu_periph_clock_enable(RCU_USART1);
        usart_deinit(USART1);
        usart_word_length_set(USART1, USART_WL_8BIT);
        usart_stop_bit_set(USART1, USART_STB_1BIT);
        usart_parity_config(USART1, USART_PM_NONE);
        usart_baudrate_set(USART1, SERIAL_DEFAULT_BITRATE);
        usart_receive_config(USART1, USART_RECEIVE_ENABLE);
        usart_transmit_config(USART1, USART_TRANSMIT_ENABLE);
        usart_dma_transmit_config(USART1, USART_DENT_ENABLE);
        usart_dma_transmit_config(USART1, USART_DENR_ENABLE);
        usart_enable(USART1);
        // nvicEnableVector(USART1_IRQn, STM32_IRQ_USART1_PRIORITY);
        // usart_interrupt_enable(USART1, USART_INT_RBNE);

        uartp->dmatx = dmaStreamAlloc(STM32_UART_USART1_TX_DMA_STREAM,
                                        STM32_UART_USART1_IRQ_PRIORITY,
                                        (stm32_dmaisr_t)uart_lld_serve_tx_end_irq,
                                        (void *)uartp);
        uartp->dmarx = dmaStreamAlloc(STM32_UART_USART1_RX_DMA_STREAM,
                                        STM32_UART_USART1_IRQ_PRIORITY,
                                        (stm32_dmaisr_t)uart_lld_serve_rx_end_irq,
                                        (void *)uartp);
    }
#endif
  }
  /* Configures the peripheral.*/
  uartp->rxstate = UART_RX_IDLE;
  uartp->txstate = UART_TX_IDLE;
}





/**
 * @brief   Deactivates the UART peripheral.
 *
 * @param[in] uartp      pointer to the @p UARTDriver object
 *
 * @notapi
 */
void uart_lld_stop(UARTDriver *uartp) {

  if (uartp->state == UART_READY) {
    /* Resets the peripheral.*/

    /* Disables the peripheral.*/
#if PLATFORM_UART_USE_UART1 == TRUE
    if (&UARTD1 == uartp) {
        usart_disable(USART1);
    }
#endif
  }
}




/**
 * @brief   Starts a transmission on the UART peripheral.
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 * @param[in] n         number of data frames to send
 * @param[in] txbuf     the pointer to the transmit buffer
 *
 * @notapi
 */
void uart_lld_start_send(UARTDriver *uartp, size_t n, const void *txbuf) {
    uint32_t channelx = uartp->dmatx->shift >> 2;

    DMA_CHCTL(uartp->dmatx->basic, channelx) = 0;
    DMA_CHPADDR(uartp->dmatx->basic, channelx) = (uint32_t)&USART_DATA(uartp->uart_basic);
    DMA_CHMADDR(uartp->dmatx->basic, channelx) = (uint32_t)txbuf;
    DMA_CHCNT(uartp->dmatx->basic, channelx) = n;
    DMA_CHCTL(uartp->dmatx->basic, channelx) = (1 << 7) | (1 << 4) | (1 << 1) ;     //m2p, comp interrupt, mem increase
    DMA_CHCTL(uartp->dmatx->basic, channelx) |= (1 << 0);   
}

/**
 * @brief   Stops any ongoing transmission.
 * @note    Stopping a transmission also suppresses the transmission callbacks.
 *
 * @param[in] uartp      pointer to the @p UARTDriver object
 *
 * @return              The number of data frames not transmitted by the
 *                      stopped transmit operation.
 *
 * @notapi
 */
size_t uart_lld_stop_send(UARTDriver *uartp) {
    uint32_t channelx = uartp->dmatx->shift >> 2;
    DMA_CHCTL(uartp->dmatx->basic, channelx) &= ~(1 << 0);

    return DMA_CHCNT(uartp->dmatx->basic, channelx);
}

/**
 * @brief   Starts a receive operation on the UART peripheral.
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 * @param[in] n         number of data frames to send
 * @param[out] rxbuf    the pointer to the receive buffer
 *
 * @notapi
 */
void uart_lld_start_receive(UARTDriver *uartp, size_t n, void *rxbuf) {
    uint32_t channelx = uartp->dmarx->shift >> 2;

    DMA_CHCTL(uartp->dmarx->basic, channelx) = 0;
    DMA_CHPADDR(uartp->dmarx->basic, channelx) = (uint32_t)&USART_DATA(uartp->uart_basic);
    DMA_CHMADDR(uartp->dmarx->basic, channelx) = (uint32_t)rxbuf;
    DMA_CHCNT(uartp->dmarx->basic, channelx) = n;
    DMA_CHCTL(uartp->dmarx->basic, channelx) = (1 << 7) | (0 << 4) | (1 << 1) ;     //m2p, comp interrupt, mem increase
    DMA_CHCTL(uartp->dmarx->basic, channelx) |= (1 << 0);   
}

/**
 * @brief   Stops any ongoing receive operation.
 * @note    Stopping a receive operation also suppresses the receive callbacks.
 *
 * @param[in] uartp      pointer to the @p UARTDriver object
 *
 * @return              The number of data frames not received by the
 *                      stopped receive operation.
 *
 * @notapi
 */
size_t uart_lld_stop_receive(UARTDriver *uartp) {
    uint32_t channelx = uartp->dmatx->shift >> 2;
    DMA_CHCTL(uartp->dmatx->basic, channelx) &= ~(1 << 0);

    return DMA_CHCNT(uartp->dmatx->basic, channelx);
}

#endif /* HAL_USE_UART == TRUE */

/** @} */
