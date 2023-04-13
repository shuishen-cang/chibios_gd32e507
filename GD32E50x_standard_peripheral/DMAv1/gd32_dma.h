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
 * @file    DMAv1/stm32_dma.h
 * @brief   DMA helper driver header.
 * @note    This driver uses the new naming convention used for the STM32F2xx
 *          so the "DMA channels" are referred as "DMA streams".
 *
 * @addtogroup STM32_DMA
 * @{
 */

#ifndef GD32_DMA_H
#define GD32_DMA_H


// #include "gd32_registry.h"
#include "gd32e50x_libopt.h"
/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   DMA capability.
 * @details if @p TRUE then the DMA is able of burst transfers, FIFOs,
 *          scatter gather and other advanced features.
 */
#define STM32_DMA_ADVANCED          FALSE

/**
 * @brief   Maximum number of transfers in a single operation.
 */
#define STM32_DMA_MAX_TRANSFER      65535

/**
 * @brief   Total number of DMA streams.
 * @details This is the total number of streams among all the DMA units.
 */
#define STM32_DMA1_NUM_CHANNELS     7
#define STM32_DMA2_NUM_CHANNELS     5
#define STM32_DMA_STREAMS           STM32_DMA1_NUM_CHANNELS + STM32_DMA2_NUM_CHANNELS

/**
 * @brief   Returns an unique numeric identifier for a DMA stream.
 *
 * @param[in] dma       the DMA unit number
 * @param[in] stream    the stream number
 * @return              An unique numeric stream identifier.
 */
#define STM32_DMA_STREAM_ID(dma, stream)                                    \
    ((((dma) - 1) * STM32_DMA1_NUM_CHANNELS) + ((stream) - 1))

/**
 * @brief   Returns a DMA stream identifier mask.
 *
 *
 * @param[in] dma       the DMA unit number
 * @param[in] stream    the stream number
 * @return              A DMA stream identifier mask.
 */
#define STM32_DMA_STREAM_ID_MSK(dma, stream)                                \
  (1U << STM32_DMA_STREAM_ID(dma, stream))

/**
 * @brief   Checks if a DMA stream unique identifier belongs to a mask.
 *
 * @param[in] id        the stream numeric identifier
 * @param[in] mask      the stream numeric identifiers mask
 *
 * @retval              The check result.
 * @retval false        id does not belong to the mask.
 * @retval true         id belongs to the mask.
 */
#define STM32_DMA_IS_VALID_ID(id, mask) (((1U << (id)) & (mask)))

/**
 * @name    DMA streams identifiers
 * @{
 */
/**
 * @brief   Returns a pointer to a stm32_dma_stream_t structure.
 *
 * @param[in] id        the stream numeric identifier
 * @return              A pointer to the stm32_dma_stream_t constant structure
 *                      associated to the DMA stream.
 */
#define STM32_DMA_STREAM(id)        (&_stm32_dma_streams[id])

#if STM32_DMA1_NUM_CHANNELS > 0
#define STM32_DMA1_STREAM1          STM32_DMA_STREAM(0)
#endif
#if STM32_DMA1_NUM_CHANNELS > 1
#define STM32_DMA1_STREAM2          STM32_DMA_STREAM(1)
#endif
#if STM32_DMA1_NUM_CHANNELS > 2
#define STM32_DMA1_STREAM3          STM32_DMA_STREAM(2)
#endif
#if STM32_DMA1_NUM_CHANNELS > 3
#define STM32_DMA1_STREAM4          STM32_DMA_STREAM(3)
#endif
#if STM32_DMA1_NUM_CHANNELS > 4
#define STM32_DMA1_STREAM5          STM32_DMA_STREAM(4)
#endif
#if STM32_DMA1_NUM_CHANNELS > 5
#define STM32_DMA1_STREAM6          STM32_DMA_STREAM(5)
#endif
#if STM32_DMA1_NUM_CHANNELS > 6
#define STM32_DMA1_STREAM7          STM32_DMA_STREAM(6)
#endif
#if STM32_DMA1_NUM_CHANNELS > 7
#define STM32_DMA1_STREAM8          STM32_DMA_STREAM(7)
#endif
#if STM32_DMA2_NUM_CHANNELS > 0
#define STM32_DMA2_STREAM1          STM32_DMA_STREAM(STM32_DMA1_NUM_CHANNELS + 0)
#endif
#if STM32_DMA2_NUM_CHANNELS > 1
#define STM32_DMA2_STREAM2          STM32_DMA_STREAM(STM32_DMA1_NUM_CHANNELS + 1)
#endif
#if STM32_DMA2_NUM_CHANNELS > 2
#define STM32_DMA2_STREAM3          STM32_DMA_STREAM(STM32_DMA1_NUM_CHANNELS + 2)
#endif
#if STM32_DMA2_NUM_CHANNELS > 3
#define STM32_DMA2_STREAM4          STM32_DMA_STREAM(STM32_DMA1_NUM_CHANNELS + 3)
#endif
#if STM32_DMA2_NUM_CHANNELS > 4
#define STM32_DMA2_STREAM5          STM32_DMA_STREAM(STM32_DMA1_NUM_CHANNELS + 4)
#endif
#if STM32_DMA2_NUM_CHANNELS > 5
#define STM32_DMA2_STREAM6          STM32_DMA_STREAM(STM32_DMA1_NUM_CHANNELS + 5)
#endif
#if STM32_DMA2_NUM_CHANNELS > 6
#define STM32_DMA2_STREAM7          STM32_DMA_STREAM(STM32_DMA1_NUM_CHANNELS + 6)
#endif
#if STM32_DMA2_NUM_CHANNELS > 7
#define STM32_DMA2_STREAM8          STM32_DMA_STREAM(STM32_DMA1_NUM_CHANNELS + 7)
#endif
/** @} */

/**
 * @brief   Type of a DMA callback.
 *
 * @param[in] p         parameter for the registered function
 * @param[in] flags     pre-shifted content of the ISR register, the bits
 *                      are aligned to bit zero
 */
typedef void (*stm32_dmaisr_t)(void *p, uint32_t flags);


#define     STM32_DMA1_CH1_NUMBER       11
#define     STM32_DMA1_CH2_NUMBER       12
#define     STM32_DMA1_CH3_NUMBER       13
#define     STM32_DMA1_CH4_NUMBER       14
#define     STM32_DMA1_CH5_NUMBER       15
#define     STM32_DMA1_CH6_NUMBER       16
#define     STM32_DMA1_CH7_NUMBER       17
#define     STM32_DMA2_CH1_NUMBER       56
#define     STM32_DMA2_CH2_NUMBER       57
#define     STM32_DMA2_CH3_NUMBER       58
#define     STM32_DMA2_CH4_NUMBER       59
#define     STM32_DMA2_CH5_NUMBER       60

#define STM32_DMA1_CH1_HANDLER              Vector6C
#define STM32_DMA1_CH2_HANDLER              Vector70
#define STM32_DMA1_CH3_HANDLER              Vector74
#define STM32_DMA1_CH4_HANDLER              Vector78
#define STM32_DMA1_CH5_HANDLER              Vector7C
#define STM32_DMA1_CH6_HANDLER              Vector80
#define STM32_DMA1_CH7_HANDLER              Vector84
#define STM32_DMA2_CH1_HANDLER              Vector120
#define STM32_DMA2_CH2_HANDLER              Vector124
#define STM32_DMA2_CH3_HANDLER              Vector128
#define STM32_DMA2_CH4_HANDLER              Vector12C
#define STM32_DMA2_CH5_HANDLER              Vector130

#define STM32_DMA_ISR_MASK                  0x0E

/**
 * @brief   STM32 DMA stream descriptor structure.
 */
typedef struct {
//   DMA_TypeDef           *dma;           /**< @brief Associated DMA.         */
//   DMA_Channel_TypeDef   *channel;       /**< @brief Associated DMA channel. */
//   uint32_t              cmask;          /**< @brief Mask of streams sharing
//                                              the same ISR.                  */
//   uint8_t               dummy;          /**< @brief Filler.                 */
    uint32_t            basic;
  uint8_t               shift;          /**< @brief Bit offset in ISR, IFCR
//                                              and CSELR registers.           */
  uint8_t               selfindex;      /**< @brief Index to self in array. */
  uint8_t               vector;         /**< @brief Associated IRQ vector.  */
} stm32_dma_stream_t;

// /*===========================================================================*/
// /* External declarations.                                                    */
// /*===========================================================================*/

// #if !defined(__DOXYGEN__)
// extern const stm32_dma_stream_t _stm32_dma_streams[STM32_DMA_STREAMS];
// #endif

#ifdef __cplusplus
extern "C" {
#endif
  void dmaInit(void);
  const stm32_dma_stream_t *dmaStreamAllocI(uint32_t id,
                                            uint32_t priority,
                                            stm32_dmaisr_t func,
                                            void *param);
  const stm32_dma_stream_t *dmaStreamAlloc(uint32_t id,
                                           uint32_t priority,
                                           stm32_dmaisr_t func,
                                           void *param);
  void dmaStreamFreeI(const stm32_dma_stream_t *dmastp);
  void dmaStreamFree(const stm32_dma_stream_t *dmastp);
  void dmaServeInterrupt(const stm32_dma_stream_t *dmastp);
#ifdef __cplusplus
}
#endif

#endif /* STM32_DMA_H */

/** @} */
