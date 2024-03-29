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
 * @file    DMAv1/stm32_dma.c
 * @brief   DMA helper driver code.
 *
 * @addtogroup STM32_DMA
 * @details DMA sharing helper driver. In the STM32 the DMA streams are a
 *          shared resource, this driver allows to allocate and free DMA
 *          streams at runtime in order to allow all the other device
 *          drivers to coordinate the access to the resource.
 * @note    The DMA ISR handlers are all declared into this module because
 *          sharing, the various device drivers can associate a callback to
 *          ISRs when allocating streams.
 * @{
 */

#include "hal.h"

/* The following macro is only defined if some driver requiring DMA services
   has been enabled.*/
#if defined(STM32_DMA_REQUIRED) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/**
 * @brief   Mask of the DMA1 streams in @p dma_streams_mask.
 */
#define STM32_DMA1_STREAMS_MASK     ((1U << STM32_DMA1_NUM_CHANNELS) - 1U)

/**
 * @brief   Mask of the DMA2 streams in @p dma_streams_mask.
 */
#define STM32_DMA2_STREAMS_MASK     (((1U << STM32_DMA2_NUM_CHANNELS) -     \
                                      1U) << STM32_DMA1_NUM_CHANNELS)

/*
 * Default ISR collision masks.
 */
#if !defined(STM32_DMA1_CH1_CMASK)
#define STM32_DMA1_CH1_CMASK        (1U << 0U)
#endif

#if !defined(STM32_DMA1_CH2_CMASK)
#define STM32_DMA1_CH2_CMASK        (1U << 1U)
#endif

#if !defined(STM32_DMA1_CH3_CMASK)
#define STM32_DMA1_CH3_CMASK        (1U << 2U)
#endif

#if !defined(STM32_DMA1_CH4_CMASK)
#define STM32_DMA1_CH4_CMASK        (1U << 3U)
#endif

#if !defined(STM32_DMA1_CH5_CMASK)
#define STM32_DMA1_CH5_CMASK        (1U << 4U)
#endif

#if !defined(STM32_DMA1_CH6_CMASK)
#define STM32_DMA1_CH6_CMASK        (1U << 5U)
#endif

#if !defined(STM32_DMA1_CH7_CMASK)
#define STM32_DMA1_CH7_CMASK        (1U << 6U)
#endif

#if !defined(STM32_DMA2_CH1_CMASK)
#define STM32_DMA2_CH1_CMASK        (1U << (STM32_DMA1_NUM_CHANNELS + 0U))
#endif

#if !defined(STM32_DMA2_CH2_CMASK)
#define STM32_DMA2_CH2_CMASK        (1U << (STM32_DMA1_NUM_CHANNELS + 1U))
#endif

#if !defined(STM32_DMA2_CH3_CMASK)
#define STM32_DMA2_CH3_CMASK        (1U << (STM32_DMA1_NUM_CHANNELS + 2U))
#endif

#if !defined(STM32_DMA2_CH4_CMASK)
#define STM32_DMA2_CH4_CMASK        (1U << (STM32_DMA1_NUM_CHANNELS + 3U))
#endif

#if !defined(STM32_DMA2_CH5_CMASK)
#define STM32_DMA2_CH5_CMASK        (1U << (STM32_DMA1_NUM_CHANNELS + 4U))
#endif

#if !defined(STM32_DMA2_CH6_CMASK)
#define STM32_DMA2_CH6_CMASK        (1U << (STM32_DMA1_NUM_CHANNELS + 5U))
#endif


/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   DMA streams descriptors.
 * @details This table keeps the association between an unique stream
 *          identifier and the involved physical registers.
 * @note    Don't use this array directly, use the appropriate wrapper macros
 *          instead: @p STM32_DMA1_STREAM1, @p STM32_DMA1_STREAM2 etc.
 */
const stm32_dma_stream_t _stm32_dma_streams[STM32_DMA_STREAMS] = {
    {DMA0, 0,  0, STM32_DMA1_CH1_NUMBER},  
    {DMA0, 4,  1, STM32_DMA1_CH2_NUMBER},  
    {DMA0, 8,  2, STM32_DMA1_CH3_NUMBER},  
    {DMA0, 12,  3, STM32_DMA1_CH4_NUMBER},  
    {DMA0, 16,  4, STM32_DMA1_CH5_NUMBER},  
    {DMA0, 20,  5, STM32_DMA1_CH6_NUMBER},  
    {DMA0, 24,  6, STM32_DMA1_CH7_NUMBER},  
    {DMA1, 0,  7, STM32_DMA2_CH1_NUMBER},  
    {DMA1, 4,  8, STM32_DMA2_CH2_NUMBER},  
    {DMA1, 8,  9, STM32_DMA2_CH3_NUMBER},  
    {DMA1, 12,  10, STM32_DMA2_CH4_NUMBER},  
    {DMA1, 16,  11, STM32_DMA2_CH5_NUMBER}
};


// /*===========================================================================*/
// /* Driver local variables and types.                                         */
// /*===========================================================================*/

/**
 * @brief   Global DMA-related data structures.
 */
static struct {
    /**
     * @brief   Mask of the allocated streams.
     */
    uint32_t          allocated_mask;
    /**
     * @brief   Mask of the enabled streams ISRs.
     */
    uint32_t          isr_mask;
    /**
     * @brief   DMA IRQ redirectors.
     */
    struct {
    /**
     * @brief   DMA callback function.
     */
    stm32_dmaisr_t    func;
    /**
     * @brief   DMA callback parameter.
     */
    void              *param;
    } streams[STM32_DMA_STREAMS];
} dma;

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if defined(STM32_DMA1_CH1_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA1 stream 1 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA1_CH1_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dmaServeInterrupt(STM32_DMA1_STREAM1);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_DMA1_CH2_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA1 stream 2 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA1_CH2_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dmaServeInterrupt(STM32_DMA1_STREAM2);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_DMA1_CH3_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA1 stream 3 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA1_CH3_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dmaServeInterrupt(STM32_DMA1_STREAM3);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_DMA1_CH4_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA1 stream 4 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA1_CH4_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dmaServeInterrupt(STM32_DMA1_STREAM4);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_DMA1_CH5_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA1 stream 5 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA1_CH5_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dmaServeInterrupt(STM32_DMA1_STREAM5);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_DMA1_CH6_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA1 stream 6 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA1_CH6_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dmaServeInterrupt(STM32_DMA1_STREAM6);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_DMA1_CH7_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA1 stream 7 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA1_CH7_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dmaServeInterrupt(STM32_DMA1_STREAM7);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_DMA1_CH8_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA1 stream 8 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA1_CH8_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dmaServeInterrupt(STM32_DMA1_STREAM8);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_DMA2_CH1_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA2 stream 1 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA2_CH1_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dmaServeInterrupt(STM32_DMA2_STREAM1);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_DMA2_CH2_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA2 stream 2 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA2_CH2_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dmaServeInterrupt(STM32_DMA2_STREAM2);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_DMA2_CH3_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA2 stream 3 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA2_CH3_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dmaServeInterrupt(STM32_DMA2_STREAM3);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_DMA2_CH4_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA2 stream 4 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA2_CH4_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dmaServeInterrupt(STM32_DMA2_STREAM4);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_DMA2_CH5_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA2 stream 5 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA2_CH5_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dmaServeInterrupt(STM32_DMA2_STREAM5);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_DMA2_CH6_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA2 stream 6 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA2_CH6_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dmaServeInterrupt(STM32_DMA2_STREAM6);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_DMA2_CH7_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA2 stream 7 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA2_CH7_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dmaServeInterrupt(STM32_DMA2_STREAM7);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_DMA2_CH8_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA2 stream 8 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA2_CH8_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dmaServeInterrupt(STM32_DMA2_STREAM8);

  OSAL_IRQ_EPILOGUE();
}
#endif

// /*===========================================================================*/
// /* Driver exported functions.                                                */
// /*===========================================================================*/

/**
 * @brief   STM32 DMA helper initialization.
 *
 * @init
 */
void dmaInit(void) {
    dma_deinit(DMA0, DMA_CH0);
    dma_deinit(DMA0, DMA_CH1);
    dma_deinit(DMA0, DMA_CH2);
    dma_deinit(DMA0, DMA_CH3);
    dma_deinit(DMA0, DMA_CH4);
    dma_deinit(DMA0, DMA_CH5);
    dma_deinit(DMA0, DMA_CH6);
    dma_deinit(DMA1, DMA_CH0);
    dma_deinit(DMA1, DMA_CH1);
    dma_deinit(DMA1, DMA_CH2);
    dma_deinit(DMA1, DMA_CH3);
    dma_deinit(DMA1, DMA_CH4);

    dma.isr_mask = 0;
}

/**
 * @brief   Allocates a DMA stream.
 * @details The stream is allocated and, if required, the DMA clock enabled.
 *          The function also enables the IRQ vector associated to the stream
 *          and initializes its priority.
 *
 * @param[in] id        numeric identifiers of a specific stream or:
 *                      - @p STM32_DMA_STREAM_ID_ANY for any stream.
 *                      - @p STM32_DMA_STREAM_ID_ANY_DMA1 for any stream
 *                        on DMA1.
 *                      - @p STM32_DMA_STREAM_ID_ANY_DMA2 for any stream
 *                        on DMA2.
 *                      .
 * @param[in] priority  IRQ priority for the DMA stream
 * @param[in] func      handling function pointer, can be @p NULL
 * @param[in] param     a parameter to be passed to the handling function
 * @return              Pointer to the allocated @p stm32_dma_stream_t
 *                      structure.
 * @retval NULL         if a/the stream is not available.
 *
 * @iclass
 */
const stm32_dma_stream_t *dmaStreamAllocI(uint32_t id,
                                          uint32_t priority,
                                          stm32_dmaisr_t func,
                                          void *param) {
  uint32_t i, startid, endid;

  osalDbgCheckClassI();

  if (id < STM32_DMA_STREAMS) {
        startid = id;
        endid   = id;
  }
  else {
    osalDbgCheck(false);
    return NULL;
  }

  for (i = startid; i <= endid; i++) {
    uint32_t mask = (1U << i);
    if ((dma.allocated_mask & mask) == 0U) {
      const stm32_dma_stream_t *dmastp = STM32_DMA_STREAM(i);

      /* Installs the DMA handler.*/
      dma.streams[i].func  = func;
      dma.streams[i].param = param;
      dma.allocated_mask  |= mask;

      /* Enabling DMA clocks required by the current streams set.*/
      if ((STM32_DMA1_STREAMS_MASK & mask) != 0U) {
        rcu_periph_clock_enable(RCU_DMA0);
      }
#if STM32_DMA2_NUM_CHANNELS > 0
      if ((STM32_DMA2_STREAMS_MASK & mask) != 0U) {
        rcu_periph_clock_enable(RCU_DMA1);
      }
#endif

      /* Enables the associated IRQ vector if not already enabled and if a
         callback is defined.*/
      if (func != NULL) {

        if ((dma.isr_mask & (1 << id)) == 0U) {
          nvicEnableVector(dmastp->vector, priority);
        }
        dma.isr_mask |= (1 << id);
      }

      /* Putting the stream in a known state.*/

      return dmastp;
    }
  }

  return NULL;
}

/**
 * @brief   Allocates a DMA stream.
 * @details The stream is allocated and, if required, the DMA clock enabled.
 *          The function also enables the IRQ vector associated to the stream
 *          and initializes its priority.
 *
 * @param[in] id        numeric identifiers of a specific stream or:
 *                      - @p STM32_DMA_STREAM_ID_ANY for any stream.
 *                      - @p STM32_DMA_STREAM_ID_ANY_DMA1 for any stream
 *                        on DMA1.
 *                      - @p STM32_DMA_STREAM_ID_ANY_DMA2 for any stream
 *                        on DMA2.
 *                      .
 * @param[in] priority  IRQ priority for the DMA stream
 * @param[in] func      handling function pointer, can be @p NULL
 * @param[in] param     a parameter to be passed to the handling function
 * @return              Pointer to the allocated @p stm32_dma_stream_t
 *                      structure.
 * @retval NULL         if a/the stream is not available.
 *
 * @api
 */
const stm32_dma_stream_t *dmaStreamAlloc(uint32_t id,
                                         uint32_t priority,
                                         stm32_dmaisr_t func,
                                         void *param) {
  const stm32_dma_stream_t *dmastp;

  osalSysLock();
  dmastp = dmaStreamAllocI(id, priority, func, param);
  osalSysUnlock();

  return dmastp;
}

/**
 * @brief   Releases a DMA stream.
 * @details The stream is freed and, if required, the DMA clock disabled.
 *          Trying to release a unallocated stream is an illegal operation
 *          and is trapped if assertions are enabled.
 *
 * @param[in] dmastp    pointer to a stm32_dma_stream_t structure
 *
 * @iclass
 */
void dmaStreamFreeI(const stm32_dma_stream_t *dmastp) {
  uint32_t selfindex = (uint32_t)dmastp->selfindex;

  osalDbgCheck(dmastp != NULL);

  /* Check if the streams is not taken.*/
  osalDbgAssert((dma.allocated_mask & (1 << selfindex)) != 0U,
                "not allocated");

  /* Marks the stream as not allocated.*/
  dma.allocated_mask &= ~(1U << selfindex);
  dma.isr_mask &= ~(1U << selfindex);

  /* Disables the associated IRQ vector if it is no more in use.*/
//   if ((dma.isr_mask & dmastp->cmask) == 0U) {
//     nvicDisableVector(dmastp->vector);
//   }

  /* Removes the DMA handler.*/
  dma.streams[selfindex].func  = NULL;
  dma.streams[selfindex].param = NULL;

  /* Shutting down clocks that are no more required, if any.*/
  if ((dma.allocated_mask & STM32_DMA1_STREAMS_MASK) == 0U) {
    // rccDisableDMA1();
  }
#if STM32_DMA2_NUM_CHANNELS > 0
  if ((dma.allocated_mask & STM32_DMA2_STREAMS_MASK) == 0U) {
    // rccDisableDMA2();
  }
#endif
}

/**
 * @brief   Releases a DMA stream.
 * @details The stream is freed and, if required, the DMA clock disabled.
 *          Trying to release a unallocated stream is an illegal operation
 *          and is trapped if assertions are enabled.
 *
 * @param[in] dmastp    pointer to a stm32_dma_stream_t structure
 *
 * @api
 */
void dmaStreamFree(const stm32_dma_stream_t *dmastp) {

  osalSysLock();
  dmaStreamFreeI(dmastp);
  osalSysUnlock();
}

/**
 * @brief   Serves a DMA IRQ.
 *
 * @param[in] dmastp    pointer to a stm32_dma_stream_t structure
 *
 * @special
 */
void dmaServeInterrupt(const stm32_dma_stream_t *dmastp) {
  uint32_t flags;
  uint32_t selfindex = (uint32_t)dmastp->selfindex;

  flags = (DMA_INTF(dmastp->basic) >> dmastp->shift) & STM32_DMA_ISR_MASK;
//   if (flags & dmastp->channel->CCR) {
    DMA_INTC(dmastp->basic) = flags << dmastp->shift;
    if (dma.streams[selfindex].func) {
        dma.streams[selfindex].func(dma.streams[selfindex].param, flags);
    }
//   }
}

#endif /* STM32_DMA_REQUIRED */

/** @} */
