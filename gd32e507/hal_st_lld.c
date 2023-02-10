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
 * @file    hal_st_lld.c
 * @brief   PLATFORM ST subsystem low level driver source.
 *
 * @addtogroup ST
 * @{
 */

#include "hal.h"

#if (OSAL_ST_MODE != OSAL_ST_MODE_NONE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local types.                                                       */
/*===========================================================================*/

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
 * @brief   Low level ST driver initialization.
 *
 * @notapi
 */

void st_lld_init(void) {
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER1);

    timer_deinit(TIMER1);

    /* TIMER configuration */
    timer_initpara.prescaler         = (SystemCoreClock / OSAL_ST_FREQUENCY) - 1;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 0xFFFFFFFF;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER1,&timer_initpara);

    timer_auto_reload_shadow_enable(TIMER1);
    timer_enable(TIMER1);
	// timer_interrupt_enable(TIMER1, TIMER_INT_CH0);
    nvicEnableVector(TIMER1_IRQn, STM32_ST_IRQ_PRIORITY);
}

OSAL_IRQ_HANDLER(VectorB0) {
    uint32_t sr;

    OSAL_IRQ_PROLOGUE();
    sr = TIMER_INTF(TIMER1);
    TIMER_INTF(TIMER1) = ~sr;

    if(sr & TIMER_INT_CH0){
        osalSysLockFromISR();
        osalOsTimerHandlerI();
        osalSysUnlockFromISR();
    }

    OSAL_IRQ_EPILOGUE();
}

#endif /* OSAL_ST_MODE != OSAL_ST_MODE_NONE */

/** @} */
