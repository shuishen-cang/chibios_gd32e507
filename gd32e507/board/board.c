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

#include "hal.h"





/**
 * @brief   Board-specific initialization code.
 * @note    You can add your board-specific code here.
 */
void boardInit(void) {

}

void __early_init(void) {
    SystemInit();

    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOD);

    // palSetLineMode(LINE_LED0, GDPAL_MODE(GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ));
    // palSetLineMode(LINE_LED1, GDPAL_MODE(GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ));
    // palSetLineMode(LINE_LED2, GDPAL_MODE(GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ));
    // palSetLineMode(LINE_LED3, GDPAL_MODE(GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ));

    // gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_7);

    palSetPadMode(GPIOA, GPIO_PIN_7, GDPAL_MODE(GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ));
    palSetPadMode(GPIOA, GPIO_PIN_8, GDPAL_MODE(GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ));
    palSetPadMode(GPIOA, GPIO_PIN_10, GDPAL_MODE(GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ));
    palSetPadMode(GPIOC, GPIO_PIN_13, GDPAL_MODE(GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ));    
}
