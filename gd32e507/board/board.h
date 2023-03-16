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

#ifndef BOARD_H
#define BOARD_H

#include "system_gd32e50x.h"
/*
 * Setup for a generic board.
 */
#define LINE_LED0                   PAL_LINE(GPIOA, 7U)
#define LINE_LED1                   PAL_LINE(GPIOA, 8U)
#define LINE_LED2                   PAL_LINE(GPIOA, 10U)
#define LINE_LED3                   PAL_LINE(GPIOC, 13U)

#define LINE_UART3_TX               PAL_LINE(GPIOC, 10U)  
#define LINE_UART3_RX               PAL_LINE(GPIOC, 11U)  

#define LINE_UART4_TX               PAL_LINE(GPIOC, 12U)  
#define LINE_UART4_RX               PAL_LINE(GPIOD, 2U)  

#define LINE_LED                    PAL_LINE(GPIOB, 8U)  

/*
 * Board identifier.
 */
#define BOARD_GENERIC
#define BOARD_NAME                  "Generic Board"

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* BOARD_H */
