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

#include <stdint.h>
#include <stdbool.h>

#include "ch.h"
#include "hal.h"
#include "chprintf.h"
// #include "rt_test_root.h"
// #include "oslib_test_root.h"

// uint8_t buff[2];

/*
* Application entry point.
*/

int main(void) {
    rtcnt_t x1, x2;
    /*
    * System initializations.
    * - HAL initialization, this also initializes the configured device drivers
    *   and performs the board-specific initializations.
    * - Kernel initialization, the main() function becomes a thread and the
    *   RTOS is active.
    */
    halInit();
    chSysInit();

    palSetLineMode(LINE_UART3_TX, GDPAL_MODE(GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ));
    palSetLineMode(LINE_UART3_RX, GDPAL_MODE(GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ));
    sdStart(&SD3, NULL);

    chprintf((void*)&SD3, "hello, chibios\n");  


    while (true) {
        // chnReadTimeout(&SD3, buff, 1, TIME_INFINITE);
        // chnWrite(&SD3, buff, 1);

        x1 = chSysGetRealtimeCounterX();

        x2 = chSysGetRealtimeCounterX();

        // for(uint32_t i = 0; i < NPOINT; i += 1){
        //     // fprintf(mat_wr, "%d %d\n",xout[i].real, xout[i].imag);       

        //     chprintf((void*)&SD3, "%d:%d %d\n",i,xout[i].real, xout[i].imag); 
        //     chThdSleepMilliseconds(50);   
        // } 



        chprintf((void*)&SD3, "%d\n", (x2 - x1) >> 0); 
        chThdSleepMilliseconds(50000);

        // palToggleLine(LINE_LED1);
        // palSetLine(LINE_LED0);
        // chThdSleepMilliseconds(500);
        // palClearLine(LINE_LED0);
        // chThdSleepMilliseconds(500);

        // chprintf((void*)&SD3, "hello\n");  
    }
}
