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
#include "usbcfg.h"
#include "usb_user.h"
// #include "usbd_core.h"
// #include "usbd_msc_core.h"
// #include "rt_test_root.h"
// #include "oslib_test_root.h"

uint8_t buff[128];


// const USBConfig usbcfg = {
//   NULL,
//   NULL,
//   NULL,
//   NULL
// };

/*
* Application entry point.
*/


// usb_core_driver msc_udisk;

unsigned char SRAM[40 * 1024];
extern void pllusb_rcu_config(void);

int main(void) {
    // rtcnt_t x1, x2;
    // event_listener_t el;

    /*
    * System initializations.
    * - HAL initialization, this also initializes the configured device drivers
    *   and performs the board-specific initializations.
    * - Kernel initialization, the main() function becomes a thread and the
    *   RTOS is active.
    */
    halInit();
    chSysInit();

    // palSetLineMode(LINE_UART3_TX, GDPAL_MODE(GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ));
    // palSetLineMode(LINE_UART3_RX, GDPAL_MODE(GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ));

    // palSetLineMode(LINE_UART4_TX, GDPAL_MODE(GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ));
    // palSetLineMode(LINE_UART4_RX, GDPAL_MODE(GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ));

    // sdStart(&SD4, NULL);
    // usart_receiver_timeout_threshold_config(UART4,30);                                          //帧尾检测
    // usart_receiver_timeout_enable(UART4);
    // usart_interrupt_enable(UART4, USART_INT_RT);

    usbStart(&USBD1, &usbcfg);

    usb_user_initial();

    while(1){
        chThdSleepMicroseconds(1000000);
    }

    // x1 = chSysGetRealtimeCounterX();
    // x2 = chSysGetRealtimeCounterX();

    // uint32_t system_clock = ;
    // chprintf((void*)&SD3, "hello, chibios, %dMHz\n", rcu_clock_freq_get(CK_SYS) / 1000000);  

    // chEvtRegisterMaskWithFlags(&SD4.event, &el, EVENT_MASK(1), CHN_BREAK_DETECTED);
    // while (true) {
    //     chEvtWaitOne(EVENT_MASK(1));

    //     size_t xlen = chnReadTimeout(&SD4, buff, 128, TIME_IMMEDIATE);
    //     // chprintf((void*)&SD4, "system size:%d\n", xlen); 
    //     chnWrite(&SD4, buff, xlen);  
    // }

    // chEvtUnregister(&SD4.event, &el);
}

