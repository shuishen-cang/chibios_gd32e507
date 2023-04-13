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

uint8_t buff[128] = {1,2,3,4};

UARTConfig uart1_param = {
    .txend1_cb = NULL           ,
    .txend2_cb = NULL           ,
    .rxend_cb = NULL            ,
    .rxchar_cb = NULL           ,
    .rxerr_cb = NULL           
};

int main(void) {
    // rtcnt_t x1, x2;
    // event_listener_t el;

    halInit();
    chSysInit();

    palSetLineMode(LINE_LED, GDPAL_MODE(GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ));

    palSetLineMode(LINE_UART1_TX, GDPAL_MODE(GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ));
    palSetLineMode(LINE_UART1_RX, GDPAL_MODE(GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ));

    // palSetLineMode(LINE_UART4_TX, GDPAL_MODE(GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ));
    // palSetLineMode(LINE_UART4_RX, GDPAL_MODE(GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ));

    // sdStart(&SD1, NULL);
    // usart_receiver_timeout_threshold_config(UART4,30);                                          //帧尾检测
    // usart_receiver_timeout_enable(UART4);
    // usart_interrupt_enable(UART4, USART_INT_RT);

    usbStart(&USBD1, &usbcfg);
    // usb_user_initial();

    uartStart(&UARTD1, &uart1_param);

    while(1){
        size_t tsize = 4;
        uartReceiveTimeout(&UARTD1, &tsize, buff, TIME_INFINITE);
        uartSendTimeout(&UARTD1, &tsize, buff, TIME_INFINITE);

        // palSetLine(LINE_LED);
        // chThdSleepMilliseconds(200);
        // palClearLine(LINE_LED);
        // chThdSleepMilliseconds(200);

        // uartStartSend(&UARTD1, 4, buff);
        // USART_DATA(USART1) = 0X55;
        // chprintf((void*)&SD1,"hello!\n");
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

