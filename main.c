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
#include "usbd_core.h"
#include "usbd_msc_core.h"
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

    

    // rcu_usb_clock_config(RCU_CKUSB_CKPLL_DIV3_5);   
    // rcu_periph_clock_enable(RCU_USBHS);


    usbd_init(&USBD1.udev, &msc_desc, &msc_class);

    usbStart(&USBD1, &usbcfg);

    // pllusb_rcu_config();
//     USBD1.udev.dev.desc = &msc_desc;

// //    /* class callbacks */
//     USBD1.udev.dev.class_core = &msc_class;

// //    /* create serial string */
//     // serial_string_get(udev->dev.desc->strings[STR_IDX_SERIAL]);

//     /* configure USB capabilities */
//     (void)usb_basic_init (&USBD1.udev.bp, &USBD1.udev.regs);

//     /* initializes the USB core*/
//     (void)usb_core_init (USBD1.udev.bp, &USBD1.udev.regs);

//     /* set device disconnect */
//     usbd_disconnect (&USBD1.udev);

//     /* initializes device mode */
//     (void)usb_devcore_init (&USBD1.udev);

//     /* set device connect */
//     usbd_connect (&USBD1.udev);

//     USBD1.udev.dev.cur_status = (uint8_t)USBD_DEFAULT;


    

    // usbStart(&USBD1, &usbcfg);


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

// OSAL_IRQ_HANDLER(STM32_OTG1_HANDLER) {

//   OSAL_IRQ_PROLOGUE();

//   // usb_lld_serve_interrupt(&USBD1);

//   usbd_isr (&msc_udisk);

//   OSAL_IRQ_EPILOGUE();
// }
