#include "usb_user.h"

uint32_t usb_head = 0xeb90d5c8;
uint8_t usb_txbuff[1024], usb_rxbuff[1024];
thread_t *usbtx_tp;

static THD_WORKING_AREA(wa_usbtx, 128);
static THD_FUNCTION(usb_tx, arg) {

    (void)arg;
    chRegSetThreadName("usb_tx");

    for(uint16_t i = 0; i < 1024; i++){
        usb_txbuff[i] = i;
    }
    chThdSleepMilliseconds(10000);
    while (true) {
        usbTransmit(&USBD1, USBD2_DATA_REQUEST_EP, usb_txbuff, 4);   
        chThdSleepMilliseconds(1000);
        // thread_t *tp = chMsgWait();
        // msg_t tmsg = chMsgGet(tp);
        // chMsgRelease(tp, tmsg);

        // if(tmsg == 0x01 || tmsg == 0x02){
        //     if(tmsg == 0x01){
        //         // chprintf((void*)&SD3, "data,%x,%x,%x,%x,%x,%x,%x,%x!\n",spi_buff1.buff[0], spi_buff1.buff[1], spi_buff1.buff[2], spi_buff1.buff[3], spi_buff1.buff[4], spi_buff1.buff[5], spi_buff1.buff[6], spi_buff1.buff[7]);
        //         usbTransmit(&USBD1, USBD2_DATA_REQUEST_EP, (uint8_t*)&usb_head, 4);            //第一包，发送一个包头  
        //     }

        //     // osalMutexLock(&mutex_1);
        //     usbTransmit(&USBD1, USBD2_DATA_REQUEST_EP, usb_txbuff, 4);   
        //     // osalMutexUnlock(&mutex_1);
        // }
        // else if(tmsg == 0x03){
        //     // osalMutexLock(&mutex_2);
        //     usbTransmit(&USBD1, USBD2_DATA_REQUEST_EP, usb_txbuff, 6);   
        //     // osalMutexUnlock(&mutex_2);  
        // }
    }
}

void usb_user_initial(void){
    usbtx_tp = chThdCreateStatic(wa_usbtx, sizeof(wa_usbtx), NORMALPRIO + 2, usb_tx, NULL);
}

