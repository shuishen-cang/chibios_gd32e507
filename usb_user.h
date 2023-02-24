#ifndef __USB_USER_H__
#define __USB_USER_H__

#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "usbcfg.h"

extern thread_t *usbtx_tp;

void usb_user_initial(void);

#endif

