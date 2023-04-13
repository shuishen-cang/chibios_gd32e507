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
 * @file    OTGv1/hal_usb_lld.c
 * @brief   STM32 USB subsystem low level driver source.
 *
 * @addtogroup USB
 * @{
 */

#include <string.h>
// #include "drv_usbd_int.h"
#include "hal.h"

#if HAL_USE_USB || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#define TRDT_VALUE_FS           5
#define TRDT_VALUE_HS           9

#define EP0_MAX_INSIZE          64
#define EP0_MAX_OUTSIZE         64

#if STM32_OTG_STEPPING == 1
#if defined(BOARD_OTG_NOVBUSSENS)
#define GCCFG_INIT_VALUE        (GCCFG_NOVBUSSENS | GCCFG_VBUSASEN |        \
                                 GCCFG_VBUSBSEN | GCCFG_PWRDWN)
#else
#define GCCFG_INIT_VALUE        (GCCFG_VBUSASEN | GCCFG_VBUSBSEN |          \
                                 GCCFG_PWRDWN)
#endif

#elif STM32_OTG_STEPPING == 2
#if defined(BOARD_OTG_NOVBUSSENS)
#define GCCFG_INIT_VALUE        GCCFG_PWRDWN
#else
#define GCCFG_INIT_VALUE        (GCCFG_VBDEN | GCCFG_PWRDWN)
#endif

#endif

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief OTG_FS driver identifier.*/
#if STM32_USB_USE_OTG1 || defined(__DOXYGEN__)
USBDriver USBD1;
#endif

/** @brief OTG_HS driver identifier.*/
#if STM32_USB_USE_OTG2 || defined(__DOXYGEN__)
USBDriver USBD2;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/**
 * @brief   EP0 state.
 * @note    It is an union because IN and OUT endpoints are never used at the
 *          same time for EP0.
 */
static union {
  /**
   * @brief   IN EP0 state.
   */
  USBInEndpointState in;
  /**
   * @brief   OUT EP0 state.
   */
  USBOutEndpointState out;
} ep0_state;

/**
 * @brief   Buffer for the EP0 setup packets.
 */
static uint8_t ep0setup_buffer[8];

/**
 * @brief   EP0 initialization structure.
 */
static const USBEndpointConfig ep0config = {
  USB_EP_MODE_TYPE_CTRL,
  _usb_ep0setup,
  _usb_ep0in,
  _usb_ep0out,
  0x40,
  0x40,
  &ep0_state.in,
  &ep0_state.out,
  1,
  ep0setup_buffer
};

#if STM32_USB_USE_OTG1
static const stm32_otg_params_t fsparams = {
  STM32_USB_OTG1_RX_FIFO_SIZE,
  STM32_OTG1_FIFO_MEM_SIZE,
  STM32_OTG1_ENDPOINTS
};
#endif

#if STM32_USB_USE_OTG2
static const stm32_otg_params_t hsparams = {
  STM32_USB_OTG2_RX_FIFO_SIZE / 4,
  STM32_OTG2_FIFO_MEM_SIZE,
  STM32_OTG2_ENDPOINTS
};
#endif

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static void otg_core_reset(USBDriver *usbp) {
  stm32_otg_t *otgp = usbp->otg;

  /* Wait AHB idle condition.*/
  while ((otgp->GRSTCTL & GRSTCTL_AHBIDL) == 0)
    ;

  /* Core reset and delay of at least 3 PHY cycles.*/
  otgp->GRSTCTL = GRSTCTL_CSRST;
  osalSysPolledDelayX(12);
  while ((otgp->GRSTCTL & GRSTCTL_CSRST) != 0)
    ;

  osalSysPolledDelayX(18);

  /* Wait AHB idle condition again.*/
  while ((otgp->GRSTCTL & GRSTCTL_AHBIDL) == 0)
    ;
}

static void otg_disable_ep(USBDriver *usbp) {
  stm32_otg_t *otgp = usbp->otg;
  unsigned i;

  for (i = 0; i <= usbp->otgparams->num_endpoints; i++) {

    if ((otgp->ie[i].DIEPCTL & DIEPCTL_EPENA) != 0U) {
      otgp->ie[i].DIEPCTL |= DIEPCTL_EPDIS;
    }

    if ((otgp->oe[i].DOEPCTL & DIEPCTL_EPENA) != 0U) {
      otgp->oe[i].DOEPCTL |= DIEPCTL_EPDIS;
    }

    otgp->ie[i].DIEPINT = 0xFFFFFFFF;
    otgp->oe[i].DOEPINT = 0xFFFFFFFF;
  }
  otgp->DAINTMSK = DAINTMSK_OEPM(0) | DAINTMSK_IEPM(0);
}

static void otg_rxfifo_flush(USBDriver *usbp) {
  stm32_otg_t *otgp = usbp->otg;

  otgp->GRSTCTL = GRSTCTL_RXFFLSH;
  while ((otgp->GRSTCTL & GRSTCTL_RXFFLSH) != 0)
    ;
  /* Wait for 3 PHY Clocks.*/
  osalSysPolledDelayX(18);
}

static void otg_txfifo_flush(USBDriver *usbp, uint32_t fifo) {
  stm32_otg_t *otgp = usbp->otg;

  otgp->GRSTCTL = GRSTCTL_TXFNUM(fifo) | GRSTCTL_TXFFLSH;
  while ((otgp->GRSTCTL & GRSTCTL_TXFFLSH) != 0)
    ;
  /* Wait for 3 PHY Clocks.*/
  osalSysPolledDelayX(18);
}

/**
 * @brief   Resets the FIFO RAM memory allocator.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
static void otg_ram_reset(USBDriver *usbp) {

  usbp->pmnext = usbp->otgparams->rx_fifo_size;
}

/**
 * @brief   Allocates a block from the FIFO RAM memory.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] size      size of the packet buffer to allocate in words
 *
 * @notapi
 */
static uint32_t otg_ram_alloc(USBDriver *usbp, size_t size) {
  uint32_t next;

  next = usbp->pmnext;
  usbp->pmnext += size;
  osalDbgAssert(usbp->pmnext <= usbp->otgparams->otg_ram_size,
                "OTG FIFO memory overflow");
  return next;
}

/**
 * @brief   Writes to a TX FIFO.
 *
 * @param[in] fifop     pointer to the FIFO register
 * @param[in] buf       buffer where to copy the endpoint data
 * @param[in] n         maximum number of bytes to copy
 *
 * @notapi
 */
static void otg_fifo_write_from_buffer(volatile uint32_t *fifop,
                                       void *buf,
                                       size_t n) {

  osalDbgAssert(n > 0, "is zero");

  // uint32_t aa;

  // llk_set(1, 3, buf, n);

  while (true) {
    *fifop = *((uint32_t *)buf);
    // aa = *((uint32_t *)buf);
    if (n <= 4) {
      break;
    }
    n -= 4;
    buf += 4;
  }
}

/**
 * @brief   Reads a packet from the RXFIFO.
 *
 * @param[in] fifop     pointer to the FIFO register
 * @param[out] buf      buffer where to copy the endpoint data
 * @param[in] n         number of bytes to pull from the FIFO
 * @param[in] max       number of bytes to copy into the buffer
 *
 * @notapi
 */
static void otg_fifo_read_to_buffer(volatile uint32_t *fifop,
                                    uint8_t *buf,
                                    size_t n,
                                    size_t max) {
  uint32_t w = 0;
  size_t i = 0;

  // uint8_t *ptr2 = buf;

  while (i < n) {
    if ((i & 3) == 0) {
      w = *fifop;
    }
    if (i < max) {
      *buf++ = (uint8_t)w;
      w >>= 8;
    }
    i++;
  }

  // llk_set(0, 2, ptr2, n);
}

/**
 * @brief   Incoming packets handler.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
// extern usb_core_driver msc_udisk;

static void otg_rxfifo_handler(USBDriver *usbp) {     //rx fifo非空
  uint32_t sts, cnt, ep;

  /* Popping the event word out of the RX FIFO.*/
  sts = usbp->otg->GRXSTSP;

  /* Event details.*/
  cnt = (sts & GRXSTSP_BCNT_MASK) >> GRXSTSP_BCNT_OFF;
  ep  = (sts & GRXSTSP_EPNUM_MASK) >> GRXSTSP_EPNUM_OFF;

  switch (sts & GRXSTSP_PKTSTS_MASK) {
  case GRXSTSP_SETUP_DATA:                                                      //接收到setup数据包
    otg_fifo_read_to_buffer(usbp->otg->FIFO[0], usbp->epc[ep]->setup_buf,       //数据读取到setup buff中，8byte
                            cnt, 8);
    break;
  case GRXSTSP_SETUP_COMP:                                                      //setup事务完成中断
    break;
  case GRXSTSP_OUT_DATA:                                                        //接收到out数据包
    otg_fifo_read_to_buffer(usbp->otg->FIFO[0],                                 
                            usbp->epc[ep]->out_state->rxbuf,
                            cnt,
                            usbp->epc[ep]->out_state->rxsize -
                            usbp->epc[ep]->out_state->rxcnt);
    usbp->epc[ep]->out_state->rxbuf += cnt;                                     
    usbp->epc[ep]->out_state->rxcnt += cnt;                                     //数据接收到的长度递增                   
    break;
  case GRXSTSP_OUT_COMP:
    break;
  case GRXSTSP_OUT_GLOBAL_NAK:
    break;
  default:
    break;
  }
}

/**
 * @brief   Outgoing packets handler.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
static bool otg_txfifo_handler(USBDriver *usbp, usbep_t ep) {                   //发送fifo空中断，类似于串口发送fifo空中断

  /* The TXFIFO is filled until there is space and data to be transmitted.*/
  while (true) {
    uint32_t n;

    /* Transaction end condition.*/
    if (usbp->epc[ep]->in_state->txcnt >= usbp->epc[ep]->in_state->txsize) {    //如果发送计数已经满足了，关闭发送空中断
#if 1
      usbp->otg->DIEPEMPMSK &= ~DIEPEMPMSK_INEPTXFEM(ep);
#endif
      return true;
    }

    /* Number of bytes remaining in current transaction.*/
    n = usbp->epc[ep]->in_state->txsize - usbp->epc[ep]->in_state->txcnt;       //长度等于发送长度减去以及发送的长度
    if (n > usbp->epc[ep]->in_maxsize)                                          //大于端点的最大长度，截断
      n = usbp->epc[ep]->in_maxsize;

    /* Checks if in the TXFIFO there is enough space to accommodate the
       next packet.*/
    if (((usbp->otg->ie[ep].DTXFSTS & DTXFSTS_INEPTFSAV_MASK) * 4) < n)       //如果fifo的有效长度小于将要发送的，直接退出
      return false;

#if STM32_USB_OTGFIFO_FILL_BASEPRI
    __set_BASEPRI(CORTEX_PRIO_MASK(STM32_USB_OTGFIFO_FILL_BASEPRI));
#endif  
    otg_fifo_write_from_buffer(usbp->otg->FIFO[ep],                           //装填数据到fifo中
                               (void*)usbp->epc[ep]->in_state->txbuf,
                               n);
    usbp->epc[ep]->in_state->txbuf += n;                                      //发送数据指针递增                                  
    usbp->epc[ep]->in_state->txcnt += n;                                      //发送的长度递增
#if STM32_USB_OTGFIFO_FILL_BASEPRI
  __set_BASEPRI(0);
#endif
  }
}

/**
 * @brief   Generic endpoint IN handler.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
static void otg_epin_handler(USBDriver *usbp, usbep_t ep) {
  stm32_otg_t *otgp = usbp->otg;
  uint32_t epint = otgp->ie[ep].DIEPINT;

  otgp->ie[ep].DIEPINT = epint;

  if (epint & DIEPINT_TOC) {                                        //超时
    /* Timeouts not handled yet, not sure how to handle.*/
  }
  if ((epint & DIEPINT_XFRC) && (otgp->DIEPMSK & DIEPMSK_XFRCM)) {
    /* Transmit transfer complete.*/
    USBInEndpointState *isp = usbp->epc[ep]->in_state;

    if (isp->txsize < isp->totsize) {                               //如果发送的长度小于最小长度
      /* In case the transaction covered only part of the total transfer
         then another transaction is immediately started in order to
         cover the remaining.*/
      isp->txsize = isp->totsize - isp->txsize;                     //发送长度设置
      isp->txcnt  = 0;                                              //重新计数
      osalSysLockFromISR();
      usb_lld_start_in(usbp, ep);                                   //再次启动发送 
      osalSysUnlockFromISR();
    }
    else {
      /* End on IN transfer.*/
      _usb_isr_invoke_in_cb(usbp, ep);
    }
  }
  if ((epint & DIEPINT_TXFE) &&                                     //如果发送fifo空了
      (otgp->DIEPEMPMSK & DIEPEMPMSK_INEPTXFEM(ep))) {
    /* TX FIFO empty or emptying.*/
    otg_txfifo_handler(usbp, ep);
  }
}

/**
 * @brief   Generic endpoint OUT handler.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
static void otg_epout_handler(USBDriver *usbp, usbep_t ep) {
  stm32_otg_t *otgp = usbp->otg;
  uint32_t epint = otgp->oe[ep].DOEPINT;

  /* Resets all EP IRQ sources.*/
  otgp->oe[ep].DOEPINT = epint;

  if ((epint & DOEPINT_STUP) && (otgp->DOEPMSK & DOEPMSK_STUPM)) {      //令牌包
    /* Setup packets handling, setup packets are handled using a
       specific callback.*/
    _usb_isr_invoke_setup_cb(usbp, ep);                                 //令牌包回调函数
  }

  if ((epint & DOEPINT_XFRC) && (otgp->DOEPMSK & DOEPMSK_XFRCM)) {      //传输完成中断
    USBOutEndpointState *osp;

    /* OUT state structure pointer for this endpoint.*/
    osp = usbp->epc[ep]->out_state;                                     //输出端点状态寄存器

    /* EP0 requires special handling.*/
    if (ep == 0) {                                                      //如果是0端点，每次最多只能发送64byte，超过的部分需要分多次发送

#if defined(STM32_OTG_SEQUENCE_WORKAROUND)
      /* If an OUT transaction end interrupt is processed while the state
         machine is not in an OUT state then it is ignored, this is caused
         on some devices (L4) apparently injecting spurious data complete
         words in the RX FIFO.*/
      if ((usbp->ep0state & USB_OUT_STATE) == 0)
        return;
#endif

      /* In case the transaction covered only part of the total transfer
         then another transaction is immediately started in order to
         cover the remaining.*/
      if (((osp->rxcnt % usbp->epc[ep]->out_maxsize) == 0) &&           //端点0，超过64byte的需要多次传输， 比如，67要分为64和3两次传输，那么第一次就是 rxcnt == 64并且rxsize计算要小于totsize         
          (osp->rxsize < osp->totsize)) {
        osp->rxsize = osp->totsize - osp->rxsize;                       //继续发送的长度等于总发送长度减去以及发送的长度
        osp->rxcnt  = 0;                                                //重新从0开始记录
        osalSysLockFromISR();
        usb_lld_start_out(usbp, ep);                                    //继续发送                               
        osalSysUnlockFromISR();
        return;
      }
    }

    /* End on OUT transfer.*/
    _usb_isr_invoke_out_cb(usbp, ep);
  }
}

/**
 * @brief   Isochronous IN transfer failed handler.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
static void otg_isoc_in_failed_handler(USBDriver *usbp) {
  usbep_t ep;
  stm32_otg_t *otgp = usbp->otg;

  for (ep = 0; ep <= usbp->otgparams->num_endpoints; ep++) {
    if (((otgp->ie[ep].DIEPCTL & DIEPCTL_EPTYP_MASK) == DIEPCTL_EPTYP_ISO) &&
        ((otgp->ie[ep].DIEPCTL & DIEPCTL_EPENA) != 0)) {
      /* Endpoint enabled -> ISOC IN transfer failed */
      /* Disable endpoint */
      otgp->ie[ep].DIEPCTL |= (DIEPCTL_EPDIS | DIEPCTL_SNAK);
      while (otgp->ie[ep].DIEPCTL & DIEPCTL_EPENA)
        ;

      /* Flush FIFO */
      otg_txfifo_flush(usbp, ep);

      /* Prepare data for next frame */
      _usb_isr_invoke_in_cb(usbp, ep);

      /* TX FIFO empty or emptying.*/
      otg_txfifo_handler(usbp, ep);
    }
  }
}

/**
 * @brief   Isochronous OUT transfer failed handler.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
static void otg_isoc_out_failed_handler(USBDriver *usbp) {
  usbep_t ep;
  stm32_otg_t *otgp = usbp->otg;

  for (ep = 0; ep <= usbp->otgparams->num_endpoints; ep++) {
    if (((otgp->oe[ep].DOEPCTL & DOEPCTL_EPTYP_MASK) == DOEPCTL_EPTYP_ISO) &&
        ((otgp->oe[ep].DOEPCTL & DOEPCTL_EPENA) != 0)) {
      /* Endpoint enabled -> ISOC OUT transfer failed */
      /* Disable endpoint */
      /* CHTODO:: Core stucks here */
      /*otgp->oe[ep].DOEPCTL |= (DOEPCTL_EPDIS | DOEPCTL_SNAK);
      while (otgp->oe[ep].DOEPCTL & DOEPCTL_EPENA)
        ;*/
      /* Prepare transfer for next frame.*/
      _usb_isr_invoke_out_cb(usbp, ep);
    }
  }
}

/**
 * @brief   OTG shared ISR.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */

// #include "drv_usb_dev.h"



void usb_lld_serve_interrupt(USBDriver *usbp) {
    stm32_otg_t *otgp = usbp->otg;
    uint32_t sts, src;

    sts  = otgp->GINTSTS;
    sts &= otgp->GINTMSK;
    otgp->GINTSTS = sts;

    /* Reset interrupt handling.*/
    if (sts & GINTSTS_USBRST) {
        /* Default reset action.*/
        _usb_reset(usbp);
        return;
    }
    /* Wake-up handling.*/
    if (sts & GINTSTS_WKUPINT) {
      /* If clocks are gated off, turn them back on (may be the case if
        coming out of suspend mode).*/
      if (otgp->PCGCCTL & (PCGCCTL_STPPCLK | PCGCCTL_GATEHCLK)) {
        /* Set to zero to un-gate the USB core clocks.*/
        otgp->PCGCCTL &= ~(PCGCCTL_STPPCLK | PCGCCTL_GATEHCLK);
      }

      /* Clear the Remote Wake-up Signaling.*/
      otgp->DCTL &= ~DCTL_RWUSIG;

      _usb_wakeup(usbp);
        // (void)usbd_int_wakeup (&usbp->udev);
    }

    /* Suspend handling.*/
    if (sts & GINTSTS_USBSUSP) {
        // (void)usbd_int_suspend (&usbp->udev);
      /* Stopping all ongoing transfers.*/
      otg_disable_ep(usbp);

      /* Default suspend action.*/
      _usb_suspend(usbp);
    }

    /* Enumeration done.*/
    if (sts & GINTSTS_ENUMDNE) {
      // (void)usbd_int_enumfinish (&usbp->udev);
      /* Full or High speed timing selection.*/
      if ((otgp->DSTS & DSTS_ENUMSPD_MASK) == DSTS_ENUMSPD_HS_480) {
        otgp->GUSBCFG = (otgp->GUSBCFG & ~(GUSBCFG_TRDT_MASK)) |
                        GUSBCFG_TRDT(TRDT_VALUE_HS);
      }
      else {
        otgp->GUSBCFG = (otgp->GUSBCFG & ~(GUSBCFG_TRDT_MASK)) |
                        GUSBCFG_TRDT(TRDT_VALUE_FS);
      }
    }

    /* SOF interrupt handling.*/
    if (sts & GINTSTS_SOF) {
      _usb_isr_invoke_sof_cb(usbp);

            // if (usbp->udev.dev.class_core->SOF) {
            //     (void)usbp->udev.dev.class_core->SOF(&usbp->udev); 
            // }

            // /* clear interrupt */
            // usbp->udev.regs.gr->GINTF = GINTF_SOF;

    }

    /* Isochronous IN failed handling */
    if (sts & GINTSTS_IISOIXFR) {
      otg_isoc_in_failed_handler(usbp);
    }

    /* Isochronous OUT failed handling */
    if (sts & GINTSTS_IISOOXFR) {
      otg_isoc_out_failed_handler(usbp);
    }

    /* Performing the whole FIFO emptying in the ISR, it is advised to keep
      this IRQ at a very low priority level.*/
    if ((sts & GINTSTS_RXFLVL) != 0U) {
      otg_rxfifo_handler(usbp);
    }

    /* IN/OUT endpoints event handling.*/
    src = otgp->DAINT;
    if (sts & GINTSTS_OEPINT) {
      // usbd_int_epout (&usbp->udev);
      if (src & (1 << 16))
        otg_epout_handler(usbp, 0);
      if (src & (1 << 17))
        otg_epout_handler(usbp, 1);
      if (src & (1 << 18))
        otg_epout_handler(usbp, 2);
      if (src & (1 << 19))
        otg_epout_handler(usbp, 3);
      if (src & (1 << 20))
        otg_epout_handler(usbp, 4);
      if (src & (1 << 21))
        otg_epout_handler(usbp, 5);

    }
    if (sts & GINTSTS_IEPINT) {
      if (src & (1 << 0))
        otg_epin_handler(usbp, 0);
      if (src & (1 << 1))
        otg_epin_handler(usbp, 1);
      if (src & (1 << 2))
        otg_epin_handler(usbp, 2);
      if (src & (1 << 3))
        otg_epin_handler(usbp, 3);
      if (src & (1 << 4))
        otg_epin_handler(usbp, 4);
      if (src & (1 << 5))
        otg_epin_handler(usbp, 5);

    }
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if STM32_USB_USE_OTG1 || defined(__DOXYGEN__)
/**
 * @brief   OTG1 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_OTG1_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  usb_lld_serve_interrupt(&USBD1);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if STM32_USB_USE_OTG2 || defined(__DOXYGEN__)
/**
 * @brief   OTG2 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_OTG2_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  usb_lld_serve_interrupt(&USBD2);

  OSAL_IRQ_EPILOGUE();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level USB driver initialization.
 *
 * @notapi
 */
void usb_lld_init(void) {

  /* Driver initialization.*/
#if STM32_USB_USE_OTG1
  usbObjectInit(&USBD1);
  USBD1.otg       = OTG_FS;
  USBD1.otgparams = &fsparams;
  rcu_usb_clock_config(RCU_CKUSB_CKPLL_DIV3_5);   
  rcu_periph_clock_enable(RCU_USBHS);
#endif

#if STM32_USB_USE_OTG2
  usbObjectInit(&USBD2);
  USBD2.otg       = OTG_HS;
  USBD2.otgparams = &hsparams;
#endif
}

static void pllusb_rcu_config(void)
{
    if (rcu_flag_get(RCU_FLAG_HXTALSTB) != SET) {
        rcu_pllusbpresel_config(RCU_PLLUSBPRESRC_IRC48M);
        rcu_pllusbpredv_config(RCU_PLLUSBPREDVSRC_HXTAL_IRC48M, RCU_PLLUSBPREDV_DIV4);
        rcu_pllusb_config(RCU_PLLUSB_MUL40);
    } else {
        rcu_pllusbpresel_config(RCU_PLLUSBPRESRC_HXTAL);
        rcu_pllusbpredv_config(RCU_PLLUSBPREDVSRC_HXTAL_IRC48M, RCU_PLLUSBPREDV_DIV5);
        rcu_pllusb_config(RCU_PLLUSB_MUL96);
    }

    RCU_ADDCTL |= RCU_ADDCTL_PLLUSBEN;
    while((RCU_ADDCTL & RCU_ADDCTL_PLLUSBSTB) == 0U) {
    }
}

/**
 * @brief   Configures and activates the USB peripheral.
 * @note    Starting the OTG cell can be a slow operation carried out with
 *          interrupts disabled, perform it before starting time-critical
 *          operations.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_start(USBDriver *usbp) {
  stm32_otg_t *otgp = usbp->otg;

  if (usbp->state == USB_STOP) {
    /* Clock activation.*/

#if STM32_USB_USE_OTG1
    if (&USBD1 == usbp) {
      otgp->GAHBCFG &= ~GAHBCFG_GINTMSK;
      otgp->GUSBCFG = (1 << 5);

      otg_core_reset(usbp);
      otgp->GCCFG = 0;
      otgp->GOTGCTL |= GOTGCTL_BVALOVAL | GOTGCTL_BVALOEN;
      otgp->GCCFG |= GCCFG_PWRDWN;
      otgp->GCCFG |= GCCFG_SOFOUTEN;

      chThdSleepMilliseconds(20);

      otgp->DCTL |= DCTL_SDIS;        //disconnect
      chThdSleepMilliseconds(3U);

      otgp->GUSBCFG &= ~(GUSBCFG_FDMOD | GUSBCFG_FHMOD);
      otgp->GUSBCFG |= GUSBCFG_FDMOD; 

      otgp->PCGCCTL = 0U;             ////PWRCLKCTL
      /* configure periodic frame interval to default value */
      otgp->DCFG &= ~DCFG_PFIVL_MASK;
      otgp->DCFG |= DCFG_PFIVL(0);    //80%

      otgp->DCFG &= ~DCFG_DSPD_MASK;
      otgp->DCFG |= DCFG_DSPD_HS;

      usb_lld_reset(usbp);

      otgp->GOTGINT  = 0xFFFFFFFF;
      otgp->GINTSTS  = 0xFFFFFFFF;
      
      /* Global interrupts enable.*/
      otgp->GAHBCFG |= GAHBCFG_GINTMSK;

      otgp->DCTL &= ~DCTL_SDIS;        //connect 
      chThdSleepMilliseconds(3U);

      pllusb_rcu_config();
      nvicEnableVector(STM32_OTG1_NUMBER, STM32_USB_OTG1_IRQ_PRIORITY);
    }
    #endif
  }
}

/**
 * @brief   Deactivates the USB peripheral.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_stop(USBDriver *usbp) {
  stm32_otg_t *otgp = usbp->otg;

  /* If in ready state then disables the USB clock.*/
  if (usbp->state != USB_STOP) {

    /* Disabling all endpoints in case the driver has been stopped while
       active.*/
    otg_disable_ep(usbp);

    otgp->DAINTMSK   = 0;
    otgp->GAHBCFG    = 0;
    otgp->GCCFG      = 0;

#if STM32_USB_USE_OTG1
    if (&USBD1 == usbp) {
      nvicDisableVector(STM32_OTG1_NUMBER);
      // rccDisableOTG_FS();
    }
#endif

#if STM32_USB_USE_OTG2
    if (&USBD2 == usbp) {
      nvicDisableVector(STM32_OTG2_NUMBER);
      rccDisableOTG_HS();
#if defined(BOARD_OTG2_USES_ULPI)
      rccDisableOTG_HSULPI()
#endif
    }
#endif
  }
}

/**
 * @brief   USB low level reset routine.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_reset(USBDriver *usbp) {
  unsigned i;
  stm32_otg_t *otgp = usbp->otg;

  /* Flush the Tx FIFO.*/
  otg_txfifo_flush(usbp, 0);

  /* Endpoint interrupts all disabled and cleared.*/
  otgp->DIEPEMPMSK = 0;
  otgp->DAINTMSK   = DAINTMSK_OEPM(0) | DAINTMSK_IEPM(0);

  /* All endpoints in NAK mode, interrupts cleared.*/
  for (i = 0; i <= usbp->otgparams->num_endpoints; i++) {
    otgp->ie[i].DIEPCTL = DIEPCTL_SNAK;
    otgp->oe[i].DOEPCTL = DOEPCTL_SNAK;
    otgp->ie[i].DIEPINT = 0xFFFFFFFF;
    otgp->oe[i].DOEPINT = 0xFFFFFFFF;
  }

  /* Resets the FIFO memory allocator.*/
  otg_ram_reset(usbp);

  /* Receive FIFO size initialization, the address is always zero.*/
  otgp->GRXFSIZ = usbp->otgparams->rx_fifo_size;                      //set rxfifo
  otg_rxfifo_flush(usbp);

  /* Resets the device address to zero.*/
  otgp->DCFG = (otgp->DCFG & ~DCFG_DAD_MASK) | DCFG_DAD(0);

  /* Enables also EP-related interrupt sources.*/
  otgp->GINTMSK  |= GINTMSK_RXFLVLM | GINTMSK_OEPM  | GINTMSK_IEPM;   //GINTEN
  otgp->DIEPMSK   = DIEPMSK_TOCM    | DIEPMSK_XFRCM;
  otgp->DOEPMSK   = DOEPMSK_STUPM   | DOEPMSK_XFRCM;

  /* EP0 initialization, it is a special case.*/
  usbp->epc[0] = &ep0config;
  otgp->oe[0].DOEPTSIZ = DOEPTSIZ_STUPCNT(3);
  otgp->oe[0].DOEPCTL = DOEPCTL_SD0PID | DOEPCTL_USBAEP | DOEPCTL_EPTYP_CTRL |
                        DOEPCTL_MPSIZ(ep0config.out_maxsize);
  otgp->ie[0].DIEPTSIZ = 0;
  otgp->ie[0].DIEPCTL = DIEPCTL_SD0PID | DIEPCTL_USBAEP | DIEPCTL_EPTYP_CTRL |
                        DIEPCTL_TXFNUM(0) | DIEPCTL_MPSIZ(ep0config.in_maxsize);
  otgp->DIEPTXF0 = DIEPTXF_INEPTXFD(ep0config.in_maxsize / 4) |
                   DIEPTXF_INEPTXSA(otg_ram_alloc(usbp,
                                                  ep0config.in_maxsize / 4));
}

/**
 * @brief   Sets the USB address.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_set_address(USBDriver *usbp) {
  stm32_otg_t *otgp = usbp->otg;

  otgp->DCFG = (otgp->DCFG & ~DCFG_DAD_MASK) | DCFG_DAD(usbp->address);
}

/**
 * @brief   Enables an endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_init_endpoint(USBDriver *usbp, usbep_t ep) {
  uint32_t ctl, fsize;
  stm32_otg_t *otgp = usbp->otg;

  /* IN and OUT common parameters.*/
  switch (usbp->epc[ep]->ep_mode & USB_EP_MODE_TYPE) {
  case USB_EP_MODE_TYPE_CTRL:
    ctl = DIEPCTL_SD0PID | DIEPCTL_USBAEP | DIEPCTL_EPTYP_CTRL;
    break;
  case USB_EP_MODE_TYPE_ISOC:
    ctl = DIEPCTL_SD0PID | DIEPCTL_USBAEP | DIEPCTL_EPTYP_ISO;
    break;
  case USB_EP_MODE_TYPE_BULK:
    ctl = DIEPCTL_SD0PID | DIEPCTL_USBAEP | DIEPCTL_EPTYP_BULK;
    break;
  case USB_EP_MODE_TYPE_INTR:
    ctl = DIEPCTL_SD0PID | DIEPCTL_USBAEP | DIEPCTL_EPTYP_INTR;
    break;
  default:
    return;
  }

  /* OUT endpoint activation or deactivation.*/
  otgp->oe[ep].DOEPTSIZ = 0;
  if (usbp->epc[ep]->out_state != NULL) {
    otgp->oe[ep].DOEPCTL = ctl | DOEPCTL_MPSIZ(usbp->epc[ep]->out_maxsize);
    otgp->DAINTMSK |= DAINTMSK_OEPM(ep);
  }
  else {
    otgp->oe[ep].DOEPCTL &= ~DOEPCTL_USBAEP;
    otgp->DAINTMSK &= ~DAINTMSK_OEPM(ep);
  }

  /* IN endpoint activation or deactivation.*/
  otgp->ie[ep].DIEPTSIZ = 0;
  if (usbp->epc[ep]->in_state != NULL) {
    /* FIFO allocation for the IN endpoint.*/
    fsize = usbp->epc[ep]->in_maxsize / 4;
    if (usbp->epc[ep]->in_multiplier > 1)
      fsize *= usbp->epc[ep]->in_multiplier;
    otgp->DIEPTXF[ep - 1] = DIEPTXF_INEPTXFD(fsize) |
                            DIEPTXF_INEPTXSA(otg_ram_alloc(usbp, fsize));
    otg_txfifo_flush(usbp, ep);

    otgp->ie[ep].DIEPCTL = ctl |
                           DIEPCTL_TXFNUM(ep) |
                           DIEPCTL_MPSIZ(usbp->epc[ep]->in_maxsize);
    otgp->DAINTMSK |= DAINTMSK_IEPM(ep);
  }
  else {
    otgp->DIEPTXF[ep - 1] = 0x02000400; /* Reset value.*/
    otg_txfifo_flush(usbp, ep);
    otgp->ie[ep].DIEPCTL &= ~DIEPCTL_USBAEP;
    otgp->DAINTMSK &= ~DAINTMSK_IEPM(ep);
  }
}

/**
 * @brief   Disables all the active endpoints except the endpoint zero.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_disable_endpoints(USBDriver *usbp) {

  /* Resets the FIFO memory allocator.*/
  otg_ram_reset(usbp);

  /* Disabling all endpoints.*/
  otg_disable_ep(usbp);
}

/**
 * @brief   Returns the status of an OUT endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @return              The endpoint status.
 * @retval EP_STATUS_DISABLED The endpoint is not active.
 * @retval EP_STATUS_STALLED  The endpoint is stalled.
 * @retval EP_STATUS_ACTIVE   The endpoint is active.
 *
 * @notapi
 */
usbepstatus_t usb_lld_get_status_out(USBDriver *usbp, usbep_t ep) {
  uint32_t ctl;

  (void)usbp;

  ctl = usbp->otg->oe[ep].DOEPCTL;
  if (!(ctl & DOEPCTL_USBAEP))
    return EP_STATUS_DISABLED;
  if (ctl & DOEPCTL_STALL)
    return EP_STATUS_STALLED;
  return EP_STATUS_ACTIVE;
}

/**
 * @brief   Returns the status of an IN endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @return              The endpoint status.
 * @retval EP_STATUS_DISABLED The endpoint is not active.
 * @retval EP_STATUS_STALLED  The endpoint is stalled.
 * @retval EP_STATUS_ACTIVE   The endpoint is active.
 *
 * @notapi
 */
usbepstatus_t usb_lld_get_status_in(USBDriver *usbp, usbep_t ep) {
  uint32_t ctl;

  (void)usbp;

  ctl = usbp->otg->ie[ep].DIEPCTL;
  if (!(ctl & DIEPCTL_USBAEP))
    return EP_STATUS_DISABLED;
  if (ctl & DIEPCTL_STALL)
    return EP_STATUS_STALLED;
  return EP_STATUS_ACTIVE;
}

/**
 * @brief   Reads a setup packet from the dedicated packet buffer.
 * @details This function must be invoked in the context of the @p setup_cb
 *          callback in order to read the received setup packet.
 * @pre     In order to use this function the endpoint must have been
 *          initialized as a control endpoint.
 * @post    The endpoint is ready to accept another packet.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @param[out] buf      buffer where to copy the packet data
 *
 * @notapi
 */
void usb_lld_read_setup(USBDriver *usbp, usbep_t ep, uint8_t *buf) {
  memcpy(buf, usbp->epc[ep]->setup_buf, 8);
}

/**
 * @brief   Starts a receive operation on an OUT endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_start_out(USBDriver *usbp, usbep_t ep) {
  uint32_t pcnt, rxsize;
  USBOutEndpointState *osp = usbp->epc[ep]->out_state;

  /* Transfer initialization.*/
  osp->totsize = osp->rxsize;                                     //总传输字节
  if ((ep == 0) && (osp->rxsize > EP0_MAX_OUTSIZE))               //端点0， 每次最大64byte
      osp->rxsize = EP0_MAX_OUTSIZE;

  /* Transaction size is rounded to a multiple of packet size because the
     following requirement in the RM:
     "For OUT transfers, the transfer size field in the endpoint's transfer
     size register must be a multiple of the maximum packet size of the
     endpoint, adjusted to the Word boundary".*/
  pcnt   = (osp->rxsize + usbp->epc[ep]->out_maxsize - 1U) /        //包的个数
           usbp->epc[ep]->out_maxsize;
  rxsize = (pcnt * usbp->epc[ep]->out_maxsize + 3U) & 0xFFFFFFFCU;  //总共的size

  /*Setting up transaction parameters in DOEPTSIZ.*/
  usbp->otg->oe[ep].DOEPTSIZ = DOEPTSIZ_STUPCNT(3) | DOEPTSIZ_PKTCNT(pcnt) |
                               DOEPTSIZ_XFRSIZ(rxsize);

  /* Special case of isochronous endpoint.*/
  if ((usbp->epc[ep]->ep_mode & USB_EP_MODE_TYPE) == USB_EP_MODE_TYPE_ISOC) {
    /* Odd/even bit toggling for isochronous endpoint.*/
    if (usbp->otg->DSTS & DSTS_FNSOF_ODD)
      usbp->otg->oe[ep].DOEPCTL |= DOEPCTL_SEVNFRM;
    else
      usbp->otg->oe[ep].DOEPCTL |= DOEPCTL_SODDFRM;
  }

  /* Starting operation.*/
  usbp->otg->oe[ep].DOEPCTL |= DOEPCTL_EPENA | DOEPCTL_CNAK;        //开启端点，设置好等中断就可以了
}

/**
 * @brief   Starts a transmit operation on an IN endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_start_in(USBDriver *usbp, usbep_t ep) {
  USBInEndpointState *isp = usbp->epc[ep]->in_state;

  /* Transfer initialization.*/
  isp->totsize = isp->txsize;
  if (isp->txsize == 0) {                                                           //长度为0
    /* Special case, sending zero size packet.*/
    usbp->otg->ie[ep].DIEPTSIZ = DIEPTSIZ_PKTCNT(1) | DIEPTSIZ_XFRSIZ(0);           //发送空的数据包
  }
  else {
    if ((ep == 0) && (isp->txsize > EP0_MAX_INSIZE))                                //端点0，最大64byte
      isp->txsize = EP0_MAX_INSIZE;

    /* Normal case.*/
    uint32_t pcnt = (isp->txsize + usbp->epc[ep]->in_maxsize - 1) /                 //0:0， 1~64:1， 表示总共发送的数据包个数
                    usbp->epc[ep]->in_maxsize;
    /* CHTODO: Support more than one packet per frame for isochronous transfers.*/
    usbp->otg->ie[ep].DIEPTSIZ = DIEPTSIZ_MCNT(1) | DIEPTSIZ_PKTCNT(pcnt) |         //数据包以及长度
                                 DIEPTSIZ_XFRSIZ(isp->txsize);                      //DIEPTSIZ_MCNT(1):非周期传输无效
  }

  /* Special case of isochronous endpoint.*/
  if ((usbp->epc[ep]->ep_mode & USB_EP_MODE_TYPE) == USB_EP_MODE_TYPE_ISOC) {
    /* Odd/even bit toggling.*/
    if (usbp->otg->DSTS & DSTS_FNSOF_ODD)
      usbp->otg->ie[ep].DIEPCTL |= DIEPCTL_SEVNFRM;
    else
      usbp->otg->ie[ep].DIEPCTL |= DIEPCTL_SODDFRM;
  }

  /* Starting operation.*/
  usbp->otg->ie[ep].DIEPCTL |= DIEPCTL_EPENA | DIEPCTL_CNAK;                        //使能端点，清除NAK标志
  usbp->otg->DIEPEMPMSK |= DIEPEMPMSK_INEPTXFEM(ep);                                //开启发送完毕中断使能
}

/**
 * @brief   Brings an OUT endpoint in the stalled state.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_stall_out(USBDriver *usbp, usbep_t ep) {

  usbp->otg->oe[ep].DOEPCTL |= DOEPCTL_STALL;
}

/**
 * @brief   Brings an IN endpoint in the stalled state.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_stall_in(USBDriver *usbp, usbep_t ep) {

  usbp->otg->ie[ep].DIEPCTL |= DIEPCTL_STALL;
}

/**
 * @brief   Brings an OUT endpoint in the active state.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_clear_out(USBDriver *usbp, usbep_t ep) {

  usbp->otg->oe[ep].DOEPCTL &= ~DOEPCTL_STALL;
}

/**
 * @brief   Brings an IN endpoint in the active state.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_clear_in(USBDriver *usbp, usbep_t ep) {

  usbp->otg->ie[ep].DIEPCTL &= ~DIEPCTL_STALL;
}

#endif /* HAL_USE_USB */

/** @} */
