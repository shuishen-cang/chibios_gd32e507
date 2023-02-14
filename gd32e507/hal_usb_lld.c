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
 * @file    hal_usb_lld.c
 * @brief   PLATFORM USB subsystem low level driver source.
 *
 * @addtogroup USB
 * @{
 */

#include "hal.h"


#if (HAL_USE_USB == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   USB1 driver identifier.
 */
#if (PLATFORM_USB_USE_USB1 == TRUE) || defined(__DOXYGEN__)
USBDriver USBD1;
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
  &ep0_state.out
};

static uint32_t kkk = 0;
OSAL_IRQ_HANDLER(Vector14C) {

  OSAL_IRQ_PROLOGUE();

  kkk ++;

  // usb_lld_serve_interrupt(&USBD1);

  OSAL_IRQ_EPILOGUE();
}

#define RX_FIFO_SIZE                          512U
#define TX0_FIFO_SIZE                         128U
#define TX1_FIFO_SIZE                         384U
#define TX2_FIFO_SIZE                         0U
#define TX3_FIFO_SIZE                         0U
#define TX4_FIFO_SIZE                         0U
#define TX5_FIFO_SIZE                         0U

// const uint16_t USBHS_TX_FIFO_SIZE[USBHS_MAX_EP_COUNT] = 
// {
//     (uint16_t)TX0_FIFO_SIZE,
//     (uint16_t)TX1_FIFO_SIZE,
//     (uint16_t)TX2_FIFO_SIZE,
//     (uint16_t)TX3_FIFO_SIZE,
//     (uint16_t)TX4_FIFO_SIZE,
//     (uint16_t)TX5_FIFO_SIZE
// };

extern const uint16_t USBHS_TX_FIFO_SIZE[USBHS_MAX_EP_COUNT];

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers and threads.                                    */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

static void usb_set_txfifo(usb_gr *gr, uint8_t fifo, uint16_t size)
{
    uint32_t tx_offset;

    tx_offset = gr->GRFLEN;

    if (fifo == 0U) {
        gr->DIEP0TFLEN_HNPTFLEN = ((uint32_t)size << 16U) | tx_offset;
    } else {
        tx_offset += (gr->DIEP0TFLEN_HNPTFLEN) >> 16U;

        for (uint8_t i = 0U; i < (fifo - 1U); i++) {
            tx_offset += (gr->DIEPTFLEN[i] >> 16U);
        }

        gr->DIEPTFLEN[fifo - 1U] = ((uint32_t)size << 16U) | tx_offset;
    }
}

static int32_t usb_txfifo_flush (usb_gr *gr, uint8_t fifo_num)
{
    gr->GRSTCTL = ((uint32_t)fifo_num << 6U) | GRSTCTL_TXFF;

    /* wait for Tx FIFO flush bit is set */
    while (gr->GRSTCTL & GRSTCTL_TXFF) {
        /* no operation */
    }

    /* wait for 3 PHY clocks*/
    chThdSleepMicroseconds(5);

    return 0;
}

/*!
    \brief      flush the entire Rx FIFO
    \param[in]  usb_regs: pointer to USB core registers
    \param[out] none
    \retval     operation status
*/
static int32_t usb_rxfifo_flush (usb_gr *gr)
{
    gr->GRSTCTL = GRSTCTL_RXFF;

    /* wait for RX FIFO flush bit is set */
    while (gr->GRSTCTL & GRSTCTL_RXFF) {
        /* no operation */
    }

    /* wait for 3 PHY clocks */
    chThdSleepMicroseconds(5);

    return 0;
}

static int32_t usb_devint_enable (usb_gr *gr)
{
    /* clear any pending USB OTG interrupts */
    gr->GOTGINTF = 0xFFFFFFFFU;

    /* clear any pending interrupts */
    gr->GINTF = 0xBFFFFFFFU;

    /* enable the USB wakeup and suspend interrupts */
    gr->GINTEN = GINTEN_WKUPIE | GINTEN_SPIE;

    /* enable device_mode-related interrupts */
    // if ((uint8_t)USB_USE_FIFO == udev->bp.transfer_mode) {
    gr->GINTEN |= GINTEN_RXFNEIE;
    // }

    gr->GINTEN |= GINTEN_RSTIE | GINTEN_ENUMFIE | GINTEN_IEPIE |\
                             GINTEN_OEPIE | GINTEN_SOFIE | GINTEN_ISOONCIE | GINTEN_ISOINCIE;

#ifdef VBUS_SENSING_ENABLED
    gr->GINTEN |= GINTEN_SESIE | GINTEN_OTGIE;
#endif /* VBUS_SENSING_ENABLED */


    /* enable USB global interrupt */
    gr->GAHBCS |= GAHBCS_GINTEN;

    return 0;
}


/**
 * @brief   Low level USB driver initialization.
 *
 * @notapi
 */
void usb_lld_init(void) {

#if PLATFORM_USB_USE_USB1 == TRUE
  /* Driver initialization.*/
    USBD1.dev_gr = (usb_gr*)USBHS_REG_BASE;
    USBD1.dev_dr = (usb_dr*)(USBHS_REG_BASE + 0x0800);
    usbObjectInit(&USBD1);

    rcu_usb_clock_config(RCU_CKUSB_CKPLL_DIV3_5);   
    rcu_periph_clock_enable(RCU_USBHS);
#endif
}

/**
 * @brief   Configures and activates the USB peripheral.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_start(USBDriver *usbp) {

  if (usbp->state == USB_STOP) {
    /* Enables the peripheral.*/
#if PLATFORM_USB_USE_USB1 == TRUE
    if (&USBD1 == usbp) {
    /* disable USB global interrupt */
      usbp->dev_gr->GAHBCS &= ~GAHBCS_GINTEN;
      usbp->dev_gr->GUSBCS |= GUSBCS_EMBPHY_HS;

      usbp->dev_gr->GRSTCTL |= GRSTCTL_CSRST;
      while (usbp->dev_gr->GRSTCTL & GRSTCTL_CSRST) {}
      chThdSleepMicroseconds(4);

      usbp->dev_gr->GCCFG = 0U;
      usbp->dev_gr->GOTGCS |= GOTGCS_BVOV | GOTGCS_BVOE;
      usbp->dev_gr->GCCFG |= GCCFG_PWRON;

      usbp->dev_gr->GCCFG |= GCCFG_SOFOEN;          //sof enable
      chThdSleepMilliseconds(20);   

      usbp->dev_dr->DCTL |= DCTL_SD;               //soft restart
      chThdSleepMilliseconds(20); 

      usbp->dev_gr->GUSBCS &= ~(GUSBCS_FDM | GUSBCS_FHM);
      usbp->dev_gr->GUSBCS |= GUSBCS_FDM;          //set ad device


      usbp->dev_dr->DCFG &= ~DCFG_EOPFT;
      usbp->dev_dr->DCFG |= FRAME_INTERVAL_80;
      usbp->dev_dr->DCFG &= ~DCFG_DS;
      usbp->dev_dr->DCFG |= USB_SPEED_INP_HIGH;    //high speed

      usbp->dev_gr->GRFLEN = RX_FIFO_SIZE;          //recv fifo 512byte

      for (uint8_t i = 0U; i < USBHS_MAX_EP_COUNT; i++) {						//send fifo
          usb_set_txfifo(usbp->dev_gr, i, USBHS_TX_FIFO_SIZE[i]);
      }

      (void)usb_txfifo_flush (usbp->dev_gr, 0x10U);
      (void)usb_rxfifo_flush (usbp->dev_gr);

      usbp->dev_dr->DIEPINTEN = 0U;
      usbp->dev_dr->DOEPINTEN = 0U;
      usbp->dev_dr->DAEPINT = 0xFFFFFFFFU;
      usbp->dev_dr->DAEPINTEN = 0U;

      //initial endpoint
      usbp->dev_dr->DIEPINTEN |= DIEPINTEN_EPTXFUDEN;

      usb_devint_enable (usbp->dev_gr);

      usbp->dev_dr->DCTL &= ~DCTL_SD;

      nvic_irq_enable((uint8_t)USBHS_IRQn, 2U, 0U);
    }
#endif
  }
  /* Configures the peripheral.*/
}

/**
 * @brief   Deactivates the USB peripheral.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_stop(USBDriver *usbp) {

  if (usbp->state == USB_READY) {
    /* Resets the peripheral.*/

    /* Disables the peripheral.*/
#if PLATFORM_USB_USE_USB1 == TRUE
    if (&USBD1 == usbp) {

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

  /* Post reset initialization.*/

  /* EP0 initialization.*/
  usbp->epc[0] = &ep0config;
  usb_lld_init_endpoint(usbp, 0);
}

/**
 * @brief   Sets the USB address.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_set_address(USBDriver *usbp) {

  (void)usbp;

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

  (void)usbp;
  (void)ep;

}

/**
 * @brief   Disables all the active endpoints except the endpoint zero.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_disable_endpoints(USBDriver *usbp) {

  (void)usbp;

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

  (void)usbp;
  (void)ep;

  return EP_STATUS_DISABLED;
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

  (void)usbp;
  (void)ep;

  return EP_STATUS_DISABLED;
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

  (void)usbp;
  (void)ep;
  (void)buf;

}

/**
 * @brief   Prepares for a receive operation.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_prepare_receive(USBDriver *usbp, usbep_t ep) {

  (void)usbp;
  (void)ep;

}

/**
 * @brief   Prepares for a transmit operation.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_prepare_transmit(USBDriver *usbp, usbep_t ep) {

  (void)usbp;
  (void)ep;

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

  (void)usbp;
  (void)ep;

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

  (void)usbp;
  (void)ep;

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

  (void)usbp;
  (void)ep;

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

  (void)usbp;
  (void)ep;

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

  (void)usbp;
  (void)ep;

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

  (void)usbp;
  (void)ep;

}

#endif /* HAL_USE_USB == TRUE */

/** @} */
