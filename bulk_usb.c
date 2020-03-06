/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012,2013 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file    bulk_usb.c
 * @brief   Bulk USB Driver code.
 *
 * @addtogroup BULK_USB
 * @{
 */

#include "hal.h"
#include "bulk_usb.h"

#if 1 // HAL_USE_BULK_USB || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static bool bdu_start_receive(BulkUSBDriver *bdup) {
  uint8_t *buf;

  /* If the USB driver is not in the appropriate state then transactions
     must not be started.*/
  if ((usbGetDriverStateI(bdup->config->usbp) != USB_ACTIVE) ||
      (bdup->state != BDU_READY)) {
    return true;
  }

  /* Checking if there is already a transaction ongoing on the endpoint.*/
  if (usbGetReceiveStatusI(bdup->config->usbp, bdup->config->bulk_in)) {
    return true;
  }

  /* Checking if there is a buffer ready for incoming data.*/
  buf = ibqGetEmptyBufferI(&bdup->ibqueue);
  if (buf == NULL) {
    return true;
  }

  /* Buffer found, starting a new transaction.*/
  usbStartReceiveI(bdup->config->usbp, bdup->config->bulk_out,
                   buf, BULK_USB_BUFFERS_SIZE);

  return false;
}

/*
 * Interface implementation.
 */

static size_t _write(void *ip, const uint8_t *bp, size_t n) {

  return obqWriteTimeout(&((BulkUSBDriver *)ip)->obqueue, bp,
                         n, TIME_INFINITE);
}

static size_t _read(void *ip, uint8_t *bp, size_t n) {

  return ibqReadTimeout(&((BulkUSBDriver *)ip)->ibqueue, bp,
                        n, TIME_INFINITE);
}

static msg_t _put(void *ip, uint8_t b) {

  return obqPutTimeout(&((BulkUSBDriver *)ip)->obqueue, b, TIME_INFINITE);
}

static msg_t _get(void *ip) {

  return ibqGetTimeout(&((BulkUSBDriver *)ip)->ibqueue, TIME_INFINITE);
}

static msg_t _putt(void *ip, uint8_t b, sysinterval_t timeout) {

  return obqPutTimeout(&((BulkUSBDriver *)ip)->obqueue, b, timeout);
}

static msg_t _gett(void *ip, sysinterval_t timeout) {

  return ibqGetTimeout(&((BulkUSBDriver *)ip)->ibqueue, timeout);
}

static size_t _writet(void *ip, const uint8_t *bp, size_t n,
                      sysinterval_t timeout) {

  return obqWriteTimeout(&((BulkUSBDriver *)ip)->obqueue, bp, n, timeout);
}

static size_t _readt(void *ip, uint8_t *bp, size_t n,
                     sysinterval_t timeout) {

  return ibqReadTimeout(&((BulkUSBDriver *)ip)->ibqueue, bp, n, timeout);
}

static msg_t _ctl(void *ip, unsigned int operation, void *arg) {
  BulkUSBDriver *bdup = (BulkUSBDriver *)ip;

  osalDbgCheck(bdup != NULL);

  switch (operation) {
  case CHN_CTL_NOP:
    osalDbgCheck(arg == NULL);
    break;
  case CHN_CTL_INVALID:
    osalDbgAssert(false, "invalid CTL operation");
    break;
  default:
#if defined(BDU_LLD_IMPLEMENTS_CTL)
    /* The BDU driver does not have a LLD but the application can use this
       hook to implement extra controls by supplying this function.*/ 
    extern msg_t bdu_lld_control(BulkUSBDriver *bdup,
                                 unsigned int operation,
                                 void *arg);
    return bdu_lld_control(bdup, operation, arg);
#else
    break;
#endif
  }
  return MSG_OK;
}

static const struct BulkUSBDriverVMT vmt = {
  (size_t)0,
  _write, _read, _put, _get,
  _putt, _gett, _writet, _readt,
  _ctl
};

/**
 * @brief   Notification of empty buffer released into the input buffers queue.
 *
 * @param[in] bqp       the buffers queue pointer.
 */
static void ibnotify(io_buffers_queue_t *bqp) {
  BulkUSBDriver *bdup = bqGetLinkX(bqp);
  (void) bdu_start_receive(bdup);
}

/**
 * @brief   Notification of filled buffer inserted into the output buffers queue.
 *
 * @param[in] bqp       the buffers queue pointer.
 */
static void obnotify(io_buffers_queue_t *bqp) {
  size_t n;
  BulkUSBDriver *bdup = bqGetLinkX(bqp);

  /* If the USB driver is not in the appropriate state then transactions
     must not be started.*/
  if ((usbGetDriverStateI(bdup->config->usbp) != USB_ACTIVE) ||
      (bdup->state != BDU_READY))
    return;

  /* Checking if there is already a transaction ongoing on the endpoint.*/
  if (!usbGetTransmitStatusI(bdup->config->usbp, bdup->config->bulk_in)) {
    /* Getting a full buffer, a buffer is available for sure because this
       callback is invoked when one has been inserted.*/
    uint8_t *buf = obqGetFullBufferI(&bdup->obqueue, &n);
    osalDbgAssert(buf != NULL, "buffer not found");
    usbStartTransmitI(bdup->config->usbp, bdup->config->bulk_in, buf, n);
  }
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Bulk USB Driver initialization.
 * @note    This function is implicitly invoked by @p halInit(), there is
 *          no need to explicitly initialize the driver.
 *
 * @init
 */
void bduInit(void) {
}

/**
 * @brief   Initializes a generic full duplex driver object.
 * @details The HW dependent part of the initialization has to be performed
 *          outside, usually in the hardware initialization code.
 *
 * @param[out] bdup     pointer to a @p BulkUSBDriver structure
 *
 * @init
 */
void bduObjectInit(BulkUSBDriver *bdup) {

  bdup->vmt = &vmt;
  osalEventObjectInit(&bdup->event);
  bdup->state = BDU_STOP;
  ibqObjectInit(&bdup->ibqueue, true, bdup->ib,
                BULK_USB_BUFFERS_SIZE, BULK_USB_BUFFERS_NUMBER,
                ibnotify, bdup);
  obqObjectInit(&bdup->obqueue, true, bdup->ob,
                BULK_USB_BUFFERS_SIZE, BULK_USB_BUFFERS_NUMBER,
                obnotify, bdup);
}

/**
 * @brief   Configures and starts the driver.
 *
 * @param[in] bdup      pointer to a @p BulkUSBDriver object
 * @param[in] config    the Bulk USB driver configuration
 *
 * @api
 */
void bduStart(BulkUSBDriver *bdup, const BulkUSBConfig *config) {
  USBDriver *usbp = config->usbp;

  osalDbgCheck(bdup != NULL);

  osalSysLock();
  osalDbgAssert((bdup->state == BDU_STOP) || (bdup->state == BDU_READY),
                "invalid state");
  usbp->in_params[config->bulk_in - 1U]   = bdup;
  usbp->out_params[config->bulk_out - 1U] = bdup;
  bdup->config = config;
  bdup->state = BDU_READY;
  osalSysUnlock();
}

/**
 * @brief   Stops the driver.
 * @details Any thread waiting on the driver's queues will be awakened with
 *          the message @p MSG_RESET.
 *
 * @param[in] bdup      pointer to a @p BulkUSBDriver object
 *
 * @api
 */
void bduStop(BulkUSBDriver *bdup) {
  USBDriver *usbp = bdup->config->usbp;

  osalDbgCheck(bdup != NULL);

  osalSysLock();

  osalDbgAssert((bdup->state == BDU_STOP) || (bdup->state == BDU_READY),
                "invalid state");

  /* Driver in stopped state.*/
  usbp->in_params[bdup->config->bulk_in - 1U]   = NULL;
  usbp->out_params[bdup->config->bulk_out - 1U] = NULL;
  bdup->config = NULL;
  bdup->state = BDU_STOP;

  /* Enforces a disconnection.*/
  chnAddFlagsI(bdup, CHN_DISCONNECTED);
  ibqResetI(&bdup->ibqueue);
  obqResetI(&bdup->obqueue);
  osalOsRescheduleS();

  osalSysUnlock();
}

/**
 * @brief   USB device suspend handler.
 * @details Generates a @p CHN_DISCONNECT event and puts queues in
 *          non-blocking mode, this way the application cannot get stuck
 *          in the middle of an I/O operations.
 * @note    If this function is not called from an ISR then an explicit call
 *          to @p osalOsRescheduleS() in necessary afterward.
 *
 * @param[in] bdup      pointer to a @p BulkUSBDriver object
 *
 * @iclass
 */
void bduSuspendHookI(BulkUSBDriver *bdup) {

  /* Avoiding events spam.*/
  if(bqIsSuspendedX(&bdup->ibqueue) && bqIsSuspendedX(&bdup->obqueue)) {
    return;
  }
  chnAddFlagsI(bdup, CHN_DISCONNECTED);
  bqSuspendI(&bdup->ibqueue);
  bqSuspendI(&bdup->obqueue);
}

/**
 * @brief   USB device wakeup handler.
 * @details Generates a @p CHN_CONNECT event and resumes normal queues
 *          operations.
 *
 * @note    If this function is not called from an ISR then an explicit call
 *          to @p osalOsRescheduleS() in necessary afterward.
 *
 * @param[in] bdup      pointer to a @p BulkUSBDriver object
 *
 * @iclass
 */
void bduWakeupHookI(BulkUSBDriver *bdup) {

  chnAddFlagsI(bdup, CHN_CONNECTED);
  bqResumeX(&bdup->ibqueue);
  bqResumeX(&bdup->obqueue);
}

/**
 * @brief   USB device configured handler.
 *
 * @param[in] bdup      pointer to a @p BulkUSBDriver object
 *
 * @iclass
 */
void bduConfigureHookI(BulkUSBDriver *bdup) {

  ibqResetI(&bdup->ibqueue);
  bqResumeX(&bdup->ibqueue);
  obqResetI(&bdup->obqueue);
  bqResumeX(&bdup->obqueue);
  chnAddFlagsI(bdup, CHN_CONNECTED);
  (void) bdu_start_receive(bdup);
}

/**
 * @brief   Default requests hook.
 * @details Applications wanting to use the Bulk USB driver can use
 *          this function as requests hook in the USB configuration.
 *          The following requests are emulated:
 *          - CDC_GET_LINE_CODING.
 *          - CDC_SET_LINE_CODING.
 *          - CDC_SET_CONTROL_LINE_STATE.
 *          .
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @return              The hook status.
 * @retval true         Message handled internally.
 * @retval false        Message not handled.
 */
bool bduRequestsHook(USBDriver *usbp) {

  (void)usbp;
  return FALSE;
}

/**
 * @brief   SOF handler.
 * @details The SOF interrupt is used for automatic flushing of incomplete
 *          buffers pending in the output queue.
 *
 * @param[in] bdup      pointer to a @p BulkUSBDriver object
 *
 * @iclass
 */
void bduSOFHookI(BulkUSBDriver *bdup) {

  /* If the USB driver is not in the appropriate state then transactions
     must not be started.*/
  if ((usbGetDriverStateI(bdup->config->usbp) != USB_ACTIVE) ||
      (bdup->state != BDU_READY)) {
    return;
  }

  /* If there is already a transaction ongoing then another one cannot be
     started.*/
  if (usbGetTransmitStatusI(bdup->config->usbp, bdup->config->bulk_in)) {
    return;
  }

  /* Checking if there only a buffer partially filled, if so then it is
     enforced in the queue and transmitted.*/
  if (obqTryFlushI(&bdup->obqueue)) {
    size_t n;
    uint8_t *buf = obqGetFullBufferI(&bdup->obqueue, &n);

    osalDbgAssert(buf != NULL, "queue is empty");

    usbStartTransmitI(bdup->config->usbp, bdup->config->bulk_in, buf, n);
  }
}

/**
 * @brief   Default data transmitted callback.
 * @details The application must use this function as callback for the IN
 *          data endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        IN endpoint number
 */
void bduDataTransmitted(USBDriver *usbp, usbep_t ep) {
  uint8_t *buf;
  size_t n;
  BulkUSBDriver *bdup = usbp->in_params[ep - 1U];

  if (bdup == NULL) {
    return;
  }

  osalSysLockFromISR();

  /* Signaling that space is available in the output queue.*/
  chnAddFlagsI(bdup, CHN_OUTPUT_EMPTY);

  /* Freeing the buffer just transmitted, if it was not a zero size packet.*/
  if (usbp->epc[ep]->in_state->txsize > 0U) {
    obqReleaseEmptyBufferI(&bdup->obqueue);
  }

  /* Checking if there is a buffer ready for transmission.*/
  buf = obqGetFullBufferI(&bdup->obqueue, &n);

  if (buf != NULL) {
    /* The endpoint cannot be busy, we are in the context of the callback,
       so it is safe to transmit without a check.*/
    usbStartTransmitI(usbp, ep, buf, n);
  }
  else if ((usbp->epc[ep]->in_state->txsize > 0U) &&
           ((usbp->epc[ep]->in_state->txsize &
            ((size_t)usbp->epc[ep]->in_maxsize - 1U)) == 0U)) {
    /* Transmit zero sized packet in case the last one has maximum allowed
       size. Otherwise the recipient may expect more data coming soon and
       not return buffered data to app. See section 5.8.3 Bulk Transfer
       Packet Size Constraints of the USB Specification document.*/
    usbStartTransmitI(usbp, ep, usbp->setup, 0);

  }
  else {
    /* Nothing to transmit.*/
  }

  osalSysUnlockFromISR();
}

/**
 * @brief   Default data received callback.
 * @details The application must use this function as callback for the OUT
 *          data endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        OUT endpoint number
 */
void bduDataReceived(USBDriver *usbp, usbep_t ep) {
  size_t size;
  BulkUSBDriver *bdup = usbp->out_params[ep - 1U];

  if (bdup == NULL) {
    return;
  }

  osalSysLockFromISR();

  /* Checking for zero-size transactions.*/
  size = usbGetReceiveTransactionSizeX(bdup->config->usbp,
                                       bdup->config->bulk_out);
  if (size > (size_t)0) {
    /* Signaling that data is available in the input queue.*/
    chnAddFlagsI(bdup, CHN_INPUT_AVAILABLE);

    /* Posting the filled buffer in the queue.*/
    ibqPostFullBufferI(&bdup->ibqueue, size);
  }

  /* The endpoint cannot be busy, we are in the context of the callback,
     so a packet is in the buffer for sure. Trying to get a free buffer
     for the next transaction.*/
  (void) bdu_start_receive(bdup);

  osalSysUnlockFromISR();
}

/**
 * @brief   Default data received callback.
 * @details The application must use this function as callback for the IN
 *          interrupt endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 */
void bduInterruptTransmitted(USBDriver *usbp, usbep_t ep) {

  (void)usbp;
  (void)ep;
}

/**
 * @brief   Control operation on a Bulk USB port.
 *
 * @param[in] usbp       pointer to a @p USBDriver object
 * @param[in] operation control operation code
 * @param[in,out] arg   operation argument
 *
 * @return              The control operation status.
 * @retval MSG_OK       in case of success.
 * @retval MSG_TIMEOUT  in case of operation timeout.
 * @retval MSG_RESET    in case of operation reset.
 *
 * @api
 */
msg_t bduControl(USBDriver *usbp, unsigned int operation, void *arg) {

  return _ctl((void *)usbp, operation, arg);
}

#endif /* HAL_USE_BULK_USB */

/** @} */
