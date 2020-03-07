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
 * @file    midi_usb.c
 * @brief   Midi USB Driver code.
 *
 * @addtogroup MIDI_USB
 * @{
 */

#include "hal.h"
#include "midi_usb.h"
#include "usbcfg.h"

#define MIDISEND_TIMEOUT TIME_IMMEDIATE

#if 1 // HAL_USE_MIDI_USB || defined(__DOXYGEN__)

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

static bool mdu_start_receive(MidiUSBDriver *mdup) {
  uint8_t *buf;

  /* If the USB driver is not in the appropriate state then transactions
     must not be started.*/
  if ((usbGetDriverStateI(mdup->config->usbp) != USB_ACTIVE) ||
      (mdup->state != MDU_READY)) {
    return true;
  }

  /* Checking if there is already a transaction ongoing on the endpoint.*/
  if (usbGetReceiveStatusI(mdup->config->usbp, mdup->config->bulk_in)) {
    return true;
  }

  /* Checking if there is a buffer ready for incoming data.*/
  buf = ibqGetEmptyBufferI(&mdup->ibqueue);
  if (buf == NULL) {
    return true;
  }

  /* Buffer found, starting a new transaction.*/
  usbStartReceiveI(mdup->config->usbp, mdup->config->bulk_out,
                   buf, MIDI_USB_BUFFERS_SIZE);

  return false;
}

/*
 * Interface implementation.
 */

static size_t _write(void *ip, const uint8_t *bp, size_t n) {

  return obqWriteTimeout(&((MidiUSBDriver *)ip)->obqueue, bp,
                         n, TIME_INFINITE);
}

static size_t _read(void *ip, uint8_t *bp, size_t n) {

  return ibqReadTimeout(&((MidiUSBDriver *)ip)->ibqueue, bp,
                        n, TIME_INFINITE);
}

static msg_t _put(void *ip, uint8_t b) {

  return obqPutTimeout(&((MidiUSBDriver *)ip)->obqueue, b, TIME_INFINITE);
}

static msg_t _get(void *ip) {

  return ibqGetTimeout(&((MidiUSBDriver *)ip)->ibqueue, TIME_INFINITE);
}

static msg_t _putt(void *ip, uint8_t b, sysinterval_t timeout) {

  return obqPutTimeout(&((MidiUSBDriver *)ip)->obqueue, b, timeout);
}

static msg_t _gett(void *ip, sysinterval_t timeout) {

  return ibqGetTimeout(&((MidiUSBDriver *)ip)->ibqueue, timeout);
}

static size_t _writet(void *ip, const uint8_t *bp, size_t n,
                      sysinterval_t timeout) {

  return obqWriteTimeout(&((MidiUSBDriver *)ip)->obqueue, bp, n, timeout);
}

static size_t _readt(void *ip, uint8_t *bp, size_t n,
                     sysinterval_t timeout) {

  return ibqReadTimeout(&((MidiUSBDriver *)ip)->ibqueue, bp, n, timeout);
}

static msg_t _ctl(void *ip, unsigned int operation, void *arg) {
  MidiUSBDriver *mdup = (MidiUSBDriver *)ip;

  osalDbgCheck(mdup != NULL);

  switch (operation) {
  case CHN_CTL_NOP:
    osalDbgCheck(arg == NULL);
    break;
  case CHN_CTL_INVALID:
    osalDbgAssert(false, "invalid CTL operation");
    break;
  default:
#if defined(MDU_LLD_IMPLEMENTS_CTL)
    /* The MDU driver does not have a LLD but the application can use this
       hook to implement extra controls by supplying this function.*/ 
    extern msg_t mdu_lld_control(MidiUSBDriver *mdup,
                                 unsigned int operation,
                                 void *arg);
    return mdu_lld_control(mdup, operation, arg);
#else
    break;
#endif
  }
  return MSG_OK;
}

static const struct MidiUSBDriverVMT vmt = {
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
  MidiUSBDriver *mdup = bqGetLinkX(bqp);
  (void) mdu_start_receive(mdup);
}

/**
 * @brief   Notification of filled buffer inserted into the output buffers queue.
 *
 * @param[in] bqp       the buffers queue pointer.
 */
static void obnotify(io_buffers_queue_t *bqp) {
  size_t n;
  MidiUSBDriver *mdup = bqGetLinkX(bqp);

  /* If the USB driver is not in the appropriate state then transactions
     must not be started.*/
  if ((usbGetDriverStateI(mdup->config->usbp) != USB_ACTIVE) ||
      (mdup->state != MDU_READY))
    return;

  /* Checking if there is already a transaction ongoing on the endpoint.*/
  if (!usbGetTransmitStatusI(mdup->config->usbp, mdup->config->bulk_in)) {
    /* Getting a full buffer, a buffer is available for sure because this
       callback is invoked when one has been inserted.*/
    uint8_t *buf = obqGetFullBufferI(&mdup->obqueue, &n);
    osalDbgAssert(buf != NULL, "buffer not found");
    if (!(n & 3)) { // skip if not a multiple of 4 bytes
      usbStartTransmitI(mdup->config->usbp, mdup->config->bulk_in, buf, n);
    }
  }
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Midi USB Driver initialization.
 * @note    This function is implicitly invoked by @p halInit(), there is
 *          no need to explicitly initialize the driver.
 *
 * @init
 */
void mduInit(void) {
}

/**
 * @brief   Initializes a generic full duplex driver object.
 * @details The HW dependent part of the initialization has to be performed
 *          outside, usually in the hardware initialization code.
 *
 * @param[out] mdup     pointer to a @p MidiUSBDriver structure
 *
 * @init
 */
void mduObjectInit(MidiUSBDriver *mdup) {

  mdup->vmt = &vmt;
  osalEventObjectInit(&mdup->event);
  mdup->state = MDU_STOP;
  ibqObjectInit(&mdup->ibqueue, true, mdup->ib,
                MIDI_USB_BUFFERS_SIZE, MIDI_USB_BUFFERS_NUMBER,
                ibnotify, mdup);
  obqObjectInit(&mdup->obqueue, true, mdup->ob,
                MIDI_USB_BUFFERS_SIZE, MIDI_USB_BUFFERS_NUMBER,
                obnotify, mdup);
}

/**
 * @brief   Configures and starts the driver.
 *
 * @param[in] mdup      pointer to a @p MidiUSBDriver object
 * @param[in] config    the Midi USB driver configuration
 *
 * @api
 */
void mduStart(MidiUSBDriver *mdup, const MidiUSBConfig *config) {
  USBDriver *usbp = config->usbp;

  osalDbgCheck(mdup != NULL);

  osalSysLock();
  osalDbgAssert((mdup->state == MDU_STOP) || (mdup->state == MDU_READY),
                "invalid state");
  usbp->in_params[config->bulk_in - 1U]   = mdup;
  usbp->out_params[config->bulk_out - 1U] = mdup;
  mdup->config = config;
  mdup->state = MDU_READY;
  osalSysUnlock();
}

/**
 * @brief   Stops the driver.
 * @details Any thread waiting on the driver's queues will be awakened with
 *          the message @p MSG_RESET.
 *
 * @param[in] mdup      pointer to a @p MidiUSBDriver object
 *
 * @api
 */
void mduStop(MidiUSBDriver *mdup) {
  USBDriver *usbp = mdup->config->usbp;

  osalDbgCheck(mdup != NULL);

  osalSysLock();

  osalDbgAssert((mdup->state == MDU_STOP) || (mdup->state == MDU_READY),
                "invalid state");

  /* Driver in stopped state.*/
  usbp->in_params[mdup->config->bulk_in - 1U]   = NULL;
  usbp->out_params[mdup->config->bulk_out - 1U] = NULL;
  mdup->config = NULL;
  mdup->state = MDU_STOP;

  /* Enforces a disconnection.*/
  chnAddFlagsI(mdup, CHN_DISCONNECTED);
  ibqResetI(&mdup->ibqueue);
  obqResetI(&mdup->obqueue);
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
 * @param[in] mdup      pointer to a @p MidiUSBDriver object
 *
 * @iclass
 */
void mduSuspendHookI(MidiUSBDriver *mdup) {

  /* Avoiding events spam.*/
  if(bqIsSuspendedX(&mdup->ibqueue) && bqIsSuspendedX(&mdup->obqueue)) {
    return;
  }
  chnAddFlagsI(mdup, CHN_DISCONNECTED);
  bqSuspendI(&mdup->ibqueue);
  bqSuspendI(&mdup->obqueue);
}

/**
 * @brief   USB device wakeup handler.
 * @details Generates a @p CHN_CONNECT event and resumes normal queues
 *          operations.
 *
 * @note    If this function is not called from an ISR then an explicit call
 *          to @p osalOsRescheduleS() in necessary afterward.
 *
 * @param[in] mdup      pointer to a @p MidiUSBDriver object
 *
 * @iclass
 */
void mduWakeupHookI(MidiUSBDriver *mdup) {

  chnAddFlagsI(mdup, CHN_CONNECTED);
  bqResumeX(&mdup->ibqueue);
  bqResumeX(&mdup->obqueue);
}

/**
 * @brief   USB device configured handler.
 *
 * @param[in] mdup      pointer to a @p MidiUSBDriver object
 *
 * @iclass
 */
void mduConfigureHookI(MidiUSBDriver *mdup) {

  ibqResetI(&mdup->ibqueue);
  bqResumeX(&mdup->ibqueue);
  obqResetI(&mdup->obqueue);
  bqResumeX(&mdup->obqueue);
  chnAddFlagsI(mdup, CHN_CONNECTED);
  (void) mdu_start_receive(mdup);
}

/**
 * @brief   Default requests hook.
 * @details Applications wanting to use the Midi USB driver can use
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
bool mduRequestsHook(USBDriver *usbp) {

  (void)usbp;
  return FALSE;
}

/**
 * @brief   SOF handler.
 * @details The SOF interrupt is used for automatic flushing of incomplete
 *          buffers pending in the output queue.
 *
 * @param[in] mdup      pointer to a @p MidiUSBDriver object
 *
 * @iclass
 */
void mduSOFHookI(MidiUSBDriver *mdup) {

  /* If the USB driver is not in the appropriate state then transactions
     must not be started.*/
  if ((usbGetDriverStateI(mdup->config->usbp) != USB_ACTIVE) ||
      (mdup->state != MDU_READY)) {
    return;
  }

  /* If there is already a transaction ongoing then another one cannot be
     started.*/
  if (usbGetTransmitStatusI(mdup->config->usbp, mdup->config->bulk_in)) {
    return;
  }

  /* Checking if there only a buffer partially filled, if so then it is
     enforced in the queue and transmitted.*/
  if (obqTryFlushI(&mdup->obqueue)) {
    size_t n;
    uint8_t *buf = obqGetFullBufferI(&mdup->obqueue, &n);

    osalDbgAssert(buf != NULL, "queue is empty");

    usbStartTransmitI(mdup->config->usbp, mdup->config->bulk_in, buf, n);
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
void mduDataTransmitted(USBDriver *usbp, usbep_t ep) {
  uint8_t *buf;
  size_t n;
  MidiUSBDriver *mdup = usbp->in_params[ep - 1U];

  if (mdup == NULL) {
    return;
  }

  osalSysLockFromISR();

  /* Signaling that space is available in the output queue.*/
  chnAddFlagsI(mdup, CHN_OUTPUT_EMPTY);

  /* Freeing the buffer just transmitted, if it was not a zero size packet.*/
  if (usbp->epc[ep]->in_state->txsize > 0U) {
    obqReleaseEmptyBufferI(&mdup->obqueue);
  }

  /* Checking if there is a buffer ready for transmission.*/
  buf = obqGetFullBufferI(&mdup->obqueue, &n);

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
       not return buffered data to app. See section 5.8.3 Midi Transfer
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
void mduDataReceived(USBDriver *usbp, usbep_t ep) {
  size_t size;
  MidiUSBDriver *mdup = usbp->out_params[ep - 1U];

  if (mdup == NULL) {
    return;
  }

  osalSysLockFromISR();

  /* Checking for zero-size transactions.*/
  size = usbGetReceiveTransactionSizeX(mdup->config->usbp,
                                       mdup->config->bulk_out);
  if (size > (size_t)0) {
    /* Signaling that data is available in the input queue.*/
    chnAddFlagsI(mdup, CHN_INPUT_AVAILABLE);

    /* Posting the filled buffer in the queue.*/
    ibqPostFullBufferI(&mdup->ibqueue, size);
  }

  /* The endpoint cannot be busy, we are in the context of the callback,
     so a packet is in the buffer for sure. Trying to get a free buffer
     for the next transaction.*/
  (void) mdu_start_receive(mdup);

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
void mduInterruptTransmitted(USBDriver *usbp, usbep_t ep) {

  (void)usbp;
  (void)ep;
}

/**
 * @brief   Control operation on a Midi USB port.
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
msg_t mduControl(USBDriver *usbp, unsigned int operation, void *arg) {

  return _ctl((void *)usbp, operation, arg);
}

// the Send etc, work for everything except Sysex
uint8_t calcDS1(uint8_t b0) {
// this works for everything bar SysEx,
// for sysex you need to use 0x4-0x7 to pack messages
  return (b0 & 0xF0) >> 4;
}

uint8_t calcCIN1(uint8_t port, uint8_t b0) {
  uint8_t ds = calcDS1(b0);
  uint8_t cin = (((port - 1) & 0x0F) << 4) | ds;
  return cin;
}

void midi_usb_MidiSend1(uint8_t port, uint8_t b0) {
  uint8_t tx[4];
  tx[0] = calcCIN1(port, b0);
  tx[1] = b0;
  tx[2] = 0;
  tx[3] = 0;
  _writet(&MDU1, &tx[0], 4, MIDISEND_TIMEOUT);
}

void midi_usb_MidiSend2(uint8_t port, uint8_t b0, uint8_t b1) {
  uint8_t tx[4];
  tx[0] = calcCIN1(port, b0);
  tx[1] = b0;
  tx[2] = b1;
  tx[3] = 0;
  _writet(&MDU1, &tx[0], 4, MIDISEND_TIMEOUT);
}

void midi_usb_MidiSend3(uint8_t port, uint8_t b0, uint8_t b1, uint8_t b2) {
  uint8_t tx[4];
  tx[0] = calcCIN1(port, b0);
  tx[1] = b0;
  tx[2] = b1;
  tx[3] = b2;
  _writet(&MDU1, &tx[0], 4, MIDISEND_TIMEOUT);
}

#endif /* HAL_USE_MIDI_USB */

/** @} */
