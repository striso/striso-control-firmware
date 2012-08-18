/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012 Giovanni Di Sirio.

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

#include "ch.h"
#include "hal.h"
#include "test.h"
#include "usb_cdc.h"
#include "chprintf.h"

/*
 * USB Driver structure.
 */
static SerialUSBDriver SDU1;

#define ID_DIS 0
#define ID_BAS 1
#define ID_CONTROL 2
#define ID_ACCEL 3
#define ID_SYS 4

#define MIN_PRES 15

/* Total number of channels to be sampled by a single ADC operation.*/
#define ADC_GRP1_NUM_CHANNELS   3

/* Depth of the conversion buffer, channels are sampled one time each.*/
#define ADC_GRP1_BUF_DEPTH      1

#define ADC_GRP1_BUF_COUNT      (17*ADC_GRP1_NUM_CHANNELS)

#define OUT_NUM_CHANNELS        51

static const ioportid_t out_channels_port[51] = {
  GPIOA, GPIOA, GPIOA, GPIOA, GPIOC, GPIOC, GPIOB, GPIOB, GPIOF, GPIOF,
  GPIOF, GPIOF, GPIOF, GPIOG, GPIOG, GPIOE, GPIOE, GPIOE,
  GPIOE, GPIOE, GPIOE, GPIOE, GPIOE, GPIOE, GPIOB, GPIOB,
  GPIOB, GPIOB, GPIOB, GPIOB, GPIOD, GPIOD, GPIOD, GPIOD, GPIOD, GPIOD,
  GPIOD, GPIOD, GPIOG, GPIOG, GPIOG, GPIOG, GPIOG, GPIOG, GPIOG,
  GPIOC, GPIOC, GPIOC, GPIOC, GPIOA, GPIOA
};
static const int out_channels_pad[51] = {
   4,  5,  6,  7,  4,  5,  0,  1, 11, 12,
  13, 14, 15,  0,  1,  7,  8,  9,
  10, 11, 12, 13, 14, 15, 10, 11,
  12, 13, 14, 15,  8,  9, 10, 11, 12, 13,
  14, 15,  2,  3,  4,  5,  6,  7,  8, 
   6,  7,  8,  9,  8,  9
};

static int cur_channel = 0;

/*
 * ADC samples buffer.
 */
static adcsample_t samples[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH * ADC_GRP1_BUF_COUNT];

//#define ADC_SAMPLE_DEF ADC_SAMPLE_3
//#define ADC_SAMPLE_DEF ADC_SAMPLE_15
//#define ADC_SAMPLE_DEF ADC_SAMPLE_28
#define ADC_SAMPLE_DEF ADC_SAMPLE_56
/*
 * ADC conversion group.
 * Mode:        Linear buffer, 4 samples of 2 channels, SW triggered.
 * Channels:    IN11   (48 cycles sample time)
 *              Sensor (192 cycles sample time)
 */
static const ADCConversionGroup adcgrpcfg = {
  FALSE,
  ADC_GRP1_NUM_CHANNELS,
  NULL, //adccb,
  NULL,
  /* HW dependent part.*/
  0, // CR1
  ADC_CR2_SWSTART,
  0, // SMPR1
  ADC_SMPR2_SMP_AN0(ADC_SAMPLE_DEF) | ADC_SMPR2_SMP_AN1(ADC_SAMPLE_DEF) | ADC_SMPR2_SMP_AN2(ADC_SAMPLE_DEF),
  ADC_SQR1_NUM_CH(ADC_GRP1_NUM_CHANNELS),
  0, // SQR2
  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN0) | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN1) | ADC_SQR3_SQ3_N(ADC_CHANNEL_IN2)
  //ADC_SQR3_SQ1_N(ADC_CHANNEL_IN0) | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN3) | ADC_SQR3_SQ3_N(ADC_CHANNEL_IN1) | ADC_SQR3_SQ4_N(ADC_CHANNEL_IN3) | ADC_SQR3_SQ5_N(ADC_CHANNEL_IN2)

};

#define BUFFERSIZE 240
static int msg_buffer[BUFFERSIZE];
static int msg_read = 0;
static int msg_write = 0;
static Mutex msg_lock;
static int underruns = 0;

static int msgSend(int size, int* msg) {
  chMtxLock(&msg_lock);

  int n;
  int old_write = msg_write;
  msg_buffer[msg_write] = size;
  msg_write = (msg_write + 1) % BUFFERSIZE;
  for (n = 0; n < size; n++) {
    if (msg_read == msg_write) {
      //msg_read = (msg_read + msg_buffer[msg_read] + 1) % BUFFERSIZE;
      msg_write = old_write;
      underruns++;
      chMtxUnlock();
      return 1;
    }
    msg_buffer[msg_write] = msg[n];
    msg_write = (msg_write + 1) % BUFFERSIZE;
  }

  chMtxUnlock();
  return 0;
}

static int msgGet(int maxsize, int* msg) {
  chMtxLock(&msg_lock);

  if (msg_read == msg_write) {
    chMtxUnlock();
    return 0;
  }

  int size = msg_buffer[msg_read];

  if (size > maxsize) {
    chMtxUnlock();
    return -10;
  }
  msg_read = (msg_read + 1) % BUFFERSIZE;
  
  int n;
  for (n = 0; n < size; n++) {
    if (msg_read == msg_write) {
      chMtxUnlock();
      return -1;
    }
    msg[n] = msg_buffer[msg_read];
    msg_read = (msg_read + 1) % BUFFERSIZE;
  }

  chMtxUnlock();
  return size;
}

static SerialConfig ser_cfg = {
    2000000,
    0,
    0,
    0,
};

/*===========================================================================*/
/* USB related stuff.                                                        */
/*===========================================================================*/

/*
 * USB Device Descriptor.
 */
static const uint8_t vcom_device_descriptor_data[18] = {
  USB_DESC_DEVICE       (0x0110,        /* bcdUSB (1.1).                    */
                         0x02,          /* bDeviceClass (CDC).              */
                         0x00,          /* bDeviceSubClass.                 */
                         0x00,          /* bDeviceProtocol.                 */
                         0x40,          /* bMaxPacketSize.                  */
                         0x0483,        /* idVendor (ST).                   */
                         0x5740,        /* idProduct.                       */
                         0x0200,        /* bcdDevice.                       */
                         1,             /* iManufacturer.                   */
                         2,             /* iProduct.                        */
                         3,             /* iSerialNumber.                   */
                         1)             /* bNumConfigurations.              */
};

/*
 * Device Descriptor wrapper.
 */
static const USBDescriptor vcom_device_descriptor = {
  sizeof vcom_device_descriptor_data,
  vcom_device_descriptor_data
};

/* Configuration Descriptor tree for a CDC.*/
static const uint8_t vcom_configuration_descriptor_data[67] = {
  /* Configuration Descriptor.*/
  USB_DESC_CONFIGURATION(67,            /* wTotalLength.                    */
                         0x02,          /* bNumInterfaces.                  */
                         0x01,          /* bConfigurationValue.             */
                         0,             /* iConfiguration.                  */
                         0xC0,          /* bmAttributes (self powered).     */
                         50),           /* bMaxPower (100mA).               */
  /* Interface Descriptor.*/
  USB_DESC_INTERFACE    (0x00,          /* bInterfaceNumber.                */
                         0x00,          /* bAlternateSetting.               */
                         0x01,          /* bNumEndpoints.                   */
                         0x02,          /* bInterfaceClass (Communications
                                           Interface Class, CDC section
                                           4.2).                            */
                         0x02,          /* bInterfaceSubClass (Abstract
                                         Control Model, CDC section 4.3).   */
                         0x01,          /* bInterfaceProtocol (AT commands,
                                           CDC section 4.4).                */
                         0),            /* iInterface.                      */
  /* Header Functional Descriptor (CDC section 5.2.3).*/
  USB_DESC_BYTE         (5),            /* bLength.                         */
  USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
  USB_DESC_BYTE         (0x00),         /* bDescriptorSubtype (Header
                                           Functional Descriptor.           */
  USB_DESC_BCD          (0x0110),       /* bcdCDC.                          */
  /* Call Management Functional Descriptor. */
  USB_DESC_BYTE         (5),            /* bFunctionLength.                 */
  USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
  USB_DESC_BYTE         (0x01),         /* bDescriptorSubtype (Call Management
                                           Functional Descriptor).          */
  USB_DESC_BYTE         (0x00),         /* bmCapabilities (D0+D1).          */
  USB_DESC_BYTE         (0x01),         /* bDataInterface.                  */
  /* ACM Functional Descriptor.*/
  USB_DESC_BYTE         (4),            /* bFunctionLength.                 */
  USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
  USB_DESC_BYTE         (0x02),         /* bDescriptorSubtype (Abstract
                                           Control Management Descriptor).  */
  USB_DESC_BYTE         (0x02),         /* bmCapabilities.                  */
  /* Union Functional Descriptor.*/
  USB_DESC_BYTE         (5),            /* bFunctionLength.                 */
  USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
  USB_DESC_BYTE         (0x06),         /* bDescriptorSubtype (Union
                                           Functional Descriptor).          */
  USB_DESC_BYTE         (0x00),         /* bMasterInterface (Communication
                                           Class Interface).                */
  USB_DESC_BYTE         (0x01),         /* bSlaveInterface0 (Data Class
                                           Interface).                      */
  /* Endpoint 2 Descriptor.*/
  USB_DESC_ENDPOINT     (USB_CDC_INTERRUPT_REQUEST_EP|0x80,
                         0x03,          /* bmAttributes (Interrupt).        */
                         0x0008,        /* wMaxPacketSize.                  */
                         0xFF),         /* bInterval.                       */
  /* Interface Descriptor.*/
  USB_DESC_INTERFACE    (0x01,          /* bInterfaceNumber.                */
                         0x00,          /* bAlternateSetting.               */
                         0x02,          /* bNumEndpoints.                   */
                         0x0A,          /* bInterfaceClass (Data Class
                                           Interface, CDC section 4.5).     */
                         0x00,          /* bInterfaceSubClass (CDC section
                                           4.6).                            */
                         0x00,          /* bInterfaceProtocol (CDC section
                                           4.7).                            */
                         0x00),         /* iInterface.                      */
  /* Endpoint 3 Descriptor.*/
  USB_DESC_ENDPOINT     (USB_CDC_DATA_AVAILABLE_EP,     /* bEndpointAddress.*/
                         0x02,          /* bmAttributes (Bulk).             */
                         0x0040,        /* wMaxPacketSize.                  */
                         0x00),         /* bInterval.                       */
  /* Endpoint 1 Descriptor.*/
  USB_DESC_ENDPOINT     (USB_CDC_DATA_REQUEST_EP|0x80,  /* bEndpointAddress.*/
                         0x02,          /* bmAttributes (Bulk).             */
                         0x0040,        /* wMaxPacketSize.                  */
                         0x00)          /* bInterval.                       */
};

/*
 * Configuration Descriptor wrapper.
 */
static const USBDescriptor vcom_configuration_descriptor = {
  sizeof vcom_configuration_descriptor_data,
  vcom_configuration_descriptor_data
};

/*
 * U.S. English language identifier.
 */
static const uint8_t vcom_string0[] = {
  USB_DESC_BYTE(4),                     /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  USB_DESC_WORD(0x0409)                 /* wLANGID (U.S. English).          */
};

/*
 * Vendor string.
 */
static const uint8_t vcom_string1[] = {
  USB_DESC_BYTE(38),                    /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  'S', 0, 'T', 0, 'M', 0, 'i', 0, 'c', 0, 'r', 0, 'o', 0, 'e', 0,
  'l', 0, 'e', 0, 'c', 0, 't', 0, 'r', 0, 'o', 0, 'n', 0, 'i', 0,
  'c', 0, 's', 0
};

/*
 * Device Description string.
 */
static const uint8_t vcom_string2[] = {
  USB_DESC_BYTE(56),                    /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  'C', 0, 'h', 0, 'i', 0, 'b', 0, 'i', 0, 'O', 0, 'S', 0, '/', 0,
  'R', 0, 'T', 0, ' ', 0, 'V', 0, 'i', 0, 'r', 0, 't', 0, 'u', 0,
  'a', 0, 'l', 0, ' ', 0, 'C', 0, 'O', 0, 'M', 0, ' ', 0, 'P', 0,
  'o', 0, 'r', 0, 't', 0
};

/*
 * Serial Number string.
 */
static const uint8_t vcom_string3[] = {
  USB_DESC_BYTE(8),                     /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  '0' + CH_KERNEL_MAJOR, 0,
  '0' + CH_KERNEL_MINOR, 0,
  '0' + CH_KERNEL_PATCH, 0
};

/*
 * Strings wrappers array.
 */
static const USBDescriptor vcom_strings[] = {
  {sizeof vcom_string0, vcom_string0},
  {sizeof vcom_string1, vcom_string1},
  {sizeof vcom_string2, vcom_string2},
  {sizeof vcom_string3, vcom_string3}
};

/*
 * Handles the GET_DESCRIPTOR callback. All required descriptors must be
 * handled here.
 */
static const USBDescriptor *get_descriptor(USBDriver *usbp,
                                           uint8_t dtype,
                                           uint8_t dindex,
                                           uint16_t lang) {

  (void)usbp;
  (void)lang;
  switch (dtype) {
  case USB_DESCRIPTOR_DEVICE:
    return &vcom_device_descriptor;
  case USB_DESCRIPTOR_CONFIGURATION:
    return &vcom_configuration_descriptor;
  case USB_DESCRIPTOR_STRING:
    if (dindex < 4)
      return &vcom_strings[dindex];
  }
  return NULL;
}

/**
 * @brief   IN EP1 state.
 */
static USBInEndpointState ep1instate;

/**
 * @brief   EP1 initialization structure (IN only).
 */
static const USBEndpointConfig ep1config = {
  USB_EP_MODE_TYPE_BULK,
  NULL,
  sduDataTransmitted,
  NULL,
  0x0040,
  0x0000,
  &ep1instate,
  NULL,
  2,
  NULL
};

/**
 * @brief   IN EP2 state.
 */
USBInEndpointState ep2instate;

/**
 * @brief   EP2 initialization structure (IN only).
 */
static const USBEndpointConfig ep2config = {
  USB_EP_MODE_TYPE_INTR,
  NULL,
  sduInterruptTransmitted,
  NULL,
  0x0010,
  0x0000,
  &ep2instate,
  NULL,
  1,
  NULL
};

/**
 * @brief   OUT EP2 state.
 */
USBOutEndpointState ep3outstate;

/**
 * @brief   EP3 initialization structure (OUT only).
 */
static const USBEndpointConfig ep3config = {
  USB_EP_MODE_TYPE_BULK,
  NULL,
  NULL,
  sduDataReceived,
  0x0000,
  0x0040,
  NULL,
  &ep3outstate,
  0,
  NULL
};

/*
 * Handles the USB driver global events.
 */
static void usb_event(USBDriver *usbp, usbevent_t event) {

  switch (event) {
  case USB_EVENT_RESET:
    return;
  case USB_EVENT_ADDRESS:
    return;
  case USB_EVENT_CONFIGURED:
    chSysLockFromIsr();

    /* Enables the endpoints specified into the configuration.
       Note, this callback is invoked from an ISR so I-Class functions
       must be used.*/
    usbInitEndpointI(usbp, USB_CDC_DATA_REQUEST_EP, &ep1config);
    usbInitEndpointI(usbp, USB_CDC_INTERRUPT_REQUEST_EP, &ep2config);
    usbInitEndpointI(usbp, USB_CDC_DATA_AVAILABLE_EP, &ep3config);

    /* Resetting the state of the CDC subsystem.*/
    sduConfigureHookI(usbp);

    chSysUnlockFromIsr();
    return;
  case USB_EVENT_SUSPEND:
    return;
  case USB_EVENT_WAKEUP:
    return;
  case USB_EVENT_STALLED:
    return;
  }
  return;
}

/*
 * Serial over USB driver configuration.
 */
static const SerialUSBConfig serusbcfg = {
  &USBD1,
  {
    usb_event,
    get_descriptor,
    sduRequestsHook,
    NULL
  }
};

/*
 * This is a periodic thread that does absolutely nothing except flashing
 * a LED.
 */
static WORKING_AREA(waThread1, 128);
static msg_t Thread1(void *arg) {

  (void)arg;
  chRegSetThreadName("blinker");
  int msg[8];
  msg[0] = ID_SYS;
  msg[1] = 1;
  while (TRUE) {
    palSetPad(GPIOA, GPIOA_LED1);       /* Orange.  */
    chThdSleepMilliseconds(500);
    palClearPad(GPIOA, GPIOA_LED1);     /* Orange.  */
    chThdSleepMilliseconds(500);
    msg[2] = underruns;
    msgSend(3,msg);
    //chprintf((BaseChannel *)&SDU1, "hello world\n");
  }

  return 0;
}

static void pack(int *in, uint8_t *out, int n) {
  int c;
  for (c=0; c<n; c++) {
    out[c*2] = 0x7f & (uint8_t)(in[c]>>7);
    out[c*2+1] = 0x7f & (uint8_t)(in[c]);
  }
}

static void unpack(uint8_t *in, int *out, int n) {
  int c;
  for (c=0; c<n; c++) {
    out[c] = ((int)in[c*2])<<7 | ((int)in[c*2+1]);
  }
}

/*
 * This is a periodic thread that does absolutely nothing except flashing
 * a LED.
 */
static WORKING_AREA(waThreadSend, 128);
static msg_t ThreadSend(void *arg) {

  (void)arg;
  chRegSetThreadName("send messages over USB");
  //chprintf((BaseSequentialStream *)&SDU1, "Send thread started.\n");
  int msg[8];
  uint8_t cmsg[8];
  int size, n;
  cmsg[0] = 0;
  while (TRUE) {
    size = msgGet(8, msg);
    
    if (size == 5) {
      cmsg[0] = 0x80 | ((uint8_t)msg[0])<<3 | 0x01;
      cmsg[1] = 0x7f & (uint8_t)msg[1];
      pack(&msg[2], &cmsg[2], 3);
      size = chSequentialStreamWrite((BaseSequentialStream *)&SD2, cmsg, 8);
      //if (size < 8) {
        //chprintf((BaseSequentialStream *)&SDU1, "send size: %d\n", size);
        //cmsg[size] = 255;
        //size += chSequentialStreamWrite((BaseSequentialStream *)&SDU1, &cmsg[size], 8-size);
      //}
    }
    else if (size == 3) {
      cmsg[0] = 0x80 | ((uint8_t)msg[0])<<3 | 0x00;
      cmsg[1] = 0x7f & (uint8_t)msg[1];
      //cmsg[2] = 0x7f & (uint8_t)msg[2];
      //cmsg[3] = 0;
      pack(&msg[2], &cmsg[2], 1);
      size = chSequentialStreamWrite((BaseSequentialStream *)&SD2, cmsg, 4);
    }
    else if (size == 0) {
      chThdSleep(1);
    }
    /*
    if (size == 5) {
      chprintf((BaseSequentialStream *)&SDU1, "%2d %2d %4d %4d %4d\r", msg[0],msg[1],msg[2],msg[3],msg[4]);
    }
    else if (size == 3) {
      chprintf((BaseSequentialStream *)&SDU1, "%2d %2d %3d 0 0\r", msg[0],msg[1],msg[2]);
    }
    else if (size < 0) {
      chprintf((BaseSequentialStream *)&SDU1, "Error: %d.\n", size);
    }*/
    //chThdSleep(1);
  }

  return 0;
}

/*
 * This is a periodic thread that does absolutely nothing except flashing
 * a LED.
 */
static WORKING_AREA(waThreadRead, 128);
static msg_t ThreadRead(void *arg) {

  (void)arg;
  chRegSetThreadName("read messages over UART");
  //chprintf((BaseSequentialStream *)&SDU1, "Read thread started.\n");
  int msg[8];
  uint8_t cmsg[16];
  int size, rsize, n;
  while (TRUE) {
    size = chSequentialStreamRead((BaseSequentialStream *)&SD2, cmsg, 1);
    if (size && (cmsg[0] & 0x80)) {
      rsize = ((cmsg[0] & 0x7)+1) * 4;
      if (rsize > 16) { continue; }
      size += chSequentialStreamRead((BaseSequentialStream *)&SD2, &cmsg[1], rsize-1);
      if (size != rsize) { continue; }
      for (n=1; n < size; n++) {
        if (cmsg[n] & 0x80) { break; }
      }
      if (n<size) { continue; }

      // message OK
      if (rsize == 8) {
        msg[0] = (cmsg[0] & 0x7f)>>3;
        msg[1] = cmsg[1];
        unpack(&cmsg[2], &msg[2], 3);
        while (msgSend(5, msg)) {
		  // if it is a note-off message keep retrying
          if (msg[2] > 0 || msg[3] > 0 || msg[4] > 0) {
            break;
          }
          chThdSleep(1);
        }
      }
    }
  }
  return 0;
}

/*
 * Application entry point.
 */
int main(void) {
  int n;

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   */
  usbDisconnectBus(serusbcfg.usbp);
  chThdSleepMilliseconds(1000);
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);
  usbConnectBus(serusbcfg.usbp);

  /*
   * Activates the serial driver 2 using the driver default configuration.
   * PD5(TX) and PD6(RX) are routed to USART2.
   */
  sdStart(&SD2, &ser_cfg);
  palSetPadMode(GPIOD, 5, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOD, 6, PAL_MODE_ALTERNATE(7));

  chThdSleepMilliseconds(500);

  /*
   * If the user button is pressed after the reset then the test suite is
   * executed immediately before activating the various device drivers in
   * order to not alter the benchmark scores.
   */
  if (palReadPad(GPIOG, GPIOG_BUTTON))
    TestThread(&SDU1);

  chMtxInit(&msg_lock);

  /*
   * Initialize output channels for the buttons as opendrain
   */
  for (n=0; n<OUT_NUM_CHANNELS; n++) {
    palSetPadMode(out_channels_port[n], out_channels_pad[n], PAL_MODE_OUTPUT_OPENDRAIN);
  }

  /*
   * Initializes the ADC driver 1.
   * The pin PA0,PA1,PA2 on the port GPIOC are programmed as analog input.
   */
  adcStart(&ADCD1, NULL);
  palSetPadMode(GPIOA, 0, PAL_MODE_INPUT_ANALOG);
  palSetPadMode(GPIOA, 1, PAL_MODE_INPUT_ANALOG);
  palSetPadMode(GPIOA, 2, PAL_MODE_INPUT_ANALOG);

  /*
   * Creates the example thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  chThdCreateStatic(waThreadSend, sizeof(waThreadSend), NORMALPRIO, ThreadSend, NULL);
  //chThdCreateStatic(waThreadRead, sizeof(waThreadRead), NORMALPRIO, ThreadRead, NULL);
  chThdSleepMilliseconds(50);
  //chThdSleepSeconds(20);

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the button state, when the button is
   * pressed the test procedure is launched with output on the serial
   * driver 2.
   */
  int old_channel;
  int cur_but;
  int pressed[51];
  int msg[5];
  int sysbut[3] = {-1,-1,-1};
  msg[0] = ID_DIS;
  while (TRUE) {
    cur_but = cur_channel;

    for (n = 0; n < 3; n++) {
      adcConvert(&ADCD1, &adcgrpcfg, &samples[cur_channel*ADC_GRP1_NUM_CHANNELS], ADC_GRP1_BUF_DEPTH);

      old_channel = cur_channel;
      cur_channel = (cur_channel+1) % OUT_NUM_CHANNELS;

      palSetPad(out_channels_port[old_channel], out_channels_pad[old_channel]);       /* Open old channel */
      palClearPad(out_channels_port[cur_channel], out_channels_pad[cur_channel]);         /* Drain new channel */
    }

    for (n = 0; n < 3; n++) {
      int s0 = 4095-samples[ cur_but    * ADC_GRP1_NUM_CHANNELS + n];
      int s1 = 4095-samples[(cur_but+1) * ADC_GRP1_NUM_CHANNELS + n];// - (s0-30)/40;
      int s2 = 4095-samples[(cur_but+2) * ADC_GRP1_NUM_CHANNELS + n];// - (s1-30)/40;
      int but_id = cur_but / 3 + n * 17;

      if (s0 > MIN_PRES || s1 > MIN_PRES || s2 > MIN_PRES) {
        msg[1] = but_id;
        msg[2] = s0;
        msg[3] = s1;
        msg[4] = s2;
        while (msgSend(5, msg)) {
          if (pressed[but_id]) {
            break;
          }
          chThdSleep(1);
        }
        pressed[but_id] = 1;
      }
      else if (pressed[but_id]) {
        pressed[but_id] = 0;
        msg[1] = but_id;
        msg[2] = 0;
        msg[3] = 0;
        msg[4] = 0;
        while (msgSend(5, msg)) {
          chThdSleep(1);
        }
      }
    }

    if (sysbut[0] > 1) {
      sysbut[0] -= 2;
    }
    else {
    n = palReadPad(GPIOG, GPIOG_BUTTON);
    if (n != sysbut[0]) {
      msg[0] = ID_CONTROL;
      msg[1] = 0;
      msg[2] = n;
      if (!msgSend(3, msg))
        sysbut[0] = n + 50;
      msg[0] = ID_DIS;
    }
    }
    if (sysbut[1] > 1) {
      sysbut[1] -= 2;
    }
    else {
    n = palReadPad(GPIOB, GPIOB_BUTTON_UP);
    if (n != sysbut[1]) {
      msg[0] = ID_CONTROL;
      msg[1] = 1;
      msg[2] = n;
      if (!msgSend(3, msg))
        sysbut[1] = n + 50;
      msg[0] = ID_DIS;
    }
    }
    if (sysbut[2] > 1) {
      sysbut[2] -= 2;
    }
    else {
    n = palReadPad(GPIOB, GPIOB_BUTTON_DOWN);
    if (n != sysbut[2]) {
      msg[0] = ID_CONTROL;
      msg[1] = 2;
      msg[2] = n;
      if (!msgSend(3, msg))
        sysbut[2] = n + 50;
      msg[0] = ID_DIS;
    }
    }

    chThdSleep(1);
  }
}
