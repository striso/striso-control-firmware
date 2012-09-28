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
#include "chprintf.h"

#include "usbcfg.h"

/* Virtual serial port over USB.*/
static SerialUSBDriver SDU1;

/*
 * Accelerometer part
 */
/* buffers depth */
#define ACCEL_RX_DEPTH 6
#define ACCEL_TX_DEPTH 4

/* mma8451q specific addresses */
#define ACCEL_OUT_DATA    0x00
#define ACCEL_CTRL_REG1   0x16

static uint8_t rxbuf[ACCEL_RX_DEPTH];
static uint8_t txbuf[ACCEL_TX_DEPTH];
static i2cflags_t errors = 0;
static int16_t acceleration_x, acceleration_y, acceleration_z;
#define mma7455_addr 0b0011101

/**
 * Converts data from 2complemented representation to signed integer
 */
uint16_t complement2unsigned(uint8_t lsb, uint8_t msb){
  uint16_t word = 0;
  word = ((~msb & 2) << 8) | ((msb & 1) << 8) | lsb;
  //word = msb << 8 | lsb;
  //if (msb > 0x7F){
  //  return -1 * ((int16_t)((~word) + 1));
  //}
  //return (int16_t)word;
  return word;
}

/* I2C interface #2 */
static const I2CConfig i2cfg2 = {
    OPMODE_I2C,
    400000,
    FAST_DUTY_CYCLE_2,
};

/*
 * end accelerometer
 */

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

/*
 * SPI1 configuration structure.
 * Speed 5.25MHz, CPHA=1, CPOL=1, 8bits frames, MSb transmitted first.
 * The slave select line is the pin GPIOE_CS_SPI on the port GPIOE.
 */
static const SPIConfig spi1cfg = {
  NULL,
  /* HW dependent part.*/
  GPIOG,
  13,
  SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_CPOL | SPI_CR1_CPHA
};

/*
 * Accelerometer thread.
 */
static WORKING_AREA(waThreadAccel, 128);
static msg_t ThreadAccel(void *arg) {

  (void)arg;
  chRegSetThreadName("accelerometer");
  msg_t status = RDY_OK;
  systime_t tmo = MS2ST(4);



  /**
   * Prepares the accelerometer
   */
  txbuf[0] = ACCEL_CTRL_REG1; /* register address */
  txbuf[1] = 0x1; // Set to measurement mode
  i2cAcquireBus(&I2CD2);
  status = i2cMasterTransmitTimeout(&I2CD2, mma7455_addr, txbuf, 2, rxbuf, 0, tmo);
  i2cReleaseBus(&I2CD2);

  if (status != RDY_OK){
    errors = i2cGetErrors(&I2CD2);
  }

  int msg[8];
  msg[0] = ID_ACCEL;
  msg[1] = 3; // size
  while (TRUE) {
    chThdSleepMilliseconds(20);

    txbuf[0] = ACCEL_OUT_DATA; /* register address */
    i2cAcquireBus(&I2CD2);
    status = i2cMasterTransmitTimeout(&I2CD2, mma7455_addr, txbuf, 1, rxbuf, 6, tmo);
    i2cReleaseBus(&I2CD2);

    if (status != RDY_OK){
      errors = i2cGetErrors(&I2CD2);
    }
    else {

      msg[2] = complement2unsigned(rxbuf[0], rxbuf[1]);
      msg[3] = complement2unsigned(rxbuf[2], rxbuf[3]);
      msg[4] = complement2unsigned(rxbuf[4], rxbuf[5]);
      msgSend(5,msg);
    }
  }

  return 0;
}

/*
 * LED flash thread.
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
 * Message send thread
 */
static WORKING_AREA(waThreadSend, 128);
static msg_t ThreadSend(void *arg) {

  (void)arg;
  chRegSetThreadName("send messages over USB");
  //chprintf((BaseSequentialStream *)&SD2, "Send thread started.\n");
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
      size = chSequentialStreamWrite((BaseSequentialStream *)&SDU1, cmsg, 8);
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
      size = chSequentialStreamWrite((BaseSequentialStream *)&SDU1, cmsg, 4);
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
 * Message read thread
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
   * Activates the serial driver 2 using the driver default configuration.
   * PD5(TX) and PD6(RX) are routed to USART2.
   */
  sdStart(&SD2, &ser_cfg);
  palSetPadMode(GPIOD, 5, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOD, 6, PAL_MODE_ALTERNATE(7));

  /*
   * If the user button is pressed after the reset then the test suite is
   * executed immediately before activating the various device drivers in
   * order to not alter the benchmark scores.
   */
  if (palReadPad(GPIOG, GPIOG_BUTTON))
    TestThread(&SD2);

  /*
   * Initializes a serial-over-USB CDC driver.
   */
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);

  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * Note, a delay is inserted in order to not have to disconnect the cable
   * after a reset.
   */
  //usbDisconnectBus(serusbcfg.usbp);
  //chThdSleepMilliseconds(1000);
  usbStart(serusbcfg.usbp, &usbcfg);
  //usbConnectBus(serusbcfg.usbp);

  chThdSleepMilliseconds(1000);

  // init msg mutex
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
   * Starts I2C
   */
  i2cStart(&I2CD2, &i2cfg2);


  /*
   * Initializes the SPI driver 1. The signals
   * are already initialized in the board file.
   */
  spiStart(&SPID1, &spi1cfg);

  /*
   * Creates the LED flash thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  chThdCreateStatic(waThreadSend, sizeof(waThreadSend), NORMALPRIO, ThreadSend, NULL);
  //chThdCreateStatic(waThreadRead, sizeof(waThreadRead), NORMALPRIO, ThreadRead, NULL);
  chThdSleepMilliseconds(50);
  //chThdSleepSeconds(20);

  /*
   * Creates the accelerometer thread.
   */
  chThdCreateStatic(waThreadAccel, sizeof(waThreadAccel), NORMALPRIO, ThreadAccel, NULL);

  /*
   * Normal main() thread activity.
   * Read out buttons and create messages.
   */
  int old_channel;
  int cur_but;
  int pressed[51];
  int msg[5];
  int sysbut[3] = {-1,-1,-1};
  msg[0] = ID_DIS;
  while (TRUE) {
    cur_but = cur_channel;

    // read 3 buttons * 3 pads
    for (n = 0; n < 3; n++) {
      adcConvert(&ADCD1, &adcgrpcfg, &samples[cur_channel*ADC_GRP1_NUM_CHANNELS], ADC_GRP1_BUF_DEPTH);

      // correct crosstalk/adc start effect
      samples[cur_channel*ADC_GRP1_NUM_CHANNELS + 1] += (4095-samples[cur_channel*ADC_GRP1_NUM_CHANNELS])/250;
      samples[cur_channel*ADC_GRP1_NUM_CHANNELS + 2] += (4095-samples[cur_channel*ADC_GRP1_NUM_CHANNELS + 1])/250;

      old_channel = cur_channel;
      cur_channel = (cur_channel+1) % OUT_NUM_CHANNELS;

      palSetPad(out_channels_port[old_channel], out_channels_pad[old_channel]);       /* Open old channel */
      palClearPad(out_channels_port[cur_channel], out_channels_pad[cur_channel]);         /* Drain new channel */
    }

    // correct crosstalk/adc start effect
    samples[ cur_but    * ADC_GRP1_NUM_CHANNELS] += 8;
    samples[(cur_but+1) * ADC_GRP1_NUM_CHANNELS] += 2;
    samples[(cur_but+2) * ADC_GRP1_NUM_CHANNELS] += 2;

    for (n = 0; n < 3; n++) {
      int s0 = 4095-samples[ cur_but    * ADC_GRP1_NUM_CHANNELS + n];
      int s1 = 4095-samples[(cur_but+1) * ADC_GRP1_NUM_CHANNELS + n];
      int s2 = 4095-samples[(cur_but+2) * ADC_GRP1_NUM_CHANNELS + n];
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
