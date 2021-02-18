/**
 * Copyright (C) 2019 Piers Titus van der Torren
 *
 * This file is part of Striso Control.
 *
 * Striso Control is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * Striso Control is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * Striso Control. If not, see <http://www.gnu.org/licenses/>.
 */
#include "ch.h"
#include "hal.h"

#include "ccportab.h"

#include "motionsensor.h"
#include "config.h"
#include "striso.h"
#include "synth.h"
#include "messaging.h"
#include <math.h>

#ifdef USE_MPU6050
#include "MPU6050.h"
#endif
#ifdef USE_LSM6DSL
#include "lsm6dsl.h"
#endif

/* I2C interface #1 */
#if defined(STM32F4XX)
static const I2CConfig i2ccfg_motion = {
  OPMODE_I2C,
  400000,
  FAST_DUTY_CYCLE_2,
};
#elif defined(STM32H7XX)
/*
def timingr(t):
  print("  STM32_TIMINGR_PRESC({}U) |\n  STM32_TIMINGR_SCLDEL({}U) | STM32_TIMINGR_SDADEL({}U) |\n  STM32_TIMINGR_SCLH({}U)  | STM32_TIMINGR_SCLL({}U),".format(
    t>>28, t>>20 & 15, t>>16 & 15, t>>8 & 255, t & 255))

400kHz, 0,0 rise,fall:
  STM32_TIMINGR_PRESC(0U) |
  STM32_TIMINGR_SCLDEL(9U) | STM32_TIMINGR_SDADEL(0U) |
  STM32_TIMINGR_SCLH(52U)  | STM32_TIMINGR_SCLL(182U),

400kHz, 100,100 rise,fall:
  STM32_TIMINGR_PRESC(1U) |
  STM32_TIMINGR_SCLDEL(9U) | STM32_TIMINGR_SDADEL(1U) |
  STM32_TIMINGR_SCLH(26U)  | STM32_TIMINGR_SCLL(80U),

400kHz, 300,300 rise,fall:
  STM32_TIMINGR_PRESC(2U) |
  STM32_TIMINGR_SCLDEL(13U) | STM32_TIMINGR_SDADEL(8U) |
  STM32_TIMINGR_SCLH(17U)  | STM32_TIMINGR_SCLL(40U),

100kHz, 300,300 rise,fall:
  STM32_TIMINGR_PRESC(8U) |
  STM32_TIMINGR_SCLDEL(6U) | STM32_TIMINGR_SDADEL(3U) |
  STM32_TIMINGR_SCLH(43U)  | STM32_TIMINGR_SCLL(58U),
*/
static const I2CConfig i2ccfg_motion = {
// 100kHz, 300,300 rise,fall:
  STM32_TIMINGR_PRESC(8U) |
  STM32_TIMINGR_SCLDEL(6U) | STM32_TIMINGR_SDADEL(3U) |
  STM32_TIMINGR_SCLH(43U)  | STM32_TIMINGR_SCLL(58U),
  0,
  0
};
#endif

#define pow2(x) ((x)*(x))

#ifdef USE_LSM6DSL
/*
 * I2C TX and RX buffers.
 */
CC_ALIGN(CACHE_LINE_SIZE) static uint8_t txbuf[32];
CC_ALIGN(CACHE_LINE_SIZE) static uint8_t rxbuf[32];

uint8_t lsm6dslReadRegister(uint8_t RegisterAddr) {
  msg_t status;

  if (I2CD3.state != I2C_READY) {
    i2cStart(&I2CD_MOTION, &i2ccfg_motion);
  }

  txbuf[0] = RegisterAddr;
  cacheBufferFlush(&txbuf[0], sizeof txbuf);
  status = i2cMasterTransmit(&I2CD_MOTION, LSM6DSL_SAD_GND, txbuf, 1,
                             rxbuf, 1);
  cacheBufferInvalidate(&rxbuf[0], sizeof rxbuf);
  return (rxbuf[0]);
}

uint8_t lsm6dslReadMotion(int16_t* motion) {
  msg_t status;

  if (I2CD3.state != I2C_READY) {
    i2cStart(&I2CD_MOTION, &i2ccfg_motion);
  }

  txbuf[0] = LSM6DSL_AD_OUTX_L_G;
  cacheBufferFlush(&txbuf[0], sizeof txbuf);
  status = i2cMasterTransmit(&I2CD_MOTION, LSM6DSL_SAD_GND, txbuf, 1,
                             rxbuf, 12);
  cacheBufferInvalidate(&rxbuf[0], sizeof rxbuf);
  for (int i = 0; i<6; i++) {
    motion[i] = ((int16_t*)rxbuf)[i];
  }
  return status;
}

float lsm6dslReadTemp(void) {
  msg_t status;
  txbuf[0] = LSM6DSL_AD_OUT_TEMP_L;
  cacheBufferFlush(&txbuf[0], sizeof txbuf);
  status = i2cMasterTransmit(&I2CD_MOTION, LSM6DSL_SAD_GND, txbuf, 1,
                             rxbuf, 2);
  cacheBufferInvalidate(&rxbuf[0], sizeof rxbuf);
  return (float)(((int16_t*)rxbuf)[0])/256.0f + 25.0f;
}

void lsm6dslWriteRegister(uint8_t RegisterAddr, uint8_t RegisterValue) {
  msg_t status;

  if (I2CD3.state != I2C_READY) {
    i2cStart(&I2CD_MOTION, &i2ccfg_motion);
  }

  txbuf[0] = RegisterAddr;
  txbuf[1] = RegisterValue;

  cacheBufferFlush(&txbuf[0], sizeof txbuf);
  status = i2cMasterTransmit(&I2CD_MOTION, LSM6DSL_SAD_GND, txbuf, 2,
                             rxbuf, 0);

  static uint8_t rd;
  rd = lsm6dslReadRegister(RegisterAddr);
  if (rd != RegisterValue) {
//    setErrorFlag(ERROR_CODEC_I2C);
  } else {
  }
}
#endif // USE_LSM6DSL

/*
 * This is a periodic thread that reads accelerometer and sends messages
 */
static THD_WORKING_AREA(waThreadAccel, 256);
static void ThreadAccel(void *arg) {
  chRegSetThreadName("motion");

  int16_t ax, ay, az, gx, gy, gz;
  int msg[9];
  msg[0] = ID_ACCEL;
  msg[1] = 8; // range?

#ifdef USE_MPU6050
  bool init_succeeded = false;
  while (init_succeeded == false) {
    /* MPU6050 initialization.*/
    MPU6050_writeReg(MPU6050_RA_PWR_MGMT_1  , 0b00000000);
    MPU6050_writeReg(MPU6050_RA_CONFIG, MPU6050_DLPF_BW_42); // DLPF 44 Hz
    MPU6050_writeReg(MPU6050_RA_GYRO_CONFIG, MPU6050_GYRO_CONFIG_FS_1000); // MPU6050_GYRO_FS_500
    MPU6050_writeReg(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACCEL_CONFIG_FS_8); // MPU6050_ACCEL_FS_8
    MPU6050_writeReg(MPU6050_RA_PWR_MGMT_1  , 0b00000001);

    if (MPU6050_readReg(MPU6050_RA_CONFIG) == MPU6050_DLPF_BW_42 &&
        MPU6050_readReg(MPU6050_RA_GYRO_CONFIG) == MPU6050_GYRO_CONFIG_FS_1000 &&
        MPU6050_readReg(MPU6050_RA_ACCEL_CONFIG) == MPU6050_ACCEL_CONFIG_FS_8 &&
        MPU6050_readReg(MPU6050_RA_PWR_MGMT_1) == 0b00000001 ) {
      init_succeeded = true;
    } else {
      // msg[2] = MPU6050_readReg(MPU6050_RA_CONFIG);
      // msg[3] = MPU6050_readReg(MPU6050_RA_GYRO_CONFIG);
      // msg[4] = MPU6050_readReg(MPU6050_RA_ACCEL_CONFIG);
      //msgSend(5,msg);
      chThdSleepMilliseconds(500);
    }
  }

  /* Reader thread loop.*/
  while (TRUE) {
    //MPUgetMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    // Axis configuration for Striso
    //MPUgetMotion6(&ay, &az, &ax, &gy, &gz, &gx);
    //ay = -ay; gy = -gy;
    //az = -az; gz = -gz;
    // and for flipped sensor board
    //ax = -ax; gx = -gx;
    //az = -az; gz = -gz;
    // Axis configuration for Strisoboard
    MPUgetMotion6(&ay, &ax, &az, &gy, &gx, &gz);
    ax = -ax; gx = -gx;
    ay = -ay; gy = -gy;
    az = -az; gz = -gz;

    float acc_x = ((float)ax)*(8.0/32768.0);
    float acc_y = ((float)ay)*(8.0/32768.0);
    float acc_z = ((float)az)*(8.0/32768.0);
    float acc_abs = sqrtf(pow2(acc_x) + pow2(acc_y) + pow2(acc_z));
    if (acc_abs>0.001) {
      acc_x /= acc_abs;
      acc_y /= acc_abs;
      acc_z /= acc_abs;
    }

    msg[2] = ax>>2;
    msg[3] = ay>>2;
    msg[4] = az>>2;
    msg[5] = ((int16_t)(acc_abs * 32768.0/8.0))>>2;
    msg[6] = gx>>2;
    msg[7] = gy>>2;
    msg[8] = gz>>2;
    msgSend(9,msg);

#ifdef USE_INTERNAL_SYNTH
    float rot_x = ((float)gx)*(1.0/32768.0);
    float rot_y = ((float)gy)*(1.0/32768.0);
    float rot_z = ((float)gz)*(1.0/32768.0);

    *(synth_interface.acc_abs) = acc_abs;
    *(synth_interface.acc_x) = acc_x;
    *(synth_interface.acc_y) = acc_y;
    *(synth_interface.acc_z) = acc_z;
    *(synth_interface.rot_x) = rot_x;
    *(synth_interface.rot_y) = rot_y;
    *(synth_interface.rot_z) = rot_z;
#endif

    /* Waiting until the next 10 milliseconds time interval.*/
    // TODO: wait for interrupt at MPU_READY pin
    chThdSleepMilliseconds(3);
  }
#endif
#ifdef USE_LSM6DSL

  // motion sensor reset
  lsm6dslWriteRegister(LSM6DSL_AD_CTRL3_C, LSMDSL_CTRL3_C_IF_INC | LSMDSL_CTRL3_C_SW_RESET);

  chThdSleepMilliseconds(1);
#define ACC_FS 2
  lsm6dslWriteRegister(LSM6DSL_AD_CTRL1_XL, LSM6DSL_ACC_ODR_208Hz | LSM6DSL_ACC_FS_2G); // TODO: setting sensitivity doesn't seem to work
  lsm6dslWriteRegister(LSM6DSL_AD_CTRL2_G, LSM6DSL_GYRO_ODR_208Hz | LSM6DSL_GYRO_FS_500DPS);



  /* Reader thread loop.*/
  int16_t motion[6];
  while (TRUE) {
    lsm6dslReadMotion(motion);

    ax = -motion[3]; gx = -motion[0];
    ay = -motion[4]; gy = -motion[1];
    az = -motion[5]; gz = -motion[2];

    float acc_x = ((float)ax)*(ACC_FS/32768.0f);
    float acc_y = ((float)ay)*(ACC_FS/32768.0f);
    float acc_z = ((float)az)*(ACC_FS/32768.0f);
    float acc_abs = sqrtf(pow2(acc_x) + pow2(acc_y) + pow2(acc_z));
    if (acc_abs>0.001) {
      acc_x /= acc_abs;
      acc_y /= acc_abs;
      acc_z /= acc_abs;
    }

    // acceleration in message has maximum range 8g (16g for acc_abs)
    // TODO: make /4 dependent on ACC_FS
    msg[2] = (ax/4)>>2;
    msg[3] = (ay/4)>>2;
    msg[4] = (az/4)>>2;
    msg[5] = ((int16_t)(acc_abs * 32768.0f/8.0f))>>2;
    msg[6] = gx>>2;
    msg[7] = gy>>2;
    msg[8] = gz>>2;
    msgSend(9,msg);

#ifdef USE_INTERNAL_SYNTH
    float rot_x = ((float)gx)*(1.0f/32768.0f);
    float rot_y = ((float)gy)*(1.0f/32768.0f);
    float rot_z = ((float)gz)*(1.0f/32768.0f);

    *(synth_interface.acc_abs) = acc_abs;
    *(synth_interface.acc_x) = acc_x;
    *(synth_interface.acc_y) = acc_y;
    *(synth_interface.acc_z) = acc_z;
    *(synth_interface.rot_x) = rot_x;
    *(synth_interface.rot_y) = rot_y;
    *(synth_interface.rot_z) = rot_z;
#endif

    /* Waiting until the next 10 milliseconds time interval.*/
    // TODO: wait for interrupt at MPU_READY pin
    chThdSleepMilliseconds(3);
  }
#endif
}

void MotionSensorStart(void) {
  // Check if pullup resistors are attached to i2c port
  // If not there is no motion sensor daughterboard
  palSetLineMode(LINE_MOTION_I2C_SCL, PAL_MODE_INPUT_PULLDOWN);
  palSetLineMode(LINE_MOTION_I2C_SDA, PAL_MODE_INPUT_PULLDOWN);

  if (!palReadLine(LINE_MOTION_I2C_SCL) ||
      !palReadLine(LINE_MOTION_I2C_SDA)) {
    // No pullups on i2c pads detected, don't enable i2c,
    // otherwise the MCU crashes.
    return;
  }

  palSetLineMode(LINE_MOTION_I2C_SCL, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN | PAL_STM32_OSPEED_HIGHEST);
  palSetLineMode(LINE_MOTION_I2C_SDA, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN | PAL_STM32_OSPEED_HIGHEST);

  /* I2C interface for MPU-6050 */
  i2cStart(&I2CD_MOTION, &i2ccfg_motion);

  chThdCreateStatic(waThreadAccel, sizeof(waThreadAccel), NORMALPRIO, ThreadAccel, NULL);
}
