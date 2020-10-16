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

#ifdef USE_LSM6DSL
#include "lsm6dsl.h"
/* LSM6DSL Driver: This object represent an LSM6DSL instance */
static  LSM6DSLDriver LSM6DSLD1;

static int32_t accraw[LSM6DSL_ACC_NUMBER_OF_AXES];
static int32_t gyroraw[LSM6DSL_GYRO_NUMBER_OF_AXES];

static float acccooked[LSM6DSL_ACC_NUMBER_OF_AXES];
static float gyrocooked[LSM6DSL_GYRO_NUMBER_OF_AXES];

static const LSM6DSLConfig lsm6dslcfg = {
  &I2CD_MOTION,
  &i2ccfg_motion,
  LSM6DSL_SAD_VCC,
  NULL,
  NULL,
  LSM6DSL_ACC_FS_8G,
  LSM6DSL_ACC_ODR_104Hz,
#if LSM6DSL_USE_ADVANCED
  LSM6DSL_ACC_LP_ENABLED,
#endif
  NULL,
  NULL,
  LSM6DSL_GYRO_FS_500DPS,
  LSM6DSL_GYRO_ODR_104Hz,
#if LSM6DSL_USE_ADVANCED
  LSM6DSL_GYRO_LP_ENABLED,
  LSM6DSL_GYRO_LPF_FTYPE1,
#endif
#if LSM6DSL_USE_ADVANCED
  LSM6DSL_BDU_BLOCKED,
  LSM6DSL_END_LITTLE
#endif
};
#endif

#define pow2(x) ((x)*(x))

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
  /* Reader thread loop.*/
  while (TRUE) {
    lsm6dslAccelerometerReadCooked(&LSM6DSLD1, acccooked);
    // gyroscopeReadCooked(&LSM6DSLD1, gyrocooked);
    lsm6dslGyroscopeReadRaw(&LSM6DSLD1, gyroraw);
    float acc_abs = sqrtf(pow2(acccooked[0]) + pow2(acccooked[1]) + pow2(acccooked[2]));
    if (acc_abs>0.001) {
      acccooked[0] /= acc_abs;
      acccooked[1] /= acc_abs;
      acccooked[2] /= acc_abs;
    }

    ax = -acccooked[0] * 32768.0; gx = -gyroraw[0];
    ay = -acccooked[1] * 32768.0; gy = -gyroraw[1];
    az = -acccooked[2] * 32768.0; gz = -gyroraw[2];

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
    *(synth_interface.acc_x) = acccooked[0];
    *(synth_interface.acc_y) = acccooked[1];
    *(synth_interface.acc_z) = acccooked[2];
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

#ifdef USE_MPU6050
  /* I2C interface for MPU-6050 */
  i2cStart(&I2CD_MOTION, &i2ccfg_motion);
#endif

#ifdef USE_LSM6DSL
  /* LSM6DSL Object Initialization.*/
  lsm6dslObjectInit(&LSM6DSLD1);

  /* Activates the LSM6DSL driver.*/
  lsm6dslStart(&LSM6DSLD1, &lsm6dslcfg);

  lsm6dslGyroscopeSampleBias(&LSM6DSLD1);
#endif

  chThdCreateStatic(waThreadAccel, sizeof(waThreadAccel), NORMALPRIO, ThreadAccel, NULL);
}
