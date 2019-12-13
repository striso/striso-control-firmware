
#include "ch.h"
#include "hal.h"

#include "motionsensor.h"
#include "config.h"
#include "striso.h"
#include "MPU6050.h"
#include "synth.h"
#include "messaging.h"
#include <math.h>

#define pow2(x) ((x)*(x))

/* I2C interface #1 */
static const I2CConfig i2ccfg_motion = {
  OPMODE_I2C,
  400000,
  FAST_DUTY_CYCLE_2,
};

/*
 * This is a periodic thread that reads accelerometer and sends messages
 */
static WORKING_AREA(waThreadAccel, 256);
__attribute__  ((noreturn))
static msg_t ThreadAccel(void *arg) {
  (void)arg;
  chRegSetThreadName("motion");

  int16_t ax, ay, az, gx, gy, gz;
  int msg[9];
  msg[0] = ID_ACCEL;
  msg[1] = 8; // range?

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

  int n = 0;

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
    float rot_x = ((float)gx)*(1.0/32768.0);
    float rot_y = ((float)gy)*(1.0/32768.0);
    float rot_z = ((float)gz)*(1.0/32768.0);

    msg[2] = ax>>2;
    msg[3] = ay>>2;
    msg[4] = az>>2;
    msg[5] = ((int16_t)(acc_abs * 32768.0/8.0))>>2;
    msg[6] = gx>>2;
    msg[7] = gy>>2;
    msg[8] = gz>>2;
    msgSend(9,msg);

#ifdef USE_SYNTH_INTERFACE
    *(synth_interface.acc_abs) = acc_abs;
    *(synth_interface.acc_x) = acc_x;
    *(synth_interface.acc_y) = acc_y;
    *(synth_interface.acc_z) = acc_z;
    *(synth_interface.rot_x) = rot_x;
    *(synth_interface.rot_y) = rot_y;
    *(synth_interface.rot_z) = rot_z;
#endif
    //if (n-- <= 0) {
    //  palToggleLine(LINE_LED4);       /* Blue */
    //  n = 100;
    //}

    /* Waiting until the next 10 milliseconds time interval.*/
    // TODO: wait for interrupt at MPU_READY pin
    chThdSleepMilliseconds(3);
  }
}

void MotionSensorStart(void) {
  // Check if pullup resistors are attached to i2c port
  // If not there is no motion sensor daughterboard
  palSetPadMode(MOTION_I2C_SCL_PORT, MOTION_I2C_SCL_PIN, PAL_MODE_INPUT_PULLDOWN);
  palSetPadMode(MOTION_I2C_SDA_PORT, MOTION_I2C_SDA_PIN, PAL_MODE_INPUT_PULLDOWN);

  if (!palReadPad(MOTION_I2C_SCL_PORT, MOTION_I2C_SCL_PIN) || 
      !palReadPad(MOTION_I2C_SDA_PORT, MOTION_I2C_SDA_PIN)) {
    // No pullups on i2c pads detected, don't enable i2c,
    // otherwise the MCU crashes.
    return;
  }

  /* I2C interface for MPU-6050 */
  i2cStart(&I2CD_MOTION, &i2ccfg_motion);
  palSetPadMode(MOTION_I2C_SCL_PORT, MOTION_I2C_SCL_PIN, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);
  palSetPadMode(MOTION_I2C_SDA_PORT, MOTION_I2C_SDA_PIN, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);

  chThdCreateStatic(waThreadAccel, sizeof(waThreadAccel), NORMALPRIO, ThreadAccel, NULL);
}
