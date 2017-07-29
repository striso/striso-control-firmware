// I2Cdev library collection - MPU6050 I2C device class header file
// Based on InvenSense MPU-6050 register map document rev. 2.0, 5/19/2011 (RM-MPU-6000A-00)
// 10/3/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     ... - ongoing debug release

// NOTE: THIS IS ONLY A PARIAL RELEASE. THIS DEVICE CLASS IS CURRENTLY UNDERGOING ACTIVE
// DEVELOPMENT AND IS STILL MISSING SOME IMPORTANT FEATURES. PLEASE KEEP THIS IN MIND IF
// YOU DECIDE TO USE THIS PARTICULAR CODE FOR ANYTHING.

/* ChibiOS I2Cdev MPU6050 device class conversion 2/5/2013 by Jan Schlemminger - C conversion, ChibiOS compliance
 * First release. I just tested a few functions so this should be considered HIGHLY EXPERIMENTAL!!!
 * Feel free to test and report bugs. Updates at https://github.com/jevermeister/MPU6050-ChibiOS
*/

/* ============================================
ChibiOS I2Cdev MPU6050 device class code is placed under the MIT license
Copyright (c) 2012 Jan Schlemminger

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include "ch.h"
#include "hal.h"
#include "MPU6050.h"
#include "motionsensor.h"

uint8_t txbuf[2];
uint8_t rxbuf[14] = {0};

void MPU6050_writeReg(uint8_t addr, uint8_t data) {
    txbuf[0]=addr;
    txbuf[1]=data;
    i2cMasterTransmitTimeout(&I2CD_MOTION, MPU6050_ADDRESS, txbuf, 2, NULL, 0, MS2ST(4));
}

uint8_t MPU6050_readReg(uint8_t addr) {
    txbuf[0]=addr;
    i2cMasterTransmitTimeout(&I2CD_MOTION, MPU6050_ADDRESS, txbuf, 1, rxbuf, 2, MS2ST(4));
    return rxbuf[0];
}

/** Power on and prepare for general usage.
 * This will activate the device and take it out of sleep mode (which must be done
 * after start-up). This function also sets both the accelerometer and the gyroscope
 * to their most sensitive settings, namely +/- 2g and +/- 250 degrees/sec, and sets
 * the clock source to use the X Gyro for reference, which is slightly better than
 * the default internal clock source.
 */
void MPUinitialize() {
    MPU6050_writeReg(MPU6050_RA_GYRO_CONFIG, MPU6050_GYRO_CONFIG_FS_500);
    MPU6050_writeReg(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACCEL_CONFIG_FS_8); // doesn't work somehow
    MPU6050_writeReg(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLOCK_PLL_XGYRO);
}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return TRUE if connection is valid, FALSE otherwise
 */
bool MPUtestConnection() {
    return MPU6050_readReg(MPU6050_RA_WHO_AM_I) == 0x34;
}

/** Get raw 6-axis motion sensor readings (accel/gyro).
 * Retrieves all currently available motion sensor values.
 * @param ax 16-bit signed integer container for accelerometer X-axis value
 * @param ay 16-bit signed integer container for accelerometer Y-axis value
 * @param az 16-bit signed integer container for accelerometer Z-axis value
 * @param gx 16-bit signed integer container for gyroscope X-axis value
 * @param gy 16-bit signed integer container for gyroscope Y-axis value
 * @param gz 16-bit signed integer container for gyroscope Z-axis value
 * @see getAcceleration()
 * @see getRotation()
 * @see MPU6050_RA_ACCEL_XOUT_H
 */
void MPUgetMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
    txbuf[0]=MPU6050_RA_ACCEL_XOUT_H;
    i2cMasterTransmitTimeout(&I2CD_MOTION, MPU6050_ADDRESS, txbuf, 1, rxbuf, 14, MS2ST(4));
    *ax = (((int16_t)rxbuf[0]) << 8) | rxbuf[1];
    *ay = (((int16_t)rxbuf[2]) << 8) | rxbuf[3];
    *az = (((int16_t)rxbuf[4]) << 8) | rxbuf[5];
    *gx = (((int16_t)rxbuf[8]) << 8) | rxbuf[9];
    *gy = (((int16_t)rxbuf[10]) << 8) | rxbuf[11];
    *gz = (((int16_t)rxbuf[12]) << 8) | rxbuf[13];
}
void MPUgetMotion6t(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* t) {
    txbuf[0]=MPU6050_RA_ACCEL_XOUT_H;
    i2cMasterTransmitTimeout(&I2CD_MOTION, MPU6050_ADDRESS, txbuf, 1, rxbuf, 14, MS2ST(4));
    *ax = (((int16_t)rxbuf[0]) << 8) | rxbuf[1];
    *ay = (((int16_t)rxbuf[2]) << 8) | rxbuf[3];
    *az = (((int16_t)rxbuf[4]) << 8) | rxbuf[5];
    *gx = (((int16_t)rxbuf[8]) << 8) | rxbuf[9];
    *gy = (((int16_t)rxbuf[10]) << 8) | rxbuf[11];
    *gz = (((int16_t)rxbuf[12]) << 8) | rxbuf[13];
    *t = (((((int16_t)rxbuf[6]) << 8) | rxbuf[7]) + 12412 + 34/2) / 34;
}

/** Get temperature of last MPUgetMotion6 call in decidegree
 */
int MPUgetTemperature(void) {
  return (((((int)rxbuf[6]) << 8) | rxbuf[7]) + 12412 + 34/2) / 34;
}

/** Get 3-axis accelerometer readings.
 * These registers store the most recent accelerometer measurements.
 * Accelerometer measurements are written to these registers at the Sample Rate
 * as defined in Register 25.
 *
 * The accelerometer measurement registers, along with the temperature
 * measurement registers, gyroscope measurement registers, and external sensor
 * data registers, are composed of two sets of registers: an internal register
 * set and a user-facing read register set.
 *
 * The data within the accelerometer sensors' internal register set is always
 * updated at the Sample Rate. Meanwhile, the user-facing read register set
 * duplicates the internal register set's data values whenever the serial
 * interface is idle. This guarantees that a burst read of sensor registers will
 * read measurements from the same sampling instant. Note that if burst reads
 * are not used, the user is responsible for ensuring a set of single byte reads
 * correspond to a single sampling instant by checking the Data Ready interrupt.
 *
 * Each 16-bit accelerometer measurement has a full scale defined in ACCEL_FS
 * (Register 28). For each full scale setting, the accelerometers' sensitivity
 * per LSB in ACCEL_xOUT is shown in the table below:
 *
 * <pre>
 * AFS_SEL | Full Scale Range | LSB Sensitivity
 * --------+------------------+----------------
 * 0       | +/- 2g           | 8192 LSB/mg
 * 1       | +/- 4g           | 4096 LSB/mg
 * 2       | +/- 8g           | 2048 LSB/mg
 * 3       | +/- 16g          | 1024 LSB/mg
 * </pre>
 *
 * @param x 16-bit signed integer container for X-axis acceleration
 * @param y 16-bit signed integer container for Y-axis acceleration
 * @param z 16-bit signed integer container for Z-axis acceleration
 * @see MPU6050_RA_GYRO_XOUT_H
 */
void MPUgetAcceleration(int16_t* x, int16_t* y, int16_t* z) {
    txbuf[0]=MPU6050_RA_ACCEL_XOUT_H;
    i2cMasterTransmitTimeout(&I2CD_MOTION, MPU6050_ADDRESS, txbuf, 1, rxbuf, 6, MS2ST(4));
    *x = (((int16_t)rxbuf[0]) << 8) | rxbuf[1];
    *y = (((int16_t)rxbuf[2]) << 8) | rxbuf[3];
    *z = (((int16_t)rxbuf[4]) << 8) | rxbuf[5];
}

// GYRO_*OUT_* registers

/** Get 3-axis gyroscope readings.
 * These gyroscope measurement registers, along with the accelerometer
 * measurement registers, temperature measurement registers, and external sensor
 * data registers, are composed of two sets of registers: an internal register
 * set and a user-facing read register set.
 * The data within the gyroscope sensors' internal register set is always
 * updated at the Sample Rate. Meanwhile, the user-facing read register set
 * duplicates the internal register set's data values whenever the serial
 * interface is idle. This guarantees that a burst read of sensor registers will
 * read measurements from the same sampling instant. Note that if burst reads
 * are not used, the user is responsible for ensuring a set of single byte reads
 * correspond to a single sampling instant by checking the Data Ready interrupt.
 *
 * Each 16-bit gyroscope measurement has a full scale defined in FS_SEL
 * (Register 27). For each full scale setting, the gyroscopes' sensitivity per
 * LSB in GYRO_xOUT is shown in the table below:
 *
 * <pre>
 * FS_SEL | Full Scale Range   | LSB Sensitivity
 * -------+--------------------+----------------
 * 0      | +/- 250 degrees/s  | 131 LSB/deg/s
 * 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
 * 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
 * 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
 * </pre>
 *
 * @param x 16-bit signed integer container for X-axis rotation
 * @param y 16-bit signed integer container for Y-axis rotation
 * @param z 16-bit signed integer container for Z-axis rotation
 * @see getMotion6()
 * @see MPU6050_RA_GYRO_XOUT_H
 */
void MPUgetRotation(int16_t* x, int16_t* y, int16_t* z) {
    txbuf[0]=MPU6050_RA_GYRO_XOUT_H;
    i2cMasterTransmitTimeout(&I2CD_MOTION, MPU6050_ADDRESS, txbuf, 1, rxbuf, 6, MS2ST(4));
    *x = (((int16_t)rxbuf[0]) << 8) | rxbuf[1];
    *y = (((int16_t)rxbuf[2]) << 8) | rxbuf[3];
    *z = (((int16_t)rxbuf[4]) << 8) | rxbuf[5];
}
