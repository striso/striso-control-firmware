/*
    ChibiOS - Copyright (C) 2016..2019 Rocco Marco Guglielmi

    This file is part of ChibiOS.

    ChibiOS is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

/**
 * @file    lsm6dsl.h
 * @brief   LSM6DSL MEMS interface module header.
 */
#ifndef _LSM6DSL_H_
#define _LSM6DSL_H_

/**
 * @brief   LSM6DSL accelerometer subsystem characteristics.
 * @note    Sensitivity is expressed as milli-G/LSB whereas
 *          1 milli-G = 0.00980665 m/s^2.
 * @note    Bias is expressed as milli-G.
 *
 * @{
 */
#define LSM6DSL_ACC_NUMBER_OF_AXES          3U

#define LSM6DSL_ACC_2G                      2.0f
#define LSM6DSL_ACC_4G                      4.0f
#define LSM6DSL_ACC_8G                      8.0f
#define LSM6DSL_ACC_16G                     16.0f

#define LSM6DSL_ACC_SENS_2G                 0.061f
#define LSM6DSL_ACC_SENS_4G                 0.122f
#define LSM6DSL_ACC_SENS_8G                 0.244f
#define LSM6DSL_ACC_SENS_16G                0.488f

#define LSM6DSL_ACC_BIAS                    0.0f
/** @} */

/**
 * @brief   L3GD20 gyroscope system characteristics.
 * @note    Sensitivity is expressed as DPS/LSB whereas DPS stand for Degree
 *          per second [°/s].
 * @note    Bias is expressed as DPS.
 *
 * @{
 */
#define LSM6DSL_GYRO_NUMBER_OF_AXES         3U

#define LSM6DSL_GYRO_125DPS                 125.0f
#define LSM6DSL_GYRO_250DPS                 250.0f
#define LSM6DSL_GYRO_500DPS                 500.0f
#define LSM6DSL_GYRO_1000DPS                1000.0f
#define LSM6DSL_GYRO_2000DPS                2000.0f

#define LSM6DSL_GYRO_SENS_125DPS            0.004375f
#define LSM6DSL_GYRO_SENS_250DPS            0.008750f
#define LSM6DSL_GYRO_SENS_500DPS            0.017500f
#define LSM6DSL_GYRO_SENS_1000DPS           0.035000f
#define LSM6DSL_GYRO_SENS_2000DPS           0.070000f

#define LSM6DSL_GYRO_BIAS                   0.0f
/** @} */

/**
 * @name   LSM6DSL register addresses
 * @{
 */
#define LSM6DSL_AD_FUNC_CFG_ACCESS          0x01
#define LSM6DSL_AD_SENSOR_SYNC_TIME_FRAME   0x04
#define LSM6DSL_AD_SENSOR_SYNC_RES_RATIO    0x05
#define LSM6DSL_AD_FIFO_CTRL1               0x06
#define LSM6DSL_AD_FIFO_CTRL2               0x07
#define LSM6DSL_AD_FIFO_CTRL3               0x08
#define LSM6DSL_AD_FIFO_CTRL4               0x09
#define LSM6DSL_AD_FIFO_CTRL5               0x0A
#define LSM6DSL_AD_DRDY_PULSE_CFG_G         0x0B
#define LSM6DSL_AD_INT1_CTRL                0x0D
#define LSM6DSL_AD_INT2_CTRL                0x0E
#define LSM6DSL_AD_WHO_AM_I                 0x0F
#define LSM6DSL_AD_CTRL1_XL                 0x10
#define LSM6DSL_AD_CTRL2_G                  0x11
#define LSM6DSL_AD_CTRL3_C                  0x12
#define LSM6DSL_AD_CTRL4_C                  0x13
#define LSM6DSL_AD_CTRL5_C                  0x14
#define LSM6DSL_AD_CTRL6_C                  0x15
#define LSM6DSL_AD_CTRL7_G                  0x16
#define LSM6DSL_AD_CTRL8_XL                 0x17
#define LSM6DSL_AD_CTRL9_XL                 0x18
#define LSM6DSL_AD_CTRL10_C                 0x19
#define LSM6DSL_AD_MASTER_CONFIG            0x1A
#define LSM6DSL_AD_WAKE_UP_SRC              0x1B
#define LSM6DSL_AD_TAP_SRC                  0x1C
#define LSM6DSL_AD_D6D_SRC                  0x1D
#define LSM6DSL_AD_STATUS_REG               0x1E
#define LSM6DSL_AD_OUT_TEMP_L               0x20
#define LSM6DSL_AD_OUT_TEMP_H               0x21
#define LSM6DSL_AD_OUTX_L_G                 0x22
#define LSM6DSL_AD_OUTX_H_G                 0x23
#define LSM6DSL_AD_OUTY_L_G                 0x24
#define LSM6DSL_AD_OUTY_H_G                 0x25
#define LSM6DSL_AD_OUTZ_L_G                 0x26
#define LSM6DSL_AD_OUTZ_H_G                 0x27
#define LSM6DSL_AD_OUTX_L_XL                0x28
#define LSM6DSL_AD_OUTX_H_XL                0x29
#define LSM6DSL_AD_OUTY_L_XL                0x2A
#define LSM6DSL_AD_OUTY_H_XL                0x2B
#define LSM6DSL_AD_OUTZ_L_XL                0x2C
#define LSM6DSL_AD_OUTZ_H_XL                0x2D
#define LSM6DSL_AD_SENSORHUB1_REG           0x2E
#define LSM6DSL_AD_SENSORHUB2_REG           0x2F
#define LSM6DSL_AD_SENSORHUB3_REG           0x30
#define LSM6DSL_AD_SENSORHUB4_REG           0x31
#define LSM6DSL_AD_SENSORHUB5_REG           0x32
#define LSM6DSL_AD_SENSORHUB6_REG           0x33
#define LSM6DSL_AD_SENSORHUB7_REG           0x34
#define LSM6DSL_AD_SENSORHUB8_REG           0x35
#define LSM6DSL_AD_SENSORHUB9_REG           0x36
#define LSM6DSL_AD_SENSORHUB10_REG          0x37
#define LSM6DSL_AD_SENSORHUB11_REG          0x38
#define LSM6DSL_AD_SENSORHUB12_REG          0x39
#define LSM6DSL_AD_FIFO_STATUS1             0x3A
#define LSM6DSL_AD_FIFO_STATUS2             0x3B
#define LSM6DSL_AD_FIFO_STATUS3             0x3C
#define LSM6DSL_AD_FIFO_STATUS4             0x3D
#define LSM6DSL_AD_FIFO_DATA_OUT_L          0x3E
#define LSM6DSL_AD_FIFO_DATA_OUT_H          0x3F
#define LSM6DSL_AD_TIMESTAMP0_REG           0x40
#define LSM6DSL_AD_TIMESTAMP1_REG           0x41
#define LSM6DSL_AD_TIMESTAMP2_REG           0x42
#define LSM6DSL_AD_STEP_TIMESTAMP_L         0x49
#define LSM6DSL_AD_STEP_TIMESTAMP_H         0x4A
#define LSM6DSL_AD_STEP_COUNTER_L           0x4B
#define LSM6DSL_AD_STEP_COUNTER_H           0x4C
#define LSM6DSL_AD_SENSORHUB13_REG          0x4D
#define LSM6DSL_AD_SENSORHUB14_REG          0x4E
#define LSM6DSL_AD_SENSORHUB15_REG          0x4F
#define LSM6DSL_AD_SENSORHUB16_REG          0x50
#define LSM6DSL_AD_SENSORHUB17_REG          0x51
#define LSM6DSL_AD_SENSORHUB18_REG          0x52
#define LSM6DSL_AD_FUNC_SRC1                0x53
#define LSM6DSL_AD_FUNC_SRC2                0x54
#define LSM6DSL_AD_WRIST_TILT_IA            0x55
#define LSM6DSL_AD_TAP_CFG                  0x58
#define LSM6DSL_AD_TAP_THS_6D               0x59
#define LSM6DSL_AD_INT_DUR2                 0x5A
#define LSM6DSL_AD_WAKE_UP_THS              0x5B
#define LSM6DSL_AD_WAKE_UP_DUR              0x5C
#define LSM6DSL_AD_FREE_FALL                0x5D
#define LSM6DSL_AD_MD1_CFG                  0x5E
#define LSM6DSL_AD_MD2_CFG                  0x5F
#define LSM6DSL_AD_MASTER_CMD_CODE          0x60
#define LSM6DSL_AD_SENS_SYNC_SPI_ERROR_CODE 0x61
#define LSM6DSL_AD_OUT_MAG_RAW_X_L          0x66
#define LSM6DSL_AD_OUT_MAG_RAW_X_H          0x67
#define LSM6DSL_AD_OUT_MAG_RAW_Y_L          0x68
#define LSM6DSL_AD_OUT_MAG_RAW_Y_H          0x69
#define LSM6DSL_AD_OUT_MAG_RAW_Z_L          0x6A
#define LSM6DSL_AD_OUT_MAG_RAW_Z_H          0x6B
#define LSM6DSL_AD_X_OFS_USR                0x73
#define LSM6DSL_AD_Y_OFS_USR                0x74
#define LSM6DSL_AD_Z_OFS_USR                0x75
/** @} */

/**
 * @name    LSM6DSL_AD_CTRL1_XL register bits definitions
 * @{
 */
#define LSMDSL_CTRL1_XL_BW0_XL              (1 << 0)
#define LSMDSL_CTRL1_XL_LPF1_BW_SEL         (1 << 1)
#define LSMDSL_CTRL1_XL_FS_MASK             0x0C
#define LSMDSL_CTRL1_XL_FS_XL0              (1 << 2)
#define LSMDSL_CTRL1_XL_FS_XL1              (1 << 3)
#define LSMDSL_CTRL1_XL_ODR_XL0             (1 << 4)
#define LSMDSL_CTRL1_XL_ODR_XL1             (1 << 5)
#define LSMDSL_CTRL1_XL_ODR_XL2             (1 << 6)
#define LSMDSL_CTRL1_XL_ODR_XL3             (1 << 7)
/** @} */

/**
 * @name    LSM6DSL_AD_CTRL2_G register bits definitions
 * @{
 */
#define LSMDSL_CTRL2_G_FS_MASK              0x0E
#define LSMDSL_CTRL2_G_FS_125               (1 << 1)
#define LSMDSL_CTRL2_G_FS_G0                (1 << 2)
#define LSMDSL_CTRL2_G_FS_G1                (1 << 3)
#define LSMDSL_CTRL2_G_ODR_G0               (1 << 4)
#define LSMDSL_CTRL2_G_ODR_G1               (1 << 5)
#define LSMDSL_CTRL2_G_ODR_G2               (1 << 6)
#define LSMDSL_CTRL2_G_ODR_G3               (1 << 7)
/** @} */

/**
 * @name    LSM6DSL_AD_CTRL3_C register bits definitions
 * @{
 */
#define LSMDSL_CTRL3_C_SW_RESET             (1 << 0)
#define LSMDSL_CTRL3_C_BLE                  (1 << 1)
#define LSMDSL_CTRL3_C_IF_INC               (1 << 2)
#define LSMDSL_CTRL3_C_SIM                  (1 << 3)
#define LSMDSL_CTRL3_C_PP_OD                (1 << 4)
#define LSMDSL_CTRL3_C_H_LACTIVE            (1 << 5)
#define LSMDSL_CTRL3_C_BDU                  (1 << 6)
#define LSMDSL_CTRL3_C_BOOT                 (1 << 7)
/** @} */

/**
 * @name    LSM6DSL_AD_CTRL4_C register bits definitions
 * @{
 */
#define LSMDSL_CTRL4_C_NOT_USED_01          (1 << 0)
#define LSMDSL_CTRL4_C_LPF1_SEL_G           (1 << 1)
#define LSMDSL_CTRL4_C_I2C_DISABLE          (1 << 2)
#define LSMDSL_CTRL4_C_DRDY_MASK            (1 << 3)
#define LSMDSL_CTRL4_C_DEN_DRDY_IN          (1 << 4)
#define LSMDSL_CTRL4_C_INT2_ON_INT          (1 << 5)
#define LSMDSL_CTRL4_C_SLEEP                (1 << 6)
#define LSMDSL_CTRL4_C_DEN_XL_EN            (1 << 7)
/** @} */

/**
 * @name    LSM6DSL_AD_CTRL5_C register bits definitions
 * @{
 */
#define LSMDSL_CTRL5_C_ST0_XL               (1 << 0)
#define LSMDSL_CTRL5_C_ST1_XL               (1 << 1)
#define LSMDSL_CTRL5_C_ST0_G                (1 << 2)
#define LSMDSL_CTRL5_C_ST1_G                (1 << 3)
#define LSMDSL_CTRL5_C_DEN_LH               (1 << 4)
#define LSMDSL_CTRL5_C_ROUNDING0            (1 << 5)
#define LSMDSL_CTRL5_C_ROUNDING1            (1 << 6)
#define LSMDSL_CTRL5_C_ROUNDING2            (1 << 7)
/** @} */

/**
 * @name    LSM6DSL_AD_CTRL6_C register bits definitions
 * @{
 */
#define LSMDSL_CTRL6_C_FTYPE_0              (1 << 0)
#define LSMDSL_CTRL6_C_FTYPE_1              (1 << 1)
#define LSMDSL_CTRL6_C_USR_OFF_W            (1 << 3)
#define LSMDSL_CTRL6_C_XL_HM_MODE           (1 << 4)
#define LSMDSL_CTRL6_C_LVL2_EN              (1 << 5)
#define LSMDSL_CTRL6_C_LVL_EN               (1 << 6)
#define LSMDSL_CTRL6_C_TRIG_EN              (1 << 7)
/** @} */

/**
 * @name    LSM6DSL_AD_CTRL7_G register bits definitions
 * @{
 */
#define LSMDSL_CTRL7_G_ROUNDING_ST          (1 << 2)
#define LSMDSL_CTRL7_G_HPM0_G               (1 << 4)
#define LSMDSL_CTRL7_G_HPM1_G               (1 << 5)
#define LSMDSL_CTRL7_G_HP_EN_G              (1 << 6)
#define LSMDSL_CTRL7_G_G_HM_MODE            (1 << 7)
/** @} */

/**
 * @name    LSM6DSL_AD_CTRL8_XL register bits definitions
 * @{
 */
#define LSMDSL_CTRL8_XL_LOW_PASS_ON         (1 << 0)
#define LSMDSL_CTRL8_XL_HP_SLOPE_XL         (1 << 2)
#define LSMDSL_CTRL8_XL_INPUT_COMPO         (1 << 3)
#define LSMDSL_CTRL8_XL_HP_REF_MODE         (1 << 4)
#define LSMDSL_CTRL8_XL_HPCF_XL0            (1 << 5)
#define LSMDSL_CTRL8_XL_HPCF_XL1            (1 << 6)
#define LSMDSL_CTRL8_XL_LPF2_XL_EN          (1 << 7)
/** @} */

/**
 * @name    LSM6DSL_AD_CTRL9_XL register bits definitions
 * @{
 */
#define LSMDSL_CTRL9_XL_SOFT_EN             (1 << 2)
#define LSMDSL_CTRL9_XL_DEN_XL_G            (1 << 4)
#define LSMDSL_CTRL9_XL_DEN_Z               (1 << 5)
#define LSMDSL_CTRL9_XL_DEN_Y               (1 << 6)
#define LSMDSL_CTRL9_XL_DEN_X               (1 << 7)
/** @} */

/**
 * @name    LSM6DSL_AD_CTRL10_C register bits definitions
 * @{
 */
#define LSMDSL_CTRL10_C_SIGN_MOTION         (1 << 0)
#define LSMDSL_CTRL10_C_PEDO_RST_ST         (1 << 1)
#define LSMDSL_CTRL10_C_FUNC_EN             (1 << 2)
#define LSMDSL_CTRL10_C_TILT_EN             (1 << 3)
#define LSMDSL_CTRL10_C_PEDO_EN             (1 << 4)
#define LSMDSL_CTRL10_C_TIMER_EN            (1 << 5)
#define LSMDSL_CTRL10_C_WRIST_TILT          (1 << 7)
/** @} */

/**
 * @brief  Accelerometer and Gyroscope Slave Address.
 */
typedef enum {
  LSM6DSL_SAD_GND = 0x6A,           /**< SAD pin connected to GND.          */
  LSM6DSL_SAD_VCC = 0x6B            /**< SAD pin connected to VCC.          */
} lsm6dsl_sad_t;

/**
 * @brief   LSM6DSL accelerometer subsystem full scale.
 */
typedef enum {
  LSM6DSL_ACC_FS_2G = 0x00,         /**< Full scale �2g.                    */
  LSM6DSL_ACC_FS_4G = 0x40,         /**< Full scale �4g.                    */
  LSM6DSL_ACC_FS_8G = 0x80,         /**< Full scale �8g.                    */
  LSM6DSL_ACC_FS_16G = 0xC0         /**< Full scale �16g.                   */
} lsm6dsl_acc_fs_t;

/**
 * @brief   LSM6DSL accelerometer subsystem output data rate.
 */
typedef enum {
  LSM6DSL_ACC_ODR_PD = 0x00,        /**< Power down                         */
  LSM6DSL_ACC_ODR_1P6Hz = 0xB0,     /**< ODR 1.6 Hz (Low Power only)        */
  LSM6DSL_ACC_ODR_12P5Hz = 0x10,    /**< ODR 12.5 Hz                        */
  LSM6DSL_ACC_ODR_26Hz = 0x20,      /**< ODR 26 Hz                          */
  LSM6DSL_ACC_ODR_52Hz = 0x30,      /**< ODR 52 Hz                          */
  LSM6DSL_ACC_ODR_104Hz = 0x40,     /**< ODR 104 Hz                         */
  LSM6DSL_ACC_ODR_208Hz = 0x50,     /**< ODR 208 Hz                         */
  LSM6DSL_ACC_ODR_416Hz = 0x60,     /**< ODR 416 Hz                         */
  LSM6DSL_ACC_ODR_833Hz = 0x70,     /**< ODR 833 Hz                         */
  LSM6DSL_ACC_ODR_1P66Hz = 0x80,    /**< ODR 1.66 kHz                       */
  LSM6DSL_ACC_ODR_3P33Hz = 0x90,    /**< ODR 3.33 kHz                       */
  LSM6DSL_ACC_ODR_6P66Hz = 0xA0     /**< ODR 6.66 kHz                       */
} lsm6dsl_acc_odr_t;

/**
 * @brief   LSM6DSL accelerometer subsystem output data rate.
 */
typedef enum {
  LSM6DSL_ACC_LP_DISABLED = 0x00,   /**< Low power disabled                 */
  LSM6DSL_ACC_LP_ENABLED = 0x10     /**< Low power enabled                  */
} lsm6dsl_acc_lp_t;

/**
 * @brief LSM6DSL gyroscope subsystem full scale.
 */
typedef enum {
  LSM6DSL_GYRO_FS_125DPS  = 0x02,   /**< Full scale 125 degree per second  */
  LSM6DSL_GYRO_FS_250DPS  = 0x00,   /**< Full scale 250 degree per second  */
  LSM6DSL_GYRO_FS_500DPS  = 0x04,   /**< Full scale 500 degree per second  */
  LSM6DSL_GYRO_FS_1000DPS = 0x08,   /**< Full scale 1000 degree per second */
  LSM6DSL_GYRO_FS_2000DPS = 0x0C    /**< Full scale 2000 degree per second */
} lsm6dsl_gyro_fs_t;

/**
 * @brief   LSM6DSL gyroscope subsystem output data rate.
 */
typedef enum {
  LSM6DSL_GYRO_ODR_PD = 0x00,       /**< Power down                         */
  LSM6DSL_GYRO_ODR_12P5Hz = 0x10,   /**< ODR 12.5 Hz                        */
  LSM6DSL_GYRO_ODR_26Hz = 0x20,     /**< ODR 26 Hz                          */
  LSM6DSL_GYRO_ODR_52Hz = 0x30,     /**< ODR 52 Hz                          */
  LSM6DSL_GYRO_ODR_104Hz = 0x40,    /**< ODR 104 Hz                         */
  LSM6DSL_GYRO_ODR_208Hz = 0x50,    /**< ODR 208 Hz                         */
  LSM6DSL_GYRO_ODR_416Hz = 0x60,    /**< ODR 416 Hz                         */
  LSM6DSL_GYRO_ODR_833Hz = 0x70,    /**< ODR 833 Hz                         */
  LSM6DSL_GYRO_ODR_1P66Hz = 0x80,   /**< ODR 1.66 kHz                       */
  LSM6DSL_GYRO_ODR_3P33Hz = 0x90,   /**< ODR 3.33 kHz                       */
  LSM6DSL_GYRO_ODR_6P66Hz = 0xA0    /**< ODR 6.66 kHz                       */
} lsm6dsl_gyro_odr_t;

/**
 * @brief LSM6DSL gyroscope subsystem low mode configuration.
 */
typedef enum {
  LSM6DSL_GYRO_LP_DISABLED = 0x00,  /**< Low power mode disabled.           */
  LSM6DSL_GYRO_LP_ENABLED = 0x80    /**< Low power mode enabled.            */
} lsm6dsl_gyro_lp_t;

/**
 * @brief  LSM6DSL gyroscope subsystem output selection.
 */
typedef enum {
  LSM6DSL_GYRO_LPF_DISABLED = -1,   /**< Low pass filter disabled.          */
  LSM6DSL_GYRO_LPF_FTYPE0 = 0x00,   /**< Refer to table 68 of Datasheet.    */
  LSM6DSL_GYRO_LPF_FTYPE1 = 0x01,   /**< Refer to table 68 of Datasheet.    */
  LSM6DSL_GYRO_LPF_FTYPE2 = 0x10,   /**< Refer to table 68 of Datasheet.    */
  LSM6DSL_GYRO_LPF_FTYPE3 = 0x11    /**< Refer to table 68 of Datasheet.    */
} lsm6dsl_gyro_lpf_t;

/**
 * @brief LSM6DSL block data update.
 */
typedef enum {
  LSM6DSL_BDU_CONTINUOUS = 0x00,    /**< Block data continuously updated.   */
  LSM6DSL_BDU_BLOCKED = 0x40        /**< Block data updated after reading.  */
} lsm6dsl_bdu_t;

/**
 * @brief LSM6DSL endianness.
 */
typedef enum {
  LSM6DSL_END_LITTLE = 0x00,        /**< Little endian.                     */
  LSM6DSL_END_BIG = 0x20            /**< Big endian.                        */
} lsm6dsl_end_t;

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  LSM6DSL_UNINIT = 0,               /**< Not initialized.                   */
  LSM6DSL_STOP = 1,                 /**< Stopped.                           */
  LSM6DSL_READY = 2,                /**< Ready.                             */
} lsm6dsl_state_t;

#endif /* _LSM6DSL_H_ */