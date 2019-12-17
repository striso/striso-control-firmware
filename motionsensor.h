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
#ifndef _MOTIONSENSOR_H_
#define _MOTIONSENSOR_H_

#define I2CD_MOTION I2CD3
#define MOTION_I2C_SDA_PORT GPIOC
#define MOTION_I2C_SDA_PIN GPIOC_I2C3_SDA
#define MOTION_I2C_SCL_PORT GPIOA
#define MOTION_I2C_SCL_PIN GPIOA_I2C3_SCL

void MotionSensorStart(void);

#endif
