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
#ifndef _STRISO_H
#define _STRISO_H

/**
Striso binary protocol

byte[0]:   0b1iiiisss, header
           iiii = 4 bit source ID
           sss = 3 bit message size = number of 14 bit values
byte[1]:   0b0ccccccc, 7 bit control ID
byte[2-3]: 0b0vvvvvvv * 2, signed 14 bit value 1
byte[4-5]: signed 14 bit value 2
etc. up to 7 values

Button message:
6 values: signal[0-2], velocity[0-2]    (up to v2.0.5)
4 values: pres, vpres, x, y             (v2.1.0 and later)
*/

// Striso protocol source ID's [0-15]
#define ID_DIS 0      // buttons on dis side
#define ID_BAS 1      // buttons on bas side
#define ID_CONTROL 2  // other controls
#define ID_ACCEL 3    // motion sensor
#define ID_SYS 4      // system, status, debug
#define ID_MIDI 5     // midi monitor

// control numbers [0-127]
#define IDC_PORTAMENTO 0      // length 1
#define IDC_OCT_UP 1          // length 1
#define IDC_OCT_DOWN 2        // length 1
#define IDC_ALT 3             // length 1
#define IDC_SLD_NPRESS 8      // length 1
#define IDC_SLD_SLIDE 9       // length 1
#define IDC_SLD_SLIDEZOOM 10  // length 2
#define IDC_SLD_ONE_TAP 3     // length 1
#define IDC_SLD_TWO_TAP 4     // length 1
#define IDC_PEDAL_EXP 11      // length 1
#define IDC_PEDAL_1 12        // length 1
#define IDC_PEDAL_2 13        // length 1
#define IDC_PEDAL_3 14        // length 1

// system numbers [0-127]
#define ID_SYS_MSGQUE_OVERFLOW_BB 1
#define ID_SYS_MSGQUE_OVERFLOW_SYNTH 2
#define ID_SYS_MSG_TOO_SHORT_SYNTH 3
#define ID_SYS_MSG_TOO_LONG_SYNTH 4
#define ID_SYS_BATTERY_VOLTAGE 5
#define ID_SYS_TEMPERATURE 6

#endif
