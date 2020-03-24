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
#ifndef _CONFIG_H_
#define _CONFIG_H_

#define STRISOBOARD
#ifdef STRISOBOARD
//#define USE_BAS
//#define USE_UART
#define USE_USB
#define USE_AUX_BUTTONS
#define USE_MIDI_OUT
//#define USE_SYNTH_INTERFACE
#define USE_MPU6050
#define USE_WS2812
#else
#define USE_BAS
#define USE_UART
//#define USE_USB
//#define USE_AUX_BUTTONS
//#define USE_MIDI_OUT
//#define USE_SYNTH_INTERFACE
//#define USE_MPU6050
//#define USE_WS2812
#endif

//#define CALIBRATION_MODE TRUE
#define BUTTON_FILT

#define AUX_BUTTON_DEBOUNCE_TIME 5

#define MAX_VOICECOUNT 15

typedef enum {
  MIDI_MODE_MPE,
  MIDI_MODE_POLY,
  MIDI_MODE_MONO,
} midi_mode_t;

typedef struct struct_config {
  int message_interval;
  int send_usb_bulk;
  int send_motion_interval;
  int send_motion_14bit;
  int send_button_14bit;
  int midi_pres;
  int midi_bend;
  int midi_contvelo;
  midi_mode_t midi_mode;
} config_t;
extern config_t config;

#ifdef CONFIG_HERE
config_t config = {
  .message_interval = 1,     // interval in ms
  .send_usb_bulk = 0,         // send Striso binary protocol
  .send_motion_interval = 1,  // 0 = disable, else x10ms
  .send_motion_14bit = 0,     // send 14 bit motion CC
  .send_button_14bit = 0,     // send 14 bit MPE CC
  .midi_pres = 1,             // 1 = Channel Pressure, 2 = CC 70
  .midi_bend = 1,             // 2 = CC 71
  .midi_contvelo = 0,         // 0 = disable, 1 = enable
  .midi_mode = MIDI_MODE_MPE,
};
#endif

#endif
