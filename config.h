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
#define USE_MIDI_SERIAL
#define USE_INTERNAL_SYNTH
// #define USE_MPU6050
#define USE_LSM6DSL
// #define USE_WS2812

// #define SAMPLERATE 44100
// #define SAMPLERATE 48000
#define SAMPLERATE 48828    // 48828.125 TODO: find clock settings to get 48000 Hz
#define SAMPLINGFREQ SAMPLERATE
#define CHANNEL_BUFFER_SIZE		32
#define PLAYBACK_BUFFER_SIZE	(CHANNEL_BUFFER_SIZE*2)
#define CODEC_ENABLE_INPUT    TRUE

#else
#define USE_BAS
#define USE_UART
//#define USE_USB
//#define USE_AUX_BUTTONS
//#define USE_MIDI_OUT
//#define USE_INTERNAL_SYNTH
//#define USE_MPU6050
//#define USE_WS2812
#endif

// #define CALIBRATION_MODE TRUE      // don't linearize or calibrate sensor data
// #define BUTTON_FILT                // filter out erroneous presses
// #define DETECT_STUCK_NOTES         // dynamic zero level detection
// #define BREAKPOINT_CALIBRATION     // button sensitivity correction using a breakpoint fit

#define CALIB_OFFSET 4
#define CALIB_FORCE  ((1<<18)/64 + 1)   // +1 to signify hardware revision, 1k adc pull up resistors

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
  int common_channel_filter;
  midi_mode_t midi_mode;
  unsigned int midi_pres;
  unsigned int midi_x;
  unsigned int midi_y;
  unsigned int mpe_pres;
  unsigned int mpe_x;
  unsigned int mpe_y;
  unsigned int mpe_contvelo;
} config_t;
extern config_t config;

#define CFG_DISABLE               127
#define CFG_POLY_PRESSURE         120
#define CFG_CHANNEL_PRESSURE      121
#define CFG_PITCH_BEND            122

#ifdef CONFIG_HERE
// default config
config_t config = {
  .message_interval = 1,      // interval in ms
  .send_usb_bulk = 0,         // send Striso binary protocol
  .send_motion_interval = 1,  // 0 = disable, else x10ms
  .send_motion_14bit = 0,     // send 14 bit motion CC
  .send_button_14bit = 0,     // send 14 bit MPE CC
  .common_channel_filter = 0,
  .midi_mode = MIDI_MODE_MPE,
  // for the following: < 120: CC, or CFG_* (not all options are supported)
  .midi_pres = CFG_POLY_PRESSURE,
  .midi_x = CFG_PITCH_BEND,
  .midi_y = 74,
  .mpe_pres = CFG_CHANNEL_PRESSURE,
  .mpe_x = CFG_PITCH_BEND,
  .mpe_y = 74,
  .mpe_contvelo = CFG_DISABLE,
};
#endif

/** Device specific storage in the last flash sector
 * Stores serial number, model description, calibration values, etc.
 *
 * hardware_revision:
 *    0: v2.0.3 pcb (10k adc pullups)
 *    1: v2.0.3 pcb with 1k adc pullups
 *
 * calib type:
 *    0x00: linear force correction, measured per sensor (<= v2.0.5)
 *    0x02: linear force correction, measured per key (>= v2.1)
 *    0xb1: breakpoint location
 *    0x01: additional linear force correction after breakpoint
 */
#define DEVSPEC_FLASH_START 0x081e0000

typedef struct {
  const uint32_t UID[3];
  const unsigned char id[16];
  const unsigned char model[64];
  const uint16_t hardware_revision;
  const unsigned char reserved[256-3*4-16-64-2-2];
  const uint16_t base_calib_force;
} devspec_id_t;
typedef char assert_devspec_id_size[sizeof(devspec_id_t) == 256 ? 1 : -1]; // static assert trick
#define devspec_id ((devspec_id_t*)(DEVSPEC_FLASH_START))

typedef struct {
  const uint32_t UID[3];
  const uint16_t calib[68];
  const unsigned char reserved[256-3*4-68*2-2-16];
  const uint16_t type;
  const unsigned char date[16];
} calib_t;
typedef char assert_calib_size[sizeof(calib_t) == 256 ? 1 : -1]; // static assert trick
#define calib_dis_force ((calib_t*)(DEVSPEC_FLASH_START + 256))
#define calib_dis_breakpoint ((calib_t*)(DEVSPEC_FLASH_START + 2*256))
#define calib_dis_force2 ((calib_t*)(DEVSPEC_FLASH_START + 3*256))

#endif
