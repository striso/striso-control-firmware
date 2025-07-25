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

#include <stdint.h>
#include "config_store.h"

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

// #define OLD_CONFIG_LAYOUT

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
#define DETECT_STUCK_NOTES         // zero level detection
// #define DETECT_STUCK_NOTES_DECREASE  // zero level detection, allow notes to come back
// #define BREAKPOINT_CALIBRATION     // button sensitivity correction using a breakpoint fit

#define CALIB_FORCE  ((1<<18)/64 + 1)   // +1 to signify hardware revision, 1k adc pull up resistors

#define AUX_BUTTON_DEBOUNCE_TIME 5

#define MAX_VOICECOUNT 15

typedef enum {
  MIDI_MODE_MPE,
  MIDI_MODE_POLY,
  MIDI_MODE_MONO,
} midi_mode_t;

typedef enum {
  MIDINOTE_MODE_DEFAULT,
  MIDINOTE_MODE_TUNING,
  MIDINOTE_MODE_BUTTON,
} midinote_mode_t;

typedef enum {
  JACK2_MODE_DISABLED = 0,
  JACK2_MODE_MIDI,
  JACK2_MODE_PEDAL_EXPRESSION,
  JACK2_MODE_PEDAL_SWITCH,
  JACK2_MODE_LINEIN,
  JACK2_MODE_AUTODETECT,
  JACK2_MODE_ERROR,
} jack2_mode_t;

typedef struct struct_config {
  int message_interval;
  int send_usb_bulk;
  bool send_midi_monitor;
  int send_motion_interval;
  int send_motion_14bit;
  int send_button_14bit;
  int zero_offset;
  int debug;
  midi_mode_t midi_mode;
  midinote_mode_t midinote_mode;
  jack2_mode_t jack2_mode_setting; // mode from settings
  jack2_mode_t jack2_mode; // active mode (when jack is inserted)
  uint8_t midi_pres;
  uint8_t midi_x;
  uint8_t midi_y;
  uint8_t mpe_pres;
  uint8_t mpe_x;
  uint8_t mpe_y;
  uint8_t mpe_contvelo;
  uint8_t pedal_sw1_values[2];
  uint8_t altkey_pedal;
  uint8_t key_swap;
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
  .send_midi_monitor = 0,     // monitor MIDI in over Striso protocol
  .send_motion_interval = 127,// 0 = disable, 127 only internal, else x10ms
  .send_motion_14bit = 0,     // send 14 bit motion CC
  .send_button_14bit = 0,     // send 14 bit MPE CC
  .zero_offset = 0,
  .debug = 0,
  .midi_mode = MIDI_MODE_MPE,
  .midinote_mode = MIDINOTE_MODE_DEFAULT,
  .jack2_mode_setting = JACK2_MODE_AUTODETECT,
  .jack2_mode = JACK2_MODE_DISABLED,
  // for the following: < 120: CC, or CFG_* (not all options are supported)
  .midi_pres = CFG_POLY_PRESSURE,
  .midi_x = CFG_PITCH_BEND,
  .midi_y = 74,
  .mpe_pres = CFG_CHANNEL_PRESSURE,
  .mpe_x = CFG_PITCH_BEND,
  .mpe_y = 74,
  .mpe_contvelo = CFG_DISABLE,
  .pedal_sw1_values = {32, 96},
  .altkey_pedal = false,
  .key_swap = false,
};

/*
Config value types
M Meta, Config header and footer
s string
i integer
f float
h hexadecimal

General: tGxxx

Presets: tPnxxx

Tuning: tTnxxx
Generators for octave and fifth
Tuning per octave (17 values)
Tuning for every button

Calibration: tCxxx
User calibration values
*/

// CC_ALIGN(8)
const ConfigParam default_config[] = {
  {"MConfig ", "v1.0    "}, // start tag

  // general settings
  {"iGpreset", "        "}, // Preset to load on start [1-8]
  {"iGoct   ", "0       "}, // default octave [-2..2]
  {"sGaltkey", "settings"}, // Settings key mode [settings: Use key for settings (default), pedal: Disable settings and use key as pedal (for simplified use)]
  {"sGkeyord", "normal  "}, // Swap settings/glissando keys [normal: (default), swap: Swap settings and glissando key location]

  // preset 1
  {"sP1name ", "preset1 "},
  {"hP1color", "#380000 "},
  {"iP1Mpgm ", "        "}, // MIDI program change [0-127]
  {"iP1Mint ", "1       "}, // MIDI message interval in ms [1-127]
  {"iP1Mmint", "127     "}, // MIDI motion sensor message interval. 0 = disable, 127 only internal, else x10ms [0-127]
  {"sP1Mmode", "mpe     "}, // MIDI mode [mpe/normal/mono]
  {"sP1Mnote", "default "}, // MIDI note mode [default/tuning/button]
  {"sP1jack2", "auto    "}, // jack2 mode [auto/midi/pedal_ex/pedal_sw/linein]
  {"iP1tunin", "0       "}, // load tuning [0-8]
  {"fP1Toff ", "        "}, // additional tuning offset in cent
  {"sP1flip ", "no      "}, // Flip layout 180 degrees [no/yes]
  {"iP1Mpres", "121     "}, // MIDI CC for key pressure. Special values 127 = Disable, 120 = Polyphonic Pressure, 121 = Channel Pressure
  {"iP1Mx   ", "122     "}, // MIDI CC for key X movement. Special values 127 = Disable, 122 = Pitch Bend
  {"iP1My   ", "74      "}, // MIDI CC for key Y movement. Special values 127 = Disable
  {"iP1Mvelo", "127     "}, // MIDI continuous velocity. Special values 127 = Disable, else send continuous velocity on CC73 and continuous release velocity on CC72
  {"iP1voice", "6       "}, // Voice count/MPE channel count [1-15]
  {"iP1MChan", "2       "}, // MIDI channel [1-16]
  {"iP1MPEpb", "48      "}, // MPE pitch bend range [12/24/48/96]
  {"fP1thres", "0.0     "}, // Key sensitivity threshold [0-1]
  {"fP1bendS", "0.25    "}, // Pitch bend range in semitones [-4.0-4.0]
  {"fP1presS", "1.0     "}, // Key pressure factor [0-4]
  {"fP1veloS", "1.0     "}, // Key velocity factor [0-4]
  {"fP1tiltS", "1.0     "}, // Key tilt factor [-4.0-4.0]
  {"iP1veloO", "0       "}, // MIDI velocity offset [0-127]
  {"fP1volum", "90.0    "}, // Volume [0-127]
  {"fP1decay", "32.0    "}, // Decay, how long it takes for the sound to decay after atack or release [0-127]
  {"fP1decaP", "96.0    "}, // Pedal decay, decay value when sustain pedal is pressed [0-127]

  // preset 2
  {"sP2name ", "preset2 "},
  {"hP2color", "#342000 "},
  {"iP2Mpgm ", "        "}, // MIDI program change [0-127]
  {"iP2Mint ", "        "}, // MIDI message interval in ms [1-127]
  {"iP2Mmint", "        "}, // MIDI motion sensor message interval. 0 = disable, 127 only internal, else x10ms [0-127]
  {"sP2Mmode", "        "}, // MIDI mode [mpe/normal/mono]
  {"sP2Mnote", "        "}, // MIDI note mode [default/tuning/button]
  {"sP2jack2", "        "}, // jack2 mode [midi/pedal/linein]
  {"iP2tunin", "        "}, // load tuning [0-8]
  {"fP2Toff ", "        "}, // additional tuning offset in cent
  {"sP2flip ", "        "}, // Flip layout 180 degrees [no/yes]
  {"iP2Mpres", "        "}, // MIDI CC for key pressure. Special values 127 = Disable, 120 = Polyphonic Pressure, 121 = Channel Pressure
  {"iP2Mx   ", "        "}, // MIDI CC for key X movement. Special values 127 = Disable, 122 = Pitch Bend
  {"iP2My   ", "        "}, // MIDI CC for key Y movement. Special values 127 = Disable
  {"iP2Mvelo", "        "}, // MIDI continuous velocity. Special values 127 = Disable, else send continuous velocity on CC73 and continuous release velocity on CC72
  {"iP2voice", "        "}, // Voice count/MPE channel count [1-15]
  {"iP2MChan", "        "}, // MIDI channel [1-16]
  {"iP2MPEpb", "        "}, // MPE pitch bend range [12/24/48/96]
  {"fP2thres", "        "}, // Key sensitivity threshold [0-1]
  {"fP2bendS", "        "}, // Pitch bend range in semitones [-4.0-4.0]
  {"fP2presS", "        "}, // Key pressure factor [0-4]
  {"fP2veloS", "        "}, // Key velocity factor [0-4]
  {"fP2tiltS", "        "}, // Key tilt factor [-4.0-4.0]
  {"iP2veloO", "        "}, // MIDI velocity offset [0-127]
  {"fP2volum", "        "}, // Volume [0-127]
  {"fP2decay", "        "}, // Decay, how long it takes for the sound to decay after atack or release [0-127]
  {"fP2decaP", "        "}, // Pedal decay, decay value when sustain pedal is pressed [0-127]

  // preset 3
  {"sP3name ", "preset3 "},
  {"hP3color", "#383800 "},
  {"iP3Mpgm ", "        "}, // MIDI program change [0-127]
  {"iP3Mint ", "        "}, // MIDI message interval in ms [1-127]
  {"iP3Mmint", "        "}, // MIDI motion sensor message interval. 0 = disable, 127 only internal, else x10ms [0-127]
  {"sP3Mmode", "        "}, // MIDI mode [mpe/normal/mono]
  {"sP3Mnote", "        "}, // MIDI note mode [default/tuning/button]
  {"sP3jack2", "        "}, // jack2 mode [midi/pedal/linein]
  {"iP3tunin", "        "}, // load tuning [0-8]
  {"fP3Toff ", "        "}, // additional tuning offset in cent
  {"sP3flip ", "        "}, // Flip layout 180 degrees [no/yes]
  {"iP3Mpres", "        "}, // MIDI CC for key pressure. Special values 127 = Disable, 120 = Polyphonic Pressure, 121 = Channel Pressure
  {"iP3Mx   ", "        "}, // MIDI CC for key X movement. Special values 127 = Disable, 122 = Pitch Bend
  {"iP3My   ", "        "}, // MIDI CC for key Y movement. Special values 127 = Disable
  {"iP3Mvelo", "        "}, // MIDI continuous velocity. Special values 127 = Disable, else send continuous velocity on CC73 and continuous release velocity on CC72
  {"iP3voice", "        "}, // Voice count/MPE channel count [1-15]
  {"iP3MChan", "        "}, // MIDI channel [1-16]
  {"iP3MPEpb", "        "}, // MPE pitch bend range [12/24/48/96]
  {"fP3thres", "        "}, // Key sensitivity threshold [0-1]
  {"fP3bendS", "        "}, // Pitch bend range in semitones [-4.0-4.0]
  {"fP3presS", "        "}, // Key pressure factor [0-4]
  {"fP3veloS", "        "}, // Key velocity factor [0-4]
  {"fP3tiltS", "        "}, // Key tilt factor [-4.0-4.0]
  {"iP3veloO", "        "}, // MIDI velocity offset [0-127]
  {"fP3volum", "        "}, // Volume [0-127]
  {"fP3decay", "        "}, // Decay, how long it takes for the sound to decay after atack or release [0-127]
  {"fP3decaP", "        "}, // Pedal decay, decay value when sustain pedal is pressed [0-127]

  // preset 4
  {"sP4name ", "preset4 "},
  {"hP4color", "#0e6000 "},
  {"iP4Mpgm ", "        "}, // MIDI program change [0-127]
  {"iP4Mint ", "        "}, // MIDI message interval in ms [1-127]
  {"iP4Mmint", "        "}, // MIDI motion sensor message interval. 0 = disable, 127 only internal, else x10ms [0-127]
  {"sP4Mmode", "        "}, // MIDI mode [mpe/normal/mono]
  {"sP4Mnote", "        "}, // MIDI note mode [default/tuning/button]
  {"sP4jack2", "        "}, // jack2 mode [midi/pedal/linein]
  {"iP4tunin", "        "}, // load tuning [0-8]
  {"fP4Toff ", "        "}, // additional tuning offset in cent
  {"sP4flip ", "        "}, // Flip layout 180 degrees [no/yes]
  {"iP4Mpres", "        "}, // MIDI CC for key pressure. Special values 127 = Disable, 120 = Polyphonic Pressure, 121 = Channel Pressure
  {"iP4Mx   ", "        "}, // MIDI CC for key X movement. Special values 127 = Disable, 122 = Pitch Bend
  {"iP4My   ", "        "}, // MIDI CC for key Y movement. Special values 127 = Disable
  {"iP4Mvelo", "        "}, // MIDI continuous velocity. Special values 127 = Disable, else send continuous velocity on CC73 and continuous release velocity on CC72
  {"iP4voice", "        "}, // Voice count/MPE channel count [1-15]
  {"iP4MChan", "        "}, // MIDI channel [1-16]
  {"iP4MPEpb", "        "}, // MPE pitch bend range [12/24/48/96]
  {"fP4thres", "        "}, // Key sensitivity threshold [0-1]
  {"fP4bendS", "        "}, // Pitch bend range in semitones [-4.0-4.0]
  {"fP4presS", "        "}, // Key pressure factor [0-4]
  {"fP4veloS", "        "}, // Key velocity factor [0-4]
  {"fP4tiltS", "        "}, // Key tilt factor [-4.0-4.0]
  {"iP4veloO", "        "}, // MIDI velocity offset [0-127]
  {"fP4volum", "        "}, // Volume [0-127]
  {"fP4decay", "        "}, // Decay, how long it takes for the sound to decay after atack or release [0-127]
  {"fP4decaP", "        "}, // Pedal decay, decay value when sustain pedal is pressed [0-127]

  // preset 5
  {"sP5name ", "preset5 "},
  {"hP5color", "#003838 "},
  {"iP5Mpgm ", "        "}, // MIDI program change [0-127]
  {"iP5Mint ", "        "}, // MIDI message interval in ms [1-127]
  {"iP5Mmint", "        "}, // MIDI motion sensor message interval. 0 = disable, 127 only internal, else x10ms [0-127]
  {"sP5Mmode", "        "}, // MIDI mode [mpe/normal/mono]
  {"sP5Mnote", "        "}, // MIDI note mode [default/tuning/button]
  {"sP5jack2", "        "}, // jack2 mode [midi/pedal/linein]
  {"iP5tunin", "        "}, // load tuning [0-8]
  {"fP5Toff ", "        "}, // additional tuning offset in cent
  {"sP5flip ", "        "}, // Flip layout 180 degrees [no/yes]
  {"iP5Mpres", "        "}, // MIDI CC for key pressure. Special values 127 = Disable, 120 = Polyphonic Pressure, 121 = Channel Pressure
  {"iP5Mx   ", "        "}, // MIDI CC for key X movement. Special values 127 = Disable, 122 = Pitch Bend
  {"iP5My   ", "        "}, // MIDI CC for key Y movement. Special values 127 = Disable
  {"iP5Mvelo", "        "}, // MIDI continuous velocity. Special values 127 = Disable, else send continuous velocity on CC73 and continuous release velocity on CC72
  {"iP5voice", "        "}, // Voice count/MPE channel count [1-15]
  {"iP5MChan", "        "}, // MIDI channel [1-16]
  {"iP5MPEpb", "        "}, // MPE pitch bend range [12/24/48/96]
  {"fP5thres", "        "}, // Key sensitivity threshold [0-1]
  {"fP5bendS", "        "}, // Pitch bend range in semitones [-4.0-4.0]
  {"fP5presS", "        "}, // Key pressure factor [0-4]
  {"fP5veloS", "        "}, // Key velocity factor [0-4]
  {"fP5tiltS", "        "}, // Key tilt factor [-4.0-4.0]
  {"iP5veloO", "        "}, // MIDI velocity offset [0-127]
  {"fP5volum", "        "}, // Volume [0-127]
  {"fP5decay", "        "}, // Decay, how long it takes for the sound to decay after atack or release [0-127]
  {"fP5decaP", "        "}, // Pedal decay, decay value when sustain pedal is pressed [0-127]

  // preset 6
  {"sP6name ", "preset6 "},
  {"hP6color", "#000ea8 "},
  {"iP6Mpgm ", "        "}, // MIDI program change [0-127]
  {"iP6Mint ", "        "}, // MIDI message interval in ms [1-127]
  {"iP6Mmint", "        "}, // MIDI motion sensor message interval. 0 = disable, 127 only internal, else x10ms [0-127]
  {"sP6Mmode", "        "}, // MIDI mode [mpe/normal/mono]
  {"sP6Mnote", "        "}, // MIDI note mode [default/tuning/button]
  {"sP6jack2", "        "}, // jack2 mode [midi/pedal/linein]
  {"iP6tunin", "        "}, // load tuning [0-8]
  {"fP6Toff ", "        "}, // additional tuning offset in cent
  {"sP6flip ", "        "}, // Flip layout 180 degrees [no/yes]
  {"iP6Mpres", "        "}, // MIDI CC for key pressure. Special values 127 = Disable, 120 = Polyphonic Pressure, 121 = Channel Pressure
  {"iP6Mx   ", "        "}, // MIDI CC for key X movement. Special values 127 = Disable, 122 = Pitch Bend
  {"iP6My   ", "        "}, // MIDI CC for key Y movement. Special values 127 = Disable
  {"iP6Mvelo", "        "}, // MIDI continuous velocity. Special values 127 = Disable, else send continuous velocity on CC73 and continuous release velocity on CC72
  {"iP6voice", "        "}, // Voice count/MPE channel count [1-15]
  {"iP6MChan", "        "}, // MIDI channel [1-16]
  {"iP6MPEpb", "        "}, // MPE pitch bend range [12/24/48/96]
  {"fP6thres", "        "}, // Key sensitivity threshold [0-1]
  {"fP6bendS", "        "}, // Pitch bend range in semitones [-4.0-4.0]
  {"fP6presS", "        "}, // Key pressure factor [0-4]
  {"fP6veloS", "        "}, // Key velocity factor [0-4]
  {"fP6tiltS", "        "}, // Key tilt factor [-4.0-4.0]
  {"iP6veloO", "        "}, // MIDI velocity offset [0-127]
  {"fP6volum", "        "}, // Volume [0-127]
  {"fP6decay", "        "}, // Decay, how long it takes for the sound to decay after atack or release [0-127]
  {"fP6decaP", "        "}, // Pedal decay, decay value when sustain pedal is pressed [0-127]

  // preset 7
  {"sP7name ", "preset7 "},
  {"hP7color", "#300064 "},
  {"iP7Mpgm ", "        "}, // MIDI program change [0-127]
  {"iP7Mint ", "        "}, // MIDI message interval in ms [1-127]
  {"iP7Mmint", "        "}, // MIDI motion sensor message interval. 0 = disable, 127 only internal, else x10ms [0-127]
  {"sP7Mmode", "        "}, // MIDI mode [mpe/normal/mono]
  {"sP7Mnote", "        "}, // MIDI note mode [default/tuning/button]
  {"sP7jack2", "        "}, // jack2 mode [midi/pedal/linein]
  {"iP7tunin", "        "}, // load tuning [0-8]
  {"fP7Toff ", "        "}, // additional tuning offset in cent
  {"sP7flip ", "        "}, // Flip layout 180 degrees [no/yes]
  {"iP7Mpres", "        "}, // MIDI CC for key pressure. Special values 127 = Disable, 120 = Polyphonic Pressure, 121 = Channel Pressure
  {"iP7Mx   ", "        "}, // MIDI CC for key X movement. Special values 127 = Disable, 122 = Pitch Bend
  {"iP7My   ", "        "}, // MIDI CC for key Y movement. Special values 127 = Disable
  {"iP7Mvelo", "        "}, // MIDI continuous velocity. Special values 127 = Disable, else send continuous velocity on CC73 and continuous release velocity on CC72
  {"iP7voice", "        "}, // Voice count/MPE channel count [1-15]
  {"iP7MChan", "        "}, // MIDI channel [1-16]
  {"iP7MPEpb", "        "}, // MPE pitch bend range [12/24/48/96]
  {"fP7thres", "        "}, // Key sensitivity threshold [0-1]
  {"fP7bendS", "        "}, // Pitch bend range in semitones [-4.0-4.0]
  {"fP7presS", "        "}, // Key pressure factor [0-4]
  {"fP7veloS", "        "}, // Key velocity factor [0-4]
  {"fP7tiltS", "        "}, // Key tilt factor [-4.0-4.0]
  {"iP7veloO", "        "}, // MIDI velocity offset [0-127]
  {"fP7volum", "        "}, // Volume [0-127]
  {"fP7decay", "        "}, // Decay, how long it takes for the sound to decay after atack or release [0-127]
  {"fP7decaP", "        "}, // Pedal decay, decay value when sustain pedal is pressed [0-127]

  // preset 8
  {"sP8name ", "preset8 "},
  {"hP8color", "#2a2a2a "},
  {"iP8Mpgm ", "        "}, // MIDI program change [0-127]
  {"iP8Mint ", "        "}, // MIDI message interval in ms [1-127]
  {"iP8Mmint", "        "}, // MIDI motion sensor message interval. 0 = disable, 127 only internal, else x10ms [0-127]
  {"sP8Mmode", "        "}, // MIDI mode [mpe/normal/mono]
  {"sP8Mnote", "        "}, // MIDI note mode [default/tuning/button]
  {"sP8jack2", "        "}, // jack2 mode [midi/pedal/linein]
  {"iP8tunin", "        "}, // load tuning [0-8]
  {"fP8Toff ", "        "}, // additional tuning offset in cent
  {"sP8flip ", "        "}, // Flip layout 180 degrees [no/yes]
  {"iP8Mpres", "        "}, // MIDI CC for key pressure. Special values 127 = Disable, 120 = Polyphonic Pressure, 121 = Channel Pressure
  {"iP8Mx   ", "        "}, // MIDI CC for key X movement. Special values 127 = Disable, 122 = Pitch Bend
  {"iP8My   ", "        "}, // MIDI CC for key Y movement. Special values 127 = Disable
  {"iP8Mvelo", "        "}, // MIDI continuous velocity. Special values 127 = Disable, else send continuous velocity on CC73 and continuous release velocity on CC72
  {"iP8voice", "        "}, // Voice count/MPE channel count [1-15]
  {"iP8MChan", "        "}, // MIDI channel [1-16]
  {"iP8MPEpb", "        "}, // MPE pitch bend range [12/24/48/96]
  {"fP8thres", "        "}, // Key sensitivity threshold [0-1]
  {"fP8bendS", "        "}, // Pitch bend range in semitones [-4.0-4.0]
  {"fP8presS", "        "}, // Key pressure factor [0-4]
  {"fP8veloS", "        "}, // Key velocity factor [0-4]
  {"fP8tiltS", "        "}, // Key tilt factor [-4.0-4.0]
  {"iP8veloO", "        "}, // MIDI velocity offset [0-127]
  {"fP8volum", "        "}, // Volume [0-127]
  {"fP8decay", "        "}, // Decay, how long it takes for the sound to decay after atack or release [0-127]
  {"fP8decaP", "        "}, // Pedal decay, decay value when sustain pedal is pressed [0-127]

  // tuning 0, 12tet
  // don't use, hard coded as default
  // {"sT0name ", "12tet   "},
  // {"hT0color", "#00aa00 "},
  // {"fT0off  ", "0.0     "},
  // {"fT0oct  ", "1200.0  "},
  // {"fT0fifth", "700     "},
  // {"fT0F_1  ", "0.0     "},
  // {"fT0Gb1  ", "0.0     "},
  // {"fT0F#1  ", "0.0     "},
  // {"fT0G_1  ", "0.0     "},
  // {"fT0Ab1  ", "0.0     "},
  // {"fT0G#1  ", "0.0     "},
  // {"fT0A_1  ", "0.0     "},
  // {"fT0Bb1  ", "0.0     "},
  // {"fT0A#1  ", "0.0     "},
  // {"fT0B_1  ", "0.0     "},
  // {"fT0C_2  ", "0.0     "},
  // {"fT0Db2  ", "0.0     "},
  // {"fT0C#2  ", "0.0     "},
  // {"fT0D_2  ", "0.0     "},
  // {"fT0Eb2  ", "0.0     "},
  // {"fT0D#2  ", "0.0     "},
  // {"fT0E_2  ", "0.0     "},
  // {"fT0F_2  ", "0.0     "},
  // {"fT0Gb2  ", "0.0     "},
  // {"fT0F#2  ", "0.0     "},
  // {"fT0G_2  ", "0.0     "},
  // {"fT0Ab2  ", "0.0     "},
  // {"fT0G#2  ", "0.0     "},
  // {"fT0A_2  ", "0.0     "},
  // {"fT0Bb2  ", "0.0     "},
  // {"fT0A#2  ", "0.0     "},
  // {"fT0B_2  ", "0.0     "},
  // {"fT0C_3  ", "0.0     "},
  // {"fT0Db3  ", "0.0     "},
  // {"fT0C#3  ", "0.0     "},
  // {"fT0D_3  ", "0.0     "},
  // {"fT0Eb3  ", "0.0     "},
  // {"fT0D#3  ", "0.0     "},
  // {"fT0E_3  ", "0.0     "},
  // {"fT0F_3  ", "0.0     "},
  // {"fT0Gb3  ", "0.0     "},
  // {"fT0F#3  ", "0.0     "},
  // {"fT0G_3  ", "0.0     "},
  // {"fT0Ab3  ", "0.0     "},
  // {"fT0G#3  ", "0.0     "},
  // {"fT0A_3  ", "0.0     "},
  // {"fT0Bb3  ", "0.0     "},
  // {"fT0A#3  ", "0.0     "},
  // {"fT0B_3  ", "0.0     "},
  // {"fT0C_4  ", "0.0     "},
  // {"fT0Db4  ", "0.0     "},
  // {"fT0C#4  ", "0.0     "},
  // {"fT0D_4  ", "0.0     "},
  // {"fT0Eb4  ", "0.0     "},
  // {"fT0D#4  ", "0.0     "},
  // {"fT0E_4  ", "0.0     "},
  // {"fT0F_4  ", "0.0     "},
  // {"fT0Gb4  ", "0.0     "},
  // {"fT0F#4  ", "0.0     "},
  // {"fT0G_4  ", "0.0     "},
  // {"fT0Ab4  ", "0.0     "},
  // {"fT0G#4  ", "0.0     "},
  // {"fT0A_4  ", "0.0     "},
  // {"fT0Bb4  ", "0.0     "},
  // {"fT0A#4  ", "0.0     "},
  // {"fT0B_4  ", "0.0     "},

  // tuning 1, 31tet (very close to quarter comma meantone tuning 696.5784)
  {"sT1name ", "31tet   "},
  {"hT1color", "#005555 "},
  {"fT1off  ", "0.0     "},
  {"fT1oct  ", "1200.0  "},
  {"fT1fifth", "696.7742"}, // 1200/31*18
  {"fT1F_1  ", "0.0     "},
  {"fT1Gb1  ", "0.0     "},
  {"fT1F#1  ", "0.0     "},
  {"fT1G_1  ", "0.0     "},
  {"fT1Ab1  ", "0.0     "},
  {"fT1G#1  ", "0.0     "},
  {"fT1A_1  ", "0.0     "},
  {"fT1Bb1  ", "0.0     "},
  {"fT1A#1  ", "0.0     "},
  {"fT1B_1  ", "0.0     "},
  {"fT1C_2  ", "0.0     "},
  {"fT1Db2  ", "0.0     "},
  {"fT1C#2  ", "0.0     "},
  {"fT1D_2  ", "0.0     "},
  {"fT1Eb2  ", "0.0     "},
  {"fT1D#2  ", "0.0     "},
  {"fT1E_2  ", "0.0     "},
  {"fT1F_2  ", "0.0     "},
  {"fT1Gb2  ", "0.0     "},
  {"fT1F#2  ", "0.0     "},
  {"fT1G_2  ", "0.0     "},
  {"fT1Ab2  ", "0.0     "},
  {"fT1G#2  ", "0.0     "},
  {"fT1A_2  ", "0.0     "},
  {"fT1Bb2  ", "0.0     "},
  {"fT1A#2  ", "0.0     "},
  {"fT1B_2  ", "0.0     "},
  {"fT1C_3  ", "0.0     "},
  {"fT1Db3  ", "0.0     "},
  {"fT1C#3  ", "0.0     "},
  {"fT1D_3  ", "0.0     "},
  {"fT1Eb3  ", "0.0     "},
  {"fT1D#3  ", "0.0     "},
  {"fT1E_3  ", "0.0     "},
  {"fT1F_3  ", "0.0     "},
  {"fT1Gb3  ", "0.0     "},
  {"fT1F#3  ", "0.0     "},
  {"fT1G_3  ", "0.0     "},
  {"fT1Ab3  ", "0.0     "},
  {"fT1G#3  ", "0.0     "},
  {"fT1A_3  ", "0.0     "},
  {"fT1Bb3  ", "0.0     "},
  {"fT1A#3  ", "0.0     "},
  {"fT1B_3  ", "0.0     "},
  {"fT1C_4  ", "0.0     "},
  {"fT1Db4  ", "0.0     "},
  {"fT1C#4  ", "0.0     "},
  {"fT1D_4  ", "0.0     "},
  {"fT1Eb4  ", "0.0     "},
  {"fT1D#4  ", "0.0     "},
  {"fT1E_4  ", "0.0     "},
  {"fT1F_4  ", "0.0     "},
  {"fT1Gb4  ", "0.0     "},
  {"fT1F#4  ", "0.0     "},
  {"fT1G_4  ", "0.0     "},
  {"fT1Ab4  ", "0.0     "},
  {"fT1G#4  ", "0.0     "},
  {"fT1A_4  ", "0.0     "},
  {"fT1Bb4  ", "0.0     "},
  {"fT1A#4  ", "0.0     "},
  {"fT1B_4  ", "0.0     "},

  // tuning 2, 19tet
  {"sT2name ", "19tet   "},
  {"hT2color", "#0000aa "},
  {"fT2off  ", "0.0     "},
  {"fT2oct  ", "1200.0  "},
  {"fT2fifth", "694.7368"}, // 1200/19*11
  {"fT2F_1  ", "0.0     "},
  {"fT2Gb1  ", "0.0     "},
  {"fT2F#1  ", "0.0     "},
  {"fT2G_1  ", "0.0     "},
  {"fT2Ab1  ", "0.0     "},
  {"fT2G#1  ", "0.0     "},
  {"fT2A_1  ", "0.0     "},
  {"fT2Bb1  ", "0.0     "},
  {"fT2A#1  ", "0.0     "},
  {"fT2B_1  ", "0.0     "},
  {"fT2C_2  ", "0.0     "},
  {"fT2Db2  ", "0.0     "},
  {"fT2C#2  ", "0.0     "},
  {"fT2D_2  ", "0.0     "},
  {"fT2Eb2  ", "0.0     "},
  {"fT2D#2  ", "0.0     "},
  {"fT2E_2  ", "0.0     "},
  {"fT2F_2  ", "0.0     "},
  {"fT2Gb2  ", "0.0     "},
  {"fT2F#2  ", "0.0     "},
  {"fT2G_2  ", "0.0     "},
  {"fT2Ab2  ", "0.0     "},
  {"fT2G#2  ", "0.0     "},
  {"fT2A_2  ", "0.0     "},
  {"fT2Bb2  ", "0.0     "},
  {"fT2A#2  ", "0.0     "},
  {"fT2B_2  ", "0.0     "},
  {"fT2C_3  ", "0.0     "},
  {"fT2Db3  ", "0.0     "},
  {"fT2C#3  ", "0.0     "},
  {"fT2D_3  ", "0.0     "},
  {"fT2Eb3  ", "0.0     "},
  {"fT2D#3  ", "0.0     "},
  {"fT2E_3  ", "0.0     "},
  {"fT2F_3  ", "0.0     "},
  {"fT2Gb3  ", "0.0     "},
  {"fT2F#3  ", "0.0     "},
  {"fT2G_3  ", "0.0     "},
  {"fT2Ab3  ", "0.0     "},
  {"fT2G#3  ", "0.0     "},
  {"fT2A_3  ", "0.0     "},
  {"fT2Bb3  ", "0.0     "},
  {"fT2A#3  ", "0.0     "},
  {"fT2B_3  ", "0.0     "},
  {"fT2C_4  ", "0.0     "},
  {"fT2Db4  ", "0.0     "},
  {"fT2C#4  ", "0.0     "},
  {"fT2D_4  ", "0.0     "},
  {"fT2Eb4  ", "0.0     "},
  {"fT2D#4  ", "0.0     "},
  {"fT2E_4  ", "0.0     "},
  {"fT2F_4  ", "0.0     "},
  {"fT2Gb4  ", "0.0     "},
  {"fT2F#4  ", "0.0     "},
  {"fT2G_4  ", "0.0     "},
  {"fT2Ab4  ", "0.0     "},
  {"fT2G#4  ", "0.0     "},
  {"fT2A_4  ", "0.0     "},
  {"fT2Bb4  ", "0.0     "},
  {"fT2A#4  ", "0.0     "},
  {"fT2B_4  ", "0.0     "},

  // tuning 3, pythagorean (pure fifth)
  {"sT3name ", "pythagor"},
  {"hT3color", "#555500 "},
  {"fT3off  ", "0.0     "},
  {"fT3oct  ", "1200.0  "},
  {"fT3fifth", "701.9550"}, // 1200*log2(3/2)
  {"fT3F_1  ", "0.0     "},
  {"fT3Gb1  ", "0.0     "},
  {"fT3F#1  ", "0.0     "},
  {"fT3G_1  ", "0.0     "},
  {"fT3Ab1  ", "0.0     "},
  {"fT3G#1  ", "0.0     "},
  {"fT3A_1  ", "0.0     "},
  {"fT3Bb1  ", "0.0     "},
  {"fT3A#1  ", "0.0     "},
  {"fT3B_1  ", "0.0     "},
  {"fT3C_2  ", "0.0     "},
  {"fT3Db2  ", "0.0     "},
  {"fT3C#2  ", "0.0     "},
  {"fT3D_2  ", "0.0     "},
  {"fT3Eb2  ", "0.0     "},
  {"fT3D#2  ", "0.0     "},
  {"fT3E_2  ", "0.0     "},
  {"fT3F_2  ", "0.0     "},
  {"fT3Gb2  ", "0.0     "},
  {"fT3F#2  ", "0.0     "},
  {"fT3G_2  ", "0.0     "},
  {"fT3Ab2  ", "0.0     "},
  {"fT3G#2  ", "0.0     "},
  {"fT3A_2  ", "0.0     "},
  {"fT3Bb2  ", "0.0     "},
  {"fT3A#2  ", "0.0     "},
  {"fT3B_2  ", "0.0     "},
  {"fT3C_3  ", "0.0     "},
  {"fT3Db3  ", "0.0     "},
  {"fT3C#3  ", "0.0     "},
  {"fT3D_3  ", "0.0     "},
  {"fT3Eb3  ", "0.0     "},
  {"fT3D#3  ", "0.0     "},
  {"fT3E_3  ", "0.0     "},
  {"fT3F_3  ", "0.0     "},
  {"fT3Gb3  ", "0.0     "},
  {"fT3F#3  ", "0.0     "},
  {"fT3G_3  ", "0.0     "},
  {"fT3Ab3  ", "0.0     "},
  {"fT3G#3  ", "0.0     "},
  {"fT3A_3  ", "0.0     "},
  {"fT3Bb3  ", "0.0     "},
  {"fT3A#3  ", "0.0     "},
  {"fT3B_3  ", "0.0     "},
  {"fT3C_4  ", "0.0     "},
  {"fT3Db4  ", "0.0     "},
  {"fT3C#4  ", "0.0     "},
  {"fT3D_4  ", "0.0     "},
  {"fT3Eb4  ", "0.0     "},
  {"fT3D#4  ", "0.0     "},
  {"fT3E_4  ", "0.0     "},
  {"fT3F_4  ", "0.0     "},
  {"fT3Gb4  ", "0.0     "},
  {"fT3F#4  ", "0.0     "},
  {"fT3G_4  ", "0.0     "},
  {"fT3Ab4  ", "0.0     "},
  {"fT3G#4  ", "0.0     "},
  {"fT3A_4  ", "0.0     "},
  {"fT3Bb4  ", "0.0     "},
  {"fT3A#4  ", "0.0     "},
  {"fT3B_4  ", "0.0     "},

  // tuning 4, 5tet
  {"sT4name ", "5tet    "},
  {"hT4color", "#aa0000 "},
  {"fT4off  ", "0.0     "},
  {"fT4oct  ", "1200.0  "},
  {"fT4fifth", "720.0   "}, // 1200/5*3
  {"fT4F_1  ", "0.0     "},
  {"fT4Gb1  ", "0.0     "},
  {"fT4F#1  ", "0.0     "},
  {"fT4G_1  ", "0.0     "},
  {"fT4Ab1  ", "0.0     "},
  {"fT4G#1  ", "0.0     "},
  {"fT4A_1  ", "0.0     "},
  {"fT4Bb1  ", "0.0     "},
  {"fT4A#1  ", "0.0     "},
  {"fT4B_1  ", "0.0     "},
  {"fT4C_2  ", "0.0     "},
  {"fT4Db2  ", "0.0     "},
  {"fT4C#2  ", "0.0     "},
  {"fT4D_2  ", "0.0     "},
  {"fT4Eb2  ", "0.0     "},
  {"fT4D#2  ", "0.0     "},
  {"fT4E_2  ", "0.0     "},
  {"fT4F_2  ", "0.0     "},
  {"fT4Gb2  ", "0.0     "},
  {"fT4F#2  ", "0.0     "},
  {"fT4G_2  ", "0.0     "},
  {"fT4Ab2  ", "0.0     "},
  {"fT4G#2  ", "0.0     "},
  {"fT4A_2  ", "0.0     "},
  {"fT4Bb2  ", "0.0     "},
  {"fT4A#2  ", "0.0     "},
  {"fT4B_2  ", "0.0     "},
  {"fT4C_3  ", "0.0     "},
  {"fT4Db3  ", "0.0     "},
  {"fT4C#3  ", "0.0     "},
  {"fT4D_3  ", "0.0     "},
  {"fT4Eb3  ", "0.0     "},
  {"fT4D#3  ", "0.0     "},
  {"fT4E_3  ", "0.0     "},
  {"fT4F_3  ", "0.0     "},
  {"fT4Gb3  ", "0.0     "},
  {"fT4F#3  ", "0.0     "},
  {"fT4G_3  ", "0.0     "},
  {"fT4Ab3  ", "0.0     "},
  {"fT4G#3  ", "0.0     "},
  {"fT4A_3  ", "0.0     "},
  {"fT4Bb3  ", "0.0     "},
  {"fT4A#3  ", "0.0     "},
  {"fT4B_3  ", "0.0     "},
  {"fT4C_4  ", "0.0     "},
  {"fT4Db4  ", "0.0     "},
  {"fT4C#4  ", "0.0     "},
  {"fT4D_4  ", "0.0     "},
  {"fT4Eb4  ", "0.0     "},
  {"fT4D#4  ", "0.0     "},
  {"fT4E_4  ", "0.0     "},
  {"fT4F_4  ", "0.0     "},
  {"fT4Gb4  ", "0.0     "},
  {"fT4F#4  ", "0.0     "},
  {"fT4G_4  ", "0.0     "},
  {"fT4Ab4  ", "0.0     "},
  {"fT4G#4  ", "0.0     "},
  {"fT4A_4  ", "0.0     "},
  {"fT4Bb4  ", "0.0     "},
  {"fT4A#4  ", "0.0     "},
  {"fT4B_4  ", "0.0     "},

  // tuning 5, 7tet
  {"sT5name ", "7tet    "},
  {"hT5color", "#550055 "},
  {"fT5off  ", "0.0     "},
  {"fT5oct  ", "1200.0  "},
  {"fT5fifth", "685.7142"}, // 1200/7*4
  {"fT5F_1  ", "0.0     "},
  {"fT5Gb1  ", "0.0     "},
  {"fT5F#1  ", "0.0     "},
  {"fT5G_1  ", "0.0     "},
  {"fT5Ab1  ", "0.0     "},
  {"fT5G#1  ", "0.0     "},
  {"fT5A_1  ", "0.0     "},
  {"fT5Bb1  ", "0.0     "},
  {"fT5A#1  ", "0.0     "},
  {"fT5B_1  ", "0.0     "},
  {"fT5C_2  ", "0.0     "},
  {"fT5Db2  ", "0.0     "},
  {"fT5C#2  ", "0.0     "},
  {"fT5D_2  ", "0.0     "},
  {"fT5Eb2  ", "0.0     "},
  {"fT5D#2  ", "0.0     "},
  {"fT5E_2  ", "0.0     "},
  {"fT5F_2  ", "0.0     "},
  {"fT5Gb2  ", "0.0     "},
  {"fT5F#2  ", "0.0     "},
  {"fT5G_2  ", "0.0     "},
  {"fT5Ab2  ", "0.0     "},
  {"fT5G#2  ", "0.0     "},
  {"fT5A_2  ", "0.0     "},
  {"fT5Bb2  ", "0.0     "},
  {"fT5A#2  ", "0.0     "},
  {"fT5B_2  ", "0.0     "},
  {"fT5C_3  ", "0.0     "},
  {"fT5Db3  ", "0.0     "},
  {"fT5C#3  ", "0.0     "},
  {"fT5D_3  ", "0.0     "},
  {"fT5Eb3  ", "0.0     "},
  {"fT5D#3  ", "0.0     "},
  {"fT5E_3  ", "0.0     "},
  {"fT5F_3  ", "0.0     "},
  {"fT5Gb3  ", "0.0     "},
  {"fT5F#3  ", "0.0     "},
  {"fT5G_3  ", "0.0     "},
  {"fT5Ab3  ", "0.0     "},
  {"fT5G#3  ", "0.0     "},
  {"fT5A_3  ", "0.0     "},
  {"fT5Bb3  ", "0.0     "},
  {"fT5A#3  ", "0.0     "},
  {"fT5B_3  ", "0.0     "},
  {"fT5C_4  ", "0.0     "},
  {"fT5Db4  ", "0.0     "},
  {"fT5C#4  ", "0.0     "},
  {"fT5D_4  ", "0.0     "},
  {"fT5Eb4  ", "0.0     "},
  {"fT5D#4  ", "0.0     "},
  {"fT5E_4  ", "0.0     "},
  {"fT5F_4  ", "0.0     "},
  {"fT5Gb4  ", "0.0     "},
  {"fT5F#4  ", "0.0     "},
  {"fT5G_4  ", "0.0     "},
  {"fT5Ab4  ", "0.0     "},
  {"fT5G#4  ", "0.0     "},
  {"fT5A_4  ", "0.0     "},
  {"fT5Bb4  ", "0.0     "},
  {"fT5A#4  ", "0.0     "},
  {"fT5B_4  ", "0.0     "},

  // tuning 6, Bohlen-Pierce tuning thirds 13et
  {"sT6name ", "BP 13tet"},
  {"hT6color", "#55aa00 "},
  {"fT6off  ", "0.0     "},
  {"fT6oct  ", "1024.130"}, // 1200*log2(3)/13*7
  {"fT6fifth", "585.2169"}, // 1200*log2(3)/13*4
  {"fT6F_1  ", "0.0     "},
  {"fT6Gb1  ", "0.0     "},
  {"fT6F#1  ", "0.0     "},
  {"fT6G_1  ", "0.0     "},
  {"fT6Ab1  ", "0.0     "},
  {"fT6G#1  ", "0.0     "},
  {"fT6A_1  ", "0.0     "},
  {"fT6Bb1  ", "0.0     "},
  {"fT6A#1  ", "0.0     "},
  {"fT6B_1  ", "0.0     "},
  {"fT6C_2  ", "0.0     "},
  {"fT6Db2  ", "0.0     "},
  {"fT6C#2  ", "0.0     "},
  {"fT6D_2  ", "0.0     "},
  {"fT6Eb2  ", "0.0     "},
  {"fT6D#2  ", "0.0     "},
  {"fT6E_2  ", "0.0     "},
  {"fT6F_2  ", "0.0     "},
  {"fT6Gb2  ", "0.0     "},
  {"fT6F#2  ", "0.0     "},
  {"fT6G_2  ", "0.0     "},
  {"fT6Ab2  ", "0.0     "},
  {"fT6G#2  ", "0.0     "},
  {"fT6A_2  ", "0.0     "},
  {"fT6Bb2  ", "0.0     "},
  {"fT6A#2  ", "0.0     "},
  {"fT6B_2  ", "0.0     "},
  {"fT6C_3  ", "0.0     "},
  {"fT6Db3  ", "0.0     "},
  {"fT6C#3  ", "0.0     "},
  {"fT6D_3  ", "0.0     "},
  {"fT6Eb3  ", "0.0     "},
  {"fT6D#3  ", "0.0     "},
  {"fT6E_3  ", "0.0     "},
  {"fT6F_3  ", "0.0     "},
  {"fT6Gb3  ", "0.0     "},
  {"fT6F#3  ", "0.0     "},
  {"fT6G_3  ", "0.0     "},
  {"fT6Ab3  ", "0.0     "},
  {"fT6G#3  ", "0.0     "},
  {"fT6A_3  ", "0.0     "},
  {"fT6Bb3  ", "0.0     "},
  {"fT6A#3  ", "0.0     "},
  {"fT6B_3  ", "0.0     "},
  {"fT6C_4  ", "0.0     "},
  {"fT6Db4  ", "0.0     "},
  {"fT6C#4  ", "0.0     "},
  {"fT6D_4  ", "0.0     "},
  {"fT6Eb4  ", "0.0     "},
  {"fT6D#4  ", "0.0     "},
  {"fT6E_4  ", "0.0     "},
  {"fT6F_4  ", "0.0     "},
  {"fT6Gb4  ", "0.0     "},
  {"fT6F#4  ", "0.0     "},
  {"fT6G_4  ", "0.0     "},
  {"fT6Ab4  ", "0.0     "},
  {"fT6G#4  ", "0.0     "},
  {"fT6A_4  ", "0.0     "},
  {"fT6Bb4  ", "0.0     "},
  {"fT6A#4  ", "0.0     "},
  {"fT6B_4  ", "0.0     "},

  // tuning 7, Bohlen-Pierce tuning sixts 13tet
  {"sT7name ", "BP 13tet"},
  {"hT7color", "#55aa00 "},
  {"fT7off  ", "0.0     "},
  {"fT7oct  ", "1901.955"}, // 1200*log2(3)
  {"fT7fifth", "1024.130"}, // 1200*log2(3)/13*7
  {"fT7F_1  ", "0.0     "},
  {"fT7Gb1  ", "0.0     "},
  {"fT7F#1  ", "0.0     "},
  {"fT7G_1  ", "0.0     "},
  {"fT7Ab1  ", "0.0     "},
  {"fT7G#1  ", "0.0     "},
  {"fT7A_1  ", "0.0     "},
  {"fT7Bb1  ", "0.0     "},
  {"fT7A#1  ", "0.0     "},
  {"fT7B_1  ", "0.0     "},
  {"fT7C_2  ", "0.0     "},
  {"fT7Db2  ", "0.0     "},
  {"fT7C#2  ", "0.0     "},
  {"fT7D_2  ", "0.0     "},
  {"fT7Eb2  ", "0.0     "},
  {"fT7D#2  ", "0.0     "},
  {"fT7E_2  ", "0.0     "},
  {"fT7F_2  ", "0.0     "},
  {"fT7Gb2  ", "0.0     "},
  {"fT7F#2  ", "0.0     "},
  {"fT7G_2  ", "0.0     "},
  {"fT7Ab2  ", "0.0     "},
  {"fT7G#2  ", "0.0     "},
  {"fT7A_2  ", "0.0     "},
  {"fT7Bb2  ", "0.0     "},
  {"fT7A#2  ", "0.0     "},
  {"fT7B_2  ", "0.0     "},
  {"fT7C_3  ", "0.0     "},
  {"fT7Db3  ", "0.0     "},
  {"fT7C#3  ", "0.0     "},
  {"fT7D_3  ", "0.0     "},
  {"fT7Eb3  ", "0.0     "},
  {"fT7D#3  ", "0.0     "},
  {"fT7E_3  ", "0.0     "},
  {"fT7F_3  ", "0.0     "},
  {"fT7Gb3  ", "0.0     "},
  {"fT7F#3  ", "0.0     "},
  {"fT7G_3  ", "0.0     "},
  {"fT7Ab3  ", "0.0     "},
  {"fT7G#3  ", "0.0     "},
  {"fT7A_3  ", "0.0     "},
  {"fT7Bb3  ", "0.0     "},
  {"fT7A#3  ", "0.0     "},
  {"fT7B_3  ", "0.0     "},
  {"fT7C_4  ", "0.0     "},
  {"fT7Db4  ", "0.0     "},
  {"fT7C#4  ", "0.0     "},
  {"fT7D_4  ", "0.0     "},
  {"fT7Eb4  ", "0.0     "},
  {"fT7D#4  ", "0.0     "},
  {"fT7E_4  ", "0.0     "},
  {"fT7F_4  ", "0.0     "},
  {"fT7Gb4  ", "0.0     "},
  {"fT7F#4  ", "0.0     "},
  {"fT7G_4  ", "0.0     "},
  {"fT7Ab4  ", "0.0     "},
  {"fT7G#4  ", "0.0     "},
  {"fT7A_4  ", "0.0     "},
  {"fT7Bb4  ", "0.0     "},
  {"fT7A#4  ", "0.0     "},
  {"fT7B_4  ", "0.0     "},

  // tuning 8, JI 7-limit
  {"sT8name ", "7limitJI"},
  {"hT8color", "#00aa55 "},
  {"fT8off  ", "0.0     "},
  {"fT8oct  ", "1200.0  "},
  {"fT8fifth", "696.7741"},
  {"fT8F_1  ", "-5.18080"},
  {"fT8Gb1  ", "-1.86703"},
  {"fT8F#1  ", "+1.86703"},
  {"fT8G_1  ", "+5.18080"},
  {"fT8Ab1  ", "+0.78306"},
  {"fT8G#1  ", "-1.56612"},
  {"fT8A_1  ", "-5.96386"},
  {"fT8Bb1  ", "-10.3616"},
  {"fT8A#1  ", "+1.08397"},
  {"fT8B_1  ", "+4.39774"},
  {"fT8C_2  ", "0.0     "},
  {"fT8Db2  ", "-4.39774"},
  {"fT8C#2  ", "-6.74692"},
  {"fT8D_2  ", "+10.3616"},
  {"fT8Eb2  ", "+5.96386"},
  {"fT8D#2  ", "-4.09683"},
  {"fT8E_2  ", "-0.78306"},
  {"fT8F_2  ", "-5.18080"},
  {"fT8Gb2  ", "-1.86703"},
  {"fT8F#2  ", "+1.86703"},
  {"fT8G_2  ", "+5.18080"},
  {"fT8Ab2  ", "+0.78306"},
  {"fT8G#2  ", "-1.56612"},
  {"fT8A_2  ", "-5.96386"},
  {"fT8Bb2  ", "-10.3616"},
  {"fT8A#2  ", "+1.08397"},
  {"fT8B_2  ", "+4.39774"},
  {"fT8C_3  ", "0.0     "},
  {"fT8Db3  ", "-4.39774"},
  {"fT8C#3  ", "-6.74692"},
  {"fT8D_3  ", "+10.3616"},
  {"fT8Eb3  ", "+5.96386"},
  {"fT8D#3  ", "-4.09683"},
  {"fT8E_3  ", "-0.78306"},
  {"fT8F_3  ", "-5.18080"},
  {"fT8Gb3  ", "-1.86703"},
  {"fT8F#3  ", "+1.86703"},
  {"fT8G_3  ", "+5.18080"},
  {"fT8Ab3  ", "+0.78306"},
  {"fT8G#3  ", "-1.56612"},
  {"fT8A_3  ", "-5.96386"},
  {"fT8Bb3  ", "-10.3616"},
  {"fT8A#3  ", "+1.08397"},
  {"fT8B_3  ", "+4.39774"},
  {"fT8C_4  ", "0.0     "},
  {"fT8Db4  ", "-4.39774"},
  {"fT8C#4  ", "-6.74692"},
  {"fT8D_4  ", "+10.3616"},
  {"fT8Eb4  ", "+5.96386"},
  {"fT8D#4  ", "-4.09683"},
  {"fT8E_4  ", "-0.78306"},
  {"fT8F_4  ", "-5.18080"},
  {"fT8Gb4  ", "-1.86703"},
  {"fT8F#4  ", "+1.86703"},
  {"fT8G_4  ", "+5.18080"},
  {"fT8Ab4  ", "+0.78306"},
  {"fT8G#4  ", "-1.56612"},
  {"fT8A_4  ", "-5.96386"},
  {"fT8Bb4  ", "-10.3616"},
  {"fT8A#4  ", "+1.08397"},
  {"fT8B_4  ", "+4.39774"},

  // Calibration factor
  {"fCfacF_1", "1.0     "},
  {"fCfacGb1", "1.0     "},
  {"fCfacF#1", "1.0     "},
  {"fCfacG_1", "1.0     "},
  {"fCfacAb1", "1.0     "},
  {"fCfacG#1", "1.0     "},
  {"fCfacA_1", "1.0     "},
  {"fCfacBb1", "1.0     "},
  {"fCfacA#1", "1.0     "},
  {"fCfacB_1", "1.0     "},
  {"fCfacC_2", "1.0     "},
  {"fCfacDb2", "1.0     "},
  {"fCfacC#2", "1.0     "},
  {"fCfacD_2", "1.0     "},
  {"fCfacEb2", "1.0     "},
  {"fCfacD#2", "1.0     "},
  {"fCfacE_2", "1.0     "},
  {"fCfacF_2", "1.0     "},
  {"fCfacGb2", "1.0     "},
  {"fCfacF#2", "1.0     "},
  {"fCfacG_2", "1.0     "},
  {"fCfacAb2", "1.0     "},
  {"fCfacG#2", "1.0     "},
  {"fCfacA_2", "1.0     "},
  {"fCfacBb2", "1.0     "},
  {"fCfacA#2", "1.0     "},
  {"fCfacB_2", "1.0     "},
  {"fCfacC_3", "1.0     "},
  {"fCfacDb3", "1.0     "},
  {"fCfacC#3", "1.0     "},
  {"fCfacD_3", "1.0     "},
  {"fCfacEb3", "1.0     "},
  {"fCfacD#3", "1.0     "},
  {"fCfacE_3", "1.0     "},
  {"fCfacF_3", "1.0     "},
  {"fCfacGb3", "1.0     "},
  {"fCfacF#3", "1.0     "},
  {"fCfacG_3", "1.0     "},
  {"fCfacAb3", "1.0     "},
  {"fCfacG#3", "1.0     "},
  {"fCfacA_3", "1.0     "},
  {"fCfacBb3", "1.0     "},
  {"fCfacA#3", "1.0     "},
  {"fCfacB_3", "1.0     "},
  {"fCfacC_4", "1.0     "},
  {"fCfacDb4", "1.0     "},
  {"fCfacC#4", "1.0     "},
  {"fCfacD_4", "1.0     "},
  {"fCfacEb4", "1.0     "},
  {"fCfacD#4", "1.0     "},
  {"fCfacE_4", "1.0     "},
  {"fCfacF_4", "1.0     "},
  {"fCfacGb4", "1.0     "},
  {"fCfacF#4", "1.0     "},
  {"fCfacG_4", "1.0     "},
  {"fCfacAb4", "1.0     "},
  {"fCfacG#4", "1.0     "},
  {"fCfacA_4", "1.0     "},
  {"fCfacBb4", "1.0     "},
  {"fCfacA#4", "1.0     "},
  {"fCfacB_4", "1.0     "},
  // Calibration offset
  {"fCoffF_1", "0.0     "},
  {"fCoffGb1", "0.0     "},
  {"fCoffF#1", "0.0     "},
  {"fCoffG_1", "0.0     "},
  {"fCoffAb1", "0.0     "},
  {"fCoffG#1", "0.0     "},
  {"fCoffA_1", "0.0     "},
  {"fCoffBb1", "0.0     "},
  {"fCoffA#1", "0.0     "},
  {"fCoffB_1", "0.0     "},
  {"fCoffC_2", "0.0     "},
  {"fCoffDb2", "0.0     "},
  {"fCoffC#2", "0.0     "},
  {"fCoffD_2", "0.0     "},
  {"fCoffEb2", "0.0     "},
  {"fCoffD#2", "0.0     "},
  {"fCoffE_2", "0.0     "},
  {"fCoffF_2", "0.0     "},
  {"fCoffGb2", "0.0     "},
  {"fCoffF#2", "0.0     "},
  {"fCoffG_2", "0.0     "},
  {"fCoffAb2", "0.0     "},
  {"fCoffG#2", "0.0     "},
  {"fCoffA_2", "0.0     "},
  {"fCoffBb2", "0.0     "},
  {"fCoffA#2", "0.0     "},
  {"fCoffB_2", "0.0     "},
  {"fCoffC_3", "0.0     "},
  {"fCoffDb3", "0.0     "},
  {"fCoffC#3", "0.0     "},
  {"fCoffD_3", "0.0     "},
  {"fCoffEb3", "0.0     "},
  {"fCoffD#3", "0.0     "},
  {"fCoffE_3", "0.0     "},
  {"fCoffF_3", "0.0     "},
  {"fCoffGb3", "0.0     "},
  {"fCoffF#3", "0.0     "},
  {"fCoffG_3", "0.0     "},
  {"fCoffAb3", "0.0     "},
  {"fCoffG#3", "0.0     "},
  {"fCoffA_3", "0.0     "},
  {"fCoffBb3", "0.0     "},
  {"fCoffA#3", "0.0     "},
  {"fCoffB_3", "0.0     "},
  {"fCoffC_4", "0.0     "},
  {"fCoffDb4", "0.0     "},
  {"fCoffC#4", "0.0     "},
  {"fCoffD_4", "0.0     "},
  {"fCoffEb4", "0.0     "},
  {"fCoffD#4", "0.0     "},
  {"fCoffE_4", "0.0     "},
  {"fCoffF_4", "0.0     "},
  {"fCoffGb4", "0.0     "},
  {"fCoffF#4", "0.0     "},
  {"fCoffG_4", "0.0     "},
  {"fCoffAb4", "0.0     "},
  {"fCoffG#4", "0.0     "},
  {"fCoffA_4", "0.0     "},
  {"fCoffBb4", "0.0     "},
  {"fCoffA#4", "0.0     "},
  {"fCoffB_4", "0.0     "},

  {"MCfgEnd ", "        "},
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
