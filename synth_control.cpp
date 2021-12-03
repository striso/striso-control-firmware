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
#include <math.h>

#include "ch.h"
#include "hal.h"

extern "C" {
    #include "synth.h"
    #include "ws2812.h"
    #include "led.h"
}

#include "config.h"
#include "striso.h"
#include "midi_usb.h"
#include "midi_serial.h"
#include "midi.h"

#ifndef USE_WS2812
#define ws2812_write_led(n,r,g,b) led_rgb3(14*r,14*g,14*b)
#endif

#define BUTTONCOUNT 68
#define MAX_PORTAMENTO_BUTTONS 8

#define VOL_TICK (0.0005) // (1.0 / (SAMPLINGFREQ / CHANNEL_BUFFER_SIZE) / 0.5) // decay time of estimated volume
#define VOL_TICK_FACT (0.998) // 0.5**(1/(SAMPLINGFREQ / CHANNEL_BUFFER_SIZE)/0.1)
#define CLEAR_TIMER TIME_MS2I(500) // interval to clear dead notes

#define VOLUME_FACTOR 0.005f;

/* Custom tunings */
// key number = 17 * buttons[but].coord0 + 10 * buttons[but].coord1 + 30
int button_number_map[61] = {
    56, 51, 63, 58, 53, 65, 60, 55, 67, 62, 40, 35, 47, 42, 37, 49, 44,
    39, 34, 46, 41, 36, 48, 43, 38, 50, 45, 23, 18, 30, 25, 20, 32, 27,
    22, 17, 29, 24, 19, 31, 26, 21, 33, 28,  6,  1, 13,  8,  3, 15, 10,
     5,  0, 12,  7,  2, 14,  9,  4, 16, 11};

// offsets from 12TET in cent
float ji7limit[17] = {
    0.0,
    11.7312852697778,
    -29.3275731357177,
    3.91000173077484,
    15.6412870005526,
    -33.1290943962624,
    -13.6862861351652,
    -1.95500086538755,
    17.4878073957099,
    -17.48780739571,
    1.95500086538743,
    13.6862861351653,
    -27.3725722703303,
    -15.6412870005526,
    -3.91000173077487,
    -31.1740935308751,
    -11.7312852697778,
};

void MidiSend1(uint8_t b0) {
    midi_usb_MidiSend1(1, b0);
#ifdef USE_MIDI_SERIAL
    serial_MidiSend1(b0);
#endif
}

void MidiSend2(uint8_t b0, uint8_t b1) {
    midi_usb_MidiSend2(1, b0, b1);
#ifdef USE_MIDI_SERIAL
    serial_MidiSend2(b0, b1);
#endif
}

void MidiSend3(uint8_t b0, uint8_t b1, uint8_t b2) {
    midi_usb_MidiSend3(1, b0, b1, b2);
#ifdef USE_MIDI_SERIAL
    serial_MidiSend3(b0, b1, b2);
#endif
}

// Schlick power function, approximation of power function
float powf_schlick(const float a, const float b)
{
    return (a / (b - a * b + a));
}

// Derivative of schlick power function
float powf_schlick_d(const float a, const float b)
{
    float t = (-a*b+a+b);
    return b/(t*t);
}

#define pow2(x) ((x)*(x))
#define pow3(x) ((x)*(x)*(x))
#define max(x, y) ((x)>(y)?(x):(y))
#define min(x, y) ((x)<(y)?(x):(y))
#define clamp(x, low, high)  (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))

void update_leds(void);
void set_midi_mode(midi_mode_t mode);

typedef enum {
    STATE_OFF = 0,
    STATE_ON = 1,
    STATE_PORTAMENTO = 2,
    STATE_ALT = 3,
    STATE_TRANSPOSE = 4,
} button_state_t;

class MotionSensor {
    public:
        int send_motion_time = 0;
        int last_acc_x = INT32_MAX;
        int last_acc_y = INT32_MAX;
        int last_acc_z = INT32_MAX;
        int last_acc_abs = INT32_MAX;
        int last_rot_x = INT32_MAX;
        int last_rot_y = INT32_MAX;
        int last_rot_z = INT32_MAX;

        MotionSensor() {}

        void message(int* msg) {
            int acc_x = msg[0];
            int acc_y = msg[1];
            int acc_z = msg[2];
            int acc_abs = msg[3];
            int rot_x = msg[4];
            int rot_y = msg[5];
            int rot_z = msg[6];
#ifdef USE_MIDI_OUT
            if (config.send_motion_interval && (--send_motion_time <= 0)) {
                send_motion_time = config.send_motion_interval;
                if (config.send_motion_14bit) {
                    midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE,
                                    16|MIDI_C_LSB, acc_x&0x7F);
                    midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE,
                                    16, (64+(acc_x>>7))&0x7F);
                    midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE,
                                    17|MIDI_C_LSB, acc_y&0x7F);
                    midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE,
                                    17, (64+(acc_y>>7))&0x7F);
                    midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE,
                                    18|MIDI_C_LSB, acc_z&0x7F);
                    midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE,
                                    18, (64+(acc_z>>7))&0x7F);
                    midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE,
                                    19|MIDI_C_LSB, acc_abs&0x7F);
                    midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE,
                                    19, (acc_abs>>6)&0x7F);
                    midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE,
                                    80|MIDI_C_LSB, rot_x&0x7F);
                    midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE,
                                    80, (64+(rot_x>>7))&0x7F);
                    midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE,
                                    81|MIDI_C_LSB, rot_y&0x7F);
                    midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE,
                                    81, (64+(rot_y>>7))&0x7F);
                    midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE,
                                    82|MIDI_C_LSB, rot_z&0x7F);
                    midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE,
                                    82, (64+(rot_z>>7))&0x7F);
                } else {
                    // for binary protocol the max is +/-8g, for MIDI max 2g feels better
                    // -(1<<3) for rounding correctly
                    int d; // calculate direction for hysteresis
                    d = ((((last_acc_x-64)<<6) > acc_x)<<4)-(1<<3);
                    acc_x = __USAT(64+((acc_x+(1<<5)+d)>>5), 7)&0x7F;
                    d = ((((last_acc_y-64)<<6) > acc_y)<<4)-(1<<3);
                    acc_y = __USAT(64+((acc_y+(1<<5)+d)>>5), 7)&0x7F;
                    d = ((((last_acc_z-64)<<6) > acc_z)<<4)-(1<<3);
                    acc_z = __USAT(64+((acc_z+(1<<5)+d)>>5), 7)&0x7F;
                    d = (((last_acc_abs<<5) > acc_abs)<<3)-(1<<2);
                    acc_abs = __USAT((acc_abs+(1<<4)+d)>>4, 7)&0x7F;
                    d = ((((last_rot_x-64)<<6) > rot_x)<<5)-(1<<4);
                    rot_x = __USAT(64+((rot_x+(1<<5)+d)>>6), 7)&0x7F;
                    d = ((((last_rot_y-64)<<6) > rot_y)<<5)-(1<<4);
                    rot_y = __USAT(64+((rot_y+(1<<5)+d)>>6), 7)&0x7F;
                    d = ((((last_rot_z-64)<<6) > rot_z)<<5)-(1<<4);
                    rot_z = __USAT(64+((rot_z+(1<<5)+d)>>6), 7)&0x7F;
                    if (acc_x != last_acc_x) {
                        midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE,
                                           16, acc_x);
                        last_acc_x = acc_x;
                    }
                    if (acc_y != last_acc_y) {
                        midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE,
                                           17, acc_y);
                        last_acc_y = acc_y;
                    }
                    if (acc_z != last_acc_z) {
                        midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE,
                                           18, acc_z);
                        last_acc_z = acc_z;
                    }
                    if (acc_abs != last_acc_abs) {
                        midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE,
                                           19, acc_abs);
                        last_acc_abs = acc_abs;
                    }
                    if (rot_x != last_rot_x) {
                        midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE,
                                           80, rot_x);
                        last_rot_x = rot_x;
                    }
                    if (rot_y != last_rot_y) {
                        midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE,
                                           81, rot_y);
                        last_rot_y = rot_y;
                    }
                    if (rot_z != last_rot_z) {
                        midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE,
                                           82, rot_z);
                        last_rot_z = rot_z;
                    }
                }
            }
#endif
        }
};

class Button {
    public:
        int coord0;
        int coord1;
        float note;
        int midinote;
        int midinote_base;
        float start_note_offset;
        float tuning_note_offset = 0.0f;
        float pres;
        float vpres;
        int last_pres = INT32_MAX;
        int last_velo = INT32_MAX;
        int last_rvelo = INT32_MAX;
        int last_tilt = INT32_MAX;
        int last_bend = INT32_MAX;
        int last_pitchbend = INT32_MAX;
        float but_x;
        float but_y;
        float vol0;
        float vol = 0.0;
        button_state_t state = STATE_OFF;
        systime_t timer = -1;
        int voice = -1;

        Button() {
        }

        void message(float* msg) {
            pres = msg[0];
            vpres = msg[1];
            but_x = msg[2];
            but_y = msg[3];

            // allow vol below zero to take over oldest voice
            if (pres <= 0.0)
                vol0 = -1.0;
            else
                vol0 = pres + vpres;
            if (vol < vol0)
                vol = vol0;
        }
};

class Instrument {
    public:
        Button buttons[BUTTONCOUNT];
        int voices[MAX_VOICECOUNT];
        int portamento_buttons[MAX_PORTAMENTO_BUTTONS];
        float notegen0 = 12.00;
        float notegen1 = 7.00;
        float note_offset = 0;
        float start_note_offset = 62;
        float min_note_offset = 32;
        float max_note_offset = 92;
        int altmode = 0;
        int portamento = 0;
        int transposemode = 0;
        int last_button = 0;
        int master_button = -1;
        int transpose_button = -1;
        int transpose_button2 = -1;
        synth_interface_t* synth_interface;
        int voicecount = VOICECOUNT;
        int midi_channel_offset = 1;
        float midi_bend_range = 48.0;
        float bend_sensitivity = 0.25;
        float pres_sensitivity = 1.0f;
        float velo_sensitivity = 1.0f;

        Instrument(int* c0, int* c1, int n_buttons, synth_interface_t* si) {
            int n;
            for (n = 0; n < n_buttons; n++) {
                buttons[n].coord0 = c0[n];
                buttons[n].coord1 = c1[n];
                // calculate note number
                buttons[n].note = start_note_offset + buttons[n].tuning_note_offset +
                                  notegen0 * buttons[n].coord0 +
                                  notegen1 * buttons[n].coord1;
                buttons[n].midinote_base = (int)(notegen0 * buttons[n].coord0 +
                                                 notegen1 * buttons[n].coord1 + 0.5 + 100) - 100; // careful to keep rounded value above zero
                buttons[n].midinote = buttons[n].midinote_base + (int)(start_note_offset + 0.5);
            }
            for (n = 0; n < MAX_VOICECOUNT; n++) {
                voices[n] = -1;
            }
            for (n = 0; n < MAX_PORTAMENTO_BUTTONS; n++) {
                portamento_buttons[n] = -1;
            }
            synth_interface = si;
            volume = 79.0 * VOLUME_FACTOR;
        }

        void set_portamento(int p) {
            // in mono mode portamento should stay on
            if (config.midi_mode != MIDI_MODE_MPE || transposemode) {
                return;
            }
            if (p) {
                if (!portamento) {
                    portamento = 1;
                    if (buttons[last_button].state == STATE_ON) {
                        master_button = last_button;
                    }
                    // TODO: else check if only one button is pressed
                }
            } else if (portamento) {
                portamento = 0;
            }
        }

        void set_altmode(int a) {
            if (a) {
                altmode = 1;
            } else {
                altmode = 0;
            }
        }

        void set_free_transpose_mode(int p) {
            if (portamento) {
                return;
            }
            if (p) {
                if (!transposemode) {
                    transposemode = 1;
                    if (buttons[last_button].state == STATE_ON) {
                        transpose_button = last_button;
                    } else {
                        transpose_button = -1;
                    }
                    transpose_button2 = -1;
                }
            } else if (transposemode) {
                transposemode = 0;
                transpose_button = -1;
                transpose_button2 = -1;
            }
        }

        int change_note_offset(float offset) {
            return set_note_offset(start_note_offset + offset);
        }

        int set_note_offset(float offset) {
            if (offset >= min_note_offset && offset <= max_note_offset) {
                start_note_offset = offset;
                update_leds();
                return 0;
            }
            led_rgb3(255,0,0);
            return 1;
        }

        void set_notegen1(float g) {
            // keep generator within syntonic continuum range
            notegen1 = clamp(g, 6.85714285714286f, 7.2f);
            update_leds();
        }

        void reset_note_offsets(void) {
            for (int n = 0; n < 61; n++) {
                int but = button_number_map[n];
                buttons[n].tuning_note_offset = 0.0f;
            }
        }

        void set_note_offsets(float* offsets, int n_buttons) {
            if (n_buttons == 61) {
                // all 61 offsets are given
                for (int n = 0; n < 61; n++) {
                    int but = button_number_map[n];
                    buttons[but].tuning_note_offset = offsets[n] / 100;
                }
            }
            else if (n_buttons == 17) {
                // one repeating octave starting with C
                for (int n = 0; n < 61; n++) {
                    int but = button_number_map[n];
                    buttons[but].tuning_note_offset = offsets[(n + 7) % 17] / 100;
                }
            }
        }

        /* Rotate the layout 180 degrees */
        void flip(void) {
            for (int n = 0; n < BUTTONCOUNT; n++) {
                buttons[n].coord0 = -buttons[n].coord0;
                buttons[n].coord1 = -buttons[n].coord1;
                // calculate note number
                buttons[n].note = start_note_offset + buttons[n].tuning_note_offset +
                                  notegen0 * buttons[n].coord0 +
                                  notegen1 * buttons[n].coord1;
                buttons[n].midinote_base = -buttons[n].midinote_base;
                buttons[n].midinote = buttons[n].midinote_base + (int)(start_note_offset + 0.5);
            }
        }

        void button_message(int but, float* msg) {
            // process button message and send osc messages
            buttons[but].message(msg);

            // handle alternative functions of note buttons
            if ((altmode && buttons[but].state == STATE_OFF)
                || (buttons[but].state == STATE_ALT)) {
                // only handle on new press
                if (buttons[but].state == STATE_OFF && buttons[but].pres > 0.05) {
                    buttons[but].state = STATE_ALT;
                    switch (but) {
                        // row 1:  0  2  4
                        case (0): {
                            set_midi_mode(MIDI_MODE_MPE);
                        } return;
                        case (2): {
                            set_midi_mode(MIDI_MODE_POLY);
                        } return;
                        case (4): {
                            set_midi_mode(MIDI_MODE_MONO);
                        } return;
                        // row 2:  1  3  5  7  9 11
                        case (1): {
                            config.message_interval = 1;
                            ws2812_write_led(0, 1, 12, 0);
                        } return;
                        case (3): {
                            config.message_interval = 10;
                            ws2812_write_led(0, 4, 2, 0);
                        } return;
                        case (5): {
                            config.send_motion_interval = 1;
                            ws2812_write_led(0, 1, 12, 0);
                        } return;
                        case (7): {
                            config.send_motion_interval = 10;
                            ws2812_write_led(0, 4, 2, 0);
                        } return;
                        case (9): {
                            config.send_motion_interval = 0;
                            *(synth_interface->acc_abs) = 1.0f;
                            *(synth_interface->acc_x) = 0.0f;
                            *(synth_interface->acc_y) = 0.0f;
                            *(synth_interface->acc_z) = 0.0f;
                            *(synth_interface->rot_x) = 0.0f;
                            *(synth_interface->rot_y) = 0.0f;
                            *(synth_interface->rot_z) = 0.0f;
                            ws2812_write_led(0, 3, 0, 0);
                        } return;
                        case (11): { // debug setting for easy testing
                            if (portamento & 1) {
                                if (config.debug) {
                                    config.debug = 0;
                                    // led_updown(0x1000);
                                } else {
                                    config.debug = 1;
                                    // led_updown(0x1111);
                                }
                            } else {
                                flip();
                            }
                        } return;
                        // row 3: 17 19 21  6  8 10 12 14 16
                        case (17): {
                            pres_sensitivity = 0.67f;
                            ws2812_write_led(0, 1, 1, 0);
                        } return;
                        case (19): {
                            pres_sensitivity = 1.0f;
                            ws2812_write_led(0, 4, 4, 0);
                        } return;
                        case (21): {
                            pres_sensitivity = 1.5f;
                            ws2812_write_led(0, 12, 12, 0);
                        } return;
                        case (6): {
                            velo_sensitivity = 0.67f;
                            ws2812_write_led(0, 1, 0, 1);
                        } return;
                        case (8): {
                            velo_sensitivity = 1.0f;
                            ws2812_write_led(0, 4, 0, 4);
                        } return;
                        case (10): {
                            velo_sensitivity = 1.5f;
                            ws2812_write_led(0, 12, 0, 12);
                        } return;
                        case (12): {
                            bend_sensitivity = 0.0f;
                            ws2812_write_led(0, 1, 0, 0);
                        } return;
                        case (14): {
                            bend_sensitivity = 0.25f;
                            ws2812_write_led(0, 4, 0, 0);
                        } return;
                        case (16): {
                            bend_sensitivity = 0.5f;
                            ws2812_write_led(0, 12, 0, 0);
                        } return;
                        // row 4: 18 20 22 24 26 28 13 15
                        // row 5: 34 36 38 23 25 27 29 31 33
                        case (34): { // set 12tet tuning
                            notegen0 = 12.0f;
                            set_notegen1(7.0f);
                            reset_note_offsets();
                        } return;
                        case (36): { // set 31tet tuning (practically equal to quarter comma meantone)
                            notegen0 = 12.0f;
                            set_notegen1(6.96774193548387f);
                            reset_note_offsets();
                        } return;
                        case (38): { // set 19tet tuning
                            notegen0 = 12.0f;
                            set_notegen1(6.94736842105263f);
                            reset_note_offsets();
                        } return;
                        case (23): { // set pythagorean tuning
                            notegen0 = 12.0f;
                            set_notegen1(7.01955000865388f);
                            reset_note_offsets();
                        } return;
                        case (25): { // set 5tet tuning
                            notegen0 = 12.0f;
                            set_notegen1(7.2f);
                            reset_note_offsets();
                        } return;
                        case (27): { // set 7tet tuning
                            notegen0 = 12.0f;
                            set_notegen1(6.85714285714286f);
                            reset_note_offsets();
                        } return;
                        case (29): { // set Bohlen-Pierce tuning thirds 13et
                            notegen0 = 10.2413f;
                            notegen1 = 5.8522f;
                            reset_note_offsets();
                            update_leds();
                        } return;
                        case (31): { // set Bohlen-Pierce tuning sixts 13tet
                            notegen0 = 19.0196f;
                            notegen1 = 10.2413f;
                            update_leds();
                        } return;
                        case (33): { // set JI 7-limit
                            notegen0 = 12.0f;
                            set_notegen1(7.0f);
                            set_note_offsets(ji7limit, 17);
                            update_leds();
                        } return;
                        // row 6: 35 37 39 41 43 45 30 32
#ifdef USE_MIDI_OUT
                        case (35): {
                            MidiSend2(MIDI_PROGRAM_CHANGE,
                                                0);
                            ws2812_write_led(0, 4, 0, 0);
                        } return;
                        case (37): {
                            MidiSend2(MIDI_PROGRAM_CHANGE,
                                                1);
                            ws2812_write_led(0, 4, 1, 0);
                        } return;
                        case (39): {
                            MidiSend2(MIDI_PROGRAM_CHANGE,
                                                2);
                            ws2812_write_led(0, 4, 4, 0);
                        } return;
                        case (41): {
                            MidiSend2(MIDI_PROGRAM_CHANGE,
                                                3);
                            ws2812_write_led(0, 1, 8, 0);
                        } return;
                        case (43): {
                            MidiSend2(MIDI_PROGRAM_CHANGE,
                                                4);
                            ws2812_write_led(0, 0, 4, 4);
                        } return;
                        case (45): {
                            MidiSend2(MIDI_PROGRAM_CHANGE,
                                                5);
                            ws2812_write_led(0, 0, 1, 12);
                        } return;
                        case (30): {
                            MidiSend2(MIDI_PROGRAM_CHANGE,
                                                6);
                            ws2812_write_led(0, 2, 0, 6);
                        } return;
                        case (32): {
                            MidiSend2(MIDI_PROGRAM_CHANGE,
                                                7);
                            ws2812_write_led(0, 3, 3, 3);
                        } return;
#endif
                        // row 7: 51 53 55 40 42 44 46 48 50
                        case (51): {
                            MidiSend3(MIDI_CONTROL_CHANGE,
                                                MIDI_C_MAIN_VOLUME, 7);
                            volume = 7.0 * VOLUME_FACTOR;
                            ws2812_write_led(0, 1, 1, 1);
                        } return;
                        case (53): {
                            MidiSend3(MIDI_CONTROL_CHANGE,
                                                MIDI_C_MAIN_VOLUME, 15);
                            volume = 15.0 * VOLUME_FACTOR;
                            ws2812_write_led(0, 2, 2, 2);
                        } return;
                        case (55): {
                            MidiSend3(MIDI_CONTROL_CHANGE,
                                                MIDI_C_MAIN_VOLUME, 31);
                            volume = 31.0 * VOLUME_FACTOR;
                            ws2812_write_led(0, 3, 3, 3);
                        } return;
                        case (40): {
                            MidiSend3(MIDI_CONTROL_CHANGE,
                                                MIDI_C_MAIN_VOLUME, 47);
                            volume = 47.0 * VOLUME_FACTOR;
                            ws2812_write_led(0, 4, 4, 4);
                        } return;
                        case (42): {
                            MidiSend3(MIDI_CONTROL_CHANGE,
                                                MIDI_C_MAIN_VOLUME, 63);
                            volume = 63.0 * VOLUME_FACTOR;
                            ws2812_write_led(0, 6, 6, 6);
                        } return;
                        case (44): {
                            MidiSend3(MIDI_CONTROL_CHANGE,
                                                MIDI_C_MAIN_VOLUME, 79);
                            volume = 79.0 * VOLUME_FACTOR;
                            ws2812_write_led(0, 8, 8, 8);
                        } return;
                        case (46): {
                            MidiSend3(MIDI_CONTROL_CHANGE,
                                                MIDI_C_MAIN_VOLUME, 95);
                            volume = 95.0 * VOLUME_FACTOR;
                            ws2812_write_led(0, 10, 10, 10);
                        } return;
                        case (48): {
                            MidiSend3(MIDI_CONTROL_CHANGE,
                                                MIDI_C_MAIN_VOLUME, 111);
                            volume = 111.0 * VOLUME_FACTOR;
                            ws2812_write_led(0, 14, 14, 14);
                        } return;
                        case (50): {
                            MidiSend3(MIDI_CONTROL_CHANGE,
                                                MIDI_C_MAIN_VOLUME, 127);
                            volume = 127.0 * VOLUME_FACTOR;
                            ws2812_write_led(0, 18, 18, 18);
                        } return;

                        // row 8:       56 58 60 62 47 49
                        // row 9:                   63 65 67
                        case (63): { // panic/request config
#ifdef USE_MIDI_OUT
                            MidiSend3(MIDI_CONTROL_CHANGE,
                                                MIDI_C_ALL_NOTES_OFF, 0);
                            ws2812_write_led(0, 16, 0, 0);
#endif
                        } return;
                        case (65): { // send config
#ifdef USE_MIDI_OUT
                            // TODO: send config
                            ws2812_write_led(0, 16, 0, 0);
#endif
                        } return;
                        case (67): { // system reset
                            led_rgb(0xff0000);
                            chThdSleepMilliseconds(500);
                            NVIC_SystemReset();
                        } return;
                    }
                    return;
                } else if (buttons[but].pres == 0.0) {
                    buttons[but].state = STATE_OFF;
                    return;
                }
                return;
            }

            if (config.midi_mode == MIDI_MODE_POLY) {
                // in single channel poly mode just send out the notes on a single channel,
                // skip the whole channel assignment stuff.
#ifdef USE_MIDI_OUT
                if (buttons[but].pres > 0.0) {
                    // Note on detection
                    if (buttons[but].state == STATE_OFF) {
                        buttons[but].state = STATE_ON;
                        buttons[but].midinote = buttons[but].midinote_base + (int)(start_note_offset + 0.5);
                        buttons[but].start_note_offset = start_note_offset;
                        // multiply velo by 2 to cover full midi range on note on
                        int velo = 0 + buttons[but].vpres * velo_sensitivity * 128 * 2;
                        velo = clamp(velo, 1, 127);
                        MidiSend3(MIDI_NOTE_ON | midi_channel_offset,
                                        buttons[but].midinote, velo);

                        master_button = but;
                    }
                    if (master_button == -1) {
                        master_button = but;
                    }

                    if (config.midi_pres == CFG_POLY_PRESSURE) {
                        float presf = buttons[but].pres * pres_sensitivity;
                        float d; // calculate direction for hysteresis
                        d = (buttons[but].last_pres > (presf)) * 0.5 - 0.25;
                        int pres = presf * 127 + 0.5 + d;
                        pres = clamp(pres, 0, 127);

                        if (pres != buttons[but].last_pres) {
                            MidiSend3(MIDI_POLY_PRESSURE | midi_channel_offset,
                                            buttons[but].midinote, pres);
                            buttons[but].last_pres = pres;
                        }
                    }

                    if (but == master_button) {
                        // reduce pres, bend and tilt of all pressed buttons to single values
                        float presf = 0.0f;
                        float x = 0.0f;
                        float xw = 0.0f;
                        float y = 0.0f;
                        float yw = 0.0f;
                        for (int i=0; i<BUTTONCOUNT; i++) {
                            if (buttons[i].state == STATE_ON) {
                                if (buttons[i].pres > presf) presf = buttons[i].pres;
                                float w = pow2(buttons[i].but_x);
                                x += w * buttons[i].but_x;
                                xw += w;
                                w = pow2(buttons[i].but_y);
                                y += w * buttons[i].but_y;
                                yw += w;
                            }
                        }
                        if (xw > 0.0000001f) x /= xw;
                        if (yw > 0.0000001f) y /= yw;

                        float d; // calculate direction for hysteresis

                        if (config.midi_pres != CFG_POLY_PRESSURE) {
                            d = (buttons[0].last_pres > (presf)) * 0.5 - 0.25;
                            int pres = presf * 127 + 0.5 + d;
                            pres = clamp(pres, 0, 127);
                            if (pres != buttons[0].last_pres) {
                                if (config.midi_pres == CFG_CHANNEL_PRESSURE) {
                                    MidiSend2(MIDI_CHANNEL_PRESSURE | midi_channel_offset,
                                                    pres);
                                } else if (config.midi_pres < 120) {
                                    MidiSend3(MIDI_CONTROL_CHANGE | midi_channel_offset,
                                                    config.midi_pres, pres);
                                }
                                buttons[0].last_pres = pres;
                            }
                        }
                        if (config.midi_x == CFG_PITCH_BEND) {
                            x = pow3(x) * bend_sensitivity;
                            d = (buttons[0].last_bend > (0x2000 + x * 0x2000)) * 0.5 - 0.25;
                            int bend = 0x2000 + x * 0x2000 + 0.5 + d;
                            bend = clamp(bend, 0, 0x3fff);
                            if (bend != buttons[0].last_bend) {
                                MidiSend3(MIDI_PITCH_BEND | midi_channel_offset,
                                          bend & 0x7f, (bend >> 7) & 0x7f);
                                buttons[0].last_bend = bend;
                            }
                        } else if (config.midi_x < 120) {
                            d = (buttons[0].last_bend > (64 + x * 64)) * 0.5 - 0.25;
                            int bend = 64 + x * 64 + 0.5 + d;
                            bend = clamp(bend, 0, 127);
                            if (bend != buttons[0].last_bend) {
                                MidiSend3(MIDI_CONTROL_CHANGE | midi_channel_offset,
                                                config.midi_x, bend);
                                buttons[0].last_bend = bend;
                            }
                        }
                        d = (buttons[0].last_tilt > (64 + y * 64)) * 0.5 - 0.25;
                        int tilt = 64 + y * 64 + 0.5 + d;
                        tilt = clamp(tilt, 0, 127);
                        if (tilt != buttons[0].last_tilt) {
                            if (config.midi_y < 120) {
                                MidiSend3(MIDI_CONTROL_CHANGE | midi_channel_offset,
                                                config.midi_y, tilt);
                            }
                            buttons[0].last_tilt = tilt;
                        }
                    }

                } else { // Note off
                    buttons[but].state = STATE_OFF;
                    int velo = 0 - buttons[but].vpres * velo_sensitivity * 128 * 2;
                    velo = clamp(velo, 0, 127);
                    MidiSend3(MIDI_NOTE_OFF | midi_channel_offset,
                                    buttons[but].midinote, velo);
                    if (but == master_button) {
                        master_button = -1;
                    }
                }
                return;
#endif
            }

            // Note on detection
            if (buttons[but].state == STATE_OFF && buttons[but].pres > 0.0) {
                // calculate midinote only at note on
                buttons[but].midinote = buttons[but].midinote_base + (int)(start_note_offset + 0.5);
                buttons[but].start_note_offset = start_note_offset;

                if (portamento) {
                    if (master_button == -1) {
                        if (get_voice(but) >= 0) {
                            master_button = but;
                        }
                    } else {
                        // add button to portamento button
                        for (int n = 0; n < MAX_PORTAMENTO_BUTTONS; n++) {
                            if (portamento_buttons[n] == -1) {
                                portamento_buttons[n] = but;
                                buttons[but].state = STATE_PORTAMENTO;
                                break;
                            }
                        }
                    }
                } else if (transposemode) {
                    if (transpose_button == -1) {
                        if (get_voice(but) >= 0) {
                            transpose_button = but;
                        }
                    } else if (buttons[transpose_button].state == STATE_ON
                               && buttons[but].pres > 0.1) {
                        buttons[but].state = STATE_TRANSPOSE;
                        transpose_button2 = but;
                        transposemode = 0;
                    }
                } else {
                    get_voice(but);
                }
            }

            if (buttons[but].state) {
                // calculate note pitch
                buttons[but].note = buttons[but].start_note_offset + buttons[but].tuning_note_offset +
                                    notegen0 * buttons[but].coord0 +
                                    notegen1 * buttons[but].coord1
                                    + note_offset;
                buttons[but].timer = chVTGetSystemTime() + CLEAR_TIMER;

                if (but == master_button) {
                    // calculate average of portamento buttons
                    float temp_pres = buttons[but].pres; // save pres for note off detection

                    float pres = buttons[but].pres;
                    float sw = pres*pres;
                    float vpres = buttons[but].vpres;
                    float note  = sw * buttons[but].note;
                    float but_x = sw * buttons[but].but_x;
                    float but_y = sw * buttons[but].but_y;
                    for (int n = 0; n < MAX_PORTAMENTO_BUTTONS; n++) {
                        int b = portamento_buttons[n];
                        if (b >= 0) {
                            float w = buttons[b].pres;
                            w = w*w;
                            sw += w;
                            pres  += buttons[b].pres;
                            vpres += buttons[b].vpres;
                            note  += w * buttons[b].note;
                            but_x += w * buttons[b].but_x;
                            but_y += w * buttons[b].but_y;
                        }
                    }
                    buttons[but].pres = min(pres, 1.0f);
                    buttons[but].vpres = clamp(vpres, -1.0f, 1.0f);

                    if (sw > 0.0f) {
                        buttons[but].note = note / sw;
                        buttons[but].but_x = but_x / sw;
                        buttons[but].but_y = but_y / sw;
                    }

                    update_voice(but);
                    buttons[but].pres = temp_pres;
                }
                else if (but == transpose_button && transpose_button2) {
                    // calculate combined transpose buttons
                    float temp_pres = buttons[but].pres; // save pres for note off detection

                    float sw = buttons[but].pres + buttons[transpose_button2].pres;
                    if (sw > 0.0f) {
                        buttons[but].but_x = (buttons[but].pres * buttons[but].but_x +
                                            buttons[transpose_button2].pres * buttons[transpose_button2].but_x)
                                            / sw;
                        buttons[but].but_y = (buttons[but].pres * buttons[but].but_y +
                                            buttons[transpose_button2].pres * buttons[transpose_button2].but_y)
                                            / sw;
                    }
                    buttons[but].pres = max(buttons[but].pres, buttons[transpose_button2].pres);
                    buttons[but].vpres = buttons[but].vpres + buttons[transpose_button2].vpres;

                    update_voice(but);
                    buttons[but].pres = temp_pres;
                }
                else if (buttons[but].state == STATE_ON) {
                    // send synth parameters
                    update_voice(but);
                }

                // Note off detection
                if (buttons[but].pres <= 0) {
                    if (buttons[but].state == STATE_PORTAMENTO) {
                        int count = 0;
                        for (int n = 0; n < MAX_PORTAMENTO_BUTTONS; n++) {
                            if (portamento_buttons[n] == but) {
                                portamento_buttons[n] = -1;
                            }
                            count += portamento_buttons[n] >= 0;
                        }
                        buttons[but].state = STATE_OFF;
                        if (!portamento && count == 0) {
                            // if no portamento buttons left turn off portamento
                            master_button = -1;
                        }
                    } else if (but == master_button) {
                        // find other portamento button to take over portamento voice
                        for (int n = 0; n < MAX_PORTAMENTO_BUTTONS; n++) {
                            if (portamento_buttons[n] >= 0) {
                                buttons[portamento_buttons[n]].midinote = buttons[but].midinote;
                                buttons[portamento_buttons[n]].voice = buttons[but].voice;
                                master_button = portamento_buttons[n];
                                voices[buttons[master_button].voice] = master_button;
                                portamento_buttons[n] = -1;
                                buttons[master_button].state = STATE_ON;
                                buttons[but].state = STATE_OFF;
                                break;
                            }
                        }
                        // if no other portamento button is found turn off portamento
                        if (buttons[but].state) {
                            master_button = -1;
                        }
                    } else if (but == transpose_button) {
                        if (transpose_button2 >= 0
                            && buttons[transpose_button2].state == STATE_TRANSPOSE) {
                            // do free transpose
                            change_note_offset(buttons[transpose_button].note
                                               - buttons[transpose_button2].note);
                            buttons[transpose_button2].midinote = buttons[transpose_button].midinote;
                            buttons[transpose_button2].voice = buttons[transpose_button].voice;
                            buttons[transpose_button2].start_note_offset = start_note_offset;
                            voices[buttons[transpose_button2].voice] = transpose_button2;
                            buttons[transpose_button2].state = STATE_ON;
                            buttons[but].state = STATE_OFF;
                        }
                        transpose_button = -1;
                        transpose_button2 = -1;
                    } else if (buttons[but].state == STATE_TRANSPOSE) {
                        buttons[but].state = STATE_OFF;
                        transpose_button = -1;
                        transpose_button2 = -1;
                    }
                    if (but == last_button) {
                        // if last pressed button is released check if other button can take its place
                        for (int n = 0; n < voicecount; n++) {
                            if (voices[n] >= 0 && buttons[voices[n]].state == STATE_ON) {
                                last_button = voices[n];
                                break;
                            }
                        }
                    }

                    release_voice(but);
                };
            }
        }

        void release_voice(int but) {
            if (buttons[but].state == STATE_ON) {
                buttons[but].state = STATE_OFF;

#ifdef USE_MIDI_OUT
                int velo = 0 - buttons[but].vpres * velo_sensitivity * 128 * 2;
                velo = clamp(velo, 0, 127);
                MidiSend3(MIDI_NOTE_OFF | (midi_channel_offset + buttons[but].voice),
                                   buttons[but].midinote, velo);
#endif
            }
        }

        int get_voice(int but) {
            int voice = -1;
            float min_vol = buttons[but].vol;// * 0.9 - 0.05; // * factor for hysteresis
            for (int n = 0; n < voicecount; n++) {
                // check if empty or last used voice is available
                if (voices[n] == -1 || voices[n] == but)
                {
                    voice = n;
                    break;
                }
                // else find voice with minimum approximated volume
                float vol = buttons[voices[n]].vol;
                if (vol < min_vol) {
                    min_vol = vol;
                    voice = n;
                }
            }
            if (voice >= 0) {
                // take over the voice
                if (voices[voice] >= 0) {
                    release_voice(voices[voice]);
                }

                buttons[but].state = STATE_ON;
                buttons[but].voice = voice;
                buttons[but].last_pitchbend = INT32_MAX;
                voices[voice] = but;
                last_button = but;

#ifdef USE_MIDI_OUT
                // multiply velo by 2 to cover full midi range on note on
                int velo = 0 + buttons[but].vpres * velo_sensitivity * 128 * 2;
                velo = clamp(velo, 1, 127);

                MidiSend3(MIDI_NOTE_ON | (midi_channel_offset + buttons[but].voice),
                                   buttons[but].midinote, velo);
                buttons[but].last_velo = velo;
#endif

                return 0;
            }
            return -1;
        }

        void update_voice(int but) {
            float pb = bend_sensitivity * pow3(buttons[but].but_x);
            float presf = buttons[but].pres * pres_sensitivity;
            float velof = buttons[but].vpres * velo_sensitivity;
#ifdef USE_INTERNAL_SYNTH
            int voice = buttons[but].voice;
            *(synth_interface->note[voice])  = buttons[but].note + pb;
            *(synth_interface->pres[voice])  = presf;
            *(synth_interface->vpres[voice]) = velof;
            *(synth_interface->but_x[voice]) = buttons[but].but_x;
            *(synth_interface->but_y[voice]) = buttons[but].but_y;
#endif

#ifdef USE_MIDI_OUT
            float d; // calculate direction for hysteresis
            d = (buttons[but].last_pres > (presf)) * 0.5 - 0.25;
            int pres = presf * 127 + 0.5 + d;
            pres = clamp(pres, 0, 127);

            d = (buttons[but].last_tilt > (64 + buttons[but].but_y * 64)) * 0.5 - 0.25;
            int tilt = 64 + buttons[but].but_y * 64 + 0.5 + d;
            tilt = clamp(tilt, 0, 127);

            pb = (pb
                + buttons[but].note - buttons[but].midinote)
                * (0x2000 / midi_bend_range) + 0x2000;
            d = (buttons[but].last_pitchbend > pb) * 0.5 - 0.25;
            int pitchbend = pb + 0.5 + d;
            pitchbend = clamp(pitchbend, 0, 0x3fff);

            // pitchbend is also used for tuning and glissando
            if (pitchbend != buttons[but].last_pitchbend) {
                MidiSend3(MIDI_PITCH_BEND | (midi_channel_offset + buttons[but].voice),
                                   pitchbend & 0x7f, (pitchbend >> 7) & 0x7f);
                buttons[but].last_pitchbend = pitchbend;
            }
            if (pres != buttons[but].last_pres) {
                if (config.mpe_pres == CFG_CHANNEL_PRESSURE) {
                    MidiSend2(MIDI_CHANNEL_PRESSURE | (midi_channel_offset + buttons[but].voice),
                                    pres);
                } else if (config.mpe_pres < 120) {
                    MidiSend3(MIDI_CONTROL_CHANGE | (midi_channel_offset + buttons[but].voice),
                                    config.mpe_pres, pres);
                }
                buttons[but].last_pres = pres;
            }
            if (config.mpe_x < 120) {
                int bend = 64.5 + buttons[but].but_x * 64;
                bend = clamp(bend, 0, 127);
                if (bend != buttons[but].last_bend) {
                    MidiSend3(MIDI_CONTROL_CHANGE | (midi_channel_offset + buttons[but].voice),
                                    config.mpe_x, bend);
                    buttons[but].last_bend = bend;
                }
            }
            if (tilt != buttons[but].last_tilt) {
                if (config.mpe_y < 120) {
                    MidiSend3(MIDI_CONTROL_CHANGE | (midi_channel_offset + buttons[but].voice),
                                    config.mpe_y, tilt);
                }
                buttons[but].last_tilt = tilt;
            }
            if (config.mpe_contvelo < 120) {
                // TODO: hysteresis for continuous velocity
                // TODO: make contvelo CC configurable
                if (velof > 0) {
                    int velo = 0 + velof * 256;
                    velo = clamp(velo, 0, 127);
                    if (velo != buttons[but].last_velo) {
                        MidiSend3(MIDI_CONTROL_CHANGE | (midi_channel_offset + buttons[but].voice),
                                        73, velo);
                        buttons[but].last_velo = velo;
                    }
                    if (buttons[but].last_rvelo > 0) {
                        MidiSend3(MIDI_CONTROL_CHANGE | (midi_channel_offset + buttons[but].voice),
                                        72, 0);
                        buttons[but].last_rvelo = 0;
                    }
                } else {
                    int rvelo = 0 - velof * 256;
                    rvelo = clamp(rvelo, 0, 127);
                    if (rvelo != buttons[but].last_rvelo) {
                        MidiSend3(MIDI_CONTROL_CHANGE | (midi_channel_offset + buttons[but].voice),
                                        72, rvelo);
                        buttons[but].last_rvelo = rvelo;
                    }
                    if (buttons[but].last_velo > 0) {
                        MidiSend3(MIDI_CONTROL_CHANGE | (midi_channel_offset + buttons[but].voice),
                                        73, 0);
                        buttons[but].last_velo = 0;
                    }
                }
            }
#endif
        }

        void clear_dead_notes(void) {
            systime_t now = chVTGetSystemTime();

            for (int n = 0; n < voicecount; n++) {
                if (voices[n] >= 0 && buttons[voices[n]].state == STATE_ON
                    && buttons[voices[n]].timer < now)
                {
                    release_voice(voices[n]);
                    #ifdef USE_INTERNAL_SYNTH
                    *(synth_interface->pres[n])  = 0.0;
                    *(synth_interface->vpres[n]) = 0.0;
                    *(synth_interface->but_x[n]) = 0.0;
                    *(synth_interface->but_y[n]) = 0.0;
                    #endif
                }
            }
        }

        void tick(void) {
            // approximated volume
            for (int n = 0; n < voicecount; n++) {
                if (voices[n] >= 0) {
                    buttons[voices[n]].vol *= VOL_TICK_FACT;
                    buttons[voices[n]].vol -= VOL_TICK;
                    if (buttons[voices[n]].vol < buttons[voices[n]].vol0)
                        buttons[voices[n]].vol = buttons[voices[n]].vol0;
                }
            }
        }
};

int c0_dis[68] = {
    6, 5, 5, 4, 4, 3,  2,  2,  1,  1,  0,  0, -1, -2, -2, -3, -3,
    5, 4, 4, 3, 3, 2,  1,  1,  0,  0, -1, -1, -2, -3, -3, -4, -4,
    4, 3, 3, 2, 2, 1,  0,  0, -1, -1, -2, -2, -3, -4, -4, -5, -5,
    3, 2, 2, 1, 1, 0, -1, -1, -2, -2, -3, -3, -4, -5, -5, -6, -6};

int c1_dis[68] = {
    -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8,
    -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8,
    -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8,
    -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8};

#ifdef USE_INTERNAL_SYNTH
Instrument dis(c0_dis, c1_dis, BUTTONCOUNT, &synth_interface);
#else
Instrument dis(c0_dis, c1_dis, BUTTONCOUNT, NULL);
#endif
MotionSensor motion;

void int2float(int *msg, float *fmsg, int n) {
    int c;
    for (c=0; c<n; c++) {
        fmsg[c] = ((float)msg[c])/0x1fff; //8191, maximum of 14bit signed int
    }
}

unsigned int aux_button_map = 0;

int synth_message(int size, int* msg) {
    static float note_offset_old;
    float fmsg[7];
    int src = msg[0];
    int id = msg[1];
    msg = &msg[2];
    size -= 2;

    if (src == ID_CONTROL) {
        if (id == IDC_ALT) {
            dis.set_altmode(msg[0]);
            update_leds();
        }
        else if (id == IDC_PORTAMENTO) {
            // Portamento button
            dis.set_portamento(msg[0]);
            update_leds();
        }
        else if (msg[0]) {
            aux_button_map |= 1<<id;
            if (aux_button_map == ((1<<IDC_OCT_UP) | (1<<IDC_OCT_DOWN))) {
                if (dis.altmode) {
                    // transpose reset
                    dis.set_note_offset(62);
                } else {
                    // cancel last transpose and enable free transpose
                    dis.set_note_offset(note_offset_old);
                    dis.set_free_transpose_mode(1);
                }
            } else {
                note_offset_old = dis.start_note_offset;
                int dif = 12;
                if (dis.altmode) dif = 1;
                if (id == IDC_OCT_UP) {
                    dis.change_note_offset(dif);
                }
                if (id == IDC_OCT_DOWN) {
                    dis.change_note_offset(-dif);
                }
            }
        } else {
            aux_button_map &= ~(1<<id);
            if (aux_button_map != ((1<<IDC_OCT_UP) | (1<<IDC_OCT_DOWN))) {
                dis.set_free_transpose_mode(0);
            }
            update_leds();
        }
    }
    else if (src == ID_ACCEL && size == 7) {
        motion.message(msg);
    }
    else if ((src == ID_DIS || src == ID_BAS) && size == 4 && id < BUTTONCOUNT && id >= 0) {
        // 4x14bit value, dis or bas button, version >= 2.1

        int2float(msg, fmsg, size);

        if (src == ID_DIS) {
            dis.button_message(id, fmsg);
        }
    }
    else {
        // unknown message source
        return -1;
    }
    return 0;
}

void MidiInMsgHandler(midi_device_t dev, uint8_t port, uint8_t status,
                      uint8_t data1, uint8_t data2) {
    (void) dev;
    (void) port;
    static uint8_t lastRPN_LSB = 0;
    static uint8_t lastRPN_MSB = 0;
    static uint8_t lastNRPN_LSB = 0;
    static uint8_t lastNRPN_MSB = 0;
    static uint8_t lsb_cc1 = 0;

    uint8_t channel = status & 0x0f;
    status &= 0xf0;
    ws2812_write_led(0, 18, 0, 9);

    if (status == MIDI_CONTROL_CHANGE) {
        switch (data1) {
            case MIDI_C_RPN_LSB: lastRPN_LSB = data2; lastNRPN_LSB = lastNRPN_MSB = 0x7f; break;
            case MIDI_C_RPN_MSB: lastRPN_MSB = data2; lastNRPN_LSB = lastNRPN_MSB = 0x7f; break;
            case MIDI_C_NONRPN_LSB: lastNRPN_LSB = data2; lastRPN_LSB = lastRPN_MSB = 0x7f; break;
            case MIDI_C_NONRPN_MSB: lastNRPN_MSB = data2; lastRPN_LSB = lastRPN_MSB = 0x7f; break;
            case MIDI_C_DATA_ENTRY: {
                if (lastRPN_LSB == 0 && lastRPN_MSB == 0) {
                    dis.midi_bend_range = data2;
                } else if (lastRPN_LSB == 6 && lastRPN_MSB == 0) {
                    if (channel + data2 <= 15) {
                        set_midi_mode(MIDI_MODE_MPE);
                        dis.midi_channel_offset = channel + 1;
                        if (data2 > MAX_VOICECOUNT) {
                            dis.voicecount = MAX_VOICECOUNT;
                        } else {
                            dis.voicecount = data2;
                        }
                    }
                } else if (lastRPN_LSB == 1 && lastRPN_MSB == 0) {
                    // TODO: Tuning, done with pitch bend
                }
            } break;
            case  1: { // Fifth (microtonal) tuning
                dis.set_notegen1(6.80 + (float)data2 * (0.20/64) + (float)lsb_cc1 * (0.20/8192));
            } break;
            case  1|MIDI_C_LSB: lsb_cc1 = data2; break;
            case 16: config.send_motion_interval = data2; break;
            // case 16|MIDI_C_LSB: config.send_motion_14bit = data2; break;
            case 17: config.message_interval = data2 >= 1 ? data2 : 1; break;
            // case 17|MIDI_C_LSB: config.send_button_14bit = data2; break;
            case 18: { // Note offset
                dis.set_note_offset(data2 - 2);
            } break;
            case 65: dis.set_portamento(data2); break;
            case 70: { // pressure
                if (data2 == 0) data2 = CFG_DISABLE;
                if (config.midi_mode == MIDI_MODE_MPE) {
                    config.mpe_pres = data2;
                } else {
                    config.midi_pres = data2;
                }
            } break;
            case 71: { // x
                if (data2 == 0) data2 = CFG_DISABLE;
                if (config.midi_mode == MIDI_MODE_MPE) {
                    config.mpe_x = data2;
                } else {
                    config.midi_x = data2;
                }
            } break;
            case 72: { // cont release velo
            } break;
            case 73: { // cont velo
            } break;
            case 74: { // y
                if (data2 == 0) data2 = CFG_DISABLE;
                if (config.midi_mode == MIDI_MODE_MPE) {
                    config.mpe_y = data2;
                } else {
                    config.midi_y = data2;
                }
            } break;
            case 75: { // pitch bend sensitivity, TODO: different CC?
                dis.bend_sensitivity = (float)data2 * 0.01;
            } break;
            case 126: {
                if (data2 == 0) {
                    set_midi_mode(MIDI_MODE_MPE);
                } else if (data2 <= 16) {
                    set_midi_mode(MIDI_MODE_MONO);
                    dis.midi_channel_offset = data2 - 1;
                }
            } break;
            case 127: {
                set_midi_mode(MIDI_MODE_POLY);
            } break;
            default: break;
        }
    } else if (status == MIDI_PITCH_BEND) {
        dis.note_offset = (float)((int)((data2<<7)+data1)-0x2000) * (2.0/8192);
    }
    update_leds();
}

void set_midi_mode(midi_mode_t mode) {
    // TODO: handle switching while buttons are pressed
    switch (mode) {
        case MIDI_MODE_MPE: {
            config.midi_mode = mode;
            dis.midi_channel_offset = 1;
            dis.voicecount = VOICECOUNT;
            dis.portamento = 0;
            ws2812_write_led(0, 0, 16, 0);
        } break;
        case MIDI_MODE_POLY: {
            config.midi_mode = mode;
            dis.midi_channel_offset = 0;
            dis.portamento = 0;
            ws2812_write_led(0, 8, 1, 0);
        } break;
        case MIDI_MODE_MONO: {
            config.midi_mode = mode;
            dis.midi_channel_offset = 0;
            dis.voicecount = 1;
            dis.portamento = 2; // 2 keeps led from brightening
            ws2812_write_led(0, 0, 1, 18);
        } break;
    }
}

void midi_config(void) {
#ifdef USE_MIDI_OUT
    // Send a midi message to show we're connected
    MidiSend3(MIDI_CONTROL_CHANGE, MIDI_C_ALL_NOTES_OFF, 0);
#endif
}

void clear_dead_notes(void) {
    dis.clear_dead_notes();
}

void synth_tick(void) {
    dis.tick();
}

void update_leds(void) {
    uint8_t r = 0, g = 2, b = 0;

    // base color depending on tuning system
    if (dis.notegen1 < 6.94)       {r = 1; g = 0; b = 1;}
    else if (dis.notegen1 < 6.957) {r = 0; g = 0; b = 2;}
    else if (dis.notegen1 < 6.984) {r = 0; g = 1; b = 1;}
    else if (dis.notegen1 < 7.010) {r = 0; g = 2; b = 0;}
    else if (dis.notegen1 < 7.03)  {r = 1; g = 1; b = 0;}
    else                           {r = 2; g = 0; b = 0;}

    int m = 21 * (4 + dis.altmode + (dis.portamento & 1));
    led_rgb3(m*r, m*g, m*b);

#ifdef USE_WS2812
    ws2812_write_led(0, m*r, m*g, m*b);

    if (dis.altmode) {
        // color based on offset within octave
        int WheelPos = ((dis.start_note_offset + 2) * 8
            + (int)(dis.note_offset * 8 + 48 * 8 + .5))
            % (12 * 8);
        switch(WheelPos >> 5) {
            case 0:
                r = 31 - WheelPos % 32;   //Red down
                g = WheelPos % 32;      // Green up
                b = 0;                  //blue off
                break;
            case 1:
                g = 31 - WheelPos % 32;  //green down
                b = WheelPos % 32;      //blue up
                r = 0;                  //red off
                break;
            case 2:
                b = 31 - WheelPos % 32;  //blue down
                r = WheelPos % 32;      //red up
                g = 0;                  //green off
                break;
        }
        ws2812_write_led(1, r/4, g/4, b/4);
        ws2812_write_led(2, r/4, g/4, b/4);
    } else {
        int oct = (int)((dis.start_note_offset + 4) / 12) - 5;
        if (oct > 0) {
            oct = oct * oct;
            ws2812_write_led(1, oct * r, oct * g, oct * b);
            ws2812_write_led(2, 0, 0, 0);
        } else {
            oct = oct * oct;
            ws2812_write_led(1, 0, 0, 0);
            ws2812_write_led(2, oct * r, oct * g, oct * b);
        }
    }
#endif

    int note_offset = (int)(dis.start_note_offset + 0.5f) - 62;
    if      (note_offset < -12) led_updown(0x1100);
    else if (note_offset <   0) led_updown(0x0100);
    else if (note_offset >  12) led_updown(0x0011);
    else if (note_offset >   0) led_updown(0x0010);
    else                        led_updown(0x0000);
}
