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

#ifndef USE_WS2812
#define ws2812_write_led(n,r,g,b) led_rgb3(14*r,14*g,14*b)
#endif

#define BUTTONCOUNT 68
#define MAX_PORTAMENTO_BUTTONS 8

#define VOL_TICK (0.0005) // (1.0 / (SAMPLINGFREQ / CHANNEL_BUFFER_SIZE) / 0.5) // decay time of estimated volume
#define VOL_TICK_FACT (0.998) // 0.5**(1/(SAMPLINGFREQ / CHANNEL_BUFFER_SIZE)/0.1)
#define CLEAR_TIMER TIME_MS2I(500) // interval to clear dead notes

#define VOLUME_FACTOR 0.005f;

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

void update_leds(void);
void set_midi_mode(midi_mode_t mode);

typedef enum {
    STATE_OFF = 0,
    STATE_ON = 1,
    STATE_PORTAMENTO = 2,
    STATE_ALT = 3,
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
        float signals[6] = {0,0,0,0,0,0};
        float note;
        int midinote;
        int midinote_base;
        int start_note_offset;
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
            unsigned int n;
            // copy given parameters
            for (n = 0; n < 3; n++) {
                signals[n] = msg[n];
                signals[3+n] = msg[3+n];
            }
            // calculate pres already for note detection and voice allocation
            pres = (signals[0] + signals[1] + signals[2])/3;
            if (pres > 1.0) {
                pres = 1.0;
            }
            vpres = (signals[3] + signals[4] + signals[5])/3;
            if (vpres > 1.0) {
                vpres = 1.0;
            } else if (vpres < -1.0) {
                vpres = -1.0;
            }

            // allow vol below zero to take over oldest voice
            if (pres <= 0.0)
                vol0 = -1.0;
            else
                vol0 = pres + vpres;
            if (vol < vol0)
                vol = vol0;
        }

        #define CENTERTEND 0.02
        void calculate() {
            // m = max(signals)
            float m = signals[0];
            if (signals[1] > m) m = signals[1];
            if (signals[2] > m) m = signals[2];
            if (m > 0.0) {
                float fact = 1.0/(m + CENTERTEND/m - CENTERTEND);
                but_x = (signals[2] - signals[0]) * fact;
                but_y = (0.5 * (signals[0] + signals[2]) - signals[1]) * fact;
            } else {
                but_x = 0.0;
                but_y = 0.0;
            }
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
        int start_note_offset = 62;
        float min_note_offset = 32;
        float max_note_offset = 92;
        int altmode = 0;
        int portamento = 0;
        int last_button = 0;
        int port_voice = -1;
        int portamento_button = -1;
        synth_interface_t* synth_interface;
        int voicecount = VOICECOUNT;
        int midi_channel_offset = 1;
        float midi_bend_range = 48.0;
        float bend_sensitivity = 0.25;
        int pres_sensitivity = 127;
        int velo_sensitivity = 256;
        int rvelo_sensitivity = 256;

        Instrument(int* c0, int* c1, int n_buttons, synth_interface_t* si) {
            int n;
            for (n = 0; n < n_buttons; n++) {
                buttons[n].coord0 = c0[n];
                buttons[n].coord1 = c1[n];
                // calculate note number
                buttons[n].note = start_note_offset +
                                  notegen0 * buttons[n].coord0 +
                                  notegen1 * buttons[n].coord1;
                buttons[n].midinote_base = (int)(buttons[n].note + 0.5) - start_note_offset; // careful to keep rounded value above zero
                buttons[n].midinote = buttons[n].midinote_base;
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
            if (config.midi_mode == MIDI_MODE_MONO) {
                return;
            }
            if (p) {
                if (!portamento) {
                    portamento = 1;
                    if (buttons[last_button].state == STATE_ON) {
                        portamento_button = last_button;
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

        int change_note_offset(int offset) {
            int n = start_note_offset + offset;
            if (n >= min_note_offset && n <= max_note_offset) {
                start_note_offset = n;
                return 0;
            }
            return 1;
        }

        int set_note_offset(int offset) {
            if (offset >= min_note_offset && offset <= max_note_offset) {
                start_note_offset = offset;
                return 0;
            }
            return 1;
        }

        void set_notegen1(float g) {
            if (g < 6.85714285714286f) {
                g = 6.85714285714286f;
            } else if (g > 7.2f) {
                g = 7.2f;
            }
            notegen1 = g;
            update_leds();
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
                            ws2812_write_led(0, 3, 0, 0);
                        } return;
                        // row 3: 17 19 21  6  8 10 12 14 16
                        case (17): {
                            pres_sensitivity = 127;
                            ws2812_write_led(0, 1, 1, 0);
                        } return;
                        case (19): {
                            pres_sensitivity = 192;
                            ws2812_write_led(0, 4, 4, 0);
                        } return;
                        case (21): {
                            pres_sensitivity = 288;
                            ws2812_write_led(0, 12, 12, 0);
                        } return;
                        case (6): {
                            velo_sensitivity = 174;
                            rvelo_sensitivity = 174;
                            ws2812_write_led(0, 1, 0, 1);
                        } return;
                        case (8): {
                            velo_sensitivity = 256;
                            rvelo_sensitivity = 256;
                            ws2812_write_led(0, 4, 0, 4);
                        } return;
                        case (10): {
                            velo_sensitivity = 384;
                            rvelo_sensitivity = 384;
                            ws2812_write_led(0, 12, 0, 12);
                        } return;
                        case (12): {
                            bend_sensitivity = 0.0f;
                            ws2812_write_led(0, 1, 0, 0);
                        } return;
                        case (14): {
                            bend_sensitivity = 0.125f;
                            ws2812_write_led(0, 4, 0, 0);
                        } return;
                        case (16): {
                            bend_sensitivity = 0.25f;
                            ws2812_write_led(0, 12, 0, 0);
                        } return;
                        // row 4: 18 20 22 24 26 28 13 15
                        // row 5: 34 36 38 23 25 27 29 31 33
                        case (34): { // set 12tet tuning
                            set_notegen1(7.0f);
                        } return;
                        case (36): { // set quarter comma meantone tuning
                            set_notegen1(6.96578428466209f);
                        } return;
                        case (38): { // set 19tet tuning
                            set_notegen1(6.94736842105263f);
                        } return;
                        case (23): { // set pythagorean tuning
                            set_notegen1(7.01955000865388f);
                        } return;
                        case (25): { // set 5tet tuning
                            set_notegen1(7.2f);
                        } return;
                        case (27): { // set 7tet tuning
                            set_notegen1(6.85714285714286f);
                        } return;
                        case (29): { // set 31tet tuning
                            set_notegen1(6.96774193548387f);
                        } return;
                        // row 6: 35 37 39 41 43 45 30 32
#ifdef USE_MIDI_OUT
                        case (35): {
                            midi_usb_MidiSend2(1, MIDI_PROGRAM_CHANGE,
                                                0);
                            ws2812_write_led(0, 4, 0, 0);
                        } return;
                        case (37): {
                            midi_usb_MidiSend2(1, MIDI_PROGRAM_CHANGE,
                                                1);
                            ws2812_write_led(0, 4, 1, 0);
                        } return;
                        case (39): {
                            midi_usb_MidiSend2(1, MIDI_PROGRAM_CHANGE,
                                                2);
                            ws2812_write_led(0, 4, 4, 0);
                        } return;
                        case (41): {
                            midi_usb_MidiSend2(1, MIDI_PROGRAM_CHANGE,
                                                3);
                            ws2812_write_led(0, 1, 8, 0);
                        } return;
                        case (43): {
                            midi_usb_MidiSend2(1, MIDI_PROGRAM_CHANGE,
                                                4);
                            ws2812_write_led(0, 0, 4, 4);
                        } return;
                        case (45): {
                            midi_usb_MidiSend2(1, MIDI_PROGRAM_CHANGE,
                                                5);
                            ws2812_write_led(0, 0, 1, 12);
                        } return;
                        case (30): {
                            midi_usb_MidiSend2(1, MIDI_PROGRAM_CHANGE,
                                                6);
                            ws2812_write_led(0, 2, 0, 6);
                        } return;
                        case (32): {
                            midi_usb_MidiSend2(1, MIDI_PROGRAM_CHANGE,
                                                7);
                            ws2812_write_led(0, 3, 3, 3);
                        } return;
#endif
                        // row 7: 51 53 55 40 42 44 46 48 50
                        case (51): {
                            midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE,
                                                MIDI_C_MAIN_VOLUME, 7);
                            volume = 7.0 * VOLUME_FACTOR;
                            ws2812_write_led(0, 1, 1, 1);
                        } return;
                        case (53): {
                            midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE,
                                                MIDI_C_MAIN_VOLUME, 15);
                            volume = 15.0 * VOLUME_FACTOR;
                            ws2812_write_led(0, 2, 2, 2);
                        } return;
                        case (55): {
                            midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE,
                                                MIDI_C_MAIN_VOLUME, 31);
                            volume = 31.0 * VOLUME_FACTOR;
                            ws2812_write_led(0, 3, 3, 3);
                        } return;
                        case (40): {
                            midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE,
                                                MIDI_C_MAIN_VOLUME, 47);
                            volume = 47.0 * VOLUME_FACTOR;
                            ws2812_write_led(0, 4, 4, 4);
                        } return;
                        case (42): {
                            midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE,
                                                MIDI_C_MAIN_VOLUME, 63);
                            volume = 63.0 * VOLUME_FACTOR;
                            ws2812_write_led(0, 6, 6, 6);
                        } return;
                        case (44): {
                            midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE,
                                                MIDI_C_MAIN_VOLUME, 79);
                            volume = 79.0 * VOLUME_FACTOR;
                            ws2812_write_led(0, 8, 8, 8);
                        } return;
                        case (46): {
                            midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE,
                                                MIDI_C_MAIN_VOLUME, 95);
                            volume = 95.0 * VOLUME_FACTOR;
                            ws2812_write_led(0, 10, 10, 10);
                        } return;
                        case (48): {
                            midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE,
                                                MIDI_C_MAIN_VOLUME, 111);
                            volume = 111.0 * VOLUME_FACTOR;
                            ws2812_write_led(0, 14, 14, 14);
                        } return;
                        case (50): {
                            midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE,
                                                MIDI_C_MAIN_VOLUME, 127);
                            volume = 127.0 * VOLUME_FACTOR;
                            ws2812_write_led(0, 18, 18, 18);
                        } return;

                        // row 8:       56 58 60 62 47 49
                        // row 9:                   63 65 67
                        case (63): { // panic/request config
#ifdef USE_MIDI_OUT
                            midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE,
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
                // Note on detection
                if (buttons[but].pres > 0.0) {
                    if (buttons[but].state == STATE_OFF) {
                        buttons[but].state = STATE_ON;
                        buttons[but].midinote = buttons[but].midinote_base + start_note_offset;
                        buttons[but].start_note_offset = start_note_offset;
                        int velo = 0 + buttons[but].vpres * velo_sensitivity;
                        if (velo > 127) velo = 127;
                        else if (velo < 1) velo = 1;
                        midi_usb_MidiSend3(1, MIDI_NOTE_ON | midi_channel_offset,
                                        buttons[but].midinote, velo);
                    }
                    int pres = buttons[but].pres * pres_sensitivity;
                    if (pres > 127) pres = 127;
                    else if (pres < 0) pres = 0;
                    if (pres != buttons[but].last_pres) {
                        midi_usb_MidiSend3(1, MIDI_POLY_PRESSURE | midi_channel_offset,
                                           buttons[but].midinote, pres);
                        buttons[but].last_pres = pres;
                    }
                } else {
                    buttons[but].state = STATE_OFF;
                    int velo = 0 - buttons[but].vpres * rvelo_sensitivity;
                    if (velo > 127) velo = 127;
                    else if (velo < 0) velo = 0;
                    midi_usb_MidiSend3(1, MIDI_NOTE_OFF | midi_channel_offset,
                                    buttons[but].midinote, velo);
                }
                return;
#endif
            }

            // Note on detection
            if (buttons[but].state == STATE_OFF && buttons[but].pres > 0.0) {
                // calculate midinote only at note on
                buttons[but].midinote = buttons[but].midinote_base + start_note_offset;
                buttons[but].start_note_offset = start_note_offset;
                
                if (portamento) {
                    if (portamento_button == -1) {
                        if (get_voice(but) >= 0) {
                            portamento_button = but;
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
                } else {
                    get_voice(but);
                }
            }

            if (buttons[but].state) {
                // calculate note pitch
                buttons[but].note = buttons[but].start_note_offset + note_offset +
                                    notegen0 * buttons[but].coord0 +
                                    notegen1 * buttons[but].coord1;
                buttons[but].calculate();
                buttons[but].timer = chVTGetSystemTime() + CLEAR_TIMER;

                if (but == portamento_button) {
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
                    buttons[but].pres = pres;
                    if (sw == 0) sw = 1;
                    if (note > 0.1) {
                        buttons[but].note = note / sw;
                    }
                    buttons[but].vpres = vpres;
                    buttons[but].but_x = but_x / sw;
                    buttons[but].but_y = but_y / sw;
                    
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
                            portamento_button = -1;
                        }
                    } else if (but == portamento_button) {
                        // find other portamento button to take over portamento voice
                        for (int n = 0; n < MAX_PORTAMENTO_BUTTONS; n++) {
                            if (portamento_buttons[n] >= 0) {
                                buttons[portamento_buttons[n]].midinote = buttons[but].midinote;
                                buttons[portamento_buttons[n]].voice = buttons[but].voice;
                                portamento_button = portamento_buttons[n];
                                voices[buttons[portamento_button].voice] = portamento_button;
                                portamento_buttons[n] = -1;
                                buttons[portamento_button].state = STATE_ON;
                                buttons[but].state = STATE_OFF;
                                break;
                            }
                        }
                        // if no other portamento button is found turn off portamento
                        if (buttons[but].state) {
                            portamento_button = -1;
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
                int velo = 0 - buttons[but].vpres * rvelo_sensitivity;
                if (velo > 127) velo = 127;
                else if (velo < 0) velo = 0;
                midi_usb_MidiSend3(1, MIDI_NOTE_OFF | (midi_channel_offset + buttons[but].voice),
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
                int velo = 0 + buttons[but].vpres * velo_sensitivity;
                if (velo > 127) velo = 127;
                else if (velo < 1) velo = 1;

                midi_usb_MidiSend3(1, MIDI_NOTE_ON | (midi_channel_offset + buttons[but].voice),
                                   buttons[but].midinote, velo);
                buttons[but].last_velo = velo;
#endif

                return 0;
            }
            return -1;
        }

        void update_voice(int but) {
            float pb = bend_sensitivity * pow3(buttons[but].but_x);
#ifdef USE_INTERNAL_SYNTH
            int voice = buttons[but].voice;
            *(synth_interface->note[voice])  = buttons[but].note + pb;
            *(synth_interface->pres[voice])  = buttons[but].pres;
            *(synth_interface->vpres[voice]) = buttons[but].vpres;
            *(synth_interface->but_x[voice]) = buttons[but].but_x;
            *(synth_interface->but_y[voice]) = buttons[but].but_y;
#endif

#ifdef USE_MIDI_OUT
            float d; // calculate direction for hysteresis
            d = (buttons[but].last_pres > (buttons[but].pres * pres_sensitivity)) * 0.5 - 0.25;
            int pres = buttons[but].pres * pres_sensitivity + 0.5 + d;
            if (pres > 127) pres = 127;
            else if (pres < 0) pres = 0;

            d = (buttons[but].last_tilt > (64 + buttons[but].but_y * 64)) * 0.5 - 0.25;
            int tilt = 64 + buttons[but].but_y * 64 + 0.5 + d;
            if (tilt > 127) tilt = 127;
            else if (tilt < 0) tilt = 0;

            pb = (pb
                + buttons[but].note - buttons[but].midinote)
                * (0x2000 / midi_bend_range) + 0x2000;
            d = (buttons[but].last_pitchbend > pb) * 0.5 - 0.25;
            int pitchbend = pb + 0.5 + d;
            if (pitchbend >= 0x4000) {
                pitchbend = 0x3fff;
            } else if (pitchbend < 0) {
                pitchbend = 0;
            }

            // pitchbend is also used for tuning and glissando
            if (pitchbend != buttons[but].last_pitchbend) {
                midi_usb_MidiSend3(1, MIDI_PITCH_BEND | (midi_channel_offset + buttons[but].voice),
                                   pitchbend & 0x7f, (pitchbend >> 7) & 0x7f);
                buttons[but].last_pitchbend = pitchbend;
            }
            if (pres != buttons[but].last_pres) {
                if (config.midi_pres == 1) {
                    midi_usb_MidiSend2(1, MIDI_CHANNEL_PRESSURE | (midi_channel_offset + buttons[but].voice),
                                    pres);
                } else if (config.midi_pres == 2) {
                    midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE | (midi_channel_offset + buttons[but].voice),
                                    70, pres);
                }
                buttons[but].last_pres = pres;
            }
            if (config.midi_bend == 2) {
                int bend = 64.5 + buttons[but].but_x * 64;
                if (bend > 127) bend = 127;
                else if (bend < 0) bend = 0;
                if (bend != buttons[but].last_bend) {
                    midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE | (midi_channel_offset + buttons[but].voice),
                                    71, bend);
                    buttons[but].last_bend = bend;
                }
            }
            if (tilt != buttons[but].last_tilt) {
                midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE | (midi_channel_offset + buttons[but].voice),
                                74, tilt);
                buttons[but].last_tilt = tilt;
            }
            if (config.midi_contvelo) {
                // TODO: hysteresis for continuous velocity
                if (buttons[but].vpres > 0) {
                    int velo = 0 + buttons[but].vpres * velo_sensitivity;
                    if (velo > 127) velo = 127;
                    else if (velo < 0) velo = 0;
                    if (velo != buttons[but].last_velo) {
                        midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE | (midi_channel_offset + buttons[but].voice),
                                        73, velo);
                        buttons[but].last_velo = velo;
                    }
                    if (buttons[but].last_rvelo > 0) {
                        midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE | (midi_channel_offset + buttons[but].voice),
                                        72, 0);
                        buttons[but].last_rvelo = 0;
                    }
                } else {
                    int rvelo = 0 - buttons[but].vpres * rvelo_sensitivity;
                    if (rvelo > 127) rvelo = 127;
                    else if (rvelo < 0) rvelo = 0;
                    if (rvelo != buttons[but].last_rvelo) {
                        midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE | (midi_channel_offset + buttons[but].voice),
                                        72, rvelo);
                        buttons[but].last_rvelo = rvelo;
                    }
                    if (buttons[but].last_velo > 0) {
                        midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE | (midi_channel_offset + buttons[but].voice),
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

int synth_message(int size, int* msg) {
    float fmsg[7];
    int src = msg[0];
    int id = msg[1];
    msg = &msg[2];

    if (src == ID_CONTROL) {
        if (id == IDC_ALT) {
            dis.set_altmode(msg[0]);
        }
        else if (id == IDC_PORTAMENTO) {
            // Portamento button
            dis.set_portamento(msg[0]);
        }
        else if (msg[0]) {
            int dif = 12;
            if (dis.altmode) dif = 1;
            if (id == IDC_OCT_UP) {
                dis.change_note_offset(dif);
            }
            if (id == IDC_OCT_DOWN) {
                dis.change_note_offset(-dif);
            }
        }
        update_leds();
    }
    else if (src == ID_ACCEL && size == 9) {
        motion.message(msg);
    }
    else if ((src == ID_DIS || src == ID_BAS) && size == 8 && id < BUTTONCOUNT && id >= 0) {
        // 6x14bit value, dis or bas button

        int2float(msg, fmsg, size-2);

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
            case 16|MIDI_C_LSB: config.send_motion_14bit = data2; break;
            case 17: config.message_interval = data2 >= 1 ? data2 : 1; break;
            case 17|MIDI_C_LSB: config.send_button_14bit = data2; break;
            case 18: { // Note offset
                dis.set_note_offset(data2 - 2);
            } break;
            case 65: dis.set_portamento(data2); break;
            case 70: if (data2 > 0 && data2 < 3) config.midi_pres = data2; break;
            case 71: {
                if (data2 > 0) {
                    dis.bend_sensitivity = (float)(data2 - 1) * 0.01;
                    config.midi_bend = 0;
                } else {
                    dis.bend_sensitivity = 0;
                    config.midi_bend = 2;
                }
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
            dis.portamento = 1;
            ws2812_write_led(0, 0, 1, 18);
        } break;
    }
}

void midi_config(void) {
#ifdef USE_MIDI_OUT
    // Send a midi message to show we're connected
    midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE, MIDI_C_ALL_NOTES_OFF, 0);
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

    int m = 21 * (4 + dis.altmode + dis.portamento);
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

    int note_offset = dis.start_note_offset - 62;
    if      (note_offset < -12) led_updown(0x1100);
    else if (note_offset <   0) led_updown(0x0100);
    else if (note_offset >  12) led_updown(0x0011);
    else if (note_offset >   0) led_updown(0x0010);
    else                        led_updown(0x0000);
}
