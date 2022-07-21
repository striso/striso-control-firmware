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

#include "ccportab.h"

extern "C" {
    #include "synth.h"
    #include "ws2812.h"
    #include "led.h"
}

#include "aux_jack.h"
#include "config.h"
#include "config_store.h"
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

float volume_linear = 90.0f; // volume in range 0-127
void set_volume(float vol) {
    volume_linear = vol;
    volume = volume_linear * volume_linear * (1.0f / 16129.0f);
}

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

float clamp_rem(float x, float low, float high, float* rem) {
    if (x > high) {
        *rem = x - high;
        return high;
    } else if (x < low) {
        *rem = x - low;
        return low;
    } else {
        return x;
    }
}

static inline uint32_t log2i(const uint32_t x){return (31 - __builtin_clz (x));}

void update_leds(void);
void set_midi_mode(midi_mode_t mode);
float config_but(int but, int type, float adjust);

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
        int tuning_color = 0x00aa00;
        int cur_tuning = 0;
        float tuning_note_offset = 0;
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
        float bend_sensitivity = 1.0f;
        float y_sensitivity = 1.0f;
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
            set_volume(volume_linear);
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
                altmode |= 1;
            } else {
                altmode &= 2;
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
            // unset cur_tuning so the tuning will be reset on tuning switch
            cur_tuning = -1;
            // keep generator within syntonic continuum range
            notegen1 = clamp(g, 6.85714285714286f, 7.2f);

            // base color depending on tuning system
            if (notegen1 < 6.94)       {tuning_color = 0x550055;}
            else if (notegen1 < 6.957) {tuning_color = 0x0000aa;}
            else if (notegen1 < 6.984) {tuning_color = 0x005555;}
            else if (notegen1 < 7.010) {tuning_color = 0x00aa00;}
            else if (notegen1 < 7.03)  {tuning_color = 0x555500;}
            else                       {tuning_color = 0xaa0000;}

            update_leds();
        }

        void reset_note_offsets(void) {
            for (int n = 0; n < 61; n++) {
                int but = button_number_map[n];
                buttons[but].tuning_note_offset = 0.0f;
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

        void load_tuning(int n) {
            if (cur_tuning == n) return;
            if (n == 0) {
                // tuning 0 hard coded to 12tet
                notegen0 = 12.0f;
                set_notegen1(7.0f);
                reset_note_offsets();
                note_offset = tuning_note_offset = 0.0f;
                led_rgb(tuning_color);
                cur_tuning = n;
                return;
            }
            float f;
            CC_ALIGN(8) char key[] = "fT0fifth";
            key[2] = '0' + n;
            f = getConfigFloat(key);
            if (f == CONFIG_UNDEFINED) f = 700.0f;
            notegen1 = f / 100;
            strset(key, 3, "oct  ");
            f = getConfigFloat(key);
            if (f == CONFIG_UNDEFINED) f = 1200.0f;
            notegen0 = f / 100;
            strset(key, 3, "off  ");
            f = getConfigFloat(key);
            if (f == CONFIG_UNDEFINED) f = 0.0f;
            note_offset = tuning_note_offset = f / 100;
            for (int n = 0; n < 61; n++) {
                put_button_name(n, &key[3]);
                int but = button_number_map[n];
                f = getConfigFloat(key);
                if (f == CONFIG_UNDEFINED) f = 0.0f;
                buttons[but].tuning_note_offset = f / 100;
            }
            key[0] = 'h';
            strset(key, 3, "color");
            tuning_color = getConfigHex(key);
            led_rgb(tuning_color);
            cur_tuning = n;
        }

        /* Rotate the layout 180 degrees */
        void flip(void) {
            for (int n = 0; n < BUTTONCOUNT; n++) {
                buttons[n].coord0 = -buttons[n].coord0;
                buttons[n].coord1 = -buttons[n].coord1;
                // invert midi note number
                buttons[n].midinote_base = -buttons[n].midinote_base;
            }
        }

        void button_message(int but, float* msg) {
            static float old_angle = -1000.0f;
            static bool nudged = false;
            static systime_t next_knobchange;
            // process button message and send osc messages
            buttons[but].message(msg);

            // handle alternative functions of note buttons
            if ((altmode == 1 && buttons[but].state == STATE_OFF)
                || (buttons[but].state == STATE_ALT)) {
                // only handle on new press
                if (buttons[but].state == STATE_OFF && buttons[but].pres > 0.05) {
                    buttons[but].state = STATE_ALT;
                    altmode |= 2;
                    config_but(but, 0, 0);
                } else if (buttons[but].pres == 0.0) {
                    buttons[but].state = STATE_OFF;
                    altmode &= 1;
                    old_angle = -1000.0f;
                    nudged = false;
                    if (!altmode) update_leds();
                }
                // handle knob mode
                float a = pow2(buttons[but].but_x) + pow2(buttons[but].but_y);
                if (a > pow2(0.5f)) {
                    float angle = atan2f(buttons[but].but_x, buttons[but].but_y)*(8/3.1415926536) + 8.0f;
                    if (old_angle == -1000.0f) {
                        // nudge up or down
                        if (buttons[but].but_x < 0.3 && buttons[but].but_x > -0.3) {
                            if (buttons[but].but_y > 0) {
                                config_but(but, 1, 1.0f);
                            } else {
                                config_but(but, 1, -1.0f);
                            }
                            nudged = true;
                        }
                        old_angle = angle;
                    } else {
                        float adjust = angle - old_angle;
                        if (adjust > 8) adjust -= 16;
                        else if (adjust < -8) adjust += 16;
                        if (nudged) {
                            if (fabsf(adjust) > 4.0f) {
                                nudged = false;
                                old_angle = angle;
                            }
                        } else {
                            if (chVTGetSystemTime() > next_knobchange) {
                                adjust = config_but(but, 2, adjust);
                                old_angle = angle - clamp(adjust, -4, 4);
                                next_knobchange = chVTGetSystemTime() + TIME_MS2I(20);
                            }
                        }
                    }
                } else if (a < pow2(0.2f)) {
                    old_angle = -1000.0f;
                    nudged = false;
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
                        y = y * y_sensitivity;
                        d = (buttons[0].last_tilt > (64 + y * 64)) * 0.5 - 0.25;
                        int tilt = 64 + y * 64 + 0.5 + d;
                        tilt = clamp(tilt, 0, 127);
                        if (tilt != buttons[0].last_tilt) {
                            if (config.midi_y < 120 && y_sensitivity != 0.0f) {
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
                if (config.midinote_mode == MIDINOTE_MODE_DEFAULT) {
                    buttons[but].midinote = buttons[but].midinote_base + (int)(start_note_offset + 0.5);
                } else if (config.midinote_mode == MIDINOTE_MODE_TUNING) {
                    buttons[but].midinote = (int)(notegen0 * buttons[but].coord0 +
                                                  notegen1 * buttons[but].coord1 +
                                                  start_note_offset +
                                                  note_offset + 0.5);
                } else if (config.midinote_mode == MIDINOTE_MODE_BUTTON) {
                    buttons[but].midinote = 17 * buttons[but].coord0 + 10 * buttons[but].coord1 + 30;
                }
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
            float y = clamp(buttons[but].but_y * y_sensitivity, -1.0f, 1.0f);
#ifdef USE_INTERNAL_SYNTH
            int voice = buttons[but].voice;
            *(synth_interface->note[voice])  = buttons[but].note + pb;
            *(synth_interface->pres[voice])  = presf;
            *(synth_interface->vpres[voice]) = velof;
            *(synth_interface->but_x[voice]) = buttons[but].but_x;
            *(synth_interface->but_y[voice]) = y;
#endif

#ifdef USE_MIDI_OUT
            float d; // calculate direction for hysteresis
            // TODO: fix hysteresis!
            // TODO: last_... values per midi channel instead of per button
            d = (buttons[but].last_pres > (presf)) * 0.5 - 0.25;
            int pres = presf * 127 + 0.5 + d;
            pres = clamp(pres, 0, 127);

            d = (buttons[but].last_tilt > (64 + y * 64)) * 0.5 - 0.25;
            int tilt = 64 + y * 64 + 0.5 + d;
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
                if (dis.altmode & 1) {
                    // transpose reset
                    dis.set_note_offset(62);
                } else {
                    // cancel last transpose and enable free transpose
                    dis.set_note_offset(note_offset_old);
                    dis.set_free_transpose_mode(1);
                }
            } else {
                note_offset_old = dis.start_note_offset;
                float dif = dis.notegen0;
                if (dis.altmode & 1) dif = 1.0f;
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

void load_preset(int n) {
    int i;
    float f;
    const char* s;
    bool sendcfg = false;
    CC_ALIGN(8) char key[] = "hP0color";
    key[2] = '0' + n;
    uint32_t led = getConfigHex(key);
    led_rgb(led);

    key[0] = 'i';
    strset(key, 3, "Mint ");
    i = getConfigInt(key);
    if (i >= 1 && i <= 127) {
        config.message_interval = i;
    }

    strset(key, 3, "Mmint");
    i = getConfigInt(key);
    if (i >= 0 && i <= 127) {
        config.send_motion_interval = i;
    }

    key[0] = 's';
    strset(key, 3, "Mmode");
    s = getConfigSetting(key);
    if (cmp8(s, "mpe     ")) {
        set_midi_mode(MIDI_MODE_MPE);
        sendcfg = true;
    } else if (cmp8(s, "normal  ")) {
        set_midi_mode(MIDI_MODE_POLY);
        sendcfg = true;
    } else if (cmp8(s, "mono    ")) {
        set_midi_mode(MIDI_MODE_MONO);
        sendcfg = true;
    }

    strset(key, 3, "Mnote");
    s = getConfigSetting(key);
    if (cmp8(s, "default ")) {
        config.midinote_mode = MIDINOTE_MODE_DEFAULT;
    } else if (cmp8(s, "tuning  ")) {
        config.midinote_mode = MIDINOTE_MODE_TUNING;
    } else if (cmp8(s, "button  ")) {
        config.midinote_mode = MIDINOTE_MODE_BUTTON;
    }

    //strset(key, 3, "jack2");
    //s = getConfigSetting(key);
    //if (cmp8(s, "pedal   ")) {
    //    aux_jack_switch_mode(JACK2_MODE_PEDAL);
    //} else if (cmp8(s, "linein  ")) {
    //    aux_jack_switch_mode(JACK2_MODE_LINEIN);
    //} else if (cmp8(s, "midi    ")) {
    //    aux_jack_switch_mode(JACK2_MODE_MIDI);
    //}

    key[0] = 'i';
    strset(key, 3, "tunin");
    i = getConfigInt(key);
    if (i >= 0 && i <= 8) {
        dis.load_tuning(i);
        config.message_interval = i;
    }

    key[0] = 'f';
    strset(key, 3, "Toff ");
    f = getConfigFloat(key);
    if (f >= -4800.0f && f <= 4800.0f) {
        dis.note_offset = dis.tuning_note_offset + f / 100.0f;
    }

    key[0] = 'i';
    strset(key, 3, "Mpres");
    i = getConfigInt(key);
    if (i >= 0 && i <= 127) {
        if (config.midi_mode == MIDI_MODE_POLY) {
            config.midi_pres = i;
        } else {
            config.mpe_pres = i;
        }
    }

    strset(key, 3, "Mx   ");
    i = getConfigInt(key);
    if (i >= 0 && i <= 127) {
        if (config.midi_mode == MIDI_MODE_POLY) {
            config.midi_x = i;
        } else {
            config.mpe_x = i;
        }
    }

    strset(key, 3, "My   ");
    i = getConfigInt(key);
    if (i >= 0 && i <= 127) {
        if (config.midi_mode == MIDI_MODE_POLY) {
            config.midi_y = i;
        } else {
            config.mpe_y = i;
        }
    }

    strset(key, 3, "Mvelo");
    i = getConfigInt(key);
    if (i >= 0 && i <= 127) {
        config.mpe_contvelo = i;
    }

    strset(key, 3, "MChan");
    i = getConfigInt(key);
    if (i >= 1 && i <= 16) {
        dis.midi_channel_offset = i - 1;
        sendcfg = true;
    }

    strset(key, 3, "MPEpb");
    i = getConfigInt(key);
    if (i >= 1 && i <= 127) {
        dis.midi_bend_range = i;
        sendcfg = true;
    }

    strset(key, 3, "voice");
    i = getConfigInt(key);
    if (i >= 1 && i <= MAX_VOICECOUNT && config.midi_mode == MIDI_MODE_MPE) {
        dis.voicecount = i;
        sendcfg = true;
    }

    key[0] = 'f';
    strset(key, 3, "offse");
    f = getConfigFloat(key);
    if (f >= 0.0f && f <= 1.0f) {
        config.zero_offset = f;
    }

    strset(key, 3, "bendS");
    f = getConfigFloat(key);
    if (f >= -10.0f && f <= 10.0f) {
        dis.bend_sensitivity = f * 2;
    }

    strset(key, 3, "presS");
    f = getConfigFloat(key);
    if (f >= 0.0f && f <= 10.0f) {
        dis.pres_sensitivity = f;
    }

    strset(key, 3, "veloS");
    f = getConfigFloat(key);
    if (f >= 0.0f && f <= 10.0f) {
        dis.velo_sensitivity = f;
    }

    strset(key, 3, "tiltS");
    f = getConfigFloat(key);
    if (f >= -10.0f && f <= 10.0f) {
        dis.y_sensitivity = f;
    }

    strset(key, 3, "volum");
    f = getConfigFloat(key);
    if (f >= 0.0f && f < 255.0f) {
        set_volume(f);
    }

    if (sendcfg) {
        midi_config();
    }

    key[0] = 'i';
    strset(key, 3, "Mpgm ");
    i = getConfigInt(key);
    if (i >= 0 && i <= 127) {
        if (config.midi_mode != MIDI_MODE_POLY && dis.midi_channel_offset == 1) {
            MidiSend2(MIDI_PROGRAM_CHANGE, i);
        }
        for (int n = 0; n < dis.voicecount; n++) {
            MidiSend2(MIDI_PROGRAM_CHANGE | (dis.midi_channel_offset + n), i);
        }
    }

    led_rgb(led);
}

#ifdef OLD_CONFIG_LAYOUT
float config_but(int but, int type, float adjust) {
    if (type > 0) return 0;
    switch (but) {
        // row 1:  0  2  4
        case (0): {
            set_midi_mode(MIDI_MODE_MPE);
        } return 0;
        case (2): {
            set_midi_mode(MIDI_MODE_POLY);
        } return 0;
        case (4): {
            set_midi_mode(MIDI_MODE_MONO);
        } return 0;
        // row 2:  1  3  5  7  9 11
        case (1): {
            config.message_interval = 1;
            ws2812_write_led(0, 1, 12, 0);
        } return 0;
        case (3): {
            config.message_interval = 10;
            ws2812_write_led(0, 4, 2, 0);
        } return 0;
        case (5): {
            config.send_motion_interval = 1;
            ws2812_write_led(0, 1, 12, 0);
        } return 0;
        case (7): {
            config.send_motion_interval = 10;
            ws2812_write_led(0, 4, 2, 0);
        } return 0;
        case (9): {
            config.send_motion_interval = 0;
            *(dis.synth_interface->acc_abs) = 1.0f;
            *(dis.synth_interface->acc_x) = 0.0f;
            *(dis.synth_interface->acc_y) = 0.0f;
            *(dis.synth_interface->acc_z) = 0.0f;
            *(dis.synth_interface->rot_x) = 0.0f;
            *(dis.synth_interface->rot_y) = 0.0f;
            *(dis.synth_interface->rot_z) = 0.0f;
            ws2812_write_led(0, 3, 0, 0);
        } return 0;
        case (11): { // debug setting for easy testing
            if (dis.portamento & 1) {
                if (config.debug) {
                    config.debug = 0;
                    // led_updown(0xf000);
                } else {
                    config.debug = 1;
                    // led_updown(0xffff);
                }
            } else {
                dis.flip();
            }
        } return 0;
        // row 3: 17 19 21  6  8 10 12 14 16
        case (17): {
            dis.pres_sensitivity = 0.67f;
            ws2812_write_led(0, 1, 1, 0);
        } return 0;
        case (19): {
            dis.pres_sensitivity = 1.0f;
            ws2812_write_led(0, 4, 4, 0);
        } return 0;
        case (21): {
            dis.pres_sensitivity = 1.5f;
            ws2812_write_led(0, 12, 12, 0);
        } return 0;
        case (6): {
            dis.velo_sensitivity = 0.67f;
            ws2812_write_led(0, 1, 0, 1);
        } return 0;
        case (8): {
            dis.velo_sensitivity = 1.0f;
            ws2812_write_led(0, 4, 0, 4);
        } return 0;
        case (10): {
            dis.velo_sensitivity = 1.5f;
            ws2812_write_led(0, 12, 0, 12);
        } return 0;
        case (12): {
            dis.bend_sensitivity = 0.0f;
            ws2812_write_led(0, 1, 0, 0);
        } return 0;
        case (14): {
            dis.bend_sensitivity = 0.25f;
            ws2812_write_led(0, 4, 0, 0);
        } return 0;
        case (16): {
            dis.bend_sensitivity = 0.5f;
            ws2812_write_led(0, 12, 0, 0);
        } return 0;
        // row 4: 18 20 22 24 26 28 13 15
        case (18): {
            config.zero_offset = 0 * ((1<<24) / 128);
            led_rgb3(14, 7, 0);
        } return 0;
        case (20): {
            config.zero_offset = 4 * ((1<<24) / 128);
            led_rgb3(56, 28, 0);
        } return 0;
        case (22): {
            config.zero_offset = 8 * ((1<<24) / 128);
            led_rgb3(168, 84, 0);
        } return 0;
        // Load config preset
        // row 5: 34 36 38 23 25 27 29 31 33
        case (34): { // set 12tet tuning
            dis.load_tuning(0);
        } return 0;
        case (36): {
            dis.load_tuning(1);
        } return 0;
        case (38): {
            dis.load_tuning(2);
        } return 0;
        case (23): {
            dis.load_tuning(3);
        } return 0;
        case (25): {
            dis.load_tuning(4);
        } return 0;
        case (27): {
            dis.load_tuning(5);
        } return 0;
        case (29): {
            dis.load_tuning(6);
        } return 0;
        case (31): {
            dis.load_tuning(7);
        } return 0;
        case (33): {
            dis.load_tuning(8);
        } return 0;
        // row 6: 35 37 39 41 43 45 30 32
        case (35): {
            load_preset(1);
        } return 0;
        case (37): {
            load_preset(2);
        } return 0;
        case (39): {
            load_preset(3);
        } return 0;
        case (41): {
            load_preset(4);
        } return 0;
        case (43): {
            load_preset(5);
        } return 0;
        case (45): {
            load_preset(6);
        } return 0;
        case (30): {
            load_preset(7);
        } return 0;
        case (32): {
            load_preset(8);
        } return 0;
        // row 7: 51 53 55 40 42 44 46 48 50
        case (51): {
            MidiSend3(MIDI_CONTROL_CHANGE,
                                MIDI_C_MAIN_VOLUME, 7);
            set_volume(7);
            ws2812_write_led(0, 1, 1, 1);
        } return 0;
        case (53): {
            MidiSend3(MIDI_CONTROL_CHANGE,
                                MIDI_C_MAIN_VOLUME, 15);
            set_volume(15);
            ws2812_write_led(0, 2, 2, 2);
        } return 0;
        case (55): {
            MidiSend3(MIDI_CONTROL_CHANGE,
                                MIDI_C_MAIN_VOLUME, 31);
            set_volume(31);
            ws2812_write_led(0, 3, 3, 3);
        } return 0;
        case (40): {
            MidiSend3(MIDI_CONTROL_CHANGE,
                                MIDI_C_MAIN_VOLUME, 47);
            set_volume(47);
            ws2812_write_led(0, 4, 4, 4);
        } return 0;
        case (42): {
            MidiSend3(MIDI_CONTROL_CHANGE,
                                MIDI_C_MAIN_VOLUME, 63);
            set_volume(63);
            ws2812_write_led(0, 6, 6, 6);
        } return 0;
        case (44): {
            MidiSend3(MIDI_CONTROL_CHANGE,
                                MIDI_C_MAIN_VOLUME, 79);
            set_volume(79);
            ws2812_write_led(0, 8, 8, 8);
        } return 0;
        case (46): {
            MidiSend3(MIDI_CONTROL_CHANGE,
                                MIDI_C_MAIN_VOLUME, 95);
            set_volume(95);
            ws2812_write_led(0, 10, 10, 10);
        } return 0;
        case (48): {
            MidiSend3(MIDI_CONTROL_CHANGE,
                                MIDI_C_MAIN_VOLUME, 111);
            set_volume(111);
            ws2812_write_led(0, 14, 14, 14);
        } return 0;
        case (50): {
            MidiSend3(MIDI_CONTROL_CHANGE,
                                MIDI_C_MAIN_VOLUME, 127);
            set_volume(127);
            ws2812_write_led(0, 18, 18, 18);
        } return 0;

        // row 8:       56 58 60 62 47 49
        // row 9:                   63 65 67
        case (63): { // panic/request config
#ifdef USE_MIDI_OUT
            MidiSend3(MIDI_CONTROL_CHANGE,
                                MIDI_C_ALL_NOTES_OFF, 0);
            ws2812_write_led(0, 16, 0, 0);
#endif
        } return 0;
        case (65): { // send config
#ifdef USE_MIDI_OUT
            // TODO: send config
            ws2812_write_led(0, 16, 0, 0);
#endif
        } return 0;
        case (67): { // system reset
            led_rgb(0xff0000);
            chThdSleepMilliseconds(500);
            NVIC_SystemReset();
        } return 0;
        default: return 0;
    }
}
#else // !OLD_CONFIG_LAYOUT

/*
 * button functions in alt mode - with settings button pressed.
 *
 * type can be 0: initial press, 1: nudge up/down, 2: circular adjust
 *
 * returns part of adjust that's not used and hence needs to be saved for next call
 */
float config_but(int but, int type, float adjust) {
    switch (but) {
    // row 1:  0  2  4
    case (0): // MPE MIDI mode, knob: number of MPE channels
        if (type == 0) {
            if (config.midi_mode != MIDI_MODE_MPE) set_midi_mode(MIDI_MODE_MPE);
            led_updown_dial(dis.voicecount);
            midi_config();
        } else {
            int a = (int)adjust;
            if (type == 2) a = a / 2;
            if (a != 0) {
                dis.voicecount = clamp(dis.voicecount + a, 1, 15);
                led_updown_dial(dis.voicecount);
                midi_config();
            }
            if (type == 2) a = a * 2;
            return adjust - a;
        } return 0;
    case (2): // Normal MIDI mode
        if (type == 0) {
            if (config.midi_mode != MIDI_MODE_POLY) set_midi_mode(MIDI_MODE_POLY);
            led_updown_dial(dis.midi_channel_offset + 1);
            midi_config();
        } else {
            int a = (int)adjust;
            if (type == 2) a = a / 2;
            if (a != 0) {
                dis.midi_channel_offset = clamp(dis.midi_channel_offset + a, 0, 15);
                led_updown_dial(dis.midi_channel_offset + 1);
                midi_config();
            }
            if (type == 2) a = a * 2;
            return adjust - a;
        } return 0;
    case (4): // Mono (glissando) MIDI mode
        if (type == 0) {
            if (config.midi_mode != MIDI_MODE_MONO) set_midi_mode(MIDI_MODE_MONO);
            led_updown_dial(dis.midi_channel_offset + 1);
            midi_config();
        } else {
            int a = (int)adjust;
            if (type == 2) a = a / 2;
            if (a != 0) {
                dis.midi_channel_offset = clamp(dis.midi_channel_offset + a, 0, 15);
                led_updown_dial(dis.midi_channel_offset + 1);
                midi_config();
            }
            if (type == 2) a = a * 2;
            return adjust - a;
        } return 0;
    // row 2:  1  3  5  7  9 11
    case (11): // debug setting for easy testing
        if (type == 0) {
            if (dis.portamento & 1) {
                if (config.debug) {
                    config.debug = 0;
                    // led_updown(0xf000);
                } else {
                    config.debug = 1;
                    // led_updown(0xffff);
                }
            } else {
                dis.flip();
            }
        } return 0;
    // row 3: 17 19 21  6  8 10 12 14 16
    case (17): { // knob: key detection threshold
        int a = adjust * ((1 << 24) / 256);
        if (type > 0) {
            config.zero_offset = clamp(config.zero_offset + a, 0, (1 << 24) / 16);
        }
        int dial = config.zero_offset / ((1 << 24) / 256);
        led_updown_dial(dial);
        led_rgb3(dial * 10, dial * 5, 0);
        return 0; // TODO: sticky ends
        }
    case (19): { // knob: pres sensitivity
        float rem = 0;
        if (type > 0) {
            dis.pres_sensitivity = clamp_rem(dis.pres_sensitivity + adjust * (1.0f/8.0f), 0, 3.0f, &rem);
        }
        led_updown_dial(dis.pres_sensitivity * 8 + 0.9f);
        led_rgb3(dis.pres_sensitivity * 64.0f, dis.pres_sensitivity * 64.0f, 0);
        return rem * 8;
        }
    case (21): { // knob: velo sensitivity
        float rem = 0;
        if (type > 0) {
            dis.velo_sensitivity = clamp_rem(dis.velo_sensitivity + adjust * (1.0f/8.0f), 0, 2.0f, &rem);
        }
        led_updown_dial(min((int)(dis.velo_sensitivity * 8 + 0.9f), 15));
        led_rgb3(dis.velo_sensitivity * 64.0f, 0, dis.velo_sensitivity * 64.0f);
        return rem * 8;
        }
    case (6): { // knob: pitchbend range
        float rem = 0;
        if (type > 0) {
            dis.bend_sensitivity = clamp_rem(dis.bend_sensitivity + adjust * 0.25f, 0, 4.0f, &rem);
        }
        led_updown_dial(min((int)(dis.bend_sensitivity * 4 + 0.9f), 15));
        led_rgb3(dis.bend_sensitivity * 64.0f, 0, 0);
        return rem * 4;
        }
    case (8): { // knob: y sensitivity (TODO: flip?)
        float rem = 0;
        if (type > 0) {
            dis.y_sensitivity = clamp_rem(dis.y_sensitivity + adjust * (1.0f/8.0f), 0, 3.0f, &rem);
        }
        led_updown_dial(dis.y_sensitivity * 8 + 0.5f);
        led_rgb3(dis.y_sensitivity * 32.0f, 0, dis.y_sensitivity * 64.0f);
        return rem * 8;
        }
    // row 4: 18 20 22 24 26 28 13 15
    // row 5: 34 36 38 23 25 27 29 31 33
    case (34): // set 12tet tuning, knob: tuning offset
        if (type == 0) {
            dis.load_tuning(0);
        } else {
            dis.note_offset += adjust * (1.0f/16.0f/2.0f);
        }
        led_updown_dial(dis.note_offset * 16 + 0.5f);
        return 0;
    case (36): // load tuning 1, knob: tuning offset
        if (type == 0) {
            dis.load_tuning(1);
        } else {
            dis.note_offset += adjust * (1.0f/16.0f/2.0f);
        }
        led_updown_dial(dis.note_offset * 16 + 0.5f);
        return 0;
    case (38): // load tuning 2, knob: tuning offset
        if (type == 0) {
            dis.load_tuning(2);
        } else {
            dis.note_offset += adjust * (1.0f/16.0f/2.0f);
        }
        led_updown_dial(dis.note_offset * 16 + 0.5f);
        return 0;
    case (23): // load tuning 3, knob: tuning offset
        if (type == 0) {
            dis.load_tuning(3);
        } else {
            dis.note_offset += adjust * (1.0f/16.0f/2.0f);
        }
        led_updown_dial(dis.note_offset * 16 + 0.5f);
        return 0;
    case (25): // load tuning 4, knob: tuning offset
        if (type == 0) {
            dis.load_tuning(4);
        } else {
            dis.note_offset += adjust * (1.0f/16.0f/2.0f);
        }
        led_updown_dial(dis.note_offset * 16 + 0.5f);
        return 0;
    case (27): // load tuning 5, knob: tuning offset
        if (type == 0) {
            dis.load_tuning(5);
        } else {
            dis.note_offset += adjust * (1.0f/16.0f/2.0f);
        }
        led_updown_dial(dis.note_offset * 16 + 0.5f);
        return 0;
    case (29): // load tuning 6, knob: tuning offset
        if (type == 0) {
            dis.load_tuning(6);
        } else {
            dis.note_offset += adjust * (1.0f/16.0f/2.0f);
        }
        led_updown_dial(dis.note_offset * 16 + 0.5f);
        return 0;
    case (31): // load tuning 7, knob: tuning offset
        if (type == 0) {
            dis.load_tuning(7);
        } else {
            dis.note_offset += adjust * (1.0f/16.0f/2.0f);
        }
        led_updown_dial(dis.note_offset * 16 + 0.5f);
        return 0;
    case (33): // load tuning 8, knob: tuning offset
        if (type == 0) {
            dis.load_tuning(8);
        } else {
            dis.note_offset += adjust * (1.0f/16.0f/2.0f);
        }
        led_updown_dial(dis.note_offset * 16 + 0.5f);
        return 0;
        // row 6: 35 37 39 41 43 45 30 32
    case (35):
        if (type == 0) {
            load_preset(1);
        } return 0;
    case (37):
        if (type == 0) {
            load_preset(2);
        } return 0;
    case (39):
        if (type == 0) {
            load_preset(3);
        } return 0;
    case (41):
        if (type == 0) {
            load_preset(4);
        } return 0;
    case (43):
        if (type == 0) {
            load_preset(5);
        } return 0;
    case (45):
        if (type == 0) {
            load_preset(6);
        } return 0;
    case (30):
        if (type == 0) {
            load_preset(7);
        } return 0;
    case (32):
        if (type == 0) {
            load_preset(8);
        } return 0;
    // row 7: 51 53 55 40 42 44 46 48 50
    case (51): { // knob: Volume (CC7)
        float rem = 0;
        if (type > 0) {
            float vol = clamp_rem(volume_linear + adjust * 8.0f, 0, 127.0f, &rem);
            if ((int)(volume_linear + 0.5f) != (int)(vol + 0.5f)) {
                MidiSend3(MIDI_CONTROL_CHANGE, MIDI_C_MAIN_VOLUME, vol + 0.5f);
            }
            set_volume(vol);
        }
        led_rgb3(volume_linear*2, volume_linear*2, volume_linear*2);
        led_updown_dial((int)volume_linear/8);
        return rem * (1.0f/8.0f);
        }
    // row 8:       56 58 60 62 47 49
    case (56): { // knob: MPE pitchbend range
        int a = adjust;
        int cur = log2i(dis.midi_bend_range / 12);
        if (type > 0) {
            if (type == 2) a = a / 4;
            if (a != 0) {
                cur = clamp(cur + a, 0, 3);
                dis.midi_bend_range = 12 << cur;
                midi_config();
            }
            if (type == 2) a = a * 4;
        }
        led_rgb3(10 + 50*cur, 0, 0);
        led_updown_dial(2 + 2 * cur);
        return adjust - a;
        }
    case (58): { // knob: MIDI message interval
        int a = (int)adjust;
        if (type > 0) {
            config.message_interval = clamp(config.message_interval - a, 1, 15);
        }
        led_rgb3(164-8*config.message_interval, 164-8*config.message_interval, 0);
        led_updown_dial(16 - config.message_interval);
        return adjust - a;
        }
    // case (60): { // knob: MIDI motion message interval
    //     int a = (int)adjust;
    //     if (type > 0) {
    //         config.send_motion_interval = clamp(config.send_motion_interval - adjust, 0, 15);
    //         if (angle == 0) {
    //             config.send_motion_interval = 0;
    //             *(dis.synth_interface->acc_abs) = 1.0f;
    //             *(dis.synth_interface->acc_x) = 0.0f;
    //             *(dis.synth_interface->acc_y) = 0.0f;
    //             *(dis.synth_interface->acc_z) = 0.0f;
    //             *(dis.synth_interface->rot_x) = 0.0f;
    //             *(dis.synth_interface->rot_y) = 0.0f;
    //             *(dis.synth_interface->rot_z) = 0.0f;
    //         } else if (angle == 15) {
    //             config.send_motion_interval = 127;
    //         } else {
    //             config.send_motion_interval = angle;
    //         }
    //     }
    //     led_rgb3(164-8*config.send_motion_interval, 164-8*config.send_motion_interval, 0);
    //     led_updown_dial(16 - config.send_motion_interval);
    //     return adjust - a;
    //     }
    // row 9:                   63 65 67
    case (63): // panic/all notes off
        if (type == 0) {
#ifdef USE_MIDI_OUT
            for (int n = 0; n < (config.midi_mode == MIDI_MODE_POLY ? 1 : dis.voicecount); n++) {
                MidiSend3(MIDI_CONTROL_CHANGE | (dis.midi_channel_offset + n), MIDI_C_ALL_NOTES_OFF, 0);
            }
            led_rgb(0x342000);
#endif
        } return 0;
    case (65): // send MPE/pitch bend range config
        if (type == 0) {
            midi_config();
            led_rgb(0x300064);
        } return 0;
    case (67): // system reset
        if (type == 0) {
            led_rgb(0xff0000);
            chThdSleepMilliseconds(500);
            NVIC_SystemReset();
        } return 0;
    default: return 0;
    }
    return 0;
}
#endif // !OLD_CONFIG_LAYOUT

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
            case  7: set_volume(data2); break;
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
                dis.bend_sensitivity = (float)data2 * 0.05;
            } break;
            case MIDI_C_MONO: { // MPE/multitimbral/mono mode.
                // Set base channel based on channel the message is received
                if (data2 == 1) { // monophonic on one channel
                    set_midi_mode(MIDI_MODE_MONO);
                    dis.midi_channel_offset = channel;
                } else { // MPE like without master channel
                    set_midi_mode(MIDI_MODE_MPE);
                    dis.midi_channel_offset = channel;
                    if (data2 > 0) {
                        dis.voicecount = min(data2, 16 - channel);
                    }
                }
            } break;
            case MIDI_C_POLY: { // Polyphonic mode on the channel the message is received
                set_midi_mode(MIDI_MODE_POLY);
                dis.midi_channel_offset = channel;
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
            dis.midi_channel_offset = 1;
            dis.voicecount = 1;
            dis.portamento = 2; // 2 keeps led from brightening
            ws2812_write_led(0, 0, 1, 18);
        } break;
    }
}

void midi_config(void) {
#ifdef USE_MIDI_OUT
    if (config.midi_mode == MIDI_MODE_MPE) {
        if (dis.midi_channel_offset == 1) {
            // valid MPE should be on channel 1 (or 16 which is not implemented)
            MidiSend3(MIDI_CONTROL_CHANGE, MIDI_C_RPN_MSB, 0);
            MidiSend3(MIDI_CONTROL_CHANGE, MIDI_C_RPN_LSB, 6); // mpe set voicecount
            MidiSend3(MIDI_CONTROL_CHANGE, MIDI_C_DATA_ENTRY, dis.voicecount);
            MidiSend3(MIDI_CONTROL_CHANGE, MIDI_C_RPN_MSB, 0x7f); // reset RPN
            MidiSend3(MIDI_CONTROL_CHANGE, MIDI_C_RPN_LSB, 0x7f);
        }
        for (int n = 0; n < dis.voicecount; n++) {
            // send bend range on all channels to be more compatible
            MidiSend3(MIDI_CONTROL_CHANGE | (dis.midi_channel_offset + n), MIDI_C_RPN_MSB, 0);
            MidiSend3(MIDI_CONTROL_CHANGE | (dis.midi_channel_offset + n), MIDI_C_RPN_LSB, 0); // bend range
            MidiSend3(MIDI_CONTROL_CHANGE | (dis.midi_channel_offset + n), MIDI_C_DATA_ENTRY, dis.midi_bend_range);
            MidiSend3(MIDI_CONTROL_CHANGE | (dis.midi_channel_offset + n), MIDI_C_RPN_MSB, 0x7f); // reset RPN
            MidiSend3(MIDI_CONTROL_CHANGE | (dis.midi_channel_offset + n), MIDI_C_RPN_LSB, 0x7f);
        }
    } else if (config.midi_mode == MIDI_MODE_MONO) {
        MidiSend3(MIDI_CONTROL_CHANGE | dis.midi_channel_offset, MIDI_C_RPN_MSB, 0);
        MidiSend3(MIDI_CONTROL_CHANGE | dis.midi_channel_offset, MIDI_C_RPN_LSB, 0); // bend range
        MidiSend3(MIDI_CONTROL_CHANGE | dis.midi_channel_offset, MIDI_C_DATA_ENTRY, dis.midi_bend_range);
        MidiSend3(MIDI_CONTROL_CHANGE | dis.midi_channel_offset, MIDI_C_RPN_MSB, 0x7f); // reset RPN
        MidiSend3(MIDI_CONTROL_CHANGE | dis.midi_channel_offset, MIDI_C_RPN_LSB, 0x7f);
    } else if (config.midi_mode == MIDI_MODE_POLY) {
        MidiSend3(MIDI_CONTROL_CHANGE | dis.midi_channel_offset, MIDI_C_RPN_MSB, 0);
        MidiSend3(MIDI_CONTROL_CHANGE | dis.midi_channel_offset, MIDI_C_RPN_LSB, 0); // bend range
        MidiSend3(MIDI_CONTROL_CHANGE | dis.midi_channel_offset, MIDI_C_DATA_ENTRY, 2); // in poly mode pitch bend range is 2 semitones
        MidiSend3(MIDI_CONTROL_CHANGE | dis.midi_channel_offset, MIDI_C_RPN_MSB, 0x7f); // reset RPN
        MidiSend3(MIDI_CONTROL_CHANGE | dis.midi_channel_offset, MIDI_C_RPN_LSB, 0x7f);
    }
#endif
}

void synth_control_init(void) {
    int s = getConfigInt("iGoct   ");
    if (s >= -2 && s <= 2) {
        dis.change_note_offset(s * dis.notegen0);
    }
    s = getConfigInt("iGmotion");
    if (s >= 0 && s <= 127) {
        config.send_motion_interval = s;
    }
}

void clear_dead_notes(void) {
    dis.clear_dead_notes();
}

void synth_tick(void) {
    dis.tick();
}

void update_leds(void) {
    // if buttons in alt mode don't update leds
    if (dis.altmode & 2) return;

    uint8_t r, g, b;
    r = dis.tuning_color >> 16 & 0xff;
    g = dis.tuning_color >>  8 & 0xff;
    b = dis.tuning_color >>  0 & 0xff;

    int m = 64 + 16 * (dis.altmode + (dis.portamento & 1));
    led_rgb3(r * m / 64, g * m / 64, b * m / 64);

    int note_offset = (int)(dis.start_note_offset + 0.5f) - 62;
    if      (note_offset <= -24) led_updown(0xff00);
    else if (note_offset <  -12) led_updown(0xaf00);
    else if (note_offset <= -12) led_updown(0x0f00);
    else if (note_offset <    0) led_updown(0x0a00);
    else if (note_offset >=  24) led_updown(0x00ff);
    else if (note_offset >   12) led_updown(0x00fa);
    else if (note_offset >=  12) led_updown(0x00f0);
    else if (note_offset >    0) led_updown(0x00a0);
    else                         led_updown(0x0000);
}
