
#include <math.h>

#include "ch.h"
#include "hal.h"

extern "C" {
    #include "synth.h"
    #include "ws2812.h"
}

#include "config.h"
#include "striso.h"
#include "midi_usb.h"

#define BUTTONCOUNT 68
#define MAX_PORTAMENTO_BUTTONS 8

#define VOL_TICK (0.0005) // (1.0 / (SAMPLINGFREQ / CHANNEL_BUFFER_SIZE) / 0.5) // decay time of estimated volume
#define VOL_TICK_FACT (0.998) // 0.5**(1/(SAMPLINGFREQ / CHANNEL_BUFFER_SIZE)/0.1)
#define CLEAR_TIMER MS2ST(500) // interval to clear dead notes

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

typedef enum {
    STATE_OFF = 0,
    STATE_ON = 1,
    STATE_PORTAMENTO = 2,
} button_state_t;

class Button {
    public:
        int coord0;
        int coord1;
        float signals[6] = {0,0,0,0,0,0};
        float note;
        int midinote;
        int midinote_base;
        float pres;
        float vpres;
        float vpres_prev;
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
            float fact = 1.0/(m+CENTERTEND/m-CENTERTEND);
            but_x = (signals[2] - signals[0]) * fact;
            but_y = (0.5 * (signals[0] + signals[2]) - signals[1]) * fact;
        }
};

class Instrument {
    public:
        Button buttons[BUTTONCOUNT];
        int voices[VOICECOUNT];
        int portamento_buttons[MAX_PORTAMENTO_BUTTONS];
        float notegen0 = 12.00;
        float notegen1 = 7.00;
        float note_offset = 62;
        float min_note_offset = 32;
        float max_note_offset = 92;
        int portamento = 0;
        int last_button = 0;
        int port_voice = -1;
        int portamento_button = -1;
        synth_interface_t* synth_interface;
        int midi_channel_offset = 1;
        float midi_bend_range = 48.0;
        float bend_sensitivity = 0.5;

        Instrument(int* c0, int* c1, int n_buttons, synth_interface_t* si) {
            int n;
            for (n = 0; n < n_buttons; n++) {
                buttons[n].coord0 = c0[n];
                buttons[n].coord1 = c1[n];
                // calculate note number
                buttons[n].note = note_offset +
                                  notegen0 * buttons[n].coord0 +
                                  notegen1 * buttons[n].coord1;
                buttons[n].midinote_base = (int)(buttons[n].note + 0.5) - (int)note_offset;
                buttons[n].midinote = buttons[n].midinote_base;
            }
            for (n = 0; n < VOICECOUNT; n++) {
                voices[n] = -1;
            }
            for (n = 0; n < MAX_PORTAMENTO_BUTTONS; n++) {
                portamento_buttons[n] = -1;
            }
            synth_interface = si;
        }

        void set_portamento(int p) {
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

        int change_note_offset(float offset) {
            float n = note_offset + offset;
            if (n >= min_note_offset && n <= max_note_offset) {
                note_offset = n;
                return 0;
            }
            return 1;
        }

        void button_message(int but, float* msg) {
            // process button message and send osc messages
            buttons[but].message(msg);

            // Note on detection
            if (buttons[but].state == STATE_OFF && buttons[but].pres > 0.0) {
                // calculate midinote only at note on
                buttons[but].midinote = buttons[but].midinote_base + (int)(note_offset + 0.5);
                
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
                                ws2812_write_led(0, 0, 0, 20);
                                break;
                            }
                        }
                    }
                } else {
                    get_voice(but);
                }
            }

            if (buttons[but].state) {
                // calculate note number
                buttons[but].note = note_offset +
                                    notegen0 * buttons[but].coord0 +
                                    notegen1 * buttons[but].coord1;
                buttons[but].calculate();
                buttons[but].timer = chTimeNow() + CLEAR_TIMER;

                if (but == portamento_button) {
                    // calculate average of portamento buttons
                    float temp_pres = buttons[but].pres; // save pres for note off detection
                    
                    float sw = buttons[but].pres;
                    float note  = sw * buttons[but].note;
                    float vpres = sw * buttons[but].vpres;
                    float but_x = sw * buttons[but].but_x;
                    float but_y = sw * buttons[but].but_y;
                    for (int n = 0; n < MAX_PORTAMENTO_BUTTONS; n++) {
                        int b = portamento_buttons[n];
                        if (b >= 0) {
                            float w = buttons[b].pres;
                            sw += w;
                            vpres += buttons[b].vpres;
                            note  += w * buttons[b].note;
                            but_x += w * buttons[b].but_x;
                            but_y += w * buttons[b].but_y;
                        }
                    }
                    buttons[but].pres = sw;
                    if (sw == 0) sw = 1;
                    if (note > 0.1) {
                        buttons[but].note = note / sw;
                    }
                    buttons[but].vpres = vpres / sw;
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
                int velo = 1 - buttons[but].vpres * 127;
                if (velo > 127) velo = 127;
                else if (velo < 0) velo = 0;
                midi_usb_MidiSend3(1, MIDI_NOTE_OFF | (midi_channel_offset + buttons[but].voice),
                                   buttons[but].midinote, velo);
#endif
            }
        }

        int get_voice(int but) {
            int voice = -1;
            int n;
            float min_vol = buttons[but].vol;// * 0.9 - 0.05; // * factor for hysteresis
            for (n=0; n<VOICECOUNT; n++)
            {
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
                voices[voice] = but;
                last_button = but;

#ifdef USE_MIDI_OUT
                int velo = 1 + buttons[but].vpres * 127;
                if (velo > 127) velo = 127;
                else if (velo < 1) velo = 1;

                midi_usb_MidiSend3(1, MIDI_NOTE_ON | (midi_channel_offset + buttons[but].voice),
                                   buttons[but].midinote, velo);
#endif

                return 0;
            }
            return -1;
        }

        void update_voice(int but) {
#ifdef SYNTH_INTERFACE
            *(synth_interface->note[voice])  = buttons[but].note;
            *(synth_interface->pres[voice])  = buttons[but].pres;
            *(synth_interface->vpres[voice]) = buttons[but].vpres;
            *(synth_interface->but_x[voice]) = buttons[but].but_x;
            *(synth_interface->but_y[voice]) = buttons[but].but_y;
#endif

#ifdef USE_MIDI_OUT
            // TODO: pruning unnecessary messages
            //palTogglePad(GPIOA, GPIOA_LED1);
            int pres = buttons[but].pres * 127;
            if (pres > 127) pres = 127;
            else if (pres < 0) pres = 0;
            
            int tilt = 63.5 + buttons[but].but_y * 64;
            if (tilt > 127) tilt = 127;
            else if (tilt < 0) tilt = 0;
            
            int pitchbend = (bend_sensitivity * pow3(buttons[but].but_x)
                + buttons[but].note - buttons[but].midinote)
              * (0x2000 / midi_bend_range) + 0x2000 + 0.5;

            midi_usb_MidiSend2(1, MIDI_CHANNEL_PRESSURE | (midi_channel_offset + buttons[but].voice),
                               pres);
            midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE | (midi_channel_offset + buttons[but].voice),
                               70, pres);
            midi_usb_MidiSend3(1, MIDI_PITCH_BEND | (midi_channel_offset + buttons[but].voice),
                               pitchbend & 0x7f, (pitchbend >> 7) & 0x7f);
            midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE | (midi_channel_offset + buttons[but].voice),
                               74, tilt);
            if (buttons[but].vpres > 0) {
                int velo = 1 + buttons[but].vpres * 127;
                if (velo > 127) velo = 127;
                else if (velo < 0) velo = 0;
                midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE | (midi_channel_offset + buttons[but].voice),
                                   73, velo);
                if (buttons[but].vpres_prev <= 0) {
                    midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE | (midi_channel_offset + buttons[but].voice),
                                       72, 0);
                }
            } else {
                int velo = 1 - buttons[but].vpres * 127;
                if (velo > 127) velo = 127;
                else if (velo < 0) velo = 0;
                midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE | (midi_channel_offset + buttons[but].voice),
                                   72, velo);
                if (buttons[but].vpres_prev > 0) {
                    midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE | (midi_channel_offset + buttons[but].voice),
                                       73, 0);
                }
            }
            buttons[but].vpres_prev = buttons[but].vpres;
#endif
        }

        void clear_dead_notes(void) {
            systime_t now = chTimeNow();

            for (int n=0; n<VOICECOUNT; n++) {
                if (voices[n] >= 0 && buttons[voices[n]].state == STATE_ON
                    && buttons[voices[n]].timer < now)
                {
                    release_voice(voices[n]);
                    #ifdef SYNTH_INTERFACE
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
            for (int n=0; n<VOICECOUNT; n++) {
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

int c0_bas[68] = {
    3, 2, 2, 1, 1, 0, 0, -1, -1, -2, -2, -3, -3, -4, -4, -5, -5,
    4, 3, 3, 2, 2, 1, 1,  0,  0, -1, -1, -2, -2, -3, -3, -4, -4,
    5, 4, 4, 3, 3, 2, 2,  1,  1,  0,  0, -1, -1, -2, -2, -3, -3,
    5, 4, 4, 3, 3, 2, 2,  1,  1,  0,  0, -1, -1, -2, -2, -3, -3};

int c1_bas[68] = {
    -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8,
    -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8,
    -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8,
    -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8};

Instrument dis(c0_dis, c1_dis, BUTTONCOUNT, NULL);
Instrument bas(c0_bas, c1_bas, BUTTONCOUNT, NULL);

void int2float(int *msg, float *fmsg, int n) {
    int c;
    for (c=0; c<n; c++) {
        fmsg[c] = ((float)msg[c])/0x1fff; //8191, maximum of 14bit signed int
    }
}

int gesturestate = 0;
int amp_state = 2; // 0 = uit, 1 = wederom aangezet, 2 = initieel aan. Vies.

int synth_message(int size, int* msg) {
    float fmsg[7];
    int src = msg[0];
    int id = msg[1];
    msg = &msg[2];

    if (src == ID_CONTROL) {
        if (id == IDC_PORTAMENTO) {
            // Portamento button
            dis.set_portamento(msg[0]);
        }
        else if (id == IDC_SLD_NPRESS) {
            if (msg[0] >= 3) {
                // Portamento when 3 fingers are pressed
                dis.set_portamento(1);
            } else {
                dis.set_portamento(0);
            }
        }
        else if (id == IDC_SLD_SLIDE) {
            int2float(msg, fmsg, size-2);
            // TODO: send voume
        }
        else if (id == IDC_SLD_SLIDEZOOM && amp_state == 2) {
            // temporary hack to disable retuning by default, until amp_state is changed
        }
        else if (id == IDC_SLD_SLIDEZOOM) {
            int2float(msg, fmsg, size-2);
            float note_offset = dis.note_offset + fmsg[0] * -1;
            float notegen1 = dis.notegen1 + fmsg[1] * 0.1;
            if (notegen1 < 6.857) notegen1 = 6.857;
            else if (notegen1 > 7.20) notegen1 = 7.20;
            dis.note_offset = note_offset;
            dis.notegen1 = notegen1;
            bas.note_offset = note_offset;
            bas.notegen1 = notegen1;
        }
        else if (msg[0]) {
            float dif = 12.0;
            if (dis.portamento) dif = 1.0;
            if (id == IDC_OCT_UP) {
                dis.change_note_offset(dif);
                bas.change_note_offset(dif);
            }
            if (id == IDC_OCT_DOWN) {
                dis.change_note_offset(-dif);
                bas.change_note_offset(-dif);
            }
        }

    }
    else if (src == ID_ACCEL && size == 9) {
        // TODO: limit values, optimize MIDI range
        int acc_x = msg[0];
        int acc_y = msg[1];
        int acc_z = msg[2];
        int acc_abs = msg[3];
        int rot_x = msg[4];
        int rot_y = msg[5];
        int rot_z = msg[6];
#ifdef USE_MIDI_OUT
        midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE,
                           16, (64+(acc_x>>7))&0x7F);
        midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE,
                           17, (64+(acc_y>>7))&0x7F);
        midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE,
                           18, (64+(acc_z>>7))&0x7F);
        midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE,
                           19, (acc_abs>>6)&0x7F);
        midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE,
                           80, (64+(rot_x>>7))&0x7F);
        midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE,
                           81, (64+(rot_y>>7))&0x7F);
        midi_usb_MidiSend3(1, MIDI_CONTROL_CHANGE,
                           82, (64+(rot_z>>7))&0x7F);
#endif
#ifdef USE_SYNTH_INTERFACE
        int2float(msg, fmsg, size);
        *(synth_interface.acc_abs) = acc_abs;
        *(synth_interface.acc_x) = acc_x;
        *(synth_interface.acc_y) = acc_y;
        *(synth_interface.acc_z) = acc_z;
        *(synth_interface.rot_x) = rot_x;
        *(synth_interface.rot_y) = rot_y;
        *(synth_interface.rot_z) = rot_z;
#endif
    }
    else if ((src == ID_DIS || src == ID_BAS) && size == 8 && id < BUTTONCOUNT && id >= 0) {
        // 6x14bit value, dis or bas button

        int2float(msg, fmsg, size-2);

        if (src == ID_DIS) {
            dis.button_message(id, fmsg);
        } else if (src == ID_BAS) {
            if (id == 8) { fmsg[1] -= 3.0/64.0; fmsg[2] -= 1.0/64.0; } // hacky sticky button removal
            bas.button_message(id, fmsg);
        }
    }
    else {
        // unknown message source
        return -1;
    }
    return 0;
}

void clear_dead_notes(void) {
    dis.clear_dead_notes();
    bas.clear_dead_notes();
}

void synth_tick(void) {
    dis.tick();
    bas.tick();
}
