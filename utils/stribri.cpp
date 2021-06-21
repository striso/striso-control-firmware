//*****************************************//
//  stribri.cpp
//  by Piers Titus van der Torren, 2012.
//
//  Reads Striso data over USB and sends OSC
//  to Strisy.
//
//*****************************************//

#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <lo.h>
#include <math.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>
#include <vector>
#include <string>
#include <sstream>
using namespace std;

// Platform-dependent sleep routines.
#if defined(__WINDOWS_MM__)
    #include <windows.h>
    #define SLEEP( milliseconds ) Sleep( (DWORD) milliseconds )
#else // Unix variants
    #include <unistd.h>
    #define SLEEP( milliseconds ) usleep( (unsigned long) (milliseconds * 1000.0) )
#endif

#include "striso.h"

#define SIGCOUNT 153

#define BUTTONCOUNT 68
#define MAX_PORTAMENTO_BUTTONS 8

#define VOL_TICK (0.0005) // (1.0 / (SAMPLINGFREQ / CHANNEL_BUFFER_SIZE) / 0.5) // decay time of estimated volume
#define VOL_TICK_FACT (0.998) // 0.5**(1/(SAMPLINGFREQ / CHANNEL_BUFFER_SIZE)/0.1)

#define VOLUME_FACTOR 0.015f;

#define DISP_INTERVAL .05     // [s] time interval to display info
#define CLEAR_INTERVAL 5.1 // [s] time interval to clear dead notes

#define VOICECOUNT 4
#define MAX_VOICECOUNT 15

#define MAX_MSGSIZE 16

#define LUT_SIZE 17

#define sign(x) (x>0?1:(x<0?-1:0))
#define pow2(x) ((x)*(x))
#define pow3(x) ((x)*(x)*(x))
#define max(x, y) ((x)>(y)?(x):(y))
#define min(x, y) ((x)<(y)?(x):(y))

void osc_send_float(lo_address t, int voice, const char* addr, float f);
void osc_send_float(lo_address t, const char* addr, float f);

int display_now = 0;

std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while(std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}

typedef enum {
    STATE_OFF = 0,
    STATE_ON = 1,
    STATE_PORTAMENTO = 2,
    STATE_ALT = 3,
} button_state_t;

class MotionSensor {
    public:
        float acc_abs;
        float acc_x;
        float acc_y;
        float acc_z;
        float rot_x;
        float rot_y;
        float rot_z;

        void message(float* msg) {
            acc_x = msg[0] * 8;
            acc_y = msg[1] * 8;
            acc_z = msg[2] * 8;
            acc_abs = msg[3] * 8;//sqrt(pow2(acc_x) + pow2(acc_y) + pow2(acc_z));
            rot_x = msg[4] * 1;
            rot_y = msg[5] * 1;
            rot_z = msg[6] * 1;
        }

        void send_osc(lo_address t) {
            osc_send_float(t, (char*)"accelerometer/acc_abs", acc_abs);
            osc_send_float(t, (char*)"accelerometer/acc_x", acc_x);
            osc_send_float(t, (char*)"accelerometer/acc_y", acc_y);
            osc_send_float(t, (char*)"accelerometer/acc_z", acc_z);
            osc_send_float(t, (char*)"gyroscope/rot_x", rot_x);
            osc_send_float(t, (char*)"gyroscope/rot_y", rot_y);
            osc_send_float(t, (char*)"gyroscope/rot_z", rot_z);
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
        float but_x;
        float but_y;
        float vol0;
        float vol = 0.0;
        button_state_t state = STATE_OFF;
        int timer = -1;
        int voice = -1;

        float vpres_max = 0;

        Button() {
        }

        void message(float* msg, int size) {
            unsigned int n;
            float prevpres = pres;

            if (size == 6) { // old style message (<= v2.0.5)
                // copy given parameters
                for (n = 0; n < 3; n++) {
                    signals[n] = msg[n];
                    // velocity
                    signals[3+n] = msg[3+n];
                }

                // calculate values from signals
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

                #define CENTERTEND 0.02
                // m = max(s0, s1, s2)
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
            else if (size == 4) { // new style message (>= v2.1)
                pres = msg[0];
                vpres = msg[1];
                but_x = msg[2];
                but_y = msg[3];

                // fake signals for display
                signals[0] = pres;
                if (but_x > -1.0f/18.0f)
                    signals[1] = (1.0f/18.0f + but_x) / 7.1f;
                else
                    signals[1] = (10.0f/64.0f) - but_x / 7.1f;
                if (but_y > -1.0f/18.0f)
                    signals[2] = (1.0f/18.0f + but_y) / 7.1f;
                else
                    signals[2] = (10.0f/64.0f) - but_y / 7.1f;
            }

            if (prevpres <= 0.0 && pres > 0.0)
                vpres_max = vpres;
            if (vpres_max < vpres)
                vpres_max = vpres;

            // allow vol below zero to take over oldest voice
            if (pres <= 0.0)
                vol0 = -1.0;
            else
                vol0 = pres + vpres;
            if (vol < vol0)
                vol = vol0;
        }

        void send_osc(lo_address t) {
        	osc_send_float(t, voice, (char*)"note", note);
        	osc_send_float(t, voice, (char*)"pres", pres);
        	osc_send_float(t, voice, (char*)"vpres", vpres);
        	osc_send_float(t, voice, (char*)"but_x", but_x);
        	osc_send_float(t, voice, (char*)"but_y", but_y);
        }

        void send_osc_silent(lo_address t) {
        	osc_send_float(t, voice, (char*)"pres", 0);
        	osc_send_float(t, voice, (char*)"vpres", 0);
        	osc_send_float(t, voice, (char*)"but_x", 0);
        	osc_send_float(t, voice, (char*)"but_y", 0);
        }
/*
        void send_direct(directUI interface) {
			*(interface.note[voice])  = note;
			*(interface.pres[voice])  = pres;
			*(interface.vpres[voice]) = vpres;
			*(interface.but_x[voice]) = but_x;
			*(interface.but_y[voice]) = but_y;
        }
*/
};

/*
class directUI : public UI {
	int cur_voice = -1;
public:
	float* note[voicecount];
	float* pres[voicecount];
	float* vpres[voicecount];
	float* but_x[voicecount];
	float* but_y[voicecount];
	float* acc_abs;
	float* acc_x;
	float* acc_y;
	float* acc_z;

	virtual void addElement(const char* label, FAUSTFLOAT* zone)
	{
		if (cur_voice >= 0) {
			if (!strcmp(label, "note")) note[cur_voice] = zone;
			else if (!strcmp(label, "pres")) pres[cur_voice] = zone;
			else if (!strcmp(label, "vpres")) vpres[cur_voice] = zone;
			else if (!strcmp(label, "but_x")) but_x[cur_voice] = zone;
			else if (!strcmp(label, "but_y")) but_y[cur_voice] = zone;
		}
		else if (!strcmp(label, "acc_abs")) acc_abs = zone;
		else if (!strcmp(label, "acc_x")) acc_x = zone;
		else if (!strcmp(label, "acc_y")) acc_y = zone;
		else if (!strcmp(label, "acc_z")) acc_z = zone;
	}

	virtual void pushGroupLabel(const char* label)
	{
		if (label[0] == 'v')
		{
			int v = (int)label[1] - (int)'0';
			if (v >= 0 && v < voicecount && label[2] == '\0')
				cur_voice = v;
			else
				cur_voice = -1;
		}
	}

	virtual void popGroupLabel()
	{
		cur_voice = -1;
	};

    // -- widget's layouts (just keep track of group labels)

    virtual void openTabBox(const char* label) 			{ pushGroupLabel(label); }
    virtual void openHorizontalBox(const char* label) 	{ pushGroupLabel(label); }
    virtual void openVerticalBox(const char* label)  	{ pushGroupLabel(label); }
    virtual void closeBox() 							{ popGroupLabel(); };

    // -- active widgets (just add an element)

    virtual void addButton(const char* label, FAUSTFLOAT* zone) 		{ addElement(label, zone); }
    virtual void addCheckButton(const char* label, FAUSTFLOAT* zone) 	{ addElement(label, zone); }
    virtual void addVerticalSlider(const char* label, FAUSTFLOAT* zone, FAUSTFLOAT , FAUSTFLOAT , FAUSTFLOAT , FAUSTFLOAT)
    																{ addElement(label, zone); }
    virtual void addHorizontalSlider(const char* label, FAUSTFLOAT* zone, FAUSTFLOAT , FAUSTFLOAT , FAUSTFLOAT , FAUSTFLOAT)
    																{ addElement(label, zone); }
    virtual void addNumEntry(const char* label, FAUSTFLOAT* zone, FAUSTFLOAT , FAUSTFLOAT , FAUSTFLOAT , FAUSTFLOAT)
    																{ addElement(label, zone); }

    // -- passive widgets (are ignored)

    virtual void addHorizontalBargraph(const char*, FAUSTFLOAT*, FAUSTFLOAT, FAUSTFLOAT) {};
    virtual void addVerticalBargraph(const char*, FAUSTFLOAT*, FAUSTFLOAT, FAUSTFLOAT) {};

	// -- metadata are not used

    virtual void declare(FAUSTFLOAT*, const char*, const char*) {}
};
*/

class Instrument {
    public:
        lo_address synth_port;
        char* voice_prefix;
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
        int voicecount = VOICECOUNT;
        float bend_sensitivity = 0.25;
        //directUI interface;

        Instrument(lo_address p, char* vp, int* c0, int* c1, int n_buttons, int voicecount1) {
            synth_port = p;
            voice_prefix = vp;
            int n;
            for (n = 0; n < n_buttons; n++) {
                buttons[n].coord0 = c0[n];
                buttons[n].coord1 = c1[n];
            }
            for (n = 0; n < MAX_VOICECOUNT; n++) {
                voices[n] = -1;
            }
            for (n = 0; n < MAX_PORTAMENTO_BUTTONS; n++) {
                portamento_buttons[n] = -1;
            }
            voicecount = voicecount1;
        }

        void osc_send_float(int voice, const char* addr, float f)
        {
            char msg[64];
            sprintf(msg, "/*/v%s%d/%s", voice_prefix, voice, addr);
            lo_send(synth_port, msg, "f", f);
        }

        int osc_message(vector<string> oscpath, const char *types, lo_arg **argv, int argc, void* msg, void *user_data) {
            int s = oscpath.size();
            if (s == 2 && argc == 1) {
                if (!oscpath[1].compare("notegen0")) {
                    notegen0 = argv[0]->f * 0.01;
                } else if (!oscpath[1].compare("notegen1")) {
                    notegen1 = argv[0]->f * 0.01;
                } else if (!oscpath[1].compare("offset")) {
                    note_offset = argv[0]->i;
                } else {
                    return 1;
                }
            } else {
                return 1;
            }
            return 0;
        }

        void set_portamento(int p) {
            // in mono mode portamento should stay on
            // if (config.midi_mode == MIDI_MODE_MONO) {
            //     return;
            // }
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
        }

        void button_message(int but, float* msg, int size) {
            // process button message and send osc messages
            buttons[but].message(msg, size);

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
                // buttons[but].timer = chVTGetSystemTime() + CLEAR_TIMER;

                if (but == portamento_button) {
                    // calculate average of portamento buttons
                    float temp_pres = buttons[but].pres; // save pres for note off detection

                    float sw = buttons[but].pres;
                    float vpres = buttons[but].vpres;
                    float note  = sw * buttons[but].note;
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

                    display_now = 1;
                }

                buttons[but].timer = 1;
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
                // buttons[but].last_pitchbend = INT32_MAX;
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

            osc_send_float(buttons[but].voice, (char*)"note",  buttons[but].note + pb);
            osc_send_float(buttons[but].voice, (char*)"pres",  buttons[but].pres);
            osc_send_float(buttons[but].voice, (char*)"vpres", buttons[but].vpres);
            osc_send_float(buttons[but].voice, (char*)"but_x", buttons[but].but_x);
            osc_send_float(buttons[but].voice, (char*)"but_y", buttons[but].but_y);

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

        // void clear_dead_notes(void) {
        //     systime_t now = chVTGetSystemTime();

        //     for (int n = 0; n < voicecount; n++) {
        //         if (voices[n] >= 0 && buttons[voices[n]].state == STATE_ON
        //             && buttons[voices[n]].timer < now)
        //         {
        //             release_voice(voices[n]);
        //             #ifdef USE_INTERNAL_SYNTH
        //             *(synth_interface->pres[n])  = 0.0;
        //             *(synth_interface->vpres[n]) = 0.0;
        //             *(synth_interface->but_x[n]) = 0.0;
        //             *(synth_interface->but_y[n]) = 0.0;
        //             #endif
        //         }
        //     }
        // }

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

int osc_message(const char *path, const char *types, lo_arg **argv, int argc, void* msg, void *user_data) {
	// process incoming osc message to configure stribri
    vector<string> oscpath = split(&path[1], '/');
    Instrument **instr = (Instrument**)user_data;
    int n;
    // TODO: better code
    if (!oscpath[0].compare("dis")) {
        n = 0;
    } else if (!oscpath[0].compare("bas")) {
        n = 1;
    } else if (!oscpath[0].compare("*")) {
        return instr[0]->osc_message(oscpath, types, argv, argc, msg, NULL)
             & instr[1]->osc_message(oscpath, types, argv, argc, msg, NULL);
    } else {
        return 1;
    }
    return instr[n]->osc_message(oscpath, types, argv, argc, msg, NULL);
}

long lopt(char *argv[], const char *name, long def)
{
    int i;
    for (i = 0; argv[i]; i++) if (!strcmp(argv[i], name)) return atoi(argv[i+1]);
    return def;
}

char* lopts(char *argv[], const char *name, char* def)
{
    int i;
    for (i = 0; argv[i]; i++) if (!strcmp(argv[i], name)) return argv[i+1];
    return def;
}

char vischar[64] = {' ','0','1','2','3','4','5','6','7','8','9','a','b','c','d','e','f','g','h','i','j','k','l','m','n','o','p','q','r','s','t','u','v','w','x','y','z','A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z','#'};
char sig2char(float sig)
{
    int idx = ceil((sig)*64);
    if (idx<0) idx = 0;
    else if (idx>63) idx = 63;
    return vischar[idx];
}

char vischar2[19] = {'i','h','g','f','e','d','c','b','a','0','1','2','3','4','5','6','7','8','9'};
char sig2char2(float sig)
{
    int idx = (sig+1.0f)*9 + 0.5;
    if (idx<0) idx = 0;
    else if (idx>18) idx = 18;
    return vischar[idx];
}

void int2float(int *msg, float *fmsg, int n) {
    int c;
    for (c=0; c<n; c++) {
        fmsg[c] = ((float)msg[c])/0x1fff; //8191, maximum of 14bit signed int
    }
}

void osc_send_float(lo_address t, int voice, const char* addr, float f)
{
    char msg[64];
    sprintf(msg, "/*/v%d/%s", voice, addr);
    lo_send(t, msg, "f", f);
}

void osc_send_float(lo_address t, const char* addr, float f)
{
    char msg[64];
    sprintf(msg, "/*/%s", addr);
    lo_send(t, msg, "f", f);
}

void unpack(uint8_t *in, int *out, int n) {
    int c;
    for (c=0; c<n; c++) {
        out[c] = ( ((int)in[c*2])<<7 | ((int)in[c*2+1]) ) - ( (int)(in[c*2] & 0x40) << 8);
    }
}

const char* to_binary(int x) {
    static char b[17];
    b[0] = '\0';

    int z;
    for (z = 1<<16; z > 0; z >>= 1) {
        strcat(b, ((x & z) == z) ? "1" : "0");
    }

    return b;
}

double sec_since(struct timeval *t) {
    struct timeval t1;
    gettimeofday(&t1, NULL);
    return (double)(t1.tv_sec - t->tv_sec) + (double)(t1.tv_usec - t->tv_usec)/1000000;
}

int main( int argc, char *argv[] )
{
    struct timeval t_disp, t_cleardeadnotes, t_tmp;
    int fp_inputports[2];
    int n;

    uint8_t cmsg[MAX_MSGSIZE];
    int msg[MAX_MSGSIZE/2];
    float fmsg[MAX_MSGSIZE/2];
    int msgque_overflow_bb = 0;
    int msgque_overflow_synth = 0;
    int msg_too_short_synth = 0;
    int msg_too_long_synth = 0;
    int lostbytes = 0;
    int battery_voltage = 0;

    int sld_npress = 0;
    int sld_volume = 0;
    int sld_tuneoff = 0;
    int sld_tunegen = 0;

    int n_inputports = 1;
    int cur_inputport = 0;
    char* inputport = lopts(argv, "-p", (char*)"/dev/ttyACM0");
    char* inputport2 = lopts(argv, "-p2", (char*)"");
    if (strcmp(inputport2, "")) {
        n_inputports = 2;
    }

    FILE *fp_debug;
    char* debugfile = lopts(argv, "-o", (char*)"");
    int debugtofile = strcmp(debugfile, "");

    int voicecount = lopt(argv, "-n", 4);

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

    lo_address lo_port = lo_address_new(NULL, "5510");

    Instrument *dis = new Instrument(lo_port, (char*)"", c0_dis, c1_dis, BUTTONCOUNT, voicecount);
    Instrument *bas = new Instrument(lo_port, (char*)"_bas", c0_bas, c1_bas, BUTTONCOUNT, voicecount);

    Instrument *instrs[] = {dis, bas};

    MotionSensor *accel = new MotionSensor();

    lo_server osc_host = lo_server_new("5555", NULL);
    lo_server_add_method(osc_host, NULL, NULL, osc_message, instrs);
    lo_server_recv_noblock(osc_host, 0);

    SLEEP( 100 );

    struct termios tio;
    memset(&tio,0,sizeof(tio));
    tio.c_iflag=0;
    tio.c_oflag=0;
    tio.c_cflag=CS8|CREAD|CLOCAL;           // 8n1, see termios.h for more information
    tio.c_lflag=0;
    tio.c_cc[VMIN]=1;
    tio.c_cc[VTIME]=5;
    cfsetspeed(&tio, 500000);
    fp_inputports[0] = open(inputport, O_RDWR | O_NOCTTY | O_NDELAY);
    tcsetattr(fp_inputports[0], TCSANOW, &tio);
    if (n_inputports == 2) {
        fp_inputports[1] = open(inputport2, O_RDWR | O_NOCTTY | O_NDELAY);
        tcsetattr(fp_inputports[1], TCSANOW, &tio);
    }

    if (debugtofile) {
        fp_debug=fopen(debugfile, "w");
    }

    gettimeofday(&t_disp, NULL);
    gettimeofday(&t_cleardeadnotes, NULL);
    int rxsize, src, size, id, r;
    while (1) {

    	// Receive OSC messages
        lo_server_recv_noblock(osc_host, 0);

        // Read next message
        cmsg[0] = 0x00;
        while (!(cmsg[0] & 0x80)) {
    		// Receive OSC messages
            lo_server_recv_noblock(osc_host, 0);

            cur_inputport = (cur_inputport + 1) % n_inputports;
            r = read(fp_inputports[cur_inputport], cmsg, 1);
            if (r>0) {
                lostbytes += r;
            }
            else {
            	SLEEP(1);
            }
        }
        lostbytes--;

        rxsize = ((cmsg[0] & 0x07)+1) * 2;
        size = cmsg[0] & 0x07;

        if (rxsize > MAX_MSGSIZE) { continue; }
        for (n=1; n < rxsize; ) {
            if (read(fp_inputports[cur_inputport], &cmsg[n], 1) == 1) {
                if (cmsg[n] & 0x80) { break; }
                n++;
            }
        }
        if (n<rxsize) { continue; }

        // message OK
        src = (cmsg[0] & 0x7f)>>3;
        id = cmsg[1];
        unpack(&cmsg[2], msg, size);

        if (src == ID_SYS) {
            if (id == ID_SYS_MSGQUE_OVERFLOW_BB)
                msgque_overflow_bb = msg[0];
            else if (id == ID_SYS_MSGQUE_OVERFLOW_SYNTH)
                msgque_overflow_synth = msg[0];
            else if (id == ID_SYS_MSG_TOO_SHORT_SYNTH)
                msg_too_short_synth += msg[0];
            else if (id == ID_SYS_MSG_TOO_LONG_SYNTH)
                msg_too_long_synth += msg[0];
            else if (id == ID_SYS_BATTERY_VOLTAGE)
                battery_voltage = msg[0];
        }
        else if (src == ID_CONTROL) {
            if (id == IDC_ALT) {
                dis->set_altmode(msg[0]);
            } else if (id == IDC_PORTAMENTO) {
                // Portamento button
                dis->set_portamento(msg[0]);
            } else if (id == IDC_SLD_NPRESS) {
                sld_npress = msg[0];
            } else if (id == IDC_SLD_SLIDE) {
                sld_volume += msg[0];
            } else if (id == IDC_SLD_SLIDEZOOM) {
                sld_tuneoff += msg[0];
                sld_tunegen += msg[1];
            } else if (msg[0]) {
                float dif = 12.0;
                if (dis->portamento) dif = 1.0;
                if (id == IDC_OCT_UP) {
                    dis->change_note_offset(dif);
                    bas->change_note_offset(dif);
                }
                if (id == IDC_OCT_DOWN) {
                    dis->change_note_offset(-dif);
                    bas->change_note_offset(-dif);
                }
            }
        }
        else if (src == ID_ACCEL) {
            int2float(msg, fmsg, size);
            accel->message(fmsg);
            accel->send_osc(lo_port);

            if (debugtofile) {
                gettimeofday(&t_tmp, NULL);
                fprintf(fp_debug, "%.4f, %d, %d, %d, %d, %d, %d, %d, %d\n", (double)t_tmp.tv_sec + (double)t_tmp.tv_usec/1000000, src, msg[0], msg[1], msg[2], msg[3], msg[4], msg[5], msg[6]);
                fflush(fp_debug);
            }
        }
        else if (src == ID_DIS || src == ID_BAS) {
            // 6x14bit value, dis or bas button
            if (debugtofile) {
                gettimeofday(&t_tmp, NULL);
                if (size == 6)
                    fprintf(fp_debug, "%.4f, %d, %d, %d, %d, %d, %d, %d, %d\n", (double)t_tmp.tv_sec + (double)t_tmp.tv_usec/1000000, src, id, msg[0], msg[1], msg[2], msg[3], msg[4], msg[5]);
                if (size == 4)
                    fprintf(fp_debug, "%.4f, %d, %d, %d, %d, %d, %d\n", (double)t_tmp.tv_sec + (double)t_tmp.tv_usec/1000000, src, id, msg[0], msg[1], msg[2], msg[3]);
                fflush(fp_debug);
            }

            int2float(msg, fmsg, size);

            if (id < BUTTONCOUNT) {
                if (src == ID_DIS) {
                    dis->button_message(id, fmsg, size);
                } else if (src == ID_BAS) {
                    bas->button_message(id, fmsg, size);
                }
            }
        }
        else {
            // unknown message source
        }

        #define disp_but(but) printf(" %c%c%c", sig2char(but.signals[0]), sig2char(but.signals[1]), sig2char(but.signals[2]))
        // #define disp_but2(but) printf(" %c%c%c", sig2char(but.pres), sig2char2(but.x), sig2char2(but.y))
        #define disp_butv(but) printf(" %3.0f", but.vpres_max * 1000)

        // Display information
        if (sec_since(&t_disp) > DISP_INTERVAL || display_now) {
            display_now = 0;
            gettimeofday(&t_disp, NULL);
            printf("\n");
            printf(" ges des  as  es bes  f   c   g   d   a   e   b  fis cis gis dis ais\n");
            for (n=0;n<17;n++) disp_but(dis->buttons[n]);
            printf("\n");
            for (n=0;n<17;n++) disp_butv(dis->buttons[n]);
            printf("\n");
            for (n=17;n<34;n++) disp_but(dis->buttons[n]);
            printf("\n");
            for (n=17;n<34;n++) disp_butv(dis->buttons[n]);
            printf("\n");
            for (n=34;n<51;n++) disp_but(dis->buttons[n]);
            printf("\n");
            for (n=34;n<51;n++) disp_butv(dis->buttons[n]);
            printf("\n");
            for (n=51;n<68;n++) disp_but(dis->buttons[n]);
            printf("\n");
            for (n=51;n<68;n++) disp_butv(dis->buttons[n]);
            printf("\n");
            printf("\n");
            //for (n=34;n<51;n++) disp_but(bas->buttons[n]);
            //printf("\n");
            // for (n=17;n<34;n++) disp_but(bas->buttons[n]);
            // printf("\n");
            // for (n=0;n<17;n++) disp_but(bas->buttons[n]);
            // printf("\n");
            // printf("\n");
            for (n=0;n<dis->voicecount;n++) {
                printf(" %2d", dis->voices[n]);
            }
            printf("\n");
            printf(" accel x: % 1.2f y: % 1.2f z: % 1.2f abs:% 1.2f", accel->acc_x, accel->acc_y, accel->acc_z, accel->acc_abs);
            printf(" gyro x: % 1.2f y: % 1.2f z: % 1.2f\n", accel->rot_x, accel->rot_y, accel->rot_z);
            printf(" port: %d, alt: %d, note_offset: %.2f, sld_npress: %d, vol: %d, off: %d, gen: %d\n", dis->portamento, dis->altmode, dis->note_offset, sld_npress, sld_volume, sld_tuneoff, sld_tunegen);
            printf(" overflow bb: %d, synth: %d, synth rx errors: %d, lost bytes: %d, vbat: %d\n", msgque_overflow_bb, msgque_overflow_synth, msg_too_short_synth + msg_too_long_synth, lostbytes, battery_voltage);
            fflush(stdout);
        }

        dis->tick();

        // Turn notes off when no message received since last check
        if (sec_since(&t_cleardeadnotes) > CLEAR_INTERVAL) {
            gettimeofday(&t_cleardeadnotes, NULL);
            for (n=0;n<2;n++) {
                for (int but = 0; but < BUTTONCOUNT; but++) {
                    if (instrs[n]->buttons[but].state) {
                        if (instrs[n]->buttons[but].timer == -1) {
                            float msg[6] = {0,0,0,0,0,0};
                            instrs[n]->button_message(but, msg, 6);
                        }
                        else {
                            instrs[n]->buttons[but].timer = -1;
                        }
                    }
                }
            }
        }
    }

    return 0;
}
