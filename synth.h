
#ifndef SYNTH_H_
#define SYNTH_H_

#include "ch.h"
#include "hal.h"

#define SAMPLINGFREQ 44100
#define CHANNEL_BUFFER_SIZE		32
#define PLAYBACK_BUFFER_SIZE	(CHANNEL_BUFFER_SIZE*2)
#define VOICECOUNT 3

typedef struct struct_synth_interface {
	float* acc_abs;
	float* acc_x;
	float* acc_y;
	float* acc_z;
	float* rot_x;
	float* rot_y;
	float* rot_z;
	float* note[VOICECOUNT];
	float* pres[VOICECOUNT];
	float* vpres[VOICECOUNT];
	float* but_x[VOICECOUNT];
	float* but_y[VOICECOUNT];
} synth_interface_t;

#ifdef SYNTH_INTERFACE
extern synth_interface_t synth_interface;
extern synth_interface_t synth_interface_bas;

extern void start_synth_thread(void);
#endif

extern int synth_message(int size, int* msg);
extern void synth_tick(void);

extern void clear_dead_notes(void);

extern float volume;

#endif /* SYNTH_H_ */
