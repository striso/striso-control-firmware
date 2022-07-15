//----------------------------------------------------------
// name: "synth.dsp.tmp"
//
// Code generated with Faust 2.30.5 (https://faust.grame.fr)
//----------------------------------------------------------

/* link with : "" */
#include "fastpow.h"
#include <math.h>
#ifndef FAUSTPOWER
#define FAUSTPOWER
//#include <cmath>
template <int N> inline int faustpower(int x)              { return faustpower<N/2>(x) * faustpower<N-N/2>(x); } 
template <> 	 inline int faustpower<0>(int x)            { return 1; }
template <> 	 inline int faustpower<1>(int x)            { return x; }
template <> 	 inline int faustpower<2>(int x)            { return x*x; }
template <int N> inline float faustpower(float x)            { return faustpower<N/2>(x) * faustpower<N-N/2>(x); } 
template <> 	 inline float faustpower<0>(float x)          { return 1; }
template <> 	 inline float faustpower<1>(float x)          { return x; }
template <> 	 inline float faustpower<2>(float x)          { return x*x; }
#endif

extern "C" {
    #include "ch.h"
    #include "hal.h"
    #include "synth.h"
}

#define max(x,y) (x>y?x:y)
#define min(x,y) (x<y?x:y)

#define MAX_VAL (1<<23)
#define VOLUME_FILTER 0.99f

class dsp {
	protected:
		int fSamplingFreq;
};

struct Meta {
    virtual void declare(const char* key, const char* value) = 0;
};

// variables
synth_interface_t synth_interface;

static float dump;
float volume = 1.0f;

static int32_t *outbuf;
static int32_t *inbuf;
static float output0[CHANNEL_BUFFER_SIZE] = {0.0};
static float output1[CHANNEL_BUFFER_SIZE] = {0.0};
static float* output[2] = {output0, output1};

// Intrinsics

// Class
#ifndef FAUSTFLOAT
#define FAUSTFLOAT float
#endif 


#ifndef FAUSTCLASS 
#define FAUSTCLASS mydsp
#endif

class mydsp : public dsp {
  private:
	class SIG0 {
	  private:
		int fSampleRate;
	  public:
		int getNumInputs() { return 0; }
		int getNumOutputs() { return 1; }
		void init(int sample_rate) {
			fSampleRate = sample_rate;
		}
		void fill(int count, float output[]) {
			for (int i=0; i<count; i++) {
				output[i] = (9.3132257504915938e-10f * rand_hoaglin());
			}
		}
	};


	float 	fConst0;
	float 	fConst1;
	float 	fConst2;
	float 	fConst3;
	float 	fConst4;
	float 	fConst5;
	float 	fConst6;
	float 	fConst7;
	float 	fConst8;
	float 	fConst9;
	float 	fConst10;
	float 	fConst11;
	float 	fConst12;
	FAUSTFLOAT 	fslider0;
	static float 	ftbl0[4096];
	FAUSTFLOAT 	fslider1;
	float 	fVec0[2];
	float 	fConst13;
	float 	fConst14;
	float 	fRec2[2];
	float 	fRec1[3];
	float 	fConst15;
	float 	fConst16;
	float 	fConst17;
	float 	fConst18;
	FAUSTFLOAT 	fslider2;
	float 	fRec3[3];
	float 	fConst19;
	float 	fConst20;
	float 	fConst21;
	float 	fConst22;
	FAUSTFLOAT 	fslider3;
	float 	fRec4[3];
	float 	fConst23;
	float 	fConst24;
	float 	fConst25;
	float 	fConst26;
	float 	fConst27;
	float 	fConst28;
	float 	fConst29;
	float 	fConst30;
	FAUSTFLOAT 	fslider4;
	float 	fRec5[3];
	float 	fConst31;
	FAUSTFLOAT 	fslider5;
	float 	fConst32;
	float 	fConst33;
	float 	fRec6[2];
	float 	fRec7[2];
	float 	fConst34;
	float 	fConst35;
	float 	fConst36;
	float 	fConst37;
	float 	fConst38;
	float 	fConst39;
	float 	fConst40;
	float 	fConst41;
	float 	fConst42;
	FAUSTFLOAT 	fslider6;
	float 	fConst43;
	float 	fRec13[2];
	float 	fRec12[3];
	float 	fConst44;
	float 	fConst45;
	float 	fConst46;
	float 	fConst47;
	float 	fConst48;
	float 	fConst49;
	float 	fConst50;
	float 	fConst51;
	float 	fConst52;
	float 	fConst53;
	float 	fConst54;
	float 	fConst55;
	FAUSTFLOAT 	fslider7;
	float 	fRec16[3];
	float 	fConst56;
	float 	fRec15[3];
	float 	fConst57;
	float 	fConst58;
	float 	fRec14[2];
	float 	fRec9[2];
	float 	fRec10[2];
	FAUSTFLOAT 	fslider8;
	float 	fVec1[2];
	float 	fRec18[2];
	float 	fRec17[3];
	float 	fRec19[3];
	float 	fRec20[3];
	FAUSTFLOAT 	fslider9;
	float 	fRec21[3];
	FAUSTFLOAT 	fslider10;
	float 	fRec22[2];
	float 	fRec23[2];
	FAUSTFLOAT 	fslider11;
	float 	fRec29[2];
	float 	fRec28[3];
	float 	fRec25[2];
	float 	fRec26[2];
	FAUSTFLOAT 	fslider12;
	float 	fVec2[2];
	float 	fRec31[2];
	float 	fRec30[3];
	float 	fRec32[3];
	float 	fRec33[3];
	FAUSTFLOAT 	fslider13;
	float 	fRec34[3];
	FAUSTFLOAT 	fslider14;
	float 	fRec35[2];
	float 	fRec36[2];
	FAUSTFLOAT 	fslider15;
	float 	fRec42[2];
	float 	fRec41[3];
	float 	fRec38[2];
	float 	fRec39[2];
	FAUSTFLOAT 	fslider16;
	float 	fVec3[2];
	float 	fRec44[2];
	float 	fRec43[3];
	float 	fRec45[3];
	float 	fRec46[3];
	FAUSTFLOAT 	fslider17;
	float 	fRec47[3];
	FAUSTFLOAT 	fslider18;
	float 	fRec48[2];
	float 	fRec49[2];
	FAUSTFLOAT 	fslider19;
	float 	fRec55[2];
	float 	fRec54[3];
	float 	fRec51[2];
	float 	fRec52[2];
	FAUSTFLOAT 	fslider20;
	float 	fVec4[2];
	float 	fRec57[2];
	float 	fRec56[3];
	float 	fRec58[3];
	float 	fRec59[3];
	FAUSTFLOAT 	fslider21;
	float 	fRec60[3];
	FAUSTFLOAT 	fslider22;
	float 	fRec61[2];
	float 	fRec62[2];
	FAUSTFLOAT 	fslider23;
	float 	fRec68[2];
	float 	fRec67[3];
	float 	fRec64[2];
	float 	fRec65[2];
	FAUSTFLOAT 	fslider24;
	float 	fVec5[2];
	float 	fRec70[2];
	float 	fRec69[3];
	float 	fRec71[3];
	float 	fRec72[3];
	FAUSTFLOAT 	fslider25;
	float 	fRec73[3];
	FAUSTFLOAT 	fslider26;
	float 	fRec74[2];
	float 	fRec75[2];
	FAUSTFLOAT 	fslider27;
	float 	fRec81[2];
	float 	fRec80[3];
	float 	fRec77[2];
	float 	fRec78[2];
	float 	fConst59;
	float 	fRec0[3];
	float 	fConst60;
	int fSampleRate;

  public:
	virtual void metadata(Meta* m) { 
		m->declare("fast.lib/author", "Piers Titus van der Torren (pierstitus@toverlamp.org)");
		m->declare("fast.lib/licence", "Apache-2.0");
		m->declare("fast.lib/name", "Faust Fast Approximations Library");
		m->declare("filename", "synth.dsp.tmp");
		m->declare("filters.lib/lowpass0_highpass1", "Copyright (C) 2003-2019 by Julius O. Smith III <jos@ccrma.stanford.edu>");
		m->declare("filters.lib/name", "Faust Filters Library");
		m->declare("filters.lib/svf:author", "Oleg Nesterov");
		m->declare("filters.lib/svf:copyright", "Copyright (C) 2020 Oleg Nesterov <oleg@redhat.com>");
		m->declare("filters.lib/svf:license", "MIT-style STK-4.3 license");
		m->declare("filters.lib/version", "0.3");
		m->declare("maths.lib/author", "GRAME");
		m->declare("maths.lib/copyright", "GRAME");
		m->declare("maths.lib/license", "LGPL with exception");
		m->declare("maths.lib/name", "Faust Math Library");
		m->declare("maths.lib/version", "2.3");
		m->declare("name", "synth.dsp.tmp");
		m->declare("platform.lib/name", "Generic Platform Library");
		m->declare("platform.lib/version", "0.1");
		m->declare("routes.lib/name", "Faust Signal Routing Library");
		m->declare("routes.lib/version", "0.2");
		m->declare("signals.lib/name", "Faust Signal Routing Library");
		m->declare("signals.lib/version", "0.0");
	}

	virtual int getNumInputs() { return 0; }
	virtual int getNumOutputs() { return 1; }
	static void classInit(int sample_rate) {
		SIG0 sig0;
		sig0.init(sample_rate);
		sig0.fill(4096,ftbl0);
	}
	virtual void instanceConstants(int sample_rate) {
		fSampleRate = sample_rate;
		fConst0 = min(192000.0f, max(1.0f, (float)fSampleRate));
		fConst1 = faustpower<2>(fConst0);
		fConst2 = faustpower<2>((251.32741228718345f / fConst0));
		fConst3 = ((fConst2 * ((0.20330000000000001f * fConst2) + 0.31755f)) + 1.0f);
		fConst4 = (2.0f * ((63165.468166971892f * (faustpower<2>(fConst3) / fConst1)) + -1.0f));
		fConst5 = (63165.468166971892f * (fConst3 / fConst0));
		fConst6 = (((fConst3 * (fConst5 + -191.85298647876598f)) / fConst0) + 1.0f);
		fConst7 = (((fConst3 * (fConst5 + 191.85298647876598f)) / fConst0) + 1.0f);
		fConst8 = (1.0f / fConst7);
		fConst9 = faustpower<2>((4084.0704496667308f / fConst0));
		fConst10 = (4084.0704496667308f * (((fConst9 * ((0.20330000000000001f * fConst9) + 0.31755f)) + 1.0f) / fConst0));
		fConst11 = faustpower<2>((8168.1408993334617f / fConst0));
		fConst12 = (8168.1408993334617f * (((fConst11 * ((0.20330000000000001f * fConst11) + 0.31755f)) + 1.0f) / fConst0));
		fConst13 = (0.061156102924877762f * fConst0);
		fConst14 = (8.1757989159999997f / fConst0);
		fConst15 = faustpower<2>((942.47779607693792f / fConst0));
		fConst16 = (942.47779607693792f * (((fConst15 * ((0.20330000000000001f * fConst15) + 0.31755f)) + 1.0f) / fConst0));
		fConst17 = faustpower<2>((1884.9555921538758f / fConst0));
		fConst18 = (1884.9555921538758f * (((fConst17 * ((0.20330000000000001f * fConst17) + 0.31755f)) + 1.0f) / fConst0));
		fConst19 = faustpower<2>((2827.4333882308138f / fConst0));
		fConst20 = (2827.4333882308138f * (((fConst19 * ((0.20330000000000001f * fConst19) + 0.31755f)) + 1.0f) / fConst0));
		fConst21 = faustpower<2>((5340.7075111026479f / fConst0));
		fConst22 = (5340.7075111026479f * (((fConst21 * ((0.20330000000000001f * fConst21) + 0.31755f)) + 1.0f) / fConst0));
		fConst23 = faustpower<2>((62.831853071795862f / fConst0));
		fConst24 = ((fConst23 * ((0.20330000000000001f * fConst23) + 0.31755f)) + 1.0f);
		fConst25 = faustpower<2>(fConst24);
		fConst26 = (2.0f * ((3947.8417604357433f * (fConst25 / fConst1)) + -1.0f));
		fConst27 = (3947.8417604357433f * (fConst24 / fConst0));
		fConst28 = (((fConst24 * (fConst27 + -88.495567706754741f)) / fConst0) + 1.0f);
		fConst29 = (((fConst24 * (fConst27 + 88.495567706754741f)) / fConst0) + 1.0f);
		fConst30 = (1.0f / fConst29);
		fConst31 = (fConst25 / (fConst1 * fConst29));
		fConst32 = (3000.0f * fConst31);
		fConst33 = (3.1415926535897931f / fConst0);
		fConst34 = (0.80000000000000004f * fConst31);
		fConst35 = faustpower<2>((314.15926535897933f / fConst0));
		fConst36 = ((fConst35 * ((0.20330000000000001f * fConst35) + 0.31755f)) + 1.0f);
		fConst37 = faustpower<2>(fConst36);
		fConst38 = (2.0f * ((98696.044010893587f * (fConst37 / fConst1)) + -1.0f));
		fConst39 = (98696.044010893587f * (fConst36 / fConst0));
		fConst40 = (((fConst36 * (fConst39 + -314.15926535897933f)) / fConst0) + 1.0f);
		fConst41 = (((fConst36 * (fConst39 + 314.15926535897933f)) / fConst0) + 1.0f);
		fConst42 = (1.0f / fConst41);
		fConst43 = (0.010937499999999999f / fConst0);
		fConst44 = (fConst37 / (fConst1 * fConst41));
		fConst45 = faustpower<2>((125.66370614359172f / fConst0));
		fConst46 = ((fConst45 * ((0.20330000000000001f * fConst45) + 0.31755f)) + 1.0f);
		fConst47 = faustpower<2>(fConst46);
		fConst48 = (2.0f * ((15791.367041742973f * (fConst47 / fConst1)) + -1.0f));
		fConst49 = (15791.367041742973f * (fConst46 / fConst0));
		fConst50 = (((fConst46 * (fConst49 + -232.71056693257725f)) / fConst0) + 1.0f);
		fConst51 = (((fConst46 * (fConst49 + 232.71056693257725f)) / fConst0) + 1.0f);
		fConst52 = (1.0f / fConst51);
		fConst53 = (((fConst46 * (fConst49 + -95.926493239382992f)) / fConst0) + 1.0f);
		fConst54 = (((fConst46 * (fConst49 + 95.926493239382992f)) / fConst0) + 1.0f);
		fConst55 = (1.0f / fConst54);
		fConst56 = (fConst47 / (fConst1 * fConst54));
		fConst57 = (fConst47 / (fConst1 * fConst51));
		fConst58 = powf(0.5f,(10.0f / fConst0));
		fConst59 = (1.3700000000000001f * fConst44);
		fConst60 = (0 - (2.0f / fConst7));
	}
	virtual void instanceResetUserInterface() {
		fslider0 = 0.0f;
		fslider1 = 69.0f;
		fslider2 = 0.0f;
		fslider3 = 0.0f;
		fslider4 = 0.0f;
		fslider5 = 0.0f;
		fslider6 = 0.0f;
		fslider7 = 1.0f;
		fslider8 = 69.0f;
		fslider9 = 0.0f;
		fslider10 = 0.0f;
		fslider11 = 0.0f;
		fslider12 = 69.0f;
		fslider13 = 0.0f;
		fslider14 = 0.0f;
		fslider15 = 0.0f;
		fslider16 = 69.0f;
		fslider17 = 0.0f;
		fslider18 = 0.0f;
		fslider19 = 0.0f;
		fslider20 = 69.0f;
		fslider21 = 0.0f;
		fslider22 = 0.0f;
		fslider23 = 0.0f;
		fslider24 = 69.0f;
		fslider25 = 0.0f;
		fslider26 = 0.0f;
		fslider27 = 0.0f;
	}
	virtual void instanceClear() {
		for (int i=0; i<2; i++) fVec0[i] = 0;
		for (int i=0; i<2; i++) fRec2[i] = 0;
		for (int i=0; i<3; i++) fRec1[i] = 0;
		for (int i=0; i<3; i++) fRec3[i] = 0;
		for (int i=0; i<3; i++) fRec4[i] = 0;
		for (int i=0; i<3; i++) fRec5[i] = 0;
		for (int i=0; i<2; i++) fRec6[i] = 0;
		for (int i=0; i<2; i++) fRec7[i] = 0;
		for (int i=0; i<2; i++) fRec13[i] = 0;
		for (int i=0; i<3; i++) fRec12[i] = 0;
		for (int i=0; i<3; i++) fRec16[i] = 0;
		for (int i=0; i<3; i++) fRec15[i] = 0;
		for (int i=0; i<2; i++) fRec14[i] = 0;
		for (int i=0; i<2; i++) fRec9[i] = 0;
		for (int i=0; i<2; i++) fRec10[i] = 0;
		for (int i=0; i<2; i++) fVec1[i] = 0;
		for (int i=0; i<2; i++) fRec18[i] = 0;
		for (int i=0; i<3; i++) fRec17[i] = 0;
		for (int i=0; i<3; i++) fRec19[i] = 0;
		for (int i=0; i<3; i++) fRec20[i] = 0;
		for (int i=0; i<3; i++) fRec21[i] = 0;
		for (int i=0; i<2; i++) fRec22[i] = 0;
		for (int i=0; i<2; i++) fRec23[i] = 0;
		for (int i=0; i<2; i++) fRec29[i] = 0;
		for (int i=0; i<3; i++) fRec28[i] = 0;
		for (int i=0; i<2; i++) fRec25[i] = 0;
		for (int i=0; i<2; i++) fRec26[i] = 0;
		for (int i=0; i<2; i++) fVec2[i] = 0;
		for (int i=0; i<2; i++) fRec31[i] = 0;
		for (int i=0; i<3; i++) fRec30[i] = 0;
		for (int i=0; i<3; i++) fRec32[i] = 0;
		for (int i=0; i<3; i++) fRec33[i] = 0;
		for (int i=0; i<3; i++) fRec34[i] = 0;
		for (int i=0; i<2; i++) fRec35[i] = 0;
		for (int i=0; i<2; i++) fRec36[i] = 0;
		for (int i=0; i<2; i++) fRec42[i] = 0;
		for (int i=0; i<3; i++) fRec41[i] = 0;
		for (int i=0; i<2; i++) fRec38[i] = 0;
		for (int i=0; i<2; i++) fRec39[i] = 0;
		for (int i=0; i<2; i++) fVec3[i] = 0;
		for (int i=0; i<2; i++) fRec44[i] = 0;
		for (int i=0; i<3; i++) fRec43[i] = 0;
		for (int i=0; i<3; i++) fRec45[i] = 0;
		for (int i=0; i<3; i++) fRec46[i] = 0;
		for (int i=0; i<3; i++) fRec47[i] = 0;
		for (int i=0; i<2; i++) fRec48[i] = 0;
		for (int i=0; i<2; i++) fRec49[i] = 0;
		for (int i=0; i<2; i++) fRec55[i] = 0;
		for (int i=0; i<3; i++) fRec54[i] = 0;
		for (int i=0; i<2; i++) fRec51[i] = 0;
		for (int i=0; i<2; i++) fRec52[i] = 0;
		for (int i=0; i<2; i++) fVec4[i] = 0;
		for (int i=0; i<2; i++) fRec57[i] = 0;
		for (int i=0; i<3; i++) fRec56[i] = 0;
		for (int i=0; i<3; i++) fRec58[i] = 0;
		for (int i=0; i<3; i++) fRec59[i] = 0;
		for (int i=0; i<3; i++) fRec60[i] = 0;
		for (int i=0; i<2; i++) fRec61[i] = 0;
		for (int i=0; i<2; i++) fRec62[i] = 0;
		for (int i=0; i<2; i++) fRec68[i] = 0;
		for (int i=0; i<3; i++) fRec67[i] = 0;
		for (int i=0; i<2; i++) fRec64[i] = 0;
		for (int i=0; i<2; i++) fRec65[i] = 0;
		for (int i=0; i<2; i++) fVec5[i] = 0;
		for (int i=0; i<2; i++) fRec70[i] = 0;
		for (int i=0; i<3; i++) fRec69[i] = 0;
		for (int i=0; i<3; i++) fRec71[i] = 0;
		for (int i=0; i<3; i++) fRec72[i] = 0;
		for (int i=0; i<3; i++) fRec73[i] = 0;
		for (int i=0; i<2; i++) fRec74[i] = 0;
		for (int i=0; i<2; i++) fRec75[i] = 0;
		for (int i=0; i<2; i++) fRec81[i] = 0;
		for (int i=0; i<3; i++) fRec80[i] = 0;
		for (int i=0; i<2; i++) fRec77[i] = 0;
		for (int i=0; i<2; i++) fRec78[i] = 0;
		for (int i=0; i<3; i++) fRec0[i] = 0;
	}
	virtual void init(int sample_rate) {
		classInit(sample_rate);
		instanceInit(sample_rate);
	}
	virtual void instanceInit(int sample_rate) {
		instanceConstants(sample_rate);
		instanceResetUserInterface();
		instanceClear();
	}

	virtual int getSampleRate() {
		return fSampleRate;
	}
	virtual void buildUserInterfaceEmbedded() {
		synth_interface.acc_abs = &fslider7;
		synth_interface.rot_x = &fslider2;
		synth_interface.rot_y = &fslider3;
		synth_interface.rot_z = &fslider0;
		synth_interface.note[0] = &fslider24;
		synth_interface.pres[0] = &fslider26;
		synth_interface.vpres[0] = &fslider27;
		synth_interface.but_y[0] = &fslider25;
		synth_interface.note[1] = &fslider20;
		synth_interface.pres[1] = &fslider22;
		synth_interface.vpres[1] = &fslider23;
		synth_interface.but_y[1] = &fslider21;
		synth_interface.note[2] = &fslider16;
		synth_interface.pres[2] = &fslider18;
		synth_interface.vpres[2] = &fslider19;
		synth_interface.but_y[2] = &fslider17;
		synth_interface.note[3] = &fslider12;
		synth_interface.pres[3] = &fslider14;
		synth_interface.vpres[3] = &fslider15;
		synth_interface.but_y[3] = &fslider13;
		synth_interface.note[4] = &fslider8;
		synth_interface.pres[4] = &fslider10;
		synth_interface.vpres[4] = &fslider11;
		synth_interface.but_y[4] = &fslider9;
		synth_interface.note[5] = &fslider1;
		synth_interface.pres[5] = &fslider5;
		synth_interface.vpres[5] = &fslider6;
		synth_interface.but_y[5] = &fslider4;
	}
	virtual void compute (int count, FAUSTFLOAT** input, FAUSTFLOAT** output) {
		//zone1
		//zone2
		float 	fSlow0 = float(fslider0);
		float 	fSlow1 = ((int((fSlow0 > 0.0f)))?fConst12:fConst10);
		float 	fSlow2 = fabsf(fSlow0);
		float 	fSlow3 = ((0.5f * fSlow2) + 1.0f);
		float 	fSlow4 = (fSlow3 * fSlow1);
		float 	fSlow5 = ((fSlow3 * (fSlow1 * (fSlow4 + 0.125f))) + 1.0f);
		float 	fSlow6 = ((faustpower<2>(fSlow3) * faustpower<2>(fSlow1)) + -1.0f);
		float 	fSlow7 = ((fSlow3 * (fSlow1 * (fSlow4 + -0.125f))) + 1.0f);
		float 	fSlow8 = float(fslider1);
		float 	fSlow9 = fastpow2((0.083333333333333329f * fSlow8));
		int 	iSlow10 = int(min((fConst13 / fSlow9), (float)4096));
		float 	fSlow11 = float(iSlow10);
		float 	fSlow12 = (fConst14 * (fSlow9 * fSlow11));
		float 	fSlow13 = (fSlow2 * fSlow3);
		float 	fSlow14 = float(fslider2);
		float 	fSlow15 = ((int((fSlow14 > 0.0f)))?fConst18:fConst16);
		float 	fSlow16 = fabsf(fSlow14);
		float 	fSlow17 = ((0.5f * fSlow16) + 1.0f);
		float 	fSlow18 = (fSlow17 * fSlow15);
		float 	fSlow19 = ((fSlow17 * (fSlow15 * (fSlow18 + 0.125f))) + 1.0f);
		float 	fSlow20 = ((faustpower<2>(fSlow17) * faustpower<2>(fSlow15)) + -1.0f);
		float 	fSlow21 = ((fSlow17 * (fSlow15 * (fSlow18 + -0.125f))) + 1.0f);
		float 	fSlow22 = (fSlow16 * fSlow17);
		float 	fSlow23 = float(fslider3);
		float 	fSlow24 = ((int((fSlow23 > 0.0f)))?fConst22:fConst20);
		float 	fSlow25 = fabsf(fSlow23);
		float 	fSlow26 = ((0.5f * fSlow25) + 1.0f);
		float 	fSlow27 = (fSlow26 * fSlow24);
		float 	fSlow28 = ((fSlow26 * (fSlow24 * (fSlow27 + 0.20000000000000001f))) + 1.0f);
		float 	fSlow29 = ((faustpower<2>(fSlow26) * faustpower<2>(fSlow24)) + -1.0f);
		float 	fSlow30 = ((fSlow26 * (fSlow24 * (fSlow27 + -0.20000000000000001f))) + 1.0f);
		float 	fSlow31 = (0.20000000000000001f * (fSlow25 * fSlow26));
		float 	fSlow32 = float(fslider4);
		float 	fSlow33 = float(fslider5);
		float 	fSlow34 = (fConst32 * fSlow33);
		float 	fSlow35 = (fConst34 * fSlow33);
		float 	fSlow36 = max((float(fslider6) + -0.02f), (float)0);
		float 	fSlow37 = (fSlow36 / (fSlow36 + 0.5f));
		float 	fSlow38 = (1.0f - (fConst43 * (fSlow8 / max(min((16.0f * fSlow33), (0.5f * (fSlow33 + 1.0f))), 0.050000000000000003f))));
		float 	fSlow39 = faustpower<2>(fSlow33);
		float 	fSlow40 = float(fslider7);
		float 	fSlow41 = max((8.1757989159999997f * fSlow9), (float)200);
		float 	fSlow42 = float(fslider8);
		float 	fSlow43 = fastpow2((0.083333333333333329f * fSlow42));
		int 	iSlow44 = int(min((fConst13 / fSlow43), (float)4096));
		float 	fSlow45 = float(iSlow44);
		float 	fSlow46 = (fConst14 * (fSlow43 * fSlow45));
		float 	fSlow47 = float(fslider9);
		float 	fSlow48 = float(fslider10);
		float 	fSlow49 = (fConst32 * fSlow48);
		float 	fSlow50 = (fConst34 * fSlow48);
		float 	fSlow51 = max((float(fslider11) + -0.02f), (float)0);
		float 	fSlow52 = (fSlow51 / (fSlow51 + 0.5f));
		float 	fSlow53 = (1.0f - (fConst43 * (fSlow42 / max(min((16.0f * fSlow48), (0.5f * (fSlow48 + 1.0f))), 0.050000000000000003f))));
		float 	fSlow54 = faustpower<2>(fSlow48);
		float 	fSlow55 = max((8.1757989159999997f * fSlow43), (float)200);
		float 	fSlow56 = float(fslider12);
		float 	fSlow57 = fastpow2((0.083333333333333329f * fSlow56));
		int 	iSlow58 = int(min((fConst13 / fSlow57), (float)4096));
		float 	fSlow59 = float(iSlow58);
		float 	fSlow60 = (fConst14 * (fSlow57 * fSlow59));
		float 	fSlow61 = float(fslider13);
		float 	fSlow62 = float(fslider14);
		float 	fSlow63 = (fConst32 * fSlow62);
		float 	fSlow64 = (fConst34 * fSlow62);
		float 	fSlow65 = max((float(fslider15) + -0.02f), (float)0);
		float 	fSlow66 = (fSlow65 / (fSlow65 + 0.5f));
		float 	fSlow67 = (1.0f - (fConst43 * (fSlow56 / max(min((16.0f * fSlow62), (0.5f * (fSlow62 + 1.0f))), 0.050000000000000003f))));
		float 	fSlow68 = faustpower<2>(fSlow62);
		float 	fSlow69 = max((8.1757989159999997f * fSlow57), (float)200);
		float 	fSlow70 = float(fslider16);
		float 	fSlow71 = fastpow2((0.083333333333333329f * fSlow70));
		int 	iSlow72 = int(min((fConst13 / fSlow71), (float)4096));
		float 	fSlow73 = float(iSlow72);
		float 	fSlow74 = (fConst14 * (fSlow71 * fSlow73));
		float 	fSlow75 = float(fslider17);
		float 	fSlow76 = float(fslider18);
		float 	fSlow77 = (fConst32 * fSlow76);
		float 	fSlow78 = (fConst34 * fSlow76);
		float 	fSlow79 = max((float(fslider19) + -0.02f), (float)0);
		float 	fSlow80 = (fSlow79 / (fSlow79 + 0.5f));
		float 	fSlow81 = (1.0f - (fConst43 * (fSlow70 / max(min((16.0f * fSlow76), (0.5f * (fSlow76 + 1.0f))), 0.050000000000000003f))));
		float 	fSlow82 = faustpower<2>(fSlow76);
		float 	fSlow83 = max((8.1757989159999997f * fSlow71), (float)200);
		float 	fSlow84 = float(fslider20);
		float 	fSlow85 = fastpow2((0.083333333333333329f * fSlow84));
		int 	iSlow86 = int(min((fConst13 / fSlow85), (float)4096));
		float 	fSlow87 = float(iSlow86);
		float 	fSlow88 = (fConst14 * (fSlow85 * fSlow87));
		float 	fSlow89 = float(fslider21);
		float 	fSlow90 = float(fslider22);
		float 	fSlow91 = (fConst32 * fSlow90);
		float 	fSlow92 = (fConst34 * fSlow90);
		float 	fSlow93 = max((float(fslider23) + -0.02f), (float)0);
		float 	fSlow94 = (fSlow93 / (fSlow93 + 0.5f));
		float 	fSlow95 = (1.0f - (fConst43 * (fSlow84 / max(min((16.0f * fSlow90), (0.5f * (fSlow90 + 1.0f))), 0.050000000000000003f))));
		float 	fSlow96 = faustpower<2>(fSlow90);
		float 	fSlow97 = max((8.1757989159999997f * fSlow85), (float)200);
		float 	fSlow98 = float(fslider24);
		float 	fSlow99 = fastpow2((0.083333333333333329f * fSlow98));
		int 	iSlow100 = int(min((fConst13 / fSlow99), (float)4096));
		float 	fSlow101 = float(iSlow100);
		float 	fSlow102 = (fConst14 * (fSlow99 * fSlow101));
		float 	fSlow103 = float(fslider25);
		float 	fSlow104 = float(fslider26);
		float 	fSlow105 = (fConst32 * fSlow104);
		float 	fSlow106 = (fConst34 * fSlow104);
		float 	fSlow107 = max((float(fslider27) + -0.02f), (float)0);
		float 	fSlow108 = (fSlow107 / (fSlow107 + 0.5f));
		float 	fSlow109 = (1.0f - (fConst43 * (fSlow98 / max(min((16.0f * fSlow104), (0.5f * (fSlow104 + 1.0f))), 0.050000000000000003f))));
		float 	fSlow110 = faustpower<2>(fSlow104);
		float 	fSlow111 = max((8.1757989159999997f * fSlow99), (float)200);
		//zone2b
		//zone3
		FAUSTFLOAT* output0 = output[0];
		//LoopGraphScalar
		for (int i=0; i<count; i++) {
			fVec0[0] = fSlow8;
			fRec2[0] = (fSlow12 + (fRec2[1] - float((iSlow10 * ((fSlow12 + fRec2[1]) > fSlow11)))));
			int 	iTemp0 = int(fRec2[0]);
			float 	fTemp1 = ftbl0[iTemp0];
			float 	fTemp2 = (fTemp1 + ((fRec2[0] - float(iTemp0)) * (ftbl0[((iTemp0 + 1) % iSlow10)] - fTemp1)));
			fRec1[0] = (fTemp2 - (((fSlow7 * fRec1[2]) + (2.0f * (fSlow6 * fRec1[1]))) / fSlow5));
			fRec3[0] = (fTemp2 - (((fSlow21 * fRec3[2]) + (2.0f * (fSlow20 * fRec3[1]))) / fSlow19));
			fRec4[0] = (fTemp2 - (((fSlow30 * fRec4[2]) + (2.0f * (fSlow29 * fRec4[1]))) / fSlow28));
			fRec5[0] = (fSlow32 - (fConst30 * ((fConst28 * fRec5[2]) + (fConst26 * fRec5[1]))));
			float 	fTemp3 = (((3947.8417604357433f * fRec5[0]) + (7895.6835208714865f * fRec5[1])) + (3947.8417604357433f * fRec5[2]));
			float 	fTemp4 = (fConst31 * fTemp3);
			float 	fTemp5 = max(fTemp4, (float)0);
			float 	fTemp6 = tanf((fConst33 * ((fSlow34 * (fTemp3 * fTemp5)) + 200.0f)));
			float 	fTemp7 = ((fTemp6 * (fTemp6 + 0.5f)) + 1.0f);
			float 	fTemp8 = (fRec6[1] + (fTemp6 * (fTemp2 - fRec7[1])));
			float 	fTemp9 = (fTemp8 / fTemp7);
			fRec6[0] = ((2.0f * fTemp9) - fRec6[1]);
			float 	fTemp10 = (fRec7[1] + ((fTemp6 * fTemp8) / fTemp7));
			fRec7[0] = ((2.0f * fTemp10) - fRec7[1]);
			float 	fRec8 = fTemp9;
			float 	fTemp11 = max((-1 * fTemp4), (float)0);
			fRec13[0] = (fSlow38 * (max(fRec13[1], fSlow37) * float((fabsf((fSlow8 - fVec0[1])) < 1.0f))));
			fRec12[0] = (fSlow39 + (fRec13[0] - (fConst42 * ((fConst40 * fRec12[2]) + (fConst38 * fRec12[1])))));
			float 	fTemp12 = (((98696.044010893587f * fRec12[0]) + (197392.08802178717f * fRec12[1])) + (98696.044010893587f * fRec12[2]));
			fRec16[0] = (fSlow40 - (fConst55 * ((fConst53 * fRec16[2]) + (fConst48 * fRec16[1]))));
			fRec15[0] = ((fConst56 * (((15791.367041742973f * fRec16[0]) + (31582.734083485946f * fRec16[1])) + (15791.367041742973f * fRec16[2]))) - (fConst52 * ((fConst50 * fRec15[2]) + (fConst48 * fRec15[1]))));
			fRec14[0] = (fConst58 * max(fRec14[1], min((fConst57 * (((15791.367041742973f * fRec15[0]) + (31582.734083485946f * fRec15[1])) + (15791.367041742973f * fRec15[2]))), (float)2)));
			float 	fTemp13 = max((fRec14[0] + -1.0f), (float)0);
			float 	fTemp14 = tanf((fConst33 * min((fSlow41 * (faustpower<2>((fTemp13 + (fConst44 * (fTemp12 * (1.0f - fTemp11))))) + 1.0f)), (float)16000)));
			float 	fTemp15 = ((fTemp14 * (fTemp14 + (1.0f / ((8.0f * fTemp11) + 1.0f)))) + 1.0f);
			float 	fTemp16 = (fRec9[1] + (fTemp14 * (fTemp2 - fRec10[1])));
			float 	fTemp17 = (fTemp16 / fTemp15);
			fRec9[0] = ((2.0f * fTemp17) - fRec9[1]);
			float 	fTemp18 = (fRec10[1] + ((fTemp14 * fTemp16) / fTemp15));
			fRec10[0] = ((2.0f * fTemp18) - fRec10[1]);
			float 	fRec11 = fTemp18;
			fVec1[0] = fSlow42;
			fRec18[0] = (fSlow46 + (fRec18[1] - float((iSlow44 * ((fSlow46 + fRec18[1]) > fSlow45)))));
			int 	iTemp19 = int(fRec18[0]);
			float 	fTemp20 = ftbl0[iTemp19];
			float 	fTemp21 = (fTemp20 + ((fRec18[0] - float(iTemp19)) * (ftbl0[((iTemp19 + 1) % iSlow44)] - fTemp20)));
			fRec17[0] = (fTemp21 - (((fSlow7 * fRec17[2]) + (2.0f * (fSlow6 * fRec17[1]))) / fSlow5));
			fRec19[0] = (fTemp21 - (((fSlow21 * fRec19[2]) + (2.0f * (fSlow20 * fRec19[1]))) / fSlow19));
			fRec20[0] = (fTemp21 - (((fSlow30 * fRec20[2]) + (2.0f * (fSlow29 * fRec20[1]))) / fSlow28));
			fRec21[0] = (fSlow47 - (fConst30 * ((fConst28 * fRec21[2]) + (fConst26 * fRec21[1]))));
			float 	fTemp22 = (((3947.8417604357433f * fRec21[0]) + (7895.6835208714865f * fRec21[1])) + (3947.8417604357433f * fRec21[2]));
			float 	fTemp23 = (fConst31 * fTemp22);
			float 	fTemp24 = max(fTemp23, (float)0);
			float 	fTemp25 = tanf((fConst33 * ((fSlow49 * (fTemp22 * fTemp24)) + 200.0f)));
			float 	fTemp26 = ((fTemp25 * (fTemp25 + 0.5f)) + 1.0f);
			float 	fTemp27 = (fRec22[1] + (fTemp25 * (fTemp21 - fRec23[1])));
			float 	fTemp28 = (fTemp27 / fTemp26);
			fRec22[0] = ((2.0f * fTemp28) - fRec22[1]);
			float 	fTemp29 = (fRec23[1] + ((fTemp25 * fTemp27) / fTemp26));
			fRec23[0] = ((2.0f * fTemp29) - fRec23[1]);
			float 	fRec24 = fTemp28;
			float 	fTemp30 = max((-1 * fTemp23), (float)0);
			fRec29[0] = (fSlow53 * (max(fRec29[1], fSlow52) * float((fabsf((fSlow42 - fVec1[1])) < 1.0f))));
			fRec28[0] = (fSlow54 + (fRec29[0] - (fConst42 * ((fConst40 * fRec28[2]) + (fConst38 * fRec28[1])))));
			float 	fTemp31 = (((98696.044010893587f * fRec28[0]) + (197392.08802178717f * fRec28[1])) + (98696.044010893587f * fRec28[2]));
			float 	fTemp32 = tanf((fConst33 * min((fSlow55 * (faustpower<2>((fTemp13 + (fConst44 * (fTemp31 * (1.0f - fTemp30))))) + 1.0f)), (float)16000)));
			float 	fTemp33 = ((fTemp32 * (fTemp32 + (1.0f / ((8.0f * fTemp30) + 1.0f)))) + 1.0f);
			float 	fTemp34 = (fRec25[1] + (fTemp32 * (fTemp21 - fRec26[1])));
			float 	fTemp35 = (fTemp34 / fTemp33);
			fRec25[0] = ((2.0f * fTemp35) - fRec25[1]);
			float 	fTemp36 = (fRec26[1] + ((fTemp32 * fTemp34) / fTemp33));
			fRec26[0] = ((2.0f * fTemp36) - fRec26[1]);
			float 	fRec27 = fTemp36;
			fVec2[0] = fSlow56;
			fRec31[0] = (fSlow60 + (fRec31[1] - float((iSlow58 * ((fSlow60 + fRec31[1]) > fSlow59)))));
			int 	iTemp37 = int(fRec31[0]);
			float 	fTemp38 = ftbl0[iTemp37];
			float 	fTemp39 = (fTemp38 + ((fRec31[0] - float(iTemp37)) * (ftbl0[((iTemp37 + 1) % iSlow58)] - fTemp38)));
			fRec30[0] = (fTemp39 - (((fSlow7 * fRec30[2]) + (2.0f * (fSlow6 * fRec30[1]))) / fSlow5));
			fRec32[0] = (fTemp39 - (((fSlow21 * fRec32[2]) + (2.0f * (fSlow20 * fRec32[1]))) / fSlow19));
			fRec33[0] = (fTemp39 - (((fSlow30 * fRec33[2]) + (2.0f * (fSlow29 * fRec33[1]))) / fSlow28));
			fRec34[0] = (fSlow61 - (fConst30 * ((fConst28 * fRec34[2]) + (fConst26 * fRec34[1]))));
			float 	fTemp40 = (((3947.8417604357433f * fRec34[0]) + (7895.6835208714865f * fRec34[1])) + (3947.8417604357433f * fRec34[2]));
			float 	fTemp41 = (fConst31 * fTemp40);
			float 	fTemp42 = max(fTemp41, (float)0);
			float 	fTemp43 = tanf((fConst33 * ((fSlow63 * (fTemp40 * fTemp42)) + 200.0f)));
			float 	fTemp44 = ((fTemp43 * (fTemp43 + 0.5f)) + 1.0f);
			float 	fTemp45 = (fRec35[1] + (fTemp43 * (fTemp39 - fRec36[1])));
			float 	fTemp46 = (fTemp45 / fTemp44);
			fRec35[0] = ((2.0f * fTemp46) - fRec35[1]);
			float 	fTemp47 = (fRec36[1] + ((fTemp43 * fTemp45) / fTemp44));
			fRec36[0] = ((2.0f * fTemp47) - fRec36[1]);
			float 	fRec37 = fTemp46;
			float 	fTemp48 = max((-1 * fTemp41), (float)0);
			fRec42[0] = (fSlow67 * (max(fRec42[1], fSlow66) * float((fabsf((fSlow56 - fVec2[1])) < 1.0f))));
			fRec41[0] = (fSlow68 + (fRec42[0] - (fConst42 * ((fConst40 * fRec41[2]) + (fConst38 * fRec41[1])))));
			float 	fTemp49 = (((98696.044010893587f * fRec41[0]) + (197392.08802178717f * fRec41[1])) + (98696.044010893587f * fRec41[2]));
			float 	fTemp50 = tanf((fConst33 * min((fSlow69 * (faustpower<2>((fTemp13 + (fConst44 * (fTemp49 * (1.0f - fTemp48))))) + 1.0f)), (float)16000)));
			float 	fTemp51 = ((fTemp50 * (fTemp50 + (1.0f / ((8.0f * fTemp48) + 1.0f)))) + 1.0f);
			float 	fTemp52 = (fRec38[1] + (fTemp50 * (fTemp39 - fRec39[1])));
			float 	fTemp53 = (fTemp52 / fTemp51);
			fRec38[0] = ((2.0f * fTemp53) - fRec38[1]);
			float 	fTemp54 = (fRec39[1] + ((fTemp50 * fTemp52) / fTemp51));
			fRec39[0] = ((2.0f * fTemp54) - fRec39[1]);
			float 	fRec40 = fTemp54;
			fVec3[0] = fSlow70;
			fRec44[0] = (fSlow74 + (fRec44[1] - float((iSlow72 * ((fSlow74 + fRec44[1]) > fSlow73)))));
			int 	iTemp55 = int(fRec44[0]);
			float 	fTemp56 = ftbl0[iTemp55];
			float 	fTemp57 = (fTemp56 + ((fRec44[0] - float(iTemp55)) * (ftbl0[((iTemp55 + 1) % iSlow72)] - fTemp56)));
			fRec43[0] = (fTemp57 - (((fSlow7 * fRec43[2]) + (2.0f * (fSlow6 * fRec43[1]))) / fSlow5));
			fRec45[0] = (fTemp57 - (((fSlow21 * fRec45[2]) + (2.0f * (fSlow20 * fRec45[1]))) / fSlow19));
			fRec46[0] = (fTemp57 - (((fSlow30 * fRec46[2]) + (2.0f * (fSlow29 * fRec46[1]))) / fSlow28));
			fRec47[0] = (fSlow75 - (fConst30 * ((fConst28 * fRec47[2]) + (fConst26 * fRec47[1]))));
			float 	fTemp58 = (((3947.8417604357433f * fRec47[0]) + (7895.6835208714865f * fRec47[1])) + (3947.8417604357433f * fRec47[2]));
			float 	fTemp59 = (fConst31 * fTemp58);
			float 	fTemp60 = max(fTemp59, (float)0);
			float 	fTemp61 = tanf((fConst33 * ((fSlow77 * (fTemp58 * fTemp60)) + 200.0f)));
			float 	fTemp62 = ((fTemp61 * (fTemp61 + 0.5f)) + 1.0f);
			float 	fTemp63 = (fRec48[1] + (fTemp61 * (fTemp57 - fRec49[1])));
			float 	fTemp64 = (fTemp63 / fTemp62);
			fRec48[0] = ((2.0f * fTemp64) - fRec48[1]);
			float 	fTemp65 = (fRec49[1] + ((fTemp61 * fTemp63) / fTemp62));
			fRec49[0] = ((2.0f * fTemp65) - fRec49[1]);
			float 	fRec50 = fTemp64;
			float 	fTemp66 = max((-1 * fTemp59), (float)0);
			fRec55[0] = (fSlow81 * (max(fRec55[1], fSlow80) * float((fabsf((fSlow70 - fVec3[1])) < 1.0f))));
			fRec54[0] = (fSlow82 + (fRec55[0] - (fConst42 * ((fConst40 * fRec54[2]) + (fConst38 * fRec54[1])))));
			float 	fTemp67 = (((98696.044010893587f * fRec54[0]) + (197392.08802178717f * fRec54[1])) + (98696.044010893587f * fRec54[2]));
			float 	fTemp68 = tanf((fConst33 * min((fSlow83 * (faustpower<2>((fTemp13 + (fConst44 * (fTemp67 * (1.0f - fTemp66))))) + 1.0f)), (float)16000)));
			float 	fTemp69 = ((fTemp68 * (fTemp68 + (1.0f / ((8.0f * fTemp66) + 1.0f)))) + 1.0f);
			float 	fTemp70 = (fRec51[1] + (fTemp68 * (fTemp57 - fRec52[1])));
			float 	fTemp71 = (fTemp70 / fTemp69);
			fRec51[0] = ((2.0f * fTemp71) - fRec51[1]);
			float 	fTemp72 = (fRec52[1] + ((fTemp68 * fTemp70) / fTemp69));
			fRec52[0] = ((2.0f * fTemp72) - fRec52[1]);
			float 	fRec53 = fTemp72;
			fVec4[0] = fSlow84;
			fRec57[0] = (fSlow88 + (fRec57[1] - float((iSlow86 * ((fSlow88 + fRec57[1]) > fSlow87)))));
			int 	iTemp73 = int(fRec57[0]);
			float 	fTemp74 = ftbl0[iTemp73];
			float 	fTemp75 = (fTemp74 + ((fRec57[0] - float(iTemp73)) * (ftbl0[((iTemp73 + 1) % iSlow86)] - fTemp74)));
			fRec56[0] = (fTemp75 - (((fSlow7 * fRec56[2]) + (2.0f * (fSlow6 * fRec56[1]))) / fSlow5));
			fRec58[0] = (fTemp75 - (((fSlow21 * fRec58[2]) + (2.0f * (fSlow20 * fRec58[1]))) / fSlow19));
			fRec59[0] = (fTemp75 - (((fSlow30 * fRec59[2]) + (2.0f * (fSlow29 * fRec59[1]))) / fSlow28));
			fRec60[0] = (fSlow89 - (fConst30 * ((fConst28 * fRec60[2]) + (fConst26 * fRec60[1]))));
			float 	fTemp76 = (((3947.8417604357433f * fRec60[0]) + (7895.6835208714865f * fRec60[1])) + (3947.8417604357433f * fRec60[2]));
			float 	fTemp77 = (fConst31 * fTemp76);
			float 	fTemp78 = max(fTemp77, (float)0);
			float 	fTemp79 = tanf((fConst33 * ((fSlow91 * (fTemp76 * fTemp78)) + 200.0f)));
			float 	fTemp80 = ((fTemp79 * (fTemp79 + 0.5f)) + 1.0f);
			float 	fTemp81 = (fRec61[1] + (fTemp79 * (fTemp75 - fRec62[1])));
			float 	fTemp82 = (fTemp81 / fTemp80);
			fRec61[0] = ((2.0f * fTemp82) - fRec61[1]);
			float 	fTemp83 = (fRec62[1] + ((fTemp79 * fTemp81) / fTemp80));
			fRec62[0] = ((2.0f * fTemp83) - fRec62[1]);
			float 	fRec63 = fTemp82;
			float 	fTemp84 = max((-1 * fTemp77), (float)0);
			fRec68[0] = (fSlow95 * (max(fRec68[1], fSlow94) * float((fabsf((fSlow84 - fVec4[1])) < 1.0f))));
			fRec67[0] = (fSlow96 + (fRec68[0] - (fConst42 * ((fConst40 * fRec67[2]) + (fConst38 * fRec67[1])))));
			float 	fTemp85 = (((98696.044010893587f * fRec67[0]) + (197392.08802178717f * fRec67[1])) + (98696.044010893587f * fRec67[2]));
			float 	fTemp86 = tanf((fConst33 * min((fSlow97 * (faustpower<2>((fTemp13 + (fConst44 * (fTemp85 * (1.0f - fTemp84))))) + 1.0f)), (float)16000)));
			float 	fTemp87 = ((fTemp86 * (fTemp86 + (1.0f / ((8.0f * fTemp84) + 1.0f)))) + 1.0f);
			float 	fTemp88 = (fRec64[1] + (fTemp86 * (fTemp75 - fRec65[1])));
			float 	fTemp89 = (fTemp88 / fTemp87);
			fRec64[0] = ((2.0f * fTemp89) - fRec64[1]);
			float 	fTemp90 = (fRec65[1] + ((fTemp86 * fTemp88) / fTemp87));
			fRec65[0] = ((2.0f * fTemp90) - fRec65[1]);
			float 	fRec66 = fTemp90;
			fVec5[0] = fSlow98;
			fRec70[0] = (fSlow102 + (fRec70[1] - float((iSlow100 * ((fSlow102 + fRec70[1]) > fSlow101)))));
			int 	iTemp91 = int(fRec70[0]);
			float 	fTemp92 = ftbl0[iTemp91];
			float 	fTemp93 = (fTemp92 + ((fRec70[0] - float(iTemp91)) * (ftbl0[((iTemp91 + 1) % iSlow100)] - fTemp92)));
			fRec69[0] = (fTemp93 - (((fSlow7 * fRec69[2]) + (2.0f * (fSlow6 * fRec69[1]))) / fSlow5));
			fRec71[0] = (fTemp93 - (((fSlow21 * fRec71[2]) + (2.0f * (fSlow20 * fRec71[1]))) / fSlow19));
			fRec72[0] = (fTemp93 - (((fSlow30 * fRec72[2]) + (2.0f * (fSlow29 * fRec72[1]))) / fSlow28));
			fRec73[0] = (fSlow103 - (fConst30 * ((fConst28 * fRec73[2]) + (fConst26 * fRec73[1]))));
			float 	fTemp94 = (((3947.8417604357433f * fRec73[0]) + (7895.6835208714865f * fRec73[1])) + (3947.8417604357433f * fRec73[2]));
			float 	fTemp95 = (fConst31 * fTemp94);
			float 	fTemp96 = max(fTemp95, (float)0);
			float 	fTemp97 = tanf((fConst33 * ((fSlow105 * (fTemp94 * fTemp96)) + 200.0f)));
			float 	fTemp98 = ((fTemp97 * (fTemp97 + 0.5f)) + 1.0f);
			float 	fTemp99 = (fRec74[1] + (fTemp97 * (fTemp93 - fRec75[1])));
			float 	fTemp100 = (fTemp99 / fTemp98);
			fRec74[0] = ((2.0f * fTemp100) - fRec74[1]);
			float 	fTemp101 = (fRec75[1] + ((fTemp97 * fTemp99) / fTemp98));
			fRec75[0] = ((2.0f * fTemp101) - fRec75[1]);
			float 	fRec76 = fTemp100;
			float 	fTemp102 = max((-1 * fTemp95), (float)0);
			fRec81[0] = (fSlow109 * (max(fRec81[1], fSlow108) * float((fabsf((fSlow98 - fVec5[1])) < 1.0f))));
			fRec80[0] = (fSlow110 + (fRec81[0] - (fConst42 * ((fConst40 * fRec80[2]) + (fConst38 * fRec80[1])))));
			float 	fTemp103 = (((98696.044010893587f * fRec80[0]) + (197392.08802178717f * fRec80[1])) + (98696.044010893587f * fRec80[2]));
			float 	fTemp104 = tanf((fConst33 * min((fSlow111 * (faustpower<2>(((fConst44 * (fTemp103 * (1.0f - fTemp102))) + fTemp13)) + 1.0f)), (float)16000)));
			float 	fTemp105 = ((fTemp104 * (fTemp104 + (1.0f / ((8.0f * fTemp102) + 1.0f)))) + 1.0f);
			float 	fTemp106 = (fRec77[1] + (fTemp104 * (fTemp93 - fRec78[1])));
			float 	fTemp107 = (fTemp106 / fTemp105);
			fRec77[0] = ((2.0f * fTemp107) - fRec77[1]);
			float 	fTemp108 = (fRec78[1] + ((fTemp104 * fTemp106) / fTemp105));
			fRec78[0] = ((2.0f * fTemp108) - fRec78[1]);
			float 	fRec79 = fTemp108;
			fRec0[0] = ((fConst59 * ((((((fTemp103 * (((fRec79 * faustpower<2>((1.0f - (0.5f * fTemp102)))) + (fSlow106 * ((fRec76 * fTemp94) * fTemp96))) + (6.0f * ((fSlow31 * ((fSlow24 * (fRec72[0] - fRec72[2])) / fSlow28)) + (0.125f * ((fSlow22 * ((fSlow15 * (fRec71[0] - fRec71[2])) / fSlow19)) + (fSlow13 * ((fSlow1 * (fRec69[0] - fRec69[2])) / fSlow5)))))))) + (fTemp85 * (((fRec66 * faustpower<2>((1.0f - (0.5f * fTemp84)))) + (fSlow92 * ((fRec63 * fTemp76) * fTemp78))) + (6.0f * ((fSlow31 * ((fSlow24 * (fRec59[0] - fRec59[2])) / fSlow28)) + (0.125f * ((fSlow22 * ((fSlow15 * (fRec58[0] - fRec58[2])) / fSlow19)) + (fSlow13 * ((fSlow1 * (fRec56[0] - fRec56[2])) / fSlow5))))))))) + (fTemp67 * (((fRec53 * faustpower<2>((1.0f - (0.5f * fTemp66)))) + (fSlow78 * ((fRec50 * fTemp58) * fTemp60))) + (6.0f * ((fSlow31 * ((fSlow24 * (fRec46[0] - fRec46[2])) / fSlow28)) + (0.125f * ((fSlow22 * ((fSlow15 * (fRec45[0] - fRec45[2])) / fSlow19)) + (fSlow13 * ((fSlow1 * (fRec43[0] - fRec43[2])) / fSlow5))))))))) + (fTemp49 * (((fRec40 * faustpower<2>((1.0f - (0.5f * fTemp48)))) + (fSlow64 * ((fRec37 * fTemp40) * fTemp42))) + (6.0f * ((fSlow31 * ((fSlow24 * (fRec33[0] - fRec33[2])) / fSlow28)) + (0.125f * ((fSlow22 * ((fSlow15 * (fRec32[0] - fRec32[2])) / fSlow19)) + (fSlow13 * ((fSlow1 * (fRec30[0] - fRec30[2])) / fSlow5))))))))) + (fTemp31 * (((fRec27 * faustpower<2>((1.0f - (0.5f * fTemp30)))) + (fSlow50 * ((fRec24 * fTemp22) * fTemp24))) + (6.0f * ((fSlow31 * ((fSlow24 * (fRec20[0] - fRec20[2])) / fSlow28)) + (0.125f * ((fSlow22 * ((fSlow15 * (fRec19[0] - fRec19[2])) / fSlow19)) + (fSlow13 * ((fSlow1 * (fRec17[0] - fRec17[2])) / fSlow5))))))))) + (fTemp12 * (((fRec11 * faustpower<2>((1.0f - (0.5f * fTemp11)))) + (fSlow35 * ((fRec8 * fTemp3) * fTemp5))) + (6.0f * ((fSlow31 * ((fSlow24 * (fRec4[0] - fRec4[2])) / fSlow28)) + (0.125f * ((fSlow22 * ((fSlow15 * (fRec3[0] - fRec3[2])) / fSlow19)) + (fSlow13 * ((fSlow1 * (fRec1[0] - fRec1[2])) / fSlow5)))))))))) - (fConst8 * ((fConst6 * fRec0[2]) + (fConst4 * fRec0[1]))));
			output0[i] = (FAUSTFLOAT)((fConst60 * fRec0[1]) + (fConst8 * (fRec0[0] + fRec0[2])));
			// post processing
			fRec0[2] = fRec0[1]; fRec0[1] = fRec0[0];
			fRec78[1] = fRec78[0];
			fRec77[1] = fRec77[0];
			fRec80[2] = fRec80[1]; fRec80[1] = fRec80[0];
			fRec81[1] = fRec81[0];
			fRec75[1] = fRec75[0];
			fRec74[1] = fRec74[0];
			fRec73[2] = fRec73[1]; fRec73[1] = fRec73[0];
			fRec72[2] = fRec72[1]; fRec72[1] = fRec72[0];
			fRec71[2] = fRec71[1]; fRec71[1] = fRec71[0];
			fRec69[2] = fRec69[1]; fRec69[1] = fRec69[0];
			fRec70[1] = fRec70[0];
			fVec5[1] = fVec5[0];
			fRec65[1] = fRec65[0];
			fRec64[1] = fRec64[0];
			fRec67[2] = fRec67[1]; fRec67[1] = fRec67[0];
			fRec68[1] = fRec68[0];
			fRec62[1] = fRec62[0];
			fRec61[1] = fRec61[0];
			fRec60[2] = fRec60[1]; fRec60[1] = fRec60[0];
			fRec59[2] = fRec59[1]; fRec59[1] = fRec59[0];
			fRec58[2] = fRec58[1]; fRec58[1] = fRec58[0];
			fRec56[2] = fRec56[1]; fRec56[1] = fRec56[0];
			fRec57[1] = fRec57[0];
			fVec4[1] = fVec4[0];
			fRec52[1] = fRec52[0];
			fRec51[1] = fRec51[0];
			fRec54[2] = fRec54[1]; fRec54[1] = fRec54[0];
			fRec55[1] = fRec55[0];
			fRec49[1] = fRec49[0];
			fRec48[1] = fRec48[0];
			fRec47[2] = fRec47[1]; fRec47[1] = fRec47[0];
			fRec46[2] = fRec46[1]; fRec46[1] = fRec46[0];
			fRec45[2] = fRec45[1]; fRec45[1] = fRec45[0];
			fRec43[2] = fRec43[1]; fRec43[1] = fRec43[0];
			fRec44[1] = fRec44[0];
			fVec3[1] = fVec3[0];
			fRec39[1] = fRec39[0];
			fRec38[1] = fRec38[0];
			fRec41[2] = fRec41[1]; fRec41[1] = fRec41[0];
			fRec42[1] = fRec42[0];
			fRec36[1] = fRec36[0];
			fRec35[1] = fRec35[0];
			fRec34[2] = fRec34[1]; fRec34[1] = fRec34[0];
			fRec33[2] = fRec33[1]; fRec33[1] = fRec33[0];
			fRec32[2] = fRec32[1]; fRec32[1] = fRec32[0];
			fRec30[2] = fRec30[1]; fRec30[1] = fRec30[0];
			fRec31[1] = fRec31[0];
			fVec2[1] = fVec2[0];
			fRec26[1] = fRec26[0];
			fRec25[1] = fRec25[0];
			fRec28[2] = fRec28[1]; fRec28[1] = fRec28[0];
			fRec29[1] = fRec29[0];
			fRec23[1] = fRec23[0];
			fRec22[1] = fRec22[0];
			fRec21[2] = fRec21[1]; fRec21[1] = fRec21[0];
			fRec20[2] = fRec20[1]; fRec20[1] = fRec20[0];
			fRec19[2] = fRec19[1]; fRec19[1] = fRec19[0];
			fRec17[2] = fRec17[1]; fRec17[1] = fRec17[0];
			fRec18[1] = fRec18[0];
			fVec1[1] = fVec1[0];
			fRec10[1] = fRec10[0];
			fRec9[1] = fRec9[0];
			fRec14[1] = fRec14[0];
			fRec15[2] = fRec15[1]; fRec15[1] = fRec15[0];
			fRec16[2] = fRec16[1]; fRec16[1] = fRec16[0];
			fRec12[2] = fRec12[1]; fRec12[1] = fRec12[0];
			fRec13[1] = fRec13[0];
			fRec7[1] = fRec7[0];
			fRec6[1] = fRec6[0];
			fRec5[2] = fRec5[1]; fRec5[1] = fRec5[0];
			fRec4[2] = fRec4[1]; fRec4[1] = fRec4[0];
			fRec3[2] = fRec3[1]; fRec3[1] = fRec3[0];
			fRec1[2] = fRec1[1]; fRec1[1] = fRec1[0];
			fRec2[1] = fRec2[0];
			fVec0[1] = fVec0[0];
		}
	}
};


float 	mydsp::ftbl0[4096];

FAUSTCLASS dsp;

static THD_WORKING_AREA(waSynthThread, 1024);
static thread_t* pThreadDSP = 0;
static void synthThread(void *arg) {  // THE SYNTH THREAD
	(void)arg;
	chRegSetThreadName("SYNTH");

	int32_t tmp;
	int count = CHANNEL_BUFFER_SIZE;
	float volume_filtered = 0.0f;

	//codec_pwrCtl(1);    // POWER ON
	//codec_muteCtl(0);   // MUTE OFF

	chEvtAddEvents(1);

	// initialize interface with dump, for when controls are not used
	synth_interface.acc_abs = &dump;
	synth_interface.acc_x = &dump;
	synth_interface.acc_y = &dump;
	synth_interface.acc_z = &dump;
	synth_interface.rot_x = &dump;
	synth_interface.rot_y = &dump;
	synth_interface.rot_z = &dump;

	// initialization
	dsp.init(SAMPLINGFREQ);

	// initialize interface
	dsp.buildUserInterfaceEmbedded();

	// computation loop
	while (true) {
		// palClearLine(LINE_LED_ALT);
		chEvtWaitOne(1);
		// palSetLine(LINE_LED_ALT);

		dsp.compute(count, NULL, output);

		volume_filtered = VOLUME_FILTER * volume_filtered + (1 - VOLUME_FILTER) * volume;

		// convert float to int with scale, clamp and round
		for (int n = 0; n < CHANNEL_BUFFER_SIZE; n++) {
			tmp = (int32_t)(output0[n] * volume_filtered * MAX_VAL);
			// enable LED on clip
			if (tmp <= -MAX_VAL) {
				tmp = -(MAX_VAL-1);
			} else if (tmp >= MAX_VAL) {
				tmp = MAX_VAL-1;
			}
			// make both audio channels the same
			outbuf[2*n] = outbuf[2*n+1] = tmp * (1<<8);
		}

		//if (--n <= 0) {
		//	palToggleLine(LINE_LED1);       /* Orange.  */
		//	n = 100;
		//}

		// TODO: synth_tick better called here or in main?
		// synth_tick();

		if (chThdShouldTerminateX()) break;
	}

	//codec_muteCtl(1);
	//codec_pwrCtl(0);

	pThreadDSP=NULL;
	palToggleLine(LINE_LED1);
};

void start_synth_thread(void) {
	pThreadDSP = chThdCreateStatic(waSynthThread, sizeof(waSynthThread), NORMALPRIO+2, synthThread, NULL);
}

void computebufI(int32_t *inp, int32_t *outp) {
  int i;
  //for (i = 0; i < 32; i++) {
  //  inbuf[i] = inp[i];
  //}
  outbuf = outp;
  inbuf = inp;
  if (pThreadDSP) {
    chSysLockFromISR();
    chEvtSignalI(pThreadDSP, (eventmask_t)1);
    chSysUnlockFromISR();
  }
  else
    for (i = 0; i < PLAYBACK_BUFFER_SIZE; i++) {
      outp[i] = (i - PLAYBACK_BUFFER_SIZE / 2) * 1<<22;
      // square wave 34952 * (1<<8) * 2 * ((i>PLAYBACK_BUFFER_SIZE/2)-0.5);
      // saw ware (i - PLAYBACK_BUFFER_SIZE / 2) * 100000; // testing noise 0;
    }
}
