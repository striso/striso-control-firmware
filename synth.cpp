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
	float 	fConst23;
	float 	fConst24;
	float 	fConst25;
	float 	fConst26;
	FAUSTFLOAT 	fslider3;
	float 	fRec4[3];
	float 	fConst27;
	FAUSTFLOAT 	fslider4;
	float 	fConst28;
	float 	fConst29;
	float 	fRec5[2];
	float 	fRec6[2];
	float 	fConst30;
	float 	fConst31;
	float 	fConst32;
	float 	fConst33;
	float 	fConst34;
	float 	fConst35;
	float 	fConst36;
	float 	fConst37;
	float 	fConst38;
	FAUSTFLOAT 	fslider5;
	float 	fConst39;
	float 	fRec12[2];
	FAUSTFLOAT 	fslider6;
	float 	fRec11[3];
	float 	fConst40;
	float 	fConst41;
	float 	fConst42;
	float 	fConst43;
	float 	fConst44;
	float 	fConst45;
	float 	fConst46;
	float 	fConst47;
	float 	fConst48;
	float 	fConst49;
	float 	fConst50;
	float 	fConst51;
	FAUSTFLOAT 	fslider7;
	float 	fRec15[3];
	float 	fConst52;
	float 	fRec14[3];
	float 	fConst53;
	float 	fConst54;
	float 	fRec13[2];
	float 	fRec8[2];
	float 	fRec9[2];
	FAUSTFLOAT 	fslider8;
	float 	fVec1[2];
	float 	fRec17[2];
	float 	fRec16[3];
	float 	fRec18[3];
	FAUSTFLOAT 	fslider9;
	float 	fRec19[3];
	FAUSTFLOAT 	fslider10;
	float 	fRec20[2];
	float 	fRec21[2];
	FAUSTFLOAT 	fslider11;
	float 	fRec27[2];
	float 	fRec26[3];
	float 	fRec23[2];
	float 	fRec24[2];
	FAUSTFLOAT 	fslider12;
	float 	fVec2[2];
	float 	fRec29[2];
	float 	fRec28[3];
	float 	fRec30[3];
	FAUSTFLOAT 	fslider13;
	float 	fRec31[3];
	FAUSTFLOAT 	fslider14;
	float 	fRec32[2];
	float 	fRec33[2];
	FAUSTFLOAT 	fslider15;
	float 	fRec39[2];
	float 	fRec38[3];
	float 	fRec35[2];
	float 	fRec36[2];
	FAUSTFLOAT 	fslider16;
	float 	fVec3[2];
	float 	fRec41[2];
	float 	fRec40[3];
	float 	fRec42[3];
	FAUSTFLOAT 	fslider17;
	float 	fRec43[3];
	FAUSTFLOAT 	fslider18;
	float 	fRec44[2];
	float 	fRec45[2];
	FAUSTFLOAT 	fslider19;
	float 	fRec51[2];
	float 	fRec50[3];
	float 	fRec47[2];
	float 	fRec48[2];
	FAUSTFLOAT 	fslider20;
	float 	fVec4[2];
	float 	fRec53[2];
	float 	fRec52[3];
	float 	fRec54[3];
	FAUSTFLOAT 	fslider21;
	float 	fRec55[3];
	FAUSTFLOAT 	fslider22;
	float 	fRec56[2];
	float 	fRec57[2];
	FAUSTFLOAT 	fslider23;
	float 	fRec63[2];
	float 	fRec62[3];
	float 	fRec59[2];
	float 	fRec60[2];
	FAUSTFLOAT 	fslider24;
	float 	fVec5[2];
	float 	fRec65[2];
	float 	fRec64[3];
	float 	fRec66[3];
	FAUSTFLOAT 	fslider25;
	float 	fRec67[3];
	FAUSTFLOAT 	fslider26;
	float 	fRec68[2];
	float 	fRec69[2];
	FAUSTFLOAT 	fslider27;
	float 	fRec75[2];
	float 	fRec74[3];
	float 	fRec71[2];
	float 	fRec72[2];
	float 	fRec0[3];
	float 	fConst55;
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
		fConst9 = faustpower<2>((942.47779607693792f / fConst0));
		fConst10 = (942.47779607693792f * (((fConst9 * ((0.20330000000000001f * fConst9) + 0.31755f)) + 1.0f) / fConst0));
		fConst11 = faustpower<2>((1884.9555921538758f / fConst0));
		fConst12 = (1884.9555921538758f * (((fConst11 * ((0.20330000000000001f * fConst11) + 0.31755f)) + 1.0f) / fConst0));
		fConst13 = (0.061156102924877762f * fConst0);
		fConst14 = (8.1757989159999997f / fConst0);
		fConst15 = faustpower<2>((2827.4333882308138f / fConst0));
		fConst16 = (2827.4333882308138f * (((fConst15 * ((0.20330000000000001f * fConst15) + 0.31755f)) + 1.0f) / fConst0));
		fConst17 = faustpower<2>((5340.7075111026479f / fConst0));
		fConst18 = (5340.7075111026479f * (((fConst17 * ((0.20330000000000001f * fConst17) + 0.31755f)) + 1.0f) / fConst0));
		fConst19 = faustpower<2>((62.831853071795862f / fConst0));
		fConst20 = ((fConst19 * ((0.20330000000000001f * fConst19) + 0.31755f)) + 1.0f);
		fConst21 = faustpower<2>(fConst20);
		fConst22 = (2.0f * ((3947.8417604357433f * (fConst21 / fConst1)) + -1.0f));
		fConst23 = (3947.8417604357433f * (fConst20 / fConst0));
		fConst24 = (((fConst20 * (fConst23 + -88.495567706754741f)) / fConst0) + 1.0f);
		fConst25 = (((fConst20 * (fConst23 + 88.495567706754741f)) / fConst0) + 1.0f);
		fConst26 = (1.0f / fConst25);
		fConst27 = (fConst21 / (fConst1 * fConst25));
		fConst28 = (3000.0f * fConst27);
		fConst29 = (3.1415926535897931f / fConst0);
		fConst30 = (0.80000000000000004f * fConst27);
		fConst31 = faustpower<2>((314.15926535897933f / fConst0));
		fConst32 = ((fConst31 * ((0.20330000000000001f * fConst31) + 0.31755f)) + 1.0f);
		fConst33 = faustpower<2>(fConst32);
		fConst34 = (2.0f * ((98696.044010893587f * (fConst33 / fConst1)) + -1.0f));
		fConst35 = (98696.044010893587f * (fConst32 / fConst0));
		fConst36 = (((fConst32 * (fConst35 + -314.15926535897933f)) / fConst0) + 1.0f);
		fConst37 = (((fConst32 * (fConst35 + 314.15926535897933f)) / fConst0) + 1.0f);
		fConst38 = (1.0f / fConst37);
		fConst39 = (0.010937499999999999f / fConst0);
		fConst40 = (fConst33 / (fConst1 * fConst37));
		fConst41 = faustpower<2>((125.66370614359172f / fConst0));
		fConst42 = ((fConst41 * ((0.20330000000000001f * fConst41) + 0.31755f)) + 1.0f);
		fConst43 = faustpower<2>(fConst42);
		fConst44 = (2.0f * ((15791.367041742973f * (fConst43 / fConst1)) + -1.0f));
		fConst45 = (15791.367041742973f * (fConst42 / fConst0));
		fConst46 = (((fConst42 * (fConst45 + -232.71056693257725f)) / fConst0) + 1.0f);
		fConst47 = (((fConst42 * (fConst45 + 232.71056693257725f)) / fConst0) + 1.0f);
		fConst48 = (1.0f / fConst47);
		fConst49 = (((fConst42 * (fConst45 + -95.926493239382992f)) / fConst0) + 1.0f);
		fConst50 = (((fConst42 * (fConst45 + 95.926493239382992f)) / fConst0) + 1.0f);
		fConst51 = (1.0f / fConst50);
		fConst52 = (fConst43 / (fConst1 * fConst50));
		fConst53 = (fConst43 / (fConst1 * fConst47));
		fConst54 = powf(0.5f,(10.0f / fConst0));
		fConst55 = (0 - (2.0f / fConst7));
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
		for (int i=0; i<2; i++) fRec5[i] = 0;
		for (int i=0; i<2; i++) fRec6[i] = 0;
		for (int i=0; i<2; i++) fRec12[i] = 0;
		for (int i=0; i<3; i++) fRec11[i] = 0;
		for (int i=0; i<3; i++) fRec15[i] = 0;
		for (int i=0; i<3; i++) fRec14[i] = 0;
		for (int i=0; i<2; i++) fRec13[i] = 0;
		for (int i=0; i<2; i++) fRec8[i] = 0;
		for (int i=0; i<2; i++) fRec9[i] = 0;
		for (int i=0; i<2; i++) fVec1[i] = 0;
		for (int i=0; i<2; i++) fRec17[i] = 0;
		for (int i=0; i<3; i++) fRec16[i] = 0;
		for (int i=0; i<3; i++) fRec18[i] = 0;
		for (int i=0; i<3; i++) fRec19[i] = 0;
		for (int i=0; i<2; i++) fRec20[i] = 0;
		for (int i=0; i<2; i++) fRec21[i] = 0;
		for (int i=0; i<2; i++) fRec27[i] = 0;
		for (int i=0; i<3; i++) fRec26[i] = 0;
		for (int i=0; i<2; i++) fRec23[i] = 0;
		for (int i=0; i<2; i++) fRec24[i] = 0;
		for (int i=0; i<2; i++) fVec2[i] = 0;
		for (int i=0; i<2; i++) fRec29[i] = 0;
		for (int i=0; i<3; i++) fRec28[i] = 0;
		for (int i=0; i<3; i++) fRec30[i] = 0;
		for (int i=0; i<3; i++) fRec31[i] = 0;
		for (int i=0; i<2; i++) fRec32[i] = 0;
		for (int i=0; i<2; i++) fRec33[i] = 0;
		for (int i=0; i<2; i++) fRec39[i] = 0;
		for (int i=0; i<3; i++) fRec38[i] = 0;
		for (int i=0; i<2; i++) fRec35[i] = 0;
		for (int i=0; i<2; i++) fRec36[i] = 0;
		for (int i=0; i<2; i++) fVec3[i] = 0;
		for (int i=0; i<2; i++) fRec41[i] = 0;
		for (int i=0; i<3; i++) fRec40[i] = 0;
		for (int i=0; i<3; i++) fRec42[i] = 0;
		for (int i=0; i<3; i++) fRec43[i] = 0;
		for (int i=0; i<2; i++) fRec44[i] = 0;
		for (int i=0; i<2; i++) fRec45[i] = 0;
		for (int i=0; i<2; i++) fRec51[i] = 0;
		for (int i=0; i<3; i++) fRec50[i] = 0;
		for (int i=0; i<2; i++) fRec47[i] = 0;
		for (int i=0; i<2; i++) fRec48[i] = 0;
		for (int i=0; i<2; i++) fVec4[i] = 0;
		for (int i=0; i<2; i++) fRec53[i] = 0;
		for (int i=0; i<3; i++) fRec52[i] = 0;
		for (int i=0; i<3; i++) fRec54[i] = 0;
		for (int i=0; i<3; i++) fRec55[i] = 0;
		for (int i=0; i<2; i++) fRec56[i] = 0;
		for (int i=0; i<2; i++) fRec57[i] = 0;
		for (int i=0; i<2; i++) fRec63[i] = 0;
		for (int i=0; i<3; i++) fRec62[i] = 0;
		for (int i=0; i<2; i++) fRec59[i] = 0;
		for (int i=0; i<2; i++) fRec60[i] = 0;
		for (int i=0; i<2; i++) fVec5[i] = 0;
		for (int i=0; i<2; i++) fRec65[i] = 0;
		for (int i=0; i<3; i++) fRec64[i] = 0;
		for (int i=0; i<3; i++) fRec66[i] = 0;
		for (int i=0; i<3; i++) fRec67[i] = 0;
		for (int i=0; i<2; i++) fRec68[i] = 0;
		for (int i=0; i<2; i++) fRec69[i] = 0;
		for (int i=0; i<2; i++) fRec75[i] = 0;
		for (int i=0; i<3; i++) fRec74[i] = 0;
		for (int i=0; i<2; i++) fRec71[i] = 0;
		for (int i=0; i<2; i++) fRec72[i] = 0;
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
		synth_interface.rot_x = &fslider0;
		synth_interface.rot_y = &fslider2;
		synth_interface.rot_z = &fslider6;
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
		synth_interface.pres[5] = &fslider4;
		synth_interface.vpres[5] = &fslider5;
		synth_interface.but_y[5] = &fslider3;
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
		float 	fSlow13 = (0.125f * (fSlow2 * fSlow3));
		float 	fSlow14 = float(fslider2);
		float 	fSlow15 = ((int((fSlow14 > 0.0f)))?fConst18:fConst16);
		float 	fSlow16 = fabsf(fSlow14);
		float 	fSlow17 = ((0.5f * fSlow16) + 1.0f);
		float 	fSlow18 = (fSlow17 * fSlow15);
		float 	fSlow19 = ((fSlow17 * (fSlow15 * (fSlow18 + 0.20000000000000001f))) + 1.0f);
		float 	fSlow20 = ((faustpower<2>(fSlow17) * faustpower<2>(fSlow15)) + -1.0f);
		float 	fSlow21 = ((fSlow17 * (fSlow15 * (fSlow18 + -0.20000000000000001f))) + 1.0f);
		float 	fSlow22 = (0.20000000000000001f * (fSlow16 * fSlow17));
		float 	fSlow23 = float(fslider3);
		float 	fSlow24 = float(fslider4);
		float 	fSlow25 = (fConst28 * fSlow24);
		float 	fSlow26 = (fConst30 * fSlow24);
		float 	fSlow27 = max((float(fslider5) + -0.001f), (float)0);
		float 	fSlow28 = ((fSlow27 / (fSlow27 + 0.5f)) + (0.59999999999999998f * faustpower<2>(max((fSlow24 + -0.29999999999999999f), (float)0))));
		float 	fSlow29 = (1.0f - (fConst39 * (fSlow8 / max(min((16.0f * fSlow24), (0.5f * (fSlow24 + 1.0f))), 0.050000000000000003f))));
		float 	fSlow30 = max(((faustpower<2>(float(fslider6)) + faustpower<2>(fSlow14)) + -0.0050000000000000001f), (float)0);
		float 	fSlow31 = min((2.0f * fSlow24), fSlow30);
		float 	fSlow32 = float(fslider7);
		float 	fSlow33 = max((8.1757989159999997f * fSlow9), (float)200);
		float 	fSlow34 = float(fslider8);
		float 	fSlow35 = fastpow2((0.083333333333333329f * fSlow34));
		int 	iSlow36 = int(min((fConst13 / fSlow35), (float)4096));
		float 	fSlow37 = float(iSlow36);
		float 	fSlow38 = (fConst14 * (fSlow35 * fSlow37));
		float 	fSlow39 = float(fslider9);
		float 	fSlow40 = float(fslider10);
		float 	fSlow41 = (fConst28 * fSlow40);
		float 	fSlow42 = (fConst30 * fSlow40);
		float 	fSlow43 = max((float(fslider11) + -0.001f), (float)0);
		float 	fSlow44 = ((fSlow43 / (fSlow43 + 0.5f)) + (0.59999999999999998f * faustpower<2>(max((fSlow40 + -0.29999999999999999f), (float)0))));
		float 	fSlow45 = (1.0f - (fConst39 * (fSlow34 / max(min((16.0f * fSlow40), (0.5f * (fSlow40 + 1.0f))), 0.050000000000000003f))));
		float 	fSlow46 = min((2.0f * fSlow40), fSlow30);
		float 	fSlow47 = max((8.1757989159999997f * fSlow35), (float)200);
		float 	fSlow48 = float(fslider12);
		float 	fSlow49 = fastpow2((0.083333333333333329f * fSlow48));
		int 	iSlow50 = int(min((fConst13 / fSlow49), (float)4096));
		float 	fSlow51 = float(iSlow50);
		float 	fSlow52 = (fConst14 * (fSlow49 * fSlow51));
		float 	fSlow53 = float(fslider13);
		float 	fSlow54 = float(fslider14);
		float 	fSlow55 = (fConst28 * fSlow54);
		float 	fSlow56 = (fConst30 * fSlow54);
		float 	fSlow57 = max((float(fslider15) + -0.001f), (float)0);
		float 	fSlow58 = ((fSlow57 / (fSlow57 + 0.5f)) + (0.59999999999999998f * faustpower<2>(max((fSlow54 + -0.29999999999999999f), (float)0))));
		float 	fSlow59 = (1.0f - (fConst39 * (fSlow48 / max(min((16.0f * fSlow54), (0.5f * (fSlow54 + 1.0f))), 0.050000000000000003f))));
		float 	fSlow60 = min((2.0f * fSlow54), fSlow30);
		float 	fSlow61 = max((8.1757989159999997f * fSlow49), (float)200);
		float 	fSlow62 = float(fslider16);
		float 	fSlow63 = fastpow2((0.083333333333333329f * fSlow62));
		int 	iSlow64 = int(min((fConst13 / fSlow63), (float)4096));
		float 	fSlow65 = float(iSlow64);
		float 	fSlow66 = (fConst14 * (fSlow63 * fSlow65));
		float 	fSlow67 = float(fslider17);
		float 	fSlow68 = float(fslider18);
		float 	fSlow69 = (fConst28 * fSlow68);
		float 	fSlow70 = (fConst30 * fSlow68);
		float 	fSlow71 = max((float(fslider19) + -0.001f), (float)0);
		float 	fSlow72 = ((fSlow71 / (fSlow71 + 0.5f)) + (0.59999999999999998f * faustpower<2>(max((fSlow68 + -0.29999999999999999f), (float)0))));
		float 	fSlow73 = (1.0f - (fConst39 * (fSlow62 / max(min((16.0f * fSlow68), (0.5f * (fSlow68 + 1.0f))), 0.050000000000000003f))));
		float 	fSlow74 = min((2.0f * fSlow68), fSlow30);
		float 	fSlow75 = max((8.1757989159999997f * fSlow63), (float)200);
		float 	fSlow76 = float(fslider20);
		float 	fSlow77 = fastpow2((0.083333333333333329f * fSlow76));
		int 	iSlow78 = int(min((fConst13 / fSlow77), (float)4096));
		float 	fSlow79 = float(iSlow78);
		float 	fSlow80 = (fConst14 * (fSlow77 * fSlow79));
		float 	fSlow81 = float(fslider21);
		float 	fSlow82 = float(fslider22);
		float 	fSlow83 = (fConst28 * fSlow82);
		float 	fSlow84 = (fConst30 * fSlow82);
		float 	fSlow85 = max((float(fslider23) + -0.001f), (float)0);
		float 	fSlow86 = ((fSlow85 / (fSlow85 + 0.5f)) + (0.59999999999999998f * faustpower<2>(max((fSlow82 + -0.29999999999999999f), (float)0))));
		float 	fSlow87 = (1.0f - (fConst39 * (fSlow76 / max(min((16.0f * fSlow82), (0.5f * (fSlow82 + 1.0f))), 0.050000000000000003f))));
		float 	fSlow88 = min((2.0f * fSlow82), fSlow30);
		float 	fSlow89 = max((8.1757989159999997f * fSlow77), (float)200);
		float 	fSlow90 = float(fslider24);
		float 	fSlow91 = fastpow2((0.083333333333333329f * fSlow90));
		int 	iSlow92 = int(min((fConst13 / fSlow91), (float)4096));
		float 	fSlow93 = float(iSlow92);
		float 	fSlow94 = (fConst14 * (fSlow91 * fSlow93));
		float 	fSlow95 = float(fslider25);
		float 	fSlow96 = float(fslider26);
		float 	fSlow97 = (fConst28 * fSlow96);
		float 	fSlow98 = (fConst30 * fSlow96);
		float 	fSlow99 = max((float(fslider27) + -0.001f), (float)0);
		float 	fSlow100 = ((fSlow99 / (fSlow99 + 0.5f)) + (0.59999999999999998f * faustpower<2>(max((fSlow96 + -0.29999999999999999f), (float)0))));
		float 	fSlow101 = (1.0f - (fConst39 * (fSlow90 / max(min((16.0f * fSlow96), (0.5f * (fSlow96 + 1.0f))), 0.050000000000000003f))));
		float 	fSlow102 = min((2.0f * fSlow96), fSlow30);
		float 	fSlow103 = max((8.1757989159999997f * fSlow91), (float)200);
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
			fRec4[0] = (fSlow23 - (fConst26 * ((fConst24 * fRec4[2]) + (fConst22 * fRec4[1]))));
			float 	fTemp3 = (((3947.8417604357433f * fRec4[0]) + (7895.6835208714865f * fRec4[1])) + (3947.8417604357433f * fRec4[2]));
			float 	fTemp4 = (fConst27 * fTemp3);
			float 	fTemp5 = max(fTemp4, (float)0);
			float 	fTemp6 = tanf((fConst29 * ((fSlow25 * (fTemp3 * fTemp5)) + 200.0f)));
			float 	fTemp7 = ((fTemp6 * (fTemp6 + 0.5f)) + 1.0f);
			float 	fTemp8 = (fRec5[1] + (fTemp6 * (fTemp2 - fRec6[1])));
			float 	fTemp9 = (fTemp8 / fTemp7);
			fRec5[0] = ((2.0f * fTemp9) - fRec5[1]);
			float 	fTemp10 = (fRec6[1] + ((fTemp6 * fTemp8) / fTemp7));
			fRec6[0] = ((2.0f * fTemp10) - fRec6[1]);
			float 	fRec7 = fTemp9;
			float 	fTemp11 = max((-1 * fTemp4), (float)0);
			fRec12[0] = (fSlow29 * (max(fRec12[1], fSlow28) * float((fabsf((fSlow8 - fVec0[1])) < 1.0f))));
			fRec11[0] = (max(fRec12[0], fSlow31) - (fConst38 * ((fConst36 * fRec11[2]) + (fConst34 * fRec11[1]))));
			float 	fTemp12 = (((98696.044010893587f * fRec11[0]) + (197392.08802178717f * fRec11[1])) + (98696.044010893587f * fRec11[2]));
			fRec15[0] = (fSlow32 - (fConst51 * ((fConst49 * fRec15[2]) + (fConst44 * fRec15[1]))));
			fRec14[0] = ((fConst52 * (((15791.367041742973f * fRec15[0]) + (31582.734083485946f * fRec15[1])) + (15791.367041742973f * fRec15[2]))) - (fConst48 * ((fConst46 * fRec14[2]) + (fConst44 * fRec14[1]))));
			fRec13[0] = (fConst54 * max(fRec13[1], min((fConst53 * (((15791.367041742973f * fRec14[0]) + (31582.734083485946f * fRec14[1])) + (15791.367041742973f * fRec14[2]))), (float)2)));
			float 	fTemp13 = max((fRec13[0] + -1.0f), (float)0);
			float 	fTemp14 = tanf((fConst29 * min((fSlow33 * (faustpower<2>(faustpower<2>((fTemp13 + (fConst40 * (fTemp12 * (1.0f - fTemp11)))))) + 1.0f)), (float)16000)));
			float 	fTemp15 = ((fTemp14 * (fTemp14 + (1.0f / ((8.0f * fTemp11) + 1.0f)))) + 1.0f);
			float 	fTemp16 = (fRec8[1] + (fTemp14 * (fTemp2 - fRec9[1])));
			float 	fTemp17 = (fTemp16 / fTemp15);
			fRec8[0] = ((2.0f * fTemp17) - fRec8[1]);
			float 	fTemp18 = (fRec9[1] + ((fTemp14 * fTemp16) / fTemp15));
			fRec9[0] = ((2.0f * fTemp18) - fRec9[1]);
			float 	fRec10 = fTemp18;
			fVec1[0] = fSlow34;
			fRec17[0] = (fSlow38 + (fRec17[1] - float((iSlow36 * ((fSlow38 + fRec17[1]) > fSlow37)))));
			int 	iTemp19 = int(fRec17[0]);
			float 	fTemp20 = ftbl0[iTemp19];
			float 	fTemp21 = (fTemp20 + ((fRec17[0] - float(iTemp19)) * (ftbl0[((iTemp19 + 1) % iSlow36)] - fTemp20)));
			fRec16[0] = (fTemp21 - (((fSlow7 * fRec16[2]) + (2.0f * (fSlow6 * fRec16[1]))) / fSlow5));
			fRec18[0] = (fTemp21 - (((fSlow21 * fRec18[2]) + (2.0f * (fSlow20 * fRec18[1]))) / fSlow19));
			fRec19[0] = (fSlow39 - (fConst26 * ((fConst24 * fRec19[2]) + (fConst22 * fRec19[1]))));
			float 	fTemp22 = (((3947.8417604357433f * fRec19[0]) + (7895.6835208714865f * fRec19[1])) + (3947.8417604357433f * fRec19[2]));
			float 	fTemp23 = (fConst27 * fTemp22);
			float 	fTemp24 = max(fTemp23, (float)0);
			float 	fTemp25 = tanf((fConst29 * ((fSlow41 * (fTemp22 * fTemp24)) + 200.0f)));
			float 	fTemp26 = ((fTemp25 * (fTemp25 + 0.5f)) + 1.0f);
			float 	fTemp27 = (fRec20[1] + (fTemp25 * (fTemp21 - fRec21[1])));
			float 	fTemp28 = (fTemp27 / fTemp26);
			fRec20[0] = ((2.0f * fTemp28) - fRec20[1]);
			float 	fTemp29 = (fRec21[1] + ((fTemp25 * fTemp27) / fTemp26));
			fRec21[0] = ((2.0f * fTemp29) - fRec21[1]);
			float 	fRec22 = fTemp28;
			float 	fTemp30 = max((-1 * fTemp23), (float)0);
			fRec27[0] = (fSlow45 * (max(fRec27[1], fSlow44) * float((fabsf((fSlow34 - fVec1[1])) < 1.0f))));
			fRec26[0] = (max(fRec27[0], fSlow46) - (fConst38 * ((fConst36 * fRec26[2]) + (fConst34 * fRec26[1]))));
			float 	fTemp31 = (((98696.044010893587f * fRec26[0]) + (197392.08802178717f * fRec26[1])) + (98696.044010893587f * fRec26[2]));
			float 	fTemp32 = tanf((fConst29 * min((fSlow47 * (faustpower<2>(faustpower<2>((fTemp13 + (fConst40 * (fTemp31 * (1.0f - fTemp30)))))) + 1.0f)), (float)16000)));
			float 	fTemp33 = ((fTemp32 * (fTemp32 + (1.0f / ((8.0f * fTemp30) + 1.0f)))) + 1.0f);
			float 	fTemp34 = (fRec23[1] + (fTemp32 * (fTemp21 - fRec24[1])));
			float 	fTemp35 = (fTemp34 / fTemp33);
			fRec23[0] = ((2.0f * fTemp35) - fRec23[1]);
			float 	fTemp36 = (fRec24[1] + ((fTemp32 * fTemp34) / fTemp33));
			fRec24[0] = ((2.0f * fTemp36) - fRec24[1]);
			float 	fRec25 = fTemp36;
			fVec2[0] = fSlow48;
			fRec29[0] = (fSlow52 + (fRec29[1] - float((iSlow50 * ((fSlow52 + fRec29[1]) > fSlow51)))));
			int 	iTemp37 = int(fRec29[0]);
			float 	fTemp38 = ftbl0[iTemp37];
			float 	fTemp39 = (fTemp38 + ((fRec29[0] - float(iTemp37)) * (ftbl0[((iTemp37 + 1) % iSlow50)] - fTemp38)));
			fRec28[0] = (fTemp39 - (((fSlow7 * fRec28[2]) + (2.0f * (fSlow6 * fRec28[1]))) / fSlow5));
			fRec30[0] = (fTemp39 - (((fSlow21 * fRec30[2]) + (2.0f * (fSlow20 * fRec30[1]))) / fSlow19));
			fRec31[0] = (fSlow53 - (fConst26 * ((fConst24 * fRec31[2]) + (fConst22 * fRec31[1]))));
			float 	fTemp40 = (((3947.8417604357433f * fRec31[0]) + (7895.6835208714865f * fRec31[1])) + (3947.8417604357433f * fRec31[2]));
			float 	fTemp41 = (fConst27 * fTemp40);
			float 	fTemp42 = max(fTemp41, (float)0);
			float 	fTemp43 = tanf((fConst29 * ((fSlow55 * (fTemp40 * fTemp42)) + 200.0f)));
			float 	fTemp44 = ((fTemp43 * (fTemp43 + 0.5f)) + 1.0f);
			float 	fTemp45 = (fRec32[1] + (fTemp43 * (fTemp39 - fRec33[1])));
			float 	fTemp46 = (fTemp45 / fTemp44);
			fRec32[0] = ((2.0f * fTemp46) - fRec32[1]);
			float 	fTemp47 = (fRec33[1] + ((fTemp43 * fTemp45) / fTemp44));
			fRec33[0] = ((2.0f * fTemp47) - fRec33[1]);
			float 	fRec34 = fTemp46;
			float 	fTemp48 = max((-1 * fTemp41), (float)0);
			fRec39[0] = (fSlow59 * (max(fRec39[1], fSlow58) * float((fabsf((fSlow48 - fVec2[1])) < 1.0f))));
			fRec38[0] = (max(fRec39[0], fSlow60) - (fConst38 * ((fConst36 * fRec38[2]) + (fConst34 * fRec38[1]))));
			float 	fTemp49 = (((98696.044010893587f * fRec38[0]) + (197392.08802178717f * fRec38[1])) + (98696.044010893587f * fRec38[2]));
			float 	fTemp50 = tanf((fConst29 * min((fSlow61 * (faustpower<2>(faustpower<2>((fTemp13 + (fConst40 * (fTemp49 * (1.0f - fTemp48)))))) + 1.0f)), (float)16000)));
			float 	fTemp51 = ((fTemp50 * (fTemp50 + (1.0f / ((8.0f * fTemp48) + 1.0f)))) + 1.0f);
			float 	fTemp52 = (fRec35[1] + (fTemp50 * (fTemp39 - fRec36[1])));
			float 	fTemp53 = (fTemp52 / fTemp51);
			fRec35[0] = ((2.0f * fTemp53) - fRec35[1]);
			float 	fTemp54 = (fRec36[1] + ((fTemp50 * fTemp52) / fTemp51));
			fRec36[0] = ((2.0f * fTemp54) - fRec36[1]);
			float 	fRec37 = fTemp54;
			fVec3[0] = fSlow62;
			fRec41[0] = (fSlow66 + (fRec41[1] - float((iSlow64 * ((fSlow66 + fRec41[1]) > fSlow65)))));
			int 	iTemp55 = int(fRec41[0]);
			float 	fTemp56 = ftbl0[iTemp55];
			float 	fTemp57 = (fTemp56 + ((fRec41[0] - float(iTemp55)) * (ftbl0[((iTemp55 + 1) % iSlow64)] - fTemp56)));
			fRec40[0] = (fTemp57 - (((fSlow7 * fRec40[2]) + (2.0f * (fSlow6 * fRec40[1]))) / fSlow5));
			fRec42[0] = (fTemp57 - (((fSlow21 * fRec42[2]) + (2.0f * (fSlow20 * fRec42[1]))) / fSlow19));
			fRec43[0] = (fSlow67 - (fConst26 * ((fConst24 * fRec43[2]) + (fConst22 * fRec43[1]))));
			float 	fTemp58 = (((3947.8417604357433f * fRec43[0]) + (7895.6835208714865f * fRec43[1])) + (3947.8417604357433f * fRec43[2]));
			float 	fTemp59 = (fConst27 * fTemp58);
			float 	fTemp60 = max(fTemp59, (float)0);
			float 	fTemp61 = tanf((fConst29 * ((fSlow69 * (fTemp58 * fTemp60)) + 200.0f)));
			float 	fTemp62 = ((fTemp61 * (fTemp61 + 0.5f)) + 1.0f);
			float 	fTemp63 = (fRec44[1] + (fTemp61 * (fTemp57 - fRec45[1])));
			float 	fTemp64 = (fTemp63 / fTemp62);
			fRec44[0] = ((2.0f * fTemp64) - fRec44[1]);
			float 	fTemp65 = (fRec45[1] + ((fTemp61 * fTemp63) / fTemp62));
			fRec45[0] = ((2.0f * fTemp65) - fRec45[1]);
			float 	fRec46 = fTemp64;
			float 	fTemp66 = max((-1 * fTemp59), (float)0);
			fRec51[0] = (fSlow73 * (max(fRec51[1], fSlow72) * float((fabsf((fSlow62 - fVec3[1])) < 1.0f))));
			fRec50[0] = (max(fRec51[0], fSlow74) - (fConst38 * ((fConst36 * fRec50[2]) + (fConst34 * fRec50[1]))));
			float 	fTemp67 = (((98696.044010893587f * fRec50[0]) + (197392.08802178717f * fRec50[1])) + (98696.044010893587f * fRec50[2]));
			float 	fTemp68 = tanf((fConst29 * min((fSlow75 * (faustpower<2>(faustpower<2>((fTemp13 + (fConst40 * (fTemp67 * (1.0f - fTemp66)))))) + 1.0f)), (float)16000)));
			float 	fTemp69 = ((fTemp68 * (fTemp68 + (1.0f / ((8.0f * fTemp66) + 1.0f)))) + 1.0f);
			float 	fTemp70 = (fRec47[1] + (fTemp68 * (fTemp57 - fRec48[1])));
			float 	fTemp71 = (fTemp70 / fTemp69);
			fRec47[0] = ((2.0f * fTemp71) - fRec47[1]);
			float 	fTemp72 = (fRec48[1] + ((fTemp68 * fTemp70) / fTemp69));
			fRec48[0] = ((2.0f * fTemp72) - fRec48[1]);
			float 	fRec49 = fTemp72;
			fVec4[0] = fSlow76;
			fRec53[0] = (fSlow80 + (fRec53[1] - float((iSlow78 * ((fSlow80 + fRec53[1]) > fSlow79)))));
			int 	iTemp73 = int(fRec53[0]);
			float 	fTemp74 = ftbl0[iTemp73];
			float 	fTemp75 = (fTemp74 + ((fRec53[0] - float(iTemp73)) * (ftbl0[((iTemp73 + 1) % iSlow78)] - fTemp74)));
			fRec52[0] = (fTemp75 - (((fSlow7 * fRec52[2]) + (2.0f * (fSlow6 * fRec52[1]))) / fSlow5));
			fRec54[0] = (fTemp75 - (((fSlow21 * fRec54[2]) + (2.0f * (fSlow20 * fRec54[1]))) / fSlow19));
			fRec55[0] = (fSlow81 - (fConst26 * ((fConst24 * fRec55[2]) + (fConst22 * fRec55[1]))));
			float 	fTemp76 = (((3947.8417604357433f * fRec55[0]) + (7895.6835208714865f * fRec55[1])) + (3947.8417604357433f * fRec55[2]));
			float 	fTemp77 = (fConst27 * fTemp76);
			float 	fTemp78 = max(fTemp77, (float)0);
			float 	fTemp79 = tanf((fConst29 * ((fSlow83 * (fTemp76 * fTemp78)) + 200.0f)));
			float 	fTemp80 = ((fTemp79 * (fTemp79 + 0.5f)) + 1.0f);
			float 	fTemp81 = (fRec56[1] + (fTemp79 * (fTemp75 - fRec57[1])));
			float 	fTemp82 = (fTemp81 / fTemp80);
			fRec56[0] = ((2.0f * fTemp82) - fRec56[1]);
			float 	fTemp83 = (fRec57[1] + ((fTemp79 * fTemp81) / fTemp80));
			fRec57[0] = ((2.0f * fTemp83) - fRec57[1]);
			float 	fRec58 = fTemp82;
			float 	fTemp84 = max((-1 * fTemp77), (float)0);
			fRec63[0] = (fSlow87 * (max(fRec63[1], fSlow86) * float((fabsf((fSlow76 - fVec4[1])) < 1.0f))));
			fRec62[0] = (max(fRec63[0], fSlow88) - (fConst38 * ((fConst36 * fRec62[2]) + (fConst34 * fRec62[1]))));
			float 	fTemp85 = (((98696.044010893587f * fRec62[0]) + (197392.08802178717f * fRec62[1])) + (98696.044010893587f * fRec62[2]));
			float 	fTemp86 = tanf((fConst29 * min((fSlow89 * (faustpower<2>(faustpower<2>((fTemp13 + (fConst40 * (fTemp85 * (1.0f - fTemp84)))))) + 1.0f)), (float)16000)));
			float 	fTemp87 = ((fTemp86 * (fTemp86 + (1.0f / ((8.0f * fTemp84) + 1.0f)))) + 1.0f);
			float 	fTemp88 = (fRec59[1] + (fTemp86 * (fTemp75 - fRec60[1])));
			float 	fTemp89 = (fTemp88 / fTemp87);
			fRec59[0] = ((2.0f * fTemp89) - fRec59[1]);
			float 	fTemp90 = (fRec60[1] + ((fTemp86 * fTemp88) / fTemp87));
			fRec60[0] = ((2.0f * fTemp90) - fRec60[1]);
			float 	fRec61 = fTemp90;
			fVec5[0] = fSlow90;
			fRec65[0] = (fSlow94 + (fRec65[1] - float((iSlow92 * ((fSlow94 + fRec65[1]) > fSlow93)))));
			int 	iTemp91 = int(fRec65[0]);
			float 	fTemp92 = ftbl0[iTemp91];
			float 	fTemp93 = (fTemp92 + ((fRec65[0] - float(iTemp91)) * (ftbl0[((iTemp91 + 1) % iSlow92)] - fTemp92)));
			fRec64[0] = (fTemp93 - (((fSlow7 * fRec64[2]) + (2.0f * (fSlow6 * fRec64[1]))) / fSlow5));
			fRec66[0] = (fTemp93 - (((fSlow21 * fRec66[2]) + (2.0f * (fSlow20 * fRec66[1]))) / fSlow19));
			fRec67[0] = (fSlow95 - (fConst26 * ((fConst24 * fRec67[2]) + (fConst22 * fRec67[1]))));
			float 	fTemp94 = (((3947.8417604357433f * fRec67[0]) + (7895.6835208714865f * fRec67[1])) + (3947.8417604357433f * fRec67[2]));
			float 	fTemp95 = (fConst27 * fTemp94);
			float 	fTemp96 = max(fTemp95, (float)0);
			float 	fTemp97 = tanf((fConst29 * ((fSlow97 * (fTemp94 * fTemp96)) + 200.0f)));
			float 	fTemp98 = ((fTemp97 * (fTemp97 + 0.5f)) + 1.0f);
			float 	fTemp99 = (fRec68[1] + (fTemp97 * (fTemp93 - fRec69[1])));
			float 	fTemp100 = (fTemp99 / fTemp98);
			fRec68[0] = ((2.0f * fTemp100) - fRec68[1]);
			float 	fTemp101 = (fRec69[1] + ((fTemp97 * fTemp99) / fTemp98));
			fRec69[0] = ((2.0f * fTemp101) - fRec69[1]);
			float 	fRec70 = fTemp100;
			float 	fTemp102 = max((-1 * fTemp95), (float)0);
			fRec75[0] = (fSlow101 * (max(fRec75[1], fSlow100) * float((fabsf((fSlow90 - fVec5[1])) < 1.0f))));
			fRec74[0] = (max(fRec75[0], fSlow102) - (fConst38 * ((fConst36 * fRec74[2]) + (fConst34 * fRec74[1]))));
			float 	fTemp103 = (((98696.044010893587f * fRec74[0]) + (197392.08802178717f * fRec74[1])) + (98696.044010893587f * fRec74[2]));
			float 	fTemp104 = tanf((fConst29 * min((fSlow103 * (faustpower<2>(faustpower<2>(((fConst40 * (fTemp103 * (1.0f - fTemp102))) + fTemp13))) + 1.0f)), (float)16000)));
			float 	fTemp105 = ((fTemp104 * (fTemp104 + (1.0f / ((8.0f * fTemp102) + 1.0f)))) + 1.0f);
			float 	fTemp106 = (fRec71[1] + (fTemp104 * (fTemp93 - fRec72[1])));
			float 	fTemp107 = (fTemp106 / fTemp105);
			fRec71[0] = ((2.0f * fTemp107) - fRec71[1]);
			float 	fTemp108 = (fRec72[1] + ((fTemp104 * fTemp106) / fTemp105));
			fRec72[0] = ((2.0f * fTemp108) - fRec72[1]);
			float 	fRec73 = fTemp108;
			fRec0[0] = ((fConst40 * ((((((fTemp103 * (((fRec73 * faustpower<2>((1.0f - (0.5f * fTemp102)))) + (fSlow98 * ((fRec70 * fTemp94) * fTemp96))) + (6.0f * ((fSlow22 * ((fSlow15 * (fRec66[0] - fRec66[2])) / fSlow19)) + (fSlow13 * ((fSlow1 * (fRec64[0] - fRec64[2])) / fSlow5)))))) + (fTemp85 * (((fRec61 * faustpower<2>((1.0f - (0.5f * fTemp84)))) + (fSlow84 * ((fRec58 * fTemp76) * fTemp78))) + (6.0f * ((fSlow22 * ((fSlow15 * (fRec54[0] - fRec54[2])) / fSlow19)) + (fSlow13 * ((fSlow1 * (fRec52[0] - fRec52[2])) / fSlow5))))))) + (fTemp67 * (((fRec49 * faustpower<2>((1.0f - (0.5f * fTemp66)))) + (fSlow70 * ((fRec46 * fTemp58) * fTemp60))) + (6.0f * ((fSlow22 * ((fSlow15 * (fRec42[0] - fRec42[2])) / fSlow19)) + (fSlow13 * ((fSlow1 * (fRec40[0] - fRec40[2])) / fSlow5))))))) + (fTemp49 * (((fRec37 * faustpower<2>((1.0f - (0.5f * fTemp48)))) + (fSlow56 * ((fRec34 * fTemp40) * fTemp42))) + (6.0f * ((fSlow22 * ((fSlow15 * (fRec30[0] - fRec30[2])) / fSlow19)) + (fSlow13 * ((fSlow1 * (fRec28[0] - fRec28[2])) / fSlow5))))))) + (fTemp31 * (((fRec25 * faustpower<2>((1.0f - (0.5f * fTemp30)))) + (fSlow42 * ((fRec22 * fTemp22) * fTemp24))) + (6.0f * ((fSlow22 * ((fSlow15 * (fRec18[0] - fRec18[2])) / fSlow19)) + (fSlow13 * ((fSlow1 * (fRec16[0] - fRec16[2])) / fSlow5))))))) + (fTemp12 * (((fRec10 * faustpower<2>((1.0f - (0.5f * fTemp11)))) + (fSlow26 * ((fRec7 * fTemp3) * fTemp5))) + (6.0f * ((fSlow22 * ((fSlow15 * (fRec3[0] - fRec3[2])) / fSlow19)) + (fSlow13 * ((fSlow1 * (fRec1[0] - fRec1[2])) / fSlow5)))))))) - (fConst8 * ((fConst6 * fRec0[2]) + (fConst4 * fRec0[1]))));
			output0[i] = (FAUSTFLOAT)((fConst55 * fRec0[1]) + (fConst8 * (fRec0[0] + fRec0[2])));
			// post processing
			fRec0[2] = fRec0[1]; fRec0[1] = fRec0[0];
			fRec72[1] = fRec72[0];
			fRec71[1] = fRec71[0];
			fRec74[2] = fRec74[1]; fRec74[1] = fRec74[0];
			fRec75[1] = fRec75[0];
			fRec69[1] = fRec69[0];
			fRec68[1] = fRec68[0];
			fRec67[2] = fRec67[1]; fRec67[1] = fRec67[0];
			fRec66[2] = fRec66[1]; fRec66[1] = fRec66[0];
			fRec64[2] = fRec64[1]; fRec64[1] = fRec64[0];
			fRec65[1] = fRec65[0];
			fVec5[1] = fVec5[0];
			fRec60[1] = fRec60[0];
			fRec59[1] = fRec59[0];
			fRec62[2] = fRec62[1]; fRec62[1] = fRec62[0];
			fRec63[1] = fRec63[0];
			fRec57[1] = fRec57[0];
			fRec56[1] = fRec56[0];
			fRec55[2] = fRec55[1]; fRec55[1] = fRec55[0];
			fRec54[2] = fRec54[1]; fRec54[1] = fRec54[0];
			fRec52[2] = fRec52[1]; fRec52[1] = fRec52[0];
			fRec53[1] = fRec53[0];
			fVec4[1] = fVec4[0];
			fRec48[1] = fRec48[0];
			fRec47[1] = fRec47[0];
			fRec50[2] = fRec50[1]; fRec50[1] = fRec50[0];
			fRec51[1] = fRec51[0];
			fRec45[1] = fRec45[0];
			fRec44[1] = fRec44[0];
			fRec43[2] = fRec43[1]; fRec43[1] = fRec43[0];
			fRec42[2] = fRec42[1]; fRec42[1] = fRec42[0];
			fRec40[2] = fRec40[1]; fRec40[1] = fRec40[0];
			fRec41[1] = fRec41[0];
			fVec3[1] = fVec3[0];
			fRec36[1] = fRec36[0];
			fRec35[1] = fRec35[0];
			fRec38[2] = fRec38[1]; fRec38[1] = fRec38[0];
			fRec39[1] = fRec39[0];
			fRec33[1] = fRec33[0];
			fRec32[1] = fRec32[0];
			fRec31[2] = fRec31[1]; fRec31[1] = fRec31[0];
			fRec30[2] = fRec30[1]; fRec30[1] = fRec30[0];
			fRec28[2] = fRec28[1]; fRec28[1] = fRec28[0];
			fRec29[1] = fRec29[0];
			fVec2[1] = fVec2[0];
			fRec24[1] = fRec24[0];
			fRec23[1] = fRec23[0];
			fRec26[2] = fRec26[1]; fRec26[1] = fRec26[0];
			fRec27[1] = fRec27[0];
			fRec21[1] = fRec21[0];
			fRec20[1] = fRec20[0];
			fRec19[2] = fRec19[1]; fRec19[1] = fRec19[0];
			fRec18[2] = fRec18[1]; fRec18[1] = fRec18[0];
			fRec16[2] = fRec16[1]; fRec16[1] = fRec16[0];
			fRec17[1] = fRec17[0];
			fVec1[1] = fVec1[0];
			fRec9[1] = fRec9[0];
			fRec8[1] = fRec8[0];
			fRec13[1] = fRec13[0];
			fRec14[2] = fRec14[1]; fRec14[1] = fRec14[0];
			fRec15[2] = fRec15[1]; fRec15[1] = fRec15[0];
			fRec11[2] = fRec11[1]; fRec11[1] = fRec11[0];
			fRec12[1] = fRec12[0];
			fRec6[1] = fRec6[0];
			fRec5[1] = fRec5[0];
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
	float power = 0.0;

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

		// convert float to int with scale, clamp and round
		for (int n = 0; n < CHANNEL_BUFFER_SIZE; n++) {
			tmp = (int32_t)(output0[n] * volume * MAX_VAL);
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
