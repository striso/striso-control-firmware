//----------------------------------------------------------
// name: "synth.dsp.tmp"
//
// Code generated with Faust 2.27.2 (https://faust.grame.fr)
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
	FAUSTFLOAT 	fslider0;
	FAUSTFLOAT 	fslider1;
	float 	fConst9;
	float 	fConst10;
	float 	fConst11;
	float 	fConst12;
	static float 	ftbl0[4096];
	FAUSTFLOAT 	fslider2;
	float 	fVec0[2];
	float 	fConst13;
	float 	fConst14;
	float 	fRec2[2];
	float 	fRec1[3];
	float 	fConst15;
	float 	fConst16;
	float 	fConst17;
	float 	fConst18;
	float 	fConst19;
	float 	fConst20;
	float 	fConst21;
	float 	fConst22;
	float 	fConst23;
	FAUSTFLOAT 	fslider3;
	float 	fConst24;
	float 	fRec4[2];
	FAUSTFLOAT 	fslider4;
	FAUSTFLOAT 	fslider5;
	float 	fRec3[3];
	float 	fConst25;
	float 	fConst26;
	float 	fConst27;
	float 	fConst28;
	float 	fConst29;
	float 	fConst30;
	float 	fConst31;
	float 	fConst32;
	float 	fConst33;
	float 	fConst34;
	float 	fConst35;
	float 	fConst36;
	FAUSTFLOAT 	fslider6;
	float 	fRec7[3];
	float 	fConst37;
	float 	fRec6[3];
	float 	fConst38;
	float 	fConst39;
	float 	fRec5[2];
	float 	fRec8[3];
	FAUSTFLOAT 	fslider7;
	FAUSTFLOAT 	fslider8;
	FAUSTFLOAT 	fslider9;
	float 	fVec1[2];
	float 	fRec10[2];
	float 	fRec9[3];
	FAUSTFLOAT 	fslider10;
	float 	fRec12[2];
	float 	fRec11[3];
	float 	fRec13[3];
	FAUSTFLOAT 	fslider11;
	FAUSTFLOAT 	fslider12;
	FAUSTFLOAT 	fslider13;
	float 	fVec2[2];
	float 	fRec15[2];
	float 	fRec14[3];
	FAUSTFLOAT 	fslider14;
	float 	fRec17[2];
	float 	fRec16[3];
	float 	fRec18[3];
	FAUSTFLOAT 	fslider15;
	FAUSTFLOAT 	fslider16;
	FAUSTFLOAT 	fslider17;
	float 	fVec3[2];
	float 	fRec20[2];
	float 	fRec19[3];
	FAUSTFLOAT 	fslider18;
	float 	fRec22[2];
	float 	fRec21[3];
	float 	fRec23[3];
	float 	fRec0[3];
	float 	fConst40;
	int fSampleRate;

  public:
	virtual void metadata(Meta* m) { 
		m->declare("fast.lib/author", "Piers Titus van der Torren (pierstitus@toverlamp.org)");
		m->declare("fast.lib/licence", "Apache-2.0");
		m->declare("fast.lib/name", "Faust Fast Approximations Library");
		m->declare("filename", "synth.dsp.tmp");
		m->declare("maths.lib/author", "GRAME");
		m->declare("maths.lib/copyright", "GRAME");
		m->declare("maths.lib/license", "LGPL with exception");
		m->declare("maths.lib/name", "Faust Math Library");
		m->declare("maths.lib/version", "2.3");
		m->declare("name", "synth.dsp.tmp");
		m->declare("platform.lib/name", "Generic Platform Library");
		m->declare("platform.lib/version", "0.1");
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
		fConst9 = (3.1415926535897931f / fConst0);
		fConst10 = (9.869604401089358f / fConst1);
		fConst11 = (9.869604401089358f / fConst0);
		fConst12 = (1.0f / fConst0);
		fConst13 = (0.061156102924877762f * fConst0);
		fConst14 = (8.1757989159999997f / fConst0);
		fConst15 = (1.2566370614359172f / fConst0);
		fConst16 = faustpower<2>((314.15926535897933f / fConst0));
		fConst17 = ((fConst16 * ((0.20330000000000001f * fConst16) + 0.31755f)) + 1.0f);
		fConst18 = faustpower<2>(fConst17);
		fConst19 = (2.0f * ((98696.044010893587f * (fConst18 / fConst1)) + -1.0f));
		fConst20 = (98696.044010893587f * (fConst17 / fConst0));
		fConst21 = (((fConst17 * (fConst20 + -314.15926535897933f)) / fConst0) + 1.0f);
		fConst22 = (((fConst17 * (fConst20 + 314.15926535897933f)) / fConst0) + 1.0f);
		fConst23 = (1.0f / fConst22);
		fConst24 = (0.010937499999999999f / fConst0);
		fConst25 = (fConst18 / (fConst1 * fConst22));
		fConst26 = faustpower<2>((125.66370614359172f / fConst0));
		fConst27 = ((fConst26 * ((0.20330000000000001f * fConst26) + 0.31755f)) + 1.0f);
		fConst28 = faustpower<2>(fConst27);
		fConst29 = (2.0f * ((15791.367041742973f * (fConst28 / fConst1)) + -1.0f));
		fConst30 = (15791.367041742973f * (fConst27 / fConst0));
		fConst31 = (((fConst27 * (fConst30 + -232.71056693257725f)) / fConst0) + 1.0f);
		fConst32 = (((fConst27 * (fConst30 + 232.71056693257725f)) / fConst0) + 1.0f);
		fConst33 = (1.0f / fConst32);
		fConst34 = (((fConst27 * (fConst30 + -95.926493239382992f)) / fConst0) + 1.0f);
		fConst35 = (((fConst27 * (fConst30 + 95.926493239382992f)) / fConst0) + 1.0f);
		fConst36 = (1.0f / fConst35);
		fConst37 = (fConst28 / (fConst1 * fConst35));
		fConst38 = (fConst28 / (fConst1 * fConst32));
		fConst39 = powf(0.5f,(10.0f / fConst0));
		fConst40 = (0 - (2.0f / fConst7));
	}
	virtual void instanceResetUserInterface() {
		fslider0 = 0.0f;
		fslider1 = 0.0f;
		fslider2 = 69.0f;
		fslider3 = 0.0f;
		fslider4 = 0.0f;
		fslider5 = 0.0f;
		fslider6 = 1.0f;
		fslider7 = 0.0f;
		fslider8 = 0.0f;
		fslider9 = 69.0f;
		fslider10 = 0.0f;
		fslider11 = 0.0f;
		fslider12 = 0.0f;
		fslider13 = 69.0f;
		fslider14 = 0.0f;
		fslider15 = 0.0f;
		fslider16 = 0.0f;
		fslider17 = 69.0f;
		fslider18 = 0.0f;
	}
	virtual void instanceClear() {
		for (int i=0; i<2; i++) fVec0[i] = 0;
		for (int i=0; i<2; i++) fRec2[i] = 0;
		for (int i=0; i<3; i++) fRec1[i] = 0;
		for (int i=0; i<2; i++) fRec4[i] = 0;
		for (int i=0; i<3; i++) fRec3[i] = 0;
		for (int i=0; i<3; i++) fRec7[i] = 0;
		for (int i=0; i<3; i++) fRec6[i] = 0;
		for (int i=0; i<2; i++) fRec5[i] = 0;
		for (int i=0; i<3; i++) fRec8[i] = 0;
		for (int i=0; i<2; i++) fVec1[i] = 0;
		for (int i=0; i<2; i++) fRec10[i] = 0;
		for (int i=0; i<3; i++) fRec9[i] = 0;
		for (int i=0; i<2; i++) fRec12[i] = 0;
		for (int i=0; i<3; i++) fRec11[i] = 0;
		for (int i=0; i<3; i++) fRec13[i] = 0;
		for (int i=0; i<2; i++) fVec2[i] = 0;
		for (int i=0; i<2; i++) fRec15[i] = 0;
		for (int i=0; i<3; i++) fRec14[i] = 0;
		for (int i=0; i<2; i++) fRec17[i] = 0;
		for (int i=0; i<3; i++) fRec16[i] = 0;
		for (int i=0; i<3; i++) fRec18[i] = 0;
		for (int i=0; i<2; i++) fVec3[i] = 0;
		for (int i=0; i<2; i++) fRec20[i] = 0;
		for (int i=0; i<3; i++) fRec19[i] = 0;
		for (int i=0; i<2; i++) fRec22[i] = 0;
		for (int i=0; i<3; i++) fRec21[i] = 0;
		for (int i=0; i<3; i++) fRec23[i] = 0;
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
		synth_interface.acc_abs = &fslider6;
		synth_interface.rot_y = &fslider4;
		synth_interface.rot_z = &fslider5;
		synth_interface.note[0] = &fslider17;
		synth_interface.pres[0] = &fslider16;
		synth_interface.vpres[0] = &fslider18;
		synth_interface.but_y[0] = &fslider15;
		synth_interface.note[1] = &fslider13;
		synth_interface.pres[1] = &fslider12;
		synth_interface.vpres[1] = &fslider14;
		synth_interface.but_y[1] = &fslider11;
		synth_interface.note[2] = &fslider9;
		synth_interface.pres[2] = &fslider8;
		synth_interface.vpres[2] = &fslider10;
		synth_interface.but_y[2] = &fslider7;
		synth_interface.note[3] = &fslider2;
		synth_interface.pres[3] = &fslider1;
		synth_interface.vpres[3] = &fslider3;
		synth_interface.but_y[3] = &fslider0;
	}
	virtual void compute (int count, FAUSTFLOAT** input, FAUSTFLOAT** output) {
		//zone1
		//zone2
		float 	fSlow0 = float(fslider0);
		float 	fSlow1 = float(fslider1);
		float 	fSlow2 = ((fSlow0 * fSlow1) * max(fSlow0, (float)0));
		float 	fSlow3 = ((3000.0f * fSlow2) + 200.0f);
		float 	fSlow4 = faustpower<2>((fConst9 * fSlow3));
		float 	fSlow5 = ((fSlow4 * ((0.20330000000000001f * fSlow4) + 0.31755f)) + 1.0f);
		float 	fSlow6 = (2.0f * ((fConst10 * (faustpower<2>(fSlow3) * faustpower<2>(fSlow5))) + -1.0f));
		float 	fSlow7 = (fSlow3 * fSlow5);
		float 	fSlow8 = (fConst11 * fSlow7);
		float 	fSlow9 = ((fConst12 * (fSlow7 * (fSlow8 + -1.5707963267948966f))) + 1.0f);
		float 	fSlow10 = ((fConst12 * (fSlow7 * (fSlow8 + 1.5707963267948966f))) + 1.0f);
		float 	fSlow11 = (1.0f / fSlow10);
		float 	fSlow12 = float(fslider2);
		float 	fSlow13 = fastpow2((0.083333333333333329f * fSlow12));
		int 	iSlow14 = int(min((fConst13 / fSlow13), (float)4096));
		float 	fSlow15 = float(iSlow14);
		float 	fSlow16 = (fConst14 * (fSlow13 * fSlow15));
		float 	fSlow17 = (fConst15 * (((fSlow2 * fSlow3) * fSlow5) / fSlow10));
		float 	fSlow18 = max((float(fslider3) + -0.001f), (float)0);
		float 	fSlow19 = ((fSlow18 / (fSlow18 + 0.5f)) + (0.59999999999999998f * faustpower<2>(max((fSlow1 + -0.29999999999999999f), (float)0))));
		float 	fSlow20 = (1.0f - (fConst24 * (fSlow12 / max(min((16.0f * fSlow1), (0.5f * (fSlow1 + 1.0f))), 0.050000000000000003f))));
		float 	fSlow21 = max(((faustpower<2>(float(fslider5)) + faustpower<2>(float(fslider4))) + -0.0050000000000000001f), (float)0);
		float 	fSlow22 = min((2.0f * fSlow1), fSlow21);
		float 	fSlow23 = max((-1 * fSlow0), (float)0);
		float 	fSlow24 = (fConst25 * (1.0f - fSlow23));
		float 	fSlow25 = float(fslider6);
		float 	fSlow26 = max((8.1757989159999997f * fSlow13), (float)200);
		float 	fSlow27 = faustpower<2>((fConst9 * fSlow26));
		float 	fSlow28 = (fConst9 * (fSlow26 * ((fSlow27 * ((0.20330000000000001f * fSlow27) + 0.31755f)) + 1.0f)));
		float 	fSlow29 = (1.0f / ((8.0f * fSlow23) + 1.0f));
		float 	fSlow30 = faustpower<2>((1.0f - (0.5f * fSlow23)));
		float 	fSlow31 = float(fslider7);
		float 	fSlow32 = float(fslider8);
		float 	fSlow33 = ((fSlow31 * fSlow32) * max(fSlow31, (float)0));
		float 	fSlow34 = ((3000.0f * fSlow33) + 200.0f);
		float 	fSlow35 = faustpower<2>((fConst9 * fSlow34));
		float 	fSlow36 = ((fSlow35 * ((0.20330000000000001f * fSlow35) + 0.31755f)) + 1.0f);
		float 	fSlow37 = (2.0f * ((fConst10 * (faustpower<2>(fSlow34) * faustpower<2>(fSlow36))) + -1.0f));
		float 	fSlow38 = (fSlow34 * fSlow36);
		float 	fSlow39 = (fConst11 * fSlow38);
		float 	fSlow40 = ((fConst12 * (fSlow38 * (fSlow39 + -1.5707963267948966f))) + 1.0f);
		float 	fSlow41 = ((fConst12 * (fSlow38 * (fSlow39 + 1.5707963267948966f))) + 1.0f);
		float 	fSlow42 = (1.0f / fSlow41);
		float 	fSlow43 = float(fslider9);
		float 	fSlow44 = fastpow2((0.083333333333333329f * fSlow43));
		int 	iSlow45 = int(min((fConst13 / fSlow44), (float)4096));
		float 	fSlow46 = float(iSlow45);
		float 	fSlow47 = (fConst14 * (fSlow44 * fSlow46));
		float 	fSlow48 = (fConst15 * (((fSlow33 * fSlow34) * fSlow36) / fSlow41));
		float 	fSlow49 = max((float(fslider10) + -0.001f), (float)0);
		float 	fSlow50 = ((fSlow49 / (fSlow49 + 0.5f)) + (0.59999999999999998f * faustpower<2>(max((fSlow32 + -0.29999999999999999f), (float)0))));
		float 	fSlow51 = (1.0f - (fConst24 * (fSlow43 / max(min((16.0f * fSlow32), (0.5f * (fSlow32 + 1.0f))), 0.050000000000000003f))));
		float 	fSlow52 = min((2.0f * fSlow32), fSlow21);
		float 	fSlow53 = max((-1 * fSlow31), (float)0);
		float 	fSlow54 = (fConst25 * (1.0f - fSlow53));
		float 	fSlow55 = max((8.1757989159999997f * fSlow44), (float)200);
		float 	fSlow56 = faustpower<2>((fConst9 * fSlow55));
		float 	fSlow57 = (fConst9 * (fSlow55 * ((fSlow56 * ((0.20330000000000001f * fSlow56) + 0.31755f)) + 1.0f)));
		float 	fSlow58 = (1.0f / ((8.0f * fSlow53) + 1.0f));
		float 	fSlow59 = faustpower<2>((1.0f - (0.5f * fSlow53)));
		float 	fSlow60 = float(fslider11);
		float 	fSlow61 = float(fslider12);
		float 	fSlow62 = ((fSlow60 * fSlow61) * max(fSlow60, (float)0));
		float 	fSlow63 = ((3000.0f * fSlow62) + 200.0f);
		float 	fSlow64 = faustpower<2>((fConst9 * fSlow63));
		float 	fSlow65 = ((fSlow64 * ((0.20330000000000001f * fSlow64) + 0.31755f)) + 1.0f);
		float 	fSlow66 = (2.0f * ((fConst10 * (faustpower<2>(fSlow63) * faustpower<2>(fSlow65))) + -1.0f));
		float 	fSlow67 = (fSlow63 * fSlow65);
		float 	fSlow68 = (fConst11 * fSlow67);
		float 	fSlow69 = ((fConst12 * (fSlow67 * (fSlow68 + -1.5707963267948966f))) + 1.0f);
		float 	fSlow70 = ((fConst12 * (fSlow67 * (fSlow68 + 1.5707963267948966f))) + 1.0f);
		float 	fSlow71 = (1.0f / fSlow70);
		float 	fSlow72 = float(fslider13);
		float 	fSlow73 = fastpow2((0.083333333333333329f * fSlow72));
		int 	iSlow74 = int(min((fConst13 / fSlow73), (float)4096));
		float 	fSlow75 = float(iSlow74);
		float 	fSlow76 = (fConst14 * (fSlow73 * fSlow75));
		float 	fSlow77 = (fConst15 * (((fSlow62 * fSlow63) * fSlow65) / fSlow70));
		float 	fSlow78 = max((float(fslider14) + -0.001f), (float)0);
		float 	fSlow79 = ((fSlow78 / (fSlow78 + 0.5f)) + (0.59999999999999998f * faustpower<2>(max((fSlow61 + -0.29999999999999999f), (float)0))));
		float 	fSlow80 = (1.0f - (fConst24 * (fSlow72 / max(min((16.0f * fSlow61), (0.5f * (fSlow61 + 1.0f))), 0.050000000000000003f))));
		float 	fSlow81 = min((2.0f * fSlow61), fSlow21);
		float 	fSlow82 = max((-1 * fSlow60), (float)0);
		float 	fSlow83 = (fConst25 * (1.0f - fSlow82));
		float 	fSlow84 = max((8.1757989159999997f * fSlow73), (float)200);
		float 	fSlow85 = faustpower<2>((fConst9 * fSlow84));
		float 	fSlow86 = (fConst9 * (fSlow84 * ((fSlow85 * ((0.20330000000000001f * fSlow85) + 0.31755f)) + 1.0f)));
		float 	fSlow87 = (1.0f / ((8.0f * fSlow82) + 1.0f));
		float 	fSlow88 = faustpower<2>((1.0f - (0.5f * fSlow82)));
		float 	fSlow89 = float(fslider15);
		float 	fSlow90 = float(fslider16);
		float 	fSlow91 = ((fSlow89 * fSlow90) * max(fSlow89, (float)0));
		float 	fSlow92 = ((3000.0f * fSlow91) + 200.0f);
		float 	fSlow93 = faustpower<2>((fConst9 * fSlow92));
		float 	fSlow94 = ((fSlow93 * ((0.20330000000000001f * fSlow93) + 0.31755f)) + 1.0f);
		float 	fSlow95 = (2.0f * ((fConst10 * (faustpower<2>(fSlow92) * faustpower<2>(fSlow94))) + -1.0f));
		float 	fSlow96 = (fSlow92 * fSlow94);
		float 	fSlow97 = (fConst11 * fSlow96);
		float 	fSlow98 = ((fConst12 * (fSlow96 * (fSlow97 + -1.5707963267948966f))) + 1.0f);
		float 	fSlow99 = ((fConst12 * (fSlow96 * (fSlow97 + 1.5707963267948966f))) + 1.0f);
		float 	fSlow100 = (1.0f / fSlow99);
		float 	fSlow101 = float(fslider17);
		float 	fSlow102 = fastpow2((0.083333333333333329f * fSlow101));
		int 	iSlow103 = int(min((fConst13 / fSlow102), (float)4096));
		float 	fSlow104 = float(iSlow103);
		float 	fSlow105 = (fConst14 * (fSlow102 * fSlow104));
		float 	fSlow106 = (fConst15 * (((fSlow91 * fSlow92) * fSlow94) / fSlow99));
		float 	fSlow107 = max((float(fslider18) + -0.001f), (float)0);
		float 	fSlow108 = ((fSlow107 / (fSlow107 + 0.5f)) + (0.59999999999999998f * faustpower<2>(max((fSlow90 + -0.29999999999999999f), (float)0))));
		float 	fSlow109 = (1.0f - (fConst24 * (fSlow101 / max(min((16.0f * fSlow90), (0.5f * (fSlow90 + 1.0f))), 0.050000000000000003f))));
		float 	fSlow110 = min((2.0f * fSlow90), fSlow21);
		float 	fSlow111 = max((-1 * fSlow89), (float)0);
		float 	fSlow112 = (fConst25 * (1.0f - fSlow111));
		float 	fSlow113 = max((8.1757989159999997f * fSlow102), (float)200);
		float 	fSlow114 = faustpower<2>((fConst9 * fSlow113));
		float 	fSlow115 = (fConst9 * (fSlow113 * ((fSlow114 * ((0.20330000000000001f * fSlow114) + 0.31755f)) + 1.0f)));
		float 	fSlow116 = (1.0f / ((8.0f * fSlow111) + 1.0f));
		float 	fSlow117 = faustpower<2>((1.0f - (0.5f * fSlow111)));
		//zone2b
		//zone3
		FAUSTFLOAT* output0 = output[0];
		//LoopGraphScalar
		for (int i=0; i<count; i++) {
			fVec0[0] = fSlow12;
			fRec2[0] = (fSlow16 + (fRec2[1] - float((iSlow14 * ((fSlow16 + fRec2[1]) > fSlow15)))));
			int 	iTemp0 = int(fRec2[0]);
			float 	fTemp1 = ftbl0[iTemp0];
			float 	fTemp2 = (fTemp1 + ((fRec2[0] - float(iTemp0)) * (ftbl0[((iTemp0 + 1) % iSlow14)] - fTemp1)));
			fRec1[0] = (fTemp2 - (fSlow11 * ((fSlow9 * fRec1[2]) + (fSlow6 * fRec1[1]))));
			fRec4[0] = (fSlow20 * (max(fRec4[1], fSlow19) * float((fabsf((fSlow12 - fVec0[1])) < 1.0f))));
			fRec3[0] = (max(fRec4[0], fSlow22) - (fConst23 * ((fConst21 * fRec3[2]) + (fConst19 * fRec3[1]))));
			float 	fTemp3 = (((98696.044010893587f * fRec3[0]) + (197392.08802178717f * fRec3[1])) + (98696.044010893587f * fRec3[2]));
			fRec7[0] = (fSlow25 - (fConst36 * ((fConst34 * fRec7[2]) + (fConst29 * fRec7[1]))));
			fRec6[0] = ((fConst37 * (((15791.367041742973f * fRec7[0]) + (31582.734083485946f * fRec7[1])) + (15791.367041742973f * fRec7[2]))) - (fConst33 * ((fConst31 * fRec6[2]) + (fConst29 * fRec6[1]))));
			fRec5[0] = (fConst39 * max(fRec5[1], min((fConst38 * (((15791.367041742973f * fRec6[0]) + (31582.734083485946f * fRec6[1])) + (15791.367041742973f * fRec6[2]))), (float)2)));
			float 	fTemp4 = max((fRec5[0] + -1.0f), (float)0);
			float 	fTemp5 = (fSlow28 + faustpower<2>(faustpower<2>((fTemp4 + (fSlow24 * fTemp3)))));
			float 	fTemp6 = ((fTemp5 * (fSlow29 + fTemp5)) + 1.0f);
			float 	fTemp7 = faustpower<2>(fTemp5);
			fRec8[0] = (fTemp2 - (((((fTemp5 * (fTemp5 - fSlow29)) + 1.0f) * fRec8[2]) + (2.0f * ((fTemp7 + -1.0f) * fRec8[1]))) / fTemp6));
			fVec1[0] = fSlow43;
			fRec10[0] = (fSlow47 + (fRec10[1] - float((iSlow45 * ((fSlow47 + fRec10[1]) > fSlow46)))));
			int 	iTemp8 = int(fRec10[0]);
			float 	fTemp9 = ftbl0[iTemp8];
			float 	fTemp10 = (fTemp9 + ((fRec10[0] - float(iTemp8)) * (ftbl0[((iTemp8 + 1) % iSlow45)] - fTemp9)));
			fRec9[0] = (fTemp10 - (fSlow42 * ((fSlow40 * fRec9[2]) + (fSlow37 * fRec9[1]))));
			fRec12[0] = (fSlow51 * (max(fRec12[1], fSlow50) * float((fabsf((fSlow43 - fVec1[1])) < 1.0f))));
			fRec11[0] = (max(fRec12[0], fSlow52) - (fConst23 * ((fConst21 * fRec11[2]) + (fConst19 * fRec11[1]))));
			float 	fTemp11 = (((98696.044010893587f * fRec11[0]) + (197392.08802178717f * fRec11[1])) + (98696.044010893587f * fRec11[2]));
			float 	fTemp12 = (fSlow57 + faustpower<2>(faustpower<2>((fTemp4 + (fSlow54 * fTemp11)))));
			float 	fTemp13 = ((fTemp12 * (fSlow58 + fTemp12)) + 1.0f);
			float 	fTemp14 = faustpower<2>(fTemp12);
			fRec13[0] = (fTemp10 - (((((fTemp12 * (fTemp12 - fSlow58)) + 1.0f) * fRec13[2]) + (2.0f * ((fTemp14 + -1.0f) * fRec13[1]))) / fTemp13));
			fVec2[0] = fSlow72;
			fRec15[0] = (fSlow76 + (fRec15[1] - float((iSlow74 * ((fSlow76 + fRec15[1]) > fSlow75)))));
			int 	iTemp15 = int(fRec15[0]);
			float 	fTemp16 = ftbl0[iTemp15];
			float 	fTemp17 = (fTemp16 + ((fRec15[0] - float(iTemp15)) * (ftbl0[((iTemp15 + 1) % iSlow74)] - fTemp16)));
			fRec14[0] = (fTemp17 - (fSlow71 * ((fSlow69 * fRec14[2]) + (fSlow66 * fRec14[1]))));
			fRec17[0] = (fSlow80 * (max(fRec17[1], fSlow79) * float((fabsf((fSlow72 - fVec2[1])) < 1.0f))));
			fRec16[0] = (max(fRec17[0], fSlow81) - (fConst23 * ((fConst21 * fRec16[2]) + (fConst19 * fRec16[1]))));
			float 	fTemp18 = (((98696.044010893587f * fRec16[0]) + (197392.08802178717f * fRec16[1])) + (98696.044010893587f * fRec16[2]));
			float 	fTemp19 = (fSlow86 + faustpower<2>(faustpower<2>((fTemp4 + (fSlow83 * fTemp18)))));
			float 	fTemp20 = ((fTemp19 * (fSlow87 + fTemp19)) + 1.0f);
			float 	fTemp21 = faustpower<2>(fTemp19);
			fRec18[0] = (fTemp17 - (((((fTemp19 * (fTemp19 - fSlow87)) + 1.0f) * fRec18[2]) + (2.0f * ((fTemp21 + -1.0f) * fRec18[1]))) / fTemp20));
			fVec3[0] = fSlow101;
			fRec20[0] = (fSlow105 + (fRec20[1] - float((iSlow103 * ((fSlow105 + fRec20[1]) > fSlow104)))));
			int 	iTemp22 = int(fRec20[0]);
			float 	fTemp23 = ftbl0[iTemp22];
			float 	fTemp24 = (fTemp23 + ((fRec20[0] - float(iTemp22)) * (ftbl0[((iTemp22 + 1) % iSlow103)] - fTemp23)));
			fRec19[0] = (fTemp24 - (fSlow100 * ((fSlow98 * fRec19[2]) + (fSlow95 * fRec19[1]))));
			fRec22[0] = (fSlow109 * (max(fRec22[1], fSlow108) * float((fabsf((fSlow101 - fVec3[1])) < 1.0f))));
			fRec21[0] = (max(fRec22[0], fSlow110) - (fConst23 * ((fConst21 * fRec21[2]) + (fConst19 * fRec21[1]))));
			float 	fTemp25 = (((98696.044010893587f * fRec21[0]) + (197392.08802178717f * fRec21[1])) + (98696.044010893587f * fRec21[2]));
			float 	fTemp26 = (fSlow115 + faustpower<2>(faustpower<2>(((fSlow112 * fTemp25) + fTemp4))));
			float 	fTemp27 = ((fTemp26 * (fSlow116 + fTemp26)) + 1.0f);
			float 	fTemp28 = faustpower<2>(fTemp26);
			fRec23[0] = (fTemp24 - (((((fTemp26 * (fTemp26 - fSlow116)) + 1.0f) * fRec23[2]) + (2.0f * ((fTemp28 + -1.0f) * fRec23[1]))) / fTemp27));
			fRec0[0] = ((fConst25 * ((((fTemp25 * ((fSlow117 * ((fTemp28 * (fRec23[2] + (fRec23[0] + (2.0f * fRec23[1])))) / fTemp27)) + (fSlow106 * (fRec19[0] - fRec19[2])))) + (fTemp18 * ((fSlow88 * ((fTemp21 * (fRec18[2] + (fRec18[0] + (2.0f * fRec18[1])))) / fTemp20)) + (fSlow77 * (fRec14[0] - fRec14[2]))))) + (fTemp11 * ((fSlow59 * ((fTemp14 * (fRec13[2] + (fRec13[0] + (2.0f * fRec13[1])))) / fTemp13)) + (fSlow48 * (fRec9[0] - fRec9[2]))))) + (fTemp3 * ((fSlow30 * ((fTemp7 * (fRec8[2] + (fRec8[0] + (2.0f * fRec8[1])))) / fTemp6)) + (fSlow17 * (fRec1[0] - fRec1[2])))))) - (fConst8 * ((fConst6 * fRec0[2]) + (fConst4 * fRec0[1]))));
			output0[i] = (FAUSTFLOAT)((fConst40 * fRec0[1]) + (fConst8 * (fRec0[0] + fRec0[2])));
			// post processing
			fRec0[2] = fRec0[1]; fRec0[1] = fRec0[0];
			fRec23[2] = fRec23[1]; fRec23[1] = fRec23[0];
			fRec21[2] = fRec21[1]; fRec21[1] = fRec21[0];
			fRec22[1] = fRec22[0];
			fRec19[2] = fRec19[1]; fRec19[1] = fRec19[0];
			fRec20[1] = fRec20[0];
			fVec3[1] = fVec3[0];
			fRec18[2] = fRec18[1]; fRec18[1] = fRec18[0];
			fRec16[2] = fRec16[1]; fRec16[1] = fRec16[0];
			fRec17[1] = fRec17[0];
			fRec14[2] = fRec14[1]; fRec14[1] = fRec14[0];
			fRec15[1] = fRec15[0];
			fVec2[1] = fVec2[0];
			fRec13[2] = fRec13[1]; fRec13[1] = fRec13[0];
			fRec11[2] = fRec11[1]; fRec11[1] = fRec11[0];
			fRec12[1] = fRec12[0];
			fRec9[2] = fRec9[1]; fRec9[1] = fRec9[0];
			fRec10[1] = fRec10[0];
			fVec1[1] = fVec1[0];
			fRec8[2] = fRec8[1]; fRec8[1] = fRec8[0];
			fRec5[1] = fRec5[0];
			fRec6[2] = fRec6[1]; fRec6[1] = fRec6[0];
			fRec7[2] = fRec7[1]; fRec7[1] = fRec7[0];
			fRec3[2] = fRec3[1]; fRec3[1] = fRec3[0];
			fRec4[1] = fRec4[0];
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

		synth_tick();

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
