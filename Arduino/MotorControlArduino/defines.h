//
int test1 = 0;
int test2 = 0;
int test3 = 0;

/*//////////////////////////////////////*/
/*Tracing*/
/*//////////////////////////////////////*/
int tracearray[] = {1, 2, 3, 4, 5, 6, 7, 8, 9 , 10 , 1, 2, 3, 4, 5, 6, 7, 8, 9 , 10};
binaryFloat bf;
binaryFloat ser_in;
unsigned int zenddata = 2;
unsigned int Nsend = 0;
unsigned int sigid = 0;
int incomingByte; //Needed?


/*//////////////////////////////////////*/
/*Pinning definitions*/
/*//////////////////////////////////////*/

const int outLed = 0;

const int outPhA1 = 6;
const int outPhB1 = 7;
const int outPhC1 = 24;

const int inA1 = 18;
const int inB1 = 19;
const int inZ1 = 9;
const int inA2 = 14;
const int inB2 = 15;
const int inZ2 = 10;

const int inIA1 = A7;
const int inIB1 = A8;
const int inIC1 = A9;

/*//////////////////////////////////////*/
/*ReadInputs*/
/*//////////////////////////////////////*/

/* ADC resolution current sensors */
float adc2V = 0.00080566;

int enc1np = 4*2048;
float enc2rad = 2*M_PI / enc1np; 
float enc2m = 0.16/3000;

/*Encoder settings*/
int EncPos1Raw = 0;
int EncPos2Raw = 0;
boolean A_set1 = false;
boolean B_set1 = false;
boolean Z_set1 = false;
boolean A_set2 = false;
boolean B_set2 = false;
boolean Z_set2 = false;

float penc1 = 0;
float penc2 = 0;

float y;

/*Current measurement variables*/
float sens;
float Im1[3] = {0,0,0};
float Im1cal[3] = {0,0,0};
float sensCalVal;
int sensorValue = 0;


/*//////////////////////////////////////*/
/*Setpoints*/
/*//////////////////////////////////////*/
int state = 0;

/*Identification*/
// prbs
unsigned int Ndownsample = 1;
uint16_t start_state = 0xACE1u;  /* Any nonzero start state will work. */
uint16_t lfsr = start_state;
uint16_t noisebit;                    /* Must be 16bit to allow bit<<15 later in the code */
unsigned period = 0;
unsigned int downsample = 1;

// SS
float ss_phase = 0;
float ss_fstart = 1730;
float ss_fstep = 0.2;
float ss_fend = 1830;
unsigned int ss_n_aver = 200;
float ss_gain = 0;
float ss_offset = 0;
float ss_f = ss_fstart;
float ss_tstart;
float ss_out = 0;


float rpos1 = 0;
float rpos2 = 0;

int PCstate = 0;
int EnStateFB = 0;

float r = 0;


/*Setpoint variables*/
//fast 180 deg:
MotionProfile2 *SPprofile = new MotionProfile2( 0.01 , 0.000500000000000000 , 0.0193000000000000 , 0 , 3.14159265358979 , 157.079632679490 , 7853.98163397448 , 15632147.3532855 , T );
float acc;
float vel;
float dist;
float cursp;
float Iout;

int spGO = 0;


/*Feedforward variables*/
float velFF = 0;
float R = 0;

/*Offset velocity variables*/
float offsetVelTot = 0;
float offsetVel = 0;

unsigned int ContSelect = 1;

/*//////////////////////////////////////*/
/*PositionControl*/
/*//////////////////////////////////////*/
float Jload = 0;

/*Variables*/
float mechcontout = 0;
float PC1out = 0;
float PC2out = 0;
float Intout1 = 0;
float Intout2 = 0;

float muziek_gain = 0;
float muziek_gain_V = 0;

float distval = 0;
float distoff = 0;


float emechy = 0;
float emechphi = 0;
 
unsigned int calctime = 0;
unsigned int previousTime = 0;
float lastpmechy = 0;
float lastpmechphi = 0;
float vmechy = 0;
float vmechphi = 0;


/*Controller variables*/
float fBW = 50.0;
float alpha1 = 3.0;
float alpha2 = 4.0;
float fInt = fBW / 6.0;
float fLP = fBW * 6.0;
int IntegOn = 1;

// Controller 1
Integrator *integrator_PC1 = new Integrator( fInt , 1 / T);
LeadLag *leadlag_PC1       = new LeadLag( fBW , alpha1 , alpha2 , 1 / T);
Biquad *lowpass_PC1        = new Biquad( bq_type_lowpass , fLP , 0.7, 1 / T);
Biquad *notch_PC1          = new Biquad( bq_type_notch , 1810.0, -20.0, 0.1 , 1 / T );
float Kp_PC1 = 0;

// Controller 2
Integrator *integrator_PC2 = new Integrator( fInt , 1 / T);
LeadLag *leadlag_PC2       = new LeadLag( fBW , alpha1 , alpha2 , 1 / T);
Biquad *lowpass_PC2        = new Biquad( bq_type_lowpass , fLP , 0.7, 1 / T);
float Kp_PC2 = 0;

// Controller state feedback pendulum
float k1 = -2.5; 
float k2 = -2.66642077414672;
float k3 = -12.1591323401845;
float k4 = -2.45600514544917;


/*//////////////////////////////////////*/
/*CurrentControl*/
/*//////////////////////////////////////*/

/*Motor variables*/
float Kt = 0.39; // Nm/A
int npole1 = 4;

float e;

float Icontout;


float Vout;
float fIntCur = 700;
Integrator *integratorCC = new Integrator( fIntCur , 1 / T);
float Icontgain = 30;

Biquad *lowpasscursens = new Biquad( bq_type_lowpass , 4000 , 0.707, 1 / T);

/*Amplifier variables*/
float V_bus = 12;

/*Park clarke transform*/
float Im1ab[2] = {0,0};
float Im1dq[2] = {0,0};
float Vph1ab[2] = {0,0};
float Vph1dq[2] = {0,0};
float Vph1[3] = {0,0,0};

float ThetaE1 = 0;
float ThetaE1sin = 0;
float ThetaE1cos = 0;

/*//////////////////////////////////////*/
/*Safety*/
/*//////////////////////////////////////*/
float I_max = 5;
float V_max = V_bus/2;

int errcode = 0;

/*PWM generation*/
boolean OutputOn = true;
int PWMph1[3] = {0,0,0};
float PWMph1offs[3] = {0,0,0};

/*//////////////////////////////////////*/
/*SendOutputs*/
/*//////////////////////////////////////*/


/*//////////////////////////////////////*/
/*Timing*/
/*//////////////////////////////////////*/
int timeremain;
int TET;
unsigned int timePrev = 0;
unsigned int curtime = 0;
unsigned int curloop = 0;
/*//////////////////////////////////////*/
