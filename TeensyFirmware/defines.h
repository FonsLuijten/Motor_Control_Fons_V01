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

const int outLed = 2;

const int outPhA1 = 4;
const int outPhB1 = 5;
const int outPhC1 = 6;
const int outEn1 = 7;

const int outEnBrake1 = 24;

const int inA1 = 0;
const int inB1 = 1;
const int inZ1 = 3;
const int inA2 = 30;
const int inB2 = 31;
const int inZ2 = 33;

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

//There are 4 hardware quadrature encoder channels available the Teensy 4.x. 
//The Teensy 4.1 Encoders are supported on pins: 0, 1, 2, 3, 4, 5, 7, 30, 31, 33, 36 and 37. 
//WARNING! Pins 0, 5 and 37 share the same internal crossbar connections and are as such exclusive...pick one or the other.
QuadEncoder Encoder1(1, inA1, inB1 , 0 , inZ1);   //Encoder 1 on pins 0 and 1, index on pin 3
QuadEncoder Encoder2(2, inA2, inB2 , 0 , inZ2);//Encoder 2 on pins 30 and 31, index on pin 33
/*Encoder settings*/
int EncPos1Raw = 0;
int EncPos2Raw = 0;
int IndexFound1 = 0;
int IndexFound2 = 0;


float Xm1 = 0;
float Xm2 = 0;

/*Current measurement variables*/
float sens;
float Im1[3] = {0,0,0};
float Im1cal[3] = {0,0,0};
float sensCalVal;
int sensorValue = 0;

//Biquad *lowpasscursens = new Biquad( bq_type_lowpass , 4000 , 0.707, 1 / T);


/*//////////////////////////////////////*/
/*Setpoints*/
/*//////////////////////////////////////*/
int state = 0;
int stateprev = 0;
int enBrake = 0;

/*Identification*/
// prbs
unsigned int Ndownsample = 1;
uint16_t start_state = 0xACE1u;  /* Any nonzero start state will work. */
uint16_t lfsr = start_state;
uint16_t noisebit;                    /* Must be 16bit to allow bit<<15 later in the code */
unsigned period = 0;
unsigned int downsample = 1;
float prbs_gain = 0;
float prbs_out = 0;

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

// Music
float music_gain = 0;
float music_out = 0;
float muziek_gain_V = 0;

// disturbances
float distoff = 0;
float dist;
float distcurQ;   // Volts in Q axis FF
float distcurD;   // Volts in D axis FF
float distcurSP;  // Amps in Q axis setpoint
float distpos;
float distposSP;

int endistcurQ = 0;
int endistcurD = 0;
int endistcurSP = 0;
int endistpos = 0;
int endistposSP = 0;

/*Setpoint variables*/
//fast 180 deg:
float sp1_tstart = 0.01;
float sp1_t1 = 0.0005;
float sp1_t2 = 0.0193;
float sp1_t3 = 0;
float sp1_p = 3.14159265358979;
float sp1_vmax = 157.079632679490;
float sp1_amax = 7853.98163397448;
float sp1_jmax = 15632147.3532855;
MotionProfile2 *SP1profile = new MotionProfile2( sp1_tstart , sp1_t1 , sp1_t2 , sp1_t3 , sp1_p , sp1_vmax , sp1_amax , sp1_jmax , T );
float acc;
float vel;

float Xsp1 = 0;
float Xsp2 = 0;
float Xsp1Offs = 0;
float Xsp2Offs = 0;
int spGO = 0;
int spSET = 0;

/*Offset velocity variables*/
float offsetVelTot = 0;
float offsetVel = 0;

/*//////////////////////////////////////*/
/*PositionControl*/
/*//////////////////////////////////////*/

float Jload = 0;

/*Variables*/
float Xe1 = 0;
float Xe2 = 0;
float PC1out = 0;
float PC2out = 0;
float Intout1 = 0;
float Intout2 = 0;

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

/*Feedforward variables*/
float velFF = 0;
float R = 0;

/*//////////////////////////////////////*/
/*CurrentControl*/
/*//////////////////////////////////////*/

/*Motor variables*/
float Kt1 = 0.39; // Nm/A
int npole1 = 4;

float Isp1dq[2] = {0,0};
float Ie1dq[2] = {0,0};

float Vccout1dq[2] = {0,0};
float VccFF1dq[2] = {0,0};

float CC1kp = 10;
float CC1fInt = 1000;
Integrator *integratorCC1d = new Integrator( CC1fInt , 1 / T);
Integrator *integratorCC1q = new Integrator( CC1fInt , 1 / T);

/*Amplifier variables*/
float V_bus = 12;

/*Park clarke transform*/
float Im1ab[2] = {0,0};
float Im1dq[2] = {0,0};
float Vph1ab[2] = {0,0};
float Vph1dq[2] = {0,0};
float Vph1dqOffs[2] = {0,0};
float Vph1[3] = {0,0,0};
float ThetaEcalAmp = 0;
float ThetaEcalFreq = 0;
float ThetaE1 = 0;
float ThetaE1cal = 3.21561222;  /*calibrated electrical angle when homed*/
float ThetaE1sin = 0;
float ThetaE1cos = 0;
int enSVM = 1;
float SVM1shift = 0;

/*//////////////////////////////////////*/
/*Safety*/
/*//////////////////////////////////////*/
float I_max = 5;
float V_max = V_bus/2;
float e_max = M_PI/10;

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
