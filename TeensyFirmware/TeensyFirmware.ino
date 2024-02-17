  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*Communication with PC*/
typedef union {
  float fp;
  byte bin[4];
  unsigned int uint;
  int sint;
  bool bl;
} binaryFloat;

/*Sampling time*/
const int Ts = 50;          //Ts in microseconds
const float T = Ts / 1e6;   //T = Ts in seconds
float t = 0;

/*Includes*/
#include "QuadEncoder.h"
#include <Biquad.h>
#include <Math.h>
#include <ControlTools.h>
#include "muziek.c"
#include <MotionProfile2.h>
#include "defines.h"

/*flag to serial communicate signames to host*/
int getsignalnames = 0;

/*ADC channels*/
#include <ADC.h>
#include <ADC_util.h>
ADC *adc = new ADC(); // adc object

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup basic functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  //// get cycles from CPU
  //ARM_DEMCR |= ARM_DEMCR_TRCENA;
  //ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
  //unsigned int cycles = ARM_DWT_CYCCNT;

  /*Blink outLed while waiting for serial connection with host*/
  pinMode(outLed, OUTPUT);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
    digitalWriteFast(outLed, HIGH);
    delayMicroseconds(80);
    digitalWriteFast(outLed, LOW);
    delayMicroseconds(10);
  }
  digitalWriteFast(outLed, LOW);

  /*Set IO ports*/
  //pinMode(brake, OUTPUT);
  pinMode(outPhA1, OUTPUT);
  pinMode(outPhB1, OUTPUT);
  pinMode(outPhC1, OUTPUT);
  pinMode(outEn1, OUTPUT);
  digitalWriteFast(outEn1, HIGH);

  pinMode(outEnBrake1, OUTPUT);
  digitalWriteFast(outEnBrake1, LOW);

  /*Set PWM frequency*/
  analogWriteFrequency(outPhA1, 20e3); // Teensy 3.0 pin 3 also changes to 375 kHz
  analogWriteFrequency(outPhB1, 20e3); // Teensy 3.0 pin 3 also changes to 375 kHz
  analogWriteFrequency(outPhC1, 20e3); // Teensy 3.0 pin 3 also changes to 375 kHz 

  /*Set analog out resolution*/
  analogWriteResolution(12);  // analogWrite value 0 to 4095, or 4096 for high
  analogReadResolution(12);

  analogWrite( outPhA1, 0);
  analogWrite( outPhB1, 0);
  analogWrite( outPhC1, 0);
  delay(100);
  
  /*Setup encoders*/
  Encoders_init();

  /*Average measurement current sensor calibration to eliminate offset*/
  int i = 0;
  timePrev = micros();
  for (; i < 2000; i++) {
    while ( curtime < timePrev + Ts) {
      curtime = micros();
    }
    timePrev += Ts;
    int sensI1[] = {analogRead(inIA1),analogRead(inIB1),analogRead(inIC1)};
    for(int ii = 0; ii < 3; ii++) Im1[ii] += float(sensI1[ii]) * adc2V;
  }
  for(int ii = 0; ii < 3; ii++) Im1cal[ii] = Im1[ii] / i;

  /*Wait till host (Spyder) has asked for all signal names*/
  Nsend = 0;
  while (getsignalnames == 0) {
    processSerialIn();
  }
  delay(500);
  while (Serial.available() > 4) {
    processSerialIn();
  }

  /*Sync PWM setup*/
  syncflexpwmsetup();
  
  /*Always start loop at multiple of Ts*/
  curtime = micros();
  while (curtime % Ts > 0) {
    curtime = micros();
  }
  timePrev = micros();
  
  /*Sync PWM*/
  syncflexpwm();

  /*Set state to 0 initially*/
  state = 0;
}

/*Helper functions in setup*/
void syncflexpwmsetup() {
  cli()
  //Set CTRL2 to the normal settings:
  FLEXPWM2_SM0CTRL2 = FLEXPWM_SMCTRL2_INDEP; //Enable Independent pair, but disable Debug Enable and WAIT Enable. When set to one, the PWM will continue to run while the chip is in debug/WAIT mode.
  FLEXPWM2_SM1CTRL2 = FLEXPWM_SMCTRL2_INDEP; //Enable Independent pair, but disable Debug Enable and WAIT Enable. When set to one, the PWM will continue to run while the chip is in debug/WAIT mode.
  FLEXPWM2_SM2CTRL2 = FLEXPWM_SMCTRL2_INDEP; //Enable Independent pair, but disable Debug Enable and WAIT Enable. When set to one, the PWM will continue to run while the chip is in debug/WAIT mode.
  FLEXPWM4_SM0CTRL2 = FLEXPWM_SMCTRL2_INDEP; //Enable Independent pair, but disable Debug Enable and WAIT Enable. When set to one, the PWM will continue to run while the chip is in debug/WAIT mode.
  FLEXPWM4_SM1CTRL2 = FLEXPWM_SMCTRL2_INDEP; //Enable Independent pair, but disable Debug Enable and WAIT Enable. When set to one, the PWM will continue to run while the chip is in debug/WAIT mode.
  FLEXPWM4_SM2CTRL2 = FLEXPWM_SMCTRL2_INDEP; //Enable Independent pair, but disable Debug Enable and WAIT Enable. When set to one, the PWM will continue to run while the chip is in debug/WAIT mode.

  FLEXPWM2_SM0CTRL2 |= FLEXPWM_SMCTRL2_FRCEN;  // pin 4
  FLEXPWM2_SM1CTRL2 |= FLEXPWM_SMCTRL2_INIT_SEL(2); // pin 5 Master sync from submodule 0 causes initialization.
  FLEXPWM2_SM2CTRL2 |= FLEXPWM_SMCTRL2_INIT_SEL(2); // pin 6 Master sync from submodule 0 causes initialization.

  FLEXPWM4_SM0CTRL2 |= FLEXPWM_SMCTRL2_FRCEN; // pin 22
  FLEXPWM4_SM1CTRL2 |= FLEXPWM_SMCTRL2_INIT_SEL(2); // pin 23 Master sync from submodule 0 causes initialization.
  FLEXPWM4_SM2CTRL2 |= FLEXPWM_SMCTRL2_INIT_SEL(2); // pin 2 Master sync from submodule 0 causes initialization.
  sei();
}

void syncflexpwm() {
  cli()
  //Sync flexpwm2 and flexpwm4
  FLEXPWM2_SM0CTRL2 |= FLEXPWM_SMCTRL2_FORCE; // Force FlexPWM2
  FLEXPWM4_SM0CTRL2 |= FLEXPWM_SMCTRL2_FORCE; // Force FlexPWM4. This is about 50 ns later than force of flexpwm2. No idea on how to improve this
  sei();
}

void Encoders_init() {
  Encoder1.setInitConfig();
  //Optional filters
  Encoder1.EncConfig.filterCount = 3; //Bit of filtering to avoid spurious triggering.
  Encoder1.EncConfig.filterSamplePeriod = 3; //Bit of filtering to avoid spurious triggering.
  Encoder1.EncConfig.INDEXTriggerMode  = 1;
  Encoder1.EncConfig.IndexTrigger  = 1;
  Encoder1.EncConfig.positionInitialValue = 0;
  Encoder1.init();

  Encoder2.setInitConfig();
  Encoder2.EncConfig.filterCount = 3; //Bit of filtering to avoid spurious triggering.
  Encoder2.EncConfig.filterSamplePeriod = 3; //Bit of filtering to avoid spurious triggering.
  Encoder2.EncConfig.INDEXTriggerMode = 1;
  Encoder2.EncConfig.IndexTrigger  = 1;
  Encoder2.EncConfig.positionInitialValue = 0;
  Encoder2.init();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  curloop++;

  /*Control related tasks*/
  ReadInputs();
  Setpoints();
  PositionControl();
  CurrentControl();
  Safety();
  SendOutputs();

  /*State used in curloop*/
  stateprev = state;
  
  /*Task Execution Time*/
  TET = micros() - timePrev;
  
  /*Serial communication*/
  processSerialIn();
  processSerialOut();

   /*Wait till Ts*/  
  digitalWriteFast(outLed, LOW);
  curtime = micros();
  timeremain = timePrev + Ts - curtime;
  while ( curtime < timePrev + Ts) {
    curtime = micros();
  }
  timePrev += Ts;
  digitalWriteFast(outLed, HIGH);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Control loop functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ReadInputs() {
  /*Current control sensors read*/
  int sensI1[3] = {analogRead(inIA1),analogRead(inIB1),analogRead(inIC1)};
  for(int ii = 0; ii < 3; ii++) Im1[ii] = -((float(sensI1[ii]) * adc2V) - Im1cal[ii]) / 0.09;

  /*Positions*/
  EncPos1Raw = Encoder1.read();
  EncPos2Raw = Encoder2.read();
  IndexFound1 = Encoder1.indexfound();
  IndexFound2 = Encoder2.indexfound();
  
  Xm1 = EncPos1Raw * enc2rad;
  Xm2 = EncPos2Raw * enc2rad;
}

void Setpoints() {
  /*prbs identification*/
  downsample++;
  if ( downsample > Ndownsample) {
    downsample = 1;
    /* taps: 11 9; feedback polynomial: x^11 + x^9 + 1 */
    noisebit  = ((lfsr >> 5) ^ (lfsr >> 7) ) & 1;
    lfsr =  (lfsr >> 1) | (noisebit << 15);
    ++period;
    //randNumber = 4 * (noisebit - 0.5);
  }
  prbs_out = prbs_gain * 1 * (noisebit - 0.5);

  /*Single sine identification*/
  if (ss_gain > 0) {
    if ( (timePrev + Ts) / 1e6 >= (ss_tstart + ss_n_aver / ss_f )) {
      ss_f += ss_fstep;
      ss_tstart = (timePrev + Ts) / 1e6;
      if (ss_f > ss_fend)
      {
        ss_f = 0;
        ss_phase = 0;
        ss_tstart = 1e8;
        ss_gain = 0;
      }
    }
    ss_phase += ss_f * 2 * M_PI * T;
    if ( ss_phase > 2 * M_PI) {
      ss_phase -= 2 * M_PI; //Required, because high value floats are inaccurate
    }
  }
  else {
    ss_f = ss_fstart;
    ss_phase = 0;
    ss_tstart = (timePrev + Ts) / 1e6;
    ss_gain = 0;
  }
  ss_out = ss_offset + ss_gain * sin( ss_phase ); //sin() measured to be faster then sinf()

  /*Music setpoint*/
  music_out = music_gain * music[ (curloop / (50 / Ts)) % (sizeof(music) / 4) ];

  /*Disturbance construction for all identification methods*/
  dist = distoff + ss_out + prbs_out + music_out;
  distpos = 0;
  distposSP = 0;
  distcurQ = 0;
  distcurD = 0;
  distcurSP = 0;
  if (endistcurQ == 1){
    distcurQ = dist;
  }
  if (endistcurD == 1){
    distcurD = dist;
  }
  if (endistcurSP == 1 && state == 5){
    distcurSP = dist;
  }
  if (endistpos == 1){
    distpos = dist;
  }
  if (endistposSP == 1){
    distposSP = dist;
  }
  
  /*Position control setpoints*/
  if (stateprev != state && state == 6) {
    Xsp1Offs = Xm1;
    Xsp2Offs = Xm2;
    spSET = 1;
  }
  
  if (spSET == 1) {
    SP1profile->t1 = sp1_t1;
    SP1profile->t2 = sp1_t2;
    SP1profile->t3 = sp1_t3;
    SP1profile->p = sp1_p;
    SP1profile->v_max = sp1_vmax;
    SP1profile->a_max = sp1_amax;
    SP1profile->j_max = sp1_jmax;
    SP1profile->init();
    spSET = 0;
  }

  Xsp1 = Xsp1Offs;
  Xsp1 += SP1profile->stateCalculation( spGO );
  float acc = SP1profile->aref;
  float vel = SP1profile->vref;
  offsetVelTot += offsetVel;
  //Xsp1 += offsetVelTot ;
  Xsp2 = Xsp2Offs;
}

void PositionControl() {
  
  Xe1 = Xsp1 - Xm1;
  Xe2 = Xsp2 - Xm2;

 /*Position control*/
 if (state == 6){
   /*Feedback*/
   PC1out = Kp_PC1 * Xe1;
   PC1out = leadlag_PC1->process( PC1out );
   PC1out = lowpass_PC1->process( PC1out );
   //PC1out = notch->process( PC1out );
   if (IntegOn == 1){
     Intout1 = integrator_PC1->processclip( PC1out , -I_max * Kt1 - PC1out , I_max * Kt1 - PC1out );
     PC1out += Intout1;
   }
   else{
     integrator_PC1->setState(0);
   }

   /*Feedforward*/
   PC1out += acc * Jload;
   PC1out += vel * velFF;

   /*Disturbance*/
   PC1out += distpos;
   
 }
 else{ // Position control off
   PC1out = 0;
   PC2out = 0;
   integrator_PC1->setState(0);
   integrator_PC2->setState(0);
 }
}

void CurrentControl() {
  /*###FOC###*/
  /*Electrical angle, if state == 3: calibrate*/
  if (state == 3){ 
    ThetaE1 = (2*M_PI)*sin(timePrev/1e6*(2*M_PI*ThetaEcalFreq));
    ThetaE1cal = EncPos1Raw*npole1*enc2rad;
  }
  else {
    ThetaE1 = mod(EncPos1Raw*npole1,enc1np)*enc2rad+ThetaE1cal;
  }
  ThetaE1sin = sin(ThetaE1);
  ThetaE1cos = cos(ThetaE1);
    
  /*Clarke*/
  Im1ab[0] = Im1[0];
  Im1ab[1] = 1/sqrt(3)*Im1[0] + 2/sqrt(3)*Im1[1];
  
  /*Park*/
  Im1dq[0] = ThetaE1sin*Im1ab[0] - ThetaE1cos*Im1ab[1];
  Im1dq[1] = ThetaE1cos*Im1ab[0] + ThetaE1sin*Im1ab[1];
  
  /*Current Control, if state == 3: calibrate electrical angle, if state == 4: Open loop voltage setting*/ 
  Isp1dq[0] = 0; //PC1out / Kt1
  Isp1dq[1] = distcurSP + PC1out / Kt1;

  Ie1dq[0] = Isp1dq[0] - Im1dq[0];
  Ie1dq[1] = Isp1dq[1] - Im1dq[1];
  
  if (state == 3){
    Vph1dq[0] = 0;
    Vph1dq[1] = ThetaEcalAmp;
  }
  else if (state == 4){
    Vph1dq[0] = Vph1dqOffs[0] + distcurD;
    Vph1dq[1] = Vph1dqOffs[1] + distcurQ;
  }
  else if (state == 5 || state == 6){
    /*Current control*/
    Vccout1dq[0] = CC1kp * Ie1dq[0];
    Vccout1dq[0] += integratorCC1d->processclip( Vccout1dq[0] , -V_max - Vccout1dq[0] , V_max - Vccout1dq[0] );
    Vccout1dq[1] = CC1kp * Ie1dq[1];
    Vccout1dq[1] += integratorCC1q->processclip( Vccout1dq[1] , -V_max - Vccout1dq[1] , V_max - Vccout1dq[1] );

    VccFF1dq[0] = 0;
    VccFF1dq[1] = 0;

    Vph1dq[0] = Vccout1dq[0] + VccFF1dq[0] + Vph1dqOffs[0] + distcurD;
    Vph1dq[1] = Vccout1dq[1] + VccFF1dq[1] + Vph1dqOffs[1] + distcurQ;
  }
  else {
    Vph1dq[0] = 0;
    Vph1dq[1] = 0;
    integratorCC1d->setState(0);
    integratorCC1q->setState(0);
  }
  
  /*Inverse Park*/
  Vph1ab[0] = ThetaE1sin*Vph1dq[0] + ThetaE1cos*Vph1dq[1];
  Vph1ab[1] = -ThetaE1cos*Vph1dq[0] + ThetaE1sin*Vph1dq[1];
  
  /*Inverse Clarke*/
  Vph1[0] = Vph1ab[0];
  Vph1[1] = -0.5*Vph1ab[0] + (sqrt(3)/2)*Vph1ab[1];
  Vph1[2] = -0.5*Vph1ab[0] - (sqrt(3)/2)*Vph1ab[1];
  
  /*SVM*/
  if (enSVM == 1){
    SVM1shift = 0.5*(min(min(Vph1[0],Vph1[1]),Vph1[2])+max(max(Vph1[0],Vph1[1]),Vph1[2]));
    Vph1[0] = Vph1[0]-SVM1shift;
    Vph1[1] = Vph1[1]-SVM1shift;
    Vph1[2] = Vph1[2]-SVM1shift;
  }
}

void Safety(){
  /* First check for errors */
  if (errcode == 999){
    state = 0;
    errcode = 0;
    OutputOn = true;
    SP1profile->REFqmem = 0;
    PWMph1offs[0] = 0;
    PWMph1offs[1] = 0;
    PWMph1offs[2] = 0;
    music_gain = 0;
  }
  else if (OutputOn == true) {
    if (abs(Vph1[0]) > V_max) {
      OutputOn = false; errcode += 1;
    }
    if (abs(Vph1[1]) > V_max) {
      OutputOn = false; errcode += 2;
    }
    if (abs(Vph1[2]) > V_max) {
      OutputOn = false; errcode += 4;
    }
    if (abs(Im1[0]) > I_max) {
      OutputOn = false; errcode += 8;
    }
    if (abs(Im1[1]) > I_max) {
      OutputOn = false; errcode += 16;
    }
    if (abs(Im1[2]) > I_max) {
      OutputOn = false; errcode += 32;
    }
    if (abs(Xe1) > e_max && state == 6) {
      OutputOn = false; errcode += 64;
    }
    if (abs(Xe2) > e_max && state == 6) {
      OutputOn = false; errcode += 128;
    }
  }

  /* When errors found turn off output */
  if (OutputOn == false) {
    PWMph1[0] = 0;
    PWMph1[1] = 0;
    PWMph1[2] = 0;
    integratorCC1d->setState(0);
    integratorCC1q->setState(0);
    digitalWriteFast(outEn1, LOW);
    state = 1;
  }
  else{
    /*PWM generation*/
    if (state == 0){
      digitalWriteFast(outEn1, LOW);
      PWMph1[0] = 0;
      PWMph1[1] = 0;
      PWMph1[2] = 0;
      PWMph1offs[0] = 0;
      PWMph1offs[1] = 0;
      PWMph1offs[2] = 0;
      integratorCC1d->setState(0);
      integratorCC1q->setState(0);
    }
    else {
      digitalWriteFast(outEn1, HIGH);
    }
    
    if (state == 1 || state == 2){
      PWMph1[0] = round(PWMph1offs[0]*pow(2, 12));
      PWMph1[1] = round(PWMph1offs[1]*pow(2, 12));
      PWMph1[2] = round(PWMph1offs[2]*pow(2, 12));
    }
    else if (state >= 3){
      PWMph1[0] = round((Vph1[0] + V_bus/2) * pow(2, 12) / V_bus);
      PWMph1[1] = round((Vph1[1] + V_bus/2) * pow(2, 12) / V_bus);
      PWMph1[2] = round((Vph1[2] + V_bus/2) * pow(2, 12) / V_bus);
    }
  }
}

void SendOutputs(){
  /*Send volts to motor*/
  if (PWMph1[0] < 0) {
    PWMph1[0] = 0;
  }
  if (PWMph1[1] < 0) {
    PWMph1[1] = 0;
  }
  if (PWMph1[2] < 0) {
    PWMph1[2] = 0;
  }
  if (PWMph1[0] > 4095) {
    PWMph1[0] = 4095;
  }
  if (PWMph1[1] > 4095) {
    PWMph1[1] = 4095;
  }
  if (PWMph1[2] > 4095) {
    PWMph1[2] = 4095;
  }
  analogWrite(outPhA1, PWMph1[0]);
  analogWrite(outPhB1, PWMph1[1]);
  analogWrite(outPhC1, PWMph1[2]);

  if(enBrake == 1) {
    digitalWriteFast(outEnBrake1, HIGH);
  }
  else {
    digitalWriteFast(outEnBrake1, LOW);
  }
  
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Control helper Functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int mod( int x, int y ){
return x<0 ? ((x+1)%y)+y-1 : x%y;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Communication functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Serial communication
void processSerialIn() {
  if (Serial.available() > 4) {
    
    // Read in user command
    char settingByte = Serial.read();
    for ( int i = 0; i < 4; i++) {
      ser_in.bin[i] = Serial.read();
    }
    
    // Trace signals commands
    if (settingByte == 't') {
      tracearray[Serial.read()] = ser_in.uint;
    }
    if (settingByte == 'T') {
      printSignals( ser_in.uint );
    }
    if (settingByte == 'S') {
      for ( int i = 0; i < 4; i++) {
        bf.bin[i] = Serial.read();
      }
      setpar( ser_in.uint , bf );
    }
  }
}

void processSerialOut() {
  if (Nsend > 0) {
    bf.uint  = curloop;        Serial.write( bf.bin , 4);
    bf.uint  = curtime;        Serial.write( bf.bin , 4);
    trace( );

    Nsend--;
  }
}


void trace( ) {
  for( int i = 0; i<10; i++){
    int isignal = tracearray[i];
    switch( isignal ){
      case   0: bf.fp   = adc2V; break;
      case   1: bf.fp   = enc2rad; break;
      case   2: bf.fp   = enc2m; break;
      case   3: bf.fp   = Xm1; break;
      case   4: bf.fp   = Xm2; break;
      case   5: bf.fp   = sens; break;
      case   6: bf.fp   = Im1[0]; break;
      case   7: bf.fp   = Im1[1]; break;
      case   8: bf.fp   = Im1[2]; break;
      case   9: bf.fp   = Im1cal[0]; break;
      case  10: bf.fp   = Im1cal[1]; break;
      case  11: bf.fp   = Im1cal[2]; break;
      case  12: bf.fp   = sensCalVal; break;
      case  13: bf.fp   = prbs_gain; break;
      case  14: bf.fp   = prbs_out; break;
      case  15: bf.fp   = ss_phase; break;
      case  16: bf.fp   = ss_fstart; break;
      case  17: bf.fp   = ss_fstep; break;
      case  18: bf.fp   = ss_fend; break;
      case  19: bf.fp   = ss_gain; break;
      case  20: bf.fp   = ss_offset; break;
      case  21: bf.fp   = ss_f; break;
      case  22: bf.fp   = ss_tstart; break;
      case  23: bf.fp   = ss_out; break;
      case  24: bf.fp   = music_gain; break;
      case  25: bf.fp   = music_out; break;
      case  26: bf.fp   = muziek_gain_V; break;
      case  27: bf.fp   = distoff; break;
      case  28: bf.fp   = dist; break;
      case  29: bf.fp   = distcurQ; break;
      case  30: bf.fp   = distcurD; break;
      case  31: bf.fp   = distcurSP; break;
      case  32: bf.fp   = distpos; break;
      case  33: bf.fp   = distposSP; break;
      case  34: bf.fp   = sp1_tstart; break;
      case  35: bf.fp   = sp1_t1; break;
      case  36: bf.fp   = sp1_t2; break;
      case  37: bf.fp   = sp1_t3; break;
      case  38: bf.fp   = sp1_p; break;
      case  39: bf.fp   = sp1_vmax; break;
      case  40: bf.fp   = sp1_amax; break;
      case  41: bf.fp   = sp1_jmax; break;
      case  42: bf.fp   = acc; break;
      case  43: bf.fp   = vel; break;
      case  44: bf.fp   = Xsp1; break;
      case  45: bf.fp   = Xsp2; break;
      case  46: bf.fp   = Xsp1Offs; break;
      case  47: bf.fp   = Xsp2Offs; break;
      case  48: bf.fp   = offsetVelTot; break;
      case  49: bf.fp   = offsetVel; break;
      case  50: bf.fp   = Jload; break;
      case  51: bf.fp   = Xe1; break;
      case  52: bf.fp   = Xe2; break;
      case  53: bf.fp   = PC1out; break;
      case  54: bf.fp   = PC2out; break;
      case  55: bf.fp   = Intout1; break;
      case  56: bf.fp   = Intout2; break;
      case  57: bf.fp   = lastpmechy; break;
      case  58: bf.fp   = lastpmechphi; break;
      case  59: bf.fp   = vmechy; break;
      case  60: bf.fp   = vmechphi; break;
      case  61: bf.fp   = fBW; break;
      case  62: bf.fp   = alpha1; break;
      case  63: bf.fp   = alpha2; break;
      case  64: bf.fp   = fInt; break;
      case  65: bf.fp   = fLP; break;
      case  66: bf.fp   = Kp_PC1; break;
      case  67: bf.fp   = Kp_PC2; break;
      case  68: bf.fp   = velFF; break;
      case  69: bf.fp   = R; break;
      case  70: bf.fp   = Kt1; break;
      case  71: bf.fp   = Isp1dq[0]; break;
      case  72: bf.fp   = Isp1dq[1]; break;
      case  73: bf.fp   = Ie1dq[0]; break;
      case  74: bf.fp   = Ie1dq[1]; break;
      case  75: bf.fp   = Vccout1dq[0]; break;
      case  76: bf.fp   = Vccout1dq[1]; break;
      case  77: bf.fp   = VccFF1dq[0]; break;
      case  78: bf.fp   = VccFF1dq[1]; break;
      case  79: bf.fp   = CC1kp; break;
      case  80: bf.fp   = CC1fInt; break;
      case  81: bf.fp   = V_bus; break;
      case  82: bf.fp   = Im1ab[0]; break;
      case  83: bf.fp   = Im1ab[1]; break;
      case  84: bf.fp   = Im1dq[0]; break;
      case  85: bf.fp   = Im1dq[1]; break;
      case  86: bf.fp   = Vph1ab[0]; break;
      case  87: bf.fp   = Vph1ab[1]; break;
      case  88: bf.fp   = Vph1dq[0]; break;
      case  89: bf.fp   = Vph1dq[1]; break;
      case  90: bf.fp   = Vph1dqOffs[0]; break;
      case  91: bf.fp   = Vph1dqOffs[1]; break;
      case  92: bf.fp   = Vph1[0]; break;
      case  93: bf.fp   = Vph1[1]; break;
      case  94: bf.fp   = Vph1[2]; break;
      case  95: bf.fp   = ThetaEcalAmp; break;
      case  96: bf.fp   = ThetaEcalFreq; break;
      case  97: bf.fp   = ThetaE1; break;
      case  98: bf.fp   = ThetaE1cal; break;
      case  99: bf.fp   = ThetaE1sin; break;
      case 100: bf.fp   = ThetaE1cos; break;
      case 101: bf.fp   = SVM1shift; break;
      case 102: bf.fp   = I_max; break;
      case 103: bf.fp   = V_max; break;
      case 104: bf.fp   = e_max; break;
      case 105: bf.fp   = PWMph1offs[0]; break;
      case 106: bf.fp   = PWMph1offs[1]; break;
      case 107: bf.fp   = PWMph1offs[2]; break;
      case 108: bf.sint = incomingByte; break;
      case 109: bf.sint = outLed; break;
      case 110: bf.sint = outPhA1; break;
      case 111: bf.sint = outPhB1; break;
      case 112: bf.sint = outPhC1; break;
      case 113: bf.sint = outEn1; break;
      case 114: bf.sint = outEnBrake1; break;
      case 115: bf.sint = inA1; break;
      case 116: bf.sint = inB1; break;
      case 117: bf.sint = inZ1; break;
      case 118: bf.sint = inA2; break;
      case 119: bf.sint = inB2; break;
      case 120: bf.sint = inZ2; break;
      case 121: bf.sint = inIA1; break;
      case 122: bf.sint = inIB1; break;
      case 123: bf.sint = inIC1; break;
      case 124: bf.sint = enc1np; break;
      case 125: bf.sint = EncPos1Raw; break;
      case 126: bf.sint = EncPos2Raw; break;
      case 127: bf.sint = IndexFound1; break;
      case 128: bf.sint = IndexFound2; break;
      case 129: bf.sint = sensorValue; break;
      case 130: bf.sint = state; break;
      case 131: bf.sint = stateprev; break;
      case 132: bf.sint = enBrake; break;
      case 133: bf.sint = endistcurQ; break;
      case 134: bf.sint = endistcurD; break;
      case 135: bf.sint = endistcurSP; break;
      case 136: bf.sint = endistpos; break;
      case 137: bf.sint = endistposSP; break;
      case 138: bf.sint = spGO; break;
      case 139: bf.sint = spSET; break;
      case 140: bf.sint = IntegOn; break;
      case 141: bf.sint = npole1; break;
      case 142: bf.sint = enSVM; break;
      case 143: bf.sint = errcode; break;
      case 144: bf.sint = PWMph1[0]; break;
      case 145: bf.sint = PWMph1[1]; break;
      case 146: bf.sint = PWMph1[2]; break;
      case 147: bf.sint = timeremain; break;
      case 148: bf.sint = TET; break;
      case 149: bf.uint = zenddata; break;
      case 150: bf.uint = Nsend; break;
      case 151: bf.uint = sigid; break;
      case 152: bf.uint = Ndownsample; break;
      case 153: bf.uint = downsample; break;
      case 154: bf.uint = ss_n_aver; break;
      case 155: bf.uint = calctime; break;
      case 156: bf.uint = previousTime; break;
      case 157: bf.uint = timePrev; break;
      case 158: bf.uint = curtime; break;
      case 159: bf.uint = curloop; break;
    }
    Serial.write( bf.bin , 4);
  }
}

void setpar( int isignal , binaryFloat bf ) {
  switch( isignal ){
    case   0: adc2V = bf.fp; break;
    case   1: enc2rad = bf.fp; break;
    case   2: enc2m = bf.fp; break;
    case   3: Xm1 = bf.fp; break;
    case   4: Xm2 = bf.fp; break;
    case   5: sens = bf.fp; break;
    case   6: Im1[0] = bf.fp; break;
    case   7: Im1[1] = bf.fp; break;
    case   8: Im1[2] = bf.fp; break;
    case   9: Im1cal[0] = bf.fp; break;
    case  10: Im1cal[1] = bf.fp; break;
    case  11: Im1cal[2] = bf.fp; break;
    case  12: sensCalVal = bf.fp; break;
    case  13: prbs_gain = bf.fp; break;
    case  14: prbs_out = bf.fp; break;
    case  15: ss_phase = bf.fp; break;
    case  16: ss_fstart = bf.fp; break;
    case  17: ss_fstep = bf.fp; break;
    case  18: ss_fend = bf.fp; break;
    case  19: ss_gain = bf.fp; break;
    case  20: ss_offset = bf.fp; break;
    case  21: ss_f = bf.fp; break;
    case  22: ss_tstart = bf.fp; break;
    case  23: ss_out = bf.fp; break;
    case  24: music_gain = bf.fp; break;
    case  25: music_out = bf.fp; break;
    case  26: muziek_gain_V = bf.fp; break;
    case  27: distoff = bf.fp; break;
    case  28: dist = bf.fp; break;
    case  29: distcurQ = bf.fp; break;
    case  30: distcurD = bf.fp; break;
    case  31: distcurSP = bf.fp; break;
    case  32: distpos = bf.fp; break;
    case  33: distposSP = bf.fp; break;
    case  34: sp1_tstart = bf.fp; break;
    case  35: sp1_t1 = bf.fp; break;
    case  36: sp1_t2 = bf.fp; break;
    case  37: sp1_t3 = bf.fp; break;
    case  38: sp1_p = bf.fp; break;
    case  39: sp1_vmax = bf.fp; break;
    case  40: sp1_amax = bf.fp; break;
    case  41: sp1_jmax = bf.fp; break;
    case  42: acc = bf.fp; break;
    case  43: vel = bf.fp; break;
    case  44: Xsp1 = bf.fp; break;
    case  45: Xsp2 = bf.fp; break;
    case  46: Xsp1Offs = bf.fp; break;
    case  47: Xsp2Offs = bf.fp; break;
    case  48: offsetVelTot = bf.fp; break;
    case  49: offsetVel = bf.fp; break;
    case  50: Jload = bf.fp; break;
    case  51: Xe1 = bf.fp; break;
    case  52: Xe2 = bf.fp; break;
    case  53: PC1out = bf.fp; break;
    case  54: PC2out = bf.fp; break;
    case  55: Intout1 = bf.fp; break;
    case  56: Intout2 = bf.fp; break;
    case  57: lastpmechy = bf.fp; break;
    case  58: lastpmechphi = bf.fp; break;
    case  59: vmechy = bf.fp; break;
    case  60: vmechphi = bf.fp; break;
    case  61: fBW = bf.fp; break;
    case  62: alpha1 = bf.fp; break;
    case  63: alpha2 = bf.fp; break;
    case  64: fInt = bf.fp; break;
    case  65: fLP = bf.fp; break;
    case  66: Kp_PC1 = bf.fp; break;
    case  67: Kp_PC2 = bf.fp; break;
    case  68: velFF = bf.fp; break;
    case  69: R = bf.fp; break;
    case  70: Kt1 = bf.fp; break;
    case  71: Isp1dq[0] = bf.fp; break;
    case  72: Isp1dq[1] = bf.fp; break;
    case  73: Ie1dq[0] = bf.fp; break;
    case  74: Ie1dq[1] = bf.fp; break;
    case  75: Vccout1dq[0] = bf.fp; break;
    case  76: Vccout1dq[1] = bf.fp; break;
    case  77: VccFF1dq[0] = bf.fp; break;
    case  78: VccFF1dq[1] = bf.fp; break;
    case  79: CC1kp = bf.fp; break;
    case  80: CC1fInt = bf.fp; break;
    case  81: V_bus = bf.fp; break;
    case  82: Im1ab[0] = bf.fp; break;
    case  83: Im1ab[1] = bf.fp; break;
    case  84: Im1dq[0] = bf.fp; break;
    case  85: Im1dq[1] = bf.fp; break;
    case  86: Vph1ab[0] = bf.fp; break;
    case  87: Vph1ab[1] = bf.fp; break;
    case  88: Vph1dq[0] = bf.fp; break;
    case  89: Vph1dq[1] = bf.fp; break;
    case  90: Vph1dqOffs[0] = bf.fp; break;
    case  91: Vph1dqOffs[1] = bf.fp; break;
    case  92: Vph1[0] = bf.fp; break;
    case  93: Vph1[1] = bf.fp; break;
    case  94: Vph1[2] = bf.fp; break;
    case  95: ThetaEcalAmp = bf.fp; break;
    case  96: ThetaEcalFreq = bf.fp; break;
    case  97: ThetaE1 = bf.fp; break;
    case  98: ThetaE1cal = bf.fp; break;
    case  99: ThetaE1sin = bf.fp; break;
    case 100: ThetaE1cos = bf.fp; break;
    case 101: SVM1shift = bf.fp; break;
    case 102: I_max = bf.fp; break;
    case 103: V_max = bf.fp; break;
    case 104: e_max = bf.fp; break;
    case 105: PWMph1offs[0] = bf.fp; break;
    case 106: PWMph1offs[1] = bf.fp; break;
    case 107: PWMph1offs[2] = bf.fp; break;
    case 108: incomingByte = bf.sint; break;
    case 124: enc1np = bf.sint; break;
    case 125: EncPos1Raw = bf.sint; break;
    case 126: EncPos2Raw = bf.sint; break;
    case 127: IndexFound1 = bf.sint; break;
    case 128: IndexFound2 = bf.sint; break;
    case 129: sensorValue = bf.sint; break;
    case 130: state = bf.sint; break;
    case 131: stateprev = bf.sint; break;
    case 132: enBrake = bf.sint; break;
    case 133: endistcurQ = bf.sint; break;
    case 134: endistcurD = bf.sint; break;
    case 135: endistcurSP = bf.sint; break;
    case 136: endistpos = bf.sint; break;
    case 137: endistposSP = bf.sint; break;
    case 138: spGO = bf.sint; break;
    case 139: spSET = bf.sint; break;
    case 140: IntegOn = bf.sint; break;
    case 141: npole1 = bf.sint; break;
    case 142: enSVM = bf.sint; break;
    case 143: errcode = bf.sint; break;
    case 144: PWMph1[0] = bf.sint; break;
    case 145: PWMph1[1] = bf.sint; break;
    case 146: PWMph1[2] = bf.sint; break;
    case 147: timeremain = bf.sint; break;
    case 148: TET = bf.sint; break;
    case 149: zenddata = bf.uint; break;
    case 150: Nsend = bf.uint; break;
    case 151: sigid = bf.uint; break;
    case 152: Ndownsample = bf.uint; break;
    case 153: downsample = bf.uint; break;
    case 154: ss_n_aver = bf.uint; break;
    case 155: calctime = bf.uint; break;
    case 156: previousTime = bf.uint; break;
    case 157: timePrev = bf.uint; break;
    case 158: curtime = bf.uint; break;
    case 159: curloop = bf.uint; break;
  }
}

void printSignals( unsigned int selected ) {
  char *signalNames[] = { "adc2V", "enc2rad", "enc2m", "Xm1", "Xm2", "sens", "Im1[0]", "Im1[1]", "Im1[2]", "Im1cal[0]", "Im1cal[1]", "Im1cal[2]", "sensCalVal", "prbs_gain", "prbs_out", "ss_phase", "ss_fstart", "ss_fstep", "ss_fend", "ss_gain", "ss_offset", "ss_f", "ss_tstart", "ss_out", "music_gain", "music_out", "muziek_gain_V", "distoff", "dist", "distcurQ", "distcurD", "distcurSP", "distpos", "distposSP", "sp1_tstart", "sp1_t1", "sp1_t2", "sp1_t3", "sp1_p", "sp1_vmax", "sp1_amax", "sp1_jmax", "acc", "vel", "Xsp1", "Xsp2", "Xsp1Offs", "Xsp2Offs", "offsetVelTot", "offsetVel", "Jload", "Xe1", "Xe2", "PC1out", "PC2out", "Intout1", "Intout2", "lastpmechy", "lastpmechphi", "vmechy", "vmechphi", "fBW", "alpha1", "alpha2", "fInt", "fLP", "Kp_PC1", "Kp_PC2", "velFF", "R", "Kt1", "Isp1dq[0]", "Isp1dq[1]", "Ie1dq[0]", "Ie1dq[1]", "Vccout1dq[0]", "Vccout1dq[1]", "VccFF1dq[0]", "VccFF1dq[1]", "CC1kp", "CC1fInt", "V_bus", "Im1ab[0]", "Im1ab[1]", "Im1dq[0]", "Im1dq[1]", "Vph1ab[0]", "Vph1ab[1]", "Vph1dq[0]", "Vph1dq[1]", "Vph1dqOffs[0]", "Vph1dqOffs[1]", "Vph1[0]", "Vph1[1]", "Vph1[2]", "ThetaEcalAmp", "ThetaEcalFreq", "ThetaE1", "ThetaE1cal", "ThetaE1sin", "ThetaE1cos", "SVM1shift", "I_max", "V_max", "e_max", "PWMph1offs[0]", "PWMph1offs[1]", "PWMph1offs[2]", "incomingByte", "outLed", "outPhA1", "outPhB1", "outPhC1", "outEn1", "outEnBrake1", "inA1", "inB1", "inZ1", "inA2", "inB2", "inZ2", "inIA1", "inIB1", "inIC1", "enc1np", "EncPos1Raw", "EncPos2Raw", "IndexFound1", "IndexFound2", "sensorValue", "state", "stateprev", "enBrake", "endistcurQ", "endistcurD", "endistcurSP", "endistpos", "endistposSP", "spGO", "spSET", "IntegOn", "npole1", "enSVM", "errcode", "PWMph1[0]", "PWMph1[1]", "PWMph1[2]", "timeremain", "TET", "zenddata", "Nsend", "sigid", "Ndownsample", "downsample", "ss_n_aver", "calctime", "previousTime", "timePrev", "curtime", "curloop",  };
  char *signalTypes[] = { "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "I", "I", "I", "I", "I", "I", "I", "I", "I", "I", "I",  };
  char *signalSizes[] = { "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "3;0", "3;0", "3;0", "3;0", "3;0", "3;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "2;0", "2;0", "2;0", "2;0", "2;0", "2;0", "2;0", "2;0", "0;0", "0;0", "0;0", "2;0", "2;0", "2;0", "2;0", "2;0", "2;0", "2;0", "2;0", "2;0", "2;0", "3;0", "3;0", "3;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "3;0", "3;0", "3;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "3;0", "3;0", "3;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0",  };
  int imax = 10;
  switch(selected){
    case 0: imax = 160; break;
  }
  for ( int i = 0; i < imax; i++) {
    Serial.println( signalNames[i] );
    Serial.println( signalTypes[i] );
    Serial.println( signalSizes[i] );
  }
  Nsend = 0;
  getsignalnames = 1;
}
