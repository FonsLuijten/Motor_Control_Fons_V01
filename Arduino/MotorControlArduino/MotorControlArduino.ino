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

#include <Biquad.h>
#include <Math.h>
#include <ControlTools.h>
#include "muziek.c"
#include <MotionProfile2.h>
#include "defines.h"
#include "ringbuffer.h"

int avgvel = 50;
RingBuffer <float> ringBufY(avgvel);
RingBuffer <float> ringBufPhi(avgvel);
RingBuffer <float> ringBufTdiff(avgvel);

int getsignalnames = 0;

#include <ADC.h>
#include <ADC_util.h>

ADC *adc = new ADC(); // adc object

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup basic functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  pinMode(outLed, OUTPUT);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
    digitalWriteFast(outLed, HIGH);
    delayMicroseconds(80);
    digitalWriteFast(outLed, LOW);
    delayMicroseconds(10);
  }
  digitalWriteFast(outLed, LOW);
  
  //pinMode(brake, OUTPUT);

  pinMode(outPhA1, OUTPUT);
  pinMode(outPhB1, OUTPUT);
  pinMode(outPhC1, OUTPUT);

  analogWriteFrequency(outPhA1, 20e3); // Teensy 3.0 pin 3 also changes to 375 kHz
  analogWriteFrequency(outPhB1, 20e3); // Teensy 3.0 pin 3 also changes to 375 kHz
  analogWriteFrequency(outPhC1, 20e3); // Teensy 3.0 pin 3 also changes to 375 kHz 

  analogWriteResolution(12);  // analogWrite value 0 to 4095, or 4096 for high
  analogReadResolution(12);

  pinMode( inA1 , INPUT);
  pinMode( inB1 , INPUT);
  pinMode( inZ1 , INPUT);
  attachInterrupt( inA1 , doA1 , CHANGE);
  attachInterrupt( inB1 , doB1 , CHANGE);
  attachInterrupt( inZ1 , doZ1 , CHANGE);
  pinMode( inA2 , INPUT);
  pinMode( inB2 , INPUT);
  pinMode( inZ2 , INPUT);
  attachInterrupt( inA2 , doA2 , CHANGE);
  attachInterrupt( inB2 , doB2 , CHANGE);
  attachInterrupt( inZ2 , doZ2 , CHANGE);

  analogWrite( outPhA1, 0);
  analogWrite( outPhB1, 0);
  analogWrite( outPhC1, 0);
  delay(100);

  //Average measurement current sensor calibration
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

  Nsend = 0;
  //digitalWriteFast(brake, HIGH);
  
  while (getsignalnames == 0) {
    processSerialIn();
  }
  delay(500);
  
  while (Serial.available() > 4) {
    processSerialIn();
  }
  
  timePrev = micros();
  // Shift control loop to fixed offset versus PWM loop
  //timePrev += Ts / 4 + Ts - timePrev % Ts;
  //timePrev += 0 - timePrev % Ts; 

  
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  curloop++;

  /*Control related tasks*/
  ReadInputs();
  //Setpoints();
  //PositionControl();
  CurrentControl();
  Safety();
  SendOutputs();

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
  for(int ii = 0; ii < 3; ii++) Im1[ii] = ((float(sensI1[ii]) * adc2V) - Im1cal[ii]) / 0.09;

  /*Positions*/
  penc1 = EncPos1Raw * enc2rad;
  penc2 = EncPos2Raw * enc2rad;
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
  
  /*Single sine identification*/
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
  ss_out = ss_offset + ss_gain * sin( ss_phase ); //sin() measured to be faster then sinf()

  /*Position control setpoints*/
  rpos1 = SPprofile->stateCalculation( spGO );
  float acc = SPprofile->aref;
  float vel = SPprofile->vref;
  offsetVelTot += offsetVel;
  rpos1 += offsetVelTot ;
  rpos2 = 0;
}

void PositionControl() {
  
  emechy = rpos1 - penc1;
  emechphi = rpos2 - penc2;

  
 /*Safety*/
 if (abs(emechy) > 0.1){
   PCstate = 0;
 }
 if (abs(emechphi) > 0.5 & PCstate == 3){
   PCstate = 0;
 }
 if (OutputOn == false) {
   PCstate = 0;
 }


 /*Position control state*/
 if (PCstate == 1){ // Cart only
   mechcontout = Kp_PC1 * emechy;
   mechcontout = leadlag_PC1->process( mechcontout );
   //digitalWriteFast(1 , HIGH);
   mechcontout = lowpass_PC1->process( mechcontout );
   //mechcontout = notch->process( mechcontout );
   //digitalWriteFast(1 , LOW);
 
   if (IntegOn == 1){
     Intout1 = integrator_PC1->processclip( mechcontout , -I_max * Kt - mechcontout , I_max * Kt - mechcontout );
     mechcontout += Intout1;
   }
   else{
     integrator_PC1->setState(0);
   }

   mechcontout += acc * Jload;
   mechcontout += vel * velFF;
   
 }
 else if (PCstate == 2){ // State feedback pendulum
  calcvel();
  k1 = -2.5; 
  k2 = -2.66642077414672;
  k3 = -12.1591323401845;
  k4 = -2.45600514544917;
  mechcontout = -1*EnStateFB*(k1*penc1+k2*vmechy+k1*penc2+k2*vmechphi);
 }
 else if (PCstate == 3){ // Cascade feedback pendulum

   PC2out = Kp_PC2 * leadlag_PC2->process( emechy );
   //PC2out = lowpass_PC2->process( PC2out );
   Intout2 = integrator_PC2->processclip( PC2out , -M_PI - PC2out , M_PI - PC2out );
   PC2out += Intout2;
  
   PC1out = Kp_PC1 * leadlag_PC1->process( emechphi + PC2out );
   PC1out = lowpass_PC1->process( PC1out );   
   Intout1 = integrator_PC1->processclip( PC1out , -I_max * Kt - PC1out , I_max * Kt - PC1out );
   PC1out += Intout1;
   
   mechcontout = PC1out; //Controller tuned on 
 }
 else{ // Position control off
   mechcontout = 0;
   PC1out = 0;
   PC2out = 0;
   integrator_PC1->setState(0);
   integrator_PC2->setState(0);
 }
}

void CurrentControl() {


  /*Disturbance*/
//  dist = distval * 1 * (noisebit - 0.5) + distoff + ss_out;

  /*Current control*/
//  cursp = mechcontout / Kt + dist;
//  cursp += muziek_gain * muziek[ (curloop / (50 / Ts)) % (sizeof(muziek) / 4) ];

//  e = cursp - sens;
  
//  Vout = Icontgain * e;
//  Iout = integratorCC->processclip( Vout , -V_max - Vout , V_max - Vout );
//  Vout += Iout;
  //Vout += dist;
//  Vout += cursp * R;
//  Vout += muziek_gain_V * muziek[ (curloop / (50 / Ts)) % (sizeof(muziek) / 4) ];

  /*###FOC###*/
  /*Electrical angle*/
  ThetaE1 = mod(EncPos1Raw*npole1,enc1np)*enc2rad;
  ThetaE1sin = sin(ThetaE1);
  ThetaE1cos = cos(ThetaE1);
    
  /*Clarke*/
  Im1ab[0] = Im1[0];
  Im1ab[1] = 1/sqrt(3)*Im1[0] + 2/sqrt(3)*Im1[1];
  
  /*Park*/
  Im1dq[0] = ThetaE1sin*Im1ab[0] - ThetaE1cos*Im1ab[1];
  Im1dq[1] = ThetaE1cos*Im1ab[0] + ThetaE1sin*Im1ab[1];
  
  /*Control*/
  Vph1dq[0] = 0;
  Vph1dq[1] = 0;
  
  /*Inverse Park*/
  Vph1ab[0] = ThetaE1sin*Vph1dq[0] + ThetaE1cos*Vph1dq[1];
  Vph1ab[1] = -ThetaE1cos*Vph1dq[0] + ThetaE1sin*Vph1dq[1];
  
  /*Inverse Clarke*/
  Vph1[0] = Vph1ab[0];
  Vph1[1] = -1/2*Vph1ab[0] + sqrt(3)/2*Vph1ab[1];
  Vph1[2] = -1/2*Vph1ab[0] - sqrt(3)/2*Vph1ab[1];
  
  /*SVM*/
  float SVMshift = 1/2*(min(min(Vph1[0],Vph1[1]),Vph1[2])+max(max(Vph1[0],Vph1[1]),Vph1[2]));
  Vph1[0] = Vph1[0]-SVMshift;
  Vph1[1] = Vph1[1]-SVMshift;
  Vph1[2] = Vph1[2]-SVMshift;
}


void Safety(){
  /* First check for errors */
  if (OutputOn == true) {
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
  }

  /* When errors found turn off output */
  if (OutputOn == false) {
    PWMph1[0] = 0;
    PWMph1[1] = 0;
    PWMph1[2] = 0;
    integratorCC->setState(0);
    state = 0;
  }
  else{
    /*PWM generation*/
    if (state == 0){
      PWMph1[0] = 0;
      PWMph1[1] = 0;
      PWMph1[2] = 0;
      integratorCC->setState(0);
    }
    else if (state == 1 || state == 2){
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
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Control helper Functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void calcvel(){

  previousTime = calctime;                                           // Save the time of the previous cycle
  calctime = millis();
  float timeChange = calctime - previousTime;
  
  ringBufTdiff.addValue(timeChange);
  ringBufY.addValue(penc1 - lastpmechy);
  ringBufPhi.addValue(penc2 - lastpmechphi);
  
  lastpmechy = penc1;
  lastpmechphi = penc2;

  vmechy = ringBufY.getAverage()/ringBufTdiff.getAverage();
  vmechphi = ringBufPhi.getAverage()/ringBufTdiff.getAverage();
  
  
//  if (curloop % 10 == 0){
 //   previousTime = calctime;                                           // Save the time of the previous cycle
   // calctime = millis();
//    float timeChange = calctime - previousTime;
  //  
    //vmechy = (pmechy - lastpmechy) / timeChange;
//    lastpmechy = pmechy;
//
  //  vmechphi = (pmechphi - lastpmechphi) / timeChange;
    //lastpmechphi = pmechphi;
 // }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Encoder interrupt Functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Interrupt on A changing state
void doA1() {
  // Test transition
  A_set1 = digitalReadFast(inA1) == HIGH;
  // and adjust counter + if A leads B
  EncPos1Raw += (A_set1 != B_set1) ? +1 : -1;
}

// Interrupt on B changing state
void doB1() {
  // Test transition
  B_set1 = digitalReadFast(inB1) == HIGH;
  // and adjust counter + if B follows A
  EncPos1Raw += (A_set1 == B_set1) ? +1 : -1;
}

// Interrupt on Z changing state
void doZ1() {
  if (Z_set1 == false){
    // Test transition
    Z_set1 = digitalReadFast(inZ1) == HIGH;
    // and adjust counter + if B follows A
    EncPos1Raw = 0;
  }
}

// Interrupt on A changing state
void doA2() {
  // Test transition
  A_set2 = digitalReadFast(inA2) == HIGH;
  digitalWriteFast(outLed, A_set2);   // set the LED on
  // and adjust counter + if A leads B
  EncPos2Raw += (A_set2 != B_set2) ? +1 : -1;
}

// Interrupt on B changing state
void doB2() {
  // Test transition
  B_set2 = digitalReadFast(inB2) == HIGH;
  //digitalWriteFast(1, B_set2);   // set the LED on
  // and adjust counter + if B follows A
  EncPos2Raw += (A_set2 == B_set2) ? +1 : -1;
}

// Interrupt on Z changing state
void doZ2() {
  if (Z_set2 == false){
    // Test transition
    Z_set2 = digitalReadFast(inZ2) == HIGH;
    // and adjust counter + if B follows A
    EncPos2Raw = 0;
  }
}

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
    
    // Single sine settings
    if (settingByte == 's') {
      ss_gain = ser_in.fp;
      ss_f = ss_fstart;
      ss_phase = 0;
      ss_tstart = (timePrev + Ts) / 1e6;
    }
    //reset integrators
    if (settingByte == 'n') {
      OutputOn = false;
    }
    // Restart controller
    if (settingByte == 'o') {
      OutputOn = true;
      SPprofile->REFqmem = 0;
    }
    // Play setpoint
    if (settingByte == 'p') {
      if (spGO == 0) {
        spGO = 1;
      }
      else {
        spGO = 0;
      }
    }
    // Set setpoint
    if (settingByte == '1') {
      SPprofile->t1 = ser_in.fp;
    }
    if (settingByte == '2') {
      SPprofile->t2 = ser_in.fp;
    }
    if (settingByte == '3') {
      SPprofile->t3 = ser_in.fp;
    }
    if (settingByte == '4') {
      SPprofile->p = ser_in.fp;
    }
    if (settingByte == '5') {
      SPprofile->v_max = ser_in.fp;
    }
    if (settingByte == '6') {
      SPprofile->a_max = ser_in.fp;
    }
    if (settingByte == '7') {
      SPprofile->j_max = ser_in.fp;
      SPprofile->init();
    }
    if (settingByte == '8') {
      Icontgain = ser_in.fp;
    }
    if (settingByte == '9') {
      integratorCC = new Integrator( ser_in.fp , 1 / T);
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
    // Set controller
    if (settingByte == 'C') {
      ContSelect = ser_in.uint;
      if ( ContSelect == 1) {
        Kp_PC1 = 0;
        fBW = 50.0;
        alpha1 = 3.0;
        alpha2 = 4.0;
        fInt = fBW / 6.0;
        fLP = fBW * 6.0;
        integrator_PC1 = new Integrator( fInt , 1 / T);
        leadlag_PC1    = new LeadLag( fBW , alpha1 , alpha2 , 1 / T);
        lowpass_PC1    = new Biquad( bq_type_lowpass , fLP , 0.7, 1 / T);
      }
      if ( ContSelect == 3) {
        Kp_PC1 = 0;
        fBW = 30;
        alpha1 = 3.0;
        alpha2 = 4.0;
        fInt = fBW / 6.0;
        fLP = fBW * 6.0;
        integrator_PC1 = new Integrator( fInt , 1 / T);
        leadlag_PC1    = new LeadLag( fBW , alpha1 , alpha2 , 1 / T);
        lowpass_PC1    = new Biquad( bq_type_lowpass , fLP , 0.7, 1 / T);
        Kp_PC2 = 0;
        fBW = 0.1;
        alpha1 = 3.0;
        alpha2 = 4.0;
        fInt = fBW / 6.0;
        fLP = fBW * 6.0;
        integrator_PC2 = new Integrator( fInt , 1 / T);
        leadlag_PC2    = new LeadLag( fBW , alpha1 , alpha2 , 1 / T);
        lowpass_PC2    = new Biquad( bq_type_lowpass , fLP , 0.7, 1 / T);
      }
    }
    if (settingByte == 'c') {
      ContSelect = ser_in.uint;
      if ( ContSelect == 1) {
        integrator_PC1 = new Integrator( fInt , 1 / T);
        leadlag_PC1    = new LeadLag( fBW , alpha1 , alpha2 , 1 / T);
        lowpass_PC1    = new Biquad( bq_type_lowpass , fLP , 0.7, 1 / T);
      }
      if ( ContSelect == 2) {
        integrator_PC2 = new Integrator( fInt , 1 / T);
        leadlag_PC2    = new LeadLag( fBW , alpha1 , alpha2 , 1 / T);
        lowpass_PC2    = new Biquad( bq_type_lowpass , fLP , 0.7, 1 / T);
      }
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
      case   3: bf.fp   = penc1; break;
      case   4: bf.fp   = penc2; break;
      case   5: bf.fp   = y; break;
      case   6: bf.fp   = sens; break;
      case   7: bf.fp   = Im1[0]; break;
      case   8: bf.fp   = Im1[1]; break;
      case   9: bf.fp   = Im1[2]; break;
      case  10: bf.fp   = Im1cal[0]; break;
      case  11: bf.fp   = Im1cal[1]; break;
      case  12: bf.fp   = Im1cal[2]; break;
      case  13: bf.fp   = sensCalVal; break;
      case  14: bf.fp   = ss_phase; break;
      case  15: bf.fp   = ss_fstart; break;
      case  16: bf.fp   = ss_fstep; break;
      case  17: bf.fp   = ss_fend; break;
      case  18: bf.fp   = ss_gain; break;
      case  19: bf.fp   = ss_offset; break;
      case  20: bf.fp   = ss_f; break;
      case  21: bf.fp   = ss_tstart; break;
      case  22: bf.fp   = ss_out; break;
      case  23: bf.fp   = rpos1; break;
      case  24: bf.fp   = rpos2; break;
      case  25: bf.fp   = r; break;
      case  26: bf.fp   = acc; break;
      case  27: bf.fp   = vel; break;
      case  28: bf.fp   = dist; break;
      case  29: bf.fp   = cursp; break;
      case  30: bf.fp   = Iout; break;
      case  31: bf.fp   = velFF; break;
      case  32: bf.fp   = R; break;
      case  33: bf.fp   = offsetVelTot; break;
      case  34: bf.fp   = offsetVel; break;
      case  35: bf.fp   = Jload; break;
      case  36: bf.fp   = mechcontout; break;
      case  37: bf.fp   = PC1out; break;
      case  38: bf.fp   = PC2out; break;
      case  39: bf.fp   = Intout1; break;
      case  40: bf.fp   = Intout2; break;
      case  41: bf.fp   = muziek_gain; break;
      case  42: bf.fp   = muziek_gain_V; break;
      case  43: bf.fp   = distval; break;
      case  44: bf.fp   = distoff; break;
      case  45: bf.fp   = emechy; break;
      case  46: bf.fp   = emechphi; break;
      case  47: bf.fp   = lastpmechy; break;
      case  48: bf.fp   = lastpmechphi; break;
      case  49: bf.fp   = vmechy; break;
      case  50: bf.fp   = vmechphi; break;
      case  51: bf.fp   = fBW; break;
      case  52: bf.fp   = alpha1; break;
      case  53: bf.fp   = alpha2; break;
      case  54: bf.fp   = fInt; break;
      case  55: bf.fp   = fLP; break;
      case  56: bf.fp   = Kp_PC1; break;
      case  57: bf.fp   = Kp_PC2; break;
      case  58: bf.fp   = k1; break;
      case  59: bf.fp   = k2; break;
      case  60: bf.fp   = k3; break;
      case  61: bf.fp   = k4; break;
      case  62: bf.fp   = Kt; break;
      case  63: bf.fp   = npole1; break;
      case  64: bf.fp   = e; break;
      case  65: bf.fp   = Icontout; break;
      case  66: bf.fp   = Vout; break;
      case  67: bf.fp   = fIntCur; break;
      case  68: bf.fp   = Icontgain; break;
      case  69: bf.fp   = V_bus; break;
      case  70: bf.fp   = Im1ab[0]; break;
      case  71: bf.fp   = Im1ab[1]; break;
      case  72: bf.fp   = Im1dq[0]; break;
      case  73: bf.fp   = Im1dq[1]; break;
      case  74: bf.fp   = Vph1ab[0]; break;
      case  75: bf.fp   = Vph1ab[1]; break;
      case  76: bf.fp   = Vph1dq[0]; break;
      case  77: bf.fp   = Vph1dq[1]; break;
      case  78: bf.fp   = Vph1[0]; break;
      case  79: bf.fp   = Vph1[1]; break;
      case  80: bf.fp   = Vph1[2]; break;
      case  81: bf.fp   = ThetaE1; break;
      case  82: bf.fp   = ThetaE1sin; break;
      case  83: bf.fp   = ThetaE1cos; break;
      case  84: bf.fp   = I_max; break;
      case  85: bf.fp   = V_max; break;
      case  86: bf.fp   = PWMph1offs[0]; break;
      case  87: bf.fp   = PWMph1offs[1]; break;
      case  88: bf.fp   = PWMph1offs[2]; break;
      case  89: bf.sint = test1; break;
      case  90: bf.sint = test2; break;
      case  91: bf.sint = test3; break;
      case  92: bf.sint = incomingByte; break;
      case  93: bf.sint = outLed; break;
      case  94: bf.sint = outPhA1; break;
      case  95: bf.sint = outPhB1; break;
      case  96: bf.sint = outPhC1; break;
      case  97: bf.sint = inA1; break;
      case  98: bf.sint = inB1; break;
      case  99: bf.sint = inZ1; break;
      case 100: bf.sint = inA2; break;
      case 101: bf.sint = inB2; break;
      case 102: bf.sint = inZ2; break;
      case 103: bf.sint = inIA1; break;
      case 104: bf.sint = inIB1; break;
      case 105: bf.sint = inIC1; break;
      case 106: bf.sint = EncPos1Raw; break;
      case 107: bf.sint = EncPos2Raw; break;
      case 108: bf.sint = sensorValue; break;
      case 109: bf.sint = state; break;
      case 110: bf.sint = PCstate; break;
      case 111: bf.sint = EnStateFB; break;
      case 112: bf.sint = spGO; break;
      case 113: bf.sint = IntegOn; break;
      case 114: bf.sint = errcode; break;
      case 115: bf.sint = PWMph1[0]; break;
      case 116: bf.sint = PWMph1[1]; break;
      case 117: bf.sint = PWMph1[2]; break;
      case 118: bf.sint = timeremain; break;
      case 119: bf.sint = TET; break;
      case 120: bf.uint = zenddata; break;
      case 121: bf.uint = Nsend; break;
      case 122: bf.uint = sigid; break;
      case 123: bf.uint = Ndownsample; break;
      case 124: bf.uint = downsample; break;
      case 125: bf.uint = ss_n_aver; break;
      case 126: bf.uint = ContSelect; break;
      case 127: bf.uint = calctime; break;
      case 128: bf.uint = previousTime; break;
      case 129: bf.uint = timePrev; break;
      case 130: bf.uint = curtime; break;
      case 131: bf.uint = curloop; break;
    }
    Serial.write( bf.bin , 4);
  }
}

void setpar( int isignal , binaryFloat bf ) {
  switch( isignal ){
    case   0: adc2V = bf.fp; break;
    case   1: enc2rad = bf.fp; break;
    case   2: enc2m = bf.fp; break;
    case   3: penc1 = bf.fp; break;
    case   4: penc2 = bf.fp; break;
    case   5: y = bf.fp; break;
    case   6: sens = bf.fp; break;
    case   7: Im1[0] = bf.fp; break;
    case   8: Im1[1] = bf.fp; break;
    case   9: Im1[2] = bf.fp; break;
    case  10: Im1cal[0] = bf.fp; break;
    case  11: Im1cal[1] = bf.fp; break;
    case  12: Im1cal[2] = bf.fp; break;
    case  13: sensCalVal = bf.fp; break;
    case  14: ss_phase = bf.fp; break;
    case  15: ss_fstart = bf.fp; break;
    case  16: ss_fstep = bf.fp; break;
    case  17: ss_fend = bf.fp; break;
    case  18: ss_gain = bf.fp; break;
    case  19: ss_offset = bf.fp; break;
    case  20: ss_f = bf.fp; break;
    case  21: ss_tstart = bf.fp; break;
    case  22: ss_out = bf.fp; break;
    case  23: rpos1 = bf.fp; break;
    case  24: rpos2 = bf.fp; break;
    case  25: r = bf.fp; break;
    case  26: acc = bf.fp; break;
    case  27: vel = bf.fp; break;
    case  28: dist = bf.fp; break;
    case  29: cursp = bf.fp; break;
    case  30: Iout = bf.fp; break;
    case  31: velFF = bf.fp; break;
    case  32: R = bf.fp; break;
    case  33: offsetVelTot = bf.fp; break;
    case  34: offsetVel = bf.fp; break;
    case  35: Jload = bf.fp; break;
    case  36: mechcontout = bf.fp; break;
    case  37: PC1out = bf.fp; break;
    case  38: PC2out = bf.fp; break;
    case  39: Intout1 = bf.fp; break;
    case  40: Intout2 = bf.fp; break;
    case  41: muziek_gain = bf.fp; break;
    case  42: muziek_gain_V = bf.fp; break;
    case  43: distval = bf.fp; break;
    case  44: distoff = bf.fp; break;
    case  45: emechy = bf.fp; break;
    case  46: emechphi = bf.fp; break;
    case  47: lastpmechy = bf.fp; break;
    case  48: lastpmechphi = bf.fp; break;
    case  49: vmechy = bf.fp; break;
    case  50: vmechphi = bf.fp; break;
    case  51: fBW = bf.fp; break;
    case  52: alpha1 = bf.fp; break;
    case  53: alpha2 = bf.fp; break;
    case  54: fInt = bf.fp; break;
    case  55: fLP = bf.fp; break;
    case  56: Kp_PC1 = bf.fp; break;
    case  57: Kp_PC2 = bf.fp; break;
    case  58: k1 = bf.fp; break;
    case  59: k2 = bf.fp; break;
    case  60: k3 = bf.fp; break;
    case  61: k4 = bf.fp; break;
    case  62: Kt = bf.fp; break;
    case  63: npole1 = bf.fp; break;
    case  64: e = bf.fp; break;
    case  65: Icontout = bf.fp; break;
    case  66: Vout = bf.fp; break;
    case  67: fIntCur = bf.fp; break;
    case  68: Icontgain = bf.fp; break;
    case  69: V_bus = bf.fp; break;
    case  70: Im1ab[0] = bf.fp; break;
    case  71: Im1ab[1] = bf.fp; break;
    case  72: Im1dq[0] = bf.fp; break;
    case  73: Im1dq[1] = bf.fp; break;
    case  74: Vph1ab[0] = bf.fp; break;
    case  75: Vph1ab[1] = bf.fp; break;
    case  76: Vph1dq[0] = bf.fp; break;
    case  77: Vph1dq[1] = bf.fp; break;
    case  78: Vph1[0] = bf.fp; break;
    case  79: Vph1[1] = bf.fp; break;
    case  80: Vph1[2] = bf.fp; break;
    case  81: ThetaE1 = bf.fp; break;
    case  82: ThetaE1sin = bf.fp; break;
    case  83: ThetaE1cos = bf.fp; break;
    case  84: I_max = bf.fp; break;
    case  85: V_max = bf.fp; break;
    case  86: PWMph1offs[0] = bf.fp; break;
    case  87: PWMph1offs[1] = bf.fp; break;
    case  88: PWMph1offs[2] = bf.fp; break;
    case  89: test1 = bf.sint; break;
    case  90: test2 = bf.sint; break;
    case  91: test3 = bf.sint; break;
    case  92: incomingByte = bf.sint; break;
    case 106: EncPos1Raw = bf.sint; break;
    case 107: EncPos2Raw = bf.sint; break;
    case 108: sensorValue = bf.sint; break;
    case 109: state = bf.sint; break;
    case 110: PCstate = bf.sint; break;
    case 111: EnStateFB = bf.sint; break;
    case 112: spGO = bf.sint; break;
    case 113: IntegOn = bf.sint; break;
    case 114: errcode = bf.sint; break;
    case 115: PWMph1[0] = bf.sint; break;
    case 116: PWMph1[1] = bf.sint; break;
    case 117: PWMph1[2] = bf.sint; break;
    case 118: timeremain = bf.sint; break;
    case 119: TET = bf.sint; break;
    case 120: zenddata = bf.uint; break;
    case 121: Nsend = bf.uint; break;
    case 122: sigid = bf.uint; break;
    case 123: Ndownsample = bf.uint; break;
    case 124: downsample = bf.uint; break;
    case 125: ss_n_aver = bf.uint; break;
    case 126: ContSelect = bf.uint; break;
    case 127: calctime = bf.uint; break;
    case 128: previousTime = bf.uint; break;
    case 129: timePrev = bf.uint; break;
    case 130: curtime = bf.uint; break;
    case 131: curloop = bf.uint; break;
  }
}

void printSignals( unsigned int selected ) {
  char *signalNames[] = { "adc2V", "enc2rad", "enc2m", "penc1", "penc2", "y", "sens", "Im1[0]", "Im1[1]", "Im1[2]", "Im1cal[0]", "Im1cal[1]", "Im1cal[2]", "sensCalVal", "ss_phase", "ss_fstart", "ss_fstep", "ss_fend", "ss_gain", "ss_offset", "ss_f", "ss_tstart", "ss_out", "rpos1", "rpos2", "r", "acc", "vel", "dist", "cursp", "Iout", "velFF", "R", "offsetVelTot", "offsetVel", "Jload", "mechcontout", "PC1out", "PC2out", "Intout1", "Intout2", "muziek_gain", "muziek_gain_V", "distval", "distoff", "emechy", "emechphi", "lastpmechy", "lastpmechphi", "vmechy", "vmechphi", "fBW", "alpha1", "alpha2", "fInt", "fLP", "Kp_PC1", "Kp_PC2", "k1", "k2", "k3", "k4", "Kt", "npole1", "e", "Icontout", "Vout", "fIntCur", "Icontgain", "V_bus", "Im1ab[0]", "Im1ab[1]", "Im1dq[0]", "Im1dq[1]", "Vph1ab[0]", "Vph1ab[1]", "Vph1dq[0]", "Vph1dq[1]", "Vph1[0]", "Vph1[1]", "Vph1[2]", "ThetaE1", "ThetaE1sin", "ThetaE1cos", "I_max", "V_max", "PWMph1offs[0]", "PWMph1offs[1]", "PWMph1offs[2]", "test1", "test2", "test3", "incomingByte", "outLed", "outPhA1", "outPhB1", "outPhC1", "inA1", "inB1", "inZ1", "inA2", "inB2", "inZ2", "inIA1", "inIB1", "inIC1", "EncPos1Raw", "EncPos2Raw", "sensorValue", "state", "PCstate", "EnStateFB", "spGO", "IntegOn", "errcode", "PWMph1[0]", "PWMph1[1]", "PWMph1[2]", "timeremain", "TET", "zenddata", "Nsend", "sigid", "Ndownsample", "downsample", "ss_n_aver", "ContSelect", "calctime", "previousTime", "timePrev", "curtime", "curloop",  };
  char *signalTypes[] = { "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "i", "I", "I", "I", "I", "I", "I", "I", "I", "I", "I", "I", "I", "I",  };
  char *signalSizes[] = { "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "3;0", "3;0", "3;0", "3;0", "3;0", "3;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "2;0", "2;0", "2;0", "2;0", "2;0", "2;0", "2;0", "2;0", "3;0", "3;0", "3;0", "0;0", "0;0", "0;0", "0;0", "0;0", "3;0", "3;0", "3;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "3;0", "3;0", "3;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0", "0;0",  };
  int imax = 10;
  switch(selected){
    case 0: imax = 132; break;
  }
  for ( int i = 0; i < imax; i++) {
    Serial.println( signalNames[i] );
    Serial.println( signalTypes[i] );
    Serial.println( signalSizes[i] );
  }
  Nsend = 0;
  getsignalnames = 1;
}
