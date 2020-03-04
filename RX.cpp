#include <Arduino.h>
#include "config.h"
#include "SBUS.h"
#include "RX.h"

int throttleRx, rollRx, pitchRx, yawRx, swA, swB, swC, swD, failsafe;

// y = ((varRate*x)^3)/180^varRate

SBUS sBus;

void initSbus(){
   sBus.begin();
}

void readRx(){
    sBus.FeedLine();
    if (sBus.toChannels == 1){
      sBus.UpdateChannels();
      sBus.toChannels = 0; 

     //assign receieved remote controller values to variables
      throttleRx = sBus.channels[0];
      rollRx = sBus.channels[1];
      pitchRx = sBus.channels[2];
      yawRx = sBus.channels[3];

      swA = sBus.channels[4];
      swB = sBus.channels[5];
      swC = sBus.channels[6];
      swD = sBus.channels[7];
      
      failsafe = sBus.failsafe_status;

//  Map gimbals values to desired outputs

      throttleRx = map(throttleRx, MINTHROTTLE, MAXTHROTTLE, ESC_MIN, (ESC_MAX * ESC_TOLERANCE));
      rollRx = map(rollRx, MINTHROTTLE, MAXTHROTTLE, (RC_RATES*-1), RC_RATES);
      pitchRx = map(pitchRx, MINTHROTTLE, MAXTHROTTLE, (RC_RATES*-1), RC_RATES);
      yawRx = map(yawRx, MINTHROTTLE, MAXTHROTTLE, (RC_RATES*-1), RC_RATES);
     
//    map switches to appropriate values
      swA = map(swA, MINTHROTTLE, MAXTHROTTLE, 0, 2);
      swB = map(swB, MINTHROTTLE, MAXTHROTTLE, 0, 2);
      // swC = map(swC, MINTHROTTLE, MAXTHROTTLE, 0, 2);
      // swD = map(swD, MINTHROTTLE, MAXTHROTTLE, 0, 2);
      

    }
   
    if(swB == 0){
      Serial.print(rollRx);
      Serial.print(",");
      Serial.print(pitchRx);
      Serial.print(",");
      Serial.print(yawRx);
    }
    
}


float chThrottle(){
  return throttleRx;
}

int chRoll() {
  return rollRx;
}

int chPitch() {
  return pitchRx;
}

int chYaw() {
  return yawRx;
}

int chAux1() {
  return swA;
}
int chAux2() {
  return swB;
}
int failsafeState(){
  return failsafe;
}
