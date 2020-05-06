#include <Arduino.h>
#include "config.h"
#include "SBUS.h"
#include "RX.h"

int throttleRx, rollRx, pitchRx, yawRx, swA, swB, failsafe;
float swC, swD, swE, swF;

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
      swE = sBus.channels[8];
      swF = sBus.channels[9];

      
      failsafe = sBus.failsafe_status;

//  Map gimbals values to desired outputs

      throttleRx = map(throttleRx, MINTHROTTLE, MAXTHROTTLE, ESC_MIN, (ESC_MAX * ESC_TOLERANCE));
      rollRx = map(rollRx, MINTHROTTLE, MAXTHROTTLE, (RC_RATES*-1), RC_RATES);
      pitchRx = map(pitchRx, MINTHROTTLE, MAXTHROTTLE, (RC_RATES*-1), RC_RATES);
      yawRx = map(yawRx, MINTHROTTLE, MAXTHROTTLE, (RC_RATES*-1), RC_RATES);
     
//    map switches to appropriate values
      swA = map(swA, MINTHROTTLE, MAXTHROTTLE, 0, 2);
      swB = map(swB, MINTHROTTLE, MAXTHROTTLE, 0, 2);
      swC = map(swC, MINTHROTTLE, MAXTHROTTLE, 0, 1000);
      swD = map(swD, MINTHROTTLE, MAXTHROTTLE, 0, 1000);

      swC = swC / 1000;
      swD = swD / 5000;

      swE = map(swE, MINTHROTTLE, MAXTHROTTLE, 0, 1);
      swF = map(swF, MINTHROTTLE, MAXTHROTTLE, 0, 1);
      

    }
   
    if(swB == 0){
      Serial.print("RX Roll: ");
      Serial.print(rollRx);
      Serial.print(", RX Pitch: ");
      Serial.print(pitchRx);
      Serial.print(", RX Yaw: ");
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

float chAuxPot1(){
 return swC; 
}

float chAuxPot2(){
 return swD; 
}

int chAux3(){
  return swE;
}

int chAux4(){
  return swF;
}

int failsafeState(){
  return failsafe;
}
