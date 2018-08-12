#include <Arduino.h>
#include "config.h"
#include "SBUS.h"
#include "RX.h"

float throttleRx;
int rollRx, pitchRx, yawRx, swA, swB, swC, swD, failsafe;
float rollRate, pitchRate, yawRate, throttleRate;

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

     //map gimbals to outputs       
      throttleRx = mapFloat(throttleRx, MINTHROTTLE, MAXTHROTTLE, ESC_MIN, (ESC_MAX * ESC_TOLERANCE));
      rollRx = map(rollRx, MINTHROTTLE, MAXTHROTTLE, (RC_RATES*-1), RC_RATES);
      pitchRx = map(pitchRx, MINTHROTTLE, MAXTHROTTLE, (RC_RATES*-1), RC_RATES);
      yawRx = map(yawRx, MINTHROTTLE, MAXTHROTTLE, (RC_RATES*-1), RC_RATES);

     
    //FIX SUPER RATE MAPPING
      throttleRate = pow(throttleRx, 2.2);
//    rollRate = pow(rollRx, );
//    pitchRate = pow(pitchRx, );
//    yawRate = pow(yawRx, );

     
    //map switches to appropriate values
      swA = map(swA, MINTHROTTLE, MAXTHROTTLE, 0, 2);
      swB = map(swB, MINTHROTTLE, MAXTHROTTLE, 0, 2);
      swC = map(swC, MINTHROTTLE, MAXTHROTTLE, 0, 2);
      swD = map(swD, MINTHROTTLE, MAXTHROTTLE, 0, 2);

    }
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float chThrottle(){
  return throttleRx;
}

float chRoll() {
  //return rollRate;
  return rollRx;
}

float chPitch() {
  //return pitchRate;
  return pitchRx;
}

float chYaw() {
  //return yawRate;
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


