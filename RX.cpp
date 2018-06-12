#include <Arduino.h>
#include <Streaming.h>
#include "RX.h"

float throttleRx;
int rollRx, pitchRx, yawRx;
int swA, swB, swC, swD, failsafe;
float varRateX = 0.0333;
float varRateY = 0.0333;
float varRateZ = 0.025;

float rateRoll;
float ratePitch;
float rateYaw;
float throttleRate;
// y = ((varRate*x)^3)/180^varRate

SBUS sBus;

void initSbus(){
   sBus.begin();
}

void readRx(){
    sBus.FeedLine();
    if (sBus.toChannels == 1){
      //sBus.UpdateServos();
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
      throttleRx = mapFloat(throttleRx, 172, 1811, 0, 230);
      rollRx = map(rollRx, 172, 1811, -180, 180);
      pitchRx = map(pitchRx, 172, 1811, -180, 180);
      yawRx = map(yawRx, 172, 1811, -180, 180);

      throttleRate = pow(throttleRx, 2.2);
      
      rateRoll = pow(varRateX*rollRx, 3) / pow(180, varRateX);
      ratePitch = pow(varRateY*pitchRx, 3) / pow(180, varRateY);
      rateYaw = pow(varRateZ*yawRx, 3) / pow(180, varRateZ);

 
    //map switches to appropriate values
      swA = map(swA, 172, 1811, 0, 2);
      swB = map(swB, 172, 1811, 0, 2);
      swC = map(swC, 172, 1811, 0, 2);
      swD = map(swD, 172, 1811, 0, 2);

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
  return rateRoll;
}

float chPitch() {
  return ratePitch;
}

float chYaw() {
  return rateYaw;
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


