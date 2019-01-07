#ifndef PID_h
#define PID_h

#include "imu.h"
#include "RX.h"

/*  DEFINE PROPORTIONAL CONSTANTS */
#define KpX 0.025
#define KpY 0.025
#define KpZ 0.015

#ifndef LOOP_SAMPLING

  #define KiX 0.0000005  //0000005
  #define KiY 0.0000005  //0000005
  #define KiZ 0.0  //0000005

  #define KdX 0  //10
  #define KdY 0  //10
  #define KdZ 0  //7

#else

/*  DEFINE INTEGRAL AND DERIVATIVE CONSTANTS ACCORDING TO PID SAMPLE TIME */    
  #define KiX 0.0 * PID_SAMPLETIME  //0000005
  #define KiY 0.0 * PID_SAMPLETIME  //0000005
  #define KiZ 0.0 * PID_SAMPLETIME  //0000005

  #define KdX 0 / PID_SAMPLETIME  //10
  #define KdY 0 / PID_SAMPLETIME  //10
  #define KdZ 0 / PID_SAMPLETIME  //7
  
#endif


int_pwmOut motorPwmOut();

void initPids();
void resetPids();
void computePids();

#endif
