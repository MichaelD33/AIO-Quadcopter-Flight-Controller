#pragma once
#ifndef PID_h
#define PID_h

#include <Arduino.h>
#include "imu.h"
#include "RX.h"

#define PID_SAMPLETIME 0

#define KpX 0.07 //1.4 //3.6 //6.4//2.2
#define KpY 0.05
#define KpZ 0.07

#define KiX 0.0000
#define KiY 0.0000
#define KiZ 0.0000

#define KdX 0
#define KdY 0
#define KdZ 0

float_pwmOut motorPwmOut();

void initPids();
void resetPids();
void computePids();

#endif
