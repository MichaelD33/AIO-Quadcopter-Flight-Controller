#pragma once
#ifndef PID_h
#define PID_h

#include <Arduino.h>
#include "imu.h"
#include "RX.h"

#define PID_SAMPLETIME 5500

#define KpX 0.07 //1.4 //3.6 //6.4//2.2
#define KpY 0.07
#define KpZ 0.07

#define KiX 0.0000000 * PID_SAMPLETIME //0.00008 * PID_SAMPLETIME
#define KiY 0.0000000 * PID_SAMPLETIME
#define KiZ 0.0000000 * PID_SAMPLETIME

#define KdX 0 / PID_SAMPLETIME
#define KdY 0 / PID_SAMPLETIME
#define KdZ 0 / PID_SAMPLETIME

float_pwmOut motorPwmOut();

void initPids();
void resetPids();
//void computePids(int8_t);
void computePids();

#endif
