#pragma once
#ifndef RX_h
#define RX_h

#include <Arduino.h>
#include "SBUS.h"
//SBUS sBus;

void initSbus();
void readRx();

float chThrottle();
float chRoll();
float chPitch();
float chYaw();
int chAux1();
int chAux2();
int failsafeState();
float mapFloat(float, float, float, float, float);

#endif
