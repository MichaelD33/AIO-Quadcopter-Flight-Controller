#pragma once
#ifndef RX_h
#define RX_h

void initSbus();
void readRx();

float chThrottle();
int chRoll();
int chPitch();
int chYaw();
int chAux1();
int chAux2();
int chAux3();
int chAux4();
float chAuxPot1();
float chAuxPot2();
int failsafeState();

#endif
