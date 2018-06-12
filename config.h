#pragma once
#include <Arduino.h>
#include "stdint.h"


typedef struct {
  int16_t x, y, z;
} axis_int16_t;

typedef struct {
  int16_t x, y, z;
} axis_int32_t;

typedef struct {
  float x, y, z;
} axis_float_t;

typedef struct {
  float one, two, three, four;
} float_pwmOut;

void writeMotor(int, float);
int armingState();
int lastArmingState();
