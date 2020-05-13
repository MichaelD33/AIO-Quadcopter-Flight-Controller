#pragma once
#ifndef IMU_h_
#define IMU_h_

#include "config.h"
#include "MedianFilter.h"

void initIMU();
void readIMU();

axis_float_t imu_rates();
axis_float_t imu_angles();

#endif
