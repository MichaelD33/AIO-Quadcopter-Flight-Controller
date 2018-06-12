#pragma once
#ifndef IMU_h_
#define IMU_h_

#include <Arduino.h>
#include "config.h"

#define ACCEL_X_OFFSET (1630.0)
#define ACCEL_Y_OFFSET (-1359.0)
#define ACCEL_Z_OFFSET (869.0)

#define GYRO_X_OFFSET (16.0)
#define GYRO_Y_OFFSET (26.0)
#define GYRO_Z_OFFSET (40.0)

#define ACCEL_SENS 16384
#define GYRO_SENS 65.6

#define GYRO_PART (0.985)
#define ACC_PART (1.0 - GYRO_PART)

#define MPU_addr 0x69

axis_float_t imu_rates();
axis_float_t imu_angles();
axis_float_t imu_gyro_angles();
axis_float_t imu_accel_filtered();
axis_float_t imu_accelG();
float accel_roll();
float accel_pitch();
float benchmark();
void initIMU();
void readIMU();
void processGyro();
void processAcc();
//void imuBenchmark();
void imuCombine();


#endif

