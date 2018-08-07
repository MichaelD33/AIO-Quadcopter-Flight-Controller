#pragma once
#ifndef CONFIG_h
#define CONFIG_h
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

/* Define Parameters */
   
//  TYPE OF AIRCRAFT
    #define QUADX
    //#define QUADP not implemented
   
//  MIN/MAX REMOTE GIMBAL/SWITCH OUTPUTS
    #define MINTHROTTLE 172 //minmum throttle output
    #define MAXTHROTTLE 1811 //maximum throttle output

    #define RX_RATES 180 // Maximum rotation speed: 180 degrees per second (speed dictated by RX -> monitored by IMU)
   
//  SPEED CONTROLLER VALUES
    #define ESC_MAX 255
    #define ESC_MIN 0

   
//  INERTIAL MEASUREMENT UNIT
//  #define MPU6050_69
    #define MPU6050_68
    //#define MPU6000
    //#define MPU6500
    //#define MPU9250

//  I2C COMMUNICATION SPEED
//    #define I2C_SPEED      

//  SENSITIVITY
    #define ACCEL_SENS 16384
    #define GYRO_SENS 65.6
    
    #define ACC_PART (1.0 - GYRO_PART)
    #define GYRO_PART 0.975

//  IMU OFFSET
/*    VERSION 2     
    #define ACCEL_X_OFFSET (1630.0)
    #define ACCEL_Y_OFFSET (-1359.0)
    #define ACCEL_Z_OFFSET (869.0)

    #define GYRO_X_OFFSET (16.0)
    #define GYRO_Y_OFFSET (26.0)
    #define GYRO_Z_OFFSET (40.0)
*/

/*    VERSION 3       */
    #define ACCEL_X_OFFSET (-1822.0)
    #define ACCEL_Y_OFFSET (-2715.0)
    #define ACCEL_Z_OFFSET (853.0)

    #define GYRO_X_OFFSET (32.0)
    #define GYRO_Y_OFFSET (-86.0)
    #define GYRO_Z_OFFSET (-12.0)

//  STABILIZATION
    #define ACRO
    //#define AIR
    //#define HORIZON
    //#define ANGLE

//  PID CONFIGURATION
    #define MAX_INTEGRAL 230
    
//  SWITCH OUTPUTS
    #define ARM 1
    #define MODE 2
    #define BEEP 3
    #define FAILSAFE 4


void writeMotor(int, float);
int armingState();
int lastArmingState();

#endif
