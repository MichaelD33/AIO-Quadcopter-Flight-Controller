#pragma once
#ifndef CONFIG_h
#define CONFIG_h
#include "stdint.h"

/* Define Parameters */

/* ——————————————————————————————————————————————————————DEBUGGING————————————————————————————————————————————————————————— */

 #define PRINT_SERIALDATA     // enables the serial monitor and serial debugging via the remote's switches

/* —————————————————————————————————————————————————AIRCRAFT CONFIGURATION——————————————————————————————————————————————————— */

/*  STABILIZATION MODE  */
    #define HORIZON // This is the only supported stabilization mode (at the moment)... Do not change.

/* FLIGHT CONTROLLER BOARD CONFIG */
//  #define AIO_v01   // LEGACY
//  #define AIO_v03   // LEGACY (purple frame) (no +5V)
//  #define AIO_v04   // LEGACY (updated to 041)
    #define AIO_v041  // CURRENT REVISION (+5V enabled)

/* DIY FLIGHT CONTROLLER CONFIG (ADDITIONAL SETUP MAY BE REQUIRED) */
//  #define CUSTOM_FC

/* ———————————————————————————————————————————————————REMOTE CONTROL CONFIGURATION—————————————————————————————————————————————————————— */

//  TRANSMITTER GIMBAL/SWITCH OUTPUT VALUES
    #define MINTHROTTLE 172   // minimum throttle output
    #define MAXTHROTTLE 1811  // maximum throttle output


//  SET QUADCOPTER ROATATIONAL RATE
    #define RC_RATES 90 // Maximum rotation: 90 degrees (in order to prevent the device from flipping over)

/* ———————————————————————————————————————————————PROGRAM CONFIGURATION———————————————————————————————————————————————— */

    #define SAMPLETIME 10000 //define loop sample time at a frequency of 3000µs
    #define SAMPLETIME_S 0.01
    
/* ———————————————————————————————————————————INERTIAL MEASURMENT UNIT CONFIGURATION—————————————————————————————————————————— */

     //  I2C communication settings
     #define I2C_FASTMODE

      // Configuring IMU offsets for the MPU6050

      /*    VERSION 0.4.1    7x20mm Motors   —  White Frame */
      #ifdef AIO_v041
        
          #define ACCEL_X_OFFSET (-4803)
          #define ACCEL_Y_OFFSET (-212)
          #define ACCEL_Z_OFFSET (740)
      
          #define GYRO_X_OFFSET (145)
          #define GYRO_Y_OFFSET (-17)
          #define GYRO_Z_OFFSET (24)
          
      #endif


/* ————————————————————————————————————————————————MOTOR OUTPUT CONFIGURATION———————————————————————————————————————————————— */

// SPEED CONTROLLER CONFIG
    #define ESC_TOLERANCE 0.9        // THROTTLE MAX = (ESC_MAX * ESC_TOLERANCE)
    #define ESC_MAX 255              // 255 for BRUSHED MOTORS
    #define ESC_MIN 0                // 0 for BRUSHED MOTORS


/* ———————————————————————————————————————————————PID CONTROLLER CONFIGURATION———————————————————————————————————————————————— */

    #define MAX_INTEGRAL 230  //  integral clamping to avoid writing values outside the range of pwm output


/* ———————————————————————————————————————————————CUSTOM VARIABLE STRUCTURE CONFIGURATION————————————————————————————————————————————————— */

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
  int16_t one, two, three, four;
} int_pwmOut;

void writeMotor(int, float);
int armingState();
int lastArmingState();

#endif  //end #ifndef
