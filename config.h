#pragma once
#ifndef CONFIG_h
#define CONFIG_h
#include "stdint.h"

/* Define Parameters */

/* ——————————————————————————————————————————————————————DEBUGGING————————————————————————————————————————————————————————— */

  #define PRINT_SERIALDATA     // calls the printSerial() function in loop()
  #define LOOP_SAMPLING      // enables loop sampling for fixed PID and IMU sampling rates as well as loop time profiling

/* —————————————————————————————————————————————————AIRCRAFT CONFIGURATION——————————————————————————————————————————————————— */

/*  STABILIZATION MODE  */
    #define HORIZON // This is the only supported stabilization mode (at the moment)... Do not change.

/* FLIGHT CONTROLLER BOARD CONFIG */
//  #define AIO_v01   // LEGACY
//  #define AIO_v03   // LEGACY (purple frame) (no +5V)
//  #define AIO_v04   // LEGACY (updated to 041)
    #define AIO_v041  // CURRENT REVISION (+5V enabled)

/* DIY FLIGHT CONTROLLER CONFIG (ADDITIONAL SETUP MAY BE REQUIRED) */
//  #define ATMEGA32u4

/* ———————————————————————————————————————————————————REMOTE CONTROL CONFIGURATION—————————————————————————————————————————————————————— */

//  TRANSMITTER GIMBAL/SWITCH OUTPUT VALUES
    #define MINTHROTTLE 172 //  minimum throttle output
    #define MAXTHROTTLE 1811 // maximum throttle output


//  SET QUADCOPTER ROATATIONAL RATE
    #define RC_RATES 180 // Maximum rotation speed: 180 degrees per second

/* ———————————————————————————————————————————————PROGRAM CONFIGURATION———————————————————————————————————————————————— */

    #ifdef LOOP_SAMPLING
      #define SAMPLETIME 10000 //define loop sample time at a frequency of 3000µs
      #define SAMPLETIME_S 0.01
    #endif
    

/* ———————————————————————————————————————————INERTIAL MEASURMENT UNIT CONFIGURATION—————————————————————————————————————————— */

     //  I2C communication settings
     #define I2C_FASTMODE

      // Configuring IMU offsets for the MPU6050
      #ifdef AIO_v041
        /*    VERSION 0.4.1    7x20mm Motors   — Black PCB in Red Frame      */
      
          #define ACCEL_X_OFFSET (933)
          #define ACCEL_Y_OFFSET (-776)
          #define ACCEL_Z_OFFSET (1160)
      
          #define GYRO_X_OFFSET (108)
          #define GYRO_Y_OFFSET (13)
          #define GYRO_Z_OFFSET (-4)
          
      #endif

/*
 *   These parameters are not used in the flight control firmwqre
 * 
     #define ACC_SENSITIVITY_2G
//     #define ACC_SENSITIVITY_4G
//     #define ACC_SENSITIVITY_8G
//     #define ACC_SENSITIVITY_16G

//      #define GYRO_SENSITIVITY_250
        #ifdef GYRO_SENSITIVITY_250
          #define GYRO_SENS 131
        #endif
        
      #define GYRO_SENSITIVITY_500
        #ifdef GYRO_SENSITIVITY_500
          #define GYRO_SENS 65.6
        #endif
        
//      #define GYRO_SENSITIVITY_1000
        #ifdef GYRO_SENSITIVITY_1000
          #define GYRO_SENS 32.8
        #endif
        
//      #define GYRO_SENSITIVITY_2000
        #ifdef GYRO_SENSITIVITY_2000
         #define GYRO_SENS 16.4
       #endif
*/


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
  uint8_t one, two, three, four;
} int_pwmOut;

void writeMotor(int, float);
int armingState();
int lastArmingState();

#endif  //end #ifndef
