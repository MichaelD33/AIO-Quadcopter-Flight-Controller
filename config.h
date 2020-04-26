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
    #define HORIZON
//    #define ACRO                      

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

/* ———————————————————————————————————————————————PID CONTROLLER CONFIGURATION———————————————————————————————————————————————— */

    #ifdef LOOP_SAMPLING
      #define SAMPLETIME 10000 //define loop sample time at a frequency of 3000µs
      #define SAMPLETIME_S 0.01
    #endif
    
    #define MAX_INTEGRAL 230  //  integral clamping to avoid writing values outside the range of pwm output
  
/* ————————————————————————————————————————————————MOTOR OUTPUT CONFIGURATION———————————————————————————————————————————————— */

//  SPEED CONTROLLER TYPE
    #define BRUSHED

// SPEED CONTROLLER CONFIG
    #define ESC_TOLERANCE 0.9        // THROTTLE MAX = (ESC_MAX * ESC_TOLERANCE)
    #define ESC_MAX 255              // 255 for BRUSHED MOTORS
    #define ESC_MIN 0                // 0 for BRUSHED MOTORS

/* ———————————————————————————————————————————INERTIAL MEASURMENT UNIT CONFIGURATION—————————————————————————————————————————— */

//  IMU COMMUNICATION SETTINGS
//   #define I2C_STANDARD     
     #define I2C_FASTMODE

     #define DIGITAL_LOW_PASS_FILTER //comment this line out to deactivate the MPU6050 digital low pass filter 
//     #define DLPF_BANDWIDTH 3  // filtration bandwith configuration (coming soon)


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
    
    #define ACC_PART (1.0 - GYRO_PART)
    #define GYRO_PART 0.985

    #define FILTER_COMPARISONS 15 //number of sample comparisons for median filter



/*    THIS SECTION IS PREDEFINED ACCORDING TO AIO PCB VERSION     */
//    #define MPU6050_69
//    #define MPU6050_68

/* //  IMU OFFSET CONFIGURATION
    #define ACCEL_X_OFFSET (0.0)
    #define ACCEL_Y_OFFSET (0.0)
    #define ACCEL_Z_OFFSET (0.0)

    #define GYRO_X_OFFSET (0.0)
    #define GYRO_Y_OFFSET (0.0)
    #define GYRO_Z_OFFSET (0.0)
*/

/* ————————————————————————————————————————AIO PROTOTYPE FLIGHT CONTROLLER CONFIGURATION—————————————————————————————————————————— */

#ifdef AIO_v03
/*    VERSION 0.3  ***OLD VERSION***   Orange Bottom Mount 3D Printed Frame — v1
    #define ACCEL_X_OFFSET (1630.0)
    #define ACCEL_Y_OFFSET (-1359.0)
    #define ACCEL_Z_OFFSET (869.0)

    #define GYRO_X_OFFSET (16.0)
    #define GYRO_Y_OFFSET (26.0)
    #define GYRO_Z_OFFSET (40.0)
    
    #define MPU6050_69  
*/

/*    VERSION 0.3   Purple 3D Printed Frame v2 && 7x20mm Motors   */
    #define ACCEL_X_OFFSET (2189.0)
    #define ACCEL_Y_OFFSET (2770.0)
    #define ACCEL_Z_OFFSET (1100.0)

    #define GYRO_X_OFFSET (97.0)
    #define GYRO_Y_OFFSET (-56.0)
    #define GYRO_Z_OFFSET (-64.0)
    
    #define MPU6050_68

#endif

#ifdef AIO_v04
/*    VERSION 0.4    7x16mm Motors  — Bare Red PCB Version            */
    #define ACCEL_X_OFFSET (-1822.0)
    #define ACCEL_Y_OFFSET (-2715.0)
    #define ACCEL_Z_OFFSET (853.0)

    #define GYRO_X_OFFSET (32.0)
    #define GYRO_Y_OFFSET (-86.0)
    #define GYRO_Z_OFFSET (-12.0) 

    #define MPU6050_68
    
#endif

#ifdef AIO_v041
    /*    VERSION 0.4.1    7x20mm Motors   — Red Shapeways Frame      
    #define ACCEL_X_OFFSET (1001)
    #define ACCEL_Y_OFFSET (316)
    #define ACCEL_Z_OFFSET (1274)

    #define GYRO_X_OFFSET (42)
    #define GYRO_Y_OFFSET (10)
    #define GYRO_Z_OFFSET (25)

    #define MPU6050_68

  */
  
  /*    VERSION 0.4.1    7x20mm Motors   — Black PCB in Red Frame      */

    #define ACCEL_X_OFFSET (933)
    #define ACCEL_Y_OFFSET (-776)
    #define ACCEL_Z_OFFSET (1160)

    #define GYRO_X_OFFSET (108)
    #define GYRO_Y_OFFSET (13)
    #define GYRO_Z_OFFSET (-4)

    #define MPU6050_68
    
#endif

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
