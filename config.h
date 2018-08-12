#ifndef CONFIG_h
#define CONFIG_h
#include "stdint.h"

/* Define Parameters */ //ADD exit() for when parameters not defined?

//  QUADCOPTER FLIGHT CONTROLLER BOARD TYPE
//  #define AIO_v1
//  #define AIO_v2
    #define AIO_v3

//  #define 328
//  #define ATMEGA32u4

//  TYPE OF AIRCRAFT
    #define QUADX                      //  TBD
//  #define QUADP                      //  TBD
   
//  MIN/MAX REMOTE GIMBAL/SWITCH OUTPUTS
    #define MINTHROTTLE 172 //minmum throttle output
    #define MAXTHROTTLE 1811 //maximum throttle output

    #define RC_RATES 180 // Maximum rotation speed: 180 degrees per second (speed dictated by RX -> monitored by IMU)

//  #define SUPER_RATE = 0.0333; //applies cubic function to remote controller output values
//  #define SUPER_RATE = 0.025;

   
//  SPEED CONTROLLER VALUES
    #define ESC_TOLERANCE 0.9
    #define ESC_MAX 255
    #define ESC_MIN 0

//  PID CONFIGURATION
    #define MAX_INTEGRAL 230
   
//  INERTIAL MEASUREMENT UNIT
//  #define MPU6050_69
    #define MPU6050_68
//  #define MPU6000                       //  TBD
//  #define MPU6500                       //  TBD
//  #define MPU9250                       //  TBD

//  IMU COMMUNICATION SETTINGS
//   #define I2C_400
//   #define I2C_STANDARD
//   #define SPI  
    
//  SENSITIVITY
    #define ACCEL_SENS 16384
    #define GYRO_SENS 65.6
    
    #define ACC_PART (1.0 - GYRO_PART)
    #define GYRO_PART 0.975

    #define FILTER_COMPARISONS 19 //number of sample comparisons for median filter
    

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
//  #define AIR                      //  TBD
//  #define HORIZON                
//  #define ANGLE                    //  TBD
    
//  SWITCH OUTPUTS
    #define ARM 1                       //  TBD
    #define MODE 2                      //  TBD
    #define BEEP 3                      //  TBD
    #define FAILSAFE 4                  //  TBD


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

#endif
