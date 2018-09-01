#ifndef CONFIG_h
#define CONFIG_h
#include "stdint.h"

/* Define Parameters */

//  #define PRINT_SERIALDATA

/* ----------------------------------------------FLIGHT CONTROLLER BOARD CONFIG------------------------------------------------ */
 
//  #define AIO_v1
//    #define AIO_v2
    #define AIO_v3
//  #define 328
//  #define ATMEGA32u4

/* --------------------------------------------------AIRCRAFT CONFIGURATION----------------------------------------------------------- */

//  AIRCRAFT TYPE CONFIGURATION
    #define QUADX                      //  TBD
//  #define QUADP                      //  TBD
//  #define HEX_P                      //  TBD
//  #define TRI                        //  TBD

//  STABILIZATION MODE
//    #define ACRO                      
//  #define AIR                      //  TBD
    #define HORIZON                
//  #define ANGLE                    //  TBD

/* --------------------------------------------------MOTOR OUTPUT CONFIGURATION----------------------------------------------------------- */

//  SPEED CONTROLLER OUTPUT (brushless implementation still in development)
    #define BRUSHED
//  #define BRUSHLESS
    #define ESC_TOLERANCE 0.9   // THROTTLE MAX = (ESC_MAX * ESC_TOLERANCE)
    #define ESC_MAX 255         // 255 for BRUSHED MOTORS -- 2000 for BRUSHLESS SPEED CONTROLLERS
    #define ESC_MIN 0           // 0 for BRUSHED MOTORS -- 1000 for BRUSHLESS SPEED CONTROLLERS

/* -------------------------------------INERTIAL MEASURMENT UNIT CONFIGURATION-------------------------------------------------------- */

//  IMU TYPE CONFIGURATION
//  #define MPU6050_69
//  #define MPU6050_68
//  #define MPU6000                       //  TBD
//  #define MPU6500_SPI                   //  WIP
//  #define MPU9250_SPI                   //  WIP
//  #define MPU6500_I2C                   //  TBD
//  #define MPU9250_I2C                   //  TBD

//  IMU COMMUNICATION SETTINGS
//   #define I2C_400
//   #define I2C_STANDARD
//   #define SPI  

/*  IMU OFFSET CONFIGURATION
    #define ACCEL_X_OFFSET (0.0)
    #define ACCEL_Y_OFFSET (0.0)
    #define ACCEL_Z_OFFSET (0.0)

    #define GYRO_X_OFFSET (0.0)
    #define GYRO_Y_OFFSET (0.0)
    #define GYRO_Z_OFFSET (0.0)
*/

/* ---------------------------------------------------SENSITIVITY------------------------------------------------------------- */

     #define ACC_SENSITIVITY_2G
//   #define ACC_SENSITIVITY_4G
//   #define ACC_SENSITIVITY_8G
//   #define ACC_SENSITIVITY_16G

//    #define GYRO_SENSITIVITY_250
      #define GYRO_SENSITIVITY_500
//    #define GYRO_SENSITIVITY_1000
//    #define GYRO_SENSITIVITY_2000
    
    #define ACC_PART (1.0 - GYRO_PART)
    #define GYRO_PART 0.975

    #define FILTER_COMPARISONS 11 //number of sample comparisons for median filter

/* ----------------------------------------AIO PROTOTYPE FLIGHT CONTROLLER CONFIGURATION----------------------------------------------------- */

#ifdef AIO_v2
/*    VERSION 2     */
    #define ACCEL_X_OFFSET (1630.0)
    #define ACCEL_Y_OFFSET (-1359.0)
    #define ACCEL_Z_OFFSET (869.0)

    #define GYRO_X_OFFSET (16.0)
    #define GYRO_Y_OFFSET (26.0)
    #define GYRO_Z_OFFSET (40.0)

    #define DIGITAL_LOW_PASS_FILTER //comment this line out to deactivate the MPU6050 digital low pass filter 
    #define DLPF_BANDWIDTH 3  // filtration bandwith configuration (coming soon)
    
    #define MPU6050_69
       
#endif


#ifdef AIO_v3
/*    VERSION 3       */
    #define ACCEL_X_OFFSET (-1822.0)
    #define ACCEL_Y_OFFSET (-2715.0)
    #define ACCEL_Z_OFFSET (853.0)

    #define GYRO_X_OFFSET (32.0)
    #define GYRO_Y_OFFSET (-86.0)
    #define GYRO_Z_OFFSET (-12.0)

    #define DIGITAL_LOW_PASS_FILTER //comment this line out to deactivate the MPU6050 digital low pass filter 
    #define DLPF_BANDWIDTH 3  // filtration bandwith configuration (coming soon)

    #define MPU6050_68
  
#endif

/* ---------------------------------------------REMOTE CONTROL CONFIGURATION------------------------------------------------------------ */

//  TRANSMITTER GIMBAL/SWITCH OUTPUT VALUES
    #define MINTHROTTLE 172 //minmum throttle output
    #define MAXTHROTTLE 1811 //maximum throttle output

//  SWITCH OUTPUTS
    #define ARM 1                       //  TBD
    #define MODE 2                      //  TBD
    #define BEEP 3                      //  TBD
    #define FAILSAFE 4                  //  TBD

//  SET QUADCOPTER ROATATIONAL RATE
    #define RC_RATES 180 // Maximum rotation speed: 180 degrees per second (speed dictated by RX -> monitored by IMU)

//  CONVERTS GIMBAL OUTPUT TO EXPONENTIAL FUNCTION - increase stick sensitivity the further you move from the 
//  #define SUPER_RATE = 0.0333; //applies cubic function to remote controller output values
//  #define SUPER_RATE = 0.025;

/* ---------------------------------------------PID CONTROLLER CONFIGURATION------------------------------------------------------------ */
  
//  integral clamping to avoid writing values outside the range of pwm output
    #define MAX_INTEGRAL 230


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
