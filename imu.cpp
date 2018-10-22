#include <Math.h>
#include <Wire.h>
#include "imu.h"

   //B00000000 - 0
   //B00001000 - 1
   //B00010000 - 2
   //B00011000 - 3


static axis_float_t angle; // angle calculated using accelerometer
static axis_float_t gyroAngles;
static axis_float_t gyroRates;

float roll, pitch;
float AcXRaw,AcYRaw,AcZRaw,TmpRaw,GyXRaw,GyYRaw,GyZRaw;

float delta_t;
long previousTime = 0;
long currentTime = 0;

median_filter_t accel_x_filter = median_filter_new(FILTER_COMPARISONS,0); //declare median filter for x axis 
median_filter_t accel_y_filter = median_filter_new(FILTER_COMPARISONS,0); //declare median filter for y axis
median_filter_t accel_z_filter = median_filter_new(FILTER_COMPARISONS,0); //declare median filter for z axis

 void initIMU(){ 
   Wire.begin();
   Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
   Wire.beginTransmission(MPU_ADDR);
       Wire.write(0x6B);  // PWR_MGMT_1 register
       Wire.write(0);     // set to zero (wakes up the MPU-6050)
   Wire.endTransmission(true);  
   Wire.beginTransmission(MPU_ADDR);
     Wire.write(0x1B);  // Access register 1B - gyroscope config

     #ifdef GYRO_SENSITIVITY_250
       Wire.write(B00000000); // Setting the gyro to full scale +/- 250 deg/sec
       #define GYRO_SENS 131
     #endif
     
     #ifdef GYRO_SENSITIVITY_500
       Wire.write(B00001000); // Setting the gyro to full scale +/- 500 deg/sec
       #define GYRO_SENS 65.6
     #endif

     #ifdef GYRO_SENSITIVITY_1000
       Wire.write(B00010000); // Setting the gyro to full scale +/- 1000 deg/sec
       #define GYRO_SENS 32.8
     #endif

      #ifdef GYRO_SENSITIVITY_2000
         Wire.write(B00011000); // Setting the gyro to full scale +/- 2000 deg/sec
         #define GYRO_SENS 16.4
      #endif
      
   Wire.endTransmission(true);
   Wire.beginTransmission(MPU_ADDR);
   Wire.write(0x1C);  // Access register 1C - accelerometer config

     #ifdef ACC_SENSITIVITY_2G
       Wire.write(B00000000); // Setting the accelerometer to +/- 2g
       #define ACCEL_SENS 16384
     #endif

     #ifdef ACC_SENSITIVITY_4G
       Wire.write(B00001000); // Setting the accelerometer to +/- 2g
       #define ACCEL_SENS 8192
     #endif

     #ifdef ACC_SENSITIVITY_8G
       Wire.write(B00010000); // Setting the accelerometer to +/- 2g
       #define ACCEL_SENS 4096
     #endif

     #ifdef ACC_SENSITIVITY_16G
       Wire.write(B00011000); // Setting the accelerometer to +/- 2g
       #define ACCEL_SENS 2048
     #endif

     
   Wire.endTransmission(true);
   Wire.beginTransmission(MPU_ADDR);
   Wire.write(0x1A);  // digital low pass filter register 0x1A

     #ifdef DIGITAL_LOW_PASS_FILTER
       Wire.write(B00000100); // ENABLING LOW PASS FILTRATION
     #else
       Wire.write(B00000000);
     #endif     
     
   Wire.endTransmission(true);
   Wire.beginTransmission(MPU_ADDR);
   Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
   Wire.endTransmission(false);

 }

void readIMU(){     
   Wire.beginTransmission(MPU_ADDR);
   Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
   Wire.endTransmission(false);
   Wire.requestFrom(MPU_ADDR,14);  // request a total of 14 registers
   AcXRaw=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
   AcYRaw=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
   AcZRaw=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
   TmpRaw=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
   GyYRaw=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
   GyXRaw=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
   GyZRaw=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
   
   processGyro();  

}

void processGyro(){  
  /*
  float new_rate_x = (float)(GyXRaw - GYRO_X_OFFSET) / GYRO_SENS;
  float new_rate_y = (float)(GyYRaw - GYRO_Y_OFFSET) / GYRO_SENS;
  float new_rate_z = (float)(GyZRaw - GYRO_Z_OFFSET) / GYRO_SENS;

  Integration of gyro rates to get the angles for debugging only
  gyroAngles.x += new_rate_x * delta_t;
  gyroAngles.y += new_rate_y * delta_t;
  gyroAngles.z += new_rate_z * delta_t;

  gyroRates.y = gyroRates.y + delta_t / (delta_t) * (new_rate_y - gyroRates.y);
  gyroRates.x = gyroRates.x + delta_t / (delta_t) * (new_rate_x - gyroRates.x);
  gyroRates.z = gyroRates.z + delta_t / (delta_t) * (new_rate_z - gyroRates.z);
  */
  
  gyroRates.y = (float)(GyYRaw - GYRO_Y_OFFSET) / GYRO_SENS;
  gyroRates.x = (float)(GyXRaw - GYRO_X_OFFSET) / GYRO_SENS;
  gyroRates.z = (float)(GyZRaw - GYRO_Z_OFFSET) / GYRO_SENS;

  #if defined(ACRO) || defined(AIR)
    previousTime = currentTime;
    currentTime = micros();
  #else
    processAcc();
  #endif

  
}

void processAcc(){
    //filtering accelerometer noise using a median filter
   
    axis_float_t accel_filtered; // filtered accelerometer raw values
   
    median_filter_in(accel_x_filter, AcXRaw);
    median_filter_in(accel_y_filter, AcYRaw);
    median_filter_in(accel_z_filter, AcZRaw);
    
    //outputting filtered data
    accel_filtered.x = (float) (median_filter_out(accel_x_filter));
    accel_filtered.y = (float) (median_filter_out(accel_y_filter));
    accel_filtered.z = (float) (median_filter_out(accel_z_filter));
/*     
    //converting acceleration to force gravity
    static axis_float_t accelG; // angle calculated using accelerometer
    accelG.x = (accel_filtered.x) / ACCEL_SENS; //
    accelG.y = (accel_filtered.y) / ACCEL_SENS; //
    accelG.z = (accel_filtered.z) / ACCEL_SENS;
    
     //attitude (angle) estimation
//   roll = (atan2(accelG.x, accelG.z)*180)/M_PI; // -180° --> 180°
//   pitch = (atan2(accelG.y, accelG.z)*180)/M_PI; // -180° --> 180°     
*/

//   roll = (atan2(accel_filtered.x, accel_filtered.z)*180)/M_PI; // -180° --> 180°
//   pitch = (atan2(accel_filtered.y, accel_filtered.z)*180)/M_PI; // -180° --> 180°

    roll = (atan2(accel_filtered.x, sqrt(sq(accel_filtered.y)+sq(accel_filtered.z)))*180)/M_PI; 
    pitch = (atan2(accel_filtered.y, sqrt(sq(accel_filtered.x)+sq(accel_filtered.z)))*180)/M_PI; 

     imuCombine();
     
  }


void imuCombine(){
 
   angle.y = GYRO_PART * (angle.y + (gyroRates.x * delta_t)) + (1-GYRO_PART) * pitch; //complementary filter
   angle.x = GYRO_PART * (angle.x + (gyroRates.y * delta_t)) + (1-GYRO_PART) * roll;
   angle.z = (gyroRates.z * delta_t);
//   angle.z = (angle.z + (gyroRates.z * delta_t)); //causes directional lock according to the direction faced by the quadcopter during startup (basically a magnetometer w/out accurate North reference)

/*
   Serial.print("X: ");
   Serial.print(angle.x);
   Serial.print("    Y: ");
   Serial.print(angle.y);
   Serial.print("    Z: ");
   Serial.println(angle.z);
*/   
   delta_t = (float) (currentTime - previousTime) / 1000000;
   previousTime = currentTime;
   currentTime = micros();
}


axis_float_t imu_rates() {
  return gyroRates;
}

axis_float_t imu_angles() {
  return angle;
}

