/*
 * Inertial Measurement Unit Data Acquisition and Processsing
 * 
 * 
 * Reference:
 * 
 * B00000000 - 0
 * B00001000 - 1
 * B00010000 - 2
 * B00011000 - 3
 * 
*/

#include <Arduino.h>
#include <Math.h>
#include <Wire.h>
#include "imu.h"
#include "RX.h"

static axis_float_t angle; // angle calculated using accelerometer
static axis_int16_t gyroRates;

float roll, pitch, yaw, TmpRaw;
int AcXRaw,AcYRaw,AcZRaw,GyXRaw,GyYRaw,GyZRaw;

float delta_t;
unsigned long previousTime = 0;
unsigned long currentTime = 0;

// extern unsigned long imuEndTime;

#ifdef HORIZON
   median_filter_t accel_x_filter = median_filter_new(FILTER_COMPARISONS,0); //declare median filter for x axis 
   median_filter_t accel_y_filter = median_filter_new(FILTER_COMPARISONS,0); //declare median filter for y axis
   median_filter_t accel_z_filter = median_filter_new(FILTER_COMPARISONS,0); //declare median filter for z axis
#endif

 void initIMU(){ 
   
   Wire.begin();

   #ifdef I2C_FASTMODE
     Wire.setClock(400000); // 400kHz I2C clock. Comment this line if microcontroller does not support 400kHz
   #endif
   
   Wire.beginTransmission(MPU_ADDR);
   Wire.write(0x6B);  // PWR_MGMT_1 register
   Wire.write(0);     // set to zero (wakes up the MPU-6050)
   Wire.endTransmission(true);  
   
   Wire.beginTransmission(MPU_ADDR);
   Wire.write(0x1B);  // Access register 1B - gyroscope config
   #ifdef GYRO_SENSITIVITY_250
     Wire.write(B00000000); // Setting the gyro to full scale +/- 250 deg/sec
   #elif defined GYRO_SENSITIVITY_500
     Wire.write(B00001000); // Setting the gyro to full scale +/- 500 deg/sec
   #elif defined GYRO_SENSITIVITY_1000
     Wire.write(B00010000); // Setting the gyro to full scale +/- 1000 deg/sec
   #elif defined GYRO_SENSITIVITY_2000
     Wire.write(B00011000); // Setting the gyro to full scale +/- 2000 deg/sec
   #endif
   Wire.endTransmission(true);
   
   Wire.beginTransmission(MPU_ADDR);
   Wire.write(0x1C);  // Access register 1C - accelerometer config

   #ifdef ACC_SENSITIVITY_2G
     Wire.write(B00000000); // Setting the accelerometer to +/- 2g
     #define ACCEL_SENS 16384
   #elif defined ACC_SENSITIVITY_4G
     Wire.write(B00001000); // Setting the accelerometer to +/- 4g
     #define ACCEL_SENS 8192
   #elif defined ACC_SENSITIVITY_8G
     Wire.write(B00010000); // Setting the accelerometer to +/- 8g
     #define ACCEL_SENS 4096
   #elif defined ACC_SENSITIVITY_16G
     Wire.write(B00011000); // Setting the accelerometer to +/- 16g
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
  
  gyroRates.y = (GyYRaw - GYRO_Y_OFFSET) / GYRO_SENS;
  gyroRates.x = (GyXRaw - GYRO_X_OFFSET) / GYRO_SENS;
  gyroRates.z = (GyZRaw - GYRO_Z_OFFSET) / GYRO_SENS;
    
  #ifdef HORIZON
    processAcc();
    imuCombine();
  #endif

}

void processAcc(){
    //filtering accelerometer noise using a median filter
#ifdef HORIZON
    axis_float_t accel_filtered; // filtered accelerometer raw values
   
    median_filter_in(accel_x_filter, AcXRaw);
    median_filter_in(accel_y_filter, AcYRaw);
    median_filter_in(accel_z_filter, AcZRaw);

    accel_filtered.x = (median_filter_out(accel_x_filter));
    accel_filtered.y = (median_filter_out(accel_y_filter));
    accel_filtered.z = (median_filter_out(accel_z_filter));

    roll = (atan2(accel_filtered.x, sqrt((accel_filtered.y * accel_filtered.y) + (accel_filtered.z * accel_filtered.z))) * 180) / M_PI; 
    pitch = (atan2(accel_filtered.y, sqrt((accel_filtered.x * accel_filtered.x) + (accel_filtered.z * accel_filtered.z))) * 180) / M_PI; 
    //yaw = 
    
//    roll = (atan2(accel_filtered.x, accel_filtered.z)*180)/M_PI; // -180° --> 180°
//    pitch = (atan2(accel_filtered.y, accel_filtered.z)*180)/M_PI; // -180° --> 180°
#endif
}

void imuCombine(){

  #ifdef LOOP_SAMPLING

     angle.y = GYRO_PART * (angle.y + (gyroRates.x * PID_SAMPLETIME_S)) + (1-GYRO_PART) * pitch; //complementary filter
     angle.x = GYRO_PART * (angle.x + (gyroRates.y * PID_SAMPLETIME_S)) + (1-GYRO_PART) * roll;
     //angle.z = angle.z + (gyroRates.z * PID_SAMPLETIME_S);
     angle.z = (gyroRates.z * PID_SAMPLETIME_S);

  #else
     currentTime = micros();
     delta_t = (currentTime - previousTime) / 1000000;
     
     angle.y = GYRO_PART * (angle.y + (gyroRates.x * delta_t)) + (1-GYRO_PART) * pitch; //complementary filter
     angle.x = GYRO_PART * (angle.x + (gyroRates.y * delta_t)) + (1-GYRO_PART) * roll;
     angle.z = (gyroRates.z * delta_t);     
     
     previousTime = currentTime;
  #endif

  #ifdef PRINT_SERIALDATA
    if(chAux2() == 2){
       //Serial.print("X-Angle: ");
       Serial.print(angle.x);
  
       Serial.print(",");
       Serial.print(angle.y);
  
       Serial.print(",");
       Serial.print(angle.z);
    }
    
    if(chAux2() == 0){
       Serial.print(",");
       Serial.print(angle.x);
       Serial.print(",");
       Serial.print(angle.y);
       Serial.print(",");
       Serial.println(angle.z);
    }
  #endif
   
}


axis_int16_t imu_rates() {
  return gyroRates;
}

axis_float_t imu_angles() {
  return angle;
}
