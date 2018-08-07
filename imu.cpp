#include <Arduino.h>
#include <Math.h>
#include <Wire.h>
#include "imu.h"
#include "MedianFilter.h"


static axis_float_t accel_filtered; // filtered accelerometer raw values
static axis_float_t angle; // angle calculated using accelerometer
static axis_float_t gyroAngles;
static axis_float_t gyroRates;
static median_filter_t accel_x_filter = median_filter_new(19,0); //declare median filter for x axis
static median_filter_t accel_y_filter = median_filter_new(19,0); //declare median filter for y axis
static median_filter_t accel_z_filter = median_filter_new(19,0); //declare median filter for z axis

float AcXRaw,AcYRaw,AcZRaw,TmpRaw,GyXRaw,GyYRaw,GyZRaw;

float pitch,roll,yaw;
float delta_t;
long previousTime = 0;
long currentTime = 0;

 void initIMU(){ 
   Wire.begin();
   Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
   Wire.beginTransmission(MPU_addr);
       Wire.write(0x6B);  // PWR_MGMT_1 register
       Wire.write(0);     // set to zero (wakes up the MPU-6050)
   Wire.endTransmission(true);  
   Wire.beginTransmission(MPU_addr);
     Wire.write(0x1B);  // Access register 1B - gyroscope config
     Wire.write(B00001000); // Setting the gyro to full scale +/- 500 deg/sec
   Wire.endTransmission(true);
   Wire.beginTransmission(MPU_addr);
     Wire.write(0x1C);  //Access register 1C - accelerometer config
     Wire.write(B00000000); //Setting the accelerometer to +/- 2g
   Wire.endTransmission(true);
   //B00000000 - 0
   //B00001000 - 1
   //B00010000 - 2
   //B00011000 - 3
   Wire.beginTransmission(MPU_addr);
   Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
   Wire.endTransmission(false);
 }

void readIMU(){

   delta_t = (float) (currentTime - previousTime) / 1000000;
   previousTime = currentTime;
  
   Wire.beginTransmission(MPU_addr);
   Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
   Wire.endTransmission(false);
   Wire.requestFrom(MPU_addr,14);  // request a total of 14 registers
   AcXRaw=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
   AcYRaw=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
   AcZRaw=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
   TmpRaw=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
   GyYRaw=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
   GyXRaw=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
   GyZRaw=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

   processGyro();
   processAcc();
   imuCombine();

}

void processGyro(){
  float new_rate_x = (float)(GyXRaw - GYRO_X_OFFSET) / GYRO_SENS;
  float new_rate_y = (float)(GyYRaw - GYRO_Y_OFFSET) / GYRO_SENS;
  float new_rate_z = (float)(GyZRaw - GYRO_Z_OFFSET) / GYRO_SENS;

  // Integration of gyro rates to get the angles for debugging only
  gyroAngles.x += new_rate_x * delta_t;
  gyroAngles.y += new_rate_y * delta_t;
  gyroAngles.z += new_rate_z * delta_t;

  uint8_t f_cut = 80; // Hz                                     
  float rc = 1.0f / (2.0f * (float)M_PI * f_cut);

  gyroRates.y = gyroRates.y + delta_t / (rc + delta_t) * (new_rate_y - gyroRates.y);
  gyroRates.x = gyroRates.x + delta_t / (rc + delta_t) * (new_rate_x - gyroRates.x);
  gyroRates.z = gyroRates.z + delta_t / (rc + delta_t) * (new_rate_z - gyroRates.z);
}

void processAcc(){
    //filtering accelerometer noise using a median filter
   
    median_filter_in(accel_x_filter, AcXRaw);
    median_filter_in(accel_y_filter, AcYRaw);
    median_filter_in(accel_z_filter, AcZRaw);

    //outputting filtered data
    accel_filtered.x = (float) (median_filter_out(accel_x_filter));
    accel_filtered.y = (float) (median_filter_out(accel_y_filter));
    accel_filtered.z = (float) (median_filter_out(accel_z_filter));

    //converting acceleration to force gravity
    static axis_float_t accelG; // angle calculated using accelerometer
    accelG.x = (accel_filtered.x) / ACCEL_SENS; // 1.33g -> -0.66g
    accelG.y = (accel_filtered.y) / ACCEL_SENS; // 1.05g -> -0.96g
    accelG.z = (accel_filtered.z) / ACCEL_SENS; // 1.09g @ rest:

    //attitude (angle) estimation
     roll = (atan2(accelG.x, accelG.z)*180)/M_PI; // -180° --> 180°
     pitch = (atan2(accelG.y, accelG.z)*180)/M_PI; // -180° --> 180°
     //yaw = (atan(accelG.z / sqrt(sq(accelG.y) + sq(accelG.x))))*(360/(2*M_PI));  
    
  }


void imuCombine(){
  
   angle.y = GYRO_PART * (angle.y + (gyroRates.x * delta_t)) + (1-GYRO_PART) * pitch; //complementary filter
   angle.x = GYRO_PART * (angle.x + (gyroRates.y * delta_t)) + (1-GYRO_PART) * roll;
   angle.z = 1 * (angle.z + (gyroRates.z * delta_t)) + (0) * yaw;
       
   currentTime = micros();
}

axis_float_t imu_rates() {
  return gyroRates;
}
axis_float_t imu_gyro_angles() {
  return gyroAngles;
}
float accel_roll() {
  return roll;
}
float accel_pitch() {
  return pitch;
}
axis_float_t imu_accel_filtered() {
  return accel_filtered;
}
axis_float_t imu_angles() {
  return angle;
}
