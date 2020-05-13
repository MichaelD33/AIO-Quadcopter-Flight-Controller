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

#include "src/I2Cdev.h"
#include "src/MPU6050_6Axis_MotionApps20.h"
#include "src/helper_3dmath.h"

MPU6050 mpu;

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container

VectorFloat gravity;    // [x, y, z]            gravity vector
// float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

int rateArray[3];

axis_float_t angle;
axis_float_t rate;


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


void initIMU(){ 
   
   Wire.begin();

   #ifdef I2C_FASTMODE
     Wire.setClock(400000); // 400kHz I2C clock. Comment this line if microcontroller does not support 400kHz
   #endif

   #ifdef PRINT_SERIALDATA
    Serial.println(F("Initializing I2C devices..."));
   #endif
   
   mpu.initialize();
   pinMode(INTERRUPT_PIN, INPUT);
   
   #ifdef PRINT_SERIALDATA
     // verify connection
     Serial.println(F("Testing device connections..."));
     Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  
     // wait for ready
     Serial.println(F("\nSend any character to begin DMP programming and demo: "));
     while (Serial.available() && Serial.read()); // empty buffer
     while (!Serial.available());                 // wait for data
     while (Serial.available() && Serial.read()); // empty buffer again
  
     // configure the DMP
     Serial.println(F("Initializing DMP..."));
   #else

     delay(1000);
     
   #endif
    
   devStatus = mpu.dmpInitialize();

   // supply your own gyro offsets here, scaled for min sensitivity
   mpu.setXGyroOffset(GYRO_X_OFFSET);
   mpu.setYGyroOffset(GYRO_Y_OFFSET);
   mpu.setZGyroOffset(GYRO_Z_OFFSET);
   mpu.setZAccelOffset(ACCEL_Z_OFFSET);

   if (devStatus == 0) {
       // turn on the DMP, now that it's ready
       #ifdef PRINT_SERIALDATA
        Serial.println(F("Enabling DMP..."));
       #endif
       
       mpu.setDMPEnabled(true);

       // enable Arduino interrupt detection
       #ifdef PRINT_SERIALDATA
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
       #endif
       
       attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
       mpuIntStatus = mpu.getIntStatus();

       // set our DMP Ready flag so the main loop() function knows it's okay to use it

       #ifdef PRINT_SERIALDATA
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
       #endif
       
       dmpReady = true;

        // get expected DMP packet size for later comparison
       packetSize = mpu.dmpGetFIFOPacketSize();
   } else {
       // ERROR!
       // 1 = initial memory load failed
       // 2 = DMP configuration updates failed
       // (if it's going to break, usually the code will be 1)
       #ifdef PRINT_SERIALDATA
         Serial.print(F("DMP Initialization failed (code "));
         Serial.print(devStatus);
         Serial.println(F(")"));
       #endif
   }

}

void readIMU(){   

  if (!dmpReady) return;

      // wait for MPU interrupt or extra packet(s) available
      while (!mpuInterrupt && fifoCount < packetSize) {
          
          /*

          other program behavior stuff here...
          
          test in between other stuff to see if mpuInterrupt is true, and if so, "break;" from the loop to process MPU data           

           */

      }
  
      // reset interrupt flag and get INT_STATUS byte
      mpuInterrupt = false;
      mpuIntStatus = mpu.getIntStatus();
  
      // get current FIFO count
      fifoCount = mpu.getFIFOCount();
  
      // check for overflow (this should never happen unless our code is too inefficient)
      if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
          // reset so we can continue cleanly
          mpu.resetFIFO();

          #ifdef PRINT_SERIALDATA
            Serial.println(F("FIFO overflow!"));
          #endif
  
      // otherwise, check for DMP data ready interrupt (this should happen frequently)
      } else if (mpuIntStatus & 0x02) {
          // wait for correct available data length, should be a VERY short wait
          while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  
          // read a packet from FIFO
          mpu.getFIFOBytes(fifoBuffer, packetSize);
          
          // track FIFO count here in case there is > 1 packet available
          // (this lets us immediately read more without waiting for an interrupt)
          fifoCount -= packetSize;

          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

          angle.x = 0 - (ypr[1] * 180/M_PI);
          angle.y = 0 - (ypr[2] * 180/M_PI);
          angle.z = 0 - (ypr[0] * 180/M_PI);



          mpu.dmpGetGyro(rateArray, fifoBuffer);

          rate.y = rateArray[0];
          rate.x = rateArray[1];
          rate.z = rateArray[2];
          
          #ifdef PRINT_SERIALDATA
/*          
            if(chAux2() == 2){ 
              Serial.print("Roll:");
              Serial.print(angle.x);
              Serial.print(", Pitch:");
              Serial.print(angle.y);
              Serial.print(", Yaw:");
              Serial.print(angle.z);

              Serial.print(", \t RX Roll: ");
              Serial.print(chRoll());
              Serial.print(", RX Pitch: ");
              Serial.print(chPitch());
              Serial.print(", RX Yaw: ");
              Serial.print(chYaw());
            }
*/

            if(chAux2() == 2){ 
//              Serial.print(angle.x);
//              Serial.print(", ");
//              Serial.print(angle.y);
//              Serial.print(", ");
//              Serial.print(angle.z);
              Serial.print(rate.x);
              Serial.print(", ");
              Serial.print(rate.y);
              Serial.print(", ");
              Serial.print(rate.z);
            }
          #endif

      }

}

axis_float_t imu_angles() {
  return angle;
}

axis_float_t imu_rates() {
  return rate;
}
