/*
 *  AIO_FlightController - An integrated quadcopter flight controller program for the Arduino platform.
 *  Copyright © 2018-2019 Michael Delaney. All rights reserved.
 * 
 *  This device takes data from an inertial measurement unit about its orientation and from an external remote about 
 *  the desired change in position to the system and makes several calculations to adjust to said position by varying 
 *  the speed of its motors according to calculations made by the control loop.
 * 
 *  Active Source Code: https://github.com/MichaelD33/AIO-Quadcopter-Flight-Controller
 *  Design Files: https://github.com/MichaelD33/AIO-Quadcopter-Design
 *  
 *  Simple Quadcopter FC in ~1500 lines of code
 */

#include <Arduino.h>
#include "config.h"
#include "imu.h"
#include "RX.h"
#include "pid.h"

bool armState = false;
bool lastArmState = false;

#if defined(LOOP_SAMPLING) || defined(TIMEPROFILING) || defined(GUI_ENABLED)
  int printStartTime, imuStartTime, rxStartTime, pidStartTime;
  long timestart;
  long loopEnd = 0;
  long cycles = 1;
  int imuLastStart = 0;
  int imuEndTime = 0;
  int pidLastStart = 0;
  int pidEndTime = 0;

#endif

#ifdef AIO_v01
  byte motorOutput[] = {9, 5, 10, 6};  //prototype v0.1 configuration - RETIRED
  #define ATMEGA32u4
#endif

#ifdef AIO_v03
  byte motorOutput[] = {5, 9, 6, 10};   //version 0.3 configuration - GREEN BOARD (purple frame)
//  byte motorOutput[] = {6, 10, 5, 9};   //version 0.3 configuration - GREEN BOARD (purple frame)

  #define ATMEGA32u4
#endif

#ifdef AIO_v04
  byte motorOutput[] = {10, 9, 13, 6}; //version 0.4 configuration - RED PCB (legacy)
  #define ATMEGA32u4
#endif

#ifdef AIO_v041
  byte motorOutput[] = {10, 9, 5, 6};  //version 0.4.1 configuration — Updated RED PCB - Professionally Made RED Frame
  #define ATMEGA32u4
#endif

/*  IF NOT USING THE AIO PCB USE FORMAT AS SHOWN BELOW  (REF. DIAGRAM ON GITHUB FOR MOTOR LOCATIONS) */
//  byte motorOutput[] = {[motor 1], [motor 2], [motor 3], [motor 4]}; 

void setup() {
   
  #ifdef PRINT_SERIALDATA
    Serial.begin(115200);
  #endif
    Serial.begin(115200);
  
  #ifdef ATMEGA32u4
    DDRB = DDRB | B11110000; //sets pins D8, D9, D10, D11 as outputs
    DDRC = DDRC | B11000000; //sets pin D5 as output
    DDRD = DDRD | B10000000; //sets pin D6 as output
    DDRE = DDRE | B01000000; //sets pin D7 as output
  #endif

  delay(2000); //give time for RX to connect to remote
  initSbus();  //connect to the remote reciever (rx.cpp)
  initIMU();   //activate the imu and set gyroscope and accelerometer sensitivity (imu.cpp)
   
}

void loop() {
  
  #if defined(TIMEPROFILING) || defined(LOOP_SAMPLING)
    timestart = micros();       
    
    #ifdef LOOP_SAMPLING
      while((micros() - timestart + (loopEnd - imuLastStart)) < IMU_SAMPLING_FREQUENCY){
        imuStartTime = (micros() - timestart);
      }
    #else
        imuStartTime = (micros() - timestart);
    #endif
    
  #endif
    
// IMU supports up to 8kHz gyro update rate and 1kHz acc update rate --- when DLPF is activated this is diminished significantly (see MPU6050 register mapping datasheet)
   readIMU(); //read the imu and calculate the quadcopters position relative to gravity (imu.cpp)

   #if defined(LOOP_SAMPLING) || defined(TIMEPROFILING)
    imuEndTime = (micros() - timestart);
    Serial.print("IMU Time: ");
    Serial.println(imuEndTime - imuStartTime);

    Serial.print("IMU Sample Rate: ");
    Serial.println(micros() - imuEndTime);
   #endif

   readRx();  //read the remote and convert data to a rotational rate of ±180°/s (rx.cpp)

   if(failsafeState() == 0){
      switch(chAux1()){
        case 0: //if the arm switch is set to 0, do not enable the quadcopter
          armState = false; break;

        case 1: //if the arm switch is set to 1, start the PID calculation

          #if defined(TIMEPROFILING) || defined(LOOP_SAMPLING) 
           #ifdef LOOP_SAMPLING
              while((micros() - timestart + (loopEnd - pidLastStart)) < PID_SAMPLETIME){
                  pidStartTime = (micros() - timestart);
              }                     
            #else
              pidStartTime = (micros() - timestart);
            #endif
          #endif

          initPids();   //start PID calcuation (pid.cpp)                           
          //pidEndTime = (micros() - timestart);
          
          writeMotor(0, motorPwmOut().one);   //PWM motor 1
          writeMotor(1, motorPwmOut().two);   //PWM motor 2
          writeMotor(2, motorPwmOut().three); //PWM motor 3
          writeMotor(3, motorPwmOut().four);  //PWM motor 4
                  
          armState = true; break;
        
        case 2:
          armState = false; break;
    
        default:
          armState = false; break; 
      }
    
    }else if(failsafeState() != 0){
    
    //turn motors off if failsafe is triggered
    Serial.print(failsafeState());
    armState = false;
    
    }else{
    
    //turn motors off if something else happens
    Serial.print(failsafeState());
    armState = false;
    
    }
                
    if(armState == false){
      writeMotor(0, 0);
      writeMotor(1, 0);
      writeMotor(2, 0);
      writeMotor(3, 0);
    }
    
    lastArmState = armState;

    #ifdef TIMEPROFILING
      printStartTime = (micros() - timestart);
    #endif
  
    #ifdef PRINT_SERIALDATA
      printSerial(); // used for GUI application and debugging
      profileLoop(); // prints time profiling data to serial monitor when LOOP_SAMPLING is enabled
    #endif

    #if defined(TIMEPROFILING) || defined(LOOP_SAMPLING)
    // make sure that time profiling variables aren't being redeclared each loop.
      imuLastStart = imuStartTime;    
      loopEnd = (micros() - timestart);
    #endif
    
    #if defined (GUI_ENABLED) || defined(TIMEPROFILING) || defined(LOOP_SAMPLING)
      cycles++;
    #endif
    
}


void writeMotor(int index, int value){
  analogWrite(motorOutput[index], (uint8_t)(value));
}

int armingState(){
  return armState;
}

int lastArmingState(){
  return lastArmState;
}
  
void printSerial(){

  #ifdef GUI_ENABLED
    Serial.println("$ " + String(cycles) + " " + String(imu_angles().x, 3) + " " + String(imu_angles().y, 3) + " " + String(imu_angles().z, 3) + " 0 " + String(chRoll()) + " " + String(chPitch()) + " " + String(chYaw()) + " "  + String(KpX, 4) + " " + String(KiX, 6) + " " + String(KdX) + " " + String(KpY, 4) + " " + String(KiY, 6) + " " + String(KdY) + " " + String(KpZ, 4) + " " + String(KiZ, 6) + " " + String(KdZ) + " " + (String)loopEnd + " " + (String)failsafeState());
  #endif
      
}


void profileLoop(){

  #ifdef LOOP_SAMPLING
      Serial.print(cycles);
      Serial.print("\t");      
      Serial.print(imuStartTime);
      Serial.print("\t");
      Serial.print(rxStartTime);
      Serial.print("\t");
      Serial.print(pidStartTime);
      Serial.print("\t");
      Serial.print(printStartTime);
      Serial.print("\t");
      Serial.print(loopEnd);
      Serial.print("\n");
  #endif
}

    /* 
    Serial.print(cycles); // add number of loops
    Serial.print(" ");
    Serial.print(imu_angles().x);
    Serial.print(" ");
    Serial.print(imu_angles().y);
    Serial.print(" ");
    Serial.print(imu_angles().z);
    Serial.print(" ");
    Serial.print("0"); // place holder for temperature
    Serial.print(" ");
    Serial.print(chRoll());
    Serial.print(" ");
    Serial.print(chPitch());
    Serial.print(" ");
    Serial.print(chYaw());
    Serial.print(" ");
    Serial.print(chRoll());
    Serial.print(" ");
    Serial.print(chPitch());
    Serial.print(" ");
    Serial.print(chYaw());
    Serial.print(" ");
    Serial.println(failsafeState());
    */
