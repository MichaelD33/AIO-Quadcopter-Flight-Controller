/*
 *  AIO_FlightController - An integrated quadcopter flight controller program for the Arduino platform.
 *  Copyright © 2018-2019 Michael Delaney. All rights reserved.
 * 
 *  This device takes orientation data from an inertial measurement unit and input from an external remote to adjust 
 *  its position by varying the speed of its motors according to calculations made by the control loop.
 * 
 *  Source Code: https://github.com/MichaelD33/AIO-Quadcopter-Flight-Controller
 *  Design Files: https://github.com/MichaelD33/AIO-Quadcopter-Design
 *  
 *  A simple quadcopter flight controller in ~1500 lines of code

    This program is part of the AIO Flight Controller.
    
    The AIO Flight Controller is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    The AIO Flight Controller is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
  
 */

#include <Arduino.h>
#include "config.h"
#include "imu.h"
#include "RX.h"
#include "pid.h"

bool armState = false;
bool lastArmState = false;

#if defined(LOOP_SAMPLING) || defined(TIMEPROFILING) || defined(GUI_ENABLED)
  unsigned long printStartTime, imuStartTime, rxStartTime, pidStartTime;
  unsigned long timestart;
  unsigned long loopEnd = 0;
  unsigned long imuEndTime = 0;
  unsigned long pidEndTime = 0;
#endif

#ifdef AIO_v01 //prototype v0.1 configuration - LEGACY
  byte motorOutput[] = {9, 5, 10, 6};  
  #define ATMEGA32u4
#endif

#ifdef AIO_v03  //version 0.3 configuration - LEGACY (purple frame)
  byte motorOutput[] = {5, 9, 6, 10};  
//  byte motorOutput[] = {6, 10, 5, 9};   
  #define ATMEGA32u4
#endif

#ifdef AIO_v04 //version 0.4 configuration - LEGACY
  byte motorOutput[] = {10, 9, 13, 6}; 
  #define ATMEGA32u4
#endif

#ifdef AIO_v041 //version 0.4.1 configuration - Black PCB
  byte motorOutput[] = {10, 9, 5, 6};  
  #define ATMEGA32u4
#endif

/*  IF NOT USING THE AIO PCB, USE THE FORMAT SHOWN BELOW —— (MOTOR LOCATION REF. DIAGRAM ON GITHUB) */
//  byte motorOutput[] = {[motor 1], [motor 2], [motor 3], [motor 4]}; 

void setup() {

  #ifdef PRINT_SERIALDATA
    Serial.begin(115200);
  #endif

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
      while((micros() - imuEndTime) < IMU_SAMPLETIME){
        imuStartTime = micros();
      }
    #else
        imuStartTime = micros();
    #endif

  #endif

// IMU supports up to 8kHz gyro update rate and 1kHz acc update rate --- when DLPF is activated this is diminished significantly (see MPU6050 register mapping datasheet)
   readIMU(); //read the imu and calculate the quadcopters position relative to gravity (imu.cpp)

   #if defined(LOOP_SAMPLING) || defined(TIMEPROFILING)
    
    unsigned long imu_dt = imuStartTime - imuEndTime;
    Serial.print("IMU ∆T: ");
    Serial.println(imu_dt);
    
    imuEndTime = micros();  // record end time to use for sampling calculation
    
    unsigned long processTime = imuEndTime - imuStartTime;
    Serial.print("IMU Process Time: ");
    Serial.println(processTime);
   #endif

   readRx();  //read the remote and convert data to a rotational rate of ±180°/s (rx.cpp)

   if(failsafeState() == 0){
      switch(chAux1()){
        case 0: //if the arm switch is set to 0, do not enable the quadcopter
          armState = false; break;

        case 1: //if the arm  switch is set to 1, start the PID calculation

          #if defined(TIMEPROFILING) || defined(LOOP_SAMPLING)
           #ifdef LOOP_SAMPLING
              while((micros() - pidEndTime) < PID_SAMPLETIME){
                pidStartTime = micros();
              }                     
            #else
              pidStartTime = micros();
            #endif
          #endif

          initPids();   //start PID calcuation (pid.cpp)               

          #if defined(LOOP_SAMPLING) || defined(TIMEPROFILING)
            Serial.print("PID ∆T: ");
            unsigned long pid_dt = pidStartTime - pidEndTime;
            Serial.println(pid_dt);
            
            pidEndTime = micros();
           
            Serial.print("PID Process Time: ");
            Serial.println(pidEndTime - pidStartTime);
          #endif
          
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
    #endif

    unsigned long loop_dt = micros() - timestart;
    Serial.print("Loop Time: ");
    Serial.println(loop_dt);
    Serial.println("———————————");

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

/*

  switch(chAux2()){
    
    case 0: break;

    case 1: break;

    case 2: break;

    default: break;
    
  }
*/
    
}
