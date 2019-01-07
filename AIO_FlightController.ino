/*
 *  AIO_FlightController - An integrated quadcopter flight controller program for the Arduino platform.
 *  Copyright © 2018 Michael Delaney. All rights reserved.
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

#ifdef LOOP_SAMPLING
  int imuLastStart, pidLastStart, printStartTime, imuStartTime, pidStartTime;
//  int pidTime, rxTime, imuTime;  
  long loopEnd;
#endif

#ifdef AIO_v01
  byte motorOutput[] = {9, 5, 10, 6};  //prototype v0.1 configuration - RETIRED
  #define ATMEGA32u4
#endif

#ifdef AIO_v03
  byte motorOutput[] = {5, 9, 6, 10};   //version 0.3 configuration - GREEN BOARD
  #define ATMEGA32u4
#endif

#ifdef AIO_v04
  byte motorOutput[] = {10, 9, 13, 6}; //version 0.4 configuration - RED BOARD
  #define ATMEGA32u4
#endif

#ifdef AIO_v041
  byte motorOutput[] = {10, 9, 5, 6};  //version 0.4.1
  #define ATMEGA32u4
#endif

/*  IF NOT USING THE AIO PCB USE FORMAT AS SHOWN BELOW  (REF. DIAGRAM ON GITHUB FOR MOTOR LOCATIONS) */
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

  #ifdef LOOP_SAMPLING  
    long timestart = micros();            
       
    while((micros() + ((loopEnd - imuLastStart) + timestart)) < IMU_SAMPLING_FREQUENCY){
      //do nothing
      imuStartTime = (micros() - timestart);
    }    
   #endif
    
// IMU supports up to 8kHz gyro update rate and 1kHz acc update rate --- when DLPF is activated this is diminished significantly (see MPU6050 register mapping datasheet)
   readIMU(); //read the imu and calculate the quadcopters position relative to gravity (imu.cpp)
//   imuTime = (micros() - timestart) - imuStartTime;    
     
   readRx();  //read the remote and convert data to a rotational rate of ±180°/s (rx.cpp)
//   rxTime = (micros() - timestart) - imuTime;


   if(failsafeState() == 0){    
      switch(chAux1()){
        case 0: //if the arm switch is set to 0, do not enable the quadcopter
          armState = false; break;
       
        case 1: //if the arm switch is set to 1, start the PID calculation
          armState = true;
          
          #ifdef LOOP_SAMPLING
            while((micros() + ((loopEnd - pidLastStart) + timestart)) < PID_SAMPLETIME){
                //do nothing
                pidStartTime = (micros() - timestart);
            }     
          
           
//            pidStartTime = micros() - timestart;
            initPids();   //start PID calcuation (pid.cpp)
//            pidTime = (micros() - timestart) - (pidStartTime);
          #else
            initPids();   //start PID calcuation (pid.cpp)
          #endif
          
            //if( ( abs(imu_angles().x) < 20 && abs(imu_angles().y) < 20 ) || armState == lastArmState){
               //only activate motors if angle is less than 20°
               
              /*
             #ifdef LOOP_SAMPLING
             
               while(micros() - timestart < MOTOR_UPDATE_FREQUENCY){
                   int dT3 = micros() - timestart;
               } 
               
             #endif 
               */

                   writeMotor(0, motorPwmOut().one);   //PWM motor 1
                   writeMotor(1, motorPwmOut().two);   //PWM motor 2
                   writeMotor(2, motorPwmOut().three); //PWM motor 3
                   writeMotor(3, motorPwmOut().four);  //PWM motor 4

            break;
        
        case 2:
          armState = false; break;
    
        default: 
          armState = false; break; 
      }
    }else if(failsafeState != 0){
    //turn motors off if failsafe is triggered
    armState = false;
    }else{
    //turn motors off if something else happens
    armState = false;
    }
                
    if(armState == false){
      writeMotor(0, 0);
      writeMotor(1, 0);
      writeMotor(2, 0);
      writeMotor(3, 0);
    }
    
    lastArmState = armState;


    #ifdef LOOP_SAMPLING
      printStartTime = (micros() - timestart);
    #endif
  
    #ifdef PRINT_SERIALDATA
      printSerial(); // used for GUI application and debugging
      profileLoop(); // prints time profiling data to serial monitor when LOOP_SAMPLING is enabled
    #endif

    #ifdef LOOP_SAMPLING
    // make sure that time profiling variables aren't being redeclared each loop.
      loopEnd = (micros() - timestart);
      imuLastStart = imuStartTime;    
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

/*
    Serial.print("s ");
    Serial.print(imu_angles().x);
    Serial.print(" ");
    Serial.print(imu_angles().y);
    Serial.print(" ");
    Serial.print(imu_angles().z);
    Serial.print(" ");
    Serial.print(chRoll());
    Serial.print(" ");
    Serial.print(chPitch());
    Serial.print(" ");
    Serial.println(chYaw());
    //Serial.print(" ");
    Serial.println(failsafeState());
*/

    #ifdef FULL_PROCESS_DEBUG
      Serial.print(imu_rates().x);
      Serial.print("\t|\t");   
      Serial.print(imu_angles().x);
      Serial.print("\t|\t");   
      
      if(failsafeState() == 0){
        Serial.print("REMOTE ACTIVE — ");
        Serial.print(chRoll());
      }else if(failsafeState() == 1){
        Serial.print("SIGNAL LOST");        
      }else if(failsafeState() == 3){
        Serial.print("FAILSAFE ENGAGED");
      }else{
        Serial.print("REMOTE ERROR!");
      }

      /* add PID
      Serial.print("\t|\t");   
      Serial.print(outputX);  */
      Serial.print("\t|\t");   
      Serial.println(motorPwmOut().one);
      
    #endif

       
}


void profileLoop(){

  #ifdef LOOP_SAMPLING
//      Serial.print("IMU Start: ");
      Serial.print(imuStartTime);
      Serial.print("\t");
//      Serial.print("\tPID Start: ");
      Serial.print(pidStartTime);
      Serial.print("\t");
//      Serial.print("\tPrint Start: ");
      Serial.print(printStartTime);
      Serial.print("\t");
//      Serial.print("\tPrev. Loop End: ");
      Serial.print(loopEnd);
      Serial.print("\t");
/*        
//      Serial.print("\tIMU: ");
      Serial.print(imuTime);
      Serial.print("\t");
//      Serial.print("\tRX: ");
      Serial.print(rxTime);
      Serial.print("\t");
//      Serial.print("\t\tPID: ");
      Serial.println(pidTime);
*/
  #endif
}

