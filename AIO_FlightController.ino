/*
 * AIO_FlightController - An integrated quadcopter flight controller program for the Arduino platform.
 * Copyright © 2018 Michael Delaney. All rights reserved.
 * 
 *  This device takes data from an inertial measurement unit about it's position and angle of inclination 
 *  and receieves information from the remote about the next desired action and makes several calculations 
 *  to determine the difference between the desired action, dictated by the remote, 
 *  and the actual response from the drone, measured by the accelerometer and gyroscope. Using this
 *  information, the drone can adjust to the desired position by varying the speed of it's motors according
 *  to calculations from the control loop.
 * 
 */
#include <Arduino.h>
#include "imu.h"
#include "RX.h"
#include "pid.h"
#include "config.h"

/*
int motorOutput[] = {13, 6, 10, 9}; //BOTTOM RIGHT, BOTTOM LEFT, TOP LEFT, TOP RIGHT
int motorOutput[] = {9, 5, 10, 6};  //version 1 configuration
int motorOutput[] = {x, x, x, x};   //version 3 configuration  *STILL TBD!!*
*/
int motorOutput[] = {10, 9, 13, 6}; //version 2 configuration
bool armState = false;
bool lastArmState = false;

void setup() {
  Serial.begin(115200);
  DDRB = DDRB | B11110000; //sets pins D8, D9, D10, D11 as outputs
  DDRC = DDRC | B11000000; //sets pin D5 as output
  DDRD = DDRD | B10000000; //sets pin D6 as output **NOTICE** SETS D0 as an input, will this interfere with the sBus decoding?
  DDRE = DDRE | B01000000; //sets pin D7 as output (ties to PPM pin --> not used)

  delay(2000); //give time for RX to get connection
  initSbus();  //connect to the remote reciever (rx.cpp)
  initIMU();   //activate the imu and set gyroscope and accelerometer sensitivity (imu.cpp)
  
  /* initilization and one time data collection about the enviornment 
   * --> store information and use it to calibrate for flight
   * 
   */

}

void loop() {

    readIMU(); //read the imu and calculate the quadcopters position relative to gravity (imu.cpp)
    readRx();  //read the remote and convert data to a rotational rate of ±180°/s

    if(failsafeState() == 0){
      
      switch(chAux1()){
        case 0: //if the arm switch is set to 0, do not enable the quadcopter
          armState = false; break;
      
        case 1: //if the arm switch is set to 1, start the PID calculation
          armState = true;
          initPids(); //start PID calcuation (pid.cpp)
          
            //if(failsafeState() == 0 && abs(imu_angles().x) < 20 && abs(imu_angles().x) < 20 || armState == lastArmState){
               //only activate motors if angle is less than 20°
            
              //only activate if failsafe is disengaged
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
    
    //printGUIData(); // used for GUI application
    lastArmState = armState;

}


void writeMotor(int index, float value){
  analogWrite(motorOutput[index], (uint8_t)(value));
}
 
void printGUIData(){

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
    //Serial.println(failsafeState());
    
    
}

int armingState(){
  return armState;
}

int lastArmingState(){
  return lastArmState;
}
    

