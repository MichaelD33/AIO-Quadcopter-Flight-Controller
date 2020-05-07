/*
    The PID Controller makes adjustments to the motor speeds in order to adjust orientation in the desired angle/direction
*/
#include "pid.h"
#include "config.h"

float Kp = 0.7;
float Ki = 0.015;
float Kd = 0;

float KpZ = 0;
float KiZ = 0;
float KdZ = 0;

float outputX, outputY, outputZ;

axis_int16_t desiredAngle;
axis_float_t currentAngle, lastAngle;
axis_float_t error, deltaError, errorSum;

int_pwmOut motorSpeed;

void initPids(){
  //time since last calculation
    
    computePids();
    resetPids();
    lastAngle.x = currentAngle.x;
    lastAngle.y = currentAngle.y;
    lastAngle.z = currentAngle.z;

//    Kp = (0.7);
//    Ki = (0 + (2*chAuxPot1()) + (4*chAuxPot2())) * SAMPLETIME_S;
    Kd = (0 + (chAuxPot1()/5) + (chAuxPot2()/4)) / SAMPLETIME_S;
} 
    

void computePids(){
    
    #ifdef HORIZON
      currentAngle.x = imu_angles().x; //read angle from IMU and set it to the current angle
      currentAngle.y = imu_angles().y;
      currentAngle.z = imu_angles().z;
      //currentAngle.z = 0;
    #endif
    
    // compute all the working error vars
    // read rotational rate data (°/s) from remote and set it to the desired angle


    error.x = chRoll()  - currentAngle.x;        //present error (instantaneous error)
    error.y = chPitch() - currentAngle.y;
    error.z = chYaw()          - currentAngle.z;
    
  
    errorSum.x += Ki * error.x;                              //integral of error (total accumulation of error)
    errorSum.y += Ki * error.y;
    errorSum.z += KiZ * error.z;

    
    deltaError.x = currentAngle.x - lastAngle.x;        //derivative of error (change in error)
    deltaError.y = currentAngle.y - lastAngle.y; 
    deltaError.z = currentAngle.z - lastAngle.z; 
      

    
    //clamp the range of integral values
    if(errorSum.x > MAX_INTEGRAL){ 
      errorSum.x = MAX_INTEGRAL; 
    }else if (errorSum.x < (MAX_INTEGRAL * -1)){
      errorSum.x = MAX_INTEGRAL * -1;
    }
        
    if(errorSum.y > MAX_INTEGRAL){ 
      errorSum.y = MAX_INTEGRAL; 
    }else if (errorSum.y < (MAX_INTEGRAL * -1)){
      errorSum.y = MAX_INTEGRAL * -1;
    }
        
    if(errorSum.z > MAX_INTEGRAL){ 
      errorSum.z = MAX_INTEGRAL; 
    }else if (errorSum.z < (MAX_INTEGRAL * -1)){
      errorSum.z = MAX_INTEGRAL * -1;
    }

    outputX = (Kp * error.x + errorSum.x - Kd * deltaError.x);
    outputY = (Kp * error.y + errorSum.y - Kd * deltaError.y);
    outputZ = (KpZ * error.z + errorSum.z - KdZ * deltaError.z);
 
  
    //write outputs to corresponding motors at the corresponding speed
     motorSpeed.one = (chThrottle() - outputX + outputY - outputZ); 
     motorSpeed.two = (chThrottle() + outputX + outputY + outputZ); 
     motorSpeed.three = (chThrottle() + outputX - outputY - outputZ);
     motorSpeed.four = (chThrottle() - outputX - outputY + outputZ);
     
     //clamp the min and max output from the pid controller (to match the needed 0-255 for pwm)
     if(motorSpeed.one > ESC_MAX){
        motorSpeed.one = ESC_MAX;  
       }else if (motorSpeed.one < ESC_MIN){
        motorSpeed.one = ESC_MIN;
       }else{ }  

     if(motorSpeed.two > ESC_MAX){
        motorSpeed.two = ESC_MAX;  
       }else if (motorSpeed.two < ESC_MIN){
        motorSpeed.two = ESC_MIN;
       }else{ } 

     if(motorSpeed.three > ESC_MAX){
        motorSpeed.three = ESC_MAX;  
       }else if (motorSpeed.three < ESC_MIN){
        motorSpeed.three = ESC_MIN;
       }else{ } 

     if(motorSpeed.four > ESC_MAX){
        motorSpeed.four = ESC_MAX;  
       }else if (motorSpeed.four < ESC_MIN){
        motorSpeed.four = ESC_MIN;
       }else{ } 

    #ifdef PRINT_SERIALDATA
      if(chAux2() == 0){
        Serial.print("IMU Roll: ");      
        Serial.print(imu_angles().x);
        Serial.print(", IMU Pitch: ");      
        Serial.print(imu_angles().y);
        Serial.print(", Kp: ");
        Serial.print(Kp);
        Serial.print(", Ki: ");
        Serial.print(Ki, 4);
        Serial.print(", Kd: ");
        Serial.println(Kd);
      }
    #endif
           
}


void resetPids(){

   if(chThrottle() < 10){    
      errorSum.x = 0;
      errorSum.y = 0;
      errorSum.z = 0;   
      
    }else if(armingState() != lastArmingState()){
    //reset the integral term when the quadcopter is armed
      errorSum.x = 0;
      errorSum.y = 0;
      errorSum.z = 0;
    
  }
       
}

int_pwmOut motorPwmOut(){
  return motorSpeed;
}
