/*
    The PID Controller makes adjustments to the motor speeds in order to adjust orientation in the desired angle/direction
*/
#include "pid.h"
#include "config.h"

float outputX, outputY, outputZ;
unsigned long lastTime, currentT;

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
    lastTime = currentT;
} 
    

void computePids(){

    #ifndef LOOP_SAMPLING
      currentT = micros();
    #endif
    
    #ifdef HORIZON
      currentAngle.x = imu_angles().x; //read angle from IMU and set it to the current angle
      currentAngle.y = imu_angles().y;
      currentAngle.z = imu_angles().z;
      //currentAngle.z = 0;
    #endif

    #ifdef ACRO
      currentAngle.x = imu_rates().x; //read gyro rates from IMU and set it to the current angle
      currentAngle.y = imu_rates().y;
      currentAngle.z = imu_rates().z;
      //currentAngle.z = 0;
    #endif
    
    // compute all the working error vars
    // read rotational rate data (°/s) from remote and set it to the desired angle



    error.x = (-1 * chRoll())  - currentAngle.x;                      //present error
    error.y = (-1 * chPitch()) - currentAngle.y;
    error.z = chYaw()          - currentAngle.z;

/*
    if(chAux2() == 0){ 
      error.x = 45 - currentAngle.x;    // setting a fixed error value for step response
      error.y = currentAngle.y;
      error.z = chYaw()- currentAngle.z;
    }else{
      error.x = (-1 * chRoll())  - currentAngle.x;                      
      error.y = (-1 * chPitch()) - currentAngle.y;
      error.z = chYaw()          - currentAngle.z; 
    }
*/
    
    #ifndef LOOP_SAMPLING
    
    long timeChange = (currentT - lastTime);
    
    errorSum.x += error.x * timeChange;                               //integral of error
    errorSum.y += error.y * timeChange;
    errorSum.z += error.z * timeChange;
    
    deltaError.x = (currentAngle.x - lastAngle.x) / timeChange;       //derivative of error
    deltaError.y = (currentAngle.y - lastAngle.y) / timeChange; 
    deltaError.z = (currentAngle.z - lastAngle.z) / timeChange; 
   
    #else
    errorSum.x += error.x;                              //integral of error
    errorSum.y += error.y;
    errorSum.z += error.z;
    
    deltaError.x = currentAngle.x - lastAngle.x;        //derivative of error
    deltaError.y = currentAngle.y - lastAngle.y; 
    deltaError.z = currentAngle.z - lastAngle.z; 
    #endif
    
    //compute integral
    float Ix = KiX * errorSum.x;
    float Iy = KiY * errorSum.y;
    float Iz = KiZ * errorSum.z;

    
    //clamp the range of integral values
    if(Ix > MAX_INTEGRAL){ 
      Ix = MAX_INTEGRAL; 
    }else if (Ix < (MAX_INTEGRAL * -1)){
      Ix = MAX_INTEGRAL * -1;
    }
        
    if(Iy > MAX_INTEGRAL){ 
      Iy = MAX_INTEGRAL; 
    }else if (Iy < (MAX_INTEGRAL * -1)){
      Iy = MAX_INTEGRAL * -1;
    }
        
    if(Iz > MAX_INTEGRAL){ 
      Iz = MAX_INTEGRAL; 
    }else if (Iz < (MAX_INTEGRAL * -1)){
      Iz = MAX_INTEGRAL * -1;
    }

    outputX = (KpX * error.x + Ix - KdX * deltaError.x);
    outputY = (KpY * error.y + Iy - KdY * deltaError.y);
    outputZ = (KpZ * error.z + Iz - KdZ * deltaError.z);

/*
    #ifdef PRINT_SERIALDATA
      if(chAux2() == 0){
        Serial.print(",");
        Serial.print(outputX);
        Serial.print(",");
        Serial.print(outputY);
        Serial.print(",");
        Serial.print(outputZ);
      }
    #endif


    //removed integral clamping
    outputX = (KpX * error.x + KiX * errorSum.x - KdX * deltaError.x);
    outputY = (KpY * error.y + KiY * errorSum.y - KdY * deltaError.y);
    outputZ = (KpZ * error.z + KiZ * errorSum.z - KdZ * deltaError.z);


    //compute PID output as a function of the throttle
    outputX = (KpX * error.x + Ix - KdX * deltaError.x)*(chThrottle() / (ESC_MAX * ESC_TOLERANCE));
    outputY = (KpY * error.y + Iy - KdY * deltaError.y)*(chThrottle() / (ESC_MAX * ESC_TOLERANCE));
    outputZ = (KpZ * error.z + Iz - KdZ * deltaError.z)*(chThrottle() / (ESC_MAX * ESC_TOLERANCE));
*/
 
  
    //write outputs to corresponding motors at the corresponding speed
     motorSpeed.one = abs(chThrottle() + outputX - outputY - outputZ); 
     motorSpeed.two = abs(chThrottle() - outputX - outputY + outputZ); 
     motorSpeed.three = abs(chThrottle() - outputX + outputY - outputZ);
     motorSpeed.four = abs(chThrottle() + outputX + outputY + outputZ);
     
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
