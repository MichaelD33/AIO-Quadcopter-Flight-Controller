/*
    // The PID Controller is used to understand how much to correct the speed of the motors in order to achieve desired angle/direction
    // Each axis needs their own PID controller
    // Calculates error based on difference between sensor reading and pilot commands
    // Proportional term depends on present error
    // Integral term depends on accumulation of past errors
    // Derivative? --> maybe don't integrate because it is very suseptible to noise
*/
#include <Arduino.h>
#include "pid.h"
#include "config.h"

axis_float_t desiredAngle;
axis_float_t currentAngle;
float outputX;
float outputY;
float outputZ;
axis_float_t error, deltaError, errorSum, lastAngle;
float_pwmOut motorSpeed;
float throttleGain = 0.0023;
float timeChange;
long lastTime = 0;


void initPids(){
  //time since last calculation
            
  if(armingState() != lastArmingState()){
    //reset the integral term when the quadcopter is armed
    //this should work b/c initPids() does not run when armState switches to false
    errorSum.x = 0;
    errorSum.y = 0;
    errorSum.z = 0;
  }else{
    //do nothing
  }
  
  long currentTime = micros();
  timeChange = (float)(currentTime - lastTime);

// if(timeChange >= PID_SAMPLETIME){ 
    computePids();
    resetPids();
//  }
  lastTime = currentTime;
} 
    

void computePids(){
  
    desiredAngle.x = -1 * chRoll(); //read rotational rate data (°/s) from remote and set it to the desired angle
    desiredAngle.y = -1 * chPitch(); // FOR RATE MODE: consider using mod() to bring values back to 0 when it goes past 180 (if I change the equation to +=)
    desiredAngle.z = -1 * chYaw();  // multiplied by -1 to flip output of the remote (because right motors need to turn on in order to move to the left - and vice versa)

    #ifdef HORIZON
      currentAngle.x = imu_angles().x; //read angle from IMU and set it to the current angle
      currentAngle.y = imu_angles().y;
      currentAngle.z = 0; //need magnetometer for accurate Z reference - also using reference angle causes directional lock north.
    #endif

    #ifdef ACRO
      currentAngle.x = imu_rates().x; //read gyro rates from IMU and set it to the current angle
      currentAngle.y = imu_rates().y;
      //currentAngle.z = imu_rates().z;
      currentAngle.z = 0; //need magnetometer for accurate Z reference - also using reference angle causes directional lock north.
    #endif
    
    //compute all the working error vars
    error.x =  desiredAngle.x - currentAngle.x;                 //present error
    errorSum.x += error.x * timeChange;                         //integral of the error
    deltaError.x = (currentAngle.x - lastAngle.x) / timeChange; //derivative of the error
    
    error.y =  desiredAngle.y - currentAngle.y;
    errorSum.y += error.y * timeChange;
    deltaError.x = (currentAngle.x - lastAngle.x) / timeChange; 

    error.z =  desiredAngle.z - currentAngle.z;
    errorSum.z += error.z * timeChange;
    deltaError.z = (currentAngle.z - lastAngle.z) / timeChange; 

    //compute proportional
    float Px = KpX * error.x;
    float Py = KpY * error.y;
    float Pz = KpZ * error.z;

    //compute integral
    float Ix = KiX * errorSum.x;
    float Iy = KiY * errorSum.y;
    float Iz = KiZ * errorSum.z;

    //compute derivative
    float Dx = KdX * deltaError.x;
    float Dy = KdY * deltaError.y;
    float Dz = KdZ * deltaError.z;

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
   
    //compute PID output as a function of the throttle
    outputX = (Px + Ix - Dx)*(chThrottle() * throttleGain);
    outputY = (Py + Iy - Dy)*(chThrottle() * throttleGain);
    outputZ = (Pz + Iz - Dz)*(chThrottle() * throttleGain); 

    //write outputs to corresponding motors at the corresponding speed
    
     motorSpeed.one = abs(chThrottle() + outputX - outputY - outputZ); 
     motorSpeed.two = abs(chThrottle() - outputX - outputY + outputZ); 
     motorSpeed.three = abs(chThrottle() - outputX + outputY - outputZ);
     motorSpeed.four = abs(chThrottle() + outputX + outputY + outputZ);
     /*
     motorSpeed.one = (chThrottle() + outputX - outputY - outputZ); 
     motorSpeed.two = (chThrottle() - outputX - outputY + outputZ); 
     motorSpeed.three = (chThrottle() - outputX + outputY - outputZ);
     motorSpeed.four = (chThrottle() + outputX + outputY + outputZ);
     */
     
     //clamp the min and max output from the pid controller (to match the nedded 0-255 for pwm)
     if(motorSpeed.one > ESC_MAX){
        motorSpeed.one = ESC_MAX;  
       }else if (motorSpeed.one < ESC_MIN){
        motorSpeed.one = ESC_MIN;
       }else{  }  

     if(motorSpeed.two > ESC_MAX){
        motorSpeed.two = ESC_MAX;  
       }else if (motorSpeed.two < ESC_MIN){
        motorSpeed.two = ESC_MIN;
       }else{ } 

     if(motorSpeed.three > ESC_MAX){
        motorSpeed.three = ESC_MAX;  
       }else if (motorSpeed.three < ESC_MIN){
        motorSpeed.three = ESC_MIN;
       }else{  } 

     if(motorSpeed.four > ESC_MAX){
        motorSpeed.four = ESC_MAX;  
       }else if (motorSpeed.four < ESC_MIN){
        motorSpeed.four = ESC_MIN;
       }else{  } 
           
    /*
     Serial.print(outputX);
     Serial.print(", "); 
     Serial.print(outputY);
     Serial.print(", "); 
     Serial.print(outputZ);
     Serial.print(", "); 
     Serial.println(motorSpeed.one);
     
     Serial.print(motorSpeed.one);
     Serial.print(", "); 
     Serial.print(motorSpeed.two);
     Serial.print(", "); 
     Serial.print(motorSpeed.three);
     Serial.print(", "); 
     Serial.println(motorSpeed.four);
    
     Serial.print(Px);
     Serial.print(", "); 
     Serial.print(Ix);
     Serial.print(", "); 
     Serial.println(outputX);
      */
}

void resetPids(){
  
    //store relevant variables
    lastAngle.x = currentAngle.x;
    lastAngle.y = currentAngle.y;
    lastAngle.z = currentAngle.z;
       
}

float_pwmOut motorPwmOut(){
  //allows me to return all four speeds in one method
  return motorSpeed;
}

/*
void setSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}
*/
