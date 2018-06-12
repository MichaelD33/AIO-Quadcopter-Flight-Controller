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
float maxIx = 230;
float maxIy = 230;
float maxIz = 230;
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
     lastTime = currentTime;
 if(timeChange >= PID_SAMPLETIME){
  
    //Serial.print("1: "); 
    //Serial.println(timeChange); 
     
    computePids();
    resetPids();    
  }else if(timeChange < PID_SAMPLETIME){
  
    delayMicroseconds(PID_SAMPLETIME - timeChange);      // HOW DO I CORRECT TO GET ACCURATE PID SAMPLE TIMES? 
   // Serial.print("2: "); 
   // Serial.println(timeChange + (PID_SAMPLETIME - timeChange));    
   
    computePids();
    resetPids();
  }
} 
    

void computePids(){
  
    desiredAngle.x = -1 * chRoll(); //read rotational rate data (°/s) from remote and set it to the desired angle
    desiredAngle.y = -1 * chPitch(); // FOR RATE MODE: consider using mod() to bring values back to 0 when it goes past 180 (if I change the equation to +=)
    desiredAngle.z = -1 * chYaw();  // multiplied by -1 to flip output of the remote (because right motors need to turn on in order to move to the left - and vice versa)

    currentAngle.x = imu_angles().x; //read angle from IMU and set it to the current angle
    currentAngle.y = imu_angles().y;
    currentAngle.z = imu_angles().z;

   // currentAngle.x = imu_rates().x; //read gyro rates from IMU and set it to the current angle
   // currentAngle.y = imu_rates().y;
   // currentAngle.z = imu_rates().z;

    //compute all the working error vars
    error.x =  desiredAngle.x - currentAngle.x;  //present error
    errorSum.x += error.x;                       //integral of the error
    deltaError.x = currentAngle.x - lastAngle.x; //derivative of the error
    
    error.y =  desiredAngle.y - currentAngle.y;
    errorSum.y += error.y;
    deltaError.x = currentAngle.x - lastAngle.x; 

    error.z =  desiredAngle.z - currentAngle.z;
    errorSum.z += error.z;
    deltaError.z = currentAngle.z - lastAngle.z; 

    //compute proportional
    double Px = KpX * error.x;
    double Py = KpY * error.y;
    double Pz = KpZ * error.z;

    //compute integral
    double Ix = KiX * errorSum.x;
    double Iy = KiY * errorSum.y;
    double Iz = KiZ * errorSum.z;

    //compute derivative
    double Dx = KdX * deltaError.x;
    double Dy = KdY * deltaError.y;
    double Dz = KdZ * deltaError.z;

    //clamp the range of integral values
        if(Ix > maxIx){ 
          Ix = maxIx; 
        }else if (Ix < (maxIx * -1)){
          Ix = maxIx * -1;
        }   
        if(Iy > maxIy){ 
          Iy = maxIy; 
        }else if (Iy < (maxIy * -1)){
          Iy = maxIy * -1;
        }
         if(Iz > maxIz){ 
          Iz = maxIz; 
        }else if (Iz < (maxIz * -1)){
          Iz = maxIz * -1;
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

     //clamp the min and max output from the pid controller (to match the nedded 0-255 for pwm)
     if(motorSpeed.one > 255){
        motorSpeed.one = 255;  
       }else if (motorSpeed.one < 0){
        motorSpeed.one = 0;
       }else{  }  

     if(motorSpeed.two > 255){
        motorSpeed.two = 255;  
       }else if (motorSpeed.two < 0){
        motorSpeed.two = 0;
       }else{ } 

     if(motorSpeed.three > 255){
        motorSpeed.three = 255;  
       }else if (motorSpeed.three < 0){
        motorSpeed.three = 0;
       }else{  } 

     if(motorSpeed.four > 255){
        motorSpeed.four = 255;  
       }else if (motorSpeed.four < 0){
        motorSpeed.four = 0;
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
