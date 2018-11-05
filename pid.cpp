/*
    // The PID Controller is used to understand how much to correct the speed of the motors in order to achieve desired angle/direction
    // Each axis needs their own PID controller
    // Calculates error based on difference between sensor reading and pilot commands
    // Proportional term depends on present error
    // Integral term depends on accumulation of past errors
    // Derivative? --> maybe don't integrate because it is very suseptible to noise
*/
#include "pid.h"
#include "config.h"

float outputX;
float outputY;
float outputZ;
float timeChange;
//float throttleGain = 0.0023;

long lastTime = 0;

axis_int16_t desiredAngle;
axis_float_t currentAngle;
axis_float_t error, deltaError, errorSum, lastAngle;

float_pwmOut motorSpeed;




void initPids(){
  //time since last calculation
 
    computePids();
    resetPids();
    lastAngle.x = currentAngle.x;
    lastAngle.y = currentAngle.y;
    lastAngle.z = currentAngle.z;
    
} 
    

void computePids(){

    #ifdef HORIZON
      currentAngle.x = imu_angles().x; //read angle from IMU and set it to the current angle
      currentAngle.y = imu_angles().y;
      currentAngle.z = imu_angles().z;
      //currentAngle.z = 0;
    #endif

    #ifdef ACRO
      currentAngle.x = imu_rates().x / GYRO_SENS; //read gyro rates from IMU and set it to the current angle
      currentAngle.y = imu_rates().y / GYRO_SENS;
      currentAngle.z = imu_rates().z / GYRO_SENS;
      //currentAngle.z = 0;
    #endif
    
    // compute all the working error vars
    // read rotational rate data (°/s) from remote and set it to the desired angle
    // FOR RATE MODE: consider using mod() to bring values back to 0 when it goes past 180 (if I change the equation to +=)
    
    error.x = (-1 * chRoll()) - currentAngle.x;        //present error
    errorSum.x += error.x;                             //integral of the error
    deltaError.x = currentAngle.x - lastAngle.x;       //derivative of the error
    
    error.y = (-1 * chPitch()) - currentAngle.y;
    errorSum.y += error.y;
    deltaError.x = currentAngle.x - lastAngle.x; 

    error.z = chYaw() - currentAngle.z;
    errorSum.z += error.z * timeChange;
    deltaError.z = currentAngle.z - lastAngle.z; 
 

/*
    //compute proportional
    float Px = KpX * error.x;
    float Py = KpY * error.y;
    float Pz = KpZ * error.z;

    //compute derivative
    float Dx = KdX * deltaError.x;
    float Dy = KdY * deltaError.y;
    float Dz = KdZ * deltaError.z;
       
    //compute PID output 
    outputX = (Px + Ix - Dx);
    outputY = (Py + Iy - Dy);
    outputZ = (Pz + Iz - Dz);
*/


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
           
}

void resetPids(){

   if(chThrottle() < 10){
    
      errorSum.x = 0;
      errorSum.y = 0;
      errorSum.z = 0;   
      
    }else if(armingState() != lastArmingState()){
    //reset the integral term when the quadcopter is armed
    //this should work b/c initPids() does not run when armState switches to false
    
    errorSum.x = 0;
    errorSum.y = 0;
    errorSum.z = 0;
    
  }
       
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
