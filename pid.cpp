/*
    The PID Controller makes adjustments to the motor speeds in order to adjust orientation in the desired angle/direction
*/
#include "pid.h"
#include "config.h"

/* PID Gains for the Rate Controller */
float KpR = 0.8;
float KiR = 0.065;
float KdR = 7.5;

float KpRZ = 0.1;
float KiRZ = 0;
float KdRZ = 0;


/* PID Gains for the Stabilization Controller */
float Kp = 0; // 0.12;
float Ki = 0;
float Kd = 0; // 0.53 or 1.25

float KpZ = 0;
float KiZ = 0;
float KdZ = 0;

float outputX, outputY, outputZ;

axis_int16_t desiredAngle;
axis_float_t currentAngle, lastAngle;
axis_float_t error, deltaError, errorSum;


axis_float_t errorRate, deltaRate, rateIntegral, currentRate, previousRate, rateOutput;

int_pwmOut motorSpeed;

void initPids(){
  //time since last calculation

/* Call the computePID function to calculate the motor speeds for the quadcopter */
    computePids();
    resetPids();

/* Store the previous loop's measurements in order to calculate derivative for PID controller(s) */
    lastAngle.x = currentAngle.x;
    lastAngle.y = currentAngle.y;
    lastAngle.z = currentAngle.z;

    previousRate.x = currentRate.x;
    previousRate.y = currentRate.y;
    previousRate.z = currentRate.z;

/* On-the-fly PID tuning configuration —> Use the remote's aux channels for adjusting gains while the device is running */
//    KpR = (0.0 + chAuxPot1()/5 + chAuxPot2()/2);
//    Ki = (0 + (2*chAuxPot1()) + (4*chAuxPot2())) * SAMPLETIME_S;
//    Kd = (0.0 + (chAuxPot1()/5) + (chAuxPot2()/2)) / SAMPLETIME_S;
} 
    

void computePids(){

  
    /*  —————————————————————————————————— The Rate PID Controller  ——————————————————————————————————  */
    
    currentRate.x = imu_rates().x;                  // read rotational rate from the IMU
    currentRate.y = imu_rates().y;
    currentRate.z = imu_rates().z;
    
    errorRate.x = (-1 * chRoll())  - currentRate.x;        // compute present error (proportional)
    errorRate.y = (-1 * chPitch()) - currentRate.y;
    errorRate.z = chYaw()   - currentRate.z;

    rateIntegral.x += KiR * errorRate.x;            // compute the total accumulation of error (integral)
    rateIntegral.y += KiR * errorRate.y;  
    rateIntegral.z += KiRZ * errorRate.z;

    deltaRate.x = currentRate.x - previousRate.x;   // compute the change in error (derivative)
    deltaRate.y = currentRate.y - previousRate.y;
    deltaRate.z = currentRate.z - previousRate.z;
    
    rateOutput.x = KpR * errorRate.x  + rateIntegral.x - KdR * deltaRate.x;
    rateOutput.y = KpR * errorRate.y  + rateIntegral.y - KdR * deltaRate.y; 
    rateOutput.z = KpRZ * errorRate.z + rateIntegral.z - KdRZ * deltaRate.z;

    motorSpeed.one = (chThrottle() - rateOutput.y - rateOutput.z); 
    motorSpeed.two = (chThrottle() - rateOutput.x + rateOutput.z); 
    motorSpeed.three = (chThrottle() + rateOutput.y - rateOutput.z);
    motorSpeed.four = (chThrottle() + rateOutput.x + rateOutput.z);


    /*  —————————————————————————————————— The Angle PID Controller ——————————————————————————————————  
    
    #ifdef HORIZON
      currentAngle.x = imu_angles().x; //read angle from IMU and set it to the current angle
      currentAngle.y = imu_angles().y;
      currentAngle.z = imu_angles().z;
      //currentAngle.z = 0;
    #endif
    
    // compute all the working error vars
    // read rotational rate data (°/s) from remote and set it to the desired angle


    error.x = chRoll()  - currentAngle.x;          //present error (instantaneous error)
    error.y = chPitch() - currentAngle.y;
    error.z = chYaw()   - currentAngle.z;
    
  
    errorSum.x += Ki * error.x;                    //integral of error (total accumulation of error)
    errorSum.y += Ki * error.y;
    errorSum.z += KiZ * error.z;

    
    deltaError.x = currentAngle.x - lastAngle.x;   //derivative of error (change in error)
    deltaError.y = currentAngle.y - lastAngle.y; 
    deltaError.z = currentAngle.z - lastAngle.z; 
     
    
    /* —————————————————————————————————— Clamp the range of integral values ——————————————————————————————————
    
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

    /* —————————————————————————————————— Calculate the motor speed adjustments for each axis of rotation based on PID calculations ——————————————————————————————————

    outputX = (Kp * error.x + errorSum.x - Kd * deltaError.x);
    outputY = (Kp * error.y + errorSum.y - Kd * deltaError.y);
    outputZ = (KpZ * error.z + errorSum.z - KdZ * deltaError.z);
 
  
     /* —————————————————————————————————— Write outputs to corresponding motors at the calculated speed —————————————————————————————————— 
     
     motorSpeed.one = (chThrottle() - outputX + outputY - outputZ); 
     motorSpeed.two = (chThrottle() + outputX + outputY + outputZ); 
     motorSpeed.three = (chThrottle() + outputX - outputY - outputZ);
     motorSpeed.four = (chThrottle() - outputX - outputY + outputZ);
     
     /* —————————————————————————————————— clamp the min and max output from the pid controller (to match the needed 0-255 for pwm) —————————————————————————————————— */
     
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
        Serial.print("RX-X:");      
        Serial.print(chRoll());
        Serial.print(", RX-Y:");      
        Serial.print(chPitch());
        Serial.print(", Rate-X:");
        Serial.print(currentRate.x);
        Serial.print(", Rate-Y:");
        Serial.println(currentRate.y);
      }
    #endif
           
}


void resetPids(){

   if(chThrottle() < 10){    
      errorSum.x = 0;
      errorSum.y = 0;
      errorSum.z = 0;   

      rateIntegral.x = 0;
      rateIntegral.y = 0;
      rateIntegral.z = 0;
      
    }else if(armingState() != lastArmingState()){
    //reset the integral term when the quadcopter is armed
      errorSum.x = 0;
      errorSum.y = 0;
      errorSum.z = 0;
      
      rateIntegral.x = 0;
      rateIntegral.y = 0;
      rateIntegral.z = 0;
  }
       
}

int_pwmOut motorPwmOut(){
  return motorSpeed;
}
