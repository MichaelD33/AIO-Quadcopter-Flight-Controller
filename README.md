# AIO Quadcopter Flight Controller

Follow the continued development of this project on my website:
https://delaney.nyc/projects/

---

### Basic Overview
Flight controller firmware for micro brushed quadcopters based on the atmega32u4 chipset.

This device takes data from an inertial measurement unit about it's position and receives information from the remote about the next desired action and makes several calculations to determine the difference between the desired action, dictated by the remote, and the actual response from the motors, measured by the accelerometer and gyroscope. Using this information, the drone can adjust to the desired position by varying the speed of it's motors according to calculations from the control loop.

---

### Setup and Operation 
Configuration and setup is relatively straight forward; however, all the parameters must be edited within the IDE. Currently, in this early stage of development, the program only supports sBus type receivers and the MPU6050 combined accelerometer and gyroscope.

| Device | MCU Pin Input |
|--------|---------------|
|Frsky XM|      RX1      |
|MPU6050 |   SDA, SCL    |

__PWM Motor Outputs:__
The motor output pins are entirely configurable from within the IDE. Since the flight controller is designed to support brushed motors, the output pins must be PWM capable by default, so the motors may be pulsed continuously while the program runs. 

Determining the correct order of the motor outputs can be quite frustrating and may cause the drone to attempt to move in the wrong direction or pitch when intending to roll... When configured correctly, the bottom left motor should correspond with the first field of the pinout array, bottom left with the second, top left with the third, and top right for the fourth (orientation determined according to the direction of the IMU).
*For the AIO quadcopter PCB the outputs vary for each design, but the proper config is listed in the program main file.*

*processor note: this device requires a serial port for the RC input. The receiver is setup under Serial1 for my atmega32u4. This will not work with an atmega328.*

#### Motor Output Diagram
![alt text](http://delaney.nyc/wp-content/uploads/2018/08/aioFC_outline_annotated.png)

---

### Difficulties
This project originally began as a personal challenge in order to improve my embedded programming skills. While I acknowledged the difficulty of programming a device like this, I also was extremely motivated to learn about the ongoing computational processes that take place in an aerial vehicle like a quadcopter. Throughout the process of designing the hardware and writing the code for the flight controller I continually encountered issues with both the physical electronics and my programming. I am still attempting to fix a number of these issues, many of which emerge from the physical form factor of the device; however, I have been working on designs and form factors which will hopefully help to mitigate the issues I am currently facing.

The micro scale design of the all-in-one(AIO) frame leads to stability issues because of increased sensitivity to:
* Wind
* Vibrations
* PID Tuning
* Changes in CG
* Changes in Mass

---
