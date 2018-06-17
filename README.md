# AIO-Quadcopter-Flight-Controller
This device takes data from an inertial measurement unit about it's position and angle of inclination and receives 
information from the remote about the next desired action and makes several calculations to determine the difference
between the desired action, dictated by the remote, and the actual response from the drone, measured by the accelerometer 
and gyroscope. Using this information, the drone can adjust to the desired position by varying the speed of it's motors 
according to calculations from the control loop.

