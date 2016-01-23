## PID controller
A python implementation of a simple PID controller. Probably I will implement it in Java and C++ too in the future.

The class implements a PID controller, however, should you decide to use either a PI or PD controller just set the Kd or Ki parameter, respectively, to zero.

Implemented methods:
- output: returns the calculated PID output given a measurement of the y signal.
- antiWindUp: applies anti-wind up to the PID integral block. Maximum thresholds can be set when initializing the controller.
- transferFunction: returns a transfer function for further block integration or frequency domain analysis.

### Examples
Check the file controllerPID_test.py for a simple test.

**PID control set point and noise rejection plot**

![figure_3](https://cloud.githubusercontent.com/assets/13961654/12533129/a4a38332-c226-11e5-9469-d2969181c1a1.png)

**Frequency response plot of a real PID (not an ideal one)**

![image2](https://cloud.githubusercontent.com/assets/13961654/12492639/7b4f73fc-c081-11e5-9ab2-2e1267b49967.png)

### License
GNU GPL 3.0 check the license file for more information
