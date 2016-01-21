## PIDcontroller
A python implementation of a simple PID controller. Probably I will implement it in Java and C++ too in the future.

The class implements a PID controller, however, should you decide to use either a PI or PD controller just set the Kd or Ki parameter, respectively, to zero.

Implemented methods:
- output: returns the calculated PID output given a measurement of the y signal.
- antiWindUp: applies anti-wind up to the PID integral block. Maximum thresholds can be set when initializing the controller.
- transferFunction: returns a transfer function for further block integration or frequency domain analysis.

### Examples
Check the file controllerPID_test.py for a simple test.

### License
GNU GPL 3.0
Check the license file for more information
