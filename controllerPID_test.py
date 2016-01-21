# -*- coding: utf-8 -*-
"""
@date: 21-01-16
@author: Michy
@title: controllerPID_test.py
@description:

    PID controller test

@version: 0.0.1
@required extra libraries: numpy,matplotlib,scipy
@license: GNU GPL 3.0

    Copyright 2015 Michy

    this software is distributed under the GNU GPL license

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

@other:

"""

################################################################################
#                                 PID TEST
################################################################################

import time
import numpy as np
import control.matlab as mtl
import matplotlib.pyplot as plt
from scipy.interpolate import spline

plt.style.use('ggplot')

# Instanciate PID object
pid = PID(1.2,1,0.001)

# Set y variable initial value
y = 0

test_noise = 2

# Initialize lists for recording data
y_ = []
t_ = []
setpoints_ = []
outs_ = []
noise_ = []

# Control loop (finite loop)
for i in range(1,150):
    
    # Calculate PID output
    output = pid.output(y)
    
    # Update y
    y += output
    
    # Set unit setpoint at i = 10    
    if i == 10:
        pid.set_point = 1
        noise_.append(0)
    # Setpoint gets changed back to zero at i = 50
    elif i == 50:
        pid.set_point = 0
        noise_.append(0)
    # Check noise rejection
    elif i >= 100 and i <= 103:
        y += test_noise
        noise_.append(test_noise)
    else:
        noise_.append(0)
    
    # Record data
    y_.append(y)
    setpoints_.append(pid.set_point)
    t_.append(i)
    outs_.append(output)
    
    # Let some time pass
    time.sleep(0.01)

# Smooth out rough edges
t_sm = np.linspace(min(t_),max(t_),400)
y_sm = spline(t_,y_,t_sm)
outs_sm = spline(t_,outs_,t_sm)
fig = plt.figure()
plt.subplot(2,1,1)
plt.plot(t_sm,y_sm,label='y')
plt.plot(t_,setpoints_,label='Setpoint')
plt.grid(True)
plt.xlim([0,max(t_)])
plt.title("PID controller on y variable")
plt.legend()
plt.subplot(2,1,2)
plt.plot(t_sm,outs_sm,label='u',color='red')
plt.plot(t_,noise_,label='Noise',color='black')
plt.grid(True)
plt.xlim([0,max(t_)])
plt.ylim([-3,test_noise+1])
plt.title('Controlled variable u and noise')
plt.legend()
plt.show()

################################################################################
#                               End PID test
################################################################################


################################################################################
#                               Transfer function test
################################################################################
# Open loop control system
# u -> R -> G -> y

# Process transfer function G(s)
G = mtl.tf([1,1],[2,1,1])

# PID Controller R(s)
R = pid.transferFunction()

# Open loop system transfer function
sys = R * G
print('System transfer function',sys)

# Bode plot
mtl.bode(sys)

################################################################################
#                           End transfer function test
################################################################################
