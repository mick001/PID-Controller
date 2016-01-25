# -*- coding: utf-8 -*-
################################################################################
"""
@date: 21-01-16
@author: Michy
@title: controllerPID.py
@description:

    Simple implementation of a PID controller for a closed loop control system.
    As of now by setting either Ki or Kd equal to zero you can use, respectively
    a PD or a PI controller. If you set all three the parameters then you'll be 
    using a PID.
    
    The PID is provided with a raw (to be improved) anti-windup system.

@version: 0.0.1
@required extra libraries: control (for the transfer function only)
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

import time
import control.matlab as mtl

class PID():
    
    def __init__(self,kp = 0.5, ki = 0.1, kd = 0, windupMax=0):
        
        """ Initialize PID object 

            # PID parameters:
            Set PID parameters. By setting some parameters to zero you can get
            a P or PI controller (or even a pure integrator if you wish)
            
            # Set point: Constant value to be reached by the measured variable y
            Set point is set to zero by default
            
            # outValue: last output value returned from the output method
            
            # windupMax: threshold for the anti-windup to kick in. If set to zero
            no anti-windup system is provided. Default value is zero.
        
        """
        
        # PID parameters
        self.kp = kp
        self.ki = ki
        self.kd = kd

        # Set point is set to zero by default
        self.set_point = 0
        
        # PID terms
        self.pTerm = 0
        self.iTerm = 0
        self.dTerm = 0
        
        # Sampling time
        self.sample_time = 0.02
        
        # Timing        
        self.now_time = time.time()
        self.old_time = self.now_time        
        
        # PID last output value returned     
        self.outValue = 0
        
        # Last error
        self.last_error = 0
        
        # Last y measured
        self.last_y = 0
    
        # Anti windup        
        self.windupMax = windupMax
        
    def __repr__(self):
        r = """
        PID controller
        ##############        
        kp %s
        ki %s
        kd %s
        Set point %s
        Sample time %s
        Anti windup limit %s
        """ %(self.kp,self.ki,self.kd,self.set_point,self.sample_time,self.windupMax)
        return r
    
    def output(self,y_measured):

        """ 
            Calculate PID output value 
            
            Formula:

            outValue(t) = kd * e(t) + ki * cumulativesum(e(t) * dt) + kd * de(t)/dt
            
            Note:
            if the set point is constant, then de(t)/dt = -dy/dt
            this little tweak helps us avoiding impulsive terms (spikes) due to 
            the derivative of the error (since the error changes instantly when
            switching the set point, its derivative ends up being infinite).        
        """
        
        # Calculate current error
        error = self.set_point - y_measured
        
        # Get elapsed time
        self.now_time = time.time()        
        dt = self.now_time - self.old_time
        
        # Calculate output
        if dt >= self.sample_time:
            # P term
            self.pTerm = self.kp * error
            # I term            
            self.iTerm += self.ki * error * dt
            # D term
            self.dTerm = self.kd * (self.last_y - y_measured)/dt
            
            # Check for windup problems if anti-windup is enabled
            self.antiWindUp()
            
            # Update variables
            self.last_error = error
            self.old_time = self.now_time
            self.last_y = y_measured
            
            # Output value to be returned
            self.outValue = self.pTerm + self.iTerm + self.dTerm
            
            return self.outValue
        else:
            return 0
    
    def antiWindUp(self):

        """ 
            Anti wind-up 
    
            As far as the anti wind-up system is concerned, I implemented it as 
            it is explained in most control theory books. It only operates on 
            the integral term. It does not cap the proportional and the 
            derivative terms.
        """
        
        if self.windupMax != 0:
            if self.iTerm > self.windupMax:
                self.iTerm = self.windupMax
            elif self.iTerm < -self.windupMax:
                self.iTerm = -self.windupMax
    
    def transferFunction(self,N=15):
        
        """ PID transfer function """        
        
        r = self.kd + self.ki * mtl.tf([1],[1,0]) + self.kd * mtl.tf([1,0],[self.kd/self.kp*N,1])        
        return r

