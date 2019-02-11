#!/usr/bin/env python
"""
__author__     = "Burak Yueksel"
__copyright__  = "Copyright 2018, The RoboSim Project"
__credits__    = ["Burak Yueksel"]
__license__    = "GPL"
__version__    = "0.0.0"
__maintainer__ = "Burak Yueksel"
__email__      = "mail.burakyuksel@gmail.com"
__status__     = "Development"
"""

import matplotlib.pyplot as plt
import numpy as np
import scipy as sp
import scipy.integrate

class emotor(object):
    """ An electric motor, i.e. emotor has the followning properties:
        Attributes:
            type            : a string representing the type of the motor
            momentOfInertia : moment of inertia [kg.m^2]
            mass            : mass [kg]
            setPoint        : which angle do we want to reach [rad]
    """ 
    def __init__(self,type,mass,momentOfInertia,setPoint):
        """ return attributes of the motor """
        self.type            = type
        self.mass            = mass
        self.momentOfInertia = momentOfInertia
        self.setPoint        = setPoint
    def p_ctrl(self,state,state_d):
        """ p-controller
            state           : current state \in\mathbb{R}
            state_d         : desired state \in\mathbb{R}
            e               : error between desired and current state
            kp              : gain of the controller []
            u               : computed control input [unit depending on the system]
            u_norm          : normalized u with kp
        """
        e                    = state_d-state
        kp                   = 30
        u                    = kp*e
        u_norm               = u/kp
        return u, u_norm
    def pd_ctrl(self,state,state_d):
        """ p-controller
            state           : current state \in\mathbb{R}^2, i.e. [pos vel]
            state_d         : desired state \in\mathbb{R}^2, i.e. [pos_d vel_d]        
            e               : error between desired and current state
            kp              : gain of the controller []
            u               : computed control input [unit depending on the system]
            u_norm          : normalized u with kp
        """
        e                    = state_d[0]-state[0]
        ed                   = state_d[1]-state[1]
        kp                   = 30
        kd                   = 12
        u                    = kp*e + kd*ed
        return u       
    def p_controlled_dynamics(self, rotVel, t):
        """ dynamics of a motor (1st order Euler) 
            t      = time                [s]
            angle  = angle               [rad]
            rotvel = rotational velocity [rad/s]
            u      = input torque        [Nm]
            """
        # domega = self.momentOfInertia*u(t)
        u, u_norm = self.p_ctrl(rotVel,self.setPoint)
        rotAcc    = self.momentOfInertia*u
        return rotAcc
    def open_dynamics(self, rotVel, t, u):
        """ dynamics of a motor (1st order Euler) 
            t      = time                [s]
            angle  = angle               [rad]
            rotVel = rotational velocity [rad/s]
            u      = input torque        [Nm]
            """
        # domega = self.momentOfInertia*u(t)
        rotAcc = self.momentOfInertia*u
        return rotAcc  
    def open_dynamics_ext(self, state, t, u):
        """ dynamics of a motor 
            t      = time                [s]
            angle  = angle               [rad]
            rotVel = rotational velocity [rad/s]
            u      = input torque        [Nm]
            """
        rotVel  = state[1]
        rotAcc  = self.momentOfInertia*u
        return [rotVel, rotAcc] 

    def p_controlled_dynamics_ext(self, state, t):
        """ dynamics of a motor 
            t      = time                [s]
            angle  = angle               [rad]
            rotVel = rotational velocity [rad/s]
            u      = input torque        [Nm]
            """
        u, u_norm = self.p_ctrl(state[0],self.setPoint[0])
        rotVel  = state[1]
        rotAcc  = self.momentOfInertia*u
        return [rotVel, rotAcc] 

    def pd_controlled_dynamics_ext(self, state, t):
        """ dynamics of a motor 
            t      = time                [s]
            angle  = angle               [rad]
            rotVel = rotational velocity [rad/s]
            u      = input torque        [Nm]
            """
        u       = self.pd_ctrl(state,self.setPoint)
        rotVel  = state[1]
        rotAcc  = self.momentOfInertia*u
        return [rotVel, rotAcc]       

    def kinetic_energy(self, rotVel):
        """ kinetic energy of a motor
            rotVel = rotational velocity  [rad/s]
        """
        K = 0.5 * self.momentOfInertia*rotVel*rotVel
        return K        

''' TEST BLDC: only rotVel is controllable '''

""" define my bldc """
my_bldc     = emotor('bldc',1,0.14,5)
print my_bldc.type

""" max sim time """
tmax        = 20.0
t           = np.linspace(0.0, tmax, 1000)

""" initiate the state """
rotVel0     = 0

""" uncontrolled dynamics in ode """
u           = 0
args        = (u,)
y_bldc_unctrl                   = sp.integrate.odeint(my_bldc.open_dynamics, rotVel0, t, args)

""" p controlled dynamics in ode """
y_bldc_pctrl                    = sp.integrate.odeint(my_bldc.p_controlled_dynamics, rotVel0, t)

""" kinetic energy """
kinetic_bldc_pctrl              = my_bldc.kinetic_energy(y_bldc_pctrl)

""" get the motor control input """
u_bldc_pctrl, u_bldc_pctrl_norm = my_bldc.p_ctrl(y_bldc_pctrl,my_bldc.setPoint)

""" plot results """
fig = plt.figure()
ax = fig.add_subplot(111)
h1, = ax.plot(t, y_bldc_pctrl)
h2, = ax.plot(t, u_bldc_pctrl_norm)
h3, = ax.plot(t, kinetic_bldc_pctrl)
ax.legend([h1, h2, h3], ["rotVel", "u_norm", "kinetic"])
ax.set_xlabel("t")
ax.grid()
fig.suptitle('P-Controlled BLDC Motor', fontsize=16)
plt.show()

''' TEST SERVO: both angle and rotVel is controllable '''

""" define my servo """
# set point [angle, rotVel]
my_servo    = emotor('servo',1,0.14,[5,0])
print my_servo.type
""" max sim time """
tmax       = 20.0
t          = np.linspace(0.0, tmax, 1000)

""" initiate the state """
angle0     = 1
rotVel0    = 0

""" uncontrolled dynamics extended in ode """
u          = 0
args       = (u,)
y_servo_uctrl          = sp.integrate.odeint(my_servo.open_dynamics_ext, [angle0, rotVel0], t, args)

""" p controlled dynamics in ode """
y_servo_pctrl          = sp.integrate.odeint(my_servo.p_controlled_dynamics_ext, [angle0, rotVel0], t)

""" pd controlled dynamics in ode """
y_servo_pdctrl         = sp.integrate.odeint(my_servo.pd_controlled_dynamics_ext, [angle0, rotVel0], t)

""" get the states """

angle                  = y_servo_pdctrl[:,0]
rotVel                 = y_servo_pdctrl[:,1]

""" kinetic energy """
kinetic_servo_pdctrl              = my_servo.kinetic_energy(rotVel)

""" plot results """
fig = plt.figure()
ax = fig.add_subplot(111)
h1, = ax.plot(t, angle)
h2, = ax.plot(t, rotVel)
h3, = ax.plot(t, kinetic_servo_pdctrl)
ax.legend([h1, h2, h3], ["angle", "rotVel", "kinetic energy"])
ax.set_xlabel("t")
ax.grid()
fig.suptitle('PD-Controlled Servo Motor', fontsize=16)
plt.show()
