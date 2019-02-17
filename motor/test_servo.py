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
import numpy as np
import scipy as sp
import scipy.integrate
from emotor import *
from utils import *
        
""" init motor characteristics"""
motor_type      = 'servo'
motor_mass      = 1.0 # kg
motor_moi       = 0.14# moment of inertia
motor_set_state = [100,0]   # [angle, rotVel], [rad, rad/s]. These set values can be coming from a high-level controller.
my_servo        = emotor(motor_type,motor_mass,motor_moi,motor_set_state)
""" init simulation characteristics """
tmax            = 20.0
time            = np.linspace(0.0, tmax, 1000)
""" init state """
angle0          = 1
rotVel0         = 0
""" uncontrolled dynamics extended in ode """
u               = 0.1 # Control torque. In theory this control input can be coming from another controller.
args            = (u,)
y_servo_unctrl  = sp.integrate.odeint(my_servo.open_dynamics_ext, [angle0, rotVel0], time, args)
angle_unctrl    = y_servo_unctrl[:,0]
rotVel_unctrl   = y_servo_unctrl[:,1]
kinetic_uctrl   = my_servo.kinetic_energy(rotVel_unctrl)
""" p controlled dynamics in ode """
y_servo_pctrl   = sp.integrate.odeint(my_servo.p_controlled_dynamics_ext, [angle0, rotVel0], time)
angle_pctrl     = y_servo_pctrl[:,0]
rotVel_pctrl    = y_servo_pctrl[:,1]
kinetic_pctrl   = my_servo.kinetic_energy(rotVel_pctrl)
""" pd controlled dynamics in ode """
y_servo_pdctrl  = sp.integrate.odeint(my_servo.pd_controlled_dynamics_ext, [angle0, rotVel0], time)
angle_pdctrl    = y_servo_pdctrl[:,0]
rotVel_pdctrl   = y_servo_pdctrl[:,1]
kinetic_pdctrl  = my_servo.kinetic_energy(rotVel_pdctrl)
""" plotter """
state_plotter(time,angle_pctrl,rotVel_pctrl,kinetic_pctrl)