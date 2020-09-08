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
""" init simulation characteristics """
tmax                = 20.0
time                = np.linspace(0.0, tmax, 1000)
""" init motor characteristics"""
motor_type          = 'servo'
motor_mass          = 1.0 # kg
motor_moi           = 0.14# moment of inertia
""" first motor """
motor_0_set_state   = [3,0]   # [angle, rotVel], [rad, rad/s]. These set values can be coming from a high-level controller.
motor_0             = emotor(motor_type,motor_mass,motor_moi,motor_0_set_state)
""" second motor """
motor_1_set_state   = [1,0]   # [angle, rotVel], [rad, rad/s]. These set values can be coming from a high-level controller.
motor_1             = emotor(motor_type,motor_mass,motor_moi,motor_1_set_state)
""" init state """
motor_0_angle0      = 0
motor_0_rotVel0     = 0
motor_1_angle0      = 0
motor_1_rotVel0     = 0
""" first motor pd controlled dynamics in ode """
motor_0_output      = sp.integrate.odeint(motor_0.pd_controlled_dynamics_ext, [motor_0_angle0, motor_0_rotVel0], time)
motor_0_angle       = motor_0_output[:,0]
motor_0_rotVel      = motor_0_output[:,1]
motor_0_kinetic     = motor_0.kinetic_energy(motor_0_rotVel)
""" second motor pd controlled dynamics in ode """
motor_1_output      = sp.integrate.odeint(motor_1.pd_controlled_dynamics_ext, [motor_1_angle0, motor_1_rotVel0], time)
motor_1_angle       = motor_1_output[:,0]
motor_1_rotVel      = motor_1_output[:,1]
motor_1_kinetic     = motor_1.kinetic_energy(motor_1_rotVel)
""" plotter """
state_plotter(time,motor_0_angle,motor_0_rotVel,motor_0_kinetic)
state_plotter(time,motor_1_angle,motor_1_rotVel,motor_1_kinetic)
state_plotter(time,motor_0_angle+motor_1_angle,motor_0_rotVel+motor_1_rotVel,motor_0_kinetic+motor_1_kinetic)