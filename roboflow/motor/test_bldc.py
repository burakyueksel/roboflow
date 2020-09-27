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

""" define my bldc """
my_bldc     = emotor('bldc',1,0.14,100)
print(my_bldc.type)

""" max sim time """
tmax        = 20.0
t           = np.linspace(0.0, tmax, 1000)

""" initiate the state """
rotVel0     = 0

""" uncontrolled dynamics in ode """
u           = 0
args        = (u,)
y_bldc_unctrl                   = sp.integrate.odeint(my_bldc.open_dynamics, rotVel0, t, args)



""" pi controlled dynamics in ode """
error_init                      = 0
y_bldc_pictrl                   = sp.integrate.odeint(my_bldc.pi_controlled_dynamics, [error_init, rotVel0], t)
error_integral                  = y_bldc_pictrl[:,0]
velocities                      = y_bldc_pictrl[:,1]
accelerations                   = my_bldc.pi_controlled_dynamics
u_bldc_pi_ctrl, e_bldc_pi_ctrl  = my_bldc.pi_ctrl([error_integral, velocities],my_bldc.setPoint)
state_plotter(t,error_integral,velocities,u_bldc_pi_ctrl)


'''

""" pi (antiwindup) controlled dynamics in ode """
error_init                      = 0
y_bldc_pictrl                   = sp.integrate.odeint(my_bldc.pi_aw_controlled_dynamics, [error_init, rotVel0], t)
error_integral                  = y_bldc_pictrl[:,0]
velocities                      = y_bldc_pictrl[:,1]
accelerations                   = my_bldc.pi_aw_controlled_dynamics
u_bldc_pi_aw_ctrl, e_bldc_pi_aw_ctrl = my_bldc.pi_ctrl_aw([error_integral, velocities],my_bldc.setPoint)
state_plotter(t,error_integral,velocities,u_bldc_pi_aw_ctrl)

'''

'''
# P CONTROL. PLOT velocity, contorl inputs and the kinetic energy.
""" p controlled dynamics in ode """
y_bldc_pctrl                    = sp.integrate.odeint(my_bldc.p_controlled_dynamics, rotVel0, t)

""" kinetic energy """
kinetic_bldc_pctrl              = my_bldc.kinetic_energy(y_bldc_pctrl)

""" get the motor control input """
u_bldc_pctrl, u_bldc_pctrl_norm = my_bldc.p_ctrl(y_bldc_pctrl,my_bldc.setPoint)

""" plot results """
state_plotter(t,y_bldc_pctrl,u_bldc_pctrl_norm,kinetic_bldc_pctrl)
'''