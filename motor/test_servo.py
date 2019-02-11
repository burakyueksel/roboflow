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
import emotor as em

""" define my servo """
# set point [angle, rotVel]
my_servo    = em.emotor('servo',1,0.14,[5,0])
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