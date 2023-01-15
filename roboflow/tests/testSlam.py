#!/usr/bin/env python
"""
__author__     = "Burak Yueksel"
__copyright__  = "Copyright 2020, The Roboflow Project"
__credits__    = ["Burak Yueksel"]
__license__    = "GPL"
__version__    = "0.0.0"
__maintainer__ = "Burak Yueksel"
__email__      = "mail.burakyuksel@gmail.com"
__status__     = "Development"
__description__= "Test SW for slam algorithms"
"""
from roboflow.slam.utils import *

print(__file__ + " start!!")

'''
for both Fast Slam 2 and EKF Slam:
'''

time = 0.0

# RFID positions [x, y]
RFID = np.array([[10.0, -2.0],
                    [15.0, 10.0],
                    [15.0, 15.0],
                    [10.0, 20.0],
                    [3.0, 15.0],
                    [-5.0, 20.0],
                    [-5.0, 5.0],
                    [-10.0, 15.0]
                    ])
n_landmark = RFID.shape[0]

# State Vector [x y yaw v]'
xEst = np.zeros((STATE_SIZE, 1))  # SLAM estimation
xTrue = np.zeros((STATE_SIZE, 1))  # True state
xDR = np.zeros((STATE_SIZE, 1))  # Dead reckoning

# history
hxEst = xEst
hxTrue = xTrue
hxDR = xTrue

show_animation = True

'''
For EKF Slam only:
'''

PEst = np.eye(STATE_SIZE)

'''
For Fast Slam 2 only:

'''
particles = [Particle(n_landmark) for _ in range(N_PARTICLE)]


#fastSlam2Run(time, RFID, xTrue, xDR, particles, hxEst, hxDR, hxTrue, show_animation)

ekfSlamRun(time, RFID, xTrue, xDR, xEst, PEst, hxEst, hxDR, hxTrue, show_animation)