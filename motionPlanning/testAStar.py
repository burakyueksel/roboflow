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
__description__= "Test SW for AStar motion planning algorithm"
"""
from utils import *
# define:
# start and goal positions,
# grid size
# robot radius
sPosX       = 10.0 # [m]
sPosY       = 10.0 # [m]
gPosX       = 50.0 # [m]
gPosY       = 50.0 # [m]
gridSize    = 2.0  # [m]
robRad      = 1.0  # [m]

# animate
showAnimation = True
# obstacles
obsX, obsY = [], []
for i in range (-10, 60):
    obsX.append(i)
    obsY.append(-10.0)
for i in range (-10,60):
    obsX.append(60.0)
    obsY.append(i)

if showAnimation:
    plt.plot (obsX, obsY, '.k')
    plt.plot (sPosX,sPosY,'og')
    plt.plot (gPosX,gPosY,'xb')
    plt.grid (True)
    plt.axis ('equal')

aStar = AStar(obsX, obsY, gridSize, robRad)
posX, posY = aStar.planning (sPosX, sPosY, gPosX, gPosY)

if showAnimation:
    plt.plot (posX, posX, '-r')
    plt.pause(0.001)
    plt.show()