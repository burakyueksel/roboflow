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
sPosX       = -25.0 # [m]
sPosY       = -20.0 # [m]
gPosX       = 50.0 # [m]
gPosY       = 50.0 # [m]
gridSize    = 2.0  # [m]
robRad      = 4.0  # [m]

# animate
showAnimation = True
# obstacles
wall_xy_min = -40
wall_xy_max = 60
obsX, obsY = [], []
# left wall
for i in range (wall_xy_min, wall_xy_max):
    obsX.append(i)
    obsY.append(wall_xy_min)
# right wall
for i in range (wall_xy_min,wall_xy_max):
    obsX.append(wall_xy_max)
    obsY.append(i)
# upper wall
for i in range(wall_xy_min, wall_xy_max+1):
    obsX.append(i)
    obsY.append(wall_xy_max)
# lower wall
for i in range(wall_xy_min, wall_xy_max+1):
    obsX.append(wall_xy_min)
    obsY.append(i)
# inner wall
for i in range(wall_xy_min, 20):
    obsX.append(-20.0) # placed at x coordinate -20
    obsY.append(i) # length on y: from bottom to 20
# inner wall
for i in range(0, 60):
    obsX.append(40.0) # place at x coordinate 40
    obsY.append(wall_xy_max - i) # length on y: from top to top-60 

if showAnimation:
    plt.plot (obsX, obsY, '.k')
    plt.plot (sPosX,sPosY,'og')
    plt.plot (gPosX,gPosY,'xb')
    plt.grid (True)
    plt.axis ('equal')

aStar = AStar(obsX, obsY, gridSize, robRad, showAnimation)
tic()
#posX, posY = aStar.planning (sPosX, sPosY, gPosX, gPosY) # astar
posX, posY = aStar.biDirectionalPlanning (sPosX, sPosY, gPosX, gPosY) # bidirectional a star
toc()
if showAnimation:
    plt.plot (posX, posY, '-r')
    plt.pause(0.001)
    plt.show()