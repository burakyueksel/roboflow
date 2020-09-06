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
import math
from utils import *
import config

"test rotation matrix implementations"
alpha = 0# yaw
beta  = 0 # pitch
gamma = 0# roll
R = rotation3(alpha,beta,gamma)
R2= rot3(alpha,beta,gamma)
print(R-R2)

"test homogenous transformation"
T = trans4 ("x",math.pi,np.array([10,0,0]))
print(T)

"create 2 joint robot and test kinematics"
p01     = config.joint1['pos']
p12     = config.joint2['pos']
ang0    = 0.0
ang1    = math.pi/2
axis0   = "z"
axis1   = "z" 
T01 = trans4 (axis0,ang0,p01)
T12 = trans4 (axis1,ang1,p12)
T02 = T12 @ T01

print(T02)
print(getPos(T02))

print (config.joint1['pos'])