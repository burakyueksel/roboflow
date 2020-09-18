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

'''
"test rotation matrix implementations"
alpha = 0# yaw
beta  = 0 # pitch
gamma = 0# roll
R = rotation3(alpha,beta,gamma)
R2= rot3(alpha,beta,gamma)
print(R-R2)

"test homogenous transformation"
T = trans4 (math.pi,"x",np.array([10,0,0]))
print(T)
'''

"create 2 joint robot and test kinematics"
# The kinematic tree starts with a joint on the base.
# Then a link is attached to it.
# That link is connected to another joint.
# This goes on like that.
#start with  base rotation axis
axis0   = config.joint0['axis']
# position of first joint w.r.t base
p01     = config.joint1['pos']
# rotation of first joint
axis1   = config.joint1['axis']
# position of second joint w.r.t. first one.
p12     = config.joint2['pos']
#
# angles of two joints:
ang0    = -math.pi/2
ang1    = math.pi/2
# transformation from base to tip
# p00, rotated with 0, is not considered.
T01 = trans4 (ang0,axis0,p01)
T12 = trans4 (ang1,axis1,p12)
T02 = T01 @ T12
'''
print(T01)
print(T12)
print(T02)
# print the positions
print(getPos(T01))
print(getPos(T02))
'''

ang0_l = np.arange(0,math.pi/2,0.1)
ang1_l = np.arange(0,math.pi/2,0.1)

pos1x = []
pos1y = []
pos1z = []
pos2x = []
pos2y = []
pos2z = []
for ang0 in ang0_l:
    for ang1 in ang1_l:
        T01 = trans4 (ang0,axis0,p01)
        T12 = trans4 (ang1,axis1,p12)
        T02 = T01 @ T12
        pos1x.append(getPos(T01)[0])
        pos1y.append(getPos(T01)[1])
        pos1z.append(getPos(T01)[2])
        pos2x.append(getPos(T02)[0])
        pos2y.append(getPos(T02)[1])
        pos2z.append(getPos(T02)[2])
pos1x_s = np.squeeze(pos1x)
pos1y_s = np.squeeze(pos1y)
pos1z_s = np.squeeze(pos1z)
pos2x_s = np.squeeze(pos2x)
pos2y_s = np.squeeze(pos2y)
pos2z_s = np.squeeze(pos2z)
fig = plt.figure()
ax  = plt.axes(projection='3d')
ax.scatter3D(0,0,0)
ax.scatter3D(pos1x_s, pos1y_s, pos1z_s)
ax.scatter3D(pos2x_s, pos2y_s, pos2z_s)
plt.show()