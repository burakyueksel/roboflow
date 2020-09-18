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
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

def throws():
    raise RuntimeError('Error, ending.')

def rotZ(alpha):
    "yaw"
    c, s    = np.cos(alpha), np.sin(alpha)
    Rz      = np.array(((c, -s, 0), (s, c, 0), (0, 0, 1)))
    return Rz

def rotY(beta):
    "pitch"
    c, s    = np.cos(beta), np.sin(beta)
    Ry      = np.array(((c, 0, s), (0, 1, 0), (-s, 0, c)))
    return Ry

def rotX(gamma):
    "roll"
    c, s    = np.cos(gamma), np.sin(gamma)
    Rx      = np.array(((1, 0, 0), (0, c, -s), (0, s, c)))
    return Rx

def rotation3(alpha,beta,gamma):
    "3d rotation matrix -alibi- to be used for rotating e.g. a vector"
    R = rotZ(alpha) @ rotY(beta) @ rotX(gamma)
    return R

def rot3(alpha,beta,gamma):
    "3d rotation matrix -alibi- to be used for rotating e.g. a vector"
    "explicit implementation"
    ca, sa  = np.cos(alpha), np.sin(alpha)
    cb, sb  = np.cos(beta), np.sin(beta)
    cg, sg  = np.cos(gamma), np.sin(gamma)
    R       = np.array((    (ca*cb, ca*sb*sg - sa*cg, ca*sb*cg + sa*sg),\
                            (sa*cb, sa*sb*sg + ca*cg, sa*sb*cg - ca*sg),\
                            (-sb,   cb*sg,            cb*cg)))
    return R

def trans4 (angle,axis,pos):
    R = np.identity(3, dtype = float) 
    if      (axis == "x"):
        R = rotX(angle)
    elif    (axis == "y"):
        R = rotY(angle)
    elif    (axis == "z"):
        R = rotZ(angle)
    else:
        throws()
    pos_rot = R @ pos
    T1  = np.c_[R,pos_rot]
    bot = np.array((0,0,0,1))
    T   = np.r_[T1,bot.reshape(1,4)]
    return T

def getPos(T):
    row_idx = np.array([0, 1, 2])
    col_idx = 3
    #col_idx = np.array([0, 2])
    pos = T[row_idx[:, None], col_idx]
    return pos

def getRot(T):
    row_idx = np.array([0, 1, 2])
    col_idx = np.array([0, 1, 2])
    R = T[row_idx[:, None], col_idx]
    return R

def getEuler(R):
    "https://www.gregslabaugh.net/publications/euler.pdf"


def plot_2link_3D (pos1, pos2):
    # x,y,z values of positions of joint 1 and 2 are in separated lists (for plotting)
    pos1x_s = np.squeeze([item[0] for item in pos1])
    pos1y_s = np.squeeze([item[1] for item in pos1])
    pos1z_s = np.squeeze([item[2] for item in pos1])
    pos2x_s = np.squeeze([item[0] for item in pos2])
    pos2y_s = np.squeeze([item[1] for item in pos2])
    pos2z_s = np.squeeze([item[2] for item in pos2])
    fig = plt.figure()
    ax  = plt.axes(projection='3d')
    ax.scatter3D(0,0,0)
    ax.scatter3D(pos1x_s, pos1y_s, pos1z_s)
    ax.scatter3D(pos2x_s, pos2y_s, pos2z_s)
    for i in range(len(pos1)):
        ax.plot([0, pos1x_s[i]],[0, pos1y_s[i]],[0, pos1z_s[i]], 'Black')
        ax.plot([pos1x_s[i], pos2x_s[i]],[pos1y_s[i], pos2y_s[i]],[pos1z_s[i], pos2z_s[i]], 'Gray')
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    plt.show()