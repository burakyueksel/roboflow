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

def trans4 (axis,angle,pos):
    R = np.identity(3, dtype = float) 
    if      (axis == "x"):
        R = rotX(angle)
    elif    (axis == "y"):
        R = rotY(angle)
    elif    (axis == "z"):
        R = rotZ(angle)
    else:
        throws()
    T1  = np.c_[R,pos]
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