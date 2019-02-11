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

class emotor(object):
    """ An electric motor, i.e. emotor has the followning properties:
        Attributes:
            type            : a string representing the type of the motor
            momentOfInertia : moment of inertia [kg.m^2]
            mass            : mass [kg]
            setPoint        : which angle do we want to reach [rad]
    """ 
    def __init__(self,type,mass,momentOfInertia,setPoint):
        """ return attributes of the motor """
        self.type            = type
        self.mass            = mass
        self.momentOfInertia = momentOfInertia
        self.setPoint        = setPoint
    def p_ctrl(self,state,state_d):
        """ p-controller
            state           : current state \in\mathbb{R}
            state_d         : desired state \in\mathbb{R}
            e               : error between desired and current state
            kp              : gain of the controller []
            u               : computed control input [unit depending on the system]
            u_norm          : normalized u with kp
        """
        e                    = state_d-state
        kp                   = 30
        u                    = kp*e
        u_norm               = u/kp
        return u, u_norm
    def pd_ctrl(self,state,state_d):
        """ p-controller
            state           : current state \in\mathbb{R}^2, i.e. [pos vel]
            state_d         : desired state \in\mathbb{R}^2, i.e. [pos_d vel_d]        
            e               : error between desired and current state
            kp              : gain of the controller []
            u               : computed control input [unit depending on the system]
            u_norm          : normalized u with kp
        """
        e                    = state_d[0]-state[0]
        ed                   = state_d[1]-state[1]
        kp                   = 30
        kd                   = 12
        u                    = kp*e + kd*ed
        return u       
    def p_controlled_dynamics(self, rotVel, t):
        """ dynamics of a motor (1st order Euler) 
            t      = time                [s]
            angle  = angle               [rad]
            rotvel = rotational velocity [rad/s]
            u      = input torque        [Nm]
            """
        # domega = self.momentOfInertia*u(t)
        u, u_norm = self.p_ctrl(rotVel,self.setPoint)
        rotAcc    = self.momentOfInertia*u
        return rotAcc
    def open_dynamics(self, rotVel, t, u):
        """ dynamics of a motor (1st order Euler) 
            t      = time                [s]
            angle  = angle               [rad]
            rotVel = rotational velocity [rad/s]
            u      = input torque        [Nm]
            """
        # domega = self.momentOfInertia*u(t)
        rotAcc = self.momentOfInertia*u
        return rotAcc  
    def open_dynamics_ext(self, state, t, u):
        """ dynamics of a motor 
            t      = time                [s]
            angle  = angle               [rad]
            rotVel = rotational velocity [rad/s]
            u      = input torque        [Nm]
            """
        rotVel  = state[1]
        rotAcc  = self.momentOfInertia*u
        return [rotVel, rotAcc] 

    def p_controlled_dynamics_ext(self, state, t):
        """ dynamics of a motor 
            t      = time                [s]
            angle  = angle               [rad]
            rotVel = rotational velocity [rad/s]
            u      = input torque        [Nm]
            """
        u, u_norm = self.p_ctrl(state[0],self.setPoint[0])
        rotVel  = state[1]
        rotAcc  = self.momentOfInertia*u
        return [rotVel, rotAcc] 

    def pd_controlled_dynamics_ext(self, state, t):
        """ dynamics of a motor 
            t      = time                [s]
            angle  = angle               [rad]
            rotVel = rotational velocity [rad/s]
            u      = input torque        [Nm]
            """
        u       = self.pd_ctrl(state,self.setPoint)
        rotVel  = state[1]
        rotAcc  = self.momentOfInertia*u
        return [rotVel, rotAcc]       

    def kinetic_energy(self, rotVel):
        """ kinetic energy of a motor
            rotVel = rotational velocity  [rad/s]
        """
        K = 0.5 * self.momentOfInertia*rotVel*rotVel
        return K        