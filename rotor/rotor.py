
#!/usr/bin/env python
"""
__author__     = "Burak Yueksel"
__copyright__  = "Copyright 2020, The RoboSim Project"
__credits__    = ["Burak Yueksel"]
__license__    = "GPL"
__version__    = "0.0.0"
__maintainer__ = "Burak Yueksel"
__email__      = "mail.burakyuksel@gmail.com"
__status__     = "Development"
"""
import numpy as np

class rotor(object):
    """ A rotor, i.e. rotor has the followning properties:
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
    def rotor_poly3(self,spin_vel,poly3Thrust, poly3Torque):
        """ A polynome based rotor model.
            spin_vel        : spinning velocity of the rotor, e.g. RPM
            poly3Thrust     : second order polynome mapping spin_vel to thrust
            poly3Torque     : second order polynome mapping spin_vel to torque
        """
        thrust               = 0
        torque               = 0
        return thrust, torque