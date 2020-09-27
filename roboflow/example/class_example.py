class emotor(object):
    """ An electric motor, i.e. emotor has the followning properties:
        Attributes:
            type            : a string representing the type of the motor
            momentOfInertia : moment of inertia [kg.m^2]
            mass            : mass [kg]
    """ 
    def __init__(self,type,mass,momentOfInertia):
        """ return type of the motor """
        self.type = type
        self.mass = mass
        self.momentOfInertia = momentOfInertia
    def set_mass(self,mass):
        """ return mass of the motor """
        self.mass = mass
    def set_momentOfInertia(self,momentOfInertia):
        """ return moment of inertia of the motor """
        self.momentOfInertia = momentOfInertia

my_motor  = emotor('bldc',1,0.014)
print my_motor.momentOfInertia