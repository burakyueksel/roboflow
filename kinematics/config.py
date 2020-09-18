#!/usr/bin/env python
"create n joint robot and test kinematics"
# The kinematic tree starts with a joint on the base.
# Then a link is attached to it.
# That link is connected to another joint.
# This goes on like that.
# n = 0 is the base joint, whose position is known in the global frame.
joint0= dict(axis = "x", pos = [0,0,0])
link1 = dict(mass = 1.0, length = 1.0)
joint1= dict(axis = "z", pos = [0,0,link1['length']])
link2 = dict(mass = 1.0, length = 1.0)
joint2= dict(axis = "z", pos = [link2['length'],0,0])