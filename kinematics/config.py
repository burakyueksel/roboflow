#!/usr/bin/env python
joint0= dict(rot = "z", pos = [0,0,0])
link1 = dict(mass = 1.0, length = 1.0)
joint1= dict(rot = "z", pos = [link1['length'],0,0])
link2 = dict(mass = 1.0, length = 1.0)
joint2= dict(rot = "z", pos = [link2['length'],0,0])