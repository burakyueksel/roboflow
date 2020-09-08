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
import matplotlib.pyplot as plt

def state_plotter(*argv):
    if len(argv)<2:
        print('Warning: state_plotter function requires more than one argument. First argument is always time.')
    elif len(argv)<3:
        fig = plt.figure()
        ax = fig.add_subplot(111)
        h1, = ax.plot(argv[0], argv[1])
        ax.legend([h1], ["arg 1"])
        ax.set_xlabel("time")
        ax.grid()
        fig.suptitle('tim vs arg1', fontsize=16)
    elif len(argv)<4:
        fig = plt.figure()
        ax = fig.add_subplot(111)
        h1, = ax.plot(argv[0], argv[1])
        h2, = ax.plot(argv[0], argv[2])
        ax.legend([h1, h2], ["arg 1", "arg 2"])
        ax.set_xlabel("time")
        ax.grid()
        fig.suptitle('tim vs arg1 and arg2', fontsize=16)
    elif len(argv)<5:
        fig = plt.figure()
        ax = fig.add_subplot(111)
        h1, = ax.plot(argv[0], argv[1])
        h2, = ax.plot(argv[0], argv[2])
        h3, = ax.plot(argv[0], argv[3])
        ax.legend([h1, h2, h3], ["arg 1", "arg 2", "arg3"])
        ax.set_xlabel("time")
        ax.grid()
        fig.suptitle('tim vs arg1 and arg2 and arg3', fontsize=16)
    axes = plt.axes()
    axes.set_ylim([-1, 200])
    plt.show()