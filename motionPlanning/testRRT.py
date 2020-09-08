from configparser import ConfigParser
from utils import *
import numpy as np

#Read config.ini file
config_object = ConfigParser()
config_object.read("config.ini")

#Get the obstacles
# obstacles = config_object["OBSTACLES"]
# Get the obstacle list
# obstacleList    = [float(i) for i in obstaclesString.split(', ')]
# obstacleList = [float(config_object.items("OBSTACLES")[i][1]) for i in range(len(config_object.items("OBSTACLES")))]
obstacleList = [
    (5, 5, 1),
    (3, 6, 2),
    (3, 8, 2),
    (3, 10, 2),
    (7, 5, 2),
    (9, 5, 2),
    (8, 10, 1),
    (6, 12, 1),
]  # [x,y,size(radius)]
# Get the start and end values
startEnd = config_object["STARTEND"]
#Get the RRT properties
rrtProp     = config_object["RRTPROPERTIES"]
# Set Initial parameters
start   =   [float(i) for i in startEnd["start"].split(', ')]
goal    =   [float(i) for i in startEnd["goal"].split(', ')]
randArea=   [float(i) for i in startEnd["randArea"].split(', ')]
expandDist =float(rrtProp["expandDist"])
pathResolution = float(rrtProp["pathResolution"])
goalSampleRate = float(rrtProp["goalSampleRate"])
maxIteration= int(rrtProp["maxIteration"])
animationFlag = str2bool(rrtProp["animation"])
smoothFlag = str2bool(rrtProp["smoothFlag"])
doubleSmoothFlag = str2bool(rrtProp["doubleSmoothFlag"])

# set the rrt properities
rrt = RRT2D(start,
            goal,
            obstacleList,
            randArea,
            expandDist,
            pathResolution,
            goalSampleRate,
            maxIteration)
# compute the first path (obstacle free way points)
path = rrt.planning(animation=False)
# result
if path is None:
    print("No path is found")
else:
    print("path is found")
    if rrtProp["smoothFlag"]:
        # smooth the path
        smoothedPath = pathSmooth(path,maxIteration,obstacleList)
        rrt.drawFinalAndSmoothedPath(path,smoothedPath)
    elif rrtProp["doubleSmoothFlag"]:
        # double smooth the path
        smoothedPath = pathSmooth(path,maxIteration,obstacleList)
        smoothedPath2 = pathSmooth(smoothedPath,maxIteration,obstacleList)
        rrt.drawFinalAndSmoothedPath2(path, smoothedPath,smoothedPath2)
    else:
        rrt.drawFinalPath(path)