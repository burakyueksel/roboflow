'''

rrt for path planning

'''
# import libraries
from utils import *

mIter = 1000
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
# Set Initial parameters
rrt = RRT2D(start=[0, 0],
            goal=[6, 10],
            obstacleList=obstacleList,
            randArea=[-20, 15],
            expandDist=3.0, 
            pathResolution=0.5, 
            goalSampleRate=5, 
            maxIter=mIter)
# compute the path
path = rrt.planning(animation=False)
# quick result
if path is None:
    print("No path is found")
else:
    print("path is found")
    # draw the final result
    # rrt.drawFinalPath(path)
    # smooth the path
    smoothedPath = pathSmooth(path,mIter,obstacleList)
    # ploth with smoothed path
    # rrt.drawFinalAndSmoothedPath(path,smoothedPath)
    # double smooth the path
    smoothedPath2 = pathSmooth(smoothedPath,mIter,obstacleList)
    # ploth with smoothed path
    rrt.drawFinalAndSmoothedPath2(path, smoothedPath,smoothedPath2)


