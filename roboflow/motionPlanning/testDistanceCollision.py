# import libraries
from utils import *

point2D1 = [0.0, 0.0]
point2D2 = [3.0, 4.0]

obstacleList = [
    (2, 2, 0.5)
]  # [x,y,size]

dist = getPoint2PointDistance2D(point2D1,point2D2)
print(dist)

colCheck = checkLineCollision(point2D1,point2D2,obstacleList)
print(colCheck)