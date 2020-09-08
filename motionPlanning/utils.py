'''

Utils for path and trajectory planning
@author: Burak Yueksel 

'''

# import libraries
import math
import random
import matplotlib.pyplot as plt
import numpy as np

def str2bool(v):

	return str(v).lower() in ("True", "true", "t", "1")

def getPathLength2D(path):
    '''
    get the length of a path on a plane
    '''
    length = 0
    for i in range(len(path) - 1):
        x 	 	= path[i + 1][0] - path[i][0]
        y 	 	= path[i + 1][1] - path[i][1]
        dist 	= math.sqrt(x**2 + y**2)
        length += dist

    return length

def getPoint2PointDistance2D(point1, point2):
	'''
	get the distance between point1 and point2
	on a plane (x,y).
	'''
	dx      = point1[0] - point2[0]
	dy      = point1[1] - point2[1]
	return math.sqrt(dx**2 + dy**2)

def getPoint2PointDistance3D(point1, point2):
	'''
	get the distance between point1 and point2
	in 3D (x,y,z).
	'''
	dx      = point1.x - point2.x
	dy      = point1.y - point2.y
	dz      = point1.y - point2.y
	return math.sqrt(dx**2 + dy**2 + dz**2)

def checkLineCollision(point1, point2, obstacleList):
	'''
	check if a line between 2 points are colliding with obstacles
	'''
	point1_x = point1[0]
	point1_y = point1[1]
	point2_x = point2[0]
	point2_y = point2[1]

	try:
		alpha =   point2_y-point1_y
		beta  = -(point2_x - point1_x)
		gamma =   point2_y*(point2_x-point1_x) - point2_x*(point2_y-point1_y)
	except ZeroDivisionError:
		print('WARNING: Points are too closed to eachother or they are identical.')
		return False

	for (obst_x, obst_y, size) in obstacleList:
		dist_to_obst = abs(alpha*obst_x + beta*obst_y + gamma)/ (math.sqrt(alpha**2 + beta**2))
		if dist_to_obst <= size:
			return False # Collision
	
	return True # No collision

def getTargetPoint(path,targetLength):
	# init the length and the indice
	length 		 = 0
	targetIndice = 0
	lastLength   = 0

	for i in range(len(path)-1):
		deltaX = path[i+1][0] - path[i][0]
		deltaY = path[i+1][1] - path[i][1]
		delta  = math.sqrt(deltaX**2 + deltaY**2)
		length += delta
		if length>=targetLength:
			targetIndice = i-1
			lastLength   = delta
			break
	ratio = (length-targetLength)/lastLength

	x = path[targetIndice][0] + (path[targetIndice+1][0] - path[targetIndice][0]) * ratio
	y = path[targetIndice][1] + (path[targetIndice+1][1] - path[targetIndice][1]) * ratio

	# return the points and target indice
	return [x,y,targetIndice]

def pathSmooth(path, maxIter, obstacleList):
    lengthPath = getPathLength2D(path)

    for _ in range(maxIter):
        # Sample two points
        randomPoints	= [random.uniform(0, lengthPath), random.uniform(0, lengthPath)]
        randomPoints.sort()
        firstPoint		= getTargetPoint(path, randomPoints[0])
        secondPoint 	= getTargetPoint(path, randomPoints[1])

        if firstPoint[2] <= 0 or secondPoint[2] <= 0:
            continue

        if (secondPoint[2] + 1) > len(path):
            continue

        if secondPoint[2] == firstPoint[2]:
            continue

        # collision check
        if not checkLineCollision(firstPoint, secondPoint, obstacleList):
            continue

        # Create New path
        newPath = []
        newPath.extend(path[:firstPoint[2] + 1])
        newPath.append([firstPoint[0], firstPoint[1]])
        newPath.append([secondPoint[0], secondPoint[1]])
        newPath.extend(path[secondPoint[2] + 1:])
        path = newPath
        lengthPath = getPathLength2D(path)

    return path

class AStar:
	''' 
	class for AStar in 2D
	'''
	class Node:
		'''
		Class for Node
		'''
		def __init__(self, x, y, cost, parentIndex):
			self.x 				= x
			self.y 				= y
			self.cost			= cost
			self.parentIndex 	= parentIndex

		def __str__(self):
			return str(self.x) + "," + str(self.y) + "," + str(
				self.cost) + "," + str(self.parent_index)
	def __init__(self, obsX, obsY, res, robRad):
		'''
		init the grid map
		obsX  : X coordinates of the obstacles
		obsY  : Y coordinates of the obstacles
		res   : Grid Resolution
		robRad: Robot radius
		'''
		self.res 	= res
		self.robRad = robRad
		self.minX   = 0.0
		self.minY   = 0.0
		self.maxX   = 0.0
		self.maxY	= 0.0
		self.obstMap= None
		self.widthX = 0.0
		self.widthY = 0.0
		self.motion = self.getMotion()
		self.calcObstMap(obsX, obsY)

	def planning(self, startX, startY, goalX, goalY):
		'''
		AStar planning
		'''
		startNode = self.Node(	self.calcXYIndex(startX,self.minX),
								self.calcXYIndex(startY,self.minY), 0.0, -1)
		goalNode  = self.Node(	self.calcXYIndex(goalX,self.minX),
								self.calcXYIndex(goalX,self.minY), 0.0, -1)
		openSet = dict()
		closeSet= dict()


class RRT2D:
	'''
	class for RRT in 2D
	'''
	class Node:
		'''
		class for node of RRT2d
		'''
		def __init__(self,x,y):
			'''
			init the node
			'''
			self.x 		= x
			self.y 		= y
			self.path_x	= []
			self.path_y = []
			self.parent = None
	def __init__(self,start,goal,obstacleList,randArea,expandDist,
				pathResolution,goalSampleRate,maxIter):
		'''
		init the RRT2D
		'''
		self.start			= self.Node(start[0],start[1])
		self.goal			= self.Node(goal[0],goal[1])
		self.obstacleList 	= obstacleList
		self.minRand		= randArea[0]
		self.maxRand		= randArea[1]
		self.expandDist 	= expandDist
		self.pathResolution = pathResolution
		self.goalSampleRate = goalSampleRate
		self.maxIter		= maxIter
		self.nodeList 		= []

	def planning (self, animation):
		'''
		planning of RRT2D
		animation true or false
		'''
		self.nodeList = [self.start]
		for i in range(self.maxIter):
			randomNode	= self.getRandomNode()
			nearestInd	= self.getNearestNodeIndex(self.nodeList,randomNode)
			nearestNode	= self.nodeList[nearestInd]
			newNode		= self.steerNode(nearestNode,randomNode,self.expandDist)

			if not self.checkCollision(newNode,self.obstacleList): # no collision occurs
				self.nodeList.append(newNode)

			if animation and i % 5 == 0:
				self.drawGraph(randomNode)

			if self.calcDistToGoal(self.nodeList[-1].x, self.nodeList[-1].y) <= self.expandDist:
				finalNode = self.steerNode(self.nodeList[-1], self.goal, self.expandDist)
				if not self.checkCollision(finalNode,self.obstacleList): # no collision occurs
					return self.generatePathToGoal(len(self.nodeList)-1)

			if animation and i % 5:
				self.drawGraph(randomNode)
			
		return None # no path is found/planned



	def getRandomNode(self):
		'''
		generate a random node
		'''
		if random.randint(0,100) > self.goalSampleRate:
			randomNode = self.Node(random.uniform(self.minRand, self.maxRand),
								   random.uniform(self.minRand, self.maxRand))
		else:
			randomNode = self.Node(self.goal.x, self.goal.y)
		
		return randomNode

	def steerNode(self, fromNode, toNode, expandDist=float("inf")):
		'''
		compute the paths from node to node
		'''
		# from node is the new node
		newNode 	= self.Node(fromNode.x, fromNode.y)
		# compute the distance and the angle between the from node and the to node
		dist, angle = self.calcDistAngle(newNode, toNode)
		# init the path of the new node
		newNode.path_x = [newNode.x]
		newNode.path_y = [newNode.y]
		# expanding distance is limited to the distance between the from node and to node
		if expandDist > dist:
			expandDist = dist
		# get the number of steps in the path
		stepExpand = math.floor(expandDist/self.pathResolution)
		# compute the path of the new node (it starts with the from node)
		for _ in range(stepExpand):
			newNode.x += self.pathResolution * math.cos(angle)
			newNode.y += self.pathResolution * math.sin(angle)
			newNode.path_x.append(newNode.x)
			newNode.path_y.append(newNode.y)
		# check for the last steps that have fewer distance to the to node. Add them to the path
		dist, _ = self.calcDistAngle(newNode,toNode)
		if dist <= self.pathResolution:
			newNode.path_x.append(toNode.x)
			newNode.path_y.append(toNode.y)

		newNode.parent = fromNode

		return newNode


	def drawGraph(self, random=None):
		'''
		plot a graph
		feature used for animations
		'''
		plt.clf()
		# possibility for stopping the animation with esc key
		plt.gcf().canvas.mpl_connect('key_release_event', lambda event: [exit(0) if event.key == 'escape' else None])
		# plot differently if random is not None
		if random is not None:
			plt.plot(random.x, random.y, "^k")
		for node in self.nodeList:
			if node.parent:
				plt.plot(node.path_x, node.path_y, '-g')
		
		for (obstX, obstY, size) in self.obstacleList:
			self.plotCircle(obstX, obstY, size)

		plt.plot(self.start.x, self.start.y, 'x')
		plt.plot(self.goal.x, self.goal.y, 'x')
		plt.axis("equal")
		plt.axis([self.minRand, self.maxRand, self.minRand, self.maxRand])
		plt.grid(True)
		plt.pause(0.01)

	def drawFinalPath(self, path):
		self.drawGraph()
		plt.plot([x for (x,y) in path], [y for (x,y) in path], '-r')
		plt.grid(True)
		plt.pause(0.1)
		plt.show()

	def drawFinalAndSmoothedPath(self, path, smoothedPath):
		self.drawGraph()
		plt.plot([x for (x,y) in path], [y for (x,y) in path], '-r')
		plt.plot([x for (x,y) in smoothedPath], [y for (x,y) in smoothedPath], '-c')
		plt.grid(True)
		plt.pause(0.1)
		plt.show()

	def drawFinalAndSmoothedPath2(self, path, smoothedPath, smoothedPath2):
		self.drawGraph()
		plt.plot([x for (x,y) in path], [y for (x,y) in path], '-r')
		plt.plot([x for (x,y) in smoothedPath], [y for (x,y) in smoothedPath], '-c')
		plt.plot([x for (x,y) in smoothedPath2], [y for (x,y) in smoothedPath2], '-m')
		plt.grid(True)
		plt.pause(0.1)
		plt.show()
	
	def calcDistToGoal(self, x, y):
		'''
		calculate the Euclidean distance from a point to the goal
		'''
		distX  = x - self.goal.x
		distY  = y - self.goal.y
		return math.sqrt(distX**2 + distY**2)
	
	def generatePathToGoal(self,goalInd):
		'''
		generate the final path to the goal
		'''
		path	= [[self.goal.x, self.goal.y]]
		node 	= self.nodeList[goalInd]
		while node.parent is not None:
			path.append([node.x, node.y])
			node = node.parent
		path.append([node.x, node.y])

		return path

	@staticmethod
	def getNearestNodeIndex(nodeList, nextNode):
		'''
		get the index of the nearest node.
		in RRT, nextNode is computed after a random step, hence it is a random node.
		the nearest node in the so far availble node list to this random node is found here.
		'''
		distAll = [(node.x - nextNode.x)**2 + (node.y - nextNode.y)**2
					for node in nodeList]
		minIndi  = distAll.index(min(distAll))

		return minIndi

	@staticmethod
	def checkCollision(node, obstacleList):
		'''
		check if the node is colliding with any known obstacle
		returns true if collision occurs
		returns false otherwise
		'''
		if node is None:
			return False
		for (obsX, obsY, size) in obstacleList:
			dxAll	= [obsX - x for x in node.path_x]
			dyAll	= [obsY - y for y in node.path_y]
			distAll= [dx**2 + dy**2 for (dx,dy) in zip(dxAll, dyAll)]

			if min(distAll) <= size**2:
				return True # collision

		return False # no collision

	@staticmethod
	def calcDistAngle(fromNode, toNode):
		'''
		Simply compute the distance and the angle (on plane) between two points
		'''
		distX = toNode.x - fromNode.x
		distY = toNode.y - fromNode.y
		dist  = math.sqrt(distX**2 + distY**2)
		angle = math.atan2(distY, distX)

		return dist, angle

	@staticmethod
	def plotCircle(x,y,r,color='-y'):
		'''
		draw circle with center point (x,y) and radius (r)
		'''
		deg = list(range(0,360,5))
		deg.append(0)
		# compute the points of the circle in xy plane
		cx  = [x + r*math.cos(np.deg2rad(d)) for d in deg]
		cy  = [y + r*math.sin(np.deg2rad(d)) for d in deg]
		# plot the circle
		plt.plot(cx, cy, color)
