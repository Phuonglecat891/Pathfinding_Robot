import math

class Map:
	def __init__(self, row, col, start, goal):
		# 0: norm, 1: obstacle, 2: start, 3: goal, 4: wall, 5: path, 6: pick up
		# create 2d array
		self._map = [[0 for x in range(col)] for y in range(row)]
		self.row = row
		self.col = col

		#set wall
		self.wall = list()
		for i in range(col):
			self._map[0][i] = 4
			self.wall.append((0, i))
			self._map[row - 1][i] = 4
			self.wall.append((row - 1, i))
		for i in range(row):
			self._map[i][0] = 4
			self.wall.append((i, 0))
			self._map[i][col - 1] = 4
			self.wall.append((i, col - 1))

		# set start and goal
		self.start = start
		self.goal = goal
		self._map[self.start[0]][self.start[1]] = 2
		self._map[self.goal[0]][self.goal[1]] = 3

		self.obstacleList = list()
		self.pathList = list ()
		self.pickupPoints = list()

	def addStart(self, point):
		self.start = point
		self._map[point[0]][point[1]] = 2

	def addGoal(self, point):
		self.goal = point
		self._map[point[0]][point[1]] = 3
    
	def addObstacle(self, pointsList):
		for i in pointsList:
			self.obstacleList.append(i)
			self._map[i[0]][i[1]] = 1

	def moveLeftObstacle(self):
		A = []
		for i in self.obstacleList:
			self._map[i[0]][i[1]] = 0
			if i[1] == 1:
				self._map[i[0]][self.col - 2] = 1
			else:
				self._map[i[0]][i[1] - 1] = 1
			A.append(i)
		while len(self.obstacleList) > 0:
			self.obstacleList.pop()
		for i in A:
			if i[1] == 1:
				self.addObstacle([(i[0], self.col - 2)])
			else:
				self.addObstacle([(i[0], i[1] - 1)])

	def moveRightObstacle(self):
		A = []
		for i in self.obstacleList:
			self._map[i[0]][i[1]] = 0
			if i[1] == self.col - 2:
				self._map[i[0]][1] = 1
			else:
				self._map[i[0]][i[1] + 1] = 1
			A.append(i)
		while len(self.obstacleList) > 0:
			self.obstacleList.pop()
		for i in A:
			if i[1] == self.col-2:
				self.addObstacle([(i[0], 1)])
			else:
				self.addObstacle([(i[0], i[1] + 1)])

	def moveUpObstacle(self):
		A = []
		for i in self.obstacleList:
			self._map[i[0]][i[1]] = 0
			if i[0] == 1:
				self._map[self.row-2][i[1]] = 1
			else:
				self._map[i[0]-1][i[1]] = 1
			A.append(i)
		while len(self.obstacleList) > 0:
			self.obstacleList.pop()
		for i in A:
			if i[0] == 1:
				self.addObstacle([(self.row-2,i[1])])
			else:
				self.addObstacle([(i[0]-1, i[1])])

	def moveDownObstacle(self):
		A = []
		for i in self.obstacleList:
			self._map[i[0]][i[1]] = 0
			if i[0] == self.row - 2:
				self._map[1][i[1]] = 1
			else:
				self._map[i[0] + 1][i[1]] = 1
			A.append(i)
		while len(self.obstacleList) > 0:
			self.obstacleList.pop()
		for i in A:
			if i[0] == self.row - 2:
				self.addObstacle([(1,i[1])])
			else:
				self.addObstacle([(i[0] + 1, i[1])])		

	def addPath(self, pointsList):
		for i in pointsList:
			self.pathList.append(i)
			self._map[i[0]][i[1]] = 5

	def dePath(self, pointsList):
		for i in pointsList:
			for index, item in enumerate(self.pathList):
				if i == item:
					self.pathList.pop(index)
					self._map[i[0]][i[1]] = 0

	def addPickupPoint(self, pointsList):
		for i in pointsList:
			self.pickupPoints.append(i)
			self._map[i[0]][i[1]] = 6

	def deStartAndPickup(self, pointsList):
		for i in pointsList:
			for index, item in enumerate(self.pickupPoints):
				if i == item:
					self.pickupPoints.pop(index)
					self._map[i[0]][i[1]] = 0
			if i == self.start:
				self._map[self.start[0]][self.start[1]] = 0
				

	def printMap(self):
		for i in self._map:
			print(i)

	def isBlocked(self, x, y):
		if (x,y) in self.obstacleList:
			return True
		if (x, y) in self.wall:
			return True
		if x < 0 or y < 0 or x > self.row - 1 or y > self.col - 1:
			return True
		return False