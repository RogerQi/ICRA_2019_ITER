import numpy as np
import arena_config as arena
from arena_config import *
import cv2 as cv
import math

class MapResolution(Enum):
	MM  	= 1
	CM	 	= 10
	DM	 	= 100

class MapType(Enum):
	GO 			= 0
	NOGO		= 1

	ST_RED		= 2
	ST_BLUE 	= 3

	DEF_RED 	= 4
	DEF_BLUE	= 5

	SPL_RED 	= 6
	SPL_BLUE 	= 7

class MapRobot:
	def __init__(self):
		self.x = 500
		self.y = 500
		self.angle = 0
		self.length = arena.robot_length
		self.width = arena.robot_width
		self.speed = 30
		self.turn_rate = 2

class Map:

	__map_obj_list = [
		(0,0,1000,1000,MapType.ST_BLUE),								#
		(arena.arena_width-1000,0,1000,1000,MapType.ST_RED), 					#
		(1000+arena.obstacle_width,1200,1000,1000,MapType.DEF_BLUE),	#
		(0,3500,1000,1000,MapType.SPL_RED),
		(arena.arena_width-1000,3500,1000,1000,MapType.SPL_BLUE),
		(0,arena.arena_length-1000,1000,1000,MapType.ST_BLUE),
		(arena.arena_width-1000,arena.arena_length-1000,1000,1000,MapType.ST_RED),
		(arena.arena_width-1000-1250, arena.arena_length-2200, 1000, 1000, MapType.DEF_RED),

		#obstacles
		(1000, 1200, arena.obstacle_length, arena.obstacle_width, MapType.NOGO),
		(arena.arena_width-2400,1400, arena.obstacle_width, arena.obstacle_length, MapType.NOGO),
		(2375, 3500, arena.obstacle_length, arena.obstacle_width, MapType.NOGO),
		(0,4500, arena.obstacle_width, arena.obstacle_length, MapType.NOGO),
		(arena.arena_width-1000,3250,arena.obstacle_width, arena.obstacle_length, MapType.NOGO),
		(1400, arena.arena_length-1650, arena.obstacle_width, arena.obstacle_length, MapType.NOGO),
		(arena.arena_width-1250, arena.arena_length-2200, arena.obstacle_length, arena.obstacle_width, MapType.NOGO)
	]

	__colorMap = [
    	(155,155,155),
    	(55,55,55),
    	(255,155,155),
    	(155,155,255),
    	(255,75, 75),
    	(75,75,255),
    	(255,0,0),
    	(0,0,255),
    	(75,255,75)
	]

	def __init__(self, resolution):
		self.__map_resolution = resolution.value
		self.__map_x_cnt = arena.arena_width
		self.__map_y_cnt = arena.arena_length
		self.__map_matrix = np.zeros((self.__map_x_cnt, self.__map_y_cnt))
		self.__map_init()


	def __map_init(self):
		self.__map_rects = []
		for obj in Map.__map_obj_list:
			self.__map_matrix[obj[0]:obj[0]+obj[3], obj[1]:obj[1]+obj[2]] = obj[4].value
			self.__map_rects.append(
				(math.floor(obj[0]/self.__map_resolution),
				 math.floor((obj[0]+obj[3])/self.__map_resolution),
				 math.floor(obj[1]/self.__map_resolution),
				 math.floor((obj[1]+obj[2])/self.__map_resolution),
				 obj[4].value)
				)

	def get_x_cnt(self):
		return math.floor(self.__map_x_cnt/self.__map_resolution)

	def get_y_cnt(self):
		return math.floor(self.__map_y_cnt/self.__map_resolution)

	def move(self, robot, direction):
		tempx = robot.x
		tempy = robot.y
		temp_angle = robot.angle
		if (direction == ord('w')):
			tempx -= math.cos(math.radians(robot.angle)) * robot.speed
			tempy += math.sin(math.radians(robot.angle)) * robot.speed
		if (direction == ord('s')):
			tempx += math.cos(math.radians(robot.angle)) * robot.speed
			tempy -= math.sin(math.radians(robot.angle)) * robot.speed
		if (direction == ord('a')):
			tempx -= math.sin(math.radians(robot.angle)) * robot.speed
			tempy -= math.cos(math.radians(robot.angle)) * robot.speed
		if (direction == ord('d')):
			tempx += math.sin(math.radians(robot.angle)) * robot.speed
			tempy += math.cos(math.radians(robot.angle)) * robot.speed

		if (direction == ord('q')):
			temp_angle = (robot.angle - robot.turn_rate) % 360
		if (direction == ord('e')):
			temp_angle = (robot.angle + robot.turn_rate) % 360

		if self.isValid(tempx, tempy, temp_angle):
			robot.x=tempx
			robot.y=tempy
			robot.angle=temp_angle
			return True

		return False

	def isValid(self,x,y,angle):
		return True

	def get_cell(self, x, y):
		return self.__map_matrix[x*self.__map_resolution][y*self.__map_resolution]

	def draw_map(self,img):
		img.fill(155)
		for rect in self.__map_rects:
			img[rect[0]:rect[1], rect[2]:rect[3]] = Map.__colorMap[rect[4]]
		return

	def draw_robot(self, img, robot):
		# calculate vertices of the robot
		angle = math.radians(robot.angle)
		v0 = (robot.x-(robot.length/2)*math.cos(angle)-(robot.width/2)*math.sin(angle),
			  robot.y+(robot.length/2)*math.sin(angle)-(robot.width/2)*math.cos(angle))
		v1 = (v0[0]+math.sin(angle)*robot.width,v0[1]+math.cos(angle)*robot.width)
		v2 = (v0[0]+math.cos(angle)*robot.length,v0[1]-math.sin(angle)*robot.length)
		v3 = (v2[0]+math.sin(angle)*robot.width,v2[1]+math.cos(angle)*robot.width)
		v4 = (robot.x, robot.y)
		v5 = (robot.x-(robot.length/2)*math.cos(angle),robot.y+(robot.length/2)*math.sin(angle))

		v0 = (math.floor(v0[1]/self.__map_resolution), math.floor(v0[0]/self.__map_resolution))
		v1 = (math.floor(v1[1]/self.__map_resolution), math.floor(v1[0]/self.__map_resolution))
		v2 = (math.floor(v2[1]/self.__map_resolution), math.floor(v2[0]/self.__map_resolution))
		v3 = (math.floor(v3[1]/self.__map_resolution), math.floor(v3[0]/self.__map_resolution))
		v4 = (math.floor(v4[1]/self.__map_resolution), math.floor(v4[0]/self.__map_resolution))
		v5 = (math.floor(v5[1]/self.__map_resolution), math.floor(v5[0]/self.__map_resolution))

		cv.line(img, v0, v1, Map.__colorMap[8],2)
		cv.line(img, v0, v2, Map.__colorMap[8],2)
		cv.line(img, v1, v3, Map.__colorMap[8],2)
		cv.line(img, v2, v3, Map.__colorMap[8],2)
		cv.line(img, v4, v5, Map.__colorMap[8],2)
		return


