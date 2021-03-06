import numpy as np
import arena_config as arena
from enum import Enum
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
		self.__robot_list = []
		self.__map_rects = []
		self.__map_obs = []
		self.__map_obs_circles = []
		self.__map_init()


	def __map_init(self):

		for obj in Map.__map_obj_list:
			self.__map_matrix[obj[0]:obj[0]+obj[3], obj[1]:obj[1]+obj[2]] = obj[4].value
			self.__map_rects.append((
				# span in x,y
				math.floor(obj[0]/self.__map_resolution),
				math.floor((obj[0]+obj[3])/self.__map_resolution),
				math.floor(obj[1]/self.__map_resolution),
				math.floor((obj[1]+obj[2])/self.__map_resolution),
				# type
				obj[4].value,
				))
			if obj[4] == MapType.NOGO:
				self.__map_obs.append((
					# box center & radius
					(math.floor(obj[0]+obj[3]/2),math.floor(obj[1]+obj[2]/2)),
					math.ceil(math.sqrt(obj[3]**2+obj[2]**2)/2),
					# box vex
					(obj[0],obj[1]),
					(obj[0],obj[1]+obj[2]),
					(obj[0]+obj[3],obj[1]),
					(obj[0]+obj[3],obj[1]+obj[2])
					))
				self.__map_obs_circles.append(self.__obs_to_circles(obj))

	def __obs_to_circles(self, obj):
		# hard code alert
		radius = math.ceil(arena.robot_width*math.sqrt(2)/2)
		# laid down obs
		if obj[2] > obj[3]:
			circles = [
				(obj[0], 	 obj[1],radius),
				(obj[0], 	 obj[1]+250,radius),
				(obj[0], 	 obj[1]+500,radius),
				(obj[0], 	 obj[1]+750,radius),
				(obj[0], 	 obj[1]+1000,radius),
				(obj[0]+250, obj[1],radius),
				(obj[0]+250, obj[1]+250,radius),
				(obj[0]+250, obj[1]+500,radius),
				(obj[0]+250, obj[1]+750,radius),
				(obj[0]+250, obj[1]+1000,radius),
			]
		else:
			circles = [
				(obj[0], 	 obj[1],radius),
				(obj[0]+250, 	 obj[1],radius),
				(obj[0]+500, 	 obj[1],radius),
				(obj[0]+750, 	 obj[1],radius),
				(obj[0]+1000, 	 obj[1],radius),
				(obj[0]+0, obj[1]+250,radius),
				(obj[0]+250, obj[1]+250,radius),
				(obj[0]+500, obj[1]+250,radius),
				(obj[0]+750, obj[1]+250,radius),
				(obj[0]+1000, obj[1]+250,radius),
			]
		return circles

	def get_x_cnt(self):
		return math.floor(self.__map_x_cnt/self.__map_resolution)

	def get_y_cnt(self):
		return math.floor(self.__map_y_cnt/self.__map_resolution)

	def get_resolution(self):
		return self.__map_resolution

	def isValid(self,x,y,angle):
		return True

	def get_cell(self, x, y):
		return self.__map_matrix[x*self.__map_resolution][y*self.__map_resolution]

	def add_robot(self, robot):
		self.__robot_list.append(robot)

	def draw_map(self,img):
		img.fill(155)
		for rect in self.__map_rects:
			img[rect[0]:rect[1], rect[2]:rect[3]] = Map.__colorMap[rect[4]]
		for robot in self.__robot_list:
			self.__draw_robot(img,robot)
		return

	def __draw_robot(self, img, robot):
		angle = math.radians(robot.angle)
		v4 = (robot.x, robot.y)
		v5 = (robot.x-(robot.length/2)*math.cos(angle),robot.y+(robot.length/2)*math.sin(angle))
		v4 = (math.floor(v4[1]/self.__map_resolution), math.floor(v4[0]/self.__map_resolution))
		v5 = (math.floor(v5[1]/self.__map_resolution), math.floor(v5[0]/self.__map_resolution))

		v0 = (math.floor(robot.v0[1]/self.__map_resolution), math.floor(robot.v0[0]/self.__map_resolution))
		v1 = (math.floor(robot.v1[1]/self.__map_resolution), math.floor(robot.v1[0]/self.__map_resolution))
		v2 = (math.floor(robot.v2[1]/self.__map_resolution), math.floor(robot.v2[0]/self.__map_resolution))
		v3 = (math.floor(robot.v3[1]/self.__map_resolution), math.floor(robot.v3[0]/self.__map_resolution))

		cv.line(img, v0, v1, Map.__colorMap[8],2)
		cv.line(img, v0, v2, Map.__colorMap[8],2)
		cv.line(img, v1, v3, Map.__colorMap[8],2)
		cv.line(img, v2, v3, Map.__colorMap[8],2)
		cv.line(img, v4, v5, Map.__colorMap[8],2)
		return

	def is_valid(self, robot):
		# check if out of map first
		if robot.v0[0]<0 or robot.v0[0]>arena.arena_width: return False
		if robot.v0[1]<0 or robot.v0[1]>arena.arena_length: return False
		if robot.v1[0]<0 or robot.v1[0]>arena.arena_width: return False
		if robot.v1[1]<0 or robot.v1[1]>arena.arena_length: return False
		if robot.v2[0]<0 or robot.v2[0]>arena.arena_width: return False
		if robot.v2[1]<0 or robot.v2[1]>arena.arena_length: return False
		if robot.v3[0]<0 or robot.v3[0]>arena.arena_width: return False
		if robot.v3[1]<0 or robot.v3[1]>arena.arena_length: return False

		for i in range(len(self.__map_obs)):
			obs=self.__map_obs[i]

			# check distance first
			tempDist = math.floor(math.sqrt(((robot.x-obs[0][0])**2 + (robot.y-obs[0][1])**2)))
			if tempDist > (robot.radius + obs[1]):
				continue

			circles=self.__map_obs_circles[i]
			for circle in circles:
				tempDist = math.floor(math.sqrt(((robot.x-circle[0])**2 + (robot.y-circle[1])**2)))
				if tempDist <= circle[2]:
					return False
		return True
			# bad
			# parametrize each edge of the robot
			# pm1 = ParamLine (robot.v0,robot.v1)
			# pm2 = ParamLine (robot.v1,robot.v2)
			# pm3 = ParamLine (robot.v2,robot.v3)
			# pm4 = ParamLine (robot.v0,robot.v3)
			# for i in range(2,2+4):
			# 	if (pm1.is_below(obs[i]) and pm2.is_below(obs[i]) and pm3.is_below(obs[i]) and pm4.is_below(obs[i])):
			# 		continue
			# 	if (not pm1.is_below(obs[i])) and (not pm2.is_below(obs[i])) and (not pm3.is_below(obs[i])) and (not pm4.is_below(obs[i])):
			# 		continue
			# 	return False

		#return True

class ParamLine:
	def __init__(self, p1, p2):
		x1 = p1[0]
		x2 = p2[0]
		y1 = p1[1]
		y2 = p2[1]

		# ay + bx + c = 0
		if x1==x2:
			self.a = 0.0
			self.b = 1.0
			self.c = -x1/1.0
		else:
			self.a = 1.0
			self.b = -(y2-y1)/(x2-x1)
			self.c = -(y1+self.b*x1)

	def is_below(self,p):
		r = self.a*p[1] + self.b*p[0] + self.c
		return (r<0)

class MapRobot:
	def __init__(self, map):
		self.x = 500
		self.y = 500
		self.angle = 0
		self.length = arena.robot_length
		self.width = arena.robot_width
		self.speed = 30
		self.turn_rate = 2

		self.map = map
		self.radius = math.ceil(math.sqrt(self.x**2+self.y**2)/2)

		self.__update_vex()
		self.map.add_robot(self)

	def __update_vex(self):
		resolution = self.map.get_resolution()
		# calculate vertices of the self
		angle = math.radians(self.angle)
		v0 = (self.x-(self.length/2)*math.cos(angle)-(self.width/2)*math.sin(angle),
			  self.y+(self.length/2)*math.sin(angle)-(self.width/2)*math.cos(angle))
		v1 = (v0[0]+math.sin(angle)*self.width,v0[1]+math.cos(angle)*self.width)
		v2 = (v0[0]+math.cos(angle)*self.length,v0[1]-math.sin(angle)*self.length)
		v3 = (v2[0]+math.sin(angle)*self.width,v2[1]+math.cos(angle)*self.width)
		self.v0=v0
		self.v1=v1
		self.v2=v2
		self.v3=v3
		#self.v0 = (math.floor(v0[0]/resolution), math.floor(v0[1]/resolution))
		#self.v1 = (math.floor(v1[0]/resolution), math.floor(v1[1]/resolution))
		#self.v2 = (math.floor(v2[0]/resolution), math.floor(v2[1]/resolution))
		#self.v3 = (math.floor(v3[0]/resolution), math.floor(v3[1]/resolution))

	def move(self, direction):
		tempx = self.x
		tempy = self.y
		temp_angle = self.angle
		if (direction == ord('w')):
			self.x -= math.cos(math.radians(self.angle)) * self.speed
			self.y += math.sin(math.radians(self.angle)) * self.speed
		if (direction == ord('s')):
			self.x += math.cos(math.radians(self.angle)) * self.speed
			self.y -= math.sin(math.radians(self.angle)) * self.speed
		if (direction == ord('a')):
			self.x -= math.sin(math.radians(self.angle)) * self.speed
			self.y -= math.cos(math.radians(self.angle)) * self.speed
		if (direction == ord('d')):
			self.x += math.sin(math.radians(self.angle)) * self.speed
			self.y += math.cos(math.radians(self.angle)) * self.speed

		if (direction == ord('q')):
			self.angle = (self.angle - self.turn_rate) % 360
		if (direction == ord('e')):
			self.angle = (self.angle + self.turn_rate) % 360

		self.__update_vex()

		if not self.map.is_valid(self):
			self.x=tempx
			self.y=tempy
			self.angle=temp_angle
			self.__update_vex()
			return False

		return True


