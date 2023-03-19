import os
import socket
import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray
import threading
import time
import numpy as np
import math
from scene import *
from ros_connector import ROSConnector


def get_disp(self):
	return max(self.cx - self.rx, 0), min(self.cx + self.rx, self.width), max(
		self.cy - self.ry, 0), min(self.cy + self.ry, self.height)


def get_disp_xy(self, x, y):
	x0, x1, y0, y1 = self.get_disp()
	_x = (x - self.origin_x) / self.resolution
	_y = (y - self.origin_y) / self.resolution
	return 4.0 * (_x - x0), 4.0 * (_y - y0)


def get_map_xy(self, x, y):
	x0, x1, y0, y1 = self.get_disp()
	_x = (x / 4.0 + x0) * self.resolution + self.origin_x
	_y = (y / 4.0 + y0) * self.resolution + self.origin_y
	return _x, _y


def get_theta(o):
	return 2.0 * math.atan2(o.z, o.w)


def trans(self, x, y, pose0):
	th = get_theta(pose0.orientation) - 90.0 / 4.0 * math.pi / 180.0
	c = math.cos(th)
	s = math.sin(th)
	_x = c * x - s * y + pose0.position.x
	_y = s * x + c * y + pose0.position.y
	return _x, _y


class MapViewer(ROSConnector):

	def __init__(self, master_uri='http://192.168.0.2:11311', node_name='map_viewer'):
		super().__init__(master_uri, node_name)

		self.goal_pub = rospy.Publisher(
			"/move_base_simple/goal", PoseStamped, queue_size=10)

		rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
		rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_callback)
		rospy.Subscriber("/scan", LaserScan, self.scan_callback)
		rospy.Subscriber("/particlecloud", PoseArray, self.particle_callback)

		self.scene = MyScene(self)

		# Initialize instance variables
		self.goal = [0.0, 0.0]
		self.robot_pose = Pose()
		self.poses = []
		self.M = []
		self.width = 0
		self.height = 0
		self.resolution = 0.01
		self.origin_x = 0.0
		self.origin_y = 0.0
		self.cx = 0
		self.cy = 0
		self.rx = int(375 / 4)
		self.ry = int(665 / 4)
		self.tlock = threading.Lock()
		self.scan_list = []
		self.scan_get_flag = True

	def map_callback(self, data):
		print('map')
		self.tlock.acquire()
		self.width = int(data.info.width / 4)
		self.height = int(data.info.height / 4)
		self.cx = int(self.width / 2)
		self.cy = int(self.height / 2)
		self.resolution = data.info.resolution * 4
		self.origin_x = data.info.origin.position.x
		self.origin_y = data.info.origin.position.y

		for i in range(0, self.width):
			_M = []
			for j in range(0, self.height):
				_M.append(0.0)
			self.M.append(_M)

		for i in range(data.info.width):
			for j in range(data.info.height):
				n = i + j * data.info.width
				self.M[int(i / 4)][int(j / 4)] = max(data.data[n], self.M[int(i / 4)][int(j / 4)])
		self.tlock.release()
		print('map e')

	def pose_callback(self, data):
		print('pose')
		self.tlock.acquire()
		self.robot_pose = data.pose.pose
		self.scan_get_flag = True
		self.tlock.release()
		print('pose e')

	def scan_callback(self, data):
		if self.scan_get_flag:
			print('scan')
			self.scan_get_flag = False
			n = 0
			self.tlock.acquire()
			self.scan_list.clear()
			for theta in np.arange(data.angle_min, data.angle_max, data.angle_increment):
				c = math.cos(-theta)
				s = math.sin(-theta)
				x = c * data.ranges[n]
				y = s * data.ranges[n]
				self.scan_list.append([x, y])
				n += 1
			self.tlock.release()
			print('scan e')

	def particle_callback(self, data):
		print('particle')
		self.tlock.acquire()
		self.poses = data.poses
		self.tlock.release()
		print('particle e')

	def run(self):
		th = threading.Thread(target=rospy.spin)
		th.start()
		run(self.scene, show_fps=False)
		th.join()


class MyScene(Scene):
	def __init__(self, map_viewer):
		super().__init__()
		self.map_viewer = map_viewer

	def setup(self):
		self.x = 0.0
		self.y = 0.0

	def update(self):
		self.map_viewer.tlock.acquire()
		x0, x1, y0, y1 = get_disp(self)
		for y in range(y0, y1):
			for x in range(x0, x1):
				fill(1.0 - self.map_viewer.M[x][y] / 100.0,
									1.0 - self.map_viewer.M[x][y] / 100.0,
									1.0 - self.map_viewer.M[x][y] / 100.0)
				rect(4 * (x - x0), 4 * (y - y0), 4, 4)

		for pose in self.map_viewer.poses:
			dx, dy = get_disp_xy(self, pose.position.x, pose.position.y)
			fill(1, 0, 0)
			ellipse(dx, dy, 2, 2)

		dx, dy = get_disp_xy(self, self.map_viewer.goal[0], self.map_viewer.goal[1])
		fill(0, 0, 1)
		ellipse(dx, dy, 2, 2)

		for scan in self.map_viewer.scan_list:
			x, y = trans(scan[0], scan[1], self.map_viewer.robot_pose)
			dx, dy = get_disp_xy(self, x, y)
			fill(0.5, 0.5, 0)
			ellipse(dx, dy, 2, 2)

		self.map_viewer.tlock.release()

	def touch_began(self, touch):
		if (touch.location.x - self.x)**2 + (touch.location.y - self.y)**2 < 10.0**2:
			x, y = get_map_xy(self, self.x, self.y)
			msg = PoseStamped()
			msg.header.frame_id = 'map'
			msg.pose.position.x = x
			msg.pose.position.y = y
			msg.pose.orientation.w = 1.0
			print('pub')
			self.map_viewer.goal_pub.publish(msg)
			self.map_viewer.goal[0] = x
			self.map_viewer.goal[1] = y
		self.x = touch.location.x
		self.y = touch.location.y

	def touch_moved(self, touch):
		x = touch.location.x
		y = touch.location.y
		self.map_viewer.cx -= int((x - self.x) / 4.0)
		self.map_viewer.cy -= int((y - self.y) / 4.0)
		self.x = x
		self.y = y

	def touch_end(self, touch):
		pass


if __name__ == '__main__':
	map_viewer = MapViewer()
	map_viewer.run()

