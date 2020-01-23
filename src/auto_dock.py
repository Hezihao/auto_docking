#!/usr/bin/env python

import time
import rospy
import tf2_ros
import numpy as np
import tf2_geometry_msgs
from nav_msgs.msg import Odometry
from neo_charger.srv import auto_docking
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import TransformStamped, PoseStamped, Vector3, Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class docking:
	# initialization
	def __init__(self):
		# initial values of parameters
		self.kp_x = 0.3
		self.kp_y = 1.5
		self.kp_theta = 0.5
		self.state = 1
		self.start = 0
		self.SERVICE_CALLED = False
		self.base_pose = PoseStamped()
		self.marker_pose_calibrated = PoseStamped()
		self.base_marker_diff = PoseStamped()
		# initializing node, subscribers, publishers and servcer
		rospy.init_node('auto_docking')
		self.rate = rospy.Rate(30)
		self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
		listener = tf2_ros.TransformListener(self.tf_buffer)
		odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
		marker_pose_sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.marker_pose_calibration)
		server = rospy.Service('auto_docking', auto_docking, self.service_callback)
		rospy.loginfo("auto_docking service is ready.")
		self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
		self.ar_pose_corrected_pub = rospy.Publisher('ar_pose_corrected', PoseStamped, queue_size=1)

	# callback function of odom_sub
	def odom_callback(self, odom):
		# restore measured odom info from nav_msgs/Odometry into geometry_msgs/PoseStamped format
		self.base_pose.header = odom.header
		self.base_pose.pose = odom.pose.pose

	# establish the rotation matrix from euler angle
	def mat_from_euler(self, euler):
		alpha = euler[0]
		beta = euler[1]
		gamma = euler[2]
		sa = np.sin(alpha)		# wrt x-axis
		ca = np.cos(alpha)
		sb = np.sin(beta)		# wrt y-axis
		cb = np.cos(beta)
		sr = np.sin(gamma)		# wrt z-axis
		cr = np.cos(gamma)
		mat = [[cb*cr, sa*sb*cr - ca*sr, ca*sb*cr + sa*sr], [cb*sr, sa*sb*sr + ca*cr, ca*sb*sr - sa*cr], [-sb, sa*cb, ca*cb]]
		return mat

	# transform measured marker pose into something comparable with robot coordinate system
	def marker_pose_calibration(self, ar_markers):
		for mkr in ar_markers.markers:
			if(mkr.id == 10):
				# read pose data of the predefined marker
				self.marker_pose = mkr.pose
				self.marker_pose.header.frame_id = 'camera_link'
				# correct the published orientation of marker
				# in convinience of calculating the error of orientation between marker & base_link
				transform = self.tf_buffer.lookup_transform('map', 'camera_link', rospy.Time(0), rospy.Duration(1.0))
				# do calibration
				marker_in_map = tf2_geometry_msgs.do_transform_pose(self.marker_pose, transform)
				# do the rotation with euler rotation matrix
				marker_in_map_euler = euler_from_quaternion([marker_in_map.pose.orientation.x, marker_in_map.pose.orientation.y, marker_in_map.pose.orientation.z, marker_in_map.pose.orientation.w])
				marker_in_map_mat = self.mat_from_euler(marker_in_map_euler)
				y_axis_of_map = [[0], [1], [0]]
				axis_of_correction = np.dot(marker_in_map_mat, y_axis_of_map)
				correction_quaternion = np.zeros(4)
				correction_quaternion[0] = np.sin(0.785398)*axis_of_correction[0]
				correction_quaternion[1] = np.sin(0.785398)*axis_of_correction[1]
				correction_quaternion[2] = np.sin(0.785398)*axis_of_correction[2]
				correction_quaternion[3] = np.cos(0.785398)
				# calculate transformation with built-in function
				marker_correction = TransformStamped()
				marker_correction.header.stamp = rospy.Time.now()
				marker_correction.header.frame_id = 'map'
				marker_correction.transform.rotation.x = correction_quaternion[0]
				marker_correction.transform.rotation.y = correction_quaternion[1]
				marker_correction.transform.rotation.z = correction_quaternion[2]
				marker_correction.transform.rotation.w = correction_quaternion[3]
				marker_corrected = tf2_geometry_msgs.do_transform_pose(marker_in_map, marker_correction)
				marker_corrected.pose.position = marker_in_map.pose.position
				# pass the information to global
				self.marker_pose_calibrated = marker_corrected
	
	# calculating displacement between marker and robot			
	def calculate_diff(self):
		# remove the error due to displacement between map-frame and odom-frame
		map_to_base = self.tf_buffer.lookup_transform('base_link', 'map', rospy.Time(0), rospy.Duration(1.0))
		odom_map_diff = self.tf_buffer.lookup_transform('map', 'odom', rospy.Time(0), rospy.Duration(1.0))
		self.ar_pose_corrected_pub.publish(self.marker_pose_calibrated)
		# compute difference between marker and base_link
		self.base_pose = tf2_geometry_msgs.do_transform_pose(self.base_pose, odom_map_diff)
		self.base_pose.header.frame_id = 'map'
		self.base_marker_diff.header.stamp = rospy.Time.now()
		self.base_marker_diff.header.frame_id = 'map'
		self.base_marker_diff.pose.position.x = self.marker_pose_calibrated.pose.position.x - self.base_pose.pose.position.x
		self.base_marker_diff.pose.position.y = self.marker_pose_calibrated.pose.position.y - self.base_pose.pose.position.y
		map_to_base.transform.translation = Vector3()
		self.base_marker_diff = tf2_geometry_msgs.do_transform_pose(self.base_marker_diff, map_to_base)
		marker_pose_calibrated_euler = euler_from_quaternion([self.marker_pose_calibrated.pose.orientation.x, self.marker_pose_calibrated.pose.orientation.y, self.marker_pose_calibrated.pose.orientation.z, self.marker_pose_calibrated.pose.orientation.w])
		base_pose_euler = euler_from_quaternion([self.base_pose.pose.orientation.x, self.base_pose.pose.orientation.y, self.base_pose.pose.orientation.z, self.base_pose.pose.orientation.w])
		# calculate the difference
		self.diff_x = self.base_marker_diff.pose.position.x
		self.diff_y = self.base_marker_diff.pose.position.y
		self.diff_theta = marker_pose_calibrated_euler[2]-base_pose_euler[2]
		if(abs(self.diff_theta) > np.pi):
			self.diff_theta = self.diff_theta + np.sign(-self.diff_theta)*(2*np.pi)
		print(self.diff_x, self.diff_y, np.degrees(self.diff_theta))
		
	# execute the first phase of docking process
	def auto_docking(self):
		self.vel = Twist()
		# calculate the velocity needed for docking
		time_waited = time.time() - self.start
		# drive robot to where we start the visual servo process
		# visual servo would remove the error on x & y
		if(abs(self.diff_x) < 0.85 or time_waited > 20):
			self.vel.linear.x = 0
		else:
			self.vel.linear.x = self.kp_x * self.diff_x
		if(abs(self.diff_y) < 0.005 or time_waited > 20):
			self.vel.linear.y = 0
		else:
			self.vel.linear.y = self.kp_y * self.diff_y
			# defining the minimal cmd_vel on y-direction
			if abs(self.vel.linear.y) < 0.15:
				self.vel.linear.y = 0.15 * np.sign(self.vel.linear.y)
		if(abs(np.degrees(self.diff_theta)) < 0.03 or time_waited > 30):
			self.vel.angular.z = 0
		# filter out shakes from AR tracking package
		elif(abs(np.degrees(self.diff_theta)) > 65):
			self.vel.angular.z = 0.005 * np.sign(self.diff_theta)
		else:
			self.vel.angular.z = self.kp_theta * self.diff_theta
			if(abs(self.vel.angular.z) < 0.02):
				self.vel.angular.z = 0.02 * np.sign(self.vel.angular.z)
		self.state = self.vel.linear.x + self.vel.linear.y + self.vel.angular.z
		# check if the 1st phase of docking is done
		if(self.state == 0): 
			print("start visual servo.")
			self.visual_servo()
		else:
			self.vel_pub.publish(self.vel)

	# second phase of docking, directly using visual information
	def visual_servo(self):
		kp_x = 0.5
		kp_y = 3.0
		vel = Twist()
		# in case the 2nd docking process failed
		if(abs(np.degrees(self.diff_theta) > 5)):
			self.start = time.time()
		# won't adjust vel.linear.x and vel.linear.y at the same time,
		# to avoid causing hardware damage
		if(abs(self.marker_pose.pose.position.y) > 0.003):
			vel.linear.y = kp_y * self.marker_pose.pose.position.y
			if abs(vel.linear.y) < 0.1:
				vel.linear.y = 0.1 * np.sign(vel.linear.y)
			vel.linear.x = 0
		else:
			vel.linear.y = 0
			if(self.marker_pose.pose.position.x - 0.54 > 0.01):
				vel.linear.x = kp_x * (self.marker_pose.pose.position.x - 0.54)
				if(vel.linear.x < 0.15):
					vel.linear.x = 0.15
			else:
				vel.linear.x = 0
				vel.linear.y = 0
		self.vel_pub.publish(vel)
		# check if the process is done
		if(not (vel.linear.x + vel.linear.y)):
			self.SERVICE_CALLED = False
			print("Connection established.")

	# the callback function of service auto_docking
	def service_callback(self, auto_docking):
		self.SERVICE_CALLED = True
		self.start = time.time()
		print("Service request received.")
		return "Service requested."

if __name__ == '__main__':
	my_docking = docking()
	while(not rospy.is_shutdown()):
		# start docking when service is called
		if(my_docking.SERVICE_CALLED):
			# make sure marker is detected
			if(my_docking.marker_pose_calibrated.pose.position.x):
				my_docking.calculate_diff()
				my_docking.auto_docking()
		my_docking.rate.sleep()