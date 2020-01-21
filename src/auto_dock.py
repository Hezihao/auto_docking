#!/usr/bin/env python

import rospy
import time
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion

from nav_msgs.msg import Odometry
from neo_charger.srv import auto_docking
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import TransformStamped, PoseStamped, Vector3, Twist

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
		self.rate = rospy.Rate(15)
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
				# rotate the coordinate system of marker
				marker_correction = TransformStamped()
				marker_correction.header.stamp = rospy.Time.now()
				marker_correction.header.frame_id = 'map'
				marker_correction.child_frame_id = 'ar_marker_10'
				marker_correction.transform.rotation.x = 0.707
				marker_correction.transform.rotation.y = 0
				marker_correction.transform.rotation.z = 0
				marker_correction.transform.rotation.w = 0.707
				# do calibration
				marker_in_map = tf2_geometry_msgs.do_transform_pose(self.marker_pose, transform)
				marker_corrected = tf2_geometry_msgs.do_transform_pose(marker_in_map, marker_correction)
				marker_corrected.pose.position = marker_in_map.pose.position
				self.marker_pose_calibrated = marker_corrected
				#self.ar_pose_corrected_pub.publish(self.marker_pose)
	
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
		print(self.diff_x, self.diff_y, self.diff_theta)

	# execute the first phase of docking process
	def auto_docking(self):
		self.vel = Twist()
		# calculate the velocity needed for docking
		time_waited = time.time() - self.start
		# reset timer if the visual servo failed
		if(abs(self.diff_x) < 0.85 or time_waited > 20):
			self.vel.linear.x = 0
		else:
			self.vel.linear.x = self.kp_x * self.diff_x
		if(abs(self.diff_y) < 0.005 or time_waited > 20):
			self.vel.linear.y = 0
		else:
			self.vel.linear.y = self.kp_y * self.diff_y
			if abs(self.vel.linear.y) < 0.10:
				self.vel.linear.y = 0.10 * np.sign(self.vel.linear.y)
		if(abs(np.degrees(self.diff_theta)) < 0.03 or time_waited > 30):
			self.vel.angular.z = 0
		# filter out shakes from AR tracking
		elif(abs(np.degrees(self.diff_theta)) > 45):
			self.vel.angular.z = 0.005 * np.sign(self.diff_theta)
		else:
			self.vel.angular.z = self.kp_theta * self.diff_theta
			if(abs(self.vel.angular.z) < 0.02):
				self.vel.angular.z = 0.02 * np.sign(self.vel.angular.z)
		self.state = self.vel.linear.x + self.vel.linear.y + self.vel.angular.z
		print(self.vel)
		self.vel_pub.publish(self.vel)
		print(time_waited)
		# check if the process is done
		if(self.state == 0): 
			print("start visual servo.")
			self.visual_servo()

	# second phase of docking, directly using image
	def visual_servo(self):
		kp_x = 1.0
		kp_y = 3.0
		vel = Twist()
		# in case the 2nd docking process failed
		if(abs(np.degrees(self.diff_theta) > 5)):
			self.start = time.time()
		# won't adjust vel.linear.x and vel.linear.y at the same time,
		# to avoid causing hardware damage
		if(abs(self.marker_pose.pose.position.y) > 0.005):
			vel.linear.y = kp_y * self.marker_pose.pose.position.y
			if abs(vel.linear.y) < 0.10:
				vel.linear.y = 0.15 * np.sign(vel.linear.y)
			vel.linear.x = 0
		else:
			vel.linear.y = 0
			if(self.marker_pose.pose.position.x - 0.54 > 0.008):
				vel.linear.x = kp_x * (self.marker_pose.pose.position.x - 0.5)
				if(vel.linear.x < 0.2):
					vel.linear.x = 0.2
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