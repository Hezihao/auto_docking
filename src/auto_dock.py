#!/usr/bin/env python

import time
import rospy
import tf2_ros
import numpy as np
import tf2_geometry_msgs
from nav_msgs.msg import Odometry
from neo_docking.srv import auto_docking
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped, PoseStamped, Vector3, Twist

class Docking:
	# initialization
	def __init__(self):
		# initial values of parameters
		self.kp_x = 0.3
		self.kp_y = 1.5
		self.kp_theta = 0.5
		#self.state = 1
		self.start = 0
		self.base_pose = PoseStamped()
		self.marker_pose_calibrated = PoseStamped()
		self.base_marker_diff = PoseStamped()
		# initializing node, subscribers, publishers and servcer
		rospy.init_node('auto_docking')
		self.rate = rospy.Rate(15)
		self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
		listener = tf2_ros.TransformListener(self.tf_buffer)
		odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
		marker_pose_sub = rospy.Subscriber('ar_pose_filtered', PoseStamped, self.pose_callback)
		rospy.loginfo("auto_docking service is ready.")
		self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
		self.ar_pose_corrected_pub = rospy.Publisher('ar_pose_corrected', PoseStamped, queue_size=1)
		# in test
		self.LAST_STEP = False

	# callback function of odom_sub
	def odom_callback(self, odom):
		# restore measured odom info from nav_msgs/Odometry into geometry_msgs/PoseStamped format
		self.base_pose.header = odom.header
		self.base_pose.pose = odom.pose.pose

	# callback function of filtered_pose_sub
	def pose_callback(self, filtered_pose):
		self.marker_pose_calibrated = filtered_pose

	# calculating displacement between marker and robot			
	def calculate_diff(self):
		# remove the error due to displacement between map-frame and odom-frame
		map_to_base = self.tf_buffer.lookup_transform('base_link', 'map', rospy.Time(0), rospy.Duration(1.0))
		odom_map_diff = self.tf_buffer.lookup_transform('map', 'odom', rospy.Time(0), rospy.Duration(1.0))
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
		print("Difference: ["+str(self.diff_x)+", "+str(self.diff_y)+", "+str(np.degrees(self.diff_theta))+"]")
		
	# execute the first phase of docking process
	def auto_docking(self):
		self.vel = Twist()
		# calculate the velocity needed for docking
		time_waited = time.time() - self.start
		# drive robot to where we start the visual servo process
		# visual servo would remove the error on x & y
		'''
		if(abs(self.diff_x) < 0.70 or time_waited > 10):
			self.vel.linear.x = 0
		else:
			self.vel.linear.x = min(max(self.kp_x * self.diff_x, 0.05), 0.1)
		if(abs(self.diff_y) < 0.005 or time_waited > 10):
			self.vel.linear.y = 0
		else:
			self.vel.linear.y = self.kp_y * self.diff_y
			# defining the minimal cmd_vel on y-direction
			if abs(self.vel.linear.y) < 0.03:
				self.vel.linear.y = 0.03 * np.sign(self.vel.linear.y)
		'''
		# in test
		self.vel.linear.x = 0
		self.vel.linear.y = 0
		if(abs(np.degrees(self.diff_theta)) < 0.5 or time_waited > 20):
			self.vel.angular.z = 0
			if(abs(self.diff_y) < 0.005 or time_waited > 10):
				self.vel.linear.y = 0
				if(abs(self.diff_x) < 0.70 or time_waited > 10):
					self.vel.linear.x = 0
				else:
					self.vel.linear.x = min(max(self.kp_x * self.diff_x, 0.05), 0.1)
			else:
				self.vel.linear.y = self.kp_y * self.diff_y
				# defining the minimal cmd_vel on y-direction
				if abs(self.vel.linear.y) < 0.15:
					self.vel.linear.y = 0.15 * np.sign(self.vel.linear.y)
		# filter out shakes from AR tracking package
		elif(abs(np.degrees(self.diff_theta)) > 65):
			self.vel.angular.z = 0.005 * np.sign(self.diff_theta)
		else:
			self.vel.angular.z = self.kp_theta * self.diff_theta
			if(abs(self.vel.angular.z) < 0.02):
				self.vel.angular.z = 0.02 * np.sign(self.vel.angular.z)
		state = self.vel.linear.x + self.vel.linear.y + self.vel.angular.z
		# check if the 1st phase of docking is done
		if(state == 0): 
			#print("start visual servo.")
			self.visual_servo()
		else:
			self.vel_pub.publish(self.vel)

	# second phase of docking, directly using visual information
	def visual_servo(self):
		kp_x = 0.5
		kp_y = 3.0
		vel = Twist()
		# in case the 2nd docking process failed
		if(abs(np.degrees(self.diff_theta)) > 5 or self.diff_y > 0.02):
			self.start = time.time()
		# won't adjust vel.linear.x and vel.linear.y at the same time,
		# to avoid causing hardware damage
		
		# use a larger threshold when in last step, because the noise of visual feedback always makes vel.linear.y jumps between some value and 0
		# which destroyed the priority of y
		if(self.LAST_STEP):
			tolerance = 0.01
		else:
			tolerance = 0.003
		if(abs(self.diff_y) > tolerance):
			vel.linear.y = kp_y * self.diff_y
			if abs(vel.linear.y) < 0.03:
				vel.linear.y = 0.03 * np.sign(vel.linear.y)
			elif abs(vel.linear.y > 0.05):
				vel.linear.y = 0.05 * np.sign(vel.linear.y)
			vel.linear.x = 0
		else:
			self.LAST_STEP = True
			vel.linear.y = 0
			# correspondent: montage x = +25cm
			if(self.diff_x - 0.55 > 0.01):
				vel.linear.x = min(max(kp_x * (self.diff_x - 0.30), 0.03), 0.05)
			else:
				vel.linear.x = 0
				vel.linear.y = 0
			if(abs(np.degrees(self.diff_theta)) < 0.05):
				vel.angular.z = 0
			else:
				vel.angular.z = 0.5 * self.kp_theta * self.diff_theta
				if(abs(self.vel.angular.z) < 0.02):
					self.vel.angular.z = 0.02 * np.sign(self.vel.angular.z)
		self.vel_pub.publish(vel)
		# check if the process is done
		if(not (vel.linear.x + vel.linear.y)):
			rospy.set_param('docking', False)
			self.marker_pose_calibrated = PoseStamped()
			print("Connection established.")

if __name__ == '__main__':
	my_docking = Docking()
	while(not rospy.is_shutdown()):
		# start docking when service is called
		# make sure marker is detected
		if(my_docking.marker_pose_calibrated.pose.position.x and rospy.get_param('docking')):
			my_docking.calculate_diff()
			my_docking.auto_docking()
		my_docking.rate.sleep()
