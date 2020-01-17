#!/usr/bin/env python

import rospy
import time
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion

from nav_msgs.msg import Odometry
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import TransformStamped, PoseStamped, Vector3, Twist

class docking:

	def __init__(self):
		self.kp_x = 0.3
		self.kp_y = 1.5
		self.kp_theta = 0.5
		self.state = 1
		self.start = time.time()
		self.marker_observed = False
		self.base_pose = PoseStamped()
		self.marker_pose_calibrated = PoseStamped()
		self.base_marker_diff = PoseStamped()

		rospy.init_node('auto_docking')
		self.rate = rospy.Rate(50)
		self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
		listener = tf2_ros.TransformListener(self.tf_buffer)
		odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
		marker_pose_sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.marker_pose_calibration)
		self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
		self.ar_pose_corrected_pub = rospy.Publisher('ar_pose_corrected', PoseStamped, queue_size=1)
		rospy.loginfo("Auto docking...")

	def odom_callback(self, odom):
		self.base_pose.header = odom.header
		self.base_pose.pose = odom.pose.pose

	def marker_pose_calibration(self, ar_markers):
		for mkr in ar_markers.markers:
			if(mkr.id == 10):
				# read pose data of the predefined marker
				self.marker_observed = True
				marker_pose = mkr.pose
				marker_pose.header.frame_id = 'camera_link'
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
				marker_in_map = tf2_geometry_msgs.do_transform_pose(marker_pose, transform)
				marker_corrected = tf2_geometry_msgs.do_transform_pose(marker_in_map, marker_correction)
				marker_corrected.pose.position = marker_in_map.pose.position
				self.marker_pose_calibrated = marker_corrected
				#print(self.marker_pose_calibrated)
				
	def calculate_diff(self):

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
		#print(self.base_marker_diff.pose.position.x, self.base_marker_diff.pose.position.y)
		map_to_base.transform.translation = Vector3()
		self.base_marker_diff = tf2_geometry_msgs.do_transform_pose(self.base_marker_diff, map_to_base)
		# using trigonometry to solve new w and z
		#self.base_marker_diff.pose.orientation.z = self.marker_pose_calibrated.pose.orientation.z * self.base_pose.pose.orientation.w - self.marker_pose_calibrated.pose.orientation.w * self.base_pose.pose.orientation.z

		#self.base_marker_diff.pose.orientation.w = self.marker_pose_calibrated.pose.orientation.w * self.base_pose.pose.orientation.w + self.marker_pose_calibrated.pose.orientation.z * self.base_pose.pose.orientation.z
		marker_pose_calibrated_euler = euler_from_quaternion([self.marker_pose_calibrated.pose.orientation.x, self.marker_pose_calibrated.pose.orientation.y, self.marker_pose_calibrated.pose.orientation.z, self.marker_pose_calibrated.pose.orientation.w])
		base_pose_euler = euler_from_quaternion([self.base_pose.pose.orientation.x, self.base_pose.pose.orientation.y, self.base_pose.pose.orientation.z, self.base_pose.pose.orientation.w])
		# calculate the difference
		self.diff_x = self.base_marker_diff.pose.position.x
		self.diff_y = self.base_marker_diff.pose.position.y
		self.diff_theta = marker_pose_calibrated_euler[2]-base_pose_euler[2]
		#print(self.diff_x, self.diff_y, self.diff_theta)
		#self.marker_observed = False

	def auto_docking(self):
		vel = Twist()
		#print(self.base_marker_diff)
		# calculate the velocity needed for docking
		if(abs(self.diff_x) < 0.90):
			vel.linear.x = 0
		else:
			vel.linear.x = self.kp_x * self.diff_x
		if(abs(self.diff_y) < 0.005):
			vel.linear.y = 0
		else:
			vel.linear.y = self.kp_y * self.diff_y
		if(abs(np.degrees(self.diff_theta)) < 0.03):
			vel.angular.z = 0
		elif(abs(self.diff_theta) > 45):
			# the max. acceptable angle
			vel.angular.z = 0.005
		else:
			vel.angular.z = self.kp_theta * self.diff_theta
		
		self.vel_pub.publish(vel)
		self.state = vel.linear.x + vel.linear.y + vel.angular.z
		if(self.state == 0 or (time.time() - self.start) > 15): 
			self.state = 0
			print(self.diff_x - 0.8, self.diff_y, self.diff_theta)
			print("Docking done.")

if __name__ == '__main__':
	my_docking = docking()
	while(not rospy.is_shutdown()):
		if(my_docking.state and my_docking.marker_pose_calibrated.pose.position.x):
			my_docking.calculate_diff()
			my_docking.auto_docking()
		my_docking.rate.sleep()