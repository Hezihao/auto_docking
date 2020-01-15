#!/usr/bin/env python

import rospy
import numpy as np
import tf2_ros
import tf2_geometry_msgs

from nav_msgs.msg import Odometry
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import TransformStamped, PoseStamped, Vector3, Twist


def auto_dock(base_marker_diff):
	kp = 0.2

	vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	vel = Twist()
	vel.linear.x = kp * base_marker_diff.pose.position.x
	vel.linear.y = kp * base_marker_diff.pose.position.y
	vel.angular.z = kp * np.arccos(np.degrees(base_marker_diff.pose.orientation.w))
	vel_pub.publish(vel)

def odom_callback(odom):
	global base_pose

	base_pose = PoseStamped()
	base_pose.header = odom.header
	base_pose.pose = odom.pose.pose
	#print(base_pose)

def marker_pose_reader(ar_markers):
	global base_pose

	for mkr in ar_markers.markers:
		if(mkr.id == 10):
			marker_pose = mkr.pose
			marker_pose.header.frame_id = 'camera_link'
			#try:
			transform = tf_buffer.lookup_transform('map',
													'camera_link',
													rospy.Time(0),
													rospy.Duration(1.0))
			marker_in_map = tf2_geometry_msgs.do_transform_pose(marker_pose, transform)
			# correct the published orientation of marker
			# in convinience of calculating the error of orientation between marker & base_link
			marker_correction = TransformStamped()
			marker_correction.header.stamp = rospy.Time.now()
			marker_correction.header.frame_id = 'map'
			marker_correction.child_frame_id = 'ar_marker_10'
			marker_correction.transform.rotation.x = 0.707
			marker_correction.transform.rotation.y = 0
			marker_correction.transform.rotation.z = 0
			marker_correction.transform.rotation.w = 0.707
			marker_corrected = tf2_geometry_msgs.do_transform_pose(marker_in_map, marker_correction)
			marker_corrected.pose.position = marker_in_map.pose.position

			odom_map_diff = tf_buffer.lookup_transform('map', 'odom', rospy.Time(0), rospy.Duration(1.0))
			base_pose = tf2_geometry_msgs.do_transform_pose(base_pose, odom_map_diff)
			base_pose.header.frame_id = 'map'
			base_marker_diff = PoseStamped()
			base_marker_diff.header.stamp = rospy.Time.now()
			base_marker_diff.header.frame_id = 'map'
			# compute the diff under robot-coordinate
			base_marker_diff.pose.position.x = marker_corrected.pose.position.x - base_pose.pose.position.x
			base_marker_diff.pose.position.y = marker_corrected.pose.position.y - base_pose.pose.position.y
			map_to_base = tf_buffer.lookup_transform('base_link',
													'map',
													rospy.Time(0),
													rospy.Duration(1.0))
			map_to_base.transform.translation = Vector3()
			base_marker_diff = tf2_geometry_msgs.do_transform_pose(base_marker_diff, map_to_base)
			# using trigonometry to solve new w and z
			base_marker_diff.pose.orientation.z = marker_corrected.pose.orientation.z * base_pose.pose.orientation.w - marker_corrected.pose.orientation.w * base_pose.pose.orientation.z

			base_marker_diff.pose.orientation.w = marker_corrected.pose.orientation.w * base_pose.pose.orientation.w + marker_corrected.pose.orientation.z * base_pose.pose.orientation.z

			#print(marker_corrected.pose.orientation.z, marker_corrected.pose.orientation.w)
			#print(base_pose.pose.orientation.z, base_pose.pose.orientation.w)
			print(base_marker_diff)
			auto_dock(base_marker_diff)
			#except:
			#	pass
			#marker_in_map_pub.publish(marker_in_map)
			ar_pose_corrected_pub.publish(marker_corrected)

if __name__ == '__main__':
	# do docking here in main, and use a while loop and rospy.Rate() to control the loop length
	# by doing that, PID controller could be applied, and better structure.
	rospy.init_node('tf_reader')
	tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
	listener = tf2_ros.TransformListener(tf_buffer)
	marker_pose_sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, marker_pose_reader)
	odom_sub = rospy.Subscriber('odom', Odometry, odom_callback)
	#marker_in_map_pub = rospy.Publisher('ar_pose_in_map', PoseStamped, queue_size=1)
	ar_pose_corrected_pub = rospy.Publisher('ar_pose_corrected', PoseStamped, queue_size=1)
	rospy.spin()