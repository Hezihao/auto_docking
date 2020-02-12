#!/usr/bin/env python

import rospy
import tf2_ros
import numpy as np
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, TransformStamped
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Pose_filter:
	# initialization
	def __init__(self):
		rospy.init_node('marker_pose_filter')
		self.rate = rospy.Rate(15)
		self.marker_pose = PoseStamped()
		self.marker_pose_calibrated = PoseStamped()
		self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
		self.position_queue = []
		self.orientation_queue = []
		listener = tf2_ros.TransformListener(self.tf_buffer)
		self.filtered_pose_pub = rospy.Publisher('ar_pose_filtered', PoseStamped, queue_size=1)
		# for test
		self.calibrated_pose_pub = rospy.Publisher('ar_pose_corrected', PoseStamped, queue_size=1)
		origin_pose_sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.marker_pose_calibration)

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
	def do_calibration(self, marker):
		# correct the published orientation of marker
		# in convinience of calculating the error of orientation between marker & base_link
		cam_to_map = self.tf_buffer.lookup_transform('map', 'camera_link', rospy.Time(0), rospy.Duration(1.0))
		marker_in_map = tf2_geometry_msgs.do_transform_pose(marker, cam_to_map)
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
		self.calibrated_pose_pub.publish(marker_corrected)
		euler_vec = euler_from_quaternion([marker_corrected.pose.orientation.x, marker_corrected.pose.orientation.y, marker_corrected.pose.orientation.z, marker_corrected.pose.orientation.w])
		orient_vec = quaternion_from_euler(0, 0, euler_vec[2])
		marker_corrected.pose.orientation.x = orient_vec[0]
		marker_corrected.pose.orientation.y = orient_vec[1]
		marker_corrected.pose.orientation.z = orient_vec[2]
		marker_corrected.pose.orientation.w = orient_vec[3]
		return marker_corrected

	# callback function: transforms measured marker pose into something comparable with robot coordinate system
	def marker_pose_calibration(self, ar_markers):
		for mkr in ar_markers.markers:
			if(mkr.id == 27):
			# read pose data of the predefined marker
				self.marker_pose = mkr.pose
				self.marker_pose.header.frame_id = 'camera_link'
				# do rotation, and remove unused information
				self.marker_pose_calibrated = self.do_calibration(self.marker_pose)

	# following functions serve for temporal Sliding Window
	# pack the PoseStamped into vector
	def vec_from_pose(self, pose):
		position_vec =  []
		orient_vec = []

		position_vec.append(pose.position.x)
		position_vec.append(pose.position.y)
		position_vec.append(pose.position.z)

		orient_vec.append(pose.orientation.x)
		orient_vec.append(pose.orientation.y)
		orient_vec.append(pose.orientation.z)
		orient_vec.append(pose.orientation.w)
		return [position_vec, orient_vec]

	# unpack the vector into PoseStamped
	def pose_from_vec(self, p_vec, o_vec):
		pose = PoseStamped()
		pose.header.stamp = rospy.Time.now()
		pose.header.frame_id = "map"
		pose.pose.position.x = p_vec[0]
		pose.pose.position.y = p_vec[1]
		pose.pose.position.z = p_vec[2]
		pose.pose.orientation.x = o_vec[0]
		pose.pose.orientation.y = o_vec[1]
		pose.pose.orientation.z = o_vec[2]
		pose.pose.orientation.w = o_vec[3]
		return pose

	# average value of each vec in a matrix
	def avr(self, mat):
		if(len(mat)<2):
			return mat[0]
		ans = []
		l = len(mat[0])
		for i in range(l):
			s = 0
			for vec in mat:
				s += vec[i]
			ans.append(s/len(mat))
		return ans

if __name__ == "__main__":
	my_filter = Pose_filter()
	position_queue = []
	orientation_queue = []
	while(not rospy.is_shutdown()):
		# if marker 27 is provided
		if(my_filter.marker_pose_calibrated.pose.position.x):
			#my_filter.filtered_pose_pub.publish(my_filter.marker_pose_calibrated)
			#print(my_filter.marker_pose_calibrated.pose.orientation)
			[position_vec, orient_vec] = my_filter.vec_from_pose(my_filter.marker_pose_calibrated.pose)
			euler_vec = euler_from_quaternion(orient_vec)
			position_queue.append(position_vec)
			orientation_queue.append(orient_vec)
			filtered_position_vec = my_filter.avr(position_queue)
			filtered_orient_vec = my_filter.avr(orientation_queue)
			filtered_pose = my_filter.pose_from_vec(filtered_position_vec, filtered_orient_vec)
			my_filter.filtered_pose_pub.publish(filtered_pose)
			if(len(position_queue) == 15):
				position_queue.pop(0)
				orientation_queue.pop(0)
		my_filter.rate.sleep()