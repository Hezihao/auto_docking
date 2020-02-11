#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

position_queue = []
orientation_queue = []

# pack the PoseStamped into vector
def vec_from_pose(pose):
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
def pose_from_vec(p_vec, o_vec):
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
def avr(mat):
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

# callback function that filters marker's pose
def filter_callback(pose_stamped):
	global num_of_poses
	global position_queue
	global orientation_queue

	[position_vec, orient_vec] = vec_from_pose(pose_stamped.pose)
	euler_vec = euler_from_quaternion(orient_vec)
	position_queue.append(position_vec)
	orientation_queue.append(orient_vec)
	position_filtered_vec = avr(position_queue)
	orient_filtered_vec = avr(orientation_queue)
	pose_filtered = pose_from_vec(position_filtered_vec, orient_filtered_vec)
	pose_filtered_pub.publish(pose_filtered)
	if(len(position_queue) == 15):
		position_queue.pop(0)
		orientation_queue.pop(0)

if __name__ == "__main__":
	rospy.init_node('marker_pose_filter')
	rospy.Subscriber('ar_pose_corrected', PoseStamped, filter_callback)
	pose_filtered_pub = rospy.Publisher('ar_pose_filtered', PoseStamped, queue_size=1)
	rate = rospy.Rate(15)
	while(not rospy.is_shutdown()):

		rate.sleep()