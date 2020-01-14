#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import PoseStamped
from ar_track_alvar_msgs.msg import AlvarMarkers

marker_observed = False

def marker_pose_reader(ar_markers):
	global marker_observed
	# if any single mkr in markers has id == 10, marker_observed should be set to False
	for mkr in ar_markers.markers:
		if(mkr.id == 10):
			marker_observed = True
			marker_pose = mkr.pose
			marker_pose.header.frame_id = '/camera_link'
		if(marker_observed):
			try:
				transform = tf_buffer.lookup_transform('map',
														'camera_link',
														rospy.Time(0),
														rospy.Duration(1.0))
				marker_in_map = tf2_geometry_msgs.do_transform_pose(marker_pose, transform)
				print(marker_in_map)
			except:
				pass
			marker_in_map_pub.publish(marker_in_map)
			marker_observed = False

if __name__ == '__main__':

	global marker_in_map_pub

	rospy.init_node('tf_reader')
	tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
	listener = tf2_ros.TransformListener(tf_buffer)
	marker_pose_sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, marker_pose_reader)
	marker_in_map_pub = rospy.Publisher('ar_pose_in_map', PoseStamped, queue_size=1)
	rospy.spin()
	#rate = rospy.Rate(5)
	#while(not rospy.is_shutdown()):
	#	rate.sleep()