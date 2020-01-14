#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import TransformStamped, PoseStamped, Vector3
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
				# correct the published orientation of marker
				# in convinience of calculating the error of orientation between marker & base_link
				diff_to_dock = TransformStamped()
				diff_to_dock.header.stamp = rospy.Time.now()
				diff_to_dock.header.frame_id = 'map'
				diff_to_dock.child_frame_id = 'ar_marker_10'
				diff_to_dock.transform.rotation.x = 0.707
				diff_to_dock.transform.rotation.y = 0
				diff_to_dock.transform.rotation.z = 0
				diff_to_dock.transform.rotation.w = 0.707

				print(diff_to_dock)
				dock_ready_position = tf2_geometry_msgs.do_transform_pose(marker_in_map, diff_to_dock)
				dock_ready_position.pose.position = marker_in_map.pose.position
			except:
				pass
			marker_in_map_pub.publish(marker_in_map)
			dock_ready_position_pub.publish(dock_ready_position)
			marker_observed = False

if __name__ == '__main__':

	global marker_in_map_pub

	rospy.init_node('tf_reader')
	tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
	listener = tf2_ros.TransformListener(tf_buffer)
	marker_pose_sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, marker_pose_reader)
	marker_in_map_pub = rospy.Publisher('ar_pose_in_map', PoseStamped, queue_size=1)
	dock_ready_position_pub = rospy.Publisher('dock_ready_position', PoseStamped, queue_size=1)
	rospy.spin()