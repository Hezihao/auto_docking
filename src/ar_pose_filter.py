#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from ar_track_alvar_msgs.msg import AlvarMarkers

pose = PoseStamped()
marker = Marker()

# define callback function of ar_pose_filter
def ar_pose_filter(AlvarMarkers):
	global pose
	global marker

	for mkr in AlvarMarkers.markers:
		#print(marker)
		if(mkr.id == 10):
			pose = mkr.pose
		pose.header.frame_id = '/camera_link'
		print(pose)

if __name__ == '__main__':
	# initialization
	rospy.init_node('ar_tag_pose_reader')
	rate = rospy.Rate(5)
	while(not rospy.is_shutdown()):
		pose_sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, ar_pose_filter)
		pose_pub = rospy.Publisher('ar_pose_filter', PoseStamped, queue_size=1)
		pose_pub.publish(pose)
		rate.sleep()