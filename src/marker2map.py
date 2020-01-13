#!/usr/bin/env python

import rospy
import roslib
import tf
import numpy
import geometry_msgs.msg

if __name__ == '__main__':

	rospy.init_node('tf_reader')
	listener = tf.TransformListener()
	rate = rospy.Rate(15)
	while(not rospy.is_shutdown()):
		try:
			listener.waitForTransform('/map', '/ar_marker_10', rospy.Time(0), rospy.Duration(10.0))
			print(listener.lookupTransform('/map', '/ar_marker_10', rospy.Time(0)))
		except:
			pass
		rate.sleep()