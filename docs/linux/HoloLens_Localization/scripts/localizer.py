#!/usr/bin/env python

# --------------------------------------------------------------------------------------------
# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.
# --------------------------------------------------------------------------------------------

import roslib
import rospy
import socket
import geometry_msgs.msg
import math
import tf
import struct
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped

global trans
global rot

global brtrans
global brrot

# -----------------------------------------------------------------------------
#
def initialposeCB(msg):
	#robot odom-base (input)
	global trans
	global rot

	#robot map-odom (output)
	global brtrans
	global brrot
	
	#massage to translation, rotation
	inittrans=(msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z)
	initposequot=(msg.pose.pose.orientation.x,
		msg.pose.pose.orientation.y,
		msg.pose.pose.orientation.z,
		msg.pose.pose.orientation.w)		
	initrot=tf.transformations.quaternion_matrix(initposequot)
	map2foot= np.dot(tf.transformations.translation_matrix(inittrans),initrot)
	odom2foot = np.dot(tf.transformations.translation_matrix(trans),tf.transformations.quaternion_matrix(rot))	
	
	
	foot2odom=np.linalg.inv(odom2foot)	
	
	map2odom=np.dot(map2foot,foot2odom)
	br = tf.TransformBroadcaster()	
	#map2foot=np.dot(map2holo,holo2foot)
	brtrans = (map2odom[0][3], map2odom[1][3], map2odom[2][3])
	brrot = tf.transformations.quaternion_from_matrix(map2odom)
	
# -----------------------------------------------------------------------------
#
if __name__ == '__main__':
	rospy.init_node('localizer')

	listener = tf.TransformListener()

	# from ros
	sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, initialposeCB)

	# from dynamic_adjuster.py
	sub2 = rospy.Subscriber('/initialpose_h', PoseWithCovarianceStamped, initialposeCB)

	br = tf.TransformBroadcaster()
	brtrans=(0,0, 0)
	brrot=(0,0,0,1)
	
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		rospy.loginfo("Getting transform for '/base_footprint'!")
		try:
			# obtain robot odometry to base_footprint (for pepper)
			(trans, rot) = listener.lookupTransform('/odom', '/base_footprint', rospy.Time(0))
			rospy.loginfo("Got transform for '/base_footprint'!")
		except (tf.LookupException, tf.ConnectivityException,tf.ExtrapolationException):
			rospy.logwarn("tf error. Unable to get transform for '/base_footprint'!")
			continue

		br.sendTransform(brtrans, brrot, rospy.Time.now(), "/odom", "/map")

		rate.sleep()

	rospy.loginfo("localizer.py exit...")
