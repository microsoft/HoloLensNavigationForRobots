#!/usr/bin/env python

# --------------------------------------------------------------------------------------------
# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.
# --------------------------------------------------------------------------------------------

import rospy
import socket
import struct
import sys
import tf
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import *
import math


# -----------------------------------------------------------------------------
#
if __name__ == '__main__':
	rospy.init_node('dynamic_adjuster')
	listener=tf.TransformListener()

	if len(sys.argv)<2:
		rospy.logerr("usage: dynamic_adjuster.py <robot's foot frame name>")
		exit()

	footprintFrame="/" + sys.argv[1]

	print "dynamic_adjuster: footprintframe: " + footprintFrame

	hololensPos = rospy.Publisher('/initialpose_h', PoseWithCovarianceStamped,queue_size=1)

	rate = rospy.Rate(10.0)
	br = tf.TransformBroadcaster()
		
	last_update = rospy.get_time()
	firstloop = True
	adjuster = True

	while not rospy.is_shutdown():
		
        #get transform from map to footprint via hololens
        #   map -> HoloLens 
		try:
			#if listener.frameExists("/map") and listener.frameExists("/hololens"):
				t = listener.getLatestCommonTime('/map', '/hololens')
				(trans,rot) = listener.lookupTransform('/map', '/hololens', t)
			#else:
			#	continue
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException,tf.Exception):
			print "tf error /map /hololens"
			continue
		try:
			(trans2,rot2) = listener.lookupTransform('/hololens_p', footprintFrame, rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException,tf.Exception):
			print "tf error hololens_p footprintframe"
			continue
				
		#position jump check caused by switching spatial anchor
		if not firstloop:
			if (last_t == t):
				continue

			d = (trans[0]-last_trans[0])*(trans[0]-last_trans[0])+(trans[1]-last_trans[1])*(trans[1]-last_trans[1])
			now = rospy.get_time()

			if d > 1.0 and now - last_update < 2.0:
				print "dynamic_adjuster: position jumped (ddist , dtime) (" + str(d) + "," + str(now-last_update)
				continue        

		#compute transform matrix from map to hololens_p
		map2holo = np.dot(tf.transformations.translation_matrix(trans), tf.transformations.quaternion_matrix(rot))
		holo2foot = np.dot(tf.transformations.translation_matrix(trans2), tf.transformations.quaternion_matrix(rot2))    

		map2foot=np.dot(map2holo,holo2foot)
		map2foot_beforeAdjust=map2foot
		
        #dynamic footprint correction
		if adjuster:
			foot2holo=np.linalg.inv(holo2foot)

			#calc adjust rotation matrix 
			foot2map=np.linalg.inv(map2foot)
			rz=np.array([foot2map[0][2],foot2map[1][2],foot2map[2][2]])

			to_z=np.array([0,0,1.0])
			axis=np.cross(rz,to_z)
			angle=math.acos(np.dot(rz,to_z))
			adjustMatrixA=tf.transformations.quaternion_matrix(tf.transformations.quaternion_about_axis(angle,axis))
			adjustMatrixA=np.linalg.inv(adjustMatrixA)
			#debug
			debugMat=np.dot(map2foot,adjustMatrixA)

			#
			adjustMatrixB=np.dot(np.dot(holo2foot,adjustMatrixA),foot2holo)
			adjustMatrixB[0][3]=0
			adjustMatrixB[1][3]=0
			adjustMatrixB[2][3]=0
			map2foot=np.dot(np.dot(map2holo,adjustMatrixB),holo2foot)
		
		brtrans = (map2foot[0][3], map2foot[1][3], map2foot[2][3])
		brrot = tf.transformations.quaternion_from_matrix(map2foot)		
		br.sendTransform(brtrans, brrot, rospy.Time.now(), "/localized_footprint", "/map")
		
		brtrans = (map2foot_beforeAdjust[0][3], map2foot_beforeAdjust[1][3], map2foot_beforeAdjust[2][3])
		brrot = tf.transformations.quaternion_from_matrix(map2foot_beforeAdjust)		
		br.sendTransform(brtrans, brrot, rospy.Time.now(), "/localized_footprint_nAdj", "/map")

		cmd = PoseWithCovarianceStamped()
		cmd.pose.pose.position.x=map2foot[0][3]
		cmd.pose.pose.position.y=map2foot[1][3]
		cmd.pose.pose.position.z=0    
		q=tf.transformations.quaternion_from_matrix(map2foot)
		invq=tf.transformations.quaternion_inverse(q)

		dirq=  np.zeros((4, ), dtype=np.float64)
		dirq[0]=1  
		q1=tf.transformations.quaternion_multiply(dirq,invq)
		q2=tf.transformations.quaternion_multiply(q,q1)

		qz=q2[0]
		qw=q2[1]
		rad=math.sqrt(qz*qz+qw*qw)
		qz=qz/rad
		qw=qw/rad

		theta = math.acos(qz)
		if qw < 0:
			theta=-theta

		cmd.pose.pose.orientation.x = 0
		cmd.pose.pose.orientation.y = 0
		cmd.pose.pose.orientation.z = math.sin(theta/2)
		cmd.pose.pose.orientation.w = math.cos(theta/2)
		hololensPos.publish(cmd)
		last_trans = trans
		last_update = rospy.get_time()
		firstloop = False
		last_t = t
		rate.sleep()
