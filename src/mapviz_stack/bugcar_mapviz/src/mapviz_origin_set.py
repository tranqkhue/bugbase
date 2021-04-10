#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import NavSatFix

rospy.init_node('mapviz_origin_set')

r = rospy.Rate(0.5)
gps_info_topic = rospy.get_param('~/gps_topic', 'gps/fix')
max_gps_cov    = rospy.get_param('~/max_gps_cov', 25)
swri_tf_param  = rospy.get_param('swri_tf_topic', '/initialize_origin/local_xy_origins')

while not rospy.is_shutdown():
	try:
		gps_data = rospy.wait_for_message(gps_info_topic, NavSatFix, timeout=1)
		gps_cov  = (gps_data.position_covariance)[0]
		if (gps_cov > max_gps_cov):
			rospy.logerr('GPS covariance is too high to set mapviz datum!')
		else:
			lat = gps_data.latitude
			lon = gps_data.longitude
			altitude = gps_data.altitude
			break
	except rospy.exceptions.ROSException:
		rospy.logerr('No GPS data received!')
		
	try:
		[lat,lon,altitude] = rospy.get_param('/navsat_transform/datum')
		break
	except KeyError:
		rospy.logerr('Cannot get origin from /navsat_transform/datum, trying again')

	r.sleep()

param = {'latitude': lat,
	     'longitude': lon,
	     'altitude': altitude,
	     'name': 'origin',
	     'heading': 0.0} #Heading NED

param = [param]
rospy.set_param(swri_tf_param, param)
rospy.loginfo('Done initialize SWRI TF Origin')