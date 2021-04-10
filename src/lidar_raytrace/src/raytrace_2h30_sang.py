#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
from numpy import sin,cos

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

#===============================================================================

def scan_to_og(data):
	global NAN_TO_NUM
	global INF_TO_NUM

	img = np.ones(shape=(MAP_SIZE_PIXEL,MAP_SIZE_PIXEL))*(-1)
	polygon = []
	origin = np.array([MAP_SIZE_PIXEL/2,MAP_SIZE_PIXEL/2])
	# remember, the origins in occupancy_grid and img are different. ( one is topleft, the other is  in the botleft corner)
	# so we have to flip the image vertically in order to match the occupancy_grid origin with the image origin.
	# lidar scan direction is counter clockwise
	angle_min = data.angle_min
	angle_max = data.angle_max
	angle_max_deg = (angle_max*180/np.pi)
	angle_min_deg = (angle_min*180/np.pi)
	# print(angle_min,angle_max)
	lidar_radius_px = int(data.range_max/MAP_RESOLUTION)
	origin_as_tuple = (int(origin[0]),int(origin[1]))
	# angle is from x axis 
	img = cv2.ellipse(img,origin_as_tuple,(lidar_radius_px,lidar_radius_px),angle=-90,startAngle=angle_min_deg,endAngle=angle_max_deg,color=100,thickness=-1)
	# print(np.histogram(np.asarray(data.ranges)))
	data.ranges = (np.nan_to_num(data.ranges, nan=np.nan, posinf=INF_TO_NUM)).tolist()
	for enum, distance in enumerate(data.ranges):
		# print(distance)
		angle = angle_min + data.angle_increment * enum  # in radians
		if (np.isnan(distance) | (distance==0)):
			continue
		#it would be so computationally expensive to flip the image,so we only need to flip the vertices of the filled polygon
		#which will achieve the same result.
		cartesian_coordinate = np.array([distance*sin(angle),-distance*cos(angle)]) # in metres
		pixel_coordinate = cartesian_coordinate / MAP_RESOLUTION
		polygon.append(pixel_coordinate + origin)
	polygon.append(origin)
	polygon = np.round(np.asarray(polygon)).astype(np.int)
	# fake_poly = polygon - origin
	# print(fake_poly)
	img = cv2.fillPoly(img,[polygon],0,cv2.LINE_AA).astype(np.int8)
	# WARNING!!!
	# For 180 degree LIDAR ONLY!
	#img[:,0:round(MAP_SIZE_PIXEL/2)] = -1
	rotated = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE) 

	frame = data.header.frame_id
	map_msg = create_occ_grid(rotated, origin, frame)
	OG_publisher.publish(map_msg)

#===============================================================================

def create_occ_grid(map_img, origin, frame):
	global MAP_SIZE_PIXEL
	global MAP_RESOLUTION

	global TIMEOUT 
	TIMEOUT = rospy.Time.now().to_sec()

	occupancy_grid = map_img.flatten()
	occupancy_grid = occupancy_grid.tolist()

	map_msg = OccupancyGrid()

	map_msg.header = Header()
	map_msg.header.frame_id = "laser"
	map_msg.header.stamp    = rospy.get_rostime()

	map_msg.info= MapMetaData()
	map_msg.info.map_load_time = rospy.get_rostime()
	map_msg.info.height = MAP_SIZE_PIXEL      #Unit: Pixel
	map_msg.info.width  = MAP_SIZE_PIXEL      #Unit: Pixel
	map_msg.info.resolution = MAP_RESOLUTION

	map_msg.info.origin = Pose()
	map_msg.info.origin.position = Point()
	map_msg.info.origin.position.x = -origin[0]*MAP_RESOLUTION    #Unit: Meter
	map_msg.info.origin.position.y = -origin[1]*MAP_RESOLUTION    #Unit: Meter
	map_msg.info.origin.position.z = 0
	map_msg.info.origin.orientation = Quaternion()
	map_msg.info.origin.orientation.x = 0
	map_msg.info.origin.orientation.y = 0
	map_msg.info.origin.orientation.z = 0
	map_msg.info.origin.orientation.w = 1

	map_msg.data.extend(occupancy_grid)

	return map_msg

#===============================================================================

rospy.init_node('lidar_raytrace')
NAN_TO_NUM  	 = rospy.get_param("~nan_to_num", 0.2)
INF_TO_NUM 		 = rospy.get_param("~inf_to_num", 10)
MAP_TOPIC 	     = rospy.get_param("~map_topic", "/map/scan")
LASER_SCAN_TOPIC = "/scan"
MAP_RESOLUTION   = rospy.get_param("~map_resolution", 0.02)
MAP_SIZE         = rospy.get_param("~map_size", 10)  #Unit: Meter, Shape: Square with center "base_link"
MAP_SIZE_PIXEL   = int(MAP_SIZE / MAP_RESOLUTION)

OG_publisher = rospy.Publisher(MAP_TOPIC, OccupancyGrid, queue_size=1)
rospy.Subscriber(LASER_SCAN_TOPIC, LaserScan, scan_to_og)

TIMEOUT = rospy.Time.now().to_sec()
r = rospy.Rate(9)
print(LASER_SCAN_TOPIC)
while not rospy.is_shutdown():
	if (rospy.Time.now().to_sec() - TIMEOUT) > 2.5:
		rospy.logerr("Lidar Raytrace: No LaserScan message received!")
	r.sleep()