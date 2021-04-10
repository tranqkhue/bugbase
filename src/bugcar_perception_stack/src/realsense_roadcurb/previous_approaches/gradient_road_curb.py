#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Dec  6 22:23:44 2019

@author: tranquockhue
"""

import pyrealsense2 as rs
import numpy as np
import cv2
import skimage.morphology
import scipy
DEPTH_SHAPE = (848, 480)

# ==============================================================================

def sobel(y_frame):

	y_frame = (y_frame*30)
	y_frame = np.exp(y_frame)

	sobely = cv2.Sobel(y_frame, cv2.CV_64F, 0, 1,ksize=1)	
	sobely = np.absolute(sobely)
	sobely = sobely.astype(np.uint8)
	#cv2.imshow('sobel_y', sobely)
	sobely = (sobely/85) *255

	edges = cv2.Canny(sobely.astype(np.uint8),255,255)
	#cv2.imshow('edges', edges)

	th = cv2.inRange(sobely, 15, 255)
	#cv2.imshow('th', th.astype(np.uint8))
	
	kernel = cv2.getStructuringElement(cv2.MORPH_CROSS,(2,2))
	closed = cv2.morphologyEx(th,cv2.MORPH_CLOSE,kernel.astype(np.uint8),iterations=1)
	#cv2.imshow('closed', closed)
	mask = skimage.morphology.skeletonize(closed/255)

	return closed

# ==============================================================================

def axis_mapping(depth_frame, profile):
	global DEPTH_SHAPE

	points  = pc.calculate(depth_frame)
	v       = points.get_vertices()
	vtx     = np.asanyarray(v).view(np.float32)
	vtx = vtx.reshape(-1, 3)
	x_only = vtx[:, 0]
	y_only = vtx[:, 1]
	z_only = vtx[:, 2]

	x_frame = x_only.reshape(-1, 424)
	y_frame = y_only.reshape(-1, 424)
	z_frame = z_only.reshape(-1, 424)

	return x_frame, y_frame, z_frame

# ==============================================================================

def to_og(x_coordinate, y_coordinate, mask):
	# x_coordinate, y_coordinate are in BEV frame
	# which correspond to z_frame and x_frame in Realsense frame
	# mask = cv2.resize(mask, (0,0), fx=0.6, fy=0.6)
	# x_coordinate = cv2.resize(x_coordinate,(0,0),fx=0.6,fy=0.6)
	# y_coordinate = cv2.resize(y_coordinate,(0,0),fx=0.6,fy=0.6)
	og = np.zeros((500,500)) 

	masked_index = np.where(mask, True, False)
	x_obstacle_points = x_coordinate[masked_index]*100 # convert m to cm?
	y_obstacle_points = y_coordinate[masked_index]*100
	
	laser_like_pts = np.stack((x_obstacle_points, y_obstacle_points), axis=1)
	laser_like_pts = np.append(laser_like_pts,np.zeros((1,2)), axis=0)
	origin = np.array([[og.shape[1]/2,og.shape[0]/2]])
	laser_like_pts =np.round(laser_like_pts + origin).astype(np.int32)

	for pt in laser_like_pts:
		cv2.circle(og,(pt[0],pt[1]),1,255,thickness=-1)
	# og = cv2.resize(og,(0,0),fx=1/5,fy=1/5)
	# og = cv2.morphologyEx(og,cv2.MORPH_CLOSE,np.ones((3,3)))
	return og

# ================================  MAIN  ======================================

# Realsense config
pipeline = rs.pipeline()
config = rs.config()
# config.enable_device_from_file("realsense_day.bag", repeat_playback=True)
config.enable_stream(
	rs.stream.depth, DEPTH_SHAPE[0], DEPTH_SHAPE[1], rs.format.z16, 90)

# config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
profile = pipeline.start(config)
intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
device = profile.get_device()
depth_sensor = device.query_sensors()[0]
# laser_pwr = depth_sensor.get_option(rs.option.laser_power)
# depth_sensor.set_option(rs.option.laser_power,0)
pc = rs.pointcloud()

# ------------------------------------------------------------------------------

# Realsense-based post-processing
decimation = rs.decimation_filter()
decimation.set_option(rs.option.filter_magnitude, 2)
spatial = rs.spatial_filter()
spatial.set_option(rs.option.filter_magnitude, 3)
spatial.set_option(rs.option.filter_smooth_alpha, 0.5)
spatial.set_option(rs.option.filter_smooth_delta, 50)
temporal = rs.temporal_filter()
hole_filler = rs.hole_filling_filter()

# ------------------------------------------------------------------------------

try:
	while True:
		frames = pipeline.wait_for_frames()
		depth_frame = frames.get_depth_frame()  # (h,w,z-axis)

		decimation_processed    = decimation.process(depth_frame)
		spatial_processed 		= spatial.process(decimation_processed)
		temporal_processed 		= temporal.process(spatial_processed)
		hole_processed 			= hole_filler.process(temporal_processed)

		x_frame, y_frame, z_frame = axis_mapping(hole_processed, profile)
		#Resize to filter noise
		x_frame = cv2.resize(x_frame, (0,0), fx=0.6, fy=0.6)
		y_frame = cv2.resize(y_frame, (0,0), fx=0.6, fy=0.6)
		z_frame = cv2.resize(z_frame, (0,0), fx=0.6, fy=0.6)
		# Crop frames into lower half
		old_shape = x_frame.shape
		x_frame = x_frame[round(x_frame.shape[0]/1.8):x_frame.shape[0],\
					      round(x_frame.shape[1]*0.2):x_frame.shape[1]]
		y_frame = y_frame[round(y_frame.shape[0]/1.8):y_frame.shape[0],\
				  		  round(y_frame.shape[1]*0.2):y_frame.shape[1]]
		z_frame = z_frame[round(z_frame.shape[0]/1.8):z_frame.shape[0],\
						  round(z_frame.shape[1]*0.2):z_frame.shape[1]]

		# Remove value further than 3 meter from the camera
		x_frame[z_frame > 10] = 0
		y_frame[z_frame > 3]  = 0
		z_frame[z_frame > 10] = 0

		# Calculate sobel y gradient
		mask = sobel(y_frame)

		key = cv2.waitKey(1) & 0xFF
		if key == ord('q'):
			break
		if key == ord('p'):
			cv2.waitKey(0)

finally:
	pipeline.stop()
	#cap.release()
	cv2.destroyAllWindows()