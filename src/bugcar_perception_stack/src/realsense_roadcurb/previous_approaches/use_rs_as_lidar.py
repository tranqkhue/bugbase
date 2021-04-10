#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Dec  6 22:23:44 2019

@author: tranquockhue
"""

from re import A
import time
import pyrealsense2 as rs
import ctypes
import numpy as np
import cv2
import matplotlib.cm as cm
from matplotlib.colors import Normalize
import multiprocessing as mp
import rospy
from sklearn.linear_model import Ridge
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion
import skimage

# ==============================================================================
# ============================  OpenCV procesisng  =============================

# ------------------------------------------------------------------------------


def sobel(frame):
	frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)
	sobelx = cv2.Sobel(frame, cv2.CV_64F, 1, 0, ksize=3)
	sobely = cv2.Sobel(frame, cv2.CV_64F, 0, 1, ksize=3)
	sobelx = np.uint8(np.absolute(sobelx))
	sobely = np.uint8(np.absolute(sobely))

	# sobelx = cv2.equalizeHist(sobelx)
	# sobely = cv2.equalizeHist(sobely)
	# sobelx[sobelx < 200] = 0
	# sobely[sobely < 200] = 0
	# sobelx[sobelx > 240] = 0
	# sobely[sobely > 240] = 0
	#cv2.imshow('sobelx', sobelx)
	#cv2.imshow('sobely', sobely)

	sum_sobel = cv2.add(sobelx, sobely)
	sum_sobel = cv2.fastNlMeansDenoising(sum_sobel, None, 10, 7, 21)
	sum_sobel = cv2.equalizeHist(sum_sobel)
	sum_sobel[sum_sobel == 200] = np.min(sum_sobel)

	return sum_sobel
# ------------------------------------------------------------------------------

# ------------------------------------------------------------------------------


def getHoughline(frame):

	# Some pre-processing
	blur = cv2.GaussianBlur(frame, (5, 5), 0)
	denoise = cv2.fastNlMeansDenoising(blur, None, 10, 7, 21)
	blur = cv2.GaussianBlur(denoise, (5, 5), 0)
	clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
	hist_equal = clahe.apply(blur)

	# Find gradients and lines
	sum_sobel = sobel(hist_equal)
	ret, th = cv2.threshold(sum_sobel, 130, 255, cv2.THRESH_BINARY)
	lines = cv2.HoughLines(th, 1, np.pi/180, 100, 2)

	try:
		if lines == None:
			return frame, 0
	except:
		hough_output = frame.copy()

		# Theta is calculated in the polar-coordinate
		# Find lines with max theta, usually right
		for rho, theta in lines[int(np.where(lines[:, 0, 1] == max(lines[:, 0, 1]))[0][0])]:
			a = np.cos(theta)
			b = np.sin(theta)
			x0 = a*rho
			y0 = b*rho
			x1 = int(x0 + 1000*(-b))
			y1 = int(y0 + 1000*(a))
			x2 = int(x0 - 1000*(-b))
			y2 = int(y0 - 1000*(a))
			cv2.line(hough_output, (x1, y1), (x2, y2), (0, 0, 255), 1)

		# Find lines with min slope, usually left line
		for rho, theta in lines[int(np.where(lines[:, 0, 1] == min(lines[:, 0, 1]))[0][0])]:
			a = np.cos(theta)
			b = np.sin(theta)
			x0 = a*rho
			y0 = b*rho
			x1 = int(x0 + 1000*(-b))
			y1 = int(y0 + 1000*(a))
			x2 = int(x0 - 1000*(-b))
			y2 = int(y0 - 1000*(a))
			cv2.line(hough_output, (x1, y1), (x2, y2), (0, 0, 255), 1)

		return hough_output, len(lines)
# ------------------------------------------------------------------------------


# ==============================================================================
# =============================  Realsense related  ============================
# ------------------------------------------------------------------------------


DEPTH_SHAPE = (848, 480)
linear_regressor = Ridge()


def signed_distance_to_plane(point, coeffs, bias):
	a = (np.dot(point, coeffs)+bias)/np.linalg.norm(coeffs)
	# print("point", point[np.abs(a-0.04) < 0.0002][:, 1])
	# print("distance", a[np.abs(a-0.04) < 0.0002])
	return a


max_plane_finding_accuracy = 0.5
coeffs = np.array([0, 0])
bias = 0.05

# 86 is horizontal field of view of realsense
pub = rospy.Publisher("/realsense_grid", OccupancyGrid, queue_size=3)


def pts2og(pts, angle_step=np.pi/180/8, field_of_view=86/180*np.pi):
	# pts = pts[pts[:, 1] <= 4]
	MAX_RANGE = 3 #meter
	RESOLUTION = 0.02 #centimeter per meter
	RADIUS = np.round(MAX_RANGE/RESOLUTION).astype(np.int32)

	dist = np.linalg.norm(pts, axis=1)
	pts[dist  > MAX_RANGE] = np.array([0,0])
	dist[dist > MAX_RANGE] = 0
	theta = np.arctan2(pts[:, 1], pts[:, 0])

	og_test = np.zeros((250, 250)) 
	og_test_mask = np.zeros((250, 250)) 

	global rounded_theta_boundary_check
	rounded_theta_boundary_check = np.round(np.round(theta/20, 2)*20, 2)
	boundary_angle = np.unique(rounded_theta_boundary_check)
	boundary_angle = boundary_angle[boundary_angle != 0]
	boundary_angle_lower =  - np.round((boundary_angle - 0.1)/np.pi * 180).astype(np.uint8)
	boundary_angle_upper =  - np.round((boundary_angle + 0.1)/np.pi * 180).astype(np.uint8)

	og_test_mask = cv2.ellipse(og_test_mask, (0,125), (RADIUS, RADIUS), 0, \
					 		   0, 180, 255, 1)

	for i in range(len(boundary_angle_lower)):
		og_test = cv2.ellipse(og_test, (0,125), (RADIUS, RADIUS), 0, \
					boundary_angle_lower[i], boundary_angle_upper[i], 255, 1)

	masked_boundary = cv2.bitwise_and(og_test_mask, (255-og_test))
	boundary_pts = np.array(np.where(og_test == 255)).T

	rounded_theta = np.round(theta/5, 2)*5

	ndx = np.lexsort(keys=(-dist, rounded_theta))
	sorted_rounded_theta, sorted_dist = rounded_theta[ndx], dist[ndx]

	out = sorted_dist[np.r_
					  [np.diff(sorted_rounded_theta), True].astype(np.bool)]

	sorted_dist_mask = np.in1d(sorted_dist, out).astype(np.bool)
	sorted_dist_index = ndx[sorted_dist_mask]
	filtered_pts = pts[sorted_dist_index]/RESOLUTION
	filtered_pts = np.append(filtered_pts, np.array([[0, 0]]), axis=0)

	og = np.zeros((250, 250)) - 1
	centered_pts = filtered_pts + np.array([og.shape[0], 0])/2

	out_pts = np.append(centered_pts, masked_boundary, axis=0)
	og = cv2.fillPoly(og, [out_pts.astype(np.int32)], 0)

	#cv2.imshow("og", cv2.flip((og+1)*100, 0))
	return og


def og_msg(occ_grid, map_resolution, map_size, time_stamp):
	MAP_RESOLUTION = map_resolution  # Unit: Meter
	MAP_SIZE = map_size  # Unit: Meter, Shape: Square with center "base_link"
	map_img = cv2.flip(occ_grid, 0)
	map_img = cv2.rotate(map_img, cv2.ROTATE_90_COUNTERCLOCKWISE)
	map_img = cv2.flip(map_img, 1).astype(np.int8)
	occupancy_grid = map_img.flatten()
	occupancy_grid = occupancy_grid.tolist()

	map_msg = OccupancyGrid()

	map_msg.header = Header()
	map_msg.header.frame_id = "base_link"
	map_msg.header.stamp = time_stamp

	map_msg.info = MapMetaData()
	map_msg.info.height = int(MAP_SIZE / MAP_RESOLUTION)  # Unit: Pixel
	map_msg.info.width = int(MAP_SIZE / MAP_RESOLUTION)  # Unit: Pixel
	map_msg.info.resolution = MAP_RESOLUTION

	map_msg.info.origin = Pose()
	map_msg.info.origin.position = Point()
	map_msg.info.origin.position.x = 0  # Unit: Meter
	map_msg.info.origin.position.y = -MAP_SIZE / 2  # Unit: Meter
	map_msg.info.origin.position.z = 0
	map_msg.info.origin.orientation = Quaternion()
	map_msg.info.origin.orientation.x = 0
	map_msg.info.origin.orientation.y = 0
	map_msg.info.origin.orientation.z = 0
	map_msg.info.origin.orientation.w = 1
	map_msg.data.extend(occupancy_grid)
	map_msg.info.map_load_time = rospy.Time.now()
	return map_msg


def axis_mapping(depth_frame, profile):
	# fk realsense frame it had me all wrong
	global max_plane_finding_accuracy, coeffs, bias, has_trained
	points = pc.calculate(depth_frame)
	v, _ = points.get_vertices(), points.get_texture_coordinates()
	# xyz # vertices are in metres unit
	vtx = np.asanyarray(v).view(np.float32).reshape(-1, 3)
	# y_only is a (2, n) dimesion array, not (h,w) dimension array

	radius = 0.3  # metres
	nonzero_pts_index, = np.nonzero(vtx[:, 2] != 0)
	nonzero_pts = vtx[nonzero_pts_index]
	# nonzero_pts_index, = np.where(vtx[:, 2] != 0)

	x_only = nonzero_pts[:, 0]
	z_only = nonzero_pts[:, 2]
	# whole_vertices = vtx.reshape((-1, DEPTH_SHAPE[0], 3))
	# print(whole_vertices[279, 845])
	# assume that we put realsense x m above the ground
	ground2realsense = 0.08
	depth_frame = np.asarray(depth_frame.get_data())
	# print(depth_frame.shape)
	# depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
	depth_frame = cv2.cvtColor(
		(depth_frame / 8000 * 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
	proximity_pts_y = (nonzero_pts[np.sqrt(
		x_only**2+z_only**2) < radius])
	proximity_pts_y = proximity_pts_y[proximity_pts_y[:, 1] > ground2realsense]
	# print("number of proximity point", proximity_pts_y.shape[0])

	train_data = proximity_pts_y[:, (0, 2)]
	target = proximity_pts_y[:, 1]
	try:
		linear_regressor.fit(train_data, target)
		# plane_finding_acc = linear_regressor.score(train_data, target)
		# print("acc: ", plane_finding_acc)
		# max_plane_finding_accuracy = plane_finding_accuracy
		coeffs = np.array([linear_regressor.coef_[0], -
						   1, linear_regressor.coef_[1]])
		bias = linear_regressor.intercept_
		# print("bias:", bias)
		# print("intercept+ coeffs", coeffs,
		#   bias)

		# this is in point cloud coordinate frame, so z point upward
		height_from_ground = 0.02
		ground_approximation_mask_index, = np.nonzero(np.abs(
			signed_distance_to_plane(nonzero_pts, coeffs, bias)-height_from_ground) < 0.001)
		# ground_approximation_plane_mask = np.where(
		#     ground_approximation_mask_index, 1, 0).astype(np.bool)
		# ground_approximation_mask_index, = np.where(
		# ground_approximation_mask_index)
		# print("number of points on the surface:", np.count_nonzero(
		# ground_approximation_plane_mask))
		ground_approximation_plane = nonzero_pts[ground_approximation_mask_index]
		size = 500
		a = np.zeros((size, size)).astype(np.uint8)
		pts = ground_approximation_plane[:, (0, 2)]
		og = pts2og(pts)
		msg = og_msg(og, 0.02, 5, rospy.Time.now())
		pub.publish(msg)
		pts *= 100
		if len(ground_approximation_plane) > 0:
			# print("max z ", np.max(ground_approximation_plane[:, 2]))
			# print("max_x", np.max(ground_approximation_plane[:, 0]))
			# print("max_y", np.max(ground_approximation_plane[:, 1]))
			# print(np.histogram(pts))
			pts[:, 1] *= -1
			pts += np.array([size/2, size-1])
			pts = np.round(pts).astype(np.int)
			for pt in pts:
				a = cv2.circle(a, (pt[0], pt[1]), 1, 255, -1)
		#cv2.imshow("bev", a)
		pts_on_ground = nonzero_pts_index[ground_approximation_mask_index]
		frame_width = DEPTH_SHAPE[0]
		coordx_pt_on_ground = pts_on_ground % frame_width
		coordy_pt_on_ground = pts_on_ground // frame_width
		# print(coordx_pt_on_ground)
		t0 = time.time()
		for i in range(len(pts_on_ground)):
			cv2.circle(
				depth_frame, (coordx_pt_on_ground[i], coordy_pt_on_ground[i]), 1, (255, 0, 0), -1)
		# print("color fps", 1/(time.time()-t0))
		#cv2.imshow("depth frame", depth_frame/100)

	except () as e:
		print(e)
	# this part is for illustration purpose:
	# print(ground_approximation_mask_index, nonzero_pts_index)

	return depth_frame
# ------------------------------------------------------------------------------


# ------------------------------------------------------------------------------
# Realsense config
pipeline = rs.pipeline()
config = rs.config()
config.enable_device_from_file(
	"./realsense_school_day.bag", repeat_playback=True)
# config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 90)
profile = pipeline.start(config)
pc = rs.pointcloud()
# ------------------------------------------------------------------------------

rospy.init_node("realsense_as_lidar", anonymous=True)
# ------------------------------------------------------------------------------
# Realsense-based post-processing
decimation = rs.decimation_filter()
decimation.set_option(rs.option.filter_magnitude, 3)
spatial = rs.spatial_filter()
spatial.set_option(rs.option.filter_magnitude, 3)
spatial.set_option(rs.option.filter_smooth_alpha, 0.5)
spatial.set_option(rs.option.filter_smooth_delta, 50)
temporal = rs.temporal_filter()
# ------------------------------------------------------------------------------

# ==============================================================================
# ================================  MAIN  ======================================
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
# fourcc = cv2.VideoWriter_fourcc(*'H264')
# out = cv2.VideoWriter('demo.mp4', fourcc, 30, (FRAME_WIDTH, FRAME_HEIGHT))
waitTime = 1
align = rs.align(rs.stream.color)
try:
	while True:
		frames = pipeline.wait_for_frames()
		depth_frame = frames.get_depth_frame()  # (h,w,z-axis)
		# normal_frame = align.process(frames).get_color_frame()
		# color_image = np.asanyarray(normal_frame.get_data())
		# #cv2.imshow('color', color_image)

		# decimation_processed    = decimation.process(depth_frame)
		# depth2disparity_processed = depth2disparity.process(depth_frame)
		spatial_processed = spatial.process(depth_frame)
		temporal_processed = temporal.process(spatial_processed)
		depth_img = np.asarray(depth_frame.get_data())/256
		# print("max", np.min(depth_img))
		# #cv2.imshow("depth", depth_img.astype(np.uint8))
		t0 = time.time()
		y_frame = axis_mapping(temporal_processed, profile)
		# print("processing fps: ", 1/(time.time()-t0))
		key = cv2.waitKey(waitTime) & 0xFF
		if key == ord("q"):
			break
		if key == ord("p"):
			cv2.waitKey(0)
		if key == ord("s"):
			waitTime = 200 - waitTime

		# out.write(cv2.cvtColor(hough_output,cv2.COLOR_GRAY2BGR))

finally:
	pipeline.stop()
	# out.release()
	cv2.destroyAllWindows()
