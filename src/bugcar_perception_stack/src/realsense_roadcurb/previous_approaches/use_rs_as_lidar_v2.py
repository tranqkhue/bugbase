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

import rospy
from sklearn.linear_model import Ridge
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion

# ==============================================================================
# ============================  OpenCV procesisng  =============================


# ==============================================================================
# =============================  Realsense related  ============================
# ------------------------------------------------------------------------------


def signed_distance_to_plane(point, coeffs, bias,b):
    print(coeffs.shape)
    print(point.shape)
    a = (np.matmul(point,coeffs)+bias) / b
    return a


def points_to_laser(pts, angle_step=np.pi/180/8, field_of_view=86/180*np.pi):
    x = pts[:, 0]
    y = pts[:, 1]
    distance = np.sqrt(x**2 + y**2)
    num_of_observation = int(field_of_view/angle_step)
    range_list = np.ones((num_of_observation,))*np.Inf
    cos_pts = x / distance
    rad_pts = np.arccos(cos_pts)
    sorted_rad_index = np.argsort(rad_pts)
    angle_min = np.pi/2 - field_of_view/2
    angle_max = np.pi/2 + field_of_view/2
    place_in_range_list = np.round(
        (rad_pts-angle_min)/angle_step).astype(np.uint8)
    index = np.lexsort((distance, place_in_range_list))
    _, index_new = np.unique(place_in_range_list[index])
    pts = pts[index_new]

    print("start")
    # for i in sorted_rad_index:
    #     place_in_range_list = int((rad_pts[i]-angle_min) / angle_step)
    #     if place_in_range_list > num_of_observation:
    #         continue
    #     if distance[i] < range_list[place_in_range_list]:
    #         range_list[place_in_range_list] = distance[i]
    #         legit_pts.append(pts[i])
    #         # print(distance)
    # return range_list


def pts2og(pts, angle_step=np.pi/180/4, field_of_view=84/180*np.pi):
    og = np.zeros((250, 250))-1
    angle_min = np.pi/2 - field_of_view/2
    angle_max = np.pi/2 + field_of_view/2
    max_distance = 3
    y = pts[:, 1]
    pts = pts[y <= max_distance]
    theta = np.arctan2(pts[:, 1], pts[:, 0])
    angle_is_valid = np.logical_and(
        theta > angle_min, theta < angle_max)
    pts = pts[angle_is_valid]
    theta = theta[angle_is_valid]
    binned_theta = np.round(
        (theta-angle_min)/angle_step).astype(np.int)
    dist = np.linalg.norm(pts, axis=1)
    angle_bin = np.arange(start=angle_min, stop=angle_max, step=angle_step)
    bin_num = angle_bin.shape[0]
    x_max = max_distance*np.cos(angle_bin)
    y_max = max_distance*np.sin(angle_bin)
    placeholder_pts = np.stack([x_max, y_max], axis=1)
    print("placeholder shape", placeholder_pts.shape)
    sorted_by_theta_and_dist_index = np.lexsort((dist, binned_theta))
    # print("shape", index.shape)
    # print("binned_shape", binned_theta.shape)
    angle, index_new = np.unique(
        binned_theta[sorted_by_theta_and_dist_index], return_index=True)
    # print(angle)
    # print("index new", index_new)
    sorted_by_theta_and_dist_index = sorted_by_theta_and_dist_index[index_new]
    # print("binned angle", angle)
    # print("sorted + unique indices", index)
    sorted_by_angle_occupied_pts = pts[sorted_by_theta_and_dist_index]
    new_pts = sorted_by_angle_occupied_pts
    if len(angle) > 0:
        if angle[0] > 0:
            print("ok")
            new_pts = np.concatenate([placeholder_pts[0:angle[0]+1], new_pts])
        if angle[-1] < bin_num:
            print(new_pts.shape, placeholder_pts[angle[-1]:bin_num].shape)
            new_pts = np.concatenate(
                [new_pts, placeholder_pts[angle[-1]:bin_num]], axis=0)
    else:
        new_pts = placeholder_pts

    print("shape newpts", new_pts.shape)
    new_pts = np.append(
        new_pts, np.array([[0, 0]]), axis=0)/0.02
    origin = np.array([125, 0])
    new_pts += origin
    new_pts = new_pts.astype(np.int32)
    og = cv2.fillPoly(og, [new_pts], 0)
    for pt in (sorted_by_angle_occupied_pts/0.02 + origin).astype(np.int):
        cv2.circle(og, (pt[0], pt[1]), radius=3, color=100, thickness=-1)
    return og


def og_msg(occ_grid, map_resolution, map_size, time_stamp):
    MAP_RESOLUTION = map_resolution  # Unit: Meter
    MAP_SIZE = map_size  # Unit: Meter, Shape: Square with center "base_link"
    map_img = cv2.rotate(
        occ_grid, cv2.ROTATE_90_COUNTERCLOCKWISE).astype(np.int8)

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


def depth_frame_2_occ_grid(depth_frame):
    global coeffs, bias
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

    proximity_camera_area = (nonzero_pts[np.sqrt(
        x_only**2+z_only**2) < radius])
    proximity_pts_y = proximity_camera_area[proximity_camera_area[:, 1] > ground2realsense]
    # if proximity_pts_y.shape[0] < proximity_camera_area.shape[0]/3:
    #     print("EMERGENCY_STOP! ,OBJECT TOO CLOSE!")
    #     og = (np.ones((250,250))*100)
    #     msg = og_msg(og, 0.02, 5, rospy.Time.now())
    #     pub.publish(msg)
    #     return 
    # print("number of proximity point", proximity_pts_y.shape[0])
    train_data = proximity_pts_y[:, (0, 2)]
    target = proximity_pts_y[:, 1]
    try:
        linear_regressor.fit(train_data, target)
        # finding score then retrain is much slower than training directly

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
        height_from_ground = 0.04
        b = np.linalg.norm(coeffs)
        coeffs = np.reshape(coeffs,(3,1))
        ground_approximation_mask_index, = np.nonzero(np.abs(
            signed_distance_to_plane(nonzero_pts, coeffs, bias,b)-height_from_ground) < 0.001)

        # print("number of points on the surface:", np.count_nonzero(
        # ground_approximation_plane_mask))
        ground_approximation_plane = nonzero_pts[ground_approximation_mask_index]

        pts = ground_approximation_plane[:, (0, 2)]
        og = pts2og(pts)
        msg = og_msg(og, 0.02, 5, rospy.Time.now())
        pub.publish(msg)

        # for illustration purpose
        pts *= 100
        size = 500
        a = np.zeros((size, size)).astype(np.uint8)
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
      ##cv2.imshow("bev", a)

        # this part is for illustration purpose:
        # print(ground_approximation_mask_index, nonzero_pts_index)
        pts_on_ground = nonzero_pts_index[ground_approximation_mask_index]
        frame_width = DEPTH_SHAPE[0]
        coordx_pt_on_ground = pts_on_ground % frame_width
        coordy_pt_on_ground = pts_on_ground // frame_width
        # print(coordx_pt_on_ground)
        # t0 = time.time()
        depth_frame = np.asarray(depth_frame.get_data())
        # print(depth_frame.shape)
        # depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        depth_frame = cv2.cvtColor(
            (depth_frame / 8000 * 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
        for i in range(len(pts_on_ground)):
            cv2.circle(
                depth_frame, (coordx_pt_on_ground[i], coordy_pt_on_ground[i]), 1, (255, 0, 0), -1)
        # print("color fps", 1/(time.time()-t0))
      ##cv2.imshow("depth frame", depth_frame.astype(np.uint8))

    except (ValueError) as e:
        print(e)
# ------------------------------------------------------------------------------


# ------------------------------------------------------------------------------
# Realsense config
DEPTH_SHAPE = (848, 480)
linear_regressor = Ridge()
coeffs = np.array([0, 0])
bias = 0.05

# 86 is horizontal field of view of realsense
rospy.init_node("realsense_as_lidar", anonymous=True)
pub = rospy.Publisher("/realsense_grid", OccupancyGrid, queue_size=3)
pipeline = rs.pipeline()
config = rs.config()
config.enable_device_from_file("./realsense_school_day.bag", repeat_playback=True)
#config.enable_device("819312071039")
#config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 60)
profile = pipeline.start(config)
pc = rs.pointcloud()
# ------------------------------------------------------------------------------

# ------------------------------------------------------------------------------
# Realsense-based post-processing
decimation = rs.decimation_filter()
decimation.set_option(rs.option.filter_magnitude, 2)
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
waitTime = 1
try:
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()  # (h,w,z-axis)
        decimation_processed = decimation.process(depth_frame)
        spatial_processed = spatial.process(depth_frame)
        temporal_processed = temporal.process(spatial_processed)

        depth_frame_2_occ_grid(temporal_processed)

        key = cv2.waitKey(waitTime) & 0xFF
        if key == ord("q"):
            break
        if key == ord("p"):
            cv2.waitKey(0)
        if key == ord("s"):
            waitTime = 200 - waitTime
        #time.sleep(0.05)

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
