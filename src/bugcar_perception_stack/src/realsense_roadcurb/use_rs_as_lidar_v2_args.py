#!/home/tranquockhue/anaconda3/envs/tf2.2/bin/python
# -*- coding: utf-8 -*-
"""
Created on Fri Dec  6 22:23:44 2019

@author: tranquockhue
"""
import argparse
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from std_msgs.msg import Header, Bool
from nav_msgs.msg import OccupancyGrid, MapMetaData
from sklearn.linear_model import Ridge, RANSACRegressor
import rospy
import cv2
import numpy as np
import pyrealsense2 as rs
import time
from scipy.spatial.transform import Rotation


parser = argparse.ArgumentParser()
parser.add_argument("serial_no", help="serial number of camera", type=str)
parser.add_argument(
    "-f", "--fps", help="fps for depth camera", type=int, default=60)
parser.add_argument("-o",
                    "--orientation",
                    help="the orientation of output occupancy grid w.r.t the head of vehicle (in quartenion)",
                    default=(0, 0, 0, 1),
                    type=float, nargs=4)
parser.add_argument("-p",
                    "--position",
                    help="the position of occ grid center in pixel",
                    type=float, nargs=3)
parser.add_argument('-m', "--map_size", default=5, type=float,
                    help="the size(in m) of occupancy grid. Default to 5")
parser.add_argument('-r', "--map_resolution", default=0.02, type=float,
                    help="how small is each pixel in occupancy grid. Default to 0.02(m)")
parser.add_argument('-s', "--sampling_radius", type=float, default=0.3)
parser.add_argument("-d", "--min_height_from_realsense", type=float, default=0.08,
                    help="min distance from realsense to determine ground, default to 0.08(m)")
parser.add_argument("-g", "--height_from_ground", default=0.04, type=float,
                    help="minimal height of object that you wish to be detected by realsense. Default to 0.04")
parser.add_argument("-t", "--topic_name", type=str, default="/realsense_grid")
parser.add_argument("-e" "--emergency",
                    help="whether to enable emergency", type=bool, default=False)
args, unknowns = parser.parse_known_args()
topic_name = args.topic_name
SERIAL_NUM = args.serial_no
MAP_SIZE = args.map_size
MAP_RESOLUTION = args.map_resolution
ORIENTATION = args.orientation
emergency_handler_enabled = False
FPS = args.fps
sampling_radius = args.sampling_radius
samples_min_height_from_realsense = args.min_height_from_realsense
height_from_ground = args.height_from_ground
if not args.position:
    POSITION = (0, -MAP_SIZE/2, 0)
else:
    POSITION = args.position
print(args)


# ==========parameters and pub/sub system for activating emergency control==========
# ==================================================================================
isEmergency = False
heading = np.pi/2 - Rotation.from_quat(ORIENTATION).as_euler("xyz")[2]
print("heading in radians", heading)
twist_msg = Twist()
twist_msg.linear.x = 0
cmd_vel = Twist()


def emergency_callback(data):
    global cmd_vel
    cmd_vel = data


emergency_topic = "stop_vel/cmd_vel"
normal_control_topic = "move_base/cmd_vel"
# nav_modes = rospy.get_param("/twist_mux/topics")
# for nav_mode in nav_modes:
#     if nav_mode["name"] == "perception_stop":
#         emergency_topic = nav_mode["topic"]
#     if nav_mode["name"] == "auto navigation":
#         normal_control_topic = nav_mode["topic"]
print(normal_control_topic, emergency_topic)
emergency_vel_pub = rospy.Publisher(emergency_topic, Twist, queue_size=1)
rospy.Subscriber(normal_control_topic, Twist, emergency_callback)
# ===================================================================================


def signed_distance_to_plane(point, coeffs, bias):
    # dont use np.dot here or cpu usage will skyrocket.
    # https://www.pugetsystems.com/labs/hpc/How-To-Use-MKL-with-AMD-Ryzen-and-Threadripper-CPU-s-Effectively-for-Python-Numpy-And-Other-Applications-1637/
    a = (point[:, 0]*coeffs[0]+point[:, 1]*coeffs[1] +
         point[:, 2]*coeffs[2]+bias)/np.linalg.norm(coeffs)
    return a


def pts2og(pts, angle_step=np.pi/180/4, field_of_view=86.0/180.0*np.pi):
    grid_size = int(MAP_SIZE/MAP_RESOLUTION)
    og = np.zeros((grid_size, grid_size))-1

    # define the minimum angle and max angle the camera can cover
    angle_min = np.pi/2 - field_of_view/2
    angle_max = np.pi/2 + field_of_view/2  # - 3/180*np.pi
    max_distance = 4  # points that are above this distance will be discarded from og
    angle_bin = np.arange(start=angle_min, stop=angle_max, step=angle_step)
    bin_num = angle_bin.shape[0]
    x_max = max_distance*np.cos(angle_bin)
    y_max = max_distance*np.sin(angle_bin)
    placeholder_pts = np.stack([x_max, y_max], axis=1)

    y = pts[:, 1]
    pts = pts[y <= max_distance]
    theta = np.arctan2(pts[:, 1], pts[:, 0])
    angle_is_valid = np.logical_and(
        theta >= angle_min, theta <= angle_max)
    pts = pts[angle_is_valid]
    theta = theta[angle_is_valid]
    binned_theta = np.round(
        (theta-angle_min)/angle_step).astype(np.int)
    dist = np.linalg.norm(pts, axis=1)
    sorted_by_theta_and_dist_index = np.lexsort((dist, binned_theta))
    angle, index_new = np.unique(
        binned_theta[sorted_by_theta_and_dist_index], return_index=True)
    sorted_by_theta_and_dist_index = sorted_by_theta_and_dist_index[index_new]
    sorted_by_angle_occupied_pts = pts[sorted_by_theta_and_dist_index]
    new_pts = sorted_by_angle_occupied_pts

    # add default value to rightmost and leftmost missing angles that are not provided by laser detector.
    if len(angle) > 0:
        # print(angle[[0, -1]], bin_num)
        if angle[0] > 0:
            new_pts = np.concatenate([placeholder_pts[0:angle[0]+1], new_pts])
        if angle[-1] < bin_num-1:
            # print(new_pts.shape, placeholder_pts[angle[-1]:bin_num].shape)
            new_pts = np.concatenate(
                [new_pts, placeholder_pts[angle[-1]:bin_num]], axis=0)
    else:
        new_pts = placeholder_pts

    new_pts = np.append(
        new_pts, np.array([[0, 0]]), axis=0)/MAP_RESOLUTION
    origin = np.array([grid_size/2, 0])
    new_pts += origin
    new_pts = new_pts.astype(np.int32)
    og = cv2.fillPoly(og, [new_pts], 0)
    for pt in (sorted_by_angle_occupied_pts/MAP_RESOLUTION + origin).astype(np.int):
        cv2.circle(og, (pt[0], pt[1]), radius=1, color=100, thickness=-1)
    og = cv2.dilate(og, np.ones((3, 3)))
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
    map_msg.info.origin.position.x = POSITION[0]  # Unit: Meter
    # -MAP_SIZE / 2  # Unit: Meter
    map_msg.info.origin.position.y = POSITION[1]
    map_msg.info.origin.position.z = POSITION[2]
    map_msg.info.origin.orientation = Quaternion()
    map_msg.info.origin.orientation.x = ORIENTATION[0]
    map_msg.info.origin.orientation.y = ORIENTATION[1]
    map_msg.info.origin.orientation.z = ORIENTATION[2]
    map_msg.info.origin.orientation.w = ORIENTATION[3]
    map_msg.data.extend(occupancy_grid)
    map_msg.info.map_load_time = rospy.Time.now()
    return map_msg


def depth_frame_2_occ_grid(depth_frame):
    global coeffs, bias
    global twist_msg, cmd_vel, heading
    points = pc.calculate(depth_frame)

    v, _ = points.get_vertices(), points.get_texture_coordinates()
    # xyz # vertices are in metres unit
    vtx = np.asanyarray(v).view(np.float32).reshape(-1, 3)
    # y_only is a (2, n) dimesion array, not (h,w) dimension array

    nonzero_pts_index, = np.nonzero(vtx[:, 2] != 0)
    nonzero_pts = vtx[nonzero_pts_index]
    nonzero_pts[:,[0,1]] = -nonzero_pts[:,[0,1]]
    x_only = nonzero_pts[:, 0]
    z_only = nonzero_pts[:, 2]

    # proximity_camera_area = (nonzero_pts[np.sqrt(
    #     x_only**2+z_only**2) < sampling_radius])
    proximity_camera_area = (nonzero_pts[np.sqrt(
        z_only**2) < sampling_radius])
    proximity_pts_near_ground = proximity_camera_area[proximity_camera_area[:, 1]
                                                      > samples_min_height_from_realsense]

    # ==========================Handle emergency control here===============================
    if emergency_handler_enabled:
        
        speed = cmd_vel.linear.x
        emergency_range = np.clip(np.abs(speed*0.5), 0.3, 0.5)
        emergency_area = nonzero_pts[z_only < emergency_range]
        emergency_area_y = emergency_area[emergency_area[:, 1]
                                          > samples_min_height_from_realsense]
        if emergency_area_y.shape[0] < emergency_area.shape[0]*9/10:
            print("EMERGENCY_STOP! ,OBJECT TOO CLOSE!")
            grid_size = int(MAP_SIZE/MAP_RESOLUTION)
            if heading*cmd_vel.linear.x > 0.0:
                emergency_vel_pub.publish(twist_msg)
            og = (np.ones((grid_size, grid_size))*100)
            msg = og_msg(og, MAP_RESOLUTION, MAP_SIZE, rospy.Time.now())
            pub.publish(msg)
            return
    # =======================================================================================

    train_data = proximity_pts_near_ground[:, (0, 2)]
    target = proximity_pts_near_ground[:, 1]
    try:
        linear_regressor.fit(train_data, target)

        # finding score then retrain is much slower than training directly
        # plane_finding_acc = linear_regressor.score(train_data, target)

        coeffs = np.array([linear_regressor.coef_[0], -
                           1, linear_regressor.coef_[1]])
        bias = linear_regressor.intercept_

        # this is in point cloud coordinate frame, so z axis  points upward
        ground_approximation_mask_index, = np.nonzero(np.abs(
            signed_distance_to_plane(nonzero_pts, coeffs, bias)-height_from_ground) < 0.001)
        # max_detectable_height = 0.3
        # average = (max_detectable_height+height_from_ground) /2
        # distance = max_detectable_height - average
        # ground_approximation_mask_index, = np.nonzero(np.abs(
        #     signed_distance_to_plane(nonzero_pts, coeffs, bias)-average) < distance) 
        ground_approximation_plane = nonzero_pts[ground_approximation_mask_index]

        pts = ground_approximation_plane[:, (0, 2)]
        og = pts2og(pts)
        msg = og_msg(og, MAP_RESOLUTION, MAP_SIZE, rospy.Time.now())
        pub.publish(msg)
    except (ValueError) as e:
        print(e)
# ------------------------------------------------------------------------------


# ------------------------------------------------------------------------------
# Realsense config
DEPTH_SHAPE = (848, 480)
linear_regressor = Ridge(copy_X=False)
coeffs = np.array([0, 0])
bias = 0.05

# 86 is horizontal field of view of realsense
rospy.init_node("realsense_as_lidar", anonymous=True, disable_signals=True)
pub = rospy.Publisher(topic_name, OccupancyGrid, queue_size=2)
pipeline = rs.pipeline()
rate = rospy.Rate(15)
config = rs.config()
# config.enable_device_from_file(
# "./realsense_school_day.bag", repeat_playback=True)
# config.enable_device("819312071039")
config.enable_device(SERIAL_NUM)
config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, FPS)
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
try:
    while True:
        t0 = time.time()
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()  # (h,w,z-axis)
        decimation_processed = decimation.process(depth_frame)
        spatial_processed = spatial.process(depth_frame)
        temporal_processed = temporal.process(spatial_processed)

        depth_frame_2_occ_grid(temporal_processed)
        rate.sleep()
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
