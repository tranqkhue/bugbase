#!/home/tranquockhue/anaconda3/envs/tf2.2/bin/python
# -*- coding: utf-8 -*-
"""
Created on Fri Dec  6 22:23:44 2019

@author: tranquockhue
"""
import argparse
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from realsense_roadcurb.road_curb_detector import RoadCurbDetector
from std_msgs.msg import Header, Bool
from nav_msgs.msg import OccupancyGrid, MapMetaData
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


detector = RoadCurbDetector()


# 86 is horizontal field of view of realsense
rospy.init_node("realsense_as_lidar", anonymous=True, disable_signals=True)
pub = rospy.Publisher(topic_name, OccupancyGrid, queue_size=2)
pipeline = rs.pipeline()
rate = rospy.Rate(15)

# Realsense config
config = rs.config()
# config.enable_device_from_file(
# "./realsense_school_day.bag", repeat_playback=True)
# config.enable_device("819312071039")
config.enable_device(SERIAL_NUM)
config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, FPS)
profile = pipeline.start(config)
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

        og_msg = detector.depth_frame_2_occ_grid(temporal_processed)
        pub.publish(og_msg)
        rate.sleep()
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
