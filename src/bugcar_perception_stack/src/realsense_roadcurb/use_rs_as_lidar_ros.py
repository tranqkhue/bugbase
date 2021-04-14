#!/home/thang/anaconda3/envs/tf2env/bin/python
#!/root/miniconda3/envs/tf2.2/bin/python
# -*- coding: utf-8 -*-
"""
Created on Fri Dec  6 22:23:44 2019

@author: tranquockhue
"""

from emergency_stop import Emergency
from road_curb_detector import RoadCurbDetector
from nav_msgs.msg import OccupancyGrid
import rospy
import cv2
import numpy as np
import pyrealsense2 as rs
import time


rospy.init_node("realsense_as_lidar", anonymous=True, disable_signals=True)
rate = rospy.Rate(15)


# =====================SECTION FOR COLLECTING PARAMETERS====================
node_name, node_namespace = rospy.get_name(), rospy.get_namespace()
params_dict = {"map_resolution": 0, "map_size": 0, "orientation": 0, "position": 0,
               "sampling_radius": 0,
               "min_height_from_realsense":  0, "height_from_ground": 0,
               "emergency": False,
               "topic_name": "", "serial_no": "",
               "fps": 0
               }
arg_list_for_detector = []
for param in params_dict:
    try:
        value = rospy.get_param(node_name+"/"+param)
        print(value)
    except KeyError:  # rospy cannot find the desired parameters
        value = rospy.get_param("/road_curb/"+param)
        # if value is None:
        #     raise ValueError("you lack a parameter: " + param)
    params_dict[param] = value
    arg_list_for_detector.append(value)
arg_list_for_detector = arg_list_for_detector[:8]
detector = RoadCurbDetector(*arg_list_for_detector)

# ===========================================================================
# ===========================================================================


pub = rospy.Publisher(
    node_namespace+params_dict["topic_name"], OccupancyGrid, queue_size=2)
# Also setup an Emergency object here:
emergency_handler = Emergency(params_dict["orientation"])


# ========================Realsense configuratiom===============================
SERIAL_NUM = params_dict['serial_no']
FPS = params_dict['fps']
pipeline = rs.pipeline()
config = rs.config()
# config.enable_device_from_file(
# "./realsense_school_day.bag", repeat_playback=True)
# config.enable_device("819312071039")
config.enable_device(SERIAL_NUM)
config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, FPS)
profile = pipeline.start(config)
# =============================================================================

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
        emergency_handler.check_condition_and_stop(detector.near_collision)
        pub.publish(og_msg)
        rate.sleep()
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
