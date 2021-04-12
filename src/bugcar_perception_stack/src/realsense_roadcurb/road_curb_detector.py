from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid, MapMetaData
from sklearn.linear_model import Ridge
import rospy
import cv2
import numpy as np
import pyrealsense2 as rs


def signed_distance_to_plane(point, coeffs, bias):
    # dont use np.dot here or cpu usage will skyrocket.
    # https://www.pugetsystems.com/labs/hpc/How-To-Use-MKL-with-AMD-Ryzen-and-Threadripper-CPU-s-Effectively-for-Python-Numpy-And-Other-Applications-1637/
    a = (point[:, 0]*coeffs[0]+point[:, 1]*coeffs[1] +
         point[:, 2]*coeffs[2]+bias)/np.linalg.norm(coeffs)
    return a


class RoadCurbDetector:
    """
    a class that implement RoadCurb
    """
    linear_regressor = Ridge()
    pc = rs.pointcloud()
    near_collision = False

    def __init__(self, map_resolution=0.02, map_size=5, orientation=(0, 0, 0, 1), position=(0, 0, 0),
                 sampling_radius=0.3, min_height_from_realsense=0.08, height_from_ground=0.04, emergency=False):
        self.MAP_RESOLUTION = map_resolution
        self.MAP_SIZE = map_size
        self.orientation = orientation
        self.position = position
        self.sampling_radius = sampling_radius
        self.min_height_from_realsense = min_height_from_realsense
        self.height_from_ground = height_from_ground
        self.emergency_handler_enabled = emergency

    def __pts2og(self, pts, angle_step=np.pi/180/4, field_of_view=86.0/180.0*np.pi):
        MAP_SIZE = self.MAP_SIZE
        MAP_RESOLUTION = self.MAP_RESOLUTION
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
                new_pts = np.concatenate(
                    [placeholder_pts[0:angle[0]+1], new_pts])
            if angle[-1] < bin_num-1:
                # print(new_pts.shape, placeholder_pts[angle[-1]:bin_num].shape)
                new_pts = np.concatenate(
                    [new_pts, placeholder_pts[angle[-1]:bin_num]], axis=0)
        else:
            new_pts = placeholder_pts

        # draw the laser points on the canvas
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

    def _og_msg(self, occ_grid, map_resolution, map_size, time_stamp):
        ORIENTATION = self.orientation
        POSITION = self.position
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

    def depth_frame_2_occ_grid(self, depth_frame):
        MAP_SIZE = self.MAP_SIZE
        MAP_RESOLUTION = self.MAP_RESOLUTION
        points = self.pc.calculate(depth_frame)
        v, _ = points.get_vertices(), points.get_texture_coordinates()
        # xyz # vertices are in metres unit
        vtx = np.asanyarray(v).view(np.float32).reshape(-1, 3)
        # y_only is a (2, n) dimesion array, not (h,w) dimension array

        nonzero_pts_index, = np.nonzero(vtx[:, 2] != 0)
        nonzero_pts = vtx[nonzero_pts_index]
        nonzero_pts[:, [0, 1]] = -nonzero_pts[:, [0, 1]]
        z_only = nonzero_pts[:, 2]

        proximity_camera_area = (nonzero_pts[np.sqrt(
            z_only**2) < self.sampling_radius])
        proximity_pts_near_ground = proximity_camera_area[proximity_camera_area[:, 1]
                                                          > self.samples_min_height_from_realsense]

        # ==========================Handle emergency control here===============================
        if self.emergency_handler_enabled:
            emergency_range = 0.4  # np.clip(np.abs(speed*0.5), 0.3, 0.5)
            emergency_area = nonzero_pts[z_only < emergency_range]
            emergency_area_y = emergency_area[emergency_area[:, 1]
                                              > self.min_height_from_realsense]
            if emergency_area_y.shape[0] < emergency_area.shape[0]*9/10:
                self.near_collision = True
                print("EMERGENCY_STOP! ,OBJECT TOO CLOSE!")
                grid_size = int(MAP_SIZE/MAP_RESOLUTION)
                og = (np.ones((grid_size, grid_size))*100)
                msg = self._og_msg(og, MAP_RESOLUTION,
                                   MAP_SIZE, rospy.Time.now())
                return msg
            self.near_collision = False
        # =======================================================================================

        train_data = proximity_pts_near_ground[:, (0, 2)]
        target = proximity_pts_near_ground[:, 1]
        try:
            linear_regressor = self.linear_regressor
            linear_regressor.fit(train_data, target)

            # finding score then retrain is much slower than training directly
            # plane_finding_acc = linear_regressor.score(train_data, target)

            coeffs = np.array([linear_regressor.coef_[0], -
                               1, linear_regressor.coef_[1]])
            bias = linear_regressor.intercept_

            # this is in point cloud coordinate frame, so z axis  points upward
            ground_approximation_mask_index, = np.nonzero(np.abs(
                signed_distance_to_plane(nonzero_pts, coeffs, bias)-self.height_from_ground) < 0.001)
            ground_approximation_plane = nonzero_pts[ground_approximation_mask_index]

            pts = ground_approximation_plane[:, (0, 2)]
            og = self.__pts2og(pts)
            msg = self._og_msg(og, MAP_RESOLUTION, MAP_SIZE, rospy.Time.now())
            return msg
        except (ValueError) as e:
            raise ValueError
