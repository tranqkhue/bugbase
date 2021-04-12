import rospy
from geometry_msgs.msg import Twist
from scipy.spatial.transform import Rotation
import numpy as np
# ==================================================================================
# ==========parameters and pub/sub system for activating emergency control==========


# ==================================================================================
# ==================================================================================


class Emergency:
    twist_msg = Twist()
    twist_msg.linear.x = 0
    cmd_vel = Twist()
    emergency_topic = "stop_vel/cmd_vel"
    normal_control_topic = "move_base/cmd_vel"
    emergency_vel_pub = rospy.Publisher(emergency_topic, Twist, queue_size=1)

    def __init__(self, camera_orientation) -> None:
        self.heading = np.pi/2 - \
            Rotation.from_quat(camera_orientation).as_euler("xyz")[2]
        self.subscriber = rospy.Subscriber(self.normal_control_topic,
                                           Twist, self.emergency_callback)

    def emergency_callback(self, data):
        self.cmd_vel = data

    def check_condition_and_stop(self, near_collision):
        if self.heading*self.cmd_vel.linear.x > 0 and near_collision:
            self.emergency_vel_pub.publish(self.twist_msgs)
