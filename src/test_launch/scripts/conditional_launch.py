#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import roslaunch
import tf
import numpy as np

from geometry_msgs.msg import Twist
from bno055_usb_stick_msgs.msg import Output as BNO055_OUTPUT
from bno055_usb_stick_msgs.msg import CalibrationStatus as BNO055_STATUS
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from nav_msgs.msg    import Odometry

import diagnostic_updater
import diagnostic_msgs

class SensorDiagnostics():
    def __init__(self):
        self.imu_stat = 0
        self.odom_stat = 0
        self.gps_stat = {'gps_Hacc' : 9999,\
                         'gps_Vacc' : 9999}

        self.sensor_checker = SensorLaunch()

        self.process_id = str()

    def produce_diagnostics(self, stat):
        stat.add('Running process', self.process_id)
        stat.add('IMU status', self.imu_stat)
        stat.add('ODOM status', self.odom_stat)
        stat.add('GPS Horizontal Accuracy', self.gps_stat['gps_Hacc'])
        stat.add('GPS Vertical Accuracy', self.gps_stat['gps_Vacc'])

    def run_checker(self):
        self.process_id = 'Imu status wait'
        while self.sensor_checker.checklist['imu']:
            pass
        self.imu_stat = "READY"
        rospy.sleep(0.5)

        self.process_id = 'Odometry test'
        self.sensor_checker.vel_test_odom()
        self.odom_stat = "READY"
        rospy.sleep(0.5)

        self.process_id = 'Gps status wait'
        while self.sensor_checker.checklist['gps']:
            pass
        self.gps_stat['gps_Hacc'] = self.sensor_checker.gps_Hacc
        self.gps_stat['gps_Vacc'] = self.sensor_checker.gps_Vacc
        rospy.sleep(0.5)


class SensorLaunch():
    def __init__(self):
        self.gps_stat_sub = rospy.Subscriber('/gps/fix', NavSatFix, self.gps_status_callback, queue_size=1)
        self.imu_stat_sub = rospy.Subscriber('/output', BNO055_OUTPUT, self.imu_status_callback, queue_size=1)
        self.raw_odom_sub = rospy.Subscriber('/odom/wheel', Odometry, self.raw_odom_callback, queue_size=1)
        self.raw_imu_sub  = rospy.Subscriber('/imu/data', Imu, self.raw_imu_callback, queue_size=1) 
        self.vel_test_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        self.imu_status = BNO055_STATUS()
        self.gps_status = NavSatFix()
        self.imu_data   = Imu()
        self.odom_data  = Odometry()

        self.imu_orientation = list()
        self.odom_orientation = list()

        self.test_vel   = Twist()
        self.max_test_vel_value = 1    # rad/s
        self.test_vel_step      = 0.1  # rad/s
        self.test_vel_tolerance = 0.01 # relative tolerance

        self.gps_Hacc = float();
        self.gps_Vacc = float();

        self.check_list = {'gps' : False,\
                           'imu' : False,\
                           'odom': False}

        ''' Roslaunch section '''
        self.main_folder       = "/home/quan/bugcar/src/bugcar_bringup/"
        self.gps_path          = self.main_folder + "launch/single_gps.launch"
        self.motor_driver_path = self.main_folder + "launch/roboclaw.launch"
        self.imu_path          = self.main_folder + "launch/bno055.launch"
        self.rc_path           = self.main_folder + "launch/rc_only.launch"

        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)

        self.sensor_launch = roslaunch.parent.ROSLaunchParent(self.uuid, [self.gps_path, self.imu_path, self.motor_driver_path])        

    def raw_odom_callback(self,data):
        self.odom_data = data
        self.odom_orientation = [self.odom_data.pose.pose.orientation.x,\
                                 self.odom_data.pose.pose.orientation.y,\
                                 self.odom_data.pose.pose.orientation.z,\
                                 self.odom_data.pose.pose.orientation.w]

    def raw_imu_callback(self,data):
        self.imu_data = data
        self.imu_orientation = [self.imu_data.orientation.x,\
                                self.imu_data.orientation.y,\
                                self.imu_data.orientation.z,\
                                self.imu_data.orientation.w]

    def imu_status_callback(self,data):
        self.imu_status = data
        if self.imu_status.calibration_status.system == 3:
            self.check_list['imu'] = True
    
    def gps_status_callback(self,data):
        gps_covariance = data.position_covariance
        self.gps_Hacc = np.sqrt(gps_covariance[0])
        self.gps_Vacc = np.sqrt(gps_covariance[8])
        if self.gps_Hacc < 10.0 and self.gps_Vacc < 10.0:
            self.check_list['gps'] = True

    def vel_test_odom(self):
        wait_duration = rospy.Duration(2, 0)
        self.test_vel.angular.z = 0.0

        for i in range(int(self.max_test_vel_value / self.test_vel_step)):
            # Make sure that the imu is not disconnected and reinitialize during run
            if self.imu_status.calibration_status.system != 3:
                rospy.logerr('Imu is not calibrated properly')
                rospy.logerr('Please check for stable power source connection and re-calibrate')
                return False

            self.test_vel.angular.z += self.test_vel_step
            self.vel_test_pub.publish(self.test_vel)
            #rospy.sleep(0.1)
            rospy.sleep(wait_duration)

            error = np.allclose(tf.transformations.euler_from_quaternion(self.imu_orientation),\
                                tf.transformations.euler_from_quaternion(self.odom_orientation),\
                                rtol = self.test_vel_tolerance)
            if error:
                rospy.logwarn('At ' + str(self.test_vel.angular.z) + ': \nRelative error between odometry and imu is greater than ' + str(self.test_vel_tolerance))
                rospy.logwarn('     Imu orientation report: ' + str(tf.transformations.euler_from_quaternion(self.imu_orientation)))
                rospy.logwarn('Odometry orientation report: ' + str(tf.transformations.euler_from_quaternion(self.odom_orientation)))

                return False
        
        return True

if __name__ == '__main__':
    rospy.init_node('custom_launch')
    sensor_diag = SensorDiagnostics()
    updater = diagnostic_updater.Updater()
    updater.setHardwareID("none")
    updater.add("Sensor Diagnostics", sensor_diag.produce_diagnostics)
    sensor_diag.run_checker()

    try:
        while not rospy.is_shutdown():
            rospy.sleep(1)
            updater.update()

    except SystemExit:
        pass    
