#!/usr/bin/env python

import rospy
import tf as ros_tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import numpy as np
import serial
import time

#=========================================================================
def stepper_params_loader():
	INITIAL_CHAR = "i"
	STEP_EN    = str(rospy.get_param("~enable_pin", 8))
	STEP_A_PIN = str(rospy.get_param("~stp_a_pin",  2))
	STEP_B_PIN = str(rospy.get_param("~stp_b_pin",  3))
	DIR_A_PIN  = str(rospy.get_param("~dir_a_pin",  5))
	DIR_B_PIN  = str(rospy.get_param("~dir_b_pin",  6))

	initial_string = INITIAL_CHAR+"|"+STEP_EN+"|"+STEP_A_PIN+"|"+\
					 STEP_B_PIN+"|"+DIR_A_PIN+"|"+DIR_B_PIN+"\n"
	ser.write(initial_string.encode())
	output = ser.readline
	'''
	if (output == "DI\r\n"):
		time.sleep(0.5)
		return True
	else:
		return False
	'''
	return True
#=========================================================================

#=========================================================================
def calculate_odom():
	global last_time
	global twist_linear_x 
	global twist_angular_z 

	global odom_pos_x 
	global odom_pos_y
	global odom_angular_z

	ros_current_time= rospy.Time.now()
	current_time	= ros_current_time.to_sec()
	dt 				= current_time - last_time
	delta_pos_x 	= (twist_linear_x * np.cos(odom_angular_z)) * dt
	delta_pos_y 	= (twist_linear_x * np.sin(odom_angular_z)) * dt
	delta_angular_z = twist_angular_z * dt

	odom_pos_x = odom_pos_x + delta_pos_x
	odom_pos_y = odom_pos_y + delta_pos_y
	odom_angular_z = odom_angular_z + delta_angular_z
	odom_quat_orientation = ros_tf.transformations.quaternion_from_euler(\
							0, 0, odom_angular_z)

	#Initialize the message
	odom_msg = Odometry()
	odom_msg.header.stamp 	 = ros_current_time
	odom_msg.header.frame_id = "odom"
	odom_msg.child_frame_id  = "base_link"

	#Position
	odom_msg.pose.pose.position.x = odom_pos_x
	odom_msg.pose.pose.position.y = odom_pos_y
	#odom_msg.pose.pose.position.z = 0.0

	#Orientation
	#odom_msg.pose.pose.orientation.x = odom_quat_orientation[0]
	#odom_msg.pose.pose.orientation.y = odom_quat_orientation[1]
	odom_msg.pose.pose.orientation.z = odom_quat_orientation[2]
	odom_msg.pose.pose.orientation.w = odom_quat_orientation[3]

	#Velocity
	#Remember: velocity is measured on the base_link frame!
	odom_msg.twist.twist.linear.x   = twist_linear_x
	#odom_msg.twist.twist.linear.y  = 0
	#odom_msg.twist.twist.linear.z  = 0
	#odom_msg.twist.twist.angular.x = 0
	#odom_msg.twist.twist.angular.y = 0
	odom_msg.twist.twist.angular.z  = twist_angular_z

	last_time = current_time

	return odom_msg
#=========================================================================

#=========================================================================
def send_cmd(v_left_steps, v_right_steps):
	cmd = str(round(v_left_steps,2))+"|"+str(round(v_right_steps,2))+"\n"
	ser.write(cmd.encode())

def inversed_kinematics(linear_x, angular_z):
	global WHEEL_BASE
	global STEPS_PER_METER
	global LEFT_INVERSED 
	global RIGHT_INVERSED

	v_left  = (linear_x - angular_z * WHEEL_BASE/2.0) #Unit: m/s
	v_right = (linear_x + angular_z * WHEEL_BASE/2.0) #Unit: m/s
	v_left_steps  = v_left  * STEPS_PER_METER
	v_right_steps = v_right * STEPS_PER_METER
	if (LEFT_INVERSED  == True):
		v_left_steps = -v_left_steps
	if (RIGHT_INVERSED == True):
		v_right_steps = -v_right_steps
	return(v_left_steps, v_right_steps) #Velocity out: steps/second

def cmd_callback(data):
	global twist_linear_x
	global twist_angular_z
	twist_linear_x  = data.linear.x 
	twist_angular_z = data.angular.z 
	v_left_steps, v_right_steps = inversed_kinematics(twist_linear_x, \
													  twist_angular_z)
	send_cmd(v_left_steps, v_right_steps)
#=========================================================================

#=========================================================================
rospy.init_node("bugbase")

port = rospy.get_param("~port", "/dev/ttyACM0")
baud = rospy.get_param("~baudrate", 115200)
ser  = serial.Serial(port, baud, timeout=0.5)
time.sleep(3) #Give Arduino some time to startup
base_status = stepper_params_loader()

twist_linear_x  = 0
twist_angular_z = 0
odom_pos_x  	= 0
odom_pos_y 		= 0
odom_angular_z  = 0
last_time 		= None

if __name__ == "__main__":
	if (base_status == True):
		rospy.loginfo("Stepper wheel base done initializing!")
		send_cmd(0, 0)
		rate = rospy.Rate(30)
		WHEEL_BASE 		= rospy.get_param("~wheel_base",      	0.25)
		#For stepper with 200 steps per revolution and wheel diameter of 85mm
		STEPS_PER_METER = rospy.get_param("~steps_per_meter", 	749.344) 
		LEFT_INVERSED 	= rospy.get_param("~left_inversed", 	False)
		RIGHT_INVERSED 	= rospy.get_param("~right_inversed", 	False)

		rospy.Subscriber("cmd_vel", Twist, cmd_callback)
		pub = rospy.Publisher("odom", Odometry, queue_size=1)
		while not rospy.is_shutdown():
			if (last_time == None):
				last_time = rospy.get_time()
				continue
			odom_msg = calculate_odom()
			pub.publish(odom_msg)
			rate.sleep()
	else:
		rospy.logerr("Cannot initialize stepper wheel base!")

	ser.close()
