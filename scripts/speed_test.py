#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

rospy.init_node('speed_test')
cmd_pub = rospy.Publisher("/cmd_vel",Twist, queue_size = 1)

# Set linear velocity to 0.2 m/s
vel = Twist()
vel.linear.x = -0.2 # m/s
vel.linear.y = 0.0
vel.linear.z = 0.0

vel.angular.x = 0.0 # rad /s
vel.angular.y = 0.0
vel.angular.z = 0.0 #0.78539816339

cmd_pub.publish(vel)

# Wait for duration seconds
duration = rospy.Duration(4.0,0)
rospy.sleep(rospy.Duration(1.0,0))

# Set linear velocity to 0.0 m/s
#vel.linear.x = 0.0
cmd_pub.publish(vel)
rospy.sleep(duration)
vel.angular.z = 0.0
vel.linear.x = 0.0
cmd_pub.publish(vel)
