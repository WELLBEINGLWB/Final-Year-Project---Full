#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point

rospy.init_node('gaze_publisher', anonymous="True")

pub = rospy.Publisher('/gaze_point', Point, queue_size=10)
rate = rospy.Rate(1) # 3 Hz

gaze_point = Point()
gaze_point.x = 0.52 #0.44
gaze_point.y = 0.78 #0.60
gaze_point.z = 0.26


while not rospy.is_shutdown():
	pub.publish(gaze_point)
	rate.sleep()
