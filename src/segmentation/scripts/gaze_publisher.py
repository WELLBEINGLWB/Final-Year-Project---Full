#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point

rospy.init_node('gaze_publisher', anonymous="True")

pub = rospy.Publisher('gaze', Point, queue_size=10)
rate = rospy.Rate(3) # 3 Hz

gaze_point = Point()
gaze_point.x = 1.0
gaze_point.y = 3.0
gaze_point.z = 0.5

while not rospy.is_shutdown():
	pub.publish(gaze_point)
	rate.sleep()
