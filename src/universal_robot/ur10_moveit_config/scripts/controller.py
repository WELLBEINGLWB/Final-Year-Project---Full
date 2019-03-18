#!/usr/bin/env python
# import tf
import rospy
from sensor_msgs.msg import PointCloud2
# import math
# import sys
# import copy
# import moveit_commander
# import moveit_python
# import moveit_msgs.msg
# import numpy as np
# import geometry_msgs.msg
# import shape_msgs.msg
# from math import pi
# from std_msgs.msg import String
# from std_msgs.msg import Float32MultiArray
# from visualization_msgs.msg import Marker
# from moveit_commander.conversions import pose_to_list

from segmentation.srv import*

class ExecutionManager(object):
  """ExecutionManager"""
  def __init__(self):
    super(ExecutionManager, self).__init__()

    ## Initialize rospy`_ node:
    rospy.init_node('controller_node', anonymous=True)

    self.pCloud = PointCloud2()


  def cloud_callback(self, pointcloud):
      self.pCloud = pointcloud
      # print(pointcloud)


  def segmentation_client():
      rospy.wait_for_service('add_two_ints')
      try:
          add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
          resp1 = add_two_ints(x, y)
          return resp1.sum
      except rospy.ServiceException, e:
          print "Service call failed: %s"%e


if __name__ == '__main__':

    controller = ExecutionManager()

    rospy.Subscriber("/camera/depth_registered/points", PointCloud2, controller.cloud_callback, queue_size=1)
    rospy.spin()

# def add_two_ints_client(x, y):
#     rospy.wait_for_service('add_two_ints')
#     try:
#         add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
#         resp1 = add_two_ints(x, y)
#         return resp1.sum
#     except rospy.ServiceException, e:
#         print "Service call failed: %s"%e
#
# def usage():
#     return "%s [x y]"%sys.argv[0]
#
# if __name__ == "__main__":
#     if len(sys.argv) == 3:
#         x = int(sys.argv[1])
#         y = int(sys.argv[2])
#     else:
#         print usage()
#         sys.exit(1)
#     print "Requesting %s+%s"%(x, y)
#     print "%s + %s = %s"%(x, y, add_two_ints_client(x, y))
