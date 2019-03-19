#!/usr/bin/env python
# import tf
import rospy
from sensor_msgs.msg import PointCloud2
from segmentation import*
# import math
# import sys
# import copy
# import moveit_commander
# import moveit_python
# import moveit_msgs.msg
# import numpy as np
import geometry_msgs.msg
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
    self.gaze_point = geometry_msgs.msg.Point()
    self.gaze_sent = 0

  def cloud_callback(self, pointcloud):
      self.pCloud = pointcloud
      # print(pointcloud)

  # Waiting for a gaze point to be sent from the gaze_client
  def gaze_server_setup(self):
      gaze_server = rospy.Service('send_gaze', gazePoint, self.handle_gaze_point)
      print "Ready to send gaze point"
      # rospy.spin()

  # Gaze service call back
  def handle_gaze_point(self, req):
      print "Got gaze point: [x:%s  y:%s  z:%s]"%(req.gaze_x, req.gaze_y, req.gaze_z)
      self.gaze_point.x = req.gaze_x
      self.gaze_point.y = req.gaze_y
      self.gaze_point.z = req.gaze_z
      # self.debug_printer()
      self.gaze_sent = 1

      return gazePointResponse(True)


  def request_segmentation(self):
      print("Sending pointcloud to segmentation")
      rospy.wait_for_service('req_segmentation')
      try:
          send_gaze = rospy.ServiceProxy('req_segmentation', seg)
          resp = req_segmentation(self.pCloud)
          return resp.success
      except rospy.ServiceException, e:
          print "Service call failed: %s"%e

  # Function purely for debugging
  def debug_printer(self):
      print("Got here!")
      # print(self.gaze_point.y)



if __name__ == '__main__':

    controller = ExecutionManager()
    # controller.gaze_server_setup()

    # controller.request_segmentation()
    # if controller.gaze_sent == 1:
        # controller.request_segmentation()
    rospy.Subscriber("/camera/depth_registered/points", PointCloud2, controller.cloud_callback, queue_size=1)
    print("blah")

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
