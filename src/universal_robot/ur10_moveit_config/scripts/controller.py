#!/usr/bin/env python
# import tf
import rospy
from sensor_msgs.msg import PointCloud2
# from segmentation import*
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
      # print(self.pCloud )
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
      rospy.wait_for_service('segmentation_service')
      try:
          segmentation_service = rospy.ServiceProxy('segmentation_service', seg)
          resp = segmentation_service(self.pCloud)
          print("Sent")
          # print(resp.objects)
          return resp.objects
      except rospy.ServiceException, e:
          print "Service call failed: %s"%e

  # Function purely for debugging
  def debug_printer(self):
      print("Got here!")
      # print(self.gaze_point.y)



if __name__ == '__main__':

    controller = ExecutionManager()
    # controller.gaze_server_setup()
    rospy.Subscriber("/camera/depth_registered/points", PointCloud2, controller.cloud_callback, queue_size=1)
    rospy.sleep(1.0)
    objs = controller.request_segmentation()
    print(objs)
    # if controller.gaze_sent == 1:
        # controller.request_segmentation()
    # rospy.Subscriber("/camera/depth_registered/points", PointCloud2, controller.cloud_callback, queue_size=1)
    print("blah")

    rospy.spin()




# import numpy as np
# import pylab
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import axes3d
# import matplotlib.patches as patches
# from mpl_toolkits.mplot3d.art3d import Poly3DCollection
#
#
# objects = [0.08766677975654602, 0.08212397247552872, 0.19581645727157593, 0.5342333912849426, -0.08212301135063171, 0.0106821209192276, 0.04192066192626953, 0.0972205102443695, 0.11764948815107346, 0.5432109832763672, 0.09034746885299683, -0.008821483701467514]
# n_objects = int(len(objects)/6)
#
# #center = np.array( [[], []], np.float64 )
# center = np.empty([2, n_objects])
#
# i = 0
# while i < len(objects):
# #    center[0].append(objects[i+3])
# #    center[1].append(objects[i+4])
# #    np.append(center,[objects[i+3]])
#     iteration = int(i/6)
#     center[0,iteration] = objects[i+3]
#     center[1,iteration] = objects[i+4]
#
#     print(center)
#     i+=6
#
# def knn_search(x, D, K):
#  """ find K nearest neighbours of data among D """
#  ndata = D.shape[1]
#
#  K = K if K < ndata else ndata
#  # euclidean distances from the other points
#  sqd = np.sqrt(((D - x[:,:ndata])**2).sum(axis=0))
#  idx = np.argsort(sqd) # sorting
# # print(idx[:K])
#  # return the indexes of K nearest neighbours
#  return idx[:K]
#
# x = np.array([[.5],[-0.04]])
#
# # performing the search
# neig_idx = knn_search(x,center,1)
#
#
# # highlighting the neighbours
# pylab.plot(center[1,neig_idx],center[0,neig_idx],'o',
#   markerfacecolor='None',markersize=15,markeredgewidth=1)
# # plotting the data and the input point
# pylab.plot(center[1,:],center[0,:],'ob',x[1,0],x[0,0],'or')
# plt.axis([1.7,-0.2, 0.0, 0.6])
#plt.gca().invert_xaxis()
#show()

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
#X = np.array([1,4])
#Y = np.array([3,2])
#Z = np.array([4,3])
# x = [0,0,0.6,0.6]
# y = [0,1.7,1.7,0]
# z = [0.2,0.2,0.2,0.2]
# verts = [list(zip(y,x,z))]
# ax.add_collection3d(Poly3DCollection(verts))
#
# X, Y, Z = [0.5,0.4,0.4,0.2],[-0.2,0.2,0.2,0.5],[0.6,0.35,0.35,0.25]
# ax.plot3D(X,Y,Z)
# plt.xlim(-0.3, 1.0)
# plt.ylim(2.0, -0.2)

# pylab.show()


# [0.08766677975654602, 0.08212397247552872, 0.19581645727157593, 0.5342333912849426, -0.08212301135063171, 0.0106821209192276, 0.04192066192626953, 0.0972205102443695, 0.11764948815107346, 0.5432109832763672, 0.09034746885299683, -0.008821483701467514]
#
# Cluster 0:
# Dim x: 0.0876668
# Dim y: 0.082124
# Dim z: 0.195816
# Center x: 0.534233
# Center y: -0.082123
# Center z: 0.0106821
# PointCloud representing the Cluster: 94 data points.
# Cluster 1:
# Dim x: 0.0419207
# Dim y: 0.0972205
# Dim z: 0.117649
# Center x: 0.543211
# Center y: 0.0903475
# Center z: -0.00882148
