#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
import geometry_msgs.msg
from std_msgs.msg import Float32MultiArray
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

  # Waiting for a gaze point to be sent by the gaze_client
  def gaze_server_setup(self):
      gaze_server = rospy.Service('send_gaze', gazePoint, self.handle_gaze_point)
      print "Ready to send gaze point"
      rospy.spin()

  # Gaze service call back
  def handle_gaze_point(self, req):
      print "Got gaze point: [x:%s  y:%s  z:%s]"%(req.gaze_x, req.gaze_y, req.gaze_z)
      self.gaze_point.x = req.gaze_x
      self.gaze_point.y = req.gaze_y
      self.gaze_point.z = req.gaze_z
      # self.debug_printer()
      self.gaze_sent = 1
      # objects = Float32MultiArray()
      # objects.data = [0.09238377213478088, 0.07335389405488968, 0.1415911614894867, 0.43495261669158936, -0.11511331796646118, -0.0012646578252315521, 0.08798286318778992, 0.07133589684963226, 0.18319405615329742, 0.5219529271125793, 0.02710883319377899, 0.08386678248643875, 0.0326995849609375, 0.06856178492307663, 0.05430290102958679, 0.45429980754852295, 0.14136792719364166, -0.02174977958202362]
      # objects.data = [0.09238377213478088, 0.07335389405488968, 0.1415911614894867, 0.43495261669158936, -0.11511331796646118, -0.0012646578252315521]
      # objects.data = [0.09238377213478088, 0.07335389405488968, 0.0715911614894867, 0.33495261669158936, -0.11511331796646118, -0.0012646578252315521, 0.08798286318778992, 0.07133589684963226, 0.18319405615329742, 0.5219529271125793, 0.02710883319377899, 0.08386678248643875, 0.0326995849609375, 0.06856178492307663, 0.05430290102958679, 0.45429980754852295, 0.14136792719364166, -0.02174977958202362]
      objects = self.request_segmentation()
      path_found = self.gaze_optimiser_caller(objects)
      if path_found == True:

          return gazePointResponse(True)
      elif path_found == False:
          return gazePointResponse(False)

  def request_segmentation(self):
      print("Sending pointcloud to segmentation")
      rospy.wait_for_service('segmentation_service')

      try:
          #print("Sending pointcloud to segmentation")
          segmentation_service = rospy.ServiceProxy('segmentation_service', seg)
          resp = segmentation_service(self.pCloud)
          print("Sent")
          # print(resp.objects)
          return resp.objects
      except rospy.ServiceException, e:
          print "Service call failed: %s"%e

  def gaze_optimiser_caller(self, objs):
      print("Sent data to gaze optimiser")
      #self.gaze_point.x = 0.4
      #self.gaze_point.y = 0.9
      #self.gaze_point.z = 0.25
      rospy.wait_for_service('gaze_optimiser_service')
      try:
          gaze_optimiser_service = rospy.ServiceProxy('gaze_optimiser_service', gazeOptimiser)
          data = gaze_optimiser_service(objs,self.gaze_point)
          if(data!=None):
          # self.request_segmentation()
              print(data)
              self.path_planner_caller(data)
              return True
          else:
              return False
      except rospy.ServiceException, e:
          print "Service call failed: %s"%e

  def path_planner_caller(self, data):
      print("test")
      # print(data)
      planner_req = pathPlannerRequest()
      planner_req.sorted_objects = data.sorted_objects
      planner_req.target_id = data.target_id
      planner_req.grasp_point = data.grasp_point
      print(planner_req)
      rospy.wait_for_service('path_planner_service')
      try:
          path_planner_service = rospy.ServiceProxy('path_planner_service', pathPlanner)
          print("Sent data to path planner")
          planner_resp = path_planner_service(data.sorted_objects, data.target_id, data.grasp_point)
          print(planner_resp.path_found)
          if(planner_resp.path_found == True):
              option=raw_input("Execute? (y/n)")
              if option=="y":
                print("\n ")
                self.path_executer(True)
          return planner_resp.path_found
      except rospy.ServiceException, e:
          print "Service call failed: %s"%e

  def path_executer(self,req):

      rospy.wait_for_service('path_executer_service')
      try:
          path_executer_service = rospy.ServiceProxy('path_executer_service', executionOrder)
          print("Sent order to path executer")
          executer_resp = path_executer_service(req)
          print(executer_resp)
          return executer_resp
      except rospy.ServiceException, e:
          print "Service call failed: %s"%e

  # Function purely for debugging
  def debug_printer(self):
      print("Got here!")
      # print(self.gaze_point.y)



if __name__ == '__main__':

    controller = ExecutionManager()
    rospy.Subscriber("/camera/depth_registered/points", PointCloud2, controller.cloud_callback, queue_size=1)
    rospy.sleep(1.0)
    success = controller.gaze_server_setup()
    print(success)
    #objects = controller.request_segmentation()
    # objects = Float32MultiArray()
    # objects.data = [0.08766677975654602, 0.08212397247552872, 0.19581645727157593, 0.5342333912849426, -0.08212301135063171, 0.0106821209192276, 0.04192066192626953, 0.0972205102443695, 0.11764948815107346, 0.5432109832763672, 0.09034746885299683, -0.008821483701467514]
    #print(objects)
    # objects.data = [0.09238377213478088, 0.07335389405488968, 0.1415911614894867, 0.43495261669158936, -0.11511331796646118, -0.0012646578252315521, 0.08798286318778992, 0.07133589684963226, 0.18319405615329742, 0.5219529271125793, 0.02710883319377899, 0.08386678248643875, 0.0326995849609375, 0.06856178492307663, 0.05430290102958679, 0.45429980754852295, 0.14136792719364166, -0.02174977958202362]
    # optimiser_output = controller.gaze_optimiser_caller(objects)
    # print(optimiser_output)
    # if controller.gaze_sent == 1:
        # controller.request_segmentation()
    # rospy.Subscriber("/camera/depth_registered/points", PointCloud2, controller.cloud_callback, queue_size=1)
    print("blah")
    # p = controller.path_planner_caller(optimiser_output)
    # print(p)
    rospy.spin()
