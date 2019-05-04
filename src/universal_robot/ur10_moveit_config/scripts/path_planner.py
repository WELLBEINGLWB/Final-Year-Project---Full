#!/usr/bin/env python
import tf
import rospy
import math
import sys
import copy
import moveit_commander
import moveit_python
import moveit_msgs.msg
import numpy as np
import geometry_msgs.msg
import trajectory_msgs.msg
import shape_msgs.msg
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.srv import GetPositionFKRequest
from moveit_msgs.srv import GetPositionFKResponse
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from moveit_commander.conversions import pose_to_list
from segmentation.srv import*

import pylab
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
import matplotlib.patches as patches
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib import colors

from heapq import *


def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

# Delete
class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

class MoveGroupPythonInterface(object):
  """MoveGroupPythonInterface"""
  def __init__(self):
    super(MoveGroupPythonInterface, self).__init__()


    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('path_planner', anonymous=True)

    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to the robot:
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface to one group of joints.
    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)
    # group.set_planner_id("RRTConnectkConfigDefault")
    ## Create a `DisplayTrajectory`_ publisher which is used later to publish trajectories for RViz to visualize:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

	## Getting Basic Information
    # Get name of the reference frame for the robot:
    planning_frame = group.get_planning_frame()

    # Print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()

    # Get a list of all the groups in the robot:
    group_names = robot.get_group_names()

    #print "============ Printing robot state"
    #print robot.get_current_state()



	# Misc variables
    self.objectAdder = moveit_python.PlanningSceneInterface("world")
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

    self.fk_srv = rospy.ServiceProxy('/compute_fk',GetPositionFK)
    self.fk_srv.wait_for_service()
    # self.objs = []

# delete
  def poseCallback(self, pose):
      self.q = (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w)
      self.translation = (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
      self.q = 5
          # self.poseCallback()
      print(self.q)
# delete
  def gaze_callback(self,point):
      self.gaze_point = geometry_msgs.msg.Point()
      self.gaze_point.x = point.x
      self.gaze_point.y = point.y
      self.gaze_point.z = point.z
      # print("Point x: ")
      # print(self.gaze_point.x)
      # print("Point y: ")
      # print(self.gaze_point.y)
      # print("Point z: ")
      # print(self.gaze_point.z)

  def go_to_initial_state(self):
    group = self.group

    # joint_goal = group.get_current_joint_values()
    group.set_planning_time(10)

    constraint = moveit_msgs.msg.Constraints()
    constraint.name = "fixed wrist orientation"
    #
    # constraint.joint_constraints.append(moveit_msgs.msg.JointConstraint(joint_name='elbow_joint', position=0, tolerance_above=math.pi,tolerance_below=math.pi, weight=1))
    # # Create an orientation constraint for the
    orientation_constraint = moveit_msgs.msg.OrientationConstraint()
    orientation_constraint.header.frame_id = "world"
    orientation_constraint.link_name = group.get_end_effector_link()
    orientation_constraint.orientation.x = 0.00111054358639
    orientation_constraint.orientation.y = 0.70699483645
    orientation_constraint.orientation.z = 0.00111089701837
    orientation_constraint.orientation.w = 0.707216963763
    orientation_constraint.absolute_x_axis_tolerance = 0.05
    orientation_constraint.absolute_y_axis_tolerance = 0.05
    orientation_constraint.absolute_z_axis_tolerance = 0.14
    orientation_constraint.weight = 1.0

    ## Append the constraint to the list of contraints
    constraint.orientation_constraints.append(orientation_constraint)

    primitive = shape_msgs.msg.SolidPrimitive()
    primitive.type = primitive.BOX
    ws_x = 0.7
    ws_y = 1.7
    ws_z = 0.3
    ## The workspace dimensions and center point will change according to the object to be grasped
    dim = [ws_x,ws_y,ws_z]
    primitive.dimensions = dim
    ws_pose = geometry_msgs.msg.Pose();
    ## table center point = (0.44,0.65,0.25)
    ws_pose.position.x = 0.44
    ws_pose.position.y = 0.65
    ws_pose.position.z = 0.25
    ws_pose.orientation.w = 0.0
    pos_constraint = moveit_msgs.msg.PositionConstraint()
    pos_constraint.header.frame_id = "world"
    pos_constraint.link_name = group.get_end_effector_link()
    pos_constraint.weight = 0.9
    primitives = [primitive]
    ws_poses = [ws_pose]
    pos_constraint.constraint_region.primitives = primitives
    pos_constraint.constraint_region.primitive_poses = ws_poses

    constraint.position_constraints.append(pos_constraint)


    initial_pose = geometry_msgs.msg.Pose()
    # pose_goal = geometry_msgs.msg.PoseStamped()
    # pose_goal.header.frame_id = group.get_end_effector_link()
    initial_pose.orientation.x = 0.00111054358639
    initial_pose.orientation.y = 0.70699483645
    initial_pose.orientation.z = 0.00111089701837
    initial_pose.orientation.w = 0.707216963763
    initial_pose.position.x = 0.2#0.297788083223
    initial_pose.position.y = 0.4#0.373332917389
    initial_pose.position.z = 0.20

    group.set_pose_target(initial_pose, group.get_end_effector_link())
    group.set_path_constraints(constraint)
    group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    group.stop()
    # It is always good to clear your targets after planning with poses.
    group.clear_pose_targets()

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    # current_joints = self.group.get_current_joint_values()





    current_pose = self.group.get_current_pose().pose
    return all_close(initial_pose, current_pose, 0.01)

  def path_planner_server(self):
      # Initialize server proxy for path_planner_service
      s = rospy.Service('path_planner_service', pathPlanner, self.planner)
      print "Ready to plan."
      rospy.spin()
      # print("Not get here")

  def path_executer_server(self):
      # Initialize server proxy for path_planner_service
      s = rospy.Service('path_executer_service', executionOrder, self.path_executer)
      print "Ready to execute path."
      #rospy.spin()
      # print("Not get here")

  def planner(self,request):
      # print(request)

      plot_request = 0 # 0 for no plots, 1 for plots

      objects = request.sorted_objects.data
      optimal_grasp_point = request.grasp_point
      grasp_point = geometry_msgs.msg.Point()
      target_id = request.target_id
      # Add object bounding boxes to planning scene
      self.add_box(objects, target_id)

      ratio = 1.7/0.7
      rows = 169
      cols = int(rows/ratio) + 1
      print("num cols: %s" %cols)
      data = [[0 for _ in range(rows)] for _ in range(cols)]
      Yresolution = 1.7/rows
      Xresolution = 0.7/cols
      print(optimal_grasp_point)
      target_x = optimal_grasp_point.x  # - world_objects.data[neig_idx*6]/2
      target_y = optimal_grasp_point.y
      target_index = [round(target_x/Xresolution)-1,round(target_y/Yresolution)-2]
      print(target_index)

      # Initial position of the robot/wrist
      wrist_co = geometry_msgs.msg.Point()
      wrist_co = self.group.get_current_pose().pose.position

      elbow_angle = 0

      # start_point = [0.15,0.38]
      start_point = [wrist_co.x,wrist_co.y]
      print("start_point:", start_point)
      start = (int(round(start_point[0]/Xresolution)-1), int(round(start_point[1]/Yresolution)-1))
      end = (int(target_index[0]), int(target_index[1]))
      print("start:", start)
      print("end:", end)




      for i in range(rows):
          for j in range(cols):
              #target = [Xresolution*(j+1),Yresolution*(i+1)]
              grasp_point.x = Xresolution*(j+1)
              grasp_point.y = Yresolution*(i+1)
              co_e, co_s, elbow_angle = self.ik_calculator(grasp_point,wrist_co)
              # co_e, co_s = angle_ik(grasp_point)
              collision_state = object_collision(co_e,grasp_point, objects, target_id)
              if(collision_state == True):
                  data[j][i] = 1

      #data[end[0]][end[1]]=2
      #data[start[0]][start[1]]=3

      test_data = np.array(data)
      test_data.shape
      # Plot collision state grid
      # fig2 = plt.figure(figsize=(10,10/ratio))
      # cmap = colors.ListedColormap(['white','red','green','#0060ff','#aaaaaa'])
      # plt.figure(figsize=(20,20/ratio))
      # plt.pcolor(test_data[::1],cmap=cmap,edgecolors='k', linewidths=1)
      # plt.gca().invert_xaxis()
      # plt.show(block=False)
      # rospy.sleep(10.0)
      # plt.close('all')


      #print astar(nmap, (0,0), (10,13))

      if data[end[0]][end[1]] != 1:
          print("A star working...")
          #path = astar(data, start, end)
          path = astar2(np.array(data), start, end)
          print("A star done")
          print(path)
          path_xy =  [[0]*3 for k in range(len(path))]
          # print(path)
          for j in range(len(path)):
                  r = path[j][0]
                  c = path[j][1]
      #                print(r,c)
                  data[r][c]= 4
                  path_xy[j][0]=Xresolution*r
                  path_xy[j][1]=Yresolution*c
                  grasp_point.x = path_xy[j][0]
                  grasp_point.y = path_xy[j][1]
                  co_e, co_s, elbow_angle = self.ik_calculator(grasp_point,wrist_co)
                  path_xy[j][2] = elbow_angle
          data[end[0]][end[1]]=2
          data[start[0]][start[1]]=3
          data = np.array(data)
          data.shape
          # Plot collision state grid
          # fig2 = plt.figure(figsize=(10,10/ratio))
          if(plot_request==1):
              cmap = colors.ListedColormap(['white','red','green','#0060ff','#777777'])
              plt.figure(figsize=(15,15/ratio))
              plt.pcolor(data[::1],cmap=cmap,edgecolors='k', linewidths=1)
              plt.gca().invert_xaxis()
              plt.show(block=False)
              rospy.sleep(5.0)
              plt.close('all')

          grasp_point.x = optimal_grasp_point.x
          grasp_point.y = optimal_grasp_point.y
          # grasp_point.x = 0.15
          # grasp_point.y = 0.38
          # co_e, co_s = angle_ik(grasp_point)
          co_e, co_s, elbow_angle = self.ik_calculator(grasp_point,wrist_co)
          print(grasp_point)

          print(path_xy)


          # Animation of the arm trajectory
          if(plot_request==1):
              fig = plt.figure(figsize=(10,10/(1.7/0.8)))
              ax = fig.add_subplot(111)

          center = np.empty([2, int(len(objects)/6)])
          for i in range(len(path_xy)):
              if(plot_request==1):
                  plt.cla()
                  ax.axis([-0.2,1.7, -0.2, 0.75])
                  plt.gca().invert_xaxis()
              #ax.add_patch(rect)

              j = 0
              while j < len(objects):
                  iteration = int(j/6)
                  center[0,iteration] = objects[j+3]
                  center[1,iteration] = objects[j+4]
                  if(plot_request==1):
                      rect = patches.Rectangle((center[1,iteration]-objects[j+1]/2,center[0,iteration]-objects[j]/2),objects[j+1],objects[j],linewidth=1,edgecolor='r',facecolor='none')
                      ax.add_patch(rect)

                  j+=6

              grasp_point.x = path_xy[i][0]
              grasp_point.y = path_xy[i][1]
              co_e, co_s, elbow_angle = self.ik_calculator(grasp_point,wrist_co)
              # co_e, co_s = angle_ik(grasp_point)
              if(plot_request==1):
                  ax.plot(center[1,:],center[0,:],'ob',grasp_point.y,grasp_point.x,'og',co_e.y,co_e.x,'ok',0.38,-0.15,'oy')
                  ax.plot(center[1,target_id],center[0,target_id],'o', markerfacecolor='None',markersize=15,markeredgewidth=1)
                  ax.plot([co_s.y,co_e.y,grasp_point.y],[co_s.x,co_e.x,grasp_point.x])
                  plt.draw()
                  plt.pause(0.01)

              # plt.cla()
              i+=1
          if(plot_request==1):
              rospy.sleep(5.0)
              plt.close('all')

          self.point_planner(path_xy, optimal_grasp_point)

          #self.go_to_initial_state()
          return True

      elif data[end[0]][end[1]] == 1:
          print("The target state is in collision")
          self.orientation_point_planner(optimal_grasp_point)
          return True

  def ik_calculator(self, grasp_point, wrist_co):

      e_co = geometry_msgs.msg.Point()
      sh_co = geometry_msgs.msg.Point()
      # wrist_co = geometry_msgs.msg.Point()
      # wrist_co = self.group.get_current_pose().pose.position
      e_co.z = grasp_point.z

      sh_co.x = -0.15
      sh_co.y = 0.42
      sh_co.z = 0.54
      u_length = 0.36 # upepr arm length
      f_length = 0.30 # forearm length

      #print("Grasp point: %s" %grasp_point)
      # wrist_0_x = .x - f_length
      # elbow_angle = math.degrees(math.atan((grasp_point.y - sh_co.y)/(grasp_point.x - sh_co.x)))
      # e_angle = math.atan((grasp_point.y - sh_co.y)/(grasp_point.x - sh_co.x - 0.29))
      e_angle = math.atan((grasp_point.y - wrist_co.y)/(grasp_point.x - wrist_co.x + f_length))
      elbow_angle = math.degrees(e_angle)
      # print("Elbow angle: %s" %elbow_angle)

      e_co.x = grasp_point.x - f_length*math.cos(e_angle)
      e_co.y = grasp_point.y - f_length*math.sin(e_angle)

      return (e_co, sh_co, e_angle)

  def point_planner(self, path_xy, optimal_grasp_point):
      group = self.group
      scene = self.scene
      group.set_planning_time(15)

      # pose_goal = geometry_msgs.msg.Pose()
      # pose_goal.orientation.x = 0.00111054358639
      # pose_goal.orientation.y = 0.70699483645
      # pose_goal.orientation.z = 0.00111089701837
      # pose_goal.orientation.w = 0.707216963763

      self.waypoints = []
      wpose = group.get_current_pose().pose
      print("Initial pose: %s" %wpose)
      waypoints_number = len(path_xy)
      print("Number of waypoints: %s" %waypoints_number)
      print("Object center points: %s" %optimal_grasp_point.z)
      z_increment = (wpose.position.z - optimal_grasp_point.z)/waypoints_number
      z_imcrement_threshold = (wpose.position.z - 0.20)/waypoints_number

      # distance from star point to target point
      dist_target = math.sqrt((optimal_grasp_point.y - wpose.position.y)**2 + (optimal_grasp_point.x - wpose.position.x)**2)
      print("Distance to target: %s" %dist_target)
      angle_z = math.asin((optimal_grasp_point.z-wpose.position.z)/dist_target)
      z_angle_increment = angle_z/waypoints_number
      angle_z = math.degrees(angle_z)
      print("Z angle: %s" %angle_z)

      ## Convert arm trajectory to waypoints:
      z_rot = 1.5707
      for i in range(len(path_xy)):
          wpose.position.x = path_xy[i][0]
          wpose.position.y = path_xy[i][1]
          ################################################################## CHECK height limit
          if(optimal_grasp_point.z >=0.20):
              wpose.position.z = wpose.position.z - z_increment
          else:
              wpose.position.z =  wpose.position.z - z_imcrement_threshold


          # euler = [1.571212121212, 1.5711348887669473,1.5711348887669473]

          # euler = [0, 1.5707, path_xy[i][2]]
          euler = [0, z_rot, path_xy[i][2]]
          # euler = [-1.5707, 0, -0.707]
          quat = tf.transformations.quaternion_from_euler(euler[0],euler[1],euler[2])
          # quat_test = [-0.51025,0.51025,-0.51025,0.51025]
          # quat_test = [-0.661025,0.251025,-0.251025,0.661025]
          # euler_test = tf.transformations.euler_from_quaternion(quat_test)
          # print(euler_test)
          # print(quat)
          # print(path_xy[i][2])
          wpose.orientation.x = quat[0]
          wpose.orientation.y = quat[1]
          wpose.orientation.z = quat[2]
          wpose.orientation.w = quat[3]

          # wpose.orientation.x = 0.489916391691
          # wpose.orientation.y = -0.510273396847
          # wpose.orientation.z = 0.531276672863
          # wpose.orientation.w = -0.466206055832

          self.waypoints.append(copy.deepcopy(wpose))
          z_rot = z_rot - z_angle_increment
      # waypoints_number = len(waypoints)
      # print("Number of waypoints: %s" %waypoints_number)
      (self.plan, fraction) = group.compute_cartesian_path(
                                         self.waypoints,   # waypoints to follow
                                         0.01,        # eef_step
                                         0.0, True)         # jump_threshold


      print("Fraction: %s" %fraction)
      successful_points = int(fraction * waypoints_number)
      print("Number of successful points: %s" %successful_points)
      if(fraction < 1):
          # failure_point = successful_points + 1
          print("Point at which it failed: %s" %(successful_points+1))
      attempt = 0
      while fraction < 1:
          print("Attempt number: %s" %attempt)
          successful_points = int(fraction * waypoints_number)
          failure_point_index = successful_points
          self.waypoints[failure_point_index].position.x = self.waypoints[failure_point_index].position.x - 0.01
          (self.plan, fraction) = group.compute_cartesian_path(
                                             self.waypoints,   # waypoints to follow
                                             0.01,        # eef_step
                                             0.0, True)
          attempt+=1
          print(fraction)

      # current_pose = self.group.get_current_pose().pose
      #success = group.execute(plan, wait=True)
      #print(success)
      # group.stop()
      # Clear  targets after planning with poses.
      # group.clear_pose_targets()
      #self.path_executer()
      # group.set_start_state_to_current_state()
      return self.plan, fraction

  def orientation_point_planner(self, optimal_grasp_point):
      group = self.group
      scene = self.scene
      group.set_planning_time(15)

      # pose_goal = geometry_msgs.msg.Pose()
      # pose_goal.orientation.x = 0.00111054358639
      # pose_goal.orientation.y = 0.70699483645
      # pose_goal.orientation.z = 0.00111089701837
      # pose_goal.orientation.w = 0.707216963763

      wrist_co = geometry_msgs.msg.Point()
      wrist_co = self.group.get_current_pose().pose.position
      co_e, co_s, elbow_angle = self.ik_calculator(optimal_grasp_point,wrist_co)

      self.waypoints = []
      wpose = group.get_current_pose().pose
      print("Initial pose: %s" %wpose)

      wpose.position.z = optimal_grasp_point.z + 0.1
      self.waypoints.append(copy.deepcopy(wpose))

      euler = [-1.5707, 0, -1.5707]
      quat = tf.transformations.quaternion_from_euler(euler[0],euler[1],euler[2])
      wpose.orientation.x = quat[0]
      wpose.orientation.y = quat[1]
      wpose.orientation.z = quat[2]
      wpose.orientation.w = quat[3]
      self.waypoints.append(copy.deepcopy(wpose))

      wpose.position.x = optimal_grasp_point.x + 0.03
      wpose.position.y = optimal_grasp_point.y + 0.015
      euler = [-1.5707, 0, -elbow_angle]
      quat = tf.transformations.quaternion_from_euler(euler[0],euler[1],euler[2])
      wpose.orientation.x = quat[0]
      wpose.orientation.y = quat[1]
      wpose.orientation.z = quat[2]
      wpose.orientation.w = quat[3]
      self.waypoints.append(copy.deepcopy(wpose))

      wpose.orientation.z -= 0.05
      self.waypoints.append(copy.deepcopy(wpose))


      (self.plan, fraction) = group.compute_cartesian_path(
                                         self.waypoints,   # waypoints to follow
                                         0.01,        # eef_step
                                         0.0, True)         # jump_threshold


      print("Fraction: %s" %fraction)

      # if(fraction < 1):
      #     # failure_point = successful_points + 1
      #     print("Point at which it failed: %s" %(successful_points+1))
      # attempt = 0
      # while fraction < 1:
      #     print("Attempt number: %s" %attempt)
      #     successful_points = int(fraction * waypoints_number)
      #     failure_point_index = successful_points
      #     self.waypoints[failure_point_index].position.x = self.waypoints[failure_point_index].position.x - 0.01
      #     (self.plan, fraction) = group.compute_cartesian_path(
      #                                        self.waypoints,   # waypoints to follow
      #                                        0.01,        # eef_step
      #                                        0.0, True)
      #     attempt+=1
      #     print(fraction)

      # current_pose = self.group.get_current_pose().pose
      #success = group.execute(plan, wait=True)
      #print(success)
      # group.stop()
      # Clear  targets after planning with poses.
      # group.clear_pose_targets()
      #self.path_executer()
      # group.set_start_state_to_current_state()
      return self.plan, fraction

  def path_executer(self,request):
      if request.execute_order == True:
          success = self.group.execute(self.plan, wait = True)
          print(success)
          # self.group.stop()
          # self.group.clear_pose_targets()

          rospy.sleep(5)
          reverse_waypoints = self.waypoints[::-1]

          (self.reverse_plan, fraction) = self.group.compute_cartesian_path(
                                             reverse_waypoints,   # waypoints to follow
                                             0.01,        # eef_step
                                             0.0, True)
          rospy.sleep(5)
          self.group.execute(self.reverse_plan, wait = True)

          if success == True:
              return True
          elif success == False:
              return False
      elif request.execute_order == False:
          print("Why the hell did I bother planning this?!")
          # self.group.stop()
          self.group.clear_pose_targets()

  def get_fk(self,fk_req):

      #fk_req = GetPositionFKRequest()
      # print(fk_req)
      # print(ee_position)
      #fk_req.header.frame_id = 'world'
      #fk_req.fk_link_names = ['wrist_3_joint']
      #fk_req.robot_state.joint_state.position = ee_position
      #fk_req.robot_state.joint_state.header.frame_id = 'world'
      #fk_req.header.stamp = rospy.Time()
      #print(fk_req)

      try:
            resp = self.fk_srv(fk_req)
            return resp
      except rospy.ServiceException as e:
            rospy.logerr("Service exception: " + str(e))
            resp = GetPositionFKResponse()
            resp.error_code = 99999  # Failure
      return resp

  def go_to_pose_goal(self):
    group = self.group
    scene = self.scene
    group.set_planning_time(15)

     # Create a contraints list and name it
    constraint = moveit_msgs.msg.Constraints()
    constraint.name = "fixed wrist orientation"
    #
    # constraint.joint_constraints.append(moveit_msgs.msg.JointConstraint(joint_name='elbow_joint', position=0, tolerance_above=math.pi,tolerance_below=math.pi, weight=1))
    ## Orientation constraint for the end effector to keep the orientation constant
    orientation_constraint = moveit_msgs.msg.OrientationConstraint()
    orientation_constraint.header.frame_id = "world"
    orientation_constraint.link_name = group.get_end_effector_link()
    orientation_constraint.orientation.x = 0.00111054358639
    orientation_constraint.orientation.y = 0.70699483645
    orientation_constraint.orientation.z = 0.00111089701837
    orientation_constraint.orientation.w = 0.707216963763
    orientation_constraint.absolute_x_axis_tolerance = 0.05
    orientation_constraint.absolute_y_axis_tolerance = 0.05
    orientation_constraint.absolute_z_axis_tolerance = 0.14
    orientation_constraint.weight = 1.0

    # # Append the constraint to the list of contraints
    constraint.orientation_constraints.append(orientation_constraint)

    # j_val = group.get_current_joint_values()
    # joint_constraint = moveit_msgs.msg.JointConstraint()
    # joint_constraint.joint_name = "elbow_joint"
    # joint_constraint.position = j_val[2]
    # # print(j_val[2])
    # joint_constraint.tolerance_above = 0.3
    # joint_constraint.tolerance_below = 0.3
    # joint_constraint.weight = 0.9
    # constraint.joint_constraints.append(joint_constraint)


    # self.target_obj = [0.055,0.055,0.15,0.3,0.7,0.26,"ID"] #dummy0

    self.target_obj = [0.055,0.055,0.15,0.4,0.9,0.25,"ID"] #dummy1
    ws_margin = 0.05
    # Position constraint that define the workspace of the end effector link
    primitive = shape_msgs.msg.SolidPrimitive()
    primitive.type = primitive.BOX
    ## The workspace dimensions and center point will change according to the object to be grasped
    # ws_x = 0.7 #self.table_x #table original dimensions
    # ws_y = 1.7 #self.table_y
    # ws_z = 0.2 #self.table_z
    ws_x = (self.target_obj[3] + (self.target_obj[0]/2) + ws_margin) - 0.15 # (0.15 is the x coordinate of the bottom margin of the table)
    # ws_y = (self.target_obj[4] + (self.target_obj[1]/2) + ws_margin) + 0.2
    ws_y = self.target_obj[4]  + 0.2
    ws_z = (self.target_obj[5] + (self.target_obj[2]/2)) - 0.185
    dim = [ws_x,ws_y,ws_z]
    primitive.dimensions = dim
    ws_pose = geometry_msgs.msg.Pose();
    ## Table center point = (0.44,0.65,0.25)
    # ws_pose.position.x = 0.44
    # ws_pose.position.y = 0.65
    # ws_pose.position.z = 0.25
    ws_pose.position.x = 0.15 + ws_x/2
    ws_pose.position.y = -0.2 + ws_y/2 + 0.5
    ws_pose.position.z = 0.185 + ws_z/2
    # targe_obj is set as such: [xDimension, yDimension, zDimension, xCenter, yCenter, zCenter, objectID]
    # self.objectAdder.addBox("ws", ws_x, ws_y, ws_z, ws_pose.position.x, ws_pose.position.y, ws_pose.position.z)
    # self.objectAdder.setColor("ws", 0.0, 0.0, 0.0, a=0.7)
    # self.objectAdder.sendColors()
    # rospy.sleep(4.0)
    # self.objectAdder.removeCollisionObject("ws")
    # scene.remove_world_object("ws")
    # table_pose.pose.position.x = 0.30
    # table_pose.pose.position.y = 0.70
    # table_pose.pose.position.z = 0.26
    # scene.add_box("dummy0", table_pose, size=( 0.055, 0.055, 0.15))

    ws_pose.orientation.w = 0.0
    pos_constraint = moveit_msgs.msg.PositionConstraint()
    pos_constraint.header.frame_id = "world"
    pos_constraint.link_name = group.get_end_effector_link()
    pos_constraint.weight = 0.9
    primitives = [primitive]
    ws_poses = [ws_pose]
    pos_constraint.constraint_region.primitives = primitives
    pos_constraint.constraint_region.primitive_poses = ws_poses

    # pos_constraint.constraint_region.primitives.type=SPHERE
    # pos_constraint.constraint_region.primitive_poses.position.x=0.0
    # pos_constraint.constraint_region.primitive_poses.position.y=0.0
    # pos_constraint.constraint_region.primitive_poses.position.z=0.0
    constraint.position_constraints.append(pos_constraint)
    # Set the path constraints on the end effector
    group.set_path_constraints(constraint)

    pos_x = float(input("Enter x coordinate: "))
    pos_y = float(input("Enter y coordinate: "))
    pos_z = float(input("Enter z coordinate: "))
    # Tagret pose to where the hand will be sent. The
    offset_x = 0.015
    offset_y = 0.035
    # pos_x = self.target_obj[3] - self.target_obj[0]/2 - offset_x
    # pos_y = self.target_obj[4] - self.target_obj[1]/2 - offset_y
    # pos_z = self.target_obj[5]
    ## Planning to a Pose Goal of the end-effector:
    pose_goal = geometry_msgs.msg.Pose()
    # pose_goal = geometry_msgs.msg.PoseStamped()
    # pose_goal.header.frame_id = group.get_end_effector_link()
    pose_goal.orientation.x = 0.00111054358639
    pose_goal.orientation.y = 0.70699483645
    pose_goal.orientation.z = 0.00111089701837
    pose_goal.orientation.w = 0.707216963763
    # pose_goal.orientation.x = 0.0
    # pose_goal.orientation.y = 0.7
    # pose_goal.orientation.z = 0.0
    # pose_goal.orientation.w = 0.7
    # pose_goal.orientation.x = -0.267023522538
    # pose_goal.orientation.y = 0.653702052828
    # pose_goal.orientation.z = 0.269304381591
    # pose_goal.orientation.w = 0.654864271888
    # x: -0.267023522538
    #     y: 0.653702052828
    #     z: 0.269304381591
    #     w: 0.654864271888

    # orientation_list = [pose_goal.orientation.x, pose_goal.orientation.y,pose_goal.orientation.z, pose_goal.orientation.w]
    # eul = tf.transformations.euler_from_quaternion(orientation_list)
    # print(eul)


    # (1.4711348887669473, 1.5676390675265013, 1.4711353885958782)
    # (0.7105587279368307, 1.5671760841731328, 1.4885120467903026)

    # euler = [0.8271760841731328, 1.5671760841731328, 1.4711348887669473]
    # quat = tf.transformations.quaternion_from_euler(euler[0],euler[1],euler[2])
    # print(quat)
    # pose_goal.orientation.x = quat[0]
    # pose_goal.orientation.y = quat[1]
    # pose_goal.orientation.z = quat[2]
    # pose_goal.orientation.w = quat[3]
    # x: -0.228603116552
    #     y: 0.668342268725
    #     z: 0.230398602104
    #     w: 0.669309876729
    # x: -0.264022915886
    #     y: 0.655028719657
    #     z: 0.266384321892
    #     w: 0.655948678909
    pose_goal.position.x = pos_x
    pose_goal.position.y = pos_y
    pose_goal.position.z = pos_z

    # waypoints = []
    # x = 0.40
    # y = 0.70
    # z = 0.25
    # wpose = group.get_current_pose().pose

    # wpose.position.y = 0.45
    # wpose.position.x = 0.4
    # waypoints.append(copy.deepcopy(wpose))
    #
    # wpose.position.y = 0.6
    # wpose.position.x = 0.2
    # waypoints.append(copy.deepcopy(wpose))
    #
    # wpose.position.y = 0.4
    # wpose.position.x = 0.4
    # wpose.position.z = 0.25
    # waypoints.append(copy.deepcopy(wpose))
    #
    # wpose.position.y = 0.8
    # wpose.position.x = 0.4
    # wpose.position.z = 0.25
    # waypoints.append(copy.deepcopy(wpose))

    # # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # # which is why we will specify 0.01 as the eef_step in Cartesian
    # # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    # (plan, fraction) = group.compute_cartesian_path(
    #                                    waypoints,   # waypoints to follow
    #                                    0.01,        # eef_step
    #                                    0.0, True)         # jump_threshold


    # group.set_max_acceleration_scaling_factor(0.1)
    # group.get_planner_id()
    # [minX, minY, minZ, maxX, maxY, maxZ]
    # ws = [0.0,0.0,0.0,0.0,0.1,0.1]
    # group.set_workspace(ws)
    group.set_pose_target(pose_goal, group.get_end_effector_link())
    plan_manipulator = group.plan()

    # print(plan_manipulator)
    #s = rospy.Service('compute_fk', AddTwoInts, handle_add_two_ints)

    # fk_req = GetPositionFK()
    # fk_req.header.frame_id = 'world'
    # fk_req.fk_link_names = ['wrist_3_joint']
    #print(rospy.Time())
    n_points = len(plan_manipulator.joint_trajectory.points)
    for n in range(n_points):
       current_pose = group.get_current_pose()
       print("Point %s" %n)
       print("Angle: %s" %plan_manipulator.joint_trajectory.points[n].positions[5])
       print("---------")
       ee_position = plan_manipulator.joint_trajectory.points[n].positions
       fk_req = GetPositionFKRequest()
       # print(fk_req)
       # print(ee_position)
       fk_req.header.frame_id = 'world'
       fk_req.fk_link_names = ['fake_hand_link']
       fk_req.robot_state.joint_state.position = ee_position
       #fk_req.robot_state
       fk_req.robot_state.joint_state.header.frame_id = 'world'
       fk_req.robot_state.joint_state.name = ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
       fk_req.robot_state.joint_state.header.stamp = rospy.Time.now()
       fk_req.header.stamp = rospy.Time.now()
       #print(fk_req)
       fk_resp = self.get_fk(fk_req)
       print(fk_resp.pose_stamped[0].pose.position)
    # print(plan_manipulator.joint_trajectory.points[0])

    # group.execute(plan, wait=True)
    group.plan(pose_goal)
    option=raw_input("Execute? (y/n)")
    if option=="y":
      print("\n ")
      group.go(pose_goal, wait=True)
    # Call the planner to compute the plan and execute it.
    # group.go(wait=True)
    # plan = group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    group.stop()
    # It is always good to clear your targets after planning with poses.
    group.clear_pose_targets()

    # objs = scene.get_objects()
    # print(objs)
    # print("---------------")
    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.group.get_current_pose().pose
    # group.set_start_state_to_current_state()
    return all_close(pose_goal, current_pose, 0.01)

  def plan_pose_goal(self):
    group = self.group
    group.set_planning_time(10)

     # Create a contraints list and name it
    # constraint = moveit_msgs.msg.Constraints()
    # constraint.name = "fixed wrist orientation"
    # #
    # # Create an orientation constraint for the
    # orientation_constraint = moveit_msgs.msg.OrientationConstraint()
    # orientation_constraint.header.frame_id = "world"
    # orientation_constraint.link_name = group.get_end_effector_link()
    # orientation_constraint.orientation.x = 0.00111054358639
    # orientation_constraint.orientation.y = 0.70699483645
    # orientation_constraint.orientation.z = 0.00111089701837
    # orientation_constraint.orientation.w = 0.707216963763
    # orientation_constraint.absolute_x_axis_tolerance = 0.05
    # orientation_constraint.absolute_y_axis_tolerance = 0.05
    # orientation_constraint.absolute_z_axis_tolerance = 0.14
    # orientation_constraint.weight = 1.0

    ## Append the constraint to the list of contraints
    # constraint.orientation_constraints.append(orientation_constraint)

    # joint_constraint = moveit_msgs.msg.JointConstraint()

    # group.setMaxVelocityScalingFactor(0.1);


    # Set the path constraints on the end effector
    # group.set_path_constraints(constraint)


    pos_x = float(input("Enter x coordinate: "))
    pos_y = float(input("Enter y coordinate: "))
    pos_z = float(input("Enter z coordinate: "))

    pose_goal = geometry_msgs.msg.Pose()

    # pose_goal.orientation.x = 0.00111054358639
    # pose_goal.orientation.y = 0.70699483645
    # pose_goal.orientation.z = 0.00111089701837
    # pose_goal.orientation.w = 0.707216963763
    pose_goal.position.x = pos_x
    pose_goal.position.y = pos_y
    pose_goal.position.z = pos_z

    group.plan(pose_goal)

    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    ## Displaying a Trajectory
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
    ## group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again):
    ##
    ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    ## We populate the trajectory_start with our current robot state to copy over
    ## any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(pose_goal)
    # Publish
    # display_trajectory_publisher.publish(display_trajectory);

    # group.set_max_acceleration_scaling_factor(0.1)
    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow the plan that has already been computed:
    option=raw_input("Execute? (y/n)")
    if option=="y":
      print("\n ")
      group.go(pose_goal, wait=True)

    # group.clearPathConstraints();


    # group.set_pose_target(pose_goal, group.get_end_effector_link())

    ## Now, we call the planner to compute the plan and execute it.
    # plan = group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    # group.stop()
    # Clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    # group.clear_pose_targets()

    # current_pose = self.group.get_current_pose().pose
    # return all_close(pose_goal, current_pose, 0.01)
  ##DELETE
  def plan_cartesian_path(self, scale=1):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good reason not to.
    group = self.group

    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a Cartesian path directly by specifying a list of waypoints for the end-effector to go through:

    waypoints = []

    wpose = group.get_current_pose().pose
    wpose.position.z -= scale * 0.1  # First move up (z)
    wpose.position.y += scale * 0.2  # and sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= scale * 0.1  # Third move sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction
  ##DELETE
  def display_trajectory(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    ## Displaying a Trajectory
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
    ## group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again):
    ##
    ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    ## We populate the trajectory_start with our current robot state to copy over
    ## any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(pose_goal)
    # Publish
    display_trajectory_publisher.publish(display_trajectory);

  def execute_plan(self, plan):

    group = self.group
    group.set_planning_time(10)
    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    group.execute(pose_goal, wait=True)

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail

  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):

    box_name = self.box_name
    scene = self.scene


    ## Ensuring Collision Updates Are Receieved
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## If the Python node dies before publishing a collision object update message, the message
    ## could get lost and the box will not appear. To ensure that the updates are
    ## made, we wait until we see the changes reflected in the
    ## ``get_known_object_names()`` and ``get_known_object_names()`` lists.
    ## call this function after adding,
    ## removing, attaching or detaching an object in the planning scene. We then wait
    ## until the updates have been made or ``timeout`` seconds have passed
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False

  def add_table(self):
    # box_name = self.box_name
    scene = self.scene

    # group = self.group
    # robot = self.robot
    # eef_link = self.eef_link
    # group_names = self.group_names
    #
    # camera_pose = geometry_msgs.msg.PoseStamped()
    # camera_pose.header.frame_id = group.get_end_effector_link()
    # camera_pose.pose.orientation.w = 1.0
    # camera_pose.pose.position.x = 0.0
    # camera_pose.pose.position.y = 0.0
    # camera_pose.pose.position.z = 0.0
    # scene.add_mesh("hand", camera_pose, "/home/faisallab008/catkin_ws/src/universal_robot/ur_description/meshes/hand.stl", size=(0.001,-0.001,0.001))
    #
    # grasping_group = 'manipulator'
    # touch_links = robot.get_link_names(group=grasping_group)
    # scene.attach_box(eef_link, "hand", touch_links=touch_links)

    print("Adding table")
    table_pose = geometry_msgs.msg.PoseStamped()
    table_pose.header.frame_id = "world"
    table_pose.pose.orientation.w = 0.0
    # table_pose.pose.position.x = 0.44
    # table_pose.pose.position.y = 0.65
    # table_pose.pose.position.z = -0.24
    # scene.add_box("table", table_pose, size=( 0.7, 1.7, 0.85))
    self.table_x_dim = 0.7
    self.table_y_dim = 1.7
    # self.table_z_dim = 0.85
    self.table_z_dim = 0.08
    self.table_x_center = 0.44
    self.table_y_center = 0.65
    # self.table_z_center = -0.24
    self.table_z_center = 0.12 # 0.145
    self.table_height = self.table_z_center +  self.table_z_dim/2
    print(self.table_height)
    # self.objectAdder.addBox("table1", self.table_x_dim, self.table_y_dim, 0.85, self.table_x_center, self.table_y_center, -0.24)
    self.objectAdder.addBox("table", self.table_x_dim, self.table_y_dim, self.table_z_dim, self.table_x_center, self.table_y_center, self.table_z_center)
    # self.objectAdder.setColor("table", 0.1, 1.0, 0.2, a=0.9)
    self.objectAdder.setColor("table", 0.57, 0.73, 1.0, a=0.9)

    leg_side_length = 0.05
    leg_height = 0.77
    leg_center = self.table_z_center - self.table_z_dim/2 - leg_height/2
    self.objectAdder.addBox("leg_1", leg_side_length , leg_side_length , leg_height , self.table_x_center + self.table_x_dim/2 - leg_side_length/2 , self.table_y_center - self.table_y_dim/2 + leg_side_length/2 , leg_center)
    self.objectAdder.addBox("leg_2", leg_side_length , leg_side_length , leg_height , self.table_x_center + self.table_x_dim/2 - leg_side_length/2 , self.table_y_center + self.table_y_dim/2 - leg_side_length/2 , leg_center)
    self.objectAdder.addBox("leg_3", leg_side_length , leg_side_length , leg_height , self.table_x_center - self.table_x_dim/2 + leg_side_length/2 , self.table_y_center - self.table_y_dim/2 + leg_side_length/2 , leg_center)
    self.objectAdder.addBox("leg_4", leg_side_length , leg_side_length , leg_height , self.table_x_center - self.table_x_dim/2 + leg_side_length/2 , self.table_y_center + self.table_y_dim/2 - leg_side_length/2 , leg_center)
    self.objectAdder.setColor("leg_1", 0.78, 0.44, 0.2, a=1.0)
    self.objectAdder.setColor("leg_2", 0.78, 0.44, 0.2, a=1.0)
    self.objectAdder.setColor("leg_3", 0.78, 0.44, 0.2, a=1.0)
    self.objectAdder.setColor("leg_4", 0.78, 0.44, 0.2, a=1.0)


    # table_pose.pose.position.x = 0.30
    # table_pose.pose.position.y = 0.70
    # table_pose.pose.position.z = 0.26
    # scene.add_box("dummy0", table_pose, size=( 0.055, 0.055, 0.15))

    # table_pose.pose.position.x = 0.40
    # table_pose.pose.position.y = 0.90
    # table_pose.pose.position.z = 0.25
    # scene.add_box("dummy1", table_pose, size=( 0.055, 0.055, 0.15))

    # table_pose.pose.position.x = 0.40
    # table_pose.pose.position.y = 0.45
    # table_pose.pose.position.z = 0.20
    # scene.add_box("dummy2", table_pose, size=( 0.055, 0.055, 0.05))



    # table_pose.header.frame_id = "world"
    # table_pose.pose.position.x = 0.44
    # table_pose.pose.position.y = 0.65
    # table_pose.pose.position.z = 0.25
    # scene.add_box("ws", table_pose, size=( 0.7, 1.7, 0.2))

    # table_pose.pose.position.x = -0.45
    # table_pose.pose.position.y = -0.55
    # table_pose.pose.position.z = 0
    # scene.add_box("2", table_pose, size=( 0.25, 0.25, 1.75))
    # print("Table height: ")
    # print(-0.24 + (0.85/2))
    #
    # wall_pose = geometry_msgs.msg.PoseStamped()
    # wall_pose.header.frame_id = "base"
    # wall_pose.pose.orientation.w = 0.0
    # wall_pose.pose.position.x = -0.45
    # wall_pose.pose.position.y = 0.40
    # wall_pose.pose.position.z = 0.20
    # scene.add_box("wall", wall_pose, size=( 2.0, 0.1, 1.5))

    self.objectAdder.addBox("side_wall", 2.0, 0.1, 1.5, 0.45, -0.40, 0.20)
    self.objectAdder.setColor("side_wall", 0.1, 1.0, 0.2, a=0.9)
    self.objectAdder.sendColors()

    # top_pose = geometry_msgs.msg.PoseStamped()
    # top_pose.header.frame_id = "base"
    # top_pose.pose.orientation.w = 0.0
    # top_pose.pose.position.x = -0.7
    # top_pose.pose.position.y = -1.0
    # top_pose.pose.position.z = 0.75
    # scene.add_box("ceilling", top_pose, size=( 1.0, 2.0, 0.05))


    # top_pose.pose.position.x = 0.3
    # top_pose.pose.position.y = -0.7
    # top_pose.pose.position.z = 0.2
    # scene.add_box("person", top_pose, size=( 0.4, 0.4, 1.75))

    # arrow = Marker

    # scene.setColor("table", 0.9, 0.2, 0.9, 1.0)
    # Add a mesh
    # self.leg_mesh = "/home/faisallab008/catkin_ws/src/universal_robot/ur_description/meshes/cube.stl" # or better, use find_package()
    # scene.add_mesh("mesh_test", box_pose, self.leg_mesh)

    # Copy local variables back to class variables. In practice, you should use the class
    # variables directly unless you have a good reason not to.
    # print(scene.get_known_object_names())
    return self.wait_for_state_update(box_is_known=True, timeout=4)

  def add_box(self, objects, target_id):
    # Add the bounding boxes of the objects to the planning scene
    # box_name = self.box_name
    scene = self.scene

    ## Removing all the objects in the scene before adding the new ones
    # print("Number of objects found in the scene:" , len(scene.get_known_object_names()))
    no = len(scene.get_known_object_names())
    f = 0
    while f < no:
        box_nam = str(f)
        scene.remove_world_object(box_nam)
        # print("Removed object number ")
        # print(f)
        f+=1

    # print(msg.data)
    ## Adding Objects to the Planning Scene
    # objects = msg.data
    world_objects = objects
    # print(world_objects)

    # Number of objects in the array (each has 6 dimensions)
    num_objects = len(world_objects)/6
    # print("Number of objects: ")
    # print(num_objects)
    # obj_name={}
    # for n in range(1,10):
    #     obj_name["object{0}".format(n)]="Hello"

    # Extra margin to add to the bounding boxes and to find gaze point within one object(in meters)
    margin = 0.0
    i = 0
    while i < len(world_objects):
        object_id = str(i/6)
        if(object_id!=str(target_id)):
            self.objectAdder.addBox(object_id, world_objects[i] + margin, world_objects[i+1] + margin, world_objects[i+2], world_objects[i+3], world_objects[i+4], world_objects[i+5])
            if(object_id == str(target_id)):
                self.objectAdder.setColor(object_id, 1.0, 0.2, 0.2, a=1.0)
                ## Target object will have a different colour
            else:
                self.objectAdder.setColor(object_id, 0.1, 0.4, 1.0, a=1.0)
            # All obstacles have the same colour
        # All the colours are set at the same time
        self.objectAdder.sendColors()
        i+=6



    # transformer = tf.TransformListener()

    # i = 0
    # while i < len(objects):
    #
    #     # Transform the center point of the object from cameraLink frame to world frame
    #     transformer.waitForTransform("camera_link", "world", rospy.Time(0),rospy.Duration(4.0))
    #     pointstamp = geometry_msgs.msg.PointStamped()
    #     pointstamp.header.frame_id = "camera_link"
    #     pointstamp.header.stamp = rospy.Time()
    #     pointstamp.point.x = objects[i+3]
    #     pointstamp.point.y = objects[i+4]
    #     pointstamp.point.z = objects[i+5]
    #
    #     # print("Before: ")
    #     # print(pointstamp.point.x)
    #     # print(pointstamp.point.y)
    #     # print(pointstamp.point.z)
    #     # transformer.waitForTransform("camera_link", "world", rospy.Time(0),rospy.Duration(4.0))
    #
    #     # Converting the center point if the object to /world frame
    #     p_tr = transformer.transformPoint("world", pointstamp)
    #
    #     # world_objects[i+3] = p_tr.point.x
    #     # world_objects[i+4] = p_tr.point.y
    #     # world_objects[i+5] = p_tr.point.z
    #     # print("After: ")
    #     # print(p_tr.point.x)
    #     # print(p_tr.point.y)
    #     # print(p_tr.point.z)
    #
    #     height_to_table = 0.185 - (p_tr.point.z - (objects[i+2]/2))
    #     # print("Object height: ")
    #     # print(objects[i+2])
    #     # print("Height to table: ")
    #     # print(height_to_table)
    #     # print("------- ")
    #
    #     box_name = self.box_name
    #     box_pose = geometry_msgs.msg.PoseStamped()
    #     box_pose.header.frame_id = "world"
    #     box_pose.pose.position.x = p_tr.point.x
    #     box_pose.pose.position.y = p_tr.point.y
    #     # box_pose.pose.position.z = p_tr.point.z  # - height_to_table
    #
    #     box_pose.pose.position.z = 0.185 + (objects[i+2]/2)
    #     box_pose.pose.position.z = box_pose.pose.position.z + (p_tr.point.z - box_pose.pose.position.z)/2
    #     # world_objects[i+2] = objects[i+2] + (p_tr.point.z - box_pose.pose.position.z)
    #
    #     world_objects.insert(i, objects[i])
    #     world_objects.insert(i+1, objects[i+1])
    #     world_objects.insert(i+2, objects[i+2] + (p_tr.point.z - box_pose.pose.position.z))
    #     world_objects.insert(i+3, p_tr.point.x)
    #     world_objects.insert(i+4, p_tr.point.y)
    #     world_objects.insert(i+5, box_pose.pose.position.z)
    #     # print(world_objects)
    #     object_id = str(i/6)
    #     # self.objectAdder.addBox(object_id, world_objects[i] + margin, world_objects[i+1] + margin, world_objects[i+2] , world_objects[i+3], world_objects[i+4], world_objects[i+5])
    #     # self.objectAdder.addBox(object_id, world_objects[i] + margin, world_objects[i+1] + margin, world_objects[i+2], world_objects[i+3], world_objects[i+4], world_objects[i+5])
    #     # scene.add_box(object_id, box_pose, size=(objects[i] + margin, objects[i+1] + margin, objects[i+2] + (p_tr.point.z - box_pose.pose.position.z)))
    #     # self.objectAdder.setColor(object_id, 0.1, 0.4, 1.0, a=0.9)
    #     # self.objectAdder.sendColors()
    #
    #     ## Gaze integration
    #     margin = 0.0 #0.042
    #     gaze_margin = 0.05
    #     # self.gaze_point = geometry_msgs.msg.Point()
    #     # self.gaze_point.x = 0.4
    #     # self.gaze_point.y = 0.6
    #     # self.gaze_point.z = 0.2
    #     print("x,y,z = ")
    #     print(world_objects[i+3])
    #     print(world_objects[i+4])
    #     print(world_objects[i+5])
    #     print("------")
    #
    #     #Adding the objects to the world
    #     ## If the gaze point is within the object, that is the target object
    #     if (((world_objects[i+3] - (world_objects[i]/2) - gaze_margin) <= self.gaze_point.x <= (world_objects[i+3] + (world_objects[i]/2) + gaze_margin))
    #           and ((world_objects[i+4] - (world_objects[i+1]/2) - gaze_margin) <= self.gaze_point.y <= (world_objects[i+4] + (world_objects[i+1]/2) + gaze_margin))
    #           and ((world_objects[i+5] - (world_objects[i+2]/2) - gaze_margin) <= self.gaze_point.z <= (world_objects[i+5] + (world_objects[i+2]/2) + gaze_margin))):
    #
    #             # self.target_obj_name = object_id
    #             self.target_obj = []
    #             self.target_obj.insert(0, world_objects[i])
    #             self.target_obj.insert(1, world_objects[i+1])
    #             self.target_obj.insert(2, world_objects[i+2])
    #             self.target_obj.insert(3, world_objects[i+3])
    #             self.target_obj.insert(4, world_objects[i+4])
    #             self.target_obj.insert(5, world_objects[i+5])
    #             self.target_obj.insert(6, object_id)
    #             print(self.target_obj)
    #             # targe_obj is set as such: [xDimension, yDimension, zDimension, xCenter, yCenter, zCenter, objectID]
    #
    #             # self.target_obj_x = world_objects[i+3]
    #             # self.target_obj_y = world_objects[i+4]
    #             # self.target_obj_z = world_objects[i+5]
    #             self.objectAdder.addBox(object_id, world_objects[i] + margin, world_objects[i+1] + margin, world_objects[i+2], world_objects[i+3], world_objects[i+4], world_objects[i+5])
    #             ## remove target object from world_objects???
    #             self.objectAdder.setColor(object_id, 1.0, 0.2, 0.2, a=1.0)
    #             ## Target object will have a different colour
    #
    #     else:
    #             self.objectAdder.addBox(object_id, world_objects[i] + margin, world_objects[i+1] + margin, world_objects[i+2], world_objects[i+3], world_objects[i+4], world_objects[i+5])
    #             self.objectAdder.setColor(object_id, 0.1, 0.4, 1.0, a=1.0)
    #             # All obstacles have the same colour
    #     # All the colours are set at the same time
    #     self.objectAdder.sendColors()
    #     i+=6

        # target_obj = ['0', 0.11087217926979065, 0.3403069078922272, 0.11774600305213856, 0.48992229410969163, 0.49934569425808273, 0.27480948858513754]


    # j = 0
    # while j < len(world_objects):
    #     if(world_objects[j]- <= self.target_obj)
    #
    #     j+=6
    # obj_ids=['0','1','2','table']
    # print(scene.get_object_poses(obj_ids))
    # obj = ['table']
    # objs = scene.get_objects()
    # print(objs)
    # print("---------------")
    # print(scene.get_known_object_names())
    # print("Number of objects in the scene: ")
    # print(len(scene.get_known_object_names()))

    # Add a mesh
    # self.leg_mesh = "/home/faisallab008/catkin_ws/src/universal_robot/ur_description/meshes/cube.stl" # or better, use find_package()
    # scene.add_mesh("mesh_test", box_pose, self.leg_mesh)


    #self.box_name=box_name
    # print(scene.get_known_object_names())
    return self.wait_for_state_update(box_is_known=True, timeout=4)

  def attach_box(self, timeout=4):

    box_name = self.box_name
    robot = self.robot
    scene = self.scene
    eef_link = self.eef_link
    group_names = self.group_names

    ## Attaching Objects to the Robot
    ## Attach the box to the robot and add "touch link" to tell the plannign scene to ingore collision between the object and the linkself.
    grasping_group = 'manipulator'
    touch_links = robot.get_link_names(group=grasping_group)
    self.objectAdder.setColor(self.target_obj[6],0.0,1.0,0.2,a=0.9)
    self.objectAdder.sendColors()
    scene.attach_box(eef_link, self.target_obj[6], touch_links=touch_links)
    # self.attached_box =

    # Wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

  def detach_box(self, timeout=4):

    box_name = self.box_name
    scene = self.scene
    eef_link = self.eef_link

    ## Detaching Objects from the Robot
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## We can also detach and remove the object from the planning scene:
    scene.remove_attached_object(eef_link, name=box_name)


    # Wait for the planning scene to update.
    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

  def remove_box(self, timeout=4):

    box_name = self.box_name
    scene = self.scene

    f = 0
    print("Number of objects found in the scene:" , len(scene.get_known_object_names()))
    no = len(scene.get_known_object_names())
    print(scene.get_known_object_names())
    while f < no:
        box_nam = str(f)
        scene.remove_world_object(box_nam)
        print("Removed object number ")
        print(f)
        f+=1

    #scene.remove_world_object(box_name)

    ## **Note:** The object must be detached before it can removed it from the world

    # Wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

  def knn_search(x, D, K):
       # find nearest bounding box to given point
       ndata = D.shape[1]

       K = K if K < ndata else ndata
       # euclidean distances from the other points
       sqd = np.sqrt(((D - x[:,:ndata])**2).sum(axis=0))
       idx = np.argsort(sqd) # sorting
       # return the index of the nearest neighbour
       return idx[:K]

def line_collision(x1,y1,x2,y2,x3,y3,x4,y4):
    uA = ((x4-x3)*(y1-y3) - (y4-y3)*(x1-x3)) / ((y4-y3)*(x2-x1) - (x4-x3)*(y2-y1))
    uB = ((x2-x1)*(y1-y3) - (y2-y1)*(x1-x3)) / ((y4-y3)*(x2-x1) - (x4-x3)*(y2-y1))
    # print(uA)
    # print(uB)
    if( 0<= uA <= 1 and 0<= uB <= 1):
        intersectionX = x1 + (uA*(x2-x1))
        intersectionY = y1 + (uB*(y2-y1))
        # print(intersectionX)
        # print(intersectionY)
        return True
    return False

def object_collision(e_co, grasp_point, objects_array, neig_idx):
    i = 0
    num_collisions = 0
    target_index = neig_idx
    # Add margin to avoid the objects with a safer distance
    margin = 0.03
    while i < len(objects_array):
        if(i/6 != neig_idx):
            # bottom = line_collision(e_co.x, e_co.y, grasp_point.x, grasp_point.y,objects_array[i+3]-objects_array[i+0]/2 - margin,objects_array[i+4]-objects_array[i+1]/2 - margin,objects_array[i+3]-objects_array[i+0]/2 - margin, objects_array[i+4]+objects_array[i+1]/2 + margin)
            # top = line_collision(e_co.x, e_co.y, grasp_point.x, grasp_point.y,objects_array[i+3]+objects_array[i+0]/2,objects_array[i+4]-objects_array[i+1]/2,objects_array[i+3]+objects_array[i+0]/2, objects_array[i+4]+objects_array[i+1]/2)
            # left = line_collision(e_co.x, e_co.y, grasp_point.x, grasp_point.y,objects_array[i+3]-objects_array[i+0]/2 - margin ,objects_array[i+4]+objects_array[i+1]/2 + margin ,objects_array[i+3]+objects_array[i+0]/2 + margin, objects_array[i+4]+objects_array[i+1]/2 + margin)
            # right = line_collision(e_co.x, e_co.y, grasp_point.x, grasp_point.y,objects_array[i+3]-objects_array[i+0]/2,objects_array[i+4]-objects_array[i+1]/2,objects_array[i+3]+objects_array[i+0]/2, objects_array[i+4]-objects_array[i+1]/2)

            bottom = line_collision(e_co.x, e_co.y, grasp_point.x, grasp_point.y,objects_array[i+3]-objects_array[i+0]/2 - margin,objects_array[i+4]-objects_array[i+1]/2 - margin,objects_array[i+3]-objects_array[i+0]/2 - margin, objects_array[i+4]+objects_array[i+1]/2 + margin)
            top = line_collision(e_co.x, e_co.y, grasp_point.x, grasp_point.y,objects_array[i+3]+objects_array[i+0]/2,objects_array[i+4]-objects_array[i+1]/2,objects_array[i+3]+objects_array[i+0]/2, objects_array[i+4]+objects_array[i+1]/2)
            left = line_collision(e_co.x, e_co.y, grasp_point.x, grasp_point.y,objects_array[i+3]-objects_array[i+0]/2,objects_array[i+4]+objects_array[i+1]/2,objects_array[i+3]+objects_array[i+0]/2, objects_array[i+4]+objects_array[i+1]/2)
            right = line_collision(e_co.x, e_co.y, grasp_point.x, grasp_point.y,objects_array[i+3]-objects_array[i+0]/2,objects_array[i+4]-objects_array[i+1]/2,objects_array[i+3]+objects_array[i+0]/2, objects_array[i+4]-objects_array[i+1]/2)

            # bottom = line_collision(e_co.x, e_co.y, grasp_point.x, grasp_point.y,objects_array[i+3]-objects_array[i]/2,objects_array[i+4]-objects_array[i+1]/2,objects_array[i+3]-objects_array[i]/2, objects_array[i+4]+objects_array[i+1]/2)
            # top = line_collision(e_co.x, e_co.y, grasp_point.x, grasp_point.y,objects_array[i+3]+objects_array[i]/2,objects_array[i+4]-objects_array[i+1]/2,objects_array[i+3]+objects_array[i]/2, objects_array[i+4]+objects_array[i+1]/2)
            # left = line_collision(e_co.x, e_co.y, grasp_point.x, grasp_point.y,objects_array[i+3]-objects_array[i]/2 - margin ,objects_array[i+4]+objects_array[i+1]/2 + margin ,objects_array[i+3]+objects_array[i+0]/2 + margin, objects_array[i+4]+objects_array[i+1]/2 + margin)
            # right = line_collision(e_co.x, e_co.y, grasp_point.x, grasp_point.y,objects_array[i+3]-objects_array[i]/2,objects_array[i+4]-objects_array[i+1]/2,objects_array[i+3]+objects_array[i]/2, objects_array[i+4]-objects_array[i+1]/2)

            if(bottom == True or top == True or left == True or right == True):
                # Number of collisions in a given xy position
                num_collisions+=1
        i+=6

    #print("number of collisions in state: %s" %num_collisions)
    if(num_collisions > 0):
        return True # There is at least one collision, the state is not possible
    else:
        return False # There are no collisions

def angle_ik(grasp_point):
    e_co = geometry_msgs.msg.Point()
    e_co.z = grasp_point.z

    sh_co = geometry_msgs.msg.Point()
    sh_co.x = -0.15
    sh_co.y = 0.38
    sh_co.z = 0.54
    u_length = 0.36
    f_length = 0.3
    #print("Grasp point: %s" %grasp_point)
    wrist_0_x = sh_co.x + f_length
    # elbow_angle = math.degrees(math.atan((grasp_point.y - sh_co.y)/(grasp_point.x - sh_co.x)))
    e_angle = math.atan((grasp_point.y - sh_co.y)/(grasp_point.x - sh_co.x - 0.29))
    elbow_angle = math.degrees(e_angle)
    # print("Elbow angle: %s" %elbow_angle)

    e_co.x = grasp_point.x - f_length*math.cos(e_angle)
    e_co.y = grasp_point.y - f_length*math.sin(e_angle)

    return (e_co, sh_co)

def astar(maze, start, end):
    """Returns a list of tuples as a path from the given start to the given end point"""

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0
    print(start_node.position)
    print(end_node.position)
    # Initialize open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)

    # Loop until you find the end
    while len(open_list) > 0:

        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                print(path[::-1])
                current = current.parent
            return path[::-1] # Return reversed path

        # Generate children
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # Adjacent squares

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] != 0:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:

            # Child is on the closed list
            for closed_child in closed_list:
                if child == closed_child:
                    continue

            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h

            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue

            # Add the child to the open list
            open_list.append(child)
            print(child.position)

def heuristic(a, b):
    return (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2

def astar2(array, start, goal):

    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]

    close_set = set()
    came_from = {}
    gscore = {start:0}
    fscore = {start:heuristic(start, goal)}
    oheap = []

    heappush(oheap, (fscore[start], start))

    while oheap:

        current = heappop(oheap)[1]

        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            return data[::-1] # Return reversed path

        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:
                    if array[neighbor[0]][neighbor[1]] == 1:
                        continue
                else:
                    # array bound y walls
                    continue
            else:
                # array bound x walls
                continue

            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue

            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score# + heuristic(neighbor, goal)
                heappush(oheap, (fscore[neighbor], neighbor))

    return False


if __name__ == '__main__':
	#main()
    commander = MoveGroupPythonInterface()
    commander.go_to_initial_state()
    commander.add_table()
    commander.path_executer_server()
    commander.path_planner_server()


    # commander.go_to_initial_state()
    # print(commander.robot.get_current_state())
    # commander.go_to_initial_state()
    # commander.add_table()
