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
import shape_msgs.msg
from math import pi
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker
from moveit_commander.conversions import pose_to_list



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




class MoveGroupPythonInterface(object):
  """MoveGroupPythonInterface"""
  def __init__(self):
    super(MoveGroupPythonInterface, self).__init__()


    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface', anonymous=True)

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

	# Getting Basic Information
	##################
    # Get the name of the reference frame for the robot:
    planning_frame = group.get_planning_frame()
    #print "============ Reference frame: %s" % planning_frame

    # Print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()

    # Get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    #print "Robot Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the robot:
    #print "============ Printing robot state"
    #print robot.get_current_state()
    #print ""

    self.objectAdder = moveit_python.PlanningSceneInterface("world")
    # groupTest.addCube("my_cube", 0.5, 0.7, 0.5, 0.5)
    # groupTest.addBox("name", size_x, size_y, size_z, x, y, z)
    # groupTest.addBox("name", 0.1, 0.1, 0.2, 0.4, 0.6, 0.25)
    # groupTest.setColor("name", 0.2, 1.0, 0.1, a=0.9)
    # groupTest.sendColors()

	# Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
    # self.objs = []

  def poseCallback(self, pose):
      self.q = (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w)
      self.translation = (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
      self.q = 5
          # self.poseCallback()
      print(self.q)

  def gaze_callback(self,point):
      self.gaze_point.x = point.x
      self.gaze_point.y = point.y
      self.gaze_point.z = point.z

  def go_to_joint_state(self):
      # most likely to be deleted
    group = self.group
    group.set_planning_time(10)
    elbow_joint = float(input("Enter angle for elbow_joint: "))
    shoulder_lift_joint = float(input("Enter angle for shoulder_lift_joint: "))
    shoulder_pan_joint = float(input("Enter angle for shoulder_pan_joint: "))
    wrist_1_joint = float(input("Enter angle for wrist_1_joint: "))
    wrist_2_joint = float(input("Enter angle for wrist_2_joint: "))
    wrist_3_joint = float(input("Enter angle for wrist_3_joint: "))

    joint_goal = group.get_current_joint_values()
    joint_goal[0] = elbow_joint # elbow_joint
    joint_goal[1] = shoulder_lift_joint # shoulder_lift_joint
    joint_goal[2] = shoulder_pan_joint # shoulder_pan_joint
    joint_goal[3] = wrist_1_joint # wrist_1_joint
    joint_goal[4] = wrist_2_joint # wrist_2_joint
    joint_goal[5] = wrist_3_joint # wrist_3_joint

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    group.stop()

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def go_to_initial_state(self):
    group = self.group

    joint_goal = group.get_current_joint_values()
    # joint_goal[0] = 0 # elbow_joint
    # joint_goal[1] = -4*pi/5 # shoulder_lift_joint
    # joint_goal[2] = 2*pi/3 # shoulder_pan_joint
    # joint_goal[3] = -pi/2 # wrist_1_joint
    # joint_goal[4] = -pi/2 # wrist_2_joint
    # joint_goal[5] = 0 # wrist_3_joint
    joint_goal[0] = 0.61 # shoulder_pan_joint
    joint_goal[1] = -1.87 # shoulder_lift_joint
    joint_goal[2] = 2.44 # elbow_joint
    joint_goal[3] = -2.14 # wrist_1_joint
    joint_goal[4] = -1.57 # wrist_2_joint
    joint_goal[5] = 0.61 # wrist_3_joint
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
    #
    # # Append the constraint to the list of contraints
    constraint.orientation_constraints.append(orientation_constraint)

    group.set_path_constraints(constraint)
    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    group.stop()

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def go_to_hand_state(self):
      # to be deleted
    group = self.group

    joint_goal = group.get_current_joint_values()
    joint_goal[0] = pi/4 # elbow_joint
    joint_goal[1] = -2*pi/5 # shoulder_lift_joint
    joint_goal[2] = 2*pi/3 # shoulder_pan_joint
    joint_goal[3] = -pi # wrist_1_joint
    joint_goal[4] = -pi/2 # wrist_2_joint
    joint_goal[5] = 0 # wrist_3_joint

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    group.stop()

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def go_to_pose_goal(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good reason not to.
    group = self.group
    scene = self.scene
    group.set_planning_time(10)

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

    # Positin constraint that define the workspace of the end effector link
    primitive = shape_msgs.msg.SolidPrimitive()
    primitive.type = primitive.BOX
    dim = [0.5,1.7,0.2]
    primitive.dimensions = dim
    ws_pose = geometry_msgs.msg.Pose();
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
    # or_w = float(input("Enter w orientation: "))

    ## Planning to a Pose Goal of the end-effector:
    pose_goal = geometry_msgs.msg.Pose()
    # pose_goal = geometry_msgs.msg.PoseStamped()
    # pose_goal.header.frame_id = group.get_end_effector_link()
    pose_goal.orientation.x = 0.00111054358639
    pose_goal.orientation.y = 0.70699483645
    pose_goal.orientation.z = 0.00111089701837
    pose_goal.orientation.w = 0.707216963763
    pose_goal.position.x = pos_x
    pose_goal.position.y = pos_y
    pose_goal.position.z = pos_z

    waypoints = []
    # x = 0.40
    # y = 0.70
    # z = 0.25
    wpose = group.get_current_pose().pose

    # wpose.position.y = 0.45
    # wpose.position.x = 0.4
    # waypoints.append(copy.deepcopy(wpose))
    #
    # wpose.position.y = 0.6
    # wpose.position.x = 0.2
    # waypoints.append(copy.deepcopy(wpose))

    wpose.position.y = 0.4
    wpose.position.x = 0.4
    wpose.position.z = 0.25
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y = 0.8
    wpose.position.x = 0.4
    wpose.position.z = 0.25
    waypoints.append(copy.deepcopy(wpose))

    # # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # # which is why we will specify 0.01 as the eef_step in Cartesian
    # # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    # (plan, fraction) = group.compute_cartesian_path(
    #                                    waypoints,   # waypoints to follow
    #                                    0.01,        # eef_step
    #                                    0.0, True)         # jump_threshold


    group.set_max_acceleration_scaling_factor(0.1)
    # group.get_planner_id()
    # [minX, minY, minZ, maxX, maxY, maxZ]
    # ws = [0.0,0.0,0.0,0.0,0.1,0.1]
    # group.set_workspace(ws)
    group.set_pose_target(pose_goal, group.get_end_effector_link())
    # group.execute(plan, wait=True)

    # Call the planner to compute the plan and execute it.
    group.go(wait=True)
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
    constraint = moveit_msgs.msg.Constraints()
    constraint.name = "fixed wrist orientation"
    #
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

    joint_constraint = moveit_msgs.msg.JointConstraint()

    # group.setMaxVelocityScalingFactor(0.1);


    # Set the path constraints on the end effector
    group.set_path_constraints(constraint)


    pos_x = float(input("Enter x coordinate: "))
    pos_y = float(input("Enter y coordinate: "))
    pos_z = float(input("Enter z coordinate: "))

    pose_goal = geometry_msgs.msg.Pose()

    pose_goal.orientation.x = 0.00111054358639
    pose_goal.orientation.y = 0.70699483645
    pose_goal.orientation.z = 0.00111089701837
    pose_goal.orientation.w = 0.707216963763
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

    group.set_max_acceleration_scaling_factor(0.1)
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
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    # group.clear_pose_targets()

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    # current_pose = self.group.get_current_pose().pose
    # return all_close(pose_goal, current_pose, 0.01)

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
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
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
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL wait_for_scene_update
    ##
    ## Ensuring Collision Updates Are Receieved
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## If the Python node dies before publishing a collision object update message, the message
    ## could get lost and the box will not appear. To ensure that the updates are
    ## made, we wait until we see the changes reflected in the
    ## ``get_known_object_names()`` and ``get_known_object_names()`` lists.
    ## For the purpose of this tutorial, we call this function after adding,
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
    table_pose.pose.position.x = 0.44
    table_pose.pose.position.y = 0.65
    table_pose.pose.position.z = -0.24
    # scene.add_box("table", table_pose, size=( 0.7, 1.7, 0.85))

    self.objectAdder.addBox("table", 0.7, 1.7, 0.85, 0.44, 0.65, -0.24)
    self.objectAdder.setColor("table", 0.1, 1.0, 0.2, a=0.9)

    table_pose.pose.position.x = 0.40
    table_pose.pose.position.y = 0.70
    table_pose.pose.position.z = 0.28
    scene.add_box("dummy0", table_pose, size=( 0.055, 0.055, 0.15))

    table_pose.pose.position.x = 0.40
    table_pose.pose.position.y = 0.90
    table_pose.pose.position.z = 0.25
    scene.add_box("dummy1", table_pose, size=( 0.055, 0.055, 0.15))

    table_pose.pose.position.x = 0.40
    table_pose.pose.position.y = 0.45
    table_pose.pose.position.z = 0.20
    scene.add_box("dummy2", table_pose, size=( 0.055, 0.055, 0.05))



    # table_pose.header.frame_id = "world"
    table_pose.pose.position.x = 0.44
    table_pose.pose.position.y = 0.65
    table_pose.pose.position.z = 0.25
    # scene.add_box("ws", table_pose, size=( 0.5, 1.7, 0.2))

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

    top_pose = geometry_msgs.msg.PoseStamped()
    top_pose.header.frame_id = "base"
    top_pose.pose.orientation.w = 0.0
    top_pose.pose.position.x = -0.7
    top_pose.pose.position.y = -1.0
    top_pose.pose.position.z = 0.75
    # scene.add_box("ceilling", top_pose, size=( 1.0, 2.0, 0.05))


    top_pose.pose.position.x = 0.3
    top_pose.pose.position.y = -0.7
    top_pose.pose.position.z = 0.2
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

  def add_box(self, msg):

    # box_name = self.box_name
    scene = self.scene

    f = 0
    # print("Number of objects found in the scene:" , len(scene.get_known_object_names()))
    no = len(scene.get_known_object_names())
    while f < no:
        box_nam = str(f)
        scene.remove_world_object(box_nam)
        # print("Removed object number ")
        # print(f)
        f+=1

    # print(msg.data)
    ## Adding Objects to the Planning Scene
    objects = msg.data
    # print(objects)
    # objects =  [0.17243778705596924, 0.1683375120162964, 0.1370002746582031, -1.2913718223571777, -0.9397485852241516, 0.703500270843506,
    # 0.12552762031555176, 0.10640859603881836, 0.18959981203079224, 0.4404555559158325, -0.33142730593681335, 0.9314001798629761,
    # 0.14826665818691254, 0.11540680378675461, 0.12574449181556702, -0.12143077701330185, -0.046661581844091415, 0.4185221791267395,
    # 0.07196833193302155, 0.07469882071018219, 0.08375966548919678, 0.005321848206222057, -0.14881637692451477, 0.512786865234375,
    # 0.07071880251169205, 0.06149909645318985, 0.04001554846763611, 0.14499202370643616, -0.05652853474020958, 0.41299229860305786]
    # Number of objects in the array (each has 6 dimensions)
    num_objects = len(objects)/6
    # print("Number of objects: ")
    # print(num_objects)
    # obj_name={}
    # for n in range(1,10):
    #     obj_name["object{0}".format(n)]="Hello"

    i = 0

    # Extra margin to add to the bounding boxes (in meters)
    margin = 0.0

    transformer = tf.TransformListener()

    # Iterating through the array from the segmentation noe and adding object with the respective center point coordinates and dimensions
    # The array is structurer as follows: [xDim, yDim, zDim, xCenter, yCenter, zCenter]
    while i < len(objects):

        # Transform the center point of the object from cameraLink frame to world frame
        transformer.waitForTransform("camera_link", "world", rospy.Time(0),rospy.Duration(4.0))
        pointstamp = geometry_msgs.msg.PointStamped()
        pointstamp.header.frame_id = "camera_link"
        pointstamp.header.stamp = rospy.Time()
        pointstamp.point.x = objects[i+3]
        pointstamp.point.y = objects[i+4]
        pointstamp.point.z = objects[i+5]
        # Converting the center point of the
        # objects[i+3] = pointstamp.point.x
        # objects[i+4] = pointstamp.point.y
        # objects[i+5] = pointstamp.point.z

        # print("Before: ")
        # print(pointstamp.point.x)
        # print(pointstamp.point.y)
        # print(pointstamp.point.z)
        # transformer.waitForTransform("camera_link", "world", rospy.Time(0),rospy.Duration(4.0))
        p_tr = transformer.transformPoint("world", pointstamp)
        # print("After: ")
        # print(p_tr.point.x)
        # print(p_tr.point.y)
        # print(p_tr.point.z)

        height_to_table = 0.185 - (p_tr.point.z - (objects[i+2]/2))
        # print("Object height: ")
        # print(objects[i+2])
        # print("Height to table: ")
        # print(height_to_table)
        # print("------- ")

        box_name = self.box_name
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.position.x = p_tr.point.x
        box_pose.pose.position.y = p_tr.point.y
        # box_pose.pose.position.z = p_tr.point.z  # - height_to_table

        box_pose.pose.position.z = 0.185 + (objects[i+2]/2)
        box_pose.pose.position.z = box_pose.pose.position.z + (p_tr.point.z - box_pose.pose.position.z)/2
        # objects[i+2] = objects[i+2] + (p_tr.point.z - box_pose.pose.position.z)


        box_name = str(i/6)
        scene.add_box(box_name, box_pose, size=(objects[i] + margin, objects[i+1] + margin, objects[i+2] + (p_tr.point.z - box_pose.pose.position.z)))
        # print( box_pose.pose.position.z - (objects[i+2] + (p_tr.point.z - box_pose.pose.position.z))/2)

        # scene.setColor(box_name, 0.5, 0.5, 0.5, 1.0)
        # print("Added object number: ")
        # print(i/6)
        i += 6


    while i < len(objects):

        # Transform the center point of the object from cameraLink frame to world frame
        transformer.waitForTransform("camera_link", "world", rospy.Time(0),rospy.Duration(4.0))
        pointstamp = geometry_msgs.msg.PointStamped()
        pointstamp.header.frame_id = "camera_link"
        pointstamp.header.stamp = rospy.Time()
        pointstamp.point.x = objects[i+3]
        pointstamp.point.y = objects[i+4]
        pointstamp.point.z = objects[i+5]
        # Converting the center point if the object to /world frame
        objects[i+3] = pointstamp.point.x
        objects[i+4] = pointstamp.point.y
        objects[i+5] = pointstamp.point.z

        # print("Before: ")
        # print(pointstamp.point.x)
        # print(pointstamp.point.y)
        # print(pointstamp.point.z)
        # transformer.waitForTransform("camera_link", "world", rospy.Time(0),rospy.Duration(4.0))
        p_tr = transformer.transformPoint("world", pointstamp)
        # print("After: ")
        # print(p_tr.point.x)
        # print(p_tr.point.y)
        # print(p_tr.point.z)

        height_to_table = 0.185 - (p_tr.point.z - (objects[i+2]/2))
        # print("Object height: ")
        # print(objects[i+2])
        # print("Height to table: ")
        # print(height_to_table)
        # print("------- ")

        box_name = self.box_name
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.position.x = p_tr.point.x
        box_pose.pose.position.y = p_tr.point.y
        # box_pose.pose.position.z = p_tr.point.z  # - height_to_table

        box_pose.pose.position.z = 0.185 + (objects[i+2]/2)
        box_pose.pose.position.z = box_pose.pose.position.z + (p_tr.point.z - box_pose.pose.position.z)/2
        # objects[i+2] = objects[i+2] + (p_tr.point.z - box_pose.pose.position.z)


        box_name = str(i/6)
        scene.add_box(box_name, box_pose, size=(objects[i] + margin, objects[i+1] + margin, objects[i+2] + (p_tr.point.z - box_pose.pose.position.z)))

        ## Gaze integration
        # gaze_margin = 0 #0.042
        # objects = [1.0,1.0,1.0,1.0,1.0,1.0]
        # # Finding whithin which object the gaze point lies
        # for j in objects:
        #     obj_name = str(j/6)
        #     if (((objects[j+3] - (objects[j]/2) - gaze_margin) <= point.x <= (objects[j+3] + (objects[j]/2) + gaze_margin))
        #       and ((objects[j+4] - (objects[j+1]/2) - gaze_margin) <= point.y <= (objects[j+4] + (objects[j+1]/2) + gaze_margin))
        #       and ((objects[j+5] - (objects[j+2]/2) - gaze_margin) <= point.y <= (objects[j+5] + (objects[j+2]/2) + gaze_margin))):
        #
        #         target_obj.name = obj_name
        #         target_obj.x = objects[j+3]
        #         target_obj.y = objects[j+4]
        #         target_obj.z = objects[j+5]
        #         self.objectAdder.addBox(str(i), )
        #
        #     else:
        #         self.objectAdder.addBox(str(i), )
        #         self.objectAdder.setColor(s)
        #     j+=6
        #
        # self.objectAdder.sendColors()
        # # print( box_pose.pose.position.z - (objects[i+2] + (p_tr.point.z - box_pose.pose.position.z))/2)
        # i += 6


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


    self.box_name=box_name
    # print(scene.get_known_object_names())
    return self.wait_for_state_update(box_is_known=True, timeout=4)

  def attach_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good reason not to.
    box_name = self.box_name
    robot = self.robot
    scene = self.scene
    eef_link = self.eef_link
    group_names = self.group_names

    ## Attaching Objects to the Robot
    ## Attach the box to the robot and add "touch link" to tell the plannign scene to ingore collision between the object and the linkself.
    grasping_group = 'manipulator'
    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box(eef_link, "dummy0", touch_links=touch_links)
    # self.attached_box =

    # Wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

  def detach_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good reason not to.
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
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good reason not to.
    box_name = self.box_name
    scene = self.scene

    ## Remove the box from the world.
    # while op:
    #     print("""
    #     1) Remove all objects
    #     2) Remove one object
    #     3) Back to main menu
    #     """)
    #
    #     op=raw_input("Choose an option ")
    #     if option=="1":
    #       print("\n ")
    #       #remove all objects
    #     elif op=="2":
    #       print("\n ")
    #       #obj_rem = raw_input("Which object would you like to remove?")
    #     elif op=="3":
    #       print("\n ")
    #       break
    #     elif option !="":
    #       print("\n Invalid input, try again")

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

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

def main():

    commander = MoveGroupPythonInterface()
    option = True
    #rospy.init_node('send_pose', anonymous="True")

    commander.go_to_initial_state()

    rospy.Subscriber("/objects_array", Float32MultiArray, commander.add_box, queue_size=1)
    rospy.Subscriber("/gaze_point", geometry_msgs.msg.Point, commander.gaze_callback, queue_size=1)
    # rospy.Subscriber("/mocap/rigid_bodies/RigidBody1/pose", geometry_msgs.msg.PoseStamped, commander.poseCallback, queue_size=1)

    rospy.sleep(0.5)
    # commander.add_table()

    while option:
        print("""
        1) Go to joint state
        2) Go to pose goal
        3) Add table
        4) Remove Objects
        5) Go to initial state
        6) Attach object
        7) Plan pose Goal
        8) Display trajectory
        9) Go to planned trajectory
        10) Exit
        """)

        option=raw_input("Choose an option ")
        if option=="1":
          print("\n ")
          commander.go_to_joint_state()
        elif option=="2":
          print("\n ")
          commander.go_to_pose_goal()
        elif option=="3":
          print("\n ")
          commander.add_table()
        elif option=="4":
          print("\n ")
          commander.remove_box()
        elif option=="5":
            print("\n ")
            commander.go_to_initial_state()
        elif option=="6":
            print("\n ")
            commander.attach_box()
        elif option=="7":
          print("\n Plan pose")
          commander.plan_pose_goal()
        elif option=="8":
          print("\n Plan pose")
          commander.display_trajectory(commander.pose_goal)
        elif option=="9":
          print("\n Plan pose")
          commander.execute_plan(commander.pose_goal)
        elif option=="10":
          print("\n Exit")
          break
        elif option !="":
          print("\n Invalid input, try again")


if __name__ == '__main__':
	main()
