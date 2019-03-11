#!/usr/bin/env python
import tf
import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import numpy as np
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
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


class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to the robot:
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface to one group of joints.
    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)

    ## Create a `DisplayTrajectory`_ publisher which is used later to publish trajectories for RViz to visualize:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

	# Getting Basic Information
	##################
    # Get the name of the reference frame for the robot:
    planning_frame = group.get_planning_frame()
    #print "============ Reference frame: %s" % planning_frame

    # Print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    #print "============ End effector: %s" % eef_link

    # Get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    #print "============ Robot Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the robot:
    #print "============ Printing robot state"
    #print robot.get_current_state()
    #print ""

	# Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

  def go_to_joint_state(self):
    group = self.group

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

  def go_to_pose_goal(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good reason not to.
    group = self.group

    pos_x = float(input("Enter x coordinate: "))
    pos_y = float(input("Enter y coordinate: "))
    pos_z = float(input("Enter z coordinate: "))
    or_w = float(input("Enter w orientation: "))

    ## Planning to a Pose Goal of the end-effector:
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = or_w
    pose_goal.position.x = pos_x
    pose_goal.position.y = pos_y
    pose_goal.position.z = pos_z
    group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

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

    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory);

  def execute_plan(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    group.execute(plan, wait=True)

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

  def add_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    # box_name = self.box_name
    scene = self.scene

    ## Adding Objects to the Planning Scene

    ## Create a box in the planning scene at the location of the end effector:
    # box_pose = geometry_msgs.msg.PoseStamped()
    # box_pose.header.frame_id = "ee_link"
    # box_pose.pose.orientation.w = 1.0
    # box_name = "box"
    # box_x = float(input("Enter x dimension: "))
    # box_y = float(input("Enter y dimension: "))
    # box_z = float(input("Enter z dimension: "))
    # scene.add_box(box_name, box_pose, size=( box_x, box_y, box_z))

    objects =  [0.17243778705596924, 0.1683375120162964, 0.1370002746582031, -1.2913718223571777, -0.9397485852241516, 0.703500270843506,
    0.12552762031555176, 0.10640859603881836, 0.18959981203079224, 0.4404555559158325, -0.33142730593681335, 0.9314001798629761,
    0.14826665818691254, 0.11540680378675461, 0.12574449181556702, -0.12143077701330185, -0.046661581844091415, 0.4185221791267395,
    0.07196833193302155, 0.07469882071018219, 0.08375966548919678, 0.005321848206222057, -0.14881637692451477, 0.512786865234375,
    0.07071880251169205, 0.06149909645318985, 0.04001554846763611, 0.14499202370643616, -0.05652853474020958, 0.41299229860305786]
    # Number of objects in the array (each has 6 dimensions)
    num_objects = len(objects)/6
    print("Number of objects: ")
    print(num_objects)
    # obj_name={}
    # for n in range(1,10):
    #     obj_name["object{0}".format(n)]="Hello"

    i = 0
    while i < len(objects):
        box_name = self.box_name
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.position.x = objects[i+3]
        box_pose.pose.position.y = objects[i+4]
        box_pose.pose.position.z = objects[i+5]
        box_pose.pose.orientation.w = 1.0
        box_name = str(i/6)
        scene.add_box(box_name, box_pose, size=(objects[i], objects[i+1], objects[i+2]))
        print("Added object number: ")
        print(i/6)
        i += 6

    print(scene.get_known_object_names())
    # print("Number of objects in the scene: ")
    # print(len(scene.get_known_object_names()))

    table_pose = geometry_msgs.msg.PoseStamped()
    table_pose.header.frame_id = "world"
    table_pose.pose.orientation.w = 1.0
    table_pose.pose.position.x = 0.7
    table_pose.pose.position.y = 0.0
    table_pose.pose.position.z = -0.1
    # scene.add_box("table", table_pose, size=( 1.0, 2.0, 0.05))
    # scene.add_box("table", table_pose, size=( 0.05, 0.5, 0.05))
    # scene.add_box("2", table_pose, size=( 1.0, 0.05, 0.05))

    # Add a mesh
    # self.leg_mesh = "/home/faisallab008/catkin_ws/src/universal_robot/ur_description/meshes/cube.stl" # or better, use find_package()
    # scene.add_mesh("mesh_test", box_pose, self.leg_mesh)

    # Copy local variables back to class variables. In practice, you should use the class
    # variables directly unless you have a good reason not to.
    self.box_name=box_name
    print(scene.get_known_object_names())
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)

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
    scene.attach_box(eef_link, box_name, touch_links=touch_links)

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

    commander = MoveGroupPythonIntefaceTutorial()
    option = True
    # rospy.init_node('send_pose', anonymous="True")
    # rospy.Subscriber("/objects_array", Float32MultiArray, commander.add_box(), queue_size=1)
#    rospy.spin()

    while option:
        print("""
        1) Go to joint state
        2) Go to pose goal
        3) Add object to scene
        4) Remove Objects
        5) Exit
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
          commander.add_box()
        elif option=="4":
          print("\n ")
          commander.remove_box()
        elif option=="5":
          print("\n Exiting")
          break
        elif option !="":
          print("\n Invalid input, try again")

            # 1) Attach to robot
            # 2) Detach from robot
            # 3) Remove object from scene
            # 4) Main menu

if __name__ == '__main__':
	main()
