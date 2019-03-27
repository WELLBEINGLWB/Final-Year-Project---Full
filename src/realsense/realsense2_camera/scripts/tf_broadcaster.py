#!/usr/bin/env python
import tf
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped  # import geometry_msgs.msg  pose = geometry_msgs.msg.Pose()
# from visualization_msgs.msg import Marker


# def callback(pose):
# 	# print(pose.pose)
# 	translation = (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
# 	q = (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w)
# 	rotation_euler = tf.transformations.euler_from_quaternion(q) #Convert a quaternion message into an angle in radians.
# 	# tf_brl.waitForTransform("camera_link", "mocap", rospy.Time(0),rospy.Duration(4.0))
# 	tf_br.sendTransform((translation[0],translation[1],translation[2]), (q[0], q[1], q[2], q[3]),rospy.Time.now(), "camera_link", "mocap")
# 	# tf_br.sendTransform((translation[0],translation[1],translation[2]), (q[0], q[1], q[2], q[3]),rospy.Time.now(), "RigidBody1", "mocap")
# 	# tf_brl.waitForTransform("mocap", "world", rospy.Time(0),rospy.Duration(4.0))
# 	tf_br.sendTransform((t[0], t[1], t[2]), (r[0], r[1], r[2], r[3]),rospy.Time.now(), "mocap", "world")
# 	# tf_br.sendTransform((t[0], t[1], t[2]), (r[0], r[1], r[2], r[3]),rospy.Time.now(), "RigidBody1", "world")
#
# 	# print("Tf sent")
# 	# tf_br.sendTransform((t[0], t[1], t[2]), (r[0], r[1], r[2], r[3]),rospy.Time.now(), "RigidBody_02", "world")
# #	tf_br2.sendTransform((translation[0],translation[1],translation[2]), (q[0], q[1], q[2], q[3]),pose.header.stamp, "camera_color_optical_frame", "mocap")
#
#
# if __name__ == '__main__':
# 	# Initialize the node
# 	rospy.init_node('camera_tf_node', anonymous="True")
# 	tf_br = tf.TransformBroadcaster() # Broadcaster for camera_link
# 	# world_br = tf.TransformBroadcaster() # Broadcaster for robot world
# 	# Transformation matrix
# 	# A = np.array([[0.99932430208527279, 0.0090973927011712696, 0.013948279486413945, 0.42786135388386903],
# 	#     [0.00027708163676559705, -0.0068271270060802358, -1.0010137094996121, 0.97182492778108842],
# 	#     [ -0.015069175090019371, 0.98950606361910065,  0.022382400003154883, 0.13883507833401487],
# 	#     [0.0, 0.0, 0.0, 1.0]])
#
# 	A = np.array([[1.0, 0, 0, 0.42786135388386903],
# 		    [0, 1.0, 0, 0.97182492778108842],
# 		    [ 0, 0,  1.0, 0.13883507833401487],
# 		    [0.0, 0.0, 0.0, 1.0]])
#
# 	r = tf.transformations.quaternion_from_matrix(A)
# 	t = tf.transformations.translation_from_matrix(A)
#
#
# 	rospy.Subscriber("/mocap/rigid_bodies/RigidBody1/pose", PoseStamped, callback, queue_size=1)
# 	rospy.spin()
#####################################

#
# [WARN] [WallTime: 1550689513.059872] Inbound TCP/IP connection failed: connection from sender terminated before handshake header received. 0 bytes were received. Please check sender for additional details.
# [ WARN] [1550689567.832457572]: MessageFilter [target=world ]: Dropped 100.00% of messages so far. Please turn the [ros.rviz.message_notifier] rosconsole logger to DEBUG for more information.
# [ WARN] [1550689567.832503357]: MessageFilter [target=world ]:   The majority of dropped messages were due to messages growing older than the TF cache time.  The last message's timestamp was: 1550677468.961002, and the last frame_id was: mocap
#



rospy.init_node('camera_tf_node', anonymous="True")
tf_br = tf.TransformBroadcaster() # Broadcaster for camera_link
# pub = rospy.Publisher('gaze', Point, queue_size=10)
rate = rospy.Rate(100) # 3 Hz
A = np.array([[1.0, 0.0090973927011712696, 0.013948279486413945, 0.42786135388386903],
	    [0.00027708163676559705, 1.0, 0.0, 0.97182492778108842],
	    [ -0.015069175090019371, 0.0,  1.0, 0.13883507833401487],
	    [0.0, 0.0, 0.0, 1.0]])

r = tf.transformations.quaternion_from_matrix(A)
t = tf.transformations.translation_from_matrix(A)

while not rospy.is_shutdown():
	# camera on top of box
	tf_br.sendTransform((-0.42427238822,-0.339268058538, 0.313522875309), ( -0.085530012846,0.201468780637,  0.0283537395298,  0.975341498852),rospy.Time.now(), "camera_link", "mocap")

	## tf_br.sendTransform((-0.589579701424,-0.279752343893,0.329931378365), (-0.0747203528881, 0.102493800223, 0.0811263248324, 0.988600313663),rospy.Time.now(), "camera_link", "mocap")
	# tf_br.sendTransform(( -0.686933040619,0.355587393045,0.316830247641), (0.0121359312907, 0.0546515248716,  0.227592453361, -0.972145974636),rospy.Time.now(), "camera_link", "mocap")
	tf_br.sendTransform((t[0], t[1], t[2]), (r[0], r[1], r[2], r[3]),rospy.Time.now(), "mocap", "world")
	rate.sleep()


'''
camera on top of box

  position:
    x: -0.42427238822
    y: -0.339268058538
    z: 0.313522875309
  orientation:
    x: -0.085530012846
    y: 0.201468780637
    z: 0.0283537395298
    w: 0.975341498852


 x: -0.431204140186
    y: -0.304282069206
    z: 0.319113969803
  orientation:
    x: 0.0686277598143
    y: -0.196518585086
    z: -0.0011869504815
    w: -0.978094816208



'''


'''
Camera pose for static experiment

header:
  seq: 43749
  stamp:
    secs: 1548861535
    nsecs: 317270517
  frame_id: mocap
pose:
  position:
    x: -0.707861959934
    y: -0.336078286171
    z: 0.231971353292
  orientation:
    x: 0.00498800771311
    y: -0.102054305375
    z: 0.069711573422
    w: -0.992320716381


	# static_pose = PoseStamped()
	# static_pose.header.stamp = rospy.Time.now()
	# static_pose.header.frame_id = "mocap"
	# static_pose.pose.position.x = -0.707861959934
	# static_pose.pose.position.y = -0.336078286171
	# static_pose.pose.position.z = 0.231971353292
	# static_pose.pose.orientation.x = 0.00498800771311
	# static_pose.pose.orientation.y = -0.102054305375
	# static_pose.pose.orientation.z = 0.069711573422
	# static_pose.pose.orientation.w = -0.992320716381



	Y-up:
	header:
  seq: 6746
  stamp:
    secs: 1548861533
    nsecs: 508814573
  frame_id: mocap
pose:
  position:
    x: -0.686933040619
    y: 0.355587393045
    z: 0.316830247641
  orientation:
    x: 0.0121359312907
    y: 0.0546515248716
    z: 0.227592453361
    w: -0.972145974636

'''
