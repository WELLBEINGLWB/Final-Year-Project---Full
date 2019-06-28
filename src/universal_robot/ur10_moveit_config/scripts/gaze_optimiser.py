#!/usr/bin/env python

import tf
import rospy
import geometry_msgs.msg
import pylab
import matplotlib.pyplot as plt
import numpy as np
import math
import matplotlib.patches as patches
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import PointCloud2
from segmentation.srv import*
from mpl_toolkits.mplot3d import axes3d
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib import colors

def gaze_optimiser_server():
    # Initialize server proxy for gaze omptimiser service
    s = rospy.Service('gaze_optimiser_service', gazeOptimiser, handle_objects)
    print "Ready to receive objects."
    rospy.spin()
    print("Not get here")

def knn_search(x, D, K):
     # find nearest bounding box to gaze point in order to identify target object
     ndata = D.shape[1]

     K = K if K < ndata else ndata
     # euclidean distances from the other points
     sqd = np.sqrt(((D - x[:,:ndata])**2).sum(axis=0)) # calculating Euclidean distance
     idx = np.argsort(sqd) # sorting the array

     # return the indexes of nearest neighbour
     return idx[:K]

def correct_ik(grasp_point, wrist_co):
    e_co = geometry_msgs.msg.Point()
    sh_co = geometry_msgs.msg.Point()
    # wrist_co = geometry_msgs.msg.Point()
    # wrist_co = self.group.get_current_pose().pose.position
    e_co.z = grasp_point.z

    sh_co.x = -0.18
    sh_co.y = 0.36
    sh_co.z = 0.54
    u_length = 0.36 # upepr arm length
    f_length = 0.38 # forearm length

    #print("Grasp point: %s" %grasp_point)
    # wrist_0_x = .x - f_length
    # elbow_angle = math.degrees(math.atan((grasp_point.y - sh_co.y)/(grasp_point.x - sh_co.x)))
    # e_angle = math.atan((grasp_point.y - sh_co.y)/(grasp_point.x - sh_co.x - 0.29))
    e_angle = math.atan((grasp_point.y - wrist_co.y)/(grasp_point.x - wrist_co.x + f_length))
    elbow_angle = math.degrees(e_angle)
    # print("Elbow angle: %s" %elbow_angle)

    e_co.x = grasp_point.x - f_length*math.cos(e_angle)
    e_co.y = grasp_point.y - f_length*math.sin(e_angle)

    return (e_co, sh_co)

# def line_collision(x1,y1,x2,y2,x3,y3,x4,y4):
#     uA = ((x4-x3)*(y1-y3) - (y4-y3)*(x1-x3)) / ((y4-y3)*(x2-x1) - (x4-x3)*(y2-y1))
#     uB = ((x2-x1)*(y1-y3) - (y2-y1)*(x1-x3)) / ((y4-y3)*(x2-x1) - (x4-x3)*(y2-y1))
#     # print(uA)
#     # print(uB)
#     if( 0<= uA <= 1 and 0<= uB <= 1):
#         intersectionX = x1 + (uA*(x2-x1))
#         intersectionY = y1 + (uB*(y2-y1))
#         # print(intersectionX)
#         # print(intersectionY)
#         return True
#     return False
#
# def object_collision(e_co, grasp_point, objects_array, neig_idx):
#     i = 0
#     num_collisions = 0
#     target_index = neig_idx
#     # Add margin to avoid the objects with a safer distance
#     margin = 0.0
#     while i < len(objects_array):
#         if(i/6 != neig_idx):
#             bottom = line_collision(e_co.x, e_co.y, grasp_point.x, grasp_point.y,objects_array[i+3]-objects_array[i+0]/2 - margin,objects_array[i+4]-objects_array[i+1]/2 - margin,objects_array[i+3]-objects_array[i+0]/2 - margin, objects_array[i+4]+objects_array[i+1]/2 + margin)
#             top = line_collision(e_co.x, e_co.y, grasp_point.x, grasp_point.y,objects_array[i+3]+objects_array[i+0]/2,objects_array[i+4]-objects_array[i+1]/2,objects_array[i+3]+objects_array[i+0]/2, objects_array[i+4]+objects_array[i+1]/2)
#             left = line_collision(e_co.x, e_co.y, grasp_point.x, grasp_point.y,objects_array[i+3]-objects_array[i+0]/2 - margin ,objects_array[i+4]+objects_array[i+1]/2 + margin ,objects_array[i+3]+objects_array[i+0]/2 + margin, objects_array[i+4]+objects_array[i+1]/2 + margin)
#             right = line_collision(e_co.x, e_co.y, grasp_point.x, grasp_point.y,objects_array[i+3]-objects_array[i+0]/2,objects_array[i+4]-objects_array[i+1]/2,objects_array[i+3]+objects_array[i+0]/2, objects_array[i+4]-objects_array[i+1]/2)
#             if(bottom == True or top == True or left == True or right == True):
#                 # Number of collisions in a given xy position
#                 num_collisions+=1
#         i+=6
#
#     #print("number of collisions in state: %s" %num_collisions)
#     if(num_collisions > 0):
#         return True # There is at least one collision, the state is not possible
#     else:
#         return False # There are no collisions

def handle_objects(req):

    transformer = tf.TransformListener()

    # Iterating through the array from the segmentation noe and adding object with the respective center point coordinates and dimensions
    # The array is structurer as follows: [xDim, yDim, zDim, xCenter, yCenter, zCenter]

    objects = req.objects.data
    gaze = req.gaze_point
    print(objects)
    print("--------------")
    print("Gaze point: %s" %gaze)
    num_objects = int(len(objects)/6) # number of object

    world_objects = Float32MultiArray()
    world_objects.data = [0]*len(objects)
    center = np.empty([2, num_objects]) # array that will contain the x,y center of the objects

    i = 0
    while i < len(objects):

        # Transform the center point of the object from camera_link frame to world frame
        transformer.waitForTransform("camera_link", "world", rospy.Time(0),rospy.Duration(4.0))
        pointstamp = geometry_msgs.msg.PointStamped()
        pointstamp.header.frame_id = "camera_link"
        pointstamp.header.stamp = rospy.Time()
        pointstamp.point.x = objects[i+3]
        pointstamp.point.y = objects[i+4]
        pointstamp.point.z = objects[i+5]

        # Converting the center point of the object to /world frame
        p_tr = transformer.transformPoint("world", pointstamp)

        height_to_table = 0.165 - (p_tr.point.z - (objects[i+2]/2))

        box_z = 0.11 + (objects[i+2]/2)
        box_z = box_z + (p_tr.point.z - box_z)/2

        world_objects.data[i] = objects[i]
        world_objects.data[i+1] = objects[i+1]
        world_objects.data[i+2] = objects[i+2] + (p_tr.point.z - box_z)
        world_objects.data[i+3] = p_tr.point.x
        world_objects.data[i+4] = p_tr.point.y
        world_objects.data[i+5] = box_z

        object_id = str(i/6)

        print("x,y,z = ")
        print(world_objects.data[i+3])
        print(world_objects.data[i+4])
        print(world_objects.data[i+5])
        print("------")

        i+=6

    # 1 for plots and animation, 0 for no plos
    plot_request = 0
    if(plot_request==1):
         fig = plt.figure(figsize=(10,10/(1.7/0.8)))
         ax = fig.add_subplot(111)
         # plt.cla()
         ax.axis([-0.2,1.7, -0.2, 0.75])
         plt.gca().invert_xaxis()

    j = 0
    while j < len(objects):
        iteration = int(j/6)
        center[0,iteration] = world_objects.data[j+3]
        center[1,iteration] = world_objects.data[j+4]
        if(plot_request==1):
            rect = patches.Rectangle((center[1,iteration]-world_objects.data[j+1]/2,center[0,iteration]-world_objects.data[j]/2),world_objects.data[j+1],world_objects.data[j],linewidth=1,edgecolor='r',facecolor='none')
            ax.add_patch(rect)
        j+=6

    print(center)

    x = np.array([[gaze.x],[gaze.y]])
    neig_idx = knn_search(x,center,1)
    print("Index %s" %neig_idx)
    print("target object xyz:")
    print(world_objects.data[neig_idx*6 + 3])
    print(world_objects.data[neig_idx*6 + 4])
    print(world_objects.data[neig_idx*6 + 5])
    offset_x = -0.015
    offset_y = -0.015

    grasp_point = geometry_msgs.msg.Point()
    grasp_point.x = world_objects.data[neig_idx*6 + 3] - offset_x # - world_objects.data[neig_idx*6]/2
    grasp_point.y = world_objects.data[neig_idx*6 + 4] - offset_y #- world_objects.data[neig_idx*6 + 1]/2 - offset_y
    grasp_point.z = world_objects.data[neig_idx*6 + 5]

    print("Grasp point: %s" %grasp_point)

    wrist_co = geometry_msgs.msg.Point()
    wrist_co.x = 0.2
    wrist_co.y = 0.36
    wrist_co.z = 0.2

    co_e, co_s = correct_ik(grasp_point, wrist_co)
    print("Coord %s" %co_e)


    if(plot_request==1):
    # highlighting the target object center
        pylab.plot(center[1,neig_idx],center[0,neig_idx],'o', markerfacecolor='None',markersize=15,markeredgewidth=1)
        ax.plot(center[1,:],center[0,:],'ob',x[1,0],x[0,0],'or',grasp_point.y,grasp_point.x,'og')
        ax.plot(center[1,neig_idx],center[0,neig_idx],'o', markerfacecolor='None',markersize=15,markeredgewidth=1)
        plt.plot([co_s.y,co_e.y,grasp_point.y],[co_s.x,co_e.x,grasp_point.x])
        plt.plot([wrist_co.y,grasp_point.y],[wrist_co.x - 0.05,grasp_point.x + offset_x])
        plt.plot([co_s.y,co_e.y,grasp_point.y],[co_s.x,co_e.x,grasp_point.x],'o')
        plt.show(block=False)
        rospy.sleep(5.0)
        plt.close('all')

    print("Grasp point: %s" %grasp_point)
    print("Elbow point: %s" %co_e)

    index = int(neig_idx)
    print("Index %s" %index)

    srv_response = gazeOptimiserResponse()
    srv_response.sorted_objects = world_objects
    srv_response.target_id = index
    srv_response.grasp_point = grasp_point

    return srv_response

if __name__ == '__main__':
    rospy.init_node('gaze_optimiser', anonymous=True)
    # Initialize the gaze optimiser service
    gaze_optimiser_server()
    print ("Ready to receive objects.")
