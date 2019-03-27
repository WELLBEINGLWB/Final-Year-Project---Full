#!/usr/bin/env python
import tf
import rospy
import geometry_msgs.msg

import numpy as np
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import PointCloud2
from segmentation.srv import*


import pylab
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
import matplotlib.patches as patches
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def gaze_optimiser_server():
    s = rospy.Service('gaze_optimiser_service', gazeOptimiser, handle_objects)
    print "Ready to receive objects."
    rospy.spin()

def knn_search(x, D, K):
     """ find nearest neighbours of gaze point """
     ndata = D.shape[1]

     K = K if K < ndata else ndata
     # euclidean distances from the other points
     sqd = np.sqrt(((D - x[:,:ndata])**2).sum(axis=0))
     idx = np.argsort(sqd) # sorting
    # print(idx[:K])
     # return the indexes of K nearest neighbours
     return idx[:K]

def handle_objects(req):

    transformer = tf.TransformListener()

    # Iterating through the array from the segmentation noe and adding object with the respective center point coordinates and dimensions
    # The array is structurer as follows: [xDim, yDim, zDim, xCenter, yCenter, zCenter]

    objects = req.objects.data
    gaze = req.gaze_point
    print(objects)
    print("--------------")
    print(gaze)
    num_objects = int(len(objects)/6) # number of object

    world_objects = Float32MultiArray()
    world_objects.data = [0]*len(objects)
    center = np.empty([2, num_objects]) # array that will contain the x,y center of the objects

    i = 0
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
        p_tr = transformer.transformPoint("world", pointstamp)

        height_to_table = 0.185 - (p_tr.point.z - (objects[i+2]/2))

        # box_pose = geometry_msgs.msg.PoseStamped()
        # box_pose.header.frame_id = "world"
        # box_pose.pose.position.x = p_tr.point.x
        # box_pose.pose.position.y = p_tr.point.y
        # box_pose.pose.position.z = p_tr.point.z  # - height_to_table

        box_z = 0.185 + (objects[i+2]/2)
        box_z = box_z + (p_tr.point.z - box_z)/2
        # world_objects[i+2] = objects[i+2] + (p_tr.point.z - box_pose.pose.position.z)

        # world_objects.insert(i, objects[i])
        # world_objects.insert(i+1, objects[i+1])
        # world_objects.insert(i+2, objects[i+2] + (p_tr.point.z - box_z))
        # world_objects.insert(i+3, p_tr.point.x)
        # world_objects.insert(i+4, p_tr.point.y)
        # world_objects.insert(i+5, box_z)

        world_objects.data[i] = objects[i]
        world_objects.data[i+1] = objects[i+1]
        world_objects.data[i+2] = objects[i+2] + + (p_tr.point.z - box_z)
        world_objects.data[i+3] = p_tr.point.x
        world_objects.data[i+4] = p_tr.point.y
        world_objects.data[i+5] = box_z
        # print(world_objects)
        object_id = str(i/6)

        print("x,y,z = ")
        print(world_objects.data[i+3])
        print(world_objects.data[i+4])
        print(world_objects.data[i+5])
        print("------")

        i+=6

        # target_obj = ['0', 0.11087217926979065, 0.3403069078922272, 0.11774600305213856, 0.48992229410969163, 0.49934569425808273, 0.27480948858513754]


    fig,ax = plt.subplots(1)
    j = 0
    while j < len(objects):
    #    center[0].append(objects[i+3])
    #    center[1].append(objects[i+4])
    #    np.append(center,[objects[i+3]])
        iteration = int(j/6)
        center[0,iteration] = world_objects.data[j+3]
        center[1,iteration] = world_objects.data[j+4]
        # rect = patches.Rectangle((center[1,iteration],center[0,iteration]),world_objects[j],world_objects[j+1],linewidth=1,edgecolor='r',facecolor='none')
        # ax.add_patch(rect)
        print(center)
        j+=6

    print(center)

    x = np.array([[.45],[0.5]])
    neig_idx = knn_search(x,center,1)
    print(neig_idx)
    print("target object xyz:")
    print(world_objects.data[neig_idx*6 + 3])
    print(world_objects.data[neig_idx*6 + 4])
    print(world_objects.data[neig_idx*6 + 5])
    offset_x = 0.015
    offset_y = 0.035

    grasp_point = geometry_msgs.msg.Point()
    grasp_point.x = world_objects.data[neig_idx*6 + 3] - world_objects.data[neig_idx*6]/2 - offset_x
    grasp_point.y = world_objects.data[neig_idx*6 + 4] - world_objects.data[neig_idx*6 + 1]/2 - offset_y
    grasp_point.z = world_objects.data[neig_idx*6 + 5]

    rect = patches.Rectangle((0.5,0.2),0.15,0.1,linewidth=1,edgecolor='r',facecolor='none')
    ax.add_patch(rect)

    # highlighting the neighbours
    pylab.plot(center[1,neig_idx],center[0,neig_idx],'o', markerfacecolor='None',markersize=15,markeredgewidth=1)
    # plotting the data and the input point
    # pylab.plot(center[1,:],center[0,:],'ob',x[1,0],x[0,0],'or')

    plt.axis([-0.2,1.7, 0.0, 0.75])
    ax.plot(center[1,:],center[0,:],'ob',x[1,0],x[0,0],'or')
    ax.plot(center[1,neig_idx],center[0,neig_idx],'o', markerfacecolor='None',markersize=15,markeredgewidth=1)

    plt.gca().invert_xaxis()
    # plt.show()
    # rospy.sleep(1.0)
    # plt.close('all')
    # pylab.close(fig)
    index = int(neig_idx)
    print(index)

    # b = Float32MultiArray()
    # b.data = [0]*len(objects)
    # print(b)

    srv_response = gazeOptimiserResponse()
    srv_response.sorted_objects = world_objects
    srv_response.target_id = index
    srv_response.grasp_point = grasp_point
    return srv_response

if __name__ == '__main__':
    rospy.init_node('gaze_optimiser', anonymous=True)
    # req =  [0.09238377213478088, 0.07335389405488968, 0.1415911614894867, 0.43495261669158936, -0.11511331796646118, -0.0012646578252315521, 0.08798286318778992, 0.07133589684963226, 0.18319405615329742, 0.5219529271125793, 0.02710883319377899, 0.08386678248643875, 0.0326995849609375, 0.06856178492307663, 0.05430290102958679, 0.45429980754852295, 0.14136792719364166, -0.02174977958202362]

    gaze_optimiser_server()

    # handle_objects(req)
    rospy.spin()