#!/usr/bin/env python


import tf
import rospy
import geometry_msgs.msg

import numpy as np
import math
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import PointCloud2
from segmentation.srv import*


import pylab
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
import matplotlib.patches as patches
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib import colors




def gaze_optimiser_server():
    s = rospy.Service('gaze_optimiser_service', gazeOptimiser, handle_objects)
    print "Ready to receive objects."
    rospy.spin()


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


def astar(maze, start, end):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
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

def circular_ik (grasp_point):
    e_co = geometry_msgs.msg.Point()
    e_co.z = grasp_point.z

    sh_co = geometry_msgs.msg.Point()
    sh_co.x = -0.15
    sh_co.y = 0.38
    sh_co.z = 0.54
    sh_length = 0.36
    e_length = 0.32

def angle_ik(grasp_point):
    e_co = geometry_msgs.msg.Point()
    e_co.z = grasp_point.z

    sh_co = geometry_msgs.msg.Point()
    sh_co.x = -0.15
    sh_co.y = 0.38
    sh_co.z = 0.54
    u_length = 0.36
    f_length = 0.3

    wrist_0_x = sh_co.x + f_length
    elbow_angle = math.degrees(math.tan((grasp_point.y - sh_co.y)/(grasp_point.x - sh_co.x)))
    e_angle = math.tan((grasp_point.y - sh_co.y)/(grasp_point.x - sh_co.x))
    print("Elbow angle: %s" %elbow_angle)

    e_co.x = grasp_point.x - f_length*math.cos(e_angle)
    e_co.y = grasp_point.y - f_length*math.sin(e_angle)

    return (e_co, sh_co)

def ik_calculator(grasp_point):
    e_co = geometry_msgs.msg.Point()
    e_co.z = grasp_point.z
    sh_co = geometry_msgs.msg.Point()
    sh_co.x = -0.15
    sh_co.y = 0.38
    sh_co.z = 0.54
    sh_length = 0.40
    e_length = 0.40

    # Euclidean distance from shoulder joint to wrist joint
    dist_ws = math.sqrt(pow(sh_co.x - grasp_point.x,2) + pow(sh_co.y - grasp_point.y,2) + pow(sh_co.z - grasp_point.z,2))
    print("Euclidean distance: %s" %dist_ws)

    gamma_rad = math.acos((dist_ws**2-sh_length**2-e_length**2)/(2*sh_length*e_length))
    gamma = math.degrees(gamma_rad)
    print("Gamma: %s" %gamma)
    print("Grasp point before %s" %grasp_point)
    # grasp_x = grasp_point.x - sh_co.x
    # grasp_y = grasp_point.y - sh_co.y
    grasp_point.x = grasp_point.x - sh_co.x
    grasp_point.y = grasp_point.y - sh_co.y
    # grasp_point.z =
    print("Grasp point after %s" %grasp_point)

    # Calculate upper arm projection
    sh_top = math.sqrt(pow(sh_length,2)-pow((sh_co.z - e_co.z),2))
    print("Sh_top %s" %sh_top)
    # Calculate angle of forearm
    print((pow(grasp_point.x,2) + pow(grasp_point.y,2) - pow(sh_top,2) - pow(e_length,2))/(2*sh_top*e_length))
    qe_rad = math.acos((pow(grasp_point.x,2) + pow(grasp_point.y,2) - pow(sh_top,2) - pow(e_length,2))/(2*sh_top*e_length))
    qe = math.degrees(qe_rad)
    print(qe)

    # Calculate upper arm angle
    alpha = math.atan(grasp_point.x/grasp_point.y)
    beta = math.acos((pow(sh_top,2) - pow(e_length,2) + pow(grasp_point.x,2) + pow(grasp_point.y,2))/(2*sh_top*math.sqrt(pow(grasp_point.x,2) + pow(grasp_point.y,2))))
    qs_rad = alpha - beta
    qs = math.degrees(qs_rad)
    print(qs)

    # Obtain elbow joint position
    e_co.x = math.cos(qs)*sh_top + sh_co.x
    e_co.y = math.sin(qs)*sh_top + sh_co.y
    print("Elbow: %s" %e_co)

    grasp_point.x = grasp_point.x + sh_co.x
    grasp_point.y = grasp_point.y + sh_co.y

    return (e_co, sh_co)

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

def object_collision(e_co, grasp_point, objects_array):
    i = 0
    num_collisions = 0
    while i < len(objects_array):
        bottom = line_collision(e_co.x, e_co.y, grasp_point.x, grasp_point.y,objects_array[i+3]-objects_array[i+0]/2,objects_array[i+4]-objects_array[i+1]/2,objects_array[i+3]-objects_array[i+0]/2, objects_array[i+4]+objects_array[i+1]/2)
        top = line_collision(e_co.x, e_co.y, grasp_point.x, grasp_point.y,objects_array[i+3]+objects_array[i+0]/2,objects_array[i+4]-objects_array[i+1]/2,objects_array[i+3]+objects_array[i+0]/2, objects_array[i+4]+objects_array[i+1]/2)
        left = line_collision(e_co.x, e_co.y, grasp_point.x, grasp_point.y,objects_array[i+3]-objects_array[i+0]/2,objects_array[i+4]+objects_array[i+1]/2,objects_array[i+3]+objects_array[i+0]/2, objects_array[i+4]+objects_array[i+1]/2)
        right = line_collision(e_co.x, e_co.y, grasp_point.x, grasp_point.y,objects_array[i+3]-objects_array[i+0]/2,objects_array[i+4]-objects_array[i+1]/2,objects_array[i+3]+objects_array[i+0]/2, objects_array[i+4]-objects_array[i+1]/2)
        if(bottom == True or top == True or left == True or right == True):
            num_collisions+=1
        i+=6

    print("number of collisions in state: %s" %num_collisions)
    if(num_collisions > 0):
        return True # There is at least one collision
    else:
        return False # There are no collisions

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
    # fig.set_size_inches(18.5, 10.5)
    # pylab.rcParams['figure.figsize'] = 5, 10
    # plt.subplots_adjust(left=0.1, right=0.9, bottom=0.1, top=0.9)

    j = 0
    while j < len(objects):
    #    center[0].append(objects[i+3])
    #    center[1].append(objects[i+4])
    #    np.append(center,[objects[i+3]])
        iteration = int(j/6)
        center[0,iteration] = world_objects.data[j+3]
        center[1,iteration] = world_objects.data[j+4]
        rect = patches.Rectangle((center[1,iteration]-world_objects.data[j+1]/2,center[0,iteration]-world_objects.data[j]/2),world_objects.data[j+1],world_objects.data[j],linewidth=1,edgecolor='r',facecolor='none')
        ax.add_patch(rect)
        # print(center)
        j+=6

    print(center)

    x = np.array([[.45],[0.4]])
    neig_idx = knn_search(x,center,1)
    print("Index %s" %neig_idx)
    print("target object xyz:")
    print(world_objects.data[neig_idx*6 + 3])
    print(world_objects.data[neig_idx*6 + 4])
    print(world_objects.data[neig_idx*6 + 5])
    offset_x = 0.015
    offset_y = 0.015

    grasp_point = geometry_msgs.msg.Point()
    grasp_point.x = world_objects.data[neig_idx*6 + 3] - world_objects.data[neig_idx*6]/2 - offset_x
    grasp_point.y = world_objects.data[neig_idx*6 + 4] - world_objects.data[neig_idx*6 + 1]/2 - offset_y
    grasp_point.z = world_objects.data[neig_idx*6 + 5]

    co_e, co_s = ik_calculator(grasp_point)
    print("Coord %s" %co_e)

    # rect = patches.Rectangle((0.5,0.2),0.15,0.1,linewidth=1,edgecolor='r',facecolor='none')
    # ax.add_patch(rect)

    # highlighting the neighbours
    pylab.plot(center[1,neig_idx],center[0,neig_idx],'o', markerfacecolor='None',markersize=15,markeredgewidth=1)
    # plotting the data and the input point
    # pylab.plot(center[1,:],center[0,:],'ob',x[1,0],x[0,0],'or')

    plt.axis([-0.2,1.7, -0.2, 0.75])
    ax.plot(center[1,:],center[0,:],'ob',x[1,0],x[0,0],'or',grasp_point.y,grasp_point.x,'og')
    ax.plot(center[1,neig_idx],center[0,neig_idx],'o', markerfacecolor='None',markersize=15,markeredgewidth=1)
    plt.plot([co_s.y,co_e.y,grasp_point.y],[co_s.x,co_e.x,grasp_point.x])
    # ax.plot(x, x + 4, linestyle='-')
    plt.gca().invert_xaxis()
    plt.show()
    # rospy.sleep(15.0)
    plt.close('all')
    index = int(neig_idx)
    print("Index %s" %index)

    # b = Float32MultiArray()
    # b.data = [0]*len(objects)
    # print(b)

    srv_response = gazeOptimiserResponse()
    srv_response.sorted_objects = world_objects
    srv_response.target_id = index
    srv_response.grasp_point = grasp_point
    return srv_response

def sim(req):
    transformer = tf.TransformListener()

    # Iterating through the array from the segmentation noe and adding object with the respective center point coordinates and dimensions
    # The array is structurer as follows: [xDim, yDim, zDim, xCenter, yCenter, zCenter]

    objects = req
    # gaze = req.gaze_point
    print(objects)
    print("--------------")
    # print(gaze)
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
#    plt.figure(figsize=(10,10/(1.7/0.7)))
    # fig3 = plt.figure(2)
    # ax3 = fig3.gca(projection='3d')
    # fig.set_size_inches(18.5, 10.5)
    # pylab.rcParams['figure.figsize'] = 5, 10
    # plt.subplots_adjust(left=0.1, right=0.9, bottom=0.1, top=0.9)

    j = 0
    while j < len(objects):
    #    center[0].append(objects[i+3])
    #    center[1].append(objects[i+4])
    #    np.append(center,[objects[i+3]])
        iteration = int(j/6)
        center[0,iteration] = world_objects.data[j+3]
        center[1,iteration] = world_objects.data[j+4]
        rect = patches.Rectangle((center[1,iteration]-world_objects.data[j+1]/2,center[0,iteration]-world_objects.data[j]/2),world_objects.data[j+1],world_objects.data[j],linewidth=1,edgecolor='r',facecolor='none')
        ax.add_patch(rect)
        # print(center)
        j+=6

    print(center)
    ##########~~~~~~~~~~~~~~~~~~~~~
    # Gaze point xy
    x = np.array([[.39],[0.84]])
    neig_idx = knn_search(x,center,1)
    print("Index %s" %neig_idx)
    print("target object xyz:")
    print(world_objects.data[neig_idx*6 + 3])
    print(world_objects.data[neig_idx*6 + 4])
    print(world_objects.data[neig_idx*6 + 5])
    offset_x = 0.015
    offset_y = 0.015

    grasp_point = geometry_msgs.msg.Point()
    grasp_point.x = world_objects.data[neig_idx*6 + 3] - world_objects.data[neig_idx*6]/2 - offset_x
    grasp_point.y = world_objects.data[neig_idx*6 + 4] - world_objects.data[neig_idx*6 + 1]/2 - offset_y
    grasp_point.z = world_objects.data[neig_idx*6 + 5]
    objects_array = world_objects.data
    # co_e, co_s = ik_calculator(grasp_point)
    co_e, co_s = angle_ik(grasp_point)
    #print("Elbow point: %s" %co_e)
    #print("Grasp point: %s" %grasp_point)
    # rect = patches.Rectangle((0.5,0.2),0.15,0.1,linewidth=1,edgecolor='r',facecolor='none')
    # ax.add_patch(rect)
    collision = object_collision(co_e,grasp_point, objects_array)
    #~~~~~~~~~~~~~~~~~~~~

    ratio = 1.7/0.7
    rows = 80
    cols = int(rows/ratio)
    data = [[0 for _ in range(rows)] for _ in range(cols)]
    data[20][26]=1
    data[20][25]=1
    data[19][28]=1
    Yresolution = 1.7/rows
    Xresolution = 0.7/cols
    # target=[0.22,0.534]
    # target_index = [round(target[0]/Xresolution),round(target[1]/Yresolution)]
    target_x = world_objects.data[neig_idx*6 + 3] - world_objects.data[neig_idx*6]/2 - offset_x
    target_y = world_objects.data[neig_idx*6 + 4] - world_objects.data[neig_idx*6 + 1]/2 - offset_y
    target_index = [round(target_x/Xresolution)-1,round(target_y/Yresolution)-1]
    start = (8, 20)
    end = (int(target_index[0]), int(target_index[1]))
    for i in range(rows):
        for j in range(cols):
            #target = [Xresolution*(j+1),Yresolution*(i+1)]
            grasp_point.x = Xresolution*(j+1)
            grasp_point.y = Yresolution*(i+1)
            co_e, co_s = angle_ik(grasp_point)
            collision_state = object_collision(co_e,grasp_point, objects_array)
            if(collision_state == True):
                data[j][i] = 1

    if data[end[0]][end[1]] != 1:
        path = astar(data, start, end)
        print(path)
        for j in range(len(path)):
                r = path[j][0]
                c = path[j][1]
#                print(r,c)
                data[r][c]= 4
    data[end[0]][end[1]]=2
    data[start[0]][start[1]]=3
    data = np.array(data)
    data.shape
    fig2 = plt.figure(figsize=(10,10/ratio))
    cmap = colors.ListedColormap(['white','red','green','#0060ff','#aaaaaa'])
    plt.figure(figsize=(10,10/ratio))
    plt.pcolor(data[::1],cmap=cmap,edgecolors='k', linewidths=1)
    fig2.gca().invert_xaxis()
    fig2.show()
    # rospy.sleep(10.0)
    # plt.close('all')





    # plotting the data and the input point
    # pylab.plot(center[1,:],center[0,:],'ob',x[1,0],x[0,0],'or')
    # ax3.scatter([0], [0], [0], color="g", s=100)
    # plt.plot([co_s.y,co_e.y,grasp_point.y],[co_s.x,co_e.x,grasp_point.x],[co_s.z,co_e.z,grasp_point.z])

    ax.axis([-0.2,1.7, -0.2, 0.75])
    # ax.axis([-0.2,1.7, -0.2, 0.75])
    # Plot center of objects, grasp point and elbow joint
    # highlighting the target object
    ax.plot(center[1,neig_idx],center[0,neig_idx],'o', markerfacecolor='None',markersize=15,markeredgewidth=1)
    grasp_point.x = world_objects.data[neig_idx*6 + 3] - world_objects.data[neig_idx*6]/2 - offset_x
    grasp_point.y = world_objects.data[neig_idx*6 + 4] - world_objects.data[neig_idx*6 + 1]/2 - offset_y
    co_e, co_s = angle_ik(grasp_point)
    ax.plot(center[1,:],center[0,:],'ob',x[1,0],x[0,0],'or',grasp_point.y,grasp_point.x,'og',co_e.y,co_e.x,'ok')
    # ax.plot(0.525,0.380,'oy')
    # Mark target object
    ax.plot(center[1,neig_idx],center[0,neig_idx],'o', markerfacecolor='None',markersize=15,markeredgewidth=1)

    ax.plot([co_s.y,co_e.y,grasp_point.y],[co_s.x,co_e.x,grasp_point.x])
    plt.gca().invert_xaxis()

    # draw sphere
    # u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
    # x = np.cos(u)*np.sin(v)
    # y = np.sin(u)*np.sin(v)
    # z = np.cos(v)
    # ax3.plot_wireframe(x, y, z, color="r")

    # plt.gca().invert_xaxis()
    # plt.figure(size=(5,6))
    plt.show(block=True)
    # plt.show()
    rospy.sleep(10.0)
    plt.close('all')
    index = int(neig_idx)
    print("Index %s" %index)



    # b = Float32MultiArray()
    # b.data = [0]*len(objects)
    # print(b)

    # srv_response = gazeOptimiserResponse()
    # srv_response.sorted_objects = world_objects
    # srv_response.target_id = index
    # srv_response.grasp_point = grasp_point
    # return srv_response

if __name__ == '__main__':
    rospy.init_node('gaze_optimiser', anonymous=True)
    #req =  [0.09238377213478088, 0.07335389405488968, 0.1415911614894867, 0.43495261669158936, -0.11511331796646118, -0.0012646578252315521, 0.08798286318778992, 0.07133589684963226, 0.18319405615329742, 0.5219529271125793, 0.02710883319377899, 0.08386678248643875, 0.0326995849609375, 0.06856178492307663, 0.05430290102958679, 0.45429980754852295, 0.14136792719364166, -0.02174977958202362]

    # gaze_optimiser_server()
    req = [0.08420228958129883, 0.0866108238697052, 0.16340254247188568, 0.5440202951431274, 0.07170078903436661, 0.11486805230379105, 0.044422149658203125, 0.054751671850681305, 0.11145603656768799, 0.4410666823387146, -0.08078508079051971, 0.02292603999376297, 0.03234878182411194, 0.03743894398212433, 0.046912916004657745, 0.45620059967041016, 0.2094809114933014, 0.030203916132450104]
    # req = [0.10357585549354553, 0.07956766337156296, 0.1900981366634369, 0.3480975031852722, 0.038999784737825394, 0.025573041290044785, 0.09558302164077759, 0.06678294390439987, 0.1394098997116089, 0.3654365837574005, -0.11884936690330505, -0.04283341020345688, 0.02408367395401001, 0.07848946005105972, 0.06253930926322937, 0.5783331394195557, -0.037476059049367905, 0.04785224795341492]
    sim(req)
    # handle_objects(req)
    rospy.spin()



# (0.08420228958129883, 0.0866108238697052, 0.16340254247188568, 0.5440202951431274, 0.07170078903436661, 0.11486805230379105, 0.044422149658203125, 0.054751671850681305, 0.11145603656768799, 0.4410666823387146, -0.08078508079051971, 0.02292603999376297, 0.03234878182411194, 0.03743894398212433, 0.046912916004657745, 0.45620059967041016, 0.2094809114933014, 0.030203916132450104)
# (0.10357585549354553, 0.07956766337156296, 0.1900981366634369, 0.3480975031852722, 0.038999784737825394, 0.025573041290044785, 0.09558302164077759, 0.06678294390439987, 0.1394098997116089, 0.3654365837574005, -0.11884936690330505, -0.04283341020345688, 0.02408367395401001, 0.07848946005105972, 0.06253930926322937, 0.5783331394195557, -0.037476059049367905, 0.04785224795341492)
