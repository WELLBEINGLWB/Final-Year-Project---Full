#!/usr/bin/env python
import tf
import rospy
import geometry_msgs.msg
import numpy as np
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import PointCloud2
from segmentation import*

def handle_objects(req):


    # print(msg.data)
    ## Adding Objects to the Planning Scene
    # objects = msg.data
    world_objects = []
    # print(objects)

    # Number of objects in the array (each has 6 dimensions)
    num_objects = len(objects)/6

    # Extra margin to add to the bounding boxes and to find gaze point within one object(in meters)
    margin = 0.0

    transformer = tf.TransformListener()

    # Iterating through the array from the segmentation noe and adding object with the respective center point coordinates and dimensions
    # The array is structurer as follows: [xDim, yDim, zDim, xCenter, yCenter, zCenter]

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

        # print("Before: ")
        # print(pointstamp.point.x)
        # print(pointstamp.point.y)
        # print(pointstamp.point.z)
        # transformer.waitForTransform("camera_link", "world", rospy.Time(0),rospy.Duration(4.0))

        # Converting the center point if the object to /world frame
        p_tr = transformer.transformPoint("world", pointstamp)

        # world_objects[i+3] = p_tr.point.x
        # world_objects[i+4] = p_tr.point.y
        # world_objects[i+5] = p_tr.point.z
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
        # world_objects[i+2] = objects[i+2] + (p_tr.point.z - box_pose.pose.position.z)

        world_objects.insert(i, objects[i])
        world_objects.insert(i+1, objects[i+1])
        world_objects.insert(i+2, objects[i+2] + (p_tr.point.z - box_pose.pose.position.z))
        world_objects.insert(i+3, p_tr.point.x)
        world_objects.insert(i+4, p_tr.point.y)
        world_objects.insert(i+5, box_pose.pose.position.z)
        # print(world_objects)
        object_id = str(i/6)
        # self.objectAdder.addBox(object_id, world_objects[i] + margin, world_objects[i+1] + margin, world_objects[i+2] , world_objects[i+3], world_objects[i+4], world_objects[i+5])
        # self.objectAdder.addBox(object_id, world_objects[i] + margin, world_objects[i+1] + margin, world_objects[i+2], world_objects[i+3], world_objects[i+4], world_objects[i+5])
        # scene.add_box(object_id, box_pose, size=(objects[i] + margin, objects[i+1] + margin, objects[i+2] + (p_tr.point.z - box_pose.pose.position.z)))
        # self.objectAdder.setColor(object_id, 0.1, 0.4, 1.0, a=0.9)
        # self.objectAdder.sendColors()

        ## Gaze integration
        gaze_margin = 0.05
        # self.gaze_point = geometry_msgs.msg.Point()
        # self.gaze_point.x = 0.4
        # self.gaze_point.y = 0.6
        # self.gaze_point.z = 0.2
        print("x,y,z = ")
        print(world_objects[i+3])
        print(world_objects[i+4])
        print(world_objects[i+5])
        print("------")

        offset_x = 0.015
        offset_y = 0.035
        # if world_objects[i]:
        #   resp.target_id = i
        #     grasp_point.x = world_objects[i+3] - world_objects[i]/2 - offset_x
        #     grasp_point.y = world_objects[i+4] - world_objects[i+1]/2 - offset_y
        #     grasp_point.z = world_objects[i+5]


        #Adding the objects to the world
        ## If the gaze point is within the object, that is the target object
        # if (((world_objects[i+3] - (world_objects[i]/2) - gaze_margin) <= self.gaze_point.x <= (world_objects[i+3] + (world_objects[i]/2) + gaze_margin))
        #       and ((world_objects[i+4] - (world_objects[i+1]/2) - gaze_margin) <= self.gaze_point.y <= (world_objects[i+4] + (world_objects[i+1]/2) + gaze_margin))
        #       and ((world_objects[i+5] - (world_objects[i+2]/2) - gaze_margin) <= self.gaze_point.z <= (world_objects[i+5] + (world_objects[i+2]/2) + gaze_margin))):
        #
        #         # self.target_obj_name = object_id
        #         self.target_obj = []
        #         self.target_obj.insert(0, world_objects[i])
        #         self.target_obj.insert(1, world_objects[i+1])
        #         self.target_obj.insert(2, world_objects[i+2])
        #         self.target_obj.insert(3, world_objects[i+3])
        #         self.target_obj.insert(4, world_objects[i+4])
        #         self.target_obj.insert(5, world_objects[i+5])
        #         self.target_obj.insert(6, object_id)
        #         print(self.target_obj)
        #         # targe_obj is set as such: [xDimension, yDimension, zDimension, xCenter, yCenter, zCenter, objectID]
        #
        #         # self.target_obj_x = world_objects[i+3]
        #         # self.target_obj_y = world_objects[i+4]
        #         # self.target_obj_z = world_objects[i+5]
        #         self.objectAdder.addBox(object_id, world_objects[i] + margin, world_objects[i+1] + margin, world_objects[i+2], world_objects[i+3], world_objects[i+4], world_objects[i+5])
        #         ## remove target object from world_objects???
        #         self.objectAdder.setColor(object_id, 1.0, 0.2, 0.2, a=1.0)
        #         ## Target object will have a different colour
        #
        # else:
        #         self.objectAdder.addBox(object_id, world_objects[i] + margin, world_objects[i+1] + margin, world_objects[i+2], world_objects[i+3], world_objects[i+4], world_objects[i+5])
        #         self.objectAdder.setColor(object_id, 0.1, 0.4, 1.0, a=1.0)
        #         # All obstacles have the same colour
        # # All the colours are set at the same time
        # self.objectAdder.sendColors()
        i+=6

        # target_obj = ['0', 0.11087217926979065, 0.3403069078922272, 0.11774600305213856, 0.48992229410969163, 0.49934569425808273, 0.27480948858513754]



if __name__ == '__main__':


    rospy.spin()
