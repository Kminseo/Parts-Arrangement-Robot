#!/usr/bin/env python
import sys, random, copy
import rospy, tf, rospkg
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import Image
from yolov3_pytorch_ros.msg import BoundingBox, BoundingBoxes

import numpy as np
import math as m

import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list

def get_model_pose():
    model=[]
    real_model = []
    model_pose = rospy.wait_for_message("/detected_objects_in_image",BoundingBoxes)
    gazebo_model=get_gazebo_model_pose()
    
    for i in range(len(model_pose.bounding_boxes)):
        # print(model_pose.bounding_boxes[i].Class)
        model.append([model_pose.bounding_boxes[i].Class,model_pose.bounding_boxes[i].cx,model_pose.bounding_boxes[i].cy,model_pose.bounding_boxes[i].degree ])
      
    for i in range(len(model_pose.bounding_boxes)):
        coord = trans_coord(model[i][1],model[i][2])
        real_model.append([model[i][0],coord[0],coord[1],model[i][3]])

    find_model=matching_model2(real_model, gazebo_model)
    
    print(real_model)
    print(find_model)

def get_gazebo_model_pose():
    parts_pose=[]
    model_pose = rospy.wait_for_message("gazebo/model_states",ModelStates)
    for count in range(len(model_pose.name)):
        name = model_pose.name[count]
        x = model_pose.pose[count].position.x
        y = model_pose.pose[count].position.y
        parts_pose.append([name, x, y])
    
    return parts_pose


def matching_model( real_model, gazebo_model):
    matched_model=[]
    for find_count in range(len(real_model)):
        for x_count in range(len(gazebo_model)):
            if(x_count == 0):
                x_min = abs(gazebo_model[x_count][1]-real_model[find_count][1])
            if(x_min >= abs(gazebo_model[x_count][1]-real_model[find_count][1])):
                x_min = abs(gazebo_model[x_count][1]-real_model[find_count][1])
                index_x = x_count
        
        for y_count in range(len(gazebo_model)):
            if(y_count == 0):
                y_min = abs(gazebo_model[y_count][2]-real_model[find_count][2])
            if(y_min >= abs(gazebo_model[y_count][2]-real_model[find_count][2])):
                index_y = y_count
                y_min = abs(gazebo_model[y_count][2]-real_model[find_count][2])
    
        if(index_x == index_y):
            matched_model.append([gazebo_model[index_y][0],gazebo_model[index_y][1],gazebo_model[index_y][2],real_model[find_count][3]]) 
            # if you want realmodel pose, you have to change gazebo_mode->real_model[find_count] 
            
    return matched_model

def matching_model2( real_model, gazebo_model):
    matched_model=[]
    for find_count in range(len(real_model)):
        
        for gazebo_count in range(len(gazebo_model)):
            a = (gazebo_model[gazebo_count][1]-real_model[find_count][1])**2
            b = (gazebo_model[gazebo_count][2]-real_model[find_count][2])**2
            dist = (a+b)**0.5
            if(gazebo_count == 0):
                dist_min = dist
            
            if(dist_min >= dist):
                dist_min = dist
                min_index = gazebo_count

        if(gazebo_model[min_index][0][:7]!="product"):
                continue

        matched_model.append([gazebo_model[min_index][0],gazebo_model[min_index][1],gazebo_model[min_index][2],real_model[find_count][3]]) 
        # if you want realmodel pose, you have to change gazebo_mode->real_model[find_count] 
            
    return matched_model
 
def trans_coord(x,y):
    x = x-640.0
    y = y-480.0
    x_offset = 0.0
    y_offset = 0.0
    z_offset = 1.9
    r_orient = 180.0
    p_orient = 0.0
    y_orient = 0.0

    t= np.array([[1, 0, 0, x_offset],
				[0, 1, 0, y_offset],
				[0, 0, 1, z_offset],
				[0, 0, 0, 1]])
    ty = np.array([[1, 0, 0, 0],
				    [0, m.cos(m.radians(r_orient)), -m.sin(m.radians(r_orient)), 0],
				    [0, m.sin(m.radians(r_orient)), m.cos(m.radians(r_orient)), 0],
					[0, 0, 0, 1]])
    t12 = np.matmul(t,ty)
    coor = np.array([[x], [y], [0], [1]])
    result = np.matmul(t12,coor)
    x = result[1][0]/614.215 # tranform imagecoord x -> world coord
    y = result[0][0]/-686.822 # transform imagecoord y -> real world coord 
    return x, y


def product_spawn():
    # This function spawn three types parts(screw1, screw2, woodbolt) in gazebo
    rospy.wait_for_service("gazebo/spawn_urdf_model")
    spawn_model = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)

    rospack = rospkg.RosPack()
    part_pkg = rospack.get_path('cai_env')

    for count in range(3): # three types products spawn in gazebo

        if(count == 0):
            with open(part_pkg+'/urdf/'+'chair_pin.urdf', "r") as wood1:
                product_xml = wood1.read()
                wood1.close()

        elif(count == 1):
            with open(part_pkg+'/urdf/'+'screw1.urdf', "r") as screw1:
                product_xml = screw1.read()
                screw1.close()
        else:
            with open(part_pkg+'/urdf/'+'screw2.urdf', "r") as screw2:
                product_xml = screw2.read()
                screw2.close()

        for num in range(3):
            x_rand = random.randrange(-20,20)*0.01
            y_rand = random.randrange(-20,20)*0.01
            R_rand = random.randrange(-314,314)*0.01
            P_rand = random.randrange(-314,314)*0.01
            Y_rand = random.randrange(-314,314)*0.01
            quat=tf.transformations.quaternion_from_euler(R_rand,P_rand,Y_rand) 
            orient = Quaternion(quat[0],quat[1],quat[2],quat[3])
            item_name   =   "product_{0}_{1}".format(count,num)
            print("Spawning model:%s", item_name)
            item_pose   =   Pose(Point(x=x_rand, y=y_rand, z=1), orient)
            spawn_model(item_name, product_xml, "", item_pose, "world")
            rospy.sleep(0.5)

rospy.init_node("sort_robot_program")
product_spawn()
get_model_pose()
get_gazebo_model_pose()

