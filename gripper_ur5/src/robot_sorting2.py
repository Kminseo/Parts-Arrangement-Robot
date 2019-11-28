#!/usr/bin/env python
import sys, random, copy
import rospy, tf, rospkg
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import *
from std_msgs.msg import *
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list




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

        for num in range(0,5):
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

def part_pose_collect():
    parts_pose=[]
    model_pose = rospy.wait_for_message("gazebo/model_states",ModelStates)
    for count in range(3):
        for num in range(5):
            product_num = model_pose.name.index("product_{0}_{1}".format(count,num))
            x = model_pose.pose[product_num].position.x
            y = model_pose.pose[product_num].position.y
            z = model_pose.pose[product_num].position.z
            X = model_pose.pose[product_num].orientation.x
            Y = model_pose.pose[product_num].orientation.y
            Z = model_pose.pose[product_num].orientation.z
            W = model_pose.pose[product_num].orientation.w
            euler=tf.transformations.euler_from_quaternion((X,Y,Z,W)) 
            parts_pose.append([x,y,z,euler[0],euler[1],euler[2]])     
    return parts_pose
    

def robot_move(x,y,z,Y):
    
    group1.clear_pose_targets()
    quat=tf.transformations.quaternion_from_euler(3.14,0,Y)
    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.x = x
    pose_target.position.y = y
    pose_target.position.z = z
    pose_target.orientation.x = quat[0]
    pose_target.orientation.y = quat[1]
    pose_target.orientation.z = quat[2]
    pose_target.orientation.w = quat[3]
    group1.set_pose_target(pose_target)

    plan1 = group1.plan()
    group1.execute(plan1, wait= True)

def gripper_control(close):
    
    
    group2.clear_pose_targets()
    group_variable_values = group2.get_current_joint_values()
    # close : 0.02, open : 0.00
    if close==True:
        group_variable_values[0] = 0.02
        group_variable_values[1] = 0.02
    else:
        group_variable_values[0] = 0.00
        group_variable_values[1] = 0.00

    group2.set_joint_value_target(group_variable_values)
    plan2 = group2.plan()
    group2.execute(plan2, wait= True)
    rospy.sleep(1)

def robot_move(x,y,z,Y):
    group1.clear_pose_targets()
    quat=tf.transformations.quaternion_from_euler(3.14,0,Y)
    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.x = x
    pose_target.position.y = y
    pose_target.position.z = z
    pose_target.orientation.x = quat[0]
    pose_target.orientation.y = quat[1]
    pose_target.orientation.z = quat[2]
    pose_target.orientation.w = quat[3]
    group1.set_pose_target(pose_target)

    plan1 = group1.plan()
    group1.execute(plan1, wait= True)

def robot_move_line(x1,y1,z1,x2,y2,z2):
    
    step = 5.0
    x_step_unit = (x2-x1)/step # target_pose - start pose / move_step
    y_step_unit= (y2-y1)/step # target_pose - start pose / move_step
    z_step_unit = (z2-z1)/step # target_pose - start pose / move_step

    waypoints = []
    robot_pose = geometry_msgs.msg.Pose()
    now_pose = group1.get_current_pose().pose

    # robot move following vector
    for i in range(1,int(step)):
        now_pose.position.x = x1 + x_step_unit*i
        now_pose.position.y = y1 + y_step_unit*i
        now_pose.position.z = z1 + z_step_unit*i
        waypoints.append(copy.deepcopy(now_pose))

    # last target pose add in memory
    robot_pose.position.x = x2
    robot_pose.position.y = y2
    robot_pose.position.z = z2
    robot_pose.orientation.x = now_pose.orientation.x
    robot_pose.orientation.y = now_pose.orientation.y
    robot_pose.orientation.z = now_pose.orientation.z
    robot_pose.orientation.w = now_pose.orientation.w
    waypoints.append(copy.deepcopy(robot_pose))

    plan, fraction = group1.compute_cartesian_path(waypoints,0.01,0.0) 
    group1.execute(plan,wait=True)

def goal_move_line(x,y,z): 
    step = 5.0
    waypoints = []
    robot_pose = geometry_msgs.msg.Pose()
    now_pose = group1.get_current_pose().pose
    x_step_unit = (x-now_pose.position.x)/step # target_pose - start pose / move_step
    y_step_unit= (y-now_pose.position.y)/step # target_pose - start pose / move_step
    z_step_unit = (z-now_pose.position.z)/step # target_pose - start pose / move_step
    robot_pose.orientation.x = now_pose.orientation.x
    robot_pose.orientation.y = now_pose.orientation.y
    robot_pose.orientation.z = now_pose.orientation.z
    robot_pose.orientation.w = now_pose.orientation.w

    # robot move following vector
    for i in range(1,int(step)):
        robot_pose.position.x = now_pose.position.x + x_step_unit*i
        robot_pose.position.y = now_pose.position.y + y_step_unit*i
        robot_pose.position.z = now_pose.position.z + z_step_unit*i
        waypoints.append(copy.deepcopy(robot_pose))

    # last target pose add in memory
    robot_pose.position.x = x
    robot_pose.position.y = y
    robot_pose.position.z = z
    waypoints.append(copy.deepcopy(robot_pose))

    plan, fraction = group1.compute_cartesian_path(waypoints,0.01,0.0) 
    group1.execute(plan,wait=True)

def gripper_move_to_object(x1,y1,Degree):

    waypoints = []
    robot_pose = geometry_msgs.msg.Pose()
    now_pose = group1.get_current_pose().pose
    robot_pose.position.x = x1
    robot_pose.position.y = y1
    robot_pose.position.z = 1.3
    robot_pose.orientation.x = now_pose.orientation.x
    robot_pose.orientation.y = now_pose.orientation.y
    robot_pose.orientation.z = now_pose.orientation.z 

    
    robot_pose.orientation.w = now_pose.orientation.w
    waypoints.append(copy.deepcopy(robot_pose)) # above Object
    quat=tf.transformations.quaternion_from_euler(3.14,0,Degree)
    robot_pose.orientation.x = quat[0]
    robot_pose.orientation.y = quat[1]
    robot_pose.orientation.z = quat[2]
    robot_pose.orientation.w = quat[3]
    waypoints.append(copy.deepcopy(robot_pose)) # roate Object Direction
    robot_pose.position.z = 0.815
    waypoints.append(copy.deepcopy(robot_pose))
    plan, fraction = group1.compute_cartesian_path(waypoints,0.001,0.0) 
    group1.execute(plan,wait=True) 

def attach_link(model1, link1, model2, link2):
    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach',Attach)
    attach_srv.wait_for_service()
    req = AttachRequest()
    req.model_name_1 = model1
    req.link_name_1 = link1
    req.model_name_2 = model2
    req.link_name_2 = link2

    attach_srv.call(req)

def detach_link(model1, link1, model2, link2):
    detach_srv = rospy.ServiceProxy('/link_attacher_node/detach',Attach)
    detach_srv.wait_for_service()
    req = AttachRequest()
    req.model_name_1 = model1
    req.link_name_1 = link1
    req.model_name_2 = model2
    req.link_name_2 = link2

    detach_srv.call(req)


rospy.init_node("sort_robot_program")
product_spawn()
parts_pose = part_pose_collect()

moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
group_name2 = "gripper"
group2 = moveit_commander.MoveGroupCommander(group_name2)

group_name1 = "arm"
group1 = moveit_commander.MoveGroupCommander(group_name1)
group1.set_planner_id("RRTConnectkConfigDefault")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)
scene = moveit_commander.PlanningSceneInterface()
rospy.sleep(2)
box_pose = geometry_msgs.msg.PoseStamped()
scene.remove_world_object("box1")
scene.remove_world_object("box2")
scene.remove_world_object("box3")
box_pose.header.frame_id = robot.get_planning_frame()
box_pose.pose.position.x = -0.55
box_pose.pose.position.y = 0.45
box_pose.pose.position.z = 1
box_pose.pose.orientation.w = 1.0
scene.add_box("box1", box_pose, size=(0.3, 0.4, 0.28))
box_pose.pose.position.x = -0.55
box_pose.pose.position.y = -0.45
box_pose.pose.position.z = 1
scene.add_box("box2", box_pose, size=(0.3, 0.4, 0.28))
box_pose.pose.position.x = -0.275
box_pose.pose.position.y = 0
box_pose.pose.position.z = 0.95
scene.add_box("box3", box_pose, size=(0.05, 0.4, 0.2))
rospy.sleep(2)

group1.set_named_target("Home")
plan1 = group1.plan()
group1.execute(plan1,wait=True)


model_num = 0
for count in range(3):
    for num in range(5):
        parts_pose = part_pose_collect()
        if (abs(parts_pose[model_num][0])>30.0 and  abs(parts_pose[model_num][1])>30.0):
            continue

        if(model_num >= 0 and model_num <5):
            basket_pose=[-0.55,0.60,1.3]
            direc = parts_pose[model_num][5]
        elif(model_num >= 5 and model_num <10):
            basket_pose=[-0.55,0.40,1.3]
            direc = parts_pose[model_num][5]
        else:
            basket_pose=[-0.55,-0.60,1.3]
            direc = parts_pose[model_num][4]
        
        goal_move_line(parts_pose[model_num][0],parts_pose[model_num][1],1.3)
        robot_move_line(parts_pose[model_num][0], parts_pose[model_num][1], 1.3, parts_pose[model_num][0], parts_pose[model_num][1], 0.88)
        gripper_control(True)
        attach_link('robot','wrist_3_link',"product_{0}_{1}".format(count,num),"link")
        rospy.sleep(1)
        robot_move_line(parts_pose[model_num][0], parts_pose[model_num][1], 0.88, parts_pose[model_num][0], parts_pose[model_num][1], 1.3)
        robot_move_line(parts_pose[model_num][0], parts_pose[model_num][1], 1.3, basket_pose[0], basket_pose[1], 1.3)
        detach_link('robot','wrist_3_link',"product_{0}_{1}".format(count,num),"link")
        gripper_control(False)
        
        rospy.sleep(1)
        model_num += 1


    

    
