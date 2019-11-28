#!/usr/bin/env python
import sys
import copy
import rospkg
from time import sleep
from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import *
import rospy, tf

rospy.init_node("spawn_products")
rospy.wait_for_service("gazebo/delete_model")
rospy.wait_for_service("gazebo/spawn_urdf_model")

rospack = rospkg.RosPack()
part_pkg = rospack.get_path('cai_env')

delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
spawn_model = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)

with open(part_pkg+'/urdf/'+'chair_pin.urdf', "r") as f:
    product_xml = f.read()

# get Quaternion Coordinate
quat=tf.transformations.quaternion_from_euler(0,0,0)
orient = Quaternion(quat[0],quat[1],quat[2],quat[3])
print(orient)

# Delete Part in gazebo
for num in range(0,12):
    item_name = "product_{0}_0".format(num)
    print("Deleting model:%s", item_name)
    delete_model(item_name)

# Spawn Part in gazebi
for num in range(0,999):
    item_name   =   "product_{0}_0".format(num)
    print("Spawning model:%s", item_name)
    item_pose   =   Pose(Point(x=0, y=0, z=1), orient)
    spawn_model(item_name, product_xml, "", item_pose, "world")
    