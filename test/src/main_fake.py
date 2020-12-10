#!/usr/bin/env python

#python script which spawns block in gazebo
import rospy, tf
from gazebo_msgs.srv import DeleteModel, SpawnModel
import geometry_msgs.msg

if __name__ == '__main__':
    print("Waiting for gazebo services...")
    rospy.init_node("spawn_products_in_bins")
    rospy.wait_for_service("/gazebo/delete_model")
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    print(".................")
    print("All services stand by.")
    print(".................")
    delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
    spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

    with open("/home/li/catkin_ws/src/kinova-ros/kinova_gazebo/worlds/blocks.sdf", "r") as f:
        product_xml = f.read()
    
    poss = geometry_msgs.msg.Point()
    orii = geometry_msgs.msg.Quaternion()
    item_pose = geometry_msgs.msg.Pose()
    poss.x = -2.0
    poss.y = -2.0
    poss.z = 2
    orii.x = 0.0
    orii.y = 0.0
    orii.z = 0.0
    orii.w = 0.0
    item_pose.position =  poss
    item_pose.orientation = orii
    
    
    spawn_model("new", product_xml,"", item_pose,"world")
    
    poss.x = -2.5
    item_pose.position = poss
    spawn_model("new2",product_xml,"",item_pose,"world")
    delete_model("red1")
