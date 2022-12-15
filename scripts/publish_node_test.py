#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import tf.transformations
import numpy as np

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import WrenchStamped
from franka_msgs.msg import FrankaState
from nav_msgs.msg import Odometry

import actionlib
from franka_gripper.msg import GraspAction, GraspGoal, GraspEpsilon


# import actionlib_tutorials.msg
# actionlib_tutorials.msg.FibonacciAction
# goal = actionlib_tutorials.msg.FibonacciGoal(order=20)

panda1_pose_msg = PoseStamped()

# publisher
panda1_pose_pub = None

# 状态，包括末端位置和姿态
panda1_pos = np.zeros(3)
panda1_ori = np.zeros(4)

panda1_eef_pos = np.zeros(3)
panda1_eef_quat = np.zeros(4)

# [[min_x, max_x], [min_y, max_y], [min_z, max_z]]
position_limits = [[-0.6, 0.6], [-0.6, 0.6], [0.05, 0.9]]

# 设置平衡位置消息并发送
def set_pub_msgs(link_name_1):
    panda1_pose_msg.pose.position.x = max([min([panda1_pos[0], position_limits[0][1]]), position_limits[0][0]])
    panda1_pose_msg.pose.position.y = max([min([panda1_pos[1], position_limits[1][1]]), position_limits[1][0]])
    panda1_pose_msg.pose.position.z = max([min([panda1_pos[2], position_limits[2][1]]), position_limits[2][0]])
    panda1_pose_msg.pose.orientation.x = panda1_ori[0]
    panda1_pose_msg.pose.orientation.y = panda1_ori[1]
    panda1_pose_msg.pose.orientation.z = panda1_ori[2]
    panda1_pose_msg.pose.orientation.w = panda1_ori[3]

    panda1_pose_msg.header.frame_id = link_name_1
    panda1_pose_msg.header.stamp = rospy.Time(0)
    panda1_pose_pub.publish(panda1_pose_msg)


def panda1_franka_callback(msg):
    panda1_eef_pos[0] = msg.O_T_EE[12]
    panda1_eef_pos[1] = msg.O_T_EE[13]
    panda1_eef_pos[2] = msg.O_T_EE[14]

    quaternion = tf.transformations.quaternion_from_matrix(np.transpose(np.reshape(msg.O_T_EE, (4, 4))))
    quaternion = quaternion / np.linalg.norm(quaternion) # x,y,z,w
    panda1_eef_quat[0] = quaternion[0]
    panda1_eef_quat[1] = quaternion[1]
    panda1_eef_quat[2] = quaternion[2]
    panda1_eef_quat[3] = quaternion[3]


if __name__ == "__main__":
    rospy.init_node("equilibrium_pose_node")

    panda1_franka_sub = rospy.Subscriber("franka_state_controller/franka_states", FrankaState, panda1_franka_callback)

    listener = tf.TransformListener()
    link_name_1 = rospy.get_param("~link_name_1")

    # Get initial pose for the interactive marker
    while np.sum(panda1_eef_pos) == 0:
        rospy.sleep(1)

    panda1_pos = panda1_eef_pos.copy()
    panda1_ori = panda1_eef_quat.copy()

    panda1_pose_pub = rospy.Publisher("panda_1_equilibrium_pose", PoseStamped, queue_size=10)

    # panda1 gripper client
    # panda1_gripper_client = actionlib.SimpleActionClient("/panda_1/franka_gripper/grasp", GraspAction)
    # panda1_gripper_client.wait_for_server()
    # epsilon = GraspEpsilon(inner=0.01 ,outer=0.01)
    # panda1_gripper_goal = GraspGoal(width=0.02, epsilon=epsilon, speed=0.01, force=10)
    # panda1_gripper_client.send_goal(panda1_gripper_goal)
    # print("panda1 gripper client send message!")
    # panda1_gripper_client.wait_for_result()
    # print("panda1 gripper client get results!")

    
    while not rospy.is_shutdown():
        panda1_pos += np.array([0.0, 0.001, 0.0])
        
        # update new equilibrium pose, delta_pos: 2*3
        set_pub_msgs(link_name_1)

        print("go to pos:", panda1_pos)

        rospy.sleep(1)

