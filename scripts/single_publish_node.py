#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import tf.transformations
import numpy as np

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import WrenchStamped
from franka_msgs.msg import FrankaState
from nav_msgs.msg import Odometry
# from runner import Runner

import actionlib
from franka_gripper.msg import GraspAction, GraspGoal, GraspEpsilon, MoveAction, MoveGoal


# import actionlib_tutorials.msg
# actionlib_tutorials.msg.FibonacciAction
# goal = actionlib_tutorials.msg.FibonacciGoal(order=20)

panda1_pose_msg = PoseStamped()

# publisher
panda1_pose_pub = None

equilibrium_pose_msg_set = False

# 状态，包括末端位置和姿态
panda1_pose = {'position': np.zeros(3), 'orientation': np.zeros(4)}

panda1_eef_pos = np.zeros(3)
panda1_eef_quat = np.zeros(4)

# [[min_x, max_x], [min_y, max_y], [min_z, max_z]]
position_limits = [[-0.6, 0.6], [-0.6, 0.6], [0.05, 0.9]]

# 设置平衡位置消息并发送
def set_pub_msgs(link_name_1):
    panda1_pose_msg.pose.position.x = max([min([panda1_pose["position"][0], position_limits[0][1]]), position_limits[0][0]])
    panda1_pose_msg.pose.position.y = max([min([panda1_pose["position"][1], position_limits[1][1]]), position_limits[1][0]])
    panda1_pose_msg.pose.position.z = max([min([panda1_pose["position"][2], position_limits[2][1]]), position_limits[2][0]])
    panda1_pose_msg.pose.orientation.x = panda1_pose["orientation"][0]
    panda1_pose_msg.pose.orientation.y = panda1_pose["orientation"][1]
    panda1_pose_msg.pose.orientation.z = panda1_pose["orientation"][2]
    panda1_pose_msg.pose.orientation.w = panda1_pose["orientation"][3]

    panda1_pose_msg.header.frame_id = link_name_1
    panda1_pose_msg.header.stamp = rospy.Time(0)
    panda1_pose_pub.publish(panda1_pose_msg)

# get panda1's obs
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

    panda1_franka_sub = rospy.Subscriber("panda_1/franka_state_controller/franka_states", FrankaState, panda1_franka_callback)

    listener = tf.TransformListener()
    link_name_1 = rospy.get_param("~link_name_1")

    # Get initial pose for the interactive marker
    while np.sum(panda1_eef_pos) == 0:
        rospy.sleep(1)

    panda1_pose["position"] = panda1_eef_pos.copy()
    panda1_pose["orientation"] = panda1_eef_quat.copy()

    panda1_pose_pub = rospy.Publisher("panda_1_equilibrium_pose", PoseStamped, queue_size=10)

    panda1_obs = np.zeros(19)
    while not rospy.is_shutdown():

        print("-------------------------------------------------------")
        print("panda1_obs:")
        print(panda1_obs[0:3])
        print(panda1_obs[3:7])
        print(panda1_obs[7:13])
        print(panda1_obs[13:16])
        print(panda1_obs[16:19])

        print("panda1_command:")
        print(panda1_pose["position"])
        print(panda1_pose["orientation"])
        print("-------------------------------------------------------")

        # obs -> action
        # delta_pos = runner.step(obs, avail_actions)
        # panda1_pose["position"] += delta_pos[0]
        # panda2_pose["position"] += delta_pos[1]

        # update new equilibrium pose, delta_pos: 2*3
        # set_pub_msgs(link_name_1)

        rospy.sleep(3)

