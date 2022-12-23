#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import tf.transformations
import numpy as np

from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import FrankaState


panda_pose_msg = PoseStamped()


# 状态，包括末端位置和姿态
panda_pose = {'position': None, 'orientation': None}

panda_eef_pos = np.zeros(3)
panda_eef_quat = np.zeros(4)


# get panda's obs
def panda_franka_callback(msg):
    panda_eef_pos[0] = msg.O_T_EE[12]
    panda_eef_pos[1] = msg.O_T_EE[13]
    panda_eef_pos[2] = msg.O_T_EE[14]

    quaternion = tf.transformations.quaternion_from_matrix(np.transpose(np.reshape(msg.O_T_EE, (4, 4))))
    quaternion = quaternion / np.linalg.norm(quaternion) # x,y,z,w
    panda_eef_quat[0] = quaternion[0]
    panda_eef_quat[1] = quaternion[1]
    panda_eef_quat[2] = quaternion[2]
    panda_eef_quat[3] = quaternion[3]


if __name__ == "__main__":
    # print("Equilibrium pose publisher is starting ...")
    rospy.init_node("equilibrium_pose_node")

    panda_franka_sub = rospy.Subscriber("franka_state_controller/franka_states", FrankaState, panda_franka_callback)

    listener = tf.TransformListener()
    link_name = rospy.get_param("~link_name")

    print("Waiting for getting initial pose ...")

    # Get initial pose
    while np.sum(panda_eef_pos) == 0:
        rospy.sleep(1)

    panda_pose["position"] = panda_eef_pos.copy()
    panda_pose["orientation"] = panda_eef_quat.copy()

    print("Initial pose Got!")

    print("panda:")
    print("position:")
    print(panda_pose["position"])
    print("orientation (quat -- x,y,z,w):")
    print(panda_pose["orientation"])
