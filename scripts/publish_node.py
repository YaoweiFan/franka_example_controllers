#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import tf.transformations
import numpy as np

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import WrenchStamped
from franka_msgs.msg import FrankaState
from runner.rnn_runner import Runner

import actionlib
from franka_gripper.msg import GraspAction, GraspGoal, GraspEpsilon, MoveAction, MoveGoal

from prettytable import PrettyTable

# import actionlib_tutorials.msg
# actionlib_tutorials.msg.FibonacciAction
# goal = actionlib_tutorials.msg.FibonacciGoal(order=20)

panda1_pose_msg = PoseStamped()
panda2_pose_msg = PoseStamped()

# publisher
panda1_pose_pub = None
panda2_pose_pub = None

equilibrium_pose_msg_set = False

# 状态，包括末端位置和姿态
panda1_pose = {'position': np.zeros(3), 'orientation': np.zeros(4)}
panda2_pose = {'position': np.zeros(3), 'orientation': np.zeros(4)}

panda1_eef_pos = np.zeros(3)
panda1_eef_quat = np.zeros(4)
left_hole_pos = np.zeros(3)

panda2_eef_pos = np.zeros(3)
panda2_eef_quat = np.zeros(4)
right_hole_pos = np.zeros(3)

peg_pos = np.zeros(3)

# [[min_x, max_x], [min_y, max_y], [min_z, max_z]]
position_limits = [[-0.6, 0.8], [-0.6, 0.6], [0, 0.9]]

# 设置平衡位置消息并发送
def set_pub_msgs(link_name_1, link_name_2):
    panda1_pose_msg.pose.position.x = max([min([panda1_pose["position"][0], position_limits[0][1]]), position_limits[0][0]])
    panda1_pose_msg.pose.position.y = max([min([panda1_pose["position"][1], position_limits[1][1]]), position_limits[1][0]])
    panda1_pose_msg.pose.position.z = max([min([panda1_pose["position"][2], position_limits[2][1]]), position_limits[2][0]])
    panda1_pose_msg.pose.orientation.x = panda1_pose["orientation"][0]
    panda1_pose_msg.pose.orientation.y = panda1_pose["orientation"][1]
    panda1_pose_msg.pose.orientation.z = panda1_pose["orientation"][2]
    panda1_pose_msg.pose.orientation.w = panda1_pose["orientation"][3]

    panda2_pose_msg.pose.position.x = max([min([panda2_pose["position"][0], position_limits[0][1]]), position_limits[0][0]])
    panda2_pose_msg.pose.position.y = max([min([panda2_pose["position"][1], position_limits[1][1]]), position_limits[1][0]])
    panda2_pose_msg.pose.position.z = max([min([panda2_pose["position"][2], position_limits[2][1]]), position_limits[2][0]])
    panda2_pose_msg.pose.orientation.x = panda2_pose["orientation"][0]
    panda2_pose_msg.pose.orientation.y = panda2_pose["orientation"][1]
    panda2_pose_msg.pose.orientation.z = panda2_pose["orientation"][2]
    panda2_pose_msg.pose.orientation.w = panda2_pose["orientation"][3]

    panda1_pose_msg.header.frame_id = link_name_1
    panda1_pose_msg.header.stamp = rospy.Time(0)
    panda1_pose_pub.publish(panda1_pose_msg)
    panda2_pose_msg.header.frame_id = link_name_2
    panda2_pose_msg.header.stamp = rospy.Time(0)
    panda2_pose_pub.publish(panda2_pose_msg)

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

# get panda2's obs
def panda2_franka_callback(msg):
    panda2_eef_pos[0] = msg.O_T_EE[12]
    panda2_eef_pos[1] = msg.O_T_EE[13]
    panda2_eef_pos[2] = msg.O_T_EE[14]

    quaternion = tf.transformations.quaternion_from_matrix(np.transpose(np.reshape(msg.O_T_EE, (4, 4))))
    quaternion = quaternion / np.linalg.norm(quaternion) # x,y,z,w
    panda2_eef_quat[0] = quaternion[0]
    panda2_eef_quat[1] = quaternion[1]
    panda2_eef_quat[2] = quaternion[2]
    panda2_eef_quat[3] = quaternion[3]


if __name__ == "__main__":
    # print("Equilibrium pose publisher is starting ...")
    rospy.init_node("equilibrium_pose_node")

    panda1_franka_sub = rospy.Subscriber("panda_1/franka_state_controller/franka_states", FrankaState, panda1_franka_callback)
    panda2_franka_sub = rospy.Subscriber("panda_2/franka_state_controller/franka_states", FrankaState, panda2_franka_callback)

    listener = tf.TransformListener()
    link_name_1 = rospy.get_param("~link_name_1")
    link_name_2 = rospy.get_param("~link_name_2")

    print("Waiting for getting initial pose ...")

    # Get initial pose
    while np.sum(panda1_eef_pos) == 0 or np.sum(panda2_eef_pos) == 0:
        rospy.sleep(1)

    print("Initial pose Got!")

    panda1_pose["position"] = panda1_eef_pos.copy()
    panda1_pose["orientation"] = panda1_eef_quat.copy()
    panda2_pose["position"] = panda2_eef_pos.copy()
    panda2_pose["orientation"] = panda2_eef_quat.copy()

    panda1_pose_pub = rospy.Publisher("panda_1_equilibrium_pose", PoseStamped, queue_size=10)
    panda2_pose_pub = rospy.Publisher("panda_2_equilibrium_pose", PoseStamped, queue_size=10)

    runner = Runner()

    print("============ Press `Enter` to open both grippers ...")
    input()

    # panda1 gripper open hand
    panda1_move_client = actionlib.SimpleActionClient("/panda_1/franka_gripper/move", MoveAction)
    panda1_move_client.wait_for_server()
    print("panda1 start opening ...")
    panda1_move_goal = MoveGoal(width=0.05, speed=0.03)
    panda1_move_client.send_goal(panda1_move_goal)
    panda1_move_client.wait_for_result()
    print("panda1's hand opened!")

    # panda2 gripper open hand
    panda2_move_client = actionlib.SimpleActionClient("/panda_2/franka_gripper/move", MoveAction)
    panda2_move_client.wait_for_server()
    print("panda2 start opening ...")
    panda2_move_goal = MoveGoal(width=0.05, speed=0.03)
    panda2_move_client.send_goal(panda2_move_goal)
    panda2_move_client.wait_for_result()
    print("panda2's hand opened!")

    print("============ Press `Enter` to close gripper1 ...")
    input()

    # panda1 gripper grasp obj
    panda1_grasp_client = actionlib.SimpleActionClient("/panda_1/franka_gripper/grasp", GraspAction)
    panda1_grasp_client.wait_for_server()
    print("panda1 start grasping ...")
    epsilon = GraspEpsilon(inner=0.005 ,outer=0.005)
    panda1_grasp_goal = GraspGoal(width=0.025, epsilon=epsilon, speed=0.01, force=1)
    panda1_grasp_client.send_goal(panda1_grasp_goal)
    panda1_grasp_client.wait_for_result()
    print("panda1 grasped object!")

    print("============ Press `Enter` to close gripper2 ...")
    input()

    # panda2 gripper grasp obj
    panda2_grasp_client = actionlib.SimpleActionClient("/panda_2/franka_gripper/grasp", GraspAction)
    panda2_grasp_client.wait_for_server()
    print("panda2 start grasping ...")
    epsilon = GraspEpsilon(inner=0.005 ,outer=0.005)
    panda2_grasp_goal = GraspGoal(width=0.025, epsilon=epsilon, speed=0.01, force=1)
    panda2_grasp_client.send_goal(panda2_grasp_goal)
    panda2_grasp_client.wait_for_result()
    print("panda2 grasped object!")

    print("============ Press `Enter` to start policy ...")
    input()

    panda1_obs = np.zeros(19)
    panda2_obs = np.zeros(19)
    while not rospy.is_shutdown():
        # [-0.2, -0.25, 1.05]  = [0.35926132 0.22942945 0.03901864] + []
        # print(panda1_eef_pos)
        # print(panda2_eef_pos)
        panda1_obs[0:3] = panda1_eef_pos + np.array([-0.55926132, -0.47942945, 1.01098136])
        panda1_obs[3:7] = panda1_eef_quat
        panda1_obs[7:13] = np.array([0,0,0,0,0,0])
        # peg 的位置在 eef 之下 0.05m
        panda1_obs[13:16] = panda1_obs[0:3] + np.array([0, 0, -0.05])
        # left hole 的位置是固定的
        panda1_obs[16:19] = panda1_obs[13:16] - np.array([0, -0.25, 1.02])
        # print("------------------------------------------------------------")
        # print("panda1_obs:")
        # print("panda1_eef_pos:", panda1_obs[0:3])
        # print("panda1_eef_quat:", panda1_obs[3:7])
        # print("panda1_ft:", panda1_obs[7:13])
        # print("panda1_peg_pos:", panda1_obs[13:16])
        # print("panda1_left_hole_pos:", panda1_obs[16:19])

        # [-0.2, 0.25, 1.05]  = [ 0.35748203 -0.23044253  0.03546662] + []
        panda2_obs[0:3] = panda2_eef_pos + np.array([-0.55748203, 0.48044253, 1.01453338])
        panda2_obs[3:7] = panda2_eef_quat
        panda2_obs[7:13] = np.array([0,0,0,0,0,0])
        # peg 的位置在 eef 之下 0.05m
        panda2_obs[13:16] = panda2_obs[0:3] + np.array([0, 0, -0.05])
        # right hole 的位置是固定的
        panda2_obs[16:19] = panda2_obs[13:16] - np.array([0, 0.25, 1.02])
        # print("panda2_obs:")
        # print("panda2_eef_pos:", panda2_obs[0:3])
        # print("panda2_eef_quat:", panda2_obs[3:7])
        # print("panda2_ft:", panda2_obs[7:13])
        # print("panda2_peg_pos:", panda2_obs[13:16])
        # print("panda2_right_hole_pos:", panda2_obs[16:19])
        # print("------------------------------------------------------------")

        pos_table = PrettyTable(['Px','Py','Pz','Qx','Qy','Qz'])
        pos_table.add_row([panda1_obs[0], panda1_obs[1], panda1_obs[2], panda2_obs[0], panda2_obs[1], panda2_obs[2]])
        print(pos_table)
        quat_table = PrettyTable(['Peef_quatx','Peef_quaty','Peef_quatz','Peef_quatw','Qeef_quatx','Qeef_quaty','Qeef_quatz','Qeef_quatw'])
        quat_table.add_row([panda1_obs[3], panda1_obs[4], panda1_obs[5], panda1_obs[6], panda2_obs[3], panda2_obs[4], panda2_obs[5], panda2_obs[6]])
        print(quat_table)
        peg_table = PrettyTable(['Ppegx','Ppegy','Ppegz','Qpegx','Qpegy','Qpegz'])
        peg_table.add_row([panda1_obs[13], panda1_obs[14], panda1_obs[15], panda2_obs[13], panda2_obs[14], panda2_obs[15]])
        print(peg_table)
        pth_table = PrettyTable(['Ppthx','Ppthy','Ppthz','Qpthx','Qpthy','Qpthz'])
        pth_table.add_row([panda1_obs[16], panda1_obs[17], panda1_obs[18], panda2_obs[16], panda2_obs[17], panda2_obs[18]])
        print(pth_table)

        obs = np.array([panda1_obs, panda2_obs])
        avail_actions = np.array([[1,1,1,1,1,1,1], [1,1,1,1,1,1,1]])
        # obs -> action
        delta_pos = runner.step(obs, avail_actions)
        panda1_pose["position"] += delta_pos[0]
        panda2_pose["position"] += delta_pos[1]

        pub_table = PrettyTable(['Ppubx','Ppuby','Ppubz','Qpubx','Qpuby','Qpubz'])
        pub_table.add_row([panda1_pose["position"][0], panda1_pose["position"][1], panda1_pose["position"][2], panda2_pose["position"][0], panda2_pose["position"][1], panda2_pose["position"][2]])
        print(pub_table)

        # panda1_pose["position"] -= np.array([0, 0, 0.002])
        # panda2_pose["position"] -= np.array([0, 0, 0.002])

        print("============ Press `Enter` to execute a step ...")
        input()

        # update new equilibrium pose, delta_pos: 2*3
        set_pub_msgs(link_name_1, link_name_2)

        # rospy.sleep(1)

