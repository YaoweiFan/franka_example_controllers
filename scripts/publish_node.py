#!/usr/bin/env python
#-*- coding: utf-8 -*-
import rospy
import tf.transformations
import numpy as np
from prettytable import PrettyTable
import os
import pandas as pd

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import WrenchStamped
from franka_msgs.msg import FrankaState
import actionlib
from franka_gripper.msg import GraspAction, GraspGoal, GraspEpsilon, MoveAction, MoveGoal

from runner.rnn_runner import Runner
from utils.transfer import transfer_to_sim_frame, transfer_to_real_frame

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
panda1_pose = {'position': None, 'orientation': None}
panda2_pose = {'position': None, 'orientation': None}
panda1_pose_command = {'position': None, 'orientation': None}
panda2_pose_command = {'position': None, 'orientation': None}

panda1_eef_pos = np.zeros(3)
panda1_eef_quat = np.zeros(4)
left_hole_pos = np.zeros(3)

panda2_eef_pos = np.zeros(3)
panda2_eef_quat = np.zeros(4)
right_hole_pos = np.zeros(3)

peg_pos = np.zeros(3)



# 设置平衡位置消息并发送
def set_pub_msgs(link_name_1, link_name_2):
    panda1_pose_msg.pose.position.x = panda1_pose_command["position"][0]
    panda1_pose_msg.pose.position.y = panda1_pose_command["position"][1]
    panda1_pose_msg.pose.position.z = panda1_pose_command["position"][2]
    panda1_pose_msg.pose.orientation.x = panda1_pose_command["orientation"][0]
    panda1_pose_msg.pose.orientation.y = panda1_pose_command["orientation"][1]
    panda1_pose_msg.pose.orientation.z = panda1_pose_command["orientation"][2]
    panda1_pose_msg.pose.orientation.w = panda1_pose_command["orientation"][3]

    panda2_pose_msg.pose.position.x = panda2_pose_command["position"][0]
    panda2_pose_msg.pose.position.y = panda2_pose_command["position"][1]
    panda2_pose_msg.pose.position.z = panda2_pose_command["position"][2]
    panda2_pose_msg.pose.orientation.x = panda2_pose_command["orientation"][0]
    panda2_pose_msg.pose.orientation.y = panda2_pose_command["orientation"][1]
    panda2_pose_msg.pose.orientation.z = panda2_pose_command["orientation"][2]
    panda2_pose_msg.pose.orientation.w = panda2_pose_command["orientation"][3]

    panda1_pose_msg.header.frame_id = link_name_1
    panda1_pose_msg.header.stamp = rospy.Time(0)
    panda1_pose_pub.publish(panda1_pose_msg)
    panda2_pose_msg.header.frame_id = link_name_2
    panda2_pose_msg.header.stamp = rospy.Time(0)
    panda2_pose_pub.publish(panda2_pose_msg)


# get panda1's obs
def panda1_franka_callback(msg):
    global panda1_eef_pos, panda1_eef_quat
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
    global panda2_eef_pos, panda2_eef_quat
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
    panda1_pose_command["position"] = panda1_eef_pos.copy()
    panda1_pose_command["orientation"] = panda1_eef_quat.copy()
    panda2_pose_command["position"] = panda2_eef_pos.copy()
    panda2_pose_command["orientation"] = panda2_eef_quat.copy()
    print(panda1_pose_command["position"])
    print(panda2_pose_command["position"])
    print("Initial pose Got!")

    panda1_pose_pub = rospy.Publisher("panda_1_equilibrium_pose", PoseStamped, queue_size=10)
    panda2_pose_pub = rospy.Publisher("panda_2_equilibrium_pose", PoseStamped, queue_size=10)

    runner = Runner()

    # print("============ Press `Enter` to open both grippers ...")
    # raw_input()

    # # panda1 gripper open hand
    # panda1_move_client = actionlib.SimpleActionClient("/panda_1/franka_gripper/move", MoveAction)
    # panda1_move_client.wait_for_server()
    # print("panda1 start opening ...")
    # panda1_move_goal = MoveGoal(width=0.05, speed=0.03)
    # panda1_move_client.send_goal(panda1_move_goal)
    # panda1_move_client.wait_for_result()
    # print("panda1's hand opened!")

    # # panda2 gripper open hand
    # panda2_move_client = actionlib.SimpleActionClient("/panda_2/franka_gripper/move", MoveAction)
    # panda2_move_client.wait_for_server()
    # print("panda2 start opening ...")
    # panda2_move_goal = MoveGoal(width=0.05, speed=0.03)
    # panda2_move_client.send_goal(panda2_move_goal)
    # panda2_move_client.wait_for_result()
    # print("panda2's hand opened!")

    print("============ Press `Enter` to close gripper1 ...")
    raw_input()

    # panda1 gripper grasp obj
    panda1_grasp_client = actionlib.SimpleActionClient("/panda_1/franka_gripper/grasp", GraspAction)
    panda1_grasp_client.wait_for_server()
    print("panda1 start grasping ...")
    epsilon = GraspEpsilon(inner=0.001 ,outer=0.001)
    panda1_grasp_goal = GraspGoal(width=-0.02, epsilon=epsilon, speed=0.01, force=5)
    # panda1_grasp_goal = MoveGoal(width=0.015, speed=0.01)
    panda1_grasp_client.send_goal(panda1_grasp_goal)
    panda1_grasp_client.wait_for_result()
    print("panda1 grasped object!")

    print("============ Press `Enter` to close gripper2 ...")
    raw_input()

    # panda2 gripper grasp obj
    panda2_grasp_client = actionlib.SimpleActionClient("/panda_2/franka_gripper/grasp", GraspAction)
    panda2_grasp_client.wait_for_server()
    print("panda2 start grasping ...")
    epsilon = GraspEpsilon(inner=0.001 ,outer=0.001)
    panda2_grasp_goal = GraspGoal(width=-0.02, epsilon=epsilon, speed=0.01, force=5)
    # panda2_grasp_goal = MoveGoal(width=0.015, speed=0.01)
    panda2_grasp_client.send_goal(panda2_grasp_goal)
    panda2_grasp_client.wait_for_result()
    print("panda2 grasped object!")

    # # hard assemble 
    # discrete_path = '/home/fyw/Documents/discrete/DualArmMimic/results/models/discrete/offpg_dualarm__2022-11-01_22-16-42'
    # dataframe = pd.read_csv(os.path.join(discrete_path, "fix_deterministic_state3/0.csv"))
    # scene = 'hard'

    # soft assemble 
    discrete_path = '/home/fyw/Documents/discrete/DualArmMimic/results/models/dualarmrod/change_environment_mimic_200/offpg_dualarmrod__2022-12-30_23-31-08'
    dataframe = pd.read_csv(os.path.join(discrete_path, "fix_deterministic_state_best/0.csv"))
    scene = 'soft'

    steps = 1

    print("============ Press `Enter` to start policy ...")
    raw_input()

    panda1_obs = np.zeros(13)
    panda2_obs = np.zeros(13)
    while not rospy.is_shutdown():
        # 记录当前机械臂末端位置姿态（sim 坐标系下）  
        panda1_pose["position"] = transfer_to_sim_frame(panda1_eef_pos.copy(), 'panda_1', scene)
        panda1_pose["orientation"] = panda1_eef_quat.copy()
        panda2_pose["position"] = transfer_to_sim_frame(panda2_eef_pos.copy(), 'panda_2', scene)
        panda2_pose["orientation"] = panda2_eef_quat.copy()

        # 装填 obs
        panda1_obs[0:3] = panda1_pose["position"]
        panda1_obs[3:7] = panda1_pose_command["orientation"]
        # peg 的位置在 eef 之下 0.05m
        left_peg_pos = panda1_pose["position"] + np.array([0, 0, -0.05])
        panda1_obs[7:10] = left_peg_pos
        panda1_obs[10:13] = left_peg_pos - np.array([0, -0.25, 1.02])

        panda2_obs[0:3] = panda2_pose["position"]
        panda2_obs[3:7] = panda2_pose_command["orientation"]
        # peg 的位置在 eef 之下 0.05m
        right_peg_pos = panda2_pose["position"] + np.array([0, 0, -0.05])
        panda2_obs[7:10] = right_peg_pos
        panda2_obs[10:13] = right_peg_pos - np.array([0, 0.25, 1.02])


        # 打印 obs  
        print("=========== Step " + str(steps) + " ===========")
        pos_table = PrettyTable(['Px','Py','Pz','Qx','Qy','Qz'])
        pos_table.add_row([panda1_obs[0], panda1_obs[1], panda1_obs[2], panda2_obs[0], panda2_obs[1], panda2_obs[2]])
        print(pos_table)
        quat_table = PrettyTable(['Peef_quatx','Peef_quaty','Peef_quatz','Peef_quatw','Qeef_quatx','Qeef_quaty','Qeef_quatz','Qeef_quatw'])
        quat_table.add_row([panda1_obs[3], panda1_obs[4], panda1_obs[5], panda1_obs[6], panda2_obs[3], panda2_obs[4], panda2_obs[5], panda2_obs[6]])
        print(quat_table)
        peg_table = PrettyTable(['Ppegx','Ppegy','Ppegz','Qpegx','Qpegy','Qpegz'])
        peg_table.add_row([panda1_obs[7], panda1_obs[8], panda1_obs[9], panda2_obs[7], panda2_obs[8], panda2_obs[9]])
        print(peg_table)
        pth_table = PrettyTable(['Ppthx','Ppthy','Ppthz','Qpthx','Qpthy','Qpthz'])
        pth_table.add_row([panda1_obs[10], panda1_obs[11], panda1_obs[12], panda2_obs[10], panda2_obs[11], panda2_obs[12]])
        print(pth_table)


        # obs -> action
        obs = np.array([panda1_obs, panda2_obs])
        avail_actions = np.array([[1,1,1,1,1,1,1], [1,1,1,1,1,1,1]])
        delta_pos = runner.step(obs, avail_actions)

        # command 是在 real 场景下的，需要转换到 sim 场景下
        panda1_pose_command["position"] = transfer_to_sim_frame(panda1_pose_command["position"], 'panda_1', scene)
        panda2_pose_command["position"] = transfer_to_sim_frame(panda2_pose_command["position"], 'panda_2', scene)
        # panda1_pose_command["position"] += delta_pos[0]
        # panda2_pose_command["position"] += delta_pos[1]

        # 直接表演轨迹
        panda1_pose_command["position"][0] = dataframe["left_x"][steps] - 0.008
        # panda1_pose_command["position"][1] = dataframe["left_y"][steps]
        panda1_pose_command["position"][2] = dataframe["left_z"][steps] + 0.008
        panda2_pose_command["position"][0] = dataframe["right_x"][steps] - 0.003
        # panda2_pose_command["position"][1] = dataframe["right_y"][steps]
        panda2_pose_command["position"][2] = dataframe["right_z"][steps] + 0.008

        # 转换回 real 场景下的真实命令  
        panda1_pose_command["position"] = transfer_to_real_frame(panda1_pose_command["position"], 'panda_1', scene)
        panda2_pose_command["position"] = transfer_to_real_frame(panda2_pose_command["position"], 'panda_2', scene)


        # 打印 action
        pub_table = PrettyTable(['Ppubx','Ppuby','Ppubz','Qpubx','Qpuby','Qpubz'])
        pub_table.add_row([panda1_pose_command["position"][0], panda1_pose_command["position"][1], panda1_pose_command["position"][2], 
                            panda1_pose_command["position"][0], panda1_pose_command["position"][1], panda1_pose_command["position"][2]])
        print(pub_table)


        # 执行
        print("============ Press `Enter` to execute a step ...")
        raw_input()
        set_pub_msgs(link_name_1, link_name_2)
        steps += 1
        # 手动确认执行完毕
        print("============ Press `Enter` to ensure last step was finished totally ...")
        raw_input()
        