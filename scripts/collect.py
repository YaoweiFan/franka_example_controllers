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

# panda1_pose_msg = PoseStamped()

# publisher
# panda1_pose_pub = None

# 状态，包括末端位置和姿态
# panda1_pos = np.zeros(3)
# panda1_ori = np.zeros(4)

# panda1_eef_pos = np.zeros(3)
# panda1_eef_quat = np.zeros(4)

# [[min_x, max_x], [min_y, max_y], [min_z, max_z]]
# position_limits = [[-0.6, 0.6], [-0.6, 0.6], [0.05, 0.9]]

# 设置平衡位置消息并发送
# def set_pub_msgs(link_name_1):
#     panda1_pose_msg.pose.position.x = max([min([panda1_pos[0], position_limits[0][1]]), position_limits[0][0]])
#     panda1_pose_msg.pose.position.y = max([min([panda1_pos[1], position_limits[1][1]]), position_limits[1][0]])
#     panda1_pose_msg.pose.position.z = max([min([panda1_pos[2], position_limits[2][1]]), position_limits[2][0]])
#     panda1_pose_msg.pose.orientation.x = panda1_ori[0]
#     panda1_pose_msg.pose.orientation.y = panda1_ori[1]
#     panda1_pose_msg.pose.orientation.z = panda1_ori[2]
#     panda1_pose_msg.pose.orientation.w = panda1_ori[3]

#     panda1_pose_msg.header.frame_id = link_name_1
#     panda1_pose_msg.header.stamp = rospy.Time(0)
#     panda1_pose_pub.publish(panda1_pose_msg)

transform = np.zeros((4, 4))
ft = np.zeros(6)

def tansform_callback(msg):
    global transform
    transform = np.transpose(np.reshape(msg.O_T_EE, (4, 4)))

def ft_callback(msg):
    global ft
    ft[0] = 0.9 * ft[0] + 0.1 * msg.wrench.force.x
    ft[1] = 0.9 * ft[1] + 0.1 * msg.wrench.force.y
    ft[2] = 0.9 * ft[2] + 0.1 * msg.wrench.force.z
    ft[3] = 0.9 * ft[3] + 0.1 * msg.wrench.torque.x
    ft[4] = 0.9 * ft[4] + 0.1 * msg.wrench.torque.y
    ft[5] = 0.9 * ft[5] + 0.1 * msg.wrench.torque.z

if __name__ == "__main__":
    rospy.init_node("collect")

    tansform_sub = rospy.Subscriber("franka_state_controller/franka_states", FrankaState, tansform_callback)
    ft_sub = rospy.Subscriber("/ft_sensor/data", WrenchStamped, ft_callback)

    # listener = tf.TransformListener()
    # link_name_1 = rospy.get_param("~link_name_1")

    # Get initial pose for the interactive marker
    while np.linalg.norm(transform) == 0 or np.linalg.norm(ft) == 0:
        rospy.sleep(1)

    # panda1_pos = panda1_eef_pos.copy()
    # panda1_ori = panda1_eef_quat.copy()

    # panda1_pose_pub = rospy.Publisher("panda_1_equilibrium_pose", PoseStamped, queue_size=10)

    # panda1 gripper client
    # panda1_gripper_client = actionlib.SimpleActionClient("/panda_1/franka_gripper/grasp", GraspAction)
    # panda1_gripper_client.wait_for_server()
    # epsilon = GraspEpsilon(inner=0.01 ,outer=0.01)
    # panda1_gripper_goal = GraspGoal(width=0.02, epsilon=epsilon, speed=0.01, force=10)
    # panda1_gripper_client.send_goal(panda1_gripper_goal)
    # print("panda1 gripper client send message!")
    # panda1_gripper_client.wait_for_result()
    # print("panda1 gripper client get results!")

    transform_list = []
    ft_list = []

    while not rospy.is_shutdown():

        print "============ Press `Enter` to record data ", len(ft_list), " :"
        raw_input()
        # 等待获取稳定的 transform 和 ft 数据
        rospy.sleep(1)
        transform_list.append(transform.copy())
        ft_list.append(ft.copy())

        if(len(ft_list) == 20): break

    print "============ Collect finished, start writing to file ..."

    with open("data/collect/record_panda_2.txt", 'w') as f:
        for i in range(20):
            for j in range(4):
                for k in range(4):
                    f.write(str(transform_list[i][j][k]))
                    f.write(" ")
                f.write("\r\n")
            for j in range(6):
                f.write(str(ft_list[i][j]))
                f.write(" ")
            f.write("\r\n")

    print "============ File saved!"

