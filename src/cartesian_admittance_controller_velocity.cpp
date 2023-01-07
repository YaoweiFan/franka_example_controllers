// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/cartesian_admittance_controller_velocity.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <controller_interface/controller_base.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include "franka_example_controllers/info.h"

namespace franka_example_controllers {

bool CartesianAdmittanceControllerVelocity::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  sub_equilibrium_pose_ = node_handle.subscribe(
    "equilibrium_pose", 20, &CartesianAdmittanceControllerVelocity::equilibriumPoseCallback, this,
    ros::TransportHints().reliable().tcpNoDelay());

  sub_ft_ = node_handle.subscribe(
      "ft_sensor/data", 20, &CartesianAdmittanceControllerVelocity::ftCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("CartesianVelocityExampleController: Could not get parameter arm_id");
    return false;
  }

  velocity_cartesian_interface_ =
      robot_hardware->get<franka_hw::FrankaVelocityCartesianInterface>();
  if (velocity_cartesian_interface_ == nullptr) {
    ROS_ERROR(
        "CartesianVelocityExampleController: Could not get Cartesian velocity interface from "
        "hardware");
    return false;
  }
  try {
    velocity_cartesian_handle_ = std::make_unique<franka_hw::FrankaCartesianVelocityHandle>(
        velocity_cartesian_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianVelocityExampleController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("CartesianVelocityExampleController: Could not get state interface from hardware");
    return false;
  }

  if (!node_handle.getParam("fz_bias", fz_bias)) {
    ROS_ERROR("CartesianVelocityExampleController: Could not get parameter fz_bias");
    return false;
  }
  // std::cout << fz_bias << std::endl;
  ee2ft_pos[0] = 0; ee2ft_pos[1] = 0;
  if (!node_handle.getParam("ee2ft_posz", ee2ft_pos[2])) {
    ROS_ERROR("CartesianVelocityExampleController: Could not get parameter ee2ft_posz");
    return false;
  }
  if (!node_handle.getParam("handcm2ft_posx", handcm2ft_pos[0])) {
    ROS_ERROR("CartesianVelocityExampleController: Could not get parameter handcm2ft_posx");
    return false;
  }
  if (!node_handle.getParam("handcm2ft_posy", handcm2ft_pos[1])) {
    ROS_ERROR("CartesianVelocityExampleController: Could not get parameter handcm2ft_posy");
    return false;
  }
  if (!node_handle.getParam("handcm2ft_posz", handcm2ft_pos[2])) {
    ROS_ERROR("CartesianVelocityExampleController: Could not get parameter handcm2ft_posz");
    return false;
  }

  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  cartesian_inertia_.topLeftCorner(3,3) = Eigen::MatrixXd::Identity(3,3);
  cartesian_inertia_.bottomRightCorner(3,3) = Eigen::MatrixXd::Identity(3,3);
  // cartesian_damping_.setZero();
  cartesian_damping_.topLeftCorner(3,3) = 2.0 * sqrt(1000) * Eigen::MatrixXd::Identity(3,3);
  cartesian_damping_.bottomRightCorner(3,3) = 2.0 * sqrt(10) * Eigen::MatrixXd::Identity(3,3);
  // cartesian_stiffness_.setZero();
  cartesian_stiffness_.topLeftCorner(3,3) = 300 * Eigen::MatrixXd::Identity(3,3);
  cartesian_stiffness_.bottomRightCorner(3,3) = 10 * Eigen::MatrixXd::Identity(3,3);

  twist_.setZero();

  ft_.setZero(); ft_[2] = fz_bias;
  ft_filtered_.setZero(); ft_filtered_[2] = fz_bias;
  ft_d_.setZero();

  force_gain_.topLeftCorner(3,3) = 0.1 * Eigen::MatrixXd::Identity(3,3);
  force_gain_.bottomRightCorner(3,3) = 0.1 * Eigen::MatrixXd::Identity(3,3);

  // 计算末端外力相关矩阵系数初始化
  ee2ft_ori << 0.7071, -0.7071, 0,
               0.7071,  0.7071, 0,
                    0,       0, 1;
  cee2ft_pos << 0, -ee2ft_pos[2], ee2ft_pos[1],
                ee2ft_pos[2], 0, -ee2ft_pos[0],
                -ee2ft_pos[1], ee2ft_pos[0], 0;
  Adee2ft.topLeftCorner(3,3) = ee2ft_ori;
  Adee2ft.topRightCorner(3,3) = Eigen::MatrixXd::Zero(3,3);
  Adee2ft.bottomLeftCorner(3,3) = cee2ft_pos * ee2ft_ori;
  Adee2ft.bottomRightCorner(3,3) = ee2ft_ori;

  handcm2ft_ori << 0.7071, -0.7071, 0,
                   0.7071,  0.7071, 0,
                        0,       0, 1;
  chandcm2ft_pos << 0, -handcm2ft_pos[2], handcm2ft_pos[1],
                    handcm2ft_pos[2], 0, -handcm2ft_pos[0],
                    -handcm2ft_pos[1], handcm2ft_pos[0], 0;
  Adhandcm2ft.topLeftCorner(3,3) = handcm2ft_ori;
  Adhandcm2ft.topRightCorner(3,3) = Eigen::MatrixXd::Zero(3,3);
  Adhandcm2ft.bottomLeftCorner(3,3) = chandcm2ft_pos * handcm2ft_ori;
  Adhandcm2ft.bottomRightCorner(3,3) = handcm2ft_ori;

  // 发布 info 消息
  pub_ = node_handle.advertise<franka_example_controllers::info>("info_msg", 1000);

  return true;
}

void CartesianAdmittanceControllerVelocity::starting(const ros::Time& /* time */) {
  franka::RobotState initial_state = velocity_cartesian_handle_->getRobotState();
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE_d.data()));

  // 把平衡点设置为当前状态
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
  position_d_target_ = initial_transform.translation();
  orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());
}

void CartesianAdmittanceControllerVelocity::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {
  // 得到当前机械臂末端实际位置和姿态
  franka::RobotState robot_state = velocity_cartesian_handle_->getRobotState();
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE_d.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());

  // 计算当前位置与期望位置之间的误差（位置误差和方向误差）
  Vector6d error;
  error.head(3) << position - position_d_;
  // 求得末端坐标系下表示的方向误差，并将方向误差转换到 base 坐标系下
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0)
    orientation.coeffs() << -orientation.coeffs();
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  error.tail(3) << -transform.linear() * error.tail(3);

  // 将力/力矩传感器的度数转换到 base 坐标系下
  ft_filtered_ = 0.99 * ft_filtered_ + 0.01 * ft_;
  Vector6d ef = computeExternalFt(transform);

  // 计算当前加速度
  acc_ = cartesian_inertia_.inverse() * (force_gain_ * ef - cartesian_damping_ * twist_ - cartesian_stiffness_ * error);
  
  // 线加速度限幅
  double linear_acc_norm = (acc_.segment(0, 3)).norm();
  if (linear_acc_norm > linear_max_acc_) {
    // ROS_WARN_STREAM_THROTTLE(1, "Admittance generates high linear accelaration! accelaration norm: " << linear_acc_norm);
    acc_.segment(0, 3) *= (linear_max_acc_ / linear_acc_norm);
  }
  // 角加速度限幅
  double angular_acc_norm = (acc_.segment(3, 3)).norm();
  if (angular_acc_norm > angular_max_acc_) {
    // ROS_WARN_STREAM_THROTTLE(1, "Admittance generates high angular accelaration! accelaration norm: " << angular_acc_norm);
    acc_.segment(3, 3) *= (angular_max_acc_ / angular_acc_norm);
  }

  // 计算当前速度
  twist_ += acc_ * period.toSec();

  // 线速度限幅
  double linear_vel_norm = (twist_.segment(0, 3)).norm();
  if (linear_vel_norm > linear_max_vel_) {
    // ROS_WARN_STREAM_THROTTLE(1, "Admittance generate fast linear velocity! velocity norm: " << linear_vel_norm);
    twist_.segment(0, 3) *= (linear_max_vel_ / linear_vel_norm);
  }
  // 角速度限幅
  double angular_vel_norm = (twist_.segment(3, 3)).norm();
  if (angular_vel_norm > angular_max_vel_) {
    // ROS_WARN_STREAM_THROTTLE(1, "Admittance generate fast angular velocity! velocity norm: " << angular_vel_norm);
    twist_.segment(3, 3) *= (angular_max_vel_ / angular_vel_norm);
  }

  std::array<double, 6> command = {{twist_[0], twist_[1], twist_[2], twist_[3], twist_[4], twist_[5]}};
  velocity_cartesian_handle_->setCommand(command);

  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);


  // 发布 info 信息
  franka_example_controllers::info info_msg;
  // 当前位置姿态
  info_msg.posx = position[0];
  info_msg.posy = position[1];
  info_msg.posz = position[2];
  Eigen::Vector3d euler_angle = transform.linear().eulerAngles(2,1,0); // RPY (Z-Y-X)
  info_msg.euler_z = euler_angle[0];
  info_msg.euler_y = euler_angle[1];
  info_msg.euler_x = euler_angle[2];
  // 目标位置姿态
  info_msg.posx_d = position_d_target_[0];
  info_msg.posy_d = position_d_target_[1];
  info_msg.posz_d = position_d_target_[2];
  Eigen::Vector3d euler_angle_d = orientation_d_.matrix().eulerAngles(2,1,0); // RPY (Z-Y-X)
  info_msg.euler_z_d = euler_angle_d[0];
  info_msg.euler_y_d = euler_angle_d[1];
  info_msg.euler_x_d = euler_angle_d[2];
  // 位置误差
  info_msg.error_pos_x = error[0];
  info_msg.error_pos_y = error[1];
  info_msg.error_pos_z = error[2];
  info_msg.error_ori_x = error[3];
  info_msg.error_ori_y = error[4];
  info_msg.error_ori_z = error[5];
  // 外力
  info_msg.ef_fx = ef[0];
  info_msg.ef_fy = ef[1];
  info_msg.ef_fz = ef[2];
  info_msg.ef_tx = ef[3];
  info_msg.ef_ty = ef[4];
  info_msg.ef_tz = ef[5];
  // 速度指令
  info_msg.velx = twist_[0];
  info_msg.vely = twist_[1];
  info_msg.velz = twist_[2];
  info_msg.wx = twist_[3];
  info_msg.wy = twist_[4];
  info_msg.wz = twist_[5];
  // 加速度
  info_msg.accx = acc_[0];
  info_msg.accy = acc_[1];
  info_msg.accz = acc_[2];
  info_msg.betax = acc_[3];
  info_msg.betay = acc_[4];
  info_msg.betaz = acc_[5];

  pub_.publish(info_msg);

  // update parameters changed online either through dynamic reconfigure or through the interactive
  // target by filtering
  // cartesian_damping_ = filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
  // cartesian_stiffness_ = filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
}

// 订阅平衡位置
void CartesianAdmittanceControllerVelocity::equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
  // std::cout << msg->pose.position.x << " " << msg->pose.position.y << " " << msg->pose.position.z << std::endl;
  position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w;
  if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  }
}

Vector6d CartesianAdmittanceControllerVelocity::computeExternalFt(const Eigen::Affine3d& transform) {
  // gripper重力对传感器读数的贡献
  Vector6d hand_gravity_handcm;
  Eigen::Matrix<double, 3, 3> base2handcm_ori = transform.linear();
  hand_gravity_handcm.head(3) = base2handcm_ori.inverse() * Eigen::Vector3d{0, 0, 0}; // 力矩
  hand_gravity_handcm.tail(3) = base2handcm_ori.inverse() * Eigen::Vector3d{0, 0, -fz_bias}; // 力
  Vector6d hand_gravity_contribution = Adhandcm2ft.transpose() * hand_gravity_handcm;

  // std::cout << "hand_gravity_contribution: ";
  // for(int i=0; i<6; ++i)
  //   std::cout << hand_gravity_contribution[i] << "   ";
  // std::cout << std::endl;

  // 外力对传感器读数的贡献
  Vector6d mf;
  mf.head(3) = ft_filtered_.tail(3);
  mf.tail(3) = ft_filtered_.head(3);
  Vector6d external_force_contribution = mf - hand_gravity_contribution;
  // 外力在末端坐标系下的表示
  Vector6d external_force_ee = Adee2ft.transpose().inverse() * external_force_contribution;
  // 外力在基坐标系下的表示
  Vector6d external_force_base;
  external_force_base.head(3) = transform.linear() * external_force_ee.tail(3);
  external_force_base.tail(3) = transform.linear() * external_force_ee.head(3);

  // 计算当前末端力与期望力之差
  return external_force_base;
}

// 订阅力/力矩传感器读数
void CartesianAdmittanceControllerVelocity::ftCallback(const geometry_msgs::WrenchStamped& msg) {
  ft_[0] = msg.wrench.force.x;
  ft_[1] = msg.wrench.force.y;
  ft_[2] = msg.wrench.force.z + fz_bias;
  ft_[3] = msg.wrench.torque.x;
  ft_[4] = msg.wrench.torque.y;
  ft_[5] = msg.wrench.torque.z;
  // std::cout << "***" << std::endl;
}

void CartesianAdmittanceControllerVelocity::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianAdmittanceControllerVelocity,
                       controller_interface::ControllerBase)