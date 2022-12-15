// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/cartesian_admittance_controller_torque.h>

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
#include "franka_example_controllers/debug.h"

namespace franka_example_controllers {

bool CartesianAdmittanceControllerTorque::init(hardware_interface::RobotHW* robot_hw,
                                          ros::NodeHandle& node_handle) {
  sub_equilibrium_pose_ = node_handle.subscribe(
    "equilibrium_pose", 20, &CartesianAdmittanceControllerTorque::equilibriumPoseCallback, this,
    ros::TransportHints().reliable().tcpNoDelay());

  sub_ft_ = node_handle.subscribe(
      "ft_sensor/data", 20, &CartesianAdmittanceControllerTorque::ftCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  // 获得 arm_id 和 joint_names
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("CartesianAdmittanceControllerTorque: Could not get parameter arm_id");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "CartesianAdmittanceControllerTorque: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  // 获得 model 句柄
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianAdmittanceControllerTorque: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianAdmittanceControllerTorque: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  // 获得 state 句柄
  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianAdmittanceControllerTorque: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianAdmittanceControllerTorque: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  // 获得 joint 句柄
  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianAdmittanceControllerTorque: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "CartesianAdmittanceControllerTorque: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  // cartesian_inertia_.topLeftCorner(3,3) = 10 * Eigen::MatrixXd::Identity(3,3);
  // cartesian_inertia_.bottomRightCorner(3,3) = 0.1 * Eigen::MatrixXd::Identity(3,3);
  // cartesian_damping_.setZero();
  cartesian_damping_.topLeftCorner(3,3) = 2.0 * sqrt(150) * Eigen::MatrixXd::Identity(3,3);
  cartesian_damping_.bottomRightCorner(3,3) = 2.0 * sqrt(1) * Eigen::MatrixXd::Identity(3,3);
  // cartesian_stiffness_.setZero();
  cartesian_stiffness_.topLeftCorner(3,3) = 150 * Eigen::MatrixXd::Identity(3,3);
  cartesian_stiffness_.bottomRightCorner(3,3) = 1500 * Eigen::MatrixXd::Identity(3,3);

  twist_.setZero();

  ft_.setZero();
  ft_[2] = 7.3;
  ft_d_.setZero();
  // ft_d_[5] = 5;

  ft_filtered_.setZero();
  ft_filtered_[2] = 7.3;
  // recv_ft_data = false;

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

  // 发布 debug 消息
  pub_ = node_handle.advertise<franka_example_controllers::info>("info_msg", 1000);

  return true;
}

void CartesianAdmittanceControllerTorque::starting(const ros::Time& /* time */) {
  franka::RobotState initial_state = state_handle_->getRobotState();
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // 把平衡点设置为当前状态
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
  position_d_target_ = initial_transform.translation();
  orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());

  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  q_d_nullspace_ = q_initial;
}

void CartesianAdmittanceControllerTorque::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {
  // 得到当前机械臂末端实际位置和姿态
  franka::RobotState robot_state = state_handle_->getRobotState();
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
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

  // std::cout << "***" << std::endl;
  // std::cout << error << std::endl;
  // std::cout << std::endl;
  
  // Eigen::Vector3d rc1 = transform.linear().col(0);
  // Eigen::Vector3d rc2 = transform.linear().col(1);
  // Eigen::Vector3d rc3 = transform.linear().col(2);
  // Eigen::Vector3d rd1 = orientation_d_.toRotationMatrix().col(0);
  // Eigen::Vector3d rd2 = orientation_d_.toRotationMatrix().col(1);
  // Eigen::Vector3d rd3 = orientation_d_.toRotationMatrix().col(2);

  // Eigen::Vector3d err = 0.5 * (rd1.cross(rc1) + rd2.cross(rc2) + rd3.cross(rc3));
  // error.tail(3) = 0.5 * (rd1.cross(rc1) + rd2.cross(rc2) + rd3.cross(rc3));
  // std::cout << err << std::endl;
  // std::cout << std::endl;
  // std::cout << error[0] / err[0] << std::endl;
  // std::cout << error[1] / err[1] << std::endl;
  // std::cout << error[2] / err[2] << std::endl;
  // std::cout << "***" << std::endl;

  // 对力/力矩传感器的读数进行滤波
  // if(!recv_ft_data) {
  //   while(ft_.sum() == 0);
  //   ft_filtered_ = ft_;
  //   recv_ft_data = true;
  // }

  ft_filtered_ = 0.9 * ft_filtered_ + 0.1 * ft_;

  // 计算末端实际受到的力大小 -- 表示在基坐标系下
  Vector6d ef = computeExternalFt(transform);
  // 打印末端位置
  // std::cout << "eff_pos: ";
  // for(int i=0; i<3; ++i)
  //   std::cout << position[i] << "   ";
  // std::cout << std::endl;
  // // 打印测量到的 力/力矩
  // std::cout << "ft_filtered: ";
  // for(int i=0; i<6; ++i)
  //   std::cout << ft_filtered_[i] << "   ";
  // std::cout << std::endl;
  // 打印估计的等效末端 力/力矩
  // std::cout << "ExternalFt: ";
  // for(int i=0; i<6; ++i)
  //   std::cout << ef[i] << "   ";
  // std::cout << std::endl;
  // // 打印末端位置偏差
  // std::cout << "error: ";
  // for(int i=0; i<6; ++i)
  //   std::cout << error[i] << "   ";
  // std::cout << std::endl;
  // // 打印由位置偏差计算得到的力
  // std::cout << "150*error: ";
  // for(int i=0; i<6; ++i)
  //   std::cout << 150*error[i] << "   ";
  // std::cout << std::endl;

  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  // 计算 lambda matrices (J * M^-1 * J^T)^-1
  std::array<double, 49> mass_matrix_array = model_handle_->getMass();
  Eigen::Map<Eigen::Matrix<double, 7, 7>> mass_matrix(mass_matrix_array.data());
  std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());

  auto lambda_inv =  jacobian * mass_matrix.inverse() * jacobian.transpose();
  auto lambda = lambda_inv.inverse();

  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);
  // Vector6d desired_wrench = - cartesian_damping_ * jacobian * dq - cartesian_stiffness_ * error + ef + ft_d_;
  Vector6d desired_wrench = - cartesian_damping_ * jacobian * dq - cartesian_stiffness_ * error;

  // std::cout << desired_wrench << std::endl;

  Vector6d decoupled_wrench = lambda * desired_wrench;
  // tau_task << jacobian.transpose() * (decoupled_wrench - ef);
  tau_task << jacobian.transpose() * decoupled_wrench;

  // tau_task << jacobian.transpose() * (decoupled_wrench);
  // for(int i=0; i<6; ++i){
  //   std::cout << twist_[i] << ", ";
  // }
  // std::cout << std::endl;
  // for(int i=0; i<6; ++i){
  //   std::cout << error[i] << "   ";
  // }
  // std::cout << std::endl;
  // std::cout << std::endl;
  auto jbar = mass_matrix.inverse() * jacobian.transpose() * lambda;

  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jbar * jacobian).transpose() * mass_matrix *
                       (nullspace_stiffness_ * (q_d_nullspace_ - q) -
                        (2.0 * sqrt(nullspace_stiffness_)) * dq);

  tau_d << tau_task + tau_nullspace + coriolis;

  tau_d << saturateTorqueRate(tau_d, tau_J_d);
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }
  // std::cout << position_d_target_[0] << " " << position_d_target_[1] << " " << position_d_target_[2] << std::endl;
  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);

  // 发布 debug 信息
  // franka_example_controllers::debug debug_msg;
  // debug_msg.error_pos_x = error[0];
  // debug_msg.error_pos_y = error[1];
  // debug_msg.error_pos_z = error[2];
  // debug_msg.error_ori_x = error[3];
  // debug_msg.error_ori_y = error[4];
  // debug_msg.error_ori_z = error[5];

  // debug_msg.ef_fx = ef[0];
  // debug_msg.ef_fy = ef[1];
  // debug_msg.ef_fz = ef[2];
  // debug_msg.ef_tx = ef[3];
  // debug_msg.ef_ty = ef[4];
  // debug_msg.ef_tz = ef[5];

  // debug_msg.desired_wrench_fx = desired_wrench[0];
  // debug_msg.desired_wrench_fy = desired_wrench[1];
  // debug_msg.desired_wrench_fz = desired_wrench[2];
  // debug_msg.desired_wrench_tx = desired_wrench[3];
  // debug_msg.desired_wrench_ty = desired_wrench[4];
  // debug_msg.desired_wrench_tz = desired_wrench[5];

  // debug_msg.decoupled_wrench_fx = decoupled_wrench[0];
  // debug_msg.decoupled_wrench_fy = decoupled_wrench[1];
  // debug_msg.decoupled_wrench_fz = decoupled_wrench[2];
  // debug_msg.decoupled_wrench_tx = decoupled_wrench[3];
  // debug_msg.decoupled_wrench_ty = decoupled_wrench[4];
  // debug_msg.decoupled_wrench_tz = decoupled_wrench[5];

  // pub_.publish(debug_msg);

  // 发布 info 信息
  franka_example_controllers::info info_msg;
  info_msg.posx = position[0];
  info_msg.posy = position[1];
  info_msg.posz = position[2];
  info_msg.posx_d = position_d_target_[0];
  info_msg.posy_d = position_d_target_[1];
  info_msg.posz_d = position_d_target_[2];
  info_msg.err_ori_x = error[3];
  info_msg.err_ori_y = error[4];
  info_msg.err_ori_z = error[5];

  pub_.publish(info_msg);

}

Vector6d CartesianAdmittanceControllerTorque::computeExternalFt(const Eigen::Affine3d& transform) {
  // gripper重力对传感器读数的贡献
  Vector6d hand_gravity_handcm;
  Eigen::Matrix<double, 3, 3> base2handcm_ori = transform.linear();
  hand_gravity_handcm.head(3) = base2handcm_ori.inverse() * Eigen::Vector3d{0, 0, 0}; // 力矩
  hand_gravity_handcm.tail(3) = base2handcm_ori.inverse() * Eigen::Vector3d{0, 0, -7.3}; // 力
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

Eigen::Matrix<double, 7, 1> CartesianAdmittanceControllerTorque::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

// 订阅平衡位置
void CartesianAdmittanceControllerTorque::equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
  // std::cout << msg->pose.position.x << " " << msg->pose.position.y << " " << msg->pose.position.z << std::endl;
  position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w;
  if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  }
  // std::cout << "yyds!" << std::endl;
}

// 订阅力/力矩传感器读数
void CartesianAdmittanceControllerTorque::ftCallback(const geometry_msgs::WrenchStamped& msg) {
  ft_[0] = msg.wrench.force.x;
  ft_[1] = msg.wrench.force.y;
  ft_[2] = msg.wrench.force.z;
  ft_[3] = msg.wrench.torque.x;
  ft_[4] = msg.wrench.torque.y;
  ft_[5] = msg.wrench.torque.z;
  // for(int i=0; i<6; ++i)
  //   std::cout << ft_[i] << "   ";
  // std::cout << std::endl;
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianAdmittanceControllerTorque,
                       controller_interface::ControllerBase)