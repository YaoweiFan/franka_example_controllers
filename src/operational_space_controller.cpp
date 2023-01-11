// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/operational_space_controller.h>

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
#include <franka_example_controllers/iden_dynamics.h>
#include <franka_example_controllers/p2p.h>


namespace franka_example_controllers {

bool OperationalSpaceController::init(hardware_interface::RobotHW* robot_hw,
                                          ros::NodeHandle& node_handle) {
  sub_equilibrium_pose_ = node_handle.subscribe(
    "equilibrium_pose", 20, &OperationalSpaceController::equilibriumPoseCallback, this,
    ros::TransportHints().reliable().tcpNoDelay());

  sub_ft_ = node_handle.subscribe(
      "ft_sensor/data", 20, &OperationalSpaceController::ftCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  // 获得 arm_id 和 joint_names
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("OperationalSpaceController: Could not get parameter arm_id");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "OperationalSpaceController: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  // 获得 model 句柄
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "OperationalSpaceController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "OperationalSpaceController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  // 获得 state 句柄
  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "OperationalSpaceController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "OperationalSpaceController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  // 获得 joint 句柄
  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "OperationalSpaceController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "OperationalSpaceController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  cartesian_inertia_.setZero();
  cartesian_inertia_.topLeftCorner(3,3) = 1.0 * Eigen::MatrixXd::Identity(3,3);
  cartesian_inertia_.bottomRightCorner(3,3) = 0.01 * Eigen::MatrixXd::Identity(3,3);

  cartesian_stiffness_.setZero();
  cartesian_stiffness_.topLeftCorner(3,3) = 500.0 * 1.0 * Eigen::MatrixXd::Identity(3,3);
  cartesian_stiffness_.bottomRightCorner(3,3) = 100.0 * 0.01 * Eigen::MatrixXd::Identity(3,3);

  cartesian_damping_.setZero();
  cartesian_damping_.topLeftCorner(3,3) = 2.0 * sqrt(500.0 * 1.0 * 1.0) * Eigen::MatrixXd::Identity(3,3);
  cartesian_damping_.bottomRightCorner(3,3) = 2.0 * sqrt(100.0 * 0.01 * 0.01) * Eigen::MatrixXd::Identity(3,3);

  twist_.setZero();

  ft_.setZero();
  ft_d_.setZero();
  ft_filtered_.setZero();

  ee2ft_ori << 0.7071, -0.7071, 0,
               0.7071,  0.7071, 0,
                    0,       0, 1;

  // 发布 debug 消息
  pub_ = node_handle.advertise<franka_example_controllers::info>("info_msg", 1000);

  return true;
}

void OperationalSpaceController::starting(const ros::Time& /* time */) {
  franka::RobotState initial_state = state_handle_->getRobotState();
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // 把平衡点设置为当前状态
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
  position_d_target_ = initial_transform.translation();
  orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());
  position_initial_ = initial_transform.translation();
  orientation_initial_ = Eigen::Quaterniond(initial_transform.linear());
  position_d_dot_.setZero();
  position_d_dot_dot_.setZero();

  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  q_d_nullspace_ = q_initial;

  std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  jacobian_ = jacobian;
}

void OperationalSpaceController::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {
  // 得到当前机械臂末端实际位置和姿态
  franka::RobotState robot_state = state_handle_->getRobotState();
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());

  elapsed_time_ += period;

  ft_filtered_ = 0.99 * ft_filtered_ + 0.01 * ft_;
  // 外力在基坐标系下的表示，需要确认一下 ee2ft_ori 是否是 eef 坐标系在传感器坐标系下的表示?
  Vector6d external_force_base;
  external_force_base.head(3) = transform.linear() * ee2ft_ori * ft_filtered_.head(3);
  external_force_base.tail(3) = transform.linear() * ee2ft_ori * ft_filtered_.tail(3);

  // 从控制器获得的运动学参数
  std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());

  // 从控制器获得的动力学参数
  std::array<double, 49> mass_matrix_array = model_handle_->getMass();
  Eigen::Map<Eigen::Matrix<double, 7, 7>> mass_matrix(mass_matrix_array.data());
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());
  Eigen::Matrix<double, 7, 1> friction;
  friction.setZero(); // 控制器的反馈信息没有携带摩擦力这一项

  // // 辨识得到的动力学参数
  // Eigen::Matrix<double, 7, 7> mass_matrix = MassMatrix(q);
  // Eigen::Matrix<double, 7, 7> coriolis = CoriolisMatrix(q, dq);
  // Eigen::Matrix<double, 7, 1> friction = Friction(dq);

  // 计算 lambda matrices (J * M^-1 * J^T)^-1
  auto lambda_inv =  jacobian * mass_matrix.inverse() * jacobian.transpose();
  auto lambda = lambda_inv.inverse();

  // *****************************************************************************************
  // 期望位置通过点到点路径规划获得
  double refx[4],refy[4],refz[4];
  p2p(elapsed_time_.toSec(), period.toSec(), 0.2, 0.1, 0.5, refx);
  p2p(elapsed_time_.toSec(), period.toSec(), 0.2, 0.1, 0.5, refy);
  p2p(elapsed_time_.toSec(), period.toSec(), 0.2, 0.1, 0.5, refz);

  position_d_[0] =  position_initial_[0] + refx[0];
  position_d_dot_[0] = refx[1];
  position_d_dot_dot_[0] = refx[2];

  position_d_[1] =  position_initial_[1] + refy[0];
  position_d_dot_[1] = refy[1];
  position_d_dot_dot_[1] = refy[2];

  position_d_[2] =  position_initial_[2] + refz[0];
  position_d_dot_[2] = refz[1];
  position_d_dot_dot_[2] = refz[2];

  // *****************************************************************************************

  // 计算当前位置与期望位置之间的误差（位置误差和方向误差）
  Vector6d error;
  error.head(3) << position - position_d_;
  // 求得末端坐标系下表示的方向误差，并将方向误差转换到 base 坐标系下
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0)
    orientation.coeffs() << -orientation.coeffs();
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  error.tail(3) << -transform.linear() * error.tail(3);

  Vector6d error_dot;
  error_dot = jacobian * dq;
  error_dot.head(3) -= position_d_dot_;

  Vector6d x_dot_dot;
  x_dot_dot.setZero();
  x_dot_dot.head(3) = position_d_dot_dot_;

  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);
  Vector6d desired_wrench = x_dot_dot- cartesian_damping_ * error_dot - cartesian_stiffness_ * error;
  Vector6d decoupled_wrench = lambda * desired_wrench; // -ef
  tau_task << jacobian.transpose() * decoupled_wrench;

  auto jbar = mass_matrix.inverse() * jacobian.transpose() * lambda;
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jbar * jacobian).transpose() * mass_matrix *
                       (nullspace_stiffness_ * (q_d_nullspace_ - q) -
                        (2.0 * sqrt(nullspace_stiffness_)) * dq);

  auto jacobian_dot = (jacobian - jacobian_) / period.toSec();
  jacobian_ = jacobian;
  auto h = jacobian.transpose() * lambda * jacobian_dot * dq;
  tau_d << tau_task + coriolis - h + jacobian.transpose()*external_force_base + tau_nullspace; // + friction


  tau_d << saturateTorqueRate(tau_d, tau_J_d);
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }

  // 期望位置通过订阅消息后插值获得
  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);

  // 发布 info 信息
  franka_example_controllers::info info_msg;
  info_msg.posx = position[0];
  info_msg.posy = position[1];
  info_msg.posz = position[2];
  info_msg.posx_d = position_d_[0];
  info_msg.posy_d = position_d_[1];
  info_msg.posz_d = position_d_[2];
  Eigen::Vector3d euler_d = orientation_d_.matrix().eulerAngles(2,1,0); // 目标欧拉角 Z-Y-X (R-P-Y)
  Eigen::Vector3d euler_c = transform.linear().eulerAngles(2,1,0); // 当前欧拉角 Z-Y-X (R-P-Y)
  info_msg.euler_z_d = euler_d[0];
  info_msg.euler_y_d = euler_d[1];
  info_msg.euler_x_d = euler_d[2];
  info_msg.euler_z = euler_c[0];
  info_msg.euler_y = euler_c[1];
  info_msg.euler_x = euler_c[2];
  pub_.publish(info_msg);

}

// 输出饱和限制
Eigen::Matrix<double, 7, 1> OperationalSpaceController::saturateTorqueRate(
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
void OperationalSpaceController::equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
  position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w;
  if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  }
}

// 订阅力/力矩传感器读数
void OperationalSpaceController::ftCallback(const geometry_msgs::WrenchStamped& msg) {
  ft_[0] = msg.wrench.force.x;
  ft_[1] = msg.wrench.force.y;
  ft_[2] = msg.wrench.force.z;
  ft_[3] = msg.wrench.torque.x;
  ft_[4] = msg.wrench.torque.y;
  ft_[5] = msg.wrench.torque.z;
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::OperationalSpaceController, controller_interface::ControllerBase)