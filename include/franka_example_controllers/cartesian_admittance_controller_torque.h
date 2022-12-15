// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <memory>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>

#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_model_interface.h>

namespace franka_example_controllers {

typedef Eigen::Matrix<double, 6, 1> Vector6d;

class CartesianAdmittanceControllerTorque
    : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface,
                                                            franka_hw::FrankaModelInterface,
                                                            franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  // Saturation
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
                                                 const Eigen::Matrix<double, 7, 1>& tau_J_d);
  Vector6d computeExternalFt(const Eigen::Affine3d& transform);

  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  double filter_params_{0.005};
  double linear_max_acc_{1.0};
  double angular_max_acc_{1.0};
  double linear_max_vel_{0.5};
  double angular_max_vel_{0.5};

  double nullspace_stiffness_{20.0};
  const double delta_tau_max_{1.0};
  Eigen::Matrix<double, 7, 1> q_d_nullspace_;

  Eigen::Matrix<double, 6, 6> cartesian_inertia_;
  // Eigen::Matrix<double, 6, 6> cartesian_inertia_target_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_;
  // Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
  // Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;

  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_;
  Eigen::Vector3d position_d_target_;
  Eigen::Quaterniond orientation_d_target_;
  Vector6d ft_d_;

  // 当前数值
  std::array<double, 16> pose_;
  Vector6d twist_;
  Vector6d acc_;
  Vector6d ft_;
  Vector6d ft_filtered_;
  bool recv_ft_data;


  // 计算末端外力相关矩阵系数
  Eigen::Matrix<double, 3, 3> ee2ft_ori;
  Eigen::Vector3d ee2ft_pos{0, 0, -0.123};
  Eigen::Matrix<double, 3, 3> cee2ft_pos;
  Eigen::Matrix<double, 6, 6> Adee2ft;

  Eigen::Matrix<double, 3, 3> handcm2ft_ori;
  Eigen::Vector3d handcm2ft_pos{0.01, 0, -0.050};
  Eigen::Matrix<double, 3, 3> chandcm2ft_pos;
  Eigen::Matrix<double, 6, 6> Adhandcm2ft;


  // Equilibrium pose subscriber
  ros::Subscriber sub_equilibrium_pose_;
  void equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  // Force/Torque subscriber
  ros::Subscriber sub_ft_;
  void ftCallback(const geometry_msgs::WrenchStamped& msg);
  // infomation publisher
  ros::Publisher pub_;
};

}  // namespace franka_example_controllers
