
#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>

#include <franka_example_controllers/armrc_out.h>
// #include <franka_example_controllers/vsmrc_paramConfig.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/trigger_rate.h>

#include <geometry_msgs/WrenchStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

using namespace std;
// #define PREDICT 80
namespace franka_example_controllers
{

  class ARCmodel_reaching : public controller_interface::MultiInterfaceController<
                                     franka_hw::FrankaModelInterface,
                                     hardware_interface::EffortJointInterface,
                                    //  franka_hw::FrankaPoseCartesianInterface,
                                     franka_hw::FrankaStateInterface>
                                  
  {
  public:
    bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) override;
    void starting(const ros::Time &) override;
    void update(const ros::Time &, const ros::Duration &period) override;

  private:
    /* data */
    Eigen::Matrix<double, 6, 6> M_d;
    Eigen::Matrix<double, 6, 6> D_d;
    Eigen::Matrix<double, 6, 6> K_d;
    Eigen::Matrix<double, 6, 6> K_f;

    Eigen::Matrix<double, 6, 1> F_ft;

    Eigen::Matrix<double, 6, 6> A;
    Eigen::Matrix<double, 6, 6> F_1;
    Eigen::Matrix<double, 6, 6> F_2;
    Eigen::Matrix<double, 6, 6> Ks;

    Eigen::Vector3d r0_pos;
    Eigen::Quaterniond r0_ori;
    Eigen::Matrix<double, 6, 7> jacobian0;
    Eigen::Matrix<double, 6, 1> z;
    Eigen::Matrix<double, 6, 1> s_filter;


    Eigen::Matrix<double, 7, 1> q_filter;
    Eigen::Matrix<double, 7, 1> dq_filter;
    Eigen::Matrix<double, 7, 1> tau_J_filter;
    Eigen::Matrix<double, 7, 1> q_init;
 
    double fref;
    
   
    // Saturation
    Eigen::Matrix<double, 7, 1> saturateTorqueRate(
        const Eigen::Matrix<double, 7, 1> &tau_d_calculated,
        const Eigen::Matrix<double, 7, 1> &tau_J_d); // NOLINT (readability-identifier-naming)

    
    unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
    unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
    vector<hardware_interface::JointHandle> joint_handles_;
    // franka_hw::FrankaPoseCartesianInterface* cartesian_pose_interface_;
    // unique_ptr<franka_hw::FrankaCartesianPoseHandle> cartesian_pose_handle_;

    //total time
    ros::Duration elapsed_time_;
    const double delta_tau_max_{1.0};
    franka_hw::TriggerRate rate_trigger_vsmrc{100.0};

    realtime_tools::RealtimePublisher<armrc_out> out_publisher;
    // unique_ptr<dynamic_reconfigure::Server<franka_example_controllers::vsmrc_paramConfig>>
    //     dynamic_server_vsmrc_param_;
    // ros::NodeHandle dynamic_reconfigure_vsmrc_param_node_;
    // void vsmrcParamCallback(franka_example_controllers::vsmrc_paramConfig &config,
    //                         uint32_t level);
    ros::Subscriber sub_ftsensor;
    void ftsensorCallback(const geometry_msgs::WrenchStamped& msg);
    //numerical differentiation of jacobian matrix
    Eigen::Matrix<double, 6, 7> Jacobian_diff(
      const Eigen::Matrix<double, 6, 7> &jaco_0, const Eigen::Matrix<double, 6, 7> &jaco_1, const double sample);
    
    // void qpParameterCal(double stiffness, double damping);
    
  };
    

} // namespace franka_example_controllers
