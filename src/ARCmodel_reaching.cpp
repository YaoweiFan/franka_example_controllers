#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <franka_example_controllers/ARCmodel_reaching.h>
#include <franka_example_controllers/iden_dynamics.h>
#include <franka_example_controllers/pseudo_inversion.h>
#include <geometry_msgs/WrenchStamped.h>

#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <cmath>
#include <memory>
// #include <franka_example_controllers/spline.h>
// #include <qpOASES.hpp>
#include <franka_example_controllers/p2p.h>

// USING_NAMESPACE_QPOASES;
using namespace std;
// using namespace tk;


namespace franka_example_controllers
{

  bool ARCmodel_reaching::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
  {
    sub_ftsensor = node_handle.subscribe(
      "/ft_sensor/data", 10, &ARCmodel_reaching::ftsensorCallback, this, 
      ros::TransportHints().reliable().tcpNoDelay()
    );

    string arm_id;
    if (!node_handle.getParam("arm_id", arm_id))
    {
      ROS_ERROR("ARCmodel_reaching: Could not read parameter arm_id");
      return false;
    }

    vector<string> joint_names;
    if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7)
    {
      ROS_ERROR(
          "ARCmodel_reaching: Invalid or no joint_names parameters provided, "
          "aborting controller init!");
      return false;
    }

    auto *model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    if (model_interface == nullptr)
    {
      ROS_ERROR_STREAM("ARCmodel_reaching: Error getting model interface from hardware");
      return false;
    }
    try
    {
      model_handle_ =
          make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
    }
    catch (hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM(
          "ARCmodel_reaching: Exception getting model handle from interface: " << ex.what());
      return false;
    }

    auto *state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr)
    {
      ROS_ERROR_STREAM("ARCmodel_reaching: Error getting state interface from hardware");
      return false;
    }
    try
    {
      state_handle_ =
          make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(arm_id + "_robot"));
    }
    catch (hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM(
          "ARCmodel_reaching: Exception getting state handle from interface: " << ex.what());
      return false;
    }

    auto *effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (effort_joint_interface == nullptr)
    {
      ROS_ERROR_STREAM("ARCmodel_reaching: Error getting effort joint interface from hardware");
      return false;
    }
    for (size_t i = 0; i < 7; ++i)
    {
      try
      {
        joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
      }
      catch (const hardware_interface::HardwareInterfaceException &ex)
      {
        ROS_ERROR_STREAM("ARCmodel_reaching: Exception getting joint handles: " << ex.what());
        return false;
      }
    }

    // vsmrc_publisher_.init(node_handle, "vsmrc_out_data", 4);
    out_publisher.init(node_handle, "out_data_show", 4);
    
    r0_pos.setZero();
    r0_ori.coeffs() << 0.0, 0.0, 0.0, 1.0;
    jacobian0.setZero();
   
    z.setZero();
    s_filter.setZero();
    A.setZero();
    F_1.setZero();
    F_2.setZero();
    K_d.setZero();
    D_d.setZero();
    K_f.setZero();
    Ks.setZero();
    
    M_d.setZero();

    F_ft.setZero();

    return true;
  }

  void ARCmodel_reaching::starting(const ros::Time & /*time*/)
  {
    // get the initial state
    franka::RobotState initial_state = state_handle_->getRobotState();
    // initial_pose_ = initial_state.O_T_EE_d;
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    r0_pos = initial_transform.translation();
    r0_ori = Eigen::Quaterniond(initial_transform.linear());
    array<double, 42> initial_jacobian = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian_0(initial_jacobian.data());
    jacobian0 = jacobian_0;
    Eigen::Map<Eigen::Matrix<double, 7, 1> > q_0(initial_state.q.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1> > dq_0(initial_state.dq.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1> > tau_J_0(initial_state.tau_J.data());
    q_filter = q_0;
    dq_filter = dq_0;
    tau_J_filter = tau_J_0;
    q_init = q_0;

    A << -2 * Eigen::Matrix<double, 6, 6>::Identity(); //smaller integrator
    F_1 << 1 * Eigen::Matrix<double, 6, 6>::Identity();
    F_2 << 10 * Eigen::Matrix<double, 6, 6>::Identity();

    //set initial M D K Ks matrix
    Ks.topLeftCorner(3,3) << 50 * Eigen::Matrix3d::Identity(); //50   20
    Ks.bottomRightCorner(3,3) << 5 * Eigen::Matrix3d::Identity();//5   2

    M_d.setIdentity();
    M_d.topLeftCorner(3, 3) << 1.0 * Eigen::Matrix3d::Identity();
    M_d.bottomRightCorner(3, 3) << 0.01 * Eigen::Matrix3d::Identity();

    K_d.setIdentity();
    K_d.topLeftCorner(3, 3) << 500.0 * 1.0 * Eigen::Matrix3d::Identity(); //500
    K_d.bottomRightCorner(3, 3) << 100.0 * 0.01 * Eigen::Matrix3d::Identity(); //100
    // K_d(2, 2) = 100.0;

    D_d.setIdentity();
    D_d.topLeftCorner(3, 3) << 2.0 * sqrt(500.0 * 1.0 * 1.0) * Eigen::Matrix3d::Identity();
    D_d.bottomRightCorner(3, 3) << 2.0 * sqrt(100.0 * 0.01 * 0.01) * Eigen::Matrix3d::Identity();
    // D_d(2,2) = 2.0 * sqrt(100.0) * 1.0;

    K_f.setIdentity();
    K_f.topLeftCorner(3, 3) << 1.0 * Eigen::Matrix3d::Identity();
    K_f.bottomRightCorner(3, 3) << 0.01 * Eigen::Matrix3d::Identity();
    // K_f(2,2) = -0.5;//-1.0;
    
    fref = 0.0;

    elapsed_time_ = ros::Duration(0.0);
  }

  void ARCmodel_reaching::update(const ros::Time & /*time*/, const ros::Duration &period)
  {

    Eigen::Vector3d rd_pos;
    Eigen::Quaterniond rd_ori;
    Eigen::Matrix<double, 6, 1> rd_dot;
    Eigen::Matrix<double, 6, 1> rd_ddot;
    Eigen::Matrix<double, 6, 1> F_d;
    
    elapsed_time_ += period;
    double Ts_ = period.toSec();
    // state and model
    franka::RobotState robot_state = state_handle_->getRobotState();
    // array<double, 7> coriolis_array = model_handle_->getCoriolis();
    array<double, 7> gravity_array = model_handle_->getGravity();
    array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector); //choose EE as the task space
    array<double, 49> mass_array = model_handle_->getMass();
    // convert to Eigen
    // Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1> > gravity(gravity_array.data());
    Eigen::Map<Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
    Eigen::Map<Eigen::Matrix<double, 7, 7>> mass(mass_array.data());

    //robot states
    Eigen::Map<Eigen::Matrix<double, 7, 1> > q(robot_state.q.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1> > dq(robot_state.dq.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1> > tau_J_d(robot_state.tau_J_d.data());     //desired toqure without gravity(last sample)
    Eigen::Map<Eigen::Matrix<double, 7, 1> > tau_J(robot_state.tau_J.data());         //measured joint torque
    Eigen::Map<Eigen::Matrix<double, 6, 1> > F_ext(robot_state.K_F_ext_hat_K.data()); //estimated stiffness frame force

    
    //identified dynamics MCG
    Eigen::Matrix<double, 7, 7> Mass_iden = MassMatrix(q);
    Eigen::Matrix<double, 7, 7> Coriolis_iden = CoriolisMatrix(q, dq);
    Eigen::Matrix<double, 7, 1> Friction_iden = Friction(dq);
    Eigen::Matrix<double, 7, 1> Gravity_iden = GravityVector(q);

    //get the current pose
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data())); //transform matrix, pose
    Eigen::Vector3d r_pos(transform.translation());
    Eigen::Quaterniond r_ori(transform.linear());
    Eigen::Matrix<double, 6, 1> r_dot(jacobian * dq);

    //convert the external wrench to cartesian coordinate
    Eigen::Matrix<double, 6, 1> Fc_ft;
    Fc_ft.head(3) = transform.linear() * F_ft.head(3);
    Fc_ft.tail(3) = transform.linear() * F_ft.tail(3);
    
    //calculate jacobian_diff inverse
    Eigen::Matrix<double, 6, 7> jacobian_diff = Jacobian_diff(jacobian0, jacobian, Ts_);
    jacobian0 = jacobian;
    Eigen::MatrixXd jacobian_pinv;
    pseudoInverse(jacobian, jacobian_pinv);  //damped Singular value decomposition

    // model 中的期望项  
    rd_pos = r0_pos;
    rd_ori = r0_ori;
    rd_dot.setZero();
    rd_ddot.setZero();
    F_d.setZero();

    double fref_target;
    fref_target = 0.0;
    if (elapsed_time_.toSec() > 2.0) {
      fref += 0.01 * (fref_target - fref);
    }

    // 点到点路径规划
    double refx[4],refy[4],refz[4];
    p2p(elapsed_time_.toSec(), Ts_, 0.2, 0.1, 0.5, refx);
    p2p(elapsed_time_.toSec(), Ts_, 0.2, 0.1, 0.5, refy);
    p2p(elapsed_time_.toSec(), Ts_, 0.2, 0.1, 0.5, refz);

    
    rd_pos[0] = refx[0] + r0_pos[0];
    rd_dot[0] = refx[1];
    rd_ddot[0] = refx[2];

    rd_pos[1] = -refy[0] + r0_pos[1];
    rd_dot[1] = -refy[1];
    rd_ddot[1] = -refy[2];

    rd_pos[2] = refz[0] + r0_pos[2];
    rd_dot[2] = refz[1];
    rd_ddot[2] = refz[2];

    F_d << 0.0, 0.0, 0, 0.0, 0.0, 0.0;

    //dynamics in task space
    Eigen::Matrix<double, 6, 6> Mr;
    Eigen::Matrix<double, 6, 6> Cr;

    Mr = jacobian_pinv.transpose() * Mass_iden * jacobian_pinv;
    Cr = jacobian_pinv.transpose() * Coriolis_iden * jacobian_pinv - Mr * jacobian_diff * jacobian_pinv;

    //compute error
    Eigen::Matrix<double, 6, 1> r_error;
    r_error.head(3) << r_pos - rd_pos;
    if (rd_ori.coeffs().dot(r_ori.coeffs()) < 0.0)
    {
      r_ori.coeffs() << -r_ori.coeffs();
    }
    Eigen::Quaterniond error_quaternion(r_ori.inverse() * rd_ori);
    r_error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    r_error.tail(3) << -transform.linear() * r_error.tail(3);
    Eigen::Matrix<double, 6, 1> r_doterror(r_dot - rd_dot);
    Eigen::Matrix<double, 6, 1> f_error(Fc_ft - F_d);

    //control law design
    Eigen::Matrix<double, 6, 6> Kpz;
    Eigen::Matrix<double, 6, 6> Kdz;
    Eigen::Matrix<double, 6, 6> Kfz;

    Eigen::Matrix<double, 6, 1> s;
    Eigen::Matrix<double, 6, 1> z_dot;
    Eigen::Matrix<double, 6, 1> req_dot;
    Eigen::Matrix<double, 6, 1> req_ddot;
    Eigen::Matrix<double, 6, 1> Tr;
    Eigen::Matrix<double, 7, 1> tau_d;
    Eigen::Matrix<double, 7, 1> tau_sd;

    Kpz << F_2.inverse() * (M_d.inverse() * K_d + A * F_1);
    Kdz << F_2.inverse() * (M_d.inverse() * D_d - F_1 + A);
    Kfz << F_2.inverse() * M_d.inverse() * K_f;

    z_dot << A * z + Kpz * r_error + Kdz * r_doterror + Kfz * f_error;
    s << r_doterror + F_1 * r_error + F_2 * z;
    req_dot << rd_dot - F_1 * r_error - F_2 * z;
    req_ddot << rd_ddot - F_1 * r_doterror - F_2 * z_dot;

    Eigen::Matrix<double, 7, 1> qeq_dot;
    Eigen::Matrix<double, 7, 1> qeq_ddot;
    Eigen::Matrix<double, 6, 1> Tr_fb;
    Tr_fb = - Fc_ft - Ks * s;
    qeq_dot = jacobian_pinv * req_dot;
    qeq_ddot = jacobian_pinv * (req_ddot - jacobian_diff * qeq_dot);

    z += z_dot * Ts_;

    Eigen::Matrix<double, 7, 1> tau_mismatch;
    Eigen::Matrix<double, 7, 1> q_filter_dot;
    Eigen::Matrix<double, 7, 1> q_filter_ddot;
    Eigen::Matrix<double, 7, 1> tau_J_dot;

    // omega_f 是什么含义？
    double omega_f = 2 * M_PI * 20;

    q_filter_dot = omega_f * q - omega_f * q_filter;
    q_filter += q_filter_dot * Ts_;

    q_filter_ddot = omega_f * dq - omega_f * dq_filter;
    dq_filter += q_filter_ddot * Ts_;

    tau_J_dot = omega_f * tau_J - omega_f * tau_J_filter;
    tau_J_filter += tau_J_dot * Ts_;

    Eigen::Matrix<double, 7, 7> Mass_iden_f = MassMatrix(q_filter);
    Eigen::Matrix<double, 7, 7> Coriolis_iden_f = CoriolisMatrix(q_filter, dq_filter);
    Eigen::Matrix<double, 7, 1> Friction_iden_f = Friction(dq_filter);
    Eigen::Matrix<double, 7, 1> Gravity_iden_f = GravityVector(q_filter);
    
    tau_mismatch = tau_J_filter - (Mass_iden_f * q_filter_ddot + Coriolis_iden_f * dq_filter + Gravity_iden_f);

    // 计算 nullspace 的力矩
    Eigen::Matrix<double, 7, 1> K_j_v, D_j_v, tau_null;
    Eigen::Matrix<double, 7, 7> K_joint, D_joint, M_joint;
    K_j_v << 1 * 0.5, 30, 20, 5, 100*0.05, 200*0.05, 50*0.01;
    D_j_v << 2*sqrt(1)*0.5, 2*sqrt(30), 2*sqrt(20), 2*sqrt(5), 2*sqrt(100)*0.05, 2*sqrt(200)*0.05, 2*sqrt(50)*0.01;
    K_joint = K_j_v.asDiagonal();
    D_joint = D_j_v.asDiagonal();
    tau_null << (Eigen::MatrixXd::Identity(7,7) - jacobian.transpose() * jacobian_pinv.transpose()) * Mass_iden * (K_joint * (q_init - q) - (D_joint * dq));
    
    if (elapsed_time_.toSec() <= 0.01){
      tau_d << 0,0,0,0,0,0;
    }
    else{
      tau_d << tau_null + jacobian.transpose() * Tr_fb +  Mass_iden * qeq_ddot + Coriolis_iden * qeq_dot;
    }
    tau_sd << saturateTorqueRate(tau_d, tau_J_d);

    // set command
    for (size_t i = 0; i < 7; ++i){
      joint_handles_[i].setCommand(tau_sd(i));
    }

    // calculate the impedance error
    Eigen::Matrix<double, 6, 1> s_f_dot;
    Eigen::Matrix<double, 6, 1> imp_e;
    s_f_dot = s * 100 - s_filter * 100;
    s_filter += s_f_dot * Ts_;
    imp_e = M_d * (s_f_dot - A * s_filter);

    
    Eigen::Matrix<double, 7, 1> tau_hat_;
    tau_hat_ = Mass_iden_f * q_filter_ddot + Coriolis_iden_f * dq_filter + Gravity_iden_f - jacobian.transpose() * Fc_ft;
    

    // 发布消息，这里为什么要加上锁？ 在 realtime 的情况下保证消息的准确性  
    if (rate_trigger_vsmrc() && out_publisher.trylock())
    {
      out_publisher.msg_.time_data = elapsed_time_.toSec(); //Ts_;
      for (size_t i = 0; i < 7; i++)
      {
        out_publisher.msg_.input[i] = tau_hat_[i]; //tau_d[i] + gravity[i]; //
        out_publisher.msg_.actual[i] = tau_J[i]; //tau_J_d[i] + gravity[i]; //
        out_publisher.msg_.tau_mis[i] = tau_mismatch[i];
      }
      for (size_t i = 0; i < 6; i++)
      {
        out_publisher.msg_.r_error[i] = r_error[i];
        out_publisher.msg_.s_out[i] = s[i];
        out_publisher.msg_.z_out[i] = z[i];
        out_publisher.msg_.F_e[i] = Fc_ft[i];
        out_publisher.msg_.F_d[i] = F_d[i];
        out_publisher.msg_.impe_error[i] = imp_e[i];
      }
      for (size_t i = 0; i < 3; i++)
      {
        out_publisher.msg_.r_out[i] = r_pos[i];
        out_publisher.msg_.r_d[i] = rd_pos[i];
      }
      out_publisher.msg_.traj_[0] = refx[0];
      out_publisher.msg_.traj_[1] = refx[1];
      out_publisher.msg_.traj_[2] = refx[2];
      out_publisher.msg_.traj_[3] = F_d[2];

      out_publisher.unlockAndPublish();
    }

  }

  Eigen::Matrix<double, 7, 1> ARCmodel_reaching::saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1> &tau_d_calculated,
      const Eigen::Matrix<double, 7, 1> &tau_J_d)
  { // NOLINT (readability-identifier-naming)
    Eigen::Matrix<double, 7, 1> tau_d_saturated{};
    for (size_t i = 0; i < 7; i++)
    {
      double difference = tau_d_calculated[i] - tau_J_d[i];
      tau_d_saturated[i] = tau_J_d[i] + max(min(difference, delta_tau_max_), -delta_tau_max_);
      if (i==0 || i==1 || i==2 || i==3)
      {
        /* code */
        tau_d_saturated[i] = max(min(tau_d_saturated[i], 80.0), -80.0);
      }

      if (i==4 || i==5 || i==6)
      {
        tau_d_saturated[i] = max(min(tau_d_saturated[i], 10.0), -10.0);
      }
      
    }
    return tau_d_saturated;
  }

  Eigen::Matrix<double, 6, 7> ARCmodel_reaching::Jacobian_diff(
    const Eigen::Matrix<double, 6, 7> &jaco_0, const Eigen::Matrix<double, 6, 7> &jaco_1, const double sample)
    {
      Eigen::Matrix<double, 6, 7> jaco_diff;
      for (size_t i = 0; i < 6; i++)
      {
        for (size_t j = 0; j < 7; j++)
        {
          /* code */
          jaco_diff(i,j) = (jaco_1(i,j) - jaco_0(i,j)) / sample;
        }
      }
      return jaco_diff;
    }
  
 
  void ARCmodel_reaching::ftsensorCallback(const geometry_msgs::WrenchStamped& msg)
  {
      F_ft[0] = msg.wrench.force.x;
      F_ft[1] = msg.wrench.force.y;
      F_ft[2] = msg.wrench.force.z;
      F_ft[3] = msg.wrench.torque.x;
      F_ft[4] = msg.wrench.torque.y;
      F_ft[5] = msg.wrench.torque.z;

  }


  // void ARCmodel_reaching::vsmrcParamCallback(
  //     franka_example_controllers::vsmrc_paramConfig &config,
  //     uint32_t level)
  // {
  //   A << config.A * Eigen::Matrix<double, 6, 6>::Identity();
  //   F_1 << config.F_1 * Eigen::Matrix<double, 6, 6>::Identity();
  //   F_2 << config.F_2 * Eigen::Matrix<double, 6, 6>::Identity();
  //   Ks << config.Ks * Eigen::Matrix<double, 6, 6>::Identity();

  //   //set initial M D K Ks matrix
  //   K_d.setIdentity();
  //   K_d.topLeftCorner(3, 3) << config.translational_stiffness * Eigen::Matrix3d::Identity();
  //   K_d.bottomRightCorner(3, 3) << config.rotational_stiffness * Eigen::Matrix3d::Identity();

  //   M_d.setIdentity();
  //   M_d.topLeftCorner(3, 3) << config.M_d_tran * Eigen::Matrix3d::Identity();
  //   M_d.bottomRightCorner(3, 3) << config.M_d_rota * Eigen::Matrix3d::Identity();
  //   // Damping ratio = 1
  //   D_d.topLeftCorner(3, 3)
  //       << 2.0 * sqrt(config.translational_stiffness * config.M_d_tran) * Eigen::Matrix3d::Identity();
  //   D_d.bottomRightCorner(3, 3)
  //       << 2.0 * sqrt(config.rotational_stiffness * config.M_d_rota) * Eigen::Matrix3d::Identity();
  //   K_f.setIdentity();
  //   K_f.topLeftCorner(3, 3) << config.xyz_force_gain * Eigen::Matrix3d::Identity();
  //   K_f.bottomRightCorner(3, 3) << config.rpy_torque_gain * Eigen::Matrix3d::Identity();
  // }

  // namespace end
} // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::ARCmodel_reaching,
                       controller_interface::ControllerBase)