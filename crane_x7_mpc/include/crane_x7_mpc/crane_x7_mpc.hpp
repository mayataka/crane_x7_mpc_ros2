#ifndef CRANE_X7_MPC__CRANE_X7_MPC_HPP_
#define CRANE_X7_MPC__CRANE_X7_MPC_HPP_

#include <string>
#include <memory>
#include <cmath>

#include "Eigen/Core"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "idocp/solver/unconstr_ocp_solver.hpp"
#include "idocp/cost/cost_function.hpp"
#include "idocp/cost/configuration_space_cost.hpp"
#include "idocp/cost/time_varying_task_space_3d_cost.hpp"
#include "idocp/cost/time_varying_task_space_6d_cost.hpp"
#include "idocp/constraints/constraints.hpp"
#include "idocp/constraints/joint_position_lower_limit.hpp"
#include "idocp/constraints/joint_position_upper_limit.hpp"
#include "idocp/constraints/joint_velocity_lower_limit.hpp"
#include "idocp/constraints/joint_velocity_upper_limit.hpp"
#include "idocp/constraints/joint_torques_lower_limit.hpp"
#include "idocp/constraints/joint_torques_upper_limit.hpp"

#include "crane_x7_mpc/visiblity_control.h"


namespace crane_x7_mpc 
{

class TimeVaryingTaskSpace3DRef final : public idocp::TimeVaryingTaskSpace3DRefBase {
public:
  TimeVaryingTaskSpace3DRef() 
    : TimeVaryingTaskSpace3DRefBase() {
    pos0_ << 0.546, 0, 0.76;
    radius_ = 0.05;
    is_active_ = false;
  }

  ~TimeVaryingTaskSpace3DRef() {}

  void update_q_3d_ref(const double t, Eigen::VectorXd& q_3d_ref) const override {
    q_3d_ref = pos0_;
    q_3d_ref.coeffRef(1) += radius_ * sin(M_PI*t);
    q_3d_ref.coeffRef(2) += radius_ * cos(M_PI*t);
  }

  bool isActive(const double t) const override {
    return is_active_;
  }

  void activate() {
    is_active_ = true;
  }

  void deactivate() {
    is_active_ = false;
  }

private:
  double radius_;
  Eigen::Vector3d pos0_;
  bool is_active_;
};


class TimeVaryingTaskSpace6DRef final : public idocp::TimeVaryingTaskSpace6DRefBase {
public:
  TimeVaryingTaskSpace6DRef() 
    : TimeVaryingTaskSpace6DRefBase() {
    rotm_  <<  0, 0, 1, 
               0, 1, 0,
              -1, 0, 0;
    pos0_ << 0.546, 0, 0.76;
    radius_ = 0.05;
    is_active_ = false;
  }

  ~TimeVaryingTaskSpace6DRef() {}

  void update_SE3_ref(const double t, pinocchio::SE3& SE3_ref) const override {
    Eigen::Vector3d pos(pos0_);
    pos.coeffRef(1) += radius_ * sin(M_PI*t);
    pos.coeffRef(2) += radius_ * cos(M_PI*t);
    SE3_ref = pinocchio::SE3(rotm_, pos);
  }

  bool isActive(const double t) const override {
    return is_active_;
  }

  void activate() {
    is_active_ = true;
  }

  void deactivate() {
    is_active_ = false;
  }

private:
  double radius_;
  Eigen::Matrix3d rotm_;
  Eigen::Vector3d pos0_;
  bool is_active_;
};


class CraneX7MPC : public rclcpp::Node 
{
public:
  CRANE_X7_MPC_PUBLIC 
  CraneX7MPC();

  CRANE_X7_MPC_PUBLIC 
  ~CraneX7MPC();

  CRANE_X7_MPC_PUBLIC 
  void init(const double barrier=1.0e-01, const int iter=0);

private:
  // OCP solver 
  idocp::UnconstrOCPSolver ocp_solver_;
  int N_, nthreads_, niter_; 
  double T_, dt_, barrier_;
  idocp::Robot robot_;
  // Cost function
  int end_effector_frame_;
  std::shared_ptr<idocp::CostFunction> cost_;
  std::shared_ptr<idocp::ConfigurationSpaceCost> config_cost_;
  std::shared_ptr<idocp::TimeVaryingTaskSpace3DCost> task_cost_3d_;
  std::shared_ptr<idocp::TimeVaryingTaskSpace6DCost> task_cost_6d_;
  std::shared_ptr<TimeVaryingTaskSpace3DRef> ref_3d_;
  std::shared_ptr<TimeVaryingTaskSpace6DRef> ref_6d_;
  // Constraints
  std::shared_ptr<idocp::Constraints> constraints_;
  std::shared_ptr<idocp::JointPositionLowerLimit> joint_position_lower_limit_;
  std::shared_ptr<idocp::JointPositionUpperLimit> joint_position_upper_limit_;
  std::shared_ptr<idocp::JointVelocityLowerLimit> joint_velocity_lower_limit_;
  std::shared_ptr<idocp::JointVelocityUpperLimit> joint_velocity_upper_limit_;
  std::shared_ptr<idocp::JointTorquesLowerLimit> joint_torques_lower_limit_;
  std::shared_ptr<idocp::JointTorquesUpperLimit> joint_torques_upper_limit_;
  // Subscriber and publisher for state-feedback control
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::JointState>> joint_state_subscriber_;
  std_msgs::msg::Float64MultiArray command_message_;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> joint_command_publisher_;
  Eigen::VectorXd q_, v_, a_;

  void create_cost();
  void create_constraints();

};

} // namespace crane_x7_mpc 

#endif  // CRANE_X7_MPC__CRANE_X7_MPC_HPP_