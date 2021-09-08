#ifndef CRANE_X7_MPC__CRANE_X7_MPC_HPP_
#define CRANE_X7_MPC__CRANE_X7_MPC_HPP_

#include <string>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "idocp/solver/unconstr_ocp_solver.hpp"
#include "idocp/cost/cost_function.hpp"
#include "idocp/cost/configuration_space_cost.hpp"
#include "idocp/cost/time_varying_task_space_3d_cost.hpp"
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

class CraneX7MPC : public rclcpp::Node 
{
public:
  CRANE_X7_MPC_PUBLIC 
  CraneX7MPC(const std::string& path_to_urdf);

  CRANE_X7_MPC_PUBLIC 
  ~CraneX7MPC();

private:
  idocp::Robot robot_;
  std::shared_ptr<idocp::CostFunction> cost_;
  std::shared_ptr<idocp::ConfigurationSpaceCost> config_cost_;
  std::shared_ptr<idocp::TimeVaryingTaskSpace3DCost> task_cost_;

  std::shared_ptr<idocp::Constraints> constraints_;
  std::shared_ptr<idocp::JointPositionLowerLimit> joint_position_lower_limit_;
  std::shared_ptr<idocp::JointPositionUpperLimit> joint_position_upper_limit_;
  std::shared_ptr<idocp::JointVelocityLowerLimit> joint_velocity_lower_limit_;
  std::shared_ptr<idocp::JointVelocityUpperLimit> joint_velocity_upper_limit_;
  std::shared_ptr<idocp::JointTorquesLowerLimit> joint_torques_lower_limit_;
  std::shared_ptr<idocp::JointTorquesUpperLimit> joint_torques_upper_limit_;

};

} // namespace crane_x7_mpc 

#endif  // CRANE_X7_MPC__CRANE_X7_MPC_HPP_