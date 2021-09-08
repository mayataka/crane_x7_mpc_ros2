#include "crane_x7_mpc/crane_x7_mpc.hpp"

#include <chrono>


namespace crane_x7_mpc 
{

CraneX7MPC::CraneX7MPC(const std::string& path_to_urdf) :
    Node("CraneX7MPC") {
  // Create the OCP solver
  robot_ = idocp::Robot(path_to_urdf);
  end_effector_frame_ = 18; 
  create_cost();
  create_constraints();
  N_ = 20;
  nthreads_ = 4;
  iter_ = 2;
  T_ = 0.5;
  dt_ = T_ / N_;
  ocp_solver_ = idocp::UnconstrOCPSolver(robot_, cost_, constraints_, 
                                         T_, N_, nthreads_);
  // Create the state feedback controller
  auto mpc_callback = [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
    const auto t0_ch = std::chrono::system_clock::now();
    const double t0 = 1e-03 * std::chrono::duration_cast<std::chrono::microseconds>(
        t0_ch.time_since_epoch()).count();
    for (int i=0; i<7; ++i) {
      q_.coeff(i) = msg->position[i];
      v_.coeff(i) = msg->velocity[i];
    }
    for (int i=0; i<iter_; ++i) {
      ocp_solver_.updateSolution(t0, q_, v_);
    }
    const Eigen::VectorXd& a_opt = ocp_solver_.getSolution(0).a;
    const auto t1_ch = std::chrono::system_clock::now();
    const double dt = 1e-03 * std::chrono::duration_cast<std::chrono::microseconds>(
        t1_ch-t0_ch).count(); 
    for (int i=0; i<7; ++i) {
      command_message_.data[i] = v_.coeff(i) + dt*a_opt.coeff(i);
    }
    joint_command_publisher_->publish(command_message_);
  };
  joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "controller_manager/joint_state_broadcaster/joint_states",
    rclcpp::QoS(10), mpc_callback);
  joint_command_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/joint_velocity_controller/commands", 10);
  double memv[7];
  for (int i=0; i<7; ++i) { memv[i] = 0.0; }
  command_message_.data = msmv;
}


CraneX7MPC::~CraneX7MPC() 
{
}


void CraneX7MPC::init(const double t, const Eigen::VectorXd& q, 
                      const Eigen::VectorXd& v, const double barrier, 
                      const int iter) 
{
  ocp_solver_.setSolution("q", q);
  ocp_solver_.setSolution("v", v);
  ocp_solver_.initConstraints();
  for (int i=0; i<iter; ++i) {
    ocp_solver_.updateSolution(t, q, v);
  }
}


void CraneX7MPC::create_cost() {
  cost_ = std::make_shared<idocp::CostFunction>();
  config_cost_ = std::make_shared<idocp::ConfigurationSpaceCost>(robot_);
  config_cost_->set_q_weight(Eigen::VectorXd::Constant(robot_.dimv(), 0.1));
  config_cost_->set_qf_weight(Eigen::VectorXd::Constant(robot_.dimv(), 0.1));
  config_cost_->set_v_weight(Eigen::VectorXd::Constant(robot_.dimv(), 0.0001));
  config_cost_->set_vf_weight(Eigen::VectorXd::Constant(robot_.dimv(), 0.0001));
  config_cost_->set_a_weight(Eigen::VectorXd::Constant(robot_.dimv(), 0.0001));
  3d_ref_ = std::make_shared<TimeVaryingTaskSpace3DRef>();
  3d_ref_->deactivate();
  3d_task_cost_ = std::make_shared<idocp::TimeVaryingTaskSpace6DCost>(robot_, end_effector_frame_ 3d_ref_);
  3d_task_cost->set_q_weight(Eigen::Vector3d::Constant(1000));
  3d_task_cost->set_qf_weight(Eigen::Vector3d::Constant(1000));
  6d_ref_ = std::make_shared<TimeVaryingTaskSpace6DRef>();
  6d_ref_->deactivate();
  6d_task_cost_ = std::make_shared<idocp::TimeVaryingTaskSpace6DCost>(robot_, end_effector_frame_, 6d_ref_);
  6d_task_cost->set_q_weight(Eigen::Vector3d::Constant(1000), Eigen::Vector3d::Constant(1000));
  6d_task_cost->set_qf_weight(Eigen::Vector3d::Constant(1000), Eigen::Vector3d::Constant(1000));
  cost_->push_back(config_cost_);
  cost_->push_back(3d_task_cost_);
  cost_->push_back(6d_task_cost_);
}


void CraneX7MPC::create_constraints() {
  constraints_                = std::make_shared<idocp::Constraints>();
  joint_position_lower_limit_ = std::make_shared<idocp::JointPositionLowerLimit>(robot_);
  joint_position_upper_limit_ = std::make_shared<idocp::JointPositionUpperLimit>(robot_);
  joint_velocity_lower_limit_ = std::make_shared<idocp::JointVelocityLowerLimit>(robot_);
  joint_velocity_upper_limit_ = std::make_shared<idocp::JointVelocityUpperLimit>(robot_);
  joint_torques_lower_limit_  = std::make_shared<idocp::JointTorquesLowerLimit>(robot_);
  joint_torques_upper_limit_  = std::make_shared<idocp::JointTorquesUpperLimit>(robot_);
  constraints_->push_back(joint_position_lower_limit_);
  constraints_->push_back(joint_position_upper_limit_);
  constraints_->push_back(joint_velocity_lower_limit_);
  constraints_->push_back(joint_velocity_upper_limit_);
  constraints_->push_back(joint_torques_lower_limit_);
  constraints_->push_back(joint_torques_upper_limit_);
}

} // namespace crane_x7_mpc 