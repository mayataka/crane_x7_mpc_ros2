#include "crane_x7_mpc/crane_x7_mpc.hpp"

#include <chrono>


namespace crane_x7_mpc 
{

CraneX7MPC::CraneX7MPC() 
  : Node("CraneX7MPC"),
    ocp_solver_(),
    N_(20), 
    nthreads_(4), 
    iter_(2),
    T_(0.5), 
    dt_(T_/N_),
    robot_(),
    end_effector_frame_(18),
    cost_(),
    config_cost_(),
    task_cost_3d_(),
    task_cost_6d_(),
    ref_3d_(),
    ref_6d_(),
    constraints_(),
    joint_position_lower_limit_(),
    joint_position_upper_limit_(),
    joint_velocity_lower_limit_(),
    joint_velocity_upper_limit_(),
    joint_torques_lower_limit_(),
    joint_torques_upper_limit_(),
    joint_state_subscriber_(),
    command_message_(),
    joint_command_publisher_(),
    q_(Eigen::VectorXd::Zero(7)), 
    v_(Eigen::VectorXd::Zero(7)), 
    a_(Eigen::VectorXd::Zero(7)) {
  // Create the MPC solver
  const std::string crane_x7_mpc_path 
      = ament_index_cpp::get_package_share_directory("crane_x7_mpc");
  const std::string path_to_urdf = crane_x7_mpc_path + "/" + "urdf/crane_x7.urdf";
  robot_ = idocp::Robot(path_to_urdf);
  create_cost();
  create_constraints();
  ocp_solver_ = idocp::UnconstrOCPSolver(robot_, cost_, constraints_, 
                                         T_, N_, nthreads_);
  init(1.0e-01, 0);

  // Create the state feedback controller
  auto mpc_callback = [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
    const auto t0_cl = std::chrono::system_clock::now();
    const double t0 = 1e-03 * std::chrono::duration_cast<std::chrono::microseconds>(
        t0_cl.time_since_epoch()).count();
    for (int i=0; i<7; ++i) {
      q_.coeffRef(i) = msg->position[i];
      v_.coeffRef(i) = msg->velocity[i];
    }
    for (int i=0; i<iter_; ++i) {
      ocp_solver_.updateSolution(t0, q_, v_);
    }
    const Eigen::VectorXd& a_opt = ocp_solver_.getSolution(0).a;
    const auto t1_cl = std::chrono::system_clock::now();
    const double dt = 1e-03 * std::chrono::duration_cast<std::chrono::microseconds>(
        t1_cl-t0_cl).count(); 
    for (int i=0; i<7; ++i) {
      command_message_.data[i] = v_.coeff(i) + dt*a_opt.coeff(i);
    }
    joint_command_publisher_->publish(command_message_);
  };
  joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states",
    rclcpp::QoS(10), mpc_callback);
  joint_command_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/joint_velocity_controller/commands", 10);

  // Init command msg
  command_message_.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  command_message_.layout.dim[0].size = 7;
  command_message_.layout.dim[0].stride = 7;
  command_message_.layout.dim[0].label = "velocity_command";
  command_message_.data.clear();
  for (int i=0; i<7; ++i) { command_message_.data.push_back(0.0); }
}


CraneX7MPC::~CraneX7MPC() 
{
}


void CraneX7MPC::init(const double barrier, const int iter) 
{
  ocp_solver_.setSolution("q", q_);
  ocp_solver_.setSolution("v", v_);
  constraints_->setBarrier(barrier);
  ocp_solver_.initConstraints();
  if (iter > 0) {
    const auto t_cl = std::chrono::system_clock::now();
    const double t = 1e-03 * std::chrono::duration_cast<std::chrono::microseconds>(
        t_cl.time_since_epoch()).count();
    for (int i=0; i<iter; ++i) {
      ocp_solver_.updateSolution(t, q_, v_);
    }
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
  ref_3d_ = std::make_shared<TimeVaryingTaskSpace3DRef>();
  ref_3d_->deactivate();
  task_cost_3d_ = std::make_shared<idocp::TimeVaryingTaskSpace3DCost>(robot_, end_effector_frame_, ref_3d_);
  task_cost_3d_->set_q_weight(Eigen::Vector3d::Constant(1000));
  task_cost_3d_->set_qf_weight(Eigen::Vector3d::Constant(1000));
  ref_6d_ = std::make_shared<TimeVaryingTaskSpace6DRef>();
  ref_6d_->deactivate();
  task_cost_6d_ = std::make_shared<idocp::TimeVaryingTaskSpace6DCost>(robot_, end_effector_frame_, ref_6d_);
  task_cost_6d_->set_q_weight(Eigen::Vector3d::Constant(1000), Eigen::Vector3d::Constant(1000));
  task_cost_6d_->set_qf_weight(Eigen::Vector3d::Constant(1000), Eigen::Vector3d::Constant(1000));
  cost_->push_back(config_cost_);
  cost_->push_back(task_cost_3d_);
  cost_->push_back(task_cost_6d_);

  // ref_3d_->activate();
  ref_6d_->activate();
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