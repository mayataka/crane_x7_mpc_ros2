#include "crane_x7_mpc/crane_x7_mpc.hpp"

#include <chrono>


namespace crane_x7_mpc 
{

CraneX7MPC::CraneX7MPC() 
  : Node("CraneX7MPC"),
    ocp_solver_(),
    N_(20), 
    nthreads_(4), 
    niter_(2),
    T_(0.5), 
    dt_(T_/N_),
    robot_(),
    end_effector_frame_(26),
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
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);
  using namespace std::chrono_literals;
  while (!parameters_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      rclcpp::shutdown();
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }
  T_ = parameters_client->get_parameter("T", 0.5);
  N_ = parameters_client->get_parameter("N", 20);
  dt_ = parameters_client->get_parameter("dt", (T_/N_));
  nthreads_ = parameters_client->get_parameter("nthreads", 4);
  niter_ = parameters_client->get_parameter("niter", 2);
  barrier_ = parameters_client->get_parameter("barrier", 0.1);

  ocp_solver_ = idocp::UnconstrOCPSolver(robot_, cost_, constraints_, 
                                         T_, N_, nthreads_);
  init(barrier_, 0);

  // Create the state feedback controller
  auto mpc_callback = [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
    const auto t_cl = std::chrono::system_clock::now();
    const double t = 1e-06 * std::chrono::duration_cast<std::chrono::microseconds>(
        t_cl.time_since_epoch()).count();
    for (int i=0; i<7; ++i) {
      q_.coeffRef(i) = msg->position[i];
      v_.coeffRef(i) = msg->velocity[i];
    }
    for (int i=0; i<niter_; ++i) {
      ocp_solver_.updateSolution(t, q_, v_);
    }
    const Eigen::VectorXd& a_opt = ocp_solver_.getSolution(0).a;
    for (int i=0; i<7; ++i) {
      command_message_.data[i] = v_.coeff(i) + dt_*a_opt.coeff(i);
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
  command_message_.layout.dim[0].label = "velocity_commands";
  command_message_.data.clear();
  for (int i=0; i<7; ++i) { command_message_.data.push_back(0.0); }

  // Create the service to enable/disable costs
  auto cost_3d_service = [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    if (request->data) ref_3d_->activate();
    else ref_3d_->deactivate();
    response->success = true;
  };
  auto cost_6d_service = [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    if (request->data) ref_6d_->activate();
    else ref_6d_->deactivate();
    response->success = true;
  };
  enable_3d_ref_ = this->create_service<std_srvs::srv::SetBool>("enable_3d_ref", cost_3d_service);
  enable_6d_ref_ = this->create_service<std_srvs::srv::SetBool>("enable_6d_ref", cost_6d_service);
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
    const double t = 1e-06 * std::chrono::duration_cast<std::chrono::microseconds>(
        t_cl.time_since_epoch()).count();
    for (int i=0; i<iter; ++i) {
      ocp_solver_.updateSolution(t, q_, v_);
    }
  }
}


void CraneX7MPC::create_cost() {
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);
  using namespace std::chrono_literals;
  while (!parameters_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      rclcpp::shutdown();
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }
  cost_ = std::make_shared<idocp::CostFunction>();
  config_cost_ = std::make_shared<idocp::ConfigurationSpaceCost>(robot_);
  config_cost_->set_q_weight(Eigen::VectorXd::Constant(
      robot_.dimv(), parameters_client->get_parameter("config_cost.q_weight", 0.1)));
  config_cost_->set_v_weight(Eigen::VectorXd::Constant(
      robot_.dimv(), parameters_client->get_parameter("config_cost.v_weight", 0.001)));
  config_cost_->set_a_weight(Eigen::VectorXd::Constant(
      robot_.dimv(), parameters_client->get_parameter("config_cost.a_weight", 0.001)));
  config_cost_->set_u_weight(Eigen::VectorXd::Constant(
      robot_.dimv(), parameters_client->get_parameter("config_cost.u_weight", 0.0)));
  config_cost_->set_qf_weight(Eigen::VectorXd::Constant(
      robot_.dimv(), parameters_client->get_parameter("config_cost.qf_weight", 0.1)));
  config_cost_->set_vf_weight(Eigen::VectorXd::Constant(
      robot_.dimv(), parameters_client->get_parameter("config_cost.vf_weight", 0.001)));
  ref_3d_ = std::make_shared<TimeVaryingTaskSpace3DRef>();
  task_cost_3d_ = std::make_shared<idocp::TimeVaryingTaskSpace3DCost>(robot_, end_effector_frame_, ref_3d_);
  task_cost_3d_->set_q_weight(Eigen::Vector3d::Constant(
      parameters_client->get_parameter("task_3d_cost.q_weight", 1000)));
  task_cost_3d_->set_qf_weight(Eigen::Vector3d::Constant(
      parameters_client->get_parameter("task_3d_cost.qf_weight", 1000)));
  ref_6d_ = std::make_shared<TimeVaryingTaskSpace6DRef>();
  task_cost_6d_ = std::make_shared<idocp::TimeVaryingTaskSpace6DCost>(robot_, end_effector_frame_, ref_6d_);
  task_cost_6d_->set_q_weight(
      Eigen::Vector3d::Constant(parameters_client->get_parameter("config_cost.q_trans_weight", 1000)), 
      Eigen::Vector3d::Constant(parameters_client->get_parameter("config_cost.q_rot_weight", 1000)));
  task_cost_6d_->set_qf_weight(
      Eigen::Vector3d::Constant(parameters_client->get_parameter("config_cost.qf_trans_weight", 1000)), 
      Eigen::Vector3d::Constant(parameters_client->get_parameter("config_cost.qf_rot_weight", 1000)));
  cost_->push_back(config_cost_);
  cost_->push_back(task_cost_3d_);
  cost_->push_back(task_cost_6d_);

  ref_3d_->deactivate();
  ref_6d_->deactivate();
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