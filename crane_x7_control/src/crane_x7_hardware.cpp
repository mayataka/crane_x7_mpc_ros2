#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "crane_x7_control/crane_x7_hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"


namespace crane_x7_control {

CraneX7Hardware::CraneX7Hardware(const ControlMode control_mode)
  : hardware_interface::BaseInterface<hardware_interface::SystemInterface>(),
    control_mode_(control_mode) {
}


CraneX7Hardware::~CraneX7Hardware() {
  driver_->torque_enable(false);
  driver_->close_port();
}


return_type CraneX7Hardware::configure(const hardware_interface::HardwareInfo& info) {
  if (configure_default(info) != return_type::OK) {
    return return_type::ERROR;
  }
  // Get parameters from URDF
  // Initialize member variables
  std::string port_name = info_.hardware_parameters["port_name"];
  int baudrate = std::stoi(info_.hardware_parameters["baudrate"]);
  timeout_seconds_ = std::stod(info_.hardware_parameters["timeout_seconds"]);
  read_velocities_ = std::stoi(info_.hardware_parameters["read_velocities"]);
  read_currents_ = std::stoi(info_.hardware_parameters["read_currents"]);
  read_voltages_ = std::stoi(info_.hardware_parameters["read_voltages"]);
  read_temperatures_ = std::stoi(info_.hardware_parameters["read_temperatures"]);

  std::vector<uint8_t> dxl_id_list;
  for (auto joint : info_.joints) {
    if (joint.parameters["dxl_id"] != "") {
      dxl_id_list.push_back(std::stoi(joint.parameters["dxl_id"]));
    } else {
      RCLCPP_ERROR(
        rclcpp::get_logger("CraneX7Hardware"),
        "Joint '%s' does not have 'dxl_id' parameter.",
        joint.name.c_str());
      return return_type::ERROR;
    }
  }
  hw_position_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocity_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_effort_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_position_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocity_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_voltage_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_temperature_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  // Open a crane_x7_driver
  driver_ = std::make_shared<CraneX7Driver>(port_name, baudrate, dxl_id_list);
  if (!driver_->open_port()) {
    RCLCPP_ERROR(rclcpp::get_logger("CraneX7Hardware"), driver_->get_last_error_log());
    return return_type::ERROR;
  }
  if (!driver_->torque_enable(false)) {
    RCLCPP_ERROR(rclcpp::get_logger("CraneX7Hardware"), driver_->get_last_error_log());
    return return_type::ERROR;
  }
  // Verify that the interface required by CraneX7Hardware is set in the URDF.
  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_ERROR(rclcpp::get_logger("CraneX7Hardware"),
                   "Joint '%s' has %d command interfaces found. 1 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return return_type::ERROR;
    }
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_ERROR(rclcpp::get_logger("CraneX7Hardware"),
                   "Joint '%s' have %s command interfaces found. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_POSITION);
      return return_type::ERROR;
    }
  }
  steady_clock_ = rclcpp::Clock(RCL_STEADY_TIME);
  status_ = hardware_interface::status::CONFIGURED;
  return return_type::OK;
}


std::vector<hardware_interface::StateInterface>
CraneX7Hardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i=0; i<info_.joints.size(); ++i) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(info_.joints[i].name, 
                                         hardware_interface::HW_IF_POSITION,
                                         &hw_position_states_[i])
    );
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(info_.joints[i].name, 
                                         hardware_interface::HW_IF_VELOCITY,
                                         &hw_velocity_states_[i])
    );
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(info_.joints[i].name, "current",
                                         &hw_current_states_[i])
    );
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(info_.joints[i].name, "voltage",
                                         &hw_voltage_states_[i])
    );
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(info_.joints[i].name, "temperature",
                                         &hw_temperature_states_[i])
    );
  }
  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface>
CraneX7Hardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  if (control_mode_ == ControlMode::Position) {
    for (uint i=0; i<info_.joints.size(); ++i) {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(info_.joints[i].name, 
                                            hardware_interface::HW_IF_POSITION, 
                                            &hw_position_commands_[i])
      );
    }
  }
  else if (control_mode_ == ControlMode::Velocity) {
    for (uint i=0; i<info_.joints.size(); ++i) {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(info_.joints[i].name, 
                                            hardware_interface::HW_IF_VELOCITY, 
                                            &hw_velocity_commands_[i])
      );
    }
  }
  else {
    for (uint i=0; i<info_.joints.size(); ++i) {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(info_.joints[i].name, 
                                            hardware_interface::HW_IF_EFFORT, 
                                            &hw_velocity_commands_[i])
      );
    }
  }
  return command_interfaces;
}


return_type CraneX7Hardware::start() {
  if (!driver_->torque_enable(false)) {
    RCLCPP_ERROR(rclcpp::get_logger("CraneX7Hardware"),
                 driver_->get_last_error_log());
    return return_type::ERROR;
  }
  // Set current timestamp to disable the communication timeout.
  prev_comm_timestamp_ = steady_clock_.now();
  timeout_has_printed_ = false;
  // Set current joint positions to hw_position_commands.
  read();
  if (control_mode_ == ControlMode::Position) {
    for (uint i=0; i<hw_position_commands_.size(); ++i) {
      hw_position_commands_[i] = hw_position_states_[i];
    }
  }
  else if (control_mode_ == ControlMode::Velocity) {
    for (uint i=0; i<hw_velocity_commands_.size(); ++i) {
      hw_velocity_commands_[i] = hw_velocity_states_[i];
    }
  }
  else {
    for (uint i=0; i<hw_effort_commands_.size(); ++i) {
      hw_effort_commands_[i] = hw_effort_states_[i];
    }
  }
  status_ = hardware_interface::status::STARTED;
  return return_type::OK;
}


return_type CraneX7Hardware::stop() {
  driver_->torque_enable(false);
  status_ = hardware_interface::status::STOPPED;
  return return_type::OK;
}


return_type CraneX7Hardware::read() {
  if (communication_timeout()) {
    if (!timeout_has_printed_) {
      RCLCPP_ERROR(rclcpp::get_logger("CraneX7Hardware"), "Communication timeout!");
      timeout_has_printed_ = true;
    }
    return return_type::ERROR;
  }

  std::vector<double> joint_positions;
  if (!driver_->read_present_joint_positions(joint_positions)) {
    RCLCPP_ERROR(rclcpp::get_logger("CraneX7Hardware"),
                 driver_->get_last_error_log());
    return return_type::ERROR;
  } 
  else {
    for (uint i=0; i<hw_position_states_.size(); ++i) {
      hw_position_states_[i] = joint_positions[i];
    }
  }
  // Reading joint speeds, currents, voltages or temperatures
  // causes a decrease of the communication rate.
  if (read_velocities_) {
    std::vector<double> joint_speeds;
    if (driver_->read_present_joint_speeds(joint_speeds)) {
      for (uint i=0; i<hw_velocity_states_.size(); ++i) {
        hw_velocity_states_[i] = joint_speeds[i];
      }
    }
  }
  if (read_currents_) {
    std::vector<double> joint_currents;
    if (driver_->read_present_joint_currents(joint_currents)) {
      for (uint i=0; i<hw_current_states_.size(); ++i) {
        hw_current_states_[i] = joint_currents[i];
      }
    }
  }
  if (read_voltages_) {
    std::vector<double> joint_voltages;
    if (driver_->read_present_joint_voltages(joint_voltages)) {
      for (uint i=0; i<hw_voltage_states_.size(); ++i) {
        hw_voltage_states_[i] = joint_voltages[i];
      }
    }
  }
  if (read_temperatures_) {
    std::vector<double> joint_temperatures;
    if (driver_->read_present_joint_temperatures(joint_temperatures)) {
      for (uint i=0; i<hw_temperature_states_.size(); ++i) {
        hw_temperature_states_[i] = joint_temperatures[i];
      }
    }
  }
  prev_comm_timestamp_ = steady_clock_.now();
  return return_type::OK;
}


return_type CraneX7Hardware::write() {
  if (communication_timeout()) {
    if (!timeout_has_printed_) {
      RCLCPP_ERROR(
        rclcpp::get_logger("CraneX7Hardware"), "Communication timeout!");
      timeout_has_printed_ = true;
    }
    return return_type::ERROR;
  }

  if (!driver_->write_goal_joint_positions(hw_position_commands_)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("CraneX7Hardware"),
      driver_->get_last_error_log());
    return return_type::ERROR;
  }

  prev_comm_timestamp_ = steady_clock_.now();
  return return_type::OK;
}


bool CraneX7Hardware::communication_timeout() {
  if (steady_clock_.now().seconds()-prev_comm_timestamp_.seconds() 
        >= timeout_seconds_) {
    return true;
  } 
  else {
    return false;
  }
}

}  // namespace crane_x7_control


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  crane_x7_control::CraneX7Hardware,
  hardware_interface::SystemInterface
)