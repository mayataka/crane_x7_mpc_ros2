#ifndef CRANE_X7_CONTROL__CRANE_X7_HARDWARE_HPP_
#define CRANE_X7_CONTROL__CRANE_X7_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "crane_x7_control/crane_x7_driver.hpp"
#include "crane_x7_control/visibility_control.h"
#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"


using hardware_interface::return_type;

namespace crane_x7_control {

enum class ControlMode {
  Position,
  Velocity,
  Effort
};

class CraneX7Hardware : public
  hardware_interface::BaseInterface<hardware_interface::SystemInterface> {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(CraneX7Hardware)

  CRANE_X7_CONTROL_PUBLIC
  CraneX7Hardware();

  CRANE_X7_CONTROL_PUBLIC
  ~CraneX7Hardware();

  CRANE_X7_CONTROL_PUBLIC
  return_type set_control_mode(const ControlMode control_mode);

  CRANE_X7_CONTROL_PUBLIC
  return_type configure(const hardware_interface::HardwareInfo& info) override;

  CRANE_X7_CONTROL_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  CRANE_X7_CONTROL_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CRANE_X7_CONTROL_PUBLIC
  return_type start() override;

  CRANE_X7_CONTROL_PUBLIC
  return_type stop() override;

  CRANE_X7_CONTROL_PUBLIC
  return_type read() override;

  CRANE_X7_CONTROL_PUBLIC
  return_type write() override;

private:
  bool communication_timeout();

  ControlMode control_mode_;
  std::shared_ptr<CraneX7Driver> driver_;
  double timeout_seconds_;
  bool read_velocities_, read_currents_, read_voltages_, read_temperatures_;
  std::vector<double> hw_position_commands_, hw_velocity_commands_, hw_effort_commands_,
                      hw_position_states_, hw_velocity_states_, hw_effort_states_,
                      hw_current_states_, hw_voltage_states_, hw_temperature_states_;
  rclcpp::Clock steady_clock_;
  rclcpp::Time prev_comm_timestamp_;
  bool timeout_has_printed_;
};

}  // namespace crane_x7_control

#endif  // CRANE_X7_CONTROL__CRANE_X7_HARDWARE_HPP_