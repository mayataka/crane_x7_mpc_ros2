#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "crane_x7_control/crane_x7_driver.hpp"


CraneX7Driver::CraneX7Driver(const std::string& port_name, const int baudrate,
                             const std::vector<uint8_t>& id_list)
: baudrate_(baudrate), 
  id_list_(id_list) {
  dxl_port_handler_ = std::shared_ptr<dynamixel::PortHandler>(
      dynamixel::PortHandler::getPortHandler(port_name.c_str()));
  dxl_packet_handler_ = std::shared_ptr<dynamixel::PacketHandler>(
      dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION));
}


CraneX7Driver::~CraneX7Driver() {
  close_port();
}


bool CraneX7Driver::open_port(void) {
  if (!dxl_port_handler_->openPort()) {
    last_error_log_ = std::string(__func__) 
                        + ": unable to open dynamixel port: " 
                        + dxl_port_handler_->getPortName();
    return false;
  }
  if (!dxl_port_handler_->setBaudRate(baudrate_)) {
    last_error_log_ = std::string(__func__) 
                        + ": unable to set baudrate" 
                        + std::to_string(dxl_port_handler_->getBaudRate());
    return false;
  }
  return true;
}


void CraneX7Driver::close_port(void) {
  dxl_port_handler_->closePort();
}


std::string CraneX7Driver::get_last_error_log(void) {
  return last_error_log_;
}


bool CraneX7Driver::torque_enable(const bool enable) {
  bool retval = true;
  for (auto dxl_id : id_list_) {
    uint8_t dxl_error = 0;
    const int dxl_result = dxl_packet_handler_->write1ByteTxRx(
        dxl_port_handler_.get(), 
        dxl_id, ADDR_TORQUE_ENABLE, enable, &dxl_error);
    if (!parse_dxl_error(std::string(__func__), dxl_id, dxl_result, dxl_error)) {
      retval = false;
    }
  }
  return retval;
}


bool CraneX7Driver::write_goal_joint_positions(const std::vector<double>& goal_positions) {
  bool retval = true;
  if (goal_positions.size() != id_list_.size()) {
    last_error_log_ = std::string(__func__) + ": vectors size does not match: " +
      " goal_positions:" + std::to_string(goal_positions.size()) +
      ", id_list:" + std::to_string(id_list_.size());
    return false;
  }
  for (size_t i=0; i<goal_positions.size(); i++) {
    const uint16_t goal_position = radian_to_dxl_pos(goal_positions[i]);
    const auto dxl_id = id_list_[i];
    uint8_t dxl_error = 0;
    int dxl_result = dxl_packet_handler_->write2ByteTxRx(
        dxl_port_handler_.get(),
        dxl_id, ADDR_GOAL_POSITION, goal_position, &dxl_error);
    if (!parse_dxl_error(std::string(__func__), dxl_id, dxl_result, dxl_error)) {
      retval = false;
    }
  }
  return retval;
}


bool CraneX7Driver::write_goal_speed_rpm(const std::vector<double>& goal_speed_rpm) {
  bool retval = true;
  if (goal_speed_rpm.size() != id_list_.size()) {
    last_error_log_ = std::string(__func__) + ": vectors size does not match: " +
      " goal_speed_rpm:" + std::to_string(goal_speed_rpm.size()) +
      ", id_list:" + std::to_string(id_list_.size());
    return false;
  }
  constexpr double SPEED_UNIT = 0.229; // rpm
  for (size_t i=0; i<goal_speed_rpm.size(); i++) {
    const int dxl_goal_speed = goal_speed_rpm[i] / SPEED_UNIT;
    const auto dxl_id = id_list_[i];
    uint8_t dxl_error = 0;
    int dxl_result = dxl_packet_handler_->write2ByteTxRx(
        dxl_port_handler_.get(),
        dxl_id, ADDR_GOAL_VELOCITY, dxl_goal_speed, &dxl_error);
    if (!parse_dxl_error(std::string(__func__), dxl_id, dxl_result, dxl_error)) {
      retval = false;
    }
  }
  return retval;
}


bool CraneX7Driver::write_goal_effort(const std::vector<double>& goal_effort) {
  bool retval = true;
  if (goal_effort.size() != id_list_.size()) {
    last_error_log_ = std::string(__func__) + ": vectors size does not match: " +
      " goal_effort:" + std::to_string(goal_effort.size()) +
      ", id_list:" + std::to_string(id_list_.size());
    return false;
  }
  for (size_t i=0; i<goal_effort.size(); i++) {
    const uint16_t dxl_goal_current = dxl_effort_to_current(goal_effort[i]);
    const auto dxl_id = id_list_[i];
    uint8_t dxl_error = 0;
    int dxl_result = dxl_packet_handler_->write2ByteTxRx(
        dxl_port_handler_.get(),
        dxl_id, ADDR_GOAL_CURRENT, dxl_goal_current, &dxl_error);
    if (!parse_dxl_error(std::string(__func__), dxl_id, dxl_result, dxl_error)) {
      retval = false;
    }
  }
  return retval;
}


bool CraneX7Driver::read_present_joint_positions(std::vector<double>& joint_positions) {
  std::vector<uint16_t> buffer;
  bool retval = read_word_data_from_each_joints(ADDR_PRESENT_POSITION, buffer);
  for (auto data : buffer) {
    joint_positions.push_back(dxl_pos_to_radian(data));
  }
  return retval;
}


bool CraneX7Driver::read_present_joint_speeds(std::vector<double> & joint_speeds) {
  std::vector<uint16_t> buffer;
  bool retval = read_word_data_from_each_joints(ADDR_PRESENT_VELOCITY, buffer);
  for (auto data : buffer) {
    joint_speeds.push_back(dxl_speed_to_rps(data));
  }
  return retval;
}


bool CraneX7Driver::read_present_joint_currents(std::vector<double>& joint_currents) {
  std::vector<uint16_t> buffer;
  bool retval = read_word_data_from_each_joints(ADDR_PRESENT_CURRENT, buffer);
  for (auto data : buffer) {
    joint_currents.push_back(data);
  }
  return retval;
}


bool CraneX7Driver::read_present_joint_voltages(std::vector<double>& joint_voltages) {
  std::vector<uint8_t> buffer;
  bool retval = read_byte_data_from_each_joints(ADDR_PRESENT_INPUT_VOLTAGE, buffer);
  for (auto data : buffer) {
    joint_voltages.push_back(dxl_voltage_to_actual_voltage(data));
  }
  return retval;
}


bool CraneX7Driver::read_present_joint_temperatures(std::vector<double>& joint_temperatures) {
  std::vector<uint8_t> buffer;
  bool retval = read_byte_data_from_each_joints(ADDR_PRESENT_TEMPERATURE, buffer);
  for (auto data : buffer) {
    joint_temperatures.push_back(data);
  }
  return retval;
}