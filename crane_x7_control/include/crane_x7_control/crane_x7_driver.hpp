#ifndef CRANE_X7_CONTROL__CRANE_X7_DRIVER_HPP_
#define CRANE_X7_CONTROL__CRANE_X7_DRIVER_HPP_

#include <dynamixel_sdk/dynamixel_sdk.h>
#include <memory>
#include <string>
#include <vector>

class CraneX7Driver {
public:
  CraneX7Driver(const std::string& port_name, const int baudrate, 
                const std::vector<uint8_t>& id_list);
  ~CraneX7Driver();

  bool open_port(void);
  void close_port(void);
  std::string get_last_error_log(void);

  bool torque_enable(const bool enable);
  bool write_goal_joint_positions(const std::vector<double> & goal_positions);
  bool write_goal_speed_rpm(const uint8_t dxl_id, const double speed_rpm);
  bool write_goal_speed_rpm_all(const double speed_rpm);
  bool write_goal_effort(const uint8_t dxl_id, const double effort);
  bool read_present_joint_positions(std::vector<double>& joint_positions);
  bool read_present_joint_speeds(std::vector<double>& joint_speeds);
  bool read_present_joint_currents(std::vector<double>& joint_loads);
  bool read_present_joint_voltages(std::vector<double>& joint_voltages);
  bool read_present_joint_temperatures(std::vector<double>& joint_temperatures);

private:
  std::shared_ptr<dynamixel::PortHandler> dxl_port_handler_;
  std::shared_ptr<dynamixel::PacketHandler> dxl_packet_handler_;
  int baudrate_;
  std::vector<uint8_t> id_list_;
  std::string last_error_log_;

  bool read_byte_data_from_each_joints(const uint16_t address, std::vector<uint8_t> & buffer);
  bool read_word_data_from_each_joints(const uint16_t address, std::vector<uint16_t> & buffer);
  bool parse_dxl_error(
    const std::string func_name, const uint8_t dxl_id,
    const int dxl_comm_result, const uint8_t dxl_packet_error);
  double dxl_pos_to_radian(const uint16_t position);
  uint16_t radian_to_dxl_pos(const double position);
  double dxl_speed_to_rps(const uint16_t speed);
  double dxl_load_to_percent(const uint16_t load);
  double dxl_voltage_to_actual_voltage(const uint8_t voltage);
};

#endif  // CRANE_X7_CONTROL__CRANE_X7_DRIVER_HPP_