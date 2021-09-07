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

  static constexpr int OPE_CURRENT_MODE  = 0;
  static constexpr int OPE_VELOCITY_MODE = 1;
  static constexpr int OPE_POSITION_MODE = 3;
  static constexpr int OPE_EXT_POS_MODE  = 4;
  static constexpr int OPE_CURR_POS_MODE = 5;
  static constexpr int OPE_PWM_MODE      = 16;

  static constexpr std::string KEY_DXL_PORT       = "dynamixel_port";
  static constexpr std::string KEY_PORTNAME       = "/port_name";
  static constexpr std::string KEY_BAUDRATE       = "/baud_rate";
  static constexpr std::string KEY_JOINTS         = "/joints";
  static constexpr std::string KEY_JPARAM_ID      = "/id";
  static constexpr std::string KEY_JPARAM_CENTER  = "/center";
  static constexpr std::string KEY_JPARAM_HOME    = "/home";
  static constexpr std::string KEY_JPARAM_EFFCNST = "/effort_const";
  static constexpr std::string KEY_JPARAM_OPEMODE = "/mode";

  static constexpr int DEFAULT_CENTER = 2048;
  static constexpr double DEFAULT_EFF_CNST = 1.0;
  static constexpr int DEFAULT_OPE_MODE = OPE_POSITION_MODE;

  static constexpr double PROTOCOL_VERSION = 2.0;
  static constexpr int DXL_HOME_POSITION = 511;  // value range:0 ~ 1023
  static constexpr double DXL_MAX_POSITION = 1023.0;
  static constexpr double DXL_MAX_POSITION_DEGREES = 300.0;
  static constexpr double TO_RADIANS = (DXL_MAX_POSITION_DEGREES / DXL_MAX_POSITION) * M_PI / 180.0;
  static constexpr double TO_DXL_POS = 1.0 / TO_RADIANS;
  static constexpr double TO_SPEED_REV_PER_MIN = 0.111;
  static constexpr double TO_SPEED_RAD_PER_MIN = TO_SPEED_REV_PER_MIN * 2.0 * M_PI;
  static constexpr double TO_SPEED_RAD_PER_SEC = TO_SPEED_RAD_PER_MIN / 60.0;
  static constexpr double TO_LOAD_PERCENT = 0.1;
  static constexpr double TO_VOLTAGE = 0.1;

  // Dynamixel AX-12A address table
  // Ref: https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/ and https://emanual.robotis.com/docs/en/dxl/x/xm540-w270/ 
  static constexpr uint16_t ADDR_TORQUE_ENABLE = 64;
  static constexpr uint16_t ADDR_GOAL_POSITION = 116;
  static constexpr uint16_t ADDR_PRESENT_POSITION = 132;
  static constexpr uint16_t ADDR_PRESENT_VELOCITY = 128;
  static constexpr uint16_t ADDR_PRESENT_CURRENT = 126;
  static constexpr uint16_t ADDR_PRESENT_INPUT_VOLTAGE = 144;
  static constexpr uint16_t ADDR_PRESENT_TEMPERATURE = 146;

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
  double dxl_effort_to_current(const uint16_t effort);
  double dxl_voltage_to_actual_voltage(const uint8_t voltage);
};

#endif  // CRANE_X7_CONTROL__CRANE_X7_DRIVER_HPP_