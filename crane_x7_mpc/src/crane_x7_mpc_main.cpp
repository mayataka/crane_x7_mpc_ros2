#include <rclcpp/rclcpp.hpp>

#include "crane_x7_mpc/crane_x7_mpc.hpp"


int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<crane_x7_mpc::CraneX7MPC>());
  rclcpp::shutdown();
  return 0;
}
