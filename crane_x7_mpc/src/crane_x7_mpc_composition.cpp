#include <rclcpp/rclcpp.hpp>

#include "crane_x7_mpc/crane_x7_mpc.hpp"


int main(int argc, char * argv[]){
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  auto mpc = std::make_shared<crane_x7_mpc::CraneX7MPC>(options);
  exec.add_node(mpc);

  exec.spin();

  rclcpp::shutdown();
  return 0;
}
