#include "crane_x7_mpc/crane_x7_mpc.hpp"


namespace crane_x7_mpc 
{

CraneX7MPC::CraneX7MPC(const std::string& path_to_urdf) {
  robot_ = idocp::Robot(path_to_urdf);
  cost_ = std::make_shared<idocp::CostFunction>();
}

{
public:
  CRANE_X7_MPC_PUBLIC 
  CraneX7MPC();

} // namespace crane_x7_mpc 