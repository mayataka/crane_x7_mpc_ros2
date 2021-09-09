cmake_minimum_required(VERSION 3.5.0)
project(crane_x7_mpc)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(NOT WIN32)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# find_package(Eigen3 REQUIRED)
# find_package(idocp REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(rclcpp_components REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(sensor_msgs REQUIRED)

# set(dependencies
#     Eigen3
#     idocp
#     rclcpp
#     rclcpp_components
#     std_msgs
#     sensor_msgs)

# set(CRANE_X7_MPC_LIB crane_x7_mpc)
# add_library(
#   ${CRANE_X7_MPC_LIB} 
#   SHARED
#   src/crane_x7_mpc.cpp
# )
# target_include_directories(
#   ${CRANE_X7_MPC_LIB}
#   PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )
# ament_target_dependencies(${CRANE_X7_MPC_LIB} ${dependencies})

# rclcpp_components_register_nodes(${CRANE_X7_MPC_LIB} "crane_x7_mpc::MPC")

# # Generate standalone node executable
# set(CRANE_X7_MPC_EXE "${CRANE_X7_MPC_LIB}_exe")
# add_executable(${CRANE_X7_MPC_EXE} src/crane_x7_mpc_main.cpp)
# target_link_libraries(${CRANE_X7_MPC_EXE} ${CRANE_X7_MPC_LIB})

# ament_export_targets(export_${CRANE_X7_MPC_LIB} HAS_LIBRARY_TARGET)
# ament_export_dependencies(${dependencies})


# install(
#   DIRECTORY include/
#   DESTINATION include
# )
# install(
#   TARGETS ${CRANE_X7_MPC_LIB} ${CRANE_X7_MPC_EXE}
#   EXPORT export_${PENDULUM_CONTROLLER_LIB}
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
#   INCLUDES DESTINATION include
# )

ament_package()