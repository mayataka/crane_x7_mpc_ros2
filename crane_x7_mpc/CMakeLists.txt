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
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(idocp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(
  ${PROJECT_NAME} 
  src/crane_x7_mpc.cpp
  src/crane_x7_mpc_main.cpp
)
target_include_directories(
  ${PROJECT_NAME} 
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE
  ${EIGEN3_INCLUDE_DIR}
  ${IDOCP_INCLUDE_DIR}
  ${PINOCCHIO_INCLUDE_DIRS}
)
ament_target_dependencies(
  ${PROJECT_NAME} 
  ament_index_cpp
  Eigen3
  rclcpp
  std_msgs
  sensor_msgs
  std_srvs
)
target_link_libraries(
  ${PROJECT_NAME} 
  idocp::idocp
)

install(
  TARGETS ${PROJECT_NAME} 
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY
  urdf
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  # find_package(ament_cmake_gtest REQUIRED)
endif()

ament_package()