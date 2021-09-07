# CRANE_X7_MPC_ROS2

Build and run an example with Docker:
```
docker build -t crane_x7_mpc_ros2 .
docker run -it --rm --name crane_x7_mpc_ros2 --net host crane_x7_mpc_ros2 
```
You can visualize the simulator via another terminal
```
gzclient
```