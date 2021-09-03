# CRANE_X7_MPC_ROS2

- build the Docker 
```
docker build -t crane_x7_mpc_ros2 .
```

- run Docker
```
docker run -it --rm --name crane_x7_mpc_ros2 --net host crane_x7_mpc_ros2 ros2 launch crane_x7_description_ros2 display.launch.py 
```