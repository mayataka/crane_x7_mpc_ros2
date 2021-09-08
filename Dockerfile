FROM osrf/ros:foxy-desktop

# Make sure everything is up to date before building from source
RUN apt-get update \
    && apt-get upgrade -y \
    && apt-get clean

RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    lsb-release \
    python3-colcon-ros \
    && apt-get clean

RUN mkdir -p /home/ros2_ws/src/
# COPY . /home/ros2_ws/src/crane_x7_mpc_ros2
COPY ./crane_x7_mpc_ros2 /home/ros2_ws/src/crane_x7_mpc_ros2/crane_x7_mpc_ros2 
COPY ./crane_x7_gazebo /home/ros2_ws/src/crane_x7_mpc_ros2/crane_x7_gazebo
COPY ./crane_x7_description /home/ros2_ws/src/crane_x7_mpc_ros2/crane_x7_description
RUN cd /home/ros2_ws \
    && rosdep update \
    && rosdep install --from-paths ./ -y --rosdistro foxy \
      --ignore-src --verbose 

RUN cd /home/ros2_ws \
    && . /opt/ros/foxy/setup.sh \
    && colcon build --symlink-install 

COPY ./.docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

CMD ros2 launch crane_x7_gazebo crane_x7_effort_ctrl.launch.py gui:=false