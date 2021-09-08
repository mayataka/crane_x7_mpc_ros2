import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command


def generate_launch_description():
    # Get URDF via xacro
    crane_x7_description_path = os.path.join(
        get_package_share_directory('crane_x7_description'))
    xacro_file = os.path.join(crane_x7_description_path,
                              'urdf', 'crane_x7.urdf.xacro')
    robot_description = {'robot_description': Command(['xacro ', xacro_file, ' use_gazebo:=false'])}

    rviz_config = os.path.join(crane_x7_description_path,
                               'config', 'display.rviz')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='both',
            parameters=[robot_description]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='log',
            arguments=['-d', rviz_config]
        )
    ])