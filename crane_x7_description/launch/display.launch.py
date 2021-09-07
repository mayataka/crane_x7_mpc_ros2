import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command


def generate_launch_description():
    pkg_share = FindPackageShare('crane_x7_description').find('crane_x7_description')
    urdf_dir = os.path.join(pkg_share, 'urdf')
    xacro_file = os.path.join(urdf_dir, 'crane_x7.urdf.xacro')
    params = {'robot_description': Command(['xacro ', xacro_file, ' use_gazebo:=false'])}

    rsp = Node(package='robot_state_publisher',
               executable='robot_state_publisher',
               output='both',
               parameters=[params])
    jsp = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
    )

    rviz_config_file = get_package_share_directory(
        'crane_x7_description') + '/config/urdf.rviz'
    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     output='log',
                     arguments=['-d', rviz_config_file])

    return LaunchDescription([rsp, jsp, rviz_node])
