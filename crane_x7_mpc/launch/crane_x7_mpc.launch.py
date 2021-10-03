import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Get MPC config
    crane_x7_mpc_path = os.path.join(
        get_package_share_directory('crane_x7_mpc'))
    config_file = os.path.join(crane_x7_mpc_path,
                               'config', 'crane_x7_mpc.yaml')
    spawn_mpc = Node(
        package="crane_x7_mpc",
        executable="crane_x7_mpc_composition",
        output='screen',
        parameters=[config_file],
    )

    return LaunchDescription([
        spawn_mpc,
    ])