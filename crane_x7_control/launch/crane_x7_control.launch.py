import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command


def generate_launch_description():
    # Get URDF via xacro
    crane_x7_description_path = os.path.join(
        get_package_share_directory('crane_x7_description'))
    xacro_file = os.path.join(crane_x7_description_path,
                              'urdf', 'crane_x7.urdf.xacro')
    robot_description = {'robot_description': Command(['xacro ', xacro_file, ' use_gazebo:=true'])}

    crane_x7_controllers = os.path.join(
        get_package_share_directory('crane_x7_control'),
        'config',
        'crane_x7_controllers.yaml'
        )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, crane_x7_controllers],
        output={
          'stdout': 'screen',
          'stderr': 'screen',
          },
        )

    spawn_joint_state_broadcaster = ExecuteProcess(
                cmd=['ros2 run controller_manager spawner.py joint_state_broadcaster'],
                shell=True,
                output='screen',
            )

    # spawn_arm_controller = ExecuteProcess(
    #             cmd=['ros2 run controller_manager spawner.py crane_x7_arm_controller'],
    #             shell=True,
    #             output='screen',
    #         )

    # spawn_gripper_controller = ExecuteProcess(
    #             cmd=['ros2 run controller_manager spawner.py crane_x7_gripper_controller'],
    #             shell=True,
    #             output='screen',
    #         )

    return LaunchDescription([
      controller_manager,
      spawn_joint_state_broadcaster,
    #   spawn_arm_controller,
    #   spawn_gripper_controller
    ])