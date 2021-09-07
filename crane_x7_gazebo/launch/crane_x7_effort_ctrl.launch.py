import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
             )

    crane_x7_description_path = os.path.join(
        get_package_share_directory('crane_x7_description'))
    xacro_file = os.path.join(crane_x7_description_path,
                              'urdf', 'crane_x7.urdf.xacro')
    params = {'robot_description': Command(['xacro ', xacro_file, ' use_gazebo:=true'])}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'crane_x7',
                                   '-x', '0', '-y', '0', '-z', '0'],
                        output='screen')

    # spawn_joint_state_broadcaster = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
    #          'joint_state_broadcaster'],
    #     shell=True,
    #     output='screen'
    # )

    # spawn_effort_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'effort_controllers'],
    #     shell=True,
    #     output='screen'
    # )

    # spawn_joint_state_broadcaster = ExecuteProcess(
    #     cmd=['ros2', 'run', 'controller_manager', 'spawner.py', 
    #          'joint_state_broadcaster'],
    #     shell=True,
    #     output='screen',
    # )

    return LaunchDescription([
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=spawn_entity,
        #         on_exit=[spawn_joint_state_broadcaster],
        #     )
        # ),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=spawn_joint_state_broadcaster,
        #         on_exit=[spawn_effort_controller],
        #     )
        # ),
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
    ])