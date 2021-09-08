import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), 
                    '/gazebo.launch.py']),
             )

    # Get URDF via xacro
    crane_x7_description_path = os.path.join(
        get_package_share_directory('crane_x7_description'))
    xacro_file = os.path.join(crane_x7_description_path,
                              'urdf', 'crane_x7.urdf.xacro')
    robot_description = {'robot_description': Command(['xacro ', xacro_file, ' use_gazebo:=true'])}

    # Robot state publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher', 
        executable='robot_state_publisher', 
        name='robot_state_publisher', 
        output='screen', 
        parameters=[robot_description]
    )

    # Gazebo's robot entity
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py', 
        arguments=['-topic', 'robot_description', '-entity', 'crane_x7', 
                   '-x', '0', '-y', '0', '-z', '0'], 
        output='screen'
    )

    # controllers
    crane_x7_gazebo_controllers_path = os.path.join(
        get_package_share_directory('crane_x7_gazebo'),
        'config',
        'crane_x7_gazebo_controllers.yaml'
    )
    spawn_controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, crane_x7_gazebo_controllers_path],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['joint_state_broadcaster']
    )

    # spawn_joint_state_broadcaster = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
    #          'joint_state_broadcaster'],
    #     shell=True,
    #     output='screen'
    # )

    # spawn_effort_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'joint_effort_controllers'],
    #     shell=True,
    #     output='screen'
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
        spawn_controller_manager,
        # spawn_joint_state_broadcaster,
    ])