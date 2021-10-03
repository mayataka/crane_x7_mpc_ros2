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

    crane_x7_gazebo_controllers_config = os.path.join(
        get_package_share_directory('crane_x7_gazebo'),
        'config', 'crane_x7_gazebo_controllers.yaml'
    )

    spawn_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

    # Gazebo's robot entity
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py', 
        arguments=['-topic', 'robot_description', '-entity', 'crane_x7', 
                   '-x', '0', '-y', '0', '-z', '0'], 
        output='screen'
    )

    spawn_controller_manager = Node(
        package='controller_manager', 
        executable='ros2_control_node', 
        parameters=[robot_description, crane_x7_gazebo_controllers_config], 
        output={'stdout': 'screen', 
                'stderr': 'screen', 
        },
    )
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )
    spawn_joint_position_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_position_controller", "-c", "/controller_manager"],
    )
    # spawn_joint_velocity_controller = Node(
    #     package="controller_manager",
    #     executable="spawner.py",
    #     arguments=["joint_velocity_controller", "-c", "/controller_manager"],
    # )
    spawn_mpc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('crane_x7_mpc'), 'launch'), 
            '/crane_x7_mpc.launch.py']),
    )


    return LaunchDescription([
        gazebo,
        spawn_robot_state_publisher,
        spawn_entity,
        spawn_controller_manager,
        spawn_joint_state_broadcaster,
        spawn_joint_position_controller,
        # spawn_joint_velocity_controller,
        spawn_mpc,
    ])