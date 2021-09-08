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

    crane_x7_controllers_config = os.path.join(
        get_package_share_directory('crane_x7_control'),
        'config', 'crane_x7_controllers.yaml'
    )

    rviz_config= os.path.join(crane_x7_description_path,
                              'config', 'display.rviz')

    spawn_controller_manager = Node(
        package='controller_manager', 
        executable='ros2_control_node', 
        parameters=[robot_description, crane_x7_controllers_config], 
        output={'stdout': 'screen', 
                'stderr': 'screen', },
    )
    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    # spawn_joint_trajectory_controller = Node(
    #     package="controller_manager",
    #     executable="spawner.py",
    #     arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    # )

    spawn_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[robot_description],
        output="screen"
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        spawn_controller_manager,
        spawn_joint_state_broadcaster,
        # spawn_joint_trajectory_controller 
        spawn_robot_state_publisher,
        rviz,
    ])