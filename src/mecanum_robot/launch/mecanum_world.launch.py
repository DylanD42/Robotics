import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('mecanum_robot')

    xacro_file = os.path.join(pkg_share, 'urdf', 'mecanum_robot.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'small_room.world')

    # --- Process the xacro file into a URDF/XML string -----------------
    # This is the piece that was missing: robot_description must be the
    # *processed XML content*, not the raw file path. Command(...) runs
    # `xacro <file>` and captures stdout; ParameterValue(..., value_type=str)
    # forces rclpy to treat that output as a plain string parameter instead
    # of trying to auto-detect its type (which is what triggers the
    # "unable to parse robot_description as yaml" error on Humble).
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # --- Launch Gazebo Classic with our custom world --------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={'world': world_file}.items()
    )

    # --- robot_state_publisher: publishes TF from robot_description -----
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    # --- Spawn the robot into the running Gazebo world -------------------
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'mecanum_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.05'
        ],
        output='screen'
    )
    # --- HW3: Forward Kinematics node -----------------------------------
    # Subscribes to /wheel_speeds, publishes /cmd_vel (consumed by the
    # planar_move plugin already in our URDF from HW2).
    forward_kinematics_node = Node(
        package='mecanum_robot',
        executable='forward_kinematics_node.py',
        name='forward_kinematics_node',
        output='screen',
        parameters=[{
            'wheel_radius': 0.034,
            'lx': 0.097,
            'ly': 0.105,
        }]
    )

    # --- HW3: Robot Controller node --------------------------------------
    # Publishes wheel-speed test schemes on a timer -> /wheel_speeds.
    robot_controller_node = Node(
        package='mecanum_robot',
        executable='robot_controller_node.py',
        name='robot_controller_node',
        output='screen'
    )
    return LaunchDescription([
        gazebo,
        robot_state_publisher_node,
        spawn_entity,
        forward_kinematics_node,
        robot_controller_node
    ])
