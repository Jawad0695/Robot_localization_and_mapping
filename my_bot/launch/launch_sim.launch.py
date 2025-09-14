import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    urdf_file_path = '/home/jawad/bocbot_ws/src/mobile-3d-lidar-sim/my_bot/urdf/robot.urdf'
    world_file_path = '/home/jawad/bocbot_ws/src/mobile-3d-lidar-sim/my_bot/worlds/bocbot_office.world'

    # Load robot description directly from URDF file
    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

    # Gazebo launch
    gazebo_process = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', LaunchConfiguration('world')],
        output='screen'
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', LaunchConfiguration('model'),
            '-entity', 'my_bot',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-R', LaunchConfiguration('R'),
            '-P', LaunchConfiguration('P'),
            '-Y', LaunchConfiguration('Y')
        ],
        output='screen'
    )

    # Joint State Publisher
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('model', default_value=urdf_file_path,
                              description='Absolute path to robot URDF file'),
        DeclareLaunchArgument('world', default_value=world_file_path,
                              description='Absolute path to world file'),
        DeclareLaunchArgument('x', default_value='-0.142601', description='Initial x position of the robot'),
        DeclareLaunchArgument('y', default_value='-2.065040', description='Initial y position of the robot'),
        DeclareLaunchArgument('z', default_value='0.150008', description='Initial z position of the robot'),
        DeclareLaunchArgument('R', default_value='-0.000005', description='Initial roll orientation of the robot'),
        DeclareLaunchArgument('P', default_value='0.000040', description='Initial pitch orientation of the robot'),
        DeclareLaunchArgument('Y', default_value='-0.062120', description='Initial yaw orientation of the robot'),

        # Nodes and processes
        gazebo_process,
        spawn_entity,
        joint_state_publisher,
        robot_state_publisher
    ])

