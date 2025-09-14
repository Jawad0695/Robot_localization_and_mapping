import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_my_bot = get_package_share_directory('my_bot')

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='rviz.rviz',
        description='RViz config file'
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Flag to enable use_sim_time'
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([pkg_my_bot, 'map', 'my_map_save.yaml']),
        description='Full path to map yaml file to load'
    )

    # Path to Nav2 localization launch file
    nav2_localization_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'localization_launch.py'
    )

    # Use the correct parameters file structure for Nav2
    localization_params_path = PathJoinSubstitution([
        pkg_my_bot,
        'config',
        'nav2_params.yaml'  # contains both map_server and amcl parameters
    ])

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_my_bot, 'urdf', LaunchConfiguration('rviz_config')])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    # Static Transform Publisher (base_link -> lidar_link)
    static_tf_pub = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0.2", "0", "0", "0", "map", "odom"],
        output="screen"
    )

    # Nav2 localization (map_server + AMCL + lifecycle manager)
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_localization_launch_path),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': localization_params_path,
            'map': LaunchConfiguration('map'),
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(rviz_launch_arg)
    ld.add_action(rviz_config_arg)
    ld.add_action(sim_time_arg)
    ld.add_action(map_arg)
    ld.add_action(rviz_node)
    ld.add_action(static_tf_pub)   # ✅ added here
    ld.add_action(localization_launch)

    return ld

