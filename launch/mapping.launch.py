"""Handheld mapping: RPLidar C1 + static TF + Cartographer + RViz.

Usage:
  ros2 launch handheld_mapping mapping.launch.py
  ros2 launch handheld_mapping mapping.launch.py serial_port:=/dev/ttyUSB1
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('handheld_mapping'), 'config')
    serial_port = LaunchConfiguration('serial_port')

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0',
                              description='USB port of the RPLidar C1'),

        # 1) RPLidar C1 driver: Slamtec's own launch file -> publishes /scan in frame "laser"
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('sllidar_ros2'), 'launch', 'sllidar_c1_launch.py')),
            launch_arguments={
                'serial_port': serial_port,
                'frame_id': 'laser',
            }.items(),
        ),

        # 2) base_link -> laser. Handheld: the lidar IS the "robot", so zero offset.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            arguments=['--x', '0', '--y', '0', '--z', '0',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'base_link', '--child-frame-id', 'laser'],
        ),

        # 3) rf2o laser odometry -> publishes /odom and odom -> base_link
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'laser_scan_topic': '/scan',
                'odom_topic': '/odom',
                'publish_tf': True,
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'init_pose_from_topic': '',   # empty = start at (0,0,0) instead of waiting for a pose
                'freq': 20.0,
            }],
        ),

        # 4) Cartographer SLAM -> publishes map -> odom (using /scan and /odom)
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=['-configuration_directory', config_dir,
                       '-configuration_basename', 'handheld_mapping.lua'],
        ),

        # 5) Turns Cartographer submaps into a /map occupancy grid (for RViz and map_saver).
        #    If this executable isn't found on your distro, run
        #    `ros2 pkg executables cartographer_ros` and use the occupancy grid node listed there.
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'],
        ),

        # 6) Nav2 navigation stack (equivalent to
        #    `ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false`)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': 'false',
            }.items(),
        ),

        # 7) RViz
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen'),
    ])