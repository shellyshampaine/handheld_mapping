"""Handheld mapping: RPLidar C1 + static TF + Cartographer + Nav2 + RViz.

Usage:
  ros2 launch handheld_mapping mapping.launch.py
  ros2 launch handheld_mapping mapping.launch.py serial_port:=/dev/ttyUSB1
  ros2 launch handheld_mapping mapping.launch.py odom_topic:=/group_4/odom
  ros2 launch handheld_mapping mapping.launch.py cmd_vel_topic:=/group_4/cmd_vel
  ros2 launch handheld_mapping mapping.launch.py tf_namespace:=group_4   # only if TF is on /group_4/tf
  ros2 launch handheld_mapping mapping.launch.py laser_z:=0.12           # lidar height above base_link (m)
  ros2 launch handheld_mapping mapping.launch.py laser_yaw:=0.0          # lidar rotation about z (rad)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('handheld_mapping'), 'config')
    serial_port = LaunchConfiguration('serial_port')
    odom_topic = LaunchConfiguration('odom_topic')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    tf_namespace = LaunchConfiguration('tf_namespace')
    laser_z = LaunchConfiguration('laser_z')
    laser_yaw = LaunchConfiguration('laser_yaw')

    # True when the robot publishes TF on /<ns>/tf instead of /tf
    use_tf_ns = PythonExpression(["'", tf_namespace, "' != ''"])

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0',
                              description='USB port of the RPLidar C1'),
        DeclareLaunchArgument('odom_topic', default_value='/group_4/odom',
                              description='Topic the robot publishes nav_msgs/Odometry on'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/group_4/cmd_vel',
                              description='Topic the robot takes geometry_msgs/Twist on'),
        DeclareLaunchArgument('tf_namespace', default_value='',
                              description='Set (e.g. group_4) if TF is published on /<ns>/tf'),
        # Must be above 0: the local costmap's voxel grid starts at z=0, so a
        # laser at z=0 falls outside it and obstacles are never cleared.
        DeclareLaunchArgument('laser_z', default_value='0.1',
                              description='Height of the lidar above base_link in metres'),
        # Default is 180 deg: with yaw 0, Cartographer saw forward driving as
        # reversing, while rotation still looked right.
        DeclareLaunchArgument('laser_yaw', default_value='3.14159',
                              description='Yaw of the laser frame relative to base_link in radians'),

        # Everything inside this group gets the remaps below, including the
        # nodes started by the included Slamtec and Nav2 launch files.
        GroupAction([
            # /odom -> /group_4/odom for Cartographer and Nav2 (bt_navigator,
            # velocity_smoother, controller etc. all subscribe to "odom")
            SetRemap(src='/odom', dst=odom_topic),

            # Send only Nav2's two final outputs to the robot. A plain /cmd_vel
            # remap would also catch the controller and the smoother's input,
            # bypassing the velocity smoother (first matching remap wins).
            SetRemap(src='/cmd_vel_smoothed', dst=cmd_vel_topic),  # path following
            SetRemap(src='behavior_server:/cmd_vel', dst=cmd_vel_topic),  # spin, backup

            # Optional: follow the robot's namespaced TF tree
            SetRemap(src='/tf', dst=['/', tf_namespace, '/tf'], condition=IfCondition(use_tf_ns)),
            SetRemap(src='/tf_static', dst=['/', tf_namespace, '/tf_static'],
                     condition=IfCondition(use_tf_ns)),

            # 1) RPLidar C1 driver -> /scan in frame "laser"
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('sllidar_ros2'), 'launch', 'sllidar_c1_launch.py')),
                launch_arguments={
                    'serial_port': serial_port,
                    'frame_id': 'laser',
                }.items(),
            ),

            # 2) base_link -> laser
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_to_laser',
                arguments=['--x', '0', '--y', '0', '--z', laser_z,
                           '--roll', '0', '--pitch', '0', '--yaw', laser_yaw,
                           '--frame-id', 'base_link', '--child-frame-id', 'laser'],
            ),

            # 3) Cartographer SLAM -> map -> odom (uses /scan and the remapped odom topic)
            Node(
                package='cartographer_ros',
                executable='cartographer_node',
                name='cartographer_node',
                output='screen',
                parameters=[{'use_sim_time': False}],
                arguments=['-configuration_directory', config_dir,
                           '-configuration_basename', 'handheld_mapping.lua'],
                remappings=[('odom', odom_topic)],  # explicit, in case SetRemap is missed
            ),

            # 4) Submaps -> /map occupancy grid
            Node(
                package='cartographer_ros',
                executable='cartographer_occupancy_grid_node',
                name='occupancy_grid_node',
                output='screen',
                parameters=[{'use_sim_time': False}],
                arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'],
            ),

            # 5) Nav2
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
                launch_arguments={
                    'use_sim_time': 'false',
                    'params_file': os.path.join(config_dir, 'nav2_params.yaml'),
                }.items(),
            ),

            
        ]),
    ])