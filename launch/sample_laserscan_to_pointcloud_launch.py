from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import LifecycleNode
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import yaml


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='scanner', default_value='scanner',
            description='Namespace for sample topics'
        ),
        # ExecuteProcess(
        #     cmd=[
        #         'ros2', 'topic', 'pub', '-r', '10',
        #         '--qos-profile', 'sensor_data',
        #         [LaunchConfiguration(variable_name='lidar2'), '/scan'],
        #         'sensor_msgs/msg/LaserScan', yaml.dump({
        #             'header': {'frame_id': 'scan'}, 'angle_min': -1.0,
        #             'angle_max': 1.0, 'angle_increment': 0.1, 'range_max': 10.0,
        #             'ranges': [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
        #         })
        #     ],
        #     name='scan_publisher'
        # ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_transform_publisher',
        #     arguments='0 0 0 0 0 0 1 map scan'
        # ),
        Node(
            package='pointcloud_to_laserscan',
            executable='laserscan_to_pointcloud_node',
            name='laserscan_to_pointcloud',
            remappings=[('scan_in', 'lidar2/scan'),
                        ('cloud', [LaunchConfiguration(variable_name='scanner'), '/cloud1'])],
            parameters=[{'target_frame': 'lidar2', 'transform_tolerance': 0.01}]
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='laserscan_to_pointcloud_node',
            name='laserscan_to_pointcloud',
            remappings=[('scan_in', 'lidar1/scan'),
                        ('cloud', [LaunchConfiguration(variable_name='scanner'), '/cloud2'])],
            parameters=[{'target_frame': 'lidar2', 'transform_tolerance': 0.01}]
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_mixer',
            name='pointcloud_mixer',
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/cloud'),
                        ('scan', [LaunchConfiguration(variable_name='scanner'), '/scan'])],
            parameters=[{
                'target_frame': 'lidar2',
                'transform_tolerance': 0.01,
                'min_height': 0.0,
                'max_height': 1.0,
                'angle_min': -1.5708*2,  # -M_PI/2
                'angle_max': 1.5708*2,  # M_PI/2
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.01,
                'range_max': 64.0,
                'use_inf': False,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        )
    ])
