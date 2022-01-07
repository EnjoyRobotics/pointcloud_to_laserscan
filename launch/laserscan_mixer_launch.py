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
        Node(
            package='laserscan_mixer',
            executable='laserscan_to_pointcloud_node',
            name='laserscan_to_pointcloud',
            remappings=[('scan_in', 'lidar1/scan'),
                        ('cloud', [LaunchConfiguration(variable_name='scanner'), '/cloud1'])],
            parameters=[{'target_frame': 'base_link', 'transform_tolerance': 0.01}]
        ),
        Node(
            package='laserscan_mixer',
            executable='laserscan_to_pointcloud_node',
            name='laserscan_to_pointcloud',
            remappings=[('scan_in', 'lidar2/scan'),
                        ('cloud', [LaunchConfiguration(variable_name='scanner'), '/cloud2'])],
            parameters=[{'target_frame': 'base_link', 'transform_tolerance': 0.01}]
        ),
        Node(
            package='laserscan_mixer',
            executable='pointcloud_mixer',
            name='pointcloud_mixer',
        ),
        Node(
            package='laserscan_mixer', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/cloud'),
                        ('scan' '/scan')],
            parameters=[{
                'target_frame': 'base_link',
                'transform_tolerance': 0.01,
                'min_height': 0.0,
                'max_height': 1.0,
                'angle_min': -1.5708*2,  # -M_PI/2
                'angle_max': 1.5708*2,  # M_PI/2
                'angle_increment': 0.0087,  # M_PI/360
                'scan_time': 0.3333,
                'range_min': 0.01,
                'range_max': 64.0,
                'use_inf': False,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        )
    ])
