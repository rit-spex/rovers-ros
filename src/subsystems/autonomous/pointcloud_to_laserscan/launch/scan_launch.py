from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='', default_value='',
            description='Namespace for sample topics'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--qx', '0', '--qy', '-0.7071068', '--qz', '0', '--qw', '0.7071068',
                '--frame-id', 'laser_frame', '--child-frame-id', 'cloud'
            ]
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('scan', [LaunchConfiguration(variable_name=''), '/scan'])],
            parameters=[{
                'target_frame': 'cloud',
                'transform_tolerance': 0.01,
                'min_height': 0.0,
                'max_height': 5.0,
                'angle_min': -3.1415,  
                'angle_max': 3.1415,  
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.45,
                'range_max': 10.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        )
    ])

