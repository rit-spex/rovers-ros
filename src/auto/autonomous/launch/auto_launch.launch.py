from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autonomous',
            executable='arcu_tracker',
            name='auto_node'
        ),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='camera_node'
        ),
        Node(
            package='autonomous',
            executable='run_auto',
            name='pathfinding_node'
        ),
    ])
