import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_config = LaunchConfiguration('robot_config')
    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('rover_simulation'))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    # robot_description_config = xacro.process_file(xacro_file).toxml()
    robot_description_config = Command([
        'xacro ', xacro_file,
        ' robot_config:=', robot_config])
    # Create a robot_state_publisher node
    params = {
        'robot_description': ParameterValue(
            robot_description_config, value_type=str),
        'use_sim_time': use_sim_time,
    }
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
        # Gazebo's joint_state_broadcaster lives in the /GAZEBO namespace.
        remappings=[('joint_states', '/GAZEBO/joint_states')],
    )


    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'robot_config',
            default_value='arm_drivebase',
            description='Rover build: drivebase, arm_drivebase, or '
                        'life_drivebase'),

        node_robot_state_publisher
    ])
