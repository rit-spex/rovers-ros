import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import LaunchConfigurationEquals
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'rover_simulation'

    package_share = get_package_share_directory(package_name)

    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    robot_config = LaunchConfiguration('robot_config')

    # Include the robot_state_publisher launch file, provided by our own package.
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            package_share, 'launch', 'rsp.launch.py'
        )]), launch_arguments={
            'use_sim_time': use_sim_time,
            'robot_config': robot_config,
        }.items()
    )

    gazebo_params_file = os.path.join(package_share, 'config', 'gazebo_params.yaml')
    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={
            'world': PathJoinSubstitution([package_share, 'worlds', world]),
            'params_file': gazebo_params_file,
            'gui': gui,
        }.items()
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't
    # really matter if you only have a single robot. Spawn at wheel height so the
    # wheels start resting on the ground instead of intersecting it.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'rover',
                                   '-z', '0.081674'],
                        output='screen')

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont", "-c", "/GAZEBO/controller_manager"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad", "-c", "/GAZEBO/controller_manager"],
    )

    arm_cont_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_cont", "-c", "/GAZEBO/controller_manager"],
        condition=LaunchConfigurationEquals(
            'robot_config', 'arm_drivebase'),
    )

    # The controller_manager only exists once gazebo_ros2_control has loaded the
    # robot, so wait for the spawn to finish before starting the spawners.
    # Chain them: configuring several controllers at once races inside the
    # controller_manager and can leave joint_state_broadcaster unconfigured,
    # which stops the arm angle stream and leaves IK waiting forever.
    spawn_joint_broad = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_broad_spawner],
        )
    )
    spawn_diff_drive = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_broad_spawner,
            on_exit=[diff_drive_spawner],
        )
    )
    spawn_arm = RegisterEventHandler(
        OnProcessExit(
            target_action=diff_drive_spawner,
            on_exit=[arm_cont_spawner],
        )
    )

    lidar_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '-0.7071068', '--qz', '0', '--qw', '0.7071068',
            '--frame-id', 'laser_frame', '--child-frame-id', 'cloud'
        ]
    )
    points_2_scan = Node(
        package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        parameters=[{
            'use_sim_time': use_sim_time,
            'target_frame': 'cloud',
            'transform_tolerance': 0.01,
            'min_height': 0.0508,
            'max_height': 1.3208,
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

    # Stands in for the chassis Teensy: turns the same CAN Drive_Power frame
    # the real rover's chassis_node publishes into a diff-drive command.
    # It also arbitrates the simulation-only navigation and joystick inputs,
    # handles /ESTOP, applies the motor deadzone, and talks to diff_cont
    # directly.
    drivebase_sim = Node(
            package='rover_simulation',
            executable='drivebase_sim_node',
            name='drivebase_sim_node',
            parameters=[{'use_sim_time': use_sim_time}],
        )

    # Launch them all!
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='empty.world',
            description='World file from the package worlds/ directory to load'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the Gazebo clock'),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Set to "false" to run Gazebo headless'),
        DeclareLaunchArgument(
            'robot_config',
            default_value='arm_drivebase',
            description='Rover build: drivebase, arm_drivebase, or '
                        'life_drivebase'),
        rsp,
        gazebo,
        spawn_entity,
        spawn_joint_broad,
        spawn_diff_drive,
        spawn_arm,
        lidar_transform,
        points_2_scan,
        drivebase_sim,
    ])
