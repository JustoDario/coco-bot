#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare arguments
    # Slidar args
    channel_type =  LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='460800')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Standard')
    
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="coco_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="coco.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="false",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
                'channel_type',
                default_value=channel_type,
                description='Specifying channel type of lidar')
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifying usb port to connected lidar')
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying usb port baudrate to connected lidar')
    )
    declared_arguments.append(        
        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar')
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data')
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data')
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode,
            description='Specifying scan mode of lidar')
    )
    # Initialize Arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    controllers_file = LaunchConfiguration("controllers_file")
    launch_rviz = LaunchConfiguration("launch_rviz")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Get controllers file
    controllers_file_path = PathJoinSubstitution(
        [FindPackageShare("coco_bringup"), "config", controllers_file]
    )
    controllers_spawner_file_path = PathJoinSubstitution(
        [FindPackageShare("coco_bringup"), "config", "controllers_spawner.yaml"]
    )

    # Controller manager node
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_file_path],
        output="both",
    )

    # Joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--param-file", controllers_spawner_file_path
        ],
        output="screen",
    )

    # BNO055 IMU node
    bno_config = os.path.join(
        get_package_share_directory('bno055'),
        'config',
        'bno055_params.yaml'
    )
    bno_node = Node(
        package='bno055',
        executable='bno055',
        parameters=[bno_config],
        output='screen',
    )

    # Joint trajectory controller
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager", "/controller_manager",
            "--param-file", controllers_spawner_file_path
        ],
        output="screen",
    )

    # Robot state publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Joint state publisher
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[
            {
                "source_list": ["joint_states"],
                "rate": 50,
            }
        ],
    )

    # Gait planifier node
    gait_planifier_node = Node(
        package="coco_mov_control",
        executable="gait_planifier",
        name="gait_planifier",
        output="screen",
    )
    # Event handler para lanzar gait_planifier después del spawner del joint_trajectory_controller
    delay_gait_planifier_after_joint_trajectory_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_trajectory_controller_spawner,
            on_exit=[gait_planifier_node],
        )
    )

    # RViz2
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "urdf", "coco.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
    )
    #slidar_ros2(c1)
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{'channel_type':channel_type,
                        'serial_port': serial_port, 
                        'serial_baudrate': serial_baudrate, 
                        'frame_id': frame_id,
                        'inverted': inverted, 
                        'angle_compensate': angle_compensate, 
                        'scan_mode': scan_mode}],
        output='screen',
    )
    #laser box filter
    laser_filter_node = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[
            PathJoinSubstitution([
                get_package_share_directory("laser_filters"),
                "examples", "box_filter_example.yaml",
            ])],
    )
    #odometry
    odometry_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic' : '/scan_filtered',
            'odom_topic' : '/odom',
            'publish_tf' : True,
            'base_frame_id' : 'base_footprint',
            'odom_frame_id' : 'odom',
            'init_pose_from_topic' : '',
            'freq' : 30.0}],
    )

    nodes = [
        controller_manager_node,
        robot_state_pub_node,
        joint_state_publisher_node,
        joint_state_broadcaster_spawner,
        bno_node,
        joint_trajectory_controller_spawner,
        delay_gait_planifier_after_joint_trajectory_controller_spawner,
        rviz_node,
        lidar_node,
        laser_filter_node,
        odometry_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
