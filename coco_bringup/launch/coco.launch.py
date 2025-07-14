#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
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
    #rviz_config_file = PathJoinSubstitution(
    #    [FindPackageShare(description_package), "rviz", "coco.rviz"]
   # )
   # rviz_node = Node(
    #    package="rviz2",
     #   executable="rviz2",
      #  name="rviz2",
       # output="log",
       # arguments=["-d", rviz_config_file],
      #  condition=IfCondition(launch_rviz),
    #)

    nodes = [
        controller_manager_node,
        robot_state_pub_node,
        joint_state_publisher_node,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        delay_gait_planifier_after_joint_trajectory_controller_spawner,
        #rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
