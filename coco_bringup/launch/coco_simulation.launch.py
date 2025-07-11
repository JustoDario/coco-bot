
#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    # Argumentos
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="coco_description",
            description="Paquete con los xacro/urdf del robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="coco.gazebo.xacro",
            description="Archivo XACRO para simulación.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="controllers.yaml",
            description="YAML de controladores.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "world",
            default_value="empty.world",
            description="Archivo de mundo de Gazebo.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="false",
            description="Lanzar RViz2.",
        )
    )

    # Inicialización de argumentos
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    controllers_file = LaunchConfiguration("controllers_file")
    world = LaunchConfiguration("world")
    launch_rviz = LaunchConfiguration("launch_rviz")

    # Obtener URDF desde xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
    ])
    robot_description = {"robot_description": robot_description_content}

    # Paths de controladores
    controllers_file_path = PathJoinSubstitution(
        [FindPackageShare("coco_bringup"), "config", controllers_file]
    )
    controllers_spawner_file_path = PathJoinSubstitution(
        [FindPackageShare("coco_bringup"), "config", "controllers_spawner.yaml"]
    )

    # Lanzar Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
        ]),
        launch_arguments={'world': world}.items(),
    )

    # Nodo robot_state_publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Spawners de controladores
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

    # Event handler para lanzar el gait_planifier después del spawner
    gait_planifier_node = Node(
        package="coco_mov_control",
        executable="gait_planifier",
        name="gait_planifier",
        output="screen",
    )
    delay_gait_planifier_after_joint_trajectory_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_trajectory_controller_spawner,
            on_exit=[gait_planifier_node],
        )
    )

    # (Opcional) RViz2
    # rviz_config_file = PathJoinSubstitution(
    #     [FindPackageShare(description_package), "rviz", "coco.rviz"]
    # )
    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="log",
    #     arguments=["-d", rviz_config_file],
    #     condition=IfCondition(launch_rviz),
    # )

    nodes = [
        gazebo_launch,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        delay_gait_planifier_after_joint_trajectory_controller_spawner,
        # rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)