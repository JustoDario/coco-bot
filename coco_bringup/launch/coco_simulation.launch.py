# Copyright (c) 2025 Justo Dario Valverde
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.descriptions import ParameterValue

# Obtén el URDF de simulación (el xacro que incluye el plugin de Gazebo)
robot_description_content = Command([
    PathJoinSubstitution([FindExecutable(name="xacro")]),
    " ",
    PathJoinSubstitution(
        [FindPackageShare("coco_description"), "urdf", "coco.gazebo.xacro"]
    ),
])
robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

controllers_file_path = PathJoinSubstitution(
    [FindPackageShare("coco_bringup"), "config", "controllers.yaml"]
)
controllers_spawner_file_path = PathJoinSubstitution(
    [FindPackageShare("coco_bringup"), "config", "controllers_spawner.yaml"]
)

#
#controller_manager_node = Node(
#    package="controller_manager",
#    executable="ros2_control_node",
#    parameters=[robot_description, controllers_file_path],
#    output="both",
#)
#
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

# (Opcional) Lanza el gait_planifier solo después de que el joint_trajectory_controller esté activo
gait_planifier = Node(
    package='coco_mov_control',
    executable='gait_planifier',
    name='gait_planifier',
    output='screen',
)

delay_gait_planifier_after_joint_trajectory_controller_spawner = RegisterEventHandler(
    event_handler=OnProcessExit(
        target_action=joint_trajectory_controller_spawner,
        on_exit=[gait_planifier],
    )
)


def generate_launch_description():

    world_arg = DeclareLaunchArgument(
        'world', default_value=os.path.join(
            get_package_share_directory('aws_robomaker_small_house_world'),
            'worlds',
            'small_house.world'))

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Set to false to run gazebo headless',
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch',
                         'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -s ', LaunchConfiguration('world')]}.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch',
                         'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': [' -g ']}.items(),
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('coco_description'),
            'launch/', 'spawn.launch.py')]),
        launch_arguments={
            'description_file': os.path.join(
                get_package_share_directory('coco_description'),
                'urdf', 'coco.gazebo.xacro'
            )
        }.items()
    )

    model_path = ''
    resource_path = ''
    pkg_path = get_package_share_directory('coco_bringup')
    model_path += os.path.join(pkg_path, 'models')
    resource_path += pkg_path + model_path

    if 'GZ_SIM_MODEL_PATH' in os.environ:
        model_path += os.pathsep+os.environ['GZ_SIM_MODEL_PATH']
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        resource_path += os.pathsep+os.environ['GZ_SIM_RESOURCE_PATH']

    ld = LaunchDescription()
    ld.add_action(SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', model_path))
    ld.add_action(world_arg)
    ld.add_action(gui_arg)
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)
    ld.add_action(spawn_robot)
    # ld.add_action(controller_manager_node)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(joint_trajectory_controller_spawner)
    ld.add_action(delay_gait_planifier_after_joint_trajectory_controller_spawner)

    return ld
