# Copyright (c) 2024, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

#
# Source of this file are templates in
# [RosTeamWorkspace](https://github.com/StoglRobotics/ros_team_workspace) repository.
#
# Author: Dr. Denis
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "mock_sensor_commands",
            default_value="false",
            description="Enable mock command interfaces for sensors used for simple simulations. \
            Used only if 'use_mock_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_gazebo_classic",
            default_value="false",
            description="Start robot with gazebo classic simulation.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_gazebo",
            default_value="false",
            description="Start robot with gazebo simulation.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_isaac_sim",
            default_value="false",
            description="Start robot with isaac sim simulation.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "real_world",
            default_value="false",
            description="Start robot with real world",
        )
    )
    # Initialize Arguments
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    sim_gazebo_classic = LaunchConfiguration("sim_gazebo_classic")
    sim_gazebo = LaunchConfiguration("sim_gazebo")
    sim_isaac_sim = LaunchConfiguration("sim_isaac_sim")
    real_world = LaunchConfiguration("real_world")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("piper_bringup"), "urdf", "agilex_piper_single.system.xacro"]
            ),
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
            " ",
            "mock_sensor_commands:=",
            mock_sensor_commands,
            " ",
            "sim_gazebo_classic:=",
            sim_gazebo_classic,
            " ",
            "sim_gazebo:=",
            sim_gazebo,
            " ",
            "sim_isaac_sim:=",
            sim_isaac_sim,
            " ",
            "real_world:=",
            real_world,
        ]
    )

    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("piper_bringup"), "config", "controllers.yaml"]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("piper_bringup"), "rviz", "piper.rviz"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[robot_controllers],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ]
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_names = []
    robot_controller_names += ["arm_trajectory_controller"]
    robot_controller_spawners = []
    for controller in robot_controller_names:
        robot_controller_spawners += [
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "-c", "/controller_manager"],
            )
        ]

    inactive_robot_controller_names = ["arm_forward_position_controller"]
    inactive_robot_controller_spawners = []
    for controller in inactive_robot_controller_names:
        inactive_robot_controller_spawners += [
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "-c", "/controller_manager", "--inactive"],
            )
        ]

    # Delay loading and activation of `joint_state_broadcaster` after start of ros2_control_node
    delay_joint_state_broadcaster_spawner_after_ros2_control_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[
                TimerAction(
                    period=3.0,
                    actions=[joint_state_broadcaster_spawner],
                ),
            ],
        )
    )

    # Delay loading and activation of robot_controller_names after `joint_state_broadcaster`
    delay_robot_controller_spawners_after_joint_state_broadcaster_spawner = []
    for i, controller in enumerate(robot_controller_spawners):
        delay_robot_controller_spawners_after_joint_state_broadcaster_spawner += [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=(
                        robot_controller_spawners[i - 1]
                        if i > 0
                        else joint_state_broadcaster_spawner
                    ),
                    on_exit=[controller],
                )
            )
        ]

    # Delay start of inactive_robot_controller_names after other controllers
    delay_inactive_robot_controller_spawners_after_joint_state_broadcaster_spawner = []
    for i, controller in enumerate(inactive_robot_controller_spawners):
        delay_inactive_robot_controller_spawners_after_joint_state_broadcaster_spawner += [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=(
                        inactive_robot_controller_spawners[i - 1]
                        if i > 0
                        else robot_controller_spawners[-1]
                    ),
                    on_exit=[controller],
                )
            )
        ]

    rviz_node = Node(
        condition=UnlessCondition(LaunchConfiguration("real_world")),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription(
        declared_arguments
        + [
            robot_state_pub_node,
            control_node,
            rviz_node,
            delay_joint_state_broadcaster_spawner_after_ros2_control_node,
        ]
        + delay_robot_controller_spawners_after_joint_state_broadcaster_spawner
        + delay_inactive_robot_controller_spawners_after_joint_state_broadcaster_spawner
    )
