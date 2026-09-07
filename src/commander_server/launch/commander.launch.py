# Copyright 2026 kineticsystem
# SPDX-License-Identifier: MIT

"""Start the commander action server."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    parameters = PathJoinSubstitution(
        [FindPackageShare("commander_server"), "config", "commander_server.yaml"]
    )

    commander_server = Node(
        package="commander_server",
        executable="commander_server",
        name="commander_server",
        output="screen",
        parameters=[parameters],
    )

    return LaunchDescription([commander_server])
