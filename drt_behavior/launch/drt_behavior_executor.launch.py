#!/usr/bin/env python3
#
# Copyright (c) 2025, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
#
# All rights reserved.
#
# This software is licensed under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):

    # YAML file containing BT executor parameters
    package_name = LaunchConfiguration("package_name").perform(context)
    file_name = LaunchConfiguration("file_name").perform(context)

    bt_config = os.path.join(get_package_share_directory(package_name), "config", file_name)

    # Behavior executor node
    behavior_executor = Node(
        package="drt_behavior",
        executable="demo",
        output="both",
        parameters=[bt_config],
    )

    return [behavior_executor]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "package_name",
            default_value="drt_behavior",
            description="Name of the package in which to search for the config file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "file_name",
            default_value="sample_bt_executor.yaml",
            description="Name of the file in the config directory.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
