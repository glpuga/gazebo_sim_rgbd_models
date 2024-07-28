#!/usr/bin/python3

# Copyright 2024 Gerardo Puga.
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


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    urdf_models_path = os.path.join(
        get_package_share_directory("yolo_rplidar_models"), "urdf", "testbenchs"
    )
    available_models = [
        x for x in os.listdir(urdf_models_path) if x.startswith("sensor_")
    ]

    camera_model_arg = DeclareLaunchArgument(
        "lidar_model",
        choices=available_models,
    )

    camera_model_conf = LaunchConfiguration("lidar_model")

    rviz_config_filepath = PathJoinSubstitution(
        [FindPackageShare("yolo_rplidar_models"), "rviz", "display.rviz"]
    )

    urdf_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("urdf_launch"), "launch", "display.launch.py"]
                )
            ]
        ),
        launch_arguments={
            "urdf_package": "yolo_rplidar_models",
            "urdf_package_path": PathJoinSubstitution(
                ["urdf", "testbenchs", camera_model_conf]
            ),
            "rviz_config": rviz_config_filepath,
            "jsp_gui": "false",
        }.items(),
    )

    return LaunchDescription(
        [
            camera_model_arg,
            urdf_launch_include,
        ]
    )
