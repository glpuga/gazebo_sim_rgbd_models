# Copyright 2024 Gerardo Puga
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


from ament_index_python.packages import get_package_share_directory

from launch.actions import OpaqueFunction

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    world_conf = LaunchConfiguration("world")
    urdf_wrapper_basename_conf = LaunchConfiguration("urdf_wrapper_basename")
    rviz_file_conf = LaunchConfiguration("rviz_config")
    bridge_file_conf = LaunchConfiguration("bridge_config")

    this_pkg_share = FindPackageShare("yolo_rplidar_models_testbench")

    world_file_path = PathJoinSubstitution([this_pkg_share, "worlds", world_conf])
    rviz_config_file_path = PathJoinSubstitution(
        [this_pkg_share, "rviz", rviz_file_conf]
    )
    bridge_config_file_path = PathJoinSubstitution(
        [this_pkg_share, "config", bridge_file_conf]
    )

    urdf_wrapper_rel_path = PathJoinSubstitution(
        ["urdf", "testbenchs", urdf_wrapper_basename_conf]
    )

    gzserver_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={
            "gz_args": ["-r -s -v4 ", world_file_path],
            "on_exit_shutdown": "true",
        }.items(),
    )

    gzclient_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={"gz_args": "-g -v4 "}.items(),
    )

    gazebo_ros_bride_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={bridge_config_file_path.perform(context)}",
        ],
        output="screen",
    )

    robot_spawner_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "test_device",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "1.1",
        ],
        output="screen",
    )

    urdf_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("urdf_launch"), "launch", "display.launch.py"]
            )
        ),
        launch_arguments={
            "urdf_package": "yolo_rplidar_models",
            "urdf_package_path": urdf_wrapper_rel_path,
            "rviz_config": rviz_config_file_path,
            "jsp_gui": "false",
        }.items(),
    )

    return [
        gzserver_launch_include,
        gzclient_launch_include,
        gazebo_ros_bride_node,
        robot_spawner_node,
        urdf_launch_include,
    ]


def generate_launch_description():
    world_arg = DeclareLaunchArgument("world", default_value="minimal.sdf")
    urdf_wrapper_basename_arg = DeclareLaunchArgument("urdf_wrapper_basename")
    rviz_file_arg = DeclareLaunchArgument("rviz_config")
    bridge_file_arg = DeclareLaunchArgument("bridge_config")

    # The OpaqueFunction thingie is needed because we need to turn a single input argument
    # into a string in a way that substitutions do not support. ROS2 python launch files are
    # a thing of beauty, really.
    return LaunchDescription(
        [
            world_arg,
            urdf_wrapper_basename_arg,
            rviz_file_arg,
            bridge_file_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
