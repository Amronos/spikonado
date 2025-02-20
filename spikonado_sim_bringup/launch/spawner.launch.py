# Copyright 2025 Aarav Gupta
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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('spikonado_sim_bringup')
    description_pkg_share = get_package_share_directory('spikonado_description')
    bringup_pkg_share = get_package_share_directory('spikonado_bringup')

    rviz_config_file = os.path.join(pkg_share, 'config', 'rviz_config.rviz')

    rviz_rsp_launch_source = os.path.join(
        description_pkg_share, 'launch', 'display.launch.py'
    )
    ros_gz_bridge_launch_source = os.path.join(
        pkg_share, 'launch', 'utils', 'ros_gz_bridge.launch.py'
    )
    controller_spawner_launch_source = os.path.join(
        bringup_pkg_share, 'launch', 'controller_spawner.launch.py'
    )

    rviz_rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rviz_rsp_launch_source),
        launch_arguments={
            'rviz_config_file': rviz_config_file,
            'use_sim_time': 'true',
        }.items(),
    )

    ros_gz_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ros_gz_bridge_launch_source)
    )

    spawn_model = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name',
            'spikonado',
            '-topic',
            'robot_description',
            '-z',
            '0.025',
        ],
        output='screen',
    )

    controller_spawner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(controller_spawner_launch_source)
    )

    return LaunchDescription([
        ros_gz_bridge,
        rviz_rsp,
        spawn_model,
        controller_spawner,
    ])
