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

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'spikonado_description'
    pkg_share_directory = get_package_share_directory(pkg_name)

    # Check if the nodes need to use simulation time
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='If true, use simulated clock.'
    )

    # Declare launch argument for the model file to display
    declare_model_file = DeclareLaunchArgument(
        'model_file',
        default_value=os.path.join(pkg_share_directory, 'description', 'sdf', 'model.sdf.xacro'),
        description='Path to the model file to use.'
    )

    # Get the config file for RViz
    declare_rviz_config_file = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(pkg_share_directory, 'config', 'rviz_config.rviz'),
        description='Path to RViz config file.'
    )
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    # Define the node for RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
        arguments=['-d', rviz_config_file],
    )

    def robot_state_publisher(context):
        # Get the model file and process it with xacro
        model_file = LaunchConfiguration('model_file').perform(context)
        xacro_processed = xacro.process(model_file)

        # Define the node for the robot_state_publisher
        node_robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'robot_description': xacro_processed},
            ],
        )
        return [node_robot_state_publisher]

    ld = LaunchDescription([
        declare_model_file,
        declare_rviz_config_file,
        declare_use_sim_time,
        rviz,
    ])
    ld.add_action(OpaqueFunction(function=robot_state_publisher))
    return ld
