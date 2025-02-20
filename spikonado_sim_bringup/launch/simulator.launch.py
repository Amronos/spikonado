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
from launch.actions import ExecuteProcess
from ros_gz_sim.actions import GzServer


def generate_launch_description():
    pkg_share = get_package_share_directory('spikonado_sim_bringup')

    world_path = os.path.join(pkg_share, 'worlds', 'basic.sdf')

    # Launch just the Gazebo server as a composable node.
    gz_server = GzServer(
        world_sdf_file=world_path,
        container_name='ros_gz_container',
        create_own_container='True',
        use_composition='True',
    )

    gz_gui = ExecuteProcess(cmd=['gz', 'sim', '-g'], output='screen')

    return LaunchDescription([
        gz_server,
        gz_gui,
    ])
