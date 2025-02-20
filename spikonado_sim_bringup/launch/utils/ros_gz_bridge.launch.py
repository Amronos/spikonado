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
from ros_gz_bridge.actions import RosGzBridge


def generate_launch_description():
    pkg_share = get_package_share_directory('spikonado_sim_bringup')

    bridge_config_path = os.path.join(pkg_share, 'config', 'bridge_config.yaml')

    # Setup ros_gz_bridge to bridge topics between ROS and Gazebo.
    # It is launched as a composable node in the container created by the Gazebo server.
    ros_gz_bridge = RosGzBridge(
        bridge_name='ros_gz_bridge',
        config_file=bridge_config_path,
        container_name='ros_gz_container',
        create_own_container='False',
        use_composition='True',
    )

    return LaunchDescription([
        ros_gz_bridge,
    ])
