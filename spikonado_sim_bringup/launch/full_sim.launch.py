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


def generate_launch_description():
    pkg_share = get_package_share_directory('spikonado_sim_bringup')

    simulator_launch_source = os.path.join(pkg_share, 'launch', 'simulator.launch.py')
    spawner_launch_source = os.path.join(pkg_share, 'launch', 'spawner.launch.py')

    simulator = IncludeLaunchDescription(PythonLaunchDescriptionSource(simulator_launch_source))
    spawner = IncludeLaunchDescription(PythonLaunchDescriptionSource(spawner_launch_source))

    return LaunchDescription([
        simulator,
        spawner,
    ])
