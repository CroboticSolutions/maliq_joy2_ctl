# Copyright 2022 Open Source Robotics Foundation, Inc.
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import os

def launch(context, *args, **kwargs):

    # https://index.ros.org/p/joy/ --> joy node as joystick (Create subscriber that takes cmd_vel)
    joy_node = Node(
        package='joy', 
        executable="joy_node", 
        output="screen", 
        arguments={'device_name':'js0'}.items()
    )

    joy_ctl_node = Node(
        package="joy2_ctl", 
        executable="joy_ctl", 
        output="screen" 
    )

    # Add spawning of UAVs
    return [joy_node,  
            joy_ctl_node, 
            ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function = launch)
        ])
