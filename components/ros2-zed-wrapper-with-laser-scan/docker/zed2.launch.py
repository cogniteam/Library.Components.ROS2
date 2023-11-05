# Copyright 2022 Stereolabs
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

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution,LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    # Camera model (force value)
    camera_model = 'zed2'

    # ZED Wrapper node
    zed_wrapper_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('zed_wrapper'),
            '/launch/include/zed_camera.launch.py'
        ]),
        launch_arguments={
            'camera_model': camera_model
        }.items()
    )

    # ZED Wrapper node
    zed_wrapper_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('zed_wrapper'),
            '/launch/include/zed_camera.launch.py'
        ]),
        launch_arguments={
            'camera_model': camera_model
        }.items()
    )
    
    # args that can be set from the command line or a default will be used
    scan_height_arg = DeclareLaunchArgument(
        "scan_height", default_value=TextSubstitution(text="1")
    )

    depthimage_to_laserscan_node = Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan_node',
            parameters=[{
                "scan_height": LaunchConfiguration('scan_height'),
                "output_frame":"zed2_left_camera_optical_frame"
            }],
            remappings=[
            ('/depth', '/zed2/zed_node/depth/depth_registered'),
            ('/depth_camera_info', '/zed2/zed_node/depth/camera_info'),
            ]
        )
    # Define LaunchDescription variable
    ld = LaunchDescription()

    # Add nodes to LaunchDescription
    ld.add_action(zed_wrapper_launch)
    ld.add_action(scan_height_arg)
    ld.add_action(depthimage_to_laserscan_node)
    return ld