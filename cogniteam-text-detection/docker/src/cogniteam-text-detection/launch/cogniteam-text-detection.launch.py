#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_arg =  LaunchConfiguration('model', default='src/cogniteam-text-detection/resource/frozen_east_text_detection.pb')
    width_arg = LaunchConfiguration('width', default='640')
    height_arg = LaunchConfiguration('height', default='480')
    thr_arg = LaunchConfiguration('thr', default='0.5')
    nms_arg = LaunchConfiguration('nms', default='0.4')
    device_arg = LaunchConfiguration('device', default='cpu')


    return LaunchDescription([

        DeclareLaunchArgument(
            'model',
            default_value=model_arg,
            description='Path to a binary .pb file contains trained network.'),

        DeclareLaunchArgument(
            'width',
            default_value=width_arg,
            description='Preprocess input image by resizing to a specific width. It should be multiple by 32.'),

        DeclareLaunchArgument(
            'height',
            default_value=height_arg,
            description='Preprocess input image by resizing to a specific height. It should be multiple by 32.'),
        
        DeclareLaunchArgument(
            'thr',
            default_value=thr_arg,
            description='Confidence threshold.'),

        DeclareLaunchArgument(
            'nms',
            default_value=nms_arg,
            description='Non-maximum suppression threshold.'),

        DeclareLaunchArgument(
            'device',
            default_value=device_arg,
            description='Device to run Deep Learning inference.'),


        Node(
            package='cogniteam-text-detection',
            executable='text_detection',
            name='textDetection_Node',
            parameters=[{'model':model_arg,
                         'width': width_arg, 
                         'height': height_arg, 
                         'thr': thr_arg,
                         'nms': nms_arg, 
                         'device': device_arg,}],
            output='screen'),
    ])

