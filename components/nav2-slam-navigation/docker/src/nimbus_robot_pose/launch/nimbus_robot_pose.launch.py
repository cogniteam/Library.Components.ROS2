from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node



def generate_launch_description():


    nimbus_robot_pose_node = Node(
            package='nimbus_robot_pose',
            executable='nimbus_robot_pose_node',
            name='nimbus_robot_pose_node',
        )

    return LaunchDescription([
        nimbus_robot_pose_node,
    ])