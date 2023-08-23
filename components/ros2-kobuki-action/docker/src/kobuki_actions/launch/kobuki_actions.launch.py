from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node



def generate_launch_description():

    # args that can be set from the command line or a default will be used
    json_path_arg = DeclareLaunchArgument(
        "jsonPath", default_value=TextSubstitution(text="src/kobuki_actions/resource/kobuki_actions.json")
    )


    kobuki_actions_node = Node(
            package='kobuki_actions',
            executable='kobuki_actions_node',
            name='kobuki_actions_node',
            parameters=[{
                "jsonPath": LaunchConfiguration('jsonPath')
            }]
        )

    return LaunchDescription([
        json_path_arg,
        kobuki_actions_node,
    ])