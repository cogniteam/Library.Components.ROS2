import os

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument ,SetEnvironmentVariable,LogInfo
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
   x = os.path.join(
         get_package_share_directory('nav2_bt_navigator'),
         'behavior_trees', 'navigate_w_replanning_and_recovery.xml')
   print(x)
   stdout_linebuf_envvar = SetEnvironmentVariable(
                           'RCUTILS_LOGGING_BUFFERED_STREAM', '1')
   # Get the launch directory
   bringup_dir = get_package_share_directory('nav2_bringup')
   print(bringup_dir)
################################################ SLAM Argument #############################################################
   
   declare_params_file_slam = DeclareLaunchArgument(
                              'params_file_slam',
                              default_value=os.path.join(get_package_share_directory("slam_toolbox"),
                                                         'config', 'mapper_params_online_async.yaml'),
                              description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

################################################ Nav2 Argument #############################################################
   
   declare_namespace_cmd = DeclareLaunchArgument(
      'namespace',
      default_value='',
      description='Top-level namespace')

   declare_use_namespace_cmd = DeclareLaunchArgument(
      'use_namespace',
      default_value='false',
      description='Whether to apply a namespace to the navigation stack')

   declare_slam_cmd = DeclareLaunchArgument(
      'slam',
      default_value='False',
      description='Whether run a SLAM')

   declare_map_yaml_cmd = DeclareLaunchArgument(
      'map',
      default_value='',
      description='Full path to map yaml file to load')

   declare_params_file_cmd = DeclareLaunchArgument(
      'params_file',
      default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
      description='Full path to the ROS2 parameters file to use for all launched nodes')
   
   declare_use_sim_time_cmd = DeclareLaunchArgument(
    'use_sim_time',
    default_value='false',
    description='Use simulation (Gazebo) clock if true')

   declare_bt_xml_cmd = DeclareLaunchArgument(
      'default_bt_xml_filename',
      default_value=os.path.join(
         get_package_share_directory('nav2_bt_navigator'),
         'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
      description='Full path to the behavior tree xml file to use')

   declare_autostart_cmd = DeclareLaunchArgument(
      'autostart', default_value='true',
      description='Automatically startup the nav2 stack')


   # Create the launch configuration variables
   namespace = LaunchConfiguration('namespace')
   use_namespace = LaunchConfiguration('use_namespace')
   slam = LaunchConfiguration('slam')
   map_yaml_file = LaunchConfiguration('map')
   use_sim_time = LaunchConfiguration('use_sim_time')
   params_file = LaunchConfiguration('params_file')
   default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
   autostart = LaunchConfiguration('autostart')
   params_file_slam = LaunchConfiguration('params_file_slam')

   nav2_bringup = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('nav2_bringup'), 'launch'),
         '/bringup_launch.py']),
   		launch_arguments={
            'namespace': namespace,
            'use_namespace': use_namespace,
            'slam': slam,
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'default_bt_xml_filename': default_bt_xml_filename,
            'autostart': autostart
            }.items(),
      )
   
   slam_toolbox = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('slam_toolbox'), 'launch'),
         '/online_async_launch.py']),
         launch_arguments={
            'use_sim_time': use_sim_time,
            "params_file" : params_file_slam
            }.items(),
      )
   return LaunchDescription([
      declare_params_file_cmd,
      stdout_linebuf_envvar,
      declare_namespace_cmd,
      declare_use_namespace_cmd,
      declare_slam_cmd,
      declare_map_yaml_cmd,
      declare_use_sim_time_cmd,
      declare_params_file_slam,
      declare_bt_xml_cmd,
      declare_autostart_cmd,
      slam_toolbox,
      nav2_bringup,
   ])

