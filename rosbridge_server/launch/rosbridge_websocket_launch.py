import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    port = LaunchConfiguration('port')
    retry_startup_delay = LaunchConfiguration('retry_startup_delay')
    max_message_size = LaunchConfiguration('max_message_size')
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')
    
    declare_port_cmd = DeclareLaunchArgument(
        'port',
        default_value='9090',
        description='Websocket port')
    
    declare_retry_startup_delay_cmd = DeclareLaunchArgument(
        'retry_startup_delay',
        default_value='5.0',
        description="retry_startup_delay")

    declare_max_message_size_cmd = DeclareLaunchArgument(
        'max_message_size',
        default_value='10000000',
        description="max_message_size")


    start_websocket_cmd = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        namespace=namespace,
        output='screen',
        parameters=[{'port': port,
                     'retry_startup_delay': retry_startup_delay,
                     'max_message_size': max_message_size,}])
    
    return LaunchDescription([
        declare_namespace_cmd,
        declare_use_namespace_cmd,
        declare_port_cmd,
        declare_retry_startup_delay_cmd,
        declare_max_message_size_cmd,
        start_websocket_cmd
    ])