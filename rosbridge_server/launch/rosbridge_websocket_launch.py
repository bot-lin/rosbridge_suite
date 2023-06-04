import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    address = LaunchConfiguration('address')
    port = LaunchConfiguration('port')
    retry_startup_delay = LaunchConfiguration('retry_startup_delay')
    max_message_size = LaunchConfiguration('max_message_size')
    fragment_timeout = LaunchConfiguration('fragment_timeout')
    delay_between_messages = LaunchConfiguration('delay_between_messages')
    unregister_timeout = LaunchConfiguration('unregister_timeout')
    use_compression = LaunchConfiguration('use_compression')
    bson_only_mode = LaunchConfiguration('bson_only_mode')
    topics_glob = LaunchConfiguration('topics_glob')
    
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')
    
    declare_topics_glob_cmd = DeclareLaunchArgument(
        'topics_glob',
        default_value='robot_pose',
        description='topics_glob')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')
    
    declare_port_cmd = DeclareLaunchArgument(
        'port',
        default_value='9090',
        description='Websocket port')

    declare_address_cmd = DeclareLaunchArgument(
        'address',
        default_value='0.0.0.0',
        description='Websocket address')
    
    declare_retry_startup_delay_cmd = DeclareLaunchArgument(
        'retry_startup_delay',
        default_value='5.0',
        description="retry_startup_delay")

    declare_fragment_timeout_cmd = DeclareLaunchArgument(
        'fragment_timeout',
        default_value='600',
        description="fragment_timeout")

    declare_max_message_size_cmd = DeclareLaunchArgument(
        'max_message_size',
        default_value='10000000',
        description="max_message_size")

    declare_unregister_timeout_cmd = DeclareLaunchArgument(
        'unregister_timeout',
        default_value='10.0',
        description="unregister_timeout")

    declare_delay_between_messages_cmd = DeclareLaunchArgument(
        'delay_between_messages',
        default_value='0',
        description="delay_between_messages")

    declare_use_compression_cmd = DeclareLaunchArgument(
        'use_compression',
        default_value='true',
        description="use_compression")

    declare_bson_only_mode_cmd = DeclareLaunchArgument(
        'bson_only_mode',
        default_value='false',
        description="bson_only_mode")

    start_websocket_cmd = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        namespace=namespace,
        output='screen',
        parameters=[{'port': port,
                    'address': address,
                     'retry_startup_delay': retry_startup_delay,
                     'max_message_size': max_message_size,
                     'fragment_timeout' : fragment_timeout,
                     'delay_between_messages' : delay_between_messages,
                     'unregister_timeout' : unregister_timeout,
                     'use_compression': use_compression,
                     'bson_only_mode' : bson_only_mode,
                     'topics_glob' : topics_glob}])
    
    return LaunchDescription([
        declare_namespace_cmd,
        declare_use_namespace_cmd,
        declare_port_cmd,
        declare_address_cmd,
        declare_retry_startup_delay_cmd,
        declare_max_message_size_cmd,
        declare_bson_only_mode_cmd,
        declare_use_compression_cmd,
        declare_delay_between_messages_cmd,
        declare_unregister_timeout_cmd,
        declare_fragment_timeout_cmd,
        declare_topics_glob_cmd,
        start_websocket_cmd,
    ])