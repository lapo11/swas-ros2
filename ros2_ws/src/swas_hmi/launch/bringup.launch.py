"""Operator services only. Does not launch a robot, demo, controller or gate."""
import os
import socket
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    def enabled(name):
        return value(name).lower() == 'true'

    def port(name):
        try:
            number = int(value(name))
        except ValueError as error:
            raise RuntimeError(f'{name} must be an integer') from error
        if not 1 <= number <= 65535:
            raise RuntimeError(f'{name} must be between 1 and 65535')
        return number

    frontend = enabled('serve_hmi')
    bridge = enabled('launch_rosbridge')
    video = enabled('launch_video_server')
    http_port = port('hmi_port')
    bridge_port = port('rosbridge_port')
    video_port = port('video_port')
    bind = value('hmi_bind_address')
    web_root = os.path.join(get_package_share_directory('swas_hmi'), 'web')
    if frontend and not os.path.isfile(os.path.join(web_root, 'index.html')):
        raise RuntimeError('Installed frontend missing. Build npm dist, then rebuild swas_hmi with --cmake-force-configure and source install/setup.bash.')

    # This is a helpful early check, not race-free port reservation or readiness.
    listeners = []
    if frontend:
        listeners.append(('HMI HTTP', bind, http_port, 'serve_hmi'))
    if bridge:
        listeners.append(('rosbridge', '0.0.0.0', bridge_port, 'launch_rosbridge'))
    if video:
        listeners.append(('video server', '0.0.0.0', video_port, 'launch_video_server'))
    ports = [item[2] for item in listeners]
    if len(ports) != len(set(ports)):
        raise RuntimeError('Enabled HMI services must use distinct port numbers.')
    for label, address, number, flag in listeners:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
                probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                probe.bind((address, number))
        except OSError as error:
            raise RuntimeError(f'{label} cannot bind {address}:{number}. Stop the existing service, change the port, or pass {flag}:=false. {error}') from error

    actions = []
    if frontend:
        actions.extend([
            LogInfo(msg=f'SWAS HMI: http://localhost:{http_port} (HTTP bind {bind}). Connect from the UI; no robot commands are issued by bringup.'),
            ExecuteProcess(
                cmd=[sys.executable, '-m', 'http.server', str(http_port), '--bind', bind, '--directory', web_root],
                name='operator_web_server', output='screen',
            ),
        ])
    if bridge:
        bridge_launch = os.path.join(get_package_share_directory('rosbridge_server'), 'launch', 'rosbridge_websocket_launch.xml')
        actions.append(IncludeLaunchDescription(
            AnyLaunchDescriptionSource(bridge_launch),
            launch_arguments={'port': str(bridge_port)}.items(),
        ))
    if video:
        actions.append(Node(
            package='web_video_server', executable='web_video_server', name='web_video_server',
            output='screen', parameters=[{'port': video_port, 'use_sim_time': False}],
        ))
    if not actions:
        actions.append(LogInfo(msg='All operator services disabled. Nothing to start.'))
    return actions


def generate_launch_description():
    booleans = {'serve_hmi': 'true', 'launch_rosbridge': 'true', 'launch_video_server': 'true'}
    values = {'hmi_port': '5173', 'hmi_bind_address': '127.0.0.1', 'rosbridge_port': '9090', 'video_port': '8080'}
    declarations = [DeclareLaunchArgument(k, default_value=v, choices=['true', 'false']) for k,v in booleans.items()]
    declarations.extend(DeclareLaunchArgument(k, default_value=v) for k,v in values.items())
    return LaunchDescription(declarations + [OpaqueFunction(function=setup)])
