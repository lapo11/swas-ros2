"""Preparation ONLY: existing marker approach controller, disconnected from motion.
Requires existing simulation platform/perception. Never includes a world, gate or
supervisor. Do not publish true to the private enable input in this checkpoint.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    config = LaunchConfiguration('controller_params').perform(context)
    if not os.path.isfile(config):
        raise RuntimeError('Controller parameter file missing: '+config)
    return [
        LogInfo(msg=(
            'PREPARATION ONLY: marker approach starts disabled under the existing node logic. '
            'Start/stop services are disabled, enable input and velocity output are isolated. '
            'No motion gate connection is created. Do not enable it in this checkpoint.'
        )),
        Node(
            package='swas_system',
            executable='bxu_front_approach_controller_node',
            name='bxu_front_approach_controller_node',
            namespace='/',
            output='screen',
            parameters=[config, {
                'use_sim_time': True,
                'enable_control_services': False,
            }],
            remappings=[
                ('/bxu/cmd_vel', '/bxu/controllers/front_markers/cmd_vel'),
                ('/bxu/front_controller/enabled', '/bxu/controllers/front_markers/enabled'),
                ('/bxu/front_approach/controller_status', '/bxu/controllers/front_markers/status'),
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'controller_params',
            default_value=os.path.join(
                get_package_share_directory('swas_system'),
                'config',
                'bxu_front_approach_controller.yaml',
            ),
        ),
        OpaqueFunction(function=setup),
    ])
