"""One-terminal marker-approach preparation session.

Composes the existing managed platform, isolated disabled controller,
preparation monitor and preparation-query coordinator. It does not load
perception applications, enable a controller, publish velocity, grant AUTO
authority, or start a supervisor.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PLATFORM_FORWARD = (
    'scene','world','preset',
    'bxu_x','bxu_y','bxu_yaw','bru_x','bru_y','bru_z','bru_yaw',
    'x_offset','y_offset','yaw_offset_deg','bru_joint_gui',
    'launch_rosbridge','rosbridge_port','launch_video_server','video_port',
)


def setup(context):
    platform = os.path.join(
        get_package_share_directory('swas_bringup'),
        'launch',
        'managed_platform_sim.launch.py',
    )
    if not os.path.isfile(platform):
        raise RuntimeError('Managed platform launch missing: '+platform)

    config = LaunchConfiguration('controller_params').perform(context)
    if not os.path.isfile(config):
        raise RuntimeError('Controller parameter file missing: '+config)

    arguments = {
        name: LaunchConfiguration(name).perform(context)
        for name in PLATFORM_FORWARD
        if name in context.launch_configurations
    }
    arguments['with_hmi'] = 'true'
    arguments['serve_hmi'] = 'true'
    arguments['with_component_manager'] = 'false'

    return [
        LogInfo(msg=(
            'MARKER APPROACH PREPARATION SESSION: one managed platform, integrated HMI, '
            'vision registry, isolated disabled controller, preparation monitor and '
            'preparation-query coordinator. Load perception from the HMI. '
            'No controller enable, AUTO authority, supervisor or lift actuation is started.'
        )),
        GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(platform),
            launch_arguments=arguments.items(),
        )]),
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
        Node(
            package='swas_system',
            executable='bxu_marker_approach_preparation_node',
            name='bxu_marker_approach_preparation_node',
            namespace='/',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='swas_system',
            executable='bxu_marker_approach_motion_relay_node',
            name='bxu_marker_approach_motion_relay_node',
            namespace='/',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='swas_system',
            executable='bxu_marker_approach_operation_node',
            name='bxu_marker_approach_operation_node',
            namespace='/',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scene', default_value='markers', choices=['plain', 'markers', 'tesla']),
        DeclareLaunchArgument('world', default_value='station', choices=['empty', 'station', 'station_w_ceiling']),
        DeclareLaunchArgument('controller_params', default_value=os.path.join(
            get_package_share_directory('swas_system'),
            'config',
            'bxu_front_approach_controller.yaml',
        )),
        OpaqueFunction(function=setup),
    ])
