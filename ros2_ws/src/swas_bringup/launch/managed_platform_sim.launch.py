"""Additive simulation session: existing manual platform + tested vision registry.
No app auto-load, controller, supervisor, AUTO or lift actuation is introduced.
Do not start another platform or manager alongside this launch.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Forward overrides only; the existing platform/inspection launch retains ownership
# of camera, pose, preset and server defaults.
FORWARD = (
    'scene','world','with_hmi','preset',
    'bxu_x','bxu_y','bxu_yaw','bru_x','bru_y','bru_z','bru_yaw',
    'x_offset','y_offset','yaw_offset_deg','bru_joint_gui',
    'serve_hmi','hmi_port','hmi_bind_address',
    'launch_rosbridge','rosbridge_port','launch_video_server','video_port',
)

def setup(context):
    platform=os.path.join(get_package_share_directory('swas_bringup'),'launch','manual_platform_sim.launch.py')
    if not os.path.isfile(platform):
        raise RuntimeError('Existing manual platform launch missing: '+platform)
    arguments={name:LaunchConfiguration(name).perform(context)
               for name in FORWARD if name in context.launch_configurations}
    # Always suppress the older front-only manager within the included platform.
    arguments['with_component_manager']='false'
    return [
        LogInfo(msg='MANAGED PERCEPTION SESSION: existing manual-only platform plus vision registry. No applications load automatically. Do not run the older manager or another session simultaneously.'),
        GroupAction(scoped=True,actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(platform),
            launch_arguments=arguments.items(),
        )]),
        Node(
            package='swas_system',
            executable='bxu_vision_registry_manager',
            name='bxu_vision_registry_manager',
            namespace='/',
            output='screen',
            parameters=[{'use_sim_time':False}],
        ),
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scene',default_value='markers',choices=['plain','markers','tesla']),
        DeclareLaunchArgument('world',default_value='empty',choices=['empty','station','station_w_ceiling']),
        DeclareLaunchArgument('with_hmi',default_value='true',choices=['true','false']),
        OpaqueFunction(function=setup),
    ])
