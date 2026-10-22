"""Additive inspection session: one existing scene + optional operator services.
NO gate, lift, controller, supervisor, runtime manager or motion-enable call.
Stop existing simulations/legacy demos before using this session in the same ROS domain.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

SCENES = {
    'plain': 'docking_sim.launch.py',
    'markers': 'docking_bru_markers_sim.launch.py',
    'tesla': 'docking_tesla_sim.launch.py',
}
BASE_DEFAULTS = {
    'world': 'empty', 'preset': 'manual',
    'bxu_x': '0.0', 'bxu_y': '0.0', 'bxu_yaw': '0.0',
    'bru_x': '3.0', 'bru_y': '0.0', 'bru_z': '0.15', 'bru_yaw': '0.0',
    'x_offset': '0.0', 'y_offset': '0.0', 'yaw_offset_deg': '0.0',
    'bru_joint_gui': 'false',
}
HMI_DEFAULTS = {
    'serve_hmi': 'true', 'hmi_port': '5173', 'hmi_bind_address': '127.0.0.1',
    'launch_rosbridge': 'true', 'rosbridge_port': '9090',
    'launch_video_server': 'true', 'video_port': '8080',
}

def setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)
    scene = value('scene')
    if scene not in SCENES:
        raise RuntimeError('Unsupported inspection scene: '+scene)
    scene_path = os.path.join(get_package_share_directory('swas_simulation'), 'launch', SCENES[scene])
    if not os.path.isfile(scene_path):
        raise RuntimeError('Installed scene launch missing: '+scene_path)
    actions = [
        LogInfo(msg='INSPECTION ONLY: starts scene/cameras/TF and optional HMI. No control gate, lift, supervisor or application is started. Do not command motion. This is not the final managed runtime.'),
        GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(scene_path),
            launch_arguments={name:value(name) for name in BASE_DEFAULTS}.items(),
        )]),
    ]
    if value('with_hmi').lower() == 'true':
        hmi_path = os.path.join(get_package_share_directory('swas_hmi'), 'launch', 'bringup.launch.py')
        if not os.path.isfile(hmi_path):
            raise RuntimeError('Generic HMI bringup missing: '+hmi_path)
        actions.append(GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hmi_path),
            launch_arguments={name:value(name) for name in HMI_DEFAULTS}.items(),
        )]))
    return actions

def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('scene', default_value='markers', choices=list(SCENES)),
        DeclareLaunchArgument('with_hmi', default_value='true', choices=['true', 'false']),
    ]
    arguments.extend(DeclareLaunchArgument(name, default_value=default) for name,default in BASE_DEFAULTS.items())
    arguments.extend(DeclareLaunchArgument(name, default_value=default) for name,default in HMI_DEFAULTS.items())
    return LaunchDescription(arguments+[OpaqueFunction(function=setup)])
