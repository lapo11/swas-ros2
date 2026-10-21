import math
import os
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.dirname(__file__))

from bru_markers import attach_front_markers

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    sim = get_package_share_directory('swas_simulation')
    description = get_package_share_directory('swas_description')
    def number(name):
        value = float(LaunchConfiguration(name).perform(context))
        if not math.isfinite(value):
            raise RuntimeError(f'{name} must be finite.')
        return value
    def flag(name):
        return LaunchConfiguration(name).perform(context).lower() == 'true'

    mx, my, mz = [number(k) for k in ('marker_x', 'marker_y', 'marker_z')]
    mount_yaw = math.radians(number('marker_yaw_deg'))
    target_file = os.path.join(sim, 'config', 'front_marker_target.sdf')
    bru = ET.fromstring(xacro.process_file(os.path.join(
        description, 'urdf', 'bru.urdf.xacro')).toxml())
    attach_front_markers(bru, target_file, (mx, my, mz), mount_yaw)
    bru_xml = ET.tostring(bru, encoding='unicode')


    actions = [
        LogInfo(msg=(
            f'Markers relative to BRU: '
            f'xyz=({mx:.3f}, {my:.3f}, {mz:.3f}), '
            f'yaw={math.degrees(mount_yaw):.1f} deg'
        )),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    sim, 'launch', 'docking_sim.launch.py'
                )
            ),
            launch_arguments={
                'preset': LaunchConfiguration('preset'),
                'bru_description_override': bru_xml,
            }.items(),
        ),
    ]

    if flag('vision'):
        system = get_package_share_directory('swas_system')
        actions.append(Node(
            package='swas_system', executable='bxu_front_vision_node',
            name='bxu_front_vision_node', output='screen',
            parameters=[os.path.join(system, 'config', 'bxu_front_vision.yaml'), {
                'use_sim_time': True, 'input_source': 'ros',
                'image_topic': '/bxu/front_camera/image_raw',
                'camera_info_topic': '/bxu/front_camera/camera_info',
                'publish_overlay_image': True,
            }]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('marker_x', default_value='-0.75'),
        DeclareLaunchArgument('marker_y', default_value='0.0'),
        DeclareLaunchArgument('marker_z', default_value='0.08'),
        DeclareLaunchArgument(
            'marker_yaw_deg', default_value='0.0'
        ),
        DeclareLaunchArgument('preset', default_value='manual'),
        DeclareLaunchArgument(
            'vision',
            default_value='false',
            choices=['true', 'false'],
        ),
        OpaqueFunction(function=setup),
    ])