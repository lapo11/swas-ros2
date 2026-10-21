import math
import os
import sys
import xml.etree.ElementTree as ET

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

sys.path.insert(0, os.path.dirname(__file__))
from bru_tesla import attach_tesla


# Forward these unchanged to docking_sim.launch.py.
BASE_DEFAULTS = {
    'world': 'empty',
    'preset': 'manual',
    'bxu_x': '0.0',
    'bxu_y': '0.0',
    'bxu_yaw': '0.0',
    'bru_x': '3.0',
    'bru_y': '0.0',
    'bru_z': '0.15',
    'bru_yaw': '0.0',
    'x_offset': '0.0',
    'y_offset': '0.0',
    'yaw_offset_deg': '0.0',
    'bru_joint_gui': 'false',
}


def setup(context):
    sim = get_package_share_directory('swas_simulation')
    description = get_package_share_directory('swas_description')

    def number(name):
        value = float(LaunchConfiguration(name).perform(context))
        if not math.isfinite(value):
            raise RuntimeError(f'{name} must be finite.')
        return value

    xyz = tuple(number(name) for name in ('tesla_x', 'tesla_y', 'tesla_z'))
    rpy = tuple(math.radians(number(name)) for name in (
        'tesla_roll_deg', 'tesla_pitch_deg', 'tesla_yaw_deg',
    ))
    source_sdf = LaunchConfiguration('tesla_sdf').perform(context)
    override = LaunchConfiguration('bru_description_override').perform(context)
    if override.strip():
        bru = ET.fromstring(override)
    else:
        bru = ET.fromstring(xacro.process_file(os.path.join(
            description, 'urdf', 'bru.urdf.xacro'
        )).toxml())

    attach_tesla(bru, source_sdf, xyz, rpy)
    arguments = {
        name: LaunchConfiguration(name).perform(context)
        for name in BASE_DEFAULTS
    }
    arguments['bru_description_override'] = ET.tostring(bru, encoding='unicode')

    return [
        LogInfo(msg=(
            f'Tesla attached to bru_base_link: xyz={xyz}, '
            f'rpy_deg={tuple(round(math.degrees(v), 3) for v in rpy)}. '
            f'Using mesh scale and visual offset from {source_sdf}'
        )),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                sim, 'launch', 'docking_sim.launch.py'
            )),
            launch_arguments=arguments.items(),
        ),
    ]


def generate_launch_description():
    declarations = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in BASE_DEFAULTS.items()
    ]
    declarations.extend([
        DeclareLaunchArgument(
            'tesla_sdf',
            default_value=os.path.join(
                get_package_share_directory('swas_simulation'),
                'config',
                'tesla_visual.sdf',
            ),
        ),
        DeclareLaunchArgument('tesla_x', default_value='0.0'),
        DeclareLaunchArgument('tesla_y', default_value='-0.11'),
        DeclareLaunchArgument('tesla_z', default_value='-0.15'),
        DeclareLaunchArgument('tesla_roll_deg', default_value='0.0'),
        DeclareLaunchArgument('tesla_pitch_deg', default_value='0.0'),
        DeclareLaunchArgument('tesla_yaw_deg', default_value='0.0'),
        DeclareLaunchArgument('bru_description_override', default_value=''),
    ])
    return LaunchDescription(declarations + [OpaqueFunction(function=setup)])
