import math
import os
import sys
import xml.etree.ElementTree as ET

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

sys.path.insert(0, os.path.dirname(__file__))

from bru_markers import attach_docking_markers


def setup(context):
    sim = get_package_share_directory('swas_simulation')
    description = get_package_share_directory('swas_description')

    def number(name):
        value = float(LaunchConfiguration(name).perform(context))

        if not math.isfinite(value):
            raise RuntimeError(f'{name} must be finite.')

        return value

    marker_x = number('marker_x')
    marker_y_0 = number('marker_y_0')
    marker_y_1 = number('marker_y_1')
    marker_z = number('marker_z')
    marker_0_yaw_deg = number('marker_0_yaw_deg')
    marker_1_yaw_deg = number('marker_1_yaw_deg')

    target_file = os.path.join(
        sim,
        'config',
        'docking_markers_target.sdf',
    )

    bru = ET.fromstring(
        xacro.process_file(
            os.path.join(
                description,
                'urdf',
                'bru.urdf.xacro',
            )
        ).toxml()
    )

    attach_docking_markers(
        bru,
        target_file,
        marker_x,
        marker_y_0,
        marker_y_1,
        marker_z,
        marker_0_yaw_deg,
        marker_1_yaw_deg,
    )

    bru_xml = ET.tostring(bru, encoding='unicode')

    actions = [
        LogInfo(
            msg=(
                'Docking markers relative to BRU: '
                f'marker 0 xyz=({marker_x:.3f}, {marker_y_0:.3f}, {marker_z:.3f}), '
                f'yaw={marker_0_yaw_deg:.1f} deg; '
                f'marker 1 xyz=({marker_x:.3f}, {marker_y_1:.3f}, {marker_z:.3f}), '
                f'yaw={marker_1_yaw_deg:.1f} deg'
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    sim,
                    'launch',
                    'docking_sim.launch.py',
                )
            ),
            launch_arguments={
                'preset': LaunchConfiguration('preset'),
                'bru_description_override': bru_xml,
            }.items(),
        ),
    ]

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'marker_x',
                default_value='0.400',
            ),
            DeclareLaunchArgument(
                'marker_y_0',
                default_value='0.500',
            ),
            DeclareLaunchArgument(
                'marker_y_1',
                default_value='-0.500',
            ),
            DeclareLaunchArgument(
                'marker_z',
                default_value='-0.000',
            ),
            DeclareLaunchArgument(
                'marker_0_yaw_deg',
                default_value='180.0',
            ),
            DeclareLaunchArgument(
                'marker_1_yaw_deg',
                default_value='0.0',
            ),
            DeclareLaunchArgument(
                'preset',
                default_value='manual',
            ),
            OpaqueFunction(function=setup),
        ]
    )