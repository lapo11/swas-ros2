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

from bru_markers import attach_front_markers, attach_docking_markers


def setup(context):
    sim = get_package_share_directory('swas_simulation')
    description = get_package_share_directory('swas_description')

    def number(name):
        value = float(LaunchConfiguration(name).perform(context))

        if not math.isfinite(value):
            raise RuntimeError(f'{name} must be finite.')

        return value

    # Front-marker mounting pose, relative to bru_base_link.
    front_x = number('front_marker_x')
    front_y = number('front_marker_y')
    front_z = number('front_marker_z')
    front_yaw_deg = number('front_marker_yaw_deg')

    # Docking-marker mounting poses, relative to bru_base_link.
    docking_x = number('docking_marker_x')
    docking_y_0 = number('docking_marker_y_0')
    docking_y_1 = number('docking_marker_y_1')
    docking_z = number('docking_marker_z')
    docking_yaw_0_deg = number('docking_marker_0_yaw_deg')
    docking_yaw_1_deg = number('docking_marker_1_yaw_deg')

    front_target_file = os.path.join(
        sim,
        'config',
        'front_marker_target.sdf',
    )
    docking_target_file = os.path.join(
        sim,
        'config',
        'docking_markers_target.sdf',
    )
    bru_xacro_file = os.path.join(
        description,
        'urdf',
        'bru.urdf.xacro',
    )

    for path in (
        front_target_file,
        docking_target_file,
        bru_xacro_file,
    ):
        if not os.path.isfile(path):
            raise RuntimeError(f'Required scene file not found: {path}')

    # Build one BRU description containing both marker sets.
    bru = ET.fromstring(
        xacro.process_file(bru_xacro_file).toxml()
    )

    attach_front_markers(
        bru,
        front_target_file,
        (front_x, front_y, front_z),
        math.radians(front_yaw_deg),
    )

    attach_docking_markers(
        bru,
        docking_target_file,
        docking_x,
        docking_y_0,
        docking_y_1,
        docking_z,
        docking_yaw_0_deg,
        docking_yaw_1_deg,
    )

    bru_xml = ET.tostring(bru, encoding='unicode')

    scene_arguments = {
        'preset': LaunchConfiguration('preset').perform(context),
        'bru_description_override': bru_xml,
    }

    # Forward supplied scene settings without duplicating the
    # defaults owned by docking_sim.launch.py.
    optional_scene_arguments = (
        'bxu_x',
        'bxu_y',
        'bxu_yaw',
        'bru_x',
        'bru_y',
        'bru_z',
        'bru_yaw',
        'x_offset',
        'y_offset',
        'yaw_offset_deg',
    )

    for name in optional_scene_arguments:
        if name in context.launch_configurations:
            scene_arguments[name] = (
                LaunchConfiguration(name).perform(context)
            )

    return [
        LogInfo(
            msg=(
                'Front markers relative to BRU: '
                f'xyz=({front_x:.3f}, {front_y:.3f}, {front_z:.3f}), '
                f'yaw={front_yaw_deg:.1f} deg'
            )
        ),
        LogInfo(
            msg=(
                'Docking markers relative to BRU: '
                f'marker 0 xyz=({docking_x:.3f}, '
                f'{docking_y_0:.3f}, {docking_z:.3f}), '
                f'yaw={docking_yaw_0_deg:.1f} deg; '
                f'marker 1 xyz=({docking_x:.3f}, '
                f'{docking_y_1:.3f}, {docking_z:.3f}), '
                f'yaw={docking_yaw_1_deg:.1f} deg'
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
            launch_arguments=scene_arguments.items(),
        ),
    ]


def generate_launch_description():
    defaults = {
        # Preserve the front-marker defaults from the existing launch.
        'front_marker_x': '-0.75',
        'front_marker_y': '0.0',
        'front_marker_z': '0.08',
        'front_marker_yaw_deg': '0.0',

        # Preserve the docking-marker defaults from the existing launch.
        'docking_marker_x': '0.400',
        'docking_marker_y_0': '0.500',
        'docking_marker_y_1': '-0.500',
        'docking_marker_z': '0.0',
        'docking_marker_0_yaw_deg': '180.0',
        'docking_marker_1_yaw_deg': '0.0',
    }

    declarations = [
        DeclareLaunchArgument(
            name,
            default_value=value,
        )
        for name, value in defaults.items()
    ]

    declarations.append(
        DeclareLaunchArgument(
            'preset',
            default_value='manual',
            description=(
                'Preset handled by the existing docking simulation.'
            ),
        )
    )

    return LaunchDescription(
        declarations + [OpaqueFunction(function=setup)]
    )