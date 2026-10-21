"""Tesla YOLO docking scene plus image-space YOLO approach controller.

The existing docking_tesla_yolo_sim launch starts the Tesla scene and the
front YOLO vision node. This launch adds only the YOLO approach controller.

Start the controller with:
    ros2 service call /bxu/front_yolo_approach/start std_srvs/srv/Trigger

Stop it with:
    ros2 service call /bxu/front_yolo_approach/stop std_srvs/srv/Trigger
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


# These match the arguments and defaults of the existing
# docking_tesla_yolo_sim.launch.py. They are forwarded unchanged.
FORWARDED_DEFAULTS = {
    'world': 'empty',
    'preset': 'manual',
    'bxu_x': '0.0',
    'bxu_y': '0.0',
    'bxu_yaw': '0.0',
    'bru_x': '5.0',
    'bru_y': '0.0',
    'bru_z': '0.15',
    'bru_yaw': '0.0',
    'x_offset': '0.0',
    'y_offset': '0.0',
    'yaw_offset_deg': '0.0',
    'bru_joint_gui': 'false',
    'tesla_x': '0.0',
    'tesla_y': '-0.11',
    'tesla_z': '-0.15',
    'tesla_roll_deg': '0.0',
    'tesla_pitch_deg': '0.0',
    'tesla_yaw_deg': '0.0',
    'bru_description_override': '',
    'tesla_sdf': os.path.join(
        get_package_share_directory('swas_simulation'),
        'config',
        'tesla_visual.sdf',
    ),
    'yolo_model': 'models/first_yolov8_24092026.onnx',
    'yolo_max_frame_age_s': '1.0',
    'yolo_publish_rate_hz': '15.0',
    'yolo_confidence': '0.30',
}


def launch_setup(context):
    sim_share = get_package_share_directory('swas_simulation')
    system_share = get_package_share_directory('swas_system')

    scene_launch_path = os.path.join(
        sim_share,
        'launch',
        'docking_tesla_yolo_sim.launch.py',
    )

    if not os.path.isfile(scene_launch_path):
        raise RuntimeError(
            f'Existing YOLO scene launch not found: {scene_launch_path}'
        )

    scene_arguments = {
        name: LaunchConfiguration(name).perform(context)
        for name in FORWARDED_DEFAULTS
    }

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(scene_launch_path),
            launch_arguments=scene_arguments.items(),
        ),
        Node(
            package='swas_system',
            executable='bxu_front_yolo_approach_controller_node',
            name='bxu_front_yolo_approach_controller_node',
            output='screen',
            parameters=[
                LaunchConfiguration('controller_params'),
                {
                    'use_sim_time': True,
                    'enable_control_services': True,
                },
            ],
        ),
    ]


def generate_launch_description():
    system_share = get_package_share_directory('swas_system')

    declarations = [
        DeclareLaunchArgument(
            'controller_params',
            default_value=os.path.join(
                system_share,
                'config',
                'bxu_front_yolo_approach_controller.yaml',
            ),
            description='YOLO front approach controller parameters.',
        ),
    ]

    declarations.extend(
        DeclareLaunchArgument(name, default_value=default_value)
        for name, default_value in FORWARDED_DEFAULTS.items()
    )

    return LaunchDescription(
        declarations + [OpaqueFunction(function=launch_setup)]
    )