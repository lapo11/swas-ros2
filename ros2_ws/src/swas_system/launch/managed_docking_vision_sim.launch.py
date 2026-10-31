"""Application-only docking vision for an ALREADY RUNNING marker simulation.
No scene, controller, supervisor, gate, lift or motion-start command is launched.
Use the existing manual platform with scene:=markers; do not run a second copy.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    config = LaunchConfiguration('vision_params').perform(context)
    if not os.path.isfile(config):
        raise RuntimeError('Docking vision parameter file missing: '+config)
    return [
        LogInfo(msg='APPLICATION ONLY: docking marker perception. Existing scene/cameras must already be running. No motion controller is loaded.'),
        Node(
            package='swas_system',
            executable='bxu_docking_vision_node',
            name='bxu_docking_vision_node',
            output='screen',
            parameters=[config, {
                'use_sim_time': True,
                'input_source': 'ros',
                'cam0_image_topic': '/bxu/docking_camera_0/image_raw',
                'cam1_image_topic': '/bxu/docking_camera_1/image_raw',
                'cam0_info_topic': '/bxu/docking_camera_0/camera_info',
                'cam1_info_topic': '/bxu/docking_camera_1/camera_info',
                'cam_width': 640,
                'cam_height': 480,
                'marker_size_mm': 9.0,
                'cam0_pos_mm': [400.0, 500.0],
                'cam1_pos_mm': [400.0, -500.0],
                'cam_to_robot_r': [1.0, 0.0, 0.0, 1.0],
                'zero_file': '',
                'publish_rate_hz': 10.0,
                'image_timeout_s': 0.5,
                'publish_overlay_image': True,
                'enable_mjpeg_stream': False,
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'vision_params',
            default_value=os.path.join(
                get_package_share_directory('swas_system'),
                'config',
                'bxu_docking_vision.yaml',
            ),
        ),
        OpaqueFunction(function=setup),
    ])
