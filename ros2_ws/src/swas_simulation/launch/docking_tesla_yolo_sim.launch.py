"""Tesla docking scene plus image-space YOLO only; no controller is started."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# These match the current working docking_tesla_sim.launch.py supplied by Lapo.
SCENE_DEFAULTS = {
    'world': 'empty', 'preset': 'manual',
    'bxu_x': '0.0', 'bxu_y': '0.0', 'bxu_yaw': '0.0',
    'bru_x': '5.0', 'bru_y': '0.0', 'bru_z': '0.15', 'bru_yaw': '0.0',
    'x_offset': '0.0', 'y_offset': '0.0', 'yaw_offset_deg': '0.0',
    'bru_joint_gui': 'false',
    'tesla_x': '0.0', 'tesla_y': '-0.11', 'tesla_z': '-0.15',
    'tesla_roll_deg': '0.0', 'tesla_pitch_deg': '0.0', 'tesla_yaw_deg': '0.0',
    'bru_description_override': '',
}


def setup(context):
    sim = get_package_share_directory('swas_simulation')
    system = get_package_share_directory('swas_system')
    arguments = {name: LaunchConfiguration(name).perform(context)
                 for name in (*SCENE_DEFAULTS, 'tesla_sdf')}
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                sim, 'launch', 'docking_tesla_sim.launch.py')),
            launch_arguments=arguments.items(),
        ),
        Node(
            package='swas_system', executable='bxu_front_yolo_vision_node',
            name='bxu_front_yolo_vision_node', output='screen',
            parameters=[
                os.path.join(system, 'config', 'bxu_front_yolo_vision.yaml'),
                {
                    'use_sim_time': True,
                    'input_source': 'ros',
                    'image_topic': '/bxu/front_camera/image_raw',
                    'camera_info_topic': '/bxu/front_camera/camera_info',
                    'model_path': LaunchConfiguration('yolo_model').perform(context),
                    'max_frame_age_s': float(LaunchConfiguration('yolo_max_frame_age_s').perform(context)),
                    'publish_rate_hz': float(LaunchConfiguration('yolo_publish_rate_hz').perform(context)),
                    'confidence_threshold': float(LaunchConfiguration('yolo_confidence').perform(context)),
                    # THIS IS MAKES IT LESS ROBUST, ASSUMES ONLY 2 WHEELS ARE IN SCENE, AVOIDS ALL THE CAR SIZE/WHEEL PAIR MATCHING
                    # 'wheel_only_mode': True,
                },
            ],
        ),
    ]


def generate_launch_description():
    declarations = [DeclareLaunchArgument(name, default_value=value)
                    for name, value in SCENE_DEFAULTS.items()]
    declarations.extend([
        DeclareLaunchArgument(
            'tesla_sdf', default_value=os.path.join(
                get_package_share_directory('swas_simulation'),
                'config', 'tesla_visual.sdf')),
        DeclareLaunchArgument(
            'yolo_model', default_value='models/first_yolov8_24092026.onnx'),
        DeclareLaunchArgument('yolo_max_frame_age_s', default_value='1.0'),
        DeclareLaunchArgument('yolo_publish_rate_hz', default_value='15.0'),
        DeclareLaunchArgument('yolo_confidence', default_value='0.30'),
    ])
    return LaunchDescription(declarations + [OpaqueFunction(function=setup)])