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


## NOTE:
# To test the docking on its own, launch with bru and bxu slightly disaligned, for example:
# ros2 launch swas_simulation docking_docking_markers_dock_sim.launch.py \
#   preset:=manual \
#   bxu_x:=3.02 \
#   bxu_y:=0.01 \
#   bxu_yaw:=0.05 \
#   bru_x:=3.0 \
#   bru_y:=0.0 \
#   bru_yaw:=0.0
#
# and then, start in another sourced terminal: 
# ros2 service call /bxu/docking_controller/start std_srvs/srv/Trigger {}\
# 
# to see estimated alignment error and state, run:
# ros2 topic echo /bxu/docking_vision/alignment
#
# and to see the alignment error, run:
# ros2 run tf2_ros tf2_echo bru_base_link bxu_base_link --ros-args -p use_sim_time:=true
#


def launch_setup(context):
    sim_share = get_package_share_directory('swas_simulation')
    system_share = get_package_share_directory('swas_system')

    markers_launch_name = LaunchConfiguration(
        'markers_launch'
    ).perform(context)

    markers_launch_path = os.path.join(
        sim_share,
        'launch',
        markers_launch_name,
    )

    if not os.path.isfile(markers_launch_path):
        raise RuntimeError(
            f'Marker launch not found: {markers_launch_path}. '
            'Set markers_launch to your existing docking-markers launch filename.'
        )

    scene_arguments = {
        'preset': LaunchConfiguration('preset').perform(context),
    }

    optional_scene_arguments = (
        'bxu_x',
        'bxu_y',
        'bxu_yaw',
        'bru_x',
        'bru_y',
        'bru_z',
        'bru_yaw',

        'marker_x',
        'marker_y_0',
        'marker_y_1',
        'marker_z',
        'marker_0_yaw_deg',
        'marker_1_yaw_deg',

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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(markers_launch_path),
            launch_arguments=scene_arguments.items(),
        ),
        Node(
            package='swas_system',
            executable='bxu_docking_controller_node',
            name='bxu_docking_controller_node',
            output='screen',
            parameters=[
                LaunchConfiguration('controller_params'),
                {
                    'use_sim_time': True,
                },
            ],
        ),
    ]


def generate_launch_description():
    system_share = get_package_share_directory('swas_system')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'markers_launch',
                default_value='docking_docking_markers_sim.launch.py',
                description=(
                    'Existing docking scene launch with attached '
                    'docking markers.'
                ),
            ),
            DeclareLaunchArgument(
                'preset',
                default_value='manual',
                description='Preset handled by the existing docking launch.',
            ),
            Node(
                package='swas_system',
                executable='bxu_docking_vision_node',
                name='bxu_docking_vision_node',
                output='screen',
                parameters=[
                    os.path.join(
                        system_share,
                        'config',
                        'bxu_docking_vision.yaml',
                    ),
                    {
                        'use_sim_time': True,
                        'input_source': 'ros',

                        'cam0_image_topic':
                            '/bxu/docking_camera_0/image_raw',
                        'cam1_image_topic':
                            '/bxu/docking_camera_1/image_raw',

                        'cam0_info_topic':
                            '/bxu/docking_camera_0/camera_info',
                        'cam1_info_topic':
                            '/bxu/docking_camera_1/camera_info',

                        'cam_width': 640,
                        'cam_height': 480,

                        # Use optical centres as the simulated docking targets.
                        # Do not load the hardware alignment-zero file.
                        'zero_file': '',

                        # Match the current simulated camera geometry.
                        'cam0_pos_mm': [400.0, 500.0],
                        'cam1_pos_mm': [400.0, -500.0],

                        'cam_to_robot_r': [1.0, 0.0, 0.0, 1.0],

                        'publish_rate_hz': 10.0,
                        'image_timeout_s': 0.5,

                        'publish_overlay_image': True,
                        'enable_mjpeg_stream': False,
                    },
                ],
            ),
            DeclareLaunchArgument(
                'controller_params',
                default_value=os.path.join(
                    system_share,
                    'config',
                    'bxu_docking_controller.yaml',
                ),
            ),
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
                default_value='0.0',
            ),
            DeclareLaunchArgument(
                'marker_0_yaw_deg',
                default_value='180.0',
            ),
            DeclareLaunchArgument(
                'marker_1_yaw_deg',
                default_value='0.0',
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )