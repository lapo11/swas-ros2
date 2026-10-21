import math
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


## NOTE:
#
# To make the demo start, launch the file, then, in another sourced terminal, run:
# ros2 service call /bxu/supervisor/start std_srvs/srv/Trigger 
#
# To confirm (ex: when it asks to proceed with lifting), run
# ros2 service call /bxu/supervisor/confirm std_srvs/srv/Trigger 
#
# To stop demo run:
# ros2 service call /bxu/supervisor/stop std_srvs/srv/Trigger 

def launch_setup(context):
    sim_share = get_package_share_directory('swas_simulation')
    system_share = get_package_share_directory('swas_system')

    def value(name):
        return LaunchConfiguration(name).perform(context)

    def number(name):
        result = float(value(name))

        if not math.isfinite(result):
            raise RuntimeError(f'{name} must be finite.')

        return result

    def config_path(argument):
        path = value(argument)

        if not os.path.isfile(path):
            raise RuntimeError(f'Parameter file not found: {path}')

        return path

    launch_supervisor = value('launch_supervisor') == 'true'

    scene_arguments = {
        'preset': value('preset'),
    }

    # The combined marker scene owns marker defaults.
    # docking_sim.launch.py owns robot-pose and preset defaults.
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

        'front_marker_x',
        'front_marker_y',
        'front_marker_z',
        'front_marker_yaw_deg',

        'docking_marker_x',
        'docking_marker_y_0',
        'docking_marker_y_1',
        'docking_marker_z',
        'docking_marker_0_yaw_deg',
        'docking_marker_1_yaw_deg',
    )

    for name in optional_scene_arguments:
        if name in context.launch_configurations:
            scene_arguments[name] = value(name)

    scene = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                sim_share,
                'launch',
                'docking_bru_markers_sim.launch.py',
            )
        ),
        launch_arguments=scene_arguments.items(),
    )

    front_vision = Node(
        package='swas_system',
        executable='bxu_front_vision_node',
        name='bxu_front_vision_node',
        output='screen',
        parameters=[
            config_path('front_vision_params'),
            {
                'use_sim_time': True,
                'input_source': 'ros',
                'image_topic': '/bxu/front_camera/image_raw',
                'camera_info_topic': '/bxu/front_camera/camera_info',
                'publish_overlay_image': True,
            },
        ],
    )

    docking_vision = Node(
        package='swas_system',
        executable='bxu_docking_vision_node',
        name='bxu_docking_vision_node',
        output='screen',
        parameters=[
            config_path('docking_vision_params'),
            {
                'use_sim_time': True,
                'input_source': 'ros',

                'cam0_image_topic':
                    '/bxu/docking_camera_0/image_raw',
                'cam1_image_topic':
                    '/bxu/docking_camera_1/image_raw',

                # These match the parameter names in your current node.
                'cam0_info_topic':
                    '/bxu/docking_camera_0/camera_info',
                'cam1_info_topic':
                    '/bxu/docking_camera_1/camera_info',

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
            },
        ],
    )

    front_controller = Node(
        remappings=[('/bxu/cmd_vel', '/swas/auto/cmd_vel')],
        package='swas_system',
        executable='bxu_front_approach_controller_node',
        name='bxu_front_approach_controller_node',
        output='screen',
        parameters=[
            config_path('front_controller_params'),
            {
                'use_sim_time': True,
                'enable_control_services': False,
            },
        ],
    )

    docking_controller = Node(
        remappings=[('/bxu/cmd_vel', '/swas/auto/cmd_vel')],
        package='swas_system',
        executable='bxu_docking_controller_node',
        name='bxu_docking_controller_node',
        output='screen',
        parameters=[
            config_path('docking_controller_params'),
            {
                'use_sim_time': True,
                'enable_control_services': False,
            },
        ],
    )

    lift = Node(
        package='swas_simulation',
        executable='bxu_battery_lift_sim_node.py',
        name='bxu_battery_lift_sim_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'command_topic': '/bxu_battery_lift/command',
            'state_topic': '/bxu_battery_lift/state',
        }],
    )
    
    runtime_nodes = [
        front_vision,
        docking_vision,
        front_controller,
        docking_controller,
        lift,
    ]

    if launch_supervisor:
        supervisor = Node(
            remappings=[('/bxu/cmd_vel', '/swas/auto/cmd_vel')],
            package='swas_system',
            executable='bxu_supervisor_node',
            name='bxu_supervisor_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,

                'front_status_topic':
                    '/bxu/front_approach/controller_status',
                'dock_status_topic':
                    '/bxu/docking/controller_status',
                'supervisor_state_topic':
                    '/bxu/supervisor/state',

                'front_enable_topic':
                    '/bxu/front_controller/enabled',
                'dock_enable_topic':
                    '/bxu/docking_controller/enabled',

                'cmd_vel_topic': '/bxu/cmd_vel',
                'lift_command_topic': '/bxu_battery_lift/command',
                'lift_state_topic': '/bxu_battery_lift/state',

                'auto_start': False,
                'drive_away_speed': number('drive_away_speed'),
                'drive_away_duration': number('drive_away_duration'),
            }],
        )

        runtime_nodes.append(supervisor)

    # End the demo if one of its application processes exits.
    # This avoids intentionally leaving the rest of the demo running
    # after, for example, a vision-node failure.
    exit_handlers = [
        RegisterEventHandler(
            OnProcessExit(
                target_action=node,
                on_exit=[
                    EmitEvent(
                        event=Shutdown(
                            reason='A demo application process exited.'
                        )
                    )
                ],
            )
        )
        for node in runtime_nodes
    ]

    return [
        LogInfo(
            msg=(
                'Starting combined marker scene, both vision nodes, '
                'both controllers and simulated battery lift. '
                'The supervisor owns controller activation.'
            )
        ),
        *exit_handlers,
        scene,
        *runtime_nodes,

        TimerAction(
            period=20.0,
            actions=[
                LogInfo(
                    msg=(
                        '\n'
                        '============================================================\n'
                        '                 SIMULATION DEMO COMMANDS\n'
                        '============================================================\n'
                        '\n'
                        'Run these commands in another sourced terminal.\n'
                        '\n'
                        'START DEMO:\n'
                        '  ros2 service call /bxu/supervisor/start '
                        'std_srvs/srv/Trigger\n'
                        '\n'
                        'CONFIRM / CONTINUE:\n'
                        '  When the supervisor reports WAIT_USER_AFTER_LIFT\n'
                        '  or WAIT_USER_AFTER_LOWER, run:\n'
                        '  ros2 service call /bxu/supervisor/confirm '
                        'std_srvs/srv/Trigger\n'
                        '\n'
                        'STOP DEMO:\n'
                        '  ros2 service call /bxu/supervisor/stop '
                        'std_srvs/srv/Trigger\n'
                        '\n'
                        'MONITOR SUPERVISOR:\n'
                        '  ros2 topic echo /bxu/supervisor/state\n'
                        '\n'
                        'This banner is informational, not a readiness check.\n'
                        '============================================================\n'
                    )
                ),
            ],
        ),
    ]


def generate_launch_description():
    system_share = get_package_share_directory('swas_system')

    parameter_files = {
        'front_vision_params': 'bxu_front_vision.yaml',
        'front_controller_params': 'bxu_front_approach_controller.yaml',
        'docking_vision_params': 'bxu_docking_vision.yaml',
        'docking_controller_params': 'bxu_docking_controller.yaml',
    }

    declarations = [
        DeclareLaunchArgument(
            name,
            default_value=os.path.join(
                system_share,
                'config',
                filename,
            ),
        )
        for name, filename in parameter_files.items()
    ]

    declarations.extend([
        DeclareLaunchArgument(
            'preset',
            default_value='manual',
        ),
        DeclareLaunchArgument(
            'launch_supervisor',
            default_value='true',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'drive_away_speed',
            default_value='-0.15',
        ),
        DeclareLaunchArgument(
            'drive_away_duration',
            default_value='15.0',
        ),
    ])

    return LaunchDescription(
        declarations + [OpaqueFunction(function=launch_setup)]
    )