from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('swas_system')

    front_controller_params = os.path.join(
        pkg_share,
        'config',
        'bxu_front_approach_controller.yaml'
    )

    docking_controller_params = os.path.join(
        pkg_share,
        'config',
        'bxu_docking_controller.yaml'
    )

    swas_msgs_lib = os.path.join(
        os.path.expanduser('~'),
        'swas-ros2',
        'ros2_ws',
        'install',
        'swas_msgs',
        'lib'
    )

    swas_system_lib = os.path.join(
        os.path.expanduser('~'),
        'swas-ros2',
        'ros2_ws',
        'install',
        'swas_system',
        'lib'
    )

    current_ld_library_path = os.environ.get('LD_LIBRARY_PATH', '')

    return LaunchDescription([
        SetEnvironmentVariable(
            name='LD_LIBRARY_PATH',
            value=':'.join([
                swas_msgs_lib,
                swas_system_lib,
                current_ld_library_path,
            ])
        ),

        Node(
            package='swas_system',
            executable='bxu_front_approach_controller_node',
            name='bxu_front_approach_controller_node',
            output='screen',
            parameters=[front_controller_params],
        ),

        Node(
            package='swas_system',
            executable='bxu_docking_controller_node',
            name='bxu_docking_controller_node',
            output='screen',
            parameters=[docking_controller_params],
        ),

        Node(
            package='swas_system',
            executable='bxu_supervisor_node',
            name='bxu_supervisor_node',
            output='screen',
            parameters=[{
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
                'cmd_vel_topic':
                    '/bxu/cmd_vel',
                'lift_command_topic':
                    '/bxu_battery_lift/command',
                'lift_state_topic':
                    '/bxu_battery_lift/state',
                'auto_start': False,
                'drive_away_speed': -0.15,
                'drive_away_duration': 15.0,
            }],
        ),
    ])