import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('swas_system')

    docking_vision_params = os.path.join(pkg_share, 'config', 'bxu_docking_vision.yaml')
    docking_controller_params = os.path.join(pkg_share, 'config', 'bxu_docking_controller.yaml')
    motion_params = os.path.join(pkg_share, 'config', 'bxu_motion.yaml')

    return LaunchDescription([
        Node(
            package='swas_system',
            executable='bxu_docking_vision_node',
            name='bxu_docking_vision_node',
            output='screen',
            parameters=[docking_vision_params]
        ),
        Node(
            package='swas_system',
            executable='bxu_docking_controller_node',
            name='bxu_docking_controller_node',
            output='screen',
            parameters=[docking_controller_params]
        ),
        Node(
            package='swas_system',
            executable='bxu_motion_node',
            name='bxu_motion_node',
            output='screen',
            parameters=[motion_params]
        ),
    ])