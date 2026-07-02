from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('swas_system')

    front_vision_params = os.path.join(pkg_share, 'config', 'bxu_front_vision.yaml')
    front_controller_params = os.path.join(pkg_share, 'config', 'bxu_front_approach_controller.yaml')
    motion_params = os.path.join(pkg_share, 'config', 'bxu_motion.yaml')

    front_vision_node = Node(
        package='swas_system',
        executable='bxu_front_vision_node',
        name='bxu_front_vision_node',
        output='screen',
        parameters=[front_vision_params]
    )

    front_controller_node = Node(
        package='swas_system',
        executable='bxu_front_approach_controller_node',
        name='bxu_front_approach_controller_node',
        output='screen',
        parameters=[front_controller_params]
    )

    motion_node = Node(
        package='swas_system',
        executable='bxu_motion_node',
        name='bxu_motion_node',
        output='screen',
        parameters=[motion_params]
    )

    return LaunchDescription([
        front_vision_node,
        front_controller_node,
        motion_node,
    ])