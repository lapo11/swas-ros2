from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('swas_system')

    front_vision_params = os.path.join(pkg_share, 'config', 'bxu_front_vision.yaml')
    front_controller_params = os.path.join(pkg_share, 'config', 'bxu_front_approach_controller.yaml')
    docking_vision_params = os.path.join(pkg_share, 'config', 'bxu_docking_vision.yaml')
    motion_params = os.path.join(pkg_share, 'config', 'bxu_motion.yaml')

    return LaunchDescription([
        SetEnvironmentVariable(
            'GST_PLUGIN_PATH',
            '/usr/local/lib/aarch64-linux-gnu/gstreamer-1.0'
        ),
        SetEnvironmentVariable(
            'LD_LIBRARY_PATH',
            '/opt/ros/jazzy/lib:/usr/local/lib/aarch64-linux-gnu:/home/bxu-swas-dev/swas-ros2/ros2_ws/install/swas_msgs/lib:/home/bxu-swas-dev/swas-ros2/ros2_ws/install/swas_system/lib'
        ),

        Node(
            package='swas_system',
            executable='bxu_front_vision_node',
            name='bxu_front_vision_node',
            output='screen',
            parameters=[front_vision_params]
        ),
        Node(
            package='swas_system',
            executable='bxu_docking_vision_node',
            name='bxu_docking_vision_node',
            output='screen',
            parameters=[docking_vision_params]
        ),
        Node(
            package='swas_system',
            executable='bxu_front_approach_controller_node',
            name='bxu_front_approach_controller_node',
            output='screen',
            parameters=[front_controller_params]
        ),
        Node(
            package='swas_system',
            executable='bxu_supervisor_node',
            name='bxu_supervisor_node',
            output='screen',
            parameters=[
                {'auto_start': True}
            ]
        ),
        Node(
            package='swas_system',
            executable='bxu_motion_node',
            name='bxu_motion_node',
            output='screen',
            parameters=[motion_params]
        ),
    ])