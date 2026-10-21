import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def generate_launch_description():
    pkg_share = get_package_share_directory('swas_description')

    urdf_path = os.path.join(
        pkg_share, 'urdf', 'bxu.urdf.xacro'
    )
    rviz_config_path = os.path.join(
        pkg_share, 'rviz', 'bxu.rviz'
    )

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )

    bxu_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='bxu_state_publisher',
        parameters=[{
            'robot_description': robot_description,
        }],
        remappings=[
            ('robot_description', '/bxu/robot_description'),
            ('joint_states', '/bxu/joint_states'),
        ],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen',
    )

    return LaunchDescription([
        bxu_state_publisher,
        rviz_node,
    ])