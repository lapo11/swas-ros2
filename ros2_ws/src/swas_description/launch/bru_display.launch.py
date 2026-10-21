import os

import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('swas_description')

    urdf_path = os.path.join(
        pkg_share, 'urdf', 'bru.urdf.xacro'
    )
    rviz_config_path = os.path.join(
        pkg_share, 'rviz', 'bru.rviz'
    )

    robot_description = xacro.process_file(urdf_path).toxml()

    bru_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='bru_state_publisher',
        parameters=[{
            'robot_description': robot_description,
        }],
        remappings=[
            ('robot_description', '/bru/robot_description'),
            ('joint_states', '/bru/joint_states'),
        ],
        output='screen',
    )

    bru_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='bru_joint_state_publisher_gui',
        remappings=[
            ('joint_states', '/bru/joint_states'),
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
        bru_state_publisher,
        bru_joint_state_publisher_gui,
        rviz_node,
    ])