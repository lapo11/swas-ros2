import os
from pathlib import Path
import xml.etree.ElementTree as ET
import math

import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    OpaqueFunction,
    ExecuteProcess,
    LogInfo,
    SetLaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

# Additional BRU pose offsets from its configured world pose.
# Values: world X offset [m], world Y offset [m], yaw offset [degrees].
# No preset changes the BXU spawn pose.
PRESETS = {
    'front': (0.0, 0.0, 0.0),
    'lateral': (0.0, -0.80, 0.0),
    'rotated': (0.0, 0.0, -35.0),
    'combined': (0.0, -0.80, -35.0),
    'away': (-7.0, 0.0, 180.0),
}

def launch_setup(context):

    def number(name):
        value = float(LaunchConfiguration(name).perform(context))
        if not math.isfinite(value):
            raise RuntimeError(f'{name} must be finite.')
        return value

    # BXU pose is independent of the selected preset.
    x0 = number('bxu_x')
    y0 = number('bxu_y')
    z0 = 0.002
    yaw0 = number('bxu_yaw')

    # Start from the configured BRU world pose.
    bru_x = number('bru_x')
    bru_y = number('bru_y')
    bru_z = number('bru_z')
    bru_yaw = number('bru_yaw')

    preset = LaunchConfiguration('preset').perform(context)

    if preset != 'manual':
        if preset not in PRESETS:
            raise RuntimeError(f'Unknown preset: {preset}')

        dx, dy, dyaw_deg = PRESETS[preset]

        bru_x += dx + number('x_offset')
        bru_y += dy + number('y_offset')
        bru_yaw += math.radians(
            dyaw_deg + number('yaw_offset_deg')
        )

        bru_yaw = math.atan2(
            math.sin(bru_yaw),
            math.cos(bru_yaw),
        )



    sim_share = get_package_share_directory('swas_simulation')
    description_share = get_package_share_directory('swas_description')
    gz_share = get_package_share_directory('ros_gz_sim')

    bxu_xacro_path = os.path.join(
        description_share, 'urdf', 'bxu.urdf.xacro'
    )
    bru_xacro_path = os.path.join(
        description_share, 'urdf', 'bru.urdf.xacro'
    )
    world_files = {
        'empty': 'empty_docking_world.sdf',
        'station': 'swas_station_world.sdf',
        'station_w_ceiling': 'swas_station_with_ceiling_world.sdf',
    }

    world_choice = LaunchConfiguration('world').perform(context)

    world_path = os.path.join(
        sim_share, 'worlds', world_files[world_choice]
    )

    if not os.path.isfile(world_path):
        raise RuntimeError(f'World file not found: {world_path}')

    bridge_path = os.path.join(
        sim_share, 'config', 'bridge.yaml'
    )
    camera_config_paths = [os.path.join(sim_share, 'config', filename)
        for filename in (
            'front_camera.gazebo.xml',
            'docking_camera_0.gazebo.xml',
            'docking_camera_1.gazebo.xml',
        )
    ]

    # ------------------------------------------------------------------
    # BXU description
    # ------------------------------------------------------------------
    bxu_xml = xacro.process_file(bxu_xacro_path).toxml()
    bxu_robot = ET.fromstring(bxu_xml)

    # Simulation-only front-camera configuration.
    for camera_config_path in camera_config_paths:
        bxu_robot.append(ET.parse(camera_config_path).getroot())

    # Convert package:// mesh paths to absolute file:// paths for Gazebo.
    package_uri = 'package://swas_description/'
    for mesh in bxu_robot.iter('mesh'):
        filename = mesh.get('filename', '')
        if filename.startswith(package_uri):
            relative_path = filename[len(package_uri):]
            mesh.set(
                'filename',
                Path(description_share, relative_path).as_uri()
            )

    # BXU velocity control.
    bxu_gazebo = ET.SubElement(bxu_robot, 'gazebo')
    velocity_plugin = ET.SubElement(
        bxu_gazebo,
        'plugin',
        filename='gz-sim-velocity-control-system',
        name='gz::sim::systems::VelocityControl',
    )
    ET.SubElement(velocity_plugin, 'topic').text = '/model/bxu/cmd_vel'

    # BXU odometry and odom -> base_link TF.
    odom_plugin = ET.SubElement(
        bxu_gazebo,
        'plugin',
        filename='gz-sim-odometry-publisher-system',
        name='gz::sim::systems::OdometryPublisher',
    )
    ET.SubElement(odom_plugin, 'odom_frame').text = 'odom'
    ET.SubElement(odom_plugin, 'robot_base_frame').text = 'bxu_base_link'
    ET.SubElement(odom_plugin, 'odom_topic').text = '/model/bxu/odometry'
    ET.SubElement(odom_plugin, 'tf_topic').text = '/model/bxu/odometry_tf'
    ET.SubElement(odom_plugin, 'odom_publish_frequency').text = '50'
    ET.SubElement(odom_plugin, 'dimensions').text = '3'

    # Kinematic prototype: disable gravity on every BXU link.
    for link in bxu_robot.findall('link'):
        gazebo_link = ET.SubElement(
            bxu_robot, 'gazebo', reference=link.get('name')
        )
        ET.SubElement(gazebo_link, 'gravity').text = 'false'

    bxu_robot_description = ET.tostring(bxu_robot, encoding='unicode')

    # ------------------------------------------------------------------
    # BRU description
    # ------------------------------------------------------------------
    bru_override = LaunchConfiguration(
            'bru_description_override'
        ).perform(context)

    bru_robot_description = (
        bru_override
        if bru_override.strip()
        else xacro.process_file(bru_xacro_path).toxml()
    )

    bru_robot = ET.fromstring(bru_robot_description)

    # Apply the same mesh-path conversion to the BRU.
    for mesh in bru_robot.iter('mesh'):
        filename = mesh.get('filename', '')
        if filename.startswith(package_uri):
            relative_path = filename[len(package_uri):]
            mesh.set(
                'filename',
                Path(description_share, relative_path).as_uri()
            )

    bru_robot_description = ET.tostring(bru_robot, encoding='unicode')

    # ------------------------------------------------------------------
    # Gazebo resource path
    # ------------------------------------------------------------------
    resource_paths = [
        str(Path(description_share).parent),
        description_share,
    ]
    existing_paths = os.environ.get('GZ_SIM_RESOURCE_PATH')
    if existing_paths:
        resource_paths.append(existing_paths)

    return [

        SetLaunchConfiguration('bxu_x', str(x0)),
        SetLaunchConfiguration('bxu_y', str(y0)),
        SetLaunchConfiguration('bxu_yaw', str(yaw0)),

        SetLaunchConfiguration('bru_x', str(bru_x)),
        SetLaunchConfiguration('bru_y', str(bru_y)),
        SetLaunchConfiguration('bru_z', str(bru_z)),
        SetLaunchConfiguration('bru_yaw', str(bru_yaw)),

        LogInfo(msg=(
            f'BXU: xyz=({x0:.3f}, {y0:.3f}, {z0:.3f}), '
            f'yaw={math.degrees(yaw0):.1f} deg. '
            f'BRU: xyz=({bru_x:.3f}, {bru_y:.3f}, {bru_z:.3f}), '
            f'yaw={math.degrees(bru_yaw):.1f} deg. '
            f'Preset={preset}'
        )),


        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.pathsep.join(resource_paths),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gz_share, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': f'-r -v 3 {world_path}',
            }.items(),
        ),

        # --------------------------------------------------------------
        # BXU
        # --------------------------------------------------------------
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='bxu_state_publisher',
            parameters=[{
                'robot_description': bxu_robot_description,
                'use_sim_time': True,
            }],
            remappings=[
                ('robot_description', '/bxu/robot_description'),
                ('joint_states', '/bxu/joint_states'),
            ],
            output='screen',
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            parameters=[{'config_file': bridge_path}],
            output='screen',
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_bxu',
            arguments=[
                '-world', 'swas_world',
                '-name', 'bxu',
                '-topic', '/bxu/robot_description',
                '-x', LaunchConfiguration('bxu_x'),
                '-y', LaunchConfiguration('bxu_y'),
                '-z', '0.002',
                '-R', '0.0',
                '-P', '0.0',
                '-Y', LaunchConfiguration('bxu_yaw'),
            ],
            output='screen',
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_bxu_odom_tf',
            arguments=[
                '--x', '0.0',
                '--y', '0.0',
                '--z', '0.0',
                '--yaw', '0.0',
                '--pitch', '0.0',
                '--roll', '0.0',
                '--frame-id', 'world',
                '--child-frame-id', 'odom',
            ],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),

        # --------------------------------------------------------------
        # BRU
        # --------------------------------------------------------------
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='bru_state_publisher',
            parameters=[{
                'robot_description': bru_robot_description,
                'use_sim_time': True,
            }],
            remappings=[
                ('robot_description', '/bru/robot_description'),
                ('joint_states', '/bru/joint_states'),
            ],
            output='screen',
        ),

        # Default: publish BRU joint positions without opening a window.
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='bru_joint_state_publisher',
            condition=UnlessCondition(
                LaunchConfiguration('bru_joint_gui')
            ),
            parameters=[{
                'use_sim_time': True,
                'publish_default_positions': True,
                'rate': 30,
            }],
            remappings=[
                ('robot_description', '/bru/robot_description'),
                ('joint_states', '/bru/joint_states'),
            ],
            output='screen',
        ),

        # Optional: open the sliders instead of the non-GUI publisher.
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='bru_joint_state_publisher_gui',
            condition=IfCondition(
                LaunchConfiguration('bru_joint_gui')
            ),
            parameters=[{
                'use_sim_time': True,
                'rate': 30,
            }],
            remappings=[
                ('robot_description', '/bru/robot_description'),
                ('joint_states', '/bru/joint_states'),
            ],
            output='screen',
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_bru',
            arguments=[
                '-world', 'swas_world',
                '-name', 'bru',
                '-topic', '/bru/robot_description',
                '-x', LaunchConfiguration('bru_x'),
                '-y', LaunchConfiguration('bru_y'),
                '-z', LaunchConfiguration('bru_z'),
                '-R', '0.0',
                '-P', '0.0',
                '-Y', LaunchConfiguration('bru_yaw'),
            ],
            output='screen',
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_bru_base_tf',
            arguments=[
                '--x', LaunchConfiguration('bru_x'),
                '--y', LaunchConfiguration('bru_y'),
                '--z', LaunchConfiguration('bru_z'),
                '--yaw', LaunchConfiguration('bru_yaw'),
                '--pitch', '0.0',
                '--roll', '0.0',
                '--frame-id', 'world',
                '--child-frame-id', 'bru_base_link',
            ],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),

        # --------------------------------------------------------------
        # Visualization
        # --------------------------------------------------------------
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=[
                '-d',
                os.path.join(description_share, 'rviz', 'docking_sim.rviz'),
            ],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
    ]

def generate_launch_description():
    defaults = {
        # Manual BXU pose. Used when preset=manual.
        'bxu_x': '0.0',
        'bxu_y': '0.0',
        'bxu_yaw': '0.0',  # radians

        # The single default BRU world pose.
        'bru_x': '3.0',
        'bru_y': '0.0',
        'bru_z': '0.15',
        'bru_yaw': '0.0',  # radians

        # Additional offsets relative to the approach reference.
        'x_offset': '0.0',
        'y_offset': '0.0',
        'yaw_offset_deg': '0.0',

        'bru_description_override': '',
    }

    declarations = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in defaults.items()
    ]

    declarations.append(
        DeclareLaunchArgument(
            'bru_joint_gui',
            default_value='false',
            choices=['true', 'false'],
            description='Open the BRU joint-control GUI.',
        )
    )

    declarations.append(
        DeclareLaunchArgument(
            'world',
            default_value='empty',
            choices=['empty', 'station', 'station_w_ceiling'],
            description=(
                'Simulation environment: empty docking world '
                'or detailed SWAS station '
                'or detailed SWAS station with ceiling (less visibility).'
            ),
        )
    )

    declarations.append(
        DeclareLaunchArgument(
            'preset',
            default_value='manual',
            choices=['manual', *PRESETS],
        )
    )

    return LaunchDescription(
        declarations + [OpaqueFunction(function=launch_setup)]
    )