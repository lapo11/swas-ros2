# SWAS_OPTIONAL_COMPONENT_MANAGER_V1
"""Opt-in manual-only base. Reuses the already tested inspection scene composition."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

DEFAULTS = {
    'with_component_manager':'false',
    'scene':'markers','world':'empty','preset':'manual','with_hmi':'true',
    'bxu_x':'0.0','bxu_y':'0.0','bxu_yaw':'0.0',
    'bru_x':'3.0','bru_y':'0.0','bru_z':'0.15','bru_yaw':'0.0',
    'x_offset':'0.0','y_offset':'0.0','yaw_offset_deg':'0.0','bru_joint_gui':'false',
    'serve_hmi':'true','hmi_port':'5173','hmi_bind_address':'127.0.0.1',
    'launch_rosbridge':'true','rosbridge_port':'9090','launch_video_server':'true','video_port':'8080',
}

def setup(context):
    share = get_package_share_directory('swas_bringup')
    inspection = os.path.join(share,'launch','simulation_inspection.launch.py')
    if not os.path.isfile(inspection):
        raise RuntimeError('First milestone inspection launch is missing: '+inspection)
    gate = Node(package='swas_system',executable='bxu_platform_gate',name='bxu_command_gate',output='screen',
                parameters=[{'use_sim_time':False,'expected_sink_node':'ros_gz_bridge'}])
    # Reuse the uploaded no-motion, immediate-ack simulator, on NEW canonical topics.
    # No lift command service/button is added; any external command publisher blocks manual driving.
    lift = Node(package='swas_simulation',executable='bxu_battery_lift_sim_node.py',name='bxu_platform_lift_sim',output='screen',
                parameters=[{'use_sim_time':True,'command_topic':'/bxu/lift/command','state_topic':'/bxu/lift/state'}])
    component_manager = Node(
        package='swas_system',
        executable='bxu_component_manager',
        name='bxu_component_manager',
        output='screen',
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(LaunchConfiguration('with_component_manager')),
    )
    # Manager is optional; keep the existing gate/lift fail-together policy unchanged.
    handlers = [RegisterEventHandler(OnProcessExit(target_action=node,on_exit=[EmitEvent(event=Shutdown(reason='A persistent manual-platform process exited.'))])) for node in (gate,lift)]
    return [LogInfo(msg='NEW MANUAL-ONLY SIMULATION PLATFORM: no supervisor/controllers/perception apps or AUTO. Stop old demos/gates first. Legacy launch files are unchanged.'),
            *handlers,gate,lift, component_manager,
            GroupAction(scoped=True,actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(inspection),
                launch_arguments={name:LaunchConfiguration(name).perform(context) for name in DEFAULTS if name != 'with_component_manager'}.items())])]

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(k,default_value=v) for k,v in DEFAULTS.items()]+[OpaqueFunction(function=setup)])
