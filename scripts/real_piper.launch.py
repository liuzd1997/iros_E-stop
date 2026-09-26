"""Measured-feedback MoveIt launch; no mock controllers or joint-command relays."""
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from hardware_core import load_settings, require_commissioned


def build(context):
    path=LaunchConfiguration('hardware_config').perform(context)
    cfg=load_settings(path)
    if os.environ.get('ROS_DOMAIN_ID')!=str(cfg.get('ros_domain_id',79)):
        raise RuntimeError('ROS domain must match the hardware configuration')
    enabled=LaunchConfiguration('enable_execution').perform(context)=='true'
    if enabled:
        require_commissioned(cfg)
    package=Path(get_package_share_directory('agx_arm_moveit'))
    sys.path.insert(0,str(package/'launch'))
    from _moveit_config_builder import build_moveit_config
    moveit=build_moveit_config(context)
    # This launch uses a measured-feedback action server, not ros2_control.
    robot=ET.fromstring(moveit.robot_description['robot_description'])
    for item in robot.findall('ros2_control'): robot.remove(item)
    moveit.robot_description['robot_description']=ET.tostring(robot,encoding='unicode')
    moveit.sensors_3d={}
    controllers=moveit.trajectory_execution['moveit_simple_controller_manager']
    controllers['controller_names']=['arm_controller']
    controllers.pop('gripper_controller',None)
    params=moveit.to_dict()
    params.update({'publish_robot_description_semantic': True,'publish_planning_scene': True,
                   'publish_geometry_updates': True,'publish_state_updates': True,
                   'publish_transforms_updates': True,'trajectory_execution.allowed_execution_duration_scaling':3.,
                   'trajectory_execution.allowed_goal_duration_margin':8.})
    feedback='/piper/feedback/joint_states'
    processes=[
        Node(package='agx_arm_ctrl',executable='agx_arm_ctrl_single',namespace='piper',
             name='driver',output='screen',parameters=[dict(can_port=cfg['can_port'],arm_type='piper',
             effector_type='agx_gripper',auto_enable=False,control_enabled=False,fast_mode=False,
             speed_percent=cfg['speed_percent'],pub_rate=100)]),
        Node(package='robot_state_publisher',executable='robot_state_publisher',
             parameters=[moveit.robot_description],remappings=[('joint_states',feedback)]),
        Node(package='moveit_ros_move_group',executable='move_group',parameters=[params],
             remappings=[('joint_states',feedback)],output='screen'),
        ExecuteProcess(cmd=['/usr/bin/python3',str(ROOT/'scripts/piper_trajectory_controller.py'),
                            '--config',path]+(['--enable-execution'] if enabled else []),output='screen'),
    ]
    if LaunchConfiguration('use_rviz').perform(context)=='true':
        processes.append(Node(package='rviz2',executable='rviz2',arguments=['-d',str(package/'config/moveit.rviz')],
            parameters=[moveit.robot_description,moveit.robot_description_semantic,
                        moveit.robot_description_kinematics,moveit.planning_pipelines,moveit.joint_limits],
            remappings=[('joint_states',feedback)],output='screen'))
    return processes


def generate_launch_description():
    # Parameters needed by the existing config builder are fixed to standard Piper.
    return LaunchDescription([
        DeclareLaunchArgument('hardware_config',default_value=str(ROOT/'config/piper_hardware.yaml')),
        DeclareLaunchArgument('enable_execution',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('use_rviz',default_value='true',choices=['true','false']),
        DeclareLaunchArgument('arm_type',default_value='piper',choices=['piper']),
        DeclareLaunchArgument('effector_type',default_value='agx_gripper',choices=['agx_gripper']),
        DeclareLaunchArgument('revo2_type',default_value='left',choices=['left']),
        DeclareLaunchArgument('tcp_offset',default_value='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]'),
        OpaqueFunction(function=build),
    ])
