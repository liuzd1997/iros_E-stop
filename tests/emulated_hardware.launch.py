"""Only domain 80; replace the CAN driver with a ROS test fixture."""
import importlib.util
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction,ExecuteProcess
from launch.substitutions import LaunchConfiguration
ROOT=Path(__file__).resolve().parents[1]


def build(context):
    if os.environ.get('ROS_DOMAIN_ID')!='80': raise RuntimeError('Emulated hardware requires domain 80')
    spec=importlib.util.spec_from_file_location('real_launch',ROOT/'scripts/real_piper.launch.py')
    module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    actions=module.build(context)
    # Remove the CAN driver action before the launch service sees it.
    actions.pop(0)
    cfg=LaunchConfiguration('hardware_config').perform(context)
    actions.insert(0,ExecuteProcess(cmd=['/usr/bin/python3',str(ROOT/'tests/emulated_hardware.py'),'--config',cfg],output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hardware_config'),
        DeclareLaunchArgument('enable_execution',default_value='true'),
        DeclareLaunchArgument('use_rviz',default_value='false'),
        DeclareLaunchArgument('arm_type',default_value='piper'),
        DeclareLaunchArgument('effector_type',default_value='agx_gripper'),
        DeclareLaunchArgument('revo2_type',default_value='left'),
        DeclareLaunchArgument('tcp_offset',default_value='[0.0,0.0,0.0,0.0,0.0,0.0]'),
        OpaqueFunction(function=build),
    ])
