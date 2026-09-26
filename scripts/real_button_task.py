#!/usr/bin/env python3
"""Detect and plan with measured Piper feedback; optional commissioned hardware execution."""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from moveit_msgs.msg import DisplayTrajectory, RobotState
import yaml

from button_scene import call, plan
from camera_buttons import CameraButtons, load_calibration
from hardware_core import JOINTS, JointFeedback, SwitchFeedback, associate_switches, load_settings, require_commissioned, trajectory_arrays
from multi_button_demo import RunReport, scene, update_scene, object_id, target_pose
from press_button import cartesian, return_plan, tip_position, check_joints
from touch_button import apply_matrix, contact_matrix, execute, final_state

ROOT=Path(__file__).resolve().parents[1]


def slow(trajectory,config):
    result=copy.deepcopy(trajectory)
    points=result.joint_trajectory.points
    times=np.array([p.time_from_start.sec+p.time_from_start.nanosec*1e-9 for p in points])
    if len(times)<2 or np.any(np.diff(times)<=0):
        raise ValueError('Planner returned invalid trajectory times')
    positions=np.array([p.positions for p in points])
    peak=float(np.max(np.abs(np.diff(positions,axis=0)/np.diff(times)[:,None])))
    for p in points:
        if p.velocities: peak=max(peak,max(abs(v) for v in p.velocities))
    # Half the adapter limit leaves margin for timing and physical tracking.
    factor=max(2.,peak/(config['max_joint_speed']*.5))
    for p,t in zip(points,times):
        nanoseconds=round(t*factor*1e9)
        p.time_from_start=Duration(sec=nanoseconds//10**9,nanosec=nanoseconds%10**9)
        p.velocities=[v/factor for v in p.velocities]
        p.accelerations=[a/factor**2 for a in p.accelerations]
    trajectory_arrays(result.joint_trajectory,config)
    return result


class Observer:
    def __init__(self,node,cfg):
        self.node=node; self.cfg=cfg
        self.joints=JointFeedback(cfg['motion'])
        self.switches=SwitchFeedback(cfg['press']['feedback_timeout'])
        self.switch_error=''
        self.joint_sub=node.create_subscription(JointState,'/piper/feedback/joint_states',self.joint_callback,1)
        self.switch_sub=node.create_subscription(String,cfg['press']['feedback_topic'],self.switch_callback,10)

    def joint_callback(self,msg):
        self.joints.update(msg,self.node.get_clock().now().nanoseconds/1e9)

    def switch_callback(self,msg):
        try:
            self.switches.update(json.loads(msg.data),self.node.get_clock().now().nanoseconds/1e9)
            self.switch_error=''
        except (ValueError,TypeError,KeyError) as exc:
            self.switch_error=str(exc)

    def state(self):
        self.joints.check()
        return RobotState(joint_state=copy.deepcopy(self.joints.message),is_diff=False)

    def wait(self,timeout=10):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            rclpy.spin_once(self.node,timeout_sec=.02)
            try: return self.state()
            except RuntimeError: pass
        raise RuntimeError('No fresh, valid physical Piper joints with closed gripper')

    def switch(self,channel):
        if self.switch_error:
            raise RuntimeError(self.switch_error)
        return self.switches.pressed(channel)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--hardware-config',default=str(ROOT/'config/piper_hardware.yaml'))
    parser.add_argument('--expected-count',type=int,required=True)
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--report')
    args=parser.parse_args()
    if args.expected_count<1: parser.error('Expected count must be positive')
    cfg=load_settings(args.hardware_config)
    if os.environ.get('ROS_DOMAIN_ID')!=str(cfg.get('ros_domain_id',79)):
        parser.error('ROS domain must match the hardware configuration; use run_real_button_task.sh')
    if args.execute: require_commissioned(cfg)
    calibration,_=load_calibration(cfg['calibration'],allow_assumed=not args.execute)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('real_button_task')
    args.source='camera'
    report=RunReport(node,args)
    report.data.update(mode='execute_hardware' if args.execute else 'hardware_plan_only',
                       calibration_assumed=calibration.get('calibrated') is not True,
                       hardware_validated=False,emulated_test=cfg.get('ros_domain_id')==80)
    report.data['note']='Hardware has not been independently validated; emulated tests are explicitly marked.'
    observer=Observer(node,cfg)
    original=None; active=None; started_motion=False
    def readiness():
        result=call(node,Trigger,'/piper_execution/readiness',Trigger.Request())
        if not result.success: raise RuntimeError(result.message)
        details=json.loads(result.message)
        if details.get('backend')!='agilex_measured_feedback': raise RuntimeError('Wrong execution backend')
        if args.execute and not details.get('execution_enabled'): raise RuntimeError('Launch the commissioned adapter with enable_execution:=true')
    def run(trajectory):
        nonlocal started_motion
        readiness()
        check_joints(observer.wait(),RobotState(joint_state=JointState(name=trajectory.joint_trajectory.joint_names,
            position=trajectory.joint_trajectory.points[0].positions)),cfg['motion']['start_tolerance'])
        started_motion=True
        p=trajectory.joint_trajectory.points[-1]
        timeout=p.time_from_start.sec+p.time_from_start.nanosec*1e-9+15
        execute(node,trajectory,timeout=max(45,timeout))
        observer.wait()
    try:
        deadline=time.monotonic()+10
        while True:
            try:
                readiness()
                break
            except RuntimeError:
                if time.monotonic()>=deadline: raise
                rclpy.spin_once(node,timeout_sec=.05)
        initial=observer.wait()
        report.data['initial_joints']={name:float(value) for name,value in zip(initial.joint_state.name,initial.joint_state.position) if name in JOINTS}
        report.event('Saved physical joint pose; acquiring camera targets')
        camera=CameraButtons(node,yaml.safe_load(Path(cfg['camera_config']).read_text()),calibration=cfg['calibration'],allow_assumed=not args.execute)
        inventory=camera.wait(args.expected_count)
        ordered=sorted(inventory,key=lambda key:(-inventory[key]['position'][1],inventory[key]['position'][2]))
        report.data['order']=ordered
        channels=associate_switches(inventory,cfg['press']['bindings'],cfg['press']['association_tolerance']) if args.execute else {}
        for key in ordered:
            target=inventory[key]
            report.data['buttons'][key]=dict(position=target['position'].tolist(),direction=target['direction'].tolist(),
                status='pending',verification=None,switch_channel=channels.get(key))
        if args.execute:
            # All channels must have a live, unpressed baseline before any movement.
            for channel in channels.values(): observer.switches.arm(channel)
        snapshot=scene(node)
        original=copy.deepcopy(snapshot.allowed_collision_matrix)
        update_scene(node,inventory,snapshot)
        itinerary={}; start=initial
        stroke=cfg['press']; tip=cfg['tip_offset_z']
        depths=np.linspace(0,stroke['max_travel'],math.ceil(stroke['max_travel']/stroke['step'])+1)
        for key in ordered:
            target=inventory[key]
            apply_matrix(node,contact_matrix(original,object_id(key,'cap')))
            approach=target_pose(target,-tip-stroke['approach_distance'])
            travel=slow(plan(node,approach,link_name='link6',position_tolerance=.0001,
                       orientation_tolerance=.001,start_state=start).trajectory[0],cfg['motion'])
            at_approach=final_state(start,travel)
            current=at_approach
            pushes=[]; retracts=[]
            for depth in depths:
                segment=slow(cartesian(node,current,target_pose(target,-tip+float(depth))),cfg['motion'])
                current=final_state(current,segment)
                error=np.linalg.norm(np.asarray(tip_position(node,current,tip))-(target['position']+depth*target['direction']))
                if error>.001: raise RuntimeError('Planned fingertip pose fails FK check')
                pushes.append(segment)
                retracts.append(slow(cartesian(node,current,approach),cfg['motion']))
            itinerary[key]=(travel,pushes,retracts)
            start=final_state(current,retracts[-1])
            report.event('Planned approach, bounded press steps, and every retract branch',key)
        apply_matrix(node,original)
        returning=slow(return_plan(node,start,initial),cfg['motion'])
        report.event('Complete itinerary and return planned')
        if not args.execute:
            pub=node.create_publisher(DisplayTrajectory,'/display_planned_path',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            trajectory=[]
            for key in ordered:
                travel,pushes,retracts=itinerary[key]
                trajectory.extend([travel,*pushes,retracts[-1]])
            trajectory.append(returning)
            pub.publish(DisplayTrajectory(model_id='agx_arm',trajectory_start=initial,trajectory=trajectory))
            report.data['result']='planned_only'
            for value in report.data['buttons'].values(): value['status']='planned'
            report.event('Plan only; assumed calibration is not physical execution calibration')
            print('Enable Planned Path visuals to inspect. Ctrl+C closes.',flush=True)
            try: rclpy.spin(node)
            except KeyboardInterrupt: pass
            return
        check_joints(observer.wait(),initial,cfg['motion']['start_tolerance'])
        for key in ordered:
            active=key; target=inventory[key]; channel=channels[key]
            camera.revalidate(key,target,inventory)
            observer.switches.arm(channel)
            apply_matrix(node,contact_matrix(original,object_id(key,'cap')))
            travel,pushes,retracts=itinerary[key]
            report.data['buttons'][key]['status']='in_progress'
            report.event('Approach',key); run(travel)
            camera.revalidate(key,target,inventory)
            # A switch activation during approach is not the requested controlled press.
            if observer.switch(channel): raise RuntimeError('Switch changed before the press started')
            pressed_index=None
            for index,segment in enumerate(pushes):
                if observer.switch(channel):
                    pressed_index=index-1
                    break
                report.event('Touch' if index==0 else f'Press step {depths[index]*1000:.1f} mm',key)
                run(segment)
                if observer.switch(channel):
                    pressed_index=index
                    break
            if pressed_index is None: raise RuntimeError(f'{key}: no switch activation within maximum travel')
            error=np.linalg.norm(np.asarray(tip_position(node,observer.state(),tip))-
                                 (target['position']+depths[pressed_index]*target['direction']))
            if error>stroke['tip_tolerance']: raise RuntimeError('Measured fingertip differs from expected pressed pose')
            report.event('Hold with continuous switch and robot feedback',key)
            began=time.monotonic()
            held=observer.state()
            while time.monotonic()-began<stroke['hold_seconds']:
                rclpy.spin_once(node,timeout_sec=.02)
                check_joints(observer.state(),held,cfg['motion']['goal_tolerance'])
                if not observer.switch(channel): raise RuntimeError('Switch released during hold')
            readiness()
            report.data['buttons'][key]['verification']=dict(method='physical_switch_transition_and_joint_feedback',
                channel=channel,hold_seconds=time.monotonic()-began,tip_error_m=float(error),travel_m=float(depths[pressed_index]))
            report.event('Verified; retract',key); run(retracts[pressed_index])
            # Physical E-stops may latch: do not command a fictitious cap reset.
            apply_matrix(node,original)
            report.data['buttons'][key]['status']='completed'
            report.event('Completed',key); active=None
        report.event('Return to saved physical joint pose')
        run(returning)
        error=check_joints(observer.wait(),initial,cfg['motion']['goal_tolerance'])
        report.data.update(result='completed',initial_pose_restored=True,return_max_joint_error_rad=error)
        report.event('All physical switch transitions verified; initial pose restored')
    except (Exception,KeyboardInterrupt) as exc:
        if started_motion:
            try: call(node,Trigger,'/piper_execution/stop',Trigger.Request())
            except Exception as stop_error: report.data['stop_error']=str(stop_error)
        if active: report.data['buttons'][active]['status']='failed'
        report.data.update(result='aborted',error=str(exc) or 'Interrupted')
        report.event('Aborted; no automatic recovery movement')
        raise
    finally:
        if original is not None and rclpy.ok():
            try: apply_matrix(node,original)
            except Exception as exc: report.data['cleanup_error']=str(exc)
        report.save(); print(f'Run report: {report.path}',flush=True)
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__=='__main__':
    try: main()
    except KeyboardInterrupt: pass
    except Exception as exc:
        print(f'ERROR: {exc}',flush=True)
        raise SystemExit(1)
