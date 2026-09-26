#!/usr/bin/env python3
"""MoveIt FollowJointTrajectory adapter using AgileX commands and measured feedback.

No ros2_control mock controller is involved. Position streaming is not force
control or a safety-rated stop. Faults latch until this adapter is restarted.
"""
import argparse
import json
import threading
import time

import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.task import Future
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, SetBool, Trigger
from agx_arm_msgs.msg import AgxArmStatus
from trajectory_msgs.msg import JointTrajectoryPoint

from hardware_core import JOINTS, JointFeedback, load_settings, require_commissioned, trajectory_arrays, sample_trajectory


class PiperController(Node):
    def __init__(self, config, enable_execution=False):
        if enable_execution:
            require_commissioned(config)
        super().__init__('piper_trajectory_controller')
        self.config=config
        self.motion=config['motion']
        self.enable_execution=enable_execution
        self.feedback=JointFeedback(self.motion)
        self.status=None
        self.status_time=0.
        self.fault=''
        self.engaged=False
        self.busy=False
        self.lock=threading.RLock()
        self.group=ReentrantCallbackGroup()
        self.feedback_group=MutuallyExclusiveCallbackGroup()
        self.command=self.create_publisher(JointState,'/piper/control/joint_states',1)
        self.create_subscription(JointState,'/piper/feedback/joint_states',self.on_joints,1,callback_group=self.feedback_group)
        self.create_subscription(AgxArmStatus,'/piper/feedback/arm_status',self.on_status,1,callback_group=self.feedback_group)
        self.enable=self.create_client(SetBool,'/piper/enable_agx_arm',callback_group=self.group)
        self.gate=self.create_client(SetBool,'/piper/control_enable',callback_group=self.group)
        self.halt=self.create_client(Empty,'/piper/emergency_stop',callback_group=self.group)
        self.create_service(Trigger,'/piper_execution/readiness',self.readiness,callback_group=self.group)
        self.create_service(Trigger,'/piper_execution/stop',self.stop_request,callback_group=self.group)
        self.action=ActionServer(self,FollowJointTrajectory,'/arm_controller/follow_joint_trajectory',
            execute_callback=self.execute,goal_callback=self.goal,cancel_callback=lambda _:CancelResponse.ACCEPT,
            callback_group=self.group)
        self.create_timer(.05,self.watchdog,callback_group=self.feedback_group)

    def on_joints(self,msg):
        with self.lock:
            self.feedback.update(msg,self.get_clock().now().nanoseconds/1e9)

    def on_status(self,msg):
        with self.lock:
            self.status=msg
            self.status_time=time.monotonic()

    def healthy(self):
        with self.lock:
            if self.fault:
                raise RuntimeError(self.fault)
            if self.count_publishers('/piper/feedback/joint_states') != 1 or self.count_publishers('/piper/feedback/arm_status') != 1:
                raise RuntimeError('Expected exactly one physical joint/status feedback publisher')
            q=self.feedback.check()
            if self.status is None or time.monotonic()-self.status_time>self.motion['feedback_timeout']:
                raise RuntimeError('Arm status feedback stopped')
            s=self.status
            if s.arm_status!=0 or s.err_status!=0 or any(s.joint_angle_limit) or any(s.communication_status_joint):
                raise RuntimeError(f'Arm fault/status {s.arm_status}, error {s.err_status}')
            return q

    def readiness(self,request,response):
        try:
            q=self.healthy()
            response.success=True
            response.message=json.dumps(dict(backend='agilex_measured_feedback',joints=q.tolist(),
                execution_enabled=self.enable_execution,busy=self.busy,fault=self.fault))
        except Exception as exc:
            response.success=False; response.message=str(exc)
        return response

    async def pause(self, seconds):
        future=Future()
        def wake():
            if not future.done(): future.set_result(None)
        timer=self.create_timer(seconds,wake,callback_group=self.group)
        try: await future
        finally: self.destroy_timer(timer)

    async def rpc(self,client,request,timeout=7.):
        if not client.service_is_ready():
            raise RuntimeError(f'Driver service unavailable: {client.srv_name}')
        future=client.call_async(request)
        deadline=time.monotonic()+timeout
        while not future.done():
            if time.monotonic()>deadline:
                future.cancel()
                raise RuntimeError(f'Driver service timeout: {client.srv_name}')
            await self.pause(.01)
        result=future.result()
        if hasattr(result,'success') and not result.success:
            raise RuntimeError(result.message)
        return result

    def trip(self,reason):
        with self.lock:
            if self.fault:
                return
            self.fault=reason
        self.get_logger().error('Motion stopped: '+reason)
        # The vendor service holds the measured joint pose; it is NOT a wired E-stop.
        if self.halt.service_is_ready():
            self.halt.call_async(Empty.Request())
        if self.gate.service_is_ready():
            self.gate.call_async(SetBool.Request(data=False))

    def stop_request(self,request,response):
        self.trip('Stop requested by task/operator')
        response.success=True
        response.message='Fault latched; software hold/gate-close requests sent. Restart to re-arm.'
        return response

    def watchdog(self):
        if self.engaged and not self.fault:
            try:
                self.healthy()
            except Exception as exc:
                self.trip(str(exc))

    def goal(self,request):
        with self.lock:
            try:
                if self.busy or not self.enable_execution:
                    raise RuntimeError('Execution disabled or another trajectory is active')
                require_commissioned(self.config)
                times,q=trajectory_arrays(request.trajectory,self.motion)
                stamp=request.trajectory.header.stamp
                scheduled=stamp.sec+stamp.nanosec*1e-9
                if scheduled and abs(self.get_clock().now().nanoseconds/1e9-scheduled)>.25:
                    raise ValueError('Scheduled/stale trajectories are unsupported')
                if request.multi_dof_trajectory.points:
                    raise ValueError('Multi-DOF trajectories are unsupported')
                for tolerance in list(request.path_tolerance)+list(request.goal_tolerance):
                    if tolerance.name not in JOINTS or tolerance.velocity!=0 or tolerance.acceleration!=0 or tolerance.position<0:
                        raise ValueError('Only nonnegative joint-position tolerances are supported')
                if np.max(np.abs(self.healthy()-q[0]))>self.motion['start_tolerance']:
                    raise ValueError('Trajectory start does not match physical feedback')
                self.busy=True
                return GoalResponse.ACCEPT
            except Exception as exc:
                self.get_logger().warn('Rejected trajectory: '+str(exc))
                return GoalResponse.REJECT

    async def execute(self,handle):
        result=FollowJointTrajectory.Result()
        succeeded=False
        try:
            times,q=trajectory_arrays(handle.request.trajectory,self.motion)
            goal_tol=np.full(6,self.motion['goal_tolerance'])
            path_tol=np.full(6,self.motion['path_tolerance'])
            for values,target in ((handle.request.path_tolerance,path_tol),(handle.request.goal_tolerance,goal_tol)):
                for tolerance in values:
                    if tolerance.position>0:
                        target[JOINTS.index(tolerance.name)]=min(target[JOINTS.index(tolerance.name)],tolerance.position)
            self.healthy()
            await self.rpc(self.enable,SetBool.Request(data=True))
            self.engaged=True
            self.healthy()
            await self.rpc(self.gate,SetBool.Request(data=True))
            started=time.monotonic()
            settled=0
            last_sample=-1.
            period=1/self.motion['command_rate']
            requested_slack=handle.request.goal_time_tolerance.sec+handle.request.goal_time_tolerance.nanosec*1e-9
            slack=min(self.motion['settle_timeout'],requested_slack) if requested_slack>0 else self.motion['settle_timeout']
            while rclpy.ok():
                if handle.is_cancel_requested:
                    self.trip('Trajectory canceled')
                    handle.canceled()
                    result.error_code=result.PATH_TOLERANCE_VIOLATED
                    result.error_string='Canceled; software hold requested'
                    return result
                elapsed=time.monotonic()-started
                measured=self.healthy()
                desired=sample_trajectory(times,q,elapsed)
                error=desired-measured
                if np.any(np.abs(error)>path_tol):
                    raise RuntimeError(f'Physical tracking error exceeds path tolerance at {elapsed:.3f}s: '
                                       f'desired={desired.round(5).tolist()}, measured={measured.round(5).tolist()}, '
                                       f'tolerance={path_tol.tolist()}')
                cmd=JointState()
                cmd.header.stamp=self.get_clock().now().to_msg()
                cmd.name=JOINTS; cmd.position=desired.tolist()
                self.command.publish(cmd)
                feedback=FollowJointTrajectory.Feedback()
                feedback.header=cmd.header; feedback.joint_names=JOINTS
                feedback.desired=JointTrajectoryPoint(positions=desired.tolist())
                feedback.actual=JointTrajectoryPoint(positions=measured.tolist())
                feedback.error=JointTrajectoryPoint(positions=error.tolist())
                handle.publish_feedback(feedback)
                if elapsed>=times[-1]:
                    # Require distinct measured samples, not repeated reads of one sample.
                    if self.feedback.stamp!=last_sample:
                        settled=settled+1 if np.all(np.abs(q[-1]-measured)<=goal_tol) and self.status.motion_status==0 else 0
                        last_sample=self.feedback.stamp
                    if settled>=3:
                        await self.rpc(self.gate,SetBool.Request(data=False))
                        self.healthy()
                        handle.succeed(); result.error_code=result.SUCCESSFUL; succeeded=True
                        return result
                    if elapsed>times[-1]+slack:
                        raise RuntimeError('Physical arm did not reach the goal before timeout')
                await self.pause(period)
            raise RuntimeError('ROS shutdown during trajectory')
        except Exception as exc:
            self.trip(str(exc))
            if handle.is_active:
                handle.abort()
            result.error_code=result.PATH_TOLERANCE_VIOLATED
            result.error_string=str(exc)
            return result
        finally:
            if not succeeded and self.gate.service_is_ready():
                self.gate.call_async(SetBool.Request(data=False))
            with self.lock:
                self.busy=False


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',required=True)
    parser.add_argument('--enable-execution',action='store_true')
    args=parser.parse_args()
    cfg=load_settings(args.config)
    if args.enable_execution:
        require_commissioned(cfg)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=PiperController(cfg,args.enable_execution)
    executor=SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        if node.engaged:
            node.trip('Adapter shutdown')
            # Give asynchronous hold/gate requests a chance to be sent.
            for _ in range(5): executor.spin_once(timeout_sec=.05)
    finally:
        executor.shutdown(timeout_sec=3)
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__=='__main__':
    main()
