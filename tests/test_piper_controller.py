"""ROS contract tests with an emulated driver; never opens CAN or moves a robot."""
import copy
import os
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from agx_arm_msgs.msg import AgxArmStatus
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool,Empty
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from hardware_core import load_settings,JOINTS
from piper_trajectory_controller import PiperController
ROOT=Path(__file__).resolve().parents[1]


class FakeDriver(Node):
    def __init__(self):
        super().__init__('emulated_piper_driver')
        self.q=[0.,1.,-1.,0.,0.,0.]
        self.enabled=False; self.gated=False; self.holds=0; self.commands=0
        self.frozen=False; self.fault=False
        self.pub=self.create_publisher(JointState,'/piper/feedback/joint_states',1)
        self.status=self.create_publisher(AgxArmStatus,'/piper/feedback/arm_status',1)
        self.create_subscription(JointState,'/piper/control/joint_states',self.command,1)
        self.create_service(SetBool,'/piper/enable_agx_arm',self.enable)
        self.create_service(SetBool,'/piper/control_enable',self.gate)
        self.create_service(Empty,'/piper/emergency_stop',self.halt)
        self.create_timer(.01,self.publish)

    def enable(self,req,res): self.enabled=req.data; res.success=True; return res
    def gate(self,req,res): self.gated=req.data; res.success=True; return res
    def halt(self,req,res): self.holds+=1; return res
    def command(self,msg):
        if self.enabled and self.gated:
            self.commands+=1; self.q=list(msg.position)
    def publish(self):
        if not self.frozen:
            msg=JointState(name=JOINTS+['gripper'],position=self.q+[0.],effort=[0.]*7)
            msg.header.stamp=self.get_clock().now().to_msg(); self.pub.publish(msg)
        self.status.publish(AgxArmStatus(ctrl_mode=1,arm_status=1 if self.fault else 0,motion_status=0))


class ControllerContractTests(unittest.TestCase):
    def setUp(self):
        if os.environ.get('ROS_DOMAIN_ID')!='80':
            self.skipTest('Requires isolated test domain 80; no physical driver may run there')
        self.tmp=tempfile.TemporaryDirectory()
        cfg=load_settings(ROOT/'config/piper_hardware.yaml')
        cfg['ros_domain_id']=80
        for key in cfg['commissioning']: cfg['commissioning'][key]=True
        data=yaml.safe_load((ROOT/'config/camera_front_assumed.yaml').read_text())
        data.update(calibrated=True,assumed=False,test_fixture_only=True)
        path=Path(self.tmp.name)/'calibration.yaml'; path.write_text(yaml.safe_dump(data))
        cfg['calibration']=str(path)
        rclpy.init()
        self.driver=FakeDriver(); self.controller=PiperController(cfg,True)
        self.client_node=Node('test_trajectory_client')
        self.client=ActionClient(self.client_node,FollowJointTrajectory,'/arm_controller/follow_joint_trajectory')
        self.executor=SingleThreadedExecutor()
        for n in (self.driver,self.controller,self.client_node): self.executor.add_node(n)
        self.thread=threading.Thread(target=self.executor.spin,daemon=True); self.thread.start()
        self.assertTrue(self.client.wait_for_server(timeout_sec=5.))
        end=time.monotonic()+5
        while time.monotonic()<end:
            try: self.controller.healthy(); break
            except RuntimeError: time.sleep(.02)
        self.controller.healthy()

    def tearDown(self):
        if not hasattr(self,'executor'): return
        self.executor.shutdown(timeout_sec=3)
        self.thread.join(timeout=3)
        for n in (self.driver,self.controller,self.client_node): n.destroy_node()
        rclpy.shutdown(); self.tmp.cleanup()

    def wait(self,future,timeout=6):
        end=time.monotonic()+timeout
        while not future.done() and time.monotonic()<end: time.sleep(.01)
        self.assertTrue(future.done(),'Action timed out')
        return future.result()

    def goal(self):
        req=FollowJointTrajectory.Goal()
        req.trajectory.joint_names=JOINTS
        q=self.driver.q.copy(); end=q.copy(); end[0]+=.05
        req.trajectory.points=[JointTrajectoryPoint(positions=q,time_from_start=Duration()),
                               JointTrajectoryPoint(positions=end,time_from_start=Duration(sec=1))]
        return self.wait(self.client.send_goal_async(req))

    def test_success_uses_feedback_and_closes_gate(self):
        handle=self.goal(); self.assertTrue(handle.accepted)
        result=self.wait(handle.get_result_async()).result
        self.assertEqual(result.error_code,0)
        self.assertGreater(self.driver.commands,5)
        self.assertAlmostEqual(self.driver.q[0],.05,places=3)
        self.assertFalse(self.driver.gated)

    def test_feedback_loss_aborts_and_latches_stop(self):
        handle=self.goal(); self.assertTrue(handle.accepted)
        time.sleep(.15); self.driver.frozen=True
        result=self.wait(handle.get_result_async()).result
        self.assertNotEqual(result.error_code,0)
        self.assertTrue(self.controller.fault)
        time.sleep(.1); self.assertGreater(self.driver.holds,0); self.assertFalse(self.driver.gated)
        self.assertFalse(self.goal().accepted)

    def test_arm_fault_aborts(self):
        handle=self.goal(); self.assertTrue(handle.accepted)
        time.sleep(.15); self.driver.fault=True
        self.assertNotEqual(self.wait(handle.get_result_async()).result.error_code,0)
        self.assertTrue(self.controller.fault)

    def test_fresh_feedback_with_wrong_position_aborts(self):
        self.controller.motion['path_tolerance']=.01
        handle=self.goal(); self.assertTrue(handle.accepted)
        time.sleep(.15)
        self.driver.enabled=False  # Feedback stays fresh, but the drive ignores commands.
        result=self.wait(handle.get_result_async()).result
        self.assertNotEqual(result.error_code,0)
        self.assertIn('tracking error',self.controller.fault)
        time.sleep(.1)
        self.assertFalse(self.driver.gated)

    def test_cancel_requests_hold_and_closes_gate(self):
        handle=self.goal(); self.assertTrue(handle.accepted)
        time.sleep(.15); self.wait(handle.cancel_goal_async())
        result=self.wait(handle.get_result_async())
        self.assertEqual(result.status,5)
        time.sleep(.1); self.assertGreater(self.driver.holds,0); self.assertFalse(self.driver.gated)

    def test_uncommissioned_or_disabled_rejects_without_commands(self):
        self.controller.enable_execution=False
        self.assertFalse(self.goal().accepted)
        self.controller.enable_execution=True
        self.controller.config['commissioning']['tool_geometry_checked']=False
        self.assertFalse(self.goal().accepted)
        self.assertEqual(self.driver.commands,0)


if __name__=='__main__': unittest.main()
