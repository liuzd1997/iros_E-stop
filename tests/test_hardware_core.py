import copy
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace as Obj
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from hardware_core import JointFeedback,SwitchFeedback,trajectory_arrays,sample_trajectory,load_settings,require_commissioned,associate_switches,JOINTS
from camera_buttons import load_calibration

ROOT=Path(__file__).resolve().parents[1]
CFG=load_settings(ROOT/'config/piper_hardware.yaml')


def trajectory():
    return Obj(joint_names=JOINTS,points=[Obj(positions=[v,1.,-1.,0.,0.,0.],velocities=[],accelerations=[],effort=[],
        time_from_start=Obj(sec=i,nanosec=0)) for i,v in enumerate([0.,.1])])


def joints(stamp):
    return Obj(header=Obj(stamp=Obj(sec=int(stamp),nanosec=round((stamp-int(stamp))*1e9))),
               name=JOINTS+['gripper'],position=[0.,1.,-1.,0.,0.,0.,0.],effort=[0.]*7)


class HardwareCoreTests(unittest.TestCase):
    def test_assumed_calibration_only_loads_for_planning(self):
        path=ROOT/'config/camera_front_assumed.yaml'
        with self.assertRaises(ValueError): load_calibration(path)
        data,matrix=load_calibration(path,allow_assumed=True)
        self.assertFalse(data['calibrated'])
        np.testing.assert_allclose(matrix[:3,:3] @ [0.,0.,1.],[1,0,0])
        np.testing.assert_allclose(matrix[:3,3],[.1,0,.08])
        with self.assertRaises(ValueError): require_commissioned(CFG)

    def test_trajectory_validation_and_sampling(self):
        t,q=trajectory_arrays(trajectory(),CFG['motion'])
        np.testing.assert_allclose(sample_trajectory(t,q,.5),[.05,1,-1,0,0,0])
        for change in ('nan','joint','speed','timing','limits'):
            bad=trajectory()
            if change=='nan': bad.points[1].positions[0]=float('nan')
            if change=='joint': bad.joint_names=JOINTS[::-1]
            if change=='speed': bad.points[1].positions[0]=.5
            if change=='timing': bad.points[1].time_from_start.sec=0
            if change=='limits': bad.points[1].positions[2]=.01
            with self.subTest(change=change),self.assertRaises(ValueError): trajectory_arrays(bad,CFG['motion'])

    def test_duplicates_do_not_refresh_feedback_watchdog(self):
        f=JointFeedback(CFG['motion']); m=joints(100.)
        f.update(m,100.,now=0.)
        f.update(m,100.1,now=.1)
        f.check(now=.2)
        with self.assertRaises(RuntimeError): f.check(now=.3)
        f.update(joints(100.3),100.3,now=.3)
        f.check(now=.3)

    def test_stale_malformed_open_gripper_and_torque_feedback_rejected(self):
        cfg=copy.deepcopy(CFG['motion']); cfg['torque_limits']=[1.]*6
        for case in ('stale','gripper','nan','torque','missing'):
            f=JointFeedback(cfg); m=joints(100.)
            if case=='gripper': m.position[-1]=.02
            if case=='nan': m.position[0]=float('nan')
            if case=='torque': m.effort[0]=2.
            if case=='missing': m.name=[]
            f.update(m,101. if case=='stale' else 100.,now=0.)
            with self.subTest(case=case),self.assertRaises(RuntimeError): f.check(now=0.)

    def test_switch_requires_fresh_baseline_edge_and_heartbeat(self):
        s=SwitchFeedback(.5)
        s.update(dict(channel='a',pressed=True,stamp_sec=100.),100.,now=0.)
        with self.assertRaises(RuntimeError): s.arm('a',now=.01)
        s.update(dict(channel='a',pressed=False,stamp_sec=100.1),100.1,now=.1)
        s.arm('a',now=.11)
        self.assertFalse(s.pressed('a',now=.12))
        s.update(dict(channel='a',pressed=True,stamp_sec=100.2),100.2,now=.2)
        self.assertTrue(s.pressed('a',now=.21))
        with self.assertRaises(RuntimeError): s.pressed('a',now=.8)
        with self.assertRaises(ValueError): s.update(dict(channel='a',pressed=True,stamp_sec=100.2),100.2)
        with self.assertRaises(ValueError): s.update([],100.2)

    def test_switches_associated_by_position_not_detection_id(self):
        targets={'button_002':dict(position=np.array([.5,0,.3])),'button_001':dict(position=np.array([.5,.1,.3]))}
        bindings=[dict(channel='left',position=[.5,.1,.3]),dict(channel='right',position=[.5,0,.3])]
        self.assertEqual(associate_switches(targets,bindings,.02),{'button_002':'right','button_001':'left'})
        bindings[1]['position']=[.5,.1,.3]
        with self.assertRaises(ValueError): associate_switches(targets,bindings,.02)

    def test_gpio_debounce_rejects_short_contact_bounce(self):
        from switch_feedback_gpio import Debouncer
        d=Debouncer(3)
        self.assertIsNone(d.update(False))
        self.assertIsNone(d.update(True))
        self.assertIsNone(d.update(False))
        self.assertIsNone(d.update(False))
        self.assertFalse(d.update(False))
        self.assertFalse(d.update(True))
        self.assertFalse(d.update(False))
        self.assertFalse(d.update(True))
        self.assertFalse(d.update(True))
        self.assertTrue(d.update(True))


if __name__=='__main__': unittest.main()
