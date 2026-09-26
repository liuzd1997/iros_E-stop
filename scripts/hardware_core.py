"""Validation shared by hardware execution and hardware-free tests."""
from pathlib import Path
import math
import os
import time
import numpy as np
import yaml

JOINTS = [f'joint{i}' for i in range(1, 7)]
# Standard Piper URDF, not Piper H/L/X/Nero.
LOWER = np.array([-2.6179938, 0., -2.9670597, -1.7453292, -1.2217304, -2.0943951])
UPPER = np.array([2.6179938, 3.1415926, 0., 1.7453292, 1.2217304, 2.0943951])


def load_settings(path):
    path = Path(path).resolve()
    cfg = yaml.safe_load(path.read_text())
    for field in ('camera_config', 'calibration'):
        cfg[field] = str((path.parent/ cfg[field]).resolve())
    for section, names in [('motion', ('max_joint_speed','command_rate','feedback_timeout','start_tolerance','path_tolerance','goal_tolerance','settle_timeout','max_duration')),
                           ('press', ('approach_distance','step','max_travel','hold_seconds','tip_tolerance','feedback_timeout','association_tolerance'))]:
        for name in names:
            value = cfg[section][name]
            if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'{section}.{name} must be finite and positive')
    if not .01 <= cfg['tip_offset_z'] <= .3:
        raise ValueError('tip_offset_z must be 0.01–0.3 m')
    if not 0 < cfg['press']['step'] <= cfg['press']['max_travel'] <= .008:
        raise ValueError('Press step/travel exceeds the supported 8 mm collision model')
    if not 1 <= cfg['speed_percent'] <= 10 or cfg['motion']['max_joint_speed'] > .5:
        raise ValueError('This adapter supports driver speed 1–10% and joint speed <=0.5 rad/s')
    if not 10 <= cfg['motion']['command_rate'] <= 100:
        raise ValueError('command_rate must be 10–100 Hz')
    limits=cfg['motion'].get('torque_limits')
    if limits is not None and (np.asarray(limits).shape!=(6,) or not np.isfinite(limits).all() or min(limits)<=0):
        raise ValueError('torque_limits must contain six positive finite Nm limits')
    return cfg


def require_commissioned(cfg):
    required = ('robot_and_scene_checked','tool_geometry_checked','compliant_contact_tool_checked',
                'button_travel_checked','external_estop_available','targets_do_not_stop_piper')
    missing=[key for key in required if cfg.get('commissioning',{}).get(key) is not True]
    if missing:
        raise ValueError('Hardware setup is not commissioned: '+', '.join(missing))
    from camera_buttons import load_calibration
    data,_=load_calibration(cfg['calibration'])  # Assumed transforms are never execution calibration.
    if data.get('test_fixture_only') and (cfg.get('ros_domain_id',79)!=80 or os.environ.get('ROS_DOMAIN_ID')!='80'):
        raise ValueError('Emulated calibration is restricted to test domain 80')


def trajectory_arrays(trajectory, cfg):
    if list(trajectory.joint_names) != JOINTS:
        raise ValueError('Trajectory must name all six Piper joints in order')
    if len(trajectory.points)<2:
        raise ValueError('Trajectory needs at least two points')
    times=np.array([p.time_from_start.sec+p.time_from_start.nanosec*1e-9 for p in trajectory.points])
    q=np.array([list(p.positions) for p in trajectory.points],float)
    if q.shape!=(len(times),6) or not np.isfinite(q).all() or not np.isfinite(times).all():
        raise ValueError('Malformed/nonfinite trajectory')
    if times[0]<0 or times[0]>.1 or np.any(np.diff(times)<=0) or times[-1]>cfg['max_duration']:
        raise ValueError('Invalid trajectory timing or duration')
    if np.any(q < LOWER-1e-6) or np.any(q > UPPER+1e-6):
        raise ValueError('Trajectory exceeds standard Piper joint limits')
    speed=np.abs(np.diff(q,axis=0)/np.diff(times)[:,None])
    if np.any(speed>cfg['max_joint_speed']+1e-6):
        raise ValueError('Trajectory exceeds configured joint speed; retime before execution')
    for point in trajectory.points:
        for field in ('velocities','accelerations','effort'):
            values=getattr(point,field)
            if values and (len(values)!=6 or not np.isfinite(values).all()):
                raise ValueError('Invalid trajectory '+field)
        if point.velocities and max(abs(v) for v in point.velocities)>cfg['max_joint_speed']+1e-6:
            raise ValueError('Trajectory velocity exceeds configured limit')
    return times,q


def sample_trajectory(times,q,elapsed):
    return np.array([np.interp(elapsed,times,q[:,i]) for i in range(6)])


class JointFeedback:
    def __init__(self, cfg):
        self.cfg=cfg
        self.q=None
        self.message=None
        self.received=-math.inf
        self.stamp=-math.inf
        self.error='No joint feedback'

    def update(self,message,ros_now,now=None):
        now=time.monotonic() if now is None else now
        stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        try:
            if len(message.name)!=len(message.position) or len(set(message.name))!=len(message.name):
                raise ValueError('Malformed joint feedback')
            values=dict(zip(message.name,message.position))
            q=np.array([values[j] for j in JOINTS])
            if not np.isfinite(q).all() or abs(values.get('gripper',math.inf))>.001:
                raise ValueError('Invalid joints or gripper is not closed')
            if stamp<=self.stamp or not -.1 <= ros_now-stamp <=self.cfg['feedback_timeout']:
                # Ignore queued/repeated/out-of-order samples without refreshing
                # the watchdog. A stream containing only these still times out.
                return
            if np.any(q<LOWER-.01) or np.any(q>UPPER+.01):
                raise ValueError('Feedback outside Piper joint limits')
            torque=self.cfg.get('torque_limits')
            if torque is not None:
                if len(message.effort)!=len(message.name):
                    raise ValueError('Required torque feedback is missing')
                efforts=dict(zip(message.name,message.effort))
                t=np.array([efforts[j] for j in JOINTS])
                if not np.isfinite(t).all() or np.any(np.abs(t)>torque):
                    raise ValueError('Joint torque limit exceeded')
            self.q=q; self.message=message; self.received=now; self.stamp=stamp; self.error=''
        except (KeyError,ValueError) as exc:
            self.error=str(exc)

    def check(self,now=None):
        now=time.monotonic() if now is None else now
        if self.error or self.q is None or now-self.received>self.cfg['feedback_timeout']:
            raise RuntimeError(self.error or 'Joint feedback stopped')
        return self.q.copy()


class SwitchFeedback:
    """Timestamped heartbeat plus a fresh false→true transition per physical channel."""
    def __init__(self,timeout):
        self.timeout=timeout
        self.states={}
        self.armed={}

    def update(self,data,ros_now,now=None):
        now=time.monotonic() if now is None else now
        if not isinstance(data,dict):
            raise ValueError('Switch message must be a JSON object')
        channel=data.get('channel')
        stamp=data.get('stamp_sec')
        if not isinstance(channel,str) or not channel or type(data.get('pressed')) is not bool:
            raise ValueError('Switch message needs channel, boolean pressed, and stamp_sec')
        if not isinstance(stamp,(float,int)) or not math.isfinite(stamp) or not -.1<=ros_now-stamp<=self.timeout:
            raise ValueError('Invalid/stale switch timestamp')
        previous=self.states.get(channel)
        if previous and stamp<=previous['stamp']:
            raise ValueError('Switch timestamps must increase')
        edge = now if previous and not previous['pressed'] and data['pressed'] else (previous or {}).get('edge',-math.inf)
        self.states[channel]=dict(pressed=data['pressed'],stamp=stamp,received=now,edge=edge)

    def state(self,channel,now=None):
        now=time.monotonic() if now is None else now
        state=self.states.get(channel)
        if state is None or now-state['received']>self.timeout:
            raise RuntimeError(f'No fresh switch feedback for {channel}')
        return state

    def arm(self,channel,now=None):
        now=time.monotonic() if now is None else now
        if self.state(channel,now)['pressed']:
            raise RuntimeError(f'{channel} is already pressed; no new press can be verified')
        self.armed[channel]=now

    def pressed(self,channel,now=None):
        state=self.state(channel,now)
        return state['pressed'] and state['edge']>self.armed.get(channel,math.inf)


def associate_switches(inventory,bindings,tolerance):
    if len(bindings)!=len(inventory):
        raise ValueError('Provide one measured physical switch binding per detected button')
    assigned={}; used=set()
    for key,target in inventory.items():
        matches=[b for b in bindings if np.linalg.norm(np.asarray(b['position'],float)-target['position'])<=tolerance]
        if len(matches)!=1 or matches[0]['channel'] in used:
            raise ValueError(f'{key}: ambiguous/missing physical switch association')
        assigned[key]=matches[0]['channel']; used.add(matches[0]['channel'])
    return assigned
