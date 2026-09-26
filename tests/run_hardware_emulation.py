"""Run the real execution stack against ideal ROS fixtures, never a CAN driver."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import yaml
import rclpy
from std_srvs.srv import Trigger

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from hardware_core import load_settings
from button_perception import synthetic_rgbd


def stop(process):
    if process is None: return
    try: process.send_signal(signal.SIGINT)
    except ProcessLookupError: return
    try: process.wait(timeout=15)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid,signal.SIGTERM)
        try: process.wait(timeout=5)
        except subprocess.TimeoutExpired: os.killpg(process.pid,signal.SIGKILL); process.wait()


def main():
    if os.environ.get('ROS_DOMAIN_ID')!='80': raise RuntimeError('Requires isolated test domain 80')
    rclpy.init(); node=rclpy.create_node('hardware_emulation_test_runner')
    client=node.create_client(Trigger,'/piper_execution/readiness')
    if client.wait_for_service(timeout_sec=2.):
        node.destroy_node(); rclpy.shutdown()
        raise RuntimeError('Domain 80 already has a Piper adapter; stop that test first')
    launch=None; task=None
    output=ROOT/'outputs'; output.mkdir(exist_ok=True)
    report=output/'hardware_emulated_execution.json'
    try:
        with tempfile.TemporaryDirectory(prefix='piper_emulation_') as directory:
            cfg=load_settings(ROOT/'config/piper_hardware.yaml')
            cfg['ros_domain_id']=80
            cfg['commissioning']={k:True for k in cfg['commissioning']}
            _,_,_,_,transform=synthetic_rgbd()
            calibration=dict(calibrated=True,assumed=False,test_fixture_only=True,
                parent_frame='base_link',camera_frame='camera_color_optical_frame',camera_to_base=transform.tolist())
            path=Path(directory)
            (path/'camera.yaml').write_text(yaml.safe_dump(calibration))
            cfg['calibration']=str(path/'camera.yaml')
            cfg['press']['bindings']=[dict(channel=f'switch_{i}',position=[.5,y,.3]) for i,y in enumerate((.12,0.,-.12))]
            (path/'hardware.yaml').write_text(yaml.safe_dump(cfg))
            with (output/'hardware_integration_launch.log').open('w') as log:
                launch=subprocess.Popen(['ros2','launch',str(ROOT/'tests/emulated_hardware.launch.py'),
                    'hardware_config:='+str(path/'hardware.yaml')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                end=time.monotonic()+40
                ready=False
                while time.monotonic()<end:
                    if launch.poll() is not None: raise RuntimeError('Emulated launch failed; inspect launch log')
                    if not client.wait_for_service(timeout_sec=.5): continue
                    future=client.call_async(Trigger.Request())
                    rclpy.spin_until_future_complete(node,future,timeout_sec=1.)
                    if future.done() and future.result().success:
                        ready=True; break
                if not ready: raise RuntimeError('Emulated adapter did not become ready')
                task=subprocess.Popen([sys.executable,str(ROOT/'scripts/real_button_task.py'),
                    '--hardware-config',str(path/'hardware.yaml'),'--expected-count','3','--execute','--report',str(report)],
                    start_new_session=True)
                if task.wait(timeout=300)!=0: raise RuntimeError('Button integration failed; inspect report and launch log')
                data=json.loads(report.read_text())
                assert data['emulated_test'] is True and data['result']=='completed',data
                assert len(data['buttons'])==3
                for target in data['buttons'].values():
                    assert target['status']=='completed',target
                    assert target['verification']['hold_seconds']>=3.,target
                print('PASS: three detected buttons, verified switches, >=3 s holds, retracts and return; no CAN.')
    finally:
        stop(task); stop(launch)
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__=='__main__': main()
