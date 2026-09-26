#!/usr/bin/env python3
"""Launch RViz + mock Piper, wait for controllers, and run one virtual E-stop cycle."""
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.signals import SignalHandlerOptions
from controller_manager_msgs.srv import ListControllers

ROOT = Path(__file__).resolve().parents[1]


def stop(process):
    if process is None or process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=12)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=8)


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '77':
        raise RuntimeError('Use run_virtual_estop_demo.sh')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('virtual_estop_launcher')
    client = node.create_client(ListControllers, '/controller_manager/list_controllers')
    launch = task = None
    log = None
    try:
        if not client.wait_for_service(timeout_sec=2):
            (ROOT/'outputs').mkdir(exist_ok=True)
            log = (ROOT/'outputs'/'virtual_estop_launch.log').open('w')
            print('Opening RViz and starting mock Piper. Startup log: outputs/virtual_estop_launch.log', flush=True)
            launch = subprocess.Popen(['bash', str(ROOT/'scripts/run_moveit_demo.sh')],
                                      stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        else:
            print('Using the existing demo controller manager; task will verify mock hardware.', flush=True)
        deadline = time.monotonic()+60
        ready = False
        while time.monotonic() < deadline:
            if launch and launch.poll() is not None:
                raise RuntimeError('MoveIt launch exited; inspect outputs/virtual_estop_launch.log')
            if not client.wait_for_service(timeout_sec=1):
                continue
            future = client.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=2)
            if not future.done():
                future.cancel()
                continue
            active = {c.name for c in future.result().controller if c.state == 'active'}
            if {'arm_controller', 'gripper_controller', 'joint_state_broadcaster'} <= active:
                ready = True
                break
            rclpy.spin_once(node, timeout_sec=.2)
        if not ready:
            raise RuntimeError('Mock controllers did not become ready within 60 seconds')
        print('Controllers ready. Running detection and one press/hold/return cycle.', flush=True)
        task = subprocess.Popen(['bash', str(ROOT/'scripts/run_multi_button_demo.sh'),
            '--source', 'synthetic', '--expected-count', '3', '--execute', '--hold', '3', '--keep-open'],
            start_new_session=True)
        code = task.wait()
        if code:
            raise RuntimeError(f'Button task exited with code {code}')
    except KeyboardInterrupt:
        pass
    finally:
        stop(task)
        stop(launch)
        if log:
            log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print(f'ERROR: {exc}', flush=True)
        raise SystemExit(1)
