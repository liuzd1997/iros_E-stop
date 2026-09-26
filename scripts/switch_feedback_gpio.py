#!/usr/bin/env python3
"""Publish timestamped auxiliary-contact states using Ubuntu libgpiod 1.x.

Read-only GPIO inputs. Does not bypass/reset an E-stop or control its circuit.
"""
import argparse
import json
import math
from pathlib import Path
import rclpy
from std_msgs.msg import String
import yaml


class Debouncer:
    def __init__(self,samples):
        if type(samples) is not int or samples<1: raise ValueError('debounce_samples must be a positive integer')
        self.samples=samples; self.last=None; self.count=0; self.value=None

    def update(self,value):
        self.count=self.count+1 if value==self.last else 1
        self.last=value
        if self.count>=self.samples: self.value=value
        return self.value


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',default=str(Path(__file__).resolve().parents[1]/'config/switch_inputs.yaml'))
    args=parser.parse_args(); cfg=yaml.safe_load(Path(args.config).read_text())
    inputs=cfg['inputs']
    if not inputs: raise ValueError('Configure actual auxiliary-contact GPIO lines in switch_inputs.yaml')
    channels=[item['channel'] for item in inputs]; offsets=[item['offset'] for item in inputs]
    if len(set(channels))!=len(channels) or len(set(offsets))!=len(offsets): raise ValueError('Duplicate GPIO channel/offset')
    if any(not isinstance(c,str) or not c for c in channels) or any(type(o) is not int or o<0 for o in offsets):
        raise ValueError('Invalid GPIO channel/offset')
    if any(type(item['active_low']) is not bool for item in inputs): raise ValueError('active_low must be boolean')
    rate=cfg['rate_hz']
    if not math.isfinite(rate) or not 10<=rate<=100: raise ValueError('rate_hz must be 10–100')
    import gpiod
    if not hasattr(gpiod,'LINE_REQ_DIR_IN'):
        raise RuntimeError('This adapter uses Ubuntu python3-libgpiod 1.x; use its system Python binding')
    chip=gpiod.Chip(cfg['chip']); lines=chip.get_lines(offsets)
    lines.request(consumer='piper_estop_feedback',type=gpiod.LINE_REQ_DIR_IN)
    filters=[Debouncer(cfg['debounce_samples']) for _ in inputs]
    rclpy.init(); node=rclpy.create_node('estop_auxiliary_gpio')
    pub=node.create_publisher(String,cfg['topic'],10)
    def publish():
        # Failed reads raise and terminate the node; no guessed state is published.
        levels=lines.get_values()
        stamp=node.get_clock().now().nanoseconds/1e9
        for item,level,debouncer in zip(inputs,levels,filters):
            value=debouncer.update((level==0) if item['active_low'] else (level==1))
            if value is not None:
                pub.publish(String(data=json.dumps(dict(channel=item['channel'],pressed=value,stamp_sec=stamp))))
    node.create_timer(1/rate,publish)
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        lines.release(); chip.close(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__=='__main__':
    try: main()
    except Exception as exc: raise SystemExit(str(exc))
