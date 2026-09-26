#!/usr/bin/env python3
"""Read-only SDK, CAN, and setup checks. Never enables or commands the arm."""
import argparse
import importlib.util
import json
from pathlib import Path
import subprocess
from hardware_core import load_settings,require_commissioned
from camera_buttons import load_calibration

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',default=str(ROOT/'config/piper_hardware.yaml'))
    p.add_argument('--require-can',action='store_true')
    args=p.parse_args()
    cfg=load_settings(args.config)
    sdk=all(importlib.util.find_spec(name) is not None for name in ('pyAgxArm','can'))
    result=dict(sdk_installed=sdk,can_interface=cfg['can_port'],can_ready=False)
    try:
        links=json.loads(subprocess.check_output(['ip','-details','-json','link','show','dev',cfg['can_port']],text=True,stderr=subprocess.PIPE))
        link=links[0]; data=link.get('linkinfo',{}).get('info_data',{})
        result['can_ready']=('UP' in link.get('flags',[]) and link.get('linkinfo',{}).get('info_kind')=='can'
            and data.get('bittiming',{}).get('bitrate')==cfg['can_bitrate'] and data.get('state') not in ('BUS-OFF','STOPPED'))
        result['can_state']=data.get('state')
        result['can_bitrate']=data.get('bittiming',{}).get('bitrate')
    except (subprocess.CalledProcessError,FileNotFoundError,ValueError,IndexError) as exc:
        result['can_error']=str(exc)
    calibration,_=load_calibration(cfg['calibration'],allow_assumed=True)
    result['calibration_measured']=calibration.get('calibrated') is True
    result['configured_switch_channels']=[b['channel'] for b in cfg['press']['bindings']]
    try:
        require_commissioned(cfg)
        result['commissioning_complete']=True
    except ValueError as exc:
        result['commissioning_complete']=False; result['commissioning_error']=str(exc)
    result['configuration_ready_for_execution']=bool(sdk and result['can_ready'] and result['calibration_measured']
                                                    and result['commissioning_complete'] and cfg['press']['bindings'])
    result['note']='Configuration checks only; physical feedback, scene and switch associations are checked at runtime.'
    print(json.dumps(result,indent=2))
    if args.require_can and not (sdk and result['can_ready']):
        raise SystemExit('Select/configure the actual Piper CAN adapter at 1 Mbit/s before starting the driver.')


if __name__=='__main__': main()
