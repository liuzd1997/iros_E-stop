"""ROS-only fixture: ideal joints, synthetic images and FK-derived fake switches."""
import argparse
import json
import os
from pathlib import Path
import sys
import numpy as np
import rclpy
from sensor_msgs.msg import Image,CameraInfo
from std_msgs.msg import String
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from rclpy.qos import qos_profile_sensor_data
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from test_piper_controller import FakeDriver
from hardware_core import load_settings,JOINTS
from button_perception import synthetic_rgbd


class Fixture(FakeDriver):
    def __init__(self,cfg):
        super().__init__()
        self.cfg=cfg
        self.q=[0.]*6
        self.bgr,self.depth,self.k,_,_=synthetic_rgbd()
        self.rgb=self.create_publisher(Image,'/camera/camera/color/image_raw',qos_profile_sensor_data)
        self.dep=self.create_publisher(Image,'/camera/camera/aligned_depth_to_color/image_raw',qos_profile_sensor_data)
        self.info=self.create_publisher(CameraInfo,'/camera/camera/color/camera_info',qos_profile_sensor_data)
        self.switch=self.create_publisher(String,'/estop/switch_states',10)
        self.fk=self.create_client(GetPositionFK,'/compute_fk')
        self.pending=False
        self.states={binding['channel']:False for binding in cfg['press']['bindings']}
        self.create_timer(.1,self.perception)
        self.create_timer(.04,self.switches)

    def perception(self):
        info=CameraInfo(height=480,width=640,k=self.k.ravel().tolist(),d=[0.]*5,distortion_model='plumb_bob')
        info.header.frame_id='camera_color_optical_frame'; info.header.stamp=self.get_clock().now().to_msg()
        rgb=Image(height=480,width=640,encoding='bgr8',step=640*3,data=self.bgr.tobytes()); rgb.header=info.header
        dep=Image(height=480,width=640,encoding='32FC1',step=640*4,data=self.depth.tobytes()); dep.header=info.header
        self.info.publish(info); self.rgb.publish(rgb); self.dep.publish(dep)

    def switches(self):
        stamp=self.get_clock().now().nanoseconds/1e9
        for channel,pressed in self.states.items():
            self.switch.publish(String(data=json.dumps(dict(channel=channel,pressed=pressed,stamp_sec=stamp))))
        if not self.pending and self.fk.service_is_ready():
            req=GetPositionFK.Request(); req.header.frame_id='base_link'; req.fk_link_names=['link6']
            req.robot_state=RobotState(joint_state=JointState(name=JOINTS+['gripper'],position=self.q+[0.]))
            future=self.fk.call_async(req); self.pending=True
            future.add_done_callback(self.got_fk)

    def got_fk(self,future):
        self.pending=False
        result=future.result()
        if result.error_code.val!=1: return
        p=result.pose_stamped[0].pose; q=p.orientation; offset=self.cfg['tip_offset_z']
        tip=np.array([p.position.x+2*(q.x*q.z+q.w*q.y)*offset,
                      p.position.y+2*(q.y*q.z-q.w*q.x)*offset,
                      p.position.z+(1-2*(q.x*q.x+q.y*q.y))*offset])
        for binding in self.cfg['press']['bindings']:
            delta=tip-np.asarray(binding['position'])
            if .0015<=delta[0]<=.01 and np.linalg.norm(delta[1:])<.01:
                self.states[binding['channel']]=True  # Latching simulated switch.


if __name__=='__main__':
    if os.environ.get('ROS_DOMAIN_ID')!='80': raise SystemExit('Requires test domain 80')
    parser=argparse.ArgumentParser(); parser.add_argument('--config',required=True); args=parser.parse_args()
    rclpy.init(); node=Fixture(load_settings(args.config))
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
