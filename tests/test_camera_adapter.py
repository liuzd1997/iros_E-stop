import sys
from pathlib import Path
import tempfile
import time
import unittest
import numpy as np
import yaml
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from camera_buttons import CameraButtons, decode_image, load_calibration
from button_perception import synthetic_rgbd


class ImageTests(unittest.TestCase):
    def test_padded_big_endian_depth(self):
        msg = Image(height=2,width=2,encoding='16UC1',is_bigendian=1,step=6)
        msg.data = bytes.fromhex('03e8 07d0 0000 0bb8 0fa0 0000')
        np.testing.assert_allclose(decode_image(msg), [[1,2],[3,4]])
        msg.step = 2
        with self.assertRaises(ValueError):
            decode_image(msg)

    def test_rgb_channel_conversion(self):
        msg = Image(height=1,width=1,encoding='rgb8',step=3,data=bytes([255,0,0]))
        np.testing.assert_array_equal(decode_image(msg), [[[0,0,255]]])

    def test_unmeasured_calibration_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'calibration.yaml'
            path.write_text(yaml.safe_dump(dict(calibrated=False,parent_frame='base_link')))
            with self.assertRaises(ValueError):
                load_calibration(path)


class StreamTests(unittest.TestCase):
    def test_synchronized_ros_stream_then_missing_transform_and_stale_data(self):
        rclpy.init()
        node = rclpy.create_node('test_camera_buttons')
        config = dict(color_topic='/button_test/color',depth_topic='/button_test/depth',
                      camera_info_topic='/button_test/info',stable_frames=5)
        adapter = CameraButtons(node,config,camera_frame_only=True)
        pubs = [node.create_publisher(kind,config[key],qos_profile_sensor_data) for kind,key in
                [(Image,'color_topic'),(Image,'depth_topic'),(CameraInfo,'camera_info_topic')]]
        bgr, depth, k, _, _ = synthetic_rgbd()
        def publish():
            stamp = node.get_clock().now().to_msg()
            info = CameraInfo(height=480,width=640,k=k.ravel().tolist(),d=[0.]*5,distortion_model='plumb_bob')
            info.header.frame_id='test_color_optical_frame'
            info.header.stamp=stamp
            rgb = Image(height=480,width=640,encoding='bgr8',step=640*3,data=bgr.tobytes())
            dep = Image(height=480,width=640,encoding='32FC1',step=640*4,data=depth.tobytes())
            rgb.header=info.header
            dep.header=info.header
            pubs[2].publish(info)
            pubs[0].publish(rgb)
            pubs[1].publish(dep)
        timer = node.create_timer(.12,publish)
        try:
            targets = adapter.wait(3,timeout=10)
            self.assertEqual(len(targets),3)
            self.assertEqual(adapter.frame,'test_color_optical_frame')
            for value in targets.values():
                self.assertAlmostEqual(value['position'][2],.6,places=5)
            # A camera image is insufficient for a robot-frame target without calibration.
            adapter.camera_frame_only=False
            deadline=time.monotonic()+.4
            while time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.02)
            self.assertEqual(adapter.ready(),{})
            self.assertTrue(adapter.last_error)
            adapter.camera_frame_only=True
            self.assertEqual(len(adapter.wait(3,timeout=5)),3)
            timer.cancel()
            deadline=time.monotonic()+.8
            while time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.02)
            self.assertEqual(adapter.ready(),{})
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__=='__main__':
    unittest.main()
