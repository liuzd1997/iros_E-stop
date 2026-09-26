import sys
from pathlib import Path
import unittest
import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from button_perception import detect_buttons, synthetic_rgbd, Tracker, orientation
from calibrate_camera import fit_transform


class PerceptionTests(unittest.TestCase):
    def test_three_buttons_in_base_frame(self):
        detections = detect_buttons(*synthetic_rgbd())
        self.assertEqual(len(detections), 3)
        positions = sorted([d.position.tolist() for d in detections], key=lambda p:p[1])
        np.testing.assert_allclose(positions, [[.5,-.12,.3],[.5,0,.3],[.5,.12,.3]], atol=.0001)
        for d in detections:
            np.testing.assert_allclose(d.direction, [1,0,0], atol=1e-5)

    def test_invalid_depth_is_not_a_target(self):
        image, depth, *rest = synthetic_rgbd()
        for value in (0., np.nan, np.inf):
            self.assertEqual(detect_buttons(image, np.full_like(depth, value), *rest), [])

    def test_red_without_yellow_is_rejected(self):
        image, *rest = synthetic_rgbd()
        image[(image[:,:,1] > 180)] = 125
        self.assertEqual(detect_buttons(image, *rest), [])

    def test_flat_painted_discs_are_rejected(self):
        image, depth, *rest = synthetic_rgbd()
        self.assertEqual(detect_buttons(image, np.full_like(depth,.6), *rest), [])

    def test_ids_survive_order_change_and_require_stability_after_occlusion(self):
        detections = detect_buttons(*synthetic_rgbd())
        tracker = Tracker()
        for i in range(5):
            tracker.update(detections if i%2 else detections[::-1], i*.1)
        before = tracker.ready(.4)
        self.assertEqual(len(before), 3)
        self.assertEqual(tracker.ready(1.), {})
        tracker.update([], .5)
        for i in range(4):
            tracker.update(detections, .6+i*.1)
            self.assertEqual(tracker.ready(.6+i*.1), {})
        tracker.update(detections, 1.)
        after = tracker.ready(1.)
        self.assertEqual(set(before), set(after))
        for key in before:
            np.testing.assert_allclose(before[key]['position'], after[key]['position'])

    def test_orientation_aligns_press_axis(self):
        for d in ([1.,0,0], [.8,.2,.3], [0.,0,1.]):
            unit = np.array(d)/np.linalg.norm(d)
            np.testing.assert_allclose(Rotation.from_quat(orientation(d)).apply([0,0,1]), unit, atol=1e-12)

    def test_bad_transform_rejected(self):
        *rest, transform = synthetic_rgbd()
        transform[0,0] = 2
        with self.assertRaises(ValueError):
            detect_buttons(*rest, transform)

    def test_calibration_recovers_known_transform(self):
        camera = np.random.default_rng(7).uniform([-.2,-.2,.4],[.2,.2,.8], (12,3))
        expected = synthetic_rgbd()[-1]
        base = camera @ expected[:3,:3].T+expected[:3,3]
        actual, errors = fit_transform(camera,base)
        np.testing.assert_allclose(actual, expected, atol=1e-12)
        self.assertLess(errors.max(), 1e-12)
        base[0] += .03
        with self.assertRaises(ValueError):
            fit_transform(camera,base)
        with self.assertRaises(ValueError):
            fit_transform(np.zeros((6,3)),np.zeros((6,3)))


if __name__ == '__main__':
    unittest.main()
