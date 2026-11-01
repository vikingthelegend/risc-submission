import unittest

import cv2
import numpy as np

from linebot_control.vision import LineTracker, TrackerConfig

W, H = 640, 480


def blank():
    return np.full((H, W, 3), 230, np.uint8)      # light floor


def draw_line(img, pts, thickness=40):
    cv2.polylines(img, [np.array(pts, np.int32)], False, (10, 10, 10), thickness)
    return img


class TestVision(unittest.TestCase):
    def test_centered_line(self):
        img = draw_line(blank(), [(320, H), (320, 0)])
        r = LineTracker().process(img)
        self.assertTrue(r.detected)
        self.assertEqual(len(r.slices), 3)
        self.assertLess(abs(r.error), 0.02)

    def test_line_right_gives_positive_error(self):
        img = draw_line(blank(), [(480, H), (480, 0)])
        r = LineTracker().process(img)
        self.assertGreater(r.error, 0.4)

    def test_line_left_gives_negative_error(self):
        img = draw_line(blank(), [(160, H), (160, 0)])
        r = LineTracker().process(img)
        self.assertLess(r.error, -0.4)

    def test_no_line(self):
        r = LineTracker().process(blank())
        self.assertFalse(r.detected)
        self.assertEqual(r.error, 0.0)

    def test_lookahead_sees_curve(self):
        # line starts centred at the bottom and bends to the right
        img = draw_line(blank(), [(320, H), (320, 330), (520, 180)])
        r = LineTracker().process(img)
        self.assertTrue(r.detected)
        self.assertGreater(r.far_error, r.near_error + 0.2)

    def test_ignores_small_noise(self):
        img = blank()
        cv2.circle(img, (50, 400), 3, (0, 0, 0), -1)
        self.assertFalse(LineTracker().process(img).detected)

    def test_tracks_nearest_line_at_fork(self):
        t = LineTracker()
        t.process(draw_line(blank(), [(300, H), (300, 0)]))            # lock on at x~300
        img = draw_line(blank(), [(300, H), (300, 0)])
        draw_line(img, [(580, H), (580, 0)])                           # second line appears
        r = t.process(img)
        self.assertLess(abs(r.near_error), 0.1)

    def test_otsu_and_adaptive_modes(self):
        img = draw_line(blank(), [(400, H), (400, 0)])
        for mode in ('otsu', 'adaptive'):
            cfg = TrackerConfig(threshold_mode=mode)
            r = LineTracker(cfg).process(img)
            self.assertTrue(r.detected, mode)
            self.assertAlmostEqual(r.error, (400 - 320) / 320, delta=0.05)

    def test_debug_image_shape(self):
        img = draw_line(blank(), [(320, H), (320, 0)])
        t = LineTracker()
        dbg = t.draw_debug(img, t.process(img))
        self.assertEqual(dbg.shape, img.shape)


if __name__ == '__main__':
    unittest.main()
