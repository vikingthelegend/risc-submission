import math
import unittest

from linebot_control.pid import PID


class TestPID(unittest.TestCase):
    def test_proportional(self):
        pid = PID(kp=2.0)
        self.assertAlmostEqual(pid.update(0.5, 0.1), 1.0)

    def test_saturation(self):
        pid = PID(kp=10.0, out_min=-1.0, out_max=1.0)
        self.assertEqual(pid.update(1.0, 0.1), 1.0)
        self.assertEqual(pid.update(-1.0, 0.1), -1.0)

    def test_integral_accumulates(self):
        pid = PID(ki=1.0)
        out = 0.0
        for _ in range(10):
            out = pid.update(1.0, 0.1)
        self.assertAlmostEqual(out, 1.0, places=6)

    def test_anti_windup(self):
        pid = PID(kp=1.0, ki=5.0, out_min=-1.0, out_max=1.0)
        for _ in range(1000):
            pid.update(5.0, 0.01)               # saturated hard
        self.assertLess(pid.integral, 1.0)       # did not wind up
        # and it recovers quickly once the error flips sign
        for _ in range(20):
            out = pid.update(-1.0, 0.01)
        self.assertLess(out, 0.0)

    def test_no_derivative_kick_on_first_sample(self):
        pid = PID(kd=1.0)
        self.assertEqual(pid.update(100.0, 0.1), 0.0)

    def test_derivative(self):
        pid = PID(kd=1.0)
        pid.update(0.0, 0.1)
        self.assertAlmostEqual(pid.update(0.1, 0.1), 1.0)

    def test_derivative_filter_smooths(self):
        raw, filt = PID(kd=1.0), PID(kd=1.0, d_filter=0.8)
        raw.update(0.0, 0.1)
        filt.update(0.0, 0.1)
        self.assertLess(abs(filt.update(1.0, 0.1)), abs(raw.update(1.0, 0.1)))

    def test_reset(self):
        pid = PID(kp=1.0, ki=1.0)
        pid.update(1.0, 0.1)
        pid.reset()
        self.assertEqual(pid.integral, 0.0)
        self.assertEqual(pid.output, 0.0)

    def test_zero_dt_is_ignored(self):
        pid = PID(kp=1.0)
        pid.update(1.0, 0.1)
        self.assertEqual(pid.update(5.0, 0.0), 1.0)

    def test_closed_loop_converges(self):
        """Simple plant: heading rate = u, line at fixed lateral offset."""
        pid = PID(kp=2.0, ki=0.5, kd=0.1, out_min=-2, out_max=2, d_filter=0.5)
        x, dt = 1.0, 0.03
        for _ in range(600):
            x -= pid.update(x, dt) * dt
        self.assertLess(abs(x), 0.01)


if __name__ == '__main__':
    unittest.main()
