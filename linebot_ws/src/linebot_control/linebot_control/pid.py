"""Plain-Python PID controller (no ROS dependency, so it is unit-testable)."""
import math


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


class PID:
    """PID with output saturation, conditional-integration anti-windup and a
    low-pass filtered derivative.

    u = kp*e + ki*integral(e dt) + kd*filtered(de/dt)

    d_filter in [0, 1): 0 = raw derivative, closer to 1 = heavier smoothing.
    i_limit clamps |integral| (units: error * seconds).
    """

    def __init__(self, kp=2.0, ki=0.0, kd=0.5,
                 out_min=-math.inf, out_max=math.inf,
                 i_limit=math.inf, d_filter=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self.i_limit = i_limit
        self.d_filter = d_filter
        self.reset()

    def reset(self):
        self.integral = 0.0
        self._prev_error = None
        self._d = 0.0
        # last computed terms, handy for debugging / plotting
        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0
        self.output = 0.0

    def update(self, error, dt):
        if dt <= 0.0:
            return self.output

        p = self.kp * error

        if self._prev_error is None:
            raw_d = 0.0                      # no derivative kick on first sample
        else:
            raw_d = (error - self._prev_error) / dt
        self._d = self.d_filter * self._d + (1.0 - self.d_filter) * raw_d
        d = self.kd * self._d
        self._prev_error = error

        candidate_integral = _clamp(self.integral + error * dt, -self.i_limit, self.i_limit)
        unsat = p + self.ki * candidate_integral + d
        out = _clamp(unsat, self.out_min, self.out_max)

        # Conditional integration: don't keep winding up while saturated
        # in the direction the error is pushing.
        saturated = out != unsat
        if not (saturated and error * unsat > 0.0):
            self.integral = candidate_integral

        self.p_term = p
        self.i_term = self.ki * self.integral
        self.d_term = d
        self.output = out
        return out
