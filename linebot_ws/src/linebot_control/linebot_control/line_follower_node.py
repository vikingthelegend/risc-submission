#!/usr/bin/env python3
"""PID line follower.

Sub:  /line/state            linebot_interfaces/LineState
Pub:  /cmd_vel               geometry_msgs/Twist
      /pid/debug             std_msgs/Float32MultiArray
                             [error, p, i, d, angular_z, linear_x]   (for PlotJuggler / rqt_plot)
Srv:  /line_follower/enable  std_srvs/SetBool

Sign convention: error > 0 means the line is to the RIGHT of the image centre, so the
robot must turn right = NEGATIVE angular.z (ROS: +z is counter-clockwise). Therefore
    angular.z = -PID(error)
The PID runs once per new image (dt from the image timestamps), so it is not aliased
against the camera rate. A 20 Hz watchdog timer handles "line lost" behaviour.

Live tuning:   ros2 param set /line_follower kp 2.5     (use 2.5, not 2, for doubles)
"""
import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import SetBool

from linebot_interfaces.msg import LineState
from linebot_control.control_law import command_from_error
from linebot_control.pid import PID

NOMINAL_DT = 1.0 / 30.0


class LineFollowerNode(Node):
    def __init__(self):
        super().__init__('line_follower')

        # ---- parameters -------------------------------------------------
        self.declare_parameter('state_topic', '/line/state')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('enabled', True)
        # PID on the normalised error -> angular velocity [rad/s]
        self.declare_parameter('kp', 2.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.2)
        self.declare_parameter('i_limit', 1.0)
        self.declare_parameter('d_filter', 0.7)
        self.declare_parameter('max_angular', 2.0)
        # speed scheduling: slow down when the line is off-centre / curving away
        self.declare_parameter('max_speed', 0.25)
        self.declare_parameter('min_speed', 0.08)
        self.declare_parameter('slow_gain', 0.6)
        # line lost behaviour
        self.declare_parameter('lost_hold_time', 0.25)      # s: coast on last command
        self.declare_parameter('lost_search_angular', 0.8)  # rad/s: spin towards last seen side
        self.declare_parameter('lost_stop_after', 8.0)      # s: give up and stop

        self.cfg = {}
        for name in ('kp', 'ki', 'kd', 'i_limit', 'd_filter', 'max_angular', 'max_speed',
                     'min_speed', 'slow_gain', 'lost_hold_time', 'lost_search_angular',
                     'lost_stop_after'):
            self.cfg[name] = float(self.get_parameter(name).value)
        self.enabled = bool(self.get_parameter('enabled').value)

        self.pid = PID()
        self._apply_pid_cfg()
        self.add_on_set_parameters_callback(self._on_params)

        # ---- state ------------------------------------------------------
        self._tracking = False
        self._prev_stamp = None
        self._last_detect = self.get_clock().now()
        self._last_err_sign = 1.0
        self._cmd = (0.0, 0.0)
        self._lost_logged = False

        # ---- ROS interfaces --------------------------------------------
        self.cmd_pub = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.dbg_pub = self.create_publisher(Float32MultiArray, '/pid/debug', 10)
        self.create_subscription(LineState, self.get_parameter('state_topic').value,
                                 self.on_state, 10)
        self.create_service(SetBool, '~/enable', self.on_enable)
        self.create_timer(0.05, self.watchdog)
        self.get_logger().info('line_follower ready (enabled=%s)' % self.enabled)

    # ------------------------------------------------------------------
    def _apply_pid_cfg(self):
        c = self.cfg
        self.pid.kp, self.pid.ki, self.pid.kd = c['kp'], c['ki'], c['kd']
        self.pid.i_limit = c['i_limit']
        self.pid.d_filter = min(max(c['d_filter'], 0.0), 0.99)
        self.pid.out_min, self.pid.out_max = -c['max_angular'], c['max_angular']

    def _on_params(self, params):
        for p in params:
            if p.name in self.cfg:
                self.cfg[p.name] = float(p.value)
            elif p.name == 'enabled':
                self._set_enabled(bool(p.value))
        self._apply_pid_cfg()
        return SetParametersResult(successful=True)

    def _set_enabled(self, value: bool):
        if value == self.enabled:
            return
        self.enabled = value
        self.pid.reset()
        self._tracking = False
        if not value:
            self._publish(0.0, 0.0)
        self.get_logger().info('follower %s' % ('ENABLED' if value else 'DISABLED'))

    def on_enable(self, req, resp):
        self._set_enabled(req.data)
        resp.success = True
        resp.message = 'enabled' if self.enabled else 'disabled'
        return resp

    # ------------------------------------------------------------------
    def _publish(self, v, w):
        self._cmd = (v, w)
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(w)
        self.cmd_pub.publish(t)

    def on_state(self, msg: LineState):
        stamp = Time.from_msg(msg.header.stamp)
        dt = NOMINAL_DT
        if self._prev_stamp is not None:
            dt = (stamp - self._prev_stamp).nanoseconds * 1e-9
        self._prev_stamp = stamp

        if not self.enabled or not msg.detected:
            self._tracking = False        # watchdog takes over if this persists
            return

        if not self._tracking:            # (re)acquired the line: start clean
            self.pid.reset()
            self._tracking = True
            dt = NOMINAL_DT
            if self._lost_logged:
                self.get_logger().info('Line re-acquired')
                self._lost_logged = False
        if dt <= 0.0 or dt > 0.5:         # clock jump or long gap
            dt = NOMINAL_DT

        self._last_detect = self.get_clock().now()
        if abs(msg.error) > 0.02:
            self._last_err_sign = 1.0 if msg.error > 0 else -1.0

        v, w = command_from_error(self.pid, msg.error, msg.far_error, self.cfg, dt)

        self._publish(v, w)
        dbg = Float32MultiArray()
        dbg.data = [float(msg.error), self.pid.p_term, self.pid.i_term, self.pid.d_term,
                    float(w), float(v)]
        self.dbg_pub.publish(dbg)

    # ------------------------------------------------------------------
    def watchdog(self):
        """Runs at 20 Hz. Takes over when no fresh detection has arrived."""
        if not self.enabled:
            return
        lost_for = (self.get_clock().now() - self._last_detect).nanoseconds * 1e-9
        c = self.cfg
        if lost_for <= c['lost_hold_time']:
            return                                     # coast on the last command

        if not self._lost_logged:
            self.get_logger().warn('Line lost - searching')
            self._lost_logged = True
        self.pid.reset()
        self._tracking = False

        if lost_for < c['lost_stop_after']:
            # spin towards the side where the line was last seen
            self._publish(0.0, -self._last_err_sign * c['lost_search_angular'])
        else:
            if lost_for < c['lost_stop_after'] + 0.1:
                self.get_logger().error('Line not found - stopping')
            self._publish(0.0, 0.0)

    def stop(self):
        try:
            self._publish(0.0, 0.0)
        except Exception:  # noqa: BLE001  (context may already be shut down)
            pass


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
