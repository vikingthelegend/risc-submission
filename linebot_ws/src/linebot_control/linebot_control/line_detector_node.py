#!/usr/bin/env python3
"""ROS 2 wrapper around vision.LineTracker.

Sub:  /camera/image_raw        sensor_msgs/Image
Pub:  /line/state              linebot_interfaces/LineState
      /line/debug_image        sensor_msgs/Image   (only built when someone subscribes)

All parameters are re-read every frame, so you can tune live:
    ros2 param set /line_detector threshold 100
"""
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from linebot_interfaces.msg import LineState
from linebot_control.vision import LineTracker, TrackerConfig


class LineDetectorNode(Node):
    def __init__(self):
        super().__init__('line_detector')
        d = TrackerConfig()
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('state_topic', '/line/state')
        self.declare_parameter('debug_topic', '/line/debug_image')
        self.declare_parameter('publish_debug', True)
        self.declare_parameter('show_window', False)     # cv2.imshow (needs a display)
        self.declare_parameter('roi_top', d.roi_top)
        self.declare_parameter('roi_bottom', d.roi_bottom)
        self.declare_parameter('slice_weights', [float(x) for x in d.slice_weights])
        self.declare_parameter('threshold_mode', d.threshold_mode)
        self.declare_parameter('threshold', d.threshold)
        self.declare_parameter('blur_kernel', d.blur_kernel)
        self.declare_parameter('morph_kernel', d.morph_kernel)
        self.declare_parameter('min_area', d.min_area)
        self.declare_parameter('max_slice_jump', d.max_slice_jump)

        self.bridge = CvBridge()
        self.tracker = LineTracker(self._read_cfg())

        self.state_pub = self.create_publisher(LineState, self._p('state_topic'), 10)
        self.debug_pub = self.create_publisher(Image, self._p('debug_topic'), 1)
        self.create_subscription(Image, self._p('image_topic'), self.on_image,
                                 qos_profile_sensor_data)
        self.get_logger().info(f"Listening on {self._p('image_topic')}, "
                               f"publishing {self._p('state_topic')}")

    def _p(self, name):
        return self.get_parameter(name).value

    def _read_cfg(self) -> TrackerConfig:
        return TrackerConfig(
            roi_top=float(self._p('roi_top')),
            roi_bottom=float(self._p('roi_bottom')),
            slice_weights=tuple(float(x) for x in self._p('slice_weights')),
            threshold_mode=str(self._p('threshold_mode')),
            threshold=int(self._p('threshold')),
            blur_kernel=int(self._p('blur_kernel')),
            morph_kernel=int(self._p('morph_kernel')),
            min_area=float(self._p('min_area')),
            max_slice_jump=float(self._p('max_slice_jump')),
        )

    def on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'cv_bridge conversion failed: {e}', throttle_duration_sec=2.0)
            return

        self.tracker.cfg = self._read_cfg()
        res = self.tracker.process(frame)

        out = LineState()
        out.header = msg.header
        out.detected = res.detected
        out.error = float(res.error)
        out.near_error = float(res.near_error)
        out.far_error = float(res.far_error)
        out.slices_detected = len(res.slices)
        self.state_pub.publish(out)

        want_window = bool(self._p('show_window'))
        want_topic = bool(self._p('publish_debug')) and self.debug_pub.get_subscription_count() > 0
        if want_window or want_topic:
            dbg = self.tracker.draw_debug(frame, res)
            if want_topic:
                dbg_msg = self.bridge.cv2_to_imgmsg(dbg, encoding='bgr8')
                dbg_msg.header = msg.header
                self.debug_pub.publish(dbg_msg)
            if want_window:
                cv2.imshow('line_detector', dbg)
                cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = LineDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
