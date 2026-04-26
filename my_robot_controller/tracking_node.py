#!/usr/bin/env python3
"""
Week 3 — Tracking Node  |  Reformerz Healthcare Robotics Internship
====================================================================
Subscribes : /detected_objects  (vision_msgs/Detection2DArray)
Publishes  : /tracking/error    (geometry_msgs/Point)
             /tracking/status   (std_msgs/String)   — "TRACKING" | "LOST"

Architecture:
  • Filters detections for the highest-confidence "person" in each frame.
  • Computes pixel error relative to the image centre.
  • Normalises error to [-1.0, 1.0] so the control node is resolution-agnostic.
  • Publishes a zero-error + LOST status immediately when the target disappears,
    so the control node can hover safely.

Coordinate convention (matches perception_node 128×128 output):
  error.x  >0  → target is RIGHT  of centre  (drone should yaw/strafe right)
  error.y  >0  → target is BELOW  centre      (drone should move forward / descend)
  error.z       = 0.0 (reserved for depth / altitude error in a future week)
"""

import time

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import (
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray


# ---------------------------------------------------------------------------
# QoS helpers
# ---------------------------------------------------------------------------
def _reliable_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
    )


# ---------------------------------------------------------------------------
# Tracking Node
# ---------------------------------------------------------------------------
class TrackingNode(Node):
    """
    Converts Detection2DArray → normalised pixel error for a P-controller.

    Parameters (tunable at launch):
      image_width   — width of the model's coordinate space  (default: 128)
      image_height  — height of the model's coordinate space (default: 128)
      target_class  — COCO class name to track               (default: "person")
      lost_timeout  — seconds before declaring target LOST   (default: 0.5)
      input_topic   — detection source                       (default: /detected_objects)
    """

    STATUS_TRACKING = "TRACKING"
    STATUS_LOST     = "LOST"

    def __init__(self) -> None:
        super().__init__("tracking_node")

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter("image_width",   128)
        self.declare_parameter("image_height",  128)
        self.declare_parameter("target_class",  "person")
        self.declare_parameter("lost_timeout",  0.5)
        self.declare_parameter("input_topic",   "/detected_objects")

        # ── Stress Parameters (Week 4) ──────────────────────────────────────
        self.declare_parameter("imu_noise", 0.0)    # Gaussian noise std dev
        self.declare_parameter("gps_drift", 0.0)    # cumulative drift rate

        self._img_w:        int   = self.get_parameter("image_width").value
        self._img_h:        int   = self.get_parameter("image_height").value
        self._target_class: str   = self.get_parameter("target_class").value
        self._lost_timeout: float = self.get_parameter("lost_timeout").value
        input_topic:        str   = self.get_parameter("input_topic").value

        self._cx = self._img_w / 2.0   # image centre x
        self._cy = self._img_h / 2.0   # image centre y

        # ── Stress State ─────────────────────────────────────────────────────
        self._drift_x = 0.0
        self._drift_y = 0.0
        self._last_t = self.get_clock().now().nanoseconds / 1e9

        # ── State ────────────────────────────────────────────────────────────
        self._last_seen: float = 0.0
        self._status:   str    = self.STATUS_LOST

        # ── Publishers ───────────────────────────────────────────────────────
        self._err_pub = self.create_publisher(
            Point, "/tracking/error", _reliable_qos()
        )
        self._status_pub = self.create_publisher(
            String, "/tracking/status", _reliable_qos()
        )

        # ── Subscriber ───────────────────────────────────────────────────────
        self._sub = self.create_subscription(
            Detection2DArray,
            input_topic,
            self._detection_callback,
            _reliable_qos(),
        )

        # ── Watchdog timer: publishes LOST if no detection for lost_timeout ──
        self._watchdog = self.create_timer(0.1, self._watchdog_callback)

        self.get_logger().info(
            f"TrackingNode ready — tracking '{self._target_class}' "
            f"in {self._img_w}×{self._img_h} space  "
            f"(lost_timeout={self._lost_timeout}s)"
        )

    # ── Detection callback ───────────────────────────────────────────────────

    def _detection_callback(self, msg: Detection2DArray) -> None:
        """Select the highest-confidence 'person', compute and publish error."""
        best_score   = -1.0
        best_cx      = None
        best_cy      = None

        for det in msg.detections:
            for hyp in det.results:
                if hyp.hypothesis.class_id == self._target_class:
                    if hyp.hypothesis.score > best_score:
                        best_score = hyp.hypothesis.score
                        best_cx    = det.bbox.center.position.x
                        best_cy    = det.bbox.center.position.y

        if best_cx is None:
            # No person in this frame — watchdog will handle LOST after timeout
            return

        # Compute raw pixel error and normalise to [-1, 1]
        raw_ex = best_cx - self._cx
        raw_ey = best_cy - self._cy
        norm_ex = raw_ex / self._cx   # divides by half-width
        norm_ey = raw_ey / self._cy   # divides by half-height

        # ── Stress Injection ────────────────────────────────────────────────
        now = self.get_clock().now().nanoseconds / 1e9
        dt = now - self._last_t
        self._last_t = now

        # 1. IMU Noise (Gaussian random walk / jitter)
        import random
        noise_std = self.get_parameter("imu_noise").value
        if noise_std > 0:
            norm_ex += random.gauss(0, noise_std)
            norm_ey += random.gauss(0, noise_std)

        # 2. GPS Drift (slow cumulative offset)
        drift_rate = self.get_parameter("gps_drift").value
        if drift_rate > 0:
            self._drift_x += drift_rate * dt
            self._drift_y += drift_rate * dt
            norm_ex += self._drift_x
            norm_ey += self._drift_y

        err_msg = Point()
        err_msg.x = norm_ex
        err_msg.y = norm_ey
        err_msg.z = 0.0
        self._err_pub.publish(err_msg)

        self._last_seen = time.monotonic()

        if self._status != self.STATUS_TRACKING:
            self._status = self.STATUS_TRACKING
            self.get_logger().info(
                f"[TRACKING] Target acquired — "
                f"bbox_center=({best_cx:.1f},{best_cy:.1f})  "
                f"error=({norm_ex:+.3f},{norm_ey:+.3f})  "
                f"conf={best_score:.2f}"
            )

        status_msg = String()
        status_msg.data = self.STATUS_TRACKING
        self._status_pub.publish(status_msg)

    # ── Watchdog callback ────────────────────────────────────────────────────

    def _watchdog_callback(self) -> None:
        """Publish zero-error + LOST if target has not been seen recently."""
        elapsed = time.monotonic() - self._last_seen
        if elapsed < self._lost_timeout:
            return

        if self._status != self.STATUS_LOST:
            self._status = self.STATUS_LOST
            self.get_logger().warning(
                f"[TRACKING] Target LOST for {elapsed:.1f}s — "
                "publishing zero error. Control node will hover."
            )

        # Publish zero error so the control node can react immediately
        self._err_pub.publish(Point())

        status_msg = String()
        status_msg.data = self.STATUS_LOST
        self._status_pub.publish(status_msg)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
