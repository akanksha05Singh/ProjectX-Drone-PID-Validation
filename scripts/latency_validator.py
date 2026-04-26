#!/usr/bin/env python3
"""
Week 3 — End-to-End Latency Validator  |  Reformerz Healthcare Robotics
========================================================================
Measures the full pipeline latency:
  Camera Image → /detected_objects → /tracking/error → /mavros/.../cmd_vel

How it works:
  • Subscribes to all three topics simultaneously.
  • Uses each message's header.stamp (set by the perception node from the
    original camera frame) to track the same frame through the pipeline.
  • Reports rolling statistics every 5 seconds.

Pass/Fail criteria (Shiva's requirements):
  Stage                 Target      Hard limit
  ─────────────────────────────────────────────
  Perception latency    < 80 ms     < 120 ms
  Tracking latency      < 5  ms     < 20  ms
  E2E (cam → cmd_vel)   < 100 ms    < 150 ms

Usage:
  # In a sourced terminal, after all other nodes are running:
  python3 scripts/latency_validator.py
"""

import collections
import statistics
import time

import rclpy
from geometry_msgs.msg import Point, TwistStamped
from rclpy.node import Node
from rclpy.qos import (
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

# ── Thresholds (ms) ──────────────────────────────────────────────────────────
PERCEPTION_TARGET    =  80.0
PERCEPTION_HARD      = 120.0
TRACKING_TARGET      =   5.0
TRACKING_HARD        =  20.0
E2E_TARGET           = 100.0
E2E_HARD             = 150.0
WINDOW               = 50        # rolling window size for stats


def _reliable_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def _sensor_qos() -> QoSProfile:
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
    )


def _stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class LatencyValidator(Node):
    def __init__(self) -> None:
        super().__init__("latency_validator")

        # Rolling latency windows
        self._perc_lat:  collections.deque = collections.deque(maxlen=WINDOW)
        self._track_lat: collections.deque = collections.deque(maxlen=WINDOW)
        self._e2e_lat:   collections.deque = collections.deque(maxlen=WINDOW)

        # Timestamps indexed by frame_id/stamp key
        self._cam_stamps:  dict[float, float] = {}   # ros_stamp → wall_time
        self._det_stamps:  dict[float, float] = {}
        self._err_t:       float = 0.0
        self._last_det_t:  float = 0.0

        # Subscribers
        self.create_subscription(
            Image, "/camera/image_raw", self._cam_cb, _sensor_qos()
        )
        self.create_subscription(
            Detection2DArray, "/detected_objects", self._det_cb, _reliable_qos()
        )
        self.create_subscription(
            Point, "/tracking/error", self._err_cb, _reliable_qos()
        )
        self.create_subscription(
            TwistStamped,
            "/mavros/setpoint_velocity/cmd_vel",
            self._cmd_cb,
            _reliable_qos(depth=1),
        )

        self.create_timer(5.0, self._report)

        self.get_logger().info(
            "LatencyValidator running — reporting every 5 s\n"
            f"  Targets: perception<{PERCEPTION_TARGET}ms  "
            f"tracking<{TRACKING_TARGET}ms  e2e<{E2E_TARGET}ms"
        )

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _cam_cb(self, msg: Image) -> None:
        key = _stamp_to_sec(msg.header.stamp)
        self._cam_stamps[key] = time.monotonic()
        # Keep dict bounded
        if len(self._cam_stamps) > 200:
            oldest = min(self._cam_stamps)
            del self._cam_stamps[oldest]

    def _det_cb(self, msg: Detection2DArray) -> None:
        now = time.monotonic()
        key = _stamp_to_sec(msg.header.stamp)
        self._det_stamps[key] = now
        self._last_det_t = now

        if key in self._cam_stamps:
            lat_ms = (now - self._cam_stamps[key]) * 1000.0
            self._perc_lat.append(lat_ms)

        if len(self._det_stamps) > 200:
            oldest = min(self._det_stamps)
            del self._det_stamps[oldest]

    def _err_cb(self, msg: Point) -> None:
        self._err_t = time.monotonic()

        # Tracking latency = error publish time - last detection publish time
        if self._last_det_t > 0.0:
            lat_ms = (self._err_t - self._last_det_t) * 1000.0
            if 0.0 < lat_ms < 1000.0:   # sanity check
                self._track_lat.append(lat_ms)

    def _cmd_cb(self, msg: TwistStamped) -> None:
        now = time.monotonic()
        key = _stamp_to_sec(msg.header.stamp)

        # E2E: find the camera frame closest in time to this cmd_vel stamp
        if self._cam_stamps:
            closest = min(self._cam_stamps, key=lambda k: abs(k - key))
            if abs(closest - key) < 1.0:   # within 1 second → same pipeline
                lat_ms = (now - self._cam_stamps[closest]) * 1000.0
                if 0.0 < lat_ms < 5000.0:
                    self._e2e_lat.append(lat_ms)

    # ── Report ───────────────────────────────────────────────────────────────

    def _report(self) -> None:
        lines = ["\n" + "═" * 62]
        lines.append("  LATENCY REPORT  (rolling window = last {} samples)".format(WINDOW))
        lines.append("═" * 62)

        for label, data, target, hard in [
            ("Perception (cam→det)",    self._perc_lat,  PERCEPTION_TARGET, PERCEPTION_HARD),
            ("Tracking   (det→error)",  self._track_lat, TRACKING_TARGET,   TRACKING_HARD),
            ("E2E        (cam→cmd)",    self._e2e_lat,   E2E_TARGET,        E2E_HARD),
        ]:
            if len(data) < 3:
                lines.append(f"  {label:<28}  — insufficient data ({len(data)} samples)")
                continue

            avg  = statistics.mean(data)
            p95  = sorted(data)[int(len(data) * 0.95)]
            mx   = max(data)

            if avg <= target:
                verdict = "✓ PASS"
            elif avg <= hard:
                verdict = "⚠ MARGINAL"
            else:
                verdict = "✗ FAIL"

            lines.append(
                f"  {label:<28}  avg={avg:6.1f}ms  "
                f"p95={p95:6.1f}ms  max={mx:6.1f}ms  {verdict}"
            )

        lines.append("═" * 62)
        self.get_logger().info("\n".join(lines))


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main(args=None) -> None:
    rclpy.init(args=args)
    node = LatencyValidator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
