#!/usr/bin/env python3
"""
scenario_runner.py — Automated Test Scenario Injector
======================================================
Reformerz Healthcare | Drone Tracking Pipeline v2

Acts as a transparent middleware between perception_node and tracking_node:
  perception_node  →  /detected_objects
                       ↓
               [scenario_runner]
                       ↓
                /detected_objects_filtered
                       ↓
               tracking_node

Tracking_node must be configured with:
  input_topic: /detected_objects_filtered

Scenarios
---------
  normal     — Pass-through, no modification
  flicker    — 30% random drop probability per frame
  occlusion  — Block all detections for 2 s every 5 s
  high_error — Inject +0.3 normalised offset to bbox center (simulates off-centre target)

Usage
-----
  ros2 run my_robot_controller_v2 scenario_runner \\
      --ros-args -p scenario:=occlusion

  ros2 run my_robot_controller_v2 scenario_runner \\
      --ros-args -p scenario:=high_error -p error_offset_x:=0.3
"""

import random
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from vision_msgs.msg import Detection2DArray

SCENARIOS = ("normal", "flicker", "occlusion", "high_error")


def _reliable_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
    )


class ScenarioRunner(Node):
    """
    Intercepts /detected_objects and republishes to /detected_objects_filtered
    after applying the selected scenario transformation.
    """

    def __init__(self) -> None:
        super().__init__("scenario_runner")

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter("scenario",        "normal")
        self.declare_parameter("flicker_prob",    0.30)    # 30% drop
        self.declare_parameter("occlusion_period", 5.0)   # s between occlusions
        self.declare_parameter("occlusion_duration", 2.0) # s each occlusion lasts
        self.declare_parameter("error_offset_x",  0.3)    # normalised units
        self.declare_parameter("error_offset_y",  0.0)
        self.declare_parameter("image_width",  128)
        self.declare_parameter("image_height", 128)

        self._scenario:          str   = self.get_parameter("scenario").value
        self._flicker_prob:      float = self.get_parameter("flicker_prob").value
        self._occ_period:        float = self.get_parameter("occlusion_period").value
        self._occ_duration:      float = self.get_parameter("occlusion_duration").value
        self._offset_x:          float = self.get_parameter("error_offset_x").value
        self._offset_y:          float = self.get_parameter("error_offset_y").value
        self._img_w:             float = self.get_parameter("image_width").value
        self._img_h:             float = self.get_parameter("image_height").value

        if self._scenario not in SCENARIOS:
            self.get_logger().error(
                f"Unknown scenario '{self._scenario}'. "
                f"Valid: {SCENARIOS}")
            raise ValueError(self._scenario)

        # ── Occlusion state ──────────────────────────────────────────────────
        self._occ_start:   float = time.monotonic()
        self._in_occlusion: bool = False

        # ── I/O ─────────────────────────────────────────────────────────────
        self._pub = self.create_publisher(
            Detection2DArray, "/detected_objects_filtered", _reliable_qos())
        self.create_subscription(
            Detection2DArray, "/detected_objects",
            self._detection_cb, _reliable_qos())

        self.get_logger().info(
            f"[ScenarioRunner] Active scenario: '{self._scenario}'")

    # ── Main callback ─────────────────────────────────────────────────────────

    def _detection_cb(self, msg: Detection2DArray) -> None:
        if self._scenario == "normal":
            self._pub.publish(msg)

        elif self._scenario == "flicker":
            if random.random() >= self._flicker_prob:
                self._pub.publish(msg)
            else:
                self._pub.publish(Detection2DArray())   # empty → LOST

        elif self._scenario == "occlusion":
            self._pub.publish(self._apply_occlusion(msg))

        elif self._scenario == "high_error":
            self._pub.publish(self._apply_offset(msg))

    # ── Scenario transforms ───────────────────────────────────────────────────

    def _apply_occlusion(self, msg: Detection2DArray) -> Detection2DArray:
        """Block all detections for occlusion_duration every occlusion_period."""
        now = time.monotonic()
        cycle_t = (now - self._occ_start) % self._occ_period

        if cycle_t < self._occ_duration:
            if not self._in_occlusion:
                self._in_occlusion = True
                self.get_logger().info("[SCENARIO] Occlusion started")
            return Detection2DArray()   # empty
        else:
            if self._in_occlusion:
                self._in_occlusion = False
                self.get_logger().info("[SCENARIO] Occlusion ended")
            return msg

    def _apply_offset(self, msg: Detection2DArray) -> Detection2DArray:
        """
        Shift every bbox centre by (offset_x, offset_y) pixels.
        offset is specified in normalised units [-1,1], converted to pixels here.
        """
        if not msg.detections:
            return msg

        import copy
        out = copy.deepcopy(msg)
        px_offset_x = self._offset_x * (self._img_w / 2.0)
        px_offset_y = self._offset_y * (self._img_h / 2.0)

        for det in out.detections:
            det.bbox.center.position.x += px_offset_x
            det.bbox.center.position.y += px_offset_y

        return out


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScenarioRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
