#!/usr/bin/env python3
"""
Week 3 — Logger Node  |  Reformerz Healthcare Robotics Internship
==================================================================
The "Flight Recorder" — logs all control-loop signals to CSV for
post-flight analysis and formal engineering report evidence.

Subscribes:
  /tracking/error                (geometry_msgs/Point)
  /mavros/setpoint_velocity/cmd_vel  (geometry_msgs/TwistStamped)
  /tracking/status               (std_msgs/String)

Output: ~/ros2_ws/logs/session_YYYYMMDD_HHMMSS.csv

CSV columns:
  timestamp_unix  — wall-clock seconds since epoch (float, µs precision)
  error_x         — normalised horizontal error [-1, 1]  (0 = target centred)
  error_y         — normalised vertical   error [-1, 1]  (0 = target centred)
  cmd_vx          — commanded forward velocity  (m/s)
  cmd_vy          — commanded lateral velocity  (m/s)
  cmd_yaw         — commanded yaw rate          (rad/s)
  status          — "TRACKING" | "LOST"

Why this proves "reliably, measurably, safely":
  • RELIABLY  : Every control cycle is timestamped → can reconstruct playback
  • MEASURABLY: error_x/y → zero convergence is mathematically quantifiable
  • SAFELY    : status="LOST" rows prove the watchdog fired correctly
"""

import csv
import os
import time
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point, TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String


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
# Logger Node
# ---------------------------------------------------------------------------
class LoggerNode(Node):
    """
    Synchronises three asynchronous ROS 2 topics into a single CSV row.

    Design notes:
      • Topics arrive at different rates (tracking/error ~10 Hz, cmd_vel
        ~10 Hz, status ~10 Hz).  We store the latest value from each topic
        and write a row whenever ANY of them updates — this gives the highest
        temporal resolution without blocking.
      • A write_timer at 10 Hz guarantees a row is written even during LOST
        phases when no tracking error is published.
      • try-except-finally in main() ensures the CSV is flushed+closed on
        Ctrl-C or any crash.
    """

    def __init__(self) -> None:
        super().__init__("logger_node")

        # ── Output file setup ────────────────────────────────────────────────
        log_dir = Path.home() / "ros2_ws" / "logs"
        log_dir.mkdir(parents=True, exist_ok=True)
        session_name = datetime.now().strftime("session_%Y%m%d_%H%M%S")
        self._csv_path = log_dir / f"{session_name}.csv"

        self._csv_file   = open(self._csv_path, "w", newline="")
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow([
            "timestamp_unix",
            "error_x", "error_y",
            "cmd_vx", "cmd_vy", "cmd_yaw",
            "status",
        ])
        self._csv_file.flush()

        # ── Latest values from each topic ────────────────────────────────────
        self._error_x:  float = 0.0
        self._error_y:  float = 0.0
        self._cmd_vx:   float = 0.0
        self._cmd_vy:   float = 0.0
        self._cmd_yaw:  float = 0.0
        self._status:   str   = "LOST"
        self._row_count: int  = 0

        # ── Subscriptions ────────────────────────────────────────────────────
        self.create_subscription(
            Point, "/tracking/error",
            self._error_callback, _reliable_qos()
        )
        self.create_subscription(
            TwistStamped, "/mavros/setpoint_velocity/cmd_vel",
            self._cmd_vel_callback, _reliable_qos()
        )
        self.create_subscription(
            String, "/tracking/status",
            self._status_callback, _reliable_qos()
        )

        # ── Write timer: 10 Hz — guarantees rows even when LOST ──────────────
        self.create_timer(0.1, self._write_row)

        self.get_logger().info(
            f"LoggerNode started — writing to {self._csv_path}"
        )

    # ── Topic callbacks (update latest values only) ──────────────────────────

    def _error_callback(self, msg: Point) -> None:
        self._error_x = msg.x
        self._error_y = msg.y

    def _cmd_vel_callback(self, msg: TwistStamped) -> None:
        self._cmd_vx  = msg.twist.linear.x
        self._cmd_vy  = msg.twist.linear.y
        self._cmd_yaw = msg.twist.angular.z

    def _status_callback(self, msg: String) -> None:
        self._status = msg.data

    # ── Write one row to CSV ─────────────────────────────────────────────────

    def _write_row(self) -> None:
        ts = time.time()   # Unix wall-clock with µs precision
        self._csv_writer.writerow([
            f"{ts:.6f}",
            f"{self._error_x:.6f}",
            f"{self._error_y:.6f}",
            f"{self._cmd_vx:.6f}",
            f"{self._cmd_vy:.6f}",
            f"{self._cmd_yaw:.6f}",
            self._status,
        ])
        self._row_count += 1

        # Flush every 100 rows (~10 s) so data is safe if process is killed
        if self._row_count % 100 == 0:
            self._csv_file.flush()
            self.get_logger().info(
                f"[Logger] {self._row_count} rows written → {self._csv_path.name}"
            )

    # ── Clean shutdown ───────────────────────────────────────────────────────

    def close(self) -> None:
        """Flush and close the CSV.  Called from finally block in main()."""
        try:
            self._csv_file.flush()
            self._csv_file.close()
            self.get_logger().info(
                f"[Logger] Session saved: {self._csv_path}  "
                f"({self._row_count} rows)"
            )
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main(args=None) -> None:
    rclpy.init(args=args)
    node = LoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # ALWAYS flush+close the CSV — even on Ctrl-C or crash
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
