#!/usr/bin/env python3
"""
logger_v2.py — Week 4 Flight Data Recorder
============================================
Reformerz Healthcare | Drone Tracking Pipeline v2

Subscribes:
  /tracking/error                        (geometry_msgs/Point)
  /mavros/setpoint_velocity/cmd_vel      (geometry_msgs/TwistStamped)
  /tracking/status                       (std_msgs/String)  TRACKING | LOST
  /system_state                          (std_msgs/String)  TRACK | SEARCH | HOLD | FAILSAFE

Output CSV columns:
  timestamp_unix   wall-clock (float, µs precision)
  elapsed_s        seconds since session start
  error_x          normalised horizontal error [-1, 1]
  error_y          normalised vertical error   [-1, 1]
  error_mag        Euclidean |error| = sqrt(ex²+ey²)
  vx               commanded forward velocity (m/s)
  vy               commanded lateral velocity (m/s)
  yaw_rate         commanded yaw rate (rad/s)
  detection_status TRACKING | LOST
  system_state     TRACK | SEARCH | HOLD | FAILSAFE
  controller       PID_V2

Differences from Week 3 logger:
  • Adds error_mag column (pre-computed for metrics.py)
  • Adds system_state column (4-level FSM vs 2-level)
  • Adds controller tag column (v2 vs baseline comparison)
  • Logs at 10 Hz (same) but also flushes on FAILSAFE state
"""

import csv
import time
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point, TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String


def _reliable_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
    )


class LoggerV2(Node):
    """
    Week 4 flight recorder with extended state logging.

    Writes one row every 100 ms (10 Hz).  Flushes immediately on FAILSAFE
    so data is never lost during an FCU disconnect event.
    """

    CONTROLLER_TAG = "PID_V2"

    def __init__(self) -> None:
        super().__init__("logger_v2")

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter("log_dir",
                               str(Path.home() / "ros2_ws" / "logs"))
        self.declare_parameter("session_prefix", "session_v2")
        self.declare_parameter("flush_interval_rows", 100)

        log_dir    = Path(self.get_parameter("log_dir").value)
        prefix     = self.get_parameter("session_prefix").value
        self._flush_every: int = self.get_parameter("flush_interval_rows").value

        log_dir.mkdir(parents=True, exist_ok=True)
        session_name      = datetime.now().strftime(f"{prefix}_%Y%m%d_%H%M%S")
        self._csv_path    = log_dir / f"{session_name}.csv"
        self._session_tag = session_name
        self._start_time  = time.time()

        self._csv_file   = open(self._csv_path, "w", newline="")
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow([
            "timestamp_unix", "elapsed_s",
            "error_x", "error_y", "error_mag",
            "vx", "vy", "yaw_rate",
            "detection_status", "system_state",
            "controller",
        ])
        self._csv_file.flush()

        # ── State ────────────────────────────────────────────────────────────
        self._error_x:  float = 0.0
        self._error_y:  float = 0.0
        self._vx:       float = 0.0
        self._vy:       float = 0.0
        self._yaw_rate: float = 0.0
        self._det_status:    str = "LOST"
        self._system_state:  str = "HOLD"
        self._row_count:     int = 0

        # ── Subscriptions ────────────────────────────────────────────────────
        self.create_subscription(
            Point, "/tracking/error", self._error_cb, _reliable_qos())
        self.create_subscription(
            TwistStamped, "/mavros/setpoint_velocity/cmd_vel",
            self._cmd_vel_cb, _reliable_qos())
        self.create_subscription(
            String, "/tracking/status", self._det_status_cb, _reliable_qos())
        self.create_subscription(
            String, "/system_state", self._sys_state_cb, _reliable_qos())

        self.create_timer(0.1, self._write_row)

        self.get_logger().info(
            f"[LoggerV2] Recording to {self._csv_path}")

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _error_cb(self, msg: Point) -> None:
        self._error_x = msg.x
        self._error_y = msg.y

    def _cmd_vel_cb(self, msg: TwistStamped) -> None:
        self._vx       = msg.twist.linear.x
        self._vy       = msg.twist.linear.y
        self._yaw_rate = msg.twist.angular.z

    def _det_status_cb(self, msg: String) -> None:
        self._det_status = msg.data

    def _sys_state_cb(self, msg: String) -> None:
        prev = self._system_state
        self._system_state = msg.data
        if msg.data == "FAILSAFE" and prev != "FAILSAFE":
            self._csv_file.flush()   # flush immediately on failsafe

    # ── Write ────────────────────────────────────────────────────────────────

    def _write_row(self) -> None:
        ts      = time.time()
        elapsed = ts - self._start_time
        mag     = (self._error_x ** 2 + self._error_y ** 2) ** 0.5

        self._csv_writer.writerow([
            f"{ts:.6f}",
            f"{elapsed:.3f}",
            f"{self._error_x:.6f}",
            f"{self._error_y:.6f}",
            f"{mag:.6f}",
            f"{self._vx:.6f}",
            f"{self._vy:.6f}",
            f"{self._yaw_rate:.6f}",
            self._det_status,
            self._system_state,
            self.CONTROLLER_TAG,
        ])
        self._row_count += 1

        if self._row_count % self._flush_every == 0:
            self._csv_file.flush()
            self.get_logger().info(
                f"[LoggerV2] {self._row_count} rows  |  "
                f"state={self._system_state}  elapsed={elapsed:.1f}s")

    def close(self) -> None:
        try:
            self._csv_file.flush()
            self._csv_file.close()
            self.get_logger().info(
                f"[LoggerV2] Saved {self._row_count} rows -> {self._csv_path}")
        except Exception:
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LoggerV2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
