#!/usr/bin/env python3
"""
control_node_v2.py — Week 4 PID Drone Tracking Controller
==========================================================
Reformerz Healthcare | Drone Tracking Pipeline v2

Upgrades from Week 3 P-controller:
  • Full PID on both yaw and forward velocity axes
  • Integral anti-windup (configurable clamp)
  • Derivative-on-measurement (no derivative kick)
  • 4-state FSM: TRACK / SEARCH / HOLD / FAILSAFE
  • Publishes /system_state for logger_v2 and monitoring

Subscribes:
  /tracking/error                    (geometry_msgs/Point)
  /tracking/status                   (std_msgs/String)
  /mavros/state                      (mavros_msgs/State)
Publishes:
  /mavros/setpoint_velocity/cmd_vel  (geometry_msgs/TwistStamped)
  /system_state                      (std_msgs/String)
Services:
  /mavros/cmd/arming                 (mavros_msgs/srv/CommandBool)
  /mavros/set_mode                   (mavros_msgs/srv/SetMode)

PID gains (tunable at launch):
  kp_yaw=0.5  ki_yaw=0.05  kd_yaw=0.08
  kp_vx=0.4   ki_vx=0.04   kd_vx=0.06

4-State FSM:
  TRACK     — OFFBOARD + armed + TRACKING  (PID active)
  SEARCH    — OFFBOARD + armed + LOST < search_timeout (hover, reset integrators)
  HOLD      — OFFBOARD + armed + LOST >= search_timeout (hover, log as HOLD)
  FAILSAFE  — FCU disconnected or unexpectedly disarmed
"""

import time

import rclpy
from geometry_msgs.msg import Point, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

from .pid_controller import PIDController

# ── Constants ─────────────────────────────────────────────────────────────────
CONTROL_HZ      = 10.0
HEARTBEAT_COUNT = 25
TAKEOFF_SECONDS = 4.0
TAKEOFF_VZ      = 0.5

PHASE_HEARTBEAT = "HEARTBEAT"
PHASE_OFFBOARD  = "REQ_OFFBOARD"
PHASE_ARM       = "REQ_ARM"
PHASE_TAKEOFF   = "TAKEOFF"
PHASE_TRACKING  = "TRACKING"   # internal FSM phase
PHASE_SAFE      = "SAFE_HOVER"

# 4-state output for logger
STATE_TRACK    = "TRACK"
STATE_SEARCH   = "SEARCH"
STATE_HOLD     = "HOLD"
STATE_FAILSAFE = "FAILSAFE"


def _reliable_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
    )


class ControlNodeV2(Node):

    def __init__(self) -> None:
        super().__init__("control_node_v2")

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter("kp_yaw",         0.5)
        self.declare_parameter("ki_yaw",         0.05)
        self.declare_parameter("kd_yaw",         0.08)
        self.declare_parameter("kp_vx",          0.4)
        self.declare_parameter("ki_vx",          0.04)
        self.declare_parameter("kd_vx",          0.06)
        self.declare_parameter("max_yaw",        0.6)
        self.declare_parameter("max_vx",         0.5)
        self.declare_parameter("deadband",       0.05)
        self.declare_parameter("integral_limit", 2.0)
        self.declare_parameter("search_timeout", 1.5)   # s before HOLD

        p = lambda name: self.get_parameter(name).value

        self._search_timeout: float = p("search_timeout")
        self._deadband:       float = p("deadband")

        self._pid_yaw = PIDController(
            kp=p("kp_yaw"), ki=p("ki_yaw"), kd=p("kd_yaw"),
            output_limit=p("max_yaw"),
            integral_limit=p("integral_limit"),
        )
        self._pid_vx = PIDController(
            kp=p("kp_vx"), ki=p("ki_vx"), kd=p("kd_vx"),
            output_limit=p("max_vx"),
            integral_limit=p("integral_limit"),
        )

        # ── State ────────────────────────────────────────────────────────────
        self._phase:          str   = PHASE_HEARTBEAT
        self._heartbeat_sent: int   = 0
        self._fcu_state:      State = State()
        self._error:          Point = Point()
        self._target_status:  str   = "LOST"
        self._lost_since:     float = 0.0
        self._last_t:         float = time.monotonic()
        self._takeoff_cycles: int   = 0
        self._system_state:   str   = STATE_FAILSAFE

        # ── Publishers ───────────────────────────────────────────────────────
        self._cmd_pub = self.create_publisher(
            TwistStamped, "/mavros/setpoint_velocity/cmd_vel",
            _reliable_qos(depth=1))
        self._state_pub = self.create_publisher(
            String, "/system_state", _reliable_qos())

        # ── Subscribers ──────────────────────────────────────────────────────
        self.create_subscription(
            Point, "/tracking/error", self._error_cb, _reliable_qos())
        self.create_subscription(
            String, "/tracking/status", self._status_cb, _reliable_qos())
        self.create_subscription(
            State, "/mavros/state", self._fcu_state_cb, _reliable_qos())

        # ── Services ─────────────────────────────────────────────────────────
        self._arming_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self._mode_client   = self.create_client(SetMode,     "/mavros/set_mode")

        self.create_timer(1.0 / CONTROL_HZ, self._control_loop)

        self.get_logger().info(
            f"ControlNodeV2 ready  "
            f"PID_yaw={self._pid_yaw}  PID_vx={self._pid_vx}"
        )

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _error_cb(self, msg: Point) -> None:
        self._error = msg

    def _status_cb(self, msg: String) -> None:
        prev = self._target_status
        self._target_status = msg.data
        if msg.data == "LOST" and prev == "TRACKING":
            self._lost_since = time.monotonic()
            # Reset PID integrators on target loss to prevent windup
            self._pid_yaw.reset()
            self._pid_vx.reset()

    def _fcu_state_cb(self, msg: State) -> None:
        prev_conn = self._fcu_state.connected
        self._fcu_state = msg

        if not msg.connected and prev_conn:
            self.get_logger().error("[FCU] Disconnected — FAILSAFE")
            self._phase = PHASE_SAFE
            self._set_system_state(STATE_FAILSAFE)

        if msg.connected and not prev_conn:
            self.get_logger().info("[FCU] Reconnected — restarting heartbeat")
            self._phase          = PHASE_HEARTBEAT
            self._heartbeat_sent = 0

    # ── Control loop ─────────────────────────────────────────────────────────

    def _control_loop(self) -> None:
        now = time.monotonic()
        dt  = now - self._last_t
        self._last_t = now

        if self._phase == PHASE_HEARTBEAT:
            self._do_heartbeat()
        elif self._phase == PHASE_OFFBOARD:
            self._request_offboard()
        elif self._phase == PHASE_ARM:
            self._request_arm()
        elif self._phase == PHASE_TAKEOFF:
            self._do_takeoff()
        elif self._phase == PHASE_TRACKING:
            self._do_tracking(dt)
        elif self._phase == PHASE_SAFE:
            self._publish_vel(0.0, 0.0, 0.0)
            self._set_system_state(STATE_FAILSAFE)

    # ── Phase handlers ────────────────────────────────────────────────────────

    def _do_heartbeat(self) -> None:
        self._publish_vel(0.0, 0.0, 0.0)
        self._heartbeat_sent += 1
        if self._heartbeat_sent >= HEARTBEAT_COUNT:
            self.get_logger().info("[HEARTBEAT] Done -> requesting OFFBOARD")
            self._phase = PHASE_OFFBOARD

    def _request_offboard(self) -> None:
        self._publish_vel(0.0, 0.0, 0.0)
        if not self._mode_client.service_is_ready():
            return
        req = SetMode.Request()
        req.custom_mode = "OFFBOARD"
        future = self._mode_client.call_async(req)
        future.add_done_callback(self._offboard_response)
        self._phase = PHASE_ARM

    def _offboard_response(self, future) -> None:
        try:
            if not future.result().mode_sent:
                self.get_logger().error("[OFFBOARD] Rejected — restarting")
                self._phase          = PHASE_HEARTBEAT
                self._heartbeat_sent = 0
        except Exception as exc:
            self.get_logger().error(f"[OFFBOARD] {exc}")

    def _request_arm(self) -> None:
        self._publish_vel(0.0, 0.0, 0.0)
        if self._fcu_state.armed:
            self.get_logger().info("[ARM] Armed -> TAKEOFF")
            self._takeoff_cycles = 0
            self._phase = PHASE_TAKEOFF
            return
        if not self._arming_client.service_is_ready():
            return
        req = CommandBool.Request()
        req.value = True
        self._arming_client.call_async(req).add_done_callback(self._arm_response)

    def _arm_response(self, future) -> None:
        try:
            if not future.result().success:
                self.get_logger().error("[ARM] Rejected — retrying")
        except Exception as exc:
            self.get_logger().error(f"[ARM] {exc}")

    def _do_takeoff(self) -> None:
        if not self._fcu_state.armed:
            self.get_logger().error("[TAKEOFF] Disarmed — restarting")
            self._phase = PHASE_HEARTBEAT
            self._heartbeat_sent = 0
            return

        self._takeoff_cycles += 1
        self._publish_vel(0.0, 0.0, 0.0, vz=TAKEOFF_VZ)

        if self._takeoff_cycles >= int(TAKEOFF_SECONDS * CONTROL_HZ):
            self.get_logger().info("[TAKEOFF] Done -> TRACKING")
            self._phase = PHASE_TRACKING
            self._pid_yaw.reset()
            self._pid_vx.reset()
            self._lost_since = 0.0

    def _do_tracking(self, dt: float) -> None:
        if not self._fcu_state.armed:
            self.get_logger().error("[TRACKING] Disarmed — restarting")
            self._phase = PHASE_HEARTBEAT
            self._heartbeat_sent = 0
            self._set_system_state(STATE_FAILSAFE)
            return

        if self._fcu_state.mode != "OFFBOARD":
            self.get_logger().warning(
                f"[TRACKING] FCU mode={self._fcu_state.mode} — reclaiming")
            self._publish_vel(0.0, 0.0, 0.0)
            return

        # ── Classify system state ────────────────────────────────────────────
        if self._target_status == "TRACKING":
            self._set_system_state(STATE_TRACK)
        else:
            lost_duration = time.monotonic() - self._lost_since
            if lost_duration < self._search_timeout:
                self._set_system_state(STATE_SEARCH)
            else:
                self._set_system_state(STATE_HOLD)
            self._publish_vel(0.0, 0.0, 0.0)
            return

        # ── PID tracking ─────────────────────────────────────────────────────
        err_x = self._error.x
        err_y = self._error.y

        if abs(err_x) < self._deadband:
            err_x = 0.0
        if abs(err_y) < self._deadband:
            err_y = 0.0

        yaw_rate = self._pid_yaw.compute(err_x, self._error.x, dt)
        vx       = self._pid_vx.compute(err_y,  self._error.y, dt)

        self._publish_vel(vx, 0.0, yaw_rate)

        self.get_logger().debug(
            f"[PID] err=({err_x:+.3f},{err_y:+.3f})  "
            f"vx={vx:+.3f}  yaw={yaw_rate:+.3f}")

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _set_system_state(self, state: str) -> None:
        if state != self._system_state:
            self.get_logger().info(f"[STATE] {self._system_state} -> {state}")
        self._system_state = state
        msg = String()
        msg.data = state
        self._state_pub.publish(msg)

    def _publish_vel(self, vx: float, vy: float, yaw_rate: float,
                     vz: float = 0.0) -> None:
        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x  = vx
        msg.twist.linear.y  = vy
        msg.twist.linear.z  = vz
        msg.twist.angular.z = yaw_rate
        self._cmd_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControlNodeV2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutdown — sending zero velocity")
        node._publish_vel(0.0, 0.0, 0.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
