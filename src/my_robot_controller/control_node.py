#!/usr/bin/env python3
"""
Week 3 — Control Node ("Shiva-Ready" Controller)  |  Reformerz Healthcare
==========================================================================
Subscribes : /tracking/error              (geometry_msgs/Point)
             /tracking/status             (std_msgs/String)
             /mavros/state                (mavros_msgs/State)
Publishes  : /mavros/setpoint_velocity/cmd_vel  (geometry_msgs/TwistStamped)
Services   : /mavros/cmd/arming           (mavros_msgs/srv/CommandBool)
             /mavros/set_mode             (mavros_msgs/srv/SetMode)

Offboard Control Sequence (PX4 requirement):
  ┌─────────────────────────────────────────────────────────────────────┐
  │  Phase 1 — HEARTBEAT (≥20 setpoints at ≥2 Hz before mode switch)   │
  │  Phase 2 — REQUEST OFFBOARD mode                                    │
  │  Phase 3 — ARM the drone                                            │
  │  Phase 4 — TAKEOFF  (climb at +0.5 m/s for TAKEOFF_SECONDS)        │
  │  Phase 5 — TRACK (P-controller on error.x / error.y)               │
  └─────────────────────────────────────────────────────────────────────┘

Safety rules:
  • Control loop runs on a TIMER at CONTROL_HZ — never purely event-driven.
    PX4 exits Offboard mode if setpoints stop for > 500 ms.
  • If /tracking/status == LOST: all velocities → 0.0 (drone hovers in place).
  • If FCU disconnects: all velocities → 0.0, re-enter heartbeat phase.

P-Controller mapping (error is normalised to [-1, 1]):
  error.x (right +)  →  yaw_rate  (turn to face target)
  error.y (down  +)  →  vx        (fly toward target)
  Altitude hold is delegated to PX4's own altitude controller.
"""

import time

import rclpy
from geometry_msgs.msg import Point, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.node import Node
from rclpy.qos import (
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import String

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
CONTROL_HZ      = 10.0          # Hz — control loop rate (must be > 2 Hz)
HEARTBEAT_COUNT = 25            # setpoints sent before requesting OFFBOARD
                                # (PX4 needs >20; 25 gives safety margin)
TAKEOFF_SECONDS = 4.0           # seconds to climb before switching to tracking
TAKEOFF_VZ      = 0.5           # climb speed (m/s)

# FSM states
PHASE_HEARTBEAT = "HEARTBEAT"
PHASE_OFFBOARD  = "REQ_OFFBOARD"
PHASE_ARM       = "REQ_ARM"
PHASE_TAKEOFF   = "TAKEOFF"
PHASE_TRACKING  = "TRACKING"
PHASE_SAFE      = "SAFE_HOVER"   # FCU disconnected


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
# Control Node
# ---------------------------------------------------------------------------
class ControlNode(Node):
    """
    Proportional drone-tracking controller with PX4 Offboard sequence.

    Parameters (tunable at launch):
      kp_yaw      — P-gain: normalised error.x  → yaw_rate (rad/s) default 0.5
      kp_vx       — P-gain: normalised error.y  → forward speed (m/s) default 0.4
      max_yaw     — saturation limit for yaw_rate  (rad/s)   default 0.6
      max_vx      — saturation limit for forward speed (m/s)  default 0.5
      deadband    — errors below this magnitude are zeroed     default 0.05
      takeoff_alt — altitude to hold during tracking (m)       default 2.0
    """

    def __init__(self) -> None:
        super().__init__("control_node")

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter("kp_yaw",      0.5)
        self.declare_parameter("kp_vx",       0.4)
        self.declare_parameter("max_yaw",     0.6)
        self.declare_parameter("max_vx",      0.5)
        self.declare_parameter("deadband",    0.05)
        self.declare_parameter("takeoff_alt", 2.0)

        # ── PID Gains (Week 4 Upgrade) ──────────────────────────────────────
        self.declare_parameter("ki_yaw",      0.05)
        self.declare_parameter("ki_vx",       0.05)
        self.declare_parameter("i_limit",     0.2)

        # ── Stress Parameters ───────────────────────────────────────────────
        self.declare_parameter("esc_delay_ms",   0)      # 50-100ms buffer
        self.declare_parameter("battery_sag",    False)  # grad scale 1.0 -> 0.8
        self.declare_parameter("bias_vx",        0.0)    # e.g. 0.05 m/s

        self._kp_yaw:      float = self.get_parameter("kp_yaw").value
        self._kp_vx:       float = self.get_parameter("kp_vx").value
        self._max_yaw:     float = self.get_parameter("max_yaw").value
        self._max_vx:      float = self.get_parameter("max_vx").value
        self._deadband:    float = self.get_parameter("deadband").value
        self._takeoff_alt: float = self.get_parameter("takeoff_alt").value

        # PID State
        self._ki_yaw:      float = self.get_parameter("ki_yaw").value
        self._ki_vx:       float = self.get_parameter("ki_vx").value
        self._i_limit:     float = self.get_parameter("i_limit").value
        self._integral_yaw: float = 0.0
        self._integral_vx:  float = 0.0
        self._last_control_t: float = self.get_clock().now().nanoseconds / 1e9

        # Stress State
        self._cmd_buffer = [] # for ESC delay
        self._battery_scale = 1.0

        # Low-Pass Filter State (Week 4 Validation requirement)
        self._lpf_alpha = 0.3 # Smoothing factor (0.3 = 70% old, 30% new)
        self._filtered_err_x = 0.0
        self._filtered_err_y = 0.0

        # ── State ────────────────────────────────────────────────────────────
        self._phase:          str   = PHASE_HEARTBEAT
        self._heartbeat_sent: int   = 0
        self._fcu_state:      State = State()        # last known FCU state
        self._error:          Point = Point()        # last known tracking error
        self._target_status:  str   = "LOST"
        self._last_error_t:   float = 0.0
        self._takeoff_cycles: int   = 0              # counts 10Hz ticks during takeoff

        # ── Publishers ───────────────────────────────────────────────────────
        self._cmd_pub = self.create_publisher(
            TwistStamped,
            "/mavros/setpoint_velocity/cmd_vel",
            _reliable_qos(depth=1),
        )

        # ── Subscribers ──────────────────────────────────────────────────────
        self._err_sub = self.create_subscription(
            Point, "/tracking/error", self._error_callback, _reliable_qos()
        )
        self._status_sub = self.create_subscription(
            String, "/tracking/status", self._status_callback, _reliable_qos()
        )
        self._state_sub = self.create_subscription(
            State, "/mavros/state", self._state_callback, _reliable_qos()
        )

        # ── Service clients ──────────────────────────────────────────────────
        self._arming_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self._mode_client   = self.create_client(SetMode,     "/mavros/set_mode")

        # ── Control loop timer ───────────────────────────────────────────────
        self._timer = self.create_timer(1.0 / CONTROL_HZ, self._control_loop)

        self.get_logger().info(
            f"ControlNode Week 4 (PID) ready — "
            f"kp_yaw={self._kp_yaw} ki_yaw={self._ki_yaw} "
            f"kp_vx={self._kp_vx} ki_vx={self._ki_vx} "
            f"CONTROL_HZ={CONTROL_HZ}\n"
            f"Phase: {self._phase}"
        )

    # ── Subscriber callbacks (store latest values only) ──────────────────────

    def _error_callback(self, msg: Point) -> None:
        self._error = msg
        self._last_error_t = time.monotonic()

    def _status_callback(self, msg: String) -> None:
        self._target_status = msg.data

    def _state_callback(self, msg: State) -> None:
        prev_connected = self._fcu_state.connected
        self._fcu_state = msg

        if not msg.connected and prev_connected:
            self.get_logger().error(
                "[FCU] Lost connection to MAVROS — entering SAFE_HOVER"
            )
            self._phase = PHASE_SAFE

        if msg.connected and not prev_connected:
            self.get_logger().info(
                "[FCU] Connection (re-)established — restarting heartbeat"
            )
            self._phase          = PHASE_HEARTBEAT
            self._heartbeat_sent = 0

    # ── Main control loop (timer-driven) ────────────────────────────────────

    def _control_loop(self) -> None:
        """
        Called at CONTROL_HZ regardless of incoming detections.
        This guarantees PX4 always receives setpoints and stays in Offboard mode.
        """
        if self._phase == PHASE_HEARTBEAT:
            self._do_heartbeat()

        elif self._phase == PHASE_OFFBOARD:
            self._request_offboard()

        elif self._phase == PHASE_ARM:
            self._request_arm()

        elif self._phase == PHASE_TAKEOFF:
            self._do_takeoff()

        elif self._phase == PHASE_TRACKING:
            self._do_tracking()

        elif self._phase == PHASE_SAFE:
            # FCU disconnected — publish zero and wait for reconnect
            self._publish_velocity(0.0, 0.0, 0.0)

    # ── Phase handlers ───────────────────────────────────────────────────────

    def _do_heartbeat(self) -> None:
        """Phase 1: flood zero-velocity setpoints so PX4 accepts OFFBOARD."""
        self._publish_velocity(0.0, 0.0, 0.0)
        self._heartbeat_sent += 1

        if self._heartbeat_sent % 5 == 0:
            self.get_logger().info(
                f"[HEARTBEAT] {self._heartbeat_sent}/{HEARTBEAT_COUNT} setpoints sent…"
            )

        if self._heartbeat_sent >= HEARTBEAT_COUNT:
            self.get_logger().info(
                f"[HEARTBEAT] Done ({self._heartbeat_sent} setpoints). "
                "→ Requesting OFFBOARD mode…"
            )
            self._phase = PHASE_OFFBOARD

    def _request_offboard(self) -> None:
        """Phase 2: call /mavros/set_mode → OFFBOARD, then move to ARM phase."""
        # Keep publishing setpoints while waiting for service
        self._publish_velocity(0.0, 0.0, 0.0)

        if not self._mode_client.service_is_ready():
            self.get_logger().warning("[OFFBOARD] Waiting for /mavros/set_mode service…")
            return

        req = SetMode.Request()
        req.custom_mode = "OFFBOARD"
        future = self._mode_client.call_async(req)
        future.add_done_callback(self._offboard_response)
        # Move to ARM immediately; the callback confirms success
        self._phase = PHASE_ARM

    def _offboard_response(self, future) -> None:
        try:
            result = future.result()
            if result.mode_sent:
                self.get_logger().info("[OFFBOARD] Mode accepted by FCU ✓")
            else:
                self.get_logger().error(
                    "[OFFBOARD] FCU rejected mode — restarting heartbeat"
                )
                self._phase          = PHASE_HEARTBEAT
                self._heartbeat_sent = 0
        except Exception as exc:
            self.get_logger().error(f"[OFFBOARD] Service call failed: {exc}")

    def _request_arm(self) -> None:
        """Phase 3: arm the drone, then move to TAKEOFF phase."""
        self._publish_velocity(0.0, 0.0, 0.0)

        if self._fcu_state.armed:
            self.get_logger().info(
                f"[ARM] Drone is ARMED ✓  → Taking off at {TAKEOFF_VZ} m/s "
                f"for {TAKEOFF_SECONDS}s …"
            )
            self._takeoff_cycles = 0
            self._phase = PHASE_TAKEOFF
            return

        if not self._arming_client.service_is_ready():
            self.get_logger().warning("[ARM] Waiting for /mavros/cmd/arming service…")
            return

        req = CommandBool.Request()
        req.value = True
        future = self._arming_client.call_async(req)
        future.add_done_callback(self._arm_response)

    def _arm_response(self, future) -> None:
        try:
            result = future.result()
            if result.success:
                self.get_logger().info("[ARM] Arm command accepted ✓")
                # FCU state callback will confirm and advance phase
            else:
                self.get_logger().error(
                    "[ARM] Arm command rejected — retrying next cycle"
                )
        except Exception as exc:
            self.get_logger().error(f"[ARM] Service call failed: {exc}")

    def _do_takeoff(self) -> None:
        """Phase 4: climb at TAKEOFF_VZ for TAKEOFF_SECONDS, then switch to tracking."""
        if not self._fcu_state.armed:
            self.get_logger().error("[TAKEOFF] Disarmed during takeoff — restarting")
            self._phase          = PHASE_HEARTBEAT
            self._heartbeat_sent = 0
            return

        target_cycles = int(TAKEOFF_SECONDS * CONTROL_HZ)
        self._takeoff_cycles += 1

        self._publish_velocity(0.0, 0.0, 0.0, vz=TAKEOFF_VZ)

        if self._takeoff_cycles % 10 == 0:
            remaining = (target_cycles - self._takeoff_cycles) / CONTROL_HZ
            self.get_logger().info(
                f"[TAKEOFF] Climbing… {remaining:.0f}s remaining"
            )

        if self._takeoff_cycles >= target_cycles:
            self.get_logger().info(
                "[TAKEOFF] Altitude reached ✓  → Switching to TRACKING"
            )
            self._phase = PHASE_TRACKING

    def _do_tracking(self) -> None:
        """Phase 5: PID-controller → velocity setpoints."""

        # ── Safety: check FCU is still in OFFBOARD and armed ────────────────
        if not self._fcu_state.armed:
            self.get_logger().error(
                "[TRACKING] Drone disarmed — restarting full sequence"
            )
            self._phase          = PHASE_HEARTBEAT
            self._heartbeat_sent = 0
            return

        if self._fcu_state.mode != "OFFBOARD":
            self.get_logger().warning(
                f"[TRACKING] FCU mode changed to '{self._fcu_state.mode}' "
                "— publishing zero to reclaim Offboard"
            )
            self._publish_velocity(0.0, 0.0, 0.0)
            return

        # ── Safety: if target LOST, hover in place ───────────────────────────
        if self._target_status == "LOST":
            self._publish_velocity(0.0, 0.0, 0.0)
            return

        # ── PID Controller ───────────────────────────────────────────────────
        err_x = self._error.x   # normalised [-1, 1], right is positive
        err_y = self._error.y   # normalised [-1, 1], down  is positive

        # Calculate dt for integral
        now = self.get_clock().now().nanoseconds / 1e9
        dt = now - self._last_control_t
        self._last_control_t = now

        # Apply deadband — ignore tiny errors (prevents jitter)
        if abs(err_x) < self._deadband:
            err_x = 0.0
        if abs(err_y) < self._deadband:
            err_y = 0.0

        # Apply Low-Pass Filter (EMA) to mitigate IMU Noise oscillations
        self._filtered_err_x = (self._lpf_alpha * err_x) + (1.0 - self._lpf_alpha) * self._filtered_err_x
        self._filtered_err_y = (self._lpf_alpha * err_y) + (1.0 - self._lpf_alpha) * self._filtered_err_y
        
        # Use filtered errors for PID calculation
        err_x = self._filtered_err_x
        err_y = self._filtered_err_y

        # Update Integrals (with safety: reset if target LOST)
        if self._target_status == "TRACKING":
            self._integral_yaw += err_x * dt
            self._integral_vx  += err_y * dt
            # Anti-windup clamping
            self._integral_yaw = self._clamp(self._integral_yaw, self._i_limit)
            self._integral_vx  = self._clamp(self._integral_vx, self._i_limit)
        else:
            self._integral_yaw = 0.0
            self._integral_vx  = 0.0

        # PID Formula
        yaw_rate = (self._kp_yaw * err_x) + (self._ki_yaw * self._integral_yaw)
        vx       = (self._kp_vx  * err_y) + (self._ki_vx  * self._integral_vx)

        # Apply Saturation Limits
        yaw_rate = float(self._clamp(yaw_rate, self._max_yaw))
        vx       = float(self._clamp(vx, self._max_vx))

        # ── Stress Injection ────────────────────────────────────────────────
        
        # 1. Mechanical Bias (constant 0.05 m/s velocity offset)
        bias = self.get_parameter("bias_vx").value
        vx += bias

        # 2. Battery Sag (gradually scale down thrust effectiveness)
        if self.get_parameter("battery_sag").value:
            # Decay from 1.0 to 0.8 over ~60 seconds of tracking
            self._battery_scale = max(0.8, self._battery_scale - 0.002 * dt)
            vx *= self._battery_scale

        # 3. ESC Delay (command buffer)
        delay_ms = self.get_parameter("esc_delay_ms").value
        if delay_ms > 0:
            # Add current command to buffer
            self._cmd_buffer.append((now + (delay_ms / 1000.0), vx, yaw_rate))
            
            # Check if oldest command is ready to be sent
            if self._cmd_buffer[0][0] <= now:
                _, vx, yaw_rate = self._cmd_buffer.pop(0)
            else:
                # Still waiting for delay, send zero or last? 
                # To be safe, we don't publish yet or publish hover.
                # Actually, if we want to simulate delay, we should send the "delayed" command.
                # If buffer isn't ready, just hover.
                return 

        self._publish_velocity(vx, 0.0, yaw_rate)

        self.get_logger().debug(
            f"[CTRL] err=({err_x:+.3f},{err_y:+.3f})  "
            f"vx={vx:+.3f} m/s  yaw_rate={yaw_rate:+.3f} rad/s"
        )

    # ── Helper methods ───────────────────────────────────────────────────────

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        return max(-limit, min(limit, value))

    def _publish_velocity(self, vx: float, vy: float, yaw_rate: float,
                          vz: float = 0.0) -> None:
        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x  = vx
        msg.twist.linear.y  = vy
        msg.twist.linear.z  = vz
        msg.twist.angular.z = yaw_rate
        self._cmd_pub.publish(msg)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Disarm-safe shutdown: publish zero before exiting
        node.get_logger().info("Shutting down — sending zero velocity")
        node._publish_velocity(0.0, 0.0, 0.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
