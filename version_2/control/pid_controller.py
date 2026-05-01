"""
pid_controller.py — Discrete PID Controller
============================================
Week 4 | Reformerz Healthcare Drone Tracking Pipeline

A clean, reusable PID class used by control_node_v2.py.

Features:
  • Integral anti-windup (clamp)
  • Derivative-on-measurement (avoids derivative kick on setpoint change)
  • Output saturation
  • Reset on state transition (prevents integral carry-over)
"""

from __future__ import annotations


class PIDController:
    """
    Discrete PID controller with anti-windup and derivative-on-measurement.

    Args:
        kp: Proportional gain
        ki: Integral gain
        kd: Derivative gain
        output_limit: Symmetric saturation limit (|output| <= output_limit)
        integral_limit: Anti-windup clamp on integral accumulator
    """

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        output_limit: float = 1.0,
        integral_limit: float = 2.0,
    ) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit   = output_limit
        self.integral_limit = integral_limit

        self._integral:    float = 0.0
        self._prev_measure: float = 0.0   # derivative-on-measurement
        self._first_call:   bool  = True

    # ── Public API ─────────────────────────────────────────────────────────────

    def compute(self, error: float, measurement: float, dt: float) -> float:
        """
        Compute PID output.

        Args:
            error:       Setpoint - Measurement  (i.e. desired - actual)
            measurement: Raw measurement value (used for derivative term)
            dt:          Time delta since last call (seconds)

        Returns:
            Control output clamped to [-output_limit, output_limit]
        """
        if dt <= 0.0:
            return 0.0

        # Proportional
        p_term = self.kp * error

        # Integral with anti-windup clamp
        self._integral += error * dt
        self._integral = max(-self.integral_limit,
                             min(self.integral_limit, self._integral))
        i_term = self.ki * self._integral

        # Derivative on measurement (avoids kick on setpoint step)
        if self._first_call:
            d_term = 0.0
            self._first_call = False
        else:
            d_measure = (measurement - self._prev_measure) / dt
            d_term = -self.kd * d_measure   # negative: derivative of error = -d(measurement)
        self._prev_measure = measurement

        output = p_term + i_term + d_term
        return max(-self.output_limit, min(self.output_limit, output))

    def reset(self) -> None:
        """Reset integrator and derivative state (call on mode transitions)."""
        self._integral     = 0.0
        self._prev_measure = 0.0
        self._first_call   = True

    def __repr__(self) -> str:
        return (f"PID(kp={self.kp}, ki={self.ki}, kd={self.kd}, "
                f"lim={self.output_limit})")
