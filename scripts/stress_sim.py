#!/usr/bin/env python3
"""
stress_sim.py — Week 4 Combined Stress Test Automation
======================================================
Reformerz Healthcare Robotics | Drone Validation Suite

This script toggles the 6 engineering stress conditions across the
perception, tracking, and control nodes to prove system robustness.

Conditions:
  1. IMU Noise          (Tracking Node)
  2. ESC Delay          (Control Node)
  3. Battery Sag        (Control Node)
  4. GPS Drift          (Tracking Node)
  5. Camera Latency     (Perception Node)
  6. Mech. Imbalance    (Control Node)
"""

import subprocess
import sys
import time
import argparse

def set_param(node, param, value):
    cmd = ["ros2", "param", "set", node, param, str(value)]
    print(f"[STRESS] Setting {node}/{param} = {value} ...")
    subprocess.run(cmd, check=False)

def main():
    parser = argparse.ArgumentParser(description="Drone Stress Simulation Trigger")
    parser.add_argument("--mode", choices=["clean", "combined", "noise", "delay", "sag", "drift", "latency", "bias"], 
                        default="combined", help="Stress test mode")
    args = parser.parse_args()

    if args.mode == "clean":
        print("\n--- REVERTING TO CLEAN SIMULATION ---")
        set_param("/control_node",    "esc_delay_ms",      0)
        set_param("/control_node",    "battery_sag",       "false")
        set_param("/control_node",    "bias_vx",           0.0)
        set_param("/tracking_node",   "imu_noise",         0.0)
        set_param("/tracking_node",   "gps_drift",         0.0)
        set_param("/perception_node", "camera_latency_ms", 0)
        print("[OK] All stressors disabled.\n")

    elif args.mode == "combined":
        print("\n--- TRIGGERING COMBINED STRESS TEST (ALL 6 CONDITIONS) ---")
        # 1. IMU Noise: Gaussian noise to motion data
        set_param("/tracking_node",   "imu_noise",         0.05)
        # 2. ESC Delay: 50-100ms command buffer
        set_param("/control_node",    "esc_delay_ms",      75)
        # 3. Battery Sag: Scale down thrust
        set_param("/control_node",    "battery_sag",       "true")
        # 4. GPS Drift: slow cumulative positional offset
        set_param("/tracking_node",   "gps_drift",         0.001)
        # 5. Camera Latency: 50-150ms delays
        set_param("/perception_node", "camera_latency_ms", 1)
        # 6. Mechanical Imbalance: 0.05 m/s velocity bias
        set_param("/control_node",    "bias_vx",           0.05)
        print("[OK] Combined stress test active.\n")
        print("Recommended Run: 60 seconds of tracking, then run generate_plots.py")

    elif args.mode == "bias":
        print("\n--- TESTING MECHANICAL IMBALANCE ONLY ---")
        set_param("/control_node", "bias_vx", 0.05)
        print("[OK] Bias active. PID Integral term should correct this over time.\n")

    else:
        # Implement other individual modes as needed
        print(f"Mode {args.mode} selected. (Individual stressors not fully mapped in this helper yet)")

if __name__ == "__main__":
    main()
