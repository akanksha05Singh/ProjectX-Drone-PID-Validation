# Drone Tracking PID & Stress Validation Suite
## Reformerz Healthcare Robotics | Week 4 Milestone

This repository contains the engineering-grade PID controller upgrade for the "Shiva-Ready" drone tracking system, along with a comprehensive stress simulation suite for robustness testing.

### 🚀 System Requirements
- **OS**: Windows 11 with WSL2 (Ubuntu 22.04)
- **Middleware**: ROS 2 Humble
- **Simulation**: PX4 SITL with Gazebo Classic
- **Dependencies**: MAVROS, OpenCV, NumPy, Matplotlib, Pandas

### 🛠️ Installation
1. Clone the repository into your ROS 2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone <your-repo-link> my_robot_controller
   ```
2. Install dependencies and build:
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --packages-select my_robot_controller
   source install/setup.bash
   ```

### 🎮 Usage
#### 1. Launch the Baseline Tracking Stack
```bash
ros2 launch my_robot_controller drone_tracking.launch.py
```

#### 2. Trigger the Combined Stress Test
In a new terminal, run the stress simulation script to inject IMU noise, ESC delay, battery sag, GPS drift, camera latency, and mechanical imbalance:
```bash
python3 src/my_robot_controller/scripts/stress_sim.py --mode combined
```

#### 3. Generate Performance Analytics
After completing a test run, generate the comparison plots:
```bash
python3 src/my_robot_controller/scripts/generate_plots.py --csv logs/latest_pid_stress.csv --compare logs/week3_p_baseline.csv
```

### 🧠 Key Engineering Features
- **PID Controller**: Replaces the basic P-controller with Integral (Ki=0.05) correction for bias elimination.
- **Anti-Windup**: Integral clamping ($\pm 0.2$) ensures the drone remains stable during extreme disturbances.
- **Low-Pass Filtering**: Exponential Moving Average (EMA) filter on error inputs to mitigate high-frequency IMU noise.
- **Stress Sim Suite**: Real-time parameter injection for 6 critical failure modes.

---
**Lead Validation Engineer**: Shiva-Ready | **Date**: April 2026
