# Version 2 — PID Drone Tracking Validation
**Reformerz Healthcare | Week 4 Engineering Deliverable**

## Directory Layout

```
version_2/
├── control/
│   ├── pid_controller.py        PID class (anti-windup, derivative-on-measurement)
│   └── control_node_v2.py      ROS 2 node — PID controller + 4-state FSM
├── logger/
│   └── logger_v2.py            ROS 2 node — extended CSV recorder (10 Hz)
├── metrics/
│   └── metrics.py              Standalone: mean/RMS error, track ratio, recovery time
├── scenarios/
│   └── scenario_runner.py      ROS 2 node — test scenario middleware
├── plots/
│   └── plot.py                 Standalone: 4 publication-quality figures
├── scripts/
│   ├── repeat_runs.py          Repeatability harness (5+ automated runs)
│   └── compare_baselines.py    Week 3 vs Week 4 one-shot comparison
├── launch/
│   └── full_system_v2.launch.py  Single-command stack launcher
└── reports/
    └── metric_summary_template.md  Fill-in report template for Shiva
```

## Quick Start

### 1 — Prerequisites (run in separate terminals)
```bash
# Terminal A: PX4 SITL
make px4_sitl gazebo-classic_iris

# Terminal B: MAVROS
ros2 launch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"

# Terminal C: Camera source
ros2 run my_robot_controller test_image_pub
```

### 2 — Launch full Week 4 stack
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch my_robot_controller full_system_v2.launch.py
```

### 3 — Run with a test scenario
```bash
ros2 launch my_robot_controller full_system_v2.launch.py scenario:=occlusion
ros2 launch my_robot_controller full_system_v2.launch.py scenario:=flicker
ros2 launch my_robot_controller full_system_v2.launch.py scenario:=high_error
```

### 4 — Automated 5-run repeatability test
```bash
python3 version_2/scripts/repeat_runs.py --runs 5 --duration 600
```

### 5 — Compare Week 3 vs Week 4
```bash
python3 version_2/scripts/compare_baselines.py \
    --baseline ~/ros2_ws/logs/session_20260418_151251.csv \
    --v2       ~/ros2_ws/logs/session_v2_20260419_100000.csv \
    --report   ~/ros2_ws/logs/week4_report.md
```

### 6 — Generate plots from any CSV
```bash
python3 version_2/plots/plot.py \
    --csv      ~/ros2_ws/logs/session_v2_20260419.csv \
    --baseline ~/ros2_ws/logs/session_20260418.csv
```

## Success Targets

| Metric | Target |
|--------|--------|
| Runtime | 10–15 min, no crash |
| Consistency | Stable across 5+ runs |
| Recovery Time | < 2.0 s |
| Velocity oscillation (vx σ) | < 0.15 m/s |
| Mean error | PID v2 < Week 3 P-ctrl |

## PID Gains (defaults)

| Axis | kp | ki | kd | limit |
|------|----|----|----|-------|
| yaw  | 0.5 | 0.05 | 0.08 | 0.6 rad/s |
| vx   | 0.4 | 0.04 | 0.06 | 0.5 m/s |

Tune at launch time:
```bash
ros2 launch my_robot_controller full_system_v2.launch.py \
    kp_yaw:=0.6 ki_yaw:=0.06 kd_yaw:=0.10
```
