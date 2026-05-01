# Week 4 Metric Summary Report
**Project:** Reformerz Healthcare — Drone Tracking Pipeline  
**Engineer:** [Your Name]  
**Director:** Shiva  
**Date:** [YYYY-MM-DD]  
**Version:** 2 (PID Controller)

---

## 1. System Overview

| Component | Version | Description |
|-----------|---------|-------------|
| Perception | v2 (unchanged) | YOLOv8n ONNX 128×128, ~26 Hz |
| Tracking | v2 (unchanged) | Normalised error, 0.5s watchdog |
| Controller | **PID v2** | kp_yaw=0.5 ki_yaw=0.05 kd_yaw=0.08 |
| | | kp_vx=0.4 ki_vx=0.04 kd_vx=0.06 |
| Logger | v2 | 10 Hz, 4-state FSM logging |
| Scenario Runner | v2 | normal / flicker / occlusion / high_error |

**Key PID improvements over Week 3:**
- Integral term eliminates steady-state error accumulation
- Derivative-on-measurement prevents velocity spikes on target reacquisition
- Anti-windup clamp prevents integrator saturation during LOST periods
- Integrator reset on target loss prevents carry-over between tracking events

---

## 2. Validation Results

### 2a. Single-Run Metrics

| Metric | Baseline (W3 P-ctrl) | Week 4 PID v2 | Delta |
|--------|---------------------|---------------|-------|
| Duration (s) | [FILL] | [FILL] | — |
| Mean Error | [FILL] | [FILL] | **[FILL]%** |
| RMS Error | [FILL] | [FILL] | **[FILL]%** |
| Track Ratio (%) | [FILL] | [FILL] | [FILL] pp |
| Recovery Events | [FILL] | [FILL] | — |
| Mean Recovery (s) | [FILL] | [FILL] | — |
| **Max Recovery (s)** | [FILL] | **[FILL]** | **[FILL] s** |
| vx σ (m/s) | [FILL] | [FILL] | [FILL]% |
| yaw σ (rad/s) | [FILL] | [FILL] | [FILL]% |

> **To fill this table:** Run `python3 compare_baselines.py --baseline <w3.csv> --v2 <w4.csv> --report report.md`

### 2b. Repeatability (5 Runs)

| Run | Mean Error | RMS Error | Track % | Max Recovery (s) | Duration |
|-----|-----------|-----------|---------|-----------------|---------|
| R1  | [FILL] | [FILL] | [FILL] | [FILL] | [FILL]s |
| R2  | [FILL] | [FILL] | [FILL] | [FILL] | [FILL]s |
| R3  | [FILL] | [FILL] | [FILL] | [FILL] | [FILL]s |
| R4  | [FILL] | [FILL] | [FILL] | [FILL] | [FILL]s |
| R5  | [FILL] | [FILL] | [FILL] | [FILL] | [FILL]s |
| **MEAN** | [FILL] | [FILL] | [FILL] | [FILL] | — |
| **STD** | [FILL] | [FILL] | [FILL] | [FILL] | — |

> **To fill this table:** Run `python3 repeat_runs.py --runs 5 --duration 600`

---

## 3. Success Criteria

| # | Criterion | Target | Achieved | Status |
|---|-----------|--------|----------|--------|
| 1 | Mean error PID < Baseline (W3) | < baseline | [FILL] | [ ] PASS / [ ] FAIL |
| 2 | Max recovery time | < 2.0 s | [FILL] s | [ ] PASS / [ ] FAIL |
| 3 | Track ratio | >= 80% | [FILL]% | [ ] PASS / [ ] FAIL |
| 4 | vx oscillation (σ) | < 0.15 m/s | [FILL] m/s | [ ] PASS / [ ] FAIL |
| 5 | Consistent across 5 runs (σ of means) | < 0.05 | [FILL] | [ ] PASS / [ ] FAIL |
| 6 | Runtime without crash | 10–15 min | [FILL] min | [ ] PASS / [ ] FAIL |

---

## 4. Evidence Figures

| Figure | File | Description |
|--------|------|-------------|
| Fig 1 | `session_v2_*_error.png` | Error X, Y, magnitude vs time |
| Fig 2 | `session_v2_*_velocity.png` | vx, vy, yaw_rate vs time |
| Fig 3 | `session_v2_*_states.png` | TRACK/SEARCH/HOLD/FAILSAFE transitions |
| Fig 4 | `session_v2_*_comparison.png` | Side-by-side PID vs P-ctrl bar chart |
| Fig 5 | `repeatability_normal_5runs.png` | 5-run error overlay (consistency) |
| Fig 6 | `rqt_graph.png` | Node topology diagram |

---

## 5. Scenario Test Results

| Scenario | Runs | Mean Error | Max Recovery (s) | Pass? |
|----------|------|-----------|-----------------|-------|
| Normal | 5 | [FILL] | [FILL] | [ ] |
| Flicker (30% drop) | 3 | [FILL] | [FILL] | [ ] |
| Occlusion (2s/5s) | 3 | [FILL] | [FILL] | [ ] |
| High Error (+0.3) | 3 | [FILL] | [FILL] | [ ] |

> **To run:** `python3 repeat_runs.py --scenario occlusion --runs 3 --duration 300`

---

## 6. Conclusion

[FILL: 2–3 sentences summarising whether the Week 4 PID system meets all success criteria, referencing the specific percentage improvements in mean error and the recovery time result.]

---

## 7. Appendix — Run Configuration

```
Controller  : PID v2
kp_yaw      : 0.5    ki_yaw : 0.05   kd_yaw : 0.08
kp_vx       : 0.4    ki_vx  : 0.04   kd_vx  : 0.06
max_yaw     : 0.6 rad/s
max_vx      : 0.5 m/s
deadband    : 0.05
integral_limit: 2.0
search_timeout: 1.5 s
Control rate: 10 Hz
Logger rate : 10 Hz

Platform    : PX4 SITL (Iris quad) + Gazebo-classic
MAVROS      : px4.launch (UDP)
Perception  : YOLOv8n ONNX 128x128, conf_threshold=0.40
```
