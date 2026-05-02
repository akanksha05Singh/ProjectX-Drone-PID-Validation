# Week 4 Metric Summary Report
**Project:** Reformerz Healthcare — Drone Tracking Pipeline
**Engineer:** [Your Name]  |  **Date:** 2026-05-02 10:22

## Validation Results

| Metric | Baseline (W3 P-ctrl) | Week 4 PID v2 | Delta |
|--------|---------------------|---------------|-------|
| Duration (s) | 699.4 | 894.9 | — |
| Mean Error | 0.6861 | 0.6929 | **-1.0%** |
| RMS Error | 0.6902 | 0.6936 | **-0.5%** |
| Track Ratio (%) | 98.8 | 99.8 | +1.0 pp |
| Recovery Events | 15 | 16 | — |
| Mean Recovery (s) | 0.560 | 0.116 | — |
| Max Recovery (s) | 6.500 | 0.258 | **+6.243 s** |
| vx σ (m/s) | 0.0091 | 0.0222 | -143.7% |

## Success Criteria

- [ ] Mean error PID < Baseline
- [x] Max recovery time < 2.0 s
- [x] Track ratio >= 80%
- [x] vx σ < 0.15 m/s (no oscillation)

## Notes

- Baseline: Week 3 P-controller (kp_yaw=0.5, kp_vx=0.4)
- Week 4: PID with integral anti-windup + derivative-on-measurement
- All runs validated via automated 10-minute repeatability harness