#!/usr/bin/env python3
"""
metrics.py — Week 4 Performance Metrics Engine
===============================================
Reformerz Healthcare | Drone Tracking Pipeline v2

Pure Python / pandas — no ROS dependency.
Reads session CSVs produced by logger_v2.py (or logger_node.py for baseline).

Functions
---------
load_session(path)          -> pd.DataFrame
mean_error(df)              -> float
rms_error(df)               -> float
track_ratio(df)             -> float   (0.0 – 1.0)
recovery_times(df)          -> list[float]  (seconds per recovery event)
max_recovery_time(df)       -> float
velocity_oscillation(df)    -> dict   {vx_std, vy_std, yaw_std}
compare(df_baseline, df_v2) -> dict   (all metrics side-by-side)
print_report(metrics_dict)           (pretty-print to terminal)
save_report(metrics_dict, path)      (write Markdown summary)

Usage
-----
  # Single session:
  python3 metrics.py --csv ~/ros2_ws/logs/session_v2_20260418.csv

  # Comparison (Week 3 baseline vs Week 4 PID):
  python3 metrics.py \\
      --csv      ~/ros2_ws/logs/session_v2_20260418.csv \\
      --baseline ~/ros2_ws/logs/session_20260418_151251.csv
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from datetime import datetime

import numpy as np
import pandas as pd


# ── Column name aliases ────────────────────────────────────────────────────────
# Week 3 baseline CSV uses "status"; Week 4 uses "detection_status".
# We normalise to a single internal name.
_STATUS_ALIASES = ["detection_status", "status"]
_STATE_ALIASES  = ["system_state"]


def _find_col(df: pd.DataFrame, candidates: list[str]) -> str | None:
    for c in candidates:
        if c in df.columns:
            return c
    return None


# ── Loaders ───────────────────────────────────────────────────────────────────

def load_session(path: Path) -> pd.DataFrame:
    """Load a session CSV, normalise time axis to elapsed_s from start."""
    df = pd.read_csv(path)

    # Ensure elapsed_s column
    if "elapsed_s" not in df.columns:
        if "timestamp_unix" in df.columns:
            df["elapsed_s"] = df["timestamp_unix"] - df["timestamp_unix"].iloc[0]
        else:
            df["elapsed_s"] = np.arange(len(df)) * 0.1   # assume 10 Hz

    # Normalise status column
    status_col = _find_col(df, _STATUS_ALIASES)
    if status_col and status_col != "detection_status":
        df = df.rename(columns={status_col: "detection_status"})

    # Normalise system_state (Week 3 baseline doesn't have it — derive from status)
    if "system_state" not in df.columns:
        if "detection_status" in df.columns:
            df["system_state"] = df["detection_status"].map(
                {"TRACKING": "TRACK", "LOST": "HOLD"}).fillna("HOLD")
        else:
            df["system_state"] = "UNKNOWN"

    # Ensure error_mag
    if "error_mag" not in df.columns:
        df["error_mag"] = np.sqrt(df["error_x"] ** 2 + df["error_y"] ** 2)

    return df


# ── Core metrics ──────────────────────────────────────────────────────────────

def mean_error(df: pd.DataFrame) -> float:
    """Mean absolute Euclidean tracking error over entire session."""
    return float(df["error_mag"].mean())


def rms_error(df: pd.DataFrame) -> float:
    """Root Mean Square of Euclidean tracking error."""
    return float(np.sqrt((df["error_mag"] ** 2).mean()))


def track_ratio(df: pd.DataFrame) -> float:
    """Fraction of rows where detection_status == TRACKING (or system_state == TRACK)."""
    if "detection_status" in df.columns:
        n_track = (df["detection_status"] == "TRACKING").sum()
    else:
        n_track = (df["system_state"] == "TRACK").sum()
    return float(n_track / max(len(df), 1))


def recovery_times(df: pd.DataFrame) -> list[float]:
    """
    List of recovery durations (seconds) for each LOST → TRACKING transition.

    A recovery event starts when detection_status first becomes LOST and ends
    when it returns to TRACKING.  Returns one float per event.
    """
    if "detection_status" not in df.columns:
        return []

    times: list[float] = []
    in_lost = False
    lost_start = 0.0

    for _, row in df.iterrows():
        status = row["detection_status"]
        t      = row["elapsed_s"]

        if status == "LOST" and not in_lost:
            lost_start = t
            in_lost    = True
        elif status == "TRACKING" and in_lost:
            times.append(t - lost_start)
            in_lost = False

    return times


def max_recovery_time(df: pd.DataFrame) -> float:
    """Worst-case recovery time across all events.  0.0 if no losses occurred."""
    rt = recovery_times(df)
    return float(max(rt)) if rt else 0.0


def mean_recovery_time(df: pd.DataFrame) -> float:
    """Average recovery time across all events.  0.0 if no losses occurred."""
    rt = recovery_times(df)
    return float(np.mean(rt)) if rt else 0.0


def velocity_oscillation(df: pd.DataFrame) -> dict[str, float]:
    """
    Standard deviation of each velocity command channel.
    Low σ = smooth commands (no oscillation).  Target: σ_vx < 0.15 m/s.
    """
    result: dict[str, float] = {}
    for col, key in [("vx", "vx_std"), ("vy", "vy_std"), ("yaw_rate", "yaw_std"),
                     ("cmd_vx", "vx_std"), ("cmd_vy", "vy_std"), ("cmd_yaw", "yaw_std")]:
        if col in df.columns and key not in result:
            result[key] = float(df[col].std())
    return result


def session_duration(df: pd.DataFrame) -> float:
    """Total session length in seconds."""
    return float(df["elapsed_s"].iloc[-1])


# ── Comparison ───────────────────────────────────────────────────────────────

def compare(df_baseline: pd.DataFrame, df_v2: pd.DataFrame) -> dict:
    """
    Side-by-side metric dict for Week 3 P-controller baseline vs Week 4 PID.

    Returns a dict with keys: 'baseline' and 'v2', each containing
    the full metric sub-dict, plus a 'delta' section showing improvement.
    """
    def _metrics(df: pd.DataFrame) -> dict:
        osc = velocity_oscillation(df)
        return {
            "duration_s":       session_duration(df),
            "mean_error":       mean_error(df),
            "rms_error":        rms_error(df),
            "track_ratio_pct":  track_ratio(df) * 100,
            "recovery_count":   len(recovery_times(df)),
            "mean_recovery_s":  mean_recovery_time(df),
            "max_recovery_s":   max_recovery_time(df),
            "vx_std":           osc.get("vx_std", float("nan")),
            "vy_std":           osc.get("vy_std", float("nan")),
            "yaw_std":          osc.get("yaw_std", float("nan")),
        }

    b = _metrics(df_baseline)
    v = _metrics(df_v2)

    delta = {
        "mean_error_reduction_pct": (
            (b["mean_error"] - v["mean_error"]) / max(b["mean_error"], 1e-9) * 100
        ),
        "rms_error_reduction_pct": (
            (b["rms_error"] - v["rms_error"]) / max(b["rms_error"], 1e-9) * 100
        ),
        "track_ratio_gain_ppt": v["track_ratio_pct"] - b["track_ratio_pct"],
        "max_recovery_improvement_s": b["max_recovery_s"] - v["max_recovery_s"],
        "vx_oscillation_reduction_pct": (
            (b["vx_std"] - v["vx_std"]) / max(b["vx_std"], 1e-9) * 100
        ),
    }

    return {"baseline": b, "v2": v, "delta": delta}


# ── Reporting ────────────────────────────────────────────────────────────────

def print_report(result: dict) -> None:
    """Pretty-print a comparison report to the terminal."""
    b, v, d = result["baseline"], result["v2"], result["delta"]

    W = 68
    print()
    print("=" * W)
    print(" WEEK 4 VALIDATION REPORT — Reformerz Healthcare".center(W))
    print(" PID v2  vs  P-Controller Baseline".center(W))
    print("=" * W)
    fmt = "{:<28} {:>16} {:>16}"
    print(fmt.format("Metric", "Baseline (W3)", "PID v2 (W4)"))
    print("-" * W)

    rows = [
        ("Duration (s)",           f"{b['duration_s']:.1f}",       f"{v['duration_s']:.1f}"),
        ("Mean Error",             f"{b['mean_error']:.4f}",        f"{v['mean_error']:.4f}"),
        ("RMS Error",              f"{b['rms_error']:.4f}",         f"{v['rms_error']:.4f}"),
        ("Track Ratio (%)",        f"{b['track_ratio_pct']:.1f}",   f"{v['track_ratio_pct']:.1f}"),
        ("Recovery Events",        f"{b['recovery_count']}",        f"{v['recovery_count']}"),
        ("Mean Recovery (s)",      f"{b['mean_recovery_s']:.3f}",   f"{v['mean_recovery_s']:.3f}"),
        ("Max Recovery (s)",       f"{b['max_recovery_s']:.3f}",    f"{v['max_recovery_s']:.3f}"),
        ("vx σ (m/s)",             f"{b['vx_std']:.4f}",            f"{v['vx_std']:.4f}"),
        ("yaw σ (rad/s)",          f"{b['yaw_std']:.4f}",           f"{v['yaw_std']:.4f}"),
    ]
    for label, bval, vval in rows:
        print(fmt.format(label, bval, vval))

    print("=" * W)
    print(" KEY IMPROVEMENTS".center(W))
    print("-" * W)
    print(f"  Mean error reduction    : {d['mean_error_reduction_pct']:+.1f}%")
    print(f"  RMS  error reduction    : {d['rms_error_reduction_pct']:+.1f}%")
    print(f"  Track ratio gain        : {d['track_ratio_gain_ppt']:+.1f} pp")
    print(f"  Max recovery improvement: {d['max_recovery_improvement_s']:+.3f} s")
    print(f"  vx oscillation reduction: {d['vx_oscillation_reduction_pct']:+.1f}%")

    # Pass/fail criteria
    print()
    print(" SUCCESS CRITERIA".center(W))
    print("-" * W)
    _pf("Mean error PID < Baseline",
        v["mean_error"] < b["mean_error"])
    _pf("Max recovery time < 2.0 s",
        v["max_recovery_s"] < 2.0)
    _pf("Track ratio >= 80%",
        v["track_ratio_pct"] >= 80.0)
    _pf("vx σ < 0.15 m/s (no oscillation)",
        v["vx_std"] < 0.15)
    print("=" * W)
    print()


def _pf(label: str, passed: bool) -> None:
    mark = "PASS" if passed else "FAIL"
    print(f"  [{mark}]  {label}")


def save_report(result: dict, path: Path) -> None:
    """Write a Markdown metric summary to disk."""
    b, v, d = result["baseline"], result["v2"], result["delta"]
    now = datetime.now().strftime("%Y-%m-%d %H:%M")

    lines = [
        "# Week 4 Metric Summary Report",
        f"**Project:** Reformerz Healthcare — Drone Tracking Pipeline",
        f"**Engineer:** [Your Name]  |  **Date:** {now}",
        "",
        "## Validation Results",
        "",
        "| Metric | Baseline (W3 P-ctrl) | Week 4 PID v2 | Delta |",
        "|--------|---------------------|---------------|-------|",
        f"| Duration (s) | {b['duration_s']:.1f} | {v['duration_s']:.1f} | — |",
        f"| Mean Error | {b['mean_error']:.4f} | {v['mean_error']:.4f} | **{d['mean_error_reduction_pct']:+.1f}%** |",
        f"| RMS Error | {b['rms_error']:.4f} | {v['rms_error']:.4f} | **{d['rms_error_reduction_pct']:+.1f}%** |",
        f"| Track Ratio (%) | {b['track_ratio_pct']:.1f} | {v['track_ratio_pct']:.1f} | {d['track_ratio_gain_ppt']:+.1f} pp |",
        f"| Recovery Events | {b['recovery_count']} | {v['recovery_count']} | — |",
        f"| Mean Recovery (s) | {b['mean_recovery_s']:.3f} | {v['mean_recovery_s']:.3f} | — |",
        f"| Max Recovery (s) | {b['max_recovery_s']:.3f} | {v['max_recovery_s']:.3f} | **{d['max_recovery_improvement_s']:+.3f} s** |",
        f"| vx σ (m/s) | {b['vx_std']:.4f} | {v['vx_std']:.4f} | {d['vx_oscillation_reduction_pct']:+.1f}% |",
        "",
        "## Success Criteria",
        "",
        f"- [{'x' if v['mean_error'] < b['mean_error'] else ' '}] Mean error PID < Baseline",
        f"- [{'x' if v['max_recovery_s'] < 2.0 else ' '}] Max recovery time < 2.0 s",
        f"- [{'x' if v['track_ratio_pct'] >= 80.0 else ' '}] Track ratio >= 80%",
        f"- [{'x' if v['vx_std'] < 0.15 else ' '}] vx σ < 0.15 m/s (no oscillation)",
        "",
        "## Notes",
        "",
        "- Baseline: Week 3 P-controller (kp_yaw=0.5, kp_vx=0.4)",
        "- Week 4: PID with integral anti-windup + derivative-on-measurement",
        "- All runs validated via automated 10-minute repeatability harness",
    ]
    path.write_text("\n".join(lines), encoding="utf-8")
    print(f"[OK] Markdown report saved -> {path}")


# ── CLI ───────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="Week 4 metrics engine")
    parser.add_argument("--csv",      type=Path, required=True,
                        help="Path to Week 4 session CSV")
    parser.add_argument("--baseline", type=Path, default=None,
                        help="Path to Week 3 baseline CSV (for comparison)")
    parser.add_argument("--report",   type=Path, default=None,
                        help="Save Markdown report to this path")
    args = parser.parse_args()

    print(f"\n[INFO] Loading v2 session: {args.csv}")
    df_v2 = load_session(args.csv)
    print(f"[INFO] {len(df_v2)} rows  {session_duration(df_v2):.1f}s")

    if args.baseline:
        print(f"[INFO] Loading baseline : {args.baseline}")
        df_b = load_session(args.baseline)

        result = compare(df_b, df_v2)
        print_report(result)

        if args.report:
            save_report(result, args.report)
    else:
        # Single-session summary
        osc = velocity_oscillation(df_v2)
        rt  = recovery_times(df_v2)
        print(f"\n  Duration        : {session_duration(df_v2):.1f} s")
        print(f"  Mean Error      : {mean_error(df_v2):.4f}")
        print(f"  RMS  Error      : {rms_error(df_v2):.4f}")
        print(f"  Track Ratio     : {track_ratio(df_v2)*100:.1f}%")
        print(f"  Recovery Events : {len(rt)}")
        print(f"  Max Recovery    : {max_recovery_time(df_v2):.3f} s")
        print(f"  vx σ            : {osc.get('vx_std', float('nan')):.4f} m/s")


if __name__ == "__main__":
    main()
