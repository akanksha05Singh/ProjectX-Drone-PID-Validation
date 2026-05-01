#!/usr/bin/env python3
"""
compare_baselines.py — Week 3 vs Week 4 Quick Comparison
=========================================================
Reformerz Healthcare | Drone Tracking Pipeline v2

One-shot script to compare a Week 3 P-controller CSV against
a Week 4 PID CSV and produce both a terminal report and plots.

Usage:
  python3 compare_baselines.py \\
      --baseline ~/ros2_ws/logs/session_20260418_151251.csv \\
      --v2       ~/ros2_ws/logs/session_v2_20260419_100000.csv

  # Save a Markdown report:
  python3 compare_baselines.py \\
      --baseline ... --v2 ... \\
      --report   ~/ros2_ws/logs/week4_validation_report.md
"""

import argparse
import sys
from pathlib import Path

# Allow running from any directory
sys.path.insert(0, str(Path(__file__).parent.parent / "metrics"))
sys.path.insert(0, str(Path(__file__).parent.parent / "plots"))

import metrics as M
import plot    as P


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Compare Week 3 baseline vs Week 4 PID session")
    parser.add_argument("--baseline", type=Path, required=True,
                        help="Week 3 P-controller session CSV")
    parser.add_argument("--v2",       type=Path, required=True,
                        help="Week 4 PID v2 session CSV")
    parser.add_argument("--report",   type=Path, default=None,
                        help="Output path for Markdown report")
    args = parser.parse_args()

    if not args.baseline.exists():
        print(f"[ERROR] Baseline CSV not found: {args.baseline}")
        sys.exit(1)
    if not args.v2.exists():
        print(f"[ERROR] v2 CSV not found: {args.v2}")
        sys.exit(1)

    print(f"\n[INFO] Baseline : {args.baseline.name}")
    print(f"[INFO] PID v2   : {args.v2.name}\n")

    df_b  = M.load_session(args.baseline)
    df_v2 = M.load_session(args.v2)

    result = M.compare(df_b, df_v2)
    M.print_report(result)

    # Generate comparison plot
    out_plot = args.v2.parent / f"{args.v2.stem}_vs_baseline_comparison.png"
    P.plot_comparison(df_b, df_v2, out_plot)

    # Generate individual plots for v2
    P.plot_error(    df_v2, args.v2.parent / f"{args.v2.stem}_error.png")
    P.plot_velocity( df_v2, args.v2.parent / f"{args.v2.stem}_velocity.png")
    P.plot_states(   df_v2, args.v2.parent / f"{args.v2.stem}_states.png")

    if args.report:
        M.save_report(result, args.report)
        print(f"[OK] Report saved -> {args.report}")


if __name__ == "__main__":
    main()
