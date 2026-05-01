#!/usr/bin/env python3
"""
repeat_runs.py — Automated Repeatability Harness
=================================================
Reformerz Healthcare | Drone Tracking Pipeline v2

Runs the full tracking stack for N sessions, each lasting RUN_DURATION seconds,
saving a unique CSV per run.  After all runs, computes and prints aggregate
metrics to prove consistency.

Prerequisites:
  PX4 SITL, MAVROS, and test_image_pub must already be running.

Usage:
  python3 repeat_runs.py                         # 5 runs × 600 s
  python3 repeat_runs.py --runs 10 --duration 120  # 10 short runs
  python3 repeat_runs.py --scenario occlusion --runs 5 --duration 180

The script calls:
  ros2 launch <launch_file> [scenario:=X]
and sends SIGINT after RUN_DURATION to trigger clean CSV flush.
"""

import argparse
import subprocess
import sys
import time
import os
import signal
from pathlib import Path


# ── Config ────────────────────────────────────────────────────────────────────
DEFAULT_RUNS      = 5
DEFAULT_DURATION  = 600          # seconds per run
DEFAULT_COOLDOWN  = 15           # seconds between runs (lets nodes fully shutdown)
LOG_DIR           = Path.home() / "ros2_ws" / "logs"
LAUNCH_PKG        = "my_robot_controller"
LAUNCH_FILE       = "full_system_v2.launch.py"
SETUP_BASH        = Path.home() / "ros2_ws" / "install" / "setup.bash"

GREEN  = "\033[0;32m"
RED    = "\033[0;31m"
YELLOW = "\033[1;33m"
BLUE   = "\033[1;34m"
NC     = "\033[0m"

def info(msg):  print(f"{BLUE}[INFO]{NC}  {msg}")
def ok(msg):    print(f"{GREEN}[PASS]{NC}  {msg}")
def warn(msg):  print(f"{YELLOW}[WARN]{NC}  {msg}")
def fail(msg):  print(f"{RED}[FAIL]{NC}  {msg}")


# ── Helpers ───────────────────────────────────────────────────────────────────

def latest_csv_after(log_dir: Path, after_epoch: float) -> Path | None:
    """Return the most recently modified session_v2_*.csv created after after_epoch."""
    candidates = sorted(
        [p for p in log_dir.glob("session_v2_*.csv")
         if p.stat().st_mtime >= after_epoch],
        key=lambda p: p.stat().st_mtime
    )
    return candidates[-1] if candidates else None


def run_once(run_idx: int, duration: int, scenario: str,
             extra_args: list[str]) -> Path | None:
    """
    Launch the full stack, wait for duration seconds, then send SIGINT.
    Returns the CSV path written by logger_v2, or None on failure.
    """
    info(f"=== RUN {run_idx} / SCENARIO={scenario} / DURATION={duration}s ===")
    t_start = time.time()

    cmd = ["bash", "-c",
           f"source {SETUP_BASH} && "
           f"ros2 launch {LAUNCH_PKG} {LAUNCH_FILE} "
           f"scenario:={scenario} "
           + " ".join(extra_args)]

    try:
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,   # give it its own process group for clean kill
            text=True,
        )
    except FileNotFoundError:
        fail("ros2 not found — is the workspace sourced?")
        return None

    # Stream output with a prefix so the user can follow along
    import threading

    def _stream():
        for line in proc.stdout:
            print(f"  | {line}", end="")

    t = threading.Thread(target=_stream, daemon=True)
    t.start()

    # Monitor + countdown
    deadline = t_start + duration
    while time.time() < deadline:
        remaining = int(deadline - time.time())
        print(f"\r  [{_bar(duration - remaining, duration)}] "
              f"Run {run_idx}  {duration - remaining}s/{duration}s  "
              f"remaining={remaining}s  ",
              end="", flush=True)
        time.sleep(2)

    print()  # newline after progress bar
    info(f"Run {run_idx} complete — sending SIGINT to stack…")

    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    except ProcessLookupError:
        pass

    proc.wait(timeout=20)
    t.join(timeout=5)

    # Give logger_v2 time to flush its CSV
    time.sleep(3)

    csv = latest_csv_after(LOG_DIR, t_start)
    if csv:
        ok(f"Run {run_idx} CSV: {csv.name}")
    else:
        warn(f"Run {run_idx}: no CSV found in {LOG_DIR}")

    return csv


def _bar(filled: int, total: int, width: int = 20) -> str:
    n = int(filled * width / max(total, 1))
    return "#" * n + "." * (width - n)


# ── Aggregate stats ───────────────────────────────────────────────────────────

def aggregate(csv_paths: list[Path]) -> None:
    """Print a summary table across all runs."""
    try:
        import pandas as pd
        import numpy as np
        # Add version_2 to path so metrics.py is importable
        sys.path.insert(0, str(Path(__file__).parent.parent / "metrics"))
        import metrics as M
    except ImportError as e:
        warn(f"Cannot compute aggregate stats: {e}")
        return

    print()
    print("=" * 70)
    print(" REPEATABILITY SUMMARY".center(70))
    print("=" * 70)
    fmt = "{:<6} {:>10} {:>10} {:>10} {:>12} {:>10}"
    print(fmt.format("Run", "MeanErr", "RMSErr",
                     "Track%", "MaxRecov(s)", "Duration"))
    print("-" * 70)

    all_mean = []; all_rms = []; all_tr = []; all_recov = []

    for i, p in enumerate(csv_paths, 1):
        df = M.load_session(p)
        me = M.mean_error(df)
        rm = M.rms_error(df)
        tr = M.track_ratio(df) * 100
        rc = M.max_recovery_time(df)
        du = M.session_duration(df)
        all_mean.append(me); all_rms.append(rm)
        all_tr.append(tr);   all_recov.append(rc)
        print(fmt.format(f"R{i}", f"{me:.4f}", f"{rm:.4f}",
                         f"{tr:.1f}%", f"{rc:.3f}", f"{du:.0f}s"))

    print("-" * 70)
    print(fmt.format("MEAN",
                     f"{np.mean(all_mean):.4f}",
                     f"{np.mean(all_rms):.4f}",
                     f"{np.mean(all_tr):.1f}%",
                     f"{np.mean(all_recov):.3f}",
                     ""))
    print(fmt.format("STD",
                     f"{np.std(all_mean):.4f}",
                     f"{np.std(all_rms):.4f}",
                     f"{np.std(all_tr):.1f}pp",
                     f"{np.std(all_recov):.3f}",
                     ""))
    print("=" * 70)

    # Pass/fail
    print()
    passed_recov = all(r < 2.0 for r in all_recov)
    passed_tr    = all(t >= 80.0 for t in all_tr)
    consistent   = np.std(all_mean) < 0.05

    def _pf(label, ok_): print(f"  [{'PASS' if ok_ else 'FAIL'}]  {label}")
    _pf("Max recovery < 2.0 s in ALL runs", passed_recov)
    _pf("Track ratio >= 80% in ALL runs",   passed_tr)
    _pf("Run-to-run std(mean_error) < 0.05 (consistent)", consistent)
    print()


# ── CLI ───────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Automated repeatability runner for Week 4 PID validation")
    parser.add_argument("--runs",     type=int, default=DEFAULT_RUNS,
                        help=f"Number of runs (default: {DEFAULT_RUNS})")
    parser.add_argument("--duration", type=int, default=DEFAULT_DURATION,
                        help=f"Seconds per run (default: {DEFAULT_DURATION})")
    parser.add_argument("--scenario", type=str, default="normal",
                        choices=("normal","flicker","occlusion","high_error"),
                        help="Scenario to inject (default: normal)")
    parser.add_argument("--cooldown", type=int, default=DEFAULT_COOLDOWN,
                        help=f"Seconds between runs (default: {DEFAULT_COOLDOWN})")
    parser.add_argument("--dry-run",  action="store_true",
                        help="Print commands without executing")
    args = parser.parse_args()

    LOG_DIR.mkdir(parents=True, exist_ok=True)

    print()
    print("=" * 70)
    print(" WEEK 4 REPEATABILITY HARNESS — Reformerz Healthcare".center(70))
    print(f" {args.runs} runs × {args.duration}s  |  scenario={args.scenario}".center(70))
    print("=" * 70)
    print()

    if args.dry_run:
        info("DRY RUN — would execute:")
        info(f"  source {SETUP_BASH}")
        info(f"  ros2 launch {LAUNCH_PKG} {LAUNCH_FILE} scenario:={args.scenario}")
        info(f"  × {args.runs} runs,  {args.duration}s each,  {args.cooldown}s cooldown")
        return

    csv_paths: list[Path] = []

    for run in range(1, args.runs + 1):
        csv = run_once(run, args.duration, args.scenario, [])
        if csv:
            csv_paths.append(csv)

        if run < args.runs:
            info(f"Cooldown {args.cooldown}s before next run…")
            time.sleep(args.cooldown)

    if csv_paths:
        aggregate(csv_paths)

        # Generate repeatability overlay plot
        try:
            sys.path.insert(0, str(Path(__file__).parent.parent / "plots"))
            import plot as P
            out = LOG_DIR / f"repeatability_{args.scenario}_{len(csv_paths)}runs.png"
            P.plot_repeatability(csv_paths, out)
        except Exception as e:
            warn(f"Could not generate repeatability plot: {e}")
    else:
        fail("No CSVs were produced. Check that logger_v2 is running correctly.")


if __name__ == "__main__":
    main()
