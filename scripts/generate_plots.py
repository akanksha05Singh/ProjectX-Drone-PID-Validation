#!/usr/bin/env python3
"""
generate_plots.py — Week 3 Analysis Engine
===========================================
Reformerz Healthcare | Drone Tracking Pipeline

Reads session_data.csv produced by logger_node.py and generates
three publication-quality graphs for the formal engineering report:

  Fig 1 — Stability Graph    : error_x / error_y vs time
  Fig 2 — Control Response   : cmd_vx / cmd_vy / cmd_yaw vs time
  Fig 3 — Latency Histogram  : E2E latency distribution from latency_validator

Usage:
  # Latest session (auto-detected):
  python3 generate_plots.py

  # Specific file:
  python3 generate_plots.py --csv ~/ros2_ws/logs/session_20260418_143000.csv

  # Also generate latency histogram from a latency log:
  python3 generate_plots.py --latency ~/ros2_ws/logs/latency_log.csv

Outputs (saved next to the CSV):
  session_<name>_stability.png
  session_<name>_control_response.png
  session_<name>_latency_histogram.png   (if --latency provided)
"""

import argparse
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")          # headless — works in WSL2 without a display
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
import pandas as pd

# ── Plot style ────────────────────────────────────────────────────────────────
plt.rcParams.update({
    "figure.dpi":       150,
    "figure.facecolor": "#0f0f0f",
    "axes.facecolor":   "#1a1a2e",
    "axes.edgecolor":   "#444",
    "axes.labelcolor":  "#e0e0e0",
    "axes.titlesize":   13,
    "axes.titlecolor":  "#ffffff",
    "axes.grid":        True,
    "grid.color":       "#333355",
    "grid.linewidth":   0.6,
    "xtick.color":      "#aaa",
    "ytick.color":      "#aaa",
    "text.color":       "#e0e0e0",
    "legend.facecolor": "#1a1a2e",
    "legend.edgecolor": "#555",
    "lines.linewidth":  1.6,
})

COLORS = {
    "error_x":  "#00d4ff",
    "error_y":  "#ff6b35",
    "vx":       "#00ff88",
    "vy":       "#ffcc00",
    "yaw":      "#ff4488",
    "tracking": "#00d4ff",
    "lost":     "#ff4444",
    "p95":      "#ffcc00",
}


# ── Helpers ───────────────────────────────────────────────────────────────────

def _load_csv(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    required = {"timestamp_unix", "error_x", "error_y",
                "cmd_vx", "cmd_vy", "cmd_yaw", "status"}
    missing = required - set(df.columns)
    if missing:
        print(f"[ERROR] CSV missing columns: {missing}")
        sys.exit(1)
    # Normalise time axis to seconds-from-start
    df["t"] = df["timestamp_unix"] - df["timestamp_unix"].iloc[0]
    return df


def _add_status_shading(ax, df: pd.DataFrame) -> None:
    """Shade background red where status == LOST."""
    lost = df["status"] == "LOST"
    in_block = False
    block_start = 0.0
    for i, row in df.iterrows():
        if lost[i] and not in_block:
            block_start = row["t"]; in_block = True
        elif not lost[i] and in_block:
            ax.axvspan(block_start, row["t"],
                       alpha=0.15, color=COLORS["lost"], zorder=0)
            in_block = False
    if in_block:
        ax.axvspan(block_start, df["t"].iloc[-1],
                   alpha=0.15, color=COLORS["lost"], zorder=0)


def _annotate_stats(ax, series: pd.Series, color: str, label: str) -> None:
    """Print mean ± std in a text box on the axes."""
    mu = series.mean(); sigma = series.std()
    ax.text(0.01, 0.97, f"{label}: μ={mu:.3f}  σ={sigma:.3f}",
            transform=ax.transAxes, fontsize=8, color=color,
            verticalalignment="top",
            bbox=dict(boxstyle="round,pad=0.3", facecolor="#111122",
                      edgecolor=color, alpha=0.8))


# ── Figure 1: Stability Graph ─────────────────────────────────────────────────

def plot_stability(df: pd.DataFrame, out_path: Path) -> None:
    """
    Proves the P-controller is STABLE:
      • Lines should start non-zero and converge toward 0.0 as target is centred.
      • Flat lines at ~0 mean the drone is holding the target perfectly.
      • Red shading shows LOST intervals — error correctly returns to 0 (watchdog).

    Engineering interpretation for report:
      σ < 0.1  → excellent gain tuning
      σ 0.1–0.3 → acceptable, slight oscillation
      σ > 0.5  → gains too high or target moving too fast
    """
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    fig.suptitle(
        "Stability Analysis — Tracking Error Convergence\n"
        "Reformerz Healthcare | Week 3 Drone Tracking",
        fontsize=14, color="white", y=0.98
    )

    # ── Top: error_x (yaw axis) ──
    ax1.plot(df["t"], df["error_x"], color=COLORS["error_x"],
             label="error_x  (horizontal / yaw axis)", alpha=0.9)
    ax1.axhline(0, color="#ffffff", linewidth=0.8, linestyle="--", alpha=0.4)
    ax1.set_ylabel("Normalised Error", fontsize=10)
    ax1.set_ylim(-1.2, 1.2)
    ax1.legend(loc="upper right", fontsize=9)
    _add_status_shading(ax1, df)
    _annotate_stats(ax1, df["error_x"], COLORS["error_x"], "error_x")

    # ── Bottom: error_y (forward axis) ──
    ax2.plot(df["t"], df["error_y"], color=COLORS["error_y"],
             label="error_y  (vertical / forward axis)", alpha=0.9)
    ax2.axhline(0, color="#ffffff", linewidth=0.8, linestyle="--", alpha=0.4)
    ax2.set_ylabel("Normalised Error", fontsize=10)
    ax2.set_xlabel("Time (seconds)", fontsize=10)
    ax2.set_ylim(-1.2, 1.2)
    ax2.legend(loc="upper right", fontsize=9)
    _add_status_shading(ax2, df)
    _annotate_stats(ax2, df["error_y"], COLORS["error_y"], "error_y")

    # ── Legend for red shading ──
    lost_patch = mpatches.Patch(color=COLORS["lost"], alpha=0.3, label="LOST interval")
    fig.legend(handles=[lost_patch], loc="lower right", fontsize=8, framealpha=0.5)

    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(out_path, bbox_inches="tight")
    plt.close(fig)
    print(f"[OK] Stability graph saved → {out_path}")


# ── Figure 2: Control Response ────────────────────────────────────────────────

def plot_control_response(df: pd.DataFrame, out_path: Path) -> None:
    """
    Proves the P-controller is RESPONSIVE:
      • Velocity commands should mirror the error: large error → large velocity.
      • Smooth curves = well-tuned gains.
      • Spikes = target jumps or gain too high.
      • Zero-flat during LOST = safety watchdog working.

    Engineering interpretation for report:
      max(|cmd_vx|) should stay within declared limits (max_vx = 0.5 m/s).
      max(|cmd_yaw|) should stay within declared limits (max_yaw = 0.6 rad/s).
    """
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    fig.suptitle(
        "Control Response — Velocity Commands to MAVROS\n"
        "Reformerz Healthcare | Week 3 Drone Tracking",
        fontsize=14, color="white", y=0.98
    )

    # ── vx (forward) ──
    ax1.plot(df["t"], df["cmd_vx"], color=COLORS["vx"],
             label="cmd_vx  (forward, m/s)")
    ax1.axhline(0, color="#ffffff", linewidth=0.6, linestyle="--", alpha=0.3)
    ax1.axhline( 0.5, color="#888", linewidth=0.8, linestyle=":", alpha=0.5)
    ax1.axhline(-0.5, color="#888", linewidth=0.8, linestyle=":", alpha=0.5)
    ax1.set_ylabel("vx (m/s)", fontsize=9)
    ax1.legend(loc="upper right", fontsize=9)
    _add_status_shading(ax1, df)
    _annotate_stats(ax1, df["cmd_vx"], COLORS["vx"], "cmd_vx")

    # ── vy (lateral) ──
    ax2.plot(df["t"], df["cmd_vy"], color=COLORS["vy"],
             label="cmd_vy  (lateral, m/s)")
    ax2.axhline(0, color="#ffffff", linewidth=0.6, linestyle="--", alpha=0.3)
    ax2.set_ylabel("vy (m/s)", fontsize=9)
    ax2.legend(loc="upper right", fontsize=9)
    _add_status_shading(ax2, df)
    _annotate_stats(ax2, df["cmd_vy"], COLORS["vy"], "cmd_vy")

    # ── yaw ──
    ax3.plot(df["t"], df["cmd_yaw"], color=COLORS["yaw"],
             label="cmd_yaw  (yaw rate, rad/s)")
    ax3.axhline(0, color="#ffffff", linewidth=0.6, linestyle="--", alpha=0.3)
    ax3.axhline( 0.6, color="#888", linewidth=0.8, linestyle=":", alpha=0.5)
    ax3.axhline(-0.6, color="#888", linewidth=0.8, linestyle=":", alpha=0.5)
    ax3.set_ylabel("yaw_rate (rad/s)", fontsize=9)
    ax3.set_xlabel("Time (seconds)", fontsize=10)
    ax3.legend(loc="upper right", fontsize=9)
    _add_status_shading(ax3, df)
    _annotate_stats(ax3, df["cmd_yaw"], COLORS["yaw"], "cmd_yaw")

    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(out_path, bbox_inches="tight")
    plt.close(fig)
    print(f"[OK] Control response graph saved → {out_path}")


# ── Figure 3: Latency Histogram ───────────────────────────────────────────────

def plot_latency_histogram(latency_samples_ms: list, out_path: Path,
                           title_suffix: str = "") -> None:
    """
    Proves the pipeline is LOW-LATENCY:
      • Histogram shape should be tight (small σ) and left-skewed.
      • p95 < 100ms = excellent (bare-metal).
      • p95 100–250ms = acceptable (WSL2 overhead).
      • p95 > 250ms = needs investigation.

    Engineering interpretation for report:
      A narrow histogram with p95 below the target line proves the system
      can react to a moving target within the required time budget.
    """
    if not latency_samples_ms:
        print("[WARN] No latency samples — skipping histogram")
        return

    arr = np.array(latency_samples_ms)
    p50 = np.percentile(arr, 50)
    p95 = np.percentile(arr, 95)
    p99 = np.percentile(arr, 99)

    fig, ax = plt.subplots(figsize=(10, 6))
    fig.suptitle(
        f"E2E Latency Distribution — Camera → cmd_vel{title_suffix}\n"
        "Reformerz Healthcare | Week 3 Drone Tracking",
        fontsize=13, color="white"
    )

    # Histogram
    n_bins = min(50, max(10, len(arr) // 5))
    n, bins, patches = ax.hist(arr, bins=n_bins, color=COLORS["tracking"],
                               alpha=0.75, edgecolor="#004466",
                               label=f"Samples (n={len(arr)})")

    # Colour bars above p95 red
    for patch, left in zip(patches, bins[:-1]):
        if left >= p95:
            patch.set_facecolor(COLORS["lost"])
            patch.set_alpha(0.85)

    # Percentile lines
    ax.axvline(p50, color="#ffffff",       linewidth=1.5, linestyle="--",
               label=f"p50 = {p50:.1f} ms")
    ax.axvline(p95, color=COLORS["p95"],   linewidth=2.0, linestyle="-",
               label=f"p95 = {p95:.1f} ms")
    ax.axvline(p99, color=COLORS["lost"],  linewidth=1.5, linestyle=":",
               label=f"p99 = {p99:.1f} ms")

    # Target line
    target = 100.0
    ax.axvline(target, color="#00ff88", linewidth=1.2, linestyle="-.",
               alpha=0.7, label=f"Target: {target:.0f} ms")

    ax.set_xlabel("Latency (ms)", fontsize=11)
    ax.set_ylabel("Count", fontsize=11)
    ax.legend(fontsize=10)

    # Stats box
    stats = (f"mean = {arr.mean():.1f} ms\n"
             f"std  = {arr.std():.1f} ms\n"
             f"min  = {arr.min():.1f} ms\n"
             f"p95  = {p95:.1f} ms\n"
             f"max  = {arr.max():.1f} ms")
    verdict = "PASS" if p95 <= 250 else "FAIL"
    color   = "#00ff88" if p95 <= 100 else ("#ffcc00" if p95 <= 250 else "#ff4444")
    ax.text(0.97, 0.97, stats,
            transform=ax.transAxes, fontsize=9, color=color,
            verticalalignment="top", horizontalalignment="right",
            bbox=dict(boxstyle="round,pad=0.5", facecolor="#111122",
                      edgecolor=color, alpha=0.9))
    ax.set_title(f"Result: {verdict} (p95={p95:.1f}ms, target<250ms for WSL2)",
                 fontsize=10, color=color, pad=4)

    fig.tight_layout()
    fig.savefig(out_path, bbox_inches="tight")
    plt.close(fig)
    print(f"[OK] Latency histogram saved → {out_path}")


# ── Figure 4: Session Summary (bonus) ────────────────────────────────────────

def plot_session_summary(df: pd.DataFrame, out_path: Path) -> None:
    """One-page summary combining all key metrics — use as report cover figure."""
    duration_s = df["t"].iloc[-1]
    total_rows  = len(df)
    tracking_pct = (df["status"] == "TRACKING").mean() * 100
    lost_pct     = 100 - tracking_pct

    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    fig.suptitle(
        "Week 3 Session Summary | Reformerz Healthcare\n"
        f"Duration: {duration_s:.0f}s  |  Rows: {total_rows}  |  "
        f"Tracking: {tracking_pct:.1f}%  |  Lost: {lost_pct:.1f}%",
        fontsize=12, color="white"
    )

    ax = axes[0, 0]
    ax.plot(df["t"], df["error_x"], color=COLORS["error_x"], alpha=0.8, label="error_x")
    ax.plot(df["t"], df["error_y"], color=COLORS["error_y"], alpha=0.8, label="error_y")
    ax.axhline(0, color="white", lw=0.6, linestyle="--", alpha=0.4)
    ax.set_title("Tracking Error"); ax.set_ylabel("Norm. error"); ax.legend(fontsize=8)
    _add_status_shading(ax, df)

    ax = axes[0, 1]
    ax.plot(df["t"], df["cmd_vx"],  color=COLORS["vx"],  alpha=0.8, label="vx")
    ax.plot(df["t"], df["cmd_yaw"], color=COLORS["yaw"], alpha=0.8, label="yaw")
    ax.axhline(0, color="white", lw=0.6, linestyle="--", alpha=0.4)
    ax.set_title("Control Output"); ax.set_ylabel("m/s  |  rad/s"); ax.legend(fontsize=8)
    _add_status_shading(ax, df)

    ax = axes[1, 0]
    labels = ["TRACKING", "LOST"]
    sizes  = [tracking_pct, lost_pct]
    colors = [COLORS["tracking"], COLORS["lost"]]
    ax.pie(sizes, labels=labels, colors=colors, autopct="%1.1f%%",
           textprops={"color": "white"}, startangle=90)
    ax.set_title("Status Distribution")

    ax = axes[1, 1]
    abs_err = np.sqrt(df["error_x"]**2 + df["error_y"]**2)
    ax.plot(df["t"], abs_err, color="#aa88ff", alpha=0.85)
    ax.axhline(abs_err.mean(), color="white", lw=0.8, linestyle="--",
               label=f"mean={abs_err.mean():.3f}")
    ax.set_title("Euclidean Error Magnitude")
    ax.set_ylabel("|error|"); ax.set_xlabel("Time (s)"); ax.legend(fontsize=8)
    _add_status_shading(ax, df)

    fig.tight_layout(rect=[0, 0, 1, 0.94])
    fig.savefig(out_path, bbox_inches="tight")
    plt.close(fig)
    print(f"[OK] Session summary saved → {out_path}")


# ── Main ──────────────────────────────────────────────────────────────────────

def _find_latest_csv() -> Path | None:
    log_dir = Path.home() / "ros2_ws" / "logs"
    csvs = sorted(log_dir.glob("session_*.csv"), key=lambda p: p.stat().st_mtime)
    return csvs[-1] if csvs else None


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate Week 3 analysis plots")
    parser.add_argument("--csv",     type=Path, default=None,
                        help="Path to session_data.csv (default: latest in ~/ros2_ws/logs/)")
    parser.add_argument("--latency", type=Path, default=None,
                        help="Path to latency CSV with a 'latency_ms' column")
    parser.add_argument("--compare", type=Path, default=None,
                        help="Path to a second session_data.csv for comparison (e.g. P-only baseline)")
    args = parser.parse_args()

    # ── Load session CSV ─────────────────────────────────────────────────────
    csv_path = args.csv or _find_latest_csv()
    if csv_path is None or not csv_path.exists():
        print("[ERROR] No session CSV found. Run logger_node first, then re-run this script.")
        sys.exit(1)

    print(f"\n[INFO] Loading {csv_path}")
    df = _load_csv(csv_path)
    print(f"[INFO] {len(df)} rows, {df['t'].iloc[-1]:.1f}s session")

    stem   = csv_path.stem
    outdir = csv_path.parent

    # ── Print quick terminal stats ───────────────────────────────────────────
    tracking_pct = (df["status"] == "TRACKING").mean() * 100
    print(f"\n  Duration     : {df['t'].iloc[-1]:.1f} s")
    print(f"  Tracking     : {tracking_pct:.1f}%")
    print(f"  error_x mean : {df['error_x'].mean():.4f}  σ={df['error_x'].std():.4f}")
    print(f"  error_y mean : {df['error_y'].mean():.4f}  σ={df['error_y'].std():.4f}")
    print(f"  max |cmd_vx| : {df['cmd_vx'].abs().max():.3f} m/s")
    print(f"  max |cmd_yaw|: {df['cmd_yaw'].abs().max():.3f} rad/s")

    # ── Comparison Logic ─────────────────────────────────────────────────────
    if args.compare and args.compare.exists():
        print(f"[INFO] Comparing with {args.compare}")
        df_comp = _load_csv(args.compare)
        
        # We'll create a special comparison plot
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
        fig.suptitle(f"Engineering Validation: P-Controller vs PID Under Stress\n{stem} vs {args.compare.stem}", 
                     fontsize=14, color="white")
        
        # Error X Comparison
        ax1.plot(df_comp["t"], df_comp["error_x"], color="#888", alpha=0.5, label="P-Only Error X")
        ax1.plot(df["t"], df["error_x"], color=COLORS["error_x"], label="PID Error X")
        ax1.set_ylabel("Error X"); ax1.legend()
        
        # Error Y Comparison (The Bias correction is most visible here)
        ax2.plot(df_comp["t"], df_comp["error_y"], color="#888", alpha=0.5, label="P-Only Error Y")
        ax2.plot(df["t"], df["error_y"], color=COLORS["error_y"], label="PID Error Y")
        ax2.set_ylabel("Error Y"); ax2.set_xlabel("Time (s)"); ax2.legend()
        
        comp_path = outdir / f"{stem}_vs_baseline.png"
        fig.tight_layout(rect=[0, 0, 1, 0.95])
        fig.savefig(comp_path)
        plt.close(fig)
        print(f"[OK] Comparison plot saved → {comp_path}")

    # ── Generate plots ───────────────────────────────────────────────────────
    print()
    plot_stability(df,         outdir / f"{stem}_stability.png")
    plot_control_response(df,  outdir / f"{stem}_control_response.png")
    plot_session_summary(df,   outdir / f"{stem}_summary.png")

    # ── Latency histogram (from external CSV or session-derived) ─────────────
    if args.latency and args.latency.exists():
        lat_df = pd.read_csv(args.latency)
        if "latency_ms" in lat_df.columns:
            samples = lat_df["latency_ms"].dropna().tolist()
        elif "e2e_ms" in lat_df.columns:
            samples = lat_df["e2e_ms"].dropna().tolist()
        else:
            print(f"[WARN] --latency CSV has no 'latency_ms' column. Columns: {list(lat_df.columns)}")
            samples = []
        if samples:
            plot_latency_histogram(
                samples,
                outdir / f"{stem}_latency_histogram.png",
                title_suffix=" (from latency_validator)"
            )
    else:
        # Derive a pseudo-latency from the timestamp spacing as a fallback
        print("[INFO] No --latency file provided. Generating latency proxy from cmd_vel timestamps.")
        dt_ms = df["timestamp_unix"].diff().dropna() * 1000
        dt_ms = dt_ms[(dt_ms > 0) & (dt_ms < 500)].tolist()
        plot_latency_histogram(
            dt_ms,
            outdir / f"{stem}_latency_histogram.png",
            title_suffix=" (control-loop interval proxy)"
        )

    print(f"\n[DONE] All plots saved to {outdir}/")
    print("       Copy these PNG files into your report.\n")


if __name__ == "__main__":
    main()
