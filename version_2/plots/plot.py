#!/usr/bin/env python3
"""
plot.py — Week 4 Analysis Plots
=================================
Reformerz Healthcare | Drone Tracking Pipeline v2

Generates 4 publication-quality figures from session CSVs:

  Fig 1 — Error vs Time         : error_x, error_y, error_mag  (3 sub-axes)
  Fig 2 — Velocity vs Time      : vx, vy, yaw_rate             (3 sub-axes)
  Fig 3 — State Transitions     : numeric FSM state over time
  Fig 4 — Baseline Comparison   : PID v2 vs P-ctrl mean error + RMS side-by-side
                                  (only generated when --baseline is provided)

Usage
-----
  # Single session:
  python3 plot.py --csv ~/ros2_ws/logs/session_v2_20260418.csv

  # With baseline comparison:
  python3 plot.py \\
      --csv      ~/ros2_ws/logs/session_v2_20260418.csv \\
      --baseline ~/ros2_ws/logs/session_20260418_151251.csv

Outputs are saved next to the input CSV.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
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
    "axes.titlesize":   11,
    "axes.titlecolor":  "#ffffff",
    "axes.grid":        True,
    "grid.color":       "#333355",
    "grid.linewidth":   0.6,
    "xtick.color":      "#aaa",
    "ytick.color":      "#aaa",
    "text.color":       "#e0e0e0",
    "legend.facecolor": "#1a1a2e",
    "legend.edgecolor": "#555",
    "lines.linewidth":  1.5,
})

C = {
    "ex":       "#00d4ff",
    "ey":       "#ff6b35",
    "emag":     "#aa88ff",
    "vx":       "#00ff88",
    "vy":       "#ffcc00",
    "yaw":      "#ff4488",
    "track":    "#00ff88",
    "search":   "#ffcc00",
    "hold":     "#ff8800",
    "failsafe": "#ff4444",
    "lost":     "#ff4444",
    "baseline": "#888888",
    "v2":       "#00d4ff",
}

STATE_MAP  = {"TRACK": 3, "TRACKING": 3, "SEARCH": 2, "HOLD": 1, "FAILSAFE": 0}
STATE_TICK = {0: "FAILSAFE", 1: "HOLD", 2: "SEARCH", 3: "TRACK"}
STATE_COL  = {3: C["track"], 2: C["search"], 1: C["hold"], 0: C["failsafe"]}


# ── Loader (mirrors metrics.py) ───────────────────────────────────────────────

def _load(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    if "elapsed_s" not in df.columns:
        if "timestamp_unix" in df.columns:
            df["elapsed_s"] = df["timestamp_unix"] - df["timestamp_unix"].iloc[0]
        else:
            df["elapsed_s"] = np.arange(len(df)) * 0.1
    if "error_mag" not in df.columns:
        df["error_mag"] = np.sqrt(df["error_x"] ** 2 + df["error_y"] ** 2)
    # normalise velocity column names (Week 3 uses cmd_vx/cmd_vy/cmd_yaw)
    for old, new in [("cmd_vx","vx"),("cmd_vy","vy"),("cmd_yaw","yaw_rate")]:
        if old in df.columns and new not in df.columns:
            df[new] = df[old]
    # normalise status column
    for alias in ["detection_status", "status"]:
        if alias in df.columns and "detection_status" not in df.columns:
            df["detection_status"] = df[alias]
    if "system_state" not in df.columns:
        df["system_state"] = df.get("detection_status", pd.Series(
            ["HOLD"] * len(df))).map({"TRACKING": "TRACK", "LOST": "HOLD"}).fillna("HOLD")
    df["state_num"] = df["system_state"].map(STATE_MAP).fillna(1)
    return df


# ── Status shading ────────────────────────────────────────────────────────────

def _shade_lost(ax, df: pd.DataFrame) -> None:
    col = "detection_status"
    if col not in df.columns:
        return
    in_block = False
    t0 = 0.0
    for _, row in df.iterrows():
        if row[col] in ("LOST", "HOLD") and not in_block:
            t0 = row["elapsed_s"]; in_block = True
        elif row[col] not in ("LOST", "HOLD") and in_block:
            ax.axvspan(t0, row["elapsed_s"], alpha=0.12,
                       color=C["lost"], zorder=0)
            in_block = False
    if in_block:
        ax.axvspan(t0, df["elapsed_s"].iloc[-1], alpha=0.12,
                   color=C["lost"], zorder=0)


def _stat_box(ax, series: pd.Series, color: str, label: str) -> None:
    mu = series.mean(); sigma = series.std()
    ax.text(0.01, 0.97, f"{label}: mean={mu:.3f}  std={sigma:.3f}",
            transform=ax.transAxes, fontsize=8, color=color,
            va="top", bbox=dict(boxstyle="round,pad=0.25",
                                facecolor="#111122", edgecolor=color, alpha=0.8))


# ── Figure 1: Error vs Time ───────────────────────────────────────────────────

def plot_error(df: pd.DataFrame, out: Path, title_tag: str = "") -> None:
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
    fig.suptitle(
        f"Tracking Error vs Time{title_tag}\nReformerz Healthcare | Week 4 PID v2",
        fontsize=13, color="white", y=0.99)

    ax1.plot(df["elapsed_s"], df["error_x"], color=C["ex"], alpha=0.9)
    ax1.axhline(0, color="white", lw=0.7, linestyle="--", alpha=0.3)
    ax1.set_ylabel("error_x (norm)", fontsize=9)
    ax1.set_ylim(-1.3, 1.3)
    _shade_lost(ax1, df); _stat_box(ax1, df["error_x"], C["ex"], "error_x")
    ax1.legend([mpatches.Patch(color=C["ex"])], ["error_x (horiz / yaw)"],
               fontsize=8, loc="upper right")

    ax2.plot(df["elapsed_s"], df["error_y"], color=C["ey"], alpha=0.9)
    ax2.axhline(0, color="white", lw=0.7, linestyle="--", alpha=0.3)
    ax2.set_ylabel("error_y (norm)", fontsize=9)
    ax2.set_ylim(-1.3, 1.3)
    _shade_lost(ax2, df); _stat_box(ax2, df["error_y"], C["ey"], "error_y")
    ax2.legend([mpatches.Patch(color=C["ey"])], ["error_y (fwd)"],
               fontsize=8, loc="upper right")

    ax3.plot(df["elapsed_s"], df["error_mag"], color=C["emag"], alpha=0.9)
    ax3.axhline(df["error_mag"].mean(), color="white", lw=0.9,
                linestyle="--", alpha=0.5, label=f"mean={df['error_mag'].mean():.3f}")
    ax3.set_ylabel("|error|", fontsize=9)
    ax3.set_xlabel("Time (s)", fontsize=9)
    ax3.set_ylim(0, 1.5)
    _shade_lost(ax3, df); _stat_box(ax3, df["error_mag"], C["emag"], "|error|")
    ax3.legend(fontsize=8, loc="upper right")

    lost_patch = mpatches.Patch(color=C["lost"], alpha=0.3, label="LOST / HOLD")
    fig.legend(handles=[lost_patch], loc="lower right", fontsize=8, framealpha=0.4)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)
    print(f"[OK] Error plot -> {out}")


# ── Figure 2: Velocity vs Time ────────────────────────────────────────────────

def plot_velocity(df: pd.DataFrame, out: Path, title_tag: str = "") -> None:
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
    fig.suptitle(
        f"Velocity Commands vs Time{title_tag}\nReformerz Healthcare | Week 4 PID v2",
        fontsize=13, color="white", y=0.99)

    for ax, col, color, label, limit in [
        (ax1, "vx",       C["vx"],  "vx (forward m/s)",   0.5),
        (ax2, "vy",       C["vy"],  "vy (lateral m/s)",   0.3),
        (ax3, "yaw_rate", C["yaw"], "yaw_rate (rad/s)",   0.6),
    ]:
        if col not in df.columns:
            continue
        ax.plot(df["elapsed_s"], df[col], color=color, alpha=0.9)
        ax.axhline(0, color="white", lw=0.6, linestyle="--", alpha=0.3)
        ax.axhline( limit, color="#888", lw=0.8, linestyle=":", alpha=0.4)
        ax.axhline(-limit, color="#888", lw=0.8, linestyle=":", alpha=0.4)
        ax.set_ylabel(label, fontsize=9)
        _shade_lost(ax, df)
        _stat_box(ax, df[col], color, col)
        ax.legend([mpatches.Patch(color=color)], [label], fontsize=8, loc="upper right")

    ax3.set_xlabel("Time (s)", fontsize=9)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)
    print(f"[OK] Velocity plot -> {out}")


# ── Figure 3: State Transitions ───────────────────────────────────────────────

def plot_states(df: pd.DataFrame, out: Path, title_tag: str = "") -> None:
    fig, ax = plt.subplots(figsize=(13, 5))
    fig.suptitle(
        f"System State Transitions{title_tag}\nReformerz Healthcare | Week 4 PID v2",
        fontsize=13, color="white")

    t = df["elapsed_s"].values
    s = df["state_num"].values

    # Fill between state steps
    for i in range(len(t) - 1):
        state_val = int(s[i])
        color = STATE_COL.get(state_val, "#888888")
        ax.fill_between([t[i], t[i+1]], 0, s[i],
                        color=color, alpha=0.35, step="post")

    ax.step(t, s, where="post", color="white", lw=0.8, alpha=0.6)
    ax.set_yticks(list(STATE_TICK.keys()))
    ax.set_yticklabels([STATE_TICK[k] for k in STATE_TICK.keys()], fontsize=9)
    ax.set_ylim(-0.3, 3.6)
    ax.set_xlabel("Time (s)", fontsize=10)
    ax.set_ylabel("System State", fontsize=10)

    # Legend
    handles = [mpatches.Patch(color=STATE_COL[k], alpha=0.6, label=STATE_TICK[k])
               for k in sorted(STATE_TICK.keys(), reverse=True)]
    ax.legend(handles=handles, fontsize=9, loc="upper right")

    # Annotate state counts
    for k, label in STATE_TICK.items():
        pct = (df["state_num"] == k).mean() * 100
        ax.text(t[-1] * 1.01, k, f"{pct:.1f}%", va="center",
                fontsize=8, color=STATE_COL[k])

    fig.tight_layout()
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)
    print(f"[OK] State transition plot -> {out}")


# ── Figure 4: Baseline Comparison ─────────────────────────────────────────────

def plot_comparison(df_b: pd.DataFrame, df_v2: pd.DataFrame, out: Path) -> None:
    """Bar chart comparing key metrics: Week 3 P-ctrl vs Week 4 PID."""
    metrics_b  = {
        "Mean\nError":    df_b["error_mag"].mean(),
        "RMS\nError":     float(np.sqrt((df_b["error_mag"]**2).mean())),
        "vx σ\n(m/s)":   df_b["vx"].std() if "vx" in df_b else float("nan"),
    }
    metrics_v2 = {
        "Mean\nError":    df_v2["error_mag"].mean(),
        "RMS\nError":     float(np.sqrt((df_v2["error_mag"]**2).mean())),
        "vx σ\n(m/s)":   df_v2["vx"].std() if "vx" in df_v2 else float("nan"),
    }

    labels = list(metrics_b.keys())
    vals_b  = [metrics_b[l] for l in labels]
    vals_v2 = [metrics_v2[l] for l in labels]

    x = np.arange(len(labels))
    w = 0.34

    fig, ax = plt.subplots(figsize=(10, 6))
    fig.suptitle(
        "Week 3 P-Controller vs Week 4 PID — Metric Comparison\n"
        "Reformerz Healthcare | Validation Report",
        fontsize=13, color="white")

    bars_b  = ax.bar(x - w/2, vals_b,  w, label="W3 Baseline (P-ctrl)",
                     color=C["baseline"], alpha=0.8, edgecolor="#555")
    bars_v2 = ax.bar(x + w/2, vals_v2, w, label="W4 PID v2",
                     color=C["v2"],       alpha=0.9, edgecolor="#005577")

    # Value labels on bars
    for bar in bars_b:
        h = bar.get_height()
        ax.text(bar.get_x() + bar.get_width()/2, h + 0.002, f"{h:.4f}",
                ha="center", va="bottom", fontsize=8, color=C["baseline"])
    for bar in bars_v2:
        h = bar.get_height()
        ax.text(bar.get_x() + bar.get_width()/2, h + 0.002, f"{h:.4f}",
                ha="center", va="bottom", fontsize=8, color=C["v2"])

    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=11)
    ax.set_ylabel("Value", fontsize=10)
    ax.legend(fontsize=10)

    # Improvement annotations
    for i, (vb, vv) in enumerate(zip(vals_b, vals_v2)):
        if not (np.isnan(vb) or np.isnan(vv) or vb == 0):
            pct = (vb - vv) / vb * 100
            color = "#00ff88" if pct > 0 else "#ff4444"
            ax.text(x[i], max(vb, vv) + 0.01, f"{pct:+.1f}%",
                    ha="center", fontsize=9, color=color, fontweight="bold")

    fig.tight_layout()
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)
    print(f"[OK] Comparison plot -> {out}")


# ── Multi-run repeatability plot ──────────────────────────────────────────────

def plot_repeatability(csv_paths: list[Path], out: Path) -> None:
    """Overlay error_mag for multiple runs to prove repeatability."""
    fig, ax = plt.subplots(figsize=(13, 6))
    fig.suptitle(
        f"Repeatability — {len(csv_paths)} Runs Overlaid\n"
        "Reformerz Healthcare | Week 4 PID v2",
        fontsize=13, color="white")

    palette = ["#00d4ff","#ff6b35","#00ff88","#ffcc00","#ff4488",
               "#aa88ff","#38bdf8","#fb923c","#34d399","#f472b6"]

    means = []
    for i, path in enumerate(csv_paths):
        df = _load(path)
        color = palette[i % len(palette)]
        ax.plot(df["elapsed_s"], df["error_mag"],
                color=color, alpha=0.55, lw=1.1,
                label=f"Run {i+1}  mean={df['error_mag'].mean():.3f}")
        means.append(df["error_mag"].mean())

    overall_mean = float(np.mean(means))
    ax.axhline(overall_mean, color="white", lw=1.5, linestyle="--",
               label=f"Overall mean={overall_mean:.3f}")

    ax.set_xlabel("Time (s)", fontsize=10)
    ax.set_ylabel("|error| (normalised)", fontsize=10)
    ax.legend(fontsize=8, loc="upper right")

    # Consistency annotation
    std_of_means = float(np.std(means))
    ax.text(0.01, 0.03,
            f"Run-to-run consistency: mean of means={overall_mean:.4f}  "
            f"std of means={std_of_means:.4f}",
            transform=ax.transAxes, fontsize=8, color="#ffcc00",
            bbox=dict(boxstyle="round,pad=0.3", facecolor="#111122",
                      edgecolor="#ffcc00", alpha=0.8))

    fig.tight_layout()
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)
    print(f"[OK] Repeatability plot -> {out}")


# ── CLI ───────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="Week 4 analysis plots")
    parser.add_argument("--csv",           type=Path, required=True,
                        help="Primary (v2) session CSV")
    parser.add_argument("--baseline",      type=Path, default=None,
                        help="Week 3 baseline CSV for comparison plot")
    parser.add_argument("--runs",          type=Path, nargs="*", default=None,
                        help="Multiple CSVs for repeatability overlay")
    parser.add_argument("--tag",           type=str, default="",
                        help="Optional title suffix (e.g. scenario name)")
    args = parser.parse_args()

    df = _load(args.csv)
    stem   = args.csv.stem
    outdir = args.csv.parent

    tag = f" — {args.tag}" if args.tag else ""

    print(f"\n[INFO] Loaded {len(df)} rows  {df['elapsed_s'].iloc[-1]:.1f}s")
    plot_error(df,    outdir / f"{stem}_error.png",    tag)
    plot_velocity(df, outdir / f"{stem}_velocity.png", tag)
    plot_states(df,   outdir / f"{stem}_states.png",   tag)

    if args.baseline:
        df_b = _load(args.baseline)
        plot_comparison(df_b, df, outdir / f"{stem}_comparison.png")

    if args.runs:
        all_csv = [args.csv] + list(args.runs)
        plot_repeatability(all_csv, outdir / f"{stem}_repeatability.png")

    print(f"\n[DONE] Plots saved to {outdir}/")


if __name__ == "__main__":
    main()
