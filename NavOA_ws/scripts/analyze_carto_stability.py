#!/usr/bin/env python3.8
"""
analyze_carto_stability.py — Isolate cartographer (map→odom TF) stability.

The map→odom transform is cartographer's output: it shifts the map frame to absorb
the drift of the odometry source. A *stable* cartographer moves map→odom slowly and
smoothly (tracking odom drift). A jump in map→odom = the map frame teleporting =
scan not matching map / localization lost-and-found.

This script reads map→odom TFs at their native rate (no interpolation, so jumps are
not smeared) and produces:

  1. map→odom X/Y/yaw over time, jump locations marked
  2. per-sample change |Δpos| and |Δyaw| (THE instability signal), with threshold
  3. histogram of per-sample |Δpos| (how "jumpy" cartographer is, vs smooth drift)

Usage:
  python3.8 scripts/analyze_carto_stability.py <bag_path> [--output-dir <dir>]
"""

import argparse
import os
import sys
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

try:
    import rosbag
except ImportError:
    print("ERROR: rosbag module not found. Source your ROS setup.bash first.")
    sys.exit(1)


TOPIC_TF = "/tf"
JUMP_DIST_M = 0.10
JUMP_YAW_DEG = 5.0


def quat_to_yaw(q_xyzw):
    x, y, z, w = q_xyzw
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def shortest_angle(delta):
    return math.atan2(math.sin(delta), math.cos(delta))


def extract_map_to_odom(bag):
    """Extract map→odom TFs at native rate. Returns arrays t, x, y, yaw."""
    recs = []
    for _topic, msg, t in bag.read_messages(topics=[TOPIC_TF]):
        for transform in msg.transforms:
            if transform.header.frame_id == "map" and transform.child_frame_id == "odom":
                secs = transform.header.stamp.to_sec()
                if secs < 1e-6:
                    secs = t.to_sec()
                tx = transform.transform.translation.x
                ty = transform.transform.translation.y
                qx = transform.transform.rotation.x
                qy = transform.transform.rotation.y
                qz = transform.transform.rotation.z
                qw = transform.transform.rotation.w
                yaw = quat_to_yaw(np.array([qx, qy, qz, qw]))
                recs.append((secs, tx, ty, yaw))
    recs.sort(key=lambda r: r[0])
    t = np.array([r[0] for r in recs])
    x = np.array([r[1] for r in recs])
    y = np.array([r[2] for r in recs])
    yaw = np.array([r[3] for r in recs])
    return t, x, y, yaw


def analyze(bag_path, output_dir):
    os.makedirs(output_dir, exist_ok=True)
    bag = rosbag.Bag(bag_path, "r")
    t, x, y, yaw = extract_map_to_odom(bag)
    bag.close()

    if len(t) < 2:
        print("ERROR: fewer than 2 map→odom TFs found.")
        return

    # Drop duplicate timestamps (identical-stamp TFs) to avoid dt=0
    keep = np.concatenate(([True], np.diff(t) > 1e-6))
    t, x, y, yaw = t[keep], x[keep], y[keep], yaw[keep]

    t_rel = t - t[0]
    dt = np.diff(t)

    # Per-sample change of the map→odom transform (native rate)
    dx = np.diff(x)
    dy = np.diff(y)
    dpos = np.sqrt(dx ** 2 + dy ** 2)
    dyaw = np.array([abs(shortest_angle(yaw[i] - yaw[i - 1]))
                     for i in range(1, len(yaw))])

    jump_mask = (dpos > JUMP_DIST_M) | (dyaw > math.radians(JUMP_YAW_DEG))
    jump_idx = np.where(jump_mask)[0] + 1  # index into x/y/t arrays

    # ---- metrics ----
    n = len(t)
    dur = t[-1] - t[0]
    rate = n / dur if dur > 0 else float("nan")
    n_jumps = int(jump_mask.sum())
    total_var = float(dpos.sum())
    rms_step = float(np.sqrt(np.mean(dpos ** 2))) if len(dpos) else 0.0
    med_step = float(np.median(dpos)) if len(dpos) else 0.0

    # Smooth-vs-jumpy split
    smooth = dpos[~jump_mask]
    frac_jumpy = n_jumps / len(dpos) * 100 if len(dpos) else 0.0

    # Top-5 jumps
    order = np.argsort(dpos)[::-1][:5]
    top5 = [(t_rel[i + 1], dpos[i], math.degrees(dyaw[i])) for i in order]

    # ---- plotting ----
    plt.rcParams.update({
        "figure.dpi": 150, "savefig.dpi": 150, "savefig.bbox": "tight",
        "font.family": "sans-serif", "font.size": 10,
    })
    C_X = "#0072B2"
    C_Y = "#009E73"
    C_YAW = "#CC79A7"
    C_D = "#D55E00"

    fig, (axA, axB, axC) = plt.subplots(3, 1, figsize=(16, 13), sharex=False,
                                        gridspec_kw={"height_ratios": [2, 2, 1]})

    # --- Panel A: map→odom TF over time ---
    axA.plot(t_rel, x, color=C_X, linewidth=1.2, label="map→odom X")
    axA.plot(t_rel, y, color=C_Y, linewidth=1.2, label="map→odom Y")
    axA.set_ylabel("map→odom position [m]")
    axA.set_title("Cartographer map→odom TF (native rate) — jumps = map frame teleporting")
    axA.grid(True, alpha=0.3)

    axA_twin = axA.twinx()
    axA_twin.plot(t_rel, np.degrees(np.unwrap(yaw)), color=C_YAW, linewidth=1.0,
                  linestyle="--", alpha=0.7, label="Yaw [°]")
    axA_twin.set_ylabel("Yaw [deg]", color=C_YAW)
    axA_twin.tick_params(axis="y", labelcolor=C_YAW)

    for i in jump_idx:
        axA.axvline(t_rel[i], color="red", linewidth=0.7, alpha=0.4)
    if n_jumps > 0:
        axA.axvline(t_rel[jump_idx[0]], color="red", linewidth=0.8, alpha=0.7,
                    label=f"Jump ({n_jumps} total)")
    axA.legend(loc="upper left", framealpha=0.9, fontsize=8)

    # --- Panel B: per-sample change (the instability signal) ---
    ts_mid = t_rel[1:]  # dpos[i] is the change from sample i to i+1
    axB.plot(ts_mid, dpos, color=C_D, linewidth=1.0, label="|Δ position| per sample")
    axB.axhline(JUMP_DIST_M, color="red", linewidth=1.0, linestyle="--",
                label=f"Jump threshold ({JUMP_DIST_M} m)")
    axB.set_ylabel("|Δ map→odom| [m/sample]")
    axB.set_title("Per-sample change of map→odom — spikes = cartographer instability")
    axB.grid(True, alpha=0.3)
    axB.set_ylim(bottom=0)

    axB_twin = axB.twinx()
    axB_twin.plot(ts_mid, np.degrees(dyaw), color=C_YAW, linewidth=0.8,
                  linestyle=":", alpha=0.6, label="|Δ yaw| [°/sample]")
    axB_twin.set_ylabel("|Δ yaw| [°/sample]", color=C_YAW)
    axB_twin.tick_params(axis="y", labelcolor=C_YAW)
    axB_twin.set_ylim(bottom=0)

    axB.legend(loc="upper left", framealpha=0.9, fontsize=8)
    axB_twin.legend(loc="upper right", framealpha=0.9, fontsize=7)

    # --- Panel C: histogram of per-sample |Δpos| ---
    bins = np.logspace(-4, 1, 60)
    axC.hist(dpos, bins=bins, color=C_D, alpha=0.8, edgecolor="white", linewidth=0.3)
    axC.axvline(JUMP_DIST_M, color="red", linewidth=1.2, linestyle="--")
    axC.set_xscale("log")
    axC.set_xlabel("|Δ map→odom| per sample [m] (log)")
    axC.set_ylabel("Count")
    axC.set_title(f"Distribution of per-sample map→odom change "
                  f"(median={med_step*1e3:.1f} mm, {frac_jumpy:.1f}% of samples are jumps)")
    axC.grid(True, alpha=0.3, which="both")

    fig.tight_layout()
    out_png = os.path.join(output_dir, "nav_carto_stability.png")
    fig.savefig(out_png)
    plt.close(fig)
    print(f"  Saved: {out_png}")

    # ---- summary ----
    print("=" * 60)
    print("  CARTOGRAPHER (map→odom TF) STABILITY")
    print("=" * 60)
    print(f"  Bag                 : {bag_path}")
    print(f"  Duration            : {dur:.1f} s")
    print(f"  map→odom samples    : {n}  (native rate ≈ {rate:.1f} Hz)")
    print(f"  Jumps               : {n_jumps}  ({frac_jumpy:.1f}% of samples)")
    print(f"  Total variation     : {total_var:.2f} m (sum of |Δ|)")
    print(f"  Median |Δ|/sample   : {med_step*1e3:.1f} mm  (normal drift-tracking)")
    print(f"  RMS |Δ|/sample      : {rms_step*1e3:.1f} mm")
    print("  Top-5 jumps (t | Δpos | Δyaw):")
    for tt, dd, yy in top5:
        print(f"    t={tt:7.1f}s | {dd:6.3f} m | {yy:6.1f}°")
    print("=" * 60)


def main():
    parser = argparse.ArgumentParser(description="Cartographer map→odom stability analysis.")
    parser.add_argument("bag_path", help="Path to the .bag file")
    parser.add_argument("--output-dir", "-o", default="./analysis_carto",
                        help="Output directory (default: ./analysis_carto)")
    args = parser.parse_args()

    if not os.path.isfile(args.bag_path):
        print(f"ERROR: bag file not found: {args.bag_path}")
        sys.exit(1)

    analyze(args.bag_path, args.output_dir)


if __name__ == "__main__":
    main()
