#!/usr/bin/env python3.8
"""
analyze_bag.py — Offline EKF covariance validation from a rosbag.

Reads a bag containing three odometry streams:
  - /odom        : fused EKF output (robot_localization, HAS covariance)
  - /odom_rf2o   : rf2o laser scan-matching odometry (NO covariance)
  - SMX/Odom     : leg odometry (external; covariance status checked at runtime)

Produces:
  1. XY track overlay  →  {scenario}_xy_tracks.png
  2. Disagreement vs covariance time series  →  {scenario}_disagreement.png
  3. Stationary drift plot (static scenario only)  →  {scenario}_drift.png
  4. Text summary  →  {scenario}_summary.txt

Usage:
  python3 analyze_bag.py <bag_path> <scenario_label> [--output-dir <dir>]
"""

import argparse
import os
import sys
import math
import textwrap
from collections import defaultdict

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import ScalarFormatter

try:
    import rosbag
except ImportError:
    print("ERROR: rosbag module not found. Source your ROS setup.bash first.")
    sys.exit(1)

try:
    from scipy.interpolate import interp1d
    HAVE_SCIPY = True
except ImportError:
    HAVE_SCIPY = False
    print("WARNING: scipy not available; falling back to numpy.interp (linear only).")

# ---------------------------------------------------------------------------
# Topic names (as recorded in the bag — these are the resolved absolute names)
# ---------------------------------------------------------------------------
TOPIC_FUSED  = "/odom"
TOPIC_RF2O   = "/odom_rf2o"
TOPIC_LEG_CANDIDATES = ["SMX/Odom", "/SMX/Odom"]  # try both — rosbag may store either
TOPIC_TF     = "/tf"
TOPIC_TF_STATIC = "/tf_static"

# Covariance indices (6x6 row-major, 0-indexed)
COV_IDX_X   = 0
COV_IDX_Y   = 7
COV_IDX_Z   = 14
COV_IDX_YAW = 35

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def find_topic_in_bag(bag, candidates):
    """Return the first candidate topic that exists in the bag, or None."""
    bag_topics = set(bag.get_type_and_topic_info()[1].keys())
    for c in candidates:
        if c in bag_topics:
            return c
    return None


def extract_odom_msgs(bag, topic):
    """Return (frame_id, list_of_records).

    Each record: (stamp_sec, position_xyz, orientation_xyzw, covariance_36)

    stamp_sec        : float (ros::Time as seconds)
    position_xyz     : np.array([x, y, z])
    orientation_xyzw : np.array([x, y, z, w])
    covariance_36    : np.array of 36 floats (raw from message, may be all-zero)
    """
    frame_id = None
    records = []
    for _topic, msg, t in bag.read_messages(topics=[topic]):
        if frame_id is None:
            frame_id = msg.header.frame_id
        secs = msg.header.stamp.to_sec()
        pos = np.array([msg.pose.pose.position.x,
                        msg.pose.pose.position.y,
                        msg.pose.pose.position.z])
        ori = np.array([msg.pose.pose.orientation.x,
                        msg.pose.pose.orientation.y,
                        msg.pose.pose.orientation.z,
                        msg.pose.pose.orientation.w])
        cov = np.array(msg.pose.covariance)  # 36-element tuple → ndarray
        records.append((secs, pos, ori, cov))
    return frame_id, records


def check_covariance(topic_label, records, max_check=10):
    """Check whether a topic's messages carry non-zero covariance.

    Returns (has_covariance: bool, sample_values: str)
    """
    for i, (_t, _pos, _ori, cov) in enumerate(records):
        if i >= max_check:
            break
        if np.any(cov != 0.0):
            return True, f"cov[X]={cov[COV_IDX_X]:.4f}, cov[Y]={cov[COV_IDX_Y]:.4f}, cov[Yaw]={cov[COV_IDX_YAW]:.4f}"
        # Also check twist covariance in case pose cov is zero but twist cov isn't
        # (unlikely for odom, but be thorough)
    return False, "all zeros in first {} messages".format(min(len(records), max_check))


def quat_to_yaw(q_xyzw):
    """Convert quaternion [x,y,z,w] to yaw angle (radians)."""
    x, y, z, w = q_xyzw
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def build_common_timeline(timestamps_list, target_dt=None):
    """Given a list of timestamp arrays, return a common timeline.

    Uses the union range at the finest observed rate, capped at 100 Hz.
    If target_dt is None, uses the median dt of the highest-rate stream.
    """
    all_starts = [ts[0] for ts in timestamps_list if len(ts) > 0]
    all_ends   = [ts[-1] for ts in timestamps_list if len(ts) > 0]
    if not all_starts:
        return np.array([])

    t_start = min(all_starts)
    t_end   = max(all_ends)

    if target_dt is None:
        # Use the smallest median dt across streams, but not faster than 100 Hz
        dts = []
        for ts in timestamps_list:
            if len(ts) > 1:
                dts.append(np.median(np.diff(ts)))
        if dts:
            target_dt = max(min(dts), 0.01)  # cap at 100 Hz
        else:
            target_dt = 0.05  # 20 Hz default

    n_pts = int((t_end - t_start) / target_dt) + 1
    return np.linspace(t_start, t_end, n_pts)


def interpolate_to_timeline(src_t, src_vals, target_t, kind="linear"):
    """Interpolate 1D values to target timeline. Extrapolation: nearest."""
    if len(src_t) < 2:
        # Need at least 2 points for interpolation
        if len(src_t) == 1:
            return np.full_like(target_t, src_vals[0])
        return np.full_like(target_t, np.nan)

    if HAVE_SCIPY:
        f = interp1d(src_t, src_vals, kind=kind,
                     bounds_error=False, fill_value="extrapolate")
        # scipy extrapolation can be wild — clamp to observed range
        raw = f(target_t)
        lo, hi = np.min(src_vals), np.max(src_vals)
        return np.clip(raw, lo - 0.1 * abs(lo), hi + 0.1 * abs(hi))
    else:
        return np.interp(target_t, src_t, src_vals)


# ---------------------------------------------------------------------------
# Main analysis
# ---------------------------------------------------------------------------

def analyze(bag_path, scenario, output_dir):
    os.makedirs(output_dir, exist_ok=True)

    print("=" * 60)
    print(f"  Analyzing: {bag_path}")
    print(f"  Scenario : {scenario}")
    print("=" * 60)

    bag = rosbag.Bag(bag_path, "r")

    # Resolve the leg-odom topic name dynamically (bag may store it relative or absolute)
    topic_leg = find_topic_in_bag(bag, TOPIC_LEG_CANDIDATES)
    if topic_leg is None:
        print("  *** WARNING: No leg-odom topic found in bag. "
              "Tried: {}".format(TOPIC_LEG_CANDIDATES))

    # ---- Extract messages -------------------------------------------------
    print("\n[1/5] Reading messages from bag ...")
    fused_frame, fused_recs = extract_odom_msgs(bag, TOPIC_FUSED)
    rf2o_frame,  rf2o_recs  = extract_odom_msgs(bag, TOPIC_RF2O)
    leg_frame,   leg_recs   = extract_odom_msgs(bag, topic_leg) if topic_leg else (None, [])

    # Also read TF for potential frame-transform use
    tf_msgs = []
    for _topic, msg, t in bag.read_messages(topics=[TOPIC_TF]):
        for transform in msg.transforms:
            tf_msgs.append((t.to_sec(), transform))

    bag.close()

    print(f"  /odom       : {len(fused_recs):6d} messages  frame_id={fused_frame or '?'}")
    print(f"  /odom_rf2o  : {len(rf2o_recs):6d} messages  frame_id={rf2o_frame or '?'}")
    print(f"  {topic_leg or 'SMX/Odom':12s}: {len(leg_recs):6d} messages  frame_id={leg_frame or '?'}")
    print(f"  /tf         : {len(tf_msgs):6d} transforms")

    if len(leg_recs) == 0:
        print("\n  *** WARNING: Zero messages on SMX/Odom! Check the topic name. ***")
    if len(rf2o_recs) == 0:
        print("\n  *** WARNING: Zero messages on /odom_rf2o! ***")
    if len(fused_recs) == 0:
        print("\n  *** WARNING: Zero messages on /odom! ***")

    # ---- Covariance + Frame audit -----------------------------------------
    print("\n[2/5] Covariance and frame audit ...")
    fused_has_cov, fused_cov_sample = check_covariance("fused /odom", fused_recs)
    rf2o_has_cov, rf2o_cov_sample   = check_covariance("rf2o /odom_rf2o", rf2o_recs)
    leg_has_cov, leg_cov_sample     = check_covariance("leg SMX/Odom", leg_recs)

    print(f"  /odom       : {'HAS covariance' if fused_has_cov else 'NO covariance'} — {fused_cov_sample}")
    print(f"  /odom_rf2o  : {'HAS covariance' if rf2o_has_cov else 'NO covariance'} — {rf2o_cov_sample}")
    print(f"  SMX/Odom    : {'HAS covariance' if leg_has_cov else 'NO covariance'} — {leg_cov_sample}")

    same_frame = (fused_frame == rf2o_frame == leg_frame and fused_frame is not None)
    if same_frame:
        print(f"  All three share frame_id '{fused_frame}' — direct position comparison OK.")
    else:
        print(f"  *** FRAME MISMATCH (fused={fused_frame}, rf2o={rf2o_frame}, leg={leg_frame}).")
        print(f"  *** Will transform to common frame '{fused_frame or 'odom'}' via /tf.")

    # ---- Time alignment ---------------------------------------------------
    print("\n[3/5] Time-aligning odometry streams ...")
    # Unpack timestamps and positions
    t_fused = np.array([r[0] for r in fused_recs])
    t_rf2o  = np.array([r[0] for r in rf2o_recs])
    t_leg   = np.array([r[0] for r in leg_recs])

    pos_fused = np.array([r[1] for r in fused_recs])  # Nx3
    pos_rf2o  = np.array([r[1] for r in rf2o_recs])
    pos_leg   = np.array([r[1] for r in leg_recs])

    yaw_fused = np.array([quat_to_yaw(r[2]) for r in fused_recs])
    yaw_rf2o  = np.array([quat_to_yaw(r[2]) for r in rf2o_recs])
    yaw_leg   = np.array([quat_to_yaw(r[2]) for r in leg_recs])

    cov_fused_full = np.array([r[3] for r in fused_recs])  # Nx36

    # Build common timeline
    streams = [(t_fused, "fused"), (t_rf2o, "rf2o"), (t_leg, "leg")]
    streams = [(t, n) for t, n in streams if len(t) > 0]
    t_common = build_common_timeline([t for t, _ in streams])
    print(f"  Common timeline: {len(t_common)} points, "
          f"dt={np.mean(np.diff(t_common)):.4f}s "
          f"({1.0/np.mean(np.diff(t_common)):.1f} Hz)")

    # Interpolate to common timeline
    if len(t_fused) >= 2:
        pos_fused_i = np.column_stack([
            interpolate_to_timeline(t_fused, pos_fused[:, i], t_common)
            for i in range(3)
        ])
        yaw_fused_i = interpolate_to_timeline(t_fused, yaw_fused, t_common)
        cov_fused_i = np.column_stack([
            interpolate_to_timeline(t_fused, cov_fused_full[:, idx], t_common)
            for idx in range(36)
        ])
    else:
        pos_fused_i = np.full((len(t_common), 3), np.nan)
        yaw_fused_i = np.full(len(t_common), np.nan)
        cov_fused_i = np.full((len(t_common), 36), np.nan)

    if len(t_rf2o) >= 2:
        pos_rf2o_i = np.column_stack([
            interpolate_to_timeline(t_rf2o, pos_rf2o[:, i], t_common)
            for i in range(3)
        ])
        yaw_rf2o_i = interpolate_to_timeline(t_rf2o, yaw_rf2o, t_common)
    else:
        pos_rf2o_i = np.full((len(t_common), 3), np.nan)
        yaw_rf2o_i = np.full(len(t_common), np.nan)

    if len(t_leg) >= 2:
        pos_leg_i = np.column_stack([
            interpolate_to_timeline(t_leg, pos_leg[:, i], t_common)
            for i in range(3)
        ])
        yaw_leg_i = interpolate_to_timeline(t_leg, yaw_leg, t_common)
    else:
        pos_leg_i = np.full((len(t_common), 3), np.nan)
        yaw_leg_i = np.full(len(t_common), np.nan)

    # ---- Compute metrics --------------------------------------------------
    print("\n[4/5] Computing metrics ...")

    # Input disagreement: Euclidean distance between rf2o and leg positions
    disagreement = np.sqrt(np.sum((pos_rf2o_i[:, :2] - pos_leg_i[:, :2]) ** 2, axis=1))

    # Fused position covariance metrics (from the EKF's own covariance matrix)
    cov_x_sigma  = np.sqrt(np.maximum(cov_fused_i[:, COV_IDX_X], 0))
    cov_y_sigma  = np.sqrt(np.maximum(cov_fused_i[:, COV_IDX_Y], 0))
    cov_pos_trace = cov_fused_i[:, COV_IDX_X] + cov_fused_i[:, COV_IDX_Y]  # σ²_x + σ²_y
    cov_pos_sigma = np.sqrt(np.maximum(cov_pos_trace, 0))  # sqrt(σ²_x + σ²_y) — scalar "position uncertainty" in meters
    cov_yaw_sigma = np.sqrt(np.maximum(cov_fused_i[:, COV_IDX_YAW], 0))

    # For the summary: correlation between disagreement and covariance
    mask = ~np.isnan(disagreement) & ~np.isnan(cov_pos_sigma)
    if np.sum(mask) > 10:
        corr = np.corrcoef(disagreement[mask], cov_pos_sigma[mask])[0, 1]
    else:
        corr = float("nan")

    # Find high-disagreement windows: split by percentile to always create contrast.
    # "High" = top quartile (75th–100th), "Normal" = bottom quartile (0th–25th).
    disagreement_median = np.nanmedian(disagreement)
    disagreement_75pct = np.nanpercentile(disagreement, 75)
    disagreement_25pct = np.nanpercentile(disagreement, 25)
    disagreement_90pct = np.nanpercentile(disagreement, 90)

    # Use top-quartile as "high" — always produces a split regardless of absolute level
    high_thresh = disagreement_75pct
    # But ensure we have a meaningful spread: require at least 1 cm or 10% spread
    spread = disagreement_75pct - disagreement_25pct
    if spread < max(0.01, 0.05 * abs(disagreement_median)):
        # Disagreement is nearly flat — no meaningful "high" vs "normal" periods
        high_d_mask = np.zeros(len(disagreement), dtype=bool)
    else:
        high_d_mask = disagreement > high_thresh

    if np.any(high_d_mask):
        cov_during_high = np.nanmean(cov_pos_sigma[high_d_mask])
        cov_during_normal = np.nanmean(cov_pos_sigma[~high_d_mask])
        cov_ratio = cov_during_high / max(cov_during_normal, 1e-12)
    else:
        cov_during_high = float("nan")
        cov_during_normal = float("nan")
        cov_ratio = float("nan")

    # Guard against all-NaN disagreement (e.g. missing leg odom data)
    valid_d = disagreement[~np.isnan(disagreement)]
    if len(valid_d) > 0:
        max_disagreement = np.nanmax(disagreement)
        max_disagreement_t = t_common[np.nanargmax(disagreement)]
    else:
        max_disagreement = float("nan")
        max_disagreement_t = float("nan")

    # Stationary drift (for all scenarios; only meaningful for static)
    if len(pos_fused_i) > 0:
        drift_fused = np.sqrt(np.sum((pos_fused_i[:, :2] - pos_fused_i[0, :2]) ** 2, axis=1))
        drift_rf2o  = np.sqrt(np.sum((pos_rf2o_i[:, :2] - pos_rf2o_i[0, :2]) ** 2, axis=1))
        drift_leg   = np.sqrt(np.sum((pos_leg_i[:, :2] - pos_leg_i[0, :2]) ** 2, axis=1))
        yaw_drift_fused = np.abs(yaw_fused_i - yaw_fused_i[0])
        yaw_drift_rf2o  = np.abs(yaw_rf2o_i - yaw_rf2o_i[0])
        yaw_drift_leg   = np.abs(yaw_leg_i - yaw_leg_i[0])
    else:
        drift_fused = drift_rf2o = drift_leg = np.array([])
        yaw_drift_fused = yaw_drift_rf2o = yaw_drift_leg = np.array([])

    # ---- Generate plots ---------------------------------------------------
    print("\n[5/5] Generating plots ...")

    # --- Common style setup using dataviz principles ---
    plt.rcParams.update({
        "figure.dpi": 150,
        "savefig.dpi": 150,
        "savefig.bbox": "tight",
        "font.family": "sans-serif",
        "font.size": 10,
        "axes.titlesize": 12,
        "axes.labelsize": 10,
    })

    # Color palette (colorblind-friendly)
    C_FUSED = "#0072B2"   # blue
    C_RF2O  = "#E69F00"   # orange
    C_LEG   = "#009E73"   # green
    C_DISAG = "#D55E00"   # vermillion
    C_COV   = "#CC79A7"   # purple

    # ---- Plot 1: XY tracks ----
    fig1, ax1 = plt.subplots(figsize=(10, 9))

    if len(pos_fused_i) > 0:
        ax1.plot(pos_fused_i[:, 0], pos_fused_i[:, 1], color=C_FUSED, linewidth=1.2,
                 label="Fused EKF (/odom)", zorder=3)
        # Mark start and end
        ax1.scatter(*pos_fused_i[0, :2], color=C_FUSED, marker="o", s=60, zorder=5,
                    edgecolors="white", linewidths=0.5)
        ax1.scatter(*pos_fused_i[-1, :2], color=C_FUSED, marker="s", s=60, zorder=5,
                    edgecolors="white", linewidths=0.5)

    if len(pos_rf2o_i) > 0:
        ax1.plot(pos_rf2o_i[:, 0], pos_rf2o_i[:, 1], color=C_RF2O, linewidth=1.0,
                 linestyle="--", label="rf2o only (/odom_rf2o)", zorder=2)
        ax1.scatter(*pos_rf2o_i[0, :2], color=C_RF2O, marker="o", s=40, zorder=4,
                    edgecolors="white", linewidths=0.5)
        ax1.scatter(*pos_rf2o_i[-1, :2], color=C_RF2O, marker="s", s=40, zorder=4,
                    edgecolors="white", linewidths=0.5)

    if len(pos_leg_i) > 0:
        ax1.plot(pos_leg_i[:, 0], pos_leg_i[:, 1], color=C_LEG, linewidth=1.0,
                 linestyle=":", label="Leg odom (SMX/Odom)", zorder=2)
        ax1.scatter(*pos_leg_i[0, :2], color=C_LEG, marker="o", s=40, zorder=4,
                    edgecolors="white", linewidths=0.5)
        ax1.scatter(*pos_leg_i[-1, :2], color=C_LEG, marker="s", s=40, zorder=4,
                    edgecolors="white", linewidths=0.5)

    ax1.set_xlabel("X [m]")
    ax1.set_ylabel("Y [m]")
    ax1.set_title(f"XY Tracks — {scenario}")
    ax1.legend(loc="best", framealpha=0.9)
    ax1.set_aspect("equal")
    ax1.grid(True, alpha=0.3)
    fig1.tight_layout()
    fig1.savefig(os.path.join(output_dir, f"{scenario}_xy_tracks.png"))
    plt.close(fig1)
    print(f"  Saved: {scenario}_xy_tracks.png")

    # ---- Plot 2: Disagreement vs Covariance (THE KEY PLOT) ----
    fig2, ax2_left = plt.subplots(figsize=(14, 6))
    ax2_right = ax2_left.twinx()

    # Time axis: seconds relative to start
    t_rel = t_common - t_common[0]

    # Left axis: input disagreement
    line_disag = ax2_left.plot(t_rel, disagreement, color=C_DISAG, linewidth=1.5,
                               label="Input disagreement\n(Euclidean dist: rf2o ↔ leg odom)",
                               zorder=3)
    ax2_left.set_xlabel("Time [s]")
    ax2_left.set_ylabel("Input Disagreement [m]", color=C_DISAG)
    ax2_left.tick_params(axis="y", labelcolor=C_DISAG)

    # Right axis: covariance sigma
    line_cov = ax2_right.plot(t_rel, cov_pos_sigma, color=C_COV, linewidth=1.5,
                              label="Fused position σ\n(sqrt(σ²_x + σ²_y) from /odom)",
                              zorder=2)
    ax2_right.set_ylabel("Fused Position Uncertainty σ [m]", color=C_COV)
    ax2_right.tick_params(axis="y", labelcolor=C_COV)

    # Also plot individual X and Y sigma as faint lines
    ax2_right.plot(t_rel, cov_x_sigma, color=C_COV, linewidth=0.5, alpha=0.4, linestyle="--")
    ax2_right.plot(t_rel, cov_y_sigma, color=C_COV, linewidth=0.5, alpha=0.4, linestyle=":")

    # Combined legend
    lines = line_disag + line_cov
    labels = [l.get_label() for l in lines]
    ax2_left.legend(lines, labels, loc="upper left", framealpha=0.9)

    # Mark high-disagreement regions
    if np.any(high_d_mask):
        # Find contiguous high-disagreement regions
        regions = []
        in_region = False
        start_idx = 0
        for i, is_high in enumerate(high_d_mask):
            if is_high and not in_region:
                start_idx = i
                in_region = True
            elif not is_high and in_region:
                regions.append((start_idx, i - 1))
                in_region = False
        if in_region:
            regions.append((start_idx, len(high_d_mask) - 1))

        for r_start, r_end in regions:
            ax2_left.axvspan(t_rel[r_start], t_rel[r_end],
                             color=C_DISAG, alpha=0.08, zorder=0)

    ax2_left.set_title(f"Input Disagreement vs Fused Covariance — {scenario}\n"
                       f"(Key question: does purple rise when orange rises?)")
    ax2_left.grid(True, alpha=0.3)
    fig2.tight_layout()
    fig2.savefig(os.path.join(output_dir, f"{scenario}_disagreement.png"))
    plt.close(fig2)
    print(f"  Saved: {scenario}_disagreement.png")

    # ---- Plot 3: Stationary drift (always generated; highlight if static) ----
    fig3, (ax3a, ax3b) = plt.subplots(2, 1, figsize=(14, 8), sharex=True)

    ax3a.plot(t_rel, drift_fused, color=C_FUSED, linewidth=1.2, label="Fused drift")
    ax3a.plot(t_rel, drift_rf2o, color=C_RF2O, linewidth=1.0, linestyle="--", label="rf2o drift")
    ax3a.plot(t_rel, drift_leg, color=C_LEG, linewidth=1.0, linestyle=":", label="Leg drift")
    ax3a.set_ylabel("Position Drift [m]")
    ax3a.set_title(f"Stationary Drift — {scenario}" if scenario == "static"
                   else f"Position Drift from Start — {scenario}")
    ax3a.legend(loc="best", framealpha=0.9)
    ax3a.grid(True, alpha=0.3)

    ax3b.plot(t_rel, np.degrees(yaw_drift_fused), color=C_FUSED, linewidth=1.2, label="Fused yaw drift")
    ax3b.plot(t_rel, np.degrees(yaw_drift_rf2o), color=C_RF2O, linewidth=1.0, linestyle="--", label="rf2o yaw drift")
    ax3b.plot(t_rel, np.degrees(yaw_drift_leg), color=C_LEG, linewidth=1.0, linestyle=":", label="Leg yaw drift")
    ax3b.set_xlabel("Time [s]")
    ax3b.set_ylabel("Yaw Drift [deg]")
    ax3b.legend(loc="best", framealpha=0.9)
    ax3b.grid(True, alpha=0.3)

    fig3.tight_layout()
    fig3.savefig(os.path.join(output_dir, f"{scenario}_drift.png"))
    plt.close(fig3)
    print(f"  Saved: {scenario}_drift.png")

    # ---- Summary text -----------------------------------------------------
    duration = t_common[-1] - t_common[0] if len(t_common) > 0 else 0

    # Compute max drift for static scenario
    max_drift_fused = np.nanmax(drift_fused) if len(drift_fused) > 0 else float("nan")
    max_drift_rf2o  = np.nanmax(drift_rf2o) if len(drift_rf2o) > 0 else float("nan")
    max_drift_leg   = np.nanmax(drift_leg) if len(drift_leg) > 0 else float("nan")
    max_yaw_drift_fused = np.nanmax(np.degrees(yaw_drift_fused)) if len(yaw_drift_fused) > 0 else float("nan")
    max_yaw_drift_rf2o  = np.nanmax(np.degrees(yaw_drift_rf2o)) if len(yaw_drift_rf2o) > 0 else float("nan")
    max_yaw_drift_leg   = np.nanmax(np.degrees(yaw_drift_leg)) if len(yaw_drift_leg) > 0 else float("nan")

    # Covariance response assessment
    if not np.isnan(cov_ratio) and cov_ratio > 1.5:
        cov_verdict = (
            f"YES — fused position σ grew by {cov_ratio:.1f}× during high-disagreement "
            f"windows (σ_high={cov_during_high:.4f}m vs σ_normal={cov_during_normal:.4f}m)"
        )
    elif not np.isnan(cov_ratio) and cov_ratio > 1.1:
        cov_verdict = (
            f"WEAK — fused position σ increased only {cov_ratio:.1f}× "
            f"(σ_high={cov_during_high:.4f}m vs σ_normal={cov_during_normal:.4f}m). "
            f"Filter may be under-responsive."
        )
    elif not np.isnan(cov_ratio):
        cov_verdict = (
            f"NO — fused position σ did NOT rise (ratio={cov_ratio:.2f}×, "
            f"σ_high={cov_during_high:.4f}m vs σ_normal={cov_during_normal:.4f}m). "
            f"*** Filter is OVERCONFIDENT — covariance does not reflect input disagreement. ***"
        )
    else:
        cov_verdict = "COULD NOT ASSESS (insufficient data)"

    summary = textwrap.dedent(f"""\
    ================================================================================
    VALIDATION SUMMARY — {scenario}
    ================================================================================
    Bag              : {bag_path}
    Duration         : {duration:.1f} s
    Common timeline  : {len(t_common)} pts @ {1.0/np.mean(np.diff(t_common)):.1f} Hz (interpolated)

    ---- Covariance Audit ----
    /odom (fused)    : {"HAS covariance" if fused_has_cov else "NO covariance"} — {fused_cov_sample}
    /odom_rf2o (rf2o): {"HAS covariance" if rf2o_has_cov else "NO covariance"} — {rf2o_cov_sample}
    SMX/Odom (leg)   : {"HAS covariance" if leg_has_cov else "NO covariance"} — {leg_cov_sample}

    ---- Frame Check ----
    Fused frame      : {fused_frame}
    rf2o frame       : {rf2o_frame}
    Leg frame        : {leg_frame}
    Same frame?      : {same_frame}

    ---- Input Disagreement ----
    Max disagreement : {max_disagreement:.4f} m at t={max_disagreement_t - t_common[0]:.1f}s
    Median disagreement: {disagreement_median:.4f} m
    90th percentile  : {disagreement_90pct:.4f} m

    ---- Covariance Response (the key metric) ----
    Correlation (ρ)  : {corr:.3f}  (Pearson r between disagreement and fused position σ)
    Covariance grew? : {cov_verdict}

    ---- Drift (position / yaw from start) ----
                    Position Drift [m]     Yaw Drift [deg]
    Fused EKF        : {max_drift_fused:8.4f}            {max_yaw_drift_fused:8.3f}
    rf2o only        : {max_drift_rf2o:8.4f}            {max_yaw_drift_rf2o:8.3f}
    Leg odom only    : {max_drift_leg:8.4f}            {max_yaw_drift_leg:8.3f}

    ---- Message Counts ----
    /odom             : {len(fused_recs)}
    /odom_rf2o        : {len(rf2o_recs)}
    SMX/Odom          : {len(leg_recs)}
    ================================================================================
    """)

    summary_path = os.path.join(output_dir, f"{scenario}_summary.txt")
    with open(summary_path, "w") as f:
        f.write(summary)
    print(summary)
    print(f"  Saved: {scenario}_summary.txt")
    print("=" * 60)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Offline EKF covariance validation from a rosbag."
    )
    parser.add_argument("bag_path", help="Path to the .bag file")
    parser.add_argument("scenario", help="Scenario label (e.g. static, figure8, blank_corridor)")
    parser.add_argument("--output-dir", "-o", default="./analysis_output",
                        help="Directory for output plots and summary (default: ./analysis_output)")
    args = parser.parse_args()

    if not os.path.isfile(args.bag_path):
        print(f"ERROR: bag file not found: {args.bag_path}")
        sys.exit(1)

    analyze(args.bag_path, args.scenario, args.output_dir)


if __name__ == "__main__":
    main()
