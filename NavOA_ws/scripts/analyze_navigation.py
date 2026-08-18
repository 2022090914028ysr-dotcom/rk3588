#!/usr/bin/env python3.8
"""
analyze_navigation.py — Full-stack navigation odometry jump diagnostics from a rosbag.

Reads a bag containing:
  - /odom              : EKF fused odometry (odom→base_link)
  - /odom_rf2o         : rf2o laser scan-matching odometry
  - SMX/Odom           : leg odometry
  - /tf                : map→odom transform (from cartographer), plus odom→base_link (from EKF)
  - /scan              : LiDAR scans
  - /move_base/...     : global plan, local plan, current goal, result, status
  - /cmd_vel           : commanded velocity
  - /map               : cartographer map

Produces:
  1. Map-frame trajectory with plans overlay  →  nav_trajectory.png
  2. Odom frame stability (THE KEY PLOT)      →  nav_odom_stability.png
  3. TF vs /odom-topic divergence             →  nav_tf_divergence.png
  4. Path following (cross-track error)       →  nav_path_following.png
  5. Goal distance + velocity                 →  nav_goal_velocity.png
  6. Text summary                             →  nav_summary.txt

Jump detection: flags displacements >0.1 m or yaw changes >5° in one timestep,
classifies source as cartographer (map→odom TF), TF conflict (odom→base_link TF
diverging from /odom topic), odom topic (/odom), or sensor disagreement.

Usage:
  python3.8 scripts/analyze_navigation.py <bag_path> [--output-dir <dir>]
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
# Topic names
# ---------------------------------------------------------------------------
TOPIC_FUSED   = "/odom"
TOPIC_RF2O    = "/odom_rf2o"
TOPIC_LEG_CANDIDATES = ["SMX/Odom", "/SMX/Odom"]
TOPIC_TF      = "/tf"
TOPIC_TF_STATIC = "/tf_static"
TOPIC_SCAN    = "/scan"
TOPIC_GLOBAL_PLAN = "/move_base/GlobalPlanner/plan"
TOPIC_LOCAL_PLAN  = "/move_base/TrajectoryPlannerROS/local_plan"
TOPIC_CURRENT_GOAL = "/move_base/current_goal"
TOPIC_RESULT      = "/move_base/result"
TOPIC_STATUS      = "/move_base/status"
TOPIC_CMD_VEL     = "/cmd_vel"
TOPIC_MAP         = "/map"

# Jump detection thresholds
JUMP_DIST_M = 0.10       # 10 cm in one timestep
JUMP_YAW_DEG = 5.0       # 5 degrees in one timestep

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
        cov = np.array(msg.pose.covariance)
        records.append((secs, pos, ori, cov))
    return frame_id, records


def quat_to_yaw(q_xyzw):
    """Convert quaternion [x,y,z,w] to yaw angle (radians)."""
    x, y, z, w = q_xyzw
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quat(yaw):
    """Convert yaw angle to quaternion [x,y,z,w]."""
    return np.array([0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)])


def shortest_angle(delta):
    """Wrap an angle difference to [-π, π]."""
    return math.atan2(math.sin(delta), math.cos(delta))


def compose_pose(parent_pos, parent_yaw, child_pos, child_yaw):
    """Compose two 2D poses: parent ∘ child. Returns (x, y, yaw)."""
    # child in parent frame: rotate child by parent yaw, then translate
    cos_p, sin_p = math.cos(parent_yaw), math.sin(parent_yaw)
    x = parent_pos[0] + cos_p * child_pos[0] - sin_p * child_pos[1]
    y = parent_pos[1] + sin_p * child_pos[0] + cos_p * child_pos[1]
    yaw = parent_yaw + child_yaw
    # Normalize yaw
    yaw = math.atan2(math.sin(yaw), math.cos(yaw))
    return np.array([x, y]), yaw


def invert_pose(pos, yaw):
    """Invert a 2D pose: if T gives child in parent, return parent in child."""
    cos_y = math.cos(-yaw)
    sin_y = math.sin(-yaw)
    inv_x = cos_y * (-pos[0]) - sin_y * (-pos[1])
    inv_y = sin_y * (-pos[0]) + cos_y * (-pos[1])
    return np.array([inv_x, inv_y]), -yaw


def extract_tf_pair(bag, parent_frame, child_frame):
    """Extract parent→child transforms from /tf messages.

    Returns: list of (stamp_sec, xyz_array, yaw).
    The TF stored is parent→child: parent is frame_id, child is child_frame_id.
    """
    transforms = []
    for _topic, msg, t in bag.read_messages(topics=[TOPIC_TF]):
        for transform in msg.transforms:
            if (transform.header.frame_id == parent_frame and
                    transform.child_frame_id == child_frame):
                secs = transform.header.stamp.to_sec()
                # For stamp=0 TFs (static-like), use the message receipt time
                if secs < 1e-6:
                    secs = t.to_sec()
                tx = transform.transform.translation.x
                ty = transform.transform.translation.y
                tz = transform.transform.translation.z
                qx = transform.transform.rotation.x
                qy = transform.transform.rotation.y
                qz = transform.transform.rotation.z
                qw = transform.transform.rotation.w
                yaw = quat_to_yaw(np.array([qx, qy, qz, qw]))
                transforms.append((secs, np.array([tx, ty, tz]), yaw))
    return transforms


def build_common_timeline(timestamps_list, target_dt=None):
    """Given a list of timestamp arrays, return a common timeline."""
    all_starts = [ts[0] for ts in timestamps_list if len(ts) > 0]
    all_ends   = [ts[-1] for ts in timestamps_list if len(ts) > 0]
    if not all_starts:
        return np.array([])

    t_start = min(all_starts)
    t_end   = max(all_ends)

    if target_dt is None:
        dts = []
        for ts in timestamps_list:
            if len(ts) > 1:
                dts.append(np.median(np.diff(ts)))
        if dts:
            target_dt = max(min(dts), 0.02)  # cap at 50 Hz
        else:
            target_dt = 0.05  # 20 Hz default

    n_pts = int((t_end - t_start) / target_dt) + 1
    return np.linspace(t_start, t_end, n_pts)


def interpolate_to_timeline(src_t, src_vals, target_t, kind="linear"):
    """Interpolate 1D values to target timeline."""
    if len(src_t) < 2:
        if len(src_t) == 1:
            return np.full_like(target_t, src_vals[0])
        return np.full_like(target_t, np.nan)

    if HAVE_SCIPY:
        f = interp1d(src_t, src_vals, kind=kind,
                     bounds_error=False, fill_value="extrapolate")
        raw = f(target_t)
        lo, hi = np.min(src_vals), np.max(src_vals)
        # Only clip for bounded values, not for angles
        if np.issubdtype(np.asarray(src_vals).dtype, np.floating):
            return np.clip(raw, lo - 0.5 * abs(lo), hi + 0.5 * abs(hi))
        return raw
    else:
        return np.interp(target_t, src_t, src_vals)


def unwrap_yaw(yaws):
    """Unwrap yaw angles to avoid ±π jumps in visualization."""
    return np.unwrap(yaws)


def detect_jumps(t_common, pos_map, yaw_map, pos_odom_bl, yaw_odom_bl,
                 pos_fused, yaw_fused, pos_map_to_odom, yaw_map_to_odom):
    """Detect jumps in the map-frame pose consumers see, and classify source.

    The pose consumers use is map→base_link = (map→odom TF) ∘ (odom→base_link TF).
    A jump in it must come from map→odom (cartographer), odom→base_link TF (what
    consumers see), or the /odom topic (EKF). The key TF-conflict test: if
    odom→base_link TF jumps while /odom topic stays smooth, a second broadcaster
    is contaminating the TF tree.

    Returns list of dicts: {t, displacement_m, yaw_change_deg, divergence_m, source}
      source: 'cartographer' | 'tf_conflict' | 'odom_topic' | 'unknown'
    """
    jumps = []
    if len(t_common) < 2:
        return jumps

    for i in range(1, len(t_common)):
        dt = t_common[i] - t_common[i - 1]
        if dt <= 0:
            continue

        # Map-frame displacement (what consumers see)
        dpos = np.sqrt((pos_map[i, 0] - pos_map[i - 1, 0]) ** 2 +
                       (pos_map[i, 1] - pos_map[i - 1, 1]) ** 2)
        dyaw = abs(shortest_angle(yaw_map[i] - yaw_map[i - 1]))

        if dpos <= JUMP_DIST_M and dyaw <= math.radians(JUMP_YAW_DEG):
            continue

        # map→odom TF change (cartographer)
        dpos_tf = np.sqrt((pos_map_to_odom[i, 0] - pos_map_to_odom[i - 1, 0]) ** 2 +
                          (pos_map_to_odom[i, 1] - pos_map_to_odom[i - 1, 1]) ** 2)
        dyaw_tf = abs(shortest_angle(yaw_map_to_odom[i] - yaw_map_to_odom[i - 1]))

        # odom→base_link TF change (what consumers see)
        dpos_odom_bl = np.sqrt((pos_odom_bl[i, 0] - pos_odom_bl[i - 1, 0]) ** 2 +
                               (pos_odom_bl[i, 1] - pos_odom_bl[i - 1, 1]) ** 2)
        dyaw_odom_bl = abs(shortest_angle(yaw_odom_bl[i] - yaw_odom_bl[i - 1]))

        # /odom topic change (EKF fused estimate)
        dpos_fused = np.sqrt((pos_fused[i, 0] - pos_fused[i - 1, 0]) ** 2 +
                             (pos_fused[i, 1] - pos_fused[i - 1, 1]) ** 2)
        dyaw_fused = abs(shortest_angle(yaw_fused[i] - yaw_fused[i - 1]))

        # TF-vs-topic divergence at this instant
        div_pos = np.sqrt((pos_odom_bl[i, 0] - pos_fused[i, 0]) ** 2 +
                          (pos_odom_bl[i, 1] - pos_fused[i, 1]) ** 2)

        if dpos_tf > JUMP_DIST_M or dyaw_tf > math.radians(JUMP_YAW_DEG):
            source = "cartographer (map→odom TF)"
        elif dpos_odom_bl > JUMP_DIST_M or dyaw_odom_bl > math.radians(JUMP_YAW_DEG):
            if dpos_fused > JUMP_DIST_M or dyaw_fused > math.radians(JUMP_YAW_DEG):
                source = "odom topic (/odom) — TF followed the topic"
            else:
                source = "TF conflict (odom→base_link TF ≠ /odom topic)"
        elif dpos_fused > JUMP_DIST_M or dyaw_fused > math.radians(JUMP_YAW_DEG):
            source = "odom topic (/odom) only"
        else:
            source = "unknown (sub-threshold components)"

        jumps.append({
            "t": t_common[i],
            "displacement_m": dpos,
            "yaw_change_deg": math.degrees(dyaw),
            "dpos_tf": dpos_tf,
            "dpos_odom_tf": dpos_odom_bl,
            "dpos_fused": dpos_fused,
            "divergence_m": div_pos,
            "source": source,
        })

    return jumps


# ---------------------------------------------------------------------------
# Path parsing
# ---------------------------------------------------------------------------

def extract_plans(bag, topic, topic_label):
    """Extract nav_msgs/Path messages.

    Returns list of (stamp_sec, array_of_xy_points, frame_id).
    Only keeps the latest plan per goal cycle to avoid clutter.
    """
    plans = []
    for _topic, msg, t in bag.read_messages(topics=[topic]):
        secs = msg.header.stamp.to_sec()
        pts = np.array([(pose.pose.position.x, pose.pose.position.y)
                        for pose in msg.poses])
        frame_id = msg.header.frame_id
        if len(pts) > 0:
            plans.append((secs, pts, frame_id))
    return plans


def extract_goals(bag):
    """Extract move_base goals and results.

    Returns: (goals, results)
      goals: list of (stamp_sec, x, y, yaw, goal_id)
      results: list of (stamp_sec, status_text)
    """
    import actionlib_msgs.msg as actionlib_msgs

    goals = []
    for _topic, msg, t in bag.read_messages(topics=[TOPIC_CURRENT_GOAL]):
        secs = msg.header.stamp.to_sec()
        px = msg.pose.position.x
        py = msg.pose.position.y
        yaw = quat_to_yaw(np.array([
            msg.pose.orientation.x, msg.pose.orientation.y,
            msg.pose.orientation.z, msg.pose.orientation.w
        ]))
        goals.append((secs, px, py, yaw))

    results = []
    for _topic, msg, t in bag.read_messages(topics=[TOPIC_RESULT]):
        secs = msg.header.stamp.to_sec()
        # status is an actionlib_msgs/GoalStatus
        results.append((secs, msg.status.status, msg.status.text))

    statuses = []
    for _topic, msg, t in bag.read_messages(topics=[TOPIC_STATUS]):
        # msg is actionlib_msgs/GoalStatusArray — has status_list, not goal_id
        for gs in msg.status_list:
            secs = t.to_sec()
            if gs.goal_id.id:
                statuses.append((secs, gs.status, gs.text))

    return goals, results, statuses


def extract_cmd_vel(bag):
    """Extract /cmd_vel messages.

    Returns: list of (stamp_sec, linear_x, angular_z)
    """
    cmd_vels = []
    for _topic, msg, t in bag.read_messages(topics=[TOPIC_CMD_VEL]):
        secs = t.to_sec()
        cmd_vels.append((secs, msg.linear.x, msg.angular.z))
    return cmd_vels


# ---------------------------------------------------------------------------
# Main analysis
# ---------------------------------------------------------------------------

def analyze(bag_path, output_dir):
    os.makedirs(output_dir, exist_ok=True)

    print("=" * 60)
    print(f"  NAVIGATION ODOMETRY ANALYSIS")
    print(f"  Bag: {bag_path}")
    print("=" * 60)

    bag = rosbag.Bag(bag_path, "r")

    # Resolve leg-odom topic
    topic_leg = find_topic_in_bag(bag, TOPIC_LEG_CANDIDATES)
    if topic_leg is None:
        print("  *** WARNING: No leg-odom topic found. Tried: {}".format(TOPIC_LEG_CANDIDATES))
        topic_leg_display = "SMX/Odom (NOT FOUND)"
    else:
        topic_leg_display = topic_leg

    # ---- Extract messages -------------------------------------------------
    print("\n[1/6] Reading messages from bag ...")
    fused_frame, fused_recs = extract_odom_msgs(bag, TOPIC_FUSED)
    rf2o_frame,  rf2o_recs  = extract_odom_msgs(bag, TOPIC_RF2O)
    leg_frame,   leg_recs   = extract_odom_msgs(bag, topic_leg) if topic_leg else (None, [])

    print("  Reading /tf for map→odom and odom→base_link transforms ...")
    map_to_odom_tfs = extract_tf_pair(bag, "map", "odom")
    odom_to_bl_tfs = extract_tf_pair(bag, "odom", "base_link")

    print("  Reading nav plans + goals + cmd_vel ...")
    global_plans = extract_plans(bag, TOPIC_GLOBAL_PLAN, "global")
    local_plans  = extract_plans(bag, TOPIC_LOCAL_PLAN, "local")
    goals, results, statuses = extract_goals(bag)
    cmd_vels = extract_cmd_vel(bag)

    # Scan count for info
    scan_count = bag.get_message_count(TOPIC_SCAN)

    bag.close()

    print(f"  /odom               : {len(fused_recs):6d} messages  frame_id={fused_frame or '?'}")
    print(f"  /odom_rf2o          : {len(rf2o_recs):6d} messages  frame_id={rf2o_frame or '?'}")
    print(f"  {topic_leg_display:20s}: {len(leg_recs):6d} messages  frame_id={leg_frame or '?'}")
    print(f"  map→odom TFs        : {len(map_to_odom_tfs):6d} transforms")
    print(f"  odom→base_link TFs  : {len(odom_to_bl_tfs):6d} transforms")
    print(f"  /scan               : {scan_count:6d} messages")
    print(f"  Global plans        : {len(global_plans):6d}")
    print(f"  Local plans         : {len(local_plans):6d}")
    print(f"  Goals               : {len(goals):6d}")
    print(f"  Results             : {len(results):6d}")
    print(f"  Status msgs         : {len(statuses):6d}")
    print(f"  /cmd_vel            : {len(cmd_vels):6d}")

    # ---- Time alignment ---------------------------------------------------
    print("\n[2/6] Time-aligning streams ...")

    t_fused = np.array([r[0] for r in fused_recs])
    t_rf2o  = np.array([r[0] for r in rf2o_recs])
    t_leg   = np.array([r[0] for r in leg_recs]) if len(leg_recs) > 0 else np.array([])
    t_tf    = np.array([r[0] for r in map_to_odom_tfs])
    t_tf_odom_bl = np.array([r[0] for r in odom_to_bl_tfs])
    t_cmd   = np.array([r[0] for r in cmd_vels]) if len(cmd_vels) > 0 else np.array([])

    # Build common timeline from all available streams
    all_ts = [t_fused, t_rf2o, t_tf]
    if len(t_leg) > 0:
        all_ts.append(t_leg)
    streams = [(t, n) for t, n in zip(all_ts, ["fused", "rf2o", "tf", "leg"][:len(all_ts)]) if len(t) > 0]
    t_common = build_common_timeline([t for t, _ in streams])
    print(f"  Common timeline: {len(t_common)} points, "
          f"dt={np.mean(np.diff(t_common)):.4f}s "
          f"({1.0/np.mean(np.diff(t_common)):.1f} Hz)")

    t_rel = t_common - t_common[0]

    # ---- Interpolate odometry to common timeline --------------------------
    print("\n[3/6] Interpolating to common timeline ...")

    # Fused /odom (odom→base_link)
    if len(t_fused) >= 2:
        pos_fused_i = np.column_stack([
            interpolate_to_timeline(t_fused, np.array([r[1][i] for r in fused_recs]), t_common)
            for i in range(3)
        ])
        yaw_fused_raw = np.array([quat_to_yaw(r[2]) for r in fused_recs])
        yaw_fused_i = interpolate_to_timeline(t_fused, np.unwrap(yaw_fused_raw), t_common)
    else:
        pos_fused_i = np.full((len(t_common), 3), np.nan)
        yaw_fused_i = np.full(len(t_common), np.nan)

    # rf2o
    if len(t_rf2o) >= 2:
        pos_rf2o_i = np.column_stack([
            interpolate_to_timeline(t_rf2o, np.array([r[1][i] for r in rf2o_recs]), t_common)
            for i in range(3)
        ])
        yaw_rf2o_raw = np.array([quat_to_yaw(r[2]) for r in rf2o_recs])
        yaw_rf2o_i = interpolate_to_timeline(t_rf2o, np.unwrap(yaw_rf2o_raw), t_common)
    else:
        pos_rf2o_i = np.full((len(t_common), 3), np.nan)
        yaw_rf2o_i = np.full(len(t_common), np.nan)

    # Leg odom
    if len(t_leg) >= 2:
        pos_leg_i = np.column_stack([
            interpolate_to_timeline(t_leg, np.array([r[1][i] for r in leg_recs]), t_common)
            for i in range(3)
        ])
        yaw_leg_raw = np.array([quat_to_yaw(r[2]) for r in leg_recs])
        yaw_leg_i = interpolate_to_timeline(t_leg, np.unwrap(yaw_leg_raw), t_common)
    else:
        pos_leg_i = np.full((len(t_common), 3), np.nan)
        yaw_leg_i = np.full(len(t_common), np.nan)

    # map→odom TF
    if len(t_tf) >= 2:
        pos_tf_i = np.column_stack([
            interpolate_to_timeline(t_tf, np.array([r[1][i] for r in map_to_odom_tfs]), t_common)
            for i in range(3)
        ])
        yaw_tf_raw = np.array([r[2] for r in map_to_odom_tfs])
        yaw_tf_i = interpolate_to_timeline(t_tf, np.unwrap(yaw_tf_raw), t_common)
    else:
        pos_tf_i = np.full((len(t_common), 3), np.nan)
        yaw_tf_i = np.full(len(t_common), np.nan)

    # odom→base_link TF (what consumers actually see — may diverge from /odom topic)
    if len(t_tf_odom_bl) >= 2:
        pos_tf_odom_bl_i = np.column_stack([
            interpolate_to_timeline(t_tf_odom_bl, np.array([r[1][i] for r in odom_to_bl_tfs]), t_common)
            for i in range(3)
        ])
        yaw_tf_odom_bl_raw = np.array([r[2] for r in odom_to_bl_tfs])
        yaw_tf_odom_bl_i = interpolate_to_timeline(t_tf_odom_bl, np.unwrap(yaw_tf_odom_bl_raw), t_common)
    else:
        pos_tf_odom_bl_i = np.full((len(t_common), 3), np.nan)
        yaw_tf_odom_bl_i = np.full(len(t_common), np.nan)

    # ---- Compute map-frame trajectory -------------------------------------
    print("[4/6] Computing map-frame trajectory ...")

    # odom→base_link source for the map-frame pose. Prefer the TF (what consumers
    # actually see via lookupTransform); fall back to the /odom topic if no
    # odom→base_link TF was recorded in the bag.
    use_tf_for_odom_bl = np.any(~np.isnan(pos_tf_odom_bl_i[:, 0]))
    pos_odom_bl_i = pos_tf_odom_bl_i if use_tf_for_odom_bl else pos_fused_i
    yaw_odom_bl_i = yaw_tf_odom_bl_i if use_tf_for_odom_bl else yaw_fused_i

    pos_map = np.full((len(t_common), 3), np.nan)
    yaw_map = np.full(len(t_common), np.nan)

    for i in range(len(t_common)):
        if np.isnan(pos_tf_i[i, 0]) or np.isnan(pos_odom_bl_i[i, 0]):
            continue
        # map→base_link = (map→odom) ∘ (odom→base_link)
        pmap, ymap = compose_pose(
            pos_tf_i[i, :2], yaw_tf_i[i],
            pos_odom_bl_i[i, :2], yaw_odom_bl_i[i]
        )
        pos_map[i, :2] = pmap
        pos_map[i, 2] = pos_odom_bl_i[i, 2]
        yaw_map[i] = ymap

    # ---- TF-vs-topic divergence (odom→base_link TF vs /odom topic) --------
    # The odometry consumers read the odom→base_link TF; the EKF publishes its
    # fused estimate on the /odom topic. If they diverge, a second broadcaster
    # (leg/rf2o) is contaminating the TF tree — the root cause of the "odometry
    # jumps during navigation" bug.
    tf_topic_divergence_pos = np.sqrt(
        np.sum((pos_tf_odom_bl_i[:, :2] - pos_fused_i[:, :2]) ** 2, axis=1))
    yaw_diff = np.abs(yaw_tf_odom_bl_i - yaw_fused_i)
    tf_topic_divergence_yaw = np.minimum(yaw_diff, 2 * math.pi - yaw_diff)

    # ---- Sensor disagreement ---------------------------------------------
    disagreement = np.sqrt(np.sum((pos_rf2o_i[:, :2] - pos_leg_i[:, :2]) ** 2, axis=1))
    yaw_disagreement = np.abs(yaw_rf2o_i - yaw_leg_i)
    # Handle wrap
    yaw_disagreement = np.minimum(yaw_disagreement, 2 * math.pi - yaw_disagreement)

    # ---- Jump detection ----------------------------------------------------
    print("[5/6] Detecting jumps ...")
    jumps = detect_jumps(t_common, pos_map, yaw_map,
                         pos_odom_bl_i, yaw_odom_bl_i,
                         pos_fused_i, yaw_fused_i,
                         pos_tf_i, yaw_tf_i)
    print(f"  Detected {len(jumps)} jumps/teleports (> {JUMP_DIST_M}m or > {JUMP_YAW_DEG}°)")

    # ---- Cross-track error ------------------------------------------------
    # Use the latest global plan before each timestep
    global_plan_t = np.array([p[0] for p in global_plans])
    cross_track_error = np.full(len(t_common), np.nan)
    for i in range(len(t_common)):
        # Find the most recent global plan before this timestamp
        mask = global_plan_t <= t_common[i]
        if not np.any(mask):
            continue
        plan_idx = np.argmax(global_plan_t[mask])  # latest before now
        plan_pts = global_plans[np.where(mask)[0][plan_idx]][1]
        if len(plan_pts) < 2:
            continue
        # Distance from robot to nearest segment of the plan
        rx, ry = pos_map[i, 0], pos_map[i, 1]
        if np.isnan(rx) or np.isnan(ry):
            continue
        min_dist = float("inf")
        for j in range(len(plan_pts) - 1):
            # Point-to-segment distance
            p1, p2 = plan_pts[j], plan_pts[j + 1]
            seg = p2 - p1
            seg_len_sq = seg[0] ** 2 + seg[1] ** 2
            if seg_len_sq < 1e-12:
                dist = np.sqrt((rx - p1[0]) ** 2 + (ry - p1[1]) ** 2)
            else:
                t_param = max(0.0, min(1.0, np.dot([rx - p1[0], ry - p1[1]], seg) / seg_len_sq))
                proj = p1 + t_param * seg
                dist = np.sqrt((rx - proj[0]) ** 2 + (ry - proj[1]) ** 2)
            min_dist = min(min_dist, dist)
        cross_track_error[i] = min_dist

    # ---- Goal distance over time -----------------------------------------
    goal_distance = np.full(len(t_common), np.nan)
    active_goal_idx = np.full(len(t_common), -1, dtype=int)

    if len(goals) > 0:
        for i in range(len(t_common)):
            # Find the active goal at this timestamp
            # A goal is active from its timestamp until the next goal or a result
            for g in range(len(goals) - 1, -1, -1):
                if t_common[i] >= goals[g][0]:
                    active_goal_idx[i] = g
                    gx, gy = goals[g][1], goals[g][2]
                    if not np.isnan(pos_map[i, 0]):
                        goal_distance[i] = np.sqrt((pos_map[i, 0] - gx) ** 2 +
                                                   (pos_map[i, 1] - gy) ** 2)
                    break

    # ---- Commanded velocity to common timeline ----------------------------
    cmd_v_i = np.full(len(t_common), np.nan)
    cmd_w_i = np.full(len(t_common), np.nan)
    if len(cmd_vels) >= 2:
        cmd_v_i = interpolate_to_timeline(
            np.array([c[0] for c in cmd_vels]),
            np.array([c[1] for c in cmd_vels]),
            t_common
        )
        cmd_w_i = interpolate_to_timeline(
            np.array([c[0] for c in cmd_vels]),
            np.array([c[2] for c in cmd_vels]),
            t_common
        )

    # ---- Compute actual velocity from odometry ----------------------------
    odom_v_i = np.full(len(t_common), np.nan)
    odom_w_i = np.full(len(t_common), np.nan)
    for i in range(1, len(t_common)):
        dt = t_common[i] - t_common[i - 1]
        if dt <= 0:
            continue
        if not np.isnan(pos_fused_i[i, 0]) and not np.isnan(pos_fused_i[i - 1, 0]):
            dpos = np.sqrt((pos_fused_i[i, 0] - pos_fused_i[i - 1, 0]) ** 2 +
                           (pos_fused_i[i, 1] - pos_fused_i[i - 1, 1]) ** 2)
            odom_v_i[i] = dpos / dt
            dyaw = yaw_fused_i[i] - yaw_fused_i[i - 1]
            # Handle wrap
            dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))
            odom_w_i[i] = dyaw / dt

    # ---- Generate plots ---------------------------------------------------
    print("\n[6/6] Generating plots ...")

    # Common style
    plt.rcParams.update({
        "figure.dpi": 150,
        "savefig.dpi": 150,
        "savefig.bbox": "tight",
        "font.family": "sans-serif",
        "font.size": 10,
        "axes.titlesize": 12,
        "axes.labelsize": 10,
    })

    # Colorblind-friendly palette
    C_MAP      = "#0072B2"  # blue — robot trajectory
    C_PLAN_G   = "#56B4E9"  # sky blue — global plan
    C_PLAN_L   = "#E69F00"  # orange — local plan
    C_GOAL     = "#D55E00"  # vermillion — goals
    C_RF2O     = "#009E73"  # green
    C_LEG      = "#CC79A7"  # purple
    C_DISAG    = "#D55E00"  # vermillion
    C_CMD      = "#E69F00"  # orange — commanded
    C_ACTUAL   = "#0072B2"  # blue — actual

    # =====================================================================
    # Plot 1: Map-frame trajectory with plans
    # =====================================================================
    fig1, ax1 = plt.subplots(figsize=(11, 10))

    # Plot global plans (only show distinct ones to avoid clutter)
    plotted_global = set()
    for secs, pts, _fid in global_plans:
        # dedup by first point
        key = (round(pts[0, 0], 2), round(pts[0, 1], 2))
        if key in plotted_global:
            continue
        plotted_global.add(key)
        ax1.plot(pts[:, 0], pts[:, 1], color=C_PLAN_G, linewidth=1.5, alpha=0.6,
                 linestyle="--", zorder=1)

    # Plot local plans (last few only)
    for secs, pts, _fid in local_plans[-5:]:
        ax1.plot(pts[:, 0], pts[:, 1], color=C_PLAN_L, linewidth=1.0, alpha=0.4,
                 linestyle=":", zorder=1)

    # Add legend entries for plans
    from matplotlib.lines import Line2D
    custom_lines = [
        Line2D([0], [0], color=C_PLAN_G, linestyle="--", linewidth=1.5, alpha=0.8,
               label="Global plans"),
        Line2D([0], [0], color=C_PLAN_L, linestyle=":", linewidth=1.0, alpha=0.8,
               label="Local plans (last 5)"),
    ]

    # Robot trajectory
    valid_map = ~np.isnan(pos_map[:, 0])
    if np.any(valid_map):
        ax1.plot(pos_map[valid_map, 0], pos_map[valid_map, 1],
                 color=C_MAP, linewidth=1.8, label="Robot trajectory (map frame)",
                 zorder=3)
        # Start marker
        ax1.scatter(pos_map[valid_map][0, 0], pos_map[valid_map][0, 1],
                    color=C_MAP, marker="o", s=80, zorder=5,
                    edgecolors="white", linewidths=1.0, label="Start")
        # End marker
        ax1.scatter(pos_map[valid_map][-1, 0], pos_map[valid_map][-1, 1],
                    color=C_MAP, marker="s", s=80, zorder=5,
                    edgecolors="white", linewidths=1.0, label="End")

    # Goal markers
    for g_idx, (g_t, gx, gy, gyaw) in enumerate(goals):
        ax1.scatter(gx, gy, color=C_GOAL, marker="*", s=200, zorder=6,
                    edgecolors="white", linewidths=0.8)
        ax1.annotate(f"G{g_idx + 1}", (gx, gy),
                     textcoords="offset points", xytext=(8, 8), fontsize=9,
                     color=C_GOAL, fontweight="bold")

    # Jump markers
    for j in jumps:
        t_j = j["t"]
        idx = np.argmin(np.abs(t_common - t_j))
        if not np.isnan(pos_map[idx, 0]):
            ax1.scatter(pos_map[idx, 0], pos_map[idx, 1],
                        color="red", marker="x", s=50, zorder=7, alpha=0.7)

    ax1.set_xlabel("X [m] (map frame)")
    ax1.set_ylabel("Y [m] (map frame)")
    ax1.set_title(f"Map-Frame Trajectory with Navigation Plans\n"
                  f"({len(jumps)} jumps detected, {len(goals)} goals)")
    ax1.legend(handles=custom_lines + ax1.get_legend_handles_labels()[0],
               loc="best", framealpha=0.9, fontsize=8)
    ax1.set_aspect("equal")
    ax1.grid(True, alpha=0.3)
    fig1.tight_layout()
    fig1.savefig(os.path.join(output_dir, "nav_trajectory.png"))
    plt.close(fig1)
    print(f"  Saved: nav_trajectory.png")

    # =====================================================================
    # Plot 2: Odom frame stability (THE KEY PLOT)
    # =====================================================================
    fig2, (ax2a, ax2b, ax2c) = plt.subplots(3, 1, figsize=(16, 12), sharex=True)

    # Subplot A: map→odom TF
    ax2a.plot(t_rel, pos_tf_i[:, 0], color=C_MAP, linewidth=1.2, label="TF X")
    ax2a.plot(t_rel, pos_tf_i[:, 1], color=C_RF2O, linewidth=1.2, label="TF Y")
    ax2a.set_ylabel("map→odom TF [m]")
    ax2a.set_title("Odom Frame Stability — JUMP DIAGNOSIS (check which subplot has spikes)")
    ax2a.legend(loc="upper left", framealpha=0.9, fontsize=8)
    ax2a.grid(True, alpha=0.3)

    ax2a_twin = ax2a.twinx()
    ax2a_twin.plot(t_rel, np.degrees(yaw_tf_i), color=C_LEG, linewidth=1.0,
                   linestyle="--", alpha=0.7, label="TF Yaw [°]")
    ax2a_twin.set_ylabel("Yaw [deg]", color=C_LEG)
    ax2a_twin.tick_params(axis="y", labelcolor=C_LEG)

    # Subplot B: odom→base_link (fused /odom)
    ax2b.plot(t_rel, pos_fused_i[:, 0], color=C_MAP, linewidth=1.2, label="Odom X")
    ax2b.plot(t_rel, pos_fused_i[:, 1], color=C_RF2O, linewidth=1.2, label="Odom Y")
    ax2b.set_ylabel("/odom (fused) [m]")
    ax2b.legend(loc="upper left", framealpha=0.9, fontsize=8)
    ax2b.grid(True, alpha=0.3)

    ax2b_twin = ax2b.twinx()
    ax2b_twin.plot(t_rel, np.degrees(yaw_fused_i), color=C_LEG, linewidth=1.0,
                   linestyle="--", alpha=0.7, label="Odom Yaw [°]")
    ax2b_twin.set_ylabel("Yaw [deg]", color=C_LEG)
    ax2b_twin.tick_params(axis="y", labelcolor=C_LEG)

    # Subplot C: rf2o vs leg disagreement
    ax2c.plot(t_rel, disagreement, color=C_DISAG, linewidth=1.2,
              label="Position disagreement (rf2o ↔ leg)")
    ax2c.set_ylabel("Disagreement [m]")
    ax2c.set_xlabel("Time [s]")
    ax2c.legend(loc="upper left", framealpha=0.9, fontsize=8)
    ax2c.grid(True, alpha=0.3)
    ax2c.set_ylim(bottom=0)

    # Mark jumps with vertical lines on all subplots
    for j in jumps:
        t_j = j["t"] - t_common[0]
        for ax in [ax2a, ax2b, ax2c]:
            ax.axvline(t_j, color="red", linewidth=0.8, alpha=0.4, linestyle="-")

    # If we have jumps, add one visible label
    if len(jumps) > 0:
        ax2a.axvline(jumps[0]["t"] - t_common[0], color="red", linewidth=0.8,
                     alpha=0.7, linestyle="-", label=f"Jump ({len(jumps)} total)")
        ax2a.legend(loc="upper left", framealpha=0.9, fontsize=8)

    fig2.tight_layout()
    fig2.savefig(os.path.join(output_dir, "nav_odom_stability.png"))
    plt.close(fig2)
    print(f"  Saved: nav_odom_stability.png")

    # =====================================================================
    # Plot 3: Path following (cross-track error)
    # =====================================================================
    fig3, ax3 = plt.subplots(figsize=(16, 5))

    # Different color per active goal
    unique_goals = np.unique(active_goal_idx[active_goal_idx >= 0])
    goal_colors = plt.cm.tab10(np.linspace(0, 1, max(len(unique_goals), 1)))

    for g_uniq in unique_goals:
        mask = active_goal_idx == g_uniq
        color = goal_colors[list(unique_goals).index(g_uniq) % len(goal_colors)]
        ax3.plot(t_rel[mask], cross_track_error[mask], color=color, linewidth=0.8,
                 label=f"Goal {g_uniq + 1}")

    ax3.set_ylabel("Cross-Track Error [m]")
    ax3.set_xlabel("Time [s]")
    ax3.set_title("Cross-Track Error (distance to nearest point on global plan)")
    ax3.legend(loc="upper right", framealpha=0.9, fontsize=8)
    ax3.grid(True, alpha=0.3)
    ax3.set_ylim(bottom=0)

    # Mark jumps
    for j in jumps:
        ax3.axvline(j["t"] - t_common[0], color="red", linewidth=0.8, alpha=0.3,
                    linestyle="-")

    fig3.tight_layout()
    fig3.savefig(os.path.join(output_dir, "nav_path_following.png"))
    plt.close(fig3)
    print(f"  Saved: nav_path_following.png")

    # =====================================================================
    # Plot 4: Goal distance + velocity
    # =====================================================================
    fig4, (ax4a, ax4b) = plt.subplots(2, 1, figsize=(16, 8), sharex=True)

    # Subplot A: goal distance
    for g_uniq in unique_goals:
        mask = active_goal_idx == g_uniq
        color = goal_colors[list(unique_goals).index(g_uniq) % len(goal_colors)]
        valid = mask & ~np.isnan(goal_distance)
        ax4a.plot(t_rel[valid], goal_distance[valid], color=color, linewidth=1.2,
                  label=f"Goal {g_uniq + 1}")

    # Mark goal timestamps
    for g_idx, (g_t, _, _, _) in enumerate(goals):
        ax4a.axvline(g_t - t_common[0], color=goal_colors[g_idx % len(goal_colors)],
                     linewidth=1.0, alpha=0.5, linestyle="--",
                     label=f"Goal {g_idx + 1} sent" if g_idx == 0 or len(goals) <= 5 else "")

    ax4a.set_ylabel("Distance to Goal [m]")
    ax4a.set_title("Goal Progress + Velocity Tracking")
    ax4a.legend(loc="upper right", framealpha=0.9, fontsize=7)
    ax4a.grid(True, alpha=0.3)
    ax4a.set_ylim(bottom=0)

    # Subplot B: commanded vs actual velocity
    ax4b.plot(t_rel, cmd_v_i, color=C_CMD, linewidth=1.2,
              label="Cmd linear vel")
    ax4b.plot(t_rel, odom_v_i, color=C_ACTUAL, linewidth=1.0, alpha=0.7,
              label="Actual linear vel (from /odom)")
    ax4b.set_ylabel("Linear Velocity [m/s]")
    ax4b.set_xlabel("Time [s]")
    ax4b.legend(loc="upper left", framealpha=0.9, fontsize=8)
    ax4b.grid(True, alpha=0.3)

    # Angular velocity on twin axis
    ax4b_twin = ax4b.twinx()
    ax4b_twin.plot(t_rel, cmd_w_i, color=C_CMD, linewidth=1.0, alpha=0.5,
                   linestyle="--", label="Cmd angular vel")
    ax4b_twin.plot(t_rel, odom_w_i, color=C_ACTUAL, linewidth=0.8, alpha=0.4,
                   linestyle=":", label="Actual angular vel")
    ax4b_twin.set_ylabel("Angular Velocity [rad/s]")
    ax4b_twin.legend(loc="upper right", framealpha=0.9, fontsize=7)

    fig4.tight_layout()
    fig4.savefig(os.path.join(output_dir, "nav_goal_velocity.png"))
    plt.close(fig4)
    print(f"  Saved: nav_goal_velocity.png")

    # =====================================================================
    # Plot 5: odom→base_link TF vs /odom topic divergence (TF-CONFLICT signal)
    # =====================================================================
    fig5, (ax5a, ax5b) = plt.subplots(2, 1, figsize=(16, 9), sharex=True)

    # Subplot A: odom→base_link TF (solid) vs /odom topic (dashed)
    ax5a.plot(t_rel, pos_tf_odom_bl_i[:, 0], color=C_MAP, linewidth=1.2,
              label="odom→base_link TF X (consumers see)")
    ax5a.plot(t_rel, pos_tf_odom_bl_i[:, 1], color=C_RF2O, linewidth=1.2,
              label="odom→base_link TF Y")
    ax5a.plot(t_rel, pos_fused_i[:, 0], color=C_MAP, linewidth=1.0,
              linestyle="--", alpha=0.7, label="/odom topic X (EKF)")
    ax5a.plot(t_rel, pos_fused_i[:, 1], color=C_RF2O, linewidth=1.0,
              linestyle="--", alpha=0.7, label="/odom topic Y (EKF)")
    ax5a.set_ylabel("odom→base_link [m]")
    ax5a.set_title("TF vs /odom-topic divergence — if solid & dashed separate, "
                   "a second broadcaster is contaminating the TF tree")
    ax5a.legend(loc="upper left", framealpha=0.9, fontsize=8)
    ax5a.grid(True, alpha=0.3)

    # Subplot B: divergence magnitude
    ax5b.plot(t_rel, tf_topic_divergence_pos, color=C_DISAG, linewidth=1.2,
              label="Position divergence (TF ↔ topic)")
    ax5b.set_ylabel("Position divergence [m]")
    ax5b.set_xlabel("Time [s]")
    ax5b.set_ylim(bottom=0)
    ax5b.legend(loc="upper left", framealpha=0.9, fontsize=8)
    ax5b.grid(True, alpha=0.3)

    ax5b_twin = ax5b.twinx()
    ax5b_twin.plot(t_rel, np.degrees(tf_topic_divergence_yaw), color=C_LEG,
                   linewidth=1.0, linestyle="--", alpha=0.7,
                   label="Yaw divergence [°]")
    ax5b_twin.set_ylabel("Yaw divergence [deg]", color=C_LEG)
    ax5b_twin.tick_params(axis="y", labelcolor=C_LEG)
    ax5b_twin.legend(loc="upper right", framealpha=0.9, fontsize=7)

    # Mark jumps
    for j in jumps:
        for ax in [ax5a, ax5b]:
            ax.axvline(j["t"] - t_common[0], color="red", linewidth=0.8,
                       alpha=0.3, linestyle="-")

    fig5.tight_layout()
    fig5.savefig(os.path.join(output_dir, "nav_tf_divergence.png"))
    plt.close(fig5)
    print(f"  Saved: nav_tf_divergence.png")

    # =====================================================================
    # Summary text
    # =====================================================================
    duration = t_common[-1] - t_common[0] if len(t_common) > 0 else 0

    # Jump source breakdown
    jump_sources = defaultdict(int)
    for j in jumps:
        jump_sources[j["source"]] += 1

    # Per-goal analysis
    goal_summaries = []
    for g_idx, (g_t, gx, gy, gyaw) in enumerate(goals):
        # Find result for this goal
        reached_pose = None
        reached_status = None
        for r_t, r_status, r_text in results:
            if r_t > g_t:
                reached_status = r_text
                # Get robot pose at result time
                idx = np.argmin(np.abs(t_common - r_t))
                if idx < len(t_common) and not np.isnan(pos_map[idx, 0]):
                    reached_pose = (pos_map[idx, 0], pos_map[idx, 1], yaw_map[idx])
                break

        if reached_pose is not None:
            goal_error = np.sqrt((reached_pose[0] - gx) ** 2 +
                                (reached_pose[1] - gy) ** 2)
            goal_yaw_error = abs(reached_pose[2] - gyaw)
            goal_yaw_error = min(goal_yaw_error, 2 * math.pi - goal_yaw_error)
        else:
            goal_error = float("nan")
            goal_yaw_error = float("nan")

        # Count jumps during this goal period
        next_g_t = goals[g_idx + 1][0] if g_idx + 1 < len(goals) else t_common[-1]
        jumps_during = sum(1 for j in jumps if g_t <= j["t"] <= next_g_t)

        # Max cross-track error during this goal
        mask = (t_common >= g_t) & (t_common <= next_g_t)
        max_cte = np.nanmax(cross_track_error[mask]) if np.any(mask) else float("nan")

        goal_summaries.append({
            "idx": g_idx + 1,
            "pose": (gx, gy, math.degrees(gyaw)),
            "reached": reached_pose,
            "error_m": goal_error,
            "error_yaw_deg": math.degrees(goal_yaw_error) if not np.isnan(goal_yaw_error) else float("nan"),
            "status": reached_status or "unknown",
            "jumps": jumps_during,
            "max_cte": max_cte,
        })

    # Build summary
    jump_source_lines = "\n".join(
        f"    {src}: {cnt} jumps" for src, cnt in sorted(jump_sources.items(), key=lambda x: -x[1])
    ) if jump_sources else "    No jumps detected"

    goal_lines = []
    for gs in goal_summaries:
        goal_lines.append(
            f"  Goal {gs['idx']}: ({gs['pose'][0]:.2f}, {gs['pose'][1]:.2f}, {gs['pose'][2]:.1f}°) → "
            f"error={gs['error_m']:.3f}m, yaw_err={gs['error_yaw_deg']:.1f}°, "
            f"max_cte={gs['max_cte']:.3f}m, {gs['jumps']} jumps, status={gs['status']}"
        )
    goal_text = "\n".join(goal_lines) if goal_lines else "  No goals detected"

    # Biggest jump
    if len(jumps) > 0:
        biggest = max(jumps, key=lambda j: j["displacement_m"])
        biggest_line = (f"  Largest: {biggest['displacement_m']:.3f}m, "
                        f"{biggest['yaw_change_deg']:.1f}° yaw at t={biggest['t'] - t_common[0]:.1f}s\n"
                        f"  Source: {biggest['source']}")
    else:
        biggest_line = "  No jumps detected — odometry is stable"

    # Per-jump attribution table
    jump_table_lines = []
    for j in jumps:
        jump_table_lines.append(
            f"    t={j['t'] - t_common[0]:7.1f}s | {j['displacement_m']:.3f} m | "
            f"{j['yaw_change_deg']:6.1f}° | div={j['divergence_m']:.3f} m | {j['source']}"
        )
    jump_table = "\n".join(jump_table_lines) if jump_table_lines else "    (none)"

    # Overall metrics
    odo_travel = 0.0
    valid = ~np.isnan(pos_map[:, 0])
    if np.sum(valid) > 1:
        diffs = np.diff(pos_map[valid, :2], axis=0)
        odo_travel = np.sum(np.sqrt(np.sum(diffs ** 2, axis=1)))

    summary = textwrap.dedent(f"""\
    ================================================================================
    NAVIGATION ODOMETRY ANALYSIS — SUMMARY
    ================================================================================
    Bag              : {bag_path}
    Duration         : {duration:.1f} s
    Common timeline  : {len(t_common)} pts @ {1.0 / np.mean(np.diff(t_common)):.1f} Hz

    ---- Topics Recorded ----
    /odom             : {len(fused_recs):6d} messages  frame={fused_frame or '?'}
    /odom_rf2o        : {len(rf2o_recs):6d} messages  frame={rf2o_frame or '?'}
    {topic_leg_display:20s}: {len(leg_recs):6d} messages  frame={leg_frame or '?'}
    map→odom TFs      : {len(map_to_odom_tfs):6d} transforms
    /scan             : {scan_count:6d} messages
    Global plans      : {len(global_plans):6d}
    Local plans       : {len(local_plans):6d}
    Goals             : {len(goals):6d}
    Results           : {len(results):6d}

    ---- Jump Detection (threshold: >{JUMP_DIST_M}m or >{JUMP_YAW_DEG}°) ----
    Total jumps       : {len(jumps)}
    {jump_source_lines}

    {biggest_line}

    ---- Per-Jump Attribution (t | displacement | yaw | TF-vs-topic divergence | source) ----
    {jump_table}

    ---- Per-Goal Breakdown ----
    {goal_text}

    ---- Overall Metrics ----
    Total odometry travel: {odo_travel:.2f} m
    Max cross-track error: {np.nanmax(cross_track_error):.3f} m
    Mean cross-track error: {np.nanmean(cross_track_error):.3f} m

    ---- Jump Classification Guide ----
    cartographer (map→odom TF)     → map→odom TF jumps: relocalizing, scan-match
                                      glitch, or loop closure
    TF conflict (TF ≠ /odom topic) → odom→base_link TF jumps while /odom topic
                                      stays smooth: a second broadcaster (leg/rf2o)
                                      is contaminating the TF tree
    odom topic (/odom)             → /odom topic jumps: input sensor spike or
                                      filter instability

    ---- Interpretation ----
    Check {os.path.join(output_dir, 'nav_odom_stability.png')}:
      - Subplot A (map→odom TF): spikes = cartographer issue
      - Subplot B (/odom): spikes = EKF/sensor issue
      - Subplot C (disagreement): spikes = rf2o vs leg divergence

    Check {os.path.join(output_dir, 'nav_tf_divergence.png')} (THE TF-CONFLICT TEST):
      - If the solid (TF) and dashed (/odom topic) lines separate → a second
        broadcaster is contaminating odom→base_link. This is the root cause of
        "odometry jumps during navigation" when the EKF was active.

    Check {os.path.join(output_dir, 'nav_trajectory.png')}:
      - Red X markers show jump locations on the map-frame trajectory
      - Compare robot path vs planned path for deviations

    ================================================================================
    """)

    summary_path = os.path.join(output_dir, "nav_summary.txt")
    with open(summary_path, "w") as f:
        f.write(summary)
    print(summary)
    print(f"  Saved: nav_summary.txt")
    print("=" * 60)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Full-stack navigation odometry jump diagnostics from a rosbag."
    )
    parser.add_argument("bag_path", help="Path to the .bag file")
    parser.add_argument("--output-dir", "-o", default="./analysis_nav",
                        help="Directory for output plots and summary (default: ./analysis_nav)")
    args = parser.parse_args()

    if not os.path.isfile(args.bag_path):
        print(f"ERROR: bag file not found: {args.bag_path}")
        sys.exit(1)

    analyze(args.bag_path, args.output_dir)


if __name__ == "__main__":
    main()
