#!/usr/bin/env python3
"""
anylize_Scan_planner.py — Offline SCAN planner diagnostics from a rosbag.

This analyzer is focused on the depth-mode failure modes seen in this workspace:
- false floor obstacles
- floor ramp / world-pitch inconsistency
- TF / extrinsic pose stability
- obstacle height / distance plausibility
- LiDAR-vs-depth scale mismatch
- image-column bias that suggests radial-vs-orthogonal depth interpretation bugs
- occupancy filter effectiveness for floor clutter

Usage:
  /usr/bin/python3 scripts/anylize_Scan_planner.py <bag_path> [-o output_dir]
"""

import argparse
import math
import os
import sys
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
    import sensor_msgs.point_cloud2 as pc2
except ImportError:
    print("ERROR: sensor_msgs.point_cloud2 not found. Source your ROS setup.bash first.")
    sys.exit(1)

TOPIC_DEPTH_IMAGE = "/ascamera_hp60c/depth0/image_raw"
TOPIC_CAMERA_INFO = "/ascamera_hp60c/depth0/camera_info"
TOPIC_CAMERA_POINTS = "/ascamera_hp60c/depth0/points"
TOPIC_SCAN = "/scan"
TOPIC_SCAN_MATCHED_POINTS = "/scan_matched_points2"
TOPIC_TF = "/tf"
TOPIC_TF_STATIC = "/tf_static"
TOPIC_DEPTH_CLOUD = "/grid_map/depth_cloud"
TOPIC_OCCUPANCY_INFLATE = "/grid_map/occupancy_inflate"
TOPIC_EXTRINSIC_POSE = "/grid_map/sensor_pose_extrinsic"

FLOOR_X_BINS = [(0.5, 1.0), (1.0, 1.5), (1.5, 2.0), (2.0, 2.5)]
COLUMN_BANDS = {
    "left": (0.5, 1.0),
    "center": (-0.2, 0.2),
    "right": (-1.0, -0.5),
}


def quat_to_rpy_deg(x, y, z, w):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return np.degrees([roll, pitch, yaw])


def percentile(values, q):
    if len(values) == 0:
        return None
    return float(np.percentile(values, q))


def fit_line(x, y):
    if len(x) < 2:
        return None, None, None
    coeffs = np.polyfit(x, y, 1)
    y_hat = coeffs[0] * x + coeffs[1]
    ss_res = np.sum((y - y_hat) ** 2)
    ss_tot = np.sum((y - np.mean(y)) ** 2)
    r2 = 1.0 if ss_tot < 1e-12 else 1.0 - (ss_res / ss_tot)
    return coeffs[0], coeffs[1], r2


def ensure_dir(path):
    os.makedirs(path, exist_ok=True)


def read_cloud_points(msg):
    points = []
    for pt in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        x, y, z = pt
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            continue
        points.append((x, y, z))
    if not points:
        return np.empty((0, 3), dtype=np.float64)
    return np.asarray(points, dtype=np.float64)


def summarize_topics(bag):
    info = bag.get_type_and_topic_info()[1]
    required = [
        TOPIC_DEPTH_IMAGE,
        TOPIC_CAMERA_INFO,
        TOPIC_SCAN,
        TOPIC_SCAN_MATCHED_POINTS,
        TOPIC_TF,
        TOPIC_TF_STATIC,
        TOPIC_DEPTH_CLOUD,
        TOPIC_OCCUPANCY_INFLATE,
        TOPIC_EXTRINSIC_POSE,
    ]
    result = {}
    for topic in required:
        topic_info = info.get(topic)
        if topic_info is None:
            result[topic] = None
        else:
            result[topic] = {
                "type": topic_info.msg_type,
                "count": topic_info.message_count,
                "frequency": topic_info.frequency,
            }
    return result


def analyze_camera_info(bag):
    for _, msg, _ in bag.read_messages(topics=[TOPIC_CAMERA_INFO]):
        return {
            "frame_id": msg.header.frame_id,
            "width": msg.width,
            "height": msg.height,
            "K": list(msg.K),
            "P": list(msg.P),
        }
    return None


def analyze_extrinsic_pose(bag):
    poses = []
    for _, msg, _ in bag.read_messages(topics=[TOPIC_EXTRINSIC_POSE]):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        rpy = quat_to_rpy_deg(ori.x, ori.y, ori.z, ori.w)
        poses.append({
            "t": msg.header.stamp.to_sec(),
            "frame_id": msg.header.frame_id,
            "child_frame_id": msg.child_frame_id,
            "x": pos.x,
            "y": pos.y,
            "z": pos.z,
            "roll": float(rpy[0]),
            "pitch": float(rpy[1]),
            "yaw": float(rpy[2]),
        })
    return poses


def analyze_depth_clouds(bag):
    cloud_stats = []
    floor_bin_rows = []
    floor_fit_x = []
    floor_fit_z = []
    floor_column_rows = []
    obstacle_samples = []
    frame_cache = []

    for _, msg, t in bag.read_messages(topics=[TOPIC_DEPTH_CLOUD]):
        pts = read_cloud_points(msg)
        if pts.size == 0:
            continue

        ts = t.to_sec()
        x = pts[:, 0]
        y = pts[:, 1]
        z = pts[:, 2]
        rng = np.linalg.norm(pts, axis=1)
        frame_cache.append((ts, pts))

        cone_mask = (x >= 0.5) & (x <= 2.5) & (np.abs(y) <= 0.4) & (np.abs(z) <= 0.8)
        cone = pts[cone_mask]
        far_count = int(np.sum((x >= 2.0) & (x <= 2.5)))

        cloud_stats.append({
            "t": ts,
            "count": len(pts),
            "range_min": float(np.min(rng)),
            "range_p50": float(np.percentile(rng, 50)),
            "range_p95": float(np.percentile(rng, 95)),
            "range_max": float(np.max(rng)),
            "z_min": float(np.min(z)),
            "z_p05": float(np.percentile(z, 5)),
            "z_p50": float(np.percentile(z, 50)),
            "z_p95": float(np.percentile(z, 95)),
            "z_max": float(np.max(z)),
            "far_count": far_count,
        })

        if cone.size > 0:
            cone_x = cone[:, 0]
            cone_y = cone[:, 1]
            cone_z = cone[:, 2]

            for lo, hi in FLOOR_X_BINS:
                mask = (cone_x >= lo) & (cone_x < hi)
                if np.count_nonzero(mask) == 0:
                    floor_bin_rows.append({
                        "t": ts,
                        "x_bin": f"[{lo:.1f}, {hi:.1f})",
                        "n_points": 0,
                        "floor_z_p5": None,
                        "floor_z_p50": None,
                    })
                    continue
                z_values = cone_z[mask]
                p5 = float(np.percentile(z_values, 5))
                p50 = float(np.percentile(z_values, 50))
                floor_bin_rows.append({
                    "t": ts,
                    "x_bin": f"[{lo:.1f}, {hi:.1f})",
                    "n_points": int(len(z_values)),
                    "floor_z_p5": p5,
                    "floor_z_p50": p50,
                })
                floor_fit_x.append((lo + hi) * 0.5)
                floor_fit_z.append(p5)

            x_range_mask = (cone_x >= 0.75) & (cone_x <= 1.75)
            if np.count_nonzero(x_range_mask) > 0:
                for label, (y_lo, y_hi) in COLUMN_BANDS.items():
                    band_mask = x_range_mask & (cone_y >= y_lo) & (cone_y < y_hi)
                    if np.count_nonzero(band_mask) == 0:
                        continue
                    floor_points = cone_z[band_mask]
                    floor_column_rows.append({
                        "t": ts,
                        "band": label,
                        "n_points": int(len(floor_points)),
                        "floor_z_mean": float(np.mean(floor_points)),
                        "floor_z_p5": float(np.percentile(floor_points, 5)),
                    })

        obstacle_mask = (x >= 0.3) & (x <= 2.5) & (np.abs(y) <= 1.0)
        obstacle_pts = pts[obstacle_mask]
        if obstacle_pts.size > 0:
            obstacle_samples.append({
                "t": ts,
                "min_x": float(np.min(obstacle_pts[:, 0])),
                "max_x": float(np.max(obstacle_pts[:, 0])),
                "min_z": float(np.min(obstacle_pts[:, 2])),
                "max_z": float(np.max(obstacle_pts[:, 2])),
                "p50_z": float(np.percentile(obstacle_pts[:, 2], 50)),
                "p95_z": float(np.percentile(obstacle_pts[:, 2], 95)),
            })

    return {
        "cloud_stats": cloud_stats,
        "floor_bin_rows": floor_bin_rows,
        "floor_fit_x": np.asarray(floor_fit_x),
        "floor_fit_z": np.asarray(floor_fit_z),
        "floor_column_rows": floor_column_rows,
        "obstacle_samples": obstacle_samples,
        "frames": frame_cache,
    }


def scan_to_front_range(scan_msg, max_abs_angle_deg=20.0):
    ranges = []
    angle = scan_msg.angle_min
    for value in scan_msg.ranges:
        if abs(math.degrees(angle)) <= max_abs_angle_deg and math.isfinite(value):
            if scan_msg.range_min <= value <= scan_msg.range_max:
                ranges.append(value)
        angle += scan_msg.angle_increment
    if not ranges:
        return None
    return float(min(ranges))


def depth_cloud_front_range(points, max_abs_angle_deg=20.0):
    if points.size == 0:
        return None
    x = points[:, 0]
    y = points[:, 1]
    angles = np.degrees(np.arctan2(y, x))
    mask = (x > 0.0) & (np.abs(angles) <= max_abs_angle_deg)
    if np.count_nonzero(mask) == 0:
        return None
    front_pts = points[mask]
    dists = np.linalg.norm(front_pts[:, :2], axis=1)
    if len(dists) == 0:
        return None
    return float(np.min(dists))


def analyze_depth_scale_against_lidar(bag, depth_frames):
    if not depth_frames:
        return []

    depth_times = np.asarray([t for t, _ in depth_frames])
    ratios = []
    for _, scan_msg, t in bag.read_messages(topics=[TOPIC_SCAN]):
        lidar_dist = scan_to_front_range(scan_msg)
        if lidar_dist is None:
            continue
        scan_t = t.to_sec()
        idx = int(np.argmin(np.abs(depth_times - scan_t)))
        if abs(depth_times[idx] - scan_t) > 0.2:
            continue
        depth_dist = depth_cloud_front_range(depth_frames[idx][1])
        if depth_dist is None or lidar_dist < 1e-6:
            continue
        ratios.append({
            "t": scan_t,
            "lidar_wall_distance": lidar_dist,
            "depth_wall_distance": depth_dist,
            "ratio": depth_dist / lidar_dist,
        })
    return ratios


def analyze_occupancy_filter(bag):
    rows = []
    for _, msg, t in bag.read_messages(topics=[TOPIC_OCCUPANCY_INFLATE]):
        pts = read_cloud_points(msg)
        if pts.size == 0:
            continue
        z = pts[:, 2]
        floor_occ = int(np.sum(z < 0.15))
        raised_occ = int(np.sum(z >= 0.15))
        total = int(len(z))
        ratio = float(floor_occ) / total if total > 0 else 0.0
        rows.append({
            "t": t.to_sec(),
            "floor_occ": floor_occ,
            "raised_occ": raised_occ,
            "total_occ": total,
            "floor_ratio": ratio,
        })
    return rows


def write_table(f, title, rows, columns):
    f.write(title + "\n")
    f.write("-" * len(title) + "\n")
    if not rows:
        f.write("No data.\n\n")
        return
    header = " | ".join(columns)
    f.write(header + "\n")
    f.write("-" * len(header) + "\n")
    for row in rows:
        values = []
        for col in columns:
            value = row.get(col)
            if isinstance(value, float):
                values.append(f"{value:.4f}")
            else:
                values.append(str(value))
        f.write(" | ".join(values) + "\n")
    f.write("\n")


def write_summary(path, topic_summary, camera_info, depth_results, extrinsic_poses, depth_scale_rows, occupancy_rows):
    cloud_stats = depth_results["cloud_stats"]
    floor_fit_x = depth_results["floor_fit_x"]
    floor_fit_z = depth_results["floor_fit_z"]
    obstacle_samples = depth_results["obstacle_samples"]
    floor_bin_rows = depth_results["floor_bin_rows"]
    floor_column_rows = depth_results["floor_column_rows"]
    slope, intercept, r2 = fit_line(floor_fit_x, floor_fit_z)

    with open(path, "w", encoding="utf-8") as f:
        f.write("SCAN Planner Offline Diagnostics\n")
        f.write("=" * 60 + "\n\n")

        f.write("Recorded Topic Coverage\n")
        f.write("-" * 30 + "\n")
        for topic, info in topic_summary.items():
            if info is None:
                f.write(f"[--] {topic}\n")
            else:
                f.write(f"[OK] {topic}  type={info['type']}  count={info['count']}  freq={info['frequency']:.3f}\n")
        f.write("\n")

        f.write("Camera Info Summary\n")
        f.write("-" * 30 + "\n")
        if camera_info is None:
            f.write("No camera_info found.\n\n")
        else:
            f.write(f"frame_id: {camera_info['frame_id']}\n")
            f.write(f"width/height: {camera_info['width']} x {camera_info['height']}\n")
            f.write(f"K: {camera_info['K']}\n")
            f.write(f"P: {camera_info['P']}\n\n")

        f.write("Depth Cloud Summary\n")
        f.write("-" * 30 + "\n")
        if not cloud_stats:
            f.write("No /grid_map/depth_cloud messages found.\n\n")
        else:
            f.write(f"Frames analyzed: {len(cloud_stats)}\n")
            f.write(f"Point count range: {min(s['count'] for s in cloud_stats)} .. {max(s['count'] for s in cloud_stats)}\n")
            f.write(f"Range p95 span: {min(s['range_p95'] for s in cloud_stats):.3f} .. {max(s['range_p95'] for s in cloud_stats):.3f} m\n")
            f.write(f"Far-point count (2.0m<=x<=2.5m): min {min(s['far_count'] for s in cloud_stats)}, max {max(s['far_count'] for s in cloud_stats)}\n")
            f.write("\n")

        write_table(
            f,
            "Near vs Far Floor Bins",
            floor_bin_rows,
            ["t", "x_bin", "n_points", "floor_z_p5", "floor_z_p50"],
        )

        f.write("Floor Ramp Fit\n")
        f.write("-" * 30 + "\n")
        if len(floor_fit_x) < 2:
            f.write("Not enough floor bins for line fit.\n\n")
        else:
            spread = float(np.max(floor_fit_z) - np.min(floor_fit_z))
            f.write(f"Line fit: z = {slope:.4f} * x + {intercept:.4f}, R^2 = {r2:.4f}\n")
            f.write(f"Floor z spread: {spread:.4f} m\n")
            if r2 is not None and r2 > 0.98:
                f.write("Interpretation: floor profile is close to a synthetic straight line.\n")
            if spread > 0.10:
                f.write("Interpretation: floor ramp is significant.\n")
            f.write("\n")

        write_table(
            f,
            "Floor Points per Pixel Band",
            floor_column_rows,
            ["t", "band", "n_points", "floor_z_mean", "floor_z_p5"],
        )

        f.write("Depth Scale Check Against LiDAR\n")
        f.write("-" * 30 + "\n")
        if not depth_scale_rows:
            f.write("No scan/depth pairs found for front-wall ratio check.\n\n")
        else:
            ratios = np.array([row["ratio"] for row in depth_scale_rows])
            f.write(f"Frames compared: {len(depth_scale_rows)}\n")
            f.write(f"ratio mean/std: {np.mean(ratios):.4f} / {np.std(ratios):.4f}\n")
            f.write(f"ratio min/max: {np.min(ratios):.4f} / {np.max(ratios):.4f}\n")
            if np.mean(ratios) > 1.05:
                f.write("Interpretation: depth appears over-scaled relative to LiDAR.\n")
            elif np.mean(ratios) < 0.95:
                f.write("Interpretation: depth appears under-scaled relative to LiDAR.\n")
            else:
                f.write("Interpretation: average depth/LiDAR wall ratio is close to 1.0.\n")
            f.write("\n")
            write_table(
                f,
                "Per-frame Depth/LiDAR Wall Ratio",
                depth_scale_rows,
                ["t", "lidar_wall_distance", "depth_wall_distance", "ratio"],
            )

        f.write("Extrinsic / TF Stability\n")
        f.write("-" * 30 + "\n")
        if not extrinsic_poses:
            f.write("No /grid_map/sensor_pose_extrinsic messages found.\n\n")
        else:
            zs = np.array([p["z"] for p in extrinsic_poses])
            pitches = np.array([p["pitch"] for p in extrinsic_poses])
            f.write(f"Extrinsic samples: {len(extrinsic_poses)}\n")
            f.write(f"z mean/std: {np.mean(zs):.4f} / {np.std(zs):.4f} m\n")
            f.write(f"pitch mean/std: {np.mean(pitches):.4f} / {np.std(pitches):.4f} deg\n")
            f.write(f"First frame: {extrinsic_poses[0]['frame_id']} -> {extrinsic_poses[0]['child_frame_id']}\n\n")

        f.write("Obstacle Height / Distance Summary\n")
        f.write("-" * 30 + "\n")
        if not obstacle_samples:
            f.write("No obstacle-region samples extracted.\n\n")
        else:
            p95_heights = [s["p95_z"] for s in obstacle_samples]
            min_ranges = [s["min_x"] for s in obstacle_samples]
            max_ranges = [s["max_x"] for s in obstacle_samples]
            f.write(f"Obstacle samples: {len(obstacle_samples)}\n")
            f.write(f"Obstacle p95 z range: {min(p95_heights):.3f} .. {max(p95_heights):.3f} m\n")
            f.write(f"Obstacle forward distance span: {min(min_ranges):.3f} .. {max(max_ranges):.3f} m\n\n")

        f.write("Filter Effectiveness Proof\n")
        f.write("-" * 30 + "\n")
        if not occupancy_rows:
            f.write("No /grid_map/occupancy_inflate messages found.\n")
        else:
            floor_occ = np.array([row["floor_occ"] for row in occupancy_rows])
            raised_occ = np.array([row["raised_occ"] for row in occupancy_rows])
            floor_ratio = np.array([row["floor_ratio"] for row in occupancy_rows])
            f.write(f"Frames analyzed: {len(occupancy_rows)}\n")
            f.write(f"floor_occ mean: {np.mean(floor_occ):.2f}\n")
            f.write(f"raised_occ mean: {np.mean(raised_occ):.2f}\n")
            f.write(f"floor_ratio mean: {np.mean(floor_ratio):.4f}\n\n")
            write_table(
                f,
                "Per-frame Occupancy Ratio",
                occupancy_rows,
                ["t", "floor_occ", "raised_occ", "total_occ", "floor_ratio"],
            )


def plot_floor_profile(output_dir, depth_results):
    floor_fit_x = depth_results["floor_fit_x"]
    floor_fit_z = depth_results["floor_fit_z"]
    if len(floor_fit_x) < 2:
        return
    slope, intercept, r2 = fit_line(floor_fit_x, floor_fit_z)
    xs = np.linspace(np.min(floor_fit_x), np.max(floor_fit_x), 100)
    ys = slope * xs + intercept
    plt.figure(figsize=(8, 5))
    plt.scatter(floor_fit_x, floor_fit_z, s=20, label="floor z p5")
    plt.plot(xs, ys, label=f"fit R^2={r2:.3f}")
    plt.xlabel("forward x (m)")
    plt.ylabel("floor z p5 (m)")
    plt.title("Near/Far Floor Profile")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, "scan_floor_profile.png"), dpi=150)
    plt.close()


def plot_depth_range(output_dir, cloud_stats):
    if not cloud_stats:
        return
    times = np.array([row["t"] for row in cloud_stats])
    rel_t = times - times[0]
    plt.figure(figsize=(10, 6))
    plt.plot(rel_t, [row["range_p95"] for row in cloud_stats], label="range p95")
    plt.plot(rel_t, [row["range_max"] for row in cloud_stats], label="range max")
    plt.plot(rel_t, [row["far_count"] for row in cloud_stats], label="points 2.0m<=x<=2.5m")
    plt.xlabel("time (s)")
    plt.ylabel("value")
    plt.title("Depth Cloud Range / Far-point Persistence")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, "scan_depth_range.png"), dpi=150)
    plt.close()


def plot_extrinsic_stability(output_dir, extrinsic_poses):
    if not extrinsic_poses:
        return
    times = np.array([row["t"] for row in extrinsic_poses])
    rel_t = times - times[0]
    plt.figure(figsize=(10, 6))
    plt.plot(rel_t, [row["z"] for row in extrinsic_poses], label="z (m)")
    plt.plot(rel_t, [row["pitch"] for row in extrinsic_poses], label="pitch (deg)")
    plt.xlabel("time (s)")
    plt.ylabel("value")
    plt.title("Sensor Pose Extrinsic Stability")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, "scan_extrinsic_stability.png"), dpi=150)
    plt.close()


def plot_depth_scale(output_dir, depth_scale_rows):
    if not depth_scale_rows:
        return
    ratios = np.array([row["ratio"] for row in depth_scale_rows])
    times = np.array([row["t"] for row in depth_scale_rows])
    rel_t = times - times[0]

    plt.figure(figsize=(10, 5))
    plt.plot(rel_t, ratios, marker=".")
    plt.axhline(1.0, color="k", linestyle="--", linewidth=1.0)
    plt.axhline(1.02, color="g", linestyle=":", linewidth=1.0)
    plt.axhline(0.98, color="g", linestyle=":", linewidth=1.0)
    plt.axhline(1.05, color="r", linestyle=":", linewidth=1.0)
    plt.xlabel("time (s)")
    plt.ylabel("depth / lidar ratio")
    plt.title("Depth Scale vs LiDAR")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, "scan_depth_scale_timeseries.png"), dpi=150)
    plt.close()

    plt.figure(figsize=(7, 5))
    plt.hist(ratios, bins=30)
    plt.xlabel("depth / lidar ratio")
    plt.ylabel("count")
    plt.title("Depth Scale Ratio Histogram")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, "scan_depth_scale_hist.png"), dpi=150)
    plt.close()


def plot_floor_column_bias(output_dir, floor_column_rows):
    if not floor_column_rows:
        return
    grouped = defaultdict(list)
    for row in floor_column_rows:
        grouped[row["band"]].append(row["floor_z_mean"])

    labels = []
    values = []
    for label in ["left", "center", "right"]:
        if label in grouped:
            labels.append(label)
            values.append(np.mean(grouped[label]))

    if not labels:
        return

    plt.figure(figsize=(7, 5))
    plt.bar(labels, values)
    plt.ylabel("mean floor z (m)")
    plt.title("Floor Height by Column Band")
    plt.grid(True, alpha=0.3, axis="y")
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, "scan_floor_column_bias.png"), dpi=150)
    plt.close()


def plot_occupancy_filter(output_dir, occupancy_rows):
    if not occupancy_rows:
        return
    times = np.array([row["t"] for row in occupancy_rows])
    rel_t = times - times[0]
    plt.figure(figsize=(10, 6))
    plt.plot(rel_t, [row["floor_occ"] for row in occupancy_rows], label="z < 0.15")
    plt.plot(rel_t, [row["raised_occ"] for row in occupancy_rows], label="z >= 0.15")
    plt.plot(rel_t, [row["floor_ratio"] for row in occupancy_rows], label="floor ratio")
    plt.xlabel("time (s)")
    plt.ylabel("count / ratio")
    plt.title("Occupancy Filter Effectiveness")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, "scan_filter_effectiveness.png"), dpi=150)
    plt.close()


def make_plots(output_dir, depth_results, extrinsic_poses, depth_scale_rows, occupancy_rows):
    plot_depth_range(output_dir, depth_results["cloud_stats"])
    plot_floor_profile(output_dir, depth_results)
    plot_extrinsic_stability(output_dir, extrinsic_poses)
    plot_depth_scale(output_dir, depth_scale_rows)
    plot_floor_column_bias(output_dir, depth_results["floor_column_rows"])
    plot_occupancy_filter(output_dir, occupancy_rows)


def analyze(bag_path, output_dir):
    ensure_dir(output_dir)
    print("=" * 60)
    print(f"Analyzing SCAN bag: {bag_path}")
    print(f"Output dir       : {output_dir}")
    print("=" * 60)

    bag = rosbag.Bag(bag_path, "r")
    topic_summary = summarize_topics(bag)
    camera_info = analyze_camera_info(bag)

    print("\n[1/5] Reading depth cloud and floor statistics ...")
    depth_results = analyze_depth_clouds(bag)

    print("[2/5] Reading extrinsic stability ...")
    extrinsic_poses = analyze_extrinsic_pose(bag)

    print("[3/5] Comparing depth cloud against LiDAR front-wall range ...")
    depth_scale_rows = analyze_depth_scale_against_lidar(bag, depth_results["frames"])

    print("[4/5] Reading occupancy inflate filter metrics ...")
    occupancy_rows = analyze_occupancy_filter(bag)

    bag.close()

    print("[5/5] Writing summary and plots ...")
    summary_path = os.path.join(output_dir, "scan_planner_summary.txt")
    write_summary(summary_path, topic_summary, camera_info, depth_results, extrinsic_poses, depth_scale_rows, occupancy_rows)
    make_plots(output_dir, depth_results, extrinsic_poses, depth_scale_rows, occupancy_rows)

    print("Done.")
    print(f"Summary: {summary_path}")
    for name in [
        "scan_depth_range.png",
        "scan_floor_profile.png",
        "scan_extrinsic_stability.png",
        "scan_depth_scale_timeseries.png",
        "scan_depth_scale_hist.png",
        "scan_floor_column_bias.png",
        "scan_filter_effectiveness.png",
    ]:
        path = os.path.join(output_dir, name)
        if os.path.exists(path):
            print(f"Plot   : {path}")


def main():
    parser = argparse.ArgumentParser(description="Offline SCAN planner rosbag analyzer")
    parser.add_argument("bag_path", help="Path to rosbag file")
    parser.add_argument("-o", "--output-dir", default=None, help="Directory for outputs")
    args = parser.parse_args()

    bag_path = os.path.abspath(args.bag_path)
    if not os.path.isfile(bag_path):
        print(f"ERROR: bag file not found: {bag_path}")
        sys.exit(1)

    output_dir = args.output_dir
    if output_dir is None:
        base = os.path.splitext(os.path.basename(bag_path))[0]
        output_dir = os.path.join(os.path.dirname(bag_path), f"{base}_scan_analysis")
    output_dir = os.path.abspath(output_dir)

    analyze(bag_path, output_dir)


if __name__ == "__main__":
    main()
