#!/usr/bin/env python3
"""
Floor_z_and_quaternion_check.py

Diagnostic tool to check the live camera TF and the apparent floor height in the
forward driving corridor. Subscribes to /grid_map/depth_cloud, filters to a
forward floor corridor, and reports an honest floor profile: the p-th percentile
of z per forward x-bin.

Why per-bin low percentile (not a median): the median of a corridor that contains
furniture/obstacles measures the furniture, not the floor. The floor is the lowest
consistent surface, so a low percentile per horizontal bin tracks the actual
ground and also reveals tilt (near bins vs far bins) vs pure offset (flat).
"""

import argparse
import math
import statistics
import sys

import rospy
import tf
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2


def get_current_tf(parent_frame, child_frame, timeout=3.0):
    listener = tf.TransformListener()
    rospy.sleep(0.5)
    try:
        listener.waitForTransform(parent_frame, child_frame, rospy.Time(0), rospy.Duration(timeout))
        (trans, rot) = listener.lookupTransform(parent_frame, child_frame, rospy.Time(0))
        # rot is [qx, qy, qz, qw]
        matrix = tf.transformations.quaternion_matrix(rot)
        rpy_rad = tf.transformations.euler_from_matrix(matrix)
        rpy_deg = [math.degrees(a) for a in rpy_rad]
        return trans, rot, rpy_deg
    except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logwarn(f"Could not lookup transform {parent_frame} -> {child_frame}: {e}")
        return None, None, None


def point_in_floor_cone(point, min_x, max_x, max_abs_y, max_abs_z):
    x, y, z = point
    return min_x <= x <= max_x and abs(y) < max_abs_y and abs(z) < max_abs_z


def percentile(values, p):
    """Return the p-th percentile (p in [0, 1]) using linear interpolation.

    Returns None for an empty sample. Robust to small bins (no error thrown for
    a single element).
    """
    if not values:
        return None
    s = sorted(values)
    if len(s) == 1:
        return s[0]
    k = (len(s) - 1) * p
    lo = int(math.floor(k))
    hi = int(math.ceil(k))
    if lo == hi:
        return s[lo]
    return s[lo] * (hi - k) + s[hi] * (k - lo)


def main():
    parser = argparse.ArgumentParser(description="Floor z and camera quaternion diagnostic check.")
    parser.add_argument("--topic", type=str, default="/grid_map/depth_cloud", help="PointCloud2 topic to inspect")
    parser.add_argument("--duration", type=float, default=5.0, help="Sampling duration in seconds")
    parser.add_argument("--parent", type=str, default="base_link", help="TF parent frame")
    parser.add_argument("--child", type=str, default="ascamera_hp60c_camera_link_0", help="TF child frame")
    parser.add_argument("--tolerance", type=float, default=0.05, help="Floor z acceptable tolerance (meters)")
    parser.add_argument("--min-x", type=float, default=0.5, help="Minimum forward x distance (meters)")
    parser.add_argument("--max-x", type=float, default=2.5, help="Maximum forward x distance (meters)")
    parser.add_argument("--max-abs-y", type=float, default=0.4, help="Maximum lateral |y| distance (meters)")
    parser.add_argument("--max-abs-z", type=float, default=0.6, help="Maximum vertical |z| distance (meters)")
    parser.add_argument("--bin-width", type=float, default=0.25, help="Forward x-bin width for the floor profile (meters)")
    parser.add_argument("--floor-percentile", type=float, default=0.05,
                        help="Percentile of z used as the floor height per bin (0.05 = 5th percentile)")
    parser.add_argument("--tilt-threshold", type=float, default=0.05,
                        help="Max near-vs-far floor spread (m) before flagging tilt instead of pure offset")
    parser.add_argument("--target-floor-z", type=float, default=0.0,
                        help="Expected floor z in the cloud frame (m). Odom-frame floor is 0.0: EKF publishes "
                             "odom->base_footprint at z=0 (ground) and base_footprint->base_link is a +0.45 static lift.")
    args, _ = parser.parse_known_args()

    rospy.init_node("floor_z_and_quaternion_check", anonymous=True)

    print("\n" + "=" * 65)
    print("      FLOOR Z & CAMERA QUATERNION DIAGNOSTIC TEST")
    print("=" * 65)

    trans, rot, rpy_deg = get_current_tf(args.parent, args.child)

    total_points = 0
    cone_points = []  # list of (x, z) for every point inside the floor cone

    def cloud_callback(msg):
        nonlocal total_points
        for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            total_points += 1
            x, y, z = point
            if not all(math.isfinite(value) for value in (x, y, z)):
                continue
            if point_in_floor_cone((x, y, z), args.min_x, args.max_x, args.max_abs_y, args.max_abs_z):
                cone_points.append((x, z))

    sub = rospy.Subscriber(args.topic, PointCloud2, cloud_callback, queue_size=10)

    print(f"\n[1/3] Collecting point cloud samples from {args.topic} for {args.duration:.1f}s...")
    rate = rospy.Rate(20)
    t_start = rospy.Time.now()
    while not rospy.is_shutdown() and (rospy.Time.now() - t_start).to_sec() < args.duration:
        rate.sleep()
    sub.unregister()

    print("\n[2/3] Analyzing floor-corridor points...")

    if total_points == 0:
        print("\nERROR: No points received on topic:", args.topic)
        print("Result: [STATUS: REJECTED / NO DATA]")
        sys.exit(1)

    if not cone_points:
        print("\nERROR: Point cloud was received, but no points landed inside the requested floor cone.")
        print(f"Cone: x in [{args.min_x:.2f}, {args.max_x:.2f}], |y| < {args.max_abs_y:.2f}, |z| < {args.max_abs_z:.2f}")
        print("Result: [STATUS: REJECTED / NO DATA]")
        sys.exit(1)

    # --- All summary statistics come from this ONE filtered array ---
    all_z = sorted(z for _, z in cone_points)
    cone_count = len(all_z)
    z_min = all_z[0]
    z_max = all_z[-1]
    z_mean = sum(all_z) / cone_count
    z_median = statistics.median(all_z)
    z_p5_overall = percentile(all_z, args.floor_percentile)

    # --- Floor profile: p-th percentile of z per forward x-bin ---
    bin_width = args.bin_width
    n_bins = max(1, int(round((args.max_x - args.min_x) / bin_width)))
    bins = [[] for _ in range(n_bins)]
    for x, z in cone_points:
        idx = int(math.floor((x - args.min_x) / bin_width))
        if 0 <= idx < n_bins:
            bins[idx].append(z)

    floor_profile = []  # (x_lo, x_hi, count, floor_z)
    for i, zs in enumerate(bins):
        x_lo = args.min_x + i * bin_width
        x_hi = x_lo + bin_width
        floor_profile.append((x_lo, x_hi, len(zs), percentile(zs, args.floor_percentile)))

    valid_floor = [fz for (_, _, cnt, fz) in floor_profile if fz is not None]
    if not valid_floor:
        print("\nERROR: No populated x-bins; cannot compute a floor profile.")
        sys.exit(1)

    floor_estimate = statistics.median(valid_floor)
    floor_spread = max(valid_floor) - min(valid_floor)
    target_floor_z = args.target_floor_z
    z_err = floor_estimate - target_floor_z
    is_accepted = abs(z_err) <= args.tolerance

    # --- Report ---
    print("\n" + "-" * 65)
    print("                  CURRENT TF TRANSFORM")
    print("-" * 65)
    if trans is not None and rot is not None:
        print(f"Frames:          {args.parent} -> {args.child}")
        print(f"Translation:     X={trans[0]:.3f} m, Y={trans[1]:.3f} m, Z={trans[2]:.3f} m")
        print(f"Quaternion:      [{rot[0]:.6f}, {rot[1]:.6f}, {rot[2]:.6f}, {rot[3]:.6f}]")
        print(f"RPY (degrees):   Roll={rpy_deg[0]:.2f}°, Pitch={rpy_deg[1]:.2f}°, Yaw={rpy_deg[2]:.2f}°")
    else:
        print(f"TF lookup failed for {args.parent} -> {args.child}")

    print("\n" + "-" * 65)
    print("                FLOOR CORRIDOR SAMPLING METRICS")
    print("-" * 65)
    print(f"Topic:           {args.topic}")
    print(f"Cone Filter:     x in [{args.min_x:.2f}, {args.max_x:.2f}], |y| < {args.max_abs_y:.2f}, |z| < {args.max_abs_z:.2f}")
    print(f"Total Points:    {total_points}")
    print(f"Cone Points:     {cone_count}")
    print(f"Cone Z Min/Max:  {z_min:.4f} m / {z_max:.4f} m")
    print(f"Cone Z Mean:     {z_mean:.4f} m   (median {z_median:.4f} m — includes furniture, not floor)")
    print(f"Overall {args.floor_percentile:.0%}ile Z: {z_p5_overall:.4f} m")

    print("\n" + "-" * 65)
    print(f"       FLOOR PROFILE ({args.floor_percentile:.0%}ile z per {bin_width:.2f} m x-bin)")
    print("-" * 65)
    print(f"{'x range (m)':>16} {'points':>8} {'floor z (m)':>12}")
    for x_lo, x_hi, cnt, fz in floor_profile:
        if fz is None:
            print(f"  [{x_lo:5.2f},{x_hi:5.2f}]  {cnt:>6}  {'--':>10}")
        else:
            print(f"  [{x_lo:5.2f},{x_hi:5.2f}]  {cnt:>6}  {fz:>10.4f}")

    print("\n" + "=" * 65)
    print("                    TEST EVALUATION")
    print("=" * 65)
    print(f"  Floor estimate (median of per-bin floor z): {floor_estimate:.4f} m")
    print(f"  Target: {target_floor_z:.2f} m ± {args.tolerance:.2f} m  ->  error {z_err:+.4f} m")

    if is_accepted:
        print("  ACCEPTANCE RESULT: [ ACCEPTED / PASS ]")
    else:
        print("  ACCEPTANCE RESULT: [ REJECTED / FAIL ]")

    if floor_spread > args.tilt_threshold:
        print(f"  Diagnosis: floor z varies across range (spread {floor_spread:.4f} m > {args.tilt_threshold:.2f} m)")
        print("             -> TILT / pitch / depth-scale error, not a pure offset.")
        print("             Near-to-far slope: ", end="")
        first = floor_profile[0][3]
        last = floor_profile[-1][3]
        if first is not None and last is not None:
            print(f"{first:+.4f} m -> {last:+.4f} m")
        else:
            print("(insufficient end-bins)")
    else:
        print(f"  Diagnosis: floor z is flat across range (spread {floor_spread:.4f} m <= {args.tilt_threshold:.2f} m)")
        print("             -> pure height OFFSET. Shift camera extrinsic z by the reported error sign.")
    print("=" * 65 + "\n")

    sys.exit(0 if is_accepted else 2)


if __name__ == "__main__":
    main()
