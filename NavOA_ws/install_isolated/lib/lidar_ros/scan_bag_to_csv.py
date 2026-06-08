#!/usr/bin/env python3

import argparse
import csv
import math
import os
import sys
from datetime import datetime


def load_rosbag_module():
    try:
        import rosbag  # type: ignore

        return rosbag
    except ModuleNotFoundError as exc:
        system_python = "/usr/bin/python3"
        if sys.executable != system_python and os.path.exists(system_python):
            print(
                "Current Python cannot import rosbag dependencies; retrying with /usr/bin/python3",
                file=sys.stderr,
            )
            os.execv(system_python, [system_python, os.path.abspath(__file__), *sys.argv[1:]])
        raise RuntimeError(
            "Unable to import rosbag. Install the missing ROS Python dependencies or run this script with /usr/bin/python3."
        ) from exc


rosbag = load_rosbag_module()


def default_output_path(bag_path):
    package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    bag_stem = os.path.splitext(os.path.basename(bag_path))[0]
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.join(package_dir, f"{bag_stem}_{timestamp}.csv")


def sanitize_values(values):
    sanitized = []
    for value in values:
        if math.isinf(value) or math.isnan(value):
            sanitized.append("")
        else:
            sanitized.append(value)
    return sanitized


def build_header(range_count, intensity_count):
    range_columns = [f"range_{index}" for index in range(range_count)]
    intensity_columns = [f"intensity_{index}" for index in range(intensity_count)]
    header = [
        "seq",
        "stamp_sec",
        "stamp_nsec",
        "angle_min",
        "angle_max",
        "angle_increment",
        "time_increment",
        "scan_time",
        "range_min",
        "range_max",
    ]
    header.extend(range_columns)
    header.extend(intensity_columns)
    return header


def pad_values(values, target_length):
    padded = list(values)
    if len(padded) < target_length:
        padded.extend([""] * (target_length - len(padded)))
    return padded[:target_length]


def build_row(scan, range_count, intensity_count):
    row = [
        scan.header.seq,
        scan.header.stamp.secs,
        scan.header.stamp.nsecs,
        scan.angle_min,
        scan.angle_max,
        scan.angle_increment,
        scan.time_increment,
        scan.scan_time,
        scan.range_min,
        scan.range_max,
    ]
    row.extend(pad_values(sanitize_values(scan.ranges), range_count))
    row.extend(pad_values(sanitize_values(scan.intensities), intensity_count))
    return row


def collect_scans(bag_path, scan_topic, max_scans):
    scans = []
    max_range_count = 0
    max_intensity_count = 0

    with rosbag.Bag(bag_path, "r") as bag:
        for _, scan, _ in bag.read_messages(topics=[scan_topic]):
            if getattr(scan, "_type", "") != "sensor_msgs/LaserScan":
                continue

            scans.append(scan)
            max_range_count = max(max_range_count, len(scan.ranges))
            max_intensity_count = max(max_intensity_count, len(scan.intensities))

            if max_scans > 0 and len(scans) >= max_scans:
                break

    return scans, max_range_count, max_intensity_count


def parse_args():
    parser = argparse.ArgumentParser(
        description="Convert LaserScan messages from a ROS bag into the lidar_ros CSV format."
    )
    parser.add_argument("bag_path", help="Path to the input .bag file")
    parser.add_argument(
        "--scan-topic",
        default="/scan",
        help="LaserScan topic to export from the bag (default: /scan)",
    )
    parser.add_argument(
        "--output-csv",
        default=None,
        help="Path to the output CSV file (default: saved in lidar_ros directory)",
    )
    parser.add_argument(
        "--max-scans",
        type=int,
        default=0,
        help="Maximum number of scans to export; 0 means export all",
    )
    parser.add_argument(
        "--no-header",
        action="store_true",
        help="Do not write the CSV header row",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    bag_path = os.path.abspath(args.bag_path)
    if not os.path.exists(bag_path):
        raise FileNotFoundError(f"Bag file does not exist: {bag_path}")

    output_csv = os.path.abspath(args.output_csv) if args.output_csv else default_output_path(bag_path)
    output_dir = os.path.dirname(output_csv)
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)

    print(f"Reading scans from: {bag_path}")
    print(f"Using topic: {args.scan_topic}")
    print(f"Writing CSV to: {output_csv}")

    scans, max_range_count, max_intensity_count = collect_scans(bag_path, args.scan_topic, args.max_scans)
    if not scans:
        raise RuntimeError(f"No LaserScan messages found on topic {args.scan_topic} in {bag_path}")

    with open(output_csv, "w", newline="") as file_handle:
        writer = csv.writer(file_handle)

        if not args.no_header:
            writer.writerow(build_header(max_range_count, max_intensity_count))

        for index, scan in enumerate(scans, start=1):
            writer.writerow(build_row(scan, max_range_count, max_intensity_count))

            if index % 20 == 0:
                file_handle.flush()
                print(f"Exported {index} scans")

    print(
        f"Saved {len(scans)} scans to {output_csv} "
        f"(max ranges={max_range_count}, max intensities={max_intensity_count})"
    )


if __name__ == "__main__":
    main()