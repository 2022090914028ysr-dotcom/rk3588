#!/usr/bin/env python3

import csv
import math
import os
from datetime import datetime

import rospy
from sensor_msgs.msg import LaserScan


def default_output_path():
    package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.join(package_dir, f"scan_data_{timestamp}.csv")


class ScanToCsvRecorder:
    def __init__(self):
        rospy.init_node("scan_to_csv", anonymous=False)

        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.output_csv = rospy.get_param("~output_csv", default_output_path())
        self.max_scans = int(rospy.get_param("~max_scans", 0))
        self.write_header = bool(rospy.get_param("~write_header", True))

        output_dir = os.path.dirname(self.output_csv)
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)

        self.scan_count = 0
        self.rows = []
        self.max_range_count = 0
        self.max_intensity_count = 0

        rospy.on_shutdown(self.shutdown)
        self.subscriber = rospy.Subscriber(self.scan_topic, LaserScan, self.callback, queue_size=20)

        rospy.loginfo("Recording LaserScan from %s to %s", self.scan_topic, self.output_csv)
        if self.max_scans > 0:
            rospy.loginfo("Will stop after %d scans", self.max_scans)

    def write_csv_header(self, writer):
        range_columns = [f"range_{index}" for index in range(self.max_range_count)]
        intensity_columns = [f"intensity_{index}" for index in range(self.max_intensity_count)]
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
        writer.writerow(header)

    def callback(self, scan):
        ranges = self._sanitize_values(scan.ranges)
        intensities = self._sanitize_values(scan.intensities)

        self.max_range_count = max(self.max_range_count, len(ranges))
        self.max_intensity_count = max(self.max_intensity_count, len(intensities))

        metadata = [
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
        self.rows.append((metadata, ranges, intensities))
        self.scan_count += 1

        if self.scan_count % 20 == 0:
            rospy.loginfo("Recorded %d scans", self.scan_count)

        if self.max_scans > 0 and self.scan_count >= self.max_scans:
            rospy.loginfo("Reached max_scans=%d, stopping recorder", self.max_scans)
            rospy.signal_shutdown("Completed requested scan capture")

    @staticmethod
    def _sanitize_values(values):
        sanitized = []
        for value in values:
            if math.isinf(value) or math.isnan(value):
                sanitized.append("")
            else:
                sanitized.append(value)
        return sanitized

    @staticmethod
    def _pad_values(values, target_length):
        padded = list(values)
        if len(padded) < target_length:
            padded.extend([""] * (target_length - len(padded)))
        return padded[:target_length]

    def shutdown(self):
        if not self.rows:
            return

        with open(self.output_csv, "w", newline="") as file_handle:
            writer = csv.writer(file_handle)
            if self.write_header:
                self.write_csv_header(writer)

            for metadata, ranges, intensities in self.rows:
                row = list(metadata)
                row.extend(self._pad_values(ranges, self.max_range_count))
                row.extend(self._pad_values(intensities, self.max_intensity_count))
                writer.writerow(row)

        rospy.loginfo(
            "Saved %d scans to %s (max ranges=%d, max intensities=%d)",
            self.scan_count,
            self.output_csv,
            self.max_range_count,
            self.max_intensity_count,
        )


if __name__ == "__main__":
    recorder = ScanToCsvRecorder()
    rospy.spin()