#!/usr/bin/env python
"""Merge two LaserScan topics into /mergescan.
Resample both to common angle grid, then per-angle min (closest obstacle wins).
"""

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np


def scan_to_angles(scan):
    """Return array of angles (rad) for each range index."""
    n = len(scan.ranges)
    return np.linspace(scan.angle_min, scan.angle_max, n)


def get_range_at_angle(scan, angles_arr, target_angle):
    """Get interpolated range at target_angle from scan (0 if out of FOV or invalid)."""
    if target_angle < scan.angle_min or target_angle > scan.angle_max:
        return 0.0
    idx = (target_angle - scan.angle_min) / scan.angle_increment
    i0 = int(np.floor(idx))
    i1 = i0 + 1
    if i0 < 0 or i1 >= len(scan.ranges):
        return 0.0
    r0 = scan.ranges[i0] if scan.ranges[i0] >= scan.range_min else 0.0
    r1 = scan.ranges[i1] if scan.ranges[i1] >= scan.range_min else 0.0
    frac = idx - i0
    r = r0 + (r1 - r0) * frac
    return max(r, 0.0)


def rotate_scan(scan, offset):
    """Return a new LaserScan with ranges rotated by offset radians."""
    if offset == 0:
        return scan
    out = LaserScan()
    out.header = scan.header
    out.angle_min = scan.angle_min + offset
    out.angle_max = scan.angle_max + offset
    out.angle_increment = scan.angle_increment
    out.time_increment = scan.time_increment
    out.scan_time = scan.scan_time
    out.range_min = scan.range_min
    out.range_max = scan.range_max
    shift = int(round(offset / scan.angle_increment))
    out.ranges = list(np.roll(np.array(scan.ranges, dtype=np.float32), shift))
    out.intensities = list(scan.intensities)
    return out


class ScanMerger:
    def __init__(self):
        rospy.init_node('scan_merger', anonymous=True)

        self.topic_a = rospy.get_param('~topic_a', '/scan')
        self.topic_b = rospy.get_param('~topic_b', '/camscan')
        self.output_frame = rospy.get_param('~output_frame', 'base_footprint')

        self.scan_a = None
        self.scan_b = None
        self.last_a = rospy.Time(0)
        self.last_b = rospy.Time(0)
        self.timeout = rospy.Duration(rospy.get_param('~timeout', 1.0))
        self.resolution = rospy.get_param('~merge_resolution', 0.005)  # rad
        self.offset_a = rospy.get_param('~angle_offset_a', 0.0)
        self.offset_b = rospy.get_param('~angle_offset_b', -1.5708)  # camscan -90 deg
        self.pub_rate = rospy.get_param('~pub_rate', 50.0)

        self.sub_a = rospy.Subscriber(self.topic_a, LaserScan, self.cb_a)
        self.sub_b = rospy.Subscriber(self.topic_b, LaserScan, self.cb_b)
        self.pub = rospy.Publisher('/mergescan', LaserScan, queue_size=10)

        rospy.loginfo("ScanMerger: %s + %s -> /mergescan (angle-aligned)", self.topic_a, self.topic_b)

    def cb_a(self, msg):
        self.scan_a = msg
        self.last_a = msg.header.stamp

    def cb_b(self, msg):
        self.scan_b = msg
        self.last_b = msg.header.stamp

    def merge_and_publish(self):
        now = rospy.Time.now()
        a_ok = (now - self.last_a) < self.timeout and self.scan_a is not None
        b_ok = (now - self.last_b) < self.timeout and self.scan_b is not None

        if not a_ok and not b_ok:
            return

        if a_ok and b_ok:
            # apply per-topic rotation
            sa = rotate_scan(self.scan_a, self.offset_a)
            sb = rotate_scan(self.scan_b, self.offset_b)

            # common angle range: union of both FOVs
            ang_min = min(sa.angle_min, sb.angle_min)
            ang_max = max(sa.angle_max, sb.angle_max)
            n = int((ang_max - ang_min) / self.resolution) + 1
            out_angles = np.linspace(ang_min, ang_max, n)

            ranges_a = np.array(sa.ranges, dtype=np.float32)
            ranges_b = np.array(sb.ranges, dtype=np.float32)
            ang_a = scan_to_angles(self.scan_a)
            ang_b = scan_to_angles(self.scan_b)

            merged = np.zeros(n, dtype=np.float32)
            for i, ang in enumerate(out_angles):
                # get range from scan A
                if ang >= sa.angle_min and ang <= sa.angle_max:
                    idx = (ang - sa.angle_min) / sa.angle_increment
                    i0 = int(np.clip(idx, 0, len(ranges_a) - 1))
                    i1 = min(i0 + 1, len(ranges_a) - 1)
                    ra = float(ranges_a[i0]) if ranges_a[i0] > 0 else 0.0
                    if i1 != i0:
                        rb_val = float(ranges_a[i1]) if ranges_a[i1] > 0 else 0.0
                        ra = ra + (rb_val - ra) * (idx - i0) if ra and rb_val else max(ra, rb_val)
                else:
                    ra = 0.0

                # get range from scan B
                if ang >= sb.angle_min and ang <= sb.angle_max:
                    idx = (ang - sb.angle_min) / sb.angle_increment
                    i0 = int(np.clip(idx, 0, len(ranges_b) - 1))
                    i1 = min(i0 + 1, len(ranges_b) - 1)
                    rb = float(ranges_b[i0]) if ranges_b[i0] > 0 else 0.0
                    if i1 != i0:
                        rb2 = float(ranges_b[i1]) if ranges_b[i1] > 0 else 0.0
                        rb = rb + (rb2 - rb) * (idx - i0) if rb and rb2 else max(rb, rb2)
                else:
                    rb = 0.0

                # merge: min of valid, prefer non-zero
                if ra > 0 and rb > 0:
                    merged[i] = min(ra, rb)
                else:
                    merged[i] = max(ra, rb)

            out = LaserScan()
            out.header.stamp = now
            out.header.frame_id = self.output_frame
            out.angle_min = ang_min
            out.angle_max = ang_max
            out.angle_increment = (ang_max - ang_min) / float(n - 1) if n > 1 else 0
            out.range_min = min(sa.range_min, sb.range_min)
            out.range_max = max(sa.range_max, sb.range_max)
            out.scan_time = max(sa.scan_time, sb.scan_time)
            out.ranges = merged.tolist()

        elif a_ok:
            out = rotate_scan(self.scan_a, self.offset_a)
        else:
            out = rotate_scan(self.scan_b, self.offset_b)

        out.header.stamp = now
        self.pub.publish(out)

    def run(self):
        r = rospy.Rate(self.pub_rate)
        while not rospy.is_shutdown():
            self.merge_and_publish()
            r.sleep()


if __name__ == '__main__':
    try:
        ScanMerger().run()
    except rospy.ROSInterruptException:
        pass
