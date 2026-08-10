#!/usr/bin/env python
"""Merge multiple LaserScan topics into /mergescan.
All inputs resampled to common angle grid, per-angle min (closest obstacle wins).

Supports 2-way (legacy) or N-way merge via ~topics / ~angle_offsets params.

Legacy (2-way):
    ~topic_a, ~topic_b, ~angle_offset_a, ~angle_offset_b

New (N-way):
    ~topics:         ["/voscan", "/camscan", "/scan"]
    ~angle_offsets:  [-0.7854, 0.0, 0.0]
"""

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np


def rotate_scan(scan, offset):
    """Return a new LaserScan with ranges rotated by offset radians.

    For full 360 deg scans, np.roll is used (circular shift).
    For partial-FOV scans, np.roll would wrap data incorrectly, so we pad
    the ranges with inf on both sides to create a 360 deg virtual array before
    rolling, preserving the original FOV boundary.
    """
    if offset == 0.0:
        return scan

    fov = scan.angle_max - scan.angle_increment - scan.angle_min  # actual data span
    is_full = (2.0 * np.pi - fov) < scan.angle_increment * 2  # ~360 deg?

    out = LaserScan()
    out.header = scan.header
    out.angle_min = scan.angle_min + offset
    out.angle_max = scan.angle_max + offset
    out.angle_increment = scan.angle_increment
    out.time_increment = scan.time_increment
    out.scan_time = scan.scan_time
    out.range_min = scan.range_min
    out.range_max = scan.range_max

    ranges_arr = np.array(scan.ranges, dtype=np.float32)

    if is_full:
        shift = int(round(offset / scan.angle_increment))
        out.ranges = list(np.roll(ranges_arr, shift))
    else:
        # Pad to 360 deg with inf, roll, then keep only the moved FOV
        total_beams = int(round(2.0 * np.pi / scan.angle_increment))
        padded = np.full(total_beams, float('inf'), dtype=np.float32)
        start_idx = int(round((scan.angle_min + np.pi) / scan.angle_increment))
        padded[start_idx:start_idx + len(ranges_arr)] = ranges_arr
        shift = int(round(offset / scan.angle_increment))
        padded = np.roll(padded, shift)
        out.ranges = list(padded[start_idx:start_idx + len(ranges_arr)])

    out.intensities = list(scan.intensities)
    return out


def sample_scan(scan, angle):
    """Sample a rotated scan at a given angle, return float (m) or inf (no data)."""
    if angle < scan.angle_min or angle > scan.angle_max:
        return float('inf')
    idx = (angle - scan.angle_min) / scan.angle_increment
    i0 = int(np.clip(idx, 0, len(scan.ranges) - 1))
    i1 = min(i0 + 1, len(scan.ranges) - 1)
    r0 = scan.ranges[i0] if scan.range_min < scan.ranges[i0] < float('inf') else float('inf')
    r1 = scan.ranges[i1] if scan.range_min < scan.ranges[i1] < float('inf') else float('inf')
    if r0 == float('inf') and r1 == float('inf'):
        return float('inf')
    if r0 == float('inf'):
        return r1
    if r1 == float('inf'):
        return r0
    return r0 + (r1 - r0) * (idx - i0)


# ---------------------------------------------------------------------------
class ScanMerger:
    def __init__(self):
        rospy.init_node('scan_merger', anonymous=True)

        # ---- resolve topics -------------------------------------------------
        topics_param = rospy.get_param('~topics', None)
        if topics_param is not None:
            # N-way mode
            self.topics = topics_param
            self.offsets = rospy.get_param('~angle_offsets',
                                           [0.0] * len(self.topics))
            if len(self.offsets) != len(self.topics):
                rospy.logerr("len(angle_offsets) != len(topics)")
                raise ValueError("angle_offsets must match topics")
        else:
            # legacy 2-way mode
            self.topics = [
                rospy.get_param('~topic_a', '/scan'),
                rospy.get_param('~topic_b', '/camscan'),
            ]
            self.offsets = [
                rospy.get_param('~angle_offset_a', 0.0),
                rospy.get_param('~angle_offset_b', -1.5708),
            ]

        self.output_frame = rospy.get_param('~output_frame', 'base_link')
        self.timeout = rospy.Duration(rospy.get_param('~timeout', 1.0))
        self.resolution = rospy.get_param('~merge_resolution', 0.005)  # rad
        self.pub_rate = rospy.get_param('~pub_rate', 50.0)

        # per-topic state
        self.N = len(self.topics)
        self.scans = [None] * self.N       # latest LaserScan
        self.last_stamps = [rospy.Time(0)] * self.N

        self.subs = []
        for i, t in enumerate(self.topics):
            sub = rospy.Subscriber(t, LaserScan,
                                   lambda msg, idx=i: self.cb(msg, idx))
            self.subs.append(sub)

        self.pub = rospy.Publisher('/mergescan', LaserScan, queue_size=10)

        rospy.loginfo("ScanMerger %d-way: %s -> /mergescan",
                      self.N, " + ".join(self.topics))
        for i in range(self.N):
            rospy.loginfo("  [%d] %s  offset=%.4f rad", i, self.topics[i], self.offsets[i])

    # -----------------------------------------------------------------------
    def cb(self, msg, idx):
        self.scans[idx] = msg
        self.last_stamps[idx] = msg.header.stamp

    # -----------------------------------------------------------------------
    def merge_and_publish(self):
        now = rospy.Time.now()

        # gather active (non-timed-out) scans, apply rotation
        active = []
        for i in range(self.N):
            if (now - self.last_stamps[i]) < self.timeout and self.scans[i] is not None:
                active.append(rotate_scan(self.scans[i], self.offsets[i]))

        if not active:
            return

        if len(active) == 1:
            out = active[0]
        else:
            # union FOV
            ang_min = min(s.angle_min for s in active)
            ang_max = max(s.angle_max for s in active)
            n = int((ang_max - ang_min) / self.resolution) + 1

            merged = np.full(n, float('inf'), dtype=np.float32)
            for i, ang in enumerate(np.linspace(ang_min, ang_max, n)):
                best = float('inf')
                for s in active:
                    r = sample_scan(s, ang)
                    if s.range_min < r < best:
                        best = r
                merged[i] = best  # inf if no sensor covers this angle

            out = LaserScan()
            out.header.frame_id = self.output_frame
            out.angle_min = ang_min
            out.angle_max = ang_max
            out.angle_increment = (ang_max - ang_min) / float(n - 1) if n > 1 else 0
            out.range_min = min(s.range_min for s in active)
            out.range_max = max(s.range_max for s in active)
            out.scan_time = max(s.scan_time for s in active if hasattr(s, 'scan_time'))
            out.ranges = merged.tolist()

        out.header.stamp = now
        self.pub.publish(out)

    # -----------------------------------------------------------------------
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

