#!/usr/bin/env python

import math
import threading
from typing import List, Tuple

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion
import tf2_ros
import tf


def _dist(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


def _yaw_to_quat(yaw: float) -> Quaternion:
    q = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


def _turn_angle(p_prev: Tuple[float, float], p_curr: Tuple[float, float], p_next: Tuple[float, float]) -> float:
    v1x = p_curr[0] - p_prev[0]
    v1y = p_curr[1] - p_prev[1]
    v2x = p_next[0] - p_curr[0]
    v2y = p_next[1] - p_curr[1]
    dot = v1x * v2x + v1y * v2y
    det = v1x * v2y - v1y * v2x
    return math.atan2(det, dot)  # signed angle in radians


class WaypointSequencer:
    def __init__(self):
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.path_topic = rospy.get_param('~path_topic', '/move_base/GlobalPlanner/plan')
        self.goal_topic = rospy.get_param('~goal_topic', '/move_base_simple/goal')
        self.angle_threshold_deg = float(rospy.get_param('~angle_threshold_deg', 25.0))
        self.min_spacing = float(rospy.get_param('~min_spacing', 0.6))
        self.waypoint_radius = float(rospy.get_param('~waypoint_radius', 0.35))

        self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=1)
        self._path_sub = rospy.Subscriber(self.path_topic, Path, self._on_path, queue_size=1)

        self._waypoints: List[Tuple[float, float, float]] = []  # (x, y, yaw)
        self._current_idx: int = -1
        self._final_target: Tuple[float, float] = None
        self._lock = threading.Lock()

        rospy.Timer(rospy.Duration(0.1), self._on_timer)
        rospy.loginfo('[sequencer] Initialized. path=%s goal=%s', self.path_topic, self.goal_topic)

    def _on_path(self, path_msg: Path):
        poses = path_msg.poses
        if not poses:
            return

        # Extract 2D points from Path
        points = [(ps.pose.position.x, ps.pose.position.y) for ps in poses]
        final = points[-1]

        with self._lock:
            # If this is a fresh plan to a new final goal, build waypoints
            if self._final_target is None or _dist(self._final_target, final) > 0.5:
                self._final_target = final
                self._waypoints = self._extract_keypoints(points)
                self._current_idx = 0 if self._waypoints else -1
                rospy.loginfo('[sequencer] Captured plan with %d points, selected %d waypoints.',
                              len(points), len(self._waypoints))
                if self._current_idx >= 0:
                    self._publish_current_goal()
            else:
                # Ignore replans to interim waypoints
                pass

    def _extract_keypoints(self, points: List[Tuple[float, float]]) -> List[Tuple[float, float, float]]:
        n = len(points)
        if n == 0:
            return []
        if n == 1:
            return [(points[0][0], points[0][1], 0.0)]

        selected: List[int] = [0]
        last_sel = points[0]
        angle_thresh = math.radians(self.angle_threshold_deg)

        for i in range(1, n - 1):
            ang = abs(_turn_angle(points[i - 1], points[i], points[i + 1]))
            far_enough = _dist(points[i], last_sel) >= self.min_spacing
            if ang >= angle_thresh and far_enough:
                selected.append(i)
                last_sel = points[i]

        # Ensure final point is included
        if selected[-1] != n - 1:
            selected.append(n - 1)

        # Build (x, y, yaw) tuples; yaw aligned to next segment direction
        waypoints: List[Tuple[float, float, float]] = []
        for idx, si in enumerate(selected):
            x, y = points[si]
            if idx < len(selected) - 1:
                nx, ny = points[selected[idx + 1]]
                yaw = math.atan2(ny - y, nx - x)
            else:
                # Last waypoint: face along last segment if possible
                if len(selected) >= 2:
                    px, py = points[selected[-2]]
                    yaw = math.atan2(y - py, x - px)
                else:
                    yaw = 0.0
            waypoints.append((x, y, yaw))

        return waypoints

    def _lookup_robot_xy(self) -> Tuple[float, float]:
        try:
            t = self._tf_buffer.lookup_transform(self.map_frame, self.base_frame, rospy.Time(0), rospy.Duration(0.2))
            tx = t.transform.translation.x
            ty = t.transform.translation.y
            return (tx, ty)
        except Exception as e:
            rospy.logdebug('[sequencer] tf lookup failed: %s', str(e))
            return None

    def _on_timer(self, _evt):
        with self._lock:
            if self._current_idx < 0 or self._current_idx >= len(self._waypoints):
                return
            robot_xy = self._lookup_robot_xy()
            if robot_xy is None:
                return
            wx, wy, _ = self._waypoints[self._current_idx]
            if _dist(robot_xy, (wx, wy)) <= self.waypoint_radius:
                # Advance to next waypoint
                self._current_idx += 1
                if self._current_idx < len(self._waypoints):
                    rospy.loginfo('[sequencer] Reached waypoint %d, publishing %d/%d',
                                  self._current_idx - 1, self._current_idx + 1, len(self._waypoints))
                    self._publish_current_goal()
                else:
                    rospy.loginfo('[sequencer] All waypoints completed.')

    def _publish_current_goal(self):
        x, y, yaw = self._waypoints[self._current_idx]
        ps = PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = self.map_frame
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = 0.0
        ps.pose.orientation = _yaw_to_quat(yaw)
        self._goal_pub.publish(ps)


def main():
    rospy.init_node('navoa_waypoint_sequencer', anonymous=False)
    WaypointSequencer()
    rospy.spin()


if __name__ == '__main__':
    main()
