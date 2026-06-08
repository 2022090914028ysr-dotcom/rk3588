#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import threading
from typing import List, Tuple

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


def angle_between(v1: Tuple[float, float], v2: Tuple[float, float]) -> float:
    """Return the angle between two 2D vectors."""
    x1, y1 = v1
    x2, y2 = v2
    n1 = math.hypot(x1, y1)
    n2 = math.hypot(x2, y2)
    if n1 == 0.0 or n2 == 0.0:
        return 0.0
    dot = x1 * x2 + y1 * y2
    cosang = max(-1.0, min(1.0, dot / (n1 * n2)))
    return math.acos(cosang)


def distance(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


class WaypointManager:
    def __init__(self):
        self.path_topic = rospy.get_param('~path_topic', '/move_base/GlobalPlanner/plan')
        self.goal_pub_topic = rospy.get_param('~goal_pub_topic', '/NavOA_goal')
        self.base_frame = rospy.get_param('~base_frame', 'base_footprint')
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.5)
        self.min_spacing = rospy.get_param('~min_spacing', 1.0)
        self.curve_angle_deg = rospy.get_param('~curve_angle_deg', 35.0)
        self.sample_every_n = rospy.get_param('~sample_every_n', 0)  # 0 means no downsampling

        self._tf_listener = tf.TransformListener()
        self._goal_pub = rospy.Publisher(self.goal_pub_topic, PoseStamped, queue_size=1)
        self._path_sub = rospy.Subscriber(self.path_topic, Path, self._on_path, queue_size=1)

        self._lock = threading.RLock()
        self._path_frame = None
        self._waypoints = []  # List[PoseStamped]
        self._current_idx = -1

        rospy.loginfo('[waypoint_manager] initialized, listening path: %s, publishing goals: %s',
                      self.path_topic, self.goal_pub_topic)

    def _on_path(self, msg: Path):
        with self._lock:
            self._path_frame = msg.header.frame_id or 'map'
            poses = msg.poses
            if len(poses) < 2:
                rospy.logwarn('[waypoint_manager] received path with <2 poses, ignoring')
                return

            # extract keypoints
            keypoints = self._extract_keypoints(poses)
            if len(keypoints) == 0:
                rospy.logwarn('[waypoint_manager] no keypoints extracted, using endpoints')
                keypoints = [poses[0], poses[-1]]

            # update waypoint list and publish first goal
            self._waypoints = keypoints
            self._current_idx = 0
            rospy.loginfo('[waypoint_manager] extracted %d waypoints, publishing first', len(keypoints))
            self._publish_current_goal()

    def _extract_keypoints(self, poses: List[PoseStamped]) -> List[PoseStamped]:
        curve_th = math.radians(self.curve_angle_deg)
        selected: List[PoseStamped] = []

        # always include first pose
        selected.append(poses[0])
        last_added_xy = (poses[0].pose.position.x, poses[0].pose.position.y)

        # optionally sample every N poses
        if self.sample_every_n and self.sample_every_n > 0:
            for i in range(self.sample_every_n, len(poses) - 1, self.sample_every_n):
                p = poses[i].pose.position
                xy = (p.x, p.y)
                if distance(last_added_xy, xy) >= self.min_spacing:
                    selected.append(poses[i])
                    last_added_xy = xy

        # detect curvature / turn points
        for i in range(1, len(poses) - 1):
            p_prev = poses[i - 1].pose.position
            p_curr = poses[i].pose.position
            p_next = poses[i + 1].pose.position

            v1 = (p_curr.x - p_prev.x, p_curr.y - p_prev.y)
            v2 = (p_next.x - p_curr.x, p_next.y - p_curr.y)
            ang = angle_between(v1, v2)

            xy_curr = (p_curr.x, p_curr.y)
            if ang >= curve_th or distance(last_added_xy, xy_curr) >= self.min_spacing:
                selected.append(poses[i])
                last_added_xy = xy_curr

        # always include last pose
        selected.append(poses[-1])
        return selected

    def _publish_current_goal(self):
        if 0 <= self._current_idx < len(self._waypoints):
            goal = self._waypoints[self._current_idx]
            # preserve original path timestamps/frame
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = self._path_frame or goal.header.frame_id or 'map'
            self._goal_pub.publish(goal)
            rospy.loginfo('[waypoint_manager] publish goal %d/%d in frame %s',
                          self._current_idx + 1, len(self._waypoints), goal.header.frame_id)

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                with self._lock:
                    if 0 <= self._current_idx < len(self._waypoints):
                        # get current robot pose in path frame
                        base_frame = self.base_frame
                        path_frame = self._path_frame or 'map'
                        self._tf_listener.waitForTransform(path_frame, base_frame, rospy.Time(0), rospy.Duration(0.2))
                        (trans, rot) = self._tf_listener.lookupTransform(path_frame, base_frame, rospy.Time(0))
                        robot_xy = (trans[0], trans[1])

                        goal = self._waypoints[self._current_idx]
                        goal_xy = (goal.pose.position.x, goal.pose.position.y)
                        d = distance(robot_xy, goal_xy)

                        if d <= self.goal_tolerance:
                            # advance to next goal
                            self._current_idx += 1
                            if self._current_idx < len(self._waypoints):
                                self._publish_current_goal()
                            else:
                                rospy.loginfo('[waypoint_manager] all waypoints reached')
                    # else: no active waypoints - waiting for path
            except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass
            rate.sleep()


def main():
    rospy.initNode('waypoint_manager')
    mgr = WaypointManager()
    mgr.spin()


if __name__ == '__main__':
    main()