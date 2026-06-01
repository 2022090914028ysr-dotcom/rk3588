#!/usr/bin/python3
"""
Autonomous Exploration Node - Single Robot Real-World Mapping
Publishes exploration goals to move_base / NavOA navigation stack.
Does NOT publish /cmd_vel - delegates all motion to the nav stack.
Topics: /map (in), /odom_rf2o (in), /move_base_simple/goal (out)
"""

import os
import time
import math
import numpy as np
import cv2
import rospy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray

from marl_mapping.utils.map_builder import MapBuilder
from marl_mapping.utils.frontier import detect_frontiers, select_best_frontier


class ExplorationNode:
    """Autonomous exploration: detects frontiers, sends goals to move_base."""

    def __init__(self):
        rospy.init_node('autonomous_exploration', anonymous=True)

        # ---- Parameters ----
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.map_topic = rospy.get_param('~map_topic', '/map')
        self.odom_topic = rospy.get_param('~odom_topic', '/odom_rf2o')
        self.goal_topic = rospy.get_param('~goal_topic', '/move_base_simple/goal')
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.5)    # meters
        self.goal_timeout = rospy.get_param('~goal_timeout', 60.0)       # seconds
        self.idle_wait = rospy.get_param('~idle_wait', 1.0)

        self.map_size_cm = rospy.get_param('~map_size_cm', 2500)
        self.map_resolution = rospy.get_param('~map_resolution', 5)
        self.unit_size_cm = rospy.get_param('~unit_size_cm', 5)
        self.max_steps = rospy.get_param('~max_steps', 500)
        self.min_frontier_size = rospy.get_param('~min_frontier_size', 5)
        self.exploration_method = rospy.get_param('~exploration_method', 'nearest')

        self.vis_enabled = rospy.get_param('~visualization', True)
        self.save_data = rospy.get_param('~save_data', True)
        self.data_dir = rospy.get_param('~data_dir',
                                         os.path.expanduser('~/exploration_data'))

        # ---- State ----
        self.map_size = self.map_size_cm // self.map_resolution
        self.obstacle_map = np.zeros((self.map_size, self.map_size), dtype=np.uint8)
        self.explored_map = np.zeros((self.map_size, self.map_size), dtype=np.uint8)
        self.frontier_map = np.zeros((self.map_size, self.map_size), dtype=np.uint8)
        self.robot_pos = [0.0, 0.0, 0.0]       # world (x, y, z)
        self.robot_quat = [0.0, 0.0, 0.0, 1.0]
        self.current_goal_world = None          # world (wx, wy)
        self.goal_sent_time = None
        self.step_count = 0
        self.explored_ratio = 0.0
        self.is_exploring = False

        # Map builder
        self.map_builder = MapBuilder(
            map_size_cm=self.map_size_cm,
            map_resolution=self.map_resolution,
            unit_size_cm=self.unit_size_cm)

        # ---- Publishers ----
        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=1)
        self.frontier_marker_pub = rospy.Publisher(
            '/exploration/frontiers', MarkerArray, queue_size=1)
        self.goal_marker_pub = rospy.Publisher(
            '/exploration/goal_marker', Marker, queue_size=1)

        # ---- Subscribers ----
        rospy.Subscriber(self.map_topic, OccupancyGrid,
                         self.map_callback, queue_size=1)
        rospy.Subscriber(self.odom_topic, rospy.AnyMsg,
                         self.odom_callback, queue_size=1)

        # ---- Timer ----
        rospy.Timer(rospy.Duration(self.idle_wait), self.exploration_loop)

        # ---- Data dir ----
        if self.save_data:
            ts = time.strftime('%Y%m%d_%H%M%S')
            self.data_dir = os.path.join(self.data_dir, ts)
            os.makedirs(self.data_dir, exist_ok=True)

        rospy.loginfo("[Explore] Ready. goal¡ú%s, method=%s",
                      self.goal_topic, self.exploration_method)

    # ================================================================
    #  Callbacks
    # ================================================================
    def map_callback(self, msg):
        data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        p = self.map_builder.process_map(data, msg.info)
        self.obstacle_map = p['obstacle']
        self.explored_map = p['explored']
        self.frontier_map = self.map_builder.compute_frontier(
            self.obstacle_map, self.explored_map)
        obs = np.sum(self.obstacle_map > 0)
        total = self.obstacle_map.size
        exp = np.sum(self.explored_map > 0) - obs
        self.explored_ratio = exp / max(total - obs, 1)

    def odom_callback(self, msg):
        try:
            self.robot_pos = [msg.pose.pose.position.x,
                              msg.pose.pose.position.y,
                              msg.pose.pose.position.z]
            q = msg.pose.pose.orientation
            self.robot_quat = [q.x, q.y, q.z, q.w]
        except Exception:
            pass

    # ================================================================
    #  Main loop
    # ================================================================
    def exploration_loop(self, event):
        if not self.is_exploring:
            return

        # --- termination ---
        if self.step_count >= self.max_steps:
            rospy.loginfo("[Explore] Max steps %d", self.max_steps)
            self.stop_exploration(); return
        if self.explored_ratio > 0.99:
            rospy.loginfo("[Explore] %.1f%% explored", self.explored_ratio * 100)
            self.stop_exploration(); return

        # --- check current goal ---
        if self.current_goal_world is not None:
            d = math.hypot(self.robot_pos[0] - self.current_goal_world[0],
                           self.robot_pos[1] - self.current_goal_world[1])
            elapsed = (rospy.Time.now() - self.goal_sent_time).to_sec()
            if d < self.goal_tolerance:
                rospy.loginfo("[Explore] Goal reached (%.2fm)", d)
                self.current_goal_world = None
                self.goal_sent_time = None
                return
            if elapsed > self.goal_timeout:
                rospy.logwarn("[Explore] Goal timeout %.0fs, skip", elapsed)
                self.current_goal_world = None
                self.goal_sent_time = None
                return
            return  # still moving

        # --- pick new goal ---
        goal_xy = self._select_goal()
        if goal_xy is None:
            rospy.loginfo("[Explore] No frontiers ¡ª done.")
            self.stop_exploration(); return

        self._send_goal(goal_xy)
        self.step_count += 1

    # ================================================================
    #  Goal selection & sending
    # ================================================================
    def _select_goal(self):
        """Return (gx, gy) in grid coords, or None."""
        rx = int(self.robot_pos[0] * 100.0 / self.unit_size_cm + self.map_size // 2)
        ry = int(self.robot_pos[1] * 100.0 / self.unit_size_cm + self.map_size // 2)
        rx = max(0, min(self.map_size - 1, rx))
        ry = max(0, min(self.map_size - 1, ry))

        f = detect_frontiers(self.obstacle_map, self.explored_map,
                             min_frontier_size=self.min_frontier_size)
        if self.vis_enabled:
            self._pub_frontier_markers(f)
        if not f:
            return None
        return select_best_frontier((rx, ry), f, self.obstacle_map,
                                     method=self.exploration_method)

    def _send_goal(self, goal_xy):
        """Publish PoseStamped to navigation stack."""
        wx = (goal_xy[0] - self.map_size // 2) * self.unit_size_cm / 100.0
        wy = (goal_xy[1] - self.map_size // 2) * self.unit_size_cm / 100.0

        ps = PoseStamped()
        ps.header.frame_id = self.map_frame
        ps.header.stamp = rospy.Time.now()
        ps.pose.position.x = wx
        ps.pose.position.y = wy
        ps.pose.orientation.w = 1.0

        self.current_goal_world = (wx, wy)
        self.goal_sent_time = rospy.Time.now()
        self.goal_pub.publish(ps)

        rospy.loginfo("[Explore] step=%d ¡ú (%.2f, %.2f)",
                      self.step_count, wx, wy)
        if self.vis_enabled:
            self._pub_goal_marker(goal_xy)

    # ================================================================
    #  Visualization
    # ================================================================
    def _pub_frontier_markers(self, frontiers):
        ma = MarkerArray()
        for i, (cx, cy, sz) in enumerate(frontiers):
            m = Marker()
            m.header.frame_id = self.map_frame; m.header.stamp = rospy.Time.now()
            m.ns = "frontiers"; m.id = i; m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = (cx - self.map_size // 2) * self.unit_size_cm / 100.0
            m.pose.position.y = (cy - self.map_size // 2) * self.unit_size_cm / 100.0
            s = 0.08 + sz * 0.008
            m.scale.x = s; m.scale.y = s; m.scale.z = s
            m.color.a = 0.7; m.color.g = 1.0
            ma.markers.append(m)
        for i in range(len(frontiers), len(frontiers) + 50):
            m = Marker(); m.header.frame_id = self.map_frame
            m.header.stamp = rospy.Time.now()
            m.ns = "frontiers"; m.id = i; m.action = Marker.DELETE
            ma.markers.append(m)
        self.frontier_marker_pub.publish(ma)

    def _pub_goal_marker(self, goal_xy):
        m = Marker()
        m.header.frame_id = self.map_frame; m.header.stamp = rospy.Time.now()
        m.ns = "explore_goal"; m.id = 0; m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = (goal_xy[0] - self.map_size // 2) * self.unit_size_cm / 100.0
        m.pose.position.y = (goal_xy[1] - self.map_size // 2) * self.unit_size_cm / 100.0
        m.pose.position.z = 0.35
        m.scale.x = 0.25; m.scale.y = 0.25; m.scale.z = 0.25
        m.color.a = 1.0; m.color.r = 1.0
        self.goal_marker_pub.publish(m)

    # ================================================================
    #  Lifecycle
    # ================================================================
    def start_exploration(self):
        rospy.loginfo("[Explore] START ¡ú %s", self.goal_topic)
        self.is_exploring = True
        self.step_count = 0

    def stop_exploration(self):
        self.is_exploring = False
        self.current_goal_world = None
        rospy.loginfo("[Explore] STOP %.1f%%", self.explored_ratio * 100)
        self._save_map()

    def _save_map(self):
        if not self.save_data:
            return
        img = np.zeros((self.map_size, self.map_size, 3), dtype=np.uint8)
        img[:] = [128, 128, 128]
        img[(self.explored_map > 0) & (self.obstacle_map == 0)] = [255, 255, 255]
        img[self.obstacle_map > 0] = [0, 0, 0]
        img[self.frontier_map > 0] = [0, 255, 0]
        cv2.imwrite(os.path.join(self.data_dir, 'final_map.png'),
                    cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        rospy.loginfo("[Explore] Map saved ¡ú %s", self.data_dir)

    def run(self):
        rospy.loginfo("[Explore] Waiting for map...")
        t0 = rospy.Time.now() + rospy.Duration(60.0)
        while not rospy.is_shutdown():
            if self.obstacle_map is not None and np.any(self.explored_map > 0):
                break
            if rospy.Time.now() > t0:
                rospy.logwarn("[Explore] Map timeout, start anyway")
                break
            rospy.sleep(0.5)
        self.start_exploration()
        rospy.spin()
        self._save_map()


def main():
    node = ExplorationNode()
    try:
        node.run()
    except rospy.ROSInterruptException:
        node._save_map()


if __name__ == '__main__':
    main()
