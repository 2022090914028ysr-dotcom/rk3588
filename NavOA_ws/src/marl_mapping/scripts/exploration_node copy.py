#!/usr/bin/python3
"""
Autonomous Exploration Node - Single Robot Real-World Mapping
Publishes PoseStamped goals to move_base only.
move_base -> path_segment_publisher -> /split_goal -> NavOA -> cmd_vel
Topics: /map (in), /odom_rf2o (in), /move_base_simple/goal (out)
"""

import os
import time
import math
from collections import deque
import numpy as np
import cv2
import rospy

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path

from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros

from marl_mapping.utils.frontier import detect_frontiers


class ExplorationNode:
    """Autonomous exploration: detects frontiers, sends goals to move_base."""

    def __init__(self):
        rospy.init_node('autonomous_exploration', anonymous=True)

        # ---- Parameters ----
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.map_topic = rospy.get_param('~map_topic', '/map')
        self.odom_topic = rospy.get_param('~odom_topic', '/odom_rf2o')
        self.goal_topic = rospy.get_param('~goal_topic', '/move_base_simple/goal')
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.5)    # meters
        self.goal_timeout = rospy.get_param('~goal_timeout', 60.0)       # seconds
        self.idle_wait = rospy.get_param('~idle_wait', 1.0)

        # ---- Plan monitoring (watch global planner for empty plans) ----
        self.plan_watch_enabled = rospy.get_param('~plan_watch_enabled', True)
        self.plan_topic = rospy.get_param('~plan_topic',
                                           '/move_base/GlobalPlanner/plan')
        self.plan_empty_count = 0           # consecutive empty plans
        self.plan_empty_threshold = 3        # switch goal after N empty plans

        self.max_steps = rospy.get_param('~max_steps', 500)
        self.min_frontier_size = rospy.get_param('~min_frontier_size', 5)
        self.exploration_method = rospy.get_param('~exploration_method', 'nearest')
        # Costmap: subscribe to move_base's global costmap for accurate
        # reachability info (accounts for inflation, footprint, etc.)
        self.costmap_topic = rospy.get_param('~costmap_topic',
                                              '/move_base/global_costmap/costmap')
        # Fallback: manual inflation when costmap not available
        self.inflation_cells = rospy.get_param('~inflation_cells', 6)

        self.vis_enabled = rospy.get_param('~visualization', True)
        self.save_data = rospy.get_param('~save_data', True)
        self.data_dir = rospy.get_param('~data_dir',
                                         os.path.expanduser('~/exploration_data'))

        # ---- State ----
        self.obstacle_map = None
        self.explored_map = None
        self.frontier_map = None
        self.safe_free_map = None     # free cells outside inflation radius (fallback)
        self.costmap = None           # 2D array from move_base global costmap
        self.robot_pos = [0.0, 0.0, 0.0]       # world (x, y, z)
        self.robot_quat = [0.0, 0.0, 0.0, 1.0]
        self.current_goal_world = None          # world (wx, wy)
        self.goal_sent_time = None
        self.step_count = 0
        self.explored_ratio = 0.0
        self.is_exploring = False
        self.visited_goals = set()          # track visited grid cells to avoid repeats
        self._failed_frontiers = set()      # frontiers that failed path planning
        self.goal_stuck_count = 0           # consecutive timeouts on same area
        self._frontier_reset_count = 0      # times we cleared & re-detected frontiers
        self._max_frontier_resets = 3       # give up after this many full resets
        self._stuck_since = None            # timestamp when no reachable goal found
        self._stuck_retry_interval = 5.0    # retry frontier detection every N sec
        self._max_stuck_duration = 120.0    # give up completely after this long

        # ---- TF (for transforming robot pose to map frame) ----
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Actual map metadata (from OccupancyGrid msg)
        self.map_origin = [0.0, 0.0, 0.0]
        self.map_res = 0.05
        self.map_width = 0
        self.map_height = 0

        # ---- Publishers ----
        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=1)
        self.frontier_marker_pub = rospy.Publisher(
            '/exploration/frontiers', MarkerArray, queue_size=1)
        self.goal_marker_pub = rospy.Publisher(
            '/exploration/goal_marker', Marker, queue_size=1)

        # ---- Subscribers ----
        rospy.Subscriber(self.map_topic, OccupancyGrid,
                         self.map_callback, queue_size=1)
        rospy.Subscriber(self.odom_topic, Odometry,
                         self.odom_callback, queue_size=1)
        self.plan_sub = rospy.Subscriber(self.plan_topic, Path,
                                          self.plan_callback, queue_size=1)
        self.costmap_sub = rospy.Subscriber(self.costmap_topic, OccupancyGrid,
                                             self.costmap_callback, queue_size=1)

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
        # Work directly with raw cartographer map -- NO resizing
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_res = msg.info.resolution
        self.map_origin = [msg.info.origin.position.x,
                           msg.info.origin.position.y,
                           msg.info.origin.position.z]

        raw = np.array(msg.data).reshape((self.map_height, self.map_width))

        # obstacle: value > 50 (occupied), explored: value >= 0 (known)
        self.obstacle_map = (raw > 50).astype(np.uint8)
        self.explored_map = (raw >= 0).astype(np.uint8)

        # frontier: known-free cells adjacent to unknown
        kernel = np.ones((3, 3), dtype=np.uint8)
        safe_obs = cv2.dilate(self.obstacle_map, kernel, iterations=2)
        unknown = (self.explored_map == 0).astype(np.uint8)
        dil_unk = cv2.dilate(unknown, kernel, iterations=1)
        self.frontier_map = ((self.explored_map > 0) & (safe_obs == 0) &
                             (dil_unk > 0)).astype(np.uint8)

        # Safe free map: cells that are reachable in the inflated costmap.
        # Must be explored, not obstacle-dilated, not frontier.
        ksize = 2 * self.inflation_cells + 1
        inf_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                                (ksize, ksize))
        obs_inflated = cv2.dilate(self.obstacle_map, inf_kernel, iterations=1)
        self.safe_free_map = ((self.explored_map > 0) &
                              (obs_inflated == 0) &
                              (self.frontier_map == 0))

        obs = np.sum(self.obstacle_map)
        total = self.obstacle_map.size
        exp = np.sum(self.explored_map) - obs
        self.explored_ratio = exp / max(total - obs, 1)

        rospy.loginfo_throttle(30, "[Explore] Map: %dx%d res=%.3f origin=(%.1f,%.1f) %.1f%%",
                               self.map_width, self.map_height, self.map_res,
                               self.map_origin[0], self.map_origin[1],
                               self.explored_ratio * 100)

    def odom_callback(self, msg):
        self.robot_pos = [msg.pose.pose.position.x,
                          msg.pose.pose.position.y,
                          msg.pose.pose.position.z]
        q = msg.pose.pose.orientation
        self.robot_quat = [q.x, q.y, q.z, q.w]

    def costmap_callback(self, msg):
        """Store move_base global costmap for accurate reachability."""
        try:
            self.costmap = np.array(msg.data, dtype=np.int8).reshape(
                (msg.info.height, msg.info.width))
        except Exception as e:
            rospy.logwarn_throttle(10, "[Explore] costmap parse error: %s", e)

    def plan_callback(self, msg):
        """Monitor /move_base/GlobalPlanner/plan.

        When a goal is active and the global planner publishes an empty
        path (no poses), it means no path can be found.  After a small
        debounce threshold we immediately abort the current goal and
        pick a different one.
        """
        if not self.plan_watch_enabled:
            return
        if self.current_goal_world is None:
            self.plan_empty_count = 0
            return

        if len(msg.poses) == 0:
            self.plan_empty_count += 1
            rospy.logwarn_throttle(2.0,
                "[Explore] GlobalPlanner returned EMPTY plan "
                "(%d/%d)", self.plan_empty_count,
                self.plan_empty_threshold)
            if self.plan_empty_count >= self.plan_empty_threshold:
                rospy.logwarn("[Explore] GlobalPlanner FAILED to plan "
                              "to (%.2f,%.2f) -- switching goal",
                              self.current_goal_world[0],
                              self.current_goal_world[1])
                # Mark as failed frontier so we don't retry it
                self._failed_frontiers.add(self._last_goal_grid)
                self.current_goal_world = None
                self.goal_sent_time = None
                self.plan_empty_count = 0
        else:
            # Plan is non-empty -- reset debounce counter
            self.plan_empty_count = 0

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
        if self.explored_ratio > 0.95:
            rospy.loginfo("[Explore] %.1f%% explored (>=95%%), stopping",
                          self.explored_ratio * 100)
            self.stop_exploration(); return

        # --- check current goal ---
        if self.current_goal_world is not None:
            robot_map = self._get_robot_pose_map()
            if robot_map is None:
                return  # can't determine robot pose, skip this tick
            d = math.hypot(robot_map[0] - self.current_goal_world[0],
                           robot_map[1] - self.current_goal_world[1])
            elapsed = (rospy.Time.now() - self.goal_sent_time).to_sec()
            if d < self.goal_tolerance:
                rospy.loginfo("[Explore] Goal reached (%.2fm)", d)
                self.current_goal_world = None
                self.goal_sent_time = None
                # Mark as visited, reset retry counter (we made progress)
                if self._last_goal_grid:
                    self.visited_goals.add(self._last_goal_grid)
                self._frontier_reset_count = 0
                return
            if elapsed > self.goal_timeout:
                rospy.logwarn("[Explore] Goal timeout %.0fs, skip", elapsed)
                self.current_goal_world = None
                self.goal_sent_time = None
                # Mark as visited to avoid repeating
                if self._last_goal_grid:
                    self.visited_goals.add(self._last_goal_grid)
                    self._failed_frontiers.add(self._last_goal_grid)
                return
            return  # still moving

        # --- pick new goal ---
        goal_xy = self._select_goal()
        if goal_xy is None:
            if self._stuck_since is None:
                self._stuck_since = rospy.Time.now()
                rospy.logwarn("[Explore] No reachable frontiers -- "
                              "waiting for map to grow (max %.0fs)",
                              self._max_stuck_duration)
                return
            stuck_elapsed = (rospy.Time.now() - self._stuck_since).to_sec()
            if stuck_elapsed > self._max_stuck_duration:
                rospy.logwarn("[Explore] Stuck for %.0fs, giving up",
                              stuck_elapsed)
                self.stop_exploration(); return
            # Still waiting — retry next tick (throttled log)
            rospy.loginfo_throttle(10.0,
                "[Explore] Still stuck (%.0fs/%.0fs), "
                "waiting for new frontiers...",
                stuck_elapsed, self._max_stuck_duration)
            return

        # Got a goal — reset stuck timer
        self._stuck_since = None
        self._send_goal(goal_xy)
        self.step_count += 1

    # ================================================================
    #  Goal selection & sending
    # ================================================================
    def _select_goal(self):
        """Return (gx, gy) in raw /map grid coords, or None.

        Sorts frontiers by the configured method, snaps each to the
        nearest free cell, and returns the first valid candidate.
        No synchronous path-planning pre-check is performed here;
        plan_callback handles auto-switching on empty plans.
        """
        robot_map = self._get_robot_pose_map()
        if robot_map is None:
            rospy.logwarn_throttle(10, "[Explore] Cannot get robot pose for goal selection")
            return None
        if self.map_res > 0:
            rx = int((robot_map[0] - self.map_origin[0]) / self.map_res)
            ry = int((robot_map[1] - self.map_origin[1]) / self.map_res)
        else:
            rx, ry = 0, 0
        rx = max(0, min(self.map_width - 1, rx))
        ry = max(0, min(self.map_height - 1, ry))

        f = detect_frontiers(self.obstacle_map, self.explored_map,
                             min_frontier_size=self.min_frontier_size)

        # Filter out visited frontiers (within 10 grid cells ~0.5m)
        if self.visited_goals:
            f = [(cx, cy, sz) for (cx, cy, sz) in f
                 if all((cx - vx)**2 + (cy - vy)**2 >= 100
                        for (vx, vy) in self.visited_goals)]
        # Filter out frontiers that previously failed path planning
        if self._failed_frontiers:
            f = [(cx, cy, sz) for (cx, cy, sz) in f
                 if all((cx - fx)**2 + (cy - fy)**2 >= 100
                        for (fx, fy) in self._failed_frontiers)]

        if not f:
            self._frontier_reset_count += 1
            if self._frontier_reset_count > self._max_frontier_resets:
                rospy.logwarn("[Explore] Frontier rescan limit (%d) reached "
                              "-- all frontiers unreachable, stopping",
                              self._max_frontier_resets)
                return None
            self.visited_goals.clear()
            self._failed_frontiers.clear()
            rospy.loginfo("[Explore] All visited / unreachable, rescanning "
                          "(attempt %d/%d)",
                          self._frontier_reset_count,
                          self._max_frontier_resets)
            f = detect_frontiers(self.obstacle_map, self.explored_map,
                                 min_frontier_size=self.min_frontier_size)
            if not f:
                return None

        if self.vis_enabled:
            self._pub_frontier_markers(f)

        # Sort frontiers by configured method, pick best non-obstacle one.
        # Path reachability is NOT pre-checked here (would block the
        # control loop).  plan_callback monitors the global planner and
        # auto-switches goals when empty plans are detected.
        sorted_f = self._sort_frontiers_by_method((rx, ry), f)
        for cx, cy, sz in sorted_f:
            fx, fy = self._snap_to_free_cell(cx, cy)
            # Quick obstacle check (no service call)
            if self.obstacle_map is not None:
                h, w = self.obstacle_map.shape
                if 0 <= fy < h and 0 <= fx < w:
                    if self.obstacle_map[fy, fx] != 0:
                        rospy.loginfo("[Explore] Frontier (%d,%d) in obstacle, skip",
                                      fx, fy)
                        self._failed_frontiers.add((fx, fy))
                        continue
            return (fx, fy)
        return None

    def _sort_frontiers_by_method(self, robot_pos, frontiers):
        """Sort frontiers by the configured exploration method.

        Returns a list of (cx, cy, sz) tuples sorted so that the
        'best' frontier comes first.
        """
        rx, ry = robot_pos
        if self.exploration_method == 'nearest':
            return sorted(frontiers,
                          key=lambda f: (f[0] - rx)**2 + (f[1] - ry)**2)
        elif self.exploration_method == 'largest':
            return sorted(frontiers, key=lambda f: f[2], reverse=True)
        elif self.exploration_method == 'combined':
            return sorted(frontiers,
                          key=lambda f: f[2] / max((f[0] - rx)**2 +
                                                    (f[1] - ry)**2, 1.0),
                          reverse=True)
        return frontiers

    def _snap_to_free_cell(self, cx, cy, max_radius=40):
        """BFS outward from a frontier centroid to find the nearest
        free (white) cell ¡ª i.e. explored, not obstacle, not frontier.

        This ensures goals are always placed in safe known-free space
        adjacent to the frontier, never on the frontier edge itself.

        Args:
            cx, cy: frontier cluster centroid (grid coords)
            max_radius: max search radius in grid cells

        Returns:
            (gx, gy) grid coords of nearest free cell, or (cx, cy)
            as fallback.
        """
        h, w = self.explored_map.shape
        if not (0 <= cx < w and 0 <= cy < h):
            return (cx, cy)

        visited = np.zeros((h, w), dtype=bool)
        q = deque()
        q.append((cx, cy, 0))
        visited[cy, cx] = True

        # 8-directional neighbours
        dirs = [(-1, -1), (-1, 0), (-1, 1),
                (0, -1),           (0, 1),
                (1, -1),  (1, 0),  (1, 1)]

        while q:
            x, y, d = q.popleft()
            if d > max_radius:
                break

            # Free cell: prefer real costmap (cost==0 = reachable),
            # fallback to manual safe_free_map, fallback to raw maps
            if self.costmap is not None:
                ch, cw = self.costmap.shape
                if 0 <= x < cw and 0 <= y < ch and self.costmap[y, x] == 0:
                    return (x, y)
            elif self.safe_free_map is not None and self.safe_free_map[y, x]:
                return (x, y)

            for dx, dy in dirs:
                nx, ny = x + dx, y + dy
                if 0 <= nx < w and 0 <= ny < h and not visited[ny, nx]:
                    visited[ny, nx] = True
                    q.append((nx, ny, d + 1))

        # Fallback: return original centroid
        rospy.logwarn("[Explore] snap_to_free: no free cell within %d "
                      "cells of (%d,%d), using centroid", max_radius, cx, cy)
        return (cx, cy)

    def _get_robot_pose_map(self):
        """Return (x, y) of the robot in the map frame via TF, or None."""
        try:
            trans = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame,
                rospy.Time(0), rospy.Duration(2.0))
            return (trans.transform.translation.x,
                    trans.transform.translation.y)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(10, "[Explore] TF lookup failed: %s", e)
            return None

    def _send_goal(self, goal_xy):
        """Publish PoseStamped to move_base + NavOA."""
        wx = self.map_origin[0] + goal_xy[0] * self.map_res
        wy = self.map_origin[1] + goal_xy[1] * self.map_res

        ps = PoseStamped()
        ps.header.frame_id = self.map_frame
        ps.header.stamp = rospy.Time.now()
        ps.pose.position.x = wx
        ps.pose.position.y = wy
        ps.pose.orientation.w = 1.0

        self.current_goal_world = (wx, wy)
        self._last_goal_grid = (goal_xy[0], goal_xy[1])
        self.goal_sent_time = rospy.Time.now()
        self.plan_empty_count = 0    # reset plan-watch debounce

        # Send to move_base only; path_segment_publisher relays to /split_goal
        self.goal_pub.publish(ps)

        rospy.loginfo("[Explore] step=%d -> world(%.2f,%.2f) [%dx%d map]",
                      self.step_count, wx, wy,
                      self.map_width, self.map_height)
        if self.vis_enabled:
            self._pub_goal_marker(goal_xy)

    # ================================================================
    #  Visualization
    # ================================================================
    def _grid_to_world(self, gx, gy):
        """Convert raw /map grid coords to world coords."""
        return (self.map_origin[0] + gx * self.map_res,
                self.map_origin[1] + gy * self.map_res)

    def _pub_frontier_markers(self, frontiers):
        ma = MarkerArray()
        for i, (cx, cy, sz) in enumerate(frontiers):
            wx, wy = self._grid_to_world(cx, cy)
            m = Marker()
            m.header.frame_id = self.map_frame; m.header.stamp = rospy.Time.now()
            m.ns = "frontiers"; m.id = i; m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = wx
            m.pose.position.y = wy
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
        wx, wy = self._grid_to_world(goal_xy[0], goal_xy[1])
        m = Marker()
        m.header.frame_id = self.map_frame; m.header.stamp = rospy.Time.now()
        m.ns = "explore_goal"; m.id = 0; m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = wx
        m.pose.position.y = wy
        m.pose.position.z = 0.35
        m.scale.x = 0.25; m.scale.y = 0.25; m.scale.z = 0.25
        m.color.a = 1.0; m.color.r = 1.0
        self.goal_marker_pub.publish(m)

    # ================================================================
    #  Lifecycle
    # ================================================================
    def start_exploration(self):
        rospy.loginfo("[Explore] START -> %s", self.goal_topic)
        # Give nav stack time to initialize costmaps
        rospy.loginfo("[Explore] Waiting 3s for nav stack init...")
        rospy.sleep(3.0)
        self.is_exploring = True
        self.step_count = 0

    def stop_exploration(self):
        self.is_exploring = False
        self.current_goal_world = None
        rospy.loginfo("[Explore] STOP %.1f%%", self.explored_ratio * 100)
        self._save_map()

    def _save_map(self):
        if not self.save_data or self.obstacle_map is None or self.explored_map is None:
            return
        h, w = self.obstacle_map.shape
        img = np.zeros((h, w, 3), dtype=np.uint8)
        img[:] = [128, 128, 128]
        img[(self.explored_map > 0) & (self.obstacle_map == 0)] = [255, 255, 255]
        img[self.obstacle_map > 0] = [0, 0, 0]
        if self.frontier_map is not None:
            img[self.frontier_map > 0] = [0, 255, 0]
        cv2.imwrite(os.path.join(self.data_dir, 'final_map.png'),
                    cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        rospy.loginfo("[Explore] Map saved -> %s", self.data_dir)

    def run(self):
        rospy.loginfo("[Explore] Waiting for /map ...")
        t0 = rospy.Time.now() + rospy.Duration(60.0)
        while not rospy.is_shutdown():
            if self.obstacle_map is not None and self.explored_map is not None:
                if np.any(self.explored_map > 0):
                    break
            if rospy.Time.now() > t0:
                rospy.logwarn("[Explore] Map wait timeout")
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
