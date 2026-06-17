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

from geometry_msgs.msg import PoseStamped, PointStamped, TransformStamped, Point
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
        self.base_frame = rospy.get_param('~base_frame', 'base_footprint')
        self.map_topic = rospy.get_param('~map_topic', '/map')
        self.odom_topic = rospy.get_param('~odom_topic', '/odom_rf2o')
        self.goal_topic = rospy.get_param('~goal_topic', '/move_base_simple/goal')
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.5)    # meters
        self.goal_timeout = rospy.get_param('~goal_timeout', 60.0)       # seconds
        self.idle_wait = rospy.get_param('~idle_wait', 1.0)
        self.post_reach_wait = rospy.get_param('~post_reach_wait', 2.5)  # sec to wait after reaching goal

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

        # ---- Map canvas expansion (for Rviz Publish Point outside map) ----
        # Pad the incoming /map with unknown cells on all sides so Rviz
        # has a larger surface to click on via Publish Point.
        # 0 = no padding (original behaviour).
        # 400 cells at 0.05 res = 20 m extra on each side.
        self._map_padding_cells = rospy.get_param('~map_padding_cells', 0)

        # ---- State ----
        self.obstacle_map = None
        self.explored_map = None
        self.frontier_map = None
        self.safe_free_map = None     # free cells outside inflation radius (fallback)
        self.costmap = None           # 2D array from move_base global costmap
        self.free_mask = None          # truly-free cells (cost==0) for frontier detection
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
        self._post_reach_time = None        # timestamp when last goal was reached
        self._last_goal_world = None        # world coords of the last successfully sent goal
        self._local_radius = rospy.get_param('~local_radius', 60)  # grid cells (~3m at 0.05 res)
        self._use_regional_explore = rospy.get_param('~regional_explore', True)

        # ---- Loop closure revisit ----
        # When no new frontiers detected in the region for trigger_delay
        # seconds, revisit previously explored points to trigger SLAM
        # loop-closure.  If new frontiers appear mid-revisit, abort and
        # explore them; loop closure will retrigger after the next idle.
        self._enable_loop_closure = rospy.get_param('~enable_loop_closure', True)
        self._loop_closure_min_distance = rospy.get_param('~loop_closure_min_distance', 80)  # grid cells (~4m at 0.05 res)
        self._loop_closure_max_revisits = rospy.get_param('~loop_closure_max_revisits', 5)
        self._loop_closure_trigger_delay = rospy.get_param('~loop_closure_trigger_delay', 20.0)  # seconds
        self._loop_closure_history = []       # list of (wx, wy, gx, gy) world+grid coords visited
        self._loop_closure_phase = False      # True when actively revisiting
        self._loop_closure_count = 0          # revisits sent in current phase
        self._loop_closure_visited_now = set()  # grid cells visited during current phase
        self._no_frontier_since = None        # rospy.Time when we first detected no frontiers
        # Periodic frontier recalculation during navigation
        self._frontier_update_interval = rospy.get_param('~frontier_update_interval', 5.0)  # sec
        self._last_frontier_update = None       # rospy.Time of last recalculation
        self._goal_check_radius = rospy.get_param('~goal_check_radius', 30)  # cells (~1.5m)

        # ---- Exploration region (Rviz "Publish Point" tool) ----
        # Click points in Rviz to define a polygon region.
        # Frontiers outside this region are ignored.
        # Click additional points at any time to expand the region.
        self._enable_region_filter = rospy.get_param('~enable_region_filter', False)
        self._clicked_point_topic = rospy.get_param('~clicked_point_topic', '/clicked_point')
        self._region_default_radius = rospy.get_param('~region_default_radius', 3.0)  # meters, for single-point circle
        # "convex_hull" — order-independent, always convex
        # "polygon"    — points connected in click order, can be concave;
        #                 polygon only activates when the last point is
        #                 within _region_close_threshold of the first point
        self._region_shape_mode = rospy.get_param('~region_shape_mode', 'convex_hull')
        self._region_close_threshold = rospy.get_param('~region_close_threshold', 2.0)  # meters
        self.exploration_region_points = []      # list of (wx, wy) world coords
        self.exploration_region_mask = None      # binary mask at map resolution

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
        self.region_marker_pub = rospy.Publisher(
            '/exploration/region', Marker, queue_size=1, latch=True)
        self.expanded_map_pub = rospy.Publisher(
            '/exploration/expanded_map', OccupancyGrid, queue_size=1, latch=True)

        # ---- Subscribers ----
        rospy.Subscriber(self.map_topic, OccupancyGrid,
                         self.map_callback, queue_size=1)
        rospy.Subscriber(self.odom_topic, Odometry,
                         self.odom_callback, queue_size=1)
        self.plan_sub = rospy.Subscriber(self.plan_topic, Path,
                                          self.plan_callback, queue_size=1)
        self.costmap_sub = rospy.Subscriber(self.costmap_topic, OccupancyGrid,
                                             self.costmap_callback, queue_size=1)
        self.clicked_point_sub = rospy.Subscriber(self._clicked_point_topic, PointStamped,
                                                   self._clicked_point_callback, queue_size=5)

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
        # Work directly with raw cartographer map, then optionally pad
        # the canvas with unknown cells so Rviz Publish Point works
        # over a larger area.
        raw_w = msg.info.width
        raw_h = msg.info.height
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        oz = msg.info.origin.position.z

        pad = self._map_padding_cells
        if pad > 0:
            # Expand the canvas: add `pad` unknown cells on all four sides
            self.map_width = raw_w + 2 * pad
            self.map_height = raw_h + 2 * pad
            self.map_res = res
            self.map_origin = [ox - pad * res,
                               oy - pad * res,
                               oz]
            # Large canvas filled with -1 (unknown)
            raw = np.full((self.map_height, self.map_width), -1, dtype=np.int8)
            raw[pad:pad + raw_h, pad:pad + raw_w] = (
                np.array(msg.data, dtype=np.int8).reshape((raw_h, raw_w)))

            # Publish expanded map for Rviz (subscribe in Rviz to
            # /exploration/expanded_map instead of /map)
            expanded = OccupancyGrid()
            expanded.header = msg.header
            expanded.info.resolution = self.map_res
            expanded.info.width = self.map_width
            expanded.info.height = self.map_height
            expanded.info.origin.position.x = self.map_origin[0]
            expanded.info.origin.position.y = self.map_origin[1]
            expanded.info.origin.position.z = self.map_origin[2]
            expanded.info.origin.orientation.w = 1.0
            expanded.data = raw.flatten().tolist()
            self.expanded_map_pub.publish(expanded)
        else:
            self.map_width = raw_w
            self.map_height = raw_h
            self.map_res = res
            self.map_origin = [ox, oy, oz]
            raw = np.array(msg.data, dtype=np.int8).reshape((raw_h, raw_w))

        # obstacle: value > 50 (occupied), explored: value >= 0 (known)
        self.obstacle_map = (raw > 50).astype(np.uint8)
        self.explored_map = (raw >= 0).astype(np.uint8)

        # ---- Compute inflation kernels once ----
        kernel = np.ones((3, 3), dtype=np.uint8)
        ksize = 2 * self.inflation_cells + 1
        inf_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                                (ksize, ksize))
        obs_inflated = cv2.dilate(self.obstacle_map, inf_kernel, iterations=1)

        # safe_free_map: explored cells outside inflated obstacle zone
        # (fallback when costmap is unavailable)
        self.safe_free_map = ((self.explored_map > 0) &
                              (obs_inflated == 0))

        # ---- free_mask: truly-free cells for frontier detection ----
        # Prefer move_base costmap (cost==0 = no inflation, truly reachable).
        # Fall back to safe_free_map when costmap is unavailable.
        if (self.costmap is not None and
                self.costmap.shape[0] == self.map_height and
                self.costmap.shape[1] == self.map_width):
            self.free_mask = (self.costmap == 0)
            rospy.loginfo_throttle(30,
                "[Explore] free_mask from costmap: %.0f%% truly-free cells",
                100.0 * np.sum(self.free_mask) / max(self.free_mask.size, 1))
        else:
            self.free_mask = self.safe_free_map
            rospy.loginfo_throttle(30,
                "[Explore] free_mask from safe_free_map (costmap unavailable)")

        # frontier_map: free cells adjacent to unknown
        unknown = (self.explored_map == 0).astype(np.uint8)
        dil_unk = cv2.dilate(unknown, kernel, iterations=1)
        self.frontier_map = (self.free_mask & (dil_unk > 0)).astype(np.uint8)

        obs = np.sum(self.obstacle_map)
        total = self.obstacle_map.size
        exp = np.sum(self.explored_map) - obs
        # Compute explored_ratio over the *original* map area only
        # (padding cells are always unknown and would skew the ratio)
        if pad > 0:
            obs_orig = int(np.sum(self.obstacle_map[pad:pad + raw_h,
                                                     pad:pad + raw_w]))
            total_orig = raw_h * raw_w
            exp_orig = int(np.sum(self.explored_map[pad:pad + raw_h,
                                                     pad:pad + raw_w])) - obs_orig
            self.explored_ratio = exp_orig / max(total_orig - obs_orig, 1)
        else:
            self.explored_ratio = exp / max(total - obs, 1)

        rospy.loginfo_throttle(30, "[Explore] Map: %dx%d res=%.3f origin=(%.1f,%.1f) %.1f%%",
                               self.map_width, self.map_height, self.map_res,
                               self.map_origin[0], self.map_origin[1],
                               self.explored_ratio * 100)

        # Rebuild exploration region mask when map updates
        # (map origin / resolution / dimensions may have changed)
        if self._enable_region_filter and self.exploration_region_points:
            self._build_exploration_mask()

    def odom_callback(self, msg):
        self.robot_pos = [msg.pose.pose.position.x,
                          msg.pose.pose.position.y,
                          msg.pose.pose.position.z]
        q = msg.pose.pose.orientation
        self.robot_quat = [q.x, q.y, q.z, q.w]

    def _clicked_point_callback(self, msg):
        """Handle Rviz "Publish Point" clicks to define/expand exploration region.

        Transforms the clicked point to map_frame, adds it to the region
        point list, rebuilds the binary mask, and publishes a polygon
        marker for visualization in Rviz.
        """
        if not self._enable_region_filter:
            return

        # Transform to map frame if needed
        try:
            if msg.header.frame_id and msg.header.frame_id != self.map_frame:
                trans = self.tf_buffer.lookup_transform(
                    self.map_frame, msg.header.frame_id,
                    msg.header.stamp, rospy.Duration(1.0))
                # Simple transform (no rotation for PointStamped — we just
                # need the translation offset)
                wx = msg.point.x + trans.transform.translation.x
                wy = msg.point.y + trans.transform.translation.y
            else:
                wx = msg.point.x
                wy = msg.point.y
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("[Explore] TF error on clicked point: %s", e)
            return

        self.exploration_region_points.append((wx, wy))
        n = len(self.exploration_region_points)
        rospy.loginfo("[Explore] Region point #%d added: world(%.2f, %.2f)",
                      n, wx, wy)

        # Rebuild mask & republish polygon
        self._build_exploration_mask()
        self._pub_region_marker()

    def _build_exploration_mask(self):
        """Build a binary mask from accumulated clicked points.

        - 0 points: mask = None (no filtering, all frontiers allowed)
        - 1 point:  circle with radius _region_default_radius
        - 2 points: axis-aligned rectangle
        - 3+ points: convex-hull or polygon (see region_shape_mode)

        Mask shape matches the current /map.  A value of 1 means the
        cell is inside the allowed exploration region.
        """
        if (not self._enable_region_filter or
                not self.exploration_region_points or
                self.map_width == 0 or self.map_height == 0):
            self.exploration_region_mask = None
            return

        # Require at least 4 points to define a meaningful polygon region
        if len(self.exploration_region_points) < 4:
            self.exploration_region_mask = None
            return

        # Convert world coords to grid coords (no clamping — cv2.fillPoly
        # naturally clips to image bounds, allowing points outside the
        # current map to define a region that covers future map expansion)
        grid_pts = []
        for wx, wy in self.exploration_region_points:
            gx = int((wx - self.map_origin[0]) / self.map_res)
            gy = int((wy - self.map_origin[1]) / self.map_res)
            grid_pts.append([gx, gy])
        grid_pts = np.array(grid_pts, dtype=np.int32)

        mask = np.zeros((self.map_height, self.map_width), dtype=np.uint8)
        n = len(grid_pts)

        if self._region_shape_mode == 'polygon':
            # Check that the polygon loop is closed:
            # the last point must be within _region_close_threshold
            # of the first point, otherwise wait for more points.
            x0, y0 = self.exploration_region_points[0]
            xn, yn = self.exploration_region_points[-1]
            dist = math.hypot(xn - x0, yn - y0)
            if dist > self._region_close_threshold:
                self.exploration_region_mask = None
                rospy.loginfo_throttle(10.0,
                    "[Explore] Polygon not closed yet: last point "
                    "%.2fm from first (threshold %.2fm). "
                    "Keep adding points to close the loop.",
                    dist, self._region_close_threshold)
                return
            # Polygon closed — fill the shape formed by click order
            cv2.fillPoly(mask, [grid_pts], 1)
        else:
            # convex_hull (default): order-independent, always convex
            hull = cv2.convexHull(grid_pts)
            cv2.fillPoly(mask, [hull], 1)

        self.exploration_region_mask = mask
        rospy.loginfo("[Explore] Region mask built (%s): %d points, "
                      "%d cells inside",
                      self._region_shape_mode, n, int(np.sum(mask)))

    def _pub_region_marker(self):
        """Publish a LINE_STRIP marker showing the exploration region polygon.

        Only meaningful when >= 3 points exist (polygon).  For 1-2 points
        we publish a simplified representation.
        """
        if not self._enable_region_filter:
            return

        m = Marker()
        m.header.frame_id = self.map_frame
        m.header.stamp = rospy.Time.now()
        m.ns = "exploration_region"
        m.id = 0
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.08  # line width
        m.color.a = 0.6
        m.color.b = 1.0

        pts = self.exploration_region_points
        if len(pts) == 0:
            m.action = Marker.DELETE
            self.region_marker_pub.publish(m)
            return

        if len(pts) == 1:
            # Single point: show a circle approximation
            m.type = Marker.LINE_STRIP
            cx, cy = pts[0]
            r = self._region_default_radius
            n_seg = 32
            for i in range(n_seg + 1):
                angle = 2.0 * math.pi * i / n_seg
                p = Point()
                p.x = cx + r * math.cos(angle)
                p.y = cy + r * math.sin(angle)
                p.z = 0.15
                m.points.append(p)
        elif len(pts) == 2:
            # Rectangle corners
            m.type = Marker.LINE_STRIP
            (x1, y1), (x2, y2) = pts
            corners = [(x1, y1), (x2, y1), (x2, y2), (x1, y2), (x1, y1)]
            for cx, cy in corners:
                p = Point()
                p.x = cx; p.y = cy; p.z = 0.15
                m.points.append(p)
        else:
            # polygon mode: connect points in click order (can be concave)
            # convex_hull mode: compute convex hull for order-independent coverage
            m.type = Marker.LINE_STRIP
            if self._region_shape_mode == 'polygon':
                # Draw vertices in click order, close the loop
                for wx, wy in pts:
                    p = Point()
                    p.x = wx; p.y = wy; p.z = 0.15
                    m.points.append(p)
                if len(m.points) > 0:
                    m.points.append(m.points[0])
            else:
                # convex_hull: compute hull from world coords
                scale = 10000.0
                wp = np.array([[int(p[0] * scale), int(p[1] * scale)] for p in pts],
                              dtype=np.int32)
                hull = cv2.convexHull(wp)
                for pt in hull:
                    p = Point()
                    p.x = pt[0][0] / scale
                    p.y = pt[0][1] / scale
                    p.z = 0.15
                    m.points.append(p)
                if len(m.points) > 0:
                    m.points.append(m.points[0])

        self.region_marker_pub.publish(m)

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

            # ---- Periodic frontier recalculation ----
            # Re-detect frontiers from the latest map while moving.
            # If the target frontier cluster has vanished (area now fully
            # explored), abort the stale goal and pick a new one immediately.
            now = rospy.Time.now()
            if (self._last_frontier_update is None or
                    (now - self._last_frontier_update).to_sec() >= self._frontier_update_interval):
                self._last_frontier_update = now
                if (self._last_goal_grid is not None and
                        self.obstacle_map is not None and
                        self.explored_map is not None):
                    f = detect_frontiers(self.obstacle_map, self.explored_map,
                                         min_frontier_size=self.min_frontier_size,
                                         free_mask=self.free_mask)
                    gx, gy = self._last_goal_grid
                    still_exists = any((cx - gx)**2 + (cy - gy)**2 <= self._goal_check_radius**2
                                       for (cx, cy, _sz, _wb) in f)
                    if not still_exists:
                        rospy.logwarn("[Explore] Frontier near goal (%d,%d) "
                                      "vanished — aborting, will reselect", gx, gy)
                        self.current_goal_world = None
                        self.goal_sent_time = None
                        if self._last_goal_grid:
                            self.visited_goals.add(self._last_goal_grid)
                        # Fall through to goal selection below
                    else:
                        rospy.loginfo_throttle(10.0,
                            "[Explore] Frontier still alive near goal (%d,%d), continuing",
                            gx, gy)

            # If goal was just cleared above, skip the rest of goal checking
            if self.current_goal_world is not None:
                d = math.hypot(robot_map[0] - self.current_goal_world[0],
                               robot_map[1] - self.current_goal_world[1])
                elapsed = (rospy.Time.now() - self.goal_sent_time).to_sec()
                if d < self.goal_tolerance:
                    rospy.loginfo("[Explore] Goal reached (%.2fm) — "
                                  "waiting %.1fs for map update",
                                  d, self.post_reach_wait)
                    self.current_goal_world = None
                    self.goal_sent_time = None
                    # Mark as visited, reset retry counter (we made progress)
                    if self._last_goal_grid:
                        self.visited_goals.add(self._last_goal_grid)
                        # Record in loop-closure history for future revisits
                        self._add_to_loop_closure_history(
                            self._last_goal_world, self._last_goal_grid)
                    self._frontier_reset_count = 0
                    self._post_reach_time = rospy.Time.now()
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

        # --- post-reach wait: let Cartographer update the map ---
        if self._post_reach_time is not None:
            elapsed = (rospy.Time.now() - self._post_reach_time).to_sec()
            if elapsed < self.post_reach_wait:
                return  # still waiting for map to settle
            self._post_reach_time = None
            rospy.loginfo("[Explore] Post-reach wait done, selecting next goal")

        # --- wait for exploration region to be defined (polygon mode only) ---
        # convex_hull mode: region is optional — explore freely if not set
        # polygon mode: region is mandatory — must close the loop first
        if (self._enable_region_filter and
                self._region_shape_mode == 'polygon' and
                self.exploration_region_mask is None):
            pts_needed = max(0, 4 - len(self.exploration_region_points))
            rospy.loginfo_throttle(5.0,
                "[Explore] Waiting for region definition "
                "(%d more point(s) needed)...", pts_needed)
            return

        # --- pick new goal ---
        goal_xy = self._select_goal()
        if goal_xy is None:
            # ---- No frontiers found ----
            now = rospy.Time.now()

            if self._enable_loop_closure:

                if not self._loop_closure_phase:
                    # ---- Track idle time before triggering loop closure ----
                    if self._no_frontier_since is None:
                        self._no_frontier_since = now
                        rospy.loginfo("[Explore] No frontiers — will trigger "
                                      "loop-closure in %.0fs if none appear",
                                      self._loop_closure_trigger_delay)
                        return
                    idle = (now - self._no_frontier_since).to_sec()
                    if idle < self._loop_closure_trigger_delay:
                        rospy.loginfo_throttle(5.0,
                            "[Explore] No frontiers for %.0f/%.0fs, waiting...",
                            idle, self._loop_closure_trigger_delay)
                        return
                    # ---- Trigger: enter loop closure phase ----
                    if not self._loop_closure_history:
                        rospy.logwarn("[Explore] No frontiers & no revisit "
                                      "history — giving up")
                        self.stop_exploration(); return
                    rospy.loginfo("[Explore] No frontiers for %.0fs — "
                                  "starting loop-closure revisit (%d history)",
                                  idle, len(self._loop_closure_history))
                    self._loop_closure_phase = True
                    self._loop_closure_count = 0
                    self._loop_closure_visited_now.clear()

                # ---- In loop closure phase: pick next revisit point ----
                if self._loop_closure_count < self._loop_closure_max_revisits:
                    goal_xy = self._select_loop_closure_goal()
                    if goal_xy is not None:
                        self._send_goal(goal_xy)
                        self.step_count += 1
                        self._loop_closure_count += 1
                        return
                    # No more suitable revisit candidates
                    rospy.loginfo("[Explore] Loop-closure: no more distant "
                                  "revisit candidates (%d/%d sent)",
                                  self._loop_closure_count,
                                  self._loop_closure_max_revisits)

                # ---- Loop closure finished: stop exploration ----
                rospy.loginfo("[Explore] Loop-closure revisit done "
                              "(%d points) — stopping exploration",
                              self._loop_closure_count)
                self._loop_closure_phase = False
                self.stop_exploration(); return

            # ---- Normal stuck handling (loop closure disabled) ----
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
            rospy.loginfo_throttle(10.0,
                "[Explore] Still stuck (%.0fs/%.0fs), "
                "waiting for new frontiers...",
                stuck_elapsed, self._max_stuck_duration)
            return

        # Got a goal — reset stuck / no-frontier timers
        self._stuck_since = None
        self._no_frontier_since = None
        # If we were in loop-closure phase but a new frontier appeared,
        # exit the phase to prioritise fresh exploration.
        if self._loop_closure_phase:
            rospy.loginfo("[Explore] New frontiers found during "
                          "loop-closure phase — resuming normal exploration")
            self._loop_closure_phase = False
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
                             min_frontier_size=self.min_frontier_size,
                             free_mask=self.free_mask)

        # ---- Filter by exploration region mask (Rviz Publish Point) ----
        if self.exploration_region_mask is not None:
            before = len(f)
            f = [(cx, cy, sz, wb) for (cx, cy, sz, wb) in f
                 if (0 <= cx < self.map_width and 0 <= cy < self.map_height
                     and self.exploration_region_mask[cy, cx] > 0)]
            rospy.loginfo_throttle(15.0,
                "[Explore] Region filter: %d/%d frontiers inside region",
                len(f), before)

        # Filter out visited frontiers (within 10 grid cells ~0.5m)
        if self.visited_goals:
            f = [(cx, cy, sz, wb) for (cx, cy, sz, wb) in f
                 if all((cx - vx)**2 + (cy - vy)**2 >= 100
                        for (vx, vy) in self.visited_goals)]
        # Filter out frontiers that previously failed path planning
        if self._failed_frontiers:
            f = [(cx, cy, sz, wb) for (cx, cy, sz, wb) in f
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
                                 min_frontier_size=self.min_frontier_size,
                                 free_mask=self.free_mask)
            # Re-apply region filter after rescan (bugfix: was leaking
            # frontiers from outside the user-defined region)
            if self.exploration_region_mask is not None:
                before = len(f)
                f = [(cx, cy, sz, wb) for (cx, cy, sz, wb) in f
                     if (0 <= cx < self.map_width and 0 <= cy < self.map_height
                         and self.exploration_region_mask[cy, cx] > 0)]
                rospy.loginfo("[Explore] Region filter (rescan): %d/%d inside",
                              len(f), before)
            if not f:
                return None

        if self.vis_enabled:
            self._pub_frontier_markers(f)

        # ---- Regional exploration: prefer frontiers near last goal ----
        # This prevents the robot from jumping across the map after a
        # frontier cluster disappears mid-navigation.  By prioritizing
        # frontiers close to the area we just reached, we achieve a
        # depth-first local exploration pattern.
        if (self._use_regional_explore and
                self._last_goal_world is not None and
                self._last_goal_grid is not None):
            lgx, lgy = self._last_goal_grid
            nearby = [(cx, cy, sz, wb) for (cx, cy, sz, wb) in f
                      if (cx - lgx)**2 + (cy - lgy)**2 <= self._local_radius**2]
            if nearby:
                rospy.loginfo("[Explore] Regional: %d frontiers within "
                              "%d cells of last goal (%d,%d)",
                              len(nearby), self._local_radius, lgx, lgy)
                # Sort nearby frontiers by the configured method
                f = self._sort_frontiers_by_method((rx, ry), nearby)
            else:
                rospy.loginfo("[Explore] No frontiers near last goal, "
                              "falling back to global selection")
                f = self._sort_frontiers_by_method((rx, ry), f)
        else:
            # Global selection (original behavior)
            f = self._sort_frontiers_by_method((rx, ry), f)

        for cx, cy, sz, wb in f:
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

    def _select_loop_closure_goal(self):
        """Select a previously visited point far from the robot for loop closure.

        Returns (gx, gy) grid coords of a suitable revisit point, or None.

        Strategy:
          - Pick visited points that are far from the current robot pose
            (>= _loop_closure_min_distance cells away).
          - Prefer points that haven't been visited in this loop-closure round.
          - If multiple candidates, pick the farthest one to maximise
            the loop-closure trajectory.
        """
        robot_map = self._get_robot_pose_map()
        if robot_map is None:
            return None
        if self.map_res > 0:
            rx = int((robot_map[0] - self.map_origin[0]) / self.map_res)
            ry = int((robot_map[1] - self.map_origin[1]) / self.map_res)
        else:
            rx, ry = 0, 0
        rx = max(0, min(self.map_width - 1, rx))
        ry = max(0, min(self.map_height - 1, ry))

        min_dist_sq = self._loop_closure_min_distance ** 2
        candidates = []

        for (wx, wy, gx, gy, _ts) in self._loop_closure_history:
            # Skip points already visited in this loop-closure round
            if (gx, gy) in self._loop_closure_visited_now:
                continue
            # Skip points too close to current position
            d2 = (gx - rx) ** 2 + (gy - ry) ** 2
            if d2 < min_dist_sq:
                continue
            # Skip points that are now obstacles (map may have changed)
            if self.obstacle_map is not None:
                h, w = self.obstacle_map.shape
                if 0 <= gy < h and 0 <= gx < w:
                    if self.obstacle_map[gy, gx] != 0:
                        continue
            # Check costmap reachability if available
            if self.costmap is not None:
                ch, cw = self.costmap.shape
                if 0 <= gx < cw and 0 <= gy < ch:
                    if self.costmap[gy, gx] > 0:
                        continue

            candidates.append((gx, gy, d2))

        if not candidates:
            rospy.loginfo("[Explore] LoopClosure: no distant revisit "
                          "candidates (min_dist=%d cells, history=%d)",
                          self._loop_closure_min_distance,
                          len(self._loop_closure_history))
            return None

        # Pick the farthest candidate to maximise loop-closure path
        candidates.sort(key=lambda x: x[2], reverse=True)
        gx, gy, d2 = candidates[0]

        # Snap to a nearby free cell in case the exact grid cell is not ideal
        fx, fy = self._snap_to_free_cell(gx, gy)
        self._loop_closure_visited_now.add((fx, fy))

        rospy.loginfo("[Explore] LoopClosure: revisiting (%d,%d) "
                      "dist=%.1f cells, %d/%d",
                      fx, fy, math.sqrt(d2),
                      self._loop_closure_count + 1,
                      self._loop_closure_max_revisits)
        return (fx, fy)

    def _sort_frontiers_by_method(self, robot_pos, frontiers):
        """Sort frontiers by the configured exploration method.

        frontiers: list of (cx, cy, size, white_border) tuples.
        Returns the list sorted so the 'best' frontier comes first.

        Methods:
          nearest  – closest Euclidean distance to robot
          largest  – biggest frontier cluster (by cell count)
          whitest  – most adjacent explored-free (white) cells
          combined – score = size * white_border / distance
        """
        rx, ry = robot_pos
        if self.exploration_method == 'nearest':
            return sorted(frontiers,
                          key=lambda f: (f[0] - rx)**2 + (f[1] - ry)**2)
        elif self.exploration_method == 'largest':
            return sorted(frontiers, key=lambda f: f[2], reverse=True)
        elif self.exploration_method == 'whitest':
            # Frontier with most adjacent white (explored-free) cells.
            # Tie-break by size for stability.
            return sorted(frontiers,
                          key=lambda f: (f[3], f[2]), reverse=True)
        elif self.exploration_method == 'combined':
            # Score = size * white_border / distance.
            # Favours large frontiers with lots of known free space nearby.
            return sorted(frontiers,
                          key=lambda f: (f[2] * max(f[3], 1) /
                                         max((f[0] - rx)**2 +
                                             (f[1] - ry)**2, 1.0)),
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

    def _add_to_loop_closure_history(self, world_xy, grid_xy):
        """Record a visited point for potential loop-closure revisits.

        Deduplicates by grid coords (within 5 cells ~0.25m).
        Caps history at 200 entries to limit memory.
        """
        wx, wy = world_xy
        gx, gy = grid_xy
        # Deduplicate: skip if a point within 5 cells already exists
        for (_, _, hgx, hgy, _) in self._loop_closure_history:
            if abs(hgx - gx) <= 5 and abs(hgy - gy) <= 5:
                return
        self._loop_closure_history.append((wx, wy, gx, gy, rospy.Time.now()))
        # Cap history size
        if len(self._loop_closure_history) > 200:
            self._loop_closure_history = self._loop_closure_history[-200:]

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
        self._last_goal_world = (wx, wy)
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
        for i, (cx, cy, sz, _wb) in enumerate(frontiers):
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
