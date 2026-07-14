#!/usr/bin/env python3
"""
status_state.py — ROS1 node that publishes localization, mapping, navigation,
and boundary status on four topics by deriving the values from real ROS topics
and services.

Topic derivation logic (overridable via private params for testing):

  /localization_status    ->  /matching_score (Cartographer) + timeout detection.
                              Values: "LOCALIZED HIGH" | "LOCALIZED LOW" | "LOST"

  /Mapping_status         ->  /get_trajectory_states service (Cartographer) +
                              /app/map_status (map_control).
                              Values: "MAPPING ON" | "LOCALIZATION ON" | "LOST" | "INITIALIZING"

  /Nav_status             ->  /move_base/status (GoalStatusArray) + cmd_vel activity.
                              Values: "IDLE" | "NAVIGATING" | "ARRIVED" | "FAILED" | "CANCELLED"

  /monitor_map_boundary   ->  /map (OccupancyGrid) + map→base_footprint TF.
                              Values: "INSIDE_MAP" | "NEAR_BOUNDARY" | "OUT_OF_MAP"

Private parameters (~):
  _localization  (str, default: auto) — override to force a value
  _mapping       (str, default: auto) — override to force a value
  _nav           (str, default: auto) — override to force a value
  _rate          (float, default: 1.0)     — publish rate in Hz
"""

import os
import json
import math

import numpy as np

import rospy
import actionlib_msgs.msg
from std_msgs.msg import String, Float32
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist, PoseStamped
import tf2_ros

# Optional import: cartographer_ros_msgs is only available when the workspace
# is sourced. The service call is guarded with try/except so the node works
# gracefully without it (falls back to matching_score-based mapping logic).
try:
    from cartographer_ros_msgs.srv import GetTrajectoryStates
    _HAS_CARTO_SRV = True
except ImportError:
    _HAS_CARTO_SRV = False
    GetTrajectoryStates = None


# ---------------------------------------------------------------------------
#  Valid status sets
# ---------------------------------------------------------------------------
VALID_LOC_STATUSES = frozenset({"LOCALIZED HIGH", "LOCALIZED LOW", "LOST"})
VALID_MAP_STATUSES = frozenset({"MAPPING ON", "LOCALIZATION ON", "LOST", "INITIALIZING"})
VALID_NAV_STATUSES = frozenset({"IDLE", "NAVIGATING", "ARRIVED", "FAILED", "CANCELLED"})
VALID_OOB_STATUSES = frozenset({"INSIDE_MAP", "NEAR_BOUNDARY", "OUT_OF_MAP"})

# Cartographer trajectory state constants (from TrajectoryStates.msg)
TRAJ_ACTIVE = 0
TRAJ_FINISHED = 1
TRAJ_FROZEN = 2
TRAJ_DELETED = 3

# move_base GoalStatus constants (from actionlib_msgs/GoalStatus.msg)
GB_ACTIVE = 1
GB_SUCCEEDED = 3
GB_ABORTED = 4
GB_PREEMPTED = 5  # cancelled


class MapInfo:
    """Wraps a nav_msgs/OccupancyGrid for boundary-monitor queries.

    Provides grid-to-world coordinate helpers and switch detection.
    """

    __slots__ = (
        "frame_id", "resolution", "width", "height",
        "x0", "y0", "data", "_data_np",
    )

    def __init__(self, msg):
        self.frame_id = msg.header.frame_id
        self.resolution = msg.info.resolution
        self.width = msg.info.width
        self.height = msg.info.height
        self.x0 = msg.info.origin.position.x
        self.y0 = msg.info.origin.position.y
        self.data = msg.data
        self._data_np = np.array(msg.data, dtype=np.int8).reshape(
            (self.height, self.width))

    @property
    def rect(self):
        """Full grid rectangle in map frame: ``(xmin, ymin, xmax, ymax)``."""
        return (
            self.x0,
            self.y0,
            self.x0 + self.width * self.resolution,
            self.y0 + self.height * self.resolution,
        )

    @property
    def tight(self):
        """Bounding box of known cells (value >= 0) in map frame.

        Falls back to ``.rect`` when no known cells exist.
        """
        known = np.argwhere(self._data_np >= 0)
        if known.size == 0:
            return self.rect
        rows, cols = known[:, 0], known[:, 1]
        ymin = self.y0 + rows.min() * self.resolution
        ymax = self.y0 + (rows.max() + 1) * self.resolution
        xmin = self.x0 + cols.min() * self.resolution
        xmax = self.x0 + (cols.max() + 1) * self.resolution
        return (xmin, ymin, xmax, ymax)

    def signature(self):
        """Return a tuple that uniquely identifies this map (for switch detection)."""
        return (
            self.frame_id,
            self.width,
            self.height,
            round(self.resolution, 6),
            round(self.x0, 6),
            round(self.y0, 6),
        )

    def cell_value(self, x, y):
        """Occupancy value (-1 unknown, 0…100) at world ``(x, y)``.

        Returns ``None`` when ``(x, y)`` falls outside the grid.
        """
        col = int(math.floor((x - self.x0) / self.resolution))
        row = int(math.floor((y - self.y0) / self.resolution))
        if 0 <= row < self.height and 0 <= col < self.width:
            return int(self._data_np[row, col])
        return None


class StatusStateNode(object):
    """Derives and publishes robot status from real ROS topics/services."""

    def __init__(self):
        rospy.init_node("status_state")

        # ------------------------------------------------------------------
        #  Publishers
        # ------------------------------------------------------------------
        self._pub_loc = rospy.Publisher(
            "/localization_status", String, queue_size=5, latch=True)
        self._pub_map = rospy.Publisher(
            "/Mapping_status", String, queue_size=5, latch=True)
        self._pub_nav = rospy.Publisher(
            "/Nav_status", String, queue_size=5, latch=True)
        self._pub_oob = rospy.Publisher(
            "/monitor_map_boundary", String, queue_size=5, latch=True)

        # ------------------------------------------------------------------
        #  Configuration parameters
        # ------------------------------------------------------------------
        self._rate_hz = max(rospy.get_param("~rate", 1.0), 0.1)
        self._rate = rospy.Rate(self._rate_hz)

        # Matching-score thresholds for localization health
        self._score_ok = rospy.get_param("~score_ok", 0.7)
        self._score_warn = rospy.get_param("~score_warn", 0.3)
        # How long (seconds) without a matching_score before we call ERROR
        self._score_timeout = rospy.get_param("~score_timeout", 5.0)

        # Override params — set to a valid value to pin a status manually
        self._loc_override = self._resolve_initial("_localization", None,
                                                    VALID_LOC_STATUSES)
        self._map_override = self._resolve_initial("_mapping", None,
                                                    VALID_MAP_STATUSES)
        self._nav_override = self._resolve_initial("_nav", None,
                                                    VALID_NAV_STATUSES)

        # ------------------------------------------------------------------
        #  Map boundary config (for /monitor_map_boundary detection)
        # ------------------------------------------------------------------
        self._map_topic           = rospy.get_param("~map_topic", "/map")
        self._warn_distance_m     = rospy.get_param("~warn_distance_m", 0.5)
        self._exit_hysteresis_m   = rospy.get_param("~exit_hysteresis_m", 0.1)
        self._treat_unknown_as_oob = rospy.get_param(
            "~treat_unknown_as_oob", True)
        self._use_tight_bbox      = rospy.get_param("~use_tight_bbox", True)
        self._map_settle_time_s   = rospy.get_param("~map_settle_time_s", 3.0)
        self._oob_wall_threshold  = rospy.get_param("~oob_wall_threshold", 90)
        self._robot_frame         = rospy.get_param("~robot_frame", "base_footprint")
        self._map_frame           = rospy.get_param("~map_frame", "map")

        # ------------------------------------------------------------------
        #  Internal state
        # ------------------------------------------------------------------
        # Latest values from subscribers
        self._matching_score = None          # float
        self._score_stamp = None             # rospy.Time
        self._map_control_status = None      # str from /app/map_status
        self._traj_states = None             # list of int (trajectory states)
        self._move_base_status = None        # int (GoalStatus status)
        self._move_base_text = None          # str
        self._last_cmd_stamp = None          # rospy.Time — last non-zero cmd_vel
        self._map_info = None                # MapInfo — latest cached OccupancyGrid
        self._prev_map_info_sig = None       # tuple — signature for switch detection
        self._settle_until = None            # rospy.Time — suppress alerts until

        # Previous published values (for change detection)
        self._prev_loc = None
        self._prev_map = None
        self._prev_nav = None
        self._prev_oob = None

        # Whether services are available (cache to avoid spamming)
        self._traj_srv_available = None

        # Duration for "no command" = IDLE threshold
        self._nav_idle_timeout = rospy.get_param("~nav_idle_timeout", 2.0)

        # ------------------------------------------------------------------
        #  Subscribers
        # ------------------------------------------------------------------
        self._sub_score = rospy.Subscriber(
            "/matching_score", Float32, self._cb_matching_score, queue_size=5)
        self._sub_map_ctrl = rospy.Subscriber(
            "/app/map_status", String, self._cb_map_control, queue_size=5)
        self._sub_move_base = rospy.Subscriber(
            "/move_base/status", actionlib_msgs.msg.GoalStatusArray,
            self._cb_move_base_status, queue_size=5)
        self._sub_cmd_vel = rospy.Subscriber(
            "/NavOA_cmd_vel", Twist, self._cb_cmd_vel, queue_size=5)
        self._sub_map = rospy.Subscriber(
            self._map_topic, OccupancyGrid, self._cb_map, queue_size=1)

        # ------------------------------------------------------------------
        #  TF listener (for boundary-monitor detection)
        # ------------------------------------------------------------------
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        # ------------------------------------------------------------------
        #  Service proxies
        # ------------------------------------------------------------------
        self._traj_srv_name = "/get_trajectory_states"

        rospy.loginfo(
            "StatusStateNode initialised — "
            "rate=%.1f Hz, score_ok=%.2f, score_warn=%.2f",
            self._rate_hz, self._score_ok, self._score_warn)

    # ------------------------------------------------------------------
    #  Initial-value resolution (same env/config/param chain)
    # ------------------------------------------------------------------
    @staticmethod
    def _resolve_initial(param_name, default, valid_set):
        ros_val = rospy.get_param("~" + param_name.lstrip("_"), None)
        if ros_val is not None:
            ros_val = str(ros_val)
            if ros_val in valid_set:
                return ros_val
            rospy.logwarn("ROS param %s invalid '%s'; falling back",
                          param_name, ros_val)

        env_key = "STATUS_STATE_" + param_name.lstrip("_").upper()
        env_val = os.environ.get(env_key)
        if env_val is not None:
            env_val = str(env_val)
            if env_val in valid_set:
                return env_val
            rospy.logwarn("Env var %s invalid '%s'; falling back",
                          env_key, env_val)

        config_path = os.environ.get("STATUS_STATE_CONFIG")
        if config_path:
            try:
                with open(config_path, "r") as f:
                    config = json.load(f)
                cfg_key = param_name.lstrip("_")
                cfg_val = config.get(cfg_key)
                if cfg_val is not None:
                    cfg_val = str(cfg_val)
                    if cfg_val in valid_set:
                        return cfg_val
                    rospy.logwarn("Config key '%s' invalid '%s'; falling back",
                                  cfg_key, cfg_val)
            except (OSError, json.JSONDecodeError) as exc:
                rospy.logwarn("Could not read config file %s: %s",
                              config_path, exc)
        return default

    # ------------------------------------------------------------------
    #  Callbacks
    # ------------------------------------------------------------------
    def _cb_matching_score(self, msg):
        self._matching_score = msg.data
        self._score_stamp = rospy.Time.now()

    def _cb_map_control(self, msg):
        self._map_control_status = msg.data

    def _cb_move_base_status(self, msg):
        """Track the most recent goal's status from move_base."""
        if msg.status_list:
            last = msg.status_list[-1]
            self._move_base_status = last.status
            self._move_base_text = last.text

    def _cb_cmd_vel(self, msg):
        """Track when we last received a non-zero velocity command."""
        vel = math.sqrt(msg.linear.x ** 2 + msg.linear.y ** 2 +
                        msg.angular.z ** 2)
        if vel > 0.001:
            self._last_cmd_stamp = rospy.Time.now()

    def _cb_map(self, msg):
        """Cache the latest /map OccupancyGrid and detect map switches."""
        new_info = MapInfo(msg)
        sig = new_info.signature()
        if self._prev_map_info_sig == sig:
            # Latched re-send — just refresh the cached object
            self._map_info = new_info
            return
        # Genuine map switch
        self._map_info = new_info
        self._prev_map_info_sig = sig
        self._settle_until = rospy.Time.now() + rospy.Duration(
            self._map_settle_time_s)
        self._prev_oob = None
        r = new_info.rect
        rospy.logwarn(
            "Map switch detected: %dx%d, res=%.3f, "
            "rect=[%.2f, %.2f, %.2f, %.2f]. "
            "Suppressing boundary alerts for %.1f s.",
            new_info.width, new_info.height, new_info.resolution,
            r[0], r[1], r[2], r[3],
            self._map_settle_time_s)

    # ------------------------------------------------------------------
    #  Status derivation methods
    # ------------------------------------------------------------------
    def _derive_localization(self):
        """Derive /localization_status from Cartographer matching_score.

        Override via ``~_localization`` param for testing.
        """
        if self._loc_override is not None:
            return self._loc_override

        # --- Score timeout → LOST -------------------------------------------
        if self._score_stamp is not None:
            elapsed = (rospy.Time.now() - self._score_stamp).to_sec()
            if elapsed > self._score_timeout:
                rospy.logwarn_throttle(
                    10.0, "Matching-score timeout (%.1f > %.1f s)",
                    elapsed, self._score_timeout)
                return "LOST"

        # --- Score-based ------------------------------------------------------
        if self._matching_score is not None:
            if self._matching_score >= self._score_ok:
                return "LOCALIZED HIGH"
            elif self._matching_score >= self._score_warn:
                return "LOCALIZED LOW"
            else:
                return "LOST"

        # --- No data yet → LOCALIZED LOW (transitional) ----------------------
        return "LOCALIZED LOW"

    def _derive_mapping(self):
        """Derive /Mapping_status from trajectory states + map_control.

        - Any ACTIVE trajectory (livox_2d.launch)         → "MAPPING ON"
        - All trajectories FINISHED/FROZEN + matching_score → "LOCALIZATION ON"
        - Sensor data arriving but no trajectory yet        → "INITIALIZING"
        - No data at all                                    → "LOST"

        Override via ``~_mapping`` param.
        """
        if self._map_override is not None:
            return self._map_override

        # --- Check map_control status ----------------------------------------
        if self._map_control_status is not None:
            ctrl = self._map_control_status.lower()
            if "mapping started" in ctrl or "start" in ctrl:
                return "MAPPING ON"
            if "stopped" in ctrl or "finish" in ctrl or "saved" in ctrl:
                pass  # fall through to trajectory check

        # --- Check Cartographer trajectory states -----------------------------
        states = self._call_trajectory_states()
        if states is not None:
            # Any ACTIVE trajectory → mapping mode
            if any(s == TRAJ_ACTIVE for s in states):
                return "MAPPING ON"

            # All trajectories FINISHED/FROZEN + matching score → localization mode
            valid = [s for s in states if s != TRAJ_DELETED]
            if valid:
                if self._matching_score is not None:
                    return "LOCALIZATION ON"
                else:
                    # Trajectories exist but no sensor data yet
                    return "INITIALIZING"

        # --- No trajectory data → depends on matching_score -------------------
        if self._matching_score is not None:
            return "INITIALIZING"

        return "LOST"

    def _derive_navigation(self):
        """Derive /Nav_status from move_base status + cmd_vel activity.

        Override via ``~_nav`` param.
        """
        if self._nav_override is not None:
            return self._nav_override

        # --- move_base GoalStatus takes priority ------------------------------
        if self._move_base_status is not None:
            st = self._move_base_status
            if st == GB_ACTIVE:
                return "NAVIGATING"
            elif st == GB_SUCCEEDED:
                return "ARRIVED"
            elif st == GB_ABORTED:
                return "FAILED"
            elif st == GB_PREEMPTED:
                return "CANCELLED"

        # --- Check cmd_vel recency -------------------------------------------
        if self._last_cmd_stamp is not None:
            elapsed = (rospy.Time.now() - self._last_cmd_stamp).to_sec()
            if elapsed <= self._nav_idle_timeout:
                # We got a command recently — must be navigating
                return "NAVIGATING"

        return "IDLE"

    # ------------------------------------------------------------------
    #  Boundary monitor derivation
    # ------------------------------------------------------------------
    def _derive_out_of_bounds(self):
        """Derive /monitor_map_boundary from /map OccupancyGrid + TF.

        Returns ``"INSIDE_MAP"`` | ``"NEAR_BOUNDARY"`` | ``"OUT_OF_MAP"``,
        or ``None`` when no map has arrived yet, a settle window is active,
        or the TF lookup fails.
        """
        # --- No map yet or settle window active ---
        if self._map_info is None:
            return None
        if self._settle_until is not None:
            if rospy.Time.now() < self._settle_until:
                return None
            self._settle_until = None

        # --- TF lookup ---
        try:
            trans = self._tf_buffer.lookup_transform(
                self._map_frame, self._robot_frame,
                rospy.Time(0), rospy.Duration(0.5))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            rospy.logdebug_throttle(
                5.0, "TF lookup failed for boundary check: %s", exc)
            return None

        x = trans.transform.translation.x
        y = trans.transform.translation.y

        # --- Choose bounding box ---
        info = self._map_info
        bbox = info.tight if self._use_tight_bbox else info.rect
        xmin, ymin, xmax, ymax = bbox

        # --- Inside rectangle? ---
        inside_rect = (xmin <= x <= xmax and ymin <= y <= ymax)

        # --- Distance to nearest edge ---
        dists = [x - xmin, xmax - x, y - ymin, ymax - y]
        d_edge = min(dists) if inside_rect else max(-min(dists), 0.0)

        # --- Cell value checks ---
        val = info.cell_value(x, y)
        on_unknown = (val is None) or (val == -1 and self._treat_unknown_as_oob)
        on_wall = (val is not None and val >= 0
                   and val >= self._oob_wall_threshold)

        # --- Decide state with hysteresis ---
        if not inside_rect or on_unknown or on_wall:
            rospy.logwarn_throttle(
                2.0,
                "OUT_OF_MAP: (x=%.2f, y=%.2f) inside_rect=%s, val=%s, "
                "bbox=[%.2f, %.2f, %.2f, %.2f]",
                x, y, inside_rect, val,
                xmin, ymin, xmax, ymax)
            return "OUT_OF_MAP"

        warn_d = self._warn_distance_m
        if d_edge < warn_d:
            rospy.logwarn_throttle(
                2.0,
                "NEAR_BOUNDARY: (x=%.2f, y=%.2f) %.2fm from edge "
                "(threshold %.2fm)",
                x, y, d_edge, warn_d)
            return "NEAR_BOUNDARY"

        prev = self._prev_oob
        if prev == "NEAR_BOUNDARY" and d_edge < warn_d + self._exit_hysteresis_m:
            return "NEAR_BOUNDARY"

        return "INSIDE_MAP"

    # ------------------------------------------------------------------
    #  Service helpers
    # ------------------------------------------------------------------
    def _call_trajectory_states(self):
        """Call /get_trajectory_states and return list of trajectory states,
        or None on failure (unavailable / timeout / error)."""
        if not _HAS_CARTO_SRV:
            if self._traj_srv_available is not False:
                rospy.loginfo("cartographer_ros_msgs not installed — "
                              "trajectory state query disabled.")
                self._traj_srv_available = False
            return None
        try:
            rospy.wait_for_service(self._traj_srv_name, timeout=0.5)
            srv = rospy.ServiceProxy(self._traj_srv_name, GetTrajectoryStates)
            resp = srv()
            return list(resp.trajectory_states.trajectory_state)
        except rospy.ROSException:
            # Timeout — service not available (pure-localisation mode, etc.)
            if self._traj_srv_available is not False:
                rospy.loginfo("Trajectory states service not available "
                              "(expected in pure-localization mode).")
                self._traj_srv_available = False
            return None
        except rospy.ServiceException as exc:
            rospy.logerr_throttle(30.0, "Failed to call %s: %s",
                                  self._traj_srv_name, exc)
            return None

    # ------------------------------------------------------------------
    #  Publication helper
    # ------------------------------------------------------------------
    def _publish_with_change_log(self, pub, topic, new_val, prev_val):
        msg = String(data=new_val)
        pub.publish(msg)
        if prev_val is None:
            rospy.loginfo("[%s] initial status → %s", topic, new_val)
        elif new_val != prev_val:
            rospy.loginfo("[%s] status changed: %s → %s",
                          topic, prev_val, new_val)

    # ------------------------------------------------------------------
    #  Main loop
    # ------------------------------------------------------------------
    def run(self):
        while not rospy.is_shutdown():
            try:
                # --- Localization ---
                loc = self._derive_localization()
                self._publish_with_change_log(
                    self._pub_loc, "/localization_status",
                    loc, self._prev_loc)
                self._prev_loc = loc

                # --- Mapping ---
                mapping = self._derive_mapping()
                self._publish_with_change_log(
                    self._pub_map, "/Mapping_status",
                    mapping, self._prev_map)
                self._prev_map = mapping

                # --- Navigation ---
                nav = self._derive_navigation()
                self._publish_with_change_log(
                    self._pub_nav, "/Nav_status",
                    nav, self._prev_nav)
                self._prev_nav = nav

                # --- Out of bounds ---
                oob = self._derive_out_of_bounds()
                if oob is not None:
                    self._publish_with_change_log(
                        self._pub_oob, "/monitor_map_boundary",
                        oob, self._prev_oob)
                    self._prev_oob = oob

            except rospy.ROSException:
                break
            except Exception as exc:
                rospy.logerr("Unexpected error in status loop: %s", exc)

            self._rate.sleep()

        rospy.loginfo("StatusStateNode shutting down.")


def main():
    node = StatusStateNode()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
