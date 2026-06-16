#!/usr/bin/env python3
"""
status_state.py — ROS1 node that publishes localization, mapping, and navigation
status on three topics by deriving the values from real ROS topics and services.

Topic derivation logic (overridable via private params for testing):

  /localization_status  ->  /matching_score (Cartographer) + timeout detection.
                             Values: "LOCALIZED HIGH" | "LOCALIZED LOW" | "LOST"

  /Mapping_status       ->  /get_trajectory_states service (Cartographer) +
                             /app/map_status (map_control).
                             Values: "MAPPING ON" | "LOCALIZATION ON" | "LOST" | "INITIALIZING"

  /Nav_status           ->  /move_base/status (GoalStatusArray) + cmd_vel activity.
                             Values: "IDLE" | "NAVIGATING" | "ARRIVED" | "FAILED" | "CANCELLED"

Private parameters (~):
  _localization  (str, default: auto) — override to force a value
  _mapping       (str, default: auto) — override to force a value
  _nav           (str, default: auto) — override to force a value
  _rate          (float, default: 1.0)     — publish rate in Hz
"""

import os
import json
import math

import rospy
import actionlib_msgs.msg
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist, PoseStamped

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

        # Previous published values (for change detection)
        self._prev_loc = None
        self._prev_map = None
        self._prev_nav = None

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
