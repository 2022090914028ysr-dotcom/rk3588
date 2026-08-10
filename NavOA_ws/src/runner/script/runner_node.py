#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
runner_node.py — System lifecycle manager for the SlamNav workspace.

Central integration point for FastAPI middleware via rosbridge_server.
Controls cartographer trajectory switching, system modes, and publishes
detailed JSON status for easy API consumption.

ARCHITECTURE (manager mode, full.launch mode:=manager):

  Top-level roslaunch (ALWAYS ON, never killed):
    sensors.launch      — lidar, camera, ultrasonic, serial bridge, rf2o, IMU mux,
                          NavOA navigation (move_base + controllers)
    cartographer_node   — SLAM node (trajectory services for mode switching)
    cartographer_occupancy_grid_node

  ROS nodes:
    runner_status_state — enriched status from real topics
    runner_manager      — this node

  Managed subprocess (started/stopped only as needed):
    MARL exploration    — roslaunch marl_mapping exploration.launch

  KEY INVARIANT: sensors, cartographer_node, and NavOA are ALWAYS running.
  Mode switching uses Cartographer trajectory services ONLY
  (finish_trajectory / start_trajectory / write_state).
  No processes are killed or relaunched for any core component.

Commands (publish to /runner/command as std_msgs/String):
  "mode:mapping"       — Start mapping trajectory
  "mode:localization"  — Save map, finish mapping, start localization
  "mode:auto_mapping"  — Mapping trajectory + MARL exploration
  "mode:sensors"       — Finish active trajectory (cartographer idle)
  "start_mapping"      — Alias for "mode:mapping"
  "stop_mapping"       — Alias for "mode:localization"
  "save_map [name]"    — Save current map with optional name
  "list_maps"          — List saved .pbstream files in JSON status
  "status"             — Publish current system status
  "shutdown"           — Gracefully shut down the system

Status (published on /runner/status as JSON std_msgs/String):
  {
    "mode": "MAPPING|LOCALIZATION|SENSORS_ONLY|...",
    "timestamp": 1234567890.123,
    "trajectory": {"active_id": 0, "all_states": [1, 0, 3]},
    "loc_status": "LOCALIZED HIGH|LOCALIZED LOW|LOST",
    "loc_score": 0.85,
    "map_status": "MAPPING ON|LOCALIZATION ON|LOST|INITIALIZING",
    "nav_status": "IDLE|NAVIGATING|ARRIVED|FAILED|CANCELLED",
    "oob_status": "INSIDE_MAP|NEAR_BOUNDARY|OUT_OF_MAP",
    "maps": ["map_123.pbstream", "map_456.pbstream"],
    "cartographer_alive": true,
    "error": ""
  }

Also publishes /runner/mode (simple string) for backward compatibility.
"""

import json
import math
import os
import signal
import subprocess
import threading
import rospy
import rospkg
import tf2_ros
from geometry_msgs.msg import Pose
from std_msgs.msg import String, Float32

# Cartographer service imports — gracefully degrade if unavailable
try:
    from cartographer_ros_msgs.srv import (
        FinishTrajectory, FinishTrajectoryRequest,
        StartTrajectory, StartTrajectoryRequest,
        WriteState, WriteStateRequest,
        GetTrajectoryStates,
    )
    _HAS_CARTO = True
except ImportError:
    _HAS_CARTO = False


class RunnerNode:
    # ── Mode constants ──────────────────────────────────────────────────
    MODE_STARTUP = "STARTUP"
    MODE_SENSORS = "SENSORS_ONLY"
    MODE_MAPPING = "MAPPING"
    MODE_LOCALIZATION = "LOCALIZATION"
    MODE_AUTO_MAPPING = "AUTO_MAPPING"
    MODE_SWITCHING = "SWITCHING"
    MODE_SHUTDOWN = "SHUTTING_DOWN"
    MODE_ERROR = "ERROR"

    # ── TrajectoryState constants ───────────────────────────────────────
    TRAJ_ACTIVE = 0
    TRAJ_FINISHED = 1
    TRAJ_FROZEN = 2
    TRAJ_DELETED = 3

    def __init__(self):
        rospy.on_shutdown(self._on_shutdown)

        # ── Parameters ──────────────────────────────────────────────────
        self._config_dir = rospy.get_param(
            "~config_dir",
            os.path.join(rospkg.RosPack().get_path('cartographer_ros'), 'configuration_files'))
        self._config_mapping = rospy.get_param(
            "~config_mapping", "livox_2d.lua")
        self._config_localization = rospy.get_param(
            "~config_localization", "livox_2d_localization.lua")
        self._map_save_dir = rospy.get_param(
            "~map_save_dir",
            os.path.normpath(
                os.path.join(rospkg.RosPack().get_path('cartographer_ros'),
                             os.pardir, os.pardir, os.pardir, 'map')))
        self._map_frame = rospy.get_param("~map_frame", "map")
        self._base_frame = rospy.get_param("~base_frame", "base_link")
        self._tf_timeout = rospy.get_param("~tf_timeout", 1.0)

        # ── Dynamic system state ────────────────────────────────────────
        self._mode = self.MODE_STARTUP
        self._lock = threading.Lock()
        self._shutdown_requested = False

        # ── Status tracking ─────────────────────────────────────────────
        self._matching_score = None
        self._loc_status = None      # from /localization_status
        self._map_status = None      # from /Mapping_status
        self._nav_status = None      # from /Nav_status
        self._oob_status = None      # from /monitor_map_boundary

        # ── Subprocess handles ──────────────────────────────────────────
        # NOTE: sensors, cartographer_node, and NavOA all run at the top
        # level of full.launch mode:=manager and are NEVER managed as
        # subprocesses. Only MARL exploration is started/killed on demand.
        self._explore_process = None

        # ── TF (for pose caching during trajectory switches) ────────────
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        # ── Publishers ──────────────────────────────────────────────────
        self._status_pub = rospy.Publisher(
            "/runner/status", String, queue_size=1, latch=True)
        self._mode_pub = rospy.Publisher(
            "/runner/mode", String, queue_size=1, latch=True)

        # ── Subscribers ─────────────────────────────────────────────────
        rospy.Subscriber(
            "/runner/command", String, self._on_command)

        # Listen to status_state outputs for enriched JSON status
        rospy.Subscriber(
            "/localization_status", String, self._cb_loc_status)
        rospy.Subscriber(
            "/Mapping_status", String, self._cb_map_status)
        rospy.Subscriber(
            "/Nav_status", String, self._cb_nav_status)
        rospy.Subscriber(
            "/monitor_map_boundary", String, self._cb_oob_status)
        rospy.Subscriber(
            "/matching_score", Float32, self._cb_score)

        # ── Initialise ──────────────────────────────────────────────────
        self._set_mode(self.MODE_SENSORS, publish=True)
        rospy.loginfo(
            "RunnerNode initialised — listening on /runner/command. "
            "Cartographer API: %s", "available" if _HAS_CARTO else "UNAVAILABLE")
        rospy.loginfo(
            "RunnerNode: sensors + cartographer_node run at top level — "
            "they are NEVER managed as subprocesses. "
            "Only MARL exploration uses subprocess lifecycle.")

    # ═══════════════════════════════════════════════════════════════════
    #  Command handler
    # ═══════════════════════════════════════════════════════════════════

    def _on_command(self, msg):
        cmd = msg.data.strip()
        rospy.loginfo("[Runner] command: '%s'", cmd)

        with self._lock:
            cmd_lower = cmd.lower()

            if cmd_lower == "mode:mapping" or cmd_lower == "start_mapping":
                self._switch_to_mapping()

            elif cmd_lower == "mode:localization" or cmd_lower == "stop_mapping":
                self._switch_to_localization()

            elif cmd_lower == "mode:sensors":
                self._switch_to_sensors()

            elif cmd_lower == "mode:auto_mapping":
                self._switch_to_auto_mapping()

            elif cmd_lower.startswith("save_map") or cmd_lower.startswith("save map"):
                parts = cmd.split(None, 1)
                name = parts[1].strip() if len(parts) > 1 else None
                self._save_map(name)

            elif cmd_lower == "list_maps" or cmd_lower == "list maps":
                self._list_maps()

            elif cmd_lower == "status":
                self._publish_status()

            elif cmd_lower == "shutdown":
                rospy.logwarn("[Runner] SHUTDOWN requested — finishing traj, "
                             "killing NavOA + MARL. "
                             "Node stays alive — send mode:mapping to resume.")
                self._shutdown_requested = True
                self._shutdown_cleanup()
                self._set_mode(self.MODE_SHUTDOWN)
                rospy.loginfo(
                    "[Runner] shutdown complete — node still running. "
                    "Send 'mode:mapping' or 'mode:localization' to resume.")

            else:
                rospy.logwarn("[Runner] unknown command: '%s'", cmd)

    # ═══════════════════════════════════════════════════════════════════
    #  Mode switching — trajectory services only, no process mgmt
    # ═══════════════════════════════════════════════════════════════════

    def _switch_to_mapping(self):
        """Switch to mapping mode via trajectory services.

        Precondition: cartographer_node is running (top-level roslaunch).
        Sensors remain ON throughout.
        """
        self._set_mode(self.MODE_SWITCHING)

        if not self._is_carto_alive():
            rospy.logerr(
                "[Runner] cartographer_node is NOT running — "
                "cannot switch to mapping. "
                "Start with: roslaunch runner full.launch mode:=manager")
            self._set_mode(self.MODE_ERROR)
            return

        active_id = self._get_active_trajectory_id()

        if active_id is not None:
            # Finish current active trajectory, start mapping trajectory.
            ref_id = self._get_reference_trajectory_id()
            pose = self._get_pose()

            ok = self._call_finish(active_id)
            if not ok:
                self._set_mode(self.MODE_ERROR)
                return

            new_id = self._call_start(
                config=self._config_mapping,
                pose=pose,
                relative_to=ref_id,
            )
            if new_id is None:
                self._set_mode(self.MODE_ERROR)
                return

            rospy.loginfo(
                "[Runner] mapping trajectory %d started", new_id)
        else:
            # No active trajectory — start a fresh one.
            ref_id = self._get_reference_trajectory_id()
            new_id = self._call_start(
                config=self._config_mapping,
                pose=None,
                relative_to=ref_id,
            )
            if new_id is None:
                self._set_mode(self.MODE_ERROR)
                return

            rospy.loginfo(
                "[Runner] mapping trajectory %d started (no prior traj)",
                new_id)

        # Kill MARL if it was running.
        self._kill_exploration()

        # Log: sensors always stay on.
        rospy.loginfo("[Runner] sensors remain ON — mode switch: MAPPING")
        self._set_mode(self.MODE_MAPPING)

    def _switch_to_localization(self):
        """Save map, finish mapping, start localisation.

        Precondition: cartographer_node is running (top-level roslaunch).
        Sensors remain ON throughout.
        """
        self._set_mode(self.MODE_SWITCHING)

        if not self._is_carto_alive():
            rospy.logerr(
                "[Runner] cartographer_node is NOT running — "
                "cannot switch to localization. "
                "Start with: roslaunch runner full.launch mode:=manager")
            self._set_mode(self.MODE_ERROR)
            return

        # 1. Cache current robot pose.
        pose = self._get_pose()
        if pose is None:
            rospy.logerr(
                "[Runner] cannot get TF %s->%s — aborting switch",
                self._map_frame, self._base_frame)
            self._set_mode(self.MODE_ERROR)
            return

        # 2. Save map to disk.
        timestamp = rospy.Time.now().to_sec()
        map_path = os.path.join(
            self._map_save_dir,
            "map_{:.0f}.pbstream".format(timestamp))
        ok = self._call_write_state(map_path)
        if ok:
            rospy.logwarn("[Runner] map saved: %s", map_path)
        else:
            rospy.logwarn(
                "[Runner] map save failed — continuing switch")

        # 3. Find stable reference trajectory before finishing active one.
        ref_id = self._get_reference_trajectory_id()

        # 4. Finish the active (mapping) trajectory.
        active_id = self._get_active_trajectory_id()
        if active_id is None:
            rospy.logerr(
                "[Runner] no active trajectory to finish — "
                "starting fresh localization traj")
            active_id = ref_id if ref_id != 0 else 0

        ok = self._call_finish(active_id)
        if not ok:
            self._set_mode(self.MODE_ERROR)
            return

        # 5. Start localization trajectory with cached pose.
        new_id = self._call_start(
            config=self._config_localization,
            pose=pose,
            relative_to=ref_id,
        )
        if new_id is None:
            self._set_mode(self.MODE_ERROR)
            return

        rospy.loginfo(
            "[Runner] localization trajectory %d started", new_id)

        # 6. Kill MARL if running.
        self._kill_exploration()

        rospy.loginfo("[Runner] sensors + NavOA remain ON — mode switch: LOCALIZATION")
        self._set_mode(self.MODE_LOCALIZATION)

    def _switch_to_sensors(self):
        """Sensors-only: finish active trajectory, kill MARL.

        cartographer_node stays running (no active trajectory).
        Sensors + cartographer_node are NEVER killed.
        """
        self._set_mode(self.MODE_SWITCHING)

        # Finish any active trajectory (cartographer_node stays alive).
        active_id = self._get_active_trajectory_id()
        if active_id is not None:
            rospy.loginfo(
                "[Runner] finishing trajectory %d (cartographer stays ON)",
                active_id)
            self._call_finish(active_id)

        # Kill MARL if running.
        self._kill_exploration()

        rospy.loginfo(
            "[Runner] sensors + cartographer remain ON (no active traj) — "
            "mode switch: SENSORS_ONLY")
        self._set_mode(self.MODE_SENSORS)

    def _switch_to_auto_mapping(self):
        """Mapping + MARL exploration."""
        self._set_mode(self.MODE_SWITCHING)

        # Ensure we're in mapping mode first (via trajectory services).
        self._switch_to_mapping_base()

        # Launch MARL exploration.
        self._launch_exploration()

        rospy.loginfo(
            "[Runner] sensors + NavOA remain ON — mode switch: AUTO_MAPPING")
        self._set_mode(self.MODE_AUTO_MAPPING)

    def _switch_to_mapping_base(self):
        """Internal: ensure a mapping trajectory is active."""
        if not self._is_carto_alive():
            rospy.logerr(
                "[Runner] cartographer_node not running — "
                "cannot start mapping")
            return

        active_id = self._get_active_trajectory_id()
        if active_id is not None:
            ref_id = self._get_reference_trajectory_id()
            pose = self._get_pose()
            self._call_finish(active_id)
            self._call_start(
                config=self._config_mapping,
                pose=pose,
                relative_to=ref_id,
            )

    # ═══════════════════════════════════════════════════════════════════
    #  Cartographer trajectory operations
    # ═══════════════════════════════════════════════════════════════════

    def _get_carto_service(self, srv_type, srv_name):
        """Get a Cartographer service proxy."""
        if not _HAS_CARTO:
            return None
        try:
            rospy.wait_for_service(srv_name, timeout=0.5)
            return rospy.ServiceProxy(srv_name, srv_type)
        except rospy.ROSException:
            return None

    def _is_carto_alive(self):
        """Check if cartographer_node is running by probing its services."""
        return self._get_carto_service(
            GetTrajectoryStates, "/get_trajectory_states") is not None

    def _get_active_trajectory_id(self):
        """Return the latest ACTIVE trajectory ID, or None."""
        srv = self._get_carto_service(
            GetTrajectoryStates, "/get_trajectory_states")
        if srv is None:
            return None
        try:
            resp = srv()
            states = resp.trajectory_states
            for i in range(len(states.trajectory_id) - 1, -1, -1):
                if states.trajectory_state[i] == self.TRAJ_ACTIVE:
                    return states.trajectory_id[i]
            return None
        except Exception as e:
            rospy.logwarn("[Runner] GetTrajectoryStates failed: %s", e)
            return None

    def _get_reference_trajectory_id(self):
        """Find the first FINISHED trajectory as a stable reference frame.

        The first mapping trajectory (ID 0, no trimmer) stays FINISHED
        and never gets auto-deleted. Falls back to 0.
        """
        srv = self._get_carto_service(
            GetTrajectoryStates, "/get_trajectory_states")
        if srv is None:
            return 0
        try:
            resp = srv()
            states = resp.trajectory_states
            for i in range(len(states.trajectory_id)):
                if states.trajectory_state[i] == self.TRAJ_FINISHED:
                    return states.trajectory_id[i]
            return 0
        except Exception:
            return 0

    def _call_finish(self, traj_id):
        """Finish a trajectory. Returns True on success."""
        srv = self._get_carto_service(
            FinishTrajectory, "/finish_trajectory")
        if srv is None:
            return False
        try:
            req = FinishTrajectoryRequest()
            req.trajectory_id = traj_id
            resp = srv(req)
            if resp.status.code == 0:
                rospy.loginfo("[Runner] finished trajectory %d", traj_id)
                return True
            else:
                rospy.logerr("[Runner] finish traj %d failed: %s",
                             traj_id, resp.status.message)
                return False
        except Exception as e:
            rospy.logerr("[Runner] FinishTrajectory exception: %s", e)
            return False

    def _call_start(self, config, pose, relative_to):
        """Start a new trajectory. Returns trajectory ID or None."""
        srv = self._get_carto_service(
            StartTrajectory, "/start_trajectory")
        if srv is None:
            return None
        try:
            req = StartTrajectoryRequest()
            req.configuration_directory = self._config_dir
            req.configuration_basename = config
            req.relative_to_trajectory_id = relative_to

            if pose is not None:
                req.use_initial_pose = True
                req.initial_pose = pose
            else:
                req.use_initial_pose = False

            resp = srv(req)
            if resp.status.code == 0:
                rospy.loginfo("[Runner] started trajectory %d with '%s'",
                              resp.trajectory_id, config)
                return resp.trajectory_id
            else:
                rospy.logerr("[Runner] start trajectory failed: %s",
                             resp.status.message)
                return None
        except Exception as e:
            rospy.logerr("[Runner] StartTrajectory exception: %s", e)
            return None

    def _call_write_state(self, filename):
        """Save map state to .pbstream. Returns True on success."""
        srv = self._get_carto_service(WriteState, "/write_state")
        if srv is None:
            return False
        try:
            req = WriteStateRequest()
            req.filename = filename
            req.include_unfinished_submaps = True
            resp = srv(req)
            if resp.status.code == 0:
                rospy.loginfo("[Runner] state saved: %s", filename)
                return True
            else:
                rospy.logwarn("[Runner] WriteState: %s", resp.status.message)
                return False
        except Exception as e:
            rospy.logwarn("[Runner] WriteState exception: %s", e)
            return False

    # ═══════════════════════════════════════════════════════════════════
    #  Subprocess management (MARL exploration only)
    # ═══════════════════════════════════════════════════════════════════
    #
    #  IMPORTANT: This is the ONLY place where subprocesses are managed.
    #  sensors.launch, cartographer_node, and NavOA run at the top level
    #  of full.launch (mode:=manager) and are NEVER launched/killed here.

    def _launch_exploration(self):
        """Launch MARL exploration node as a subprocess."""
        if self._explore_process is not None:
            ret = self._explore_process.poll()
            if ret is None:
                return
        try:
            self._explore_process = subprocess.Popen(
                ["roslaunch", "marl_mapping", "exploration.launch"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid,
            )
            rospy.loginfo("[Runner] started MARL exploration (PID %d)",
                          self._explore_process.pid)
        except Exception as e:
            rospy.logerr("[Runner] failed to start exploration: %s", e)

    def _kill_exploration(self):
        """Kill MARL exploration node subprocess."""
        if self._explore_process is not None:
            try:
                pgid = os.getpgid(self._explore_process.pid)
                os.killpg(pgid, signal.SIGTERM)
                self._explore_process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(pgid, signal.SIGKILL)
                    self._explore_process.wait()
                except Exception:
                    pass
            except Exception:
                pass
            self._explore_process = None
            rospy.loginfo("[Runner] stopped MARL exploration")

    # ═══════════════════════════════════════════════════════════════════
    #  Status callbacks (from status_state node)
    # ═══════════════════════════════════════════════════════════════════

    def _cb_loc_status(self, msg):
        self._loc_status = msg.data

    def _cb_map_status(self, msg):
        self._map_status = msg.data

    def _cb_nav_status(self, msg):
        self._nav_status = msg.data

    def _cb_oob_status(self, msg):
        self._oob_status = msg.data

    def _cb_score(self, msg):
        self._matching_score = msg.data

    # ═══════════════════════════════════════════════════════════════════
    #  TF helpers
    # ═══════════════════════════════════════════════════════════════════

    def _get_pose(self):
        """Look up map→base_link and return as a Pose, or None."""
        try:
            tf_stamped = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame,
                rospy.Time(0), rospy.Duration(self._tf_timeout))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("[Runner] TF %s→%s lookup failed: %s",
                          self._map_frame, self._base_frame, e)
            return None

        pose = Pose()
        pose.position.x = tf_stamped.transform.translation.x
        pose.position.y = tf_stamped.transform.translation.y
        pose.position.z = tf_stamped.transform.translation.z
        pose.orientation = tf_stamped.transform.rotation
        return pose

    # ═══════════════════════════════════════════════════════════════════
    #  Map management
    # ═══════════════════════════════════════════════════════════════════

    def _save_map(self, name=None):
        """Save the current map. Uses name or auto-generates a timestamped name."""
        if name:
            filename = name
            if not filename.endswith(".pbstream"):
                filename += ".pbstream"
            map_path = os.path.join(self._map_save_dir, filename)
        else:
            timestamp = rospy.Time.now().to_sec()
            map_path = os.path.join(
                self._map_save_dir,
                "map_{:.0f}.pbstream".format(timestamp))

        ok = self._call_write_state(map_path)
        if ok:
            rospy.logwarn("[Runner] map saved: %s", map_path)
        else:
            rospy.logerr("[Runner] failed to save map: %s", map_path)
        self._publish_status()

    def _list_maps(self):
        """List saved .pbstream files in the map save directory."""
        self._publish_status()

    def _get_map_list(self):
        """Return list of .pbstream filenames in the map directory."""
        try:
            if not os.path.isdir(self._map_save_dir):
                return []
            files = sorted(
                f for f in os.listdir(self._map_save_dir)
                if f.endswith(".pbstream"))
            return files
        except Exception as e:
            rospy.logwarn("[Runner] error listing maps: %s", e)
            return []

    # ═══════════════════════════════════════════════════════════════════
    #  Status publishing
    # ═══════════════════════════════════════════════════════════════════

    def _set_mode(self, mode, publish=True):
        """Set internal mode and publish status."""
        self._mode = mode
        self._mode_pub.publish(String(data=mode))
        if publish:
            self._publish_status()

    def _get_trajectory_info(self):
        """Return (active_id, state_map) from cartographer, or (None, {})."""
        srv = self._get_carto_service(
            GetTrajectoryStates, "/get_trajectory_states")
        if srv is None:
            return None, {}
        try:
            resp = srv()
            states = resp.trajectory_states
            active_id = None
            state_map = {}
            for i in range(len(states.trajectory_id)):
                tid = states.trajectory_id[i]
                ts = states.trajectory_state[i]
                state_map[tid] = ts
                if ts == self.TRAJ_ACTIVE:
                    active_id = tid
            return active_id, state_map
        except Exception:
            return None, {}

    def _publish_status(self):
        """Publish full JSON status on /runner/status."""
        active_id, state_map = self._get_trajectory_info()
        traj_states = list(state_map.values())

        status = {
            "mode": self._mode,
            "timestamp": rospy.Time.now().to_sec(),
            "trajectory": {
                "active_id": active_id,
                "all_states": traj_states,
            },
            "loc_status": self._loc_status or "UNKNOWN",
            "loc_score": self._matching_score if self._matching_score is not None else None,
            "map_status": self._map_status or "UNKNOWN",
            "nav_status": self._nav_status or "UNKNOWN",
            "oob_status": self._oob_status or "UNKNOWN",
            "maps": self._get_map_list(),
            "cartographer_alive": self._is_carto_alive(),
            "sensors_always_on": True,
            "error": "",
        }

        if self._mode == self.MODE_ERROR:
            status["error"] = "Mode switch failed — see /runner/log for details"

        json_str = json.dumps(status)
        self._status_pub.publish(String(data=json_str))
        rospy.loginfo("[Runner] status: mode=%s, trajs=%s, loc=%s, "
                      "map=%s, nav=%s, oob=%s",
                      self._mode, traj_states,
                      self._loc_status, self._map_status,
                      self._nav_status, self._oob_status)

    # ═══════════════════════════════════════════════════════════════════
    #  Shutdown
    # ═══════════════════════════════════════════════════════════════════

    def _kill_navoa_pkill(self):
        """Kill NavOA-specific nodes via pkill.

        NavOA runs inside sensors.launch (top-level roslaunch) so we
        can't kill the launch parent without taking down sensors too.
        Instead we target the specific NavOA binary names.
        """
        for pattern in ["move_base", "map_server", "amcl"]:
            try:
                subprocess.run(
                    ["pkill", "-f", pattern],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL)
                rospy.loginfo("[Runner] pkill'd '%s'", pattern)
            except Exception:
                pass

    def _shutdown_cleanup(self):
        """Full cleanup on shutdown: finish cartographer traj, kill NavOA + MARL."""
        # 1. Finish active cartographer trajectory.
        if self._is_carto_alive():
            active_id = self._get_active_trajectory_id()
            if active_id is not None:
                rospy.loginfo(
                    "[Runner] finishing trajectory %d", active_id)
                self._call_finish(active_id)

        # 2. Kill NavOA nodes (move_base, map_server, amcl).
        self._kill_navoa_pkill()

        # 3. Kill MARL subprocess.
        self._kill_exploration()

        rospy.logwarn(
            "[Runner] shutdown cleanup complete — cartographer idle, "
            "NavOA killed, MARL killed. "
            "Sensors + cartographer_node stay managed by roslaunch.")

    def _on_shutdown(self):
        """Cleanup on node shutdown (also called by rospy shutdown hook)."""
        self._shutdown_cleanup()


if __name__ == "__main__":
    rospy.init_node("runner_node")
    node = RunnerNode()
    rospy.spin()
