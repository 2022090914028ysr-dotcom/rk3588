#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
mode_manager.py — Cartographer mode switch manager

Subscribes to /mode_command (std_msgs/String) for mode-switch commands
and publishes status feedback on /mode_status (std_msgs/String).

Commands:
  "start_mapping"   — Finish current traj (if any), start mapping trajectory
  "stop_mapping"    — Save map, finish mapping traj, start localization traj
                       with cached map→base_footprint pose
  "carto_start"     — Launch cartographer_node via roslaunch (configurable)
  "carto_stop"      — Kill cartographer_node
  "get_status"      — Publish current mode on /mode_status

Status values:
  IDLE, MAPPING, LOCALIZING, SWITCHING, ERROR

Integration with FastAPI middleware:
  FastAPI uses roslibpy/rosbridge to publish to /mode_command and
  subscribe to /mode_status.
"""

import math
import os
import signal
import subprocess
import threading
import rospy
import rospkg
import tf2_ros
from geometry_msgs.msg import Pose, TransformStamped
from std_msgs.msg import String
from cartographer_ros_msgs.srv import (
    FinishTrajectory, FinishTrajectoryRequest,
    StartTrajectory, StartTrajectoryRequest,
    WriteState, WriteStateRequest,
    GetTrajectoryStates,
)


class ModeManager:
    # ── Mode constants ──────────────────────────────────────────────────
    MODE_IDLE = "IDLE"
    MODE_MAPPING = "MAPPING"
    MODE_LOCALIZING = "LOCALIZING"
    MODE_SWITCHING = "SWITCHING"
    MODE_ERROR = "ERROR"

    # TrajectoryState: ACTIVE=0, FINISHED=1, FROZEN=2, DELETED=3
    _TRAJ_ACTIVE = 0
    _TRAJ_FINISHED = 1

    def __init__(self):
        rospy.on_shutdown(self._on_shutdown)

        # ── Parameters ──────────────────────────────────────────────────
        self.config_dir = rospy.get_param(
            "~config_dir",
            os.path.join(rospkg.RosPack().get_path('cartographer_ros'), 'configuration_files'))
        self.config_file_mapping = rospy.get_param(
            "~config_file_mapping", "livox_2d.lua")
        self.config_file_localization = rospy.get_param(
            "~config_file_localization", "livox_2d_localization.lua")
        self.map_save_dir = rospy.get_param(
            "~map_save_dir",
            os.path.join(
                os.path.dirname(rospkg.RosPack().get_path('cartographer_ros')),
                os.pardir, os.pardir, 'map'))
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")
        self.tf_timeout = rospy.get_param("~tf_timeout", 1.0)
        self.carto_launch_pkg = rospy.get_param(
            "~carto_launch_pkg", "cartographer_ros")
        self.carto_launch_file = rospy.get_param(
            "~carto_launch_file", "livox_2d.launch")

        # Service names
        self._finish_svc_name = rospy.get_param(
            "~finish_service", "/finish_trajectory")
        self._start_svc_name = rospy.get_param(
            "~start_service", "/start_trajectory")
        self._write_svc_name = rospy.get_param(
            "~write_service", "/write_state")
        self._states_svc_name = rospy.get_param(
            "~states_service", "/get_trajectory_states")

        # Topic names
        cmd_topic = rospy.get_param("~command_topic", "/mode_command")
        status_topic = rospy.get_param("~status_topic", "/mode_status")

        # ── Internal state ──────────────────────────────────────────────
        self._mode = self.MODE_IDLE
        self._lock = threading.Lock()
        self._carto_process = None

        # ── TF ──────────────────────────────────────────────────────────
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        # ── Publishers ──────────────────────────────────────────────────
        self._status_pub = rospy.Publisher(
            status_topic, String, queue_size=1, latch=True)
        self._publish_status(self.MODE_IDLE)

        # ── Subscribers ─────────────────────────────────────────────────
        self._cmd_sub = rospy.Subscriber(
            cmd_topic, String, self._on_command)

        # ── Service proxies (lazy: created on first use) ────────────────
        self._finish_srv = None
        self._start_srv = None
        self._write_srv = None
        self._states_srv = None

        rospy.loginfo(
            "ModeManager initialised — listening on '%s'", cmd_topic)
        self._publish_status(self.MODE_IDLE)

    # ───────────────────────── Public API ────────────────────────────────

    def _on_command(self, msg):
        """Callback for /mode_command — thread-safe via lock."""
        cmd = msg.data.strip()
        rospy.loginfo("ModeManager received command: '%s'", cmd)

        with self._lock:
            if cmd == "start_mapping":
                self._start_mapping()
            elif cmd == "stop_mapping":
                self._stop_mapping()
            elif cmd == "carto_start":
                self._carto_start()
            elif cmd == "carto_stop":
                self._carto_stop()
            elif cmd == "get_status":
                self._publish_status(self._mode)
            else:
                rospy.logwarn(
                    "ModeManager unknown command: '%s'", cmd)

    def _on_shutdown(self):
        """Cleanup on node shutdown."""
        with self._lock:
            if self._carto_process is not None:
                rospy.loginfo("Shutting down cartographer_node ...")
                self._carto_process.terminate()
                self._carto_process = None

    # ──────────────────── Command handlers ───────────────────────────────

    def _start_mapping(self):
        """Reverse of Scenario 1: finish localization, start mapping."""
        self._set_mode(self.MODE_SWITCHING)

        # Check if cartographer_node is running.
        if not self._is_carto_running():
            rospy.logwarn(
                "cartographer_node not running — launching now ...")
            self._carto_start()
            # _carto_start already set mode to MAPPING.
            return

        # Get current robot pose before switching.
        pose = self._get_pose()
        if pose is None:
            rospy.logwarn(
                "Could not get TF pose — starting new traj without"
                " initial_pose (global relocalisation)")
            use_initial_pose = False
        else:
            use_initial_pose = True

        # Find a stable reference trajectory before finishing the active one
        # (the active one may get DELETED by PureLocalizationTrimmer).
        ref_id = self._get_reference_trajectory_id()

        # Finish the current active trajectory.
        traj_id = self._get_active_trajectory_id()
        if traj_id is not None:
            ok = self._call_finish_trajectory(traj_id)
            if not ok:
                self._set_mode(self.MODE_ERROR)
                return
        else:
            rospy.loginfo(
                "No active trajectory to finish — starting fresh")

        # Start a mapping trajectory relative to the stable reference frame.
        new_id = self._call_start_trajectory(
            config_file=self.config_file_mapping,
            use_initial_pose=use_initial_pose,
            pose=pose,
            relative_to=ref_id,
        )
        if new_id is None:
            self._set_mode(self.MODE_ERROR)
            return

        rospy.logwarn(
            "Mapping trajectory %d started — building new map",
            new_id)
        self._set_mode(self.MODE_MAPPING)

    def _stop_mapping(self):
        """Scenario 1: save map, finish mapping, start localisation."""
        self._set_mode(self.MODE_SWITCHING)

        if not self._is_carto_running():
            rospy.logerr(
                "cartographer_node not running — cannot stop mapping")
            self._set_mode(self.MODE_ERROR)
            return

        # 1. Cache current robot pose.
        pose = self._get_pose()
        if pose is None:
            rospy.logerr(
                "Cannot get TF %s→%s — aborting switch",
                self.map_frame, self.base_frame)
            self._set_mode(self.MODE_ERROR)
            return

        # 2. Save map to disk.
        timestamp = rospy.Time.now().to_sec()
        map_path = os.path.join(
            self.map_save_dir,
            "map_{:.0f}.pbstream".format(timestamp))
        ok = self._call_write_state(map_path)
        if ok:
            rospy.logwarn("Map saved to %s", map_path)
        else:
            rospy.logwarn(
                "Map save failed or skipped — continuing switch")

        # 3. Find a stable reference trajectory (may differ from active one).
        ref_id = self._get_reference_trajectory_id()

        # 4. Finish the mapping trajectory.
        traj_id = self._get_active_trajectory_id()
        if traj_id is None:
            rospy.logerr(
                "No active trajectory to finish — aborting")
            self._set_mode(self.MODE_ERROR)
            return

        ok = self._call_finish_trajectory(traj_id)
        if not ok:
            self._set_mode(self.MODE_ERROR)
            return

        # 5. Start localization trajectory with cached pose.
        new_id = self._call_start_trajectory(
            config_file=self.config_file_localization,
            use_initial_pose=True,
            pose=pose,
            relative_to=ref_id,
        )
        if new_id is None:
            self._set_mode(self.MODE_ERROR)
            return

        rospy.logwarn(
            "Localisation trajectory %d started — pure_localisation"
            " active", new_id)
        self._set_mode(self.MODE_LOCALIZING)

    def _carto_start(self):
        """Launch cartographer_node via roslaunch subprocess."""
        if self._carto_process is not None:
            ret = self._carto_process.poll()
            if ret is None:
                rospy.logwarn("cartographer_node already running")
                # Ensure mode reflects reality.
                if self._mode == self.MODE_IDLE:
                    traj_id = self._get_active_trajectory_id()
                    if traj_id is not None:
                        self._set_mode(self.MODE_MAPPING)
                return

        rospy.logwarn("Launching cartographer_node ...")
        launch_path = "$(find {})/launch/{}".format(
            self.carto_launch_pkg, self.carto_launch_file)
        try:
            self._carto_process = subprocess.Popen(
                ["roslaunch", self.carto_launch_pkg, self.carto_launch_file],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid,
            )
        except Exception as e:
            rospy.logerr("Failed to launch cartographer_node: %s", e)
            self._set_mode(self.MODE_ERROR)
            return

        rospy.logwarn("cartographer_node launched (PID %d)",
                      self._carto_process.pid)
        self._set_mode(self.MODE_MAPPING)

    def _carto_stop(self):
        """Kill cartographer_node."""
        if self._carto_process is None:
            rospy.logwarn("cartographer_node was not started by this"
                          " manager — will try pkill")
            try:
                subprocess.run(
                    ["pkill", "-f", "cartographer_node"],
                    check=False)
            except Exception as e:
                rospy.logerr("pkill failed: %s", e)
            self._set_mode(self.MODE_IDLE)
            return

        try:
            pgid = os.getpgid(self._carto_process.pid)
            os.killpg(pgid, signal.SIGTERM)
            self._carto_process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            rospy.logwarn(
                "cartographer_node did not terminate — sending SIGKILL")
            os.killpg(pgid, signal.SIGKILL)
            self._carto_process.wait()
        except Exception as e:
            rospy.logerr("Error stopping cartographer_node: %s", e)

        self._carto_process = None
        rospy.logwarn("cartographer_node stopped")
        self._set_mode(self.MODE_IDLE)

    # ──────────────────── Internal helpers ───────────────────────────────

    def _set_mode(self, mode):
        """Set the current mode and publish status."""
        self._mode = mode
        self._publish_status(mode)

    def _publish_status(self, mode):
        """Publish current mode on /mode_status."""
        try:
            self._status_pub.publish(String(data=mode))
        except Exception as e:
            rospy.logwarn_throttle(5.0,
                                   "Failed to publish status: %s", e)

    def _get_pose(self):
        """Look up map→base_footprint transform and return as Pose.

        Retries with tf_timeout. Returns None on failure.
        """
        try:
            tf_stamped = self._tf_buffer.lookup_transform(
                self.map_frame, self.base_frame,
                rospy.Time(0),
                rospy.Duration(self.tf_timeout))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("TF lookup %s→%s failed: %s",
                          self.map_frame, self.base_frame, e)
            return None

        pose = Pose()
        pose.position.x = tf_stamped.transform.translation.x
        pose.position.y = tf_stamped.transform.translation.y
        pose.position.z = tf_stamped.transform.translation.z
        pose.orientation = tf_stamped.transform.rotation

        # Log yaw for debugging.
        q = pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        rospy.loginfo(
            "Cached pose: x=%.2f y=%.2f yaw=%.1f°",
            pose.position.x, pose.position.y, yaw * 57.3)
        return pose

    def _is_carto_running(self):
        """Check whether cartographer_node is up by probing a service."""
        try:
            rospy.wait_for_service(
                self._states_svc_name, timeout=1.0)
            return True
        except rospy.ROSException:
            return False

    def _get_service(self, srv_type, srv_name):
        """Get (or create) a service proxy, waiting for it."""
        try:
            rospy.wait_for_service(srv_name, timeout=1.0)
            return rospy.ServiceProxy(srv_name, srv_type)
        except rospy.ROSException:
            return None

    def _get_reference_trajectory_id(self):
        """Find a stable FINISHED trajectory to use as a relative reference.

        The first mapping trajectory (ID 0) is a safe choice — it has no
        PureLocalizationTrimmer so it stays FINISHED (never auto-deleted).
        Falls back to 0 if no FINISHED trajectory is found.
        """
        srv = self._get_service(GetTrajectoryStates,
                                self._states_svc_name)
        if srv is None:
            return 0
        try:
            resp = srv()
            states = resp.trajectory_states
            for i in range(len(states.trajectory_id)):
                if states.trajectory_state[i] == self._TRAJ_FINISHED:
                    tid = states.trajectory_id[i]
                    rospy.loginfo(
                        "Using FINISHED trajectory %d as reference", tid)
                    return tid
            return 0
        except Exception as e:
            rospy.logwarn("GetTrajectoryStates for reference failed: %s", e)
            return 0

    def _get_active_trajectory_id(self):
        """Return the latest ACTIVE trajectory ID, or None."""
        srv = self._get_service(GetTrajectoryStates,
                                self._states_svc_name)
        if srv is None:
            return None
        try:
            resp = srv()
            states = resp.trajectory_states
            # Iterate in reverse to find the latest active trajectory.
            for i in range(len(states.trajectory_id) - 1, -1, -1):
                if states.trajectory_state[i] == self._TRAJ_ACTIVE:
                    return states.trajectory_id[i]
            return None
        except Exception as e:
            rospy.logerr("GetTrajectoryStates failed: %s", e)
            return None

    def _call_finish_trajectory(self, traj_id):
        """Finish a trajectory. Returns True on success."""
        srv = self._get_service(FinishTrajectory,
                                self._finish_svc_name)
        if srv is None:
            rospy.logerr("finish_trajectory service unavailable")
            return False
        try:
            req = FinishTrajectoryRequest()
            req.trajectory_id = traj_id
            resp = srv(req)
            if resp.status.code == 0:  # StatusCode.OK
                rospy.loginfo("Finished trajectory %d", traj_id)
                return True
            else:
                rospy.logerr("Finish trajectory %d failed: %s",
                             traj_id, resp.status.message)
                return False
        except Exception as e:
            rospy.logerr("FinishTrajectory exception: %s", e)
            return False

    def _call_start_trajectory(self, config_file,
                               use_initial_pose, pose,
                               relative_to):
        """Start a new trajectory. Returns the new trajectory ID or None."""
        srv = self._get_service(StartTrajectory,
                                self._start_svc_name)
        if srv is None:
            rospy.logerr("start_trajectory service unavailable")
            return None
        try:
            req = StartTrajectoryRequest()
            req.configuration_directory = self.config_dir
            req.configuration_basename = config_file
            req.use_initial_pose = use_initial_pose
            req.relative_to_trajectory_id = relative_to
            if use_initial_pose and pose is not None:
                req.initial_pose = pose
            resp = srv(req)
            if resp.status.code == 0:  # StatusCode.OK
                rospy.loginfo(
                    "Started trajectory %d with '%s'",
                    resp.trajectory_id, config_file)
                return resp.trajectory_id
            else:
                rospy.logerr(
                    "Start trajectory failed: %s",
                    resp.status.message)
                return None
        except Exception as e:
            rospy.logerr("StartTrajectory exception: %s", e)
            return None

    def _call_write_state(self, filename):
        """Save state to .pbstream. Returns True on success."""
        srv = self._get_service(WriteState, self._write_svc_name)
        if srv is None:
            rospy.logwarn("write_state service unavailable —"
                          " map save skipped")
            return False
        try:
            req = WriteStateRequest()
            req.filename = filename
            req.include_unfinished_submaps = True
            resp = srv(req)
            if resp.status.code == 0:  # StatusCode.OK
                rospy.loginfo("State saved to %s", filename)
                return True
            else:
                rospy.logwarn("WriteState returned: %s",
                              resp.status.message)
                return False
        except Exception as e:
            rospy.logwarn("WriteState exception: %s", e)
            return False


if __name__ == "__main__":
    rospy.init_node("mode_manager")
    manager = ModeManager()
    rospy.spin()
