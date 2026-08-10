#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
cartographer_guard_imu.py

IMU-based fall guard — hard reset only.
Detection: roll / pitch only.
Recovery flow:
  IDLE -> fall detected -> FALLEN (wait upright)
       -> upright confirmed -> RECOVERING -> hard reset -> IDLE
"""

import math
import os
import signal
import subprocess

import rospy
import tf2_ros
from cartographer_ros_msgs.srv import StartTrajectory, StartTrajectoryRequest
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class CartographerImuGuard:
    STATE_IDLE       = 0
    STATE_FALLEN     = 1
    STATE_RECOVERING = 2

    RECOVERY_HARD_RESET = "hard_reset"
    RECOVERY_SEEDED_HARD_RESTART = "seeded_hard_restart"

    def __init__(self):
        # ── Fall detection (roll/pitch only) ──────────────────────────────
        self.imu_topic = rospy.get_param("~imu_topic", "/imu_mux")
        self.fall_roll_threshold_deg     = rospy.get_param("~fall_roll_threshold_deg",     20.0)
        self.fall_pitch_threshold_deg    = rospy.get_param("~fall_pitch_threshold_deg",    30.0)
        self.upright_roll_threshold_deg  = rospy.get_param("~upright_roll_threshold_deg",   5.0)
        self.upright_pitch_threshold_deg = rospy.get_param("~upright_pitch_threshold_deg",  5.0)
        self.upright_confirm_frames      = rospy.get_param("~upright_confirm_frames",        5)
        self.min_pending_duration        = rospy.get_param("~min_pending_duration",         1.5)
        self.reset_cooldown              = rospy.get_param("~reset_cooldown",               10.0)

        # ── Hard reset ────────────────────────────────────────────────────
        self.cartographer_node_name      = rospy.get_param("~cartographer_node_name",      "/cartographer_node")
        self.cartographer_grid_node_name = rospy.get_param("~cartographer_grid_node_name", "/cartographer_occupancy_grid_node")
        self.reset_launch_package        = rospy.get_param("~reset_launch_package",        "cartographer_ros")
        self.reset_launch_file           = rospy.get_param("~reset_launch_file",           "cartographer_localization_runtime.launch")
        self.config_dir                  = rospy.get_param("~config_dir",                  "")
        self.config_file                 = rospy.get_param("~config_file",                 "")
        self.load_state_filename         = rospy.get_param("~load_state_filename",         "")
        self.recovery_mode               = rospy.get_param("~recovery_mode",               self.RECOVERY_HARD_RESET)

        # ── Seeded restart ────────────────────────────────────────────────
        self.map_frame                   = rospy.get_param("~map_frame",                   "map")
        self.robot_frame                 = rospy.get_param("~robot_frame",                 "base_link")
        self.start_service               = rospy.get_param("~start_service",               "/start_trajectory")
        self.seed_pose_max_age           = rospy.get_param("~seed_pose_max_age",           30.0)
        self.seed_start_timeout          = rospy.get_param("~seed_start_timeout",          20.0)

        # ── Internal state ────────────────────────────────────────────────
        self.state                 = self.STATE_IDLE
        self.last_reset_time       = 0.0
        self._reset_launch_proc    = None
        self._is_hard_resetting    = False
        self._cartographer_stopped = False
        self.pending_reset_reason  = None
        self.pending_reset_since   = 0.0
        self.upright_confirm_count = 0
        self.last_cached_pose      = None

        # ── TF pose cache ─────────────────────────────────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.Timer(rospy.Duration(0.5), self._pose_cache_callback)

        # ── Subscriptions ─────────────────────────────────────────────────
        rospy.Subscriber(self.imu_topic, Imu, self.imu_callback, queue_size=20)

        rospy.loginfo(
            f"CartographerImuGuard: fall detect |roll|>{self.fall_roll_threshold_deg:.0f}deg "
            f"or |pitch|>{self.fall_pitch_threshold_deg:.0f}deg. "
            f"Recovery mode: {self.recovery_mode}.")

    # ─────────────────────────────────────────────────────────────────────
    # Hard reset
    # ─────────────────────────────────────────────────────────────────────

    def _kill_node_if_running(self, node_name):
        try:
            subprocess.run(
                ["rosnode", "kill", node_name],
                check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as exc:
            rospy.logwarn(f"Failed to kill {node_name}: {exc}")

    def _pose_cache_callback(self, event):
        if self.recovery_mode != self.RECOVERY_SEEDED_HARD_RESTART:
            return
        if self.state != self.STATE_IDLE:
            return
        try:
            trans = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame,
                rospy.Time(0), rospy.Duration(0.05))
            px = trans.transform.translation.x
            py = trans.transform.translation.y
            q = trans.transform.rotation
            yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
            self.last_cached_pose = (rospy.Time.now().to_sec(), px, py, yaw)
        except Exception:
            pass

    def _stop_cartographer_nodes(self):
        self._kill_node_if_running(self.cartographer_grid_node_name)
        self._kill_node_if_running(self.cartographer_node_name)
        rospy.sleep(1.0)
        self._cartographer_stopped = True

    def _start_seeded_trajectory(self):
        if self.last_cached_pose is None:
            raise RuntimeError("no cached pose available")

        pose_age = rospy.Time.now().to_sec() - self.last_cached_pose[0]
        if pose_age > self.seed_pose_max_age:
            raise RuntimeError(f"cached pose too old: {pose_age:.1f}s")

        rospy.wait_for_service(self.start_service, timeout=self.seed_start_timeout)
        start_proxy = rospy.ServiceProxy(self.start_service, StartTrajectory)

        _, px, py, yaw = self.last_cached_pose
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)

        req = StartTrajectoryRequest()
        req.configuration_directory = self.config_dir
        req.configuration_basename = self.config_file
        req.use_initial_pose = True
        req.relative_to_trajectory_id = 0
        req.initial_pose.position.x = float(px)
        req.initial_pose.position.y = float(py)
        req.initial_pose.position.z = 0.0
        req.initial_pose.orientation.x = float(qx)
        req.initial_pose.orientation.y = float(qy)
        req.initial_pose.orientation.z = float(qz)
        req.initial_pose.orientation.w = float(qw)

        resp = start_proxy(req)
        if not resp.status.code == 0:
            raise RuntimeError(f"start_trajectory failed: {resp.status.message}")

        rospy.logwarn(
            f"IMU SEEDED RESTART: trajectory_id={resp.trajectory_id}, "
            f"pose_age={pose_age:.1f}s, x={px:.2f}, y={py:.2f}, yaw={math.degrees(yaw):.1f}deg")

    def _hard_reset_cartographer(self, reason):
        self._is_hard_resetting = True
        self.last_reset_time    = rospy.Time.now().to_sec()
        rospy.logwarn(f"IMU HARD RESET: {reason}")

        if self._reset_launch_proc is not None and self._reset_launch_proc.poll() is None:
            try:
                os.killpg(os.getpgid(self._reset_launch_proc.pid), signal.SIGTERM)
            except Exception:
                pass

        self._stop_cartographer_nodes()

        cmd = [
            "roslaunch",
            self.reset_launch_package,
            self.reset_launch_file,
            f"configuration_directory:={self.config_dir}",
            f"configuration_basename:={self.config_file}",
            f"load_state_filename:={self.load_state_filename}",
            "launch_rviz:=false",
        ]
        rospy.logwarn("IMU HARD RESET: relaunch: %s", " ".join(cmd))
        self._reset_launch_proc = subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid,
        )
        self._cartographer_stopped = False
        self._is_hard_resetting = False
        self.state              = self.STATE_IDLE
        rospy.logwarn("IMU HARD RESET: relaunch started.")

    def _seeded_restart_cartographer(self, reason):
        self._is_hard_resetting = True
        self.last_reset_time = rospy.Time.now().to_sec()
        rospy.logwarn(f"IMU SEEDED HARD RESTART: {reason}")

        if self._reset_launch_proc is not None and self._reset_launch_proc.poll() is None:
            try:
                os.killpg(os.getpgid(self._reset_launch_proc.pid), signal.SIGTERM)
            except Exception:
                pass

        cmd = [
            "roslaunch",
            self.reset_launch_package,
            self.reset_launch_file,
            f"configuration_directory:={self.config_dir}",
            f"configuration_basename:={self.config_file}",
            f"load_state_filename:={self.load_state_filename}",
            "launch_rviz:=false",
            "start_trajectory_with_default_topics:=false",
        ]
        rospy.logwarn("IMU SEEDED HARD RESTART: relaunch: %s", " ".join(cmd))
        self._reset_launch_proc = subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid,
        )

        try:
            self._start_seeded_trajectory()
            self._cartographer_stopped = False
            self._is_hard_resetting = False
            self.state = self.STATE_IDLE
        except Exception as exc:
            rospy.logerr(f"Seeded restart failed: {exc}. Falling back to hard reset.")
            self._cartographer_stopped = False
            self._is_hard_resetting = False
            self._hard_reset_cartographer(f"seeded restart fallback after {reason}")

    # ─────────────────────────────────────────────────────────────────────
    # IMU callback
    # ─────────────────────────────────────────────────────────────────────

    def imu_callback(self, msg):
        if self._is_hard_resetting:
            return
        if self.state == self.STATE_RECOVERING:
            return

        now = rospy.Time.now().to_sec()
        if now - self.last_reset_time < self.reset_cooldown:
            rospy.loginfo_throttle(
                2.0,
                f"IMU COOLDOWN: {self.reset_cooldown - (now - self.last_reset_time):.1f}s remaining")
            return

        q = msg.orientation
        roll, pitch, _ = euler_from_quaternion([q.x, q.y, q.z, q.w])
        roll_deg  = abs(math.degrees(roll))
        pitch_deg = abs(math.degrees(pitch))

        if self.state == self.STATE_FALLEN:
            pending_elapsed = now - self.pending_reset_since
            if (roll_deg  <= self.upright_roll_threshold_deg and
                    pitch_deg <= self.upright_pitch_threshold_deg):
                self.upright_confirm_count += 1
            else:
                self.upright_confirm_count = 0

            if (pending_elapsed >= self.min_pending_duration and
                    self.upright_confirm_count >= self.upright_confirm_frames):
                reason = self.pending_reset_reason
                self.pending_reset_reason  = None
                self.pending_reset_since   = 0.0
                self.upright_confirm_count = 0
                self.state                 = self.STATE_RECOVERING
                rospy.logwarn(
                    f"IMU UPRIGHT CONFIRMED: |roll|={roll_deg:.1f}deg, "
                    f"|pitch|={pitch_deg:.1f}deg after {pending_elapsed:.1f}s. "
                    "Triggering recovery.")
                if self.recovery_mode == self.RECOVERY_SEEDED_HARD_RESTART:
                    self._seeded_restart_cartographer(reason)
                else:
                    self._hard_reset_cartographer(reason)
            else:
                rospy.loginfo_throttle(
                    2.0,
                    f"IMU WAITING UPRIGHT: "
                    f"|roll|={roll_deg:.1f}/{self.upright_roll_threshold_deg:.1f}deg, "
                    f"|pitch|={pitch_deg:.1f}/{self.upright_pitch_threshold_deg:.1f}deg, "
                    f"frames={self.upright_confirm_count}/{self.upright_confirm_frames}, "
                    f"elapsed={pending_elapsed:.1f}/{self.min_pending_duration:.1f}s")
            return

        # STATE_IDLE: detect fall
        if (roll_deg  > self.fall_roll_threshold_deg or
                pitch_deg > self.fall_pitch_threshold_deg):
            self.pending_reset_reason  = (
                f"fall: |roll|={roll_deg:.1f}deg, |pitch|={pitch_deg:.1f}deg")
            self.pending_reset_since   = now
            self.upright_confirm_count = 0
            self.state                 = self.STATE_FALLEN
            rospy.logwarn(
                f"IMU FALL DETECTED: |roll|={roll_deg:.1f}deg "
                f"(thr={self.fall_roll_threshold_deg:.0f}), "
                f"|pitch|={pitch_deg:.1f}deg "
                f"(thr={self.fall_pitch_threshold_deg:.0f}). "
                "Waiting for upright.")
            if self.recovery_mode == self.RECOVERY_SEEDED_HARD_RESTART:
                rospy.logwarn("IMU SEEDED HARD RESTART: stopping Cartographer while fallen.")
                self._stop_cartographer_nodes()


if __name__ == "__main__":
    rospy.init_node("cartographer_guard_imu")
    node = CartographerImuGuard()
    rospy.spin()
