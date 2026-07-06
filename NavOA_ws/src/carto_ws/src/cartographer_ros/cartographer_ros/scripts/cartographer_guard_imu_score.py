#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
cartographer_guard_imu_score.py

Combined guard: IMU fall detection + matching-score-based triggering.
Both paths use the same recovery: hard reset (kill nodes + roslaunch relaunch).

Detection sources (both enabled by default):
  1. IMU:  roll/pitch fall detection → hard reset
  2. Score: matching score below threshold for N consecutive frames → hard reset

States:
  STATE_IDLE        - monitoring both IMU and score
  STATE_FALLEN      - IMU detected fall, waiting for upright
  STATE_RECOVERING  - executing hard reset (from either trigger)
"""

import math
import os
import signal
import subprocess

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion


class CartographerImuScoreGuard:
    STATE_IDLE       = 0
    STATE_FALLEN     = 1
    STATE_RECOVERING = 2

    def __init__(self):
        # ==================================================================
        # IMU fall detection (roll/pitch only)
        # ==================================================================
        self.imu_enabled                = rospy.get_param("~imu_enabled", True)
        self.imu_topic                  = rospy.get_param("~imu_topic", "/imu_serial")
        self.fall_roll_threshold_deg    = rospy.get_param("~fall_roll_threshold_deg",     20.0)
        self.fall_pitch_threshold_deg   = rospy.get_param("~fall_pitch_threshold_deg",    30.0)
        self.upright_roll_threshold_deg = rospy.get_param("~upright_roll_threshold_deg",   5.0)
        self.upright_pitch_threshold_deg= rospy.get_param("~upright_pitch_threshold_deg", 10.0)
        self.upright_confirm_frames     = rospy.get_param("~upright_confirm_frames",        5)
        self.min_pending_duration       = rospy.get_param("~min_pending_duration",         1.5)

        # ==================================================================
        # Score-based triggering
        # ==================================================================
        self.score_enabled            = rospy.get_param("~score_enabled", True)
        self.score_topic              = rospy.get_param("~score_topic", "/matching_score")
        self.score_threshold          = rospy.get_param("~score_threshold", 0.60)
        self.score_consecutive_frames = rospy.get_param("~score_consecutive_frames", 10)

        # ==================================================================
        # Shared cooldown (applies to both IMU and score paths)
        # ==================================================================
        self.reset_cooldown = rospy.get_param("~reset_cooldown", 10.0)

        # ==================================================================
        # Hard reset config (shared by both paths)
        # ==================================================================
        self.cartographer_node_name      = rospy.get_param("~cartographer_node_name",      "/cartographer_node")
        self.cartographer_grid_node_name = rospy.get_param("~cartographer_grid_node_name", "/cartographer_occupancy_grid_node")
        self.reset_launch_package        = rospy.get_param("~reset_launch_package",        "cartographer_ros")
        self.reset_launch_file           = rospy.get_param("~reset_launch_file",           "cartographer_localization_runtime.launch")
        self.config_dir                  = rospy.get_param("~config_dir",                  "")
        self.config_file                 = rospy.get_param("~config_file",                 "")
        self.load_state_filename         = rospy.get_param("~load_state_filename",         "")

        # ==================================================================
        # Internal state
        # ==================================================================
        self.state = self.STATE_IDLE

        self.last_reset_time       = 0.0
        self._reset_launch_proc    = None
        self._is_hard_resetting    = False
        self.pending_reset_reason  = None
        self.pending_reset_since   = 0.0
        self.upright_confirm_count = 0
        self.score_low_count       = 0

        # ==================================================================
        # Subscriptions
        # ==================================================================
        if self.imu_enabled:
            rospy.Subscriber(self.imu_topic, Imu, self.imu_callback, queue_size=20)
        if self.score_enabled:
            rospy.Subscriber(self.score_topic, Float32, self.score_callback)

        rospy.loginfo(
            f"CartographerImuScoreGuard: IMU={'ON' if self.imu_enabled else 'OFF'} "
            f"(|roll|>{self.fall_roll_threshold_deg:.0f}deg or "
            f"|pitch|>{self.fall_pitch_threshold_deg:.0f}deg), "
            f"Score={'ON' if self.score_enabled else 'OFF'} "
            f"(thr={self.score_threshold:.2f}, frames={self.score_consecutive_frames})")

    # ======================================================================
    # Helpers
    # ======================================================================

    def _kill_node_if_running(self, node_name):
        try:
            subprocess.run(
                ["rosnode", "kill", node_name],
                check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as exc:
            rospy.logwarn(f"Failed to kill {node_name}: {exc}")

    def _do_hard_reset(self, reason, source="IMU"):
        """Kill cartographer nodes and relaunch via roslaunch."""
        self.state = self.STATE_RECOVERING
        self._is_hard_resetting = True
        self.last_reset_time = rospy.Time.now().to_sec()
        rospy.logwarn(f"[{source}] HARD RESET: {reason}")

        # Kill old launch process if still running
        if self._reset_launch_proc is not None and self._reset_launch_proc.poll() is None:
            try:
                os.killpg(os.getpgid(self._reset_launch_proc.pid), signal.SIGTERM)
            except Exception:
                pass

        # Kill cartographer nodes
        self._kill_node_if_running(self.cartographer_grid_node_name)
        self._kill_node_if_running(self.cartographer_node_name)
        rospy.sleep(1.0)

        # Relaunch
        cmd = [
            "roslaunch",
            self.reset_launch_package,
            self.reset_launch_file,
            f"configuration_directory:={self.config_dir}",
            f"configuration_basename:={self.config_file}",
            f"load_state_filename:={self.load_state_filename}",
            "launch_rviz:=false",
        ]
        rospy.logwarn(f"[{source}] HARD RESET: relaunch: %s", " ".join(cmd))
        self._reset_launch_proc = subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid,
        )

        self._is_hard_resetting = False
        self.state = self.STATE_IDLE
        self.score_low_count = 0
        rospy.logwarn(f"[{source}] HARD RESET: relaunch started.")

    # ======================================================================
    # IMU callback
    # ======================================================================

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
                rospy.logwarn(
                    f"IMU UPRIGHT CONFIRMED: |roll|={roll_deg:.1f}deg, "
                    f"|pitch|={pitch_deg:.1f}deg after {pending_elapsed:.1f}s. "
                    "Triggering recovery.")
                self._do_hard_reset(reason, source="IMU")
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
            self.state = self.STATE_FALLEN
            rospy.logwarn(
                f"IMU FALL DETECTED: |roll|={roll_deg:.1f}deg "
                f"(thr={self.fall_roll_threshold_deg:.0f}), "
                f"|pitch|={pitch_deg:.1f}deg "
                f"(thr={self.fall_pitch_threshold_deg:.0f}). "
                "Waiting for upright.")

    # ======================================================================
    # Score callback
    # ======================================================================

    def score_callback(self, msg):
        """Matching score callback — triggers hard reset when score drops."""
        if self.state in (self.STATE_FALLEN, self.STATE_RECOVERING):
            return
        if self._is_hard_resetting:
            return

        now = rospy.Time.now().to_sec()
        if now - self.last_reset_time < self.reset_cooldown:
            return

        if msg.data < self.score_threshold:
            self.score_low_count += 1
            rospy.logdebug(
                f"Score low: {msg.data:.3f} < {self.score_threshold}, "
                f"count={self.score_low_count}/{self.score_consecutive_frames}")

            if self.score_low_count >= self.score_consecutive_frames:
                rospy.logwarn(
                    f"SCORE TRIGGER: Score < {self.score_threshold} for "
                    f"{self.score_low_count} consecutive frames "
                    f"(last={msg.data:.3f})")
                self._do_hard_reset(
                    f"score={msg.data:.3f} < {self.score_threshold} "
                    f"for {self.score_low_count} frames",
                    source="SCORE")
        else:
            if self.score_low_count > 0:
                rospy.loginfo(f"Score recovered to {msg.data:.3f}, resetting count")
            self.score_low_count = 0


if __name__ == "__main__":
    rospy.init_node("cartographer_guard_imu_score")
    node = CartographerImuScoreGuard()
    rospy.spin()
