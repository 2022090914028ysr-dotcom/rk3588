#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
cartographer_guard_imu_score.py

Combined guard: IMU fall detection + matching-score-based triggering.
Both paths use the same recovery: hard reset (kill nodes + roslaunch relaunch).

Detection sources (all enabled by default):
  1. IMU:      roll/pitch fall detection → hard reset
  2. Score:    matching score below threshold for N consecutive frames → hard reset
  3. Offset:   map→base_link position jump > threshold → hard reset
  4. Rotation: map→base_link |Δyaw| > 90° → hard reset

Both offset and rotation only become active after the cartographer node has
been running for offset_startup_delay (default 120s).

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
import tf2_ros
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
        # Offset-based triggering (map→base_link jump detection)
        #   Monitors instantaneous position jumps in map frame.
        #   Only active after cartographer has been running for startup_delay.
        # ==================================================================
        self.offset_enabled            = rospy.get_param("~offset_enabled", True)
        self.offset_threshold          = rospy.get_param("~offset_threshold", 0.50)
        self.offset_consecutive_frames = rospy.get_param("~offset_consecutive_frames", 3)
        self.offset_startup_delay      = rospy.get_param("~offset_startup_delay", 120.0)
        self.map_frame                 = rospy.get_param("~map_frame", "map")
        self.base_frame                = rospy.get_param("~base_frame", "base_link")

        # ==================================================================
        # Rotation-based triggering (yaw jump detection in map frame)
        #   Same startup delay as offset. Triggers when |Δyaw| > threshold.
        # ==================================================================
        self.rotation_enabled            = rospy.get_param("~rotation_enabled", True)
        self.rotation_threshold_deg      = rospy.get_param("~rotation_threshold_deg", 90.0)
        self.rotation_consecutive_frames = rospy.get_param("~rotation_consecutive_frames", 3)

        # ==================================================================
        # Shared cooldown (applies to IMU, score, offset and rotation paths)
        # ==================================================================
        self.reset_cooldown = rospy.get_param("~reset_cooldown", 10.0)

        # ==================================================================
        # Max idle timeout: if no reset has occurred for this many seconds,
        # force a fresh restart anyway.  0.0 (default) = disabled.
        # Any existing trigger (IMU, score, offset, rotation) resets the
        # timer since they all update last_reset_time via _do_hard_reset.
        # ==================================================================
        self.max_idle_timeout = rospy.get_param("~max_idle_timeout", 0.0)

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

        self.last_reset_time       = rospy.Time.now().to_sec()
        self._reset_launch_proc    = None
        self._is_hard_resetting    = False
        self.pending_reset_reason  = None
        self.pending_reset_since   = 0.0
        self.upright_confirm_count = 0
        self.score_low_count       = 0

        self.node_start_time          = rospy.Time.now().to_sec()
        self.last_position            = None
        self.offset_violation_count   = 0
        self.last_yaw                 = None
        self.rotation_violation_count = 0

        # ==================================================================
        # Subscriptions
        # ==================================================================
        if self.imu_enabled:
            rospy.Subscriber(self.imu_topic, Imu, self.imu_callback, queue_size=20)
        if self.score_enabled:
            rospy.Subscriber(self.score_topic, Float32, self.score_callback)
        if self.offset_enabled or self.rotation_enabled:
            self.tf_buffer   = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
            rospy.Timer(rospy.Duration(0.1), self.offset_timer_callback)

        # ==================================================================
        # Max idle timeout timer: checks every 10s if max_idle_timeout
        # has been exceeded since last reset. Idempotent — no cooldown
        # interaction, only fires when STATE_IDLE and timeout > 0.
        # ==================================================================
        if self.max_idle_timeout > 0.0:
            rospy.Timer(rospy.Duration(10.0), self.idle_timeout_callback)

        rospy.loginfo(
            f"CartographerImuScoreGuard: IMU={'ON' if self.imu_enabled else 'OFF'} "
            f"(|roll|>{self.fall_roll_threshold_deg:.0f}deg or "
            f"|pitch|>{self.fall_pitch_threshold_deg:.0f}deg), "
            f"Score={'ON' if self.score_enabled else 'OFF'} "
            f"(thr={self.score_threshold:.2f}, frames={self.score_consecutive_frames}), "
            f"Offset={'ON' if self.offset_enabled else 'OFF'} "
            f"(thr={self.offset_threshold:.2f}m, frames={self.offset_consecutive_frames}, "
            f"delay={self.offset_startup_delay:.0f}s), "
            f"Rotation={'ON' if self.rotation_enabled else 'OFF'} "
            f"(thr={self.rotation_threshold_deg:.0f}deg, "
            f"frames={self.rotation_consecutive_frames}), "
            f"MaxIdleTimeout={'ON' if self.max_idle_timeout > 0 else 'OFF'}"
            f"{f' ({self.max_idle_timeout:.0f}s)' if self.max_idle_timeout > 0 else ''}")

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

        # Reset offset/rotation startup timer so the 2-min delay restarts
        # with the newly launched cartographer node
        self.node_start_time          = rospy.Time.now().to_sec()
        self.last_position            = None
        self.offset_violation_count   = 0
        self.last_yaw                 = None
        self.rotation_violation_count = 0

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

    # ======================================================================
    # Offset / Rotation timer callback (map→base_link)
    # ======================================================================

    def offset_timer_callback(self, event):
        """Check instantaneous position/rotation jumps of base_link.

        Only active after startup_delay (default 120s) has elapsed since the
        cartographer node started.

        Triggers:
          - Position: Euclidean distance > offset_threshold for N frames
          - Rotation: |Δyaw| > rotation_threshold_deg for N frames
        """
        if self.state in (self.STATE_FALLEN, self.STATE_RECOVERING):
            return
        if self._is_hard_resetting:
            return

        now = rospy.Time.now().to_sec()
        if now - self.last_reset_time < self.reset_cooldown:
            return

        # Wait for cartographer to have been running at least startup_delay
        elapsed_since_start = now - self.node_start_time
        if elapsed_since_start < self.offset_startup_delay:
            self.last_position = None
            self.offset_violation_count = 0
            self.last_yaw = None
            self.rotation_violation_count = 0
            return

        # Look up transform
        try:
            trans = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame,
                rospy.Time(0), rospy.Duration(1.0))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            self.last_position = None
            self.offset_violation_count = 0
            self.last_yaw = None
            self.rotation_violation_count = 0
            return

        x = trans.transform.translation.x
        y = trans.transform.translation.y

        # --- Position jump check ---
        if self.offset_enabled:
            self._check_offset(x, y)

        # --- Rotation (yaw) jump check ---
        if self.rotation_enabled:
            q = trans.transform.rotation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            self._check_rotation(yaw)

    # ------------------------------------------------------------------
    def _check_offset(self, x, y):
        """Lock reference position on first violation, compare all subsequent
        frames against the same locked reference until count clears or
        threshold is reached.  This correctly handles the case where the
        robot stays at the jumped-to position."""
        if self.last_position is None:
            self.last_position = (x, y)
            return

        dx = x - self.last_position[0]
        dy = y - self.last_position[1]
        dist = math.hypot(dx, dy)

        if dist > self.offset_threshold:
            self.offset_violation_count += 1
            rospy.logwarn(
                f"OFFSET JUMP: dist={dist:.3f}m > "
                f"thr={self.offset_threshold:.3f}m "
                f"(ref=({self.last_position[0]:.3f},{self.last_position[1]:.3f})), "
                f"count={self.offset_violation_count}/"
                f"{self.offset_consecutive_frames}")
            # Keep last_position LOCKED — do NOT update it
        else:
            if self.offset_violation_count > 0:
                rospy.loginfo(
                    f"Offset normal: dist={dist:.3f}m, resetting count")
            self.offset_violation_count = 0
            # Only update reference when no violation is in progress
            self.last_position = (x, y)

        if self.offset_violation_count >= self.offset_consecutive_frames:
            rospy.logwarn(
                f"OFFSET TRIGGER: Position jump > "
                f"{self.offset_threshold:.3f}m for "
                f"{self.offset_violation_count} consecutive frames "
                f"(ref=({self.last_position[0]:.3f},{self.last_position[1]:.3f}), "
                f"current=({x:.3f},{y:.3f}))")
            self._do_hard_reset(
                f"position jump={dist:.3f}m > {self.offset_threshold:.3f}m "
                f"for {self.offset_violation_count} frames",
                source="OFFSET")
            self.last_position = None
            self.offset_violation_count = 0

    # ------------------------------------------------------------------
    def _check_rotation(self, yaw):
        """Lock reference yaw on first violation, compare all subsequent
        frames against the same locked reference until count clears or
        threshold is reached.  This correctly handles the case where the
        robot stays at the jumped-to orientation."""
        if self.last_yaw is None:
            self.last_yaw = yaw
            return

        # Compute smallest angular difference (handles ±π wrap-around)
        dyaw = abs(yaw - self.last_yaw)
        if dyaw > math.pi:
            dyaw = 2.0 * math.pi - dyaw
        dyaw_deg = math.degrees(dyaw)

        if dyaw_deg > self.rotation_threshold_deg:
            self.rotation_violation_count += 1
            rospy.logwarn(
                f"ROTATION JUMP: |Δyaw|={dyaw_deg:.1f}deg > "
                f"thr={self.rotation_threshold_deg:.0f}deg "
                f"(ref={math.degrees(self.last_yaw):.1f}deg), "
                f"count={self.rotation_violation_count}/"
                f"{self.rotation_consecutive_frames}")
            # Keep last_yaw LOCKED — do NOT update it
        else:
            if self.rotation_violation_count > 0:
                rospy.loginfo(
                    f"Rotation normal: |Δyaw|={dyaw_deg:.1f}deg, resetting count")
            self.rotation_violation_count = 0
            # Only update reference when no violation is in progress
            self.last_yaw = yaw

        if self.rotation_violation_count >= self.rotation_consecutive_frames:
            rospy.logwarn(
                f"ROTATION TRIGGER: |Δyaw| > "
                f"{self.rotation_threshold_deg:.0f}deg for "
                f"{self.rotation_violation_count} consecutive frames "
                f"(ref={math.degrees(self.last_yaw):.1f}deg, "
                f"current={math.degrees(yaw):.1f}deg)")
            self._do_hard_reset(
                f"yaw jump={dyaw_deg:.1f}deg > {self.rotation_threshold_deg:.0f}deg "
                f"for {self.rotation_violation_count} frames",
                source="ROTATION")
            self.last_yaw = None
            self.rotation_violation_count = 0


    # ======================================================================
    # Max idle timeout callback
    # ======================================================================

    def idle_timeout_callback(self, event):
        """Force a fresh restart if no reset has happened in max_idle_timeout.

        Any existing trigger (IMU, score, offset, rotation) resets the
        timer because they all call _do_hard_reset, which updates
        last_reset_time.

        Does NOT check cooldown (cooldown is for preventing rapid successive
        triggers; this is the opposite — a long-absence-of-triggers check).
        """
        if self.state in (self.STATE_FALLEN, self.STATE_RECOVERING):
            return
        if self._is_hard_resetting:
            return
        if self.max_idle_timeout <= 0.0:
            return

        now = rospy.Time.now().to_sec()
        idle = now - self.last_reset_time

        if idle >= self.max_idle_timeout:
            rospy.logwarn(
                f"MAX IDLE TIMEOUT: no reset for {idle:.0f}s "
                f"(threshold={self.max_idle_timeout:.0f}s). "
                "Triggering preventive restart.")
            self._do_hard_reset(
                f"idle timeout: no reset for {idle:.0f}s",
                source="IDLE_TIMEOUT")


if __name__ == "__main__":
    rospy.init_node("cartographer_guard_imu_score")
    node = CartographerImuScoreGuard()
    rospy.spin()
