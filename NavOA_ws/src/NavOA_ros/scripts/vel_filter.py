#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import copy
import math

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String


class VelocityFilter:
    def __init__(self):
        rospy.init_node('cmd_vel_modifier_node', anonymous=True)

        self.scan_topic = rospy.get_param('~scan_topic', '/scan')
        self.cmd_in_topic = rospy.get_param('~cmd_in_topic', '/cmd_vel_tem')
        self.cmd_out_topic = rospy.get_param('~cmd_out_topic', '/cmd_vel')

        self.cmd_timeout = float(rospy.get_param('~cmd_timeout', 0.5))
        self.publish_rate = float(rospy.get_param('~publish_rate', 20.0))
        self.debug_topic = rospy.get_param('~debug_topic', '/vel_filter/debug')

        self.deadband_linear_x = float(rospy.get_param('~deadband_linear_x', 0.0))
        self.deadband_angular_z = float(rospy.get_param('~deadband_angular_z', 0.0))

        self.allow_reverse = bool(rospy.get_param('~allow_reverse', True))
        self.reverse_requires_obstacle = bool(rospy.get_param('~reverse_requires_obstacle', True))
        self.reverse_obstacle_distance = float(rospy.get_param('~reverse_obstacle_distance', 0.35))
        self.max_reverse_speed = float(rospy.get_param('~max_reverse_speed', 0.25))

        self.enable_forward_safety = bool(rospy.get_param('~enable_forward_safety', True))
        self.front_center_angle = float(rospy.get_param('~front_center_angle', 0.0))
        self.forward_stop_distance = float(rospy.get_param('~forward_stop_distance', 0.30))
        self.forward_slow_distance = float(rospy.get_param('~forward_slow_distance', 0.60))
        self.forward_stop_release_distance = float(
            rospy.get_param('~forward_stop_release_distance', self.forward_slow_distance)
        )
        self.min_forward_speed = float(rospy.get_param('~min_forward_speed', 0.05))
        self.front_half_angle = float(rospy.get_param('~front_half_angle', 0.52))

        # Output clamps (safety / easy tuning)
        # Use <=0 to disable.
        self.max_forward_speed = float(rospy.get_param('~max_forward_speed', 0.0))
        self.max_angular_speed = float(rospy.get_param('~max_angular_speed', 0.0))
        # Differential-drive pivot rule: once the final command reaches the
        # configured angular-speed limit, remove all translation so the robot
        # rotates about its current position instead of drawing a tight arc.
        self.pivot_at_max_angular_speed = bool(
            rospy.get_param('~pivot_at_max_angular_speed', True)
        )

        # Global angular velocity scale (applied to all modes).
        self.wz_scale = float(rospy.get_param('~wz_scale', 1.0))

        # If enabled: when an obstacle is closer than the stop distance and an incoming
        # command requests forward motion, override to a small reverse speed.
        # This is useful when the upstream planner keeps commanding forward.
        self.enable_auto_reverse = bool(rospy.get_param('~enable_auto_reverse', False))
        self.auto_reverse_speed = float(rospy.get_param('~auto_reverse_speed', -0.1))

        self.stop_turn_when_forward_blocked = bool(
            rospy.get_param('~stop_turn_when_forward_blocked', True)
        )

        # Optional reactive steering: when front is blocked, steer toward the side
        # with larger free space estimated from LaserScan.
        self.enable_free_space_steer = bool(rospy.get_param('~enable_free_space_steer', False))
        self.steer_scan_half_angle = float(rospy.get_param('~steer_scan_half_angle', 1.57))
        self.steer_deadzone = float(rospy.get_param('~steer_deadzone', 0.10))
        self.steer_gain = float(rospy.get_param('~steer_gain', 1.2))
        self.steer_max_angular = float(rospy.get_param('~steer_max_angular', 0.6))
        self.steer_activation_distance = float(
            rospy.get_param('~steer_activation_distance', self.forward_slow_distance)
        )
        self.steer_release_distance = float(
            rospy.get_param('~steer_release_distance', self.steer_activation_distance + 0.20)
        )
        self.steer_forward_speed = float(rospy.get_param('~steer_forward_speed', 0.05))
        self.enable_turn_search_when_blocked = bool(
            rospy.get_param('~enable_turn_search_when_blocked', True)
        )
        self.turn_search_angular = float(rospy.get_param('~turn_search_angular', 0.35))

        # Ignore scan ranges below this distance when estimating obstacle distance.
        # Keep 0.0 by default for safety.
        self.min_scan_range = float(rospy.get_param('~min_scan_range', 0.0))

        # ---- ultrasonic virtual-bumper (separate from merged scan) ----
        self.enable_ultrasonic_bumper = bool(
            rospy.get_param('~enable_ultrasonic_bumper', True)
        )
        self.ultrasonic_stop_distance = float(
            rospy.get_param('~ultrasonic_stop_distance', 0.25)
        )
        self.ultrasonic_slow_distance = float(
            rospy.get_param('~ultrasonic_slow_distance', 0.45)
        )
        self.ultrasonic_side_stop_distance = float(
            rospy.get_param('~ultrasonic_side_stop_distance', 0.20)
        )
        # Distance above which the original algorithm command is blended in
        # (vx/wz not overridden). Below this: pure evasion with fixed forward speed.
        self.ultrasonic_blend_distance = float(
            rospy.get_param('~ultrasonic_blend_distance', 0.13)
        )
        # evasive action speeds when ultrasonic triggers lateral evasion
        self.ultrasonic_evade_forward_speed = float(
            rospy.get_param('~ultrasonic_evade_forward_speed', 0.05)
        )
        self.ultrasonic_evade_lateral_speed = float(
            rospy.get_param('~ultrasonic_evade_lateral_speed', 0.10)
        )
        # Linear evade: speed scales linearly with proximity (closer → faster).
        self.ultrasonic_linear_evade = bool(
            rospy.get_param('~ultrasonic_linear_evade', True)
        )

        # Expanded side-stop distance used specifically when the robot is reversing.
        # Larger than ultrasonic_side_stop_distance → detects side obstacles earlier
        # and strafes away while reversing.  0 = disable.
        self.reverse_side_stop_distance = float(
            rospy.get_param('~reverse_side_stop_distance', 0.35)
        )
        # Hold lateral velocity for this long after reverse evasion ends (seconds).
        self.reverse_side_hold_duration = float(
            rospy.get_param('~reverse_side_hold_duration', 0.5)
        )
        self._reverse_last_vy = 0.0
        self._reverse_hold_until = rospy.Time(0)
        # Latched lateral velocity during reverse (prevents left-right oscillation)
        self._reverse_evade_vy = 0.0

        # ---- camscan bias: when /camscan sees a front obstacle but /scan doesn't ----
        self.enable_camscan_bias = bool(
            rospy.get_param('~enable_camscan_bias', True)
        )
        self.camscan_topic = rospy.get_param('~camscan_topic', '/camscan')
        self.camscan_front_threshold = float(
            rospy.get_param('~camscan_front_threshold', 0.4)
        )
        self.camscan_bias_gain = float(
            rospy.get_param('~camscan_bias_gain', 0.5)
        )

        # ---- camscan virtual bumper: /camscan acts as additional front sensor ----
        # Merged with ultrasonic front detection in _apply_ultrasonic_bumper.
        self.enable_camscan_bumper = bool(
            rospy.get_param('~enable_camscan_bumper', False)
        )
        self.camscan_stop_distance = float(
            rospy.get_param('~camscan_stop_distance', 0.30)
        )
        self.camscan_slow_distance = float(
            rospy.get_param('~camscan_slow_distance', 0.50)
        )
        # Raw lidar scan topic (un-merged) for camscan bias comparison
        self.raw_scan_topic = rospy.get_param('~raw_scan_topic', '/scan')
        # per-sensor latest distances (meters), inf = no reading
        self._us_dist = {"front": float('inf'), "left": float('inf'), "right": float('inf')}
        self._us_last_time = {"front": rospy.Time(0), "left": rospy.Time(0), "right": rospy.Time(0)}
        self._us_timeout = rospy.Duration(
            rospy.get_param('~ultrasonic_timeout', 1.0)
        )

        # camscan front tracking
        self._camscan_front_dist = float('inf')
        self._camscan_last_time = rospy.Time(0)
        self._camscan_timeout = rospy.Duration(
            rospy.get_param('~camscan_timeout', 1.0)
        )
        # raw /scan front tracking (for camscan-vs-lidar comparison)
        self._raw_scan_front_dist = float('inf')
        self._raw_scan_last_time = rospy.Time(0)

        self.front_min_range = float('inf')
        self.obstacle_close = False
        self.left_clearance = float('inf')
        self.right_clearance = float('inf')

        self._last_cmd_time = rospy.Time(0)
        self._latest_output = Twist()
        self._has_valid_cmd = False
        self._timeout_published = False
        self._steer_block_latched = False
        self._forward_stop_latched = False
        self._last_turn_sign = 1.0
        self._last_mode = 'idle'

        rospy.Subscriber(self.cmd_in_topic, Twist, self.callback)
        rospy.Subscriber(self.scan_topic, LaserScan, self.scan_callback)

        # ultrasonic virtual-bumper subscriptions
        if self.enable_ultrasonic_bumper:
            rospy.Subscriber("/ultrasonic/front", Float32, self._us_front_cb)
            rospy.Subscriber("/ultrasonic/left", Float32, self._us_left_cb)
            rospy.Subscriber("/ultrasonic/right", Float32, self._us_right_cb)

        # camscan subscription (for glass/transparent obstacle detection)
        if self.enable_camscan_bias or self.enable_camscan_bumper:
            rospy.Subscriber(self.camscan_topic, LaserScan, self._camscan_callback)
        if self.enable_camscan_bias:
            rospy.Subscriber(self.raw_scan_topic, LaserScan, self._raw_scan_callback)

        self.pub = rospy.Publisher(self.cmd_out_topic, Twist, queue_size=10)
        self.debug_pub = rospy.Publisher(self.debug_topic, String, queue_size=10)
        self.status_pub = rospy.Publisher('/vel_filter/status', String, queue_size=10)
        self._timer = rospy.Timer(
            rospy.Duration(1.0 / max(self.publish_rate, 1.0)), self._publish_timer
        )

        rospy.loginfo(
            "vel_filter running: %s -> %s (timeout=%.2fs, rate=%.1fHz, front_center=%.2frad)",
            self.cmd_in_topic,
            self.cmd_out_topic,
            self.cmd_timeout,
            self.publish_rate,
            self.front_center_angle,
        )

    def _angle_diff(self, angle: float, center: float) -> float:
        return math.atan2(math.sin(angle - center), math.cos(angle - center))

    def _apply_deadband(self, twist: Twist) -> Twist:
        if self.deadband_linear_x > 0.0 and abs(twist.linear.x) < self.deadband_linear_x:
            twist.linear.x = 0.0
        if self.deadband_angular_z > 0.0 and abs(twist.angular.z) < self.deadband_angular_z:
            twist.angular.z = 0.0
        return twist

    def _apply_max_angular_pivot(self, twist: Twist) -> bool:
        """Zero translation when the final yaw rate reaches its limit."""
        if (not self.pivot_at_max_angular_speed or
                self.max_angular_speed <= 0.0):
            return False
        tolerance = max(1e-6, self.max_angular_speed * 1e-6)
        if abs(twist.angular.z) < self.max_angular_speed - tolerance:
            return False
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        return True

    def _publish_debug(self, cmd_in: Twist, cmd_out: Twist):
        msg = (
            f"mode={self._last_mode} "
            f"front={self.front_min_range:.3f} left={self.left_clearance:.3f} right={self.right_clearance:.3f} "
            f"stop_latched={self._forward_stop_latched} steer_latched={self._steer_block_latched} "
            f"in_vx={cmd_in.linear.x:.3f} in_w={cmd_in.angular.z:.3f} "
            f"out_vx={cmd_out.linear.x:.3f} out_w={cmd_out.angular.z:.3f}"
        )
        self.debug_pub.publish(String(data=msg))

    def _publish_status(self, twist, source):
        """Publish human-readable motion status: source + trend."""
        vx = twist.linear.x
        vy = twist.linear.y
        wz = twist.angular.z

        # classify motion trend
        if vx > 0.01:
            if wz > 0.05:
                trend = 'forward_left'
            elif wz < -0.05:
                trend = 'forward_right'
            elif vy > 0.005:
                trend = 'strafe_left'
            elif vy < -0.005:
                trend = 'strafe_right'
            else:
                trend = 'forward'
        elif vx < -0.01:
            trend = 'reverse'
        elif abs(vy) > 0.005:
            trend = 'strafe_left' if vy > 0 else 'strafe_right'
        elif abs(wz) > 0.01:
            trend = 'turn_left' if wz > 0 else 'turn_right'
        else:
            trend = 'stop'

        # source label
        src = 'ultrasonic' if source.startswith('us_') else 'navoa'

        msg = (f"source={src} mode={source} trend={trend} "
               f"vx={vx:.3f} vy={vy:.3f} wz={wz:.3f}")
        self.status_pub.publish(String(data=msg))

    def _publish_timer(self, _evt):
        now = rospy.Time.now()
        if self._last_cmd_time != rospy.Time(0) and (now - self._last_cmd_time).to_sec() <= self.cmd_timeout:
            out = copy.deepcopy(self._latest_output)
            self.pub.publish(out)
            self._timeout_published = False
        elif self._has_valid_cmd and not self._timeout_published:
            # Publish a single zero command when the last valid command expires,
            # then stop publishing until a new command arrives.
            rospy.loginfo_once("[vel_filter] cmd_vel timeout expired; publishing single zero command.")
            self.pub.publish(Twist())
            self._timeout_published = True
            self._has_valid_cmd = False
        else:
            pass

    def scan_callback(self, msg: LaserScan):
        if not msg.ranges or msg.angle_increment == 0.0:
            self.front_min_range = float('inf')
            self.obstacle_close = False
            self.left_clearance = float('inf')
            self.right_clearance = float('inf')
            return

        min_valid = max(float(getattr(msg, 'range_min', 0.0) or 0.0), self.min_scan_range)
        min_r = float('inf')
        range_max = float(getattr(msg, 'range_max', 30.0) or 30.0)

        left_sum = 0.0
        left_count = 0
        right_sum = 0.0
        right_count = 0

        angle = msg.angle_min
        for r in msg.ranges:
            valid = r is not None and math.isfinite(r) and r >= min_valid
            rel_angle = self._angle_diff(angle, self.front_center_angle)

            if -self.front_half_angle <= rel_angle <= self.front_half_angle:
                if valid:
                    if r < min_r:
                        min_r = r

            if self.front_half_angle < rel_angle <= self.steer_scan_half_angle and valid:
                left_sum += min(r, range_max)
                left_count += 1

            if -self.steer_scan_half_angle <= rel_angle < -self.front_half_angle and valid:
                right_sum += min(r, range_max)
                right_count += 1

            angle += msg.angle_increment

        self.front_min_range = min_r
        self.obstacle_close = math.isfinite(min_r) and (min_r < self.reverse_obstacle_distance)
        self.left_clearance = left_sum / left_count if left_count > 0 else float('inf')
        self.right_clearance = right_sum / right_count if right_count > 0 else float('inf')

        if math.isfinite(min_r):
            if min_r < self.forward_stop_distance:
                self._forward_stop_latched = True
            elif min_r > self.forward_stop_release_distance:
                self._forward_stop_latched = False

    # ---- ultrasonic callbacks (virtual bumper) -----------------------------
    def _us_front_cb(self, msg):
        self._us_dist['front'] = msg.data
        self._us_last_time['front'] = rospy.Time.now()

    def _us_left_cb(self, msg):
        self._us_dist['left'] = msg.data
        self._us_last_time['left'] = rospy.Time.now()

    def _us_right_cb(self, msg):
        self._us_dist['right'] = msg.data
        self._us_last_time['right'] = rospy.Time.now()

    def _us_get_valid(self, name):
        """Return distance (m) if fresh, else inf."""
        now = rospy.Time.now()
        if (now - self._us_last_time[name]) < self._us_timeout:
            d = self._us_dist[name]
            if math.isfinite(d) and d > 0.0:
                return d
        return float('inf')

    # ---- camscan callback (glass / transparent obstacle detection) ---------
    def _camscan_callback(self, msg: LaserScan):
        """Extract minimum front distance from /camscan."""
        if not msg.ranges or msg.angle_increment == 0.0:
            self._camscan_front_dist = float('inf')
            return
        min_valid = max(float(getattr(msg, 'range_min', 0.0) or 0.0), self.min_scan_range)
        min_r = float('inf')
        angle = msg.angle_min
        for r in msg.ranges:
            if r is not None and math.isfinite(r) and r >= min_valid:
                rel = self._angle_diff(angle, self.front_center_angle)
                if -self.front_half_angle <= rel <= self.front_half_angle:
                    if r < min_r:
                        min_r = r
            angle += msg.angle_increment
        self._camscan_front_dist = min_r
        self._camscan_last_time = rospy.Time.now()

    def _camscan_front_valid(self):
        """Return camscan front distance if fresh, else inf."""
        if (rospy.Time.now() - self._camscan_last_time) < self._camscan_timeout:
            d = self._camscan_front_dist
            if math.isfinite(d) and d > 0.0:
                return d
        return float('inf')

    def _raw_scan_callback(self, msg: LaserScan):
        """Extract minimum front distance from raw /scan (lidar only, 60° FOV)."""
        if not msg.ranges or msg.angle_increment == 0.0:
            self._raw_scan_front_dist = float('inf')
            return
        min_valid = max(float(getattr(msg, 'range_min', 0.0) or 0.0), self.min_scan_range)
        min_r = float('inf')
        angle = msg.angle_min
        for r in msg.ranges:
            if r is not None and math.isfinite(r) and r >= min_valid:
                rel = self._angle_diff(angle, self.front_center_angle)
                if -self.front_half_angle <= rel <= self.front_half_angle:
                    if r < min_r:
                        min_r = r
            angle += msg.angle_increment
        self._raw_scan_front_dist = min_r
        self._raw_scan_last_time = rospy.Time.now()

    def _raw_scan_front_valid(self):
        """Return raw /scan front distance if fresh, else inf."""
        if (rospy.Time.now() - self._raw_scan_last_time) < self._camscan_timeout:
            d = self._raw_scan_front_dist
            if math.isfinite(d) and d > 0.0:
                return d
        return float('inf')

    def _apply_camscan_bias(self, twist):
        """When /camscan sees a front obstacle (< threshold) but raw /scan
        does not (e.g. glass wall, lidar blind spot), add a lateral bias
        derived from rotation trend and ultrasonic side distances."""
        if not self.enable_camscan_bias:
            return twist

        cam_front = self._camscan_front_valid()
        if not math.isfinite(cam_front) or cam_front >= self.camscan_front_threshold:
            return twist  # camscan sees nothing close in front

        # Only act when raw /scan does NOT see the same obstacle (60° FOV front)
        raw_front = self._raw_scan_front_valid()
        if math.isfinite(raw_front) and raw_front < self.camscan_front_threshold:
            return twist  # lidar already sees it → normal processing handles it

        # --- compute rotation bias: follow the turn direction ---
        rot_bias = 0.0
        if twist.angular.z > 0.03:
            rot_bias = 1.0   # turning left → bias left (vy +)
        elif twist.angular.z < -0.03:
            rot_bias = -1.0  # turning right → bias right (vy -)

        # --- compute ultrasonic bias: evade the closer side ---
        us_bias = 0.0
        if self.enable_ultrasonic_bumper:
            d_left = self._us_get_valid('left')
            d_right = self._us_get_valid('right')
            left_ok = math.isfinite(d_left)
            right_ok = math.isfinite(d_right)
            if left_ok and right_ok:
                diff = d_right - d_left  # positive = right is farther → go right
                us_bias = math.copysign(1.0, diff) if abs(diff) > 0.03 else 0.0
            elif left_ok:
                us_bias = -1.0  # only left → bias right
            elif right_ok:
                us_bias = 1.0   # only right → bias left

        # --- combine biases ---
        bias = rot_bias + us_bias
        if abs(bias) < 0.01:
            return twist

        # Scale and apply
        vy_add = math.copysign(self.camscan_bias_gain, bias)
        twist.linear.y += vy_add
        twist.angular.z = 0.0

        return twist

    def _compute_linear_evade_speed(self, distance):
        """Linear evade speed: closer → faster, 0 at threshold, max at 0 distance.

        speed = max_speed * (threshold - distance) / threshold
        """
        if not self.ultrasonic_linear_evade:
            return abs(self.ultrasonic_evade_lateral_speed)
        threshold = self.ultrasonic_side_stop_distance
        if threshold <= 0.0 or distance >= threshold:
            return 0.0
        scale = (threshold - distance) / threshold
        return abs(self.ultrasonic_evade_lateral_speed) * max(0.0, min(1.0, scale))

    def _apply_ultrasonic_bumper(self, twist):
        """Ultrasonic + camscan virtual bumper with active evasion.

        Front detection merges ultrasonic and /camscan:
          - Front blocked  -> reverse
          - Left blocked   -> strafe right (linear.y negative)
          - Right blocked  -> strafe left  (linear.y positive)
        Both sides blocked + front clear → stop rotation, slow reverse.
        All three blocked → reverse.
        """
        if not self.enable_ultrasonic_bumper and not self.enable_camscan_bumper:
            return twist, 'pass_through'

        d_front = self._us_get_valid('front')
        d_left  = self._us_get_valid('left')
        d_right = self._us_get_valid('right')

        # ---- merge camscan front into detection ----
        cam_front_only = False
        if self.enable_camscan_bumper:
            d_cam_front = self._camscan_front_valid()
            if math.isfinite(d_cam_front):
                if math.isfinite(d_front):
                    d_front = min(d_front, d_cam_front)
                else:
                    d_front = d_cam_front
                # Track whether camscan is the sole front trigger
                if (d_cam_front < self.camscan_stop_distance and
                        (not math.isfinite(self._us_get_valid('front')) or
                         self._us_get_valid('front') >= self.ultrasonic_stop_distance)):
                    cam_front_only = True

        # Effective thresholds: most conservative of both sensors
        eff_stop_dist = self.ultrasonic_stop_distance
        eff_slow_dist = self.ultrasonic_slow_distance
        if self.enable_camscan_bumper:
            eff_stop_dist = min(eff_stop_dist, self.camscan_stop_distance)
            eff_slow_dist = max(eff_slow_dist, self.camscan_slow_distance)

        front_blocked = math.isfinite(d_front) and d_front < eff_stop_dist
        left_blocked  = math.isfinite(d_left)  and d_left  < self.ultrasonic_side_stop_distance
        right_blocked = math.isfinite(d_right) and d_right < self.ultrasonic_side_stop_distance

        # ---- no trigger: maybe just slow-down ----
        if not (front_blocked or left_blocked or right_blocked):
            # Reset latched reverse evade direction when front is clear
            self._reverse_evade_vy = 0.0
            # Hold lateral velocity briefly after reverse evasion ends
            now = rospy.Time.now()
            if (self.reverse_side_stop_distance > 0.0 and
                    now < self._reverse_hold_until and
                    abs(self._reverse_last_vy) > 0.001):
                twist.linear.y = self._reverse_last_vy
                return twist, 'us_reverse_hold'

            if math.isfinite(d_front) and d_front < eff_slow_dist:
                if twist.linear.x > 0.0:
                    denom = eff_slow_dist - eff_stop_dist
                    if denom > 0.0:
                        scale = (d_front - eff_stop_dist) / denom
                    else:
                        scale = 0.0
                    scale = max(0.0, min(1.0, scale))
                    twist.linear.x *= scale
                    return twist, 'us_slow_forward'
            return twist, 'pass_through'

        # ---- determine evasive action ----
        mode = 'pass_through'

        if front_blocked:
            # reverse — front obstacle, must reverse
            if self.enable_auto_reverse and self.allow_reverse:
                orig_wz = twist.angular.z  # capture algorithm turn direction before zeroing
                twist.linear.x = -abs(float(self.auto_reverse_speed))
                twist.angular.z = 0.0
                mode = 'cam_reverse' if cam_front_only else 'us_reverse'

                # Expand left/right detection range while reversing,
                # so the robot can simultaneously strafe away from side obstacles.
                # Latch the first chosen direction to prevent ping-pong oscillation.
                if self.reverse_side_stop_distance > 0.0:
                    if abs(self._reverse_evade_vy) < 0.001:
                        # First frame of reverse: decide direction
                        rev_left  = (math.isfinite(d_left) and
                                     d_left < self.reverse_side_stop_distance)
                        rev_right = (math.isfinite(d_right) and
                                     d_right < self.reverse_side_stop_distance)

                        if rev_left and rev_right:
                            # Both sides close — strafe toward the farther side
                            if d_right > d_left:
                                self._reverse_evade_vy = abs(self.ultrasonic_evade_lateral_speed)
                            else:
                                self._reverse_evade_vy = -abs(self.ultrasonic_evade_lateral_speed)
                        elif rev_left:
                            self._reverse_evade_vy = abs(self.ultrasonic_evade_lateral_speed)
                        elif rev_right:
                            self._reverse_evade_vy = -abs(self.ultrasonic_evade_lateral_speed)
                        else:
                            # Both sides clear — follow algorithm's turn direction
                            if orig_wz > 0.03:
                                self._reverse_evade_vy = abs(self.ultrasonic_evade_lateral_speed)
                            elif orig_wz < -0.03:
                                self._reverse_evade_vy = -abs(self.ultrasonic_evade_lateral_speed)
                    twist.linear.y = self._reverse_evade_vy

                    # Safety override: stop lateral movement if strafing toward
                    # a close obstacle (uses ultrasonic_side_stop_distance).
                    if (self._reverse_evade_vy > 0.001 and
                            math.isfinite(d_right) and
                            d_right < self.ultrasonic_side_stop_distance):
                        twist.linear.y = 0.0
                    elif (self._reverse_evade_vy < -0.001 and
                            math.isfinite(d_left) and
                            d_left < self.ultrasonic_side_stop_distance):
                        twist.linear.y = 0.0

                # Record vy and arm hold timer for when reverse ends
                self._reverse_last_vy = twist.linear.y
                self._reverse_hold_until = rospy.Time.now() + rospy.Duration(
                    self.reverse_side_hold_duration
                )
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                mode = 'cam_front_stop' if cam_front_only else 'us_front_stop'

        elif left_blocked and right_blocked:
            speed_left = self._compute_linear_evade_speed(d_left)
            speed_right = self._compute_linear_evade_speed(d_right)
            twist.linear.y = speed_left - speed_right
            # Blend algorithm vx based on closest side distance
            closest_side = min(d_left, d_right)
            if closest_side < self.ultrasonic_blend_distance:
                # Very close: scale down forward speed
                scale = closest_side / max(self.ultrasonic_blend_distance, 0.01)
                twist.linear.x *= max(0.0, scale)
            # Keep algorithm's wz during lateral translation
            mode = 'us_narrow_pass'

        elif left_blocked:
            speed = self._compute_linear_evade_speed(d_left)
            twist.linear.y = speed
            # Blend algorithm vx based on distance
            if d_left < self.ultrasonic_blend_distance:
                scale = d_left / max(self.ultrasonic_blend_distance, 0.01)
                twist.linear.x *= max(0.0, scale)
            # Keep algorithm's wz during lateral translation
            mode = 'us_evade_left'

        elif right_blocked:
            speed = self._compute_linear_evade_speed(d_right)
            twist.linear.y = -speed
            # Blend algorithm vx based on distance
            if d_right < self.ultrasonic_blend_distance:
                scale = d_right / max(self.ultrasonic_blend_distance, 0.01)
                twist.linear.x *= max(0.0, scale)
            # Keep algorithm's wz during lateral translation
            mode = 'us_evade_right'

        return twist, mode

    def _apply_free_space_steer(self, twist: Twist, forward_blocked: bool) -> None:
        if not self.enable_free_space_steer:
            return
        if not forward_blocked:
            return
        if not math.isfinite(self.front_min_range):
            return

        balance = self.left_clearance - self.right_clearance
        if math.isfinite(balance) and abs(balance) >= self.steer_deadzone:
            target_angular = self.steer_gain * balance
            self._last_turn_sign = 1.0 if target_angular >= 0.0 else -1.0
        elif self.enable_turn_search_when_blocked:
            target_angular = self._last_turn_sign * abs(self.turn_search_angular)
        else:
            return

        target_angular = max(-self.steer_max_angular, min(self.steer_max_angular, target_angular))

        # Keep slow creep while steering, or rotate in place if already stopped.
        if twist.linear.x > 0.0:
            twist.linear.x = min(twist.linear.x, self.steer_forward_speed)

        twist.angular.z = target_angular

    def callback(self, msg: Twist):
        self._last_cmd_time = rospy.Time.now()
        self._has_valid_cmd = True

        modified = copy.deepcopy(msg)
        self._last_mode = 'pass_through'

        # ---- ultrasonic virtual bumper (highest priority) ----
        if self.enable_ultrasonic_bumper:
            modified, us_mode = self._apply_ultrasonic_bumper(modified)
            if us_mode != 'pass_through':
                # ultrasonic overrides everything when triggered
                modified.angular.z *= self.wz_scale
                # Apply speed clamps even during ultrasonic evasion (they were
                # previously bypassed by the early return below).
                if self.max_forward_speed > 0.0 and modified.linear.x > self.max_forward_speed:
                    modified.linear.x = self.max_forward_speed
                if self.max_angular_speed > 0.0:
                    az = modified.angular.z
                    if abs(az) > self.max_angular_speed:
                        modified.angular.z = math.copysign(self.max_angular_speed, az)
                pivoting = self._apply_max_angular_pivot(modified)
                modified = self._apply_deadband(modified)
                self._latest_output = modified
                self._last_mode = (us_mode + '_pivot') if pivoting else us_mode
                self._publish_debug(msg, modified)
                self._publish_status(modified, self._last_mode)
                return

        # ---- camscan bias: when camscan sees front obstacle but lidar doesn't ----
        modified = self._apply_camscan_bias(modified)

        # Global angular scale (applied before clamp)
        modified.angular.z *= self.wz_scale

        # Clamp outputs (after all safety logic)
        if self.max_forward_speed > 0.0 and modified.linear.x > self.max_forward_speed:
            modified.linear.x = self.max_forward_speed
        if self.max_angular_speed > 0.0:
            az = modified.angular.z
            if abs(az) > self.max_angular_speed:
                modified.angular.z = math.copysign(self.max_angular_speed, az)

        pivoting = self._apply_max_angular_pivot(modified)
        modified = self._apply_deadband(modified)
        self._latest_output = modified
        self._last_mode = 'max_angular_pivot' if pivoting else 'pass_through'
        self._publish_debug(msg, modified)
        self._publish_status(modified, self._last_mode)


if __name__ == '__main__':
    try:
        VelocityFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass