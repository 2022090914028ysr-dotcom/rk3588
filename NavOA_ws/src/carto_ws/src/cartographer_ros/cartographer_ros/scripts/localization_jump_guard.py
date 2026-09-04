#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
localization_jump_guard.py

Gates the map->odom transform published during Cartographer pure localization,
rejecting the false ~180 deg global relocalizations that the pose graph
occasionally accepts on a symmetric map.

Rationale
---------
map->odom is a *drift correction*: it does not change when the robot moves --
that motion lives in odom->base_footprint. Legitimate corrections are
centimetres and fractions of a degree. A 4 m / 174 deg step in map->odom means
the estimate teleported while the robot did not, which leg odometry (validated
at 0.0004 deg/s yaw) contradicts directly. Cartographer does not check that
contradiction. This node does.

Architecture
------------
A node cannot suppress another node's TF broadcast, so Cartographer must stop
publishing map->odom and this node becomes the sole publisher:

    livox_2d_localization.lua:
        options.publish_to_tf        = false
        options.publish_tracked_pose = true   -- now load-bearing

    /scan,/odom -> cartographer_node -> /tracked_pose -> [this node] -> /tf

~base_frame must equal Cartographer's `tracking_frame` (not `published_frame`):
/tracked_pose carries the pose of the tracking frame in map.

Known limitation
----------------
A *persistent* wrong relocalization still passes the persistence override in
section 3.3 -- by construction, since on a 180-deg-symmetric map that
distinction does not exist in the sensor data. This node reduces flip
frequency; it does not make the map unambiguous. The durable fix is breaking
the map's symmetry physically and re-mapping. Treat this as defence in depth.
"""

import math
from collections import deque

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Int32, String
from std_srvs.srv import Empty, EmptyResponse
from tf.transformations import (concatenate_matrices, euler_from_quaternion,
                                inverse_matrix, quaternion_from_matrix,
                                quaternion_matrix, translation_from_matrix,
                                translation_matrix)


def wrap_pi(a):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(m):
    """Yaw of a 4x4 homogeneous transform."""
    return euler_from_quaternion(quaternion_from_matrix(m))[2]


def pose_to_matrix(pose):
    """geometry_msgs/Pose -> 4x4 homogeneous transform."""
    t = (pose.position.x, pose.position.y, pose.position.z)
    q = (pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w)
    return concatenate_matrices(translation_matrix(t), quaternion_matrix(q))


def transform_to_matrix(tf_msg):
    """geometry_msgs/TransformStamped -> 4x4 homogeneous transform."""
    t = (tf_msg.transform.translation.x,
         tf_msg.transform.translation.y,
         tf_msg.transform.translation.z)
    q = (tf_msg.transform.rotation.x, tf_msg.transform.rotation.y,
         tf_msg.transform.rotation.z, tf_msg.transform.rotation.w)
    return concatenate_matrices(translation_matrix(t), quaternion_matrix(q))


def delta(a, b):
    """(2D translation distance, |wrapped yaw difference|) between transforms."""
    ta, tb = translation_from_matrix(a), translation_from_matrix(b)
    dd = math.hypot(ta[0] - tb[0], ta[1] - tb[1])
    dth = abs(wrap_pi(yaw_of(a) - yaw_of(b)))
    return dd, dth


class JumpGuard(object):

    STATUS_UNLOCKED  = "UNLOCKED"
    STATUS_LOCKED    = "LOCKED"
    STATUS_REJECTING = "REJECTING"

    def __init__(self):
        # ── Frames ────────────────────────────────────────────────────────
        self.map_frame  = rospy.get_param("~map_frame",  "map")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        # Must match Cartographer's tracking_frame: /tracked_pose is the pose
        # of the *tracking* frame in map, not of published_frame.
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")

        # ── Gate thresholds (section 3.2) ─────────────────────────────────
        # Real corrections here are centimetre-scale; observed flips were
        # 4.2 m / 174 deg. Anything between 0.5 m and 3 m separates them
        # cleanly. Widen only if normal operation triggers rejections.
        self.max_d  = rospy.get_param("~max_jump_translation", 0.5)
        self.max_th = math.radians(rospy.get_param("~max_jump_rotation", 30.0))

        # ── Persistence override (section 3.3) ────────────────────────────
        self.persist_s  = rospy.get_param("~persistence_seconds", 5.0)
        self.persist_d  = rospy.get_param("~persistence_radius_m", 0.3)
        self.persist_th = math.radians(
            rospy.get_param("~persistence_radius_deg", 10.0))

        # ── Publishing ────────────────────────────────────────────────────
        self.rate      = rospy.get_param("~publish_rate", 100.0)
        self.tol       = rospy.Duration(
            rospy.get_param("~transform_tolerance", 0.1))
        self.pub_ident = rospy.get_param("~publish_identity_before_lock", True)

        # ── Reset behaviour ───────────────────────────────────────────────
        self.gap_reset_s   = rospy.get_param("~gap_reset_seconds", 5.0)
        # The odom->base_footprint TF arrives slightly AFTER the scan that
        # produced a given /tracked_pose (leg odometry is 2-50 ms behind on
        # this platform), so a look-up at the exact pose stamp is routinely a
        # hair into the future. 0.05 s caused a steady stream of "would require
        # extrapolation" skips; 0.2 s absorbs the lag without loosening the
        # gate itself.
        self.lookup_timeout = rospy.Duration(
            rospy.get_param("~lookup_timeout", 0.2))
        # Spec section 5: ~reset_lock clears `held`. That briefly republishes
        # identity, which yanks the nav stack; set false to keep publishing the
        # last good transform while still accepting the next candidate
        # unconditionally.
        self.reset_clears_held = rospy.get_param("~reset_clears_held", True)

        # ── State ─────────────────────────────────────────────────────────
        self.held       = None    # 4x4 matrix, currently published map->odom
        self.locked     = False
        self.rejects    = 0
        self.last_stamp = None    # stamp of the last /tracked_pose seen
        self.last_pub   = None    # stamp of the last TF broadcast

        # Consecutive rejected candidates that mutually agree. Bounded so a
        # faster-than-expected /tracked_pose cannot grow it without limit;
        # `pending_start` carries the span independently of the deque, so
        # dropping the oldest entries never defeats the override.
        self.pending = deque(maxlen=max(64, int(self.rate * self.persist_s)))
        self.pending_start = None

        self.warned_identity = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.bcaster = tf2_ros.TransformBroadcaster()

        self.status_pub = rospy.Publisher("~status", String, queue_size=1)
        self.count_pub  = rospy.Publisher("~rejection_count", Int32,
                                          queue_size=1)
        rospy.Service("~reset_lock", Empty, self.on_reset)
        rospy.Subscriber(rospy.get_param("~tracked_pose_topic",
                                         "/tracked_pose"),
                         PoseStamped, self.on_pose, queue_size=10)

        rospy.Timer(rospy.Duration(1.0 / self.rate), self.on_publish)
        rospy.Timer(rospy.Duration(1.0), self.on_status)

        rospy.loginfo(
            "jump_guard: %s -> %s gated via %s | gate %.2f m / %.1f deg | "
            "persistence %.1f s within %.2f m / %.1f deg",
            self.map_frame, self.odom_frame, self.base_frame,
            self.max_d, math.degrees(self.max_th), self.persist_s,
            self.persist_d, math.degrees(self.persist_th))

    # ── Pose handling ─────────────────────────────────────────────────────

    def on_pose(self, msg):
        stamp = msg.header.stamp

        # Edge cases 3 & 4: time ran backwards (bag replay / sim time reset) or
        # /tracked_pose went quiet long enough that Cartographer likely
        # restarted. Either way the old lock describes a different world.
        if self.last_stamp is not None:
            gap = (stamp - self.last_stamp).to_sec()
            if gap < 0.0:
                rospy.logwarn("jump_guard: stamp went backwards by %.3f s -- "
                              "resetting and unlocking", -gap)
                self.reset_state(clear_held=False)
            elif gap > self.gap_reset_s:
                rospy.logwarn("jump_guard: %.1f s gap in /tracked_pose -- "
                              "assuming restart, resetting and unlocking", gap)
                self.reset_state(clear_held=False)
        self.last_stamp = stamp

        # Look up at the pose's own timestamp, never Time(0): a mismatched
        # timestamp injects a spurious delta that looks exactly like a jump.
        try:
            tf_ob = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, stamp, self.lookup_timeout)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException, tf2_ros.TransformException) as e:
            # Edge case 2: not a rejection. Skip the candidate, keep
            # publishing `held`.
            rospy.logwarn_throttle(
                5.0, "jump_guard: %s->%s lookup failed (%s) -- skipping "
                     "candidate", self.odom_frame, self.base_frame, e)
            return

        # T_map_odom = T_map_base * inverse(T_odom_base)
        m_map_base  = pose_to_matrix(msg.pose)
        m_odom_base = transform_to_matrix(tf_ob)
        cand = concatenate_matrices(m_map_base, inverse_matrix(m_odom_base))

        # Edge case 5: the first candidate is accepted whatever its magnitude.
        # That is the intended cold-start relocalization landing.
        if not self.locked or self.held is None:
            rospy.loginfo("jump_guard: initial lock at (%.2f, %.2f, %.1f deg)",
                          translation_from_matrix(cand)[0],
                          translation_from_matrix(cand)[1],
                          math.degrees(yaw_of(cand)))
            self.accept(cand)
            return

        dd, dth = delta(cand, self.held)
        if dd <= self.max_d and dth <= self.max_th:
            self.accept(cand)          # normal small correction
        else:
            self.reject(cand, stamp, dd, dth)

    def accept(self, m):
        self.held = m
        self.locked = True
        self.pending.clear()
        self.pending_start = None

    def reject(self, m, stamp, dd, dth):
        """Hold the current transform, then run the persistence check."""
        self.rejects += 1
        rospy.logwarn(
            "jump_guard: rejected map->odom jump of %.2f m / %.1f deg "
            "(limits %.2f m / %.1f deg), rejection #%d",
            dd, math.degrees(dth), self.max_d, math.degrees(self.max_th),
            self.rejects)

        # A spurious flip is a single bad constraint that later constraints
        # disagree with; a genuine relocalization is asserted consistently by
        # every subsequent node. So: if the rejected candidates all agree with
        # each other and keep agreeing for long enough, Cartographer is not
        # flip-flopping -- it is telling us we are the ones who are wrong.
        if not all(self.agrees(m, prev) for prev in self.pending):
            # The assertion changed: this is flip-flopping, not persistence.
            # Start a fresh run anchored on the new candidate.
            self.pending.clear()
            self.pending_start = stamp

        if self.pending_start is None:
            self.pending_start = stamp
        self.pending.append(m)

        span = (stamp - self.pending_start).to_sec()
        if span > self.persist_s:
            rospy.logwarn(
                "jump_guard: PERSISTENCE OVERRIDE -- %d consecutive rejected "
                "candidates agreed within %.2f m / %.1f deg for %.1f s. "
                "Accepting; Cartographer is consistently asserting this pose.",
                len(self.pending), self.persist_d,
                math.degrees(self.persist_th), span)
            self.accept(m)

    def agrees(self, a, b):
        dd, dth = delta(a, b)
        return dd <= self.persist_d and dth <= self.persist_th

    # ── Publishing ────────────────────────────────────────────────────────

    def on_publish(self, _evt):
        """Republish `held` on a timer -- TF must not be allowed to go stale."""
        m = self.held
        if m is None:
            # Edge case 1: nothing locked yet.
            if not self.pub_ident:
                return
            if not self.warned_identity:
                rospy.logwarn("jump_guard: no /tracked_pose yet -- publishing "
                              "identity %s->%s to keep the TF tree complete",
                              self.map_frame, self.odom_frame)
                self.warned_identity = True
            m = np.identity(4)

        now = rospy.Time.now()
        if now.is_zero():
            return                  # use_sim_time before the first /clock
        if self.last_pub is not None and now <= self.last_pub:
            return                  # don't emit TF_REPEATED_DATA
        self.last_pub = now

        t = translation_from_matrix(m)
        q = quaternion_from_matrix(m)
        n = math.sqrt(sum(c * c for c in q))
        if n <= 0.0:
            return
        q = [c / n for c in q]      # matrix round-trips leave q slightly off

        msg = TransformStamped()
        # Forward-dated like AMCL, so consumers looking slightly ahead of now
        # don't fail their lookups.
        msg.header.stamp = now + self.tol
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.odom_frame
        msg.transform.translation.x = t[0]
        msg.transform.translation.y = t[1]
        msg.transform.translation.z = t[2]
        msg.transform.rotation.x = q[0]
        msg.transform.rotation.y = q[1]
        msg.transform.rotation.z = q[2]
        msg.transform.rotation.w = q[3]
        self.bcaster.sendTransform(msg)

    def on_status(self, _evt):
        if self.pending:
            status = self.STATUS_REJECTING
        elif self.locked:
            status = self.STATUS_LOCKED
        else:
            status = self.STATUS_UNLOCKED
        self.status_pub.publish(String(data=status))
        self.count_pub.publish(Int32(data=self.rejects))

    # ── Reset ─────────────────────────────────────────────────────────────

    def reset_state(self, clear_held):
        """Unlock so the next candidate is accepted unconditionally."""
        self.locked = False
        self.pending.clear()
        self.pending_start = None
        if clear_held:
            self.held = None

    def on_reset(self, _req):
        rospy.logwarn("jump_guard: lock reset by service call")
        self.reset_state(clear_held=self.reset_clears_held)
        self.last_stamp = None
        return EmptyResponse()


if __name__ == "__main__":
    rospy.init_node("localization_jump_guard")
    JumpGuard()
    rospy.spin()
