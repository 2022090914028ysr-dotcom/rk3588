#!/usr/bin/env python3
"""
Persistent node: keeps the leg odometry (SMX/Odom) origin aligned with rf2o.

Publishes a pose reset to /fusion_estimator_node/set_pose in two situations:

  1. Startup   — after a delay, publish rf2o's CURRENT pose (NOT identity) so the
     leg starts aligned to rf2o even if the robot has already moved/turned before
     this 3 s delay fires.  Zeroing to identity assumes rf2o is still at yaw 0,
     which is only true if the robot is stationary — a real v2 bag showed the leg
     ~40° off rf2o for the first ~100 s because the robot had turned before the
     reset.  Falls back to identity only if rf2o is not fresh yet.
  2. Reconnect — if SMX/Odom drops out (leg node restart / network blip) and then
     resumes, publish rf2o's CURRENT pose so the leg re-anchors to where rf2o
     actually is.  The robot may have turned since startup, so re-zeroing to
     identity mid-run would corrupt absolute yaw; rf2o's current pose is correct.

The publisher is latched and this node stays alive, so a fusion_estimator_node
that (re)subscribes after this node published still receives the last reset pose.
A latched message only survives while this node is running — which is why this
node must NOT be one-shot.

Assumes the remote PC (192.168.110.132) is up — fusion_estimator_node must be
running to receive the reset.

Usage (roslaunch):
  <node pkg="NavOA_ros" type="align_leg_origin.py" name="align_leg_origin"
        output="screen" required="false">
    <param name="delay_s"         value="3.0" />
    <param name="set_pose_topic"  value="/fusion_estimator_node/set_pose" />
    <param name="leg_topic"       value="SMX/Odom" />
    <param name="rf2o_topic"      value="/odom_rf2o" />
    <param name="dropout_timeout" value="1.0" />
  </node>
"""

import rospy
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


class AlignLegOrigin:
    def __init__(self):
        self.delay_s = rospy.get_param("~delay_s", 3.0)
        self.set_pose_topic = rospy.get_param(
            "~set_pose_topic", "/fusion_estimator_node/set_pose")
        leg_topic = rospy.get_param("~leg_topic", "SMX/Odom")
        rf2o_topic = rospy.get_param("~rf2o_topic", "/odom_rf2o")
        self.dropout_timeout = rospy.get_param("~dropout_timeout", 1.0)

        self.pub = rospy.Publisher(self.set_pose_topic, PoseWithCovarianceStamped,
                                   queue_size=1, latch=True)

        # Latest rf2o pose (geometry_msgs/Pose) + receive time, for re-anchoring.
        self.latest_rf2o = None
        self.last_rf2o_time = None

        # Leg (SMX/Odom) drop-out tracking.
        self.last_leg_time = None
        self.have_seen_leg = False
        self.leg_dropped = False

        self.rf2o_sub = rospy.Subscriber(rf2o_topic, Odometry, self._rf2o_cb)
        self.leg_sub = rospy.Subscriber(leg_topic, Odometry, self._leg_cb)

        # Startup reset fires once after the remote estimator has booted.
        rospy.Timer(rospy.Duration(self.delay_s), self._startup_cb, oneshot=True)
        # Drop-out monitor runs at 5 Hz.
        rospy.Timer(rospy.Duration(0.2), self._monitor_cb)

    # ---- callbacks ---------------------------------------------------------
    def _rf2o_cb(self, msg):
        self.latest_rf2o = msg.pose.pose
        self.last_rf2o_time = rospy.Time.now()

    def _leg_cb(self, msg):
        now = rospy.Time.now()
        if self.have_seen_leg and self.leg_dropped:
            # First message after a drop-out -> the leg restarted with a new
            # arbitrary origin; re-anchor it to rf2o's current pose (NOT identity).
            if self.latest_rf2o is not None and self._rf2o_fresh():
                rospy.loginfo("Leg odometry reconnected — re-aligning to rf2o pose")
                self._publish(self.latest_rf2o)
            else:
                rospy.logwarn("Leg reconnected but rf2o pose is unavailable/stale — "
                              "skipping re-align")
        self.have_seen_leg = True
        self.leg_dropped = False
        self.last_leg_time = now

    def _startup_cb(self, event):
        # Align to rf2o's CURRENT pose, not identity.  The robot may have already
        # turned before this delay fires, so rf2o is at some non-zero yaw; zeroing
        # the leg to identity would leave it that many degrees off (v2 bag: ~40°).
        if self.latest_rf2o is not None and self._rf2o_fresh():
            rospy.loginfo("Leg odometry aligned to rf2o pose at startup on %s",
                          self.set_pose_topic)
            self._publish(self.latest_rf2o)
        else:
            identity = Pose()
            identity.orientation.w = 1.0  # position (0,0,0), orientation (0,0,0,1) = yaw 0
            rospy.logwarn("rf2o pose unavailable/stale at startup — zeroing leg to identity")
            self._publish(identity)

    def _monitor_cb(self, event):
        if not self.have_seen_leg or self.last_leg_time is None:
            return
        gap = (rospy.Time.now() - self.last_leg_time).to_sec()
        if gap > self.dropout_timeout:
            self.leg_dropped = True

    # ---- helpers -----------------------------------------------------------
    def _rf2o_fresh(self):
        if self.last_rf2o_time is None:
            return False
        return (rospy.Time.now() - self.last_rf2o_time).to_sec() < self.dropout_timeout

    def _publish(self, pose):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "odom"
        msg.header.stamp = rospy.Time.now()
        msg.pose.pose = pose
        # Covariance left zero (matches the original one-shot behavior: the remote
        # fusion_estimator treats a set_pose as a hard reset).
        self.pub.publish(msg)


def main():
    rospy.init_node("align_leg_origin")
    AlignLegOrigin()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
