#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Ephemeral global-costmap marking for NeuPAN bypass deadlocks.

The global costmap normally holds only the static map; the GlobalPlanner
therefore replans through a low obstacle that only the depth camera
sees, and the robot re-commits to the same blocked corridor.  When the
bypass recovery in NavOA detects a deadlock it publishes a trigger on
``/navoa/bypass_trigger``; this node then:

  1. replays the live fused obstacle points (already lidar+depth merged
     and world-frame correct) onto ``/navoa/bypass_obstacle_marks`` so
     the global costmap's bypass ObstacleLayer keeps them marked
     (marking-only, no raytrace side effects);
  2. keeps marking until the robot has actually detoured PAST the
     blockage — move_base replans at 1 Hz and looks at the current
     costmap every time, so clearing on a fixed timer would flip the
     global route right back into the obstacle and the robot would
     oscillate between two routes;
  3. once the robot is clear (or an absolute timeout expires), publishes
     one omnidirectional clearing ring on ``/navoa/bypass_obstacle_clears``
     (frame: base_link, so the layer's raytraces start at the robot) to
     erase the temporary marks.
"""
import math

import rospy
import tf
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header, String


def should_clear_marks(elapsed, robot_distance,
                       min_duration=3.0, max_duration=25.0,
                       departure_distance=1.5):
    """Pure clearing policy, unit-testable without ROS.

    elapsed: seconds since the last trigger
    robot_distance: robot's distance to the nearest marked point, or
        None when unknown (no TF yet / no marks published)
    Returns (clear: bool, reason: str).
    """
    if elapsed >= max_duration:
        return True, 'max-duration(%.0fs)' % max_duration
    if elapsed < min_duration:
        return False, 'min-duration'
    if robot_distance is None:
        return False, 'no-robot-distance'
    if robot_distance >= departure_distance:
        return True, 'robot-departed(%.2fm)' % robot_distance
    return False, 'robot-near-marks(%.2fm)' % robot_distance


class BypassObstaclePublisher:
    def __init__(self):
        self.fused_topic = rospy.get_param(
            '~fused_obstacle_topic', '/navoa/fused_obstacle_points')
        self.trigger_topic = rospy.get_param(
            '~bypass_trigger_topic', '/navoa/bypass_trigger')
        self.mark_topic = rospy.get_param(
            '~bypass_mark_topic', '/navoa/bypass_obstacle_marks')
        self.clear_topic = rospy.get_param(
            '~bypass_clear_topic', '/navoa/bypass_obstacle_clears')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.map_frame = rospy.get_param('~map_frame', 'map')

        # Clearing policy: the marks stay while the robot is still near
        # the blockage (1 Hz replanning keeps routing around it), and
        # are erased once the robot has detoured past it.
        self.min_duration = float(rospy.get_param('~mark_min_duration', 3.0))
        self.max_duration = float(rospy.get_param('~mark_max_duration', 25.0))
        self.departure_distance = float(
            rospy.get_param('~clear_departure_distance', 1.5))
        self.mark_rate = float(rospy.get_param('~mark_rate', 2.0))
        # Only points near the robot are marked; distant walls only
        # shrink the planner's options without helping the deadlock.
        self.mark_radius = float(rospy.get_param('~mark_radius', 2.0))
        # Clearing ring geometry; raytrace_range on the costmap side
        # must cover ring_radius.
        self.ring_radius = float(rospy.get_param('~clear_ring_radius', 3.5))
        self.ring_points = int(rospy.get_param('~clear_ring_points', 144))

        self.fused_cloud = None
        self.marking = False
        self.mark_start = rospy.Time(0)
        self.last_mark = rospy.Time(0)
        self.mark_points_xy = None   # (N, 2) last marked points, map frame

        self.tf_listener = tf.TransformListener()
        self.mark_pub = rospy.Publisher(
            self.mark_topic, PointCloud2, queue_size=1)
        self.clear_pub = rospy.Publisher(
            self.clear_topic, PointCloud2, queue_size=1)
        rospy.Subscriber(
            self.fused_topic, PointCloud2, self.fused_callback,
            queue_size=1, buff_size=2 * 1024 * 1024)
        rospy.Subscriber(
            self.trigger_topic, String, self.trigger_callback, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)
        rospy.loginfo(
            "[BypassObstacles] ready: trigger=%s marks=%s clears=%s "
            "(clear when departed>%.1fm, max %.0fs)",
            self.trigger_topic, self.mark_topic, self.clear_topic,
            self.departure_distance, self.max_duration)

    def fused_callback(self, msg):
        self.fused_cloud = msg

    def trigger_callback(self, _msg):
        now = rospy.Time.now()
        if self.marking:
            # A renewed deadlock while already marking extends the
            # window instead of resetting the departure clock.
            rospy.loginfo(
                "[BypassObstacles] retrigger while marking; window extended")
        else:
            self.marking = True
            self.mark_start = now
            self.mark_points_xy = None
        rospy.loginfo(
            "[BypassObstacles] deadlock trigger: marking fused obstacles "
            "until the robot detours past them")

    def control_loop(self, _event):
        if not self.marking:
            return
        now = rospy.Time.now()

        # Keep replaying the latest fused cloud so the layer's marks
        # never go stale while the robot works the blockage.
        if (self.fused_cloud is not None
                and (now - self.last_mark).to_sec() >= 1.0 / self.mark_rate):
            self.last_mark = now
            self.publish_marks()

        elapsed = (now - self.mark_start).to_sec()
        clear, reason = should_clear_marks(
            elapsed, self._robot_distance_to_marks(),
            min_duration=self.min_duration,
            max_duration=self.max_duration,
            departure_distance=self.departure_distance)
        if clear:
            self.publish_clearing_ring()
            self.marking = False
            self.mark_points_xy = None
            rospy.loginfo(
                "[BypassObstacles] marking window closed (%s); cleared the "
                "ephemeral global obstacles", reason)

    def publish_marks(self):
        cloud = self.fused_cloud
        if cloud is None or cloud.width == 0:
            return
        frame = cloud.header.frame_id or 'map'
        robot = self._robot_position_in(frame)
        xyz = []
        for x, y, z in pc2.read_points(
                cloud, field_names=('x', 'y', 'z'), skip_nans=True):
            if robot is not None:
                if math.hypot(x - robot[0], y - robot[1]) > self.mark_radius:
                    continue
            xyz.append((x, y, z))
        if not xyz:
            return
        header = Header(stamp=rospy.Time.now(), frame_id=frame)
        self.mark_pub.publish(pc2.create_cloud_xyz32(header, xyz))
        self.mark_frame = frame
        self.mark_points = xyz

    def _robot_position_in(self, frame):
        """Robot (x, y) in the given frame via TF, or None."""
        try:
            trans, _rot = self.tf_listener.lookupTransform(
                frame, self.base_frame, rospy.Time(0))
            return trans[0], trans[1]
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            return None

    def _robot_distance_to_marks(self):
        """Distance from the robot to the nearest marked point, or None."""
        if not getattr(self, 'mark_points', None):
            return None
        robot = self._robot_position_in(self.mark_frame)
        if robot is None:
            return None
        best = None
        for x, y, _z in self.mark_points:
            d = math.hypot(x - robot[0], y - robot[1])
            if best is None or d < best:
                best = d
        return best

    def publish_clearing_ring(self):
        """One omnidirectional ring in base_link: the layer raytraces
        from the robot outward and erases every ephemeral mark."""
        header = Header(stamp=rospy.Time.now(), frame_id=self.base_frame)
        ring = [(self.ring_radius * math.cos(a),
                 self.ring_radius * math.sin(a), 0.0)
                for a in [2.0 * math.pi * i / self.ring_points
                          for i in range(self.ring_points)]]
        self.clear_pub.publish(pc2.create_cloud_xyz32(header, ring))


def np_xy(_xyz):
    # Kept for symmetry; unused placeholder removed from logic.
    return None


if __name__ == '__main__':
    try:
        rospy.init_node('bypass_obstacle_publisher')
        BypassObstaclePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
