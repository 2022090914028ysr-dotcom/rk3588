#!/usr/bin/env python3
import threading
import unittest
from unittest.mock import Mock, patch

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
import tf
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header

from obstacle_fusion import (DepthObstacleMemory, ObstacleFusion,
                             _deduplicate_nearest, _limit_nearest)


class ObstacleFusionTests(unittest.TestCase):
    def make_fusion(self):
        fusion = ObstacleFusion.__new__(ObstacleFusion)
        fusion.listener = Mock()
        fusion.planner_frame = 'map'
        fusion.base_frame = 'base_link'
        fusion.fusion_frame = 'odom'
        fusion.lidar_timeout = 0.25
        fusion.depth_timeout = 0.80
        fusion.grid_resolution = 0.05
        fusion.scan_range = (0.1, 5.0)
        fusion.scan_angle_range = (-3.14, 3.14)
        fusion.scan_downsample = 1
        fusion.flip_angle = False
        fusion.use_depth = True
        fusion.lock = threading.Lock()
        fusion.lidar_observation = None
        fusion.depth_observation = None
        fusion.clear_observation = None
        fusion.memory = None
        fusion.memory_depth_stamp = rospy.Time(0)
        fusion.memory_clear_stamp = rospy.Time(0)
        fusion.last_publish = rospy.Time(0)
        fusion.pub = Mock()

        def lookup(target, source, stamp):
            if (target, source) == ('odom', 'laser'):
                return (1, 0, 0), (0, 0, 0, 1)
            if (target, source) == ('odom', 'camera'):
                return (0, 1, 0), (0, 0, 0, 1)
            if (target, source) == ('odom', 'base_link'):
                return (0, 0, 0), (0, 0, 0, 1)
            if (target, source) == ('map', 'odom'):
                return (10, 0, 0), (0, 0, 0, 1)
            raise AssertionError((target, source, stamp))
        fusion.listener.lookupTransform.side_effect = lookup
        return fusion

    def test_both_sensors_use_their_exact_acquisition_stamp(self):
        fusion = self.make_fusion()
        stamp = rospy.Time(100)
        scan = LaserScan(header=Header(stamp=stamp, frame_id='laser'),
                         angle_min=0.0, angle_increment=1.0,
                         range_min=0.1, range_max=10.0,
                         ranges=[1.0, float('nan')])
        depth = pc2.create_cloud_xyz32(
            Header(stamp=stamp, frame_id='camera'), [[1.0, 0.0, 0.3]])
        with patch('rospy.Time.now', return_value=rospy.Time(100, 100000000)):
            fusion.lidar_callback(scan)
            fusion.depth_callback(depth)
        calls = fusion.listener.lookupTransform.call_args_list
        self.assertEqual(calls[0].args, ('odom', 'laser', stamp))
        self.assertEqual(calls[1].args, ('odom', 'camera', stamp))
        points, status = fusion.snapshot(rospy.Time(100, 200000000))
        self.assertTrue(status['lidar_fresh'])
        self.assertTrue(status['depth_fresh'])
        # lidar=(2,0) in odom and depth=(1,1), then current odom->map +10 x.
        np.testing.assert_allclose(
            points[:, np.argsort(points[1])], [[12.0, 11.0], [0.0, 1.0]], atol=1e-5)

    def test_staleness_is_independent(self):
        fusion = self.make_fusion()
        fusion.lidar_observation = (rospy.Time(10), np.array([[1.0], [0.0]]))
        fusion.depth_observation = (rospy.Time(9), np.array([[2.0], [0.0]]))
        points, status = fusion.snapshot(rospy.Time(10, 200000000))
        self.assertTrue(status['lidar_fresh'])
        self.assertFalse(status['depth_fresh'])
        np.testing.assert_allclose(points, [[11.0], [0.0]])

    def test_grid_keeps_nearest_point(self):
        points = np.array([[1.04, 1.01, 2.0], [0.01, 0.02, 0.0]])
        result = _deduplicate_nearest(points, 0.05, np.array([0.0, 0.0]))
        self.assertEqual(result.shape, (2, 2))
        self.assertTrue(any(np.allclose(p, [1.01, 0.02]) for p in result.T))
        self.assertFalse(any(np.allclose(p, [1.04, 0.01]) for p in result.T))

    def test_point_limit_preserves_closest_hazards(self):
        points = np.array([[3.0, 0.7, 2.0, 0.4], [0.0, 0.1, 0.0, 0.0]])
        result = _limit_nearest(points, np.array([0.0, 0.0]), 2)
        np.testing.assert_allclose(result, [[0.4, 0.7], [0.0, 0.1]])

    def test_empty_depth_replaces_depth_without_clearing_lidar(self):
        fusion = self.make_fusion()
        stamp = rospy.Time(100)
        fusion.lidar_observation = (stamp, np.array([[2.0], [0.0]]))
        fusion.depth_observation = (stamp, np.array([[1.0], [1.0]]))
        # A valid newer frame containing no obstacle is explicit free evidence
        # for depth only. The independent lidar state must survive.
        fusion.depth_observation = (
            rospy.Time(100, 100000000), np.empty((2, 0), dtype=np.float32))
        points, status = fusion.snapshot(rospy.Time(100, 200000000))
        self.assertTrue(status['lidar_fresh'])
        self.assertTrue(status['depth_fresh'])
        np.testing.assert_allclose(points, [[12.0], [0.0]])

    def test_planner_transform_failure_is_reported(self):
        fusion = self.make_fusion()
        fusion.lidar_observation = (rospy.Time(10), np.array([[1.0], [0.0]]))
        fusion.listener.lookupTransform.side_effect = tf.Exception('missing map to odom')
        with patch('rospy.logwarn_throttle'):
            points, status = fusion.snapshot(rospy.Time(10, 100000000))
        self.assertIsNone(points)
        self.assertFalse(status['transform_valid'])

    def test_memory_survives_camera_turn_until_ttl(self):
        memory = DepthObstacleMemory(resolution=0.05, ttl=4.0, radius=3.0)
        memory.add(np.array([[1.0], [0.0]]), rospy.Time(10))
        self.assertEqual(memory.points(rospy.Time(13), [0.0, 0.0]).shape, (2, 1))
        self.assertIsNone(memory.points(rospy.Time(14, 100000000), [0.0, 0.0]))

    def test_only_measured_rays_clear_memory(self):
        memory = DepthObstacleMemory(resolution=0.05, ttl=4.0, radius=3.0)
        memory.add(np.array([[1.0, 1.0], [0.0, 1.0]]), rospy.Time(10))
        # The first cell is on the measured ray; the off-axis cell is outside
        # the visible ray and must remain remembered.
        memory.clear_rays(np.array([0.0, 0.0]), np.array([[2.0], [0.0]]))
        points = memory.points(rospy.Time(11), [0.0, 0.0])
        np.testing.assert_allclose(points, [[1.0], [1.0]])


if __name__ == '__main__':
    unittest.main(verbosity=2)
