#!/usr/bin/env python3
import threading
import unittest
from unittest.mock import Mock, patch
import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from depth_obstacle_input import DepthObstacleInput, merge_obstacles


class DepthInputTests(unittest.TestCase):
    def setUp(self):
        self.input = DepthObstacleInput.__new__(DepthObstacleInput)
        self.input.listener = Mock()
        self.input.listener.lookupTransform.return_value = ((5, -3, .6), (-.5, .5, -.5, .5))
        self.input.map_frame = 'map'
        self.input.timeout = .8
        self.input.lock = threading.Lock()
        self.input.observation = None

    def test_full_optical_transform_and_exact_acquisition_stamp(self):
        stamp = rospy.Time(100)
        msg = pc2.create_cloud_xyz32(Header(stamp=stamp, frame_id='optical'), [[.2, .3, 1.2]])
        with patch('rospy.Time.now', return_value=rospy.Time(100, 100000000)):
            self.input.callback(msg)
        self.input.listener.lookupTransform.assert_called_once_with('map', 'optical', stamp)
        points, fresh = self.input.snapshot(rospy.Time(100, 200000000))
        self.assertTrue(fresh)
        np.testing.assert_allclose(points, [[6.2], [-3.2]])
        self.assertEqual(self.input.snapshot(rospy.Time(101)), (None, False))

    def test_empty_valid_observation_and_merge_do_not_erase_lidar(self):
        msg = pc2.create_cloud_xyz32(Header(stamp=rospy.Time(100), frame_id='optical'), [])
        with patch('rospy.Time.now', return_value=rospy.Time(100)):
            self.input.callback(msg)
        depth, fresh = self.input.snapshot(rospy.Time(100))
        self.assertTrue(fresh)
        lidar = np.array([[1], [2]])
        np.testing.assert_array_equal(merge_obstacles(lidar, depth), lidar)
        np.testing.assert_array_equal(merge_obstacles(None, lidar), lidar)

    def test_uninitialized_or_future_data_is_not_fresh(self):
        self.assertEqual(self.input.snapshot(rospy.Time(100)), (None, False))
        msg = pc2.create_cloud_xyz32(Header(stamp=rospy.Time(200), frame_id='optical'), [[0, 0, 1]])
        with patch('rospy.Time.now', return_value=rospy.Time(100)):
            self.input.callback(msg)
        self.assertIsNone(self.input.observation)


if __name__ == '__main__':
    unittest.main(verbosity=2)
