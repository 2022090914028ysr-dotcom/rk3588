#!/usr/bin/env python3
"""Regression tests; no ROS master or hardware is required."""
import struct
import unittest
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from ground_segmentation_node import read_xyz, make_cloud, ground_mask, spatial_mask, TemporalFilter


class FilterTests(unittest.TestCase):
    def test_sensor_stamp_frame_roundtrip_and_empty(self):
        h = Header(stamp=rospy.Time(123, 45), frame_id='camera_optical')
        cloud = make_cloud(h, [[1, 2, 3]])
        self.assertEqual(cloud.header, h)
        np.testing.assert_array_equal(read_xyz(cloud), [[1, 2, 3]])
        self.assertEqual(read_xyz(make_cloud(h, [])).shape, (0, 3))

    def test_padded_big_endian_cloud(self):
        data = b''.join(struct.pack('>ffff', 99, x, x+1, x+2)+b'pad!' for x in (1, 4))
        cloud = PointCloud2(height=2, width=1, point_step=16, row_step=20,
                            fields=[PointField(n, 4+4*i, PointField.FLOAT32, 1)
                                    for i, n in enumerate(('x', 'y', 'z'))],
                            is_bigendian=True, data=data)
        np.testing.assert_array_equal(read_xyz(cloud), [[1, 2, 3], [4, 5, 6]])

    def test_invalid_pixels_never_become_free_rays(self):
        cloud = make_cloud(Header(), [[1, 2, 3], [np.nan, 0, 0], [np.inf, 0, 0]])
        self.assertEqual(len(read_xyz(cloud)), 1)

    def test_unorganized_cloud_uses_equivalent_2d_stride(self):
        points = [[i, 0, 0] for i in range(16)]
        cloud = make_cloud(Header(), points)
        sampled = read_xyz(cloud, stride=2)
        # A 2x2 image stride retains one in four pixels even though the camera
        # driver flattens the cloud to height=1.
        np.testing.assert_array_equal(sampled[:, 0], [0, 4, 8, 12])

    def test_floor_removed_low_box_preserved(self):
        x, y = np.meshgrid(np.linspace(.4, 2, 25), np.linspace(-1, 1, 25))
        floor = np.c_[x.ravel(), y.ravel(), np.zeros(x.size)]
        box = np.c_[x.ravel(), y.ravel(), np.full(x.size, .08)]
        mask = ground_mask(np.vstack([floor, box]))
        self.assertTrue(mask[:len(floor)].all())
        self.assertFalse(mask[len(floor):].any())
        self.assertFalse(ground_mask(box).any())

    def test_vertical_plane_not_ground(self):
        y, z = np.meshgrid(np.linspace(-1, 1, 40), np.linspace(.055, .09, 30))
        wall = np.c_[np.ones(y.size), y.ravel(), z.ravel()]
        self.assertFalse(ground_mask(wall).any())

    def test_negative_cells_do_not_merge_across_origin(self):
        points = np.array([[-.001, 0, .2], [.001, 0, .2], [.002, 0, .2]])
        self.assertEqual(spatial_mask(points, .05, 2).tolist(), [False, True, True])

    def test_temporal_requires_all_frames_and_empty_breaks_history(self):
        filt = TemporalFilter(3, .1)
        p = np.array([[-.3, -.4, .2]])
        self.assertFalse(filt.mask(p)[0])
        self.assertFalse(filt.mask(p)[0])
        self.assertTrue(filt.mask(p)[0])
        filt.mask(np.empty((0, 3)))
        self.assertFalse(filt.mask(p)[0])

    def test_temporal_does_not_accept_distant_new_obstacle(self):
        filt = TemporalFilter(2, .1)
        filt.mask(np.array([[1, 0, .2]]))
        self.assertFalse(filt.mask(np.array([[2, 0, .2]]))[0])


if __name__ == '__main__':
    unittest.main(verbosity=2)
