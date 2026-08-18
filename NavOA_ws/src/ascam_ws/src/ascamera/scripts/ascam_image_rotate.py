#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from sensor_msgs.msg import Image

# encoding -> (numpy dtype, channel count)
ENCODINGS = {
    "mono8": (np.uint8, 1),
    "8UC1": (np.uint8, 1),
    "bgr8": (np.uint8, 3),
    "rgb8": (np.uint8, 3),
    "16UC1": (np.uint16, 1),
    "32FC1": (np.float32, 1),
}

# rotation name -> np.rot90 k (k=1 is CCW, so 90cw -> k=-1)
ROTATIONS = {
    "90cw": -1,
    "90ccw": 1,
    "180": 2,
    "180cw": 2,   # aliases: 180deg is identical in either direction
    "180ccw": 2,
}


def img_to_numpy(msg):
    """Decode a sensor_msgs/Image into an (H, W[, C]) numpy array."""
    info = ENCODINGS.get(msg.encoding)
    if info is None:
        raise ValueError("Unsupported image encoding: %s" % msg.encoding)
    dtype, channels = info
    arr = np.frombuffer(msg.data, dtype=dtype)
    if channels > 1:
        arr = arr.reshape(msg.height, msg.width, channels)
    else:
        arr = arr.reshape(msg.height, msg.width)
    return arr


def numpy_to_img(arr, header, encoding):
    out = Image()
    out.header = header
    out.height = arr.shape[0]
    out.width = arr.shape[1]
    out.encoding = encoding
    out.is_bigendian = 0
    if arr.ndim == 3:
        out.step = out.width * arr.shape[2] * arr.itemsize
    else:
        out.step = out.width * arr.itemsize
    out.data = np.ascontiguousarray(arr).tobytes()
    return out


class ImageRotator(object):
    def __init__(self):
        rot_name = rospy.get_param("~rotation", "90cw")
        if rot_name not in ROTATIONS:
            rospy.logwarn("Unknown rotation '%s', falling back to '90cw'", rot_name)
            rot_name = "90cw"
        self.k = ROTATIONS[rot_name]

        rgb_in = rospy.get_param("~rgb_in", "/ascamera_hp60c/rgb0/image")
        rgb_out = rospy.get_param("~rgb_out", "/ascamera_hp60c/rgb0/image_rotated")
        depth_in = rospy.get_param("~depth_in", "/ascamera_hp60c/depth0/image_raw")
        depth_out = rospy.get_param("~depth_out", "/ascamera_hp60c/depth0/image_rotated")

        self.pub_rgb = rospy.Publisher(rgb_out, Image, queue_size=10)
        self.pub_depth = rospy.Publisher(depth_out, Image, queue_size=10)
        rospy.Subscriber(rgb_in, Image, self.cb_rgb)
        rospy.Subscriber(depth_in, Image, self.cb_depth)

        rospy.loginfo("ascam_image_rotate: rotation=%s", rot_name)
        rospy.loginfo("  rgb:   %s -> %s", rgb_in, rgb_out)
        rospy.loginfo("  depth: %s -> %s", depth_in, depth_out)

    def _rotate(self, msg):
        arr = img_to_numpy(msg)
        arr = np.rot90(arr, k=self.k, axes=(0, 1))
        return numpy_to_img(arr, msg.header, msg.encoding)

    def cb_rgb(self, msg):
        self.pub_rgb.publish(self._rotate(msg))

    def cb_depth(self, msg):
        self.pub_depth.publish(self._rotate(msg))


if __name__ == "__main__":
    rospy.init_node("ascam_image_rotate")
    ImageRotator()
    rospy.spin()
