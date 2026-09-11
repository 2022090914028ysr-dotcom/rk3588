"""Stamped depth observations for NavOA's world-frame 2-D obstacle input."""
import threading
import numpy as np
import rospy
import tf
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2


class DepthObstacleInput:
    def __init__(self, listener, map_frame, topic='/obstacle_points', timeout=0.8):
        self.listener = listener
        self.map_frame = map_frame
        self.timeout = timeout
        self.lock = threading.Lock()
        self.observation = None
        self.sub = rospy.Subscriber(topic, PointCloud2, self.callback, queue_size=1,
                                    buff_size=4*1024*1024)

    def callback(self, msg):
        age = (rospy.Time.now()-msg.header.stamp).to_sec()
        if msg.header.stamp == rospy.Time(0) or not -0.1 <= age <= self.timeout:
            return
        try:
            self.listener.waitForTransform(self.map_frame, msg.header.frame_id,
                                           msg.header.stamp, rospy.Duration(0.2))
            trans, quat = self.listener.lookupTransform(self.map_frame, msg.header.frame_id,
                                                        msg.header.stamp)
            xyz = np.asarray(list(pc2.read_points(msg, field_names=('x', 'y', 'z'),
                                                  skip_nans=True)), dtype=np.float32).reshape(-1, 3)
            xyz = xyz[np.isfinite(xyz).all(axis=1)]
            rotation = tf.transformations.quaternion_matrix(quat)[:3, :3]
            world = xyz @ rotation.T + np.asarray(trans)
            if len(world):
                _, indices = np.unique(np.floor(world[:, :2]/0.05).astype(np.int64),
                                       axis=0, return_index=True)
                world = world[indices]
            with self.lock:
                self.observation = (msg.header.stamp, world[:, :2].T)
        except (tf.Exception, ValueError) as exc:
            rospy.logwarn_throttle(5.0, '[NavOA] Depth transform failed: %s', exc)

    def snapshot(self, now):
        with self.lock:
            observation = self.observation
        if observation is None:
            return None, False
        stamp, points = observation
        fresh = -0.1 <= (now-stamp).to_sec() <= self.timeout
        return (points if fresh else None), fresh


def merge_obstacles(lidar, depth):
    parts = [p for p in (lidar, depth) if p is not None and p.shape[1] > 0]
    return np.hstack(parts) if parts else None
