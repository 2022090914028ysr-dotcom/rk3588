#!/usr/bin/env python3
import math

import rospy
import tf
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from std_srvs.srv import Trigger, TriggerResponse
from cartographer_ros_msgs.srv import FinishTrajectory, FinishTrajectoryRequest, StartTrajectory, StartTrajectoryRequest


def quaternion_to_yaw(quat):
    return tf.transformations.euler_from_quaternion(
        [quat.x, quat.y, quat.z, quat.w]
    )[2]


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class CartographerResetService(object):
    def __init__(self):
        rospy.init_node('cartographer_reset_service')

        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.base_frame = rospy.get_param('~base_frame', 'base_footprint')
        self.configuration_directory = rospy.get_param('~configuration_directory', '')
        self.configuration_basename = rospy.get_param('~configuration_basename', '')
        self.use_initial_pose = rospy.get_param('~use_initial_pose', True)
        self.trajectory_id = rospy.get_param('~trajectory_id', 0)
        self.cartographer_namespace = rospy.get_param('~cartographer_namespace', '')
        self.start_after_finish = rospy.get_param('~start_after_finish', True)
        self.publish_initialpose = rospy.get_param('~publish_initialpose', True)
        self.initialpose_topic = rospy.get_param('~initialpose_topic', '/initialpose')

        self.tf_listener = tf.TransformListener()
        self.initialpose_pub = rospy.Publisher(self.initialpose_topic, PoseWithCovarianceStamped, queue_size=1)
        self.reset_service = rospy.Service('/cartographer_reset', Trigger, self.on_reset)

        rospy.loginfo('CartographerResetService ready: namespace=%s, trajectory_id=%s', self.cartographer_namespace, self.trajectory_id)
        rospy.spin()

    def on_reset(self, req):
        rospy.loginfo('Cartographer reset requested')
        if self.publish_initialpose:
            self.publish_initialpose_guess()

        finished = self.finish_trajectory()
        started = False
        if self.start_after_finish:
            started = self.start_trajectory()

        if finished or started or self.publish_initialpose:
            return TriggerResponse(success=True, message='Cartographer reset completed')

        return TriggerResponse(success=False, message='Cartographer reset did not perform any action')

    def finish_trajectory(self):
        service_name = self.service_name('finish_trajectory')
        try:
            rospy.wait_for_service(service_name, timeout=2.0)
            finish_srv = rospy.ServiceProxy(service_name, FinishTrajectory)
            req = FinishTrajectoryRequest()
            req.trajectory_id = self.trajectory_id
            resp = finish_srv(req)
            rospy.loginfo('finish_trajectory returned %s: %s', resp.status.code, resp.status.message)
            return resp.status.code == 0
        except Exception as ex:
            rospy.logwarn('Cartographer finish_trajectory failed: %s', ex)
            return False

    def start_trajectory(self):
        if not self.configuration_directory or not self.configuration_basename:
            rospy.logwarn('Cartographer start_trajectory skipped because configuration directory/basename is not set')
            return False

        service_name = self.service_name('start_trajectory')
        try:
            rospy.wait_for_service(service_name, timeout=2.0)
            start_srv = rospy.ServiceProxy(service_name, StartTrajectory)
            req = StartTrajectoryRequest()
            req.configuration_directory = self.configuration_directory
            req.configuration_basename = self.configuration_basename
            req.use_initial_pose = self.use_initial_pose
            req.initial_pose = self.seed_pose()
            req.relative_to_trajectory_id = self.trajectory_id
            resp = start_srv(req)
            rospy.loginfo('start_trajectory returned %s: %s trajectory_id=%s', resp.status.code, resp.status.message, resp.trajectory_id)
            return resp.status.code == 0
        except Exception as ex:
            rospy.logwarn('Cartographer start_trajectory failed: %s', ex)
            return False

    def service_name(self, name):
        if self.cartographer_namespace:
            return self.cartographer_namespace.rstrip('/') + '/' + name
        return '/' + name

    def publish_initialpose_guess(self):
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.map_frame
        pose.pose.pose = self.seed_pose()
        pose.pose.covariance = [0.0] * 36
        pose.pose.covariance[0] = 0.5
        pose.pose.covariance[7] = 0.5
        pose.pose.covariance[35] = 0.8
        self.initialpose_pub.publish(pose)
        rospy.loginfo('Published /initialpose from Cartographer reset service')

    def seed_pose(self):
        pose = Pose()
        try:
            self.tf_listener.waitForTransform(self.map_frame, self.base_frame, rospy.Time(0), rospy.Duration(0.5))
            trans, rot = self.tf_listener.lookupTransform(self.map_frame, self.base_frame, rospy.Time(0))
            pose.position.x = trans[0]
            pose.position.y = trans[1]
            pose.position.z = trans[2]
            pose.orientation.x = rot[0]
            pose.orientation.y = rot[1]
            pose.orientation.z = rot[2]
            pose.orientation.w = rot[3]
            rospy.logdebug('Seed pose from TF %s->%s', self.map_frame, self.base_frame)
        except Exception as ex:
            rospy.logwarn('Unable to get TF for seed pose: %s. Using zero pose.', ex)
            pose.position.x = 0.0
            pose.position.y = 0.0
            pose.position.z = 0.0
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0
        return pose


if __name__ == '__main__':
    try:
        CartographerResetService()
    except rospy.ROSInterruptException:
        pass
