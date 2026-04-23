#!/usr/bin/env python3
import os
import math
import subprocess

import rospy
import tf
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from std_srvs.srv import Trigger, TriggerResponse, SetBool, SetBoolResponse


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
        self.normal_configuration_basename = rospy.get_param('~normal_configuration_basename', rospy.get_param('~configuration_basename', 'limo_2d.lua'))
        self.relocalization_configuration_basename = rospy.get_param('~relocalization_configuration_basename', 'limo_2d_reloc.lua')
        self.cartographer_launch = rospy.get_param('~cartographer_launch', '')
        self.cartographer_namespace = rospy.get_param('~cartographer_namespace', '')
        self.cartographer_node_name = rospy.get_param('~cartographer_node_name', '/cartographer_node')
        self.cartographer_occupancy_node_name = rospy.get_param('~cartographer_occupancy_node_name', '/cartographer_occupancy_grid_node')
        self.use_initial_pose = rospy.get_param('~use_initial_pose', True)
        self.relocalization_hold = rospy.get_param('~relocalization_hold', 15.0)
        self.publish_initialpose = rospy.get_param('~publish_initialpose', True)
        self.initialpose_topic = rospy.get_param('~initialpose_topic', '/initialpose')
        self.current_mode = 'normal'
        self.relocalization_deadline = None
        self.cartographer_proc = None

        self.tf_listener = tf.TransformListener()
        self.initialpose_pub = rospy.Publisher(self.initialpose_topic, PoseWithCovarianceStamped, queue_size=1)
        self.reset_service = rospy.Service('/cartographer_reset', Trigger, self.on_reset)
        self.mode_service = rospy.Service('/cartographer_set_mode', SetBool, self.on_set_mode)
        self.timer = rospy.Timer(rospy.Duration(1.0), self.timer_callback)

        rospy.loginfo('CartographerResetService ready: namespace=%s, normal_config=%s, relocalization_config=%s, cartographer_launch=%s',
                      self.cartographer_namespace,
                      self.normal_configuration_basename, self.relocalization_configuration_basename,
                      self.cartographer_launch)
        rospy.spin()

    def on_reset(self, req):
        rospy.loginfo('Cartographer reset requested')
        if self.publish_initialpose:
            self.publish_initialpose_guess()

        success = self.restart_cartographer(self.relocalization_configuration_basename)
        if success:
            return TriggerResponse(success=True, message='Cartographer reset completed and relocalization restarted')
        return TriggerResponse(success=False, message='Cartographer reset failed to restart Cartographer')

    def start_relocalization_mode(self):
        rospy.loginfo('Starting Cartographer relocalization mode')
        if self.current_mode == 'relocalization':
            rospy.loginfo('Already in relocalization mode')
            return True

        success = self.restart_cartographer(self.relocalization_configuration_basename)
        if success:
            self.current_mode = 'relocalization'
            self.relocalization_deadline = rospy.Time.now() + rospy.Duration(self.relocalization_hold)
            rospy.loginfo('Cartographer relocalization mode active for %.1fs', self.relocalization_hold)
        return success

    def switch_to_normal_mode(self):
        if self.current_mode == 'normal':
            return True
        rospy.loginfo('Switching Cartographer back to normal mode')
        success = self.restart_cartographer(self.normal_configuration_basename)
        if success:
            self.current_mode = 'normal'
            self.relocalization_deadline = None
        return success

    def on_set_mode(self, req):
        if req.data:
            success = self.start_relocalization_mode()
            msg = 'relocalization mode enabled' if success else 'failed to enable relocalization mode'
        else:
            success = self.switch_to_normal_mode()
            msg = 'normal mode enabled' if success else 'failed to enable normal mode'
        return SetBoolResponse(success=success, message=msg)

    def timer_callback(self, event):
        if self.current_mode == 'relocalization' and self.relocalization_deadline is not None:
            if rospy.Time.now() >= self.relocalization_deadline:
                self.switch_to_normal_mode()

    def restart_cartographer(self, config_basename):
        if not self.cartographer_launch:
            rospy.logwarn('Cartographer restart skipped because cartographer_launch is not configured')
            return False

        self.kill_node(self.cartographer_node_name)
        self.kill_node(self.cartographer_occupancy_node_name)
        rospy.sleep(0.5)

        if self.cartographer_proc is not None and self.cartographer_proc.poll() is None:
            rospy.loginfo('Terminating previous cartographer launch process')
            self.cartographer_proc.terminate()
            try:
                self.cartographer_proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.cartographer_proc.kill()

        cmd = [
            'roslaunch',
            self.cartographer_launch,
            'configuration_basename:=' + config_basename,
        ]
        rospy.loginfo('Starting Cartographer launch: %s', ' '.join(cmd))
        try:
            self.cartographer_proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=os.environ.copy())
            rospy.loginfo('Cartographer restart command launched with pid=%s', self.cartographer_proc.pid)
        except Exception as ex:
            rospy.logwarn('Failed to launch Cartographer process: %s', ex)
            return False
        return True

    def kill_node(self, node_name):
        try:
            subprocess.check_call(['rosnode', 'kill', node_name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            rospy.loginfo('Killed ROS node %s', node_name)
            return True
        except subprocess.CalledProcessError:
            rospy.logdebug('ROS node %s was not running or could not be killed', node_name)
        except Exception as ex:
            rospy.logwarn('Failed to kill ROS node %s: %s', node_name, ex)
        return False

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
