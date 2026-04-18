#!/usr/bin/env python
from __future__ import print_function
import math
import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse


def quaternion_to_yaw(quat):
    return tf.transformations.euler_from_quaternion(
        [quat.x, quat.y, quat.z, quat.w]
    )[2]


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class LocalizationRecoveryNode(object):
    def __init__(self):
        rospy.init_node('localization_recovery')

        # Detection thresholds
        self.position_jump_threshold = rospy.get_param('~position_jump_threshold', 1.0)
        self.rotation_jump_threshold = rospy.get_param('~rotation_jump_threshold', 1.0)
        self.imu_accel_spike_threshold = rospy.get_param('~imu_accel_spike_threshold', 15.0)
        self.imu_delta_accel_threshold = rospy.get_param('~imu_delta_accel_threshold', 7.0)
        self.odom_inconsistency_threshold = rospy.get_param('~odom_inconsistency_threshold', 1.5)
        self.scan_timeout = rospy.get_param('~scan_timeout', 2.0)
        self.trigger_cooldown = rospy.get_param('~trigger_cooldown', 30.0)
        self.recovery_hold = rospy.get_param('~recovery_hold', 20.0)

        # Localization mode and integration settings
        self.localization_mode = rospy.get_param('~localization_mode', 'slam_toolbox')
        self.global_localization_service = rospy.get_param('~global_localization_service', '/global_localization')
        self.slam_toolbox_node_name = rospy.get_param('~slam_toolbox_node_name', 'slam_toolbox')
        self.slam_toolbox_reset_service = rospy.get_param('~slam_toolbox_reset_service', '/' + self.slam_toolbox_node_name + '/reset')
        self.cartographer_reset_service = rospy.get_param('~cartographer_reset_service', '/cartographer_reset')
        self.initialpose_topic = rospy.get_param('~initialpose_topic', '/initialpose')
        self.base_frame = rospy.get_param('~base_frame', 'base_footprint')
        self.publish_initialpose = rospy.get_param('~publish_initialpose', True)

        # Optional recovery strategy
        self.stop_robot_on_relocalization = rospy.get_param('~stop_robot_on_relocalization', False)
        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')

        # ROS interfaces
        self.odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback, queue_size=5)
        self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.imu_callback, queue_size=10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=1)
        self.status_pub = rospy.Publisher('/localization_status', String, queue_size=1, latch=True)
        self.initialpose_pub = rospy.Publisher(self.initialpose_topic, PoseWithCovarianceStamped, queue_size=1)
        self.cmd_vel_pub = None
        if self.stop_robot_on_relocalization:
            self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)

        self.trigger_service = rospy.Service('/relocalization_trigger', Trigger, self.on_manual_trigger)

        self.tf_listener = tf.TransformListener()
        self.prev_odom = None
        self.prev_imu = None
        self.last_scan_time = rospy.Time.now()
        self.last_trigger_time = rospy.Time(0)
        self.recovery_active = False
        self.localization_status = 'OK'

        self.publish_status(self.localization_status)
        rospy.Timer(rospy.Duration(0.5), self.timer_callback)

        rospy.loginfo('LocalizationRecoveryNode initialized in %s mode', self.localization_mode)
        rospy.spin()

    def odom_callback(self, msg):
        if self.prev_odom is None:
            self.prev_odom = msg
            return

        dt = (msg.header.stamp - self.prev_odom.header.stamp).to_sec()
        if dt <= 0.0 or dt > 5.0:
            self.prev_odom = msg
            return

        dx = msg.pose.pose.position.x - self.prev_odom.pose.pose.position.x
        dy = msg.pose.pose.position.y - self.prev_odom.pose.pose.position.y
        dist = math.hypot(dx, dy)
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        prev_yaw = quaternion_to_yaw(self.prev_odom.pose.pose.orientation)
        yaw_delta = normalize_angle(yaw - prev_yaw)

        if dist > self.position_jump_threshold or abs(yaw_delta) > self.rotation_jump_threshold:
            self.report_abnormal('pose jump: dist=%.2fm, yaw=%.2frad' % (dist, yaw_delta))

        odom_speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        measured_speed = dist / dt
        if abs(measured_speed - odom_speed) > self.odom_inconsistency_threshold:
            self.report_abnormal('odometry inconsistency: measured=%.2fm/s, reported=%.2fm/s' % (measured_speed, odom_speed))

        self.prev_odom = msg

    def imu_callback(self, msg):
        if self.prev_imu is None:
            self.prev_imu = msg
            return

        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        prev = self.prev_imu.linear_acceleration
        da = math.sqrt((ax - prev.x) ** 2 + (ay - prev.y) ** 2 + (az - prev.z) ** 2)
        acc_norm = math.sqrt(ax * ax + ay * ay + az * az)

        if da > self.imu_delta_accel_threshold or acc_norm > self.imu_accel_spike_threshold:
            self.report_abnormal('IMU spike: delta=%.2fm/s^2, norm=%.2fm/s^2' % (da, acc_norm))

        self.prev_imu = msg

    def scan_callback(self, msg):
        self.last_scan_time = msg.header.stamp if msg.header.stamp != rospy.Time(0) else rospy.Time.now()

    def on_manual_trigger(self, req):
        rospy.loginfo('Manual relocalization trigger requested')
        self.trigger_relocalization('manual request')
        return TriggerResponse(success=True, message='Relocalization triggered')

    def report_abnormal(self, reason):
        now = rospy.Time.now()
        if (now - self.last_trigger_time).to_sec() < self.trigger_cooldown:
            rospy.logwarn('Abnormal condition detected but still in cooldown: %s', reason)
            return

        rospy.logwarn('Abnormal localization condition detected: %s', reason)
        self.localization_status = 'LOST'
        self.publish_status(self.localization_status)
        self.trigger_relocalization(reason)

    def trigger_relocalization(self, reason):
        self.last_trigger_time = rospy.Time.now()
        self.recovery_active = True
        self.localization_status = 'RECOVERING'
        self.publish_status(self.localization_status)
        self.publish_recovery_halt()

        if self.localization_mode == 'amcl':
            self.call_amcl_global_localization()
        elif self.localization_mode == 'slam_toolbox':
            self.perform_slam_toolbox_relocalization()
        elif self.localization_mode == 'cartographer':
            self.perform_cartographer_relocalization()
        else:
            rospy.logwarn('Unknown localization_mode=%s, skipping automatic recovery', self.localization_mode)

    def call_amcl_global_localization(self):
        try:
            rospy.wait_for_service(self.global_localization_service, timeout=2.0)
            service = rospy.ServiceProxy(self.global_localization_service, Trigger)
            response = service()
            rospy.loginfo('AMCL global localization service returned: success=%s, message=%s', response.success, response.message)
        except Exception as ex:
            rospy.logwarn('Failed to call AMCL global localization service %s: %s', self.global_localization_service, ex)
        self.publish_initial_pose_guess()

    def perform_slam_toolbox_relocalization(self):
        self.publish_initial_pose_guess()
        reset_success = False

        if self.slam_toolbox_reset_service:
            try:
                rospy.wait_for_service(self.slam_toolbox_reset_service, timeout=2.0)
                reset_srv = rospy.ServiceProxy(self.slam_toolbox_reset_service, Trigger)
                response = reset_srv()
                reset_success = response.success
                rospy.loginfo('Slam Toolbox reset service returned: success=%s, message=%s', response.success, response.message)
            except Exception as ex:
                rospy.logwarn('Slam Toolbox reset service not available: %s', ex)

        if not reset_success:
            rospy.loginfo('Published initialpose to help SLAM Toolbox relocalize')

    def perform_cartographer_relocalization(self):
        self.publish_initial_pose_guess()
        if self.cartographer_reset_service:
            try:
                rospy.wait_for_service(self.cartographer_reset_service, timeout=2.0)
                reset_srv = rospy.ServiceProxy(self.cartographer_reset_service, Trigger)
                response = reset_srv()
                rospy.loginfo('Cartographer reset service returned: success=%s, message=%s', response.success, response.message)
            except Exception as ex:
                rospy.logwarn('Cartographer reset service not available: %s', ex)
        else:
            rospy.loginfo('Published initialpose for Cartographer relocalization')

    def publish_initial_pose_guess(self):
        if not self.publish_initialpose:
            return

        pose = PoseWithCovarianceStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'map'
        pose.pose.pose = self.get_map_pose_guess()

        # high covariance encourages global recovery instead of overconfidence
        cov = [0.0] * 36
        cov[0] = 0.5
        cov[7] = 0.5
        cov[35] = 0.8
        pose.pose.covariance = cov

        self.initialpose_pub.publish(pose)
        rospy.loginfo('Published initialpose guess for relocalization')

    def get_map_pose_guess(self):
        pose = PoseWithCovarianceStamped().pose.pose
        try:
            self.tf_listener.waitForTransform('map', self.base_frame, rospy.Time(0), rospy.Duration(0.5))
            trans, rot = self.tf_listener.lookupTransform('map', self.base_frame, rospy.Time(0))
            pose.position.x = trans[0]
            pose.position.y = trans[1]
            pose.position.z = trans[2]
            pose.orientation.x = rot[0]
            pose.orientation.y = rot[1]
            pose.orientation.z = rot[2]
            pose.orientation.w = rot[3]
            rospy.logdebug('Using tf map->%s pose as relocalization seed', self.base_frame)
        except Exception:
            rospy.logwarn('Unable to get map->%s transform for pose guess; using zero pose', self.base_frame)
            pose.position.x = 0.0
            pose.position.y = 0.0
            pose.position.z = 0.0
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0
        return pose

    def publish_recovery_halt(self):
        if self.stop_robot_on_relocalization and self.cmd_vel_pub is not None:
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            rospy.loginfo('Published zero cmd_vel while relocalization is active')

    def timer_callback(self, event):
        now = rospy.Time.now()

        if self.recovery_active:
            self.publish_recovery_halt()
            if (now - self.last_trigger_time).to_sec() >= self.recovery_hold:
                self.recovery_active = False
                self.localization_status = 'OK'
                self.publish_status(self.localization_status)
                rospy.loginfo('Localization recovery hold complete, setting status OK')
                return

        if (now - self.last_scan_time).to_sec() > self.scan_timeout and self.localization_status != 'LOST':
            self.localization_status = 'LOST'
            self.publish_status(self.localization_status)
            rospy.logwarn('No /scan message received for %.1fs, marking localization status LOST', (now - self.last_scan_time).to_sec())

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


if __name__ == '__main__':
    try:
        LocalizationRecoveryNode()
    except rospy.ROSInterruptException:
        pass
