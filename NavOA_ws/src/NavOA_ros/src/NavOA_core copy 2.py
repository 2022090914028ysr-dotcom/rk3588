#!/usr/bin/env python
import sys
from pathlib import Path

# Add NavOA package into sys.path
_cur = Path(__file__).resolve()
_pkg_parent = _cur.parent.parent  # NavOA_ros
_navoa_pkg = _pkg_parent / "NavOA"  # NavOA_ros/NavOA
sys.path.insert(0, str(_navoa_pkg))

from NavOA import NavOA
import rospy
from geometry_msgs.msg import Twist, PoseStamped, Quaternion, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import LaserScan, PointCloud2
from math import sin, cos, atan2
import numpy as np
from NavOA.util import get_transform
import tf
import sensor_msgs.point_cloud2 as pc2


class NavOA_core:
    def __init__(self) -> None:

        rospy.init_node("NavOA_node", anonymous=True)

        # ros parameters
        self.planner_config_file = rospy.get_param("~config_file", None)
        self.map_frame = rospy.get_param("~map_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")
        self.lidar_frame = rospy.get_param("~lidar_frame", "laser_frame")
        self.marker_size = float(rospy.get_param("~marker_size", "0.05"))
        self.marker_z = float(rospy.get_param("~marker_z", "1.0"))

        scan_angle_range_para = rospy.get_param("~scan_angle_range", "-3.14 3.14")
        self.scan_angle_range = np.fromstring(
            scan_angle_range_para, dtype=np.float32, sep=" "
        )

        self.scan_downsample = int(rospy.get_param("~scan_downsample", "1"))

        scan_range_para = rospy.get_param("~scan_range", "0.0, 5.0")
        self.scan_range = np.fromstring(scan_range_para, dtype=np.float32, sep=" ")

        self.dune_checkpoint = rospy.get_param("~dune_checkpoint", None)
        self.refresh_initial_path = rospy.get_param("~refresh_initial_path", False)
        self.flip_angle = rospy.get_param("~flip_angle", False)
        self.include_initial_path_direction = rospy.get_param("~include_initial_path_direction", False)
        self.slow_down_radius = float(rospy.get_param("~slow_down_radius", "3.0"))
        self.max_speed_scale = float(rospy.get_param("~max_speed_scale", "1.5"))
        self.min_speed_near_goal = float(rospy.get_param("~min_speed_near_goal", "0.05"))
        self.goal = None
        
        rospy.loginfo(f"=== Speed Control Configuration ===")
        rospy.loginfo(f"Slow radius: {self.slow_down_radius}m")
        rospy.loginfo(f"Max speed scale: {self.max_speed_scale}x")
        rospy.loginfo(f"Min near-goal speed: {self.min_speed_near_goal}m/s")
        rospy.loginfo(f"Config file: {self.planner_config_file}")
        rospy.loginfo(f"=== Speed Control Configuration ===")
        
        if self.planner_config_file is None:
            raise ValueError(
                "No planner config file provided! Please set the parameter ~config_file"
            )

        pan = {'dune_checkpoint': self.dune_checkpoint}
        self.NavOA_planner = NavOA.init_from_yaml(
            self.planner_config_file, pan=pan
        )

        # data
        self.obstacle_points = None  # (2, n)  n number of points
        self.robot_state = None  # (3, 1) [x, y, theta]
        self.stop = False
        self.arrive = False

        # publisher
        self.vel_pub = rospy.Publisher("/NavOA_cmd_vel", Twist, queue_size=10)
        self.plan_pub = rospy.Publisher("/NavOA_plan", Path, queue_size=10)
        self.ref_state_pub = rospy.Publisher(
            "/NavOA_ref_state", Path, queue_size=10
        )  # current reference state
        self.ref_path_pub = rospy.Publisher(
            "/NavOA_initial_path", Path, queue_size=10
        )  # initial path

        ## for rviz visualization
        self.point_markers_pub_dune = rospy.Publisher(
            "/dune_point_markers", MarkerArray, queue_size=10
        )
        self.robot_marker_pub = rospy.Publisher("/robot_marker", Marker, queue_size=10)
        self.point_markers_pub_nrmp = rospy.Publisher(
            "/nrmp_point_markers", MarkerArray, queue_size=10
        )

        self.listener = tf.TransformListener()

        # subscriber
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        # three types of initial path:
        # 1. from given path
        # 2. from waypoints
        # 3. from goal position
        rospy.Subscriber("/initial_path", Path, self.path_callback)
        rospy.Subscriber("/NavOA_waypoints", Path, self.waypoints_callback)
        rospy.Subscriber("/NavOA_goal", PoseStamped, self.goal_callback)
        
    def run(self):

        r = rospy.Rate(50)

        while not rospy.is_shutdown():

            try:
                (trans, rot) = self.listener.lookupTransform(
                    self.map_frame, self.base_frame, rospy.Time(0)
                )

                yaw = self.quat_to_yaw_list(rot)
                x, y = trans[0], trans[1]
                self.robot_state = np.array([x, y, yaw]).reshape(3, 1)

            except (
                tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException,
            ):
                rospy.loginfo_throttle(
                    1,
                    "waiting for tf for the transform from {} to {}".format(
                        self.base_frame, self.map_frame
                    ),
                )
                continue

            if self.robot_state is None:
                rospy.logwarn_throttle(1, "waiting for robot state")
                continue

            rospy.loginfo_once(
                "robot state received {}".format(self.robot_state.tolist())
            )

            if (
                len(self.NavOA_planner.waypoints) >= 1
                and self.NavOA_planner.initial_path is None
            ):
                self.NavOA_planner.set_initial_path_from_state(self.robot_state)

            if self.NavOA_planner.initial_path is None:
                rospy.logwarn_throttle(1, "waiting for NavOA initial path")
                continue

            rospy.loginfo_once("initial Path Received")
            self.ref_path_pub.publish(
                self.generate_path_msg(self.NavOA_planner.initial_path)
            )

            if self.obstacle_points is None:
                rospy.logwarn_throttle(
                    1, "No obstacle points, only path tracking task will be performed"
                )

            # 首先检查是否已经到达或停止
            if self.arrive or self.stop:
                rospy.loginfo_throttle(1.0, 
                    f"Status: {'ARRIVED' if self.arrive else 'STOPPED'}, "
                    f"publishing zero velocity")
                self.vel_pub.publish(Twist())
                r.sleep()
                continue
            
            # 获取规划器动作
            action, info = self.NavOA_planner(self.robot_state, self.obstacle_points)
            
            # 更新状态
            self.stop = info["stop"]
            self.arrive = info["arrive"]
            
            # 如果刚刚到达，记录日志并跳过速度计算
            if self.arrive:
                if self.goal is not None and self.robot_state is not None:
                    dist = np.linalg.norm(self.goal[:2, 0] - self.robot_state[:2, 0])
                    rospy.loginfo(f"*** ARRIVED AT GOAL! Distance: {dist:.2f}m ***")
                else:
                    rospy.loginfo("*** ARRIVED AT GOAL! ***")
                
                self.vel_pub.publish(Twist())
                r.sleep()
                continue
            
            # 只有未到达且未停止时才进行速度计算和发布
            if action is not None:
                # 记录原始速度用于调试
                original_action = action.copy()
                original_speed = original_action[0, 0]
                
                # 应用速度限制
                action = self.apply_goal_speed_limit(action)
                
                # 生成并发布速度命令
                vel_msg = self.generate_twist_msg(action)
                self.vel_pub.publish(vel_msg)
                
                # 调试信息
                if abs(original_speed) > 0.01 and action is not None:
                    scaled_speed = action[0, 0]
                    if abs(original_speed) > 0.001:  # 避免除以0
                        scale_factor = scaled_speed / original_speed
                        rospy.loginfo_throttle(0.5, 
                            f"Speed change: {original_speed:.3f} -> {scaled_speed:.3f} m/s, "
                            f"scale: {scale_factor:.2f}x")
            else:
                rospy.logwarn_throttle(1, "No valid action generated")
                self.vel_pub.publish(Twist())

            # publish the path
            self.plan_pub.publish(self.generate_path_msg(info["opt_state_list"]))
            self.ref_state_pub.publish(self.generate_path_msg(info["ref_state_list"]))

            # publish visualization markers
            self.point_markers_pub_dune.publish(self.generate_dune_points_markers_msg())
            self.point_markers_pub_nrmp.publish(self.generate_nrmp_points_markers_msg())
            self.robot_marker_pub.publish(self.generate_robot_marker_msg())

            if info["stop"]:
                rospy.logwarn_throttle(
                    0.5,
                    "NavOA stop with the min distance "
                    + str(self.NavOA_planner.min_distance.detach().item())
                    + " threshold "
                    + str(self.NavOA_planner.collision_threshold),
                )
                
            # Show goal distance info and status
            if self.goal is not None and self.robot_state is not None:
                dist = np.linalg.norm(self.goal[:2, 0] - self.robot_state[:2, 0])
                
                # Determine current zone
                if dist >= self.slow_down_radius:
                    zone = f"> {self.slow_down_radius}m (Normal Speed)"
                    zone_desc = "Normal"
                elif dist >= self.slow_down_radius * 0.9:
                    zone = f"{self.slow_down_radius}-{self.slow_down_radius*0.9:.1f}m (Acceleration)"
                    zone_desc = "Acceleration"
                else:
                    zone = f"< {self.slow_down_radius*0.9:.1f}m (Deceleration)"
                    zone_desc = "Deceleration"
                
                rospy.loginfo_throttle(1.0,
                    f"Status: Distance={dist:.2f}m, Zone={zone_desc}, "
                    f"Arrive={self.arrive}, Stop={self.stop}")

            r.sleep()

    # scan callback
    def scan_callback(self, scan_msg):

        if self.robot_state is None:
            return None

        ranges = np.array(scan_msg.ranges)
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))

        points = []
        # x, y, z, yaw, pitch, roll = self.lidar_offset

        if self.flip_angle:
            angles = np.flip(angles)

        for i in range(len(ranges)):
            distance = ranges[i]
            angle = angles[i]

            if (
                i % self.scan_downsample == 0
                and distance >= self.scan_range[0]
                and distance <= self.scan_range[1]
                and angle > self.scan_angle_range[0]
                and angle < self.scan_angle_range[1]
            ):
                point = np.array([[distance * cos(angle)], [distance * sin(angle)]])
                points.append(point)

        if len(points) == 0:
            self.obstacle_points = None
            rospy.loginfo_once("No valid scan points")
            return None

        point_array = np.hstack(points)

        try:
            (trans, rot) = self.listener.lookupTransform(
                self.map_frame, self.lidar_frame, rospy.Time(0)
            )

            yaw = self.quat_to_yaw_list(rot)
            x, y = trans[0], trans[1]

            trans_matrix, rot_matrix = get_transform(np.c_[x, y, yaw].reshape(3, 1))
            self.obstacle_points = rot_matrix @ point_array + trans_matrix
            rospy.loginfo_once("Scan obstacle points Received")

            return self.obstacle_points

        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            rospy.loginfo_throttle(
                1,
                "waiting for tf for the transform from {} to {}".format(
                    self.lidar_frame, self.map_frame
                ),
            )
            return

    def path_callback(self, path):

        initial_point_list = []

        for i in range(len(path.poses)):
            p = path.poses[i]
            x = p.pose.position.x
            y = p.pose.position.y
            
            if self.include_initial_path_direction:
                theta = self.quat_to_yaw(p.pose.orientation)
            else:
                rospy.loginfo_once("Using the points gradient as the initial path direction")

                if i + 1 < len(path.poses):
                    p2 = path.poses[i + 1]
                    x2 = p2.pose.position.x
                    y2 = p2.pose.position.y
                    theta = atan2(y2 - y, x2 - x)
                else:
                    theta = initial_point_list[-1][2, 0]
            
            points = np.array([x, y, theta, 1]).reshape(4, 1)
            initial_point_list.append(points)

        if self.NavOA_planner.initial_path is None or self.refresh_initial_path:
            rospy.loginfo_throttle(0.1, "initial path update from given path")
            self.NavOA_planner.set_initial_path(initial_point_list)
            self.NavOA_planner.reset()
    
    def waypoints_callback(self, path):
        
        '''
        Utilize multiple waypoints (goals) to set the initial path
        '''

        waypoints_list = [self.robot_state]

        for i in range(len(path.poses)):
            p = path.poses[i]
            x = p.pose.position.x
            y = p.pose.position.y
            
            if self.include_initial_path_direction:
                theta = self.quat_to_yaw(p.pose.orientation)
            else:
                rospy.loginfo_once("Using the points gradient as the initial path direction")

                if i + 1 < len(path.poses):
                    p2 = path.poses[i + 1]
                    x2 = p2.pose.position.x
                    y2 = p2.pose.position.y
                    theta = atan2(y2 - y, x2 - x)
                else:
                    theta = waypoints_list[-1][2, 0]
            
            points = np.array([x, y, theta, 1]).reshape(4, 1)
            waypoints_list.append(points)

        if self.NavOA_planner.initial_path is None or self.refresh_initial_path:
            rospy.loginfo_throttle(0.1, "initial path update from waypoints")
            self.NavOA_planner.update_initial_path_from_waypoints(waypoints_list)
            self.NavOA_planner.reset()

    def goal_callback(self, goal):

        x = goal.pose.position.x
        y = goal.pose.position.y
        theta = self.quat_to_yaw(goal.pose.orientation)

        self.goal = np.array([[x], [y], [theta]])

        rospy.loginfo(f"=== Goal Set ===")
        rospy.loginfo(f"Goal position: [{x:.2f}, {y:.2f}, {theta:.2f}]")
        rospy.loginfo(f"Slow radius: {self.slow_down_radius}m")
        rospy.loginfo(f"Max speed scale: {self.max_speed_scale}x")
        rospy.loginfo(f"Min near-goal speed: {self.min_speed_near_goal}m/s")
        rospy.loginfo(f"=== Goal Set ===")

        # Check if robot state is available before updating path
        if self.robot_state is None:
            rospy.logwarn("Cannot set goal: robot state not yet available. Waiting for odometry...")
            return

        rospy.loginfo_throttle(0.1, "initial path update from goal position")
        self.NavOA_planner.update_initial_path_from_goal(self.robot_state, self.goal)
        self.NavOA_planner.reset()

    def quat_to_yaw_list(self, quater):

        x = quater[0]
        y = quater[1]
        z = quater[2]
        w = quater[3]

        yaw = atan2(2 * (w * z + x * y), 1 - 2 * (pow(z, 2) + pow(y, 2)))

        return yaw

    def apply_goal_speed_limit(self, action):
        """
        Three-stage speed control:
        1. >5m: original speed
        2. 5m-3m: max speed (2.0x)
        3. <3m: exponential deceleration to min speed
        """
        if self.arrive or self.stop:
            if action is not None:
                return np.zeros_like(action)
            return action
        
        if action is None or self.goal is None or self.robot_state is None:
            return action

        dist = np.linalg.norm(self.goal[:2, 0] - self.robot_state[:2, 0])
        original_speed = action[0, 0]
        speed_magnitude = abs(original_speed)
        
        # Safety stop distance
        if dist < 0.3:
            rospy.loginfo_once(f"Very close to goal ({dist:.2f}m), stopping")
            if action is not None:
                return np.zeros_like(action)
            return action
        
        # 1. Outside 5m: original speed
        if dist >= self.slow_down_radius:  # 5.0m
            rospy.loginfo_throttle(1.0, 
                f"Distance > {self.slow_down_radius}m: original speed {original_speed:.3f}m/s")
            return action
        
        # 2. 5m-3m: max speed
        max_speed_distance = 3.0  # Start deceleration at 3m
        if dist >= max_speed_distance:
            # Keep max speed
            scaled_action = action.copy()
            scaled_action[0, 0] = original_speed * self.max_speed_scale
            
            rospy.loginfo_throttle(0.5,
                f"[MAX SPEED] Distance: {dist:.2f}m, "
                f"Speed: {original_speed:.3f}->{original_speed*self.max_speed_scale:.3f}m/s")
            
            return scaled_action
        
        # 3. Inside 3m: exponential deceleration
        # Calculate minimum scaling factor
        min_scale = max(self.min_speed_near_goal / speed_magnitude, 0.1)
        
        # Normalized distance: between 0 and 1
        norm_dist = dist / max_speed_distance
        
        # Use quadratic function for steeper deceleration
        scale = min_scale + (self.max_speed_scale - min_scale) * (norm_dist ** 2)
        
        # Ensure scaling factor is within reasonable range
        scale = max(min_scale, min(scale, self.max_speed_scale))
        
        # Apply scaling
        scaled_action = action.copy()
        scaled_action[0, 0] = original_speed * scale
        
        # Angular velocity also decelerates but doesn't accelerate
        if abs(action[1, 0]) > 0.001:
            angular_scale = min(scale, 1.0)  # Angular speed doesn't exceed original
            scaled_action[1, 0] = action[1, 0] * angular_scale
        
        rospy.loginfo_throttle(0.5,
            f"[STRONG DECELERATION] Distance: {dist:.2f}m, "
            f"Speed: {original_speed:.3f}->{original_speed*scale:.3f}m/s "
            f"(Scale: {scale:.2f}x)")
        
        return scaled_action

    # generate ros message
    def generate_path_msg(self, path_list):

        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = rospy.Time.now()
        path.header.seq = 0

        for index, point in enumerate(path_list):
            ps = PoseStamped()
            ps.header.frame_id = self.map_frame
            ps.header.seq = index

            ps.pose.position.x = point[0, 0]
            ps.pose.position.y = point[1, 0]
            ps.pose.orientation = self.yaw_to_quat(point[2, 0])

            path.poses.append(ps)

        return path

    def generate_twist_msg(self, vel):
        if vel is None:
            rospy.loginfo("No velocity command generated")
            return Twist()

        speed = vel[0, 0]
        steer = vel[1, 0]
        speed_magnitude = abs(speed)
        
        # Check if we should stop
        if self.stop:
            rospy.loginfo("Stop flag True, publishing zero velocity")
            return Twist()
        
        if self.arrive:
            rospy.loginfo("Arrive flag True, publishing zero velocity")
            return Twist()
        
        if speed_magnitude < 0.01:  # Speed too low
            rospy.loginfo("Speed too low (<0.01m/s), publishing zero velocity")
            return Twist()
        
        # Normal velocity publishing
        action = Twist()
        action.linear.x = speed
        action.angular.z = steer
        
        rospy.loginfo_throttle(0.5, 
            f"Publishing velocity: linear={speed:.3f}m/s, angular={steer:.3f}rad/s")
        
        return action

    def generate_dune_points_markers_msg(self):

        marker_array = MarkerArray()

        if self.NavOA_planner.dune_points is None:
            return
        else:
            points = self.NavOA_planner.dune_points

            for index, point in enumerate(points.T):

                marker = Marker()
                marker.header.frame_id = self.map_frame
                marker.header.seq = 0
                marker.header.stamp = rospy.get_rostime()

                marker.scale.x = self.marker_size
                marker.scale.y = self.marker_size
                marker.scale.z = self.marker_size
                marker.color.a = 1.0

                marker.color.r = 160 / 255
                marker.color.g = 32 / 255
                marker.color.b = 240 / 255

                marker.id = index
                marker.type = 1
                marker.pose.position.x = point[0]
                marker.pose.position.y = point[1]
                marker.pose.position.z = 0.3
                marker.pose.orientation = Quaternion()

                marker_array.markers.append(marker)

            return marker_array

    def generate_nrmp_points_markers_msg(self):

        marker_array = MarkerArray()

        if self.NavOA_planner.nrmp_points is None:
            return
        else:
            points = self.NavOA_planner.nrmp_points

            for index, point in enumerate(points.T):

                marker = Marker()
                marker.header.frame_id = self.map_frame
                marker.header.seq = 0
                marker.header.stamp = rospy.get_rostime()

                marker.scale.x = self.marker_size
                marker.scale.y = self.marker_size
                marker.scale.z = self.marker_size
                marker.color.a = 1.0

                marker.color.r = 255 / 255
                marker.color.g = 128 / 255
                marker.color.b = 0 / 255

                marker.id = index
                marker.type = 1
                marker.pose.position.x = point[0]
                marker.pose.position.y = point[1]
                marker.pose.position.z = 0.3
                marker.pose.orientation = Quaternion()

                marker_array.markers.append(marker)

            return marker_array

    def generate_robot_marker_msg(self):

        marker = Marker()

        marker.header.frame_id = self.map_frame
        marker.header.seq = 0
        marker.header.stamp = rospy.get_rostime()

        marker.color.a = 1.0
        marker.color.r = 0 / 255
        marker.color.g = 255 / 255
        marker.color.b = 0 / 255

        marker.id = 0

        if self.NavOA_planner.robot.shape == "rectangle":
            length = self.NavOA_planner.robot.length
            width = self.NavOA_planner.robot.width
            wheelbase = self.NavOA_planner.robot.wheelbase

            marker.scale.x = length
            marker.scale.y = width
            marker.scale.z = self.marker_z

            marker.type = 1

            x = self.robot_state[0, 0]
            y = self.robot_state[1, 0]
            theta = self.robot_state[2, 0]

            if self.NavOA_planner.robot.kinematics == "acker":
                diff_len = (length - wheelbase) / 2
                marker_x = x + diff_len * cos(theta)
                marker_y = y + diff_len * sin(theta)
            else:
                marker_x = x
                marker_y = y

            marker.pose.position.x = marker_x
            marker.pose.position.y = marker_y
            marker.pose.position.z = 0
            marker.pose.orientation = self.yaw_to_quat(self.robot_state[2, 0])

        return marker

    @staticmethod
    def yaw_to_quat(yaw):

        quater = Quaternion()

        quater.x = 0
        quater.y = 0
        quater.z = sin(yaw / 2)
        quater.w = cos(yaw / 2)

        return quater

    @staticmethod
    def quat_to_yaw(quater):

        x = quater.x
        y = quater.y
        z = quater.z
        w = quater.w

        raw = atan2(2 * (w * z + x * y), 1 - 2 * (pow(z, 2) + pow(y, 2)))

        return raw

if __name__ == "__main__":
    try:
        navoa = NavOA_core()
        navoa.run()
    except rospy.ROSInterruptException:
        pass