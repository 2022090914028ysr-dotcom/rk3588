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
from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import LaserScan
from math import sin, cos, atan2
import numpy as np
import traceback
from NavOA.util import get_transform
import tf
from std_msgs.msg import Float32, Bool, Int32MultiArray


class NavOA_core:
    def __init__(self) -> None:

        rospy.init_node("NavOA_node", anonymous=False)

        # ros parameters
        self.planner_config_file = rospy.get_param("~config_file", None)
        self.map_frame = rospy.get_param("~map_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
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
        self.use_rknn = rospy.get_param("~use_rknn", False)
        self.dune_rknn = rospy.get_param("~dune_rknn", None)
        self.rknn_target = rospy.get_param("~rknn_target", "")
        self.rknn_core = rospy.get_param("~rknn_core", "auto")
        self.refresh_initial_path = rospy.get_param("~refresh_initial_path", False)
        self.control_rate = float(rospy.get_param("~control_rate", 50.0))
        if self.control_rate <= 0.0:
            rospy.logwarn("[NavOA] Invalid ~control_rate=%.3f, fallback to 50.0 Hz", self.control_rate)
            self.control_rate = 50.0
        self.flip_angle = rospy.get_param("~flip_angle", False)
        self.include_initial_path_direction = rospy.get_param("~include_initial_path_direction", False)
        self.goal = None
        
        if self.planner_config_file is None:
            raise ValueError(
                "No planner config file provided! Please set the parameter ~config_file"
            )

        pan = {
            'dune_checkpoint': self.dune_checkpoint,
            'dune_rknn': self.dune_rknn,
            'use_rknn': self.use_rknn,
            'rknn_target': self.rknn_target,
            'rknn_core': self.rknn_core,
        }
        if self.use_rknn:
            rospy.loginfo(f"[NavOA] use_rknn=True, dune_rknn={self.dune_rknn}, target={self.rknn_target}, core={self.rknn_core}")
        else:
            rospy.loginfo(f"[NavOA] use_rknn=False, dune_checkpoint={self.dune_checkpoint}")
        self.NavOA_planner = NavOA.init_from_yaml(
            self.planner_config_file, pan=pan
        )

        # If a ref_speed parameter was provided via launch or rosparam, override planner value
        ref_speed_param = rospy.get_param("~ref_speed", None)
        if ref_speed_param is not None:
            try:
                ref_speed_val = float(ref_speed_param)
                self.NavOA_planner.set_reference_speed(ref_speed_val)
                rospy.loginfo(f"[NavOA] ref_speed set from param: {ref_speed_val}")
            except Exception as e:
                rospy.logwarn(f"[NavOA] Invalid ~ref_speed parameter: {ref_speed_param} ({e})")

        # dynamic_reconfigure server for runtime updates
        try:
            # Import dynamically so node won't crash if package isn't built yet
            from dynamic_reconfigure.server import Server
            from NavOA_ros.cfg import RefSpeedConfig

            self.dyn_reconf_server = Server(RefSpeedConfig, self.dynamic_reconfigure_callback)
            rospy.loginfo("[NavOA] dynamic_reconfigure server initialized")
        except Exception as e:
            rospy.logwarn(f"[NavOA] dynamic_reconfigure not available: {e}")

        # Publisher for current ref_speed for verification
        self.ref_speed_pub = rospy.Publisher('/NavOA/ref_speed', Float32, queue_size=1)

        # publish current ref_speed on startup
        try:
            current_speed = float(self.NavOA_planner.ref_speed)
            self.ref_speed_pub.publish(Float32(data=current_speed))
        except Exception:
            pass

        # data
        self.obstacle_points = None  # (2, n)  n number of points
        self.robot_state = None  # (3, 1) [x, y, theta]
        self.stop = False
        self.arrive = False
        
        # --- 新增：规划完成标志位，用于控制流程 ---
        self.planning_finished = False

        # If enabled, do not move until an explicit goal/path/waypoints arrives.
        # This prevents unexpected motion on startup or after recovering from faults.
        self.require_explicit_goal = bool(rospy.get_param("~require_explicit_goal", False))
        self._explicit_goal_active = False

        # publisher
        self.vel_pub = rospy.Publisher("/NavOA_cmd_vel", Twist, queue_size=10)
        self.plan_pub = rospy.Publisher("/NavOA_plan", Path, queue_size=10)
        self.ref_state_pub = rospy.Publisher(
            "/NavOA_ref_state", Path, queue_size=10
        )  # current reference state
        self.ref_path_pub = rospy.Publisher(
            "/NavOA_initial_path", Path, queue_size=10
        )  # initial path

        # --- External control (interrupt) settings ---
        self.external_control_topic = rospy.get_param("~external_control_topic", "/external_cmd_vel")
        self.external_control_enable_topic = rospy.get_param("~external_control_enable_topic", "/external_control_enable")
        self.external_control_timeout = float(rospy.get_param("~external_control_timeout", 1.0))

        # External control state
        self.external_control_active = False  # whether currently using external commands
        self.external_control_forced = False  # if enabled via explicit enable topic
        self.external_last_time = rospy.Time(0)
        self.external_cmd = Twist()

        self.goal_tolerance = float(rospy.get_param('~goal_tolerance', 0.20))

        # Previous/return goal logic
        self.previous_goal = None
        self.goal_reached_time = rospy.Time(0)
        self.return_to_previous_goal_timeout = float(
            rospy.get_param('~return_to_previous_goal_timeout', 5.0)
        )
        self.return_to_previous_goal_enabled = bool(
            rospy.get_param('~return_to_previous_goal_enabled', True)
        )
        self.goal_timeout_button_enabled = bool(
            rospy.get_param('~goal_timeout_button_enabled', False)
        )
        self.goal_timeout_button_topic = rospy.get_param(
            '~goal_timeout_button_topic', '/app_controller/button'
        )
        self.goal_timeout_button_id = int(
            rospy.get_param('~goal_timeout_button_id', 9)
        )
        self.goal_timeout_button_event_type = int(
            rospy.get_param('~goal_timeout_button_event_type', 1)
        )
        self.goal_timeout_button_release_event_type = int(
            rospy.get_param('~goal_timeout_button_release_event_type', 0)
        )
        self.goal_timeout_button_release_delay = float(
            rospy.get_param('~goal_timeout_button_release_delay', 0.10)
        )
        self.goal_timeout_button_send_release = bool(
            rospy.get_param('~goal_timeout_button_send_release', True)
        )
        self.goal_timeout_button_sent = False
        self.goal_timeout_button_standing_active = False
        self.goal_timeout_button_release_timer = None
        self.goal_timeout_button_pub = rospy.Publisher(
            self.goal_timeout_button_topic, Int32MultiArray, queue_size=1
        )

        # Subscribers for external control
        rospy.Subscriber(self.external_control_topic, Twist, self.external_cmd_callback)
        rospy.Subscriber(self.external_control_enable_topic, Bool, self.external_enable_callback)

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
        

    def dynamic_reconfigure_callback(self, config, level):
        """dynamic_reconfigure callback to update ref_speed at runtime"""
        try:
            speed = float(config.get('ref_speed', 0.0))
            self.NavOA_planner.set_reference_speed(speed)
            rospy.loginfo(f"[NavOA] ref_speed updated dynamically: {speed}")
            # publish new value for verification
            try:
                self.ref_speed_pub.publish(Float32(data=speed))
            except Exception:
                pass
        except Exception as e:
            rospy.logerr(f"[NavOA] Failed to update ref_speed dynamically: {e}")
        return config

    def run(self):

        r = rospy.Rate(self.control_rate)
        rospy.loginfo("[NavOA] control loop rate target: %.1f Hz", self.control_rate)

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
                continue

            if self.robot_state is None:
                continue

            # --- External control interrupt handling ---
            now = rospy.Time.now()
            if self.external_control_forced:
                # forced external control: always publish last received external command (could be zero)
                self.vel_pub.publish(self.external_cmd)
                r.sleep()
                continue

            if self.external_control_active:
                # if timed out since last external cmd, deactivate and resume planner
                if (now - self.external_last_time).to_sec() > self.external_control_timeout:
                    self.external_control_active = False
                    rospy.loginfo("[NavOA] External control timeout; resuming planner")
                else:
                    # still receiving external commands: publish and skip planner
                    self.vel_pub.publish(self.external_cmd)
                    r.sleep()
                    continue

            if (
                self.goal is not None
                and self.robot_state is not None
                and not self.planning_finished
            ):
                dist_to_goal = np.linalg.norm(self.goal[:2, 0] - self.robot_state[:2, 0])
                if dist_to_goal <= self.goal_tolerance:
                    rospy.loginfo(
                        "[NavOA] Goal tolerance reached (%.3fm <= %.3fm); stopping.",
                        dist_to_goal,
                        self.goal_tolerance,
                    )
                    self.vel_pub.publish(Twist())
                    self.goal_reached_time = rospy.Time.now()
                    self.goal_timeout_button_sent = False
                    self.goal = None
                    self._explicit_goal_active = False
                    self.planning_finished = True
                    r.sleep()
                    continue

            if (
                self.planning_finished
                and self.goal_reached_time != rospy.Time(0)
                and (now - self.goal_reached_time).to_sec() > self.return_to_previous_goal_timeout
            ):
                if self.goal_timeout_button_enabled and not self.goal_timeout_button_sent:
                    self._publish_goal_timeout_button(reason="goal-timeout-stand")
                elif self.return_to_previous_goal_enabled and self.previous_goal is not None:
                    rospy.loginfo(
                        "[NavOA] No new goal received after %.1fs, returning to previous goal",
                        self.return_to_previous_goal_timeout,
                    )
                    self._return_to_previous_goal()

            if self.require_explicit_goal and (not self._explicit_goal_active):
                self.vel_pub.publish(Twist())
                r.sleep()
                continue

            if (
                len(self.NavOA_planner.waypoints) >= 1
                and self.NavOA_planner.initial_path is None
            ):
                self.NavOA_planner.set_initial_path_from_state(self.robot_state)


            if self.NavOA_planner.initial_path is None or self.planning_finished:
                self.vel_pub.publish(Twist())
                r.sleep()
                continue

            self.ref_path_pub.publish(
                self.generate_path_msg(self.NavOA_planner.initial_path)
            )

            # --- 增加异常捕获，防止规划器崩溃 ---
            try:
                action, info = self.NavOA_planner(self.robot_state, self.obstacle_points)
                
                self.stop = info["stop"]
                self.arrive = info["arrive"]

                # --- 核心修改：到达终点后设置完成标志位 ---
                if info["arrive"]:
                    self.vel_pub.publish(Twist())
                    self.goal_reached_time = rospy.Time.now()
                    self.goal_timeout_button_sent = False
                    self.goal = None
                    self._explicit_goal_active = False
                    self.planning_finished = True
                    r.sleep()
                    continue

                # publish the path and velocity
                self.plan_pub.publish(self.generate_path_msg(info["opt_state_list"]))
                self.ref_state_pub.publish(self.generate_path_msg(info["ref_state_list"]))
                self.vel_pub.publish(self.generate_twist_msg(action))

                self.point_markers_pub_dune.publish(self.generate_dune_points_markers_msg())
                self.point_markers_pub_nrmp.publish(self.generate_nrmp_points_markers_msg())
                self.robot_marker_pub.publish(self.generate_robot_marker_msg())
            
            except Exception:
                rospy.logerr_throttle(
                    1.0,
                    "[NavOA] Error in main loop:\n" + traceback.format_exc(),
                )
                self.vel_pub.publish(Twist())

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
            return self.obstacle_points

        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            return None

    def path_callback(self, path):
        # ---------------- 修复：空路径检查 ----------------
        if not path.poses:
            rospy.logwarn_throttle(1.0, "[NavOA] Received an empty path from Global Planner. Ignoring.")
            return
        # Do not auto-release external hold for planner path refreshes.
        # A hard stop should remain latched until an explicit release command
        # or an explicit user goal/waypoint input arrives.

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
                    # 保护：防止列表为空时访问索引
                    if len(initial_point_list) > 0:
                        theta = initial_point_list[-1][2, 0]
                    else:
                        theta = 0.0
            
            points = np.array([x, y, theta, 1]).reshape(4, 1)
            initial_point_list.append(points)

        # 保护：再次检查解析结果
        if len(initial_point_list) == 0:
            rospy.logwarn_throttle(1.0, "[NavOA] Parsed initial path list is empty. Ignoring.")
            return

        if self.NavOA_planner.initial_path is None or self.refresh_initial_path:
            try:
                self.NavOA_planner.set_initial_path(initial_point_list)
                self.NavOA_planner.reset()
                
                # 收到新路径，激活规划逻辑
                self.planning_finished = False
                self._explicit_goal_active = True
                self.goal_timeout_button_sent = False
                
                rospy.loginfo("[NavOA] Initial path updated successfully.")
            except IndexError as e:
                rospy.logerr(f"[NavOA] CRITICAL: Failed to set path (IndexError): {e}. Path size: {len(initial_point_list)}")
            except Exception as e:
                rospy.logerr(f"[NavOA] CRITICAL: Unexpected error in set_initial_path: {e}")
    
    def waypoints_callback(self, path):
        
        '''
        Utilize multiple waypoints (goals) to set the initial path
        '''

        self._ensure_walking_mode("new-waypoints")

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
            self.NavOA_planner.update_initial_path_from_waypoints(waypoints_list)
            self.NavOA_planner.reset()
            self.planning_finished = False
            self._explicit_goal_active = True
            self.goal_timeout_button_sent = False

    def goal_callback(self, goal):

        self._ensure_walking_mode("new-goal")

        x = goal.pose.position.x
        y = goal.pose.position.y
        theta = self.quat_to_yaw(goal.pose.orientation)

        if self.goal is not None:
            try:
                if not np.allclose(self.goal, np.array([[x], [y], [theta]])):
                    self.previous_goal = self.goal.copy()
            except Exception:
                self.previous_goal = self.goal

        self.goal = np.array([[x], [y], [theta]])
        self._explicit_goal_active = True
        self.goal_reached_time = rospy.Time(0)
        self.goal_timeout_button_sent = False

        if self.robot_state is None:
            rospy.logwarn("Cannot set goal: robot state not yet available. Waiting for odometry...")
            return

        self.NavOA_planner.update_initial_path_from_goal(self.robot_state, self.goal)
        self.NavOA_planner.reset()

        self.planning_finished = False


    # ---------------- External control callbacks ----------------
    def external_cmd_callback(self, msg):
        """Receive external Twist commands and switch to external control mode."""
        try:
            self.external_cmd = msg
            self.external_last_time = rospy.Time.now()
            self.external_control_active = True
            rospy.loginfo_once("[NavOA] External command received: switching to external control")
        except Exception as e:
            rospy.logerr(f"[NavOA] Error in external_cmd_callback: {e}")

    def external_enable_callback(self, msg):
        """Enable/disable forced external control via Bool topic."""
        try:
            if msg.data:
                self.external_control_forced = True
                self.external_control_active = True
                rospy.loginfo("[NavOA] External control FORCED enabled")
            else:
                self.external_control_forced = False
                # if no recent external cmd, deactivate immediately
                if (rospy.Time.now() - self.external_last_time).to_sec() > self.external_control_timeout:
                    self.external_control_active = False
                rospy.loginfo("[NavOA] External control FORCED disabled")
        except Exception as e:
            rospy.logerr(f"[NavOA] Error in external_enable_callback: {e}")

    def quat_to_yaw_list(self, quater):

        x = quater[0]
        y = quater[1]
        z = quater[2]
        w = quater[3]

        yaw = atan2(2 * (w * z + x * y), 1 - 2 * (pow(z, 2) + pow(y, 2)))

        return yaw

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
            return Twist()

        speed = vel[0, 0]
        steer = vel[1, 0]

        if self.stop or self.arrive:
            # print('stop flag true')
            return Twist()

        else:
            action = Twist()

            action.linear.x = speed
            action.angular.z = steer

            return action

    def _return_to_previous_goal(self):
        if self.previous_goal is None or self.robot_state is None:
            return

        rospy.loginfo(
            "[NavOA] Fallback replan: returning to previous goal after timeout: %s",
            self.previous_goal.T.tolist(),
        )
        self.goal = self.previous_goal.copy()
        self.NavOA_planner.update_initial_path_from_goal(self.robot_state, self.goal)
        self.NavOA_planner.reset()
        self.planning_finished = False
        self._explicit_goal_active = True
        self.goal_reached_time = rospy.Time(0)
        self.goal_timeout_button_sent = False

    def _publish_button_event(self, event_type, reason):
        msg = Int32MultiArray(
            data=[self.goal_timeout_button_id, int(event_type)]
        )
        self.goal_timeout_button_pub.publish(msg)
        rospy.loginfo(
            "[NavOA] Sent button id=%d event=%d on %s (%s)",
            self.goal_timeout_button_id,
            int(event_type),
            self.goal_timeout_button_topic,
            reason,
        )

    def _goal_timeout_button_release_callback(self, _event):
        self._publish_button_event(
            self.goal_timeout_button_release_event_type,
            "button-release",
        )

    def _send_goal_timeout_button_pair(self, reason):
        if self.goal_timeout_button_release_timer is not None:
            self.goal_timeout_button_release_timer.shutdown()
            self.goal_timeout_button_release_timer = None

        self._publish_button_event(self.goal_timeout_button_event_type, reason)

        if self.goal_timeout_button_send_release:
            delay = max(0.0, self.goal_timeout_button_release_delay)
            self.goal_timeout_button_release_timer = rospy.Timer(
                rospy.Duration(delay),
                self._goal_timeout_button_release_callback,
                oneshot=True,
            )

    def _publish_goal_timeout_button(self, reason="goal-timeout"):
        self._send_goal_timeout_button_pair(reason)
        self.goal_timeout_button_sent = True
        self.goal_timeout_button_standing_active = True
        rospy.loginfo(
            "[NavOA] No new goal within %.1fs; toggled stand mode with button id=%d",
            self.return_to_previous_goal_timeout,
            self.goal_timeout_button_id,
        )

    def _ensure_walking_mode(self, reason):
        # New navigation input should always release forced external hold,
        # no matter whether it came from middleware or a manual RViz click.
        if self.external_control_forced or self.external_control_active:
            self.external_control_forced = False
            self.external_control_active = False
            self.external_cmd = Twist()
            rospy.loginfo(
                "[NavOA] New navigation input received; releasing external control hold (%s)",
                reason,
            )

        if not self.goal_timeout_button_enabled:
            return
        if not self.goal_timeout_button_standing_active:
            return

        self._send_goal_timeout_button_pair(reason)
        self.goal_timeout_button_standing_active = False
        self.goal_timeout_button_sent = False
        rospy.loginfo(
            "[NavOA] New navigation input received; toggled walking mode with button id=%d",
            self.goal_timeout_button_id,
        )

    def generate_dune_points_markers_msg(self):

        marker_array = MarkerArray()

        if self.NavOA_planner.dune_points is None:
            return marker_array
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
            return marker_array
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
        navoa_core = NavOA_core()
        navoa_core.run()
    except rospy.ROSInterruptException:
        pass