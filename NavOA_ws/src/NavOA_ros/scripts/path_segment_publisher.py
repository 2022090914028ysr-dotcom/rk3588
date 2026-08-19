#!/usr/bin/env python
# -*- coding: utf-8 -*-

import copy
import rospy
import math
import tf
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalID
from std_msgs.msg import String
from std_srvs.srv import Empty

class PathToGoalFollower:
    def __init__(self):
        rospy.init_node('path_segment_publisher', anonymous=True)

        # === 参数配置 ===
        # 切分间隔：每隔 1.5 米提取一个关键点
        self.segment_length = rospy.get_param('~segment_length', 1.5) 
        
        # 中间点到达阈值：距离 0.5 米时切换下一个，保证行进流畅不卡顿
        self.reach_tolerance = rospy.get_param('~reach_tolerance', 0.5) 
        
        # 终点到达阈值：必须非常接近终点（0.15米）才算完成，防止终点处飘移
        self.final_tolerance = rospy.get_param('~final_tolerance', 0.15)

        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.map_frame = rospy.get_param('~map_frame', 'map')

        # === 可配置话题 (SCAN 集成时通过 launch 覆盖) ===
        self.path_topic = rospy.get_param(
            '~path_topic', '/move_base/GlobalPlanner/plan')
        self.split_goal_topic = rospy.get_param(
            '~split_goal_topic', '/split_goal')
        self.goal_input_topic = rospy.get_param(
            '~goal_input_topic', '/move_base_simple/goal')
        self.cancel_topic = rospy.get_param('~cancel_topic', '/move_base/cancel')

        # Generic navigation recovery.  This node sits between move_base and
        # NavOA, so the protection also applies to manual RViz goals.
        self.progress_timeout = max(
            0.0, float(rospy.get_param('~progress_timeout', 25.0)))
        self.progress_distance = max(
            0.01, float(rospy.get_param('~progress_distance', 0.20)))
        self.replan_cooldown = max(
            0.0, float(rospy.get_param('~replan_cooldown', 5.0)))
        self.max_replan_attempts = max(
            0, int(rospy.get_param('~max_replan_attempts', 3)))
        self.empty_plan_threshold = max(
            1, int(rospy.get_param('~empty_plan_threshold', 3)))
        self.clear_costmaps_service = rospy.get_param(
            '~clear_costmaps_service', '/move_base/clear_costmaps')
        self.clear_costmaps_timeout = max(
            0.1, float(rospy.get_param('~clear_costmaps_timeout', 2.0)))

        # === 变量初始化 ===
        self.waypoints = []        # 存储切分后的子目标
        self.current_goal_idx = 0  # 当前正在追踪的点索引
        self.active_path = False   # 是否处于激活状态
        self.tf_listener = tf.TransformListener()
        self.requested_goal = None
        self.empty_plan_count = 0
        self.replan_attempts = 0
        self.best_goal_distance = None
        self.last_progress_time = rospy.Time.now()
        self.last_replan_time = rospy.Time(0)
        self.recovery_in_progress = False
        self.pending_internal_goal_stamp = None

        # === 核心订阅与发布 ===
        
        # 1. 监听用户在 Rviz 的点击 (用于立即打断旧任务)
        rospy.Subscriber(self.goal_input_topic, PoseStamped, self.goal_reset_callback)

        # A move_base cancel must also stop the last split goal retained by
        # NavOA; otherwise NavOA continues driving after move_base has stopped.
        rospy.Subscriber(self.cancel_topic, GoalID, self.cancel_callback)

        # 2. 监听全局规划器生成的长路径 (用于生成新任务)
        rospy.Subscriber(self.path_topic, Path, self.path_callback)

        # 3. 发布给 NavOA 的切分点 (注意：Launch文件中需将 NavOA 的输入 remap 到这里)
        self.goal_pub = rospy.Publisher(self.split_goal_topic, PoseStamped, queue_size=1)
        self.goal_replan_pub = rospy.Publisher(
            self.goal_input_topic, PoseStamped, queue_size=1)
        self.cancel_pub = rospy.Publisher(self.cancel_topic, GoalID, queue_size=1)
        self.recovery_status_pub = rospy.Publisher(
            '/navoa/recovery_status', String, queue_size=1, latch=True)
        self.clear_costmaps = rospy.ServiceProxy(
            self.clear_costmaps_service, Empty)

        rospy.loginfo("Path Segment Publisher Initialized.")
        rospy.loginfo("Logic: Loose tolerance for mid-points (%.2fm), Strict for final (%.2fm).", 
                      self.reach_tolerance, self.final_tolerance)

        # 启动控制循环 (10Hz)
        rospy.Timer(rospy.Duration(0.1), self.control_loop)

    def goal_reset_callback(self, msg):
        """
        当用户点击新目标时触发。
        立即清空当前队列，防止机器人试图跑完旧路径的剩余点。
        """
        is_internal = (
            self.pending_internal_goal_stamp is not None and
            msg.header.stamp == self.pending_internal_goal_stamp)
        self.pending_internal_goal_stamp = None
        self.requested_goal = copy.deepcopy(msg)
        self.last_progress_time = rospy.Time.now()
        self.empty_plan_count = 0
        if is_internal:
            # The recovery path was already stopped before republishing.  Do
            # not clear a new global plan if this loopback callback arrives
            # after move_base has produced it.
            rospy.loginfo("Internally republished goal accepted for replanning.")
            return

        self.replan_attempts = 0
        self.best_goal_distance = None
        rospy.loginfo("New goal received; stopping the old split path.")
        self.active_path = False
        self.waypoints = []
        self.current_goal_idx = 0
        self.publish_hold_goal('new-goal')

    def cancel_callback(self, _msg):
        """Stop NavOA's retained split goal when move_base is cancelled."""
        self.active_path = False
        self.waypoints = []
        self.current_goal_idx = 0
        self.publish_hold_goal('move-base-cancel')
        self._publish_recovery_status('cancelled')

    def path_callback(self, msg):
        """
        当接收到全局路径时触发。
        """
        if len(msg.poses) == 0:
            self.empty_plan_count += 1
            self.active_path = False
            self.waypoints = []
            self.publish_hold_goal('empty-global-plan')
            rospy.logwarn_throttle(
                2.0, "Global planner returned an empty path (%d/%d).",
                self.empty_plan_count, self.empty_plan_threshold)
            if self.empty_plan_count >= self.empty_plan_threshold:
                self._trigger_replan('empty-global-plan', clear_costmaps=True)
            return

        self.empty_plan_count = 0

        rospy.loginfo("Received new global plan with %d poses.", len(msg.poses))

        # 1. 采样路径
        self.waypoints = self.sample_path(msg.poses, self.segment_length)
        
        # 2. 重置状态
        self.current_goal_idx = 0
        self.active_path = True
        
        rospy.loginfo("Path sampled to %d waypoints. Starting execution.", len(self.waypoints))
        
        # 3. 立即发布第一个点
        self.publish_current_goal()

    def sample_path(self, poses, interval):
        """
        对原始路径进行稀疏采样
        """
        sampled_goals = []
        if not poses:
            return sampled_goals

        last_added_pose = poses[0]
        # 起点可以不加，直接往远处走，防止机器人原地纠结
        
        for pose in poses:
            dist = self.calculate_distance(pose.pose.position, last_added_pose.pose.position)
            if dist >= interval:
                sampled_goals.append(pose)
                last_added_pose = pose
        
        # 【关键】强制加入终点，确保一定能走到最后
        final_pose = poses[-1]
        
        # 如果最后一个采样点距离终点太近，就替换掉；否则追加终点
        if not sampled_goals:
             sampled_goals.append(final_pose)
        elif self.calculate_distance(sampled_goals[-1].pose.position, final_pose.pose.position) > 0.1:
            sampled_goals.append(final_pose)

        return sampled_goals

    def control_loop(self, event):
        """
        循环检查：机器人走到哪里了？该切下一个点了吗？
        """
        if not self.active_path or not self.waypoints:
            return

        # 1. 获取机器人当前坐标
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.map_frame, self.base_frame, rospy.Time(0))
            robot_x, robot_y = trans[0], trans[1]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return

        self._monitor_navigation_progress(robot_x, robot_y)
        if not self.active_path:
            return

        # 2. 获取当前目标坐标
        if self.current_goal_idx >= len(self.waypoints):
            return 
            
        current_goal = self.waypoints[self.current_goal_idx]
        goal_x = current_goal.pose.position.x
        goal_y = current_goal.pose.position.y

        # 3. 计算距离
        dist = math.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)

        # 4. 判断逻辑 (核心优化部分)
        is_final_goal = (self.current_goal_idx == len(self.waypoints) - 1)
        
        # 根据是否是终点，决定使用哪个阈值
        current_threshold = self.final_tolerance if is_final_goal else self.reach_tolerance

        if dist < current_threshold:
            if not is_final_goal:
                # === 中间点：流畅切换 ===
                rospy.loginfo("Waypoint %d reached (Dist: %.2f). Moving to next.", self.current_goal_idx, dist)
                self.current_goal_idx += 1
                self.publish_current_goal()
            else:
                # === 终点：精确停止 ===
                rospy.loginfo("FINAL goal reached precisely (Dist: %.2f)! Mission Complete.", dist)
                self.active_path = False
                self.waypoints = []
                # 此时停止发布新目标，NavOA 会在最后一个点减速停止
        
        # 补充：为了防止 NavOA 丢失目标，如果是终点且尚未到达精度要求，可以周期性重发(可选)
        # if is_final_goal and self.active_path and (event.current_real.to_sec() % 1.0 < 0.1):
        #     self.publish_current_goal()

    def publish_current_goal(self):
        if self.current_goal_idx < len(self.waypoints):
            goal = self.waypoints[self.current_goal_idx]
            goal.header.stamp = rospy.Time.now()
            self.goal_pub.publish(goal)
            
            # 打印日志方便调试
            rospy.loginfo(">> Publishing goal %d/%d to /split_goal: x=%.2f, y=%.2f", 
                          self.current_goal_idx + 1, len(self.waypoints), 
                          goal.pose.position.x, goal.pose.position.y)

    def _monitor_navigation_progress(self, robot_x, robot_y):
        """Request a fresh global route when the robot makes no progress."""
        if (self.progress_timeout <= 0.0 or self.requested_goal is None or
                self.recovery_in_progress):
            return
        goal_frame = (self.requested_goal.header.frame_id or
                      self.map_frame).lstrip('/')
        if goal_frame != self.map_frame.lstrip('/'):
            rospy.logwarn_throttle(
                10.0, "Cannot monitor goal progress in frame '%s'", goal_frame)
            return

        goal_pos = self.requested_goal.pose.position
        distance = math.hypot(goal_pos.x - robot_x, goal_pos.y - robot_y)
        now = rospy.Time.now()
        if self.best_goal_distance is None:
            self.best_goal_distance = distance
            self.last_progress_time = now
            return

        if distance <= self.best_goal_distance - self.progress_distance:
            self.best_goal_distance = distance
            self.last_progress_time = now
            self.replan_attempts = 0
            return

        stalled_for = (now - self.last_progress_time).to_sec()
        if stalled_for >= self.progress_timeout:
            rospy.logwarn(
                "No goal progress for %.1fs (distance=%.2fm); requesting "
                "a different global route.", stalled_for, distance)
            # Do not clear live obstacle observations here.  The newly added
            # global obstacle layer must retain them so GlobalPlanner routes
            # around the blockage instead of selecting the same path again.
            self._trigger_replan('no-navigation-progress',
                                 clear_costmaps=False)

    def _trigger_replan(self, reason, clear_costmaps):
        if self.requested_goal is None or self.recovery_in_progress:
            return False

        now = rospy.Time.now()
        if ((now - self.last_replan_time).to_sec() < self.replan_cooldown):
            return False
        if self.replan_attempts >= self.max_replan_attempts:
            rospy.logerr(
                "Navigation recovery exhausted after %d attempts; stopping "
                "the current mission.", self.replan_attempts)
            self.active_path = False
            self.waypoints = []
            self.publish_hold_goal('recovery-exhausted')
            self._publish_recovery_status('failed:%s' % reason)
            self.cancel_pub.publish(GoalID())
            return False

        self.recovery_in_progress = True
        self.replan_attempts += 1
        self.last_replan_time = now
        self.last_progress_time = now
        self.active_path = False
        self.waypoints = []
        self.publish_hold_goal(reason)

        if clear_costmaps:
            try:
                rospy.wait_for_service(
                    self.clear_costmaps_service,
                    timeout=self.clear_costmaps_timeout)
                self.clear_costmaps()
                rospy.logwarn("Cleared move_base costmaps before replanning.")
            except (rospy.ROSException, rospy.ServiceException) as exc:
                rospy.logwarn("Failed to clear move_base costmaps: %s", exc)

        goal = copy.deepcopy(self.requested_goal)
        goal.header.stamp = rospy.Time.now()
        self.pending_internal_goal_stamp = goal.header.stamp
        self.goal_replan_pub.publish(goal)
        self.empty_plan_count = 0
        self._publish_recovery_status(
            'replan:%s:%d/%d' % (
                reason, self.replan_attempts, self.max_replan_attempts))
        rospy.logwarn(
            "Republished navigation goal for replanning (%s, attempt %d/%d).",
            reason, self.replan_attempts, self.max_replan_attempts)
        self.recovery_in_progress = False
        return True

    def publish_hold_goal(self, reason):
        """Send NavOA a goal at the current pose to stop stale-path motion."""
        try:
            trans, rot = self.tf_listener.lookupTransform(
                self.map_frame, self.base_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            rospy.logwarn_throttle(
                5.0, "Cannot publish NavOA hold goal: TF unavailable")
            return False

        hold = PoseStamped()
        hold.header.frame_id = self.map_frame
        hold.header.stamp = rospy.Time.now()
        hold.pose.position.x = trans[0]
        hold.pose.position.y = trans[1]
        hold.pose.position.z = trans[2]
        hold.pose.orientation.x = rot[0]
        hold.pose.orientation.y = rot[1]
        hold.pose.orientation.z = rot[2]
        hold.pose.orientation.w = rot[3]
        self.goal_pub.publish(hold)
        rospy.logwarn_throttle(
            2.0, "Holding NavOA at current pose (%s).", reason)
        return True

    def _publish_recovery_status(self, status):
        self.recovery_status_pub.publish(String(data=status))

    def calculate_distance(self, p1, p2):
        return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

if __name__ == '__main__':
    try:
        PathToGoalFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
