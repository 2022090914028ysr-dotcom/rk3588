#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import tf
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class PathToGoalFollower:
    def __init__(self):
        rospy.init_node('path_segment_publisher', anonymous=True)

        # === 参数配置 ===
        # 切分间隔：每隔 1.5 米提取一个关键点
        self.segment_length = rospy.get_param('~segment_length', 1.5) 
        
        # 中间点到达阈值：距离 0.5 米时切换下一个，保证行进流畅不卡顿
        self.reach_tolerance = rospy.get_param('~reach_tolerance', 0.5) 
        
        # 终点到达阈值：必须非常接近终点（0.15米）才算完成，防止终点处飘移
        self.final_tolerance = 0.15 
        
        self.base_frame = rospy.get_param('~base_frame', 'base_footprint')
        self.map_frame = rospy.get_param('~map_frame', 'map')

        # === 变量初始化 ===
        self.waypoints = []        # 存储切分后的子目标
        self.current_goal_idx = 0  # 当前正在追踪的点索引
        self.active_path = False   # 是否处于激活状态
        self.tf_listener = tf.TransformListener()

        # === 核心订阅与发布 ===
        
        # 1. 监听用户在 Rviz 的点击 (用于立即打断旧任务)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_reset_callback)

        # 2. 监听全局规划器生成的长路径 (用于生成新任务)
        rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, self.path_callback)
        
        # 3. 发布给 NavOA 的切分点 (注意：Launch文件中需将 NavOA 的输入 remap 到这里)
        self.goal_pub = rospy.Publisher('/split_goal', PoseStamped, queue_size=1)

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
        rospy.loginfo("New goal received! Resetting current path immediately...")
        self.active_path = False
        self.waypoints = []
        self.current_goal_idx = 0

    def path_callback(self, msg):
        """
        当接收到全局路径时触发。
        """
        if len(msg.poses) == 0:
            return

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

    def calculate_distance(self, p1, p2):
        return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

if __name__ == '__main__':
    try:
        PathToGoalFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass