#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
cartographer_guard.py

功能：基于匹配分数和雷达稳定检测触发 Cartographer 重定位

状态机：
STATE_MONITOR    - 监控第一个trigger（匹配分数）
STATE_WAITING    - 等待第二个trigger（雷达稳定）
STATE_RECOVERING - 执行重定位中
STATE_STABILIZING - 重定位后稳定期（不监控分数）
"""

import math
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from cartographer_ros_msgs.srv import FinishTrajectory, StartTrajectory, FinishTrajectoryRequest, StartTrajectoryRequest


class CartographerGuard:
    # 状态定义
    STATE_MONITOR = 0      # 监控第一个trigger
    STATE_WAITING = 1      # 等待第二个trigger
    STATE_RECOVERING = 2   # 执行重定位中
    STATE_STABILIZING = 3  # 重定位后稳定期

    def __init__(self):
        # ========== 参数 ==========
        self.score_topic = rospy.get_param("~score_topic", "/matching_score")
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        
        # 分数触发条件
        self.score_threshold = rospy.get_param("~score_threshold", 0.8)
        self.score_consecutive_frames = rospy.get_param("~score_consecutive_frames", 10)
        
        # 雷达触发条件
        self.scan_stable_frames = rospy.get_param("~scan_stable_frames", 10)
        self.scan_change_threshold = rospy.get_param("~scan_change_threshold", 0.05)
        
        # 稳定期时长（秒）- 重定位后等待定位稳定
        self.stabilizing_duration = rospy.get_param("~stabilizing_duration", 5.0)
        
        # Cartographer 服务
        self.finish_service = rospy.get_param("~finish_service", "/finish_trajectory")
        self.start_service = rospy.get_param("~start_service", "/start_trajectory")
        self.config_dir = rospy.get_param("~config_dir", "")
        self.config_file = rospy.get_param("~config_file", "")
        self.trajectory_id = rospy.get_param("~trajectory_id", 1)
        
        # ========== 状态变量 ==========
        self.state = self.STATE_MONITOR
        self.score_low_count = 0      # 连续低分数计数
        self.scan_stable_count = 0    # 连续稳定计数
        self.prev_scan = None         # 上一帧雷达数据
        self.last_recovery_time = 0   # 上次恢复时间
        self.stabilizing_start = 0    # 稳定期开始时间
        
        # ========== 订阅 ==========
        rospy.Subscriber(self.score_topic, Float32, self.score_callback)
        rospy.Subscriber(self.scan_topic, LaserScan, self.scan_callback)
        
        # ========== 服务 ==========
        rospy.wait_for_service(self.finish_service)
        rospy.wait_for_service(self.start_service)
        self.finish = rospy.ServiceProxy(self.finish_service, FinishTrajectory)
        self.start = rospy.ServiceProxy(self.start_service, StartTrajectory)
        
        # 定时器 - 每秒检查一次状态
        self.timer = rospy.Timer(rospy.Duration(0.5), self.timer_callback)
        
        rospy.loginfo("CartographerGuard initialized, state=MONITOR")
    
    def reset_scan_monitor(self):
        """重置雷达监控状态"""
        self.scan_stable_count = 0
        self.prev_scan = None
    
    def set_state(self, new_state):
        """状态切换"""
        old_state = self.state
        self.state = new_state
        
        if new_state == self.STATE_MONITOR:
            rospy.loginfo("State: MONITOR (waiting for score to drop)")
            self.score_low_count = 0
            self.reset_scan_monitor()
            
        elif new_state == self.STATE_WAITING:
            rospy.loginfo("State: WAITING (trigger1 active, waiting for scan to stabilize)")
            self.reset_scan_monitor()
            
        elif new_state == self.STATE_RECOVERING:
            rospy.loginfo("State: RECOVERING (performing cartographer reset)")
            
        elif new_state == self.STATE_STABILIZING:
            self.stabilizing_start = rospy.Time.now().to_sec()
            rospy.loginfo(f"State: STABILIZING (waiting {self.stabilizing_duration}s for localization to stabilize)")
            self.reset_scan_monitor()
            self.score_low_count = 0
    
    def timer_callback(self, event):
        """定时器 - 处理稳定期超时"""
        now = rospy.Time.now().to_sec()
        
        # 稳定期超时，回到监控状态
        if self.state == self.STATE_STABILIZING:
            if now - self.stabilizing_start >= self.stabilizing_duration:
                rospy.loginfo("Stabilizing period finished, returning to MONITOR")
                self.set_state(self.STATE_MONITOR)
    
    def score_callback(self, msg):
        """匹配分数回调"""
        # 只在 MONITOR 状态下处理分数
        if self.state != self.STATE_MONITOR:
            return
        
        if msg.data < self.score_threshold:
            self.score_low_count += 1
            rospy.logdebug(f"Score low: {msg.data:.3f} < {self.score_threshold}, count={self.score_low_count}/{self.score_consecutive_frames}")
            
            # 达到阈值，触发第一个trigger
            if self.score_low_count >= self.score_consecutive_frames:
                rospy.logwarn(f"TRIGGER1: Score low for {self.score_low_count} consecutive frames")
                self.set_state(self.STATE_WAITING)
        else:
            # 分数恢复正常，重置计数
            if self.score_low_count > 0:
                rospy.loginfo(f"Score recovered to {msg.data:.3f}, resetting count")
            self.score_low_count = 0
    
    def scan_callback(self, msg):
        """雷达回调 - 只在 WAITING 状态下处理"""
        if self.state != self.STATE_WAITING:
            return
        
        # 采样雷达数据（每5个点取1个，降低计算量）
        step = 5
        ranges = []
        for i, r in enumerate(msg.ranges):
            if i % step == 0 and not (math.isinf(r) or math.isnan(r)) and 0.1 < r < 10.0:
                ranges.append(r)
        
        if not ranges:
            return
        
        # 与上一帧比较
        if self.prev_scan is not None:
            min_len = min(len(ranges), len(self.prev_scan))
            if min_len > 0:
                total_diff = 0.0
                for i in range(min_len):
                    total_diff += abs(ranges[i] - self.prev_scan[i])
                diff = total_diff / min_len
                
                if diff < self.scan_change_threshold:
                    self.scan_stable_count += 1
                    rospy.logdebug(f"Scan stable: diff={diff:.4f}, count={self.scan_stable_count}/{self.scan_stable_frames}")
                    
                    # 达到稳定阈值，触发第二个trigger
                    if self.scan_stable_count >= self.scan_stable_frames:
                        rospy.logwarn(f"TRIGGER2: Scan stable for {self.scan_stable_count} consecutive frames")
                        self.do_recovery()
                else:
                    # 不稳定，重置计数
                    if self.scan_stable_count > 0:
                        rospy.logdebug(f"Scan unstable: diff={diff:.4f}, resetting count")
                    self.scan_stable_count = 0
        else:
            # 第一帧，重置计数
            self.scan_stable_count = 0
        
        self.prev_scan = ranges
    
    def do_recovery(self):
        """执行重定位"""
        now = rospy.Time.now().to_sec()
        
        # 冷却检查（防止频繁重定位）
        if now - self.last_recovery_time < 10.0:
            rospy.logwarn("Recovery in cooldown (10s), waiting...")
            # 回到等待状态
            if self.state != self.STATE_WAITING:
                self.set_state(self.STATE_WAITING)
            return
        
        self.set_state(self.STATE_RECOVERING)
        
        rospy.logwarn("=" * 50)
        rospy.logwarn("Starting cartographer recovery...")
        
        try:
            # 结束当前轨迹
            req = FinishTrajectoryRequest()
            req.trajectory_id = self.trajectory_id
            self.finish(req)
            rospy.logwarn(f"Finished trajectory {self.trajectory_id}")
            
            rospy.sleep(0.5)
            
            # 启动新轨迹
            req = StartTrajectoryRequest()
            req.configuration_directory = self.config_dir
            req.configuration_basename = self.config_file
            req.use_initial_pose = False
            req.relative_to_trajectory_id = 0
            resp = self.start(req)
            
            self.trajectory_id = resp.trajectory_id
            rospy.logwarn(f"Started new trajectory {self.trajectory_id}")
            rospy.logwarn("Recovery completed!")
            rospy.logwarn("=" * 50)
            
            # 记录恢复时间
            self.last_recovery_time = now
            
            # 进入稳定期，让 Cartographer 有时间重建定位
            self.set_state(self.STATE_STABILIZING)
            
        except Exception as e:
            rospy.logerr(f"Recovery failed: {e}")
            # 恢复失败，直接回到监控状态
            self.set_state(self.STATE_MONITOR)


if __name__ == "__main__":
    rospy.init_node("cartographer_guard")
    node = CartographerGuard()
    rospy.spin()