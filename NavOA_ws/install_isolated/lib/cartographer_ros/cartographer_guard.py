#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
cartographer_guard.py

功能：基于匹配分数和雷达稳定检测触发 Cartographer 重定位

状态机：
STATE_MONITOR     - 监控第一个trigger（匹配分数）
STATE_WAITING     - 等待第二个trigger（雷达稳定）
STATE_RECOVERING  - 执行重定位中
STATE_STABILIZING - 重定位后稳定期（锁定机器人，等待分数恢复）

方案 A：MONITOR 期间持续缓存最近一次 map -> base_link 位姿；
        do_recovery 时通过 StartTrajectory.initial_pose 注入，
        避免新轨迹从 (0,0,0) 起步导致 local/global 错位。
方案 E：RECOVERING + STABILIZING 期间持续发布零 Twist 并 latch holding=True；
        分数连续若干帧恢复后早释放，否则走 stabilizing_duration 超时兜底。

依赖的类：
- tf2_ros.Buffer / tf2_ros.TransformListener: 从 TF 树读取 map -> base_link
- geometry_msgs.msg.Pose: StartTrajectoryRequest.initial_pose 的载体
- geometry_msgs.msg.Twist: 锁定机器人时发布的零速度
- geometry_msgs.msg.TransformStamped: tf2 lookup_transform 的返回类型
- std_msgs.msg.Bool: 广播 holding 状态给下游导航/控制节点
- cartographer_ros_msgs.srv.FinishTrajectory / StartTrajectory: 重定位用的 cartographer 服务
"""

import math
import rospy
import tf2_ros
from geometry_msgs.msg import Pose, Twist, TransformStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
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

        # ========== 方案 A: 位姿缓存参数 ==========
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.pose_cache_rate_hz = rospy.get_param("~pose_cache_rate_hz", 2.0)

        # ========== 方案 E: 锁定机器人参数 ==========
        # hold_topic 留空可关闭零速度发布（仍会发 ~holding 状态）
        self.hold_topic = rospy.get_param("~hold_topic", "/cmd_vel")
        self.hold_rate_hz = rospy.get_param("~hold_rate_hz", 20.0)
        # 稳定期内分数连续多少帧 >= 阈值即视为收敛，提前释放
        self.score_recovery_frames = rospy.get_param("~score_recovery_frames", 5)

        # ========== 方案 D: AMCL 旁路全局定位 ==========
        # use_amcl=false 时整个 AMCL 分支不生效（兼容老 launch）
        self.use_amcl = rospy.get_param("~use_amcl", False)
        self.amcl_pose_topic = rospy.get_param("~amcl_pose_topic", "/amcl_pose")
        self.amcl_initial_topic = rospy.get_param("~amcl_initial_topic", "/initialpose")
        # 协方差和阈值：var(x)+var(y) 小于此值才信 AMCL 位姿
        self.amcl_cov_xy_max = rospy.get_param("~amcl_cov_xy_max", 0.5)
        # 等 AMCL 收敛的最大时间（秒）
        self.amcl_wait_timeout = rospy.get_param("~amcl_wait_timeout", 5.0)
        # 种子 AMCL 时使用的协方差（让 AMCL 在 ±2m / ±0.7rad 内搜索）
        self.amcl_seed_cov_xy = rospy.get_param("~amcl_seed_cov_xy", 4.0)
        self.amcl_seed_cov_yaw = rospy.get_param("~amcl_seed_cov_yaw", 0.5)

        # ========== 状态变量 ==========
        self.state = self.STATE_MONITOR
        self.score_low_count = 0      # 连续低分数计数
        self.scan_stable_count = 0    # 连续稳定计数
        self.score_recovery_count = 0 # 稳定期内连续高分数计数（方案 E 早释放）
        self.prev_scan = None         # 上一帧雷达数据
        self.last_recovery_time = 0   # 上次恢复时间
        self.stabilizing_start = 0    # 稳定期开始时间
        self.last_good_pose = None    # 方案 A: 缓存的最后可信 map->base_link 位姿

        # 方案 D: AMCL 旁路状态
        self.amcl_pose = None          # 最近一次 AMCL 输出的位姿（Pose 类型）
        self.amcl_cov_ok = False       # 当前 AMCL 协方差是否够小

        # ========== TF（方案 A） ==========
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # ========== 发布器（方案 E） ==========
        self.cmd_vel_pub = None
        if self.hold_topic:
            self.cmd_vel_pub = rospy.Publisher(self.hold_topic, Twist, queue_size=1)
        # latch=True 让晚启动的下游节点也能立刻拿到当前 holding 状态
        self.holding_pub = rospy.Publisher("~holding", Bool, queue_size=1, latch=True)
        self._publish_holding(False)

        # ========== 订阅 ==========
        rospy.Subscriber(self.score_topic, Float32, self.score_callback)
        rospy.Subscriber(self.scan_topic, LaserScan, self.scan_callback)

        # ========== 方案 D: AMCL 旁路 ==========
        self.amcl_initial_pub = None
        if self.use_amcl:
            rospy.Subscriber(self.amcl_pose_topic, PoseWithCovarianceStamped, self.amcl_callback)
            self.amcl_initial_pub = rospy.Publisher(
                self.amcl_initial_topic, PoseWithCovarianceStamped, queue_size=1)
            rospy.loginfo(
                f"AMCL bypass enabled: subscribe {self.amcl_pose_topic}, "
                f"publish {self.amcl_initial_topic}")

        # ========== 服务 ==========
        rospy.wait_for_service(self.finish_service)
        rospy.wait_for_service(self.start_service)
        self.finish = rospy.ServiceProxy(self.finish_service, FinishTrajectory)
        self.start = rospy.ServiceProxy(self.start_service, StartTrajectory)
        
        # 定时器 - 每秒检查一次状态
        self.timer = rospy.Timer(rospy.Duration(0.5), self.timer_callback)
        # 方案 A: 低频缓存 map -> base_link 位姿
        self.pose_timer = rospy.Timer(
            rospy.Duration(1.0 / self.pose_cache_rate_hz), self.cache_pose_callback)
        # 方案 E: 锁定期内持续发布零速度
        self.hold_timer = rospy.Timer(
            rospy.Duration(1.0 / self.hold_rate_hz), self.hold_callback)

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
            self._publish_holding(False)

        elif new_state == self.STATE_WAITING:
            rospy.loginfo("State: WAITING (trigger1 active, waiting for scan to stabilize)")
            self.reset_scan_monitor()

        elif new_state == self.STATE_RECOVERING:
            rospy.loginfo("State: RECOVERING (performing cartographer reset)")
            self._publish_holding(True)

        elif new_state == self.STATE_STABILIZING:
            self.stabilizing_start = rospy.Time.now().to_sec()
            rospy.loginfo(
                f"State: STABILIZING (max {self.stabilizing_duration}s, "
                f"early-release after {self.score_recovery_frames} good frames)")
            self.reset_scan_monitor()
            self.score_low_count = 0
            self.score_recovery_count = 0
            self._publish_holding(True)
    
    def _publish_holding(self, holding):
        """广播当前 holding 状态（latched）"""
        try:
            self.holding_pub.publish(Bool(data=bool(holding)))
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"Failed to publish holding: {e}")

    def cache_pose_callback(self, event):
        """方案 A: 仅在 MONITOR 状态下缓存最新 map -> base_link 位姿。

        进入 WAITING/RECOVERING/STABILIZING 后停止更新，
        保证注入回 cartographer 的初始位姿来自"还可信"的时段。
        """
        if self.state != self.STATE_MONITOR:
            return
        try:
            tf_stamped: TransformStamped = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame,
                rospy.Time(0), rospy.Duration(0.1))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logdebug(f"TF lookup ({self.map_frame}->{self.base_frame}) failed: {e}")
            return

        pose = Pose()
        pose.position.x = tf_stamped.transform.translation.x
        pose.position.y = tf_stamped.transform.translation.y
        pose.position.z = tf_stamped.transform.translation.z
        pose.orientation = tf_stamped.transform.rotation
        self.last_good_pose = pose

    def amcl_callback(self, msg):
        """方案 D: 缓存 AMCL 位姿 + 协方差质量判断。

        msg: geometry_msgs.PoseWithCovarianceStamped
        msg.pose.covariance 是 6x6 行优先：var(x)=[0], var(y)=[7], var(yaw)=[35]
        协方差小说明粒子收敛紧，位姿可信。
        """
        cov = msg.pose.covariance
        var_xx = cov[0]
        var_yy = cov[7]
        if (var_xx + var_yy) < self.amcl_cov_xy_max:
            self.amcl_pose = msg.pose.pose
            self.amcl_cov_ok = True
        else:
            self.amcl_cov_ok = False

    def _seed_amcl(self, pose):
        """方案 D: 用 cached_pose 给 AMCL 播种，加宽协方差让粒子在附近重新搜索。"""
        if self.amcl_initial_pub is None or pose is None:
            return
        seed = PoseWithCovarianceStamped()
        seed.header.frame_id = self.map_frame
        seed.header.stamp = rospy.Time.now()
        seed.pose.pose = pose
        # 6x6 协方差矩阵，只设对角线（其它默认 0）
        cov = [0.0] * 36
        cov[0]  = self.amcl_seed_cov_xy   # var(x)
        cov[7]  = self.amcl_seed_cov_xy   # var(y)
        cov[35] = self.amcl_seed_cov_yaw  # var(yaw)
        seed.pose.covariance = cov
        self.amcl_initial_pub.publish(seed)
        rospy.logwarn(
            f"Seeded AMCL at x={pose.position.x:.2f}, y={pose.position.y:.2f} "
            f"with cov_xy={self.amcl_seed_cov_xy}, cov_yaw={self.amcl_seed_cov_yaw}")

    def _wait_amcl_converge(self, timeout_sec):
        """方案 D: 阻塞等 AMCL 协方差降到阈值以下。返回 True 表示收敛了。"""
        # 重置 cov_ok，等新的高质量回调
        self.amcl_cov_ok = False
        deadline = rospy.Time.now() + rospy.Duration(timeout_sec)
        rate = rospy.Rate(10)
        while rospy.Time.now() < deadline and not rospy.is_shutdown():
            if self.amcl_cov_ok:
                return True
            rate.sleep()
        return False

    def hold_callback(self, event):
        """方案 E: 在 RECOVERING / STABILIZING 期间持续发布零速度。"""
        if self.state not in (self.STATE_RECOVERING, self.STATE_STABILIZING):
            return
        if self.cmd_vel_pub is None:
            return
        self.cmd_vel_pub.publish(Twist())  # 全零 Twist

    def timer_callback(self, event):
        """定时器 - 稳定期早释放 + 超时兜底"""
        if self.state != self.STATE_STABILIZING:
            return

        now = rospy.Time.now().to_sec()
        elapsed = now - self.stabilizing_start

        # 方案 E 早释放：分数恢复后立即解锁
        if self.score_recovery_count >= self.score_recovery_frames:
            rospy.loginfo(
                f"Score recovered ({self.score_recovery_count} frames in {elapsed:.1f}s), "
                f"early-release to MONITOR")
            self.set_state(self.STATE_MONITOR)
            return

        # 超时兜底：分数没回来也要释放，避免机器人被永久锁死
        if elapsed >= self.stabilizing_duration:
            rospy.logwarn(
                f"Stabilizing timeout ({elapsed:.1f}s) without score recovery, "
                f"releasing to MONITOR anyway")
            self.set_state(self.STATE_MONITOR)
    
    def score_callback(self, msg):
        """匹配分数回调"""
        if self.state == self.STATE_MONITOR:
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

        elif self.state == self.STATE_STABILIZING:
            # 方案 E: 监控分数恢复用于早释放
            if msg.data >= self.score_threshold:
                self.score_recovery_count += 1
                rospy.logdebug(
                    f"Stabilizing score good: {msg.data:.3f}, "
                    f"count={self.score_recovery_count}/{self.score_recovery_frames}")
            else:
                self.score_recovery_count = 0
    
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
                    if self.scan_stable_count == self.scan_stable_frames:
                        rospy.logwarn(f"TRIGGER2: Scan stable for {self.scan_stable_count} consecutive frames")
                    if self.scan_stable_count >= self.scan_stable_frames:
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
            # 节流：每 10s 最多打一次，避免刷屏
            rospy.logwarn_throttle(10.0, "Recovery in cooldown (10s), waiting...")
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
            req.relative_to_trajectory_id = 0

            # 选 initial_pose 的优先级：AMCL（方案 D）> cached_pose（方案 A）> cold start
            # 方案 D：先用 cached_pose 种子 AMCL，等它收敛后拿 AMCL 输出当 initial_pose
            initial_pose = None
            initial_source = "cold start"

            if self.use_amcl and self.amcl_initial_pub is not None:
                if self.last_good_pose is not None:
                    self._seed_amcl(self.last_good_pose)
                converged = self._wait_amcl_converge(self.amcl_wait_timeout)
                if converged and self.amcl_pose is not None:
                    initial_pose = self.amcl_pose
                    initial_source = "AMCL"
                else:
                    rospy.logwarn(
                        f"AMCL did not converge within {self.amcl_wait_timeout}s, "
                        f"falling back to cached pose")

            if initial_pose is None and self.last_good_pose is not None:
                initial_pose = self.last_good_pose
                initial_source = "cached_pose"

            if initial_pose is not None:
                req.use_initial_pose = False
                req.initial_pose = initial_pose
                p = initial_pose.position
                q = initial_pose.orientation
                # 从四元数算 yaw（绕 z 轴），公式 atan2(2(wz+xy), 1-2(y²+z²))
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                rospy.logwarn(
                    f"Using {initial_source} as initial: "
                    f"x={p.x:.2f}, y={p.y:.2f}, yaw={yaw:.2f}rad ({yaw*57.3:.0f}°)")
            else:
                req.use_initial_pose = False
                rospy.logwarn("No initial pose available, cold start global localization")

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