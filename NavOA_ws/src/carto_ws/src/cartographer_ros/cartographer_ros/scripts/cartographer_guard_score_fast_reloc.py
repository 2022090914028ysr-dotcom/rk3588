#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
cartographer_trajectory_manager_score.py

Cartographer 轨迹管理工具（Score 版本），提供以下功能：
  1. 查阅活跃状态的轨迹 ID
  2. 获取 map→base_footprint 变换（监听 TF 的 map→odom 和 odom→base_footprint 并合成）
  3. 关闭最新的活跃轨迹
  4. 按照指定配置文件创建新轨迹
  5. 按照指定配置文件和初始位姿创建新轨迹
  6. Score / Offset / Rotation 检测，判断机器人位姿是否出现问题并触发恢复流程

与 cartographer_trajectory_manager.py 的区别：
  - 移除了 IMU 摔倒检测（IMU 硬件损坏）
  - 改用 matching score、map→base_footprint 位置跳变、yaw 跳变三种检测方式
  - 恢复流程仍使用 Cartographer 服务（FinishTrajectory / StartTrajectory），
    而非杀节点 hard reset
"""

import math
import threading

import os
import rospy
import rospkg
import tf2_ros
from cartographer_ros_msgs.srv import (
    StartTrajectory,
    StartTrajectoryRequest,
    FinishTrajectory,
    FinishTrajectoryRequest,
    GetTrajectoryStates,
    GetTrajectoryStatesRequest,
)
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class CartographerTrajectoryManagerScore:
    """Cartographer 轨迹管理 + Score/Offset/Rotation 位姿异常检测。"""

    # ── 状态机 ──────────────────────────────────────────────────────────
    STATE_IDLE       = 0   # 监控中
    STATE_RECOVERING = 1   # 正在执行恢复流程

    def __init__(self):
        # ── 服务名称 ────────────────────────────────────────────────────
        self.start_trajectory_service  = rospy.get_param("~start_service",   "/start_trajectory")
        self.finish_trajectory_service = rospy.get_param("~finish_service",  "/finish_trajectory")
        self.get_states_service        = rospy.get_param("~get_states_service", "/get_trajectory_states")

        # ── 配置 ────────────────────────────────────────────────────────
        self.config_dir  = rospy.get_param("~config_dir",
            os.path.join(rospkg.RosPack().get_path('cartographer_ros'), 'configuration_files'))
        self.config_file            = rospy.get_param("~config_file",            "livox_2d_localization_old.lua")
        self.config_file_fast_reloc = rospy.get_param("~config_file_fast_reloc", "livox_2d_localization_fast_reloc.lua")

        # ── TF 帧 ───────────────────────────────────────────────────────
        self.map_frame         = rospy.get_param("~map_frame",         "map")
        self.odom_frame        = rospy.get_param("~odom_frame",        "odom")
        self.robot_frame       = rospy.get_param("~robot_frame",       "base_footprint")
        self.tf_lookup_timeout = rospy.get_param("~tf_lookup_timeout",  0.1)

        # ==================================================================
        # Score-based 检测
        # ==================================================================
        self.score_enabled            = rospy.get_param("~score_enabled", True)
        self.score_topic              = rospy.get_param("~score_topic", "/matching_score")
        self.score_threshold          = rospy.get_param("~score_threshold", 0.40)
        self.score_consecutive_frames = rospy.get_param("~score_consecutive_frames", 10)

        # ==================================================================
        # Offset-based 检测（map→base_footprint 位置跳变）
        #   仅在 cartographer 启动 startup_delay 秒后激活
        # ==================================================================
        self.offset_enabled            = rospy.get_param("~offset_enabled", True)
        self.offset_threshold          = rospy.get_param("~offset_threshold", 0.50)
        self.offset_consecutive_frames = rospy.get_param("~offset_consecutive_frames", 3)
        self.offset_startup_delay      = rospy.get_param("~offset_startup_delay", 1200.0)

        # ==================================================================
        # Rotation-based 检测（map→base_footprint yaw 跳变）
        #   与 offset 共用 startup_delay
        # ==================================================================
        self.rotation_enabled            = rospy.get_param("~rotation_enabled", True)
        self.rotation_threshold_deg      = rospy.get_param("~rotation_threshold_deg", 90.0)
        self.rotation_consecutive_frames = rospy.get_param("~rotation_consecutive_frames", 3)

        # ==================================================================
        # 共享冷却时间（score / offset / rotation 路径共用）
        # ==================================================================
        self.reset_cooldown = rospy.get_param("~reset_cooldown", 2.0)

        # ==================================================================
        # 最大空闲超时：若超过此秒数未发生恢复，强制执行一次恢复。
        # 0.0（默认）= 禁用。
        # ==================================================================
        self.max_idle_timeout = rospy.get_param("~max_idle_timeout", 0.0)

        # ── 恢复流程参数 ──────────────────────────────────────────────────
        self.recovery_pose_retry_interval = rospy.get_param("~recovery_pose_retry_interval", 1.0)
        self.recovery_pose_max_retries    = rospy.get_param("~recovery_pose_max_retries",    30)
        self.recovery_settle_duration     = rospy.get_param("~recovery_settle_duration",     10.0)

        # ==================================================================
        # 恢复流程中等待 score 恢复参数
        #   启动 fast_reloc 后，不再固定等待 settle_duration 秒，
        #   而是轮询 /matching_score，直到达到此阈值或超时。
        # ==================================================================
        self.recovery_score_ok_threshold = rospy.get_param("~recovery_score_ok_threshold", 0.65)
        # After fast_reloc starts, wait for this many consecutive good-score
        # frames before proceeding to the old-lua swap (no timeout).
        self.recovery_consecutive_good  = rospy.get_param("~recovery_consecutive_good", 10)

        # ── 内部状态 ────────────────────────────────────────────────────
        self._lock = threading.Lock()
        self.state = self.STATE_IDLE

        self.last_reset_time = rospy.Time.now().to_sec()
        self.score_low_count = 0
        self.latest_score    = None

        self.node_start_time          = rospy.Time.now().to_sec()
        self.last_position            = None
        self.offset_violation_count   = 0
        self.last_yaw                 = None
        self.rotation_violation_count = 0

        # ── TF ──────────────────────────────────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # ── Service proxies ─────────────────────────────────────────────
        self._start_proxy  = None
        self._finish_proxy = None
        self._states_proxy = None

        # ── 订阅 ─────────────────────────────────────────────────────────
        if self.score_enabled:
            rospy.Subscriber(self.score_topic, Float32, self.score_callback)
        if self.offset_enabled or self.rotation_enabled:
            rospy.Timer(rospy.Duration(0.1), self.offset_timer_callback)

        # ── 最大空闲超时定时器 ────────────────────────────────────────────
        if self.max_idle_timeout > 0.0:
            rospy.Timer(rospy.Duration(10.0), self.idle_timeout_callback)

        rospy.loginfo(
            "CartographerTrajectoryManagerScore: ready. "
            f"start={self.start_trajectory_service}, "
            f"finish={self.finish_trajectory_service}, "
            f"states={self.get_states_service}. "
            f"Score={'ON' if self.score_enabled else 'OFF'} "
            f"(thr={self.score_threshold:.2f}, frames={self.score_consecutive_frames}), "
            f"Offset={'ON' if self.offset_enabled else 'OFF'} "
            f"(thr={self.offset_threshold:.2f}m, frames={self.offset_consecutive_frames}, "
            f"delay={self.offset_startup_delay:.0f}s), "
            f"Rotation={'ON' if self.rotation_enabled else 'OFF'} "
            f"(thr={self.rotation_threshold_deg:.0f}deg, "
            f"frames={self.rotation_consecutive_frames}), "
            f"MaxIdleTimeout={'ON' if self.max_idle_timeout > 0 else 'OFF'}"
            f"{f' ({self.max_idle_timeout:.0f}s)' if self.max_idle_timeout > 0 else ''}")

    # ═══════════════════════════════════════════════════════════════════════
    # 1. 查阅活跃状态的轨迹 ID
    # ═══════════════════════════════════════════════════════════════════════

    def get_active_trajectory_ids(self):
        """调用 GetTrajectoryStates 服务，返回活跃轨迹 ID 列表。

        Returns:
            list[int]: 活跃轨迹 ID 列表，失败时返回 None。
        """
        try:
            rospy.wait_for_service(self.get_states_service, timeout=5.0)
        except rospy.ROSException as exc:
            rospy.logerr(f"get_active_trajectory_ids: service unavailable: {exc}")
            return None

        if self._states_proxy is None:
            self._states_proxy = rospy.ServiceProxy(
                self.get_states_service, GetTrajectoryStates)

        try:
            resp = self._states_proxy(GetTrajectoryStatesRequest())
            # resp.trajectory_states 是 TrajectoryStates 消息对象，
            # 内含平行数组 trajectory_id[] 和 trajectory_state[]，
            # 而非可迭代的对象列表。
            ts = resp.trajectory_states
            ids = [tid for tid, st in zip(ts.trajectory_id, ts.trajectory_state)
                   if st == ts.ACTIVE]
            rospy.loginfo(f"get_active_trajectory_ids: {ids}")
            return ids
        except Exception as exc:
            rospy.logerr(f"get_active_trajectory_ids: call failed: {exc}")
            return None

    # ═══════════════════════════════════════════════════════════════════════
    # 2. 获取 map→base_footprint 变换
    # ═══════════════════════════════════════════════════════════════════════

    def get_map_to_base_footprint(self):
        """监听 TF，合成 map→odom 和 odom→base_footprint，返回 map→base_footprint。

        返回 (x, y, yaw_rad) 元组，失败时返回 None。
        """
        try:
            # 直接查找 map→base_footprint（TF2 会自动合成链）
            trans = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame,
                rospy.Time(0), rospy.Duration(self.tf_lookup_timeout))
            x   = trans.transform.translation.x
            y   = trans.transform.translation.y
            q   = trans.transform.rotation
            yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
            rospy.loginfo(
                f"get_map_to_base_footprint (direct): "
                f"x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.1f}deg")
            return (x, y, yaw)
        except Exception as exc_direct:
            # 直接查找失败，尝试手动合成 map→odom × odom→base_footprint
            rospy.logwarn(
                f"get_map_to_base_footprint: direct lookup failed ({exc_direct}), "
                "trying manual composition map→odom × odom→base_footprint")
            try:
                map_to_odom = self.tf_buffer.lookup_transform(
                    self.map_frame, self.odom_frame,
                    rospy.Time(0), rospy.Duration(self.tf_lookup_timeout))
                odom_to_base = self.tf_buffer.lookup_transform(
                    self.odom_frame, self.robot_frame,
                    rospy.Time(0), rospy.Duration(self.tf_lookup_timeout))

                # 合成变换
                x = (map_to_odom.transform.translation.x
                     + odom_to_base.transform.translation.x)
                y = (map_to_odom.transform.translation.y
                     + odom_to_base.transform.translation.y)

                # 简化处理：取 odom→base_footprint 的 yaw
                q_base = odom_to_base.transform.rotation
                yaw = euler_from_quaternion([q_base.x, q_base.y, q_base.z, q_base.w])[2]

                rospy.loginfo(
                    f"get_map_to_base_footprint (composed): "
                    f"x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.1f}deg")
                return (x, y, yaw)
            except Exception as exc_compose:
                rospy.logerr(
                    f"get_map_to_base_footprint: composition also failed: {exc_compose}")
                return None

    # ═══════════════════════════════════════════════════════════════════════
    # 3. 关闭最新的活跃轨迹
    # ═══════════════════════════════════════════════════════════════════════

    def finish_latest_trajectory(self):
        """关闭最新的活跃轨迹（ID 最大的那个）。

        Returns:
            int: 被关闭的轨迹 ID，失败时返回 None。
        """
        ids = self.get_active_trajectory_ids()
        if not ids:
            rospy.logwarn("finish_latest_trajectory: no active trajectories.")
            return None

        latest_id = max(ids)
        rospy.loginfo(f"finish_latest_trajectory: finishing trajectory {latest_id}")

        try:
            rospy.wait_for_service(self.finish_trajectory_service, timeout=5.0)
        except rospy.ROSException as exc:
            rospy.logerr(f"finish_latest_trajectory: service unavailable: {exc}")
            return None

        if self._finish_proxy is None:
            self._finish_proxy = rospy.ServiceProxy(
                self.finish_trajectory_service, FinishTrajectory)

        try:
            req = FinishTrajectoryRequest()
            req.trajectory_id = latest_id
            resp = self._finish_proxy(req)
            if resp.status.code == 0:
                rospy.loginfo(
                    f"finish_latest_trajectory: trajectory {latest_id} finished OK.")
                return latest_id
            else:
                rospy.logerr(
                    f"finish_latest_trajectory: trajectory {latest_id} "
                    f"finish failed: {resp.status.message}")
                return None
        except Exception as exc:
            rospy.logerr(f"finish_latest_trajectory: call failed: {exc}")
            return None

    # ═══════════════════════════════════════════════════════════════════════
    # 4. 按照指定配置文件创建新轨迹（无初始位姿）
    # ═══════════════════════════════════════════════════════════════════════

    def start_trajectory(self, config_dir=None, config_file=None):
        """按照指定配置文件创建新轨迹（不使用初始位姿）。

        Args:
            config_dir:  配置目录路径，为 None 时使用默认值。
            config_file: 配置文件名，为 None 时使用默认值。

        Returns:
            int: 新轨迹 ID，失败时返回 None。
        """
        cdir  = config_dir  if config_dir  is not None else self.config_dir
        cfile = config_file if config_file is not None else self.config_file

        if not cdir or not cfile:
            rospy.logerr("start_trajectory: config_dir or config_file is empty.")
            return None

        try:
            rospy.wait_for_service(self.start_trajectory_service, timeout=5.0)
        except rospy.ROSException as exc:
            rospy.logerr(f"start_trajectory: service unavailable: {exc}")
            return None

        if self._start_proxy is None:
            self._start_proxy = rospy.ServiceProxy(
                self.start_trajectory_service, StartTrajectory)

        try:
            req = StartTrajectoryRequest()
            req.configuration_directory = cdir
            req.configuration_basename   = cfile
            req.use_initial_pose         = False

            resp = self._start_proxy(req)
            if resp.status.code == 0:
                rospy.loginfo(
                    f"start_trajectory: new trajectory_id={resp.trajectory_id}")
                return resp.trajectory_id
            else:
                rospy.logerr(
                    f"start_trajectory: failed: {resp.status.message}")
                return None
        except Exception as exc:
            rospy.logerr(f"start_trajectory: call failed: {exc}")
            return None

    # ═══════════════════════════════════════════════════════════════════════
    # 5. 按照指定配置文件和初始位姿创建新轨迹
    # ═══════════════════════════════════════════════════════════════════════

    def start_trajectory_with_pose(
        self,
        x, y, yaw_deg,
        config_dir=None,
        config_file=None,
        relative_to_trajectory_id=0,
    ):
        """按照指定配置文件和初始位姿创建新轨迹。

        Args:
            x:                       初始 x 坐标 (m)。
            y:                       初始 y 坐标 (m)。
            yaw_deg:                 初始偏航角 (度)。
            config_dir:              配置目录路径，为 None 时使用默认值。
            config_file:             配置文件名，为 None 时使用默认值。
            relative_to_trajectory_id: 相对轨迹 ID，默认 0。

        Returns:
            int: 新轨迹 ID，失败时返回 None。
        """
        cdir  = config_dir  if config_dir  is not None else self.config_dir
        cfile = config_file if config_file is not None else self.config_file

        if not cdir or not cfile:
            rospy.logerr("start_trajectory_with_pose: config_dir or config_file is empty.")
            return None

        try:
            rospy.wait_for_service(self.start_trajectory_service, timeout=5.0)
        except rospy.ROSException as exc:
            rospy.logerr(f"start_trajectory_with_pose: service unavailable: {exc}")
            return None

        if self._start_proxy is None:
            self._start_proxy = rospy.ServiceProxy(
                self.start_trajectory_service, StartTrajectory)

        try:
            yaw_rad = math.radians(yaw_deg)
            qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw_rad)

            req = StartTrajectoryRequest()
            req.configuration_directory   = cdir
            req.configuration_basename     = cfile
            req.use_initial_pose           = True
            req.relative_to_trajectory_id  = relative_to_trajectory_id
            req.initial_pose.position.x    = float(x)
            req.initial_pose.position.y    = float(y)
            req.initial_pose.position.z    = 0.0
            req.initial_pose.orientation.x = float(qx)
            req.initial_pose.orientation.y = float(qy)
            req.initial_pose.orientation.z = float(qz)
            req.initial_pose.orientation.w = float(qw)

            resp = self._start_proxy(req)
            if resp.status.code == 0:
                rospy.loginfo(
                    f"start_trajectory_with_pose: new trajectory_id={resp.trajectory_id}, "
                    f"x={x:.2f}, y={y:.2f}, yaw={yaw_deg:.1f}deg")
                return resp.trajectory_id
            else:
                rospy.logerr(
                    f"start_trajectory_with_pose: failed: {resp.status.message}")
                return None
        except Exception as exc:
            rospy.logerr(f"start_trajectory_with_pose: call failed: {exc}")
            return None

    # ═══════════════════════════════════════════════════════════════════════
    # 6. 恢复流程
    # ═══════════════════════════════════════════════════════════════════════

    def _trigger_recovery(self, reason, source="SCORE"):
        """触发恢复流程（线程安全）。

        Args:
            reason:  触发原因描述。
            source:  触发来源（SCORE / OFFSET / ROTATION / IDLE_TIMEOUT）。
        """
        with self._lock:
            if self.state != self.STATE_IDLE:
                rospy.logwarn(
                    f"[{source}] trigger ignored: already in state {self.state}")
                return
            self.state = self.STATE_RECOVERING
            self.last_reset_time = rospy.Time.now().to_sec()
            # 重置检测计数器
            self.score_low_count = 0
            self.last_position = None
            self.offset_violation_count = 0
            self.last_yaw = None
            self.rotation_violation_count = 0

        rospy.logwarn(f"[{source}] RECOVERY TRIGGERED: {reason}")
        t = threading.Thread(
            target=self._recovery_sequence,
            args=(reason, source),
            daemon=True)
        t.start()

    def _recovery_sequence(self, reason, source="SCORE"):
        """位姿异常后的恢复流程（在独立线程中执行）：

        1. 等待 score ≥ recovery_score_ok_threshold 连续 N 帧（无超时，任何下降重置计数）
        2. 关闭最新的活跃轨迹
        3. 启动 fast_reloc 轨迹（盲启，无初始位姿）
        4. 等待 10s 让 fast_reloc 稳定
        5. 获取当前 map→base_footprint 位姿
        6. 关闭 fast_reloc 轨迹
        7. 使用步骤 5 捕获的位姿 + 默认配置启动新轨迹
        """
        rospy.logwarn(f"[{source}] RECOVERY: === 位姿恢复流程开始 === ({reason})")

        # ── 步骤 1: 等待 score 恢复（旧轨迹仍在运行）─────────────────────
        rospy.logwarn(
            f"[{source}] RECOVERY [1/7]: 等待 score ≥ {self.recovery_score_ok_threshold} "
            f"连续 {self.recovery_consecutive_good} 帧...")
        good_count = 0
        while not rospy.is_shutdown():
            s = self.latest_score
            if s is not None and s >= self.recovery_score_ok_threshold:
                good_count += 1
                if good_count >= self.recovery_consecutive_good:
                    rospy.logwarn(
                        f"[{source}] RECOVERY [1/7]: score={s:.3f} ≥ "
                        f"{self.recovery_score_ok_threshold} "
                        f"连续 {good_count} 帧，定位恢复。")
                    break
                rospy.loginfo_throttle(
                    2.0,
                    f"[{source}] RECOVERY [1/7]: score={s:.3f} ≥ threshold, "
                    f"计数 {good_count}/{self.recovery_consecutive_good}")
            else:
                if good_count > 0:
                    rospy.logwarn(
                        f"[{source}] RECOVERY [1/7]: score={'N/A' if s is None else f'{s:.3f}'} "
                        f"低于阈值，重置计数 (之前已累积 {good_count})")
                good_count = 0
            rospy.sleep(0.2)

        # ── 步骤 2: 关闭最新的活跃轨迹 ──────────────────────────────────
        rospy.logwarn(f"[{source}] RECOVERY [2/7]: 关闭最新的活跃轨迹...")
        finished_id = self.finish_latest_trajectory()
        if finished_id is None:
            rospy.logwarn(f"[{source}] RECOVERY [2/7]: 没有活跃轨迹可关闭，继续流程。")
        else:
            rospy.logwarn(f"[{source}] RECOVERY [2/7]: 已关闭轨迹 {finished_id}。")
        rospy.sleep(1.0)

        # ── 步骤 3: 启动 fast_reloc 轨迹（无初始位姿）─────────────────────
        rospy.logwarn(f"[{source}] RECOVERY [3/7]: 启动 fast_reloc 轨迹（盲启，无初始位姿）...")
        fast_tid = self.start_trajectory(
            config_file=self.config_file_fast_reloc)
        if fast_tid is None:
            rospy.logerr(f"[{source}] RECOVERY [3/7]: fast_reloc 轨迹启动失败，恢复流程中止。")
            with self._lock:
                self.state = self.STATE_IDLE
            return
        rospy.logwarn(f"[{source}] RECOVERY [3/7]: fast_reloc 轨迹已启动，trajectory_id={fast_tid}。")

        # ── 步骤 4: 等待 10s 让 fast_reloc 稳定 ──────────────────────────
        rospy.logwarn(f"[{source}] RECOVERY [4/7]: 等待 10s 让 fast_reloc 稳定...")
        rospy.sleep(60.0)

        # ── 步骤 5: 获取当前位姿（关闭 fast_reloc 之前）───────────────────
        rospy.logwarn(f"[{source}] RECOVERY [5/7]: 正在捕获当前位姿...")
        pose = None
        for attempt in range(1, self.recovery_pose_max_retries + 1):
            if rospy.is_shutdown():
                return
            pose = self.get_map_to_base_footprint()
            if pose is not None:
                rospy.logwarn(
                    f"[{source}] RECOVERY [5/7]: 第 {attempt} 次尝试成功获取位姿: "
                    f"x={pose[0]:.3f}, y={pose[1]:.3f}, yaw={math.degrees(pose[2]):.1f}deg")
                break
            rospy.logwarn(
                f"[{source}] RECOVERY [5/7]: 第 {attempt}/{self.recovery_pose_max_retries} 次获取位姿失败，"
                f"{self.recovery_pose_retry_interval}s 后重试...")
            rospy.sleep(self.recovery_pose_retry_interval)

        if pose is None:
            rospy.logerr(f"[{source}] RECOVERY [5/7]: 无法获取位姿，恢复流程中止。")
            with self._lock:
                self.state = self.STATE_IDLE
            return

        px, py, yaw_rad = pose
        yaw_deg = math.degrees(yaw_rad)

        # ── 步骤 6: 关闭 fast_reloc 轨迹 ──────────────────────────────────
        rospy.logwarn(f"[{source}] RECOVERY [6/7]: 关闭 fast_reloc 轨迹...")
        finished_fast = self.finish_latest_trajectory()
        if finished_fast is None:
            rospy.logwarn(f"[{source}] RECOVERY [6/7]: 关闭 fast_reloc 轨迹失败，继续尝试。")
        else:
            rospy.logwarn(f"[{source}] RECOVERY [6/7]: 已关闭 fast_reloc 轨迹 {finished_fast}。")
        rospy.sleep(1.0)

        # ── 步骤 7: 使用捕获的位姿启动默认轨迹 ────────────────────────────
        rospy.logwarn(
            f"[{source}] RECOVERY [7/7]: 使用默认配置 {self.config_file} "
            f"和捕获的位姿启动新轨迹...")
        new_tid = self.start_trajectory_with_pose(
            x=px, y=py, yaw_deg=yaw_deg,
            config_file=self.config_file)
        if new_tid is None:
            rospy.logerr(f"[{source}] RECOVERY [7/7]: 带位姿的新轨迹启动失败！")
        else:
            rospy.logwarn(
                f"[{source}] RECOVERY [7/7]: 新轨迹已启动，trajectory_id={new_tid}, "
                f"x={px:.3f}, y={py:.3f}, yaw={yaw_deg:.1f}deg")

        # 恢复到 IDLE 状态，重置 startup timer
        with self._lock:
            self.state = self.STATE_IDLE
            self.last_reset_time = rospy.Time.now().to_sec()
            self.node_start_time = rospy.Time.now().to_sec()
            self.last_position = None
            self.offset_violation_count = 0
            self.last_yaw = None
            self.rotation_violation_count = 0
            self.score_low_count = 0
        rospy.logwarn(f"[{source}] RECOVERY: === 位姿恢复流程完成 ===")

    # ═══════════════════════════════════════════════════════════════════════
    # 7. Score 检测回调
    # ═══════════════════════════════════════════════════════════════════════

    def score_callback(self, msg):
        """Matching score 回调 — score 连续 N 帧低于阈值时触发恢复。"""
        self.latest_score = msg.data
        with self._lock:
            if self.state != self.STATE_IDLE:
                return

        now = rospy.Time.now().to_sec()
        if now - self.last_reset_time < self.reset_cooldown:
            return

        if msg.data < self.score_threshold:
            self.score_low_count += 1
            rospy.logdebug(
                f"Score low: {msg.data:.3f} < {self.score_threshold}, "
                f"count={self.score_low_count}/{self.score_consecutive_frames}")

            if self.score_low_count >= self.score_consecutive_frames:
                reason = (
                    f"score={msg.data:.3f} < {self.score_threshold} "
                    f"for {self.score_low_count} frames")
                self.score_low_count = 0
                self._trigger_recovery(reason, source="SCORE")
        else:
            if self.score_low_count > 0:
                rospy.loginfo(f"Score recovered to {msg.data:.3f}, resetting count")
            self.score_low_count = 0

    # ═══════════════════════════════════════════════════════════════════════
    # 8. Offset / Rotation 定时回调（map→base_footprint）
    # ═══════════════════════════════════════════════════════════════════════

    def offset_timer_callback(self, event):
        """检测 base_footprint 在 map 坐标系下的瞬时位置/姿态跳变。

        仅在 cartographer 启动 startup_delay 秒后激活。

        触发条件：
          - 位置：欧氏距离 > offset_threshold，连续 N 帧
          - 旋转：|Δyaw| > rotation_threshold_deg，连续 N 帧
        """
        with self._lock:
            if self.state != self.STATE_IDLE:
                return

        now = rospy.Time.now().to_sec()
        if now - self.last_reset_time < self.reset_cooldown:
            return

        # 等待 cartographer 至少运行 startup_delay 秒
        elapsed_since_start = now - self.node_start_time
        if elapsed_since_start < self.offset_startup_delay:
            self.last_position = None
            self.offset_violation_count = 0
            self.last_yaw = None
            self.rotation_violation_count = 0
            return

        # 查找 TF 变换
        try:
            trans = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame,
                rospy.Time(0), rospy.Duration(1.0))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            self.last_position = None
            self.offset_violation_count = 0
            self.last_yaw = None
            self.rotation_violation_count = 0
            return

        x = trans.transform.translation.x
        y = trans.transform.translation.y

        # --- 位置跳变检测 ---
        if self.offset_enabled:
            if self._check_offset(x, y):
                return  # 已触发恢复

        # --- 旋转（yaw）跳变检测 ---
        if self.rotation_enabled:
            q = trans.transform.rotation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            self._check_rotation(yaw)

    # ------------------------------------------------------------------

    def _check_offset(self, x, y):
        """位置跳变检测：首次违规时锁定参考位置，后续帧与同一参考比较，
        直到计数清零或达到阈值。正确处理机器人停留在跳变后位置的情况。

        Returns:
            bool: 是否已触发恢复流程。
        """
        if self.last_position is None:
            self.last_position = (x, y)
            return False

        dx = x - self.last_position[0]
        dy = y - self.last_position[1]
        dist = math.hypot(dx, dy)

        if dist > self.offset_threshold:
            self.offset_violation_count += 1
            rospy.logwarn(
                f"OFFSET JUMP: dist={dist:.3f}m > "
                f"thr={self.offset_threshold:.3f}m "
                f"(ref=({self.last_position[0]:.3f},{self.last_position[1]:.3f})), "
                f"count={self.offset_violation_count}/"
                f"{self.offset_consecutive_frames}")
            # 保持 last_position 锁定 — 不更新
        else:
            if self.offset_violation_count > 0:
                rospy.loginfo(
                    f"Offset normal: dist={dist:.3f}m, resetting count")
            self.offset_violation_count = 0
            # 仅在无违规时更新参考位置
            self.last_position = (x, y)

        if self.offset_violation_count >= self.offset_consecutive_frames:
            reason = (
                f"position jump={dist:.3f}m > {self.offset_threshold:.3f}m "
                f"for {self.offset_violation_count} frames")
            self.offset_violation_count = 0
            self.last_position = None
            self._trigger_recovery(reason, source="OFFSET")
            return True
        return False

    # ------------------------------------------------------------------

    def _check_rotation(self, yaw):
        """yaw 跳变检测：首次违规时锁定参考 yaw，后续帧与同一参考比较，
        直到计数清零或达到阈值。正确处理机器人停留在跳变后朝向的情况。

        Returns:
            bool: 是否已触发恢复流程。
        """
        if self.last_yaw is None:
            self.last_yaw = yaw
            return False

        # 计算最小角度差（处理 ±π 回绕）
        dyaw = abs(yaw - self.last_yaw)
        if dyaw > math.pi:
            dyaw = 2.0 * math.pi - dyaw
        dyaw_deg = math.degrees(dyaw)

        if dyaw_deg > self.rotation_threshold_deg:
            self.rotation_violation_count += 1
            rospy.logwarn(
                f"ROTATION JUMP: |Δyaw|={dyaw_deg:.1f}deg > "
                f"thr={self.rotation_threshold_deg:.0f}deg "
                f"(ref={math.degrees(self.last_yaw):.1f}deg), "
                f"count={self.rotation_violation_count}/"
                f"{self.rotation_consecutive_frames}")
            # 保持 last_yaw 锁定 — 不更新
        else:
            if self.rotation_violation_count > 0:
                rospy.loginfo(
                    f"Rotation normal: |Δyaw|={dyaw_deg:.1f}deg, resetting count")
            self.rotation_violation_count = 0
            # 仅在无违规时更新参考 yaw
            self.last_yaw = yaw

        if self.rotation_violation_count >= self.rotation_consecutive_frames:
            reason = (
                f"yaw jump={dyaw_deg:.1f}deg > {self.rotation_threshold_deg:.0f}deg "
                f"for {self.rotation_violation_count} frames")
            self.rotation_violation_count = 0
            self.last_yaw = None
            self._trigger_recovery(reason, source="ROTATION")
            return True
        return False

    # ═══════════════════════════════════════════════════════════════════════
    # 9. 最大空闲超时回调
    # ═══════════════════════════════════════════════════════════════════════

    def idle_timeout_callback(self, event):
        """若超过 max_idle_timeout 秒未发生恢复，强制执行一次恢复。

        任何已有触发（score / offset / rotation）都会通过 _trigger_recovery
        更新 last_reset_time，从而重置此计时器。
        """
        with self._lock:
            if self.state != self.STATE_IDLE:
                return
        if self.max_idle_timeout <= 0.0:
            return

        now = rospy.Time.now().to_sec()
        idle = now - self.last_reset_time

        if idle >= self.max_idle_timeout:
            reason = f"idle timeout: no recovery for {idle:.0f}s"
            self._trigger_recovery(reason, source="IDLE_TIMEOUT")


# ═════════════════════════════════════════════════════════════════════════
# Main
# ═════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    rospy.init_node("cartographer_trajectory_manager_score")
    manager = CartographerTrajectoryManagerScore()
    rospy.spin()