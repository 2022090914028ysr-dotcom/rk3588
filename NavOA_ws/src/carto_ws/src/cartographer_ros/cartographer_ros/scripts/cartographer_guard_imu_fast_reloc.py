#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
cartographer_trajectory_manager.py

Cartographer 轨迹管理工具，提供以下功能：
  1. 查阅活跃状态的轨迹 ID
  2. 获取 map→base_footprint 变换（监听 TF 的 map→odom 和 odom→base_footprint 并合成）
  3. 关闭最新的活跃轨迹
  4. 按照指定配置文件创建新轨迹
  5. 按照指定配置文件和初始位姿创建新轨迹
  6. IMU 监听，判断机器人是否摔倒
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
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class CartographerTrajectoryManager:
    """Cartographer 轨迹管理 + IMU 摔倒检测。"""

    # ── 摔倒检测状态机 ──────────────────────────────────────────────────
    STATE_IDLE       = 0
    STATE_FALLEN     = 1
    STATE_RECOVERING = 2

    def __init__(self):
        # ── 服务名称 ────────────────────────────────────────────────────
        self.start_trajectory_service   = rospy.get_param("~start_service",   "/start_trajectory")
        self.finish_trajectory_service  = rospy.get_param("~finish_service",  "/finish_trajectory")
        self.get_states_service         = rospy.get_param("~get_states_service", "/get_trajectory_states")

        # ── 配置 ────────────────────────────────────────────────────────
        self.config_dir   = rospy.get_param("~config_dir",   os.path.join(rospkg.RosPack().get_path('cartographer_ros'), 'configuration_files'))
        self.config_file             = rospy.get_param("~config_file",             "livox_2d_localization_old.lua")
        self.config_file_fast_reloc  = rospy.get_param("~config_file_fast_reloc",  "livox_2d_localization_fast_reloc.lua")

        # ── TF 帧 ───────────────────────────────────────────────────────
        self.map_frame           = rospy.get_param("~map_frame",           "map")
        self.odom_frame          = rospy.get_param("~odom_frame",          "odom")
        self.robot_frame         = rospy.get_param("~robot_frame",         "base_footprint")
        self.tf_lookup_timeout   = rospy.get_param("~tf_lookup_timeout",    0.1)

        # ── IMU 摔倒检测 ─────────────────────────────────────────────────
        self.imu_topic                = rospy.get_param("~imu_topic",                "/imu_mux")
        self.fall_roll_threshold_deg  = rospy.get_param("~fall_roll_threshold_deg",  20.0)
        self.fall_pitch_threshold_deg = rospy.get_param("~fall_pitch_threshold_deg", 30.0)
        self.upright_roll_threshold_deg  = rospy.get_param("~upright_roll_threshold_deg",   5.0)
        self.upright_pitch_threshold_deg = rospy.get_param("~upright_pitch_threshold_deg",  5.0)
        self.upright_confirm_frames      = rospy.get_param("~upright_confirm_frames",       5)

        # ── 恢复流程参数 ──────────────────────────────────────────────────
        self.recovery_upright_confirm_frames = rospy.get_param(
            "~recovery_upright_confirm_frames", 10)
        self.recovery_pose_retry_interval     = rospy.get_param(
            "~recovery_pose_retry_interval",   1.0)
        self.recovery_pose_max_retries        = rospy.get_param(
            "~recovery_pose_max_retries",      30)
        self.recovery_settle_duration         = rospy.get_param(
            "~recovery_settle_duration",       2.0)

        # ── 内部状态 ────────────────────────────────────────────────────
        self._lock = threading.Lock()
        self.state = self.STATE_IDLE
        self.upright_confirm_count = 0
        self.fall_detected = False
        self.fall_reason = None

        # ── TF ──────────────────────────────────────────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # ── Service proxies ─────────────────────────────────────────────
        self._start_proxy  = None
        self._finish_proxy = None
        self._states_proxy = None

        # ── IMU 订阅 ─────────────────────────────────────────────────────
        rospy.Subscriber(self.imu_topic, Imu, self.imu_callback, queue_size=20)

        rospy.loginfo(
            "CartographerTrajectoryManager: ready. "
            f"start={self.start_trajectory_service}, "
            f"finish={self.finish_trajectory_service}, "
            f"states={self.get_states_service}")

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
            # GetTrajectoryStates 响应包含 trajectory_states 列表，
            # 每个 trajectory_state 有 trajectory_id 字段
            ids = []
            for ts in resp.trajectory_states:
                ids.append(ts.trajectory_id)
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
    # 7. 摔倒恢复流程
    # ═══════════════════════════════════════════════════════════════════════

    def _wait_for_upright(self):
        """阻塞等待机器人恢复直立状态（连续 recovery_upright_confirm_frames 帧）。"""
        confirm_count = 0
        rate = rospy.Rate(10)  # 10 Hz 检测频率
        while not rospy.is_shutdown():
            with self._lock:
                if self.state != self.STATE_FALLEN:
                    # 状态已被其他逻辑修改
                    return False
                if confirm_count >= self.recovery_upright_confirm_frames:
                    return True
            rate.sleep()
            # 读取 IMU 缓存的姿态判断直立
            # 这里复用 imu_callback 中的直立检测逻辑，通过检查 fall_detected 标志
            with self._lock:
                if not self.fall_detected:
                    confirm_count += 1
                else:
                    confirm_count = 0
        return False

    def _recovery_sequence(self):
        """摔倒后的恢复流程（在独立线程中执行）：

        1. 关闭最新的活跃轨迹
        2. 启动 fast_reloc 配置的新轨迹（无初始位姿）
        3. 等待机器人站起来稳定
        4. 尝试获取 map→base_footprint 位姿作为初始位姿
        5. 关闭 fast_reloc 轨迹
        6. 使用默认配置和初始位姿启动新轨迹
        """
        rospy.logwarn("RECOVERY: === 摔倒恢复流程开始 ===")

        # ── 步骤 1: 关闭最新的活跃轨迹 ──────────────────────────────────
        rospy.logwarn("RECOVERY [1/6]: 关闭最新的活跃轨迹...")
        finished_id = self.finish_latest_trajectory()
        if finished_id is None:
            rospy.logwarn("RECOVERY [1/6]: 没有活跃轨迹可关闭，继续流程。")
        else:
            rospy.logwarn(f"RECOVERY [1/6]: 已关闭轨迹 {finished_id}。")
        rospy.sleep(1.0)

        # ── 步骤 2: 启动 fast_reloc 轨迹（无初始位姿）────────────────────
        rospy.logwarn("RECOVERY [2/6]: 启动 fast_reloc 轨迹（无初始位姿）...")
        fast_tid = self.start_trajectory(
            config_file=self.config_file_fast_reloc)
        if fast_tid is None:
            rospy.logerr("RECOVERY [2/6]: fast_reloc 轨迹启动失败，恢复流程中止。")
            with self._lock:
                self.state = self.STATE_IDLE
                self.fall_detected = False
                self.fall_reason = None
            return
        rospy.logwarn(f"RECOVERY [2/6]: fast_reloc 轨迹已启动，trajectory_id={fast_tid}。")

        # ── 步骤 3: 等待机器人站起来稳定 ──────────────────────────────────
        rospy.logwarn("RECOVERY [3/6]: 等待机器人恢复直立...")
        upright_ok = self._wait_for_upright()
        if not upright_ok:
            rospy.logerr("RECOVERY [3/6]: 等待直立失败或被中断，恢复流程中止。")
            with self._lock:
                self.state = self.STATE_IDLE
                self.fall_detected = False
                self.fall_reason = None
            return

        # 进入恢复状态，IMU 回调不再处理
        with self._lock:
            self.state = self.STATE_RECOVERING

        # 额外等待一段时间让定位稳定
        rospy.logwarn(
            f"RECOVERY [3/6]: 机器人已直立，等待 {self.recovery_settle_duration}s 让定位稳定...")
        rospy.sleep(self.recovery_settle_duration)

        # ── 步骤 4: 尝试获取 map→base_footprint 位姿 ─────────────────────
        rospy.logwarn("RECOVERY [4/6]: 尝试获取 map→base_footprint 位姿...")
        pose = None
        for attempt in range(1, self.recovery_pose_max_retries + 1):
            if rospy.is_shutdown():
                return
            pose = self.get_map_to_base_footprint()
            if pose is not None:
                rospy.logwarn(
                    f"RECOVERY [4/6]: 第 {attempt} 次尝试成功获取位姿: "
                    f"x={pose[0]:.3f}, y={pose[1]:.3f}, yaw={math.degrees(pose[2]):.1f}deg")
                break
            rospy.logwarn(
                f"RECOVERY [4/6]: 第 {attempt}/{self.recovery_pose_max_retries} 次获取位姿失败，"
                f"{self.recovery_pose_retry_interval}s 后重试...")
            rospy.sleep(self.recovery_pose_retry_interval)

        if pose is None:
            rospy.logerr("RECOVERY [4/6]: 多次尝试后仍无法获取位姿，恢复流程中止。")
            with self._lock:
                self.state = self.STATE_IDLE
                self.fall_detected = False
                self.fall_reason = None
            return

        px, py, yaw_rad = pose
        yaw_deg = math.degrees(yaw_rad)

        # ── 步骤 5: 关闭 fast_reloc 轨迹 ──────────────────────────────────
        rospy.logwarn("RECOVERY [5/6]: 关闭 fast_reloc 轨迹...")
        finished_fast = self.finish_latest_trajectory()
        if finished_fast is None:
            rospy.logwarn("RECOVERY [5/6]: 关闭 fast_reloc 轨迹失败，继续尝试。")
        else:
            rospy.logwarn(f"RECOVERY [5/6]: 已关闭 fast_reloc 轨迹 {finished_fast}。")
        rospy.sleep(1.0)

        # ── 步骤 6: 使用默认配置和初始位姿启动新轨迹 ───────────────────────
        rospy.logwarn(
            f"RECOVERY [6/6]: 使用默认配置 {self.config_file} 和初始位姿启动新轨迹...")
        new_tid = self.start_trajectory_with_pose(
            x=px, y=py, yaw_deg=yaw_deg,
            config_file=self.config_file)
        if new_tid is None:
            rospy.logerr("RECOVERY [6/6]: 带位姿的新轨迹启动失败！")
        else:
            rospy.logwarn(
                f"RECOVERY [6/6]: 新轨迹已启动，trajectory_id={new_tid}, "
                f"x={px:.3f}, y={py:.3f}, yaw={yaw_deg:.1f}deg")

        # 恢复到 IDLE 状态
        with self._lock:
            self.state = self.STATE_IDLE
            self.fall_detected = False
            self.fall_reason = None
        rospy.logwarn("RECOVERY: === 摔倒恢复流程完成 ===")

    # ═══════════════════════════════════════════════════════════════════════
    # 6. IMU 监听，判断机器人是否摔倒
    # ═══════════════════════════════════════════════════════════════════════

    def imu_callback(self, msg):
        """IMU 回调：基于 roll/pitch 判断摔倒与恢复，并驱动恢复流程。"""
        with self._lock:
            if self.state == self.STATE_RECOVERING:
                return

            q = msg.orientation
            roll, pitch, _ = euler_from_quaternion([q.x, q.y, q.z, q.w])
            roll_deg  = abs(math.degrees(roll))
            pitch_deg = abs(math.degrees(pitch))

            if self.state == self.STATE_IDLE:
                # 检测摔倒
                if (roll_deg  > self.fall_roll_threshold_deg or
                        pitch_deg > self.fall_pitch_threshold_deg):
                    self.state = self.STATE_FALLEN
                    self.fall_detected = True
                    self.fall_reason = (
                        f"fall: |roll|={roll_deg:.1f}deg, |pitch|={pitch_deg:.1f}deg")
                    self.upright_confirm_count = 0
                    rospy.logwarn(
                        f"IMU FALL DETECTED: |roll|={roll_deg:.1f}deg "
                        f"(thr={self.fall_roll_threshold_deg:.0f}), "
                        f"|pitch|={pitch_deg:.1f}deg "
                        f"(thr={self.fall_pitch_threshold_deg:.0f}). "
                        "Starting recovery sequence...")
                    # 启动恢复线程
                    t = threading.Thread(target=self._recovery_sequence, daemon=True)
                    t.start()

            elif self.state == self.STATE_FALLEN:
                # 检测恢复直立
                if (roll_deg  <= self.upright_roll_threshold_deg and
                        pitch_deg <= self.upright_pitch_threshold_deg):
                    self.upright_confirm_count += 1
                else:
                    self.upright_confirm_count = 0

                if self.upright_confirm_count >= self.upright_confirm_frames:
                    self.state = self.STATE_IDLE
                    self.fall_detected = False
                    self.fall_reason = None
                    self.upright_confirm_count = 0
                    rospy.loginfo(
                        f"IMU UPRIGHT CONFIRMED: |roll|={roll_deg:.1f}deg, "
                        f"|pitch|={pitch_deg:.1f}deg. Robot is upright.")
                else:
                    rospy.loginfo_throttle(
                        2.0,
                        f"IMU WAITING UPRIGHT: "
                        f"|roll|={roll_deg:.1f}/{self.upright_roll_threshold_deg:.1f}deg, "
                        f"|pitch|={pitch_deg:.1f}/{self.upright_pitch_threshold_deg:.1f}deg, "
                        f"frames={self.upright_confirm_count}/{self.upright_confirm_frames}")

    def is_fallen(self):
        """返回当前是否处于摔倒状态。"""
        with self._lock:
            return self.fall_detected

    def get_fall_reason(self):
        """返回摔倒原因字符串，未摔倒时返回 None。"""
        with self._lock:
            return self.fall_reason


# ═════════════════════════════════════════════════════════════════════════
# Main
# ═════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    rospy.init_node("cartographer_trajectory_manager")
    manager = CartographerTrajectoryManager()
    rospy.spin()