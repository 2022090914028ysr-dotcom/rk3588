#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
reloc_then_map.py

"先定位，15s 后保存当前 TF，然后再建图" 的状态机控制器。

流程
----
IDLE
 1. 等待 cartographer_node 服务就绪。
 2. 释放话题：把加载 .pbstream 时被恢复的 ACTIVE 轨迹全部 finish（否则
    scan/odom 被占用，/start_trajectory 会报 "Topics are already used"）。
 3. 解析地图参考轨迹 id（优先 FROZEN，其次 FINISHED）与其首个子图位姿
    （= 该轨迹本地坐标系原点，用于把 map 系位姿换算成 relative_pose）。
 4. （可选）等待/使用初始位姿，开始定位轨迹（livox_2d_reloc.lua）。

RELOCATING
 5. 等待 localize_seconds（默认 15s）让 Cartographer 完成重定位。
 6. 捕获当前 map->base 的 TF（即"保存当前 tf"），发布到
    /reloc_then_map/locked_pose，同时写入输出目录 yaml 文件并打印。
 7. finish 定位轨迹。

MAPPING
 8. 以保存的位姿为种子启动建图轨迹（livox_2d_map_continue.lua），
    种子 = inv(冻结地图首子图位姿) * 保存位姿（Cartographer 语义：
    local_to_global = 参考轨迹首节点全局位姿 * relative_pose）。
    这样建图轨迹从当前位置无缝继续扩展地图，map->odom TF 连续。
 9. 用户控制机器人继续探索，之后调用 /reloc_then_map/save_map 保存
    合并地图（旧图 + 新扩展）。

运行信息
--------
  /reloc_then_map/state        std_msgs/String  IDLE/RELOCATING/RELOCATED/MAPPING/ERROR
  /reloc_then_map/localized    std_msgs/Bool    定位已确认（RELOCATED/MAPPING 时为 True）
  /reloc_then_map/status       std_msgs/String  进度消息
  /reloc_then_map/locked_pose  geometry_msgs/PoseStamped  15s 时保存的 map 系位姿

控制服务
--------
  rosservice call /reloc_then_map/save_map true   # 写入合并后的 .pbstream
  rosservice call /reloc_then_map/proceed {}      # 立即跳过剩余定位时间 / 从错误态重启
"""

import math
import os
import yaml

import rospy
import tf2_ros
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse, SetBool, SetBoolResponse
from geometry_msgs.msg import PoseStamped

from cartographer_ros_msgs.msg import SubmapList
from cartographer_ros_msgs.srv import (
    StartTrajectory,
    FinishTrajectory,
    WriteState,
    GetTrajectoryStates,
)

ACTIVE, FINISHED, FROZEN = 0, 1, 2


class RelocThenMap:
    STATE_IDLE       = "IDLE"
    STATE_RELOCATING = "RELOCATING"
    STATE_RELOCATED  = "RELOCATED"
    STATE_MAPPING    = "MAPPING"
    STATE_ERROR      = "ERROR"

    def __init__(self):
        self.config_dir = rospy.get_param("~configuration_directory", "")
        self.reloc_config = rospy.get_param("~reloc_config", "livox_2d_reloc.lua")
        self.map_config = rospy.get_param("~map_config", "livox_2d_map_continue.lua")

        self.start_srv_name = rospy.get_param("~start_srv", "/start_trajectory")
        self.finish_srv_name = rospy.get_param("~finish_srv", "/finish_trajectory")
        self.write_srv_name = rospy.get_param("~write_srv", "/write_state")
        self.states_srv_name = rospy.get_param("~states_srv", "/get_trajectory_states")

        self.tracked_pose_topic = rospy.get_param("~tracked_pose_topic", "/tracked_pose")
        self.init_pose_topic = rospy.get_param("~initial_pose_topic", "")

        # ---- 定位时长与 TF 捕获 ----
        self.localize_seconds = rospy.get_param("~localize_seconds", 15.0)
        self.tracking_frame = rospy.get_param("~tracking_frame", "base_footprint")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.tf_timeout = rospy.get_param("~tf_timeout", 1.0)

        # ---- 初始位姿（可选，map 系）----
        self.init_pose_x = rospy.get_param("~initial_pose_x", None)
        self.init_pose_y = rospy.get_param("~initial_pose_y", None)
        self.init_pose_yaw_deg = rospy.get_param("~initial_pose_yaw_deg", None)
        self.init_pose_timeout = rospy.get_param("~initial_pose_timeout", 15.0)

        # ---- 保存 ----
        self.output_dir = rospy.get_param("~output_dir", "")
        self.output_name = rospy.get_param("~output_name", "map_continued")
        self.auto_save_on_exit = rospy.get_param("~auto_save_on_exit", False)

        # ---- 运行状态 ----
        self.state = self.STATE_IDLE
        self.map_traj_id = None        # 加载的地图轨迹（参考轨迹）
        self.loc_traj_id = None        # 定位轨迹
        self.mapping_traj_id = None    # 建图轨迹
        self._first_submap_pose = {}   # traj_id -> PoseStamped(map 系)
        self._last_tracked_pose = None
        self._map_pose_seed = None     # (x, y, yaw) map 系，15s 捕获
        self._init_pose = None         # (x, y, yaw) map 系，初始位姿
        self._topic_pose_received = False
        self._transitioning = False
        self._traj_start_time = None
        self._state_pub_time = 0.0

        rospy.on_shutdown(self._on_shutdown)

        # ---- 客户端 ----
        self.start_srv = rospy.ServiceProxy(self.start_srv_name, StartTrajectory)
        self.finish_srv = rospy.ServiceProxy(self.finish_srv_name, FinishTrajectory)
        self.write_srv = rospy.ServiceProxy(self.write_srv_name, WriteState)
        self.states_srv = rospy.ServiceProxy(self.states_srv_name, GetTrajectoryStates)

        # ---- TF ----
        self._tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        # ---- 订阅 ----
        self.tracked_pose_sub = rospy.Subscriber(
            self.tracked_pose_topic, PoseStamped, self._on_tracked_pose)
        self.submap_list_sub = rospy.Subscriber(
            "/submap_list", SubmapList, self._on_submap_list)
        if self.init_pose_topic:
            rospy.Subscriber(self.init_pose_topic, PoseStamped, self._on_init_pose)

        # ---- 发布 ----
        self.state_pub = rospy.Publisher(
            "/reloc_then_map/state", String, queue_size=1, latch=True)
        self.loc_pub = rospy.Publisher(
            "/reloc_then_map/localized", Bool, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher(
            "/reloc_then_map/status", String, queue_size=1, latch=True)
        self.locked_pose_pub = rospy.Publisher(
            "/reloc_then_map/locked_pose", PoseStamped, queue_size=1, latch=True)

        # ---- 服务 ----
        self.save_srv = rospy.Service("/reloc_then_map/save_map", SetBool, self._on_save_map)
        self.proceed_srv = rospy.Service("/reloc_then_map/proceed", Trigger, self._on_proceed)

        self._set_state(self.STATE_IDLE, "waiting for cartographer_node")

    # ==================================================================
    # 回调
    # ==================================================================
    def _on_tracked_pose(self, msg):
        self._last_tracked_pose = msg

    def _on_init_pose(self, msg):
        if self._init_pose is None:
            self._init_pose = (msg.pose.position.x, msg.pose.position.y,
                               self._quat_yaw(msg.pose.orientation))
        self._topic_pose_received = True

    def _on_submap_list(self, msg):
        """缓存每个轨迹的第一个子图位姿（= 该轨迹本地原点在 map 系的位置）。"""
        for entry in msg.submap:
            if entry.submap_index == 0:
                p = entry.pose
                self._first_submap_pose[entry.trajectory_id] = (
                    p.position.x, p.position.y, self._quat_yaw(p.orientation))

    # ==================================================================
    # 工具
    # ==================================================================
    @staticmethod
    def _quat_yaw(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _publish_runtime(self):
        now = rospy.Time.now().to_sec()
        if now - self._state_pub_time < 0.2:
            return
        self._state_pub_time = now
        self.loc_pub.publish(Bool(data=self.state in (self.STATE_RELOCATED,
                                                      self.STATE_MAPPING)))

    def _set_state(self, state, status):
        self.state = state
        self.state_pub.publish(String(data=state))
        self.status_pub.publish(String(data=status))
        rospy.loginfo("[reloc_then_map] STATE=%s : %s", state, status)

    def _rc(self, resp, prefix):
        code = getattr(resp.status, "code", None)
        if code is None or code == 0:
            return True
        rospy.logwarn("[reloc_then_map] %s failed: code=%s msg=%s",
                      prefix, code, getattr(resp.status, "message", ""))
        return False

    # ==================================================================
    # 轨迹状态操作
    # ==================================================================
    def _get_states(self):
        try:
            return self.states_srv().trajectory_states
        except rospy.ServiceException as e:
            rospy.logwarn("[reloc_then_map] get_trajectory_states error: %s", e)
            return None

    def _free_unused_trajectories(self):
        """释放加载 .pbstream 时恢复的 ACTIVE 轨迹，腾出 scan/odom 话题。"""
        states = self._get_states()
        if states is None:
            return
        for i in range(len(states.trajectory_id)):
            tid = states.trajectory_id[i]
            if states.trajectory_state[i] == ACTIVE:
                rospy.loginfo("[reloc_then_map] finishing loaded ACTIVE trajectory %d "
                              "to free its topics", tid)
                try:
                    self.finish_srv(tid)
                except rospy.ServiceException as e:
                    rospy.logwarn("[reloc_then_map] finish_trajectory(%d) error: %s", tid, e)

    def _resolve_map_trajectory(self):
        """地图参考轨迹：优先 FROZEN，其次 FINISHED（加载自 pbstream）。"""
        states = self._get_states()
        if states is None:
            return None
        frozen = [states.trajectory_id[i] for i in range(len(states.trajectory_id))
                  if states.trajectory_state[i] == FROZEN]
        finished = [states.trajectory_id[i] for i in range(len(states.trajectory_id))
                    if states.trajectory_state[i] == FINISHED]
        return (frozen or finished or [None])[0]

    def _first_submap_pose_of(self, traj_id):
        return self._first_submap_pose.get(traj_id)

    def _relative_pose_from_map_pose(self, map_pose, ref_traj_id):
        """把 map 系位姿换算成 Cartographer relative_pose：
        local_to_global = ref 轨迹首子图位姿 * relative_pose
        => relative_pose = inv(ref 首子图位姿) * map_pose"""
        ref = self._first_submap_pose_of(ref_traj_id)
        if ref is None:
            return None
        rx, ry, ryaw = ref
        mx, my, myaw = map_pose
        c, s = math.cos(ryaw), math.sin(ryaw)
        dx = mx - rx
        dy = my - ry
        lx = c * dx + s * dy
        ly = -s * dx + c * dy
        lyaw = myaw - ryaw
        return lx, ly, lyaw

    # ==================================================================
    # 定位阶段
    # ==================================================================
    def _start_reloc(self):
        map_traj = self.map_traj_id
        req = StartTrajectory._request_class()
        req.configuration_directory = self.config_dir
        req.configuration_basename = self.reloc_config

        if self._init_pose is not None and map_traj is not None:
            rel = self._relative_pose_from_map_pose(self._init_pose, map_traj)
            if rel is not None:
                x, y, yaw = rel
                req.use_initial_pose = True
                req.relative_to_trajectory_id = map_traj
                req.initial_pose.position.x = x
                req.initial_pose.position.y = y
                req.initial_pose.orientation.z = math.sin(yaw / 2.0)
                req.initial_pose.orientation.w = math.cos(yaw / 2.0)
                rospy.loginfo("[reloc_then_map] seed loc at rel=(%.3f, %.3f, %.1f deg)",
                              x, y, math.degrees(yaw))
        if not req.use_initial_pose:
            rospy.loginfo("[reloc_then_map] no initial pose -> global relocalization")

        try:
            resp = self.start_srv(req)
        except rospy.ServiceException as e:
            self._set_state(self.STATE_ERROR, "start_trajectory(localization) error: %s" % e)
            return
        if not self._rc(resp, "start_trajectory(localization)"):
            self._set_state(self.STATE_ERROR, "start_trajectory(localization) rejected")
            return

        self.loc_traj_id = resp.trajectory_id
        self._traj_start_time = rospy.Time.now().to_sec()
        self._set_state(self.STATE_RELOCATING,
                        "relocalizing %ds ... traj=%d (move robot inside mapped area)" %
                        (self.localize_seconds, self.loc_traj_id))

    def _capture_tf_pose(self):
        """捕获当前 map->base 位姿：TF 优先，/tracked_pose 兜底。"""
        for frame in (self.tracking_frame, "base_link"):
            try:
                t = self._tf_buffer.lookup_transform(
                    self.map_frame, frame, rospy.Time(0),
                    rospy.Duration(self.tf_timeout))
                return (t.transform.translation.x,
                        t.transform.translation.y,
                        self._quat_yaw(t.transform.rotation))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as e:
                rospy.logwarn("[reloc_then_map] TF %s->%s unavailable: %s",
                              self.map_frame, frame, e)
        if self._last_tracked_pose is not None:
            p = self._last_tracked_pose.pose
            return (p.position.x, p.position.y, self._quat_yaw(p.orientation))
        return None

    def _save_locked_pose(self, pose):
        x, y, yaw = pose
        self._map_pose_seed = pose

        stamped = PoseStamped()
        stamped.header.stamp = rospy.Time.now()
        stamped.header.frame_id = self.map_frame
        stamped.pose.position.x = x
        stamped.pose.position.y = y
        stamped.pose.orientation.z = math.sin(yaw / 2.0)
        stamped.pose.orientation.w = math.cos(yaw / 2.0)
        self.locked_pose_pub.publish(stamped)

        rospy.loginfo("[reloc_then_map] SAVED CURRENT TF (map->%s): "
                      "(%.3f, %.3f, %.1f deg)",
                      self.tracking_frame, x, y, math.degrees(yaw))

        if self.output_dir:
            try:
                path = os.path.join(self.output_dir, "locked_pose.yaml")
                with open(path, "w") as f:
                    yaml.safe_dump({"map_frame": self.map_frame,
                                    "x": round(x, 6), "y": round(y, 6),
                                    "yaw_deg": round(math.degrees(yaw), 4),
                                    "tracking_frame": self.tracking_frame},
                                   f, allow_unicode=True)
                rospy.loginfo("[reloc_then_map] locked pose written to %s", path)
            except OSError as e:
                rospy.logwarn("[reloc_then_map] cannot write locked pose file: %s", e)

    def _finish_reloc_start_map(self):
        if self._map_pose_seed is None:
            rospy.logwarn("[reloc_then_map] no locked pose captured -> mapping unpinned")
        try:
            fresp = self.finish_srv(self.loc_traj_id)
        except rospy.ServiceException as e:
            self._set_state(self.STATE_ERROR, "finish_trajectory(localization) error: %s" % e)
            return
        if not self._rc(fresp, "finish_trajectory(localization)"):
            self._set_state(self.STATE_ERROR, "finish_trajectory(localization) rejected")
            return
        rospy.loginfo("[reloc_then_map] localization trajectory %d finished", self.loc_traj_id)
        rospy.sleep(0.5)
        self._start_mapping()

    # ==================================================================
    # 建图阶段
    # ==================================================================
    def _start_mapping(self):
        req = StartTrajectory._request_class()
        req.configuration_directory = self.config_dir
        req.configuration_basename = self.map_config

        if self._map_pose_seed is not None and self.map_traj_id is not None:
            rel = self._relative_pose_from_map_pose(self._map_pose_seed, self.map_traj_id)
            if rel is not None:
                x, y, yaw = rel
                req.use_initial_pose = True
                req.relative_to_trajectory_id = self.map_traj_id
                req.initial_pose.position.x = x
                req.initial_pose.position.y = y
                req.initial_pose.orientation.z = math.sin(yaw / 2.0)
                req.initial_pose.orientation.w = math.cos(yaw / 2.0)
                rospy.loginfo("[reloc_then_map] seed mapping at rel=(%.3f, %.3f, %.1f deg)",
                              x, y, math.degrees(yaw))

        try:
            resp = self.start_srv(req)
        except rospy.ServiceException as e:
            self._set_state(self.STATE_ERROR, "start_trajectory(mapping) error: %s" % e)
            return
        if not self._rc(resp, "start_trajectory(mapping)"):
            self._set_state(self.STATE_ERROR, "start_trajectory(mapping) rejected")
            return

        self.mapping_traj_id = resp.trajectory_id
        self._set_state(self.STATE_MAPPING,
                        "mapping-continue active (traj=%d). explore new area, "
                        "then call /reloc_then_map/save_map" % self.mapping_traj_id)

    # ==================================================================
    # 服务
    # ==================================================================
    def _on_save_map(self, req):
        resp = SetBoolResponse()
        try:
            fname = os.path.join(self.output_dir, self.output_name + ".pbstream") \
                if self.output_dir else self.output_name + ".pbstream"
            wreq = WriteState._request_class()
            wreq.filename = fname
            wreq.include_unfinished_submaps = True
            wres = self.write_srv(wreq)
            if self._rc(wres, "write_state"):
                resp.success = True
                resp.message = "saved merged map -> %s" % fname
            else:
                resp.success = False
                resp.message = "write_state non-OK: %s" % wres.status.message
        except rospy.ServiceException as e:
            resp.success = False
            resp.message = "write_state error: %s" % e
        return resp

    def _on_proceed(self, req):
        resp = TriggerResponse()
        if self.state == self.STATE_RELOCATING:
            if self._transitioning:
                resp.success = False
                resp.message = "transition already in progress"
                return resp
            self._transitioning = True
            pose = self._capture_tf_pose()
            if pose is None:
                self._transitioning = False
                resp.success = False
                resp.message = "cannot capture TF pose; is map->base published?"
                return resp
            self._save_locked_pose(pose)
            self._set_state(self.STATE_RELOCATED, "relocalization manually confirmed")
            self._finish_reloc_start_map()
            resp.success = True
            resp.message = "manual proceed accepted"
        elif self.state in (self.STATE_IDLE, self.STATE_ERROR):
            self._transitioning = False
            self._set_state(self.STATE_IDLE, "restarting relocalization")
            self._start_reloc()
            resp.success = True
            resp.message = "restarted relocalization"
        else:
            resp.success = False
            resp.message = "proceed only valid in IDLE/ERROR/RELOCATING (state=%s)" % self.state
        return resp

    # ==================================================================
    # 主循环
    # ==================================================================
    def _init_pose_ready(self):
        if self.init_pose_x is not None and self.init_pose_y is not None:
            yaw = math.radians(self.init_pose_yaw_deg) if self.init_pose_yaw_deg is not None else 0.0
            self._init_pose = (float(self.init_pose_x), float(self.init_pose_y), yaw)
            return True
        return self._topic_pose_received if self.init_pose_topic else False

    def run(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            self._publish_runtime()

            if self.state == self.STATE_IDLE:
                try:
                    rospy.wait_for_service(self.start_srv_name, timeout=1.0)
                    rospy.wait_for_service(self.states_srv_name, timeout=1.0)
                except rospy.ROSException:
                    rospy.loginfo_throttle(5.0,
                        "[reloc_then_map] waiting for cartographer_node services ...")
                    rate.sleep()
                    continue

                # 释放加载时恢复的 ACTIVE 轨迹（腾出 scan/odom）
                self._free_unused_trajectories()
                rospy.sleep(0.5)

                # 解析地图参考轨迹
                self.map_traj_id = self._resolve_map_trajectory()
                if self.map_traj_id is None:
                    self._set_state(self.STATE_ERROR,
                                    "no FROZEN/FINISHED map trajectory found")
                    rate.sleep()
                    continue
                rospy.loginfo("[reloc_then_map] map reference trajectory = %d",
                              self.map_traj_id)

                # 可选初始位姿
                if self.init_pose_x is not None or self.init_pose_topic:
                    deadline = rospy.Time.now().to_sec() + self.init_pose_timeout
                    while not self._init_pose_ready() and rospy.Time.now().to_sec() < deadline:
                        rate.sleep()
                    if not self._init_pose_ready():
                        rospy.logwarn("[reloc_then_map] no initial pose within %.0fs "
                                      "-> pure global relocalization", self.init_pose_timeout)

                self._start_reloc()

            elif self.state == self.STATE_RELOCATING:
                # 固定时长定位后，保存当前 TF 并切换建图
                if not self._transitioning and \
                        (rospy.Time.now().to_sec() - self._traj_start_time) >= self.localize_seconds:
                    self._transitioning = True
                    pose = self._capture_tf_pose()
                    if pose is None:
                        self._transitioning = False
                        self._set_state(self.STATE_ERROR,
                                        "TF capture failed after %.0fs (map->%s not published)"
                                        % (self.localize_seconds, self.tracking_frame))
                        rate.sleep()
                        continue
                    self._save_locked_pose(pose)
                    self._set_state(self.STATE_RELOCATED,
                                    "localized after %.0fs; TF saved" % self.localize_seconds)
                    self._finish_reloc_start_map()

            elif self.state == self.STATE_ERROR:
                rospy.logwarn_throttle(10.0,
                    "[reloc_then_map] in ERROR state; call /reloc_then_map/proceed to retry")
            rate.sleep()

    def _on_shutdown(self):
        if self.auto_save_on_exit and self.state == self.STATE_MAPPING:
            try:
                fname = os.path.join(self.output_dir, self.output_name + ".pbstream") \
                    if self.output_dir else self.output_name + ".pbstream"
                wreq = WriteState._request_class()
                wreq.filename = fname
                wreq.include_unfinished_submaps = True
                self.write_srv(wreq)
                rospy.loginfo("[reloc_then_map] auto-saved map -> %s", fname)
            except Exception as e:  # noqa
                rospy.logerr("[reloc_then_map] auto-save failed: %s", e)


if __name__ == "__main__":
    rospy.init_node("reloc_then_map")
    try:
        RelocThenMap().run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass