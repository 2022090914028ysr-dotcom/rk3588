#!/usr/bin/env bash
# ============================================================================
# hard_reset.sh — 持续守护版
#
# 内嵌匹配分数 + 雷达稳定性检测，自动触发 cartographer_node 硬重置，
# 循环守护，无需外部调用。
#
# 检测流程：
#   1. /matching_score 连续 N 帧低于阈值 → 进入 WAITING
#   2. /scan 连续 M 帧稳定（机器人已停下） → 触发硬重置
#   3. 杀进程 → 等待稳定期 → 从 pbstream 冷重启 → 继续监控
#
# 参数全部对齐 livox_2d_localization.launch 里的 cartographer_node 配置。
#
# 不会动的节点 (留在原 bringup 终端继续跑):
#   lidar_node         (激光驱动, 独立进程)
#   rf2o_laser_odometry (里程计, 独立进程)
#   map_server          (静态地图 /map, 不受影响)
#   rviz                (纯显示)
#
# 用法:
#   ./hard_reset.sh           # 替代原来的 cartographer_node，自带守护
#   Ctrl-C 退出整个守护循环
# ============================================================================
set -uo pipefail

# --- Cartographer 参数 (与 livox_2d_localization.launch 保持一致) ----------
ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
CONFIG_DIR="${CONFIG_DIR:-$(timeout 5 rospack find cartographer_ros 2>/dev/null || echo '')/configuration_files}"
: "${CONFIG_DIR:?无法找到 cartographer_ros 包，请 source 工作空间}"
CONFIG_BASENAME="livox_2d_localization.lua"
PBSTREAM="${PBSTREAM:-/home/ubuntu/Nav_Release/Robit_SlamNav/NavOA_ws/src/map/meeting_room/meeting_room.pbstream}"
SCAN_TOPIC="/scan"
ODOM_TOPIC="/odom_rf2o"

# --- Guard 检测参数 -------------------------------------------------------
SCORE_TOPIC="${SCORE_TOPIC:-/matching_score}"
SCORE_THRESHOLD="${SCORE_THRESHOLD:-0.75}"
SCORE_CONSECUTIVE="${SCORE_CONSECUTIVE:-10}"
SCAN_STABLE_FRAMES="${SCAN_STABLE_FRAMES:-10}"
SCAN_CHANGE_THRESHOLD="${SCAN_CHANGE_THRESHOLD:-0.5}"
STABILIZING_WAIT="${STABILIZING_WAIT:-5.0}"     # 重置后稳定等待时间
RECOVERY_COOLDOWN="${RECOVERY_COOLDOWN:-10.0}"   # 两次重置最短间隔

export ROS_MASTER_URI

# ============================================================================
# 内嵌 Python 守护监测节点
# 退出码 0  = 触发硬重置
# 退出码 1  = 内部错误（也会触发重启）
# ============================================================================
run_guard() {
  python3 - "$@" << 'PYEOF'
import sys, math
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

score_topic        = sys.argv[1]
score_threshold    = float(sys.argv[2])
score_consecutive  = int(sys.argv[3])
scan_topic         = sys.argv[4]
scan_stable_frames = int(sys.argv[5])
scan_change_thr    = float(sys.argv[6])
cooldown_sec       = float(sys.argv[7])

class HardResetGuard:
    STATE_MONITOR = 0
    STATE_WAITING = 1

    def __init__(self):
        self.state           = self.STATE_MONITOR
        self.score_low_count = 0
        self.scan_stable_cnt = 0
        self.prev_scan       = None
        self.last_trigger    = 0.0

        rospy.init_node("hard_reset_guard", anonymous=True)
        rospy.Subscriber(score_topic, Float32, self.score_cb)
        rospy.Subscriber(scan_topic, LaserScan, self.scan_cb)
        rospy.loginfo("HardResetGuard active: "
                      f"score<{score_threshold}/{score_consecutive}frames, "
                      f"scan_stable<{scan_change_thr}/{scan_stable_frames}frames")

    def set_state(self, s):
        self.state = s
        if s == self.STATE_MONITOR:
            self.score_low_count = 0
            self.scan_stable_cnt = 0
            self.prev_scan = None
        elif s == self.STATE_WAITING:
            self.scan_stable_cnt = 0
            self.prev_scan = None
        rospy.loginfo(f"Guard state -> {'MONITOR' if s==0 else 'WAITING'}")

    def score_cb(self, msg):
        if self.state != self.STATE_MONITOR:
            return
        if msg.data < score_threshold:
            self.score_low_count += 1
            if self.score_low_count >= score_consecutive:
                rospy.logwarn(f"TRIGGER1: score < {score_threshold} "
                              f"for {self.score_low_count} frames -> WAITING")
                self.set_state(self.STATE_WAITING)
        else:
            self.score_low_count = 0

    def scan_cb(self, msg):
        if self.state != self.STATE_WAITING:
            return

        now = rospy.Time.now().to_sec()
        if now - self.last_trigger < cooldown_sec:
            return  # 冷却中

        ranges = [r for i, r in enumerate(msg.ranges)
                  if i % 5 == 0 and not (math.isinf(r) or math.isnan(r))
                  and 0.1 < r < 10.0]
        if not ranges:
            return

        if self.prev_scan is not None:
            n = min(len(ranges), len(self.prev_scan))
            if n > 0:
                diff = sum(abs(ranges[i] - self.prev_scan[i])
                          for i in range(n)) / n
                if diff < scan_change_thr:
                    self.scan_stable_cnt += 1
                    if self.scan_stable_cnt >= scan_stable_frames:
                        rospy.logwarn("TRIGGER2: scan stable -> signal hard reset")
                        self.last_trigger = now
                        sys.exit(0)
                else:
                    self.scan_stable_cnt = 0
        self.prev_scan = ranges


if __name__ == "__main__":
    g = HardResetGuard()
    rospy.spin()
PYEOF
}

# ============================================================================
# 清理 cartographer_node
# ============================================================================
kill_cartographer() {
  echo "[guard] killing /cartographer_node ..."
  timeout 5 rosnode kill /cartographer_node 2>/dev/null || true
  sleep 0.3
  pkill -f "[c]artographer_node" 2>/dev/null || true
  sleep 0.5
}

# ============================================================================
# 主循环
# ============================================================================
echo "=============================================="
echo "[guard] HardResetGuard + cartographer_node"
echo "[guard]   score   : ${SCORE_TOPIC}  threshold=${SCORE_THRESHOLD}  frames=${SCORE_CONSECUTIVE}"
echo "[guard]   scan    : ${SCAN_TOPIC}  stable_thr=${SCAN_CHANGE_THRESHOLD}  frames=${SCAN_STABLE_FRAMES}"
echo "[guard]   cooldown: ${RECOVERY_COOLDOWN}s  stabilize: ${STABILIZING_WAIT}s"
echo "[guard]   pbstream: ${PBSTREAM}"
echo "[guard]   config  : ${CONFIG_DIR}/${CONFIG_BASENAME}"
echo "=============================================="

# 先清理可能残留的旧进程
kill_cartographer

while true; do
  echo ""
  echo "[guard] >>> Starting cartographer_node (cycle ${cycle_count:-0}) ..."

  rosrun cartographer_ros cartographer_node \
    -configuration_directory "${CONFIG_DIR}" \
    -configuration_basename  "${CONFIG_BASENAME}" \
    -load_state_filename     "${PBSTREAM}" \
    scan:=${SCAN_TOPIC} \
    odom:=${ODOM_TOPIC} \
    __name:=cartographer_node &
  CARTO_PID=$!

  # 等 cartographer_node 初始化完成
  echo "[guard] Waiting for cartographer_node to initialize (PID ${CARTO_PID})..."
  sleep 4

  # 运行守护监测（阻塞直到触发或崩溃）
  echo "[guard] Guard monitor starting..."
  set +e
  run_guard "${SCORE_TOPIC}" "${SCORE_THRESHOLD}" "${SCORE_CONSECUTIVE}" \
            "${SCAN_TOPIC}" "${SCAN_STABLE_FRAMES}" "${SCAN_CHANGE_THRESHOLD}" \
            "${RECOVERY_COOLDOWN}"
  guard_rc=$?
  set -e

  if [ $guard_rc -eq 0 ]; then
    echo "[guard] !!! HARD RESET TRIGGERED !!!"
    echo "[guard] Waiting ${STABILIZING_WAIT}s for robot stabilization..."
    sleep "${STABILIZING_WAIT}"
  else
    echo "[guard] Guard exited with code ${guard_rc}, restarting cartographer_node..."
  fi

  # 杀掉当前 cartographer_node
  kill_cartographer

  cycle_count=$((cycle_count + 1))
  echo "[guard] === Cycle ${cycle_count} complete, restarting... ==="
done
