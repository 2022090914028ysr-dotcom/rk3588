#!/usr/bin/env bash
# ============================================================================
# hard_reset.sh — 持续守护版（仅匹配分数触发）
#
# 监听 /matching_score，低于阈值立刻杀进程硬重置，循环守护。
#
# 参数全部对齐 livox_2d_localization.launch 里的 cartographer_node 配置。
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
PBSTREAM="${PBSTREAM:-/home/ubuntu/Robit_SlamNav/NavOA_ws/src/map/meeting_room/meeting_room.pbstream}"
SCAN_TOPIC="/scan"
ODOM_TOPIC="/odom_rf2o"

# --- Guard 检测参数 -------------------------------------------------------
SCORE_TOPIC="${SCORE_TOPIC:-/matching_score}"
SCORE_THRESHOLD="${SCORE_THRESHOLD:-0.60}"
SCORE_CONSECUTIVE="${SCORE_CONSECUTIVE:-30}"
RECOVERY_COOLDOWN="${RECOVERY_COOLDOWN:-1.0}"   # 两次重置最短间隔(秒)

export ROS_MASTER_URI

# ============================================================================
# 内嵌 Python 守护监测 — 仅监听 matching_score，低于阈值立刻退出
# ============================================================================
run_guard() {
  python3 - "$@" << 'PYEOF'
import sys, os
import rospy
from std_msgs.msg import Float32

score_topic       = sys.argv[1]
score_threshold   = float(sys.argv[2])
score_consecutive = int(sys.argv[3])
cooldown_sec      = float(sys.argv[4])

class ScoreGuard:
    def __init__(self):
        self.low_count    = 0
        self.last_trigger = 0.0
        rospy.init_node("hard_reset_guard", anonymous=True)
        rospy.Subscriber(score_topic, Float32, self.cb)
        rospy.loginfo(f"ScoreGuard active: score<{score_threshold}/{score_consecutive}frames -> hard reset")

    def cb(self, msg):
        now = rospy.Time.now().to_sec()
        if now - self.last_trigger < cooldown_sec:
            self.low_count = 0
            return
        if msg.data < score_threshold:
            self.low_count += 1
            if self.low_count >= score_consecutive:
                rospy.logwarn(f"TRIGGER: score={msg.data:.3f} < {score_threshold} x{self.low_count} -> EXIT")
                os._exit(0)
        else:
            self.low_count = 0

if __name__ == "__main__":
    ScoreGuard()
    rospy.spin()
PYEOF
}

# ============================================================================
# 清理 cartographer_node（确保死透）
# ============================================================================
kill_cartographer() {
  echo "[guard] killing /cartographer_node ..."

  # 1) 通过 rosnode 优雅退出
  timeout 5 rosnode kill /cartographer_node 2>/dev/null || true
  # 2) pkill 兜底
  pkill -f "[c]artographer_node" 2>/dev/null || true
  # 3) 强制 kill（确保不残留）
  pkill -9 -f "[c]artographer_node" 2>/dev/null || true
  # 4) 验证是否已死
  if rosnode list 2>/dev/null | grep -q "/cartographer_node"; then
    echo "[guard] WARNING: /cartographer_node still alive!"
  else
    echo "[guard] /cartographer_node confirmed dead"
  fi
}

# ============================================================================
# 主循环 — 只做监测 + 杀/重启，不自己起 cartographer_node
# 前提：cartographer_node 已由 livox_2d_localization.launch 启动
# ============================================================================
echo "=============================================="
echo "[guard] HardResetGuard (sidecar mode)"
echo "[guard]   score     : ${SCORE_TOPIC}  threshold=${SCORE_THRESHOLD}  consecutive=${SCORE_CONSECUTIVE}"
echo "[guard]   cooldown  : ${RECOVERY_COOLDOWN}s"
echo "[guard]   pbstream  : ${PBSTREAM}"
echo "[guard]   config    : ${CONFIG_DIR}/${CONFIG_BASENAME}"
echo "[guard]   scan/odom : ${SCAN_TOPIC} / ${ODOM_TOPIC}"
echo "[guard] (依赖 launch 里的 lidar_node / rf2o / map_server 保持运行)"
echo "=============================================="

while true; do
  echo ""
  echo "[guard] >>> Monitoring /matching_score ..."

  set +e
  run_guard "${SCORE_TOPIC}" "${SCORE_THRESHOLD}" "${SCORE_CONSECUTIVE}" "${RECOVERY_COOLDOWN}"
  guard_rc=$?
  set -e

  echo "[guard] !!! TRIGGER (rc=${guard_rc}) — killing cartographer_node !!!"

  # 杀掉 launch 中正在运行的 cartographer_node
  kill_cartographer

  echo "[guard] Waiting ${RECOVERY_COOLDOWN}s before restart..."
  sleep "${RECOVERY_COOLDOWN}"

  # 重新拉起（参数对齐 livox_2d_localization.launch）
  echo "[guard] >>> Restarting cartographer_node ..."
  rosrun cartographer_ros cartographer_node \
    -configuration_directory "${CONFIG_DIR}" \
    -configuration_basename  "${CONFIG_BASENAME}" \
    -load_state_filename     "${PBSTREAM}" \
    scan:=${SCAN_TOPIC} \
    odom:=${ODOM_TOPIC} \
    __name:=cartographer_node &
  CARTO_PID=$!

  echo "[guard] Waiting for cartographer_node init (PID ${CARTO_PID})..."
  sleep 5

  # 验证节点是否成功启动
  if rosnode list 2>/dev/null | grep -q "/cartographer_node"; then
    echo "[guard] cartographer_node restarted OK, resuming monitor..."
  else
    echo "[guard] WARNING: cartographer_node may have failed to restart"
  fi
done
