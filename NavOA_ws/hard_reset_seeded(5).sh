#!/usr/bin/env bash
# ============================================================================
# hard_reset_seeded.sh — 监听 /matching_score，定位失败时硬重启 cartographer_node
# 重启后以 use_initial_pose=False 起新轨迹，由 cartographer 后端全局定位自行收敛
# （与首次启动行为完全一致，不注入任何 seed）。
#
# 注意：cartographer_guard.py 是软重启竞品，部署时与本脚本二选一，不应并存。
#
# 关键设计：
# - /tf 桥接只在 /tf 上 10Hz 动态广播 map→odom，绝不写 /tf_static（避免 latched 污染）
# - /scan_matched_points2 桥接以 2Hz 重发缓存帧，RViz 订阅不断
# - bridge 事件驱动停止：检测到新 carto 发出 scan_matched_points2 即自杀
# - 重启后不注入 seed：start_trajectory 用 use_initial_pose=False，靠后端全局定位
# - 全部 kill 走 kill_robust / kill_pattern_robust：TERM → 验证 → KILL → 验证 → FATAL
# ============================================================================
set -uo pipefail

# ----- 工作区 source -----
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_SETUP="${SCRIPT_DIR}/devel_isolated/setup.bash"
if [[ -f "${WS_SETUP}" ]]; then
  source "${WS_SETUP}"
elif [[ -f "/home/ubuntu/Robit_SlamNav/carto_ws/devel_isolated/setup.bash" ]]; then
  source "/home/ubuntu/Robit_SlamNav/carto_ws/devel_isolated/setup.bash"
fi

ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
CONFIG_DIR="${CONFIG_DIR:-$(timeout 5 rospack find cartographer_ros 2>/dev/null || echo '')/configuration_files}"
: "${CONFIG_DIR:?Cannot find cartographer_ros package, source your ROS workspace}"

# ----- 日志 -----
LOG_DIR="${LOG_DIR:-/home/ubuntu/Robit_SlamNav/NavOA_ws/logs}"
LOG_FILE="${LOG_FILE:-${LOG_DIR}/hard_reset_seeded.log}"
mkdir -p "${LOG_DIR}"
: > "${LOG_FILE}"
exec 3>>"${LOG_FILE}"

# ----- 配置 -----
BRIDGE_READY_FILE="${BRIDGE_READY_FILE:-${LOG_DIR}/hard_reset_bridge.ready}"
BRIDGE_READY_TIMEOUT="${BRIDGE_READY_TIMEOUT:-5}"

CONFIG_BASENAME="${CONFIG_BASENAME:-livox_2d_localization.lua}"
PBSTREAM="${PBSTREAM:-/home/ubuntu/Robit_SlamNav/NavOA_ws/src/map/meeting_room/meeting_room.pbstream}"
SCAN_TOPIC="${SCAN_TOPIC:-/scan}"
ODOM_TOPIC="${ODOM_TOPIC:-/odom_rf2o}"
MAP_FRAME="${MAP_FRAME:-map}"
ODOM_FRAME="${ODOM_FRAME:-odom}"
SCAN_MATCHED_TOPIC="${SCAN_MATCHED_TOPIC:-/scan_matched_points2}"

SCORE_TOPIC="${SCORE_TOPIC:-/matching_score}"
SCORE_THRESHOLD="${SCORE_THRESHOLD:-0.65}"
SCORE_CONSECUTIVE="${SCORE_CONSECUTIVE:-30}"
RECOVERY_COOLDOWN="${RECOVERY_COOLDOWN:-0.3}"
STABILIZING_WAIT="${STABILIZING_WAIT:-2}"
BRIDGE_WAIT_TIMEOUT="${BRIDGE_WAIT_TIMEOUT:-5}"

BRIDGE_PID=""

export ROS_MASTER_URI

# ============================================================================
# 通用工具：日志 + 计时
# ============================================================================
log() {
  local line
  line="[$(date '+%F %T')] $*"
  printf '%s\n' "$line"
  printf '%s\n' "$line" >&3
}

ts_ms() { date '+%s%3N'; }

time_log() {
  local name="$1"
  local start_ms="$2"
  local end_ms
  end_ms="$(ts_ms)"
  local elapsed=$(( end_ms - start_ms ))
  log "[timing] ${name}: ${elapsed}ms"
}

# ============================================================================
# Kill 工具：TERM → 3×0.2s 验证 → KILL → 0.1s 验证 → 仍存活记 FATAL
# ============================================================================
kill_robust() {
  local pid="$1"
  local name="${2:-pid=$pid}"
  [[ -z "${pid}" ]] && return 0
  if ! kill -0 "${pid}" 2>/dev/null; then
    return 0
  fi
  kill -TERM "${pid}" 2>/dev/null || true
  local i
  for i in 1 2 3; do
    sleep 0.2
    if ! kill -0 "${pid}" 2>/dev/null; then
      return 0
    fi
  done
  log "[kill] ${name} (pid=${pid}) ignored TERM, escalating to SIGKILL"
  kill -9 "${pid}" 2>/dev/null || true
  sleep 0.1
  if kill -0 "${pid}" 2>/dev/null; then
    log "[kill] FATAL: ${name} (pid=${pid}) still alive after SIGKILL"
    return 1
  fi
  return 0
}

kill_pattern_robust() {
  local pattern="$1"
  local name="${2:-$pattern}"
  local pids
  pids="$(pgrep -f -- "${pattern}" 2>/dev/null || true)"
  if [[ -z "${pids}" ]]; then
    return 0
  fi
  log "[kill] ${name}: TERM to PIDs: $(echo ${pids} | tr '\n' ' ')"
  # shellcheck disable=SC2086
  kill -TERM ${pids} 2>/dev/null || true
  local i
  for i in 1 2 3; do
    sleep 0.2
    if ! pgrep -f -- "${pattern}" >/dev/null 2>&1; then
      return 0
    fi
  done
  log "[kill] ${name} ignored TERM, escalating to SIGKILL"
  pkill -9 -f -- "${pattern}" 2>/dev/null || true
  sleep 0.1
  if pgrep -f -- "${pattern}" >/dev/null 2>&1; then
    log "[kill] FATAL: ${name} still alive after SIGKILL"
    return 1
  fi
  return 0
}

# ============================================================================
# Cleanup：三时机
# ============================================================================
cleanup_orphans_startup() {
  log "[cleanup] startup: removing stale cache + orphan processes ..."
  rm -f "${BRIDGE_READY_FILE}"
  local node
  for node in $(rosnode list 2>/dev/null | grep -E '^/hard_reset_' || true); do
    log "[cleanup]   rosnode kill ${node}"
    timeout 1 rosnode kill "${node}" 2>/dev/null || true
  done
  sleep 0.2
  kill_pattern_robust '__hr_tag=' 'hard_reset_python'
  kill_pattern_robust "static_transform_publisher.*${MAP_FRAME}.*${ODOM_FRAME}" "stale_static_${MAP_FRAME}_to_${ODOM_FRAME}"
  log "[cleanup] startup done"
}

cleanup_pre_trigger() {
  log "[cleanup] pre-trigger: removing stale bridge processes ..."
  local node
  for node in $(rosnode list 2>/dev/null | grep -E '^/hard_reset_bridge' || true); do
    timeout 1 rosnode kill "${node}" 2>/dev/null || true
  done
  kill_pattern_robust '__hr_tag=bridge' 'leftover_bridge'
}

on_exit() {
  # 幂等：重复触发只跑一次（exit 链路里 INT/TERM trap 再走 EXIT trap 会重入）
  if [[ "${ON_EXIT_DONE:-}" == "1" ]]; then return 0; fi
  ON_EXIT_DONE=1
  log "[trap] cleaning up ..."
  if [[ -n "${BRIDGE_PID:-}" ]]; then
    kill_robust "${BRIDGE_PID}" "bridge"
  fi
  kill_pattern_robust '__hr_tag=' 'hard_reset_python'
  kill_pattern_robust "static_transform_publisher.*${MAP_FRAME}.*${ODOM_FRAME}" "stale_static_${MAP_FRAME}_to_${ODOM_FRAME}"
}

# EXIT trap 只负责清理；
# INT/TERM trap 显式 exit，否则 trap 跑完会回到主循环里把 run_guard 退出当作 TRIGGER 错触发一次硬重启。
trap on_exit EXIT
trap 'log "[trap] SIGINT received, exiting"; exit 130' INT
trap 'log "[trap] SIGTERM received, exiting"; exit 143' TERM

# ============================================================================
# 同步抓 map->odom（由 bash 在 kill 老 carto 之前调用，保证 lookup 时 carto 还活着）
# stdout 打印 7 个浮点："tx ty tz rx ry rz rw"；失败 stderr + exit 1
# ============================================================================
capture_map_odom_sync() {
  python3 - "${MAP_FRAME}" "${ODOM_FRAME}" 2>>"${LOG_FILE}" <<'PYEOF'
import sys
import rospy
import tf2_ros

m = sys.argv[1]
o = sys.argv[2]

rospy.init_node('hard_reset_tf_capture', anonymous=True, disable_signals=True)
buf = tf2_ros.Buffer()
tf2_ros.TransformListener(buf)

deadline = rospy.Time.now() + rospy.Duration(2.0)
while not rospy.is_shutdown() and rospy.Time.now() < deadline:
  try:
    ts = buf.lookup_transform(m, o, rospy.Time(0), rospy.Duration(0.3))
    t = ts.transform.translation
    r = ts.transform.rotation
    print(f"{t.x} {t.y} {t.z} {r.x} {r.y} {r.z} {r.w}")
    sys.exit(0)
  except Exception:
    rospy.sleep(0.05)
sys.exit(1)
PYEOF
}

# ============================================================================
# 合并 bridge：TF (10Hz on /tf) + PC republish (2Hz on /scan_matched_points2)
# 事件驱动停止：SIGUSR1 后进入 detecting 模式，看到非自身 caller_id 的 PC 消息即自杀
#
# 设计变化（与日志反推）：
#   - tf 抓取从 bridge 内部挪到 bash 主流程同步执行（capture_map_odom_sync），
#     避免 bridge 还在 import 时老 carto 已经被 kill 掉、lookup 拿不到东西的竞态。
#   - bridge 通过 argv 接收 tf 参数，启动后立即开广播，不再做 lookup。
#   - bridge 起完 broadcaster 后写一个 ready 标志文件，主循环 poll 后再 kill carto，
#     保证整个 kill 流程中 /tf 一直有人发。
# ============================================================================
start_combined_bridge() {
  cleanup_pre_trigger
  rm -f "${BRIDGE_READY_FILE}"

  # 1) 同步抓 map->odom（阻塞 ~1-2s，期间老 carto 仍在发 tf）
  local tf_line tx ty tz rx ry rz rw
  tf_line="$(capture_map_odom_sync || true)"
  if [[ -z "${tf_line}" ]]; then
    log "[guard] FATAL: capture map->odom failed; bridge will broadcast identity (RViz 会跳)"
    tf_line="0 0 0 0 0 0 1"
  fi
  # shellcheck disable=SC2034
  read -r tx ty tz rx ry rz rw <<< "${tf_line}"
  log "[guard] captured ${MAP_FRAME}->${ODOM_FRAME}: t=(${tx},${ty},${tz}) q=(${rx},${ry},${rz},${rw})"

  # 2) 异步起 bridge，tf 从 argv 传入，无内部 lookup
  python3 - "__hr_tag=bridge" "${MAP_FRAME}" "${ODOM_FRAME}" "${SCAN_MATCHED_TOPIC}" "${LOG_FILE}" "${BRIDGE_READY_FILE}" "${tx}" "${ty}" "${tz}" "${rx}" "${ry}" "${rz}" "${rw}" >>"${LOG_FILE}" 2>&1 <<'PYEOF' &
# ============================================================================
# 注意顺序：先装信号 handler 再 import 重模块。
# rospy 在 arm64 上 import 要 1-2s，主脚本 ~2s 后就发 SIGUSR1，
# 默认 SIGUSR1 动作是 terminate；早装 handler 才不会被自己的信号打死。
# ============================================================================
import os
import signal
import sys
import threading
import time

# --- argv (在重 import 前只读必要的几个) ---
log_file = sys.argv[5]
ready_file = sys.argv[6]

def log(msg):
  with open(log_file, 'a', encoding='utf-8') as fp:
    fp.write(f"[{time.strftime('%F %T')}] [bridge] {msg}\n")

# --- 共享状态 + 信号 handler，必须在 ROS import 之前装好 ---
shutdown_event = threading.Event()
mode = {'v': 'caching'}  # caching → detecting

def request_shutdown(reason):
  if not shutdown_event.is_set():
    log(f"shutdown requested: {reason}")
    shutdown_event.set()

def on_sigterm(signum, frame):
  request_shutdown(f"signal {signum}")

def on_sigusr1(signum, frame):
  if mode['v'] != 'detecting':
    mode['v'] = 'detecting'
    log("mode -> detecting (got SIGUSR1, watch for new carto)")

signal.signal(signal.SIGTERM, on_sigterm)
signal.signal(signal.SIGUSR1, on_sigusr1)
log("signal handlers installed, importing ROS ...")

# --- 慢操作 ---
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2

map_frame = sys.argv[2]
odom_frame = sys.argv[3]
scan_matched_topic = sys.argv[4]
tx = float(sys.argv[7])
ty = float(sys.argv[8])
tz = float(sys.argv[9])
rx = float(sys.argv[10])
ry = float(sys.argv[11])
rz = float(sys.argv[12])
rw = float(sys.argv[13])

rospy.init_node('hard_reset_bridge', anonymous=True, disable_signals=True)
my_caller_id = rospy.get_caller_id()
log(f"ros init complete, caller_id={my_caller_id}")
log(f"using {map_frame}->{odom_frame}: "
    f"t=({tx:.3f},{ty:.3f},{tz:.3f}) q=({rx:.6f},{ry:.6f},{rz:.6f},{rw:.6f})")

last_pc = {'msg': None}

def pc_cb(msg):
  hdr = getattr(msg, '_connection_header', None) or {}
  caller = hdr.get('callerid', '')
  if caller == my_caller_id:
    return  # 自己 republish 的回环
  last_pc['msg'] = msg
  if mode['v'] == 'detecting':
    request_shutdown(f"saw {scan_matched_topic} from external caller={caller}")

pc_sub = rospy.Subscriber(scan_matched_topic, PointCloud2, pc_cb, queue_size=1)
pc_pub = rospy.Publisher(scan_matched_topic, PointCloud2, queue_size=1)

def tf_loop():
  br = tf2_ros.TransformBroadcaster()
  ts = TransformStamped()
  ts.header.frame_id = map_frame
  ts.child_frame_id = odom_frame
  ts.transform.translation.x = tx
  ts.transform.translation.y = ty
  ts.transform.translation.z = tz
  ts.transform.rotation.x = rx
  ts.transform.rotation.y = ry
  ts.transform.rotation.z = rz
  ts.transform.rotation.w = rw
  rate = rospy.Rate(10)
  log(f"TF broadcaster started: {map_frame}->{odom_frame} 10Hz")
  while not rospy.is_shutdown() and not shutdown_event.is_set():
    ts.header.stamp = rospy.Time.now()
    try:
      br.sendTransform(ts)
    except Exception:
      pass
    try:
      rate.sleep()
    except Exception:
      break
  log("TF broadcaster stopped")

def pc_loop():
  rate = rospy.Rate(2)
  log(f"PC bridge started: {scan_matched_topic} 2Hz republish")
  while not rospy.is_shutdown() and not shutdown_event.is_set():
    m = last_pc['msg']
    if m is not None:
      try:
        m.header.stamp = rospy.Time.now()
        pc_pub.publish(m)
      except Exception:
        pass
    try:
      rate.sleep()
    except Exception:
      break
  log("PC bridge stopped")

t_tf = threading.Thread(target=tf_loop, daemon=True)
t_pc = threading.Thread(target=pc_loop, daemon=True)
t_tf.start()
t_pc.start()

# 通知主脚本: broadcaster 已起, 可以放心 kill old carto 了
try:
  tmp = ready_file + '.tmp'
  with open(tmp, 'w', encoding='utf-8') as fp:
    fp.write(f"{os.getpid()}\n")
  os.replace(tmp, ready_file)
  log(f"ready signal written: {ready_file}")
except Exception as e:
  log(f"WARNING: failed to write ready file: {e}")

# 主线程：等 shutdown 信号
while not rospy.is_shutdown() and not shutdown_event.is_set():
  time.sleep(0.1)

# 收尾
try:
  pc_sub.unregister()
except Exception:
  pass
try:
  pc_pub.unregister()
except Exception:
  pass
try:
  os.unlink(ready_file)
except Exception:
  pass
rospy.signal_shutdown('bridge exit')
t_tf.join(timeout=0.5)
t_pc.join(timeout=0.5)
log("clean exit")
sys.exit(0)
PYEOF
  BRIDGE_PID=$!
  log "[guard] combined bridge PID=${BRIDGE_PID} spawned, waiting for ready signal ..."

  # 3) 等 bridge ready (最多 BRIDGE_READY_TIMEOUT 秒)
  local i
  local max_loops=$(( BRIDGE_READY_TIMEOUT * 5 ))
  for i in $(seq 1 ${max_loops}); do
    if [[ -f "${BRIDGE_READY_FILE}" ]]; then
      log "[guard] bridge ready after $(( i * 200 ))ms"
      return 0
    fi
    if ! kill -0 "${BRIDGE_PID}" 2>/dev/null; then
      log "[guard] WARNING: bridge died before ready signal (check log)"
      BRIDGE_PID=""
      return 1
    fi
    sleep 0.2
  done
  log "[guard] WARNING: bridge ready timeout ${BRIDGE_READY_TIMEOUT}s, continuing anyway"
  return 0
}

# 主循环：bridge 自检测后会自杀；这里 timeout 兜底
wait_bridge_exit_or_timeout() {
  local secs="${1:-5}"
  local i
  local max_loops=$(( secs * 5 ))
  for i in $(seq 1 "${max_loops}"); do
    if [[ -z "${BRIDGE_PID}" ]] || ! kill -0 "${BRIDGE_PID}" 2>/dev/null; then
      log "[bridge] exited cleanly"
      BRIDGE_PID=""
      return 0
    fi
    sleep 0.2
  done
  log "[bridge] did not self-exit within ${secs}s, escalating"
  if [[ -n "${BRIDGE_PID}" ]]; then
    kill_robust "${BRIDGE_PID}" "bridge"
    BRIDGE_PID=""
  fi
  kill_pattern_robust '__hr_tag=bridge' 'bridge_fallback'
}

# ============================================================================
# Cartographer 控制
# ============================================================================
kill_cartographer() {
  log "[guard] killing /cartographer_node ..."
  timeout 1 rosnode kill /cartographer_node 2>/dev/null || true
  kill_pattern_robust "[c]artographer_node" "cartographer_node"
}

cartographer_running() {
  pgrep -f "[c]artographer_node" >/dev/null 2>&1
}

restart_cartographer() {
  log "[guard] >>> restarting cartographer_node ..."
  rosrun cartographer_ros cartographer_node \
    -configuration_directory "${CONFIG_DIR}" \
    -configuration_basename "${CONFIG_BASENAME}" \
    -load_state_filename "${PBSTREAM}" \
    -start_trajectory_with_default_topics=false \
    scan:=${SCAN_TOPIC} \
    odom:=${ODOM_TOPIC} \
    __name:=cartographer_node >>"${LOG_FILE}" 2>&1 &
  log "[guard] cartographer_node spawned"
}

wait_for_cartographer_ready() {
  # 已知问题: 用 STABILIZING_WAIT=2s 偏短, 大 pbstream 加载可能超时.
  # 留作下一轮单独修. 本次不动.
  local max_wait="${1:-30}"
  local poll_interval=0.3
  local max_loops=$(( max_wait * 10 / 3 ))
  local count=0
  log "[guard] waiting for cartographer_node (max ${max_wait}s, poll=${poll_interval}s) ..."
  while [[ $count -lt $max_loops ]]; do
    if cartographer_running; then
      if rosservice list 2>/dev/null | grep -q "/start_trajectory"; then
        local elapsed=$(( count * 3 / 10 ))
        log "[guard] cartographer_node ready after ~${elapsed}s"
        return 0
      fi
    fi
    sleep "${poll_interval}"
    count=$((count + 1))
  done
  log "[guard] WARNING: cartographer_node not ready after ${max_wait}s"
  return 1
}

# ============================================================================
# 起新轨迹：固定 use_initial_pose=False，不注入任何 seed。
# 由 cartographer 后端全局定位（fast_correlative_scan_matcher）自行收敛，
# 与首次启动行为完全一致。
# ============================================================================
start_localization_trajectory() {
  python3 - "__hr_tag=start_trajectory" "${CONFIG_DIR}" "${CONFIG_BASENAME}" "${LOG_FILE}" >>"${LOG_FILE}" 2>&1 <<'PYEOF'
import sys
import time

import rospy
from cartographer_ros_msgs.srv import StartTrajectory, StartTrajectoryRequest

config_dir = sys.argv[2]
config_basename = sys.argv[3]
log_file = sys.argv[4]

def log(msg):
  with open(log_file, 'a', encoding='utf-8') as fp:
    fp.write(f"[{time.strftime('%F %T')}] [start_trajectory] {msg}\n")

rospy.init_node('hard_reset_start_trajectory', anonymous=True, disable_signals=True)

# 等服务
try:
  rospy.wait_for_service('/start_trajectory', timeout=10.0)
except Exception as e:
  log(f"FATAL: /start_trajectory unavailable: {e}")
  sys.exit(2)

start = rospy.ServiceProxy('/start_trajectory', StartTrajectory)

req = StartTrajectoryRequest()
req.configuration_directory = config_dir
req.configuration_basename = config_basename
req.use_initial_pose = False
req.relative_to_trajectory_id = 0

try:
  resp = start(req)
  msg = resp.status.message if resp.status.message else 'OK'
  log(f"start_trajectory returned trajectory_id={resp.trajectory_id} status={msg} "
      f"use_initial_pose=False")
except Exception as e:
  log(f"FATAL: start_trajectory call failed: {e}")
  sys.exit(3)
PYEOF
  local rc=$?
  if [[ $rc -ne 0 ]]; then
    log "[guard] WARNING: start_localization_trajectory exit rc=${rc}"
    return 1
  fi
  return 0
}

# ============================================================================
# Score guard：连续 N 帧低分即 os._exit(0)，由主循环捕获退出
# ============================================================================
run_guard() {
  python3 - "__hr_tag=guard" "$@" <<'PYEOF'
import os
import sys

import rospy
from std_msgs.msg import Float32

score_topic = sys.argv[2]
score_threshold = float(sys.argv[3])
score_consecutive = int(sys.argv[4])

class ScoreGuard:
  def __init__(self):
    self.low_count = 0
    rospy.init_node('hard_reset_guard', anonymous=True)
    rospy.Subscriber(score_topic, Float32, self.cb)
    rospy.loginfo(
      f"ScoreGuard active: score<{score_threshold}/{score_consecutive}frames -> hard reset")

  def cb(self, msg):
    if msg.data < score_threshold:
      self.low_count += 1
      if self.low_count >= score_consecutive:
        rospy.logwarn(
          f"TRIGGER: score={msg.data:.3f} < {score_threshold} x{self.low_count} -> EXIT")
        os._exit(0)
    else:
      self.low_count = 0

if __name__ == '__main__':
  ScoreGuard()
  rospy.spin()
PYEOF
}

# ============================================================================
# 主入口
# ============================================================================
log "=============================================="
log "[guard] HardResetSeeded"
log "[guard]   score      : ${SCORE_TOPIC} threshold=${SCORE_THRESHOLD} consecutive=${SCORE_CONSECUTIVE}"
log "[guard]   cooldown   : ${RECOVERY_COOLDOWN}s"
log "[guard]   stabilize  : ${STABILIZING_WAIT}s"
log "[guard]   bridge to  : ${BRIDGE_WAIT_TIMEOUT}s (timeout)"
log "[guard]   map        : ${MAP_FRAME}"
log "[guard]   scan_mp    : ${SCAN_MATCHED_TOPIC}"
log "[guard]   pbstream   : ${PBSTREAM}"
log "[guard]   config     : ${CONFIG_DIR}/${CONFIG_BASENAME}"
log "=============================================="

# 启动时彻底清场
cleanup_orphans_startup

log "[guard] >>> checking cartographer_node before monitor loop ..."
if cartographer_running; then
  log "[guard] /cartographer_node already running, continue monitoring"
else
  log "[guard] /cartographer_node not running, starting it now"
  _t0="$(ts_ms)"
  restart_cartographer
  wait_for_cartographer_ready "${STABILIZING_WAIT}"
  time_log "startup: restart + wait_ready" "${_t0}"

  log "[guard] >>> starting trajectory via /start_trajectory (first start, no seed) ..."
  _t0="$(ts_ms)"
  if ! start_localization_trajectory; then
    log "[guard] WARNING: first-start trajectory failed"
  fi
  time_log "startup: start_localization_trajectory" "${_t0}"
fi

while true; do
  log ""
  log "[guard] >>> monitoring ${SCORE_TOPIC} ..."

  run_guard "${SCORE_TOPIC}" "${SCORE_THRESHOLD}" "${SCORE_CONSECUTIVE}" >>"${LOG_FILE}" 2>&1
  guard_rc=$?

  log "[guard] !!! TRIGGER (rc=${guard_rc})"
  _t_total="$(ts_ms)"

  # 1) 起合并 bridge（caching 模式：缓存最后一帧 PC + 10Hz 顶住 /tf）
  _t0="$(ts_ms)"
  start_combined_bridge
  time_log "trigger: start_combined_bridge" "${_t0}"

  # 2) 杀老 carto
  _t0="$(ts_ms)"
  if cartographer_running; then
    kill_cartographer
    time_log "trigger: kill_cartographer" "${_t0}"
  else
    log "[guard] /cartographer_node not running, skip kill"
  fi

  # 3) 通知 bridge：老 carto 已死，进入 detecting 模式，准备捕获新 carto 的发布
  if [[ -n "${BRIDGE_PID}" ]] && kill -0 "${BRIDGE_PID}" 2>/dev/null; then
    kill -USR1 "${BRIDGE_PID}" 2>/dev/null || true
    log "[guard] sent SIGUSR1 to bridge (-> detecting mode)"
  fi

  # 4) 冷却
  _t0="$(ts_ms)"
  sleep "${RECOVERY_COOLDOWN}"
  time_log "trigger: cooldown_sleep" "${_t0}"

  # 5) 拉起新 carto
  _t0="$(ts_ms)"
  restart_cartographer
  wait_for_cartographer_ready "${STABILIZING_WAIT}"
  time_log "trigger: restart + wait_ready" "${_t0}"

  # 6) 起新轨迹（无 seed，use_initial_pose=False，与首次启动一致）
  _t0="$(ts_ms)"
  log "[guard] >>> starting trajectory via /start_trajectory (no seed) ..."
  if ! start_localization_trajectory; then
    log "[guard] WARNING: start_trajectory failed"
  fi
  time_log "trigger: start_localization_trajectory" "${_t0}"

  # 7) 等 bridge 自检测到新 carto 后自杀；超时兜底
  _t0="$(ts_ms)"
  wait_bridge_exit_or_timeout "${BRIDGE_WAIT_TIMEOUT}"
  time_log "trigger: wait_bridge_exit" "${_t0}"

  time_log "trigger: == TOTAL RECOVERY" "${_t_total}"

  if rosnode list 2>/dev/null | grep -q "/cartographer_node"; then
    log "[guard] cartographer_node restarted OK, resuming monitor ..."
  else
    log "[guard] WARNING: cartographer_node may have failed to restart"
  fi
done
