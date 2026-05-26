#!/usr/bin/env bash
# ============================================================================
# hard_reset_seeded.sh �� ��λ��ע���Ӳ�����ػ��ű�
#
# Ŀ�꣺
#   �� /matching_score ������������������ cartographer_node��
#   ����ˤ��ǰ�����ԭʼλ����Ϊ initial pose ע�룬
#   ���¹켣��ˤ��λ�ø�����ʼ����ƥ�䣬�����Ǵӽ�ͼ��ʼ����������
#
# �÷���
#   ./hard_reset_seeded.sh
# ============================================================================
set -uo pipefail

ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
CONFIG_DIR="${CONFIG_DIR:-$(timeout 5 rospack find cartographer_ros 2>/dev/null || echo '')/configuration_files}"
: "${CONFIG_DIR:?�޷��ҵ� cartographer_ros �������� source �����ռ�}"

LOG_DIR="${LOG_DIR:-/home/ubuntu/Robit_SlamNav/NavOA_ws/logs}"
LOG_FILE="${LOG_FILE:-${LOG_DIR}/hard_reset_seeded.log}"
mkdir -p "${LOG_DIR}"
: > "${LOG_FILE}"
exec 3>>"${LOG_FILE}"

POSE_CACHE_FILE="${POSE_CACHE_FILE:-${LOG_DIR}/hard_reset_seeded.pose}"

log() {
  local line
  line="[$(date '+%F %T')] $*"
  printf '%s\n' "$line"
  printf '%s\n' "$line" >&3
}

CONFIG_BASENAME="${CONFIG_BASENAME:-livox_2d_localization.lua}"
PBSTREAM="${PBSTREAM:-/home/ubuntu/Robit_SlamNav/NavOA_ws/src/map/meeting_room/meeting_room.pbstream}"
SCAN_TOPIC="${SCAN_TOPIC:-/scan}"
ODOM_TOPIC="${ODOM_TOPIC:-/odom_rf2o}"
MAP_FRAME="${MAP_FRAME:-map}"
BASE_FRAME="${BASE_FRAME:-base_footprint}"
ODOM_FRAME="${ODOM_FRAME:-odom}"

TF_BRIDGE_PID=""

SCORE_TOPIC="${SCORE_TOPIC:-/matching_score}"
SCORE_THRESHOLD="${SCORE_THRESHOLD:-0.65}"
SCORE_CONSECUTIVE="${SCORE_CONSECUTIVE:-30}"
RECOVERY_COOLDOWN="${RECOVERY_COOLDOWN:-0.3}"
STABILIZING_WAIT="${STABILIZING_WAIT:-3}"

export ROS_MASTER_URI

cache_last_pose() {
  python3 - "$MAP_FRAME" "$BASE_FRAME" <<'PYEOF'
import sys
import rospy
import tf2_ros

map_frame = sys.argv[1]
base_frame = sys.argv[2]

rospy.init_node('hard_reset_pose_cache', anonymous=True, disable_signals=True)
buffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(buffer)

deadline = rospy.Time.now() + rospy.Duration(1.0)
while not rospy.is_shutdown() and rospy.Time.now() < deadline:
    try:
        tf_stamped = buffer.lookup_transform(map_frame, base_frame, rospy.Time(0), rospy.Duration(0.2))
        p = tf_stamped.transform.translation
        q = tf_stamped.transform.rotation
        print(f"{p.x} {p.y} {p.z} {q.x} {q.y} {q.z} {q.w}")
        sys.exit(0)
    except Exception:
        rospy.sleep(0.05)

sys.exit(1)
PYEOF
}

start_pose_cache_daemon() {
  python3 - "$MAP_FRAME" "$BASE_FRAME" "$POSE_CACHE_FILE" "$LOG_FILE" >>"${LOG_FILE}" 2>&1 <<'PYEOF' &
import os
import sys
import time

import rospy
import tf2_ros

map_frame = sys.argv[1]
base_frame = sys.argv[2]
pose_file = sys.argv[3]
log_file = sys.argv[4]

def log(msg):
  with open(log_file, 'a', encoding='utf-8') as fp:
    fp.write(f"[{time.strftime('%F %T')}] [pose-cache] {msg}\n")

def write_pose(line):
  tmp_file = pose_file + '.tmp'
  with open(tmp_file, 'w', encoding='utf-8') as fp:
    fp.write(line + '\n')
  os.replace(tmp_file, pose_file)

rospy.init_node('hard_reset_pose_cache', anonymous=True, disable_signals=True)
buffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(buffer)
rate = rospy.Rate(0.5)

log(f"daemon started: map={map_frame}, base={base_frame}, file={pose_file}")

while not rospy.is_shutdown():
  try:
    tf_stamped = buffer.lookup_transform(map_frame, base_frame, rospy.Time(0), rospy.Duration(0.2))
    p = tf_stamped.transform.translation
    q = tf_stamped.transform.rotation
    write_pose(f"{p.x} {p.y} {p.z} {q.x} {q.y} {q.z} {q.w}")
  except Exception:
    pass
  rate.sleep()
PYEOF
  POSE_CACHE_PID=$!
  log "[guard] pose cache daemon PID=${POSE_CACHE_PID}"
}

run_guard() {
  python3 - "$@" <<'PYEOF'
import sys
import os
import rospy
from std_msgs.msg import Float32

score_topic = sys.argv[1]
score_threshold = float(sys.argv[2])
score_consecutive = int(sys.argv[3])
cooldown_sec = float(sys.argv[4])

class ScoreGuard:
    def __init__(self):
        self.low_count = 0
        self.last_trigger = 0.0
        rospy.init_node('hard_reset_guard', anonymous=True)
        rospy.Subscriber(score_topic, Float32, self.cb)
        rospy.loginfo(
            f"ScoreGuard active: score<{score_threshold}/{score_consecutive}frames -> hard reset")

    def cb(self, msg):
        now = rospy.Time.now().to_sec()
        if now - self.last_trigger < cooldown_sec:
            self.low_count = 0
            return
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

kill_cartographer() {
  log "[guard] killing /cartographer_node ..."
  timeout 5 rosnode kill /cartographer_node 2>/dev/null || true
  pkill -f "[c]artographer_node" 2>/dev/null || true
  pkill -9 -f "[c]artographer_node" 2>/dev/null || true
}

cartographer_running() {
  rosnode list 2>/dev/null | grep -q "/cartographer_node"
}

# ---------------------------------------------------------------------------
# TF �Žӣ�ɱ cartographer_node �ڼ�ά�� map��odom����ֹ RViz ����
# ---------------------------------------------------------------------------
capture_map_odom_tf() {
  python3 - "$MAP_FRAME" "$ODOM_FRAME" <<'PYEOF'
import sys
import rospy
import tf2_ros

map_frame = sys.argv[1]
odom_frame = sys.argv[2]

rospy.init_node('hard_reset_tf_capture', anonymous=True, disable_signals=True)
buf = tf2_ros.Buffer()
tf2_ros.TransformListener(buf)

deadline = rospy.Time.now() + rospy.Duration(1.0)
while not rospy.is_shutdown() and rospy.Time.now() < deadline:
    try:
        ts = buf.lookup_transform(map_frame, odom_frame, rospy.Time(0), rospy.Duration(0.3))
        t = ts.transform.translation
        r = ts.transform.rotation
        print(f"{t.x} {t.y} {t.z} {r.x} {r.y} {r.z} {r.w}")
        sys.exit(0)
    except Exception:
        rospy.sleep(0.05)
sys.exit(1)
PYEOF
}

start_tf_bridge() {
  local tf_line
  tf_line="$(capture_map_odom_tf 2>/dev/null || true)"

  if [[ -z "$tf_line" ]]; then
    log "[guard] WARNING: �޷����� map->odom TF��TF �Ž�����"
    return 1
  fi

  read -r tx ty tz rx ry rz rw <<< "$tf_line"
  log "[guard] TF bridge: map->odom = (${tx},${ty},${tz}) q=(${rx},${ry},${rz},${rw})"

  rosrun tf static_transform_publisher \
    "${tx}" "${ty}" "${tz}" "${rx}" "${ry}" "${rz}" "${rw}" \
    "${MAP_FRAME}" "${ODOM_FRAME}" 100 &
  TF_BRIDGE_PID=$!
  log "[guard] TF bridge PID=${TF_BRIDGE_PID}"
  return 0
}

stop_tf_bridge() {
  if [[ -n "${TF_BRIDGE_PID:-}" ]]; then
    log "[guard] stopping TF bridge PID=${TF_BRIDGE_PID}"
    kill "${TF_BRIDGE_PID}" 2>/dev/null || true
    TF_BRIDGE_PID=""
  fi
}

restart_cartographer() {
  log "[guard] >>> Restarting cartographer_node ..."
  rosrun cartographer_ros cartographer_node \
    -configuration_directory "${CONFIG_DIR}" \
    -configuration_basename "${CONFIG_BASENAME}" \
    -load_state_filename "${PBSTREAM}" \
    -start_trajectory_with_default_topics=false \
    scan:=${SCAN_TOPIC} \
    odom:=${ODOM_TOPIC} \
    __name:=cartographer_node >>"${LOG_FILE}" 2>&1 &
  CARTO_PID=$!
  log "[guard] cartographer_node PID=${CARTO_PID}"
}

wait_for_cartographer_ready() {
  local max_wait="${1:-30}"
  local waited=0
  log "[guard] waiting for cartographer_node (max ${max_wait}s) ..."

  while [[ $waited -lt $max_wait ]]; do
    if cartographer_running; then
      if rosservice list 2>/dev/null | grep -q "/start_trajectory"; then
        log "[guard] cartographer_node ready after ${waited}s"
        return 0
      fi
      log "[guard]   node up, waiting for /start_trajectory ..."
    else
      log "[guard]   waiting for node registration ..."
    fi
    sleep 1
    waited=$((waited + 1))
  done

  log "[guard] WARNING: cartographer_node not ready after ${max_wait}s"
  return 1
}

start_seeded_trajectory() {
  python3 - "$CONFIG_DIR" "$CONFIG_BASENAME" >>"${LOG_FILE}" 2>&1 <<'PYEOF'
import sys
import rospy
from cartographer_ros_msgs.srv import StartTrajectory, StartTrajectoryRequest

config_dir = sys.argv[1]
config_basename = sys.argv[2]

rospy.init_node('hard_reset_start_trajectory', anonymous=True, disable_signals=True)
rospy.wait_for_service('/start_trajectory', timeout=10.0)
start = rospy.ServiceProxy('/start_trajectory', StartTrajectory)

req = StartTrajectoryRequest()
req.configuration_directory = config_dir
req.configuration_basename = config_basename
req.use_initial_pose = False
req.relative_to_trajectory_id = 0

resp = start(req)
rospy.loginfo('start_trajectory returned trajectory_id=%s cold start (no prior)',
             resp.trajectory_id)
PYEOF
  local rc_start=$?
  if [[ $rc_start -ne 0 ]]; then
    log "[guard] WARNING: /start_trajectory call failed (rc=${rc_start})"
    return 1
  fi

  local pose_line
  pose_line="$(cat "${POSE_CACHE_FILE}" 2>/dev/null || true)"

  if [[ -z "$pose_line" ]]; then
    log "[guard] WARNING: no cached pose, skip /initialpose"
    return 0
  fi

  read -r px py pz qx qy qz qw <<< "$pose_line"
  log "[guard] cached pose: x=${px} y=${py} z=${pz} q=(${qx},${qy},${qz},${qw}) -> /initialpose (high cov)"

  python3 - "$px" "$py" "$pz" "$qx" "$qy" "$qz" "$qw" >>"${LOG_FILE}" 2>&1 <<'PYEOF'
import sys
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

px, py, pz, qx, qy, qz, qw = map(float, sys.argv[1:8])

rospy.init_node('hard_reset_initialpose', anonymous=True, disable_signals=True)
pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1, latch=True)
rospy.sleep(0.05)

msg = PoseWithCovarianceStamped()
msg.header.stamp = rospy.Time.now()
msg.header.frame_id = 'map'
msg.pose.pose.position.x = px
msg.pose.pose.position.y = py
msg.pose.pose.position.z = pz
msg.pose.pose.orientation.x = qx
msg.pose.pose.orientation.y = qy
msg.pose.pose.orientation.z = qz
msg.pose.pose.orientation.w = qw

cov = [0.0] * 36
cov[0] = 4.0
cov[7] = 4.0
cov[35] = 0.5
msg.pose.covariance = cov

pub.publish(msg)
rospy.loginfo('Published /initialpose as search hint (high covariance)')
PYEOF
  local rc_pub=$?
  if [[ $rc_pub -ne 0 ]]; then
    log "[guard] WARNING: /initialpose publish failed (rc=${rc_pub})"
    return 1
  fi

  return 0
}

log "=============================================="
log "[guard] HardResetSeeded"
log "[guard]   score      : ${SCORE_TOPIC} threshold=${SCORE_THRESHOLD} consecutive=${SCORE_CONSECUTIVE}"
log "[guard]   cooldown   : ${RECOVERY_COOLDOWN}s"
log "[guard]   stabilize  : ${STABILIZING_WAIT}s"
log "[guard]   map/base   : ${MAP_FRAME} / ${BASE_FRAME}"
log "[guard]   pbstream   : ${PBSTREAM}"
log "[guard]   config     : ${CONFIG_DIR}/${CONFIG_BASENAME}"
log "[guard]   pose cache : ${POSE_CACHE_FILE}"
log "=============================================="

start_pose_cache_daemon

log "[guard] >>> Checking cartographer_node before monitor loop ..."
if cartographer_running; then
  log "[guard] /cartographer_node already running, continue monitoring"
else
  log "[guard] /cartographer_node not running, starting it now"
  restart_cartographer

  wait_for_cartographer_ready "${STABILIZING_WAIT}"

  log "[guard] >>> Injecting cached pose via /start_trajectory ..."
  if ! start_seeded_trajectory; then
    log "[guard] WARNING: start_trajectory seed injection failed"
  fi
fi

while true; do
  log ""
  log "[guard] >>> Monitoring ${SCORE_TOPIC} ..."

  set +e
  run_guard "${SCORE_TOPIC}" "${SCORE_THRESHOLD}" "${SCORE_CONSECUTIVE}" "${RECOVERY_COOLDOWN}" >>"${LOG_FILE}" 2>&1
  guard_rc=$?
  set -e

  log "[guard] !!! TRIGGER (rc=${guard_rc})"

  # 1. ���浱ǰ map->odom TF��������̬�Žӷ�ֹ RViz ��ס
  start_tf_bridge

  # 2. ɱ�ɽڵ㣨TB �Ž��ڼ䣬RViz ���ܿ��� map ֡��
  if cartographer_running; then
    kill_cartographer
  else
    log "[guard] /cartographer_node not running, will start localization node first"
  fi

  log "[guard] Waiting ${RECOVERY_COOLDOWN}s before restart..."
  sleep "${RECOVERY_COOLDOWN}"

  # 3. �����½ڵ�
  restart_cartographer

  wait_for_cartographer_ready "${STABILIZING_WAIT}"

  # 4. �½ڵ��ѽӹ� TF���رվ�̬�Ž�
  stop_tf_bridge

  log "[guard] >>> Injecting cached pose via /start_trajectory ..."
  if ! start_seeded_trajectory; then
    log "[guard] WARNING: start_trajectory seed injection failed"
  fi

  if rosnode list 2>/dev/null | grep -q "/cartographer_node"; then
    log "[guard] cartographer_node restarted OK, resuming monitor..."
  else
    log "[guard] WARNING: cartographer_node may have failed to restart"
  fi
done