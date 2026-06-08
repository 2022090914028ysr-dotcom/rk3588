#!/usr/bin/env bash
# ============================================================================
# hard_reset_hot.sh �� �������ػ��ű�����ɱ�ڵ㣩
#
# �� hard_reset_seeded.sh ������
#   - ��ɱ cartographer_node ����
#   - ͨ�� finish_trajectory + start_trajectory ����ԭ�����ù켣
#   - TF (map->odom) ���ж� �� RViz ����
#   - ���ú󷢲� /initialpose ��Ϊ������ʾ����Э�����Լ����
#
# �÷���
#   ./hard_reset_hot.sh
# ============================================================================
set -uo pipefail

ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
CONFIG_DIR="${CONFIG_DIR:-$(timeout 5 rospack find cartographer_ros 2>/dev/null || echo '')/configuration_files}"
: "${CONFIG_DIR:?�޷��ҵ� cartographer_ros �������� source �����ռ�}"

LOG_DIR="${LOG_DIR:-/home/ubuntu/Nav_Release/Robit_SlamNav/NavOA_ws/logs}"
LOG_FILE="${LOG_FILE:-${LOG_DIR}/hard_reset_hot.log}"
mkdir -p "${LOG_DIR}"
: > "${LOG_FILE}"
exec 3>>"${LOG_FILE}"

POSE_CACHE_FILE="${POSE_CACHE_FILE:-${LOG_DIR}/hard_reset_hot.pose}"

log() {
  local line
  line="[$(date '+%F %T')] $*"
  printf '%s\n' "$line"
  printf '%s\n' "$line" >&3
}

CONFIG_BASENAME="${CONFIG_BASENAME:-livox_2d_localization.lua}"
PBSTREAM="${PBSTREAM:-/home/ubuntu/Nav_Release/Robit_SlamNav/NavOA_ws/src/map/meeting_room/meeting_room.pbstream}"
SCAN_TOPIC="${SCAN_TOPIC:-/scan}"
ODOM_TOPIC="${ODOM_TOPIC:-/odom_rf2o}"
MAP_FRAME="${MAP_FRAME:-map}"
BASE_FRAME="${BASE_FRAME:-base_footprint}"

SCORE_TOPIC="${SCORE_TOPIC:-/matching_score}"
SCORE_THRESHOLD="${SCORE_THRESHOLD:-0.60}"
SCORE_CONSECUTIVE="${SCORE_CONSECUTIVE:-30}"
RECOVERY_COOLDOWN="${RECOVERY_COOLDOWN:-0.3}"
STABILIZING_WAIT="${STABILIZING_WAIT:-0.5}"

export ROS_MASTER_URI

# ============================================================================
# ��̨λ�˻����ػ����� hard_reset_seeded.sh һ�£�
# ============================================================================
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
rate = rospy.Rate(5)

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

# ============================================================================
# �����ػ����� hard_reset_seeded.sh һ�£�
# ============================================================================
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

# ============================================================================
# �ڵ��������������ڲ�����ʱ����
# ============================================================================
cartographer_running() {
  rosnode list 2>/dev/null | grep -q "/cartographer_node"
}

start_cartographer_if_needed() {
  if cartographer_running; then
    log "[guard] /cartographer_node already running"
    return 0
  fi

  log "[guard] /cartographer_node not running, starting..."
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

  sleep "${STABILIZING_WAIT}"
}

# ============================================================================
# �����ã�finish_trajectory + start_trajectory����ɱ�ڵ㣬TF ���жϣ�
# ============================================================================
do_hot_reset() {
  log "[guard] >>> Hot reset: finish_trajectory + start_trajectory ..."

  python3 - "$CONFIG_DIR" "$CONFIG_BASENAME" "$POSE_CACHE_FILE" "$LOG_FILE" >>"${LOG_FILE}" 2>&1 <<'PYEOF'
import sys
import time
import rospy
from cartographer_ros_msgs.srv import FinishTrajectory, FinishTrajectoryRequest
from cartographer_ros_msgs.srv import StartTrajectory, StartTrajectoryRequest
from geometry_msgs.msg import PoseWithCovarianceStamped

config_dir = sys.argv[1]
config_basename = sys.argv[2]
pose_cache_file = sys.argv[3]
log_file = sys.argv[4]

def log(msg):
    with open(log_file, 'a', encoding='utf-8') as fp:
        fp.write(f"[{time.strftime('%F %T')}] [hot-reset] {msg}\n")

def read_cached_pose():
    try:
        with open(pose_cache_file, 'r') as fp:
            line = fp.read().strip()
        if not line:
            return None
        parts = line.split()
        return tuple(map(float, parts[:7]))
    except Exception:
        return None

rospy.init_node('hard_reset_hot', anonymous=True, disable_signals=True)

# Step 1: finish current trajectory
log("calling /finish_trajectory ...")
rospy.wait_for_service('/finish_trajectory', timeout=10.0)
finish = rospy.ServiceProxy('/finish_trajectory', FinishTrajectory)

finish_req = FinishTrajectoryRequest()
finish_req.trajectory_id = 0
finish_resp = finish(finish_req)
log(f"/finish_trajectory done: code={finish_resp.status.code}")

rospy.sleep(0.3)

# Step 2: start new trajectory WITHOUT prior constraint
log("calling /start_trajectory (no prior) ...")
rospy.wait_for_service('/start_trajectory', timeout=10.0)
start = rospy.ServiceProxy('/start_trajectory', StartTrajectory)

start_req = StartTrajectoryRequest()
start_req.configuration_directory = config_dir
start_req.configuration_basename = config_basename
start_req.use_initial_pose = False          # ��������Լ��
start_req.relative_to_trajectory_id = 0

start_resp = start(start_req)
traj_id = start_resp.trajectory_id
log(f"/start_trajectory done: trajectory_id={traj_id}")

# Step 3: publish /initialpose as search hint (high covariance)
cached = read_cached_pose()
if cached:
    px, py, pz, qx, qy, qz, qw = cached
    log(f"publishing /initialpose: x={px:.2f} y={py:.2f} (high cov)")

    rospy.sleep(0.5)
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1, latch=True)
    rospy.sleep(0.3)

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

    # ��Э���������ʾ����Լ��
    cov = [0.0] * 36
    cov[0] = 4.0   # var(x)
    cov[7] = 4.0   # var(y)
    cov[35] = 0.5  # var(yaw)
    msg.pose.covariance = cov

    pub.publish(msg)
    log("/initialpose published as search hint")
else:
    log("no cached pose, skipping /initialpose")
PYEOF

  local rc=$?
  if [ $rc -ne 0 ]; then
    log "[guard] WARNING: hot reset failed (rc=${rc})"
    return 1
  fi
  log "[guard] Hot reset complete"
}

# ============================================================================
# ������
# ============================================================================
log "=============================================="
log "[guard] HardResetHot (in-place reset, no kill)"
log "[guard]   score      : ${SCORE_TOPIC} threshold=${SCORE_THRESHOLD} consecutive=${SCORE_CONSECUTIVE}"
log "[guard]   cooldown   : ${RECOVERY_COOLDOWN}s"
log "[guard]   map/base   : ${MAP_FRAME} / ${BASE_FRAME}"
log "[guard]   pbstream   : ${PBSTREAM}"
log "[guard]   config     : ${CONFIG_DIR}/${CONFIG_BASENAME}"
log "[guard]   pose cache : ${POSE_CACHE_FILE}"
log "=============================================="

# ����λ�˻����ػ�
start_pose_cache_daemon

# ȷ����λ�ڵ�����
start_cartographer_if_needed

# �״γ�ʼ�� trajectory + ������ʾ
log "[guard] >>> Initial trajectory setup ..."
do_hot_reset

# ���ѭ��
while true; do
  log ""
  log "[guard] >>> Monitoring ${SCORE_TOPIC} ..."

  set +e
  run_guard "${SCORE_TOPIC}" "${SCORE_THRESHOLD}" "${SCORE_CONSECUTIVE}" "${RECOVERY_COOLDOWN}" >>"${LOG_FILE}" 2>&1
  guard_rc=$?
  set -e

  log "[guard] !!! TRIGGER (rc=${guard_rc})"

  # ȷ���ڵ㻹�ڣ��쳣�˳�ʱ�Զ�����
  start_cartographer_if_needed

  log "[guard] Waiting ${RECOVERY_COOLDOWN}s before hot reset..."
  sleep "${RECOVERY_COOLDOWN}"

  do_hot_reset

  if cartographer_running; then
    log "[guard] cartographer_node OK, resuming monitor..."
  else
    log "[guard] WARNING: cartographer_node lost, will restart on next cycle"
  fi
done
