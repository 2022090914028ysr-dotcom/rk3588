#!/bin/bash
# ==============================================================================
# cartographer_node_watchdog.sh �� Cartographer �ڵ��ص����л����Ź�
#
# ���ԣ��¾ɽڵ��ò�ͬ TF frame �������ͻ���л�ʱ�� TF relay ת����
#
# ʱ���ߣ����䣩��
#   t=0          ���ڿ�ʼ���ɽڵ������У����� map��odom��
#   t=160s       ����Ԥ�Ƚڵ㣨���� map_warmup��odom_warmup����ɽڵ㲻��ͻ��
#   t=190s       ɱ�ɽڵ� + ���� TF relay���� map_warmup��odom_warmup ת�� map��odom��
#   t=190s       �����ڿ�ʼ
#
# ������
#   - cartographer_ros ����cartographer_node��
#   - tf_frame_switch.py��TF ֡��ת���ڵ㣬���ű��Զ�������
#
# �÷���
#   �� launch �ļ����ڵ��ն��� source ������Ȼ��
#     ./cartographer_node_watchdog.sh
#
# ������������ѡ���ǣ���
#   CYCLE_PERIOD=180        ����(��)��Ĭ��180
#   WARMUP_BEFORE=20        ��ǰ����������Ԥ�Ƚڵ㣬Ĭ��20
#   OVERLAP_SEC=30          �¾��ص�ʱ��(��)��Ĭ��30
#   PBSTREAM=...            pbstream ·��
#   CONFIG_DIR=...          lua ����Ŀ¼
# ==============================================================================

set -e

# ======================== ���ò��� ========================
CYCLE_PERIOD="${CYCLE_PERIOD:-180}"           # �������� 3 ����
WARMUP_BEFORE="${WARMUP_BEFORE:-20}"           # ��ǰ 20s ����Ԥ�ȣ����� 2min40s ʱ��
OVERLAP_SEC="${OVERLAP_SEC:-30}"               # �¾��ص� 30s

PBSTREAM="${PBSTREAM:-/home/ubuntu/Nav_Release/Robit_SlamNav/NavOA_ws/src/carto_ws/map/meeting_room/test4.pbstream}"
CONFIG_DIR="${CONFIG_DIR:-}"
MAP_FILE="${MAP_FILE:-/home/ubuntu/Nav_Release/Robit_SlamNav/NavOA_ws/src/carto_ws/map/meeting_room/test4.yaml}"

# cartographer_node ����
CARTO_PKG="cartographer_ros"
CARTO_NODE="cartographer_node"

# ��ʽ��λ�õ� lua ���ã����� map��odom��
CONFIG_LIVE="livox_2d_localization.lua"
# Ԥ���õ� lua ���ã����� map_warmup��odom_warmup��
CONFIG_WARMUP="livox_2d_localization_warmup.lua"

# TF relay: �ű�����Ŀ¼
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
TF_SWITCH_SCRIPT="${SCRIPT_DIR}/tf_frame_switch.py"

SLEEP_BEFORE_WARMUP=$((CYCLE_PERIOD - WARMUP_BEFORE))    # 160s (2min40s)
TOTAL_CYCLE=$((SLEEP_BEFORE_WARMUP + OVERLAP_SEC))       # 190s (3min10s)

# ======================== ���ߺ��� ========================
log() { echo "[$(date '+%H:%M:%S')] [watchdog] $*"; }

find_carto_pids() {
    ps aux | grep -E '[c]artographer_node' | awk '{print $2}'
}

find_tf_switch_pids() {
    ps aux | grep -E '[t]f_frame_switch' | awk '{print $2}'
}

kill_proc() {
    local pid="$1"
    local label="$2"
    if [ -z "$pid" ]; then return; fi
    if ! kill -0 "$pid" 2>/dev/null; then return; fi
    log "  Killing $label (PID=$pid)..."
    kill "$pid" 2>/dev/null || true
    for i in $(seq 1 20); do
        if ! kill -0 "$pid" 2>/dev/null; then break; fi
        sleep 0.1
    done
    kill -9 "$pid" 2>/dev/null || true
}

kill_all_carto() {
    for pid in $(find_carto_pids); do
        kill_proc "$pid" "cartographer_node"
    done
}

cleanup() {
    log "Shutting down..."
    kill_all_carto
    for pid in $(find_tf_switch_pids); do
        kill_proc "$pid" "tf_frame_switch"
    done
    log "Cleanup done."
    exit 0
}
trap cleanup SIGINT SIGTERM

# ======================== ����Ԥ�Ƚڵ� ========================
start_warmup_node() {
    local iter="$1"
    local node_name="carto_warmup_${iter}"

    log "--- Starting WARMUP node: $node_name ---"
    log "    config: $CONFIG_WARMUP (publishes map_warmup -> odom_warmup)"

    ROS_NAMESPACE="/" rosrun "$CARTO_PKG" "$CARTO_NODE" \
        -configuration_directory "${CONFIG_DIR}" \
        -configuration_basename "${CONFIG_WARMUP}" \
        -load_state_filename "${PBSTREAM}" \
        __name:="${node_name}" \
        scan:=/scan \
        odom:=/odom_rf2o \
        &
}

# ======================== ����ԭʼ cartographer_node ========================
start_live_node() {
    local node_name="cartographer_node"

    log "--- Starting LIVE node: $node_name ---"
    log "    config: $CONFIG_LIVE (publishes map -> odom)"

    ROS_NAMESPACE="/" rosrun "$CARTO_PKG" "$CARTO_NODE" \
        -configuration_directory "${CONFIG_DIR}" \
        -configuration_basename "${CONFIG_LIVE}" \
        -load_state_filename "${PBSTREAM}" \
        __name:="${node_name}" \
        scan:=/scan \
        odom:=/odom_rf2o \
        &
}

# ======================== ���� TF ת���ڵ� ========================
start_tf_switch() {
    log "--- Starting TF Frame Switch (relay: map_warmup->odom_warmup => map->odom) ---"
    rosrun "$CARTO_PKG" tf_frame_switch.py __name:=tf_frame_switch _active:=true &
}

# ======================== ���� TF ת�� ========================
activate_tf_switch() {
    log "Activating TF relay..."
    # ͨ�� service �������ڵ���������
    if rosservice list 2>/dev/null | grep -q "/tf_frame_switch/enable"; then
        rosservice call /tf_frame_switch/enable "data: true" 2>/dev/null || true
    else
        # �ڵ㻹û������ֱ������
        start_tf_switch
    fi
}

deactivate_tf_switch() {
    if rosservice list 2>/dev/null | grep -q "/tf_frame_switch/enable"; then
        rosservice call /tf_frame_switch/enable "data: false" 2>/dev/null || true
    fi
}

# ======================== ��ѭ�� ========================
main() {
    log "============================================"
    log " Cartographer Node Hot-Swap Watchdog"
    log "   Cycle period:    ${CYCLE_PERIOD}s"
    log "   Warmup before:   ${WARMUP_BEFORE}s (at ${SLEEP_BEFORE_WARMUP}s mark)"
    log "   Overlap:         ${OVERLAP_SEC}s"
    log "   Total per cycle: ${TOTAL_CYCLE}s"
    log "   Live config:     ${CONFIG_LIVE}  => map -> odom"
    log "   Warmup config:   ${CONFIG_WARMUP} => map_warmup -> odom_warmup"
    log "   Pbstream:        ${PBSTREAM}"
    log "============================================"

    local iter=0
    local warmup_pid=""
    local tf_switch_pid=""

    while true; do
        iter=$((iter + 1))
        log ""
        log "==================== CYCLE $iter START ===================="

        # ---- Phase 1: �ȴ���������Ԥ�ȵ�ʱ�� ----
        log "Phase 1: Sleeping ${SLEEP_BEFORE_WARMUP}s (until 2min40s mark)..."
        sleep "$SLEEP_BEFORE_WARMUP"

        # ---- Phase 2: ����Ԥ�Ƚڵ㣨��ͬ TF frame �����޳�ͻ��----
        log "Phase 2: Starting warmup cartographer_node..."
        start_warmup_node "$iter"
        warmup_pid=$!
        log "  Warmup PID=$warmup_pid"

        # ---- Phase 3: �ص��ȴ� ----
        log "Phase 3: Overlap period �� waiting ${OVERLAP_SEC}s for warmup to stabilize..."
        sleep "$OVERLAP_SEC"

        # ---- Phase 4: �л���ɱ�ɽڵ� + ���� TF relay ----
        log "Phase 4: SWITCH! Killing OLD node(s)..."
        deactivate_tf_switch 2>/dev/null || true
        kill_all_carto

        # ���ݵȴ��� TF ���������
        sleep 0.5

        log "Phase 4: Activating TF relay (map_warmup->odom_warmup => map->odom)..."
        activate_tf_switch

        log "==================== SWITCH COMPLETE ===================="
        log "Active: warmup node (map_warmup->odom_warmup) + TF relay -> map->odom"
        log "Navigation stack sees: map -> odom (via relay)"
        log ""
    done
}

# ======================== ��� ========================
if ! command -v rosrun &> /dev/null; then
    echo "ERROR: rosrun not found. Please source your ROS setup.bash."
    echo "  e.g.: source /opt/ros/noetic/setup.bash"
    echo "        source /home/ubuntu/Nav_Release/Robit_SlamNav/NavOA_ws/devel/setup.bash"
    exit 1
fi

main "$@"
