#!/bin/bash
# ==============================================================================
# cartographer_node_watchdog.sh �� �ڵ㼶��ʱ�������Ź�
#
# ���ܣ�ÿ�� CYCLE_PERIOD �룬��"�����º�ɱ��"�������� cartographer_node��
#       �������ٶ�λä��ʱ�䡣
#
# ��������������ص����л�������
#   ���ۿ��У��� Cartographer ͨ�� tf::TransformBroadcaster �� /tf ����
#   map��odom �任��ROS1 �� tf::TransformBroadcaster ��֧�� topic remap��
#   �����ڵ�ͬʱ�� /tf �ᵼ�� TF ������������ջ�յ����������λ�ˡ�
#
#   ���ű����õ����з�����
#     1. �����½ڵ㣨ʹ����ʱ namespace ���� TF���������������еĽڵ㣩
#     2. ɱ���ɽڵ�
#     3. �½ڵ�� TF �� namespace �²��ɼ�����˶������� tf_relay ת��
#     4. ��һ��ʱɱ�� relay + ���½ڵ㣬���¿�ʼ
#
#   ʵ�ʲ����У�Cartographer �� pbstream ���غܿ죨1~3�룩��
#   ֱ�ӿ���˳��������ɱ������Ҳ��ȫ���С�
#
# ʹ�÷�����
#   ./cartographer_node_watchdog.sh
#
# ������������ѡ����Ĭ��ֵ����
#   CYCLE_PERIOD=180        # ��ת����(��)��Ĭ��180
#   OVERLAP_TIME=30         # �¾ɽڵ��ص�ʱ��(��)��Ĭ��30
#   PBSTREAM_FILE=...       # pbstream ·��
#   CONFIG_DIR=...          # �����ļ�Ŀ¼
#   CONFIG_FILE=...         # �����ļ���
# ==============================================================================

set -e

# ===================== ���� =====================
CYCLE_PERIOD="${CYCLE_PERIOD:-180}"           # 3����
OVERLAP_TIME="${OVERLAP_TIME:-30}"             # �¾��ص�30��
WARMUP_BEFORE_KILL=$((CYCLE_PERIOD - OVERLAP_TIME))  # �����½ڵ��ʱ���

PBSTREAM_FILE="${PBSTREAM_FILE:-/home/ubuntu/Nav_Release/Robit_SlamNav/NavOA_ws/src/carto_ws/map/meeting_room/test_2.pbstream}"
CONFIG_DIR="${CONFIG_DIR:-}"
CONFIG_FILE="${CONFIG_FILE:-livox_2d_localization.lua}"

# cartographer_node ��������
CARTO_ARGS="-configuration_directory ${CONFIG_DIR} -configuration_basename ${CONFIG_FILE} -load_state_filename ${PBSTREAM_FILE}"

# ROS ����
ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"

# ===================== ��־ =====================
log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] [watchdog] $*"
}

# ===================== ���� cartographer_node ���� =====================
find_carto_pids() {
    # �������� cartographer_node ����
    ps aux | grep -E 'cartographer_node' | grep -v grep | grep -v watchdog | awk '{print $2}'
}

find_tf_relay_pids() {
    ps aux | grep -E 'tf_relay|tf2_ros.*relay' | grep -v grep | awk '{print $2}'
}

# ===================== ɱ��ָ������ =====================
kill_node() {
    local pids="$1"
    local label="$2"
    if [ -z "$pids" ]; then
        return
    fi
    for pid in $pids; do
        log "Killing $label (PID=$pid)..."
        kill "$pid" 2>/dev/null || true
        # �ȴ���� 3 �룬���о�ǿɱ
        for i in $(seq 1 30); do
            if ! kill -0 "$pid" 2>/dev/null; then
                break
            fi
            sleep 0.1
        done
        kill -9 "$pid" 2>/dev/null || true
    done
}

# ===================== �ȴ��½ڵ���� =====================
wait_for_carto_ready() {
    local timeout="${1:-30}"
    local start_time=$(date +%s)
    
    log "Waiting for cartographer_node to be ready (timeout=${timeout}s)..."
    
    while true; do
        # ��� cartographer_node �Ƿ��Ѿ�������ͨ�� rosnode list��
        local nodes
        nodes=$(rosnode list 2>/dev/null || echo "")
        if echo "$nodes" | grep -q "cartographer_node"; then
            log "cartographer_node is up!"
            # �ٵ�һ��������� pbstream ���غͳ�ʼ��λ
            sleep 2
            return 0
        fi
        
        local now=$(date +%s)
        if [ $((now - start_time)) -ge "$timeout" ]; then
            log "ERROR: timeout waiting for cartographer_node"
            return 1
        fi
        sleep 0.5
    done
}

# ===================== ��ѭ�� =====================
main() {
    log "============================================"
    log "Cartographer Node Watchdog Started"
    log "  Cycle period:     ${CYCLE_PERIOD}s"
    log "  Overlap time:     ${OVERLAP_TIME}s"
    log "  Warmup trigger at: ${WARMUP_BEFORE_KILL}s"
    log "  Pbstream:         ${PBSTREAM_FILE}"
    log "  Config:           ${CONFIG_FILE}"
    log "============================================"

    local iteration=0
    local active_ns=""        # ��ǰ��Ծ�� namespace���� = Ĭ�� / �����ռ䣩
    local active_node_name="" # ��ǰ��Ծ�Ľڵ���

    while true; do
        iteration=$((iteration + 1))
        log "=== Iteration $iteration ==="

        # ---- �׶�һ���ȴ��� WARMUP_BEFORE_KILL ----
        log "Waiting ${WARMUP_BEFORE_KILL}s until next warmup..."
        sleep "$WARMUP_BEFORE_KILL"

        # ---- �׶ζ��������½ڵ㣨��ʱ namespace ���룩 ----
        local new_ns="carto_$iteration"
        local new_node_name="cartographer_node_$iteration"

        log "Starting NEW cartographer_node in namespace /$new_ns (warming up)..."
        
        # �ڶ����� namespace ��������TF ������Ⱦ /tf
        ROS_NAMESPACE="$new_ns" \
            rosrun cartographer_ros cartographer_node \
            ${CARTO_ARGS} \
            __name:="$new_node_name" \
            scan:=/scan \
            odom:=/odom_rf2o \
            &
        
        local new_pid=$!
        log "New node PID=$new_pid, namespace=/$new_ns"

        # ---- �׶������ȴ��ص�ʱ�� ----
        log "Overlap period: waiting ${OVERLAP_TIME}s for new node to stabilize..."
        sleep "$OVERLAP_TIME"

        # ---- �׶��ģ�ɱ���ɽڵ� ----
        if [ -n "$active_ns" ]; then
            log "Killing OLD active node (namespace=/$active_ns)..."
            kill_node "$(find_carto_pids)" "old cartographer_node"
        else
            # �״Σ�ɱ�� launch �ļ�������ԭʼ cartographer_node
            log "Killing original cartographer_node (no namespace)..."
            kill_node "$(find_carto_pids)" "original cartographer_node"
        fi

        # ---- �׶��壺TF �Žӣ��� namespace �µ� TF ת���� /tf�� ----
        # ɱ���ɵ� relay������У�
        local old_relay_pids=$(find_tf_relay_pids)
        kill_node "$old_relay_pids" "old tf_relay"

        log "Starting TF relay from /$new_ns/tf -> /tf ..."
        # ʹ�� tf2_ros �� static_transform_publisher ��������Ҫ��̬ת��
        # ������һ���򵥵� rostopic relay �� TF ��Ϣת��
        # ע�⣺tf2_ros ���� namespaced TF ��Ҫ����·��
        rosrun topic_tools relay "/$new_ns/tf" "/tf" &
        local relay_pid=$!
        log "TF relay PID=$relay_pid"

        # ---- �׶���������״̬ ----
        active_ns="$new_ns"
        active_node_name="$new_node_name"

        log "Switch complete! Active node: /$active_ns/$active_node_name"
        log "Next cycle in ${CYCLE_PERIOD}s"
    done
}

# ===================== �źŴ��� =====================
cleanup() {
    log "Shutting down watchdog..."
    local pids=$(find_carto_pids)
    kill_node "$pids" "cartographer_node"
    local rpids=$(find_tf_relay_pids)
    kill_node "$rpids" "tf_relay"
    log "Cleanup done."
    exit 0
}

trap cleanup SIGINT SIGTERM

# ===================== ��� =====================
# ��� ROS ����
if ! command -v rosnode &> /dev/null; then
    echo "ERROR: rosnode not found. Please source your ROS setup.bash first."
    exit 1
fi

main "$@"
