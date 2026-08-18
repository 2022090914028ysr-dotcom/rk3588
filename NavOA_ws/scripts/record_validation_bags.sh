#!/usr/bin/env bash
# =============================================================================
# record_validation_bags.sh
#
# Record rosbags for EKF covariance validation under sensor degradation.
#
# Usage:
#   source /opt/ros/noetic/setup.bash   # or your ROS distro
#   bash record_validation_bags.sh [output_dir]
#
# Each bag captures: fused /odom, rf2o-only /odom_rf2o, leg-odom SMX/Odom,
#                     /tf, /tf_static, and /scan.
#
# IMPORTANT — before running:
#   1. Verify the leg-odom topic is publishing:
#        rostopic echo SMX/Odom -n1 | head
#      If the resolved name is different (e.g. /SMX/Odom), update TOPIC_LEG below.
#   2. Confirm all topics are live:
#        rostopic list | grep -E '/odom$|/odom_rf2o|SMX|/tf$|/tf_static|/scan'
# =============================================================================

set -euo pipefail

OUTPUT_DIR="${1:-$(pwd)/validation_bags}"
TOPIC_FUSED="/odom"
TOPIC_RF2O="/odom_rf2o"
TOPIC_LEG="SMX/Odom"          # relative → resolves to /SMX/Odom in global namespace
TOPIC_TF="/tf"
TOPIC_TF_STATIC="/tf_static"
TOPIC_SCAN="/scan"

ALL_TOPICS="$TOPIC_FUSED $TOPIC_RF2O $TOPIC_LEG $TOPIC_TF $TOPIC_TF_STATIC $TOPIC_SCAN"

mkdir -p "$OUTPUT_DIR"

echo "============================================"
echo " EKF Covariance Validation — Bag Recorder"
echo "============================================"
echo "Output dir : $OUTPUT_DIR"
echo "Topics     : $ALL_TOPICS"
echo ""

# --- Sanity check: are topics publishing? --------------------------------
echo ">>> Checking topic liveness (rostopic list)..."
if ! rostopic list > /dev/null 2>&1; then
    echo "ERROR: roscore does not appear to be running."
    exit 1
fi

LIVE_TOPICS=$(rostopic list)
for t in $ALL_TOPICS; do
    # Resolve relative names the same way ROS does
    resolved="${t#/}"          # strip leading / if present
    resolved="/${resolved}"    # prepend / → always absolute
    if echo "$LIVE_TOPICS" | grep -qF "$resolved"; then
        echo "  [OK]  $resolved"
    else
        echo "  [WARN] $resolved — not found in rostopic list. Continuing anyway..."
    fi
done
echo ""

# =============================================================================
# SCENARIO 1 — static (5 min)
#   Robot stationary. Baseline for drift and noise level.
# =============================================================================
echo "============================================"
echo " SCENARIO: static"
echo " Place the robot still on the ground."
echo " Press ENTER to start recording (5 min) ..."
echo "============================================"
read -r
rosbag record -O "$OUTPUT_DIR/static.bag" \
    --duration=300 \
    --buffsize=4096 \
    --chunksize=2048 \
    $ALL_TOPICS
echo ">>> static.bag recorded."
echo ""

# =============================================================================
# SCENARIO 2 — figure8 (normal conditions, accuracy sanity)
#   Drive a figure-8 loop and return to a tape mark.
# =============================================================================
echo "============================================"
echo " SCENARIO: figure8"
echo " Drive a figure-8 loop. Return to the marked start position."
echo " Press ENTER to start recording. Press Ctrl+C to stop."
echo "============================================"
read -r
rosbag record -O "$OUTPUT_DIR/figure8.bag" \
    --buffsize=4096 \
    --chunksize=2048 \
    $ALL_TOPICS
echo ">>> figure8.bag recorded."
echo ""

# =============================================================================
# SCENARIO 3 — blank_corridor
#   Drive through a featureless corridor where rf2o should degrade.
#   Expect: rf2o drift grows, leg-odom stays reasonable.
# =============================================================================
echo "============================================"
echo " SCENARIO: blank_corridor"
echo " Drive through a long, featureless corridor."
echo " rf2o scan-matching should degrade (few geometric features)."
echo " Press ENTER to start. Press Ctrl+C to stop."
echo "============================================"
read -r
rosbag record -O "$OUTPUT_DIR/blank_corridor.bag" \
    --buffsize=4096 \
    --chunksize=2048 \
    $ALL_TOPICS
echo ">>> blank_corridor.bag recorded."
echo ""

# =============================================================================
# SCENARIO 4 — lidar_cross
#   Someone walks through the LiDAR plane mid-walk.
#   Expect: rf2o jumps or drifts briefly.
# =============================================================================
echo "============================================"
echo " SCENARIO: lidar_cross"
echo " Drive normally. Have someone walk through the LiDAR plane."
echo " Press ENTER to start. Press Ctrl+C to stop."
echo "============================================"
read -r
rosbag record -O "$OUTPUT_DIR/lidar_cross.bag" \
    --buffsize=4096 \
    --chunksize=2048 \
    $ALL_TOPICS
echo ">>> lidar_cross.bag recorded."
echo ""

# =============================================================================
# SCENARIO 5 — kidnap
#   Lift and move the robot ~0.5 m mid-run, then set it back down.
#   Expect: large discontinuity in rf2o; leg-odom may also jump.
# =============================================================================
echo "============================================"
echo " SCENARIO: kidnap"
echo " Drive. Mid-run, lift the robot, move ~0.5 m, set it down."
echo " Press ENTER to start. Press Ctrl+C to stop."
echo "============================================"
read -r
rosbag record -O "$OUTPUT_DIR/kidnap.bag" \
    --buffsize=4096 \
    --chunksize=2048 \
    $ALL_TOPICS
echo ">>> kidnap.bag recorded."
echo ""

echo "============================================"
echo " All bags recorded in: $OUTPUT_DIR"
echo ""
echo " Next: run analysis on each bag:"
echo "   python3 analyze_bag.py $OUTPUT_DIR/static.bag static"
echo "   python3 analyze_bag.py $OUTPUT_DIR/figure8.bag figure8"
echo "   ..."
echo "============================================"
