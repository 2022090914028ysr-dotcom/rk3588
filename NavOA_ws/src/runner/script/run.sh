#!/usr/bin/env bash
#===============================================================================
# run.sh — SlamNav system launcher
#
# Convenience wrapper around roslaunch runner <mode>.launch
#
# Prerequisites:
#   Source the workspace before running:
#     source workspace_root/devel_isolated/setup.bash
#
# Usage:
#   ./run.sh                   Show this help
#   ./run.sh mapping           Start SLAM mapping mode
#   ./run.sh localization      Start localization + navigation mode
#   ./run.sh auto_mapping      Start autonomous mapping mode (SLAM + NavOA + MARL)
#   ./run.sh sensors           Start sensors only (debug mode)
#   ./run.sh mapping detached  Launch in a new tmux session (non-interactive)
#===============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

# ── Auto-source workspace if ROS_DISTRO is not set ───────────────────────────
if [ -z "${ROS_DISTRO}" ]; then
    # Source workspace setup — it chains to the ROS distro setup automatically
    for ws_setup in \
        "${WORKSPACE_DIR}/devel_isolated/setup.bash" \
        "${WORKSPACE_DIR}/devel/setup.bash"; do
        if [ -f "$ws_setup" ]; then
            source "$ws_setup"
            break
        fi
    done

    # Still no ROS?  Ask the user to source manually.
    if [ -z "${ROS_DISTRO}" ]; then
        echo "ERROR: ROS not found. Source your workspace first, e.g.:"
        echo "  source ${WORKSPACE_DIR}/devel_isolated/setup.bash"
        exit 1
    fi
fi

# ── Help ─────────────────────────────────────────────────────────────────────
show_help() {
    echo "SlamNav System Launcher"
    echo ""
    echo "Usage: $0 [mode] [detached]"
    echo ""
    echo "Modes:"
    echo "  mapping         SLAM mapping — sensors + Cartographer SLAM + RViz"
    echo "  localization    Localization + navigation — sensors + Cartographer"
    echo "                  localization + NavOA move_base + controllers + RViz"
    echo "  auto_mapping    Autonomous mapping — sensors + Cartographer SLAM"
    echo "                  + NavOA navigation + MARL exploration + RViz"
    echo "  sensors         Sensors only — LiDAR + camera + ultrasonic + IMU"
    echo "                  + RF2O odometry (no SLAM or navigation)"
    echo ""
    echo "Append 'detached' to launch in a tmux session."
}

# ── TMUX launcher (detached mode) ────────────────────────────────────────────
launch_detached() {
    local mode="$1"
    local session_name="slamnav_${mode}"

    if tmux has-session -t "$session_name" 2>/dev/null; then
        echo "Session '$session_name' already exists. Attaching ..."
        tmux attach-session -t "$session_name"
        exit 0
    fi

    tmux new-session -d -s "$session_name" \
        "roslaunch runner ${mode}.launch; read -p 'Press Enter to close ...'"
    echo "Launched mode '${mode}' in tmux session '${session_name}'."
    echo "Attach: tmux attach -t ${session_name}"
}

# ── Main ─────────────────────────────────────────────────────────────────────
MODE="${1:-help}"
DETACHED="${2:-}"

if [ "$MODE" = "help" ] || [ "$MODE" = "-h" ] || [ "$MODE" = "--help" ]; then
    show_help
    exit 0
fi

VALID_MODES="mapping localization auto_mapping sensors"
if ! echo "$VALID_MODES" | grep -qw "$MODE"; then
    echo "ERROR: Unknown mode '$MODE'"
    echo "Valid modes: mapping, localization, auto_mapping, sensors"
    exit 1
fi

if [ "$DETACHED" = "detached" ]; then
    launch_detached "$MODE"
else
    echo "=== SlamNav: Starting '${MODE}' mode ==="
    echo "Press Ctrl+C to stop all processes."
    echo ""
    roslaunch runner "${MODE}.launch"
fi
