#!/usr/bin/env bash
set -euo pipefail

SESSION_NAME="${1:-navoa}"

# Prefer the known workspace path; fall back to script directory.
DEFAULT_WS="$HOME/Robit_SlamNav/NavOA_ws"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ -d "$DEFAULT_WS" ]]; then
	WORKSPACE_DIR="$DEFAULT_WS"
else
	WORKSPACE_DIR="$SCRIPT_DIR"
fi

if [[ ! -d "$WORKSPACE_DIR/devel_isolated" ]]; then
	echo "ERROR: Expected a ROS workspace at: $WORKSPACE_DIR" >&2
	echo "Run this script from NavOA_ws or ensure $HOME/Robit_SlamNav/NavOA_ws exists." >&2
	exit 1
fi

if ! command -v tmux >/dev/null 2>&1; then
	echo "ERROR: tmux is required to split the terminal." >&2
	echo "Install it with: sudo apt-get update && sudo apt-get install -y tmux" >&2
	exit 1
fi

# Ensure conda activation works in non-interactive shells.
ensure_conda() {
	if command -v conda >/dev/null 2>&1; then
		return 0
	fi

	local candidates=(
		"$HOME/miniconda3/etc/profile.d/conda.sh"
		"$HOME/anaconda3/etc/profile.d/conda.sh"
		"/opt/conda/etc/profile.d/conda.sh"
	)

	for f in "${candidates[@]}"; do
		if [[ -f "$f" ]]; then
			# shellcheck disable=SC1090
			source "$f"
			return 0
		fi
	done

	echo "ERROR: conda not found in PATH, and conda.sh not found in common locations." >&2
	echo "Fix by running: source \"$(conda info --base)/etc/profile.d/conda.sh\" (or install conda)." >&2
	exit 1
}

ensure_conda
CONDA_BASE="$(conda info --base)"

# Run once: open serial port (as per README).
echo "[start_nav] Using workspace: $WORKSPACE_DIR"
echo "[start_nav] Opening serial port via port.sh ..."
cd "$WORKSPACE_DIR"
sh port.sh

# If the session already exists, just attach.
if tmux has-session -t "$SESSION_NAME" >/dev/null 2>&1; then
	echo "[start_nav] tmux session '$SESSION_NAME' already exists; attaching..."
	exec tmux attach -t "$SESSION_NAME"
fi

COMMON_SETUP=$(cat <<EOF
cd "$WORKSPACE_DIR" \
 && source "$WORKSPACE_DIR/devel_isolated/setup.bash" \
 && source "$CONDA_BASE/etc/profile.d/conda.sh" \
 && conda activate control
EOF
)

CMD_PANE1="$COMMON_SETUP && taskset -c 4,5 roslaunch NavOA_ros NavOA_gazebo_limo_all.launch"
CMD_PANE2="$COMMON_SETUP && export DISPLAY=:0 && roslaunch NavOA_ros NavOA_gazebo_limo.launch"

# Create tmux session and split into 2 panes.
tmux new-session -d -s "$SESSION_NAME" -c "$WORKSPACE_DIR"
tmux split-window -h -t "$SESSION_NAME":0 -c "$WORKSPACE_DIR"

# Start pane 1 immediately.
tmux send-keys -t "$SESSION_NAME":0.0 "bash -lc $(printf %q "$CMD_PANE1")" C-m

# Wait a bit for limo_all to bring everything up, then start pane 2.
sleep 5
tmux send-keys -t "$SESSION_NAME":0.1 "bash -lc $(printf %q "$CMD_PANE2")" C-m

# Attach to the session.
exec tmux attach -t "$SESSION_NAME"
