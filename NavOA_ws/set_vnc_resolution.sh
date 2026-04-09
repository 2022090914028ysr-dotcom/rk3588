#!/usr/bin/env bash
set -euo pipefail

# Script: set_vnc_resolution.sh
# Description: Sets virtual desktop to 1920x1080 for VNC while keeping physical display at native resolution
# Usage: bash set_vnc_resolution.sh

RESOLUTION="1920x1080"
PRIMARY_OUTPUT="DSI-1"

echo "Setting virtual resolution to ${RESOLUTION}..."

# Auto-detect active display
for disp in ":0" ":1" ":1024" ":1025"; do
    if DISPLAY=$disp xrandr -q &>/dev/null; then
        export DISPLAY=$disp
        echo "Using display: $disp"
        break
    fi
done

# Get current output name if not known
if [ "$PRIMARY_OUTPUT" = "DSI-1" ]; then
    PRIMARY_OUTPUT=$(xrandr | grep " connected primary" | cut -d' ' -f1 || echo "DSI-1")
    echo "Detected output: $PRIMARY_OUTPUT"
fi

# Get native resolution
NATIVE_RES=$(xrandr | grep -A1 "^${PRIMARY_OUTPUT}" | grep -oP '\d+x\d+' | head -1 || echo "360x360")
echo "Native resolution: $NATIVE_RES"

# Set framebuffer to 1920x1080 with scaling
xrandr --fb ${RESOLUTION} --output ${PRIMARY_OUTPUT} --mode ${NATIVE_RES} --scale-from ${RESOLUTION} --panning ${RESOLUTION}

echo "Done! VNC will now show ${RESOLUTION}. Physical display remains at native resolution."
echo "Connect via VNC to see the full 1920x1080 desktop."
