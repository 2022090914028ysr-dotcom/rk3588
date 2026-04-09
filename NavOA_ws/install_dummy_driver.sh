#!/usr/bin/env bash
set -euo pipefail

# Script: install_dummy_driver.sh
# Description: Installs and configures xserver-xorg-video-dummy for 1920x1080 headless display
# Usage: sudo bash install_dummy_driver.sh  OR  bash install_dummy_driver.sh (will prompt for sudo password)

RESOLUTION="1920x1080"
REFRESH_RATE="60.00"

echo "=== Installing xserver-xorg-video-dummy for ${RESOLUTION}@${REFRESH_RATE} ==="

# Check if running as root
if [ "$EUID" -eq 0 ]; then
    SUDO=""
else
    echo "This script requires sudo privileges."
    read -sp "Enter sudo password: " SUDO_PASS
    echo
    SUDO="echo '${SUDO_PASS}' | sudo -S"
fi

# Install dummy driver
echo "[1/3] Installing xserver-xorg-video-dummy..."
eval $SUDO apt-get update -y
eval $SUDO apt-get install -y xserver-xorg-video-dummy

# Create xorg.conf
echo "[2/3] Creating /etc/X11/xorg.conf..."
eval $SUDO tee /etc/X11/xorg.conf > /dev/null << 'XORG_EOF'
Section "Device"
    Identifier  "DummyDevice"
    Driver      "dummy"
    VideoRam    256000
EndSection

Section "Monitor"
    Identifier  "DummyMonitor"
    HorizSync   30-80
    VertRefresh 50-70
    ModeLine    "1920x1080_60.00" 173.00 1920 2048 2248 2576 1080 1083 1088 1120 -hsync +vsync
    Option      "PreferredMode" "1920x1080_60.00"
EndSection

Section "Screen"
    Identifier  "DummyScreen"
    Device      "DummyDevice"
    Monitor     "DummyMonitor"
    DefaultDepth 24
    SubSection "Display"
        Viewport 0 0
        Depth    24
        Modes    "1920x1080_60.00"
    EndSubSection
EndSection
XORG_EOF

# Restart display manager
echo "[3/3] Restarting display manager..."
if eval $SUDO systemctl is-active --quiet gdm3 2>/dev/null; then
    eval $SUDO systemctl restart gdm3
elif eval $SUDO systemctl is-active --quiet gdm 2>/dev/null; then
    eval $SUDO systemctl restart gdm
else
    echo "Warning: Could not detect display manager (gdm/gdm3). Please reboot manually."
fi

# Verify
sleep 8
echo "Checking display..."
for disp in ":0" ":1" ":1024" ":1025"; do
    if DISPLAY=$disp xrandr -q &>/dev/null; then
        echo "Found active display: $disp"
        break
    fi
done

if command -v xrandr &> /dev/null; then
    echo ""
    echo "=== Display Configuration ==="
    xrandr | head -10
    echo "============================="
fi

echo ""
echo "Done! Resolution set to ${RESOLUTION}@${REFRESH_RATE}"
echo "This configuration will persist across reboots."
echo ""
echo "IMPORTANT: You may also want to run set_vnc_resolution.sh to enable 1920x1080 VNC."
