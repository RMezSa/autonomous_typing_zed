#!/bin/bash
# Universal camera optimization script for ArUco detection
# Auto-detects camera capabilities and applies best settings

DEVICE="${1:-/dev/video0}"

echo "=========================================="
echo "Universal Camera Optimizer for ArUco"
echo "=========================================="

# Check if device exists
if [ ! -e "$DEVICE" ]; then
    echo "ERROR: Camera device $DEVICE not found"
    exit 1
fi

# Get camera info
CAM_NAME=$(v4l2-ctl -d $DEVICE --info | grep "Card type" | cut -d: -f2 | xargs)
echo "Detected: $CAM_NAME"
echo ""

# Function to safely set control
set_control() {
    local ctrl=$1
    local value=$2
    
    if v4l2-ctl -d $DEVICE --list-ctrls | grep -q "$ctrl"; then
        if v4l2-ctl -d $DEVICE --set-ctrl=$ctrl=$value 2>/dev/null; then
            echo "✓ Set $ctrl = $value"
            return 0
        else
            echo "✗ Failed to set $ctrl (may be inactive)"
            return 1
        fi
    else
        echo "⊘ Control $ctrl not available"
        return 2
    fi
}

echo "Applying optimizations..."
echo ""

# Disable auto exposure (critical for stable detection)
echo "--- Exposure Settings ---"
set_control "auto_exposure" "1"  # Manual mode
sleep 0.2
set_control "exposure_absolute" "156"
set_control "exposure_dynamic_framerate" "0"

# Disable auto focus (critical for sharp markers)
echo ""
echo "--- Focus Settings ---"
set_control "focus_automatic_continuous" "0"
sleep 0.2
set_control "focus_absolute" "20"

# Image quality optimizations
echo ""
echo "--- Image Quality ---"
set_control "sharpness" "200"
set_control "contrast" "150"
set_control "saturation" "128"
set_control "brightness" "128"

# White balance
echo ""
echo "--- White Balance ---"
set_control "white_balance_automatic" "0"
sleep 0.2
set_control "white_balance_temperature" "4500"

# Additional optimizations
echo ""
echo "--- Additional ---"
set_control "backlight_compensation" "1"
set_control "gain" "64"
set_control "power_line_frequency" "2"  # 60Hz

echo ""
echo "=========================================="
echo "Camera optimization complete!"
echo "=========================================="
echo ""
echo "Current settings:"
v4l2-ctl -d $DEVICE --list-ctrls-menus | grep -E "(exposure|focus|sharpness|contrast)" | head -10
