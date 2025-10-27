#!/bin/bash
# Convenience launcher for Vive Tracker teleoperation
# Checks prerequisites and launches teleoperation with recommended settings

echo "============================================================"
echo "Vive Tracker Teleoperation Launcher"
echo "============================================================"

# Check SteamVR is running
if ! pgrep -f vrcompositor > /dev/null; then
    echo "❌ SteamVR is not running!"
    echo "   Start SteamVR first: steam → Click VR button"
    exit 1
fi
echo "✅ SteamVR is running"

# Set environment variables with safe defaults
export XARM_IP="${XARM_IP:-192.168.1.214}"
export VIVE_POSITION_SCALE="${VIVE_POSITION_SCALE:-1.5}"
export VIVE_ROTATION_SCALE="${VIVE_ROTATION_SCALE:-1.0}"
export TELEOP_RATE_HZ="${TELEOP_RATE_HZ:-30}"
export TELEOP_SPEED="${TELEOP_SPEED:-200}"

# Safety settings (from xarm_adapter.py defaults)
export TABLE_Z_MM="${TABLE_Z_MM:-15}"
export GRIPPER_TIP_LENGTH_MM="${GRIPPER_TIP_LENGTH_MM:-200}"
export TABLE_CLEARANCE_MM="${TABLE_CLEARANCE_MM:-10}"
export GRIPPER_RADIUS_MM="${GRIPPER_RADIUS_MM:-50}"
export GRIPPER_TIP_RADIUS_MM="${GRIPPER_TIP_RADIUS_MM:-18}"
export GRIPPER_TIP_AXIS="${GRIPPER_TIP_AXIS:-x}"
export GRIPPER_TIP_AXIS_SIGN="${GRIPPER_TIP_AXIS_SIGN:-1}"

echo ""
echo "Configuration:"
echo "  xArm IP: $XARM_IP"
echo "  Position scale: $VIVE_POSITION_SCALE"
echo "  Rotation scale: $VIVE_ROTATION_SCALE"
echo "  Control rate: $TELEOP_RATE_HZ Hz"
echo "  Motion speed: $TELEOP_SPEED mm/s"
echo ""

# Ask for confirmation
read -p "Start teleoperation? [y/N] " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Cancelled by user"
    exit 0
fi

# Run teleoperation
cd "$(dirname "$0")"
python3 vive_teleop_xarm.py

# Cleanup message
echo ""
echo "============================================================"
echo "Teleoperation session ended"
echo "============================================================"

