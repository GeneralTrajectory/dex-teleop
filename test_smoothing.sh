#!/bin/bash
# Test script for smoothing filter with left arm

echo "Testing smoothing filter with left arm..."
echo ""
echo "Configuration:"
echo "  - Smoothing enabled with moderate settings"
echo "  - Alpha: 0.3 (balanced)"
echo "  - Max position delta: 50mm/frame"
echo "  - Max rotation delta: 10°/frame"
echo ""
echo "Watch for outlier rejection messages during operation."
echo "Press 'b' (foot pedal) or Ctrl+C to stop."
echo ""

export XARM_IP_LEFT="192.168.1.111"
export VIVE_SMOOTHING=1
export VIVE_SMOOTHING_ALPHA=0.3
export VIVE_MAX_POSITION_DELTA_MM=50.0
export VIVE_MAX_ROTATION_DELTA_DEG=10.0

python vive_teleop_xarm.py --mode left
