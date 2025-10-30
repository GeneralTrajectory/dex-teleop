#!/bin/bash
# Setup script for Quest hand tracking teleoperation
# Sets all environment variables for optimal real-time performance

echo "🔧 Setting up Quest hand tracking teleoperation environment..."

# Performance settings (from INSPIRE_TELEOP_LESSONS.md)
export QUEST_RATE_HZ=60
export QUEST_USE_MEDIAN=0          # disable for max speed (default is 1)
export QUEST_EMA_BYPASS=8          # bypass filter for Δ≥8 counts
export QUEST_EMA_SLOW=0.5          # smooth only micro-jitter
export QUEST_QUANT=4               # quantize to ±4 counts
export INSPIRE_SPEED=1000          # max actuator speed

# Calibrated ranges (from user's Quest data)
export QUEST_CURL_MIN=0.2
export QUEST_CURL_MAX=1.0
export QUEST_TFLEX_MIN=30.0
export QUEST_TFLEX_MAX=100.0
export QUEST_TOPP_MIN=90.0
export QUEST_TOPP_MAX=100.0
export QUEST_TOPP_INVERT=1         # invert opposition direction

echo "✅ Environment variables set:"
echo "   QUEST_RATE_HZ=$QUEST_RATE_HZ"
echo "   QUEST_USE_MEDIAN=$QUEST_USE_MEDIAN (CRITICAL: must be 0 for real-time)"
echo "   QUEST_EMA_BYPASS=$QUEST_EMA_BYPASS"
echo "   INSPIRE_SPEED=$INSPIRE_SPEED"
echo ""
echo "Now run: python quest_inspire_teleop.py"
echo ""
echo "Or source this file in your current shell:"
echo "  source setup_quest_teleop.sh"

