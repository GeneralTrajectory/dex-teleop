# Bimanual Teleoperation Usage Guide

## Overview
The teleoperation system now supports three modes:
- **Right arm only** (default)
- **Left arm only**
- **Bimanual** (both arms simultaneously)

## Hardware Requirements

### Unimanual (left or right)
- 1 Vive Tracker 3.0
- 1 xArm robot
- 2 Vive Base Stations

### Bimanual
- 2 Vive Trackers 3.0 (one per wrist)
- 2 xArm robots (left and right)
- 2 Vive Base Stations

## Quick Start

### Right Arm Only (Default)
```bash
export XARM_IP_RIGHT="192.168.1.214"
python vive_teleop_xarm.py --mode right
```

### Left Arm Only
```bash
export XARM_IP_LEFT="192.168.1.111"
python vive_teleop_xarm.py --mode left
```

### Bimanual (Both Arms)
```bash
export XARM_IP_LEFT="192.168.1.111"
export XARM_IP_RIGHT="192.168.1.214"
python vive_teleop_xarm.py --mode both
```

## Tracker Assignment

### Hardcoded Serial Numbers
The system uses specific tracker serial numbers for each arm:
- **Right arm**: `LHR-A56A8A21` (right wrist tracker)
- **Left arm**: `LHR-D51DBC11` (left wrist tracker)

These are hardcoded in the script to ensure consistent mapping. If you need to change them, edit the constants in `vive_teleop_xarm.py`:
```python
RIGHT_TRACKER_SERIAL = 'LHR-A56A8A21'  # Right wrist
LEFT_TRACKER_SERIAL = 'LHR-D51DBC11'   # Left wrist
```

### Verification
When you run the script, you'll see:
```bash
python vive_teleop_xarm.py --mode both
```

Output will confirm tracker assignment:
```
📡 Initializing Vive Tracker system...
✅ Found 2 tracker(s)
✅ Using left-hand tracker: LHR-D51DBC11
✅ Using right-hand tracker: LHR-A56A8A21

🤖 Initializing left arm mapper...
  Tracker: LHR-D51DBC11

🤖 Initializing right arm mapper...
  Tracker: LHR-A56A8A21
```

## Configuration Options

### Environment Variables
```bash
# Network
export XARM_IP_LEFT="192.168.1.111"
export XARM_IP_RIGHT="192.168.1.214"

# Scaling
export VIVE_POSITION_SCALE="1.0"      # Position sensitivity
export VIVE_ROTATION_SCALE="1.0"      # Global rotation sensitivity
export VIVE_ROTATION_SCALE_ROLL="1.0"
export VIVE_ROTATION_SCALE_PITCH="2.0"  # Increase pitch sensitivity
export VIVE_ROTATION_SCALE_YAW="-1.0"   # Flip yaw direction

# Performance
export TELEOP_RATE_HZ="100"           # Control loop rate
export TELEOP_REENGAGEMENT_STEPS="30"  # Smooth re-engagement frames

# Safety
export TABLE_Z_MM="15"                # Table height in mm
```

### Command-Line Arguments
```bash
python vive_teleop_xarm.py \
  --mode both \
  --left-ip 192.168.1.111 \
  --right-ip 192.168.1.214 \
  --position-scale 1.0 \
  --rotation-scale 1.0 \
  --rate 100 \
  --speed 100
```

## Safety Features

### Per-Arm Safety
Each arm has independent safety monitoring:
- Workspace boundary enforcement
- IK reachability validation
- Table collision prevention
- Joint1 constraint enforcement (-90° to +90°)
- Smooth re-engagement after violations

### Emergency Stop
- **Ctrl+C**: Stop all arms immediately
- **Foot Pedal ('b' key)**: Stop all arms immediately

### Status Display
```
✅ [right] Pos: [350.0, 100.0, 250.0] mm | Ori: [180.0, 0.0, 0.0]° | Violations: left:0, right:2
```

## Calibration Workflow

### Bimanual Calibration
1. **Start script**: `python vive_teleop_xarm.py --mode both`
2. **Arms move to home**: Both arms move to home position sequentially
3. **Simultaneous calibration**: Single 6-second countdown
   - Hold **BOTH** trackers steady in starting positions
   - Both arms calibrate at the same time
4. **Teleoperation active**: Move both arms independently

### Unimanual Calibration
1. **Start script**: `python vive_teleop_xarm.py --mode left` (or `right`)
2. **Arm moves to home**: Single arm moves to home position
3. **Single arm calibration**: 6-second countdown
   - Hold tracker steady in starting position
4. **Teleoperation active**: Move arm

### Calibration Tips
- Stand in a comfortable position
- Arms should be in a natural, relaxed pose
- Trackers should be stable (no movement during countdown)
- **Bimanual mode**: Both trackers calibrate simultaneously with one countdown
- **Unimanual mode**: Single tracker calibrates independently

## Troubleshooting

### Only One Tracker Detected
```
❌ Bimanual mode requires 2 trackers, but only 1 found
```
**Solution**: 
- Check both trackers are powered on (green LED)
- Ensure both trackers are visible to base stations
- Try re-pairing trackers in SteamVR

### Arm Stops Unexpectedly
```
🛑 [left] Workspace limit: X too large (750.0 > 700.0mm)
```
**Solution**:
- Move tracker back into workspace
- Arm will automatically re-engage smoothly
- Adjust workspace limits if needed

### Connection Failed
```
❌ Failed to connect to xArm at 192.168.1.111
```
**Solution**:
- Verify IP addresses are correct
- Check network connectivity: `ping 192.168.1.111`
- Ensure xArm is powered on and ready

### Trackers Swapped
If left/right trackers are reversed:
- Physically swap the trackers on your wrists, OR
- Swap the IP addresses in environment variables

## Performance Tuning

### Reduce Sensitivity
```bash
export VIVE_POSITION_SCALE="0.5"  # Half sensitivity
python vive_teleop_xarm.py --mode both
```

### Increase Control Rate
```bash
python vive_teleop_xarm.py --mode both --rate 150
```

### Smoother Re-engagement
```bash
export TELEOP_REENGAGEMENT_STEPS="50"  # 0.5s at 100Hz
python vive_teleop_xarm.py --mode both
```

## Session Statistics

At the end of each session:
```
📊 Session statistics:
   Total frames: 12450
   left arm violations: 3
   right arm violations: 5
```

## Best Practices

1. **Calibrate carefully**: Take time during the 6-second countdown
2. **Start slow**: Begin with small movements to get comfortable
3. **Monitor status**: Watch for workspace violations
4. **Use foot pedal**: Keep foot pedal accessible for emergency stops
5. **Independent control**: Each arm operates independently - one can stop while the other continues
6. **Smooth movements**: Avoid jerky motions to prevent safety violations

## Advanced Usage

### Per-Arm Scaling (Future Enhancement)
Currently, both arms use the same scaling factors. To implement per-arm scaling, modify the environment variables before starting each mapper.

### Custom Workspace Limits
Edit the `workspace_limits` in `ViveToXArmMapper.__init__()` for custom boundaries per arm.

### Logging
Redirect output to log file:
```bash
python vive_teleop_xarm.py --mode both 2>&1 | tee teleop_session.log
```

