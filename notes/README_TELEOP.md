# Vive Tracker Teleoperation for xArm

Safe, research-grade teleoperation system for controlling xArm robots using HTC Vive Tracker 3.0.

## Features

- **Relative offset control**: Natural hand movements map to robot movements with configurable scaling
- **Comprehensive safety**:
  - Table collision prevention (accounts for 200mm gripper extension)
  - Joint limit enforcement (Joint1: -90° to +90°)
  - Workspace boundary constraints
  - Multi-layer validation before every motion command
- **Smooth control**: 30Hz control loop with rate limiting
- **Easy calibration**: 3-second auto-calibration on startup

## Quick Start

### 1. Hardware Setup

1. Power on both Vive base stations (green LEDs)
2. Power on Vive Tracker 3.0 (hold button until LED is on)
3. Ensure tracker USB dongle is connected to computer
4. Start SteamVR (must be running in background)
5. Position xArm at home pose: `arm.set_position(x=275, y=0, z=670, roll=180, pitch=-1, yaw=0)`

### 2. Test Connectivity

Run the test suite to verify everything is working:

```bash
cd /home/joshua/Research/dex-teleop
python test_vive_teleop.py
```

Expected output:
```
✅ PASS: Tracker Detection
✅ PASS: Pose Stream
✅ PASS: Coordinate Transform
✅ PASS: Xarm Connection
```

### 3. Run Teleoperation

```bash
# Basic usage (right arm, default settings)
export XARM_IP="192.168.1.214"
python vive_teleop_xarm.py

# Custom scaling
python vive_teleop_xarm.py --position-scale 2.0 --rotation-scale 0.5

# Custom control rate and speed
python vive_teleop_xarm.py --rate 20 --speed 150
```

### 4. Operation

1. Script will move arm to home position automatically
2. Wait for 3-second countdown while holding tracker steady
3. After "Teleoperation active" message appears:
   - Move tracker to control arm position
   - Rotate tracker to control arm orientation
   - Movements are relative to your starting position
4. Press `Ctrl+C` to stop (arm will stop safely)

## Configuration

### Environment Variables

```bash
# Tracker settings
export VIVE_TRACKER_SERIAL=""        # Auto-detect if empty
export VIVE_POSITION_SCALE="1.5"     # Scale factor (1.5 = tracker moves 1m → arm moves 1.5m)
export VIVE_ROTATION_SCALE="1.0"     # Rotation scaling (1.0 = 1:1 mapping)

# xArm settings
export XARM_IP="192.168.1.214"       # Right arm IP (left: 192.168.1.111)
export TELEOP_RATE_HZ="30"           # Control loop frequency (Hz)
export TELEOP_SPEED="200"            # Motion speed (mm/s)

# Safety settings (reuse from xarm_adapter.py)
export TABLE_Z_MM="15"               # Table surface height in base frame
export GRIPPER_TIP_LENGTH_MM="200"   # Inspire gripper extension
export TABLE_CLEARANCE_MM="10"       # Minimum clearance above table
```

### Command Line Arguments

```bash
python vive_teleop_xarm.py --help

Options:
  --xarm-ip IP          xArm IP address (default: 192.168.1.214)
  --position-scale F    Position scaling factor (default: 1.5)
  --rotation-scale F    Rotation scaling factor (default: 1.0)
  --rate HZ            Control loop rate in Hz (default: 30)
  --speed MM_S         Motion speed in mm/s (default: 200)
```

## Safety Features

### 1. Table Collision Prevention

The system automatically prevents the gripper from colliding with the table:
- Uses existing collision detection from `xarm_adapter.py`
- Accounts for 200mm gripper extension
- Samples multiple points along gripper body
- Blocks any command that would cause collision

**What happens**: If you try to move below the table, motion is blocked and held at last safe position.

### 2. Joint Limit Enforcement

Joint1 (base rotation) is constrained to [-90°, +90°] to prevent reaching behind the table:
- IK solution is computed for every target pose
- Joint1 angle is checked before motion
- Poses requiring rotation behind table are rejected

**What happens**: If you try to rotate behind the table, motion is blocked.

### 3. Workspace Boundaries

Conservative workspace limits based on xArm 6 specifications:
- **X**: 100mm to 450mm (forward reach only)
- **Y**: -400mm to +400mm (symmetric left/right)
- **Z**: TABLE_Z + 50mm to 800mm (above table, below shoulder)

**What happens**: Commands outside workspace are rejected, arm holds at boundary.

### 4. Emergency Stop

- **Software**: Press `Ctrl+C` for clean shutdown
- **Hardware**: xArm GUI STOP button always works (independent of software)

## Troubleshooting

### Tracker not detected

```bash
# Check SteamVR is running
ps aux | grep vrcompositor

# Check tracker is paired and showing green in SteamVR interface
# Re-pair if needed: SteamVR → Devices → Pair Controller
```

### "Safety block: Table collision"

- Tracker position is below table height
- Move tracker upward to continue
- Adjust `VIVE_POSITION_SCALE` if workspace mapping feels wrong

### "Safety block: Joint1 too positive/negative"

- Tracker orientation would require arm to rotate behind table
- Rotate tracker back toward neutral position
- This is expected safety behavior

### Motion is jerky or slow

```bash
# Increase control rate (but may stress xArm controller)
python vive_teleop_xarm.py --rate 50

# Increase motion speed
python vive_teleop_xarm.py --speed 300
```

### Workspace feels too small/large

Adjust position scaling:

```bash
# Larger workspace (tracker 1m → arm 2m)
python vive_teleop_xarm.py --position-scale 2.0

# Smaller workspace (tracker 1m → arm 1m)
python vive_teleop_xarm.py --position-scale 1.0
```

## Architecture

### Control Flow

```
Vive Tracker → ViveTrackerModule.get_T() → 4x4 pose matrix
                                               ↓
                                    ViveToXArmMapper.compute_target_pose()
                                               ↓
                                    Delta computation + scaling
                                               ↓
                                    Target pose {x,y,z,roll,pitch,yaw}
                                               ↓
                                    ViveToXArmMapper.is_pose_safe()
                                               ↓
                              ┌─────────────────┴─────────────────┐
                              ↓                                   ↓
                          [SAFE]                              [UNSAFE]
                              ↓                                   ↓
                  XArmAdapter._move_position()          Hold at last safe pose
                              ↓                           + log violation
                         xArm motion
```

### Coordinate Frames

- **Tracker frame**: SteamVR base station coordinate system
- **Tracker home**: Captured at calibration (t=3s)
- **xArm base frame**: Robot base coordinate system (fixed)
- **xArm home**: Standard HOME_POSE {x:275, y:0, z:670, roll:180, pitch:-1, yaw:0}

Delta calculation:
```
tracker_delta = current_tracker_pose - tracker_home
scaled_delta = tracker_delta * position_scale
target_pose = arm_home + scaled_delta
```

## Files

- `vive_teleop_xarm.py` - Main teleoperation script
- `test_vive_teleop.py` - Connectivity and mapping tests
- `Vive_Tracker/` - Vive Tracker interface library
- `../Documents/Sources/Papers/Cursor/AnyDexGrasp/xarm_adapter.py` - xArm safety wrapper

## Future Enhancements

- [ ] Data recording for imitation learning (save trajectory + camera frames)
- [ ] Bimanual control (second tracker → left arm)
- [ ] Gripper teleoperation (tracker button → open/close)
- [ ] Visual feedback overlay (camera view + arm state)
- [ ] Force feedback (haptic feedback on collisions)
- [ ] Workspace visualization (Open3D real-time view)

## Safety Notes

⚠️ **IMPORTANT**: This is a research system controlling expensive hardware. Always:
- Keep emergency stop button accessible
- Start with conservative scaling factors (1.0-1.5)
- Test workspace boundaries before demonstrations
- Monitor for unexpected behavior and stop immediately
- Never disable safety checks unless you fully understand implications
- Ensure adequate clearance around robot workspace

## Citation

If you use this teleoperation system in research, please cite:
- HTC Vive Tracker: https://github.com/snuvclab/Vive_Tracker
- xArm SDK: https://github.com/xArm-Developer/xArm-Python-SDK

