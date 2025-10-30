# Quick Start Guide - dex-teleop

Complete user guide for Vive Tracker (xArm) and Quest Hand Tracking (Inspire Hands) teleoperation.

---

## Table of Contents
1. [Vive Tracker → xArm](#vive-tracker--xarm)
2. [Bimanual xArm Control](#bimanual-xarm-control)
3. [Quest Hand Tracking → Inspire Hands](#quest-hand-tracking--inspire-hands)

---

## Vive Tracker → xArm

### Physical Setup (5 minutes)

#### Step 1: Power On Equipment
```
1. Plug in both Vive base stations → Green LEDs should appear
2. Turn on Vive Tracker (hold power button) → Blue LED should appear
3. Plug tracker USB dongle into computer
4. Power on xArm robot
```

#### Step 2: Start Software
```bash
# Terminal 1: Start SteamVR (must stay running)
steam

# In Steam UI: Click VR button in top-right corner
# SteamVR window should show green icons for base stations and tracker
```

#### Step 3: Position Robot
```bash
# Terminal 2: Move xArm to home position
cd /home/joshua/Documents/Sources/Papers/Cursor/AnyDexGrasp
python3 -c "
from xarm.wrapper import XArmAPI
arm = XArmAPI('192.168.1.214')
arm.motion_enable(True)
arm.set_mode(0)
arm.set_state(0)
arm.set_position(x=275, y=0, z=670, roll=180, pitch=-1, yaw=0, speed=300, wait=True)
print('✅ Arm at home position')
arm.disconnect()
"
```

### Run Teleoperation

#### Test First (Recommended)
```bash
cd /home/joshua/Research/dex-teleop
python test_vive_teleop.py
```

All 4 tests should pass before proceeding.

#### Start Teleoperation
```bash
# With default settings (right arm)
export XARM_IP_RIGHT="192.168.1.214"
python vive_teleop_xarm.py --mode right
```

### Controls
- **Move tracker left/right** → Arm moves left/right (Y-axis)
- **Move tracker forward/backward** → Arm moves forward/backward (X-axis)
- **Move tracker up/down** → Arm moves up/down (Z-axis)
- **Rotate tracker** → Arm orientation changes

Movements are scaled by `VIVE_POSITION_SCALE` (default: 1.0x).

### Safety Behavior

**Table Collision**: Arm stops if gripper would hit table  
**Behind Table**: Arm stops if rotating behind table (Joint1 limit)  
**Out of Workspace**: Arm stops at workspace boundary, smooth re-engagement when returning

### Stopping
Press `Ctrl+C` or foot pedal ('b' key) to stop cleanly.

### Basic Tuning

**Workspace feels cramped:**
```bash
python vive_teleop_xarm.py --position-scale 2.0
```

**Movements too fast:**
```bash
python vive_teleop_xarm.py --speed 100
```

**Rotation feels wrong:**
```bash
python vive_teleop_xarm.py --rotation-scale 0.5
```

---

## Bimanual xArm Control

### Hardware Requirements
- 2 Vive Trackers 3.0 (one per wrist)
- 2 xArm robots (left and right)
- 2 Vive Base Stations

### Tracker Assignment
Trackers are assigned by serial number (hardcoded in script):
- **Right arm**: `LHR-A56A8A21` (right wrist tracker)
- **Left arm**: `LHR-D51DBC11` (left wrist tracker)

### Quick Start

```bash
export XARM_IP_LEFT="192.168.1.111"
export XARM_IP_RIGHT="192.168.1.214"
python vive_teleop_xarm.py --mode both
```

### Calibration
Both arms calibrate **simultaneously** with a single 6-second countdown:
1. Hold **BOTH trackers** steady in starting positions
2. Both arms move to home automatically
3. 6-second countdown for calibration
4. Teleoperation active - move both arms independently

### Configuration

```bash
# Scaling (applies to both arms)
export VIVE_POSITION_SCALE="1.0"
export VIVE_ROTATION_SCALE="1.0"

# Performance
export TELEOP_RATE_HZ="100"
export TELEOP_REENGAGEMENT_STEPS="30"  # Smooth re-engagement

# Start
python vive_teleop_xarm.py --mode both
```

### Status Display
```
✅ [right] Pos: [350.0, 100.0, 250.0] mm | Ori: [180.0, 0.0, 0.0]° | Violations: 0
✅ [left]  Pos: [300.0, -150.0, 250.0] mm | Ori: [180.0, 0.0, 0.0]° | Violations: 0
```

### Troubleshooting

**Only one tracker detected**: Check both trackers are powered on and visible to base stations

**Trackers swapped**: Physically swap trackers on wrists, or swap IP addresses

---

## Quest Hand Tracking → Inspire Hands

### Prerequisites
1. **Meta Quest headset** with Unity hand tracking app running
2. **Two Inspire Hands** connected:
   - Left hand: `/dev/ttyUSB1`, slave_id 1
   - Right hand: `/dev/ttyUSB0`, slave_id 1
3. **Network connection** between Quest and computer (same WiFi or USB tethering)

### Quick Start

**⚠️ IMPORTANT**: First, set up environment for real-time performance:

```bash
cd /home/joshua/Research/dex-teleop
source setup_quest_teleop.sh  # Sets all required environment variables
python quest_inspire_teleop.py
```

Or set manually:
```bash
export QUEST_RATE_HZ=60
export QUEST_USE_MEDIAN=0          # CRITICAL: disable for real-time
export QUEST_EMA_BYPASS=8
export QUEST_EMA_SLOW=0.5
export QUEST_QUANT=4
export INSPIRE_SPEED=1000

# Calibrated ranges
export QUEST_CURL_MIN=0.2
export QUEST_CURL_MAX=1.0
export QUEST_TFLEX_MIN=30.0
export QUEST_TFLEX_MAX=100.0
export QUEST_TOPP_MIN=90.0
export QUEST_TOPP_MAX=100.0
export QUEST_TOPP_INVERT=1

python quest_inspire_teleop.py
```

### Expected Output
```
============================================================
🎮 Quest Hand Tracking → Inspire Hands
============================================================

📡 Quest hand receiver listening on UDP port 9000
🤖 Connecting to left Inspire Hand...
✅ Connected to left Inspire Hand (/dev/ttyUSB1)
🤖 Connecting to right Inspire Hand...
✅ Connected to right Inspire Hand (/dev/ttyUSB0)
🖐️ Opening all fingers...
🦶 Foot pedal enabled - press 'b' to stop

✅ Bimanual hand control active at 60 Hz
   Press 'b' (foot pedal) or Ctrl+C to stop
------------------------------------------------------------
[left ] L:0.2 R:0.5 M:0.8 I:1.2 T:1.0
[right] L:0.1 R:0.3 M:0.4 I:0.5 T:0.8
```

### How It Works

**Data Flow:**
```
Quest Headset (Unity App, 90 Hz)
    ↓ UDP packets (port 9000)
QuestHandReceiver (non-blocking)
    ↓ Parse joint rotations
HandTrackingMapper
    ↓ Map curls → angles [0-1000]
Per-Hand Async Sender Threads
    ↓ Modbus RTU commands
Inspire Hands (physical)
```

**Finger Mappings:**
- Little, Ring, Middle, Index: Curl (0.2-1.0) → Angle (1000-0)
- Thumb bend: Flexion (30-100°) → Angle (1000-0)
- Thumb rotate: Opposition (90-100°) → Angle (0-1000, inverted)

### Gesture Examples
- **Open Hand**: Fingers extended → Inspire angles ~1000
- **Closed Fist**: Fingers curled → Inspire angles ~0
- **Pinch**: Thumb + index curled → Inspire thumb + index ~0, others ~1000
- **Point**: Index extended, others curled → Inspire index ~1000, others ~0

### Troubleshooting

**No UDP data received:**
- Verify Quest app is running and streaming
- Check network: `ping <quest-ip>`
- Verify port 9000 is not blocked

**Hands don't respond:**
- Check USB: `ls -l /dev/ttyUSB*`
- Test with CLI: `python -m inspire_hand --port /dev/ttyUSB0 --slave-id 1 open all`

**Jittery/slow control:**
- **CRITICAL**: Ensure `QUEST_USE_MEDIAN=0` (median filter adds delay)
- Check environment variables are set (use `setup_quest_teleop.sh`)
- See [TECHNICAL.md](TECHNICAL.md#quest-performance-tuning) for detailed tuning

### Testing UDP Reception
```bash
cd /home/joshua/Research/dex-teleop
python recv_rotations.py
```

Should show:
```
listening on UDP *:9000
left  joints=21 tracked=21/21 curls={'Thumb': 0.5, 'Index': 1.2, ...}
right joints=21 tracked=21/21 dilation={'Thumb': 0.3, 'Index': 0.8, ...}
```

---

## Troubleshooting (All Systems)

### "No Vive trackers detected"
- Ensure SteamVR is running
- Check tracker is powered on (blue LED)
- Re-pair tracker: SteamVR → Devices → Pair Controller

### "Failed to connect to xArm"
- Check xArm is powered on
- Verify IP address: `ping 192.168.1.214`
- Ensure no other programs are connected to xArm

### "IK failed" during operation
- Target pose is unreachable
- Reduce position/rotation scale
- Stay within comfortable workspace region

### General Performance Issues
- See [TECHNICAL.md](TECHNICAL.md) for detailed tuning guides
- Check environment variables are set correctly
- Verify hardware connections

---

## Safety Checklist (Before Each Session)

- [ ] Clear workspace around robot (no obstacles)
- [ ] Table is secure and at calibrated height
" ] Emergency stop button is accessible
- [ ] SteamVR shows green icons (all devices connected, if using Vive)
- [ ] xArm is at home position before starting (if using xArm)
- [ ] Inspire Hands are powered and connected (if using Quest)
- [ ] You understand how to stop (Ctrl+C or foot pedal)

---

---

## Recording Demonstrations

### Prerequisites
Install h5py for data recording:
```bash
pip install h5py
```

### Record a Demonstration

**Setup environment:**
```bash
cd /home/joshua/Research/dex-teleop
source setup_recorder.sh
```

**Start recording:**
```bash
python teleop_recorder.py record --subject S1 --task pick --notes "red cube"
```

**Workflow:**
1. System initializes all devices (Vive, Quest, xArms, Inspire Hands)
2. Press 'b' (foot pedal) → Arms home, then recording starts
3. Perform demonstration (teleoperate both arms + hands)
4. Press 'b' again → Recording stops and saves

**Ephemeral mode (testing, no save):**
```bash
python teleop_recorder.py record --ephemeral
```

### List Recordings
```bash
python teleop_recorder.py list
```

### Validate Recording
```bash
python teleop_recorder.py validate --trial-id 20250130_143022
```

### Replay Recording

**Dry-run (no robot motion, just stats):**
```bash
python teleop_recorder.py replay --trial-id 20250130_143022 --dry-run
```

**Live replay:**
```bash
python teleop_recorder.py replay --trial-id 20250130_143022 --speed 0.5
```

---

## Next Steps

- **Advanced configuration**: See [TECHNICAL.md](TECHNICAL.md)
- **Performance optimization**: See [TECHNICAL.md](TECHNICAL.md#performance-tuning)
- **Troubleshooting Quest issues**: See [INSPIRE_TELEOP_LESSONS.md](INSPIRE_TELEOP_LESSONS.md)
