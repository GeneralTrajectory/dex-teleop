# Quick Start Guide: Vive Tracker Teleoperation

## Physical Setup (5 minutes)

### Step 1: Power On Equipment
```
1. Plug in both Vive base stations → Green LEDs should appear
2. Turn on Vive Tracker (hold power button) → Blue LED should appear
3. Plug tracker USB dongle into computer
4. Power on xArm robot
```

### Step 2: Start Software
```bash
# Terminal 1: Start SteamVR (must stay running)
steam

# In Steam UI: Click VR button in top-right corner
# SteamVR window should show green icons for base stations and tracker
```

### Step 3: Position Robot
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

## Run Teleoperation (2 minutes)

### Test First (Recommended)
```bash
cd /home/joshua/Research/dex-teleop
python test_vive_teleop.py
```

All 4 tests should pass before proceeding.

### Start Teleoperation
```bash
# With default settings
export XARM_IP="192.168.1.214"
python vive_teleop_xarm.py
```

### What You'll See:
```
Initializing ViveToXArmMapper...
  xArm IP: 192.168.1.214
  Position scale: 1.5
  Rotation scale: 1.0

📡 Initializing Vive Tracker...
Found 1 Tracker
  tracker_1 (LHR-A56A8A21, VIVE Tracker 3.0 MV)
✅ Using first detected tracker: LHR-A56A8A21

🤖 Connecting to xArm at 192.168.1.214...
✅ xArm connected successfully

🏠 Moving arm to home position...
✅ Arm at home position

🎯 Calibrating home poses (3s countdown)...
   Hold tracker steady in your starting position!
   3...
   2...
   1...

✅ Tracker home captured:
   Position (m): [0.123, 0.456, 0.789]
✅ Arm home captured:
   Position (mm): [275.0, 0.0, 670.0]

🎮 Teleoperation active at 30 Hz
   Press Ctrl+C to stop
------------------------------------------------------------
✅ Pos: [275.0, 0.0, 670.0] mm | Ori: [180.0, -1.0, 0.0]° | Violations: 0
```

## Controls

- **Move tracker left/right** → Arm moves left/right (Y-axis)
- **Move tracker forward/backward** → Arm moves forward/backward (X-axis)
- **Move tracker up/down** → Arm moves up/down (Z-axis)
- **Rotate tracker** → Arm orientation changes

Movements are scaled by `VIVE_POSITION_SCALE` (default: 1.5x).

## Safety Behavior

### Table Collision
If you move tracker downward too far:
```
🛑 Safety block: Table collision detected (gripper would hit table)
```
→ Arm stops moving, holds last safe position

### Behind Table
If you rotate tracker to point behind:
```
🛑 Safety block: Joint1 too positive (95.3° > 90.0°)
```
→ Arm stops, cannot rotate behind table

### Out of Workspace
If you move tracker too far:
```
🛑 Safety block: X too large (480.2 > 450.0mm)
```
→ Arm stops at workspace boundary

## Stopping

Press `Ctrl+C` to stop cleanly:
```
⚠️ Teleoperation stopped by user (Ctrl+C)
🛑 Shutting down teleoperation...
✅ Shutdown complete

📊 Session statistics:
   Total frames: 1234
   Safety violations: 5
   Success rate: 99.6%
```

## Tuning Tips

### Workspace feels cramped
```bash
# Increase position scale (more sensitive)
python vive_teleop_xarm.py --position-scale 2.0
```

### Movements too fast
```bash
# Reduce motion speed
python vive_teleop_xarm.py --speed 100
```

### Need finer control
```bash
# Reduce position scale (less sensitive)
python vive_teleop_xarm.py --position-scale 1.0
```

### Rotation feels wrong
```bash
# Reduce rotation scaling for more stable orientation
python vive_teleop_xarm.py --rotation-scale 0.5
```

## Next Steps

1. **Record demonstrations**: Add `--record` flag (future enhancement)
2. **Bimanual control**: Add second tracker for left arm (future)
3. **Gripper control**: Map tracker button to open/close (future)

## Troubleshooting

### "No Vive trackers detected"
- Ensure SteamVR is running
- Check tracker is powered on (blue LED)
- Re-pair tracker: SteamVR → Devices → Pair Controller

### "Failed to connect to xArm"
- Check xArm is powered on
- Verify IP address: `ping 192.168.1.214`
- Check network connection
- Ensure no other programs are connected to xArm

### "IK failed" during operation
- Target pose is unreachable
- Reduce position/rotation scale
- Stay within comfortable workspace region
- This is expected safety behavior at workspace edges

### Arm doesn't move smoothly
- Check control rate: higher = smoother (try `--rate 50`)
- Check motion speed: higher = faster but less smooth
- Ensure xArm controller has no errors/warnings
- Verify no other programs are connected to xArm

## Safety Checklist Before Each Session

- [ ] Clear workspace around robot (no obstacles)
- [ ] Table is secure and at calibrated height
- [ ] Emergency stop button is accessible
- [ ] SteamVR shows green icons (all devices connected)
- [ ] xArm is at home position before starting
- [ ] You understand how to stop (Ctrl+C or STOP button)

## Contact

For issues specific to:
- **Vive Tracker**: See `/home/joshua/Research/dex-teleop/Vive_Tracker/README.md`
- **xArm**: See `/home/joshua/Documents/Sources/Papers/Cursor/AnyDexGrasp/README.md`
- **This teleoperation system**: Check `README_TELEOP.md` for detailed documentation

