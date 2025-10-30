# Technical Documentation - dex-teleop

Architecture, configuration, and performance tuning guides.

---

## Architecture

### Vive Tracker → xArm Control Flow

```
Vive Tracker → get_T() → 4x4 pose matrix
                         ↓
              compute_target_pose()
                         ↓
              Delta computation + scaling
                         ↓
              Target pose {x,y,z,roll,pitch,yaw}
                         ↓
              is_pose_safe() [3-layer checks]
                         ↓
         ┌───────────────┴───────────────┐
         ↓                               ↓
     [SAFE]                          [UNSAFE]
         ↓                               ↓
   set_servo_cartesian()         Hold last safe pose
         ↓                         + log violation
    xArm motion
```

### Control Mode: Relative Offset Control

**Why this design?**
- Standard in research teleoperation (ALOHA, DROID datasets)
- Natural for human demonstrations
- No drift from velocity integration
- Allows workspace scaling (human arm reach ≠ robot reach)

**How it works:**
1. **Calibration** (6 seconds): Capture tracker_home and arm_home poses
2. **Control loop** (100 Hz): 
   - Read current_tracker pose
   - Compute delta: `Δ = current - tracker_home`
   - Scale delta: `Δ_scaled = Δ × scale_factor`
   - Compute target: `target = arm_home + Δ_scaled`
   - Validate safety
   - Send to xArm if safe

### Safety Architecture (Three-Layer System)

#### Layer 1: Workspace Boundaries
Fast pre-check before expensive IK computations.
- X: [100, 450] mm (forward reach only)
- Y: [-400, 400] mm (symmetric left/right)
- Z: [TABLE_Z+50, 800] mm (above table, below shoulder)
- **Checked every frame** (< 1ms)

#### Layer 2: Table Collision Detection
Prevents gripper from hitting table.
- Multi-point sampling along 200mm gripper cylinder
- Accounts for gripper orientation
- **Checked every 5 frames** (~2-3ms)

#### Layer 3: Joint Limit Enforcement
Prevents reaching behind table (Joint1: -90° to +90°).
- Uses xArm IK solver
- Validates joint angles
- **Checked every 5 frames** (~10-12ms)

### Quest Hand Tracking → Inspire Hands Architecture

```
Quest Headset (90 Hz)
    ↓ UDP (port 9000, non-blocking)
Background UDP Drainer Thread
    ↓ Latest packet per hand
Main Control Loop (60 Hz)
    ↓ Map curl → angles (calibrated ranges)
    ↓ Bypass filter (Δ≥8) OR EMA (Δ<8, α=0.5)
    ↓ Quantize (±4 counts)
Per-Hand Async Sender Threads (parallel)
    ↓ Modbus RTU (optimized polling, <5ms)
Inspire Hands (physical)
```

---

## Configuration

### Vive Tracker Environment Variables

```bash
# Network
export XARM_IP_LEFT="192.168.1.111"     # Left arm
export XARM_IP_RIGHT="192.168.1.214"    # Right arm (default)

# Scaling
export VIVE_POSITION_SCALE="1.0"        # Position sensitivity
export VIVE_ROTATION_SCALE="1.0"        # Global rotation
export VIVE_ROTATION_SCALE_ROLL="1.0"   # Roll sensitivity
export VIVE_ROTATION_SCALE_PITCH="2.0"  # Pitch sensitivity
export VIVE_ROTATION_SCALE_YAW="-1.0"   # Yaw direction (flip if needed)

# Performance
export TELEOP_RATE_HZ="100"             # Control loop rate
export TELEOP_SPEED="100"               # Motion speed (reserved in servo mode)
export TELEOP_REENGAGEMENT_STEPS="30"   # Smooth re-engagement frames

# Safety
export TABLE_Z_MM="15"                  # Table height
```

### Quest Hand Tracking Environment Variables

**CRITICAL**: Use `setup_quest_teleop.sh` to set these, or set manually:

```bash
# Performance (for real-time control)
export QUEST_RATE_HZ=60
export QUEST_USE_MEDIAN=0               # MUST be 0 for real-time (default is 1)
export QUEST_EMA_BYPASS=8               # Bypass threshold (counts)
export QUEST_EMA_SLOW=0.5               # EMA alpha for small changes
export QUEST_QUANT=4                    # Quantization (suppress micro-jitter)
export INSPIRE_SPEED=1000               # Max actuator speed

# Calibrated ranges (from Quest data)
export QUEST_CURL_MIN=0.2               # Finger curl min (open)
export QUEST_CURL_MAX=1.0               # Finger curl max (closed)
export QUEST_TFLEX_MIN=30.0             # Thumb flexion min (open)
export QUEST_TFLEX_MAX=100.0            # Thumb flexion max (closed)
export QUEST_TOPP_MIN=90.0              # Thumb opposition min (extended)
export QUEST_TOPP_MAX=100.0             # Thumb opposition max (opposed)
export QUEST_TOPP_INVERT=1              # Invert opposition direction
```

---

## Performance Tuning

### Vive Tracker Performance

#### Control Rate
- **30 Hz**: Standard, balanced latency and smoothness
- **50 Hz**: High performance, lower latency (may stress xArm)
- **100 Hz**: Servo mode streaming (default, recommended)

#### Smooth Re-Engagement

When returning to workspace after violation:
- **30 steps** (default): 0.3s at 100Hz, balanced
- **50 steps**: 0.5s, extra smooth for safety
- **20 steps**: 0.2s, faster response

```bash
export TELEOP_REENGAGEMENT_STEPS="潜在的"  # Adjust as needed
```

Uses smoothstep easing (S-curve) for gradual acceleration/deceleration.

#### Latency Optimization

Default configuration achieves ~12-15ms latency:
- Workspace check every frame (< 1ms)
- Full safety check every 5 frames (~3ms avg)
- Direct servo mode streaming (~5ms)

### Quest Performance Tuning

#### ⚠️ Critical: Real-Time Setup

**The median filter is enabled by default**, which adds 2-3 frame delay. **You must disable it**:

```bash
export QUEST_USE_MEDIAN=0  # CRITICAL for real-time
```

#### Responsiveness vs Smoothness Trade-off

**Maximum responsiveness** (use `setup_quest_teleop.sh`):
```bash
export QUEST_RATE_HZ=60
export QUEST_USE_MEDIAN=0
export QUEST_EMA_BYPASS=8      # Large changes pass through instantly
export QUEST_EMA_SLOW=0.5      # Only smooth micro-jitter
export QUEST_QUANT=4
export INSPIRE_SPEED=1000
```

**More smoothing** (if jittery):
```bash
export QUEST_EMA_BYPASS=4      # Lower threshold = more smoothing
export QUEST_EMA_SLOW=0.3      # Stronger smoothing
export QUEST_USE_MEDIAN=1      # Enable median filter (adds delay)
export QUEST_MEDIAN_WIN=3
```

**Less smoothing** (if laggy):
```bash
export QUEST_EMA_BYPASS=12     # Higher threshold = less smoothing
export QUEST_EMA_SLOW=0.7      # Lighter smoothing
export INSPIRE_SPEED=1200      # Higher speed
```

#### Bypass-Based Adaptive Filtering

The EMA filter uses a bypass mechanism:
- **Changes ≥ bypass_threshold**: Pass through instantly (zero lag)
- **Changes < bypass_threshold**: Apply EMA smoothing (reduces jitter)

This gives instant response for gestures while filtering micro-tremor.

---

## Troubleshooting

### Vive Tracker Issues

**"IK failed" during operation:**
- Target pose is unreachable
- Reduce position/rotation scale
- Stay within comfortable workspace region

**Arm doesn't move smoothly:**
- Increase control rate: `--rate 50`
- Ensure xArm controller has no errors
- Verify no other programs connected to xArm

**Re-engagement feels jerky:**
- Increase `TELEOP_REENGAGEMENT_STEPS` to 50
- Uses smoothstep easing (not linear)

### Quest Hand Tracking Issues

**Hands are slow/laggy:**
1. **Check environment variables**: Use `setup_quest_teleop.sh`
2. **Verify `QUEST_USE_MEDIAN=0`**: This is critical
3. **Check Modbus latency**: Should be <5ms per command
4. See [INSPIRE_TELEOP_LESSONS.md](INSPIRE_TELEOP_LESSONS.md) for detailed diagnosis

**Hands are jittery:**
- Increase smoothing: Lower `QUEST_EMA_BYPASS` to 4-6
- Enable median filter: `QUEST_USE_MEDIAN=1` (adds delay)
- Increase quantization: `QUEST_QUANT=8`

**No response:**
- Verify UDP data: Run `python recv_rotations.py`
- Check USB connections: `ls -l /dev/ttyUSB*`
- Test hands directly: `python -m inspire_hand --port /dev/ttyUSB0 --slave-id 1 open all`

---

## Performance Metrics

### Vive Tracker (Optimized)
- **Latency**: 12-15ms end-to-end
- **Control rate**: 100 Hz (servo mode)
- **Safety check overhead**: ~3ms average (workspace every frame, full check every 5 frames)

### Quest Hand Tracking (Optimized)
- **Latency**: 30-50ms end-to-end
- **Control rate**: 60 Hz per hand
- **Modbus latency**: <5ms per command
- **Perceived lag**: Imperceptible when properly configured

---

## Code Organization

### Key Files

**Vive Tracker:**
- `vive_teleop_xarm.py`: Main teleoperation script
- `test_vive_teleop.py`: Connectivity tests
- `Vive_Tracker/`: Tracker interface library

**Quest Hand Tracking:**
- `quest_inspire_teleop.py`: Main hand control script
- `quest_hand_receiver.py`: UDP receiver
- `setup_quest_teleop.sh`: Environment setup script

**Shared:**
- `xarm_adapter.py`: xArm safety wrapper (from AnyDexGrasp)
- `inspire_hands/`: Inspire Hand API

---

## Safety Notes

⚠️ **IMPORTANT**: This system controls expensive hardware.

**Always:**
- Keep emergency stop accessible
- Start with conservative scaling
- Test workspace boundaries before demonstrations
- Monitor for unexpected behavior
- Never disable safety checks

**xArm Safety:**
- Three-layer safety system (workspace, collision, joint limits)
- Smooth re-engagement after violations
- Independent safety monitoring per arm (bimanual mode)

**Inspire Hand Safety:**
- Angle clamping (0-1000 range)
- Missing tracking defaults to safe positions
- Emergency stop (Ctrl+C or foot pedal)
- All fingers open on shutdown

---

## Advanced Topics

### Custom Workspace Limits

Edit `workspace_limits` in `ViveToXArmMapper.__init__()`:

```python
self.workspace_limits = {
    'x': (100.0, 450.0),   # Forward reach
    'y': (-400.0, 400.0),  # Left/right
    'z': (TABLE_Z_MM + 50, 800.0),  # Up/down
}
```

### Custom Quest Mapping Ranges

Adjust environment variables based on your Quest tracking:
- Measure actual curl ranges with `recv_rotations.py`
- Update `QUEST_CURL_MIN/MAX` accordingly
- Do the same for thumb flexion/opposition

### Performance Profiling

Add timing diagnostics temporarily:

```python
import time
t0 = time.time()
# ... operation ...
t1 = time.time()
print(f"Operation took: {(t1-t0)*1000:.1f}ms")
```

---

## Further Reading

- **Quest setup details**: See [QUICKSTART.md](QUICKSTART.md#quest-hand-tracking--inspire-hands)
- **Quest troubleshooting**: See [INSPIRE_TELEOP_LESSONS.md](INSPIRE_TELEOP_LESSONS.md)
- **Vive setup details**: See [QUICKSTART.md](QUICKSTART.md#vive-tracker--xarm)

