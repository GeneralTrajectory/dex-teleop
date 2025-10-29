# Meta Quest Hand Tracking → Inspire Hand Control

## Overview

Real-time bimanual dexterous hand control using Meta Quest hand tracking streamed over UDP to control physical Inspire Hands.

## Quick Start

### Prerequisites

1. **Meta Quest headset** with Unity hand tracking app running
2. **Two Inspire Hands** connected:
   - Left hand: `/dev/ttyUSB1`, slave_id 1
   - Right hand: `/dev/ttyUSB0`, slave_id 1
3. **Network connection** between Quest and computer (same WiFi or USB tethering)

### Running

```bash
# Terminal 1: Start Quest app on headset
# [Launch Unity app, ensure UDP streaming enabled to port 9000]

# Terminal 2: Run hand control
cd /home/joshua/Research/dex-teleop
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

✅ Bimanual hand control active at ~30 Hz
   Press 'b' (foot pedal) or Ctrl+C to stop
------------------------------------------------------------
[left ] Curls: L:0.2 R:0.5 M:0.8 I:1.2 T:1.0 | Angles: 933 833 733 600 667
[right] Curls: L:0.1 R:0.3 M:0.4 I:0.5 T:0.8 | Angles: 967 900 867 833 733
```

## How It Works

### Data Flow

```
Quest Headset (Unity App)
    ↓
UDP packets @ 30-90 Hz
    ↓ JSON: {hand, q[], pos[], m[]}
QuestHandReceiver
    ↓ Parse rotations & positions
    ↓ Compute finger curls
HandTrackingMapper
    ↓ Map curls → angles [0-1000]
InspireHand API
    ↓ Modbus RTU commands
Physical Inspire Hands
```

### Finger Mappings

**Inspire Hand 6 DOF:**
- Finger 0: Little finger
- Finger 1: Ring finger
- Finger 2: Middle finger
- Finger 3: Index finger
- Finger 4: Thumb bend (flexion)
- Finger 5: Thumb rotate (opposition)

**Quest → Inspire Mapping:**

| Quest Metric | Range | Inspire Finger | Angle Range | Mapping |
|--------------|-------|----------------|-------------|---------|
| Little curl | 0-3 | Little (0) | 1000-0 | Linear invert |
| Ring curl | 0-3 | Ring (1) | 1000-0 | Linear invert |
| Middle curl | 0-3 | Middle (2) | 1000-0 | Linear invert |
| Index curl | 0-3 | Index (3) | 1000-0 | Linear invert |
| Thumb flexion | 0-180° | Thumb bend (4) | 1000-0 | Linear invert |
| Thumb opposition | 10-90° | Thumb rotate (5) | 0-1000 | Inverted scale |

### Mapping Functions

```python
# Main fingers: curl → angle
curl_to_angle(curl):
    # curl: 0.0 (open) → 3.0 (fully curled)
    # angle: 1000 (open) → 0 (closed)
    angle = 1000 - (curl / 3.0) * 1000
    
# Thumb bend: flexion → angle
flexion_to_angle(flexion_deg):
    # flexion: 0° (open) → 180° (closed)
    # angle: 1000 (open) → 0 (closed)
    angle = 1000 - (flexion / 180.0) * 1000
    
# Thumb rotate: opposition → angle
opposition_to_angle(opposition_deg):
    # opposition: 90° (opposed) → 10° (parallel)
    # angle: 1000 (opposed) → 0 (parallel)
    angle = ((90 - opposition) / 80.0) * 1000
```

## Configuration

### Network

**Quest IP Address**: Check in Quest settings
**Computer**: Ensure reachable from Quest (same network)
**Port**: 9000 (UDP, must be open on firewall)

### Inspire Hands

**Left Hand:**
- Port: `/dev/ttyUSB1`
- Slave ID: 1
- Baudrate: 115200

**Right Hand:**
- Port: `/dev/ttyUSB0`
- Slave ID: 1
- Baudrate: 115200

### Performance

**Control Rate**: ~30 Hz (adjustable in code)
**Latency**: 50-100ms typical
**Smoothing**: None by default (add EMA if jittery)

## Safety Features

### Angle Clamping
All angles are clamped to [0, 1000] range to prevent invalid commands.

### Missing Tracking Handling
If tracking is lost for a finger:
- Main fingers: Default to open (1000)
- Thumb bend: Default to open (1000)
- Thumb rotate: Default to middle (500)

### Emergency Stop
- **Ctrl+C**: Immediate stop
- **Foot Pedal ('b' key)**: Immediate stop
- **On stop**: All fingers open to safe position before disconnect

## Troubleshooting

### No UDP Data Received

**Symptom**: No status updates, hands don't move

**Solutions**:
1. Verify Quest app is running and streaming
2. Check network connectivity: `ping <quest-ip>`
3. Verify port 9000 is not blocked:
   ```bash
   sudo netstat -tulpn | grep 9000
   ```
4. Check Quest app settings (ensure UDP enabled)

### Hands Don't Respond

**Symptom**: Status updates appear but hands don't move

**Solutions**:
1. Check USB connections: `ls -l /dev/ttyUSB*`
2. Verify slave IDs match configuration
3. Test with inspire_hand CLI:
   ```bash
   python -m inspire_hand --port /dev/ttyUSB0 --slave-id 1 open all
   ```
4. Check for Modbus errors in output

### Jittery Motion

**Symptom**: Hands vibrate or shake

**Solutions**:
1. Reduce control rate (change `1.0 / 30.0` to `1.0 / 20.0`)
2. Add EMA smoothing in `HandTrackingMapper`
3. Increase Quest app filtering/smoothing

### Tracking Loss

**Symptom**: Fingers snap to default positions

**Solutions**:
1. Improve Quest tracking (better lighting, hand visibility)
2. Hold hands within Quest's tracking volume
3. Check `tracked` count in status output

## Performance Tuning

### Increase Responsiveness
```python
# In quest_inspire_teleop.py, line ~250
sleep_time = (1.0 / 50.0) - elapsed  # 50Hz instead of 30Hz
```

### Add Smoothing (Reduce Jitter)
```python
class HandTrackingMapper:
    def __init__(self):
        self.ema_alpha = 0.3  # Smoothing factor
        self.last_angles = [1000] * 6
    
    def smooth_angles(self, new_angles):
        smoothed = []
        for i, new_val in enumerate(new_angles):
            smooth_val = self.ema_alpha * new_val + (1 - self.ema_alpha) * self.last_angles[i]
            smoothed.append(int(smooth_val))
        self.last_angles = smoothed
        return smoothed
```

### Adjust Sensitivity

Modify mapping functions to change sensitivity:
```python
# More sensitive (smaller curls = more closure)
angle = int(1000 - (curl_clamped / 2.0) * 1000)  # Use /2.0 instead of /3.0

# Less sensitive (requires larger curls)
angle = int(1000 - (curl_clamped / 4.0) * 1000)  # Use /4.0 instead of /3.0
```

## Testing

### Test UDP Reception
```bash
# Run the original receiver to verify data
cd /home/joshua/Desktop
python recv_rotations.py
```

Expected output:
```
listening on UDP *:9000
left  joints=21 tracked=21/21 curls={'Thumb': 0.5, 'Index': 1.2, ...}
right joints=21 tracked=21/21 curls={'Thumb': 0.3, 'Index': 0.8, ...}
```

### Test Inspire Hands
```bash
# Left hand
cd /home/joshua/Research/inspire_hands
./hand1.sh info

# Right hand
./hand2.sh info
```

### Test Integration
```bash
# Run hand control with Quest app active
python quest_inspire_teleop.py
```

## Gesture Examples

With Quest hand tracking active:

**Open Hand**: Fingers extended → Inspire angles ~1000  
**Closed Fist**: Fingers curled → Inspire angles ~0  
**Pinch**: Thumb + index curled → Inspire thumb + index ~0, others ~1000  
**Point**: Index extended, others curled → Inspire index ~1000, others ~0  
**Thumbs Up**: Thumb extended, fingers curled → Inspire thumb ~1000, fingers ~0

## Advanced Usage

### Single Hand Mode

Modify initialization to use only one hand:
```python
# Only right hand
hands = {}
hands['right'] = InspireHand(port='/dev/ttyUSB0', slave_id=1)
hands['right'].open()
```

### Custom Port Configuration

```python
# Different ports or slave IDs
hands['left'] = InspireHand(port='/dev/ttyUSB2', slave_id=2)
hands['right'] = InspireHand(port='/dev/ttyUSB3', slave_id=1)
```

### Logging

Redirect output for analysis:
```bash
python quest_inspire_teleop.py 2>&1 | tee hand_control.log
```

## Troubleshooting Checklist

- [ ] Quest app is running and streaming UDP
- [ ] Computer and Quest on same network
- [ ] Port 9000 is reachable
- [ ] Both Inspire Hands powered and connected
- [ ] USB ports `/dev/ttyUSB0` and `/dev/ttyUSB1` exist
- [ ] Slave IDs match configuration
- [ ] inspire_hand library installed in Python environment
- [ ] No other programs using the serial ports

## Summary

This implementation provides **simple, robust, real-time control** of bimanual Inspire Hands using Meta Quest hand tracking. The direct curl-to-angle mapping requires no calibration and works immediately for most gestures and manipulation tasks.

**Key Features:**
- ✅ Simple linear mapping (no calibration)
- ✅ Real-time streaming (30Hz)
- ✅ Bimanual control (left + right hands)
- ✅ Graceful tracking loss handling
- ✅ Emergency stop (foot pedal)
- ✅ ~200 lines of code (maintainable)

