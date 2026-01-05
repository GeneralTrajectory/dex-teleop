# Teleoperation Recorder

Complete guide for recording bimanual xArm + Inspire Hands demonstrations.

---

## Quick Start

### Install
```bash
pip install h5py
```

### Record a Demo
```bash
source setup_recorder.sh
python teleop_recorder.py record --subject S1 --task pick --notes "red cube"
```

**Workflow:**
1. System initializes all devices
2. **Press 'b'** (foot pedal) → Arms home, recording starts
3. Perform demonstration via teleoperation
4. **Press 'b'** again → Recording stops and saves to `data/trial_<timestamp>.h5`

### Single-Arm Recording
```bash
# Record with only left arm
python teleop_recorder.py record --arms left --subject S1 --task pick

# Record with only right arm
python teleop_recorder.py record --arms right --subject S1 --task pick

# Record with both arms (default)
python teleop_recorder.py record --arms both --subject S1 --task pick
```

**Use cases:**
- Testing with only one arm available
- Unimanual task demonstrations
- Hardware troubleshooting

### Test Without Saving
```bash
python teleop_recorder.py record --ephemeral
python teleop_recorder.py record --ephemeral --arms left  # Single-arm ephemeral
```

---

## Commands

### record
```bash
python teleop_recorder.py record [--subject ID] [--task NAME] [--notes TEXT] [--ephemeral] [--arms ARMS]

Arguments:
  --subject ID    Subject identifier (e.g., S1)
  --task NAME     Task name (e.g., pick, place)
  --notes TEXT    Session notes
  --ephemeral     Discard data after stop (testing mode)
  --arms ARMS     Which arm(s) to use: left, right, or both (default: both)

Examples:
  python teleop_recorder.py record --subject S1 --task pick
  python teleop_recorder.py record --arms left --subject S1 --task pick
  python teleop_recorder.py record --ephemeral  # Testing mode
  python teleop_recorder.py record --ephemeral --arms right  # Test right arm only
```

### list
```bash
python teleop_recorder.py list

Output:
  Trial: 20250130_143022
    Subject: S1
    Task: pick
    Duration: 15.3s
    Samples: xArm_L=1543, xArm_R=1541
```

### validate
```bash
python teleop_recorder.py validate --trial-id 20250130_143022
```

Checks schema, timestamps, and data integrity.

### replay
```bash
# Dry-run (no robot motion)
python teleop_recorder.py replay --trial-id 20250130_143022 --dry-run

# Live replay
python teleop_recorder.py replay --trial-id 20250130_143022 --speed 0.5
```

---

## Data Format

### HDF5 Schema (v1.0)
```
trial_<timestamp>.h5
├── /metadata (attributes: subject, task, notes, duration_s, active_arms, etc.)
├── /xarm_left (only if 'left' in active_arms)
│   ├── timestamps_mono: float64[N]
│   ├── joint_positions: float32[N, 7]  # REAL joint angles in degrees
│   └── tcp_poses: float32[N, 6]        # REAL TCP pose [x,y,z,roll,pitch,yaw]
├── /xarm_right (only if 'right' in active_arms)
│   ├── timestamps_mono: float64[N]
│   ├── joint_positions: float32[N, 7]  # REAL joint angles in degrees
│   └── tcp_poses: float32[N, 6]        # REAL TCP pose [x,y,z,roll,pitch,yaw]
├── /inspire_left (only if 'left' in active_arms)
│   ├── timestamps_mono: float64[N]
│   └── angles: int16[N, 6]             # Commanded angles [0-1000]
└── /inspire_right (only if 'right' in active_arms)
    ├── timestamps_mono: float64[N]
    └── angles: int16[N, 6]             # Commanded angles [0-1000]
```

**Data Sources:**
- **xArm joint/TCP**: Read from robot feedback via isolated I/O proxy (real positions)
- **Inspire angles**: Commanded values sent to hands (what we told them to do)
- `active_arms` metadata field indicates which arms were recorded ('left', 'right', or 'left,right')

### Loading Data

```python
from load_recording import load_trial

data = load_trial('20250130_143022')

# Check which arms were recorded
active_arms = data['metadata']['active_arms'].split(',')
print(f"Active arms: {active_arms}")

# Access trajectories (check existence first for single-arm recordings)
if 'left' in active_arms:
    left_joints = data['xarm_left']['joint_positions']  # (N, 7)
    left_hand = data['inspire_left']['angles']          # (N, 6)
    times = data['xarm_left']['timestamps_mono']        # (N,)

if 'right' in active_arms:
    right_joints = data['xarm_right']['joint_positions']  # (N, 7)
    right_hand = data['inspire_right']['angles']           # (N, 6)

# Metadata
print(data['metadata']['subject'], data['metadata']['task'])
```

### PyTorch Integration

```python
from torch.utils.data import Dataset
from load_recording import load_trial

class DemoDataset(Dataset):
    def __init__(self, trial_ids):
        self.data = [load_trial(tid) for tid in trial_ids]
    
    def __getitem__(self, idx):
        trial = self.data[idx]
        active_arms = trial['metadata']['active_arms'].split(',')
        
        # Build state vector based on active arms
        # Bimanual: 26D state: xarm_L(7) + xarm_R(7) + inspire_L(6) + inspire_R(6)
        # Single-arm: 13D state: xarm(7) + inspire(6)
        # Return (state, action) pairs
        ...
```

---

## Recording Rates & Storage

- **xArm**: 100 Hz per arm
- **Inspire**: 60 Hz per hand
- **Total**: ~400 samples/second
- **File size**: ~17 MB/minute (~1 GB/hour)

---

## Troubleshooting

**"Need N Vive tracker(s)"**: Power on required trackers (1 for single-arm, 2 for bimanual), check SteamVR

**"xArm not ready"**: Clear errors on teach pendant, reset arm

**"Inspire read failed"**: Check USB (`ls -l /dev/ttyUSB*`)

**"Quest receiver not active"**: Ensure Quest app streaming to port 9000

**Slow/laggy**: Ensure `source setup_recorder.sh` was run (`INSPIRE_SPEED=1000` is critical)

**Pedal doesn't work**: Verify stdin is a terminal, press 'b' key

---

## Implementation Details

### Architecture
```
RecorderOrchestrator
├── ViveToXArmMapper × N (teleoperation, N = 1 or 2 based on --arms)
│   ├── XArmRecorder × N (capture @ 100Hz)
│   └── XArmIOProxy × N (isolated SDK reads in child process)
├── InspireHand × N (teleoperation)
│   └── InspireRecorder × N (capture @ 60Hz, commanded angles)
├── TrackerProxy × N (isolated Vive SDK in child process)
└── HDF5Writer (atomic file I/O)
```

**Single-arm mode:** Only initializes left OR right arm components.
**Bimanual mode (default):** Initializes both left and right.

**Crash Isolation:**
- Vive tracker SDK runs in separate process (RECORDER_USE_TRACKER_PROXY=1, default)
- xArm I/O SDK runs in separate process (RECORDER_USE_XARM_IO_PROXY=1, default)
- If vendor SDKs crash → only child processes die, main recorder continues

### State Machine
```
IDLE → HOMING → RECORDING → STOPPING → SAVING/DISCARDING → IDLE
```

### Safety
- Pre-flight health checks (xArm, trackers, hands, Quest)
- HOME_POSE enforced before every recording
- All teleoperation safety features active during recording
- Atomic file writes (no corrupted recordings)

### Files Created
- `recorder_lib/` (7 modules, ~1850 lines)
- `teleop_recorder.py` (160 lines)
- `load_recording.py` (150 lines)
- `test_recorder.py` (240 lines)
- **Total**: ~2400 lines

---

## Advanced Usage

### Batch Recording
```bash
for i in {1..50}; do
    python teleop_recorder.py record --subject S1 --task pick --notes "trial $i"
    # Operator performs demo between pedal presses
done
```

### Batch Validation
```bash
for trial in data/trial_*.h5; do
    tid=$(basename $trial .h5 | sed 's/trial_//')
    python teleop_recorder.py validate --trial-id $tid
done
```

### Export to CSV
```python
import h5py, pandas as pd

with h5py.File('data/trial_20250130_143022.h5', 'r') as f:
    df = pd.DataFrame(f['/xarm_left/joint_positions'][:])
    df['timestamp'] = f['/xarm_left/timestamps_mono'][:]
    df.to_csv('xarm_left.csv')
```

---

## Summary

Complete recording system for ML data collection:
- ✅ Pedal-triggered start/stop
- ✅ Synchronized 4-device capture
- ✅ HDF5 format (ML-ready)
- ✅ Ephemeral mode (testing)
- ✅ Replay (validation + live)
- ✅ Safe by design
- ✅ ~2400 lines, tested, documented

**Ready for production use in imitation learning pipelines.**



