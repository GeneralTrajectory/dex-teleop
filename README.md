# dex-teleop: VR-based Teleoperation for Dexterous Manipulation

Research-grade teleoperation system for xArm robots with Inspire dexterous grippers using HTC Vive Tracker 3.0.

## Project Overview

This repository implements a safe, intuitive teleoperation interface for controlling robotic arms with 6-DOF tracking. The system is designed for collecting high-quality demonstration data for imitation learning and reinforcement learning of dexterous manipulation tasks.

## Current Status

### ✅ Phase 1: Single-Arm Teleoperation (COMPLETE)

- **Control system**: Relative offset control with workspace scaling
- **Hardware**: 1 Vive Tracker → Right xArm (192.168.1.214)
- **Safety features**: 
  - Table collision prevention (200mm gripper-aware)
  - Joint limit enforcement (Joint1: ±90°)
  - Workspace boundary enforcement
- **Control rate**: 100Hz servo mode streaming with smooth motion interpolation

### ✅ Phase 2: Bimanual Control (COMPLETE)

- **Control system**: Independent control of left and right arms
- **Hardware**: 2 Vive Trackers → Left + Right xArms
- **Features**:
  - Simultaneous calibration for both arms
  - Independent safety monitoring per arm
  - Tracker assignment by serial number
- **Usage**: `python vive_teleop_xarm.py --mode both`

### ✅ Phase 3: Dexterous Gripper Control (COMPLETE)

- **Control system**: Meta Quest hand tracking → Inspire Hands
- **Hardware**: Meta Quest headset → 2 Inspire Hands via UDP
- **Features**:
  - Real-time bimanual finger control (6 DOF per hand)
  - 60Hz control loop with adaptive smoothing
  - Background UDP receiver and async command sending
- **Usage**: `python quest_inspire_teleop.py`

### ✅ Phase 4: Data Recording (COMPLETE)

- **Control system**: Unified recorder for xArm + Inspire Hands
- **Features**:
  - Pedal-triggered start/stop recording
  - Synchronized capture (100Hz xArm, 60Hz Inspire)
  - HDF5 format for ML pipelines
  - Ephemeral mode for testing
  - Replay functionality
- **Usage**: `python teleop_recorder.py record --subject S1 --task pick`

### 🚧 Future Phases

- **Phase 5**: Visual feedback and workspace visualization
- **Phase 6**: Force feedback integration

## Quick Start

### Prerequisites

**Hardware:**
- 2× HTC Vive Base Station 2.0 (mounted, powered on)
- 1× HTC Vive Tracker 3.0 (charged, paired via SteamVR)
- 1× xArm robot (right arm: 192.168.1.214)
- 1× Inspire Hand gripper (mounted on xArm)

**Software:**
- Ubuntu 20.04+ with SteamVR running
- Python 3.7+
- All dependencies from `Vive_Tracker/requirements.txt` installed

### Installation

```bash
cd /home/joshua/Research/dex-teleop

# Install Vive Tracker dependencies
cd Vive_Tracker
pip install openvr scipy ipython PyOpenGL PyOpenGL_accelerate

# Verify installation
cd ..
python test_vive_teleop.py
```

### Basic Usage

**Teleoperation only:**
```bash
export XARM_IP_RIGHT="192.168.1.214"
python vive_teleop_xarm.py --mode right
```

**Record demonstrations:**
```bash
source setup_recorder.sh
python teleop_recorder.py record --subject S1 --task pick

# Press 'b' to start → perform demo → press 'b' to stop
```

See [notes/QUICKSTART.md](notes/QUICKSTART.md) for detailed instructions.
See [RECORDER.md](RECORDER.md) for recording guide.

## Repository Structure

```
dex-teleop/
├── Vive_Tracker/              # Vive Tracker interface library
│   ├── track.py               # Core tracking module
│   ├── run_tracker.py         # Standalone tracker viewer
│   ├── fairmotion_ops/        # Transformation utilities
│   └── fairmotion_vis/        # OpenGL visualization
│
├── vive_teleop_xarm.py        # Vive → xArm teleoperation ⭐
├── quest_inspire_teleop.py    # Quest → Inspire Hands teleoperation ⭐
├── teleop_recorder.py         # Unified recorder (xArm + Inspire) ⭐
├── recorder_lib/              # Recording library
│   ├── state_machine.py       # Recorder orchestrator
│   ├── writer.py              # HDF5 data writer
│   ├── xarm_worker.py         # xArm recording worker
│   ├── inspire_worker.py      # Inspire recording worker
│   ├── pedal.py               # Toggle pedal with debouncing
│   └── replay.py              # Replay functionality
├── load_recording.py          # Data loader utility
├── quest_hand_receiver.py     # UDP receiver for Quest hand data
├── recv_rotations.py          # Simple Quest data test receiver
├── test_vive_teleop.py        # Vive connectivity tests
├── test_safety_features.py    # Safety validation tests
├── test_recorder.py           # Recorder tests
├── move_arm_to_home.py        # Helper: position arm before teleop
├── setup_recorder.sh          # Recorder environment setup
├── launch_teleop.sh           # Vive teleop launcher
│
├── data/                      # Recording output directory
├── README.md                  # This file
├── RECORDER_README.md         # Recording system guide
├── notes/                     # Detailed documentation
│   ├── QUICKSTART.md          # Complete user guide
│   ├── TECHNICAL.md           # Technical reference
│   └── INSPIRE_TELEOP_LESSONS.md  # Quest troubleshooting
└── Vive_Tracker/              # Vive interface library
```

## Key Features

### 🎮 Intuitive Control

- **Relative offset mapping**: Your hand movements → robot movements (scaled)
- **Auto-calibration**: 3-second setup, no manual alignment needed
- **Smooth tracking**: 30Hz control loop with motion interpolation
- **Configurable scaling**: Adjust sensitivity to match your preference

### 🛡️ Comprehensive Safety

- **Table collision prevention**: Accounts for 200mm gripper extension, multi-point sampling
- **Joint limits**: Prevents reaching behind table (Joint1: -90° to +90°)
- **Workspace boundaries**: Hard limits on X, Y, Z positions
- **Graceful degradation**: Holds safe position on violations, never executes unsafe commands
- **Emergency stop**: Ctrl+C or xArm STOP button

### 🔬 Research-Ready

- **Standard approach**: Relative offset control (ALOHA/DROID methodology)
- **Reproducible**: Full configuration via environment variables
- **Well-tested**: Comprehensive test suite validates all safety features
- **Extensible**: Clean class structure for adding features
- **Documented**: Extensive documentation for future researchers

## Documentation

- **[notes/QUICKSTART.md](notes/QUICKSTART.md)** - Complete user guide (Vive + Quest + Recording)
- **[RECORDER.md](RECORDER.md)** - Recording system guide (NEW)
- **[notes/TECHNICAL.md](notes/TECHNICAL.md)** - Technical reference and tuning
- **[notes/INSPIRE_TELEOP_LESSONS.md](notes/INSPIRE_TELEOP_LESSONS.md)** - Quest troubleshooting guide
- **[Vive_Tracker/README.md](Vive_Tracker/README.md)** - Vive Tracker setup and usage

## Safety Notes

⚠️ **CRITICAL SAFETY REMINDERS:**

1. **Always have emergency stop accessible** - xArm STOP button or Ctrl+C
2. **Clear workspace before each session** - Remove obstacles, verify table is secure
3. **Start conservatively** - Use low scaling (1.0-1.5) and speed (100-200 mm/s)
4. **Test safety features** - Run `test_safety_features.py` after any code changes
5. **Never disable safety** - Don't bypass collision checks or workspace limits
6. **Monitor continuously** - Watch arm behavior, stop immediately if unexpected

The system is designed to prevent damage, but human supervision is essential.

## Configuration Examples

### Conservative (First-Time Use)
```bash
export VIVE_POSITION_SCALE="1.0"
export VIVE_ROTATION_SCALE="0.8"
export TELEOP_SPEED="100"
python vive_teleop_xarm.py
```

### Balanced (Default)
```bash
export VIVE_POSITION_SCALE="1.5"
export VIVE_ROTATION_SCALE="1.0"
export TELEOP_SPEED="200"
python vive_teleop_xarm.py
```

### Responsive (Experienced Users)
```bash
export VIVE_POSITION_SCALE="2.0"
export VIVE_ROTATION_SCALE="1.0"
export TELEOP_SPEED="250"
export TELEOP_RATE_HZ="50"
python vive_teleop_xarm.py
```

## Testing

### Run All Tests

```bash
# 1. Connectivity tests (safe, no motion)
python test_vive_teleop.py

# 2. Safety feature tests (safe, no motion)
python test_safety_features.py

# 3. Live teleoperation (moves robot)
python vive_teleop_xarm.py --position-scale 1.0 --speed 100
```

### Expected Test Results

All tests should pass before using the system for demonstrations:
- ✅ Tracker detection
- ✅ Pose streaming
- ✅ xArm connection
- ✅ Coordinate transformation
- ✅ Workspace boundaries
- ✅ Table collision detection
- ✅ Integrated safety validation

## Troubleshooting

See [QUICKSTART.md](QUICKSTART.md#troubleshooting) for common issues and solutions.

**Most common issues:**

1. **"No Vive trackers detected"** → Ensure SteamVR running, tracker paired and powered
2. **"Failed to connect to xArm"** → Check IP, network, ensure no other connections
3. **"Safety block: Table collision"** → Expected behavior, move tracker upward
4. **Motion feels wrong** → Adjust `VIVE_POSITION_SCALE` (try 1.0, 1.5, or 2.0)

## Development

### Adding Features

The codebase is designed for extension:

**Add data recording:**
```python
# In ViveToXArmMapper.compute_target_pose()
# Save (timestamp, tracker_pose, arm_pose, camera_frame) to dataset
```

**Add data recording:**
```python
# See quest_inspire_teleop.py for example of recording hand tracking data
# Similar pattern can be applied to vive_teleop_xarm.py
```

### Code Organization

- **`vive_teleop_xarm.py`**: Core implementation (280 lines)
  - `ViveToXArmMapper`: Main class handling all logic
  - `run_teleoperation()`: Control loop
  - `main()`: CLI interface

- **Test scripts**: Comprehensive validation
  - Unit tests for safety features
  - Integration tests for full system
  - Hardware-in-loop tests with real devices

- **Documentation**: Multiple levels
  - Quick start for immediate use
  - Technical docs for understanding
  - Implementation summary for maintenance

## Contributing

When modifying this code:

1. **Test safety features first**: Run `test_safety_features.py`
2. **Test connectivity**: Run `test_vive_teleop.py`
3. **Test with robot**: Start conservatively, verify behavior
4. **Update documentation**: Keep docs in sync with code
5. **Maintain safety**: Never bypass collision checks

## License

This project combines code from:
- **Vive_Tracker**: Original implementation from snuvclab/Vive_Tracker
- **AnyDexGrasp**: Original grasping framework
- **This teleoperation layer**: Integration and safety features

Refer to individual component licenses for details.

## Citation

If you use this teleoperation system in your research:

```bibtex
@software{vive_xarm_teleop,
  title={Vive Tracker Teleoperation for xArm with Dexterous Grippers},
  author={Your Name},
  year={2025},
  howpublished={\url{https://github.com/yourusername/dex-teleop}}
}
```

Also cite the underlying systems:
- HTC Vive Tracker implementation: https://github.com/snuvclab/Vive_Tracker
- xArm Python SDK: https://github.com/xArm-Developer/xArm-Python-SDK

## Support

For issues:
1. Check [notes/QUICKSTART.md](notes/QUICKSTART.md#troubleshooting) troubleshooting section
2. Review [notes/README_TELEOP.md](notes/README_TELEOP.md) for technical details
3. Verify hardware connections (SteamVR, xArm, tracker)
4. Run test suite to isolate problem

## Roadmap

- [x] Single-arm teleoperation (right arm)
- [x] Safety system (collision, limits, boundaries)
- [x] Test suite (connectivity, safety, integration)
- [x] Documentation (quick start, technical, implementation)
- [x] Bimanual teleoperation (both arms simultaneously)
- [x] Dexterous gripper control (Quest hand tracking → Inspire Hands)
- [x] Data recording (synchronized xArm + Inspire capture, HDF5 format)
- [ ] Camera integration (synchronized RGB-D frames)
- [ ] Visual feedback (Open3D real-time visualization)
- [ ] Force feedback (haptic feedback on collisions)

## Acknowledgments

Built on top of:
- **Vive_Tracker** by snuvclab - VR tracking interface
- **AnyDexGrasp** - Dexterous grasping framework
- **xArm Python SDK** by UFACTORY - Robot control API
- **Inspire Hand** - Dexterous gripper hardware

