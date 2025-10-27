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
- **Control rate**: 30Hz with smooth motion interpolation

### 🚧 Future Phases

- **Phase 2**: Bimanual control (2 trackers → left + right arms)
- **Phase 3**: Gripper teleoperation (button → open/close)
- **Phase 4**: Data recording for imitation learning
- **Phase 5**: Visual feedback and workspace visualization

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

```bash
# 1. Ensure SteamVR is running (green icons for base stations + tracker)
# 2. Move arm to home position
python move_arm_to_home.py

# 3. Start teleoperation
export XARM_IP="192.168.1.214"
python vive_teleop_xarm.py

# Follow on-screen prompts:
#   - Arm moves to home automatically
#   - 3-second countdown for calibration
#   - Hold tracker steady, then start moving
#   - Press Ctrl+C to stop
```

See [QUICKSTART.md](QUICKSTART.md) for detailed step-by-step instructions.

## Repository Structure

```
dex-teleop/
├── Vive_Tracker/              # Vive Tracker interface library
│   ├── track.py               # Core tracking module
│   ├── run_tracker.py         # Standalone tracker viewer
│   ├── fairmotion_ops/        # Transformation utilities
│   └── fairmotion_vis/        # OpenGL visualization
│
├── vive_teleop_xarm.py        # Main teleoperation script ⭐
├── test_vive_teleop.py        # Connectivity test suite
├── test_safety_features.py    # Safety validation suite
├── move_arm_to_home.py        # Helper: position arm before teleop
├── launch_teleop.sh           # Convenience launcher
│
├── README.md                  # This file
├── README_TELEOP.md           # Detailed technical documentation
├── QUICKSTART.md              # Step-by-step usage guide
└── IMPLEMENTATION_SUMMARY.md  # Implementation details
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

- **[QUICKSTART.md](QUICKSTART.md)** - Start here! Step-by-step usage guide
- **[README_TELEOP.md](README_TELEOP.md)** - Technical details and API reference
- **[IMPLEMENTATION_SUMMARY.md](IMPLEMENTATION_SUMMARY.md)** - Architecture and design decisions
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

**Add gripper control:**
```python
# In control loop, check tracker button state
if tracker.get_controller_inputs()['trigger']:
    mapper.adapter.hand.grip(force=800)
```

**Add second arm:**
```python
# Create ViveToXArmMapper for each arm
mapper_left = ViveToXArmMapper(xarm_ip='192.168.1.111', tracker_serial='LHR-XXX')
mapper_right = ViveToXArmMapper(xarm_ip='192.168.1.214', tracker_serial='LHR-YYY')
# Run both in parallel control loop
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
1. Check [QUICKSTART.md](QUICKSTART.md#troubleshooting) troubleshooting section
2. Review [README_TELEOP.md](README_TELEOP.md) for technical details
3. Verify hardware connections (SteamVR, xArm, tracker)
4. Run test suite to isolate problem

## Roadmap

- [x] Single-arm teleoperation (right arm)
- [x] Safety system (collision, limits, boundaries)
- [x] Test suite (connectivity, safety, integration)
- [x] Documentation (quick start, technical, implementation)
- [ ] Bimanual teleoperation (both arms simultaneously)
- [ ] Gripper teleoperation (button mapping)
- [ ] Data recording (trajectory + camera logging)
- [ ] Visual feedback (Open3D real-time visualization)
- [ ] Force feedback (haptic feedback on collisions)

## Acknowledgments

Built on top of:
- **Vive_Tracker** by snuvclab - VR tracking interface
- **AnyDexGrasp** - Dexterous grasping framework
- **xArm Python SDK** by UFACTORY - Robot control API
- **Inspire Hand** - Dexterous gripper hardware

