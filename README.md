# dex-teleop

**VR-based Teleoperation for Dexterous Manipulation**

A complete system for teleoperating robot arms and dexterous grippers using VR tracking. Control xArm robots with HTC Vive Trackers and Inspire Hands with Meta Quest hand tracking, all synchronized and recorded for imitation learning.

<p align="center">
  <img src="https://img.shields.io/badge/python-3.8+-blue.svg" alt="Python 3.8+">
  <img src="https://img.shields.io/badge/license-MIT-green.svg" alt="MIT License">
  <img src="https://img.shields.io/badge/platform-Ubuntu%2020.04+-orange.svg" alt="Ubuntu 20.04+">
</p>

<p align="center">
  <img src="docs/images/teleop_compressed.gif" alt="Bimanual teleoperation demo" width="600">
</p>

## Features

- **Vive Tracker → xArm Control**: 6-DOF teleoperation with 100Hz servo streaming
- **Quest Hand Tracking → Inspire Hands**: Real-time bimanual dexterous control at 60Hz
- **Synchronized Recording**: Capture demonstrations in HDF5 format for ML pipelines
- **Safety System**: Multi-layer collision detection, workspace limits, smooth re-engagement
- **Bimanual Support**: Control two arms and two hands simultaneously

## Quick Start

### 1. Install Dependencies

```bash
# Clone the repository
git clone https://github.com/your-username/dex-teleop.git
cd dex-teleop

# Install Python dependencies
pip install -r requirements.txt

# Install xArm SDK (from GitHub, not PyPI)
pip install git+https://github.com/xArm-Developer/xArm-Python-SDK.git

# Install Vive Tracker library (as submodule)
git submodule update --init --recursive
```

### 2. Configure Your Hardware

```bash
# Copy and edit the configuration template
cp config/example.env .env
# Edit .env with your robot IPs and tracker serials
```

### 3. Run Teleoperation

```bash
# Source your configuration
source .env

# Test xArm connection (dry run - robot won't move)
python tests/test_vive_teleop.py

# Enable live control (CAUTION: robot will move!)
export EXECUTE_LIVE=1

# Start teleoperation (right arm)
python vive_teleop_xarm.py --mode right

# Or bimanual control
python vive_teleop_xarm.py --mode both
```

> ⚠️ **Safety**: By default, `EXECUTE_LIVE=0` and the robot won't move.
> Set `EXECUTE_LIVE=1` only after verifying your workspace is clear.

See [INSTALLATION.md](INSTALLATION.md) for detailed setup instructions.

## System Overview

```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│  Vive Tracker   │────▶│  vive_teleop.py  │────▶│   xArm Robot    │
│   (per wrist)   │     │   100Hz servo    │     │  (left/right)   │
└─────────────────┘     └──────────────────┘     └─────────────────┘

┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│  Quest Headset  │────▶│ quest_teleop.py  │────▶│  Inspire Hand   │
│  (hand track)   │ UDP │    60Hz async    │     │  (left/right)   │
└─────────────────┘     └──────────────────┘     └─────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│                    teleop_recorder.py                            │
│  Synchronized capture → HDF5 files → ML training pipelines       │
└─────────────────────────────────────────────────────────────────┘
```

## Hardware Requirements

| Component | Requirement |
|-----------|-------------|
| **Robot Arm** | xArm 6/7 (tested with xArm 7) |
| **Gripper** | Inspire Hand (left and/or right) |
| **Arm Tracking** | HTC Vive Tracker 3.0 + Base Stations 2.0 |
| **Hand Tracking** | Meta Quest 2/3/Pro |
| **Computer** | Ubuntu 20.04+, NVIDIA GPU recommended |

## Usage Examples

### Single-Arm Teleoperation

```bash
export XARM_IP_RIGHT="192.168.1.214"
python vive_teleop_xarm.py --mode right --position-scale 1.5
```

### Bimanual with Recording

```bash
source config/example.env
python teleop_recorder.py record --subject S1 --task pick --arms both
# Press 'b' to start/stop recording
```

### Quest Hand Control Only

```bash
source setup_quest_teleop.sh
python quest_inspire_teleop.py
```

### Replay a Recording

```bash
python teleop_recorder.py replay --trial-id 20250130_143022 --speed 0.5 --dry-run
```

## Configuration

All settings are configurable via environment variables. See [config/example.env](config/example.env) for a complete list.

Key settings:

| Variable | Description | Default |
|----------|-------------|---------|
| `XARM_IP_LEFT` | Left arm IP address | `192.168.1.111` |
| `XARM_IP_RIGHT` | Right arm IP address | `192.168.1.214` |
| `VIVE_TRACKER_LEFT` | Left tracker serial | Auto-detect |
| `VIVE_TRACKER_RIGHT` | Right tracker serial | Auto-detect |
| `VIVE_POSITION_SCALE` | Movement sensitivity | `1.5` |
| `TABLE_Z_MM` | Table height for collision | `15` |

## Project Structure

```
dex-teleop/
├── vive_teleop_xarm.py      # Vive → xArm teleoperation
├── quest_inspire_teleop.py  # Quest → Inspire Hands control
├── teleop_recorder.py       # Unified recorder with pedal control
├── xarm_adapter.py          # xArm safety wrapper
├── quest_hand_receiver.py   # UDP receiver for Quest data
├── load_recording.py        # Data loading utilities
├── recv_rotations.py        # Quest data debug receiver
├── move_arm_to_home.py      # Helper to position robot
│
├── recorder_lib/            # Recording library modules
├── Vive_Tracker/            # Vive tracking interface (submodule)
├── quest_app/               # Quest Unity app source + APK
│
├── scripts/                 # Shell scripts for setup
├── tests/                   # Test suite
├── docs/                    # Additional documentation
├── config/                  # Configuration templates
└── data/                    # Recorded demonstrations (gitignored)
```

## Recording Data Format

Recordings are saved in HDF5 format:

```
trial_<timestamp>.h5
├── /metadata              # Subject, task, duration, etc.
├── /xarm_left             # Left arm trajectory (100Hz)
│   ├── timestamps_mono    # Monotonic timestamps
│   ├── joint_positions    # 7-DOF joint angles
│   └── tcp_poses          # 6-DOF TCP pose
├── /xarm_right            # Right arm trajectory
├── /inspire_left          # Left hand angles (60Hz)
└── /inspire_right         # Right hand angles
```

Load recordings in Python:

```python
from load_recording import load_trial

data = load_trial('20250130_143022')
joints = data['xarm_left']['joint_positions']  # (N, 7)
hand = data['inspire_left']['angles']          # (N, 6)
```

## Safety Features

The system includes multiple safety layers:

1. **Workspace Boundaries**: Prevents reaching outside defined limits
2. **Table Collision Detection**: Multi-point gripper collision check
3. **Joint Limits**: Prevents unreachable configurations
4. **Smooth Re-engagement**: Gradual motion resumption after violations
5. **Emergency Stop**: Ctrl+C or foot pedal ('b' key)

## Documentation

- [INSTALLATION.md](INSTALLATION.md) - Detailed setup guide
- [docs/RECORDER.md](docs/RECORDER.md) - Recording system guide
- [docs/QUICKSTART.md](docs/QUICKSTART.md) - Quick reference
- [docs/TECHNICAL.md](docs/TECHNICAL.md) - Technical details
- [quest_app/README.md](quest_app/README.md) - Quest app documentation

## Troubleshooting

**"No Vive trackers detected"**
- Ensure SteamVR is running
- Check tracker is powered on (blue LED)
- Re-pair tracker in SteamVR

**"Failed to connect to xArm"**
- Verify xArm is powered on
- Check IP address: `ping 192.168.1.214`
- Ensure no other connections to xArm

**Quest hands don't respond**
- Verify Quest app is streaming (check HUD)
- Test with: `python recv_rotations.py`
- Check USB ports: `ls -l /dev/ttyUSB*`

## Citation

If you use this system in your research, please cite:

```bibtex
@software{dex_teleop,
  title={dex-teleop: VR-based Teleoperation for Dexterous Manipulation},
  author={Belofsky, Joshua},
  year={2025},
  url={https://github.com/your-username/dex-teleop}
}
```

Also cite the underlying systems:

- **Vive Tracker**: [snuvclab/Vive_Tracker](https://github.com/snuvclab/Vive_Tracker)
- **xArm SDK**: [xArm-Developer/xArm-Python-SDK](https://github.com/xArm-Developer/xArm-Python-SDK)

## License

This project is licensed under the MIT License - see [LICENSE](LICENSE) for details.

## Acknowledgments

Built with:
- [Vive_Tracker](https://github.com/snuvclab/Vive_Tracker) by snuvclab
- [xArm Python SDK](https://github.com/xArm-Developer/xArm-Python-SDK) by UFACTORY
- Meta Quest XR SDK for hand tracking
