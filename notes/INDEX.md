# Vive Tracker Teleoperation - File Index

Quick reference guide to all files in the dex-teleop repository.

## 📖 Start Here

**New to this system?** Read in this order:

1. **[README.md](README.md)** - Project overview and quick start
2. **[QUICKSTART.md](QUICKSTART.md)** - Step-by-step usage guide
3. **[README_TELEOP.md](README_TELEOP.md)** - Technical documentation

## 🚀 Core Scripts

### Main Application
- **`vive_teleop_xarm.py`** (19 KB)
  - Main teleoperation script
  - Class: `ViveToXArmMapper`
  - Usage: `python vive_teleop_xarm.py`
  - What it does: Real-time control of xArm using Vive Tracker

### Helper Scripts
- **`move_arm_to_home.py`** (2.9 KB)
  - Positions arm before teleoperation
  - Usage: `python move_arm_to_home.py`
  - What it does: Moves xArm to HOME_POSE safely

- **`launch_teleop.sh`** (1.9 KB)
  - Convenience launcher with checks
  - Usage: `./launch_teleop.sh`
  - What it does: Checks prerequisites, sets env vars, launches teleop

## 🧪 Test Scripts

### Connectivity Tests (Safe - No Motion)
- **`test_vive_teleop.py`** (7.7 KB)
  - Test 1: Tracker detection
  - Test 2: Pose streaming
  - Test 3: xArm connection
  - Test 4: Coordinate transformation
  - Usage: `python test_vive_teleop.py`
  - Run this FIRST before teleoperation

### Safety Tests (Safe - No Motion)
- **`test_safety_features.py`** (14 KB)
  - Test: Table collision detection
  - Test: Joint1 limit enforcement
  - Test: Workspace boundaries
  - Test: Integrated safety validation
  - Usage: `python test_safety_features.py`
  - Run this SECOND to verify safety

## 📚 Documentation

### User Documentation
- **[README.md](README.md)** (10 KB)
  - Project overview
  - Quick start guide
  - Configuration examples
  - Repository structure

- **[QUICKSTART.md](QUICKSTART.md)** (5.3 KB)
  - Physical setup steps
  - Running teleoperation
  - Controls and behavior
  - Troubleshooting guide

- **[README_TELEOP.md](README_TELEOP.md)** (8.2 KB)
  - Technical details
  - Safety features explained
  - Configuration reference
  - API documentation

### Developer Documentation
- **[IMPLEMENTATION_SUMMARY.md](IMPLEMENTATION_SUMMARY.md)** (18 KB)
  - Architecture overview
  - Design decisions
  - Implementation details
  - Performance characteristics
  - Future enhancements

- **[SYSTEM_DIAGRAM.md](SYSTEM_DIAGRAM.md)** (31 KB)
  - Visual diagrams
  - Control flow charts
  - Coordinate transformations
  - Safety validation flow
  - Data flow examples

## 📁 Libraries

### Vive_Tracker/ (External)
Third-party library for HTC Vive Tracker interface:

- **`track.py`** - Core tracking module
  - `ViveTrackerModule`: Main interface
  - `vr_tracked_device`: Tracker abstraction
  - `get_T()`: Returns 4×4 pose matrix

- **`run_tracker.py`** - Standalone visualization
- **`vive_visualizer.py`** - OpenGL 3D viewer
- **`fairmotion_ops/`** - Transformation utilities
- **`fairmotion_vis/`** - Rendering utilities

See [Vive_Tracker/README.md](Vive_Tracker/README.md) for details.

## 🗂️ File Reference by Purpose

### Getting Started
1. Read: `README.md`
2. Read: `QUICKSTART.md`
3. Run: `test_vive_teleop.py`
4. Run: `test_safety_features.py`
5. Run: `move_arm_to_home.py`
6. Run: `vive_teleop_xarm.py`

### Understanding the System
- Architecture: `IMPLEMENTATION_SUMMARY.md`
- Control flow: `SYSTEM_DIAGRAM.md`
- API details: `README_TELEOP.md`

### Troubleshooting
- Common issues: `QUICKSTART.md` → Troubleshooting section
- Test failures: `README_TELEOP.md` → Troubleshooting section
- Safety validation: `test_safety_features.py`

### Development
- Main code: `vive_teleop_xarm.py`
- Safety logic: Lines 172-240 (`is_pose_safe()`)
- Delta computation: Lines 111-170 (`compute_target_pose()`)
- Calibration: Lines 87-109 (`calibrate_home_poses()`)

### Configuration
- Environment variables: `README_TELEOP.md` → Configuration section
- Command-line args: `vive_teleop_xarm.py --help`
- Default values: `vive_teleop_xarm.py` lines 270-277

## 📊 File Sizes

```
Core implementation:
  vive_teleop_xarm.py          19 KB  (main application)

Tests:
  test_vive_teleop.py           8 KB  (connectivity tests)
  test_safety_features.py      14 KB  (safety validation)

Helpers:
  move_arm_to_home.py           3 KB  (home positioning)
  launch_teleop.sh              2 KB  (launcher)

Documentation:
  README.md                    10 KB  (main readme)
  QUICKSTART.md                 5 KB  (quick start)
  README_TELEOP.md              8 KB  (technical docs)
  IMPLEMENTATION_SUMMARY.md    18 KB  (implementation)
  SYSTEM_DIAGRAM.md            31 KB  (diagrams)
  INDEX.md                      5 KB  (this file)

Total: ~123 KB documentation + code
```

## 🔍 Quick Lookup

**I want to...**

- **Start using the system**
  → Read: `QUICKSTART.md`
  → Run: `launch_teleop.sh`

- **Understand how it works**
  → Read: `IMPLEMENTATION_SUMMARY.md`
  → Read: `SYSTEM_DIAGRAM.md`

- **Test my setup**
  → Run: `test_vive_teleop.py`
  → Run: `test_safety_features.py`

- **Configure settings**
  → Read: `README_TELEOP.md` → Configuration section
  → Edit: Environment variables or CLI args

- **Troubleshoot problems**
  → Read: `QUICKSTART.md` → Troubleshooting section
  → Run: Test scripts to isolate issue

- **Modify the code**
  → Read: `IMPLEMENTATION_SUMMARY.md` → Architecture
  → Edit: `vive_teleop_xarm.py`
  → Test: Run all test scripts

- **Add features**
  → Read: `IMPLEMENTATION_SUMMARY.md` → Future Enhancements
  → Reference: `vive_teleop_xarm.py` → `ViveToXArmMapper` class

## 🎯 Common Tasks

### First-Time Setup
```bash
# 1. Hardware setup (see QUICKSTART.md)
# 2. Test connectivity
python test_vive_teleop.py

# 3. Test safety
python test_safety_features.py

# 4. Position arm
python move_arm_to_home.py

# 5. Start teleop
python vive_teleop_xarm.py
```

### Daily Usage
```bash
# Quick launch (assumes setup done)
./launch_teleop.sh
```

### After Code Changes
```bash
# Re-run safety tests
python test_safety_features.py

# Re-run connectivity tests
python test_vive_teleop.py

# Test with robot (conservative settings)
python vive_teleop_xarm.py --position-scale 1.0 --speed 100
```

## 📝 Version History

### v1.0 (Current) - Single-Arm Teleoperation
- ✅ Right arm control via Vive Tracker
- ✅ Relative offset control with scaling
- ✅ Three-layer safety system
- ✅ Comprehensive test suite
- ✅ Full documentation

### Future Versions
- v1.1: Data recording for imitation learning
- v1.2: Gripper teleoperation (button control)
- v2.0: Bimanual control (both arms simultaneously)
- v2.1: Visual feedback overlay
- v3.0: Force feedback and haptics

## 🆘 Support

### Getting Help

1. **Check documentation**
   - Quick issues: `QUICKSTART.md` → Troubleshooting
   - Technical issues: `README_TELEOP.md`
   - Understanding system: `IMPLEMENTATION_SUMMARY.md`

2. **Run diagnostics**
   - `test_vive_teleop.py` - Hardware connectivity
   - `test_safety_features.py` - Safety validation

3. **Common fixes**
   - Tracker not detected → Check SteamVR, re-pair tracker
   - xArm connection fails → Check IP, network, other connections
   - Safety blocks → Expected! Review safety logs

### Known Issues

None currently. If you discover issues:
1. Run test suite to isolate problem
2. Check relevant documentation section
3. Review code comments in affected module

## 🔗 Related Repositories

- **AnyDexGrasp**: `/home/joshua/Documents/Sources/Papers/Cursor/AnyDexGrasp`
  - Provides: `xarm_adapter.py`, safety features, collision detection
  - Used by: Teleoperation for xArm control and safety

- **inspire_hands**: `/home/joshua/Research/inspire_hands`
  - Provides: Inspire Hand gripper API
  - Future: Will be integrated for gripper control

- **Vive_Tracker**: `./Vive_Tracker/` (submodule)
  - Provides: VR tracking interface
  - Used by: All teleoperation scripts

## 📄 License

See individual component licenses:
- Vive_Tracker: Original project license
- AnyDexGrasp integration: Project license
- Teleoperation layer: (Your license choice)

