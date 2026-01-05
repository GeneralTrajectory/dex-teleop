# Installation Guide

Complete setup instructions for the dex-teleop system.

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Software Installation](#software-installation)
3. [Hardware Setup](#hardware-setup)
4. [Configuration](#configuration)
5. [Testing](#testing)
6. [Troubleshooting](#troubleshooting)

---

## Prerequisites

### System Requirements

- **Operating System**: Ubuntu 20.04 or 22.04 (tested)
- **Python**: 3.8 or higher
- **GPU**: NVIDIA GPU recommended (for SteamVR)
- **Network**: Ethernet connection to xArm robot(s)

### Hardware Requirements

| Component | Specification |
|-----------|--------------|
| xArm Robot | xArm 6 or xArm 7 |
| Inspire Hand | Left and/or Right |
| Vive Base Stations | 2× Base Station 2.0 |
| Vive Trackers | 1-2× Vive Tracker 3.0 |
| Meta Quest | Quest 2, Quest 3, or Quest Pro |
| USB Serial | For Inspire Hand communication |

---

## Software Installation

### Step 1: Clone the Repository

```bash
git clone https://github.com/your-username/dex-teleop.git
cd dex-teleop

# Initialize submodules (Vive_Tracker)
git submodule update --init --recursive
```

### Step 2: Create Virtual Environment (Recommended)

```bash
python3 -m venv venv
source venv/bin/activate
```

### Step 3: Install Python Dependencies

```bash
pip install -r requirements.txt
```

### Step 4: Install xArm Python SDK

The xArm SDK must be installed from GitHub (the PyPI package is outdated):

```bash
pip install git+https://github.com/xArm-Developer/xArm-Python-SDK.git
```

Verify installation:

```python
python -c "from xarm.wrapper import XArmAPI; print('xArm SDK installed!')"
```

### Step 5: Install Inspire Hand Library

```bash
# Install the performance-optimized fork (recommended for real-time control):
pip install git+https://github.com/GeneralTrajectory/inspire_hands.git

# Or install the original Sentdex version (may have latency issues):
# pip install git+https://github.com/Sentdex/inspire_hands.git
```

> ⚠️ **Important**: The stock Sentdex library has 200ms fixed delays that limit control
> to ~5Hz. The performance fork replaces these with fast polling (<30ms) to achieve 60Hz.
> See [docs/INSPIRE_TELEOP_LESSONS.md](docs/INSPIRE_TELEOP_LESSONS.md) for details.

### Step 6: Install SteamVR (for Vive Tracking)

1. Install Steam:
   ```bash
   sudo apt install steam
   ```

2. Launch Steam and install SteamVR from the Steam Store

3. Connect your Vive base stations and tracker

4. Launch SteamVR and pair the tracker:
   - SteamVR menu → Devices → Pair Controller
   - Follow the pairing instructions

### Step 7: Install Quest Hand Streaming App

Install the pre-built APK on your Quest headset:

```bash
# Enable developer mode on Quest (via Oculus app on phone)
# Connect Quest via USB

adb install quest_app/HandStream.apk
```

Or build from source - see [quest_app/README.md](quest_app/README.md).

---

## Hardware Setup

### xArm Robot Setup

1. **Power on the xArm** and wait for initialization

2. **Connect via Ethernet**:
   - Connect xArm to your network switch
   - Default IP: `192.168.1.xxx` (check teach pendant)

3. **Verify connection**:
   ```bash
   ping 192.168.1.214  # Replace with your xArm IP
   ```

4. **Clear any errors** on the teach pendant before teleoperation

### Vive Tracker Setup

<p align="center">
  <img src="docs/images/vive_room_setup.png" alt="Vive Base Station Room Setup" width="500">
</p>

1. **Mount base stations**:
   - Position 2 base stations in opposite corners of your workspace (as shown above)
   - Mount at 2+ meters height, angled down ~30°
   - Ensure clear line-of-sight between stations and to your tracking area

2. **Pair tracker**:
   - Power on tracker (hold button until blue LED)
   - In SteamVR: Devices → Pair Controller
   - When paired, LED turns green

3. **Find tracker serial number**:
   - SteamVR → Devices → Device Info
   - Note the serial (format: `LHR-XXXXXXXX`)

4. **Attach tracker to wrist**:
   - Secure tracker to wrist/forearm with strap
   - Tracker orientation: buttons facing up

### Inspire Hand Setup

1. **Connect via USB**:
   - Connect Inspire Hand USB cable to computer
   - Check device appears: `ls -l /dev/ttyUSB*`

2. **Set permissions**:
   ```bash
   sudo chmod 666 /dev/ttyUSB0
   sudo chmod 666 /dev/ttyUSB1
   
   # Or add yourself to dialout group (permanent):
   sudo usermod -a -G dialout $USER
   # Log out and back in
   ```

3. **Test connection**:
   ```bash
   python -c "from inspire_hand import InspireHand; h = InspireHand('/dev/ttyUSB0'); h.open(); print('Connected!'); h.close()"
   ```

### Quest Setup

1. **Enable developer mode**:
   - Oculus app on phone → Settings → Developer Mode

2. **Connect to same network**:
   - Quest and computer must be on same WiFi
   - 5GHz WiFi recommended for lower latency

3. **Find computer IP**:
   ```bash
   ip addr show | grep "inet "
   # Note your local IP (e.g., 192.168.1.100)
   ```

4. **Configure Quest app**:
   - Currently hardcoded in Unity source
   - Rebuild with your IP or use default and configure firewall

---

## Configuration

### Create Configuration File

```bash
cp config/example.env .env
```

### Edit Configuration

Open `.env` and set your values:

```bash
# xArm IPs (find in teach pendant)
export XARM_IP_LEFT="192.168.1.111"
export XARM_IP_RIGHT="192.168.1.214"

# Vive Tracker serials (find in SteamVR)
export VIVE_TRACKER_LEFT="LHR-XXXXXXXX"
export VIVE_TRACKER_RIGHT="LHR-YYYYYYYY"

# Movement scaling (adjust to preference)
export VIVE_POSITION_SCALE="1.5"

# Table height (measure in robot base frame)
export TABLE_Z_MM="15"
```

### Load Configuration

```bash
source .env
```

---

## Testing

### Test 1: Vive Tracker Detection

```bash
# Ensure SteamVR is running
python test_vive_teleop.py
```

Expected output:
```
✅ Detected 1 tracker(s)
   - LHR-A56A8A21
✅ Streamed 150 poses successfully
✅ Connected to xArm at 192.168.1.214
```

### Test 2: xArm Safety Features

```bash
python test_safety_features.py
```

### Test 3: Quest UDP Reception

```bash
# On computer:
python recv_rotations.py

# Launch Quest app
# You should see:
# left  joints=21 tracked=21/21 curls={'Thumb': 0.5, ...}
```

### Test 4: Inspire Hand Connection

```bash
python -c "
from inspire_hand import InspireHand
h = InspireHand('/dev/ttyUSB0', slave_id=1)
h.open()
h.open_all_fingers()
print('✅ Hand opened!')
import time; time.sleep(1)
h.close()
"
```

### Test 5: Full Teleoperation (Conservative)

```bash
source .env
python vive_teleop_xarm.py --mode right --position-scale 1.0 --speed 100
```

---

## Troubleshooting

### SteamVR Issues

**"SteamVR not running"**
```bash
# Start Steam
steam &
# Click VR button in Steam UI
```

**"No base stations detected"**
- Check base stations are powered (green LED)
- Verify USB link box connection
- Try re-pairing in SteamVR

**Tracker not tracking**
- Ensure line-of-sight to base stations
- Check tracker battery (charge if needed)
- Reduce reflective surfaces in room

### xArm Issues

**"Cannot connect to xArm"**
```bash
# Check network
ping 192.168.1.214

# If no response:
# 1. Verify xArm is powered on
# 2. Check Ethernet cable
# 3. Verify IP in teach pendant
```

**"xArm not ready"**
- Clear errors on teach pendant
- Press enable button on teach pendant
- Run `python move_arm_to_home.py`

**Error codes**
| Code | Meaning | Solution |
|------|---------|----------|
| -1 | Disconnected | Check network |
| -2 | Not ready | Enable motion |
| -7 | Joint limit | Reduce motion |
| -8 | Out of range | Check workspace |
| 31 | Collision | Clear error, reduce force |

### Inspire Hand Issues

**"/dev/ttyUSB* not found"**
```bash
# Check USB connection
lsusb | grep -i serial

# Unplug and replug USB cable
dmesg | tail -20
```

**"Permission denied"**
```bash
sudo chmod 666 /dev/ttyUSB0
# Or add to dialout group (permanent)
sudo usermod -a -G dialout $USER
```

**"Modbus timeout"**
- Check cable connections
- Verify slave_id (usually 1)
- Reduce polling rate

### Quest Issues

**"No UDP data received"**
```bash
# Check firewall
sudo ufw allow 9000/udp

# Verify Quest IP
adb shell ip addr show wlan0

# Test with netcat
nc -ul 9000
```

**"High latency"**
- Use 5GHz WiFi network
- Reduce network congestion
- Consider USB tethering

---

## Next Steps

After successful installation:

1. **Read the Quick Start**: [docs/QUICKSTART.md](docs/QUICKSTART.md)
2. **Learn Recording**: [docs/RECORDER.md](docs/RECORDER.md)
3. **Tune Performance**: [docs/TECHNICAL.md](docs/TECHNICAL.md)

For issues, check the [Troubleshooting](#troubleshooting) section or open a GitHub issue.
