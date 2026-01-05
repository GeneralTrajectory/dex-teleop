# Quest Hand Tracking Streaming App

This directory contains the Unity source code for the Meta Quest hand tracking streamer app.
The app captures hand tracking data from the Quest headset and streams it over UDP to your computer
for controlling the Inspire Hands.

## Pre-built APK

For convenience, a pre-built APK is available. Install it on your Quest using:

```bash
adb install HandStream.apk
```

## Building from Source

### Requirements

- **Unity 6000.0.23f1** (or compatible Unity 6 version)
- **Meta XR SDK** packages
- **Android Build Support** in Unity

### Unity Packages Required

The app uses the following Unity packages (from `Packages/manifest.json`):

```json
{
  "com.unity.xr.hands": "1.6.2",
  "com.unity.xr.oculus": "4.5.2",
  "com.unity.xr.openxr": "1.16.0",
  "com.unity.inputsystem": "1.14.2",
  "com.unity.render-pipelines.universal": "17.2.0"
}
```

### Setup Steps

1. **Create a new Unity project** (3D URP template recommended)

2. **Install XR packages:**
   - Window → Package Manager
   - Add `com.unity.xr.hands`
   - Add `com.unity.xr.oculus`
   - Add `com.unity.xr.openxr`

3. **Configure XR settings:**
   - Edit → Project Settings → XR Plug-in Management
   - Enable OpenXR for Android
   - Add Meta Quest Support feature
   - Enable Hand Tracking and Meta Hand Tracking Aim features

4. **Copy the scripts** from this directory to your Unity project's `Assets/` folder:
   - `HandRotationsUdpStreamer.cs` - Main UDP streaming logic
   - `HandStreamHUD.cs` - Optional HUD display
   - `QuestPerfInit.cs` - Performance optimization

5. **Create the scene:**
   - Create an empty GameObject and attach `HandRotationsUdpStreamer`
   - Create a Canvas with TMP_Text and attach `HandStreamHUD` (optional)
   - Attach `QuestPerfInit` to any persistent GameObject

6. **Configure the streamer:**
   - Set `receiverIp` to your computer's IP address
   - Set `receiverPort` to 9000 (default)
   - Enable `sendLeft` and/or `sendRight` as needed

7. **Build for Android:**
   - File → Build Settings → Android
   - Set minimum API level to 29+
   - Build and deploy to Quest

## Source Files

### HandRotationsUdpStreamer.cs

The main hand tracking streamer. Captures joint rotations and positions from Quest's XR Hand Subsystem
and sends them as JSON over UDP.

**Key features:**
- Streams at display refresh rate (~90Hz)
- Sends local joint rotations (quaternions)
- Includes thumb position data for flexion/opposition metrics
- Optional heartbeat for connection monitoring

**Configuration:**
```csharp
public string receiverIp = "192.168.1.100";  // Your computer's IP
public int receiverPort = 9000;              // UDP port
public bool sendLeft = true;                 // Stream left hand
public bool sendRight = true;                // Stream right hand
public bool includeTips = false;             // Include fingertip joints
public bool heartbeat = true;                // Send keepalive packets
```

### JSON Packet Format

Each UDP packet contains JSON with:

```json
{
  "hand": "left",
  "f": 12345,
  "t": 1234567890.123456,
  "m": [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
  "q": [x,y,z,w, x,y,z,w, ...],
  "pos": [x,y,z, x,y,z, ...]
}
```

| Field | Description |
|-------|-------------|
| `hand` | "left" or "right" |
| `f` | Frame number |
| `t` | High-precision timestamp (seconds) |
| `m` | Tracking mask (1=tracked, 0=filled) for each joint |
| `q` | Quaternions (x,y,z,w) for 21 joints (84 floats) |
| `pos` | Positions (x,y,z) for 6 key joints (18 floats) |

**Joint order (21 joints):**
1. Wrist, Palm
2. ThumbMetacarpal, ThumbProximal, ThumbDistal
3. IndexMetacarpal, IndexProximal, IndexIntermediate, IndexDistal
4. MiddleMetacarpal, MiddleProximal, MiddleIntermediate, MiddleDistal
5. RingMetacarpal, RingProximal, RingIntermediate, RingDistal
6. LittleMetacarpal, LittleProximal, LittleIntermediate, LittleDistal

**Position order (6 joints):**
Palm, ThumbMetacarpal, ThumbProximal, ThumbDistal, ThumbTip, IndexMetacarpal

### HandStreamHUD.cs

Optional HUD component that displays streaming status on a TextMeshPro text element.
Shows target IP, port, and approximate FPS.

### QuestPerfInit.cs

Performance initialization script that:
- Disables VSync for maximum frame rate
- Requests highest available display refresh rate (90Hz or 120Hz)

## Usage

1. **Install the app on Quest:**
   ```bash
   adb install HandStream.apk
   ```

2. **Launch the app** from Quest's Unknown Sources menu

3. **Configure your computer's IP:**
   - Currently hardcoded in the streamer script
   - TODO: Add in-VR configuration UI

4. **Verify data reception:**
   ```bash
   # On your computer
   cd /path/to/dex-teleop
   python recv_rotations.py
   ```

   You should see:
   ```
   listening on UDP *:9000
   left  joints=21 tracked=21/21 curls={'Thumb': 0.5, 'Index': 1.2, ...}
   right joints=21 tracked=21/21 curls={'Thumb': 0.3, 'Index': 0.8, ...}
   ```

## Troubleshooting

**No data received:**
- Ensure Quest and computer are on the same network
- Check firewall settings (allow UDP port 9000)
- Verify IP address is correct in the Unity script
- Try `ping <quest-ip>` from your computer

**Choppy tracking:**
- Enable 120Hz display mode in Quest settings
- Ensure good lighting for hand tracking
- Keep hands in view of Quest cameras

**High latency:**
- Use 5GHz WiFi network
- Reduce network congestion
- Consider USB tethering for lowest latency

## License

This Quest app is released under the MIT License as part of the dex-teleop project.
