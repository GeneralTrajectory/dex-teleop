# Re-Engagement Tuning Guide

## Overview

When you move outside the workspace and then return, the system performs a **smooth re-engagement** to prevent jolting the arm. The arm gradually ramps up from stopped to the new target position over multiple frames.

## How It Works

### 1. Leaving Workspace

When you violate workspace limits or reach an unreachable pose:
- Arm stops cleanly (switches to position mode, state 4)
- System prints violation reason
- Waits for you to return to valid workspace

### 2. Re-Entering Workspace

When you move back to valid workspace:
1. **Restart servo mode** (mode 1 for streaming)
2. **Smoothstep interpolation** over N frames
   - Frame 1: Very slow start (gentle acceleration)
   - Middle frames: Gradual ramp-up
   - Final frames: Smooth deceleration to target
3. **Resume normal control** after interpolation completes

### 3. Smoothstep Easing

Uses the formula: `alpha = 3t² - 2t³`

This provides:
- **Gentle start**: Acceleration begins gradually (safer for hardware)
- **Smooth end**: Deceleration to target (no overshoot)
- **Natural feel**: S-curve motion profile (better than linear)

## Configuration

### Default (Recommended for Demonstrations)

```bash
export TELEOP_REENGAGEMENT_STEPS="30"
python vive_teleop_xarm.py
```

**Result**: 0.3 seconds re-engagement at 100Hz
- Very smooth, safe for hardware
- Good balance between safety and responsiveness

### Extra Smooth (Maximum Safety)

```bash
export TELEOP_REENGAGEMENT_STEPS="50"
python vive_teleop_xarm.py
```

**Result**: 0.5 seconds re-engagement
- Extremely gentle on hardware
- Best for initial testing or when nervous
- Use if you see any jerking with default settings

### Faster (For Experienced Users)

```bash
export TELEOP_REENGAGEMENT_STEPS="20"
python vive_teleop_xarm.py
```

**Result**: 0.2 seconds re-engagement
- Quicker response after violations
- Still smoother than instant snap
- Use once you're comfortable with the system

### Very Fast (Not Recommended)

```bash
export TELEOP_REENGAGEMENT_STEPS="10"
python vive_teleop_xarm.py
```

**Result**: 0.1 seconds re-engagement
- Minimal delay
- May be abrupt on the hardware
- Only for advanced users

## Smoothstep vs Linear Comparison

### Linear Interpolation (Old)
```
Speed │     /────────
      │    /
      │   /
      │  /
      │ /
      │/
      └──────────────> Time
      Start        End
```
**Problem**: Instant acceleration/deceleration = jerky motion

### Smoothstep (New)
```
Speed │      ╭────────╮
      │     ╱          ╲
      │    ╱            ╲
      │   ╱              ╲
      │  ╱                ╲
      │ ╱                  ╲
      └──────────────────────> Time
      Start              End
```
**Benefit**: Gradual acceleration/deceleration = smooth motion

## Mathematical Details

**Smoothstep function:**
```
t = frame_number / total_frames  (0.0 to 1.0)
alpha = 3t² - 2t³

At t=0.0: alpha=0.0, velocity=0    (stopped)
At t=0.5: alpha=0.5, velocity=max  (midpoint, full speed)
At t=1.0: alpha=1.0, velocity=0    (reached target, decelerated)
```

**Derivative (velocity profile):**
```
v(t) = 6t - 6t²

Peaks at t=0.5, zero at t=0 and t=1
Creates the S-curve acceleration/deceleration
```

## Tuning Recommendations

### For Different Scenarios

| Scenario | Steps | Duration | Rationale |
|----------|-------|----------|-----------|
| Initial testing | 50 | 0.5s | Maximum safety, minimal stress |
| Normal demos | 30 | 0.3s | Balanced, feels natural |
| Experienced user | 20 | 0.2s | Responsive, still safe |
| Quick response | 10 | 0.1s | Minimal delay, use with caution |

### Signs You Need More Steps

- Arm jerks or jolts during re-engagement
- Audible "thunk" sound from joints
- Visible shaking or vibration
- → **Increase TELEOP_REENGAGEMENT_STEPS**

### Signs You Can Use Fewer Steps

- Re-engagement feels sluggish
- Too much delay after returning to workspace
- You're experienced and comfortable with the system
- → **Decrease TELEOP_REENGAGEMENT_STEPS**

## Example Session

```bash
# Start with extra smooth for safety
export TELEOP_REENGAGEMENT_STEPS="50"
python vive_teleop_xarm.py

# After testing, reduce to default
export TELEOP_REENGAGEMENT_STEPS="30"
python vive_teleop_xarm.py

# Once comfortable, try faster
export TELEOP_REENGAGEMENT_STEPS="20"
python vive_teleop_xarm.py
```

## Technical Notes

### Why Not Just Use Lower Speed?

Setting a lower speed in `set_servo_cartesian()` would slow down **all** motion, not just re-engagement. The interpolation approach allows:
- Fast response during normal operation (100Hz)
- Smooth, gradual re-engagement only when needed
- Best of both worlds!

### Frame Count vs Duration

At 100Hz control rate:
- 10 frames = 0.1 seconds
- 20 frames = 0.2 seconds
- 30 frames = 0.3 seconds (default)
- 50 frames = 0.5 seconds
- 100 frames = 1.0 second

If you change `TELEOP_RATE_HZ`, adjust steps proportionally:
```bash
# 50Hz control rate
export TELEOP_RATE_HZ="50"
export TELEOP_REENGAGEMENT_STEPS="15"  # Still 0.3s duration
```

## Summary

**Current settings (default):**
- **30 interpolation steps**
- **Smoothstep easing** (S-curve, not linear)
- **0.3 second duration** at 100Hz
- **Very safe and smooth** for hardware

This should be gentle enough to prevent any damage while still feeling responsive. If you need it even smoother, increase to 50 steps for 0.5 seconds of gradual re-engagement.

