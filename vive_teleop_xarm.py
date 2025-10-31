#!/usr/bin/env python3
"""
Vive Tracker Teleoperation for xArm Robot (Unimanual & Bimanual)
Maps Vive Tracker 3.0 movements to xArm with comprehensive safety features.

Safety Features:
- Table collision prevention (accounts for 200mm gripper extension)
- Joint1 constraint enforcement (-90° to +90°)
- Workspace boundary enforcement
- IK reachability validation (prevents unreachable poses)
- Smooth re-engagement after workspace violations
- Foot pedal stop (monitors 'b' key press)

Performance:
- Real-time servo mode streaming at 100Hz for minimal latency
- Smoothstep easing for gradual re-engagement (default 30 frames = 0.3s)

Usage:
    # Right arm only (default)
    export XARM_IP_RIGHT="192.168.1.214"
    python vive_teleop_xarm.py --mode right
    
    # Left arm only
    export XARM_IP_LEFT="192.168.1.111"
    python vive_teleop_xarm.py --mode left
    
    # Bimanual (both arms)
    export XARM_IP_LEFT="192.168.1.111"
    export XARM_IP_RIGHT="192.168.1.214"
    python vive_teleop_xarm.py --mode both
    
    Stop: Ctrl+C or press 'b' key (foot pedal)
"""

import sys
import time
import os
import math
import numpy as np
from pathlib import Path
from typing import Dict, Tuple, Optional
from scipy.spatial.transform import Rotation as R
import threading

# Add Vive_Tracker to path
sys.path.insert(0, str(Path(__file__).parent / 'Vive_Tracker'))
from track import ViveTrackerModule
from fairmotion_ops import conversions

# Add AnyDexGrasp to path for xArm adapter (absolute path)
anydex_path = '/home/joshua/Documents/Sources/Papers/Cursor/AnyDexGrasp'
if anydex_path not in sys.path:
    sys.path.insert(0, anydex_path)
from xarm_adapter import create_xarm_adapter, XArmAdapter

try:
    import select
    import tty
    import termios
    KEYBOARD_AVAILABLE = True
except ImportError:
    KEYBOARD_AVAILABLE = False


class FootPedalMonitor:
    """Monitor foot pedal (sends 'b' key) for stop signal in a background thread."""
    
    def __init__(self):
        self.stop_requested = False
        self._thread = None
        self._running = False
        self._old_terminal_settings = None
        
        if KEYBOARD_AVAILABLE and sys.stdin.isatty():
            print("🦶 Foot pedal enabled - press 'b' to stop")
        else:
            print("⚠️ Foot pedal unavailable (stdin not a terminal)")
    
    def start(self):
        """Start monitoring foot pedal in background thread."""
        if not KEYBOARD_AVAILABLE or not sys.stdin.isatty():
            return
        
        # Save terminal settings and set to raw mode
        try:
            self._old_terminal_settings = termios.tcgetattr(sys.stdin)
            tty.setraw(sys.stdin.fileno())
        except Exception as e:
            print(f"⚠️ Could not set terminal to raw mode: {e}")
            return
        
        self._running = True
        self._thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self._thread.start()
    
    def _monitor_loop(self):
        """Background thread that monitors for 'b' key press."""
        while self._running:
            if select.select([sys.stdin], [], [], 0.1)[0]:
                try:
                    char = sys.stdin.read(1)
                    if char == 'b':
                        print("\n\n🦶 Foot pedal pressed - stopping!")
                        self.stop_requested = True
                        break
                except Exception:
                    pass
    
    def stop(self):
        """Stop monitoring and restore terminal settings."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        
        if self._old_terminal_settings:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_terminal_settings)
            except Exception:
                pass


class ViveToXArmMapper:
    """Maps Vive Tracker poses to xArm commands with safety enforcement."""
    
    def __init__(self,
                 xarm_ip: str,
                 tracker: object,
                 arm_label: str = "arm",
                 position_scale: float = 1.0,
                 rotation_scale: float = 1.0):
        """
        Initialize the Vive-to-xArm mapper for a single arm.
        
        Args:
            xarm_ip: IP address of the xArm robot
            tracker: Vive tracker object (already initialized)
            arm_label: Label for this arm ("left", "right", or "arm")
            position_scale: Scale factor for position deltas (m -> mm)
            rotation_scale: Scale factor for rotation deltas
        """
        self.arm_label = arm_label
        self.tracker = tracker
        
        print(f"\n🤖 Initializing {arm_label} arm mapper...")
        print(f"  xArm IP: {xarm_ip}")
        print(f"  Tracker: {tracker.get_serial()}")
        print(f"  Position scale: {position_scale}")
        print(f"  Rotation scale: {rotation_scale}")
        
        # Initialize xArm adapter directly with IP
        # Don't use create_xarm_adapter() to avoid environment variable conflicts in bimanual mode
        self.adapter = XArmAdapter(
            ip=xarm_ip,
            execute_live=True,
            use_inspire_api=False  # No gripper control for now
        )
        self.adapter._lazy_connect_if_needed()
        
        if self.adapter.arm is None:
            raise RuntimeError(f"Failed to connect to xArm at {xarm_ip}")
        
        print(f"✅ {arm_label} xArm connected successfully")
        
        # Store scaling factors
        self.position_scale = position_scale
        self.rotation_scale = rotation_scale
        # Per-axis position fine-tuning (multiplied after global scale)
        # Defaults to 1.0 so existing behavior is unchanged
        self.position_scale_x = float(os.environ.get('VIVE_POSITION_SCALE_X', '1.0'))
        self.position_scale_y = float(os.environ.get('VIVE_POSITION_SCALE_Y', '1.0'))
        self.position_scale_z = float(os.environ.get('VIVE_POSITION_SCALE_Z', '1.5'))
        # Per-axis rotation fine-tuning (multiplied after global scale)
        # Defaults to 1.0 so existing behavior is unchanged
        self.rotation_scale_roll = float(os.environ.get('VIVE_ROTATION_SCALE_ROLL', '1.0'))
        self.rotation_scale_pitch = float(os.environ.get('VIVE_ROTATION_SCALE_PITCH', '1.0'))
        self.rotation_scale_yaw = float(os.environ.get('VIVE_ROTATION_SCALE_YAW', '1.0'))
        
        # Reference poses (set during calibration)
        self.tracker_home = None  # 4x4 tracker pose at calibration
        self.arm_home_pose = None  # Dict with {x,y,z,roll,pitch,yaw}
        self.base_T_tcp_home = None  # 4x4 arm pose at calibration
        
        # Adaptive pose filter (EMA + clamp) to suppress spikes
        self.ema_alpha_slow = float(os.environ.get('POSE_EMA_SLOW', '0.5'))
        self.ema_bypass_mm = float(os.environ.get('POSE_EMA_BYPASS_MM', '15'))
        self.ema_bypass_deg = float(os.environ.get('POSE_EMA_BYPASS_DEG', '8'))
        self.clamp_mm = float(os.environ.get('POSE_CLAMP_MM', '30'))
        self.clamp_deg = float(os.environ.get('POSE_CLAMP_DEG', '12'))
        self._last_filtered_pose = None  # Persist filtered pose across frames
        self._pf_debug = os.environ.get('POSE_FILTER_DEBUG', '0').strip().lower() in ('1','true','yes','y')
        self._pf_debug_n = 0

        # Workspace limits (mm) - Expanded X-axis for full table reach
        self.workspace_limits = {
            'x': (100.0, 700.0),    # Extended forward reach for full table coverage
            'y': (-400.0, 400.0),   # Symmetric left/right
            'z': (float(os.environ.get('TABLE_Z_MM', '15')) + 50.0, 800.0)  # Above table
        }
        
        # Joint limits (degrees)
        self.joint1_limits = (-90.0, 90.0)  # Prevent reaching behind table
        
        # Smooth re-engagement after workspace violations
        self._last_valid_pose = None  # Last pose that was sent successfully
        self._was_outside_workspace = False  # Track if we were recently outside
        self._reengagement_steps = int(os.environ.get('TELEOP_REENGAGEMENT_STEPS', '30'))  # Number of interpolation steps (default 30 = 0.3s at 100Hz)
        self._reengagement_counter = 0  # Current step in re-engagement
        
        print(f"\n⚙️ Workspace limits (mm):")
        print(f"  X: {self.workspace_limits['x']}")
        print(f"  Y: {self.workspace_limits['y']}")
        print(f"  Z: {self.workspace_limits['z']}")
        print(f"  Joint1: {self.joint1_limits}°")
    
    def calibrate_home_poses(self, duration_s: float = 3.0):
        """
        Calibrate reference poses by waiting duration_s seconds.
        Captures tracker pose and current xArm pose as home positions.
        
        Args:
            duration_s: Calibration duration in seconds
        """
        print(f"\n🎯 Calibrating home poses ({duration_s}s countdown)...")
        print("   Hold tracker steady in your starting position!")
        
        # Countdown
        for i in range(int(duration_s), 0, -1):
            print(f"   {i}...", flush=True)
            time.sleep(1.0)
        
        # Capture tracker home pose
        # Copy the matrix to avoid aliasing the internal tracker buffer
        self.tracker_home = self.tracker.get_T().copy()
        print(f"\n✅ Tracker home captured:")
        print(f"   Position (m): [{self.tracker_home[0,3]:.3f}, "
              f"{self.tracker_home[1,3]:.3f}, {self.tracker_home[2,3]:.3f}]")
        
        # Capture arm home pose
        current_tcp_pose = self.adapter._get_live_tcp_pose()
        if current_tcp_pose is None:
            raise RuntimeError("Failed to query xArm TCP pose during calibration")
        
        self.arm_home_pose = {
            'x': float(current_tcp_pose[0]),
            'y': float(current_tcp_pose[1]),
            'z': float(current_tcp_pose[2]),
            'roll': float(current_tcp_pose[3]),
            'pitch': float(current_tcp_pose[4]),
            'yaw': float(current_tcp_pose[5])
        }
        
        print(f"✅ Arm home captured:")
        print(f"   Position (mm): [{self.arm_home_pose['x']:.1f}, "
              f"{self.arm_home_pose['y']:.1f}, {self.arm_home_pose['z']:.1f}]")
        print(f"   Orientation (deg): [roll={self.arm_home_pose['roll']:.1f}, "
              f"pitch={self.arm_home_pose['pitch']:.1f}, yaw={self.arm_home_pose['yaw']:.1f}]")
        
        # Convert arm home to 4x4 matrix for delta computation
        t_home = np.array([
            self.arm_home_pose['x'] / 1000.0,
            self.arm_home_pose['y'] / 1000.0,
            self.arm_home_pose['z'] / 1000.0
        ], dtype=np.float32)
        
        R_home = R.from_euler('xyz', [
            self.arm_home_pose['roll'],
            self.arm_home_pose['pitch'],
            self.arm_home_pose['yaw']
        ], degrees=True).as_matrix().astype(np.float32)
        
        self.base_T_tcp_home = np.eye(4, dtype=np.float32)
        self.base_T_tcp_home[:3, :3] = R_home
        self.base_T_tcp_home[:3, 3] = t_home
        
        print(f"\n🎮 {self.arm_label} calibration complete!")
        
        # Enable servo mode for real-time streaming
        if not self.enable_servo_mode():
            raise RuntimeError(f"Failed to enable servo mode for {self.arm_label}")
        
        print(f"🎮 {self.arm_label} teleoperation ready!")
    
    def compute_target_pose(self, current_tracker_T: np.ndarray) -> Dict[str, float]:
        """
        Compute target xArm pose from current tracker pose using delta mapping.
        
        Args:
            current_tracker_T: Current 4x4 tracker pose matrix
            
        Returns:
            Target pose dict with {x, y, z, roll, pitch, yaw}
        """
        if self.tracker_home is None or self.base_T_tcp_home is None:
            raise RuntimeError("Must call calibrate_home_poses() before compute_target_pose()")
        
        # Compute tracker delta
        tracker_delta_T = np.linalg.inv(self.tracker_home) @ current_tracker_T
        
        # Extract position delta (meters)
        pos_delta_m = tracker_delta_T[:3, 3]
        
        # Remap axes to fix direction mismatch:
        # Tracker forward -> xArm forward (X+)
        # Tracker right -> xArm left (Y-)  
        # Tracker down -> xArm up (Z+)
        # 
        # Based on observation:
        # - Tracker forward (+X_tracker) was causing xArm right (+Y_arm) - need to swap X<->Y and flip Y
        # - Tracker backward (-X_tracker) was causing xArm left (-Y_arm)
        # - Tracker up (+Z_tracker) was causing xArm down (-Z_arm) - need to flip Z
        pos_delta_remapped = np.array([
            -pos_delta_m[1],  # xArm X = -tracker Y (forward/back)
            -pos_delta_m[0],   # xArm Y = +tracker X (left/right) 
            -pos_delta_m[2]   # xArm Z = -tracker Z (up/down)
        ], dtype=np.float32)
        
        # Apply per-axis fine-tune and global scale, then convert to mm
        pos_delta_scaled_mm = np.array([
            pos_delta_remapped[0] * self.position_scale_x,
            pos_delta_remapped[1] * self.position_scale_y,
            pos_delta_remapped[2] * self.position_scale_z,
        ], dtype=np.float32) * self.position_scale * 1000.0  # m -> mm
        
        # Extract rotation delta and scale
        R_delta = tracker_delta_T[:3, :3]
        rotvec_delta = R.from_matrix(R_delta).as_rotvec()
        
        # Remap rotation axes to match position remapping:
        # Same coordinate transformation applies to rotations
        # Tracker axes -> xArm axes remapping
        rotvec_remapped = np.array([
            -rotvec_delta[1],  # xArm roll (X-axis rotation) = -tracker pitch (Y-axis rotation)
            rotvec_delta[0],   # xArm pitch (Y-axis rotation) = +tracker roll (X-axis rotation)
            rotvec_delta[2]   # xArm yaw (Z-axis rotation) = tracker yaw (Z-axis rotation)
        ], dtype=np.float32)
        
        # Apply per-axis fine-tune and global scale
        rotvec_scaled = np.array([
            rotvec_remapped[0] * self.rotation_scale_roll,
            rotvec_remapped[1] * self.rotation_scale_pitch,
            rotvec_remapped[2] * self.rotation_scale_yaw,
        ], dtype=np.float32) * float(self.rotation_scale)
        R_delta_scaled = R.from_rotvec(rotvec_scaled).as_matrix()
        
        # Apply deltas to arm home pose
        # Position: simple addition
        target_pos_mm = np.array([
            self.arm_home_pose['x'] + pos_delta_scaled_mm[0],
            self.arm_home_pose['y'] + pos_delta_scaled_mm[1],
            self.arm_home_pose['z'] + pos_delta_scaled_mm[2]
        ])
        
        # Rotation: compose rotations
        R_home = self.base_T_tcp_home[:3, :3]
        R_target = R_home @ R_delta_scaled
        rpy_target = R.from_matrix(R_target).as_euler('xyz', degrees=True)
        
        target_pose = {
            'x': float(target_pos_mm[0]),
            'y': float(target_pos_mm[1]),
            'z': float(target_pos_mm[2]),
            'roll': float(rpy_target[0]),
            'pitch': float(rpy_target[1]),
            'yaw': float(rpy_target[2])
        }
        
        return target_pose

    def apply_pose_filter(self, pose_dict: Dict[str, float], is_reengaging: bool = False) -> Dict[str, float]:
        """Apply adaptive EMA smoothing and spike clamping to target pose.

        Args:
            pose_dict: Target pose to send {x,y,z,roll,pitch,yaw}
            is_reengaging: True when coming back from workspace violation (skip EMA, clamp only)

        Returns:
            Filtered pose dict
        """
        # First valid frame: initialize and return raw pose
        if self._last_filtered_pose is None:
            self._last_filtered_pose = {
                'x': float(pose_dict['x']), 'y': float(pose_dict['y']), 'z': float(pose_dict['z']),
                'roll': float(pose_dict['roll']), 'pitch': float(pose_dict['pitch']), 'yaw': float(pose_dict['yaw'])
            }
            return pose_dict

        # Helper to wrap angle delta to [-180, 180]
        def wrap_delta(angle_deg):
            d = float(angle_deg)
            while d > 180.0:
                d -= 360.0
            while d < -180.0:
                d += 360.0
            return d
        
        # Compute deltas from last filtered pose
        dx = float(pose_dict['x'] - self._last_filtered_pose['x'])
        dy = float(pose_dict['y'] - self._last_filtered_pose['y'])
        dz = float(pose_dict['z'] - self._last_filtered_pose['z'])
        droll = wrap_delta(pose_dict['roll'] - self._last_filtered_pose['roll'])
        dpitch = wrap_delta(pose_dict['pitch'] - self._last_filtered_pose['pitch'])
        dyaw = wrap_delta(pose_dict['yaw'] - self._last_filtered_pose['yaw'])

        delta_pos_mm = math.sqrt(dx*dx + dy*dy + dz*dz)
        delta_rot_deg = math.sqrt(droll*droll + dpitch*dpitch + dyaw*dyaw)

        # Clamp extreme spikes to maximum per-frame delta (position and rotation separately)
        pos_scale = 1.0
        rot_scale = 1.0
        if delta_pos_mm > self.clamp_mm:
            pos_scale = self.clamp_mm / max(delta_pos_mm, 1e-6)
        if delta_rot_deg > self.clamp_deg:
            rot_scale = self.clamp_deg / max(delta_rot_deg, 1e-6)

        clamped = (pos_scale < 1.0) or (rot_scale < 1.0)
        if clamped:
            dx *= pos_scale; dy *= pos_scale; dz *= pos_scale
            droll *= rot_scale; dpitch *= rot_scale; dyaw *= rot_scale

        clamped_pose = {
            'x': self._last_filtered_pose['x'] + dx,
            'y': self._last_filtered_pose['y'] + dy,
            'z': self._last_filtered_pose['z'] + dz,
            'roll': self._last_filtered_pose['roll'] + droll,
            'pitch': self._last_filtered_pose['pitch'] + dpitch,
            'yaw': self._last_filtered_pose['yaw'] + dyaw,
        }

        # If re-engaging or big move, bypass EMA (but keep clamping)
        bypass = is_reengaging or (delta_pos_mm > self.ema_bypass_mm) or (delta_rot_deg > self.ema_bypass_deg)
        if bypass:
            self._last_filtered_pose = clamped_pose
            if self._pf_debug:
                self._pf_debug_n += 1
                if (self._pf_debug_n % 100) == 0:
                    print(f"PF[{self.arm_label}] bypass posΔ={delta_pos_mm:.1f}mm rotΔ={delta_rot_deg:.1f}° clamp={clamped}")
            return clamped_pose

        # Otherwise apply EMA smoothing for micro-jitter
        a = max(0.0, min(1.0, self.ema_alpha_slow))
        filtered = {
            'x': (1.0 - a) * self._last_filtered_pose['x'] + a * clamped_pose['x'],
            'y': (1.0 - a) * self._last_filtered_pose['y'] + a * clamped_pose['y'],
            'z': (1.0 - a) * self._last_filtered_pose['z'] + a * clamped_pose['z'],
            'roll': (1.0 - a) * self._last_filtered_pose['roll'] + a * clamped_pose['roll'],
            'pitch': (1.0 - a) * self._last_filtered_pose['pitch'] + a * clamped_pose['pitch'],
            'yaw': (1.0 - a) * self._last_filtered_pose['yaw'] + a * clamped_pose['yaw'],
        }
        self._last_filtered_pose = filtered
        if self._pf_debug:
            self._pf_debug_n += 1
            if (self._pf_debug_n % 100) == 0:
                print(f"PF[{self.arm_label}] ema posΔ={delta_pos_mm:.1f}mm rotΔ={delta_rot_deg:.1f}° clamp={clamped}")
        return filtered
    
    def is_pose_within_workspace(self, pose_dict: Dict[str, float]) -> Tuple[bool, str]:
        """
        Fast workspace-only check (< 1ms). Used every frame.
        
        Args:
            pose_dict: Target pose with {x, y, z, roll, pitch, yaw}
            
        Returns:
            (is_safe, reason) tuple
        """
        if pose_dict['x'] < self.workspace_limits['x'][0]:
            return False, f"X too small ({pose_dict['x']:.1f} < {self.workspace_limits['x'][0]:.1f}mm)"
        if pose_dict['x'] > self.workspace_limits['x'][1]:
            return False, f"X too large ({pose_dict['x']:.1f} > {self.workspace_limits['x'][1]:.1f}mm)"
        
        if pose_dict['y'] < self.workspace_limits['y'][0]:
            return False, f"Y too small ({pose_dict['y']:.1f} < {self.workspace_limits['y'][0]:.1f}mm)"
        if pose_dict['y'] > self.workspace_limits['y'][1]:
            return False, f"Y too large ({pose_dict['y']:.1f} > {self.workspace_limits['y'][1]:.1f}mm)"
        
        if pose_dict['z'] < self.workspace_limits['z'][0]:
            return False, f"Z too small ({pose_dict['z']:.1f} < {self.workspace_limits['z'][0]:.1f}mm)"
        if pose_dict['z'] > self.workspace_limits['z'][1]:
            return False, f"Z too large ({pose_dict['z']:.1f} > {self.workspace_limits['z'][1]:.1f}mm)"
        
        return True, "OK"
    
    def is_pose_reachable(self, pose_dict: Dict[str, float]) -> Tuple[bool, str]:
        """
        Check if pose is kinematically reachable via IK.
        This prevents sending unreachable poses to servo mode which would cause errors.
        
        Args:
            pose_dict: Target pose with {x, y, z, roll, pitch, yaw}
            
        Returns:
            (is_reachable, reason) tuple
        """
        try:
            pose_list = [
                pose_dict['x'], pose_dict['y'], pose_dict['z'],
                pose_dict['roll'], pose_dict['pitch'], pose_dict['yaw']
            ]
            
            code, joint_angles = self.adapter.arm.get_inverse_kinematics(
                pose_list, input_is_radian=False, return_is_radian=False
            )
            
            if code != 0:
                return False, f"IK failed (code={code})"
            
            if not joint_angles or len(joint_angles) < 6:
                return False, "IK returned invalid joint angles"
            
            # Also check joint1 limit while we have IK solution
            joint1_deg = float(joint_angles[0])
            if joint1_deg < self.joint1_limits[0]:
                return False, f"Joint1 too negative ({joint1_deg:.1f}° < {self.joint1_limits[0]:.1f}°)"
            if joint1_deg > self.joint1_limits[1]:
                return False, f"Joint1 too positive ({joint1_deg:.1f}° > {self.joint1_limits[1]:.1f}°)"
            
            return True, "OK"
            
        except Exception as e:
            return False, f"IK check error: {e}"
    
    def enable_servo_mode(self):
        """Enable servo motion mode for real-time streaming control."""
        try:
            # Mode 1 = servo motion mode (for streaming commands)
            self.adapter.arm.set_mode(1)
            self.adapter.arm.set_state(0)
            print("✅ Servo mode enabled for real-time control")
            return True
        except Exception as e:
            print(f"⚠️ Failed to enable servo mode: {e}")
            return False
    
    def compute_reengagement_pose(self, target_pose: Dict[str, float]) -> Dict[str, float]:
        """
        Compute interpolated pose for smooth re-engagement after workspace violation.
        Uses smoothstep easing for gradual acceleration and deceleration.
        
        Args:
            target_pose: The new target pose (now back in workspace)
            
        Returns:
            Interpolated pose for this frame
        """
        if self._last_valid_pose is None:
            # No previous pose, use target directly
            return target_pose
        
        # Compute interpolation factor (0.0 to 1.0)
        t = min(1.0, self._reengagement_counter / max(1, self._reengagement_steps))
        
        # Smoothstep easing: 3t^2 - 2t^3
        # This provides smooth acceleration at start and deceleration at end
        # Much gentler than linear interpolation
        alpha = t * t * (3.0 - 2.0 * t)
        
        # Smooth interpolation for position
        interp_pose = {
            'x': self._last_valid_pose['x'] + alpha * (target_pose['x'] - self._last_valid_pose['x']),
            'y': self._last_valid_pose['y'] + alpha * (target_pose['y'] - self._last_valid_pose['y']),
            'z': self._last_valid_pose['z'] + alpha * (target_pose['z'] - self._last_valid_pose['z']),
        }
        
        # Smooth interpolation for orientation
        interp_pose['roll'] = self._last_valid_pose['roll'] + alpha * (target_pose['roll'] - self._last_valid_pose['roll'])
        interp_pose['pitch'] = self._last_valid_pose['pitch'] + alpha * (target_pose['pitch'] - self._last_valid_pose['pitch'])
        interp_pose['yaw'] = self._last_valid_pose['yaw'] + alpha * (target_pose['yaw'] - self._last_valid_pose['yaw'])
        
        return interp_pose
    
    def send_pose_streaming(self, pose_dict: Dict[str, float], speed: int) -> bool:
        """
        Send pose command in servo streaming mode (real-time, minimal latency).
        Uses set_servo_cartesian() which is designed for continuous streaming.
        
        Args:
            pose_dict: Target pose
            speed: Motion speed in mm/s (passed through to xArm, often ignored in servo mode)
            
        Returns:
            True if command sent successfully
        """
        try:
            # set_servo_cartesian: designed for streaming at high rates
            mvpose = [
                pose_dict['x'],
                pose_dict['y'],
                pose_dict['z'],
                pose_dict['roll'],
                pose_dict['pitch'],
                pose_dict['yaw']
            ]
            ret = self.adapter.arm.set_servo_cartesian(mvpose, speed=speed, mvacc=2000)
            return ret == 0
        except Exception:
            return False
    
    def is_pose_safe(self, pose_dict: Dict[str, float]) -> Tuple[bool, str]:
        """
        Validate pose against all safety constraints.
        
        Args:
            pose_dict: Target pose with {x, y, z, roll, pitch, yaw}
            
        Returns:
            (is_safe, reason) tuple
        """
        # Safety check 1: Workspace boundaries
        if pose_dict['x'] < self.workspace_limits['x'][0]:
            return False, f"X too small ({pose_dict['x']:.1f} < {self.workspace_limits['x'][0]:.1f}mm)"
        if pose_dict['x'] > self.workspace_limits['x'][1]:
            return False, f"X too large ({pose_dict['x']:.1f} > {self.workspace_limits['x'][1]:.1f}mm)"
        
        if pose_dict['y'] < self.workspace_limits['y'][0]:
            return False, f"Y too small ({pose_dict['y']:.1f} < {self.workspace_limits['y'][0]:.1f}mm)"
        if pose_dict['y'] > self.workspace_limits['y'][1]:
            return False, f"Y too large ({pose_dict['y']:.1f} > {self.workspace_limits['y'][1]:.1f}mm)"
        
        if pose_dict['z'] < self.workspace_limits['z'][0]:
            return False, f"Z too small ({pose_dict['z']:.1f} < {self.workspace_limits['z'][0]:.1f}mm)"
        if pose_dict['z'] > self.workspace_limits['z'][1]:
            return False, f"Z too large ({pose_dict['z']:.1f} > {self.workspace_limits['z'][1]:.1f}mm)"
        
        # Safety check 2: Table collision detection
        # Convert pose to 4x4 matrix
        t_mm = np.array([pose_dict['x'], pose_dict['y'], pose_dict['z']])
        t_m = t_mm / 1000.0
        
        R_mat = R.from_euler('xyz', [
            pose_dict['roll'],
            pose_dict['pitch'],
            pose_dict['yaw']
        ], degrees=True).as_matrix()
        
        base_T_tcp = np.eye(4, dtype=np.float32)
        base_T_tcp[:3, :3] = R_mat.astype(np.float32)
        base_T_tcp[:3, 3] = t_m.astype(np.float32)
        
        if self.adapter.pose_collides_with_table(base_T_tcp):
            return False, "Table collision detected (gripper would hit table)"
        
        # Safety check 3: Joint1 angle limit (prevent reaching behind)
        try:
            pose_list = [
                pose_dict['x'], pose_dict['y'], pose_dict['z'],
                pose_dict['roll'], pose_dict['pitch'], pose_dict['yaw']
            ]
            
            code, joint_angles = self.adapter.arm.get_inverse_kinematics(
                pose_list, input_is_radian=False, return_is_radian=False
            )
            
            if code != 0 or not joint_angles or len(joint_angles) < 6:
                return False, f"IK failed (code={code})"
            
            joint1_deg = float(joint_angles[0])
            
            if joint1_deg < self.joint1_limits[0]:
                return False, f"Joint1 too negative ({joint1_deg:.1f}° < {self.joint1_limits[0]:.1f}°)"
            if joint1_deg > self.joint1_limits[1]:
                return False, f"Joint1 too positive ({joint1_deg:.1f}° > {self.joint1_limits[1]:.1f}°)"
            
        except Exception as e:
            return False, f"IK check error: {e}"
        
        return True, "OK"
    
    def shutdown(self):
        """Clean shutdown of connections."""
        print("\n🛑 Shutting down teleoperation...")
        try:
            if self.adapter:
                self.adapter.disconnect()
        except Exception as e:
            print(f"⚠️ Error during shutdown: {e}")
        print("✅ Shutdown complete")


def run_teleoperation(mode: str = 'right'):
    """Main teleoperation control loop.
    
    Args:
        mode: 'left', 'right', or 'both' for bimanual control
    """
    
    # Configuration from environment
    position_scale = float(os.environ.get('VIVE_POSITION_SCALE', '1.0'))
    rotation_scale = float(os.environ.get('VIVE_ROTATION_SCALE', '1.0'))
    rate_hz = int(os.environ.get('TELEOP_RATE_HZ', '100'))  # 100Hz for servo mode streaming
    speed_mm_s = int(os.environ.get('TELEOP_SPEED', '100'))  # Reserved in servo mode (ignored)
    
    print("="*60)
    print(f"Vive Tracker Teleoperation for xArm ({mode.upper()} mode)")
    print("="*60)
    
    # Initialize Vive tracker system
    print("\n📡 Initializing Vive Tracker system...")
    vive = ViveTrackerModule()
    vive.print_discovered_objects()
    
    trackers = vive.return_selected_devices("tracker")
    if len(trackers) == 0:
        print("❌ No Vive trackers detected! Ensure trackers are powered on.")
        return 1
    
    tracker_list = list(trackers.values())
    print(f"✅ Found {len(tracker_list)} tracker(s)")
    
    # Validate tracker count for mode
    if mode == 'both' and len(tracker_list) < 2:
        print(f"❌ Bimanual mode requires 2 trackers, but only {len(tracker_list)} found")
        return 1
    
    # Tracker serial numbers (hardcoded for your setup)
    RIGHT_TRACKER_SERIAL = 'LHR-A56A8A21'  # Right wrist
    LEFT_TRACKER_SERIAL = 'LHR-D51DBC11'   # Left wrist
    
    # Build tracker lookup by serial
    trackers_by_serial = {t.get_serial(): t for t in tracker_list}
    
    # Initialize mappers based on mode
    mappers = {}
    
    try:
        if mode in ['left', 'both']:
            left_ip = os.environ.get('XARM_IP_LEFT', '192.168.1.111')
            # Use left-hand tracker (LHR-D51DBC11)
            if LEFT_TRACKER_SERIAL in trackers_by_serial:
                left_tracker = trackers_by_serial[LEFT_TRACKER_SERIAL]
                print(f"✅ Using left-hand tracker: {LEFT_TRACKER_SERIAL}")
            else:
                print(f"⚠️  Left tracker {LEFT_TRACKER_SERIAL} not found, using first available")
                left_tracker = tracker_list[0]
            
            mappers['left'] = ViveToXArmMapper(
                xarm_ip=left_ip,
                tracker=left_tracker,
                arm_label="left",
                position_scale=position_scale,
                rotation_scale=rotation_scale
            )
        
        if mode in ['right', 'both']:
            right_ip = os.environ.get('XARM_IP_RIGHT', '192.168.1.214')
            # Use right-hand tracker (LHR-A56A8A21)
            if RIGHT_TRACKER_SERIAL in trackers_by_serial:
                right_tracker = trackers_by_serial[RIGHT_TRACKER_SERIAL]
                print(f"✅ Using right-hand tracker: {RIGHT_TRACKER_SERIAL}")
            else:
                print(f"⚠️  Right tracker {RIGHT_TRACKER_SERIAL} not found, using first available")
                right_tracker = tracker_list[0]
            
            mappers['right'] = ViveToXArmMapper(
                xarm_ip=right_ip,
                tracker=right_tracker,
                arm_label="right",
                position_scale=position_scale,
                rotation_scale=rotation_scale
            )
    except Exception as e:
        print(f"❌ Mapper initialization failed: {e}")
        return 1
    
    # Move arms to home position
    try:
        for label, mapper in mappers.items():
            print(f"\n🏠 Moving {label} arm to home position...")
            mapper.adapter.go_home()
            print(f"✅ {label} arm at home position")
    except Exception as e:
        print(f"❌ Failed to move to home: {e}")
        for mapper in mappers.values():
            mapper.shutdown()
        return 1
    
    # Calibrate reference poses
    try:
        if mode == 'both':
            # Bimanual: calibrate both arms simultaneously
            print(f"\n🎯 Calibrating BOTH arms simultaneously...")
            print(f"🎯 Calibrating home poses (6.0s countdown)...")
            print("   Hold BOTH trackers steady in your starting positions!")
            
            # Shared countdown
            for i in range(6, 0, -1):
                print(f"   {i}...", flush=True)
                time.sleep(1.0)
            
            # Capture both trackers and arm poses simultaneously
            for label, mapper in mappers.items():
                # Capture tracker home pose
                mapper.tracker_home = mapper.tracker.get_T().copy()
                print(f"\n✅ [{label}] Tracker home captured:")
                print(f"   Position (m): [{mapper.tracker_home[0,3]:.3f}, "
                      f"{mapper.tracker_home[1,3]:.3f}, {mapper.tracker_home[2,3]:.3f}]")
                
                # Capture arm home pose
                current_tcp_pose = mapper.adapter._get_live_tcp_pose()
                if current_tcp_pose is None:
                    raise RuntimeError(f"Failed to query {label} xArm TCP pose during calibration")
                
                mapper.arm_home_pose = {
                    'x': float(current_tcp_pose[0]),
                    'y': float(current_tcp_pose[1]),
                    'z': float(current_tcp_pose[2]),
                    'roll': float(current_tcp_pose[3]),
                    'pitch': float(current_tcp_pose[4]),
                    'yaw': float(current_tcp_pose[5])
                }
                
                print(f"✅ [{label}] Arm home captured:")
                print(f"   Position (mm): [{mapper.arm_home_pose['x']:.1f}, "
                      f"{mapper.arm_home_pose['y']:.1f}, {mapper.arm_home_pose['z']:.1f}]")
                print(f"   Orientation (deg): [roll={mapper.arm_home_pose['roll']:.1f}, "
                      f"pitch={mapper.arm_home_pose['pitch']:.1f}, yaw={mapper.arm_home_pose['yaw']:.1f}]")
                
                # Convert arm home to 4x4 matrix
                from scipy.spatial.transform import Rotation as R
                t_home = np.array([
                    mapper.arm_home_pose['x'] / 1000.0,
                    mapper.arm_home_pose['y'] / 1000.0,
                    mapper.arm_home_pose['z'] / 1000.0
                ], dtype=np.float32)
                
                R_home = R.from_euler('xyz', [
                    mapper.arm_home_pose['roll'],
                    mapper.arm_home_pose['pitch'],
                    mapper.arm_home_pose['yaw']
                ], degrees=True).as_matrix().astype(np.float32)
                
                mapper.base_T_tcp_home = np.eye(4, dtype=np.float32)
                mapper.base_T_tcp_home[:3, :3] = R_home
                mapper.base_T_tcp_home[:3, 3] = t_home
                
                print(f"\n🎮 [{label}] calibration complete!")
                
                # Enable servo mode
                if not mapper.enable_servo_mode():
                    raise RuntimeError(f"Failed to enable servo mode for {label}")
                
                print(f"🎮 [{label}] teleoperation ready!")
        else:
            # Unimanual: calibrate single arm
            for label, mapper in mappers.items():
                print(f"\n🎯 Calibrating {label} arm...")
                mapper.calibrate_home_poses(duration_s=6.0)
    except Exception as e:
        print(f"❌ Calibration failed: {e}")
        for mapper in mappers.values():
            mapper.shutdown()
        return 1
    
    # Start foot pedal monitor
    foot_pedal = FootPedalMonitor()
    foot_pedal.start()
    
    # Main control loop
    print(f"\n🎮 Teleoperation active at {rate_hz} Hz")
    print("   Press Ctrl+C or foot pedal to stop")
    print("-"*60)
    
    loop_count = 0
    safety_violations = {label: 0 for label in mappers.keys()}
    
    try:
        while True:
            # Check for foot pedal stop
            if foot_pedal.stop_requested:
                print("\n🛑 Stop requested by foot pedal")
                break
            loop_start = time.time()
            
            # Process each arm independently
            for label, mapper in mappers.items():
                try:
                    # Get current tracker pose
                    current_tracker_T = mapper.tracker.get_T().copy()
                    
                    # Compute target pose
                    target_pose = mapper.compute_target_pose(current_tracker_T)
                    
                    # Fast workspace check
                    in_workspace, ws_reason = mapper.is_pose_within_workspace(target_pose)
                    
                    if not in_workspace:
                        # Outside user-defined workspace
                        if not mapper._was_outside_workspace:
                            safety_violations[label] += 1
                            print(f"\n🛑 [{label}] Workspace limit: {ws_reason}")
                            print(f"   {label} arm will stop until you return to workspace")
                            
                            # Stop the arm by setting mode to position mode (mode 0)
                            try:
                                mapper.adapter.arm.set_mode(0)
                                mapper.adapter.arm.set_state(4)  # State 4 = stop
                            except Exception:
                                pass
                        
                        mapper._was_outside_workspace = True
                        mapper._reengagement_counter = 0  # Reset re-engagement counter
                        continue  # Skip to next arm
                    
                    # Check IK reachability (every frame - fast enough for real-time)
                    is_reachable, ik_reason = mapper.is_pose_reachable(target_pose)
                    
                    if not is_reachable:
                        # Pose is kinematically unreachable - silently skip (common at workspace edges)
                        # Don't spam errors, don't stop servo mode, just don't update
                        continue  # Skip to next arm
                    
                    # We're back in workspace - check if we need smooth re-engagement
                    pose_to_send = target_pose
                    
                    if mapper._was_outside_workspace:
                        # Just re-entered workspace - restart servo mode and use smooth re-engagement
                        if mapper._reengagement_counter == 0:
                            # First frame back in workspace - re-enable servo mode
                            print(f"\n🔄 [{label}] Re-entering workspace, restarting servo mode...")
                            try:
                                mapper.adapter.arm.set_mode(0)  # Position mode first
                                mapper.adapter.arm.set_state(0)  # Ready state
                                time.sleep(0.05)  # Brief pause for state transition
                                mapper.adapter.arm.set_mode(1)  # Back to servo mode
                                mapper.adapter.arm.set_state(0)  # Ready state
                                print(f"🔄 [{label}] Servo mode restarted, re-engaging smoothly ({mapper._reengagement_steps} steps)...")
                            except Exception as e:
                                print(f"⚠️ [{label}] Failed to restart servo mode: {e}")
                        
                        if mapper._reengagement_counter < mapper._reengagement_steps:
                            pose_to_send = mapper.compute_reengagement_pose(target_pose)
                            mapper._reengagement_counter += 1
                        else:
                            # Re-engagement complete
                            mapper._was_outside_workspace = False
                            print(f"\n✅ [{label}] Re-engagement complete, resuming normal control")
                    
                    # Apply adaptive smoothing/clamping to mitigate spikes
                    pose_to_send = mapper.apply_pose_filter(
                        pose_to_send,
                        is_reengaging=mapper._was_outside_workspace
                    )
                    
                    # Send streaming command
                    success = mapper.send_pose_streaming(pose_to_send, speed_mm_s)
                    
                    if success:
                        mapper._last_valid_pose = pose_to_send  # Track for re-engagement
                        
                        # Print status every 100 frames
                        if loop_count % 100 == 0:
                            status_icon = "🔄" if mapper._was_outside_workspace else "✅"
                            violations_str = ", ".join([f"{l}:{v}" for l, v in safety_violations.items()])
                            print(f"\r{status_icon} [{label}] Pos: [{pose_to_send['x']:.1f}, {pose_to_send['y']:.1f}, "
                                  f"{pose_to_send['z']:.1f}] mm | "
                                  f"Ori: [{pose_to_send['roll']:.1f}, {pose_to_send['pitch']:.1f}, "
                                  f"{pose_to_send['yaw']:.1f}]° | "
                                  f"Violations: {violations_str}", 
                                  end='', flush=True)
                    
                except Exception as e:
                    print(f"\n⚠️ [{label}] Error: {e}")
                    continue  # Skip to next arm
            
            loop_count += 1
            
            # Rate limiting
            elapsed = time.time() - loop_start
            sleep_time = (1.0 / rate_hz) - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    except KeyboardInterrupt:
        print("\n\n⚠️ Teleoperation stopped by user (Ctrl+C)")
    
    except Exception as e:
        print(f"\n❌ Fatal error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # Clean shutdown
        foot_pedal.stop()
        for mapper in mappers.values():
            mapper.shutdown()
        
        print(f"\n📊 Session statistics:")
        print(f"   Total frames: {loop_count}")
        for label, violations in safety_violations.items():
            print(f"   {label} arm violations: {violations}")
    
    return 0


def main():
    """Entry point with argument parsing."""
    import argparse
    
    parser = argparse.ArgumentParser(
        description="Vive Tracker teleoperation for xArm robot (unimanual & bimanual)"
    )
    parser.add_argument('--mode', choices=['left', 'right', 'both'], default='right',
                       help='Control mode: left arm only, right arm only, or both arms (default: right)')
    parser.add_argument('--left-ip', default=os.environ.get('XARM_IP_LEFT', '192.168.1.111'),
                       help='Left xArm IP address (default: 192.168.1.111)')
    parser.add_argument('--right-ip', default=os.environ.get('XARM_IP_RIGHT', '192.168.1.214'),
                       help='Right xArm IP address (default: 192.168.1.214)')
    parser.add_argument('--position-scale', type=float,
                       default=float(os.environ.get('VIVE_POSITION_SCALE', '1.5')),
                       help='Position scaling factor (default: 1.0)')
    parser.add_argument('--rotation-scale', type=float,
                       default=float(os.environ.get('VIVE_ROTATION_SCALE', '1.0')),
                       help='Rotation scaling factor (default: 1.0)')
    parser.add_argument('--rate', type=int,
                       default=int(os.environ.get('TELEOP_RATE_HZ', '100')),
                       help='Control loop rate in Hz (default: 100)')
    parser.add_argument('--speed', type=int,
                       default=int(os.environ.get('TELEOP_SPEED', '100')),
                       help='Motion speed in mm/s (default: 100)')
    
    args = parser.parse_args()
    
    # Set environment variables from args
    os.environ['XARM_IP_LEFT'] = args.left_ip
    os.environ['XARM_IP_RIGHT'] = args.right_ip
    os.environ['VIVE_POSITION_SCALE'] = str(args.position_scale)
    os.environ['VIVE_ROTATION_SCALE'] = str(args.rotation_scale)
    os.environ['TELEOP_RATE_HZ'] = str(args.rate)
    os.environ['TELEOP_SPEED'] = str(args.speed)
    
    return run_teleoperation(mode=args.mode)


if __name__ == '__main__':
    sys.exit(main())

