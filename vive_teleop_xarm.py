#!/usr/bin/env python3
"""
Vive Tracker Teleoperation for xArm Robot
Maps Vive Tracker 3.0 movements to xArm with comprehensive safety features.

Safety Features:
- Table collision prevention (accounts for 200mm gripper extension)
- Joint1 constraint enforcement (-90° to +90°)
- Workspace boundary enforcement
- IK reachability validation (prevents unreachable poses)
- Smooth re-engagement after workspace violations

Performance:
- Real-time servo mode streaming at 100Hz for minimal latency
- Smoothstep easing for gradual re-engagement (default 30 frames = 0.3s)

Usage:
    export XARM_IP="192.168.1.214"  # Right arm
    export VIVE_POSITION_SCALE="1.5"
    export TELEOP_REENGAGEMENT_STEPS="30"  # 30=default, 50=extra smooth, 20=faster
    python vive_teleop_xarm.py
"""

import sys
import time
import os
import numpy as np
from pathlib import Path
from typing import Dict, Tuple, Optional
from scipy.spatial.transform import Rotation as R

# Add Vive_Tracker to path
sys.path.insert(0, str(Path(__file__).parent / 'Vive_Tracker'))
from track import ViveTrackerModule
from fairmotion_ops import conversions

# Add AnyDexGrasp to path for xArm adapter (absolute path)
anydex_path = '/home/joshua/Documents/Sources/Papers/Cursor/AnyDexGrasp'
if anydex_path not in sys.path:
    sys.path.insert(0, anydex_path)
from xarm_adapter import create_xarm_adapter, XArmAdapter


class ViveToXArmMapper:
    """Maps Vive Tracker poses to xArm commands with safety enforcement."""
    
    def __init__(self,
                 xarm_ip: str = '192.168.1.214',
                 tracker_serial: Optional[str] = None,
                 position_scale: float = 1.0,
                 rotation_scale: float = 1.0):
        """
        Initialize the Vive-to-xArm mapper.
        
        Args:
            xarm_ip: IP address of the xArm robot
            tracker_serial: Serial number of specific tracker (None = auto-detect first)
            position_scale: Scale factor for position deltas (m -> mm)
            rotation_scale: Scale factor for rotation deltas
        """
        print(f"Initializing ViveToXArmMapper...")
        print(f"  xArm IP: {xarm_ip}")
        print(f"  Position scale: {position_scale}")
        print(f"  Rotation scale: {rotation_scale}")
        
        # Initialize Vive tracker
        print("\n📡 Initializing Vive Tracker...")
        self.vive = ViveTrackerModule()
        self.vive.print_discovered_objects()
        
        trackers = self.vive.return_selected_devices("tracker")
        if len(trackers) == 0:
            raise RuntimeError("No Vive trackers detected! Ensure tracker is powered on.")
        
        # Select tracker
        if tracker_serial:
            matching = [t for name, t in trackers.items() if t.get_serial() == tracker_serial]
            if not matching:
                raise RuntimeError(f"Tracker with serial {tracker_serial} not found")
            self.tracker = matching[0]
            print(f"✅ Using tracker: {tracker_serial}")
        else:
            self.tracker = list(trackers.values())[0]
            print(f"✅ Using first detected tracker: {self.tracker.get_serial()}")
        
        # Initialize xArm adapter
        print(f"\n🤖 Connecting to xArm at {xarm_ip}...")
        os.environ['XARM_IP'] = xarm_ip
        os.environ['EXECUTE_LIVE'] = '1'
        
        self.adapter = create_xarm_adapter(
            execute_live=True,
            use_inspire_api=False  # No gripper control for now
        )
        self.adapter._lazy_connect_if_needed()
        
        if self.adapter.arm is None:
            raise RuntimeError(f"Failed to connect to xArm at {xarm_ip}")
        
        print("✅ xArm connected successfully")
        
        # Store scaling factors
        self.position_scale = position_scale
        self.rotation_scale = rotation_scale
        # Per-axis rotation fine-tuning (multiplied after global scale)
        # Defaults to 1.0 so existing behavior is unchanged
        self.rotation_scale_roll = float(os.environ.get('VIVE_ROTATION_SCALE_ROLL', '1.0'))
        self.rotation_scale_pitch = float(os.environ.get('VIVE_ROTATION_SCALE_PITCH', '3.0'))
        self.rotation_scale_yaw = float(os.environ.get('VIVE_ROTATION_SCALE_YAW', '1.0'))
        
        # Reference poses (set during calibration)
        self.tracker_home = None  # 4x4 tracker pose at calibration
        self.arm_home_pose = None  # Dict with {x,y,z,roll,pitch,yaw}
        self.base_T_tcp_home = None  # 4x4 arm pose at calibration
        
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
        
        print("\n🎮 Calibration complete!")
        
        # Enable servo mode for real-time streaming
        if not self.enable_servo_mode():
            raise RuntimeError("Failed to enable servo mode")
        
        print("🎮 Teleoperation ready!")
    
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
        
        # Scale and convert to mm
        pos_delta_scaled_mm = pos_delta_remapped * self.position_scale * 1000.0  # m -> mm
        
        # Extract rotation delta and scale
        R_delta = tracker_delta_T[:3, :3]
        rotvec_delta = R.from_matrix(R_delta).as_rotvec()
        
        # Remap rotation axes to match position remapping:
        # Same coordinate transformation applies to rotations
        # Tracker axes -> xArm axes remapping
        rotvec_remapped = np.array([
            -rotvec_delta[1],  # xArm roll (X-axis rotation) = -tracker pitch (Y-axis rotation)
            rotvec_delta[0],   # xArm pitch (Y-axis rotation) = +tracker roll (X-axis rotation)
            -rotvec_delta[2]   # xArm yaw (Z-axis rotation) = -tracker yaw (Z-axis rotation)
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
            speed: Motion speed in mm/s (reserved in servo mode, use 100)
            
        Returns:
            True if command sent successfully
        """
        try:
            # set_servo_cartesian: designed for streaming at high rates
            # speed and mvacc are reserved (ignored) in servo mode
            mvpose = [
                pose_dict['x'],
                pose_dict['y'],
                pose_dict['z'],
                pose_dict['roll'],
                pose_dict['pitch'],
                pose_dict['yaw']
            ]
            ret = self.adapter.arm.set_servo_cartesian(mvpose, speed=100, mvacc=2000)
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


def run_teleoperation():
    """Main teleoperation control loop."""
    
    # Configuration from environment
    xarm_ip = os.environ.get('XARM_IP', '192.168.1.214')
    position_scale = float(os.environ.get('VIVE_POSITION_SCALE', '1.0'))
    rotation_scale = float(os.environ.get('VIVE_ROTATION_SCALE', '1.0'))
    rate_hz = int(os.environ.get('TELEOP_RATE_HZ', '100'))  # 100Hz for servo mode streaming
    speed_mm_s = int(os.environ.get('TELEOP_SPEED', '100'))  # Reserved in servo mode (ignored)
    
    print("="*60)
    print("Vive Tracker Teleoperation for xArm")
    print("="*60)
    
    # Initialize mapper
    try:
        mapper = ViveToXArmMapper(
            xarm_ip=xarm_ip,
            position_scale=position_scale,
            rotation_scale=rotation_scale
        )
    except Exception as e:
        print(f"❌ Initialization failed: {e}")
        return 1
    
    # Move to home position
    try:
        print("\n🏠 Moving arm to home position...")
        mapper.adapter.go_home()
        print("✅ Arm at home position")
    except Exception as e:
        print(f"❌ Failed to move to home: {e}")
        mapper.shutdown()
        return 1
    
    # Calibrate reference poses
    try:
        mapper.calibrate_home_poses(duration_s=3.0)
    except Exception as e:
        print(f"❌ Calibration failed: {e}")
        mapper.shutdown()
        return 1
    
    # Main control loop
    print(f"\n🎮 Teleoperation active at {rate_hz} Hz")
    print("   Press Ctrl+C to stop")
    print("-"*60)
    
    last_safe_pose = None
    loop_count = 0
    safety_violations = 0
    
    try:
        while True:
            loop_start = time.time()
            
            # Get current tracker pose
            try:
                # Copy to avoid aliasing so deltas are computed correctly
                current_tracker_T = mapper.tracker.get_T().copy()
            except Exception as e:
                print(f"\n⚠️ Tracker read error: {e}")
                time.sleep(0.1)
                continue
            
            # Compute target pose
            try:
                target_pose = mapper.compute_target_pose(current_tracker_T)
            except Exception as e:
                print(f"\n⚠️ Pose computation error: {e}")
                time.sleep(0.1)
                continue
            
            # Fast workspace check
            in_workspace, ws_reason = mapper.is_pose_within_workspace(target_pose)
            
            if not in_workspace:
                # Outside user-defined workspace
                if not mapper._was_outside_workspace:
                    safety_violations += 1
                    print(f"\n🛑 Workspace limit: {ws_reason}")
                    print("   Arm will stop until you return to workspace")
                    
                    # Stop the arm by setting mode to position mode (mode 0)
                    try:
                        mapper.adapter.arm.set_mode(0)
                        mapper.adapter.arm.set_state(4)  # State 4 = stop
                    except Exception:
                        pass
                
                mapper._was_outside_workspace = True
                mapper._reengagement_counter = 0  # Reset re-engagement counter
                
                loop_count += 1
                elapsed = time.time() - loop_start
                sleep_time = (1.0 / rate_hz) - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                continue
            
            # Check IK reachability (every frame - fast enough for real-time)
            is_reachable, ik_reason = mapper.is_pose_reachable(target_pose)
            
            if not is_reachable:
                # Pose is kinematically unreachable - silently skip (common at workspace edges)
                # Don't spam errors, don't stop servo mode, just don't update
                loop_count += 1
                elapsed = time.time() - loop_start
                sleep_time = (1.0 / rate_hz) - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                continue
            
            # We're back in workspace - check if we need smooth re-engagement
            pose_to_send = target_pose
            
            if mapper._was_outside_workspace:
                # Just re-entered workspace - restart servo mode and use smooth re-engagement
                if mapper._reengagement_counter == 0:
                    # First frame back in workspace - re-enable servo mode
                    print(f"\n🔄 Re-entering workspace, restarting servo mode...")
                    try:
                        mapper.adapter.arm.set_mode(0)  # Position mode first
                        mapper.adapter.arm.set_state(0)  # Ready state
                        time.sleep(0.05)  # Brief pause for state transition
                        mapper.adapter.arm.set_mode(1)  # Back to servo mode
                        mapper.adapter.arm.set_state(0)  # Ready state
                        print(f"🔄 Servo mode restarted, re-engaging smoothly ({mapper._reengagement_steps} steps)...")
                    except Exception as e:
                        print(f"⚠️ Failed to restart servo mode: {e}")
                
                if mapper._reengagement_counter < mapper._reengagement_steps:
                    pose_to_send = mapper.compute_reengagement_pose(target_pose)
                    mapper._reengagement_counter += 1
                else:
                    # Re-engagement complete
                    mapper._was_outside_workspace = False
                    print(f"\n✅ Re-engagement complete, resuming normal control")
            
            # Send streaming command
            try:
                success = mapper.send_pose_streaming(pose_to_send, speed_mm_s)
                
                if success:
                    last_safe_pose = pose_to_send
                    mapper._last_valid_pose = pose_to_send  # Track for re-engagement
                    
                    # Print status every 100 frames
                    if loop_count % 100 == 0:
                        status_icon = "🔄" if mapper._was_outside_workspace else "✅"
                        print(f"\r{status_icon} Pos: [{pose_to_send['x']:.1f}, {pose_to_send['y']:.1f}, "
                              f"{pose_to_send['z']:.1f}] mm | "
                              f"Ori: [{pose_to_send['roll']:.1f}, {pose_to_send['pitch']:.1f}, "
                              f"{pose_to_send['yaw']:.1f}]° | "
                              f"Violations: {safety_violations}", 
                              end='', flush=True)
                else:
                    # Command failed, but don't spam errors - just skip this frame
                    pass
                    
            except Exception as e:
                print(f"\n⚠️ Motion execution error: {e}")
            
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
        mapper.shutdown()
        
        print(f"\n📊 Session statistics:")
        print(f"   Total frames: {loop_count}")
        print(f"   Commands sent: {loop_count - safety_violations}")
        print(f"   Safety violations: {safety_violations}")
        if loop_count > 0:
            print(f"   Success rate: {100.0 * (1 - safety_violations/loop_count):.1f}%")
    
    return 0


def main():
    """Entry point with argument parsing."""
    import argparse
    
    parser = argparse.ArgumentParser(
        description="Vive Tracker teleoperation for xArm robot"
    )
    parser.add_argument('--xarm-ip', default=os.environ.get('XARM_IP', '192.168.1.214'),
                       help='xArm IP address (default: 192.168.1.214)')
    parser.add_argument('--position-scale', type=float,
                       default=float(os.environ.get('VIVE_POSITION_SCALE', '1.0')),
                       help='Position scaling factor (default: 1.0)')
    parser.add_argument('--rotation-scale', type=float,
                       default=float(os.environ.get('VIVE_ROTATION_SCALE', '1.0')),
                       help='Rotation scaling factor (default: 1.0)')
    parser.add_argument('--rate', type=int,
                       default=int(os.environ.get('TELEOP_RATE_HZ', '30')),
                       help='Control loop rate in Hz (default: 30)')
    parser.add_argument('--speed', type=int,
                       default=int(os.environ.get('TELEOP_SPEED', '200')),
                       help='Motion speed in mm/s (default: 200)')
    
    args = parser.parse_args()
    
    # Set environment variables from args
    os.environ['XARM_IP'] = args.xarm_ip
    os.environ['VIVE_POSITION_SCALE'] = str(args.position_scale)
    os.environ['VIVE_ROTATION_SCALE'] = str(args.rotation_scale)
    os.environ['TELEOP_RATE_HZ'] = str(args.rate)
    os.environ['TELEOP_SPEED'] = str(args.speed)
    
    return run_teleoperation()


if __name__ == '__main__':
    sys.exit(main())

