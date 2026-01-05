"""
xArm Adapter for Teleoperation
Provides safety features, collision detection, and xArm control for VR teleoperation.

This is a simplified standalone adapter extracted for the dex-teleop system.
"""

import numpy as np
from scipy.spatial.transform import Rotation as R
import time
from typing import Optional, Tuple
import os


# ============================================================================
# Configuration (all configurable via environment variables)
# ============================================================================

# Safety flag - set to False to only print commands without moving the arm
EXECUTE_LIVE = os.environ.get('EXECUTE_LIVE', '0').strip().lower() in ('1', 'true', 'yes', 'y')

# xArm connection settings
XARM_IP = os.environ.get('XARM_IP', os.environ.get('XARM_IP_RIGHT', "192.168.1.214"))

# Table collision detection parameters
TABLE_Z_MM = int(os.environ.get('TABLE_Z_MM', '15'))  # Z-height of table surface (mm)
GRIPPER_TIP_LENGTH_MM = int(os.environ.get('GRIPPER_TIP_LENGTH_MM', '200'))  # Inspire gripper length
GRIPPER_RADIUS_MM = int(os.environ.get('GRIPPER_RADIUS_MM', '50'))  # Palm radius
GRIPPER_TIP_RADIUS_MM = int(os.environ.get('GRIPPER_TIP_RADIUS_MM', '18'))  # Fingertip radius
NUM_COLLISION_SAMPLES = int(os.environ.get('NUM_COLLISION_SAMPLES', '12'))  # Samples along gripper
TABLE_CLEARANCE_MM = int(os.environ.get('TABLE_CLEARANCE_MM', '5'))  # Safety margin

# Gripper orientation (for Inspire: fingertips point along +x)
GRIPPER_TIP_AXIS = os.environ.get('GRIPPER_TIP_AXIS', 'x').strip().lower()
try:
    GRIPPER_TIP_AXIS_SIGN = int(os.environ.get('GRIPPER_TIP_AXIS_SIGN', '1'))
    if GRIPPER_TIP_AXIS_SIGN not in (-1, 1):
        GRIPPER_TIP_AXIS_SIGN = 1
except Exception:
    GRIPPER_TIP_AXIS_SIGN = 1

# Extra margin for tilted grippers
EXTRA_TIP_TILT_MARGIN_MM = float(os.environ.get('EXTRA_TIP_TILT_MARGIN_MM', '12'))

# Thumb collision detection
THUMB_LENGTH_MM = float(os.environ.get('THUMB_LENGTH_MM', '80'))
THUMB_RADIUS_MM = float(os.environ.get('THUMB_RADIUS_MM', '20'))
THUMB_NUM_SAMPLES = int(os.environ.get('THUMB_NUM_SAMPLES', '6'))
THUMB_LATERAL_OFFSET_MM = float(os.environ.get('THUMB_LATERAL_OFFSET_MM', '35'))

# Debug flags
PRINT_COLLISION_DEBUG = os.environ.get('PRINT_COLLISION_DEBUG', '0').strip().lower() in ('1', 'true', 'yes', 'y')

# TCP to Gripper transform (Inspire hand mounted on xArm)
TCP_T_GRIPPER = np.array([
    [0, 0, -1, 0],
    [0, 1,  0, 0],
    [1, 0,  0, 0],
    [0, 0,  0, 1]
], dtype=np.float32)

# Home pose (mm, degrees)
HOME_POSE = {
    'x': int(os.environ.get('XARM_HOME_X_MM', '275')),
    'y': int(os.environ.get('XARM_HOME_Y_MM', '0')),
    'z': int(os.environ.get('XARM_HOME_Z_MM', '670')),
    'roll': float(os.environ.get('XARM_HOME_ROLL_DEG', '180')),
    'pitch': float(os.environ.get('XARM_HOME_PITCH_DEG', '-1')),
    'yaw': float(os.environ.get('XARM_HOME_YAW_DEG', '0'))
}

# Joint angles for home position (prevents IK configuration flips)
HOME_JOINTS = [-23.8, -32.1, -66.6, 2.3, 97.9, -22.7]


class XArmAdapter:
    """Adapter for xArm control with safety features for teleoperation."""
    
    def __init__(self, ip: str = XARM_IP, execute_live: bool = EXECUTE_LIVE,
                 use_inspire_api: bool = False):
        """
        Initialize xArm adapter.
        
        Args:
            ip: xArm IP address
            execute_live: If True, actually move the arm. If False, only print commands.
            use_inspire_api: Ignored (kept for compatibility)
        """
        self.ip = ip
        self.execute_live = execute_live
        self.arm = None
        self._deferred_connect = False
        
        if self.execute_live:
            self.arm = None
            self._deferred_connect = True
            print(f"🔌 xArm connection will be established at first motion (IP: {ip})")
        else:
            print("🔒 SAFETY MODE: Commands will be printed but not executed")
    
    def _lazy_connect_if_needed(self):
        """Connect to xArm just-in-time before first motion."""
        if not self.execute_live:
            return
        
        if self.arm is not None:
            # Check connectivity
            try:
                if not self.arm.connected:
                    print("⚠️ xArm disconnected; attempting reconnect...")
                    self.arm.connect()
                    self._connect_arm()
            except Exception:
                self.arm = None
        
        if self.arm is not None:
            return
        
        try:
            from xarm.wrapper import XArmAPI
            disable_report = os.environ.get('XARM_DISABLE_REPORT', '1').strip().lower() in ('1', 'true', 'yes', 'y')
            try:
                self.arm = XArmAPI(self.ip, do_not_open_report=disable_report)
            except TypeError:
                self.arm = XArmAPI(self.ip)
            self._connect_arm()
            print(f"✅ Connected to xArm at {self.ip}")
        except ImportError:
            print("❌ xArm Python SDK not installed.")
            print("   Install from: https://github.com/xArm-Developer/xArm-Python-SDK")
            print("   pip install git+https://github.com/xArm-Developer/xArm-Python-SDK.git")
            self.execute_live = False
    
    def _connect_arm(self):
        """Initialize xArm connection settings."""
        if self.arm is None:
            return
        
        try:
            try:
                self.arm.set_timeout(10)
            except Exception:
                pass
            self._ensure_ready()
        except Exception:
            pass
        
        # Set TCP offset
        try:
            self.arm.set_tcp_offset([0, 0, 0, 0, 0, 0])
        except Exception:
            pass
        
        # Set speed/acceleration limits
        try:
            tcp_speed_limit = int(os.environ.get('XARM_TCP_SPEED_LIMIT', '200'))
            tcp_acc_limit = int(os.environ.get('XARM_TCP_ACC_LIMIT', '2000'))
            self.arm.set_tcp_maxacc(tcp_acc_limit)
        except Exception:
            pass
    
    def _get_live_tcp_pose(self) -> Optional[np.ndarray]:
        """Query the arm for current TCP pose (mm, deg).
        
        Returns:
            Array [x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg] or None
        """
        if not (self.arm and self.execute_live):
            return None
        try:
            code, data = self.arm.get_position(is_radian=False)
            if code == 0 and isinstance(data, (list, tuple)) and len(data) >= 6:
                return np.array(data[:6], dtype=float)
        except Exception:
            pass
        return None
    
    @staticmethod
    def _explain_api_code(code: int) -> str:
        """Explain xArm API return codes."""
        mapping = {
            -9: "emergency stop",
            -8: "out of range",
            -7: "joint angle limit",
            -6: "cartesian pos limit",
            -2: "xArm not ready",
            -1: "xArm disconnected",
             0: "success",
             1: "uncleared errors",
             2: "uncleared warnings",
             3: "get response timeout",
             9: "state not ready",
        }
        return mapping.get(int(code), "unknown")
    
    def _ensure_ready(self) -> bool:
        """Clear errors and ensure motion is enabled.
        
        Returns:
            True if controller is ready
        """
        if not self.arm:
            return False
        
        try:
            self.arm.clean_error()
            self.arm.clean_warn()
        except Exception:
            pass
        
        try:
            self.arm.motion_enable(True)
            self.arm.set_mode(0)
            self.arm.set_state(0)
            
            # Set collision sensitivity
            coll_sens = int(os.environ.get('XARM_COLLISION_SENSITIVITY', '4'))
            if coll_sens > 0:
                self.arm.set_collision_sensitivity(coll_sens)
        except Exception:
            pass
        
        try:
            code, ew = self.arm.get_err_warn_code()
            if code == 0 and isinstance(ew, (list, tuple)) and len(ew) >= 2:
                err = int(ew[0])
                if err == 0:
                    return True
        except Exception:
            pass
        
        return False
    
    def _wait_until_ready(self, timeout_s: float = 5.0) -> bool:
        """Wait until controller is ready for motion commands.
        
        Args:
            timeout_s: Maximum time to wait
            
        Returns:
            True if ready within timeout
        """
        if not self.arm:
            return False
        
        t_end = time.time() + timeout_s
        while time.time() < t_end:
            try:
                code, state = self.arm.get_state()
                if code == 0 and state in (0, 1, 2):
                    code2, ew = self.arm.get_err_warn_code()
                    if code2 == 0 and isinstance(ew, (list, tuple)) and len(ew) >= 2:
                        if int(ew[0]) == 0:
                            return True
            except Exception:
                pass
            
            # Attempt recovery
            try:
                self.arm.clean_error()
                self.arm.clean_warn()
                self.arm.motion_enable(True)
                self.arm.set_mode(0)
                self.arm.set_state(0)
            except Exception:
                pass
            
            time.sleep(0.1)
        
        return False
    
    def pose_collides_with_table(self, base_T_tcp: np.ndarray,
                                  tip_length_mm: Optional[float] = None,
                                  table_z_mm: Optional[float] = None,
                                  clearance_mm: Optional[float] = None,
                                  gripper_radius_mm: Optional[float] = None,
                                  num_samples: Optional[int] = None,
                                  gripper_tip_radius_mm: Optional[float] = None) -> bool:
        """Check if gripper would collide with table.
        
        Models the gripper as a tapered cylinder and checks multiple points
        along its length for table collision.
        
        Args:
            base_T_tcp: 4x4 transformation matrix (base frame to TCP)
            tip_length_mm: Length of gripper cylinder
            table_z_mm: Z-height of table surface
            clearance_mm: Required clearance above table
            gripper_radius_mm: Radius at gripper base
            gripper_tip_radius_mm: Radius at fingertips
            num_samples: Number of points to sample
        
        Returns:
            True if collision detected, False if safe
        """
        try:
            L = float(GRIPPER_TIP_LENGTH_MM if tip_length_mm is None else tip_length_mm)
            z_tbl = float(TABLE_Z_MM if table_z_mm is None else table_z_mm)
            z_clear = float(TABLE_CLEARANCE_MM if clearance_mm is None else clearance_mm)
            radius_base = float(GRIPPER_RADIUS_MM if gripper_radius_mm is None else gripper_radius_mm)
            radius_tip = float(GRIPPER_TIP_RADIUS_MM if gripper_tip_radius_mm is None else gripper_tip_radius_mm)
            n_samples = int(NUM_COLLISION_SAMPLES if num_samples is None else num_samples)
            
            # Get gripper direction in base frame
            base_R_tcp = base_T_tcp[:3, :3]
            tcp_R_gripper = TCP_T_GRIPPER[:3, :3]
            axis_map = {'x': 0, 'y': 1, 'z': 2}
            ax = axis_map.get(GRIPPER_TIP_AXIS, 2)
            dir_in_tcp = tcp_R_gripper[:, ax] * float(GRIPPER_TIP_AXIS_SIGN)
            dir_in_base = base_R_tcp @ dir_in_tcp
            p_tcp_b = base_T_tcp[:3, 3]
            
            thresh_m = (z_tbl + z_clear) / 1000.0
            
            # Check main fingers
            for i in range(n_samples):
                alpha_i = i / max(1, n_samples - 1)
                t = (L / 1000.0) * alpha_i
                p_center = p_tcp_b + t * dir_in_base
                
                # Tapered radius
                radius_at_sample = radius_base * (1.0 - alpha_i) + radius_tip * alpha_i
                
                # Vertical projection
                n_z = float(dir_in_base[2])
                vertical_proj = float(np.sqrt(max(0.0, 1.0 - n_z * n_z)))
                p_lowest_z = p_center[2] - (radius_at_sample / 1000.0) * vertical_proj
                
                # Tilt margin
                tilt_down = max(0.0, -n_z)
                p_lowest_z -= (EXTRA_TIP_TILT_MARGIN_MM / 1000.0) * tilt_down
                
                if p_lowest_z < thresh_m:
                    if PRINT_COLLISION_DEBUG:
                        print(f"[collision] Finger collision at sample {i}/{n_samples-1}")
                    return True
            
            # Check thumb
            thumb_pitch_deg = 30.0
            lateral_axis_tcp = tcp_R_gripper[:, 1]
            lateral_dir_base = base_R_tcp @ lateral_axis_tcp
            
            thumb_rot = R.from_rotvec(lateral_dir_base * np.deg2rad(thumb_pitch_deg))
            thumb_dir_in_base = thumb_rot.apply(dir_in_base)
            thumb_length_m = THUMB_LENGTH_MM / 1000.0
            thumb_offset_m = THUMB_LATERAL_OFFSET_MM / 1000.0
            
            for i in range(THUMB_NUM_SAMPLES):
                alpha_thumb = i / max(1, THUMB_NUM_SAMPLES - 1)
                t_thumb = thumb_length_m * alpha_thumb
                
                p_thumb_center = p_tcp_b + thumb_offset_m * lateral_dir_base + t_thumb * thumb_dir_in_base
                thumb_radius = THUMB_RADIUS_MM
                
                n_z_thumb = float(thumb_dir_in_base[2])
                vertical_proj_thumb = float(np.sqrt(max(0.0, 1.0 - n_z_thumb * n_z_thumb)))
                p_thumb_lowest_z = p_thumb_center[2] - (thumb_radius / 1000.0) * vertical_proj_thumb
                
                tilt_down_thumb = max(0.0, -n_z_thumb)
                p_thumb_lowest_z -= (EXTRA_TIP_TILT_MARGIN_MM / 1000.0) * tilt_down_thumb
                
                if p_thumb_lowest_z < thresh_m:
                    if PRINT_COLLISION_DEBUG:
                        print(f"[collision] Thumb collision at sample {i}/{THUMB_NUM_SAMPLES-1}")
                    return True
            
            return False
            
        except Exception as e:
            if PRINT_COLLISION_DEBUG:
                print(f"[collision] Error in collision check: {e}")
            return False  # Fail-safe: allow motion if check fails
    
    def go_home(self, speed: int = 60):
        """Move xArm to home position.
        
        Uses lift-then-joint strategy:
        1. Lifts UP using Cartesian motion
        2. Moves to home using joint angles
        
        Args:
            speed: Motion speed in deg/s
        """
        SAFE_LIFT_Z = 300  # mm
        
        if self.execute_live and self.arm:
            try:
                # Get current position
                code, current_pos = self.arm.get_position()
                
                if code == 0 and current_pos is not None:
                    current_z = float(current_pos[2])
                    
                    # Lift if below safe height
                    if current_z < SAFE_LIFT_Z:
                        lift_pose = list(current_pos[:6])
                        lift_pose[2] = SAFE_LIFT_Z
                        print(f"   🔼 Lifting to safe height (Z={SAFE_LIFT_Z}mm)...")
                        self.arm.set_position(*lift_pose, speed=speed, wait=True)
                
                # Move to home using joint angles
                print(f"   🏠 Moving to home position...")
                ret = self.arm.set_servo_angle(angle=HOME_JOINTS, speed=speed, wait=True, is_radian=False)
                if ret != 0:
                    print(f"⚠️ Home move failed with code {ret}")
                else:
                    print("✅ Arm at home position")
                    
            except Exception as e:
                print(f"❌ go_home error: {e}")
        else:
            print(f"[SAFETY MODE] Would move to home: {HOME_POSE}")
    
    def disconnect(self):
        """Disconnect from xArm."""
        if self.arm and self.execute_live:
            try:
                self.arm.disconnect()
                print("🔌 Disconnected from xArm")
            except Exception:
                pass


def create_xarm_adapter(execute_live: bool = None, *, use_inspire_api: bool = False,
                        hand_port: str = '/dev/ttyUSB0', hand_slave_id: int = 1,
                        hand_debug: bool = False) -> XArmAdapter:
    """Factory function to create xArm adapter.
    
    Args:
        execute_live: If True, connect and move arm. If None, uses EXECUTE_LIVE env var.
        use_inspire_api: Ignored (kept for compatibility)
        hand_port: Ignored (kept for compatibility)
        hand_slave_id: Ignored (kept for compatibility)
        hand_debug: Ignored (kept for compatibility)
    
    Returns:
        XArmAdapter instance
    """
    if execute_live is None:
        execute_live = EXECUTE_LIVE
    return XArmAdapter(execute_live=execute_live)


if __name__ == "__main__":
    print("🤖 xArm Adapter for Teleoperation")
    print("=" * 50)
    
    # Test in safety mode
    adapter = create_xarm_adapter(execute_live=False)
    adapter.go_home()
    
    print("\n✅ Adapter test complete")
